// FMeshingStructs.h - Data structures for dual-contour mesh generation: per-corner
// sample batching, mesh vertex/edge data, RealtimeMesh stream wrappers, and chunk
// component management.

#pragma once

#include "FAdaptiveOctreeNode.h"
#include "CoreGlobals.h"
#include "Misc/ScopeLock.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>
#include <atomic>

// Required by RealtimeMesh stream builder types. Narrower scoping is not feasible
// due to the template-heavy API surface used throughout this file.
using namespace RealtimeMesh;

/** A single corner sample for batched density evaluation.
 *  Position is the world-space location to sample. Density receives the computed
 *  SDF value. Targets holds pointers to all FNodeCorner::Density fields that share
 *  this position, allowing a single sample to update multiple nodes at once
 *  during GatherUniqueCorners. */
struct VOXELPLUGIN_API FCornerSample {
    FVector Position;
    double Density;
    TArray<float*> Targets;
};

/** A mesh vertex produced by dual contouring. Position is chunk-local (relative to
 *  FMeshChunk::ChunkCenter). Equality is exact on position since vertices sharing
 *  a dual contour point have identical double-precision values. */
struct VOXELPLUGIN_API FMeshVertex
{
    FVector Position;
    FVector3f Normal;
    FColor Color;

    bool operator==(const FMeshVertex& Other) const
    {
        return Position == Other.Position;
    }
};

/** Per-edge output from the parallel meshing pass. Stores the vertices (one per
 *  neighboring leaf node) that will form a quad or tri across a sign-change edge,
 *  along with the edge itself. IsValid is false if the edge produces no faces in this
 *  chunk. BoundaryHash summarizes the leaves outside the chunk that the edge's
 *  faces depend on (0 if none); it feeds FMeshChunk::SeamHash. */
struct VOXELPLUGIN_API FEdgeVertexData {
    TArray<FMeshVertex> Vertices;
    TOptional<FNodeEdge> Edge;
    uint32 BoundaryHash = 0;
    bool IsValid = false;
};

/** Wrapper around a RealtimeMesh FRealtimeMeshStreamSet, pre-configured with
 *  the six vertex streams needed for rendering: position, tangents, triangles,
 *  polygroups, texcoords, and vertex color. Provides typed stream builder
 *  accessors for each.
 *
 *  Generation and PassId identify the publish that produced this stream: Generation
 *  counts publishes on its chunk, PassId is the octree pass (FAdaptiveOctree::BeginPass)
 *  that built it. Both are set when the stream is published and never change after. */
struct VOXELPLUGIN_API FMeshStreamData {
    FRealtimeMeshBufferSetKey MeshGroupKey;
    FRealtimeMeshSectionKey MeshSectionKey;
    FRealtimeMeshStreamSet MeshStream;
    uint32 Generation = 0;
    uint32 PassId = 0;

    FMeshStreamData() {
        MeshStream.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
        MeshStream.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>());
        MeshStream.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>());
        MeshStream.AddStream(FRealtimeMeshStreams::PolyGroups, GetRealtimeMeshBufferLayout<uint16>());
        MeshStream.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2DHalf>());
        MeshStream.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>());
    }

    TRealtimeMeshStreamBuilder<FVector, FVector3f> GetPositionStream() {
        return TRealtimeMeshStreamBuilder<FVector, FVector3f>(*MeshStream.Find((FRealtimeMeshStreams::Position)));
    }
    TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision> GetTangentStream() {
        return TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision>(*MeshStream.Find(FRealtimeMeshStreams::Tangents));
    }
    TRealtimeMeshStreamBuilder<TIndex3<uint32>> GetTriangleStream() {
        return TRealtimeMeshStreamBuilder<TIndex3<uint32>>(*MeshStream.Find(FRealtimeMeshStreams::Triangles));
    }
    TRealtimeMeshStreamBuilder<uint32, uint16> GetPolygroupStream() {
        return TRealtimeMeshStreamBuilder<uint32, uint16>(*MeshStream.Find(FRealtimeMeshStreams::PolyGroups));
    }
    TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf> GetTexCoordStream() {
        return TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf>(*MeshStream.Find(FRealtimeMeshStreams::TexCoords));
    }
    TRealtimeMeshStreamBuilder<FColor> GetColorStream() {
        return TRealtimeMeshStreamBuilder<FColor>(*MeshStream.Find(FRealtimeMeshStreams::Color));
    }

    int32 NumTriangles() {
        auto* Triangles = MeshStream.Find(FRealtimeMeshStreams::Triangles);
        return Triangles ? Triangles->Num() : 0;
    }
};

/** A spatial subdivision of the octree used for mesh generation. Each chunk corresponds
 *  to a chunk-root node and owns a RealtimeMesh component that renders the dual-contour
 *  surface within its bounds.
 *
 *  Lifecycle: InitializeData sets the spatial bounds and stream keys. A worker builds a
 *  stream and calls Publish. The game thread creates the component (EnsureComponent,
 *  budgeted) and later swaps the stream in (ApplyStream).
 *
 *  Dirty state is two counters: PublishedGen (bumped by Publish) and AppliedGen (the
 *  generation the game thread last applied or settled). The chunk is dirty while they
 *  differ, so a stream published while an older one is being applied stays dirty and is
 *  applied on a later frame. */
struct VOXELPLUGIN_API FMeshChunk : public TSharedFromThis<FMeshChunk> {
    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;
    TWeakObjectPtr<USceneComponent> CachedMeshAttachRoot;
    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;

    FVector ChunkCenter;
    double ChunkExtent;

    /** All sign-change edges within this chunk's leaf nodes, collected during LOD updates. */
    TArray<FNodeEdge> ChunkEdges;

    /** Sum of the BoundaryHash of every edge in ChunkEdges at the last mesh: a summary of
     *  the leaves outside this chunk that its faces were built against. Written by the
     *  mesher and read by the seam check, both under the octree write lock. */
    uint32 SeamHash = 0;

    /** Triangles of the last mesh that face against their vertex normals (folds), counted
     *  only with flicker diagnostics on. Same lock rules as SeamHash. */
    int32 DebugFoldedTriangles = 0;

    /** Latest published stream. Guarded by MeshDataCS for the pointer swap and copy only;
     *  a published stream is never modified (each re-mesh publishes a fresh one). */
    TSharedPtr<FMeshStreamData> SurfaceMeshData;
    FCriticalSection MeshDataCS;

    std::atomic<uint32> PublishedGen{ 0 };
    std::atomic<uint32> AppliedGen{ 0 };

    /** Pass of the oldest publish not yet applied: set by a publish that finds the chunk
     *  clean, left alone by later ones. The drain groups chunks by the passes between this
     *  and their latest stream's pass. */
    std::atomic<uint32> FirstUnappliedPass{ 0 };

    /** GFrameCounter at the latest publish and at the latest apply (debug statistics). */
    std::atomic<uint64> PublishFrame{ 0 };
    uint64 AppliedFrame = 0;

    /** What the render proxy has absorbed, from the update futures: the generation (high 32
     *  bits) and the first frame it can be on screen (low 32 bits, GFrameCounter truncated).
     *  Updated as one value, only ever to a newer generation; 0 = nothing displayed. A
     *  component has no scene proxy until the end-of-frame update after it is created, so
     *  its first data counts as displayed two frames after creation (or after a "NoProxy"
     *  apply). */
    std::atomic<uint64> DisplayState{ 0 };

    /** Bumped whenever the component is created or released, so a late future from an
     *  earlier component can't mark the current one displayed. */
    std::atomic<uint32> ComponentEpoch{ 0 };

    /** GFrameCounter when the current component was created (game thread). */
    uint64 ComponentCreatedFrame = 0;

    /** Frames a retired chunk has waited in the destroy queue (game thread only). */
    int32 RetireWaitFrames = 0;

    bool IsInitialized = false;

    /** True while this chunk is sitting in AAdaptiveVoxelActor::PendingApply, so the mesh
     *  pass doesn't enqueue it twice before Tick drains it. Only touched under
     *  PendingApplyCS. */
    bool bApplyQueued = false;

    /** Set when this chunk is retired by the chunk-cut pass (e.g. promoted into finer
     *  children). A retired chunk must never (re)create its component: the apply drain
     *  skips it and the destroy drain releases its component. */
    bool bRetired = false;

    /** When retired, the chunk(s) that now cover this chunk's space (finer children on
     *  promote, the coarser parent on demote). The destroy drain keeps this chunk's
     *  component alive -- still rendering its last mesh -- until every entry here has
     *  applied (or is itself retired with its own replacements settled), so a swap never
     *  opens a hole. Cleared when the chunk is finally released. */
    TArray<TSharedPtr<FMeshChunk>> ReplacedBy;

    /** True once a non-empty UpdateBufferSet has actually created the poly-group section.
     *  Empty chunks (no triangles) never create one, so touching its config would throw
     *  "Failed to find Section". */
    bool bHasSection = false;

    TWeakObjectPtr<URealtimeMeshSimple> ChunkRtMesh;
    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;
    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey(0);

    bool IsDirty() const {
        return PublishedGen.load(std::memory_order_acquire) != AppliedGen.load(std::memory_order_acquire);
    }

    /** True once some generation of this chunk is on screen (game thread). */
    bool IsDisplayed() const {
        const uint64 State = DisplayState.load(std::memory_order_acquire);
        if ((State >> 32) == 0) return false;
        return (uint32)GFrameCounter >= (uint32)(State & 0xFFFFFFFFull);
    }

    /** Any thread: record that Gen of component Epoch is in the render proxy and can be on
     *  screen from NotBeforeFrame. Ignored for an older component or an older generation. */
    void MarkDisplayed(uint32 Epoch, uint32 Gen, uint64 NotBeforeFrame) {
        if (ComponentEpoch.load(std::memory_order_acquire) != Epoch) return;
        const uint64 NewState = ((uint64)Gen << 32) | (uint64)(uint32)NotBeforeFrame;
        uint64 Old = DisplayState.load(std::memory_order_relaxed);
        while ((Old >> 32) < Gen)
        {
            if (DisplayState.compare_exchange_weak(Old, NewState, std::memory_order_release, std::memory_order_relaxed))
                break;
        }
    }

    /** Game thread: the component changed identity (created or released). */
    void ResetDisplayState() {
        ComponentEpoch.fetch_add(1, std::memory_order_acq_rel);
        DisplayState.store(0, std::memory_order_release);
    }

    /** Copy of the latest published stream pointer (keeps that stream alive). */
    TSharedPtr<FMeshStreamData> GetPublished() {
        FScopeLock Lock(&MeshDataCS);
        return SurfaceMeshData;
    }

    /** Pass that produced the latest published stream (0 before the first publish). */
    uint32 GetPublishedPass() {
        FScopeLock Lock(&MeshDataCS);
        return SurfaceMeshData ? SurfaceMeshData->PassId : 0;
    }

    /** A fresh, empty stream carrying this chunk's section keys. */
    TSharedPtr<FMeshStreamData> MakeStream() {
        TSharedPtr<FMeshStreamData> NewData = MakeShared<FMeshStreamData>();
        FScopeLock Lock(&MeshDataCS);
        NewData->MeshGroupKey = SurfaceMeshData->MeshGroupKey;
        NewData->MeshSectionKey = SurfaceMeshData->MeshSectionKey;
        return NewData;
    }

    /** Publishes a finished stream as the chunk's latest and marks the chunk dirty. Any
     *  thread; the stream must not be modified afterwards. */
    void Publish(const TSharedPtr<FMeshStreamData>& NewData, uint32 InPassId) {
        FScopeLock Lock(&MeshDataCS);
        if (!IsDirty())
            FirstUnappliedPass.store(InPassId, std::memory_order_relaxed);
        NewData->PassId = InPassId;
        NewData->Generation = PublishedGen.load(std::memory_order_relaxed) + 1;
        SurfaceMeshData = NewData;
        PublishFrame.store(GFrameCounter, std::memory_order_relaxed);
        PublishedGen.store(NewData->Generation, std::memory_order_release);
    }

    /** Game thread: Gen is now what the chunk shows (or, for a chunk with no component and
     *  an empty stream, all it needs to show). If a newer stream was published meanwhile,
     *  the oldest unapplied work is now that stream's pass. */
    void MarkApplied(uint32 Gen) {
        FScopeLock Lock(&MeshDataCS);
        AppliedGen.store(Gen, std::memory_order_release);
        AppliedFrame = GFrameCounter;
        if (PublishedGen.load(std::memory_order_relaxed) != Gen && SurfaceMeshData)
            FirstUnappliedPass.store(SurfaceMeshData->PassId, std::memory_order_relaxed);
    }

    /** Sets spatial bounds and creates an empty generation-0 stream holding the section
     *  group/section keys. */
    void InitializeData(FVector InCenter, double InExtent) {
        SurfaceMeshData = MakeShared<FMeshStreamData>();

        ChunkCenter = InCenter;
        ChunkExtent = InExtent;

        SurfaceMeshData->MeshGroupKey = FRealtimeMeshBufferSetKey::Create(LODKey, FName("SURFACE"));
        SurfaceMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(SurfaceMeshData->MeshGroupKey, 0);
    };

    /** Creates the RealtimeMesh UObject and component and attaches it to the parent actor.
     *  Render only: terrain collision comes from FVoxelCollisionBricks. The section group
     *  starts empty, so the new component draws nothing until ApplyStream swaps a stream
     *  in. Game thread. */
    void InitializeComponent(ARealtimeMeshActor* InParentActor, USceneComponent* InAttachRoot, UMaterialInterface* InSurfaceMaterial, const FRealtimeMeshBufferSetKey& InGroupKey) {
        ChunkRtMesh = NewObject<URealtimeMeshSimple>(InParentActor);
        ChunkRtMesh->SetupMaterialSlot(0, "Surface Material");

        ChunkRtComponent = NewObject<URealtimeMeshComponent>(InParentActor, URealtimeMeshComponent::StaticClass());
        // Real geometry: rendered by the main camera only, excluded from the virtual
        // backdrop SceneCapture. Set before RegisterComponent so it's baked into the
        // scene proxy at creation.
        ChunkRtComponent->bHiddenInSceneCapture = true;
        ChunkRtComponent->RegisterComponent();

        ChunkRtComponent->SetMaterial(0, InSurfaceMaterial);

        ChunkRtComponent->AttachToComponent(InAttachRoot, FAttachmentTransformRules::KeepRelativeTransform);
        ChunkRtComponent->SetRelativeLocation(ChunkCenter);
        ChunkRtComponent->SetMobility(EComponentMobility::Movable);
        ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
        ChunkRtComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh.Get());
        ChunkRtComponent->SetRenderCustomDepth(true);
        ChunkRtComponent->SetVisibleInRayTracing(false);

        ChunkRtMesh->CreateBufferSet(InGroupKey, FRealtimeMeshStreamSet());

        ResetDisplayState();
        ComponentCreatedFrame = GFrameCounter;
        IsInitialized = true;
    }

    /** Filters edges that should not produce mesh faces in this chunk.
     *  Rejects edges with fewer than 3 neighbors (incomplete face on chunk boundary),
     *  and edges where any neighbor node is smaller than the edge's owner node.
     *  The size check prevents T-junctions: when a coarse node borders finer neighbors,
     *  the finer nodes will handle meshing at their own resolution instead.
     *  The 0.9 multiplier is a floating-point tolerance on the extent comparison. */
    bool ShouldProcessEdge(const FNodeEdge& Edge, const FEdgeNeighbors& Neighbors) {
        if (Neighbors.Count < 3) return false;

        double OwnerExtent = Edge.Size * 0.5;
        for (int32 i = 0; i < Neighbors.Count; i++)
        {
            if (Neighbors.Nodes[i]->Extent < OwnerExtent * 0.9)
                return false;
        }
        return true;
    }

    /** True if applying Data needs a component that doesn't exist yet (an empty stream on
     *  a chunk without a component needs none). */
    bool NeedsComponentFor(const TSharedPtr<FMeshStreamData>& Data) {
        if (IsInitialized || bRetired) return false;
        return Data.IsValid() && Data->NumTriangles() > 0;
    }

    /** Game thread: creates the component if it doesn't exist. This is the expensive part
     *  of a first apply (RegisterComponent + section group), so the drain budgets it
     *  separately. Returns false if the owning actor or attach root is gone. */
    bool EnsureComponent() {
        if (IsInitialized) return true;
        if (bRetired) return false;
        ARealtimeMeshActor* Parent = CachedParentActor.Get();
        USceneComponent* AttachRoot = CachedMeshAttachRoot.Get();
        if (!Parent || !AttachRoot) return false;

        FRealtimeMeshBufferSetKey GroupKey;
        {
            FScopeLock Lock(&MeshDataCS);
            if (!SurfaceMeshData) return false;
            GroupKey = SurfaceMeshData->MeshGroupKey;
        }
        InitializeComponent(Parent, AttachRoot, CachedSurfaceMaterial.Get(), GroupKey);
        return true;
    }

    /** Game thread: swaps Data (a stream this chunk published) into the component, creating
     *  the component first if needed, and marks that generation applied. A stream published
     *  after Data leaves the chunk dirty for a later apply. Clears the section group when
     *  Data has no triangles. */
    void ApplyStream(const TSharedPtr<FMeshStreamData>& Data) {
        // Retired by the chunk-cut pass -- never (re)create a component for it; the
        // destroy drain will release any component it already had.
        if (bRetired || !Data) return;
        if (Data->Generation <= AppliedGen.load(std::memory_order_acquire)) return;

        const int32 SrfNumTris = Data->NumTriangles();
        if (!IsInitialized)
        {
            // Nothing to draw and nothing on screen to clear.
            if (SrfNumTris <= 0) { MarkApplied(Data->Generation); return; }
            if (!EnsureComponent()) return;
        }

        URealtimeMeshSimple* MeshPtr = ChunkRtMesh.Get();
        URealtimeMeshComponent* CompPtr = ChunkRtComponent.Get();
        if (!MeshPtr || !CompPtr)
        {
            // The component went away under us; the next drain recreates it.
            IsInitialized = false;
            bHasSection = false;
            ResetDisplayState();
            return;
        }

        TFuture<ERealtimeMeshProxyUpdateStatus> Update;
        if (SrfNumTris <= 0)
        {
            Update = MeshPtr->UpdateBufferSet(Data->MeshGroupKey, FRealtimeMeshStreamSet());
            bHasSection = false; // empty stream clears the poly-group section
        }
        else
        {
            Update = MeshPtr->UpdateBufferSet(Data->MeshGroupKey, Data->MeshStream);
            // A new section gets its config (material slot 0, no collision); a re-upload
            // keeps it.
            if (!bHasSection)
                MeshPtr->UpdateSectionConfig(Data->MeshSectionKey, FRealtimeMeshSectionConfig(0), false);
            bHasSection = true;
        }

        MarkApplied(Data->Generation);

        // Record when the render proxy has this generation. The continuation may run on any
        // thread (inline if the future is already complete), so it only touches atomics.
        const TWeakPtr<FMeshChunk> WeakSelf = AsShared();
        const uint32 Gen = Data->Generation;
        const uint32 Epoch = ComponentEpoch.load(std::memory_order_acquire);
        const uint64 ApplyFrame = GFrameCounter;
        const uint64 CreatedFrame = ComponentCreatedFrame;
        Update.Next([WeakSelf, Gen, Epoch, ApplyFrame, CreatedFrame](ERealtimeMeshProxyUpdateStatus Status)
            {
                const TSharedPtr<FMeshChunk> Self = WeakSelf.Pin();
                if (!Self) return;
                uint64 NotBefore = CreatedFrame + 2;
                if (Status == ERealtimeMeshProxyUpdateStatus::NoProxy)
                    NotBefore = FMath::Max(NotBefore, ApplyFrame + 2);
                Self->MarkDisplayed(Epoch, Gen, NotBefore);
            });
    }

    /** Destroys this chunk's RealtimeMesh component. MUST be called on the game thread
     *  (the chunk-cut pass queues retired chunks; AAdaptiveVoxelActor::DrainPendingDestroy
     *  calls this). Safe to call when no component was ever created. */
    void ReleaseComponent() {
        if (URealtimeMeshComponent* Comp = ChunkRtComponent.Get())
            Comp->DestroyComponent();
        ChunkRtComponent = nullptr;
        ChunkRtMesh = nullptr;
        IsInitialized = false;
        ResetDisplayState();
    }
};

FORCEINLINE uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}
