// FMeshingStructs.h - Data structures for dual-contour mesh generation: per-corner
// sample batching, mesh vertex/edge data, RealtimeMesh stream wrappers, and chunk
// component management.

#pragma once

#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

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
 *  along with the edge itself. IsValid is false if the edge was filtered out by
 *  ShouldProcessEdge. */
struct VOXELPLUGIN_API FEdgeVertexData {
    TArray<FMeshVertex> Vertices;
    TOptional<FNodeEdge> Edge;
    bool IsValid = false;
};

/** Wrapper around a RealtimeMesh FRealtimeMeshStreamSet, pre-configured with
 *  the six vertex streams needed for rendering: position, tangents, triangles,
 *  polygroups, texcoords, and vertex color. Provides typed stream builder
 *  accessors for each. */
struct VOXELPLUGIN_API FMeshStreamData {
    FRealtimeMeshSectionGroupKey MeshGroupKey;
    FRealtimeMeshSectionKey MeshSectionKey;
    FRealtimeMeshStreamSet MeshStream;

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

    void ResetStreams() {
        if (auto* Position = MeshStream.Find(FRealtimeMeshStreams::Position)) Position->Empty();
        if (auto* Tangent = MeshStream.Find(FRealtimeMeshStreams::Tangents)) Tangent->Empty();
        if (auto* Triangle = MeshStream.Find(FRealtimeMeshStreams::Triangles)) Triangle->Empty();
        if (auto* Polygroup = MeshStream.Find(FRealtimeMeshStreams::PolyGroups)) Polygroup->Empty();
        if (auto* TexCoord = MeshStream.Find(FRealtimeMeshStreams::TexCoords)) TexCoord->Empty();
        if (auto* Color = MeshStream.Find(FRealtimeMeshStreams::Color)) Color->Empty();
    }
};

/** A spatial subdivision of the octree used for mesh generation. Each chunk corresponds
 *  to a node at ChunkDepth and owns a RealtimeMesh component that renders the dual-contour
 *  surface within its bounds.
 *
 *  Lifecycle: InitializeData sets the spatial bounds and stream keys. The RealtimeMesh
 *  component is lazily created on the game thread during the first UpdateComponent call
 *  (via InitializeComponent). Subsequent updates push new stream data when IsDirty is set. */
struct VOXELPLUGIN_API FMeshChunk {
    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;
    TWeakObjectPtr<USceneComponent> CachedMeshAttachRoot;
    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;

    FVector ChunkCenter;
    double ChunkExtent;

    /** All sign-change edges within this chunk's leaf nodes, collected during LOD updates. */
    TArray<FNodeEdge> ChunkEdges;

    TSharedPtr<FMeshStreamData> SurfaceMeshData;

    bool IsDirty = false;
    bool IsInitialized = false;

    TWeakObjectPtr<URealtimeMeshSimple> ChunkRtMesh;
    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;
    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey(0);

    /** Sets spatial bounds and creates the stream data with section group/section keys. */
    void InitializeData(FVector InCenter, double InExtent) {
        SurfaceMeshData = MakeShared<FMeshStreamData>();

        ChunkCenter = InCenter;
        ChunkExtent = InExtent;

        SurfaceMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName("SURFACE"));
        SurfaceMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(SurfaceMeshData->MeshGroupKey, 0);
    };

    /** Creates the RealtimeMesh UObject and component, attaches to the parent actor,
     *  and configures collision. Called lazily from UpdateComponent on the game thread. */
    void InitializeComponent(ARealtimeMeshActor* InParentActor, USceneComponent* InAttachRoot, UMaterialInterface* InSurfaceMaterial) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        ChunkRtMesh = NewObject<URealtimeMeshSimple>(InParentActor);
        ChunkRtMesh->SetCollisionConfig(cConfig);
        ChunkRtMesh->SetupMaterialSlot(0, "Surface Material");

        ChunkRtComponent = NewObject<URealtimeMeshComponent>(InParentActor, URealtimeMeshComponent::StaticClass());
        ChunkRtComponent->RegisterComponent();

        ChunkRtComponent->SetMaterial(0, InSurfaceMaterial);

        ChunkRtComponent->AttachToComponent(InAttachRoot, FAttachmentTransformRules::KeepRelativeTransform);
        ChunkRtComponent->SetRelativeLocation(ChunkCenter);
        ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        ChunkRtComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh.Get());
        ChunkRtComponent->SetRenderCustomDepth(true);
        ChunkRtComponent->SetVisibleInRayTracing(false);

        ChunkRtMesh->CreateSectionGroup(SurfaceMeshData->MeshGroupKey, FRealtimeMeshStreamSet());

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

    /** Pushes current stream data to the RealtimeMesh component on the game thread.
     *  Takes a TSharedPtr to Self to prevent destruction while the async task is in flight.
     *  Lazily initializes the component on first call. Clears the section group if there
     *  are no triangles to render. */
    void UpdateComponent(TSharedPtr<FMeshChunk> Self) {
        AsyncTask(ENamedThreads::GameThread, [Self]() {
            // 1. Lazy Init
            if (!Self->IsInitialized) {
                ARealtimeMeshActor* Parent = Self->CachedParentActor.Get();
                USceneComponent* AttachRoot = Self->CachedMeshAttachRoot.Get();
                UMaterialInterface* SurfaceMaterial = Self->CachedSurfaceMaterial.Get();
                if (!Parent || !AttachRoot || !SurfaceMaterial) return;
                Self->InitializeComponent(Parent, AttachRoot, SurfaceMaterial);
            }

            // 2. Ensure valid component pointers
            URealtimeMeshSimple* MeshPtr = Self->ChunkRtMesh.Get();
            URealtimeMeshComponent* CompPtr = Self->ChunkRtComponent.Get();
            if (!MeshPtr || !CompPtr) return;
            if (!Self->IsDirty) return;

            // 3. Surface update
            auto* SrfTriStream = Self->SurfaceMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles);
            int32 SrfNumTris = SrfTriStream ? SrfTriStream->Num() : 0;
            if (SrfNumTris <= 0)
            {
                MeshPtr->UpdateSectionGroup(Self->SurfaceMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
            }
            else
            {
                MeshPtr->UpdateSectionGroup(Self->SurfaceMeshData->MeshGroupKey, Self->SurfaceMeshData->MeshStream);
                FRealtimeMeshSectionConfig SrfConfig(0); // material slot 0
                MeshPtr->UpdateSectionConfig(Self->SurfaceMeshData->MeshSectionKey, SrfConfig, true);
            }

            Self->IsDirty = false;
            });
    }
};

FORCEINLINE uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}