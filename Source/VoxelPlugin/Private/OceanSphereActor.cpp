#include "OceanSphereActor.h"
#include "FOceanQuadTreeNode.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

AOceanSphereActor::AOceanSphereActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
}

void AOceanSphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (bIsInitializing) return;
    if (GetWorld() && !GetWorld()->IsPreviewWorld() && bTickInEditor)
        Initialize();
}

void AOceanSphereActor::BeginPlay()
{
    Super::BeginPlay();
    Initialize();
}

void AOceanSphereActor::BeginDestroy()
{
    bIsDestroyed = true;
    if (UWorld* W = GetWorld())
        W->GetTimerManager().ClearAllTimersForObject(this);
    Super::BeginDestroy();
}

bool AOceanSphereActor::ShouldTickIfViewportsOnly() const
{
    return bTickInEditor && bInitialized;
}

void AOceanSphereActor::Initialize()
{
    bIsInitializing = true;
    bInitialized = false;
    ++InitGeneration;

    if (UWorld* W = GetWorld())
        W->GetTimerManager().ClearTimer(LodTimerHandle);

    bLodUpdateRunning = false;

    CleanupComponents();
    ChunkMap.Empty();
    for (int32 i = 0; i < 6; ++i)
        RootNodes[i].Reset();

    const double Size = 1000.0;
    const double HalfSize = Size * 0.5;

    const FVector FaceCenters[6] = {
        FVector(HalfSize, 0, 0),
        FVector(-HalfSize, 0, 0),
        FVector(0,  HalfSize, 0),
        FVector(0, -HalfSize, 0),
        FVector(0, 0,  HalfSize),
        FVector(0, 0, -HalfSize),
    };

    for (int32 i = 0; i < 6; ++i)
    {
        RootNodes[i] = MakeShared<FOceanQuadTreeNode>(
            this,
            FCubeTransform::FaceTransforms[i],
            FQuadIndex((uint8)i),
            FaceCenters[i],
            Size,
            OceanRadius,
            MinDepth,
            MaxDepth,
            ChunkDepth);
        RootNodes[i]->ChunkAnchorCenter = RootNodes[i]->SphereCenter;
    }

    for (int32 i = 0; i < 6; ++i)
    {
        RootNodes[i]->GenerateMeshData();
        RootNodes[i]->SplitToDepth(ChunkDepth);
    }

    PopulateChunks();

    bInitialized = true;
    bIsInitializing = false;
    RunLodUpdateTask();
}

void AOceanSphereActor::PopulateChunks()
{
    TArray<TSharedPtr<FOceanQuadTreeNode>> ChunkNodes;
    for (int32 i = 0; i < 6; ++i)
    {
        TArray<TSharedPtr<FOceanQuadTreeNode>> Stack;
        Stack.Push(RootNodes[i]);
        while (Stack.Num() > 0)
        {
            TSharedPtr<FOceanQuadTreeNode> Node = Stack.Pop(EAllowShrinking::No);
            if (!Node.IsValid()) continue;
            if (Node->GetDepth() == ChunkDepth)
                ChunkNodes.Add(Node);
            else
                for (auto& Child : Node->Children)
                    if (Child.IsValid()) Stack.Add(Child);
        }
    }

    TArray<TSharedPtr<FOceanMeshChunk>> NewChunks;
    NewChunks.SetNum(ChunkNodes.Num());

    ParallelFor(ChunkNodes.Num(), [&](int32 i)
        {
            NewChunks[i] = MakeShared<FOceanMeshChunk>();
            NewChunks[i]->CachedOwner = this;
            NewChunks[i]->InitializeData(ChunkNodes[i]->SphereCenter);
            RebuildChunkStreamData(NewChunks[i], ChunkNodes[i]);
        });

    for (int32 i = 0; i < ChunkNodes.Num(); ++i)
    {
        NewChunks[i]->IsDirty = true;
        ChunkMap.Add(ChunkNodes[i], NewChunks[i]);
    }
}

void AOceanSphereActor::RebuildChunkStreamData(
    TSharedPtr<FOceanMeshChunk> Chunk,
    TSharedPtr<FOceanQuadTreeNode> ChunkNode)
{
    if (!Chunk.IsValid() || !ChunkNode.IsValid()) return;
    if (!Chunk->InnerMeshData.IsValid() || !Chunk->EdgeMeshData.IsValid()) return;

    TArray<TSharedPtr<FOceanQuadTreeNode>> Leaves;
    FOceanQuadTreeNode::CollectLeaves(ChunkNode, Leaves);

    // Inner stream
    TSharedPtr<FOceanStreamData> NewInner = MakeShared<FOceanStreamData>();
    NewInner->MeshGroupKey = Chunk->InnerMeshData->MeshGroupKey;
    NewInner->MeshSectionKey = Chunk->InnerMeshData->MeshSectionKey;

    auto InnerPos = NewInner->GetPositionStream();
    auto InnerTan = NewInner->GetTangentStream();
    auto InnerTex = NewInner->GetTexCoordStream();
    auto InnerTri = NewInner->GetTriangleStream();
    auto InnerPG = NewInner->GetPolygroupStream();

    for (auto& Leaf : Leaves)
    {
        if (!Leaf->HasGenerated) continue;
        for (int32 PatchIdx : Leaf->PatchTriangleIndices)
        {
            FIndex3UI Tri = Leaf->AllTriangles[PatchIdx];
            for (int32 v = 0; v < 3; ++v)
            {
                uint32 vi = (v == 0) ? Tri.V0 : (v == 1) ? Tri.V1 : Tri.V2;
                InnerPos.Add(Leaf->Vertices[vi]);
                InnerTex.Add(Leaf->TexCoords[vi]);
                FRealtimeMeshTangentsHighPrecision T; T.SetNormal(Leaf->Normals[vi]);
                InnerTan.Add(T);
            }
            uint32 base = (uint32)InnerPos.Num() - 3;
            InnerTri.Add(FIndex3UI(base, base + 1, base + 2));
            InnerPG.Add(0);
        }
    }
    Chunk->InnerMeshData = NewInner;

    // Edge stream
    TSharedPtr<FOceanStreamData> NewEdge = MakeShared<FOceanStreamData>();
    NewEdge->MeshGroupKey = Chunk->EdgeMeshData->MeshGroupKey;
    NewEdge->MeshSectionKey = Chunk->EdgeMeshData->MeshSectionKey;

    auto EdgePos = NewEdge->GetPositionStream();
    auto EdgeTan = NewEdge->GetTangentStream();
    auto EdgeTex = NewEdge->GetTexCoordStream();
    auto EdgeTri = NewEdge->GetTriangleStream();
    auto EdgePG = NewEdge->GetPolygroupStream();

    for (auto& Leaf : Leaves)
    {
        if (!Leaf->HasGenerated) continue;
        for (FIndex3UI Tri : Leaf->EdgeTriangles)
        {
            if (Leaf->FaceTransform.bFlipWinding) Tri = FIndex3UI(Tri.V0, Tri.V2, Tri.V1);
            for (int32 v = 0; v < 3; ++v)
            {
                uint32 vi = (v == 0) ? Tri.V0 : (v == 1) ? Tri.V1 : Tri.V2;
                EdgePos.Add(Leaf->Vertices[vi]);
                EdgeTex.Add(Leaf->TexCoords[vi]);
                FRealtimeMeshTangentsHighPrecision T; T.SetNormal(Leaf->Normals[vi]);
                EdgeTan.Add(T);
            }
            uint32 base = (uint32)EdgePos.Num() - 3;
            EdgeTri.Add(FIndex3UI(base, base + 1, base + 2));
            EdgePG.Add(0);
        }
    }
    Chunk->EdgeMeshData = NewEdge;
    Chunk->IsDirty = true;
}

void AOceanSphereActor::CleanupComponents()
{
    TArray<USceneComponent*> SceneChildren;
    GetRootComponent()->GetChildrenComponents(false, SceneChildren);
    for (USceneComponent* Child : SceneChildren)
    {
        if (URealtimeMeshComponent* RtComp = Cast<URealtimeMeshComponent>(Child))
            RtComp->DestroyComponent();
    }
}

void AOceanSphereActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!bInitialized) return;

    if (UWorld* W = GetWorld())
    {
        const TArray<FVector>& Views = W->ViewLocationsRenderedLastFrame;
        if (Views.Num() > 0)
            CameraPosition = Views[0];

        APlayerCameraManager* Cam = UGameplayStatics::GetPlayerCameraManager(W, 0);
        if (Cam)
            CameraFOV = Cam->GetFOVAngle();
    }
}

// ---------------------------------------------------------------------------
// LOD pipeline -- optimized: single CollectLeaves per chunk, dirty tracking
//
// ONE background task:
//   1. Per-chunk: collect leaves ONCE, LOD pass, track if tree changed
//   2. Neighbor stitch on pre-collected leaves, track if edges changed
//   3. Rebuild stream data ONLY for chunks that changed
//   4. Single game-thread dispatch for dirty chunk component updates
// ---------------------------------------------------------------------------

void AOceanSphereActor::RunLodUpdateTask()
{
    if (bLodUpdateRunning || bIsDestroyed) return;
    bLodUpdateRunning = true;

    TWeakObjectPtr<AOceanSphereActor> WeakThis(this);
    uint32 CapturedGen = InitGeneration;

    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, CapturedGen]()
        {
            AOceanSphereActor* Self = WeakThis.Get();
            if (!Self || Self->bIsDestroyed || Self->InitGeneration != CapturedGen)
            {
                if (Self) Self->bLodUpdateRunning = false;
                return;
            }

            FVector Velocity = Self->CameraPosition - Self->LastLodCameraPos;
            FVector Predicted = Self->CameraPosition + Velocity * Self->VelocityLookAheadFactor;
            Self->LastLodCameraPos = Self->CameraPosition;

            double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(Self->CameraFOV * 0.5));
            double ThresholdSq = Self->ScreenSpaceThreshold * Self->ScreenSpaceThreshold;
            double MergeThresholdSq =
                (Self->ScreenSpaceThreshold * 0.5) * (Self->ScreenSpaceThreshold * 0.5);

            // Snapshot chunk arrays
            TArray<TSharedPtr<FOceanQuadTreeNode>> ChunkNodes;
            TArray<TSharedPtr<FOceanMeshChunk>>    MeshChunks;
            Self->ChunkMap.GenerateKeyArray(ChunkNodes);
            for (auto& CN : ChunkNodes)
                MeshChunks.Add(Self->ChunkMap[CN]);

            int32 NumChunks = ChunkNodes.Num();

            // Per-chunk: collect leaves once, do LOD, collect leaves again only if changed
            // Store leaves for reuse in neighbor stitch and stream rebuild
            TArray<TArray<TSharedPtr<FOceanQuadTreeNode>>> AllChunkLeaves;
            AllChunkLeaves.SetNum(NumChunks);
            TArray<bool> ChunkTreeChanged;
            ChunkTreeChanged.SetNumZeroed(NumChunks);

            // Phase 1: LOD pass (parallel per chunk)
            ParallelFor(NumChunks, [&](int32 idx)
                {
                    if (Self->InitGeneration != CapturedGen) return;
                    TSharedPtr<FOceanQuadTreeNode> ChunkNode = ChunkNodes[idx];
                    if (!ChunkNode.IsValid()) return;

                    // Early-out coarse test
                    FVector WorldCenter = ChunkNode->SphereCenter + Self->GetActorLocation();
                    double DistSq = FMath::Max(FVector::DistSquared(Predicted, WorldCenter), 1e-12);

                    double ChunkLhs = 2.0 * ChunkNode->WorldExtent * FOVScale;
                    bool CouldSplit = (ChunkLhs * ChunkLhs) > (ThresholdSq * DistSq)
                        || ChunkNode->GetDepth() < ChunkNode->MinDepth;

                    double SmallestExtent = ChunkNode->WorldExtent /
                        (double)(1 << (ChunkNode->MaxDepth - ChunkNode->GetDepth()));
                    double SmallLhs = 2.0 * SmallestExtent * FOVScale;
                    bool CouldMerge = (SmallLhs * SmallLhs) < (MergeThresholdSq * DistSq);

                    if (!CouldSplit && !CouldMerge)
                    {
                        // Still need leaves for neighbor stitch
                        FOceanQuadTreeNode::CollectLeaves(ChunkNode, AllChunkLeaves[idx]);
                        return;
                    }

                    // Collect leaves once
                    TArray<TSharedPtr<FOceanQuadTreeNode>> Leaves;
                    FOceanQuadTreeNode::CollectLeaves(ChunkNode, Leaves);
                    int32 PreCount = Leaves.Num();

                    // LOD evaluation
                    bool bAnyChanged = false;
                    for (auto& Leaf : Leaves)
                    {
                        if (Leaf->TrySetLod(Predicted, ThresholdSq, MergeThresholdSq, FOVScale))
                            bAnyChanged = true;
                    }

                    if (bAnyChanged)
                    {
                        // Tree changed -- re-collect leaves for the updated structure
                        AllChunkLeaves[idx].Reset();
                        FOceanQuadTreeNode::CollectLeaves(ChunkNode, AllChunkLeaves[idx]);
                        ChunkTreeChanged[idx] = true;
                    }
                    else
                    {
                        // No change -- reuse the leaves we already collected
                        AllChunkLeaves[idx] = MoveTemp(Leaves);
                    }
                });

            if (Self->InitGeneration != CapturedGen) { Self->bLodUpdateRunning = false; return; }

            // Phase 2: Neighbor stitch using pre-collected leaves (parallel per chunk)
            TArray<bool> ChunkEdgeChanged;
            ChunkEdgeChanged.SetNumZeroed(NumChunks);

            ParallelFor(NumChunks, [&](int32 idx)
                {
                    if (Self->InitGeneration != CapturedGen) return;
                    bool bAnyEdgeChanged = false;
                    for (auto& Leaf : AllChunkLeaves[idx])
                    {
                        if (Leaf->CheckNeighbors())
                        {
                            Leaf->RebuildEdgeTriangles();
                            bAnyEdgeChanged = true;
                        }
                    }
                    ChunkEdgeChanged[idx] = bAnyEdgeChanged;
                });

            if (Self->InitGeneration != CapturedGen) { Self->bLodUpdateRunning = false; return; }

            // Phase 3: Rebuild stream data only for chunks where tree or edges changed (parallel)
            ParallelFor(NumChunks, [&](int32 idx)
                {
                    if (Self->InitGeneration != CapturedGen) return;
                    if (ChunkTreeChanged[idx] || ChunkEdgeChanged[idx])
                        RebuildChunkStreamData(MeshChunks[idx], ChunkNodes[idx]);
                });

            Self->bLodUpdateRunning = false;

            // Phase 4: Single game-thread dispatch for dirty chunks
            TArray<TSharedPtr<FOceanMeshChunk>> DirtyChunks;
            for (int32 idx = 0; idx < NumChunks; ++idx)
            {
                if (MeshChunks[idx]->IsDirty)
                    DirtyChunks.Add(MeshChunks[idx]);
            }

            AsyncTask(ENamedThreads::GameThread, [WeakThis, DirtyChunks, CapturedGen]()
                {
                    AOceanSphereActor* Self = WeakThis.Get();
                    if (!Self || Self->bIsDestroyed || Self->InitGeneration != CapturedGen) return;

                    int32 NumInitThisFrame = 0;
                    const int32 MaxInitPerFrame = 32;

                    for (auto& Chunk : DirtyChunks)
                    {
                        if (!Chunk.IsValid()) continue;

                        if (!Chunk->IsInitialized)
                        {
                            if (NumInitThisFrame >= MaxInitPerFrame) continue;
                            AOceanSphereActor* OwnerPtr = Chunk->CachedOwner.Get();
                            if (!OwnerPtr) continue;
                            Chunk->InitializeComponent(OwnerPtr);
                            NumInitThisFrame++;
                        }

                        URealtimeMeshSimple* MeshPtr = Chunk->ChunkRtMesh.Get();
                        if (!MeshPtr || !Chunk->IsDirty) continue;

                        // Inner
                        auto* InnerTriStream = Chunk->InnerMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles);
                        if (InnerTriStream && InnerTriStream->Num() > 0)
                        {
                            MeshPtr->UpdateSectionGroup(Chunk->InnerMeshData->MeshGroupKey, Chunk->InnerMeshData->MeshStream);
                            FRealtimeMeshSectionConfig Config(0);
                            MeshPtr->UpdateSectionConfig(Chunk->InnerMeshData->MeshSectionKey, Config, true);
                        }
                        else
                            MeshPtr->UpdateSectionGroup(Chunk->InnerMeshData->MeshGroupKey, FRealtimeMeshStreamSet());

                        // Edge
                        auto* EdgeTriStream = Chunk->EdgeMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles);
                        if (EdgeTriStream && EdgeTriStream->Num() > 0)
                        {
                            MeshPtr->UpdateSectionGroup(Chunk->EdgeMeshData->MeshGroupKey, Chunk->EdgeMeshData->MeshStream);
                            FRealtimeMeshSectionConfig Config(0);
                            Config.bCastsShadow = false;
                            MeshPtr->UpdateSectionConfig(Chunk->EdgeMeshData->MeshSectionKey, Config, true);
                        }
                        else
                            MeshPtr->UpdateSectionGroup(Chunk->EdgeMeshData->MeshGroupKey, FRealtimeMeshStreamSet());

                        Chunk->IsDirty = false;
                    }

                    // Reschedule
                    if (UWorld* W = Self->GetWorld())
                    {
                        W->GetTimerManager().SetTimer(
                            Self->LodTimerHandle,
                            Self,
                            &AOceanSphereActor::RunLodUpdateTask,
                            (float)Self->MinLodInterval,
                            false);
                    }
                });

        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

TSharedPtr<FOceanQuadTreeNode> AOceanSphereActor::GetNodeByIndex(
    const FQuadIndex& Index) const
{
    if (Index.FaceId >= 6) return nullptr;
    TSharedPtr<FOceanQuadTreeNode> Cur = RootNodes[Index.FaceId];
    if (!Cur.IsValid() || Index.IsRoot()) return Cur;

    uint8 Depth = Index.GetDepth();
    for (uint8 Level = 0; Level < Depth; ++Level)
    {
        uint8 Quad = Index.GetQuadrantAtDepth(Level);
        if (Cur->IsLeaf()) return Cur;
        if (Quad >= Cur->Children.Num()) return Cur;  // bounds safety
        Cur = Cur->Children[Quad];
        if (!Cur.IsValid()) return nullptr;
    }
    return Cur;
}