#include "OceanSphereActor.h"
#include "FOceanQuadTreeNode.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include "FastNoise/FastNoise.h"

AOceanSphereActor::AOceanSphereActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    // Mesh components attach here. Position and rotation follow the actor transform;
    // scale is held absolute at (1,1,1) so the tree-at-origin geometry is never scaled.
    MeshAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("MeshAttachmentRoot"));
    MeshAttachmentRoot->SetupAttachment(GetRootComponent());
    MeshAttachmentRoot->SetAbsolute(false, false, true);
}

void AOceanSphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (bIsInitializing) return;
    if (!GetWorld() || GetWorld()->IsPreviewWorld() || !bTickInEditor) return;

    // Only re-initialize when structural params change.
    // Position and rotation are handled live by the actor transform — no re-init needed.
    bool bParamsChanged =
        !FMath::IsNearlyEqual(OceanRadius, LastInitOceanRadius, 0.01) ||
        !FMath::IsNearlyEqual((double)OceanSurfaceThreshold, (double)LastInitThreshold, 0.01) ||
        !FMath::IsNearlyEqual(StandalonePlanetRadius, LastInitStandalonePlanetRadius, 0.01) ||
        !FMath::IsNearlyEqual(StandaloneNoiseAmplitudeRatio, LastInitStandaloneNoiseAmplitudeRatio, 1e-6) ||
        FaceResolution != LastInitFaceResolution ||
        ChunkDepth != LastInitChunkDepth ||
        MinDepth != LastInitMinDepth ||
        MaxDepth != LastInitMaxDepth;

    if (bParamsChanged)
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

    // If no compositor was provided from outside, build a standalone one using the
    // same heightmap noise as AdaptiveVoxelActor so depth culling and sampling work
    // correctly when the ocean actor is placed independently.
    // When the planet actor calls SetCompositor() before Initialize(), this is skipped.
    if (!Compositor.IsValid())
    {
        double PlanetRadius = StandalonePlanetRadius;
        double NoiseAmplitude = PlanetRadius * StandaloneNoiseAmplitudeRatio;
        double RootExtent = (PlanetRadius + NoiseAmplitude) * 1.05;

        Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");

        auto HeightmapLayer = [NoiseNode = Noise, PlanetRadius, NoiseAmplitude, RootExtent]
        (const FSampleInput& Input, float* DensityOut)
            {
                int32 Count = Input.Num();
                double InvNoiseAmplitude = 1.0 / NoiseAmplitude;

                constexpr int32 StackLimit = 64;
                float  StackPX[StackLimit], StackPY[StackLimit], StackPZ[StackLimit], StackNoise[StackLimit];
                double StackDist[StackLimit];
                TArray<float>  HeapPX, HeapPY, HeapPZ, HeapNoise;
                TArray<double> HeapDist;

                float* PX; float* PY; float* PZ; float* NoiseOut; double* Distances;
                if (Count <= StackLimit)
                {
                    PX = StackPX; PY = StackPY; PZ = StackPZ;
                    NoiseOut = StackNoise; Distances = StackDist;
                }
                else
                {
                    HeapPX.SetNumUninitialized(Count); HeapPY.SetNumUninitialized(Count);
                    HeapPZ.SetNumUninitialized(Count); HeapNoise.SetNumUninitialized(Count);
                    HeapDist.SetNumUninitialized(Count);
                    PX = HeapPX.GetData(); PY = HeapPY.GetData(); PZ = HeapPZ.GetData();
                    NoiseOut = HeapNoise.GetData(); Distances = HeapDist.GetData();
                }

                for (int32 i = 0; i < Count; i++)
                {
                    double px = Input.X[i], py = Input.Y[i], pz = Input.Z[i];
                    double Dist = FMath::Sqrt(px * px + py * py + pz * pz);
                    Distances[i] = Dist;
                    double InvDist = (Dist > 1e-10) ? (RootExtent / Dist) : 0.0;
                    PX[i] = (float)(px * InvDist * InvNoiseAmplitude);
                    PY[i] = (float)(py * InvDist * InvNoiseAmplitude);
                    PZ[i] = (float)(pz * InvDist * InvNoiseAmplitude);
                }

                NoiseNode->GenPositionArray3D(NoiseOut, Count, PX, PY, PZ, 0, 0, 0, 0);

                for (int32 i = 0; i < Count; i++)
                {
                    double Clamped = FMath::Clamp((double)NoiseOut[i], -1.0, 1.0);
                    double Height = (Clamped + 1.0) * 0.5 * NoiseAmplitude;
                    DensityOut[i] = (float)(Distances[i] - (PlanetRadius + Height));
                }
            };

        Compositor = MakeShared<FDensitySampleCompositor>();
        Compositor->AddSampleLayer(HeightmapLayer);
    }

    // The cube-sphere is always built at origin with a unit cube of size 1000
    // regardless of actor position. Actor transform handles world placement.
    const double Size = 1000.0;
    const double HalfSize = Size * 0.5;

    const FVector FaceCenters[6] = {
        FVector(HalfSize,       0,       0),
        FVector(-HalfSize,       0,       0),
        FVector(0,  HalfSize,       0),
        FVector(0, -HalfSize,       0),
        FVector(0,       0,  HalfSize),
        FVector(0,       0, -HalfSize),
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
            ChunkDepth,
            Compositor);  // may be null — nodes handle null compositor gracefully

        RootNodes[i]->ChunkAnchorCenter = RootNodes[i]->SphereCenter;

        // Sample root corner densities before splitting so SplitToDepth can
        // distribute them downward without extra full-tree resampling passes.
        if (Compositor.IsValid())
            RootNodes[i]->SampleCornerDensities();
    }

    for (int32 i = 0; i < 6; ++i)
    {
        RootNodes[i]->GenerateMeshData();
        RootNodes[i]->SplitToDepth(ChunkDepth);
    }

    PopulateChunks();

    // Record params so OnConstruction can detect actual structural changes
    LastInitOceanRadius = OceanRadius;
    LastInitFaceResolution = FaceResolution;
    LastInitChunkDepth = ChunkDepth;
    LastInitMinDepth = MinDepth;
    LastInitMaxDepth = MaxDepth;
    LastInitThreshold = OceanSurfaceThreshold;
    LastInitStandalonePlanetRadius = StandalonePlanetRadius;
    LastInitStandaloneNoiseAmplitudeRatio = StandaloneNoiseAmplitudeRatio;

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
            {
                // Skip chunks with no ocean surface — all corners below sea level threshold.
                // These will never have visible geometry and don't need LOD updates.
                if (Compositor.IsValid() && Node->IsFullySubmerged(OceanSurfaceThreshold))
                    continue;
                ChunkNodes.Add(Node);
            }
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

    UE_LOG(LogTemp, Log, TEXT("[Ocean] PopulateChunks: %d chunks active (culled submerged)"), ChunkNodes.Num());
}

void AOceanSphereActor::RebuildChunkStreamData(
    TSharedPtr<FOceanMeshChunk> Chunk,
    TSharedPtr<FOceanQuadTreeNode> ChunkNode)
{
    if (!Chunk.IsValid() || !ChunkNode.IsValid()) return;
    if (!Chunk->InnerMeshData.IsValid() || !Chunk->EdgeMeshData.IsValid()) return;

    TArray<TSharedPtr<FOceanQuadTreeNode>> Leaves;
    FOceanQuadTreeNode::CollectLeaves(ChunkNode, Leaves);

    // Depth encoding range — negative values = below terrain surface, positive = above.
    // These bounds should comfortably cover the planet's noise amplitude range.
    // The planet actor can expose these as params and pass them down later.
    constexpr float DepthRangeMin = -100000.f;
    constexpr float DepthRangeMax = 100000.f;

    // Inner stream
    TSharedPtr<FOceanStreamData> NewInner = MakeShared<FOceanStreamData>();
    NewInner->MeshGroupKey = Chunk->InnerMeshData->MeshGroupKey;
    NewInner->MeshSectionKey = Chunk->InnerMeshData->MeshSectionKey;

    auto InnerPos = NewInner->GetPositionStream();
    auto InnerTan = NewInner->GetTangentStream();
    auto InnerTex = NewInner->GetTexCoordStream();
    auto InnerCol = NewInner->GetColorStream();
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
                float Depth = Leaf->VertexDepths.IsValidIndex(vi) ? Leaf->VertexDepths[vi] : 0.f;
                InnerCol.Add(FOceanQuadTreeNode::EncodeDepthToColor(Depth, DepthRangeMin, DepthRangeMax));
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
    auto EdgeCol = NewEdge->GetColorStream();
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
                float Depth = Leaf->VertexDepths.IsValidIndex(vi) ? Leaf->VertexDepths[vi] : 0.f;
                EdgeCol.Add(FOceanQuadTreeNode::EncodeDepthToColor(Depth, DepthRangeMin, DepthRangeMax));
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
    if (!MeshAttachmentRoot) return;
    TArray<USceneComponent*> AttachedComponents;
    MeshAttachmentRoot->GetChildrenComponents(false, AttachedComponents);
    for (USceneComponent* Comp : AttachedComponents)
    {
        if (URealtimeMeshComponent* RtComp = Cast<URealtimeMeshComponent>(Comp))
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
// LOD pipeline
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

            TArray<TSharedPtr<FOceanQuadTreeNode>> ChunkNodes;
            TArray<TSharedPtr<FOceanMeshChunk>>    MeshChunks;
            Self->ChunkMap.GenerateKeyArray(ChunkNodes);
            for (auto& CN : ChunkNodes)
                MeshChunks.Add(Self->ChunkMap[CN]);

            int32 NumChunks = ChunkNodes.Num();

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
                        FOceanQuadTreeNode::CollectLeaves(ChunkNode, AllChunkLeaves[idx]);
                        return;
                    }

                    TArray<TSharedPtr<FOceanQuadTreeNode>> Leaves;
                    FOceanQuadTreeNode::CollectLeaves(ChunkNode, Leaves);

                    bool bAnyChanged = false;
                    for (auto& Leaf : Leaves)
                    {
                        if (Leaf->TrySetLod(Predicted, ThresholdSq, MergeThresholdSq, FOVScale))
                            bAnyChanged = true;
                    }

                    if (bAnyChanged)
                    {
                        AllChunkLeaves[idx].Reset();
                        FOceanQuadTreeNode::CollectLeaves(ChunkNode, AllChunkLeaves[idx]);
                        ChunkTreeChanged[idx] = true;
                    }
                    else
                    {
                        AllChunkLeaves[idx] = MoveTemp(Leaves);
                    }
                });

            if (Self->InitGeneration != CapturedGen) { Self->bLodUpdateRunning = false; return; }

            // Phase 2: Neighbor stitch
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

            // Phase 3: Rebuild stream data for changed chunks
            ParallelFor(NumChunks, [&](int32 idx)
                {
                    if (Self->InitGeneration != CapturedGen) return;
                    if (ChunkTreeChanged[idx] || ChunkEdgeChanged[idx])
                        RebuildChunkStreamData(MeshChunks[idx], ChunkNodes[idx]);
                });

            Self->bLodUpdateRunning = false;

            // Phase 4: Game-thread component updates
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
        if (Quad >= (uint8)Cur->Children.Num()) return Cur;
        Cur = Cur->Children[Quad];
        if (!Cur.IsValid()) return nullptr;
    }
    return Cur;
}