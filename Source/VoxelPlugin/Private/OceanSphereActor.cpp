#include "OceanSphereActor.h"
#include "FOceanQuadTreeNode.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

AOceanSphereActor::AOceanSphereActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    // Default scale = planet radius in world units (cm).
    // 80,000,000 cm = 800 km radius, matching the default terrain actor.
    SetActorScale3D(FVector(80000000.0));

    // Mesh components attach here. Inherits actor position and rotation,
    // absolute scale (1,1,1) so the sphere built at world-scale radius
    // is never additionally scaled by the actor transform.
    MeshAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("MeshAttachmentRoot"));
    MeshAttachmentRoot->SetupAttachment(GetRootComponent());
    MeshAttachmentRoot->SetAbsolute(false, false, true);
}

void AOceanSphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (bIsInitializing) return;
    if (!GetWorld() || GetWorld()->IsPreviewWorld() || !bTickInEditor) return;

    // Only reconstruct when scale changes � position and rotation are handled
    // live by MeshAttachmentRoot inheriting the actor transform.
    if (!bInitialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
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

    // Derive ocean radius from actor scale � uniform across all axes.
    double ActorRadius = GetActorScale3D().GetMax();
    double ActorNoiseAmplitude = ActorRadius * NoiseAmplitudeRatio;
    double ActorRootExtent = (ActorRadius + ActorNoiseAmplitude) * 1.05;
    OceanRadius = ActorRadius;

    // Build the density compositor � identical heightmap layer to AAdaptiveVoxelActor
    // so the ocean and terrain evaluate the same SDF. The tree pushes sampled densities
    // down to child node corners at split time; nodes do not call the compositor directly.
    Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");

    auto HeightmapLayer = [NoiseNode = Noise, PlanetRadius = ActorRadius, NoiseAmplitude = ActorNoiseAmplitude](const FSampleInput& Input, float* DensityOut)
        {
            int32 Count = Input.Num();
            double InvNoiseAmplitude = 1.0 / NoiseAmplitude;
            double RootExtent = (PlanetRadius + NoiseAmplitude) * 1.05;

            constexpr int32 StackLimit = 64;
            float  StackPX[StackLimit], StackPY[StackLimit], StackPZ[StackLimit], StackNoise[StackLimit];
            double StackDist[StackLimit];
            TArray<float>  HeapPX, HeapPY, HeapPZ, HeapNoise;
            TArray<double> HeapDist;

            float* PX; float* PY; float* PZ; float* NoiseOut; double* Distances;
            if (Count <= StackLimit)
            {
                PX = StackPX; PY = StackPY; PZ = StackPZ; NoiseOut = StackNoise; Distances = StackDist;
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

    // The cube-sphere is always built at world scale in actor-local space (origin 0,0,0).
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
            ChunkDepth);
        RootNodes[i]->ChunkAnchorCenter = RootNodes[i]->SphereCenter;
        ComputeRootNodeDensities(RootNodes[i]);
    }

    for (int32 i = 0; i < 6; ++i)
    {
        RootNodes[i]->GenerateMeshData();
        RootNodes[i]->SplitToDepth(ChunkDepth);
        PushDensityToChildren(RootNodes[i]);
    }

    PopulateChunks();

    LastInitScale = GetActorScale3D();
    bInitialized = true;
    bIsInitializing = false;
    RunLodUpdateTask();
}

// ---------------------------------------------------------------------------
// Density sampling — does not touch mesh data, positions, or winding
// ---------------------------------------------------------------------------

void AOceanSphereActor::ComputeRootNodeDensities(TSharedPtr<FOceanQuadTreeNode> Node)
{
    // Sample the 4 sphere-projected corners of a root node before any splits.
    // Corner layout: BL=0 (U-,V-), TL=1 (U-,V+), BR=2 (U+,V-), TR=3 (U+,V+).
    if (!Node.IsValid() || !Compositor.IsValid()) return;

    const double H = Node->HalfSize;
    const int32  U = Node->FaceTransform.AxisMap[0];
    const int32  V = Node->FaceTransform.AxisMap[1];
    const int32  DU = Node->FaceTransform.AxisDir[0];
    const int32  DV = Node->FaceTransform.AxisDir[1];

    const double USigns[4] = { -1, -1, +1, +1 };
    const double VSigns[4] = { -1, +1, -1, +1 };

    float SX[4], SY[4], SZ[4], Out[4];
    for (int32 i = 0; i < 4; ++i)
    {
        FVector P = Node->CubeCenter;
        P[U] += DU * USigns[i] * H;
        P[V] += DV * VSigns[i] * H;
        FVector SP = P.GetSafeNormal() * Node->OceanRadius;
        SX[i] = (float)SP.X; SY[i] = (float)SP.Y; SZ[i] = (float)SP.Z;
    }
    FSampleInput Input(SX, SY, SZ, 4);
    Compositor->Sample(Input, Out);
    for (int32 i = 0; i < 4; ++i)
        Node->CornerDensities[i] = Out[i];
    Node->bDensitySampled = true;
}

void AOceanSphereActor::PushDensityToChildren(TSharedPtr<FOceanQuadTreeNode> Node)
{
    // Recursive post-pass over an already-split subtree.
    // If this node has density and its children don't yet, sample the 5 new
    // grid positions (4 edge mids + 1 center) and distribute all 4 corner
    // densities to each child. Does not touch mesh data at all.
    //
    // 9-point grid (U right, V up):
    //   G6--G7--G8
    //   |   |   |
    //   G3--G4--G5
    //   |   |   |
    //   G0--G1--G2
    //
    // Parent corners: BL=0→G0, TL=1→G6, BR=2→G2, TR=3→G8
    // New samples:    G1=bot-edge, G3=left-edge, G4=center, G5=right-edge, G7=top-edge

    if (!Node.IsValid() || Node->IsLeaf()) return;

    if (Node->bDensitySampled && !Node->Children[0]->bDensitySampled)
    {
        if (!Compositor.IsValid()) return;

        const double H = Node->HalfSize;
        const int32  U = Node->FaceTransform.AxisMap[0];
        const int32  V = Node->FaceTransform.AxisMap[1];
        const int32  DU = Node->FaceTransform.AxisDir[0];
        const int32  DV = Node->FaceTransform.AxisDir[1];

        auto ToSphere = [&](double UOff, double VOff) -> FVector
            {
                FVector P = Node->CubeCenter;
                P[U] += DU * UOff; P[V] += DV * VOff;
                return P.GetSafeNormal() * Node->OceanRadius;
            };

        FVector NP[5] = {
            ToSphere(0, -H),  // G1 bottom-edge mid
            ToSphere(-H,  0),  // G3 left-edge mid
            ToSphere(0,  0),  // G4 center
            ToSphere(+H,  0),  // G5 right-edge mid
            ToSphere(0, +H),  // G7 top-edge mid
        };
        float SX[5], SY[5], SZ[5], Out[5];
        for (int32 i = 0; i < 5; ++i)
        {
            SX[i] = (float)NP[i].X; SY[i] = (float)NP[i].Y; SZ[i] = (float)NP[i].Z;
        }
        FSampleInput Input(SX, SY, SZ, 5);
        Compositor->Sample(Input, Out);

        const float G0 = Node->CornerDensities[0], G6 = Node->CornerDensities[1];
        const float G2 = Node->CornerDensities[2], G8 = Node->CornerDensities[3];
        const float G1 = Out[0], G3 = Out[1], G4 = Out[2], G5 = Out[3], G7 = Out[4];

        // Child 0 (BL quadrant): corners G0, G3, G1, G4
        Node->Children[0]->CornerDensities[0] = G0; Node->Children[0]->CornerDensities[1] = G3;
        Node->Children[0]->CornerDensities[2] = G1; Node->Children[0]->CornerDensities[3] = G4;
        Node->Children[0]->bDensitySampled = true;

        // Child 1 (TL quadrant): corners G3, G6, G4, G7
        Node->Children[1]->CornerDensities[0] = G3; Node->Children[1]->CornerDensities[1] = G6;
        Node->Children[1]->CornerDensities[2] = G4; Node->Children[1]->CornerDensities[3] = G7;
        Node->Children[1]->bDensitySampled = true;

        // Child 2 (BR quadrant): corners G1, G4, G2, G5
        Node->Children[2]->CornerDensities[0] = G1; Node->Children[2]->CornerDensities[1] = G4;
        Node->Children[2]->CornerDensities[2] = G2; Node->Children[2]->CornerDensities[3] = G5;
        Node->Children[2]->bDensitySampled = true;

        // Child 3 (TR quadrant): corners G4, G7, G5, G8
        Node->Children[3]->CornerDensities[0] = G4; Node->Children[3]->CornerDensities[1] = G7;
        Node->Children[3]->CornerDensities[2] = G5; Node->Children[3]->CornerDensities[3] = G8;
        Node->Children[3]->bDensitySampled = true;
    }

    for (auto& Child : Node->Children)
        PushDensityToChildren(Child);
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
    if (!MeshAttachmentRoot) return;
    TArray<USceneComponent*> Attached;
    MeshAttachmentRoot->GetChildrenComponents(false, Attached);
    for (USceneComponent* Child : Attached)
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
        {
            // Convert world-space camera to actor-local space (position + rotation only).
            FVector WorldCamPos = Views[0];
            FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
            CameraPosition = NoScaleTransform.InverseTransformPosition(WorldCamPos);
        }

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
            // Camera (Predicted) and SphereCenter are both actor-local � direct distance.
            ParallelFor(NumChunks, [&](int32 idx)
                {
                    if (Self->InitGeneration != CapturedGen) return;
                    TSharedPtr<FOceanQuadTreeNode> ChunkNode = ChunkNodes[idx];
                    if (!ChunkNode.IsValid()) return;

                    double DistSq = FMath::Max(FVector::DistSquared(ChunkNode->SphereCenter, Predicted), 1e-12);

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

            // Density push-down: for any chunk where the tree structure changed,
            // walk it and fill in CornerDensities for any newly-split children.
            // Serial, safe — no other thread touches the tree at this point.
            for (int32 idx = 0; idx < NumChunks; ++idx)
            {
                if (ChunkTreeChanged[idx])
                    Self->PushDensityToChildren(ChunkNodes[idx]);
            }

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
        if (Quad >= Cur->Children.Num()) return Cur;
        Cur = Cur->Children[Quad];
        if (!Cur.IsValid()) return nullptr;
    }
    return Cur;
}