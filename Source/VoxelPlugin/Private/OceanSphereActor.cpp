#include "OceanSphereActor.h"
#include "FOceanQuadTreeNode.h"
#include "FDensitySampleCompositor.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

AOceanSphereActor::AOceanSphereActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    // Default scale = ocean radius matching APlanetActor defaults:
    // PlanetRadius(100M) + SeaLevel(0.5) * NoiseAmplitude(25M) = 112,500,000 cm
    SetActorScale3D(FVector(112500000.0));

    MeshAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("MeshAttachmentRoot"));
    MeshAttachmentRoot->SetupAttachment(GetRootComponent());
    MeshAttachmentRoot->SetAbsolute(false, false, true);
}

void AOceanSphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (bIsInitializing) return;
    if (!GetWorld() || GetWorld()->IsPreviewWorld() || !bTickInEditor) return;
    if (!bInitialized
        || !GetActorScale3D().Equals(LastInitScale, 0.01)
        || LastTerrainPlanetRadius != TerrainPlanetRadius)
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

// ---------------------------------------------------------------------------
// Mesh grid cache
// ---------------------------------------------------------------------------

const FOceanMeshGrid& AOceanSphereActor::GetMeshGrid(int32 Res)
{
    FOceanMeshGrid* Existing = MeshGridCache.Find(Res);
    if (Existing) return *Existing;

    FOceanMeshGrid& Grid = MeshGridCache.Add(Res);
    Grid.Build(Res);
    return Grid;
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void AOceanSphereActor::Initialize()
{
    InitializeInternal(nullptr);
}

void AOceanSphereActor::InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
    USceneComponent* InAttachParent)
{
    // Re-parent MeshAttachmentRoot to the planet's component hierarchy
    // so mesh chunks follow the planet's position/rotation.
    if (InAttachParent && MeshAttachmentRoot)
    {
        MeshAttachmentRoot->AttachToComponent(InAttachParent,
            FAttachmentTransformRules::KeepWorldTransform);
    }

    InitializeInternal(InCompositor);
}

void AOceanSphereActor::InitializeInternal(TSharedPtr<FDensitySampleCompositor> InCompositor)
{
    bIsInitializing = true;
    bInitialized = false;
    ++InitGeneration;

    if (UWorld* W = GetWorld())
        W->GetTimerManager().ClearTimer(LodTimerHandle);

    bLodUpdateRunning = false;
    MeshGridCache.Empty();

    CleanupComponents();
    ChunkMap.Empty();
    for (int32 i = 0; i < 6; ++i)
        RootNodes[i].Reset();

    double ActorRadius = GetActorScale3D().GetMax();
    OceanRadius = ActorRadius;
    double TerrainNoiseAmplitude = TerrainPlanetRadius * NoiseAmplitudeRatio;

    // Compute MaxDepth from TargetPrecision.
    {
        double EffectiveRes = FMath::Max((double)(FaceResolution - 1), 1.0);
        double Ratio = 2.0 * OceanRadius / (EffectiveRes * TargetPrecision);
        int32 IdealDepth = (int32)FMath::CeilToInt(FMath::Log2(Ratio));
        MaxDepth = FMath::Clamp(IdealDepth, MinDepth, MaxKeyDepth);
        ActualPrecision = 2.0 * OceanRadius / (EffectiveRes * FMath::Pow(2.0, (double)MaxDepth));
    }

    if (InCompositor.IsValid())
    {
        // Use planet-provided compositor
        Compositor = InCompositor;
    }
    else
    {
        // Standalone — build our own noise + compositor
        Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");

        auto HeightmapLayer = [NoiseNode = Noise, PlanetRadius = TerrainPlanetRadius, NoiseAmplitude = TerrainNoiseAmplitude]
        (const FSampleInput& Input, float* DensityOut)
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
    }

    const double Size = 1000.0;
    const double HalfSize = Size * 0.5;

    const FVector FaceCenters[6] = {
        FVector(HalfSize,        0,        0),
        FVector(-HalfSize,        0,        0),
        FVector(0,  HalfSize,        0),
        FVector(0, -HalfSize,        0),
        FVector(0,        0,  HalfSize),
        FVector(0,        0, -HalfSize),
    };

    for (int32 i = 0; i < 6; ++i)
    {
        RootNodes[i] = MakeShared<FOceanQuadTreeNode>(
            this, FCubeTransform::FaceTransforms[i], FQuadIndex((uint8)i),
            FaceCenters[i], Size, OceanRadius, MinDepth, MaxDepth, ChunkDepth);
        RootNodes[i]->ChunkAnchorCenter = RootNodes[i]->SphereCenter;
    }

    for (int32 i = 0; i < 6; ++i)
        RootNodes[i]->SplitToDepth(ChunkDepth);

    PopulateChunks();

    LastInitScale = GetActorScale3D();
    LastTerrainPlanetRadius = TerrainPlanetRadius;
    bInitialized = true;
    bIsInitializing = false;
    RunLodUpdateTask();
}

// ---------------------------------------------------------------------------
// PopulateChunks
// ---------------------------------------------------------------------------

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

    // Pre-build the grid cache before parallel work to avoid thread-unsafe lazy init.
    GetMeshGrid(FaceResolution);

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

// ---------------------------------------------------------------------------
// RebuildChunkStreamData
//
// Generates vertex positions/normals/UVs/depths on the fly from node topology.
// Triangle indices come from the static FOceanMeshGrid cache — no per-node
// mesh arrays, no GenerateMeshData, no HasGenerated flag.
//
// Per-leaf vertex layout: (Res+2) x (Res+2) extended grid, row-major ix*ExtRes+iy.
// SpherePos  = (CubeCenter + CubeOffset).GetSafeNormal() * OceanRadius
// LocalPos   = SpherePos - ChunkAnchorCenter
// ---------------------------------------------------------------------------

void AOceanSphereActor::RebuildChunkStreamData(
    TSharedPtr<FOceanMeshChunk>    Chunk,
    TSharedPtr<FOceanQuadTreeNode> ChunkNode)
{
    if (!Chunk.IsValid() || !ChunkNode.IsValid()) return;
    if (!Chunk->InnerMeshData.IsValid() || !Chunk->EdgeMeshData.IsValid()) return;

    AOceanSphereActor* Actor = ChunkNode->Owner;
    FDensitySampleCompositor* Comp = Actor ? Actor->GetCompositor().Get() : nullptr;

    const float TriCullThreshold = Actor ? Actor->TriangleCullDepthThreshold : -10000.f;
    const int32 Res = Actor ? Actor->FaceResolution : 3;
    const int32 ExtRes = Res + 2;
    const int32 Total = ExtRes * ExtRes;

    TArray<TSharedPtr<FOceanQuadTreeNode>> Leaves;
    FOceanQuadTreeNode::CollectLeaves(ChunkNode, Leaves);

    // ---------------------------------------------------------------------------
    // Inner stream
    // ---------------------------------------------------------------------------
    TSharedPtr<FOceanStreamData> NewInner = MakeShared<FOceanStreamData>();
    NewInner->MeshGroupKey = Chunk->InnerMeshData->MeshGroupKey;
    NewInner->MeshSectionKey = Chunk->InnerMeshData->MeshSectionKey;

    auto InnerPos = NewInner->GetPositionStream();
    auto InnerTan = NewInner->GetTangentStream();
    auto InnerTex = NewInner->GetTexCoordStream();
    auto InnerCol = NewInner->GetColorStream();
    auto InnerTri = NewInner->GetTriangleStream();
    auto InnerPG = NewInner->GetPolygroupStream();

    // ---------------------------------------------------------------------------
    // Edge stream
    // ---------------------------------------------------------------------------
    TSharedPtr<FOceanStreamData> NewEdge = MakeShared<FOceanStreamData>();
    NewEdge->MeshGroupKey = Chunk->EdgeMeshData->MeshGroupKey;
    NewEdge->MeshSectionKey = Chunk->EdgeMeshData->MeshSectionKey;

    auto EdgePos = NewEdge->GetPositionStream();
    auto EdgeTan = NewEdge->GetTangentStream();
    auto EdgeTex = NewEdge->GetTexCoordStream();
    auto EdgeCol = NewEdge->GetColorStream();
    auto EdgeTri = NewEdge->GetTriangleStream();
    auto EdgePG = NewEdge->GetPolygroupStream();

    // ---------------------------------------------------------------------------
    // Per-leaf: generate vertex data on the fly, emit triangles from grid cache.
    // Triangles where all 3 vertices are above water (depth < 0) are culled.
    // ---------------------------------------------------------------------------
    for (auto& Leaf : Leaves)
    {
        const double Step = Leaf->Size / (double)(Res - 1);

        // Build vertex data for this leaf's (Res+2)x(Res+2) extended grid.
        TArray<FVector>   LeafPos;   LeafPos.SetNumUninitialized(Total);
        TArray<FVector3f> LeafNorm;  LeafNorm.SetNumUninitialized(Total);
        TArray<float>     LeafDepth; LeafDepth.SetNumUninitialized(Total);
        TArray<FColor>    LeafColor; LeafColor.SetNumUninitialized(Total);

        // Stage 1: positions, normals, sphere positions for sampling
        TArray<float> SX, SY, SZ;
        SX.SetNumUninitialized(Total);
        SY.SetNumUninitialized(Total);
        SZ.SetNumUninitialized(Total);

        for (int32 ix = 0; ix < ExtRes; ++ix)
        {
            for (int32 iy = 0; iy < ExtRes; ++iy)
            {
                double normX = -Leaf->HalfSize + Step * (ix - 1);
                double normY = -Leaf->HalfSize + Step * (iy - 1);

                FVector CubeOffset = FVector::ZeroVector;
                CubeOffset[Leaf->FaceTransform.AxisMap[0]] = Leaf->FaceTransform.AxisDir[0] * normX;
                CubeOffset[Leaf->FaceTransform.AxisMap[1]] = Leaf->FaceTransform.AxisDir[1] * normY;

                FVector SpherePos = (Leaf->CubeCenter + CubeOffset).GetSafeNormal() * Leaf->OceanRadius;
                int32   vi = ix * ExtRes + iy;

                LeafPos[vi] = SpherePos - Leaf->ChunkAnchorCenter;
                FVector Normal = SpherePos.GetSafeNormal();
                LeafNorm[vi] = FVector3f(Normal);

                SX[vi] = (float)SpherePos.X;
                SY[vi] = (float)SpherePos.Y;
                SZ[vi] = (float)SpherePos.Z;
            }
        }

        // Stage 2: batch compositor sample → depths + encoded colors
        if (Comp)
        {
            TArray<float> DensityOut;
            DensityOut.SetNumUninitialized(Total);
            FSampleInput Input(SX.GetData(), SY.GetData(), SZ.GetData(), Total);
            Comp->Sample(Input, DensityOut.GetData());

            float NewMaxDepth = -FLT_MAX;
            for (int32 i = 0; i < Total; ++i)
            {
                float depth = DensityOut[i];
                NewMaxDepth = FMath::Max(NewMaxDepth, depth);
                LeafDepth[i] = depth;
                float absDepth = FMath::Max(depth, 0.0f);
                uint32 id = (uint32)FMath::Min(absDepth * 1000.0f, 4294967040.0f);
                LeafColor[i] = FColor((id >> 24) & 0xFF, (id >> 16) & 0xFF, (id >> 8) & 0xFF, id & 0xFF);
            }
            Leaf->MaxVertexDepth = NewMaxDepth;
        }
        else
        {
            for (int32 i = 0; i < Total; ++i)
            {
                LeafDepth[i] = 0.f;
                LeafColor[i] = FColor(0, 0, 0, 0);
            }
            Leaf->MaxVertexDepth = 0.f;
        }

        // Stage 3: emit inner triangles — cull if all 3 verts above water
        const FOceanMeshGrid& Grid = Actor->GetMeshGrid(Res);
        const bool bFlip = Leaf->FaceTransform.bFlipWinding;

        for (FIndex3UI Tri : Grid.PatchTriangles)
        {
            if (bFlip)
                Tri = FIndex3UI(Tri.V0, Tri.V2, Tri.V1);

            if (LeafDepth[Tri.V0] < TriCullThreshold && LeafDepth[Tri.V1] < TriCullThreshold && LeafDepth[Tri.V2] < TriCullThreshold)
                continue;

            for (int32 v = 0; v < 3; ++v)
            {
                uint32 vi = (v == 0) ? Tri.V0 : (v == 1) ? Tri.V1 : Tri.V2;
                InnerPos.Add(LeafPos[vi]);
                FRealtimeMeshTangentsHighPrecision T; T.SetNormal(LeafNorm[vi]);
                InnerTan.Add(T);
                InnerTex.Add(FVector2f(LeafDepth[vi], 0.f));
                InnerCol.Add(LeafColor[vi]);
            }
            uint32 base = (uint32)InnerPos.Num() - 3;
            InnerTri.Add(FIndex3UI(base, base + 1, base + 2));
            InnerPG.Add(0);
        }

        // Stage 4: emit edge stitch triangles — same per-tri cull
        uint8 EdgeFlags =
            ((Leaf->GetDepth() > Leaf->NeighborLods[(uint8)EdgeOrientation::LEFT]) ? 0x1 : 0) |
            ((Leaf->GetDepth() > Leaf->NeighborLods[(uint8)EdgeOrientation::RIGHT]) ? 0x2 : 0) |
            ((Leaf->GetDepth() > Leaf->NeighborLods[(uint8)EdgeOrientation::UP]) ? 0x4 : 0) |
            ((Leaf->GetDepth() > Leaf->NeighborLods[(uint8)EdgeOrientation::DOWN]) ? 0x8 : 0);

        for (FIndex3UI Tri : Grid.EdgeTriangles[EdgeFlags])
        {
            if (bFlip)
                Tri = FIndex3UI(Tri.V0, Tri.V2, Tri.V1);

            if (LeafDepth[Tri.V0] < TriCullThreshold && LeafDepth[Tri.V1] < TriCullThreshold && LeafDepth[Tri.V2] < TriCullThreshold)
                continue;

            for (int32 v = 0; v < 3; ++v)
            {
                uint32 vi = (v == 0) ? Tri.V0 : (v == 1) ? Tri.V1 : Tri.V2;
                EdgePos.Add(LeafPos[vi]);
                FRealtimeMeshTangentsHighPrecision T; T.SetNormal(LeafNorm[vi]);
                EdgeTan.Add(T);
                EdgeTex.Add(FVector2f(LeafDepth[vi], 0.f));
                EdgeCol.Add(LeafColor[vi]);
            }
            uint32 base = (uint32)EdgePos.Num() - 3;
            EdgeTri.Add(FIndex3UI(base, base + 1, base + 2));
            EdgePG.Add(0);
        }
    }

    Chunk->InnerMeshData = NewInner;
    Chunk->EdgeMeshData = NewEdge;
    Chunk->IsDirty = true;
}

// ---------------------------------------------------------------------------
// CleanupComponents
// ---------------------------------------------------------------------------

void AOceanSphereActor::CleanupComponents()
{
    if (!MeshAttachmentRoot) return;
    TArray<USceneComponent*> Attached;
    MeshAttachmentRoot->GetChildrenComponents(false, Attached);
    for (USceneComponent* Child : Attached)
        if (URealtimeMeshComponent* RtComp = Cast<URealtimeMeshComponent>(Child))
            RtComp->DestroyComponent();
}

// ---------------------------------------------------------------------------
// Tick
// ---------------------------------------------------------------------------

void AOceanSphereActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!bInitialized) return;

    if (UWorld* W = GetWorld())
    {
        const TArray<FVector>& Views = W->ViewLocationsRenderedLastFrame;
        if (Views.Num() > 0)
        {
            FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
            CameraPosition = NoScaleTransform.InverseTransformPosition(Views[0]);
        }
        APlayerCameraManager* Cam = UGameplayStatics::GetPlayerCameraManager(W, 0);
        if (Cam) CameraFOV = Cam->GetFOVAngle();
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
            double MergeThresholdSq = (Self->ScreenSpaceThreshold * 0.5) * (Self->ScreenSpaceThreshold * 0.5);

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

            // Phase 1: LOD pass
            ParallelFor(NumChunks, [&](int32 idx)
                {
                    if (Self->InitGeneration != CapturedGen) return;
                    TSharedPtr<FOceanQuadTreeNode> ChunkNode = ChunkNodes[idx];
                    if (!ChunkNode.IsValid()) return;

                    // Back-face cull: skip chunks on the far side of the planet.
                    // If the chunk center and camera are on opposite sides of the origin,
                    // and the angle between them exceeds ~78 degrees, skip LOD work.
                    double Dot = FVector::DotProduct(Predicted, ChunkNode->SphereCenter);
                    if (Dot < 0.0)
                    {
                        double CamDistSq = Predicted.SizeSquared();
                        double NodeDistSq = ChunkNode->SphereCenter.SizeSquared();
                        if ((Dot * Dot) > (0.04 * CamDistSq * NodeDistSq))
                        {
                            FOceanQuadTreeNode::CollectLeaves(ChunkNode, AllChunkLeaves[idx]);
                            return;
                        }
                    }

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
                        if (Leaf->TrySetLod(Predicted, ThresholdSq, MergeThresholdSq, FOVScale))
                            bAnyChanged = true;

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

            // Phase 2: Neighbor stitch — CheckNeighbors now has no HasGenerated guard
            TArray<bool> ChunkEdgeChanged;
            ChunkEdgeChanged.SetNumZeroed(NumChunks);

            ParallelFor(NumChunks, [&](int32 idx)
                {
                    if (Self->InitGeneration != CapturedGen) return;
                    bool bAnyEdgeChanged = false;
                    for (auto& Leaf : AllChunkLeaves[idx])
                        if (Leaf->CheckNeighbors())
                            bAnyEdgeChanged = true;
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
                if (MeshChunks[idx]->IsDirty)
                    DirtyChunks.Add(MeshChunks[idx]);

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

                        auto* InnerTriStream = Chunk->InnerMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles);
                        if (InnerTriStream && InnerTriStream->Num() > 0)
                        {
                            MeshPtr->UpdateSectionGroup(Chunk->InnerMeshData->MeshGroupKey, Chunk->InnerMeshData->MeshStream);
                            FRealtimeMeshSectionConfig Config(0);
                            MeshPtr->UpdateSectionConfig(Chunk->InnerMeshData->MeshSectionKey, Config, true);
                        }
                        else
                            MeshPtr->UpdateSectionGroup(Chunk->InnerMeshData->MeshGroupKey, FRealtimeMeshStreamSet());

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
                        W->GetTimerManager().SetTimer(
                            Self->LodTimerHandle, Self,
                            &AOceanSphereActor::RunLodUpdateTask,
                            (float)Self->MinLodInterval, false);
                });

        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

// ---------------------------------------------------------------------------
// GetNodeByIndex
// ---------------------------------------------------------------------------

TSharedPtr<FOceanQuadTreeNode> AOceanSphereActor::GetNodeByIndex(const FQuadIndex& Index) const
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