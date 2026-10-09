#include "VoxelCollisionBricks.h"
#include "VoxelMeshStream.h"
#include "Misc/ScopeLock.h"
#include "FDensitySampleCompositor.h"
#include "FSparseEditStore.h"
#include "Async/ParallelFor.h"
#include "Async/TaskGraphInterfaces.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Components/PrimitiveComponent.h"

static TAutoConsoleVariable<int32> CVarVoxelDebugCollision(
    TEXT("voxel.Debug.Collision"), 0,
    TEXT("1: log each collision task and outline collision bricks (green: has collision, orange: waiting for a rebuild)."),
    ECVF_Default);

namespace
{
    constexpr double TaskInterval = 0.1;            // seconds between collision tasks
    constexpr int32 MaxCreatesPerTick = 16;         // component creations per frame
    constexpr int64 MaxSamplesPerTask = 3000000;    // density samples one task may spend on meshing
    constexpr int32 RootBlockBricks = 8;            // bricks per axis in the descent's top blocks
    constexpr int32 MeshBatch = 32;                 // bricks meshed per parallel batch

    const FName CollisionGroupName(TEXT("COLLISION"));

    FRealtimeMeshCollisionConfiguration MakeCollisionConfig()
    {
        FRealtimeMeshCollisionConfiguration Config;
        Config.bUseComplexAsSimpleCollision = true;
        Config.bUseAsyncCook = true;
        Config.bShouldFastCookMeshes = false;
        Config.bDeformableMesh = false;
        return Config;
    }

    double BoxDistanceSquared(const FBox& Box, const FVector& P)
    {
        double D = 0.0;
        for (int32 Axis = 0; Axis < 3; Axis++)
        {
            const double Below = Box.Min[Axis] - P[Axis];
            const double Above = P[Axis] - Box.Max[Axis];
            const double Out = FMath::Max3(Below, Above, 0.0);
            D += Out * Out;
        }
        return D;
    }

    /** Smallest squared distance from Box to a source, relative to that source's radius
     *  (<= 0 when inside some source's sphere). */
    double NearestSource(const FBox& Box, const TArray<FVoxelCollisionSource>& Sources, double RadiusScale, double& OutDistance)
    {
        double Best = TNumericLimits<double>::Max();
        OutDistance = TNumericLimits<double>::Max();
        for (const FVoxelCollisionSource& Source : Sources)
        {
            const double DistSq = BoxDistanceSquared(Box, Source.Position);
            const double R = Source.Radius * RadiusScale;
            Best = FMath::Min(Best, DistSq - R * R);
            OutDistance = FMath::Min(OutDistance, FMath::Sqrt(DistSq));
        }
        return Best;
    }

    FBox CellBounds(const FVoxelBrickGrid& Grid, const FInt64Vector& Min, int64 SizeInBricks)
    {
        const FVector Lo = Grid.BrickMin(Min);
        return FBox(Lo, Lo + FVector(Grid.BrickSize() * (double)SizeInBricks));
    }

    int64 FloorDiv(int64 A, int64 B)
    {
        return A >= 0 ? A / B : -((-A + B - 1) / B);
    }

    struct FCandidate
    {
        FInt64Vector Coord;
        double Distance;
        bool bEdited;
    };

    /** Octree-like descent over blocks of bricks: a block is dropped when it is outside every
     *  source or its centre sample proves its dependency region free of surface (unless edits
     *  reach it). Surviving single bricks that are not Known become candidates. */
    void Descend(const FDensitySampleCompositor& Density, const FSparseEditStore* Edits, const FVoxelBrickGrid& Grid,
        const TArray<FVoxelCollisionSource>& Sources, const TSet<FInt64Vector>& Known, double Slope,
        const FInt64Vector& Min, int64 Size, TArray<FCandidate>& Out, int64& InOutSamples)
    {
        double Distance;
        if (NearestSource(CellBounds(Grid, Min, Size), Sources, 1.0, Distance) > 0.0) return;
        if (Size == 1 && Known.Contains(Min)) return;

        const double H = Grid.CellSize;
        const FBox Region(Grid.BrickMin(Min) - FVector(2.0 * H), Grid.BrickMin(Min) + FVector(Grid.BrickSize() * (double)Size + H));
        const bool bEdited = Edits && Edits->HasEditsInBox(Region);
        if (!bEdited)
        {
            const FVector Centre = Region.GetCenter();
            double CX = Centre.X, CY = Centre.Y, CZ = Centre.Z;
            float D = 0.0f;
            Density.Sample(FSampleInput(&CX, &CY, &CZ, 1, H), &D);
            InOutSamples++;
            if (FMath::Abs((double)D) > Slope * Region.GetExtent().Size()) return;
        }

        if (Size == 1)
        {
            Out.Add({ Min, Distance, bEdited });
            return;
        }
        const int64 Half = Size / 2;
        for (int32 c = 0; c < 8; c++)
        {
            const FInt64Vector Child(Min.X + ((c & 1) ? Half : 0), Min.Y + ((c & 2) ? Half : 0), Min.Z + ((c & 4) ? Half : 0));
            Descend(Density, Edits, Grid, Sources, Known, Slope, Child, Half, Out, InOutSamples);
        }
    }

    TSharedPtr<FMeshStreamData> BuildStream(const FVoxelBrickMesh& Mesh)
    {
        if (Mesh.Indices.Num() == 0) return nullptr;

        TSharedPtr<FMeshStreamData> Data = MakeShared<FMeshStreamData>();
        Data->MeshGroupKey = FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), CollisionGroupName);
        Data->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(Data->MeshGroupKey, 0);

        const int32 NumVerts = Mesh.Positions.Num();
        const int32 NumTris = Mesh.Indices.Num() / 3;
        auto Positions = Data->GetPositionStream();
        auto Tangents = Data->GetTangentStream();
        auto Colors = Data->GetColorStream();
        auto TexCoords = Data->GetTexCoordStream();
        auto Triangles = Data->GetTriangleStream();
        auto PolyGroups = Data->GetPolygroupStream();
        Positions.SetNumUninitialized(NumVerts);
        Tangents.SetNumUninitialized(NumVerts);
        Colors.SetNumUninitialized(NumVerts);
        TexCoords.SetNumUninitialized(NumVerts);
        Triangles.SetNumUninitialized(NumTris);
        PolyGroups.SetNumUninitialized(NumTris);

        for (int32 i = 0; i < NumVerts; i++)
        {
            Positions.Set(i, FVector(Mesh.Positions[i]));
            FRealtimeMeshTangentsHighPrecision Tangent;
            Tangent.SetNormal(Mesh.Normals[i]);
            Tangents.Set(i, Tangent);
            Colors.Set(i, FColor::White);
            TexCoords.Set(i, FVector2f::ZeroVector);
        }
        for (int32 t = 0; t < NumTris; t++)
        {
            Triangles.Set(t, FIndex3UI((uint32)Mesh.Indices[3 * t], (uint32)Mesh.Indices[3 * t + 1], (uint32)Mesh.Indices[3 * t + 2]));
            PolyGroups.Set(t, 0);
        }
        return Data;
    }
}

void FVoxelCollisionBricks::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, TSharedPtr<FDensitySampleCompositor> InDensity,
    const FVoxelBrickGrid& InGrid, double InSlopeBound)
{
    Reset();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Density = InDensity;
    Grid = InGrid;
    Grid.CellsPerBrick = FMath::Max(4, Grid.CellsPerBrick / 4 * 4);
    SlopeBound = FMath::Max(InSlopeBound, 1.0);
    bWarnedSlope = false;
}

void FVoxelCollisionBricks::Reset()
{
    for (TPair<FInt64Vector, FBrick>& Pair : Bricks)
        DestroyComponent(Pair.Value);
    Bricks.Reset();
    ApplyQueue.Reset();
    RecentEdits.Reset();
    Generation++;
    LastLaunchTime = 0.0;
    FScopeLock Lock(&ResultCS);
    Results.Reset();
}

void FVoxelCollisionBricks::NotifyEdit(uint32 Sequence, const FVector& Center, double Radius)
{
    // The edit store spreads a brush over the nodes it overlaps, which can reach up to one
    // node size past the brush; twice the radius covers it.
    const double Reach = 2.0 * Radius + 2.0 * Grid.CellSize;
    const FBox Bounds(Center - FVector(Reach), Center + FVector(Reach));
    RecentEdits.Add({ Sequence, Bounds });

    for (TPair<FInt64Vector, FBrick>& Pair : Bricks)
    {
        if (Grid.DependencyBounds(Pair.Key).Intersect(Bounds))
            Pair.Value.bDirty = true;
    }
}

void FVoxelCollisionBricks::Tick(const TArray<FVoxelCollisionSource>& Sources, double Hysteresis)
{
    DrainResults();
    ApplyQueued();

    const double Now = FPlatformTime::Seconds();
    if (bTaskRunning.load(std::memory_order_acquire) || Now - LastLaunchTime < TaskInterval) return;

    RemoveFar(Sources, Hysteresis);
    if (Sources.Num() == 0 || !Density.IsValid()) return;

    LastLaunchTime = Now;
    LaunchTask(Sources);
}

void FVoxelCollisionBricks::RemoveFar(const TArray<FVoxelCollisionSource>& Sources, double Hysteresis)
{
    const double Scale = FMath::Max(Hysteresis, 1.0);
    for (auto It = Bricks.CreateIterator(); It; ++It)
    {
        double Distance;
        if (Sources.Num() == 0 || NearestSource(CellBounds(Grid, It.Key(), 1), Sources, Scale, Distance) > 0.0)
        {
            DestroyComponent(It.Value());
            It.RemoveCurrent();
        }
    }
    ApplyQueue.RemoveAll([this](const FInt64Vector& Coord) { return !Bricks.Contains(Coord); });
}

void FVoxelCollisionBricks::LaunchTask(const TArray<FVoxelCollisionSource>& Sources)
{
    FTaskInput Input;
    Input.Generation = Generation;
    Input.Sources = Sources;
    Input.Density = Density;
    Input.Grid = Grid;
    Input.SlopeBound = SlopeBound;
    Input.Known.Reserve(Bricks.Num());
    // Every brick still held is within the drop distance; a dirty one is rebuilt even
    // outside the build radius, so no stale collision stays up.
    for (TPair<FInt64Vector, FBrick>& Pair : Bricks)
    {
        Input.Known.Add(Pair.Key);
        if (!Pair.Value.bDirty) continue;
        Pair.Value.bDirty = false;
        Input.Rebuild.Add(Pair.Key);
    }

    bTaskRunning.store(true, std::memory_order_release);
    TSharedRef<FVoxelCollisionBricks> Self = AsShared();
    FFunctionGraphTask::CreateAndDispatchWhenReady([Self, Input = MoveTemp(Input)]() mutable
        {
            RunTask(Self, MoveTemp(Input));
        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadNormalTask);
}

void FVoxelCollisionBricks::RunTask(const TSharedRef<FVoxelCollisionBricks>& Self, FTaskInput Input)
{
    const double T0 = FPlatformTime::Seconds();
    FTaskResult Result;
    Result.Generation = Input.Generation;

    const FDensitySampleCompositor& Dens = *Input.Density;
    const TSharedPtr<FSparseEditStore> Edits = Input.Density->GetEditStore();
    Result.StartSequence = Edits.IsValid() ? Edits->GetLastSequence() : 0;

    // Candidates: bricks to rebuild first, then new bricks that may hold surface, nearest first.
    TArray<FCandidate> Candidates;
    for (const FInt64Vector& Coord : Input.Rebuild)
    {
        const bool bEdited = Edits.IsValid() && Edits->HasEditsInBox(Input.Grid.DependencyBounds(Coord));
        Candidates.Add({ Coord, -1.0, bEdited });
    }
    {
        TArray<FCandidate> Found;
        TSet<FInt64Vector> Roots;
        for (const FVoxelCollisionSource& Source : Input.Sources)
        {
            const FInt64Vector Lo = Input.Grid.BrickAt(Source.Position - FVector(Source.Radius));
            const FInt64Vector Hi = Input.Grid.BrickAt(Source.Position + FVector(Source.Radius));
            for (int64 Z = FloorDiv(Lo.Z, RootBlockBricks); Z <= FloorDiv(Hi.Z, RootBlockBricks); Z++)
                for (int64 Y = FloorDiv(Lo.Y, RootBlockBricks); Y <= FloorDiv(Hi.Y, RootBlockBricks); Y++)
                    for (int64 X = FloorDiv(Lo.X, RootBlockBricks); X <= FloorDiv(Hi.X, RootBlockBricks); X++)
                    {
                        const FInt64Vector Root(X * RootBlockBricks, Y * RootBlockBricks, Z * RootBlockBricks);
                        bool bSeen = false;
                        Roots.Add(Root, &bSeen);
                        if (!bSeen)
                            Descend(Dens, Edits.Get(), Input.Grid, Input.Sources, Input.Known, Input.SlopeBound, Root, RootBlockBricks, Found, Result.Samples);
                    }
        }
        Found.Sort([](const FCandidate& A, const FCandidate& B) { return A.Distance < B.Distance; });
        Candidates.Append(Found);
    }
    Result.NumCandidates = Candidates.Num();

    // Mesh in batches until the sample budget is spent; new bricks left over are found again
    // next task. Rebuilds are never left over: nothing would mark them dirty again.
    std::atomic<int64> Samples{ 0 };
    std::atomic<int32> Skipped{ 0 };
    float MaxGradient = 0.0f;
    FCriticalSection GradientCS;
    const int32 NumRebuild = Input.Rebuild.Num();
    for (int32 Start = 0; Start < Candidates.Num() && (Start < NumRebuild || Samples.load() < MaxSamplesPerTask); Start += MeshBatch)
    {
        const int32 Count = FMath::Min(MeshBatch, Candidates.Num() - Start);
        TArray<FBuilt> Batch;
        Batch.SetNum(Count);
        ParallelFor(Count, [&](int32 i)
            {
                const FCandidate& Candidate = Candidates[Start + i];
                FVoxelBrickMesh Mesh;
                FVoxelBrickStats Stats;
                VoxelBrickMesher::MeshBrick(Dens, Input.Grid, Candidate.Coord, Input.SlopeBound, Candidate.bEdited, Mesh, Stats);
                Batch[i].Coord = Candidate.Coord;
                Batch[i].Stream = BuildStream(Mesh);
                Batch[i].bSkipped = Stats.bSkipped;
                Samples.fetch_add(Stats.Samples, std::memory_order_relaxed);
                if (Stats.bSkipped) Skipped.fetch_add(1, std::memory_order_relaxed);
                if (!Candidate.bEdited && Stats.MaxGradient > 0.0f)
                {
                    FScopeLock Lock(&GradientCS);
                    MaxGradient = FMath::Max(MaxGradient, Stats.MaxGradient);
                }
            });
        Result.Built.Append(MoveTemp(Batch));
    }
    Result.Samples += Samples.load();
    Result.NumSkipped = Skipped.load();
    Result.MaxGradient = MaxGradient;
    Result.Milliseconds = (FPlatformTime::Seconds() - T0) * 1000.0;

    {
        FScopeLock Lock(&Self->ResultCS);
        Self->Results.Add(MoveTemp(Result));
    }
    Self->bTaskRunning.store(false, std::memory_order_release);
}

void FVoxelCollisionBricks::DrainResults()
{
    TArray<FTaskResult> Ready;
    {
        FScopeLock Lock(&ResultCS);
        Ready = MoveTemp(Results);
        Results.Reset();
    }

    const bool bDebug = CVarVoxelDebugCollision.GetValueOnGameThread() > 0;
    for (FTaskResult& Result : Ready)
    {
        if (Result.Generation != Generation) continue;

        int32 NumWithSurface = 0;
        for (FBuilt& Built : Result.Built)
        {
            FBrick& Brick = Bricks.FindOrAdd(Built.Coord);
            Brick.Pending = MoveTemp(Built.Stream);
            Brick.bSkipped = Built.bSkipped;
            NumWithSurface += Brick.Pending.IsValid() ? 1 : 0;
            if (!Brick.bHasPending)
            {
                Brick.bHasPending = true;
                ApplyQueue.Add(Built.Coord);
            }

            // An edit published after the task read the edit store isn't in this mesh.
            const FBox Bounds = Grid.DependencyBounds(Built.Coord);
            for (const FRecentEdit& Edit : RecentEdits)
            {
                if (Edit.Sequence > Result.StartSequence && Edit.Bounds.Intersect(Bounds))
                    Brick.bDirty = true;
            }
        }
        RecentEdits.RemoveAll([&Result](const FRecentEdit& Edit) { return Edit.Sequence <= Result.StartSequence; });

        // The empty test assumes |grad density| <= SlopeBound; a steeper surface than that
        // could hide a brick, so the bound follows what the meshed bricks show.
        if (Result.MaxGradient > SlopeBound)
        {
            if (!bWarnedSlope)
            {
                UE_LOG(LogTemp, Warning, TEXT("[VoxelCollision] Surface gradient %.2f exceeds CollisionSlopeBound %.2f; raising the bound to %.2f. Raise CollisionSlopeBound for these surface settings."),
                    Result.MaxGradient, SlopeBound, Result.MaxGradient * 1.25);
                bWarnedSlope = true;
            }
            SlopeBound = Result.MaxGradient * 1.25;

            // Bricks proven empty under the old bound are tested again under the new one.
            for (auto It = Bricks.CreateIterator(); It; ++It)
            {
                if (It.Value().bSkipped && !It.Value().Component.IsValid())
                    It.RemoveCurrent();
            }
        }

        if (bDebug)
        {
            UE_LOG(LogTemp, Log, TEXT("[VoxelCollision] %d candidates, %d meshed (%d with surface, %d proven empty), %lld samples, %.1f ms, %d bricks known"),
                Result.NumCandidates, Result.Built.Num(), NumWithSurface, Result.NumSkipped, Result.Samples, Result.Milliseconds, Bricks.Num());
        }
    }
}

void FVoxelCollisionBricks::ApplyQueued()
{
    int32 Creates = 0;
    int32 Kept = 0;
    for (int32 i = 0; i < ApplyQueue.Num(); i++)
    {
        const FInt64Vector Coord = ApplyQueue[i];
        FBrick* Brick = Bricks.Find(Coord);
        if (!Brick || !Brick->bHasPending) continue;

        if (!Brick->Pending.IsValid())
        {
            // No surface: nothing to collide with.
            DestroyComponent(*Brick);
            Brick->bHasPending = false;
            continue;
        }

        if (!Brick->Component.IsValid() || !Brick->Mesh.IsValid())
        {
            DestroyComponent(*Brick);
            if (Creates >= MaxCreatesPerTick)
            {
                ApplyQueue[Kept++] = Coord;
                continue;
            }
            Creates++;
            if (!EnsureComponent(Coord, *Brick))
            {
                Brick->bHasPending = false;
                continue;
            }
        }

        URealtimeMeshSimple* Mesh = Brick->Mesh.Get();
        const TSharedPtr<FMeshStreamData> Data = MoveTemp(Brick->Pending);
        Brick->bHasPending = false;
        Brick->NumTriangles = Data->NumTriangles();
        Mesh->UpdateBufferSet(Data->MeshGroupKey, Data->MeshStream);
        if (!Brick->bHasSection)
        {
            Mesh->UpdateSectionConfig(Data->MeshSectionKey, FRealtimeMeshSectionConfig(0), true);
            Brick->bHasSection = true;
        }

        // The stream update re-cooks the section's collision; SetCollisionConfig joins that
        // cook and returns its future, so the brick shows as waiting until the new body is in.
        if (!Brick->CooksInFlight.IsValid())
            Brick->CooksInFlight = MakeShared<std::atomic<int32>>(0);
        if (!Brick->LastCookResult.IsValid())
            Brick->LastCookResult = MakeShared<std::atomic<int32>>(-1);
        Brick->CooksInFlight->fetch_add(1);
        Brick->NumApplies++;
        const TSharedPtr<std::atomic<int32>> InFlight = Brick->CooksInFlight;
        const TSharedPtr<std::atomic<int32>> LastResult = Brick->LastCookResult;
        const bool bDebug = CVarVoxelDebugCollision.GetValueOnGameThread() > 0;
        const FInt64Vector BrickCoord = Coord;
        Mesh->SetCollisionConfig(MakeCollisionConfig()).Next([InFlight, LastResult, bDebug, BrickCoord](ERealtimeMeshCollisionUpdateResult Result)
            {
                InFlight->fetch_sub(1);
                LastResult->store((int32)Result);
                if (bDebug && Result != ERealtimeMeshCollisionUpdateResult::Updated && Result != ERealtimeMeshCollisionUpdateResult::Ignored)
                {
                    UE_LOG(LogTemp, Warning, TEXT("[VoxelCollision] Cook of brick (%lld, %lld, %lld) failed (result %d)"),
                        BrickCoord.X, BrickCoord.Y, BrickCoord.Z, (int32)Result);
                }
            });
    }
    ApplyQueue.SetNum(Kept);
}

bool FVoxelCollisionBricks::EnsureComponent(const FInt64Vector& Coord, FBrick& Brick)
{
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!OwnerActor || !Root) return false;

    // Transient: built at run time, never saved with the level or copied into PIE.
    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetCollisionConfig(MakeCollisionConfig());
    Mesh->SetupMaterialSlot(0, "Collision");

    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->SetVisibility(false);
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->RegisterComponent();
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetRelativeLocation(Grid.BrickMin(Coord));
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    Component->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
    Component->SetRealtimeMesh(Mesh);

    Mesh->CreateBufferSet(FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), CollisionGroupName), FRealtimeMeshStreamSet());

    Brick.Mesh = Mesh;
    Brick.Component = Component;
    Brick.bHasSection = false;
    return true;
}

void FVoxelCollisionBricks::DestroyComponent(FBrick& Brick)
{
    if (URealtimeMeshComponent* Component = Brick.Component.Get())
        Component->DestroyComponent();
    Brick.Component = nullptr;
    Brick.Mesh = nullptr;
    Brick.bHasSection = false;
    Brick.NumTriangles = 0;
}

FString FVoxelCollisionBricks::DescribeHit(const UPrimitiveComponent* HitComponent, const FVector& LocalPoint) const
{
    auto Describe = [](const FInt64Vector& Coord, const FBrick& Brick)
        {
            return FString::Printf(TEXT("brick (%lld, %lld, %lld): %d tris, %d applies, %d cooks in flight, last cook result %d, dirty %d, pending %d"),
                Coord.X, Coord.Y, Coord.Z, Brick.NumTriangles, Brick.NumApplies,
                Brick.CooksInFlight.IsValid() ? Brick.CooksInFlight->load() : 0,
                Brick.LastCookResult.IsValid() ? Brick.LastCookResult->load() : -1,
                Brick.bDirty ? 1 : 0, Brick.bHasPending ? 1 : 0);
        };

    FString HitText = FString::Printf(TEXT("hit %s (owner %s) is not a collision brick"),
        HitComponent ? *HitComponent->GetName() : TEXT("nothing"),
        HitComponent && HitComponent->GetOwner() ? *HitComponent->GetOwner()->GetName() : TEXT("none"));
    for (const TPair<FInt64Vector, FBrick>& Pair : Bricks)
    {
        if (HitComponent && Pair.Value.Component.Get() == HitComponent)
        {
            HitText = TEXT("hit collision ") + Describe(Pair.Key, Pair.Value);
            break;
        }
    }

    const FInt64Vector Here = Grid.BrickAt(LocalPoint);
    const FBrick* Brick = Bricks.Find(Here);
    const FString HereText = Brick ? TEXT("point is in ") + Describe(Here, *Brick)
        : FString::Printf(TEXT("point is in brick (%lld, %lld, %lld), not held"), Here.X, Here.Y, Here.Z);
    return HitText + TEXT("; ") + HereText;
}

void FVoxelCollisionBricks::DrawDebug(UWorld* World, const FTransform& ToWorld) const
{
    if (!World || CVarVoxelDebugCollision.GetValueOnGameThread() <= 0) return;
    for (const TPair<FInt64Vector, FBrick>& Pair : Bricks)
    {
        const bool bHasCollision = Pair.Value.Component.IsValid() && Pair.Value.NumTriangles > 0;
        const bool bCooking = Pair.Value.CooksInFlight.IsValid() && Pair.Value.CooksInFlight->load() > 0;
        const bool bWaiting = Pair.Value.bDirty || Pair.Value.bHasPending || bCooking;
        if (!bHasCollision && !bWaiting) continue;
        const FBox Box = CellBounds(Grid, Pair.Key, 1);
        DrawDebugBox(World, ToWorld.TransformPosition(Box.GetCenter()), Box.GetExtent(), ToWorld.GetRotation(),
            bWaiting ? FColor::Orange : FColor::Green, false, -1.f, 0, 5.f);
    }
}
