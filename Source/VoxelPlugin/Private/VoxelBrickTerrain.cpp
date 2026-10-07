#include "VoxelBrickTerrain.h"
#include "FMeshingStructs.h"
#include "FDensitySampleCompositor.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctree.h"
#include "Async/ParallelFor.h"
#include "Async/TaskGraphInterfaces.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<int32> CVarVoxelDebugBricks(
    TEXT("voxel.Debug.Bricks"), 0,
    TEXT("1: log each brick terrain task (wanted, meshed, samples, time)."),
    ECVF_Default);

namespace VoxelBrickTerrainDetail
{
    constexpr double TaskInterval = 0.1;            // seconds between tasks
    constexpr int32 MaxCreatesPerTick = 32;         // component creations per frame
    constexpr int64 MaxSamplesPerTask = 4000000;    // density samples one task may spend on meshing
    constexpr int32 MeshBatch = 16;                 // bricks meshed per parallel batch
    constexpr double SkirtCells = 2.5;              // skirt depth in cells of the brick's level

    const FName RenderGroupName(TEXT("BRICK"));

    double BoxDistanceSquared(const FBox& Box, const FVector& P)
    {
        double D = 0.0;
        for (int32 Axis = 0; Axis < 3; Axis++)
        {
            const double Out = FMath::Max3(Box.Min[Axis] - P[Axis], P[Axis] - Box.Max[Axis], 0.0);
            D += Out * Out;
        }
        return D;
    }

    int64 FloorShift(int64 Value, int32 Levels)
    {
        return Value >= 0 ? (Value >> Levels) : -((-Value + (1LL << Levels) - 1) >> Levels);
    }

    FVoxelBrickGrid MakeGrid(const FVoxelBrickTerrainSettings& Settings, int32 Level)
    {
        FVoxelBrickGrid Grid;
        Grid.Origin = Settings.Origin;
        Grid.CellSize = Settings.FinestCell * FMath::Pow(2.0, (double)Level);
        Grid.CellsPerBrick = Settings.CellsPerBrick;
        return Grid;
    }

    struct FWantedBrick
    {
        FVoxelBrickKey Key;
        double Distance;
        bool bEdited;
    };

    /** The LOD descent: drops bricks proven free of surface (unless edits reach them), splits
     *  bricks the camera is close to, and collects the rest. */
    void Descend(const FDensitySampleCompositor& Density, const FSparseEditStore* Edits, const FVoxelBrickTerrainSettings& Settings,
        const FVector& Camera, const FVoxelBrickKey& Key, TArray<FWantedBrick>& Out, int64& InOutSamples)
    {
        const FVoxelBrickGrid Grid = MakeGrid(Settings, Key.Level);
        const double H = Grid.CellSize;
        const FVector Min = Grid.BrickMin(Key.Coord);
        const FBox Region(Min - FVector(2.0 * H), Min + FVector(Grid.BrickSize() + H));
        const bool bEdited = Edits && Edits->HasEditsInBox(Region);
        if (!bEdited)
        {
            const FVector Centre = Region.GetCenter();
            double CX = Centre.X, CY = Centre.Y, CZ = Centre.Z;
            float D = 0.0f;
            Density.Sample(FSampleInput(&CX, &CY, &CZ, 1, H), &D);
            InOutSamples++;
            if (FMath::Abs((double)D) > Settings.SlopeBound * Region.GetExtent().Size()) return;
        }

        const double Distance = FMath::Sqrt(BoxDistanceSquared(FBox(Min, Min + FVector(Grid.BrickSize())), Camera));
        if (Key.Level > 0 && Distance < Settings.LodDistanceFactor * Grid.BrickSize())
        {
            for (int32 c = 0; c < 8; c++)
            {
                FVoxelBrickKey Child;
                Child.Level = Key.Level - 1;
                Child.Coord = FInt64Vector(2 * Key.Coord.X + (c & 1), 2 * Key.Coord.Y + ((c >> 1) & 1), 2 * Key.Coord.Z + ((c >> 2) & 1));
                Descend(Density, Edits, Settings, Camera, Child, Out, InOutSamples);
            }
            return;
        }
        Out.Add({ Key, Distance, bEdited });
    }

    FColor LevelColor(int32 Level)
    {
        static const FColor Palette[8] = {
            FColor(230, 80, 80), FColor(240, 170, 60), FColor(230, 230, 80), FColor(110, 210, 90),
            FColor(70, 200, 200), FColor(80, 130, 240), FColor(170, 100, 230), FColor(230, 110, 190) };
        return Palette[Level % 8];
    }

    FVector2f TriplanarUV(const FVector& Position, const FVector3f& Normal, float Scale)
    {
        const FVector3f A = Normal.GetAbs();
        if (A.X > A.Y && A.X > A.Z) return FVector2f(Position.Y, Position.Z) * Scale;
        if (A.Y > A.X && A.Y > A.Z) return FVector2f(Position.X, Position.Z) * Scale;
        return FVector2f(Position.X, Position.Y) * Scale;
    }

    TSharedPtr<FMeshStreamData> BuildRenderStream(const FVoxelBrickMesh& Mesh, const FVector& BrickMin, int32 Level, float UVScale)
    {
        if (Mesh.Indices.Num() == 0) return nullptr;

        TSharedPtr<FMeshStreamData> Data = MakeShared<FMeshStreamData>();
        Data->MeshGroupKey = FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), RenderGroupName);
        Data->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(Data->MeshGroupKey, 0);

        const FColor Color = CVarVoxelDebugChunkColor.GetValueOnAnyThread() > 0 ? LevelColor(Level) : FColor::White;
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
            const FVector Local(Mesh.Positions[i]);
            Positions.Set(i, Local);
            FRealtimeMeshTangentsHighPrecision Tangent;
            Tangent.SetNormal(Mesh.Normals[i]);
            Tangents.Set(i, Tangent);
            Colors.Set(i, Color);
            TexCoords.Set(i, TriplanarUV(BrickMin + Local, Mesh.Normals[i], UVScale));
        }
        for (int32 t = 0; t < NumTris; t++)
        {
            Triangles.Set(t, FIndex3UI((uint32)Mesh.Indices[3 * t], (uint32)Mesh.Indices[3 * t + 1], (uint32)Mesh.Indices[3 * t + 2]));
            PolyGroups.Set(t, 0);
        }
        return Data;
    }
}

namespace VBT = VoxelBrickTerrainDetail;

void FVoxelBrickTerrain::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, TSharedPtr<FDensitySampleCompositor> InDensity,
    const FVoxelBrickTerrainSettings& InSettings)
{
    Reset();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Density = InDensity;
    Settings = InSettings;
    Settings.CellsPerBrick = FMath::Max(4, Settings.CellsPerBrick / 4 * 4);
    Settings.MaxLevel = FMath::Max(0, Settings.MaxLevel);
    Settings.SlopeBound = FMath::Max(Settings.SlopeBound, 1.0);
}

void FVoxelBrickTerrain::Reset()
{
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
        DestroyComponent(Pair.Value);
    Bricks.Reset();
    Wanted.Reset();
    bHasWanted = false;
    ApplyQueue.Reset();
    RecentEdits.Reset();
    Generation++;
    LastLaunchTime = 0.0;
    FScopeLock Lock(&ResultCS);
    Results.Reset();
}

FVoxelBrickGrid FVoxelBrickTerrain::GridFor(int32 Level) const
{
    return VBT::MakeGrid(Settings, Level);
}

FBox FVoxelBrickTerrain::BoundsOf(const FVoxelBrickKey& Key) const
{
    return GridFor(Key.Level).DependencyBounds(Key.Coord);
}

void FVoxelBrickTerrain::NotifyEdit(uint32 Sequence, const FVector& Center, double Radius)
{
    const double Reach = 2.0 * Radius + 2.0 * Settings.FinestCell;
    const FBox Bounds(Center - FVector(Reach), Center + FVector(Reach));
    RecentEdits.Add({ Sequence, Bounds });
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        if (BoundsOf(Pair.Key).Intersect(Bounds))
            Pair.Value.bDirty = true;
    }
}

void FVoxelBrickTerrain::Tick(const FVector& Camera)
{
    DrainResults();
    ApplyQueued();
    RemoveReplaced();

    const double Now = FPlatformTime::Seconds();
    if (bTaskRunning.load(std::memory_order_acquire) || Now - LastLaunchTime < VBT::TaskInterval || !Density.IsValid()) return;
    LastLaunchTime = Now;

    // A task that finished since the drain above must be in Known before the next starts.
    DrainResults();

    FTaskInput Input;
    Input.Generation = Generation;
    Input.Camera = Camera;
    Input.Density = Density;
    Input.Settings = Settings;
    Input.Known.Reserve(Bricks.Num());
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        // Dirty stays set until a rebuilt mesh arrives (DrainResults), so a rebuild of a brick
        // that is not wanted this pass is not lost.
        Input.Known.Add(Pair.Key);
        if (Pair.Value.bDirty && Wanted.Contains(Pair.Key))
            Input.Rebuild.Add(Pair.Key);
    }

    bTaskRunning.store(true, std::memory_order_release);
    TSharedRef<FVoxelBrickTerrain> Self = AsShared();
    FFunctionGraphTask::CreateAndDispatchWhenReady([Self, Input = MoveTemp(Input)]() mutable
        {
            RunTask(Self, MoveTemp(Input));
        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadNormalTask);
}

void FVoxelBrickTerrain::RunTask(const TSharedRef<FVoxelBrickTerrain>& Self, FTaskInput Input)
{
    const double T0 = FPlatformTime::Seconds();
    FTaskResult Result;
    Result.Generation = Input.Generation;

    const FDensitySampleCompositor& Dens = *Input.Density;
    const TSharedPtr<FSparseEditStore> Edits = Input.Density->GetEditStore();
    Result.StartSequence = Edits.IsValid() ? Edits->GetLastSequence() : 0;

    TArray<VBT::FWantedBrick> Found;
    FVoxelBrickKey Root;
    Root.Level = Input.Settings.MaxLevel;
    Root.Coord = FInt64Vector(0, 0, 0);
    VBT::Descend(Dens, Edits.Get(), Input.Settings, Input.Camera, Root, Found, Result.Samples);

    // To mesh: wanted bricks to rebuild, then wanted bricks without a mesh, nearest first.
    TSet<FVoxelBrickKey> RebuildSet;
    RebuildSet.Append(Input.Rebuild);
    TArray<VBT::FWantedBrick> ToMesh;
    Result.Wanted.Reserve(Found.Num());
    for (const VBT::FWantedBrick& W : Found)
    {
        Result.Wanted.Add(W.Key);
        if (RebuildSet.Contains(W.Key))
            ToMesh.Add({ W.Key, -1.0, W.bEdited });
        else if (!Input.Known.Contains(W.Key))
            ToMesh.Add(W);
    }
    ToMesh.Sort([](const VBT::FWantedBrick& A, const VBT::FWantedBrick& B) { return A.Distance < B.Distance; });
    int32 NumRebuild = 0;
    while (NumRebuild < ToMesh.Num() && ToMesh[NumRebuild].Distance < 0.0) NumRebuild++;

    // New bricks left over when the budget runs out are found again next task; rebuilds are
    // always finished.
    std::atomic<int64> Samples{ 0 };
    for (int32 Start = 0; Start < ToMesh.Num() && (Start < NumRebuild || Samples.load() < VBT::MaxSamplesPerTask); Start += VBT::MeshBatch)
    {
        const int32 Count = FMath::Min(VBT::MeshBatch, ToMesh.Num() - Start);
        TArray<FBuilt> Batch;
        Batch.SetNum(Count);
        ParallelFor(Count, [&](int32 i)
            {
                const VBT::FWantedBrick& W = ToMesh[Start + i];
                const FVoxelBrickGrid Grid = VBT::MakeGrid(Input.Settings, W.Key.Level);
                FVoxelBrickMesh Mesh;
                FVoxelBrickStats Stats;
                VoxelBrickMesher::MeshBrick(Dens, Grid, W.Key.Coord, Input.Settings.SlopeBound, W.bEdited, Mesh, Stats);
                if (Mesh.Indices.Num() > 0)
                    VoxelBrickMesher::AddSkirts(Mesh, (float)(VBT::SkirtCells * Grid.CellSize));
                Batch[i].Key = W.Key;
                Batch[i].Stream = VBT::BuildRenderStream(Mesh, Grid.BrickMin(W.Key.Coord), W.Key.Level, (float)Input.Settings.TriplanarUVScale);
                Samples.fetch_add(Stats.Samples, std::memory_order_relaxed);
            });
        Result.Built.Append(MoveTemp(Batch));
    }
    Result.Samples += Samples.load();
    Result.Milliseconds = (FPlatformTime::Seconds() - T0) * 1000.0;

    {
        FScopeLock Lock(&Self->ResultCS);
        Self->Results.Add(MoveTemp(Result));
    }
    Self->bTaskRunning.store(false, std::memory_order_release);
}

void FVoxelBrickTerrain::DrainResults()
{
    TArray<FTaskResult> Ready;
    {
        FScopeLock Lock(&ResultCS);
        Ready = MoveTemp(Results);
        Results.Reset();
    }

    for (FTaskResult& Result : Ready)
    {
        if (Result.Generation != Generation) continue;

        Wanted.Reset();
        Wanted.Append(Result.Wanted);
        bHasWanted = true;

        int32 NumWithSurface = 0;
        for (FBuilt& Built : Result.Built)
        {
            FBrick& Brick = Bricks.FindOrAdd(Built.Key);
            Brick.Pending = MoveTemp(Built.Stream);
            NumWithSurface += Brick.Pending.IsValid() ? 1 : 0;
            if (!Brick.bHasPending)
            {
                Brick.bHasPending = true;
                ApplyQueue.Add(Built.Key);
            }

            // An edit published after the task read the edit store isn't in this mesh.
            Brick.bDirty = false;
            const FBox Bounds = BoundsOf(Built.Key);
            for (const FRecentEdit& Edit : RecentEdits)
            {
                if (Edit.Sequence > Result.StartSequence && Edit.Bounds.Intersect(Bounds))
                    Brick.bDirty = true;
            }
        }
        RecentEdits.RemoveAll([&Result](const FRecentEdit& Edit) { return Edit.Sequence <= Result.StartSequence; });

        if (CVarVoxelDebugBricks.GetValueOnGameThread() > 0)
        {
            UE_LOG(LogTemp, Log, TEXT("[VoxelBricks] %d wanted, %d meshed (%d with surface), %lld samples, %.1f ms, %d bricks held"),
                Result.Wanted.Num(), Result.Built.Num(), NumWithSurface, Result.Samples, Result.Milliseconds, Bricks.Num());
        }
    }
}

void FVoxelBrickTerrain::ApplyQueued()
{
    int32 Creates = 0;
    int32 Kept = 0;
    for (int32 i = 0; i < ApplyQueue.Num(); i++)
    {
        const FVoxelBrickKey Key = ApplyQueue[i];
        FBrick* Brick = Bricks.Find(Key);
        if (!Brick || !Brick->bHasPending) continue;

        if (!Brick->Pending.IsValid())
        {
            DestroyComponent(*Brick);
            Brick->bHasPending = false;
            Brick->bApplied = true;
            continue;
        }

        if (!Brick->Component.IsValid() || !Brick->Mesh.IsValid())
        {
            DestroyComponent(*Brick);
            if (Creates >= VBT::MaxCreatesPerTick)
            {
                ApplyQueue[Kept++] = Key;
                continue;
            }
            Creates++;
            if (!EnsureComponent(Key, *Brick))
            {
                // Owner or attach root gone: forget the brick so it is meshed again later.
                Bricks.Remove(Key);
                continue;
            }
        }

        URealtimeMeshSimple* Mesh = Brick->Mesh.Get();
        const TSharedPtr<FMeshStreamData> Data = MoveTemp(Brick->Pending);
        Brick->bHasPending = false;
        TFuture<ERealtimeMeshProxyUpdateStatus> Update = Mesh->UpdateBufferSet(Data->MeshGroupKey, Data->MeshStream);
        if (!Brick->bHasSection)
        {
            Mesh->UpdateSectionConfig(Data->MeshSectionKey, FRealtimeMeshSectionConfig(0), false);
            Brick->bHasSection = true;
        }
        Brick->bApplied = true;

        // The future completes once the render proxy has the mesh. A component has no proxy
        // until the end-of-frame update after its creation, so the mesh counts from two
        // frames after that (or after the apply, when it found no proxy). Same rule as
        // FMeshChunk::ApplyStream.
        const TSharedPtr<std::atomic<uint64>> Shown = Brick->ShownFrame;
        const uint64 CreatedFrame = Brick->CreatedFrame;
        const uint64 ApplyFrame = GFrameCounter;
        Update.Next([Shown, CreatedFrame, ApplyFrame](ERealtimeMeshProxyUpdateStatus Status)
            {
                uint64 NotBefore = CreatedFrame + 2;
                if (Status == ERealtimeMeshProxyUpdateStatus::NoProxy)
                    NotBefore = FMath::Max(NotBefore, ApplyFrame + 2);
                uint64 Old = Shown->load(std::memory_order_relaxed);
                while (Old == 0 || NotBefore < Old)
                {
                    if (Shown->compare_exchange_weak(Old, NotBefore, std::memory_order_release, std::memory_order_relaxed))
                        break;
                }
            });
    }
    ApplyQueue.SetNum(Kept);
}

bool FVoxelBrickTerrain::IsOnScreen(const FVoxelBrickKey& Key) const
{
    const FBrick* Brick = Bricks.Find(Key);
    if (!Brick || !Brick->bApplied) return false;
    if (!Brick->Component.IsValid()) return true;   // applied empty: nothing to show
    const uint64 Frame = Brick->ShownFrame.IsValid() ? Brick->ShownFrame->load(std::memory_order_acquire) : 0;
    return Frame != 0 && GFrameCounter >= Frame;
}

bool FVoxelBrickTerrain::IsCovered(const FVoxelBrickKey& Key) const
{
    // A wanted ancestor replaces it on a merge.
    for (int32 Level = Key.Level + 1; Level <= Settings.MaxLevel; Level++)
    {
        const int32 Up = Level - Key.Level;
        FVoxelBrickKey Ancestor;
        Ancestor.Level = Level;
        Ancestor.Coord = FInt64Vector(VBT::FloorShift(Key.Coord.X, Up), VBT::FloorShift(Key.Coord.Y, Up), VBT::FloorShift(Key.Coord.Z, Up));
        if (Wanted.Contains(Ancestor))
            return IsOnScreen(Ancestor);
    }

    // Wanted descendants replace it on a split. None at all: its region holds no surface now.
    for (const FVoxelBrickKey& W : Wanted)
    {
        if (W.Level >= Key.Level) continue;
        const int32 Up = Key.Level - W.Level;
        if (VBT::FloorShift(W.Coord.X, Up) == Key.Coord.X && VBT::FloorShift(W.Coord.Y, Up) == Key.Coord.Y && VBT::FloorShift(W.Coord.Z, Up) == Key.Coord.Z
            && !IsOnScreen(W))
            return false;
    }
    return true;
}

void FVoxelBrickTerrain::RemoveReplaced()
{
    if (!bHasWanted) return;

    TArray<FVoxelBrickKey> Remove;
    for (const TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        if (!Wanted.Contains(Pair.Key) && IsCovered(Pair.Key))
            Remove.Add(Pair.Key);
    }
    for (const FVoxelBrickKey& Key : Remove)
    {
        DestroyComponent(Bricks[Key]);
        Bricks.Remove(Key);
    }
    if (Remove.Num() > 0)
        ApplyQueue.RemoveAll([this](const FVoxelBrickKey& Key) { return !Bricks.Contains(Key); });
}

bool FVoxelBrickTerrain::EnsureComponent(const FVoxelBrickKey& Key, FBrick& Brick)
{
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!OwnerActor || !Root) return false;

    // Transient: built at run time, never saved with the level or copied into PIE.
    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetupMaterialSlot(0, "Surface Material");

    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->bHiddenInSceneCapture = true;
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->RegisterComponent();
    Component->SetMaterial(0, Settings.Material.Get());
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetRelativeLocation(GridFor(Key.Level).BrickMin(Key.Coord));
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetRealtimeMesh(Mesh);
    Component->SetRenderCustomDepth(true);
    Component->SetVisibleInRayTracing(false);

    Mesh->CreateBufferSet(FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), VBT::RenderGroupName), FRealtimeMeshStreamSet());

    Brick.Mesh = Mesh;
    Brick.Component = Component;
    Brick.bHasSection = false;
    Brick.CreatedFrame = GFrameCounter;
    Brick.ShownFrame = MakeShared<std::atomic<uint64>>(0);
    return true;
}

void FVoxelBrickTerrain::DestroyComponent(FBrick& Brick)
{
    if (URealtimeMeshComponent* Component = Brick.Component.Get())
        Component->DestroyComponent();
    Brick.Component = nullptr;
    Brick.Mesh = nullptr;
    Brick.bHasSection = false;
}
