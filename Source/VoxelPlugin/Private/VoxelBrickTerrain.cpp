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

static TAutoConsoleVariable<int32> CVarVoxelBrickSeams(
    TEXT("voxel.Render.BrickSeams"), 0,
    TEXT("Seams between brick-terrain LOD levels. 0: stitched. 1: stitched, plus skirts. 2: skirts only (unstitched bricks)."),
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
        Grid.CellSize = Settings.FinestCell * (double)(1LL << Level);   // exact: stitched bricks share lattice points across levels
        Grid.CellsPerBrick = Settings.CellsPerBrick;
        return Grid;
    }

    enum class ENode : uint8
    {
        Internal,
        Wanted,     // leaf that may hold surface
        Culled      // leaf proven free of surface
    };

    struct FWantedInfo
    {
        double Distance;
        bool bEdited;
    };

    FVoxelBrickKey ParentOf(const FVoxelBrickKey& Key)
    {
        FVoxelBrickKey Parent;
        Parent.Level = Key.Level + 1;
        Parent.Coord = FInt64Vector(FloorShift(Key.Coord.X, 1), FloorShift(Key.Coord.Y, 1), FloorShift(Key.Coord.Z, 1));
        return Parent;
    }

    FVoxelBrickKey ChildOf(const FVoxelBrickKey& Key, int32 Index)
    {
        FVoxelBrickKey Child;
        Child.Level = Key.Level - 1;
        Child.Coord = FInt64Vector(2 * Key.Coord.X + (Index & 1), 2 * Key.Coord.Y + ((Index >> 1) & 1), 2 * Key.Coord.Z + ((Index >> 2) & 1));
        return Child;
    }

    FVoxelBrickKey Offset(const FVoxelBrickKey& Key, int32 DX, int32 DY, int32 DZ)
    {
        FVoxelBrickKey Moved = Key;
        Moved.Coord = FInt64Vector(Key.Coord.X + DX, Key.Coord.Y + DY, Key.Coord.Z + DZ);
        return Moved;
    }

    /** The LOD tree of one task, over the whole root: every internal node has all eight
     *  children, every leaf is wanted or culled. */
    struct FTree
    {
        FTree(const FDensitySampleCompositor& InDensity, const FSparseEditStore* InEdits, const FVoxelBrickTerrainSettings& InSettings, const FVector& InCamera)
            : Density(InDensity), Edits(InEdits), Settings(InSettings), Camera(InCamera)
        {
        }

        const FDensitySampleCompositor& Density;
        const FSparseEditStore* Edits;
        const FVoxelBrickTerrainSettings& Settings;
        FVector Camera;

        TMap<FVoxelBrickKey, ENode> Nodes;
        TMap<FVoxelBrickKey, FWantedInfo> WantedInfo;
        int64 Samples = 0;
        int32 BalanceSplits = 0;

        bool InRoot(const FVoxelBrickKey& Key) const
        {
            const int64 Count = 1LL << (Settings.MaxLevel - Key.Level);
            return Key.Coord.X >= 0 && Key.Coord.Y >= 0 && Key.Coord.Z >= 0 && Key.Coord.X < Count && Key.Coord.Y < Count && Key.Coord.Z < Count;
        }

        double DistanceTo(const FVoxelBrickKey& Key) const
        {
            const FVoxelBrickGrid Grid = MakeGrid(Settings, Key.Level);
            const FVector Min = Grid.BrickMin(Key.Coord);
            return FMath::Sqrt(BoxDistanceSquared(FBox(Min, Min + FVector(Grid.BrickSize())), Camera));
        }

        /** False when the brick's centre sample proves it free of surface (never where edits
         *  reach it). */
        bool MayHoldSurface(const FVoxelBrickKey& Key, bool& bOutEdited)
        {
            const FVoxelBrickGrid Grid = MakeGrid(Settings, Key.Level);
            const double H = Grid.CellSize;
            const FVector Min = Grid.BrickMin(Key.Coord);
            const FBox Region(Min - FVector(2.0 * H), Min + FVector(Grid.BrickSize() + H));
            bOutEdited = Edits && Edits->HasEditsInBox(Region);
            if (bOutEdited) return true;

            const FVector Centre = Region.GetCenter();
            double CX = Centre.X, CY = Centre.Y, CZ = Centre.Z;
            float D = 0.0f;
            Density.Sample(FSampleInput(&CX, &CY, &CZ, 1, H), &D);
            Samples++;
            return FMath::Abs((double)D) <= Settings.SlopeBound * Region.GetExtent().Size();
        }

        /** The LOD descent: culls bricks proven free of surface and splits bricks the camera
         *  is close to. */
        void Descend(const FVoxelBrickKey& Key)
        {
            bool bEdited = false;
            if (!MayHoldSurface(Key, bEdited))
            {
                Nodes.Add(Key, ENode::Culled);
                return;
            }
            const double Distance = DistanceTo(Key);
            if (Key.Level > 0 && Distance < Settings.LodDistanceFactor * MakeGrid(Settings, Key.Level).BrickSize())
            {
                Nodes.Add(Key, ENode::Internal);
                for (int32 c = 0; c < 8; c++)
                    Descend(ChildOf(Key, c));
                return;
            }
            Nodes.Add(Key, ENode::Wanted);
            WantedInfo.Add(Key, { Distance, bEdited });
        }

        /** Splits a leaf for the balance. A culled leaf's children are culled; a wanted leaf's
         *  children are tested. */
        void Split(const FVoxelBrickKey& Key, TArray<FVoxelBrickKey>& Work)
        {
            const ENode Was = Nodes.FindRef(Key);
            Nodes.Add(Key, ENode::Internal);
            WantedInfo.Remove(Key);
            BalanceSplits++;
            for (int32 c = 0; c < 8; c++)
            {
                const FVoxelBrickKey Child = ChildOf(Key, c);
                bool bEdited = false;
                if (Was == ENode::Wanted && MayHoldSurface(Child, bEdited))
                {
                    Nodes.Add(Child, ENode::Wanted);
                    WantedInfo.Add(Child, { DistanceTo(Child), bEdited });
                }
                else
                {
                    Nodes.Add(Child, ENode::Culled);
                }
                Work.Add(Child);
            }
        }

        /** Leaf covering a brick-sized region (the region itself, or the first ancestor in
         *  the tree when the region lies inside a coarser leaf). */
        FVoxelBrickKey CoveringNode(const FVoxelBrickKey& Key) const
        {
            FVoxelBrickKey Cover = Key;
            while (Cover.Level < Settings.MaxLevel && !Nodes.Contains(Cover))
                Cover = ParentOf(Cover);
            return Cover;
        }

        /** Splits leaves until every two leaves that touch (faces, edges or corners) are at
         *  most one level apart, over wanted and culled leaves alike, so every point of the
         *  root has one owning leaf level that each neighbouring brick agrees on. */
        void Balance()
        {
            TArray<FVoxelBrickKey> Work;
            for (const TPair<FVoxelBrickKey, ENode>& Pair : Nodes)
            {
                if (Pair.Value != ENode::Internal)
                    Work.Add(Pair.Key);
            }
            while (Work.Num() > 0)
            {
                const FVoxelBrickKey Key = Work.Pop();
                const ENode* State = Nodes.Find(Key);
                if (!State || *State == ENode::Internal) continue;

                for (int32 d = 0; d < 27; d++)
                {
                    if (d == 13) continue;
                    const FVoxelBrickKey Neighbour = Offset(Key, d % 3 - 1, (d / 3) % 3 - 1, d / 9 - 1);
                    if (!InRoot(Neighbour)) continue;
                    const FVoxelBrickKey Cover = CoveringNode(Neighbour);
                    if (Cover.Level >= Key.Level + 2)
                    {
                        Split(Cover, Work);
                        Work.Add(Key);   // the split may not be enough yet
                        break;
                    }
                }
            }
        }

        /** Level of the leaves touching Key in direction d, relative to Key (balanced tree),
         *  and whether that neighbour is a wanted brick of Key's level. */
        int32 NeighbourDelta(const FVoxelBrickKey& Key, int32 d, bool& bOutMeshed) const
        {
            bOutMeshed = false;
            const FVoxelBrickKey Neighbour = Offset(Key, d % 3 - 1, (d / 3) % 3 - 1, d / 9 - 1);
            if (!InRoot(Neighbour)) return 0;
            if (const ENode* State = Nodes.Find(Neighbour))
            {
                bOutMeshed = *State == ENode::Wanted;
                return *State == ENode::Internal ? -1 : 0;
            }
            return 1;
        }

        /** Wanted leaves touching Key. */
        void TouchingWanted(const FVoxelBrickKey& Key, TArray<FVoxelBrickKey>& Out) const
        {
            for (int32 d = 0; d < 27; d++)
            {
                if (d == 13) continue;
                const int32 Dir[3] = { d % 3 - 1, (d / 3) % 3 - 1, d / 9 - 1 };
                const FVoxelBrickKey Neighbour = Offset(Key, Dir[0], Dir[1], Dir[2]);
                if (!InRoot(Neighbour)) continue;
                const ENode* State = Nodes.Find(Neighbour);
                if (!State)
                {
                    const FVoxelBrickKey Parent = ParentOf(Neighbour);
                    if (Nodes.FindRef(Parent) == ENode::Wanted)
                        Out.AddUnique(Parent);
                }
                else if (*State == ENode::Wanted)
                {
                    Out.Add(Neighbour);
                }
                else if (*State == ENode::Internal)
                {
                    for (int32 c = 0; c < 8; c++)
                    {
                        bool bTouches = true;
                        for (int32 Axis = 0; Axis < 3; Axis++)
                        {
                            const int32 Bit = (c >> Axis) & 1;
                            bTouches &= Dir[Axis] == 0 || (Dir[Axis] < 0 ? Bit == 1 : Bit == 0);
                        }
                        const FVoxelBrickKey Child = ChildOf(Neighbour, c);
                        if (bTouches && Nodes.FindRef(Child) == ENode::Wanted)
                            Out.Add(Child);
                    }
                }
            }
        }
    };

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
    Settings.CellsPerBrick = FMath::Max(8, Settings.CellsPerBrick / 4 * 4);
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
    // A stitched brick also solves cells of coarser neighbours, whose gradients reach four of
    // its own cells beyond it.
    const FVoxelBrickGrid Grid = GridFor(Key.Level);
    const FVector Min = Grid.BrickMin(Key.Coord);
    return FBox(Min - FVector(4.0 * Grid.CellSize), Min + FVector(Grid.BrickSize() + 4.0 * Grid.CellSize));
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
    Input.SeamMode = FMath::Clamp(CVarVoxelBrickSeams.GetValueOnGameThread(), 0, 2);
    Input.Known.Reserve(Bricks.Num());
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        // Dirty stays set until a rebuilt mesh arrives (DrainResults), so a rebuild of a brick
        // that is not wanted this pass is not lost.
        Input.Known.Add(Pair.Key, Pair.Value.bHasPending ? Pair.Value.PendingSignature : Pair.Value.Signature);
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

    const TSharedPtr<FSparseEditStore> Edits = Input.Density->GetEditStore();
    Result.StartSequence = Edits.IsValid() ? Edits->GetLastSequence() : 0;

    VBT::FTree Tree(*Input.Density, Edits.Get(), Input.Settings, Input.Camera);
    FVoxelBrickKey Root;
    Root.Level = Input.Settings.MaxLevel;
    Root.Coord = FInt64Vector(0, 0, 0);
    Tree.Descend(Root);
    if (Input.SeamMode != 2)
        Tree.Balance();
    Result.BalanceSplits = Tree.BalanceSplits;

    // Each wanted brick's neighbours, packed with the seam mode into its signature: per
    // direction 0 finer, 1 same level and meshed, 2 same level and not meshed, 3 coarser.
    struct FToMesh
    {
        FVoxelBrickKey Key;
        double Distance;
        bool bEdited;
        uint64 Signature;
        uint32 Meshed;
        int8 Levels[27];
    };
    TSet<FVoxelBrickKey> RebuildSet;
    RebuildSet.Append(Input.Rebuild);
    TArray<FToMesh> Rebuilds, NewBricks, Restitches;
    Result.Wanted.Reserve(Tree.WantedInfo.Num());
    for (const TPair<FVoxelBrickKey, VBT::FWantedInfo>& Pair : Tree.WantedInfo)
    {
        FToMesh Entry;
        Entry.Key = Pair.Key;
        Entry.Distance = Pair.Value.Distance;
        Entry.bEdited = Pair.Value.bEdited;
        Entry.Signature = (1ULL << 63) | ((uint64)Input.SeamMode << 56);
        Entry.Meshed = 0;
        for (int32 d = 0, Slot = 0; d < 27; d++)
        {
            bool bMeshed = false;
            const int32 Delta = Input.SeamMode == 2 ? 0 : Tree.NeighbourDelta(Pair.Key, d, bMeshed);
            Entry.Levels[d] = (int8)Delta;
            Entry.Meshed |= bMeshed ? (1u << d) : 0u;
            if (d == 13) continue;
            const uint64 State = Delta < 0 ? 0 : (Delta > 0 ? 3 : (bMeshed ? 1 : 2));
            Entry.Signature |= State << (2 * Slot++);
        }
        Result.Wanted.Add(Pair.Key, Entry.Signature);

        const uint64* Held = Input.Known.Find(Pair.Key);
        if (!Held)
            NewBricks.Add(Entry);
        else if (RebuildSet.Contains(Pair.Key))
            Rebuilds.Add(Entry);
        else if (*Held != Entry.Signature)
            Restitches.Add(Entry);
    }
    NewBricks.Sort([](const FToMesh& X, const FToMesh& Y) { return X.Distance < Y.Distance; });

    const double SkirtDepth = Input.SeamMode == 0 ? 0.0 : VBT::SkirtCells;
    std::atomic<int64> Samples{ 0 };
    auto MeshAll = [&Input, &Tree, &Result, &Samples, SkirtDepth](const TArray<FToMesh>& List, int32 First, int32 Count)
        {
            TArray<FBuilt> Batch;
            Batch.SetNum(Count);
            ParallelFor(Count, [&](int32 i)
                {
                    const FToMesh& Entry = List[First + i];
                    const FVoxelBrickGrid Grid = VBT::MakeGrid(Input.Settings, Entry.Key.Level);
                    FVoxelBrickMesh Mesh;
                    FVoxelBrickStats Stats;
                    if (Input.SeamMode == 2)
                        VoxelBrickMesher::MeshBrick(*Input.Density, Grid, Entry.Key.Coord, Input.Settings.SlopeBound, Entry.bEdited, Mesh, Stats);
                    else
                        VoxelBrickMesher::MeshBrickStitched(*Input.Density, Grid, Entry.Key.Coord, Entry.Levels, Entry.Meshed, Input.Settings.SlopeBound, Entry.bEdited, Mesh, Stats);
                    if (SkirtDepth > 0.0 && Mesh.Indices.Num() > 0)
                        VoxelBrickMesher::AddSkirts(Mesh, (float)(SkirtDepth * Grid.CellSize));

                    FBuilt& Built = Batch[i];
                    Built.Key = Entry.Key;
                    Built.Signature = Entry.Signature;
                    Built.Stream = VBT::BuildRenderStream(Mesh, Grid.BrickMin(Entry.Key.Coord), Entry.Key.Level, (float)Input.Settings.TriplanarUVScale);
                    Tree.TouchingWanted(Entry.Key, Built.Neighbours);
                    Samples.fetch_add(Stats.Samples, std::memory_order_relaxed);
                });
            Result.Built.Append(MoveTemp(Batch));
        };

    // Edit rebuilds always finish. New bricks are meshed nearest first while the budget lasts;
    // those left over are found again next task.
    MeshAll(Rebuilds, 0, Rebuilds.Num());
    for (int32 Start = 0; Start < NewBricks.Num() && Samples.load() < VBT::MaxSamplesPerTask; Start += VBT::MeshBatch)
        MeshAll(NewBricks, Start, FMath::Min(VBT::MeshBatch, NewBricks.Num() - Start));

    // Held bricks whose neighbours changed are restitched once every brick touching them is
    // held or built, so the restitched mesh never waits long on screen for its neighbours.
    TSet<FVoxelBrickKey> Available;
    for (const FBuilt& Built : Result.Built)
        Available.Add(Built.Key);
    TArray<FToMesh> Ready;
    TArray<FVoxelBrickKey> Touching;
    for (const FToMesh& Entry : Restitches)
    {
        Touching.Reset();
        Tree.TouchingWanted(Entry.Key, Touching);
        bool bAvailable = true;
        for (const FVoxelBrickKey& Key : Touching)
            bAvailable &= Input.Known.Contains(Key) || Available.Contains(Key);
        if (bAvailable)
            Ready.Add(Entry);
    }
    Result.NumRestitched = Ready.Num();
    for (int32 Start = 0; Start < Ready.Num(); Start += VBT::MeshBatch)
        MeshAll(Ready, Start, FMath::Min(VBT::MeshBatch, Ready.Num() - Start));

    Result.Samples = Tree.Samples + Samples.load();
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

        Wanted = MoveTemp(Result.Wanted);
        bHasWanted = true;

        int32 NumWithSurface = 0;
        for (FBuilt& Built : Result.Built)
        {
            FBrick& Brick = Bricks.FindOrAdd(Built.Key);
            Brick.Pending = MoveTemp(Built.Stream);
            Brick.PendingSignature = Built.Signature;
            NumWithSurface += Brick.Pending.IsValid() ? 1 : 0;

            // A mesh stitched to other neighbours than the one on screen waits until its new
            // neighbours are on screen (ApplyQueued), then swaps in the same frame as the
            // bricks they replace leave (RemoveReplaced).
            Brick.WaitFor.Reset();
            if (Brick.bApplied && Brick.Signature != Built.Signature)
                Brick.WaitFor = MoveTemp(Built.Neighbours);
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
            UE_LOG(LogTemp, Log, TEXT("[VoxelBricks] %d wanted (%d balance splits), %d meshed (%d with surface, %d restitched), %lld samples, %.1f ms, %d bricks held"),
                Result.Wanted.Num(), Result.BalanceSplits, Result.Built.Num(), NumWithSurface, Result.NumRestitched, Result.Samples, Result.Milliseconds, Bricks.Num());
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

        bool bReady = true;
        for (const FVoxelBrickKey& Neighbour : Brick->WaitFor)
            bReady &= !Wanted.Contains(Neighbour) || IsOnScreen(Neighbour);
        if (!bReady)
        {
            ApplyQueue[Kept++] = Key;
            continue;
        }
        Brick->WaitFor.Reset();

        if (!Brick->Pending.IsValid())
        {
            DestroyComponent(*Brick);
            Brick->bHasPending = false;
            Brick->bApplied = true;
            Brick->Signature = Brick->PendingSignature;
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
        Brick->Signature = Brick->PendingSignature;
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

bool FVoxelBrickTerrain::IsSettled(const FVoxelBrickKey& Key) const
{
    const FBrick* Brick = Bricks.Find(Key);
    const uint64* Signature = Wanted.Find(Key);
    return Brick && Signature && Brick->Signature == *Signature && IsOnScreen(Key);
}

void FVoxelBrickTerrain::TouchingWanted(const FVoxelBrickKey& Key, TArray<FVoxelBrickKey>& Out) const
{
    for (int32 d = 0; d < 27; d++)
    {
        if (d == 13) continue;
        const int32 Dir[3] = { d % 3 - 1, (d / 3) % 3 - 1, d / 9 - 1 };
        const FVoxelBrickKey Neighbour = VBT::Offset(Key, Dir[0], Dir[1], Dir[2]);
        if (Wanted.Contains(Neighbour))
        {
            Out.AddUnique(Neighbour);
            continue;
        }
        const FVoxelBrickKey Parent = VBT::ParentOf(Neighbour);
        if (Wanted.Contains(Parent))
        {
            Out.AddUnique(Parent);
            continue;
        }
        for (int32 c = 0; c < 8 && Neighbour.Level > 0; c++)
        {
            bool bTouches = true;
            for (int32 Axis = 0; Axis < 3; Axis++)
            {
                const int32 Bit = (c >> Axis) & 1;
                bTouches &= Dir[Axis] == 0 || (Dir[Axis] < 0 ? Bit == 1 : Bit == 0);
            }
            const FVoxelBrickKey Child = VBT::ChildOf(Neighbour, c);
            if (bTouches && Wanted.Contains(Child))
                Out.AddUnique(Child);
        }
    }
}

bool FVoxelBrickTerrain::IsCovered(const FVoxelBrickKey& Key, const TArray<FVoxelBrickKey>& Descendants) const
{
    // Replacements: a wanted ancestor on a merge, the wanted descendants on a split. None at
    // all: its region holds no surface now.
    TArray<FVoxelBrickKey> Replacements;
    for (int32 Level = Key.Level + 1; Level <= Settings.MaxLevel && Replacements.Num() == 0; Level++)
    {
        const int32 Up = Level - Key.Level;
        FVoxelBrickKey Ancestor;
        Ancestor.Level = Level;
        Ancestor.Coord = FInt64Vector(VBT::FloorShift(Key.Coord.X, Up), VBT::FloorShift(Key.Coord.Y, Up), VBT::FloorShift(Key.Coord.Z, Up));
        if (Wanted.Contains(Ancestor))
            Replacements.Add(Ancestor);
    }
    if (Replacements.Num() == 0)
        Replacements = Descendants;

    // The replacements and the bricks around them must show meshes stitched to the new tree,
    // or removing this brick opens the cracks it was hiding.
    for (const FVoxelBrickKey& Replacement : Replacements)
    {
        if (!IsSettled(Replacement)) return false;
    }
    TArray<FVoxelBrickKey> Around;
    for (const FVoxelBrickKey& Replacement : Replacements)
    {
        Around.Reset();
        TouchingWanted(Replacement, Around);
        for (const FVoxelBrickKey& Neighbour : Around)
        {
            if (!IsSettled(Neighbour)) return false;
        }
    }
    return true;
}

void FVoxelBrickTerrain::RemoveReplaced()
{
    if (!bHasWanted) return;

    // Held bricks no longer wanted, each with its wanted descendants.
    TMap<FVoxelBrickKey, TArray<FVoxelBrickKey>> Candidates;
    int32 MaxCandidateLevel = -1;
    for (const TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        if (Wanted.Contains(Pair.Key)) continue;
        Candidates.Add(Pair.Key);
        MaxCandidateLevel = FMath::Max(MaxCandidateLevel, Pair.Key.Level);
    }
    if (Candidates.Num() == 0) return;
    for (const TPair<FVoxelBrickKey, uint64>& Pair : Wanted)
    {
        FVoxelBrickKey Ancestor = Pair.Key;
        while (Ancestor.Level < MaxCandidateLevel)
        {
            Ancestor = VBT::ParentOf(Ancestor);
            if (TArray<FVoxelBrickKey>* Descendants = Candidates.Find(Ancestor))
                Descendants->Add(Pair.Key);
        }
    }

    TArray<FVoxelBrickKey> Remove;
    for (const TPair<FVoxelBrickKey, TArray<FVoxelBrickKey>>& Candidate : Candidates)
    {
        if (IsCovered(Candidate.Key, Candidate.Value))
            Remove.Add(Candidate.Key);
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
