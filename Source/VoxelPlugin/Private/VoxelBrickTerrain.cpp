#include "VoxelBrickTerrain.h"
#include "VoxelBrickPages.h"
#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "FMeshingStructs.h"
#include "FDensitySampleCompositor.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctree.h"
#include "Async/ParallelFor.h"
#include "Async/TaskGraphInterfaces.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Misc/App.h"
#include "Misc/ScopeExit.h"
#include "Engine/World.h"
#include "Materials/MaterialParameterCollection.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "VoxelTerrainStats.h"

DECLARE_CYCLE_STAT(TEXT("Bricks: tick"), STAT_VoxelBricksTick, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: drain task results"), STAT_VoxelBricksDrain, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: apply queue or commit"), STAT_VoxelBricksApply, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: remove replaced"), STAT_VoxelBricksRemove, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: flush pages"), STAT_VoxelBricksFlush, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: launch task"), STAT_VoxelBricksLaunch, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: task (worker)"), STAT_VoxelBricksTask, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: task tree (worker)"), STAT_VoxelBricksTaskTree, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Bricks held"), STAT_VoxelBricksHeld, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Bricks wanted"), STAT_VoxelBricksWanted, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Bricks queued to apply"), STAT_VoxelBricksQueued, STATGROUP_VoxelTerrain);

static TAutoConsoleVariable<int32> CVarVoxelBrickPerf(
    TEXT("voxel.Debug.BrickPerf"), 0,
    TEXT("1: once a second, log where the brick terrain spent its time (game thread sections, tasks, page work and render-thread time)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugBricks(
    TEXT("voxel.Debug.Bricks"), 0,
    TEXT("1: log each brick terrain task (wanted, meshed, samples, time)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelBrickSeams(
    TEXT("voxel.Render.BrickSeams"), 0,
    TEXT("Seams between brick-terrain LOD levels. 0: stitched. 1: stitched, plus skirts. 2: skirts only (unstitched bricks)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelBrickMorph(
    TEXT("voxel.Render.BrickMorph"), 1,
    TEXT("Geomorphing of the GPU brick terrain, in the terrain material (VoxelMorph.ush, through the actor's Brick Morph Parameters collection). 1: vertices blend toward the next coarser level with distance, so LOD changes don't pop. 0: off. 2: fully blended everywhere (shows the coarser levels; for checking)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelBrickBackend(
    TEXT("voxel.Render.BrickBackend"), 1,
    TEXT("Where brick-terrain bricks are meshed. 1 (default): GPU (pages of 8x8x8 bricks meshed ahead on async compute, written by compute providers; stitched seams only). 0: CPU (one component per brick; also used when the GPU path is unavailable)."),
    ECVF_Default);

namespace VoxelBrickTerrainDetail
{
    constexpr double TaskInterval = 0.1;            // seconds between tasks
    constexpr double TaskIntervalGPU = 0.05;        // GPU backend: meshing costs the task nothing
    constexpr double VelocitySmoothing = 0.15;      // seconds: camera velocity filter
    constexpr double MaxCameraStep = 0.5;           // seconds: a longer frame resets the velocity
    constexpr double MaxCameraJumpBricks = 8.0;     // finest bricks: a longer step in a frame too
    constexpr double SlackFraction = 0.25;          // of the distance flown in the tree's latency
    /** A brick split in the previous tree stays split until this many times the split
     *  distance. Free for the morph: level-l vertices still meet level l + 1 only beyond the
     *  split distance (Design/GPUPath.md section 4), and up to 1.15 times it level l + 1
     *  is unblended where level l is fully blended, for any F below 11. */
    constexpr double MergeHysteresis = 1.15;
    constexpr int32 MaxCreatesPerTick = 32;         // component creations per frame
    constexpr int64 MaxSamplesPerTask = 4000000;    // CPU backend: density samples one task may spend on meshing
    constexpr double CommitWaitBlend = 0.5;          // filter of the commit wait
    constexpr int32 MeshBatch = 16;                 // bricks meshed per parallel batch
    constexpr double SkirtCells = 2.5;              // skirt depth in cells of the brick's level

    const FName RenderGroupName(TEXT("BRICK"));
    // Vectors of the morph collection (VoxelMorph.ush).
    const FName MorphCameraHighName(TEXT("VoxelMorphCameraHigh"));
    const FName MorphCameraLowName(TEXT("VoxelMorphCameraLow"));
    const FName MorphParamsName(TEXT("VoxelMorphParams"));

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
        FTree(const FDensitySampleCompositor& InDensity, const FSparseEditStore* InEdits, const FVoxelBrickTerrainSettings& InSettings, const FVector& InCamera,
            const TSet<FVoxelBrickKey>* InSplitBefore)
            : Density(InDensity), Edits(InEdits), Settings(InSettings), Camera(InCamera), SplitBefore(InSplitBefore)
        {
        }

        const FDensitySampleCompositor& Density;
        const FSparseEditStore* Edits;
        const FVoxelBrickTerrainSettings& Settings;
        FVector Camera;
        const TSet<FVoxelBrickKey>* SplitBefore;   // internal nodes of the previous tree

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
            const double SplitDistance = Settings.LodDistanceFactor * MakeGrid(Settings, Key.Level).BrickSize();
            const bool bWasSplit = SplitBefore && SplitBefore->Contains(Key);
            if (Key.Level > 0 && Distance < (bWasSplit ? MergeHysteresis * SplitDistance : SplitDistance))
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

    /** Neighbour levels and the meshed mask a signature packs (see RunTask). */
    void DecodeSignature(uint64 Signature, int8(&OutLevels)[27], uint32& OutMeshed)
    {
        OutLevels[13] = 0;
        OutMeshed = 1u << 13;
        for (int32 d = 0, Slot = 0; d < 27; d++)
        {
            if (d == 13) continue;
            const uint32 State = (uint32)(Signature >> (2 * Slot++)) & 3u;
            OutLevels[d] = State == 0u ? (int8)-1 : (State == 3u ? (int8)1 : (int8)0);
            OutMeshed |= State == 1u ? (1u << d) : 0u;
        }
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

FVoxelBrickTerrain::FVoxelBrickTerrain()
    : Pages(MakeUnique<FVoxelBrickPages>())
{
}

FVoxelBrickTerrain::~FVoxelBrickTerrain() = default;

FColor FVoxelBrickTerrain::LevelColor(int32 Level)
{
    return VBT::LevelColor(Level);
}

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
    Pages->Initialize(InOwner, InAttachRoot, Settings);
    GPUDensity.Reset();
    GPUEdits.Reset();
    bWarnedBackend = false;
    bReportedMorph = false;
}

void FVoxelBrickTerrain::Reset()
{
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
        DestroyComponent(Pair.Value);
    Bricks.Reset();
    Pages->Reset();
    Wanted.Reset();
    bHasWanted = false;
    ApplyQueue.Reset();
    RecentEdits.Reset();
    Generation++;
    LastLaunchTime = 0.0;
    bLastComplete = false;
    CameraVelocity = FVector::ZeroVector;
    LastTickTime = 0.0;
    bCommitPending = false;
    bCommitProgressive = false;
    PendingChanges.Reset();
    NumInPage = 0;
    CommitWaitSeconds = 0.0;
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

int32 FVoxelBrickTerrain::ResolveBackend()
{
    if (CVarVoxelBrickBackend.GetValueOnGameThread() != 1) return 0;

    // The GPU lattice is centred on the root: Origin must be -HalfFinest * 2^MaxDepth.
    const double RootExtent = 0.5 * Settings.FinestCell * FMath::Pow(2.0, (double)Pages->GetMesherSettings().MaxDepth);
    const TCHAR* Reason = nullptr;
    if (!FApp::CanEverRender())
        Reason = TEXT("no renderer");
    else if (!Density.IsValid() || !Density->GetBaseDensity().IsValid())
        Reason = TEXT("the density has no GPU version");
    else if (!Settings.Origin.Equals(FVector(-RootExtent), 1e-6 * RootExtent))
        Reason = TEXT("the brick lattice is not centred on the root");
    if (!Reason) return 1;

    if (!bWarnedBackend)
        UE_LOG(LogTemp, Warning, TEXT("[VoxelBricks] voxel.Render.BrickBackend 1 is not available (%s); meshing on the CPU."), Reason);
    bWarnedBackend = true;
    return 0;
}

void FVoxelBrickTerrain::FlushPages(double TaskInterval)
{
    if (!GPUDensity.IsValid())
    {
        const TSharedRef<FVoxelDensityGPUData> Data = MakeShared<FVoxelDensityGPUData>();
        Density->GetBaseDensity()->BuildGPUData(*Data);
        GPUDensity = Data;
    }
    const TSharedPtr<FSparseEditStore> Store = Density->GetEditStore();
    const uint32 Sequence = Store.IsValid() ? Store->GetLastSequence() : 0;
    if (!GPUEdits.IsValid() || Sequence != GPUEditsSequence)
    {
        GPUEdits = FVoxelEditGPUData::Build(Store.Get(), 0.5 * Settings.FinestCell, Pages->GetMesherSettings().MaxDepth);
        GPUEditsSequence = Sequence;
    }
    Pages->Flush(GPUDensity, GPUEdits);

    // The tree is built for the camera predicted half an interval past its latency; what the
    // prediction misses (half an interval of flight, a guess at the rest) moves the morph bands
    // nearer, so a level is still fully blended wherever the tree on screen meets the next.
    const double Speed = CameraVelocity.Size();
    UpdateMorphParameters(Speed * (0.5 * TaskInterval + VBT::SlackFraction * TreeLatency));
}

void FVoxelBrickTerrain::SetMorphParameters(UMaterialParameterCollection* InCollection)
{
    Settings.MorphParameters = InCollection;
    bReportedMorph = false;
}

void FVoxelBrickTerrain::UpdateMorphParameters(double Slack)
{
    const int32 Mode = FMath::Clamp(CVarVoxelBrickMorph.GetValueOnGameThread(), 0, 2);
    UMaterialParameterCollection* Collection = Settings.MorphParameters.Get();
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    UWorld* World = OwnerActor ? OwnerActor->GetWorld() : nullptr;
    UMaterialParameterCollectionInstance* Instance = Collection && World ? World->GetParameterCollectionInstance(Collection) : nullptr;
    if (!Instance || !Root)
    {
        if (Mode != 0 && !bReportedMorph)
            UE_LOG(LogTemp, Warning, TEXT("[VoxelBricks] No Brick Morph Parameters collection on %s: the GPU terrain's LOD changes won't blend (Design/MeshingEngine.md section 7)."),
                OwnerActor ? *OwnerActor->GetName() : TEXT("the terrain"));
        bReportedMorph = true;
        return;
    }

    // The camera as a float and the rest, so the material subtracts it at full precision.
    const FVector Camera = Root->GetComponentTransform().TransformPosition(LastCamera);
    const FVector3f High(Camera);
    const FVector Low = Camera - FVector(High);
    const double Scale = Root->GetComponentScale().X;
    bool bSet = Instance->SetVectorParameterValue(VBT::MorphCameraHighName, FLinearColor(High.X, High.Y, High.Z, 0.0f));
    bSet &= Instance->SetVectorParameterValue(VBT::MorphCameraLowName, FLinearColor((float)Low.X, (float)Low.Y, (float)Low.Z, 0.0f));
    bSet &= Instance->SetVectorParameterValue(VBT::MorphParamsName, FLinearColor((float)(Slack * Scale), (float)Mode, 0.0f, 0.0f));
    if (!bReportedMorph)
    {
        if (bSet)
        {
            UE_LOG(LogTemp, Log, TEXT("[VoxelBricks] Morph parameters go to %s (mode %d)."), *Collection->GetPathName(), Mode);
        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("[VoxelBricks] %s lacks a vector parameter VoxelMorphCameraHigh, VoxelMorphCameraLow or VoxelMorphParams (Design/MeshingEngine.md section 7)."),
                *Collection->GetName());
        }
        bReportedMorph = true;
    }
}

void FVoxelBrickTerrain::LogPerf(double Now)
{
    Perf.Frames++;
    if (Perf.Start == 0.0) Perf.Start = Now;
    if (Now - Perf.Start < 1.0) return;
    if (CVarVoxelBrickPerf.GetValueOnGameThread() > 0)
    {
        const double PerFrame = 1.0 / FMath::Max(1, Perf.Frames);
        const double PerTask = 1.0 / FMath::Max(1, Perf.Tasks);
        UE_LOG(LogTemp, Log, TEXT("[VoxelBrickPerf] %d frames in %.2f s. Game thread ms per frame (average / max): drain %.2f / %.2f, apply or commit %.2f / %.2f, remove %.2f / %.2f, flush pages %.2f / %.2f, launch %.2f / %.2f. Tasks: %d, %.1f ms average (tree %.1f), %.1f ms max, %lld samples, %d bricks built. %d held, %d wanted, %d queued to apply; commits wait %.0f ms (filtered)."),
            Perf.Frames, Now - Perf.Start,
            Perf.SectionMs[0] * PerFrame, Perf.MaxSectionMs[0], Perf.SectionMs[1] * PerFrame, Perf.MaxSectionMs[1], Perf.SectionMs[2] * PerFrame, Perf.MaxSectionMs[2],
            Perf.SectionMs[3] * PerFrame, Perf.MaxSectionMs[3], Perf.SectionMs[4] * PerFrame, Perf.MaxSectionMs[4],
            Perf.Tasks, Perf.TaskMs * PerTask, Perf.TreeMs * PerTask, Perf.MaxTaskMs, Perf.TaskSamples, Perf.TaskBuilt, Bricks.Num(), Wanted.Num(), ApplyQueue.Num(),
            CommitWaitSeconds * 1000.0);
        if (Backend == 1)
            UE_LOG(LogTemp, Log, TEXT("[VoxelBrickPerf] %s"), *Pages->TakePerf());
    }
    else if (Backend == 1)
    {
        Pages->TakePerf();
    }
    Perf = FPerf();
    Perf.Start = Now;
}

void FVoxelBrickTerrain::Tick(const FVector& Camera)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelBricksTick);
    const double Now = FPlatformTime::Seconds();
    const double Step = Now - LastTickTime;
    // A long frame or a jump of many bricks (a teleport) restarts the velocity.
    const double JumpDistance = VBT::MaxCameraJumpBricks * Settings.FinestCell * Settings.CellsPerBrick;
    if (LastTickTime > 0.0 && Step > 0.0 && Step < VBT::MaxCameraStep && FVector::DistSquared(Camera, LastCamera) < FMath::Square(JumpDistance))
    {
        const double Blend = 1.0 - FMath::Exp(-Step / VBT::VelocitySmoothing);
        CameraVelocity += ((Camera - LastCamera) / Step - CameraVelocity) * Blend;
        FrameSeconds += (Step - FrameSeconds) * Blend;
    }
    else
    {
        CameraVelocity = FVector::ZeroVector;
    }
    LastTickTime = Now;
    LastCamera = Camera;

    const int32 WantedBackend = ResolveBackend();
    if (WantedBackend != Backend)
    {
        Reset();
        Backend = WantedBackend;
    }

    // Each section timed for voxel.Debug.BrickPerf.
    double Mark = FPlatformTime::Seconds();
    auto Lap = [this, &Mark](int32 Section)
        {
            const double Now2 = FPlatformTime::Seconds();
            const double Ms = (Now2 - Mark) * 1000.0;
            Perf.SectionMs[Section] += Ms;
            Perf.MaxSectionMs[Section] = FMath::Max(Perf.MaxSectionMs[Section], Ms);
            Mark = Now2;
        };
    {
        SCOPE_CYCLE_COUNTER(STAT_VoxelBricksDrain);
        DrainResults();
    }
    Lap(0);
    const double TaskInterval = Backend == 1 ? VBT::TaskIntervalGPU : VBT::TaskInterval;
    if (Backend == 1)
    {
        // From a task's launch until its tree is drawn: the task, the wait for its commit,
        // and the frame it is committed in.
        TreeLatency = TaskSeconds + CommitWaitSeconds + FrameSeconds;
        {
            SCOPE_CYCLE_COUNTER(STAT_VoxelBricksFlush);
            FlushPages(TaskInterval);
        }
        Lap(3);
        {
            SCOPE_CYCLE_COUNTER(STAT_VoxelBricksApply);
            CommitPages();
        }
        Lap(1);
    }
    else
    {
        // The task, then the component update and its two frames.
        TreeLatency = TaskSeconds + 3.0 * FrameSeconds;
        {
            SCOPE_CYCLE_COUNTER(STAT_VoxelBricksApply);
            ApplyQueued();
        }
        Lap(1);
        {
            SCOPE_CYCLE_COUNTER(STAT_VoxelBricksRemove);
            RemoveReplaced();
        }
        Lap(2);
    }
    SET_DWORD_STAT(STAT_VoxelBricksHeld, Bricks.Num());
    SET_DWORD_STAT(STAT_VoxelBricksWanted, Wanted.Num());
    SET_DWORD_STAT(STAT_VoxelBricksQueued, ApplyQueue.Num());
    LogPerf(Now);

    if (bTaskRunning.load(std::memory_order_acquire) || bCommitPending || Now - LastLaunchTime < TaskInterval || !Density.IsValid()) return;
    SCOPE_CYCLE_COUNTER(STAT_VoxelBricksLaunch);
    ON_SCOPE_EXIT{ Lap(4); };

    // The tree is on screen from about TreeLatency after the launch until the next one is,
    // an interval later: build it for the camera predicted halfway through.
    const FVector TaskCamera = Camera + CameraVelocity * (TreeLatency + 0.5 * TaskInterval);

    // A task that finished since the drain above must be in Known (CPU) or committed (GPU)
    // before the next starts.
    DrainResults();
    if (bCommitPending) return;
    LastLaunchTime = Now;

    // Nothing to do: the latest result is complete for (about) this camera and nothing changed.
    // The GPU mesher is stitched only.
    const int32 SeamMode = Backend == 1 ? 0 : FMath::Clamp(CVarVoxelBrickSeams.GetValueOnGameThread(), 0, 2);
    if (bLastComplete && SeamMode == LastSeamMode && RecentEdits.Num() == 0
        && FVector::Dist(TaskCamera, LastTaskCamera) < Settings.FinestCell * Settings.CellsPerBrick / 16.0)
    {
        bool bAnyDirty = false;
        for (const TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
            bAnyDirty |= Pair.Value.bDirty;
        if (!bAnyDirty) return;
    }

    FTaskInput Input;
    Input.Generation = Generation;
    Input.Camera = TaskCamera;
    Input.Density = Density;
    Input.Settings = Settings;
    Input.SeamMode = SeamMode;
    Input.Backend = Backend;
    Input.PrevWanted = Wanted;
    Input.Known.Reserve(Bricks.Num());
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        // Dirty stays set until a rebuilt mesh arrives (DrainResults), so a rebuild of a brick
        // that is not wanted this pass is not lost.
        Input.Known.Add(Pair.Key, Pair.Value.bHasPending ? Pair.Value.PendingSignature : Pair.Value.Signature);
        if (Pair.Value.bDirty && Wanted.Contains(Pair.Key))
            Input.Rebuild.Add(Pair.Key);
        if (Pair.Value.bProvenEmpty)
            Input.ProvenEmpty.Add(Pair.Key);
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
    SCOPE_CYCLE_COUNTER(STAT_VoxelBricksTask);
    const double T0 = FPlatformTime::Seconds();
    FTaskResult Result;
    Result.Generation = Input.Generation;

    const TSharedPtr<FSparseEditStore> Edits = Input.Density->GetEditStore();
    Result.StartSequence = Edits.IsValid() ? Edits->GetLastSequence() : 0;

    // The previous tree's internal nodes: every ancestor of a brick it wanted.
    TSet<FVoxelBrickKey> SplitBefore;
    SplitBefore.Reserve(Input.PrevWanted.Num());
    for (const TPair<FVoxelBrickKey, uint64>& Pair : Input.PrevWanted)
    {
        FVoxelBrickKey Ancestor = Pair.Key;
        while (Ancestor.Level < Input.Settings.MaxLevel)
        {
            Ancestor = VBT::ParentOf(Ancestor);
            bool bAlready = false;
            SplitBefore.Add(Ancestor, &bAlready);
            if (bAlready) break;
        }
    }
    VBT::FTree Tree(*Input.Density, Edits.Get(), Input.Settings, Input.Camera, &SplitBefore);
    FVoxelBrickKey Root;
    Root.Level = Input.Settings.MaxLevel;
    Root.Coord = FInt64Vector(0, 0, 0);
    {
        SCOPE_CYCLE_COUNTER(STAT_VoxelBricksTaskTree);
        Tree.Descend(Root);
        if (Input.SeamMode != 2)
            Tree.Balance();
    }
    Result.BalanceSplits = Tree.BalanceSplits;
    Result.TreeMilliseconds = (FPlatformTime::Seconds() - T0) * 1000.0;

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
                    FBuilt& Built = Batch[i];
                    Built.Key = Entry.Key;
                    Built.Signature = Entry.Signature;

                    // GPU backend: only the empty test runs here; the page meshes the brick with
                    // the levels in its signature.
                    if (Input.Backend == 1)
                    {
                        int32 TestSamples = 0;
                        Built.bGPU = true;
                        Built.bProvenEmpty = !Entry.bEdited
                            && VoxelBrickMesher::IsBrickProvablyEmpty(*Input.Density, Grid, Entry.Key.Coord, Input.Settings.SlopeBound, TestSamples);
                        Samples.fetch_add(TestSamples, std::memory_order_relaxed);
                        return;
                    }
                    Tree.TouchingWanted(Entry.Key, Built.Neighbours);

                    FVoxelBrickMesh Mesh;
                    FVoxelBrickStats Stats;
                    if (Input.SeamMode == 2)
                        VoxelBrickMesher::MeshBrick(*Input.Density, Grid, Entry.Key.Coord, Input.Settings.SlopeBound, Entry.bEdited, Mesh, Stats);
                    else
                        VoxelBrickMesher::MeshBrickStitched(*Input.Density, Grid, Entry.Key.Coord, Entry.Levels, Entry.Meshed, Input.Settings.SlopeBound, Entry.bEdited, Mesh, Stats);
                    if (SkirtDepth > 0.0 && Mesh.Indices.Num() > 0)
                        VoxelBrickMesher::AddSkirts(Mesh, (float)(SkirtDepth * Grid.CellSize));

                    Built.Stream = VBT::BuildRenderStream(Mesh, Grid.BrickMin(Entry.Key.Coord), Entry.Key.Level, (float)Input.Settings.TriplanarUVScale);
                    Built.bProvenEmpty = Stats.bSkipped;
                    Samples.fetch_add(Stats.Samples, std::memory_order_relaxed);
                });
            Result.Built.Append(MoveTemp(Batch));
        };

    // Edit rebuilds always finish. New bricks are meshed nearest first while the budget lasts
    // (GPU backend: all of them, a result is committed whole); those left over are found
    // again next task.
    // GPU backend: one parallel batch (only the empty test runs here).
    const int32 BatchSize = Input.Backend == 1 ? FMath::Max(1, FMath::Max(NewBricks.Num(), Restitches.Num())) : VBT::MeshBatch;
    MeshAll(Rebuilds, 0, Rebuilds.Num());
    int32 NewMeshed = 0;
    for (int32 Start = 0; Start < NewBricks.Num() && (Input.Backend == 1 || Samples.load() < VBT::MaxSamplesPerTask); Start += BatchSize)
    {
        const int32 Count = FMath::Min(BatchSize, NewBricks.Num() - Start);
        MeshAll(NewBricks, Start, Count);
        NewMeshed += Count;
    }

    // The empty test reads neither the neighbour levels nor the meshed mask, so a brick it
    // skipped would be skipped again: it takes the new signature without meshing. A seam
    // mode change switches mesher, so those bricks are meshed again.
    TArray<FToMesh> Remesh;
    for (const FToMesh& Entry : Restitches)
    {
        const uint64 SeamBits = 3ULL << 56;
        if (!Input.ProvenEmpty.Contains(Entry.Key) || ((Input.Known[Entry.Key] ^ Entry.Signature) & SeamBits) != 0)
        {
            Remesh.Add(Entry);
            continue;
        }
        FBuilt& Built = Result.Built.AddDefaulted_GetRef();
        Built.Key = Entry.Key;
        Built.Signature = Entry.Signature;
        Built.bProvenEmpty = true;
        Built.bGPU = Input.Backend == 1;
        if (!Built.bGPU)
            Tree.TouchingWanted(Entry.Key, Built.Neighbours);
        Result.NumRestitchedEmpty++;
    }

    // Held bricks whose neighbours changed are restitched once every brick touching them is
    // held or built, so the restitched mesh never waits long on screen for its neighbours.
    TSet<FVoxelBrickKey> Available;
    for (const FBuilt& Built : Result.Built)
        Available.Add(Built.Key);
    TArray<FToMesh> Ready;
    TArray<FVoxelBrickKey> Touching;
    for (const FToMesh& Entry : Remesh)
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
    for (int32 Start = 0; Start < Ready.Num(); Start += BatchSize)
        MeshAll(Ready, Start, FMath::Min(BatchSize, Ready.Num() - Start));
    Result.Camera = Input.Camera;
    Result.SeamMode = Input.SeamMode;
    Result.bComplete = NewMeshed == NewBricks.Num() && Ready.Num() == Remesh.Num();

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
        bLastComplete = Result.bComplete;
        LastTaskCamera = Result.Camera;
        TaskSeconds += (Result.Milliseconds / 1000.0 - TaskSeconds) * 0.5;
        Perf.Tasks++;
        Perf.TaskMs += Result.Milliseconds;
        Perf.MaxTaskMs = FMath::Max(Perf.MaxTaskMs, Result.Milliseconds);
        Perf.TreeMs += Result.TreeMilliseconds;
        Perf.TaskSamples += Result.Samples;
        Perf.TaskBuilt += Result.Built.Num();
        LastSeamMode = Result.SeamMode;

        int32 NumWithSurface = 0;
        for (FBuilt& Built : Result.Built)
        {
            FBrick& Brick = Bricks.FindOrAdd(Built.Key);
            Brick.bProvenEmpty = Built.bProvenEmpty;
            if (Built.bGPU)
            {
                // Committed with the whole result (CommitPages).
                Brick.bRemesh = true;
                NumWithSurface += Built.bProvenEmpty ? 0 : 1;
            }
            else
            {
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
        if (Backend == 1)
        {
            // The whole difference in one frame; page by page only while nothing is on screen
            // (the first fill), where a partly filled tree hides nothing.
            bCommitProgressive = NumInPage == 0;
            bCommitPending = true;
            CommitPendingFrame = GFrameCounter;
            PendingChanges.Reset();
            BuildCommit(PendingChanges);
            // Meshed ahead of the commit, a few batches per frame.
            Pages->Stage(PendingChanges, bCommitProgressive);
        }

        if (CVarVoxelDebugBricks.GetValueOnGameThread() > 0)
        {
            UE_LOG(LogTemp, Log, TEXT("[VoxelBricks] %d wanted (%d balance splits), %d meshed (%d with surface, %d restitched, %d empty bricks restitched without meshing), %lld samples, %.1f ms, %d bricks held%s; camera %.0f m/s, tree latency %.0f ms"),
                Wanted.Num(), Result.BalanceSplits, Result.Built.Num() - Result.NumRestitchedEmpty, NumWithSurface, Result.NumRestitched, Result.NumRestitchedEmpty,
                Result.Samples, Result.Milliseconds, Bricks.Num(), Result.bComplete ? TEXT(", complete") : TEXT(""),
                CameraVelocity.Size() / 100.0, TreeLatency * 1000.0);
            if (Backend == 1)
            {
                UE_LOG(LogTemp, Log, TEXT("[VoxelBricks] %d page members committed; commits wait %.0f ms (filtered). %s"),
                    NumInPage, CommitWaitSeconds * 1000.0, *Pages->Describe());
            }
            else
            {
                int32 Waiting = 0;
                for (const FVoxelBrickKey& Queued : ApplyQueue)
                {
                    const FBrick* QueuedBrick = Bricks.Find(Queued);
                    Waiting += QueuedBrick && QueuedBrick->bHasPending && QueuedBrick->WaitFor.Num() > 0 ? 1 : 0;
                }
                UE_LOG(LogTemp, Log, TEXT("[VoxelBricks] Removal: %d held bricks not wanted, %d removed in the last pass; kept because a replacement or neighbour is not held %d, not applied %d, stitched to other neighbours %d, not yet drawn %d. %d meshes queued, %d waiting for neighbours"),
                    RemovalCandidates, RemovalRemoved, RemovalStalls[1], RemovalStalls[2], RemovalStalls[3], RemovalStalls[4], ApplyQueue.Num(), Waiting);
            }
        }
    }
}

void FVoxelBrickTerrain::BuildCommit(TArray<FVoxelBrickPageChange>& Out) const
{
    for (const TPair<FVoxelBrickKey, uint64>& Pair : Wanted)
    {
        // A brick no task has tested (possible only after a reset race) may hold surface.
        const FBrick* Brick = Bricks.Find(Pair.Key);
        const bool bInPage = Brick && Brick->bInPage;
        const bool bWantPage = !(Brick && Brick->bProvenEmpty);
        if (bWantPage && (!bInPage || Brick->Signature != Pair.Value || Brick->bRemesh))
        {
            FVoxelBrickPageChange& Change = Out.AddDefaulted_GetRef();
            Change.Key = Pair.Key;
            VBT::DecodeSignature(Pair.Value, Change.Levels, Change.Meshed);
        }
        else if (!bWantPage && bInPage)
        {
            FVoxelBrickPageChange& Change = Out.AddDefaulted_GetRef();
            Change.Key = Pair.Key;
            Change.bRemove = true;
        }
    }
    for (const TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        if (Pair.Value.bInPage && !Wanted.Contains(Pair.Key))
        {
            FVoxelBrickPageChange& Change = Out.AddDefaulted_GetRef();
            Change.Key = Pair.Key;
            Change.bRemove = true;
        }
    }
}

void FVoxelBrickTerrain::ApplyCommitted(TConstArrayView<FVoxelBrickPageChange> Changes)
{
    for (const FVoxelBrickPageChange& Change : Changes)
    {
        FBrick& Brick = Bricks.FindOrAdd(Change.Key);
        if (Change.bRemove)
        {
            NumInPage -= Brick.bInPage ? 1 : 0;
            Brick.bInPage = false;
            continue;
        }
        NumInPage += Brick.bInPage ? 0 : 1;
        Brick.bInPage = true;
        Brick.bApplied = true;
        Brick.bRemesh = false;
        Brick.Signature = Wanted.FindRef(Change.Key);
    }
}

void FVoxelBrickTerrain::CommitPages()
{
    if (!bCommitPending) return;
    if (!bCommitProgressive)
    {
        if (PendingChanges.Num() > 0)
        {
            if (!Pages->PrepareCommit(PendingChanges)) return;
            Pages->Commit(PendingChanges);
            ApplyCommitted(PendingChanges);
        }
    }
    else
    {
        // Every page whose bricks are meshed and whose buffer sets are ready, committed together;
        // pages that only lose bricks once every addition is in.
        TMap<FVoxelBrickKey, TArray<FVoxelBrickPageChange>> Groups;
        for (const FVoxelBrickPageChange& Change : PendingChanges)
            Groups.FindOrAdd(FVoxelBrickPages::PageOf(Change.Key)).Add(Change);
        TArray<FVoxelBrickPageChange> Ready;
        TArray<FVoxelBrickPageChange> Left;
        for (int32 Pass = 0; Pass < 2; Pass++)
        {
            const bool bAdditionsLeft = Left.Num() > 0;
            for (const TPair<FVoxelBrickKey, TArray<FVoxelBrickPageChange>>& Group : Groups)
            {
                bool bAdds = false;
                for (const FVoxelBrickPageChange& Change : Group.Value)
                    bAdds |= !Change.bRemove;
                if (bAdds != (Pass == 0)) continue;
                if (!bAdditionsLeft && Pages->PreparePages(Group.Value))
                    Ready.Append(Group.Value);
                else
                    Left.Append(Group.Value);
            }
        }
        if (Ready.Num() > 0 && Pages->PrepareCommit(Ready))
        {
            Pages->Commit(Ready);
            ApplyCommitted(Ready);
        }
        else
        {
            Left.Append(Ready);
        }
        PendingChanges = MoveTemp(Left);
        if (PendingChanges.Num() > 0) return;
    }

    // The tree on screen is Wanted.
    for (auto It = Bricks.CreateIterator(); It; ++It)
    {
        const uint64* Signature = Wanted.Find(It.Key());
        if (!Signature)
        {
            It.RemoveCurrent();
            continue;
        }
        It.Value().Signature = *Signature;
        It.Value().bApplied = true;
        It.Value().bRemesh = false;
    }
    bCommitPending = false;
    PendingChanges.Reset();
    const double Wait = (double)(GFrameCounter - CommitPendingFrame) * FrameSeconds;
    CommitWaitSeconds += (Wait - CommitWaitSeconds) * VBT::CommitWaitBlend;
}

bool FVoxelBrickTerrain::IsNeighbourReady(const FVoxelBrickKey& Key) const
{
    return !Wanted.Contains(Key) || IsOnScreen(Key);
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
            bReady &= IsNeighbourReady(Neighbour);
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
        // until the end-of-frame update after its creation, so the mesh counts from two frames
        // after that (or after the apply, when it found no proxy). Same rule as
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
    if (Brick->bInPage) return true;                // committed: drawn from the commit's frame
    if (!Brick->Component.IsValid()) return true;   // applied empty: nothing to show
    const uint64 Frame = Brick->ShownFrame.IsValid() ? Brick->ShownFrame->load(std::memory_order_acquire) : 0;
    return Frame != 0 && GFrameCounter >= Frame;
}

bool FVoxelBrickTerrain::HoldsSurface(const FVoxelBrickKey& Key) const
{
    const FBrick* Brick = Bricks.Find(Key);
    return Brick && (Brick->Component.IsValid() || Brick->bInPage);
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

int32 FVoxelBrickTerrain::StallOf(const FVoxelBrickKey& Key) const
{
    const FBrick* Brick = Bricks.Find(Key);
    const uint64* Signature = Wanted.Find(Key);
    if (!Brick) return 1;
    if (!Signature || !Brick->bApplied) return 2;
    if (Brick->Signature != *Signature) return 3;
    if (!IsOnScreen(Key)) return 4;
    return 0;
}

bool FVoxelBrickTerrain::IsCovered(const FVoxelBrickKey& Key, const TArray<FVoxelBrickKey>& Descendants, int32* OutStall) const
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
        if (!IsSettled(Replacement))
        {
            if (OutStall) *OutStall = StallOf(Replacement);
            return false;
        }
    }
    TArray<FVoxelBrickKey> Around;
    for (const FVoxelBrickKey& Replacement : Replacements)
    {
        Around.Reset();
        TouchingWanted(Replacement, Around);
        for (const FVoxelBrickKey& Neighbour : Around)
        {
            if (!IsSettled(Neighbour))
            {
                if (OutStall) *OutStall = StallOf(Neighbour);
                return false;
            }
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
    if (Candidates.Num() == 0)
    {
        RemovalCandidates = 0;
        RemovalRemoved = 0;
        FMemory::Memzero(RemovalStalls);
        return;
    }
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
    FMemory::Memzero(RemovalStalls);
    for (const TPair<FVoxelBrickKey, TArray<FVoxelBrickKey>>& Candidate : Candidates)
    {
        int32 Stall = 0;
        if (IsCovered(Candidate.Key, Candidate.Value, &Stall))
            Remove.Add(Candidate.Key);
        else
            RemovalStalls[FMath::Clamp(Stall, 0, 4)]++;
    }
    RemovalCandidates = Candidates.Num();
    RemovalRemoved = Remove.Num();
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