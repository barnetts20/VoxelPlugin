#include "VoxelBrickTerrain.h"
#include "VoxelBrickPages.h"
#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "VoxelPlugin.h"
#include "FDensitySampleCompositor.h"
#include "FSparseEditStore.h"
#include "VoxelEditCore.h"
#include "Misc/ScopeLock.h"
#include "Async/ParallelFor.h"
#include "Async/TaskGraphInterfaces.h"
#include "GameFramework/Actor.h"
#include "HAL/IConsoleManager.h"
#include "Misc/App.h"
#include "Misc/ScopeExit.h"
#include "Engine/World.h"
#include "Materials/MaterialParameterCollection.h"
#include "Materials/MaterialParameterCollectionInstance.h"
#include "Engine/Engine.h"
#include "VoxelTerrainStats.h"
#include "VoxelShaderCompat.h"

namespace VoxelShader
{
#include "VoxelLandDebugPalette.ush"
}

DECLARE_CYCLE_STAT(TEXT("Bricks: tick"), STAT_VoxelBricksTick, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: drain task results"), STAT_VoxelBricksDrain, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: commit"), STAT_VoxelBricksApply, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: flush pages"), STAT_VoxelBricksFlush, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: launch task"), STAT_VoxelBricksLaunch, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: task (worker)"), STAT_VoxelBricksTask, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Bricks: task tree (worker)"), STAT_VoxelBricksTaskTree, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Bricks held"), STAT_VoxelBricksHeld, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Bricks wanted"), STAT_VoxelBricksWanted, STATGROUP_VoxelTerrain);

TAutoConsoleVariable<int32> CVarVoxelDebugLevelColor(
    TEXT("voxel.Debug.LevelColor"), 0,
    TEXT("1: vertex colour of the terrain bricks by level (the material must show vertex colour; a change meshes the terrain again). 0: white."),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarVoxelDebugLandViewSource(
    TEXT("voxel.Debug.LandViewSource"), 0,
    TEXT("Where voxel.Debug.LandView colours terrestrial terrain. 0: per pixel, in the terrain material's land node (VoxelLandMaterial.ush; STAGE_M1_NOTES.md): a view change is instant (where the material has no land node, or it isn't bound, the vertex colours show it). 1: per vertex, in the vertex colour with alpha 0 (STAGE_L1b_NOTES.md): a view change meshes the terrain again. The noise surface always uses the vertex colour."),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarVoxelDebugLandView(
    TEXT("voxel.Debug.LandView"), 0,
    TEXT("Colours the terrain by a component of the generator: per pixel in the terrain material's land node, or per vertex in the vertex colour (voxel.Debug.LandViewSource). A legend is on screen while a view is on.\n")
    TEXT(" 0: off\n")
    TEXT(" 1: elevation about sea level (hypsometric; 500 m bands on land, 1 km at sea, close up)\n")
    TEXT(" 2: the bake's height alone (no gullies or detail)\n")
    TEXT(" 3: the detail layer (blue down, red up, over its amplitude)\n")
    TEXT(" 4: crust (oceanic blue to continental tan; the threshold yellow)\n")
    TEXT(" 5: boundary class (red collision, orange arc range, purple trench, yellow island arc, cyan rift, green ridge)\n")
    TEXT(" 6: plates (a hue each, brighter on continental crust)\n")
    TEXT(" 7: slope (green flat, yellow 20 deg, red 45 deg, purple 60+)\n")
    TEXT(" 8: the bake's cells (a checker in each cube face's colour)\n")
    TEXT(" 9: drainage (rivers: blue from 1,000 km^2)\n")
    TEXT(" 10: lakes by depth\n")
    TEXT(" 11: the gullies (blue down, red up, over their local depth)\n")
    TEXT(" 12: lakes and rivers (blue over neutral ground)\n")
    TEXT(" 13: elevation with the lakes and rivers over it\n")
    TEXT(" 14: the landform (flat plains straw, rolling ground green, hill country dark green to brown by roughness)\n")
    TEXT(" 15: the hills layer (white none, green to brown over HillRelief)\n")
    TEXT(" 16: the peaks layer (blue valleys, red summits, over their local relief)\n")
    TEXT(" 17: the peaks' relief (white none, green to brown, dark at 3 km: mountain crests)\n")
    TEXT(" 18: the sedimentary province (grey, lighter sedimentary) and where the strata replace the hills (orange)\n")
    TEXT(" 19: the rivers (blue channels, darker for larger rivers, a pale wash over their valleys) over the elevation\n")
    TEXT("2 to 6 and 8 to 19 need a terrestrial planet, 9, 10, 12, 13 and 19 a GPU bake (magenta otherwise)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelBrickPerf(
    TEXT("voxel.Debug.BrickPerf"), 0,
    TEXT("1: once a second, log where the brick terrain spent its time (game thread sections, tasks, page work and render-thread time)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugBricks(
    TEXT("voxel.Debug.Bricks"), 0,
    TEXT("1: log each brick terrain task (wanted, tested, samples, time) and the pages."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelBrickMorph(
    TEXT("voxel.Render.BrickMorph"), 1,
    TEXT("Geomorphing of the brick terrain, in the terrain material (VoxelMorph.ush, through the actor's Brick Morph Parameters collection). 1: vertices blend toward the next coarser level with distance, so LOD changes don't pop. 0: off. 2: fully blended everywhere (shows the coarser levels; for checking)."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelBrickPrediction(
    TEXT("voxel.Render.BrickPrediction"), 0.75f,
    TEXT("How far ahead of the camera the brick terrain builds its tree, as a fraction of the time from a task's launch to the middle of its tree's time on screen (both measured). 1: centred on the camera in steady flight; lower overshoots less when the camera stops suddenly but trails further in flight, where the morph bands move nearer to match. 0: no prediction."),
    ECVF_Default);

namespace VoxelBrickTerrainDetail
{
    constexpr double TaskInterval = 0.05;           // seconds between tasks (meshing costs the task nothing)
    constexpr double VelocitySmoothing = 0.15;      // seconds: camera velocity filter
    constexpr double VelocityRelease = 0.04;        // seconds: the prediction's, while slowing down
    constexpr double MaxCameraStep = 0.5;           // seconds: a longer frame resets the velocity
    constexpr double MaxCameraJumpBricks = 8.0;     // finest bricks: a longer step in a frame too
    /** A brick split in the previous tree stays split until this many times the split
     *  distance. Free for the morph: level-l vertices still meet level l + 1 only beyond the
     *  split distance (Design/GPUPath.md section 4), and up to 1.15 times it level l + 1
     *  is unblended where level l is fully blended, for any F below 11. */
    constexpr double MergeHysteresis = 1.15;
    /** At or above this LodDistanceFactor F the tree split by distance alone is balanced: a
     *  leaf of level l (no nearer than F B_l) touching a split node of level l - 1 (nearer
     *  than 1.15 F B_l / 2) lies within that node's diagonal, sqrt(3) B_l / 2, of it, which
     *  needs F (1 - 1.15 / 2) < sqrt(3) / 2. Below it the tree is balanced explicitly. */
    constexpr double BalancedFactor = UE_DOUBLE_SQRT_3 / (2.0 - MergeHysteresis);
    constexpr int32 DescentChunk = 64;              // bricks per parallel batch of the descent (one density batch)
    constexpr double CommitWaitBlend = 0.5;          // filter of the commit wait
    constexpr double LatencyBlend = 0.5;             // filter of the measured tree latency
    /** Bricks a debug-colour change meshes again per commit: the whole terrain in one commit
     *  would need all of it in the mesh cache at once. */
    constexpr int32 RemeshChunkBricks = 512;

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

    enum class ENode : uint8
    {
        Internal,
        Wanted,     // leaf that may hold surface
        Culled      // leaf proven free of surface
    };

    struct FWantedInfo
    {
        bool bEdited;   // an edit reaches the brick: the lattice empty test is skipped (its samples
                        // read their own cells' edits; IsBrickProvablyEmpty)
    };

    /** What the descent does with a brick. */
    enum class EVisit : uint8
    {
        Cull,
        Split,
        Want
    };

    /** The LOD tree of one task, over the whole root: every internal node has all eight
     *  children, every leaf is wanted or culled. */
    struct FTree
    {
        FTree(const FDensitySampleCompositor& InDensity, const FSparseEditStore* InEdits, const FVoxelBrickTerrainSettings& InSettings, const FVector& InCamera,
            const TSet<FVoxelBrickKey>* InSplitBefore)
            : Density(InDensity), Edits(InEdits), Settings(InSettings), Camera(InCamera), SplitBefore(InSplitBefore)
        {
            // The edit-reach test (IsEdited) reads one version of the edits without a lock per
            // brick; the density samples pin the store's latest each batch (the task read its
            // sequence first, so they are at least as new).
            if (Edits)
            {
                EditState = Edits->GetSnapshot();
                EditCenter = Edits->GetCenter();
            }
        }

        const FDensitySampleCompositor& Density;
        const FSparseEditStore* Edits;
        TSharedPtr<const VoxelEditCore::FState, ESPMode::ThreadSafe> EditState;
        FVector EditCenter = FVector::ZeroVector;
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
            const FVoxelBrickGrid Grid = Settings.GridFor(Key.Level);
            const FVector Min = Grid.BrickMin(Key.Coord);
            return FMath::Sqrt(BoxDistanceSquared(FBox(Min, Min + FVector(Grid.BrickSize())), Camera));
        }

        /** The region a brick's centre sample must prove free of surface: its cells and the
         *  cells around them its stitched mesh reads. */
        static FBox TestRegion(const FVoxelBrickGrid& Grid, const FVoxelBrickKey& Key)
        {
            const double H = Grid.CellSize;
            const FVector Min = Grid.BrickMin(Key.Coord);
            return FBox(Min - FVector(2.0 * H), Min + FVector(Grid.BrickSize() + H));
        }

        bool IsEdited(const FBox& Region) const
        {
            if (!EditState.IsValid() || EditState->Ops.empty()) return false;
            VoxelEditCore::FAabb Box;
            for (int32 a = 0; a < 3; a++)
            {
                Box.Min[a] = Region.Min[a] - EditCenter[a];
                Box.Max[a] = Region.Max[a] - EditCenter[a];
            }
            Box.bValid = true;
            return EditState->AnyInBox(Box);
        }

        /** False when the brick's centre sample proves it free of surface. The edits are as
         *  slope-bounded as the land, so edited bricks are tested too: the sample applies every
         *  edit reaching the region (SampleWithin), not only its own index cell's. */
        bool MayHoldSurface(const FVoxelBrickKey& Key, bool& bOutEdited)
        {
            const FVoxelBrickGrid Grid = Settings.GridFor(Key.Level);
            const FBox Region = TestRegion(Grid, Key);
            bOutEdited = IsEdited(Region);

            const FVector Centre = Region.GetCenter();
            const double Radius = Region.GetExtent().Size();
            double CX = Centre.X, CY = Centre.Y, CZ = Centre.Z;
            float D = 0.0f;
            Density.SampleWithin(FSampleInput(&CX, &CY, &CZ, 1, Grid.CellSize), Radius, &D);
            Samples++;
            return FMath::Abs((double)D) <= Settings.SlopeBound * Radius;
        }

        /** Thread safe. The descent's verdict on bricks of one level: culled when the centre
         *  sample proves the brick free of surface (one density batch for all of them), split
         *  while the camera is close, wanted otherwise. */
        void VisitChunk(TConstArrayView<FVoxelBrickKey> Keys, TArrayView<EVisit> OutVisits, TArrayView<FWantedInfo> OutInfos, int64& OutSamples) const
        {
            const FVoxelBrickGrid Grid = Settings.GridFor(Keys[0].Level);
            const double SplitDistance = Settings.LodDistanceFactor * Grid.BrickSize();
            const double Radius = TestRegion(Grid, Keys[0]).GetExtent().Size();
            const double Bound = Settings.SlopeBound * Radius;
            TArray<double, TInlineAllocator<DescentChunk>> X, Y, Z;
            TArray<int32, TInlineAllocator<DescentChunk>> Sampled;
            for (int32 i = 0; i < Keys.Num(); i++)
            {
                const FBox Region = TestRegion(Grid, Keys[i]);
                OutVisits[i] = EVisit::Want;
                OutInfos[i].bEdited = IsEdited(Region);
                const FVector Centre = Region.GetCenter();
                X.Add(Centre.X);
                Y.Add(Centre.Y);
                Z.Add(Centre.Z);
                Sampled.Add(i);
            }
            if (Sampled.Num() > 0)
            {
                TArray<float, TInlineAllocator<DescentChunk>> D;
                D.SetNumUninitialized(Sampled.Num());
                Density.SampleWithin(FSampleInput(X.GetData(), Y.GetData(), Z.GetData(), Sampled.Num(), Grid.CellSize), Radius, D.GetData());
                for (int32 s = 0; s < Sampled.Num(); s++)
                {
                    if (FMath::Abs((double)D[s]) > Bound)
                        OutVisits[Sampled[s]] = EVisit::Cull;
                }
                OutSamples += Sampled.Num();
            }
            for (int32 i = 0; i < Keys.Num(); i++)
            {
                if (OutVisits[i] == EVisit::Cull) continue;
                const double Distance = DistanceTo(Keys[i]);
                const bool bWasSplit = SplitBefore && SplitBefore->Contains(Keys[i]);
                if (Keys[i].Level > 0 && Distance < (bWasSplit ? MergeHysteresis * SplitDistance : SplitDistance))
                    OutVisits[i] = EVisit::Split;
            }
        }

        /** The LOD descent from the root, a level at a time with each level's bricks in
         *  parallel. */
        void Descend(const FVoxelBrickKey& Root)
        {
            TArray<FVoxelBrickKey> Level = { Root };
            TArray<FVoxelBrickKey> Next;
            TArray<EVisit> Visits;
            TArray<FWantedInfo> Infos;
            TArray<int64> ChunkSamples;
            while (Level.Num() > 0)
            {
                const int32 NumKeys = Level.Num();
                const int32 NumChunks = FMath::DivideAndRoundUp(NumKeys, DescentChunk);
                Visits.SetNumUninitialized(NumKeys);
                Infos.SetNumUninitialized(NumKeys);
                ChunkSamples.Init(0, NumChunks);
                ParallelFor(NumChunks, [&](int32 Chunk)
                    {
                        const int32 First = Chunk * DescentChunk;
                        const int32 Count = FMath::Min(DescentChunk, NumKeys - First);
                        VisitChunk(MakeArrayView(Level.GetData() + First, Count), MakeArrayView(Visits.GetData() + First, Count),
                            MakeArrayView(Infos.GetData() + First, Count), ChunkSamples[Chunk]);
                    }, NumChunks > 1 ? EParallelForFlags::None : EParallelForFlags::ForceSingleThread);
                for (const int64 ChunkCount : ChunkSamples)
                    Samples += ChunkCount;

                Next.Reset();
                for (int32 i = 0; i < NumKeys; i++)
                {
                    if (Visits[i] == EVisit::Cull)
                    {
                        Nodes.Add(Level[i], ENode::Culled);
                    }
                    else if (Visits[i] == EVisit::Want)
                    {
                        Nodes.Add(Level[i], ENode::Wanted);
                        WantedInfo.Add(Level[i], Infos[i]);
                    }
                    else
                    {
                        Nodes.Add(Level[i], ENode::Internal);
                        for (int32 c = 0; c < 8; c++)
                            Next.Add(Level[i].Child(c));
                    }
                }
                Swap(Level, Next);
            }
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
                const FVoxelBrickKey Child = Key.Child(c);
                bool bEdited = false;
                if (Was == ENode::Wanted && MayHoldSurface(Child, bEdited))
                {
                    Nodes.Add(Child, ENode::Wanted);
                    WantedInfo.Add(Child, { bEdited });
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
                Cover = Cover.Parent();
            return Cover;
        }

        /** Splits leaves until every two leaves that touch (faces, edges or corners) are at
         *  most one level apart, over wanted and culled leaves alike. Only needed below
         *  BalancedFactor. */
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
                    const FVoxelBrickKey Neighbour = Key.Offset(d % 3 - 1, (d / 3) % 3 - 1, d / 9 - 1);
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

        /** Whether a child of Node on the side facing back along Dir (touching the brick Node
         *  neighbours in direction Dir) may hold surface. */
        bool TouchesUnculledChild(const FVoxelBrickKey& Node, const int32 (&Dir)[3]) const
        {
            for (int32 c = 0; c < 8; c++)
            {
                bool bTouches = true;
                for (int32 Axis = 0; Axis < 3; Axis++)
                {
                    const int32 Bit = (c >> Axis) & 1;
                    bTouches &= Dir[Axis] == 0 || (Dir[Axis] < 0 ? Bit == 1 : Bit == 0);
                }
                if (bTouches && Nodes.FindRef(Node.Child(c)) != ENode::Culled)
                    return true;
            }
            return false;
        }

        /** Level of the leaves touching Key in direction d, relative to Key, and whether that
         *  neighbour is a wanted brick of Key's level. Wanted leaves are balanced. Where only
         *  culled leaves touch Key, their boxes are proven free of surface, so Key's mesh is
         *  the same whatever their level: Key's own is reported, and the signature doesn't
         *  change with theirs. */
        int32 NeighbourDelta(const FVoxelBrickKey& Key, int32 d, bool& bOutMeshed) const
        {
            bOutMeshed = false;
            const int32 Dir[3] = { d % 3 - 1, (d / 3) % 3 - 1, d / 9 - 1 };
            const FVoxelBrickKey Neighbour = Key.Offset(Dir[0], Dir[1], Dir[2]);
            if (!InRoot(Neighbour)) return 0;
            if (const ENode* State = Nodes.Find(Neighbour))
            {
                bOutMeshed = *State == ENode::Wanted;
                return *State == ENode::Internal && TouchesUnculledChild(Neighbour, Dir) ? -1 : 0;
            }
            return Nodes.FindRef(CoveringNode(Neighbour)) == ENode::Wanted ? 1 : 0;
        }

    };

    FColor LevelColor(int32 Level)
    {
        static const FColor Palette[8] = {
            FColor(230, 80, 80), FColor(240, 170, 60), FColor(230, 230, 80), FColor(110, 210, 90),
            FColor(70, 200, 200), FColor(80, 130, 240), FColor(170, 100, 230), FColor(230, 110, 190) };
        return Palette[Level % 8];
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

int32 FVoxelBrickTerrain::GetLandView()
{
    const int32 View = CVarVoxelDebugLandView.GetValueOnGameThread();
    return View > 0 && View < (int32)VOXEL_LAND_VIEW_COUNT ? View : 0;
}

int32 FVoxelBrickTerrain::GetVertexLandView(bool bMaterialLandViews)
{
    return !bMaterialLandViews || CVarVoxelDebugLandViewSource.GetValueOnGameThread() != 0 ? GetLandView() : 0;
}

int32 FVoxelBrickTerrain::GetMaterialLandView()
{
    return CVarVoxelDebugLandViewSource.GetValueOnGameThread() == 0 ? GetLandView() : 0;
}

void FVoxelBrickTerrain::SetMaterialLandViews(bool bInMaterialLandViews)
{
    // TickDebugViews meshes the terrain again if the vertex view changes with it.
    Settings.bMaterialLandViews = bInMaterialLandViews;
    Pages->SetMaterialLandViews(bInMaterialLandViews);
}

void FVoxelBrickTerrain::SetDebugSeaRadius(double InSeaRadius)
{
    if (Settings.DebugSeaRadius == InSeaRadius) return;
    Settings.DebugSeaRadius = InSeaRadius;
    Pages->SetDebugSeaRadius(InSeaRadius);
    if (GetVertexLandView(Settings.bMaterialLandViews) == (int32)VOXEL_LAND_VIEW_ELEVATION)
        RemeshAll();
}

void FVoxelBrickTerrain::RemeshAll()
{
    RemeshQueue.Reset();
    RemeshHead = 0;
    for (const TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        if (Pair.Value.bInPage && Wanted.Contains(Pair.Key))
            RemeshQueue.Add(Pair.Key);
    }
    // A pending commit's bricks may be meshed with the old colours already: staged again.
    if (bCommitPending && PendingChanges.Num() > 0)
        StagePending();
}

void FVoxelBrickTerrain::FeedRemesh()
{
    if (RemeshHead >= RemeshQueue.Num())
    {
        RemeshQueue.Reset();
        RemeshHead = 0;
        return;
    }
    // The next chunk once the previous one (or an edit, or a result) is committed.
    if (bCommitPending || EditBacklog.Num() > 0) return;
    int32 Added = 0;
    while (RemeshHead < RemeshQueue.Num() && Added < VBT::RemeshChunkBricks)
    {
        const FVoxelBrickKey Key = RemeshQueue[RemeshHead++];
        const FBrick* Brick = Bricks.Find(Key);
        if (!Brick || !Brick->bInPage || !Wanted.Contains(Key)) continue;
        EditBacklog.Add(Key);
        Added++;
    }
}

namespace VoxelBrickTerrainDetail
{
    /** The legend of a land view: a title, then one line per colour, each in its colour. */
    struct FLegendLine
    {
        FString Text;
        FColor Color;
    };

    FColor ToColor(const VoxelShader::float3& C)
    {
        return FColor((uint8)FMath::RoundToInt(FMath::Clamp(C.x, 0.0f, 1.0f) * 255.0f), (uint8)FMath::RoundToInt(FMath::Clamp(C.y, 0.0f, 1.0f) * 255.0f),
            (uint8)FMath::RoundToInt(FMath::Clamp(C.z, 0.0f, 1.0f) * 255.0f));
    }

    TArray<FLegendLine> LandViewLegend(int32 View, bool bTerrestrial, bool bPerPixel)
    {
        using namespace VoxelShader;
        static const TCHAR* Names[VOXEL_LAND_VIEW_COUNT] = { TEXT("off"), TEXT("elevation"), TEXT("bake height"), TEXT("detail layer"),
            TEXT("crust"), TEXT("boundary class"), TEXT("plates"), TEXT("slope"), TEXT("bake cells"), TEXT("drainage"), TEXT("lakes"), TEXT("gullies"),
            TEXT("lakes and rivers"), TEXT("elevation, lakes and rivers"), TEXT("landform"), TEXT("hills"), TEXT("peaks"), TEXT("peak amplitude"),
            TEXT("strata province"), TEXT("rivers") };
        TArray<FLegendLine> Lines;
        Lines.Add({ FString::Printf(TEXT("voxel.Debug.LandView %d: %s (0 hides it; %s)"), View, Names[View],
            bPerPixel
                ? TEXT("per pixel, in the terrain material's land node")
                : bTerrestrial && CVarVoxelDebugLandViewSource.GetValueOnGameThread() == 0
                ? TEXT("per vertex: the terrain material has no land node bound (STAGE_M1_NOTES.md), so it must show vertex colour")
                : TEXT("per vertex: the terrain material must show vertex colour")), FColor::White });
        const bool bBakeView = View != (int32)VOXEL_LAND_VIEW_ELEVATION && View != (int32)VOXEL_LAND_VIEW_SLOPE;
        if (bBakeView && !bTerrestrial)
        {
            Lines.Add({ TEXT("  needs a terrestrial planet (Shape > Generator): magenta here"), ToColor(VoxelLandViewNone()) });
            return Lines;
        }
        auto Add = [&Lines](const TCHAR* Text, const float3& C) { Lines.Add({ FString(TEXT("  ")) + Text, ToColor(C) }); };
        switch ((uint32)View)
        {
        case VOXEL_LAND_VIEW_ELEVATION:
        case VOXEL_LAND_VIEW_BAKE:
        {
            static const float Heights[] = { -7000.0f, -3000.0f, -800.0f, -100.0f, 0.0f, 300.0f, 800.0f, 1500.0f, 2500.0f, 4000.0f, 6000.0f };
            static const TCHAR* Labels[] = { TEXT("-7 km"), TEXT("-3 km"), TEXT("-800 m"), TEXT("-100 m (shelf)"), TEXT("0 m"), TEXT("300 m"),
                TEXT("800 m"), TEXT("1.5 km"), TEXT("2.5 km"), TEXT("4 km"), TEXT("6 km and up") };
            for (int32 i = 0; i < (int32)UE_ARRAY_COUNT(Heights); i++)
                Add(Labels[i], VoxelLandViewHeight(Heights[i], 1.0e9f));
            break;
        }
        case VOXEL_LAND_VIEW_DETAIL:
            Add(TEXT("-1 (down by the full amplitude)"), VoxelLandViewSigned(-1.0f));
            Add(TEXT("-0.5"), VoxelLandViewSigned(-0.5f));
            Add(TEXT("0"), VoxelLandViewSigned(0.0f));
            Add(TEXT("+0.5"), VoxelLandViewSigned(0.5f));
            Add(TEXT("+1 (up; a quarter as strong under the sea, scaled by GullyDetailScale on land)"), VoxelLandViewSigned(1.0f));
            break;
        case VOXEL_LAND_VIEW_GULLY:
            Add(TEXT("-1 (a gully floor: down by the coarsest octave's local half-depth)"), VoxelLandViewSigned(-1.0f));
            Add(TEXT("-0.5"), VoxelLandViewSigned(-0.5f));
            Add(TEXT("0 (none: flats, the sea, valley floors, or too coarse a level)"), VoxelLandViewSigned(0.0f));
            Add(TEXT("+0.5"), VoxelLandViewSigned(0.5f));
            Add(TEXT("+1 (a spur between gullies)"), VoxelLandViewSigned(1.0f));
            Add(TEXT("magenta: no gullies (GullyStrength 0, or GullyFinestWavelength above two bake cells)"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_CRUST:
            Add(TEXT("0 oceanic"), VoxelLandViewCrust(0.0f));
            Add(TEXT("0.25"), VoxelLandViewCrust(0.25f));
            Add(TEXT("0.5 the threshold (coasts lie near it)"), VoxelLandViewCrust(0.5f));
            Add(TEXT("0.75"), VoxelLandViewCrust(0.75f));
            Add(TEXT("1 continental"), VoxelLandViewCrust(1.0f));
            break;
        case VOXEL_LAND_VIEW_CLASS:
            Add(TEXT("no boundary (oceanic crust)"), VoxelLandViewClass(0u, 0.0f));
            Add(TEXT("no boundary (continental crust)"), VoxelLandViewClass(0u, 1.0f));
            Add(TEXT("collision range"), VoxelLandViewClass(1u, 0.0f));
            Add(TEXT("arc range (ocean under a continent)"), VoxelLandViewClass(2u, 0.0f));
            Add(TEXT("trench"), VoxelLandViewClass(3u, 0.0f));
            Add(TEXT("island arc"), VoxelLandViewClass(4u, 0.0f));
            Add(TEXT("rift"), VoxelLandViewClass(5u, 0.0f));
            Add(TEXT("mid-ocean ridge"), VoxelLandViewClass(6u, 0.0f));
            break;
        case VOXEL_LAND_VIEW_PLATES:
            for (uint32 p = 0; p < 4u; p++)
                Add(*FString::Printf(TEXT("plate %u (continental crust)"), p), VoxelLandViewPlate(p, 1.0f));
            Add(TEXT("plate 0 (oceanic crust): darker"), VoxelLandViewPlate(0u, 0.0f));
            break;
        case VOXEL_LAND_VIEW_SLOPE:
            Add(TEXT("0 deg"), VoxelLandViewSlope(0.0f));
            Add(TEXT("10 deg"), VoxelLandViewSlope(10.0f));
            Add(TEXT("20 deg"), VoxelLandViewSlope(20.0f));
            Add(TEXT("30 deg"), VoxelLandViewSlope(30.0f));
            Add(TEXT("45 deg"), VoxelLandViewSlope(45.0f));
            Add(TEXT("60 deg and up"), VoxelLandViewSlope(60.0f));
            break;
        case VOXEL_LAND_VIEW_CELLS:
        {
            static const TCHAR* Faces[] = { TEXT("face +X"), TEXT("face -X"), TEXT("face +Y"), TEXT("face -Y"), TEXT("face +Z"), TEXT("face -Z") };
            for (uint32 f = 0; f < 6u; f++)
                Add(Faces[f], VoxelLandViewFace(f));
            Add(TEXT("alternate cells darker (close up)"), VoxelLandViewCell(0u, 1, 0, 1.0f));
            break;
        }
        case VOXEL_LAND_VIEW_FLOW:
            Add(TEXT("hillslope"), VoxelLandViewFlow(0.0f, false));
            Add(TEXT("10 km^2"), VoxelLandViewFlow(1.0f, false));
            Add(TEXT("100 km^2"), VoxelLandViewFlow(2.0f, false));
            Add(TEXT("1,000 km^2 (river)"), VoxelLandViewFlow(3.0f, false));
            Add(TEXT("100,000 km^2"), VoxelLandViewFlow(5.0f, false));
            Add(TEXT("sea"), VoxelLandViewFlow(0.0f, true));
            Add(TEXT("magenta: the bake has no drainage (a CPU bake)"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_LAKES:
            Add(TEXT("lake, shallow"), VoxelLandViewLake(0.0f, true, false));
            Add(TEXT("lake, 50 m"), VoxelLandViewLake(50.0f, true, false));
            Add(TEXT("lake, 100 m and deeper"), VoxelLandViewLake(100.0f, true, false));
            Add(TEXT("dry land"), VoxelLandViewLake(0.0f, false, false));
            Add(TEXT("sea"), VoxelLandViewLake(0.0f, false, true));
            Add(TEXT("magenta: the bake has no drainage (a CPU bake)"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_WATER:
        case VOXEL_LAND_VIEW_TERRAIN_WATER:
        {
            const bool bTerrain = View == (int32)VOXEL_LAND_VIEW_TERRAIN_WATER;
            const float3 Land = bTerrain ? VoxelLandViewHeight(300.0f, 1.0e9f) : VoxelLandViewDry(false);
            Add(TEXT("lake, shallow"), VoxelLandViewWater(Land, 0.0f, 0.0f, true, false));
            Add(TEXT("lake, 100 m and deeper"), VoxelLandViewWater(Land, 0.0f, 100.0f, true, false));
            Add(TEXT("stream, 10 km^2"), VoxelLandViewWater(Land, 1.0f, 0.0f, false, false));
            Add(TEXT("river, 1,000 km^2"), VoxelLandViewWater(Land, 3.0f, 0.0f, false, false));
            Add(TEXT("river, 100,000 km^2"), VoxelLandViewWater(Land, 5.0f, 0.0f, false, false));
            Add(bTerrain ? TEXT("land (the elevation's colours; 300 m here)") : TEXT("land"), Land);
            Add(TEXT("sea"), bTerrain ? VoxelLandViewHeight(-800.0f, 1.0e9f) : VoxelLandViewDry(true));
            Add(TEXT("magenta: the bake has no drainage (a CPU bake)"), VoxelLandViewNone());
            break;
        }
        case VOXEL_LAND_VIEW_LANDFORM:
            Add(TEXT("flat plains"), VoxelLandViewLandform(0.0f, 0.0f, false));
            Add(TEXT("rolling ground"), VoxelLandViewLandform(0.4f, 0.0f, false));
            Add(TEXT("hill country, smooth"), VoxelLandViewLandform(1.0f, 0.0f, false));
            Add(TEXT("hill country, rough-topped"), VoxelLandViewLandform(1.0f, 1.0f, false));
            Add(TEXT("sea"), VoxelLandViewLandform(0.0f, 0.0f, true));
            Add(TEXT("magenta: the bake has no landform"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_HILLS:
            Add(TEXT("none (flat plains, river corridors, coasts)"), VoxelLandViewHills(0.0f));
            Add(TEXT("half HillRelief"), VoxelLandViewHills(0.5f));
            Add(TEXT("HillRelief"), VoxelLandViewHills(1.0f));
            Add(TEXT("1.5 x HillRelief"), VoxelLandViewHills(1.5f));
            Add(TEXT("magenta: no hills (HillRelief 0, or no landform)"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_PEAKS:
            Add(TEXT("-1 (down by the peaks' local relief)"), VoxelLandViewSigned(-1.0f));
            Add(TEXT("-0.5"), VoxelLandViewSigned(-0.5f));
            Add(TEXT("0 (the set's mean; none on plains, in the bake's valleys, at sea)"), VoxelLandViewSigned(0.0f));
            Add(TEXT("+0.5"), VoxelLandViewSigned(0.5f));
            Add(TEXT("+1 (up by the peaks' local relief)"), VoxelLandViewSigned(1.0f));
            Add(TEXT("magenta: no peaks (PeakHeightScale 0, or no landform)"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_PEAK_AREA:
            Add(TEXT("none (plains, valleys, coasts)"), VoxelLandViewPeakArea(0.0f));
            Add(TEXT("250 m"), VoxelLandViewPeakArea(250.0f));
            Add(TEXT("500 m"), VoxelLandViewPeakArea(500.0f));
            Add(TEXT("1 km"), VoxelLandViewPeakArea(1000.0f));
            Add(TEXT("2 km"), VoxelLandViewPeakArea(2000.0f));
            Add(TEXT("3 km"), VoxelLandViewPeakArea(3000.0f));
            Add(TEXT("magenta: no peaks (PeakHeightScale 0, or no landform)"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_STRATA:
            Add(TEXT("crystalline (province 0)"), VoxelLandViewStrata(0.0f, 0.0f, false));
            Add(TEXT("province 0.5"), VoxelLandViewStrata(0.5f, 0.0f, false));
            Add(TEXT("sedimentary (province 1), hills"), VoxelLandViewStrata(1.0f, 0.0f, false));
            Add(TEXT("strata half in (the switch)"), VoxelLandViewStrata(0.5f, 0.5f, false));
            Add(TEXT("strata (StrataCoverage of the land)"), VoxelLandViewStrata(0.7f, 1.0f, false));
            Add(TEXT("sea"), VoxelLandViewStrata(0.0f, 0.0f, true));
            Add(TEXT("magenta: the bake has no landform"), VoxelLandViewNone());
            break;
        case VOXEL_LAND_VIEW_RIVERS:
        {
            const float3 Ground = VoxelLandViewHeight(300.0f, 1.0e9f);
            Add(TEXT("a river of 200 km^2 (the smallest at the default RiverMinArea)"), VoxelLandViewRiver(Ground, 1.0f, 1.0f, 27.6f));
            Add(TEXT("10,000 km^2"), VoxelLandViewRiver(Ground, 1.0f, 1.0f, 33.2f));
            Add(TEXT("100,000 km^2 and more"), VoxelLandViewRiver(Ground, 1.0f, 1.0f, 36.6f));
            Add(TEXT("its valley (ValleyWidthScale)"), VoxelLandViewRiver(Ground, 0.0f, 1.0f, 27.6f));
            Add(TEXT("a kept lake, shallow to 100 m (the lakes view's colours)"), VoxelLandViewLake(20.0f, true, false));
            Add(TEXT("land (the elevation's colours)"), Ground);
            Add(TEXT("magenta: no rivers (a CPU bake, or RiverMinArea 0)"), VoxelLandViewNone());
            break;
        }
        default:
            break;
        }
        return Lines;
    }
}

void FVoxelBrickTerrain::TickDebugViews()
{
    // Only the vertex colours need the terrain meshed again; the material's views are a
    // parameter.
    const int32 View = GetLandView();
    const int32 VertexView = GetVertexLandView(Settings.bMaterialLandViews);
    const int32 Level = CVarVoxelDebugLevelColor.GetValueOnGameThread() > 0 ? 1 : 0;
    if (VertexView != LastLandView || Level != LastLevelColor)
    {
        LastLandView = VertexView;
        LastLevelColor = Level;
        RemeshAll();
    }
    FeedRemesh();
    if (View == 0 || !GEngine || !GPUDensity.IsValid()) return;
    // One key per line, shared by every terrain (they show the same legend).
    const bool bTerrestrial = GPUDensity->LandMode != 0;
    const TArray<VBT::FLegendLine> Lines = VBT::LandViewLegend(View, bTerrestrial, Settings.bMaterialLandViews && GetMaterialLandView() != 0);
    constexpr uint64 KeyBase = 0x564f584c56000000ull;   // "VOXLV"
    for (int32 i = 0; i < Lines.Num(); i++)
        GEngine->AddOnScreenDebugMessage(KeyBase + (uint64)i, 0.25f, Lines[i].Color, Lines[i].Text, false);
}

void FVoxelBrickTerrain::DecodeSignature(uint64 Signature, int8 (&OutLevels)[27], uint32& OutMeshed)
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

const FVoxelBrickMesherGPUSettings& FVoxelBrickTerrain::GetMesherSettings() const
{
    return Pages->GetMesherSettings();
}

void FVoxelBrickTerrain::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, TSharedPtr<FDensitySampleCompositor> InDensity,
    const FVoxelBrickTerrainSettings& InSettings)
{
    Reset();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Density = InDensity;
    Settings = InSettings;
    Settings.CellsPerBrick = FMath::Clamp(Settings.CellsPerBrick / 8 * 8, 8, 64);   // as VoxelBrickMesherGPU::CellsFor
    Settings.MaxLevel = FMath::Max(0, Settings.MaxLevel);
    Settings.SlopeBound = FMath::Max(Settings.SlopeBound, 1.0);
    Pages->Initialize(InOwner, InAttachRoot, Settings);
    // The first fill is meshed with the vertex view of the new settings (Reset ran with the old).
    LastLandView = GetVertexLandView(Settings.bMaterialLandViews);
    GPUDensity.Reset();
    GPUEdits.Reset();
    GPUEditsSequence = 0;
    EditBuild.Reset();
    bWarnedUnavailable = false;
    bReportedMorph = false;
    bAvailable = CheckAvailable();

    // The GPU mesher addresses the lattice in 32 bits: a lattice deeper than about 27 levels
    // (at 8 cells per brick) can't mesh its coarsest levels, and those bricks draw nothing.
    const int32 Coarsest = VoxelBrickMesherGPU::CoarsestMeshableLevel(Pages->GetMesherSettings());
    if (Coarsest < Settings.MaxLevel)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelBricks] %s: lattice depth %d exceeds the GPU mesher's 32-bit range for brick levels %d to %d (of %d): bricks of those levels are not drawn. Lower MaxDepth (a larger TargetPrecision) or use larger bricks."),
            InOwner ? *InOwner->GetName() : TEXT("terrain"), Pages->GetMesherSettings().MaxDepth, Coarsest + 1, Settings.MaxLevel, Settings.MaxLevel + 1);
    }
}

void FVoxelBrickTerrain::Reset()
{
    Bricks.Reset();
    Pages->Reset();
    Wanted.Reset();
    RecentEdits.Reset();
    Generation++;
    LastLaunchTime = 0.0;
    bLastComplete = false;
    CameraVelocity = FVector::ZeroVector;
    PredictionVelocity = FVector::ZeroVector;
    LastTickTime = 0.0;
    bCommitPending = false;
    bCommitProgressive = false;
    PendingChanges.Reset();
    NumInPage = 0;
    CommitWaitSeconds = 0.0;
    HeldResult.Reset();
    CommitLaunchTime = 0.0;
    CommitEditSequence = 0;
    MeasuredLatency = 0.0;
    bLatencyMeasured = false;
    ScreenPeriod = 0.0;
    LastScreenCommitTime = 0.0;
    EditBacklog.Reset();
    TargetBacklog.Reset();
    EditBacklogSequence = 0;
    // The terrain is meshed from scratch with the current debug colours.
    RemeshQueue.Reset();
    RemeshHead = 0;
    LastLandView = GetVertexLandView(Settings.bMaterialLandViews);
    LastLevelColor = CVarVoxelDebugLevelColor.GetValueOnGameThread() > 0 ? 1 : 0;
    FScopeLock Lock(&ResultCS);
    Results.Reset();
}

FBox FVoxelBrickTerrain::BoundsOf(const FVoxelBrickKey& Key) const
{
    // A stitched brick also solves cells of coarser neighbours, whose gradients reach four of
    // its own cells beyond it.
    const FVoxelBrickGrid Grid = Settings.GridFor(Key.Level);
    const FVector Min = Grid.BrickMin(Key.Coord);
    return FBox(Min - FVector(4.0 * Grid.CellSize), Min + FVector(Grid.BrickSize() + 4.0 * Grid.CellSize));
}

FBox FVoxelBrickTerrain::TargetBoundsOf(const FVoxelBrickKey& Key) const
{
    const FVoxelBrickGrid Grid = Settings.GridFor(Key.Level);
    const FVector Min = Grid.BrickMin(Key.Coord);
    const double Reach = VoxelBrickMesherGPU::TargetReachCells * Grid.CellSize;
    return FBox(Min - FVector(Reach), Min + FVector(Grid.BrickSize() + Reach));
}

void FVoxelBrickTerrain::NotifyEdit(uint32 Sequence, const FBox& Changed)
{
    // Where an edit can change what a brick of a level reads: its shape's reach (Changed), and
    // around it, deep inside solid or air, the values of the edit store's index cells that list
    // it (a carve raises the solid near it; FSparseEditStore::Apply). A level's surface crossings
    // read values up to about the slope bound times its cell from the surface, so the margin
    // grows with the level, up to the index cell's diagonal.
    const TSharedPtr<FSparseEditStore> Store = Density.IsValid() ? Density->GetEditStore() : nullptr;
    const double IndexDiagonal = Store.IsValid() ? Store->GetIndexCellSize() * FMath::Sqrt(3.0) : 0.0;
    auto BoundsAt = [this, &Changed, IndexDiagonal](int32 Level)
        {
            const double Margin = FMath::Min(Settings.SlopeBound * Settings.GridFor(Level).CellSize, IndexDiagonal);
            return Changed.ExpandBy(2.0 * Settings.FinestCell + Margin);
        };
    RecentEdits.Add({ Sequence, Changed.ExpandBy(2.0 * Settings.FinestCell + IndexDiagonal) });
    // An empty test from before the edit proves nothing (an edit can add surface).
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        if (!BoundsOf(Pair.Key).Intersect(BoundsAt(Pair.Key.Level))) continue;
        Pair.Value.bDirty = true;
        Pair.Value.bProvenEmpty = false;
    }
    // The bricks of the tree whose mesh the edit reaches are meshed again as they are, without
    // waiting for a task (one that adds or removes bricks still follows). Bricks only whose
    // morph targets it reaches (many more: the targets read the density up to TargetReachCells
    // away) go with the next result's commit, which follows the edit anyway.
    for (const TPair<FVoxelBrickKey, uint64>& Pair : Wanted)
    {
        const FBox Bounds = BoundsAt(Pair.Key.Level);
        if (BoundsOf(Pair.Key).Intersect(Bounds))
            EditBacklog.Add(Pair.Key);
        else if (TargetBoundsOf(Pair.Key).Intersect(Bounds))
            TargetBacklog.Add(Pair.Key);
    }
    EditBacklogSequence = FMath::Max(EditBacklogSequence, Sequence);
}

void FVoxelBrickTerrain::SetMaterial(UMaterialInterface* InMaterial)
{
    Settings.Material = InMaterial;
    Pages->SetMaterial(InMaterial);
}

bool FVoxelBrickTerrain::CheckAvailable()
{
    // The GPU lattice is centred on the root: Origin must be -HalfFinest * 2^MaxDepth.
    const double RootExtent = 0.5 * Settings.FinestCell * FMath::Pow(2.0, (double)Pages->GetMesherSettings().MaxDepth);
    const TCHAR* Reason = nullptr;
    if (!FApp::CanEverRender())
        Reason = TEXT("no renderer");
    else if (!Density.IsValid() || !Density->GetBaseDensity().IsValid())
        Reason = TEXT("the density has no GPU version");
    else if (!Settings.Origin.Equals(FVector(-RootExtent), 1e-6 * RootExtent))
        Reason = TEXT("the brick lattice is not centred on the root");
    if (!Reason) return true;

    if (!bWarnedUnavailable)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelBricks] %s: the brick terrain can't run (%s); nothing is drawn."),
            Owner.IsValid() ? *Owner->GetName() : TEXT("terrain"), Reason);
    }
    bWarnedUnavailable = true;
    return false;
}

void FVoxelBrickTerrain::FlushPages()
{
    if (!GPUDensity.IsValid())
    {
        const TSharedRef<FVoxelDensityGPUData> Data = MakeShared<FVoxelDensityGPUData>();
        Density->GetBaseDensity()->BuildGPUData(*Data);
        GPUDensity = Data;
    }
    // The edit tree flattened for the GPU: the first time here, then on a worker, one build at
    // a time (it grows with the edits). Edit commits wait for the build holding their edit.
    const TSharedPtr<FSparseEditStore> Store = Density->GetEditStore();
    const uint32 Sequence = Store.IsValid() ? Store->GetLastSequence() : 0;
    const double HalfFinest = 0.5 * Settings.FinestCell;
    const int32 MaxDepth = Pages->GetMesherSettings().MaxDepth;
    if (EditBuild.IsValid() && EditBuild->bDone.load(std::memory_order_acquire))
    {
        GPUEdits = EditBuild->Data;
        GPUEditsSequence = EditBuild->Sequence;
        EditBuild.Reset();
    }
    if (!GPUEdits.IsValid())
    {
        GPUEdits = FVoxelEditGPUData::Build(Store.Get(), HalfFinest, MaxDepth);
        GPUEditsSequence = Sequence;
    }
    else if (Sequence != GPUEditsSequence && !EditBuild.IsValid())
    {
        const TSharedRef<FEditBuild, ESPMode::ThreadSafe> Build = MakeShared<FEditBuild, ESPMode::ThreadSafe>();
        Build->Sequence = Sequence;
        EditBuild = Build;
        FFunctionGraphTask::CreateAndDispatchWhenReady([Build, Store, HalfFinest, MaxDepth]()
            {
                Build->Data = FVoxelEditGPUData::Build(Store.Get(), HalfFinest, MaxDepth);
                Build->bDone.store(true, std::memory_order_release);
            }, TStatId(), nullptr, ENamedThreads::AnyBackgroundThreadNormalTask);
    }
    Pages->Flush(GPUDensity, GPUEdits);
}

double FVoxelBrickTerrain::MorphSlack(double ScreenTime, double Prediction) const
{
    // A tree meets the next level at distances from the camera it was built for; moving the
    // bands nearer by how far the camera can be from it keeps each level fully blended where
    // it meets the next. In steady flight a tree built for Prediction of the time to the
    // middle of its time on screen is behind the camera by up to (1 - Prediction) Latency +
    // (1 - Prediction / 2) ScreenTime of flight (ahead, above 1, by the opposite at its
    // start); a quarter of the latency more covers changes of speed and latency. From the
    // filtered speed, so the slack changes smoothly: a jump would move every blending vertex
    // at once.
    const double Behind = (1.0 - Prediction) * TreeLatency + (1.0 - 0.5 * Prediction) * ScreenTime;
    const double Ahead = (Prediction - 1.0) * TreeLatency + 0.5 * Prediction * ScreenTime;
    return CameraVelocity.Size() * (FMath::Max3(Behind, Ahead, 0.0) + 0.25 * TreeLatency);
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
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelBricks] No Brick Morph Parameters collection on %s: the GPU terrain's LOD changes won't blend (Design/MeshingEngine.md section 7)."),
                OwnerActor ? *OwnerActor->GetName() : TEXT("the terrain"));
        }
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
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelBricks] Morph parameters go to %s (mode %d)."), *Collection->GetPathName(), Mode);
        }
        else
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelBricks] %s lacks a vector parameter VoxelMorphCameraHigh, VoxelMorphCameraLow or VoxelMorphParams (Design/MeshingEngine.md section 7)."),
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
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelBrickPerf] %d frames in %.2f s. Game thread ms per frame (average / max): take results %.2f / %.2f, commit %.2f / %.2f, flush pages %.2f / %.2f, launch %.2f / %.2f. Tasks: %d, %.1f ms average (tree %.1f), %.1f ms max, %lld samples, %d bricks tested (%d new, %d restitched, %d after edits). %d held, %d wanted; commits wait %.0f ms (filtered); launch to screen %.0f ms average, %.0f ms max over %d results, %d of them held for a commit; %d edit commits. Camera %.0f m/s; trees built %.1f m ahead of it (average, %.1f max over %d tasks); morph slack %.1f m average, %.1f max."),
            Perf.Frames, Now - Perf.Start,
            Perf.SectionMs[0] * PerFrame, Perf.MaxSectionMs[0], Perf.SectionMs[1] * PerFrame, Perf.MaxSectionMs[1], Perf.SectionMs[2] * PerFrame, Perf.MaxSectionMs[2],
            Perf.SectionMs[3] * PerFrame, Perf.MaxSectionMs[3],
            Perf.Tasks, Perf.TaskMs * PerTask, Perf.TreeMs * PerTask, Perf.MaxTaskMs, Perf.TaskSamples, Perf.TaskTested, Perf.TaskNew, Perf.TaskRestitched, Perf.TaskRebuilt,
            Bricks.Num(), Wanted.Num(),
            CommitWaitSeconds * 1000.0, Perf.Latencies > 0 ? Perf.LatencyMs / Perf.Latencies : 0.0, Perf.MaxLatencyMs, Perf.Latencies, Perf.HeldResults, Perf.EditCommits,
            CameraVelocity.Size() / 100.0, Perf.Launches > 0 ? Perf.Lead / Perf.Launches / 100.0 : 0.0, Perf.MaxLead / 100.0, Perf.Launches,
            Perf.Slack * PerFrame / 100.0, Perf.MaxSlack / 100.0);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelBrickPerf] %s"), *Pages->TakePerf());
    }
    else
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
        const FVector Instant = (Camera - LastCamera) / Step;
        const double Blend = 1.0 - FMath::Exp(-Step / VBT::VelocitySmoothing);
        CameraVelocity += (Instant - CameraVelocity) * Blend;
        FrameSeconds += (Step - FrameSeconds) * Blend;
        // The prediction lets go of speed faster than it picks it up, so the trees launched
        // after a sudden stop aren't built ahead of the camera.
        const double Smoothing = Instant.SizeSquared() < PredictionVelocity.SizeSquared() ? VBT::VelocityRelease : VBT::VelocitySmoothing;
        PredictionVelocity += (Instant - PredictionVelocity) * (1.0 - FMath::Exp(-Step / Smoothing));
    }
    else
    {
        CameraVelocity = FVector::ZeroVector;
        PredictionVelocity = FVector::ZeroVector;
    }
    LastTickTime = Now;
    LastCamera = Camera;
    if (!bAvailable) return;
    TickDebugViews();

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
        TakeHeldResult();
    }
    Lap(0);
    // A tree is on screen until the next is: a task interval, or the measured time between
    // commits.
    const double ScreenTime = ScreenPeriod > 0.0 ? ScreenPeriod : VBT::TaskInterval;
    const double Prediction = FMath::Max(0.0, (double)CVarVoxelBrickPrediction.GetValueOnGameThread());
    // From a task's launch until its tree is drawn: measured once a result is committed (the
    // task, any wait for the commit before it, its own commit), estimated before.
    TreeLatency = bLatencyMeasured ? MeasuredLatency : TaskSeconds + CommitWaitSeconds + FrameSeconds;
    StartEditCommit();
    // While the next tree is on its way, the commit ahead of it is meshed faster.
    Pages->SetBusy(bCommitPending && (HeldResult.IsSet() || bTaskRunning.load(std::memory_order_acquire)));
    {
        SCOPE_CYCLE_COUNTER(STAT_VoxelBricksFlush);
        FlushPages();
    }
    Lap(2);
    {
        SCOPE_CYCLE_COUNTER(STAT_VoxelBricksApply);
        CommitPages();
    }
    const double Slack = MorphSlack(ScreenTime, Prediction);
    UpdateMorphParameters(Slack);
    Perf.Slack += Slack;
    Perf.MaxSlack = FMath::Max(Perf.MaxSlack, Slack);
    Lap(1);
    SET_DWORD_STAT(STAT_VoxelBricksHeld, Bricks.Num());
    SET_DWORD_STAT(STAT_VoxelBricksWanted, Wanted.Num());
    LogPerf(Now);

    // The next task runs while a commit is meshed, and its result waits for it.
    if (bTaskRunning.load(std::memory_order_acquire) || HeldResult.IsSet() || Now - LastLaunchTime < VBT::TaskInterval || !Density.IsValid()) return;
    SCOPE_CYCLE_COUNTER(STAT_VoxelBricksLaunch);
    ON_SCOPE_EXIT { Lap(3); };

    // The tree is on screen from about TreeLatency after the launch for ScreenTime: build it
    // for the camera predicted halfway through, scaled by voxel.Render.BrickPrediction (a
    // sudden stop then overshoots less).
    const FVector TaskCamera = Camera + PredictionVelocity * (Prediction * (TreeLatency + 0.5 * ScreenTime));

    // A task that finished since the drain above must be applied before the next starts.
    DrainResults();
    if (HeldResult.IsSet()) return;
    LastLaunchTime = Now;

    // Nothing to do: the latest result is complete for (about) this camera and nothing changed.
    if (bLastComplete && RecentEdits.Num() == 0
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
    Input.PrevWanted = Wanted;
    Input.LaunchTime = Now;
    // The next result is a difference from the tree being committed (Wanted), whose bricks are
    // tested; dirty ones are tested again.
    for (TPair<FVoxelBrickKey, FBrick>& Pair : Bricks)
    {
        // Dirty stays set until a new test arrives (ApplyResult), so a rebuild of a brick that
        // is not wanted this pass is not lost.
        if (Pair.Value.bDirty && Wanted.Contains(Pair.Key))
            Input.Rebuild.Add(Pair.Key);
        if (Pair.Value.bProvenEmpty)
            Input.ProvenEmpty.Add(Pair.Key);
    }

    const double Lead = FVector::Dist(TaskCamera, Camera);
    Perf.Lead += Lead;
    Perf.MaxLead = FMath::Max(Perf.MaxLead, Lead);
    Perf.Launches++;

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
    Result.LaunchTime = Input.LaunchTime;

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
            Ancestor = Ancestor.Parent();
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
        // From BalancedFactor on the wanted leaves are balanced as split (culled leaves needn't
        // be, see NeighbourDelta).
        if (Input.Settings.LodDistanceFactor < VBT::BalancedFactor)
            Tree.Balance();
    }
    // The bricks the task knows are the previous tree's.
    const TMap<FVoxelBrickKey, uint64>& Known = Input.PrevWanted;
    Result.BalanceSplits = Tree.BalanceSplits;
    Result.TreeMilliseconds = (FPlatformTime::Seconds() - T0) * 1000.0;

    // Each wanted brick's neighbours, packed into its signature: per direction 0 finer, 1 same
    // level and meshed, 2 same level and not meshed, 3 coarser; bit 63 set.
    struct FEntry
    {
        FVoxelBrickKey Key;
        bool bEdited;
        uint64 Signature;
    };
    TSet<FVoxelBrickKey> RebuildSet;
    RebuildSet.Append(Input.Rebuild);
    TArray<FEntry> Entries;
    Entries.SetNum(Tree.WantedInfo.Num());
    {
        int32 i = 0;
        for (const TPair<FVoxelBrickKey, VBT::FWantedInfo>& Pair : Tree.WantedInfo)
        {
            Entries[i].Key = Pair.Key;
            Entries[i].bEdited = Pair.Value.bEdited;
            i++;
        }
    }
    // Signatures in parallel (the tree is only read), then each brick's kind: 0 held as it
    // is, 1 new, 2 reached by an edit, 3 restitched.
    TArray<uint8> Kinds;
    Kinds.SetNumZeroed(Entries.Num());
    ParallelFor(Entries.Num(), [&](int32 i)
        {
            FEntry& Entry = Entries[i];
            Entry.Signature = 1ULL << 63;
            for (int32 d = 0, Slot = 0; d < 27; d++)
            {
                if (d == 13) continue;
                bool bMeshed = false;
                const int32 Delta = Tree.NeighbourDelta(Entry.Key, d, bMeshed);
                const uint64 State = Delta < 0 ? 0 : (Delta > 0 ? 3 : (bMeshed ? 1 : 2));
                Entry.Signature |= State << (2 * Slot++);
            }
            const uint64* Held = Known.Find(Entry.Key);
            Kinds[i] = (uint8)(!Held ? 1 : (RebuildSet.Contains(Entry.Key) ? 2 : (*Held != Entry.Signature ? 3 : 0)));
        });

    // The empty test, for the new bricks and those an edit reached, in one parallel batch; the
    // pages mesh them with the levels in their signatures. The test reads neither the neighbour
    // levels nor the meshed mask, so a restitched brick keeps its result (an edit makes it a
    // rebuild).
    TArray<int32> ToTest;
    Result.Wanted.Reserve(Entries.Num());
    for (int32 i = 0; i < Entries.Num(); i++)
    {
        Result.Wanted.Add(Entries[i].Key, Entries[i].Signature);
        if (Kinds[i] == 0) continue;
        if (Kinds[i] == 3)
        {
            FTested& Tested = Result.Tested.AddDefaulted_GetRef();
            Tested.Key = Entries[i].Key;
            Tested.Signature = Entries[i].Signature;
            Tested.bProvenEmpty = Input.ProvenEmpty.Contains(Entries[i].Key);
            Result.NumRestitched++;
            continue;
        }
        ToTest.Add(i);
        Result.NumNew += Kinds[i] == 1 ? 1 : 0;
        Result.NumRebuilt += Kinds[i] == 2 ? 1 : 0;
    }
    std::atomic<int64> Samples{ 0 };
    TArray<FTested> Batch;
    Batch.SetNum(ToTest.Num());
    ParallelFor(ToTest.Num(), [&](int32 b)
        {
            const FEntry& Entry = Entries[ToTest[b]];
            FTested& Tested = Batch[b];
            Tested.Key = Entry.Key;
            Tested.Signature = Entry.Signature;
            int32 TestSamples = 0;
            Tested.bProvenEmpty = !Entry.bEdited
                && VoxelBrickMesher::IsBrickProvablyEmpty(*Input.Density, Input.Settings.GridFor(Entry.Key.Level), Entry.Key.Coord, Input.Settings.SlopeBound, TestSamples);
            Samples.fetch_add(TestSamples, std::memory_order_relaxed);
        });
    Result.Tested.Append(MoveTemp(Batch));
    Result.Camera = Input.Camera;

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
        // A result is a difference from the tree being committed, so it waits for that commit
        // (CommitPages applies it).
        if (bCommitPending || HeldResult.IsSet())
        {
            HeldResult = MoveTemp(Result);
            Perf.HeldResults++;
            continue;
        }
        ApplyResult(Result);
    }
}

void FVoxelBrickTerrain::ApplyResult(FTaskResult& Result)
{
    Wanted = MoveTemp(Result.Wanted);
    bLastComplete = true;   // a result tests every brick it wants
    LastTaskCamera = Result.Camera;
    TaskSeconds += (Result.Milliseconds / 1000.0 - TaskSeconds) * 0.5;
    Perf.Tasks++;
    Perf.TaskMs += Result.Milliseconds;
    Perf.MaxTaskMs = FMath::Max(Perf.MaxTaskMs, Result.Milliseconds);
    Perf.TreeMs += Result.TreeMilliseconds;
    Perf.TaskSamples += Result.Samples;
    Perf.TaskTested += Result.Tested.Num();
    Perf.TaskNew += Result.NumNew;
    Perf.TaskRestitched += Result.NumRestitched;
    Perf.TaskRebuilt += Result.NumRebuilt;

    int32 NumWithSurface = 0;
    for (const FTested& Tested : Result.Tested)
    {
        FBrick& Brick = Bricks.FindOrAdd(Tested.Key);
        // An edit published after the task read the edit store isn't in this test.
        Brick.bDirty = false;
        const FBox Bounds = BoundsOf(Tested.Key);
        for (const FRecentEdit& Edit : RecentEdits)
        {
            if (Edit.Sequence > Result.StartSequence && Edit.Bounds.Intersect(Bounds))
                Brick.bDirty = true;
        }
        // Committed with the whole result (CommitPages). A test older than an edit in reach
        // proves nothing. A brick whose page mesh already holds every edit the task saw (an
        // edit commit), with the same neighbours, isn't meshed again.
        Brick.bProvenEmpty = Tested.bProvenEmpty && !Brick.bDirty;
        Brick.bRemesh = !(Brick.bInPage && Brick.Signature == Tested.Signature && Brick.EditSequence >= Result.StartSequence);
        NumWithSurface += Brick.bProvenEmpty ? 0 : 1;
    }
    RecentEdits.RemoveAll([&Result](const FRecentEdit& Edit) { return Edit.Sequence <= Result.StartSequence; });

    // The whole difference in one frame; page by page only while nothing is on screen (the
    // first fill), where a partly filled tree hides nothing.
    bCommitProgressive = NumInPage == 0;
    bCommitPending = true;
    CommitPendingFrame = GFrameCounter;
    CommitLaunchTime = Result.LaunchTime;
    PendingChanges.Reset();
    BuildCommit(PendingChanges);
    if (!bCommitProgressive)
        MergeEdits(PendingChanges, true);
    StagePending();

    if (CVarVoxelDebugBricks.GetValueOnGameThread() > 0)
    {
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelBricks] %d wanted (%d balance splits), %d tested (%d with surface, %d new, %d after edits, %d restitched without a test), %lld samples, %.1f ms, %d bricks held; camera %.0f m/s, tree latency %.0f ms"),
            Wanted.Num(), Result.BalanceSplits, Result.Tested.Num() - Result.NumRestitched, NumWithSurface, Result.NumNew, Result.NumRebuilt, Result.NumRestitched,
            Result.Samples, Result.Milliseconds, Bricks.Num(), CameraVelocity.Size() / 100.0, TreeLatency * 1000.0);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelBricks] %d page members committed; commits wait %.0f ms (filtered). %s"),
            NumInPage, CommitWaitSeconds * 1000.0, *Pages->Describe());
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
            DecodeSignature(Pair.Value, Change.Levels, Change.Meshed);
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
        Brick.bRemesh = false;
        Brick.Signature = Wanted.FindRef(Change.Key);
        Brick.EditSequence = CommitEditSequence;
    }
}

void FVoxelBrickTerrain::MergeEdits(TArray<FVoxelBrickPageChange>& Changes, bool bWithTargets)
{
    // Meshed with GPU edit data that misses an edit, they would only be meshed again later.
    if (GPUEditsSequence < EditBacklogSequence) return;
    if (EditBacklog.Num() == 0 && (!bWithTargets || TargetBacklog.Num() == 0)) return;
    TSet<FVoxelBrickKey> Changed;
    Changed.Reserve(Changes.Num());
    for (const FVoxelBrickPageChange& Change : Changes)
        Changed.Add(Change.Key);
    auto Merge = [this, &Changes, &Changed](TSet<FVoxelBrickKey>& Backlog)
        {
            for (const FVoxelBrickKey& Key : Backlog)
            {
                // A brick leaving, or changed by this commit anyway (meshed with the same edits).
                const uint64* Signature = Wanted.Find(Key);
                if (!Signature || Changed.Contains(Key)) continue;
                Changed.Add(Key);
                FVoxelBrickPageChange& Change = Changes.AddDefaulted_GetRef();
                Change.Key = Key;
                DecodeSignature(*Signature, Change.Levels, Change.Meshed);
            }
            Backlog.Reset();
        };
    Merge(EditBacklog);
    if (bWithTargets)
        Merge(TargetBacklog);
}

void FVoxelBrickTerrain::StartEditCommit()
{
    // Edits that come with no commit pending go to the pages on their own (not while the
    // first fill is on its way).
    if (bCommitPending || EditBacklog.Num() == 0 || NumInPage == 0 || GPUEditsSequence < EditBacklogSequence) return;
    PendingChanges.Reset();
    MergeEdits(PendingChanges, false);
    if (PendingChanges.Num() == 0) return;
    bCommitPending = true;
    bCommitProgressive = false;
    CommitPendingFrame = GFrameCounter;
    CommitLaunchTime = 0.0;
    StagePending();
    Perf.EditCommits++;
}

void FVoxelBrickTerrain::StagePending()
{
    // Every brick is meshed with this edit data or newer, so the mesh holds these edits.
    CommitEditSequence = GPUEditsSequence;
    Pages->Stage(PendingChanges, bCommitProgressive);
}

void FVoxelBrickTerrain::CommitPages()
{
    if (!bCommitPending) return;
    if (!bCommitProgressive)
    {
        if (PendingChanges.Num() > 0)
        {
            if (!Pages->PrepareCommit(PendingChanges, true)) return;
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
        if (Ready.Num() > 0 && Pages->PrepareCommit(Ready, false))
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

    bCommitPending = false;
    PendingChanges.Reset();
    // A result's commit (an edit commit leaves the tree as it is): the tree on screen is
    // Wanted, and the result's launch-to-screen time is measured.
    if (CommitLaunchTime > 0.0)
    {
        for (auto It = Bricks.CreateIterator(); It; ++It)
        {
            const uint64* Signature = Wanted.Find(It.Key());
            if (!Signature)
            {
                It.RemoveCurrent();
                continue;
            }
            It.Value().Signature = *Signature;
            It.Value().bRemesh = false;
        }
        const double Wait = (double)(GFrameCounter - CommitPendingFrame) * FrameSeconds;
        CommitWaitSeconds += (Wait - CommitWaitSeconds) * VBT::CommitWaitBlend;
        const double Now = FPlatformTime::Seconds();
        const double Latency = Now - CommitLaunchTime;
        MeasuredLatency = bLatencyMeasured ? MeasuredLatency + (Latency - MeasuredLatency) * VBT::LatencyBlend : Latency;
        bLatencyMeasured = true;
        Perf.LatencyMs += Latency * 1000.0;
        Perf.MaxLatencyMs = FMath::Max(Perf.MaxLatencyMs, Latency * 1000.0);
        Perf.Latencies++;
        CommitLaunchTime = 0.0;
        // That result's tree is on screen until the next result's commit.
        if (LastScreenCommitTime > 0.0)
        {
            const double Period = FMath::Clamp(Now - LastScreenCommitTime, VBT::TaskInterval, FMath::Max(MeasuredLatency, VBT::TaskInterval));
            ScreenPeriod = ScreenPeriod > 0.0 ? ScreenPeriod + (Period - ScreenPeriod) * VBT::LatencyBlend : Period;
        }
        LastScreenCommitTime = Now;
    }
}

void FVoxelBrickTerrain::TakeHeldResult()
{
    // A result that finished during the previous commit, staged the frame after it, so no
    // frame both commits a tree and stages the next. (The mesher takes its first batches in
    // the next frame either way.)
    if (bCommitPending || !HeldResult.IsSet()) return;
    FTaskResult Held = MoveTemp(HeldResult.GetValue());
    HeldResult.Reset();
    ApplyResult(Held);
}

bool FVoxelBrickTerrain::HoldsSurface(const FVoxelBrickKey& Key) const
{
    const FBrick* Brick = Bricks.Find(Key);
    return Brick && Brick->bInPage;
}
