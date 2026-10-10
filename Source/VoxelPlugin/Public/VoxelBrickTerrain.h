// VoxelBrickTerrain.h - Terrain rendered from bricks (VoxelBrickMesher) in a level-of-detail
// tree around the camera. The tree is built on the CPU; the bricks are meshed on the GPU and
// drawn from pages (VoxelBrickPages).

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "HAL/IConsoleManager.h"
#include "Misc/Optional.h"
#include "VoxelBrickKey.h"
#include "VoxelBrickMesher.h"
#include <atomic>

/** voxel.Debug.LevelColor: vertex colour of the bricks by level (0 off). */
extern TAutoConsoleVariable<int32> CVarVoxelDebugLevelColor;

/** voxel.Debug.LandView: vertex colour of the bricks by a component of the generator (0 off;
 *  VOXEL_LAND_VIEW_* in VoxelLandDebugPalette.ush). Read it through
 *  FVoxelBrickTerrain::GetLandView. */
extern TAutoConsoleVariable<int32> CVarVoxelDebugLandView;

class AActor;
class USceneComponent;
class UMaterialInterface;
class UMaterialParameterCollection;
class FDensitySampleCompositor;
struct FVoxelDensityGPUData;
struct FVoxelEditGPUData;
struct FVoxelBrickMesherGPUSettings;
class FVoxelBrickPages;

/** Inputs of the brick terrain, fixed until the next Initialize. */
struct FVoxelBrickTerrainSettings
{
    FVector Origin = FVector::ZeroVector;   // lattice point 0 (-RootExtent on each axis)
    double FinestCell = 100.0;              // level-0 cell size (ActualPrecision)
    int32 CellsPerBrick = 32;
    int32 MaxLevel = 0;                     // level whose single brick covers the whole root
    double LodDistanceFactor = 2.5;         // a brick splits when the camera is closer than this many brick sizes
    double SlopeBound = 3.5;                // the largest |grad density| assumed by the empty test
    double TriplanarUVScale = 0.0001;
    /** The sea level the elevation debug view measures from (cm from the centre); 0: the
     *  density's own (the terrestrial sea radius, or the noise surface's base radius). */
    double DebugSeaRadius = 0.0;
    TWeakObjectPtr<UMaterialInterface> Material;
    /** Receives the camera and morph slack every frame (VoxelMorph.ush). */
    TWeakObjectPtr<UMaterialParameterCollection> MorphParameters;

    /** The lattice of one level: cells of FinestCell * 2^Level (exact, so stitched bricks
     *  share lattice points across levels). */
    FVoxelBrickGrid GridFor(int32 Level) const
    {
        FVoxelBrickGrid Grid;
        Grid.Origin = Origin;
        Grid.CellSize = FinestCell * (double)(1LL << Level);
        Grid.CellsPerBrick = CellsPerBrick;
        return Grid;
    }
};

/** Brick terrain of one terrain actor.
 *
 *  Every TaskInterval a worker task walks the tree from the root brick, a level at a time in
 *  parallel, for the camera predicted to the time the tree will be on screen: a brick is
 *  culled when its centre sample proves it free of surface, split while the camera is within
 *  LodDistanceFactor brick sizes (1.15 times that if the previous tree split it), and
 *  otherwise wanted. Touching wanted leaves differ by at most one level (from a factor of
 *  about 2.04 by the split rule alone, below it by an explicit balance), and each wanted
 *  brick is meshed stitched to its neighbours' levels (VoxelBrickMesher::MeshBrickStitched on
 *  the GPU); culled neighbours count as its own level, since nothing they hold reaches its
 *  mesh. The task only tests the wanted bricks for emptiness; the pages mesh them.
 *
 *  The bricks a task result changes are meshed ahead of its commit (FVoxelBrickPages::Stage),
 *  then the result is committed to the pages whole, in one frame, once every page it touches
 *  is ready (FVoxelBrickPages::PrepareCommit), so the tree on screen switches from one frame
 *  to the next. Only the first fill, with nothing on screen, commits page by page. The next
 *  task runs while a commit is meshed (which then meshes faster); its result waits for that
 *  commit and is staged the frame after it. Edits go to the pages directly: the held bricks
 *  in an edit's reach are meshed again as the tree has them, in their own commit or with the
 *  next result's.
 *
 *  The terrain needs a renderer, a density with a GPU version and a lattice centred on the
 *  root (the GPU mesher's requirements); otherwise it draws nothing and warns once. */
class VOXELPLUGIN_API FVoxelBrickTerrain : public TSharedFromThis<FVoxelBrickTerrain>
{
public:
    FVoxelBrickTerrain();
    ~FVoxelBrickTerrain();

    /** Game thread. Drops every brick and starts over. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, TSharedPtr<FDensitySampleCompositor> InDensity,
        const FVoxelBrickTerrainSettings& InSettings);

    /** Game thread. Destroys every page; results of a running task are dropped. */
    void Reset();

    /** Game thread. The collection the terrain material's morph reads (Settings.MorphParameters). */
    void SetMorphParameters(UMaterialParameterCollection* InCollection);

    /** Game thread. Changes the material of every page component, without rebuilding. */
    void SetMaterial(UMaterialInterface* InMaterial);

    /** Game thread. See FVoxelCollisionBricks::NotifyEdit. The held bricks whose mesh the
     *  edit reaches are meshed again at once (in their own commit when none is pending);
     *  those only whose morph targets it reaches, with the next task result. */
    void NotifyEdit(uint32 Sequence, const FVector& Center, double Radius);

    /** Game thread, every frame. Camera in actor-local space. */
    void Tick(const FVector& Camera);

    int32 NumBricks() const { return Bricks.Num(); }

    /** Game thread. Bricks of the latest tree and their signatures (neighbour levels, see
     *  RunTask), and the settings they were built with. */
    const TMap<FVoxelBrickKey, uint64>& GetWantedSignatures() const { return Wanted; }
    const FVoxelBrickTerrainSettings& GetSettings() const { return Settings; }
    /** The GPU mesher's view of the lattice (FVoxelBrickPages derives it from Settings). */
    const FVoxelBrickMesherGPUSettings& GetMesherSettings() const;

    /** Game thread. True if the brick may hold surface: a member of a page, as committed. */
    bool HoldsSurface(const FVoxelBrickKey& Key) const;

    /** Game thread. The edit tree as the pages mesh with it (null before the first Tick) and
     *  the newest edit it holds. The ocean samples its depth with the same data. */
    TSharedPtr<const FVoxelEditGPUData> GetGPUEdits() const { return GPUEdits; }
    uint32 GetGPUEditsSequence() const { return GPUEditsSequence; }

    /** Debug vertex colour of a level (voxel.Debug.LevelColor). */
    static FColor LevelColor(int32 Level);

    /** Game thread. voxel.Debug.LandView, 0 when out of range. */
    static int32 GetLandView();

    /** Game thread. The sea radius the elevation debug view measures from (Settings.
     *  DebugSeaRadius); with that view on, the terrain is meshed again. */
    void SetDebugSeaRadius(double InSeaRadius);

    /** Neighbour levels and the meshed mask a signature packs (see RunTask), as
     *  VoxelBrickMesher::MeshBrickStitched takes them. */
    static void DecodeSignature(uint64 Signature, int8 (&OutLevels)[27], uint32& OutMeshed);

private:
    /** A brick the terrain holds: tested by a task, and as committed to the pages. */
    struct FBrick
    {
        /** Neighbour levels the committed mesh was built for. */
        uint64 Signature = 0;
        bool bDirty = false;          // an edit reached it since its last test
        bool bProvenEmpty = false;    // the newest test proves it free of surface
        /** A page member as committed, and built by the pending result (meshed again at the
         *  commit even if its signature is unchanged). */
        bool bInPage = false;
        bool bRemesh = false;
        /** The committed mesh holds every edit up to this sequence. */
        uint32 EditSequence = 0;
    };

    /** One brick a task tested. */
    struct FTested
    {
        FVoxelBrickKey Key;
        uint64 Signature = 0;
        bool bProvenEmpty = false;            // the empty test proved it free of surface
    };

    struct FTaskInput
    {
        uint32 Generation = 0;
        FVector Camera = FVector::ZeroVector;
        TArray<FVoxelBrickKey> Rebuild;       // held bricks an edit reached: tested again
        TSet<FVoxelBrickKey> ProvenEmpty;     // held bricks whose newest test proved them empty
        TMap<FVoxelBrickKey, uint64> PrevWanted;   // the latest tree (merge hysteresis, and what is known)
        TSharedPtr<FDensitySampleCompositor> Density;
        FVoxelBrickTerrainSettings Settings;
        double LaunchTime = 0.0;
    };

    struct FTaskResult
    {
        uint32 Generation = 0;
        uint32 StartSequence = 0;
        TMap<FVoxelBrickKey, uint64> Wanted;   // and their signatures
        TArray<FTested> Tested;
        FVector Camera = FVector::ZeroVector;
        int32 BalanceSplits = 0;
        int32 NumNew = 0;
        int32 NumRebuilt = 0;
        int32 NumRestitched = 0;
        int64 Samples = 0;
        double Milliseconds = 0.0;
        double TreeMilliseconds = 0.0;        // the descent and the balance
        double LaunchTime = 0.0;
    };

    struct FRecentEdit
    {
        uint32 Sequence;
        FBox Bounds;
    };

    static void RunTask(const TSharedRef<FVoxelBrickTerrain>& Self, FTaskInput Input);

    /** The region a brick's mesh depends on: its cells and the four around them its stitched
     *  mesh reads. */
    FBox BoundsOf(const FVoxelBrickKey& Key) const;
    /** The region a brick's morph targets depend on (VoxelBrickMesherGPU::TargetReachCells). */
    FBox TargetBoundsOf(const FVoxelBrickKey& Key) const;

    void DrainResults();
    /** Takes a result's tree and bricks, builds its commit and stages it. */
    void ApplyResult(FTaskResult& Result);
    /** Commits PendingChanges when their pages are ready. */
    void CommitPages();
    /** Applies the result that waited for the previous commit, once it is done. */
    void TakeHeldResult();
    /** The page changes from the committed tree to Wanted. */
    void BuildCommit(TArray<FVoxelBrickPageChange>& Out) const;
    /** Records committed changes on the bricks. */
    void ApplyCommitted(TConstArrayView<FVoxelBrickPageChange> Changes);
    /** Adds changes for the edited bricks (EditBacklog, and TargetBacklog when bWithTargets)
     *  that Changes doesn't hold, once the GPU edit data has every edit they wait for. */
    void MergeEdits(TArray<FVoxelBrickPageChange>& Changes, bool bWithTargets);
    /** Commits the edited bricks on their own when no commit is pending. */
    void StartEditCommit();
    /** Meshes every brick on screen again, as the tree has it, for new debug colours: queued
     *  for FeedRemesh, and a pending commit staged again. */
    void RemeshAll();
    /** Moves the next VBT::RemeshChunkBricks queued bricks into the edit backlog once nothing
     *  is pending, so each chunk is its own commit (the old mesh drawn until it is ready). */
    void FeedRemesh();
    /** Watches the debug colour variables: a change meshes the terrain again. While a land
     *  view is on, its legend is on screen. */
    void TickDebugViews();
    /** Stages PendingChanges with the edits the GPU data holds. */
    void StagePending();
    /** Whether the GPU path can run for this terrain (renderer, GPU density, centred lattice);
     *  warns once when not. */
    bool CheckAvailable();
    /** voxel.Debug.BrickPerf: once a second, logs and resets Perf. */
    void LogPerf(double Now);
    /** Hands the pages the current density and edits. */
    void FlushPages();
    /** How far the tree on screen can be from the camera in steady flight (actor units),
     *  which moves the morph bands nearer. ScreenTime: how long a tree stays on screen;
     *  Prediction: voxel.Render.BrickPrediction. */
    double MorphSlack(double ScreenTime, double Prediction) const;
    /** Sets the morph collection's camera (world, as a float and the rest), slack (world
     *  units) and voxel.Render.BrickMorph mode. */
    void UpdateMorphParameters(double Slack);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    TSharedPtr<FDensitySampleCompositor> Density;
    FVoxelBrickTerrainSettings Settings;

    // Game thread only.
    TMap<FVoxelBrickKey, FBrick> Bricks;
    TMap<FVoxelBrickKey, uint64> Wanted;   // bricks of the latest tree and their signatures
    TArray<FRecentEdit> RecentEdits;
    uint32 Generation = 0;
    double LastLaunchTime = 0.0;
    /** The latest result left nothing to do for its camera; no new task runs until the
     *  camera moves or an edit arrives. */
    bool bLastComplete = false;
    FVector LastTaskCamera = FVector::ZeroVector;

    /** Wanted waits to be committed since CommitPendingFrame, with the changes left (page by
     *  page when progressive); page members as committed; the wait, filtered. */
    bool bCommitPending = false;
    bool bCommitProgressive = false;
    uint64 CommitPendingFrame = 0;
    TArray<FVoxelBrickPageChange> PendingChanges;
    int32 NumInPage = 0;
    double CommitWaitSeconds = 0.0;
    /** A result that finished while a commit was meshed, waiting for it; the pending commit's
     *  task launch (0 for an edit commit) and the GPU edit data it is meshed with; launch to
     *  screen, measured and filtered. */
    TOptional<FTaskResult> HeldResult;
    double CommitLaunchTime = 0.0;
    uint32 CommitEditSequence = 0;
    double MeasuredLatency = 0.0;
    bool bLatencyMeasured = false;
    /** How long a tree stays on screen (between result commits, at most the latency),
     *  filtered; the latest such commit. */
    double ScreenPeriod = 0.0;
    double LastScreenCommitTime = 0.0;
    /** Held bricks in the reach of edits not yet committed (their mesh, and only their morph
     *  targets), and the newest such edit. */
    TSet<FVoxelBrickKey> EditBacklog;
    TSet<FVoxelBrickKey> TargetBacklog;
    uint32 EditBacklogSequence = 0;

    bool bAvailable = false;
    FVector LastCamera = FVector::ZeroVector;   // the latest Tick's camera (actor-local)
    /** Camera motion, filtered (the morph slack); the prediction's, which lets go of speed
     *  faster than it picks it up. */
    FVector CameraVelocity = FVector::ZeroVector;
    FVector PredictionVelocity = FVector::ZeroVector;
    double LastTickTime = 0.0;
    double FrameSeconds = 1.0 / 60.0;
    double TaskSeconds = 0.0;      // a task's run time, filtered
    double TreeLatency = 0.0;      // seconds from a task's launch until its tree is drawn
    TUniquePtr<FVoxelBrickPages> Pages;
    TSharedPtr<const FVoxelDensityGPUData> GPUDensity;
    TSharedPtr<const FVoxelEditGPUData> GPUEdits;
    uint32 GPUEditsSequence = 0;
    /** The edit tree flattened for the GPU on a worker (it grows with the edits). */
    struct FEditBuild
    {
        TSharedPtr<const FVoxelEditGPUData> Data;
        uint32 Sequence = 0;
        std::atomic<bool> bDone{ false };
    };
    TSharedPtr<FEditBuild, ESPMode::ThreadSafe> EditBuild;
    bool bWarnedUnavailable = false;
    bool bReportedMorph = false;
    /** The debug colours the terrain was last meshed with (voxel.Debug.LandView, LevelColor). */
    int32 LastLandView = 0;
    int32 LastLevelColor = 0;
    /** Bricks to mesh again for new debug colours, from RemeshHead (RemeshAll). */
    TArray<FVoxelBrickKey> RemeshQueue;
    int32 RemeshHead = 0;

    /** Time spent since the last voxel.Debug.BrickPerf line. Sections: drain, commit, flush
     *  pages, launch. */
    struct FPerf
    {
        double Start = 0.0;
        int32 Frames = 0;
        double SectionMs[4] = {};
        double MaxSectionMs[4] = {};
        int32 Tasks = 0;
        double TaskMs = 0.0;
        double MaxTaskMs = 0.0;
        double TreeMs = 0.0;
        int64 TaskSamples = 0;
        int32 TaskTested = 0;
        int32 TaskNew = 0;
        int32 TaskRestitched = 0;
        int32 TaskRebuilt = 0;
        int32 HeldResults = 0;         // results that waited for a commit
        int32 EditCommits = 0;
        double LatencyMs = 0.0;        // launch to screen, summed over the results committed
        double MaxLatencyMs = 0.0;
        int32 Latencies = 0;
        double Lead = 0.0;             // prediction ahead of the camera at launch, summed over tasks
        double MaxLead = 0.0;
        int32 Launches = 0;
        double Slack = 0.0;            // morph slack, summed over frames
        double MaxSlack = 0.0;
    };
    FPerf Perf;

    std::atomic<bool> bTaskRunning{ false };
    FCriticalSection ResultCS;
    TArray<FTaskResult> Results;
};
