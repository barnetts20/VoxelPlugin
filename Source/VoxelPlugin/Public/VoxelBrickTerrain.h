// VoxelBrickTerrain.h - Terrain rendered from bricks (VoxelBrickMesher) in a level-of-detail
// tree around the camera. The tree is built on the CPU; the bricks are meshed on the CPU (one
// component per brick) or on the GPU (VoxelBrickPages), chosen with voxel.Render.BrickBackend.
// Switched against the octree renderer with voxel.Render.Bricks.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "VoxelBrickMesher.h"
#include <atomic>

class AActor;
class USceneComponent;
class UMaterialInterface;
class UMaterialParameterCollection;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class FDensitySampleCompositor;
struct FMeshStreamData;
struct FVoxelDensityGPUData;
struct FVoxelEditGPUData;
class FVoxelBrickPages;

/** A brick of the LOD tree: level 0 has the finest cells, each level up doubles the cell size.
 *  Coordinates count bricks of that level from the shared lattice origin. */
struct FVoxelBrickKey
{
    int32 Level = 0;
    FInt64Vector Coord;

    bool operator==(const FVoxelBrickKey& Other) const
    {
        return Level == Other.Level && Coord == Other.Coord;
    }

    friend uint32 GetTypeHash(const FVoxelBrickKey& Key)
    {
        return HashCombine(::GetTypeHash(Key.Level), GetTypeHash(Key.Coord));
    }
};

/** One brick change of a GPU page commit (FVoxelBrickPages::Commit). */
struct FVoxelBrickPageChange
{
    FVoxelBrickKey Key;
    bool bRemove = false;
    int8 Levels[27] = {};          // neighbour levels relative to the brick (VoxelBrickMesher)
    uint32 Meshed = 0;             // same-level neighbours that are meshed
};

/** Inputs of the brick terrain, fixed until the next Initialize. */
struct FVoxelBrickTerrainSettings
{
    FVector Origin = FVector::ZeroVector;   // lattice point 0 (-RootExtent on each axis)
    double FinestCell = 100.0;              // level-0 cell size (ActualPrecision)
    int32 CellsPerBrick = 32;
    int32 MaxLevel = 0;                     // level whose single brick covers the whole root
    double LodDistanceFactor = 2.5;         // a brick splits when the camera is closer than this many brick sizes
    double SlopeBound = 3.5;
    double TriplanarUVScale = 0.0001;
    TWeakObjectPtr<UMaterialInterface> Material;
    /** GPU backend: receives the camera and morph slack every frame (VoxelMorph.ush). */
    TWeakObjectPtr<UMaterialParameterCollection> MorphParameters;
};

/** Brick terrain of one terrain actor.
 *
 *  Every TaskInterval a worker task walks the tree from the root brick, for the camera
 *  predicted to the time the tree will be on screen: a brick is culled
 *  when its centre sample proves it free of surface, split while the camera is within
 *  LodDistanceFactor brick sizes (1.15 times that if the previous tree split it), and
 *  otherwise wanted. The tree is then balanced so that
 *  touching leaves differ by at most one level, and each wanted brick is meshed stitched to
 *  its neighbours' levels (VoxelBrickMesher::MeshBrickStitched). Wanted bricks without a
 *  mesh are meshed nearest first within a sample budget; held bricks whose neighbour levels
 *  changed are meshed again.
 *
 *  CPU backend: on the game thread a brick that is no longer wanted stays until the bricks
 *  replacing it are on screen, and a restitched mesh waits until the neighbours it was
 *  stitched to are on screen, so swaps open neither holes nor cracks.
 *
 *  GPU backend: each task result is committed to the pages whole, in one frame, once every
 *  page it touches is ready (FVoxelBrickPages::PrepareCommit), so the tree on screen switches
 *  from one frame to the next. Only the first fill, with nothing on screen, commits page by
 *  page. No task starts while a commit waits. */
class VOXELPLUGIN_API FVoxelBrickTerrain : public TSharedFromThis<FVoxelBrickTerrain>
{
public:
    FVoxelBrickTerrain();
    ~FVoxelBrickTerrain();

    /** Game thread. Drops every brick and starts over. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, TSharedPtr<FDensitySampleCompositor> InDensity,
        const FVoxelBrickTerrainSettings& InSettings);

    /** Game thread. Destroys every brick's component; results of a running task are dropped. */
    void Reset();

    /** Game thread. The collection the terrain material's morph reads (Settings.MorphParameters). */
    void SetMorphParameters(UMaterialParameterCollection* InCollection);

    /** Game thread. See FVoxelCollisionBricks::NotifyEdit. */
    void NotifyEdit(uint32 Sequence, const FVector& Center, double Radius);

    /** Game thread, every frame. Camera in actor-local space. */
    void Tick(const FVector& Camera);

    int32 NumBricks() const { return Bricks.Num(); }

    /** Game thread. Bricks of the latest tree and their signatures (neighbour levels, see
     *  RunTask), and the settings they were built with. */
    const TMap<FVoxelBrickKey, uint64>& GetWantedSignatures() const { return Wanted; }
    const FVoxelBrickTerrainSettings& GetSettings() const { return Settings; }

    /** Game thread. True if the brick holds a mesh with surface (a component), or on the
     *  GPU backend may hold surface (a member of a page, as committed). */
    bool HoldsSurface(const FVoxelBrickKey& Key) const;

    /** Debug vertex colour of a level (voxel.Debug.ChunkColor). */
    static FColor LevelColor(int32 Level);

private:
    struct FBrick
    {
        TWeakObjectPtr<URealtimeMeshSimple> Mesh;
        TWeakObjectPtr<URealtimeMeshComponent> Component;
        TSharedPtr<FMeshStreamData> Pending;
        /** Set from the update futures (any thread): the first frame some mesh of the current
         *  component can be on screen, 0 until then. */
        TSharedPtr<std::atomic<uint64>> ShownFrame;
        uint64 CreatedFrame = 0;
        /** Neighbour levels and seam mode the applied (GPU: committed) mesh was built for, and
         *  the pending one. */
        uint64 Signature = 0;
        uint64 PendingSignature = 0;
        /** The pending mesh is stitched to these bricks and waits until they are on screen. */
        TArray<FVoxelBrickKey> WaitFor;
        bool bHasPending = false;
        bool bApplied = false;        // a mesh (or, empty, nothing) has been applied or committed
        bool bHasSection = false;
        bool bDirty = false;
        bool bProvenEmpty = false;    // the newest mesh is empty by the empty test
        /** GPU backend: a page member as committed, and built by the pending result (meshed
         *  again at the commit even if its signature is unchanged). */
        bool bInPage = false;
        bool bRemesh = false;
    };

    struct FBuilt
    {
        FVoxelBrickKey Key;
        TSharedPtr<FMeshStreamData> Stream;   // null when the brick holds no surface
        uint64 Signature = 0;
        TArray<FVoxelBrickKey> Neighbours;    // wanted bricks touching it
        bool bProvenEmpty = false;            // the empty test skipped it
        bool bGPU = false;                    // GPU backend: tested only, meshed by the pages
    };

    struct FTaskInput
    {
        uint32 Generation = 0;
        FVector Camera = FVector::ZeroVector;
        TMap<FVoxelBrickKey, uint64> Known;   // held bricks and the signature of their newest mesh
        TArray<FVoxelBrickKey> Rebuild;
        TSet<FVoxelBrickKey> ProvenEmpty;     // held bricks whose newest mesh the empty test skipped
        TMap<FVoxelBrickKey, uint64> PrevWanted;   // the latest tree (merge hysteresis)
        int32 SeamMode = 0;
        int32 Backend = 0;                    // 0 CPU meshes, 1 GPU pages
        TSharedPtr<FDensitySampleCompositor> Density;
        FVoxelBrickTerrainSettings Settings;
    };

    struct FTaskResult
    {
        uint32 Generation = 0;
        uint32 StartSequence = 0;
        TMap<FVoxelBrickKey, uint64> Wanted;   // and their signatures
        TArray<FBuilt> Built;
        FVector Camera = FVector::ZeroVector;
        int32 SeamMode = 0;
        bool bComplete = false;               // every wanted brick has a mesh for its signature
        int32 BalanceSplits = 0;
        int32 NumRestitched = 0;
        int32 NumRestitchedEmpty = 0;
        int64 Samples = 0;
        double Milliseconds = 0.0;
        double TreeMilliseconds = 0.0;        // the descent and the balance
    };

    struct FRecentEdit
    {
        uint32 Sequence;
        FBox Bounds;
    };

    static void RunTask(const TSharedRef<FVoxelBrickTerrain>& Self, FTaskInput Input);

    FVoxelBrickGrid GridFor(int32 Level) const;
    FBox BoundsOf(const FVoxelBrickKey& Key) const;
    bool IsOnScreen(const FVoxelBrickKey& Key) const;
    /** Wanted, on screen, and showing a mesh stitched to the current tree. */
    bool IsSettled(const FVoxelBrickKey& Key) const;
    /** Why a brick is not settled (CPU backend, debug counts): 0 settled, 1 not held, 2 not
     *  wanted or not applied, 3 stitched to other neighbours, 4 on a component not yet drawn. */
    int32 StallOf(const FVoxelBrickKey& Key) const;
    void TouchingWanted(const FVoxelBrickKey& Key, TArray<FVoxelBrickKey>& Out) const;
    bool IsCovered(const FVoxelBrickKey& Key, const TArray<FVoxelBrickKey>& Descendants, int32* OutStall = nullptr) const;

    void DrainResults();
    /** CPU backend. */
    void ApplyQueued();
    void RemoveReplaced();
    /** GPU backend: commits PendingChanges when their pages are ready. */
    void CommitPages();
    /** The page changes from the committed tree to Wanted. */
    void BuildCommit(TArray<FVoxelBrickPageChange>& Out) const;
    /** Records committed changes on the bricks. */
    void ApplyCommitted(TConstArrayView<FVoxelBrickPageChange> Changes);
    bool EnsureComponent(const FVoxelBrickKey& Key, FBrick& Brick);
    static void DestroyComponent(FBrick& Brick);
    /** The backend voxel.Render.BrickBackend asks for, if this terrain can run it. */
    int32 ResolveBackend();
    /** voxel.Debug.BrickPerf: once a second, logs and resets Perf. */
    void LogPerf(double Now);
    /** GPU backend: hands the pages the current density and edits, and the material the
     *  camera with the morph slack the tree's latency calls for. */
    void FlushPages(double TaskInterval);
    /** Sets the morph collection's camera (world, as a float and the rest), slack (world
     *  units) and voxel.Render.BrickMorph mode. */
    void UpdateMorphParameters(double Slack);
    /** CPU backend: a brick a pending mesh is stitched to is on screen (or no longer wanted). */
    bool IsNeighbourReady(const FVoxelBrickKey& Key) const;

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    TSharedPtr<FDensitySampleCompositor> Density;
    FVoxelBrickTerrainSettings Settings;

    // Game thread only.
    TMap<FVoxelBrickKey, FBrick> Bricks;
    TMap<FVoxelBrickKey, uint64> Wanted;   // bricks of the latest tree and their signatures
    TArray<FVoxelBrickKey> ApplyQueue;
    TArray<FRecentEdit> RecentEdits;
    uint32 Generation = 0;
    bool bHasWanted = false;      // a task result has set Wanted since the last Reset
    double LastLaunchTime = 0.0;
    /** The latest result left nothing to do for its camera; no new task runs until the
     *  camera moves, an edit arrives or the seam mode changes. */
    bool bLastComplete = false;
    FVector LastTaskCamera = FVector::ZeroVector;
    int32 LastSeamMode = -1;

    /** RemoveReplaced's latest pass, for the debug log: held bricks not wanted, removed, and
     *  the kept ones by the first StallOf reason that blocked them. */
    int32 RemovalCandidates = 0;
    int32 RemovalRemoved = 0;
    int32 RemovalStalls[5] = {};

    /** GPU backend: Wanted waits to be committed since CommitPendingFrame, with the changes
     *  left (page by page when progressive); page members as committed; the wait, filtered. */
    bool bCommitPending = false;
    bool bCommitProgressive = false;
    uint64 CommitPendingFrame = 0;
    TArray<FVoxelBrickPageChange> PendingChanges;
    int32 NumInPage = 0;
    double CommitWaitSeconds = 0.0;

    int32 Backend = 0;
    FVector LastCamera = FVector::ZeroVector;   // the latest Tick's camera (actor-local)
    /** Camera motion, filtered, for the task's camera prediction and the morph slack. */
    FVector CameraVelocity = FVector::ZeroVector;
    double LastTickTime = 0.0;
    double FrameSeconds = 1.0 / 60.0;
    double TaskSeconds = 0.0;      // a task's run time, filtered
    double TreeLatency = 0.0;      // seconds from a task's launch until its tree is drawn
    TUniquePtr<FVoxelBrickPages> Pages;
    TSharedPtr<const FVoxelDensityGPUData> GPUDensity;
    TSharedPtr<const FVoxelEditGPUData> GPUEdits;
    uint32 GPUEditsSequence = 0;
    bool bWarnedBackend = false;
    bool bWarnedMorph = false;

    /** Time spent since the last voxel.Debug.BrickPerf line. Sections: drain, apply (GPU:
     *  commit), remove, flush pages, launch. */
    struct FPerf
    {
        double Start = 0.0;
        int32 Frames = 0;
        double SectionMs[5] = {};
        double MaxSectionMs[5] = {};
        int32 Tasks = 0;
        double TaskMs = 0.0;
        double MaxTaskMs = 0.0;
        double TreeMs = 0.0;
        int64 TaskSamples = 0;
        int32 TaskBuilt = 0;
    };
    FPerf Perf;

    std::atomic<bool> bTaskRunning{ false };
    FCriticalSection ResultCS;
    TArray<FTaskResult> Results;
};
