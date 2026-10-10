// VoxelBrickPages.h - The renderer of the brick terrain (Design/MeshingEngine.md). Bricks
// that may hold surface are grouped into pages of 8x8x8 bricks of one level; a page draws
// through one or more buffer sets (RealtimeMesh components whose compute-writable buffers an
// on-demand compute provider fills).
//
// A task result reaches the screen in three steps:
// 1. Stage: the bricks it changes are queued for the mesher, which runs a few batches per
//    frame on async compute (from a scene view extension) into a mesh cache, in the page
//    vertex format. Their exact vertex and triangle counts are read back.
// 2. Prepare: with the counts known, each page's changes are placed on the CPU: a brick goes
//    to the lowest free range of the first buffer set with room for it. A page whose sets
//    can't hold its new bricks gets more sets (spares), from the pool or created, each at
//    most MaxSlab large so no creation stalls a frame.
// 3. Commit: in one frame every touched buffer set runs once, copying its new bricks from the
//    cache and clearing the triangles of bricks that left. Unchanged bricks stay where they
//    are. The terrain switches from one tree to the next between two frames.
//
// Vertices are stored in units of the page's level (the component carries 2^level as its
// scale), so any pooled buffer set can serve any page. Buffer-set capacities come in classes
// of 2^k and 1.5 * 2^k. The mesh cache grows as staging needs and shrinks back to its recent
// peak when it empties.

#pragma once

#include "CoreMinimal.h"
#include "VoxelBrickTerrain.h"
#include "VoxelBrickMesherGPU.h"
#include "VoxelRangeAllocator.h"
#include <atomic>

class AActor;
class USceneComponent;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class ISceneViewExtension;
class UMaterialInterface;
class FVoxelBrickPageProvider;
struct FVoxelBrickPageGPU;

/** Game thread. The pages of one brick terrain. */
class FVoxelBrickPages
{
public:
    FVoxelBrickPages();
    ~FVoxelBrickPages();

    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelBrickTerrainSettings& InSettings);

    /** Destroys every page and drops what is being meshed. */
    void Reset();

    /** Changes the material of every buffer set's component (the pages keep their contents). */
    void SetMaterial(UMaterialInterface* InMaterial);

    /** Queues the bricks Changes add or change for meshing, replacing whatever was staged.
     *  bFirstFill: nothing is on screen, so mesh faster. */
    void Stage(TConstArrayView<FVoxelBrickPageChange> Changes, bool bFirstFill);

    /** The terrain is behind: a newer task result waits for the staged commit, which is then
     *  meshed at voxel.Render.BrickMeshBatchesBusy batches per frame. */
    void SetBusy(bool bInBusy) { bBusy = bInBusy; }

    /** True when Changes can be committed in this frame: every brick they add is meshed and
     *  its counts read back, and every page has room for its new bricks in ready buffer sets.
     *  Otherwise gets more buffer sets ready (within voxel.Render.BrickPageCreates new ones per
     *  frame) and returns false. */
    bool PreparePages(TConstArrayView<FVoxelBrickPageChange> Changes);

    /** PreparePages, counted for the perf log. bWhole: Changes are everything staged, so
     *  while the mesher is still at work only the new pages get buffer sets. */
    bool PrepareCommit(TConstArrayView<FVoxelBrickPageChange> Changes, bool bWhole);

    /** Applies Changes; every buffer set they touch runs in this frame's graph. Call after
     *  PreparePages returned true for the same changes. */
    void Commit(TConstArrayView<FVoxelBrickPageChange> Changes);

    /** Once per frame, before any commit: takes meshing results, sends the next batches to the
     *  mesher (with the density and edits given here), registers buffer sets, pools the
     *  emptied ones and shrinks an idle cache. */
    void Flush(const TSharedPtr<const FVoxelDensityGPUData>& Density, const TSharedPtr<const FVoxelEditGPUData>& Edits);

    /** Pages, buffer memory and staging, for the debug log. */
    FString Describe() const;

    /** voxel.Debug.BrickPerf: page work since the last call, per frame, and resets it. */
    FString TakePerf();

    /** The mesher settings for this terrain's lattice. */
    const FVoxelBrickMesherGPUSettings& GetMesherSettings() const { return MesherSettings; }

    /** The page holding a brick: same level, 8x8x8 bricks. */
    static FVoxelBrickKey PageOf(const FVoxelBrickKey& Key);

    /** The sea radius the elevation debug view measures from, for bricks sent to the mesher
     *  from now on (cm; 0: the density's sea level). */
    void SetDebugSeaRadius(double InSeaRadius) { Settings.DebugSeaRadius = InSeaRadius; }

private:
    struct FInstance
    {
        TWeakObjectPtr<URealtimeMeshSimple> Mesh;
        TWeakObjectPtr<URealtimeMeshComponent> Component;
        TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe> Provider;
        TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> bBuffersReady;
        uint64 Handle = 0;             // compute registration, 0 until registered
        int32 VertexCapacity = 0;
        int32 TriangleCapacity = 0;
        uint64 CreatedFrame = 0;
        uint64 PushedVersion = 0;      // the newest state sent to the provider
        uint64 LastKickFrame = 0;
        uint64 RegisteredFrame = 0;
        uint64 ReadyFrame = 0;         // not filled before (a recycled instance moves first)

        bool IsValid() const { return Provider.IsValid(); }
    };

    /** A buffer set of a page and the ranges its bricks hold. */
    struct FSlab
    {
        FInstance Instance;
        FVoxelRangeAllocator Vertices;
        FVoxelRangeAllocator Triangles;
        int32 NumMembers = 0;
        uint32 Id = 0;
    };

    struct FMember
    {
        uint32 Slab = 0;               // FSlab::Id, 0 for a brick without triangles
        int32 VertexStart = 0;
        int32 Vertices = 0;
        int32 TriangleStart = 0;
        int32 Triangles = 0;
        uint32 Errors = 0;             // the mesher's error bits
    };

    struct FPage
    {
        TMap<FVoxelBrickKey, FMember> Members;
        TArray<FSlab> Slabs;
        TArray<FInstance> Spares;      // ready for bricks the slabs can't hold
        uint64 SpareUsedFrame = 0;     // the last frame a commit wanted them
        uint32 NextSlabId = 0;
    };

    enum class EStage : uint8 { Queued, Meshing, Ready };

    /** A brick of the staged commit and its mesh in the cache. */
    struct FStaged
    {
        FVoxelMesherBrickGPU Brick;
        uint32 Stamp = 0;
        EStage State = EStage::Queued;
        bool bMeshable = true;         // its lattice indices fit 32 bits
        FIntPoint Expected = FIntPoint::ZeroValue;   // vertices, triangles (exact after a miss)
        int32 CacheVertex = 0;
        int32 Vertices = 0;
        int32 CacheTriangle = 0;
        int32 Triangles = 0;
        uint32 Errors = 0;
        bool bDebugColor = false;      // meshed with a land debug view: its copy writes its own colours
    };

    struct FQueued
    {
        FVoxelBrickKey Key;
        uint32 Stamp = 0;
    };

    /** A mesher batch in flight, and the cache regions it was given. */
    struct FBatch
    {
        TArray<FVoxelBrickKey> Keys;
        TArray<uint32> Stamps;
        FIntPoint Vertices = FIntPoint::ZeroValue;    // start, count
        FIntPoint Triangles = FIntPoint::ZeroValue;
    };

    /** Cache ranges a commit copied from, freed once every run of that commit is recorded. */
    struct FCacheRelease
    {
        TArray<TPair<TWeakPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>, uint64>> Runs;
        TArray<FIntPoint> Vertices;
        TArray<FIntPoint> Triangles;
        uint64 Frame = 0;
    };

    /** A buffer set emptied by a commit: pooled once that run is recorded. */
    struct FReleasing
    {
        FInstance Instance;
        uint64 Version = 0;
        uint64 Frame = 0;
    };

    /** A plan's view of one slab (the spares last). */
    struct FPlanSlab
    {
        FVoxelRangeAllocator Vertices;
        FVoxelRangeAllocator Triangles;
        TArray<FIntPoint> Freed;       // triangle ranges of departing bricks
        TArray<FIntPoint> Placed;      // triangle ranges of new bricks
        TArray<uint32> Jobs;
        int32 Joined = 0;
        int32 Left = 0;
        bool bTouched = false;
    };

    /** Every staged brick is meshed and its counts read back. */
    bool IsMeshed() const { return NumStagedReady >= Staged.Num(); }
    FInstance CreateInstance(int32 VertexCapacity, int32 TriangleCapacity);
    static void DestroyInstance(FInstance& Instance);
    /** Moves and scales the instance onto the page, with the page's level for the morph. */
    void PlaceInstance(const FVoxelBrickKey& PageKey, FInstance& Instance) const;
    /** Pooled buffer sets of Capacity's classes or up to VBP::PoolSlack classes larger on each
     *  count (at most twice as large). */
    TArray<FInstance>* FindPooled(FIntPoint Capacity);
    /** A pooled buffer set or a new one, placed on the page; invalid when the owner is gone. */
    FInstance AcquireInstance(const FVoxelBrickKey& PageKey, FIntPoint Capacity);
    /** Keeps an instance that draws nothing for reuse, or destroys it. */
    void PoolInstance(FInstance& Instance);
    /** Vertices and triangles with the margin, rounded up to a capacity class, within
     *  MinVertices / MinTriangles and MaxSlab. */
    FIntPoint RoundCapacity(double Vertices, double Triangles) const;
    /** Buffer sets that may be created this frame (raised while a commit waits for them). */
    int32 CreateBudget() const;
    /** Starts the creation count of a new frame. */
    void BeginCreates();
    /** Registered, and past the frame it was registered or moved in. */
    static bool IsReady(const FInstance& Instance);
    /** Gives the page one more spare holding Capacity, within the creation budget; false when
     *  none could be had this frame. */
    bool AddSpare(const FVoxelBrickKey& PageKey, FPage& Page, FIntPoint Capacity);
    /** Spares for Counts more vertices and triangles, each holding at least MinCounts, as many
     *  as the creation budget allows. */
    void AddSpares(const FVoxelBrickKey& PageKey, FPage& Page, FIntPoint Counts, FIntPoint MinCounts);
    /** While a whole commit is meshed: the pages it creates get their first buffer set. */
    void PrepareNewPages();
    /** The counts a brick is expected to mesh to, before it is meshed. */
    FIntPoint ExpectedCounts(const FVoxelBrickKey& Key) const;
    /** Places the page's changes (see the file comment) over its slabs, then its spares.
     *  Returns what none of them could hold (vertices, triangles); bOutUsesSpare: some brick
     *  goes to a spare. bApply: the page, its slabs and the staged bricks are updated (spares
     *  that take bricks become slabs) and the touched slabs pushed. */
    FIntPoint PlanPage(const FVoxelBrickKey& PageKey, FPage& Page, TConstArrayView<const FVoxelBrickPageChange*> PageChanges, bool bApply,
        FCacheRelease* Release, bool* bOutUsesSpare = nullptr);
    /** Sends a state (copy and clear jobs, triangles drawn) to the instance's provider. */
    void Push(const FVoxelBrickKey& PageKey, FInstance& Instance, TArray<uint32>&& Jobs, int32 TriangleEnd);
    void Advance(FInstance& Instance) const;
    /** Frees a staged brick's cache ranges, if it has any. */
    void DiscardStaged(const FStaged& Brick);
    /** Takes the mesher's read-back entries. */
    void TakeBatchResults();
    /** Sends queued bricks to the mesher, voxel.Render.BrickMeshBatches batches per frame. */
    void DispatchBatches();
    /** Tracks the cache's peak use and, once nothing is in it, shrinks it toward that peak. */
    void ShrinkCache();
    static FString DescribeMissing(uint32 Bits);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    FVoxelBrickTerrainSettings Settings;
    FVoxelBrickMesherGPUSettings MesherSettings;
    /** The largest buffer set (vertices, triangles): about 5 MB, and at least two of the
     *  mesher's largest bricks. */
    FIntPoint MaxSlab = FIntPoint::ZeroValue;
    TMap<FVoxelBrickKey, FPage> Pages;   // keyed by level and page coordinate
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TSharedRef<FVoxelBrickPageGPU, ESPMode::ThreadSafe> GPU;
    TSharedPtr<ISceneViewExtension, ESPMode::ThreadSafe> MeshExtension;
    uint32 WarnedMissing = 0;   // missing-item bits already logged (VoxelBrickPagesDetail::EMissing)
    bool bWarnedReadback = false;   // a mesher readback that returned nothing was logged
    uint64 NextVersion = 1;     // over all buffer sets
    int32 CreatesThisFrame = 0;
    uint64 CreatesFrame = 0;
    uint64 WaitingSince = 0;    // first frame a commit waited for buffer sets, 0 none
    /** Emptied instances by capacities, ready for any page. */
    TMap<uint64, TArray<FInstance>> Pool;
    int32 NumPooled = 0;
    TArray<FReleasing> Releasing;
    /** Read-back averages per brick and level (vertices, triangles), for estimates. */
    TMap<int32, FVector2D> LevelAverages;

    // Staging: the bricks of the commit being prepared, the mesher queue (from QueueHead),
    // the batches in flight, and the cache's free ranges (vertices, triangles).
    TMap<FVoxelBrickKey, FStaged> Staged;
    TArray<FQueued> MeshQueue;
    int32 QueueHead = 0;
    int32 NumStagedReady = 0;
    uint32 NextStamp = 0;
    uint32 NextBatchId = 0;
    TMap<uint32, FBatch> Batches;
    FVoxelRangeAllocator CacheVertices;
    FVoxelRangeAllocator CacheTriangles;
    TArray<FCacheRelease> CacheReleases;
    /** The cache's peak use (vertices, triangles) in the current and the previous window of
     *  VBP::CachePeakFrames. */
    FIntPoint CachePeak = FIntPoint::ZeroValue;
    FIntPoint CachePeakBefore = FIntPoint::ZeroValue;
    uint64 CachePeakFrame = 0;
    uint64 NoShrinkUntil = 0;      // after cache ranges were freed by the timeout
    /** Pages the staged commit creates and their expected counts (vertices, triangles). */
    TArray<TPair<FVoxelBrickKey, FIntPoint>> NewPages;
    bool bFirstFill = false;
    bool bBusy = false;
    bool bPrepareMeshed = false;   // the last PreparePages found every brick meshed

    /** Since the last TakePerf. */
    struct FPerf
    {
        int32 Frames = 0;
        double FlushMs = 0.0;
        double MaxFlushMs = 0.0;
        int32 Commits = 0;
        int32 CommitChanges = 0;
        int32 CommitPages = 0;
        int32 CommitSlabs = 0;
        int32 CommitJobs = 0;
        int32 MaxCommitChanges = 0;
        int32 MeshWaits = 0;           // PrepareCommit calls waiting for the mesher
        int32 SlabWaits = 0;           // and for buffer sets
        int32 Batches = 0;
        int32 BricksSent = 0;
        int32 BricksBack = 0;          // didn't fit their batch's cache region: sent again
        int32 Created = 0;
        double CreateMs = 0.0;
        double MaxCreateMs = 0.0;
        int32 Recycled = 0;
        int32 SparesAdded = 0;
        int32 CacheShrinks = 0;
        FIntPoint CacheUsed = FIntPoint::ZeroValue;   // peak
        int64 Calls = 0;               // as the providers counted them
        int64 IdleCalls = 0;
        int64 Runs = 0;
        int64 RenderMicros = 0;
    };
    FPerf Perf;
};
