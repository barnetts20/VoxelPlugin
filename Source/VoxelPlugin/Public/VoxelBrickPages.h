// VoxelBrickPages.h - The GPU backend of the brick terrain (Design/GPUPath.md stage c,
// Design/MeshingEngine.md). Bricks that may hold surface are grouped into pages of 4x4x4
// bricks of one level; each page is one RealtimeMesh component whose compute-writable buffers
// an on-demand compute provider fills in the frame. A page run copies the bricks it holds
// unchanged and packs the changed ones (VoxelBrickPageGPU.usf), which the stitched mesher
// (VoxelBrickMesherGPU) meshes for all pages at once in a per-frame batch. One indirect draw
// per page. Buffer sets are recycled between pages.
//
// Changes are committed as transactions: every page a commit touches is published and run in
// the same frame, so the terrain switches from one tree to the next between two frames, with
// no holes or overlaps. The GPU places every page's bricks before writing any: if one doesn't
// fit, nothing of that frame is drawn and the old tree stays; the pages are then resized to
// the exact counts read back and committed again together. No commit starts until the last
// one's verdict is read back. Pages whose vertices are partly blended toward the next coarser level
// (voxel.Render.BrickMorph) are morphed again when the camera moves.

#pragma once

#include "CoreMinimal.h"
#include "VoxelBrickTerrain.h"
#include "VoxelBrickMesherGPU.h"
#include <atomic>

class AActor;
class USceneComponent;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class FVoxelBrickPageProvider;
struct FVoxelBrickPageGPUInputs;

/** What a page's compute run lacked. Runs missing a Required item are skipped. */
enum class EVoxelPageMissing : uint32
{
    MesherShaders = 1 << 0,
    PageShaders = 1 << 1,
    Positions = 1 << 2,
    Tangents = 1 << 3,
    TexCoords = 1 << 4,
    Colors = 1 << 5,
    Triangles = 1 << 6,
    Args = 1 << 7,
    Capacity = 1 << 8,
    Required = MesherShaders | PageShaders | Positions | Triangles | Args | Capacity
};

/** What one run of a page's provider produced (read back a few frames later). */
struct FVoxelBrickPageStats
{
    uint64 Version = 0;
    uint32 VerticesNeeded = 0;
    uint32 TrianglesNeeded = 0;
    uint32 Errors = 0;             // OR of the mesher's error bits over the page's bricks
    uint32 VerticesWritten = 0;
    uint32 TrianglesWritten = 0;
    uint32 BricksDropped = 0;      // did not fit the page buffers
    uint32 VertexCapacity = 0;     // as the provider used them
    uint32 TriangleCapacity = 0;
    int32 Bricks = 0;
    bool bCommitted = true;        // false: a page of that frame's commit didn't fit, nothing was drawn
    /** Per member of that run: key, stamp, and vertices and triangles (0 if dropped). */
    TArray<FVoxelBrickKey> Keys;
    TArray<uint32> Stamps;
    TArray<uint32> Counts;
};

/** Game thread. The pages of one brick terrain. */
class FVoxelBrickPages
{
public:
    FVoxelBrickPages();
    ~FVoxelBrickPages();

    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelBrickTerrainSettings& InSettings);

    /** Destroys every page. */
    void Reset();

    /** True when every page Changes touch can show its new contents in the next frame: its
     *  buffers are registered and large enough, and the last commit has been read back as
     *  drawn. Otherwise starts getting the buffers ready (within voxel.Render.BrickPageCreates
     *  new buffer sets per frame) and returns false. */
    bool PrepareCommit(TConstArrayView<FVoxelBrickPageChange> Changes);

    /** PrepareCommit without the wait for the last commit's verdict: the buffers only. */
    bool PreparePages(TConstArrayView<FVoxelBrickPageChange> Changes);

    /** A commit's verdict is not read back yet, or a refused one is not committed again. */
    bool IsSettling() const;

    /** Applies Changes and publishes every page they touch; all of them run in this frame's
     *  graph. Call after PrepareCommit returned true for the same changes. */
    void Commit(TConstArrayView<FVoxelBrickPageChange> Changes);

    /** Once per frame, before any commit: registers buffer sets, reads back commit verdicts
     *  and commits refused pages again, collects the counters, and hands the camera (actor-local) to
     *  the morph, with the slack (cm) by which it may differ from the camera the tree on
     *  screen was built for. Commits push the density and edits given here. */
    void Flush(const TSharedPtr<const FVoxelDensityGPUData>& Density, const TSharedPtr<const FVoxelEditGPUData>& Edits, const FVector& Camera,
        double MorphSlack);

    /** Pages, buffer memory and the latest counters, for the debug log. */
    FString Describe() const;

    /** voxel.Debug.BrickPerf: page work since the last call, per frame, and resets it. */
    FString TakePerf();

    /** The mesher settings for this terrain's lattice. */
    const FVoxelBrickMesherGPUSettings& GetMesherSettings() const { return MesherSettings; }

    /** The page holding a brick: same level, 4x4x4 bricks. */
    static FVoxelBrickKey PageOf(const FVoxelBrickKey& Key);

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
        uint64 ReadyFrame = 0;         // no contents before (a recycled instance moves first)
        uint32 MorphCode = ~0u;        // what the last morph request covered
        FVector MorphCamera = FVector(1.0e30);
        float MorphSlack = 0.0f;

        bool IsValid() const { return Provider.IsValid(); }
    };

    struct FMember
    {
        int8 Levels[27] = {};
        uint32 Meshed = 0;
        uint32 Stamp = 0;              // new on every change: the brick must be meshed again
        int32 Vertices = -1;           // as read back for this stamp, or carried over; -1 unknown
        int32 Triangles = -1;
    };

    struct FPage
    {
        TMap<FVoxelBrickKey, FMember> Members;
        FInstance Current;             // drawn
        FInstance Next;                // larger buffers made ready for a commit or a repair
        uint64 Published = 0;
        uint64 PublishedFrame = 0;
        uint64 NextUsedFrame = 0;      // the last frame a commit or re-commit wanted Next
        FVoxelBrickPageStats Stats;    // latest read back
        bool bVerifying = false;       // the published version's verdict isn't read back yet
        bool bRecommit = false;        // refused: published again once it has RecommitCapacity
        FIntPoint RecommitCapacity = FIntPoint::ZeroValue;
    };

    /** A buffer set emptied by a commit: pooled once that commit is read back as drawn. If the
     *  commit was refused it still draws its page, and is emptied again with the re-commit. */
    struct FReleasing
    {
        FVoxelBrickKey PageKey;
        FInstance Instance;
        uint64 Version = 0;
        uint64 Frame = 0;
        bool bRefused = false;
    };

    FInstance CreateInstance(const FVoxelBrickKey& PageKey, int32 VertexCapacity, int32 TriangleCapacity);
    static void DestroyInstance(FInstance& Instance);
    /** Capacity for the page's members after Changes (vertices, triangles; powers of two). */
    FIntPoint EstimateCapacity(const FPage* Page, const FVoxelBrickKey& PageKey, TConstArrayView<const FVoxelBrickPageChange*> PageChanges) const;
    /** Emptied buffer sets of the level holding at least Capacity, at most four times it on
     *  each count; null if none. */
    TArray<FInstance>* FindPooled(int32 Level, FIntPoint Capacity);
    /** A buffer set for the page: a recycled one of the same level and at least the capacity,
     *  or a new one (registered with empty contents). Invalid when the owner is gone. */
    FInstance AcquireInstance(const FVoxelBrickKey& PageKey, int32 VertexCapacity, int32 TriangleCapacity);
    /** Empties the instance in this frame's commit (see FReleasing), or destroys it. */
    void ReleaseInstance(const FVoxelBrickKey& PageKey, FInstance& Instance);
    /** Keeps an emptied instance for reuse, or destroys it. */
    void PoolInstance(const FVoxelBrickKey& PageKey, FInstance& Instance);
    /** Commits the refused pages again, all in one frame, once every verdict is in and their
     *  buffers are large enough; empties again the instances the refused commit released. */
    void Recommit();
    /** Vertices and triangles with the margin, as powers of two within the limits. */
    static FIntPoint RoundCapacity(double Vertices, double Triangles);
    /** Buffer sets that may be created this frame (voxel.Render.BrickPageCreates, raised while a
     *  commit waits for buffers). */
    int32 CreateBudget() const;
    /** Registered, and past the frame it was registered or moved in. */
    static bool IsReady(const FInstance& Instance);
    /** Makes sure the page has an instance of at least Capacity ready to take its contents:
     *  Current, or a Next being prepared. True when it is ready now. */
    bool EnsureCapacity(const FVoxelBrickKey& PageKey, FPage& Page, FIntPoint Capacity, int32& InOutCreates);
    /** Publishes the page's members into its ready instance (Next replaces Current). */
    void Publish(const FVoxelBrickKey& PageKey, FPage& Page);
    /** Sends the members to the instance's provider at Version. */
    void Push(const FVoxelBrickKey& PageKey, const TMap<FVoxelBrickKey, FMember>& Members, uint64 Version, FInstance& Instance) const;
    void Advance(FInstance& Instance) const;
    /** Asks the provider for a morph when the camera moved through the page's blend bands. */
    bool RequestMorph(const FVoxelBrickKey& PageKey, FInstance& Instance, const FVector& Camera, float Slack, uint32 Mode) const;
    FBox PageBounds(const FVoxelBrickKey& PageKey) const;
    static FString DescribeMissing(uint32 Bits);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    FVoxelBrickTerrainSettings Settings;
    FVoxelBrickMesherGPUSettings MesherSettings;
    TMap<FVoxelBrickKey, FPage> Pages;   // keyed by level and page coordinate
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TSharedRef<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe> GPUInputs;
    uint32 WarnedMissing = 0;   // EVoxelPageMissing bits already logged
    uint64 NextVersion = 1;     // over all pages
    uint32 NextStamp = 0;
    int32 CreatesThisFrame = 0;
    uint64 CreatesFrame = 0;
    uint64 WaitingSince = 0;    // first frame a commit waited for buffers, 0 none
    /** Emptied instances by level and capacities, ready for another page. */
    TMap<uint64, TArray<FInstance>> Pool;
    int32 NumPooled = 0;
    TArray<FReleasing> Releasing;
    /** Read-back averages per brick and level (vertices, triangles), for new bricks. */
    TMap<int32, FVector2D> LevelAverages;

    /** Since the last TakePerf. */
    struct FPerf
    {
        int32 Frames = 0;
        double FlushMs = 0.0;
        double MaxFlushMs = 0.0;
        int32 Commits = 0;
        int32 CommitChanges = 0;
        int32 CommitPages = 0;
        int32 MaxCommitChanges = 0;
        int32 PrepareWaits = 0;        // PrepareCommit calls that returned false
        int32 VerifyWaits = 0;         // of which the last commit's verdict was not in
        int32 Refused = 0;             // pages refused by the GPU (a brick didn't fit)
        int32 Recommits = 0;
        int32 Created = 0;
        double CreateMs = 0.0;
        double MaxCreateMs = 0.0;
        int32 Recycled = 0;
        int32 MorphRequests = 0;
        int64 Calls = 0;               // as the providers counted them
        int64 IdleCalls = 0;
        int64 Runs = 0;
        int64 MorphRuns = 0;
        int64 BricksMeshed = 0;
        int64 BricksReused = 0;
        int64 RenderMicros = 0;
    };
    FPerf Perf;
};
