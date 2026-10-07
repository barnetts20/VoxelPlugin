// AdaptiveVoxelActor.h — Standalone or planet-driven terrain actor that owns an
// adaptive octree and manages its async LOD/mesh/edit update pipeline.
//
// Scale determines planet radius (max component, in cm). The octree is built at
// world scale in actor-local space. Position and rotation are handled live by the
// actor transform; scale changes trigger full reconstruction.
//
// When spawned by APlanetActor (bIsPlanetOwned=true), the compositor is shared
// and planet-driven properties become read-only on this actor.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "MaterialDomain.h"
#include "FAdaptiveOctree.h"
#include "VoxelDensity.h"
#include "VoxelCollisionBricks.h"
#include "RealtimeMeshActor.h"
#include "AdaptiveVoxelActor.generated.h"

/** Terrain actor that owns an adaptive octree and drives its async update pipeline.
 *
 *  Three async task chains run on background threads, gated by atomic flags:
 *    - DataUpdate: LOD split/merge and chunk cut (writes octree)
 *    - MeshUpdate: collects dirty chunks for the game-thread apply (reads octree)
 *    - EditUpdate: applies brush edits and rebuilds affected chunks (writes octree)
 *
 *  DataUpdate and MeshUpdate chain continuously with a configurable minimum interval.
 *  EditUpdate runs independently when triggered. All three acquire OctreeLock.
 *  The game thread applies finished meshes in Tick, a whole octree pass at a time.
 *  Collision is separate: FVoxelCollisionBricks meshes the density around each pawn at the
 *  finest cell size on its own worker task, without the octree lock. */
UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

private:
    /** The adaptive octree that holds all terrain data. Created on the background
     *  thread during the first RunDataUpdateTask after Initialize sets PendingParams. */
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;

    /** Deferred construction: params are stored on Initialize (game thread),
     *  octree is built on the first RunDataUpdateTask (background thread). */
    TSharedPtr<FOctreeParams> PendingParams;

    /** Mesh chunks attach to this component. Inherits actor position and rotation
     *  but uses absolute scale (1,1,1). Octree is built at world scale; scale changes
     *  trigger full reconstruction via OnConstruction. */
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    /** Camera position in actor-local space, updated each tick from the viewport. */
    FVector CameraPosition = FVector::ZeroVector;

    /** Last position used for an LOD update — compared against current to detect movement. */
    FVector LastLodUpdatePosition = FVector(FLT_MAX);

    double CameraFOV = 90;

    /** Read-write lock protecting AdaptiveOctree. DataUpdate and EditUpdate acquire
     *  write; MeshUpdate acquires read-only. */
    FRWLock OctreeLock;

    bool bTickInEditor = true;

    /** Set to true after Initialize completes param setup. Gates Tick and ShouldTickIfViewportsOnly. */
    std::atomic<bool> Initialized = false;

    /** Set in BeginDestroy to prevent async tasks from touching a dying actor. */
    std::atomic<bool> IsDestroyed = false;

    /** Last scale at which Initialize ran — used to detect scale changes in OnConstruction. */
    FVector LastInitScale = FVector::ZeroVector;

    /** Cached scale set by the planet actor, used by the transform guard to restore
     *  position/rotation/scale when the editor tries to modify locked components. */
    FVector PlanetDrivenScale = FVector::OneVector;

    /** Bound to RootComponent->TransformUpdated when planet-owned.
     *  Reverts any editor-driven transform changes on locked components. */
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);

public:
    AAdaptiveVoxelActor();

    /** True when this actor is spawned and driven by an APlanetActor.
     *  Planet-driven properties (NoiseAmplitudeRatio, SurfaceMaterial) become
     *  read-only on this actor — edit them on the planet instead. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain")
    bool bIsPlanetOwned = false;

    /** Surface material applied to all mesh chunks. When standalone, auto-loads the
     *  plugin's triplanar material on first init. When planet-owned, set by the planet. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials",
        meta = (EditCondition = "!bIsPlanetOwned"))
    UMaterialInterface* SurfaceMaterial = nullptr;

    // --- Terrain Geometry ---

    /** Ratio of noise amplitude to planet radius. Surface elevation ranges from
     *  PlanetRadius to PlanetRadius * (1 + NoiseAmplitudeRatio). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain",
        meta = (ClampMin = "0.01", ClampMax = "1.0", EditCondition = "!bIsPlanetOwned"))
    double NoiseAmplitudeRatio = 0.15;

    /** Shape of the surface when standalone. When planet-owned, the planet's settings are used. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain",
        meta = (EditCondition = "!bIsPlanetOwned"))
    FVoxelSurfaceSettings SurfaceSettings;

    // --- Octree Structure ---

    /** Computed at init: the depth the chunk cut is built at (= MinChunkDepth). Shown for
     *  reference. Chunks promote deeper than this near the camera. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Octree")
    int ChunkDepth = 4;

    /** Shallow floor the chunk cut is built at — the spawn / far-away chunk depth. Keep it
     *  low (2-3) so spawn and distant terrain use few, large components. Precision is
     *  recovered near the camera by promoting toward MaxChunkDepth. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "2"))
    int MinChunkDepth = 3;

    /** Deepest a chunk root may sit — the near-camera precision ceiling. Chunks under the
     *  camera promote toward this until their float jitter is sub-pixel. Raise it for very
     *  large terrestrial planets that need walking-precision on the surface (a 10,000 km
     *  planet wants ~12-14 for sub-mm); lower it to cap near-camera component count. Must
     *  be <= MaxDepth. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "2"))
    int MaxChunkDepth = 14;

    /** Screen-space tolerance for chunk float jitter (fraction of the view). A chunk
     *  promotes when its worst-case FVector3f jitter would project larger than this at the
     *  camera's near distance. Smaller = promote sooner / finer origins / more components.
     *  ~5e-4 targets roughly half a pixel at 1080p. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "0.00001"))
    double ChunkPrecisionThreshold = 5e-4;

    /** Demote hysteresis (0-1]. A promoted chunk only collapses back once its parent is
     *  precise to ChunkPrecisionThreshold * this. Lower = wider dead zone / less
     *  create-destroy churn near the promote/demote boundary, at the cost of holding finer
     *  chunks slightly longer as you leave. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "0.05", ClampMax = "1.0"))
    double ChunkDemoteHysteresis = 0.5;

    /** Restricted-octree balance: largest depth gap allowed between face-adjacent chunks.
     *  Chunks force-refine (and demotes are refused) to hold this, preventing the >2-level
     *  boundaries the mesher can't stitch. 1 = strict 2:1 balance (most components, safest);
     *  2 = steeper funnel / fewer components, usually still crack-free. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "1", ClampMax = "2"))
    int MaxChunkDepthDelta = 1;

    /** Master switch for terrain collision. When on, collision bricks are built within the
     *  collision distance of every player pawn (of the camera when there is none). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    bool bEnableSurfaceCollision = true;

    /** Collision radius kept around a pawn regardless of speed (cm). ~75 m by default. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "0"))
    double WalkBaseCollisionDistance = 7500.0;

    /** Seconds of lead the collision radius adds per unit speed: radius = WalkBase + Speed *
     *  this, up to MaxCollisionDistance. Covers building and cooking ahead of a moving pawn. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "0"))
    double CollisionCookLeadTime = 0.3;

    /** Largest collision radius (cm). Bricks are built at the finest cell size, so the cost
     *  grows with the square of the radius. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "0", UIMin = "5000", UIMax = "100000"))
    double MaxCollisionDistance = 25000.0;

    /** Bricks are dropped beyond this multiple of the collision radius, so a brick on the
     *  boundary doesn't rebuild over and over. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "1.0", UIMax = "3.0"))
    double CollisionHysteresis = 1.3;

    /** Collision cell size as a multiple of the finest render cell (ActualPrecision). 1 matches
     *  the closest render LOD; 2 is about 8x cheaper and a little coarser. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "1", UIMax = "4"))
    int32 CollisionCellScale = 1;

    /** Steepest density gradient assumed when proving a brick empty (1 = flat ground; the
     *  default surface reaches about 2.8). Too low can leave holes on steep ground; the
     *  bricks raise it at run time, with a warning, if a steeper surface shows up. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "1.0", UIMax = "8.0"))
    double CollisionSlopeBound = 3.5;

    /** Minimum subdivision depth maintained regardless of camera distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree")
    int MinDepth = 5;

    /** Target voxel spacing in world units (cm). MaxDepth is computed automatically
     *  so the finest LOD voxel cells are approximately this size. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 150.0;

    /** Computed from TargetPrecision and planet radius at init time.
     *  Clamped to [MinDepth, MaxKeyDepth].
     *  At default scale (100M radius, 0.15 NAR, TargetPrecision=100): MaxDepth=22. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Octree")
    int MaxDepth = 22;

    /** The actual voxel spacing (cm) achieved at MaxDepth after key-limit clamping. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Octree")
    double ActualPrecision = 0.0;

    /** Hard limit imposed by FMortonIndex (3 bits per level, 126 bits across two uint64s). */
    static constexpr int32 MaxKeyDepth = 42;

    // --- LOD ---

    /** Screen-space size threshold for LOD split/merge decisions. Smaller values
     *  produce finer detail at greater distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double ScreenSpaceThreshold = 0.075;

    /** Minimum interval (seconds) between LOD update passes. Prevents the
     *  DataUpdate→MeshUpdate chain from saturating the thread pool. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double MinDataUpdateInterval = 0.05;

    /** Multiplier on camera velocity for predictive LOD. Higher values split nodes
     *  earlier in the direction of camera movement. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double VelocityLookAheadFactor = 2.0;

    /** Per-frame game-thread time budget (milliseconds) for creating chunk components
     *  (RegisterComponent + section-group setup), the expensive part of a chunk's first
     *  appearance. Creation runs oldest work first; mesh swaps are not budgeted, since a
     *  group of chunks meshed together must appear in the same frame. The drain runs every
     *  frame, not on the slower data/mesh loop cadence. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD", meta = (ClampMin = "0.1"))
    double ChunkApplyBudgetMs = 2.0;

    // --- Lifecycle Overrides ---

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

    // --- Editor Property & Transform Locking ---

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual bool CanEditChange(const FProperty* InProperty) const override;
    virtual void PostEditMove(bool bFinished) override;
    virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
#endif

    // --- Public API ---

    USceneComponent* GetMeshAttachmentRoot() const { return MeshAttachmentRoot; }

    /** Called by APlanetActor — uses the provided compositor (shared noise + edit store)
     *  instead of creating one. InScale sets the actor's scale (= planet radius) before
     *  initialization reads it. If InAttachParent is provided, MeshAttachmentRoot is
     *  re-parented to it. */
    void InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
        USceneComponent* InAttachParent = nullptr,
        FVector InScale = FVector::ZeroVector);

    /** Tears down the current octree and rebuilds from scratch using current property values.
     *  Called by OnConstruction (scale change), PostEditChangeProperty (structural param change),
     *  InitializeFromPlanet (planet-driven init), and BeginPlay. */
    void Initialize();

protected:
    // --- Async Task Flags ---
    // Each flag gates its corresponding task so only one instance runs at a time.

    std::atomic<bool> DataUpdateIsRunning = false;
    std::atomic<bool> MeshUpdateIsRunning = false;
    std::atomic<bool> EditUpdateIsRunning = false;

    /** Timer handle for the MinDataUpdateInterval delay between MeshUpdate completion
     *  and the next DataUpdate kickoff. */
    FTimerHandle DataUpdateTimerHandle;

    /** Game-thread apply queue. The mesh/edit tasks enqueue dirty chunks here (deduped
     *  via FMeshChunk::bApplyQueued); Tick drains it (DrainPendingApply). Guarded by
     *  PendingApplyCS. */
    TArray<TSharedPtr<FMeshChunk>> PendingApply;
    FCriticalSection PendingApplyCS;

    /** Chunks retired by the chunk-cut pass (coarse parents replaced by finer children).
     *  Their RealtimeMesh components must be destroyed on the game thread; Tick drains this
     *  after DrainPendingApply so the finer children get a chance to appear first. Guarded
     *  by PendingApplyCS (same low-contention queue family as PendingApply). */
    TArray<TSharedPtr<FMeshChunk>> PendingDestroy;

    /** The density every terrain consumer samples (set by Initialize / InitializeFromPlanet). */
    TSharedPtr<FDensitySampleCompositor> DensityCompositor;

    /** Collision around the pawns, built from DensityCompositor. */
    TSharedPtr<FVoxelCollisionBricks> CollisionBricks;

    /** Edits applied by the edit task, for the collision bricks (drained in Tick). Guarded by
     *  PendingApplyCS. */
    struct FEditNotice
    {
        uint32 Sequence;
        FVector Center;
        double Radius;
    };
    TArray<FEditNotice> PendingEditNotices;

    /** Smoothed camera speed (cm/s) from position deltas, updated in Tick. Feeds the
     *  collision lead distance when a pawn reports no velocity. */
    double PlayerSpeed = 0.0;
    FVector PrevSpeedSamplePos = FVector::ZeroVector;
    bool bHasSpeedSample = false;

    // --- Flicker diagnostics (voxel.Debug.Flicker) ---

    /** Actor-local bounds of chunks the last stale-seam check flagged, and when it ran.
     *  Written by the data task, drawn by Tick. Guarded by PendingApplyCS. */
    TArray<FBox> DebugStaleBounds;
    double DebugStaleBoundsTime = 0.0;

    /** Actor-local bounds of chunks whose current mesh holds folded triangles, refreshed
     *  every data pass. Guarded by PendingApplyCS. */
    TArray<FBox> DebugFoldedBounds;

    /** Data passes since init (data task only); paces the stale-seam check. */
    uint32 DebugDataPassCount = 0;

    /** Last time the mesh task logged chunk statistics (mesh task only). */
    double DebugLastStatsTime = 0.0;

    /** Consecutive frames the oldest apply group has been waiting for component creation
     *  (game thread only). Caps how long it holds newer groups back. */
    int32 ApplyHeldFrames = 0;

    /** Destroys all RealtimeMesh components attached to MeshAttachmentRoot. */
    void CleanSceneRoot();

    /** Background task: builds the octree (if PendingParams set), then runs one LOD
     *  update pass. Chains to RunMeshUpdateTask on completion. */
    void RunDataUpdateTask();

    /** Background task: collects dirty chunks (under the read lock) and enqueues them for
     *  the game-thread drain via EnqueueDirtyChunksForApply, then chains the next
     *  RunDataUpdateTask on the normal interval. The apply itself is decoupled (Tick). */
    void RunMeshUpdateTask();

    /** Adds dirty chunks to PendingApply, skipping any already queued. Callable from any
     *  thread -- takes only PendingApplyCS. */
    void EnqueueDirtyChunksForApply(const TArray<TSharedPtr<FMeshChunk>>& DirtyChunks);

    /** Game thread, every Tick. Captures each queued chunk's latest stream, creates missing
     *  components oldest work first within ChunkApplyBudgetMs, then applies: initial-build
     *  chunks as soon as their component exists, later work in groups of overlapping pass
     *  ranges, each group in one frame once all its components exist -- chunks meshed
     *  together (a changed chunk and its neighbours) appear together. The oldest waiting
     *  group holds back newer ones for a bounded number of frames. No octree lock: each
     *  apply reads an immutable published stream. */
    void DrainPendingApply();

    /** Game-thread: destroys the RealtimeMesh components of chunks retired by the chunk-cut
     *  pass. Called from Tick after DrainPendingApply. No octree lock needed (touches only
     *  the retired chunks' own components). */
    void DrainPendingDestroy();

    /** Game thread: (re)starts the collision bricks for the current density and precision. */
    void InitializeCollision(double RootExtent);

    /** Game thread: forwards finished edits, gathers the collision sources (player pawns, or
     *  the camera) and ticks the collision bricks. */
    void TickCollision(UWorld* World);

    /** Background task: applies a spherical brush edit, reconstructs affected subtrees,
     *  and pushes updated meshes. Runs independently of the Data/Mesh chain. */
    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};
