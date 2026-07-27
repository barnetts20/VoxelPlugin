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
#include "FastNoise/FastNoise.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "MaterialDomain.h"
#include "FAdaptiveOctree.h"
#include "RealtimeMeshActor.h"
#include "AdaptiveVoxelActor.generated.h"

/** Terrain actor that owns an adaptive octree and drives its async update pipeline.
 *
 *  Three async task chains run on background threads, gated by atomic flags:
 *    - DataUpdate: LOD split/merge pass (reads camera, writes octree)
 *    - MeshUpdate: pushes dirty chunks to RealtimeMesh components (reads octree)
 *    - EditUpdate: applies brush edits and rebuilds affected chunks (writes octree)
 *
 *  DataUpdate and MeshUpdate chain continuously with a configurable minimum interval.
 *  EditUpdate runs independently when triggered. All three acquire OctreeLock. */
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

    /** FastNoise node tree for terrain heightmap generation. Configured once in Initialize
     *  and captured by value into the heightmap sample layer lambda. */
    FastNoise::SmartNode<> Noise;

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
    int MaxChunkDepth = 10;

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

    /** Master switch for distance-gated surface collision. When off, no chunk cooks a
     *  collision body (pure render performance). When on, only chunks within the
     *  speed-scaled lead distance cook + enable query collision. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision")
    bool bEnableSurfaceCollision = true;

    /** Collision shell kept underfoot regardless of speed (cm). Sized for walking/hovering.
     *  ~75 m by default. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "0"))
    double WalkBaseCollisionDistance = 7500.0;

    /** Seconds of lead the collision shell extends ahead per unit speed: CollisionDistance =
     *  WalkBase + Speed * this. Covers the worst-case cook latency (data-pass interval +
     *  async cook + a frame or two) so a chunk is queryable before you reach it at speed.
     *  At 5 km/s this adds ~1.5 km of lead. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "0"))
    double CollisionCookLeadTime = 0.3;

    /** Hysteresis on the collision distance: collision turns on at CollisionDistance, off at
     *  this multiple of it, so a chunk sitting on the boundary doesn't cook/uncook. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Collision", meta = (ClampMin = "1.0", ClampMax = "3.0"))
    double CollisionHysteresis = 1.3;

    /** Minimum subdivision depth maintained regardless of camera distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree")
    int MinDepth = 4;

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

    /** Depth beyond which noise sampling is replaced by trilinear interpolation
     *  from parent corner densities. Noise loses float precision past this depth,
     *  but deeper splits still provide geometric detail for editing. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree",
        meta = (ClampMin = "1"))
    int32 PrecisionDepthFloor = 20;

    // --- LOD ---

    /** Screen-space size threshold for LOD split/merge decisions. Smaller values
     *  produce finer detail at greater distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double ScreenSpaceThreshold = 0.075;

    /** Minimum interval (seconds) between LOD update passes. Prevents the
     *  DataUpdate→MeshUpdate chain from saturating the thread pool. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double MinDataUpdateInterval = 0.05;

    /** LOD heartbeat interval (seconds) while in cheap mode. Passes are near-no-op
     *  once the octree has settled at MinDepth, so we run them less often. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double CheapDataUpdateInterval = 0.25;

    /** Multiplier on camera velocity for predictive LOD. Higher values split nodes
     *  earlier in the direction of camera movement. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double VelocityLookAheadFactor = 2.0;

    /** Per-frame game-thread time budget (milliseconds) for draining pending chunk
     *  applies in Tick. First-touch component creation (RegisterComponent + section-group
     *  setup) is the expensive game-thread cost; draining a fixed slice per frame spreads
     *  the first-build population across frames without a hitch, while adapting to the
     *  actual per-chunk cost (unlike a fixed count). The drain runs every frame (60/s),
     *  not on the slower data/mesh loop cadence, so a modest budget still populates quickly.
     *  Chunks not reached this frame stay queued for the next. Tune against the
     *  particle-fade budget and your frame-time headroom. */
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

    /** Cheap mode: while active, the LOD pass suppresses screen-space splits and forces
     *  merges, so the octree settles at MinDepth and holds there regardless of
     *  camera/planet motion (used during parallax travel at SpeedScale != 1). Set by
     *  APlanetActor. On the true->false transition the LOD pass is kicked immediately
     *  so full detail resumes without waiting on the slow heartbeat. */
    void SetCheapMode(bool bInCheap);

protected:
    // --- Async Task Flags ---
    // Each flag gates its corresponding task so only one instance runs at a time.

    std::atomic<bool> DataUpdateIsRunning = false;
    std::atomic<bool> MeshUpdateIsRunning = false;
    std::atomic<bool> EditUpdateIsRunning = false;

    /** When true, the LOD pass runs in cheap mode (settle at MinDepth, no further
     *  split/merge work). Read at the start of each DataUpdate pass. */
    std::atomic<bool> bCheapMode = false;

    /** Timer handle for the MinDataUpdateInterval delay between MeshUpdate completion
     *  and the next DataUpdate kickoff. */
    FTimerHandle DataUpdateTimerHandle;

    /** Game-thread apply queue. The mesh/edit tasks enqueue dirty chunks here (deduped
     *  via FMeshChunk::bApplyQueued); Tick drains a per-frame time slice
     *  (ChunkApplyBudgetMs) from it and applies each to its RealtimeMesh component. This
     *  decouples the (expensive) component creation from the data/mesh loop cadence so it
     *  runs at frame rate. Guarded by PendingApplyCS. */
    TArray<TSharedPtr<FMeshChunk>> PendingApply;
    FCriticalSection PendingApplyCS;

    /** Chunks retired by the chunk-cut pass (coarse parents replaced by finer children).
     *  Their RealtimeMesh components must be destroyed on the game thread; Tick drains this
     *  after DrainPendingApply so the finer children get a chance to appear first. Guarded
     *  by PendingApplyCS (same low-contention queue family as PendingApply). */
    TArray<TSharedPtr<FMeshChunk>> PendingDestroy;

    /** Chunks whose desired collision state changed (set by the collision pass on a worker).
     *  Drained on the game thread to flip SetCollisionEnabled / re-cook. Guarded by
     *  PendingApplyCS. */
    TArray<TSharedPtr<FMeshChunk>> PendingCollision;

    /** Smoothed player speed (cm/s) from camera-position deltas, updated in Tick. Drives the
     *  speed-scaled collision lead distance. */
    double PlayerSpeed = 0.0;
    FVector PrevSpeedSamplePos = FVector::ZeroVector;
    bool bHasSpeedSample = false;

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

    /** Game-thread: applies queued chunks to their RealtimeMesh components until the
     *  per-frame ChunkApplyBudgetMs is spent, leaving the rest for next frame. Called from
     *  Tick. Holds the octree read lock while applying so a concurrent data pass can't
     *  rewrite a chunk's stream mid-copy. */
    void DrainPendingApply();

    /** Game-thread: destroys the RealtimeMesh components of chunks retired by the chunk-cut
     *  pass. Called from Tick after DrainPendingApply. No octree lock needed (touches only
     *  the retired chunks' own components). */
    void DrainPendingDestroy();

    /** Game-thread: flips SetCollisionEnabled / re-cooks for chunks whose desired collision
     *  changed. Called from Tick. No octree lock needed. */
    void DrainPendingCollision();

    /** Background task: applies a spherical brush edit, reconstructs affected subtrees,
     *  and pushes updated meshes. Runs independently of the Data/Mesh chain. */
    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};