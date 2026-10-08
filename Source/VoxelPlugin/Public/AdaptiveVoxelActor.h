// AdaptiveVoxelActor.h — Terrain actor (standalone or driven by APlanetActor) owning an adaptive
// octree and its async LOD/mesh/edit pipeline. Scale (max component, cm) is the planet radius;
// scale changes rebuild, position and rotation are live.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "MaterialDomain.h"
#include "FAdaptiveOctree.h"
#include "VoxelDensity.h"
#include "VoxelCollisionBricks.h"
#include "VoxelBrickTerrain.h"
#include "RealtimeMeshActor.h"
#include "AdaptiveVoxelActor.generated.h"

/** Three task chains, each under OctreeLock: DataUpdate (LOD split/merge, chunk cut) chained
 *  with MeshUpdate (collects dirty chunks) at a minimum interval, and EditUpdate when
 *  triggered. Tick applies finished meshes a whole pass at a time. Collision
 *  (FVoxelCollisionBricks) and the brick terrain run on their own tasks without the lock. */
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

    /** Collision bricks and brick-terrain components attach to this one, placed like
     *  MeshAttachmentRoot. Kept apart so hiding the octree's chunks leaves them alone. */
    UPROPERTY()
    TObjectPtr<USceneComponent> BrickAttachmentRoot;

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

    /** Deepest a chunk root may sit (<= MaxDepth): chunks under the camera promote toward it
     *  until their float jitter is sub-pixel (a 10,000 km planet wants ~12-14 for sub-mm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "2"))
    int MaxChunkDepth = 14;

    /** Chunk float jitter tolerance (fraction of the view) at the camera's near distance;
     *  smaller promotes sooner. ~5e-4 is about half a pixel at 1080p. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "0.00001"))
    double ChunkPrecisionThreshold = 5e-4;

    /** Demote hysteresis: a promoted chunk collapses once its parent is precise to
     *  ChunkPrecisionThreshold * this. Lower = less churn, finer chunks held longer. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree", meta = (ClampMin = "0.05", ClampMax = "1.0"))
    double ChunkDemoteHysteresis = 0.5;

    /** Largest depth gap between face-adjacent chunks (forced refines, refused demotes).
     *  1 = strict 2:1 (safest); 2 = fewer components, usually still crack-free. */
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

    /** Brick terrain: a brick splits within this many brick sizes of the camera. Detail goes
     *  with this times BrickCellsPerBrick; geomorphing needs it above about 2 (else level
     *  changes step), so lower BrickCellsPerBrick for less detail. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Bricks", meta = (ClampMin = "0.5", UIMin = "2.0", UIMax = "6.0"))
    double BrickLodDistanceFactor = 2.5;

    /** Brick terrain: cells along a brick edge, a power of two. 16 at factor 2.5 has about
     *  the detail of 32 at 1.25, and geomorphs. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Bricks", meta = (ClampMin = "8", ClampMax = "64"))
    int32 BrickCellsPerBrick = 16;

    /** Minimum subdivision depth maintained regardless of camera distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree")
    int MinDepth = 5;

    /** Target voxel spacing in world units (cm). MaxDepth is computed automatically
     *  so the finest LOD voxel cells are approximately this size. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 150.0;

    /** From TargetPrecision and the planet radius at init, in [MinDepth, MaxKeyDepth] (22 at
     *  the default scale). */
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

    /** Per-frame budget (ms) for creating chunk components, oldest work first. Mesh swaps are
     *  not budgeted: chunks meshed together appear in the same frame. */
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

    /** The density this terrain samples (null before initialization). */
    TSharedPtr<FDensitySampleCompositor> GetDensityCompositor() const { return DensityCompositor; }

    /** The camera position last seen by Tick, in actor space without scale. */
    FVector GetCameraPositionLocal() const { return CameraPosition; }

    /** The brick terrain (exists once initialized; holds bricks while voxel.Render.Bricks is on). */
    TSharedPtr<FVoxelBrickTerrain> GetBrickTerrain() const { return BrickTerrain; }

    /** Called by APlanetActor: uses its compositor (shared density and edits), sets the scale
     *  (planet radius) first, and re-parents MeshAttachmentRoot to InAttachParent if given. */
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

    /** Terrain rendered from bricks while voxel.Render.Bricks is 1 (the octree's chunks are
     *  hidden and its LOD passes paused meanwhile). */
    TSharedPtr<FVoxelBrickTerrain> BrickTerrain;
    bool bBrickRenderActive = false;

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

    /** Destroys all RealtimeMesh components attached to MeshAttachmentRoot and BrickAttachmentRoot. */
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

    /** Game thread, every Tick. Creates missing components within ChunkApplyBudgetMs, then
     *  applies initial-build chunks at once and later work in groups of overlapping pass
     *  ranges, a group in one frame once its components exist (the oldest waiting group holds
     *  newer ones back for a bounded time). No lock: streams are immutable. */
    void DrainPendingApply();

    /** Game-thread: destroys the RealtimeMesh components of chunks retired by the chunk-cut
     *  pass. Called from Tick after DrainPendingApply. No octree lock needed (touches only
     *  the retired chunks' own components). */
    void DrainPendingDestroy();

    /** Game thread: (re)starts the collision bricks for the current density and precision. */
    void InitializeCollision(double RootExtent);

    /** Game thread: (re)starts the brick terrain for the current density and precision. */
    void InitializeBrickTerrain(double RootExtent);

    /** Game thread: switches rendering between the octree and the brick terrain. */
    void SetBrickRender(bool bOn);

    /** Game thread: forwards finished edits, gathers the collision sources (player pawns, or
     *  the camera) and ticks the collision bricks. */
    void TickCollision(UWorld* World);

    /** Diagnostics (voxel.Debug.Collision): logs what the edit trace hit. */
    void LogEditHit(const FHitResult& Hit, const FVector& LocalHit) const;

    /** Background task: applies a spherical brush edit, reconstructs affected subtrees,
     *  and pushes updated meshes. Runs independently of the Data/Mesh chain. */
    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};
