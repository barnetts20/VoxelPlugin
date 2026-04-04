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

    /** Computed at init from float precision requirements. Determines the depth at
     *  which the tree is spatially partitioned into mesh chunks.
     *  At default scale (100M radius, 0.15 NAR): ChunkDepth=4. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Octree")
    int ChunkDepth = 4;

    /** Minimum subdivision depth maintained regardless of camera distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree")
    int MinDepth = 7;

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

    /** Multiplier on camera velocity for predictive LOD. Higher values split nodes
     *  earlier in the direction of camera movement. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double VelocityLookAheadFactor = 2.0;

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

    /** Destroys all RealtimeMesh components attached to MeshAttachmentRoot. */
    void CleanSceneRoot();

    /** Background task: builds the octree (if PendingParams set), then runs one LOD
     *  update pass. Chains to RunMeshUpdateTask on completion. */
    void RunDataUpdateTask();

    /** Background task: pushes all dirty chunks to their RealtimeMesh components.
     *  Chains back to RunDataUpdateTask after MinDataUpdateInterval. */
    void RunMeshUpdateTask();

    /** Background task: applies a spherical brush edit, reconstructs affected subtrees,
     *  and pushes updated meshes. Runs independently of the Data/Mesh chain. */
    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};