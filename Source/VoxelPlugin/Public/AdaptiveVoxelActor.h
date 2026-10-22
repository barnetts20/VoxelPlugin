// AdaptiveVoxelActor.h - Terrain actor (standalone or driven by APlanetActor): renders the
// planet from LOD bricks (FVoxelBrickTerrain), keeps collision bricks around the pawns
// (FVoxelCollisionBricks) and applies edits to the shared edit store. Scale (max component,
// cm) is the planet radius; scale changes rebuild, position and rotation are live.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "Materials/MaterialParameterCollection.h"
#include "MaterialDomain.h"
#include "VoxelDensity.h"
#include "VoxelCollisionBricks.h"
#include "VoxelBrickTerrain.h"
#include "RealtimeMeshActor.h"
#include "VoxelLandCache.h"
#include "FSparseEditStore.h"
#include <atomic>
#include "AdaptiveVoxelActor.generated.h"

class UMaterialInstanceDynamic;
class UTexture2DDynamic;
class UTextureRenderTarget2D;

/** An edit written to the edit store, as the terrain forwards it to its bricks: its sequence,
 *  centre (actor space without scale) and radius (cm). */
DECLARE_MULTICAST_DELEGATE_ThreeParams(FOnVoxelTerrainEdited, uint32 /*Sequence*/, const FVector& /*Center*/, double /*Radius*/);

/** The brick terrain and the collision bricks run their own tasks; edits are written to the
 *  edit store on a task and forwarded to both from Tick. */
UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

private:
    /** Collision bricks and brick-terrain components attach to this one. It inherits the
     *  actor's position and rotation but uses absolute scale (1,1,1): the terrain is built at
     *  world scale, and scale changes rebuild it (OnConstruction). */
    UPROPERTY()
    TObjectPtr<USceneComponent> BrickAttachmentRoot;

    /** Camera position in actor-local space, updated each tick from the viewport. */
    FVector CameraPosition = FVector::ZeroVector;

    bool bTickInEditor = true;

    /** Set to true after Initialize completes param setup. Gates Tick and ShouldTickIfViewportsOnly. */
    std::atomic<bool> Initialized = false;

    /** Set in BeginDestroy to prevent async tasks from touching a dying actor. */
    std::atomic<bool> IsDestroyed = false;

    /** Last scale at which Initialize ran - used to detect scale changes in OnConstruction. */
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
     *  read-only on this actor - edit them on the planet instead. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain")
    bool bIsPlanetOwned = false;

    /** Surface material of the terrain bricks. When standalone, auto-loads the plugin's
     *  triplanar material on first init. When planet-owned, set by the planet. Change it at
     *  run time with SetSurfaceMaterial. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, BlueprintSetter = SetSurfaceMaterial, Category = "Terrain|Materials",
        meta = (EditCondition = "!bIsPlanetOwned"))
    UMaterialInterface* SurfaceMaterial = nullptr;

    /** Terrestrial planets, materials with the land node (VoxelLandMaterial.ush): how far the
     *  rendered surface may lie off the analytic one (m) before the node's normal fades to the
     *  mesh's (edits, overhangs). It also grows by 16 pixel footprints, for the coarse mesh far
     *  off. A mesh surface facing well away from the land's takes the mesh's normal whatever
     *  this is (VoxelLandFacingToMesh). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials", meta = (ClampMin = "0.0", UIMax = "50"))
    double LandSurfaceTolerance = 0.33;

    /** Terrestrial planets, the land node: close to the camera, where the finest mesh cells
     *  (ActualPrecision) carry all the land's detail, the node uses the mesh's normal. Below a
     *  pixel footprint of this fraction of the finest cell it is the mesh's, above 2.5 times it
     *  the node's (0 never). At the default 0.05 and 1 m cells, looking down at 1080p and 90
     *  degrees: the mesh's within about 50 m, the node's beyond about 120 m (nearer at grazing
     *  angles, where pixels stretch). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials", meta = (ClampMin = "0.0", UIMax = "0.5"))
    double LandNearMeshNormal = 0.05;

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
     *  changes step), so lower BrickCellsPerBrick for less detail. Below about 2.04 every
     *  terrain task also balances its tree, which costs time. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Bricks", meta = (ClampMin = "0.5", UIMin = "2.0", UIMax = "6.0"))
    double BrickLodDistanceFactor = 2.5;

    /** Brick terrain: cells along a brick edge, a power of two. 16 at factor 2.5 has about
     *  the detail of 32 at 1.25, and geomorphs. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Bricks", meta = (ClampMin = "8", ClampMax = "64"))
    int32 BrickCellsPerBrick = 16;

    /** Brick terrain: the collection the terrain material's morph node reads (vectors
     *  VoxelMorphCameraHigh, VoxelMorphCameraLow, VoxelMorphParams; Design/MeshingEngine.md
     *  section 7). Empty: /VoxelPlugin/MPC_VoxelMorph if it exists. Without one LOD changes
     *  don't blend. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Bricks")
    TObjectPtr<UMaterialParameterCollection> BrickMorphParameters = nullptr;

    /** Target cell size of the finest terrain lattice level in world units (cm). MaxDepth is
     *  computed so the finest cells are about this size. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Lattice",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 150.0;

    /** Halvings of the root box down to the finest lattice level, from TargetPrecision and the
     *  planet radius at init (21 at the default scale). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Lattice")
    int MaxDepth = 21;

    /** The finest cell size (cm) MaxDepth gives. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Lattice")
    double ActualPrecision = 0.0;

    /** Deepest lattice: the bricks address points by 32-bit indices in half finest cells
     *  from the root centre. */
    static constexpr int32 MaxLatticeDepth = 30;

    /** MaxDepth for a root half-extent and target cell size, in [6, MaxLatticeDepth] (the
     *  coarsest brick level must exist). */
    static int32 MaxDepthFor(double RootExtent, double InTargetPrecision);

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

    /** The density this terrain samples (null before initialization). */
    TSharedPtr<FDensitySampleCompositor> GetDensityCompositor() const { return DensityCompositor; }

    /** The camera position last seen by Tick, in actor space without scale. */
    FVector GetCameraPositionLocal() const { return CameraPosition; }

    /** The brick terrain (exists once initialized). */
    TSharedPtr<FVoxelBrickTerrain> GetBrickTerrain() const { return BrickTerrain; }

    /** The land surface cache (Design/LandSurfaceCache.md) once it's made (the ocean's depths read
     *  it while it's written for the camera, FVoxelLandCache::IsLive; stage O9m); null otherwise.
     *  Game thread. */
    const FVoxelLandCache* GetLandCache() const { return LandCache.IsValid() && LandCache->IsInitialized() ? LandCache.Get() : nullptr; }

    /** Game thread. Broadcast from Tick for every edit written to the edit store, as it is
     *  forwarded to the collision and brick terrain (the ocean rewrites its depth there):
     *  within twice Radius of Center the surface may have moved. */
    FOnVoxelTerrainEdited& OnTerrainEdited() { return TerrainEdited; }

    /** Called by APlanetActor: uses its compositor (shared density and edits), sets the scale
     *  (planet radius) first, and re-parents BrickAttachmentRoot to InAttachParent if given. */
    void InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
        USceneComponent* InAttachParent = nullptr,
        FVector InScale = FVector::ZeroVector);

    /** Rebuilds the density, collision and brick terrain from the current property values.
     *  Called by OnConstruction (scale change), PostEditChangeProperty (structural param change)
     *  and BeginPlay; InitializeFromPlanet does the same with the planet's density. */
    void Initialize();

    /** Changes the terrain material on every brick page without a rebuild (null: the
     *  plugin's triplanar material). */
    UFUNCTION(BlueprintCallable, Category = "Terrain|Materials")
    void SetSurfaceMaterial(UMaterialInterface* InMaterial);

    /** The sea radius the elevation debug view (voxel.Debug.LandView 1) measures heights from
     *  (cm, actor space; 0: the density's own). The planet sets its ocean radius. */
    void SetDebugSeaRadius(double InSeaRadius);

protected:
    /** SetDebugSeaRadius's value, for the brick terrain's settings. */
    double DebugSeaRadius = 0.0;

    /** The density every terrain consumer samples (set by Initialize / InitializeFromPlanet). */
    TSharedPtr<FDensitySampleCompositor> DensityCompositor;

    /** Collision around the pawns, built from DensityCompositor. */
    TSharedPtr<FVoxelCollisionBricks> CollisionBricks;

    /** The rendered terrain. */
    TSharedPtr<FVoxelBrickTerrain> BrickTerrain;

    /** Listeners to the edits (OnTerrainEdited). */
    FOnVoxelTerrainEdited TerrainEdited;

    /** An edit written to the edit store, for the collision bricks and the brick terrain:
     *  where the surface may have moved (actor space). */
    struct FEditNotice
    {
        uint32 Sequence;
        FBox Bounds;
    };

    /** State shared with the edit task, which holds a reference instead of the actor so it
     *  can finish after the actor is destroyed. */
    struct FEditQueue
    {
        /** An edit is being written to the edit store (one at a time). */
        std::atomic<bool> bRunning = false;
        /** Finished edits, drained in Tick. Guarded by CS. */
        TArray<FEditNotice> Notices;
        FCriticalSection CS;
    };
    TSharedRef<FEditQueue> EditQueue = MakeShared<FEditQueue>();

    /** Smoothed camera speed (cm/s) from position deltas, updated in Tick. Feeds the
     *  collision lead distance when a pawn reports no velocity. */
    double PlayerSpeed = 0.0;
    FVector PrevSpeedSamplePos = FVector::ZeroVector;
    bool bHasSpeedSample = false;

    /** Destroys all RealtimeMesh components attached to BrickAttachmentRoot. */
    void CleanSceneRoot();

    /** Game thread: waits for the edit task, drops queued notices and components, then builds
     *  the collision and brick terrain for Compositor at the actor's current scale. */
    void Rebuild(TSharedPtr<FDensitySampleCompositor> Compositor);

    /** Game thread: (re)starts the collision bricks for the current density and precision. */
    void InitializeCollision(double RootExtent);

    /** Game thread: (re)starts the brick terrain for the current density and precision. */
    void InitializeBrickTerrain(double RootExtent);

    /** Game thread: forwards finished edits, gathers the collision sources (player pawns, or
     *  the camera) and ticks the collision bricks. */
    void TickCollision(UWorld* World);

    /** Diagnostics (voxel.Debug.Collision): logs what the edit trace hit. */
    void LogEditHit(const FHitResult& Hit, const FVector& LocalHit) const;

    /** The plugin's triplanar material, or the engine default when the plugin content is
     *  missing. */
    static UMaterialInterface* DefaultSurfaceMaterial();

    /** Game thread. The material the bricks use: a dynamic instance of SurfaceMaterial, its
     *  land node (VoxelLandMaterial.ush) bound to the land textures on a terrestrial planet
     *  (bLandMaterialBound) and off otherwise. */
    UMaterialInterface* ResolveBrickMaterial();

    /** Drops the land textures (and the instance's references to them). */
    void ReleaseLandTextures();

    /** Game thread, per frame after every actor's tick: the planet's frame for the land node;
     *  bTickCache (from the end of the actors' tick only) moves the land surface cache too. */
    void UpdateLandMaterial(bool bTickCache = false);
    void OnWorldPostActorTick(UWorld* World, ELevelTick TickType, float DeltaSeconds);

    /** The land material (VoxelLandMaterial.h): the instance, the textures, and what they were
     *  made from. */
    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> LandMaterialInstance;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DDynamic> LandHeightTexture;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DDynamic> LandFieldTexture;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DDynamic> LandConstantTexture;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DDynamic> LandNoiseTexture;

    UPROPERTY(Transient)
    TObjectPtr<UMaterialInterface> LandMaterialParent;

    TWeakPtr<const FVoxelLandBake, ESPMode::ThreadSafe> LandTexturesBake;   // the height and field textures'
    TWeakPtr<const FVoxelDensityCPU> LandTexturesDensity;                   // the constants'
    TWeakPtr<const FVoxelDensityCPU> LandTexturesFailed;                    // the textures couldn't be made for it
    /** The instance's land node is bound to the textures. */
    bool bLandMaterialBound = false;
    FDelegateHandle LandMaterialTickHandle;

    /** The land surface cache (Design/LandSurfaceCache.md): its textures and the levels' plan,
     *  made with the land textures, ticked with the frame. */
    UPROPERTY(Transient)
    TObjectPtr<UTextureRenderTarget2D> LandCacheHeight;

    UPROPERTY(Transient)
    TObjectPtr<UTextureRenderTarget2D> LandCacheSlope;

    UPROPERTY(Transient)
    TObjectPtr<UTextureRenderTarget2D> LandCacheFields;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DDynamic> LandCacheState;

    TUniquePtr<FVoxelLandCache> LandCache;
    TWeakPtr<const FVoxelDensityCPU> LandCacheDensity;   // the density the cache was made for
    TWeakPtr<const FVoxelDensityCPU> LandCacheFailed;    // the cache couldn't be made for it

    /** Makes the cache for Density (or keeps it), and binds it; false when it can't be made. */
    bool ResolveLandCache(const TSharedPtr<const FVoxelDensityCPU>& Density);
    void ReleaseLandCache();
    /** The camera the land surface cache follows, in the planet's frame (cm, no scale). */
    bool GetLandCacheCamera(const FTransform& PlanetFrame, FVector& OutCameraPlanet) const;

    /** Example edit flow (E carves, Q fills), not in shipping builds: traces from the camera
     *  along the control rotation and edits this terrain where the trace hits one of its
     *  collision bricks. Held: one capsule stretched from there to a point kept at that
     *  distance along the view until released (voxel.Debug.EditStretch 1, the default), or a
     *  stroke of segments along the surface the trace hits (0). */
    void TickDebugEdits(UWorld* World);

    /** The debug brush's stroke: the mode held since the last edit (-1: none). */
    int32 DebugStrokeMode = -1;
    /** The stroke stretches one capsule; its end is DebugStretchDistance (world cm) along the view. */
    bool bDebugStretching = false;
    double DebugStretchDistance = 0.0;
    /** A stretched stroke released whose end (EndStroke) hasn't run yet. */
    bool bDebugStrokeEndPending = false;

    /** Background task: writes an edit (a sphere at InEditCenter, or the stroke from the
     *  previous one when bContinueStroke; stretched to InEditCenter when bStretch) to the edit
     *  store and queues its notice. */
    void RunEditUpdateTask(EVoxelEditMode Mode, FVector InEditCenter, double InEditRadius, double InEditBlend, bool bContinueStroke, bool bStretch = false);

    /** Background task: ends the store's stroke (FSparseEditStore::EndStroke). */
    void RunEndStrokeTask();
};
