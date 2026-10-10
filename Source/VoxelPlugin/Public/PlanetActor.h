// PlanetActor.h - Top-level planet actor that orchestrates terrain, ocean, and
// gravity child actors. Owns the shared density compositor (noise + edit store)
// that both terrain and ocean subsystems sample from.
//
// Actor scale determines planet radius (max component, in cm). Child actors
// inherit position and rotation from PlanetRoot but use absolute scale
// (set explicitly per-child based on their respective radii).
//
// Initialization is deferred to Tick via bPendingInitialize to avoid spawning
// child actors during OnConstruction (which is unsafe in UE5).

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AdaptiveVoxelActor.h"
#include "OceanSphereActor.h"
#include "FDensitySampleCompositor.h"
#include "VoxelDensity.h"
#include "VoxelLandSettings.h"
#include "VoxelOceanSettings.h"
#include "PooledActor.h"
#include "PlanetActor.generated.h"

class APlanetGravityZone;
class FVoxelLandBaker;
struct FVoxelLandBake;

/** Top-level planet actor that spawns and coordinates terrain, ocean, and
 *  gravity child actors. The atmosphere is not attached while mesh generation is
 *  reworked; the star position the star system pushes is kept for its return.
 *
 *  Builds a shared FDensitySampleCompositor (noise heightmap + sparse edit store)
 *  that is passed to both the terrain and ocean actors so they sample identical
 *  density and share edit state. Scale determines planet radius; child actors
 *  are scaled independently (terrain = PlanetRadius, ocean = OceanRadius,
 *  gravity = max of the two).
 *
 *  The surface comes from one of two generators (Generator): the noise surface
 *  (SurfaceSettings), or the terrestrial generator (TerrestrialSettings), whose bake is made
 *  before the planet is built (time-sliced, cached on disk): until it is ready the planet
 *  keeps what it showed. A terrestrial planet's radius is its sea level.
 *
 *  Property changes are handled via two deferred flags:
 *    - bPendingInitialize: full rebuild (scale, NAR, generator or surface settings changes)
 *    - bPendingOceanUpdate: ocean/gravity toggle or sea level change only
 *  Both go through ConfigureOcean / ConfigureGravity, as the first initialization does.
 *  The ocean's own settings (OceanSettings) and material apply in place, without either. */
UCLASS()
class VOXELPLUGIN_API APlanetActor : public AActor, public IPooledActor, public IStarLit
{
    GENERATED_BODY()

public:
    APlanetActor();

    /** Root component -- provides the transform gizmo in the editor. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet")
    TObjectPtr<USceneComponent> PlanetRoot;

    // --- Shape ---

    /** Which generator shapes the surface (a change rebuilds the planet). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape")
    EVoxelSurfaceGenerator Generator = EVoxelSurfaceGenerator::Noise;

    /** Noise surface: ratio of noise amplitude to planet radius, shared with child actors.
     *  Terrestrial: the ceiling on the height range (the bake sets the range itself). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.15;

    /** Shape of the noise surface's heightfield, shared by terrain and ocean. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape",
        meta = (EditCondition = "Generator == EVoxelSurfaceGenerator::Noise", EditConditionHides))
    FVoxelSurfaceSettings SurfaceSettings;

    /** The terrestrial generator: plates, landforms and the bake. Heights are metres about sea
     *  level, which is the planet's radius (the actor scale). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape",
        meta = (EditCondition = "Generator == EVoxelSurfaceGenerator::Terrestrial", EditConditionHides))
    FVoxelTerrestrialSettings TerrestrialSettings;

    /** Noise surface: sea level as a fraction of the noise distribution [0, 1].
     *  OceanRadius = PlanetRadius + SeaLevel * NoiseAmplitude (at least 1 cm). A terrestrial
     *  planet's sea level is its radius. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape",
        meta = (ClampMin = "-1", ClampMax = "2.0", EditCondition = "Generator == EVoxelSurfaceGenerator::Noise"))
    double SeaLevel = 0.5;

    // --- Ocean ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Ocean")
    bool bEnableOcean = true;

    /** The ocean's surface material (empty: the plugin's MT_Ocean). Applies without a rebuild. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Ocean")
    UMaterialInterface* OceanMaterial = nullptr;

    /** The ocean's settings: mesh, land cull, underwater view, optics and environment. The
     *  planet forwards them whole to its ocean (SetOceanSettings), so they survive a respawn;
     *  only what changed is applied (the mesh rebuilds, the rest applies in place). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, BlueprintSetter = SetOceanSettings, Category = "Planet|Ocean")
    FVoxelOceanSettings OceanSettings;

    // --- Terrain ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Terrain")
    UMaterialInterface* TerrainMaterial = nullptr;

    // --- Gravity ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Gravity")
    bool bEnableGravity = true;

    // --- Accessors ---

    UFUNCTION(BlueprintCallable, Category = "Planet")
    AAdaptiveVoxelActor* GetTerrainActor() const { return TerrainActor; }

    UFUNCTION(BlueprintCallable, Category = "Planet")
    AOceanSphereActor* GetOceanActor() const { return OceanActor; }

    UFUNCTION(BlueprintCallable, Category = "Planet")
    APlanetGravityZone* GetGravityZone() const { return GravityZone; }

    /** 1 when the planet's surface is ready to build: always for the noise surface; for the
     *  terrestrial generator once the bake for the current settings is made or loaded. */
    UFUNCTION(BlueprintCallable, Category = "Planet")
    float GetBakeProgress() const;

    /** The terrestrial bake the planet is built with (null for the noise surface). */
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> GetLandBake() const { return LandBake; }

    /** Rebuilds the compositor and every child on the next Tick (a child's structural
     *  setting changed, e.g. the terrain's TargetPrecision). */
    void RequestRebuild() { bPendingInitialize = true; }

    /** New ocean settings at run time: stored on the planet and forwarded to its ocean, which
     *  applies only what changed. */
    UFUNCTION(BlueprintCallable, Category = "Planet|Ocean")
    void SetOceanSettings(const FVoxelOceanSettings& InSettings);

    // --- Lifecycle ---

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Destroyed() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override { return true; }
    virtual void Tick(float DeltaTime) override;

    // IPooledActor as a proxy-CARRIED body -- driven only by the wrapping proxy,
    // never registered with the pool manager (so the arg-less OnAcquired() is unused).
    using IPooledActor::OnAcquired;   // un-hide the arg-less overload we don't implement
    virtual void OnAcquired(double WorldRadius) override;
    virtual void OnReturnToPool() override;

    // IStarLit -- the star system pushes its star's world position each frame. Stored
    // for the atmosphere and lighting integration; nothing consumes it yet.
    virtual void SetStarWorldPosition(const FVector& StarWorldPos) override;

    /** Last star position pushed through IStarLit (world space), and whether one arrived. */
    FVector GetStarWorldPosition() const { return StarWorldPosition; }
    bool HasStarWorldPosition() const { return bHasStarWorldPosition; }

    /** Master dormancy toggle: hides + tick-disables this actor and its child actors
     *  (terrain/ocean/gravity) and drops collision. A dormant planet does NO per-frame
     *  CPU or GPU work; the ocean keeps its pages, so waking is instant. Waking respects the
     *  bEnableOcean/Gravity feature flags. */
    void SetDormant(bool bDormant);

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual void PostEditMove(bool bFinished) override;
#endif

private:
    // The child actors. Not exported with a copy or a duplicate of the planet (the copy
    // spawns its own; SpawnChildActors also drops a child another planet owns).

    /** Terrain child actor (AAdaptiveVoxelActor). Scale = PlanetRadius. */
    UPROPERTY(TextExportTransient)
    AAdaptiveVoxelActor* TerrainActor = nullptr;

    /** Ocean child actor (AOceanSphereActor). Scale = OceanRadius. */
    UPROPERTY(TextExportTransient)
    AOceanSphereActor* OceanActor = nullptr;

    /** Gravity zone child actor (APlanetGravityZone). Centered on planet. */
    UPROPERTY(TextExportTransient)
    APlanetGravityZone* GravityZone = nullptr;

    /** Shared compositor (surface density + edit store) passed to both terrain and
     *  ocean actors so they sample identical density. Built during Initialize. */
    TSharedPtr<FDensitySampleCompositor> SharedCompositor;

    // --- Change Detection ---
    // Cached values from last init/update, compared in OnConstruction to determine
    // whether a full rebuild or partial ocean update is needed.
    FVector LastInitScale = FVector::ZeroVector;
    double LastSeaLevel = -1.0;
    double LastNoiseAmplitudeRatio = -1.0;
    uint32 LastSurfaceSettingsHash = 0;
    EVoxelSurfaceGenerator LastGenerator = EVoxelSurfaceGenerator::Noise;
    uint32 LastTerrestrialHash = 0;
    bool bLastEnableOcean = true;
    bool bLastEnableGravity = true;

    bool bInitialized = false;

    /** A slider is still moving (editor): construction doesn't flag rebuilds until release. */
    bool bInteractiveEdit = false;

    FVector StarWorldPosition = FVector::ZeroVector;
    bool bHasStarWorldPosition = false;

    /** When true, Initialize runs on the next Tick. Set by OnConstruction for deferred
     *  child actor spawning (unsafe during construction). */
    bool bPendingInitialize = true;

    /** When true, ocean/gravity toggles and sea level changes are applied on the
     *  next Tick without a full terrain rebuild. */
    bool bPendingOceanUpdate = false;

    /** The terrestrial bake being made for the current settings (null when none is needed). */
    TSharedPtr<FVoxelLandBaker> LandBaker;

    /** The terrestrial bake the planet is built with: kept while a new one is made, so the
     *  planet shows the old surface until then. */
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> LandBake;

    /** Terrestrial: advances the bake for the current settings and radius; true once LandBake
     *  is it. */
    bool TickLandBake();

    /** The density's shape for the current generator: the radius the heights are above, their
     *  range, and the sea radius. Terrestrial needs LandBake. */
    void ComputeShape(double& OutPlanetRadius, double& OutNoiseAmplitude, double& OutOceanRadius) const;

    /** NoiseAmplitude / PlanetRadius for the children (NoiseAmplitudeRatio itself for the noise
     *  surface, so its lattice is unchanged). */
    double ChildNoiseAmplitudeRatio(double PlanetRadius, double NoiseAmplitude) const;

    /** Full planet initialization: spawns child actors, builds the shared compositor,
     *  configures and initializes terrain/ocean/gravity. */
    void Initialize();

    /** The ocean from the planet's settings: radius, density, material and OceanSettings,
     *  shown and ticking, or hidden and stopped when bEnableOcean is off. Used by the first
     *  initialization and by every later update alike. */
    void ConfigureOcean(double PlanetRadius, double NoiseAmplitude, double OceanRadius);

    /** The gravity zone at the visible surface (the larger of the ocean and planet radii),
     *  or hidden and without collision when bEnableGravity is off. */
    void ConfigureGravity(double PlanetRadius, double OceanRadius);

    /** Records the values FlagPendingWork compares against. */
    void RecordAppliedState();

    /** Compares the shape, sea level and toggles against what was applied and flags the
     *  rebuild or the ocean update for the next Tick. From OnConstruction, and from
     *  PostEditChangeProperty (a PIE world doesn't rerun construction after an edit). */
    void FlagPendingWork();

    /** Forwards the materials and OceanSettings to the children again; each applies only what
     *  changed (after an undo, which reports no property). */
    void ForwardChildSettings();

    /** Spawns terrain, ocean, and gravity child actors if not already present.
     *  Attaches them to PlanetRoot with absolute scale. Loads default materials. */
    void SpawnChildActors();

    /** Destroys all child actors and nulls the pointers. */
    void DestroyChildActors();

    /** Builds the shared compositor (VoxelDensity::MakeCompositor) with a fresh
     *  FSparseEditStore covering the terrain lattice's root box down to MaxDepth. */
    TSharedPtr<FDensitySampleCompositor> BuildCompositor(double PlanetRadius, double NoiseAmplitude, double RootExtent, int32 MaxDepth);

    /** Computes ocean radius from planet radius, noise amplitude, and SeaLevel.
     *  OceanRadius = PlanetRadius + SeaLevel * NoiseAmplitude, at least 1 cm (a sea level
     *  below the planet's centre gives a vanishing ocean, not a negative scale). */
    double ComputeOceanRadius(double PlanetRadius, double NoiseAmplitude) const;
};
