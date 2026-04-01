// PlanetActor.h — Top-level planet actor that orchestrates terrain, ocean, and
// atmosphere child actors. Owns the shared density compositor (noise + edit store)
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
#include "FastNoise/FastNoise.h"
#include "PlanetActor.generated.h"

class APlanetAtmosphereActor;
class APlanetGravityZone;

/** Top-level planet actor that spawns and coordinates terrain, ocean, and
 *  atmosphere child actors.
 *
 *  Builds a shared FDensitySampleCompositor (noise heightmap + sparse edit store)
 *  that is passed to both the terrain and ocean actors so they sample identical
 *  density and share edit state. Scale determines planet radius; child actors
 *  are scaled independently (terrain = PlanetRadius, ocean = OceanRadius,
 *  atmosphere = max of the two).
 *
 *  Property changes are handled via two deferred flags:
 *    - bPendingInitialize: full rebuild (scale, NAR changes)
 *    - bPendingOceanUpdate: ocean/atmosphere toggle or sea level change only */
UCLASS()
class VOXELPLUGIN_API APlanetActor : public AActor
{
    GENERATED_BODY()

public:
    APlanetActor();

    /** Root component -- provides the transform gizmo in the editor. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet")
    TObjectPtr<USceneComponent> PlanetRoot;

    // --- Shape ---

    /** Ratio of noise amplitude to planet radius, shared with child actors. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.15;

    /** Sea level as a fraction of the noise distribution [0, 1].
     *  OceanRadius = PlanetRadius + SeaLevel * NoiseAmplitude. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape", meta = (ClampMin = "-1", ClampMax = "2.0"))
    double SeaLevel = 0.5;

    // --- Ocean ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Ocean")
    bool bEnableOcean = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Ocean")
    UMaterialInterface* OceanMaterial = nullptr;

    // --- Terrain ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Terrain")
    UMaterialInterface* TerrainMaterial = nullptr;

    // --- Atmosphere ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Atmosphere")
    bool bEnableAtmosphere = true;

    // --- Gravity ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Gravity")
    bool bEnableGravity = true;

    // --- Accessors ---

    UFUNCTION(BlueprintCallable, Category = "Planet")
    AAdaptiveVoxelActor* GetTerrainActor() const { return TerrainActor; }

    UFUNCTION(BlueprintCallable, Category = "Planet")
    AOceanSphereActor* GetOceanActor() const { return OceanActor; }

    UFUNCTION(BlueprintCallable, Category = "Planet")
    APlanetAtmosphereActor* GetAtmosphereActor() const { return AtmosphereActor; }

    UFUNCTION(BlueprintCallable, Category = "Planet")
    APlanetGravityZone* GetGravityZone() const { return GravityZone; }

    // --- Lifecycle ---

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override { return true; }
    virtual void Tick(float DeltaTime) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

private:
    /** Terrain child actor (AAdaptiveVoxelActor). Scale = PlanetRadius. */
    UPROPERTY()
    AAdaptiveVoxelActor* TerrainActor = nullptr;

    /** Ocean child actor (AOceanSphereActor). Scale = OceanRadius. */
    UPROPERTY()
    AOceanSphereActor* OceanActor = nullptr;

    /** Atmosphere child actor (APlanetAtmosphereActor). Scale = max(Ocean, Planet). */
    UPROPERTY()
    APlanetAtmosphereActor* AtmosphereActor = nullptr;

    /** Gravity zone child actor (APlanetGravityZone). Centered on planet. */
    UPROPERTY()
    APlanetGravityZone* GravityZone = nullptr;

    /** FastNoise node tree for the shared heightmap. Configured once in Initialize,
     *  captured by value into the compositor's heightmap layer lambda. */
    FastNoise::SmartNode<> Noise;

    /** Shared compositor (noise heightmap + edit store) passed to both terrain and
     *  ocean actors so they sample identical density. Built during Initialize. */
    TSharedPtr<FDensitySampleCompositor> SharedCompositor;

    // --- Change Detection ---
    // Cached values from last init/update, compared in OnConstruction to determine
    // whether a full rebuild or partial ocean update is needed.
    FVector LastInitScale = FVector::ZeroVector;
    double LastSeaLevel = -1.0;
    double LastNoiseAmplitudeRatio = -1.0;
    bool bLastEnableOcean = true;
    bool bLastEnableAtmosphere = true;
    bool bLastEnableGravity = true;

    bool bInitialized = false;

    /** When true, Initialize runs on the next Tick. Set by OnConstruction for deferred
     *  child actor spawning (unsafe during construction). */
    bool bPendingInitialize = true;

    /** When true, ocean/atmosphere toggles and sea level changes are applied on the
     *  next Tick without a full terrain rebuild. */
    bool bPendingOceanUpdate = false;

    /** Full planet initialization: spawns child actors, builds the shared compositor,
     *  configures and initializes terrain/ocean/atmosphere. */
    void Initialize();

    /** Spawns terrain, ocean, and atmosphere child actors if not already present.
     *  Attaches them to PlanetRoot with absolute scale. Loads default materials. */
    void SpawnChildActors();

    /** Destroys all child actors and nulls the pointers. */
    void DestroyChildActors();

    /** Builds a shared FDensitySampleCompositor with a heightmap noise layer and
     *  a fresh FSparseEditStore. The heightmap lambda projects positions onto the
     *  sphere surface, samples noise, and computes SDF density. */
    TSharedPtr<FDensitySampleCompositor> BuildCompositor(double PlanetRadius, double NoiseAmplitude, double RootExtent, int32 ChunkDepth, int32 MaxDepth);

    /** Computes ocean radius from planet radius, noise amplitude, and SeaLevel.
     *  OceanRadius = PlanetRadius + SeaLevel * NoiseAmplitude. */
    double ComputeOceanRadius(double PlanetRadius, double NoiseAmplitude) const;
};