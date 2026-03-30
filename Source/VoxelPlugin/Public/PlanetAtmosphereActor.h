// PlanetAtmosphereActor.h — Manages a post-process volume with 3 blendable
// material instances and a directional light to render a volumetric atmosphere
// and cloud layer around a planet.
//
// When owned by APlanetActor, the actor's scale is set externally to
// max(OceanRadius, PlanetRadius) — the visible surface floor. The actor
// reads its own scale as PlanetRadius for material parameters.
//
// Actor Location  → Planet Center / Atmosphere Center
// Actor Scale max → Planet Radius
// Actor Rotation  → Light Direction + directional light rotation

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Engine/VolumeTexture.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "PlanetAtmosphereActor.generated.h"

/** Renders a volumetric atmosphere and cloud layer via post-process materials.
 *
 *  Spawns two child actors (APostProcessVolume + ADirectionalLight) and creates
 *  3 dynamic material instances (preprocess, atmosphere, postprocess) that are
 *  assigned as blendables on the post-process volume.
 *
 *  All scattering, cloud, and ray marching parameters are exposed as UPROPERTYs
 *  and pushed to the materials every tick via UpdateMaterialParameters.
 *  The directional light's rotation and color are synced from the actor's
 *  rotation and LightColor property via UpdateLightFromRotation.
 *
 *  When planet-owned: location and scale are locked (driven by the planet),
 *  rotation remains editable (controls light direction). */
UCLASS()
class VOXELPLUGIN_API APlanetAtmosphereActor : public AActor
{
    GENERATED_BODY()

public:
    APlanetAtmosphereActor();

    /** True when spawned and driven by APlanetActor. Location and scale become
     *  read-only; rotation remains editable (controls light direction). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Atmosphere")
    bool bIsPlanetOwned = false;

    // --- Atmosphere Scattering ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    FLinearColor RayleighBeta = FLinearColor(0.896360f, 2.913294f, 4.0f, 1.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    float RayleighHeight = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    FLinearColor MieBeta = FLinearColor(1.0f, 0.83163f, 0.71612f, 1.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    float MieHeight = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    float MieG = 0.9f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    FLinearColor AtmosphereAbsorptionBeta = FLinearColor(0.05f, 0.05f, 0.05f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    float AtmosphereAbsorptionHeight = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    float AtmosphereAbsorptionFalloff = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Scattering")
    FLinearColor AtmosphereAmbient = FLinearColor(0.080328f, 0.080328f, 0.1f, 0.0f);

    // --- Cloud Shape ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    TObjectPtr<UVolumeTexture> CloudVolumeTexture = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    FLinearColor AnimationWeights = FLinearColor(0.0f, 0.0f, 0.0f, 990.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    float CloudCoverage = 0.6f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    float CloudDensityMultiplier = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    float CloudHeightCurveMax = 0.7f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    float CloudHeightCurveMin = 0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    float CloudNoiseFrequency = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    FLinearColor CloudNoiseInvert = FLinearColor(0.0f, 0.0f, 0.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    FLinearColor CloudNoiseWeights = FLinearColor(0.55f, 0.3f, 0.15f, 0.3f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    float DetailErodeStrength = 0.3f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    float DetailNoiseFrequency = 6.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    FLinearColor DetailNoiseInvert = FLinearColor(1.0f, 1.0f, 1.0f, 1.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Shape")
    FLinearColor DetailNoiseWeights = FLinearColor(0.2f, 0.3f, 0.3f, 0.2f);

    // --- Cloud Lighting ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudBeta = FLinearColor(150.0f, 145.3125f, 140.625f, 0.5f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudAbsorptionBeta = FLinearColor(25.0f, 25.0f, 25.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudAmbient = FLinearColor(0.04f, 0.04f, 0.05f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudPhaseParams = FLinearColor(0.9f, 0.1f, 0.5f, 0.5f);

    // --- Light ---

    /** RGB = light color direction, magnitude of RGB = intensity. Alpha = LightColor.A
     *  is passed through to the material but not used by the directional light. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Light")
    FLinearColor LightColor = FLinearColor(1.0f, 0.95f, 0.9f, 10.0f);

    // --- Ray Marching ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float AtmosphereSteps = 32.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float CloudSteps = 64.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float AtmosphereLightSteps = 16.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float CloudLightSteps = 32.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float StepScaleFactor = 2.0f;

    // --- Planet Geometry ---
    // These are ratios of PlanetRadius (actor scale), not absolute distances.

    /** Atmosphere outer shell = PlanetRadius * (1 + AtmosphereHeightScale). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float AtmosphereHeightScale = 0.5f;

    /** Cloud layer outer shell = PlanetRadius * (1 + CloudOuterHeightScale). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float CloudOuterHeightScale = 0.4f;

    /** Cloud layer inner shell = PlanetRadius * (1 + CloudInnerHeightScale). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float CloudInnerHeightScale = 0.05f;

    /** Vertical offset applied to the atmosphere floor (planet surface). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float AtmosphereFloorOffset = 0.0f;

    // --- Postprocess ---

    /** Controls how quickly the atmosphere blur falls off with distance from the planet edge. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Postprocess")
    float BlurFalloffFactor = 2.0f;

    /** Maximum blur weight at the planet edge. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Postprocess")
    float MaxW = 0.5f;

    /** Minimum blur weight (applied everywhere within the atmosphere). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Postprocess")
    float MinW = 0.05f;

    // --- Lifecycle ---

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override { return true; }
    virtual void Tick(float DeltaTime) override;

    // --- Editor Property & Transform Locking ---

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual bool CanEditChange(const FProperty* InProperty) const override;
    virtual void PostEditMove(bool bFinished) override;
    virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
#endif

    /** Called by PlanetActor after spawn + attach. Sets bIsPlanetOwned, binds the
     *  transform guard, and runs Initialize. InScale sets the actor's scale before
     *  initialization. Skips the deferred OnConstruction path. */
    void InitializeFromPlanet(USceneComponent* InAttachParent,
        FVector InScale = FVector::ZeroVector);

private:
    /** Root component — child actors (PPV, light) attach here. */
    UPROPERTY()
    TObjectPtr<USceneComponent> AtmosphereRoot;

    /** Unbound post-process volume carrying the 3 blendable material instances. */
    UPROPERTY()
    TObjectPtr<APostProcessVolume> PostProcessVolume = nullptr;

    /** Directional light whose rotation and color are synced from actor rotation
     *  and the LightColor property. */
    UPROPERTY()
    TObjectPtr<ADirectionalLight> SunLight = nullptr;

    // --- Dynamic Material Instances (created from plugin base materials) ---

    /** Pass 0: preprocess (depth/setup). */
    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Preprocess = nullptr;

    /** Pass 1: atmosphere + cloud ray marching. */
    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Atmosphere = nullptr;

    /** Pass 2: distance-based blur compositing. */
    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Postprocess = nullptr;

    bool bInitialized = false;

    /** When true, Initialize runs on the next Tick. Set by OnConstruction to defer
     *  initialization until the world is fully ready. */
    bool bPendingInitialize = true;

    /** Cached scale set by the planet actor, used by the transform guard. */
    FVector PlanetDrivenScale = FVector::OneVector;

    /** Bound to RootComponent->TransformUpdated when planet-owned.
     *  Snaps location and scale back to planet-driven values; leaves rotation alone. */
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);

    /** Runs the full initialization: spawns child actors, creates material instances,
     *  pushes all parameters, and syncs the light. */
    void Initialize();

    /** Spawns the APostProcessVolume and ADirectionalLight as child actors,
     *  attaching them to AtmosphereRoot. */
    void SpawnChildActors();

    /** Destroys the post-process volume and directional light, nulls the MID pointers. */
    void DestroyChildActors();

    /** Creates the 3 dynamic material instances from the plugin's base materials
     *  and assigns them as blendables on the post-process volume. Loads the default
     *  cloud volume texture if none is assigned. */
    void CreateMaterialInstances();

    /** Pushes all UPROPERTY values to the atmosphere and postprocess material instances.
     *  Called every tick and on property changes. */
    void UpdateMaterialParameters();

    /** Syncs the directional light's rotation and color/intensity from the actor's
     *  rotation and LightColor property. */
    void UpdateLightFromRotation();

    /** Loads a material by content path, returning nullptr on failure. */
    static UMaterialInterface* LoadMaterialAsset(const TCHAR* Path);
};