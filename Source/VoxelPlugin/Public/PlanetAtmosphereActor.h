#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Engine/VolumeTexture.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "PlanetAtmosphereActor.generated.h"

/**
 * Manages a post-process volume (with 3 blendable material instances) and a
 * directional light to render a volumetric atmosphere + cloud layer.
 *
 * When owned by APlanetActor the actor's scale is set externally to
 * max(OceanRadius, PlanetRadius) — the visible surface floor. The actor
 * reads its own scale as PlanetRadius for material parameters.
 *
 * Actor Location  → Planet Center / Atmosphere Center
 * Actor Scale max → Planet Radius
 * Actor Rotation  → Light Direction + directional light rotation
 */
UCLASS()
class VOXELPLUGIN_API APlanetAtmosphereActor : public AActor
{
    GENERATED_BODY()

public:
    APlanetAtmosphereActor();

    // ───────────────────────── Atmosphere Scattering ─────────────────────────

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

    // ───────────────────────── Cloud Shape ───────────────────────────────────

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

    // ───────────────────────── Cloud Lighting ────────────────────────────────

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudBeta = FLinearColor(150.0f, 145.3125f, 140.625f, 0.5f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudAbsorptionBeta = FLinearColor(25.0f, 25.0f, 25.0f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudAmbient = FLinearColor(0.04f, 0.04f, 0.05f, 0.0f);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Cloud Lighting")
    FLinearColor CloudPhaseParams = FLinearColor(0.9f, 0.1f, 0.5f, 0.5f);

    // ───────────────────────── Light ─────────────────────────────────────────

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Light")
    FLinearColor LightColor = FLinearColor(1.0f, 0.95f, 0.9f, 10.0f);

    // ───────────────────────── Ray Marching ──────────────────────────────────

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float AtmosphereSteps = 32.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float CloudSteps = 32.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float AtmosphereLightSteps = 8.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float CloudLightSteps = 8.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Ray Marching")
    float StepScaleFactor = 2.0f;

    // ───────────────────────── Planet Geometry ───────────────────────────────

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float AtmosphereHeightScale = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float CloudOuterHeightScale = 0.4f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float CloudInnerHeightScale = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Planet Geometry")
    float AtmosphereFloorOffset = 0.0f;

    // ───────────────────────── Postprocess ───────────────────────────────────

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Postprocess")
    float BlurFalloffFactor = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Postprocess")
    float MaxW = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Atmosphere|Postprocess")
    float MinW = 0.05f;

    // ───────────────────────── Lifecycle ─────────────────────────────────────

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override { return true; }
    virtual void Tick(float DeltaTime) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    /** Called by PlanetActor after spawn + attach. Skips self-init path. */
    void InitializeFromPlanet(USceneComponent* InAttachParent);

private:
    UPROPERTY()
    TObjectPtr<USceneComponent> AtmosphereRoot;

    UPROPERTY()
    TObjectPtr<APostProcessVolume> PostProcessVolume = nullptr;

    UPROPERTY()
    TObjectPtr<ADirectionalLight> SunLight = nullptr;

    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Preprocess = nullptr;

    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Atmosphere = nullptr;

    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Postprocess = nullptr;

    bool bInitialized = false;
    bool bPendingInitialize = true;

    void Initialize();
    void SpawnChildActors();
    void DestroyChildActors();
    void CreateMaterialInstances();
    void UpdateMaterialParameters();
    void UpdateLightFromRotation();

    /** Load a material by path, returning nullptr on failure. */
    static UMaterialInterface* LoadMaterialAsset(const TCHAR* Path);
};