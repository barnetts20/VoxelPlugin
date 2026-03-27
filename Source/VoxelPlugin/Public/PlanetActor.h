#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AdaptiveVoxelActor.h"
#include "OceanSphereActor.h"
#include "FDensitySampleCompositor.h"
#include "FSparseEditStore.h"
#include "FastNoise/FastNoise.h"
#include "PlanetActor.generated.h"

class APlanetAtmosphereActor;

UCLASS()
class VOXELPLUGIN_API APlanetActor : public AActor
{
    GENERATED_BODY()

public:
    APlanetActor();

    // Root component � provides the transform gizmo in the editor.
    // Scale.GetMax() = planet radius in cm.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Planet")
    TObjectPtr<USceneComponent> PlanetRoot;

    // ------- Shape -------

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape",
        meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.15;

    // Sea level as a fraction of the noise distribution [0, 1].
    // 0 = ocean at PlanetRadius (lowest terrain), 1 = ocean at PlanetRadius + NoiseAmplitude.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Shape",
        meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double SeaLevel = 0.5;

    // ------- Ocean -------

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Ocean")
    bool bEnableOcean = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Ocean")
    UMaterialInterface* OceanMaterial = nullptr;

    // ------- Terrain -------

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Terrain")
    UMaterialInterface* TerrainMaterial = nullptr;

    // ------- Atmosphere -------

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Planet|Atmosphere")
    bool bEnableAtmosphere = true;

    // ------- Accessors -------

    UFUNCTION(BlueprintCallable, Category = "Planet")
    AAdaptiveVoxelActor* GetTerrainActor() const { return TerrainActor; }

    UFUNCTION(BlueprintCallable, Category = "Planet")
    AOceanSphereActor* GetOceanActor() const { return OceanActor; }

    UFUNCTION(BlueprintCallable, Category = "Planet")
    APlanetAtmosphereActor* GetAtmosphereActor() const { return AtmosphereActor; }

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override { return true; }
    virtual void Tick(float DeltaTime) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

private:
    UPROPERTY()
    AAdaptiveVoxelActor* TerrainActor = nullptr;

    UPROPERTY()
    AOceanSphereActor* OceanActor = nullptr;

    UPROPERTY()
    APlanetAtmosphereActor* AtmosphereActor = nullptr;

    FastNoise::SmartNode<> Noise;

    // Shared compositor (noise + edit store) built during Initialize and reused
    // for child actor re-initialization (e.g. material swaps, ocean radius updates).
    TSharedPtr<FDensitySampleCompositor> SharedCompositor;

    FVector LastInitScale = FVector::ZeroVector;
    double LastSeaLevel = -1.0;
    double LastNoiseAmplitudeRatio = -1.0;
    bool bLastEnableOcean = true;
    bool bLastEnableAtmosphere = true;
    bool bInitialized = false;
    bool bPendingInitialize = true;
    bool bPendingOceanUpdate = false;

    void Initialize();
    void SpawnChildActors();
    void DestroyChildActors();

    TSharedPtr<FDensitySampleCompositor> BuildCompositor(double PlanetRadius,
        double NoiseAmplitude, double RootExtent, int32 ChunkDepth, int32 MaxDepth);

    double ComputeOceanRadius(double PlanetRadius, double NoiseAmplitude) const;
};