#pragma once

#include "CoreMinimal.h"
#include "FastNoise/FastNoise.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "MaterialDomain.h"
#include "FAdaptiveOctree.h"
#include "FMeshingStructs.h"
#include "RealtimeMeshActor.h"
#include "FSparseEditStore.h"
#include "AdaptiveVoxelActor.generated.h"

using namespace RealtimeMesh;

UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

private:
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;

    // Scene component with absolute scale — mesh chunks attach here
    // so they are not affected by actor scale (which is used as a parameter source only).
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    FVector CameraPosition = FVector::ZeroVector;

    FVector LastLodUpdatePosition = FVector(FLT_MAX);

    double CameraFOV = 90;

    FRWLock OctreeLock;

    bool TickInEditor = true;

    FastNoise::SmartNode<> Noise;

    std::atomic<bool> Initialized = false;

    std::atomic<bool> IsDestroyed = false;

public:
    AAdaptiveVoxelActor();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Materials")
    UMaterialInterface* SurfaceMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Materials")
    UMaterialInterface* OceanMaterial;

    // Planet radius is derived from the maximum component of actor scale (in world units).
    // All scale axes are locked to the max component to keep the planet spherical.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.25;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain", meta = (ClampMin = "-1.0", ClampMax = "2.0"))
    double SeaLevelCoefficient = 0.5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int ChunkDepth = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int MinDepth = 7;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int MaxDepth = 18;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double ScreenSpaceThreshold = .075;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double MinDataUpdateInterval = .1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double VelocityLookAheadFactor = 8;

    virtual void OnConstruction(const FTransform& Transform) override;

    virtual void BeginPlay() override;

    virtual void BeginDestroy() override;

    virtual bool ShouldTickIfViewportsOnly() const override;

    virtual void Tick(float DeltaTime) override;

protected:
    std::atomic<bool> DataUpdateIsRunning = false;

    std::atomic<bool> MeshUpdateIsRunning = false;

    std::atomic<bool> EditUpdateIsRunning = false;

    FTimerHandle DataUpdateTimerHandle;

    void CleanSceneRoot();

    void Initialize();

    void RunDataUpdateTask();

    void RunMeshUpdateTask();

    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};