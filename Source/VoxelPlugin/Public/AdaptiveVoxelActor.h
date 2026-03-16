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

    // Mesh chunks attach to this component. Inherits actor position and rotation
    // but uses absolute scale (1,1,1). Octree is built at world scale; scale changes
    // trigger full reconstruction via OnConstruction.
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

    FVector LastInitScale = FVector::ZeroVector;

public:
    AAdaptiveVoxelActor();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Materials")
    UMaterialInterface* SurfaceMaterial;

    // Planet radius in world units is determined by actor scale (max component).
    // The octree is built at world scale in actor-local space (origin 0,0,0).
    // Position and rotation changes are handled live by the actor transform.
    // Scale changes trigger full reconstruction.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int ChunkDepth = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int MinDepth = 6;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int MaxDepth = 21;

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