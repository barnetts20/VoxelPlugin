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

    FVector CameraPosition = FVector::ZeroVector;
    
    FVector LastLodUpdatePosition = FVector(FLT_MAX);
    
    double CameraFOV = 90;

    FRWLock OctreeLock;

    bool TickInEditor = false;

    FastNoise::SmartNode<> Noise;
    
    std::atomic<bool> Initialized = false;
    
    std::atomic<bool> IsDestroyed = false;

public:
    AAdaptiveVoxelActor();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Materials")
    UMaterialInterface* Material;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scale")
    double Size = 100000000.0;

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
    double MinMeshUpdateInterval = .1;

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
    
    FTimerHandle MeshUpdateTimerHandle;

    void CleanSceneRoot();
    
    void Initialize();

    void RunDataUpdateTask();

    void RunMeshUpdateTask();

    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};

