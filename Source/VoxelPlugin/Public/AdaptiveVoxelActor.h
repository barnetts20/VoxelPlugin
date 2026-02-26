#pragma once

#include "CoreMinimal.h"
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
    // Adaptive octree meshes the SDF
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;
    // Edit store saves user terraforming changes
    TSharedPtr<FSparseEditStore> EditStore; //MOVING TO OCTREE

    FVector CameraPosition = FVector::ZeroVector;
    FVector LastLodUpdatePosition = FVector(FLT_MAX);
    double CameraFOV = 90;

    FRWLock OctreeLock;

    bool TickInEditor = true;
    std::atomic<bool>  Initialized = false;
    std::atomic<bool>  IsDestroyed = false;

public:
    AAdaptiveVoxelActor();

    //Properties - TODO: Need to trigger reconstruction of entire object when any of these properties change, they are not compatible with changes while it is already running
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
    double ScreenSpaceThreshold = .07;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double MinDataUpdateInterval = .15;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double MinMeshUpdateInterval = .1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double VelocityLookAheadFactor = 16;

    virtual void OnConstruction(const FTransform& Transform) override;
    
    virtual void BeginPlay() override;
    
    virtual void BeginDestroy() override;
    
    virtual bool ShouldTickIfViewportsOnly() const override;
    
    virtual void Tick(float DeltaTime) override;

    TSharedPtr<FAdaptiveOctree> GetOctree();

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

