#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "MaterialDomain.h"
#include "FAdaptiveOctree.h"
#include "FSparseOctree.h"
#include "FMeshingStructs.h"
#include "RealtimeMeshActor.h"
#include "AdaptiveVoxelActor.generated.h"

using namespace RealtimeMesh;

UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActor : public ARealtimeMeshActor
{
    GENERATED_BODY()
    
private:
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree; //Adaptive octree meshes
    TSharedPtr<FSparseOctree> SparseOctree;     //Sparse octree will be for storing user edits (terraforming etc)

    FVector CameraPosition = FVector::ZeroVector;
    FVector LastLodUpdatePosition = FVector(FLT_MAX); // force first update
    double CameraFOV = 90;

    FRWLock OctreeLock;

    bool TickInEditor = false; //TODO: Work on a more robust lifecycle so this can be enabled while also working in play mode
    bool Initialized = false;
    bool IsDestroyed = false;

    double ChunkExtent = 0;
    double LodDistanceThreshold = 0;
public:
    AAdaptiveVoxelActor();
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

    TSharedPtr<FAdaptiveOctree> GetOctree();

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
    int MaxDepth = 19;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double ScreenSpaceThreshold = .075;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double MinDataUpdateInterval = .01;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double MinMeshUpdateInterval = .1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double VelocityLookAheadFactor = 16;

protected:
    void CleanSceneRoot();
    void InitializeChunks();

    //Async
    std::atomic<bool> DataUpdateIsRunning = false;
    std::atomic<bool> MeshUpdateIsRunning = false;

    FTimerHandle DataUpdateTimerHandle;
    FTimerHandle MeshUpdateTimerHandle;

    void RunDataUpdateTask();
    void RunMeshUpdateTask();
};

