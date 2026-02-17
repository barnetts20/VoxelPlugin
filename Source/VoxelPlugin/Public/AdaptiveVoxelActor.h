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
    double Size = 500000000.0;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int ChunkDepth = 5;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int MinDepth = 7;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int MaxDepth = 18;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int LodFactor = 8;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Update")
    double MinDataUpdateInterval = .1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Update")
    double MinMeshUpdateInterval = .2;

protected:
    void CleanSceneRoot();
    void InitializeChunks();

    //Async
    bool DataUpdateIsRunning = false;
    void ScheduleDataUpdate(float IntervalInSeconds);
    bool MeshUpdateIsRunning = false;
    void ScheduleMeshUpdate(float IntervalInSeconds);
};

