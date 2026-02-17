#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
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
    const TFunction<double(FVector)>* DensityFunctionPtr;
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;
    TSharedPtr<FSparseOctree> SparseOctree;

    FVector CameraPosition;
    FVector LastCameraPosition;
    FRWLock OctreeLock;

    int ChunkDepth = 4;
    int MinDepth = 7;
    int MaxDepth = 18;
    int LodFactor = 8;
    int CollisionDepth = 18;

    double MinDataUpdateInterval = .05;
    bool TickInEditor = false;
    bool Initialized = false;
    bool IsDestroyed = false;

    double Size = 100000000.0;

public:
    AAdaptiveVoxelActor();
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

    TSharedPtr<FAdaptiveOctree> GetOctree();

    // Material properties
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Materials")
    UMaterialInterface* Material;


protected:
    void CleanSceneRoot();
    void InitializeChunks();

    //Async
    bool DataUpdateIsRunning = false;
    void ScheduleDataUpdate(float IntervalInSeconds);
    bool MeshUpdateIsRunning = false;
    void ScheduleMeshUpdate(float IntervalInSeconds);

    //Debug
    //void DrawDebugSurfaceNodes();
};

