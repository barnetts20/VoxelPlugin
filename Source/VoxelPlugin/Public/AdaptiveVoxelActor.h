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
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;
    TSharedPtr<FSparseOctree> SparseOctree;

    FVector CameraPosition;
    FVector LastCameraPosition;
    FRWLock OctreeLock;

    int ChunkDepth = 3;
    int MinDepth = 8;
    int MaxDepth = 14;
    int LodFactor = 8;
    int CollisionDepth = 14;

    double MinDataUpdateInterval = .05;
    bool TickInEditor = false;
    bool Initialized = false;
    bool IsDestroyed = false;

    double Size = 10000000.0;

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

