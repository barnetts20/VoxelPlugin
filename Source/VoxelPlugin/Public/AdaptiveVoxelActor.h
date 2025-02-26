#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FAdaptiveOctree.h"
#include "FSparseOctree.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>
#include "FMeshingStructs.h"
#include "AdaptiveVoxelActor.generated.h"

using namespace RealtimeMesh;

UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActor : public ARealtimeMeshActor
{
    GENERATED_BODY()
    
private:
    FTimerHandle DataUpdateTimerHandle;
    FTimerHandle MeshUpdateTimerHandle;
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;
    TSharedPtr<FSparseOctree> SparseOctree;

    FVector CameraPosition;
    FVector LastCameraPosition;
    TArray<TSharedPtr<FMeshChunk>> Chunks;
    FRWLock OctreeLock;

    URealtimeMeshSimple* RtMesh;
    URealtimeMeshComponent* RtComponent;

    FRealtimeMeshSectionConfig SecConfig = FRealtimeMeshSectionConfig(0);
    int ChunkDepth = 3;
    int MinDepth = 6;
    int MaxDepth = 16;
    int LodFactor = 12;
    int CollisionDepth = 14;

    bool TickInEditor = false;
    bool Initialized = false;

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
    void DrawDebugSurfaceNodes();
};

