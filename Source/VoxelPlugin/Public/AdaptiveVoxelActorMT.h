// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FAdaptiveOctreeMT.h"
#include "AdaptiveVoxelActorMT.generated.h"

UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActorMT : public ARealtimeMeshActor
{
	GENERATED_BODY()
	
private:
    TSharedPtr<FAdaptiveOctreeMT> AdaptiveOctree;
    TSharedPtr<FSparseOctree> SparseOctree;

    FCameraInfo CameraData;
    FCameraInfo LastCameraData;
  
    FRWLock OctreeLock;

    int ChunkDepth = 4;
    int MinDepth = 7;
    int MaxDepth = 16;
    int LodFactor = 12;
    int CollisionDepth = 14;

    bool TickInEditor = false;
    bool Initialized = false;

public:
    AAdaptiveVoxelActorMT();
    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

    TSharedPtr<FAdaptiveOctreeMT> GetOctree();

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

};
