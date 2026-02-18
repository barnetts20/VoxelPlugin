//TODO: UPDATE TO DOUBLE PRECISION SPARSE OCTREE INSTEAD OF INT64 BASED

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "FSparseOctree.h"
#include "SparseVoxelActor.generated.h"

UCLASS()
class VOXELPLUGIN_API ASparseVoxelActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ASparseVoxelActor();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel Debug")
	double Precision = 0.0000001; // Default min coordinate system precision

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel Debug")
	bool bShowDebug = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Voxel Debug")
	bool bShowOccupiedNodes = true;

	TArray<FVector> SampleOffsets;
	TArray<FVector> ActualOffsets;
	TArray<int> SampleDepths;
	FVector CameraPosition;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	void InitializeOctree();

private:
	TSharedPtr<FSparseOctree> VoxelOctree;
	bool bDebugNeedsRedraw;

	//Debug Draw
	void DebugDrawOctree();
	void DebugDrawNode(TSharedPtr<FSparseOctreeNode> node);
};
