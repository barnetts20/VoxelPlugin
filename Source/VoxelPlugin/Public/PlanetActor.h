// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AdaptiveVoxelActor.h"
#include "OceanSphereActor.h"
#include "FSparseEditStore.h"

#include "PlanetActor.generated.h"

UCLASS()
class VOXELPLUGIN_API APlanetActor : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	APlanetActor();
	double PlanetRadius;
	double NoiseScale;
	double NoiseAmplitude;
	double SeaLevel;

protected:

	AAdaptiveVoxelActor* PlanetSurfaceActor;
	AOceanSphereActor* OceanSurfaceActor;
	TSharedPtr<FSparseEditStore> EditStore;
	
	TFunction<void(int streamSize, const float* xStream, const float* yStream, const float* zStream, float* sampleOut)> NoiseFunction;
	TFunction<void(int streamSize, const float* xStream, const float* yStream, const float* zStream, float* densityOut)> DensityFunction;
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
