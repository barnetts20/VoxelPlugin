// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshSpatialComponent.h"
#include "GameFramework/Actor.h"
#include "RealtimeMeshSpatialActor.generated.h"

UCLASS()
class REALTIMEMESHSPATIAL_API ARealtimeMeshSpatialActor : public AActor
{
	GENERATED_BODY()

private:
	UPROPERTY(Category="RealtimeMesh|Spatial", EditAnywhere, BlueprintReadOnly, meta=(ExposeFunctionCategories="Mesh,Rendering,Physics,Components|RealtimeMesh", AllowPrivateAccess="true"))
	TSubclassOf<URealtimeMeshSpatialComponent> SpatialComponentClass;
	
	UPROPERTY(Category="RealtimeMesh|Spatial", VisibleAnywhere, BlueprintReadOnly, meta=(ExposeFunctionCategories="Mesh,Rendering,Physics,Components|RealtimeMesh", AllowPrivateAccess="true"))
	TObjectPtr<URealtimeMeshSpatialComponent> SpatialComponent;
public:
	ARealtimeMeshSpatialActor();

	UFUNCTION(BlueprintCallable, Category="RealtimeMesh|Spatial")
	void InitializeFromClass(const TSubclassOf<URealtimeMeshSpatialComponent>& InSpatialComponentClass);

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
#endif
};