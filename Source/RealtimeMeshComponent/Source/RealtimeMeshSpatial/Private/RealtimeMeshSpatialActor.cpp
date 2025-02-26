// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.


#include "RealtimeMeshSpatialActor.h"
#include "RealtimeMeshSpatialComponent.h"


// Sets default values
ARealtimeMeshSpatialActor::ARealtimeMeshSpatialActor()
{
}

void ARealtimeMeshSpatialActor::InitializeFromClass(const TSubclassOf<URealtimeMeshSpatialComponent>& InSpatialComponentClass)
{
	FName UniqueName = MakeUniqueObjectName(this, InSpatialComponentClass, TEXT("Root"));
	RootComponent = SpatialComponent = NewObject<URealtimeMeshSpatialComponent>(this, InSpatialComponentClass, UniqueName);
}

#if WITH_EDITOR
void ARealtimeMeshSpatialActor::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

	// Reinitialize from class when changed.
	if (PropertyChangedEvent.Property && PropertyChangedEvent.Property->GetFName() == GET_MEMBER_NAME_CHECKED(ARealtimeMeshSpatialActor, SpatialComponentClass))
	{
		if (SpatialComponentClass)
		{
			InitializeFromClass(SpatialComponentClass);
		}
	}
}
#endif
