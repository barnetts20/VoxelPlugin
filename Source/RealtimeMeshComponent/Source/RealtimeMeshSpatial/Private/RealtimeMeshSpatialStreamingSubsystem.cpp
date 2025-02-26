// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.


#include "RealtimeMeshSpatialStreamingSubsystem.h"
#include "GameFramework/Character.h"
#include "GameFramework/PlayerController.h"
#include "RealtimeMeshSpatialComponent.h"
#include "RealtimeMeshSpatialStreamingSource.h"
#include "RealtimeMeshSpatialStreamingSourceComponent.h"

#if WITH_EDITOR
#include "Editor.h"
#include "Subsystems/UnrealEditorSubsystem.h"
#endif

DECLARE_CYCLE_STAT(TEXT("RealtimeMeshSpatial - SubsystemTick"), STAT_RealtimeMesh_SpatialSubsystemTick, STATGROUP_RealtimeMesh);

void URealtimeMeshSpatialStreamingSubsystem::RegisterSource(IRealtimeMeshSpatialStreamingSourceProvider* Provider)
{
	StreamingSourceProviders.AddUnique(Provider);
}

void URealtimeMeshSpatialStreamingSubsystem::UnregisterSource(IRealtimeMeshSpatialStreamingSourceProvider* Provider)
{
	StreamingSourceProviders.RemoveSwap(Provider);
}

void URealtimeMeshSpatialStreamingSubsystem::RegisterActor(URealtimeMeshSpatialComponent* Component)
{
	SpatialComponents.AddUnique(Component);
}

void URealtimeMeshSpatialStreamingSubsystem::UnregisterActor(URealtimeMeshSpatialComponent* Component)
{
	SpatialComponents.RemoveSwap(Component);
}

void URealtimeMeshSpatialStreamingSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
}

void URealtimeMeshSpatialStreamingSubsystem::Deinitialize()
{
	Super::Deinitialize();
}

bool URealtimeMeshSpatialStreamingSubsystem::IsTickable() const
{
	return SpatialComponents.Num() > 0;
	//return (StreamingSourceProviders.Num() > 0 || GetWorld()->IsEditorWorld()) && SpatialComponents .Num() > 0;
}

void URealtimeMeshSpatialStreamingSubsystem::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	SCOPE_CYCLE_COUNTER(STAT_RealtimeMesh_SpatialSubsystemTick);
		
	TArray<FRealtimeMeshSpatialStreamingSource> ActiveSources;
	ActiveSources.Reserve(StreamingSourceProviders.Num() + 1);

	APlayerController* PlayerController = GetWorld()? GetWorld()->GetFirstPlayerController() : nullptr;

	// By default we'll always have the source for the player/editor
	if (ACharacter* Player = (PlayerController? PlayerController->GetCharacter() : nullptr))
	{		
		// Auto setup streaming source if there isn't one on the player
		if (!PlayerController->FindComponentByClass<URealtimeMeshSpatialStreamingSourceComponent>() &&
			!Player->FindComponentByClass<URealtimeMeshSpatialStreamingSourceComponent>())
		{
			FRealtimeMeshSpatialStreamingSource Source;
			Source.Name = TEXT("Player");
			Source.Location = Player->GetActorLocation();
			Source.Radius = 10000.0f;
			Source.DebugColor = FColor::Green;
			Source.Priority = ERealtimeMeshStreamingSourcePriority::High;
			ActiveSources.Add(Source);
		}
	}
#if WITH_EDITOR
	else if (GEditor)
	{
		if (UUnrealEditorSubsystem* UnrealEditorSubsystem = GEditor->GetEditorSubsystem<UUnrealEditorSubsystem>())
		{
			FVector Location;
			FRotator Rotation;
			if (UnrealEditorSubsystem->GetLevelViewportCameraInfo(Location, Rotation))
			{
				FRealtimeMeshSpatialStreamingSource Source;
				Source.Name = TEXT("EditorViewport");
				Source.Location = Location;
				Source.Radius =30000.0f;
				Source.DebugColor = FColor::Green;
				Source.Priority = ERealtimeMeshStreamingSourcePriority::High;
				ActiveSources.Add(Source);	
			}
		}
	}
#endif
	
	// Grab all our active sources
	for (IRealtimeMeshSpatialStreamingSourceProvider* StreamingSourceProvider : StreamingSourceProviders)
	{
		FRealtimeMeshSpatialStreamingSource Source;
		if (StreamingSourceProvider->GetSpatialStreamingSource(Source))
		{
			ActiveSources.Add(Source);
		}
	}

	if (ActiveSources.Num() > 0)
	{
		// Walk all the spatial actors, and update their streaming status using the sources
		for (auto It = SpatialComponents.CreateIterator(); It; ++It)
		{
			// Check if this is a stale reference and remove it
			if (!It->IsValid())
			{
				It.RemoveCurrent();
				continue;
			}
			auto SpatialActor = It->Get();

			// Localize the sources to this actors position/rotation/scale so streaming can be done in local space
			TArray<FRealtimeMeshSpatialStreamingSource> LocalizedSources;
			LocalizedSources.Reserve(ActiveSources.Num());
			for (const FRealtimeMeshSpatialStreamingSource& Source : ActiveSources)
			{
				FRealtimeMeshSpatialStreamingSource& LocalizedSource = LocalizedSources.Add_GetRef(Source);
				LocalizedSource.Location = SpatialActor->GetComponentTransform().InverseTransformPosition(LocalizedSource.Location);
				LocalizedSource.Radius /= SpatialActor->GetComponentTransform().GetMaximumAxisScale();
			}
				
			SpatialActor->UpdateStreaming(LocalizedSources);
		}
	}
	
}
