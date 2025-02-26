// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "Math/RandomStream.h"

/**
 * Streaming Source Priority
 */
UENUM(BlueprintType)
enum class ERealtimeMeshStreamingSourcePriority : uint8
{
	Highest = 0,
	High = 64,
	Normal = 128,
	Low = 192,
	Lowest = 255,
	Default = Normal
};

/**
 * Structure containing all properties required to stream from a source
 */
struct REALTIMEMESHSPATIAL_API FRealtimeMeshSpatialStreamingSource
{
	FRealtimeMeshSpatialStreamingSource()
		: Location()
		, Radius(10000.0f)
		, Priority(ERealtimeMeshStreamingSourcePriority::Default)
		, DebugColor(ForceInit)
	{
	}

	FRealtimeMeshSpatialStreamingSource(FName InName, const FVector& InLocation, ERealtimeMeshStreamingSourcePriority InPriority)
		: Name(InName)
		, Location(InLocation)
		, Radius(10000.0f)
		, Priority(InPriority)
		, DebugColor(ForceInit)
	{}

	FColor GetDebugColor() const
	{
		if (!DebugColor.ToPackedBGRA())
		{
			return FColor::MakeRedToGreenColorFromScalar(FRandomStream(Name).GetFraction());
		}

		return FColor(DebugColor.R, DebugColor.G, DebugColor.B, 255);
	}

	/** Source unique name. */
	FName Name;

	/** Source location. */
	FVector Location;

	/** Source Radius */
	float Radius;

	/** Streaming source priority. */
	ERealtimeMeshStreamingSourcePriority Priority;

	/** Color used for debugging. */
	FColor DebugColor;


	FString ToString() const
	{
		return FString::Printf(
			TEXT("Priority: %d | Pos: X=%f,Y=%f,Z=%f | Radius: %f"), 
			Priority, 
			Location.X, Location.Y, Location.Z,
			Radius
		);
	}
};

/**
 * Interface for world partition streaming sources
 */
class REALTIMEMESHSPATIAL_API IRealtimeMeshSpatialStreamingSourceProvider
{
public:
	virtual ~IRealtimeMeshSpatialStreamingSourceProvider() = default;
	virtual bool GetSpatialStreamingSource(FRealtimeMeshSpatialStreamingSource& StreamingSource) = 0;
};