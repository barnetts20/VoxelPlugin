// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once
// Help intellisense to avoid interpreting this file's declaration of FVector etc as it assumes !CPP by default
#ifndef CPP
#define CPP 1
#endif

#if CPP

#include "CoreFwd.h"
#include "Spatial/RealtimeMeshStreamingPolicyInterface.h"
//#include "RealtimeMeshSpatialNoExportTypes.generated.h"

#endif

#if !CPP      //noexport struct



USTRUCT()
struct FRealtimeMeshFactoryInitializationParamsSpatialGrid3D
{
	GENERATED_BODY()
public:

	UPROPERTY()
	FIntVector CellLocation;
	UPROPERTY()
	int32 LOD;
	UPROPERTY()
	FVector3f CellSize;
};

USTRUCT()
struct FRealtimeMeshFactoryInitializationParamsSpatialGrid2D
{
	GENERATED_BODY()
public:

	UPROPERTY()
	FIntVector2 CellLocation;
	UPROPERTY()
	int32 LOD;
	UPROPERTY()
	FVector2f CellSize;
	UPROPERTY()
	float CellHeight;
}

#endif