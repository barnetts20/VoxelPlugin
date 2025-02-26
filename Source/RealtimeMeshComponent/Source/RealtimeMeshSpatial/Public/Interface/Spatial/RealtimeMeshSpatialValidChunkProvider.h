// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

#pragma once

#include "CoreFwd.h"
#include "Ext/RealtimeMeshFactoryCommon.h"

struct FRealtimeMeshSpatialComponentLocation;

namespace RealtimeMesh
{
	DECLARE_DELEGATE_OneParam(FRealtimeMeshSpatialCellMeshChangedDelegate, const FRealtimeMeshSpatialComponentLocation&);
	
	class IRealtimeMeshSpatialStreamingStructureProvider
	{
	protected:
		~IRealtimeMeshSpatialStreamingStructureProvider() = default;
	public:
		virtual int32 GetMaxLOD() const = 0;
		virtual FVector3d GetCellLocation(const FRealtimeMeshSpatialComponentLocation& CellLocation) const = 0;
		virtual FVector3d GetCellSize(int32 LOD) const = 0;
		virtual FBox3d GetCellBounds(const FRealtimeMeshSpatialComponentLocation& CellLocation) const = 0;
		
		virtual void ClampBoundsToValidRegion(FInt64Vector& Min, FInt64Vector& Max, int32 LOD) const = 0;
		virtual bool IsCellValid(const FInt64Vector& Cell, int32 LOD) const = 0;

		virtual FRealtimeMeshSpatialCellMeshChangedDelegate& OnCellChanged() = 0;
	};

	class IRealtimeMeshSpatialStreamingFactoryStructureProvider : public IRealtimeMeshSpatialStreamingStructureProvider
	{
	protected:
		~IRealtimeMeshSpatialStreamingFactoryStructureProvider() = default;
	public:
		virtual TSharedPtr<FRealtimeMeshFactoryInitializationParams> GetCellInitParams(const FRealtimeMeshSpatialComponentLocation& Cell) = 0;
	};
}
