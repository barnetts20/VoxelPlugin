#pragma once
#include "CoreMinimal.h"
#include <FInt64Coordinate.h>

struct VOXELPLUGIN_API FVoxelData : public TSharedFromThis<FVoxelData>
{
public:
    uint32 ObjectId;
    float Density;
    FInt64Coordinate Position;

    FVoxelData() : ObjectId(0), Density(0.0f), Position() {}
    FVoxelData(uint32 InObectId, float InDensity) : ObjectId(InObectId), Density(InDensity), Position() {};

    float GetDensity() const
    {
        return Density;
    }

    uint8 GetObectId() const
    {
        return ObjectId;
    }

    FInt64Coordinate GetPosition() {
        return Position;
    }
};

struct VOXELPLUGIN_API FDCVoxelData : public FVoxelData
{
public:
    TArray<double> Densities;
    FInt64Coordinate DCPosition;
};