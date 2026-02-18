#pragma once
#include "CoreMinimal.h"
#include <FInt64Coordinate.h>

//TODO: DOES NOT APPEAR TO BE IN USE IN THE COMPLETED DC IMPLEMENTATION, CHECK USAGES AND REMOVE IF POSSIBLE (WE MAY NEED IT, OR SOMETHING LIKE IT FOR SPARSE DATA)
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