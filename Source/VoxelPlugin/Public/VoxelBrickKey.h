// VoxelBrickKey.h - A brick of the terrain's level-of-detail tree and the brick changes a
// page commit takes. Shared by the terrain (VoxelBrickTerrain), the pages (VoxelBrickPages)
// and the GPU mesher (VoxelBrickMesherGPU).

#pragma once

#include "CoreMinimal.h"

/** A brick of the LOD tree: level 0 has the finest cells, each level up doubles the cell size.
 *  Coordinates count bricks of that level from the shared lattice origin. */
struct FVoxelBrickKey
{
    int32 Level = 0;
    FInt64Vector Coord;

    bool operator==(const FVoxelBrickKey& Other) const
    {
        return Level == Other.Level && Coord == Other.Coord;
    }

    friend uint32 GetTypeHash(const FVoxelBrickKey& Key)
    {
        return HashCombine(::GetTypeHash(Key.Level), GetTypeHash(Key.Coord));
    }

    /** Value / 2^Levels rounded toward minus infinity (brick coordinates are signed). */
    static int64 FloorShift(int64 Value, int32 Levels)
    {
        return Value >= 0 ? (Value >> Levels) : -((-Value + (1LL << Levels) - 1) >> Levels);
    }

    /** The brick Up levels coarser that holds this one. */
    FVoxelBrickKey Ancestor(int32 Up) const
    {
        FVoxelBrickKey Out;
        Out.Level = Level + Up;
        Out.Coord = FInt64Vector(FloorShift(Coord.X, Up), FloorShift(Coord.Y, Up), FloorShift(Coord.Z, Up));
        return Out;
    }

    /** The brick of the next coarser level that holds this one. */
    FVoxelBrickKey Parent() const
    {
        return Ancestor(1);
    }

    /** Child Index of the next finer level: bit 0 along x, bit 1 along y, bit 2 along z. */
    FVoxelBrickKey Child(int32 Index) const
    {
        FVoxelBrickKey Out;
        Out.Level = Level - 1;
        Out.Coord = FInt64Vector(2 * Coord.X + (Index & 1), 2 * Coord.Y + ((Index >> 1) & 1), 2 * Coord.Z + ((Index >> 2) & 1));
        return Out;
    }

    /** The brick of this level DX, DY, DZ bricks away. */
    FVoxelBrickKey Offset(int32 DX, int32 DY, int32 DZ) const
    {
        FVoxelBrickKey Out = *this;
        Out.Coord = FInt64Vector(Coord.X + DX, Coord.Y + DY, Coord.Z + DZ);
        return Out;
    }
};

/** One brick change of a GPU page commit (FVoxelBrickPages::Commit). */
struct FVoxelBrickPageChange
{
    FVoxelBrickKey Key;
    bool bRemove = false;
    int8 Levels[27] = {};          // neighbour levels relative to the brick (VoxelBrickMesher)
    uint32 Meshed = 0;             // same-level neighbours that are meshed
};
