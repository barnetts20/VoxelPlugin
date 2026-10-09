// VoxelBrickMesher.h - Dual contouring of uniform bricks: a global lattice of cubic cells,
// cut into bricks of CellsPerBrick^3 cells, each meshed on its own from the density alone.
// MeshBrick meshes the collision bricks; MeshBrickStitched is the reference the GPU mesher
// (VoxelBrickMesherGPU) reproduces, and the brick terrain uses its empty test.

#pragma once

#include "CoreMinimal.h"

class FDensitySampleCompositor;

/** The brick lattice: lattice point (0,0,0) at Origin, cubic cells of CellSize, bricks of
 *  CellsPerBrick cells per axis. Brick coordinates are int64 so any planet fits. */
struct VOXELPLUGIN_API FVoxelBrickGrid
{
    FVector Origin = FVector::ZeroVector;
    double CellSize = 100.0;
    int32 CellsPerBrick = 16;   // a multiple of 4 (the empty test samples every 4th point)

    double BrickSize() const { return CellSize * (double)CellsPerBrick; }

    FVector BrickMin(const FInt64Vector& Brick) const
    {
        return Origin + FVector((double)Brick.X, (double)Brick.Y, (double)Brick.Z) * BrickSize();
    }

    /** Region whose density a brick's mesh depends on: its cells plus a two-cell apron below
     *  and one above (vertices of the cells around its edges, and their gradients). */
    FBox DependencyBounds(const FInt64Vector& Brick) const
    {
        const FVector Min = BrickMin(Brick);
        return FBox(Min - FVector(2.0 * CellSize), Min + FVector((CellsPerBrick + 1) * CellSize));
    }

    FInt64Vector BrickAt(const FVector& Position) const
    {
        const FVector Local = (Position - Origin) / BrickSize();
        return FInt64Vector((int64)FMath::FloorToDouble(Local.X), (int64)FMath::FloorToDouble(Local.Y), (int64)FMath::FloorToDouble(Local.Z));
    }
};

/** One brick's surface: positions relative to the brick's min corner, three indices per
 *  triangle, in the render chunks' winding. */
struct FVoxelBrickMesh
{
    TArray<FVector3f> Positions;
    TArray<FVector3f> Normals;
    TArray<int32> Indices;

    int32 NumTriangles() const { return Indices.Num() / 3; }
};

struct FVoxelBrickStats
{
    int32 Samples = 0;            // density samples taken
    bool bSkipped = false;        // proven empty by the coarse test
    float MaxGradient = 0.0f;     // largest |grad density| seen at a surface crossing
};

namespace VoxelBrickMesher
{
    /** Meshes Brick by dual contouring. Every global lattice edge belongs to exactly one brick
     *  (the one holding its lower end), so neighbouring bricks' meshes meet without gaps or
     *  overlaps: both compute the shared cell vertices from the same samples.
     *
     *  Unless bNoSkip, the brick is first tested for emptiness: one sample at its centre, then
     *  a lattice of every 4th point. It is skipped when every test sample is farther from zero
     *  than SlopeBound (the largest |grad density| assumed) times the distance to the farthest
     *  point it covers. Pass bNoSkip where the bound may not hold (edited regions). */
    VOXELPLUGIN_API void MeshBrick(const FDensitySampleCompositor& Density, const FVoxelBrickGrid& Grid,
        const FInt64Vector& Brick, double SlopeBound, bool bNoSkip, FVoxelBrickMesh& Out, FVoxelBrickStats& OutStats);

    /** Meshes Brick so that it meets neighbours one level finer or coarser without cracks.
     *  NeighbourLevels holds the level of the leaf brick covering each of the 26 neighbouring
     *  regions of this brick's size, relative to this brick (-1, 0 or +1; index
     *  (dz+1)*9 + (dy+1)*3 + (dx+1), the centre ignored). The tree must be 2:1 balanced across
     *  faces, edges and corners. Bit d of MeshedNeighbours is set when the neighbour in
     *  direction d is a brick of this level that is meshed too; edges this brick shares only
     *  with unmeshed bricks (culled, or outside the tree) are emitted here.
     *
     *  Dual contouring on the leaf cells, as for octrees: faces come from minimal edges (edges
     *  of the finest cell touching them), and an edge whose cells are of different levels
     *  gives a triangle instead of a quad. Each brick emits the minimal edges of its own
     *  level, so the finer side of a transition stitches it. Every lattice point is sampled
     *  with the cell size of the leaf owning it as footprint, so bricks that share a point or
     *  a vertex compute it identically. CellsPerBrick must be at least 8.
     *
     *  Grid is this brick's level; the empty test is MeshBrick's. */
    VOXELPLUGIN_API void MeshBrickStitched(const FDensitySampleCompositor& Density, const FVoxelBrickGrid& Grid,
        const FInt64Vector& Brick, const int8 (&NeighbourLevels)[27], uint32 MeshedNeighbours, double SlopeBound, bool bNoSkip,
        FVoxelBrickMesh& Out, FVoxelBrickStats& OutStats);

    /** MeshBrickStitched's empty test alone: true when it would skip Brick (without bNoSkip).
     *  Adds the samples it took to InOutSamples. */
    VOXELPLUGIN_API bool IsBrickProvablyEmpty(const FDensitySampleCompositor& Density, const FVoxelBrickGrid& Grid,
        const FInt64Vector& Brick, double SlopeBound, int32& InOutSamples);
}
