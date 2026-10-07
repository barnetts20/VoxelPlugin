// VoxelBrickMesherGPU.h - The stitched brick mesher (VoxelBrickMesher::MeshBrickStitched) on
// the GPU, with edits. Stage (b) of Design/GPUPath.md: meshes a batch of bricks and reads the
// meshes back, to check them against the CPU. Rendering from GPU buffers is stage (c).

#pragma once

#include "CoreMinimal.h"
#include "VoxelBrickMesher.h"
#include "VoxelBrickTerrain.h"
#include "VoxelDensityGPU.h"

/** One brick of a GPU batch (FVoxelMesherBrick in VoxelBrickStitch.ush). */
struct FVoxelMesherBrickGPU
{
    FIntVector MinIndex = FIntVector::ZeroValue;   // half finest cells from the root centre
    int32 Level = 0;
    uint32 LevelsLo = 0;
    uint32 LevelsHi = 0;
    uint32 Meshed = 0;
    uint32 Pad = 0;
};
static_assert(sizeof(FVoxelMesherBrickGPU) == 32, "Must match FVoxelMesherBrick in VoxelBrickStitch.ush");

/** A brick to mesh: its key and neighbour levels, as MeshBrickStitched takes them. */
struct FVoxelBrickMesherGPURequest
{
    FVoxelBrickKey Key;
    int8 NeighbourLevels[27] = {};
    uint32 MeshedNeighbours = 0;
};

struct FVoxelBrickMesherGPUSettings
{
    double FinestCell = 100.0;   // level-0 cell (cm)
    int32 MaxDepth = 0;          // the root spans 2^MaxDepth finest cells
    int32 CellsPerBrick = 32;
};

/** Per brick: the counts the GPU produced and its error bits (VoxelMesherError* in
 *  VoxelBrickMesherPasses.ush: 1 faces, 2 hash, 4 missing sample, 8 vertices, 16 triangles). */
struct FVoxelBrickMesherGPUStats
{
    uint32 Faces = 0;
    uint32 Vertices = 0;
    uint32 Triangles = 0;
    uint32 Errors = 0;
};

namespace VoxelBrickMesherGPU
{
    /** Bricks per call (buffers are sized for it). */
    constexpr int32 MaxBricks = 32;

    /** Error bit for a request that wasn't meshed (beyond MaxBricks, or out of the 32-bit
     *  lattice range). */
    constexpr uint32 ErrorNotMeshed = 0x100;

    /** Meshes up to MaxBricks bricks on the GPU and calls OnDone on the game thread with one
     *  mesh and one stats entry per request (positions relative to the brick min, as
     *  MeshBrickStitched). Results arrive a few frames later (readback); empty arrays when the
     *  GPU can't run the shaders. CellsPerBrick is clamped to 8..64. Call from any thread but
     *  the render thread. */
    VOXELPLUGIN_API void MeshBricksAsync(TSharedRef<const FVoxelDensityGPUData> Density, TSharedRef<const FVoxelEditGPUData> Edits,
        const FVoxelBrickMesherGPUSettings& Settings, TArray<FVoxelBrickMesherGPURequest> Requests,
        TFunction<void(TArray<FVoxelBrickMesh>&&, TArray<FVoxelBrickMesherGPUStats>&&)> OnDone);
}
