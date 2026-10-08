// VoxelBrickMesherGPU.h - The stitched brick mesher (VoxelBrickMesher::MeshBrickStitched) on
// the GPU, with edits. AddMesherPasses records the passes into a render graph (the brick pages
// of VoxelBrickPages render from them); MeshBricksAsync reads the meshes back to check them
// against the CPU (Design/GPUPath.md stage b).

#pragma once

#include "CoreMinimal.h"
#include "VoxelBrickMesher.h"
#include "VoxelBrickTerrain.h"
#include "VoxelDensityGPU.h"
#include "RenderGraphDefinitions.h"

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
    /** Bricks per MeshBricksAsync call and per AddMesherPasses batch. */
    constexpr int32 MaxBricks = 32;

    /** A vertex record is VertexStride floats (VoxelMesherVertexStride): position cm from the
     *  brick min, normal, the parent target's position and normal, the grandparent target's
     *  position and normal, the unstitched position and normal (the cell solved as if the
     *  finer leaves around it were of its level), the cell's level relative to the brick (0 or
     *  1), pad. */
    constexpr uint32 VertexStride = 26;

    /** Per-brick slots, scaled with the brick's cell count (powers of two). */
    struct FCaps
    {
        uint32 HashSize = 0;          // points off the own lattice
        uint32 TargetHashSize = 0;    // target corners and gradient points
        uint32 FaceCap = 0;
        uint32 VertexCap = 0;
        uint32 TriangleCap = 0;
    };

    /** Error bit for a request that wasn't meshed (beyond MaxBricks, or out of the 32-bit
     *  lattice range). */
    constexpr uint32 ErrorNotMeshed = 0x100;

    /** Meshes up to MaxBricks bricks on the GPU and calls OnDone on the game thread with one
     *  mesh and one stats entry per request (positions relative to the brick min, as
     *  MeshBrickStitched). Results arrive a few frames later (readback); empty arrays when the
     *  GPU can't run the shaders. CellsPerBrick is clamped to 8..64. Call from any thread but
     *  the render thread. */
    /** Cells per brick edge the shaders use for Settings: CellsPerBrick rounded down to a
     *  multiple of 8, clamped to 8..64. */
    VOXELPLUGIN_API int32 CellsFor(const FVoxelBrickMesherGPUSettings& Settings);

    VOXELPLUGIN_API FCaps CapsFor(const FVoxelBrickMesherGPUSettings& Settings);

    /** The shader's record of a brick. False when its lattice indices, with the points
     *  sampled around it, don't fit 32 bits (the brick can't be meshed on the GPU). */
    VOXELPLUGIN_API bool MakeBrick(const FVoxelBrickKey& Key, const int8 (&NeighbourLevels)[27], uint32 MeshedNeighbours,
        const FVoxelBrickMesherGPUSettings& Settings, FVoxelMesherBrickGPU& Out);

    /** Render thread. True when every mesher pass is compiled for this platform. */
    VOXELPLUGIN_API bool AreShadersReady();

    /** The density octaves and the flattened edit tree as graph buffers. */
    struct FInputs
    {
        FRDGBufferRef Octaves = nullptr;
        FRDGBufferRef EditCorners = nullptr;
        FRDGBufferRef EditChildren = nullptr;
        FRDGBufferRef EditFlags = nullptr;
        uint32 EditNodeCount = 0;
        int32 EditRootExtent = 0;
    };

    /** Per brick b of a batch: Counters[4b..4b+3] = faces, vertices, triangles, error bits;
     *  vertices from b * Caps.VertexCap, triangles from b * Caps.TriangleCap. */
    struct FOutputs
    {
        FCaps Caps;
        FRDGBufferRef Counters = nullptr;
        FRDGBufferRef Vertices = nullptr;
        FRDGBufferRef Indices = nullptr;
    };

    /** Render thread. Uploads Density's octaves and Edits into GraphBuilder. */
    VOXELPLUGIN_API FInputs UploadInputs(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density, const FVoxelEditGPUData& Edits);

    /** Render thread. Records the mesher passes for a batch of bricks (any number; the brick
     *  pages batch BatchBricksFor of them), on PassFlags' pipe: AsyncCompute lets a batch whose
     *  results are read in a later frame overlap the frame's rendering. */
    VOXELPLUGIN_API FOutputs AddMesherPasses(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density, const FInputs& Inputs,
        const FVoxelBrickMesherGPUSettings& Settings, TConstArrayView<FVoxelMesherBrickGPU> Bricks, ERDGPassFlags PassFlags = ERDGPassFlags::Compute);

    VOXELPLUGIN_API void MeshBricksAsync(TSharedRef<const FVoxelDensityGPUData> Density, TSharedRef<const FVoxelEditGPUData> Edits,
        const FVoxelBrickMesherGPUSettings& Settings, TArray<FVoxelBrickMesherGPURequest> Requests,
        TFunction<void(TArray<FVoxelBrickMesh>&&, TArray<FVoxelBrickMesherGPUStats>&&)> OnDone);
}
