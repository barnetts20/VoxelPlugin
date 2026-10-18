// VoxelLandRiverBuild.h - The rivers' tables made on the GPU (Design/RiversLakes.md, R1b;
// VoxelLandRiverBuild.ush has the network and the tiles, VoxelLandRiverLayout.ush the tables).
// About 120 compute passes over a bake's cells, its river nodes and its tiles, recorded in
// one frame: at the end of a fresh bake's solve, on the solve's own buffers (VoxelLandSolve.cpp),
// or for a loaded bake or new river settings, on the bake's GPU copy (VoxelLandBake.cpp). The
// tables stay on the GPU for the density and are read back once for the CPU side (collision, the
// material's constants). The buffers are sized from an estimate before the counts are known; a
// build that didn't fit says so in its counts and is recorded again with the counts' sizes.
// Private: it pulls in the render graph.

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphResources.h"

class FRDGBuilder;
class FRHIGPUBufferReadback;

namespace VoxelLandRiverBuild
{
    /** RiverCounts' slots (VOXEL_RIVER_COUNT_* in VoxelLandRiverBuild.ush; keep equal). */
    enum ECount : int32
    {
        CountNodes = 0,
        CountSegments = 1,
        CountCandidates = 2,
        CountBlocks = 3,
        CountErrors = 4,
        CountRiverCells = 5,
        CountChains = 6,
        CountSources = 7,
        CountConfluences = 8,
        CountMouths = 9,
        CountInlets = 10,
        CountOutlets = 11,
        CountLengthDam = 12,
        CountTilesUsed = 13,
        CountMaxInTile = 14,
        CountDropped = 15,
        CountEntries = 16,
        NumCounts = 32
    };

    /** VOXEL_LAND_RIVER_TILE_CELLS, VOXEL_LAND_RIVER_MAX_PER_TILE, VOXEL_LAND_RIVER_SEG_FLOATS. */
    constexpr int32 TileCells = 8;
    constexpr int32 MaxPerTile = 128;
    constexpr int32 SegFloats = 14;
    constexpr int32 FrameFloats = 12;   // VOXEL_LAND_RIVER_FRAME_FLOATS
    constexpr int32 EntryFloats = 7;    // VOXEL_LAND_RIVER_ENTRY_FLOATS
    /** Builds before giving up (each grows the sizes by what the last one counted). */
    constexpr int32 MaxBuilds = 6;

    inline int32 TilesPerFace(int32 N) { return (N + TileCells - 1) / TileCells; }

    /** One build's grid and settings. */
    struct FSettings
    {
        int32 N = 0;
        int32 Pad = 4;
        float RadiusM = 1.0f;          // the sea-level radius
        float MinAreaKm2 = 200.0f;     // RiverMinArea (above 0)
        float WidthScale = 1.0f;       // RiverWidthScale (from 0.1)
        float ValleyScale = 1.0f;      // ValleyWidthScale (from 0.1)
        uint32 LandCells = 0;          // for the estimate (0: a third of the cells)
    };

    /** The buffers' sizes: river nodes, segments, tile candidates (uncapped lists), data floats. */
    struct FCaps
    {
        uint32 Nodes = 0;
        uint32 Segments = 0;
        uint32 Candidates = 0;
        uint32 Data = 0;
    };

    /** Sizes for a first build: the river cells' share of the land fitted to test bakes (it falls
     *  as the threshold's cells grow, about as their count to the -0.45), with headroom. */
    FCaps EstimateCaps(const FSettings& Settings);

    /** The sizes for the next build after one that didn't fit (Counts: its RiverCounts). */
    FCaps GrowCaps(const FCaps& Caps, const uint32* Counts);

    /** The inputs, per cell of the padded grid: the tan table (Side float2), heights, lake levels,
     *  the drainage as halves (two cells a word), receivers. CellFlags and CellNode: optional
     *  scratch of a word per cell to reuse (else made). */
    struct FInputs
    {
        FRDGBufferRef Tan = nullptr;
        FRDGBufferRef Height = nullptr;
        FRDGBufferRef Water = nullptr;
        FRDGBufferRef Flow = nullptr;
        FRDGBufferRef Rec = nullptr;
        FRDGBufferRef CellFlags = nullptr;
        FRDGBufferRef CellNode = nullptr;
    };

    struct FOutputs
    {
        FRDGBufferRef Tiles = nullptr;     // TileCount + 1 words
        FRDGBufferRef Data = nullptr;      // Caps.Data floats
        FRDGBufferRef Counts = nullptr;    // NumCounts words
    };

    /** Render thread. Records the build. */
    FOutputs AddPasses(FRDGBuilder& GraphBuilder, const FInputs& Inputs, const FSettings& Settings, const FCaps& Caps);

    /** A build read back. */
    struct FResult
    {
        int32 TilesPerFace = 0;
        TArray<uint32> Tiles;              // TileCount + 1
        TArray<float> Data;                // the segments and the tile blocks (trimmed)
        uint32 Counts[NumCounts] = {};
        bool bFits = false;                // false: record again with Needed
        FCaps Needed;
        TRefCountPtr<FRDGPooledBuffer> GPUTiles;   // the tables on the GPU (Data with its slack)
        TRefCountPtr<FRDGPooledBuffer> GPUData;
    };

    /** The readbacks of one recorded build (render thread). */
    class FPending
    {
    public:
        FPending();
        ~FPending();
        FPending(FPending&&);
        FPending& operator=(FPending&&);

        void Enqueue(FRDGBuilder& GraphBuilder, const FOutputs& Outputs, const FSettings& Settings, const FCaps& Caps);
        bool IsValid() const;
        bool IsReady() const;
        /** Once ready: the result (false when a readback couldn't be read). Resets. */
        bool Read(FResult& Out);
        void Reset();

    private:
        TUniquePtr<FRHIGPUBufferReadback> Tiles, Data, Counts;
        TRefCountPtr<FRDGPooledBuffer> GPUTiles, GPUData;
        uint32 TileWords = 0;
        FCaps Caps;
        int32 N = 0;
    };

    /** Render thread or game thread. Every build shader is compiled for this platform. */
    bool AreShadersReady();
}
