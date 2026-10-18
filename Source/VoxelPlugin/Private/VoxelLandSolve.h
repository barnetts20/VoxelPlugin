// VoxelLandSolve.h - The land bake's erosion solve and final routing on the GPU (Design/LandGen.md
// section 5; stage L2). After the plate pass (VoxelLandBake.cpp), a render-thread job runs, a
// few compute passes a frame:
//   1. the land cells (the mountains, continental crust whose uplift is past the range-front
//      ramp, and since T1 the plains with a small uplift of their own) are compacted into
//      slots, start at their static height with roughness, and are filled until they drain;
//   2. steps of the implicit stream-power solve, each: the fill along the previous receivers
//      and a few Planchon-Darboux refinements, the receivers (steepest descent; depressions
//      filled), drainage accumulation (warm-started Jacobi), the implicit update (pointer
//      jumping over affine maps), the residual; until it is below the setting or the step
//      limit (the GPU decides, so the step count is reproducible);
//   3. the uplifted mass above each cell's static surface is scaled so its 90th percentile
//      matches the profile's (the mountains'), and mapped above it by a logarithm fitted to
//      put the 99.9th percentile on the top, then settled so no path steps uphill
//      (Design/SurfaceLook.md, T1; L2 scaled the relief above the outlet, which lost the
//      static surface under ranges, and eased it flat);
//   4. the land is routed on the final heights: lake levels (a fill without descent), the
//      routing fill, receivers, exact drainage, lake labels; lakes past the size settings are
//      kept, smaller depressions filled;
//   5. the fields (aprons from the neighbouring faces) and the lake list are read back once;
//   6. with rivers on, the rivers' tables are built in the same frame on the routing's own buffers
//      (VoxelLandRiverBuild.h, R1b) and read back with the fields; the heights, the drainage and
//      the lake levels stay on the GPU for the density (FResult's GPU buffers).
// The work is VoxelLandSolve.ush / .usf. Every pass is order-independent, so the same GPU
// reproduces a bake bit for bit. Private: it pulls in the render graph.

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphResources.h"
#include "VoxelLandBake.h"
#include "VoxelLandRiverBuild.h"
#include <atomic>

class FRHICommandListImmediate;
class FRHIGPUBufferReadback;

namespace VoxelLandSolve
{
    /** The solve's inputs (fixed for a bake). */
    struct FSettings
    {
        int32 N = 0;
        int32 Pad = 4;
        float RadiusM = 1.0f;
        uint32 Seed = 0;
        float PlainsUplift = 400.0f;     // m
        float RampWidth = 300.0f;        // m
        float UpliftTime = 5.0e6f;       // years for the full uplift
        float Kdt = 0.4f;                // erodibility x step (the look test's)
        float Dt = 0.4f / 4.3e-7f;       // years per step
        float Eps = 0.01f;               // the fill's descent per cell (m)
        int32 MaxSteps = 150;
        int32 MinSteps = 10;
        float Tolerance = 0.02f;
        float MinLakeDepth = 10.0f;      // m
        float MinLakeArea = 20.0e6f;     // m^2
        // The T1 model (Design/SurfaceLook.md; VoxelLandSolve.ush): the static surface kept,
        // the plains solved with a small uplift, a jittered routing graph, uplift and
        // erodibility fields, initial roughness, the power relief map.
        float Jitter = 0.4f;             // cells
        // T1b (Design/SurfaceLook.md): the receivers' random choice, and the drainage field's
        // spread over hillslopes (cells; 0 the D8 areas).
        float RouteNoise = 1.0f;
        float SpreadCells = 16.0f;
        float UpliftVariation = 0.8f;
        float ErodibilityVariation = 0.7f;
        float VariationWavelength = 9.0e4f;   // m
        float Roughness = 150.0f;        // m
        float PlainsRelief = 100.0f;     // m
        float Undulation = 60.0f;        // m (T1b)
        float LandformErosion = 1.0f;    // T2: the landform scales PlainsRelief and Undulation
        float Incision = 150.0f;         // m
        float ReliefTopScale = 1.2f;     // the top over the profile's 99.9th percentile
        // T3g (Design/SurfaceLook.md; VoxelLandSolve.ush): the channel heads' area in cells
        // (0 off: every cell incises, no slope cap), its variation 2^(+-ChannelVariation) over
        // a noise of ChannelWavelength (m) and ChannelOctaves, and the hillslopes' cap (a
        // multiple of the steady channel slope at the cell's head area).
        float ChannelCells = 0.0f;
        float ChannelVariation = 1.5f;
        float ChannelWavelength = 1.8e5f;
        int32 ChannelOctaves = 3;
        float Hillslope = 1.5f;
        // T3h: how far the drainage density pulls the landform's character (0 off).
        float ChannelCharacter = 0.0f;
        bool bErode = true;
        // R1b: the rivers' build after the routing (Design/RiversLakes.md; VoxelLandRiverBuild.h).
        bool bRivers = false;
        float RiverMinAreaKm2 = 200.0f;
        float RiverWidthScale = 1.0f;
        float RiverValleyScale = 1.0f;
        int64 StepCellsPerFrame = 1 << 25;        // slot-steps a frame
        int64 IterationCellsPerFrame = 1 << 26;   // slot-iterations a frame (the fills, the spread, labels)
        // S1: the routing recorded in one frame with fixed pass counts, checked at the output's
        // readback (voxel.Land.SolveRouteAhead).
        bool bRouteAhead = true;
        TArray<FVector2f> TanTable;      // the bake's (FVoxelLandBake::TanTable)
    };

    /** The plate pass's per-cell fields, on the GPU (the bake job's buffers). */
    struct FPlateFields
    {
        TRefCountPtr<FRDGPooledBuffer> Height;
        TRefCountPtr<FRDGPooledBuffer> Up;
        TRefCountPtr<FRDGPooledBuffer> Crust;
        TRefCountPtr<FRDGPooledBuffer> Attr;
        TRefCountPtr<FRDGPooledBuffer> Form;     // the landform, one cell a word (VoxelLandPackForm; the province << 16)
    };

    /** The bake's fields after the solve, on the padded grid, as read back (the baker narrows
     *  them on a worker), with what the log reports. */
    struct FResult
    {
        TArray<float> Height;
        TArray<uint32> Attr;             // FVoxelLandBake::Attr in the low 16 bits
        TArray<uint32> Form;             // FVoxelLandBake::Form in the low 16 bits (the plate pass's; the character after T3h), the province (T4c) from bit 16
        TArray<uint32> FlowHalves;       // log2 m^2 as halves, two cells a word (FVoxelLandBake::Flow's bits)
        TArray<float> Water;
        TArray<uint32> Receiver;         // R1: the final routing's receiver per cell (~0 off the land, in the aprons)
        TArray<FVoxelLandLake> Lakes;
        int32 SolvedCells = 0;
        int32 LandCells = 0;
        int32 Steps = 0;
        float Residual = 0.0f;
        float Factor = 1.0f;
        float ReliefKnee = 0.0f;
        float ReliefTop = 0.0f;
        float ReliefScale = 0.0f;        // the logarithm's scale above the knee (m; large: nearly linear)
        uint32 Errors = 0;               // VOXEL_SOLVE_ERROR_* bits
        int32 LakesFound = 0;            // kept lakes, before the list's cap
        TArray<FString> PhaseLog;        // per phase: frames and seconds (voxel.Debug.Land)
        /** R1b: the rivers' tables (when FSettings::bRivers; bRivers false if the build failed). */
        bool bRivers = false;
        VoxelLandRiverBuild::FResult Rivers;
        int32 RiverBuilds = 0;           // 1, or more when the first sizes didn't fit
        /** The fields on the GPU, as read back (render thread resources: the baker hands them to
         *  the bake's GPU copy, so the density doesn't upload them again). */
        TRefCountPtr<FRDGPooledBuffer> GPUHeight, GPUWater, GPUFlow, GPUTan;
    };

    /** Render thread. One bake's solve. */
    class FJob
    {
    public:
        explicit FJob(FSettings InSettings);
        ~FJob();

        /** Once a frame: polls the readbacks and records the next passes. Sets Status. */
        void Tick(FRHICommandListImmediate& RHICmdList, const FPlateFields& Fields);

        /** 0 running, 1 the result is ready, -1 failed. */
        std::atomic<int32> Status{ 0 };
        /** 0..1, for the planet's progress. */
        std::atomic<float> Progress{ 0.0f };
        /** Written on the render thread before Status becomes 1. */
        FResult Result;

    private:
        struct FImpl;
        TUniquePtr<FImpl> Impl;
    };

    /** Render thread or game thread. Every solve shader is compiled for this platform. */
    bool AreShadersReady();
}
