// VoxelLandBake.h - The terrestrial generator's bake (Design/LandGen.md section 3): per-cell
// fields on the equal-angle cube-sphere grid, made once per planet at load and cached on disk,
// and the driver that makes it (time-sliced compute passes, read back once; on the CPU when
// there is no GPU). The density samples it on both backends (VoxelLandSample.ush).
//
// The plate model (VoxelLandPlates.ush) gives the crust hypsometry and the boundary profiles;
// on the GPU the continental ranges then rise into a stream-power erosion solve, and a final
// routing gives the drainage and the lakes (VoxelLandSolve.ush, stage L2). Without a GPU the
// bake is the plate model alone (not cached).

#pragma once

#include "CoreMinimal.h"
#include "Async/Future.h"
#include "Math/Float16.h"
#include "VoxelLandSettings.h"

struct FVoxelLandBakeGPU;
struct FVoxelLandBakeJob;
struct FVoxelLandCpuProgress;

/** A lake the final routing kept: a depression at least MinLakeDepth deep and MinLakeArea
 *  large, filled to its spill level. */
struct FVoxelLandLake
{
    uint32 Cell = 0;         // its lowest-index cell (padded cell index): its id
    float Level = 0.0f;      // water level (m about sea level)
    uint32 Outlet = 0;       // the lake cell it drains from (lowest index), or ~0 for none
    float AreaKm2 = 0.0f;
};

/** One planet's bake. Immutable once made; shared by the density on any thread. */
struct VOXELPLUGIN_API FVoxelLandBake
{
    /** Apron cells on every side of a face (the sampler's blend band and B-spline stencil). */
    static constexpr int32 Pad = 4;
    /** Changes whenever the bake's content or file layout changes (old cache files are then
     *  ignored). */
    static constexpr uint32 Version = 3;

    FVoxelLandBake();
    ~FVoxelLandBake();
    FVoxelLandBake(const FVoxelLandBake&) = delete;
    FVoxelLandBake& operator=(const FVoxelLandBake&) = delete;

    int32 N = 0;                  // cells per face edge
    double SeaRadius = 0.0;       // cm
    uint32 Key = 0;               // VoxelLand::BakeKey
    uint32 SettingsCheck = 0;     // VoxelLand::BakeCheck
    float CrustThreshold = 0.0f;  // the plate model's crust threshold for the continental fraction
    float MinHeight = 0.0f;       // m, over every cell, aprons included
    float MaxHeight = 0.0f;
    double BakeSeconds = 0.0;     // how long making it took (0 when loaded)
    bool bFromCache = false;
    /** Made by the erosion solve (else the plate model alone: the CPU fallback, or
     *  voxel.Land.Erosion 0); only eroded bakes are cached. */
    bool bEroded = false;
    int32 SolveSteps = 0;         // steps the solve ran
    float SolveResidual = 0.0f;   // its last residual (mean height change over mean uplift)
    float ReliefFactor = 1.0f;    // the relief normalisation's scale

    /** Heights (m about sea level), 6 faces of Side() x Side() cells, rows along the face's U
     *  axis; cell (i, j) of face f, i and j from -Pad, at f * Side^2 + (j + Pad) * Side + i + Pad. */
    TArray<float> Height;

    /** Per cell: crust fraction (bits 0-7, 0..255), boundary class (bits 8-10: 0 none,
     *  1 collision, 2 arc range, 3 trench, 4 island arc, 5 rift, 6 ridge), sea-connected
     *  (bit 11, stage L5), under a kept lake (bit 12). */
    TArray<uint16> Attr;
    static constexpr uint16 LakeBit = 1u << 12;

    /** Per cell: log2 of the drainage area (m^2), a cell's own area off the land; empty when
     *  not eroded. */
    TArray<FFloat16> Flow;

    /** Per cell: the kept lake's water level (m), or -3e38 where there is none; empty when not
     *  eroded. */
    TArray<float> Water;

    /** The kept lakes, by Cell. */
    TArray<FVoxelLandLake> Lakes;

    /** tan(pi s_i / 4) for the cell centres along a face axis (i from -Pad), as double-float
     *  pairs: the sampler's exact fractions. */
    TArray<FVector2f> TanTable;

    /** The plate model's plates (8 floats each, VoxelLandPlates.ush) and seed, for the land
     *  debug views; made from the settings when the bake is ready (not cached). */
    TArray<float> Plates;
    uint32 PlateSeed = 0;

    int32 Side() const { return N + 2 * Pad; }
    int32 NumCells() const { return 6 * Side() * Side(); }

    /** Fills TanTable for N. */
    void BuildTanTable();

    /** Fills MinHeight and MaxHeight from Height. */
    void UpdateRange();

    /** The GPU copy the density samples (render thread; uploaded on first use, released on
     *  the render thread with the bake). */
    TSharedPtr<FVoxelLandBakeGPU, ESPMode::ThreadSafe> GPU;
};

/** Makes a planet's bake: from the disk cache when it holds one for the key, else on the GPU
 *  (a few compute passes per frame, read back once), else on the CPU (no renderer, e.g. a
 *  dedicated server, or shaders not available). Game thread. */
class VOXELPLUGIN_API FVoxelLandBaker
{
public:
    FVoxelLandBaker(const FVoxelTerrestrialSettings& InSettings, double InSeaRadius);
    ~FVoxelLandBaker();

    uint32 GetKey() const { return Key; }

    /** Once per frame: advances the bake. */
    void Tick();

    bool IsDone() const { return Result.IsValid(); }

    /** 0..1. */
    float GetProgress() const;

    /** The finished bake (null until IsDone). */
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> GetResult() const { return Result; }

private:
    enum class EState : uint8
    {
        Start,
        LoadingCache,
        Gpu,
        GpuSolving,
        GpuAssembling,
        Cpu,
        Done
    };

    void StartGpu();
    void StartCpu();
    void Finish(TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake);

    FVoxelTerrestrialSettings Settings;
    double SeaRadius = 0.0;
    uint32 Key = 0;
    int32 N = 0;
    EState State = EState::Start;
    double StartSeconds = 0.0;
    int32 GpuStep = 0;         // compute steps issued
    int32 GpuSteps = 0;        // in all (crust and threshold, then the field chunks)
    int32 GpuWaitFrames = 0;   // frames waited for the shaders before the first step
    TSharedPtr<FVoxelLandBakeJob, ESPMode::ThreadSafe> Job;
    TSharedPtr<FVoxelLandCpuProgress, ESPMode::ThreadSafe> CpuProgress;
    TFuture<TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe>> Pending;
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Result;
};

namespace VoxelLand
{
    /** Bump when the generator's output changes for the same settings (invalidates caches). */
    constexpr uint32 GeneratorVersion = 2;

    /** voxel.Land.Erosion: whether bakes run the erosion solve (part of the key). */
    VOXELPLUGIN_API bool IsErosionEnabled();

    /** The cache key of a bake: the bake settings, the radius (to the cm), N, the versions. */
    VOXELPLUGIN_API uint32 BakeKey(const FVoxelTerrestrialSettings& Settings, double SeaRadius);

    /** A second, independent hash of the same inputs, stored in cache files and compared on
     *  load (against key collisions). */
    VOXELPLUGIN_API uint32 BakeCheck(const FVoxelTerrestrialSettings& Settings, double SeaRadius);

    /** Cells per face edge a bake of Settings has. */
    VOXELPLUGIN_API int32 BakeResolutionFor(const FVoxelTerrestrialSettings& Settings);

    /** The planet shape a bake gives the density: the base radius (sea radius plus the lowest
     *  possible height, cm) and the height range above it (cm), covering the bake and the
     *  per-sample layers' reach. */
    VOXELPLUGIN_API void ShapeFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake,
        double& OutPlanetRadius, double& OutNoiseAmplitude);

    /** The arc of one bake cell at the face centre (cm). */
    inline double CellArc(double SeaRadius, int32 N) { return UE_DOUBLE_PI * 0.5 * SeaRadius / FMath::Max(N, 1); }
}
