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
struct FVoxelLandNoiseGPU;
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
    static constexpr uint32 Version = 4;

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

    /** Per cell: the landform (VoxelLandPlates.ush VoxelLandFormAt): the hilliness (0 flat
     *  plains .. 255 hill country) in the low byte, the character (0 smooth, rolling .. 255
     *  rough-topped) in the high byte. The hills layer's shape and height (VoxelLandHills.ush),
     *  and the peaks' sharpness (VoxelLandPeaks.ush). An eroded bake's character also follows
     *  the solve's drainage density (T3h, VoxelLandSolve.ush VoxelSolveFormOf). */
    TArray<uint16> Form;

    /** Per cell: the local relief (VoxelLandRelief.h: the height range within about two cells,
     *  8 bits, q^2 x 12000 / 65025 m), the peaks' scale. Derived from Height when the bake is
     *  ready (BuildRelief), not cached. */
    TArray<uint8> Relief;
    float MaxRelief = 0.0f;       // m, the largest decoded

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

    /** The plate model's plates (8 floats each, VoxelLandPlates.ush), seed, radius (m) and
     *  landform width scale, for the land debug views (the plates view warps as the bake did);
     *  made from the settings when the bake is ready (not cached). */
    TArray<float> Plates;
    uint32 PlateSeed = 0;
    float PlateRadiusM = 1.0f;
    float PlateWidthScale = 1.0f;

    int32 Side() const { return N + 2 * Pad; }
    int32 NumCells() const { return 6 * Side() * Side(); }

    /** Fills TanTable for N. */
    void BuildTanTable();

    /** Fills MinHeight and MaxHeight from Height. */
    void UpdateRange();

    /** Fills Relief and MaxRelief from Height (any thread; parallel). */
    void BuildRelief();

    /** The GPU copy the density samples (render thread; uploaded on first use, released on
     *  the render thread with the bake). */
    TSharedPtr<FVoxelLandBakeGPU, ESPMode::ThreadSafe> GPU;
};

/** The land's baked noise as it samples it (VoxelLandNoise.ush; VoxelNoiseTiles.h,
 *  Design/NoiseTiles2D_Handoff.md): a pack of packed sets' mip chains as words (two a texel:
 *  R | G << 16, B | A << 16, the .vxnt files' own bytes), each from a whole row of RowN texels
 *  (the material's texture is those rows): the peaks' set (from texel 0; N 0 without peaks),
 *  the detail tile, then the hills' set (T4a; HillN 0 without). With what the layers place
 *  and normalise them with (VoxelLandPeaks.ush, VoxelLandHillSet.ush). Immutable once made;
 *  shared by every planet's density that uses the same sets. */
struct VOXELPLUGIN_API FVoxelLandNoiseSet
{
    FVoxelLandNoiseSet();
    ~FVoxelLandNoiseSet();
    FVoxelLandNoiseSet(const FVoxelLandNoiseSet&) = delete;
    FVoxelLandNoiseSet& operator=(const FVoxelLandNoiseSet&) = delete;

    FString Name;                 // the pack's key (VoxelLand::LandNoiseNameFor: "peaks|hills")
    FString LookTag;              // the peaks set's
    uint32 RowN = 0;              // texels per row (a power of two: the first chain's N)
    uint32 N = 0;                 // the peaks' set: texels per side of mip 0 (0: no peaks)
    uint32 Mips = 0;
    uint32 Rows = 0;              // rows of RowN texels: the peaks' chain (the last one padded),
                                  // the detail tile's (from row DetailBase / RowN), the hills'
    TArray<uint32> Words;         // Rows x RowN texels, two words each
    // What the peaks layer places and normalises the set with (VoxelLandPeaks.ush): the massif
    // spacing (mip-0 texels), R's and G's means over mip 0 (unit values), and the decode ranges
    // (Max - Min, mip-0 texels) of R, G and B.
    uint32 SpacingTexels = 0;
    float MeanR = 0.0f;
    float MeanG = 0.0f;
    float RangeR = 1.0f;
    float RangeG = 1.0f;
    float RangeB = 1.0f;
    // The peaks' detail tile (VoxelNoiseTiles::GeneratePeakDetail; VoxelLandPeaks.ush, T3e): its
    // chain in the words after the set's (from texel DetailBase, a whole row; 0 size: none), its
    // coarsest wavelength (mip-0 texels), R's and G's means (unit) and decode ranges (texels).
    uint32 DetailBase = 0;
    uint32 DetailN = 0;
    uint32 DetailMips = 0;
    uint32 DetailSpacingTexels = 0;
    float DetailMeanR = 0.0f;
    float DetailMeanG = 0.0f;
    float DetailRangeR = 1.0f;
    float DetailRangeG = 1.0f;
    // The hills' set (VoxelLandHillSet.ush, T4a): its chain from texel HillBase (a whole row; 0
    // size: none), its name, spacing (mip-0 texels: N over its cells per tile) and the decode
    // ranges (texels) of R, G and B. Read from its base network up (no means).
    FString HillName;
    uint32 HillBase = 0;
    uint32 HillN = 0;
    uint32 HillMips = 0;
    uint32 HillSpacingTexels = 0;
    float HillRangeR = 1.0f;
    float HillRangeG = 1.0f;
    float HillRangeB = 1.0f;

    /** Whether the words hold every chain (the sampler's bounds). */
    bool IsComplete() const { return RowN > 0 && Rows > 0 && Words.Num() == (int32)(2 * Rows * RowN); }

    /** The GPU copy (render thread; uploaded on first use, released on the render thread). */
    TSharedPtr<FVoxelLandNoiseGPU, ESPMode::ThreadSafe> GPU;
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
    constexpr uint32 GeneratorVersion = 10;   // 7: T3f, belts and the continental slope follow the continents' outline; 8: T3g, channel heads; 9: T3h, the character follows them; 10: T4b, the static surface rises monotonically into the ranges

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
     *  per-sample layers' reach. Noise: the noise pack the density takes (LandNoiseNameFor;
     *  its hill set's relief bounds the hills: T4a), or null (the procedural hills' bound). */
    VOXELPLUGIN_API void ShapeFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet* Noise,
        double& OutPlanetRadius, double& OutNoiseAmplitude);

    /** The gully filter's constants (VOXEL_LAND_GULLY_MAX_OCTAVES, _GAIN, _MAX_SLOPE in
     *  VoxelLandGully.ush; keep them equal). */
    constexpr int32 GullyMaxOctaves = 12;
    constexpr double GullyGain = 0.42;
    constexpr double GullyMaxSlope = 0.8;

    /** The gully octaves a bake gets (VoxelLandGully.ush): from two bake cells down to
     *  Settings.GullyFinestWavelength, at most 12; 0 without gullies. */
    VOXELPLUGIN_API int32 GullyOctavesFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake);

    /** The hills' constants (VOXEL_LAND_HILL_MAX_OCTAVES in VoxelLandHills.ush; keep equal),
     *  and a bound on their shape (a bound for the planet's height range: gully_harness hgrad
     *  measures -0.15 .. 1.93 over 800k points). */
    constexpr int32 HillMaxOctaves = 12;
    constexpr double HillMaxUnit = 2.5;

    /** The hill octaves (VoxelLandHills.ush): from HillWavelength down to HillFinestWavelength,
     *  at most 12; 0 without hills. */
    VOXELPLUGIN_API int32 HillOctavesFor(const FVoxelTerrestrialSettings& Settings);

    /** The finest hill octave's frequency (cycles per cm). */
    VOXELPLUGIN_API double HillFinestFrequency(const FVoxelTerrestrialSettings& Settings);

    /** The smallest massif spacing (m) the code accepts, whatever sets it: it keeps the peaks'
     *  lattice coordinates (VoxelDFLatticeSplit: under 2^31 texels) in range on any planet. */
    constexpr double PeakMinSpacing = 500.0;

    /** The peaks' placement (VoxelLandPeaks.ush) for a set: its mip-0 texels per bake cell (the
     *  massif spacing Settings.PeakSpacing over Set's spacing in texels); 0 without peaks. */
    VOXELPLUGIN_API float PeakTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set);

    /** The peaks' detail's coarsest wavelength (m): the setting within 16 m .. a quarter of the
     *  massif spacing. */
    VOXELPLUGIN_API double PeakDetailWavelength(const FVoxelTerrestrialSettings& Settings);

    /** The detail tile's mip-0 texels per bake cell (VoxelLandPeaks.ush, T3e): its coarsest
     *  wavelength placed at PeakDetailWavelength. 0 without detail (strength 0, or the set has
     *  no detail tile). */
    VOXELPLUGIN_API float PeakDetailTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set);

    /** How far the peaks can raise the surface (bUp; else lower it), cm. They act only on land
     *  (from 2 m up), so they lower it below the bake's land at most this far. */
    VOXELPLUGIN_API double PeakReach(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, bool bUp);

    /** The name of the plugin's own generated `peaks` set (VoxelNoiseTiles.h): the stand-in
     *  when no authored set is there, or one fails to load. */
    VOXELPLUGIN_API extern const TCHAR* GeneratedNoiseName;

    /** The folder the authored sets are read from: the plugin's NoiseSets folder (staged with a
     *  cooked game by VoxelPlugin.Build.cs). Found on the game thread (StartupModule); empty
     *  before that, or without the plugin. */
    VOXELPLUGIN_API FString NoiseSetDirectory();

    /** The peaks' set a planet samples (Design/SurfaceLook.md T3): Settings.PeakNoiseSet if set
     *  (a file name without .vxnt); else one of the usable `peaks*` sets in NoiseSetDirectory,
     *  picked by the planet's seed (rendezvous hashing: adding or removing a set moves only the
     *  planets that pick it); else the generated one. Empty without peaks. The folder is listed
     *  once a session (the first call reads its sets' headers). */
    VOXELPLUGIN_API FString PeakNoiseNameFor(const FVoxelTerrestrialSettings& Settings);

    /** The hills' set (T4a): Settings.HillNoiseSet if set ("none": the procedural hills); else
     *  one of the folder's `hills*` sets by the seed (as the peaks'). Empty without hills, or
     *  without a set (the procedural hills then). */
    VOXELPLUGIN_API FString HillNoiseNameFor(const FVoxelTerrestrialSettings& Settings);

    /** The noise pack a planet's density takes: "Peaks|Hills" (PeakNoiseNameFor,
     *  HillNoiseNameFor); empty when it needs neither. */
    VOXELPLUGIN_API FString LandNoiseNameFor(const FVoxelTerrestrialSettings& Settings);

    /** The pack Key (LandNoiseNameFor): its sets read (about 0.1 s for a 2048 set) or the peaks'
     *  generated (about a second of worker time), then shared while densities hold it (and the
     *  four most recent kept regardless); made again when asked for after that. A peaks file
     *  that is missing or isn't a valid set logs an error and gives the generated set; a hills
     *  file likewise, and the pack goes without it (the procedural hills). Null only if
     *  nothing could be made. Thread-safe; blocks while it's made (the planet waits for
     *  TryGetLandNoise before it builds a density, so the game thread doesn't). */
    VOXELPLUGIN_API TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> GetLandNoise(const FString& Key);

    /** Starts making it in the background, if it isn't already there or under way. */
    VOXELPLUGIN_API void PrewarmLandNoise(const FString& Key);

    /** Whether it is ready (GetLandNoise won't block): if so, Out is it (null if none could be
     *  made). The caller holds Out until its density has taken the pack, so the pack can't go
     *  in between and GetLandNoise then finds it at once. */
    VOXELPLUGIN_API bool TryGetLandNoise(const FString& Key, TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Out);

    /** Drops the shared references (before the RHI shuts down: each pack's GPU copy goes with
     *  its last holder, on the render thread). */
    VOXELPLUGIN_API void ReleaseLandNoise();

    /** The hills' placement (VoxelLandHillSet.ush, T4a) for a pack: the hill set's mip-0 texels
     *  per bake cell (HillSpacing over the set's spacing in texels); 0 without hills or a set. */
    VOXELPLUGIN_API float HillTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set);

    /** The hill set's height cap (m): HillHeightScale x half HillSpacing. A set's relief beyond it
     *  is scaled down to it, so the planet's height range (ShapeFor) holds whatever set is read. */
    VOXELPLUGIN_API double HillSetReliefCap(const FVoxelTerrestrialSettings& Settings);

    /** The hills' detail's coarsest wavelength (m): the setting within 16 m .. a quarter of the
     *  hill spacing. */
    VOXELPLUGIN_API double HillDetailWavelength(const FVoxelTerrestrialSettings& Settings);

    /** The smallest hill spacing (m) the code accepts (the lattice's range, as PeakMinSpacing). */
    constexpr double HillMinSpacing = 500.0;

    /** How far the gullies can move the surface either way (cm). */
    VOXELPLUGIN_API double GullyReach(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake);

    /** The arc of one bake cell at the face centre (cm). */
    inline double CellArc(double SeaRadius, int32 N) { return UE_DOUBLE_PI * 0.5 * SeaRadius / FMath::Max(N, 1); }
}
