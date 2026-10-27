#include "VoxelLandBake.h"
#include "VoxelLandGPU.h"
#include "VoxelLandClimate.h"
#include "VoxelLandSolve.h"
#include "VoxelDensityGPU.h"
#include "VoxelGPUCommon.h"
#include "VoxelLandRelief.h"
#include "VoxelLandSea.h"
#include "VoxelLandRivers.h"
#include "VoxelLandRiverBuild.h"
#include "VoxelNoiseTiles.h"
#include "VoxelPlugin.h"
#include "VoxelShaderCompat.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RHIGPUReadback.h"
#include "Async/Async.h"
#include "Async/ParallelFor.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "Misc/App.h"
#include "Misc/Compression.h"
#include "Misc/FileHelper.h"
#include "Misc/Guid.h"
#include "Misc/Crc.h"
#include "Misc/CoreDelegates.h"
#include "Misc/Paths.h"
#include "Interfaces/IPluginManager.h"
#include "Math/RandomStream.h"
#include <atomic>

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
#include "VoxelCubeSphere.ush"

    /** The plate model on the CPU: VoxelLandPlates.ush with its inputs as members. */
    struct FVoxelLandPlatesCPU
    {
        const float* LandPlates = nullptr;
        uint LandPlateCount = 0;
        uint LandSeed = 0;
        float LandRadiusM = 1.0f;
        float LandWidthScale = 1.0f;
        float LandReliefScale = 1.0f;
        float LandShelfDepth = 0.0f;
        float LandAbyssDepth = 0.0f;
        float LandCoastHeight = 0.0f;
        float LandInteriorHeight = 0.0f;

#include "VoxelLandPlates.ush"
    };
}

static TAutoConsoleVariable<int32> CVarVoxelLandCache(
    TEXT("voxel.Land.Cache"), 1,
    TEXT("1: terrestrial bakes are loaded from and saved to Saved/VoxelLand (by a hash of the bake settings and the radius). 0: always bake."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandForceCpu(
    TEXT("voxel.Land.ForceCpu"), 0,
    TEXT("1: bake terrestrial planets on the CPU even when the GPU can (to compare the two)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandBakeCellsPerFrame(
    TEXT("voxel.Land.BakeCellsPerFrame"), 1 << 23,
    TEXT("Bake cells the GPU evaluates per frame (the plate model costs a few microseconds per cell on the CPU, far less on the GPU). The default does a 6 x 1024^2 bake's plate pass in two frames; lower it to spread a bake over more, shorter frames."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandErosion(
    TEXT("voxel.Land.Erosion"), 1,
    TEXT("1: terrestrial bakes run the erosion solve on the GPU (stage L2). 0: the plate model alone (for comparison; part of the cache key, so switching rebakes once)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandSolveStepCells(
    TEXT("voxel.Land.SolveStepCells"), 1 << 25,
    TEXT("The erosion solve's budget per frame: solved cells x steps (about 70 passes over the mountain cells per step; at most 16 steps a frame). The default does about 13 steps a frame at 6 x 1024^2; lower it for shorter frames during a bake."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandSolveIterationCells(
    TEXT("voxel.Land.SolveIterationCells"), 1 << 26,
    TEXT("The fills', drainage spread's and lake labels' budget per frame: cells x iterations (one cheap pass each), until they converge."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandSolveRouteAhead(
    TEXT("voxel.Land.SolveRouteAhead"), 1,
    TEXT("1: after the erosion solve, record the whole routing (lake levels, fill, drainage, lake labels, output) in one frame with fixed pass counts, checked when the fields come back (routing again by loops if a loop hadn't settled). 0: by loops, each waiting for its readback (several frames each)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugLand(
    TEXT("voxel.Debug.Land"), 0,
    TEXT("1: log the terrestrial bake's progress and timings."),
    ECVF_Default);

namespace VoxelLandCache
{
    bool IsUsable(const FVoxelLandBake& Bake);
}

namespace VoxelLandBakeDetail
{
    constexpr uint32 HistogramBins = 4096;     // VOXEL_LAND_HISTOGRAM_BINS (VoxelLandBake.usf)
    constexpr double HistogramMin = -2.0;
    constexpr double HistogramSpan = 4.0;
    constexpr double SolidWeight = 64.0;
    constexpr uint32 CacheMagic = 0x31424c56u;  // "VLB1"

    /** The plate model's inputs, the same for both backends. */
    struct FInputs
    {
        TArray<float> Plates;    // 8 per plate
        int32 N = 0;
        uint32 Seed = 0;
        uint32 PlateCount = 0;
        float RadiusM = 1.0f;
        float WidthScale = 1.0f;
        float ReliefScale = 1.0f;
        float ShelfDepth = 0.0f;
        float AbyssDepth = 0.0f;
        float CoastHeight = 0.0f;
        float InteriorHeight = 0.0f;
        float LandFraction = 0.3f;
        double SeaRadius = 0.0;  // cm
        uint32 Key = 0;
        uint32 SettingsCheck = 0;
    };

    /** Plates: seeds on a jittered Fibonacci sphere, Euler poles and rates, continental
     *  tendency and density (which plate subducts), from the seed alone. */
    TArray<float> MakePlates(int32 Count, uint32 Seed)
    {
        FRandomStream Random((int32)Seed);
        auto Gauss = [&Random]()
            {
                const double U1 = FMath::Max((double)Random.GetFraction(), 1e-12);
                const double U2 = (double)Random.GetFraction();
                return FMath::Sqrt(-2.0 * FMath::Loge(U1)) * FMath::Cos(2.0 * UE_DOUBLE_PI * U2);
            };
        TArray<float> Out;
        Out.SetNumZeroed(Count * 8);
        for (int32 i = 0; i < Count; i++)
        {
            const double Phi = FMath::Acos(1.0 - 2.0 * (i + 0.5) / Count);
            const double Theta = UE_DOUBLE_PI * (1.0 + FMath::Sqrt(5.0)) * (i + 0.5);
            FVector SeedDir(FMath::Cos(Theta) * FMath::Sin(Phi), FMath::Sin(Theta) * FMath::Sin(Phi), FMath::Cos(Phi));
            // One draw per statement: the order of function arguments is unspecified, and the
            // plates must be the same on every compiler.
            const double JitterX = Gauss();
            const double JitterY = Gauss();
            const double JitterZ = Gauss();
            SeedDir += FVector(JitterX, JitterY, JitterZ) * 0.25;
            SeedDir = SeedDir.GetSafeNormal();
            const double PoleX = Gauss();
            const double PoleY = Gauss();
            const double PoleZ = Gauss();
            FVector Pole(PoleX, PoleY, PoleZ);
            Pole = Pole.GetSafeNormal();
            if (Pole.IsNearlyZero()) Pole = FVector::UpVector;
            const double Rate = 0.5 + (double)Random.GetFraction();
            const double Bias = -1.0 + 2.0 * (double)Random.GetFraction();
            const double Density = (double)Random.GetFraction();
            for (int32 Axis = 0; Axis < 3; Axis++)
            {
                Out[i * 8 + Axis] = (float)SeedDir[Axis];
                Out[i * 8 + 4 + Axis] = (float)(Pole[Axis] * Rate);
            }
            Out[i * 8 + 3] = (float)Bias;
            Out[i * 8 + 7] = (float)Density;
        }
        return Out;
    }

    FInputs MakeInputs(const FVoxelTerrestrialSettings& Settings, double SeaRadius, uint32 Key)
    {
        FInputs In;
        In.N = VoxelLand::BakeResolutionFor(Settings);
        In.Seed = (uint32)Settings.Seed;
        In.PlateCount = (uint32)FMath::Clamp(Settings.PlateCount, 2, 64);
        In.Plates = MakePlates((int32)In.PlateCount, In.Seed);
        In.RadiusM = (float)(SeaRadius * 0.01);
        In.WidthScale = (float)FMath::Clamp(Settings.LandformWidthScale, 0.1, 4.0);
        In.ReliefScale = (float)FMath::Clamp(Settings.RangeRelief, 0.0, 4.0);
        In.ShelfDepth = (float)Settings.ShelfDepth;
        In.AbyssDepth = (float)Settings.AbyssDepth;
        In.CoastHeight = (float)Settings.CoastHeight;
        In.InteriorHeight = (float)Settings.InteriorHeight;
        In.LandFraction = (float)FMath::Clamp(Settings.ContinentalFraction, 0.02, 0.98);
        In.SeaRadius = SeaRadius;
        In.Key = Key;
        In.SettingsCheck = VoxelLand::BakeCheck(Settings, SeaRadius);
        return In;
    }

    VoxelShader::FVoxelLandPlatesCPU MakePlatesCPU(const FInputs& In)
    {
        VoxelShader::FVoxelLandPlatesCPU P;
        P.LandPlates = In.Plates.GetData();
        P.LandPlateCount = In.PlateCount;
        P.LandSeed = In.Seed;
        P.LandRadiusM = In.RadiusM;
        P.LandWidthScale = In.WidthScale;
        P.LandReliefScale = In.ReliefScale;
        P.LandShelfDepth = In.ShelfDepth;
        P.LandAbyssDepth = In.AbyssDepth;
        P.LandCoastHeight = In.CoastHeight;
        P.LandInteriorHeight = In.InteriorHeight;
        return P;
    }

    /** The threshold that leaves LandFraction of the histogram's weight above it (ThresholdCS). */
    float ThresholdFromHistogram(const TArray<uint64>& Histogram, float LandFraction)
    {
        uint64 Total = 0;
        for (const uint64 H : Histogram)
            Total += H;
        const double Target = (1.0 - (double)LandFraction) * (double)Total;
        uint64 Cumulative = 0;
        for (int32 b = 0; b < Histogram.Num(); b++)
        {
            const uint64 H = Histogram[b];
            if (H > 0 && (double)(Cumulative + H) >= Target)
            {
                const double Inside = FMath::Clamp((Target - (double)Cumulative) / (double)H, 0.0, 1.0);
                return (float)(HistogramMin + HistogramSpan * ((double)b + Inside) / (double)HistogramBins);
            }
            Cumulative += H;
        }
        return (float)(HistogramMin + HistogramSpan);
    }

    TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> MakeBake(const FInputs& In)
    {
        const TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = MakeShared<FVoxelLandBake, ESPMode::ThreadSafe>();
        Bake->N = In.N;
        Bake->SeaRadius = In.SeaRadius;
        Bake->Key = In.Key;
        Bake->SettingsCheck = In.SettingsCheck;
        Bake->BuildTanTable();
        return Bake;
    }

    /** The whole bake on the CPU (the GPU passes' code compiled as C++), at background priority
     *  so the game's task workers stay free. Null when cancelled. */
    TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> BakeOnCpu(const FInputs& In, std::atomic<int64>* Progress, const std::atomic<bool>* Cancel)
    {
        const VoxelShader::FVoxelLandPlatesCPU Plates = MakePlatesCPU(In);
        const int32 N = In.N;
        const int32 PerFace = N * N;

        auto Cancelled = [Cancel]() { return Cancel && Cancel->load(std::memory_order_relaxed); };

        // Crust histogram over the interior cells, by solid angle (one histogram per 64k
        // cells: a few MB of partial histograms at most).
        constexpr int32 HistogramChunk = 1 << 16;
        const int32 NumInterior = 6 * PerFace;
        const int32 NumHistogramChunks = FMath::DivideAndRoundUp(NumInterior, HistogramChunk);
        TArray<TArray<uint64>> Partial;
        Partial.SetNum(NumHistogramChunks);
        ParallelFor(NumHistogramChunks, [&](int32 c)
            {
                TArray<uint64>& H = Partial[c];
                H.SetNumZeroed(HistogramBins);
                if (Cancelled()) return;
                VoxelShader::FVoxelLandPlatesCPU P = Plates;
                for (int32 Index = c * HistogramChunk; Index < FMath::Min(NumInterior, (c + 1) * HistogramChunk); Index++)
                {
                    const uint32 Face = (uint32)(Index / PerFace);
                    const int32 Rest = Index - (int32)Face * PerFace;
                    const int32 J = Rest / N;
                    const int32 I = Rest - J * N;
                    const float S = VoxelShader::VoxelCubeCellCentre(I, (uint32)N);
                    const float T = VoxelShader::VoxelCubeCellCentre(J, (uint32)N);
                    const VoxelShader::float3 Dir = VoxelShader::VoxelCubeDirection(Face, S, T);
                    const float Raw = P.VoxelLandRawCrust(Dir, P.VoxelLandNearestPlate(P.VoxelLandWarp(Dir)));
                    const float A = VoxelShader::tan(S * VOXEL_CUBE_QUARTER_PI);
                    const float B = VoxelShader::tan(T * VOXEL_CUBE_QUARTER_PI);
                    const float Q = 1.0f + A * A + B * B;
                    const float Solid = (1.0f + A * A) * (1.0f + B * B) / (Q * VoxelShader::sqrt(Q));
                    const int32 Bin = FMath::Clamp((int32)VoxelShader::floor((Raw - (float)HistogramMin) / (float)HistogramSpan * (float)HistogramBins), 0, (int32)HistogramBins - 1);
                    H[Bin] += (uint64)(Solid * (float)SolidWeight + 0.5f);
                }
                if (Progress) Progress->fetch_add(HistogramChunk / 4, std::memory_order_relaxed);
            }, EParallelForFlags::BackgroundPriority);
        if (Cancelled()) return nullptr;
        TArray<uint64> Histogram;
        Histogram.SetNumZeroed(HistogramBins);
        for (const TArray<uint64>& H : Partial)
            for (uint32 b = 0; b < HistogramBins; b++)
                Histogram[b] += H[b];
        const float Threshold = ThresholdFromHistogram(Histogram, In.LandFraction);

        const TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = MakeBake(In);
        Bake->CrustThreshold = Threshold;
        const int32 Side = Bake->Side();
        const int32 NumCells = Bake->NumCells();
        Bake->Height.SetNumUninitialized(NumCells);
        Bake->Attr.SetNumUninitialized(NumCells);
        Bake->Form.SetNumUninitialized(NumCells);
        Bake->Province.SetNumUninitialized(NumCells);
        constexpr int32 Chunk = 4096;
        const int32 NumCellChunks = FMath::DivideAndRoundUp(NumCells, Chunk);
        ParallelFor(NumCellChunks, [&](int32 c)
            {
                if (Cancelled()) return;
                VoxelShader::FVoxelLandPlatesCPU P = Plates;
                for (int32 Index = c * Chunk; Index < FMath::Min(NumCells, (c + 1) * Chunk); Index++)
                {
                    const uint32 Face = (uint32)(Index / (Side * Side));
                    const int32 Rest = Index - (int32)Face * Side * Side;
                    const int32 J = Rest / Side - FVoxelLandBake::Pad;
                    const int32 I = Rest % Side - FVoxelLandBake::Pad;
                    const VoxelShader::float3 Dir = VoxelShader::VoxelCubeDirection(Face,
                        VoxelShader::VoxelCubeCellCentre(I, (uint32)N), VoxelShader::VoxelCubeCellCentre(J, (uint32)N));
                    const VoxelShader::FVoxelLandPlatesCPU::FVoxelLandElevation E = P.VoxelLandElevationAt(Dir, Threshold);
                    Bake->Height[Index] = E.Base + E.Up - E.Down;
                    Bake->Attr[Index] = (uint16)P.VoxelLandPackAttr(E.Crust, E.Class);
                    Bake->Form[Index] = (uint16)P.VoxelLandPackForm(P.VoxelLandFormAt(Dir, E));
                    Bake->Province[Index] = (uint8)P.VoxelLandPackProvince(P.VoxelLandProvinceAt(Dir, E));
                }
                if (Progress) Progress->fetch_add(Chunk, std::memory_order_relaxed);
            }, EParallelForFlags::BackgroundPriority);
        if (Cancelled()) return nullptr;
        Bake->UpdateRange();
        Bake->BuildRelief();
        Bake->BuildProvinceStats();
        Bake->BuildSeaConnected();
        // Checked here, on the worker (a scan of every cell).
        if (!VoxelLandCache::IsUsable(*Bake)) return nullptr;
        return Bake;
    }
}

namespace VLB = VoxelLandBakeDetail;

// ---------------------------------------------------------------------------------------------
// Settings
// ---------------------------------------------------------------------------------------------

uint32 FVoxelTerrestrialSettings::GetBakeHash() const
{
    uint32 H = ::GetTypeHash(Seed);
    H = HashCombine(H, ::GetTypeHash(PlateCount));
    H = HashCombine(H, ::GetTypeHash(BakeResolution));
    for (const double V : { ContinentalFraction, LandformWidthScale, RangeRelief, CoastHeight, InteriorHeight, ShelfDepth, AbyssDepth,
        PlainsUpliftThreshold, MountainRampWidth, SolveResidual, MinLakeDepth, MinLakeArea, RoutingJitter, UpliftVariation,
        ErodibilityVariation, VariationWavelength, InitialRoughness, PlainsRelief, RiverIncision, SummitHeightScale,
        RoutingRandomness, DrainageSpreadCells, PlainsUndulation, LandformErosion, ChannelHeadCells, ChannelAreaVariation,
        ChannelAreaVariationWavelength, HillslopeSteepness, ChannelCharacterInfluence })
        H = HashCombine(H, ::GetTypeHash(V));
    H = HashCombine(H, ::GetTypeHash(SolveMaxSteps));
    H = HashCombine(H, ::GetTypeHash(ChannelAreaVariationOctaves));
    return H;
}

uint32 FVoxelTerrestrialSettings::GetSettingsHash() const
{
    uint32 H = GetBakeHash();
    for (const double V : { BakeWarp, DetailAmplitude, DetailWavelength,
        GullyStrength, GullyFinestWavelength, GullyFlowGain, GullyDetailScale, HillRelief, HillWavelength, HillFinestWavelength,
        PeakHeightScale, PeakSpacing, PeakDetailStrength, PeakDetailWavelength, HillHeightScale, HillSpacing, HillSwellHeight,
        HillSwellScale, HillDetailStrength, HillDetailWavelength, HillFoothillGrowth, PeakReliefFadeStart, PeakReliefFadeEnd,
        StrataHeightScale, StrataSpacing, StrataCoverage, StrataSwitchWidth, StrataFlatShare, StrataDetailStrength, StrataDetailWavelength,
        RiverMinArea, RiverWidthScale, ValleyWidthScale, RiverDepthScale, FloodplainScale, LakeShoreBand, RiverMeander, RiverFalls })
        H = HashCombine(H, ::GetTypeHash(V));
    H = HashCombine(H, ::GetTypeHash(bCarveRivers));
    // FString's hash is found by argument-dependent lookup only (not at global scope).
    const FString NoiseSetName = PeakNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(NoiseSetName));
    const FString HillSetName = HillNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(HillSetName));
    const FString StrataSetName = StrataNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(StrataSetName));
    const FString DetailSetName = DetailNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(DetailSetName));
    // T5c: the dunes.
    for (const double V : { DuneHeightScale, DuneSpacing, DuneErgCoverage, DuneErgSize, DuneMaxRelief, SandDetailWavelength, SandDetailStrength })
        H = HashCombine(H, ::GetTypeHash(V));
    // T5d: the glacial layer.
    for (const double V : { GlacialHeightScale, GlacialSpacing, GlacialEraCooling })
        H = HashCombine(H, ::GetTypeHash(V));
    const FString GlacialSetName = GlacialNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(GlacialSetName));
    // T5d2: the ice sheets.
    for (const double V : { IceSheetMaxThickness, IceSheetSwellHeight, IceSheetSwellSpacing })
        H = HashCombine(H, ::GetTypeHash(V));
    const FString IceSetName = IceSheetNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(IceSetName));
    // T5d3: the polar surfaces.
    for (const double V : { CrevasseSpacing, CrevasseHeightScale, SastrugiSpacing, SastrugiHeightScale, PolygonSpacing, PolygonHeightScale,
        PermafrostTemperature, SnowDuneSpacing, SnowDuneHeight })
        H = HashCombine(H, ::GetTypeHash(V));
    const FString PolarSetName = PolarNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(PolarSetName));
    const FString DuneSetName = DuneNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(DuneSetName));
    const FString SandSetName = SandDetailNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(SandSetName));
    // T5e: the ice shelf.
    for (const double V : { IceShelfHeight, IceShelfTemperature, IceShelfFrontNoise, IceShelfRise, IceShelfRoll, IceShelfDraft })
        H = HashCombine(H, ::GetTypeHash(V));
    return H;
}

// ---------------------------------------------------------------------------------------------
// FVoxelLandBake
// ---------------------------------------------------------------------------------------------

FVoxelLandBake::FVoxelLandBake()
    : GPU(MakeShared<FVoxelLandBakeGPU, ESPMode::ThreadSafe>())
{
}

FVoxelLandBake::~FVoxelLandBake()
{
    // The GPU copy (and a river build under way) are released on the render thread, which made
    // them.
    if (GPU.IsValid() || RiverJob.IsValid())
    {
        TSharedPtr<FVoxelLandBakeGPU, ESPMode::ThreadSafe> Resource = MoveTemp(GPU);
        TSharedPtr<FVoxelLandRiverJob, ESPMode::ThreadSafe> Job = MoveTemp(RiverJob);
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseBake)([Resource, Job](FRHICommandListImmediate&) mutable
            {
                Resource.Reset();
                Job.Reset();
            });
    }
}

void FVoxelLandBake::BuildTanTable()
{
    const int32 S = Side();
    TanTable.SetNumUninitialized(S);
    for (int32 i = 0; i < S; i++)
    {
        const double Centre = -1.0 + (2.0 * (double)(i - Pad) + 1.0) / (double)N;
        TanTable[i] = VoxelDoubleFloat(FMath::Tan(Centre * UE_DOUBLE_PI * 0.25));
    }
}

void FVoxelLandBake::UpdateRange()
{
    MinHeight = 0.0f;
    MaxHeight = 0.0f;
    if (Height.Num() == 0) return;
    MinHeight = Height[0];
    MaxHeight = Height[0];
    for (const float H : Height)
    {
        MinHeight = FMath::Min(MinHeight, H);
        MaxHeight = FMath::Max(MaxHeight, H);
    }
}

void FVoxelLandBake::BuildRelief()
{
    const int32 S = Side();
    Relief.SetNumZeroed(Height.Num());
    MaxRelief = 0.0f;
    if (Height.Num() != NumCells()) return;
    ParallelFor(6 * S, [this, S](int32 Row)
        {
            VoxelLandRelief::BuildRow(Height.GetData(), S, Row / S, Row % S, Relief.GetData());
        }, EParallelForFlags::BackgroundPriority);
    uint8 Max = 0;
    for (const uint8 Q : Relief)
        Max = FMath::Max(Max, Q);
    MaxRelief = VoxelLandRelief::Decode(Max);
}

void FVoxelLandBake::BuildProvinceStats()
{
    ProvinceLand.Init(0u, ProvinceLevels);
    if (Province.Num() != NumCells() || Height.Num() != NumCells()) return;
    for (int32 i = 0; i < Province.Num(); i++)
        if (Height[i] > 0.0f)
            ProvinceLand[FMath::Min((int32)Province[i], ProvinceLevels - 1)]++;
}

void FVoxelLandBake::BuildSeaConnected()
{
    SeaCells = 0;
    SeaRegions = 0;
    BasinRegions = 0;
    if (N <= 0 || Height.Num() != NumCells() || Attr.Num() != NumCells()) return;
    // (Stage W5m: with the kept lakes' levels, a lagoon at the sea's level is sea.)
    const VoxelLandSea::FStats Stats = VoxelLandSea::Mark(N, Pad, Height.GetData(), Attr.GetData(), SeaBit, SeaMinShare, 3,
        Water.Num() == NumCells() ? Water.GetData() : nullptr, SeaSpillLevel);
    SeaCells = Stats.SeaCells;
    SeaRegions = Stats.SeaRegions;
    BasinRegions = Stats.BasinRegions;
}

float FVoxelLandBake::ProvinceThreshold(double Coverage) const
{
    if (!(Coverage > 0.0) || ProvinceLand.Num() != ProvinceLevels) return 2.0f;
    uint64 Total = 0;
    for (const uint32 C : ProvinceLand) Total += C;
    if (Total == 0) return 2.0f;
    // From the top level down: level q's cells spread over [q - 1/2, q + 1/2] / (levels - 1).
    const double Wanted = FMath::Min(Coverage, 1.0) * (double)Total;
    double Above = 0.0;
    const double Step = 1.0 / (double)(ProvinceLevels - 1);
    for (int32 q = ProvinceLevels - 1; q >= 0; q--)
    {
        const double Count = (double)ProvinceLand[q];
        if (Above + Count >= Wanted && Count > 0.0)
            return (float)(((double)q + 0.5 - (Wanted - Above) / Count) * Step);
        Above += Count;
    }
    return (float)(-0.5 * Step);
}

// ---------------------------------------------------------------------------------------------
// The peaks' noise set
// ---------------------------------------------------------------------------------------------

FVoxelLandNoiseSet::FVoxelLandNoiseSet()
    : GPU(MakeShared<FVoxelLandNoiseGPU, ESPMode::ThreadSafe>())
{
}

FVoxelLandRiverSet::FVoxelLandRiverSet() = default;

FVoxelLandRiverSet::~FVoxelLandRiverSet()
{
    if (GPU.IsValid())
    {
        TSharedPtr<FVoxelLandRiverGPU, ESPMode::ThreadSafe> Resource = MoveTemp(GPU);
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseRivers)([Resource](FRHICommandListImmediate&) mutable
            {
                Resource.Reset();
            });
    }
}

uint32 VoxelLand::RiverKey(const FVoxelTerrestrialSettings& Settings)
{
    uint32 H = 0x52495652u;
    for (const double V : { Settings.RiverMinArea, Settings.RiverWidthScale, Settings.ValleyWidthScale })
        H = HashCombine(H, ::GetTypeHash(V));
    return H;
}

double VoxelLand::RiverReachM(const FVoxelTerrestrialSettings& Settings, float Log2Area)
{
    return VoxelLandRivers::ReachScale * VoxelLandRivers::ValleyHalfWidthM(VoxelLandRivers::BeltM(Log2Area), FMath::Max(Settings.ValleyWidthScale, 0.1));
}

FVoxelLandNoiseSet::~FVoxelLandNoiseSet()
{
    if (GPU.IsValid())
    {
        TSharedPtr<FVoxelLandNoiseGPU, ESPMode::ThreadSafe> Resource = MoveTemp(GPU);
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseNoise)([Resource](FRHICommandListImmediate&) mutable
            {
                Resource.Reset();
            });
    }
}

const TCHAR* VoxelLand::GeneratedNoiseName = TEXT("generated");

namespace VoxelLandNoiseDetail
{
    // The generated set's seed: one set for every planet (each planet offsets and turns it by its
    // own seed, VoxelLandPeaks.ush).
    constexpr uint32 PeakSetSeed = 0x7065616bu;
    // The generated detail set (N1: the stand-in when the folder has no `detail*` sets),
    // likewise one for every planet.
    constexpr uint32 DetailSetSeed = 0x64657461u;
    constexpr uint32 DetailStandInN = 1024;
    // The rows a pack may take (the material's texture holds them: a fixed limit, not this
    // RHI's, so every machine agrees on what the geometry has).
    constexpr uint64 MaxRows = 16384;

    /** A pack by key. The densities that sample it hold it; the registry only watches it (and
     *  keeps the few most recent alive: Recent), so a pack nobody uses any more goes, with its
     *  GPU copy, and is made again if asked for. */
    struct FEntry
    {
        TWeakPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Set;
        bool bReady = false;
        bool bNone = false;       // ready, and none could be made
        bool bPending = false;
    };

    // The packs kept alive whatever uses them: those just made, so a planet that waited for one
    // finds it when it builds its density.
    constexpr int32 RecentCount = 4;

    FCriticalSection Lock;        // Entries, Recent, the folder's list, the flags (held briefly)
    FCriticalSection MakeLock;    // one pack made at a time (held while reading or generating)
    TMap<FString, FEntry> Entries;
    TArray<TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>> Recent;
    FString Directory;            // NoiseSetDirectory, found on the game thread (StartupModule)
    bool bDirectoryKnown = false;
    TArray<FString> AuthoredPeaks;
    TArray<FString> AuthoredHills;
    TArray<FString> AuthoredStrata;
    TArray<FString> AuthoredDetail;
    TArray<FString> AuthoredDunes;    // T5c
    TArray<FString> AuthoredSand;
    TArray<FString> AuthoredGlacial;  // T5d
    TArray<FString> AuthoredIce;      // T5d2
    TArray<FString> AuthoredPolar;    // T5d3
    bool bListed = false;
    bool bExitHooked = false;
    bool bClosed = false;         // ReleaseLandNoise ran (the engine is exiting)

    /** Appends a chain (Texels: ChainTexels(N) texels, 4 x u16 each) to Out's words from a whole
     *  row; its first texel, or ~0 if the rows would pass MaxRows (MakeLock held). */
    uint32 AppendChainLocked(FVoxelLandNoiseSet& Out, const void* Texels, uint32 N)
    {
        const uint64 Count = VoxelNoiseTiles::ChainTexels(N);
        const uint32 Rows = (uint32)((Count + Out.RowN - 1) / Out.RowN);
        if ((uint64)Out.Rows + Rows > MaxRows) return ~0u;
        const uint32 Base = Out.Rows * Out.RowN;
        const int32 Old = Out.Words.Num();
        Out.Words.AddZeroed((int32)(2 * (uint64)Rows * Out.RowN));
        FMemory::Memcpy(Out.Words.GetData() + Old, Texels, Count * 8);
        Out.Rows += Rows;
        return Base;
    }

    /** A set as read or generated: its checked header (CheckPeaks: the eroded layout, R and G
     *  heights, B blurred, A flow, which the hills' and strata's sets share; CheckDetail: the
     *  detail sets') and its chain's texels. */
    struct FRead
    {
        VoxelNoiseTiles::FSet Header;
        TArray<uint32> Texels;    // ChainTexels(N) texels, two words each
    };

    /** The plugin's own `peaks` set. */
    bool Generate(FRead& Out)
    {
        const double Start = FPlatformTime::Seconds();
        VoxelNoiseTiles::FSet Set = VoxelNoiseTiles::GeneratePeaksStandIn(PeakSetSeed, 1024, [](int32 Count, auto&& Body)
            {
                ParallelFor(Count, [&Body](int32 i) { Body(i); }, EParallelForFlags::BackgroundPriority);
            });
        Set.Name = TCHAR_TO_UTF8(VoxelLand::GeneratedNoiseName);
        const std::string Error = VoxelNoiseTiles::CheckPeaks(Set);
        if (!Error.empty() || !Set.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The generated peaks' noise set is malformed (%s); the peaks are off."), UTF8_TO_TCHAR(Error.c_str()));
            return false;
        }
        Out.Texels.SetNumUninitialized((int32)(2 * VoxelNoiseTiles::ChainTexels(Set.N)));
        FMemory::Memcpy(Out.Texels.GetData(), Set.Texels.data(), Set.Texels.size() * sizeof(uint16));
        Set.Texels.clear();
        Out.Header = MoveTemp(Set);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Generated the peaks' noise set (%u^2, %u mips) in %.2f s."), Out.Header.N, Out.Header.Mips,
            FPlatformTime::Seconds() - Start);
        return true;
    }

    /** The plugin's own detail set (N1: VoxelNoiseTiles::GenerateDetailStandIn), in the authored
     *  detail sets' layout: when the folder has none, or the one picked can't be used. */
    bool GenerateDetail(FRead& Out)
    {
        const double Start = FPlatformTime::Seconds();
        VoxelNoiseTiles::FSet Set = VoxelNoiseTiles::GenerateDetailStandIn(DetailSetSeed, DetailStandInN, [](int32 Count, auto&& Body)
            {
                ParallelFor(Count, [&Body](int32 i) { Body(i); }, EParallelForFlags::BackgroundPriority);
            });
        Set.Name = TCHAR_TO_UTF8(VoxelLand::GeneratedNoiseName);
        const std::string Error = VoxelNoiseTiles::CheckDetail(Set);
        if (!Error.empty() || !Set.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The generated detail set is malformed (%s); the detail is off."), UTF8_TO_TCHAR(Error.c_str()));
            return false;
        }
        Out.Texels.SetNumUninitialized((int32)(2 * VoxelNoiseTiles::ChainTexels(Set.N)));
        FMemory::Memcpy(Out.Texels.GetData(), Set.Texels.data(), Set.Texels.size() * sizeof(uint16));
        Set.Texels.clear();
        Out.Header = MoveTemp(Set);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Generated the detail set (%u^2, %u mips) in %.2f s."), Out.Header.N, Out.Header.Mips,
            FPlatformTime::Seconds() - Start);
        return true;
    }

    /** The layouts a set is checked for (CheckPeaks: the peaks', hills' and strata's sets). */
    enum class ESetKind : uint8
    {
        Peaks,
        Detail,
        Dunes,      // T5c
        Sand,
        Glacial,    // T5d: the peaks' layout (CheckPeaks), read from the 1024 mip
        Ice,        // T5d2: likewise
        Polar       // T5d3: CheckPolar, read from the 1024 mip
    };

    std::string CheckKind(const VoxelNoiseTiles::FSet& Set, ESetKind Kind)
    {
        switch (Kind)
        {
        case ESetKind::Detail: return VoxelNoiseTiles::CheckDetail(Set);
        case ESetKind::Dunes: return VoxelNoiseTiles::CheckDunes(Set);
        case ESetKind::Sand: return VoxelNoiseTiles::CheckSandDetail(Set);
        case ESetKind::Glacial: return VoxelNoiseTiles::CheckPeaks(Set);
        case ESetKind::Ice: return VoxelNoiseTiles::CheckPeaks(Set);
        case ESetKind::Polar: return VoxelNoiseTiles::CheckPolar(Set);
        default: return VoxelNoiseTiles::CheckPeaks(Set);
        }
    }

    /** T5c: Set's header made its first mip no larger than MaxN's (0: unchanged): N, the mips, and
     *  the height channels' decode in that mip's texels. The mips skipped. */
    uint32 SkipHeaderMips(VoxelNoiseTiles::FSet& Set, uint32 MaxN)
    {
        uint32 Skip = 0;
        while (MaxN > 0 && (Set.N >> Skip) > MaxN && Skip + 1 < Set.Mips)
            Skip++;
        if (Skip == 0) return 0;
        const float Scale = 1.0f / (float)(1u << Skip);
        Set.N >>= Skip;
        Set.Mips -= Skip;
        for (VoxelNoiseTiles::FChannel& C : Set.Channel)
        {
            if (C.Kind != VoxelNoiseTiles::KindErodedHeight && C.Kind != VoxelNoiseTiles::KindBlurred) continue;
            C.Min *= Scale;
            C.Max *= Scale;
            C.Mean *= Scale;
            C.Std *= Scale;
        }
        return Skip;
    }

    /** The largest mip a kind's sets are read from (0: all). */
    uint32 MaxNOf(ESetKind Kind)
    {
        return Kind == ESetKind::Dunes || Kind == ESetKind::Sand || Kind == ESetKind::Glacial || Kind == ESetKind::Ice || Kind == ESetKind::Polar
            ? VoxelLand::DuneMaxSetN : 0u;
    }

    /** An authored set from NoiseSetDirectory()/Name.vxnt: the header checked (CheckKind), then the
     *  texels read straight into the words; MaxN (T5c; 0: all) reads the chain from its first mip no
     *  larger than that (the header made that mip's: N, the mips, and the height channels' decode
     *  in its texels; checked again). False (and Error) if it can't be used. */
    bool Read(const FString& Name, FRead& Out, FString& Error, ESetKind Kind = ESetKind::Peaks, uint32 MaxN = 0)
    {
        const double Start = FPlatformTime::Seconds();
        const FString Folder = VoxelLand::NoiseSetDirectory();
        if (Folder.IsEmpty())
        {
            Error = TEXT("the plugin's NoiseSets folder wasn't found");
            return false;
        }
        const FString Path = FPaths::Combine(Folder, Name + TEXT(".vxnt"));
        const TUniquePtr<FArchive> File(IFileManager::Get().CreateFileReader(*Path));
        if (!File.IsValid())
        {
            Error = FString::Printf(TEXT("%s isn't there"), *Path);
            return false;
        }
        const int64 FileSize = File->TotalSize();
        if (FileSize <= 0)
        {
            Error = FString::Printf(TEXT("%s is empty or unreadable"), *Path);
            return false;
        }
        TArray<uint8> Header;
        Header.SetNumZeroed((int32)FMath::Min<int64>(FileSize, 4096));
        File->Serialize(Header.GetData(), Header.Num());
        VoxelNoiseTiles::FSet Set;
        uint64_t DataOffset = 0;
        std::string Why = VoxelNoiseTiles::ReadVxntHeader(Header.GetData(), (size_t)Header.Num(), (uint64_t)FileSize, Set, DataOffset);
        if (Why.empty())
            Why = CheckKind(Set, Kind);
        // T5c: the mips skipped (each halves the size; heights are in the read mip's texels).
        const uint64 FullTexels = VoxelNoiseTiles::ChainTexels(Set.N);
        const uint32 Skip = Why.empty() ? SkipHeaderMips(Set, MaxN) : 0u;
        const uint64 SkippedTexels = FullTexels - VoxelNoiseTiles::ChainTexels(Set.N);
        if (Skip > 0)
            Why = CheckKind(Set, Kind);
        if (!Why.empty() || File->IsError())
        {
            Error = FString::Printf(TEXT("%s: %s"), *Path, Why.empty() ? TEXT("unreadable") : UTF8_TO_TCHAR(Why.c_str()));
            return false;
        }
        const uint64 Texels = VoxelNoiseTiles::ChainTexels(Set.N);
        Out.Texels.SetNumUninitialized((int32)(2 * Texels));
        // The texels as they lie (u16 little-endian, R G B A: two words a texel).
        File->Seek((int64)(DataOffset + SkippedTexels * 8));
        File->Serialize(Out.Texels.GetData(), (int64)(Texels * 8));
        if (File->IsError())
        {
            Error = FString::Printf(TEXT("%s: the texels couldn't be read"), *Path);
            return false;
        }
        Out.Header = MoveTemp(Set);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Read the noise set %s (%u^2%s, %u mips, \"%s\") in %.2f s."), *Name, Out.Header.N,
            Skip > 0 ? *FString::Printf(TEXT(" from its mip %u"), Skip) : TEXT(""), Out.Header.Mips,
            UTF8_TO_TCHAR(Out.Header.LookTag.c_str()), FPlatformTime::Seconds() - Start);
        return true;
    }

    /** A channel's unit value of V (0..1 over its decode range: what the shader sees). */
    double UnitOf(const VoxelNoiseTiles::FSet& Set, int Channel, double V)
    {
        const VoxelNoiseTiles::FChannel& C = Set.Channel[Channel];
        return (V - (double)C.Min) / FMath::Max((double)C.Max - (double)C.Min, 1e-12);
    }

    /** Makes the pack Key ("Peaks|Hills|Strata|Detail|Dunes|SandDetail"; MakeLock held): the
     *  peaks' set from texel 0 (read, or generated: also when the file can't be used), the detail
     *  set (N1: read, or generated likewise), the hills' set (left out when it can't be used: the
     *  procedural hills then), the strata's (T4c; left out when it can't be used, or without the
     *  hills' set), the dunes' and the sand detail's (T5c; likewise, from their mip no larger than
     *  DuneMaxSetN; the sand without the dunes is left out). Null if it holds nothing. */
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> MakePackLocked(const FString& Key)
    {
        TArray<FString> Parts;
        Key.ParseIntoArray(Parts, TEXT("|"), false);
        const FString PeakName = Parts.Num() > 0 ? Parts[0] : FString();
        const FString HillName = Parts.Num() > 1 ? Parts[1] : FString();
        const FString StrataName = Parts.Num() > 2 ? Parts[2] : FString();
        const FString DetailName = Parts.Num() > 3 ? Parts[3] : FString();
        const FString DuneName = Parts.Num() > 4 ? Parts[4] : FString();
        const FString SandName = Parts.Num() > 5 ? Parts[5] : FString();
        const FString GlacialName = Parts.Num() > 6 ? Parts[6] : FString();
        const FString IceName = Parts.Num() > 7 ? Parts[7] : FString();
        const FString PolarName = Parts.Num() > 8 ? Parts[8] : FString();
        FRead Peaks, Hills, Strata, Detail, Dunes, Sand, Glacial, Ice, Polar;
        bool bPeaks = false, bHills = false, bStrata = false, bDetail = false, bDunes = false, bSand = false, bGlacial = false, bIce = false;
        bool bPolar = false;
        if (!PeakName.IsEmpty())
        {
            FString Error;
            bPeaks = PeakName != VoxelLand::GeneratedNoiseName && Read(PeakName, Peaks, Error);
            if (!bPeaks)
            {
                if (PeakName != VoxelLand::GeneratedNoiseName)
                    UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The peaks' noise set %s can't be used (%s); the generated set stands in."), *PeakName, *Error);
                bPeaks = Generate(Peaks);
            }
        }
        if (!HillName.IsEmpty())
        {
            FString Error;
            bHills = Read(HillName, Hills, Error);
            if (!bHills)
                UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The hills' noise set %s can't be used (%s); the procedural hills stand in."), *HillName, *Error);
        }
        if (!StrataName.IsEmpty() && bHills)
        {
            FString Error;
            bStrata = Read(StrataName, Strata, Error);
            if (!bStrata)
                UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The strata's noise set %s can't be used (%s); the hills cover the sedimentary provinces too."), *StrataName, *Error);
        }
        FString DetailRead = DetailName;
        if (!DetailName.IsEmpty())
        {
            FString Error;
            bDetail = DetailName != VoxelLand::GeneratedNoiseName && Read(DetailName, Detail, Error, ESetKind::Detail);
            if (!bDetail)
            {
                if (DetailName != VoxelLand::GeneratedNoiseName)
                    UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The detail noise set %s can't be used (%s); the generated set stands in."), *DetailName, *Error);
                DetailRead = VoxelLand::GeneratedNoiseName;
                bDetail = GenerateDetail(Detail);
            }
        }
        if (!DuneName.IsEmpty() && bHills)
        {
            FString Error;
            bDunes = Read(DuneName, Dunes, Error, ESetKind::Dunes, MaxNOf(ESetKind::Dunes));
            if (!bDunes)
                UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The dunes' noise set %s can't be used (%s); no dunes."), *DuneName, *Error);
        }
        if (!SandName.IsEmpty() && bDunes)
        {
            FString Error;
            bSand = Read(SandName, Sand, Error, ESetKind::Sand, MaxNOf(ESetKind::Sand));
            if (!bSand)
                UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The sand's detail set %s can't be used (%s); the dunes go without it."), *SandName, *Error);
        }
        if (!GlacialName.IsEmpty() && bPeaks)
        {
            FString Error;
            bGlacial = Read(GlacialName, Glacial, Error, ESetKind::Glacial, MaxNOf(ESetKind::Glacial));
            if (!bGlacial)
                UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The glacial noise set %s can't be used (%s); the peaks everywhere."), *GlacialName, *Error);
        }
        if (!IceName.IsEmpty())
        {
            FString Error;
            bIce = Read(IceName, Ice, Error, ESetKind::Ice, MaxNOf(ESetKind::Ice));
            if (!bIce)
                UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The ice sheets' noise set %s can't be used (%s); smooth sheets."), *IceName, *Error);
        }
        if (!PolarName.IsEmpty())
        {
            FString Error;
            bPolar = Read(PolarName, Polar, Error, ESetKind::Polar, MaxNOf(ESetKind::Polar));
            if (!bPolar)
                UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The polar surfaces' noise set %s can't be used (%s); no polar surfaces."), *PolarName, *Error);
        }
        if (!bPeaks && !bHills && !bDetail) return nullptr;
        const TSharedPtr<FVoxelLandNoiseSet, ESPMode::ThreadSafe> Out = MakeShared<FVoxelLandNoiseSet, ESPMode::ThreadSafe>();
        Out->Name = Key;
        // The rows' width: the widest chain's (a small set beside a large one would otherwise
        // take far more rows).
        Out->RowN = FMath::Max(FMath::Max3(bPeaks ? Peaks.Header.N : 0u, bHills ? Hills.Header.N : 0u, bStrata ? Strata.Header.N : 0u),
            FMath::Max3(bDetail ? Detail.Header.N : 0u, bDunes ? Dunes.Header.N : 0u, FMath::Max3(bSand ? Sand.Header.N : 0u, bGlacial ? Glacial.Header.N : 0u,
                FMath::Max(bIce ? Ice.Header.N : 0u, bPolar ? Polar.Header.N : 0u))));
        if (bPeaks)
        {
            const VoxelNoiseTiles::FSet& Set = Peaks.Header;
            Out->LookTag = UTF8_TO_TCHAR(Set.LookTag.c_str());
            AppendChainLocked(*Out, Peaks.Texels.GetData(), Set.N);
            Out->N = Set.N;
            Out->Mips = Set.Mips;
            Out->SpacingTexels = Set.N / Set.Channel[0].CellsPerTile;
            // R's and G's means in unit values, and the decode ranges (mip-0 texels).
            const uint16* Texels = (const uint16*)Out->Words.GetData();
            Out->MeanR = (float)VoxelNoiseTiles::MeanOf(Set, Texels, 0, [&](double V) { return UnitOf(Set, 0, V); });
            Out->MeanG = (float)VoxelNoiseTiles::MeanOf(Set, Texels, 1, [&](double V) { return UnitOf(Set, 1, V); });
            Out->RangeR = Set.Channel[0].Max - Set.Channel[0].Min;
            Out->RangeG = Set.Channel[1].Max - Set.Channel[1].Min;
            Out->RangeB = Set.Channel[2].Max - Set.Channel[2].Min;
            Peaks.Texels.Empty();
        }
        if (bDetail)
        {
            const VoxelNoiseTiles::FSet& Set = Detail.Header;
            const uint32 Base = AppendChainLocked(*Out, Detail.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the detail set %s in a texture's rows; no detail."),
                    *Key, *DetailRead);
            }
            else
            {
                // Its statistics from mip 0 (a few hundredths of a second for a 2048 set).
                const VoxelNoiseTiles::FDetailStats Stats = VoxelNoiseTiles::DetailStatsOf(Set, (const uint16*)Detail.Texels.GetData());
                Out->DetailName = DetailRead;
                Out->DetailBase = Base;
                Out->DetailN = Set.N;
                Out->DetailMips = Set.Mips;
                Out->DetailSpacingTexels = Set.N / Set.Channel[0].CellsPerTile;
                Out->DetailMeanHard = (float)Stats.MeanHard;
                Out->DetailMeanSoft = (float)Stats.MeanSoft;
                Out->DetailRangeHard = (float)Stats.RangeHard;
                Out->DetailRangeSoft = (float)Stats.RangeSoft;
                Out->DetailMeanA = (float)Stats.MeanA;
                Out->DetailStdA = (float)Stats.StdA;
                Out->DetailScaleGain = (float)Stats.ScaleGain;
                Out->DetailBoundA = (float)Stats.BoundA;
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Detail set %s: hard rock (G) %.2f texels, soft rock (R) %.2f texels; the detail layer's bound %.2f."),
                    *DetailRead, Stats.RangeHard, Stats.RangeSoft, Stats.BoundA);
            }
            Detail.Texels.Empty();
        }
        if (bHills)
        {
            const VoxelNoiseTiles::FSet& Set = Hills.Header;
            const uint32 Base = AppendChainLocked(*Out, Hills.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the hills' set %s in a texture's rows; the procedural hills stand in."),
                    *Key, *HillName);
            }
            else
            {
                Out->HillName = HillName;
                Out->HillBase = Base;
                Out->HillN = Set.N;
                Out->HillMips = Set.Mips;
                Out->HillSpacingTexels = Set.N / Set.Channel[0].CellsPerTile;
                Out->HillRangeR = Set.Channel[0].Max - Set.Channel[0].Min;
                Out->HillRangeG = Set.Channel[1].Max - Set.Channel[1].Min;
                Out->HillRangeB = Set.Channel[2].Max - Set.Channel[2].Min;
                // R's and G's means (unit values), for the face blend (N1b).
                Out->HillMeanR = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Hills.Texels.GetData(), 0, [&](double V) { return UnitOf(Set, 0, V); });
                Out->HillMeanG = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Hills.Texels.GetData(), 1, [&](double V) { return UnitOf(Set, 1, V); });
            }
            Hills.Texels.Empty();
        }
        if (bStrata && Out->HillN > 0)
        {
            const VoxelNoiseTiles::FSet& Set = Strata.Header;
            const uint32 Base = AppendChainLocked(*Out, Strata.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the strata's set %s in a texture's rows; the hills cover the sedimentary provinces too."),
                    *Key, *StrataName);
            }
            else
            {
                Out->StrataName = StrataName;
                Out->StrataBase = Base;
                Out->StrataN = Set.N;
                Out->StrataMips = Set.Mips;
                Out->StrataSpacingTexels = Set.N / Set.Channel[0].CellsPerTile;
                Out->StrataRangeR = Set.Channel[0].Max - Set.Channel[0].Min;
                Out->StrataRangeG = Set.Channel[1].Max - Set.Channel[1].Min;
                Out->StrataRangeB = Set.Channel[2].Max - Set.Channel[2].Min;
                Out->StrataMeanR = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Strata.Texels.GetData(), 0, [&](double V) { return UnitOf(Set, 0, V); });
                Out->StrataMeanG = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Strata.Texels.GetData(), 1, [&](double V) { return UnitOf(Set, 1, V); });
            }
            Strata.Texels.Empty();
        }
        // T5c: after every chain the material reads (it reads neither of these: its texture's rows
        // can end before them).
        if (bDunes && Out->HillN > 0)
        {
            const VoxelNoiseTiles::FSet& Set = Dunes.Header;
            const uint32 Base = AppendChainLocked(*Out, Dunes.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the dunes' set %s in a texture's rows; no dunes."),
                    *Key, *DuneName);
            }
            else
            {
                Out->DuneName = DuneName;
                Out->DuneBase = Base;
                Out->DuneN = Set.N;
                Out->DuneMips = Set.Mips;
                Out->DuneSpacingTexels = Set.N / Set.Channel[0].CellsPerTile;
                Out->DuneRangeR = Set.Channel[0].Max - Set.Channel[0].Min;
                Out->DuneRangeG = Set.Channel[1].Max - Set.Channel[1].Min;
                Out->DuneMeanR = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Dunes.Texels.GetData(), 0, [&](double V) { return UnitOf(Set, 0, V); });
                Out->DuneMeanG = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Dunes.Texels.GetData(), 1, [&](double V) { return UnitOf(Set, 1, V); });
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Dune set %s: %u^2, crests %u texels apart, sharp (R) %.2f texels, rounded (G) %.2f."),
                    *DuneName, Set.N, Out->DuneSpacingTexels, Out->DuneRangeR, Out->DuneRangeG);
            }
            Dunes.Texels.Empty();
        }
        if (bSand && Out->DuneN > 0)
        {
            const VoxelNoiseTiles::FSet& Set = Sand.Header;
            const uint32 Base = AppendChainLocked(*Out, Sand.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the sand's detail set %s in a texture's rows; the dunes go without it."),
                    *Key, *SandName);
            }
            else
            {
                Out->SandName = SandName;
                Out->SandBase = Base;
                Out->SandN = Set.N;
                Out->SandMips = Set.Mips;
                Out->SandSpacingTexels = (float)Set.N / (float)Set.Channel[1].CellsPerTile;
                Out->SandRangeG = Set.Channel[1].Max - Set.Channel[1].Min;
                Out->SandMeanG = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Sand.Texels.GetData(), 1, [&](double V) { return UnitOf(Set, 1, V); });
            }
            Sand.Texels.Empty();
        }
        // T5d: the glacial set (the peaks' layout), after them too.
        if (bGlacial && Out->N > 0)
        {
            const VoxelNoiseTiles::FSet& Set = Glacial.Header;
            const uint32 Base = AppendChainLocked(*Out, Glacial.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the glacial set %s in a texture's rows; the peaks everywhere."),
                    *Key, *GlacialName);
            }
            else
            {
                Out->GlacialName = GlacialName;
                Out->GlacialBase = Base;
                Out->GlacialN = Set.N;
                Out->GlacialMips = Set.Mips;
                Out->GlacialSpacingTexels = Set.N / Set.Channel[0].CellsPerTile;
                Out->GlacialRangeR = Set.Channel[0].Max - Set.Channel[0].Min;
                Out->GlacialRangeG = Set.Channel[1].Max - Set.Channel[1].Min;
                Out->GlacialRangeB = Set.Channel[2].Max - Set.Channel[2].Min;
                Out->GlacialMeanR = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Glacial.Texels.GetData(), 0, [&](double V) { return UnitOf(Set, 0, V); });
                Out->GlacialMeanG = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Glacial.Texels.GetData(), 1, [&](double V) { return UnitOf(Set, 1, V); });
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Glacial set %s: %u^2, troughs %u texels apart, bedrock (R) %.2f texels, ice (G) %.2f."),
                    *GlacialName, Set.N, Out->GlacialSpacingTexels, Out->GlacialRangeR, Out->GlacialRangeG);
            }
            Glacial.Texels.Empty();
        }
        // T5d2: the ice sheets' set (its G, the swells), after them too.
        if (bIce)
        {
            const VoxelNoiseTiles::FSet& Set = Ice.Header;
            const uint32 Base = AppendChainLocked(*Out, Ice.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the ice sheets' set %s in a texture's rows; smooth sheets."),
                    *Key, *IceName);
            }
            else
            {
                Out->IceName = IceName;
                Out->IceBase = Base;
                Out->IceN = Set.N;
                Out->IceMips = Set.Mips;
                Out->IceSpacingTexels = Set.N / Set.Channel[1].CellsPerTile;
                Out->IceMeanG = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Ice.Texels.GetData(), 1, [&](double V) { return UnitOf(Set, 1, V); });
            }
            Ice.Texels.Empty();
        }
        // T5d3: the polar surfaces' set (R, G and B), after them too.
        if (bPolar)
        {
            const VoxelNoiseTiles::FSet& Set = Polar.Header;
            const uint32 Base = AppendChainLocked(*Out, Polar.Texels.GetData(), Set.N);
            if (Base == ~0u)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The noise pack %s can't take the polar set %s in a texture's rows; no polar surfaces."),
                    *Key, *PolarName);
            }
            else
            {
                Out->PolarName = PolarName;
                Out->PolarBase = Base;
                Out->PolarN = Set.N;
                Out->PolarMips = Set.Mips;
                for (int32 c = 0; c < 3; c++)
                {
                    Out->PolarSpacingTexels[c] = (float)Set.N / (float)Set.Channel[c].CellsPerTile;
                    Out->PolarRange[c] = Set.Channel[c].Max - Set.Channel[c].Min;
                    Out->PolarMean[c] = (float)VoxelNoiseTiles::MeanOf(Set, (const uint16*)Polar.Texels.GetData(), c, [&](double V) { return UnitOf(Set, c, V); });
                }
                // The grains: R's slots' normal (turned onto the flow), G's ridges' long axis (the
                // normal less a quarter turn, within a half turn of the set's x: turned onto the wind).
                Out->PolarTurn[0] = (float)VoxelNoiseTiles::GrainAngleOf(Set, (const uint16*)Polar.Texels.GetData(), 0);
                double SastrugiAxis = VoxelNoiseTiles::GrainAngleOf(Set, (const uint16*)Polar.Texels.GetData(), 1) - 0.5 * UE_DOUBLE_PI;
                if (SastrugiAxis <= -0.5 * UE_DOUBLE_PI) SastrugiAxis += UE_DOUBLE_PI;
                Out->PolarTurn[1] = (float)SastrugiAxis;
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Polar set %s: %u^2, crevasses (R) %.1f texels a cell, %.2f deep, %.1f deg grain; sastrugi (G) %.1f, %.2f, %.1f deg; polygons (B) %.1f, %.2f."),
                    *PolarName, Set.N, Out->PolarSpacingTexels[0], Out->PolarRange[0], FMath::RadiansToDegrees(Out->PolarTurn[0]), Out->PolarSpacingTexels[1],
                    Out->PolarRange[1], FMath::RadiansToDegrees(Out->PolarTurn[1]), Out->PolarSpacingTexels[2], Out->PolarRange[2]);
            }
            Polar.Texels.Empty();
        }
        return Out;
    }

    /** Stores Pack under Key (Lock taken here). */
    void Store(const FString& Key, const TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Pack)
    {
        FScopeLock ScopeLock(&Lock);
        if (bClosed) return;
        FEntry& Entry = Entries.FindOrAdd(Key);
        Entry.Set = Pack;
        Entry.bNone = !Pack.IsValid();
        Entry.bReady = true;
        if (Pack.IsValid())
        {
            Recent.Remove(Pack);
            Recent.Insert(Pack, 0);
            if (Recent.Num() > RecentCount) Recent.SetNum(RecentCount);
        }
        // Dropped before the RHI goes (each pack's GPU copy is released on the render thread).
        if (!bExitHooked)
        {
            bExitHooked = true;
            FCoreDelegates::OnEnginePreExit.AddStatic(&VoxelLand::ReleaseLandNoise);
        }
    }

    /** Whether Key's pack is there (Lock held): its pack (null if none could be made). */
    bool FindReadyLocked(const FString& Key, TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Out)
    {
        const FEntry* Entry = Entries.Find(Key);
        if (!Entry || !Entry->bReady) return false;
        Out = Entry->Set.Pin();
        return Entry->bNone || Out.IsValid();
    }

    bool FindReady(const FString& Key, TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Out)
    {
        FScopeLock ScopeLock(&Lock);
        return FindReadyLocked(Key, Out);
    }

    /** A set name as a setting gives it: trimmed, without a .vxnt; empty if it names a path. */
    FString CleanName(const FString& In, const TCHAR* Setting)
    {
        FString Name = In.TrimStartAndEnd();
        Name.RemoveFromEnd(TEXT(".vxnt"), ESearchCase::IgnoreCase);
        if (Name.Contains(TEXT("/")) || Name.Contains(TEXT("\\")) || Name.Contains(TEXT("..")) || Name.Contains(TEXT(":")) || Name.Contains(TEXT("|")))
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] %s \"%s\" isn't a set name (a file name in %s without .vxnt); picking by the seed."),
                Setting, *In, *VoxelLand::NoiseSetDirectory());
            return FString();
        }
        return Name;
    }

    /** Lists the folder's usable peaks*, hills*, strata*, detail*, dunes* and sanddetail* sets
     *  (Lock held, Directory known; once a session): their headers read and checked, so a file that
     *  can't be used doesn't take a share of the planets. A few small reads, on the game thread the
     *  first time a planet asks. */
    void ListLocked()
    {
        if (bListed) return;
        bListed = true;
        const FString Folder = Directory;
        const TCHAR* Patterns[9] = { TEXT("peaks*.vxnt"), TEXT("hills*.vxnt"), TEXT("strata*.vxnt"), TEXT("detail*.vxnt"), TEXT("dunes*.vxnt"),
            TEXT("sanddetail*.vxnt"), TEXT("glacial*.vxnt"), TEXT("icesheet*.vxnt"), TEXT("polar*.vxnt") };
        TArray<FString>* Lists[9] = { &AuthoredPeaks, &AuthoredHills, &AuthoredStrata, &AuthoredDetail, &AuthoredDunes, &AuthoredSand, &AuthoredGlacial,
            &AuthoredIce, &AuthoredPolar };
        const ESetKind Kinds[9] = { ESetKind::Peaks, ESetKind::Peaks, ESetKind::Peaks, ESetKind::Detail, ESetKind::Dunes, ESetKind::Sand, ESetKind::Glacial,
            ESetKind::Ice, ESetKind::Polar };
        for (int32 Kind = 0; Kind < 9; Kind++)
        {
            const TCHAR* Pattern = Patterns[Kind];
            TArray<FString>& List = *Lists[Kind];
            TArray<FString> Files;
            if (!Folder.IsEmpty())
                IFileManager::Get().FindFiles(Files, *FPaths::Combine(Folder, Pattern), true, false);
            Files.Sort();
            for (const FString& File : Files)
            {
                const FString Path = FPaths::Combine(Folder, File);
                const TUniquePtr<FArchive> Reader(IFileManager::Get().CreateFileReader(*Path));
                const int64 Size = Reader.IsValid() ? Reader->TotalSize() : 0;
                TArray<uint8> Header;
                Header.SetNumZeroed((int32)FMath::Clamp<int64>(Size, 0, 4096));
                if (Header.Num() > 0) Reader->Serialize(Header.GetData(), Header.Num());
                VoxelNoiseTiles::FSet Set;
                uint64_t DataOffset = 0;
                std::string Why = Header.Num() > 0 && !Reader->IsError()
                    ? VoxelNoiseTiles::ReadVxntHeader(Header.GetData(), (size_t)Header.Num(), (uint64_t)Size, Set, DataOffset) : std::string("unreadable");
                // Checked as it will be read (T5c: the dunes' sets at their 1024 mip, as Read does).
                if (Why.empty()) Why = CheckKind(Set, Kinds[Kind]);
                if (Why.empty() && SkipHeaderMips(Set, MaxNOf(Kinds[Kind])) > 0) Why = CheckKind(Set, Kinds[Kind]);
                if (Why.empty())
                    List.Add(FPaths::GetBaseFilename(File));
                else
                    UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] %s isn't a usable set (%s); left out."), *Path, UTF8_TO_TCHAR(Why.c_str()));
            }
        }
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Noise sets in %s: peaks %s; hills %s; strata %s; detail %s; dunes %s; sand detail %s; glacial %s; ice sheet %s; polar %s."), *Folder,
            AuthoredPeaks.Num() > 0 ? *FString::Join(AuthoredPeaks, TEXT(", ")) : TEXT("none (the generated set stands in)"),
            AuthoredHills.Num() > 0 ? *FString::Join(AuthoredHills, TEXT(", ")) : TEXT("none (the procedural hills stand in)"),
            AuthoredStrata.Num() > 0 ? *FString::Join(AuthoredStrata, TEXT(", ")) : TEXT("none (the hills cover the sedimentary provinces)"),
            AuthoredDetail.Num() > 0 ? *FString::Join(AuthoredDetail, TEXT(", ")) : TEXT("none (the generated set stands in)"),
            AuthoredDunes.Num() > 0 ? *FString::Join(AuthoredDunes, TEXT(", ")) : TEXT("none (no dunes)"),
            AuthoredSand.Num() > 0 ? *FString::Join(AuthoredSand, TEXT(", ")) : TEXT("none (the dunes go without)"),
            AuthoredGlacial.Num() > 0 ? *FString::Join(AuthoredGlacial, TEXT(", ")) : TEXT("none (the peaks everywhere)"),
            AuthoredIce.Num() > 0 ? *FString::Join(AuthoredIce, TEXT(", ")) : TEXT("none (smooth ice sheets)"),
            AuthoredPolar.Num() > 0 ? *FString::Join(AuthoredPolar, TEXT(", ")) : TEXT("none (no polar surfaces)"));
    }

    /** One of Names by the seed, rendezvous-hashed (the name with the highest hash of the seed and
     *  itself): adding or removing a set moves only the planets that pick it. */
    const FString* PickBySeed(const TArray<FString>& Names, int32 Seed, uint32 Salt)
    {
        const FString* Best = nullptr;
        uint32 BestScore = 0;
        for (const FString& Name : Names)
        {
            const uint32 Score = VoxelNoiseTiles::Mix((uint32)Seed ^ VoxelNoiseTiles::Mix(FCrc::StrCrc32(*Name.ToLower()) + Salt));
            if (!Best || Score > BestScore) { Best = &Name; BestScore = Score; }
        }
        return Best;
    }
}

FString VoxelLand::NoiseSetDirectory()
{
    using namespace VoxelLandNoiseDetail;
    FScopeLock ScopeLock(&Lock);
    // Found once, on the game thread (StartupModule calls this first): the plugin manager isn't
    // for workers.
    if (!bDirectoryKnown && IsInGameThread())
    {
        bDirectoryKnown = true;
        const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("VoxelPlugin"));
        Directory = Plugin.IsValid() ? FPaths::Combine(Plugin->GetBaseDir(), TEXT("NoiseSets")) : FString();
    }
    return Directory;
}

FString VoxelLand::PeakNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.PeakHeightScale <= 0.0) return FString();
    const FString Chosen = CleanName(Settings.PeakNoiseSet, TEXT("PeakNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    // Name it in PeakNoiseSet to keep a planet's set whatever the folder holds.
    const FString* Best = PickBySeed(AuthoredPeaks, Settings.Seed, 0x7065616bu);
    return Best ? *Best : FString(GeneratedNoiseName);
}

FString VoxelLand::HillNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.HillRelief <= 0.0 || Settings.HillHeightScale <= 0.0) return FString();
    if (Settings.HillNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    const FString Chosen = CleanName(Settings.HillNoiseSet, TEXT("HillNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    const FString* Best = PickBySeed(AuthoredHills, Settings.Seed, 0x68696c6cu);
    return Best ? *Best : FString();
}

FString VoxelLand::StrataNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.StrataHeightScale <= 0.0 || Settings.StrataCoverage <= 0.0) return FString();
    if (Settings.StrataNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    // They ride the hill set's layer.
    if (HillNoiseNameFor(Settings).IsEmpty()) return FString();
    const FString Chosen = CleanName(Settings.StrataNoiseSet, TEXT("StrataNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    const FString* Best = PickBySeed(AuthoredStrata, Settings.Seed, 0x73747261u);
    return Best ? *Best : FString();
}

FString VoxelLand::DetailNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.DetailNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    // Read by the detail layer, and by the peaks', hills' and strata's detail.
    const bool bRead = Settings.DetailAmplitude > 0.0
        || (Settings.PeakDetailStrength > 0.0 && !PeakNoiseNameFor(Settings).IsEmpty())
        || (Settings.HillDetailStrength > 0.0 && !HillNoiseNameFor(Settings).IsEmpty())
        || (Settings.StrataDetailStrength > 0.0 && !StrataNoiseNameFor(Settings).IsEmpty());
    if (!bRead) return FString();
    const FString Chosen = CleanName(Settings.DetailNoiseSet, TEXT("DetailNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    // Name it in DetailNoiseSet to keep a planet's set whatever the folder holds.
    const FString* Best = PickBySeed(AuthoredDetail, Settings.Seed, 0x64657461u);
    return Best ? *Best : FString(GeneratedNoiseName);
}

FString VoxelLand::DuneNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.DuneHeightScale <= 0.0) return FString();
    if (Settings.DuneNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    // They ride the hill set's layer.
    if (HillNoiseNameFor(Settings).IsEmpty()) return FString();
    const FString Chosen = CleanName(Settings.DuneNoiseSet, TEXT("DuneNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    const FString* Best = PickBySeed(AuthoredDunes, Settings.Seed, 0x64756e65u);
    return Best ? *Best : FString();
}

FString VoxelLand::SandDetailNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.SandDetailStrength <= 0.0) return FString();
    if (Settings.SandDetailNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    if (DuneNoiseNameFor(Settings).IsEmpty()) return FString();
    const FString Chosen = CleanName(Settings.SandDetailNoiseSet, TEXT("SandDetailNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    const FString* Best = PickBySeed(AuthoredSand, Settings.Seed, 0x73616e64u);
    return Best ? *Best : FString();
}

FString VoxelLand::GlacialNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.GlacialHeightScale <= 0.0) return FString();
    if (Settings.GlacialNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    // In the peaks' place: none without them.
    if (PeakNoiseNameFor(Settings).IsEmpty()) return FString();
    const FString Chosen = CleanName(Settings.GlacialNoiseSet, TEXT("GlacialNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    const FString* Best = PickBySeed(AuthoredGlacial, Settings.Seed, 0x676c6163u);
    return Best ? *Best : FString();
}

FString VoxelLand::IceSheetNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.IceSheetMaxThickness <= 0.0 || Settings.IceSheetSwellHeight <= 0.0) return FString();
    if (Settings.IceSheetNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    const FString Chosen = CleanName(Settings.IceSheetNoiseSet, TEXT("IceSheetNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    const FString* Best = PickBySeed(AuthoredIce, Settings.Seed, 0x69636573u);
    return Best ? *Best : FString();
}

FString VoxelLand::PolarNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandNoiseDetail;
    if (Settings.CrevasseHeightScale <= 0.0 && Settings.SastrugiHeightScale <= 0.0 && Settings.PolygonHeightScale <= 0.0 && Settings.SnowDuneHeight <= 0.0)
        return FString();
    if (Settings.PolarNoiseSet.TrimStartAndEnd().Equals(TEXT("none"), ESearchCase::IgnoreCase)) return FString();
    const FString Chosen = CleanName(Settings.PolarNoiseSet, TEXT("PolarNoiseSet"));
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    const FString* Best = PickBySeed(AuthoredPolar, Settings.Seed, 0x706f6c61u);
    return Best ? *Best : FString();
}

double VoxelLand::PolarSpacing(const FVoxelTerrestrialSettings& Settings, int32 Channel)
{
    if (Channel == 0) return FMath::Max(Settings.CrevasseSpacing, 10.0);
    if (Channel == 1) return FMath::Max(Settings.SastrugiSpacing, 1.0);
    return FMath::Max(Settings.PolygonSpacing, 4.0);
}

double VoxelLand::SnowDuneSpacing(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Max(Settings.SnowDuneSpacing, 50.0);
}

double VoxelLand::PolarReach(const FVoxelTerrestrialSettings& Settings, const FVoxelLandNoiseSet* Noise, bool bUp)
{
    if (!Noise || Noise->PolarN == 0) return 0.0;
    const double Scales[3] = { Settings.CrevasseHeightScale, Settings.SastrugiHeightScale, Settings.PolygonHeightScale };
    double Reach = 0.0;
    for (int32 c = 0; c < 3; c++)
    {
        if (!(Scales[c] > 0.0) || !(Noise->PolarSpacingTexels[c] > 0.0f)) continue;
        // One turn's value within 0 .. 1 (the B-splines are convex) about its mean (unit).
        const double TexelM = PolarSpacing(Settings, c) / (double)Noise->PolarSpacingTexels[c];
        const double Side = bUp ? 1.0 - (double)Noise->PolarMean[c] : (double)Noise->PolarMean[c];
        Reach += Scales[c] * (double)Noise->PolarRange[c] * TexelM * FMath::Max(Side, 0.0);
    }
    // The snow dunes: the dune set's G over SnowDuneHeight.
    if (Settings.SnowDuneHeight > 0.0 && Noise->DuneN > 0)
        Reach += Settings.SnowDuneHeight * FMath::Max(bUp ? 1.0 - (double)Noise->DuneMeanG : (double)Noise->DuneMeanG, 0.0);
    return 1.01 * PolarBlendBound * Reach;
}

double VoxelLand::DetailLayerWavelength(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Max(Settings.DetailWavelength, 100.0);
}

FString VoxelLand::LandNoiseNameFor(const FVoxelTerrestrialSettings& Settings)
{
    const FString Peaks = PeakNoiseNameFor(Settings);
    const FString Hills = HillNoiseNameFor(Settings);
    const FString Strata = StrataNoiseNameFor(Settings);
    const FString Detail = DetailNoiseNameFor(Settings);
    if (Peaks.IsEmpty() && Hills.IsEmpty() && Detail.IsEmpty()) return FString();
    FString Key = Peaks + TEXT("|") + Hills + TEXT("|") + Strata + TEXT("|") + Detail;
    // T5c, T5d: the dunes' and the glacial set's parts only when there are any (a planet without
    // keeps its old key).
    // T5d2: and the ice sheets' set (parts empty to its left as needed).
    // T5d3: and the polar set's.
    const FString Dunes = DuneNoiseNameFor(Settings);
    const FString Glacial = GlacialNoiseNameFor(Settings);
    const FString Ice = IceSheetNoiseNameFor(Settings);
    const FString Polar = PolarNoiseNameFor(Settings);
    if (!Dunes.IsEmpty() || !Glacial.IsEmpty() || !Ice.IsEmpty() || !Polar.IsEmpty())
        Key += TEXT("|") + Dunes + TEXT("|") + (Dunes.IsEmpty() ? FString() : SandDetailNoiseNameFor(Settings));
    if (!Glacial.IsEmpty() || !Ice.IsEmpty() || !Polar.IsEmpty())
        Key += TEXT("|") + Glacial;
    if (!Ice.IsEmpty() || !Polar.IsEmpty())
        Key += TEXT("|") + Ice;
    if (!Polar.IsEmpty())
        Key += TEXT("|") + Polar;
    return Key;
}

TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> VoxelLand::GetLandNoise(const FString& Key)
{
    using namespace VoxelLandNoiseDetail;
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Pack;
    if (Key.IsEmpty()) return Pack;
    if (FindReady(Key, Pack)) return Pack;
    FScopeLock MakeScope(&MakeLock);
    // Made meanwhile by another thread?
    if (FindReady(Key, Pack)) return Pack;
    // Kept under the key while a planet uses it (an error isn't repeated meanwhile); asked for
    // again after that, the files are read again.
    Pack = MakePackLocked(Key);
    Store(Key, Pack);
    return Pack;
}

bool VoxelLand::TryGetLandNoise(const FString& Key, TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Out)
{
    using namespace VoxelLandNoiseDetail;
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Pack;
    if (Key.IsEmpty())
    {
        Out.Reset();
        return true;
    }
    if (!FindReady(Key, Pack)) return false;
    Out = MoveTemp(Pack);
    return true;
}

void VoxelLand::PrewarmLandNoise(const FString& Key)
{
    using namespace VoxelLandNoiseDetail;
    if (Key.IsEmpty()) return;
    {
        FScopeLock ScopeLock(&Lock);
        if (bClosed) return;
        TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Pack;
        if (FindReadyLocked(Key, Pack)) return;
        FEntry& Entry = Entries.FindOrAdd(Key);
        if (Entry.bPending) return;
        Entry.bPending = true;
    }
    Async(EAsyncExecution::ThreadPool, [Key]()
        {
            GetLandNoise(Key);
            FScopeLock ScopeLock(&VoxelLandNoiseDetail::Lock);
            if (VoxelLandNoiseDetail::FEntry* Entry = VoxelLandNoiseDetail::Entries.Find(Key))
                Entry->bPending = false;
        });
}

void VoxelLand::ReleaseLandNoise()
{
    using namespace VoxelLandNoiseDetail;
    FScopeLock ScopeLock(&Lock);
    // The engine is exiting: a pack still being made is handed to its caller but not kept.
    bClosed = true;
    Entries.Empty();
    Recent.Empty();
}

// ---------------------------------------------------------------------------------------------
// Keys, shape, cache
// ---------------------------------------------------------------------------------------------

int32 VoxelLand::BakeResolutionFor(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Clamp(Settings.BakeResolution, 64, 2048);
}

bool VoxelLand::IsErosionEnabled()
{
    return CVarVoxelLandErosion.GetValueOnAnyThread() != 0;
}

uint32 VoxelLand::BakeKey(const FVoxelTerrestrialSettings& Settings, double SeaRadius)
{
    uint32 H = Settings.GetBakeHash();
    H = HashCombine(H, ::GetTypeHash((int64)FMath::RoundToDouble(SeaRadius)));
    H = HashCombine(H, ::GetTypeHash(BakeResolutionFor(Settings)));
    H = HashCombine(H, ::GetTypeHash(FVoxelLandBake::Version));
    H = HashCombine(H, ::GetTypeHash(GeneratorVersion));
    H = HashCombine(H, ::GetTypeHash(IsErosionEnabled() ? 1 : 0));
    return H;
}

uint32 VoxelLand::BakeCheck(const FVoxelTerrestrialSettings& Settings, double SeaRadius)
{
    // Every bake input in one buffer, CRC'd: independent of BakeKey's hash combine.
    TArray<double> Values = { (double)Settings.Seed, (double)Settings.PlateCount, (double)BakeResolutionFor(Settings),
        Settings.ContinentalFraction, Settings.LandformWidthScale, Settings.RangeRelief, Settings.CoastHeight,
        Settings.InteriorHeight, Settings.ShelfDepth, Settings.AbyssDepth, FMath::RoundToDouble(SeaRadius),
        (double)FVoxelLandBake::Version, (double)GeneratorVersion, Settings.PlainsUpliftThreshold, Settings.MountainRampWidth,
        Settings.SolveResidual, (double)Settings.SolveMaxSteps, Settings.MinLakeDepth, Settings.MinLakeArea, IsErosionEnabled() ? 1.0 : 0.0,
        Settings.RoutingJitter, Settings.UpliftVariation, Settings.ErodibilityVariation, Settings.VariationWavelength, Settings.InitialRoughness,
        Settings.PlainsRelief, Settings.RiverIncision, Settings.SummitHeightScale, Settings.RoutingRandomness, Settings.DrainageSpreadCells,
        Settings.PlainsUndulation, Settings.LandformErosion, Settings.ChannelHeadCells, Settings.ChannelAreaVariation,
        Settings.ChannelAreaVariationWavelength, (double)Settings.ChannelAreaVariationOctaves, Settings.HillslopeSteepness,
        Settings.ChannelCharacterInfluence };
    return FCrc::MemCrc32(Values.GetData(), Values.Num() * (int32)sizeof(double));
}

int32 VoxelLand::GullyOctavesFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake)
{
    if (Settings.GullyStrength <= 0.0) return 0;
    // From lambda0 = two bake cells, halving, while the wavelength is at least the finest.
    const double Lambda0 = 2.0 * CellArc(Bake.SeaRadius, Bake.N) * 0.01;
    const double Finest = FMath::Max(Settings.GullyFinestWavelength, 1.0);
    if (Lambda0 < Finest) return 0;
    return FMath::Clamp(FMath::FloorToInt(FMath::Log2(Lambda0 / Finest)) + 1, 0, GullyMaxOctaves);
}

int32 VoxelLand::HillOctavesFor(const FVoxelTerrestrialSettings& Settings)
{
    if (Settings.HillRelief <= 0.0) return 0;
    const double Longest = FMath::Max(Settings.HillWavelength, 1.0);
    const double Finest = FMath::Max(Settings.HillFinestWavelength, 1.0);
    if (Longest < Finest) return 1;
    return FMath::Clamp(FMath::FloorToInt(FMath::Log2(Longest / Finest)) + 1, 1, HillMaxOctaves);
}

double VoxelLand::HillFinestFrequency(const FVoxelTerrestrialSettings& Settings)
{
    const int32 Octaves = FMath::Max(HillOctavesFor(Settings), 1);
    const double FinestCm = FMath::Max(Settings.HillWavelength, 1.0) * 100.0 / FMath::Pow(2.0, (double)(Octaves - 1));
    return 1.0 / FinestCm;
}

float VoxelLand::PeakTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set)
{
    if (Settings.PeakHeightScale <= 0.0 || Set.SpacingTexels == 0) return 0.0f;
    // A texel is the spacing over the set's spacing in texels (m); a bake cell, its arc at the
    // face centre (VoxelLandPeaks.ush's lattice is in the face's cells).
    const double TexelM = FMath::Max(Settings.PeakSpacing, PeakMinSpacing) / (double)Set.SpacingTexels;
    return (float)(CellArc(Bake.SeaRadius, Bake.N) * 0.01 / TexelM);
}

double VoxelLand::PeakDetailWavelength(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Clamp(Settings.PeakDetailWavelength, 16.0, 0.25 * FMath::Max(Settings.PeakSpacing, PeakMinSpacing));
}

float VoxelLand::PeakDetailTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set)
{
    if (Settings.PeakDetailStrength <= 0.0 || Set.DetailN == 0 || Set.DetailSpacingTexels == 0) return 0.0f;
    const double TexelM = PeakDetailWavelength(Settings) / (double)Set.DetailSpacingTexels;
    return (float)(CellArc(Bake.SeaRadius, Bake.N) * 0.01 / TexelM);
}

float VoxelLand::HillTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set)
{
    if (Settings.HillRelief <= 0.0 || Settings.HillHeightScale <= 0.0 || Set.HillN == 0 || Set.HillSpacingTexels == 0) return 0.0f;
    const double TexelM = FMath::Max(Settings.HillSpacing, HillMinSpacing) / (double)Set.HillSpacingTexels;
    return (float)(CellArc(Bake.SeaRadius, Bake.N) * 0.01 / TexelM);
}

float VoxelLand::StrataTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set)
{
    if (Settings.StrataHeightScale <= 0.0 || Settings.StrataCoverage <= 0.0 || Set.StrataN == 0 || Set.StrataSpacingTexels == 0) return 0.0f;
    if (HillTexelsPerCell(Settings, Bake, Set) <= 0.0f) return 0.0f;
    const double TexelM = FMath::Max(Settings.StrataSpacing, HillMinSpacing) / (double)Set.StrataSpacingTexels;
    return (float)(CellArc(Bake.SeaRadius, Bake.N) * 0.01 / TexelM);
}

double VoxelLand::StrataSetReliefCap(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Max(Settings.StrataHeightScale, 0.0) * 0.5 * FMath::Max(Settings.StrataSpacing, HillMinSpacing);
}

double VoxelLand::StrataDetailWavelength(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Clamp(Settings.StrataDetailWavelength, 16.0, 0.25 * FMath::Max(Settings.StrataSpacing, HillMinSpacing));
}

float VoxelLand::DuneTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set)
{
    if (Settings.DuneHeightScale <= 0.0 || Set.DuneN == 0 || Set.DuneSpacingTexels == 0) return 0.0f;
    if (HillTexelsPerCell(Settings, Bake, Set) <= 0.0f) return 0.0f;
    const double TexelM = FMath::Max(Settings.DuneSpacing, DuneMinSpacing) / (double)Set.DuneSpacingTexels;
    return (float)(CellArc(Bake.SeaRadius, Bake.N) * 0.01 / TexelM);
}

double VoxelLand::DuneRelief(const FVoxelTerrestrialSettings& Settings, const FVoxelLandNoiseSet& Set)
{
    if (Set.DuneN == 0 || Set.DuneSpacingTexels == 0) return 0.0;
    const double TexelM = FMath::Max(Settings.DuneSpacing, DuneMinSpacing) / (double)Set.DuneSpacingTexels;
    return FMath::Max(Settings.DuneHeightScale, 0.0) * FMath::Max3((double)Set.DuneRangeR, (double)Set.DuneRangeG, 1e-6) * TexelM;
}

double VoxelLand::SandDetailWavelength(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Clamp(Settings.SandDetailWavelength, 2.0, FMath::Max(0.25 * FMath::Max(Settings.DuneSpacing, DuneMinSpacing), 2.0));
}

double VoxelLand::SandDetailRelief(const FVoxelTerrestrialSettings& Settings, const FVoxelLandNoiseSet& Set)
{
    if (Settings.SandDetailStrength <= 0.0 || Set.SandN == 0 || !(Set.SandSpacingTexels > 0.0f)) return 0.0;
    const double TexelM = SandDetailWavelength(Settings) / (double)Set.SandSpacingTexels;
    return Settings.SandDetailStrength * (double)Set.SandRangeG * TexelM;
}

double VoxelLand::HillSetReliefCap(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Max(Settings.HillHeightScale, 0.0) * 0.5 * FMath::Max(Settings.HillSpacing, HillMinSpacing);
}

double VoxelLand::HillDetailWavelength(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Clamp(Settings.HillDetailWavelength, 16.0, 0.25 * FMath::Max(Settings.HillSpacing, HillMinSpacing));
}

float VoxelLand::GlacialTexelsPerCell(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet& Set)
{
    if (Settings.GlacialHeightScale <= 0.0 || Set.GlacialN == 0 || Set.GlacialSpacingTexels == 0) return 0.0f;
    if (PeakTexelsPerCell(Settings, Bake, Set) <= 0.0f) return 0.0f;
    const double TexelM = FMath::Max(Settings.GlacialSpacing, PeakMinSpacing) / (double)Set.GlacialSpacingTexels;
    return (float)(CellArc(Bake.SeaRadius, Bake.N) * 0.01 / TexelM);
}

double VoxelLand::PeakReach(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet* Noise, bool bUp)
{
    // The amplitude is at most the strength times the smaller of the bake's largest local relief
    // and the set's own relief (VoxelLandPeakAmplitude), one face's shape within -1 .. 1 (over the
    // larger of R's and G's ranges, each channel's mean within its range; the B-splines are
    // convex). The detail adds its unit times the same amplitude, at most strength x
    // DetailRangeWaves of its wavelength (the detail set's bound about its means,
    // VoxelNoiseTiles::DetailStatsOf; VoxelLandPeaks.ush: its scale is that over the peaks'
    // relief, the amplitude at most the height scale times that relief). The face blend keeps the
    // variance about the zero mean: FaceBlendBound times that either way (N1b).
    if (Settings.PeakHeightScale <= 0.0) return 0.0;
    double Relief = (double)Bake.MaxRelief;
    if (Noise && Noise->N > 0 && Noise->SpacingTexels > 0)
    {
        const double TexelM = FMath::Max(Settings.PeakSpacing, PeakMinSpacing) / (double)Noise->SpacingTexels;
        Relief = FMath::Min(Relief, FMath::Max3((double)Noise->RangeR, (double)Noise->RangeG, 1e-6) * TexelM);
    }
    double Reach = FMath::Max(Settings.PeakHeightScale, 0.0) * Relief;
    if (Settings.PeakDetailStrength > 0.0)
        Reach += Settings.PeakHeightScale * Settings.PeakDetailStrength * VoxelNoiseTiles::DetailRangeWaves * PeakDetailWavelength(Settings);
    if (Noise && GlacialTexelsPerCell(Settings, Bake, *Noise) > 0.0f)
    {
        // T5d: the glacial layer, by the same rule from its set (its detail the peaks', scaled by
        // its height); the switch between the two keeps the variance: (1 - P + P) / sqrt((1 - P)^2
        // + P^2) <= sqrt 2 times the larger.
        const double TexelM = FMath::Max(Settings.GlacialSpacing, PeakMinSpacing) / (double)Noise->GlacialSpacingTexels;
        const double GRelief = FMath::Min((double)Bake.MaxRelief, FMath::Max3((double)Noise->GlacialRangeR, (double)Noise->GlacialRangeG, 1e-6) * TexelM);
        double GReach = FMath::Max(Settings.GlacialHeightScale, 0.0) * GRelief;
        if (Settings.PeakDetailStrength > 0.0)
            GReach += Settings.GlacialHeightScale * Settings.PeakDetailStrength * VoxelNoiseTiles::DetailRangeWaves * PeakDetailWavelength(Settings);
        Reach = UE_SQRT_2 * FMath::Max(Reach, GReach);
    }
    return 1.01 * FaceBlendBound * Reach * 100.0;
}

double VoxelLand::GullyReach(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake)
{
    // Each octave is half its depth times a value in -1..1, its depth at most strength x
    // lambda0 x 0.42^o x 0.8 (VoxelLandGully.ush); the face blend and the weights are convex.
    const int32 Octaves = GullyOctavesFor(Settings, Bake);
    const double Lambda0 = 2.0 * CellArc(Bake.SeaRadius, Bake.N);
    double Sum = 0.0;
    double Gain = 1.0;
    for (int32 o = 0; o < Octaves; o++)
    {
        Sum += Gain;
        Gain *= GullyGain;
    }
    return 0.5 * FMath::Max(Settings.GullyStrength, 0.0) * Lambda0 * GullyMaxSlope * Sum;
}

void VoxelLand::ShapeFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, const FVoxelLandNoiseSet* Noise,
    double& OutPlanetRadius, double& OutNoiseAmplitude)
{
    // The B-spline and the face blend are convex combinations of the cells, so the bake's
    // range holds; the detail layer reaches at most its set's bound times its amplitude (N1:
    // VoxelNoiseTiles::DetailStatsOf, a percent over; 2.5 without the pack's detail set), the
    // gullies GullyReach either way, the hills HillMaxUnit times their relief (up; down by
    // less), the peaks PeakReach up (and down from the land's lowest, sea level), and a metre of
    // margin either side.
    const double DetailUnit = Noise && Noise->DetailN > 0 ? 1.01 * (double)Noise->DetailBoundA : 0.0;
    const double DetailReach = DetailUnit * FMath::Max(Settings.DetailAmplitude, 0.0) * 100.0;
    // The procedural hills (m): up, and down by less (bounded the same).
    double HillUp = HillMaxUnit * FMath::Max(Settings.HillRelief, 0.0);
    double HillDown = HillUp;
    if (Noise && HillTexelsPerCell(Settings, Bake, *Noise) > 0.0f)
    {
        // The hill set (T4a, VoxelLandHillSet.ush; the density's numbers): one face's unit within
        // 0 .. 1 from its base network up (over the larger of R's and G's ranges) times its
        // relief, capped (HillSetReliefCap); the swells' height on top; the detail
        // (DetailRangeWaves of its wavelength, either way, as PeakReach) scaled by the same
        // relief. The face blend keeps the variance about the mean M (N1b): up to K = sqrt 3
        // times one face's unit, and down to K (-detail) - (K - 1) M, M at most 1. A percent of
        // margin for the density's float arithmetic.
        const double TexelM = FMath::Max(Settings.HillSpacing, HillMinSpacing) / (double)Noise->HillSpacingTexels;
        const double ReliefM = FMath::Max3((double)Noise->HillRangeR, (double)Noise->HillRangeG, 1e-6) * TexelM;
        const double Relief = FMath::Min(FMath::Max(Settings.HillHeightScale, 0.0) * ReliefM, HillSetReliefCap(Settings));
        double Detail = 0.0;
        if (Settings.HillDetailStrength > 0.0 && Noise->DetailN > 0 && Noise->DetailSpacingTexels > 0)
            Detail = Settings.HillDetailStrength * VoxelNoiseTiles::DetailRangeWaves * HillDetailWavelength(Settings) * Relief / ReliefM;
        // The foothill growth (T4b) multiplies the hills and swells by (1 - P)(1 + G 4P(1 - P)),
        // which peaks above 1 (about 1.32 at G = 1); 1 + G bounds it.
        const double Growth = 1.0 + FMath::Clamp(Settings.HillFoothillGrowth, 0.0, 2.0);
        const double Swell = FMath::Max(Settings.HillSwellHeight, 0.0);
        HillUp = 1.01 * Growth * FaceBlendBound * (Relief + Swell + Detail);
        HillDown = 1.01 * Growth * (FaceBlendBound * Detail + (FaceBlendBound - 1.0) * (Relief + Swell));
        if (StrataTexelsPerCell(Settings, Bake, *Noise) > 0.0f)
        {
            // The strata (T4c): the same bound from their own set. They share the layer with the
            // hills by S and 1 - S, so the larger of the two bounds holds.
            const double STexelM = FMath::Max(Settings.StrataSpacing, HillMinSpacing) / (double)Noise->StrataSpacingTexels;
            const double SReliefM = FMath::Max3((double)Noise->StrataRangeR, (double)Noise->StrataRangeG, 1e-6) * STexelM;
            const double SRelief = FMath::Min(FMath::Max(Settings.StrataHeightScale, 0.0) * SReliefM, StrataSetReliefCap(Settings));
            double SDetail = 0.0;
            if (Settings.StrataDetailStrength > 0.0 && Noise->DetailN > 0 && Noise->DetailSpacingTexels > 0)
                SDetail = Settings.StrataDetailStrength * VoxelNoiseTiles::DetailRangeWaves * StrataDetailWavelength(Settings) * SRelief / SReliefM;
            HillUp = FMath::Max(HillUp, 1.01 * FaceBlendBound * (SRelief + SDetail));
            HillDown = FMath::Max(HillDown, 1.01 * (FaceBlendBound * SDetail + (FaceBlendBound - 1.0) * SRelief));
        }
        if (DuneTexelsPerCell(Settings, Bake, *Noise) > 0.0f)
        {
            // The dunes (T5c, VoxelLandDunes.ush): added to the layer where the climate makes
            // desert (the rest gives way, so the sum bounds both). One turn's unit within 0 .. 1
            // (over the larger range) about its mean M, the blends DuneBlendBound times that: up to
            // M + B (1 - M) <= B, down to M - B M. The sand detail, zero-mean, within its range
            // either way times B.
            const double DRelief = DuneRelief(Settings, *Noise);
            const double DDetail = SandDetailRelief(Settings, *Noise);
            const double DMean = FMath::Max((double)Noise->DuneMeanR * (double)Noise->DuneRangeR, (double)Noise->DuneMeanG * (double)Noise->DuneRangeG)
                / FMath::Max3((double)Noise->DuneRangeR, (double)Noise->DuneRangeG, 1e-6);
            HillUp += 1.01 * DuneBlendBound * (DRelief + DDetail);
            HillDown += 1.01 * ((DuneBlendBound - 1.0) * DMean * DRelief + DuneBlendBound * DDetail);
        }
    }
    // (Summed in the order T3 did, so a planet without a hill set keeps its shape exactly.)
    const double ReachDown = DetailReach + GullyReach(Settings, Bake) + HillDown * 100.0 + 100.0;
    const double ReachUp = DetailReach + GullyReach(Settings, Bake) + HillUp * 100.0 + 100.0;
    // T5d3: the polar surfaces ride on whatever is there (the land, the peaks' ice, the sheets): their
    // reach either way on top.
    const double PolarUp = PolarReach(Settings, Noise, true) * 100.0;
    const double PolarDown = PolarReach(Settings, Noise, false) * 100.0;
    const double Low = FMath::Min((double)Bake.MinHeight * 100.0, -PeakReach(Settings, Bake, Noise, false)) - ReachDown - PolarDown;
    double UpReach = PeakReach(Settings, Bake, Noise, true) + ReachUp + PolarUp;
    if (Settings.IceSheetMaxThickness > 0.0)
    {
        // T5d2: the ice sheets (VoxelLandIce.ush): their surface the ground's mean under them (no
        // higher than the bake's highest), the thickness (at most the cap), the swells (zero-mean,
        // the face blend's bound about it) and the smooth maximum's bump (a quarter of its width).
        // The detail layer rides on the ice (by at most its whole reach); the bump applies wherever
        // the two meet, the land leading or the ice.
        const double Swell = IceSheetNoiseNameFor(Settings).IsEmpty() ? 0.0 : FMath::Max(Settings.IceSheetSwellHeight, 0.0);
        const double IceUp = (Settings.IceSheetMaxThickness + FaceBlendBound * Swell) * 1.01 * 100.0 + DetailReach + PolarUp + 100.0;
        UpReach = FMath::Max(UpReach, IceUp) + 0.25 * IceSheetBlend * 1.01 * 100.0;
    }
    const double High = (double)Bake.MaxHeight * 100.0 + UpReach;
    OutPlanetRadius = FMath::Max(Bake.SeaRadius + Low, 1.0);
    OutNoiseAmplitude = FMath::Max(High - Low, 1.0);
}

namespace VoxelLandCache
{
    static_assert(sizeof(FVoxelLandLake) == 16 && sizeof(FFloat16) == sizeof(uint16), "The cache stores these as they are in memory");

    struct FHeader
    {
        uint32 Magic = VLB::CacheMagic;
        uint32 Version = FVoxelLandBake::Version;
        uint32 Key = 0;
        int32 N = 0;
        int32 Pad = FVoxelLandBake::Pad;
        int32 RawSize = 0;
        int32 CompressedSize = 0;
        float Threshold = 0.0f;
        double SeaRadius = 0.0;
        double BakeSeconds = 0.0;
        uint32 SettingsCheck = 0;   // VoxelLand::BakeCheck: a second hash, against key collisions
        uint32 Flags = 0;           // 1: eroded (Flow and Water follow Attr)
        int32 LakeCount = 0;        // FVoxelLandLake records after the fields
        int32 SolveSteps = 0;
        float SolveResidual = 0.0f;
        float ReliefFactor = 1.0f;
    };

    /** The raw (uncompressed) size of a bake's data: heights, attributes, landform, province, then for an
     *  eroded bake the drainage and the lake levels, then the lakes. */
    int64 RawBytes(int64 NumCells, bool bEroded, int64 NumLakes)
    {
        return NumCells * (int64)(sizeof(float) + 2 * sizeof(uint16) + sizeof(uint8))
            + (bEroded ? NumCells * (int64)(sizeof(uint16) + sizeof(float) + sizeof(int32)) : 0)
            + NumLakes * (int64)sizeof(FVoxelLandLake);
    }

    /** Bakes kept in Saved/VoxelLand; the least recently written beyond this are deleted. */
    constexpr int32 MaxFiles = 16;

    /** A bake fit to keep or use: finite heights and a range. */
    bool IsUsable(const FVoxelLandBake& Bake)
    {
        if (Bake.Height.Num() != Bake.NumCells() || Bake.Attr.Num() != Bake.NumCells() || Bake.Form.Num() != Bake.NumCells()
            || Bake.Province.Num() != Bake.NumCells()) return false;
        if (Bake.bEroded && (Bake.Flow.Num() != Bake.NumCells() || Bake.Water.Num() != Bake.NumCells() || Bake.Receiver.Num() != Bake.NumCells())) return false;
        if (!(Bake.MinHeight <= Bake.MaxHeight)) return false;
        for (const float H : Bake.Height)
        {
            if (!FMath::IsFinite(H)) return false;
        }
        return true;
    }

    /** Deletes the oldest bake files beyond MaxFiles. */
    void Trim(const FString& Directory)
    {
        TArray<FString> Files;
        IFileManager::Get().FindFiles(Files, *FPaths::Combine(Directory, TEXT("Bake_*.vlb")), true, false);
        if (Files.Num() <= MaxFiles) return;
        TArray<TPair<FDateTime, FString>> Dated;
        for (const FString& Name : Files)
        {
            const FString Path = FPaths::Combine(Directory, Name);
            Dated.Emplace(IFileManager::Get().GetTimeStamp(*Path), Path);
        }
        Dated.Sort([](const TPair<FDateTime, FString>& A, const TPair<FDateTime, FString>& B) { return A.Key > B.Key; });
        for (int32 i = MaxFiles; i < Dated.Num(); i++)
            IFileManager::Get().Delete(*Dated[i].Value, false, true, true);
    }

    FString PathFor(uint32 Key, int32 N)
    {
        return FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("VoxelLand"), FString::Printf(TEXT("Bake_%08x_%d.vlb"), Key, N));
    }

    bool Save(const FVoxelLandBake& Bake)
    {
        const int64 NumCells = Bake.Height.Num();
        const int64 Total = RawBytes(NumCells, Bake.bEroded, Bake.Lakes.Num());
        if (Total >= MAX_int32) return false;
        TArray<uint8> Raw;
        Raw.SetNumUninitialized((int32)Total);
        int64 Offset = 0;
        auto Append = [&Raw, &Offset](const void* Data, int64 Bytes)
            {
                FMemory::Memcpy(Raw.GetData() + Offset, Data, Bytes);
                Offset += Bytes;
            };
        Append(Bake.Height.GetData(), NumCells * sizeof(float));
        Append(Bake.Attr.GetData(), NumCells * sizeof(uint16));
        Append(Bake.Form.GetData(), NumCells * sizeof(uint16));
        Append(Bake.Province.GetData(), NumCells * sizeof(uint8));
        if (Bake.bEroded)
        {
            Append(Bake.Flow.GetData(), NumCells * sizeof(uint16));
            Append(Bake.Water.GetData(), NumCells * sizeof(float));
            // The receivers as differences from their cells (0: none), mostly a few small values.
            TArray<int32> Delta;
            Delta.SetNumUninitialized((int32)NumCells);
            for (int32 i = 0; i < (int32)NumCells; i++)
                Delta[i] = Bake.Receiver[i] == ~0u ? 0 : (int32)(Bake.Receiver[i] - (uint32)i);
            Append(Delta.GetData(), NumCells * sizeof(int32));
        }
        Append(Bake.Lakes.GetData(), (int64)Bake.Lakes.Num() * sizeof(FVoxelLandLake));

        int32 CompressedSize = FCompression::CompressMemoryBound(NAME_Zlib, Raw.Num());
        TArray<uint8> File;
        File.SetNumUninitialized(sizeof(FHeader) + CompressedSize);
        if (!FCompression::CompressMemory(NAME_Zlib, File.GetData() + sizeof(FHeader), CompressedSize, Raw.GetData(), Raw.Num()))
            return false;
        File.SetNum(sizeof(FHeader) + CompressedSize);

        FHeader Header;
        Header.Key = Bake.Key;
        Header.N = Bake.N;
        Header.RawSize = Raw.Num();
        Header.CompressedSize = CompressedSize;
        Header.Threshold = Bake.CrustThreshold;
        Header.SeaRadius = Bake.SeaRadius;
        Header.BakeSeconds = Bake.BakeSeconds;
        Header.SettingsCheck = Bake.SettingsCheck;
        Header.Flags = Bake.bEroded ? 1u : 0u;
        Header.LakeCount = Bake.Lakes.Num();
        Header.SolveSteps = Bake.SolveSteps;
        Header.SolveResidual = Bake.SolveResidual;
        Header.ReliefFactor = Bake.ReliefFactor;
        FMemory::Memcpy(File.GetData(), &Header, sizeof(FHeader));

        const FString Path = PathFor(Bake.Key, Bake.N);
        // A unique temporary name: an editor and a PIE world may save the same bake at once.
        const FString Temp = FString::Printf(TEXT("%s.%s.tmp"), *Path, *FGuid::NewGuid().ToString());
        IFileManager::Get().MakeDirectory(*FPaths::GetPath(Path), true);
        if (!FFileHelper::SaveArrayToFile(File, *Temp))
        {
            IFileManager::Get().Delete(*Temp, false, true, true);
            return false;
        }
        if (!IFileManager::Get().Move(*Path, *Temp, true))
        {
            IFileManager::Get().Delete(*Temp, false, true, true);
            return false;
        }
        Trim(FPaths::GetPath(Path));
        return true;
    }

    TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Load(uint32 Key, uint32 SettingsCheck, int32 N, double SeaRadius)
    {
        const FString Path = PathFor(Key, N);
        TArray<uint8> File;
        if (!IFileManager::Get().FileExists(*Path) || !FFileHelper::LoadFileToArray(File, *Path) || File.Num() < (int32)sizeof(FHeader))
            return nullptr;
        FHeader Header;
        FMemory::Memcpy(&Header, File.GetData(), sizeof(FHeader));
        const int32 Side = N + 2 * FVoxelLandBake::Pad;
        const int64 NumCells = 6LL * Side * Side;
        if (Header.Magic != VLB::CacheMagic || Header.Version != FVoxelLandBake::Version || Header.Key != Key
            || Header.SettingsCheck != SettingsCheck || Header.N != N
            || Header.Pad != FVoxelLandBake::Pad || FMath::Abs(Header.SeaRadius - SeaRadius) > 0.5
            || Header.LakeCount < 0 || (int64)Header.RawSize != RawBytes(NumCells, (Header.Flags & 1u) != 0, Header.LakeCount)
            || Header.CompressedSize <= 0 || (int64)sizeof(FHeader) + Header.CompressedSize > File.Num())
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Ignoring cache file %s (doesn't match this bake)."), *Path);
            return nullptr;
        }
        TArray<uint8> Raw;
        Raw.SetNumUninitialized(Header.RawSize);
        if (!FCompression::UncompressMemory(NAME_Zlib, Raw.GetData(), Raw.Num(), File.GetData() + sizeof(FHeader), Header.CompressedSize))
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Cache file %s doesn't decompress; baking again."), *Path);
            return nullptr;
        }
        const TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = MakeShared<FVoxelLandBake, ESPMode::ThreadSafe>();
        Bake->N = N;
        Bake->SeaRadius = SeaRadius;
        Bake->Key = Key;
        Bake->SettingsCheck = SettingsCheck;
        Bake->CrustThreshold = Header.Threshold;
        Bake->bFromCache = true;
        Bake->bEroded = (Header.Flags & 1u) != 0;
        Bake->SolveSteps = Header.SolveSteps;
        Bake->SolveResidual = Header.SolveResidual;
        Bake->ReliefFactor = Header.ReliefFactor;
        int64 Offset = 0;
        auto Take = [&Raw, &Offset](void* Data, int64 Bytes)
            {
                FMemory::Memcpy(Data, Raw.GetData() + Offset, Bytes);
                Offset += Bytes;
            };
        Bake->Height.SetNumUninitialized((int32)NumCells);
        Bake->Attr.SetNumUninitialized((int32)NumCells);
        Bake->Form.SetNumUninitialized((int32)NumCells);
        Bake->Province.SetNumUninitialized((int32)NumCells);
        Take(Bake->Height.GetData(), NumCells * sizeof(float));
        Take(Bake->Attr.GetData(), NumCells * sizeof(uint16));
        Take(Bake->Form.GetData(), NumCells * sizeof(uint16));
        Take(Bake->Province.GetData(), NumCells * sizeof(uint8));
        if (Bake->bEroded)
        {
            Bake->Flow.SetNumUninitialized((int32)NumCells);
            Bake->Water.SetNumUninitialized((int32)NumCells);
            Take(Bake->Flow.GetData(), NumCells * sizeof(uint16));
            Take(Bake->Water.GetData(), NumCells * sizeof(float));
            Bake->Receiver.SetNumUninitialized((int32)NumCells);
            Take(Bake->Receiver.GetData(), NumCells * sizeof(int32));
            for (int32 i = 0; i < (int32)NumCells; i++)
            {
                const int32 Delta = (int32)Bake->Receiver[i];
                Bake->Receiver[i] = Delta == 0 ? ~0u : (uint32)(i + Delta);
                if (Bake->Receiver[i] >= (uint32)NumCells) Bake->Receiver[i] = ~0u;
            }
        }
        Bake->Lakes.SetNumUninitialized(Header.LakeCount);
        Take(Bake->Lakes.GetData(), (int64)Header.LakeCount * sizeof(FVoxelLandLake));
        Bake->BuildTanTable();
        Bake->UpdateRange();
        Bake->BuildRelief();
        Bake->BuildProvinceStats();
        Bake->BuildSeaConnected();
        if (!IsUsable(*Bake))
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Cache file %s holds an invalid bake; baking again."), *Path);
            return nullptr;
        }
        // Used now: Trim keeps the most recently used files.
        IFileManager::Get().SetTimeStamp(*Path, FDateTime::UtcNow());
        return Bake;
    }
}

// ---------------------------------------------------------------------------------------------
// GPU passes
// ---------------------------------------------------------------------------------------------

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelLandBakeParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, LandPlates)
    SHADER_PARAMETER(uint32, LandPlateCount)
    SHADER_PARAMETER(uint32, LandSeed)
    SHADER_PARAMETER(float, LandRadiusM)
    SHADER_PARAMETER(float, LandWidthScale)
    SHADER_PARAMETER(float, LandReliefScale)
    SHADER_PARAMETER(float, LandShelfDepth)
    SHADER_PARAMETER(float, LandAbyssDepth)
    SHADER_PARAMETER(float, LandCoastHeight)
    SHADER_PARAMETER(float, LandInteriorHeight)
    SHADER_PARAMETER(uint32, BakeN)
    SHADER_PARAMETER(uint32, BakePad)
    SHADER_PARAMETER(uint32, FirstCell)
    SHADER_PARAMETER(uint32, NumCells)
    SHADER_PARAMETER(uint32, GroupsX)
    SHADER_PARAMETER(float, LandFraction)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, Histogram)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, ThresholdBuffer)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutHeight)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutAttr)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutUp)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutCrust)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutForm)
END_SHADER_PARAMETER_STRUCT()

#define VOXEL_LAND_BAKE_SHADER(ShaderName) \
    class ShaderName : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(ShaderName); \
        using FParameters = FVoxelLandBakeParameters; \
        SHADER_USE_PARAMETER_STRUCT(ShaderName, FGlobalShader); \
        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
        { \
            return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
        } \
    };

VOXEL_LAND_BAKE_SHADER(FVoxelLandCrustCS)
VOXEL_LAND_BAKE_SHADER(FVoxelLandThresholdCS)
VOXEL_LAND_BAKE_SHADER(FVoxelLandFieldsCS)
#undef VOXEL_LAND_BAKE_SHADER

IMPLEMENT_GLOBAL_SHADER(FVoxelLandCrustCS, "/Plugin/VoxelPlugin/Private/VoxelLandBake.usf", "CrustCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelLandThresholdCS, "/Plugin/VoxelPlugin/Private/VoxelLandBake.usf", "ThresholdCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelLandFieldsCS, "/Plugin/VoxelPlugin/Private/VoxelLandBake.usf", "FieldsCS", SF_Compute);

/** A GPU bake in flight: its inputs, the buffers kept between frames (render thread), the
 *  erosion solve that follows the plate pass, and the threshold read back on the way. */
struct FVoxelLandBakeJob
{
    VLB::FInputs Inputs;
    int32 CellsPerStep = 1 << 20;

    TRefCountPtr<FRDGPooledBuffer> Plates;
    TRefCountPtr<FRDGPooledBuffer> Histogram;
    TRefCountPtr<FRDGPooledBuffer> Threshold;
    TRefCountPtr<FRDGPooledBuffer> Height;
    TRefCountPtr<FRDGPooledBuffer> Attr;
    TRefCountPtr<FRDGPooledBuffer> Up;
    TRefCountPtr<FRDGPooledBuffer> Crust;
    TRefCountPtr<FRDGPooledBuffer> Form;
    TUniquePtr<FRHIGPUBufferReadback> ThresholdReadback;

    /** The solve (VoxelLandSolve.h), run once the plate pass is done; bErode: with the
     *  erosion (else it only routes the plate model's heights). */
    TUniquePtr<VoxelLandSolve::FJob> Solve;
    bool bErode = true;

    // Written on the render thread before Status is set.
    float OutThreshold = 0.0f;
    bool bThresholdRead = false;

    /** 0 running, 1 the results are in (Solve->Result), -1 failed (the game thread bakes on
     *  the CPU). */
    std::atomic<int32> Status{ 0 };

    int32 NumCells() const
    {
        const int32 Side = Inputs.N + 2 * FVoxelLandBake::Pad;
        return 6 * Side * Side;
    }

    int32 NumInterior() const { return 6 * Inputs.N * Inputs.N; }
    int32 Chunk() const { return FMath::Max(CellsPerStep, 4096); }

    /** Crust steps (the last also finds the threshold), then field steps (the last also copies
     *  the threshold for readback); the solve follows (TickSolve). */
    int32 CrustSteps() const { return FMath::DivideAndRoundUp(NumInterior(), Chunk()); }
    int32 FieldSteps() const { return FMath::DivideAndRoundUp(NumCells(), Chunk()); }
    int32 NumSteps() const { return CrustSteps() + FieldSteps(); }

    /** Render thread: drops every GPU buffer. */
    void ReleaseBuffers()
    {
        Plates.SafeRelease();
        Histogram.SafeRelease();
        Threshold.SafeRelease();
        Height.SafeRelease();
        Attr.SafeRelease();
        Up.SafeRelease();
        Crust.SafeRelease();
        Form.SafeRelease();
        ThresholdReadback.Reset();
    }
};

namespace VoxelLandBakeDetail
{
    /** The GPU path needs a renderer with SM5 (the bake shaders' feature level). */
    bool CanBakeOnGpu()
    {
        return FApp::CanEverRender() && GMaxRHIFeatureLevel >= ERHIFeatureLevel::SM5 && CVarVoxelLandForceCpu.GetValueOnGameThread() == 0;
    }

    bool AreShadersReady()
    {
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        return ShaderMap
            && ShaderMap->GetShader(&FVoxelLandCrustCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelLandThresholdCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelLandFieldsCS::GetStaticType(), 0).IsValid()
            && VoxelLandSolve::AreShadersReady();
    }

    FIntVector GroupsFor(uint32 Threads, uint32& OutGroupsX)
    {
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(Threads, 64u), 1u);
        OutGroupsX = FMath::Min(NumGroups, 32768u);
        return FIntVector((int32)OutGroupsX, (int32)FMath::DivideAndRoundUp(NumGroups, OutGroupsX), 1);
    }

    /** The solve's settings for a bake (game thread: reads the console variables). */
    VoxelLandSolve::FSettings MakeSolveSettings(const FVoxelTerrestrialSettings& Settings, const FInputs& In, const TArray<FVector2f>& TanTable)
    {
        VoxelLandSolve::FSettings S;
        S.N = In.N;
        S.Pad = FVoxelLandBake::Pad;
        S.RadiusM = In.RadiusM;
        S.Seed = In.Seed;
        S.PlainsUplift = (float)FMath::Max(Settings.PlainsUpliftThreshold, 0.0);
        S.RampWidth = (float)FMath::Max(Settings.MountainRampWidth, 1.0);
        S.MaxSteps = FMath::Clamp(Settings.SolveMaxSteps, 10, 1000);
        S.Tolerance = (float)FMath::Clamp(Settings.SolveResidual, 0.001, 0.5);
        S.MinLakeDepth = (float)FMath::Max(Settings.MinLakeDepth, 0.0);
        S.MinLakeArea = (float)(FMath::Max(Settings.MinLakeArea, 0.0) * 1.0e6);
        S.Jitter = (float)FMath::Clamp(Settings.RoutingJitter, 0.0, 0.45);
        S.RouteNoise = (float)FMath::Clamp(Settings.RoutingRandomness, 0.0, 3.0);
        S.SpreadCells = (float)FMath::Clamp(Settings.DrainageSpreadCells, 0.0, 1024.0);
        S.UpliftVariation = (float)FMath::Clamp(Settings.UpliftVariation, 0.0, 1.5);
        S.ErodibilityVariation = (float)FMath::Clamp(Settings.ErodibilityVariation, 0.0, 2.0);
        S.VariationWavelength = (float)(FMath::Max(Settings.VariationWavelength, 10.0) * 1000.0);
        S.Roughness = (float)FMath::Max(Settings.InitialRoughness, 0.0);
        S.PlainsRelief = (float)FMath::Max(Settings.PlainsRelief, 0.0);
        S.Undulation = (float)FMath::Max(Settings.PlainsUndulation, 0.0);
        S.LandformErosion = (float)FMath::Clamp(Settings.LandformErosion, 0.0, 1.0);
        S.Incision = (float)FMath::Max(Settings.RiverIncision, 0.0);
        S.ReliefTopScale = (float)FMath::Clamp(Settings.SummitHeightScale, 1.0, 2.0);
        // T3g: the channel heads (0 cells: off, the T3f solve) and the hillslopes' threshold.
        S.ChannelCells = (float)FMath::Clamp(Settings.ChannelHeadCells, 0.0, 256.0);
        S.ChannelVariation = (float)FMath::Clamp(Settings.ChannelAreaVariation, 0.0, 3.0);
        S.ChannelWavelength = (float)(FMath::Clamp(Settings.ChannelAreaVariationWavelength, 0.05, 4.0) * S.VariationWavelength);
        S.ChannelOctaves = FMath::Clamp(Settings.ChannelAreaVariationOctaves, 1, 6);
        S.Hillslope = (float)FMath::Clamp(Settings.HillslopeSteepness, 0.25, 8.0);
        S.ChannelCharacter = (float)FMath::Clamp(Settings.ChannelCharacterInfluence, 0.0, 1.0);
        S.bErode = VoxelLand::IsErosionEnabled();
        // R1b: the rivers, built on the GPU right after the routing.
        S.bRivers = Settings.RiverMinArea > 0.0;
        S.RiverMinAreaKm2 = (float)Settings.RiverMinArea;
        S.RiverWidthScale = (float)FMath::Max(Settings.RiverWidthScale, 0.1);
        S.RiverValleyScale = (float)FMath::Max(Settings.ValleyWidthScale, 0.1);
        S.StepCellsPerFrame = FMath::Max(CVarVoxelLandSolveStepCells.GetValueOnGameThread(), 1 << 16);
        S.IterationCellsPerFrame = FMath::Max(CVarVoxelLandSolveIterationCells.GetValueOnGameThread(), 1 << 18);
        S.bRouteAhead = CVarVoxelLandSolveRouteAhead.GetValueOnGameThread() != 0;
        S.TanTable = TanTable;
        return S;
    }

    /** Render thread. One step of the plate pass (FVoxelLandBakeJob::NumSteps): a chunk of the
     *  crust histogram (the first creates the buffers, the last finds the threshold), or a
     *  chunk of the fields (the last copies the threshold for readback). */
    void RunStep(FRHICommandListImmediate& RHICmdList, FVoxelLandBakeJob& Job, int32 Step)
    {
        if (Job.Status.load(std::memory_order_acquire) != 0) return;
        if (!AreShadersReady())
        {
            Job.Status.store(-1, std::memory_order_release);
            return;
        }
        const FInputs& In = Job.Inputs;
        const uint32 N = (uint32)In.N;
        const uint32 NumCells = (uint32)Job.NumCells();
        const uint32 NumInterior = (uint32)Job.NumInterior();
        const uint32 Chunk = (uint32)Job.Chunk();
        FRDGBuilder GraphBuilder(RHICmdList);

        FRDGBufferRef Plates;
        FRDGBufferRef Histogram;
        FRDGBufferRef Threshold;
        FRDGBufferRef Height;
        FRDGBufferRef Attr;
        FRDGBufferRef Up;
        FRDGBufferRef Crust;
        FRDGBufferRef Form;
        if (Step == 0)
        {
            auto Create = [&GraphBuilder](uint32 Stride, uint32 Count, const TCHAR* Name)
                {
                    FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(Stride, Count);
                    Desc.Usage |= EBufferUsageFlags::SourceCopy;
                    return GraphBuilder.CreateBuffer(Desc, Name);
                };
            Plates = VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandPlates"), MakeArrayView(In.Plates));
            Histogram = Create(sizeof(uint32), HistogramBins, TEXT("Voxel.LandHistogram"));
            Threshold = Create(sizeof(float), 1, TEXT("Voxel.LandThreshold"));
            Height = Create(sizeof(float), NumCells, TEXT("Voxel.LandBakeHeight"));
            Attr = Create(sizeof(uint32), NumCells, TEXT("Voxel.LandBakeAttr"));
            Up = Create(sizeof(float), NumCells, TEXT("Voxel.LandBakeUp"));
            Crust = Create(sizeof(float), NumCells, TEXT("Voxel.LandBakeCrust"));
            Form = Create(sizeof(uint32), NumCells, TEXT("Voxel.LandBakeForm"));
        }
        else
        {
            Plates = GraphBuilder.RegisterExternalBuffer(Job.Plates);
            Histogram = GraphBuilder.RegisterExternalBuffer(Job.Histogram);
            Threshold = GraphBuilder.RegisterExternalBuffer(Job.Threshold);
            Height = GraphBuilder.RegisterExternalBuffer(Job.Height);
            Attr = GraphBuilder.RegisterExternalBuffer(Job.Attr);
            Up = GraphBuilder.RegisterExternalBuffer(Job.Up);
            Crust = GraphBuilder.RegisterExternalBuffer(Job.Crust);
            Form = GraphBuilder.RegisterExternalBuffer(Job.Form);
        }

        FVoxelLandBakeParameters Template;
        Template.LandPlates = GraphBuilder.CreateSRV(Plates);
        Template.LandPlateCount = In.PlateCount;
        Template.LandSeed = In.Seed;
        Template.LandRadiusM = In.RadiusM;
        Template.LandWidthScale = In.WidthScale;
        Template.LandReliefScale = In.ReliefScale;
        Template.LandShelfDepth = In.ShelfDepth;
        Template.LandAbyssDepth = In.AbyssDepth;
        Template.LandCoastHeight = In.CoastHeight;
        Template.LandInteriorHeight = In.InteriorHeight;
        Template.BakeN = N;
        Template.BakePad = (uint32)FVoxelLandBake::Pad;
        Template.FirstCell = 0;
        Template.NumCells = 0;
        Template.GroupsX = 1;
        Template.LandFraction = In.LandFraction;
        Template.Histogram = GraphBuilder.CreateUAV(Histogram);
        Template.ThresholdBuffer = GraphBuilder.CreateUAV(Threshold);
        Template.OutHeight = GraphBuilder.CreateUAV(Height);
        Template.OutAttr = GraphBuilder.CreateUAV(Attr);
        Template.OutUp = GraphBuilder.CreateUAV(Up);
        Template.OutCrust = GraphBuilder.CreateUAV(Crust);
        Template.OutForm = GraphBuilder.CreateUAV(Form);
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);

        const int32 CrustSteps = Job.CrustSteps();
        if (Step < CrustSteps)
        {
            if (Step == 0)
                AddClearUAVPass(GraphBuilder, Template.Histogram, 0u);
            const uint32 First = (uint32)Step * Chunk;
            {
                FVoxelLandBakeParameters* Parameters = GraphBuilder.AllocParameters<FVoxelLandBakeParameters>();
                *Parameters = Template;
                Parameters->FirstCell = First;
                Parameters->NumCells = FMath::Min(Chunk, NumInterior - First);
                const FIntVector Groups = GroupsFor(Parameters->NumCells, Parameters->GroupsX);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand crust %u..%u", First, First + Parameters->NumCells), TShaderMapRef<FVoxelLandCrustCS>(ShaderMap), Parameters, Groups);
            }
            if (Step == CrustSteps - 1)
            {
                FVoxelLandBakeParameters* Parameters = GraphBuilder.AllocParameters<FVoxelLandBakeParameters>();
                *Parameters = Template;
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand threshold"), TShaderMapRef<FVoxelLandThresholdCS>(ShaderMap), Parameters, FIntVector(1, 1, 1));
            }
        }
        else
        {
            const int32 FieldStep = Step - CrustSteps;
            const uint32 First = (uint32)FieldStep * Chunk;
            if (First < NumCells)
            {
                FVoxelLandBakeParameters* Parameters = GraphBuilder.AllocParameters<FVoxelLandBakeParameters>();
                *Parameters = Template;
                Parameters->FirstCell = First;
                Parameters->NumCells = FMath::Min(Chunk, NumCells - First);
                const FIntVector Groups = GroupsFor(Parameters->NumCells, Parameters->GroupsX);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand fields %u..%u", First, First + Parameters->NumCells), TShaderMapRef<FVoxelLandFieldsCS>(ShaderMap), Parameters, Groups);
            }
            if (FieldStep == Job.FieldSteps() - 1)
            {
                Job.ThresholdReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.LandThresholdReadback"));
                AddEnqueueCopyPass(GraphBuilder, Job.ThresholdReadback.Get(), Threshold, (uint32)sizeof(float));
            }
        }
        if (Step == 0)
        {
            Job.Plates = GraphBuilder.ConvertToExternalBuffer(Plates);
            Job.Histogram = GraphBuilder.ConvertToExternalBuffer(Histogram);
            Job.Threshold = GraphBuilder.ConvertToExternalBuffer(Threshold);
            Job.Height = GraphBuilder.ConvertToExternalBuffer(Height);
            Job.Attr = GraphBuilder.ConvertToExternalBuffer(Attr);
            Job.Up = GraphBuilder.ConvertToExternalBuffer(Up);
            Job.Crust = GraphBuilder.ConvertToExternalBuffer(Crust);
            Job.Form = GraphBuilder.ConvertToExternalBuffer(Form);
        }
        GraphBuilder.Execute();
    }

    /** Render thread, once a frame after the plate pass: reads the threshold back, runs the
     *  solve's next passes, and sets the job's status when the solve is done or failed. The
     *  buffers are released either way. */
    void TickSolve(FRHICommandListImmediate& RHICmdList, FVoxelLandBakeJob& Job)
    {
        if (Job.Status.load(std::memory_order_acquire) != 0 || !Job.Solve.IsValid()) return;
        if (!Job.bThresholdRead && Job.ThresholdReadback.IsValid() && Job.ThresholdReadback->IsReady())
        {
            const void* Source = Job.ThresholdReadback->Lock((uint32)sizeof(float));
            if (!Source)
            {
                Job.ReleaseBuffers();
                Job.Status.store(-1, std::memory_order_release);
                return;
            }
            FMemory::Memcpy(&Job.OutThreshold, Source, sizeof(float));
            Job.ThresholdReadback->Unlock();
            Job.bThresholdRead = true;
            Job.ThresholdReadback.Reset();
        }
        VoxelLandSolve::FPlateFields Fields;
        Fields.Height = Job.Height;
        Fields.Up = Job.Up;
        Fields.Crust = Job.Crust;
        Fields.Attr = Job.Attr;
        Fields.Form = Job.Form;
        Job.Solve->Tick(RHICmdList, Fields);
        const int32 SolveStatus = Job.Solve->Status.load(std::memory_order_acquire);
        if (SolveStatus < 0 || (SolveStatus > 0 && !Job.bThresholdRead && !Job.ThresholdReadback.IsValid()))
        {
            Job.ReleaseBuffers();
            Job.Status.store(-1, std::memory_order_release);
            return;
        }
        if (SolveStatus > 0 && Job.bThresholdRead)
        {
            const VoxelLandSolve::FResult& R = Job.Solve->Result;
            const int32 NumCells = Job.NumCells();
            const bool bOk = R.Height.Num() == NumCells && R.Attr.Num() == NumCells && R.FlowHalves.Num() == (NumCells + 1) / 2 && R.Water.Num() == NumCells
                && R.Form.Num() == NumCells;
            Job.ReleaseBuffers();
            Job.Status.store(bOk ? 1 : -1, std::memory_order_release);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// FVoxelLandBaker
// ---------------------------------------------------------------------------------------------

namespace VoxelLandBakeDetail
{
    // The rivers (below).
    TSharedPtr<FVoxelLandRiverSet, ESPMode::ThreadSafe> MakeRiverSet(VoxelLandRiverBuild::FResult&& R, uint32 Key, uint32 BakeKey, double MinAreaKm2,
        int32 Builds, double Seconds);
    void ReleaseOnRenderThread(TRefCountPtr<FRDGPooledBuffer>& A, TRefCountPtr<FRDGPooledBuffer>& B);
}

/** Progress of a CPU bake (cells done), shared with its task, and its cancel flag (set when
 *  the baker is dropped). */
struct FVoxelLandCpuProgress
{
    std::atomic<int64> Cells{ 0 };
    std::atomic<bool> bCancel{ false };
    int64 Total = 1;
};

FVoxelLandBaker::FVoxelLandBaker(const FVoxelTerrestrialSettings& InSettings, double InSeaRadius)
    : Settings(InSettings)
    , SeaRadius(InSeaRadius)
    , Key(VoxelLand::BakeKey(InSettings, InSeaRadius))
    , N(VoxelLand::BakeResolutionFor(InSettings))
{
    // The noise pack (the peaks', detail, hills' and strata's sets), read or generated meanwhile (the density
    // takes it once the bake is ready; an empty key, none needed, does nothing).
    VoxelLand::PrewarmLandNoise(VoxelLand::LandNoiseNameFor(InSettings));
}

namespace VoxelLandBakeDetail
{
    /** Drops a GPU job on the render thread: its pooled buffers belong there. */
    void ReleaseJob(TSharedPtr<FVoxelLandBakeJob, ESPMode::ThreadSafe>& Job)
    {
        if (!Job.IsValid()) return;
        TSharedPtr<FVoxelLandBakeJob, ESPMode::ThreadSafe> Dropped = MoveTemp(Job);
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseJob)([Dropped](FRHICommandListImmediate&) mutable
            {
                Dropped.Reset();
            });
    }
}

FVoxelLandBaker::~FVoxelLandBaker()
{
    // A CPU task stops at its next chunk; a GPU job's buffers are released on the render
    // thread (commands still queued keep their own references).
    if (CpuProgress.IsValid())
        CpuProgress->bCancel.store(true, std::memory_order_relaxed);
    VoxelLandBakeDetail::ReleaseJob(Job);
}

float FVoxelLandBaker::GetProgress() const
{
    switch (State)
    {
    case EState::Done: return Result.IsValid() ? 1.0f : 0.0f;
    case EState::Gpu: return GpuSteps > 0 ? 0.1f * (float)GpuStep / (float)GpuSteps : 0.0f;
    case EState::GpuSolving: return 0.1f + 0.89f * (Job.IsValid() && Job->Solve.IsValid() ? Job->Solve->Progress.load(std::memory_order_relaxed) : 0.0f);
    case EState::GpuAssembling: return 0.99f;
    case EState::Cpu:
        if (CpuProgress.IsValid())
            return FMath::Clamp(0.99f * (float)CpuProgress->Cells.load(std::memory_order_relaxed) / (float)CpuProgress->Total, 0.0f, 0.99f);
        return 0.5f;
    default: return 0.0f;
    }
}

void FVoxelLandBaker::Tick()
{
    using namespace VoxelLandBakeDetail;
    switch (State)
    {
    case EState::Start:
    {
        StartSeconds = FPlatformTime::Seconds();
        if (CVarVoxelLandCache.GetValueOnGameThread() != 0)
        {
            const uint32 InKey = Key;
            const uint32 InCheck = VoxelLand::BakeCheck(Settings, SeaRadius);
            const int32 InN = N;
            const double InSeaRadius = SeaRadius;
            Pending = Async(EAsyncExecution::ThreadPool, [InKey, InCheck, InN, InSeaRadius]()
                {
                    // (Its rivers are built on the GPU once it's loaded: VoxelLand::TickRivers.)
                    return VoxelLandCache::Load(InKey, InCheck, InN, InSeaRadius);
                });
            State = EState::LoadingCache;
        }
        else if (CanBakeOnGpu())
            StartGpu();
        else
            StartCpu();
        break;
    }
    case EState::LoadingCache:
    {
        if (!Pending.IsReady()) break;
        TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Loaded = Pending.Get();
        Pending = TFuture<TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe>>();
        if (Loaded.IsValid())
        {
            Finish(Loaded);
            break;
        }
        if (CanBakeOnGpu())
            StartGpu();
        else
            StartCpu();
        break;
    }
    case EState::Gpu:
    {
        // The shaders may still be compiling (e.g. after recompileshaders): wait a few seconds
        // before baking on the CPU instead.
        if (GpuStep == 0 && !AreShadersReady())
        {
            if (++GpuWaitFrames < 300)
                break;
            Job->Status.store(-1, std::memory_order_release);
        }
        if (Job->Status.load(std::memory_order_acquire) < 0)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The land bake shaders aren't available on this GPU; baking on the CPU."));
            StartCpu();
            break;
        }
        // One compute step per frame (the field chunks are sized by voxel.Land.BakeCellsPerFrame).
        const int32 Step = GpuStep++;
        TSharedPtr<FVoxelLandBakeJob, ESPMode::ThreadSafe> InJob = Job;
        ENQUEUE_RENDER_COMMAND(VoxelLandBakeStep)([InJob, Step](FRHICommandListImmediate& RHICmdList)
            {
                RunStep(RHICmdList, *InJob, Step);
            });
        if (GpuStep >= GpuSteps)
            State = EState::GpuSolving;
        break;
    }
    case EState::GpuSolving:
    {
        // The erosion solve and the routing: a few passes a frame (VoxelLandSolve.h).
        const int32 Status = Job->Status.load(std::memory_order_acquire);
        if (Status == 0)
        {
            TSharedPtr<FVoxelLandBakeJob, ESPMode::ThreadSafe> InJob = Job;
            ENQUEUE_RENDER_COMMAND(VoxelLandSolveTick)([InJob](FRHICommandListImmediate& RHICmdList)
                {
                    TickSolve(RHICmdList, *InJob);
                });
            break;
        }
        if (Status < 0)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The GPU bake failed; baking the plate model on the CPU (no erosion)."));
            StartCpu();
            break;
        }
        // The render thread is done with the result; the fields are narrowed and checked on a
        // worker (a few passes over every cell).
        const TSharedPtr<VoxelLandSolve::FResult, ESPMode::ThreadSafe> R = MakeShared<VoxelLandSolve::FResult, ESPMode::ThreadSafe>(MoveTemp(Job->Solve->Result));
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Solve %08x: %d solved cells, %d steps (residual %.4f), relief x %.3f (a logarithm of scale %.0f m above %.0f m, the top %.0f m); routing over %d land cells, %d lakes kept%s."),
            Key, R->SolvedCells, R->Steps, R->Residual, R->Factor, R->ReliefScale, R->ReliefKnee, R->ReliefTop, R->LandCells, R->Lakes.Num(),
            R->Errors != 0 ? *FString::Printf(TEXT("; error bits 0x%x"), R->Errors) : TEXT(""));
        if (Settings.ChannelHeadCells > 0.0)
        {
            // T3g: the channel heads' area, nominal (the noise spans about -1..1), at the mean cell.
            const double CellKm = VoxelLand::CellArc(SeaRadius, N) * 1.0e-5;
            const double BaseKm2 = FMath::Clamp(Settings.ChannelHeadCells, 0.0, 256.0) * CellKm * CellKm;
            const double Spread = FMath::Pow(2.0, FMath::Clamp(Settings.ChannelAreaVariation, 0.0, 3.0));
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Solve %08x: channel heads at %.1f km^2 (%.1f cells), varying %.1f .. %.1f km^2 over %.0f km; hillslopes x %.2f."),
                Key, BaseKm2, Settings.ChannelHeadCells, BaseKm2 / Spread, BaseKm2 * Spread,
                FMath::Clamp(Settings.ChannelAreaVariationWavelength, 0.05, 4.0) * FMath::Max(Settings.VariationWavelength, 10.0), Settings.HillslopeSteepness);
        }
        if (CVarVoxelDebugLand.GetValueOnGameThread() != 0)
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Solve phases: %s."), *FString::Join(R->PhaseLog, TEXT(", ")));
        const TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = MakeBake(Job->Inputs);
        Bake->CrustThreshold = Job->OutThreshold;
        Bake->bEroded = Job->bErode;
        Bake->SolveSteps = R->Steps;
        Bake->SolveResidual = R->Residual;
        Bake->ReliefFactor = R->Factor;
        ReleaseJob(Job);
        // The fields the density samples stay on the GPU as the solve left them (the bake's GPU
        // copy takes them; nothing to upload again).
        ENQUEUE_RENDER_COMMAND(VoxelLandKeepBakeFields)([GPU = Bake->GPU, Height = MoveTemp(R->GPUHeight), Water = MoveTemp(R->GPUWater),
            Flow = MoveTemp(R->GPUFlow), Tan = MoveTemp(R->GPUTan)](FRHICommandListImmediate&) mutable
            {
                if (!GPU.IsValid() || !Height.IsValid() || !Tan.IsValid()) return;
                GPU->Height = MoveTemp(Height);
                GPU->TanTable = MoveTemp(Tan);
                GPU->Water = MoveTemp(Water);
                GPU->Flow = MoveTemp(Flow);
            });
        // The rivers, built on the GPU after the routing (R1b); none tried (the shaders weren't
        // ready) leaves them to VoxelLand::TickRivers, a failed build is none.
        TSharedPtr<FVoxelLandRiverSet, ESPMode::ThreadSafe> Rivers;
        if (R->bRivers)
            Rivers = MakeRiverSet(MoveTemp(R->Rivers), VoxelLand::RiverKey(Settings), Key, Settings.RiverMinArea, R->RiverBuilds, 0.0);
        else if (R->RiverBuilds > 0)
        {
            Rivers = MakeShared<FVoxelLandRiverSet, ESPMode::ThreadSafe>();
            Rivers->Key = VoxelLand::RiverKey(Settings);
        }
        ReleaseOnRenderThread(R->Rivers.GPUTiles, R->Rivers.GPUData);
        Pending = Async(EAsyncExecution::ThreadPool, [Bake, R, Rivers]() -> TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe>
            {
                const int32 Num = R->Height.Num();
                if (R->FlowHalves.Num() != (Num + 1) / 2) return nullptr;
                Bake->Height = MoveTemp(R->Height);
                Bake->Water = MoveTemp(R->Water);
                Bake->Lakes = MoveTemp(R->Lakes);
                Bake->Receiver = MoveTemp(R->Receiver);
                Bake->Attr.SetNumUninitialized(Num);
                Bake->Flow.SetNumUninitialized(Num);
                Bake->Form.SetNumUninitialized(Num);
                Bake->Province.SetNumUninitialized(Num);
                for (int32 i = 0; i < Num; i++)
                {
                    Bake->Attr[i] = (uint16)R->Attr[i];
                    // The landform in the low half, the province from bit 16 (VoxelLandBake.usf).
                    Bake->Form[i] = (uint16)R->Form[i];
                    Bake->Province[i] = (uint8)FMath::Min(R->Form[i] >> 16, 63u);
                    // The drainage as the GPU rounded it (its halves are the density's too).
                    const uint16 Half = (uint16)(R->FlowHalves[i >> 1] >> ((i & 1) * 16));
                    if ((Half & 0x7c00u) == 0x7c00u) return nullptr;
                    Bake->Flow[i].Encoded = Half;
                }
                Bake->UpdateRange();
                Bake->BuildRelief();
                Bake->BuildProvinceStats();
                Bake->BuildSeaConnected();
                Bake->Rivers = Rivers;
                return VoxelLandCache::IsUsable(*Bake) ? Bake : nullptr;
            });
        State = EState::GpuAssembling;
        break;
    }
    case EState::GpuAssembling:
    {
        if (!Pending.IsReady()) break;
        TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = Pending.Get();
        Pending = TFuture<TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe>>();
        if (!Bake.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The GPU bake of %08x has non-finite values; baking the plate model on the CPU (no erosion)."), Key);
            StartCpu();
            break;
        }
        Finish(Bake);
        break;
    }
    case EState::Cpu:
    {
        if (!Pending.IsReady()) break;
        TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = Pending.Get();
        Pending = TFuture<TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe>>();
        CpuProgress.Reset();
        if (!Bake.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The CPU bake of %08x failed (non-finite heights?); the planet stays as it is."), Key);
            State = EState::Done;
            break;
        }
        Finish(Bake);
        break;
    }
    case EState::Done:
        break;
    }
}

void FVoxelLandBaker::StartGpu()
{
    Job = MakeShared<FVoxelLandBakeJob, ESPMode::ThreadSafe>();
    Job->Inputs = VoxelLandBakeDetail::MakeInputs(Settings, SeaRadius, Key);
    Job->CellsPerStep = FMath::Max(CVarVoxelLandBakeCellsPerFrame.GetValueOnGameThread(), 4096);
    {
        // The tan table the sampler uses, for the solve's cell directions.
        const TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Grid = VoxelLandBakeDetail::MakeBake(Job->Inputs);
        VoxelLandSolve::FSettings SolveSettings = VoxelLandBakeDetail::MakeSolveSettings(Settings, Job->Inputs, Grid->TanTable);
        Job->bErode = SolveSettings.bErode;
        Job->Solve = MakeUnique<VoxelLandSolve::FJob>(MoveTemp(SolveSettings));
    }
    GpuStep = 0;
    GpuSteps = Job->NumSteps();
    GpuWaitFrames = 0;
    State = EState::Gpu;
    if (CVarVoxelDebugLand.GetValueOnGameThread() != 0)
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Baking %08x on the GPU: 6 x %d^2 cells in %d steps."), Key, N, GpuSteps);
}

void FVoxelLandBaker::StartCpu()
{
    using namespace VoxelLandBakeDetail;
    ReleaseJob(Job);
    const TSharedPtr<FVoxelLandCpuProgress, ESPMode::ThreadSafe> Progress = MakeShared<FVoxelLandCpuProgress, ESPMode::ThreadSafe>();
    const int32 Side = N + 2 * FVoxelLandBake::Pad;
    Progress->Total = FMath::Max<int64>(1, 6LL * Side * Side + 6LL * N * N / 4);
    CpuProgress = Progress;
    FInputs Inputs = MakeInputs(Settings, SeaRadius, Key);
    Pending = Async(EAsyncExecution::ThreadPool, [Inputs = MoveTemp(Inputs), Progress]()
        {
            return BakeOnCpu(Inputs, &Progress->Cells, &Progress->bCancel);
        });
    State = EState::Cpu;
    if (CVarVoxelDebugLand.GetValueOnGameThread() != 0)
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Baking %08x on the CPU: 6 x %d^2 cells."), Key, N);
    if (VoxelLand::IsErosionEnabled())
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Bake %08x runs on the CPU: the plate model alone, without the erosion solve (it needs the GPU); not cached."), Key);
}

void FVoxelLandBaker::Finish(TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake)
{
    State = EState::Done;
    // The plates, for the debug views (a cached bake doesn't store them; they follow from the
    // seed).
    const VoxelLandBakeDetail::FInputs Inputs = VoxelLandBakeDetail::MakeInputs(Settings, SeaRadius, Key);
    Bake->Plates = Inputs.Plates;
    Bake->PlateSeed = Inputs.Seed;
    Bake->PlateRadiusM = Inputs.RadiusM;
    Bake->PlateWidthScale = Inputs.WidthScale;
    if (!Bake->bFromCache)
        Bake->BakeSeconds = FPlatformTime::Seconds() - StartSeconds;
    Result = Bake;
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Bake %08x ready (%s, %.2f s): 6 x %d^2 cells, heights %.0f .. %.0f m, crust threshold %.4f; %d sea cells in %d sea regions, %d inland basins below sea level."),
        Bake->Key, Bake->bFromCache ? TEXT("from the cache") : TEXT("baked"), FPlatformTime::Seconds() - StartSeconds,
        Bake->N, Bake->MinHeight, Bake->MaxHeight, Bake->CrustThreshold, Bake->SeaCells, Bake->SeaRegions, Bake->BasinRegions);

    // Only eroded bakes are cached: a CPU or erosion-off bake would stand in for the real one.
    if (!Bake->bFromCache && Bake->bEroded && CVarVoxelLandCache.GetValueOnGameThread() != 0)
    {
        TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> ToSave = Bake;
        Async(EAsyncExecution::ThreadPool, [ToSave]()
            {
                if (VoxelLandCache::IsUsable(*ToSave) && !VoxelLandCache::Save(*ToSave))
                    UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Couldn't write the bake cache %s."), *VoxelLandCache::PathFor(ToSave->Key, ToSave->N));
            });
    }
}

// ---------------------------------------------------------------------------------------------
// Sampling parameters
// ---------------------------------------------------------------------------------------------

namespace VoxelLandBakeDetail
{
    /** Two 16-bit cells per word, the lower index in the low half (VoxelLandAccess.ush's
     *  VOXEL_LAND_FLOW and VOXEL_LAND_ATTR). An odd count pads the last word with 0. */
    template <typename FGetBits>
    TArray<uint32> PackHalves(int32 NumCells, FGetBits GetBits)
    {
        TArray<uint32> Packed;
        Packed.SetNumUninitialized((NumCells + 1) / 2);
        for (int32 i = 0; i < Packed.Num(); i++)
        {
            const uint32 Low = GetBits(2 * i);
            const uint32 High = 2 * i + 1 < NumCells ? GetBits(2 * i + 1) : 0u;
            Packed[i] = Low | (High << 16);
        }
        return Packed;
    }
}

// ---------------------------------------------------------------------------------------------
// The rivers (R1b: built on the GPU)
// ---------------------------------------------------------------------------------------------

/** A river build for a loaded bake or new river settings: recorded on the render thread on the
 *  bake's GPU copy (uploaded first if the density hasn't yet) and the receivers, read back, and
 *  recorded again with larger buffers if the first sizes were short. */
struct FVoxelLandRiverJob
{
    uint32 Key = 0;
    VoxelLandRiverBuild::FSettings Settings;
    TWeakPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake;   // its CPU fields, for the first record (weak: the bake owns the job)
    TSharedPtr<FVoxelLandBakeGPU, ESPMode::ThreadSafe> BakeGPU;
    TRefCountPtr<FRDGPooledBuffer> Rec;
    VoxelLandRiverBuild::FPending Pending;
    int32 Builds = 0;
    int32 WaitFrames = 0;
    double StartSeconds = 0.0;
    std::atomic<int32> Status{ 0 };   // 0 running, 1 done (Result), -1 failed
    VoxelLandRiverBuild::FResult Result;
};

namespace VoxelLandBakeDetail
{
    /** Render thread. The bake's fields the density and the rivers read, on the GPU (uploaded
     *  once; a fresh bake's come from its solve): the heights and the tan table, the drainage
     *  (when the bake has it), and with bWater the lake levels. */
    void UploadFields(FRDGBuilder& GraphBuilder, const FVoxelLandBake& Bake, FVoxelLandBakeGPU& Resource, bool bWater)
    {
        if (!Resource.Height.IsValid() || !Resource.TanTable.IsValid())
        {
            Resource.Height = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandHeight"), MakeArrayView(Bake.Height)));
            Resource.TanTable = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandTanTable"), MakeArrayView(Bake.TanTable)));
        }
        if (Bake.Flow.Num() == Bake.NumCells() && !Resource.Flow.IsValid())
        {
            const TArray<uint32> Packed = PackHalves(Bake.Flow.Num(), [&Bake](int32 i) { return (uint32)Bake.Flow[i].Encoded; });
            Resource.Flow = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandFlow"), MakeConstArrayView(Packed)));
        }
        if (bWater && Bake.Water.Num() == Bake.NumCells() && !Resource.Water.IsValid())
        {
            Resource.Water = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandWater"), MakeArrayView(Bake.Water)));
        }
    }

    /** GPU buffers dropped where they were made. */
    void ReleaseOnRenderThread(TRefCountPtr<FRDGPooledBuffer>& A, TRefCountPtr<FRDGPooledBuffer>& B)
    {
        if (!A.IsValid() && !B.IsValid()) return;
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseRiverBuffers)([A = MoveTemp(A), B = MoveTemp(B)](FRHICommandListImmediate&) mutable
            {
                A.SafeRelease();
                B.SafeRelease();
            });
    }

    void ReleaseRiverJob(TSharedPtr<FVoxelLandRiverJob, ESPMode::ThreadSafe>& Job)
    {
        if (!Job.IsValid()) return;
        TSharedPtr<FVoxelLandRiverJob, ESPMode::ThreadSafe> Dropped = MoveTemp(Job);
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseRiverJob)([Dropped](FRHICommandListImmediate&) mutable
            {
                Dropped.Reset();
            });
    }

    /** The river set from a build read back (its GPU tables taken over), or one standing for none
     *  (TilesPerFace 0) when it didn't fit or its data is past the material's exact floats. */
    TSharedPtr<FVoxelLandRiverSet, ESPMode::ThreadSafe> MakeRiverSet(VoxelLandRiverBuild::FResult&& R, uint32 Key, uint32 BakeKey, double MinAreaKm2,
        int32 Builds, double Seconds)
    {
        using namespace VoxelLandRiverBuild;
        const TSharedPtr<FVoxelLandRiverSet, ESPMode::ThreadSafe> Set = MakeShared<FVoxelLandRiverSet, ESPMode::ThreadSafe>();
        Set->Key = Key;
        Set->Builds = Builds;
        Set->BuildSeconds = Seconds;
        // The material holds the tables as floats: exact below 2^24 (64 MB of data).
        if (!R.bFits || R.TilesPerFace <= 0 || R.Data.Num() >= (1 << 24) || R.Tiles.Num() == 0)
        {
            if (R.bFits)
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Bake %08x's rivers have too much data (%d floats); no rivers. Raise RiverMinArea."), BakeKey, R.Data.Num());
            ReleaseOnRenderThread(R.GPUTiles, R.GPUData);
            return Set;
        }
        const uint32* C = R.Counts;
        Set->TilesPerFace = R.TilesPerFace;
        Set->Tiles = MoveTemp(R.Tiles);
        Set->Data = MoveTemp(R.Data);
        Set->RiverCells = (int32)C[CountRiverCells];
        Set->Segments = (int32)C[CountSegments];
        Set->Chains = (int32)C[CountChains];
        Set->Sources = (int32)C[CountSources];
        Set->Confluences = (int32)C[CountConfluences];
        Set->Mouths = (int32)C[CountMouths];
        Set->LakeInlets = (int32)C[CountInlets];
        Set->LakeOutlets = (int32)C[CountOutlets];
        Set->Entries = C[CountEntries];
        Set->Overflow = C[CountDropped];
        Set->MaxInTile = C[CountMaxInTile];
        Set->LengthKm = (double)C[CountLengthDam] * 1.0e-2;
        Set->GPU = MakeShared<FVoxelLandRiverGPU, ESPMode::ThreadSafe>();
        Set->GPU->Tiles = MoveTemp(R.GPUTiles);
        Set->GPU->Data = MoveTemp(R.GPUData);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Rivers of bake %08x from %.0f km^2 (GPU, %d build%s%s): %d river cells, %d segments in %d chains (%.0f km), %d mouths, %d lake inlets, %d outlets; %u tile entries (at most %u in a tile), %.1f MB."),
            BakeKey, MinAreaKm2, Builds, Builds == 1 ? TEXT("") : TEXT("s"), Seconds > 0.0 ? *FString::Printf(TEXT(", %.3f s"), Seconds) : TEXT(""),
            Set->RiverCells, Set->Segments, Set->Chains, Set->LengthKm, Set->Mouths, Set->LakeInlets, Set->LakeOutlets, Set->Entries, Set->MaxInTile,
            (double)(Set->Data.Num() + Set->Tiles.Num()) * 4e-6);
        if (Set->Overflow > 0)
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The rivers overflow their tiles by %u entries (the smallest dropped there; at most %d a tile). Raise RiverMinArea or lower ValleyWidthScale."),
                Set->Overflow, VoxelLandRiverBuild::MaxPerTile);
        return Set;
    }

    /** Render thread: records the build on the bake's GPU fields and the receivers. */
    void RecordRiverJob(FRDGBuilder& GraphBuilder, FVoxelLandRiverJob& Job, const VoxelLandRiverBuild::FCaps& Caps)
    {
        FVoxelLandBakeGPU& G = *Job.BakeGPU;
        VoxelLandRiverBuild::FInputs In;
        In.Tan = GraphBuilder.RegisterExternalBuffer(G.TanTable);
        In.Height = GraphBuilder.RegisterExternalBuffer(G.Height);
        In.Water = GraphBuilder.RegisterExternalBuffer(G.Water);
        In.Flow = GraphBuilder.RegisterExternalBuffer(G.Flow);
        In.Rec = GraphBuilder.RegisterExternalBuffer(Job.Rec);
        const VoxelLandRiverBuild::FOutputs Out = VoxelLandRiverBuild::AddPasses(GraphBuilder, In, Job.Settings, Caps);
        Job.Pending.Enqueue(GraphBuilder, Out, Job.Settings, Caps);
        Job.Builds++;
    }

    /** Render thread, once a frame while the job runs. */
    void TickRiverJob(FRHICommandListImmediate& RHICmdList, FVoxelLandRiverJob& Job)
    {
        if (Job.Status.load(std::memory_order_acquire) != 0) return;
        if (!VoxelLandRiverBuild::AreShadersReady())
        {
            // Still compiling (e.g. after recompileshaders), or not on this platform.
            if (++Job.WaitFrames > 300)
                Job.Status.store(-1, std::memory_order_release);
            return;
        }
        if (Job.Builds > 0 && !Job.Pending.IsReady()) return;
        const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake = Job.Builds == 0 ? Job.Bake.Pin() : nullptr;
        if (Job.Builds == 0 && (!Bake.IsValid() || !Job.BakeGPU.IsValid()))
        {
            Job.Status.store(-1, std::memory_order_release);
            return;
        }
        FRDGBuilder GraphBuilder(RHICmdList);
        if (Job.Builds == 0)
        {
            UploadFields(GraphBuilder, *Bake, *Job.BakeGPU, true);
            Job.Rec = GraphBuilder.ConvertToExternalBuffer(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandRiverReceivers"), MakeArrayView(Bake->Receiver)));
            RecordRiverJob(GraphBuilder, Job, VoxelLandRiverBuild::EstimateCaps(Job.Settings));
        }
        else
        {
            VoxelLandRiverBuild::FResult R;
            if (!Job.Pending.Read(R))
            {
                Job.Rec.SafeRelease();
                Job.Status.store(-1, std::memory_order_release);
            }
            else if (R.bFits)
            {
                Job.Result = MoveTemp(R);
                Job.Rec.SafeRelease();
                Job.Status.store(1, std::memory_order_release);
            }
            else if (R.Counts[VoxelLandRiverBuild::CountErrors] != 0 || Job.Builds >= VoxelLandRiverBuild::MaxBuilds)
            {
                Job.Rec.SafeRelease();
                Job.Status.store(-1, std::memory_order_release);
            }
            else
            {
                RecordRiverJob(GraphBuilder, Job, R.Needed);
            }
        }
        GraphBuilder.Execute();
    }

    bool HasRiverInputs(const FVoxelLandBake& Bake)
    {
        const int32 Cells = Bake.NumCells();
        return Bake.Receiver.Num() == Cells && Bake.Height.Num() == Cells && Bake.Water.Num() == Cells && Bake.Flow.Num() == Cells
            && Bake.TanTable.Num() == Bake.Side();
    }
}

TSharedPtr<const FVoxelLandRiverSet, ESPMode::ThreadSafe> VoxelLand::GetRivers(const FVoxelLandBake& Bake, const FVoxelTerrestrialSettings& Settings)
{
    if (!(Settings.RiverMinArea > 0.0) || !VoxelLandBakeDetail::HasRiverInputs(Bake)) return nullptr;
    const uint32 Key = RiverKey(Settings);
    FScopeLock ScopeLock(&Bake.RiverLock);
    // (A set without tiles stands for none.)
    return Bake.Rivers.IsValid() && Bake.Rivers->Key == Key && Bake.Rivers->TilesPerFace > 0 ? Bake.Rivers : nullptr;
}

bool VoxelLand::TickRivers(const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& BakePtr, const FVoxelTerrestrialSettings& Settings)
{
    using namespace VoxelLandBakeDetail;
    check(IsInGameThread());
    if (!BakePtr.IsValid()) return true;
    const FVoxelLandBake& Bake = *BakePtr;
    if (!(Settings.RiverMinArea > 0.0) || !HasRiverInputs(Bake)) return true;
    const uint32 Key = RiverKey(Settings);
    FScopeLock ScopeLock(&Bake.RiverLock);
    if (Bake.Rivers.IsValid() && Bake.Rivers->Key == Key)
    {
        ReleaseRiverJob(Bake.RiverJob);
        return true;
    }
    if (Bake.RiverJob.IsValid() && Bake.RiverJob->Key != Key)
        ReleaseRiverJob(Bake.RiverJob);
    if (!Bake.RiverJob.IsValid())
    {
        if (!FApp::CanEverRender() || !GIsRHIInitialized || GMaxRHIFeatureLevel < ERHIFeatureLevel::SM5)
        {
            // No renderer (a dedicated server), or no compute shaders for it: no rivers.
            const TSharedPtr<FVoxelLandRiverSet, ESPMode::ThreadSafe> None = MakeShared<FVoxelLandRiverSet, ESPMode::ThreadSafe>();
            None->Key = Key;
            Bake.Rivers = None;
            return true;
        }
        const TSharedPtr<FVoxelLandRiverJob, ESPMode::ThreadSafe> Job = MakeShared<FVoxelLandRiverJob, ESPMode::ThreadSafe>();
        Job->Key = Key;
        Job->Settings.N = Bake.N;
        Job->Settings.Pad = FVoxelLandBake::Pad;
        Job->Settings.RadiusM = (float)(Bake.SeaRadius * 0.01);
        Job->Settings.MinAreaKm2 = (float)Settings.RiverMinArea;
        Job->Settings.WidthScale = (float)FMath::Max(Settings.RiverWidthScale, 0.1);
        Job->Settings.ValleyScale = (float)FMath::Max(Settings.ValleyWidthScale, 0.1);
        uint64 Land = 0;
        for (const uint32 Count : Bake.ProvinceLand) Land += Count;
        Job->Settings.LandCells = (uint32)FMath::Min<uint64>(Land, MAX_uint32);
        Job->Bake = BakePtr;
        Job->BakeGPU = Bake.GPU;
        Job->StartSeconds = FPlatformTime::Seconds();
        Bake.RiverJob = Job;
        ENQUEUE_RENDER_COMMAND(VoxelLandRiverStart)([Job](FRHICommandListImmediate& RHICmdList)
            {
                TickRiverJob(RHICmdList, *Job);
            });
        return false;
    }
    // (No local copy of the job: the render thread must drop its last reference.)
    FVoxelLandRiverJob& Job = *Bake.RiverJob;
    const int32 Status = Job.Status.load(std::memory_order_acquire);
    if (Status == 0)
    {
        ENQUEUE_RENDER_COMMAND(VoxelLandRiverTick)([InJob = Bake.RiverJob](FRHICommandListImmediate& RHICmdList)
            {
                TickRiverJob(RHICmdList, *InJob);
            });
        return false;
    }
    if (Status > 0)
    {
        Bake.Rivers = MakeRiverSet(MoveTemp(Job.Result), Key, Bake.Key, Settings.RiverMinArea, Job.Builds, FPlatformTime::Seconds() - Job.StartSeconds);
    }
    else
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The rivers of bake %08x couldn't be built on the GPU (after %d builds); no rivers."), Bake.Key, Job.Builds);
        const TSharedPtr<FVoxelLandRiverSet, ESPMode::ThreadSafe> None = MakeShared<FVoxelLandRiverSet, ESPMode::ThreadSafe>();
        None->Key = Key;
        Bake.Rivers = None;
    }
    ReleaseRiverJob(Bake.RiverJob);
    return true;
}

FVoxelLandShaderParameters VoxelLandGPU::GetParameters(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density)
{
    FVoxelLandShaderParameters Parameters;
    Parameters.LandMode = 0;
    Parameters.LandN = 1;
    Parameters.LandPad = 0;
    Parameters.LandHasFlow = 0;
    Parameters.LandHasForm = 0;
    Parameters.LandCellM = 1.0f;
    Parameters.LandWarpOctaves = 0;
    Parameters.LandWarpInvNorm = 0.0f;
    Parameters.LandWarpAmplitude = 0.0f;
    Parameters.LandDetailAmplitude = 0.0f;
    Parameters.LandHeightOffset = FVector2f::ZeroVector;
    Parameters.LandGullyStrength = 0.0f;
    Parameters.LandGullyOctaves = 0;
    Parameters.LandGullyFlowGain = 0.0f;
    Parameters.LandGullyDetail = 1.0f;
    Parameters.LandGullySeed = 0;
    Parameters.LandHillRelief = 0.0f;
    Parameters.LandHillFrequency = FVector2f::ZeroVector;
    Parameters.LandHillOctaves = 0;
    Parameters.LandHillSeed = 0;
    Parameters.LandPeakStrength = 0.0f;
    Parameters.LandPeakTexelsPerCell = 0.0f;
    Parameters.LandPeakSeed = 0;
    Parameters.LandPeakSpacingM = 0.0f;
    Parameters.LandPeakReliefM = 0.0f;
    Parameters.LandPeakReliefStart = 500.0f;
    Parameters.LandPeakReliefEnd = 1500.0f;
    Parameters.LandPeakMeanR = 0.0f;
    Parameters.LandPeakMeanG = 0.0f;
    Parameters.LandPeakScaleR = 1.0f;
    Parameters.LandPeakScaleG = 1.0f;
    Parameters.LandPeakScaleB = 1.0f;
    Parameters.LandNoiseN = 0;
    Parameters.LandNoiseMips = 0;
    Parameters.LandDetailBase = 0;
    Parameters.LandDetailN = 0;
    Parameters.LandDetailMips = 0;
    Parameters.LandPeakDetailTexelsPerCell = 0.0f;
    Parameters.LandDetailMeanHard = 0.0f;
    Parameters.LandDetailMeanSoft = 0.0f;
    Parameters.LandPeakDetailScaleHard = 0.0f;
    Parameters.LandPeakDetailScaleSoft = 0.0f;
    Parameters.LandPeakDetailFade = 0.0f;
    Parameters.LandHillTexelsPerCell = 0.0f;
    Parameters.LandHillBase = 0;
    Parameters.LandHillN = 0;
    Parameters.LandHillMips = 0;
    Parameters.LandHillScaleR = 1.0f;
    Parameters.LandHillScaleG = 1.0f;
    Parameters.LandHillScaleB = 1.0f;
    Parameters.LandHillMeanR = 0.0f;
    Parameters.LandHillMeanG = 0.0f;
    Parameters.LandHillSpacingM = 0.0f;
    Parameters.LandHillSwellTexelsPerCell = 0.0f;
    Parameters.LandHillSwellM = 0.0f;
    Parameters.LandHillSwellScaleG = 1.0f;
    Parameters.LandHillDetailTexelsPerCell = 0.0f;
    Parameters.LandHillDetailScaleHard = 0.0f;
    Parameters.LandHillDetailScaleSoft = 0.0f;
    Parameters.LandHillDetailFade = 0.0f;
    Parameters.LandHillFoothill = 0.0f;
    Parameters.LandStrataTexelsPerCell = 0.0f;
    Parameters.LandStrataBase = 0;
    Parameters.LandStrataN = 0;
    Parameters.LandStrataMips = 0;
    Parameters.LandStrataScaleR = 1.0f;
    Parameters.LandStrataScaleG = 1.0f;
    Parameters.LandStrataScaleB = 1.0f;
    Parameters.LandStrataMeanR = 0.0f;
    Parameters.LandStrataMeanG = 0.0f;
    Parameters.LandStrataSpacingM = 0.0f;
    Parameters.LandStrataRelief = 0.0f;
    Parameters.LandStrataFlat = 0.0f;
    Parameters.LandStrataThreshold = 2.0f;
    Parameters.LandStrataWidth = 0.05f;
    Parameters.LandStrataSeed = 0;
    Parameters.LandStrataDetailTexelsPerCell = 0.0f;
    Parameters.LandStrataDetailScaleHard = 0.0f;
    Parameters.LandStrataDetailScaleSoft = 0.0f;
    Parameters.LandStrataDetailFade = 0.0f;
    Parameters.LandRiverTilesPerFace = 0;
    Parameters.LandRiverRadiusM = 1.0f;
    Parameters.LandRiverWidthScale = 1.0f;
    Parameters.LandRiverValleyScale = 1.0f;
    Parameters.LandHasWater = 0;
    Parameters.LandRiverCarve = 0.0f;
    Parameters.LandRiverDepthScale = 1.0f;
    Parameters.LandRiverFloodplainScale = 1.0f;
    Parameters.LandRiverMinLog2Area = 0.0f;
    Parameters.LandLakeShoreBand = 0.0f;
    Parameters.LandRiverMeander = 0.0f;
    Parameters.LandRiverFalls = 0.0f;
    Parameters.LandDetailTexelsPerCell = 0.0f;
    Parameters.LandDetailScales = 0;
    Parameters.LandDetailMeanA = 0.0f;
    Parameters.LandDetailScaleA = 0.0f;
    Parameters.LandDetailScaleGain = 0.0f;
    Parameters.LandDetailFade = 0.0f;
    Parameters.LandDetailSeed = 0;
    // T5c: the dunes (off; LandClimate bound below, a stand-in without them).
    Parameters.LandHasClimate = 0;
    Parameters.LandClimateLapse = 6.5f;
    Parameters.LandDuneTexelsPerCell = 0.0f;
    Parameters.LandDuneBase = 0;
    Parameters.LandDuneN = 0;
    Parameters.LandDuneMips = 0;
    Parameters.LandDuneScaleR = 1.0f;
    Parameters.LandDuneScaleG = 1.0f;
    Parameters.LandDuneMeanR = 0.0f;
    Parameters.LandDuneMeanG = 0.0f;
    Parameters.LandDuneRelief = 0.0f;
    Parameters.LandDuneSeed = 0;
    Parameters.LandDuneErgFrequency = 1.0f;
    Parameters.LandDuneErgThreshold = 2.0f;
    Parameters.LandDuneErgWidth = 0.06f;
    Parameters.LandDuneReliefMax = 400.0f;
    Parameters.LandDuneDetailTexelsPerCell = 0.0f;
    Parameters.LandDuneDetailBase = 0;
    Parameters.LandDuneDetailN = 0;
    Parameters.LandDuneDetailMips = 0;
    Parameters.LandDuneDetailScale = 0.0f;
    Parameters.LandDuneDetailMean = 0.0f;
    Parameters.LandDuneDetailFade = 0.0f;
    // T5d: the glacial layer (off).
    Parameters.LandGlacialTexelsPerCell = 0.0f;
    Parameters.LandGlacialBase = 0;
    Parameters.LandGlacialN = 0;
    Parameters.LandGlacialMips = 0;
    Parameters.LandGlacialScaleR = 1.0f;
    Parameters.LandGlacialScaleG = 1.0f;
    Parameters.LandGlacialScaleB = 1.0f;
    Parameters.LandGlacialMeanR = 0.0f;
    Parameters.LandGlacialMeanG = 0.0f;
    Parameters.LandGlacialReliefM = 0.0f;
    Parameters.LandGlacialStrength = 0.0f;
    Parameters.LandGlacialSeed = 0;
    Parameters.LandGlacialSpacingM = 0.0f;
    Parameters.LandGlacialCooling = 0.0f;
    Parameters.LandGlacialDetailScaleHard = 0.0f;
    Parameters.LandGlacialDetailScaleSoft = 0.0f;
    // T5d2: the ice sheets (off).
    Parameters.LandIceMaxThickness = 0.0f;
    Parameters.LandIceBlend = 0.0f;
    Parameters.LandIceSwellTexelsPerCell = 0.0f;
    Parameters.LandIceBase = 0;
    Parameters.LandIceN = 0;
    Parameters.LandIceMips = 0;
    Parameters.LandIceMeanG = 0.0f;
    Parameters.LandIceSwellM = 0.0f;
    Parameters.LandIceSeed = 0;
    Parameters.LandIceSwellSpacingM = 0.0f;
    Parameters.LandPolarBase = 0;
    Parameters.LandPolarN = 0;
    Parameters.LandPolarMips = 0;
    Parameters.LandPolarSeed = 0;
    Parameters.LandPolarCrevasseTexelsPerCell = 0.0f;
    Parameters.LandPolarCrevasseM = 0.0f;
    Parameters.LandPolarCrevasseMean = 0.0f;
    Parameters.LandPolarCrevasseFade = 0.0f;
    Parameters.LandPolarSastrugiTexelsPerCell = 0.0f;
    Parameters.LandPolarSastrugiM = 0.0f;
    Parameters.LandPolarSastrugiMean = 0.0f;
    Parameters.LandPolarSastrugiFade = 0.0f;
    Parameters.LandPolarPolygonTexelsPerCell = 0.0f;
    Parameters.LandPolarPolygonM = 0.0f;
    Parameters.LandPolarPolygonMean = 0.0f;
    Parameters.LandPolarPolygonFade = 0.0f;
    Parameters.LandPolarPermafrostT = -6.0f;
    Parameters.LandPolarCrevasseTurn = 0.0f;
    Parameters.LandPolarSastrugiTurn = 0.0f;
    Parameters.LandPolarDuneBase = 0;
    Parameters.LandPolarDuneN = 0;
    Parameters.LandPolarDuneMips = 0;
    Parameters.LandPolarDuneTexelsPerCell = 0.0f;
    Parameters.LandPolarDuneM = 0.0f;
    Parameters.LandPolarDuneMean = 0.0f;
    Parameters.LandPolarDuneFade = 0.0f;
    bool bClimateBound = false;

    const FVoxelLandBake* Bake = Density.LandMode != 0 ? Density.Land.Get() : nullptr;
    if (Bake && Bake->GPU.IsValid() && Bake->Height.Num() == Bake->NumCells() && Bake->TanTable.Num() == Bake->Side())
    {
        FVoxelLandBakeGPU& Resource = *Bake->GPU;
        // The heights and the drainage (the drainage steers the gullies' strength, and shows in
        // the drainage view): a fresh bake's from its solve, else uploaded once.
        const bool bHasFlow = Bake->Flow.Num() == Bake->NumCells();
        VoxelLandBakeDetail::UploadFields(GraphBuilder, *Bake, Resource, false);
        // The landform shapes the hills and the peaks; the local relief scales the peaks.
        const bool bHasForm = Bake->Form.Num() == Bake->NumCells() && Bake->Relief.Num() == Bake->NumCells();
        if (bHasForm && !Resource.Form.IsValid())
        {
            const TArray<uint32> Packed = VoxelLandBakeDetail::PackHalves(Bake->Form.Num(), [Bake](int32 i) { return (uint32)Bake->Form[i]; });
            Resource.Form = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandForm"), MakeConstArrayView(Packed)));
        }
        if (bHasForm && !Resource.Relief.IsValid())
        {
            // Four cells per word, the lowest index in the low byte.
            TArray<uint32> Packed;
            Packed.SetNumZeroed((Bake->Relief.Num() + 3) / 4);
            for (int32 i = 0; i < Bake->Relief.Num(); i++)
                Packed[i >> 2] |= (uint32)Bake->Relief[i] << ((i & 3) * 8);
            Resource.Relief = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandRelief"), MakeConstArrayView(Packed)));
        }
        if (bHasForm && !Resource.Province.IsValid())
        {
            // The province (T4c), as the relief (zeros if the bake has none).
            TArray<uint32> Packed;
            Packed.SetNumZeroed((Bake->NumCells() + 3) / 4);
            if (Bake->Province.Num() == Bake->NumCells())
                for (int32 i = 0; i < Bake->Province.Num(); i++)
                    Packed[i >> 2] |= (uint32)FMath::Min(Bake->Province[i], (uint8)63) << ((i & 3) * 8);
            Resource.Province = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandProvince"), MakeConstArrayView(Packed)));
        }
        Parameters.LandForm = bHasForm ? GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Form))
            : GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandFormNone"), TConstArrayView<uint32>()));
        Parameters.LandRelief = bHasForm ? GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Relief))
            : GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandReliefNone"), TConstArrayView<uint32>()));
        Parameters.LandProvince = bHasForm ? GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Province))
            : GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandProvinceNone"), TConstArrayView<uint32>()));
        // The rivers (R1): the GPU build's own tables (R1b), else uploaded once per river set
        // (shared by the planet's densities).
        const FVoxelLandRiverSet* Rivers = Density.LandRivers.Get();
        if (Rivers && Rivers->GPU.IsValid() && Rivers->TilesPerFace > 0 && Rivers->Tiles.Num() > 0 && Rivers->Data.Num() > 0)
        {
            FVoxelLandRiverGPU& RG = *Rivers->GPU;
            if (!RG.Tiles.IsValid() || !RG.Data.IsValid())
            {
                RG.Tiles = GraphBuilder.ConvertToExternalBuffer(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandRiverTiles"), MakeConstArrayView(Rivers->Tiles)));
                RG.Data = GraphBuilder.ConvertToExternalBuffer(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandRiverData"), MakeConstArrayView(Rivers->Data)));
            }
            Parameters.LandRiverTiles = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(RG.Tiles));
            Parameters.LandRiverData = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(RG.Data));
            Parameters.LandRiverTilesPerFace = Density.LandRiverTilesPerFace;
            Parameters.LandRiverRadiusM = Density.LandRiverRadiusM;
            Parameters.LandRiverWidthScale = Density.LandRiverWidthScale;
            Parameters.LandRiverValleyScale = Density.LandRiverValleyScale;
        }
        else
        {
            Parameters.LandRiverTiles = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandRiverTilesNone"), TConstArrayView<uint32>()));
            Parameters.LandRiverData = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandRiverDataNone"), TConstArrayView<float>()));
        }
        // R2: the rivers' and lakes' carve. The lake levels are bound whenever the bake has them
        // (a fresh bake's from its solve, else uploaded once): the carve reads them when
        // LandHasWater, the hydrology's debug views whenever they show.
        Parameters.LandRiverCarve = Density.LandRiverCarve;
        Parameters.LandRiverDepthScale = Density.LandRiverDepthScale;
        Parameters.LandRiverFloodplainScale = Density.LandRiverFloodplainScale;
        Parameters.LandRiverMinLog2Area = Density.LandRiverMinLog2Area;
        Parameters.LandLakeShoreBand = Density.LandLakeShoreBand;
        Parameters.LandRiverMeander = Density.LandRiverMeander;
        Parameters.LandRiverFalls = Density.LandRiverFalls;
        const bool bWater = Bake->Water.Num() == Bake->NumCells();
        if (bWater)
            VoxelLandBakeDetail::UploadFields(GraphBuilder, *Bake, Resource, true);
        bool bWaterBound = false;
        if (bWater && Resource.Water.IsValid())
        {
            Parameters.LandWater = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Water));
            bWaterBound = true;
        }
        else
        {
            Parameters.LandWater = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandWaterNone"), TConstArrayView<float>()));
        }
        Parameters.LandHasWater = bWaterBound && Density.LandHasWater != 0 ? 1u : 0u;
        // The noise pack: the peaks', detail, hills' and strata's sets (shared by the planets that
        // use it: uploaded once).
        const FVoxelLandNoiseSet* Noise = Density.LandNoise.Get();
        const bool bHasNoise = Noise && Noise->GPU.IsValid() && Noise->IsComplete();
        if (bHasNoise && !Noise->GPU->Words.IsValid())
        {
            Noise->GPU->Words = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandNoise"), MakeConstArrayView(Noise->Words)));
        }
        Parameters.LandNoise = bHasNoise ? GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Noise->GPU->Words))
            : GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandNoiseNone"), TConstArrayView<uint32>()));
        if (bHasNoise)
        {
            Parameters.LandPeakStrength = Density.LandPeakStrength;
            Parameters.LandPeakTexelsPerCell = Density.LandPeakTexelsPerCell;
            Parameters.LandPeakSeed = Density.LandPeakSeed;
            Parameters.LandPeakSpacingM = Density.LandPeakSpacingM;
            Parameters.LandPeakReliefM = Density.LandPeakReliefM;
            Parameters.LandPeakReliefStart = Density.LandPeakReliefStart;
            Parameters.LandPeakReliefEnd = Density.LandPeakReliefEnd;
            Parameters.LandPeakMeanR = Density.LandPeakMeanR;
            Parameters.LandPeakMeanG = Density.LandPeakMeanG;
            Parameters.LandPeakScaleR = Density.LandPeakScaleR;
            Parameters.LandPeakScaleG = Density.LandPeakScaleG;
            Parameters.LandPeakScaleB = Density.LandPeakScaleB;
            Parameters.LandNoiseN = Noise->N;
            Parameters.LandNoiseMips = Noise->Mips;
            // The detail set's chain and means, whoever reads it (the detail layer, the peaks, the
            // hills' and the strata's sets).
            Parameters.LandDetailBase = Noise->DetailBase;
            Parameters.LandDetailN = Noise->DetailN;
            Parameters.LandDetailMips = Noise->DetailMips;
            Parameters.LandDetailMeanHard = Density.LandDetailMeanHard;
            Parameters.LandDetailMeanSoft = Density.LandDetailMeanSoft;
            // N1: the detail layer.
            if (Density.LandDetailTexelsPerCell > 0.0f && Noise->DetailN > 0)
            {
                Parameters.LandDetailTexelsPerCell = Density.LandDetailTexelsPerCell;
                Parameters.LandDetailScales = Density.LandDetailScales;
                Parameters.LandDetailMeanA = Density.LandDetailMeanA;
                Parameters.LandDetailScaleA = Density.LandDetailScaleA;
                Parameters.LandDetailScaleGain = Density.LandDetailScaleGain;
                Parameters.LandDetailFade = Density.LandDetailFade;
                Parameters.LandDetailSeed = Density.LandDetailSeed;
            }
            if (Density.LandPeakDetailTexelsPerCell > 0.0f && Noise->DetailN > 0)
            {
                Parameters.LandPeakDetailTexelsPerCell = Density.LandPeakDetailTexelsPerCell;
                Parameters.LandPeakDetailScaleHard = Density.LandPeakDetailScaleHard;
                Parameters.LandPeakDetailScaleSoft = Density.LandPeakDetailScaleSoft;
                Parameters.LandPeakDetailFade = Density.LandPeakDetailFade;
            }
            // The hills' set (T4a).
            if (Density.LandHillTexelsPerCell > 0.0f && Noise->HillN > 0)
            {
                Parameters.LandHillTexelsPerCell = Density.LandHillTexelsPerCell;
                Parameters.LandHillBase = Noise->HillBase;
                Parameters.LandHillN = Noise->HillN;
                Parameters.LandHillMips = Noise->HillMips;
                Parameters.LandHillScaleR = Density.LandHillScaleR;
                Parameters.LandHillScaleG = Density.LandHillScaleG;
                Parameters.LandHillScaleB = Density.LandHillScaleB;
                Parameters.LandHillMeanR = Density.LandHillMeanR;
                Parameters.LandHillMeanG = Density.LandHillMeanG;
                Parameters.LandHillSpacingM = Density.LandHillSpacingM;
                Parameters.LandHillSwellTexelsPerCell = Density.LandHillSwellTexelsPerCell;
                Parameters.LandHillSwellM = Density.LandHillSwellM;
                Parameters.LandHillSwellScaleG = Density.LandHillSwellScaleG;
                Parameters.LandHillDetailTexelsPerCell = Density.LandHillDetailTexelsPerCell;
                Parameters.LandHillDetailScaleHard = Density.LandHillDetailScaleHard;
                Parameters.LandHillDetailScaleSoft = Density.LandHillDetailScaleSoft;
                Parameters.LandHillDetailFade = Density.LandHillDetailFade;
                Parameters.LandHillFoothill = Density.LandHillFoothill;
                if (Density.LandStrataTexelsPerCell > 0.0f && Noise->StrataN > 0)
                {
                    Parameters.LandStrataTexelsPerCell = Density.LandStrataTexelsPerCell;
                    Parameters.LandStrataBase = Noise->StrataBase;
                    Parameters.LandStrataN = Noise->StrataN;
                    Parameters.LandStrataMips = Noise->StrataMips;
                    Parameters.LandStrataScaleR = Density.LandStrataScaleR;
                    Parameters.LandStrataScaleG = Density.LandStrataScaleG;
                    Parameters.LandStrataScaleB = Density.LandStrataScaleB;
                    Parameters.LandStrataMeanR = Density.LandStrataMeanR;
                    Parameters.LandStrataMeanG = Density.LandStrataMeanG;
                    Parameters.LandStrataSpacingM = Density.LandStrataSpacingM;
                    Parameters.LandStrataRelief = Density.LandStrataRelief;
                    Parameters.LandStrataFlat = Density.LandStrataFlat;
                    Parameters.LandStrataThreshold = Density.LandStrataThreshold;
                    Parameters.LandStrataWidth = Density.LandStrataWidth;
                    Parameters.LandStrataSeed = Density.LandStrataSeed;
                    Parameters.LandStrataDetailTexelsPerCell = Density.LandStrataDetailTexelsPerCell;
                    Parameters.LandStrataDetailScaleHard = Density.LandStrataDetailScaleHard;
                    Parameters.LandStrataDetailScaleSoft = Density.LandStrataDetailScaleSoft;
                    Parameters.LandStrataDetailFade = Density.LandStrataDetailFade;
                }
            }
        }
        // T5c, T5d, T5d2, T5d3: the climate the dunes, the glacial layer, the ice sheets and the polar
        // surfaces are placed by (uploaded once per field), and those layers (the sets' with the
        // noise pack).
        const FVoxelLandClimateField* Climate = Density.LandClimate.Get();
        const bool bDunes = bHasNoise && Density.LandDuneTexelsPerCell > 0.0f && Density.LandHillTexelsPerCell > 0.0f && Noise->HillN > 0 && Noise->DuneN > 0;
        const bool bGlacial = bHasNoise && Density.LandGlacialTexelsPerCell > 0.0f && Density.LandPeakTexelsPerCell > 0.0f && Noise->GlacialN > 0;
        const bool bIce = Density.LandIceMaxThickness > 0.0f;
        const bool bPolar = bHasNoise && Noise->PolarN > 0
            && (Density.LandPolarCrevasseTexelsPerCell > 0.0f || Density.LandPolarSastrugiTexelsPerCell > 0.0f || Density.LandPolarPolygonTexelsPerCell > 0.0f
                || Density.LandPolarDuneTexelsPerCell > 0.0f);
        // T5e: the ice shelf's share (the rivers' fade, the polar layer's ice) is in the field too.
        const bool bShelf = Density.bLandIceShelf;
        if ((bDunes || bGlacial || bIce || bPolar || bShelf) && Climate && Climate->IsValid() && Climate->GPU.IsValid())
        {
            if (!Climate->GPU->Entries.IsValid())
            {
                Climate->GPU->Entries = GraphBuilder.ConvertToExternalBuffer(
                    VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandClimate"), MakeConstArrayView(Climate->Entries)));
            }
            Parameters.LandClimate = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Climate->GPU->Entries));
            bClimateBound = true;
            Parameters.LandHasClimate = 1;
            Parameters.LandClimateLapse = Density.LandClimateLapse;
            if (bDunes)
            {
                Parameters.LandDuneBase = Noise->DuneBase;
                Parameters.LandDuneN = Noise->DuneN;
                Parameters.LandDuneMips = Noise->DuneMips;
                Parameters.LandDuneDetailBase = Noise->SandBase;
                Parameters.LandDuneDetailN = Noise->SandN;
                Parameters.LandDuneDetailMips = Noise->SandMips;
                Parameters.LandDuneTexelsPerCell = Density.LandDuneTexelsPerCell;
                Parameters.LandDuneScaleR = Density.LandDuneScaleR;
                Parameters.LandDuneScaleG = Density.LandDuneScaleG;
                Parameters.LandDuneMeanR = Density.LandDuneMeanR;
                Parameters.LandDuneMeanG = Density.LandDuneMeanG;
                Parameters.LandDuneRelief = Density.LandDuneRelief;
                Parameters.LandDuneSeed = Density.LandDuneSeed;
                Parameters.LandDuneErgFrequency = Density.LandDuneErgFrequency;
                Parameters.LandDuneErgThreshold = Density.LandDuneErgThreshold;
                Parameters.LandDuneErgWidth = Density.LandDuneErgWidth;
                Parameters.LandDuneReliefMax = Density.LandDuneReliefMax;
                Parameters.LandDuneDetailTexelsPerCell = Density.LandDuneDetailTexelsPerCell;
                Parameters.LandDuneDetailScale = Density.LandDuneDetailScale;
                Parameters.LandDuneDetailMean = Density.LandDuneDetailMean;
                Parameters.LandDuneDetailFade = Density.LandDuneDetailFade;
            }
            if (bGlacial)
            {
                Parameters.LandGlacialBase = Noise->GlacialBase;
                Parameters.LandGlacialN = Noise->GlacialN;
                Parameters.LandGlacialMips = Noise->GlacialMips;
                Parameters.LandGlacialTexelsPerCell = Density.LandGlacialTexelsPerCell;
                Parameters.LandGlacialScaleR = Density.LandGlacialScaleR;
                Parameters.LandGlacialScaleG = Density.LandGlacialScaleG;
                Parameters.LandGlacialScaleB = Density.LandGlacialScaleB;
                Parameters.LandGlacialMeanR = Density.LandGlacialMeanR;
                Parameters.LandGlacialMeanG = Density.LandGlacialMeanG;
                Parameters.LandGlacialReliefM = Density.LandGlacialReliefM;
                Parameters.LandGlacialStrength = Density.LandGlacialStrength;
                Parameters.LandGlacialSeed = Density.LandGlacialSeed;
                Parameters.LandGlacialSpacingM = Density.LandGlacialSpacingM;
                Parameters.LandGlacialCooling = Density.LandGlacialCooling;
                Parameters.LandGlacialDetailScaleHard = Density.LandGlacialDetailScaleHard;
                Parameters.LandGlacialDetailScaleSoft = Density.LandGlacialDetailScaleSoft;
            }
            if (bIce)
            {
                Parameters.LandIceMaxThickness = Density.LandIceMaxThickness;
                Parameters.LandIceBlend = Density.LandIceBlend;
                if (bHasNoise && Density.LandIceSwellTexelsPerCell > 0.0f && Noise->IceN > 0)
                {
                    Parameters.LandIceSwellTexelsPerCell = Density.LandIceSwellTexelsPerCell;
                    Parameters.LandIceBase = Noise->IceBase;
                    Parameters.LandIceN = Noise->IceN;
                    Parameters.LandIceMips = Noise->IceMips;
                    Parameters.LandIceMeanG = Density.LandIceMeanG;
                    Parameters.LandIceSwellM = Density.LandIceSwellM;
                    Parameters.LandIceSeed = Density.LandIceSeed;
                    Parameters.LandIceSwellSpacingM = Density.LandIceSwellSpacingM;
                }
            }
            if (bPolar)
            {
                Parameters.LandPolarBase = Noise->PolarBase;
                Parameters.LandPolarN = Noise->PolarN;
                Parameters.LandPolarMips = Noise->PolarMips;
                Parameters.LandPolarSeed = Density.LandPolarSeed;
                Parameters.LandPolarCrevasseTexelsPerCell = Density.LandPolarCrevasseTexelsPerCell;
                Parameters.LandPolarCrevasseM = Density.LandPolarCrevasseM;
                Parameters.LandPolarCrevasseMean = Density.LandPolarCrevasseMean;
                Parameters.LandPolarCrevasseFade = Density.LandPolarCrevasseFade;
                Parameters.LandPolarSastrugiTexelsPerCell = Density.LandPolarSastrugiTexelsPerCell;
                Parameters.LandPolarSastrugiM = Density.LandPolarSastrugiM;
                Parameters.LandPolarSastrugiMean = Density.LandPolarSastrugiMean;
                Parameters.LandPolarSastrugiFade = Density.LandPolarSastrugiFade;
                Parameters.LandPolarPolygonTexelsPerCell = Density.LandPolarPolygonTexelsPerCell;
                Parameters.LandPolarPolygonM = Density.LandPolarPolygonM;
                Parameters.LandPolarPolygonMean = Density.LandPolarPolygonMean;
                Parameters.LandPolarPolygonFade = Density.LandPolarPolygonFade;
                Parameters.LandPolarPermafrostT = Density.LandPolarPermafrostT;
                Parameters.LandPolarCrevasseTurn = Density.LandPolarCrevasseTurn;
                Parameters.LandPolarSastrugiTurn = Density.LandPolarSastrugiTurn;
                if (Density.LandPolarDuneTexelsPerCell > 0.0f && Noise->DuneN > 0)
                {
                    Parameters.LandPolarDuneBase = Noise->DuneBase;
                    Parameters.LandPolarDuneN = Noise->DuneN;
                    Parameters.LandPolarDuneMips = Noise->DuneMips;
                    Parameters.LandPolarDuneTexelsPerCell = Density.LandPolarDuneTexelsPerCell;
                    Parameters.LandPolarDuneM = Density.LandPolarDuneM;
                    Parameters.LandPolarDuneMean = Density.LandPolarDuneMean;
                    Parameters.LandPolarDuneFade = Density.LandPolarDuneFade;
                }
            }
        }
        if (!bClimateBound)
            Parameters.LandClimate = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<FVector4f>(GraphBuilder, TEXT("Voxel.LandClimateNone"), TConstArrayView<FVector4f>()));
        Parameters.LandHasForm = bHasForm ? 1u : 0u;
        Parameters.LandHeight = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Height));
        Parameters.LandTanTable = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.TanTable));
        Parameters.LandFlow = bHasFlow ? GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Flow))
            : GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandFlowNone"), TConstArrayView<uint32>()));
        Parameters.LandMode = 1;
        Parameters.LandN = (uint32)Bake->N;
        Parameters.LandPad = (uint32)FVoxelLandBake::Pad;
        Parameters.LandHasFlow = bHasFlow ? 1u : 0u;
        Parameters.LandCellM = Density.LandCellM;
        Parameters.LandWarpOctaves = Density.LandWarpOctaves;
        Parameters.LandWarpInvNorm = Density.LandWarpInvNorm;
        Parameters.LandWarpAmplitude = Density.LandWarpAmplitude;
        Parameters.LandDetailAmplitude = Density.LandDetailAmplitude;
        Parameters.LandHeightOffset = Density.LandHeightOffset;
        Parameters.LandGullyStrength = Density.LandGullyStrength;
        Parameters.LandGullyOctaves = Density.LandGullyOctaves;
        Parameters.LandGullyFlowGain = Density.LandGullyFlowGain;
        Parameters.LandGullyDetail = Density.LandGullyDetail;
        Parameters.LandGullySeed = Density.LandGullySeed;
        // The set's relief without the set would be the procedural hills at its height: none then
        // (the noise pack is always there when the density has it).
        Parameters.LandHillRelief = Density.LandHillTexelsPerCell > 0.0f && Parameters.LandHillTexelsPerCell <= 0.0f ? 0.0f : Density.LandHillRelief;
        Parameters.LandHillFrequency = Density.LandHillFrequency;
        Parameters.LandHillOctaves = Density.LandHillOctaves;
        Parameters.LandHillSeed = Density.LandHillSeed;
        return Parameters;
    }

    // The noise surface: one-element stand-ins (the shader's land branch never runs).
    Parameters.LandHeight = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandHeightNone"), TConstArrayView<float>()));
    Parameters.LandTanTable = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<FVector2f>(GraphBuilder, TEXT("Voxel.LandTanTableNone"), TConstArrayView<FVector2f>()));
    Parameters.LandFlow = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandFlowNone"), TConstArrayView<uint32>()));
    Parameters.LandForm = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandFormNone"), TConstArrayView<uint32>()));
    Parameters.LandRelief = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandReliefNone"), TConstArrayView<uint32>()));
    Parameters.LandProvince = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandProvinceNone"), TConstArrayView<uint32>()));
    Parameters.LandRiverTiles = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandRiverTilesNone"), TConstArrayView<uint32>()));
    Parameters.LandRiverData = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandRiverDataNone"), TConstArrayView<float>()));
    Parameters.LandNoise = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandNoiseNone"), TConstArrayView<uint32>()));
    Parameters.LandWater = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandWaterNone"), TConstArrayView<float>()));
    Parameters.LandClimate = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<FVector4f>(GraphBuilder, TEXT("Voxel.LandClimateNone"), TConstArrayView<FVector4f>()));
    return Parameters;
}

FVoxelLandDebugParameters VoxelLandGPU::GetDebugParameters(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density, int32 Mode, double SeaRadius)
{
    FVoxelLandDebugParameters Parameters;
    Parameters.LandDebugMode = (uint32)FMath::Max(Mode, 0);
    Parameters.LandDebugSeaRadius = VoxelDoubleFloat(FMath::Max(SeaRadius, 0.0));
    Parameters.LandPlateCount = 0;
    Parameters.LandSeed = 0;
    Parameters.LandDebugHasHydro = 0;
    Parameters.LandDebugHasAttr = 0;
    // The plate model's other inputs: the debug views read only the plates, the seed, and the
    // radius and width scale (the warp's; set with the plates below).
    Parameters.LandRadiusM = 1.0f;
    Parameters.LandWidthScale = 1.0f;
    Parameters.LandReliefScale = 1.0f;
    Parameters.LandShelfDepth = 0.0f;
    Parameters.LandAbyssDepth = 0.0f;
    Parameters.LandCoastHeight = 0.0f;
    Parameters.LandInteriorHeight = 0.0f;

    const FVoxelLandBake* Bake = Mode > 0 && Density.LandMode != 0 ? Density.Land.Get() : nullptr;
    if (Bake && Bake->GPU.IsValid() && Bake->Attr.Num() == Bake->NumCells())
    {
        FVoxelLandBakeGPU& Resource = *Bake->GPU;
        if (!Resource.Attr.IsValid())
        {
            const TArray<uint32> Packed = VoxelLandBakeDetail::PackHalves(Bake->Attr.Num(), [Bake](int32 i) { return (uint32)Bake->Attr[i]; });
            Resource.Attr = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandAttr"), MakeConstArrayView(Packed)));
        }
        Parameters.LandAttr = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Attr));
        Parameters.LandDebugHasAttr = 1;
        // The plates, when the bake kept them (without, the plates view shows "none"; the
        // attributes and lakes don't need them: the land surface cache's fields read those).
        if (Bake->Plates.Num() >= 8)
        {
            if (!Resource.Plates.IsValid())
            {
                Resource.Plates = GraphBuilder.ConvertToExternalBuffer(
                    VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandDebugPlates"), MakeArrayView(Bake->Plates)));
            }
            Parameters.LandPlates = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Plates));
            Parameters.LandPlateCount = (uint32)(Bake->Plates.Num() / 8);
            Parameters.LandSeed = Bake->PlateSeed;
            Parameters.LandRadiusM = Bake->PlateRadiusM;
            Parameters.LandWidthScale = Bake->PlateWidthScale;
        }
        else
        {
            Parameters.LandPlates = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandPlatesNone"), TConstArrayView<float>()));
        }
        // The views that show the hydrology, when the bake has it (the drainage and, from R2,
        // the lake levels are the density's: VoxelLandGPU::GetParameters binds them).
        // VOXEL_LAND_VIEW_FLOW, _LAKES, _WATER, _TERRAIN_WATER.
        const bool bHydro = (Mode == 9 || Mode == 10 || Mode == 12 || Mode == 13) && Bake->Flow.Num() == Bake->NumCells() && Bake->Water.Num() == Bake->NumCells();
        if (bHydro)
            Parameters.LandDebugHasHydro = 1;
        return Parameters;
    }

    // No view, or the noise surface: one-element stand-ins (the bake views show "none").
    Parameters.LandAttr = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandAttrNone"), TConstArrayView<uint32>()));
    Parameters.LandPlates = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandPlatesNone"), TConstArrayView<float>()));
    return Parameters;
}
