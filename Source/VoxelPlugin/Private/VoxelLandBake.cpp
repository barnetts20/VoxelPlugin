#include "VoxelLandBake.h"
#include "VoxelLandGPU.h"
#include "VoxelLandSolve.h"
#include "VoxelDensityGPU.h"
#include "VoxelGPUCommon.h"
#include "VoxelLandRelief.h"
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
    TEXT("voxel.Land.BakeCellsPerFrame"), 1 << 20,
    TEXT("Bake cells the GPU evaluates per frame (the plate model costs a few microseconds per cell on the CPU, far less on the GPU)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandErosion(
    TEXT("voxel.Land.Erosion"), 1,
    TEXT("1: terrestrial bakes run the erosion solve on the GPU (stage L2). 0: the plate model alone (for comparison; part of the cache key, so switching rebakes once)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandSolveStepCells(
    TEXT("voxel.Land.SolveStepCells"), 1 << 22,
    TEXT("The erosion solve's budget per frame: solved cells x steps (about 70 passes over the mountain cells per step)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandSolveIterationCells(
    TEXT("voxel.Land.SolveIterationCells"), 1 << 26,
    TEXT("The fills', drainage's and lake labels' budget per frame: cells x iterations (one cheap pass each), until they converge."),
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
                }
                if (Progress) Progress->fetch_add(Chunk, std::memory_order_relaxed);
            }, EParallelForFlags::BackgroundPriority);
        if (Cancelled()) return nullptr;
        Bake->UpdateRange();
        Bake->BuildRelief();
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
        RoutingRandomness, DrainageSpreadCells, PlainsUndulation, LandformErosion })
        H = HashCombine(H, ::GetTypeHash(V));
    H = HashCombine(H, ::GetTypeHash(SolveMaxSteps));
    return H;
}

uint32 FVoxelTerrestrialSettings::GetSettingsHash() const
{
    uint32 H = GetBakeHash();
    for (const double V : { BakeWarp, DetailAmplitude, DetailWavelength, DetailSlopeDamping,
        GullyStrength, GullyFinestWavelength, GullyFlowGain, GullyDetailScale, HillRelief, HillWavelength, HillFinestWavelength,
        PeakStrength, PeakFinestWavelength })
        H = HashCombine(H, ::GetTypeHash(V));
    // FString's hash is found by argument-dependent lookup only (not at global scope).
    const FString NoiseSetName = PeakNoiseSet.TrimStartAndEnd();
    H = HashCombine(H, GetTypeHash(NoiseSetName));
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
    // The GPU copy is released on the render thread, which made it.
    if (GPU.IsValid())
    {
        TSharedPtr<FVoxelLandBakeGPU, ESPMode::ThreadSafe> Resource = MoveTemp(GPU);
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseBake)([Resource](FRHICommandListImmediate&) mutable
            {
                Resource.Reset();
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

// ---------------------------------------------------------------------------------------------
// The peaks' noise set
// ---------------------------------------------------------------------------------------------

FVoxelLandNoiseSet::FVoxelLandNoiseSet()
    : GPU(MakeShared<FVoxelLandNoiseGPU, ESPMode::ThreadSafe>())
{
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

    /** A set by name. The densities that sample it hold it; the registry only watches it (and
     *  keeps the few most recent alive: Recent), so a set nobody uses any more goes, with its GPU
     *  copy, and is read again if asked for. */
    struct FEntry
    {
        TWeakPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Set;
        bool bReady = false;
        bool bNone = false;       // ready, and none could be made (the peaks are off)
        bool bPending = false;
    };

    // The sets kept alive whatever uses them: those just made, so a planet that waited for one
    // finds it when it builds its density.
    constexpr int32 RecentCount = 4;

    FCriticalSection Lock;        // Entries, Recent, the folder's list, the flags (held briefly)
    FCriticalSection MakeLock;    // one set made at a time (held while reading or generating)
    TMap<FString, FEntry> Entries;
    TArray<TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>> Recent;
    FString Directory;            // NoiseSetDirectory, found on the game thread (StartupModule)
    bool bDirectoryKnown = false;
    TArray<FString> AuthoredPeaks;
    bool bListed = false;
    bool bExitHooked = false;
    bool bClosed = false;         // ReleasePeakNoise ran (the engine is exiting)

    /** The sampler's set from a checked FSet's header (CheckPeaks) and its words (the chain,
     *  padded to whole rows: FVoxelLandNoiseSet::Words). */
    TSharedPtr<FVoxelLandNoiseSet, ESPMode::ThreadSafe> Finish(const VoxelNoiseTiles::FSet& Set, TArray<uint32>&& Words)
    {
        const TSharedPtr<FVoxelLandNoiseSet, ESPMode::ThreadSafe> Out = MakeShared<FVoxelLandNoiseSet, ESPMode::ThreadSafe>();
        Out->Name = UTF8_TO_TCHAR(Set.Name.c_str());
        Out->LookTag = UTF8_TO_TCHAR(Set.LookTag.c_str());
        Out->N = Set.N;
        Out->Mips = Set.Mips;
        Out->Rows = (uint32)((VoxelNoiseTiles::ChainTexels(Set.N) + Set.N - 1) / Set.N);
        Out->CellShift = (uint32)FMath::FloorLog2(Set.N / Set.Channel[0].CellsPerTile);
        Out->Words = MoveTemp(Words);
        // The means in unit values (0..1 over each channel's range): what the shader sees.
        auto Unit = [&Set](int Channel, double V) {
            const VoxelNoiseTiles::FChannel& C = Set.Channel[Channel];
            return (V - (double)C.Min) / FMath::Max((double)C.Max - (double)C.Min, 1e-12);
        };
        // The summit-scaled pyramid's mean (VOXEL_LAND_PEAK_SUMMIT_LO and _RANGE: 0.6 + 0.8 A),
        // and the ridged channel's.
        const uint16* Texels = (const uint16*)Out->Words.GetData();
        Out->MeanShape = (float)VoxelNoiseTiles::MeanOf2(Set, Texels, 0, 3, [&](double R, double A) { return (0.6 + 0.8 * Unit(3, A)) * Unit(0, R); });
        Out->MeanG = (float)VoxelNoiseTiles::MeanOf(Set, Texels, 1, [&](double V) { return Unit(1, V); });
        return Out;
    }

    /** The plugin's own `peaks` set. */
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Generate()
    {
        const double Start = FPlatformTime::Seconds();
        VoxelNoiseTiles::FSet Set = VoxelNoiseTiles::PeaksDefinition(PeakSetSeed);
        VoxelNoiseTiles::Generate(Set, [](int32 Count, auto&& Body)
            {
                ParallelFor(Count, [&Body](int32 i) { Body(i); }, EParallelForFlags::BackgroundPriority);
            });
        Set.Name = TCHAR_TO_UTF8(VoxelLand::GeneratedNoiseName);
        const std::string Error = VoxelNoiseTiles::CheckPeaks(Set);
        if (!Error.empty() || !Set.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The generated peaks' noise set is malformed (%s); the peaks are off."), UTF8_TO_TCHAR(Error.c_str()));
            return nullptr;
        }
        const uint32 Rows = (uint32)((VoxelNoiseTiles::ChainTexels(Set.N) + Set.N - 1) / Set.N);
        TArray<uint32> Words;
        Words.SetNumZeroed(2 * Rows * Set.N);
        FMemory::Memcpy(Words.GetData(), Set.Texels.data(), Set.Texels.size() * sizeof(uint16));
        const TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Out = Finish(Set, MoveTemp(Words));
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Generated the peaks' noise set (%u^2, %u mips) in %.2f s."), Out->N, Out->Mips,
            FPlatformTime::Seconds() - Start);
        return Out;
    }

    /** An authored set from NoiseSetDirectory()/Name.vxnt: the header checked, then the texels
     *  read straight into the words. Null (and Error) if it can't be used. */
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Read(const FString& Name, FString& Error)
    {
        const double Start = FPlatformTime::Seconds();
        const FString Folder = VoxelLand::NoiseSetDirectory();
        if (Folder.IsEmpty())
        {
            Error = TEXT("the plugin's NoiseSets folder wasn't found");
            return nullptr;
        }
        const FString Path = FPaths::Combine(Folder, Name + TEXT(".vxnt"));
        const TUniquePtr<FArchive> File(IFileManager::Get().CreateFileReader(*Path));
        if (!File.IsValid())
        {
            Error = FString::Printf(TEXT("%s isn't there"), *Path);
            return nullptr;
        }
        const int64 FileSize = File->TotalSize();
        if (FileSize <= 0)
        {
            Error = FString::Printf(TEXT("%s is empty or unreadable"), *Path);
            return nullptr;
        }
        TArray<uint8> Header;
        Header.SetNumZeroed((int32)FMath::Min<int64>(FileSize, 4096));
        File->Serialize(Header.GetData(), Header.Num());
        VoxelNoiseTiles::FSet Set;
        uint64_t DataOffset = 0;
        std::string Why = VoxelNoiseTiles::ReadVxntHeader(Header.GetData(), (size_t)Header.Num(), (uint64_t)FileSize, Set, DataOffset);
        if (Why.empty())
            Why = VoxelNoiseTiles::CheckPeaks(Set);
        if (!Why.empty() || File->IsError())
        {
            Error = FString::Printf(TEXT("%s: %s"), *Path, Why.empty() ? TEXT("unreadable") : UTF8_TO_TCHAR(Why.c_str()));
            return nullptr;
        }
        const uint64 Texels = VoxelNoiseTiles::ChainTexels(Set.N);
        const uint32 Rows = (uint32)((Texels + Set.N - 1) / Set.N);
        TArray<uint32> Words;
        Words.SetNumUninitialized(2 * Rows * Set.N);
        // The texels as they lie (u16 little-endian, R G B A: two words a texel), then the
        // padding of the last row.
        File->Seek((int64)DataOffset);
        File->Serialize(Words.GetData(), (int64)(Texels * 8));
        FMemory::Memzero(Words.GetData() + 2 * Texels, (Words.Num() - 2 * Texels) * sizeof(uint32));
        if (File->IsError())
        {
            Error = FString::Printf(TEXT("%s: the texels couldn't be read"), *Path);
            return nullptr;
        }
        const TSharedPtr<FVoxelLandNoiseSet, ESPMode::ThreadSafe> Out = Finish(Set, MoveTemp(Words));
        // The name it's asked by (the file's), whatever the header calls it.
        Out->Name = Name;
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Read the peaks' noise set %s (%u^2, %u mips, \"%s\") in %.2f s."), *Name, Out->N, Out->Mips,
            *Out->LookTag, FPlatformTime::Seconds() - Start);
        return Out;
    }

    /** Stores Set under Name (Lock taken here). */
    void Store(const FString& Name, const TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Set)
    {
        FScopeLock ScopeLock(&Lock);
        if (bClosed) return;
        FEntry& Entry = Entries.FindOrAdd(Name);
        Entry.Set = Set;
        Entry.bNone = !Set.IsValid();
        Entry.bReady = true;
        if (Set.IsValid())
        {
            Recent.Remove(Set);
            Recent.Insert(Set, 0);
            if (Recent.Num() > RecentCount) Recent.SetNum(RecentCount);
        }
        // Dropped before the RHI goes (each set's GPU copy is released on the render thread).
        if (!bExitHooked)
        {
            bExitHooked = true;
            FCoreDelegates::OnEnginePreExit.AddStatic(&VoxelLand::ReleasePeakNoise);
        }
    }

    /** Whether Name's set is there (Lock held): its set (null if none could be made). */
    bool FindReadyLocked(const FString& Name, TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Out)
    {
        const FEntry* Entry = Entries.Find(Name);
        if (!Entry || !Entry->bReady) return false;
        Out = Entry->Set.Pin();
        return Entry->bNone || Out.IsValid();
    }

    bool FindReady(const FString& Name, TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Out)
    {
        FScopeLock ScopeLock(&Lock);
        return FindReadyLocked(Name, Out);
    }

    /** The generated set (MakeLock held). */
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> GeneratedLocked()
    {
        TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Set;
        if (FindReady(VoxelLand::GeneratedNoiseName, Set)) return Set;
        Set = Generate();
        Store(VoxelLand::GeneratedNoiseName, Set);
        return Set;
    }

    /** A set name as the setting gives it: trimmed, without a .vxnt; empty if it names a path. */
    FString CleanName(const FString& In)
    {
        FString Name = In.TrimStartAndEnd();
        Name.RemoveFromEnd(TEXT(".vxnt"), ESearchCase::IgnoreCase);
        if (Name.Contains(TEXT("/")) || Name.Contains(TEXT("\\")) || Name.Contains(TEXT("..")) || Name.Contains(TEXT(":")))
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] PeakNoiseSet \"%s\" isn't a set name (a file name in %s without .vxnt); picking by the seed."),
                *In, *VoxelLand::NoiseSetDirectory());
            return FString();
        }
        return Name;
    }

    /** Lists the folder's usable peaks sets (Lock held, Directory known; once a session): their
     *  headers read and checked, so a file the peaks can't use doesn't take a share of the
     *  planets. A few small reads, on the game thread the first time a planet asks. */
    void ListLocked()
    {
        if (bListed) return;
        bListed = true;
        const FString Folder = Directory;
        TArray<FString> Files;
        if (!Folder.IsEmpty())
            IFileManager::Get().FindFiles(Files, *FPaths::Combine(Folder, TEXT("peaks*.vxnt")), true, false);
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
            if (Why.empty()) Why = VoxelNoiseTiles::CheckPeaks(Set);
            if (Why.empty())
                AuthoredPeaks.Add(FPaths::GetBaseFilename(File));
            else
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] %s isn't a usable peaks set (%s); left out."), *Path, UTF8_TO_TCHAR(Why.c_str()));
        }
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Peaks noise sets in %s: %s."), *Folder,
            AuthoredPeaks.Num() > 0 ? *FString::Join(AuthoredPeaks, TEXT(", ")) : TEXT("none (the generated set stands in)"));
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
    const FString Chosen = CleanName(Settings.PeakNoiseSet);
    if (!Chosen.IsEmpty()) return Chosen;
    NoiseSetDirectory();
    FScopeLock ScopeLock(&Lock);
    ListLocked();
    if (AuthoredPeaks.Num() == 0) return GeneratedNoiseName;
    // By the seed, rendezvous-hashed (the set with the highest hash of the seed and its name):
    // adding or removing a set moves only the planets that pick it. Name it in PeakNoiseSet to
    // keep a planet's set whatever the folder holds.
    const FString* Best = nullptr;
    uint32 BestScore = 0;
    for (const FString& Name : AuthoredPeaks)
    {
        const uint32 Score = VoxelNoiseTiles::Mix((uint32)Settings.Seed ^ VoxelNoiseTiles::Mix(FCrc::StrCrc32(*Name.ToLower()) + 0x7065616bu));
        if (!Best || Score > BestScore) { Best = &Name; BestScore = Score; }
    }
    return *Best;
}

TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> VoxelLand::GetPeakNoise(const FString& Name)
{
    using namespace VoxelLandNoiseDetail;
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Set;
    if (FindReady(Name, Set)) return Set;
    FScopeLock MakeScope(&MakeLock);
    // Made meanwhile by another thread?
    if (FindReady(Name, Set)) return Set;
    if (Name == GeneratedNoiseName) return GeneratedLocked();
    FString Error;
    Set = Read(Name, Error);
    if (!Set.IsValid())
    {
        // Kept under the name while a planet uses it (the error isn't repeated meanwhile); asked
        // for again after that, the file is read again.
        UE_LOG(LogVoxelPlugin, Error, TEXT("[VoxelLand] The peaks' noise set %s can't be used (%s); the generated set stands in."), *Name, *Error);
        Set = GeneratedLocked();
    }
    Store(Name, Set);
    return Set;
}

bool VoxelLand::TryGetPeakNoise(const FString& Name, TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& Out)
{
    using namespace VoxelLandNoiseDetail;
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Set;
    if (!FindReady(Name, Set)) return false;
    Out = MoveTemp(Set);
    return true;
}

void VoxelLand::PrewarmPeakNoise(const FString& Name)
{
    using namespace VoxelLandNoiseDetail;
    {
        FScopeLock ScopeLock(&Lock);
        if (bClosed) return;
        TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Set;
        if (FindReadyLocked(Name, Set)) return;
        FEntry& Entry = Entries.FindOrAdd(Name);
        if (Entry.bPending) return;
        Entry.bPending = true;
    }
    Async(EAsyncExecution::ThreadPool, [Name]()
        {
            GetPeakNoise(Name);
            FScopeLock ScopeLock(&VoxelLandNoiseDetail::Lock);
            if (VoxelLandNoiseDetail::FEntry* Entry = VoxelLandNoiseDetail::Entries.Find(Name))
                Entry->bPending = false;
        });
}

void VoxelLand::ReleasePeakNoise()
{
    using namespace VoxelLandNoiseDetail;
    FScopeLock ScopeLock(&Lock);
    // The engine is exiting: a set still being made is handed to its caller but not kept.
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
        Settings.PlainsUndulation, Settings.LandformErosion };
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

int32 VoxelLand::PeakOctavesFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake)
{
    if (Settings.PeakStrength <= 0.0) return 0;
    // From two bake cells (the gullies' lambda0), halving, while at least the finest.
    const double Lambda0 = 2.0 * CellArc(Bake.SeaRadius, Bake.N) * 0.01;
    const double Finest = FMath::Max(Settings.PeakFinestWavelength, 1.0);
    if (Lambda0 < Finest) return 1;
    return FMath::Clamp(FMath::FloorToInt(FMath::Log2(Lambda0 / Finest)) + 1, 1, PeakMaxOctaves);
}

double VoxelLand::PeakReach(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake, bool bUp)
{
    // The amplitude is at most the strength times the largest local relief; the shape within
    // PeakMinUnit .. PeakMaxUnit (the face blend and the B-splines are convex).
    if (PeakOctavesFor(Settings, Bake) == 0) return 0.0;
    return (bUp ? PeakMaxUnit : PeakMinUnit) * FMath::Max(Settings.PeakStrength, 0.0) * (double)Bake.MaxRelief * 100.0;
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

void VoxelLand::ShapeFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake,
    double& OutPlanetRadius, double& OutNoiseAmplitude)
{
    // The B-spline and the face blend are convex combinations of the cells, so the bake's
    // range holds; the detail reaches at most about 2.4 times its amplitude (MakeLayer's
    // normalisation), the gullies GullyReach either way, the hills HillMaxUnit times their
    // relief (up; down by less), the peaks PeakReach up (and down from the land's lowest, sea
    // level), and a metre of margin either side.
    const double DetailReach = 2.5 * FMath::Max(Settings.DetailAmplitude, 0.0) * 100.0;
    const double HillReach = HillMaxUnit * FMath::Max(Settings.HillRelief, 0.0) * 100.0;
    const double Reach = DetailReach + GullyReach(Settings, Bake) + HillReach + 100.0;
    const double Low = FMath::Min((double)Bake.MinHeight * 100.0, -PeakReach(Settings, Bake, false)) - Reach;
    const double High = (double)Bake.MaxHeight * 100.0 + PeakReach(Settings, Bake, true) + Reach;
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

    /** The raw (uncompressed) size of a bake's data: heights, attributes, landform, then for an
     *  eroded bake the drainage and the lake levels, then the lakes. */
    int64 RawBytes(int64 NumCells, bool bEroded, int64 NumLakes)
    {
        return NumCells * (int64)(sizeof(float) + 2 * sizeof(uint16))
            + (bEroded ? NumCells * (int64)(sizeof(uint16) + sizeof(float)) : 0)
            + NumLakes * (int64)sizeof(FVoxelLandLake);
    }

    /** Bakes kept in Saved/VoxelLand; the least recently written beyond this are deleted. */
    constexpr int32 MaxFiles = 16;

    /** A bake fit to keep or use: finite heights and a range. */
    bool IsUsable(const FVoxelLandBake& Bake)
    {
        if (Bake.Height.Num() != Bake.NumCells() || Bake.Attr.Num() != Bake.NumCells() || Bake.Form.Num() != Bake.NumCells()) return false;
        if (Bake.bEroded && (Bake.Flow.Num() != Bake.NumCells() || Bake.Water.Num() != Bake.NumCells())) return false;
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
        if (Bake.bEroded)
        {
            Append(Bake.Flow.GetData(), NumCells * sizeof(uint16));
            Append(Bake.Water.GetData(), NumCells * sizeof(float));
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
        Take(Bake->Height.GetData(), NumCells * sizeof(float));
        Take(Bake->Attr.GetData(), NumCells * sizeof(uint16));
        Take(Bake->Form.GetData(), NumCells * sizeof(uint16));
        if (Bake->bEroded)
        {
            Bake->Flow.SetNumUninitialized((int32)NumCells);
            Bake->Water.SetNumUninitialized((int32)NumCells);
            Take(Bake->Flow.GetData(), NumCells * sizeof(uint16));
            Take(Bake->Water.GetData(), NumCells * sizeof(float));
        }
        Bake->Lakes.SetNumUninitialized(Header.LakeCount);
        Take(Bake->Lakes.GetData(), (int64)Header.LakeCount * sizeof(FVoxelLandLake));
        Bake->BuildTanTable();
        Bake->UpdateRange();
        Bake->BuildRelief();
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
        S.bErode = VoxelLand::IsErosionEnabled();
        S.StepCellsPerFrame = FMath::Max(CVarVoxelLandSolveStepCells.GetValueOnGameThread(), 1 << 16);
        S.IterationCellsPerFrame = FMath::Max(CVarVoxelLandSolveIterationCells.GetValueOnGameThread(), 1 << 18);
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
            const bool bOk = R.Height.Num() == NumCells && R.Attr.Num() == NumCells && R.Flow.Num() == NumCells && R.Water.Num() == NumCells
                && R.Form.Num() == NumCells;
            Job.ReleaseBuffers();
            Job.Status.store(bOk ? 1 : -1, std::memory_order_release);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// FVoxelLandBaker
// ---------------------------------------------------------------------------------------------

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
    // The peaks' noise set, read or generated meanwhile (the density takes it once the bake is
    // ready).
    if (InSettings.PeakStrength > 0.0)
        VoxelLand::PrewarmPeakNoise(VoxelLand::PeakNoiseNameFor(InSettings));
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
        if (CVarVoxelDebugLand.GetValueOnGameThread() != 0)
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Solve phases: %s."), *FString::Join(R->PhaseLog, TEXT(", ")));
        const TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = MakeBake(Job->Inputs);
        Bake->CrustThreshold = Job->OutThreshold;
        Bake->bEroded = Job->bErode;
        Bake->SolveSteps = R->Steps;
        Bake->SolveResidual = R->Residual;
        Bake->ReliefFactor = R->Factor;
        ReleaseJob(Job);
        Pending = Async(EAsyncExecution::ThreadPool, [Bake, R]() -> TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe>
            {
                const int32 Num = R->Height.Num();
                Bake->Height = MoveTemp(R->Height);
                Bake->Water = MoveTemp(R->Water);
                Bake->Lakes = MoveTemp(R->Lakes);
                Bake->Attr.SetNumUninitialized(Num);
                Bake->Flow.SetNumUninitialized(Num);
                Bake->Form.SetNumUninitialized(Num);
                for (int32 i = 0; i < Num; i++)
                {
                    Bake->Attr[i] = (uint16)R->Attr[i];
                    Bake->Form[i] = (uint16)R->Form[i];
                    if (!FMath::IsFinite(R->Flow[i])) return nullptr;
                    Bake->Flow[i] = FFloat16(R->Flow[i]);
                }
                Bake->UpdateRange();
                Bake->BuildRelief();
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
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Bake %08x ready (%s, %.2f s): 6 x %d^2 cells, heights %.0f .. %.0f m, crust threshold %.4f."),
        Bake->Key, Bake->bFromCache ? TEXT("from the cache") : TEXT("baked"), FPlatformTime::Seconds() - StartSeconds,
        Bake->N, Bake->MinHeight, Bake->MaxHeight, Bake->CrustThreshold);

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
    Parameters.LandPeakOctaves = 0;
    Parameters.LandPeakSeed = 0;
    Parameters.LandNoiseN = 0;
    Parameters.LandNoiseMips = 0;
    Parameters.LandNoiseCellShift = 0;
    Parameters.LandPeakMeanShape = 0.0f;
    Parameters.LandPeakMeanG = 0.0f;

    const FVoxelLandBake* Bake = Density.LandMode != 0 ? Density.Land.Get() : nullptr;
    if (Bake && Bake->GPU.IsValid() && Bake->Height.Num() == Bake->NumCells() && Bake->TanTable.Num() == Bake->Side())
    {
        FVoxelLandBakeGPU& Resource = *Bake->GPU;
        if (!Resource.Height.IsValid() || !Resource.TanTable.IsValid())
        {
            Resource.Height = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandHeight"), MakeArrayView(Bake->Height)));
            Resource.TanTable = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandTanTable"), MakeArrayView(Bake->TanTable)));
        }
        // The drainage steers the gullies' strength (and shows in the drainage view).
        const bool bHasFlow = Bake->Flow.Num() == Bake->NumCells();
        if (bHasFlow && !Resource.Flow.IsValid())
        {
            const TArray<uint32> Packed = VoxelLandBakeDetail::PackHalves(Bake->Flow.Num(), [Bake](int32 i) { return (uint32)Bake->Flow[i].Encoded; });
            Resource.Flow = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandFlow"), MakeConstArrayView(Packed)));
        }
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
        Parameters.LandForm = bHasForm ? GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Form))
            : GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandFormNone"), TConstArrayView<uint32>()));
        Parameters.LandRelief = bHasForm ? GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Relief))
            : GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandReliefNone"), TConstArrayView<uint32>()));
        // The peaks' noise set (shared by the planets that use it: uploaded once).
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
            Parameters.LandPeakOctaves = Density.LandPeakOctaves;
            Parameters.LandPeakSeed = Density.LandPeakSeed;
            Parameters.LandNoiseN = Noise->N;
            Parameters.LandNoiseMips = Noise->Mips;
            Parameters.LandNoiseCellShift = Noise->CellShift;
            Parameters.LandPeakMeanShape = Noise->MeanShape;
            Parameters.LandPeakMeanG = Noise->MeanG;
        }
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
        Parameters.LandHillRelief = Density.LandHillRelief;
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
    Parameters.LandNoise = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandNoiseNone"), TConstArrayView<uint32>()));
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
        // The lake levels, for the views that show the hydrology, when the bake has them (the
        // drainage is the density's, VoxelLandGPU::GetParameters).
        // VOXEL_LAND_VIEW_FLOW, _LAKES, _WATER, _TERRAIN_WATER.
        const bool bHydro = (Mode == 9 || Mode == 10 || Mode == 12 || Mode == 13) && Bake->Flow.Num() == Bake->NumCells() && Bake->Water.Num() == Bake->NumCells();
        if (bHydro && !Resource.Water.IsValid())
        {
            Resource.Water = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandWater"), MakeArrayView(Bake->Water)));
        }
        if (bHydro)
        {
            Parameters.LandWater = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Water));
            Parameters.LandDebugHasHydro = 1;
        }
        else
        {
            Parameters.LandWater = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandWaterNone"), TConstArrayView<float>()));
        }
        return Parameters;
    }

    // No view, or the noise surface: one-element stand-ins (the bake views show "none").
    Parameters.LandAttr = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandAttrNone"), TConstArrayView<uint32>()));
    Parameters.LandPlates = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandPlatesNone"), TConstArrayView<float>()));
    Parameters.LandWater = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandWaterNone"), TConstArrayView<float>()));
    return Parameters;
}
