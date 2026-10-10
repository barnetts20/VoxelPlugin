#include "VoxelLandBake.h"
#include "VoxelLandGPU.h"
#include "VoxelDensityGPU.h"
#include "VoxelGPUCommon.h"
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
#include "Misc/Paths.h"
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
                }
                if (Progress) Progress->fetch_add(Chunk, std::memory_order_relaxed);
            }, EParallelForFlags::BackgroundPriority);
        if (Cancelled()) return nullptr;
        Bake->UpdateRange();
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
    for (const double V : { ContinentalFraction, LandformWidthScale, RangeRelief, CoastHeight, InteriorHeight, ShelfDepth, AbyssDepth })
        H = HashCombine(H, ::GetTypeHash(V));
    return H;
}

uint32 FVoxelTerrestrialSettings::GetSettingsHash() const
{
    uint32 H = GetBakeHash();
    for (const double V : { BakeWarp, DetailAmplitude, DetailWavelength, DetailSlopeDamping })
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

// ---------------------------------------------------------------------------------------------
// Keys, shape, cache
// ---------------------------------------------------------------------------------------------

int32 VoxelLand::BakeResolutionFor(const FVoxelTerrestrialSettings& Settings)
{
    return FMath::Clamp(Settings.BakeResolution, 64, 2048);
}

uint32 VoxelLand::BakeKey(const FVoxelTerrestrialSettings& Settings, double SeaRadius)
{
    uint32 H = Settings.GetBakeHash();
    H = HashCombine(H, ::GetTypeHash((int64)FMath::RoundToDouble(SeaRadius)));
    H = HashCombine(H, ::GetTypeHash(BakeResolutionFor(Settings)));
    H = HashCombine(H, ::GetTypeHash(FVoxelLandBake::Version));
    H = HashCombine(H, ::GetTypeHash(GeneratorVersion));
    return H;
}

uint32 VoxelLand::BakeCheck(const FVoxelTerrestrialSettings& Settings, double SeaRadius)
{
    // Every bake input in one buffer, CRC'd: independent of BakeKey's hash combine.
    TArray<double> Values = { (double)Settings.Seed, (double)Settings.PlateCount, (double)BakeResolutionFor(Settings),
        Settings.ContinentalFraction, Settings.LandformWidthScale, Settings.RangeRelief, Settings.CoastHeight,
        Settings.InteriorHeight, Settings.ShelfDepth, Settings.AbyssDepth, FMath::RoundToDouble(SeaRadius),
        (double)FVoxelLandBake::Version, (double)GeneratorVersion };
    return FCrc::MemCrc32(Values.GetData(), Values.Num() * (int32)sizeof(double));
}

void VoxelLand::ShapeFor(const FVoxelTerrestrialSettings& Settings, const FVoxelLandBake& Bake,
    double& OutPlanetRadius, double& OutNoiseAmplitude)
{
    // The B-spline and the face blend are convex combinations of the cells, so the bake's
    // range holds; the detail reaches at most about 2.4 times its amplitude (MakeLayer's
    // normalisation), and a metre of margin either side.
    const double DetailReach = 2.5 * FMath::Max(Settings.DetailAmplitude, 0.0) * 100.0;
    const double Low = (double)Bake.MinHeight * 100.0 - DetailReach - 100.0;
    const double High = (double)Bake.MaxHeight * 100.0 + DetailReach + 100.0;
    OutPlanetRadius = FMath::Max(Bake.SeaRadius + Low, 1.0);
    OutNoiseAmplitude = FMath::Max(High - Low, 1.0);
}

namespace VoxelLandCache
{
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
        uint32 Reserved = 0;
    };

    /** Bakes kept in Saved/VoxelLand; the least recently written beyond this are deleted. */
    constexpr int32 MaxFiles = 16;

    /** A bake fit to keep or use: finite heights and a range. */
    bool IsUsable(const FVoxelLandBake& Bake)
    {
        if (Bake.Height.Num() != Bake.NumCells() || Bake.Attr.Num() != Bake.NumCells()) return false;
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
        const int64 HeightBytes = (int64)Bake.Height.Num() * sizeof(float);
        const int64 AttrBytes = (int64)Bake.Attr.Num() * sizeof(uint16);
        if (HeightBytes + AttrBytes >= MAX_int32) return false;
        TArray<uint8> Raw;
        Raw.SetNumUninitialized((int32)(HeightBytes + AttrBytes));
        FMemory::Memcpy(Raw.GetData(), Bake.Height.GetData(), HeightBytes);
        FMemory::Memcpy(Raw.GetData() + HeightBytes, Bake.Attr.GetData(), AttrBytes);

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
            || (int64)Header.RawSize != NumCells * (int64)(sizeof(float) + sizeof(uint16))
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
        Bake->Height.SetNumUninitialized((int32)NumCells);
        Bake->Attr.SetNumUninitialized((int32)NumCells);
        FMemory::Memcpy(Bake->Height.GetData(), Raw.GetData(), NumCells * sizeof(float));
        FMemory::Memcpy(Bake->Attr.GetData(), Raw.GetData() + NumCells * sizeof(float), NumCells * sizeof(uint16));
        Bake->BuildTanTable();
        Bake->UpdateRange();
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

/** A GPU bake in flight: its inputs, the buffers kept between frames (render thread), and the
 *  results the readback leaves for the game thread. */
struct FVoxelLandBakeJob
{
    VLB::FInputs Inputs;
    int32 CellsPerStep = 1 << 20;

    TRefCountPtr<FRDGPooledBuffer> Plates;
    TRefCountPtr<FRDGPooledBuffer> Histogram;
    TRefCountPtr<FRDGPooledBuffer> Threshold;
    TRefCountPtr<FRDGPooledBuffer> Height;
    TRefCountPtr<FRDGPooledBuffer> Attr;
    TUniquePtr<FRHIGPUBufferReadback> HeightReadback;
    TUniquePtr<FRHIGPUBufferReadback> AttrReadback;
    TUniquePtr<FRHIGPUBufferReadback> ThresholdReadback;

    // Written on the render thread before Status is set.
    TArray<float> OutHeight;
    TArray<uint16> OutAttr;
    float OutThreshold = 0.0f;
    float OutMin = 0.0f;
    float OutMax = 0.0f;

    /** 0 running, 1 the results are in, -1 failed (the game thread bakes on the CPU). */
    std::atomic<int32> Status{ 0 };

    int32 NumCells() const
    {
        const int32 Side = Inputs.N + 2 * FVoxelLandBake::Pad;
        return 6 * Side * Side;
    }

    int32 NumInterior() const { return 6 * Inputs.N * Inputs.N; }
    int32 Chunk() const { return FMath::Max(CellsPerStep, 4096); }

    /** Crust steps (the last also finds the threshold), then field steps (the last also copies
     *  the results for readback). */
    int32 CrustSteps() const { return FMath::DivideAndRoundUp(NumInterior(), Chunk()); }
    int32 FieldSteps() const { return FMath::DivideAndRoundUp(NumCells(), Chunk()); }
    int32 NumSteps() const { return CrustSteps() + FieldSteps(); }
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
            && ShaderMap->GetShader(&FVoxelLandFieldsCS::GetStaticType(), 0).IsValid();
    }

    FIntVector GroupsFor(uint32 Threads, uint32& OutGroupsX)
    {
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(Threads, 64u), 1u);
        OutGroupsX = FMath::Min(NumGroups, 32768u);
        return FIntVector((int32)OutGroupsX, (int32)FMath::DivideAndRoundUp(NumGroups, OutGroupsX), 1);
    }

    /** Render thread. One step of the bake (FVoxelLandBakeJob::NumSteps): a chunk of the crust
     *  histogram (the first creates the buffers, the last finds the threshold), or a chunk of
     *  the fields (the last copies the results for readback). */
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
        if (Step == 0)
        {
            Plates = VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandPlates"), MakeArrayView(In.Plates));
            Histogram = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), HistogramBins), TEXT("Voxel.LandHistogram"));
            Threshold = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), 1), TEXT("Voxel.LandThreshold"));
            Height = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), NumCells), TEXT("Voxel.LandBakeHeight"));
            Attr = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), NumCells), TEXT("Voxel.LandBakeAttr"));
        }
        else
        {
            Plates = GraphBuilder.RegisterExternalBuffer(Job.Plates);
            Histogram = GraphBuilder.RegisterExternalBuffer(Job.Histogram);
            Threshold = GraphBuilder.RegisterExternalBuffer(Job.Threshold);
            Height = GraphBuilder.RegisterExternalBuffer(Job.Height);
            Attr = GraphBuilder.RegisterExternalBuffer(Job.Attr);
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
                Job.HeightReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.LandHeightReadback"));
                Job.AttrReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.LandAttrReadback"));
                Job.ThresholdReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.LandThresholdReadback"));
                AddEnqueueCopyPass(GraphBuilder, Job.HeightReadback.Get(), Height, NumCells * (uint32)sizeof(float));
                AddEnqueueCopyPass(GraphBuilder, Job.AttrReadback.Get(), Attr, NumCells * (uint32)sizeof(uint32));
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
        }
        GraphBuilder.Execute();
    }

    /** Render thread, once per frame after the last step: true once the results are copied
     *  (or the job failed). Narrows the attributes and finds the height range here, off the
     *  game thread; a non-finite height fails the job (the CPU bakes instead). */
    bool PollReadback(FVoxelLandBakeJob& Job)
    {
        if (Job.Status.load(std::memory_order_acquire) != 0) return true;
        if (!Job.HeightReadback.IsValid() || !Job.AttrReadback.IsValid() || !Job.ThresholdReadback.IsValid()) return false;
        if (!Job.HeightReadback->IsReady() || !Job.AttrReadback->IsReady() || !Job.ThresholdReadback->IsReady()) return false;
        const int32 NumCells = Job.NumCells();
        bool bOk = true;
        if (const float* Source = (const float*)Job.HeightReadback->Lock((uint32)NumCells * (uint32)sizeof(float)))
        {
            Job.OutHeight.SetNumUninitialized(NumCells);
            FMemory::Memcpy(Job.OutHeight.GetData(), Source, (SIZE_T)NumCells * sizeof(float));
            Job.HeightReadback->Unlock();
        }
        else bOk = false;
        if (const uint32* Source = (const uint32*)Job.AttrReadback->Lock((uint32)NumCells * (uint32)sizeof(uint32)))
        {
            Job.OutAttr.SetNumUninitialized(NumCells);
            for (int32 i = 0; i < NumCells; i++)
                Job.OutAttr[i] = (uint16)Source[i];
            Job.AttrReadback->Unlock();
        }
        else bOk = false;
        if (const void* Source = Job.ThresholdReadback->Lock((uint32)sizeof(float)))
        {
            FMemory::Memcpy(&Job.OutThreshold, Source, sizeof(float));
            Job.ThresholdReadback->Unlock();
        }
        else bOk = false;
        if (bOk)
        {
            float Min = TNumericLimits<float>::Max();
            float Max = TNumericLimits<float>::Lowest();
            for (const float H : Job.OutHeight)
            {
                if (!FMath::IsFinite(H))
                {
                    bOk = false;
                    break;
                }
                Min = FMath::Min(Min, H);
                Max = FMath::Max(Max, H);
            }
            Job.OutMin = Min;
            Job.OutMax = Max;
        }
        Job.HeightReadback.Reset();
        Job.AttrReadback.Reset();
        Job.ThresholdReadback.Reset();
        Job.Plates.SafeRelease();
        Job.Histogram.SafeRelease();
        Job.Threshold.SafeRelease();
        Job.Height.SafeRelease();
        Job.Attr.SafeRelease();
        Job.Status.store(bOk ? 1 : -1, std::memory_order_release);
        return true;
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
    case EState::Gpu: return GpuSteps > 0 ? 0.95f * (float)GpuStep / (float)GpuSteps : 0.0f;
    case EState::GpuReading: return 0.97f;
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
        {
            VoxelGPUCommon::PollRenderThread([InJob]() -> bool
                {
                    return PollReadback(*InJob);
                });
            State = EState::GpuReading;
        }
        break;
    }
    case EState::GpuReading:
    {
        const int32 Status = Job->Status.load(std::memory_order_acquire);
        if (Status == 0) break;
        if (Status < 0)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The GPU bake failed; baking on the CPU."));
            StartCpu();
            break;
        }
        const TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake = MakeBake(Job->Inputs);
        Bake->CrustThreshold = Job->OutThreshold;
        Bake->Height = MoveTemp(Job->OutHeight);
        Bake->Attr = MoveTemp(Job->OutAttr);
        Bake->MinHeight = Job->OutMin;
        Bake->MaxHeight = Job->OutMax;
        ReleaseJob(Job);
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
}

void FVoxelLandBaker::Finish(TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Bake)
{
    State = EState::Done;
    // The plates, for the debug views (a cached bake doesn't store them; they follow from the
    // seed).
    const VoxelLandBakeDetail::FInputs Inputs = VoxelLandBakeDetail::MakeInputs(Settings, SeaRadius, Key);
    Bake->Plates = Inputs.Plates;
    Bake->PlateSeed = Inputs.Seed;
    if (!Bake->bFromCache)
        Bake->BakeSeconds = FPlatformTime::Seconds() - StartSeconds;
    Result = Bake;
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Bake %08x ready (%s, %.2f s): 6 x %d^2 cells, heights %.0f .. %.0f m, crust threshold %.4f."),
        Bake->Key, Bake->bFromCache ? TEXT("from the cache") : TEXT("baked"), FPlatformTime::Seconds() - StartSeconds,
        Bake->N, Bake->MinHeight, Bake->MaxHeight, Bake->CrustThreshold);

    if (!Bake->bFromCache && CVarVoxelLandCache.GetValueOnGameThread() != 0)
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

FVoxelLandShaderParameters VoxelLandGPU::GetParameters(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density)
{
    FVoxelLandShaderParameters Parameters;
    Parameters.LandMode = 0;
    Parameters.LandN = 1;
    Parameters.LandPad = 0;
    Parameters.LandWarpOctaves = 0;
    Parameters.LandWarpInvNorm = 0.0f;
    Parameters.LandWarpAmplitude = 0.0f;
    Parameters.LandDetailAmplitude = 0.0f;
    Parameters.LandHeightOffset = FVector2f::ZeroVector;

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
        Parameters.LandHeight = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Height));
        Parameters.LandTanTable = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.TanTable));
        Parameters.LandMode = 1;
        Parameters.LandN = (uint32)Bake->N;
        Parameters.LandPad = (uint32)FVoxelLandBake::Pad;
        Parameters.LandWarpOctaves = Density.LandWarpOctaves;
        Parameters.LandWarpInvNorm = Density.LandWarpInvNorm;
        Parameters.LandWarpAmplitude = Density.LandWarpAmplitude;
        Parameters.LandDetailAmplitude = Density.LandDetailAmplitude;
        Parameters.LandHeightOffset = Density.LandHeightOffset;
        return Parameters;
    }

    // The noise surface: one-element stand-ins (the shader's land branch never runs).
    Parameters.LandHeight = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandHeightNone"), TConstArrayView<float>()));
    Parameters.LandTanTable = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<FVector2f>(GraphBuilder, TEXT("Voxel.LandTanTableNone"), TConstArrayView<FVector2f>()));
    return Parameters;
}

FVoxelLandDebugParameters VoxelLandGPU::GetDebugParameters(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density, int32 Mode, double SeaRadius)
{
    FVoxelLandDebugParameters Parameters;
    Parameters.LandDebugMode = (uint32)FMath::Max(Mode, 0);
    Parameters.LandDebugSeaRadius = VoxelDoubleFloat(FMath::Max(SeaRadius, 0.0));
    Parameters.LandPlateCount = 0;
    Parameters.LandSeed = 0;
    // The plate model's other inputs: the debug views read only the plates and the seed.
    Parameters.LandRadiusM = 1.0f;
    Parameters.LandWidthScale = 1.0f;
    Parameters.LandReliefScale = 1.0f;
    Parameters.LandShelfDepth = 0.0f;
    Parameters.LandAbyssDepth = 0.0f;
    Parameters.LandCoastHeight = 0.0f;
    Parameters.LandInteriorHeight = 0.0f;

    const FVoxelLandBake* Bake = Mode > 0 && Density.LandMode != 0 ? Density.Land.Get() : nullptr;
    if (Bake && Bake->GPU.IsValid() && Bake->Attr.Num() == Bake->NumCells() && Bake->Plates.Num() >= 8)
    {
        FVoxelLandBakeGPU& Resource = *Bake->GPU;
        if (!Resource.Attr.IsValid())
        {
            // Two 16-bit cells per word, the lower index in the low half (VoxelLandDebugAttr).
            const int32 NumCells = Bake->Attr.Num();
            TArray<uint32> Packed;
            Packed.SetNumUninitialized((NumCells + 1) / 2);
            for (int32 i = 0; i < Packed.Num(); i++)
            {
                const uint32 Low = Bake->Attr[2 * i];
                const uint32 High = 2 * i + 1 < NumCells ? (uint32)Bake->Attr[2 * i + 1] : 0u;
                Packed[i] = Low | (High << 16);
            }
            Resource.Attr = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandAttr"), MakeConstArrayView(Packed)));
        }
        if (!Resource.Plates.IsValid())
        {
            Resource.Plates = GraphBuilder.ConvertToExternalBuffer(
                VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandDebugPlates"), MakeArrayView(Bake->Plates)));
        }
        Parameters.LandAttr = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Attr));
        Parameters.LandPlates = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Resource.Plates));
        Parameters.LandPlateCount = (uint32)(Bake->Plates.Num() / 8);
        Parameters.LandSeed = Bake->PlateSeed;
        return Parameters;
    }

    // No view, or the noise surface: one-element stand-ins (the bake views show "none").
    Parameters.LandAttr = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.LandAttrNone"), TConstArrayView<uint32>()));
    Parameters.LandPlates = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.LandPlatesNone"), TConstArrayView<float>()));
    return Parameters;
}
