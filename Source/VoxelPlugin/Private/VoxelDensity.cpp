#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "VoxelLandBake.h"
#include "VoxelPlugin.h"
#include "VoxelShaderCompat.h"
#include "Async/ParallelFor.h"

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
#include "VoxelDF64.ush"
#include "VoxelCubeSphere.ush"

    /** The land bake's sampler on the CPU: VoxelLandSample.ush and VoxelLandGully.ush (with
     *  VoxelLandHills.ush, VoxelLandNoise.ush and VoxelLandPeaks.ush) with the shader's buffers
     *  and constants as members (the GPU runs the same code). */
    struct FVoxelLandSamplerCPU
    {
        const float* LandHeight = nullptr;
        const float2* LandTanTable = nullptr;
        const uint* LandFlow = nullptr;
        const uint* LandForm = nullptr;
        const uint* LandRelief = nullptr;
        const uint* LandProvince = nullptr;
        const uint* LandRiverTiles = nullptr;
        const float* LandRiverData = nullptr;
        const uint* LandNoise = nullptr;
        uint LandN = 1;
        uint LandPad = 0;
        uint LandHasFlow = 0;
        uint LandHasForm = 0;
        float LandCellM = 1.0f;
        float LandGullyStrength = 0.0f;
        uint LandGullyOctaves = 0;
        float LandGullyFlowGain = 0.0f;
        float LandGullyDetail = 1.0f;
        uint LandGullySeed = 0;
        float LandHillRelief = 0.0f;
        float2 LandHillFrequency = float2(0.0f, 0.0f);
        uint LandHillOctaves = 0;
        uint LandHillSeed = 0;
        float LandPeakStrength = 0.0f;
        float LandPeakTexelsPerCell = 0.0f;
        uint LandPeakSeed = 0;
        float LandPeakSpacingM = 0.0f;
        float LandPeakReliefM = 0.0f;
        float LandPeakReliefStart = 250.0f;
        float LandPeakReliefEnd = 900.0f;
        float LandPeakMeanR = 0.0f;
        float LandPeakMeanG = 0.0f;
        float LandPeakScaleR = 1.0f;
        float LandPeakScaleG = 1.0f;
        float LandPeakScaleB = 1.0f;
        uint LandNoiseN = 0;
        uint LandNoiseMips = 0;
        uint LandDetailBase = 0;
        uint LandDetailN = 0;
        uint LandDetailMips = 0;
        float LandPeakDetailTexelsPerCell = 0.0f;
        float LandDetailMeanHard = 0.0f;
        float LandDetailMeanSoft = 0.0f;
        float LandPeakDetailScaleHard = 0.0f;
        float LandPeakDetailScaleSoft = 0.0f;
        float LandPeakDetailFade = 0.0f;
        float LandHillTexelsPerCell = 0.0f;
        uint LandHillBase = 0;
        uint LandHillN = 0;
        uint LandHillMips = 0;
        float LandHillScaleR = 1.0f;
        float LandHillScaleG = 1.0f;
        float LandHillScaleB = 1.0f;
        float LandHillMeanR = 0.0f;
        float LandHillMeanG = 0.0f;
        float LandHillSpacingM = 0.0f;
        float LandHillSwellTexelsPerCell = 0.0f;
        float LandHillSwellM = 0.0f;
        float LandHillSwellScaleG = 1.0f;
        float LandHillDetailTexelsPerCell = 0.0f;
        float LandHillDetailScaleHard = 0.0f;
        float LandHillDetailScaleSoft = 0.0f;
        float LandHillDetailFade = 0.0f;
        float LandHillFoothill = 0.0f;
        float LandStrataTexelsPerCell = 0.0f;
        uint LandStrataBase = 0;
        uint LandStrataN = 0;
        uint LandStrataMips = 0;
        float LandStrataScaleR = 1.0f;
        float LandStrataScaleG = 1.0f;
        float LandStrataScaleB = 1.0f;
        float LandStrataMeanR = 0.0f;
        float LandStrataMeanG = 0.0f;
        float LandStrataSpacingM = 0.0f;
        float LandStrataRelief = 0.0f;
        float LandStrataFlat = 0.0f;
        float LandStrataThreshold = 2.0f;
        float LandStrataWidth = 0.05f;
        uint LandStrataSeed = 0;
        float LandStrataDetailTexelsPerCell = 0.0f;
        float LandStrataDetailScaleHard = 0.0f;
        float LandStrataDetailScaleSoft = 0.0f;
        float LandStrataDetailFade = 0.0f;
        uint LandRiverTilesPerFace = 0;
        float LandRiverRadiusM = 1.0f;
        float LandRiverWidthScale = 1.0f;
        float LandRiverValleyScale = 1.0f;
        const float* LandWater = nullptr;
        uint LandHasWater = 0;
        float LandRiverCarve = 0.0f;
        float LandRiverDepthScale = 1.0f;
        float LandRiverFloodplainScale = 1.0f;
        float LandRiverMinLog2Area = 0.0f;
        float LandLakeShoreBand = 0.0f;
        float LandRiverMeander = 0.0f;
        float LandRiverFalls = 0.0f;
        float LandDetailTexelsPerCell = 0.0f;
        uint LandDetailScales = 0;
        float LandDetailMeanA = 0.0f;
        float LandDetailScaleA = 0.0f;
        float LandDetailScaleGain = 0.0f;
        float LandDetailFade = 0.0f;
        uint LandDetailSeed = 0;

#include "VoxelLandSample.ush"
#include "VoxelLandGully.ush"
    };
}

namespace
{
    constexpr uint32 SaltContinent = 1u;
    constexpr uint32 SaltMountain = 2u;
    constexpr uint32 SaltDetail = 3u;
    constexpr uint32 SaltLandWarp = 0x11u;   // + component
    constexpr uint32 SaltLandGully = 0x21u;
    constexpr uint32 SaltLandHill = 0x31u;
    constexpr uint32 SaltLandPeak = 0x41u;
    constexpr uint32 SaltLandStrata = 0x51u;
    constexpr uint32 SaltLandDetail = 0x61u;

    /** The sampling warp of the land bake: three octaves of fBm per component, its longest
     *  wavelength six bake cells. */
    constexpr int32 LandWarpOctaves = 3;
    constexpr double LandWarpWavelengthCells = 6.0;

    VoxelShader::float2 ToDoubleFloat(double V)
    {
        const float Hi = (float)V;
        return VoxelShader::float2(Hi, (float)(V - (double)Hi));
    }

    /** Standard deviation of one octave of VoxelGradientNoise (measured over 400k points). */
    constexpr double SingleOctaveSigma = 0.1914;

    /** A coordinate in 32.32 fixed point (VoxelShader::FVoxelFixed). |V| below 2^31. */
    VoxelShader::FVoxelFixed ToFixed(double V)
    {
        const double Floor = FMath::FloorToDouble(V);
        VoxelShader::FVoxelFixed Out;
        Out.Cell = (uint32)(int64)Floor;
        Out.Frac = (uint32)FMath::Min((V - Floor) * 4294967296.0, 4294967295.0);
        return Out;
    }

    /** Splits a lattice coordinate into a cell (wrapped to 32 bits) and the fraction in it. */
    void SplitLattice(const FVector& L, VoxelShader::uint3& OutCell, VoxelShader::float3& OutFrac)
    {
        const double FX = FMath::FloorToDouble(L.X);
        const double FY = FMath::FloorToDouble(L.Y);
        const double FZ = FMath::FloorToDouble(L.Z);
        OutCell = VoxelShader::uint3((uint32)(int64)FX, (uint32)(int64)FY, (uint32)(int64)FZ);
        OutFrac = VoxelShader::float3((float)(L.X - FX), (float)(L.Y - FY), (float)(L.Z - FZ));
    }
}

uint32 FVoxelSurfaceSettings::GetSettingsHash() const
{
    uint32 H = ::GetTypeHash(Seed);
    for (const double V : { Lacunarity,
        ContinentWavelength, ContinentGain, ContinentWeight,
        MountainWavelength, MountainGain, MountainWeight, MountainStart, MountainBlend, RidgeGain,
        DetailWavelength, DetailGain, DetailWeight, DetailSlopeDamping })
    {
        H = HashCombine(H, ::GetTypeHash(V));
    }
    for (const int32 V : { ContinentOctaves, MountainOctaves, DetailOctaves })
    {
        H = HashCombine(H, ::GetTypeHash(V));
    }
    return H;
}

FVoxelDensityCPU::FVoxelDensityCPU(const FVoxelSurfaceSettings& InSettings, double InPlanetRadius, double InNoiseAmplitude, double InMinFootprint)
    : Settings(InSettings)
    , PlanetRadius(FMath::Max(InPlanetRadius, 1.0))
    , NoiseAmplitude(FMath::Max(InNoiseAmplitude, 0.0))
    , MinFootprint(FMath::Max(InMinFootprint, 0.0))
{
    const uint32 Seed = (uint32)Settings.Seed;
    Continent = MakeLayer(VoxelShader::VoxelLayerPlain, Settings.ContinentWavelength, PlanetRadius, Settings.Lacunarity, Settings.ContinentGain, Settings.ContinentOctaves, VoxelShader::VoxelHashSeed(Seed, SaltContinent));
    Mountain = MakeLayer(VoxelShader::VoxelLayerRidged, Settings.MountainWavelength, PlanetRadius, Settings.Lacunarity, Settings.MountainGain, Settings.MountainOctaves, VoxelShader::VoxelHashSeed(Seed, SaltMountain));
    Detail = MakeLayer(VoxelShader::VoxelLayerDamped, Settings.DetailWavelength, PlanetRadius, Settings.Lacunarity, Settings.DetailGain, Settings.DetailOctaves, VoxelShader::VoxelHashSeed(Seed, SaltDetail));
}

FVoxelDensityCPU::FVoxelDensityCPU(const FVoxelTerrestrialSettings& InLand, TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> InBake,
    double InPlanetRadius, double InNoiseAmplitude, double InMinFootprint)
    : PlanetRadius(FMath::Max(InPlanetRadius, 1.0))
    , NoiseAmplitude(FMath::Max(InNoiseAmplitude, 0.0))
    , MinFootprint(FMath::Max(InMinFootprint, 0.0))
    , LandBake(MoveTemp(InBake))
{
    check(LandBake.IsValid());
    const uint32 Seed = (uint32)InLand.Seed;
    // The per-sample layers (warp, detail) are sampled on the sea-level sphere, not on
    // PlanetRadius: that is the bottom of the height range, which moves with the layers'
    // amplitudes, and would move every pattern with it.
    LandSampleRadius = FMath::Max(LandBake->SeaRadius, 1.0);
    const double SampleRadius = LandSampleRadius;

    // The noise surface's layers are empty (N1: the detail layer is the detail set's,
    // VoxelLandDetail.ush, below); the octave table holds only the warp's.
    Settings = FVoxelSurfaceSettings();
    Settings.Seed = InLand.Seed;
    Settings.Lacunarity = 2.0;
    Settings.ContinentOctaves = 0;
    Settings.MountainOctaves = 0;
    Settings.DetailOctaves = 0;
    Continent = MakeLayer(VoxelShader::VoxelLayerPlain, 1.0, SampleRadius, 2.0, 0.5, 0, VoxelShader::VoxelHashSeed(Seed, SaltContinent));
    Mountain = MakeLayer(VoxelShader::VoxelLayerRidged, 1.0, SampleRadius, 2.0, 0.5, 0, VoxelShader::VoxelHashSeed(Seed, SaltMountain));
    Detail = MakeLayer(VoxelShader::VoxelLayerDamped, 1.0, SampleRadius, 2.0, 0.5, 0, VoxelShader::VoxelHashSeed(Seed, SaltDetail));

    const double Cell = VoxelLand::CellArc(LandBake->SeaRadius, LandBake->N);
    const double WarpFraction = LandWarpWavelengthCells * Cell / SampleRadius;
    for (int32 Axis = 0; Axis < 3; Axis++)
        LandWarp[Axis] = MakeLayer(VoxelShader::VoxelLayerPlain, WarpFraction, SampleRadius, 2.0, 0.5, LandWarpOctaves, VoxelShader::VoxelHashSeed(Seed, SaltLandWarp + (uint32)Axis));
    LandWarpAmplitude = FMath::Max(InLand.BakeWarp, 0.0) * Cell;
    LandDetailAmplitude = FMath::Max(InLand.DetailAmplitude, 0.0) * 100.0;
    LandHeightOffset = LandBake->SeaRadius - PlanetRadius;

    // The gullies (VoxelLandGully.ush): octaves from two bake cells down to the finest wavelength.
    LandCellM = (float)(Cell * 0.01);
    LandGullyStrength = (float)FMath::Max(InLand.GullyStrength, 0.0);
    LandGullyOctaves = (uint32)VoxelLand::GullyOctavesFor(InLand, *LandBake);
    LandGullyFlowGain = (float)FMath::Clamp(InLand.GullyFlowGain, 0.0, 1.0);
    LandGullyDetail = (float)FMath::Clamp(InLand.GullyDetailScale, 0.0, 1.0);
    LandGullySeed = VoxelShader::VoxelHashSeed(Seed, SaltLandGully);
    if (LandBake->Flow.Num() == LandBake->NumCells() && LandBake->NumCells() % 2 == 0)
    {
        // Two cells per word, the lower index in the low half: the GPU's packing (and the
        // little-endian layout of the f16 array).
        LandFlowPacked.SetNumUninitialized(LandBake->NumCells() / 2);
        for (int32 i = 0; i < LandFlowPacked.Num(); i++)
            LandFlowPacked[i] = (uint32)LandBake->Flow[2 * i].Encoded | ((uint32)LandBake->Flow[2 * i + 1].Encoded << 16);
    }

    // The hills (VoxelLandHills.ush), on the landform field.
    LandHillRelief = (float)FMath::Max(InLand.HillRelief, 0.0);
    LandHillOctaves = (uint32)VoxelLand::HillOctavesFor(InLand);
    LandHillFrequency = VoxelLand::HillFinestFrequency(InLand);
    LandHillSeed = VoxelShader::VoxelHashSeed(Seed, SaltLandHill);
    if (LandBake->Form.Num() == LandBake->NumCells() && LandBake->NumCells() % 2 == 0)
    {
        LandFormPacked.SetNumUninitialized(LandBake->NumCells() / 2);
        for (int32 i = 0; i < LandFormPacked.Num(); i++)
            LandFormPacked[i] = (uint32)LandBake->Form[2 * i] | ((uint32)LandBake->Form[2 * i + 1] << 16);
    }

    // The peaks (VoxelLandPeaks.ush): one lookup of the shared noise set, placed at the massif
    // spacing, on the local relief.
    LandPeakStrength = (float)FMath::Max(InLand.PeakHeightScale, 0.0);
    LandPeakSeed = VoxelShader::VoxelHashSeed(Seed, SaltLandPeak);
    // The peaks' fade-in over the local relief (T4b), ordered with a metre between.
    LandPeakReliefStart = (float)FMath::Max(InLand.PeakReliefFadeStart, 0.0);
    LandPeakReliefEnd = (float)FMath::Max(InLand.PeakReliefFadeEnd, (double)LandPeakReliefStart + 1.0);
    if (InLand.PeakReliefFadeEnd < InLand.PeakReliefFadeStart + 1.0)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelDensity] PeakReliefFadeEnd (%.0f m) is not above PeakReliefFadeStart (%.0f m): the peaks will switch on in a 1 m step."),
            InLand.PeakReliefFadeEnd, InLand.PeakReliefFadeStart);
    }
    if (LandBake->Relief.Num() == LandBake->NumCells())
    {
        LandReliefPacked.SetNumZeroed((LandBake->Relief.Num() + 3) / 4);
        for (int32 i = 0; i < LandBake->Relief.Num(); i++)
            LandReliefPacked[i >> 2] |= (uint32)LandBake->Relief[i] << ((i & 3) * 8);
        // The province (T4c), packed as the relief: zeros if the bake has none.
        LandProvincePacked.SetNumZeroed(LandReliefPacked.Num());
        if (LandBake->Province.Num() == LandBake->NumCells())
            for (int32 i = 0; i < LandBake->Province.Num(); i++)
                LandProvincePacked[i >> 2] |= (uint32)FMath::Min(LandBake->Province[i], (uint8)63) << ((i & 3) * 8);
    }
    // The noise pack: the peaks' set, the detail set, the hills' set (T4a), the strata's (T4c).
    // The layers on the landform need the landform; the detail layer (N1) doesn't.
    const bool bHasForm = LandFormPacked.Num() > 0 && LandReliefPacked.Num() > 0;
    LandNoise = VoxelLand::GetLandNoise(VoxelLand::LandNoiseNameFor(InLand));
    if (LandNoise.IsValid() && bHasForm && LandPeakStrength > 0.0f && LandNoise->N > 0)
        LandPeakTexelsPerCell = VoxelLand::PeakTexelsPerCell(InLand, *LandBake, *LandNoise);
    if (LandNoise.IsValid() && LandNoise->DetailN > 0)
    {
        // The detail set's means, whoever reads it (the peaks, the hills' and the strata's sets).
        LandDetailMeanHard = LandNoise->DetailMeanHard;
        LandDetailMeanSoft = LandNoise->DetailMeanSoft;
    }
    // N1: the detail layer (VoxelLandDetail.ush): scales of the set's A channel, four octaves
    // each, the first's coarsest at DetailWavelength, from there down to the finest cell (at most
    // DetailMaxScales), and no finer than the lattice's range holds (VoxelLandPeakRotated: the
    // turned texel coordinates, at most sqrt 2 times the face's half-width and blend band in
    // texels, under 2^30).
    LandDetailSeed = VoxelShader::VoxelHashSeed(Seed, SaltLandDetail);
    if (LandNoise.IsValid() && LandNoise->DetailN > 0 && LandNoise->DetailSpacingTexels > 0 && LandDetailAmplitude > 0.0 && LandNoise->DetailStdA > 0.0f)
    {
        const FVoxelLandNoiseSet& Set = *LandNoise;
        const double WavelengthM = VoxelLand::DetailLayerWavelength(InLand);
        const double TexelM = WavelengthM / (double)Set.DetailSpacingTexels;
        LandDetailTexelsPerCell = (float)((double)LandCellM / TexelM);
        // Octaves from the wavelength to the finest cell, four a scale.
        const double Finest = FMath::Max(MinFootprint, 1.0) * 0.01;
        const int32 Octaves = FMath::Max(FMath::CeilToInt(FMath::Log2(WavelengthM / Finest)) + 1, 1);
        int32 Scales = FMath::Clamp((Octaves + 3) / 4, 1, VoxelLand::DetailMaxScales);
        const double FaceTexels = UE_SQRT_2 * (0.5 * (double)LandBake->N + 4.0);
        while (Scales > 1 && (double)LandDetailTexelsPerCell * FMath::Pow(16.0, (double)(Scales - 1)) * FaceTexels >= 1073741824.0)
            Scales--;
        LandDetailScales = (uint32)Scales;
        LandDetailMeanA = Set.DetailMeanA;
        // A's unit over its standard deviation times 0.4 (the old fBm's normalisation).
        LandDetailScaleA = 0.4f / Set.DetailStdA;
        // One scale to the next: A's gain to the fourth (its octaves' spectrum continued).
        LandDetailScaleGain = Set.DetailScaleGain;
        LandDetailFade = (float)FMath::FloorLog2(Set.DetailSpacingTexels) - 1.0f;
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Detail %s: %d scales from %.0f m (finest octave %.2f m), %.0f m amplitude."),
            *Set.DetailName, Scales, WavelengthM, WavelengthM / FMath::Pow(16.0, (double)(Scales - 1)) / 8.0, LandDetailAmplitude * 0.01);
    }
    if (LandPeakTexelsPerCell > 0.0f)
    {
        const FVoxelLandNoiseSet& Set = *LandNoise;
        const float TexelM = LandCellM / LandPeakTexelsPerCell;
        LandPeakSpacingM = (float)FMath::Max(InLand.PeakSpacing, VoxelLand::PeakMinSpacing);
        // The shape over the larger of hard and soft rock's ranges: within -1 .. 1.
        const float Range = FMath::Max3(Set.RangeR, Set.RangeG, 1e-6f);
        LandPeakReliefM = Range * TexelM;
        LandPeakMeanR = Set.MeanR;
        LandPeakMeanG = Set.MeanG;
        LandPeakScaleR = Set.RangeR / Range;
        LandPeakScaleG = Set.RangeG / Range;
        LandPeakScaleB = Set.RangeB / Range;
        // The detail set (T3e, N1): its own texel; its heights (texels) over the peaks' relief, so
        // strength 1 keeps its slopes; faded out by the mip level where its coarsest octave
        // (DetailSpacingTexels) has two texels.
        LandPeakDetailTexelsPerCell = VoxelLand::PeakDetailTexelsPerCell(InLand, *LandBake, Set);
        if (LandPeakDetailTexelsPerCell > 0.0f)
        {
            const float DetailTexelM = LandCellM / LandPeakDetailTexelsPerCell;
            const float Strength = (float)InLand.PeakDetailStrength;
            LandPeakDetailScaleHard = Strength * Set.DetailRangeHard * DetailTexelM / LandPeakReliefM;
            LandPeakDetailScaleSoft = Strength * Set.DetailRangeSoft * DetailTexelM / LandPeakReliefM;
            LandPeakDetailFade = (float)FMath::FloorLog2(Set.DetailSpacingTexels) - 1.0f;
        }
    }
    else
    {
        LandPeakStrength = 0.0f;
        LandPeakTexelsPerCell = 0.0f;
    }

    // The hills from the pack's set (VoxelLandHillSet.ush, T4a): placed at the hill spacing, from
    // its base network up; LandHillRelief becomes the set's relief (m) times the height scale,
    // capped (HillSetReliefCap). Else the procedural hills above.
    if (LandNoise.IsValid() && bHasForm && LandHillRelief > 0.0f && LandNoise->HillN > 0)
        LandHillTexelsPerCell = VoxelLand::HillTexelsPerCell(InLand, *LandBake, *LandNoise);
    if (LandHillTexelsPerCell > 0.0f)
    {
        const FVoxelLandNoiseSet& Set = *LandNoise;
        const float TexelM = LandCellM / LandHillTexelsPerCell;
        const float Range = FMath::Max3(Set.HillRangeR, Set.HillRangeG, 1e-6f);
        const float ReliefM = Range * TexelM;
        const float Wanted = (float)InLand.HillHeightScale * ReliefM;
        const float Cap = (float)VoxelLand::HillSetReliefCap(InLand);
        if (Wanted > Cap)
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The hill set %s's relief (%.0f m at HillHeightScale %.2f) is over half HillSpacing; capped at %.0f m."),
                *Set.HillName, Wanted, InLand.HillHeightScale, Cap);
        LandHillRelief = FMath::Min(Wanted, Cap);
        LandHillSpacingM = (float)FMath::Max(InLand.HillSpacing, VoxelLand::HillMinSpacing);
        LandHillFoothill = (float)FMath::Clamp(InLand.HillFoothillGrowth, 0.0, 2.0);
        LandHillScaleR = Set.HillRangeR / Range;
        LandHillScaleG = Set.HillRangeG / Range;
        LandHillScaleB = Set.HillRangeB / Range;
        LandHillMeanR = Set.HillMeanR;
        LandHillMeanG = Set.HillMeanG;
        if (InLand.HillSwellHeight > 0.0)
        {
            LandHillSwellTexelsPerCell = LandHillTexelsPerCell / (float)FMath::Clamp(InLand.HillSwellScale, 1.5, 8.0);
            LandHillSwellM = (float)InLand.HillSwellHeight;
            // G's whole range is the swells' unit.
            LandHillSwellScaleG = 1.0f;
        }
        // The detail set, as the peaks': its heights (texels) over the set's relief, so strength 1
        // keeps its slopes where the hills keep theirs.
        if (InLand.HillDetailStrength > 0.0 && Set.DetailN > 0 && Set.DetailSpacingTexels > 0)
        {
            const double DetailTexelM = VoxelLand::HillDetailWavelength(InLand) / (double)Set.DetailSpacingTexels;
            LandHillDetailTexelsPerCell = (float)(LandCellM / DetailTexelM);
            const float DetailTexelMF = LandCellM / LandHillDetailTexelsPerCell;
            const float Strength = (float)InLand.HillDetailStrength;
            LandHillDetailScaleHard = Strength * Set.DetailRangeHard * DetailTexelMF / ReliefM;
            LandHillDetailScaleSoft = Strength * Set.DetailRangeSoft * DetailTexelMF / ReliefM;
            LandHillDetailFade = (float)FMath::FloorLog2(Set.DetailSpacingTexels) - 1.0f;
        }
    }
    // The strata (VoxelLandHillSet.ush, T4c): the pack's strata set, placed at the strata spacing,
    // where the province is above the threshold for StrataCoverage of the land; in the hill
    // set's layer (none without it).
    LandStrataSeed = VoxelShader::VoxelHashSeed(Seed, SaltLandStrata);
    if (LandHillTexelsPerCell > 0.0f && LandNoise->StrataN > 0)
        LandStrataTexelsPerCell = VoxelLand::StrataTexelsPerCell(InLand, *LandBake, *LandNoise);
    if (LandStrataTexelsPerCell > 0.0f)
    {
        const FVoxelLandNoiseSet& Set = *LandNoise;
        const float TexelM = LandCellM / LandStrataTexelsPerCell;
        const float Range = FMath::Max3(Set.StrataRangeR, Set.StrataRangeG, 1e-6f);
        const float ReliefM = Range * TexelM;
        const float Wanted = (float)InLand.StrataHeightScale * ReliefM;
        const float Cap = (float)VoxelLand::StrataSetReliefCap(InLand);
        if (Wanted > Cap)
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The strata set %s's relief (%.0f m at StrataHeightScale %.2f) is over half StrataSpacing; capped at %.0f m."),
                *Set.StrataName, Wanted, InLand.StrataHeightScale, Cap);
        LandStrataRelief = FMath::Min(Wanted, Cap);
        LandStrataSpacingM = (float)FMath::Max(InLand.StrataSpacing, VoxelLand::HillMinSpacing);
        LandStrataScaleR = Set.StrataRangeR / Range;
        LandStrataScaleG = Set.StrataRangeG / Range;
        LandStrataScaleB = Set.StrataRangeB / Range;
        LandStrataMeanR = Set.StrataMeanR;
        LandStrataMeanG = Set.StrataMeanG;
        LandStrataFlat = (float)FMath::Clamp(InLand.StrataFlatShare, 0.0, 1.0);
        LandStrataWidth = (float)FMath::Clamp(InLand.StrataSwitchWidth, 0.005, 0.5);
        // At least the switch's half-width above 0: the shields and belt cores (province 0)
        // keep the hills whatever the coverage.
        LandStrataThreshold = FMath::Max(LandBake->ProvinceThreshold(InLand.StrataCoverage), LandStrataWidth);
        if (InLand.StrataDetailStrength > 0.0 && Set.DetailN > 0 && Set.DetailSpacingTexels > 0)
        {
            const double DetailTexelM = VoxelLand::StrataDetailWavelength(InLand) / (double)Set.DetailSpacingTexels;
            LandStrataDetailTexelsPerCell = (float)(LandCellM / DetailTexelM);
            const float DetailTexelMF = LandCellM / LandStrataDetailTexelsPerCell;
            const float Strength = (float)InLand.StrataDetailStrength;
            LandStrataDetailScaleHard = Strength * Set.DetailRangeHard * DetailTexelMF / ReliefM;
            LandStrataDetailScaleSoft = Strength * Set.DetailRangeSoft * DetailTexelMF / ReliefM;
            LandStrataDetailFade = (float)FMath::FloorLog2(Set.DetailSpacingTexels) - 1.0f;
        }
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Strata %s: %.0f m relief at %.0f m spacing over %.0f%% of the land (province above %.3f)."),
            *Set.StrataName, LandStrataRelief, LandStrataSpacingM, 100.0 * FMath::Clamp(InLand.StrataCoverage, 0.0, 1.0), LandStrataThreshold);
    }
    if (LandPeakTexelsPerCell <= 0.0f && LandHillTexelsPerCell <= 0.0f && LandDetailTexelsPerCell <= 0.0f)
        LandNoise.Reset();

    // The rivers (VoxelLandRivers.ush, R1): made on the GPU with the bake, or by VoxelLand::TickRivers
    // before the planet builds (null until then).
    LandRivers = VoxelLand::GetRivers(*LandBake, InLand);
    if (LandRivers.IsValid() && LandRivers->TilesPerFace > 0)
    {
        LandRiverTilesPerFace = (uint32)LandRivers->TilesPerFace;
        LandRiverRadiusM = (float)(LandBake->SeaRadius * 0.01);
        LandRiverWidthScale = (float)FMath::Max(InLand.RiverWidthScale, 0.1);
        LandRiverValleyScale = (float)FMath::Max(InLand.ValleyWidthScale, 0.1);
    }
    else
        LandRivers.Reset();

    // R2: the rivers' and lakes' carve (VoxelLandCarve.ush). The lakes need the bake's levels.
    LandRiverCarve = InLand.bCarveRivers ? 1.0f : 0.0f;
    LandRiverDepthScale = (float)FMath::Max(InLand.RiverDepthScale, 0.0);
    LandRiverFloodplainScale = (float)FMath::Max(InLand.FloodplainScale, 0.0);
    LandRiverMinLog2Area = InLand.RiverMinArea > 0.0 ? (float)FMath::Log2(InLand.RiverMinArea * 1.0e6) : 0.0f;
    LandLakeShoreBand = InLand.bCarveRivers ? (float)FMath::Max(InLand.LakeShoreBand, 0.0) : 0.0f;
    // R3: the meanders and the knickpoints' falls.
    LandRiverMeander = (float)FMath::Max(InLand.RiverMeander, 0.0);
    LandRiverFalls = (float)FMath::Clamp(InLand.RiverFalls, 0.0, 1.4);
    LandHasWater = InLand.bCarveRivers && LandBake->Water.Num() == LandBake->NumCells() ? 1u : 0u;
}

FVoxelDensityCPU::FLayer FVoxelDensityCPU::MakeLayer(uint32 Kind, double WavelengthFraction, double InPlanetRadius, double InLacunarity, double InGain, int32 InOctaves, uint32 InSeed)
{
    FLayer Layer;
    Layer.Kind = Kind;
    const double Lacunarity = FMath::Max(InLacunarity, 1.01);
    const int32 NumOctaves = FMath::Max(InOctaves, 0);

    double Frequency = 1.0 / FMath::Max(WavelengthFraction * InPlanetRadius, 1e-3);
    // Lacunarity 2 keeps every octave's frequency the finest one's over a power of two. The
    // finest coordinate (up to the radius times its frequency) must fit the 32-bit cell.
    const bool bFixed = Lacunarity == 2.0 && NumOctaves > 0 && NumOctaves <= 32
        && InPlanetRadius * Frequency * FMath::Pow(2.0, (double)(NumOctaves - 1)) < 1073741824.0;
    float Amplitude = 1.0f;
    double AmpSum = 0.0;
    double AmpSquareSum = 0.0;
    double Amp = 1.0;
    for (int32 o = 0; o < NumOctaves; o++)
    {
        AmpSum += Amp;
        AmpSquareSum += Amp * Amp;
        Amp *= InGain;

        FOctave& Octave = Layer.Octaves.AddDefaulted_GetRef();
        Octave.Frequency = Frequency;
        Octave.Wavelength = (float)(1.0 / Frequency);
        Octave.Amplitude = Amplitude;
        Octave.Seed = InSeed + (uint32)o * 0x9e3779b9u;
        // A per-octave shift of the lattice, so octaves don't share zero crossings.
        const uint32 H = VoxelShader::VoxelHashSeed(InSeed, 0x51ed27u + (uint32)o);
        Octave.Offset = FVector((H & 0x3ffu) * 0.618034, ((H >> 10) & 0x3ffu) * 0.414214, ((H >> 20) & 0x3ffu) * 0.732051);
        if (bFixed)
        {
            Octave.Shift = (uint32)(NumOctaves - 1 - o);
            for (int32 Axis = 0; Axis < 3; Axis++)
            {
                const VoxelShader::FVoxelFixed F = ToFixed(Octave.Offset[Axis]);
                Octave.OffsetCell[Axis] = F.Cell;
                Octave.OffsetFrac[Axis] = F.Frac;
            }
        }

        Frequency *= Lacunarity;
        Amplitude *= (float)InGain;
    }
    // Ridged sums are each in [0, 1], so dividing by the amplitude sum keeps [0, 1]. fBm sums
    // are scaled to a standard deviation of 0.4, so about [-1, 1] holds 98% of the surface
    // (dividing by the amplitude sum would leave them in about +-0.25).
    if (Kind == VoxelShader::VoxelLayerRidged)
        Layer.InvNorm = AmpSum > 0.0 ? 1.0 / AmpSum : 0.0;
    else
        Layer.InvNorm = AmpSquareSum > 0.0 ? 1.0 / (2.5 * SingleOctaveSigma * FMath::Sqrt(AmpSquareSum)) : 0.0;
    return Layer;
}

double FVoxelDensityCPU::EvaluateLayer(const FLayer& Layer, const FVector& SurfacePoint, double Footprint) const
{
    return EvaluateLayerGradient(Layer, SurfacePoint, Footprint, nullptr);
}

double FVoxelDensityCPU::EvaluateLayerGradient(const FLayer& Layer, const FVector& SurfacePoint, double Footprint, FVector* OutGradient) const
{
    using namespace VoxelShader;

    // The core keeps its running sum in float; each octave's contribution is taken from a
    // zeroed sum and added here in double, so the total keeps double precision.
    FVoxelFbmState State = VoxelFbmBegin();
    double Sum = 0.0;
    FVector Gradient = FVector::ZeroVector;
    if (OutGradient) *OutGradient = FVector::ZeroVector;
    if (Layer.Octaves.Num() == 0) return 0.0;

    // Lacunarity 2: the finest octave's coordinate once, in fixed point, as the GPU takes it.
    const bool bFixed = Layer.Octaves[0].Shift != VoxelNoFixedPoint;
    FVoxelFixed Finest[3] = {};
    if (bFixed)
    {
        const FVector Coord = SurfacePoint * Layer.Octaves.Last().Frequency;
        for (int32 Axis = 0; Axis < 3; Axis++)
            Finest[Axis] = ToFixed(Coord[Axis]);
    }
    for (const FOctave& Octave : Layer.Octaves)
    {
        const float Band = VoxelOctaveBandWeight(Octave.Wavelength, (float)Footprint);
        if (Band <= 0.0f) break;

        uint3 Cell;
        float3 Frac;
        if (bFixed)
        {
            FVoxelFixed L[3];
            for (int32 Axis = 0; Axis < 3; Axis++)
                L[Axis] = VoxelFixedLattice(Finest[Axis], Octave.Shift, Octave.OffsetCell[Axis], Octave.OffsetFrac[Axis]);
            Cell = uint3(L[0].Cell, L[1].Cell, L[2].Cell);
            Frac = float3(VoxelFixedFraction(L[0].Frac), VoxelFixedFraction(L[1].Frac), VoxelFixedFraction(L[2].Frac));
        }
        else
        {
            SplitLattice(SurfacePoint * Octave.Frequency + Octave.Offset, Cell, Frac);
        }
        const FVoxelNoiseSample N = VoxelGradientNoise(Cell, Frac, Octave.Seed);

        State.Sum = 0.0f;
        State = VoxelLayerOctave(State, Layer.Kind, N, Octave.Amplitude * Band, (float)Settings.RidgeGain, (float)Settings.DetailSlopeDamping);
        Sum += State.Sum;
        if (OutGradient)
            Gradient += FVector(N.Gradient.x, N.Gradient.y, N.Gradient.z) * ((double)(Octave.Amplitude * Band) * Octave.Frequency);
    }
    if (OutGradient) *OutGradient = Gradient * Layer.InvNorm;
    return Sum * Layer.InvNorm;
}

double FVoxelDensityCPU::HeightAt(const FVector& Dir, double Footprint) const
{
    const double Len = Dir.Size();
    if (Len <= 1e-12 || NoiseAmplitude <= 0.0) return 0.0;

    // Footprints below the finest cell are raised to it, as Sample does (and the GPU).
    const double Band = FMath::Max(Footprint, MinFootprint);
    // The terrestrial layers on the sea-level sphere (see the constructor).
    if (LandBake.IsValid())
        return LandHeightAt(Dir * (LandSampleRadius / Len), Band);
    const FVector SurfacePoint = Dir * (PlanetRadius / Len);
    const double C = EvaluateLayer(Continent, SurfacePoint, Band);
    const double R = EvaluateLayer(Mountain, SurfacePoint, Band);
    const double D = EvaluateLayer(Detail, SurfacePoint, Band);

    // In double: a float height in [0, 1] would quantise the surface to NoiseAmplitude * 6e-8
    // (several cm on an Earth-sized planet). VoxelSampleDensityDF mirrors this in double-float.
    const double Base = 0.5 + 0.5 * Settings.ContinentWeight * C;
    const double MaskT = FMath::Clamp((C - Settings.MountainStart) / FMath::Max(Settings.MountainBlend, 1e-6), 0.0, 1.0);
    const double Mountains = Settings.MountainWeight * MaskT * MaskT * (3.0 - 2.0 * MaskT) * R;
    const double Details = Settings.DetailWeight * D;
    return NoiseAmplitude * FMath::Clamp(Base + Mountains + Details, 0.0, 1.0);
}

double FVoxelDensityCPU::LandHeightAt(const FVector& SurfacePoint, double Footprint) const
{
    return LandSurfaceAt(SurfacePoint, Footprint, nullptr);
}

double FVoxelDensityCPU::LandSurfaceAt(const FVector& SurfacePoint, double Footprint, FVoxelWaterSample* OutWater) const
{
    using namespace VoxelShader;

    // The bake through the sampling warp (every octave: the warp doesn't change with the
    // footprint, so the bake looks the same at every level), and the warp's Jacobian for the
    // gullies' steering.
    FVector Q = SurfacePoint;
    FVector DW[3] = { FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector };
    if (LandWarpAmplitude > 0.0)
    {
        FVector W;
        for (int32 Axis = 0; Axis < 3; Axis++)
        {
            W[Axis] = EvaluateLayerGradient(LandWarp[Axis], SurfacePoint, 0.0, &DW[Axis]) * LandWarpAmplitude;
            DW[Axis] *= LandWarpAmplitude;
        }
        Q += W;
    }
    FVoxelLandSamplerCPU Sampler;
    Sampler.LandHeight = LandBake->Height.GetData();
    Sampler.LandTanTable = reinterpret_cast<const float2*>(LandBake->TanTable.GetData());
    Sampler.LandFlow = LandFlowPacked.GetData();
    Sampler.LandForm = LandFormPacked.GetData();
    // The landform with the local relief (the GPU's LandHasForm the same).
    Sampler.LandHasForm = LandFormPacked.Num() > 0 && LandReliefPacked.Num() > 0 ? 1u : 0u;
    Sampler.LandHillRelief = LandHillRelief;
    Sampler.LandHillFrequency = ToDoubleFloat(LandHillFrequency);
    Sampler.LandHillOctaves = LandHillOctaves;
    Sampler.LandHillSeed = LandHillSeed;
    Sampler.LandRelief = LandReliefPacked.GetData();
    Sampler.LandProvince = LandProvincePacked.GetData();
    if (LandRivers.IsValid())
    {
        Sampler.LandRiverTiles = LandRivers->Tiles.GetData();
        Sampler.LandRiverData = LandRivers->Data.GetData();
        Sampler.LandRiverTilesPerFace = LandRiverTilesPerFace;
        Sampler.LandRiverRadiusM = LandRiverRadiusM;
        Sampler.LandRiverWidthScale = LandRiverWidthScale;
        Sampler.LandRiverValleyScale = LandRiverValleyScale;
    }
    // R2: the carve (the lakes with the bake's levels).
    Sampler.LandRiverCarve = LandRiverCarve;
    Sampler.LandRiverDepthScale = LandRiverDepthScale;
    Sampler.LandRiverFloodplainScale = LandRiverFloodplainScale;
    Sampler.LandRiverMinLog2Area = LandRiverMinLog2Area;
    Sampler.LandLakeShoreBand = LandLakeShoreBand;
    Sampler.LandRiverMeander = LandRiverMeander;
    Sampler.LandRiverFalls = LandRiverFalls;
    Sampler.LandHasWater = LandHasWater;
    Sampler.LandDetailSeed = LandDetailSeed;
    Sampler.LandWater = LandHasWater != 0u ? LandBake->Water.GetData() : nullptr;
    if (LandNoise.IsValid())
    {
        Sampler.LandNoise = LandNoise->Words.GetData();
        // The detail set's chain and means, whoever reads it.
        Sampler.LandDetailBase = LandNoise->DetailBase;
        Sampler.LandDetailN = LandNoise->DetailN;
        Sampler.LandDetailMips = LandNoise->DetailMips;
        Sampler.LandDetailMeanHard = LandDetailMeanHard;
        Sampler.LandDetailMeanSoft = LandDetailMeanSoft;
        Sampler.LandDetailTexelsPerCell = LandDetailTexelsPerCell;
        Sampler.LandDetailScales = LandDetailScales;
        Sampler.LandDetailMeanA = LandDetailMeanA;
        Sampler.LandDetailScaleA = LandDetailScaleA;
        Sampler.LandDetailScaleGain = LandDetailScaleGain;
        Sampler.LandDetailFade = LandDetailFade;
        Sampler.LandHillTexelsPerCell = LandHillTexelsPerCell;
        Sampler.LandHillBase = LandNoise->HillBase;
        Sampler.LandHillN = LandNoise->HillN;
        Sampler.LandHillMips = LandNoise->HillMips;
        Sampler.LandHillScaleR = LandHillScaleR;
        Sampler.LandHillScaleG = LandHillScaleG;
        Sampler.LandHillScaleB = LandHillScaleB;
        Sampler.LandHillMeanR = LandHillMeanR;
        Sampler.LandHillMeanG = LandHillMeanG;
        Sampler.LandHillSpacingM = LandHillSpacingM;
        Sampler.LandHillSwellTexelsPerCell = LandHillSwellTexelsPerCell;
        Sampler.LandHillSwellM = LandHillSwellM;
        Sampler.LandHillSwellScaleG = LandHillSwellScaleG;
        Sampler.LandHillDetailTexelsPerCell = LandHillDetailTexelsPerCell;
        Sampler.LandHillDetailScaleHard = LandHillDetailScaleHard;
        Sampler.LandHillDetailScaleSoft = LandHillDetailScaleSoft;
        Sampler.LandHillDetailFade = LandHillDetailFade;
        Sampler.LandHillFoothill = LandHillFoothill;
        Sampler.LandStrataTexelsPerCell = LandStrataTexelsPerCell;
        Sampler.LandStrataBase = LandNoise->StrataBase;
        Sampler.LandStrataN = LandNoise->StrataN;
        Sampler.LandStrataMips = LandNoise->StrataMips;
        Sampler.LandStrataScaleR = LandStrataScaleR;
        Sampler.LandStrataScaleG = LandStrataScaleG;
        Sampler.LandStrataScaleB = LandStrataScaleB;
        Sampler.LandStrataMeanR = LandStrataMeanR;
        Sampler.LandStrataMeanG = LandStrataMeanG;
        Sampler.LandStrataSpacingM = LandStrataSpacingM;
        Sampler.LandStrataRelief = LandStrataRelief;
        Sampler.LandStrataFlat = LandStrataFlat;
        Sampler.LandStrataThreshold = LandStrataThreshold;
        Sampler.LandStrataWidth = LandStrataWidth;
        Sampler.LandStrataSeed = LandStrataSeed;
        Sampler.LandStrataDetailTexelsPerCell = LandStrataDetailTexelsPerCell;
        Sampler.LandStrataDetailScaleHard = LandStrataDetailScaleHard;
        Sampler.LandStrataDetailScaleSoft = LandStrataDetailScaleSoft;
        Sampler.LandStrataDetailFade = LandStrataDetailFade;
        Sampler.LandNoiseN = LandNoise->N;
        Sampler.LandNoiseMips = LandNoise->Mips;
        Sampler.LandPeakStrength = LandPeakStrength;
        Sampler.LandPeakTexelsPerCell = LandPeakTexelsPerCell;
        Sampler.LandPeakSeed = LandPeakSeed;
        Sampler.LandPeakSpacingM = LandPeakSpacingM;
        Sampler.LandPeakReliefM = LandPeakReliefM;
        Sampler.LandPeakReliefStart = LandPeakReliefStart;
        Sampler.LandPeakReliefEnd = LandPeakReliefEnd;
        Sampler.LandPeakMeanR = LandPeakMeanR;
        Sampler.LandPeakMeanG = LandPeakMeanG;
        Sampler.LandPeakScaleR = LandPeakScaleR;
        Sampler.LandPeakScaleG = LandPeakScaleG;
        Sampler.LandPeakScaleB = LandPeakScaleB;
        if (LandPeakDetailTexelsPerCell > 0.0f)
        {
            Sampler.LandPeakDetailTexelsPerCell = LandPeakDetailTexelsPerCell;
            Sampler.LandPeakDetailScaleHard = LandPeakDetailScaleHard;
            Sampler.LandPeakDetailScaleSoft = LandPeakDetailScaleSoft;
            Sampler.LandPeakDetailFade = LandPeakDetailFade;
        }
    }
    Sampler.LandN = (uint32)LandBake->N;
    Sampler.LandPad = (uint32)FVoxelLandBake::Pad;
    Sampler.LandHasFlow = LandFlowPacked.Num() > 0 ? 1u : 0u;
    Sampler.LandCellM = LandCellM;
    Sampler.LandGullyStrength = LandGullyStrength;
    Sampler.LandGullyOctaves = LandGullyOctaves;
    Sampler.LandGullyFlowGain = LandGullyFlowGain;
    Sampler.LandGullyDetail = LandGullyDetail;
    Sampler.LandGullySeed = LandGullySeed;
    auto ToFloat3 = [](const FVector& V) { return float3((float)V.X, (float)V.Y, (float)V.Z); };
    const FVoxelLandSamplerCPU::FVoxelLandSurface Land = Sampler.VoxelLandSurfaceDF(
        ToDoubleFloat(SurfacePoint.X), ToDoubleFloat(SurfacePoint.Y), ToDoubleFloat(SurfacePoint.Z),
        ToDoubleFloat(Q.X), ToDoubleFloat(Q.Y), ToDoubleFloat(Q.Z), ToFloat3(DW[0]), ToFloat3(DW[1]), ToFloat3(DW[2]), (float)Footprint, false);

    // Detail (VoxelLandDetail.ush), a quarter as strong under the sea and scaled by
    // GullyDetailScale on land while there are gullies (VoxelDensitySample.ush the same way).
    const float DetailWeight = lerp(0.25f, 1.0f, smoothstep(-50.0f, 50.0f, Land.Bake));
    const double D = (double)Sampler.VoxelLandDetailDF(ToDoubleFloat(SurfacePoint.X), ToDoubleFloat(SurfacePoint.Y), ToDoubleFloat(SurfacePoint.Z),
        (float)Footprint, false).Unit;
    const double Height = (double)Land.Bake * 100.0 + LandHeightOffset + D * (double)((float)LandDetailAmplitude * DetailWeight * Land.DetailScale)
        + (double)Land.Hills * 100.0 + (double)Land.Peaks * 100.0 + (double)Land.Gully * 100.0 + (double)Land.Carve * 100.0;
    const double Clamped = FMath::Clamp(Height, 0.0, NoiseAmplitude);
    if (OutWater)
    {
        // R2 (Design/RiversLakes.md 3.6): a kept lake over the ground, else a river's water line
        // in its channel, else the sea below sea level.
        FVoxelWaterSample& W = *OutWater;
        W = FVoxelWaterSample();
        const float Ground = (float)((Clamped - LandHeightOffset) * 0.01);
        W.GroundHeight = Ground;
        W.SurfaceHeight = Ground;
        if (Land.Lake.Weight >= 0.5f && Land.Lake.Level > Ground)
        {
            W.Kind = EVoxelWaterKind::Lake;
            W.SurfaceHeight = Land.Lake.Level;
        }
        else if (Land.River.WaterKind == VOXEL_LAND_WATER_RIVER && Land.River.WaterLevel > Ground)
        {
            W.Kind = EVoxelWaterKind::River;
            W.SurfaceHeight = Land.River.WaterLevel;
            // Manning's mean speed, R^(2/3) S^(1/2) / n: n 0.035 (a natural channel), more on steep
            // reaches by Jarrett's 0.39 S^0.38 R^-0.16 (boulders and steps); the hydraulic radius
            // about the water's depth in a wide channel. Within 0.05 .. 5 m/s.
            const double Radius = FMath::Max((double)VOXEL_LAND_RIVER_WATER_SHARE * Land.River.ChannelDepth, 0.1);
            const double Slope = FMath::Max((double)Land.River.Grade, 1.0e-4);
            const double Roughness = FMath::Max(0.035, 0.39 * FMath::Pow(Slope, 0.38) * FMath::Pow(Radius, -0.16));
            const double Speed = FMath::Clamp(FMath::Pow(Radius, 2.0 / 3.0) * FMath::Sqrt(Slope) / Roughness, 0.05, 5.0);
            W.FlowVelocity = FVector(Land.River.FlowDirection.x, Land.River.FlowDirection.y, Land.River.FlowDirection.z) * (Speed * 100.0);
        }
        else if (Ground < 0.0f)
        {
            W.Kind = EVoxelWaterKind::Sea;
            W.SurfaceHeight = 0.0f;
        }
        W.Depth = FMath::Max(W.SurfaceHeight - W.GroundHeight, 0.0f);
    }
    return Clamped;
}

FVoxelWaterSample FVoxelDensityCPU::WaterAt(const FVector& Direction) const
{
    FVoxelWaterSample Out;
    const double Len = Direction.Size();
    if (Len < 1.0e-6) return Out;
    if (!LandBake.IsValid())
    {
        // The noise surface: the ground above the planet radius (the planet knows its sea).
        Out.GroundHeight = Out.SurfaceHeight = (float)(HeightAt(Direction, MinFootprint) * 0.01);
        return Out;
    }
    LandSurfaceAt(Direction * (LandSampleRadius / Len), MinFootprint, &Out);
    return Out;
}

float FVoxelDensityCPU::SampleOne(const FVector& Position, double Footprint) const
{
    using namespace VoxelShader;

    const double Radius = Position.Size();
    const double Surface = Radius - (PlanetRadius + HeightAt(Position, Footprint));
    return VoxelComposeTerrain((float)Surface, VoxelCaveDensity());
}

void FVoxelDensityCPU::Sample(const FSampleInput& Input, float* DensityOut) const
{
    const int32 Count = Input.Num();
    if (Count <= 0) return;
    const double Footprint = FMath::Max(Input.Footprint, MinFootprint);

    constexpr int32 BatchSize = 64;
    if (Count > 2 * BatchSize)
    {
        const int32 NumBatches = (Count + BatchSize - 1) / BatchSize;
        ParallelFor(NumBatches, [&](int32 Batch) {
            const int32 End = FMath::Min(Count, (Batch + 1) * BatchSize);
            for (int32 i = Batch * BatchSize; i < End; i++)
                DensityOut[i] = SampleOne(Input.GetPosition(i), Footprint);
            });
    }
    else
    {
        for (int32 i = 0; i < Count; i++)
            DensityOut[i] = SampleOne(Input.GetPosition(i), Footprint);
    }
}

void FVoxelDensityCPU::BuildGPUData(FVoxelDensityGPUData& Out) const
{
    Out = FVoxelDensityGPUData();
    for (const FLayer* Layer : { &Continent, &Mountain, &Detail, &LandWarp[0], &LandWarp[1], &LandWarp[2] })
    {
        for (const FOctave& Octave : Layer->Octaves)
        {
            FVoxelDensityGPUOctave& G = Out.Octaves.AddDefaulted_GetRef();
            G.Frequency = VoxelDoubleFloat(Octave.Frequency);
            G.OffsetX = VoxelDoubleFloat(Octave.Offset.X);
            G.OffsetY = VoxelDoubleFloat(Octave.Offset.Y);
            G.OffsetZ = VoxelDoubleFloat(Octave.Offset.Z);
            G.Wavelength = Octave.Wavelength;
            G.Amplitude = Octave.Amplitude;
            G.Seed = Octave.Seed;
            G.Shift = Octave.Shift;
            for (int32 Axis = 0; Axis < 3; Axis++)
            {
                G.OffsetCell[Axis] = Octave.OffsetCell[Axis];
                G.OffsetFrac[Axis] = Octave.OffsetFrac[Axis];
            }
        }
    }
    Out.ContinentOctaves = (uint32)Continent.Octaves.Num();
    Out.MountainOctaves = (uint32)Mountain.Octaves.Num();
    Out.DetailOctaves = (uint32)Detail.Octaves.Num();
    Out.InvNormContinent = VoxelDoubleFloat(Continent.InvNorm);
    Out.InvNormMountain = VoxelDoubleFloat(Mountain.InvNorm);
    Out.InvNormDetail = VoxelDoubleFloat(Detail.InvNorm);
    Out.PlanetRadius = VoxelDoubleFloat(PlanetRadius);
    Out.NoiseAmplitude = VoxelDoubleFloat(NoiseAmplitude);
    Out.ContinentWeight = VoxelDoubleFloat(Settings.ContinentWeight);
    Out.MountainWeight = VoxelDoubleFloat(Settings.MountainWeight);
    Out.MountainStart = VoxelDoubleFloat(Settings.MountainStart);
    Out.MountainBlend = VoxelDoubleFloat(FMath::Max(Settings.MountainBlend, 1e-6));
    Out.DetailWeight = VoxelDoubleFloat(Settings.DetailWeight);
    Out.RidgeGain = (float)Settings.RidgeGain;
    Out.SlopeDamping = (float)Settings.DetailSlopeDamping;
    Out.MinFootprint = MinFootprint;
    if (LandBake.IsValid())
    {
        Out.LandMode = 1;
        Out.Land = LandBake;
        Out.LandWarpOctaves = (uint32)LandWarp[0].Octaves.Num();
        Out.LandWarpInvNorm = (float)LandWarp[0].InvNorm;
        Out.LandWarpAmplitude = (float)LandWarpAmplitude;
        Out.LandDetailAmplitude = (float)LandDetailAmplitude;
        Out.LandHeightOffset = VoxelDoubleFloat(LandHeightOffset);
        Out.LandCellM = LandCellM;
        Out.LandGullyStrength = LandGullyStrength;
        Out.LandGullyOctaves = LandGullyOctaves;
        Out.LandGullyFlowGain = LandGullyFlowGain;
        Out.LandGullyDetail = LandGullyDetail;
        Out.LandGullySeed = LandGullySeed;
        Out.LandHillRelief = LandHillRelief;
        Out.LandHillFrequency = VoxelDoubleFloat(LandHillFrequency);
        Out.LandHillOctaves = LandHillOctaves;
        Out.LandHillSeed = LandHillSeed;
        Out.LandNoise = LandNoise;
        Out.LandPeakStrength = LandPeakStrength;
        Out.LandPeakTexelsPerCell = LandPeakTexelsPerCell;
        Out.LandPeakSeed = LandPeakSeed;
        Out.LandPeakSpacingM = LandPeakSpacingM;
        Out.LandPeakReliefM = LandPeakReliefM;
        Out.LandPeakReliefStart = LandPeakReliefStart;
        Out.LandPeakReliefEnd = LandPeakReliefEnd;
        Out.LandPeakMeanR = LandPeakMeanR;
        Out.LandPeakMeanG = LandPeakMeanG;
        Out.LandPeakScaleR = LandPeakScaleR;
        Out.LandPeakScaleG = LandPeakScaleG;
        Out.LandPeakScaleB = LandPeakScaleB;
        Out.LandPeakDetailTexelsPerCell = LandPeakDetailTexelsPerCell;
        Out.LandDetailMeanHard = LandDetailMeanHard;
        Out.LandDetailMeanSoft = LandDetailMeanSoft;
        Out.LandPeakDetailScaleHard = LandPeakDetailScaleHard;
        Out.LandPeakDetailScaleSoft = LandPeakDetailScaleSoft;
        Out.LandPeakDetailFade = LandPeakDetailFade;
        Out.LandHillTexelsPerCell = LandHillTexelsPerCell;
        Out.LandHillScaleR = LandHillScaleR;
        Out.LandHillScaleG = LandHillScaleG;
        Out.LandHillScaleB = LandHillScaleB;
        Out.LandHillMeanR = LandHillMeanR;
        Out.LandHillMeanG = LandHillMeanG;
        Out.LandHillSpacingM = LandHillSpacingM;
        Out.LandHillSwellTexelsPerCell = LandHillSwellTexelsPerCell;
        Out.LandHillSwellM = LandHillSwellM;
        Out.LandHillSwellScaleG = LandHillSwellScaleG;
        Out.LandHillDetailTexelsPerCell = LandHillDetailTexelsPerCell;
        Out.LandHillDetailScaleHard = LandHillDetailScaleHard;
        Out.LandHillDetailScaleSoft = LandHillDetailScaleSoft;
        Out.LandHillDetailFade = LandHillDetailFade;
        Out.LandHillFoothill = LandHillFoothill;
        Out.LandStrataTexelsPerCell = LandStrataTexelsPerCell;
        Out.LandStrataScaleR = LandStrataScaleR;
        Out.LandStrataScaleG = LandStrataScaleG;
        Out.LandStrataScaleB = LandStrataScaleB;
        Out.LandStrataMeanR = LandStrataMeanR;
        Out.LandStrataMeanG = LandStrataMeanG;
        Out.LandStrataSpacingM = LandStrataSpacingM;
        Out.LandStrataRelief = LandStrataRelief;
        Out.LandStrataFlat = LandStrataFlat;
        Out.LandStrataThreshold = LandStrataThreshold;
        Out.LandStrataWidth = LandStrataWidth;
        Out.LandStrataSeed = LandStrataSeed;
        Out.LandStrataDetailTexelsPerCell = LandStrataDetailTexelsPerCell;
        Out.LandStrataDetailScaleHard = LandStrataDetailScaleHard;
        Out.LandStrataDetailScaleSoft = LandStrataDetailScaleSoft;
        Out.LandStrataDetailFade = LandStrataDetailFade;
        Out.LandRivers = LandRivers;
        Out.LandRiverTilesPerFace = LandRiverTilesPerFace;
        Out.LandRiverRadiusM = LandRiverRadiusM;
        Out.LandRiverWidthScale = LandRiverWidthScale;
        Out.LandRiverValleyScale = LandRiverValleyScale;
        Out.LandRiverCarve = LandRiverCarve;
        Out.LandRiverDepthScale = LandRiverDepthScale;
        Out.LandRiverFloodplainScale = LandRiverFloodplainScale;
        Out.LandRiverMinLog2Area = LandRiverMinLog2Area;
        Out.LandLakeShoreBand = LandLakeShoreBand;
        Out.LandRiverMeander = LandRiverMeander;
        Out.LandRiverFalls = LandRiverFalls;
        Out.LandHasWater = LandHasWater;
        Out.LandDetailTexelsPerCell = LandDetailTexelsPerCell;
        Out.LandDetailScales = LandDetailScales;
        Out.LandDetailMeanA = LandDetailMeanA;
        Out.LandDetailScaleA = LandDetailScaleA;
        Out.LandDetailScaleGain = LandDetailScaleGain;
        Out.LandDetailFade = LandDetailFade;
        Out.LandDetailSeed = LandDetailSeed;
    }

    // The GPU converts lattice coordinates to int32 before wrapping; the CPU wraps through
    // int64. They agree while every coordinate (at most the planet radius times the highest
    // frequency, plus an offset below 750) stays below 2^31.
    double MaxFrequency = 0.0;
    for (const FLayer* Layer : { &Continent, &Mountain, &Detail, &LandWarp[0], &LandWarp[1], &LandWarp[2] })
    {
        if (Layer->Octaves.Num() > 0)
            MaxFrequency = FMath::Max(MaxFrequency, Layer->Octaves.Last().Frequency);
    }
    const double SampleRadius = LandBake.IsValid() ? FMath::Max(PlanetRadius, LandSampleRadius) : PlanetRadius;
    if (SampleRadius * MaxFrequency + 750.0 >= 2147483648.0)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelDensity] The finest octave reaches lattice coordinate %.3g (limit 2^31): GPU and CPU densities will differ. Use fewer octaves or a longer wavelength."),
            SampleRadius * MaxFrequency + 750.0);
    }
}

TSharedPtr<FDensitySampleCompositor> VoxelDensity::MakeCompositor(const FVoxelSurfaceSettings& Settings,
    double PlanetRadius, double NoiseAmplitude, TSharedPtr<FSparseEditStore> EditStore, double MinFootprint)
{
    const TSharedRef<const FVoxelDensityCPU> Density = MakeShared<FVoxelDensityCPU>(Settings, PlanetRadius, NoiseAmplitude, MinFootprint);
    TSharedPtr<FDensitySampleCompositor> Compositor = MakeShared<FDensitySampleCompositor>(EditStore);
    Compositor->SetBaseDensity(Density);
    Compositor->SetEditFootprintFloor(MinFootprint);
    Compositor->AddSampleLayer([Density](const FSampleInput& Input, float* DensityOut) {
        Density->Sample(Input, DensityOut);
        });
    return Compositor;
}

TSharedPtr<FDensitySampleCompositor> VoxelDensity::MakeTerrestrialCompositor(const FVoxelTerrestrialSettings& Land,
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake, double PlanetRadius, double NoiseAmplitude,
    TSharedPtr<FSparseEditStore> EditStore, double MinFootprint)
{
    const TSharedRef<const FVoxelDensityCPU> Density = MakeShared<FVoxelDensityCPU>(Land, MoveTemp(Bake), PlanetRadius, NoiseAmplitude, MinFootprint);
    TSharedPtr<FDensitySampleCompositor> Compositor = MakeShared<FDensitySampleCompositor>(EditStore);
    Compositor->SetBaseDensity(Density);
    Compositor->SetEditFootprintFloor(MinFootprint);
    Compositor->AddSampleLayer([Density](const FSampleInput& Input, float* DensityOut) {
        Density->Sample(Input, DensityOut);
        });
    return Compositor;
}
