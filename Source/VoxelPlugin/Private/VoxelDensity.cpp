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
        uint LandPeakDetailBase = 0;
        uint LandPeakDetailN = 0;
        uint LandPeakDetailMips = 0;
        float LandPeakDetailTexelsPerCell = 0.0f;
        float LandPeakDetailMeanR = 0.0f;
        float LandPeakDetailMeanG = 0.0f;
        float LandPeakDetailScaleR = 0.0f;
        float LandPeakDetailScaleG = 0.0f;
        float LandPeakDetailFade = 0.0f;
        float LandHillTexelsPerCell = 0.0f;
        uint LandHillBase = 0;
        uint LandHillN = 0;
        uint LandHillMips = 0;
        float LandHillScaleR = 1.0f;
        float LandHillScaleG = 1.0f;
        float LandHillScaleB = 1.0f;
        float LandHillSpacingM = 0.0f;
        float LandHillSwellTexelsPerCell = 0.0f;
        float LandHillSwellM = 0.0f;
        float LandHillSwellScaleG = 1.0f;
        float LandHillDetailTexelsPerCell = 0.0f;
        float LandHillDetailScaleR = 0.0f;
        float LandHillDetailScaleG = 0.0f;
        float LandHillDetailFade = 0.0f;
        float LandHillFoothill = 0.0f;

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

    // The detail layer through the noise surface's machinery (no continents or mountains):
    // octaves from DetailWavelength down to the finest cell, slope-damped, gain 0.5.
    Settings = FVoxelSurfaceSettings();
    Settings.Seed = InLand.Seed;
    Settings.Lacunarity = 2.0;
    Settings.ContinentOctaves = 0;
    Settings.MountainOctaves = 0;
    const double DetailWavelength = FMath::Max(InLand.DetailWavelength, 1.0) * 100.0;
    Settings.DetailWavelength = DetailWavelength / SampleRadius;
    Settings.DetailGain = 0.5;
    Settings.DetailSlopeDamping = FMath::Max(InLand.DetailSlopeDamping, 0.0);
    const double Finest = FMath::Max(MinFootprint, 1.0);
    Settings.DetailOctaves = FMath::Clamp(FMath::CeilToInt(FMath::Log2(DetailWavelength / Finest)) + 1, 1, 24);
    Continent = MakeLayer(VoxelShader::VoxelLayerPlain, 1.0, SampleRadius, 2.0, 0.5, 0, VoxelShader::VoxelHashSeed(Seed, SaltContinent));
    Mountain = MakeLayer(VoxelShader::VoxelLayerRidged, 1.0, SampleRadius, 2.0, 0.5, 0, VoxelShader::VoxelHashSeed(Seed, SaltMountain));
    Detail = MakeLayer(VoxelShader::VoxelLayerDamped, Settings.DetailWavelength, SampleRadius, 2.0, Settings.DetailGain, Settings.DetailOctaves, VoxelShader::VoxelHashSeed(Seed, SaltDetail));

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
    }
    // The noise pack: the peaks' set, the detail tile, the hills' set (T4a).
    if (LandFormPacked.Num() > 0 && LandReliefPacked.Num() > 0)
        LandNoise = VoxelLand::GetLandNoise(VoxelLand::LandNoiseNameFor(InLand));
    if (LandNoise.IsValid() && LandPeakStrength > 0.0f && LandNoise->N > 0)
        LandPeakTexelsPerCell = VoxelLand::PeakTexelsPerCell(InLand, *LandBake, *LandNoise);
    if (LandNoise.IsValid() && LandNoise->DetailN > 0)
    {
        // The detail tile's means, whoever reads it (the peaks, the hills' set).
        LandPeakDetailMeanR = LandNoise->DetailMeanR;
        LandPeakDetailMeanG = LandNoise->DetailMeanG;
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
        // The detail tile (T3e): its own texel; its heights (texels) over the peaks' relief, so
        // strength 1 keeps its slopes; faded out by the mip level where its coarsest octave
        // (DetailSpacingTexels) has two texels.
        LandPeakDetailTexelsPerCell = VoxelLand::PeakDetailTexelsPerCell(InLand, *LandBake, Set);
        if (LandPeakDetailTexelsPerCell > 0.0f)
        {
            const float DetailTexelM = LandCellM / LandPeakDetailTexelsPerCell;
            const float Strength = (float)InLand.PeakDetailStrength;
            LandPeakDetailScaleR = Strength * Set.DetailRangeR * DetailTexelM / LandPeakReliefM;
            LandPeakDetailScaleG = Strength * Set.DetailRangeG * DetailTexelM / LandPeakReliefM;
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
    if (LandNoise.IsValid() && LandHillRelief > 0.0f && LandNoise->HillN > 0)
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
        if (InLand.HillSwellHeight > 0.0)
        {
            LandHillSwellTexelsPerCell = LandHillTexelsPerCell / (float)FMath::Clamp(InLand.HillSwellScale, 1.5, 8.0);
            LandHillSwellM = (float)InLand.HillSwellHeight;
            // G's whole range is the swells' unit.
            LandHillSwellScaleG = 1.0f;
        }
        // The detail tile, as the peaks': its heights (texels) over the set's relief, so strength 1
        // keeps its slopes where the hills keep theirs.
        if (InLand.HillDetailStrength > 0.0 && Set.DetailN > 0 && Set.DetailSpacingTexels > 0)
        {
            const double DetailTexelM = VoxelLand::HillDetailWavelength(InLand) / (double)Set.DetailSpacingTexels;
            LandHillDetailTexelsPerCell = (float)(LandCellM / DetailTexelM);
            const float DetailTexelMF = LandCellM / LandHillDetailTexelsPerCell;
            const float Strength = (float)InLand.HillDetailStrength;
            LandHillDetailScaleR = Strength * Set.DetailRangeR * DetailTexelMF / ReliefM;
            LandHillDetailScaleG = Strength * Set.DetailRangeG * DetailTexelMF / ReliefM;
            LandHillDetailFade = (float)FMath::FloorLog2(Set.DetailSpacingTexels) - 1.0f;
        }
    }
    if (LandPeakTexelsPerCell <= 0.0f && LandHillTexelsPerCell <= 0.0f)
        LandNoise.Reset();
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
    if (LandNoise.IsValid())
    {
        Sampler.LandNoise = LandNoise->Words.GetData();
        // The detail tile's chain and means, whoever reads it.
        Sampler.LandPeakDetailBase = LandNoise->DetailBase;
        Sampler.LandPeakDetailN = LandNoise->DetailN;
        Sampler.LandPeakDetailMips = LandNoise->DetailMips;
        Sampler.LandPeakDetailMeanR = LandPeakDetailMeanR;
        Sampler.LandPeakDetailMeanG = LandPeakDetailMeanG;
        Sampler.LandHillTexelsPerCell = LandHillTexelsPerCell;
        Sampler.LandHillBase = LandNoise->HillBase;
        Sampler.LandHillN = LandNoise->HillN;
        Sampler.LandHillMips = LandNoise->HillMips;
        Sampler.LandHillScaleR = LandHillScaleR;
        Sampler.LandHillScaleG = LandHillScaleG;
        Sampler.LandHillScaleB = LandHillScaleB;
        Sampler.LandHillSpacingM = LandHillSpacingM;
        Sampler.LandHillSwellTexelsPerCell = LandHillSwellTexelsPerCell;
        Sampler.LandHillSwellM = LandHillSwellM;
        Sampler.LandHillSwellScaleG = LandHillSwellScaleG;
        Sampler.LandHillDetailTexelsPerCell = LandHillDetailTexelsPerCell;
        Sampler.LandHillDetailScaleR = LandHillDetailScaleR;
        Sampler.LandHillDetailScaleG = LandHillDetailScaleG;
        Sampler.LandHillDetailFade = LandHillDetailFade;
        Sampler.LandHillFoothill = LandHillFoothill;
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
            Sampler.LandPeakDetailScaleR = LandPeakDetailScaleR;
            Sampler.LandPeakDetailScaleG = LandPeakDetailScaleG;
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

    // Detail, a quarter as strong under the sea and scaled by GullyDetailScale on land while
    // there are gullies (VoxelDensitySample.ush in float the same way).
    const float DetailWeight = lerp(0.25f, 1.0f, smoothstep(-50.0f, 50.0f, Land.Bake));
    const double D = EvaluateLayer(Detail, SurfacePoint, Footprint);
    const double Height = (double)Land.Bake * 100.0 + LandHeightOffset + D * (double)((float)LandDetailAmplitude * DetailWeight * Land.DetailScale)
        + (double)Land.Hills * 100.0 + (double)Land.Peaks * 100.0 + (double)Land.Gully * 100.0;
    return FMath::Clamp(Height, 0.0, NoiseAmplitude);
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
        Out.LandPeakDetailMeanR = LandPeakDetailMeanR;
        Out.LandPeakDetailMeanG = LandPeakDetailMeanG;
        Out.LandPeakDetailScaleR = LandPeakDetailScaleR;
        Out.LandPeakDetailScaleG = LandPeakDetailScaleG;
        Out.LandPeakDetailFade = LandPeakDetailFade;
        Out.LandHillTexelsPerCell = LandHillTexelsPerCell;
        Out.LandHillScaleR = LandHillScaleR;
        Out.LandHillScaleG = LandHillScaleG;
        Out.LandHillScaleB = LandHillScaleB;
        Out.LandHillSpacingM = LandHillSpacingM;
        Out.LandHillSwellTexelsPerCell = LandHillSwellTexelsPerCell;
        Out.LandHillSwellM = LandHillSwellM;
        Out.LandHillSwellScaleG = LandHillSwellScaleG;
        Out.LandHillDetailTexelsPerCell = LandHillDetailTexelsPerCell;
        Out.LandHillDetailScaleR = LandHillDetailScaleR;
        Out.LandHillDetailScaleG = LandHillDetailScaleG;
        Out.LandHillDetailFade = LandHillDetailFade;
        Out.LandHillFoothill = LandHillFoothill;
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
