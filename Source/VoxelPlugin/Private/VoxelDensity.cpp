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

    /** The land bake's sampler on the CPU: VoxelLandSample.ush and VoxelLandGully.ush with the
     *  shader's buffers and constants as members (the GPU runs the same code). */
    struct FVoxelLandSamplerCPU
    {
        const float* LandHeight = nullptr;
        const float2* LandTanTable = nullptr;
        const uint* LandFlow = nullptr;
        uint LandN = 1;
        uint LandPad = 0;
        uint LandHasFlow = 0;
        float LandCellM = 1.0f;
        float LandGullyStrength = 0.0f;
        uint LandGullyOctaves = 0;
        float LandGullyFlowGain = 0.0f;
        float LandGullyDetail = 1.0f;
        uint LandGullySeed = 0;

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
        ToDoubleFloat(Q.X), ToDoubleFloat(Q.Y), ToDoubleFloat(Q.Z), ToFloat3(DW[0]), ToFloat3(DW[1]), ToFloat3(DW[2]), (float)Footprint);

    // Detail, a quarter as strong under the sea and scaled by GullyDetailScale on land while
    // there are gullies (VoxelDensitySample.ush in float the same way).
    const float DetailWeight = lerp(0.25f, 1.0f, smoothstep(-50.0f, 50.0f, Land.Bake));
    const double D = EvaluateLayer(Detail, SurfacePoint, Footprint);
    const double Height = (double)Land.Bake * 100.0 + LandHeightOffset + D * (double)((float)LandDetailAmplitude * DetailWeight * Land.DetailScale)
        + (double)Land.Gully * 100.0;
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
    Compositor->AddSampleLayer([Density](const FSampleInput& Input, float* DensityOut) {
        Density->Sample(Input, DensityOut);
        });
    return Compositor;
}
