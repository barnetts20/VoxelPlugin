#include "VoxelDensity.h"
#include "Async/ParallelFor.h"
#include "VoxelDensityGPU.h"
#include "VoxelShaderCompat.h"

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
}

namespace
{
    constexpr uint32 SaltContinent = 1u;
    constexpr uint32 SaltMountain = 2u;
    constexpr uint32 SaltDetail = 3u;

    /** Standard deviation of one octave of VoxelGradientNoise (measured over 400k points). */
    constexpr double SingleOctaveSigma = 0.1914;

    FVector2f DoubleFloat(double V)
    {
        const float Hi = (float)V;
        return FVector2f(Hi, (float)(V - (double)Hi));
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

FVoxelDensityCPU::FLayer FVoxelDensityCPU::MakeLayer(uint32 Kind, double WavelengthFraction, double InPlanetRadius, double InLacunarity, double InGain, int32 InOctaves, uint32 InSeed)
{
    FLayer Layer;
    Layer.Kind = Kind;
    const double Lacunarity = FMath::Max(InLacunarity, 1.01);
    const int32 NumOctaves = FMath::Max(InOctaves, 0);

    double Frequency = 1.0 / FMath::Max(WavelengthFraction * InPlanetRadius, 1e-3);
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
    using namespace VoxelShader;

    // The core keeps its running sum in float; each octave's contribution is taken from a
    // zeroed sum and added here in double, so the total keeps double precision.
    FVoxelFbmState State = VoxelFbmBegin();
    double Sum = 0.0;
    for (const FOctave& Octave : Layer.Octaves)
    {
        const float Band = VoxelOctaveBandWeight(Octave.Wavelength, (float)Footprint);
        if (Band <= 0.0f) break;

        uint3 Cell;
        float3 Frac;
        SplitLattice(SurfacePoint * Octave.Frequency + Octave.Offset, Cell, Frac);
        const FVoxelNoiseSample N = VoxelGradientNoise(Cell, Frac, Octave.Seed);

        State.Sum = 0.0f;
        State = VoxelLayerOctave(State, Layer.Kind, N, Octave.Amplitude * Band, (float)Settings.RidgeGain, (float)Settings.DetailSlopeDamping);
        Sum += State.Sum;
    }
    return Sum * Layer.InvNorm;
}

double FVoxelDensityCPU::HeightAt(const FVector& Dir, double Footprint) const
{
    const double Len = Dir.Size();
    if (Len <= 1e-12 || NoiseAmplitude <= 0.0) return 0.0;
    const FVector SurfacePoint = Dir * (PlanetRadius / Len);

    const double C = EvaluateLayer(Continent, SurfacePoint, Footprint);
    const double R = EvaluateLayer(Mountain, SurfacePoint, Footprint);
    const double D = EvaluateLayer(Detail, SurfacePoint, Footprint);

    // In double: a float height in [0, 1] would quantise the surface to NoiseAmplitude * 6e-8
    // (several cm on an Earth-sized planet). VoxelSampleDensityDF mirrors this in double-float.
    const double Base = 0.5 + 0.5 * Settings.ContinentWeight * C;
    const double MaskT = FMath::Clamp((C - Settings.MountainStart) / FMath::Max(Settings.MountainBlend, 1e-6), 0.0, 1.0);
    const double Mountains = Settings.MountainWeight * MaskT * MaskT * (3.0 - 2.0 * MaskT) * R;
    const double Details = Settings.DetailWeight * D;
    return NoiseAmplitude * FMath::Clamp(Base + Mountains + Details, 0.0, 1.0);
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
    for (const FLayer* Layer : { &Continent, &Mountain, &Detail })
    {
        for (const FOctave& Octave : Layer->Octaves)
        {
            FVoxelDensityGPUOctave& G = Out.Octaves.AddDefaulted_GetRef();
            G.Frequency = DoubleFloat(Octave.Frequency);
            G.OffsetX = DoubleFloat(Octave.Offset.X);
            G.OffsetY = DoubleFloat(Octave.Offset.Y);
            G.OffsetZ = DoubleFloat(Octave.Offset.Z);
            G.Wavelength = Octave.Wavelength;
            G.Amplitude = Octave.Amplitude;
            G.Seed = Octave.Seed;
        }
    }
    Out.ContinentOctaves = (uint32)Continent.Octaves.Num();
    Out.MountainOctaves = (uint32)Mountain.Octaves.Num();
    Out.DetailOctaves = (uint32)Detail.Octaves.Num();
    Out.InvNormContinent = DoubleFloat(Continent.InvNorm);
    Out.InvNormMountain = DoubleFloat(Mountain.InvNorm);
    Out.InvNormDetail = DoubleFloat(Detail.InvNorm);
    Out.PlanetRadius = DoubleFloat(PlanetRadius);
    Out.NoiseAmplitude = DoubleFloat(NoiseAmplitude);
    Out.ContinentWeight = DoubleFloat(Settings.ContinentWeight);
    Out.MountainWeight = DoubleFloat(Settings.MountainWeight);
    Out.MountainStart = DoubleFloat(Settings.MountainStart);
    Out.MountainBlend = DoubleFloat(FMath::Max(Settings.MountainBlend, 1e-6));
    Out.DetailWeight = DoubleFloat(Settings.DetailWeight);
    Out.RidgeGain = (float)Settings.RidgeGain;
    Out.SlopeDamping = (float)Settings.DetailSlopeDamping;
    Out.MinFootprint = MinFootprint;

    // The GPU converts lattice coordinates to int32 before wrapping; the CPU wraps through
    // int64. They agree while every coordinate (at most the planet radius times the highest
    // frequency, plus an offset below 750) stays below 2^31.
    double MaxFrequency = 0.0;
    for (const FLayer* Layer : { &Continent, &Mountain, &Detail })
    {
        if (Layer->Octaves.Num() > 0)
            MaxFrequency = FMath::Max(MaxFrequency, Layer->Octaves.Last().Frequency);
    }
    if (PlanetRadius * MaxFrequency + 750.0 >= 2147483648.0)
    {
        UE_LOG(LogTemp, Warning, TEXT("[VoxelDensity] The finest octave reaches lattice coordinate %.3g (limit 2^31): GPU and CPU densities will differ. Use fewer octaves or a longer wavelength."),
            PlanetRadius * MaxFrequency + 750.0);
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
