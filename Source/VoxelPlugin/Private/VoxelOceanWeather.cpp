#include "VoxelOceanWeather.h"

#include <cmath>

namespace VoxelOceanWeatherDetail
{
    constexpr double Pi = 3.14159265358979323846;
    constexpr double Degree = Pi / 180.0;

    uint32 Hash(uint32 X)
    {
        X ^= X >> 16;
        X *= 0x7feb352du;
        X ^= X >> 15;
        X *= 0x846ca68bu;
        X ^= X >> 16;
        return X;
    }
    uint32 Hash(uint32 A, uint32 B) { return Hash(A ^ Hash(B + 0x9e3779b9u)); }
    uint32 Hash(uint32 A, uint32 B, uint32 C) { return Hash(Hash(A, B), C); }
    /** [0, 1) from a hash. */
    double Unit(uint32 H) { return double(H & 0xffffffu) / 16777216.0; }

    double SmoothStep(double A, double B, double X)
    {
        const double T = FMath::Clamp((X - A) / (B - A), 0.0, 1.0);
        return T * T * (3.0 - 2.0 * T);
    }
    double Quintic(double T) { return T * T * T * (T * (T * 6.0 - 15.0) + 10.0); }

    /** Value noise in [-1, 1] at P, one channel of a seed. */
    double ValueNoise(const FVector& P, uint32 Seed)
    {
        const double FX = std::floor(P.X), FY = std::floor(P.Y), FZ = std::floor(P.Z);
        const int32 IX = (int32)FX, IY = (int32)FY, IZ = (int32)FZ;
        const double TX = Quintic(P.X - FX), TY = Quintic(P.Y - FY), TZ = Quintic(P.Z - FZ);
        double Corner[8];
        for (int32 c = 0; c < 8; c++)
        {
            const uint32 H = Hash(Seed, Hash((uint32)(IX + (c & 1)), (uint32)(IY + ((c >> 1) & 1)), (uint32)(IZ + ((c >> 2) & 1))));
            Corner[c] = 2.0 * Unit(H) - 1.0;
        }
        const double X00 = Corner[0] + (Corner[1] - Corner[0]) * TX;
        const double X10 = Corner[2] + (Corner[3] - Corner[2]) * TX;
        const double X01 = Corner[4] + (Corner[5] - Corner[4]) * TX;
        const double X11 = Corner[6] + (Corner[7] - Corner[6]) * TX;
        const double Y0 = X00 + (X10 - X00) * TY;
        const double Y1 = X01 + (X11 - X01) * TY;
        return Y0 + (Y1 - Y0) * TZ;
    }

    /** A smooth vector field (two octaves), each component in about [-1, 1]. */
    FVector VectorNoise(const FVector& P, uint32 Seed)
    {
        FVector V(0.0, 0.0, 0.0);
        double Amplitude = 1.0, Frequency = 1.0;
        for (int32 Octave = 0; Octave < 2; Octave++)
        {
            const FVector Q = P * Frequency;
            const uint32 S = Hash(Seed, (uint32)Octave);
            V += FVector(ValueNoise(Q, Hash(S, 1u)), ValueNoise(Q, Hash(S, 2u)), ValueNoise(Q, Hash(S, 3u))) * Amplitude;
            Amplitude *= 0.5;
            Frequency *= 2.0;
        }
        return V * (1.0 / 1.5);
    }

    /** Dir turned by Angle about the unit Axis. */
    FVector Rotate(const FVector& Dir, const FVector& Axis, double Angle)
    {
        const double C = std::cos(Angle), S = std::sin(Angle);
        return Dir * C + (Axis ^ Dir) * S + Axis * ((Axis | Dir) * (1.0 - C));
    }
}

namespace VOWE = VoxelOceanWeatherDetail;

// ---------------------------------------------------------------------------------------------

void VoxelOceanWeather::EastNorth(const FVector& Dir, FVector& OutEast, FVector& OutNorth)
{
    const FVector Z(0.0, 0.0, 1.0);
    FVector East = Z ^ Dir;
    const double Length = East.Size();
    if (Length < 1.0e-9)
    {
        // At a pole: any frame.
        East = FVector(1.0, 0.0, 0.0);
        East = (East - Dir * (East | Dir)).GetSafeNormal();
    }
    else
    {
        East = East * (1.0 / Length);
    }
    OutEast = East;
    OutNorth = Dir ^ East;
}

void VoxelOceanWeather::ToSpeedHeading(const FVector& Dir, const FVector& V, double& OutSpeed, double& OutHeading)
{
    FVector East, North;
    EastNorth(Dir, East, North);
    const double E = V | East, N = V | North;
    OutSpeed = std::sqrt(E * E + N * N);
    OutHeading = OutSpeed > 0.0 ? std::atan2(E, N) : 0.0;
}

FVector VoxelOceanWeather::FromSpeedHeading(const FVector& Dir, double Speed, double Heading)
{
    FVector East, North;
    EastNorth(Dir, East, North);
    return (East * std::sin(Heading) + North * std::cos(Heading)) * Speed;
}

// ---------------------------------------------------------------------------------------------

FVoxelOceanProceduralWeather::FVoxelOceanProceduralWeather(const FVoxelOceanWeatherSettings& InSettings, double InRadius)
    : Settings(InSettings)
    , Radius(FMath::Max(InRadius, 1.0))
{
    Settings.StormCount = FMath::Clamp(Settings.StormCount, 0, 64);
    Settings.NoiseSizeKm = FMath::Max(Settings.NoiseSizeKm, 1.0);
    Settings.NoiseHours = FMath::Max(Settings.NoiseHours, 0.01);
    Settings.StormRadiusKm = FMath::Max(Settings.StormRadiusKm, 1.0);
    Settings.StormHours = FMath::Max(Settings.StormHours, 0.01);
    Settings.SwellRangeKm = FMath::Max(Settings.SwellRangeKm, 1.0);
}

double FVoxelOceanProceduralWeather::WeatherTime(double OceanTime) const
{
    return OceanTime * Settings.TimeScale + Settings.TimeOffset;
}

FVector FVoxelOceanProceduralWeather::BackgroundWind(const FVector& Dir, double Time) const
{
    // The bands: east (u) and north (v) components by latitude.
    const double Latitude = std::asin(FMath::Clamp(Dir.Z, -1.0, 1.0)) / VOWE::Degree;
    const double A = FMath::Abs(Latitude);
    const double Sign = Latitude >= 0.0 ? 1.0 : -1.0;
    double U, V;
    if (A < 30.0)
    {
        const double S = std::sin(VOWE::Pi * A / 30.0);
        U = -Settings.TradeWind * S;
        V = -Sign * 0.4 * Settings.TradeWind * S;      // toward the equator
    }
    else if (A < 60.0)
    {
        const double S = std::sin(VOWE::Pi * (A - 30.0) / 30.0);
        U = Settings.Westerly * S;
        V = Sign * 0.3 * Settings.Westerly * S;        // poleward
    }
    else
    {
        const double S = std::sin(VOWE::Pi * (A - 60.0) / 30.0);
        U = -Settings.PolarWind * S;
        V = -Sign * 0.3 * Settings.PolarWind * S;
    }
    FVector East, North;
    VoxelOceanWeather::EastNorth(Dir, East, North);
    FVector Wind = East * U + North * V;

    // The noise: blended between seeded time slices, keeping its variance.
    if (Settings.NoiseWind > 0.0)
    {
        const double SliceTime = Time / (Settings.NoiseHours * 3600.0);
        const double Slice = std::floor(SliceTime);
        const double Blend = VOWE::SmoothStep(0.0, 1.0, SliceTime - Slice);
        const FVector P = Dir * (Radius * 1.0e-5 / Settings.NoiseSizeKm);
        const uint32 Seed = VOWE::Hash((uint32)Settings.Seed, 0x5eedu);
        FVector Noise(0.0, 0.0, 0.0);
        for (int32 k = 0; k < 2; k++)
        {
            const int64 Index = (int64)Slice + k;
            const uint32 H = VOWE::Hash(Seed, (uint32)(Index & 0xffffffff), (uint32)(Index >> 32));
            const FVector Offset(100.0 * VOWE::Unit(VOWE::Hash(H, 1u)), 100.0 * VOWE::Unit(VOWE::Hash(H, 2u)), 100.0 * VOWE::Unit(VOWE::Hash(H, 3u)));
            const double W = k == 0 ? std::cos(0.5 * VOWE::Pi * Blend) : std::sin(0.5 * VOWE::Pi * Blend);
            Noise += VOWE::VectorNoise(P + Offset, H) * W;
        }
        Noise -= Dir * (Noise | Dir);
        Wind += Noise * Settings.NoiseWind;
    }
    return Wind;
}

void FVoxelOceanProceduralWeather::StormsAt(double Time, TArray<FVoxelOceanStorm>& Out) const
{
    Out.Reset();
    const double Life = Settings.StormHours * 3600.0;
    const double RadiusM = Radius * 0.01;
    const double RadiusKm = Radius * 1.0e-5;
    for (int32 Slot = 0; Slot < Settings.StormCount; Slot++)
    {
        const uint32 SlotSeed = VOWE::Hash((uint32)Settings.Seed, 0x570e0u, (uint32)Slot);
        const double X = (Time + VOWE::Unit(SlotSeed) * Life) / Life;
        const double Generation = std::floor(X);
        const double Age = X - Generation;
        const int64 G = (int64)Generation;
        const uint32 H = VOWE::Hash(SlotSeed, (uint32)(G & 0xffffffff), (uint32)(G >> 32));
        // Born at 15-55 degrees, either hemisphere, any longitude.
        const double Latitude = (15.0 + 40.0 * VOWE::Unit(VOWE::Hash(H, 1u))) * VOWE::Degree * (VOWE::Unit(VOWE::Hash(H, 2u)) < 0.5 ? -1.0 : 1.0);
        const double Longitude = 2.0 * VOWE::Pi * VOWE::Unit(VOWE::Hash(H, 3u));
        const FVector Birth(std::cos(Latitude) * std::cos(Longitude), std::cos(Latitude) * std::sin(Longitude), std::sin(Latitude));
        // Drifting with the background wind where and when it formed.
        const FVector Drift = BackgroundWind(Birth, Time - Age * Life) * 0.7;
        const double DriftSpeed = Drift.Size();
        FVoxelOceanStorm Storm;
        Storm.Centre = Birth;
        if (DriftSpeed > 1.0e-6)
        {
            const FVector Axis = (Birth ^ Drift).GetSafeNormal();
            Storm.Centre = VOWE::Rotate(Birth, Axis, DriftSpeed * Age * Life / RadiusM).GetSafeNormal();
        }
        const double Envelope = std::sin(VOWE::Pi * Age);
        Storm.Strength = Envelope * Envelope * (0.6 + 0.4 * VOWE::Unit(VOWE::Hash(H, 4u)));
        Storm.Radius = Settings.StormRadiusKm * (0.8 + 0.4 * VOWE::Unit(VOWE::Hash(H, 5u))) / RadiusKm;
        Out.Add(Storm);
    }
}

void FVoxelOceanProceduralWeather::Evaluate(const FVector& Dir, double Time, const TArray<FVoxelOceanStorm>& Storms,
    FVector& OutWind, double& OutStorm, FVector& OutSwell) const
{
    const FVector Background = BackgroundWind(Dir, Time);
    const double RadiusKm = Radius * 1.0e-5;
    const double Inflow = 20.0 * VOWE::Degree;
    double Calm = 1.0;
    FVector StormWind(0.0, 0.0, 0.0);
    FVector Swell(0.0, 0.0, 0.0);
    for (const FVoxelOceanStorm& Storm : Storms)
    {
        if (Storm.Strength <= 0.0) continue;
        const double Distance = std::acos(FMath::Clamp(Dir | Storm.Centre, -1.0, 1.0));
        Calm *= 1.0 - Storm.Strength * std::exp(-FMath::Square(Distance / Storm.Radius));
        // Around the centre (counter-clockwise in the north), turned in toward it.
        const FVector Cross = Storm.Centre ^ Dir;
        const double CrossLength = Cross.Size();
        if (CrossLength < 1.0e-9) continue;
        const FVector Around = Cross * ((Storm.Centre.Z >= 0.0 ? 1.0 : -1.0) / CrossLength);
        const FVector Away = (Cross * (1.0 / CrossLength)) ^ Dir;
        const double X = Distance / (0.35 * Storm.Radius);
        const double Speed = Settings.StormWind * Storm.Strength * X * std::exp(1.0 - X);
        StormWind += (Around * std::cos(Inflow) - Away * std::sin(Inflow)) * Speed;
        // Swell outward, beyond the core.
        const double SwellSpeed = Settings.SwellStrength * Settings.StormWind * Storm.Strength
            * std::exp(-Distance * RadiusKm / Settings.SwellRangeKm) * VOWE::SmoothStep(Storm.Radius, 2.0 * Storm.Radius, Distance);
        Swell += Away * SwellSpeed;
    }
    OutStorm = 1.0 - Calm;
    OutWind = Background * (1.0 - 0.7 * OutStorm) + StormWind;
    OutSwell = Swell;
}

FVoxelOceanEnvironment FVoxelOceanProceduralWeather::SampleWeather(const FVector& PlanetPoint, double Time) const
{
    FVoxelOceanEnvironment E;
    SampleWeatherBatch(TConstArrayView<FVector>(&PlanetPoint, 1), Time, TArrayView<FVoxelOceanEnvironment>(&E, 1));
    return E;
}

void FVoxelOceanProceduralWeather::SampleWeatherBatch(TConstArrayView<FVector> PlanetPoints, double Time, TArrayView<FVoxelOceanEnvironment> Out) const
{
    TArray<FVoxelOceanStorm> Storms;
    StormsAt(Time, Storms);
    for (int32 i = 0; i < PlanetPoints.Num() && i < Out.Num(); i++)
    {
        const FVector Dir = PlanetPoints[i].GetSafeNormal();
        FVector Wind, Swell;
        double Storm = 0.0;
        Evaluate(Dir, Time, Storms, Wind, Storm, Swell);
        FVoxelOceanEnvironment& E = Out[i];
        E = FVoxelOceanEnvironment();
        VoxelOceanWeather::ToSpeedHeading(Dir, Wind, E.WindSpeed, E.WindHeading);
        VoxelOceanWeather::ToSpeedHeading(Dir, Swell, E.SwellSpeed, E.SwellHeading);
        E.Storm = Storm;
    }
}
