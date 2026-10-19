// VoxelOceanWeather.h - The ocean's weather until the atmosphere (Design/OceanWaves.md section 4;
// stage O7): a procedural wind, storm and swell source, a pure function of the planet seed, the
// point and the weather clock (the ocean clock scaled and offset), so the server and every
// client see the same weather with nothing replicated. And a constant source (the ocean's
// Environment settings everywhere), for tuning.
//
// THE PROCEDURAL SOURCE (FVoxelOceanProceduralWeather), from local north (the planet's +Z):
// - Latitude bands: trade winds (easterly, toward the equator) below 30 degrees, westerlies
//   (poleward) from 30 to 60, polar easterlies above, each band's speed a sine across it (the
//   calms at the equator, 30 and 60 degrees and the poles).
// - Noise: a seeded smooth vector field on the sphere (two octaves, a few hundred km), drifting
//   in time (blended between seeded time slices).
// - Storms: StormCount slots, each living storm after storm. A storm is born at a seeded point
//   (15-55 degrees, either hemisphere), drifts with the background wind where it formed, grows
//   and fades over its life (sin^2), and turns cyclonically (counter-clockwise in the north)
//   with a 20 degree inflow; its wind peaks at StormWind at about a third of its radius. Its
//   storm value (0-1) falls off with distance as a Gaussian of its radius.
// - Swell: every storm sends swell outward, as the wind speed whose sea it matches
//   (SwellStrength of its wind), fading with distance over SwellRange, and none inside the
//   storm's core (its own sea is there). The swell vectors sum.
//
// Distances are set in km on the sphere and turned into angles by the planet's radius.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanWaves.h"

/** The procedural weather's settings (FVoxelOceanSettings, Weather category). */
struct FVoxelOceanWeatherSettings
{
    int32 Seed = 1;
    /** Band speeds (m/s, at each band's middle). */
    double TradeWind = 7.0;
    double Westerly = 10.0;
    double PolarWind = 5.0;
    /** The noise's speed (m/s), size (km) and how long a time slice lasts (hours). */
    double NoiseWind = 3.0;
    double NoiseSizeKm = 600.0;
    double NoiseHours = 4.0;
    /** Storms at once, their peak wind (m/s), radius (km) and life (hours). */
    int32 StormCount = 6;
    double StormWind = 22.0;
    double StormRadiusKm = 220.0;
    double StormHours = 3.0;
    /** The swell a storm sends, as a fraction of its wind, and how far it carries (km). */
    double SwellStrength = 0.6;
    double SwellRangeKm = 1500.0;
    /** The weather clock: ocean time x TimeScale + TimeOffset (s). */
    double TimeScale = 1.0;
    double TimeOffset = 0.0;

    bool operator==(const FVoxelOceanWeatherSettings& Other) const
    {
        return Seed == Other.Seed && TradeWind == Other.TradeWind && Westerly == Other.Westerly && PolarWind == Other.PolarWind
            && NoiseWind == Other.NoiseWind && NoiseSizeKm == Other.NoiseSizeKm && NoiseHours == Other.NoiseHours
            && StormCount == Other.StormCount && StormWind == Other.StormWind && StormRadiusKm == Other.StormRadiusKm
            && StormHours == Other.StormHours && SwellStrength == Other.SwellStrength && SwellRangeKm == Other.SwellRangeKm
            && TimeScale == Other.TimeScale && TimeOffset == Other.TimeOffset;
    }
    bool operator!=(const FVoxelOceanWeatherSettings& Other) const { return !(*this == Other); }
};

/** A storm at a weather time (for debugging and the checks). */
struct FVoxelOceanStorm
{
    FVector Centre = FVector::ZeroVector;   // unit direction (planet frame)
    double Strength = 0.0;                  // 0-1: its life's envelope times its seeded strength
    double Radius = 0.0;                    // rad
};

class VOXELPLUGIN_API FVoxelOceanProceduralWeather : public IVoxelOceanEnvironmentSource
{
public:
    /** Radius: the planet's (cm), for the km settings. */
    FVoxelOceanProceduralWeather(const FVoxelOceanWeatherSettings& InSettings, double InRadius);

    virtual double WeatherTime(double OceanTime) const override;
    virtual FVoxelOceanEnvironment SampleWeather(const FVector& PlanetPoint, double Time) const override;
    virtual void SampleWeatherBatch(TConstArrayView<FVector> PlanetPoints, double Time, TArrayView<FVoxelOceanEnvironment> Out) const override;

    /** The storms alive at a weather time. */
    void StormsAt(double Time, TArray<FVoxelOceanStorm>& Out) const;

    /** The background wind (bands and noise; m/s, planet frame, tangent) at a direction. */
    FVector BackgroundWind(const FVector& Dir, double Time) const;

    /** Wind (planet frame, tangent, m/s), storm (0-1) and swell (planet frame, tangent, m/s) at
     *  a direction, with the storms of that time (StormsAt; many points share them). */
    void Evaluate(const FVector& Dir, double Time, const TArray<FVoxelOceanStorm>& Storms, FVector& OutWind, double& OutStorm, FVector& OutSwell) const;

    const FVoxelOceanWeatherSettings& GetSettings() const { return Settings; }

private:
    FVoxelOceanWeatherSettings Settings;
    double Radius = 1.0;
};

/** The ocean's Environment settings everywhere. */
class VOXELPLUGIN_API FVoxelOceanConstantWeather : public IVoxelOceanEnvironmentSource
{
public:
    explicit FVoxelOceanConstantWeather(const FVoxelOceanEnvironment& InEnvironment) : Environment(InEnvironment) {}
    virtual FVoxelOceanEnvironment SampleWeather(const FVector& PlanetPoint, double Time) const override { return Environment; }
    const FVoxelOceanEnvironment& GetEnvironment() const { return Environment; }

private:
    FVoxelOceanEnvironment Environment;
};

namespace VoxelOceanWeather
{
    /** Local east and north at a direction (north toward the planet's +Z; at a pole, any). */
    VOXELPLUGIN_API void EastNorth(const FVector& Dir, FVector& OutEast, FVector& OutNorth);
    /** A tangent vector as speed and heading (rad from north toward east), and back. */
    VOXELPLUGIN_API void ToSpeedHeading(const FVector& Dir, const FVector& V, double& OutSpeed, double& OutHeading);
    VOXELPLUGIN_API FVector FromSpeedHeading(const FVector& Dir, double Speed, double Heading);
}
