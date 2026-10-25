// VoxelOceanAtmosphereWeather.h - Stage A5: the ocean's weather from the atmosphere's flow sim
// (Design/AtmosphereOceanSide.md section 3): the wind, the storm and a swell from the field the
// atmosphere draws, read back to the CPU every few seconds (CloudAtmosphere's
// FAtmosphereWeatherFeed, AtmosphereWeather.h), through the same interface the procedural weather
// uses, so the waves' keys, the charts and the query take it unchanged.
//
// THE WIND: the sim's lowest layer, scaled from sim units so its mean over the sphere is MeanWind
// (m/s): the sim is dimensionless, and its own speeds move with its tune. The heading from local
// north toward east, as the procedural weather's (VoxelOceanWeather::ToSpeedHeading).
// THE STORM: the column's, 0 to 1, as the sim has it.
// THE SWELL: the wind at a weather system's scale (the map's box-filtered copy), SwellShare of it,
// along its heading: a stand-in for the swell distant storms send.
//
// THE CLOCK is the fallback's (the procedural weather's), so its time scale and the sweep's hold
// apply. The map is whatever has landed last: the waves' keys, built ahead, take the weather as
// it is when they're built (within one key, 300 s).
//
// UNTIL THE FIRST MAP LANDS, and on a map-less atmosphere, the fallback answers. NOT
// DETERMINISTIC ACROSS MACHINES (the sim runs on each one's GPU): a game needing the same waves
// everywhere replicates the server's map, or keeps the procedural weather.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanWaves.h"

class FAtmosphereWeatherFeed;
struct FAtmosphereWeatherMap;

class FVoxelOceanAtmosphereWeather final : public IVoxelOceanEnvironmentSource
{
public:
    FVoxelOceanAtmosphereWeather(const TSharedPtr<FAtmosphereWeatherFeed, ESPMode::ThreadSafe>& InFeed,
        const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& InFallback, double InMeanWind, double InSwellShare);

    virtual double WeatherTime(double OceanTime) const override;
    virtual FVoxelOceanEnvironment SampleWeather(const FVector& PlanetPoint, double WeatherTime) const override;
    virtual void SampleWeatherBatch(TConstArrayView<FVector> PlanetPoints, double WeatherTime, TArrayView<FVoxelOceanEnvironment> Out) const override;

    /** Whether a map has landed (the fallback answers until then). */
    bool HasMap() const;

    /** The environment at a direction from a map, its wind scaled so the map's mean is MeanWind. */
    static FVoxelOceanEnvironment FromMap(const FAtmosphereWeatherMap& Map, const FVector& PlanetPoint, double MeanWind, double SwellShare);

private:
    TSharedPtr<const FAtmosphereWeatherMap, ESPMode::ThreadSafe> LatestMap() const;

    TSharedPtr<FAtmosphereWeatherFeed, ESPMode::ThreadSafe> Feed;
    TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> Fallback;
    double MeanWind = 7.0;
    double SwellShare = 0.6;
};
