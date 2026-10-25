#include "VoxelOceanAtmosphereWeather.h"
#include "VoxelOceanWeather.h"
#include "AtmosphereWeather.h"

FVoxelOceanAtmosphereWeather::FVoxelOceanAtmosphereWeather(const TSharedPtr<FAtmosphereWeatherFeed, ESPMode::ThreadSafe>& InFeed,
    const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& InFallback, double InMeanWind, double InSwellShare)
    : Feed(InFeed)
    , Fallback(InFallback)
    , MeanWind(FMath::Max(InMeanWind, 0.0))
    , SwellShare(FMath::Max(InSwellShare, 0.0))
{
}

double FVoxelOceanAtmosphereWeather::WeatherTime(double OceanTime) const
{
    return Fallback.IsValid() ? Fallback->WeatherTime(OceanTime) : OceanTime;
}

TSharedPtr<const FAtmosphereWeatherMap, ESPMode::ThreadSafe> FVoxelOceanAtmosphereWeather::LatestMap() const
{
    return Feed.IsValid() ? Feed->Latest() : TSharedPtr<const FAtmosphereWeatherMap, ESPMode::ThreadSafe>();
}

bool FVoxelOceanAtmosphereWeather::HasMap() const
{
    return LatestMap().IsValid();
}

FVoxelOceanEnvironment FVoxelOceanAtmosphereWeather::FromMap(const FAtmosphereWeatherMap& Map, const FVector& PlanetPoint, double MeanWind,
    double SwellShare)
{
    FVoxelOceanEnvironment E;
    const FVector Dir = PlanetPoint.GetSafeNormal();
    if (Dir.SizeSquared() < 0.5) return E;
    // Sim units to m/s: the map's mean speed is MeanWind.
    const double Scale = Map.MeanSpeed > 1.0e-6f ? MeanWind / (double)Map.MeanSpeed : 0.0;
    const FVector4f Here = Map.Sample(Dir);
    VoxelOceanWeather::ToSpeedHeading(Dir, FVector(Here.X, Here.Y, Here.Z) * Scale, E.WindSpeed, E.WindHeading);
    E.Storm = FMath::Clamp((double)Here.W, 0.0, 1.0);
    const FVector4f Broad = Map.SampleBroad(Dir);
    double BroadSpeed = 0.0, BroadHeading = 0.0;
    VoxelOceanWeather::ToSpeedHeading(Dir, FVector(Broad.X, Broad.Y, Broad.Z) * Scale, BroadSpeed, BroadHeading);
    E.SwellSpeed = BroadSpeed * SwellShare;
    E.SwellHeading = BroadHeading;
    return E;
}

FVoxelOceanEnvironment FVoxelOceanAtmosphereWeather::SampleWeather(const FVector& PlanetPoint, double Time) const
{
    const TSharedPtr<const FAtmosphereWeatherMap, ESPMode::ThreadSafe> Map = LatestMap();
    if (Map.IsValid())
        return FromMap(*Map, PlanetPoint, MeanWind, SwellShare);
    return Fallback.IsValid() ? Fallback->SampleWeather(PlanetPoint, Time) : FVoxelOceanEnvironment();
}

void FVoxelOceanAtmosphereWeather::SampleWeatherBatch(TConstArrayView<FVector> PlanetPoints, double Time, TArrayView<FVoxelOceanEnvironment> Out) const
{
    // One map for the whole batch.
    const TSharedPtr<const FAtmosphereWeatherMap, ESPMode::ThreadSafe> Map = LatestMap();
    if (!Map.IsValid())
    {
        if (Fallback.IsValid())
            Fallback->SampleWeatherBatch(PlanetPoints, Time, Out);
        return;
    }
    for (int32 i = 0; i < PlanetPoints.Num() && i < Out.Num(); i++)
        Out[i] = FromMap(*Map, PlanetPoints[i], MeanWind, SwellShare);
}
