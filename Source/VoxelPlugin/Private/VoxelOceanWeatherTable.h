// VoxelOceanWeatherTable.h - The weather as the FFT waves take it (Design/OceanWaves.md section 4;
// stage O7): an environment source (VoxelOceanWeather.h, or the atmosphere's later) sampled at
// keys in its clock, written into the chart table's other regions (VoxelOceanSeaState.ush), which
// the GPU and the CPU model read alike.
//
// A KEY (FVoxelOceanWeatherKey) is the weather at weather time k KeySeconds:
// - the field: wind and swell speed at every field cell of every face, its border included;
// - each chart's headings: the long bands' (the swell's heading where the swell outweighs the
//   wind at the anchor, else the wind's) and the short bands' (the wind's), in the chart's plane,
//   turned to whole HeadingStep steps, so most keys leave most charts as they were;
// - the greatest wind and long-wave speed over the field.
// Keys are a pure function of the source and k: every machine builds the same.
//
// THE SNAPSHOT (FVoxelOceanWeatherTable) of the interval from key k to k + 1: the whole table
// (the charts, headings from keys k - 1 and k, the field of keys k and k + 1, the model's
// constants), immutable, shared by the frame, the queries and the GPU upload. At a weather time
// in the interval, Alpha runs the field from key k to k + 1 and Fade turns the headings from key
// k - 1's to key k's over the first FadeSeconds.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanWaves.h"
#include "VoxelOceanCharts.h"

#include <vector>

/** The weather at one key. */
struct FVoxelOceanWeatherKey
{
    int64 Index = 0;
    /** Per face, VOXEL_OCEAN_FIELD_SIDE^2 cells (row by row), (wind, swell) m/s each. */
    std::vector<float> Field;
    /** Per chart (face N N + j N + i), (long x, long y, short x, short y). */
    std::vector<float> Headings;
    double MaxWind = 0.0;
    double MaxLong = 0.0;
};

/** The model's constants in the table (VoxelOceanSeaState.ush, THE CONSTANTS): VOXEL_OCEAN_TABLE_CONSTANTS texels (rows 8 to 14 of the
 *  texture's column are the surface's per-frame shore frame, not the table's). */
struct FVoxelOceanTableConstants
{
    float Texels[VOXEL_OCEAN_TABLE_CONSTANTS * 4] = {};
};

/** The table of one interval (immutable once built). */
struct VOXELPLUGIN_API FVoxelOceanWeatherTable
{
    VoxelOceanCharts::FTable Table;
    TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> Source;
    FVoxelOceanWeatherTiming Timing;
    /** The interval's first key. */
    int64 Index = 0;
    /** Unique per table made (never 0). */
    uint64 Serial = 0;
    /** The greatest wind and long-wave speed (m/s) in the interval (keys k and k + 1). */
    double MaxWind = 0.0;
    double MaxLong = 0.0;

    double WeatherTime(double OceanTime) const { return Source.IsValid() ? Source->WeatherTime(OceanTime) : OceanTime; }
    /** The field's way from key k to k + 1 (0-1). */
    double Alpha(double OceanTime) const;
    /** The headings' way from key k - 1's to key k's (0-1). */
    double Fade(double OceanTime) const;
};

namespace VoxelOceanWeatherTable
{
    /** The key an ocean time falls in. */
    VOXELPLUGIN_API int64 KeyIndexAt(const IVoxelOceanEnvironmentSource& Source, const FVoxelOceanWeatherTiming& Timing, double OceanTime);

    /** Key Index of a source, for charts of ChartCells per face edge (any thread; parallel
     *  inside). */
    VOXELPLUGIN_API FVoxelOceanWeatherKey BuildKey(const IVoxelOceanEnvironmentSource& Source, int32 ChartCells,
        const FVoxelOceanWeatherTiming& Timing, int64 Index);

    /** The table of the interval from Current to Next, the headings turning from Previous's. */
    VOXELPLUGIN_API TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe> MakeTable(const VoxelOceanCharts::FTable& Charts,
        const FVoxelOceanWeatherKey& Previous, const FVoxelOceanWeatherKey& Current, const FVoxelOceanWeatherKey& Next,
        const FVoxelOceanTableConstants& Constants, const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& Source,
        const FVoxelOceanWeatherTiming& Timing);
}
