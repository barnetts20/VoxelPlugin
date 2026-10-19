// VoxelOceanWeatherKeys.h - The surface's weather keys (stage O7): which keys of the environment
// source the current interval needs (VoxelOceanWeatherTable.h), built in the background ahead
// of time, and the interval's table.
//
// Each tick (game thread) the ocean time gives the key k it falls in; the table of the interval
// needs keys k - 1 (the headings it turns from), k and k + 1 (the field it runs between). Keys
// already built are kept; a missing one is built at once (the first tick, a new source, or a
// background build that fell behind), and key k + 2 starts building on the thread pool so the
// next interval finds it ready. A new source, timing or model constants start over.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanWaves.h"
#include "VoxelOceanWeatherTable.h"
#include "Async/Future.h"

class FVoxelOceanWeatherKeys
{
public:
    /** Game thread. The table for an ocean time; true when it changed (upload it). */
    bool Update(double OceanTime, const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& Source,
        const FVoxelOceanWeatherTiming& Timing, const VoxelOceanCharts::FTable& Charts, const FVoxelOceanTableConstants& Constants);

    const TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe>& GetTable() const { return Table; }

    /** Game thread. The greatest long-wave speed of the keys an ocean time needs (the first
     *  model's reach; the keys stay for the table). */
    double MaxLongAt(double OceanTime, const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& Source,
        const FVoxelOceanWeatherTiming& Timing, int32 ChartCells);

    /** Forgets everything (a background build still running is left to finish unread). */
    void Reset();

private:
    using FKeyPtr = TSharedPtr<const FVoxelOceanWeatherKey, ESPMode::ThreadSafe>;

    FKeyPtr KeyAt(int64 Index);
    /** A new source, timing or chart count starts the keys over (true then). */
    bool Configure(const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& InSource,
        const FVoxelOceanWeatherTiming& InTiming, int32 InChartCells);

    TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> Source;
    FVoxelOceanWeatherTiming Timing;
    int32 ChartCells = 0;
    FVoxelOceanTableConstants Constants;
    TMap<int64, FKeyPtr> Keys;
    /** The background build: its key and its result. */
    int64 PendingIndex = 0;
    TFuture<FKeyPtr> Pending;
    TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe> Table;
};
