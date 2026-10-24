#include "VoxelOceanWeatherKeys.h"
#include "Async/Async.h"

void FVoxelOceanWeatherKeys::Reset()
{
    Source.Reset();
    Keys.Empty();
    Pending = TFuture<FKeyPtr>();
    Table.Reset();
    ChartCells = 0;
}

FVoxelOceanWeatherKeys::FKeyPtr FVoxelOceanWeatherKeys::KeyAt(int64 Index)
{
    if (const FKeyPtr* Found = Keys.Find(Index)) return *Found;
    // The background build's, once it's done (waiting for it if it's the one we need now).
    if (Pending.IsValid() && PendingIndex == Index)
    {
        FKeyPtr Key = Pending.Get();
        Pending = TFuture<FKeyPtr>();
        if (Key.IsValid())
        {
            Keys.Add(Index, Key);
            return Key;
        }
    }
    const FKeyPtr Key = MakeShared<FVoxelOceanWeatherKey, ESPMode::ThreadSafe>(VoxelOceanWeatherTable::BuildKey(*Source, ChartCells, Timing, Index));
    Keys.Add(Index, Key);
    return Key;
}

bool FVoxelOceanWeatherKeys::Configure(const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& InSource,
    const FVoxelOceanWeatherTiming& InTiming, int32 InChartCells)
{
    // A new source, timing or chart count: every key again.
    if (InSource == Source && InTiming == Timing && InChartCells == ChartCells) return false;
    Keys.Empty();
    // A build still running for the old source finishes unread.
    Pending = TFuture<FKeyPtr>();
    Source = InSource;
    Timing = InTiming;
    ChartCells = InChartCells;
    return true;
}

double FVoxelOceanWeatherKeys::MaxLongAt(double OceanTime, const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& InSource,
    const FVoxelOceanWeatherTiming& InTiming, int32 InChartCells)
{
    if (!InSource.IsValid()) return 0.0;
    Configure(InSource, InTiming, InChartCells);
    const int64 K = VoxelOceanWeatherTable::KeyIndexAt(*Source, Timing, OceanTime);
    return FMath::Max(KeyAt(K)->MaxLong, KeyAt(K + 1)->MaxLong);
}

bool FVoxelOceanWeatherKeys::Update(double OceanTime, const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& InSource,
    const FVoxelOceanWeatherTiming& InTiming, const VoxelOceanCharts::FTable& Charts, const FVoxelOceanTableConstants& InConstants)
{
    if (!InSource.IsValid() || !Charts.IsValid()) return false;

    // New charts or constants: the keys hold, the table doesn't.
    bool bRebuild = Configure(InSource, InTiming, Charts.N);
    if (FMemory::Memcmp(InConstants.Texels, Constants.Texels, sizeof(Constants.Texels)) != 0
        || FMemory::Memcmp(InConstants.Resolved, Constants.Resolved, sizeof(Constants.Resolved)) != 0)
    {
        Constants = InConstants;
        bRebuild = true;
    }

    const int64 K = VoxelOceanWeatherTable::KeyIndexAt(*Source, Timing, OceanTime);
    if (bRebuild || !Table.IsValid() || Table->Index != K || Table->Table.N != Charts.N)
    {
        const FKeyPtr Previous = KeyAt(K - 1);
        const FKeyPtr Current = KeyAt(K);
        const FKeyPtr Next = KeyAt(K + 1);
        Table = VoxelOceanWeatherTable::MakeTable(Charts, *Previous, *Current, *Next, Constants, Source, Timing);
        // Keys outside the interval and the one ahead go (time may jump either way).
        for (auto It = Keys.CreateIterator(); It; ++It)
        {
            if (It.Key() < K - 1 || It.Key() > K + 2) It.RemoveCurrent();
        }
        bRebuild = true;
    }
    // A background build no longer of use (time jumped past it) is left to finish unread.
    if (Pending.IsValid() && (PendingIndex < K - 1 || PendingIndex > K + 2))
        Pending = TFuture<FKeyPtr>();

    // The next interval's new key, in the background.
    const int64 Ahead = K + 2;
    if (!Keys.Contains(Ahead) && !(Pending.IsValid() && PendingIndex == Ahead))
    {
        if (Pending.IsValid() && Pending.IsReady())
        {
            // An older background key nobody asked for: keep it if it's still ahead.
            FKeyPtr Done = Pending.Get();
            if (Done.IsValid() && PendingIndex >= K - 1) Keys.Add(PendingIndex, Done);
            Pending = TFuture<FKeyPtr>();
        }
        if (!Pending.IsValid())
        {
            PendingIndex = Ahead;
            TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> BuildSource = Source;
            const FVoxelOceanWeatherTiming BuildTiming = Timing;
            const int32 Cells = ChartCells;
            Pending = Async(EAsyncExecution::ThreadPool, [BuildSource, BuildTiming, Cells, Ahead]() -> FKeyPtr
                {
                    return MakeShared<FVoxelOceanWeatherKey, ESPMode::ThreadSafe>(VoxelOceanWeatherTable::BuildKey(*BuildSource, Cells, BuildTiming, Ahead));
                });
        }
    }
    return bRebuild;
}
