// VoxelOceanWavesFFT.h - The FFT wave model (Design/OceanWaves.md sections 2-4 and 7; stages
// O6b, O7): the CPU twin of the wave hook's FFT path (VoxelOceanWaves.ush) behind
// IVoxelOceanWaveModel, for the surface query on any thread, and the parameters the shaders
// take.
//
// WHAT IT HOLDS (immutable once built; a change of settings, or of the reach's wind past a step,
// builds a new model):
// - the slot layout (VoxelOceanSpectrum::MakeSlots: the long bands at three sea states, the short
//   bands at a reference wind, ShortReferenceWind) and its GPU table, which the surface hands to
//   the cascades (FVoxelOceanCascades);
// - each slot's height and slope statistics, and the short bands' amplitude against the wind
//   (their spectrum's height deviation at the knots of VoxelOceanSeaState.ush against the
//   reference's): the table's constants (GetTableConstants), with which the hook and Evaluate
//   weigh the slots by each point's sea state;
// - the reach: 5 sigma of the height at the reach's wind (Settings.WindSpeed: the weather's
//   greatest long-wave speed, stepped by ReachWind so it changes rarely);
// - the chart table's charts (VoxelOceanCharts::Build, the same the GPU reads);
// - the query cascades: every long slot (and band 2's with bQueryShortWaves) on 128^2 grids,
//   computed at 15 Hz of the ocean clock and interpolated in time, each slot only once a query
//   weighs it (calm seas never pay for the storm states), a few frames cached (the only mutable
//   state, behind a lock held only to look up and to insert).
//
// THE WEATHER comes with the frame's context (FVoxelOceanWaveContext::Weather, the table of the
// weather's interval, VoxelOceanWeatherTable.h): Evaluate reads a point's sea state and each
// chart's headings from the same texels the GPU reads, with the hook's own code
// (VoxelOceanSeaState.ush), and turns the headings over the interval's first seconds as the hook
// does. Without a table, the context's constant environment everywhere.
//
// EVALUATE does what the hook does with the same shared code (charts, blend weights, headings,
// detiling, the sum and the normal), from the query cascades instead of the textures, with chart
// coordinates in double: the drawn surface and the query agree to the GPU's float rounding and
// its bilinear lookups, and the short bands the query leaves out (decision 6).
//
// THE SHORE (stage O8): each site's depth and shore (FillSites: the density, the shore field)
// shoal its bands with the hook's code (VoxelOceanShoal.ush): amplitude, flattened orbits,
// breaking, the fade, and the warp along the shore direction. The reach allows for the shoaled
// waves (VOXEL_OCEAN_SHOAL_REACH_HEIGHT and _HORIZONTAL) while shoaling is on.
//
// THE SURF (stage O9a): the same sites add the hook's surf (VoxelOceanSurf.ush): the long bands
// handed over near the coast, the trains, the bores, the swash; on the ocean clock modulo
// VOXEL_OCEAN_SURF_WRAP (SurfClock, the shaders' W.SlopeVar.w), the settings as the shaders' rows
// (SurfRows). The surf adds no horizontal displacement: the query's inversion is unchanged.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanWaves.h"
#include "VoxelOceanSpectrum.h"
#include "VoxelOceanCharts.h"
#include "VoxelOceanWeatherTable.h"
#include "HAL/CriticalSection.h"


class VOXELPLUGIN_API FVoxelOceanWavesFFT : public IVoxelOceanWaveModel
{
public:
    /** The short bands' spectrum's wind (m/s); the weather scales them per point. */
    static constexpr double ShortReferenceWind = 10.0;
    /** The reach's wind steps up to whole ReachStep m/s, and back down once ReachSlack below. */
    static constexpr double ReachStep = 2.0;
    static constexpr double ReachSlack = 4.0;
    /** The reach's wind for the weather's greatest long-wave speed: Current's while it covers it
     *  without ReachSlack to spare (bHasCurrent), else MaxLong rounded up to a step. */
    static double ReachWind(double MaxLong, double Current, bool bHasCurrent);

    /** Radius: the sea sphere (cm), for the charts. InSettings.WindSpeed: the reach's wind. */
    FVoxelOceanWavesFFT(const FVoxelOceanFFTSettings& InSettings, double InRadius);

    /** The weather, and with shoaling (stage O8) the depth and the shore. */
    virtual EVoxelOceanWaveInputs RequiredInputs() const override
    {
        return Settings.ShoreBreaking > 0.0 ? (EVoxelOceanWaveInputs::Environment | EVoxelOceanWaveInputs::Depth | EVoxelOceanWaveInputs::Shore)
            : EVoxelOceanWaveInputs::Environment;
    }
    virtual void Evaluate(const FVoxelOceanWaveContext& Context, TConstArrayView<FVoxelOceanWaveSite> Sites,
        TArrayView<FVoxelOceanWaveResult> Out) const override;
    virtual double MaxHeight() const override { return MaxHeightCm; }
    virtual double MaxHeightAt(const FVoxelOceanWaveContext& Context, const FVector& PlanetPoint) const override;
    virtual double MaxHorizontal() const override { return MaxHorizontalCm; }
    virtual void GetShaderParams(const FVoxelOceanWaveContext& Context, const FVector& Reference,
        const FTransform& PlanetToWorld, FVoxelOceanWaveShaderParams& Out) const override;

    const FVoxelOceanFFTSettings& GetSettings() const { return Settings; }
    double GetRadius() const { return Radius; }
    /** The slots (VoxelOceanSpectrum::MakeSlots) and the table the GPU cascades run. */
    const std::vector<VoxelOceanSpectrum::FSlot>& GetSlots() const { return Slots; }
    TConstArrayView<VoxelOceanSpectrum::FSlotGPU> GetSlotTable() const { return SlotTable; }
    /** The charts (VoxelOceanCharts::Build), without weather. */
    const VoxelOceanCharts::FTable& GetCharts() const { return Charts; }
    /** The table's constants (VoxelOceanSeaState.ush, THE CONSTANTS). */
    const FVoxelOceanTableConstants& GetTableConstants() const { return Constants; }
    /** The sea's height standard deviation (m) for a wind (all bands, the swell aside). */
    double SigmaAt(double Wind) const;
    /** Stage W6i: the reach (cm) for a sea at this wind: 5 sigma, shoaled and the surf's at most. */
    double HeightBoundCm(double Wind) const;

private:
    /** One 15 Hz frame's query cascades, per query slot; a slot no query has weighed yet is
     *  null (immutable: a frame that needs more is replaced by a fuller copy). */
    struct FFrame
    {
        int64 Index = 0;
        TArray<TSharedPtr<const VoxelOceanSpectrum::FQueryCascade, ESPMode::ThreadSafe>> Cascades;
    };
    struct FQuerySlot
    {
        int32 Slot = 0;
        int32 Band = 0;
        /** The sea state (0-2) whose weight it takes; -1 for a short band (its scale). */
        int32 State = -1;
    };

    /** The query cascades at 15 Hz step Index, at least the slots in Needed (bit q: query slot
     *  q; cached; any thread). */
    TSharedPtr<const FFrame, ESPMode::ThreadSafe> GetFrame(int64 Index, uint32 Needed) const;
    /** A short band's (2, 3) amplitude scale at a wind (the constants' curve). */
    double ShortScale(int32 Band, double Wind) const;

    FVoxelOceanFFTSettings Settings;
    double Radius = 0.0;
    std::vector<VoxelOceanSpectrum::FSlot> Slots;
    TArray<VoxelOceanSpectrum::FSlotGPU> SlotTable;
    FVector4f Tiles = FVector4f(1.0f, 1.0f, 1.0f, 1.0f);
    /** Per slot: the height's and the slope's standard deviations (m, -). */
    double SlotSigma[VoxelOceanSpectrum::SlotCount] = {};
    double SlotSlope[VoxelOceanSpectrum::SlotCount] = {};
    FVoxelOceanTableConstants Constants;
    double MaxHeightCm = 0.0;
    double MaxHorizontalCm = 0.0;
    uint32 DetileSeed = 0;
    VoxelOceanCharts::FTable Charts;
    TArray<FQuerySlot> QuerySlots;
    TArray<TSharedPtr<const VoxelOceanSpectrum::FQueryCascade, ESPMode::ThreadSafe>> Prototypes;

    mutable FCriticalSection CacheLock;
    mutable TArray<TSharedPtr<const FFrame, ESPMode::ThreadSafe>> Cache;
};
