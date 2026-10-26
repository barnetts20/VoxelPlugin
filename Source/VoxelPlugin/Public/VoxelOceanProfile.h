// VoxelOceanProfile.h - Stage A12: the ocean's seeding (Design/AtmosphereOceanSide.md, A12). A
// profile asset holds what a planet's seed draws onto its authored ocean settings, in the
// atmosphere's draw language (CloudAtmosphere's AtmosphereHarnessTypes.h: distributions, links,
// palettes, locks, a sweep), applied by its harness (AtmosphereHarness::ApplyDrawSet) on its own
// stream. The planet keeps its OceanSettings as the base; the ocean gets the base with the draws.

#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "AtmosphereHarnessTypes.h"
#include "VoxelOceanSettings.h"
#include "VoxelOceanProfile.generated.h"

/** What an ocean's seed draws: paths are FVoxelOceanSettings members (WaterColor,
 *  WaveEqualizer.Gain2m, FoamWhitecap, AtmosphereMeanWind...). The palettes apply first, then the
 *  entries in order, so a second entry on a path refines the first (a linked height, then a
 *  jitter per knot). A link shared with the atmosphere's draws follows the same number. */
UCLASS(BlueprintType)
class VOXELPLUGIN_API UVoxelOceanProfile : public UDataAsset
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ocean")
    FAtmosphereDrawSet Draws;

    /** Adds the starter draws, leaving paths and palettes already present alone:
     *  - palettes: the water's colour (six waters: the default teal, deep blue, tropical, green,
     *    navy, murky olive; scaled 0.85 to 1.15, mutated 8%) and the foam's (beige, white, cool);
     *  - the water's turbidity (log-even, 0.015 to 0.08 per metre);
     *  - the wave equalizer: every knot linked on "OceanWaveHeight" between the spectrum's own
     *    heights (1) and Godot's (2.83), then each moved by up to 20% for the spectrum's shape;
     *  - the chop, the whitecap threshold (inversely linked to the heights: higher seas break more),
     *    the atmosphere's mean wind and swell share, and the rain's amount. */
    UFUNCTION(CallInEditor, Category = "Ocean")
    void AddStarterDraws();
};

namespace VoxelOceanSeeding
{
    /** Base with Profile's draws for Seed (none without a profile), Locks left as Base has them,
     *  SweepPath (a draw's path or a palette's name) held at SweepPosition. OutReport lists every
     *  setting written. */
    VOXELPLUGIN_API FVoxelOceanSettings Generate(int32 Seed, const FVoxelOceanSettings& Base, const UVoxelOceanProfile* Profile,
        const TArray<FString>& Locks, const FString& SweepPath, float SweepPosition, TArray<FAtmosphereDrawRecord>& OutReport);
}
