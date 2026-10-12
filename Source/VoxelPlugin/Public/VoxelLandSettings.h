// VoxelLandSettings.h - The planet's surface generator switch and the terrestrial generator's
// settings (Design/LandGen.md section 9). Planet-relative values (plate count, land fraction)
// keep their meaning at any radius; landforms are absolute (Earth widths and heights, scaled
// by the two landform factors), so a small planet has fewer, not smaller, mountain ranges.

#pragma once

#include "CoreMinimal.h"
#include "VoxelLandSettings.generated.h"

/** Which generator shapes a planet's surface. */
UENUM(BlueprintType)
enum class EVoxelSurfaceGenerator : uint8
{
    /** The noise surface (SurfaceSettings): continents, ridged mountains and detail as fBm. */
    Noise,
    /** Plates and boundary profiles baked on a cube-sphere grid at load (TerrestrialSettings),
     *  with per-sample detail. Heights are metres about sea level, the planet's radius. */
    Terrestrial
};

/** The terrestrial generator. Changing a Bake setting bakes the planet again (cached on disk
 *  by a hash of them); the Sample settings change only the per-sample layers. */
USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelTerrestrialSettings
{
    GENERATED_BODY()

    // --- Bake: planet-relative ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Plates")
    int32 Seed = 7;

    /** Tectonic plates on the sphere. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Plates", meta = (ClampMin = "2", ClampMax = "64", UIMin = "4", UIMax = "24"))
    int32 PlateCount = 10;

    /** Fraction of the sphere that is land (the continental crust above its coastline; shelves
     *  lie below it). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Plates", meta = (ClampMin = "0.02", ClampMax = "0.98"))
    double ContinentalFraction = 0.3;

    // --- Bake: landforms (absolute) ---

    /** Widths of boundary landforms (ranges, trenches, rifts, ridges), times Earth's. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (ClampMin = "0.1", ClampMax = "4.0", UIMin = "0.25", UIMax = "2.0"))
    double LandformWidthScale = 1.0;

    /** Heights of boundary landforms, times Earth's. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (ClampMin = "0.0", ClampMax = "4.0", UIMin = "0.25", UIMax = "2.0"))
    double RangeRelief = 1.0;

    /** Height of the plains at the coast and deep inland (m). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (UIMin = "0", UIMax = "500"))
    double CoastHeight = 50.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (UIMin = "0", UIMax = "3000"))
    double InteriorHeight = 950.0;

    /** Depth of the continental shelf edge and of the abyssal plain (m, negative). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (UIMin = "-1000", UIMax = "0"))
    double ShelfDepth = -140.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (UIMin = "-8000", UIMax = "-1000"))
    double AbyssDepth = -4200.0;

    /** Boundary uplift above which continental crust rises into the erosion solve (m); below
     *  it the land keeps its profile as plains. The solve fades in over MountainRampWidth
     *  around it, so ranges rise from the plains without a scarp. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (ClampMin = "0.0", UIMax = "2000"))
    double PlainsUpliftThreshold = 400.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Landforms", meta = (ClampMin = "1.0", UIMax = "2000"))
    double MountainRampWidth = 300.0;

    // --- Bake: erosion solve and lakes ---

    /** The stream-power solve stops once the mean height change per step is below this
     *  fraction of the mean uplift (after at least 10 steps), or after SolveMaxSteps. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.001", ClampMax = "0.5"))
    double SolveResidual = 0.02;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "10", ClampMax = "1000"))
    int32 SolveMaxSteps = 150;

    /** Depressions the final routing keeps as lakes: at least this deep (m) and this large
     *  (km^2). Smaller ones are filled. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", UIMax = "200"))
    double MinLakeDepth = 10.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", UIMax = "1000"))
    double MinLakeArea = 20.0;

    /** How far each bake cell's node in the drainage graph is moved, at random (cells): water
     *  no longer runs along the grid's 8 directions, so channels wander and branch instead of
     *  running in straight, parallel grooves. 0: the plain grid. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "0.45"))
    double RoutingJitter = 0.4;

    /** How freely water picks among the lower neighbours instead of the steepest (each
     *  neighbour's slope times a fixed random factor between 2^-x and 2^x). On even slopes the
     *  steepest descent keeps every channel parallel to its neighbours; with a random choice
     *  they wander, meet and merge into branching networks. 0: steepest descent. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "3.0"))
    double RoutingRandomness = 1.0;

    /** The drainage field (the water views' rivers, the gullies' steering) spreads water over
     *  every lower neighbour on hillslopes, cells draining fewer than this many cells, so a
     *  slope carries an even sheet instead of a comb of parallel lines; channels keep a single
     *  path. The terrain itself is unaffected. 0: single paths everywhere. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "1024.0"))
    double DrainageSpreadCells = 16.0;

    /** Uplift along the mountain belts varies by up to this fraction (a noise field): massifs
     *  and saddles instead of a ridge of even height. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "1.5"))
    double UpliftVariation = 0.8;

    /** Rock hardness: erodibility varies by a factor of exp(+-this) (a noise field at half the
     *  variation wavelength). Rivers bend around hard rock, which holds the higher peaks. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    double ErodibilityVariation = 0.7;

    /** The longest wavelength of the uplift variation (km); erodibility uses half, the initial
     *  roughness a quarter. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "10.0", UIMax = "500"))
    double VariationWavelength = 90.0;

    /** The initial surface's roughness (m): how the drainage network starts to branch. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", UIMax = "500"))
    double InitialRoughness = 150.0;

    /** The plains take part in the erosion with this much uplift of their own (m): drained,
     *  branching valleys on the plains, and closed basins drain through cut outlets. 0: the
     *  plains keep the plate model's smooth surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", UIMax = "500"))
    double PlainsRelief = 100.0;

    /** Rolling ground (m, the largest of its scales) added to the land's base surface before
     *  the erosion, at a third of VariationWavelength and below. The plate model's surface is a
     *  smooth ramp between belts and coasts, and rivers on a smooth ramp run straight down it
     *  side by side; the hollows make them gather into branching networks. Raises the number of
     *  small lakes somewhat. 0: the plate model's ramps. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", UIMax = "400"))
    double PlainsUndulation = 60.0;

    /** How far a small river may cut below the plate model's surface (m); larger ones may cut
     *  deeper with the square root of their drainage, through basins' sills (never below the
     *  sea). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", UIMax = "1000"))
    double RiverIncision = 150.0;

    /** The highest summits relative to the profile's highest uplift: the eroded relief keeps
     *  its shape up to the profile's 90th percentile, then follows a power fitted so the
     *  highest 0.1% land here. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "1.0", ClampMax = "2.0"))
    double SummitHeightScale = 1.2;

    // --- Bake: grid ---

    /** Cells per cube face edge: 1024 (1.5 km on a 1,000 km planet, 9.8 km on Earth), or 512 or
     *  2048 for development (memory and bake time scale with its square: the GPU solve needs
     *  up to about 95 bytes a cell of GPU memory while it runs, 600 MB at 1024, 2.4 GB at
     *  2048). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Bake", meta = (ClampMin = "64", ClampMax = "2048"))
    int32 BakeResolution = 1024;

    // --- Sample: per-sample layers (no rebake) ---

    /** Sampling warp of the bake, in bake cells: hides the grid's straight lines. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "4.0"))
    double BakeWarp = 1.5;

    /** Slope-damped fBm detail on top of the bake: amplitude (m), its longest wavelength (m),
     *  and how strongly accumulated slope suppresses finer octaves. A quarter as strong under
     *  the sea, and scaled by GullyDetailScale on land while there are gullies. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "1000"))
    double DetailAmplitude = 250.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "100.0", UIMin = "1000", UIMax = "100000"))
    double DetailWavelength = 20000.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double DetailSlopeDamping = 1.0;

    /** Gullies cut down the slopes below the bake's resolution (a stateless erosion filter,
     *  from two bake cells down to GullyFinestWavelength): depth per unit of wavelength and
     *  slope. 0 turns them off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMax = "0.5"))
    double GullyStrength = 0.22;

    /** The finest gully wavelength (m); finer octaves are left out (and coarse levels leave out
     *  the ones they can't show). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "5.0", UIMin = "10", UIMax = "500"))
    double GullyFinestWavelength = 30.0;

    /** How much shallower gullies are on ridges (a bake cell's own drainage) than on valley
     *  sides (16 cells' drainage and more): 0 the same everywhere, 1 none on ridges. Eroded
     *  bakes only. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double GullyFlowGain = 0.5;

    /** The detail layer's scale on land while there are gullies (GullyStrength above 0): they
     *  replace much of the isotropic detail. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double GullyDetailScale = 0.5;

    /** Hash of the settings that change the bake (with the radius and the generator version,
     *  the cache key). */
    uint32 GetBakeHash() const;

    /** Hash of every field, for change detection. */
    uint32 GetSettingsHash() const;
};
