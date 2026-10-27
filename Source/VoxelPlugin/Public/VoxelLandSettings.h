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

    /** How much the landform field (flat plains to hill country, Design/SurfaceLook.md T2)
     *  scales the erosion's plains relief and rolling ground: at 1, 0.3 times on flat plains,
     *  2.05 times in hill country (about the same on average); 0: the same everywhere. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double LandformErosion = 1.0;

    /** Drainage density (T3g): a cell's river cuts only once it drains this many bake cells
     *  (ramped in from half), so channels start at heads instead of on every cell, and the land
     *  above them is hillslope, held at a threshold slope (HillslopeSteepness). Larger: fewer,
     *  more widely spaced valleys and smoother ground between them. 0: every cell cuts (the
     *  earlier solve, and no variation). In bake cells, as the bake can't resolve real channel
     *  heads: the same value gives the same look at any BakeResolution, at the bake's scale. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", UIMax = "32"))
    double ChannelHeadCells = 8.0;

    /** How much the channel heads' area varies over the planet: by a factor of 2^(+-this) (a
     *  noise field). Regions of sparse, long hillslopes beside finely dissected ones, instead
     *  of an even valley density everywhere. 0: the same everywhere. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "3.0"))
    double ChannelAreaVariation = 1.5;

    /** The channel heads' variation's longest wavelength, as a multiple of VariationWavelength
     *  (the massifs' scale). Regions need to span several valleys to read as regions: at half
     *  the massifs' scale the variation came and went within a valley or two. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.05", ClampMax = "4.0"))
    double ChannelAreaVariationWavelength = 2.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "1", ClampMax = "6"))
    int32 ChannelAreaVariationOctaves = 3;

    /** The hillslopes' threshold above the channel heads: the steepest a cell may stand above
     *  the cell it drains to, as a multiple of a river's steady slope at the local head area
     *  (so it follows the uplift and the rock, as the rivers do, and sparse heads give longer,
     *  gentler hillslopes). Larger: sharper, higher ridges; 1: hillslopes as steep as their
     *  channel heads. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.25", ClampMax = "8.0"))
    double HillslopeSteepness = 1.5;

    /** How far the drainage density sets the landform's character (T3h): where channel heads
     *  are sparse the land leans smooth (the peaks' soft-rock set, rolling hills), where they
     *  are dense rough (the hard-rock set, rough-topped hills), so the regional detail agrees
     *  with the bake's valleys. 0: the plate model's character alone. Needs the variation
     *  (ChannelHeadCells and ChannelAreaVariation above 0). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Solve", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double ChannelCharacterInfluence = 0.75;

    // --- Bake: grid ---

    /** Cells per cube face edge: 1024 (1.5 km on a 1,000 km planet, 9.8 km on Earth), or 512 or
     *  2048 for development (memory and bake time scale with its square: the GPU solve needs
     *  up to about 95 bytes a cell of GPU memory while it runs, 600 MB at 1024, 2.4 GB at
     *  2048). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Bake", meta = (ClampMin = "64", ClampMax = "2048"))
    int32 BakeResolution = 1024;

    // --- Sample: per-sample layers (no rebake) ---

    /** Sampling warp of the bake, in bake cells: hides the grid's straight lines. Over about 1
     *  the warp folds (1.5 folded 2% of the land and squeezed 28% to under half its width), and
     *  what's carved in the bake's frame, the rivers' valleys and channels, squeezes with it into
     *  slots and cliffs. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "4.0"))
    double BakeWarp = 1.0;

    /** The detail layer on top of the bake (N1: the planet's detail set, DetailNoiseSet): its
     *  eroded (slope-damped) fbm channel at up to four scales of four octaves, from
     *  DetailWavelength (m) down to the finest cell. DetailAmplitude (m) as before: the coarsest
     *  scale's standard deviation is 0.4 of it. A quarter as strong under the sea, and scaled by
     *  GullyDetailScale on land while there are gullies. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "1000"))
    double DetailAmplitude = 250.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "100.0", UIMin = "1000", UIMax = "100000"))
    double DetailWavelength = 20000.0;

    /** Unused since N1: the detail set's eroded channel carries its own damping (each variant
     *  its own: detail_a k 1, detail_b 0.5, detail_c 2). Kept so saved planets load. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double DetailSlopeDamping = 1.0;

    /** The detail set (N1): a file name in the plugin's NoiseSets folder without .vxnt (e.g.
     *  detail_b), empty to pick one of the folder's detail* sets by the seed, "none" for no detail
     *  at all (the detail layer and the peaks', hills' and strata's detail). Without any (or if it
     *  can't be read: the log says why) the plugin's own generated set. Its G (ridged) and R
     *  (fbm) are the layers' hard and soft rock detail, its A (eroded fbm) the detail layer. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample")
    FString DetailNoiseSet;

    /** Gullies cut down the slopes below the bake's resolution (a stateless erosion filter,
     *  from two bake cells down to GullyFinestWavelength): depth per unit of wavelength and
     *  slope. 0 turns them off: the default since N1b (the eroded sets and the detail sets carry
     *  the hillsides now; 0.22 was the default before). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "1.0", UIMax = "0.5"))
    double GullyStrength = 0.0;

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

    /** Hills on the bake (Design/SurfaceLook.md T2): the procedural hills' height (m) where the
     *  landform field makes hill country; flat plains get none, rolling plains some. Rounded
     *  rises between a network of valleys, smooth on rolling ground, rough-topped in hill
     *  country; they leave the bake's river corridors and the coasts flat. With a hill set
     *  (HillNoiseSet, T4a) the set's own relief sets their height instead (HillHeightScale).
     *  0 turns every hill off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "600"))
    double HillRelief = 150.0;

    /** The hills' longest wavelength (m): a hill and its valley. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "200.0", UIMin = "1000", UIMax = "20000"))
    double HillWavelength = 4000.0;

    /** The hills' finest wavelength (m); finer octaves are left out (and coarse levels leave
     *  out the ones they can't show). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "5.0", UIMin = "10", UIMax = "1000"))
    double HillFinestWavelength = 25.0;

    /** The hills' eroded noise set (Design/NonMountainLand.md, T4a): a file name in the plugin's
     *  NoiseSets folder without .vxnt (e.g. hills_a), empty to pick one of the folder's hills*
     *  sets by the seed, or "none". Without one (or if it can't be read: the log says why) the
     *  procedural hills above. Dissected hill country where the character is rough (the set's
     *  R), rounded where it is smooth (G); its valleys drain into the bake's. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample")
    FString HillNoiseSet;

    /** The hill set's height over its own relief at its placement (HillSpacing), at full
     *  hilliness: 1 keeps the simulated slopes. Never more than half HillSpacing (a bound for
     *  the planet's height range). 0 turns the set off (the procedural hills then). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double HillHeightScale = 1.0;

    /** The distance between the hill set's main valleys' groups (m): its spacing is placed at
     *  this. Its relief, and so the hills' height, scales with it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "500.0", UIMin = "1000", UIMax = "10000"))
    double HillSpacing = 2500.0;

    /** Broad swells on rolling plains (m): the hill set's rounded channel again at
     *  HillSwellScale times the spacing, this high at rolling hilliness, a quarter of it on the
     *  flats, none in hill country. 0 turns them off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "100"))
    double HillSwellHeight = 20.0;

    /** The swells' spacing over the hills' (HillSpacing): how much broader they are. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "1.5", ClampMax = "8.0"))
    double HillSwellScale = 3.0;

    /** The hill set's detail (as PeakDetailStrength: the detail set at the hills' own
     *  placement and amplitude). 0 turns it off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double HillDetailStrength = 0.5;

    /** Its coarsest wavelength (m); kept within a quarter of HillSpacing in code. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "16.0", UIMin = "50", UIMax = "2000"))
    double HillDetailWavelength = 300.0;

    /** The foothills (T4b): how much the hill set's hills grow toward the ranges before the peaks
     *  take over: their height is x (1 - P)(1 + this x 4P(1 - P)), P the peaks' fade-in, so at 1
     *  they keep full height where the peaks are half in (relief about 1000 m at the defaults)
     *  and peak at about 1.3x where the peaks are a fifth in. 0: the hills only fade (a
     *  smoother band along range fronts). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    double HillFoothillGrowth = 1.0;

    /** The strata set (Design/NonMountainLand.md, T4c): stepped sedimentary land (mesas, buttes,
     *  benches and cliff bands; badlands where the drainage is dense) that replaces the hills in
     *  the bake's sedimentary provinces (platforms and basins away from the ranges, from the
     *  plate model). A file name in the plugin's NoiseSets folder without .vxnt (e.g. strata_a),
     *  empty to pick one of the folder's strata* sets by the seed, or "none". Rides the hill
     *  set's layer: none without a hill set. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample")
    FString StrataNoiseSet;

    /** The strata set's height over its own relief at its placement (StrataSpacing), in hill
     *  country (StrataFlatShare of it on the flats): 1 keeps the simulated slopes. Never more than
     *  half StrataSpacing. 0 turns the strata off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double StrataHeightScale = 1.0;

    /** The distance between the strata set's main canyons' groups (m): its spacing is placed at
     *  this. Its relief, and so the mesas' height, scales with it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "500.0", UIMin = "1000", UIMax = "12000"))
    double StrataSpacing = 4000.0;

    /** The share of the land in strata country: the province's threshold is picked so this much
     *  of the land lies above it (the sedimentary provinces first). 0 turns the strata off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double StrataCoverage = 0.3;

    /** The switch from hills to strata: the province's half-width (0..1 units) over which they
     *  blend. Narrow: a brief mix (the province changes over hundreds of kilometres, so 0.05 is
     *  some tens of kilometres). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.005", ClampMax = "0.5"))
    double StrataSwitchWidth = 0.05;

    /** The strata's height on the flats (hilliness 0) over hill country's: plateaus and low
     *  benches on the plains, full mesas in hill country. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double StrataFlatShare = 0.35;

    /** The strata set's detail (as HillDetailStrength): texture on the benches and plateau tops.
     *  Much more rounds the cliff bands off. 0 turns it off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double StrataDetailStrength = 0.5;

    /** Its coarsest wavelength (m); kept within a quarter of StrataSpacing in code. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "16.0", UIMin = "50", UIMax = "2000"))
    double StrataDetailWavelength = 400.0;

    /** The dune set (Design/LandClimate.md, T5c): sand seas where the climate snapshot makes
     *  desert (Holdridge's aridity at the ground's height), on low ground only, turned to the
     *  climate's sand drift: sharp crests under one steady wind, rounded under variable winds,
     *  crossing networks under many. A file name in the plugin's NoiseSets folder without .vxnt
     *  (e.g. dunes_a), empty to pick one of the folder's dunes* sets by the seed, or "none".
     *  Rides the hill set's layer (none without a hill set) and needs the land climate
     *  (APlanetActor::LandClimate). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes")
    FString DuneNoiseSet;

    /** The dune set's height over its own relief at its placement (DuneSpacing): 1 keeps the
     *  simulated slopes (slip faces at the angle of repose). 0 turns the dunes off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double DuneHeightScale = 1.0;

    /** The distance between the dunes' crests (m): the set's wavelength is placed at this. Their
     *  height scales with it (about a fifth of it at the highest crests, a sixteenth on average,
     *  with the default sets). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes", meta = (ClampMin = "100.0", UIMin = "200", UIMax = "3000"))
    double DuneSpacing = 600.0;

    /** The share of the desert that is sand sea (ergs); the rest is rock and gravel plain. Earth's
     *  deserts are about a fifth to a third sand. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double DuneErgCoverage = 0.35;

    /** The sand seas' size (km): the scale of the noise that places them. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes", meta = (ClampMin = "10.0", UIMin = "50", UIMax = "2000"))
    double DuneErgSize = 400.0;

    /** The local relief (m, the bake's) above which there are no dunes; they are full under 0.4 of
     *  it. Dunes fill basins and plains; ranges keep their rock. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes", meta = (ClampMin = "10.0", UIMax = "2000.0"))
    double DuneMaxRelief = 400.0;

    /** The sand's detail set (T5c): the megaripples on the dunes, across the wind as the crests are.
     *  A file name (e.g. sanddetail_a), empty to pick one of the folder's sanddetail* sets by the
     *  seed, or "none". */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes")
    FString SandDetailNoiseSet;

    /** The megaripples' wavelength (m); kept within 2 m .. a quarter of DuneSpacing in code. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes", meta = (ClampMin = "2.0", UIMin = "4", UIMax = "100"))
    double SandDetailWavelength = 24.0;

    /** The sand detail's height over its set's at its placement: 1 keeps its slopes. 0 turns it off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Dunes", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double SandDetailStrength = 1.0;

    /** The glacial set (Design/LandClimate.md, T5d): ice-carved mountains (troughs, cirques, horns,
     *  arêtes, fjord beds; R the bedrock, G the ice surface over it) in the peaks' place where the
     *  climate snapshot puts glaciers now or at the last glacial maximum (GlacialEraCooling): bare
     *  carved bedrock where the ice has gone, the ice surface where it stays. A file name in the
     *  plugin's NoiseSets folder without .vxnt (e.g. glacial_a), empty to pick one of the folder's
     *  glacial* sets by the seed, or "none". Rides the peaks (none without them) and needs the
     *  land climate (APlanetActor::LandClimate). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Glacial")
    FString GlacialNoiseSet;

    /** The glacial set's height over its own relief at its placement (GlacialSpacing), as
     *  PeakHeightScale: 1 keeps the simulated slopes, never more than the bake's local relief.
     *  0 turns the glacial layer off (the peaks everywhere). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Glacial", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double GlacialHeightScale = 1.0;

    /** The distance between the glacial set's main troughs (m): its spacing is placed at this. Its
     *  relief scales with it (the walls' height is about half of it with glacial_a, capped by the
     *  bake's relief). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Glacial", meta = (ClampMin = "500.0", UIMin = "2000", UIMax = "20000"))
    double GlacialSpacing = 6000.0;

    /** How much colder (C) the last glacial maximum was: the land its glaciers reached keeps their
     *  carving (bare troughs and cirques below today's ice). Earth's was about 6 C. 0: only today's
     *  glaciers. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Glacial", meta = (ClampMin = "0.0", UIMax = "15.0"))
    double GlacialEraCooling = 6.0;

    /** The ice sheets (Design/LandClimate.md, T5d2): where the climate snapshot ices the land
     *  today, an ice sheet over it, thickening into the ice by the plastic profile (Nye: 470 m 10 km
     *  in, 1.5 km at 100 km), at most this thick (m). The land shows through where it stands higher
     *  (nunataks). Needs the land climate. 0 turns them off. The planet's height range takes this
     *  much above its highest ground if its peaks don't already. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Ice", meta = (ClampMin = "0.0", UIMax = "5000.0"))
    double IceSheetMaxThickness = 2000.0;

    /** The ice sheets' swells: the icesheet set's G (its ice surface: smooth swells over a buried
     *  bed). A file name in the plugin's NoiseSets folder without .vxnt (e.g. icesheet_a), empty to
     *  pick one of the folder's icesheet* sets by the seed, or "none" (a smooth sheet). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Ice")
    FString IceSheetNoiseSet;

    /** The swells' height (m): the set's G from its lowest to its highest. 0: no swells. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Ice", meta = (ClampMin = "0.0", UIMax = "500.0"))
    double IceSheetSwellHeight = 60.0;

    /** The swells' spacing (m): the set's tile over its cells is placed at this. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Ice", meta = (ClampMin = "1000.0", UIMin = "5000", UIMax = "100000"))
    double IceSheetSwellSpacing = 30000.0;

    /** Stage T5d3: the polar surfaces' set (VoxelLandPolar.ush): a file name in the plugin's
     *  NoiseSets folder without .vxnt (polar_*: R crevasses, G sastrugi, B ice-wedge polygons, A sea
     *  ice), or empty to pick one by the seed, or "none". Laid where the climate snapshot makes ice
     *  or frozen ground (needs the land climate). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar")
    FString PolarNoiseSet;

    /** The crevasses' scale (m): one of the set's R cells (polar_a: crevasses about half of it
     *  apart, a sixth of it wide, a third of it deep). On glaciers on steep ground and near the ice
     *  sheets' margins, across the ice's flow. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "10.0", UIMin = "20", UIMax = "200"))
    double CrevasseSpacing = 60.0;

    /** The crevasses' height over the set's own at that scale (1: the simulated walls). 0: none. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double CrevasseHeightScale = 1.0;

    /** The sastrugi's scale (m): one of the set's G cells (polar_a: ridges 1 to 3 of it long, up to
     *  a third of it high). On ice and snow that stay well below freezing, along the wind (the
     *  snapshot's, and the ice sheets' katabatic winds). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "1.0", UIMin = "2", UIMax = "20"))
    double SastrugiSpacing = 4.0;

    /** The sastrugi's height over the set's own (1: the simulated slopes). 0: none. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "0.0", UIMax = "3.0"))
    double SastrugiHeightScale = 1.0;

    /** The ice-wedge polygons' size (m): one of the set's B cells, about a polygon across. On flat
     *  bare ground over permafrost. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "4.0", UIMin = "8", UIMax = "60"))
    double PolygonSpacing = 20.0;

    /** The polygons' height over the set's own (1: rims and troughs about a metre at 20 m). 0: none. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "0.0", UIMax = "3.0"))
    double PolygonHeightScale = 1.0;

    /** The year's mean temperature (C) at which the polygons are half in, over +-2 C: continuous
     *  permafrost with active ice wedges (Pewe 1966: about -6 C to -8 C or colder). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (UIMin = "-15.0", UIMax = "0.0"))
    double PermafrostTemperature = -6.0;

    /** The snow dunes' spacing (m): crest to crest, the dune set's (its G, rounded crests) on the
     *  flat snow plains (the ice sheets' thick interiors, flat cold glacier ice), across the wind.
     *  Needs the pack's dune set (DuneNoiseSet, the hill set). The polar plateaus' snow dunes and
     *  megadunes: a few hundred metres to a few kilometres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "50.0", UIMin = "200", UIMax = "5000"))
    double SnowDuneSpacing = 400.0;

    /** The snow dunes' height (m), trough to the highest crest: low grade (the megadunes: 2 to 8
     *  m; slopes of a few percent at the default spacing). 0: none. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Polar", meta = (ClampMin = "0.0", UIMax = "40.0"))
    double SnowDuneHeight = 8.0;

    /** T5e, the polar ice shelf (Design/LandClimate.md): where the climate snapshot's year's mean
     *  at sea level is below IceShelfTemperature, the polar sea (and any low land there) becomes a
     *  shelf of ice: a solid plateau in the base elevation, like a plate placed by the cold, its
     *  front wandering into capes and bays (IceShelfFrontNoise), its surface gently rolling under
     *  the snow dunes, any ground higher than it standing through. Placed once per capture. The
     *  front's height above the sea (m; Antarctica's shelf fronts stand 15 to 50 m). 0: none.
     *  Needs the land climate. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|IceShelf", meta = (ClampMin = "0.0", UIMax = "150.0"))
    double IceShelfHeight = 40.0;

    /** The year's mean at sea level (C) at the shelf's front: colder is shelf. -15 C is about 72
     *  degrees of latitude on an Earth-like planet. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|IceShelf", meta = (UIMin = "-30.0", UIMax = "-5.0"))
    double IceShelfTemperature = -15.0;

    /** How far the front wanders from that line, at most (C of the year's mean: capes and bays
     *  from thousands of kilometres down to a few bake cells; a degree is about 100 km on an
     *  Earth-like planet's polar latitudes). 0: the line itself. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|IceShelf", meta = (ClampMin = "0.0", UIMax = "10.0"))
    double IceShelfFrontNoise = 5.0;

    /** The height the shelf gains inland (m), reached 3 C colder than the front. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|IceShelf", meta = (ClampMin = "0.0", UIMax = "200.0"))
    double IceShelfRise = 40.0;

    /** The gentle roll of its surface (m, either way; tens to a hundred kilometres across). The
     *  snow dunes and sastrugi ride on it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|IceShelf", meta = (ClampMin = "0.0", UIMax = "50.0"))
    double IceShelfRoll = 10.0;

    /** The depth (m) the sea floor rises to in front of and under the shelf (over about 1.5 C
     *  outside its front), so the front stands this far into the water, not over the abyss. 0:
     *  the sea floor as it is. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|IceShelf", meta = (ClampMin = "0.0", UIMax = "2000.0"))
    double IceShelfDraft = 300.0;

    /** Rivers (Design/RiversLakes.md, R1): the smallest drainage area (km^2) that carries a
     *  river. The network is the bake's own drainage, made into lines when the bake is ready
     *  (no rebake to change it). 0 turns the rivers off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.0", UIMin = "20", UIMax = "5000"))
    double RiverMinArea = 200.0;

    /** The channels' width over the default: 2.35 m x (drainage, km^2)^0.4, 20 m at 200 km^2,
     *  235 m at 100,000 km^2, 1 km at 3 million (R3; the floodplains and valleys don't change
     *  with it). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.1", UIMax = "4.0"))
    double RiverWidthScale = 1.0;

    /** The valleys' half-width over the default: 400 m + 15 belts (the belt is 2.5 m x
     *  sqrt(drainage, km^2): 35 m at 200 km^2, 790 m at 100,000 km^2). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.1", UIMax = "4.0"))
    double ValleyWidthScale = 1.0;

    /** Carve the rivers into the surface (Design/RiversLakes.md, R2): every layer is pulled down
     *  into each river's valley and a channel is cut below its floor. Off: the rivers are only
     *  data (the debug views, the water query). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers")
    bool bCarveRivers = true;

    /** The channels' depth over the default: 1.05 m x (drainage, km^2)^0.2, 3 m at 200 km^2,
     *  10 m at 100,000 km^2 (rivers deepen slower than they widen). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double RiverDepthScale = 1.0;

    /** The floodplains' half-width over the default: 6 channel widths beside the channel on
     *  gentle reaches (under 0.2% of grade), narrowing to the banks by 2%. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double FloodplainScale = 1.0;

    /** The band (m) about a kept lake's level where its shore is eased into a bench at the
     *  waterline (twice it each way). 0: no bench. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.0", UIMax = "20.0"))
    double LakeShoreBand = 3.0;

    /** The meanders' swing over the default (R3): up to 2.5 channel widths either side, a cycle
     *  every 11 widths, on reaches gentler than 0.6% and inside the floodplain. 0: the channels
     *  follow the lines. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double RiverMeander = 1.0;

    /** The waterfalls at knickpoints over the default (R3): a reach falling over 2.5 times faster
     *  than the reaches either side of it (and over 1.5% of grade) gathers up to 70% of its drop
     *  into a fall with a plunge pool below. 0: the bed falls evenly. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Rivers", meta = (ClampMin = "0.0", ClampMax = "1.4"))
    double RiverFalls = 1.0;

    /** Peaks on the bake's mountains (Design/SurfaceLook.md T3, T3d): summits, arêtes, faces and
     *  the valleys between them below the bake's resolution, from an eroded noise set
     *  (PeakNoiseSet), on crests and upper slopes. Sharp where the landform's character is rough
     *  (the set's hard rock), rounded where it is smooth (soft rock). Their height: the set's own
     *  relief at its placement (PeakSpacing) times this, so 1 keeps the simulated slopes; never
     *  more than the bake's local relief. 0 turns them off. (Replaces T3's PeakStrength, a
     *  fraction of the relief: a different meaning, so a new name.) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double PeakHeightScale = 1.0;

    /** The distance between the peaks' massifs (m): the set's spacing is placed at this. Its
     *  relief, and so the peaks' height, scales with it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "500.0", UIMin = "2000", UIMax = "20000"))
    double PeakSpacing = 6000.0;   // clamped to VoxelLand::PeakMinSpacing in code too

    /** Where the peaks begin and finish fading in, by the bake's local relief (m; T4b): below
     *  the start the land is the hills' (the hill set's foothills, HillFoothillGrowth), above
     *  the end the peaks'. The set's peaks are scaled to the relief, not its spacing, so at low
     *  relief they are broad and gentle: a later start keeps that land in the hills. T3's
     *  250 .. 900 put a smooth band of flattened peaks along many range fronts. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "3000"))
    double PeakReliefFadeStart = 500.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "1.0", UIMax = "4000"))
    double PeakReliefFadeEnd = 1500.0;

    /** The packed noise set the peaks are drawn from: a file name in the plugin's NoiseSets
     *  folder without .vxnt (e.g. peaks_b), or empty to pick one of the folder's peaks* sets by
     *  the seed. Without any (or if it can't be read: the log says why) the plugin's own
     *  generated set. A different set gives different summits. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample")
    FString PeakNoiseSet;

    /** The peaks' detail (Design/SurfaceLook.md T3e): a second, finer lookup over the peaks for
     *  the surface below the set's resolution (its finest valleys are hundreds of metres across),
     *  of the planet's detail set (DetailNoiseSet): rocky creases (its ridged G) where the
     *  character is rough, rounded ground (its fbm R) where it is smooth. Its height over its
     *  own: 1 keeps the set's slopes as the plugin scales them (19 degrees RMS in hard rock, 12 in
     *  soft). It lives where the peaks do, scaled with them. 0 turns it off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double PeakDetailStrength = 1.0;

    /** The detail's coarsest wavelength (m); its finest is an eighth of it (the detail sets'
     *  four octaves). Kept within a quarter of PeakSpacing in code. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "16.0", UIMin = "50", UIMax = "2000"))
    double PeakDetailWavelength = 400.0;

    /** Hash of the settings that change the bake (with the radius and the generator version,
     *  the cache key). */
    uint32 GetBakeHash() const;

    /** Hash of every field, for change detection. */
    uint32 GetSettingsHash() const;
};
