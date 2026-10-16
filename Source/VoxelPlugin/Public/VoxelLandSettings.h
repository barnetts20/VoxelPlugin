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

    /** The hill set's detail (as PeakDetailStrength: the peaks' detail tile at the hills' own
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
     *  the surface below the set's resolution (its finest valleys are hundreds of metres across):
     *  rocky creases where the character is rough, rounded ground where it is smooth. Its height
     *  over its own: 1 keeps the tile's slopes (about 19 degrees RMS in hard rock, 12 in soft). It
     *  lives where the peaks do, scaled with them. 0 turns it off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "2.0"))
    double PeakDetailStrength = 1.0;

    /** The detail's coarsest wavelength (m); its finest is a sixteenth of it. Kept within a
     *  quarter of PeakSpacing in code. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "16.0", UIMin = "50", UIMax = "2000"))
    double PeakDetailWavelength = 400.0;

    /** Hash of the settings that change the bake (with the radius and the generator version,
     *  the cache key). */
    uint32 GetBakeHash() const;

    /** Hash of every field, for change detection. */
    uint32 GetSettingsHash() const;
};
