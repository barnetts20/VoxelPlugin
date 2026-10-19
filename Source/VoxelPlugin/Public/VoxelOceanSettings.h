// VoxelOceanSettings.h - The ocean's settings as one struct, held by the ocean actor and by the
// planet that owns it (stage O5): a planet forwards its copy whole, so a respawned ocean comes
// back with the same settings and nothing is set in two places. The ocean applies only what
// changed (AOceanSphereActor::SetOceanSettings): the mesh rebuilds, the optics apply to the
// surface material, everything else is read every tick.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanSettings.generated.h"

USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelOceanSettings
{
    GENERATED_BODY()

    // --- Mesh (a change rebuilds the surface) ---

    /** Spacing between vertices at the finest level near the camera (cm). The finest level is
     *  the one at or just under it (the ocean's ActualSpacing). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "1.0", UIMin = "10.0", UIMax = "1000.0"))
    double FinestSpacing = 50.0;

    /** Quads along a tile edge: 8, 16 or 32 (snapped to one of them). The number of tiles
     *  doesn't depend on it, so the triangle count grows with its square. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "8", ClampMax = "32"))
    int32 TileQuads = 16;

    /** A tile splits while the camera is within this many tile sizes of it. Higher: more
     *  detail and a wider morph band. At least 2.25 (the morph band closes below it). Applies
     *  without a rebuild. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "2.25", UIMax = "6.0"))
    double LodDistanceFactor = 2.5;

    /** Buffer sets kept ready, so a commit doesn't wait for one to be created. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "0", UIMax = "32"))
    int32 SpareBufferSets = 8;

    // --- Depth ---

    /** Leave out the triangles the land stands well above (the terrain hides them anyway). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Depth")
    bool bCullUnderLand = true;

    /** How far (cm) the land must stand above the sea at a triangle's corners, beyond what the
     *  terrain's slope bound and the waves' reach allow for, before it is left out. Covers the
     *  terrain mesh's error. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Depth", meta = (ClampMin = "0.0", UIMax = "5000.0", EditCondition = "bCullUnderLand"))
    double LandCullMargin = 500.0;

    // --- Underwater ---

    /** The view from under water: a pass of the ocean's own after the atmosphere, ahead of
     *  depth of field. It decides each pixel itself and leaves a view whose camera can't reach
     *  the water untouched. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Underwater")
    bool bUnderwaterView = true;

    // --- Optics ---

    /** Absorption of the water per metre, per channel (red goes first). Feeds the underwater
     *  pass and the surface material (VoxelOceanAbsorption). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor Absorption = FLinearColor(0.35f, 0.07f, 0.03f);

    /** Scattering of the water per metre, per channel. Feeds the underwater pass and the
     *  surface material (VoxelOceanScattering). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor Scattering = FLinearColor(0.02f, 0.03f, 0.035f);

    /** Forward scattering anisotropy, for a Single Layer Water surface (VoxelOceanPhaseG). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
    float PhaseG = 0.8f;

    /** The light the water scatters toward the camera under water, in the scene's light units
     *  (linear, before exposure, so it holds as the exposure adapts), until the atmosphere
     *  provides the sun's. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor UnderwaterLight = FLinearColor(0.4f, 0.8f, 1.0f);

    /** The thin band (cm) where the near plane meets the surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics", meta = (ClampMin = "0.0", UIMax = "10.0"))
    double MeniscusWidth = 1.5;

    /** The waterline's tint, in the scene's light units (as UnderwaterLight). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor MeniscusColor = FLinearColor(0.02f, 0.06f, 0.08f);

    /** The foam on the surface seen from under water (whitecaps, the surf's bores and swash: the
     *  Foam category's), in the scene's light units (as UnderwaterLight): lit from above, brighter
     *  than the water around it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor UnderwaterFoamColor = FLinearColor(0.6f, 0.7f, 0.75f);

    // --- Environment (the constant weather: everywhere when bProceduralWeather is off) ---

    /** Wind speed (m/s) with the procedural weather off. The FFT waves blend their sea states by
     *  it (5, 10 and 18 m/s). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", UIMax = "40.0"))
    double WindSpeed = 8.0;

    /** Wind heading (degrees from local north toward east; north is the planet's +Z): the
     *  direction the waves travel, with the procedural weather off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (UIMin = "0.0", UIMax = "360.0"))
    double WindHeading = 0.0;

    /** Storm intensity, 0-1, with the procedural weather off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double StormIntensity = 0.0;

    /** The ocean clock's rate against the game's (server) time. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double OceanTimeScale = 1.0;

    // --- Waves (the FFT model, voxel.Ocean.WaveModel 2; a change rebuilds the waves) ---

    /** The waves' seed: every machine with the same seed, clock and wind draws the same sea. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves")
    int32 WaveSeed = 1;

    /** The distance (km) the wind has blown over open water: a shorter fetch makes younger,
     *  shorter, lower waves. Past what the wind needs to raise a fully developed sea, nothing
     *  changes (about 300 km at 15 m/s). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "1.0", UIMax = "2000.0"))
    double WaveFetch = 300.0;

    /** Horizontal displacement per unit of the waves' own (0: rounded sines; 1: sharp crests).
     *  Higher folds the surface where crests break. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    double WaveChop = 0.9;

    /** Long waves narrowed into parallel trains, 0-1 (Horvath's swell). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double WaveSwell = 0.3;

    /** The detiling lattice's cell (m) for the 1 km band: every band blends three shifted copies
     *  over a lattice this size scaled by its own tile (stage O9b3: 1,500, 363, 89 and 21 m at the
     *  default), so no band's tile repeats. 0: off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "0.0", UIMax = "5000.0"))
    double WaveDetileCell = 1500.0;

    /** The surface query includes the 3-15 m waves too (most of a calm sea; 0.7 ms more per
     *  update). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves")
    bool bQueryShortWaves = false;

    /** Mip bias of the wave lookups: above 0 blurs sooner (cheaper, softer), below sharpens. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (UIMin = "-2.0", UIMax = "2.0"))
    float WaveMipBias = 0.0f;

    // --- Weather (the procedural wind, storms and swell until the atmosphere; Design/OceanWaves.md section 4) ---

    /** The procedural weather: latitude bands, noise and drifting storms, the same on every
     *  machine. Off: the Environment constants everywhere. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
    bool bProceduralWeather = true;

    /** The weather's seed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather"))
    int32 WeatherSeed = 1;

    /** The trade winds' speed (m/s, easterly, below 30 degrees). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "20.0"))
    double TradeWind = 7.0;

    /** The westerlies' speed (m/s, 30 to 60 degrees). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "20.0"))
    double Westerly = 10.0;

    /** The polar easterlies' speed (m/s, above 60 degrees). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "20.0"))
    double PolarWind = 5.0;

    /** The wind's noise (m/s) and its size (km). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "10.0"))
    double WindNoise = 3.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "10.0", UIMax = "3000.0"))
    double WindNoiseSize = 600.0;

    /** Storms alive at once. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0", ClampMax = "64"))
    int32 StormCount = 6;

    /** A storm's peak wind (m/s), radius (km) and life (hours of weather time). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "40.0"))
    double StormWind = 22.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "10.0", UIMax = "1000.0"))
    double StormRadius = 220.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.1", UIMax = "24.0"))
    double StormHours = 3.0;

    /** The swell a storm sends (a fraction of its wind) and how far it carries (km). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", ClampMax = "1.0"))
    double SwellStrength = 0.6;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "10.0", UIMax = "5000.0"))
    double SwellRange = 1500.0;

    /** The weather clock against the ocean's (above 1 to watch storms move) and its offset (s:
     *  where in the weather a level starts). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "100.0"))
    double WeatherTimeScale = 1.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather"))
    double WeatherTimeOffset = 0.0;

    /** Weather seconds between the waves' weather keys; a heading's step (degrees); how long a
     *  turn of the waves takes (weather seconds). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", AdvancedDisplay, meta = (ClampMin = "10.0", UIMax = "3600.0"))
    double WeatherKeySeconds = 300.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "45.0"))
    double WaveHeadingStep = 10.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "300.0"))
    double WaveTurnSeconds = 30.0;

    // -- Shore (stage O8; Design/OceanWaves.md section 5) --------------------------------------

    /** Waves shoal in shallow water (they grow, steepen, break and fade at the coast) and slow
     *  toward the coast. Off: deep-water waves everywhere (the shore field still keeps inland
     *  basins dry). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore")
    bool bShoaling = true;

    /** The breaking index: the waves' significant height is held under this times the depth. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.1", ClampMax = "2.0"))
    double ShoreBreaking = 0.78;

    /** How much the orbits flatten in shallow water: the horizontal displacement grows as
     *  1 / tanh(k d) to this power (0: not at all, 1: as linear waves). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", ClampMax = "2.0"))
    double ShoreChopGrowth = 1.0;

    /** The waves shorten and slow toward the coast (the shallow-water dispersion, along the
     *  shore field's direction): 1 as the dispersion says, 0 off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", ClampMax = "2.0"))
    double ShoreWarp = 1.0;

    /** The FFT waves fade out over this distance to the coast (m; the depth over the beach
     *  slope). 0: no fade; the surf (Surf category) takes the long waves over near the coast by
     *  itself. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", UIMax = "1000.0"))
    double ShoreFadeDistance = 0.0;

    /** The depth the waves feel leaves out terrain features narrower than about twice this (m):
     *  waves don't feel bumps much shorter than themselves, and the surf's timing follows the
     *  depth so closely on a gentle beach that the terrain's fine detail broke its crests into
     *  spikes. Larger: smoother crests that follow the bottom less. 0: the mesh's own detail.
     *  The land cull and the query's reported depth keep the terrain's real height. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", UIMax = "50.0"))
    double ShoreDepthSmoothing = 10.0;

    /** The gentlest beach slope the shoaling assumes (the land bake's coastal slope, at least). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (EditCondition = "bShoaling", ClampMin = "0.0001", UIMax = "0.1"))
    double ShoreMinSlope = 0.005;

    /** The ocean mesh near coasts (stage O9c): within ShoreDetailBand of the coast it is as fine
     *  as if the camera were this many times closer, so the surf's bores keep their shape from a
     *  few hundred metres off the beach. 1: off. Costs tiles with the camera by a coast (3: about
     *  a third more by a straight one, up to twice as many by a busy one with islands). With the
     *  FFT waves (their vertex code; no other wiring). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (ClampMin = "1.0", UIMax = "4.0"))
    double ShoreDetail = 3.0;

    /** The band of finer mesh (m from the coastline, the shore field's zero line: either side);
     *  outside it the refinement fades over the camera's own distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "2000.0"))
    double ShoreDetailBand = 300.0;

    /** How far the finer mesh reaches from the camera (m, up to 2 km): it saves at most
     *  (1 - 1 / ShoreDetail) of this distance, so far coasts stay nearly as coarse as without it.
     *  The coast is measured to 10 km: past that, every point saves the same little (a third of a
     *  kilometre at the defaults, from 10 km out: a percent or two more tiles there). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "0.0", ClampMax = "2000.0"))
    double ShoreDetailReach = 500.0;

    /** The shore field holds distances up to this (km); the slowing reaches no farther. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "1.0", UIMax = "200.0"))
    double ShoreFieldReach = 40.0;

    /** Inland basins below sea level lose their ocean water beyond this many land bake cells from
     *  the sea (the bake's coast is a cell or so from the terrain's own): from here to 1.5 times it
     *  the sea's surface sinks below the ground (stage O9b3), past that the basin is land. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "8.0"))
    double ShoreBasinMargin = 1.5;

    // -- Surf (stage O9a; Design/OceanWaves.md section 6) ----------------------------------------

    /** Surf near the coast: the long waves become trains that follow the depth's contours, break
     *  into bores and run up the beach as swash. Needs bShoaling and the shore field. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bShoaling"))
    bool bSurf = true;

    /** The surf's height against the sea state's long waves (1: their significant height). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", UIMax = "3.0"))
    double SurfStrength = 1.0;

    /** How much the waves come in sets (groups of six, larger and smaller) and vary along the
     *  coast in height and timing: 0 every wave alike everywhere, 1 full. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", ClampMax = "1.0"))
    double SurfSets = 1.0;

    /** The swash's run-up against Stockdon's (2006) estimate for the waves and the beach slope
     *  (0: the waterline stays put). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", UIMax = "3.0"))
    double SurfSwash = 1.0;

    /** The swash's swing either side of its mean is held within this distance up and down the
     *  beach (m, at the shore field's beach slope): on a gentle beach Stockdon's infragravity
     *  swash alone would sweep tens of metres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "1.0", UIMax = "50.0"))
    double SurfSwashReach = 10.0;

    /** How far a breaking wave's landward face steepens: its front grows 1 / (1 - this) times as
     *  steep as its back once the wave is saturated (0: symmetric). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", ClampMax = "0.95"))
    double SurfSkew = 0.85;

    /** Where the surf takes over from the long FFT waves: this many breaking depths of their
     *  significant height. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", AdvancedDisplay, meta = (EditCondition = "bSurf", ClampMin = "1.0", UIMax = "6.0"))
    double SurfHandover = 2.5;

    // -- Foam (stage O9b; Design/OceanWaves.md section 6) ----------------------------------------

    /** Foam: whitecaps on the open sea (the material's VoxelOceanFoam texture and Foam output) and
     *  white water on the surf's bores and swash. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam")
    bool bFoam = true;

    /** How much of the open sea foams, as a multiple of the calibrated whitecap coverage (0.4% at
     *  7 m/s, 1.5% at 10, 4.7% at 14, 11% at 18 m/s, none below 3 m/s). 0: none. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", UIMax = "8.0"))
    double FoamCoverage = 1.0;

    /** How long a whitecap's trail lingers (s): the foam's envelope falls one standard deviation
     *  in this time, so a strong whitecap's trail lasts about this long, a weak one's less. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.1", UIMax = "10.0"))
    double FoamDecay = 2.5;

    /** The surf's foam: the bores' white water and trails, the swash's foamy edge (0: none). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", UIMax = "2.0"))
    double SurfFoam = 1.0;

    /** TileQuads snapped to 8, 16 or 32 and the other fields to their limits. */
    void Sanitize()
    {
        TileQuads = 1 << FMath::Clamp(FMath::RoundToInt(FMath::Log2((double)FMath::Max(TileQuads, 1))), 3, 5);
        FinestSpacing = FMath::Max(FinestSpacing, 1.0);
        LodDistanceFactor = FMath::Max(LodDistanceFactor, 2.25);
        SpareBufferSets = FMath::Max(SpareBufferSets, 0);
        LandCullMargin = FMath::Max(LandCullMargin, 0.0);
        MeniscusWidth = FMath::Max(MeniscusWidth, 0.0);
        WindSpeed = FMath::Max(WindSpeed, 0.0);
        StormIntensity = FMath::Clamp(StormIntensity, 0.0, 1.0);
        OceanTimeScale = FMath::Max(OceanTimeScale, 0.0);
        WaveFetch = FMath::Max(WaveFetch, 1.0);
        WaveChop = FMath::Clamp(WaveChop, 0.0, 2.0);
        WaveSwell = FMath::Clamp(WaveSwell, 0.0, 1.0);
        WaveDetileCell = FMath::Max(WaveDetileCell, 0.0);
        TradeWind = FMath::Max(TradeWind, 0.0);
        Westerly = FMath::Max(Westerly, 0.0);
        PolarWind = FMath::Max(PolarWind, 0.0);
        WindNoise = FMath::Max(WindNoise, 0.0);
        WindNoiseSize = FMath::Max(WindNoiseSize, 10.0);
        StormCount = FMath::Clamp(StormCount, 0, 64);
        StormWind = FMath::Max(StormWind, 0.0);
        StormRadius = FMath::Max(StormRadius, 10.0);
        StormHours = FMath::Max(StormHours, 0.1);
        SwellStrength = FMath::Clamp(SwellStrength, 0.0, 1.0);
        SwellRange = FMath::Max(SwellRange, 10.0);
        WeatherTimeScale = FMath::Max(WeatherTimeScale, 0.0);
        WeatherKeySeconds = FMath::Max(WeatherKeySeconds, 10.0);
        WaveHeadingStep = FMath::Clamp(WaveHeadingStep, 0.0, 45.0);
        // A turn ends within its key (the next key's starts from this one's headings).
        WaveTurnSeconds = FMath::Clamp(WaveTurnSeconds, 0.0, WeatherKeySeconds);
        ShoreBreaking = FMath::Clamp(ShoreBreaking, 0.1, 2.0);
        ShoreChopGrowth = FMath::Clamp(ShoreChopGrowth, 0.0, 2.0);
        ShoreWarp = FMath::Clamp(ShoreWarp, 0.0, 2.0);
        ShoreFadeDistance = FMath::Max(ShoreFadeDistance, 0.0);
        ShoreMinSlope = FMath::Max(ShoreMinSlope, 1.0e-4);
        ShoreDetail = FMath::Clamp(ShoreDetail, 1.0, 8.0);
        ShoreDetailBand = FMath::Clamp(ShoreDetailBand, 0.0, 5000.0);
        ShoreDetailReach = FMath::Clamp(ShoreDetailReach, 0.0, 2000.0);
        ShoreFieldReach = FMath::Max(ShoreFieldReach, 1.0);
        ShoreBasinMargin = FMath::Max(ShoreBasinMargin, 0.0);
        ShoreDepthSmoothing = FMath::Clamp(ShoreDepthSmoothing, 0.0, 500.0);
        SurfStrength = FMath::Max(SurfStrength, 0.0);
        SurfSets = FMath::Clamp(SurfSets, 0.0, 1.0);
        SurfSwash = FMath::Max(SurfSwash, 0.0);
        SurfSwashReach = FMath::Max(SurfSwashReach, 1.0);
        SurfSkew = FMath::Clamp(SurfSkew, 0.0, 0.95);
        SurfHandover = FMath::Max(SurfHandover, 1.0);
        FoamCoverage = FMath::Max(FoamCoverage, 0.0);
        FoamDecay = FMath::Max(FoamDecay, 0.1);
        SurfFoam = FMath::Max(SurfFoam, 0.0);
    }

    /** The fields whose change rebuilds the surface. */
    bool SameMesh(const FVoxelOceanSettings& Other) const
    {
        return FinestSpacing == Other.FinestSpacing && TileQuads == Other.TileQuads && SpareBufferSets == Other.SpareBufferSets;
    }

    /** The fields the surface material receives. */
    bool SameOptics(const FVoxelOceanSettings& Other) const
    {
        return Absorption == Other.Absorption && Scattering == Other.Scattering && PhaseG == Other.PhaseG;
    }
};
