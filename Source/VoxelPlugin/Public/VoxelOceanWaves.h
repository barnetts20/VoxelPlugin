// VoxelOceanWaves.h - The ocean's waves on the CPU (Design/OceanMeshing.md section 4.3-4.4): the
// wave model interface, the inputs every model receives per point, the frame snapshot that
// carries the waves to every consumer (surface material, underwater pass, queries), and the
// surface query. The GPU twin is Shaders/Private/VoxelOceanWaves.ush; a model implements both
// and they must agree.
//
// DETERMINISM: a model is a pure function of the planet-frame position, the ocean clock and the
// inputs below (depth, shore, environment). No camera, view or frame history enters it, so a
// server, every client and the drawn surface agree. Whatever provides the environment or the
// shore later (the atmosphere's wind and storm fields, a baked shore field) must be a
// deterministic function of seed, position and time, or replicated.
//
// INPUTS PER POINT (FVoxelOceanWaveSite), filled only when the model asks (RequiredInputs):
// - Depth: the terrain depth below the sea (the planet density at the point; an inland basin's
//   is land, stage O8);
// - Shore: distance to the coast, the direction toward it and the beach slope (the shore field,
//   IVoxelOceanShoreSource; open water without one);
// - Environment: wind and storm (IVoxelOceanEnvironmentSource; the ocean actor's constants until
//   the atmosphere provides fields).
//
// OUTPUTS: full 3D displacement (vertical along the radial plus horizontal), the normal, the
// velocity of the water there, and the surface's area ratio (Jacobian): 1 flat, below 1
// compressed toward a crest, 0 or below where the surface folds over (a breaking crest).

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanWaves.generated.h"

class FDensitySampleCompositor;
struct FVoxelOceanWeatherTable;

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

namespace VoxelOceanWaves
{
    /** A depth or shore distance meaning "none near" (cm), as VOXEL_OCEAN_DEEP. */
    constexpr double Deep = 1.0e30;

    /** The wave model the console chooses (voxel.Ocean.WaveModel): 0 a flat sea, 1 the test
     *  model, 2 the FFT model (the default). Game thread. */
    VOXELPLUGIN_API int32 ChosenModel();
}

/** Wind, storm and swell at a point. Headings are where the wind blows (or the swell runs)
 *  toward. */
struct FVoxelOceanEnvironment
{
    double WindSpeed = 0.0;      // m/s
    double WindHeading = 0.0;    // rad, from local north toward east (north: the planet's +Z)
    double Storm = 0.0;          // 0-1
    /** Swell from distant storms, as the wind speed whose sea its long waves match (m/s). */
    double SwellSpeed = 0.0;
    double SwellHeading = 0.0;   // rad, as WindHeading

    bool operator==(const FVoxelOceanEnvironment& Other) const
    {
        return WindSpeed == Other.WindSpeed && WindHeading == Other.WindHeading && Storm == Other.Storm
            && SwellSpeed == Other.SwellSpeed && SwellHeading == Other.SwellHeading;
    }
};

/** The source of the environment per point: the procedural weather until the atmosphere
 *  (VoxelOceanWeather.h), or constants. Any thread; deterministic in (point, time) or
 *  replicated. A source keeps its own clock (its weather time), a function of the ocean clock;
 *  the FFT model's weather table (VoxelOceanWeatherTable.h) keys it in that clock. */
class IVoxelOceanEnvironmentSource
{
public:
    virtual ~IVoxelOceanEnvironmentSource() = default;
    /** The source's clock at an ocean time (s). */
    virtual double WeatherTime(double OceanTime) const { return OceanTime; }
    /** The environment at a point (planet frame) at a weather time. */
    virtual FVoxelOceanEnvironment SampleWeather(const FVector& PlanetPoint, double WeatherTime) const = 0;
    /** Many points at one weather time (a source may share work between them). */
    virtual void SampleWeatherBatch(TConstArrayView<FVector> PlanetPoints, double WeatherTime, TArrayView<FVoxelOceanEnvironment> Out) const
    {
        for (int32 i = 0; i < PlanetPoints.Num() && i < Out.Num(); i++)
            Out[i] = SampleWeather(PlanetPoints[i], WeatherTime);
    }
    /** The environment at a point at an ocean time. */
    FVoxelOceanEnvironment Sample(const FVector& PlanetPoint, double OceanTime) const { return SampleWeather(PlanetPoint, WeatherTime(OceanTime)); }
};

/** How the FFT model keys the weather (VoxelOceanWeatherTable.h; the ocean's Weather settings). */
struct FVoxelOceanWeatherTiming
{
    double KeySeconds = 300.0;          // weather seconds between keys
    double HeadingStepDegrees = 10.0;   // a heading's step
    double FadeSeconds = 30.0;          // weather seconds a turn of the waves takes

    bool operator==(const FVoxelOceanWeatherTiming& Other) const
    {
        return KeySeconds == Other.KeySeconds && HeadingStepDegrees == Other.HeadingStepDegrees && FadeSeconds == Other.FadeSeconds;
    }
    bool operator!=(const FVoxelOceanWeatherTiming& Other) const { return !(*this == Other); }
};

/** The shore near a point (the shore field, stage O8; FVoxelOceanShoreField). */
struct FVoxelOceanShore
{
    /** cm to the coast: plus on the sea side, minus on land and in inland basins below sea level
     *  (the field's reach at most); Deep: no field (open water everywhere). */
    double Distance = VoxelOceanWaves::Deep;
    /** Toward the coast (planet frame, tangent), length 1 or less (shorter where two coasts'
     *  directions meet). */
    FVector Direction = FVector::ZeroVector;
    /** The beach slope at the nearest coast (the land bake's, over its cells). */
    double Slope = 0.02;

    bool HasField() const { return Distance < 0.5 * VoxelOceanWaves::Deep; }
};

/** The source of shore data per point: the shore field (FVoxelOceanShoreField). Any thread;
 *  deterministic (made from the land bake). */
class IVoxelOceanShoreSource
{
public:
    virtual ~IVoxelOceanShoreSource() = default;
    virtual FVoxelOceanShore Sample(const FVector& PlanetPoint) const = 0;
};

/** Inputs a model needs per point beyond the position. */
enum class EVoxelOceanWaveInputs : uint8
{
    None = 0,
    Depth = 1 << 0,
    Shore = 1 << 1,
    Environment = 1 << 2,
};
ENUM_CLASS_FLAGS(EVoxelOceanWaveInputs);

/** One point a model evaluates: an undisplaced point on the sea sphere (planet frame). */
struct FVoxelOceanWaveSite
{
    FVector Point = FVector::ZeroVector;
    FVector Up = FVector::UpVector;             // its radial
    double Depth = VoxelOceanWaves::Deep;       // when the model asks for it
    FVoxelOceanShore Shore;                     // when the model asks for it
    FVoxelOceanEnvironment Environment;         // the frame's constants unless a source is set
};

/** What a model returns for one site (planet frame). */
struct FVoxelOceanWaveResult
{
    FVector Displacement = FVector::ZeroVector; // cm
    double Height = 0.0;                        // its vertical part (along Up)
    FVector Normal = FVector::UpVector;         // of the displaced surface
    FVector Velocity = FVector::ZeroVector;     // of the water at the displaced point (cm/s)
    double Jacobian = 1.0;                      // area ratio; <= 0 folds over
    /** The horizontal displacement's gradient, dD_h / dx (row b, column c: dD_b / dx_c), for
     *  the query's inversion. */
    FVector HorizontalGradient[3] = { FVector::ZeroVector, FVector::ZeroVector, FVector::ZeroVector };
};

/** The waves as the shaders take them (VoxelOceanWaves.ush, FVoxelOceanWaveParams). */
struct FVoxelOceanWaveShaderParams
{
    float Amplitude = 0.0f;                     // VoxelOceanParams.z
    FVector4f Wave[3] = { FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f) };
    FVector4f Shape = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);   // choppiness, storm, wind speed, wind heading
    /** The FFT model (FVoxelOceanWavesFFT; FFT.x 0 for any other): VoxelOceanFFT, VoxelOceanRefHigh,
     *  VoxelOceanRefLow, VoxelOceanRotation, VoxelOceanTiles, VoxelOceanStates, VoxelOceanSlopeVar. */
    FVector4f FFT = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f RefHigh = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f RefLow = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f Rotation = FVector4f(0.0f, 0.0f, 0.0f, 1.0f);
    FVector4f Tiles = FVector4f(1.0f, 1.0f, 1.0f, 1.0f);
    FVector4f States = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f SlopeVar = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    /** The shore field's texel-space expansion around the reference (stage O8;
     *  VoxelOceanShore.ush, FVoxelOceanShoreFrame): the chart table's constants rows 8 to 14 for
     *  the material, the underwater pass's parameters. All zero without a field. */
    FVector4f ShoreFrame[7] = { FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f),
        FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f) };
    /** The surf's settings (stage O9a; VoxelOceanSurf.ush, VoxelOceanSurfSettingsOf): the chart
     *  table's constants rows 15 and 16, written with the shore frame (the material and the
     *  underwater pass read them from the table). All zero: no surf. */
    FVector4f Surf[2] = { FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f) };

    bool IsFFT() const { return FFT.X > 0.5f; }
};

/** What every evaluation of one frame shares. */
struct FVoxelOceanWaveContext
{
    double Time = 0.0;                          // the ocean clock (s)
    double Radius = 0.0;                        // sea sphere (cm)
    FVoxelOceanEnvironment Environment;         // the constants (the test model's; the FFT model's without a table)
    /** The weather's table for this frame (the FFT model's, VoxelOceanWeatherTable.h): the same
     *  texels the GPU reads. Null: Environment everywhere. */
    TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe> Weather;
    /** The shore field the frame's shore source holds (stage O8; FVoxelOceanShoreField): its
     *  cells per face edge (0: none) and its reach (m), for the shaders' parameters. */
    int32 ShoreCells = 0;
    double ShoreReachM = 0.0;
};

// ---------------------------------------------------------------------------
// Models
// ---------------------------------------------------------------------------

/** A wave model. Immutable once built: a new setting is a new model, so a snapshot can hold one
 *  on any thread. Evaluate is thread-safe. */
class VOXELPLUGIN_API IVoxelOceanWaveModel
{
public:
    virtual ~IVoxelOceanWaveModel() = default;

    /** The inputs it reads per point beyond the position. */
    virtual EVoxelOceanWaveInputs RequiredInputs() const { return EVoxelOceanWaveInputs::None; }

    /** Any thread. The waves at undisplaced sites on the sphere, unfaded. */
    virtual void Evaluate(const FVoxelOceanWaveContext& Context, TConstArrayView<FVoxelOceanWaveSite> Sites,
        TArrayView<FVoxelOceanWaveResult> Out) const = 0;

    /** The most the waves displace (cm): vertically (above or below the sphere) and
     *  horizontally. They size the land cull, the underwater skip test and the query's search. */
    virtual double MaxHeight() const = 0;
    virtual double MaxHorizontal() const = 0;

    /** The shaders' parameters, phases formed in double at Reference (planet frame). */
    virtual void GetShaderParams(const FVoxelOceanWaveContext& Context, const FVector& Reference,
        const FTransform& PlanetToWorld, FVoxelOceanWaveShaderParams& Out) const = 0;

    // A GPU model (the FFT cascades, FVoxelOceanWavesFFT) also runs passes every tick; the
    // ocean surface drives them (FVoxelOceanCascades), so the model itself stays a pure function.
};

/** The test model (voxel.Ocean.WaveModel 1): three Gerstner waves along fixed planet directions,
 *  mirrored exactly in VoxelOceanWaves.ush. Height A sin(theta), horizontal C A cos(theta)
 *  along each wave's tangent direction, theta = k . p + omega t, deep-water dispersion. The
 *  choppiness C is clamped so the sum of C A |k| stays below 0.95: the surface never folds. */
class VOXELPLUGIN_API FVoxelOceanWavesTest : public IVoxelOceanWaveModel
{
public:
    struct FSettings
    {
        double Amplitude = 0.0;     // per wave (cm); 0: off
        double Wavelength = 3000.0; // the longest (cm); the others are 0.61 and 0.37 of it
        double Choppiness = 0.0;    // horizontal per vertical amplitude, 0-1 (before the clamp)

        bool operator==(const FSettings& Other) const
        {
            return Amplitude == Other.Amplitude && Wavelength == Other.Wavelength && Choppiness == Other.Choppiness;
        }
        bool operator!=(const FSettings& Other) const { return !(*this == Other); }
    };

    /** The console's settings (voxel.Ocean.WaveModel 1, TestWaveHeight, TestWaveLength,
     *  TestWaveChop); Amplitude 0 when another model is chosen. Game thread. */
    static FSettings ReadConsole();

    explicit FVoxelOceanWavesTest(const FSettings& InSettings);

    virtual void Evaluate(const FVoxelOceanWaveContext& Context, TConstArrayView<FVoxelOceanWaveSite> Sites,
        TArrayView<FVoxelOceanWaveResult> Out) const override;
    virtual double MaxHeight() const override { return 3.0 * Settings.Amplitude; }
    virtual double MaxHorizontal() const override { return 3.0 * Chop * Settings.Amplitude; }
    virtual void GetShaderParams(const FVoxelOceanWaveContext& Context, const FVector& Reference,
        const FTransform& PlanetToWorld, FVoxelOceanWaveShaderParams& Out) const override;

    const FSettings& GetSettings() const { return Settings; }
    /** The choppiness after the clamp. */
    double GetChop() const { return Chop; }

private:
    FSettings Settings;
    FVector K[3];       // planet frame, rad/cm
    double Omega[3];    // rad/s
    double Chop = 0.0;
};

/** What the FFT wave model is built from (FVoxelOceanSettings' Waves and Environment;
 *  FVoxelOceanWavesFFT). */
struct FVoxelOceanFFTSettings
{
    int32 Seed = 1;
    double FetchKm = 300.0;
    double Chop = 0.9;
    double Swell = 0.3;
    double DetileCell = 1500.0;     // m; 0 off
    bool bQueryShortWaves = false;
    /** Changes only when edited; a rebuild for it gives the same waves. */
    float MipBias = 0.0f;
    /** The reach's wind (m/s): the weather's greatest long-wave speed, stepped
     *  (FVoxelOceanWavesFFT::ReachWind). The sea state itself comes from the weather per point. */
    double WindSpeed = 8.0;
    /** The shore (stage O8; VoxelOceanShoal.ush): the breaking index (0: no shoaling at all), the
     *  orbits' flattening's power, the slowing warp's strength, the coast's fade distance (m) and
     *  the least beach slope. */
    double ShoreBreaking = 0.78;
    double ShoreChopGrowth = 1.0;
    double ShoreWarp = 1.0;
    double ShoreFade = 0.0;
    double ShoreMinSlope = 0.005;
    /** The surf (stage O9a; VoxelOceanSurf.ush): its strength (0: none), sets, swash, skew and
     *  handover (breaking depths). */
    double SurfStrength = 1.0;
    double SurfSets = 1.0;
    double SurfSwash = 1.0;
    double SurfSkew = 0.85;
    double SurfHandover = 2.5;
    /** The foam (stage O9b; VoxelOceanFoam.ush, VoxelOceanSurf.ush): the open sea's decay (s; 0:
     *  none made), its coverage (0 none, 1 as calibrated) and the surf's foam. Live: the surface
     *  reads them every frame, so they aren't compared (a change doesn't rebuild the model or
     *  wipe the foam). */
    double FoamDecay = 2.5;
    double FoamCoverage = 1.0;
    double SurfFoam = 1.0;

    bool operator==(const FVoxelOceanFFTSettings& Other) const
    {
        return Seed == Other.Seed && FetchKm == Other.FetchKm && Chop == Other.Chop && Swell == Other.Swell
            && DetileCell == Other.DetileCell && bQueryShortWaves == Other.bQueryShortWaves && MipBias == Other.MipBias
            && WindSpeed == Other.WindSpeed && ShoreBreaking == Other.ShoreBreaking && ShoreChopGrowth == Other.ShoreChopGrowth
            && ShoreWarp == Other.ShoreWarp && ShoreFade == Other.ShoreFade && ShoreMinSlope == Other.ShoreMinSlope
            && SurfStrength == Other.SurfStrength && SurfSets == Other.SurfSets && SurfSwash == Other.SurfSwash
            && SurfSkew == Other.SurfSkew && SurfHandover == Other.SurfHandover;
    }
    bool operator!=(const FVoxelOceanFFTSettings& Other) const { return !(*this == Other); }
};

// ---------------------------------------------------------------------------
// The frame snapshot
// ---------------------------------------------------------------------------

/** The waves of one frame and everything needed to evaluate them, from any reference point and
 *  on any thread: the surface material (from the selection camera), the underwater pass (from
 *  each view's camera) and the surface query. Built on the game thread; immutable after. */
struct VOXELPLUGIN_API FVoxelOceanWaveFrame
{
    /** The planet frame (the sea sphere's centre and rotation) in world space, without scale. */
    FTransform PlanetToWorld = FTransform::Identity;
    FVoxelOceanWaveContext Context;
    /** Null: a flat sea. */
    TSharedPtr<const IVoxelOceanWaveModel, ESPMode::ThreadSafe> Model;
    /** Null: Context.Environment everywhere. */
    TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> Environment;
    /** Null: open water everywhere. */
    TSharedPtr<const IVoxelOceanShoreSource, ESPMode::ThreadSafe> Shore;
    /** The planet density (planet frame), for depth inputs and depth queries; may be null. */
    TSharedPtr<FDensitySampleCompositor> Density;
    /** Where the shore field marks an inland basin: points whose shore distance is below minus
     *  this (cm) are land (no water, no waves; FVoxelOceanSample::bWater false). */
    double ShoreMarginCm = 0.0;

    double Radius() const { return Context.Radius; }
    bool HasWaves() const { return Model.IsValid() && Model->MaxHeight() > 0.0; }
    double MaxHeight() const { return Model.IsValid() ? Model->MaxHeight() : 0.0; }
    double MaxHorizontal() const { return Model.IsValid() ? Model->MaxHorizontal() : 0.0; }

    /** The shaders' parameters from Reference (planet frame); zero waves without a model. */
    void ShaderParams(const FVector& Reference, FVoxelOceanWaveShaderParams& Out) const;

    /** Fills a site's inputs at an undisplaced point (planet frame) for the model. */
    void FillSites(TArrayView<FVoxelOceanWaveSite> Sites, double Time) const;
};

// ---------------------------------------------------------------------------
// The surface query
// ---------------------------------------------------------------------------

/** What a query adds beyond the height, normal and velocity. */
enum class EVoxelOceanQuery : uint8
{
    None = 0,
    /** The terrain depth below the surface point (the planet density there). */
    Depth = 1 << 0,
};
ENUM_CLASS_FLAGS(EVoxelOceanQuery);

/** The water surface at a query point. */
USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelOceanSample
{
    GENERATED_BODY()

    /** Signed height (cm) of the query point above the displaced surface; negative under water. */
    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    double Height = 0.0;

    /** The surface point on the query point's radial (world, or planet frame from the C++ core). */
    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    FVector SurfaceLocation = FVector::ZeroVector;

    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    FVector Normal = FVector::UpVector;

    /** The water's velocity at the surface point (cm/s), relative to the planet. */
    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    FVector Velocity = FVector::ZeroVector;

    /** Terrain depth below the surface point (cm); only with EVoxelOceanQuery::Depth. */
    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    double Depth = 0.0;

    /** The surface's area ratio there: 1 flat, below 1 compressed, <= 0 folded over. */
    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    double Jacobian = 1.0;

    /** The surface folds over here (a breaking crest) or the search didn't settle: more than
     *  one surface may lie on the radial, and Height is to the one the search found. */
    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    bool bAmbiguous = false;

    /** False where the shore field puts the point inland, more than the margin behind the coast
     *  (stage O8): an inland basin below sea level not joined to the sea, or dry land. It reads
     *  the field alone, not the depth (the pages and the waves' sites also want water above the
     *  terrain). True without a field. Height is still to the sea sphere there. */
    UPROPERTY(BlueprintReadOnly, Category = "Ocean")
    bool bWater = true;
};

namespace VoxelOceanQuery
{
    /** Any thread. Points and results in the planet frame. Time < 0: the frame's clock. For
     *  each point: Newton steps on X + D_h(X) = foot (the horizontal displacement inverted in
     *  the tangent plane), then the waves there; Height = |p| - R - h(X), as the shaders. */
    VOXELPLUGIN_API void Run(const FVoxelOceanWaveFrame& Frame, TConstArrayView<FVector> PlanetPoints,
        TArrayView<FVoxelOceanSample> Out, EVoxelOceanQuery Flags = EVoxelOceanQuery::None, double Time = -1.0);
}
