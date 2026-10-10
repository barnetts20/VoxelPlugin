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
// - Depth: the terrain depth below the sea (the planet density at the point);
// - Shore: distance to the shore and the direction toward it (open water until a shore field
//   exists; IVoxelOceanShoreSource);
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

// ---------------------------------------------------------------------------
// Inputs
// ---------------------------------------------------------------------------

namespace VoxelOceanWaves
{
    /** A depth or shore distance meaning "none near" (cm), as VOXEL_OCEAN_DEEP. */
    constexpr double Deep = 1.0e30;
}

/** Wind and storm at a point. */
struct FVoxelOceanEnvironment
{
    double WindSpeed = 0.0;      // m/s
    double WindHeading = 0.0;    // rad, from local north toward east (north: the planet's +Z)
    double Storm = 0.0;          // 0-1

    bool operator==(const FVoxelOceanEnvironment& Other) const
    {
        return WindSpeed == Other.WindSpeed && WindHeading == Other.WindHeading && Storm == Other.Storm;
    }
};

/** The source of the environment per point: the atmosphere's fields later. Any thread;
 *  deterministic in (point, time) or replicated. */
class IVoxelOceanEnvironmentSource
{
public:
    virtual ~IVoxelOceanEnvironmentSource() = default;
    virtual FVoxelOceanEnvironment Sample(const FVector& PlanetPoint, double Time) const = 0;
};

/** The shore near a point. */
struct FVoxelOceanShore
{
    double Distance = VoxelOceanWaves::Deep;    // cm; Deep: open water
    FVector Direction = FVector::ZeroVector;    // toward the shore (planet frame, tangent)
};

/** The source of shore data per point: a baked shore field later. Any thread. */
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
};

/** What every evaluation of one frame shares. */
struct FVoxelOceanWaveContext
{
    double Time = 0.0;                          // the ocean clock (s)
    double Radius = 0.0;                        // sea sphere (cm)
    FVoxelOceanEnvironment Environment;         // the constants (the GPU's, until fields exist)
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

    // A GPU model (FFT cascades) will also get a render-thread update from the ocean's view
    // extension, once per frame before the base pass; it comes with the first such model.
};

/** The test model (voxel.Ocean.TestWaves): three Gerstner waves along fixed planet directions,
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

    /** The console's settings (voxel.Ocean.TestWaves, TestWaveHeight, TestWaveLength,
     *  TestWaveChop); Amplitude 0 when off. Game thread. */
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
};

namespace VoxelOceanQuery
{
    /** Any thread. Points and results in the planet frame. Time < 0: the frame's clock. For
     *  each point: Newton steps on X + D_h(X) = foot (the horizontal displacement inverted in
     *  the tangent plane), then the waves there; Height = |p| - R - h(X), as the shaders. */
    VOXELPLUGIN_API void Run(const FVoxelOceanWaveFrame& Frame, TConstArrayView<FVector> PlanetPoints,
        TArrayView<FVoxelOceanSample> Out, EVoxelOceanQuery Flags = EVoxelOceanQuery::None, double Time = -1.0);
}
