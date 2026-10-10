// VoxelDensity.h - The planet's density function: surface settings, the CPU evaluation of the
// shared density core (VoxelDensityCore.ush), and the one factory every consumer (terrain,
// ocean, planet) builds its compositor from.

#pragma once

#include "CoreMinimal.h"
#include "FDensitySampleCompositor.h"
#include "VoxelDensity.generated.h"

struct FVoxelDensityGPUData;
struct FVoxelLandBake;
struct FVoxelTerrestrialSettings;

/** Shape of the planet's surface heightfield. Wavelengths are fractions of the planet radius,
 *  so a setting keeps its look at any planet size; weights are fractions of the noise
 *  amplitude (the height range above the planet radius). */
USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelSurfaceSettings
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface")
    int32 Seed = 1337;

    /** Frequency ratio between successive octaves in every layer. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface", meta = (ClampMin = "1.01", UIMin = "1.5", UIMax = "3.0"))
    double Lacunarity = 2.0;

    // --- Continents: large, smooth landmasses and basins ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Continents", meta = (ClampMin = "0.000001", UIMin = "0.05", UIMax = "2.0"))
    double ContinentWavelength = 0.6;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Continents", meta = (ClampMin = "0", UIMin = "1", UIMax = "10"))
    int32 ContinentOctaves = 5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Continents", meta = (UIMin = "0.2", UIMax = "0.8"))
    double ContinentGain = 0.5;

    /** Height swing of the continents around half the amplitude (1 = the full range). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Continents", meta = (UIMin = "0.0", UIMax = "1.0"))
    double ContinentWeight = 0.6;

    // --- Mountains: ridged ranges on the higher continental ground ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Mountains", meta = (ClampMin = "0.000001", UIMin = "0.005", UIMax = "0.5"))
    double MountainWavelength = 0.08;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Mountains", meta = (ClampMin = "0", UIMin = "1", UIMax = "12"))
    int32 MountainOctaves = 8;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Mountains", meta = (UIMin = "0.2", UIMax = "0.8"))
    double MountainGain = 0.45;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Mountains", meta = (UIMin = "0.0", UIMax = "1.0"))
    double MountainWeight = 0.25;

    /** Continent value (about -1..1) where mountains begin, and the range they fade in over. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Mountains", meta = (UIMin = "-1.0", UIMax = "1.0"))
    double MountainStart = 0.1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Mountains", meta = (ClampMin = "0.0001", UIMin = "0.05", UIMax = "1.0"))
    double MountainBlend = 0.4;

    /** Ridge sharpness feedback: higher concentrates detail on the crests. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Mountains", meta = (UIMin = "0.5", UIMax = "4.0"))
    double RidgeGain = 2.0;

    // --- Detail: fine relief, damped on steep slopes ---

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Detail", meta = (ClampMin = "0.0000001", UIMin = "0.0005", UIMax = "0.05"))
    double DetailWavelength = 0.01;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Detail", meta = (ClampMin = "0", UIMin = "1", UIMax = "20"))
    int32 DetailOctaves = 14;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Detail", meta = (UIMin = "0.2", UIMax = "0.8"))
    double DetailGain = 0.45;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Detail", meta = (UIMin = "0.0", UIMax = "0.5"))
    double DetailWeight = 0.02;

    /** How strongly accumulated slope suppresses finer octaves (0 = plain fBm). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surface|Detail", meta = (ClampMin = "0.0", UIMin = "0.0", UIMax = "4.0"))
    double DetailSlopeDamping = 3.0;

    /** Hash of every field, for change detection. */
    uint32 GetSettingsHash() const;
};

/** CPU evaluation of the planet's density: surface heightfield, cave slot and their
 *  composition, from VoxelDensityCore.ush. Positions are double and each noise octave is
 *  evaluated on an integer lattice split in double, so results are precise at any radius.
 *  Immutable after construction; safe to call from any thread. */
class VOXELPLUGIN_API FVoxelDensityCPU
{
public:
    /** InMinFootprint (cm) is the smallest footprint any sample is evaluated at: octaves finer
     *  than it are always left out, whatever footprint a caller passes (0 keeps all). */
    FVoxelDensityCPU(const FVoxelSurfaceSettings& InSettings, double InPlanetRadius, double InNoiseAmplitude, double InMinFootprint = 0.0);

    /** The terrestrial generator: Bake's heights (sampled through a warp, VoxelLandSample.ush)
     *  plus the gullies (VoxelLandGully.ush) and slope-damped detail. InPlanetRadius is the base radius and InNoiseAmplitude the
     *  height range above it that hold every height (VoxelLand::ShapeFor), so the contract of
     *  HeightAt and the lattice sizing are the noise surface's. */
    FVoxelDensityCPU(const FVoxelTerrestrialSettings& InLand, TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> InBake,
        double InPlanetRadius, double InNoiseAmplitude, double InMinFootprint = 0.0);

    /** Height above the planet radius (cm, 0..NoiseAmplitude) in direction Dir (any length),
     *  with octaves finer than Footprint (cm) left out. */
    double HeightAt(const FVector& Dir, double Footprint) const;

    /** Terrain density (negative = solid) at each input position, without edits. */
    void Sample(const FSampleInput& Input, float* DensityOut) const;

    /** The octave tables and constants the GPU evaluation (VoxelDensitySample.ush) needs to
     *  compute this density. */
    void BuildGPUData(FVoxelDensityGPUData& Out) const;

    double GetPlanetRadius() const { return PlanetRadius; }
    double GetNoiseAmplitude() const { return NoiseAmplitude; }

    /** The terrestrial generator's bake (null for the noise surface). */
    const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& GetLandBake() const { return LandBake; }

private:
    /** One octave, fixed at construction; both backends read these. With a lacunarity of 2,
     *  at most 32 octaves and the finest coordinate below 2^30 (radius times the finest
     *  frequency), its lattice coordinate is the layer's finest one shifted right by Shift, in
     *  fixed point (VoxelFixedLattice); otherwise Shift is VoxelNoFixedPoint. */
    struct FOctave
    {
        double Frequency = 0.0;       // cycles per cm
        FVector Offset;               // lattice offset (decorrelates octaves)
        float Wavelength = 0.0f;      // cm, for the band weight
        float Amplitude = 1.0f;       // gain^octave, accumulated in float
        uint32 Seed = 0;
        uint32 Shift = 0xffffffffu;
        uint32 OffsetCell[3] = {};    // Offset in 32.32 fixed point
        uint32 OffsetFrac[3] = {};
    };

    struct FLayer
    {
        uint32 Kind = 0;              // VoxelLayerPlain / Ridged / Damped
        double InvNorm = 0.0;         // output scale (see MakeLayer)
        TArray<FOctave> Octaves;
    };

    static FLayer MakeLayer(uint32 Kind, double WavelengthFraction, double InPlanetRadius, double InLacunarity, double InGain, int32 InOctaves, uint32 InSeed);
    double EvaluateLayer(const FLayer& Layer, const FVector& SurfacePoint, double Footprint) const;
    /** EvaluateLayer and, when OutGradient is set, its gradient (per cm; plain layers). */
    double EvaluateLayerGradient(const FLayer& Layer, const FVector& SurfacePoint, double Footprint, FVector* OutGradient) const;
    float SampleOne(const FVector& Position, double Footprint) const;

    /** HeightAt for the terrestrial generator, from the surface point on the sea-level sphere
     *  (the bake's SeaRadius; VoxelDensitySample.ush samples the same sphere). */
    double LandHeightAt(const FVector& SurfacePoint, double Footprint) const;

    FVoxelSurfaceSettings Settings;
    double PlanetRadius = 1.0;
    double NoiseAmplitude = 0.0;
    double MinFootprint = 0.0;
    FLayer Continent;
    FLayer Mountain;
    FLayer Detail;

    // The terrestrial generator (all unused when LandBake is null).
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> LandBake;
    FLayer LandWarp[3];                 // the sampling warp's components (no band limit)
    double LandHeightOffset = 0.0;      // cm: the bake's sea-level heights to heights above PlanetRadius
    double LandDetailAmplitude = 0.0;   // cm
    double LandWarpAmplitude = 0.0;     // cm
    double LandSampleRadius = 1.0;      // cm: the sea-level sphere the warp and detail are sampled on
    float LandCellM = 1.0f;             // the bake cell's arc at the face centre (m)
    float LandGullyStrength = 0.0f;     // VoxelLandGully.ush
    uint32 LandGullyOctaves = 0;
    float LandGullyFlowGain = 0.0f;
    float LandGullyDetail = 1.0f;
    uint32 LandGullySeed = 0;
    TArray<uint32> LandFlowPacked;      // the bake's drainage, two f16 cells per word (empty: none)
};

namespace VoxelDensity
{
    /** The compositor every terrain, ocean and planet consumer samples: the CPU density as its
     *  only layer, plus EditStore (may be null) applied on top. MinFootprint is the terrain's
     *  finest cell size (TerrainFinestCell), so every consumer of one compositor sees the same
     *  field; 0 keeps every octave. */
    VOXELPLUGIN_API TSharedPtr<FDensitySampleCompositor> MakeCompositor(const FVoxelSurfaceSettings& Settings,
        double PlanetRadius, double NoiseAmplitude, TSharedPtr<FSparseEditStore> EditStore, double MinFootprint = 0.0);

    /** The same for the terrestrial generator: PlanetRadius and NoiseAmplitude from
     *  VoxelLand::ShapeFor(Land, *Bake). */
    VOXELPLUGIN_API TSharedPtr<FDensitySampleCompositor> MakeTerrestrialCompositor(const FVoxelTerrestrialSettings& Land,
        TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake, double PlanetRadius, double NoiseAmplitude,
        TSharedPtr<FSparseEditStore> EditStore, double MinFootprint = 0.0);

    /** Root half-extent of a planet's terrain lattice: holds every possible surface point. */
    inline double RootExtentFor(double PlanetRadius, double NoiseAmplitude) { return (PlanetRadius + NoiseAmplitude) * 1.05; }

    /** Cell size (cm) of the finest terrain lattice level (MaxDepth halvings of the root). */
    inline double TerrainFinestCell(double PlanetRadius, double NoiseAmplitude, int32 MaxDepth)
    {
        return 2.0 * RootExtentFor(PlanetRadius, NoiseAmplitude) / FMath::Pow(2.0, (double)MaxDepth);
    }
}
