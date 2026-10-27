// VoxelDensity.h - The planet's density function: surface settings, the CPU evaluation of the
// shared density core (VoxelDensityCore.ush), and the one factory every consumer (terrain,
// ocean, planet) builds its compositor from.

#pragma once

#include "CoreMinimal.h"
#include "FDensitySampleCompositor.h"
#include "VoxelDensity.generated.h"

struct FVoxelDensityGPUData;
struct FVoxelLandBake;
struct FVoxelLandClimateField;
struct FVoxelLandNoiseSet;
struct FVoxelLandRiverSet;
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

/** What GetWaterAt finds (R2, Design/RiversLakes.md 3.6). */
UENUM(BlueprintType)
enum class EVoxelWaterKind : uint8
{
    None,
    Sea,
    Lake,
    River
};

/** The water at a point of a planet (APlanetActor::GetWaterAt). Heights are metres above the
 *  planet's sea level, along the planet's radius through the point. */
USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelWaterSample
{
    GENERATED_BODY()

    /** The water over the ground there (None: dry ground). */
    UPROPERTY(BlueprintReadOnly, Category = "Water")
    EVoxelWaterKind Kind = EVoxelWaterKind::None;

    /** The water's surface (m above sea level); the ground's where there is none. */
    UPROPERTY(BlueprintReadOnly, Category = "Water")
    float SurfaceHeight = 0.0f;

    /** The ground's height (m above sea level), without edits. */
    UPROPERTY(BlueprintReadOnly, Category = "Water")
    float GroundHeight = 0.0f;

    /** The water's depth there (m: surface less ground; 0 without water). */
    UPROPERTY(BlueprintReadOnly, Category = "Water")
    float Depth = 0.0f;

    /** How far the queried point lies below the water's surface (m; negative above it). */
    UPROPERTY(BlueprintReadOnly, Category = "Water")
    float PointDepth = 0.0f;

    /** A river's current (cm/s, world): downstream, faster on steeper and deeper reaches. Zero
     *  in lakes and the sea (tides and currents are later work). */
    UPROPERTY(BlueprintReadOnly, Category = "Water")
    FVector FlowVelocity = FVector::ZeroVector;
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
     *  plus the gullies (VoxelLandGully.ush) and the detail layer (VoxelLandDetail.ush, N1). InPlanetRadius is the base radius and InNoiseAmplitude the
     *  height range above it that hold every height (VoxelLand::ShapeFor), so the contract of
     *  HeightAt and the lattice sizing are the noise surface's. */
    FVoxelDensityCPU(const FVoxelTerrestrialSettings& InLand, TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> InBake,
        double InPlanetRadius, double InNoiseAmplitude, double InMinFootprint = 0.0,
        TSharedPtr<const FVoxelLandClimateField, ESPMode::ThreadSafe> InClimate = nullptr);

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

    /** The peaks' noise set (VoxelLandPeaks.ush; null without peaks). */
    const TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& GetLandNoise() const { return LandNoise; }

    /** T5c: the climate the dunes were placed by (null: no dunes). */
    const TSharedPtr<const FVoxelLandClimateField, ESPMode::ThreadSafe>& GetLandClimate() const { return LandClimate; }

    /** R2: the water at Direction from the planet's centre (any length; planet frame), from the
     *  same shared code as the carving (VoxelLandCarve.ush), at the finest footprint: the sea
     *  where the ground is below sea level, a kept lake's level, a river's water line in its
     *  channel. FlowVelocity is in the planet frame (cm/s); PointDepth is left 0 (the planet's
     *  query fills it in). The noise surface: None, with the ground's height above the planet
     *  radius (its sea level is the planet's: APlanetActor::GetWaterAt). */
    FVoxelWaterSample WaterAt(const FVector& Direction) const;

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
    /** The same, and with OutWater the water there (WaterAt). */
    double LandSurfaceAt(const FVector& SurfacePoint, double Footprint, FVoxelWaterSample* OutWater) const;

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
    double LandDetailAmplitude = 0.0;   // cm: the detail layer's (VoxelLandDetail.ush, N1)
    double LandWarpAmplitude = 0.0;     // cm
    double LandSampleRadius = 1.0;      // cm: the sea-level sphere the warp and detail are sampled on
    float LandCellM = 1.0f;             // the bake cell's arc at the face centre (m)
    float LandGullyStrength = 0.0f;     // VoxelLandGully.ush
    uint32 LandGullyOctaves = 0;
    float LandGullyFlowGain = 0.0f;
    float LandGullyDetail = 1.0f;
    uint32 LandGullySeed = 0;
    float LandHillRelief = 0.0f;        // VoxelLandHills.ush
    double LandHillFrequency = 0.0;     // cycles per cm (the finest octave's)
    uint32 LandHillOctaves = 0;
    uint32 LandHillSeed = 0;
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> LandNoise;   // the noise pack: the peaks', detail, hills' and strata's sets (null: none used)
    // VoxelLandPeaks.ush's inputs: the set's placement and normalisation (the shader's header).
    float LandPeakStrength = 0.0f;
    float LandPeakTexelsPerCell = 0.0f; // 0: no peaks
    uint32 LandPeakSeed = 0;
    float LandPeakSpacingM = 0.0f;
    float LandPeakReliefM = 0.0f;
    float LandPeakReliefStart = 500.0f;   // T4b: the peaks' fade-in over the local relief (m)
    float LandPeakReliefEnd = 1500.0f;
    float LandPeakMeanR = 0.0f;
    float LandPeakMeanG = 0.0f;
    float LandPeakScaleR = 1.0f;
    float LandPeakScaleG = 1.0f;
    float LandPeakScaleB = 1.0f;
    // The peaks' detail (VoxelLandPeaks.ush, T3e): the pack's detail set (N1; LandNoise's Detail*),
    // hard rock its G, soft rock its R, with those channels' means.
    float LandPeakDetailTexelsPerCell = 0.0f;  // 0: no detail
    float LandDetailMeanHard = 0.0f;
    float LandDetailMeanSoft = 0.0f;
    float LandPeakDetailScaleHard = 0.0f;
    float LandPeakDetailScaleSoft = 0.0f;
    float LandPeakDetailFade = 0.0f;
    // The hills' set (VoxelLandHillSet.ush, T4a; its chain: LandNoise's Hill*). 0 texels: none
    // (the procedural hills, LandHillFrequency and LandHillOctaves).
    float LandHillTexelsPerCell = 0.0f;
    float LandHillScaleR = 1.0f;
    float LandHillScaleG = 1.0f;
    float LandHillScaleB = 1.0f;
    float LandHillMeanR = 0.0f;       // N1b: R's and G's means (unit), for the face blend
    float LandHillMeanG = 0.0f;
    float LandHillSpacingM = 0.0f;
    float LandHillSwellTexelsPerCell = 0.0f;   // 0: no swells
    float LandHillSwellM = 0.0f;
    float LandHillSwellScaleG = 1.0f;
    float LandHillDetailTexelsPerCell = 0.0f;  // 0: no detail
    float LandHillDetailScaleHard = 0.0f;
    float LandHillDetailScaleSoft = 0.0f;
    float LandHillDetailFade = 0.0f;
    float LandHillFoothill = 0.0f;         // T4b: the hills' growth in the foothills
    float LandStrataTexelsPerCell = 0.0f;      // T4c: the strata set's placement (0: none)
    float LandStrataScaleR = 1.0f;
    float LandStrataScaleG = 1.0f;
    float LandStrataScaleB = 1.0f;
    float LandStrataMeanR = 0.0f;
    float LandStrataMeanG = 0.0f;
    float LandStrataSpacingM = 0.0f;
    float LandStrataRelief = 0.0f;             // m: the set's relief times the strength, capped
    float LandStrataFlat = 0.0f;               // its share on the flats
    float LandStrataThreshold = 2.0f;          // the province switch (above 1: none)
    float LandStrataWidth = 0.05f;
    uint32 LandStrataSeed = 0;
    float LandStrataDetailTexelsPerCell = 0.0f;   // 0: no detail
    float LandStrataDetailScaleHard = 0.0f;
    float LandStrataDetailScaleSoft = 0.0f;
    float LandStrataDetailFade = 0.0f;
    TArray<uint32> LandReliefPacked;    // the bake's local relief, four cells per word (empty: none)
    TArray<uint32> LandProvincePacked;  // T4c: the bake's province, four cells per word (as the relief's)
    TSharedPtr<const FVoxelLandRiverSet, ESPMode::ThreadSafe> LandRivers;   // R1: the rivers (null: none)
    uint32 LandRiverTilesPerFace = 0;          // R1: the rivers (0: none)
    float LandRiverRadiusM = 1.0f;
    float LandRiverWidthScale = 1.0f;
    float LandRiverValleyScale = 1.0f;
    // R2: the rivers' and lakes' carve (VoxelLandCarve.ush).
    float LandRiverCarve = 0.0f;               // 0: not carved
    float LandRiverDepthScale = 1.0f;
    float LandRiverFloodplainScale = 1.0f;
    float LandRiverMinLog2Area = 0.0f;
    float LandLakeShoreBand = 0.0f;
    float LandRiverMeander = 0.0f;      // R3: RiverMeander (the meanders' swing)
    float LandRiverFalls = 0.0f;        // R3: RiverFalls (the knickpoints' falls)
    uint32 LandHasWater = 0;                   // 1: the bake's lake levels (its Water) are read
    // N1: the detail layer from the pack's detail set (VoxelLandDetail.ush): its A channel at
    // LandDetailScales scales, 16 times apart.
    float LandDetailTexelsPerCell = 0.0f;      // the first scale's (0: no detail)
    uint32 LandDetailScales = 0;
    float LandDetailMeanA = 0.0f;
    float LandDetailScaleA = 0.0f;
    float LandDetailScaleGain = 0.0f;
    float LandDetailFade = 0.0f;
    uint32 LandDetailSeed = 0;
    // T5c: the dunes (VoxelLandDunes.ush; their chains: LandNoise's Dune* and Sand*), where the
    // climate (LandClimate: the land climate's cells and the sand's drift) makes desert.
    TSharedPtr<const FVoxelLandClimateField, ESPMode::ThreadSafe> LandClimate;   // null: none (no dunes)
    float LandClimateLapse = 6.5f;             // C per km
    float LandDuneTexelsPerCell = 0.0f;        // 0: no dunes
    float LandDuneScaleR = 1.0f;
    float LandDuneScaleG = 1.0f;
    float LandDuneMeanR = 0.0f;
    float LandDuneMeanG = 0.0f;
    float LandDuneRelief = 0.0f;               // m
    uint32 LandDuneSeed = 0;
    float LandDuneErgFrequency = 1.0f;
    float LandDuneErgThreshold = 2.0f;         // above 1: no ergs
    float LandDuneErgWidth = 0.06f;
    float LandDuneReliefMax = 400.0f;          // m
    float LandDuneDetailTexelsPerCell = 0.0f;  // 0: no sand detail
    float LandDuneDetailScale = 0.0f;
    float LandDuneDetailMean = 0.0f;
    float LandDuneDetailFade = 0.0f;
    // T5d: the glacial layer (VoxelLandPeaks.ush; its chain: LandNoise's Glacial*), in the peaks'
    // place where the climate (LandClimate) has had glaciers.
    float LandGlacialTexelsPerCell = 0.0f;     // 0: none
    float LandGlacialScaleR = 1.0f;
    float LandGlacialScaleG = 1.0f;
    float LandGlacialScaleB = 1.0f;
    float LandGlacialMeanR = 0.0f;
    float LandGlacialMeanG = 0.0f;
    float LandGlacialReliefM = 0.0f;
    float LandGlacialStrength = 0.0f;
    uint32 LandGlacialSeed = 0;
    float LandGlacialSpacingM = 0.0f;
    float LandGlacialCooling = 0.0f;
    float LandGlacialDetailScaleHard = 0.0f;
    float LandGlacialDetailScaleSoft = 0.0f;
    // T5d2: the ice sheets (VoxelLandIce.ush; their swells' chain: LandNoise's Ice*).
    float LandIceMaxThickness = 0.0f;          // m (0: none)
    bool bLandIceShelf = false;                // T5e: the planet has an ice shelf (the climate kept for its share)
    float LandIceBlend = 0.0f;                 // m
    float LandIceSwellTexelsPerCell = 0.0f;    // 0: no swells
    float LandIceMeanG = 0.0f;
    float LandIceSwellM = 0.0f;
    uint32 LandIceSeed = 0;
    float LandIceSwellSpacingM = 0.0f;
    // T5d3: the polar surfaces (VoxelLandPolar.ush; their chain: LandNoise's Polar*).
    uint32 LandPolarSeed = 0;
    float LandPolarCrevasseTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarCrevasseM = 0.0f;
    float LandPolarCrevasseMean = 0.0f;
    float LandPolarCrevasseFade = 0.0f;
    float LandPolarSastrugiTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarSastrugiM = 0.0f;
    float LandPolarSastrugiMean = 0.0f;
    float LandPolarSastrugiFade = 0.0f;
    float LandPolarPolygonTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarPolygonM = 0.0f;
    float LandPolarPolygonMean = 0.0f;
    float LandPolarPolygonFade = 0.0f;
    float LandPolarPermafrostT = -6.0f;    // C
    float LandPolarCrevasseTurn = 0.0f;    // radians: the set's grain
    float LandPolarSastrugiTurn = 0.0f;
    // The snow dunes: the dune set's G (its chain: LandNoise's Dune*).
    float LandPolarDuneTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarDuneM = 0.0f;
    float LandPolarDuneMean = 0.0f;
    float LandPolarDuneFade = 0.0f;

    TArray<uint32> LandFlowPacked;      // the bake's drainage, two f16 cells per word (empty: none)
    TArray<uint32> LandFormPacked;      // the bake's landform, two cells per word (empty: none)
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
     *  VoxelLand::ShapeFor(Land, *Bake, the noise pack). Climate (T5c): the land climate the
     *  dunes are placed by (null: no dunes). */
    VOXELPLUGIN_API TSharedPtr<FDensitySampleCompositor> MakeTerrestrialCompositor(const FVoxelTerrestrialSettings& Land,
        TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake, double PlanetRadius, double NoiseAmplitude,
        TSharedPtr<FSparseEditStore> EditStore, double MinFootprint = 0.0,
        TSharedPtr<const FVoxelLandClimateField, ESPMode::ThreadSafe> Climate = nullptr);

    /** Root half-extent of a planet's terrain lattice: holds every possible surface point. */
    inline double RootExtentFor(double PlanetRadius, double NoiseAmplitude) { return (PlanetRadius + NoiseAmplitude) * 1.05; }

    /** Cell size (cm) of the finest terrain lattice level (MaxDepth halvings of the root). */
    inline double TerrainFinestCell(double PlanetRadius, double NoiseAmplitude, int32 MaxDepth)
    {
        return 2.0 * RootExtentFor(PlanetRadius, NoiseAmplitude) / FMath::Pow(2.0, (double)MaxDepth);
    }
}
