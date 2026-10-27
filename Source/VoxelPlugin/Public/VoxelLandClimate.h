// VoxelLandClimate.h - The land's climate (stage T5a, Design/LandClimate.md): a map on the weather
// sim's grid (Shaders/Private/VoxelLandClimate.ush has its layout) built from the land bake and
// the atmosphere's climate (or a provisional model until the sim's has settled), shown by the
// terrain material's climate node (VoxelLandClimateMaterial.ush) as snow, glacier ice, desert and
// plant cover, and queried on the CPU with the same code (APlanetActor::GetClimateAt).

#pragma once

#include "CoreMinimal.h"
#include "VoxelLandClimate.generated.h"

struct FVoxelLandBake;
struct FAtmosphereClimateMap;
struct FVoxelLandShelfParams;
struct FVoxelTerrestrialSettings;

/** Where the land's climate comes from (stage T5b: one snapshot per seed, Design/LandClimate.md). */
UENUM(BlueprintType)
enum class EVoxelLandClimateSource : uint8
{
    /** The provisional model only: the latitude's temperature and a zonal rain profile, drier
     *  inland. Needs no atmosphere. */
    Provisional,
    /** The atmosphere's climate snapshot: on spawn or a seed change the atmosphere's sim spins up
     *  and averages a climate window (CloudAtmosphere's UFlowSimConfig::bClimateSnapshot), and the
     *  planet captures that once and keeps it for the seed. The biomes, and the terrain they shape,
     *  never change after it. The provisional model where no snapshot can come (no atmosphere, or
     *  another planet drives the sim). */
    Snapshot,
};

/** The land's climate: where it comes from and how it shows (APlanetActor::LandClimate). */
USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelLandClimateSettings
{
    GENERATED_BODY()

    /** The climate on the land: the terrain material's climate node gets its map (off: the node
     *  leaves the base colour alone). Terrestrial planets. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate")
    bool bEnabled = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate")
    EVoxelLandClimateSource Source = EVoxelLandClimateSource::Snapshot;

    /** Snapshot: the planet is built once its climate is captured (the terrain then never changes
     *  for the seed: the biomes shape it). Off: built at once on the provisional model, the snapshot
     *  fading in when it lands. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate", meta = (EditCondition = "Source == EVoxelLandClimateSource::Snapshot"))
    bool bWaitForClimate = true;

    /** Snapshot: the longest the capture waits (seconds) before the provisional model is kept
     *  instead. The atmosphere's spin-up and climate window take a second or two of frames; a
     *  planet whose atmosphere doesn't drive the sim gives up after 3 s. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate", meta = (ClampMin = "1.0", UIMax = "60.0", EditCondition = "Source == EVoxelLandClimateSource::Snapshot"))
    float ClimateWaitSeconds = 15.0f;

    /** The land's median precipitation, mm a year (Earth's land: about 600 to 700). The sim's
     *  precipitation, in its own units, is scaled to it; so is the provisional model's. A drier
     *  planet has more desert, a wetter one more plant cover and lower glaciers. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Map", meta = (ClampMin = "0.0", UIMax = "3000.0"))
    float LandPrecipitation = 650.0f;

    /** The spread of the precipitation about the median, as an exponent: above 1 wetter wet lands
     *  and drier deserts, below 1 more even. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Map", meta = (ClampMin = "0.05", UIMax = "2.0"))
    float PrecipitationContrast = 1.0f;

    /** The seasons' swing, 1 Earth's (its axial tilt): the summer's mean temperature above the
     *  year's, from about 1 C at the equator to 8 C at the poles at the coast and twice that deep
     *  inland. Decides how far glaciers and the season's snow reach. 0: no seasons. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Map", meta = (ClampMin = "0.0", UIMax = "3.0"))
    float Seasonality = 1.0f;

    /** Stage T5d2: C, the most the sim's weather may warm or cool the land's sea-level temperature
     *  (its standing warm ridges and cold troughs, ClimateWarmthScale x its warmth), soft-clamped.
     *  The part of the sim's warmth that only follows its terrain (its air thinning over high
     *  ground) is removed first: the land's lapse rate already cools by the height. 0: no clamp. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Map", meta = (ClampMin = "0.0", UIMax = "30.0"))
    float MaxWarmthAnomaly = 8.0f;

    /** Seconds a new climate fades in over the old (a recapture, a settings change, or the
     *  snapshot landing after a build that didn't wait for it). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Map", meta = (ClampMin = "0.0", UIMax = "30.0"))
    float FadeSeconds = 4.0f;

    /** The season shown, the northern hemisphere's (the south has the opposite): 0 the summer
     *  (glaciers and the summer's snow only), 0.5 the year's mean, 1 the winter (snow wherever the
     *  winter is cold enough and some precipitation falls). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Look", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float Season = 0.15f;

    /** C: how much warmer a slope facing the sun (toward the equator) is than a flat one, at
     *  90 degrees of slope (scaled by the slope's sine; none at the equator, full beyond 30 degrees
     *  of latitude). Snow lingers on the shaded side. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Look", meta = (ClampMin = "0.0", UIMax = "6.0"))
    float AspectWarming = 2.0f;

    /** C: the breakup noise's amplitude (it moves the snow line, the desert's edge and the plants'
     *  limits about, so no edge follows a contour exactly). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Look", meta = (ClampMin = "0.0", UIMax = "5.0"))
    float BreakupAmplitude = 1.5f;

    /** m: the breakup noise's scale (a second octave at 0.31 of it). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Look", meta = (ClampMin = "1.0", UIMax = "1000.0"))
    float BreakupScale = 60.0f;

    /** C: half the width of the snow line's ramp (with the lapse rate of 6.5 C/km, 1.5 C is about
     *  230 m of height either side). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Look", meta = (ClampMin = "0.05", UIMax = "5.0"))
    float SnowWidth = 1.5f;

    /** How strongly the climate node tints the base colour with each cover (0 leaves it to the
     *  graph, which has the node's Cover output). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Tint", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float SnowTint = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Tint", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float GlacierTint = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Tint", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float DesertTint = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Climate|Tint", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float VegetationTint = 0.0f;

    /** A hash of what the map depends on here (the look's settings go to the material only). */
    uint32 GetMapHash() const;
};

/** The climate map on the game thread: the cells (VOXEL_LAND_CLIMATE_WIDTH x HEIGHT,
 *  VoxelLandClimate.ush's layout: x the year's mean at sea level, y the precipitation, z the swing,
 *  w the continentality). */
struct VOXELPLUGIN_API FVoxelLandClimateMap
{
    TArray<FVector4f> Cells;

    bool IsValid() const;

    /** Bilinear at a planet-local unit direction, as the material reads it. */
    FVector4f Sample(const FVector& Dir) const;
};

/** Stage T5c: the climate as the land chain reads it (VoxelLandClimateAccess.ush's LandClimate): the
 *  map's cells, then per cell the sand's drift (x east, y north: the resultant drift's unit
 *  direction, or zero; z its steadiness, 0..1; w 0), then (T5d2) the ice sheets (x the signed
 *  distance into the ice, m, positive inside; y the land's mean height, m; z, w 0). Immutable once made: the
 *  density that placed its layers by it holds it (the terrain doesn't change with the climate shown
 *  after that; Design/LandClimate.md). Its GPU copy is uploaded on first use and released on the
 *  render thread. */
struct VOXELPLUGIN_API FVoxelLandClimateField
{
    FVoxelLandClimateField();
    ~FVoxelLandClimateField();
    FVoxelLandClimateField(const FVoxelLandClimateField&) = delete;
    FVoxelLandClimateField& operator=(const FVoxelLandClimateField&) = delete;

    TArray<FVector4f> Entries;   // 3 x Width x Height
    float LapseRate = 6.5f;      // C per km (the planet's ClimateLapseRate)
    uint32 Hash = 0;             // of the entries and the lapse rate
    bool bDriftFromSim = false;  // the drift's source: the sim's (else the provisional winds)
    float IceMaxThickness = 0.0f;  // T5d2: above 0 when it holds the ice block (the cap is the density's)

    bool IsValid() const;

    /** The GPU copy (render thread). */
    TSharedPtr<struct FVoxelLandClimateGPU, ESPMode::ThreadSafe> GPU;
};

/** The climate at a point (VoxelLandClimateAt). */
struct FVoxelLandClimatePointResult
{
    float Temperature = 0.0f;     // C: the year's mean
    float Summer = 0.0f;          // C: the summer's mean
    float Now = 0.0f;             // C: the season's (FVoxelLandClimateSettings::Season)
    float Precipitation = 0.0f;   // mm a year
    float Aridity = 0.0f;         // log2 of Holdridge's PET ratio
    float Snow = 0.0f;
    float Glacier = 0.0f;
    float Desert = 0.0f;
    float Vegetation = 0.0f;
};

namespace VoxelLandClimate
{
    /** The map's width and height (VOXEL_LAND_CLIMATE_WIDTH and HEIGHT). */
    VOXELPLUGIN_API int32 Width();
    VOXELPLUGIN_API int32 Height();

    /** Any thread: the land's share per cell (4 x 4 points of the bake each, not sea-connected)
     *  and the distance to the sea (m) on a planet of SeaRadiusM; OutHeight (T5d2) the land
     *  points' mean height above sea level (m; 0 without land). */
    VOXELPLUGIN_API void BuildBase(const FVoxelLandBake& Bake, double SeaRadiusM, TArray<float>& OutLand, TArray<float>& OutCoast,
        TArray<float>* OutHeight = nullptr);

    /** Any thread: the map, from the base and the sim's climate (null: the provisional model), the
     *  settings and the planet's latitude curve (C at sea level: the equator's, the poles', the
     *  |sin(latitude)| exponent between) and warmth scale. True when the sim's climate was used
     *  (it can't be when its precipitation is all zero). LandHeight (T5d2; BuildBase's, empty: none):
     *  the sim's warmth loses the part that follows the ground's height. Shelf (T5e; null: none):
     *  the ice shelf's share in w (VoxelLandShelf.h; ShelfParamsFor). */
    VOXELPLUGIN_API bool BuildMap(const TArray<float>& Land, const TArray<float>& Coast, const FAtmosphereClimateMap* Sim,
        const FVoxelLandClimateSettings& Settings, double SeaRadiusM, float EquatorTemperature, float PoleTemperature, float LatitudeExponent,
        float WarmthScale, FVoxelLandClimateMap& Out, const TArray<float>* LandHeight = nullptr, const FVoxelLandShelfParams* Shelf = nullptr);

    /** Any thread (T5c): the land chain's climate from a map (BuildMap's) and the sim's climate it
     *  was made from (null: the provisional winds); LapseRate C per km. T5d2: the ice sheets from
     *  BuildBase's land share and mean height (empty: none), at most IceMaxThickness (m; 0: none).
     *  Null if the map isn't valid. */
    VOXELPLUGIN_API TSharedPtr<const FVoxelLandClimateField, ESPMode::ThreadSafe> MakeField(const FVoxelLandClimateMap& Map,
        const FAtmosphereClimateMap* Sim, float LapseRate, const TArray<float>& Land, const TArray<float>& LandHeight, double SeaRadiusM,
        float IceMaxThickness);

    /** T5d2: the ice share's ramp half-width (C) about the glacier's line, per cell (as the land
     *  chain's VOXEL_LAND_GLACIAL_WIDTH). */
    constexpr float IceWidth = 2.0f;

    /** Any thread (T5c): the ergs' threshold (VoxelLandClimateErgNoise at Frequency) above which
     *  Coverage (0..1) of the sphere lies: 2 for none, -2 for all. */
    VOXELPLUGIN_API float ErgThreshold(float Frequency, float Coverage);

    /** T5e: the ice shelf's settings for a planet (off without IceShelfHeight), its seed the plate
     *  model's (FVoxelLandBake::PlateSeed). */
    VOXELPLUGIN_API FVoxelLandShelfParams ShelfParamsFor(const FVoxelTerrestrialSettings& Settings, uint32 Seed);

    /** T5e, any thread: the bake the terrain is built with, the polar ice shelf raised in Base's
     *  heights by the climate (the field's map: its year's mean at sea level) and ShelfParamsFor:
     *  under it the hills, the strata, the drainage and the lakes give way (its form flattened),
     *  and the peaks' relief at its front; the sea mask, the relief and the ranges made again, the
     *  rivers Base's (they give way on the shelf in the chain), a key of its own. Null when there's
     *  no shelf (Base is the terrain's then). A fraction of a second at 1024 cells a face. */
    VOXELPLUGIN_API TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> MakeIceShelfBake(const FVoxelLandBake& Base,
        const FVoxelLandClimateField& Field, const FVoxelTerrestrialSettings& Settings);

    /** The climate at a planet-frame point (m from the centre) with a surface normal (planet frame,
     *  unit): the material's own computation (VoxelLandClimateAt), breakup included. */
    VOXELPLUGIN_API FVoxelLandClimatePointResult PointAt(const FVoxelLandClimateMap& Map, const FVoxelLandClimateSettings& Settings, float LapseRate,
        const FVector& LocalM, const FVector& NormalLocal, double SeaRadiusM);
}
