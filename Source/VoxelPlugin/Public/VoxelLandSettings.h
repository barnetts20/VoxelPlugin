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

    // --- Bake: grid ---

    /** Cells per cube face edge: 1024 (1.5 km on a 1,000 km planet, 9.8 km on Earth), or 512 or
     *  2048 for development (memory and bake time scale with its square). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Bake", meta = (ClampMin = "64", ClampMax = "2048"))
    int32 BakeResolution = 1024;

    // --- Sample: per-sample layers (no rebake) ---

    /** Sampling warp of the bake, in bake cells: hides the grid's straight lines. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", ClampMax = "4.0"))
    double BakeWarp = 1.5;

    /** Slope-damped fBm detail on top of the bake: amplitude (m), its longest wavelength (m),
     *  and how strongly accumulated slope suppresses finer octaves. A quarter as strong under
     *  the sea. Placeholder for the hillside layers of stage L4. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "1000"))
    double DetailAmplitude = 250.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "100.0", UIMin = "1000", UIMax = "100000"))
    double DetailWavelength = 20000.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrestrial|Sample", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double DetailSlopeDamping = 1.0;

    /** Hash of the settings that change the bake (with the radius and the generator version,
     *  the cache key). */
    uint32 GetBakeHash() const;

    /** Hash of every field, for change detection. */
    uint32 GetSettingsHash() const;
};
