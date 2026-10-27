// VoxelLandMaterial.h - The terrestrial generator's data in the terrain material
// (Design/TerrainMaterial.md): the bake and the density's land constants as three textures the
// material's land node reads (Shaders/Private/VoxelLandMaterial.ush, layout
// VoxelLandMaterialLayout.ush), and the per-frame parameters that place the pixel in the planet's
// frame. The terrain actor owns the textures and the dynamic material instance.

#pragma once

#include "CoreMinimal.h"
#include "Math/Float16Color.h"

class FVoxelDensityCPU;
struct FVoxelDensityGPUData;
struct FVoxelLandBake;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class UTexture2DDynamic;

namespace VoxelLandMaterial
{
    /** The material's parameter names (the land node's inputs; STAGE_M1_NOTES.md). */
    VOXELPLUGIN_API extern const FName HeightTextureName;      // VoxelLandHeight (texture)
    VOXELPLUGIN_API extern const FName FieldTextureName;       // VoxelLandFields (texture)
    VOXELPLUGIN_API extern const FName ConstantTextureName;    // VoxelLandConstants (texture)
    VOXELPLUGIN_API extern const FName NoiseTextureName;       // VoxelLandNoise (texture: the peaks' noise set, T3)
    VOXELPLUGIN_API extern const FName OriginHighName;         // VoxelLandOriginHigh (vector: the planet's origin in world, cm)
    VOXELPLUGIN_API extern const FName OriginLowName;          // VoxelLandOriginLow (its double-float remainder)
    VOXELPLUGIN_API extern const FName AxisXName;              // VoxelLandAxisX (vector: the planet frame's X in world)
    VOXELPLUGIN_API extern const FName AxisYName;              // VoxelLandAxisY
    VOXELPLUGIN_API extern const FName AxisZName;              // VoxelLandAxisZ
    VOXELPLUGIN_API extern const FName AxisXLowName;           // VoxelLandAxisXLow (the axes' double-float remainders)
    VOXELPLUGIN_API extern const FName AxisYLowName;           // VoxelLandAxisYLow
    VOXELPLUGIN_API extern const FName AxisZLowName;           // VoxelLandAxisZLow
    VOXELPLUGIN_API extern const FName ParamsName;             // VoxelLandParams (x view, y tolerance m, z enabled, w near footprint cm)

    /** Stage T5a: the climate node's parameters (VoxelLandClimateMaterial.ush). */
    VOXELPLUGIN_API extern const FName ClimateTextureName;     // VoxelLandClimate (texture: the climate map)
    VOXELPLUGIN_API extern const FName ClimateParamsName;      // VoxelLandClimateParams (x on + 2 view, y sea radius m, z lapse C/km, w season)
    VOXELPLUGIN_API extern const FName ClimateShapeName;       // VoxelLandClimateShape (x aspect C, y breakup C, z breakup m, w snow width C)
    VOXELPLUGIN_API extern const FName ClimateTintName;        // VoxelLandClimateTint (x snow, y glacier, z desert, w plants)

    /** The field texels of a bake (Side x 6 Side, the bake's cell order). Any thread. */
    VOXELPLUGIN_API void BuildFieldTexels(const FVoxelLandBake& Bake, TArray<FFloat16Color>& Out);

    /** The height texture's texels (RG32F, VoxelLandMaterialLayout.ush): the height, and the
     *  landform's 16 bits plus the local relief's 8 above them, as a float (exact: 24 bits;
     *  0 without them). */
    VOXELPLUGIN_API void BuildHeightTexels(const FVoxelLandBake& Bake, TArray<FVector2f>& Out);

    /** The constant texels (VOXEL_LAND_MATERIAL_CONST_WIDTH x OutRows) for a density's land
     *  bake (Density.LandMode must be 1). Any thread. */
    VOXELPLUGIN_API void BuildConstantTexels(const FVoxelDensityGPUData& Density, const FVoxelLandBake& Bake, TArray<FVector4f>& Out, int32& OutRows);

    /** voxel.Land.MaterialSource 0: the node reads the land surface cache (VoxelLandCache.h). */
    VOXELPLUGIN_API bool UsesCache();

    /** voxel.Debug.LandCache: the node shows the cache's levels. */
    VOXELPLUGIN_API bool ShowsCacheLevels();

    /** Stage T5a: voxel.Debug.LandClimate, the climate node's debug view (0 none). */
    VOXELPLUGIN_API int32 ClimateView();

    /** Stage T5a, game thread: the climate map's texture (RGBA16F, VoxelLandClimate.ush's layout),
     *  empty until UploadClimateTexture fills it. Null when it can't be made. */
    VOXELPLUGIN_API UTexture2DDynamic* MakeClimateTexture();

    /** Stage T5a, game thread: the cells (x the year's mean at sea level, y the precipitation, z the
     *  swing, w the continentality; VOXEL_LAND_CLIMATE_WIDTH x HEIGHT) into the texture. */
    VOXELPLUGIN_API void UploadClimateTexture(UTexture2DDynamic* Texture, const TArray<FVector4f>& Cells);

    /** Stage T5a, game thread: the climate node's parameters on a material instance (Params.x is
     *  set here: 1 when Texture is given, plus 2 x the debug view; 0 turns the node off), and the
     *  texture when bBindTexture (the per-frame update leaves it bound). */
    VOXELPLUGIN_API void SetClimateParameters(UMaterialInstanceDynamic* Material, UTexture2DDynamic* Texture, const FLinearColor& Params,
        const FLinearColor& Shape, const FLinearColor& Tint, bool bBindTexture = true);

    /** Whether a material has the land node: its VoxelLandParams vector parameter. */
    VOXELPLUGIN_API bool HasLandNode(const UMaterialInterface* Material);

    /** Game thread. The height and field textures of a bake (they depend on nothing else, so a
     *  rebuilt density with the same bake keeps them): dynamic textures filled on the render
     *  thread, from the bake's own heights (held until the upload ran), so no CPU copy stays.
     *  False (and null) when the RHI can't hold them. */
    VOXELPLUGIN_API bool MakeBakeTextures(const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& Bake,
        UTexture2DDynamic*& OutHeight, UTexture2DDynamic*& OutFields);

    /** Game thread. The constants texture of a terrestrial density (null for the noise surface). */
    VOXELPLUGIN_API UTexture2DDynamic* MakeConstantTexture(const FVoxelDensityCPU& Density);

    /** Game thread. The peaks' noise set as a texture (RGBA16 unsigned normalised: its packed mip
     *  chain in rows of N texels, VoxelLandNoise.ush), one for every planet sharing the set, or a
     *  1 x 1 stand-in when the density has none (the constants then turn the peaks off). */
    VOXELPLUGIN_API UTexture2DDynamic* MakeNoiseTexture(const FVoxelDensityCPU& Density);

    /** Game thread. The textures on a material instance. */
    VOXELPLUGIN_API void SetTextureParameters(UMaterialInstanceDynamic* Material, UTexture2DDynamic* Height, UTexture2DDynamic* Fields,
        UTexture2DDynamic* Constants, UTexture2DDynamic* Noise);

    /** Game thread, every frame: the planet's frame (PlanetFrame: its location and rotation, no
     *  scale) as its origin and axes (double-float), the debug view, the surface tolerance
     *  (m) and the footprint (cm) below which the mesh's normal is used (NearFootprint). The material places each pixel from its own view, so no camera is needed here.
     *  bEnabled false turns the node off (it then reads no texture and leaves the mesh's
     *  normal); bCacheAvailable false makes it evaluate the land itself (the analytic node)
     *  whatever voxel.Land.MaterialSource asks. */
    VOXELPLUGIN_API void SetFrameParameters(UMaterialInstanceDynamic* Material, const FTransform& PlanetFrame,
        int32 View, double Tolerance, double NearFootprint, bool bEnabled, bool bCacheAvailable);
}
