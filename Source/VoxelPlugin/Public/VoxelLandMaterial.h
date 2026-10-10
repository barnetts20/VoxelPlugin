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
    VOXELPLUGIN_API extern const FName RefWorldHighName;       // VoxelLandRefWorldHigh (vector, world cm)
    VOXELPLUGIN_API extern const FName RefWorldLowName;        // VoxelLandRefWorldLow
    VOXELPLUGIN_API extern const FName RefLocalHighName;       // VoxelLandRefLocalHigh (vector, planet frame cm)
    VOXELPLUGIN_API extern const FName RefLocalLowName;        // VoxelLandRefLocalLow
    VOXELPLUGIN_API extern const FName AxisXName;              // VoxelLandAxisX (vector: the planet frame's X in world)
    VOXELPLUGIN_API extern const FName AxisYName;              // VoxelLandAxisY
    VOXELPLUGIN_API extern const FName AxisZName;              // VoxelLandAxisZ
    VOXELPLUGIN_API extern const FName ParamsName;             // VoxelLandParams (x view, y tolerance m, z enabled, w tolerance growth)

    /** The field texels of a bake (Side x 6 Side, the bake's cell order). Any thread. */
    VOXELPLUGIN_API void BuildFieldTexels(const FVoxelLandBake& Bake, TArray<FFloat16Color>& Out);

    /** The constant texels (VOXEL_LAND_MATERIAL_CONST_WIDTH x OutRows) for a density's land
     *  bake (Density.LandMode must be 1). Any thread. */
    VOXELPLUGIN_API void BuildConstantTexels(const FVoxelDensityGPUData& Density, const FVoxelLandBake& Bake, TArray<FVector4f>& Out, int32& OutRows);

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

    /** Game thread. The textures on a material instance. */
    VOXELPLUGIN_API void SetTextureParameters(UMaterialInstanceDynamic* Material, UTexture2DDynamic* Height, UTexture2DDynamic* Fields,
        UTexture2DDynamic* Constants);

    /** Game thread, every frame: the reference point (a world position near the camera) in
     *  world and in the planet's frame (PlanetFrame: the planet's location and rotation, no
     *  scale), the frame's axes, the debug view and the surface tolerance (m). bEnabled false
     *  turns the node off (it then reads no texture and leaves the mesh's normal). */
    VOXELPLUGIN_API void SetFrameParameters(UMaterialInstanceDynamic* Material, const FTransform& PlanetFrame, const FVector& ReferenceWorld,
        int32 View, double Tolerance, bool bEnabled);
}
