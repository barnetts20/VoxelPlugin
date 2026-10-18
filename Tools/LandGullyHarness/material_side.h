// The material harness's two halves (material_harness.cpp): the land code compiled with the
// material's accessors (material_side.cpp) and with the global shaders' (gully_harness.cpp) can't
// share a translation unit (their functions without land inputs would be defined twice), so the
// material half answers through plain types.
#pragma once
#include <cstdint>

struct FMaterialTextures
{
    const float* Height = nullptr;   // four floats a texel (as the material loads them), Side x 6 Side
    const float* Fields = nullptr;   // the same
    const float* Constants = nullptr; // VOXEL_LAND_MATERIAL_CONST_WIDTH x Rows
    const float* Noise = nullptr;    // four unit values a texel (an RGBA16 load), NoiseN x NoiseRows
    int Side = 0, Rows = 0, NoiseN = 1, NoiseRows = 1;
};

struct FMaterialHeight
{
    float Height[2];
    float Bake, Gully, Detail, GullyScale, Flow, DetailScale;
    float BakeGradient[3], Warped[3];
    float DetailGradient[3], DetailUnit, DetailAmplitudeM;
    float Hills, HillUnit, HillAmplitudeM, HillGradient[3];
    float Peaks, PeakUnit, PeakAmplitudeM, PeakGradient[3];
    float Carve, CarveScale, CarveGradient[3];   // R2
    float LakeWeight;                            // R2: the lake's weight at the point (the material's half-precision depths differ there)
};

FMaterialHeight MaterialLandHeight(const FMaterialTextures& T, const float X[2], const float Y[2], const float Z[2], float Footprint, bool bGradient);
uint32_t MaterialAttr(const FMaterialTextures& T, uint32_t Cell);
float MaterialWater(const FMaterialTextures& T, uint32_t Cell);
/** Octave I as the material decodes it: FVoxelOctave's 80 bytes. */
void MaterialOctave(const FMaterialTextures& T, uint32_t I, void* Out);
/** VoxelLandFramePoint: the pixel in the planet's frame (as doubles: the double-float sums). */
void MaterialFramePoint(const float TranslatedWorld[3], const float PreViewHigh[3], const float PreViewLow[3], const float OriginHigh[3],
    const float OriginLow[3], const float AxisX[3], const float AxisY[3], const float AxisZ[3], const float AxisXLow[3],
    const float AxisYLow[3], const float AxisZLow[3], double Out[3]);
/** The nearest river (VoxelLandRiverAtDF through the material's accessors): found, distance,
 *  signed distance, floor, log2 area, s, segment. */
struct FMaterialRiver { bool bFound; float Distance, Signed, Floor, Log2Area, S; uint32_t Segment; };
FMaterialRiver MaterialRiver(const FMaterialTextures& T, const float X[2], const float Y[2], const float Z[2]);

/** View Mode's colour (sRGB) at the surface H (the debug input as VoxelLandMaterial.ush builds it). */
void MaterialDebugColor(const FMaterialTextures& T, uint32_t Mode, const FMaterialHeight& H, float CellM, float Out[3]);
