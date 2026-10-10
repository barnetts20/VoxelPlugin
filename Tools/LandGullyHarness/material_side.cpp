// The material half of material_harness.cpp: the shared land code compiled through
// VoxelLandMaterialAccess.ush (texture loads, a constants texture), as VoxelLandMaterial.ush
// includes it, with stand-ins for HLSL's Texture2D and float4.
#include "VoxelShaderCompat.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "material_side.h"

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
#include "VoxelDF64.ush"
#include "VoxelCubeSphere.ush"

    struct float4
    {
        float x, y, z, w;
        float4() : x(0), y(0), z(0), w(0) {}
        float4(float X, float Y, float Z, float W) : x(X), y(Y), z(Z), w(W) {}
    };

    /** A texture as the material sees it: Load of texel (x, y, mip), four channels. */
    struct Texture2D
    {
        const float* Data = nullptr;
        int Width = 0, Height = 0;
        float4 Load(int3 P) const
        {
            if (P.x < 0 || P.y < 0 || P.x >= Width || P.y >= Height || P.z != 0) { fprintf(stderr, "texel out of range %d %d %d\n", P.x, P.y, P.z); exit(2); }
            const float* T = Data + 4 * ((size_t)P.y * Width + P.x);
            return float4(T[0], T[1], T[2], T[3]);
        }
    };

#include "VoxelLandMaterialAccess.ush"
#include "VoxelLandSample.ush"
#include "VoxelLandGully.ush"
#include "VoxelLandPlates.ush"
#include "VoxelDensitySample.ush"
#include "VoxelLandDebugPalette.ush"
#include "VoxelLandDebugColor.ush"

    static uint AttrAt(Texture2D LandHeightTex, Texture2D LandFieldTex, Texture2D LandConstTex, uint I) { return VOXEL_LAND_ATTR(I); }
    static float WaterAt(Texture2D LandHeightTex, Texture2D LandFieldTex, Texture2D LandConstTex, uint I) { return VOXEL_LAND_WATER(I); }
    static float CellMOf(Texture2D LandConstTex) { return LandCellM; }
#include "VoxelLandMaterialAccessEnd.ush"
}
using namespace VoxelShader;

static void Textures(const FMaterialTextures& T, Texture2D& H, Texture2D& F, Texture2D& C)
{
    H = Texture2D{ T.Height, T.Side, 6 * T.Side };
    F = Texture2D{ T.Fields, T.Side, 6 * T.Side };
    C = Texture2D{ T.Constants, VOXEL_LAND_MATERIAL_CONST_WIDTH, T.Rows };
}

FMaterialHeight MaterialLandHeight(const FMaterialTextures& T, const float X[2], const float Y[2], const float Z[2], float Footprint, bool bGradient)
{
    Texture2D H, F, C; Textures(T, H, F, C);
    const FVoxelLandHeight L = VoxelLandHeightDF(H, F, C, float2(X[0], X[1]), float2(Y[0], Y[1]), float2(Z[0], Z[1]), Footprint, bGradient);
    FMaterialHeight O;
    O.Height[0] = L.Height.x; O.Height[1] = L.Height.y;
    O.Bake = L.Bake; O.Gully = L.Gully; O.Detail = L.Detail; O.GullyScale = L.GullyScale; O.Flow = L.Flow; O.DetailScale = L.DetailScale;
    O.BakeGradient[0] = L.BakeGradient.x; O.BakeGradient[1] = L.BakeGradient.y; O.BakeGradient[2] = L.BakeGradient.z;
    O.Warped[0] = L.Warped.x; O.Warped[1] = L.Warped.y; O.Warped[2] = L.Warped.z;
    return O;
}

uint32_t MaterialAttr(const FMaterialTextures& T, uint32_t Cell) { Texture2D H, F, C; Textures(T, H, F, C); return AttrAt(H, F, C, Cell); }
float MaterialWater(const FMaterialTextures& T, uint32_t Cell) { Texture2D H, F, C; Textures(T, H, F, C); return WaterAt(H, F, C, Cell); }

void MaterialOctave(const FMaterialTextures& T, uint32_t I, void* Out)
{
    Texture2D H, F, C; Textures(T, H, F, C);
    const FVoxelOctave O = VoxelLandMaterialOctave(C, I);
    static_assert(sizeof(FVoxelOctave) == 80, "FVoxelOctave layout");
    std::memcpy(Out, &O, 80);
}

void MaterialDebugColor(const FMaterialTextures& T, uint32_t Mode, const FMaterialHeight& L, float CellM, float Out[3])
{
    Texture2D H, F, C; Textures(T, H, F, C);
    FVoxelLandDebugInput In;
    In.Mode = Mode; In.Elevation = L.Bake + L.Gully; In.SlopeDegrees = 12.0f; In.CellM = CellM;
    In.Warped = float3(L.Warped[0], L.Warped[1], L.Warped[2]);
    In.Bake = L.Bake; In.Gully = L.Gully; In.GullyScale = L.GullyScale;
    In.DetailNorm = L.Detail / 250.0f;
    In.bTerrestrial = true; In.bHydro = true;
    const float3 Color = VoxelLandDebugColor(H, F, C, In);
    Out[0] = Color.x; Out[1] = Color.y; Out[2] = Color.z;
    (void)CellMOf;
}
