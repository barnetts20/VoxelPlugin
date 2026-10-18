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
#ifdef VOXEL_FASTMATH_TEST
    // HLSL's precise under fast math (the material's compile): the double-float steps exact,
    // everything else as a fast-math compiler may rearrange it.
#undef VOXEL_FUNC
#define VOXEL_FUNC __attribute__((optimize("no-fast-math"), noinline))
#endif
#include "VoxelDF64.ush"
#ifdef VOXEL_FASTMATH_TEST
#undef VOXEL_FUNC
#define VOXEL_FUNC inline
#endif
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

static void Textures(const FMaterialTextures& T, Texture2D& H, Texture2D& F, Texture2D& C, Texture2D& N)
{
    H = Texture2D{ T.Height, T.Side, 6 * T.Side };
    F = Texture2D{ T.Fields, T.Side, 6 * T.Side };
    C = Texture2D{ T.Constants, VOXEL_LAND_MATERIAL_CONST_WIDTH, T.Rows };
    N = Texture2D{ T.Noise, T.NoiseN, T.NoiseRows };
}

FMaterialHeight MaterialLandHeight(const FMaterialTextures& T, const float X[2], const float Y[2], const float Z[2], float Footprint, bool bGradient)
{
    Texture2D H, F, C, N; Textures(T, H, F, C, N);
    const FVoxelLandHeight L = VoxelLandHeightDF(H, F, C, N, float2(X[0], X[1]), float2(Y[0], Y[1]), float2(Z[0], Z[1]), Footprint, bGradient);
    FMaterialHeight O;
    O.Height[0] = L.Height.x; O.Height[1] = L.Height.y;
    O.Bake = L.Bake; O.Gully = L.Gully; O.Detail = L.Detail; O.GullyScale = L.GullyScale; O.Flow = L.Flow; O.DetailScale = L.DetailScale;
    O.BakeGradient[0] = L.BakeGradient.x; O.BakeGradient[1] = L.BakeGradient.y; O.BakeGradient[2] = L.BakeGradient.z;
    O.Warped[0] = L.Warped.x; O.Warped[1] = L.Warped.y; O.Warped[2] = L.Warped.z;
    O.DetailGradient[0] = L.DetailGradient.x; O.DetailGradient[1] = L.DetailGradient.y; O.DetailGradient[2] = L.DetailGradient.z;
    O.DetailUnit = L.DetailUnit; O.DetailAmplitudeM = L.DetailAmplitudeM;
    O.Hills = L.Hills; O.HillUnit = L.HillUnit; O.HillAmplitudeM = L.HillAmplitudeM;
    O.HillGradient[0] = L.HillGradient.x; O.HillGradient[1] = L.HillGradient.y; O.HillGradient[2] = L.HillGradient.z;
    O.Peaks = L.Peaks; O.PeakUnit = L.PeakUnit; O.PeakAmplitudeM = L.PeakAmplitudeM;
    O.PeakGradient[0] = L.PeakGradient.x; O.PeakGradient[1] = L.PeakGradient.y; O.PeakGradient[2] = L.PeakGradient.z;
    O.Carve = L.Carve; O.CarveScale = L.CarveScale;
    O.CarveGradient[0] = L.CarveGradient.x; O.CarveGradient[1] = L.CarveGradient.y; O.CarveGradient[2] = L.CarveGradient.z;
    {
        // The lake's weight there (through the warp, as the chain finds it).
        const FVoxelLandWarp W = VoxelLandWarpDF(H, F, C, N, float2(X[0], X[1]), float2(Y[0], Y[1]), float2(Z[0], Z[1]));
        O.LakeWeight = VoxelLandLakeAt(H, F, C, N, W.QX, W.QY, W.QZ, false).Weight;
    }
    return O;
}

uint32_t MaterialAttr(const FMaterialTextures& T, uint32_t Cell) { Texture2D H, F, C, N; Textures(T, H, F, C, N); return AttrAt(H, F, C, Cell); }
float MaterialWater(const FMaterialTextures& T, uint32_t Cell) { Texture2D H, F, C, N; Textures(T, H, F, C, N); return WaterAt(H, F, C, Cell); }

void MaterialOctave(const FMaterialTextures& T, uint32_t I, void* Out)
{
    Texture2D H, F, C, N; Textures(T, H, F, C, N);
    const FVoxelOctave O = VoxelLandMaterialOctave(C, I);
    static_assert(sizeof(FVoxelOctave) == 80, "FVoxelOctave layout");
    std::memcpy(Out, &O, 80);
}

FMaterialRiver MaterialRiver(const FMaterialTextures& T, const float X[2], const float Y[2], const float Z[2])
{
    Texture2D H, F, C, N; Textures(T, H, F, C, N);
    const FVoxelLandRiver R = VoxelLandRiverAtDF(H, F, C, N, float2(X[0], X[1]), float2(Y[0], Y[1]), float2(Z[0], Z[1]));
    return { R.bFound, R.Distance, R.Signed, R.Floor, R.Log2Area, R.S, R.Segment };
}

void MaterialDebugColor(const FMaterialTextures& T, uint32_t Mode, const FMaterialHeight& L, float CellM, float Out[3])
{
    Texture2D H, F, C, N; Textures(T, H, F, C, N);
    FVoxelLandDebugInput In;
    In.Mode = Mode; In.Elevation = L.Bake + L.Gully; In.SlopeDegrees = 12.0f; In.CellM = CellM;
    In.Warped = float3(L.Warped[0], L.Warped[1], L.Warped[2]);
    In.Bake = L.Bake; In.Gully = L.Gully; In.GullyScale = L.GullyScale; In.Hills = L.Hills;
    In.Peaks = L.Peaks; In.PeakAmplitude = L.PeakAmplitudeM;
    In.DetailNorm = L.Detail / 250.0f;
    In.bTerrestrial = true; In.bHydro = true;
    const float3 Color = VoxelLandDebugColor(H, F, C, N, In);
    Out[0] = Color.x; Out[1] = Color.y; Out[2] = Color.z;
    (void)CellMOf;
}

void MaterialFramePoint(const float TW[3], const float PH[3], const float PL[3], const float OH[3], const float OL[3],
    const float AX[3], const float AY[3], const float AZ[3], const float AXL[3], const float AYL[3], const float AZL[3], double Out[3])
{
    auto V = [](const float* A) { return float3(A[0], A[1], A[2]); };
    const FVoxelLandFramePoint P = VoxelLandFramePoint(V(TW), V(PH), V(PL), V(OH), V(OL), V(AX), V(AY), V(AZ), V(AXL), V(AYL), V(AZL));
    Out[0] = (double)P.X.x + P.X.y; Out[1] = (double)P.Y.x + P.Y.y; Out[2] = (double)P.Z.x + P.Z.y;
}
