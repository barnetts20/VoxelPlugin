// glslang check of VoxelLandMaterial.ush with stand-ins for the material template.
struct FMaterialPixelParameters { float3x3 TangentToWorld; float3 WorldPosition_CamRelative; };
struct FDFVector3 { float3 High; float3 Low; };
struct ViewState { FDFVector3 PreViewTranslation; };
cbuffer ViewCB { float3 PvtHigh; float3 PvtLow; };
static ViewState ResolvedView;
#include "VoxelLandMaterial.ush"
// The input macros end with the include: the names are free again.
static float LandN = 1; static float PlanetRadius = 2; static float DetailOctaves = 3; static float LandCellM = 4;
Texture2D HeightT;
Texture2D FieldT;
Texture2D ConstT;
Texture2D NoiseT;
Texture2D CH; Texture2D CS; Texture2D CF; Texture2D CSt;
float3 OHigh; float3 OLow; float3 AX; float3 AY; float3 AZ; float3 AXL; float3 AYL; float3 AZL; float4 Prm;
float4 main(float4 Pos : SV_Position, float3 TW : TEXCOORD0, float3 N : TEXCOORD1) : SV_Target
{
    ResolvedView.PreViewTranslation.High = PvtHigh;
    ResolvedView.PreViewTranslation.Low = PvtLow;
    FMaterialPixelParameters P;
    P.TangentToWorld = float3x3(float3(1, 0, 0), float3(0, 1, 0), N);
    P.WorldPosition_CamRelative = TW;
    float H; float3 NW; float3 NT; float4 F; float4 T; float M;
    const float3 C = VoxelLandMaterial(P, HeightT, FieldT, ConstT, NoiseT, CH, CS, CF, CSt, OHigh, OLow, AX, AY, AZ, AXL, AYL, AZL, Prm, H, NW, NT, F, T, M);
    return float4(C + NW + NT * M, H + F.x + T.w + LandN + PlanetRadius + DetailOctaves + LandCellM);
}
