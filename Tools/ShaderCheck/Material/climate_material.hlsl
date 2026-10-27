// glslang/DXC check of VoxelLandClimateMaterial.ush beside the land node (both Custom nodes in one
// material, as a terrain graph has them), with stand-ins for the material template.
struct FMaterialPixelParameters { float3x3 TangentToWorld; float3 WorldPosition_CamRelative; };
struct FDFVector3 { float3 High; float3 Low; };
struct ViewState { FDFVector3 PreViewTranslation; };
cbuffer ViewCB { float3 PvtHigh; float3 PvtLow; };
static ViewState ResolvedView;
#include "VoxelLandMaterial.ush"
#include "VoxelLandClimateMaterial.ush"
Texture2D HeightT;
Texture2D FieldT;
Texture2D ConstT;
Texture2D NoiseT;
Texture2D ClimateT;
Texture2D CH; Texture2D CS; Texture2D CF; Texture2D CSt;
float3 OHigh; float3 OLow; float3 AX; float3 AY; float3 AZ; float3 AXL; float3 AYL; float3 AZL; float4 Prm;
float4 CPrm; float4 CShape; float4 CTint;
float4 main(float4 Pos : SV_Position, float3 TW : TEXCOORD0, float3 N : TEXCOORD1, float3 Base : TEXCOORD2) : SV_Target
{
    ResolvedView.PreViewTranslation.High = PvtHigh;
    ResolvedView.PreViewTranslation.Low = PvtLow;
    FMaterialPixelParameters P;
    P.TangentToWorld = float3x3(float3(1, 0, 0), float3(0, 1, 0), N);
    P.WorldPosition_CamRelative = TW;
    float H; float3 NW; float3 NT; float4 F; float4 T; float M;
    const float3 C = VoxelLandMaterialCached(P, HeightT, FieldT, ConstT, NoiseT, CH, CS, CF, CSt, OHigh, OLow, AX, AY, AZ, AXL, AYL, AZL, Prm, H, NW, NT, F, T, M);
    float4 Climate; float4 Cover;
    const float3 B = VoxelLandClimateMaterial(P, ClimateT, OHigh, OLow, AX, AY, AZ, NW, lerp(Base, C, M), CPrm, CShape, CTint, Climate, Cover);
    return float4(B + Cover.xyz, Climate.x + Climate.w + Cover.w + H);
}
