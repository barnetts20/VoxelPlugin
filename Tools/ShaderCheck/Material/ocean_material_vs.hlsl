// Check of VoxelOcean.ush's vertex entry points (the ocean material's Custom node), with
// stand-ins for the material template: the O4 signature, the FFT one, the shore's (O8) and the
// bed's (O9b2).
struct FMaterialVertexParameters { float4x4 LocalToWorld; float VertexOnly; };
struct FMaterialPixelParameters { float4x4 LocalToWorld; float2 PixelOnly; };
float3 TransformLocalVectorToWorld(FMaterialVertexParameters P, float3 V) { return mul(V, (float3x3)P.LocalToWorld); }
float3 TransformLocalVectorToWorld(FMaterialPixelParameters P, float3 V) { return mul(V, (float3x3)P.LocalToWorld); }
#include "VoxelOcean.ush"
Texture2DArray Disp; SamplerState DispSampler; Texture2DArray Der; SamplerState DerSampler; Texture2D Ch; Texture2DArray Sh;
float4 Prm; float4 W0; float4 W1; float4 W2; float3 Ctr; float4 Shp;
float4 FFTp; float4 RH; float4 RL; float4 Rot; float4 Til; float4 Sta; float4 SV; float4x4 L2W; float Lvl; float ME;
float4 main(float3 Pos : POSITION, float2 U0 : TEXCOORD0, float2 U1 : TEXCOORD1, float2 U2 : TEXCOORD2, float2 U3 : TEXCOORD3, float2 U4 : TEXCOORD4, float2 U5 : TEXCOORD5) : SV_Position
{
    FMaterialVertexParameters P;
    P.LocalToWorld = L2W;
    P.VertexOnly = 0;
    float Depth; float3 SP; float Depth2; float3 SP2; float Depth3; float3 SP3;
    const float3 O = VoxelOceanVertexFFT(P, Pos, U0, U1, U2, U3, U4, Lvl, ME, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch, Depth, SP);
    const float3 O2 = VoxelOceanVertex(P, Pos, U0, U1, U2, U3, U4, Lvl, ME, Prm, W0, W1, W2, Ctr, Shp, Depth2, SP2);
    // Stage O8: with the shore field.
    const float3 O3 = VoxelOceanVertexFFT(P, Pos, U0, U1, U2, U3, U4, Lvl, ME, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch, Sh, Depth3, SP3);
    // Stage O9b2: with the bed (TexCoord 5 in, Bed out).
    float Depth4; float2 Bed4; float3 SP4;
    const float3 O4 = VoxelOceanVertexFFT(P, Pos, U0, U1, U2, U3, U4, U5, Lvl, ME, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch, Sh, Depth4, Bed4, SP4);
    return float4(O + O2 + O3 + O4 + SP + SP2 + SP3 + SP4, Depth + Depth2 + Depth3 + Depth4 + Bed4.x + Bed4.y);
}
