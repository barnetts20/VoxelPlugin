// Check of VoxelOcean.ush's pixel entry points (the ocean material's Custom node), with
// stand-ins for the material template: the O4 signature, the FFT one, the shore's (O8) and the
// foam's (O9b), with the bed (O9b2).
#define PIXELSHADER 1
struct FMaterialVertexParameters { float4x4 LocalToWorld; float VertexOnly; };
struct FMaterialPixelParameters { float4x4 LocalToWorld; float2 PixelOnly; };
float3 TransformLocalVectorToWorld(FMaterialVertexParameters P, float3 V) { return mul(V, (float3x3)P.LocalToWorld); }
float3 TransformLocalVectorToWorld(FMaterialPixelParameters P, float3 V) { return mul(V, (float3x3)P.LocalToWorld); }
#include "VoxelOcean.ush"
Texture2DArray Disp; SamplerState DispSampler; Texture2DArray Der; SamplerState DerSampler; Texture2D Ch; Texture2DArray Sh; Texture2DArray Fm; SamplerState FmSampler; Texture2DArray CD; Texture2DArray CN; SamplerState CNSampler; Texture2D CI;
float4 Prm; float4 W0; float4 W1; float4 W2; float3 Ctr; float4 Shp;
float4 FFTp; float4 RH; float4 RL; float4 Rot; float4 Til; float4 Sta; float4 SV;
float4 main(float4 Pos : SV_Position, float3 SamplePoint : TEXCOORD0, float Depth : TEXCOORD1, float2 Bed : TEXCOORD2) : SV_Target
{
    FMaterialPixelParameters P;
    P.LocalToWorld = (float4x4)0;
    P.PixelOnly = 0;
    float Var; float Var2; float Var3; float Var4; float Foam;
    const float3 N = VoxelOceanPixelFFT(P, SamplePoint, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch, Var);
    const float3 N2 = VoxelOceanPixel(P, SamplePoint, Prm, W0, W1, W2, Ctr, Shp, Var2);
    // Stage O8: with the shore field and the interpolated depth.
    const float3 N3 = VoxelOceanPixelFFT(P, SamplePoint, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch, Sh, Depth, Var3);
    // Stage O9b: with the foam.
    const float3 N4 = VoxelOceanPixelFFT(P, SamplePoint, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch, Sh, Fm, FmSampler, Depth, Var4, Foam);
    // Stage O9b2: with the bed.
    float Var5; float Foam5;
    const float3 N5 = VoxelOceanPixelFFT(P, SamplePoint, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch, Sh, Fm, FmSampler, Depth, Bed, Var5, Foam5);
    // Stage O9k: the wave clipmap's pixel.
    float Var6; float Foam6;
    const float3 N6 = VoxelOceanPixelClip(P, SamplePoint, Ctr, Rot, CD, CN, CNSampler, CI, Var6, Foam6);
    // Stage O9l: with the detail the clipmap can't hold.
    float Var7; float Foam7;
    const float3 N7 = VoxelOceanPixelClipDetail(P, SamplePoint, Prm, W0, W1, W2, Ctr, Shp, FFTp, RH, RL, Rot, Til, Sta, SV, Disp, DispSampler, Der, DerSampler, Ch,
        CD, CN, CNSampler, CI, Depth, Var7, Foam7);
    return float4(N + N2 + N3 + N4 * Foam + N5 * Foam5 + N6 * Foam6 + N7 * Foam7, Var + Var2 + Var3 + Var4 + Var5 + Var6 + Var7);
}
