// hook_emulation.h - The wave hook's FFT path (VoxelOceanWaves.ush) ported line for line to C++,
// over emulated cascade textures (VoxelOceanFFT.usf's outputs, mips averaged as MipCS, trilinear
// wrapping lookups) and the chart table; the shared code (charts, detiling, the sum, the normal)
// is the shaders' own. Used by hook_check.cpp and hook_view.cpp.
#pragma once
#include "UEStandIn/CoreMinimal.h"
#include "VoxelOceanWavesFFT.h"
#include "VoxelOceanWeather.h"
#include "VoxelOceanWeatherTable.h"
#include "ocean_fft_emulation.h"
#include "VoxelOceanShoreCore.h"
#include "VoxelOceanCoastCore.h"

namespace VoxelShader
{
#include "VoxelOceanFFTWaves.ush"
#include "VoxelOceanShoal.ush"
#include "VoxelOceanSurf.ush"
#include "VoxelOceanFeltDepth.ush"
}

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace HookEmulation
{
using namespace VoxelShader;
using OceanFFTEmulation::F4;

// ---------------------------------------------------------------------------------------------
// Emulated textures
// ---------------------------------------------------------------------------------------------

struct FArray
{
    // [slot][mip] texels, 256 >> mip square.
    std::vector<std::vector<std::vector<F4>>> Mips;

    void Build(int Slot, const std::vector<F4>& Top)
    {
        if ((int)Mips.size() <= Slot) Mips.resize(Slot + 1);
        Mips[Slot].clear();
        Mips[Slot].push_back(Top);
        for (int N = 128; N >= 1; N /= 2)
        {
            const std::vector<F4>& P = Mips[Slot].back();
            std::vector<F4> L(size_t(N) * N);
            for (int j = 0; j < N; j++)
                for (int i = 0; i < N; i++)
                {
                    auto At = [&](int a, int b) { return P[size_t(2 * j + b) * (2 * N) + 2 * i + a]; };
                    const F4 A = At(0, 0), B = At(1, 0), C = At(0, 1), D = At(1, 1);
                    L[size_t(j) * N + i] = { 0.25f * (A.x + B.x + C.x + D.x), 0.25f * (A.y + B.y + C.y + D.y), 0.25f * (A.z + B.z + C.z + D.z), 0.25f * (A.w + B.w + C.w + D.w) };
                }
            Mips[Slot].push_back(L);
        }
    }

    F4 Bilinear(int Slot, int Mip, float U, float V) const
    {
        const int N = 256 >> Mip;
        const std::vector<F4>& T = Mips[Slot][Mip];
        // Texel centres at (i + 0.5) / N.
        const float X = U * N - 0.5f, Y = V * N - 0.5f;
        const float FX = std::floor(X), FY = std::floor(Y);
        const float A = X - FX, B = Y - FY;
        const int I0 = ((int)FX % N + N) % N, J0 = ((int)FY % N + N) % N, I1 = (I0 + 1) % N, J1 = (J0 + 1) % N;
        auto At = [&](int i, int j) { return T[size_t(j) * N + i]; };
        const F4 P = At(I0, J0), Q = At(I1, J0), R = At(I0, J1), S = At(I1, J1);
        auto L = [&](float p, float q, float r, float s) { return (1 - B) * ((1 - A) * p + A * q) + B * ((1 - A) * r + A * s); };
        return { L(P.x, Q.x, R.x, S.x), L(P.y, Q.y, R.y, S.y), L(P.z, Q.z, R.z, S.z), L(P.w, Q.w, R.w, S.w) };
    }

    F4 SampleLevel(int Slot, float U, float V, float Lod) const
    {
        Lod = std::clamp(Lod, 0.0f, 8.0f);
        const int M0 = (int)Lod, M1 = std::min(M0 + 1, 8);
        const float T = Lod - M0;
        const F4 A = Bilinear(Slot, M0, U, V), B = Bilinear(Slot, M1, U, V);
        return { A.x + T * (B.x - A.x), A.y + T * (B.y - A.y), A.z + T * (B.z - A.z), A.w + T * (B.w - A.w) };
    }
};

struct FParams
{
    // FVoxelOceanWaveParams' FFT fields (VoxelOceanWaves.ush) and the weather's (Shape.y, z).
    FVector4f FFT, RefHigh, RefLow, Rotation, Tiles, States, SlopeVar, Shape;
    /** The checks' own: the long bands only (as the query, decision 6). */
    bool bLongOnly = false;
    /** The shore field's expansion around the reference (the chart table's rows 8-14). */
    FVoxelOceanShoreFrame Frame = VoxelOceanShoreFrameNone();
    /** The surf's settings (the chart table's rows 15 and 16). */
    FVector4f Surf0 = FVector4f(0.0f, 0.0f, 0.0f, 0.0f), Surf1 = FVector4f(0.0f, 0.0f, 0.0f, 0.0f), Surf2 = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    /** The basins' margin (m; the frame's row 9 z, stage O9b3). */
    float Margin = 0.0f;
};
inline FParams ParamsOf(const FVoxelOceanWaveShaderParams& P)
{
    FParams W { P.FFT, P.RefHigh, P.RefLow, P.Rotation, P.Tiles, P.States, P.SlopeVar, P.Shape, false, VoxelOceanShoreFrameNone() };
    float T[28];
    for (int i = 0; i < 7; i++) { T[4 * i] = P.ShoreFrame[i].X; T[4 * i + 1] = P.ShoreFrame[i].Y; T[4 * i + 2] = P.ShoreFrame[i].Z; T[4 * i + 3] = P.ShoreFrame[i].W; }
    W.Frame = VoxelOceanShoreCore::FrameFrom(T);
    W.Surf0 = P.Surf[0];
    W.Surf1 = P.Surf[1];
    W.Surf2 = P.Surf[2];
    W.Margin = P.ShoreFrame[1].Z;
    return W;
}

struct FTextures
{
    FArray Displacement, Derivatives;
    /** The whole table: charts, the weather's regions, the constants (a weather table's). */
    const VoxelOceanCharts::FTable* Charts = nullptr;
    /** The shore field's texels (stage O8; null: none) and its cells per face edge. */
    const uint32_t* Shore = nullptr;
    int ShoreN = 0;
    /** The coastal field's slices (stage O9h; null: none), and its layout (the chart table's rows
     *  19 to 21: VoxelOceanCoastCore::LayoutRows, the sea sphere's radius in m). */
    const uint32_t* Coast = nullptr;
    int CoastSlices = 0;
    float CoastRow0[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float CoastRow1[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float CoastRadiusM = 0.0f;

    FVoxelOceanCoastLayout CoastLayout() const
    {
        return VoxelOceanCoastLayoutFrom(float4(CoastRow0[0], CoastRow0[1], CoastRow0[2], CoastRow0[3]),
            float4(CoastRow1[0], CoastRow1[1], CoastRow1[2], CoastRow1[3]), CoastRadiusM);
    }
};

// The hook's FVoxelOceanShore (world axes; the checks' world is the planet frame rotated).
struct FShore
{
    float Distance = 1.0e30f;   // cm; VOXEL_OCEAN_DEEP: none
    float3 Direction = float3(0.0f, 0.0f, 0.0f);
    float Slope = 0.02f;
    float BedSlope = VOXEL_OCEAN_BED_UNKNOWN;    // stage O9b2
    float BedDepth = 1.0e30f;
    float3 BedRise = float3(0.0f, 0.0f, 0.0f);   // stage O9e (world axes, unit; zero unknown)
    FVoxelOceanCoastSample Coast = VoxelOceanCoastSampleNone();   // stage O9i (planet frame)
    float CoastSeedM = 0.0f;
    float CoastReachM = 0.0f;
};
constexpr float Deep = 1.0e30f;

// A constant wind everywhere as a weather table (headings exact, no steps): for the checks that
// look at one sea.
inline TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe> ConstantWeather(const FVoxelOceanWavesFFT& Model, double Wind, double Heading)
{
    FVoxelOceanEnvironment E;
    E.WindSpeed = Wind;
    E.WindHeading = Heading;
    const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> Source(MakeShared<FVoxelOceanConstantWeather, ESPMode::ThreadSafe>(E));
    FVoxelOceanWeatherTiming Timing;
    Timing.HeadingStepDegrees = 0.0;
    const FVoxelOceanWeatherKey Key = VoxelOceanWeatherTable::BuildKey(*Source, Model.GetCharts().N, Timing, 0);
    return VoxelOceanWeatherTable::MakeTable(Model.GetCharts(), Key, Key, Key, Model.GetTableConstants(), Source, Timing);
}

// ---------------------------------------------------------------------------------------------
// The hook's FFT path, ported from VoxelOceanWaves.ush
// ---------------------------------------------------------------------------------------------

inline float3 Rotate(const FVector4f& Q, float3 V)
{
    const float3 Qv(Q.X, Q.Y, Q.Z);
    const float3 T = cross(Qv, V) * 2.0f;
    return V + T * Q.W + cross(Qv, T);
}
inline float3 RotateBack(const FVector4f& Q, float3 V) { return Rotate(FVector4f(-Q.X, -Q.Y, -Q.Z, Q.W), V); }
inline float Comp(const FVector4f& V, int i) { return i == 0 ? V.X : (i == 1 ? V.Y : (i == 2 ? V.Z : V.W)); }

inline FVoxelOceanFFTSample Lookup(const FTextures& Tx, float U, float V, int Slice, float LodD, float LodN, unsigned Mask)
{
    FVoxelOceanFFTSample S = VoxelOceanFFTSampleZero();
    const F4 D = Tx.Displacement.SampleLevel(Slice, U, V, LodD);
    S.Displacement = float2(D.x * 100.0f, D.y * 100.0f);
    S.Height = D.z * 100.0f;
    if (Mask & 2u)
    {
        const F4 G = Tx.Derivatives.SampleLevel(Slice, U, V, LodN);
        S.Jxy = Tx.Displacement.SampleLevel(Slice, U, V, LodN).w;
        S.Slope = float2(G.x, G.y);
        S.Jxx = G.z;
        S.Jyy = G.w;
    }
    return S;
}

struct FBands { FVoxelOceanFFTSample Long, Short; };

inline FBands ChartLookups(const FTextures& Tx, FVoxelOceanChartCoords LongCoords, FVoxelOceanChartCoords ShortCoords, float2 Plane,
    const FVoxelOceanSeaState& Sea, const FParams& W, float Spacing, float Footprint, unsigned Mask,
    const FVoxelOceanShoal& Shoal, float2 LongShore, float2 ShortShore)
{
    FBands Out { VoxelOceanFFTSampleZero(), VoxelOceanFFTSampleZero() };
    const float Bias = W.RefHigh.W;
    const bool bShoal = Shoal.Active > 0.0f;
    for (int B = 0; B < 4; B++)
    {
        // The band's own lattice (stage O9b3).
        const FVoxelOceanDetile Detile = VoxelOceanDetileOfBand(Plane, W.FFT.Z, uint32_t(W.FFT.W), B, float4(W.Tiles.X, W.Tiles.Y, W.Tiles.Z, W.Tiles.W));
        const float Amp = bShoal ? VoxelOceanShoalComponent(Shoal.Amp, B) : 1.0f;
        if (Amp <= 0.0f) continue;
        const float Stretch = bShoal ? VoxelOceanShoalComponent(Shoal.Stretch, B) : 0.0f;
        const float L = Comp(W.Tiles, B);
        const float InvL = 1.0f / L;
        const float TexelsPerCm = 0.01f * 256.0f * InvL * (1.0f + Stretch);
        const float LodD = std::log2(std::max(2.0f * Spacing * TexelsPerCm, 1.0f)) + Bias;   // VOXEL_OCEAN_FFT_MESH_TEXEL (O9b2)
        const float LodN = std::log2(std::max(Footprint * TexelsPerCm, 1.0f)) + Bias;
        if (LodD >= 8.0f && (!(Mask & 2u) || LodN >= 8.0f)) continue;   // VOXEL_OCEAN_FFT_TOP_MIP
        const float Half = 0.5f * L / (B == 3 ? 512.0f : 256.0f);   // stage W3z: band 3 on the fine grid's texels
        const float2 E = B < 2 ? LongShore : ShortShore;
        const float Move = bShoal ? VoxelOceanShoalComponent(Shoal.Shift, B) : 0.0f;
        const float2 Warp(E.x * Move, E.y * Move);
        FVoxelOceanFFTSample Band = VoxelOceanFFTSampleZero();
        if (B < 2)
        {
            for (int K = 0; K < 3; K++)
            {
                const float WState = K == 0 ? Sea.Long.x : (K == 1 ? Sea.Long.y : Sea.Long.z);
                if (WState <= 0.0f) continue;
                for (int Copy = 0; Copy < 3; Copy++)
                {
                    if ((Copy == 1 && Detile.Weight.y <= 0.0f) || (Copy == 2 && Detile.Weight.z <= 0.0f)) continue;
                    const float2 Shift = Copy == 0 ? Detile.Offset0 : (Copy == 1 ? Detile.Offset1 : Detile.Offset2);
                    const float WCopy = Copy == 0 ? Detile.Weight.x : (Copy == 1 ? Detile.Weight.y : Detile.Weight.z);
                    const float U = VoxelOceanWrap(LongCoords.Base.x, LongCoords.Offset.x + Shift.x + Warp.x + Half, InvL);
                    const float V = VoxelOceanWrap(LongCoords.Base.y, LongCoords.Offset.y + Shift.y + Warp.y + Half, InvL);
                    Band = VoxelOceanFFTSampleMad(Band, WState * WCopy, Lookup(Tx, U, V, 2 * K + B, LodD, LodN, Mask));
                }
            }
        }
        else
        {
            const float Scale = B == 2 ? Sea.Short2 : Sea.Short3;
            if (Scale <= 0.0f || W.bLongOnly) continue;
            for (int Copy = 0; Copy < 3; Copy++)
            {
                if ((Copy == 1 && Detile.Weight.y <= 0.0f) || (Copy == 2 && Detile.Weight.z <= 0.0f)) continue;
                const float2 Shift = Copy == 0 ? Detile.Offset0 : (Copy == 1 ? Detile.Offset1 : Detile.Offset2);
                const float WCopy = Copy == 0 ? Detile.Weight.x : (Copy == 1 ? Detile.Weight.y : Detile.Weight.z);
                const float U = VoxelOceanWrap(ShortCoords.Base.x, ShortCoords.Offset.x + Shift.x + Warp.x + Half, InvL);
                const float V = VoxelOceanWrap(ShortCoords.Base.y, ShortCoords.Offset.y + Shift.y + Warp.y + Half, InvL);
                Band = VoxelOceanFFTSampleMad(Band, Scale * WCopy, Lookup(Tx, U, V, 4 + B, LodD, LodN, Mask));
            }
        }
        if (bShoal)
            Band = VoxelOceanShoalSample(Band, Amp, VoxelOceanShoalComponent(Shoal.Chop, B), Stretch, E);
        if (B < 2) Out.Long = VoxelOceanFFTSampleMad(Out.Long, 1.0f, Band);
        else Out.Short = VoxelOceanFFTSampleMad(Out.Short, 1.0f, Band);
    }
    return Out;
}

struct FPoint { FVoxelOceanFFTResult R; FVoxelOceanFFTAccum A; float4 SlopeVar; FVoxelOceanShoal Shoal; };

inline FVoxelOceanFFTAccum AddOrientation(FVoxelOceanFFTAccum A, const FTextures& Tx, const FVoxelOceanChart& C, FVoxelOceanChartCoords Coords,
    float2 LongWind, float2 ShortWind, float LongWeight, float ShortWeight, const FVoxelOceanSeaState& Sea, const FParams& W, float Spacing, float Footprint, unsigned Mask,
    const FVoxelOceanShoal& Shoal)
{
    const float2 Plane(Coords.Base.x + Coords.Offset.x, Coords.Base.y + Coords.Offset.y);
    FVoxelOceanSeaState Used = Sea;
    if (LongWeight <= 0.0f) Used.Long = float3(0.0f, 0.0f, 0.0f);
    if (ShortWeight <= 0.0f) { Used.Short2 = 0.0f; Used.Short3 = 0.0f; }
    const float2 ShorePlane(dot(Shoal.Dir, C.E1), dot(Shoal.Dir, C.E2));
    const FBands Bands = ChartLookups(Tx, VoxelOceanCoordsToWind(Coords, LongWind), VoxelOceanCoordsToWind(Coords, ShortWind), Plane, Used, W, Spacing, Footprint, Mask,
        Shoal, VoxelOceanToWind(ShorePlane, LongWind), VoxelOceanToWind(ShorePlane, ShortWind));
    A = VoxelOceanFFTAccumAdd(A, C, LongWind, LongWeight, Bands.Long);
    return VoxelOceanFFTAccumAdd(A, C, ShortWind, ShortWeight, Bands.Short);
}

// VoxelOceanShoreAt (ported): the field at P (from the reference) whose radial is Up (world), when
// the params carry one; near the reference through the frame's expansion. bExact: the CPU's
// double-normalized lookup instead (a reference for the checks).
inline FShore ShoreAt(const FTextures& Tx, float3 P, float3 Up, const FParams& W, bool bExact = false)
{
    FShore S;
    if (W.States.X < 0.5f || !Tx.Shore) return S;
    const float3 UpP = RotateBack(W.Rotation, Up);
    FVoxelOceanShoreSample F;
    if (bExact)
    {
        F = VoxelOceanShoreCore::Sample(Tx.Shore, int(W.States.X), W.SlopeVar.Z, UpP.x, UpP.y, UpP.z);
    }
    else
    {
        VoxelShader::FVoxelOceanShoreReader Reader;
        Reader.Texels = Tx.Shore;
        Reader.Side = VoxelOceanShoreCore::Side(int(W.States.X));
        F = VoxelOceanShoreFieldNear(Reader, UpP, int(W.States.X), W.SlopeVar.Z, W.Frame, RotateBack(W.Rotation, P));
    }
    S.Distance = F.Distance * 100.0f;
    S.Direction = Rotate(W.Rotation, F.Direction);
    S.Slope = F.Slope;
    // The coastal field (stage O9i), within its reach.
    const FVoxelOceanCoastLayout L = Tx.CoastLayout();
    S.CoastSeedM = L.SeedDepthM;
    S.CoastReachM = L.ReachM;
    if (Tx.Coast && L.Valid > 0.5f && F.Distance < L.ReachM + 2.0f * L.CellM)
    {
        if (bExact)
        {
            S.Coast = VoxelOceanCoastCore::Sample(Tx.Shore, Tx.Coast, Tx.CoastSlices, int(W.States.X), Tx.CoastRow0, Tx.CoastRow1, Tx.CoastRadiusM, UpP.x, UpP.y, UpP.z);
        }
        else
        {
            VoxelShader::FVoxelOceanShoreReader Reader;
            Reader.Texels = Tx.Shore;
            Reader.Side = VoxelOceanShoreCore::Side(int(W.States.X));
            Reader.Coast = Tx.Coast;
            Reader.CoastSlices = Tx.CoastSlices;
            S.Coast = VoxelOceanCoastNear(Reader, L, UpP, W.Frame, RotateBack(W.Rotation, P));
        }
    }
    return S;
}

// VoxelOceanFFTShoalAt (ported).
inline FVoxelOceanShoal ShoalAt(float Depth, const FShore& Shore, const FVoxelOceanSeaState& Sea, float4 Sigma, float LongKeep, const FParams& W)
{
    FVoxelOceanShoalSettings Settings = VoxelOceanShoalSettingsOf(float4(W.States.X, W.States.Y, W.States.Z, W.States.W), float4(W.SlopeVar.X, W.SlopeVar.Y, W.SlopeVar.Z, W.SlopeVar.W));
    Settings.LongKeep = LongKeep;
    // The waves' floor (stage W3g; the table's row 18 z).
    Settings.Floor = std::min(std::max(W.Surf2.Z, 0.0f), 1.0f);
    if (Settings.Breaking <= 0.0f) return VoxelOceanShoalNone();
    const bool bField = W.States.X > 0.5f && Shore.Distance < 0.5f * Deep;
    FVoxelOceanShoreSample Field;
    Field.Distance = bField ? Shore.Distance * 0.01f : 1.0e30f;
    Field.Slope = Shore.Slope;
    Field.Direction = bField ? RotateBack(W.Rotation, Shore.Direction) : float3(0.0f, 0.0f, 0.0f);
    Field.Weight = 1.0f;
    const float DepthM = Depth < 0.5f * Deep ? Depth * 0.01f : 1.0e30f;
    return VoxelOceanShoalOf(DepthM, Field, bField, Sea, float4(W.Tiles.X, W.Tiles.Y, W.Tiles.Z, W.Tiles.W), Sigma, Settings);
}

// VoxelOceanFFTSurfAt (ported).
struct FSurfAt { FVoxelOceanSurf Surf; float3 Dir; float3 BedDir; };

// VoxelOceanBedRiseOf (ported).
inline float3 BedRiseOf(const FShore& Shore, const FParams& W, float3 Landward)
{
    const float L2 = dot(Shore.BedRise, Shore.BedRise);
    return L2 > 0.25f ? RotateBack(W.Rotation, Shore.BedRise) * (1.0f / std::sqrt(L2)) : Landward;
}
inline FSurfAt SurfAt(float3 D, float Depth, const FShore& Shore, const FVoxelOceanSeaState& Sea, float4 Sigma, const FParams& W, float Spacing, float Footprint,
    unsigned Mask)
{
    FSurfAt Out { VoxelOceanSurfNone(), float3(0.0f, 0.0f, 0.0f), float3(0.0f, 0.0f, 0.0f) };
    const bool bField = W.States.X > 0.5f && Shore.Distance < 0.5f * Deep;
    if (!bField || W.States.Y <= 0.0f || Depth >= 0.5f * Deep) return Out;
    const FVoxelOceanSurfSettings Settings = VoxelOceanSurfSettingsOf(float4(W.Surf0.X, W.Surf0.Y, W.Surf0.Z, W.Surf0.W),
        float4(W.Surf1.X, W.Surf1.Y, W.Surf1.Z, W.Surf1.W), float4(W.Surf2.X, W.Surf2.Y, W.Surf2.Z, W.Surf2.W), W.States.Y);
    if (Settings.Strength <= 0.0f) return Out;
    const float3 DirP = RotateBack(W.Rotation, Shore.Direction);
    const float Agree = dot(DirP, DirP);
    FVoxelOceanSurfInput In;
    In.DepthM = Depth * 0.01f;
    In.Slope = std::max(Shore.Slope, std::max(W.SlopeVar.Y, 1.0e-4f));
    In.ShoreM = Shore.Distance * 0.01f;
    In.Hs = 4.0f * std::sqrt(Sigma.x * Sigma.x + Sigma.y * Sigma.y);
    In.Speed = std::max(Sea.Wind, Sea.Swell);
    In.Units = int3(int(W.RefHigh.X * (1.0f / VOXEL_OCEAN_SURF_UNIT)), int(W.RefHigh.Y * (1.0f / VOXEL_OCEAN_SURF_UNIT)), int(W.RefHigh.Z * (1.0f / VOXEL_OCEAN_SURF_UNIT)));
    In.Low = float3(W.RefLow.X, W.RefLow.Y, W.RefLow.Z) + D;
    In.Time = W.SlopeVar.W;
    In.SpacingM = Spacing * 0.01f;
    In.FootprintM = Footprint * 0.01f;
    In.BedSlope = Shore.BedSlope;
    In.BedDepthM = Shore.BedDepth < 0.5f * Deep ? Shore.BedDepth * 0.01f : 1.0e30f;
    In.ForNormals = (Mask & 2u) != 0u ? 1.0f : 0.0f;
    In.CoastA = Shore.Coast.A;
    In.CoastB = Shore.Coast.B;
    In.CoastGradA = Shore.Coast.GradA;
    In.CoastGradB = Shore.Coast.GradB;
    In.CoastCover = Shore.Coast.Cover;
    In.CoastSeedM = Shore.CoastSeedM;
    In.CoastReachM = Shore.CoastReachM;
    Out.Surf = VoxelOceanSurfOf(In, Settings);
    Out.Dir = Agree > 1.0e-12f ? DirP * (1.0f / std::sqrt(Agree)) : float3(0.0f, 0.0f, 0.0f);
    Out.BedDir = BedRiseOf(Shore, W, Out.Dir);
    return Out;
}

inline FPoint HookEvaluate(const FTextures& Tx, float3 P, float3 Up, float Spacing, float Footprint, const FParams& W, unsigned Mask,
    float SmoothDepth = Deep, const FShore& Shore = FShore())
{
    // Stage W5e: the felt depth (VoxelOceanSurfFeltDepth), as the hook.
    const float Depth = VoxelOceanSurfFeltDepth(SmoothDepth, Shore.BedDepth, 100.0f * VOXEL_OCEAN_SURF_FELT_BLEND);
    const float3 D = RotateBack(W.Rotation, P);
    const float3 UpP = RotateBack(W.Rotation, Up);
    const int N = std::max(int(W.FFT.Y), 1);
    const FVoxelOceanTableReader Table = Tx.Charts->Reader();
    const FVoxelOceanSeaState Sea = VoxelOceanSeaStateAt(Table, UpP, N, W.Shape.Y);
    const float4 Sigma = VoxelOceanShoalBandSigma(VoxelOceanSlotSigma(Table, N, 0), VoxelOceanSlotSigma(Table, N, 1), Sea);
    const FSurfAt Surf = SurfAt(D, Depth, Shore, Sea, Sigma, W, Spacing, Footprint, Mask);
    const FVoxelOceanShoal Shoal = ShoalAt(Depth, Shore, Sea, Sigma, Surf.Surf.LongKeep, W);
    const float Fade = std::clamp(W.Shape.Z, 0.0f, 1.0f);
    const FVoxelOceanChartCandidates Cand = VoxelOceanChartCandidatesOf(UpP, N);
    const FVoxelOceanChartBlend Blend = VoxelOceanChartBlendOf(Cand.Count, VoxelOceanChartDistancesOf(Cand, UpP), N);
    FVoxelOceanFFTAccum A = VoxelOceanFFTAccumZero();
    for (int M = 0; M < VOXEL_OCEAN_CHART_MAX; M++)
    {
        if (M >= Blend.Count) continue;
        const int2 T = Cand.Texel[Blend.Index[M]];
        const VoxelOceanCharts::FChartCPU CC = VoxelOceanCharts::ChartAt(*Tx.Charts, T.x, T.y);
        const FVoxelOceanChart C = CC.Chart;
        const FVoxelOceanChartHeadings H = VoxelOceanChartHeadingsAt(Table, T);
        const FVoxelOceanChartCoords Coords = VoxelOceanChartCoordsOf(C, float3(W.RefHigh.X, W.RefHigh.Y, W.RefHigh.Z), float3(W.RefLow.X, W.RefLow.Y, W.RefLow.Z), D);
        const float PX = Coords.Base.x + Coords.Offset.x, PY = Coords.Base.y + Coords.Offset.y;
        const float Distance = std::sqrt(PX * PX + PY * PY);
        const float2 WLong = VoxelOceanTurnWeights(Fade, Distance, H.LongPrevious, H.LongCurrent, VoxelOceanLongCorrelationLength(Sea));
        const float2 WShort = VoxelOceanTurnWeights(Fade, Distance, H.ShortPrevious, H.ShortCurrent, VOXEL_OCEAN_SHORT_CORRELATION_LENGTH);
        if (WLong.x > 0.0f || WShort.x > 0.0f)
            A = AddOrientation(A, Tx, C, Coords, H.LongCurrent, H.ShortCurrent, Blend.Weight[M] * WLong.x, Blend.Weight[M] * WShort.x, Sea, W, Spacing, Footprint, Mask, Shoal);
        if (WLong.y > 0.0f || WShort.y > 0.0f)
            A = AddOrientation(A, Tx, C, Coords, H.LongPrevious, H.ShortPrevious, Blend.Weight[M] * WLong.y, Blend.Weight[M] * WShort.y, Sea, W, Spacing, Footprint, Mask, Shoal);
    }
    // The surf's height, gradient and rate (cm), as the hook.
    if (Surf.Surf.Active > 0.5f)
    {
        A.Height += Surf.Surf.Height * 100.0f;
        A.Gradient = A.Gradient + Surf.Dir * Surf.Surf.Gradient + Surf.BedDir * Surf.Surf.GradientBed + Surf.Surf.GradientPhase;
        A.VelocityUp += Surf.Surf.Rate * 100.0f;
    }
    // The drain toward inland basins (stage O9b3), as the hook.
    if (W.States.X > 0.5f && W.States.Y > 0.0f && Shore.Distance < 0.5f * Deep)
    {
        const float Bed = Shore.BedSlope > VOXEL_OCEAN_SURF_BED_KNOWN ? Shore.BedSlope : Shore.Slope;
        const FVoxelOceanSurfDrain Drain = VoxelOceanSurfDrainOf(Shore.Distance * 0.01f, Depth < 0.5f * Deep ? Depth * 0.01f : 1.0e30f, Bed, W.Margin);
        if (Drain.Keep < 1.0f)
        {
            const float3 DirP = RotateBack(W.Rotation, Shore.Direction);
            const float L2 = dot(DirP, DirP);
            // (Stage W3g: the waves kept, the surface sunk.)
            A.Height -= Drain.Drop * 100.0f;
            const float3 DrainDir = L2 > 1.0e-12f ? DirP * (1.0f / std::sqrt(L2)) : float3(0.0f, 0.0f, 0.0f);
            A.Gradient = A.Gradient - DrainDir * Drain.Slope - BedRiseOf(Shore, W, DrainDir) * Drain.SlopeBed;
        }
    }
    // Thin water (stage O9f), as the hook.
    if (Depth < 0.5f * Deep && W.States.X > 0.5f && W.States.Y > 0.0f && Shore.Distance < 0.5f * Deep)
    {
        const float Bottom = Shore.BedDepth < 0.5f * Deep ? std::min(Depth, Shore.BedDepth) : Depth;
        A.Height -= 100.0f * VoxelOceanSurfThinDrop((A.Height + Bottom) * 0.01f, Spacing * 0.01f);
    }
    // Each band's slope variance, shoaled as its slopes are (amplitude and stretch), as the hook.
    float4 SlopeVar(0, 0, 0, 0);
    if (Mask & 2u)
    {
        const float4 V = VoxelOceanBandSlopeVariance(Table, Sea, N);
        const float4 G(Shoal.Amp.x * (1.0f + Shoal.Stretch.x), Shoal.Amp.y * (1.0f + Shoal.Stretch.y),
            Shoal.Amp.z * (1.0f + Shoal.Stretch.z), Shoal.Amp.w * (1.0f + Shoal.Stretch.w));
        SlopeVar = float4(V.x * G.x * G.x, V.y * G.y * G.y, V.z * G.z * G.z, V.w * G.w * G.w);
    }
    return { VoxelOceanFFTFinish(A, UpP), A, SlopeVar, Shoal };
}

// VoxelOceanSphereHeight (VoxelOceanCommon.ush), ported: the stable height above the sphere.
inline float SphereHeight(float3 D, float Radius, float CameraHeight, float3 Up)
{
    const float A = Radius + CameraHeight;
    const float Mu = dot(Up, D);
    const float Numerator = CameraHeight * (2.0f * Radius + CameraHeight) + 2.0f * A * Mu + dot(D, D);
    const float Denominator = length(Up * A + D) + Radius;
    return Numerator / std::max(Denominator, 1.0f);
}

// VoxelOceanHeightAtFFT (ported): Q from the reference (world axes); Centre the planet centre from
// it; CameraHeight the reference's height above the sphere (formed in double, as the pass does).
// Mask: the last lookup's (3 all, 1 the displacement only); the steps use M = I + J from the
// foot, then from the first step's point, then keep it (the shader's steps).
// Footprint: the waves' mip footprint (cm; 0 unfaded, as the underwater pass's old search; the
// surface map passes its texel).
// Depth: the terrain's depth (cm) at Q's foot; DepthOf (optional) the depth at each step's point
// (X from the reference: VOXEL_OCEAN_HEIGHT_DEPTH, the underwater pass's probe), else Depth held;
// the shore is read again at every step's point (VoxelOceanHeightAtFFTShore). Depth Deep and no
// field: VoxelOceanHeightAtFFTMask.
inline float HookHeightAt(const FTextures& Tx, float3 Q, float3 Centre, double Radius, double CameraHeight, const FParams& W, FPoint& Out, int Steps = 3, unsigned Mask = 3u, float Footprint = 0.0f,
    float Depth = Deep, const std::function<float(float3)>& DepthOf = nullptr)
{
    const float Height = SphereHeight(Q, float(Radius), float(CameraHeight), normalize(Centre * -1.0f));
    const float3 Up = normalize(Q - Centre);
    const float3 Foot = Q - Up * Height;
    float3 X = Foot;
    FPoint Pt = HookEvaluate(Tx, X, normalize(X - Centre), Footprint, Footprint, W, 3u, Depth, ShoreAt(Tx, X, normalize(X - Centre), W));
    FVoxelOceanFFTAccum J = Pt.A;
    float Trace = J.J11 + J.J22 + J.J33;
    float InvDet = 1.0f / std::max(Pt.R.Jacobian, 0.05f);
    for (int Step = 0; Step < Steps; Step++)
    {
        const float3 Horizontal = Rotate(W.Rotation, Pt.R.Displacement);
        float3 R = X + Horizontal - Foot;
        R = R - Up * dot(Up, R);
        const float3 RP = RotateBack(W.Rotation, R);
        const float3 StepP = (RP * (1.0f + Trace) - VoxelOceanFFTApplyJ(J, RP)) * InvDet;
        X = X - Rotate(W.Rotation, StepP);
        const float StepDepth = DepthOf ? DepthOf(X) : Depth;
        Pt = HookEvaluate(Tx, X, normalize(X - Centre), Footprint, Footprint, W, Step == Steps - 1 ? Mask : (Step == 0 ? 3u : 1u), StepDepth, ShoreAt(Tx, X, normalize(X - Centre), W));
        if (Step == 0 && Step < Steps - 1)
        {
            J = Pt.A;
            Trace = J.J11 + J.J22 + J.J33;
            InvDet = 1.0f / std::max(Pt.R.Jacobian, 0.05f);
        }
    }
    Out = Pt;
    return Height - Pt.R.Height;
}

}
