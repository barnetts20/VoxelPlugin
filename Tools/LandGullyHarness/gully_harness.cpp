// L4 harness: the shipped VoxelLandSample.ush + VoxelLandGully.ush (and VoxelDensitySample.ush
// for the GPU path) compiled as C++, on eroded bakes dumped by solve_dump (the L2 solve at
// 6 x 1024^2). Looks (hillshade close-ups with and without gullies), slope bounds, face-edge
// continuity, LOD band changes, cost, and the GPU code path against the CPU path.
//
// g++ -O2 -std=c++20 -ffp-contract=off -pthread -I<Public> -I<Shaders/Private> gully_harness.cpp
//
// gully_harness Tag N RadiusM Mode: Tag_*.f32 from `solve_harness N Tag dump RadiusM`. Modes:
//   look   hillshade sheets Tag_lookV_Wkm.ppm (without gullies, with, the gully term)
//   slope  slope distributions (bake, gullies, totals; the gully term at forced slopes)
//   edges  continuity across cube-face edges and corners against interior lines
//   lod    what a level change (footprint F to F/2) moves
//   turn   the downhill direction's turn over the coarsest wavelength
//   probe  the sharpest kinks along a row of the first look view, by octave count
//   cost   CPU time per land sample with and without gullies
//   agree  the GPU code path (VoxelSampleDensityDF) against the CPU path
//   noise  the noise set's sampler against the handoff's B-spline in doubles (NOISE_SET=file.vxnt
//          for an authored set; every mode reads it)
//   pcost  the peaks' CPU cost and (built with -DVOXEL_HARNESS_COUNT_NOISE) noise loads a sample
//   cgrad  the peaks' gradient against differences with the real (varying) landform
//   dgrad  the detail layer's gradient (N1: VoxelLandDetailDF) against differences
//   dcost  the detail layer's CPU cost against the procedural fBm it replaced (and noise loads)
//   seam   the set layers' face blend near cube edges and corners (N1b): variance, range, gradients
// The detail set: DETAIL_SET=file.vxnt (an authored detail set), else the generated stand-in.
// Environment overrides for looks: GULLY_DETAIL, DETAIL_AMP (m), VIEWS, WIDTH (m), SUFFIX.
#include "VoxelShaderCompat.h"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <thread>
#include <random>
#include <algorithm>
#include <cmath>
#include <atomic>
#include <chrono>
#include <string>
#include <functional>
#include "VoxelNoiseTiles.h"
#include "VoxelLandRelief.h"

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
#include "VoxelDF64.ush"
#include "VoxelCubeSphere.ush"

    // The GPU path's inputs (VoxelDensitySample.ush, VoxelLandParameters.ush).
    const FVoxelOctave* Octaves = nullptr;
    uint ContinentOctaves = 0, MountainOctaves = 0, DetailOctaves = 0;
    float2 InvNormContinent, InvNormMountain, InvNormDetail, PlanetRadius, NoiseAmplitude, HalfFinest;
    float2 ContinentWeight, MountainWeight, MountainStart, MountainBlend, DetailWeight;
    float RidgeGain = 2.0f, SlopeDamping = 1.0f;
    uint LandMode = 1;
    const float* LandHeight = nullptr;
    const float2* LandTanTable = nullptr;
    const uint* LandFlow = nullptr;
    const uint* LandForm = nullptr;
    const uint* LandRelief = nullptr;
    const uint* LandProvince = nullptr;
    const uint* LandRiverTiles = nullptr;
    const float* LandRiverData = nullptr;
    const uint* LandNoise = nullptr;
    uint LandN = 0, LandPad = 4, LandHasFlow = 0, LandHasForm = 0, LandWarpOctaves = 3;
    float LandHillRelief = 0.0f;
    float2 LandHillFrequency;
    uint LandHillOctaves = 0, LandHillSeed = 0;
    float LandPeakStrength = 0.0f, LandPeakTexelsPerCell = 0.0f, LandPeakSpacingM = 0.0f, LandPeakReliefM = 0.0f, LandPeakReliefStart = 500.0f, LandPeakReliefEnd = 1500.0f;
    float LandPeakMeanR = 0.0f, LandPeakMeanG = 0.0f, LandPeakScaleR = 1.0f, LandPeakScaleG = 1.0f, LandPeakScaleB = 1.0f;
    uint LandPeakSeed = 0, LandNoiseN = 0, LandNoiseMips = 0;
    uint LandDetailBase = 0, LandDetailN = 0, LandDetailMips = 0;
    float LandPeakDetailTexelsPerCell = 0.0f, LandDetailMeanHard = 0.0f, LandDetailMeanSoft = 0.0f;
    float LandPeakDetailScaleHard = 0.0f, LandPeakDetailScaleSoft = 0.0f, LandPeakDetailFade = 0.0f;
    float LandHillTexelsPerCell = 0.0f, LandHillScaleR = 1.0f, LandHillScaleG = 1.0f, LandHillScaleB = 1.0f, LandHillMeanR = 0.0f, LandHillMeanG = 0.0f, LandHillSpacingM = 0.0f;
    uint LandHillBase = 0, LandHillN = 0, LandHillMips = 0;
    float LandHillSwellTexelsPerCell = 0.0f, LandHillSwellM = 0.0f, LandHillSwellScaleG = 1.0f;
    float LandHillDetailTexelsPerCell = 0.0f, LandHillDetailScaleHard = 0.0f, LandHillDetailScaleSoft = 0.0f, LandHillDetailFade = 0.0f, LandHillFoothill = 0.0f;
    float LandStrataTexelsPerCell = 0.0f, LandStrataScaleR = 1.0f, LandStrataScaleG = 1.0f, LandStrataScaleB = 1.0f, LandStrataMeanR = 0.0f, LandStrataMeanG = 0.0f, LandStrataSpacingM = 0.0f;
    uint LandStrataBase = 0, LandStrataN = 0, LandStrataMips = 0, LandStrataSeed = 0;
    float LandStrataRelief = 0.0f, LandStrataFlat = 0.0f, LandStrataThreshold = 2.0f, LandStrataWidth = 0.05f;
    float LandStrataDetailTexelsPerCell = 0.0f, LandStrataDetailScaleHard = 0.0f, LandStrataDetailScaleSoft = 0.0f, LandStrataDetailFade = 0.0f;
    uint LandRiverTilesPerFace = 0;
    float LandRiverRadiusM = 1.0f, LandRiverWidthScale = 1.0f, LandRiverValleyScale = 1.0f;
    const float* LandWater = nullptr;   // R2: the carve (VoxelLandCarve.ush)
    uint LandHasWater = 0;
    float LandRiverCarve = 0.0f, LandRiverDepthScale = 1.0f, LandRiverFloodplainScale = 1.0f, LandRiverMinLog2Area = 0.0f, LandLakeShoreBand = 0.0f, LandRiverMeander = 0.0f, LandRiverFalls = 0.0f;
    float LandDetailTexelsPerCell = 0.0f, LandDetailMeanA = 0.0f, LandDetailScaleA = 0.0f, LandDetailScaleGain = 0.0f, LandDetailFade = 0.0f;   // N1: the detail layer
    uint LandDetailScales = 0, LandDetailSeed = 0;
    // T5c: the dunes (VoxelLandDunes.ush; off unless a mode sets them).
    const float4* LandClimate = nullptr;
    uint LandHasClimate = 0, LandDuneBase = 0, LandDuneN = 0, LandDuneMips = 0, LandDuneSeed = 0, LandDuneDetailBase = 0, LandDuneDetailN = 0, LandDuneDetailMips = 0;
    float LandClimateLapse = 6.5f, LandDuneTexelsPerCell = 0.0f, LandDuneScaleR = 1.0f, LandDuneScaleG = 1.0f, LandDuneMeanR = 0.0f, LandDuneMeanG = 0.0f, LandDuneRelief = 0.0f;
    float LandDuneErgFrequency = 1.0f, LandDuneErgThreshold = 2.0f, LandDuneErgWidth = 0.06f, LandDuneReliefMax = 400.0f;
    float LandDuneDetailTexelsPerCell = 0.0f, LandDuneDetailScale = 0.0f, LandDuneDetailMean = 0.0f, LandDuneDetailFade = 0.0f;
    // T5d: the glacial layer (off unless a mode sets it).
    uint LandGlacialBase = 0, LandGlacialN = 0, LandGlacialMips = 0, LandGlacialSeed = 0;
    float LandGlacialTexelsPerCell = 0.0f, LandGlacialScaleR = 1.0f, LandGlacialScaleG = 1.0f, LandGlacialScaleB = 1.0f, LandGlacialMeanR = 0.0f, LandGlacialMeanG = 0.0f;
    float LandGlacialReliefM = 0.0f, LandGlacialStrength = 0.0f, LandGlacialSpacingM = 0.0f, LandGlacialCooling = 0.0f, LandGlacialDetailScaleHard = 0.0f, LandGlacialDetailScaleSoft = 0.0f;
    float LandIceMaxThickness = 0.0f, LandIceBlend = 0.0f, LandIceSwellTexelsPerCell = 0.0f, LandIceMeanG = 0.0f, LandIceSwellM = 0.0f, LandIceSwellSpacingM = 0.0f;   // T5d2
    uint LandIceBase = 0, LandIceN = 0, LandIceMips = 0, LandIceSeed = 0;
    uint LandPolarBase = 0, LandPolarN = 0, LandPolarMips = 0, LandPolarSeed = 0;   // T5d3
    float LandPolarCrevasseTexelsPerCell = 0.0f, LandPolarCrevasseM = 0.0f, LandPolarCrevasseMean = 0.0f, LandPolarCrevasseFade = 0.0f;
    float LandPolarSastrugiTexelsPerCell = 0.0f, LandPolarSastrugiM = 0.0f, LandPolarSastrugiMean = 0.0f, LandPolarSastrugiFade = 0.0f;
    float LandPolarPolygonTexelsPerCell = 0.0f, LandPolarPolygonM = 0.0f, LandPolarPolygonMean = 0.0f, LandPolarPolygonFade = 0.0f;
    float LandPolarPermafrostT = -6.0f, LandPolarCrevasseTurn = 0.0f, LandPolarSastrugiTurn = 0.0f;
    uint LandPolarDuneBase = 0, LandPolarDuneN = 0, LandPolarDuneMips = 0;
    float LandPolarDuneTexelsPerCell = 0.0f, LandPolarDuneM = 0.0f, LandPolarDuneMean = 0.0f, LandPolarDuneFade = 0.0f;
    float LandCellM = 1.0f;
    float LandWarpInvNorm = 0, LandWarpAmplitude = 0, LandDetailAmplitude = 0;
    float2 LandHeightOffset;
    float LandGullyStrength = 0.22f;
    uint LandGullyOctaves = 0;
    float LandGullyFlowGain = 0.5f, LandGullyDetail = 0.35f;
    uint LandGullySeed = 0;
#ifdef VOXEL_HARNESS_COUNT_NOISE
    // Built with -DVOXEL_HARNESS_COUNT_NOISE: counts the noise set's word loads (pcost).
    static thread_local uint64_t GNoiseLoads = 0;
#define VOXEL_LAND_NOISE_RG(I) (GNoiseLoads++, VoxelLandUnpackNoise(LandNoise[(I) * 2u]))
#define VOXEL_LAND_NOISE_BA(I) (GNoiseLoads++, VoxelLandUnpackNoise(LandNoise[(I) * 2u + 1u]))
#endif
#include "VoxelLandSample.ush"
#include "VoxelLandGully.ush"
#include "VoxelDensitySample.ush"

    // The CPU backend's sampler (VoxelDensity.cpp's FVoxelLandSamplerCPU).
    struct FSampler
    {
        const float* LandHeight = nullptr;
        const float2* LandTanTable = nullptr;
        const uint* LandFlow = nullptr;
        const uint* LandForm = nullptr;
        const uint* LandRelief = nullptr;
        const uint* LandProvince = nullptr;
        const uint* LandRiverTiles = nullptr;
        const float* LandRiverData = nullptr;
        const uint* LandNoise = nullptr;
        uint LandN = 1, LandPad = 0, LandHasFlow = 0, LandHasForm = 0;
        float LandHillRelief = 0.0f;
        float2 LandHillFrequency;
        uint LandHillOctaves = 0, LandHillSeed = 0;
        float LandPeakStrength = 0.0f, LandPeakTexelsPerCell = 0.0f, LandPeakSpacingM = 0.0f, LandPeakReliefM = 0.0f, LandPeakReliefStart = 500.0f, LandPeakReliefEnd = 1500.0f;
        float LandPeakMeanR = 0.0f, LandPeakMeanG = 0.0f, LandPeakScaleR = 1.0f, LandPeakScaleG = 1.0f, LandPeakScaleB = 1.0f;
        uint LandPeakSeed = 0, LandNoiseN = 0, LandNoiseMips = 0;
        uint LandDetailBase = 0, LandDetailN = 0, LandDetailMips = 0;
        float LandPeakDetailTexelsPerCell = 0.0f, LandDetailMeanHard = 0.0f, LandDetailMeanSoft = 0.0f;
        float LandPeakDetailScaleHard = 0.0f, LandPeakDetailScaleSoft = 0.0f, LandPeakDetailFade = 0.0f;
        float LandHillTexelsPerCell = 0.0f, LandHillScaleR = 1.0f, LandHillScaleG = 1.0f, LandHillScaleB = 1.0f, LandHillMeanR = 0.0f, LandHillMeanG = 0.0f, LandHillSpacingM = 0.0f;
        uint LandHillBase = 0, LandHillN = 0, LandHillMips = 0;
        float LandHillSwellTexelsPerCell = 0.0f, LandHillSwellM = 0.0f, LandHillSwellScaleG = 1.0f;
        float LandHillDetailTexelsPerCell = 0.0f, LandHillDetailScaleHard = 0.0f, LandHillDetailScaleSoft = 0.0f, LandHillDetailFade = 0.0f, LandHillFoothill = 0.0f;
        float LandStrataTexelsPerCell = 0.0f, LandStrataScaleR = 1.0f, LandStrataScaleG = 1.0f, LandStrataScaleB = 1.0f, LandStrataMeanR = 0.0f, LandStrataMeanG = 0.0f, LandStrataSpacingM = 0.0f;
        uint LandStrataBase = 0, LandStrataN = 0, LandStrataMips = 0, LandStrataSeed = 0;
        float LandStrataRelief = 0.0f, LandStrataFlat = 0.0f, LandStrataThreshold = 2.0f, LandStrataWidth = 0.05f;
        float LandStrataDetailTexelsPerCell = 0.0f, LandStrataDetailScaleHard = 0.0f, LandStrataDetailScaleSoft = 0.0f, LandStrataDetailFade = 0.0f;
        uint LandRiverTilesPerFace = 0;
        float LandRiverRadiusM = 1.0f, LandRiverWidthScale = 1.0f, LandRiverValleyScale = 1.0f;
        const float* LandWater = nullptr;   // R2: the carve (VoxelLandCarve.ush)
        uint LandHasWater = 0;
        float LandRiverCarve = 0.0f, LandRiverDepthScale = 1.0f, LandRiverFloodplainScale = 1.0f, LandRiverMinLog2Area = 0.0f, LandLakeShoreBand = 0.0f, LandRiverMeander = 0.0f, LandRiverFalls = 0.0f;
        float LandDetailTexelsPerCell = 0.0f, LandDetailMeanA = 0.0f, LandDetailScaleA = 0.0f, LandDetailScaleGain = 0.0f, LandDetailFade = 0.0f;   // N1: the detail layer
        uint LandDetailScales = 0, LandDetailSeed = 0;
        const float4* LandClimate = nullptr;   // T5c
        uint LandHasClimate = 0, LandDuneBase = 0, LandDuneN = 0, LandDuneMips = 0, LandDuneSeed = 0, LandDuneDetailBase = 0, LandDuneDetailN = 0, LandDuneDetailMips = 0;
        float LandClimateLapse = 6.5f, LandDuneTexelsPerCell = 0.0f, LandDuneScaleR = 1.0f, LandDuneScaleG = 1.0f, LandDuneMeanR = 0.0f, LandDuneMeanG = 0.0f, LandDuneRelief = 0.0f;
        float LandDuneErgFrequency = 1.0f, LandDuneErgThreshold = 2.0f, LandDuneErgWidth = 0.06f, LandDuneReliefMax = 400.0f;
        float LandDuneDetailTexelsPerCell = 0.0f, LandDuneDetailScale = 0.0f, LandDuneDetailMean = 0.0f, LandDuneDetailFade = 0.0f;
        uint LandGlacialBase = 0, LandGlacialN = 0, LandGlacialMips = 0, LandGlacialSeed = 0;   // T5d
        float LandGlacialTexelsPerCell = 0.0f, LandGlacialScaleR = 1.0f, LandGlacialScaleG = 1.0f, LandGlacialScaleB = 1.0f, LandGlacialMeanR = 0.0f, LandGlacialMeanG = 0.0f;
        float LandGlacialReliefM = 0.0f, LandGlacialStrength = 0.0f, LandGlacialSpacingM = 0.0f, LandGlacialCooling = 0.0f, LandGlacialDetailScaleHard = 0.0f, LandGlacialDetailScaleSoft = 0.0f;
        float LandIceMaxThickness = 0.0f, LandIceBlend = 0.0f, LandIceSwellTexelsPerCell = 0.0f, LandIceMeanG = 0.0f, LandIceSwellM = 0.0f, LandIceSwellSpacingM = 0.0f;   // T5d2
        uint LandIceBase = 0, LandIceN = 0, LandIceMips = 0, LandIceSeed = 0;
        uint LandPolarBase = 0, LandPolarN = 0, LandPolarMips = 0, LandPolarSeed = 0;   // T5d3
        float LandPolarCrevasseTexelsPerCell = 0.0f, LandPolarCrevasseM = 0.0f, LandPolarCrevasseMean = 0.0f, LandPolarCrevasseFade = 0.0f;
        float LandPolarSastrugiTexelsPerCell = 0.0f, LandPolarSastrugiM = 0.0f, LandPolarSastrugiMean = 0.0f, LandPolarSastrugiFade = 0.0f;
        float LandPolarPolygonTexelsPerCell = 0.0f, LandPolarPolygonM = 0.0f, LandPolarPolygonMean = 0.0f, LandPolarPolygonFade = 0.0f;
        float LandPolarPermafrostT = -6.0f, LandPolarCrevasseTurn = 0.0f, LandPolarSastrugiTurn = 0.0f;
        uint LandPolarDuneBase = 0, LandPolarDuneN = 0, LandPolarDuneMips = 0;
        float LandPolarDuneTexelsPerCell = 0.0f, LandPolarDuneM = 0.0f, LandPolarDuneMean = 0.0f, LandPolarDuneFade = 0.0f;
        float LandCellM = 1.0f;
        float LandGullyStrength = 0.0f;
        uint LandGullyOctaves = 0;
        float LandGullyFlowGain = 0.0f, LandGullyDetail = 1.0f;
        uint LandGullySeed = 0;
#undef VOXEL_LAND_SAMPLE_USH
#undef VOXEL_LAND_HILLS_USH
#undef VOXEL_LAND_NOISE_USH
#undef VOXEL_LAND_PEAKS_USH
#undef VOXEL_LAND_GULLY_USH
#undef VOXEL_LAND_HILL_SET_USH
#undef VOXEL_LAND_DETAIL_USH
#undef VOXEL_LAND_RIVERS_USH
#undef VOXEL_LAND_CARVE_USH
#undef VOXEL_LAND_DUNES_USH
#undef VOXEL_LAND_CLIMATE_USH
#undef VOXEL_LAND_CLIMATE_ACCESS_USH
#undef VOXEL_LAND_ICE_USH
#undef VOXEL_LAND_POLAR_USH
#include "VoxelLandSample.ush"
#include "VoxelLandGully.ush"
    };
}
using namespace VoxelShader;
// R1b: the rivers' GPU build, run on the CPU (Tools/LandRiverHarness).
#include "../LandRiverHarness/river_build_cpu.h"

static double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
static float2 DF(double V) { const float H = (float)V; return float2(H, (float)(V - (double)H)); }
static void ParallelFor(int N, const std::function<void(int)>& F)
{
    const int T = std::max(1u, std::thread::hardware_concurrency());
    std::atomic<int> Next{ 0 };
    std::vector<std::thread> Th;
    for (int t = 0; t < T; t++) Th.emplace_back([&]() { for (int i; (i = Next.fetch_add(1)) < N;) F(i); });
    for (auto& t : Th) t.join();
}

// MakeLayer / EvaluateLayerGradient from VoxelDensity.cpp, in std types.
struct FOct { double Freq; double Off[3]; float Wave; float Amp; uint32_t Seed; uint32_t Shift = 0xffffffffu; uint32_t OffCell[3]{}, OffFrac[3]{}; };
struct FLay { uint32_t Kind = 0; double InvNorm = 0; std::vector<FOct> O; };
static FVoxelFixed ToFixed(double V) { double F = std::floor(V); FVoxelFixed O; O.Cell = (uint32_t)(int64_t)F; O.Frac = (uint32_t)std::min((V - F) * 4294967296.0, 4294967295.0); return O; }
static FLay MakeLayer(uint32_t Kind, double WF, double R, double Gain, int N, uint32_t Seed)
{
    FLay L; L.Kind = Kind;
    double Freq = 1.0 / std::max(WF * R, 1e-3);
    const bool bFixed = N > 0 && N <= 32 && R * Freq * std::pow(2.0, N - 1) < 1073741824.0;
    float Amp = 1; double AS = 0, AQ = 0, A = 1;
    for (int o = 0; o < N; o++)
    {
        AS += A; AQ += A * A; A *= Gain;
        FOct O; O.Freq = Freq; O.Wave = (float)(1 / Freq); O.Amp = Amp; O.Seed = Seed + (uint32_t)o * 0x9e3779b9u;
        const uint32_t H = VoxelHashSeed(Seed, 0x51ed27u + (uint32_t)o);
        O.Off[0] = (H & 0x3ffu) * 0.618034; O.Off[1] = ((H >> 10) & 0x3ffu) * 0.414214; O.Off[2] = ((H >> 20) & 0x3ffu) * 0.732051;
        if (bFixed) { O.Shift = N - 1 - o; for (int a = 0; a < 3; a++) { auto F = ToFixed(O.Off[a]); O.OffCell[a] = F.Cell; O.OffFrac[a] = F.Frac; } }
        L.O.push_back(O); Freq *= 2; Amp *= (float)Gain;
    }
    L.InvNorm = Kind == VoxelLayerRidged ? (AS > 0 ? 1 / AS : 0) : (AQ > 0 ? 1.0 / (2.5 * 0.1914 * std::sqrt(AQ)) : 0);
    return L;
}
static double Evaluate(const FLay& L, const double SP[3], double Footprint, double* Grad = nullptr)
{
    FVoxelFbmState S = VoxelFbmBegin(); double Sum = 0;
    double G[3] = { 0, 0, 0 };
    if (Grad) Grad[0] = Grad[1] = Grad[2] = 0;
    if (L.O.empty()) return 0;
    const bool bFixed = L.O[0].Shift != VoxelNoFixedPoint;
    FVoxelFixed Fin[3];
    if (bFixed) for (int a = 0; a < 3; a++) Fin[a] = ToFixed(SP[a] * L.O.back().Freq);
    for (const FOct& O : L.O)
    {
        const float Band = VoxelOctaveBandWeight(O.Wave, (float)Footprint);
        if (Band <= 0) break;
        uint3 C; float3 F;
        if (bFixed)
        {
            FVoxelFixed X[3];
            for (int a = 0; a < 3; a++) X[a] = VoxelFixedLattice(Fin[a], O.Shift, O.OffCell[a], O.OffFrac[a]);
            C = uint3(X[0].Cell, X[1].Cell, X[2].Cell); F = float3(VoxelFixedFraction(X[0].Frac), VoxelFixedFraction(X[1].Frac), VoxelFixedFraction(X[2].Frac));
        }
        else
        {
            double V[3]; for (int a = 0; a < 3; a++) V[a] = SP[a] * O.Freq + O.Off[a];
            double Fl[3] = { std::floor(V[0]), std::floor(V[1]), std::floor(V[2]) };
            C = uint3((uint32_t)(int64_t)Fl[0], (uint32_t)(int64_t)Fl[1], (uint32_t)(int64_t)Fl[2]); F = float3((float)(V[0] - Fl[0]), (float)(V[1] - Fl[1]), (float)(V[2] - Fl[2]));
        }
        const FVoxelNoiseSample N = VoxelGradientNoise(C, F, O.Seed);
        S.Sum = 0; S = VoxelLayerOctave(S, L.Kind, N, O.Amp * Band, 2.0f, 1.0f); Sum += S.Sum;
        const double K = (double)(O.Amp * Band) * O.Freq;
        G[0] += N.Gradient.x * K; G[1] += N.Gradient.y * K; G[2] += N.Gradient.z * K;
    }
    if (Grad) for (int a = 0; a < 3; a++) Grad[a] = G[a] * L.InvNorm;
    return Sum * L.InvNorm;
}
static FVoxelOctave ToGPU(const FOct& O)
{
    FVoxelOctave G{};
    G.Frequency = DF(O.Freq); G.OffsetX = DF(O.Off[0]); G.OffsetY = DF(O.Off[1]); G.OffsetZ = DF(O.Off[2]);
    G.Wavelength = O.Wave; G.Amplitude = O.Amp; G.Seed = O.Seed; G.Shift = O.Shift;
    G.OffsetCellX = O.OffCell[0]; G.OffsetCellY = O.OffCell[1]; G.OffsetCellZ = O.OffCell[2];
    G.OffsetFracX = O.OffFrac[0]; G.OffsetFracY = O.OffFrac[1]; G.OffsetFracZ = O.OffFrac[2];
    return G;
}
static uint16_t ToHalf(float F)
{
    uint32_t X; std::memcpy(&X, &F, 4);
    const uint32_t Sign = (X >> 16) & 0x8000u;
    int E = (int)((X >> 23) & 0xff) - 127 + 15;
    uint32_t M = X & 0x7fffffu;
    if (E <= 0) return (uint16_t)Sign;
    if (E >= 31) return (uint16_t)(Sign | 0x7c00u);
    // round to nearest even
    uint32_t H = Sign | ((uint32_t)E << 10) | (M >> 13);
    const uint32_t Rest = M & 0x1fffu;
    if (Rest > 0x1000u || (Rest == 0x1000u && (H & 1u))) H++;
    return (uint16_t)H;
}

// ---------------------------------------------------------------------------------------------
// The planet: a dumped bake and the CPU density's land path (FVoxelDensityCPU::LandHeightAt).
// ---------------------------------------------------------------------------------------------

struct FPlanet
{
    int N = 0, Pad = 4;
    double SeaRadius = 0;      // cm
    double R = 0, NA = 0;      // the density's base radius and range (cm)
    double Cell = 0;           // cm
    std::vector<float> Height, FlowF;
    std::vector<uint32_t> FlowPacked, FormPacked, ReliefPacked;
    std::vector<uint32_t> Form;        // per cell (VoxelLandPackForm)
    std::vector<uint8_t> Relief;       // per cell (VoxelLandRelief.h)
    std::vector<uint8_t> Province;     // T4c: per cell (the form dump's bits 16..21)
    std::vector<uint32_t> ProvincePacked;
    std::vector<uint32_t> Rec;         // R1: the final routing's receivers (Tag_rec.u32), empty without
    RiverBuildCPU::FTables Rivers;     // R1b: the rivers (the GPU build's tables, run on the CPU), from the environment
    std::vector<float> WaterF;
    float MaxRelief = 0;
    VoxelNoiseTiles::FSet NoiseSet;    // the peaks' set (VoxelLand::GetPeakNoise's)
    std::vector<uint32_t> NoiseWords;  // FVoxelLandNoiseSet::Words: the chain in whole rows of N texels, then the detail's
    uint32_t NoiseRows = 0;
    VoxelNoiseTiles::FSet Detail;      // N1: the detail set (DETAIL_SET=file.vxnt, else the generated stand-in)
    VoxelNoiseTiles::FDetailStats DetailStats;
    uint32_t DetailBase = 0;           // its first texel in NoiseWords
    VoxelNoiseTiles::FSet HillSet;     // T4a: HILL_SET=file.vxnt, after the detail (whole rows)
    uint32_t HillBase = 0;
    VoxelNoiseTiles::FSet StrataSet;   // T4c: STRATA_SET=file.vxnt, after the hills (whole rows)
    uint32_t StrataBase = 0;
    std::vector<float2> Tan;
    FLay W[3];
    double WarpAmp = 0, DetailAmp = 0, Offset = 0, Finest = 0, Half = 0;
    int Depth = 0;
    FSampler S;
    float Strength = 0.22f;
    int Side() const { return N + 2 * Pad; }
};

static std::vector<char> ReadFile(const std::string& Name)
{
    FILE* F = fopen(Name.c_str(), "rb");
    if (!F) { printf("missing %s\n", Name.c_str()); exit(1); }
    fseek(F, 0, SEEK_END); const long L = ftell(F); fseek(F, 0, SEEK_SET);
    std::vector<char> B(L); if (fread(B.data(), 1, L, F) != (size_t)L) exit(1); fclose(F);
    return B;
}

/** The gullies' strength for the harnesses: GULLY_STRENGTH, else the plugin's default (0 since
 *  N1b: off; 0.22 before). */
static double HarnessGullyStrength()
{
    return getenv("GULLY_STRENGTH") ? atof(getenv("GULLY_STRENGTH")) : 0.0;
}

static void Load(FPlanet& P, const std::string& Tag, int N, double RadiusM, double FinestM, double Strength)
{
    P.N = N; P.SeaRadius = RadiusM * 100.0;
    auto H = ReadFile(Tag + "_height.f32"), Fl = ReadFile(Tag + "_flow.f32"), T = ReadFile(Tag + "_tan.f32");
    P.Height.resize(H.size() / 4); std::memcpy(P.Height.data(), H.data(), H.size());
    P.FlowF.resize(Fl.size() / 4); std::memcpy(P.FlowF.data(), Fl.data(), Fl.size());
    P.Tan.resize(T.size() / 8); std::memcpy(P.Tan.data(), T.data(), T.size());
    P.FlowPacked.resize(P.FlowF.size() / 2);
    for (size_t i = 0; i < P.FlowPacked.size(); i++) P.FlowPacked[i] = ToHalf(P.FlowF[2 * i]) | ((uint32_t)ToHalf(P.FlowF[2 * i + 1]) << 16);
    // The landform (T2), when the dump has it.
    if (FILE* FF = fopen((Tag + "_form.u32").c_str(), "rb"))
    {
        fclose(FF);
        auto Fo = ReadFile(Tag + "_form.u32");
        P.Form.resize(Fo.size() / 4); std::memcpy(P.Form.data(), Fo.data(), Fo.size());
        P.FormPacked.resize(P.Form.size() / 2);
        for (size_t i = 0; i < P.FormPacked.size(); i++) P.FormPacked[i] = (P.Form[2 * i] & 0xffffu) | ((P.Form[2 * i + 1] & 0xffffu) << 16);
    }
    // R1: the lake levels and the final routing's receivers (dumps from R1 on).
    {
        auto Wt = ReadFile(Tag + "_water.f32");
        P.WaterF.resize(Wt.size() / 4); std::memcpy(P.WaterF.data(), Wt.data(), Wt.size());
        if (FILE* RF = fopen((Tag + "_rec.u32").c_str(), "rb"))
        {
            fclose(RF);
            auto Rc = ReadFile(Tag + "_rec.u32");
            P.Rec.resize(Rc.size() / 4); std::memcpy(P.Rec.data(), Rc.data(), Rc.size());
        }
    }
    // The province (T4c): the form's bits from 16 (0 in dumps from before T4c), four cells per word.
    P.Province.assign(P.Height.size(), 0);
    for (size_t i = 0; i < P.Form.size() && i < P.Province.size(); i++) P.Province[i] = (uint8_t)std::min(P.Form[i] >> 16, 63u);
    P.ProvincePacked.assign((P.Province.size() + 3) / 4, 0u);
    for (size_t i = 0; i < P.Province.size(); i++) P.ProvincePacked[i >> 2] |= (uint32_t)P.Province[i] << ((i & 3) * 8);
    float Mn = 1e9, Mx = -1e9; for (float h : P.Height) { Mn = std::min(Mn, h); Mx = std::max(Mx, h); }
    P.Cell = M_PI * 0.5 * P.SeaRadius / N;
    // The local relief (FVoxelLandBake::BuildRelief) and the peaks' noise set (VoxelLand::GetPeakNoise).
    {
        const int Side = P.Side();
        P.Relief.assign(P.Height.size(), 0);
        ParallelFor(6 * Side, [&](int Row) { VoxelLandRelief::BuildRow(P.Height.data(), Side, Row / Side, Row % Side, P.Relief.data()); });
        uint8_t MaxQ = 0; for (uint8_t q : P.Relief) MaxQ = std::max(MaxQ, q);
        P.MaxRelief = VoxelLandRelief::Decode(MaxQ);
        P.ReliefPacked.assign((P.Relief.size() + 3) / 4, 0u);
        for (size_t i = 0; i < P.Relief.size(); i++) P.ReliefPacked[i >> 2] |= (uint32_t)P.Relief[i] << ((i & 3) * 8);
        // NOISE_SET=path.vxnt reads an authored set (VoxelNoiseTiles::ReadVxnt), else the
        // generated one.
        const char* SetPath = getenv("NOISE_SET");
        if (SetPath && *SetPath)
        {
            const std::vector<char> Bytes = ReadFile(SetPath);
            std::string Error = VoxelNoiseTiles::ReadVxnt((const uint8_t*)Bytes.data(), Bytes.size(), P.NoiseSet);
            if (Error.empty()) Error = VoxelNoiseTiles::CheckPeaks(P.NoiseSet);
            if (!Error.empty()) { fprintf(stderr, "%s: %s\n", SetPath, Error.c_str()); exit(1); }
        }
        else
        {
            P.NoiseSet = VoxelNoiseTiles::GeneratePeaksStandIn(0x7065616bu, 1024, [](int32_t Count, auto&& Body) { ParallelFor(Count, [&](int i) { Body(i); }); });
        }
        const size_t Texels = P.NoiseSet.Texels.size() / 4;
        P.NoiseRows = (uint32_t)((Texels + P.NoiseSet.N - 1) / P.NoiseSet.N);
        // N1: the detail set after the chain, from a whole row (VoxelLandNoiseDetail::MakePackLocked):
        // DETAIL_SET=path.vxnt, else the generated stand-in.
        if (const char* DetailPath = getenv("DETAIL_SET"))
        {
            const std::vector<char> Bytes = ReadFile(DetailPath);
            std::string Error = VoxelNoiseTiles::ReadVxnt((const uint8_t*)Bytes.data(), Bytes.size(), P.Detail);
            if (Error.empty()) Error = VoxelNoiseTiles::CheckDetail(P.Detail);
            if (!Error.empty()) { fprintf(stderr, "%s: %s\n", DetailPath, Error.c_str()); exit(1); }
        }
        else
            P.Detail = VoxelNoiseTiles::GenerateDetailStandIn(0x64657461u, 1024, [](int32_t Count, auto&& Body) { ParallelFor(Count, [&](int i) { Body(i); }); });
        P.DetailStats = VoxelNoiseTiles::DetailStatsOf(P.Detail, P.Detail.Texels.data());
        const size_t DetailTexels = P.Detail.Texels.size() / 4;
        const uint32_t DetailRows = (uint32_t)((DetailTexels + P.NoiseSet.N - 1) / P.NoiseSet.N);
        P.DetailBase = P.NoiseRows * P.NoiseSet.N;
        P.NoiseWords.assign(2 * (size_t)(P.NoiseRows + DetailRows) * P.NoiseSet.N, 0u);
        std::memcpy(P.NoiseWords.data(), P.NoiseSet.Texels.data(), Texels * 8);
        std::memcpy(P.NoiseWords.data() + 2 * (size_t)P.DetailBase, P.Detail.Texels.data(), DetailTexels * 8);
        // T4a: the hills' set (HILL_SET=file.vxnt) after the detail, from a whole row (the
        // plugin's pack: VoxelLand::GetLandNoise).
        if (const char* HillPath = getenv("HILL_SET"))
        {
            const std::vector<char> Bytes = ReadFile(HillPath);
            std::string Error = VoxelNoiseTiles::ReadVxnt((const uint8_t*)Bytes.data(), Bytes.size(), P.HillSet);
            if (Error.empty()) Error = VoxelNoiseTiles::CheckPeaks(P.HillSet);
            if (!Error.empty()) { fprintf(stderr, "%s: %s\n", HillPath, Error.c_str()); exit(1); }
            const size_t HillTexels = P.HillSet.Texels.size() / 4;
            const uint32_t HillRows = (uint32_t)((HillTexels + P.NoiseSet.N - 1) / P.NoiseSet.N);
            P.HillBase = (P.NoiseRows + DetailRows) * P.NoiseSet.N;
            P.NoiseWords.resize(2 * (size_t)(P.NoiseRows + DetailRows + HillRows) * P.NoiseSet.N, 0u);
            std::memcpy(P.NoiseWords.data() + 2 * (size_t)P.HillBase, P.HillSet.Texels.data(), HillTexels * 8);
            // T4c: the strata's set (STRATA_SET=file.vxnt) after the hills'.
            if (const char* StrataPath = getenv("STRATA_SET"))
            {
                const std::vector<char> SBytes = ReadFile(StrataPath);
                std::string SError = VoxelNoiseTiles::ReadVxnt((const uint8_t*)SBytes.data(), SBytes.size(), P.StrataSet);
                if (SError.empty()) SError = VoxelNoiseTiles::CheckPeaks(P.StrataSet);
                if (!SError.empty()) { fprintf(stderr, "%s: %s\n", StrataPath, SError.c_str()); exit(1); }
                const size_t StrataTexels = P.StrataSet.Texels.size() / 4;
                const uint32_t StrataRows = (uint32_t)((StrataTexels + P.NoiseSet.N - 1) / P.NoiseSet.N);
                const uint32_t Rows = (uint32_t)(P.NoiseWords.size() / 2 / P.NoiseSet.N);
                P.StrataBase = Rows * P.NoiseSet.N;
                P.NoiseWords.resize(2 * (size_t)(Rows + StrataRows) * P.NoiseSet.N, 0u);
                std::memcpy(P.NoiseWords.data() + 2 * (size_t)P.StrataBase, P.StrataSet.Texels.data(), StrataTexels * 8);
            }
        }
    }
    // VoxelLand::GullyOctavesFor and GullyReach.
    const double Lambda0 = 2.0 * P.Cell * 0.01;
    const int Octaves = std::clamp((int)std::floor(std::log2(Lambda0 / FinestM)) + 1, 0, 12);
    double GS = 0, Gn = 1; for (int o = 0; o < Octaves; o++) { GS += Gn; Gn *= 0.42; }
    const double GullyReach = 0.5 * Strength * Lambda0 * 100.0 * 0.8 * GS;
    P.DetailAmp = (getenv("DETAIL_AMP") ? atof(getenv("DETAIL_AMP")) : 250.0) * 100;
    // The hills (VoxelLand::HillOctavesFor, HillFinestFrequency), from the environment.
    constexpr double HillMaxUnit = 2.5;   // VoxelLand::HillMaxUnit (VoxelLandBake.h)
    const double HillRelief = getenv("HILL_RELIEF") ? atof(getenv("HILL_RELIEF")) : 150.0;
    const double HillLongest = getenv("HILL_WAVELENGTH") ? atof(getenv("HILL_WAVELENGTH")) : 4000.0;
    const double HillFinest = getenv("HILL_FINEST") ? atof(getenv("HILL_FINEST")) : 25.0;
    const int HillOctaves = P.Form.empty() || HillRelief <= 0 ? 0 : std::clamp((int)std::floor(std::log2(HillLongest / HillFinest)) + 1, 1, 12);
    const double HillFrequency = 1.0 / (HillLongest * 100.0 / std::pow(2.0, std::max(HillOctaves, 1) - 1));
    // The peaks (VoxelLand::PeakTexelsPerCell, PeakReach; FVoxelDensityCPU), from the environment.
    const double PeakStrength = getenv("PEAK_STRENGTH") ? atof(getenv("PEAK_STRENGTH")) : 1.0;
    const double PeakSpacing = getenv("PEAK_SPACING") ? atof(getenv("PEAK_SPACING")) : 6000.0;
    const bool bPeaks = !P.Form.empty() && PeakStrength > 0;
    // The detail (VoxelLand::PeakDetailWavelength, PeakReach): PEAK_DETAIL its strength,
    // PEAK_DETAIL_WAVELENGTH its coarsest wavelength (m).
    const double PeakDetail = getenv("PEAK_DETAIL") ? atof(getenv("PEAK_DETAIL")) : 1.0;
    const double PeakDetailWave = std::clamp(getenv("PEAK_DETAIL_WAVELENGTH") ? atof(getenv("PEAK_DETAIL_WAVELENGTH")) : 400.0, 16.0,
        0.25 * std::max(PeakSpacing, 500.0));
    const double DetailTexelM = PeakDetailWave / (double)(P.Detail.N / P.Detail.Channel[0].CellsPerTile);
    const double PeakUp = bPeaks ? PeakStrength * (P.MaxRelief + (PeakDetail > 0 ? PeakDetail * VoxelNoiseTiles::DetailRangeWaves * PeakDetailWave : 0.0)) * 100.0 : 0.0;
    const double PeakDown = PeakUp;
    // The detail layer's bound (VoxelLand::ShapeFor).
    const double Reach = 1.01 * P.DetailStats.BoundA * P.DetailAmp + GullyReach + HillMaxUnit * HillRelief * 100.0 + 100;
    const double Low = std::min(Mn * 100.0, -PeakDown) - Reach, High = Mx * 100.0 + PeakUp + Reach;
    P.R = P.SeaRadius + Low; P.NA = High - Low;
    const double Root = (P.R + P.NA) * 1.05;
    P.Depth = (int)std::ceil(std::log2(2 * Root / 150.0));
    P.Finest = 2 * Root / std::pow(2.0, P.Depth); P.Half = P.Finest / 2;
    for (int a = 0; a < 3; a++) P.W[a] = MakeLayer(VoxelLayerPlain, 6 * P.Cell / P.SeaRadius, P.SeaRadius, 0.5, 3, VoxelHashSeed(7, 0x11 + a));
    P.WarpAmp = (getenv("WARP") ? atof(getenv("WARP")) : 1.0) * P.Cell; P.Offset = P.SeaRadius - P.R;
    P.Strength = (float)Strength;
    FSampler& S = P.S;
    S.LandHeight = P.Height.data(); S.LandTanTable = P.Tan.data(); S.LandFlow = P.FlowPacked.data();
    S.LandN = N; S.LandPad = P.Pad; S.LandHasFlow = 1; S.LandCellM = (float)(P.Cell * 0.01);
    S.LandGullyStrength = (float)Strength; S.LandGullyOctaves = Octaves; S.LandGullyFlowGain = 0.5f; S.LandGullyDetail = 0.5f;
    S.LandGullySeed = VoxelHashSeed(7, 0x21u);
    S.LandForm = P.FormPacked.data(); S.LandHasForm = P.FormPacked.empty() ? 0u : 1u;
    S.LandHillRelief = (float)HillRelief; S.LandHillOctaves = (uint)HillOctaves; S.LandHillFrequency = DF(HillFrequency);
    S.LandHillSeed = VoxelHashSeed(7, 0x31u);
    S.LandRelief = P.ReliefPacked.data();
    S.LandProvince = P.ProvincePacked.data();
    // N1: the detail layer (FVoxelDensityCPU's terrestrial constructor): DETAIL_WAVELENGTH (m),
    // DETAIL_AMP (m, above).
    if (P.DetailAmp > 0 && P.DetailStats.StdA > 0)
    {
        const double WaveM = std::max(getenv("DETAIL_WAVELENGTH") ? atof(getenv("DETAIL_WAVELENGTH")) : 20000.0, 100.0);
        const double TexelM = WaveM / (double)(P.Detail.N / P.Detail.Channel[0].CellsPerTile);
        S.LandDetailTexelsPerCell = (float)((double)S.LandCellM / TexelM);
        const double DetailFinestM = std::max(P.Finest, 1.0) * 0.01;
        const int DOctaves = std::max((int)std::ceil(std::log2(WaveM / DetailFinestM)) + 1, 1);
        int Scales = std::clamp((DOctaves + 3) / 4, 1, 4);
        const double FaceTexels = std::sqrt(2.0) * (0.5 * (double)N + 4.0);
        while (Scales > 1 && (double)S.LandDetailTexelsPerCell * std::pow(16.0, Scales - 1) * FaceTexels >= 1073741824.0) Scales--;
        S.LandDetailScales = (uint)Scales;
        S.LandDetailMeanA = (float)P.DetailStats.MeanA;
        S.LandDetailScaleA = 0.4f / (float)P.DetailStats.StdA;
        S.LandDetailScaleGain = (float)P.DetailStats.ScaleGain;
        S.LandDetailFade = (float)std::floor(std::log2((double)(P.Detail.N / P.Detail.Channel[0].CellsPerTile))) - 1.0f;
        S.LandDetailSeed = VoxelHashSeed(7, 0x61u);
        printf("detail layer from %s: %d scales from %.0f m (texel %.2f m), A mean %.4f std %.4f, bound %.2f; hard %.2f soft %.2f texels\n",
            getenv("DETAIL_SET") ? getenv("DETAIL_SET") : "the generated set", Scales, WaveM, TexelM, P.DetailStats.MeanA, P.DetailStats.StdA,
            P.DetailStats.BoundA, P.DetailStats.RangeHard, P.DetailStats.RangeSoft);
    }
    // R1: the rivers (VoxelLand::GetRivers), when the dump has the receivers: RIVER_MIN_AREA
    // (km^2, 0 off), RIVER_WIDTH, VALLEY_WIDTH (the scales).
    if (!P.Rec.empty())
    {
        const double MinArea = getenv("RIVER_MIN_AREA") ? atof(getenv("RIVER_MIN_AREA")) : 200.0;
        const double WidthScale = getenv("RIVER_WIDTH") ? atof(getenv("RIVER_WIDTH")) : 1.0;
        const double ValleyScale = getenv("VALLEY_WIDTH") ? atof(getenv("VALLEY_WIDTH")) : 1.0;
        if (MinArea > 0)
        {
            RiverBuildCPU::FSettings RS;
            RS.N = N; RS.Pad = P.Pad; RS.RadiusM = RadiusM; RS.MinAreaKm2 = MinArea; RS.WidthScale = WidthScale; RS.ValleyScale = ValleyScale;
            for (float H : P.Height) RS.LandCells += H > 0.0f;
            P.Rivers = RiverBuildCPU::Build(RS, P.Tan.data(), P.Height.data(), P.WaterF.data(), P.FlowPacked.data(), P.Rec.data());
            S.LandRiverTiles = P.Rivers.Tiles.data(); S.LandRiverData = P.Rivers.Data.data();
            S.LandRiverTilesPerFace = (uint)P.Rivers.TilesPerFace;
            S.LandRiverRadiusM = (float)RadiusM; S.LandRiverWidthScale = (float)WidthScale; S.LandRiverValleyScale = (float)ValleyScale;
            // R2: the carve (FVoxelDensityCPU's terrestrial constructor): CARVE (0 off), RIVER_DEPTH,
            // FLOODPLAIN, LAKE_SHORE (m).
            S.LandRiverCarve = getenv("CARVE") ? (float)atof(getenv("CARVE")) : 1.0f;
            S.LandRiverDepthScale = getenv("RIVER_DEPTH") ? (float)atof(getenv("RIVER_DEPTH")) : 1.0f;
            S.LandRiverFloodplainScale = getenv("FLOODPLAIN") ? (float)atof(getenv("FLOODPLAIN")) : 1.0f;
            S.LandRiverMinLog2Area = (float)std::log2(MinArea * 1.0e6);
            // R3: MEANDER, FALLS (the settings' scales).
            S.LandRiverMeander = getenv("MEANDER") ? (float)atof(getenv("MEANDER")) : 1.0f;
            S.LandRiverFalls = getenv("FALLS") ? (float)atof(getenv("FALLS")) : 1.0f;
            const uint32_t* C = P.Rivers.Counts;
            printf("rivers from %.0f km^2 (the GPU build, %d build%s): %u segments, %u chains, %u mouths, %u lake inlets, %u outlets; %u entries (max %u a tile, overflow %u)\n",
                MinArea, P.Rivers.Builds, P.Rivers.Builds == 1 ? "" : "s", C[1], C[6], C[9], C[10], C[11], C[16], C[14], C[15]);
        }
    }
    // R2: the lakes' carve (the dump's levels): LAKE_SHORE (m, the bench's band), CARVE 0 off.
    if (P.WaterF.size() == P.Height.size() && !(getenv("CARVE") && atof(getenv("CARVE")) <= 0.0))
    {
        S.LandWater = P.WaterF.data(); S.LandHasWater = 1;
        S.LandLakeShoreBand = getenv("LAKE_SHORE") ? (float)atof(getenv("LAKE_SHORE")) : 3.0f;
    }
    S.LandNoise = P.NoiseWords.data();
    S.LandNoiseN = P.NoiseSet.N; S.LandNoiseMips = P.NoiseSet.Mips;
    if (bPeaks)
    {
        // FVoxelDensityCPU's terrestrial constructor (VoxelLand::PeakTexelsPerCell).
        const auto& Set = P.NoiseSet;
        auto Unit = [&](int c, double V) { const auto& C = Set.Channel[c]; return (V - (double)C.Min) / std::max((double)C.Max - (double)C.Min, 1e-12); };
        const double TexelM = PeakSpacing / (double)(Set.N / Set.Channel[0].CellsPerTile);
        S.LandPeakTexelsPerCell = (float)(P.Cell * 0.01 / TexelM);
        const float TexelMF = S.LandCellM / S.LandPeakTexelsPerCell;
        const float RangeR = Set.Channel[0].Max - Set.Channel[0].Min, RangeG = Set.Channel[1].Max - Set.Channel[1].Min, RangeB = Set.Channel[2].Max - Set.Channel[2].Min;
        S.LandPeakSpacingM = (float)PeakSpacing;
        const float Range = std::max({ RangeR, RangeG, 1e-6f });
        S.LandPeakReliefM = Range * TexelMF;
        S.LandPeakMeanR = (float)VoxelNoiseTiles::MeanOf(Set, 0, [&](double V) { return Unit(0, V); });
        S.LandPeakMeanG = (float)VoxelNoiseTiles::MeanOf(Set, 1, [&](double V) { return Unit(1, V); });
        S.LandPeakScaleR = RangeR / Range;
        S.LandPeakScaleG = RangeG / Range;
        S.LandPeakScaleB = RangeB / Range;
        // FVoxelDensityCPU's detail (VoxelLand::PeakDetailTexelsPerCell).
        if (PeakDetail > 0)
        {
            const auto& D = P.Detail;
            S.LandPeakDetailTexelsPerCell = (float)(P.Cell * 0.01 / DetailTexelM);
            const float DTexelMF = S.LandCellM / S.LandPeakDetailTexelsPerCell;
            S.LandPeakDetailScaleHard = (float)PeakDetail * (float)P.DetailStats.RangeHard * DTexelMF / S.LandPeakReliefM;
            S.LandPeakDetailScaleSoft = (float)PeakDetail * (float)P.DetailStats.RangeSoft * DTexelMF / S.LandPeakReliefM;
            S.LandPeakDetailFade = (float)std::floor(std::log2((double)(D.N / D.Channel[0].CellsPerTile))) - 1.0f;
        }
    }
    S.LandPeakStrength = bPeaks ? (float)PeakStrength : 0.0f; S.LandPeakSeed = VoxelHashSeed(7, 0x41u);
    if (getenv("PEAK_RELIEF_START")) S.LandPeakReliefStart = (float)atof(getenv("PEAK_RELIEF_START"));
    if (getenv("PEAK_RELIEF_END")) S.LandPeakReliefEnd = (float)atof(getenv("PEAK_RELIEF_END"));
    S.LandPeakReliefEnd = std::max(S.LandPeakReliefEnd, S.LandPeakReliefStart + 1.0f);
    // The detail set's chain and means, whoever reads it (the detail layer, the peaks, the hills'
    // and the strata's sets).
    S.LandDetailBase = P.DetailBase; S.LandDetailN = P.Detail.N; S.LandDetailMips = P.Detail.Mips;
    S.LandDetailMeanHard = (float)P.DetailStats.MeanHard;
    S.LandDetailMeanSoft = (float)P.DetailStats.MeanSoft;
    // T4a: the hills from the set (FVoxelDensityCPU's; VoxelLand::HillTexelsPerCell), from the
    // environment: HILL_SPACING (m), HILL_STRENGTH, HILL_SWELL (m), HILL_SWELL_SCALE (the swells'
    // spacing over the hills'), HILL_DETAIL (strength), HILL_DETAIL_WAVELENGTH (m).
    if (!P.HillSet.Texels.empty() && S.LandHasForm && HillRelief > 0)
    {
        const auto& Set = P.HillSet;
        const double Spacing = getenv("HILL_SPACING") ? atof(getenv("HILL_SPACING")) : 2500.0;
        const double HStrength = getenv("HILL_STRENGTH") ? atof(getenv("HILL_STRENGTH")) : 1.0;
        const double Swell = getenv("HILL_SWELL") ? atof(getenv("HILL_SWELL")) : 20.0;
        const double SwellScale = getenv("HILL_SWELL_SCALE") ? atof(getenv("HILL_SWELL_SCALE")) : 3.0;
        const double HDetail = getenv("HILL_DETAIL") ? atof(getenv("HILL_DETAIL")) : 0.5;
        const double HDetailWave = std::clamp(getenv("HILL_DETAIL_WAVELENGTH") ? atof(getenv("HILL_DETAIL_WAVELENGTH")) : 300.0, 16.0, 0.25 * Spacing);
        const double TexelM = Spacing / (double)(Set.N / Set.Channel[0].CellsPerTile);
        S.LandHillTexelsPerCell = (float)(P.Cell * 0.01 / TexelM);
        const float TexelMF = S.LandCellM / S.LandHillTexelsPerCell;
        const float RangeR = Set.Channel[0].Max - Set.Channel[0].Min, RangeG = Set.Channel[1].Max - Set.Channel[1].Min, RangeB = Set.Channel[2].Max - Set.Channel[2].Min;
        const float Range = std::max({ RangeR, RangeG, 1e-6f });
        const float ReliefM = Range * TexelMF;
        S.LandHillRelief = (float)HStrength * ReliefM;
        S.LandHillBase = P.HillBase; S.LandHillN = Set.N; S.LandHillMips = Set.Mips;
        S.LandHillScaleR = RangeR / Range; S.LandHillScaleG = RangeG / Range; S.LandHillScaleB = RangeB / Range;
        {
            // N1b: R's and G's means (unit), for the face blend (VoxelLandNoiseDetail::MakePackLocked).
            auto HUnit = [&](int c, double V) { const auto& C = Set.Channel[c]; return (V - (double)C.Min) / std::max((double)C.Max - (double)C.Min, 1e-12); };
            S.LandHillMeanR = (float)VoxelNoiseTiles::MeanOf(Set, 0, [&](double V) { return HUnit(0, V); });
            S.LandHillMeanG = (float)VoxelNoiseTiles::MeanOf(Set, 1, [&](double V) { return HUnit(1, V); });
        }
        S.LandHillSpacingM = (float)Spacing;
        S.LandHillFoothill = getenv("HILL_FOOTHILL") ? (float)atof(getenv("HILL_FOOTHILL")) : 1.0f;
        if (Swell > 0 && SwellScale > 0)
        {
            S.LandHillSwellTexelsPerCell = (float)(S.LandHillTexelsPerCell / SwellScale);
            S.LandHillSwellM = (float)Swell;
            S.LandHillSwellScaleG = 1.0f;
        }
        if (HDetail > 0)
        {
            const auto& D = P.Detail;
            const double HDTexelM = HDetailWave / (double)(D.N / D.Channel[0].CellsPerTile);
            S.LandHillDetailTexelsPerCell = (float)(P.Cell * 0.01 / HDTexelM);
            const float DTexelMF = S.LandCellM / S.LandHillDetailTexelsPerCell;
            S.LandHillDetailScaleHard = (float)HDetail * (float)P.DetailStats.RangeHard * DTexelMF / ReliefM;
            S.LandHillDetailScaleSoft = (float)HDetail * (float)P.DetailStats.RangeSoft * DTexelMF / ReliefM;
            S.LandHillDetailFade = (float)std::floor(std::log2((double)(D.N / D.Channel[0].CellsPerTile))) - 1.0f;
        }
        printf("hills from %s: spacing %.0f m (texel %.1f m), relief %.0f m at strength %.2f; swells %.0f m at %.1fx; detail %.2f from %.0f m\n",
            getenv("HILL_SET"), Spacing, TexelMF, S.LandHillRelief, HStrength, S.LandHillSwellM, SwellScale, HDetail, HDetailWave);
        // T4c: the strata (FVoxelDensityCPU's; VoxelLand::StrataTexelsPerCell), from the
        // environment: STRATA_SPACING (m), STRATA_STRENGTH, STRATA_COVERAGE (share of the land),
        // STRATA_WIDTH (the switch's half-width), STRATA_FLAT, STRATA_DETAIL, STRATA_DETAIL_WAVELENGTH.
        const double Coverage = getenv("STRATA_COVERAGE") ? atof(getenv("STRATA_COVERAGE")) : 0.3;
        const double SStrength = getenv("STRATA_STRENGTH") ? atof(getenv("STRATA_STRENGTH")) : 1.0;
        if (!P.StrataSet.Texels.empty() && Coverage > 0 && SStrength > 0)
        {
            const auto& Ts = P.StrataSet;
            const double SSpacing = getenv("STRATA_SPACING") ? atof(getenv("STRATA_SPACING")) : 4000.0;
            const double SDetail = getenv("STRATA_DETAIL") ? atof(getenv("STRATA_DETAIL")) : 0.5;
            const double SDetailWave = std::clamp(getenv("STRATA_DETAIL_WAVELENGTH") ? atof(getenv("STRATA_DETAIL_WAVELENGTH")) : 400.0, 16.0, 0.25 * SSpacing);
            const double STexelM = SSpacing / (double)(Ts.N / Ts.Channel[0].CellsPerTile);
            S.LandStrataTexelsPerCell = (float)(P.Cell * 0.01 / STexelM);
            const float STexelMF = S.LandCellM / S.LandStrataTexelsPerCell;
            const float SR = Ts.Channel[0].Max - Ts.Channel[0].Min, SG = Ts.Channel[1].Max - Ts.Channel[1].Min, SB = Ts.Channel[2].Max - Ts.Channel[2].Min;
            const float SRange = std::max({ SR, SG, 1e-6f });
            const float SReliefM = SRange * STexelMF;
            S.LandStrataRelief = std::min((float)SStrength * SReliefM, (float)(SStrength * 0.5 * SSpacing));
            S.LandStrataBase = P.StrataBase; S.LandStrataN = Ts.N; S.LandStrataMips = Ts.Mips;
            S.LandStrataScaleR = SR / SRange; S.LandStrataScaleG = SG / SRange; S.LandStrataScaleB = SB / SRange;
            {
                auto TUnit = [&](int c, double V) { const auto& C = Ts.Channel[c]; return (V - (double)C.Min) / std::max((double)C.Max - (double)C.Min, 1e-12); };
                S.LandStrataMeanR = (float)VoxelNoiseTiles::MeanOf(Ts, 0, [&](double V) { return TUnit(0, V); });
                S.LandStrataMeanG = (float)VoxelNoiseTiles::MeanOf(Ts, 1, [&](double V) { return TUnit(1, V); });
            }
            S.LandStrataSpacingM = (float)SSpacing;
            S.LandStrataFlat = getenv("STRATA_FLAT") ? (float)atof(getenv("STRATA_FLAT")) : 0.35f;
            S.LandStrataWidth = getenv("STRATA_WIDTH") ? (float)atof(getenv("STRATA_WIDTH")) : 0.05f;
            S.LandStrataSeed = VoxelHashSeed(7, 0x51u);
            // FVoxelLandBake::ProvinceThreshold over the dump's land cells.
            {
                uint64_t Count[64] = {}, Total = 0;
                for (size_t i = 0; i < P.Province.size(); i++) if (P.Height[i] > 0.0f) { Count[std::min<int>(P.Province[i], 63)]++; Total++; }
                const double Wanted = std::min(Coverage, 1.0) * (double)Total;
                double Above = 0.0;
                float Th = (float)(-0.5 / 63.0);
                for (int q = 63; q >= 0; q--)
                {
                    const double C = (double)Count[q];
                    if (Above + C >= Wanted && C > 0.0) { Th = (float)((q + 0.5 - (Wanted - Above) / C) / 63.0); break; }
                    Above += C;
                }
                S.LandStrataThreshold = Total > 0 ? std::max(Th, S.LandStrataWidth) : 2.0f;   // as FVoxelDensityCPU
            }
            if (SDetail > 0)
            {
                const auto& D = P.Detail;
                const double SDTexelM = SDetailWave / (double)(D.N / D.Channel[0].CellsPerTile);
                S.LandStrataDetailTexelsPerCell = (float)(P.Cell * 0.01 / SDTexelM);
                const float DTexelMF = S.LandCellM / S.LandStrataDetailTexelsPerCell;
                S.LandStrataDetailScaleHard = (float)SDetail * (float)P.DetailStats.RangeHard * DTexelMF / SReliefM;
                S.LandStrataDetailScaleSoft = (float)SDetail * (float)P.DetailStats.RangeSoft * DTexelMF / SReliefM;
                S.LandStrataDetailFade = (float)std::floor(std::log2((double)(D.N / D.Channel[0].CellsPerTile))) - 1.0f;
            }
            printf("strata from %s: spacing %.0f m (texel %.1f m), relief %.0f m at strength %.2f; %.0f%% of the land (province above %.3f, +-%.3f); flat %.2f; detail %.2f from %.0f m\n",
                getenv("STRATA_SET"), SSpacing, STexelMF, S.LandStrataRelief, SStrength, 100.0 * Coverage, S.LandStrataThreshold, S.LandStrataWidth,
                S.LandStrataFlat, SDetail, SDetailWave);
        }
    }
    if (P.FormPacked.empty()) S.LandHasForm = 0;
    if (getenv("GULLY_DETAIL")) S.LandGullyDetail = (float)atof(getenv("GULLY_DETAIL"));
    printf("planet %s: R %.0f km, N %d, cell %.0f m, lambda0 %.0f m, %d gully octaves, gully reach %.0f m, heights %.0f..%.0f m, lattice finest %.0f cm; hills %.0f m, %d octaves; peaks %.2f, spacing %.0f m (texel %.1f m, set relief %.0f m), relief up to %.0f m%s\n",
        Tag.c_str(), RadiusM * 1e-3, N, P.Cell * 0.01, Lambda0, Octaves, GullyReach * 0.01, Mn, Mx, P.Finest, HillRelief, HillOctaves,
        S.LandPeakStrength, PeakSpacing, S.LandPeakTexelsPerCell > 0 ? S.LandCellM / S.LandPeakTexelsPerCell : 0.0f, S.LandPeakReliefM, P.MaxRelief,
        P.Form.empty() ? " (no landform in the dump)" : "");
}

struct FTerms { double Bake, Hills, Peaks, Gully, Detail, Height, Carve; float DetailScale; };

/** FVoxelDensityCPU::LandHeightAt at direction D (unit), Footprint (cm). Height in m above sea. */
/** The strata mode's switch: Terms leaves the strata out while set (T4c). */
static bool GNoStrata = false;

static FTerms Terms(const FPlanet& P, const double D[3], double Footprint, bool bGullies = true, bool bWarpJacobian = true, bool bHills = true, bool bPeaks = true)
{
    double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius };
    double Q[3], DW[3][3];
    for (int a = 0; a < 3; a++)
    {
        Q[a] = SP[a] + Evaluate(P.W[a], SP, 0.0, DW[a]) * P.WarpAmp;
        for (int b = 0; b < 3; b++) DW[a][b] *= bWarpJacobian ? P.WarpAmp : 0.0;
    }
    FSampler S = P.S;
    if (!bGullies) S.LandGullyStrength = 0;
    if (!bHills) S.LandHillRelief = 0;
    if (!bPeaks) S.LandPeakStrength = 0;
    if (GNoStrata) S.LandStrataTexelsPerCell = 0;
    const auto L = S.VoxelLandSurfaceDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), DF(Q[0]), DF(Q[1]), DF(Q[2]),
        float3((float)DW[0][0], (float)DW[0][1], (float)DW[0][2]), float3((float)DW[1][0], (float)DW[1][1], (float)DW[1][2]),
        float3((float)DW[2][0], (float)DW[2][1], (float)DW[2][2]), (float)std::max(Footprint, P.Finest), false);
    const float Wd = lerp(0.25f, 1.0f, smoothstep(-50.0f, 50.0f, L.Bake));
    const double Dd = (double)S.VoxelLandDetailDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), (float)std::max(Footprint, P.Finest), false).Unit;
    FTerms T;
    T.Bake = L.Bake; T.Hills = L.Hills; T.Peaks = L.Peaks; T.Gully = L.Gully; T.DetailScale = L.DetailScale; T.Carve = L.Carve;
    T.Detail = Dd * (double)((float)P.DetailAmp * Wd * L.DetailScale) * 0.01;
    T.Height = T.Bake + T.Hills + T.Peaks + T.Gully + T.Carve + T.Detail;
    return T;
}

// ---------------------------------------------------------------------------------------------
// Images
// ---------------------------------------------------------------------------------------------

static void WritePPM(const std::string& Name, int W, int H, const std::vector<float3>& Px)
{
    FILE* F = fopen(Name.c_str(), "wb");
    fprintf(F, "P6\n%d %d\n255\n", W, H);
    for (const float3& C : Px)
    {
        unsigned char B[3] = { (unsigned char)std::clamp((int)std::lround(C.x * 255), 0, 255), (unsigned char)std::clamp((int)std::lround(C.y * 255), 0, 255),
            (unsigned char)std::clamp((int)std::lround(C.z * 255), 0, 255) };
        fwrite(B, 1, 3, F);
    }
    fclose(F);
}

static void Normalize(double V[3]) { const double L = std::sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]); for (int a = 0; a < 3; a++) V[a] /= L; }
static void Frame(const double C[3], double E1[3], double E2[3])
{
    double Up[3] = { 0, 0, 1 };
    if (std::fabs(C[2]) > 0.9) { Up[0] = 1; Up[2] = 0; }
    E1[0] = Up[1] * C[2] - Up[2] * C[1]; E1[1] = Up[2] * C[0] - Up[0] * C[2]; E1[2] = Up[0] * C[1] - Up[1] * C[0]; Normalize(E1);
    E2[0] = C[1] * E1[2] - C[2] * E1[1]; E2[1] = C[2] * E1[0] - C[0] * E1[2]; E2[2] = C[0] * E1[1] - C[1] * E1[0];
}

/** Hillshaded height map (px x px) of a square WidthM wide around C, sampled at the pixel
 *  footprint (as a mesh of that cell size would be). Mode 0 no gullies, 1 gullies, 2 gully term. */
static std::vector<float3> Render(const FPlanet& P, const double C[3], double WidthM, int Px, int Mode, std::vector<double>* OutH = nullptr)
{
    double E1[3], E2[3]; Frame(C, E1, E2);
    const double Pix = WidthM / Px;
    std::vector<double> H(Px * Px), Hb(Px * Px);
    ParallelFor(Px * Px, [&](int k) {
        const int x = k % Px, y = k / Px;
        const double U = ((x + 0.5) / Px - 0.5) * WidthM * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * WidthM * 100.0 / P.SeaRadius;
        double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
        Normalize(D);
        const FTerms T = Terms(P, D, Pix * 100.0, Mode != 0);
        H[k] = Mode == 2 ? T.Gully : T.Height;
        Hb[k] = T.Bake;
    });
    if (OutH) *OutH = H;
    std::vector<float3> Out(Px * Px);
    for (int y = 0; y < Px; y++)
        for (int x = 0; x < Px; x++)
        {
            const int k = y * Px + x;
            if (Mode == 2)
            {
                const double Sc = 0.5 * P.Strength * 2.0 * P.Cell * 0.01 * 0.8;
                const double t = std::clamp(H[k] / Sc, -1.0, 1.0);
                Out[k] = t < 0 ? float3(1 + 0.8f * (float)t, 1 + 0.6f * (float)t, 1.0f) : float3(1.0f, 1 - 0.7f * (float)t, 1 - 0.8f * (float)t);
                continue;
            }
            const int x0 = std::max(x - 1, 0), x1 = std::min(x + 1, Px - 1), y0 = std::max(y - 1, 0), y1 = std::min(y + 1, Px - 1);
            const double dx = (H[y * Px + x1] - H[y * Px + x0]) / ((x1 - x0) * Pix), dy = (H[y0 * Px + x] - H[y1 * Px + x]) / ((y1 - y0) * Pix);
            // Light from the north-west, 45 deg up.
            const double Nx = -dx, Ny = -dy, Nz = 1, L = std::sqrt(Nx * Nx + Ny * Ny + Nz * Nz);
            const double Shade = std::clamp((Nx * -0.5 + Ny * 0.5 + Nz * 0.7071) / L, 0.0, 1.0);
            float3 Base;
            const double h = H[k];
            if (h <= 0) Base = float3(0.25f, 0.4f, 0.6f);
            else
            {
                const double t = std::min(h / 4000.0, 1.0);
                Base = float3((float)(0.45 + 0.4 * t), (float)(0.55 + 0.25 * t), (float)(0.35 + 0.5 * t));
            }
            Out[k] = Base * (float)(0.25 + 0.85 * Shade);
        }
    return Out;
}

/** Hillshade of a square WidthM wide around C (as Render), with the hills and the gullies on or
 *  off; bTerm draws the hills term alone (white 0 to brown at Scale m). */
static std::vector<float3> RenderHills(const FPlanet& P, const double C[3], double WidthM, int Px, bool bHills, bool bGullies, bool bTerm, double Scale,
    std::vector<double>* OutH = nullptr)
{
    double E1[3], E2[3]; Frame(C, E1, E2);
    const double Pix = WidthM / Px;
    std::vector<double> H(Px * Px);
    ParallelFor(Px * Px, [&](int k) {
        const int x = k % Px, y = k / Px;
        const double U = ((x + 0.5) / Px - 0.5) * WidthM * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * WidthM * 100.0 / P.SeaRadius;
        double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
        Normalize(D);
        const FTerms T = Terms(P, D, Pix * 100.0, bGullies, true, bHills);
        H[k] = bTerm ? T.Hills : T.Height;
    });
    if (OutH) *OutH = H;
    std::vector<float3> Out(Px * Px);
    for (int y = 0; y < Px; y++)
        for (int x = 0; x < Px; x++)
        {
            const int k = y * Px + x;
            if (bTerm)
            {
                const float t = (float)std::clamp(H[k] / Scale, 0.0, 1.0);
                Out[k] = float3(1.0f - 0.45f * t, 1.0f - 0.6f * t, 1.0f - 0.8f * t);
                continue;
            }
            const int x0 = std::max(x - 1, 0), x1 = std::min(x + 1, Px - 1), y0 = std::max(y - 1, 0), y1 = std::min(y + 1, Px - 1);
            const double dx = (H[y * Px + x1] - H[y * Px + x0]) / ((x1 - x0) * Pix), dy = (H[y0 * Px + x] - H[y1 * Px + x]) / ((y1 - y0) * Pix);
            const double Nx = -dx, Ny = -dy, Nz = 1, L = std::sqrt(Nx * Nx + Ny * Ny + Nz * Nz);
            const double Shade = std::clamp((Nx * -0.5 + Ny * 0.5 + Nz * 0.7071) / L, 0.0, 1.0);
            float3 Base;
            const double h = H[k];
            if (h <= 0) Base = float3(0.25f, 0.4f, 0.6f);
            else
            {
                const double t = std::min(h / 2500.0, 1.0);
                Base = float3((float)(0.42 + 0.4 * t), (float)(0.56 + 0.22 * t), (float)(0.33 + 0.4 * t));
            }
            Out[k] = Base * (float)(0.25 + 0.85 * Shade);
        }
    return Out;
}

/** The surface around C (Px x Px pixels, WidthM wide): shaded (Term 0), or a term: 1 the peaks
 *  (signed over Scale m, blue down, red up). bPeaks, bGullies as Terms. */
static std::vector<float3> RenderPeaks(const FPlanet& P, const double C[3], double WidthM, int Px, bool bPeaks, bool bGullies, int Term, double Scale,
    std::vector<double>* OutH = nullptr)
{
    double E1[3], E2[3]; Frame(C, E1, E2);
    const double Pix = WidthM / Px;
    std::vector<double> H(Px * Px);
    ParallelFor(Px * Px, [&](int k) {
        const int x = k % Px, y = k / Px;
        const double U = ((x + 0.5) / Px - 0.5) * WidthM * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * WidthM * 100.0 / P.SeaRadius;
        double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
        Normalize(D);
        const FTerms T = Terms(P, D, Pix * 100.0, bGullies, true, true, bPeaks);
        H[k] = Term == 1 ? T.Peaks : T.Height;
    });
    if (OutH) *OutH = H;
    std::vector<float3> Out(Px * Px);
    for (int y = 0; y < Px; y++)
        for (int x = 0; x < Px; x++)
        {
            const int k = y * Px + x;
            if (Term == 1)
            {
                const float t = (float)std::clamp(H[k] / Scale, -1.0, 1.0);
                Out[k] = t >= 0 ? float3(1.0f, 1.0f - 0.75f * t, 1.0f - 0.8f * t) : float3(1.0f + 0.8f * t, 1.0f + 0.6f * t, 1.0f);
                continue;
            }
            const int x0 = std::max(x - 1, 0), x1 = std::min(x + 1, Px - 1), y0 = std::max(y - 1, 0), y1 = std::min(y + 1, Px - 1);
            const double dx = (H[y * Px + x1] - H[y * Px + x0]) / ((x1 - x0) * Pix), dy = (H[y0 * Px + x] - H[y1 * Px + x]) / ((y1 - y0) * Pix);
            const double Nx = -dx, Ny = -dy, Nz = 1, L = std::sqrt(Nx * Nx + Ny * Ny + Nz * Nz);
            const double Shade = std::clamp((Nx * -0.5 + Ny * 0.5 + Nz * 0.7071) / L, 0.0, 1.0);
            float3 Base;
            const double h = H[k];
            if (h <= 0) Base = float3(0.25f, 0.4f, 0.6f);
            else
            {
                const double t = std::min(h / 5000.0, 1.0);
                Base = float3((float)(0.42 + 0.45 * t), (float)(0.56 + 0.3 * t), (float)(0.33 + 0.55 * t));
            }
            Out[k] = Base * (float)(0.25 + 0.85 * Shade);
        }
    return Out;
}

/** A crest of high local relief (relief at least MinRelief m, drainage within two cells' of its
 *  own): a view centre for the peaks. Which picks among the candidates. */
static bool FindCrest(const FPlanet& P, double MinRelief, int Which, double OutC[3])
{
    // CENTERS="f,i,j;f,i,j;...": the views at these bake cells instead (T3h: chosen regions).
    if (const char* List = getenv("CENTERS"))
    {
        std::string L(List);
        for (int k = 0; k < Which && L.find(';') != std::string::npos; k++) L = L.substr(L.find(';') + 1);
        int f = 0, i = 0, j = 0;
        if (sscanf(L.c_str(), "%d,%d,%d", &f, &i, &j) != 3) return false;
        const float3 D = VoxelCubeDirection(f, VoxelCubeCellCentre(i, P.N), VoxelCubeCellCentre(j, P.N));
        OutC[0] = D.x; OutC[1] = D.y; OutC[2] = D.z;
        return true;
    }
    const int Side = P.Side();
    std::vector<std::pair<float, int>> Cand;
    for (int f = 0; f < 6; f++)
        for (int j = 16; j < P.N - 16; j += 4)
            for (int i = 16; i < P.N - 16; i += 4)
            {
                const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                const float R = VoxelLandRelief::Decode(P.Relief[c]);
                if (R < MinRelief || P.FlowF[c] > 2 * std::log2(P.Cell * 0.01) + 1.0) continue;
                Cand.push_back({ R, c });
            }
    if (Cand.empty()) return false;
    std::sort(Cand.begin(), Cand.end(), [](auto& a, auto& b) { return a.first > b.first; });
    const int c = Cand[std::min((size_t)Which * 23, Cand.size() - 1)].second;
    const int f = c / (Side * Side), r = c % (Side * Side), j = r / Side - P.Pad, i = r % Side - P.Pad;
    const float3 D = VoxelCubeDirection(f, VoxelCubeCellCentre(i, P.N), VoxelCubeCellCentre(j, P.N));
    OutC[0] = D.x; OutC[1] = D.y; OutC[2] = D.z;
    return true;
}

/** A land cell of the given landform (hilliness in [HMin, HMax], at least 200 m up, a gentle
 *  bake): a view centre for the hills. Which picks among the candidates. */
static bool FindLandform(const FPlanet& P, double HMin, double HMax, int Which, double OutC[3])
{
    if (P.Form.empty()) return false;
    const int Side = P.Side();
    std::vector<int> Cand;
    for (int f = 0; f < 6; f++)
        for (int j = 16; j < P.N - 16; j += 8)
            for (int i = 16; i < P.N - 16; i += 8)
            {
                const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                const double Hl = (P.Form[c] & 0xffu) / 255.0;
                if (Hl < HMin || Hl > HMax || P.Height[c] < 200 || P.Height[c] > 2000 || P.FlowF[c] > 2 * std::log2(P.Cell * 0.01) + 3) continue;
                Cand.push_back(c);
            }
    if (Cand.empty()) return false;
    const int c = Cand[(size_t)(Which * 7919) % Cand.size()];
    const int f = c / (Side * Side), r = c % (Side * Side), j = r / Side - P.Pad, i = r % Side - P.Pad;
    const float3 D = VoxelCubeDirection(f, VoxelCubeCellCentre(i, P.N), VoxelCubeCellCentre(j, P.N));
    OutC[0] = D.x; OutC[1] = D.y; OutC[2] = D.z;
    return true;
}

/** Slope percentiles (degrees) of a height grid Px x Px with pixel Pix (m). */
static void SlopeStats(const std::vector<double>& H, int Px, double Pix, double Out[3])
{
    std::vector<double> S;
    for (int y = 1; y < Px - 1; y++)
        for (int x = 1; x < Px - 1; x++)
        {
            const double dx = (H[y * Px + x + 1] - H[y * Px + x - 1]) / (2 * Pix), dy = (H[(y + 1) * Px + x] - H[(y - 1) * Px + x]) / (2 * Pix);
            S.push_back(std::atan(std::sqrt(dx * dx + dy * dy)) * 57.29578);
        }
    std::sort(S.begin(), S.end());
    Out[0] = S[S.size() / 2]; Out[1] = S[S.size() * 9 / 10]; Out[2] = S[S.size() * 99 / 100];
}

/** A land cell with steep slopes and some height: the view centre. */
static void FindMountain(const FPlanet& P, double OutC[3], int Which)
{
    const int Side = P.Side();
    std::vector<std::pair<float, int>> Cand;
    for (int f = 0; f < 6; f++)
        for (int j = 8; j < P.N - 8; j += 4)
            for (int i = 8; i < P.N - 8; i += 4)
            {
                const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                const float h = P.Height[c];
                if (h < 1500 || h > 3500) continue;
                const float s = std::fabs(P.Height[c + 1] - P.Height[c - 1]) + std::fabs(P.Height[c + Side] - P.Height[c - Side]);
                Cand.push_back({ s, c });
            }
    std::sort(Cand.begin(), Cand.end(), [](auto& a, auto& b) { return a.first > b.first; });
    const int c = Cand[std::min((size_t)Which * 37, Cand.size() - 1)].second;
    const int f = c / (Side * Side), r = c % (Side * Side), j = r / Side - P.Pad, i = r % Side - P.Pad;
    const float3 D = VoxelCubeDirection(f, VoxelCubeCellCentre(i, P.N), VoxelCubeCellCentre(j, P.N));
    OutC[0] = D.x; OutC[1] = D.y; OutC[2] = D.z;
}

static void Sheet(const std::string& Name, const std::vector<std::vector<float3>>& Images, int Px)
{
    const int Gap = 8, W = (int)Images.size() * Px + ((int)Images.size() - 1) * Gap;
    std::vector<float3> Out(W * Px, float3(1.0f));
    for (size_t n = 0; n < Images.size(); n++)
        for (int y = 0; y < Px; y++)
            for (int x = 0; x < Px; x++) Out[y * W + n * (Px + Gap) + x] = Images[n][y * Px + x];
    WritePPM(Name, W, Px, Out);
}


/** The GPU path's globals (VoxelDensitySample.ush, VoxelLandParameters.ush) for planet P. */
static void SetGpuGlobals(const FPlanet& P, std::vector<FVoxelOctave>& Gpu)
{
    for (int a = 0; a < 3; a++) for (auto& O : P.W[a].O) Gpu.push_back(ToGPU(O));
    VoxelShader::Octaves = Gpu.data();
    ContinentOctaves = 0; MountainOctaves = 0; DetailOctaves = 0;
    InvNormDetail = DF(0.0); PlanetRadius = DF(P.R); NoiseAmplitude = DF(P.NA); HalfFinest = DF(P.Half);
    VoxelShader::LandHeight = P.Height.data(); VoxelShader::LandTanTable = P.Tan.data(); VoxelShader::LandFlow = P.FlowPacked.data();
    VoxelShader::LandN = P.N; VoxelShader::LandPad = P.Pad; VoxelShader::LandHasFlow = 1; VoxelShader::LandCellM = P.S.LandCellM;
    LandWarpOctaves = 3; LandWarpInvNorm = (float)P.W[0].InvNorm; LandWarpAmplitude = (float)P.WarpAmp;
    LandDetailAmplitude = (float)P.DetailAmp; LandHeightOffset = DF(P.Offset);
    VoxelShader::LandGullyStrength = P.S.LandGullyStrength; VoxelShader::LandGullyOctaves = P.S.LandGullyOctaves;
    VoxelShader::LandGullyFlowGain = P.S.LandGullyFlowGain; VoxelShader::LandGullyDetail = P.S.LandGullyDetail; VoxelShader::LandGullySeed = P.S.LandGullySeed;
    VoxelShader::LandForm = P.FormPacked.data(); VoxelShader::LandHasForm = P.S.LandHasForm;
    VoxelShader::LandHillRelief = P.S.LandHillRelief; VoxelShader::LandHillFrequency = P.S.LandHillFrequency;
    VoxelShader::LandHillOctaves = P.S.LandHillOctaves; VoxelShader::LandHillSeed = P.S.LandHillSeed;
    VoxelShader::LandRelief = P.S.LandRelief; VoxelShader::LandNoise = P.S.LandNoise;
    VoxelShader::LandPeakStrength = P.S.LandPeakStrength; VoxelShader::LandPeakTexelsPerCell = P.S.LandPeakTexelsPerCell; VoxelShader::LandPeakSeed = P.S.LandPeakSeed;
    VoxelShader::LandPeakSpacingM = P.S.LandPeakSpacingM; VoxelShader::LandPeakReliefM = P.S.LandPeakReliefM;
    VoxelShader::LandPeakReliefStart = P.S.LandPeakReliefStart; VoxelShader::LandPeakReliefEnd = P.S.LandPeakReliefEnd;
    VoxelShader::LandPeakMeanR = P.S.LandPeakMeanR; VoxelShader::LandPeakMeanG = P.S.LandPeakMeanG;
    VoxelShader::LandPeakScaleR = P.S.LandPeakScaleR; VoxelShader::LandPeakScaleG = P.S.LandPeakScaleG; VoxelShader::LandPeakScaleB = P.S.LandPeakScaleB;
    VoxelShader::LandNoiseN = P.S.LandNoiseN; VoxelShader::LandNoiseMips = P.S.LandNoiseMips;
    VoxelShader::LandDetailBase = P.S.LandDetailBase; VoxelShader::LandDetailN = P.S.LandDetailN;
    VoxelShader::LandDetailMips = P.S.LandDetailMips; VoxelShader::LandPeakDetailTexelsPerCell = P.S.LandPeakDetailTexelsPerCell;
    VoxelShader::LandDetailMeanHard = P.S.LandDetailMeanHard; VoxelShader::LandDetailMeanSoft = P.S.LandDetailMeanSoft;
    VoxelShader::LandPeakDetailScaleHard = P.S.LandPeakDetailScaleHard; VoxelShader::LandPeakDetailScaleSoft = P.S.LandPeakDetailScaleSoft;
    VoxelShader::LandPeakDetailFade = P.S.LandPeakDetailFade;
    VoxelShader::LandHillTexelsPerCell = P.S.LandHillTexelsPerCell; VoxelShader::LandHillBase = P.S.LandHillBase;
    VoxelShader::LandHillN = P.S.LandHillN; VoxelShader::LandHillMips = P.S.LandHillMips;
    VoxelShader::LandHillScaleR = P.S.LandHillScaleR; VoxelShader::LandHillScaleG = P.S.LandHillScaleG; VoxelShader::LandHillScaleB = P.S.LandHillScaleB;
    VoxelShader::LandHillMeanR = P.S.LandHillMeanR; VoxelShader::LandHillMeanG = P.S.LandHillMeanG;
    VoxelShader::LandHillSpacingM = P.S.LandHillSpacingM;
    VoxelShader::LandHillSwellTexelsPerCell = P.S.LandHillSwellTexelsPerCell; VoxelShader::LandHillSwellM = P.S.LandHillSwellM;
    VoxelShader::LandHillSwellScaleG = P.S.LandHillSwellScaleG;
    VoxelShader::LandHillDetailTexelsPerCell = P.S.LandHillDetailTexelsPerCell; VoxelShader::LandHillDetailScaleHard = P.S.LandHillDetailScaleHard;
    VoxelShader::LandHillDetailScaleSoft = P.S.LandHillDetailScaleSoft; VoxelShader::LandHillDetailFade = P.S.LandHillDetailFade;
    VoxelShader::LandHillFoothill = P.S.LandHillFoothill;
    VoxelShader::LandProvince = P.S.LandProvince;
    VoxelShader::LandRiverTiles = P.S.LandRiverTiles; VoxelShader::LandRiverData = P.S.LandRiverData;
    VoxelShader::LandRiverTilesPerFace = P.S.LandRiverTilesPerFace; VoxelShader::LandRiverRadiusM = P.S.LandRiverRadiusM;
    VoxelShader::LandRiverWidthScale = P.S.LandRiverWidthScale; VoxelShader::LandRiverValleyScale = P.S.LandRiverValleyScale;
    VoxelShader::LandWater = P.S.LandWater; VoxelShader::LandHasWater = P.S.LandHasWater; VoxelShader::LandRiverCarve = P.S.LandRiverCarve;
    VoxelShader::LandRiverDepthScale = P.S.LandRiverDepthScale; VoxelShader::LandRiverFloodplainScale = P.S.LandRiverFloodplainScale;
    VoxelShader::LandRiverMinLog2Area = P.S.LandRiverMinLog2Area; VoxelShader::LandLakeShoreBand = P.S.LandLakeShoreBand;
    VoxelShader::LandRiverMeander = P.S.LandRiverMeander; VoxelShader::LandRiverFalls = P.S.LandRiverFalls;
    VoxelShader::LandStrataTexelsPerCell = P.S.LandStrataTexelsPerCell; VoxelShader::LandStrataBase = P.S.LandStrataBase;
    VoxelShader::LandStrataN = P.S.LandStrataN; VoxelShader::LandStrataMips = P.S.LandStrataMips;
    VoxelShader::LandStrataScaleR = P.S.LandStrataScaleR; VoxelShader::LandStrataScaleG = P.S.LandStrataScaleG; VoxelShader::LandStrataScaleB = P.S.LandStrataScaleB;
    VoxelShader::LandStrataMeanR = P.S.LandStrataMeanR; VoxelShader::LandStrataMeanG = P.S.LandStrataMeanG;
    VoxelShader::LandStrataSpacingM = P.S.LandStrataSpacingM; VoxelShader::LandStrataRelief = P.S.LandStrataRelief;
    VoxelShader::LandStrataFlat = P.S.LandStrataFlat; VoxelShader::LandStrataThreshold = P.S.LandStrataThreshold;
    VoxelShader::LandStrataWidth = P.S.LandStrataWidth; VoxelShader::LandStrataSeed = P.S.LandStrataSeed;
    VoxelShader::LandStrataDetailTexelsPerCell = P.S.LandStrataDetailTexelsPerCell; VoxelShader::LandStrataDetailScaleHard = P.S.LandStrataDetailScaleHard;
    VoxelShader::LandStrataDetailScaleSoft = P.S.LandStrataDetailScaleSoft; VoxelShader::LandStrataDetailFade = P.S.LandStrataDetailFade;
    VoxelShader::LandDetailTexelsPerCell = P.S.LandDetailTexelsPerCell; VoxelShader::LandDetailScales = P.S.LandDetailScales;
    VoxelShader::LandDetailMeanA = P.S.LandDetailMeanA; VoxelShader::LandDetailScaleA = P.S.LandDetailScaleA; VoxelShader::LandDetailScaleGain = P.S.LandDetailScaleGain;
    VoxelShader::LandDetailFade = P.S.LandDetailFade; VoxelShader::LandDetailSeed = P.S.LandDetailSeed;
}
/** pcost's profile pass (kept out of line so a profiler can pick it out). */
__attribute__((noinline)) static double PcostPass(const FPlanet& P, const std::vector<std::array<double, 3>>& Dirs, bool bPeaks)
{
    double Sink = 0;
    for (auto& D : Dirs) Sink += Terms(P, D.data(), P.Finest, true, true, true, bPeaks).Height;
    return Sink;
}

// ---------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    const std::string Tag = argc > 1 ? argv[1] : "n128";
    const int N = argc > 2 ? atoi(argv[2]) : 128;
    const double RadiusM = argc > 3 ? atof(argv[3]) : 1.0e6;
    const std::string Mode = argc > 4 ? argv[4] : "look";
    FPlanet P;
    Load(P, Tag, N, RadiusM, 30.0, HarnessGullyStrength());

    if (Mode == "look")
    {
        const int Views = getenv("VIEWS") ? atoi(getenv("VIEWS")) : 2;
        for (int v = 0; v < Views; v++)
        {
            double C[3]; FindMountain(P, C, v * 3);
            std::vector<double> Widths = { 60000.0, 15000.0, 4000.0 };
            if (getenv("WIDTH")) Widths = { atof(getenv("WIDTH")) };
            for (double Width : Widths)
            {
                const int Px = 512;
                auto A = Render(P, C, Width, Px, 0), B = Render(P, C, Width, Px, 1), G = Render(P, C, Width, Px, 2);
                char Name[256]; snprintf(Name, sizeof Name, "%s%s_look%d_%.0fkm.ppm", Tag.c_str(), getenv("SUFFIX") ? getenv("SUFFIX") : "", v, Width / 1000);
                Sheet(Name, { A, B, G }, Px);
                printf("wrote %s (left: L2 bake + detail; middle: + gullies; right: the gully term)\n", Name);
            }
        }
    }
    else if (Mode == "hills")
    {
        // The hills layer (T2) by landform: hill country, rolling plains, flat plains. Each sheet:
        // without hills and gullies; with the hills; with hills and gullies (the full surface);
        // the hills term. Slopes at the pixel size, with and without the hills.
        if (P.Form.empty()) { printf("the dump has no landform (Tag_form.u32)\n"); return 1; }
        struct FKind { const char* Name; double Lo, Hi; };
        const FKind Kinds[] = { { "hillcountry", 0.75, 1.0 }, { "rolling", 0.3, 0.6 }, { "flat", 0.0, 0.12 } };
        const int Views = getenv("VIEWS") ? atoi(getenv("VIEWS")) : 1;
        std::vector<double> Widths = { 30000.0, 8000.0, 2000.0 };
        if (getenv("WIDTH")) Widths = { atof(getenv("WIDTH")) };
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        for (const FKind& K : Kinds)
            for (int v = 0; v < Views; v++)
            {
                double C[3];
                if (!FindLandform(P, K.Lo, K.Hi, v + 1, C)) { printf("no %s cell\n", K.Name); continue; }
                for (double Width : Widths)
                {
                    std::vector<double> H0, H1;
                    auto A = RenderHills(P, C, Width, Px, false, false, false, 1.0, &H0);
                    auto B = RenderHills(P, C, Width, Px, true, false, false, 1.0);
                    auto F = RenderHills(P, C, Width, Px, true, true, false, 1.0, &H1);
                    auto T = RenderHills(P, C, Width, Px, true, false, true, P.S.LandHillRelief);
                    char Name[256]; snprintf(Name, sizeof Name, "%s%s_hills_%s%d_%.0fkm.ppm", Tag.c_str(), getenv("SUFFIX") ? getenv("SUFFIX") : "", K.Name, v, Width / 1000);
                    Sheet(Name, { A, B, F, T }, Px);
                    double S0[3], S1[3]; SlopeStats(H0, Px, Width / Px, S0); SlopeStats(H1, Px, Width / Px, S1);
                    printf("wrote %s: slope p50/p90/p99 %.1f/%.1f/%.1f deg without hills and gullies, %.1f/%.1f/%.1f with\n",
                        Name, S0[0], S0[1], S0[2], S1[0], S1[1], S1[2]);
                }
            }
    }
    else if (Mode == "rivers")
    {
        // R1b: the bake's surface over the river floors at the nodes (the lookup's own checks,
        // against brute force and across tile edges, are Tools/LandRiverHarness's).
        if (P.Rivers.Tiles.empty()) { printf("needs a dump with the receivers (Tag_rec.u32)\n"); return 1; }
        std::vector<double> Below;
        VoxelShader::FRiverBuildShader Keys;
        for (size_t n = 0; n < P.Rivers.NumNodes; n += 7)
        {
            if (P.Rivers.NodeKind[n] != 0) continue;
            const float* D = &P.Rivers.NodePos[4 * n];
            const auto Bk = P.S.VoxelLandBakeSampleDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), false);
            Below.push_back((double)Bk.Height - Keys.VoxelRiverFloorOf(P.Rivers.NodeFloor[n]));
        }
        std::sort(Below.begin(), Below.end());
        auto Pc = [&](double q) { return Below[(size_t)(q * (Below.size() - 1))]; };
        printf("the bake's height over the river floor at %zu nodes: p1 %.1f, p10 %.1f, p50 %.1f, p90 %.1f, p99 %.1f m\n", Below.size(), Pc(0.01), Pc(0.1), Pc(0.5), Pc(0.9), Pc(0.99));
    }
    else if (Mode == "strata")
    {
        // The strata (T4c): views in strata country (province well above the threshold) in hill
        // country and on the flats, and on the switch from the hills. Each sheet: the full
        // surface without the strata (the hills there), with them, the strata's share (orange)
        // over the land. Slopes at the pixel size. CENTERS picks the views instead.
        if (P.Form.empty() || !(P.S.LandStrataTexelsPerCell > 0.0f)) { printf("needs a landform and STRATA_SET\n"); return 1; }
        const float Th = P.S.LandStrataThreshold, W = P.S.LandStrataWidth;
        struct FKind { const char* Name; float PLo, PHi, HLo, HHi; };
        const FKind Kinds[] = { { "country", Th + 2 * W, 2.0f, 0.6f, 1.0f }, { "switch", Th - 0.3f * W, Th + 0.3f * W, 0.5f, 1.0f }, { "flats", Th + 2 * W, 2.0f, 0.0f, 0.25f } };
        std::vector<double> Widths = { 40000.0, 10000.0, 3000.0 };
        if (getenv("WIDTH")) Widths = { atof(getenv("WIDTH")) };
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        const int Side = P.Side();
        int v = 0;
        for (const FKind& K : Kinds)
        {
            double C[3];
            if (getenv("CENTERS"))
            {
                if (!FindCrest(P, 0.0, v++, C)) break;
            }
            else
            {
                // Land cells of the kind, low relief (not the peaks'), away from rivers; the
                // VIEW-th of them (by a stride).
                std::vector<int> Cand;
                for (int f = 0; f < 6; f++)
                    for (int j = 16; j < P.N - 16; j += 3)
                        for (int i = 16; i < P.N - 16; i += 3)
                        {
                            const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                            const float Pv = P.Province[c] / 63.0f, Hl = (P.Form[c] & 0xffu) / 255.0f;
                            if (P.Height[c] < 60.0f || Pv < K.PLo || Pv > K.PHi || Hl < K.HLo || Hl > K.HHi) continue;
                            if (VoxelLandRelief::Decode(P.Relief[c]) > 450.0f) continue;
                            Cand.push_back(c);
                        }
                if (Cand.empty()) { printf("no %s cell\n", K.Name); continue; }
                const int Pick = getenv("VIEW") ? atoi(getenv("VIEW")) : 0;
                const int c = Cand[((size_t)Pick * 7919 + Cand.size() / 2) % Cand.size()];
                const int f = c / (Side * Side), r = c % (Side * Side), j = r / Side - P.Pad, i = r % Side - P.Pad;
                const float3 D = VoxelCubeDirection(f, VoxelCubeCellCentre(i, P.N), VoxelCubeCellCentre(j, P.N));
                C[0] = D.x; C[1] = D.y; C[2] = D.z;
                printf("%s: cell %d,%d,%d\n", K.Name, f, i, j);
            }
            for (double Width : Widths)
            {
                std::vector<double> H0, H1;
                GNoStrata = true;
                auto A = RenderPeaks(P, C, Width, Px, true, true, 0, 1.0, &H0);
                GNoStrata = false;
                auto B = RenderPeaks(P, C, Width, Px, true, true, 0, 1.0, &H1);
                // The strata's share at each pixel (the bake's province through the switch).
                double E1[3], E2[3]; Frame(C, E1, E2);
                std::vector<float3> Sh(Px * Px);
                double ShareSum = 0;
                for (int k = 0; k < Px * Px; k++)
                {
                    const int x = k % Px, y = k / Px;
                    const double U = ((x + 0.5) / Px - 0.5) * Width * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * Width * 100.0 / P.SeaRadius;
                    double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
                    Normalize(D);
                    const auto Bk = P.S.VoxelLandBakeSampleDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), true);
                    const float St = P.S.VoxelLandStrataShare(Bk.Province);
                    ShareSum += St;
                    const float3 Base = B[k];
                    Sh[k] = H1[k] <= 0.0 ? Base : Base * (1.0f - 0.6f * St) + float3(0.95f, 0.5f, 0.15f) * (0.6f * St);
                }
                char Name[256]; snprintf(Name, sizeof Name, "%s%s_strata_%s_%.0fkm.ppm", Tag.c_str(), getenv("SUFFIX") ? getenv("SUFFIX") : "", K.Name, Width / 1000);
                Sheet(Name, { A, B, Sh }, Px);
                double S0[3], S1[3]; SlopeStats(H0, Px, Width / Px, S0); SlopeStats(H1, Px, Width / Px, S1);
                double Lo = 1e9, Hi = -1e9; for (size_t k = 0; k < H0.size(); k++) { Lo = std::min(Lo, H1[k] - H0[k]); Hi = std::max(Hi, H1[k] - H0[k]); }
                printf("wrote %s: strata share %.2f; slope p50/p90/p99 %.1f/%.1f/%.1f deg with the hills, %.1f/%.1f/%.1f with the strata; change %.0f .. %.0f m\n",
                    Name, ShareSum / (Px * Px), S0[0], S0[1], S0[2], S1[0], S1[1], S1[2], Lo, Hi);
            }
        }
    }
    else if (Mode == "peaks")
    {
        // The peaks layer (T3) on crests: each sheet without peaks and gullies; with the peaks;
        // the full surface; the peaks term (over 600 m). Slopes at the pixel size.
        const int Views = getenv("VIEWS") ? atoi(getenv("VIEWS")) : 3;
        std::vector<double> Widths = { 30000.0, 8000.0, 2000.0 };
        if (getenv("WIDTH")) Widths = { atof(getenv("WIDTH")) };
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        for (int v = 0; v < Views; v++)
        {
            double C[3];
            if (!FindCrest(P, 1200.0, v, C)) { printf("no crest\n"); break; }
            for (double Width : Widths)
            {
                std::vector<double> H0, H1, H2;
                auto A = RenderPeaks(P, C, Width, Px, false, false, 0, 1.0, &H0);
                auto B = RenderPeaks(P, C, Width, Px, true, false, 0, 1.0, &H1);
                auto F = RenderPeaks(P, C, Width, Px, true, true, 0, 1.0, &H2);
                auto T = RenderPeaks(P, C, Width, Px, true, false, 1, 600.0);
                char Name[256]; snprintf(Name, sizeof Name, "%s%s_peaks%d_%.0fkm.ppm", Tag.c_str(), getenv("SUFFIX") ? getenv("SUFFIX") : "", v, Width / 1000);
                Sheet(Name, { A, B, F, T }, Px);
                double S0[3], S1[3], S2[3]; SlopeStats(H0, Px, Width / Px, S0); SlopeStats(H1, Px, Width / Px, S1); SlopeStats(H2, Px, Width / Px, S2);
                double Lo = 1e9, Hi = -1e9; for (size_t k = 0; k < H0.size(); k++) { Lo = std::min(Lo, H1[k] - H0[k]); Hi = std::max(Hi, H1[k] - H0[k]); }
                printf("wrote %s: slope p50/p90/p99 %.1f/%.1f/%.1f deg without peaks and gullies, %.1f/%.1f/%.1f with peaks, %.1f/%.1f/%.1f with both; peaks %.0f .. %.0f m\n",
                    Name, S0[0], S0[1], S0[2], S1[0], S1[1], S1[2], S2[0], S2[1], S2[2], Lo, Hi);
            }
        }
    }
    else if (Mode == "pgrad")
    {
        // The peaks' analytic gradient (VoxelLandPeaksDF at a fixed amplitude: the same landform,
        // relief, drainage and bake height at both ends) against central differences, at random
        // points and at points in the face blend bands (the weights' slope is part of it).
        if (P.Form.empty()) { printf("the dump has no landform\n"); return 1; }
        std::mt19937 Rng(9);
        std::normal_distribution<double> G;
        std::uniform_real_distribution<double> U01(0, 1);
        const float OwnFlow = (float)(2 * std::log2(P.Cell * 0.01));
        auto Check = [&](bool bEdges, const char* What) {
            double WorstRel = 0, SumRel = 0; int Count = 0;
            for (int i = 0; i < 4000; i++)
            {
                double D[3];
                if (bEdges)
                {
                    // Within the 3-cell band of a face edge (or a corner, now and then).
                    const int f = (int)(U01(Rng) * 6) % 6;
                    const float Sv = (float)(1.0 - U01(Rng) * 3.0 / P.N) * (U01(Rng) < 0.5 ? -1.0f : 1.0f);
                    const float Tv = i % 10 == 0 ? (float)(1.0 - U01(Rng) * 3.0 / P.N) : (float)(U01(Rng) * 2 - 1);
                    const float3 Dir = VoxelCubeDirection(f, Sv, Tv);
                    D[0] = Dir.x; D[1] = Dir.y; D[2] = Dir.z;
                }
                else { D[0] = G(Rng); D[1] = G(Rng); D[2] = G(Rng); }
                Normalize(D);
                double E1[3], E2[3]; Frame(D, E1, E2);
                const double Th = U01(Rng) * 2 * M_PI;
                double T[3]; for (int a = 0; a < 3; a++) T[a] = std::cos(Th) * E1[a] + std::sin(Th) * E2[a];
                const float2 Form(0.9f, (float)(i % 3) * 0.5f);
                // 20 cm, or 2 m in the bands: the blend weights follow the face coordinate,
                // whose float steps (1e-7 near the edge) make millimetre steps in the value there.
                const double Step = getenv("PGRAD_STEP") ? atof(getenv("PGRAD_STEP")) : (bEdges ? 200.0 : 20.0);   // cm
                auto At = [&](double S) {
                    double Q[3]; for (int a = 0; a < 3; a++) Q[a] = D[a] * P.SeaRadius + T[a] * S;
                    return P.S.VoxelLandPeaksDF(DF(Q[0]), DF(Q[1]), DF(Q[2]), Form, 2000.0f, OwnFlow, 1500.0f, 100.0f);
                };
                const auto C = At(0), A = At(Step), B = At(-Step);
                const double Num = ((double)A.Unit * A.AmplitudeM - (double)B.Unit * B.AmplitudeM) / (2 * Step * 0.01);
                const double Ana = C.Gradient.x * T[0] + C.Gradient.y * T[1] + C.Gradient.z * T[2];
                const double Rel = std::fabs(Num - Ana) / std::max(std::fabs(Ana) + std::fabs(Num), 0.02);
                if (getenv("PGRAD_DEBUG") && Rel > 0.05)
                {
                    float Fc[3]; for (int a = 0; a < 3; a++) Fc[a] = (float)std::fabs(D[a]);
                    printf("  rel %.3f num %.4f ana %.4f amp %.0f unit %.3f; |D| %.5f %.5f %.5f\n", Rel, Num, Ana, C.AmplitudeM, C.Unit, Fc[0], Fc[1], Fc[2]);
                }
                WorstRel = std::max(WorstRel, Rel); SumRel += Rel; Count++;
            }
            printf("peaks gradient against differences (%s): mean relative error %.2g, worst %.2g (of %d)\n", What, SumRel / Count, WorstRel, Count);
        };
        Check(false, "random points");
        Check(true, "face blend bands");
        // The shape's range (VoxelLand::PeakReach assumes -1 .. 1).
        double Lo = 1e9, Hi = -1e9, Mean = 0; const int Range = 300000;
        for (int c = 0; c <= 2; c++)
            for (int i = 0; i < Range; i++)
            {
                double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
                const auto Pk = P.S.VoxelLandPeaksDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), float2(0.9f, 0.5f * c), 2000.0f, OwnFlow, 1500.0f, 100.0f);
                Lo = std::min(Lo, (double)Pk.Unit); Hi = std::max(Hi, (double)Pk.Unit); Mean += Pk.Unit;
            }
        printf("peaks shape over %d points (character 0, 0.5, 1): %.3f .. %.3f, mean %.4f\n", 3 * Range, Lo, Hi, Mean / (3 * Range));
        // Continuity: lines of 10 m steps across face edges (20 km each), the largest step against the typical one.
        double WorstRatio = 0, WorstJump = 0;
        for (int l = 0; l < 60; l++)
        {
            const int f = l % 6;
            const float Tv = (float)(U01(Rng) * 1.6 - 0.8);
            const float3 A0 = VoxelCubeDirection(f, 0.97f, Tv);
            double D0[3] = { A0.x, A0.y, A0.z }; Normalize(D0);
            const float3 A1 = VoxelCubeDirection(f, 1.0f, Tv);
            double Dir[3] = { A1.x - A0.x, A1.y - A0.y, A1.z - A0.z }; Normalize(Dir);
            double Prev = 0, SumStep = 0, MaxStep = 0; const int Steps = 2000;
            for (int k = 0; k <= Steps; k++)
            {
                double Q[3]; for (int a = 0; a < 3; a++) Q[a] = D0[a] * P.SeaRadius + Dir[a] * (P.SeaRadius * 0.06 * k / Steps);
                double Lq = std::sqrt(Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2]); for (int a = 0; a < 3; a++) Q[a] *= P.SeaRadius / Lq;
                const auto Pk = P.S.VoxelLandPeaksDF(DF(Q[0]), DF(Q[1]), DF(Q[2]), float2(0.9f, 1.0f), 2000.0f, OwnFlow, 1500.0f, 100.0f);
                const double V = (double)Pk.Unit * Pk.AmplitudeM;
                if (k > 0) { const double St = std::fabs(V - Prev); SumStep += St; MaxStep = std::max(MaxStep, St); }
                Prev = V;
            }
            WorstJump = std::max(WorstJump, MaxStep);
            WorstRatio = std::max(WorstRatio, MaxStep / std::max(SumStep / Steps, 1e-9));
        }
        printf("peaks along 60 lines across face edges in steps of %.0f m: largest step %.2f m, at most %.1f times the line's mean step\n",
            P.SeaRadius * 0.06 / 2000 * 0.01, WorstJump, WorstRatio);
        // Over the footprint: the shape, gradient and steering change smoothly (mips blend, octaves fade).
        double WorstUnit = 0, WorstGrad = 0, WorstSteer = 0;
        for (int l = 0; l < 120; l++)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            const float2 F2(0.9f, (float)(l % 3) * 0.5f);
            double PrevU = 0, PrevG[3] = {}, PrevS[3] = {};
            for (int k = 0; k <= 12000; k++)
            {
                const float Footprint = (float)(100.0 * std::pow(1.001, (double)k));
                const auto Pk = P.S.VoxelLandPeaksDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), F2, 2000.0f, OwnFlow, 1500.0f, Footprint);
                const double Gr[3] = { Pk.Gradient.x, Pk.Gradient.y, Pk.Gradient.z }, St[3] = { Pk.Steer.x, Pk.Steer.y, Pk.Steer.z };
                if (k > 0)
                {
                    WorstUnit = std::max(WorstUnit, std::fabs(Pk.Unit - PrevU));
                    double DG = 0, DS = 0;
                    for (int a = 0; a < 3; a++) { DG += (Gr[a] - PrevG[a]) * (Gr[a] - PrevG[a]); DS += (St[a] - PrevS[a]) * (St[a] - PrevS[a]); }
                    WorstGrad = std::max(WorstGrad, std::sqrt(DG));
                    WorstSteer = std::max(WorstSteer, std::sqrt(DS));
                }
                PrevU = Pk.Unit;
                for (int a = 0; a < 3; a++) { PrevG[a] = Gr[a]; PrevS[a] = St[a]; }
            }
        }
        printf("peaks over the footprint (120 points, 1 m .. 1600 km in 0.1%% steps): largest change of the shape %.4f, gradient %.5f m/m, steering %.5f m/m\n",
            WorstUnit, WorstGrad, WorstSteer);
    }
    else if (Mode == "noise")
    {
        // The noise sampler (VoxelLandNoise.ush, the global shaders' accessors) against the
        // handoff's B-spline (Addendum 1, section 2) in doubles, straight from the set's texels:
        // one mip (values, gradients, R's second derivatives), then the full lookup (orientation,
        // offset, the mip map between levels, the blend). Run with NOISE_SET for an authored set.
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        // The peaks' set (from the buffer's start), then the detail set appended after it.
        for (int Which = 0; Which < 2; Which++)
        {
            const VoxelNoiseTiles::FSet& Set = Which == 0 ? P.NoiseSet : P.Detail;
            const uint32_t SetBase = Which == 0 ? 0u : P.DetailBase;
            const uint32_t NT = Set.N;
            const FVoxelLandNoiseSetRef SetRef = VoxelLandNoiseSetOf(SetBase, NT, Set.Mips);
            printf("set %s (%s): N %u, %u mips; channels", Set.Name.c_str(), Set.LookTag.c_str(), NT, Set.Mips);
            for (int c = 0; c < 4; c++)
                printf(" %c kind %u cells %u range %.6g..%.6g", "RGBA"[c], Set.Channel[c].Kind, Set.Channel[c].CellsPerTile, Set.Channel[c].Min, Set.Channel[c].Max);
            printf("\n");
            auto Tex = [&](uint32_t m, int64_t x, int64_t y, int c) {
                const int64_t S = (int64_t)(NT >> m);
                x = ((x % S) + S) % S; y = ((y % S) + S) % S;
                return (double)Set.Texels[(size_t)(VoxelNoiseTiles::MipOffset(NT, m) + (uint64_t)(y * S + x)) * 4 + c] / 65535.0;
            };
            auto Weights = [](double s, double* w, double* d, double* d2) {
                w[0] = (1 - s) * (1 - s) * (1 - s) / 6; w[1] = (3 * s * s * s - 6 * s * s + 4) / 6;
                w[2] = (-3 * s * s * s + 3 * s * s + 3 * s + 1) / 6; w[3] = s * s * s / 6;
                d[0] = -(1 - s) * (1 - s) / 2; d[1] = (3 * s * s - 4 * s) / 2; d[2] = (-3 * s * s + 2 * s + 1) / 2; d[3] = s * s / 2;
                d2[0] = 1 - s; d2[1] = 3 * s - 2; d2[2] = -3 * s + 1; d2[3] = s;
            };
            // Value, d/du, d/dv per channel, and R's uu, uv, vv; per texel of mip m.
            struct FRef { double V[4], U[4], W[4], RUU, RUV, RVV, BUU, BUV, BVV; };
            auto Ref = [&](uint32_t m, double u, double v) {
                FRef O{};
                const double x0 = std::floor(u), y0 = std::floor(v);
                double wx[4], dx[4], cx[4], wy[4], dy[4], cy[4];
                Weights(u - x0, wx, dx, cx); Weights(v - y0, wy, dy, cy);
                for (int j = 0; j < 4; j++)
                    for (int i = 0; i < 4; i++)
                        for (int c = 0; c < 4; c++)
                        {
                            const double T = Tex(m, (int64_t)x0 - 1 + i, (int64_t)y0 - 1 + j, c);
                            O.V[c] += wx[i] * wy[j] * T; O.U[c] += dx[i] * wy[j] * T; O.W[c] += wx[i] * dy[j] * T;
                            if (c == 0) { O.RUU += cx[i] * wy[j] * T; O.RUV += dx[i] * dy[j] * T; O.RVV += wx[i] * cy[j] * T; }
                            if (c == 2) { O.BUU += cx[i] * wy[j] * T; O.BUV += dx[i] * dy[j] * T; O.BVV += wx[i] * cy[j] * T; }
                        }
                return O;
            };
            auto Get = [](const FVoxelLandNoise& Nz, double V[4], double U[4], double W[4]) {
                V[0] = Nz.R; V[1] = Nz.G; V[2] = Nz.B; V[3] = Nz.A;
                U[0] = Nz.RU; U[1] = Nz.GU; U[2] = Nz.BU; U[3] = Nz.AU;
                W[0] = Nz.RV; W[1] = Nz.GV; W[2] = Nz.BV; W[3] = Nz.AV;
            };
            std::mt19937 Rng(11);
            std::uniform_real_distribution<double> U01(0, 1);
            // Gradient scale per channel: the mip-0 gradient RMS in unit values per texel.
            double GradRms[4] = {};
            for (int i = 0; i < 20000; i++)
            {
                const FRef R0 = Ref(0, U01(Rng) * NT, U01(Rng) * NT);
                for (int c = 0; c < 4; c++) GradRms[c] += R0.U[c] * R0.U[c] + R0.W[c] * R0.W[c];
            }
            for (int c = 0; c < 4; c++) GradRms[c] = std::sqrt(GradRms[c] / 20000);
            double WorstV = 0, WorstG[4] = {}, WorstH = 0;
            int OnlyAOff = 0;
            const int Count = 100000;
            for (int i = 0; i < Count; i++)
            {
                const uint32_t m = (uint32_t)(U01(Rng) * Set.Mips) % Set.Mips;
                const uint32_t S = NT >> m;
                const int X = (int)(U01(Rng) * S), Y = (int)(U01(Rng) * S);
                const float FX = (float)U01(Rng), FY = (float)U01(Rng);
                const FVoxelLandNoise Nz = VoxelLandNoiseMip(SetRef, X, FX, Y, FY, m, VOXEL_LAND_NOISE_WANT_B | VOXEL_LAND_NOISE_WANT_A | VOXEL_LAND_NOISE_WANT_HESSIAN_R | VOXEL_LAND_NOISE_WANT_HESSIAN_B);
                const FRef R = Ref(m, X + (double)FX, Y + (double)FY);
                // N1: A alone (one word a texel) gives the same A, and nothing else.
                const FVoxelLandNoise NA = VoxelLandNoiseMip(SetRef, X, FX, Y, FY, m, VOXEL_LAND_NOISE_WANT_ONLY_A);
                if (NA.A != Nz.A || NA.AU != Nz.AU || NA.AV != Nz.AV || NA.R != 0.0f || NA.G != 0.0f || NA.B != 0.0f) OnlyAOff++;
                double V[4], U[4], W[4]; Get(Nz, V, U, W);
                for (int c = 0; c < 4; c++)
                {
                    WorstV = std::max(WorstV, std::fabs(V[c] - R.V[c]));
                    WorstG[c] = std::max(WorstG[c], std::max(std::fabs(U[c] - R.U[c]), std::fabs(W[c] - R.W[c])) / std::max(GradRms[c], 1e-12));
                }
                const double HS = std::max({ std::fabs(R.RUU), std::fabs(R.RUV), std::fabs(R.RVV), 1e-3 });
                WorstH = std::max(WorstH, std::max({ std::fabs(Nz.RUU - R.RUU), std::fabs(Nz.RUV - R.RUV), std::fabs(Nz.RVV - R.RVV) }) / HS);
                const double HB = std::max({ std::fabs(R.BUU), std::fabs(R.BUV), std::fabs(R.BVV), 1e-3 });
                WorstH = std::max(WorstH, std::max({ std::fabs(Nz.BUU - R.BUU), std::fabs(Nz.BUV - R.BUV), std::fabs(Nz.BVV - R.BVV) }) / HB);
            }
            printf("one mip, %d points: value worst %.3g (unit); gradient worst R %.3g G %.3g B %.3g A %.3g (of each channel's RMS); R's and B's curvature worst %.3g (relative); A alone differs at %d\n",
                Count, WorstV, WorstG[0], WorstG[1], WorstG[2], WorstG[3], WorstH, OnlyAOff);
            // The full lookup.
            double WorstFV = 0, WorstFG = 0;
            int Fails = 0;
            for (int i = 0; i < Count; i++)
            {
                FVoxelLandNoiseAxis AU, AV;
                AU.I = (int)(U01(Rng) * 8 * NT) - 4 * (int)NT; AV.I = (int)(U01(Rng) * 8 * NT) - 4 * (int)NT;
                AU.F = i % 7 == 0 ? 0.0f : (float)U01(Rng); AV.F = i % 11 == 0 ? 0.0f : (float)U01(Rng);
                const uint32_t Orient = (uint32_t)(U01(Rng) * 8) & 7u;
                const uint32_t OffU = (uint32_t)(U01(Rng) * 4294967295.0), OffV = (uint32_t)(U01(Rng) * 4294967295.0);
                const float Level = (float)(U01(Rng) * (Set.Mips + 1) - 1.0);
                const FVoxelLandNoise Nz = VoxelLandNoiseAt(SetRef, AU, AV, Orient, OffU, OffV, Level, VOXEL_LAND_NOISE_WANT_B | VOXEL_LAND_NOISE_WANT_A);
                // The reference: the node in the set's axes, wrapped, mapped to each mip.
                double u = AU.I + (double)AU.F, v = AV.I + (double)AV.F;
                if (Orient & 1u) std::swap(u, v);
                if (Orient & 2u) u = -u;
                if (Orient & 4u) v = -v;
                u = std::fmod(u + (double)(OffU & (NT - 1)), (double)NT); if (u < 0) u += NT;
                v = std::fmod(v + (double)(OffV & (NT - 1)), (double)NT); if (v < 0) v += NT;
                const double L = std::min(std::max((double)Level, 0.0), (double)(Set.Mips - 1));
                const uint32_t M0 = (uint32_t)std::floor(L);
                // The blend narrowed to the step's middle (VOXEL_LAND_NOISE_BLEND_WIDTH).
                const double T = std::clamp((L - M0 - 0.5) / (double)VOXEL_LAND_NOISE_BLEND_WIDTH + 0.5, 0.0, 1.0);
                double V[4] = {}, U[4] = {}, W[4] = {};
                for (int k = 0; k < 2; k++)
                {
                    const uint32_t m = M0 + k;
                    const double Wt = k == 0 ? 1 - T : T;
                    if (Wt == 0 || m >= Set.Mips) continue;
                    const double Sc = std::ldexp(1.0, -(int)m);
                    const FRef R = Ref(m, (u + 0.5) * Sc - 0.5, (v + 0.5) * Sc - 0.5);
                    for (int c = 0; c < 4; c++) { V[c] += Wt * R.V[c]; U[c] += Wt * R.U[c] * Sc; W[c] += Wt * R.W[c] * Sc; }
                }
                // Back to the caller's axes.
                for (int c = 0; c < 4; c++)
                {
                    if (Orient & 2u) U[c] = -U[c];
                    if (Orient & 4u) W[c] = -W[c];
                    if (Orient & 1u) std::swap(U[c], W[c]);
                }
                double GV[4], GU[4], GW[4]; Get(Nz, GV, GU, GW);
                for (int c = 0; c < 4; c++)
                {
                    const double EV = std::fabs(GV[c] - V[c]);
                    const double EG = std::max(std::fabs(GU[c] - U[c]), std::fabs(GW[c] - W[c])) / std::max(GradRms[c], 1e-12);
                    WorstFV = std::max(WorstFV, EV); WorstFG = std::max(WorstFG, EG);
                    if (EV > 1e-4 || EG > 1e-3) Fails++;
                }
            }
            printf("full lookup (8 orientations, offsets, levels -1 .. %u), %d points: value worst %.3g (unit), gradient worst %.3g (of the RMS); %d channel samples off\n",
                Set.Mips, Count, WorstFV, WorstFG, Fails);
        }
        // The statistics the plugin normalises with.
        printf("detail: base %u, MeanHard %.6f, MeanSoft %.6f, ScaleHard %.5f, ScaleSoft %.5f, texel %.2f m, fade level %.0f\n", P.S.LandDetailBase,
            P.S.LandDetailMeanHard, P.S.LandDetailMeanSoft, P.S.LandPeakDetailScaleHard, P.S.LandPeakDetailScaleSoft,
            P.S.LandPeakDetailTexelsPerCell > 0 ? P.S.LandCellM / P.S.LandPeakDetailTexelsPerCell : 0.0f, P.S.LandPeakDetailFade);
        printf("peaks normalisation: MeanR %.6f, MeanG %.6f, ScaleG %.4f, ScaleB %.4f, %u massifs a side (spacing %u texels)\n", P.S.LandPeakMeanR,
            P.S.LandPeakMeanG, P.S.LandPeakScaleG, P.S.LandPeakScaleB, P.NoiseSet.Channel[0].CellsPerTile,
            P.NoiseSet.N / std::max(P.NoiseSet.Channel[0].CellsPerTile, 1u));
    }
    else if (Mode == "hgrad")
    {
        // The hills' analytic gradient (VoxelLandHillsDF, at a fixed amplitude: the same landform,
        // drainage and bake height at both ends) against central differences of the hills, in
        // random tangent directions at random points, at a fine footprint.
        if (P.Form.empty()) { printf("the dump has no landform\n"); return 1; }
        // HGRAD_PROVINCE: the province the samples take (T4c: 1 tests the strata's set, 0 the hills').
        const float HProv = getenv("HGRAD_PROVINCE") ? (float)atof(getenv("HGRAD_PROVINCE")) : 0.0f;
        std::mt19937 Rng(5);
        std::normal_distribution<double> G;
        double WorstRel = 0, SumRel = 0; int Count = 0;
        for (int i = 0; i < 4000; i++)
        {
            // Hill country, and rolling ground (where a hill set's swells join, T4a).
            const float2 Form(i % 2 ? 1.0f : 0.3f, 0.7f);
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            double E1[3], E2[3]; Frame(D, E1, E2);
            const double Th = std::uniform_real_distribution<double>(0, 2 * M_PI)(Rng);
            double T[3]; for (int a = 0; a < 3; a++) T[a] = std::cos(Th) * E1[a] + std::sin(Th) * E2[a];
            const double Step = 20.0;   // cm
            auto At = [&](double S) {
                double Q[3]; for (int a = 0; a < 3; a++) Q[a] = D[a] * P.SeaRadius + T[a] * S;
                return P.S.VoxelLandHillsShapeDF(DF(Q[0]), DF(Q[1]), DF(Q[2]), Form, float3(0.0f, 0.0f, 0.0f), 0.0f, HProv, 0.0f, 500.0f, 100.0f, true);
            };
            const auto C = At(0), A = At(Step), B = At(-Step);
            const double Num = ((double)A.Unit * A.AmplitudeM - (double)B.Unit * B.AmplitudeM) / (2 * Step * 0.01);
            const double Ana = C.Gradient.x * T[0] + C.Gradient.y * T[1] + C.Gradient.z * T[2];
            const double Rel = std::fabs(Num - Ana) / std::max(std::fabs(Ana) + std::fabs(Num), 0.02);
            WorstRel = std::max(WorstRel, Rel); SumRel += Rel; Count++;
        }
        printf("hills gradient against differences over 20 cm: mean relative error %.2g, worst %.2g (of %d)\n", SumRel / Count, WorstRel, Count);
        // The shape's range (VoxelLand::HillMaxUnit bounds it for the planet's height range).
        double Lo = 1e9, Hi = -1e9, Mean = 0; const int Range = 400000;
        for (int c = 0; c <= 1; c++)
            for (int i = 0; i < Range; i++)
            {
                double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
                const auto H = P.S.VoxelLandHillsShapeDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), float2(1.0f, (float)c), float3(0.0f, 0.0f, 0.0f), 0.0f, HProv, 0.0f, 500.0f, 100.0f, true);
                Lo = std::min(Lo, (double)H.Unit); Hi = std::max(Hi, (double)H.Unit); Mean += H.Unit;
            }
        printf("hills shape over %d points (character 0 and 1): %.3f .. %.3f, mean %.3f\n", 2 * Range, Lo, Hi, Mean / (2 * Range));
        // Continuity: lines of 1 cm steps, the largest step against the typical one.
        double WorstJump = 0, WorstRatio = 0;
        for (int l = 0; l < 200; l++)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            double E1[3], E2[3]; Frame(D, E1, E2);
            double Prev = 0, SumStep = 0, MaxStep = 0;
            for (int k = 0; k <= 5000; k++)
            {
                double Q[3]; for (int a = 0; a < 3; a++) Q[a] = D[a] * P.SeaRadius + E1[a] * (double)k;
                const auto H = P.S.VoxelLandHillsShapeDF(DF(Q[0]), DF(Q[1]), DF(Q[2]), float2(1.0f, 1.0f), float3(0.0f, 0.0f, 0.0f), 0.0f, HProv, 0.0f, 500.0f, 100.0f, true);
                const double V = (double)H.Unit * H.AmplitudeM;
                if (k > 0) { const double St = std::fabs(V - Prev); SumStep += St; MaxStep = std::max(MaxStep, St); }
                Prev = V;
            }
            WorstJump = std::max(WorstJump, MaxStep);
            WorstRatio = std::max(WorstRatio, MaxStep / std::max(SumStep / 5000.0, 1e-9));
        }
        printf("hills along 200 lines of 50 m in 1 cm steps: largest step %.4f m, at most %.1f times the line's mean step\n", WorstJump, WorstRatio);
        // Continuity over the footprint: each octave fades out with its band, so the shape, its
        // gradient and the gullies' steering (the two coarsest octaves) change smoothly as the
        // footprint grows; the largest change over a 0.1% step, against the outputs' size.
        double WorstUnit = 0, WorstGrad = 0, WorstSteer = 0;
        for (int l = 0; l < 200; l++)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            const float2 F2(1.0f, (float)(l % 3) * 0.5f);
            double PrevU = 0, PrevG[3] = {}, PrevS[3] = {};
            for (int k = 0; k <= 12000; k++)
            {
                const float Footprint = (float)(100.0 * std::pow(1.001, (double)k));   // 1 m .. about 1600 km
                const auto H = P.S.VoxelLandHillsShapeDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), F2, float3(0.0f, 0.0f, 0.0f), 0.0f, HProv, 0.0f, 500.0f, Footprint, true);
                const double Gr[3] = { H.Gradient.x, H.Gradient.y, H.Gradient.z }, St[3] = { H.Steer.x, H.Steer.y, H.Steer.z };
                if (k > 0)
                {
                    WorstUnit = std::max(WorstUnit, std::fabs(H.Unit - PrevU));
                    double DG = 0, DS = 0;
                    for (int a = 0; a < 3; a++) { DG += (Gr[a] - PrevG[a]) * (Gr[a] - PrevG[a]); DS += (St[a] - PrevS[a]) * (St[a] - PrevS[a]); }
                    WorstGrad = std::max(WorstGrad, std::sqrt(DG));
                    WorstSteer = std::max(WorstSteer, std::sqrt(DS));
                }
                PrevU = H.Unit;
                for (int a = 0; a < 3; a++) { PrevG[a] = Gr[a]; PrevS[a] = St[a]; }
            }
        }
        printf("hills over the footprint (200 points, 1 m .. 1600 km in 0.1%% steps): largest change of the shape %.4f, gradient %.5f m/m, steering %.5f m/m\n",
            WorstUnit, WorstGrad, WorstSteer);
    }
    else if (Mode == "slope")
    {
        // Gradients by central differences at 0.5 m, every octave (footprint the finest cell),
        // at random land points with slope.
        std::mt19937 Rng(3);
        std::normal_distribution<double> G;
        const int Count = argc > 5 ? atoi(argv[5]) : 20000;
        std::vector<double> Gb(Count, -1), Gg(Count, -1), Gt(Count, -1), Gn(Count, -1);
        std::vector<std::array<double, 3>> Dirs(Count);
        for (int i = 0; i < Count; i++) { double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D); Dirs[i] = { D[0], D[1], D[2] }; }
        ParallelFor(Count, [&](int i) {
            const double* D = Dirs[i].data();
            const FTerms T0 = Terms(P, D, 0.0);
            if (T0.Bake < 50) return;
            double E1[3], E2[3]; Frame(D, E1, E2);
            const double Step = 50.0 / P.SeaRadius;   // 0.5 m
            double Gx[4] = {}, Gy[4] = {};
            for (int s = -1; s <= 1; s += 2)
            {
                double A[3], B[3];
                for (int a = 0; a < 3; a++) { A[a] = D[a] + s * Step * E1[a]; B[a] = D[a] + s * Step * E2[a]; }
                Normalize(A); Normalize(B);
                const FTerms TA = Terms(P, A, 0.0), TB = Terms(P, B, 0.0);
                const FTerms NA = Terms(P, A, 0.0, false), NB = Terms(P, B, 0.0, false);
                Gx[0] += s * TA.Bake; Gy[0] += s * TB.Bake;
                Gx[1] += s * TA.Gully; Gy[1] += s * TB.Gully;
                Gx[2] += s * TA.Height; Gy[2] += s * TB.Height;
                Gx[3] += s * NA.Height; Gy[3] += s * NB.Height;
            }
            const double Inv = 1.0 / (2 * 0.5);
            Gb[i] = std::hypot(Gx[0], Gy[0]) * Inv; Gg[i] = std::hypot(Gx[1], Gy[1]) * Inv;
            Gt[i] = std::hypot(Gx[2], Gy[2]) * Inv; Gn[i] = std::hypot(Gx[3], Gy[3]) * Inv;
        });
        auto Report = [&](const char* What, std::vector<double> V) {
            V.erase(std::remove_if(V.begin(), V.end(), [](double x) { return x < 0; }), V.end());
            std::sort(V.begin(), V.end());
            auto Q = [&](double q) { return V[std::min((size_t)(q * V.size()), V.size() - 1)]; };
            printf("  %-28s n %zu: median %.3f, p90 %.3f, p99 %.3f, p99.9 %.3f, max %.3f\n", What, V.size(), Q(0.5), Q(0.9), Q(0.99), Q(0.999), V.back());
        };
        printf("slopes (m/m) on land, at 0.5 m:\n");
        Report("bake (warped)", Gb);
        Report("gullies", Gg);
        Report("total without gullies", Gn);
        Report("total with gullies", Gt);

        // The gully term's worst case: forced slope (0.8 and 2) in random directions, flow at
        // full strength, every octave; the gradient of the term alone by central differences.
        for (double Forced : { 0.3, 0.8, 2.0 })
        {
            std::vector<double> Gf(Count);
            ParallelFor(Count, [&](int i) {
                const double* D = Dirs[i].data();
                double E1[3], E2[3]; Frame(D, E1, E2);
                const double Ang = (i * 0.618034) * 2 * M_PI;
                const float3 Grad((float)(Forced * (std::cos(Ang) * E1[0] + std::sin(Ang) * E2[0])), (float)(Forced * (std::cos(Ang) * E1[1] + std::sin(Ang) * E2[1])),
                    (float)(Forced * (std::cos(Ang) * E1[2] + std::sin(Ang) * E2[2])));
                const double Step = 50.0 / P.SeaRadius;
                double Gx = 0, Gy = 0;
                for (int s = -1; s <= 1; s += 2)
                {
                    double A[3], B[3];
                    for (int a = 0; a < 3; a++) { A[a] = (D[a] + s * Step * E1[a]) * P.R; B[a] = (D[a] + s * Step * E2[a]) * P.R; }
                    Normalize(A); Normalize(B);
                    for (int a = 0; a < 3; a++) { A[a] *= P.R; B[a] *= P.R; }
                    Gx += s * P.S.VoxelLandGullyDF(DF(A[0]), DF(A[1]), DF(A[2]), Grad, 0.0f, 60.0f, 1000.0f, (float)P.Finest).Height;
                    Gy += s * P.S.VoxelLandGullyDF(DF(B[0]), DF(B[1]), DF(B[2]), Grad, 0.0f, 60.0f, 1000.0f, (float)P.Finest).Height;
                }
                Gf[i] = std::hypot(Gx, Gy);
            });
            char What[64]; snprintf(What, sizeof What, "gullies, forced slope %.1f", Forced);
            Report(What, Gf);
        }
    }
    else if (Mode == "edges")
    {
        // Walk across face edges and corners at 1 m steps; the largest step-to-step change of
        // the gully term against the typical one (a seam shows as a jump).
        std::mt19937 Rng(5);
        double Worst = 0, Typical = 0; int Steps = 0;
        const int Lines = 64;
        std::vector<double> LineMax(Lines), LineMean(Lines);
        ParallelFor(Lines, [&](int l) {
            std::mt19937 R2(l * 7919 + 1);
            std::uniform_real_distribution<double> U(-1, 1);
            // A point on an edge (|x| = |y| > |z|) or near a corner for the last eight.
            double E[3];
            if (l < Lines - 8) { const double z = U(R2) * 0.7; E[0] = 1; E[1] = 1; E[2] = z; }
            else { E[0] = 1; E[1] = 1; E[2] = 1 - 0.002 * (l - Lines + 9); }
            Normalize(E);
            double T1[3] = { 1, -1, 0 }; Normalize(T1);   // across the edge
            double T2[3]; T2[0] = E[1] * T1[2] - E[2] * T1[1]; T2[1] = E[2] * T1[0] - E[0] * T1[2]; T2[2] = E[0] * T1[1] - E[1] * T1[0];
            const double Ang = U(R2) * 0.6;
            double Dir[3]; for (int a = 0; a < 3; a++) Dir[a] = std::cos(Ang) * T1[a] + std::sin(Ang) * T2[a];
            const double Span = 4.0 * P.Cell * 0.01;   // m, either side
            const int K = (int)(2 * Span);
            // Forced slope 0.6 along Dir, so the gullies are everywhere.
            double Prev = 0, Max = 0, Sum = 0;
            for (int k = 0; k <= K; k++)
            {
                const double t = (k - K / 2) * 100.0 / P.SeaRadius;
                double D[3]; for (int a = 0; a < 3; a++) D[a] = E[a] + t * Dir[a];
                Normalize(D);
                const float3 Grad((float)(0.6 * Dir[0]), (float)(0.6 * Dir[1]), (float)(0.6 * Dir[2]));
                const double H = P.S.VoxelLandGullyDF(DF(D[0] * P.R), DF(D[1] * P.R), DF(D[2] * P.R), Grad, 0.0f, 60.0f, 1000.0f, (float)P.Finest).Height;
                if (k > 0) { Max = std::max(Max, std::fabs(H - Prev)); Sum += std::fabs(H - Prev); }
                Prev = H;
            }
            LineMax[l] = Max; LineMean[l] = Sum / K;
        });
        for (int l = 0; l < Lines; l++) { Worst = std::max(Worst, LineMax[l]); Typical += LineMean[l]; }
        printf("edges: %d lines across face edges (8 near the corner), 1 m steps over +-4 cells: largest step %.3f m, mean step %.3f m\n", Lines, Worst, Typical / Lines);
        // The same away from edges, for comparison.
        double Worst2 = 0;
        const int Span2 = (int)(8.0 * P.Cell * 0.01);
        for (int l = 0; l < 64; l++)
        {
            double E[3] = { 1, 0.08 * (l % 8) - 0.3, 0.08 * (l / 8) - 0.3 }; Normalize(E);
            double Prev = 0;
            for (int k = 0; k <= Span2; k++)
            {
                double D[3] = { E[0], E[1] + k * 100.0 / P.SeaRadius, E[2] }; Normalize(D);
                const float3 Grad(0.0f, 0.6f, 0.0f);
                const double H = P.S.VoxelLandGullyDF(DF(D[0] * P.R), DF(D[1] * P.R), DF(D[2] * P.R), Grad, 0.0f, 60.0f, 1000.0f, (float)P.Finest).Height;
                if (k > 0) Worst2 = std::max(Worst2, std::fabs(H - Prev));
                Prev = H;
            }
        }
        printf("  64 interior lines of the same length for comparison: largest step %.3f m\n", Worst2);
    }
    else if (Mode == "lod")
    {
        // Per level (footprint doubling), the rms and max change of the height between it and
        // the next finer level, at land points with slope: what a level switch moves.
        std::mt19937 Rng(11);
        std::normal_distribution<double> G;
        const int Count = 20000;
        std::vector<std::array<double, 3>> Dirs;
        while ((int)Dirs.size() < Count)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            if (Terms(P, D, 1e9, false).Bake > 300) Dirs.push_back({ D[0], D[1], D[2] });
        }
        printf("level changes (m) between footprint F and F/2 at %d land points:\n", Count);
        for (double F = 1600.0 * 100; F >= 4.0 * 100; F *= 0.5)
        {
            std::vector<double> DG(Count), DT(Count), DN(Count);
            ParallelFor(Count, [&](int i) {
                const double* D = Dirs[i].data();
                const FTerms A = Terms(P, D, F), B = Terms(P, D, F * 0.5);
                const FTerms An = Terms(P, D, F, false), Bn = Terms(P, D, F * 0.5, false);
                DG[i] = A.Gully - B.Gully; DT[i] = A.Height - B.Height; DN[i] = An.Height - Bn.Height;
            });
            auto Rms = [](const std::vector<double>& V) { double S = 0; for (double v : V) S += v * v; return std::sqrt(S / V.size()); };
            auto Max = [](const std::vector<double>& V) { double S = 0; for (double v : V) S = std::max(S, std::fabs(v)); return S; };
            auto Mean = [](const std::vector<double>& V) { double S = 0; for (double v : V) S += v; return S / V.size(); };
            printf("  F %6.0f m: gullies rms %.2f max %.2f mean %+.3f; total rms %.2f max %.2f; without gullies rms %.2f max %.2f\n",
                F * 0.01, Rms(DG), Max(DG), Mean(DG), Rms(DT), Max(DT), Rms(DN), Max(DN));
        }
    }
    else if (Mode == "cost")
    {
        std::mt19937 Rng(13);
        std::normal_distribution<double> G;
        std::vector<std::array<double, 3>> Dirs;
        while (Dirs.size() < 50000)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            if (Terms(P, D, 1e9, false).Bake > 300) Dirs.push_back({ D[0], D[1], D[2] });
        }
        for (double F : { P.Finest, 100.0 * 100, 1000.0 * 100 })
        {
            double Sink = 0;
            double T0 = Now();
            for (auto& D : Dirs) Sink += Terms(P, D.data(), F, false).Height;
            const double A = (Now() - T0) / Dirs.size();
            T0 = Now();
            for (auto& D : Dirs) Sink += Terms(P, D.data(), F, true).Height;
            const double B = (Now() - T0) / Dirs.size();
            printf("cost per land sample at footprint %.0f cm, one thread: %.2f us without gullies, %.2f us with (x%.2f)  [%g]\n", F, A * 1e6, B * 1e6, B / A, Sink * 1e-30);
        }
    }
    else if (Mode == "pcost")
    {
        // The peaks' cost on the CPU path: land samples where the peaks are on (amplitude above
        // zero at the finest footprint), with and without them, at three footprints; and (built
        // with -DVOXEL_HARNESS_COUNT_NOISE) the noise set's word loads per sample.
        std::mt19937 Rng(17);
        std::normal_distribution<double> G;
        std::vector<std::array<double, 3>> Dirs;
        int Tried = 0;
        const size_t Want = getenv("PCOST_N") ? (size_t)atoi(getenv("PCOST_N")) : 20000;
        // PCOST_PATCH (m): points within a patch that wide around the first peak point, as a mesh
        // chunk samples them (memory access as coherent as in use), not all over the planet.
        const double Patch = getenv("PCOST_PATCH") ? atof(getenv("PCOST_PATCH")) : 0.0;
        double Centre[3] = {}, E1[3] = {}, E2[3] = {};
        while (Dirs.size() < Want && Tried < 20000000)
        {
            Tried++;
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            if (Patch > 0.0 && !Dirs.empty())
            {
                const double U = (std::uniform_real_distribution<double>(-0.5, 0.5)(Rng)) * Patch * 100.0 / P.SeaRadius;
                const double V = (std::uniform_real_distribution<double>(-0.5, 0.5)(Rng)) * Patch * 100.0 / P.SeaRadius;
                for (int a = 0; a < 3; a++) D[a] = Centre[a] + U * E1[a] + V * E2[a];
                Normalize(D);
            }
            if (Terms(P, D, P.Finest).Peaks != 0.0)
            {
                if (Dirs.empty()) { for (int a = 0; a < 3; a++) Centre[a] = D[a]; Frame(Centre, E1, E2); }
                Dirs.push_back({ D[0], D[1], D[2] });
            }
        }
        printf("%zu points with peaks (of %d tried)\n", Dirs.size(), Tried);
        if (getenv("PCOST_PROFILE"))
        {
            // One pass at the finest footprint, peaks on (PCOST_PROFILE=1) or off (0), for an
            // instruction count: valgrind --tool=callgrind --toggle-collect='*PcostPass*'.
            const bool bOn = atoi(getenv("PCOST_PROFILE")) != 0;
            printf("profile pass %s: %g\n", bOn ? "with peaks" : "without", PcostPass(P, Dirs, bOn));
            return 0;
        }
        for (double F : { P.Finest, 100.0 * 100, 1000.0 * 100 })
        {
            double Sink = 0;
            double Best[2] = { 1e9, 1e9 };
            for (int Rep = 0; Rep < 3; Rep++)
                for (int k = 0; k < 2; k++)
                {
                    const double T0 = Now();
                    for (auto& D : Dirs) Sink += Terms(P, D.data(), F, true, true, true, k == 1).Height;
                    Best[k] = std::min(Best[k], (Now() - T0) / Dirs.size());
                }
            double Loads = 0, LoadsNormal = 0;
#ifdef VOXEL_HARNESS_COUNT_NOISE
            GNoiseLoads = 0;
            for (auto& D : Dirs) Sink += Terms(P, D.data(), F).Height;
            Loads = (double)GNoiseLoads / Dirs.size();
            // The material's close-up normal and the cache writer: the point and two differenced
            // evaluations a 25 cm step away (the GPU path's code).
            std::vector<FVoxelOctave> Gpu;
            SetGpuGlobals(P, Gpu);
            GNoiseLoads = 0;
            for (auto& D : Dirs)
            {
                const FVoxelLandHeight H0 = VoxelLandHeightDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)F, true);
                const FVoxelLandDifferenced C1 = VoxelLandDifferencedDF(DF(D[0] * P.SeaRadius + 25.0), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)F);
                const FVoxelLandDifferenced C2 = VoxelLandDifferencedDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius + 25.0), DF(D[2] * P.SeaRadius), (float)F);
                Sink += C1.Gully + C2.Gully;
            }
            LoadsNormal = (double)GNoiseLoads / Dirs.size();
#endif
            printf("footprint %7.0f cm: %.2f us without peaks, %.2f us with (+%.2f us, x%.2f); noise word loads: %.0f a sample, %.0f with a differenced normal  [%g]\n",
                F, Best[0] * 1e6, Best[1] * 1e6, (Best[1] - Best[0]) * 1e6, Best[1] / Best[0], Loads, LoadsNormal, Sink * 1e-30);
        }
    }
    else if (Mode == "dcost")
    {
        // N1: the detail layer's CPU cost (VoxelLandDetailDF) against the procedural slope-damped
        // fBm it replaced (sixteen octaves from 20 km, gain 0.5, as FVoxelDensityCPU made it), at
        // land points over the planet, at four footprints; and (built with
        // -DVOXEL_HARNESS_COUNT_NOISE) its noise word loads a sample.
        std::mt19937 Rng(31);
        std::normal_distribution<double> G;
        std::vector<std::array<double, 3>> Dirs;
        while (Dirs.size() < 20000)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            Dirs.push_back({ D[0], D[1], D[2] });
        }
        const double DW = 20000.0 * 100;
        const int DOct = std::clamp((int)std::ceil(std::log2(DW / P.Finest)) + 1, 1, 24);
        const FLay Old = MakeLayer(VoxelLayerDamped, DW / P.SeaRadius, P.SeaRadius, 0.5, DOct, VoxelHashSeed(7, 3));
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        for (double F : { P.Finest, 1000.0, 10000.0, 100000.0 })
        {
            double Sink = 0, Best[2] = { 1e9, 1e9 };
            for (int Rep = 0; Rep < 3; Rep++)
                for (int k = 0; k < 2; k++)
                {
                    const double T0 = Now();
                    for (auto& D : Dirs)
                    {
                        const double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius };
                        Sink += k == 0 ? Evaluate(Old, SP, F) : (double)VoxelLandDetailDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), (float)F, false).Unit;
                    }
                    Best[k] = std::min(Best[k], (Now() - T0) / Dirs.size());
                }
            double Loads = 0, LoadsGrad = 0;
#ifdef VOXEL_HARNESS_COUNT_NOISE
            GNoiseLoads = 0;
            for (auto& D : Dirs) Sink += VoxelLandDetailDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)F, false).Unit;
            Loads = (double)GNoiseLoads / Dirs.size();
            GNoiseLoads = 0;
            for (auto& D : Dirs) Sink += VoxelLandDetailDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)F, true).Unit;
            LoadsGrad = (double)GNoiseLoads / Dirs.size();
#endif
            printf("footprint %7.0f cm: the fBm %.2f us (%d octaves at most), the detail set %.2f us (%u scales at most); noise word loads %.1f a sample, %.1f with the gradient  [%g]\n",
                F, Best[0] * 1e6, DOct, Best[1] * 1e6, VoxelShader::LandDetailScales, Loads, LoadsGrad, Sink * 1e-30);
        }
    }
    else if (Mode == "seam")
    {
        // N1b: the set layers' face blend (VoxelLandSetLayerDF) near cube edges and corners, at
        // amplitude 1: the unit's RMS about its mean in the blend band against inside the faces
        // (the variance kept), its range, and its gradient (the weights' slope included) and its
        // change in the character against central differences. The peaks' layer, the hills' and
        // the strata's (when the planet has them), at a 50 m footprint.
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        std::mt19937 Rng(43);
        std::uniform_real_distribution<double> U01(0, 1);
        const double Band = 2.0 * 3.0 / P.N;   // the blend band in face coordinates (VOXEL_LAND_PEAK_BAND_CELLS)
        struct FNamed { const char* Name; FVoxelLandSetLayer L; bool bOn; };
        const FNamed Layers[3] = {
            { "peaks", VoxelLandPeakLayer(0.0f), P.S.LandPeakTexelsPerCell > 0.0f },
            { "hills", VoxelLandHillLayer(0.0f), P.S.LandHillTexelsPerCell > 0.0f },
            { "strata", VoxelLandStrataLayer(0.0f), P.S.LandStrataTexelsPerCell > 0.0f } };
        for (const FNamed& NL : Layers)
        {
            if (!NL.bOn) continue;
            const float C0 = 0.5f;
            const float MeanU = std::lerp((NL.L.BlendMeanG - NL.L.MeanG) * NL.L.ScaleG, (NL.L.BlendMeanR - NL.L.MeanR) * NL.L.ScaleR, C0);
            double Sq[3] = {}, Lo[3] = { 1e9, 1e9, 1e9 }, Hi[3] = { -1e9, -1e9, -1e9 };
            std::vector<double> GErr, CErr;
            const int Count = 6000;
            for (int i = 0; i < Count; i++)
                for (int k = 0; k < 3; k++)
                {
                    // k 0 inside a face, 1 on an edge (x = y, inside the band), 2 at a corner.
                    double D[3];
                    const double T1 = std::tan((1.0 + (U01(Rng) - 0.5) * Band) * M_PI / 4.0), T2 = std::tan((1.0 + (U01(Rng) - 0.5) * Band) * M_PI / 4.0);
                    if (k == 0) { D[0] = 1.0; D[1] = U01(Rng) * 1.2 - 0.6; D[2] = U01(Rng) * 1.2 - 0.6; }
                    else if (k == 1) { D[0] = 1.0; D[1] = T1; D[2] = U01(Rng) * 1.2 - 0.6; }
                    else { D[0] = 1.0; D[1] = T1; D[2] = T2; }
                    Normalize(D);
                    auto At = [&](const double* Dir, float Ch) {
                        return VoxelLandSetLayerDF(NL.L, DF(Dir[0] * P.SeaRadius), DF(Dir[1] * P.SeaRadius), DF(Dir[2] * P.SeaRadius), Ch,
                            float3(0.0f, 0.0f, 0.0f), 1.0f, 1000.0f, 5000.0f, true);
                    };
                    const FVoxelLandPeaks L0 = At(D, C0);
                    const double Dev = (double)L0.Unit - MeanU;
                    Sq[k] += Dev * Dev; Lo[k] = std::min(Lo[k], (double)L0.Unit); Hi[k] = std::max(Hi[k], (double)L0.Unit);
                    if (k == 0 || i % 4 != 0) continue;
                    double E1[3], E2[3]; Frame(D, E1, E2);
                    double Err2 = 0, Mag2 = 0;
                    for (int e = 0; e < 2; e++)
                    {
                        const double* Ev = e == 0 ? E1 : E2;
                        const double StepM = 0.5;
                        double A[3], B[3];
                        for (int a = 0; a < 3; a++) { A[a] = D[a] + StepM * 100.0 / P.SeaRadius * Ev[a]; B[a] = D[a] - StepM * 100.0 / P.SeaRadius * Ev[a]; }
                        Normalize(A); Normalize(B);
                        const double Num = ((double)At(A, C0).Unit - At(B, C0).Unit) / (2 * StepM);
                        const double Ana = L0.Gradient.x * Ev[0] + L0.Gradient.y * Ev[1] + L0.Gradient.z * Ev[2];
                        Err2 += (Num - Ana) * (Num - Ana); Mag2 += Num * Num;
                    }
                    GErr.push_back(std::sqrt(Err2) / std::max(std::sqrt(Mag2), 1e-6));
                    // The character's change: a unit character gradient along E1 makes the gradient
                    // along E1 grow by dUnit/dC.
                    const FVoxelLandPeaks LC = VoxelLandSetLayerDF(NL.L, DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), C0,
                        float3((float)E1[0], (float)E1[1], (float)E1[2]), 1.0f, 1000.0f, 5000.0f, true);
                    const double SlopeAna = (LC.Gradient.x - L0.Gradient.x) * E1[0] + (LC.Gradient.y - L0.Gradient.y) * E1[1] + (LC.Gradient.z - L0.Gradient.z) * E1[2];
                    const double SlopeNum = ((double)At(D, C0 + 0.01f).Unit - At(D, C0 - 0.01f).Unit) / 0.02;
                    CErr.push_back(std::fabs(SlopeAna - SlopeNum) / std::max(std::fabs(SlopeNum), 1e-3));
                }
            std::sort(GErr.begin(), GErr.end()); std::sort(CErr.begin(), CErr.end());
            auto Q = [](const std::vector<double>& V, double q) { return V.empty() ? -1.0 : V[std::min((size_t)(q * V.size()), V.size() - 1)]; };
            printf("%s (mean %.4f): unit RMS about the mean inside faces %.4f, on edges %.4f, at corners %.4f; range inside %.3f..%.3f, edges %.3f..%.3f, corners %.3f..%.3f;"
                " gradient in the band vs FD relative p50 %.4f p99 %.4f; character slope relative p50 %.4f p99 %.4f\n",
                NL.Name, MeanU, std::sqrt(Sq[0] / Count), std::sqrt(Sq[1] / Count), std::sqrt(Sq[2] / Count), Lo[0], Hi[0], Lo[1], Hi[1], Lo[2], Hi[2],
                Q(GErr, 0.5), Q(GErr, 0.99), Q(CErr, 0.5), Q(CErr, 0.99));
        }
    }
    else if (Mode == "cgrad")
    {
        // The peaks' analytic gradient against central differences of PeakUnit (the real,
        // spatially varying landform) at the centre's amplitude.
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        std::mt19937 Rng(41);
        std::normal_distribution<double> G;
        const double F = getenv("CG_F") ? atof(getenv("CG_F")) : 25.0;
        const double StepM = 0.2;
        std::vector<double> E, Rel, Ang;
        int Tried = 0;
        while (E.size() < 3000 && Tried < 5000000)
        {
            Tried++;
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            auto At = [&](const double* Dir) { return VoxelLandHeightDF(DF(Dir[0] * P.SeaRadius), DF(Dir[1] * P.SeaRadius), DF(Dir[2] * P.SeaRadius), (float)F, true); };
            const FVoxelLandHeight H0 = At(D);
            if (!(H0.PeakAmplitudeM > 0.0f)) continue;
            double E1[3], E2[3]; Frame(D, E1, E2);
            double Err2 = 0, Ana2 = 0;
            for (int k = 0; k < 2; k++)
            {
                const double* Ev = k == 0 ? E1 : E2;
                double A[3], B[3];
                for (int a = 0; a < 3; a++) { A[a] = D[a] + StepM * 100.0 / P.SeaRadius * Ev[a]; B[a] = D[a] - StepM * 100.0 / P.SeaRadius * Ev[a]; }
                Normalize(A); Normalize(B);
                const FVoxelLandHeight HA = At(A), HB = At(B);
                const double Num = ((double)HA.PeakUnit - HB.PeakUnit) * H0.PeakAmplitudeM / (2 * StepM);
                const double Ana = H0.PeakGradient.x * Ev[0] + H0.PeakGradient.y * Ev[1] + H0.PeakGradient.z * Ev[2];
                Err2 += (Num - Ana) * (Num - Ana); Ana2 += Ana * Ana;
            }
            E.push_back(std::sqrt(Err2));
            Rel.push_back(std::sqrt(Err2) / std::max(std::sqrt(Ana2), 0.02));
            Ang.push_back(std::atan(std::sqrt(Err2)) * 57.2957795);
        }
        for (auto* V : { &E, &Rel, &Ang }) std::sort(V->begin(), V->end());
        auto Q = [](const std::vector<double>& V, double q) { return V.empty() ? -1.0 : V[std::min((size_t)(q * V.size()), V.size() - 1)]; };
        printf("peaks gradient vs FD with the real landform (F %.0f cm, n %zu): |err| m/m p50 %.5f p90 %.5f p99 %.5f max %.5f (deg p90 %.3f p99 %.3f max %.3f); relative p50 %.4f p90 %.4f p99 %.4f\n",
            F, E.size(), Q(E, 0.5), Q(E, 0.9), Q(E, 0.99), Q(E, 1.0), Q(Ang, 0.9), Q(Ang, 0.99), Q(Ang, 1.0), Q(Rel, 0.5), Q(Rel, 0.9), Q(Rel, 0.99));
    }
    else if (Mode == "probe")
    {
        // Along one row of a look view (fraction Row from the top), the gully term at 5 m steps;
        // the largest second differences, and which octave count first shows them.
        double C[3]; FindMountain(P, C, 0);
        double E1[3], E2[3]; Frame(C, E1, E2);
        const double Width = 15000.0, Row = argc > 5 ? atof(argv[5]) : 0.6;
        const int K = 3000;
        auto At = [&](int k, int Octaves, double* OutBake = nullptr) {
            const double U = ((k + 0.5) / K - 0.5) * Width * 100.0 / P.SeaRadius, V = (0.5 - Row) * Width * 100.0 / P.SeaRadius;
            double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
            Normalize(D);
            FPlanet& PP = const_cast<FPlanet&>(P);
            const uint Save = PP.S.LandGullyOctaves;
            PP.S.LandGullyOctaves = Octaves;
            const FTerms T = Terms(P, D, P.Finest);
            PP.S.LandGullyOctaves = Save;
            if (OutBake) *OutBake = T.Bake;
            return T.Gully;
        };
        std::vector<double> G(K), B(K);
        for (int k = 0; k < K; k++) G[k] = At(k, P.S.LandGullyOctaves, &B[k]);
        std::vector<std::pair<double, int>> Kinks;
        for (int k = 2; k < K - 2; k++) Kinks.push_back({ std::fabs(G[k + 1] - 2 * G[k] + G[k - 1]) / 25.0, k });
        std::sort(Kinks.rbegin(), Kinks.rend());
        for (int n = 0, Shown = 0; n < (int)Kinks.size() && Shown < 6; n++)
        {
            const int k = Kinks[n].second;
            bool bNear = false;
            for (int m = 0; m < n; m++) bNear |= std::abs(Kinks[m].second - k) < 20;
            if (bNear) continue;
            Shown++;
            printf("kink at x %.0f m: curvature %.4f /m, gully %.1f m, bake %.0f m; bake slope here %.3f\n", (k + 0.5) / K * Width, Kinks[n].first, G[k], B[k],
                std::fabs(B[k + 1] - B[k - 1]) / 10.0);
            for (int o = 1; o <= (int)P.S.LandGullyOctaves; o++)
            {
                const double a = At(k - 1, o), b = At(k, o), c = At(k + 1, o);
                printf("   with %d octaves: curvature %.4f /m (values %.2f %.2f %.2f)\n", o, std::fabs(a - 2 * b + c) / 25.0, a, b, c);
            }
        }
        // The term along the row, for a plot.
        FILE* F = fopen("probe.txt", "w");
        for (int k = 0; k < K; k++) fprintf(F, "%d %.4f %.3f\n", k, G[k], B[k]);
        fclose(F);
    }
    else if (Mode == "turn")
    {
        std::mt19937 Rng(17);
        std::normal_distribution<double> G;
        std::vector<double> X, XS;
        while (X.size() < 50000)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            const auto B = P.S.VoxelLandBakeSampleDF(DF(D[0] * P.R), DF(D[1] * P.R), DF(D[2] * P.R), true);
            if (B.Height < 50) continue;
            const double Sl = length(B.Gradient);
            if (Sl < 0.02) continue;
            X.push_back(B.Turn * 2.0 * P.S.LandCellM);
            if (Sl > 0.2) XS.push_back(B.Turn * 2.0 * P.S.LandCellM);
        }
        for (auto* V : { &X, &XS })
        {
            std::sort(V->begin(), V->end());
            auto Q = [&](double q) { return (*V)[std::min((size_t)(q * V->size()), V->size() - 1)]; };
            printf("lambda0 x turn (rad per coarsest wavelength) on land %s: p10 %.2f p25 %.2f median %.2f p75 %.2f p90 %.2f p99 %.2f\n", V == &X ? "(slope > 0.02)" : "(slope > 0.2)",
                Q(0.1), Q(0.25), Q(0.5), Q(0.75), Q(0.9), Q(0.99));
        }
    }
    else if (Mode == "ref")
    {
        // A checksum of the CPU path's terms and the GPU path's densities at fixed points, for
        // refactors that must not change a bit.
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        std::mt19937 Rng(21);
        std::normal_distribution<double> G;
        uint64_t Hash = 1469598103934665603ull;
        auto Mix = [&](const void* D, size_t N) { const unsigned char* B = (const unsigned char*)D; for (size_t i = 0; i < N; i++) { Hash ^= B[i]; Hash *= 1099511628211ull; } };
        for (int i = 0; i < 20000; i++)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            const double F = P.Finest * (1 << (i % 8));
            const FTerms T = Terms(P, D, F);
            Mix(&T.Bake, 8); Mix(&T.Gully, 8); Mix(&T.Detail, 8);
            int Idx[3]; for (int a = 0; a < 3; a++) Idx[a] = (int)std::llround(D[a] * (P.R + (T.Height * 100.0 + P.SeaRadius - P.R)) / P.Half);
            const float Gp = VoxelSampleDensityDF(Idx[0], Idx[1], Idx[2], (float)std::max(F, P.Finest));
            Mix(&Gp, 4);
        }
        printf("reference checksum %016llx\n", (unsigned long long)Hash);
    }
    else if (Mode == "grad")
    {
        // VoxelLandHeightDF's bake and detail gradients against central differences, and the
        // material's normal schemes against central differences of the full height, at pixel
        // footprints from 100 m down to 1 cm. Mode 0 (the default): the bake and the detail
        // exact, the gullies and the detail's amplitude differenced over a pixel (as ddx/ddy
        // do); mode 1 (M1): Gully + Detail differenced over a pixel.
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        std::mt19937 Rng(23);
        std::normal_distribution<double> G;
        for (double F : { 100.0 * 100, 10.0 * 100, 1.0 * 100, 0.1 * 100, 0.01 * 100 })
        {
            std::vector<double> EB, ED, EN0, EN0F, EN1, Slope;
            int Tries = 0;
            const double PixM = F * 0.01;
            while (EB.size() < 3000 && Tries < 400000)
            {
                Tries++;
                double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
                auto At = [&](const double* Dir, bool bGrad) {
                    return VoxelLandHeightDF(DF(Dir[0] * P.SeaRadius), DF(Dir[1] * P.SeaRadius), DF(Dir[2] * P.SeaRadius), (float)F, bGrad);
                };
                const FVoxelLandHeight H0 = At(D, true);
                if (H0.Bake < 100) continue;
                double E1[3], E2[3]; Frame(D, E1, E2);
                auto Offset = [&](const double* E, double M, double* Out) { for (int a = 0; a < 3; a++) Out[a] = D[a] + M * 100.0 / P.SeaRadius * E[a]; Normalize(Out); };
                // Reference: central differences of the full height at 10 and 20 cm, Richardson-
                // extrapolated (finer steps would measure the float sums' round-off: a few
                // percent of the detail's slope at 2 cm). Every feature is a metre and up.
                const double StepM = 0.1;
                double Ref[2], Bfd[2], Dfd[2], Pix1[2], Pix0[2], Fix0[2], Bx[2], Dx[2];
                for (int k = 0; k < 2; k++)
                {
                    const double* E = k == 0 ? E1 : E2;
                    double A[3], B[3], A2[3], B2[3], C[3];
                    Offset(E, StepM, A); Offset(E, -StepM, B); Offset(E, 2 * StepM, A2); Offset(E, -2 * StepM, B2); Offset(E, PixM, C);
                    const FVoxelLandHeight HA = At(A, true), HB = At(B, true), HA2 = At(A2, true), HB2 = At(B2, true), HC = At(C, true);
                    auto Rich = [&](double V1, double V2) { return (4 * V1 - V2) / 3; };
                    Ref[k] = Rich(((HA.Height.x - HB.Height.x) + ((double)HA.Height.y - HB.Height.y)) * 0.01 / (2 * StepM),
                        ((HA2.Height.x - HB2.Height.x) + ((double)HA2.Height.y - HB2.Height.y)) * 0.01 / (4 * StepM));
                    Bfd[k] = Rich((HA.Bake - HB.Bake) / (2 * StepM), (HA2.Bake - HB2.Bake) / (4 * StepM));
                    // The detail at fixed amplitude (its unit times the centre's amplitude).
                    Dfd[k] = Rich((HA.DetailUnit - HB.DetailUnit) / (2 * StepM), (HA2.DetailUnit - HB2.DetailUnit) / (4 * StepM)) * H0.DetailAmplitudeM;
                    // A pixel's forward differences (ddx_fine).
                    Pix1[k] = ((HC.Gully + HC.Detail) - (H0.Gully + H0.Detail)) / PixM;
                    Pix0[k] = ((HC.Gully - H0.Gully) + H0.DetailUnit * (HC.DetailAmplitudeM - H0.DetailAmplitudeM)) / PixM;
                    // Close up, mode 0 differences them over a fixed 25 cm step (VoxelLandDifferencedDF).
                    {
                        double S[3]; Offset(E, 0.25, S);
                        const FVoxelLandDifferenced C1 = VoxelLandDifferencedDF(DF(S[0] * P.SeaRadius), DF(S[1] * P.SeaRadius), DF(S[2] * P.SeaRadius), (float)F);
                        Fix0[k] = ((C1.Gully - H0.Gully) + H0.DetailUnit * (C1.DetailAmplitudeM - H0.DetailAmplitudeM)) / 0.25;
                    }
                    Bx[k] = H0.BakeGradient.x * E[0] + H0.BakeGradient.y * E[1] + H0.BakeGradient.z * E[2];
                    Dx[k] = H0.DetailGradient.x * E[0] + H0.DetailGradient.y * E[1] + H0.DetailGradient.z * E[2];
                }
                EB.push_back(std::hypot(Bx[0] - Bfd[0], Bx[1] - Bfd[1]));
                ED.push_back(std::hypot(Dx[0] - Dfd[0], Dx[1] - Dfd[1]) / std::max(std::hypot(Dfd[0], Dfd[1]), 1e-3));
                auto Angle = [&](double Nx, double Ny) {
                    const double Dot = (1 + Nx * Ref[0] + Ny * Ref[1]) / (std::sqrt(1 + Nx * Nx + Ny * Ny) * std::sqrt(1 + Ref[0] * Ref[0] + Ref[1] * Ref[1]));
                    return std::acos(std::min(1.0, Dot)) * 57.2957795;
                };
                EN0.push_back(Angle(Bx[0] + Dx[0] + Pix0[0], Bx[1] + Dx[1] + Pix0[1]));
                EN1.push_back(Angle(Bx[0] + Pix1[0], Bx[1] + Pix1[1]));
                EN0F.push_back(Angle(Bx[0] + Dx[0] + Fix0[0], Bx[1] + Dx[1] + Fix0[1]));
                Slope.push_back(std::atan(std::hypot(Ref[0], Ref[1])) * 57.2957795);
            }
            for (auto* V : { &EB, &ED, &EN0, &EN0F, &EN1, &Slope }) std::sort(V->begin(), V->end());
            auto Q = [](const std::vector<double>& V, double q) { return V[std::min((size_t)(q * V.size()), V.size() - 1)]; };
            printf("pixel %6.2f m: bake |analytic-FD| median %.5f; detail relative error median %.4f p99 %.4f; normal error (deg) mode 0 median %.2f p90 %.2f p99 %.2f, mode 1 median %.2f p90 %.2f p99 %.2f, mode 0 with the 25 cm step median %.2f p90 %.2f p99 %.2f; slope median %.1f; n %zu\n",
                PixM, Q(EB, 0.5), Q(ED, 0.5), Q(ED, 0.99), Q(EN0, 0.5), Q(EN0, 0.9), Q(EN0, 0.99), Q(EN1, 0.5), Q(EN1, 0.9), Q(EN1, 0.99),
                Q(EN0F, 0.5), Q(EN0F, 0.9), Q(EN0F, 0.99), Q(Slope, 0.5), EB.size());
        }
    }
    else if (Mode == "dgrad")
    {
        // N1: the detail layer's gradient (VoxelLandDetailDF: the detail set's B-splines, the face
        // blend's weights) against central differences of its unit, at steps of 1 m and 50 cm
        // (Richardson-extrapolated; finer steps measure the float unit's round-off, about 1e-6:
        // DSTEP sets the smaller), at footprints from the finest cell to 100 m, anywhere, face
        // edges included (every tenth point put within the blend band of an edge). DSCALES: the
        // scales.
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        std::mt19937 Rng(29);
        std::normal_distribution<double> G;
        std::uniform_real_distribution<double> U01(0, 1);
        const double StepM = getenv("DSTEP") ? atof(getenv("DSTEP")) : 0.5;
        if (getenv("DSCALES")) VoxelShader::LandDetailScales = (uint)atoi(getenv("DSCALES"));
        {
            // The face blend keeps the variance: the unit's RMS on cube edges (x = y, the middle of
            // the blend band), at corners, and inside faces, at a 100 m footprint.
            std::uniform_real_distribution<double> U01b(0, 1);
            double Sq[3] = {}; const int Count = 20000;
            for (int i = 0; i < Count; i++)
                for (int k = 0; k < 3; k++)
                {
                    double D[3];
                    if (k == 0) { D[0] = 1.0; D[1] = 1.0; D[2] = U01b(Rng) * 1.6 - 0.8; }
                    else if (k == 1) { D[0] = 1.0; D[1] = 1.0 + (U01b(Rng) - 0.5) * 0.004; D[2] = 1.0 + (U01b(Rng) - 0.5) * 0.004; }
                    else { D[0] = 1.0; D[1] = U01b(Rng) * 1.4 - 0.7; D[2] = U01b(Rng) * 1.4 - 0.7; }
                    Normalize(D);
                    const double U = VoxelLandDetailDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), 10000.0f, false).Unit;
                    Sq[k] += U * U;
                }
            printf("detail unit RMS: on cube edges %.4f, at corners %.4f, inside faces %.4f\n", std::sqrt(Sq[0] / Count), std::sqrt(Sq[1] / Count), std::sqrt(Sq[2] / Count));
        }
        for (double F : { P.Finest, 100.0, 1000.0, 10000.0 })
        {
            std::vector<double> Rel, Abs, Slope;
            for (int i = 0; i < 4000; i++)
            {
                double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
                if (i % 10 == 0)
                {
                    // Near a face edge: x = y on the +z face side (both within the blend).
                    const double T = 1.0 + (U01(Rng) - 0.5) * 4.0 * 3.0 / P.N;
                    D[0] = T; D[1] = 1.0; D[2] = 0.5 + U01(Rng) * 0.4; Normalize(D);
                }
                double E1[3], E2[3]; Frame(D, E1, E2);
                auto Unit = [&](const double* E, double M) {
                    double Q[3]; for (int a = 0; a < 3; a++) Q[a] = D[a] * P.SeaRadius + E[a] * M * 100.0;
                    return (double)VoxelLandDetailDF(DF(Q[0]), DF(Q[1]), DF(Q[2]), (float)F, false).Unit;
                };
                const FVoxelLandDetail L0 = VoxelLandDetailDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)F, true);
                double Err = 0, Mag = 0;
                for (int k = 0; k < 2; k++)
                {
                    const double* E = k == 0 ? E1 : E2;
                    const double F2 = (Unit(E, 2 * StepM) - Unit(E, -2 * StepM)) / (4 * StepM), F1 = (Unit(E, StepM) - Unit(E, -StepM)) / (2 * StepM);
                    const double Fd = (4 * F1 - F2) / 3;
                    const double An = L0.Gradient.x * E[0] + L0.Gradient.y * E[1] + L0.Gradient.z * E[2];
                    Err += (An - Fd) * (An - Fd); Mag += Fd * Fd;
                }
                Abs.push_back(std::sqrt(Err));
                Rel.push_back(std::sqrt(Err) / std::max(std::sqrt(Mag), 1e-9));
                Slope.push_back(std::sqrt(Mag) * P.DetailAmp * 0.01);
            }
            for (auto* V : { &Rel, &Abs, &Slope }) std::sort(V->begin(), V->end());
            auto Q = [](const std::vector<double>& V, double q) { return V[std::min((size_t)(q * V.size()), V.size() - 1)]; };
            printf("detail layer gradient at a %.2f m footprint: relative error median %.2g p90 %.2g p99 %.2g; absolute (per m of unit) p99 %.2g, as a normal at %.0f m amplitude %.3f deg; its slope there median %.3f p99 %.3f\n",
                F * 0.01, Q(Rel, 0.5), Q(Rel, 0.9), Q(Rel, 0.99), Q(Abs, 0.99), P.DetailAmp * 0.01, std::atan(Q(Abs, 0.99) * P.DetailAmp * 0.01) * 57.2957795,
                Q(Slope, 0.5), Q(Slope, 0.99));
        }
    }
    else if (Mode == "agree")
    {
        // The GPU code path (VoxelSampleDensityDF on lattice indices, double-float throughout)
        // against the CPU path (double positions), at lattice points within metres of the
        // surface, at several footprints.
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        std::mt19937 Rng(9);
        std::normal_distribution<double> G;
        const int Count = 40000;
        std::vector<std::array<double, 3>> Dirs;
        while ((int)Dirs.size() < Count)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            // Mostly land with slope, some anywhere.
            if (Dirs.size() % 4 == 0 || Terms(P, D, 1e9, false).Bake > 200) Dirs.push_back({ D[0], D[1], D[2] });
        }
        std::vector<double> Diff(Count);
        std::vector<int> Flip(Count);
        ParallelFor(Count, [&](int i) {
            const double* D = Dirs[i].data();
            const double Footprint = P.Finest * (1 << (i % 8));
            auto Cpu = [&](const double Pt[3]) {
                const double Len = std::sqrt(Pt[0] * Pt[0] + Pt[1] * Pt[1] + Pt[2] * Pt[2]);
                double Dir[3] = { Pt[0] / Len, Pt[1] / Len, Pt[2] / Len };
                const FTerms T = Terms(P, Dir, Footprint);
                return std::clamp(T.Height * 100.0 + P.SeaRadius - P.R, 0.0, P.NA);
            };
            double P0[3] = { D[0] * P.R, D[1] * P.R, D[2] * P.R };
            const double H0 = Cpu(P0);
            std::mt19937 R2(i);
            const double Rad = P.R + H0 + ((int)(R2() % 2001) - 1000) * 0.5;
            int Idx[3]; double Pt[3];
            for (int a = 0; a < 3; a++) { Idx[a] = (int)std::llround(D[a] * Rad / P.Half); Pt[a] = Idx[a] * P.Half; }
            const double Len = std::sqrt(Pt[0] * Pt[0] + Pt[1] * Pt[1] + Pt[2] * Pt[2]);
            const double C = Len - (P.R + Cpu(Pt));
            const float Gp = VoxelSampleDensityDF(Idx[0], Idx[1], Idx[2], (float)std::max(Footprint, P.Finest));
            Diff[i] = std::fabs(Gp - C);
            Flip[i] = std::fabs(C) < P.Finest && ((Gp < 0) != (C < 0));
        });
        double Max = 0, Sq = 0; int Flips = 0;
        for (int i = 0; i < Count; i++) { Max = std::max(Max, Diff[i]); Sq += Diff[i] * Diff[i]; Flips += Flip[i]; }
        std::vector<double> S2 = Diff; std::sort(S2.begin(), S2.end());
        printf("GPU code against the CPU path at %d lattice points near the surface: max %.3g cm, rms %.3g cm, p99 %.3g cm, p99.9 %.3g cm; sign flips within a cell %d\n",
            Count, Max, std::sqrt(Sq / Count), S2[Count * 99 / 100], S2[Count * 999 / 1000], Flips);
    }
    return 0;
}
