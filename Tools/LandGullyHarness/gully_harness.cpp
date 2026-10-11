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
    uint LandN = 0, LandPad = 4, LandHasFlow = 0, LandWarpOctaves = 3;
    float LandCellM = 1.0f;
    float LandWarpInvNorm = 0, LandWarpAmplitude = 0, LandDetailAmplitude = 0;
    float2 LandHeightOffset;
    float LandGullyStrength = 0.22f;
    uint LandGullyOctaves = 0;
    float LandGullyFlowGain = 0.5f, LandGullyDetail = 0.35f;
    uint LandGullySeed = 0;
#include "VoxelLandSample.ush"
#include "VoxelLandGully.ush"
#include "VoxelDensitySample.ush"

    // The CPU backend's sampler (VoxelDensity.cpp's FVoxelLandSamplerCPU).
    struct FSampler
    {
        const float* LandHeight = nullptr;
        const float2* LandTanTable = nullptr;
        const uint* LandFlow = nullptr;
        uint LandN = 1, LandPad = 0, LandHasFlow = 0;
        float LandCellM = 1.0f;
        float LandGullyStrength = 0.0f;
        uint LandGullyOctaves = 0;
        float LandGullyFlowGain = 0.0f, LandGullyDetail = 1.0f;
        uint LandGullySeed = 0;
#undef VOXEL_LAND_SAMPLE_USH
#undef VOXEL_LAND_GULLY_USH
#include "VoxelLandSample.ush"
#include "VoxelLandGully.ush"
    };
}
using namespace VoxelShader;

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
    std::vector<uint32_t> FlowPacked;
    std::vector<float2> Tan;
    FLay Det, W[3];
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

static void Load(FPlanet& P, const std::string& Tag, int N, double RadiusM, double FinestM, double Strength)
{
    P.N = N; P.SeaRadius = RadiusM * 100.0;
    auto H = ReadFile(Tag + "_height.f32"), Fl = ReadFile(Tag + "_flow.f32"), T = ReadFile(Tag + "_tan.f32");
    P.Height.resize(H.size() / 4); std::memcpy(P.Height.data(), H.data(), H.size());
    P.FlowF.resize(Fl.size() / 4); std::memcpy(P.FlowF.data(), Fl.data(), Fl.size());
    P.Tan.resize(T.size() / 8); std::memcpy(P.Tan.data(), T.data(), T.size());
    P.FlowPacked.resize(P.FlowF.size() / 2);
    for (size_t i = 0; i < P.FlowPacked.size(); i++) P.FlowPacked[i] = ToHalf(P.FlowF[2 * i]) | ((uint32_t)ToHalf(P.FlowF[2 * i + 1]) << 16);
    float Mn = 1e9, Mx = -1e9; for (float h : P.Height) { Mn = std::min(Mn, h); Mx = std::max(Mx, h); }
    P.Cell = M_PI * 0.5 * P.SeaRadius / N;
    // VoxelLand::GullyOctavesFor and GullyReach.
    const double Lambda0 = 2.0 * P.Cell * 0.01;
    const int Octaves = std::clamp((int)std::floor(std::log2(Lambda0 / FinestM)) + 1, 0, 12);
    double GS = 0, Gn = 1; for (int o = 0; o < Octaves; o++) { GS += Gn; Gn *= 0.42; }
    const double GullyReach = 0.5 * Strength * Lambda0 * 100.0 * 0.8 * GS;
    P.DetailAmp = 250.0 * 100;
    const double Reach = 2.5 * P.DetailAmp + GullyReach + 100;
    const double Low = Mn * 100.0 - Reach, High = Mx * 100.0 + Reach;
    P.R = P.SeaRadius + Low; P.NA = High - Low;
    const double Root = (P.R + P.NA) * 1.05;
    P.Depth = (int)std::ceil(std::log2(2 * Root / 150.0));
    P.Finest = 2 * Root / std::pow(2.0, P.Depth); P.Half = P.Finest / 2;
    const double DW = 20000.0 * 100;
    const int DOct = std::clamp((int)std::ceil(std::log2(DW / P.Finest)) + 1, 1, 24);
    // On the sea-level sphere, as FVoxelDensityCPU's terrestrial constructor.
    P.Det = MakeLayer(VoxelLayerDamped, DW / P.SeaRadius, P.SeaRadius, 0.5, DOct, VoxelHashSeed(7, 3));
    for (int a = 0; a < 3; a++) P.W[a] = MakeLayer(VoxelLayerPlain, 6 * P.Cell / P.SeaRadius, P.SeaRadius, 0.5, 3, VoxelHashSeed(7, 0x11 + a));
    P.WarpAmp = 1.5 * P.Cell; P.Offset = P.SeaRadius - P.R;
    P.Strength = (float)Strength;
    FSampler& S = P.S;
    S.LandHeight = P.Height.data(); S.LandTanTable = P.Tan.data(); S.LandFlow = P.FlowPacked.data();
    S.LandN = N; S.LandPad = P.Pad; S.LandHasFlow = 1; S.LandCellM = (float)(P.Cell * 0.01);
    S.LandGullyStrength = (float)Strength; S.LandGullyOctaves = Octaves; S.LandGullyFlowGain = 0.5f; S.LandGullyDetail = 0.5f;
    S.LandGullySeed = VoxelHashSeed(7, 0x21u);
    if (getenv("GULLY_DETAIL")) S.LandGullyDetail = (float)atof(getenv("GULLY_DETAIL"));
    if (getenv("DETAIL_AMP")) P.DetailAmp = atof(getenv("DETAIL_AMP")) * 100.0;
    printf("planet %s: R %.0f km, N %d, cell %.0f m, lambda0 %.0f m, %d gully octaves, gully reach %.0f m, heights %.0f..%.0f m, lattice finest %.0f cm\n",
        Tag.c_str(), RadiusM * 1e-3, N, P.Cell * 0.01, Lambda0, Octaves, GullyReach * 0.01, Mn, Mx, P.Finest);
}

struct FTerms { double Bake, Gully, Detail, Height; float DetailScale; };

/** FVoxelDensityCPU::LandHeightAt at direction D (unit), Footprint (cm). Height in m above sea. */
static FTerms Terms(const FPlanet& P, const double D[3], double Footprint, bool bGullies = true, bool bWarpJacobian = true)
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
    const auto L = S.VoxelLandSurfaceDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), DF(Q[0]), DF(Q[1]), DF(Q[2]),
        float3((float)DW[0][0], (float)DW[0][1], (float)DW[0][2]), float3((float)DW[1][0], (float)DW[1][1], (float)DW[1][2]),
        float3((float)DW[2][0], (float)DW[2][1], (float)DW[2][2]), (float)std::max(Footprint, P.Finest), false);
    const float Wd = lerp(0.25f, 1.0f, smoothstep(-50.0f, 50.0f, L.Bake));
    const double Dd = Evaluate(P.Det, SP, std::max(Footprint, P.Finest));
    FTerms T;
    T.Bake = L.Bake; T.Gully = L.Gully; T.DetailScale = L.DetailScale;
    T.Detail = Dd * (double)((float)P.DetailAmp * Wd * L.DetailScale) * 0.01;
    T.Height = T.Bake + T.Gully + T.Detail;
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
    for (auto& O : P.Det.O) Gpu.push_back(ToGPU(O));
    for (int a = 0; a < 3; a++) for (auto& O : P.W[a].O) Gpu.push_back(ToGPU(O));
    VoxelShader::Octaves = Gpu.data();
    ContinentOctaves = 0; MountainOctaves = 0; DetailOctaves = (uint)P.Det.O.size();
    InvNormDetail = DF(P.Det.InvNorm); PlanetRadius = DF(P.R); NoiseAmplitude = DF(P.NA); HalfFinest = DF(P.Half);
    VoxelShader::LandHeight = P.Height.data(); VoxelShader::LandTanTable = P.Tan.data(); VoxelShader::LandFlow = P.FlowPacked.data();
    VoxelShader::LandN = P.N; VoxelShader::LandPad = P.Pad; VoxelShader::LandHasFlow = 1; VoxelShader::LandCellM = P.S.LandCellM;
    LandWarpOctaves = 3; LandWarpInvNorm = (float)P.W[0].InvNorm; LandWarpAmplitude = (float)P.WarpAmp;
    LandDetailAmplitude = (float)P.DetailAmp; LandHeightOffset = DF(P.Offset);
    VoxelShader::LandGullyStrength = P.S.LandGullyStrength; VoxelShader::LandGullyOctaves = P.S.LandGullyOctaves;
    VoxelShader::LandGullyFlowGain = P.S.LandGullyFlowGain; VoxelShader::LandGullyDetail = P.S.LandGullyDetail; VoxelShader::LandGullySeed = P.S.LandGullySeed;
}
// ---------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
    const std::string Tag = argc > 1 ? argv[1] : "n128";
    const int N = argc > 2 ? atoi(argv[2]) : 128;
    const double RadiusM = argc > 3 ? atof(argv[3]) : 1.0e6;
    const std::string Mode = argc > 4 ? argv[4] : "look";
    FPlanet P;
    Load(P, Tag, N, RadiusM, 30.0, 0.22);

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
        // The detail layer's exact gradient (VoxelLayerSumDampedExactDF) against central
        // differences of its sum, at steps of 20 and 10 cm (Richardson-extrapolated).
        std::vector<FVoxelOctave> Gpu;
        SetGpuGlobals(P, Gpu);
        std::mt19937 Rng(29);
        std::normal_distribution<double> G;
        std::vector<double> Rel;
        const uint First = ContinentOctaves + MountainOctaves;
        for (int i = 0; i < 3000; i++)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            double E1[3], E2[3]; Frame(D, E1, E2);
            auto Sum = [&](double M) {
                double Q[3]; for (int a = 0; a < 3; a++) Q[a] = D[a] * P.SeaRadius + E1[a] * M;
                const FVoxelLayerSum L = VoxelLayerSumDampedExactDF(First, DetailOctaves, DF(Q[0]), DF(Q[1]), DF(Q[2]), (float)P.Finest);
                return (double)L.Sum.x + L.Sum.y;
            };
            const FVoxelLayerSum L0 = VoxelLayerSumDampedExactDF(First, DetailOctaves, DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)P.Finest);
            const double F4 = (Sum(20) - Sum(-20)) / 40, F2 = (Sum(10) - Sum(-10)) / 20;
            const double Fd = (4 * F2 - F4) / 3;
            const double An = L0.Gradient.x * E1[0] + L0.Gradient.y * E1[1] + L0.Gradient.z * E1[2];
            const double Ref = VoxelLayerSumDF(First, DetailOctaves, VoxelLayerDamped, DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)P.Finest).x;
            if (i < 3) printf("  sum %.9g (plain path %.9g), gradient analytic %.6g FD %.6g (20 cm %.6g)\n", (double)L0.Sum.x, Ref, An, Fd, F4);
            Rel.push_back(std::fabs(An - Fd) / std::max(std::fabs(Fd), 1e-9));
        }
        std::sort(Rel.begin(), Rel.end());
        printf("detail layer gradient relative error: median %.2g p90 %.2g p99 %.2g\n", Rel[Rel.size() / 2], Rel[Rel.size() * 9 / 10], Rel[Rel.size() * 99 / 100]);
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
