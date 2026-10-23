// foam_check.cpp - Stage O9b's foam offline: the per-slot envelope and the point's combination
// (VoxelOceanFoam.ush, as VoxelOceanFFT.usf's AssembleCS and the hook's lookups run them) on the
// emulated cascades (VoxelOceanFFT.usf's passes in float, ocean_fft_emulation.h) with the
// runtime's layout (the short bands built at 10 m/s and scaled to the wind; the long bands' sea
// states blended), the hook's detiling, weights and mips: the spread estimate, the coverage
// against the calibration from 5 to 25 m/s, its evenness over the detiling lattice and with
// distance, the trails, the coverage setting; and the surf's foam (VoxelOceanSurf.ush): the bores'
// faces and trails, no step at the trough, the mean where unresolved, the swash's thin edge, off
// when off.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private -I. foam_check.cpp -o foam_check

#include "VoxelShaderCompat.h"
#include "ocean_fft_emulation.h"
#include "VoxelOceanShoreCore.h"
#include "VoxelOceanCoastCore.h"

namespace VoxelShader
{
#include "VoxelOceanFFTWaves.ush"
#include "VoxelOceanShoal.ush"
#include "VoxelOceanSurf.ush"
#include "VoxelOceanFoam.ush"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace OceanFFTEmulation;

static int Fails = 0;
static void Check(bool bOk, const char* What, double Value, double Limit)
{
    std::printf("  [%s] %-80s %.6g (limit %.6g)\n", bOk ? "ok" : "FAIL", What, Value, Limit);
    if (!bOk) Fails++;
}

namespace
{
    constexpr int N = GPUSize;
    constexpr int Mips = 9;

    // One slot's foam texture: (Z, Z^2) with its mips, as AssembleCS and FoamMipCS make it.
    struct FFoamTexture
    {
        std::vector<float> Z[Mips], Z2[Mips];
        double Mean = 0.0, Var = 0.0;
    };

    void MakeMips(FFoamTexture& T)
    {
        T.Z2[0].resize(T.Z[0].size());
        for (size_t i = 0; i < T.Z[0].size(); i++) T.Z2[0][i] = T.Z[0][i] * T.Z[0][i];
        for (int m = 1; m < Mips; m++)
        {
            const int Nm = N >> m;
            for (int ch = 0; ch < 2; ch++)
            {
                const std::vector<float>& P = ch ? T.Z2[m - 1] : T.Z[m - 1];
                std::vector<float>& Q = ch ? T.Z2[m] : T.Z[m];
                Q.assign(size_t(Nm) * Nm, 0.0f);
                for (int j = 0; j < Nm; j++)
                    for (int i = 0; i < Nm; i++)
                        Q[size_t(j) * Nm + i] = 0.25f * (P[size_t(2 * j) * 2 * Nm + 2 * i] + P[size_t(2 * j) * 2 * Nm + 2 * i + 1]
                            + P[size_t(2 * j + 1) * 2 * Nm + 2 * i] + P[size_t(2 * j + 1) * 2 * Nm + 2 * i + 1]);
            }
        }
        // The last mip: the slot's statistics (VoxelOceanFoamSlotsOf).
        T.Mean = T.Z[Mips - 1][0];
        T.Var = std::max(double(T.Z2[Mips - 1][0]) - T.Mean * T.Mean, 0.0);
    }

    // A trilinear lookup at (X, Y) m of a tile L m long (the hook's half-texel shift), mip Lod.
    void Sample(const FFoamTexture& T, double X, double Y, double L, double Lod, double& OutZ, double& OutZ2)
    {
        Lod = std::clamp(Lod, 0.0, double(Mips - 1));
        const double Half = 0.5 * L / N;
        auto Bilinear = [&](int m, const std::vector<float>* Ch) {
            const int Nm = N >> m;
            const std::vector<float>& F = Ch[m];
            const double U = (X + Half) / L * Nm - 0.5, V = (Y + Half) / L * Nm - 0.5;
            const double FU = std::floor(U), FV = std::floor(V);
            const int I = int(FU), J = int(FV);
            const double A = U - FU, C = V - FV;
            auto At = [&](int i, int j) { return double(F[size_t(((j % Nm) + Nm) % Nm) * Nm + (((i % Nm) + Nm) % Nm)]); };
            return (1 - C) * ((1 - A) * At(I, J) + A * At(I + 1, J)) + C * ((1 - A) * At(I, J + 1) + A * At(I + 1, J + 1));
        };
        const int M0 = std::min(int(std::floor(Lod)), Mips - 1), M1 = std::min(M0 + 1, Mips - 1);
        const double Tm = Lod - M0;
        OutZ = (1 - Tm) * Bilinear(M0, T.Z) + Tm * Bilinear(M1, T.Z);
        OutZ2 = (1 - Tm) * Bilinear(M0, T.Z2) + Tm * Bilinear(M1, T.Z2);
    }

    // The long bands' sea-state weights (VoxelOceanStateWeights).
    void StateWeights(double U, double W[3])
    {
        const VoxelShader::float3 S = VoxelShader::VoxelOceanStateWeights(float(U));
        W[0] = S.x; W[1] = S.y; W[2] = S.z;
    }
}

int main()
{
    using VoxelShader::float4;
    std::printf("1. The shared envelope, threshold and combination\n");
    {
        // The envelope falls one sigma per decay.
        const double Dt = 1.0 / 60.0, Decay = 2.5;
        float Z = 3.0f;
        int Frames = 0;
        while (Z > 2.0f && Frames < 100000) { Z = VoxelShader::VoxelOceanFoamStep(Z, -5.0f, float(Dt / Decay)); Frames++; }
        Check(std::abs(Frames * Dt - Decay) < 0.02, "the envelope falls one sigma in the decay (s)", Frames * Dt, Decay);
        Check(VoxelShader::VoxelOceanFoamStep(0.5f, 1.2f, 0.01f) == 1.2f && VoxelShader::VoxelOceanFoamStep(-5.99f, -9.0f, 0.1f) == VOXEL_OCEAN_FOAM_LOW,
            "it rises with the compression at once, floored", 0, 0);
        Check(std::abs(VoxelShader::VoxelOceanFoamCompression(1.0f - 0.25f, 0.1f) - 2.5f) < 1e-5f, "the compression (1 - J) / sigma_J", 0, 0);
        double QErr = 0.0;
        const double Ps[] = { 0.4, 0.1587, 0.0228, 0.00135, 1e-5 };
        const double Xs[] = { 0.2533, 1.0, 2.0, 3.0, 4.2649 };
        for (int k = 0; k < 5; k++) QErr = std::max(QErr, std::abs(VoxelShader::VoxelOceanFoamQuantile(float(Ps[k])) - Xs[k]));
        Check(QErr < 2e-3, "the normal's upper quantile within 2e-3", QErr, 2e-3);
        Check(VoxelShader::VoxelOceanFoamThreshold(2.5f, 1.0f) >= VOXEL_OCEAN_FOAM_NONE && VoxelShader::VoxelOceanFoamThreshold(10.0f, 0.0f) >= VOXEL_OCEAN_FOAM_NONE,
            "no whitecaps at 2.5 m/s or with the coverage at 0", 0, 0);
        bool bFalls = true;
        for (double U = 3.5; U < 24.0; U += 0.5)
            bFalls = bFalls && VoxelShader::VoxelOceanFoamThreshold(float(U + 0.5), 1.0f) < VoxelShader::VoxelOceanFoamThreshold(float(U), 1.0f);
        Check(bFalls && VoxelShader::VoxelOceanFoamThreshold(10.0f, 2.0f) < VoxelShader::VoxelOceanFoamThreshold(10.0f, 1.0f),
            "the threshold falls as the wind (to 24 m/s) or the coverage rises", 0, 0);
        // Stage W3i: the coverage levels off in very high winds: the power law within 4% to 20 m/s,
        // about a half at 35 m/s, never the whole sea.
        {
            auto Law = [](double U) { return VOXEL_OCEAN_FOAM_COVER_AT10 * std::pow(U / 10.0, VOXEL_OCEAN_FOAM_COVER_POWER); };
            double Worst = 0.0;
            for (double U = 5.0; U <= 20.0; U += 1.0) Worst = std::max(Worst, 1.0 - VoxelShader::VoxelOceanFoamCoverage(float(U), 1.0f) / Law(U));
            const double At35 = VoxelShader::VoxelOceanFoamCoverage(35.0f, 1.0f), At60 = VoxelShader::VoxelOceanFoamCoverage(60.0f, 1.0f);
            Check(Worst < 0.04, "the coverage follows the power law within 4% to 20 m/s", Worst, 0.04);
            Check(At35 > 0.4 && At35 < 0.6 && At60 < VOXEL_OCEAN_FOAM_COVER_MAX, "and levels off: about a half at 35 m/s, under the most at 60", At35, 0.6);
        }
        // Stage W3f: the density's ramp. Its mean over a standard normal is the coverage (the ramp
        // placed by two Newton steps); it rises from none well below the threshold to solid well
        // above it, monotonically; no long bands, none.
        double RampErr = 0.0;
        for (double P : { 0.002, 0.01, 0.03, 0.1, 0.3 })
        {
            const float Lo = VoxelShader::VoxelOceanFoamRampStart(float(P));
            // The mean by quadrature against the closed form's.
            double Sum = 0.0;
            for (int k = -4000; k <= 4000; k++)
            {
                const double X = k * 0.002;
                Sum += std::clamp((X - Lo) / VOXEL_OCEAN_FOAM_RAMP, 0.0, 1.0) * std::exp(-0.5 * X * X) * 0.002 / std::sqrt(2.0 * M_PI);
            }
            RampErr = std::max(RampErr, std::abs(Sum - P) / P);
        }
        Check(RampErr < 0.01, "the ramp's mean over the sea is the coverage within 1%", RampErr, 0.01);
        VoxelShader::FVoxelOceanFoamBands B = VoxelShader::VoxelOceanFoamBandsZero();
        B.Sigma = float4(1.0f, 1.0f, 1.0f, 1.0f);
        B.Steep = float4(1.0f, 0.0f, 0.0f, 0.0f);
        B.LongWeight2 = 1.0f; B.ShortWeight2 = 1.0f;
        const float T10 = VoxelShader::VoxelOceanFoamThreshold(10.0f, 1.0f);
        bool bRises = true;
        float Before = -1.0f;
        for (float Dv = T10 - 2.0f; Dv <= T10 + 2.0f; Dv += 0.05f)
        {
            B.Dev = float4(Dv, 0.0f, 0.0f, 0.0f);
            const float Density = VoxelShader::VoxelOceanFoamOf(B, 10.0f, 1.0f);
            bRises = bRises && Density >= Before - 1e-6f;
            Before = Density;
        }
        B.Dev = float4(T10 - 2.0f, 0.0f, 0.0f, 0.0f);
        const float Low = VoxelShader::VoxelOceanFoamOf(B, 10.0f, 1.0f);
        B.Dev = float4(T10 + 2.0f, 0.0f, 0.0f, 0.0f);
        const float High = VoxelShader::VoxelOceanFoamOf(B, 10.0f, 1.0f);
        B.Steep = float4(0.0f, 0.0f, 1.0f, 1.0f);
        B.Dev = float4(0.0f, 0.0f, 10.0f, 10.0f);
        Check(bRises && Low < 0.01f && High > 0.99f && VoxelShader::VoxelOceanFoamOf(B, 10.0f, 1.0f) == 0.0f,
            "the density rises from none (2 sigma below) to solid (2 sigma above); none without the long bands", High - Low, 0.98);
    }

    std::printf("\n2. The emulated cascades (the runtime layout; 15 Hz for 20 s, decay 2.5 s; detiled, cell 1.5 km)\n");
    FLayoutSettings S;
    S.PlanetSeed = 1234567u;
    for (int b = 0; b < BandCount; b++) S.Chop[b] = 0.9;
    S.ShortWind = 10.0;
    const std::vector<FSlot> Slots = MakeSlots(S);
    double Slope[SlotCount], SigmaJ[SlotCount], ShortRef[2];
    for (int s = 0; s < SlotCount; s++)
    {
        const FBandStats St = StatsOf(Slots[s]);
        Slope[s] = std::sqrt(St.SlopeVarianceX + St.SlopeVarianceY);
        SigmaJ[s] = VOXEL_OCEAN_FOAM_CHOP * Slope[s];
    }
    for (int b = 0; b < 2; b++) ShortRef[b] = std::sqrt(StatsOf(Slots[6 + b]).HeightVariance);
    std::vector<FFoamTexture> Tex(SlotCount);
    std::vector<float> Present[SlotCount];
    double PresentMean[SlotCount], PresentSd[SlotCount];
    double WorstSpread = 0.0;
    {
        const double Dt = 1.0 / 15.0, Decay = 2.5;
        for (int s = 0; s < SlotCount; s++) { Tex[s].Z[0].assign(size_t(N) * N, 0.0f); Present[s].assign(size_t(N) * N, 0.0f); }
        for (int Step = 0; Step < 300; Step++)
        {
            const uint32_t Tick = TickOf(100.0 + Step * Dt);
            for (int s = 0; s < SlotCount; s++)
            {
                if (SigmaJ[s] <= 1e-6) continue;
                const FGPUSlot G = EmulateGPU(Slots[s], Tick);
                const float K = float(VOXEL_OCEAN_FOAM_CHOP / Slots[s].Chop);
                double Sum = 0.0, Sum2 = 0.0;
                for (size_t i = 0; i < Present[s].size(); i++)
                {
                    const float Dxx = K * G.Derivatives[i].z, Dyy = K * G.Derivatives[i].w, Dxy = K * G.Displacement[i].w;
                    const float J = (1.0f + Dxx) * (1.0f + Dyy) - Dxy * Dxy;
                    const float C = VoxelShader::VoxelOceanFoamCompression(J, float(SigmaJ[s]));
                    Tex[s].Z[0][i] = VoxelShader::VoxelOceanFoamStep(Tex[s].Z[0][i], C, float(Dt / Decay));
                    Present[s][i] = C;
                    Sum += J; Sum2 += double(J) * J;
                }
                if (Step == 0 && SigmaJ[s] > 1e-3)
                {
                    const double Measured = std::sqrt(Sum2 / Present[s].size() - std::pow(Sum / Present[s].size(), 2));
                    WorstSpread = std::max(WorstSpread, std::abs(Measured / SigmaJ[s] - 1.0));
                }
            }
        }
        for (int s = 0; s < SlotCount; s++)
        {
            MakeMips(Tex[s]);
            double A = 0.0, B2 = 0.0;
            for (float V : Present[s]) { A += V; B2 += double(V) * V; }
            A /= Present[s].size();
            PresentMean[s] = A;
            PresentSd[s] = std::sqrt(std::max(B2 / Present[s].size() - A * A, 1e-12));
        }
    }
    Check(WorstSpread < 0.12, "sigma_J = chop sqrt(slope variance) within 12% of the measured", WorstSpread, 0.12);

    // The hook's combination at a point (VoxelOceanFFTChartLookups, VoxelOceanFFTFoamOf) for one
    // chart and heading: footprint Fp (m), wind U, coverage Cov. Present: the present compression
    // instead of the envelope (standardised as it is), for the trails.
    struct FPoint { double Foam; double Long; int Kind; };
    auto PointAt = [&](double X, double Y, double U, double Cov, double Fp, bool bPresent) {
        double W[3];
        StateWeights(U, W);
        const double Sum = W[0] + W[1] + W[2];
        const double WN[3] = { W[0] / Sum, W[1] / Sum, W[2] / Sum };
        double Scale[2];
        for (int b = 0; b < 2; b++)
        {
            FLayoutSettings A = S;
            A.ShortWind = std::max(U, 0.5);
            static thread_local double CacheU[2] = { -1, -1 }, CacheV[2] = { 0, 0 };
            if (CacheU[b] != U) { CacheU[b] = U; CacheV[b] = std::sqrt(StatsOf(MakeSlots(A)[6 + b]).HeightVariance) / ShortRef[b]; }
            Scale[b] = CacheV[b];
        }
        // Each band's own lattice (stage O9b3, VoxelOceanDetileOfBand), the short bands' too.
        const float4 TileSides(float(Slots[0].Params.TileLength), float(Slots[1].Params.TileLength), float(Slots[6].Params.TileLength), float(Slots[7].Params.TileLength));
        double Wc0[3] = {};
        double Dev[4] = {}, Res[4] = {}, Roots[3] = {};
        for (int b = 0; b < 4; b++)
        {
            const VoxelShader::FVoxelOceanDetile Det = VoxelShader::VoxelOceanDetileOfBand(VoxelShader::float2(float(X), float(Y)), 1500.0f, 77u, b, TileSides);
            const double Wc[3] = { Det.Weight.x, Det.Weight.y, Det.Weight.z };
            const double Ox[3] = { Det.Offset0.x, Det.Offset1.x, Det.Offset2.x }, Oy[3] = { Det.Offset0.y, Det.Offset1.y, Det.Offset2.y };
            const double Copies2 = Wc[0] * Wc[0] + Wc[1] * Wc[1] + Wc[2] * Wc[2];
            if (b == 0) { Wc0[0] = Wc[0]; Wc0[1] = Wc[1]; Wc0[2] = Wc[2]; }
            const double L = Slots[b < 2 ? b : 4 + b].Params.TileLength;
            const double Lod = std::log2(std::max(Fp * 256.0 / L, 1.0));
            // The deviation added; the variance the lookup leaves unresolved returned.
            auto Look = [&](int s, double WF, double PX, double PY) {
                if (bPresent)
                {
                    const double Uq = (PX + 0.5 * L / N) / L * N, Vq = (PY + 0.5 * L / N) / L * N;
                    const double C = Present[s][size_t(int(std::floor(Vq)) & (N - 1)) * N + (int(std::floor(Uq)) & (N - 1))];
                    Dev[b] += WF * (C - PresentMean[s]) / PresentSd[s] * std::sqrt(Tex[s].Var);
                    return 0.0;
                }
                double Z, Z2;
                Sample(Tex[s], PX, PY, L, Lod, Z, Z2);
                Dev[b] += WF * (Z - Tex[s].Mean);
                return std::max(Z2 - Z * Z, 0.0);
            };
            // The hook's: the sea states' unresolved deviations add per copy (the square of the
            // summed roots), the copies' variances by their squared weights.
            if (Lod >= 8.0 && !bPresent)
            {
                if (b < 2)
                {
                    double Root = 0.0;
                    for (int K = 0; K < 3; K++) if (W[K] > 0) Root += WN[K] * std::sqrt(Tex[2 * K + b].Var);
                    Res[b] = Copies2 * Root * Root;
                }
                else
                    Res[b] = Copies2 * Tex[4 + b].Var;
                continue;
            }
            if (b < 2)
            {
                Roots[0] = Roots[1] = Roots[2] = 0.0;
                for (int K = 0; K < 3; K++)
                {
                    if (W[K] <= 0) continue;
                    for (int c = 0; c < 3; c++)
                        if (Wc[c] > 0) Roots[c] += WN[K] * std::sqrt(Look(2 * K + b, WN[K] * Wc[c], X + Ox[c], Y + Oy[c]));
                }
                Res[b] = Wc[0] * Wc[0] * Roots[0] * Roots[0] + Wc[1] * Wc[1] * Roots[1] * Roots[1] + Wc[2] * Wc[2] * Roots[2] * Roots[2];
            }
            else
            {
                Res[b] = 0.0;
                for (int c = 0; c < 3; c++)
                    if (Wc[c] > 0) Res[b] += Wc[c] * Wc[c] * Look(4 + b, Wc[c], X + Ox[c], Y + Oy[c]);
            }
        }
        VoxelShader::FVoxelOceanFoamBands B = VoxelShader::VoxelOceanFoamBandsZero();
        B.Dev = float4(float(Dev[0]), float(Dev[1]), float(Dev[2]), float(Dev[3]));
        B.Res = float4(float(Res[0]), float(Res[1]), float(Res[2]), float(Res[3]));
        auto Sd = [&](int s) { return std::sqrt(Tex[s].Var); };
        B.Sigma = float4(float(WN[0] * Sd(0) + WN[1] * Sd(2) + WN[2] * Sd(4)), float(WN[0] * Sd(1) + WN[1] * Sd(3) + WN[2] * Sd(5)), float(Sd(6)), float(Sd(7)));
        // The steepness as VoxelOceanBandSlopeVariance gives it (the slots' slope deviations).
        B.Steep = float4(float(W[0] * Slope[0] + W[1] * Slope[2] + W[2] * Slope[4]), float(W[0] * Slope[1] + W[1] * Slope[3] + W[2] * Slope[5]),
            float(Scale[0] * Slope[6]), float(Scale[1] * Slope[7]));
        B.LongWeight2 = 1.0f;
        B.ShortWeight2 = 1.0f;
        FPoint P;
        P.Foam = VoxelShader::VoxelOceanFoamOf(B, float(U), float(Cov));
        const double S0 = B.Steep.x * B.Sigma.x, S1 = B.Steep.y * B.Sigma.y;
        P.Long = (B.Steep.x * Dev[0] + B.Steep.y * Dev[1]) / std::sqrt(S0 * S0 + S1 * S1);
        const double Mx = std::max({ Wc0[0], Wc0[1], Wc0[2] });
        P.Kind = Mx > 0.97 ? 1 : (Mx < 0.7 ? 2 : 0);
        return P;
    };

    struct FStats { double Coverage, Mean, Vertex, Centre, ShortLed; };
    auto Survey = [&](double U, double Cov, double Fp, int Count) {
        std::mt19937 Rng(3);
        FStats St = {};
        int Foamy = 0, NV = 0, NC = 0, FV = 0, FC = 0, ShortLed = 0;
        const double T = VoxelShader::VoxelOceanFoamThreshold(float(U), float(Cov));
        for (int k = 0; k < Count; k++)
        {
            const double X = std::uniform_real_distribution<double>(0, 5000)(Rng), Y = std::uniform_real_distribution<double>(0, 5000)(Rng);
            const FPoint P = PointAt(X, Y, U, Cov, Fp, false);
            const bool bFoamy = P.Foam > 0.5;
            Foamy += bFoamy;
            St.Mean += P.Foam;
            if (bFoamy && P.Long < T - VOXEL_OCEAN_FOAM_GATE * 0.5) ShortLed++;
            if (P.Kind == 1) { NV++; FV += bFoamy; }
            if (P.Kind == 2) { NC++; FC += bFoamy; }
        }
        St.Coverage = double(Foamy) / Count;
        St.Mean /= Count;
        St.Vertex = double(FV) / std::max(NV, 1);
        St.Centre = double(FC) / std::max(NC, 1);
        St.ShortLed = double(ShortLed) / std::max(Foamy, 1);
        return St;
    };

    std::printf("   wind  target  coverage  (lattice: by a vertex, a cell's centre)  mean foam at 0, 8, 32 m footprints\n");
    const double Winds[] = { 5, 7, 10, 12, 14, 18, 25 };
    double WorstRatio = 0.0, WorstLattice = 1.0, WorstFar = 1.0, WorstShortLed = 0.0;
    double Cov5 = 0.0;
    for (double U : Winds)
    {
        const double Target = VoxelShader::VoxelOceanFoamCoverage(float(U), 1.0f);
        const FStats Near = Survey(U, 1.0, 0.0, 30000);
        const FStats Mid = Survey(U, 1.0, 8.0, 10000);
        const FStats Far = Survey(U, 1.0, 32.0, 10000);
        std::printf("   %4.0f  %5.2f%%  %6.2f%%   (%5.2f%%, %5.2f%%)                       %.4f %.4f %.4f\n", U, 100 * Target, 100 * Near.Coverage,
            100 * Near.Vertex, 100 * Near.Centre, Near.Mean, Mid.Mean, Far.Mean);
        if (U == 5) { Cov5 = Near.Coverage; continue; }
        WorstRatio = std::max(WorstRatio, std::max(Near.Coverage / Target, Target / Near.Coverage));
        if (U >= 10)
        {
            const double R = Near.Centre / std::max(Near.Vertex, 1e-9);
            WorstLattice = std::min(WorstLattice, std::min(R, 1.0 / R));
        }
        if (U >= 10)
            WorstFar = std::min({ WorstFar, Mid.Mean / Near.Mean, Far.Mean / Near.Mean, Near.Mean / std::max(Far.Mean, 1e-9) });
        WorstShortLed = std::max(WorstShortLed, Near.ShortLed);
    }
    Check(Cov5 < 0.006, "5 m/s: under 0.6% (Monahan 0.1%: the skew of one band)", Cov5, 0.006);
    Check(WorstRatio < 1.6, "7 to 25 m/s, between the sea states too: coverage within 1.6x of the calibration", WorstRatio, 1.6);
    Check(WorstLattice > 0.75, "10 m/s and up: as much foam in the detiling cells' centres as by their vertices (0.75x)", WorstLattice, 0.75);
    Check(WorstFar > 0.6, "10 m/s and up: the mean foam at 8 m and 32 m footprints within 0.6x of the near", WorstFar, 0.6);
    Check(WorstShortLed < 0.1, "the long bands lead: under a tenth of the foam where they're far below", WorstShortLed, 0.1);

    {
        const FStats One = Survey(10.0, 1.0, 0.0, 30000), Two = Survey(10.0, 2.0, 0.0, 30000);
        const double R = Two.Coverage / One.Coverage;
        std::printf("   coverage setting 2 at 10 m/s: %.2f%% (%.2fx)\n", 100 * Two.Coverage, R);
        Check(R > 1.5 && R < 2.6, "the coverage setting at 2 about doubles the foam", R, 2.0);
    }
    {
        // Trails: foam the present compression alone wouldn't make.
        std::mt19937 Rng(5);
        int Foamy = 0, Trail = 0;
        const double U = 10.0;
        const double T = VoxelShader::VoxelOceanFoamThreshold(float(U), 1.0f);
        for (int k = 0; k < 30000; k++)
        {
            const double X = std::uniform_real_distribution<double>(0, 5000)(Rng), Y = std::uniform_real_distribution<double>(0, 5000)(Rng);
            const FPoint P = PointAt(X, Y, U, 1.0, 0.0, false);
            if (P.Foam <= 0.5) continue;
            Foamy++;
            if (PointAt(X, Y, U, 1.0, 0.0, true).Long < T) Trail++;
        }
        const double Share = double(Trail) / std::max(Foamy, 1);
        std::printf("   at 10 m/s, %.0f%% of the foam is trail (the present compression alone wouldn't make it)\n", 100 * Share);
        Check(Share > 0.4 && Share < 0.95, "the foam lingers: most of it is trail (whitecaps: residual foam mostly)", Share, 0.4);
    }

    std::printf("\n3. The surf's foam (VoxelOceanSurf.ush; a 1:50 beach, 1.5 m waves at 12 m/s)\n");
    {
        using namespace VoxelShader;
        auto Settings = [](float Foam) { return VoxelOceanSurfSettingsOf(float4(1.0f, 1.0f, 1.0f, 0.85f), float4(0.0f, Foam, 1.0f, 0.0f), float4(0.2f, 0.25f, 0.0f, 0.0f), 0.78f); };
        // The coastal field on the beach (stage O9i): along straight rays from the seed depth
        // (40 m) in, A = integral ds / sqrt(held depth), B = integral sqrt(held depth) ds,
        // tabulated every 5 cm from 1 m up the beach.
        const double M = 0.02, X0 = -50.0, Step = 0.05, XSeed = 40.0 / M;
        std::vector<double> TA(size_t((XSeed - X0) / Step) + 2, 0.0), TB(TA.size(), 0.0);
        for (int i = int(TA.size()) - 2; i >= 0; i--)
        {
            const double Xa = X0 + Step * i, Xb = Xa + Step;
            if (Xb > XSeed) continue;
            const double Ha = VoxelOceanCoastHeldDepth(float(M * Xa)), Hb = VoxelOceanCoastHeldDepth(float(M * Xb));
            TA[size_t(i)] = TA[size_t(i + 1)] + 0.5 * Step * (1.0 / std::sqrt(Ha) + 1.0 / std::sqrt(Hb));
            TB[size_t(i)] = TB[size_t(i + 1)] + 0.5 * Step * (std::sqrt(Ha) + std::sqrt(Hb));
        }
        auto Lerp = [&](const std::vector<double>& T, double X) {
            const double U = std::clamp((X - X0) / Step, 0.0, double(T.size() - 2));
            const int I = int(U);
            return T[size_t(I)] + (T[size_t(I + 1)] - T[size_t(I)]) * (U - I);
        };
        auto Input = [&](float D, float T, float Footprint) {
            FVoxelOceanSurfInput In;
            In.DepthM = D; In.Slope = 0.02f; In.ShoreM = D / 0.02f; In.Hs = 1.5f; In.Speed = 12.0f;
            In.Units = int3(24414, 48, 7); In.Low = float3(1000.0f - 100.0f * D / 0.02f, 300.0f, 30.0f);
            In.Time = T; In.SpacingM = 0.0f; In.FootprintM = Footprint; In.BedSlope = VOXEL_OCEAN_BED_UNKNOWN; In.BedDepthM = 1.0e30f; In.ForNormals = Footprint > 0.0f ? 1.0f : 0.0f;
            const double X = D / M, Held = VoxelOceanCoastHeldDepth(D);
            In.CoastA = float(Lerp(TA, X)); In.CoastB = float(Lerp(TB, X));
            In.CoastGradA = float3(float(1.0 / std::sqrt(Held)), 0.0f, 0.0f); In.CoastGradB = float3(float(std::sqrt(Held)), 0.0f, 0.0f);
            In.CoastCover = 1.0f; In.CoastSeedM = 40.0f; In.CoastReachM = 8000.0f;
            return In;
        };
        const FVoxelOceanSurfSettings On = Settings(1.0f), Off = Settings(0.0f);
        double BoreMax = 0.0, BoreMean = 0.0, OffshoreMax = 0.0, OffMax = 0.0, EdgeMax = 0.0, SheetMin = 1.0, Jump = 0.0;
        double FarMean = 0.0, FarMin = 1.0, FarMax = 0.0;
        int BoreCount = 0;
        float Last = VoxelOceanSurfOf(Input(1.2f, 0.0f, 0.0f), On).Foam;
        for (int k = 0; k < 4000; k++)
        {
            const float T = 0.05f * float(k);
            const FVoxelOceanSurf B = VoxelOceanSurfOf(Input(1.2f, T, 0.0f), On);
            BoreMax = std::max(BoreMax, double(B.Foam));
            BoreMean += B.Foam; BoreCount++;
            OffshoreMax = std::max(OffshoreMax, double(VoxelOceanSurfOf(Input(4.6f, T, 0.0f), On).Foam));
            OffMax = std::max(OffMax, double(VoxelOceanSurfOf(Input(1.2f, T, 0.0f), Off).Foam) + double(VoxelOceanSurfOf(Input(-0.1f, T, 0.0f), Off).Foam));
            // Seen from far (a 200 m footprint): the bores' mean, steady.
            const float Far = VoxelOceanSurfOf(Input(1.2f, T, 200.0f), On).Foam;
            FarMean += Far; FarMin = std::min(FarMin, double(Far)); FarMax = std::max(FarMax, double(Far));
            // On the beach 10 cm up: where the swash's sheet covers it, thin, foamy.
            const FVoxelOceanSurf Sw = VoxelOceanSurfOf(Input(-0.1f, T, 0.0f), On);
            const float Thickness = Sw.Height - 0.1f;
            if (Thickness > 0.0f && Thickness < 0.02f) EdgeMax = std::max(EdgeMax, double(Sw.Foam));
            if (Thickness > 0.3f) SheetMin = std::min(SheetMin, double(Sw.Foam));
        }
        // No step through the cycle: fine steps in time at 1.2 m (a 13 s cycle in 0.002 s steps).
        for (int k = 1; k < 20000; k++)
        {
            const float F = VoxelOceanSurfOf(Input(1.2f, 0.002f * float(k), 0.0f), On).Foam;
            Jump = std::max(Jump, double(std::abs(F - Last)));
            Last = F;
        }
        BoreMean /= BoreCount;
        FarMean /= BoreCount;
        std::printf("  bores at 1.2 m deep: foam up to %.2f, mean %.2f; at 4.6 m deep (unbroken) up to %.2f\n", BoreMax, BoreMean, OffshoreMax);
        Check(BoreMax > 0.9 && BoreMean > 0.25 && BoreMean < 0.8, "the breaking bores are white on their faces with trails behind", BoreMean, 0.25);
        Check(OffshoreMax < 0.25, "unbroken waves past the surf zone carry little", OffshoreMax, 0.25);
        Check(Jump < 0.01, "no step through the cycle (the trail meets the face at the trough; it stepped 0.017)", Jump, 0.01);
        std::printf("  seen with a 200 m footprint: %.2f to %.2f (mean %.2f; near %.2f)\n", FarMin, FarMax, FarMean, BoreMean);
        Check(FarMax - FarMin < 0.25 * BoreMax && std::abs(FarMean - BoreMean) < 0.15, "unresolved, the bores' foam is their cycle's mean (no stripes)", FarMax - FarMin, 0.25 * BoreMax);
        std::printf("  the swash 10 cm up the beach: at its thin edge up to %.2f; under 30 cm of it at least %.2f\n", EdgeMax, SheetMin);
        Check(EdgeMax > 0.8, "the swash's thin edge is foamy", EdgeMax, 0.8);
        Check(SheetMin >= 0.19 && SheetMin < 0.4, "a deeper swash sheet a fifth foamy", SheetMin, 0.2);
        Check(OffMax == 0.0, "no surf foam with the setting at 0", OffMax, 0.0);
    }

    std::printf("\n4. The whitecaps (stage W3h; VoxelOceanWhitecapOf): the summed surface's area ratio on the emulated cascades\n");
    {
        // The slots' fields at one moment (the waves' own chop, 0.9 here); every band summed at a
        // point as the hook sums them (the long bands' sea states by their weights, the short bands
        // scaled to the wind), one copy, at the finest mip: the whitecaps' share of the sea against
        // VOXEL_OCEAN_WHITECAP_SHARE of the foam's coverage.
        std::vector<FGPUSlot> G(SlotCount);
        const uint32_t Tick = TickOf(137.0);
        for (int s = 0; s < SlotCount; s++) G[s] = EmulateGPU(Slots[s], Tick);
        auto Bilin = [&](const std::vector<F4>& F, double U, double V, int Comp) {
            const double X = U * N - 0.5, Y = V * N - 0.5;
            const int X0 = int(std::floor(X)), Y0 = int(std::floor(Y));
            const double Fx = X - X0, Fy = Y - Y0;
            auto At = [&](int x, int y) { const F4& T = F[size_t(y & (N - 1)) * N + (x & (N - 1))]; return Comp == 0 ? T.z : (Comp == 1 ? T.w : T.w); };
            (void)At;
            auto Get = [&](int x, int y) -> double {
                const F4& T = F[size_t(y & (N - 1)) * N + (x & (N - 1))];
                return Comp == 0 ? T.z : T.w;
            };
            return (Get(X0, Y0) * (1 - Fx) + Get(X0 + 1, Y0) * Fx) * (1 - Fy) + (Get(X0, Y0 + 1) * (1 - Fx) + Get(X0 + 1, Y0 + 1) * Fx) * Fy;
        };
        double WorstRatio = 0.0;
        double Ratio10 = 0.0;
        for (double U : { 7.0, 10.0, 14.0, 18.0, 25.0 })
        {
            double W[3];
            StateWeights(U, W);
            double Scale[2];
            for (int b = 0; b < 2; b++)
            {
                FLayoutSettings A = S;
                A.ShortWind = std::max(U, 0.5);
                Scale[b] = std::sqrt(StatsOf(MakeSlots(A)[6 + b]).HeightVariance) / ShortRef[b];
            }
            const double Steep[4] = { W[0] * Slope[0] + W[1] * Slope[2] + W[2] * Slope[4], W[0] * Slope[1] + W[1] * Slope[3] + W[2] * Slope[5],
                Scale[0] * Slope[6], Scale[1] * Slope[7] };
            const float SlopeVar = float(Steep[0] * Steep[0] + Steep[1] * Steep[1] + Steep[2] * Steep[2] + Steep[3] * Steep[3]);
            std::mt19937 Rng(11);
            std::uniform_real_distribution<double> Pos(0.0, 4000.0);
            double Sum = 0.0;
            const int Count = 60000;
            std::vector<double> Cs;
            for (int n = 0; n < Count; n++)
            {
                const double X = Pos(Rng), Y = Pos(Rng);
                double Dxx = 0.0, Dyy = 0.0, Dxy = 0.0;
                for (int b = 0; b < 4; b++)
                {
                    auto Add = [&](int Slot, double Weight) {
                        const double L = Slots[Slot].Params.TileLength;
                        const double Uq = X / L, Vq = Y / L;
                        Dxx += Weight * Bilin(G[Slot].Derivatives, Uq, Vq, 0);
                        Dyy += Weight * Bilin(G[Slot].Derivatives, Uq, Vq, 1);
                        Dxy += Weight * Bilin(G[Slot].Displacement, Uq, Vq, 1);
                    };
                    if (b < 2) { for (int K = 0; K < 3; K++) if (W[K] > 0.0) Add(2 * K + b, W[K]); }
                    else Add(4 + b, Scale[b - 2]);
                }
                const float J = float((1.0 + Dxx) * (1.0 + Dyy) - Dxy * Dxy);
                Sum += VoxelShader::VoxelOceanWhitecapOf(J, SlopeVar, 0.0f, float(Slots[0].Chop), float(U), 1.0f);
                if (std::getenv("WHITECAP_QUANTILES")) Cs.push_back((1.0 - J) / (Slots[0].Chop * std::sqrt(SlopeVar)));
            }
            if (std::getenv("WHITECAP_QUANTILES"))
            {
                std::sort(Cs.begin(), Cs.end());
                double M = 0, M2 = 0; for (double c : Cs) { M += c; M2 += c * c; } M /= Cs.size(); M2 = std::sqrt(M2 / Cs.size() - M * M);
                std::printf("    c: mean %.3f sd %.3f;", M, M2);
                for (double Pq : { 0.3, 0.1, 0.03, 0.01, 0.003, 0.001 })
                    std::printf(" P %.3f: %.3f (normal %.3f)", Pq, Cs[size_t((1.0 - Pq) * (Cs.size() - 1))], VoxelShader::VoxelOceanFoamQuantile(float(Pq)));
                std::printf("\n");
                Cs.clear();
            }
            const double Target = VOXEL_OCEAN_WHITECAP_SHARE * VoxelShader::VoxelOceanFoamCoverage(float(U), 1.0f);
            const double Ratio = (Sum / Count) / Target;
            std::printf("  %4.0f m/s: whitecaps %.3f%% of the sea, the target %.3f%% (x %.2f)\n", U, 100.0 * Sum / Count, 100.0 * Target, Ratio);
            WorstRatio = std::max(WorstRatio, std::max(Ratio, 1.0 / std::max(Ratio, 1e-6)));
            if (U == 10.0) Ratio10 = Ratio;
        }
        Check(WorstRatio < 1.2, "the whitecaps' share within 1.2x of the target from 7 to 25 m/s", WorstRatio, 1.2);
        Check(Ratio10 > 0.9 && Ratio10 < 1.1, "at 10 m/s within 10%", Ratio10, 1.1);
        // Unresolved, the expected share: with all the variance left out, the mean of the
        // whitecaps' density at the mean surface is the share.
        const float Far = VoxelShader::VoxelOceanWhitecapOf(1.0f, 1.0f, 1.0f, 0.9f, 10.0f, 1.0f);
        const double FarTarget = VOXEL_OCEAN_WHITECAP_SHARE * VoxelShader::VoxelOceanFoamCoverage(10.0f, 1.0f);
        Check(std::abs(Far / FarTarget - 1.0) < 0.1, "far away (all unresolved) the density is the share", Far / FarTarget, 1.0);
        Check(VoxelShader::VoxelOceanWhitecapOf(0.0f, 1.0f, 0.0f, 0.9f, 2.0f, 1.0f) == 0.0f && VoxelShader::VoxelOceanWhitecapOf(0.0f, 1.0f, 0.0f, 0.9f, 10.0f, 0.0f) == 0.0f,
            "none in a calm or with the coverage at 0", 0, 0);
    }

    std::printf("\n%s: %d failure(s)\n", Fails ? "FAILED" : "PASSED", Fails);
    return Fails ? 1 : 0;
}
