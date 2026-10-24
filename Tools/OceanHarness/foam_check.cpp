// foam_check.cpp - The ocean's foam offline (stage W3v; VoxelOceanFoam.ush: GodotOceanWaves' rule,
// run on the wave clipmap from the summed surface's J, as VoxelOceanClip.usf runs it), on the
// emulated cascades (VoxelOceanFFT.usf's passes in float, ocean_fft_emulation.h) with the runtime's
// layout (the long bands' sea states blended by the wind, the short bands built for it):
// 1. the rule: Godot's rates from the foam amount, the step's exact decay and linear growth, its
//    clamp, the steady state (the fresh foam), the drawn smoothstep, the same result at any frame
//    rate;
// 2. why on the clipmap: no single band's J reaches the threshold (Godot's cascades each hold the
//    whole spectrum from their tile down; ours are bands), the summed surface's does;
// 3. the coverage at rest points (as the clipmap's texels: world space) over 20 s at 15 Hz, from 7
//    to 25 m/s, at the default whitecap threshold and foam amount: rising with the wind, a few
//    percent at 10 m/s; and the trails: foam left where a crest broke fades by the decay;
// 4b. the fine foam (stage W3w): the rule in the shortest band's own tile, its share at the defaults;
// 4. the surf's foam (VoxelOceanSurf.ush; the source fed into the same buffer): the bores' faces
//    and short wake, no step at the trough, the mean where unresolved, the swash's thin edge, off
//    when off.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private -I. foam_check.cpp -o foam_check

#include "VoxelShaderCompat.h"
#include "ocean_fft_emulation.h"
#include "VoxelOceanShoreCore.h"
#include "VoxelOceanCoastCore.h"
#include "VoxelOceanResolvedShare.h"

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
    // The defaults (VoxelOceanSettings.h: FoamWhitecap, FoamAmount).
    constexpr double DefaultWhitecap = 0.85;
    constexpr double DefaultAmount = 8.0;
    // Stage W3y: the ramp (the span of J under the threshold to all of the foam; VoxelOceanSurface.cpp).
    constexpr double DefaultRamp = 0.2;

    // The long bands' sea-state weights (VoxelOceanSeaState.ush's VoxelOceanStateWeights).
    void StateWeights(double U, double W[3])
    {
        if (U <= 5.0) { const double R = U / 5.0; W[0] = R * R; W[1] = W[2] = 0.0; }
        else if (U <= 10.0) { const double T = (U - 5.0) / 5.0; W[0] = 1.0 - T; W[1] = T; W[2] = 0.0; }
        else if (U <= 18.0) { const double T = (U - 10.0) / 8.0; W[0] = 0.0; W[1] = 1.0 - T; W[2] = T; }
        else { const double R = U / 18.0; W[0] = W[1] = 0.0; W[2] = R * R; }
    }

    // A run of the rule at a rate: the source on for On seconds, then off for Off; the foam at the end.
    double RunPulse(double Hz, double On, double Off, double Source, double Amount)
    {
        const double Dt = 1.0 / Hz;
        const float Keep = float(std::exp(-Dt * VoxelShader::VoxelOceanFoamDecayRate(float(Amount))));
        const float GrowDt = float(VoxelShader::VoxelOceanFoamGrowRate(float(Amount)) / VoxelShader::VoxelOceanFoamDecayRate(float(Amount)) * (1.0 - Keep));
        float H = 0.0f;
        for (double T = 0.0; T < On + Off - 0.5 * Dt; T += Dt)
            H = VoxelShader::VoxelOceanFoamStep(H, T < On - 0.5 * Dt ? float(Source) : 0.0f, GrowDt, Keep);
        return H;
    }
}

int main()
{
    using namespace VoxelShader;
    std::printf("1. The rule (GodotOceanWaves' fft_unpack.glsl and wave_generator.gd)\n");
    {
        Check(std::abs(VoxelOceanFoamGrowRate(8.0f) - 60.0f) < 1e-4f && std::abs(VoxelOceanFoamDecayRate(8.0f) - 2.3f) < 1e-5f,
            "amount 8: grows 60 /s, decays 2.3 /s (Godot's 7.5 A and 1.15 max(0.5, 10 - A))", VoxelOceanFoamDecayRate(8.0f), 2.3);
        Check(std::abs(VoxelOceanFoamDecayRate(10.0f) - 0.575f) < 1e-5f && VoxelOceanFoamGrowRate(0.0f) == 0.0f && std::abs(VoxelOceanFoamDecayRate(0.0f) - 11.5f) < 1e-4f,
            "amount 10: the least decay (0.575 /s); amount 0: no growth", VoxelOceanFoamDecayRate(10.0f), 0.575);
        // Decay: 1 s at 60 Hz from 1, no source: e^(-2.3).
        const double Decayed = RunPulse(60.0, 0.0, 1.0, 0.0, 8.0);
        Check(Decayed == 0.0, "(from nothing, nothing)", Decayed, 0.0);
        float H = 1.0f;
        const float Keep = float(std::exp(-VoxelOceanFoamDecayRate(8.0f) / 60.0));
        for (int k = 0; k < 60; k++) H = VoxelOceanFoamStep(H, 0.0f, 0.0f, Keep);
        Check(std::abs(H - std::exp(-2.3)) < 1e-5, "decays exponentially: e^(-2.3) after 1 s at 60 Hz", H, std::exp(-2.3));
        Check(std::abs(VoxelOceanFoamStep(0.2f, 0.1f, 0.5f, 1.0f) - 0.25f) < 1e-6f, "grows linearly with the source (0.2 + 0.1 x 0.5)", VoxelOceanFoamStep(0.2f, 0.1f, 0.5f, 1.0f), 0.25);
        Check(VoxelOceanFoamStep(1.3f, 1.0f, 1.0f, 1.0f) == VOXEL_OCEAN_FOAM_MAX && VoxelOceanFoamStep(0.1f, -5.0f, 1.0f, 1.0f) == 0.1f,
            "held within 0 and 1 (Godot's clamp per cascade; stage W3z); a negative source adds nothing", VoxelOceanFoamStep(1.3f, 1.0f, 1.0f, 1.0f), VOXEL_OCEAN_FOAM_MAX);
        Check(VoxelOceanFoamSource(0.6f, 0.75f) > 0.1499f && VoxelOceanFoamSource(0.6f, 0.75f) < 0.1501f && VoxelOceanFoamSource(0.9f, 0.75f) == 0.0f && VoxelOceanFoamSource(0.3f, 0.0f) == 0.0f,
            "the source: how far J is under the threshold; none over it, none with the threshold 0", VoxelOceanFoamSource(0.6f, 0.75f), 0.15);
        // The steady state: a constant small source for 10 s (amount 2: 15 /s, 9.2 /s) against the fresh foam.
        const double Gain2 = VoxelOceanFoamGrowRate(2.0f) / VoxelOceanFoamDecayRate(2.0f);
        const double Steady = RunPulse(60.0, 10.0, 0.0, 0.3, 2.0);
        const double Fresh = VoxelOceanFoamFresh(0.3f, float(Gain2));
        Check(std::abs(Steady - Fresh) < 1e-3 * Fresh, "a constant source settles at the fresh foam, Source Grow / Decay (the exact step)", std::abs(Steady - Fresh) / Fresh, 1e-3);
        bool bRises = true;
        float Last = -1.0f;
        for (float F = 0.0f; F <= 2.0f * VOXEL_OCEAN_FOAM_MAX + 1e-4f; F += 0.01f) { bRises = bRises && VoxelOceanFoamDrawn(F) >= Last; Last = VoxelOceanFoamDrawn(F); }
        Check(bRises && VoxelOceanFoamDrawn(0.0f) == 0.0f && std::abs(VoxelOceanFoamDrawn(VOXEL_OCEAN_FOAM_MAX) - 0.84375f) < 1e-5f
                && std::abs(VoxelOceanFoamDrawn(2.0f * VOXEL_OCEAN_FOAM_MAX) - 1.0f) < 1e-5f && std::abs(VoxelOceanFoamDrawn(0.5f) - 0.31640625f) < 1e-5f,
            "drawn: smoothstep(0, 1, 0.75 foam), rising: one full layer 0.84 (Godot's), two solid (0.5: 0.316)", VoxelOceanFoamDrawn(VOXEL_OCEAN_FOAM_MAX), 0.84375);
        // The same at any frame rate: a crest compressing for 0.3 s then gone for 0.7 s.
        const double R30 = RunPulse(30.0, 0.3, 0.7, 0.05, 8.0), R60 = RunPulse(60.0, 0.3, 0.7, 0.05, 8.0), R144 = RunPulse(144.0, 0.3, 0.7, 0.05, 8.0);
        const double Spread = std::max({ R30, R60, R144 }) - std::min({ R30, R60, R144 });
        Check(Spread < 0.03 * R60, "the same at 30, 60 and 144 Hz (a crest of 0.3 s, then 0.7 s: within 3%)", Spread / R60, 0.03);
        // Stage W5a: the surf's foam a density with its own decay; the breakup.
        {
            const float KS = float(std::exp(-0.4 / 60.0));
            float F = 0.0f;
            for (int k = 0; k < 60 * 20; k++) F = VoxelOceanFoamStepSurf(F, 0.0f, 0.0f, 0.5f, 0.6f, KS, 1.0f);
            Check(std::abs(F - 0.6f) < 1e-3f, "the surf's foam settles at its density (0.6), whatever the open sea's gain", F, 0.6);
            float G = 0.6f;
            for (int k = 0; k < 60 * 2; k++) G = VoxelOceanFoamStepSurf(G, 0.0f, 0.0f, 0.5f, 0.0f, KS, 1.0f);
            Check(std::abs(G - 0.6f * std::exp(-0.8f)) < 1e-3f, "and lingers by its own decay (2.5 s: e^(-0.8) after 2 s)", G, 0.6f * std::exp(-0.8f));
            Check(VoxelOceanFoamKeepAt(0.9f, 0.95f, 6.0f) == 0.9f && std::abs(VoxelOceanFoamKeepAt(0.9f, 1.2f, 6.0f) - std::pow(0.9f, 2.2f)) < 1e-6f,
                "the breakup: a compressed texel's keep as it was, a stretched one's raised to 1 + 6 (J - 1)", VoxelOceanFoamKeepAt(0.9f, 1.2f, 6.0f), std::pow(0.9, 2.2));
        }
    }

    // The emulated cascades: the runtime's layout at the waves' chop (0.9).
    FLayoutSettings S;
    S.PlanetSeed = 1234567u;
    for (int b = 0; b < BandCount; b++) S.Chop[b] = 0.9;
    S.ShortWind = 10.0;
    const std::vector<FSlot> Slots = MakeSlots(S);

    std::printf("\n2. Why on the clipmap: the bands' own J against the summed surface's (10 m/s, chop 0.9)\n");
    {
        const uint32_t Tick = TickOf(137.0);
        double BandShare = 0.0;
        for (int s = 0; s < SlotCount; s++)
        {
            const FGPUSlot G = EmulateGPU(Slots[s], Tick);
            size_t Under = 0;
            for (size_t i = 0; i < G.Derivatives.size(); i++)
            {
                const double J = (1.0 + G.Derivatives[i].z) * (1.0 + G.Derivatives[i].w) - double(G.Displacement[i].w) * G.Displacement[i].w;
                Under += J < 0.75;
            }
            BandShare = std::max(BandShare, double(Under) / G.Derivatives.size());
        }
        // (At 0.75: where the summed surface foams a few percent of the sea; foam_check section 3's
        // summed share under it is several times this. Stage W3z: the shortest band, down to 10.7 cm,
        // dips under it on its own ripples' crests, about 1.2% of its texels: the fine foam's, not
        // the sea's crests.)
        std::printf("  the largest share of any one slot's texels under J 0.75: %.3f%%\n", 100.0 * BandShare);
        Check(BandShare < 0.02, "under 2% of any band's texels: a band's own tile can't hold the sea's foam", BandShare, 0.02);
    }

    std::printf("\n3. The coverage at rest points (world space, as the clipmap's texels), 15 Hz for 20 s, whitecap %.2f, amount %.0f, ramp %.2f\n",
        DefaultWhitecap, DefaultAmount, DefaultRamp);
    {
        const double Winds[5] = { 7.0, 10.0, 14.0, 18.0, 25.0 };
        const int M = 20000, Steps = 300;
        const double Dt = 1.0 / 15.0;
        std::vector<std::vector<FSlot>> Short(5);
        for (int w = 0; w < 5; w++) { FLayoutSettings A = S; A.ShortWind = Winds[w]; Short[w] = MakeSlots(A); }
        std::mt19937 Rng(23);
        std::uniform_real_distribution<double> Pos(0.0, 1.0e5);
        std::vector<double> PX(M), PY(M);
        for (int n = 0; n < M; n++) { PX[n] = Pos(Rng); PY[n] = Pos(Rng); }
        const float Keep = float(std::exp(-Dt * VoxelOceanFoamDecayRate(float(DefaultAmount))));
        // (With the ramp: the steady state all of the foam at DefaultRamp under the threshold.)
        const float GrowDt = float(VOXEL_OCEAN_FOAM_MAX / DefaultRamp * (1.0 - Keep));
        double Partial = 0.0, Drawn = 0.0;
        std::vector<float> H(size_t(5) * M, 0.0f);
        double SummedMin10 = 10.0;
        // The trails: at 10 m/s, the points breaking at 10 s, their foam 1 s later against e^(-decay).
        std::vector<float> At10(M, 0.0f);
        std::vector<char> Breaking(M, 0);
        double TrailSum = 0.0, TrailFresh = 0.0;
        int TrailCount = 0;
        double Coverage[5] = {};
        for (int Step = 0; Step < Steps; Step++)
        {
            const uint32_t Tick = TickOf(100.0 + Step * Dt);
            FGPUSlot GL[6];
            for (int s = 0; s < 6; s++) GL[s] = EmulateGPU(Slots[s], Tick);
            for (int w = 0; w < 5; w++)
            {
                // (Band 3 as the clipmap reads it: the long arrays' copy, stage W3z.)
                const FGPUSlot G6 = EmulateGPU(Short[w][6], Tick), G7 = LongCopy(EmulateGPU(Short[w][7], Tick));
                double Wt[3];
                StateWeights(Winds[w], Wt);
                for (int n = 0; n < M; n++)
                {
                    double Jxx = 0.0, Jyy = 0.0, Jxy = 0.0;
                    auto Add = [&](const FGPUSlot& G, double L, double Weight) {
                        const F4 D = Bilinear(G.Derivatives, PX[n] / L * G.N, PY[n] / L * G.N), X = Bilinear(G.Displacement, PX[n] / L * G.N, PY[n] / L * G.N);
                        Jxx += Weight * D.z; Jyy += Weight * D.w; Jxy += Weight * X.w;
                    };
                    for (int K = 0; K < 3; K++)
                        if (Wt[K] > 0.0) { Add(GL[2 * K], Slots[2 * K].Params.TileLength, Wt[K]); Add(GL[2 * K + 1], Slots[2 * K + 1].Params.TileLength, Wt[K]); }
                    Add(G6, Short[w][6].Params.TileLength, 1.0);
                    Add(G7, Short[w][7].Params.TileLength, 1.0);
                    const float J = float((1.0 + Jxx) * (1.0 + Jyy) - Jxy * Jxy);
                    if (w == 1) SummedMin10 = std::min(SummedMin10, double(J));
                    const float Source = VoxelOceanFoamSource(J, float(DefaultWhitecap));
                    float& Hn = H[size_t(w) * M + n];
                    Hn = VoxelOceanFoamStep(Hn, Source, GrowDt, Keep);
                    if (Step >= Steps / 2)
                    {
                        const float Dn = VoxelOceanFoamDrawn(Hn);
                        Coverage[w] += Dn;
                        if (w == 1 && Dn > 0.02f) { Drawn += 1.0; Partial += Dn < 0.8f ? 1.0 : 0.0; }
                    }
                    if (w == 1 && Step == 150) { Breaking[n] = Source > 0.02f && Hn > 0.5f; At10[n] = Hn; }
                    if (w == 1 && Step == 165 && Breaking[n])
                    {
                        // (Only the points that didn't break again in between: their foam at most the decayed.)
                        TrailSum += Hn; TrailFresh += At10[n]; TrailCount++;
                    }
                }
            }
        }
        for (int w = 0; w < 5; w++) Coverage[w] /= double(M) * (Steps - Steps / 2);
        std::printf("  the summed surface's least J at 10 m/s: %.3f\n", SummedMin10);
        Check(SummedMin10 < DefaultWhitecap - 0.2, "the summed surface's J does fall well under the threshold", SummedMin10, DefaultWhitecap - 0.2);
        std::printf("  the drawn foam's share of the sea:");
        for (int w = 0; w < 5; w++) std::printf("  %.0f m/s %.2f%%", Winds[w], 100.0 * Coverage[w]);
        std::printf("\n");
        bool bRising = true;
        for (int w = 1; w < 5; w++) bRising = bRising && Coverage[w] >= Coverage[w - 1];
        Check(bRising, "it rises with the wind", Coverage[4] / std::max(Coverage[0], 1e-9), 1.0);
        Check(Coverage[1] > 0.005 && Coverage[1] < 0.08, "a few percent at 10 m/s (0.5% to 8%)", Coverage[1], 0.08);
        Check(Coverage[4] > 2.0 * Coverage[1], "a storm (25 m/s) at least twice that", Coverage[4] / Coverage[1], 2.0);
        // Stage W3y: the ramp: most of the foam's area translucent (drawn under 0.8) at 10 m/s.
        std::printf("  at 10 m/s, of the sea foamed at all (drawn over 0.02), %.0f%% partly (under 0.8)\n", 100.0 * Partial / std::max(Drawn, 1.0));
        Check(Partial / std::max(Drawn, 1.0) > 0.9, "the ramp: the foam comes in gradually (most of its area translucent)", Partial / std::max(Drawn, 1.0), 0.9);
        // The trails: 1 s after breaking, at most the decayed foam on average (some break again), and
        // still there (it doesn't vanish at once).
        const double Ratio = TrailCount > 0 ? TrailSum / TrailFresh : 0.0;
        const double Decayed = std::exp(-1.0 * VoxelOceanFoamDecayRate(float(DefaultAmount)));
        std::printf("  breaking points 1 s on (%d of them): foam %.3f of what it was (the decay alone %.3f)\n", TrailCount, Ratio, Decayed);
        Check(TrailCount > 20 && Ratio > 0.5 * Decayed && Ratio < 0.6, "the foam a crest leaves fades as a trail", Ratio, 0.6);
    }

    std::printf("\n3c. A coarse texel's source (stage W3z; VoxelOceanFoamSourceSpread): the mean of the fine surface's over the\n"
                "    texel, against the hard rule on the texel's own J (whitecap %.2f, chop 0.9; truth: 5 cm samples, band 3 on 512^2)\n", DefaultWhitecap);
    {
        bool bOk = true;
        double WorstHard = 0.0;
        for (double U : { 10.0, 18.0 })
        {
            FLayoutSettings A = S;
            A.ShortWind = U;
            const std::vector<FSlot> Sl = MakeSlots(A);
            double Wt[3];
            StateWeights(U, Wt);
            const int K = U <= 10.0 ? 1 : 2;   // (one state at these winds)
            const int Use[4] = { 2 * K, 2 * K + 1, 6, 7 };
            const uint32_t Tick = TickOf(141.0);
            FGPUSlot G[4];
            double Var[4], MinL[4];
            FResolvedShare Share[4];
            for (int b = 0; b < 4; b++)
            {
                G[b] = EmulateGPU(Sl[Use[b]], Tick);
                Share[b] = ResolvedShareOf(Sl[Use[b]]);
                const FBandStats St = StatsOf(Sl[Use[b]]);
                Var[b] = (St.SlopeVarianceX + St.SlopeVarianceY) * (b < 2 ? Wt[K] * Wt[K] : 1.0);
                MinL[b] = b < 3 ? 0.25 * Sl[Use[b + 1]].Params.TileLength : Sl[Use[3]].Params.TileLength * 4.0 / SizeOfBand(3);
            }
            auto JAt = [&](double X, double Y, double& Jxx, double& Jyy, double& Jxy)
            {
                Jxx = Jyy = Jxy = 0.0;
                for (int b = 0; b < 4; b++)
                {
                    const double L = Sl[Use[b]].Params.TileLength, Wb = b < 2 ? Wt[K] : 1.0;
                    const F4 D = Bilinear(G[b].Derivatives, X / L * G[b].N, Y / L * G[b].N), Xd = Bilinear(G[b].Displacement, X / L * G[b].N, Y / L * G[b].N);
                    Jxx += Wb * D.z; Jyy += Wb * D.w; Jxy += Wb * Xd.w;
                }
            };
            std::printf("  %2.0f m/s:", U);
            for (double F : { 0.4, 0.8, 1.6, 3.2 })
            {
                const int M = std::max(8, int(std::lround(F / 0.05)));
                // (Stage W6b: the exact unresolved share at the band's lookups for the texel, as the
                // shading's table, VoxelOceanSlotUnresolved; until then its shortest waves' alone.)
                double Unresolved = 0.0;
                for (int b = 0; b < 4; b++)
                {
                    const double Lod = std::log2(std::max(F * SizeOfBand(b) / Sl[Use[b]].Params.TileLength, 1.0));
                    Unresolved += Var[b] * (1.0 - 0.5 * (ResolvedShareAt(Share[b], Lod, 0) + ResolvedShareAt(Share[b], Lod, 1)));
                    (void)MinL;
                }
                const double SigmaStats = VOXEL_OCEAN_FOAM_SPREAD_SCALE * 0.9 * std::sqrt(Unresolved);
                std::mt19937 Rng(91);
                std::uniform_real_distribution<double> Pos(0.0, 2.0e4);
                double Truth = 0.0, Hard = 0.0, Spread = 0.0, SigmaSample = 0.0, SpreadCell = 0.0;
                const int Cells = 300;
                for (int c = 0; c < Cells; c++)
                {
                    const double X0 = Pos(Rng), Y0 = Pos(Rng);
                    double Sxx = 0, Syy = 0, Sxy = 0, T = 0, J1 = 0, J2 = 0;
                    for (int j = 0; j < M; j++)
                        for (int i = 0; i < M; i++)
                        {
                            double Jxx, Jyy, Jxy;
                            JAt(X0 + (i + 0.5) * F / M, Y0 + (j + 0.5) * F / M, Jxx, Jyy, Jxy);
                            const double J = (1 + Jxx) * (1 + Jyy) - Jxy * Jxy;
                            T += std::max(DefaultWhitecap - J, 0.0);
                            J1 += J; J2 += J * J;
                            Sxx += Jxx; Syy += Jyy; Sxy += Jxy;
                        }
                    const double Inv = 1.0 / (double(M) * M);
                    Sxx *= Inv; Syy *= Inv; Sxy *= Inv;
                    // (The texel's J: its derivatives' means, as its mips hold them.)
                    const float JTexel = float((1 + Sxx) * (1 + Syy) - Sxy * Sxy);
                    Truth += T * Inv;
                    Hard += VoxelOceanFoamSource(JTexel, float(DefaultWhitecap));
                    Spread += VoxelOceanFoamSourceSpread(JTexel, float(SigmaStats), float(DefaultWhitecap));
                    const double SigmaCell = std::sqrt(std::max(J2 * Inv - (J1 * Inv) * (J1 * Inv), 0.0));
                    SigmaSample += SigmaCell;
                    SpreadCell += VoxelOceanFoamSourceSpread(JTexel, float(SigmaCell), float(DefaultWhitecap));
                }
                Truth /= Cells; Hard /= Cells; Spread /= Cells; SigmaSample /= Cells; SpreadCell /= Cells;
                std::printf("  %.1f m: truth %.4f, hard %.4f, mean %.4f (sigma %.3f, in the cells %.3f: %.4f) |", F, Truth, Hard, Spread, SigmaStats, SigmaSample, SpreadCell);
                bOk = bOk && std::abs(Spread - Truth) < 0.25 * Truth && std::abs(SpreadCell - Truth) < 0.08 * Truth;
                WorstHard = std::max(WorstHard, std::abs(Hard - Truth) / std::max(Truth, 1e-9));
            }
            std::printf("\n");
        }
        Check(bOk, "the Gaussian mean holds the fine surface's: within 8% with each cell's own deviation, 25% from the bands' statistics", 0.0, 0.25);
        std::printf("  (the hard rule on a texel's own J errs up to %.0f%%)\n", 100.0 * WorstHard);
    }

    std::printf("\n3d. A coarse texel's held foam (stage W5h; VoxelOceanFoamSourceHeld): the mean of the fine surface's\n"
                "    steady foam min(Gain s, 1) over the texel, at the Godot preset (height x2.83, whitecap 0.5, chop 1, gain of\n"
                "    amount 8 without a ramp), against min(Gain E[s], 1) (W3z to W5g: the held mean)\n");
    {
        const double H = 2.83, C = 0.5;
        const double Gain = VoxelOceanFoamGrowRate(8.0f) / VoxelOceanFoamDecayRate(8.0f);
        double WorstOld = 0.0, WorstNew = 0.0, WorstNewCoarse = 0.0, WorstFar = 0.0;
        for (double U : { 10.0, 18.0 })
        {
            FLayoutSettings A = S;
            A.ShortWind = U;
            const std::vector<FSlot> Sl = MakeSlots(A);
            double Wt[3];
            StateWeights(U, Wt);
            const int K = U <= 10.0 ? 1 : 2;
            const int Use[4] = { 2 * K, 2 * K + 1, 6, 7 };
            const uint32_t Tick = TickOf(141.0);
            FGPUSlot G[4];
            double Var[4], MinL[4];
            FResolvedShare Share[4];
            for (int b = 0; b < 4; b++)
            {
                G[b] = EmulateGPU(Sl[Use[b]], Tick);
                Share[b] = ResolvedShareOf(Sl[Use[b]]);
                const FBandStats St = StatsOf(Sl[Use[b]]);
                Var[b] = (St.SlopeVarianceX + St.SlopeVarianceY) * (b < 2 ? Wt[K] * Wt[K] : 1.0) * H * H;
                MinL[b] = b < 3 ? 0.25 * Sl[Use[b + 1]].Params.TileLength : Sl[Use[3]].Params.TileLength * 4.0 / SizeOfBand(3);
            }
            auto JAt = [&](double X, double Y, double& Jxx, double& Jyy, double& Jxy)
            {
                Jxx = Jyy = Jxy = 0.0;
                for (int b = 0; b < 4; b++)
                {
                    const double L = Sl[Use[b]].Params.TileLength, Wb = (b < 2 ? Wt[K] : 1.0) * H;
                    const F4 D = Bilinear(G[b].Derivatives, X / L * G[b].N, Y / L * G[b].N), Xd = Bilinear(G[b].Displacement, X / L * G[b].N, Y / L * G[b].N);
                    Jxx += Wb * D.z; Jyy += Wb * D.w; Jxy += Wb * Xd.w;
                }
            };
            std::printf("  %2.0f m/s:", U);
            double LastTruth = 0.0;
            for (double F : { 0.4, 1.6, 6.4 })
            {
                const int M = std::min(128, std::max(8, int(std::lround(F / 0.05))));
                // (Stage W6b: the exact unresolved share at the band's lookups for the texel, as the
                // shading's table, VoxelOceanSlotUnresolved; until then its shortest waves' alone.)
                double Unresolved = 0.0;
                for (int b = 0; b < 4; b++)
                {
                    const double Lod = std::log2(std::max(F * SizeOfBand(b) / Sl[Use[b]].Params.TileLength, 1.0));
                    Unresolved += Var[b] * (1.0 - 0.5 * (ResolvedShareAt(Share[b], Lod, 0) + ResolvedShareAt(Share[b], Lod, 1)));
                    (void)MinL;
                }
                const float Sigma = float(VOXEL_OCEAN_FOAM_SPREAD_SCALE * std::sqrt(Unresolved));
                std::mt19937 Rng(17);
                std::uniform_real_distribution<double> Pos(0.0, 2.0e4);
                double Truth = 0.0, Old = 0.0, New = 0.0;
                const int Cells = F > 3.0 ? 60 : 200;
                for (int c = 0; c < Cells; c++)
                {
                    const double X0 = Pos(Rng), Y0 = Pos(Rng);
                    double Sxx = 0, Syy = 0, Sxy = 0, T = 0;
                    for (int j = 0; j < M; j++)
                        for (int i = 0; i < M; i++)
                        {
                            double Jxx, Jyy, Jxy;
                            JAt(X0 + (i + 0.5) * F / M, Y0 + (j + 0.5) * F / M, Jxx, Jyy, Jxy);
                            const double J = (1 + Jxx) * (1 + Jyy) - Jxy * Jxy;
                            T += std::min(Gain * std::max(C - J, 0.0), 1.0);
                            Sxx += Jxx; Syy += Jyy; Sxy += Jxy;
                        }
                    const double Inv = 1.0 / (double(M) * M);
                    Sxx *= Inv; Syy *= Inv; Sxy *= Inv;
                    const float JTexel = float((1 + Sxx) * (1 + Syy) - Sxy * Sxy);
                    Truth += T * Inv;
                    Old += std::min(Gain * double(VoxelOceanFoamSourceSpread(JTexel, Sigma, float(C))), 1.0);
                    New += std::min(Gain * double(VoxelOceanFoamSourceHeld(JTexel, Sigma, float(C), float(Gain))), 1.0);
                }
                Truth /= Cells; Old /= Cells; New /= Cells;
                LastTruth = Truth;
                std::printf("  %.1f m: truth %.3f, held mean %.3f, mean held %.3f (sigma x gain %.1f) |", F, Truth, Old, New, Sigma * Gain);
                WorstOld = std::max(WorstOld, std::abs(Old - Truth) / std::max(Truth, 1e-9));
                WorstNew = std::max(WorstNew, std::abs(New - Truth) / std::max(Truth, 1e-9));
                if (F > 1.0) WorstNewCoarse = std::max(WorstNewCoarse, std::abs(New - Truth) / std::max(Truth, 1e-9));
            }
            // Stage W6e: past the clipmap (VoxelOceanFFTFoamFar): the held mean at J's mean 1 with all of
            // the slopes unresolved, against the fine surface's mean at the coarsest texels here.
            {
                double VarAll = 0.0;
                for (int b = 0; b < 4; b++) VarAll += Var[b];
                const float SigmaAll = float(VOXEL_OCEAN_FOAM_SPREAD_SCALE * std::sqrt(VarAll));
                const double Far = std::min(Gain * double(VoxelOceanFoamSourceHeld(1.0f, SigmaAll, float(C), float(Gain))), 1.0);
                std::printf("  past the clipmap (W6e): %.3f against the 6.4 m texels' truth %.3f |", Far, LastTruth);
                WorstFar = std::max(WorstFar, std::abs(Far - LastTruth) / std::max(LastTruth, 1e-9));
            }
            std::printf("\n");
        }
        std::printf("  worst: the held mean %.0f%% off, the mean held %.0f%% (%.0f%% at 1.6 m and coarser)\n", 100.0 * WorstOld, 100.0 * WorstNew, 100.0 * WorstNewCoarse);
        Check(WorstFar < 0.15, "past the clipmap (W6e): the sea state's held mean within 15% of the fine surface's mean foam", WorstFar, 0.15);
        Check(WorstNewCoarse < 0.3, "a coarse texel's foam holds the fine surface's mean of the held foam (30%)", WorstNewCoarse, 0.3);
        Check(WorstNew < WorstOld, "nearer than the held mean (W3z to W5g)", WorstNew, WorstOld);
    }

    std::printf("\n4b. The fine foam (stage W3w; VoxelOceanFFT.usf): Godot's rule in the shortest band's own tile, its J at the\n"
                "    waves' chop, at the defaults (whitecap 0.9, amount 5), 10 s at 50 Hz (the cascades' rate)\n");
    {
        const double FineWhitecap = 0.9, FineAmount = 5.0, Dt = 1.0 / 50.0;
        const int Steps = 500, Slot = SlotCount - 1;
        const float Keep = float(std::exp(-Dt * VoxelOceanFoamDecayRate(float(FineAmount))));
        const float GrowDt = float(VoxelOceanFoamGrowRate(float(FineAmount)) / VoxelOceanFoamDecayRate(float(FineAmount)) * (1.0 - Keep));
        const int N = SizeOfBand(Slots[Slot].Band);   // stage W3z: its own grid
        std::vector<float> F(size_t(N) * N, 0.0f);
        double Cover = 0.0, Mean = 0.0;
        for (int Step = 0; Step < Steps; Step++)
        {
            const FGPUSlot G = EmulateGPU(Slots[Slot], TickOf(100.0 + Step * Dt));
            for (size_t i = 0; i < F.size(); i++)
            {
                const float J = (1.0f + G.Derivatives[i].z) * (1.0f + G.Derivatives[i].w) - G.Displacement[i].w * G.Displacement[i].w;
                F[i] = VoxelOceanFoamStep(F[i], VoxelOceanFoamSource(J, float(FineWhitecap)), GrowDt, Keep);
                if (Step >= Steps / 2) { Cover += VoxelOceanFoamDrawn(F[i]); Mean += F[i]; }
            }
        }
        Cover /= double(F.size()) * (Steps - Steps / 2);
        Mean /= double(F.size()) * (Steps - Steps / 2);
        std::printf("  the fine foam's drawn share alone: %.2f%% (its mean %.4f; a texel %.1f cm)\n", 100.0 * Cover, Mean, 100.0 * Slots[Slot].Params.TileLength / N);
        Check(Cover > 0.003 && Cover < 0.06, "specks on the short waves' crests: 0.3% to 6% of the sea at the reference wind", Cover, 0.06);
    }

    std::printf("\n4. The surf's foam (VoxelOceanSurf.ush; a 1:50 beach, 1.5 m waves at 12 m/s)\n");
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
        double BoreMax = 0.0, BoreMean = 0.0, OffshoreMax = 0.0, OffMax = 0.0, EdgeMax = 0.0, SheetMin = 1.0, Jump = 0.0, WetMin = 1.0, WetSum = 0.0;
        int WetCount = 0;
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
            if (Thickness > 0.02f) { WetMin = std::min(WetMin, double(Sw.Foam)); WetSum += Sw.Foam; WetCount++; }
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
        // (Stage W3p: short trails, no carpet between the bores: their mean 0.1 to 0.3; 0.25 to 0.8
        // until then.)
        Check(BoreMax > 0.9 && BoreMean > 0.1 && BoreMean < 0.3, "the breaking bores are white on their faces with short trails behind", BoreMean, 0.1);
        Check(OffshoreMax < 0.25, "unbroken waves past the surf zone carry little", OffshoreMax, 0.25);
        Check(Jump < 0.01, "no step through the cycle (the trail meets the face at the trough; it stepped 0.017)", Jump, 0.01);
        std::printf("  seen with a 200 m footprint: %.2f to %.2f (mean %.2f; near %.2f)\n", FarMin, FarMax, FarMean, BoreMean);
        Check(FarMax - FarMin < 0.25 * BoreMax && std::abs(FarMean - BoreMean) < 0.15, "unresolved, the bores' foam is their cycle's mean (no stripes)", FarMax - FarMin, 0.25 * BoreMax);
        std::printf("  the swash 10 cm up the beach: at its thin edge up to %.2f; under 30 cm of it at least %.2f\n", EdgeMax, SheetMin);
        Check(EdgeMax > 0.8, "the swash's thin edge is foamy", EdgeMax, 0.8);
        // (Stage W3p: only the thin edge; until then a fifth over the whole sheet, a mat on the beach.
        // Stage W5f: the sheet white water as it rushes up, draining away after the swash's peak: no
        // standing mat still, its foam comes and goes through the cycle where it covers the beach.)
        const double WetMean = WetSum / std::max(WetCount, 1);
        std::printf("  while the sheet covers it: foam %.2f least, %.2f mean (under 30 cm at least %.2f)\n", WetMin, WetMean, SheetMin);
        Check(WetMin < 0.1, "the swash sheet drains clear (no standing mat)", WetMin, 0.1);
        Check(WetMean > 0.2 && WetMean < 0.8, "the swash sheet is white water for part of its cycle", WetMean, 0.2);
        Check(OffMax == 0.0, "no surf foam with the setting at 0", OffMax, 0.0);
    }


    std::printf("\n%s: %d failure(s)\n", Fails ? "FAILED" : "PASSED", Fails);
    return Fails ? 1 : 0;
}
