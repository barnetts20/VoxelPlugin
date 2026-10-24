// preset_check.cpp - The realistic preset's foam (stage W6g; Design/DistantOcean.md, W6g): the
// open sea's whitecap coverage at physical heights (the equalizer at 1) against Monahan and
// O'Muircheartaigh's fit, W = 3.84e-6 U^3.41 (about 1% at 10 m/s, 7% at 20), for the foam rule's
// threshold (FoamWhitecap) at Godot's amount (8: gain 60 / 2.3) without a ramp, chop 1: the mean of
// the drawn steady foam, VoxelOceanFoamDrawn(min(Gain max(T - J, 0), 1)), over the summed surface's
// points (emulated cascades, the runtime's layout). Picks the threshold for 1% at 10 m/s (the
// preset's FoamWhitecap) and reports the winds' coverage with it; checks the preset's numbers
// (VoxelOceanSettings.h, FVoxelOceanSettings::ApplyRealisticPreset) against it.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private -I. preset_check.cpp -o preset_check

#include "VoxelShaderCompat.h"
#include "ocean_fft_emulation.h"

namespace VoxelShader
{
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

// The preset's threshold at 10 m/s and its rise per m/s of wind (VoxelOceanSettings.h:
// FVoxelOceanSettings::ApplyRealisticPreset's FoamWhitecap and FoamWhitecapWind).
constexpr double PresetWhitecap = 0.66;
constexpr double PresetWhitecapWind = 0.0115;

int main()
{
    using namespace VoxelShader;
    const double Gain = VoxelOceanFoamGrowRate(8.0f) / VoxelOceanFoamDecayRate(8.0f);
    const double Winds[4] = { 7.0, 10.0, 14.0, 18.0 };
    std::vector<std::vector<double>> Js(4);
    for (int w = 0; w < 4; w++)
    {
        FLayoutSettings S;
        S.PlanetSeed = 1234567u;
        S.ShortWind = Winds[w];
        for (int b = 0; b < BandCount; b++) S.Chop[b] = 1.0;
        const std::vector<FSlot> Sl = MakeSlots(S);
        // The long bands' state weights (VoxelOceanStateWeights).
        double Wt[3] = { 0, 0, 0 };
        const double U = Winds[w];
        if (U <= 10.0) { const double T = (U - 5.0) / 5.0; Wt[0] = 1.0 - T; Wt[1] = T; }
        else { const double T = (U - 10.0) / 8.0; Wt[1] = 1.0 - T; Wt[2] = T; }
        std::vector<FGPUSlot> G(SlotCount);
        double Ls[SlotCount];
        for (int s = 0; s < SlotCount; s++) { G[s] = EmulateGPU(Sl[s], TickOf(171.0)); Ls[s] = Sl[s].Params.TileLength; }
        std::mt19937 Rng(29);
        std::uniform_real_distribution<double> Pos(0.0, 3.0e4);
        const int M = 200000;
        Js[w].resize(M);
        for (int n = 0; n < M; n++)
        {
            const double X = Pos(Rng), Y = Pos(Rng);
            double Jxx = 0, Jyy = 0, Jxy = 0;
            auto Add = [&](int s, double Weight) {
                if (Weight <= 0.0) return;
                const F4 D = Bilinear(G[s].Derivatives, X / Ls[s] * G[s].N, Y / Ls[s] * G[s].N), Xd = Bilinear(G[s].Displacement, X / Ls[s] * G[s].N, Y / Ls[s] * G[s].N);
                Jxx += Weight * D.z; Jyy += Weight * D.w; Jxy += Weight * Xd.w;
            };
            for (int k = 0; k < 3; k++) { Add(2 * k, Wt[k]); Add(2 * k + 1, Wt[k]); }
            Add(6, 1.0); Add(7, 1.0);
            Js[w][n] = (1 + Jxx) * (1 + Jyy) - Jxy * Jxy;
        }
    }
    auto Coverage = [&](int w, double T) {
        double Sum = 0.0;
        for (double J : Js[w]) Sum += VoxelOceanFoamDrawn(float(std::min(Gain * std::max(T - J, 0.0), double(VOXEL_OCEAN_FOAM_MAX))));
        return Sum / double(Js[w].size());
    };
    auto Monahan = [](double U) { return 3.84e-6 * std::pow(U, 3.41); };
    std::printf("Whitecap coverage at physical heights (chop 1, amount 8, no ramp) against Monahan's fit:\n");
    std::printf("  %9s", "threshold");
    for (double U : Winds) std::printf("  %5.0f m/s", U);
    std::printf("\n  %9s", "Monahan");
    for (double U : Winds) std::printf("  %8.2f%%", 100.0 * Monahan(U));
    std::printf("\n");
    for (double T = 0.3; T <= 0.951; T += 0.05)
    {
        std::printf("  %9.2f", T);
        for (int w = 0; w < 4; w++) std::printf("  %8.2f%%", 100.0 * Coverage(w, T));
        std::printf("\n");
    }
    // The threshold for 1% at 10 m/s (bisection).
    double Lo = 0.0, Hi = 1.2;
    for (int i = 0; i < 40; i++) { const double Mid = 0.5 * (Lo + Hi); (Coverage(1, Mid) < Monahan(10.0) ? Lo : Hi) = Mid; }
    std::printf("  for Monahan's %.2f%% at 10 m/s: threshold %.3f (coverage then %.2f%% at 7, %.2f%% at 14, %.2f%% at 18 m/s; Monahan %.2f, %.2f, %.2f)\n",
        100.0 * Monahan(10.0), Lo, 100.0 * Coverage(0, Lo), 100.0 * Coverage(2, Lo), 100.0 * Coverage(3, Lo), 100.0 * Monahan(7.0), 100.0 * Monahan(14.0), 100.0 * Monahan(18.0));
    // Stage W6g: the threshold by the local wind, T + k (U - 10) (VoxelOceanFFTWhitecapAt): the best
    // k for Monahan from 7 to 18 m/s (in log coverage), and the preset's.
    double BestK = 0.0, BestErr = 1.0e30;
    for (double K = 0.0; K <= 0.03; K += 0.0005)
    {
        double Err = 0.0;
        for (int w = 0; w < 4; w++)
        {
            const double C = std::max(Coverage(w, Lo + K * (Winds[w] - 10.0)), 1.0e-6);
            Err += std::pow(std::log(C / Monahan(Winds[w])), 2.0);
        }
        if (Err < BestErr) { BestErr = Err; BestK = K; }
    }
    std::printf("  with the threshold %.3f + k (U - 10): the best k %.4f\n", Lo, BestK);
    std::printf("  the preset (%.3f + %.4f (U - 10)):", PresetWhitecap, PresetWhitecapWind);
    double WorstRatio = 1.0;
    for (int w = 0; w < 4; w++)
    {
        const double C = Coverage(w, PresetWhitecap + PresetWhitecapWind * (Winds[w] - 10.0));
        const double Ratio = C / Monahan(Winds[w]);
        WorstRatio = std::max(WorstRatio, std::max(Ratio, 1.0 / std::max(Ratio, 1e-9)));
        std::printf("  %.0f m/s %.2f%% (Monahan %.2f%%)", Winds[w], 100.0 * C, 100.0 * Monahan(Winds[w]));
    }
    std::printf("\n");
    Check(WorstRatio < 1.6, "the realistic preset's coverage within a factor 1.6 of Monahan's from 7 to 18 m/s", WorstRatio, 1.6);
    std::printf("\n%s (%d failures)\n", Fails == 0 ? "ALL OK" : "FAILED", Fails);
    return Fails == 0 ? 0 : 1;
}
