// godot_match.cpp - Our foam against GodotOceanWaves' at its demo's defaults (stage W4a; 2Retr0's main.tscn):
// - Godot: its foamy cascades emulated as it runs them (each a whole spectrum from its tile down to its
//   grid's Nyquist, 1024^2, chop 1, its rule in the cascade's own tile, clamped to 1, the cascades'
//   foam summed and drawn smoothstep(0, 1, 0.75 sum)): cascade 0 (88 m, 10 m/s, fetch 150 km, swell
//   0.8, whitecap 0.5, amount 8) and cascade 2 (16 m, 20 m/s, fetch 550 km, whitecap 0.25, amount 3;
//   cascade 1 has no foam). Our spectrum code at its parameters (the same JONSWAP and Horvath
//   spreading; not its 20 m depth's TMA attenuation, nor its spread 0.2 toward isotropy), at ITS
//   AMPLITUDE: its h0 is gaussian(hash) sqrt(2 S d w_norm), each component of unit variance (its
//   spectrum_compute.glsl), so E|h0|^2 = 4 S dk^2 against ours, S dk^2 / 2 (a mode's wave the
//   variance S dk^2, the spectrum's: spectrum_check), its inverse FFT unnormalised as ours: its waves
//   sqrt(8) = 2.83 times the spectrum's height at the same wind.
// - Ours at the Godot preset (VoxelOceanSettings.h's defaults since W4a: chop 1, fetch 150 km, swell
//   0.8, a flat equalizer, 10 m/s): the summed surface's J at rest points (the clipmap's finest texels,
//   band 3 from the long arrays' copy) and the shortest band's own J (the fine foam, 512^2), stored
//   for every step, so the thresholds that match Godot's coverage are found by a sweep; at the
//   spectrum's height (the equalizer flat at 1) and at Godot's (flat at 2.83).
// Informational, with checks that the matched thresholds are found and in range.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private -I. godot_match.cpp -o godot_match

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

using namespace VoxelOceanSpectrum;
using namespace VoxelShader;
using namespace OceanFFTEmulation;

static int Fails = 0;
static void Check(bool bOk, const char* What, double Value)
{
    std::printf("  [%s] %-84s %.4g\n", bOk ? "ok" : "FAIL", What, Value);
    if (!bOk) Fails++;
}

namespace
{
    constexpr int GodotGrid = 1024;
    constexpr double Hz = 25.0;
    constexpr int Steps = 120;          // 4.8 s
    constexpr int Settle = 60;          // averaged over the last 2.4 s
    const double GodotAmplitude = std::sqrt(8.0);

    // A Godot cascade: every mode from the tile's fundamental to its grid's Nyquist.
    FSlot GodotSlot(double L, double U, double FetchM, double Swell, uint32_t Seed)
    {
        FSlot S;
        S.Band = 0;
        S.Chop = 1.0f;
        const double C = Gravity * 2.0 * Pi / (L * Omega0 * Omega0);
        const uint32_t C2 = PackDispersion(C * C);
        const uint32_t Half = GodotGrid / 2;
        uint32_t MMax = Half * Half;
        const double Top = std::pow(double(VOXEL_OCEAN_MAX_MULTIPLE) + 0.5, 4.0) / UnpackDispersion(C2);
        MMax = uint32_t(std::min(double(MMax), std::floor(Top)));
        S.Params = VoxelOceanMakeSlot(float(L), float(U), float(FetchM), 3.3f, float(Swell), float(GodotAmplitude), C2, 1u, MMax, Seed);
        for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) S.Params.Gain[i] = 1.0f;
        return S;
    }

    float JOf(const F4& D, const F4& X) { return (1.0f + D.z) * (1.0f + D.w) - X.w * X.w; }

    // A cascade's foam in its own tile, Godot's rule, after the run (and its drawn mean alone).
    std::vector<float> RunGodot(const FSlot& S, double Whitecap, double Amount, double& DrawnAlone)
    {
        const double Dt = 1.0 / Hz;
        const float Keep = float(std::exp(-Dt * VoxelOceanFoamDecayRate(float(Amount))));
        const float GrowDt = float(Dt * VoxelOceanFoamGrowRate(float(Amount)));   // Godot's own step
        std::vector<float> F(size_t(GodotGrid) * GodotGrid, 0.0f);
        double Sum = 0.0;
        for (int Step = 0; Step < Steps; Step++)
        {
            const FGPUSlot G = EmulateGPU(S, TickOf(50.0 + Step * Dt), GodotGrid);
            for (size_t i = 0; i < F.size(); i++)
            {
                F[i] = VoxelOceanFoamStep(F[i], VoxelOceanFoamSource(JOf(G.Derivatives[i], G.Displacement[i]), float(Whitecap)), GrowDt, Keep);
                if (Step >= Steps - Settle) Sum += VoxelOceanFoamDrawn(F[i]);
            }
        }
        DrawnAlone = Sum / (double(F.size()) * Settle);
        return F;
    }

    // Our coverage at a threshold from stored J histories (Points x Steps), at a ramp (0: Godot's gain).
    double OurCoverage(const std::vector<float>& J, int Points, double Whitecap, double Ramp, double Amount, std::vector<float>* Last)
    {
        const double Dt = 1.0 / Hz;
        const float Keep = float(std::exp(-Dt * VoxelOceanFoamDecayRate(float(Amount))));
        const double Gain = Ramp > 0.0 ? VOXEL_OCEAN_FOAM_MAX / Ramp : VoxelOceanFoamGrowRate(float(Amount)) / VoxelOceanFoamDecayRate(float(Amount));
        const float GrowDt = float(Gain * (1.0 - Keep));
        std::vector<float> F(Points, 0.0f);
        double Sum = 0.0;
        for (int Step = 0; Step < Steps; Step++)
            for (int n = 0; n < Points; n++)
            {
                F[n] = VoxelOceanFoamStep(F[n], VoxelOceanFoamSource(J[size_t(Step) * Points + n], float(Whitecap)), GrowDt, Keep);
                if (Step >= Steps - Settle) Sum += VoxelOceanFoamDrawn(F[n]);
            }
        if (Last) *Last = F;
        return Sum / (double(Points) * Settle);
    }

    // The threshold whose coverage is Target (coverage rises with the threshold).
    double Match(const std::vector<float>& J, int Points, double Target, double Ramp, double Amount)
    {
        double Lo = 0.0, Hi = 1.5;
        for (int It = 0; It < 22; It++)
        {
            const double Mid = 0.5 * (Lo + Hi);
            (OurCoverage(J, Points, Mid, Ramp, Amount, nullptr) < Target ? Lo : Hi) = Mid;
        }
        return 0.5 * (Lo + Hi);
    }
}

int main()
{
    std::printf("GodotOceanWaves' foam at its demo's defaults (emulated, 1024^2, %.1f s at %.0f Hz):\n", Steps / Hz, Hz);
    const FSlot G0 = GodotSlot(88.0, 10.0, 150.0e3, 0.8, 0x60d0u);
    const FSlot G2 = GodotSlot(16.0, 20.0, 550.0e3, 0.8, 0x60d2u);
    double Alone0 = 0.0, Alone2 = 0.0;
    const std::vector<float> F0 = RunGodot(G0, 0.5, 8.0, Alone0);
    const std::vector<float> F2 = RunGodot(G2, 0.25, 3.0, Alone2);
    // Summed at world points (each cascade's foam bilinear in its own tile), drawn.
    double Both = 0.0;
    {
        std::mt19937 Rng(3);
        std::uniform_real_distribution<double> Pos(0.0, 5000.0);
        const int M = 200000;
        auto At = [](const std::vector<float>& F, double L, double X, double Y)
        {
            const double U = X / L * GodotGrid - 0.5, V = Y / L * GodotGrid - 0.5;
            const double FU = std::floor(U), FV = std::floor(V);
            const double a = U - FU, b = V - FV;
            const int i0 = int(int64_t(FU) & (GodotGrid - 1)), j0 = int(int64_t(FV) & (GodotGrid - 1));
            const int i1 = (i0 + 1) & (GodotGrid - 1), j1 = (j0 + 1) & (GodotGrid - 1);
            auto T = [&](int i, int j) { return double(F[size_t(j) * GodotGrid + i]); };
            return (1 - b) * ((1 - a) * T(i0, j0) + a * T(i1, j0)) + b * ((1 - a) * T(i0, j1) + a * T(i1, j1));
        };
        for (int n = 0; n < M; n++)
        {
            const double X = Pos(Rng), Y = Pos(Rng);
            Both += VoxelOceanFoamDrawn(float(At(F0, 88.0, X, Y) + At(F2, 16.0, X, Y)));
        }
        Both /= M;
    }
    std::printf("  cascade 0 (88 m, 10 m/s, whitecap 0.5, amount 8) alone: %.2f%% drawn\n", 100.0 * Alone0);
    std::printf("  cascade 2 (16 m, 20 m/s, whitecap 0.25, amount 3) alone: %.2f%% drawn\n", 100.0 * Alone2);
    std::printf("  both summed and drawn, as its water.gdshader: %.2f%%\n", 100.0 * Both);

    for (double Gain : { 1.0, GodotAmplitude })
    {
    std::printf("\nOurs at the Godot preset (chop 1, fetch 150 km, swell 0.8, 10 m/s), the equalizer flat at %.2f:\n", Gain);
    FLayoutSettings S;
    S.PlanetSeed = 1234567u;
    for (int b = 0; b < BandCount; b++) S.Chop[b] = 1.0;
    for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) S.Gain[i] = Gain;
    S.Fetch = 150.0e3;
    S.Swell = 0.8;
    S.ShortWind = 10.0;
    const std::vector<FSlot> Slots = MakeSlots(S);
    // The summed surface at rest points (state 1: 10 m/s), band 3 as the clipmap reads it.
    const int Points = 20000;
    std::vector<float> JSea(size_t(Steps) * Points), JFine;
    std::vector<int> FineIdx;
    {
        std::mt19937 Rng(23);
        std::uniform_real_distribution<double> Pos(0.0, 1.0e5);
        std::vector<double> PX(Points), PY(Points);
        for (int n = 0; n < Points; n++) { PX[n] = Pos(Rng); PY[n] = Pos(Rng); }
        const int Use[4] = { 2, 3, 6, 7 };
        const int FineN = SizeOfBand(3);
        std::uniform_int_distribution<int> Tex(0, FineN * FineN - 1);
        for (int k = 0; k < Points; k++) FineIdx.push_back(Tex(Rng));
        JFine.resize(size_t(Steps) * Points);
        for (int Step = 0; Step < Steps; Step++)
        {
            const uint32_t Tick = TickOf(50.0 + Step / Hz);
            FGPUSlot G[4];
            for (int b = 0; b < 4; b++) G[b] = EmulateGPU(Slots[Use[b]], Tick);
            const FGPUSlot Long7 = LongCopy(G[3]);
            for (int n = 0; n < Points; n++)
            {
                double Jxx = 0, Jyy = 0, Jxy = 0;
                for (int b = 0; b < 4; b++)
                {
                    const FGPUSlot& T = b == 3 ? Long7 : G[b];
                    const double L = Slots[Use[b]].Params.TileLength;
                    const F4 D = Bilinear(T.Derivatives, PX[n] / L * T.N, PY[n] / L * T.N), X = Bilinear(T.Displacement, PX[n] / L * T.N, PY[n] / L * T.N);
                    Jxx += D.z; Jyy += D.w; Jxy += X.w;
                }
                JSea[size_t(Step) * Points + n] = float((1 + Jxx) * (1 + Jyy) - Jxy * Jxy);
                const size_t i = size_t(FineIdx[n]);
                JFine[size_t(Step) * Points + n] = JOf(G[3].Derivatives[i], G[3].Displacement[i]);
            }
        }
    }
    const double C085 = OurCoverage(JSea, Points, 0.85, 0.2, 8.0, nullptr);
    std::printf("  the sea's foam at W3z's defaults (whitecap 0.85, ramp 0.2): %.2f%% (Godot's cascade 0: %.2f%%)\n", 100.0 * C085, 100.0 * Alone0);
    const double W0 = Match(JSea, Points, Alone0, 0.0, 8.0);
    const double W2 = Match(JSea, Points, Alone0, 0.2, 8.0);
    std::printf("  the threshold matching Godot's cascade 0: %.3f at Godot's gain (ramp 0), %.3f at ramp 0.2\n", W0, W2);
    const double WFine = Match(JFine, Points, Alone2, 0.0, 3.0);
    std::printf("  the fine foam's threshold matching Godot's cascade 2 (amount 3): %.3f\n", WFine);
    Check(W0 > 0.0 && W0 < 1.45, "the sea's matched threshold is found", W0);
    Check(WFine > 0.0 && WFine < 1.45, "the fine foam's matched threshold is found", WFine);
    }

    std::printf("\n%s (%d failures)\n", Fails == 0 ? "ALL OK" : "FAILED", Fails);
    return Fails ? 1 : 0;
}
