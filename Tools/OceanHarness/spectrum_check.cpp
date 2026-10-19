// O6a: the FFT cascades' shared code (VoxelOceanSpectrum.ush) and the CPU side
// (VoxelOceanSpectrum.h), checked without the engine.
// - Spreading: the directional spreading integrates to 1 (Horvath's normalisation fits).
// - Spectrum: significant wave height per wind against the Pierson-Moskowitz rule, and how much
//   of the sea's variance the four bands hold, band by band.
// - Band split: every band's edges meet the next one's; every mode a slot keeps lies in its band.
// - Dispersion: the shared multiple n (integer arithmetic only) against a 128-bit reference for
//   every mode of every slot, and 400k stress cases including near-ties built on purpose.
// - The GPU passes, emulated in float with the shader's own loop (init, time step, FFT rows and
//   columns, assemble): all eight fields against a direct DFT in double. This checks the
//   packing, the signs, the Hermitian symmetry and the butterflies.
// - Physical directions: crests travel downwind; the chop compresses the surface at crests.
// - Variance: each grid's variance is its modes' sum (Parseval); averaged over 100 seeds, the
//   start spectrum's variance is the spectrum's.
// - Time: the loop period wraps exactly; the field is continuous across the wrap.
// - The query cascades (CPU, 64^2 / 128^2; bands 0-2): against the direct DFT at random points
//   (height, displacement, all gradients, all velocities), and against the GPU's bilinear lookup.
// - fp16: the output textures' rounding.
// - Folding: the share of the sea that folds (Jacobian < 0) per wind, all bands together.
// - Detiling: the variance kept, the correlation at the longest tile's period cut, continuity.
// - Determinism, and the CPU's update cost per slot.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private -I. spectrum_check.cpp -o spectrum_check
#include "VoxelOceanSpectrum.h"
#include "ocean_fft_emulation.h"

#include <algorithm>
#include <chrono>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>

using namespace VoxelOceanSpectrum;
using namespace OceanFFTEmulation;
using VoxelShader::float2;
typedef std::complex<double> cd;

static int Failures = 0;
static void Expect(bool Ok, const char* What)
{
    std::printf("  [%s] %s\n", Ok ? "ok" : "FAIL", What);
    if (!Ok) Failures++;
}

// ---------------------------------------------------------------------------------------------
// The direct DFT in double
// ---------------------------------------------------------------------------------------------

struct FTruth
{
    double Eta, Dx, Dy, EtaX, EtaY, Dxx, Dyy, Dxy, EtaT, DxT, DyT;
};

struct FTruthModes
{
    struct M { double Kx, Ky; cd H0, H0mc; uint32_t Multiple; };
    std::vector<M> Modes;
    explicit FTruthModes(const FSlot& Slot)
    {
        using namespace VoxelShader;
        const FVoxelOceanSlotParams& P = Slot.Params;
        const int R = int(std::ceil(std::sqrt(double(P.MMax))));
        const double Dk = 2.0 * Pi / double(P.TileLength);
        for (int Kj = -R; Kj <= R; Kj++)
            for (int Ki = -R; Ki <= R; Ki++)
            {
                if (!VoxelOceanInBand(P, Ki, Kj)) continue;
                const float2 H0 = VoxelOceanH0(P, Ki, Kj), Hm = VoxelOceanH0(P, -Ki, -Kj);
                Modes.push_back({ Ki * Dk, Kj * Dk, cd(H0.x, H0.y), cd(Hm.x, -Hm.y), uint32_t(VoxelOceanModeMultiple(P, Ki, Kj)) });
            }
    }
    // At (x, y) m and a tick (the phase exact, as the GPU's).
    FTruth At(double X, double Y, uint32_t Tick) const
    {
        cd S[11] = {};
        for (const M& m : Modes)
        {
            const uint32_t Cyc = m.Multiple * Tick;
            const double Phi = 2.0 * Pi * double(Cyc) / 4294967296.0;
            const cd A = m.H0 * std::polar(1.0, -Phi), B = m.H0mc * std::polar(1.0, Phi);
            const cd H = A + B;
            const double Om = m.Multiple * Omega0;
            const cd Ht = cd(0, Om) * (B - A);
            const double K = std::hypot(m.Kx, m.Ky), Ux = m.Kx / K, Uy = m.Ky / K;
            const cd E = std::polar(1.0, m.Kx * X + m.Ky * Y);
            const cd I(0, 1);
            S[0] += H * E; S[1] += I * Ux * H * E; S[2] += I * Uy * H * E;
            S[3] += I * m.Kx * H * E; S[4] += I * m.Ky * H * E;
            S[5] += -m.Kx * Ux * H * E; S[6] += -m.Ky * Uy * H * E; S[7] += -m.Kx * Uy * H * E;
            S[8] += Ht * E; S[9] += I * Ux * Ht * E; S[10] += I * Uy * Ht * E;
        }
        return { S[0].real(), S[1].real(), S[2].real(), S[3].real(), S[4].real(), S[5].real(), S[6].real(), S[7].real(),
            S[8].real(), S[9].real(), S[10].real() };
    }
};

// ---------------------------------------------------------------------------------------------

int main()
{
    FLayoutSettings Settings;
    Settings.PlanetSeed = 1234567u;
    std::vector<FSlot> Slots = MakeSlots(Settings);

    std::printf("Layout (seed %u):\n", Settings.PlanetSeed);
    for (int s = 0; s < SlotCount; s++)
    {
        const FSlot& S = Slots[s];
        std::printf("  slot %d: band %d state %2d  L %6.1f m  wind %4.1f  lambda %7.2f .. %6.2f m  m [%u, %u)\n", s, S.Band, S.State,
            S.Params.TileLength, S.Params.WindSpeed, 2 * Pi / S.KMin, 2 * Pi / S.KMax, S.Params.MMin, S.Params.MMax);
    }

    // ---- Spreading ---------------------------------------------------------------------------
    std::printf("\nSpreading normalisation:\n");
    {
        double Worst = 0.0, WorstS = 0.0;
        for (double S = 0.02; S <= 40.0; S *= 1.05)
        {
            double Sum = 0.0;
            const int Steps = 20000;
            for (int i = 0; i < Steps; i++)
            {
                const double Th = -Pi + (i + 0.5) * 2 * Pi / Steps;
                Sum += std::pow(0.5 + 0.5 * std::cos(Th), S) * 2 * Pi / Steps;
            }
            const double Err = std::fabs(VoxelShader::VoxelOceanSpreadNorm(float(S)) * Sum - 1.0);
            if (Err > Worst) { Worst = Err; WorstS = S; }
        }
        std::printf("  worst |integral - 1| = %.4f at s = %.2f (s in 0.02..40)\n", Worst, WorstS);
        Expect(Worst < 1e-5, "spreading integrates to 1 (1e-5)");
    }

    // ---- Spectrum: Hs and coverage -----------------------------------------------------------
    std::printf("\nSea states (fully developed unless fetch-limited; fetch %.0f km, gamma %.1f):\n", Settings.Fetch / 1000, Settings.Gamma);
    for (double Wind : { 3.0, 5.0, 8.0, 10.0, 14.0, 18.0, 22.0 })
    {
        const VoxelShader::FVoxelOceanJonswap J = VoxelShader::VoxelOceanJonswapOf(float(Wind), float(Settings.Fetch));
        double Var = 0.0, InRange = 0.0;
        const double WLo = std::sqrt(Gravity * BoundaryK(Settings, 0)), WHi = std::sqrt(Gravity * BoundaryK(Settings, 4));
        for (double W = 0.02; W < 60.0; W *= 1.0005)
        {
            const double S = VoxelShader::VoxelOceanJonswapS(float(W), J, float(Settings.Gamma)) * W * 0.0005;
            Var += S;
            if (W >= WLo && W < WHi) InRange += S;
        }
        FLayoutSettings L2 = Settings;
        L2.StateWind[1] = Wind;
        L2.ShortWind = Wind;
        const std::vector<FSlot> S2 = MakeSlots(L2);
        double Bands[4] = { StatsOf(S2[2]).HeightVariance, StatsOf(S2[3]).HeightVariance, StatsOf(S2[6]).HeightVariance, StatsOf(S2[7]).HeightVariance };
        const double BandSum = Bands[0] + Bands[1] + Bands[2] + Bands[3];
        const double Lp = 2 * Pi * Gravity / (J.OmegaP * J.OmegaP);
        std::printf("  U %4.1f m/s: peak %6.1f m  Hs %5.2f m (fully developed, 0.25 U^2/g: %5.2f)  bands hold %5.1f%%  [%4.1f %4.1f %4.1f %4.1f]%%\n",
            Wind, Lp, 4 * std::sqrt(Var), 0.25 * Wind * Wind / Gravity, 100 * BandSum / Var,
            100 * Bands[0] / Var, 100 * Bands[1] / Var, 100 * Bands[2] / Var, 100 * Bands[3] / Var);
        if (Gravity * Settings.Fetch / (Wind * Wind) >= VOXEL_OCEAN_FULL_FETCH)
            Expect(std::fabs(4 * std::sqrt(Var) / (0.25 * Wind * Wind / Gravity) - 1.0) < 0.1, "  a fully developed sea's Hs is the observed 0.25 U^2/g (10%)");
        if (Wind >= 5.0 && Wind <= 18.0)
            // The modes are a coarse sample of a narrow peak: up to about 10% off when the peak
            // sits on a band edge (8 m/s: 66 m against the 60 m edge).
            Expect(std::fabs(BandSum / InRange - 1.0) < 0.12, "  the bands' modes hold the spectrum's variance in their range (12%)");
    }

    // ---- Band split --------------------------------------------------------------------------
    std::printf("\nBand split:\n");
    {
        bool Ok = true;
        for (int s = 0; s < SlotCount; s++)
        {
            const FSlot& S = Slots[s];
            const double Dk = 2 * Pi / double(S.Params.TileLength);
            double KLo = 1e9, KHi = 0;
            const int R = int(std::ceil(std::sqrt(double(S.Params.MMax))));
            for (int j = -R; j <= R; j++)
                for (int i = -R; i <= R; i++)
                    if (VoxelShader::VoxelOceanInBand(S.Params, i, j)) { const double K = std::hypot(i, j) * Dk; KLo = std::min(KLo, K); KHi = std::max(KHi, K); }
            const bool In = KLo >= S.KMin * (1 - 1e-6) && KHi < S.KMax;
            const bool NoNyquist = R < GPUSize / 2;
            std::printf("  slot %d: modes |k| %.5f .. %.5f within [%.5f, %.5f)  %s\n", s, KLo, KHi, S.KMin, S.KMax, In && NoNyquist ? "" : "<-");
            Ok = Ok && In && NoNyquist;
        }
        for (int b = 0; b < 3; b++)
        {
            const FSlot& Lo = Slots[b == 0 ? 0 : (b == 1 ? 1 : 6)];
            const FSlot& Hi = Slots[b == 0 ? 1 : (b == 1 ? 6 : 7)];
            Ok = Ok && Lo.KMax == Hi.KMin;
        }
        Expect(Ok, "bands meet edge to edge, every kept mode inside its band, no Nyquist mode");
    }

    // ---- Dispersion --------------------------------------------------------------------------
    std::printf("\nDispersion:\n");
    {
        // The reference: the largest n in [0, MAX] with (2n - 1)^4 <= mant m 2^(exp + 4), in
        // 128-bit integers.
        auto Fits = [](int N, uint32_t C2, uint32_t M)
        {
            unsigned __int128 L = (unsigned __int128)(2 * N - 1);
            L = L * L * L * L;
            unsigned __int128 R = (unsigned __int128)(C2 & 0xffffffu) * M;
            const int Sh = int(C2 >> 24) - 128 + 4;
            if (Sh >= 0) R <<= Sh; else L <<= -Sh;
            return L <= R;
        };
        auto Exact = [&](uint32_t C2, uint32_t M)
        {
            const double Est = std::pow(UnpackDispersion(C2) * M, 0.25);
            int N = std::clamp(int(std::floor(Est + 0.5)) + 3, 0, VOXEL_OCEAN_MAX_MULTIPLE);
            while (N > 0 && !Fits(N, C2, M)) N--;
            return N;
        };
        long Bad = 0, Total = 0, OffTrue = 0, MaxN = 0;
        for (const FSlot& S : Slots)
        {
            const int R = int(std::ceil(std::sqrt(double(S.Params.MMax))));
            const double L = S.Params.TileLength;
            for (int j = -R; j <= R; j++)
                for (int i = -R; i <= R; i++)
                {
                    if (!VoxelShader::VoxelOceanInBand(S.Params, i, j)) continue;
                    const int N = VoxelShader::VoxelOceanModeMultiple(S.Params, i, j);
                    const uint32_t M = uint32_t(i * i + j * j);
                    if (N != Exact(S.Params.DispersionC2, M)) Bad++;
                    const double Om = std::sqrt(Gravity * std::sqrt(double(M)) * 2 * Pi / L);
                    if (N != int(std::floor(Om / Omega0 + 0.5))) OffTrue++;
                    MaxN = std::max<long>(MaxN, N);
                    Total++;
                }
        }
        std::printf("  layout: %ld modes, largest multiple %ld; against the exact reference: %ld wrong; against round(omega/omega0) in double: %ld differ (C2's 24-bit rounding, the same on both sides)\n",
            Total, MaxN, Bad, OffTrue);
        Expect(Bad == 0, "the shared multiple is exactly round((C2 m)^(1/4)) for every mode of the layout");
        // Stress: random tiles (5 m to 5 km) and modes, and near-ties built on purpose (C2 m within
        // 1e-7 of (n - 1/2)^4).
        std::mt19937_64 Rng(99);
        long StressBad = 0, Ties = 0, Tested = 0;
        const int Cases = 400000;
        for (int k = 0; k < Cases; k++)
        {
            // A mode (Ki, Kj) inside the grid's half, and C2 for it.
            const int Ki = int(Rng() % 128u), Kj = int(Rng() % 128u);
            const uint32_t M = uint32_t(Ki * Ki + Kj * Kj);
            if (M == 0u || M >= 16384u) continue;
            uint32_t C2;
            if (k & 1)
            {
                const double L = std::exp(std::uniform_real_distribution<double>(std::log(5.0), std::log(5000.0))(Rng));
                const double C = Gravity * 2.0 * Pi / (L * Omega0 * Omega0);
                C2 = PackDispersion(C * C);
            }
            else
            {
                const int N = 1 + int(Rng() % VOXEL_OCEAN_MAX_MULTIPLE);
                const double D = std::uniform_real_distribution<double>(-1e-7, 1e-7)(Rng);
                C2 = PackDispersion(std::pow(N - 0.5, 4.0) / M * (1.0 + D));
                Ties++;
            }
            if (UnpackDispersion(C2) * M >= std::pow(VOXEL_OCEAN_MAX_MULTIPLE + 0.5, 4.0)) continue;   // outside any band MakeSlot allows
            const int Got = VoxelShader::VoxelOceanModeMultiple(VoxelShader::VoxelOceanMakeSlot(1, 1, 1, 1, 0, 1, C2, 0, 16384, 0), Ki, Kj);
            if (Got != Exact(C2, M)) StressBad++;
            Tested++;
        }
        std::printf("  stress: %ld cases tested (%ld built as near-ties): %ld wrong\n", Tested, Ties, StressBad);
        Expect(StressBad == 0, "the shared multiple is exact under stress, near-ties included");
        Expect(MaxN <= VOXEL_OCEAN_MAX_MULTIPLE, "every multiple within the exact range");
    }

    // ---- The GPU passes against the direct DFT -----------------------------------------------
    std::printf("\nThe GPU passes (float) against a direct DFT (double):\n");
    const uint32_t Tick = TickOf(123.456);
    std::vector<FGPUSlot> GPU(SlotCount);
    for (int s = 0; s < SlotCount; s++) GPU[s] = EmulateGPU(Slots[s], Tick);
    {
        std::mt19937 Rng(7);
        bool Ok = true;
        for (int s : { 0, 3, 5, 6, 7 })
        {
            const FTruthModes T(Slots[s]);
            const double L = Slots[s].Params.TileLength, Chop = Slots[s].Chop;
            double Err[8] = {}, Mag[8] = {};
            for (int k = 0; k < 40; k++)
            {
                const int i = Rng() % GPUSize, j = Rng() % GPUSize;
                const FTruth R = T.At(i * L / GPUSize, j * L / GPUSize, Tick);
                const F4 D = GPU[s].Displacement[size_t(j) * GPUSize + i], G = GPU[s].Derivatives[size_t(j) * GPUSize + i];
                const double Got[8] = { D.z, D.x / Chop, D.y / Chop, G.x, G.y, G.z / Chop, G.w / Chop, D.w / Chop };
                const double Want[8] = { R.Eta, R.Dx, R.Dy, R.EtaX, R.EtaY, R.Dxx, R.Dyy, R.Dxy };
                for (int f = 0; f < 8; f++) { Err[f] = std::max(Err[f], std::fabs(Got[f] - Want[f])); Mag[f] = std::max(Mag[f], std::fabs(Want[f])); }
            }
            const char* Names[8] = { "eta", "Dx", "Dy", "eta_x", "eta_y", "Dxx", "Dyy", "Dxy" };
            std::printf("  slot %d: max error / max value:", s);
            for (int f = 0; f < 8; f++)
            {
                std::printf(" %s %.1e", Names[f], Err[f] / std::max(Mag[f], 1e-30));
                Ok = Ok && Err[f] <= 2e-4 * Mag[f] + 1e-7;
            }
            std::printf("\n");
        }
        Expect(Ok, "all eight fields match the direct DFT (relative 2e-4)");
    }

    // ---- Physical directions -----------------------------------------------------------------
    std::printf("\nPhysical directions (slot 3, 10 m/s, from the emulated GPU fields alone):\n");
    {
        const FGPUSlot Later = EmulateGPU(Slots[3], Tick + TickOf(0.02));
        double SEt = 0, SEx = 0, SEtEx = 0, SEt2 = 0, SEx2 = 0, SH = 0, SJ = 0, SHJ = 0, SH2 = 0, SJ2 = 0;
        const double N = double(GPU[3].Displacement.size());
        for (size_t i = 0; i < GPU[3].Displacement.size(); i++)
        {
            const double Et = (Later.Displacement[i].z - GPU[3].Displacement[i].z) / 0.02, Ex = GPU[3].Derivatives[i].x;
            SEt += Et; SEx += Ex; SEtEx += Et * Ex; SEt2 += Et * Et; SEx2 += Ex * Ex;
            const F4 D = GPU[3].Displacement[i], G = GPU[3].Derivatives[i];
            const double H = D.z, J = (1 + G.z) * (1 + G.w) - D.w * D.w;
            SH += H; SJ += J; SHJ += H * J; SH2 += H * H; SJ2 += J * J;
        }
        auto Corr = [&](double A, double B, double AB, double A2, double B2) { return (AB / N - A * B / (N * N)) / std::sqrt((A2 / N - A * A / (N * N)) * (B2 / N - B * B / (N * N))); };
        const double CT = Corr(SEt, SEx, SEtEx, SEt2, SEx2), CJ = Corr(SH, SJ, SHJ, SH2, SJ2);
        std::printf("  corr(eta_t, eta_x) %.3f (negative: crests move downwind, +x); corr(eta, Jacobian) %.3f (negative: crests compress)\n", CT, CJ);
        Expect(CT < -0.5, "the waves travel downwind");
        Expect(CJ < -0.3, "the chop sharpens crests (the surface compresses at crests)");
    }

    // ---- Statistics --------------------------------------------------------------------------
    std::printf("\nVariance:\n");
    {
        // Parseval: the GPU grid's variance is the sum of |h(k, t)|^2 over its modes, exactly.
        bool Ok = true;
        for (int s = 0; s < SlotCount; s++)
        {
            double Sum2 = 0;
            for (const F4& D : GPU[s].Displacement) Sum2 += double(D.z) * D.z;
            const FTruthModes T(Slots[s]);
            double Parseval = 0;
            for (const auto& m : T.Modes)
            {
                const double Phi = 2.0 * Pi * double(uint32_t(m.Multiple * Tick)) / 4294967296.0;
                Parseval += std::norm(m.H0 * std::polar(1.0, -Phi) + m.H0mc * std::polar(1.0, Phi));
            }
            const double Grid = Sum2 / GPU[s].Displacement.size();
            Ok = Ok && std::fabs(Grid / Parseval - 1) < 1e-4;
        }
        Expect(Ok, "every slot's grid variance is its modes' sum (Parseval, 1e-4)");
        // Expectation: averaged over 100 seeds, the modes' sum is the spectrum's variance.
        bool OkE = true;
        for (int s : { 1, 3, 5, 6, 7 })
        {
            const FBandStats St = StatsOf(Slots[s]);
            double Mean = 0;
            const int Seeds = 100;
            for (int k = 0; k < Seeds; k++)
            {
                FSlot S2 = Slots[s];
                S2.Params.Seed = VoxelShader::VoxelOceanHash(0x5eed0000u + uint32_t(k));
                const FTruthModes T(S2);
                double Sum = 0;
                for (const auto& m : T.Modes)
                {
                    const double Phi = 2.0 * Pi * double(uint32_t(m.Multiple * Tick)) / 4294967296.0;
                    Sum += std::norm(m.H0 * std::polar(1.0, -Phi) + m.H0mc * std::polar(1.0, Phi));
                }
                Mean += Sum / Seeds;
            }
            std::printf("  slot %d: %5d modes, spectrum variance %.5f m^2, mean over %d seeds %.5f m^2 (ratio %.3f)\n", s, St.Modes, St.HeightVariance, Seeds, Mean, Mean / St.HeightVariance);
            OkE = OkE && std::fabs(Mean / St.HeightVariance - 1) < 0.06;
        }
        Expect(OkE, "the start spectrum's expected variance is the spectrum's (6% over 100 seeds)");
    }

    // ---- Time loop ---------------------------------------------------------------------------
    std::printf("\nTime loop:\n");
    {
        Expect(TickOf(5.25) == TickOf(5.25 + LoopPeriod) && TickOf(5.25) == TickOf(5.25 - 3 * LoopPeriod), "the tick wraps at T for any sign");
        const FTruthModes T(Slots[2]);
        const FTruth A = T.At(10, 20, TickOf(LoopPeriod - 0.0005)), B = T.At(10, 20, TickOf(0.0005));
        std::printf("  height 1 ms across the wrap: %.6f vs %.6f m (vertical speed there %.3f m/s)\n", A.Eta, B.Eta, A.EtaT);
        Expect(std::fabs(A.Eta - B.Eta) < 0.001 * std::fabs(A.EtaT) + 1e-5, "the field is continuous across the wrap");
    }

    // ---- The query cascades ------------------------------------------------------------------
    std::printf("\nQuery cascades (CPU) against the direct DFT and the GPU's bilinear lookup:\n");
    for (int N : { 64, 128 })
    {
        for (int s : { 2, 3, 4, 5, 6 })
        {
            FQueryCascade Q(Slots[s], N);
            if (!Q.IsValid()) { std::printf("  slot %d N %d: band doesn't fit\n", s, N); Failures++; continue; }
            Q.Update(Tick);
            const FTruthModes T(Slots[s]);
            const double L = Slots[s].Params.TileLength, Chop = Slots[s].Chop;
            std::mt19937 Rng(11 + s);
            double EH = 0, ED = 0, EG = 0, EDG = 0, EV = 0, EVH = 0, MH = 0, MG = 0, MDG = 0, MV = 0, MVH = 0, GH = 0, GD = 0;
            for (int k = 0; k < 300; k++)
            {
                const double X = std::uniform_real_distribution<double>(-3 * L, 3 * L)(Rng), Y = std::uniform_real_distribution<double>(-3 * L, 3 * L)(Rng);
                const FQuerySample S = Q.Sample(X, Y);
                const FTruth R = T.At(X, Y, Tick);
                EH = std::max(EH, std::fabs(S.Height - R.Eta));
                ED = std::max(ED, std::hypot(S.DisplacementX - Chop * R.Dx, S.DisplacementY - Chop * R.Dy));
                EG = std::max(EG, std::hypot(S.HeightDx - R.EtaX, S.HeightDy - R.EtaY));
                EDG = std::max({ EDG, std::fabs(S.DxDx - Chop * R.Dxx), std::fabs(S.DyDy - Chop * R.Dyy), std::fabs(S.DxDy - Chop * R.Dxy), std::fabs(S.DyDx - Chop * R.Dxy) });
                EV = std::max(EV, std::fabs(S.VelocityZ - R.EtaT));
                EVH = std::max(EVH, std::hypot(S.VelocityX - Chop * R.DxT, S.VelocityY - Chop * R.DyT));
                MH = std::max(MH, std::fabs(R.Eta)); MG = std::max(MG, std::hypot(R.EtaX, R.EtaY)); MV = std::max(MV, std::fabs(R.EtaT));
                MDG = std::max({ MDG, std::fabs(Chop * R.Dxx), std::fabs(Chop * R.Dyy) });
                MVH = std::max(MVH, std::hypot(Chop * R.DxT, Chop * R.DyT));
                const F4 B = Bilinear(GPU[s].Displacement, X / L * GPUSize, Y / L * GPUSize);
                GH = std::max(GH, double(std::fabs(B.z - S.Height)));
                GD = std::max(GD, double(std::hypot(B.x - S.DisplacementX, B.y - S.DisplacementY)));
            }
            std::printf("  N %3d slot %d: height %.1f mm (of %.2f m)  displacement %.1f mm  slope %.4f (of %.3f)  dD/dx %.4f (of %.3f)  velocity %.1f mm/s (of %.2f), horizontal %.1f mm/s (of %.2f) | vs GPU bilinear: height %.1f mm, displacement %.1f mm\n",
                N, s, EH * 1000, MH, ED * 1000, EG, MG, EDG, MDG, EV * 1000, MV, EVH * 1000, MVH, GH * 1000, GD * 1000);
            if (N == 128)
            {
                Expect(EH < 0.01 * MH + 0.002 && EG < 0.05 * MG && EDG < 0.05 * MDG + 0.002 && EV < 0.01 * MV + 0.002 && EVH < 0.01 * MVH + 0.002,
                    "  128^2 query matches the true field (1% height and velocities, 5% gradients)");
                Expect(GH < 0.03 && GD < 0.03, "  the query and the GPU's bilinear lookup agree within 3 cm");
            }
        }
    }

    // ---- fp16 --------------------------------------------------------------------------------
    std::printf("\nfp16 output rounding:\n");
    for (int s : { 4, 5, 6, 7 })
    {
        double MaxErr = 0, MaxV = 0;
        for (const F4& D : GPU[s].Displacement)
            for (float V : { D.x, D.y, D.z })
            {
                MaxErr = std::max(MaxErr, std::fabs(double(VoxelShader::f16tof32(VoxelShader::f32tof16(V))) - V));
                MaxV = std::max(MaxV, double(std::fabs(V)));
            }
        std::printf("  slot %d: largest |displacement| %.2f m, fp16 rounding up to %.1f mm\n", s, MaxV, MaxErr * 1000);
        Expect(MaxErr < 0.003, "  fp16 rounding below 3 mm");
    }

    // ---- Folding -----------------------------------------------------------------------------
    std::printf("\nFolding (all four bands, chop %.2f):\n", Settings.Chop[0]);
    for (int State = 0; State < StateCount; State++)
    {
        FLayoutSettings L2 = Settings;
        L2.ShortWind = Settings.StateWind[State];
        const std::vector<FSlot> S2 = MakeSlots(L2);
        const int Use[4] = { 2 * State, 2 * State + 1, 6, 7 };
        FGPUSlot G[4];
        for (int b = 0; b < 4; b++) G[b] = EmulateGPU(S2[Use[b]], Tick);
        std::mt19937 Rng(5);
        int Folded = 0, Count = 20000;
        double SumH = 0, SumH2 = 0, MinJ = 1e9;
        for (int k = 0; k < Count; k++)
        {
            const double X = std::uniform_real_distribution<double>(0, 5000)(Rng), Y = std::uniform_real_distribution<double>(0, 5000)(Rng);
            double Dxx = 0, Dyy = 0, Dxy = 0, H = 0;
            for (int b = 0; b < 4; b++)
            {
                const double L = S2[Use[b]].Params.TileLength;
                const F4 D = Bilinear(G[b].Displacement, X / L * GPUSize, Y / L * GPUSize), R = Bilinear(G[b].Derivatives, X / L * GPUSize, Y / L * GPUSize);
                Dxx += R.z; Dyy += R.w; Dxy += D.w; H += D.z;
            }
            const double J = (1 + Dxx) * (1 + Dyy) - Dxy * Dxy;
            Folded += J < 0;
            MinJ = std::min(MinJ, J);
            SumH += H; SumH2 += H * H;
        }
        const double Sig = std::sqrt(SumH2 / Count - std::pow(SumH / Count, 2));
        std::printf("  U %4.1f m/s: Hs %.2f m, folded %.2f%% of the surface, min Jacobian %.2f\n", Settings.StateWind[State], 4 * Sig, 100.0 * Folded / Count, MinJ);
    }

    // ---- Detiling ----------------------------------------------------------------------------
    std::printf("\nDetiling (10 m/s, all four bands; the two long bands detiled, cell 1.5 km):\n");
    {
        const int Use[4] = { 2, 3, 6, 7 };
        const uint32_t Seed = VoxelShader::VoxelOceanHash(Settings.PlanetSeed ^ 0xde711eu);
        const float Cell = 1500.0f;
        auto Height = [&](double X, double Y, bool Detile, double* SlopeX)
        {
            double H = 0, Sx = 0;
            for (int b = 0; b < 4; b++)
            {
                const double L = Slots[Use[b]].Params.TileLength;
                if (Detile && b < 2)
                {
                    const VoxelShader::FVoxelOceanDetile T = VoxelShader::VoxelOceanDetileOf(float2(float(X), float(Y)), Cell, Seed);
                    const float2 O[3] = { T.Offset0, T.Offset1, T.Offset2 };
                    const float Wt[3] = { T.Weight.x, T.Weight.y, T.Weight.z };
                    for (int c = 0; c < 3; c++)
                    {
                        H += Wt[c] * Bilinear(GPU[Use[b]].Displacement, (X + O[c].x) / L * GPUSize, (Y + O[c].y) / L * GPUSize).z;
                        Sx += Wt[c] * Bilinear(GPU[Use[b]].Derivatives, (X + O[c].x) / L * GPUSize, (Y + O[c].y) / L * GPUSize).x;
                    }
                }
                else
                {
                    H += Bilinear(GPU[Use[b]].Displacement, X / L * GPUSize, Y / L * GPUSize).z;
                    Sx += Bilinear(GPU[Use[b]].Derivatives, X / L * GPUSize, Y / L * GPUSize).x;
                }
            }
            if (SlopeX) *SlopeX = Sx;
            return H;
        };
        std::mt19937 Rng(21);
        const double L0 = Slots[2].Params.TileLength;
        double Var[2] = {}, Corr[2] = {}, SCorr[2] = {}, SVar[2] = {};
        const int Count = 40000;
        for (int k = 0; k < Count; k++)
        {
            const double X = std::uniform_real_distribution<double>(0, 30000)(Rng), Y = std::uniform_real_distribution<double>(0, 30000)(Rng);
            for (int d = 0; d < 2; d++)
            {
                double S0, S1;
                const double A = Height(X, Y, d == 1, &S0), B = Height(X + L0, Y, d == 1, &S1);
                Var[d] += A * A; Corr[d] += A * B; SVar[d] += S0 * S0; SCorr[d] += S0 * S1;
            }
        }
        std::printf("  height variance: plain %.4f m^2, detiled %.4f m^2 (ratio %.3f)\n", Var[0] / Count, Var[1] / Count, Var[1] / Var[0]);
        std::printf("  correlation one 997 m tile apart: height plain %.3f, detiled %.3f; slope plain %.3f, detiled %.3f\n",
            Corr[0] / Var[0], Corr[1] / Var[1], SCorr[0] / SVar[0], SCorr[1] / SVar[1]);
        Expect(std::fabs(Var[1] / Var[0] - 1) < 0.05, "detiling keeps the height variance (5%)");
        Expect(Corr[1] / Var[1] < 0.5 * Corr[0] / Var[0], "detiling at least halves the correlation at the tile period");
        // Continuity: the largest step between points 0.1 mm apart, across many lattice edges (a
        // smooth surface with slopes below 1 changes less than 0.1 mm there; a seam would not).
        double Jump = 0;
        for (int k = 0; k < 20000; k++)
        {
            const double X = std::uniform_real_distribution<double>(0, 30000)(Rng), Y = std::uniform_real_distribution<double>(0, 30000)(Rng);
            Jump = std::max(Jump, std::fabs(Height(X, Y, true, nullptr) - Height(X + 0.0001, Y + 0.00005, true, nullptr)));
        }
        std::printf("  largest height step over 0.1 mm: %.4f mm\n", Jump * 1000);
        Expect(Jump < 0.0002, "the detiled field is continuous (under 0.2 mm per 0.1 mm)");
    }

    // ---- Determinism and cost ----------------------------------------------------------------
    std::printf("\nDeterminism and cost:\n");
    {
        const FGPUSlot Again = EmulateGPU(Slots[3], Tick);
        bool Same = std::memcmp(Again.Displacement.data(), GPU[3].Displacement.data(), Again.Displacement.size() * sizeof(F4)) == 0;
        Expect(Same, "the same seed and tick give bit-identical fields");
        FLayoutSettings Other = Settings;
        Other.PlanetSeed = Settings.PlanetSeed + 1;
        const FGPUSlot Diff = EmulateGPU(MakeSlots(Other)[3], Tick);
        Expect(Diff.Displacement[1000].z != GPU[3].Displacement[1000].z, "another seed gives another sea");
        for (int N : { 64, 128, 256 })
        {
            FQueryCascade Q(Slots[2], N);
            const auto T0 = std::chrono::steady_clock::now();
            const int Runs = 20;
            for (int r = 0; r < Runs; r++) Q.Update(Tick + r * 1000u);
            const double Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T0).count() / Runs;
            std::printf("  query cascade update, N %3d: %.3f ms per slot (6 long slots at 15 Hz: %.1f%% of a core)\n", N, Ms, Ms * 6 * 15 / 10.0);
        }
    }

    std::printf("\n%s (%d failures)\n", Failures ? "FAILED" : "ALL OK", Failures);
    return Failures ? 1 : 0;
}
