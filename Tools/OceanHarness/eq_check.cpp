// W3k: the wave equalizer (VoxelOceanEqualizerGain, VoxelOceanSpectrum.ush), checked without
// the engine.
// - The gain: exactly 1 everywhere when flat (the default sea is unchanged bit for bit); each
//   knot's gain at its own wavelength; linear in the octave between knots; held past the ends.
// - The GPU's slot table: the gains packed by ToGPU and unpacked as VoxelOceanFFT.usf does
//   (VoxelOceanSlotWithGains) give the CPU's start spectrum bit for bit, every mode, every slot.
// - The statistics (StatsOf, which the foam, the whitecaps and the reach read): a uniform gain g
//   scales every slot's height and slope variance by g^2; a single boosted octave adds what a
//   double-precision sum of the gain over the slot's modes predicts.
// - The emulated GPU field's variance follows (Parseval over the emulated grid).
// - A few settings and what they do to the sea (informational).
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private -I. eq_check.cpp -o eq_check
#include "VoxelOceanSpectrum.h"
#include "ocean_fft_emulation.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <vector>

using namespace VoxelOceanSpectrum;
using namespace OceanFFTEmulation;
using VoxelShader::float2;
using VoxelShader::float4;

static int Failures = 0;
static void Expect(bool Ok, const char* What)
{
    std::printf("  [%s] %s\n", Ok ? "ok" : "FAIL", What);
    if (!Ok) Failures++;
}

// The gain in double from the layout's knots (the reference).
static double GainRef(const FLayoutSettings& S, double K)
{
    const double Octave = std::log2(2.0 * Pi / K) - double(VOXEL_OCEAN_EQ_LOW);
    const double X = std::min(std::max(Octave, 0.0), double(VOXEL_OCEAN_EQ_KNOTS - 1));
    const int I = std::min(int(std::floor(X)), VOXEL_OCEAN_EQ_KNOTS - 2);
    const double F = X - I;
    return S.Gain[I] + (S.Gain[I + 1] - S.Gain[I]) * F;
}

static double SlotGainVarianceRef(const FSlot& Slot, const FLayoutSettings& Eq)
{
    // The flat spectrum's modes, weighted by the reference gain squared.
    using namespace VoxelShader;
    FVoxelOceanSlotParams P = Slot.Params;
    for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) P.Gain[i] = 1.0f;
    const FVoxelOceanJonswap J = VoxelOceanJonswapOf(P.WindSpeed, P.Fetch);
    const double Dk = 2.0 * Pi / P.TileLength;
    const int R = int(std::ceil(std::sqrt(double(P.MMax))));
    double Sum = 0.0;
    for (int Kj = -R; Kj <= R; Kj++)
        for (int Ki = -R; Ki <= R; Ki++)
        {
            if (!VoxelOceanInBand(P, Ki, Kj)) continue;
            const double Kx = Ki * Dk, Ky = Kj * Dk;
            const double G = GainRef(Eq, std::sqrt(Kx * Kx + Ky * Ky));
            Sum += double(VoxelOceanSpectrumK(P, J, float(Kx), float(Ky))) * Dk * Dk * G * G;
        }
    return Sum;
}

int main()
{
    using namespace VoxelShader;

    std::printf("The gain:\n");
    {
        FVoxelOceanSlotParams P = MakeSlots(FLayoutSettings())[0].Params;
        bool bFlat = true;
        for (int i = 0; i < 20000; i++)
        {
            const double Lambda = 0.05 * std::pow(10.0, 5.0 * i / 20000.0);  // 5 cm to 5 km
            if (VoxelOceanEqualizerGain(P, float(2.0 * Pi / Lambda)) != 1.0f) bFlat = false;
        }
        Expect(bFlat, "a flat equalizer is exactly 1 from 5 cm to 5 km");

        FLayoutSettings S;
        std::mt19937 Rng(7);
        std::uniform_real_distribution<double> U(0.0, 3.0);
        for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) S.Gain[i] = U(Rng);
        P = MakeSlots(S)[0].Params;
        double KnotErr = 0.0, MidErr = 0.0, RefErr = 0.0;
        for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++)
        {
            const double Lambda = std::ldexp(1.0, VOXEL_OCEAN_EQ_LOW + i);
            KnotErr = std::max(KnotErr, std::fabs(VoxelOceanEqualizerGain(P, float(2.0 * Pi / Lambda)) - S.Gain[i]));
            if (i + 1 < VOXEL_OCEAN_EQ_KNOTS)
            {
                const double Mid = Lambda * std::sqrt(2.0);
                MidErr = std::max(MidErr, std::fabs(VoxelOceanEqualizerGain(P, float(2.0 * Pi / Mid)) - 0.5 * (S.Gain[i] + S.Gain[i + 1])));
            }
        }
        for (int i = 0; i < 20000; i++)
        {
            const double Lambda = 0.05 * std::pow(10.0, 5.0 * i / 20000.0);
            const double K = 2.0 * Pi / Lambda;
            RefErr = std::max(RefErr, std::fabs(VoxelOceanEqualizerGain(P, float(K)) - GainRef(S, K)));
        }
        std::printf("  random gains 0-3: at the knots off by %.1e, halfway (in octaves) by %.1e, anywhere by %.1e\n", KnotErr, MidErr, RefErr);
        Expect(KnotErr < 2e-5, "each knot's gain at its own wavelength");
        Expect(MidErr < 2e-5, "halfway between two knots in octaves, their mean");
        Expect(RefErr < 2e-5, "the gain against the double reference everywhere");
        const float Short = VoxelOceanEqualizerGain(P, float(2.0 * Pi / 0.02));
        const float Long = VoxelOceanEqualizerGain(P, float(2.0 * Pi / 20000.0));
        Expect(Short == float(S.Gain[0]) && Long == float(S.Gain[VOXEL_OCEAN_EQ_KNOTS - 1]), "held past the ends (2 cm, 20 km)");
    }

    std::printf("\nThe GPU's slot table:\n");
    {
        FLayoutSettings S;
        std::mt19937 Rng(11);
        std::uniform_real_distribution<double> U(0.0, 3.0);
        for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) S.Gain[i] = U(Rng);
        const std::vector<FSlot> Slots = MakeSlots(S);
        long Modes = 0, Diff = 0;
        for (const FSlot& Slot : Slots)
        {
            const FSlotGPU G = ToGPU(Slot);
            // VoxelOceanFFT.usf's VoxelOceanSlotParamsOf.
            const FVoxelOceanSlotParams Gpu = VoxelOceanSlotWithGains(
                VoxelOceanMakeSlot(G.A[0], G.A[1], G.A[2], G.A[3], G.B[0], G.B[1], G.C[3], G.C[0], G.C[1], G.C[2]),
                float4(G.G[0], G.G[1], G.G[2], G.G[3]), float4(G.G[4], G.G[5], G.G[6], G.G[7]),
                float4(G.G[8], G.G[9], G.G[10], G.G[11]), float4(G.G[12], G.G[13], G.G[14], G.G[15]));
            const int R = int(std::ceil(std::sqrt(double(Slot.Params.MMax))));
            for (int Kj = -R; Kj <= R; Kj++)
                for (int Ki = -R; Ki <= R; Ki++)
                {
                    if (!VoxelOceanInBand(Slot.Params, Ki, Kj)) continue;
                    const float2 A = VoxelOceanH0(Slot.Params, Ki, Kj), B = VoxelOceanH0(Gpu, Ki, Kj);
                    Modes++;
                    if (std::memcmp(&A, &B, sizeof(float2)) != 0) Diff++;
                }
        }
        std::printf("  %ld modes over 8 slots, %ld differ\n", Modes, Diff);
        Expect(Diff == 0, "the unpacked table draws the CPU's start spectrum bit for bit");
        Expect(sizeof(FSlotGPU) == 112, "the slot is 112 bytes (VoxelOceanFFT.usf's FVoxelOceanSlotGPU: 7 float4s)");
    }

    std::printf("\nThe statistics:\n");
    {
        const std::vector<FSlot> Flat = MakeSlots(FLayoutSettings());
        FLayoutSettings Twice;
        for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) Twice.Gain[i] = 2.0;
        const std::vector<FSlot> Doubled = MakeSlots(Twice);
        double Worst = 0.0;
        for (int s = 0; s < SlotCount; s++)
        {
            const FBandStats A = StatsOf(Flat[s]), B = StatsOf(Doubled[s]);
            Worst = std::max(Worst, std::fabs(B.HeightVariance / A.HeightVariance - 4.0) / 4.0);
            Worst = std::max(Worst, std::fabs(B.SlopeVarianceX / A.SlopeVarianceX - 4.0) / 4.0);
            Worst = std::max(Worst, std::fabs(B.SlopeVarianceY / A.SlopeVarianceY - 4.0) / 4.0);
        }
        std::printf("  every gain 2: variances x4 within %.1e\n", Worst);
        Expect(Worst < 1e-5, "a uniform gain g scales every slot's height and slope variance by g^2");

        FLayoutSettings Zero;
        for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) Zero.Gain[i] = 0.0;
        double ZeroVar = 0.0;
        for (const FSlot& Slot : MakeSlots(Zero)) ZeroVar += StatsOf(Slot).HeightVariance;
        Expect(ZeroVar == 0.0, "every gain 0: no waves");

        // One octave boosted: 64 m (knot 8) to 2, its neighbours' slopes up to it.
        FLayoutSettings Boost;
        Boost.Gain[8] = 2.0;
        const std::vector<FSlot> Boosted = MakeSlots(Boost);
        double Err = 0.0;
        for (int s = 0; s < SlotCount; s++)
        {
            const double Got = StatsOf(Boosted[s]).HeightVariance;
            const double Want = SlotGainVarianceRef(Flat[s], Boost);
            const double Base = StatsOf(Flat[s]).HeightVariance;
            if (s < 6 || s == 6)
                std::printf("  slot %d (band %d, %s): sigma %.3f m flat, %.3f m with 64 m x2 (predicted %.3f)\n", s, Flat[s].Band,
                    Flat[s].State < 0 ? "short" : (Flat[s].State == 0 ? "5 m/s" : (Flat[s].State == 1 ? "10 m/s" : "18 m/s")),
                    std::sqrt(Base), std::sqrt(Got), std::sqrt(Want));
            Err = std::max(Err, std::fabs(Got - Want) / std::max(Want, 1e-12));
        }
        Expect(Err < 1e-4, "a boosted octave adds what the modes predict (every slot)");
        const double Band3 = StatsOf(Boosted[7]).HeightVariance / StatsOf(Flat[7]).HeightVariance;
        Expect(std::fabs(Band3 - 1.0) < 1e-6, "the 64 m octave leaves the 14 m band (21 cm - 3.4 m) alone");
    }

    std::printf("\nThe emulated GPU field:\n");
    {
        FLayoutSettings Boost;
        Boost.Gain[5] = 1.5;    // 8 m: band 2
        Boost.Gain[6] = 1.5;    // 16 m
        const FSlot A = MakeSlots(FLayoutSettings())[6], B = MakeSlots(Boost)[6];
        double VA = 0.0, VB = 0.0;
        const int Seeds = 12;
        for (int Seed = 0; Seed < Seeds; Seed++)
        {
            FSlot SA = A, SB = B;
            SA.Params.Seed = SB.Params.Seed = 0x5eed0000u + Seed;
            const FGPUSlot FA = EmulateGPU(SA, 123456789u), FB = EmulateGPU(SB, 123456789u);
            for (size_t i = 0; i < FA.Displacement.size(); i++)
            {
                VA += double(FA.Displacement[i].z) * FA.Displacement[i].z;
                VB += double(FB.Displacement[i].z) * FB.Displacement[i].z;
            }
        }
        const double Ratio = VB / VA;
        const double Want = StatsOf(B).HeightVariance / StatsOf(A).HeightVariance;
        std::printf("  band 2 with 8-16 m x1.5: the field's variance x%.3f, the statistics' x%.3f (%d seeds)\n", Ratio, Want, Seeds);
        Expect(std::fabs(Ratio / Want - 1.0) < 0.06, "the drawn field follows the statistics (6%, random start)");
    }

    std::printf("\nWhat some settings do (informational; Hs = 4 sigma, all four bands at 10 m/s):\n");
    {
        struct FPreset { const char* Name; double G[VOXEL_OCEAN_EQ_KNOTS]; };
        const FPreset Presets[] = {
            { "flat",                   { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 } },
            { "heavy swell (128-512 x1.5)", { 1, 1, 1, 1, 1, 1, 1, 1.25, 1.5, 1.5, 1.5, 1.5, 1.5 } },
            { "choppy (1-4 m x1.6)",    { 1, 1.3, 1.6, 1.6, 1.6, 1.3, 1, 1, 1, 1, 1, 1, 1 } },
            { "glassy (below 2 m x0.4)", { 0.4, 0.4, 0.4, 0.4, 0.7, 1, 1, 1, 1, 1, 1, 1, 1 } },
        };
        for (const FPreset& Pr : Presets)
        {
            FLayoutSettings S;
            for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++) S.Gain[i] = Pr.G[i];
            const std::vector<FSlot> Slots = MakeSlots(S);
            const int Use[4] = { 2, 3, 6, 7 };    // the 10 m/s long slots, the short ones
            double H = 0.0, Sl = 0.0;
            for (int s : Use) { const FBandStats St = StatsOf(Slots[s]); H += St.HeightVariance; Sl += St.SlopeVarianceX + St.SlopeVarianceY; }
            std::printf("  %-28s Hs %.2f m, slope deviation %.3f\n", Pr.Name, 4.0 * std::sqrt(H), std::sqrt(Sl));
        }
    }

    std::printf("\n%s (%d failures)\n", Failures == 0 ? "ALL OK" : "FAILED", Failures);
    return Failures == 0 ? 0 : 1;
}
