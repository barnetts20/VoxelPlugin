// cubic_check.cpp - The cubic B-spline lookups' taps (stage W3u; VoxelOceanCubic.ush, as the GPU
// runs them), checked without the engine:
// - the four bilinear taps (a CPU bilinear sampler with wrapping, as the cascades' and the clipmap's
//   samplers) against the 16-texel B-spline sum, on random textures of 256 and 128 texels, at
//   random coordinates, wrapped ones included;
// - the weights: positive, summing to 1, each tap between its two texels;
// - the filter reproduces a linear ramp exactly (no shift: the texel-centre convention matches the
//   bilinear sampler's);
// - smoothness: across texel boundaries the value and its first and second differences change
//   continuously (C2), where the bilinear's first difference jumps;
// - the share toward the cubic: none at a texel per footprint or coarser, all from two.
//
//   g++ -O2 -std=c++20 -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Shaders/Private cubic_check.cpp -o cubic_check

#include "VoxelShaderCompat.h"

namespace VoxelShader
{
#include "VoxelOceanCubic.ush"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace VoxelShader;

static int Failures = 0;
static void Expect(bool bOk, const char* What, double Value)
{
    std::printf("  [%s] %-70s %.4g\n", bOk ? "ok" : "FAIL", What, Value);
    if (!bOk) Failures++;
}

struct FTex
{
    int N = 0;
    std::vector<double> V;
    double At(int I, int J) const
    {
        I = ((I % N) + N) % N;
        J = ((J % N) + N) % N;
        return V[(size_t)J * N + I];
    }
    // A bilinear sampler with wrapping (texel i's centre at (i + 0.5) / N).
    double Bilinear(double U, double W) const
    {
        const double X = U * N - 0.5, Y = W * N - 0.5;
        const double I = std::floor(X), J = std::floor(Y);
        const double FX = X - I, FY = Y - J;
        const int i = (int)I, j = (int)J;
        return (1 - FY) * ((1 - FX) * At(i, j) + FX * At(i + 1, j)) + FY * ((1 - FX) * At(i, j + 1) + FX * At(i + 1, j + 1));
    }
    // The B-spline's direct sum over 16 texels.
    double Direct(double U, double W) const
    {
        const double X = U * N - 0.5, Y = W * N - 0.5;
        const double I = std::floor(X), J = std::floor(Y);
        auto B = [](double F, double* Out)
        {
            Out[0] = (1 - F) * (1 - F) * (1 - F) / 6.0;
            Out[1] = (4 - 6 * F * F + 3 * F * F * F) / 6.0;
            Out[2] = (1 + 3 * F + 3 * F * F - 3 * F * F * F) / 6.0;
            Out[3] = F * F * F / 6.0;
        };
        double WX[4], WY[4];
        B(X - I, WX);
        B(Y - J, WY);
        double S = 0.0;
        for (int b = 0; b < 4; b++)
            for (int a = 0; a < 4; a++) S += WX[a] * WY[b] * At((int)I - 1 + a, (int)J - 1 + b);
        return S;
    }
    // The four taps (VoxelOceanCubicTapsOf), as the GPU combines them.
    double Cubic(double U, double W) const
    {
        const FVoxelOceanCubicTaps X = VoxelOceanCubicTapsOf((float)U, (float)N);
        const FVoxelOceanCubicTaps Y = VoxelOceanCubicTapsOf((float)W, (float)N);
        return Y.G0 * (X.G0 * Bilinear(X.P0, Y.P0) + X.G1 * Bilinear(X.P1, Y.P0)) + Y.G1 * (X.G0 * Bilinear(X.P0, Y.P1) + X.G1 * Bilinear(X.P1, Y.P1));
    }
};

int main()
{
    std::mt19937 Rng(1234);
    std::uniform_real_distribution<double> Uni(0.0, 1.0);
    std::printf("1. The taps against the 16-texel sum\n");
    for (int N : { 256, 128 })
    {
        FTex T;
        T.N = N;
        T.V.resize((size_t)N * N);
        for (double& X : T.V) X = Uni(Rng) * 2.0 - 1.0;
        double Worst = 0.0;
        for (int K = 0; K < 200000; K++)
        {
            // Some wrapped (the clipmap's toroidal storage reads past 1 and under 0).
            const double U = Uni(Rng) * 3.0 - 1.0, W = Uni(Rng) * 3.0 - 1.0;
            Worst = std::max(Worst, std::abs(T.Cubic(U, W) - T.Direct(U, W)));
        }
        char Buf[128];
        std::snprintf(Buf, sizeof(Buf), "%d texels: four taps = sixteen texels (worst error, values +-1)", N);
        Expect(Worst < 2.0e-4, Buf, Worst);
    }

    std::printf("2. The weights\n");
    {
        double WorstSum = 0.0, MinG = 1.0, WorstBetween = 0.0;
        for (int K = 0; K <= 100000; K++)
        {
            const float U = (float)(K / 100000.0 * 4.0 / 256.0);
            const FVoxelOceanCubicTaps T = VoxelOceanCubicTapsOf(U, 256.0f);
            WorstSum = std::max(WorstSum, (double)std::abs(T.G0 + T.G1 - 1.0f));
            MinG = std::min(MinG, (double)std::min(T.G0, T.G1));
            // Each tap between its pair's centres: P0 in [I - 0.5, I + 0.5], P1 in [I + 1.5, I + 2.5] (texels).
            const double X = U * 256.0 - 0.5, I = std::floor(X);
            const double P0 = T.P0 * 256.0, P1 = T.P1 * 256.0;
            WorstBetween = std::max(WorstBetween, std::max(std::max(I - 0.5 - P0, P0 - (I + 0.5)), std::max(I + 1.5 - P1, P1 - (I + 2.5))));
        }
        Expect(WorstSum < 1.0e-6, "the pairs' weights sum to 1", WorstSum);
        Expect(MinG > 0.16, "each pair's weight positive (at least 1/6)", MinG);
        Expect(WorstBetween < 1.0e-4, "each tap between its two texels' centres (texels outside)", WorstBetween);
    }

    std::printf("3. A linear ramp reproduced (no shift)\n");
    {
        FTex T;
        T.N = 256;
        T.V.resize(256 * 256);
        // A ramp in x over the middle (away from the wrap's seam), constant in y.
        for (int J = 0; J < 256; J++)
            for (int I = 0; I < 256; I++) T.V[(size_t)J * 256 + I] = 0.01 * I;
        double Worst = 0.0;
        for (int K = 0; K < 20000; K++)
        {
            const double U = (10.0 + Uni(Rng) * 230.0) / 256.0;
            const double Want = 0.01 * (U * 256.0 - 0.5);
            Worst = std::max(Worst, std::abs(T.Cubic(U, Uni(Rng)) - Want));
        }
        Expect(Worst < 1.0e-5, "the cubic of a ramp is the ramp (worst error)", Worst);
    }

    std::printf("4. Smoothness across texel edges\n");
    {
        FTex T;
        T.N = 256;
        T.V.resize(256 * 256);
        for (double& X : T.V) X = Uni(Rng) * 2.0 - 1.0;
        // Along a row through texel centres in y, steps of 1/64 texel across 40 texels: the second
        // difference of the cubic jumps by O(h^2 * third derivative) at most; the bilinear's first
        // difference jumps by O(h * its kink).
        const double H = 1.0 / 64.0;
        double WorstCubic = 0.0, WorstLinear = 0.0, TypCubic = 0.0;
        const double W = (100.0 + 0.5) / 256.0;
        std::vector<double> C, L;
        for (int K = 0; K <= 40 * 64; K++)
        {
            const double U = (50.0 + K * H) / 256.0;
            C.push_back(T.Cubic(U, W));
            L.push_back(T.Bilinear(U, W));
        }
        for (size_t K = 2; K + 1 < C.size(); K++)
        {
            const double D2a = C[K] - 2 * C[K - 1] + C[K - 2];
            const double D2b = C[K + 1] - 2 * C[K] + C[K - 1];
            WorstCubic = std::max(WorstCubic, std::abs(D2b - D2a));
            TypCubic = std::max(TypCubic, std::abs(D2a));
            const double D1a = L[K] - L[K - 1], D1b = L[K + 1] - L[K];
            WorstLinear = std::max(WorstLinear, std::abs(D1b - D1a));
        }
        // Per step the cubic's second difference changes by h^3 times its third derivative, at most
        // the texels' third difference, 8 for values within +-1: 8 / 64^3 ~ 3e-5 (and the float
        // taps' rounding), against the bilinear's kink ~2 / 64.
        Expect(WorstCubic < 5.0e-5, "the cubic's second difference continuous (worst step change)", WorstCubic);
        Expect(WorstLinear > 1.0e-2, "(the bilinear's first difference jumps at the edges)", WorstLinear);
        Expect(TypCubic > 1.0e-5, "(the cubic's second difference not trivially zero)", TypCubic);
    }

    std::printf("5. The share toward the cubic\n");
    {
        Expect(VoxelOceanCubicShareOf(0.5f) == 0.0f, "a texel per two footprints: bilinear", VoxelOceanCubicShareOf(0.5f));
        Expect(VoxelOceanCubicShareOf(1.0f) == 0.0f, "a texel per footprint: bilinear", VoxelOceanCubicShareOf(1.0f));
        Expect(std::abs(VoxelOceanCubicShareOf(1.41421356f) - 0.5f) < 1.0e-5f, "half an octave magnified: half", VoxelOceanCubicShareOf(1.41421356f));
        Expect(VoxelOceanCubicShareOf(2.0f) == 1.0f, "two pixels a texel: cubic", VoxelOceanCubicShareOf(2.0f));
        Expect(VoxelOceanCubicShareOf(50.0f) == 1.0f, "fifty: cubic", VoxelOceanCubicShareOf(50.0f));
    }

    std::printf(Failures ? "FAILED: %d\n" : "ALL OK\n", Failures);
    return Failures ? 1 : 0;
}
