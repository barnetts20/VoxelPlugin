// VoxelOceanFoamTable.h - The foam set's thresholds (stage W3g; VoxelOceanFoamTex.ush reads them).
// Plain C++ without engine types: the plugin (VoxelOceanFoamTexture.cpp) and the harness
// (foam_set_check) share it.
//
// WHAT THE SHADER DOES with a channel: it samples the set at the pixel's level L (trilinear: the two
// mips around L, each bilinear) and covers the point by smoothstep(T - w, T + w, P): soft edges, w
// = VOXEL_OCEAN_FOAM_TEX_SOFT times the channel's spread at that level. For the cover to average a
// density D over the surface, T must be the threshold whose soft cover's mean is D, for the
// distribution of what the shader samples at that level (a bilinear or trilinear blend is narrower
// than the texels': a table of the texels' quantiles misses by half at the coarser levels).
//
// THE TABLE: a row per half level (L = r / 2, r = 0 .. 2 (Mips - 1)), columns:
//   k = 0 .. 64  each channel's T for the cover 1 - q_k, q_k = sin^2(pi k / 128) (finest near 0
//                and 1, where thin and solid foam take theirs; the shader indexes by q = 1 - D);
//   65           each channel's spread at that level, Q(0.9) - Q(0.1).
// Channels R G B: the set's lace, grain and whitecaps. A (stage W3m): not the set's streak channel
// (the shader no longer reads it) but the lace's even mix with an independent sample of itself,
// m + ((x1 - m) + (x2 - m)) / sqrt(2), m the lace's median at that level. The shader draws the
// streaks from the lace stretched downwind, mixed with the lace as the foam ages and the wind
// rises (VoxelOceanFoamTex.ush); a mix's distribution is narrower-tailed than either sample's, so
// its thresholds lie between the lace's (no mix) and these (an even one).
// From stratified sample points over the tile per row, their values binned in 65536 steps; T solved by bisection on the soft cover's mean, which the
// bins' prefix sums of 1, v, v^2 and v^3 give in closed form (the smoothstep is a cubic in v).
// Values in the texture's normalised units (u16 / 65535).

#pragma once

#include "VoxelNoiseTiles.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace VoxelOceanFoamTable
{
    constexpr int Columns = 66;
    constexpr int Knots = 65;
    // As VOXEL_OCEAN_FOAM_TEX_SOFT, per channel (A: the lace's mix, the lace's edge).
    constexpr double Soft[4] = { 0.15, 0.15, 0.15, 0.15 };

    inline int Rows(uint32_t Mips) { return 2 * (int)Mips - 1; }

    inline double KnotQuantile(int K)
    {
        const double S = std::sin(3.14159265358979323846 * (double)K / 128.0);
        return S * S;
    }

    // The set sampled as the GPU samples it (normalised, wrapped; texel i's centre at (i + 0.5) / S).
    struct FSampler
    {
        const VoxelNoiseTiles::FSet& Set;

        void Texel(int Mip, int X, int Y, double Out[4]) const
        {
            const int S = (int)(Set.N >> Mip);
            X = ((X % S) + S) % S;
            Y = ((Y % S) + S) % S;
            const uint16_t* T = Set.Texels.data() + (VoxelNoiseTiles::MipOffset(Set.N, (uint32_t)Mip) + (uint64_t)Y * S + X) * 4;
            for (int c = 0; c < 4; c++) Out[c] = T[c] / 65535.0;
        }
        void Bilinear(int Mip, double U, double V, double Out[4]) const
        {
            const int S = (int)(Set.N >> Mip);
            const double X = U * S - 0.5, Y = V * S - 0.5;
            const double X0 = std::floor(X), Y0 = std::floor(Y);
            const double Fx = X - X0, Fy = Y - Y0;
            const int Ix = (int)X0, Iy = (int)Y0;
            double A[4], B[4], C[4], D[4];
            Texel(Mip, Ix, Iy, A); Texel(Mip, Ix + 1, Iy, B); Texel(Mip, Ix, Iy + 1, C); Texel(Mip, Ix + 1, Iy + 1, D);
            for (int c = 0; c < 4; c++) Out[c] = (A[c] + (B[c] - A[c]) * Fx) * (1.0 - Fy) + (C[c] + (D[c] - C[c]) * Fx) * Fy;
        }
        void Trilinear(double Lod, double U, double V, double Out[4]) const
        {
            const int Top = (int)Set.Mips - 1;
            Lod = std::min(std::max(Lod, 0.0), (double)Top);
            const int M0 = std::min((int)Lod, Top), M1 = std::min(M0 + 1, Top);
            const double F = Lod - M0;
            double A[4], B[4];
            Bilinear(M0, U, V, A);
            if (F <= 0.0 || M1 == M0) { for (int c = 0; c < 4; c++) Out[c] = A[c]; return; }
            Bilinear(M1, U, V, B);
            for (int c = 0; c < 4; c++) Out[c] = A[c] + (B[c] - A[c]) * F;
        }
    };

    inline double Unit(uint64_t H)
    {
        H ^= H >> 33; H *= 0xff51afd7ed558ccdull; H ^= H >> 33; H *= 0xc4ceb9fe1a85ec53ull; H ^= H >> 33;
        return (double)(H >> 11) * (1.0 / 9007199254740992.0);
    }

    /** Set's table: Rows(Mips) rows of Columns texels of 4 floats (R G B A), row-major. */
    inline void Build(const VoxelNoiseTiles::FSet& Set, std::vector<float>& Out)
    {
        const int NR = Rows(Set.Mips);
        Out.assign((size_t)NR * Columns * 4, 0.0f);
        const FSampler Sampler { Set };
        std::vector<uint32_t> Hist[4];
        for (int c = 0; c < 4; c++) Hist[c].resize(65536);
        std::vector<double> P0(65537), P1(65537), P2(65537), P3(65537);
        std::vector<double> Lace;
        for (int r = 0; r < NR; r++)
        {
            const double Lod = 0.5 * r;
            // Stratified over the whole tile (its average is the surface's): a grid of twice the
            // level's texels a side (at least 64, at most 1024), a point at random in each cell.
            const uint64_t S = (uint64_t)(Set.N >> (int)Lod);
            const uint64_t G = std::min<uint64_t>(std::max<uint64_t>(2 * S, 64), 1024);
            const uint64_t Count = G * G;
            for (int c = 0; c < 4; c++) std::fill(Hist[c].begin(), Hist[c].end(), 0u);
            auto BinOf = [](double V) { return (size_t)std::min(std::max((long)std::lround(V * 65535.0), 0L), 65535L); };
            Lace.resize(Count);
            for (uint64_t n = 0; n < Count; n++)
            {
                double V[4];
                const uint64_t H = n * 2 + (uint64_t)r * 0x9e3779b97f4a7c15ull;
                Sampler.Trilinear(Lod, ((double)(n % G) + Unit(H + 1)) / (double)G, ((double)(n / G) + Unit(H + 2)) / (double)G, V);
                for (int c = 0; c < 3; c++) Hist[c][BinOf(V[c])]++;
                Lace[n] = V[0];
            }
            // A: the lace's even mix with an independent sample (another stratified point, shifted
            // by a random offset over the tile), about the lace's median.
            double Median = 0.0;
            {
                uint64_t Below = 0;
                for (int b = 0; b < 65536; b++) { Below += Hist[0][b]; if (2 * Below >= Count) { Median = b / 65535.0; break; } }
            }
            const double Ox = Unit((uint64_t)r * 0x5851f42d4c957f2dull + 11), Oy = Unit((uint64_t)r * 0x5851f42d4c957f2dull + 12);
            for (uint64_t n = 0; n < Count; n++)
            {
                double V[4];
                const uint64_t H = n * 2 + (uint64_t)r * 0xbf58476d1ce4e5b9ull + 7;
                const uint64_t m = (n * 0x9e3779b1ull + 0x7f4a7c15ull) % Count;     // a scrambled cell
                Sampler.Trilinear(Lod, ((double)(m % G) + Unit(H + 1)) / (double)G + Ox, ((double)(m / G) + Unit(H + 2)) / (double)G + Oy, V);
                Hist[3][BinOf(Median + ((Lace[n] - Median) + (V[0] - Median)) * 0.70710678118654752)]++;
            }
            for (int c = 0; c < 4; c++)
            {
                // Prefix sums of the counts and of v, v^2, v^3.
                P0[0] = P1[0] = P2[0] = P3[0] = 0.0;
                for (int b = 0; b < 65536; b++)
                {
                    const double v = b / 65535.0, h = Hist[c][b];
                    P0[b + 1] = P0[b] + h; P1[b + 1] = P1[b] + h * v; P2[b + 1] = P2[b] + h * v * v; P3[b + 1] = P3[b] + h * v * v * v;
                }
                auto Quantile = [&](double Q) -> double
                {
                    const double Rank = Q * (double)(Count - 1);
                    int b = (int)(std::lower_bound(P0.begin() + 1, P0.end(), Rank + 0.5) - (P0.begin() + 1));
                    return std::min(b, 65535) / 65535.0;
                };
                const double Spread = std::max(Quantile(0.9) - Quantile(0.1), 1.0e-4);
                const double W = Soft[c] * Spread;
                // The soft cover's mean at threshold T: the values above T + W count whole; those in
                // [T - W, T + W] by the smoothstep, a cubic in v.
                auto Bin = [](double v) { return (int)std::min(std::max(std::ceil(v * 65535.0 - 1e-9), 0.0), 65536.0); };
                auto Cover = [&](double T) -> double
                {
                    const int Lo = Bin(T - W), Hi = Bin(T + W);
                    const double Above = P0[65536] - P0[Hi];
                    const double a = 1.0 / (2.0 * W), b0 = (W - T) / (2.0 * W);
                    // t = a v + b0; s = 3 t^2 - 2 t^3.
                    const double S0 = P0[Hi] - P0[Lo], S1 = P1[Hi] - P1[Lo], S2 = P2[Hi] - P2[Lo], S3 = P3[Hi] - P3[Lo];
                    const double Et = a * S1 + b0 * S0;
                    const double Et2 = a * a * S2 + 2.0 * a * b0 * S1 + b0 * b0 * S0;
                    const double Et3 = a * a * a * S3 + 3.0 * a * a * b0 * S2 + 3.0 * a * b0 * b0 * S1 + b0 * b0 * b0 * S0;
                    (void)Et;
                    return (Above + 3.0 * Et2 - 2.0 * Et3) / (double)Count;
                };
                for (int k = 0; k < Knots; k++)
                {
                    const double Target = 1.0 - KnotQuantile(k);
                    double Lo = -2.0 * W, Hi = 1.0 + 2.0 * W;   // Cover(Lo) = 1, Cover(Hi) = 0
                    for (int it = 0; it < 48; it++)
                    {
                        const double Mid = 0.5 * (Lo + Hi);
                        if (Cover(Mid) > Target) Lo = Mid; else Hi = Mid;
                    }
                    Out[((size_t)r * Columns + k) * 4 + c] = (float)(0.5 * (Lo + Hi));
                }
                Out[((size_t)r * Columns + (Columns - 1)) * 4 + c] = (float)Spread;
            }
        }
    }
}
