// lace_check.cpp - The foam's lace (stage W3f; VoxelOceanFoamLace.ush) compiled as C++ through
// VoxelShaderCompat.h:
// 1. The patterns' distributions, and the thresholds' tables (VoxelOceanFoamLaceFit.ush, written by
//    --fit): a layer keeps the share it is given, within 1%.
// 2. The lace keeps the density: its mean over the surface is the density at every footprint, from
//    a millimetre (every layer drawn) to ten metres (none), within 4% (and 0.005).
// 3. Previews (PNG, beside the binary or in the folder given): the lace over a density ramp and
//    over a whitecap with its trail, near (1 cm a pixel) and far (25 cm a pixel).
//
//   g++ -O2 -std=c++20 -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Shaders/Private lace_check.cpp -o lace_check
//   ./lace_check [preview folder]      (the previews need ImageMagick's convert, else .pgm files stay)
//   ./lace_check --fit <plugin>/Shaders/Private/VoxelOceanFoamLaceFit.ush   (writes the thresholds' tables)

#include "VoxelShaderCompat.h"
namespace VoxelShader
{
#include "VoxelOceanFoamLace.ush"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

using namespace VoxelShader;

static int Failures = 0;
static void Expect(bool bOk, const char* What)
{
    if (!bOk)
    {
        std::printf("FAIL: %s\n", What);
        Failures++;
    }
}

// Least squares through the origin: T(d) = sum c_k d^k, k = 1..5.
static void Fit(const std::vector<double>& D, const std::vector<double>& T, double C[5])
{
    double A[5][6] = {};
    for (size_t i = 0; i < D.size(); i++)
    {
        double P[5];
        for (int k = 0; k < 5; k++) P[k] = std::pow(D[i], k + 1);
        for (int r = 0; r < 5; r++)
        {
            for (int c = 0; c < 5; c++) A[r][c] += P[r] * P[c];
            A[r][5] += P[r] * T[i];
        }
    }
    for (int r = 0; r < 5; r++)
    {
        int Pivot = r;
        for (int q = r + 1; q < 5; q++) if (std::fabs(A[q][r]) > std::fabs(A[Pivot][r])) Pivot = q;
        for (int c = 0; c < 6; c++) std::swap(A[r][c], A[Pivot][c]);
        for (int q = 0; q < 5; q++)
        {
            if (q == r) continue;
            const double F = A[q][r] / A[r][r];
            for (int c = 0; c < 6; c++) A[q][c] -= F * A[r][c];
        }
    }
    for (int r = 0; r < 5; r++) C[r] = A[r][5] / A[r][r];
}

static void Save(const std::string& Path, int W, int H, const std::vector<float>& Pixels)
{
    const std::string Pgm = Path + ".pgm";
    FILE* F = std::fopen(Pgm.c_str(), "wb");
    if (!F) return;
    std::fprintf(F, "P5 %d %d 255\n", W, H);
    for (float V : Pixels)
    {
        const unsigned char B = (unsigned char)std::lround(255.0 * std::clamp(V, 0.0f, 1.0f));
        std::fwrite(&B, 1, 1, F);
    }
    std::fclose(F);
    const std::string Cmd = "convert '" + Pgm + "' '" + Path + ".png' 2>/dev/null && rm -f '" + Pgm + "'";
    if (std::system(Cmd.c_str()) == 0) std::printf("  wrote %s.png\n", Path.c_str());
}

// A colour preview (r, g, b in [0, 1] each).
static void SaveColor(const std::string& Path, int W, int H, const std::vector<float>& Rgb)
{
    const std::string Ppm = Path + ".ppm";
    FILE* F = std::fopen(Ppm.c_str(), "wb");
    if (!F) return;
    std::fprintf(F, "P6 %d %d 255\n", W, H);
    for (float V : Rgb)
    {
        // (A display gamma.)
        const unsigned char B = (unsigned char)std::lround(255.0 * std::pow(std::clamp(V, 0.0f, 1.0f), 1.0f / 2.2f));
        std::fwrite(&B, 1, 1, F);
    }
    std::fclose(F);
    const std::string Cmd = "convert '" + Ppm + "' '" + Path + ".png' 2>/dev/null && rm -f '" + Ppm + "'";
    if (std::system(Cmd.c_str()) == 0) std::printf("  wrote %s.png\n", Path.c_str());
}

// The pass's foam shading (VoxelOceanSurfacePass.usf), simplified for a flat sea seen from above: the
// relief from the structure's finite differences, the light wrapped, the gaps darker, thin foam
// translucent, over a plain water colour.
static void Shade(const std::vector<VoxelShader::FVoxelOceanLace>& L, const std::vector<float>& Density, int W, int H, float PixelM, std::vector<float>& Rgb)
{
    Rgb.assign(3 * W * H, 0.0f);
    const float Water[3] = { 0.012f, 0.045f, 0.075f };
    const float Sun[3] = { 0.55f, 0.52f, 0.47f };
    const float Sky[3] = { 0.18f, 0.21f, 0.25f };
    const float Lx = 0.55f, Ly = -0.3f, Lz = 0.78f;
    for (int j = 0; j < H; j++)
        for (int i = 0; i < W; i++)
        {
            const int k = j * W + i;
            auto Relief = [&](int a, int b)
            {
                a = std::clamp(a, 0, W - 1); b = std::clamp(b, 0, H - 1);
                const auto& X = L[b * W + a];
                return X.Foam * (0.5f + 0.5f * X.Thick) * 2.0f;   // cm
            };
            const float GX = (Relief(i + 1, j) - Relief(i - 1, j)) / (2.0f * PixelM * 100.0f);
            const float GY = (Relief(i, j + 1) - Relief(i, j - 1)) / (2.0f * PixelM * 100.0f);
            const float NL = std::sqrt(GX * GX + GY * GY + 1.0f);
            const float NdL = (-GX * Lx - GY * Ly + Lz) / NL;
            const float Wrap = std::clamp((NdL + 0.35f) / 1.35f, 0.0f, 1.0f);
            const auto& X = L[k];
            const float Gaps = 0.7f + 0.3f * X.Thick;
            const float T = std::clamp(X.Thick / 0.6f, 0.0f, 1.0f);
            const float Smooth = T * T * (3.0f - 2.0f * T);
            const float Opacity = std::clamp(X.Foam * (0.55f + 0.45f * Smooth) * (0.8f + 0.2f * std::clamp(Density[k], 0.0f, 1.0f)), 0.0f, 1.0f);
            for (int c = 0; c < 3; c++)
            {
                const float Foam = 0.85f * (Sun[c] * Wrap + Sky[c] * Gaps);
                Rgb[3 * k + c] = Water[c] + (Foam - Water[c]) * Opacity;
            }
        }
}

int main(int Argc, char** Argv)
{
    const bool bFit = Argc > 2 && std::string(Argv[1]) == "--fit";
    const std::string FitPath = bFit ? Argv[2] : "";
    const std::string Folder = !bFit && Argc > 1 ? Argv[1] : ".";
    std::mt19937_64 Rng(7);
    std::uniform_real_distribution<double> U01(0.0, 1.0);

    // -- 1. The patterns' distributions, and the thresholds' tables ------------------------------
    {
        const int Samples = 2000000;
        std::vector<float> Streak(Samples), Wall(Samples);
        for (int i = 0; i < Samples; i++)
        {
            const float U = (float)(U01(Rng) * VOXEL_OCEAN_LACE_PERIOD);
            const float V = (float)(U01(Rng) * VOXEL_OCEAN_LACE_PERIOD);
            const float T = (float)(U01(Rng) * 3600.0);
            Streak[i] = VoxelOceanLaceStreak(U, V, 12345u, T);
            Wall[i] = VoxelOceanLaceWall(U, V, 12345u, T, VOXEL_OCEAN_LACE_RATE1);
        }
        std::sort(Streak.begin(), Streak.end());
        std::sort(Wall.begin(), Wall.end());
        auto Quantile = [Samples](const std::vector<float>& E, double D)
        {
            return E[std::min<size_t>((size_t)std::llround(D * (Samples - 1)), Samples - 1)];
        };
        if (bFit)
        {
            FILE* F = std::fopen(FitPath.c_str(), "wb");
            if (!F) { std::printf("can't write %s\n", FitPath.c_str()); return 1; }
            std::fprintf(F, "// VoxelOceanFoamLaceFit.ush - Generated by Tools/OceanHarness/lace_check --fit (stage W3f): the foam\n"
                "// structure's thresholds (VoxelOceanFoamLace.ush), the inverse of each pattern's distribution\n"
                "// (2,000,000 samples) at D = sin^2(pi k / 128) (closer toward both ends), interpolated. Write it again when the patterns change.\n\n");
            auto Table = [&](const char* Name, const char* What, const std::vector<float>& E)
            {
                std::fprintf(F, "// %s\nVOXEL_FUNC float %s(float D)\n{\n    const float K[65] = {", What, Name);
                for (int k = 0; k <= 64; k++)
                    std::fprintf(F, "%s%s%.6ff", k ? "," : "", k % 6 == 0 ? "\n        " : " ", Quantile(E, std::pow(std::sin(0.5 * M_PI * k / 64.0), 2.0)));
                std::fprintf(F, " };\n    // (The knots at D = sin^2(pi k / 128): closer toward both ends.)\n    const float S = sqrt(saturate(D));\n    const float X = atan2(S, sqrt(max(1.0f - S * S, 0.0f))) * (128.0f / 3.14159265f);\n    const int I = min(int(X), 63);\n"
                    "    return lerp(K[I], K[I + 1], X - float(I));\n}\n\n");
            };
            Table("VoxelOceanLaceThreshold0", "The streaks' pattern (VoxelOceanLaceStreak).", Streak);
            Table("VoxelOceanLaceThreshold1", "The cell walls' pattern (VoxelOceanLaceWall).", Wall);
            std::fclose(F);
            std::printf("wrote %s\n", FitPath.c_str());
            return 0;
        }
        std::printf("1. the patterns: streaks %.3f to %.3f (median %.3f), walls %.3f to %.3f (median %.3f)\n", Streak.front(), Streak.back(),
            Streak[Samples / 2], Wall.front(), Wall.back(), Wall[Samples / 2]);
        double Worst = 0.0, WorstD = 0.0;
        int WorstWhich = -1;
        for (int k = 1; k < 200; k++)
        {
            const double Dk = k / 200.0;
            const double K0 = double(std::lower_bound(Streak.begin(), Streak.end(), VoxelOceanLaceThreshold0((float)Dk)) - Streak.begin()) / Samples;
            const double K1 = double(std::lower_bound(Wall.begin(), Wall.end(), VoxelOceanLaceThreshold1((float)Dk)) - Wall.begin()) / Samples;
            const double E = std::max(std::fabs(K0 - Dk), std::fabs(K1 - Dk));
            if (E > Worst) { Worst = E; WorstD = Dk; WorstWhich = std::fabs(K0 - Dk) > std::fabs(K1 - Dk) ? 0 : 1; }
        }
        std::printf("   the tables keep their shares within %.4f (worst at %.3f, pattern %d)\n", Worst, WorstD, WorstWhich);
        Expect(Worst < 0.01, "a layer keeps its share within 1%");
    }

    // -- 1b. No seam where a pattern's period wraps ------------------------------------------------
    {
        double Jump = 0.0;
        const float P = float(VOXEL_OCEAN_LACE_PERIOD);
        for (int k = 0; k < 2000; k++)
        {
            const float V = (float)(U01(Rng) * P);
            const float T = (float)(U01(Rng) * 3600.0);
            const float E = 1.0e-3f;
            Jump = std::max(Jump, (double)std::fabs(VoxelOceanLaceWall(P - E, V, 77u, T, VOXEL_OCEAN_LACE_RATE1) - VoxelOceanLaceWall(0.0f, V, 77u, T, VOXEL_OCEAN_LACE_RATE1)));
            Jump = std::max(Jump, (double)std::fabs(VoxelOceanLaceWall(V, P - E, 77u, T, VOXEL_OCEAN_LACE_RATE1) - VoxelOceanLaceWall(V, 0.0f, 77u, T, VOXEL_OCEAN_LACE_RATE1)));
            Jump = std::max(Jump, (double)std::fabs(VoxelOceanLaceStreak(P - E, V, 77u, T) - VoxelOceanLaceStreak(0.0f, V, 77u, T)));
            Jump = std::max(Jump, (double)std::fabs(VoxelOceanLaceStreak(V, P - E, 77u, T) - VoxelOceanLaceStreak(V, 0.0f, 77u, T)));
        }
        std::printf("1b. across the period's wrap the patterns change by at most %.4f\n", Jump);
        Expect(Jump < 0.02, "no seam where the patterns' period wraps");
    }

    // -- 2. The density kept at every footprint -------------------------------------------------------
    {
        std::printf("2. the lace's mean over the surface (density by footprint)\n   density");
        const double Feet[] = { 0.001, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0 };
        for (double F : Feet) std::printf("  %6.3fm", F);
        std::printf("\n");
        double Worst = 0.0;
        for (double Dn : { 0.02, 0.1, 0.3, 0.6, 0.9 })
        {
            std::printf("   %5.2f  ", Dn);
            for (double F : Feet)
            {
                double Sum = 0.0;
                const int N = 200000;
                for (int i = 0; i < N; i++)
                {
                    const float X = (float)(U01(Rng) * 5000.0);
                    const float Y = (float)(U01(Rng) * 5000.0);
                    Sum += VoxelOceanLaceAt(0.0f, 0.0f, X, Y, (float)Dn, (float)F, (float)(U01(Rng) * 3600.0), 99u).Foam;
                }
                const double Mean = Sum / N;
                std::printf("  %7.4f", Mean);
                Worst = std::max(Worst, std::fabs(Mean - Dn) / std::max(Dn, 0.125));
            }
            std::printf("\n");
        }
        std::printf("   worst relative error %.3f\n", Worst);
        Expect(Worst < 0.04, "the lace keeps the density within 4% at every footprint");
    }

    // -- 3. Previews -------------------------------------------------------------------------------
    {
        std::printf("3. previews\n");
        // A density ramp from 0 (left) to 1 (right), 8 m by 3 m, a centimetre a pixel.
        {
            const int W = 800, H = 300;
            std::vector<float> Px(W * H);
            for (int j = 0; j < H; j++)
                for (int i = 0; i < W; i++)
                {
                    const float Dn = (float)i / (W - 1);
                    Px[j * W + i] = VoxelOceanLaceAt(0.0f, 0.0f, i * 0.01f, j * 0.01f, Dn, 0.01f, 100.0f, 7u).Foam;
                }
            Save(Folder + "/lace_ramp_near", W, H, Px);
        }
        // A whitecap and its trail (downwind: +x): fresh at the crest, fading behind it; near (1 cm a
        // pixel, 10 x 6 m) and far (25 cm a pixel, the same field scaled 25 times: a 250 x 150 m patch
        // of such caps).
        auto Cap = [](float X, float Y)
        {
            // Caps on a jittered 20 m lattice, each a crest across the wind with a trail behind it.
            const float CX = std::floor(X / 20.0f), CY = std::floor(Y / 14.0f);
            float Best = 0.0f;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    const uint H = VoxelOceanLaceHash((uint)(int)(CX + dx + 1000), (uint)(int)(CY + dy + 1000), 5u);
                    const float PX = (CX + dx) * 20.0f + 20.0f * VoxelOceanLaceUnit(H);
                    const float PY = (CY + dy) * 14.0f + 14.0f * VoxelOceanLaceUnit(H >> 8u);
                    const float Size = 0.5f + VoxelOceanLaceUnit(H >> 16u);
                    const float DX = (X - PX) / Size, DY = (Y - PY) / Size;
                    // The crest at DX 0 (2.5 m deep, 4 m across), the trail behind it (DX < 0) over 7 m.
                    const float Across = std::exp(-DY * DY / (2.0f * 1.6f * 1.6f));
                    const float Along = DX >= 0.0f ? std::exp(-DX * DX / (2.0f * 0.7f * 0.7f)) : std::exp(DX / 3.5f);
                    Best = std::max(Best, std::min(1.0f, 1.3f * Across * Along));
                }
            return Best;
        };
        {
            const int W = 1000, H = 600;
            std::vector<float> Px(W * H), Plain(W * H);
            std::vector<VoxelShader::FVoxelOceanLace> Laces(W * H);
            for (int j = 0; j < H; j++)
                for (int i = 0; i < W; i++)
                {
                    const float X = 9.0f + i * 0.01f, Y = 5.0f + j * 0.01f;
                    const float Dn = Cap(X, Y);
                    Laces[j * W + i] = VoxelOceanLaceAt(0.0f, 0.0f, X, Y, Dn, 0.01f, 100.0f, 7u);
                    Px[j * W + i] = Laces[j * W + i].Foam;
                    Plain[j * W + i] = Dn;
                }
            Save(Folder + "/lace_cap_near", W, H, Px);
            Save(Folder + "/lace_cap_near_density", W, H, Plain);
            std::vector<float> Rgb;
            Shade(Laces, Plain, W, H, 0.01f, Rgb);
            SaveColor(Folder + "/lace_cap_near_shaded", W, H, Rgb);
        }
        {
            const int W = 1000, H = 600;
            std::vector<float> Px(W * H), Plain(W * H);
            std::vector<VoxelShader::FVoxelOceanLace> Laces(W * H);
            for (int j = 0; j < H; j++)
                for (int i = 0; i < W; i++)
                {
                    const float X = i * 0.25f, Y = j * 0.25f;
                    const float Dn = Cap(X, Y);
                    Laces[j * W + i] = VoxelOceanLaceAt(0.0f, 0.0f, X, Y, Dn, 0.25f, 100.0f, 7u);
                    Px[j * W + i] = Laces[j * W + i].Foam;
                    Plain[j * W + i] = Dn;
                }
            Save(Folder + "/lace_cap_far", W, H, Px);
            Save(Folder + "/lace_cap_far_density", W, H, Plain);
            std::vector<float> Rgb;
            Shade(Laces, Plain, W, H, 0.25f, Rgb);
            SaveColor(Folder + "/lace_cap_far_shaded", W, H, Rgb);
        }
    }

    std::printf(Failures == 0 ? "lace_check: all passed\n" : "lace_check: %d failed\n", Failures);
    return Failures == 0 ? 0 : 1;
}
