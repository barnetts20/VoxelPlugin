// foam_set_check.cpp - The foam's texture set (stage W3g; VoxelOceanFoamTex.ush, VoxelOceanFoamTable.h)
// as C++: reads a foam .vxnt (Tools/NoiseTileTool/foam_tile.py), builds its quantiles' table as the
// plugin does, and runs the shader's cover (ported line for line, the hardware's trilinear sampling
// emulated) to check:
//   1. the table: each channel's quantiles rise, per mip;
//   2. the cover keeps the density: over the surface, the lace's and the streaks' covers average D
//      and the whitecaps' Dw, at every level from the finest to the coarsest drawn, for densities
//      from 0.02 to 0.95; stage W3m: the lace mixed with its stretched sample (the streaks) at
//      shares from 0.2 to 0.8, and every cover magnified (the cubic filter);
//   3. previews (PNG): the foam near (1.5 cm a pixel) and far (8 cm a pixel) over a whitecap map,
//      shaded as the own pass shades it (flat light), and the layers.
//
//   g++ -O2 -std=c++20 -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private foam_set_check.cpp -o foam_set_check
//   ./foam_set_check <set.vxnt> [preview folder]

#include "VoxelNoiseTiles.h"
#include "VoxelOceanFoamTable.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <vector>

namespace
{
    int Failures = 0;
    void Check(bool bOk, const char* What, double Value, double Limit)
    {
        std::printf("  [%s] %-80s %g (limit %g)\n", bOk ? "ok" : "FAIL", What, Value, Limit);
        if (!bOk) Failures++;
    }

    struct F4 { float x, y, z, w; };
    F4 Lerp(F4 A, F4 B, float T) { return { A.x + (B.x - A.x) * T, A.y + (B.y - A.y) * T, A.z + (B.z - A.z) * T, A.w + (B.w - A.w) * T }; }
    float Smooth(float A, float B, float X) { const float T = std::clamp((X - A) / (B - A), 0.0f, 1.0f); return T * T * (3.0f - 2.0f * T); }
    float Sat(float X) { return std::clamp(X, 0.0f, 1.0f); }

    const VoxelNoiseTiles::FSet* GSet = nullptr;
    std::vector<float> GTable;

    // The texture's texel, normalised (RGBA), wrapped.
    F4 SampleLevel(double U, double V, float Lod)
    {
        double O[4];
        VoxelOceanFoamTable::FSampler { *GSet }.Trilinear(Lod, U, V, O);
        return { (float)O[0], (float)O[1], (float)O[2], (float)O[3] };
    }
    // Stage W3m: magnified, the shader's cubic B-spline from four bilinear taps at the finest level.
    F4 SampleCubic(double U, double V)
    {
        const double Texels = (double)GSet->N;
        const double X[2] = { U * Texels - 0.5, V * Texels - 0.5 };
        double H0[2], H1[2], G0[2], G1[2];
        for (int a = 0; a < 2; a++)
        {
            const double I = std::floor(X[a]), T = X[a] - I, T2 = T * T, T3 = T2 * T;
            const double W0 = (1 - 3 * T + 3 * T2 - T3) / 6, W1 = (4 - 6 * T2 + 3 * T3) / 6, W2 = (1 + 3 * T + 3 * T2 - 3 * T3) / 6, W3 = T3 / 6;
            G0[a] = W0 + W1; G1[a] = W2 + W3;
            H0[a] = (I - 0.5 + W1 / G0[a]) / Texels; H1[a] = (I + 1.5 + W3 / G1[a]) / Texels;
        }
        const F4 A = SampleLevel(H0[0], H0[1], 0.0f), B = SampleLevel(H1[0], H0[1], 0.0f), C = SampleLevel(H0[0], H1[1], 0.0f), D = SampleLevel(H1[0], H1[1], 0.0f);
        auto Mix = [&](float a, float b, float c, float d) { return (float)(G0[1] * (G0[0] * a + G1[0] * b) + G1[1] * (G0[0] * c + G1[0] * d)); };
        return { Mix(A.x, B.x, C.x, D.x), Mix(A.y, B.y, C.y, D.y), Mix(A.z, B.z, C.z, D.z), Mix(A.w, B.w, C.w, D.w) };
    }
    F4 Sample(double U, double V, float Lod, bool bCubic) { return bCubic ? SampleCubic(U, V) : SampleLevel(U, V, Lod); }

    F4 Load(int Col, int Mip)
    {
        const float* P = GTable.data() + ((size_t)Mip * VoxelOceanFoamTable::Columns + Col) * 4;
        return { P[0], P[1], P[2], P[3] };
    }

    // --- VoxelOceanFoamTex.ush, ported ------------------------------------------------------------
    constexpr float CHURN_PERIOD = 1.6f, CHURN_SPEED = 0.6f, CHURN_TILE = 12.0f;
    constexpr float SOFT = 0.15f, FADE0 = 0.4f, FADE1 = 1.6f, CAP0 = 0.55f, CAP1 = 0.95f, OLD0 = 0.08f, OLD1 = 0.45f, STREAK = 0.4f, GRAIN = 0.2f;
    constexpr float STREAK_STRETCH = 4.0f, WIND_STREAK0 = 12.0f, WIND_STREAK1 = 22.0f, WIND_STREAK = 0.8f;

    F4 Quantile(int Mip, float Q)
    {
        const float K = std::clamp((float)std::asin(std::sqrt(Sat(Q))) * (128.0f / 3.14159265f), 0.0f, 64.0f);
        const int K0 = std::min((int)K, 63);
        return Lerp(Load(K0, Mip), Load(K0 + 1, Mip), K - (float)K0);
    }
    F4 Threshold(int M0, int M1, float F, float Q) { return Lerp(Quantile(M0, Q), Quantile(M1, Q), F); }

    struct FLace { float Foam, Layer0, Layer1, Layer2, Thick, Mixed; };

    // At tile coordinates (U, V) and level Lod (the shader's Lod from the footprint).
    // Whitecap: the whitecaps' own density (stage W3h), or negative for the foam's dense part.
    double Jitter(uint64_t K, uint64_t Salt)
    {
        uint64_t H = K * 0x9e3779b97f4a7c15ull + Salt * 0xbf58476d1ce4e5b9ull;
        H ^= H >> 31; H *= 0x94d049bb133111ebull; H ^= H >> 29;
        return (double)(H >> 11) * (1.0 / 9007199254740992.0);
    }

    // Wind: the wind (m/s; stage W3i's storm streaks). OldOverride >= 0: the streaks' share itself.
    // bCubic: magnified (Lod 0, the cubic filter).
    FLace FoamAt(double U, double V, float Density, float Lod, float FootprintM, float Whitecap = -1.0f, float Time = -1.0f,
        float Wind = 0.0f, float OldOverride = -1.0f, bool bCubic = false)
    {
        FLace L { 0, 0, 0, 0, 0.5f, 0 };
        if (bCubic) Lod = 0.0f;
        const float D = Sat(Density);
        const float Dw = Whitecap >= 0.0f ? Sat(Whitecap) : D * Smooth(CAP0, CAP1, D);
        if (D <= 0.0f && Dw <= 0.0f) return L;
        const float Fade = Smooth(FADE0, FADE1, FootprintM);
        const int Mips = (int)GSet->Mips;
        const int LastRow = 2 * (Mips - 1);
        const float Row = 2.0f * Lod;
        const int M0 = std::min((int)Row, LastRow), M1 = std::min(M0 + 1, LastRow);
        const float Fm = Row - (float)M0;
        F4 P = Sample(U, V, Lod, bCubic);
        if (Dw > 0.0f && Time >= 0.0f)
        {
            // The whitecaps' churn, as the shader's (VoxelOceanFoamTex.ush), its jumps hashed differently.
            const double Cycle = Time / CHURN_PERIOD;
            const double C1 = std::floor(Cycle), C2 = std::floor(Cycle + 0.5);
            const double Ph1 = Cycle - C1, Ph2 = Cycle + 0.5 - C2;
            const float W1 = float(1.0 - std::fabs(2.0 * Ph1 - 1.0)), W2 = 1.0f - W1;
            const double Drift = CHURN_SPEED * CHURN_PERIOD / CHURN_TILE;
            const float Z1 = Sample(U + Jitter(uint64_t(C1), 1) + Drift * Ph1, V + Jitter(uint64_t(C1), 2), Lod, bCubic).z;
            const float Z2 = Sample(U + Jitter(uint64_t(C2) ^ 0x5555u, 1) + Drift * Ph2, V + Jitter(uint64_t(C2) ^ 0x5555u, 2), Lod, bCubic).z;
            const float Median = Threshold(M0, M1, Fm, 0.5f).z;
            P.z = Median + (W1 * (Z1 - Median) + W2 * (Z2 - Median)) / std::sqrt(std::max(W1 * W1 + W2 * W2, 1e-4f));
        }
        const F4 Q = Threshold(M0, M1, Fm, 1.0f - D);
        const F4 Qw = Threshold(M0, M1, Fm, 1.0f - Dw);
        F4 Spread = Lerp(Load(65, M0), Load(65, M1), Fm);
        Spread = { std::max(Spread.x, 1e-4f), std::max(Spread.y, 1e-4f), std::max(Spread.z, 1e-4f), std::max(Spread.w, 1e-4f) };
        const F4 W { SOFT * Spread.x, SOFT * Spread.y, SOFT * Spread.z, SOFT * Spread.w };
        const float CovL = Smooth(-W.x, W.x, P.x - Q.x);
        const float CovW = Dw > 0.0f ? Smooth(-W.z, W.z, P.z - Qw.z) : 0.0f;
        // Stage W3m: the streaks, the lace stretched downwind (U), mixed in by their share.
        const float Old = OldOverride >= 0.0f ? OldOverride
            : std::max((1.0f - Smooth(OLD0, OLD1, D)) * STREAK, WIND_STREAK * Smooth(WIND_STREAK0, WIND_STREAK1, Wind));
        const float PS = Sample(U / STREAK_STRETCH + 0.37, V + 0.61, Lod, bCubic).x;
        const float CovS = Smooth(-W.x, W.x, PS - Q.x);
        const float MedianL = Threshold(M0, M1, Fm, 0.5f).x;
        const float Mix = 1.0f - std::fabs(2.0f * Old - 1.0f);
        const float Zf = MedianL + ((1.0f - Old) * (P.x - MedianL) + Old * (PS - MedianL)) / std::sqrt(std::max((1.0f - Old) * (1.0f - Old) + Old * Old, 1e-4f));
        const float Qf = Q.x + (Q.w - Q.x) * Mix;
        const float Sf = Spread.x + (Spread.w - Spread.x) * Mix;
        const float CovF = Smooth(-SOFT * Sf, SOFT * Sf, Zf - Qf);
        L.Mixed = CovF;
        const float Above = (Zf - Qf) / Sf;
        const float Grain = GRAIN * (P.y - Q.y) / Spread.y;
        float Thick = Sat(0.6f * Above + 0.2f + Grain);
        const float ThickW = Sat(0.45f + 0.6f * (P.z - Qw.z) / Spread.z + Grain);
        Thick = Thick + (ThickW - Thick) * CovW;
        L.Layer0 = CovL + (D - CovL) * Fade;
        L.Layer1 = CovS + (D - CovS) * Fade;
        L.Layer2 = CovW + (Dw - CovW) * Fade;
        L.Foam = std::max(CovF + (D - CovF) * Fade, L.Layer2);
        L.Thick = Thick + (0.5f - Thick) * Fade;
        return L;
    }

    void SaveColor(const std::string& Path, int W, int H, const std::vector<float>& Rgb)
    {
        const std::string Ppm = Path + ".ppm";
        FILE* F = std::fopen(Ppm.c_str(), "wb");
        if (!F) return;
        std::fprintf(F, "P6 %d %d 255\n", W, H);
        for (float V : Rgb)
        {
            const unsigned char B = (unsigned char)std::lround(255.0 * std::pow(std::clamp(V, 0.0f, 1.0f), 1.0f / 2.2f));
            std::fwrite(&B, 1, 1, F);
        }
        std::fclose(F);
        const std::string Cmd = "convert '" + Ppm + "' '" + Path + ".png' 2>/dev/null && rm -f '" + Ppm + "'";
        if (std::system(Cmd.c_str()) == 0) std::printf("  wrote %s.png\n", Path.c_str());
    }

    // A whitecap map (m): crests solid at their heads, trails thinning downwind (+x), as preview.py.
    struct FCap { double X, Y, L, Wd; };
    float DensityAt(const std::vector<FCap>& Caps, double X, double Y)
    {
        float D = 0.0f;
        for (const FCap& C : Caps)
        {
            const double Dx = X - C.X, Dy = Y - C.Y;
            const double Prof = std::exp(-std::pow(Dy / (C.Wd * (1.0 + 2.0 * std::max(Dx, 0.0) / C.L)), 2.0));
            const double Along = Dx < 0.0 ? std::exp(-std::pow(Dx / (0.08 * C.L), 2.0)) : std::exp(-2.2 * Dx / C.L);
            D = std::max(D, (float)(Along * Prof));
        }
        return Sat(D * 1.1f);
    }

    // The own pass's foam over water, flat-lit (its relief left out).
    void Render(const std::string& Path, double PixelM, int Size, double TileM, const std::vector<FCap>& Caps, float Wind = 0.0f)
    {
        std::vector<float> Rgb((size_t)Size * Size * 3), Layers((size_t)Size * Size * 3);
        const float Texels = (float)GSet->N;
        const float LodRaw = (float)std::log2(std::max(PixelM / TileM * Texels, 1e-6));
        const float Lod = std::clamp(LodRaw, 0.0f, (float)GSet->Mips - 1);
        const float Water[3] = { 0.03f, 0.22f, 0.42f };
        for (int j = 0; j < Size; j++)
            for (int i = 0; i < Size; i++)
            {
                const double X = (i + 0.5) * PixelM, Y = (j + 0.5) * PixelM;
                const float D = DensityAt(Caps, X, Y);
                const FLace L = FoamAt(X / TileM, Y / TileM, D, Lod, (float)PixelM, -1.0f, -1.0f, Wind, -1.0f, LodRaw < 0.0f);
                // As the pass: the haze, thin foam translucent, whitecaps opaque and brighter.
                const float Light = 0.85f;
                float Opacity = L.Foam * (0.55f + 0.45f * Smooth(0.0f, 0.6f, L.Thick)) * (0.8f + 0.2f * D);
                Opacity = std::max(Opacity, L.Layer2);
                const float Gain = 1.0f + 0.15f * L.Layer2;
                for (int c = 0; c < 3; c++)
                {
                    float Col = Water[c] + (Light - Water[c]) * 0.3f * D * (1.0f - L.Foam);
                    Col = Col + (Light * Gain * (0.8f + 0.2f * L.Thick) - Col) * Sat(Opacity);
                    Rgb[((size_t)j * Size + i) * 3 + c] = Col;
                }
                Layers[((size_t)j * Size + i) * 3 + 0] = L.Layer0;
                Layers[((size_t)j * Size + i) * 3 + 1] = L.Layer1;
                Layers[((size_t)j * Size + i) * 3 + 2] = L.Layer2;
            }
        SaveColor(Path, Size, Size, Rgb);
        SaveColor(Path + "_layers", Size, Size, Layers);
    }
}

int main(int Argc, char** Argv)
{
    if (Argc < 2) { std::printf("usage: foam_set_check <set.vxnt> [preview folder]\n"); return 2; }
    std::ifstream In(Argv[1], std::ios::binary);
    std::vector<uint8_t> Bytes((std::istreambuf_iterator<char>(In)), std::istreambuf_iterator<char>());
    VoxelNoiseTiles::FSet Set;
    std::string Why = VoxelNoiseTiles::ReadVxnt(Bytes.data(), Bytes.size(), Set);
    if (Why.empty()) Why = VoxelNoiseTiles::CheckFoam(Set);
    if (!Why.empty()) { std::printf("%s: %s\n", Argv[1], Why.c_str()); return 1; }
    GSet = &Set;
    VoxelOceanFoamTable::Build(Set, GTable);
    std::printf("foam set %s: %u^2, %u mips\n", Set.Name.c_str(), Set.N, Set.Mips);

    // 1. The table rises.
    {
        int Bad = 0;
        for (int r = 0; r < VoxelOceanFoamTable::Rows(Set.Mips); r++)
            for (int k = 1; k < VoxelOceanFoamTable::Knots; k++)
            {
                const F4 A = Load(k - 1, r), B = Load(k, r);
                if (B.x < A.x || B.y < A.y || B.z < A.z || B.w < A.w) Bad++;
            }
        Check(Bad == 0, "every channel's thresholds rise with the quantile, at every level", Bad, 0);
    }

    // 2. The covers keep the density (the footprint under the fade's start).
    {
        std::mt19937_64 Rng(7);
        std::uniform_real_distribution<double> U01(0.0, 1.0);
        const float Densities[] = { 0.02f, 0.1f, 0.3f, 0.6f, 0.95f };
        const int TopLod = std::max((int)Set.Mips - 5, 0);
        double WorstL = 0.0, WorstS = 0.0, WorstW = 0.0, MaxExcess = 0.0;
        for (float D : Densities)
            for (int Lod2 = 0; Lod2 <= 2 * TopLod; Lod2++)
            {
                const float Lod = 0.5f * (float)Lod2;
                double SL = 0, SS = 0, SW = 0, SF = 0;
                // Stratified over the tile (the surface's average), offset from the table's points;
                // over STREAK_STRETCH tiles along U, so the stretched sample covers its tile too.
                const int G = 256, Count = G * G;
                for (int n = 0; n < Count; n++)
                {
                    const FLace L = FoamAt(STREAK_STRETCH * ((n % G) + U01(Rng)) / G, ((n / G) + U01(Rng)) / G, D, Lod, 0.0f);
                    SL += L.Layer0; SS += L.Layer1; SW += L.Layer2; SF += L.Foam;
                }
                const float Dw = D * Smooth(CAP0, CAP1, D);
                const double Scale = std::max((double)D, 0.05);
                if (std::getenv("FOAM_VERBOSE")) std::printf("    D %.2f lod %.1f: lace %.4f streak %.4f whitecap %.4f (Dw %.4f)\n", D, Lod, SL / Count, SS / Count, SW / Count, Dw);
                WorstL = std::max(WorstL, std::abs(SL / Count - D) / Scale);
                WorstS = std::max(WorstS, std::abs(SS / Count - D) / Scale);
                if (Dw > 0.01f) WorstW = std::max(WorstW, std::abs(SW / Count - Dw) / std::max((double)Dw, 0.05));
                MaxExcess = std::max(MaxExcess, SF / Count - D);
            }
        Check(WorstL < 0.05, "the lace's cover averages the density (relative, every level drawn)", WorstL, 0.05);
        Check(WorstS < 0.05, "the streaks' cover averages the density", WorstS, 0.05);
        Check(WorstW < 0.08, "the whitecaps' cover averages their share", WorstW, 0.08);
        // Stage W3h: churning (two samples crossfaded, the blend's spread restored), at random
        // moments through the cycle: the whitecaps' cover still averages their share.
        double WorstChurn = 0.0;
        for (float Dw : { 0.05f, 0.2f, 0.6f })
            for (int Lod2 = 0; Lod2 <= 2 * TopLod; Lod2 += 2)
            {
                const int G = 192;
                double Sum = 0.0;
                for (int n = 0; n < G * G; n++)
                    Sum += FoamAt(STREAK_STRETCH * ((n % G) + U01(Rng)) / G, ((n / G) + U01(Rng)) / G, 0.0f, 0.5f * Lod2, 0.0f, Dw, float(100.0 * U01(Rng))).Layer2;
                WorstChurn = std::max(WorstChurn, std::fabs(Sum / (G * G) - Dw) / Dw);
            }
        Check(WorstChurn < 0.08, "churning, the whitecaps' cover still averages their share", WorstChurn, 0.08);
        // Stage W3m: the lace mixed with its stretched sample (the streaks) by their share.
        double WorstMix = 0.0;
        for (float O : { 0.2f, 0.5f, 0.8f })
            for (float D : Densities)
                for (int Lod2 = 0; Lod2 <= 2 * TopLod; Lod2 += 2)
                {
                    const int G = 192;
                    double Sum = 0.0;
                    for (int n = 0; n < G * G; n++)
                        Sum += FoamAt(STREAK_STRETCH * ((n % G) + U01(Rng)) / G, ((n / G) + U01(Rng)) / G, D, 0.5f * Lod2, 0.0f, -1.0f, -1.0f, 0.0f, O).Mixed;
                    const double Err = std::fabs(Sum / (G * G) - D) / std::max((double)D, 0.05);
                    if (std::getenv("FOAM_VERBOSE")) std::printf("    mix %.1f D %.2f lod %.1f: %.4f\n", O, D, 0.5f * Lod2, Sum / (G * G));
                    WorstMix = std::max(WorstMix, Err);
                }
        Check(WorstMix < 0.12, "the lace mixed with the streaks (shares 0.2-0.8) averages the density", WorstMix, 0.12);
        // Stage W3m: magnified (the cubic filter), the covers still average their densities.
        double WorstCubic = 0.0;
        for (float D : { 0.05f, 0.3f, 0.7f })
        {
            const int G = 256;
            double SL = 0.0, SW = 0.0;
            for (int n = 0; n < G * G; n++)
            {
                const FLace L = FoamAt(STREAK_STRETCH * ((n % G) + U01(Rng)) / G, ((n / G) + U01(Rng)) / G, D, 0.0f, 0.0f, D, -1.0f, 0.0f, 0.0f, true);
                SL += L.Layer0; SW += L.Layer2;
            }
            WorstCubic = std::max({ WorstCubic, std::fabs(SL / (G * G) - D) / D, std::fabs(SW / (G * G) - D) / D });
        }
        Check(WorstCubic < 0.08, "magnified (cubic), the lace's and whitecaps' covers average their densities (the B-spline narrows the values a little)", WorstCubic, 0.08);
        std::printf("  (the foam's cover, the lace and the whitecaps together, exceeds the density by at most %.3f)\n", MaxExcess);
    }

    // 3. Previews.
    if (Argc > 2)
    {
        const std::string Dir = Argv[2];
        const double TileM = Argc > 3 ? std::atof(Argv[3]) : 12.0;
        std::mt19937_64 Rng(11);
        std::uniform_real_distribution<double> U01(0.0, 1.0);
        auto CapsFor = [&](double Extent)
        {
            std::vector<FCap> Caps;
            for (int n = 0; n < 9; n++)
                Caps.push_back({ U01(Rng) * Extent, U01(Rng) * Extent, (0.25 + 0.35 * U01(Rng)) * Extent, (0.04 + 0.06 * U01(Rng)) * Extent });
            return Caps;
        };
        Render(Dir + "/foamset_near", 0.015, 1024, TileM, CapsFor(1024 * 0.015));
        Render(Dir + "/foamset_far", 0.08, 1024, TileM, CapsFor(1024 * 0.08));
        // Stage W3m: a storm (25 m/s: the streaks' full share) and close up (magnified, cubic).
        Render(Dir + "/foamset_storm", 0.04, 1024, TileM, CapsFor(1024 * 0.04), 25.0f);
        Render(Dir + "/foamset_close", 0.004, 768, TileM, CapsFor(768 * 0.004));
    }

    std::printf("%s: %d failure(s)\n", Failures ? "FAILED" : "PASSED", Failures);
    return Failures ? 1 : 0;
}
