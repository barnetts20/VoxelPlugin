// ledger_check.cpp - The slope-variance ledger (stage W6a; Design/DistantOcean.md, W6a and W6b): at
// every footprint, the slope variance the surface shows (its normals' spread over the pixels) plus
// the roughness it carries (the model's unresolved variance) against the sea state's total, on the
// emulated cascades (ocean_fft_emulation.h: VoxelOceanFFT.usf's passes in float, the GPU's 2 x 2
// box mips and trilinear lookups):
// 1. the bands' lookups (as VoxelOceanFFTChartLookups takes them: the normal's mip by the footprint):
//    each band's resolved variance against the model's unresolved (VoxelOceanFFTSlopeVariance);
// 2. the clipmap's chain (VoxelOceanClip.usf's texels, the normal mipped at the level's spacing, its
//    unresolved variance in the normal's w; VoxelOceanPixelClip.ush's pick and bilinear lookup, the
//    levels blended, the per-pixel detail under the first level's spacing): the pixel's ledger;
// 3. past the outermost level's spacing: the outermost level's texels under the pixel (one lookup);
// 4. the axes: along and across the wind, the total and the unresolved share.
// Measurement only (W6a): it reports, its checks the measurements' own consistency.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private -I. ledger_check.cpp -o ledger_check

#include "VoxelShaderCompat.h"
#include "ocean_fft_emulation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
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
    // A band's slope (eta_x, eta_y) and its box mips, as VoxelOceanFFT.usf's MipCS makes them.
    struct FChain
    {
        int N0 = 0;
        int TopMip = 0;
        std::vector<std::vector<float>> X, Y;
    };

    FChain BuildChain(const FGPUSlot& G)
    {
        FChain C;
        C.N0 = G.N;
        int N = G.N;
        std::vector<float> X(size_t(N) * N), Y(size_t(N) * N);
        for (size_t i = 0; i < X.size(); i++) { X[i] = G.Derivatives[i].x; Y[i] = G.Derivatives[i].y; }
        C.X.push_back(X);
        C.Y.push_back(Y);
        while (N > 1)
        {
            const int M = N / 2;
            std::vector<float> XM(size_t(M) * M), YM(size_t(M) * M);
            const std::vector<float>& XP = C.X.back();
            const std::vector<float>& YP = C.Y.back();
            for (int j = 0; j < M; j++)
                for (int i = 0; i < M; i++)
                {
                    auto At = [&](const std::vector<float>& T, int a, int b) { return T[size_t(b) * N + a]; };
                    XM[size_t(j) * M + i] = 0.25f * (At(XP, 2 * i, 2 * j) + At(XP, 2 * i + 1, 2 * j) + At(XP, 2 * i, 2 * j + 1) + At(XP, 2 * i + 1, 2 * j + 1));
                    YM[size_t(j) * M + i] = 0.25f * (At(YP, 2 * i, 2 * j) + At(YP, 2 * i + 1, 2 * j) + At(YP, 2 * i, 2 * j + 1) + At(YP, 2 * i + 1, 2 * j + 1));
                }
            C.X.push_back(XM);
            C.Y.push_back(YM);
            N = M;
        }
        C.TopMip = int(C.X.size()) - 1;
        return C;
    }

    // Bilinear, periodic, U and V in the level's texels (texel i at i: the GPU's UV N - 1/2).
    float Bil(const std::vector<float>& T, int N, double U, double V)
    {
        const double FU = std::floor(U), FV = std::floor(V);
        const double a = U - FU, b = V - FV;
        const int i0 = int(int64_t(FU) & (N - 1)), j0 = int(int64_t(FV) & (N - 1));
        const int i1 = (i0 + 1) & (N - 1), j1 = (j0 + 1) & (N - 1);
        auto At = [&](int i, int j) { return double(T[size_t(j) * N + i]); };
        return float((1 - b) * ((1 - a) * At(i0, j0) + a * At(i1, j0)) + b * ((1 - a) * At(i0, j1) + a * At(i1, j1)));
    }

    struct FSlope { double X = 0.0, Y = 0.0; };

    // The trilinear lookup at Lod (SampleLevel), UV in tiles; at the top mip and past it the band's
    // mean (0): VoxelOceanFFTChartLookups skips it.
    FSlope Sample(const FChain& C, double Lod, double U, double V)
    {
        FSlope S;
        if (Lod >= double(C.TopMip)) return S;
        Lod = std::max(Lod, 0.0);
        const int L0 = int(std::floor(Lod));
        const double T = Lod - L0;
        auto At = [&](int L, FSlope& O) {
            const int N = C.N0 >> L;
            O.X = Bil(C.X[L], N, U * N - 0.5, V * N - 0.5);
            O.Y = Bil(C.Y[L], N, U * N - 0.5, V * N - 0.5);
        };
        FSlope A, B;
        At(L0, A);
        if (T > 0.0 && L0 + 1 <= C.TopMip) At(L0 + 1, B);
        S.X = (1 - T) * A.X + T * B.X;
        S.Y = (1 - T) * A.Y + T * B.Y;
        return S;
    }

    // VoxelOceanResolved (VoxelOceanCommon.ush).
    double Resolved(double Wavelength, double Spacing) { return Spacing > 0.0 ? std::clamp(Wavelength / (2.0 * Spacing) - 1.0, 0.0, 1.0) : 1.0; }

    struct FBand
    {
        FChain Chain;
        double Tile = 0.0;      // m
        double Texels = 0.0;
        double MinWavelength = 0.0;
        double VarX = 0.0, VarY = 0.0;  // the spectrum's (StatsOf)
        double Var() const { return VarX + VarY; }
    };

    // The normal's mip for a footprint (m): VoxelOceanFFTChartLookups' LodN (bias 0).
    double LodOf(const FBand& B, double Footprint) { return std::log2(std::max(Footprint * B.Texels / B.Tile, 1.0)); }

    // The model's unresolved variance (VoxelOceanFFTSlopeVariance), both axes.
    double ModelUnresolved(const std::vector<FBand>& Bands, double Footprint)
    {
        double V = 0.0;
        for (const FBand& B : Bands) V += B.Var() * (1.0 - Resolved(B.MinWavelength, Footprint));
        return V;
    }

    // The bands' summed slope at a world point (m), each at its own lod for the footprint.
    FSlope SumAt(const std::vector<FBand>& Bands, double Px, double Py, double Footprint)
    {
        FSlope S;
        for (const FBand& B : Bands)
        {
            const FSlope Here = Sample(B.Chain, LodOf(B, Footprint), Px / B.Tile, Py / B.Tile);
            S.X += Here.X;
            S.Y += Here.Y;
        }
        return S;
    }

    struct FClip
    {
        double L0 = 0.2;        // m, the first level's spacing (ClipFinestSpacing 20 cm)
        int Count = 12;
    };

    // A clip level's slope at a point: its texels (the bands at the level's spacing, on its grid),
    // bilinear. (The texels' normals are blended, as the hardware does, then normalised: within a
    // fraction of a percent the slopes' blend at these slopes.)
    FSlope ClipLevelAt(const std::vector<FBand>& Bands, const FClip& Clip, int K, double Px, double Py)
    {
        const double S = Clip.L0 * std::ldexp(1.0, K);
        const double U = Px / S, V = Py / S;
        const double FU = std::floor(U), FV = std::floor(V);
        const double a = U - FU, b = V - FV;
        const FSlope P = SumAt(Bands, FU * S, FV * S, S), Q = SumAt(Bands, (FU + 1) * S, FV * S, S);
        const FSlope R = SumAt(Bands, FU * S, (FV + 1) * S, S), T = SumAt(Bands, (FU + 1) * S, (FV + 1) * S, S);
        FSlope O;
        O.X = (1 - b) * ((1 - a) * P.X + a * Q.X) + b * ((1 - a) * R.X + a * T.X);
        O.Y = (1 - b) * ((1 - a) * P.Y + a * Q.Y) + b * ((1 - a) * R.Y + a * T.Y);
        return O;
    }

    // The pixel's surface (VoxelOceanPixelClipDetailOf without the shore): the level pick for its
    // footprint, the two levels blended, the per-pixel detail where the clipmap is coarser. Out: the
    // slope and the roughness.
    FSlope PixelAt(const std::vector<FBand>& Bands, const FClip& Clip, double Px, double Py, double Footprint, double& Rough)
    {
        const double Want = std::max(std::log2(Footprint / Clip.L0), 0.0);
        const int First = std::min(int(std::floor(Want)), Clip.Count - 1);
        const double Frac = Want - std::floor(Want);
        const double Weight = (Want < double(Clip.Count - 1)) ? 1.0 - Frac : 1.0;
        const FSlope A = ClipLevelAt(Bands, Clip, First, Px, Py);
        FSlope O = A;
        const double SA = Clip.L0 * std::ldexp(1.0, First);
        Rough = Weight * ModelUnresolved(Bands, SA);
        if (Weight < 1.0)
        {
            const FSlope B = ClipLevelAt(Bands, Clip, First + 1, Px, Py);
            O.X = Weight * A.X + (1 - Weight) * B.X;
            O.Y = Weight * A.Y + (1 - Weight) * B.Y;
            Rough += (1 - Weight) * ModelUnresolved(Bands, 2.0 * SA);
        }
        // The pick's spacing (VoxelOceanPixelClipFilteredOf): L0 2^(level + 1 - weight).
        const double Spacing = Clip.L0 * std::exp2(double(First) + 1.0 - Weight);
        if (Spacing > Footprint)
        {
            // The detail: the bands at the pixel less the bands at the clipmap's spacing (the high pass).
            const FSlope D = SumAt(Bands, Px, Py, Footprint), H = SumAt(Bands, Px, Py, Spacing);
            O.X += D.X - H.X;
            O.Y += D.Y - H.Y;
            Rough = std::max(Rough - (ModelUnresolved(Bands, Spacing) - ModelUnresolved(Bands, Footprint)), 0.0);
        }
        return O;
    }

    struct FMoments
    {
        double X2 = 0.0, Y2 = 0.0, MX = 0.0, MY = 0.0, Rough = 0.0;
        int Count = 0;
        void Add(const FSlope& S, double R) { X2 += S.X * S.X; Y2 += S.Y * S.Y; MX += S.X; MY += S.Y; Rough += R; Count++; }
        double VarX() const { return X2 / Count - (MX / Count) * (MX / Count); }
        double VarY() const { return Y2 / Count - (MY / Count) * (MY / Count); }
        double Var() const { return VarX() + VarY(); }
        double MeanRough() const { return Rough / Count; }
    };

    std::vector<FBand> MakeBands(double Wind, uint32_t Tick)
    {
        FLayoutSettings S;
        S.PlanetSeed = 1234567u;
        S.ShortWind = Wind;
        const std::vector<FSlot> Slots = MakeSlots(S);
        // The long bands' sea state at this wind (VoxelOceanStateWeights: 10 m/s state 1, 18 state 2).
        const int State = Wind <= 7.5 ? 0 : (Wind <= 14.0 ? 1 : 2);
        const int Index[4] = { 2 * State, 2 * State + 1, 6, 7 };
        std::vector<FBand> Bands(4);
        for (int b = 0; b < 4; b++)
        {
            const FSlot& Slot = Slots[Index[b]];
            FBand& B = Bands[b];
            B.Tile = Slot.Params.TileLength;
            B.Chain = BuildChain(EmulateGPU(Slot, Tick));
            B.Texels = double(B.Chain.N0);
            const FBandStats St = StatsOf(Slot);
            B.VarX = St.SlopeVarianceX;
            B.VarY = St.SlopeVarianceY;
        }
        // VoxelOceanBandMinWavelength: a band's shortest waves at the next band's lower edge (a
        // quarter of its tile), the last band's at four of its grid's samples.
        for (int b = 0; b < 3; b++) Bands[b].MinWavelength = 0.25 * Bands[b + 1].Tile;
        Bands[3].MinWavelength = Bands[3].Tile * 4.0 / Bands[3].Texels;
        return Bands;
    }
}

int main()
{
    const uint32_t Tick = TickOf(137.0);
    const double Winds[2] = { 10.0, 18.0 };
    // The footprints: 1 cm to 100 km, half an octave apart.
    std::vector<double> Feet;
    for (double F = 0.01; F <= 1.0e5 * 1.0001; F *= std::sqrt(2.0)) Feet.push_back(F);
    std::mt19937 Rng(7);
    std::uniform_real_distribution<double> Unit(0.0, 1.0);
    const FClip Clip;

    for (double Wind : Winds)
    {
        const std::vector<FBand> Bands = MakeBands(Wind, Tick);
        double Total = 0.0, TotalX = 0.0, TotalY = 0.0;
        for (const FBand& B : Bands) { Total += B.Var(); TotalX += B.VarX; TotalY += B.VarY; }
        std::printf("\n==== %.0f m/s (physical heights; the Godot preset's 2.83 x scales every variance by 8, the ratios as here) ====\n", Wind);
        std::printf("total slope variance %.5f (x %.5f along the wind, y %.5f across: ratio %.2f); Cox-Munk clean surface 0.003 + 0.00512 U = %.5f\n",
            Total, TotalX, TotalY, TotalX / TotalY, 0.003 + 0.00512 * Wind);

        std::printf("\n0. The emulated textures against the spectrum (the texels' variance at mip 0)\n");
        for (int b = 0; b < 4; b++)
        {
            const FBand& B = Bands[b];
            double X2 = 0.0, Y2 = 0.0;
            for (size_t i = 0; i < B.Chain.X[0].size(); i++) { X2 += double(B.Chain.X[0][i]) * B.Chain.X[0][i]; Y2 += double(B.Chain.Y[0][i]) * B.Chain.Y[0][i]; }
            X2 /= B.Chain.X[0].size(); Y2 /= B.Chain.Y[0].size();
            std::printf("  band %d: tile %7.1f m, %3d texels, shortest %6.3f m; variance %.5f (%.1f%% of the total), texels %.5f (%.3f of the spectrum's)\n",
                b, B.Tile, int(B.Texels), B.MinWavelength, B.Var(), 100.0 * B.Var() / Total, X2 + Y2, (X2 + Y2) / B.Var());
            // (One draw of the random spectrum: its few dominant modes put the long and middle bands'
            // draws up to a fifth off the expectation the shading's table holds, the shortest band's
            // within a few percent.)
            Check(std::abs((X2 + Y2) / B.Var() - 1.0) < (b == 3 ? 0.05 : 0.25), "the texels hold the spectrum's slope variance (one draw)", (X2 + Y2) / B.Var(), 1.0);
        }

        // 1. The bands' lookups.
        std::printf("\n1. The bands' lookups (the normal's mip by the footprint): resolved + model unresolved against the total\n");
        std::printf("  %10s | %-35s | %-35s | %8s %8s %8s | %7s\n", "footprint", "resolved share per band", "model unresolved share per band", "resolved", "model U", "sum", "ledger");
        const int M = 6000;
        std::vector<double> UA(M), VA(M);
        for (int n = 0; n < M; n++) { UA[n] = Unit(Rng); VA[n] = Unit(Rng); }
        double WorstLookup = 0.0, WorstLookupAt = 0.0;
        std::vector<std::vector<double>> ResolvedShare(4, std::vector<double>(Feet.size()));
        std::vector<double> ResolvedX(Feet.size()), ResolvedY(Feet.size());
        for (size_t f = 0; f < Feet.size(); f++)
        {
            const double F = Feet[f];
            double R = 0.0, RX = 0.0, RY = 0.0;
            char Res[128] = "", Mod[128] = "";
            for (int b = 0; b < 4; b++)
            {
                const FBand& B = Bands[b];
                FMoments Mo;
                for (int n = 0; n < M; n++) Mo.Add(Sample(B.Chain, LodOf(B, F), UA[n], VA[n]), 0.0);
                ResolvedShare[b][f] = Mo.Var() / B.Var();
                R += Mo.Var(); RX += Mo.VarX(); RY += Mo.VarY();
                std::snprintf(Res + std::strlen(Res), sizeof(Res) - std::strlen(Res), " %7.3f", Mo.Var() / B.Var());
                std::snprintf(Mod + std::strlen(Mod), sizeof(Mod) - std::strlen(Mod), " %7.3f", 1.0 - Resolved(B.MinWavelength, F));
            }
            ResolvedX[f] = RX; ResolvedY[f] = RY;
            const double U = ModelUnresolved(Bands, F);
            const double Ledger = (R + U) / Total - 1.0;
            if (std::abs(Ledger) > std::abs(WorstLookup)) { WorstLookup = Ledger; WorstLookupAt = F; }
            std::printf("  %8.3g m | %-35s | %-35s | %8.5f %8.5f %8.5f | %+6.1f%%\n", F, Res, Mod, R, U, R + U, 100.0 * Ledger);
        }
        std::printf("  worst: %+.1f%% at %.3g m\n", 100.0 * WorstLookup, WorstLookupAt);

        // 4. The axes.
        std::printf("\n4. The axes: the unresolved variance along (x) and across (y) the wind (the shading takes half the sum for each)\n");
        for (size_t f = 0; f < Feet.size(); f += 4)
        {
            const double UX = std::max(TotalX - ResolvedX[f], 0.0), UY = std::max(TotalY - ResolvedY[f], 0.0);
            std::printf("  %8.3g m: true unresolved x %.5f, y %.5f (ratio %.2f); the shading's per axis %.5f\n", Feet[f], UX, UY, UY > 1e-12 ? UX / UY : 0.0,
                0.5 * ModelUnresolved(Bands, Feet[f]));
        }

        // 2. The clipmap's chain, at the pixel.
        std::printf("\n2. The clipmap's chain at the pixel (first level %.0f cm, %d levels; the outermost's spacing %.1f m)\n", 100.0 * Clip.L0, Clip.Count,
            Clip.L0 * std::ldexp(1.0, Clip.Count - 1));
        std::printf("  %10s | %8s %8s %8s | %7s | %s\n", "footprint", "resolved", "rough", "sum", "ledger", "(resolved as the bands' own lookups at the footprint would hold it)");
        const int MP = 2500;
        double WorstClip = 0.0, WorstClipAt = 0.0, WorstNear = 0.0;
        for (size_t f = 0; f < Feet.size(); f++)
        {
            const double F = Feet[f];
            FMoments Mo;
            for (int n = 0; n < MP; n++)
            {
                const double Px = 1.0e5 * Unit(Rng), Py = 1.0e5 * Unit(Rng);
                double Rough = 0.0;
                const FSlope S = PixelAt(Bands, Clip, Px, Py, F, Rough);
                Mo.Add(S, Rough);
            }
            double Own = 0.0;
            for (int b = 0; b < 4; b++) Own += ResolvedShare[b][f] * Bands[b].Var();
            const double Ledger = (Mo.Var() + Mo.MeanRough()) / Total - 1.0;
            if (std::abs(Ledger) > std::abs(WorstClip)) { WorstClip = Ledger; WorstClipAt = F; }
            if (F <= Clip.L0 * std::ldexp(1.0, Clip.Count - 1)) WorstNear = std::max(WorstNear, std::abs(Ledger));
            std::printf("  %8.3g m | %8.5f %8.5f %8.5f | %+6.1f%% | %8.5f\n", F, Mo.Var(), Mo.MeanRough(), Mo.Var() + Mo.MeanRough(), 100.0 * Ledger, Own);
        }
        std::printf("  worst: %+.1f%% at %.3g m\n", 100.0 * WorstClip, WorstClipAt);

        // 3. Past the outermost level's spacing: one lookup of its texels per pixel, however many
        // texels the pixel covers. Its resolved variance is the level's, as noise over the pixels.
        {
            const double SL = Clip.L0 * std::ldexp(1.0, Clip.Count - 1);
            double Own = 0.0;
            for (int b = 0; b < 4; b++)
            {
                FMoments Mo;
                for (int n = 0; n < M; n++) Mo.Add(Sample(Bands[b].Chain, LodOf(Bands[b], SL), UA[n], VA[n]), 0.0);
                Own += Mo.Var();
            }
            std::printf("\n3. Past the outermost level's spacing (%.0f m): its texels' variance %.5f (%.1f%% of the total) drawn per pixel at any footprint: noise once a pixel covers many texels\n",
                SL, Own, 100.0 * Own / Total);
            std::printf("   (past its reach, the far sphere: the radial, roughness the reach edge's texel's N.w + its slope^2: on average the total, per direction a draw of chi^2 with 2 dof of mean %.5f)\n", Own);
        }
    }

    std::printf("\n%s (%d failures)\n", Fails == 0 ? "ALL OK" : "FAILED", Fails);
    return Fails == 0 ? 0 : 1;
}
