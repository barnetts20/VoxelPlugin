// O6a: pictures of the open sea built from the four cascades (the GPU passes emulated in float),
// to judge tiling from altitude, with and without detiling (VoxelOceanDetileOf).
//
// The roughness comes from mips of the slopes' second moments (LEAN-style). The GPU's
// Derivatives texture holds the slopes only: the shading O6b builds must either add a moments
// texture or take the unresolved variance from the spectrum (StatsOf) by footprint; this view
// shows what the moments would give.
//
// Views: a camera at altitude looking toward the horizon (sky reflection with Fresnel, a sun
// glint whose roughness is the slope variance the mips filter away, as the shading will do),
// and a top-down relief map (a low sun, contrast stretched: a diagnostic, not a look). Each band is sampled through its mip pyramid by the pixel's footprint
// (slope means and second moments, LEAN-style), at its own tile coordinates; the wind is turned
// 30 degrees from the view.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private -I. spectrum_view.cpp -o spectrum_view
//   ./spectrum_view <wind m/s> <view: persp|top> <altitude m (persp) or extent m (top)> <detile cell m, 0: off>
//       <detiled bands 0-4> <out.ppm>
#include "ocean_fft_emulation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace OceanFFTEmulation;

struct FMoments { float Sx, Sy, Sxx, Syy, Sxy; };

struct FPyramid
{
    std::vector<std::vector<FMoments>> Levels;
    double Cell = 1.0;
    int Size = GPUSize;     // the slot's grid (stage W3z: 512 for the shortest band)
    explicit FPyramid(const FGPUSlot& G, double TileLength)
    {
        Size = G.N;
        Cell = TileLength / Size;
        std::vector<FMoments> L0(size_t(Size) * Size);
        for (size_t i = 0; i < L0.size(); i++)
        {
            const float Sx = G.Derivatives[i].x, Sy = G.Derivatives[i].y;
            L0[i] = { Sx, Sy, Sx * Sx, Sy * Sy, Sx * Sy };
        }
        Levels.push_back(L0);
        for (int N = Size / 2; N >= 1; N /= 2)
        {
            const std::vector<FMoments>& P = Levels.back();
            std::vector<FMoments> L(size_t(N) * N);
            for (int j = 0; j < N; j++)
                for (int i = 0; i < N; i++)
                {
                    FMoments M = { 0, 0, 0, 0, 0 };
                    for (int b = 0; b < 2; b++)
                        for (int a = 0; a < 2; a++)
                        {
                            const FMoments& Q = P[size_t(2 * j + b) * (2 * N) + 2 * i + a];
                            M.Sx += 0.25f * Q.Sx; M.Sy += 0.25f * Q.Sy; M.Sxx += 0.25f * Q.Sxx; M.Syy += 0.25f * Q.Syy; M.Sxy += 0.25f * Q.Sxy;
                        }
                    L[size_t(j) * N + i] = M;
                }
            Levels.push_back(L);
        }
    }
    FMoments Bilinear(int Level, double U, double V) const
    {
        const int N = Size >> Level;
        const double S = double(1 << Level);
        U = (U + 0.5) / S - 0.5; V = (V + 0.5) / S - 0.5;
        const double FU = std::floor(U), FV = std::floor(V);
        const float a = float(U - FU), b = float(V - FV);
        const int i0 = int(int64_t(FU) & (N - 1)), j0 = int(int64_t(FV) & (N - 1));
        const int i1 = (i0 + 1) & (N - 1), j1 = (j0 + 1) & (N - 1);
        const std::vector<FMoments>& L = Levels[Level];
        const FMoments& p = L[size_t(j0) * N + i0]; const FMoments& q = L[size_t(j0) * N + i1];
        const FMoments& r = L[size_t(j1) * N + i0]; const FMoments& s = L[size_t(j1) * N + i1];
        auto M = [&](float P, float Q, float R, float T) { return (1 - b) * ((1 - a) * P + a * Q) + b * ((1 - a) * R + a * T); };
        return { M(p.Sx, q.Sx, r.Sx, s.Sx), M(p.Sy, q.Sy, r.Sy, s.Sy), M(p.Sxx, q.Sxx, r.Sxx, s.Sxx), M(p.Syy, q.Syy, r.Syy, s.Syy), M(p.Sxy, q.Sxy, r.Sxy, s.Sxy) };
    }
    // Trilinear by footprint (m), tile-plane position (m).
    FMoments Sample(double X, double Y, double Footprint) const
    {
        const int Top = int(Levels.size()) - 1;
        const double Lod = std::clamp(std::log2(std::max(Footprint / Cell, 1.0)), 0.0, double(Top));
        const int L0 = int(Lod), L1 = std::min(L0 + 1, Top);
        const float t = float(Lod - L0);
        const FMoments A = Bilinear(L0, X / Cell, Y / Cell), B = Bilinear(L1, X / Cell, Y / Cell);
        return { A.Sx + t * (B.Sx - A.Sx), A.Sy + t * (B.Sy - A.Sy), A.Sxx + t * (B.Sxx - A.Sxx), A.Syy + t * (B.Syy - A.Syy), A.Sxy + t * (B.Sxy - A.Sxy) };
    }
};

struct V3 { double x, y, z; };
static V3 operator+(V3 a, V3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static V3 operator*(V3 a, double s) { return { a.x * s, a.y * s, a.z * s }; }
static double Dot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static V3 Norm(V3 a) { return a * (1.0 / std::sqrt(Dot(a, a))); }

static V3 Sky(V3 R)
{
    const double E = std::clamp(R.z, 0.0, 1.0);
    const V3 Horizon = { 0.78, 0.84, 0.90 }, Zenith = { 0.22, 0.42, 0.78 };
    return Horizon * (1 - std::sqrt(E)) + Zenith * std::sqrt(E);
}

int main(int argc, char** argv)
{
    if (argc < 7) { std::fprintf(stderr, "usage: spectrum_view wind persp|top altitude_or_extent detile_cell detiled_bands out.ppm\n"); return 1; }
    const double Wind = std::atof(argv[1]);
    const bool Top = std::string(argv[2]) == "top";
    const double Size = std::atof(argv[3]);
    const double DetileCell = std::atof(argv[4]);
    const int DetiledBands = std::atoi(argv[5]);
    const char* Out = argv[6];

    FLayoutSettings Settings;
    Settings.PlanetSeed = 1234567u;
    Settings.StateWind[1] = Wind;
    Settings.ShortWind = Wind;
    const std::vector<FSlot> Slots = MakeSlots(Settings);
    const int Use[4] = { 2, 3, 6, 7 };
    const uint32_t Tick = TickOf(200.0);
    std::vector<FPyramid> Bands;
    double ExtraVariance = 0.0005;   // the facets below the last band's top
    for (int b = 0; b < 4; b++) Bands.emplace_back(EmulateGPU(Slots[Use[b]], Tick), Slots[Use[b]].Params.TileLength);
    const uint32_t DetileSeed = VoxelShader::VoxelOceanHash(Settings.PlanetSeed ^ 0xde711eu);

    const int W = 1280, H = Top ? 1280 : 720;
    std::vector<unsigned char> Img(size_t(W) * H * 3);
    const double WindAngle = 30.0 * Pi / 180.0, Cw = std::cos(WindAngle), Sw = std::sin(WindAngle);
    const V3 Sun = Norm({ 0.25, 1.0, Top ? 0.6 : 0.30 });

    // Camera (perspective): at altitude Size, looking along +y, pitched down.
    const double Pitch = (Size <= 600 ? 12.0 : 25.0) * Pi / 180.0;
    const V3 Fwd = { 0, std::cos(Pitch), -std::sin(Pitch) }, Right = { 1, 0, 0 }, Up = { 0, std::sin(Pitch), std::cos(Pitch) };
    const double TanHalf = std::tan(35.0 * Pi / 180.0), PixAngle = 2 * TanHalf / W;

    for (int py = 0; py < H; py++)
        for (int px = 0; px < W; px++)
        {
            V3 Color;
            double X, Y, Footprint, Dist = 0;
            V3 D;
            if (Top)
            {
                X = (px + 0.5) / W * Size; Y = (py + 0.5) / H * Size; Footprint = Size / W;
                D = Norm({ 0.0, 0.35, -1.0 });
            }
            else
            {
                const double sx = (2 * (px + 0.5) / W - 1) * TanHalf, sy = (1 - 2 * (py + 0.5) / H) * TanHalf * H / W;
                D = Norm(Fwd + Right * sx + Up * sy);
                if (D.z > -1e-4)
                {
                    const V3 C = Sky(D);
                    for (int c = 0; c < 3; c++) Img[(size_t(py) * W + px) * 3 + c] = (unsigned char)std::clamp(255.0 * std::pow((&C.x)[c], 1 / 1.2), 0.0, 255.0);
                    continue;
                }
                Dist = Size / -D.z;
                X = D.x * Dist; Y = D.y * Dist;
                Footprint = Dist * PixAngle / std::sqrt(-D.z);
            }
            // Tile-plane coordinates (wind along the tile's +x).
            const double U = Cw * X + Sw * Y, V = -Sw * X + Cw * Y;
            double Mx = 0, My = 0, Var = ExtraVariance;
            for (int b = 0; b < 4; b++)
            {
                if (DetileCell > 0 && b < DetiledBands)
                {
                    const VoxelShader::FVoxelOceanDetile T = VoxelShader::VoxelOceanDetileOf(float2(float(U), float(V)), float(DetileCell), DetileSeed);
                    const float2 O[3] = { T.Offset0, T.Offset1, T.Offset2 };
                    const float Wt[3] = { T.Weight.x, T.Weight.y, T.Weight.z };
                    for (int c = 0; c < 3; c++)
                    {
                        const FMoments M = Bands[b].Sample(U + O[c].x, V + O[c].y, Footprint);
                        Mx += Wt[c] * M.Sx; My += Wt[c] * M.Sy;
                        Var += Wt[c] * Wt[c] * std::max(0.0, double(M.Sxx - M.Sx * M.Sx) + double(M.Syy - M.Sy * M.Sy)) * 0.5;
                    }
                }
                else
                {
                    const FMoments M = Bands[b].Sample(U, V, Footprint);
                    Mx += M.Sx; My += M.Sy;
                    Var += std::max(0.0, double(M.Sxx - M.Sx * M.Sx) + double(M.Syy - M.Sy * M.Sy)) * 0.5;
                }
            }
            // Back to world slopes.
            const double Sx = Cw * Mx - Sw * My, Sy = Sw * Mx + Cw * My;
            const V3 N = Norm({ -Sx, -Sy, 1.0 });
            const V3 Vv = D * -1.0;
            const double NV = std::max(Dot(N, Vv), 1e-3);
            const double F = 0.02 + 0.98 * std::pow(1 - NV, 5);
            V3 R = D + N * (-2 * Dot(D, N));
            if (R.z < 0) R.z = -R.z;
            // Sun glint: Gaussian facet slopes around the mean, variance Var per axis.
            const V3 Hh = Norm(Sun + Vv);
            const double HSx = -Hh.x / Hh.z, HSy = -Hh.y / Hh.z;
            const double Dsl = std::exp(-((HSx + Sx) * (HSx + Sx) + (HSy + Sy) * (HSy + Sy)) / (2 * Var)) / (2 * Pi * Var);
            const double Glint = 1.2 * Dsl * (0.02 + 0.98 * std::pow(1 - std::max(Dot(Hh, Vv), 0.0), 5)) / (4 * std::pow(Hh.z, 4) * NV) * std::max(Dot(N, Sun), 0.0);
            const V3 Water = { 0.01, 0.045, 0.07 };
            Color = Water * (1 - F) + Sky(R) * F + V3{ 1.0, 0.95, 0.85 } * std::min(Glint, 3.0);
            if (Top)
            {
                // A relief diagnostic: low sun, contrast stretched, so a repeat would show.
                const V3 L = Norm({ 0.6, 0.3, 0.55 });
                const double G = std::clamp(0.5 + 2.5 * (Dot(N, L) - L.z), 0.0, 1.0);
                Color = V3{ G, G, G };
            }
            if (!Top)
            {
                const double Haze = 1 - std::exp(-Dist / 25000.0);
                Color = Color * (1 - Haze) + Sky({ 0, 0, 0.02 }) * Haze;
            }
            for (int c = 0; c < 3; c++)
            {
                const double v = (&Color.x)[c];
                Img[(size_t(py) * W + px) * 3 + c] = (unsigned char)std::clamp(255.0 * std::pow(v / (1 + 0.15 * v), 1 / 1.2), 0.0, 255.0);
            }
        }
    FILE* f = std::fopen(Out, "wb");
    std::fprintf(f, "P6\n%d %d\n255\n", W, H);
    std::fwrite(Img.data(), 1, Img.size(), f);
    std::fclose(f);
    return 0;
}
