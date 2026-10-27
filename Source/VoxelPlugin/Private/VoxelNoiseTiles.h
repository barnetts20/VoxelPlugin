// VoxelNoiseTiles.h - Packed tiling noise sets (Design/NoiseTiles2D_Handoff.md, Addenda 1 and 2):
// square, power-of-two textures of four independent noise patterns (RGBA, 16-bit unsigned
// normalised), periodic in both axes, with a box-filtered mip chain. The land's per-sample layers
// sample them with a cubic B-spline over 4 x 4 texels (VoxelLandNoise.ush), which gives each
// channel's value and exact gradient; the planet picks a set and blends its channels.
//
// Layout, shared by the CPU copy, the global shaders' buffer and the material's texture, and the
// same as the authored files' (.vxnt v3): the mip chain packed, mip 0 first, each mip's rows in
// order: texel (x, y) of mip m (N >> m texels square) is texel MipOffset(N, m) + y (N >> m) + x;
// four u16 a texel (R, G, B, A). Texel (x, y) of mip m is the node at (x, y) in mip m's own texel
// units (VoxelLandNoise.ush maps between mips at the box filter's centres).
//
// Sets come from the noise generator's .vxnt files (ReadVxnt), or, without them, from the
// plugin's own stand-ins (GeneratePeaksStandIn: the eroded peaks' layout, Addendum 2, without the
// simulation; GenerateDetailStandIn: the detail sets' layout, the handoff's section 6). Generate
// makes the handoff's raw kinds (its hashing differs from the utility's).
// Plain C++ without engine types: the harnesses run the same reader and generators.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace VoxelNoiseTiles
{
    /** Channel kinds (the handoff's section 3; Addendum 2's 7 to 9). */
    enum EKind : uint32_t
    {
        KindNone = 0,
        KindFbm = 1,
        KindRidged = 2,
        KindBillow = 3,
        KindCellularBorder = 4,
        KindCellularValue = 5,
        KindErodedFbm = 6,
        KindErodedHeight = 7,       // a simulated heightfield, in mip-0 texel units
        KindBlurred = 8,            // a channel blurred (Source)
        KindFlowAccumulation = 9,   // log2 of a heightfield's contributing area (Source)
        // The ocean's foam set (stage W3g; NoiseTiles2D_Addendum5.md, Tools/NoiseTileTool/foam_tile.py).
        KindFoamLace = 10,          // bubbles gathered on a turbulent surface's convergence lines
        KindFoamGrain = 11,         // the bubbles' grain
        KindFoamWhitecap = 12,      // a whitecap's billowy turbulence
        KindFoamStreak = 13,        // the lace's flow stretched downwind (x)
    };

    struct FChannel
    {
        uint32_t Kind = KindNone;
        uint32_t Seed = 0;
        uint32_t CellsPerTile = 1;   // the coarsest lattice's cells along a side (a power of two)
        uint32_t Octaves = 1;
        float Lacunarity = 2.0f;
        float Gain = 0.5f;
        float Jitter = 0.0f;
        float ErosionK = 0.0f;       // eroded_fbm's damping
        int32_t Source = -1;         // cellular_value: the cellular_border channel whose sites it shares
        // Decode: value = Min + (Max - Min) x q / 65535 (q the stored u16).
        float Min = 0.0f;
        float Max = 1.0f;
        // Of the decoded mip 0.
        float Mean = 0.0f;
        float Std = 0.0f;
    };

    /** The first texel of mip Mip (0 .. log2 N + 1: one past the last mip gives the total). */
    inline uint64_t MipOffset(uint32_t N, uint32_t Mip)
    {
        // The mips above: N^2 (1 + 1/4 + ...) = 4 (N^2 - S^2) / 3, exact (N^2 - S^2 = S^2 (4^m - 1)).
        const uint64_t S = (uint64_t)(N >> Mip);
        const uint64_t NN = (uint64_t)N * N;
        return (N >> Mip) > 0 ? 4u * (NN - S * S) / 3u : (4u * NN - 1u) / 3u;
    }

    /** Texels in the whole chain, N .. 1. */
    inline uint64_t ChainTexels(uint32_t N) { return (4u * (uint64_t)N * N - 1u) / 3u; }

    struct FSet
    {
        std::string Name;
        std::string LookTag;
        uint32_t Seed = 0;
        uint32_t N = 0;              // texels per side of mip 0 (a power of two)
        uint32_t Mips = 0;           // N .. 1
        FChannel Channel[4];
        std::vector<uint16_t> Texels;   // the mip chain (MipOffset), 4 u16 a texel (RGBA)

        bool IsValid() const { return N > 0 && Texels.size() == ChainTexels(N) * 4; }
    };

    // --- Generation (double precision, integer hashing) -----------------------------------------
    // The same bytes on every machine in practice (checked across compilers and FMA contraction),
    // though cos, sin and exp aren't correctly rounded by every C runtime: a runtime that differs
    // in their last bit could move a texel by one step.

    inline uint32_t Mix(uint32_t H)
    {
        H ^= H >> 16;
        H *= 0x7feb352du;
        H ^= H >> 15;
        H *= 0x846ca68bu;
        H ^= H >> 16;
        return H;
    }

    inline uint32_t Hash3(uint32_t A, uint32_t B, uint32_t C)
    {
        return Mix(A ^ Mix(B ^ Mix(C + 0x9e3779b9u)));
    }

    inline int32_t Wrap(int32_t I, int32_t Period)
    {
        const int32_t R = I % Period;
        return R < 0 ? R + Period : R;
    }

    /** Periodic 2D gradient noise (quintic) at P in lattice cells, period Period cells. */
    inline double GradientNoise(double X, double Y, int32_t Period, uint32_t Seed)
    {
        const double FX0 = std::floor(X);
        const double FY0 = std::floor(Y);
        const int32_t IX = (int32_t)FX0;
        const int32_t IY = (int32_t)FY0;
        const double FX = X - FX0;
        const double FY = Y - FY0;
        auto Corner = [&](int32_t DX, int32_t DY) {
            const uint32_t H = Hash3((uint32_t)Wrap(IX + DX, Period), (uint32_t)Wrap(IY + DY, Period), Seed);
            const double A = (double)(H >> 8) * (6.283185307179586 / 16777216.0);
            return std::cos(A) * (FX - DX) + std::sin(A) * (FY - DY);
        };
        const double U = FX * FX * FX * (FX * (FX * 6.0 - 15.0) + 10.0);
        const double V = FY * FY * FY * (FY * (FY * 6.0 - 15.0) + 10.0);
        const double N0 = Corner(0, 0) + U * (Corner(1, 0) - Corner(0, 0));
        const double N1 = Corner(0, 1) + U * (Corner(1, 1) - Corner(0, 1));
        return N0 + V * (N1 - N0);
    }

    /** GradientNoise and its derivatives along X and Y (per lattice cell). */
    inline double GradientNoiseD(double X, double Y, int32_t Period, uint32_t Seed, double& OutDX, double& OutDY)
    {
        const double FX0 = std::floor(X);
        const double FY0 = std::floor(Y);
        const int32_t IX = (int32_t)FX0;
        const int32_t IY = (int32_t)FY0;
        const double FX = X - FX0;
        const double FY = Y - FY0;
        double GX[2][2], GY[2][2], C[2][2];
        for (int32_t j = 0; j < 2; j++)
            for (int32_t i = 0; i < 2; i++)
            {
                const uint32_t H = Hash3((uint32_t)Wrap(IX + i, Period), (uint32_t)Wrap(IY + j, Period), Seed);
                const double A = (double)(H >> 8) * (6.283185307179586 / 16777216.0);
                GX[j][i] = std::cos(A);
                GY[j][i] = std::sin(A);
                C[j][i] = GX[j][i] * (FX - i) + GY[j][i] * (FY - j);
            }
        const double U = FX * FX * FX * (FX * (FX * 6.0 - 15.0) + 10.0);
        const double V = FY * FY * FY * (FY * (FY * 6.0 - 15.0) + 10.0);
        const double DU = 30.0 * FX * FX * (FX - 1.0) * (FX - 1.0);
        const double DV = 30.0 * FY * FY * (FY - 1.0) * (FY - 1.0);
        const double N0 = C[0][0] + U * (C[0][1] - C[0][0]);
        const double N1 = C[1][0] + U * (C[1][1] - C[1][0]);
        const double N0X = GX[0][0] + U * (GX[0][1] - GX[0][0]) + DU * (C[0][1] - C[0][0]);
        const double N1X = GX[1][0] + U * (GX[1][1] - GX[1][0]) + DU * (C[1][1] - C[1][0]);
        const double N0Y = GY[0][0] + U * (GY[0][1] - GY[0][0]);
        const double N1Y = GY[1][0] + U * (GY[1][1] - GY[1][0]);
        OutDX = N0X + V * (N1X - N0X);
        OutDY = N0Y + V * (N1Y - N0Y) + DV * (N1 - N0);
        return N0 + V * (N1 - N0);
    }

    /** A channel's value at P in its coarsest cells (P ranges over 0 .. CellsPerTile). */
    inline double Evaluate(const FChannel& C, double X, double Y)
    {
        const int32_t Period = (int32_t)C.CellsPerTile;
        switch (C.Kind)
        {
        case KindFbm:
        case KindBillow:
        {
            double Sum = 0.0, Norm = 0.0, Amp = 1.0, F = 1.0;
            for (uint32_t o = 0; o < C.Octaves; o++)
            {
                const double N = GradientNoise(X * F, Y * F, Period * (int32_t)F, C.Seed + o * 0x9e3779b9u);
                Sum += Amp * (C.Kind == KindBillow ? std::fabs(N) : N);
                Norm += Amp;
                Amp *= C.Gain;
                F *= 2.0;
            }
            return Sum / Norm;
        }
        case KindErodedFbm:
        {
            // Slope-damped fbm (Inigo Quilez's): each octave over 1 + k |the octaves' gradients
            // so far|^2, the gradients in each octave's own cells.
            double Sum = 0.0, Norm = 0.0, Amp = 1.0, F = 1.0, DX = 0.0, DY = 0.0;
            for (uint32_t o = 0; o < C.Octaves; o++)
            {
                double NX = 0.0, NY = 0.0;
                const double N = GradientNoiseD(X * F, Y * F, Period * (int32_t)F, C.Seed + o * 0x9e3779b9u, NX, NY);
                DX += NX;
                DY += NY;
                Sum += Amp * N / (1.0 + (double)C.ErosionK * (DX * DX + DY * DY));
                Norm += Amp;
                Amp *= C.Gain;
                F *= 2.0;
            }
            return Sum / Norm;
        }
        case KindRidged:
        {
            // Musgrave's ridged multifractal: r = (1 - |n|)^2, each octave weighted by the
            // previous octave's r.
            double Sum = 0.0, Norm = 0.0, Amp = 1.0, F = 1.0, Weight = 1.0;
            for (uint32_t o = 0; o < C.Octaves; o++)
            {
                const double N = GradientNoise(X * F, Y * F, Period * (int32_t)F, C.Seed + o * 0x9e3779b9u);
                const double R = (1.0 - std::fabs(N)) * (1.0 - std::fabs(N));
                Sum += Amp * R * Weight;
                Norm += Amp;
                Weight = std::min(std::max(R, 0.0), 1.0);
                Amp *= C.Gain;
                F *= 2.0;
            }
            return Sum / Norm;
        }
        case KindCellularBorder:
        case KindCellularValue:
        {
            // Jittered sites, one a cell; the border distance (Inigo Quilez): the nearest site,
            // then the nearest bisector to it among the 5 x 5 cells around it.
            const double FX0 = std::floor(X);
            const double FY0 = std::floor(Y);
            const int32_t IX = (int32_t)FX0;
            const int32_t IY = (int32_t)FY0;
            const double FX = X - FX0;
            const double FY = Y - FY0;
            auto Site = [&](int32_t GX, int32_t GY, double& OX, double& OY) {
                const uint32_t H = Hash3((uint32_t)Wrap(IX + GX, Period), (uint32_t)Wrap(IY + GY, Period), C.Seed);
                OX = GX + 0.5 + C.Jitter * ((double)(H & 0xffffu) / 65535.0 - 0.5) - FX;
                OY = GY + 0.5 + C.Jitter * ((double)(H >> 16) / 65535.0 - 0.5) - FY;
            };
            double Best = 1e30, MX = 0.0, MY = 0.0;
            int32_t BX = 0, BY = 0;
            for (int32_t j = -1; j <= 1; j++)
                for (int32_t i = -1; i <= 1; i++)
                {
                    double RX, RY;
                    Site(i, j, RX, RY);
                    const double D = RX * RX + RY * RY;
                    if (D < Best) { Best = D; MX = RX; MY = RY; BX = i; BY = j; }
                }
            if (C.Kind == KindCellularValue)
            {
                const uint32_t H = Hash3((uint32_t)Wrap(IX + BX, Period), (uint32_t)Wrap(IY + BY, Period), C.Seed ^ 0x5bd1e995u);
                return (double)(H >> 8) / 16777215.0;
            }
            double Border = 1e30;
            for (int32_t j = -2; j <= 2; j++)
                for (int32_t i = -2; i <= 2; i++)
                {
                    double RX, RY;
                    Site(BX + i, BY + j, RX, RY);
                    const double DX = RX - MX, DY = RY - MY;
                    const double L2 = DX * DX + DY * DY;
                    if (L2 < 1e-12) continue;
                    const double L = std::sqrt(L2);
                    Border = std::min(Border, (0.5 * (MX + RX) * DX + 0.5 * (MY + RY) * DY) / L);
                }
            return Border;
        }
        default:
            return 0.0;
        }
    }

    inline void Finish(FSet& Set, std::vector<double> (&Level)[4]);

    /** Fills Set's texels and statistics from its channels' definitions. ParallelFor(Count,
     *  Body) runs Body(i) for i in 0 .. Count - 1 (in any order, on any threads). */
    template <typename TParallelFor>
    void Generate(FSet& Set, TParallelFor&& ParallelFor)
    {
        const uint32_t N = Set.N;
        // Mip 0 in double, per channel.
        std::vector<double> Level[4];
        for (int c = 0; c < 4; c++) Level[c].assign((size_t)N * N, 0.0);
        ParallelFor((int32_t)N, [&](int32_t y) {
            for (uint32_t x = 0; x < N; x++)
                for (int c = 0; c < 4; c++)
                {
                    const FChannel& C = Set.Channel[c];
                    if (C.Kind == KindNone) continue;
                    const double K = (double)C.CellsPerTile / (double)N;
                    Level[c][(size_t)y * N + x] = Evaluate(C, (double)x * K, (double)y * K);
                }
        });
        Finish(Set, Level);
    }

    /** Quantises the four channels' mip 0 (Level, in double, N x N each: consumed) into Set's
     *  texels with their box mip chain, and fills the decode ranges and statistics. */
    inline void Finish(FSet& Set, std::vector<double> (&Level)[4])
    {
        const uint32_t N = Set.N;
        Set.Mips = 0;
        for (uint32_t S = N; S > 0; S >>= 1) Set.Mips++;
        Set.Texels.assign((size_t)ChainTexels(N) * 4, 0);
        // The decode range and statistics of mip 0.
        for (int c = 0; c < 4; c++)
        {
            double Lo = 1e30, Hi = -1e30;
            for (double V : Level[c]) { Lo = std::min(Lo, V); Hi = std::max(Hi, V); }
            if (!(Hi > Lo)) { Lo = 0.0; Hi = 1.0; }
            Set.Channel[c].Min = (float)Lo;
            Set.Channel[c].Max = (float)Hi;
        }
        auto Quantise = [](double V, float Lo, float Hi) {
            const double Q = (V - (double)Lo) / ((double)Hi - (double)Lo) * 65535.0;
            return (uint16_t)std::min(65535.0, std::max(0.0, std::floor(Q + 0.5)));
        };
        for (uint32_t m = 0; m < Set.Mips; m++)
        {
            const uint32_t S = N >> m;
            if (m > 0)
            {
                // The exact 2 x 2 box average of the level above, in double.
                for (int c = 0; c < 4; c++)
                {
                    std::vector<double> Next((size_t)S * S);
                    const uint32_t P = S * 2;
                    for (uint32_t y = 0; y < S; y++)
                        for (uint32_t x = 0; x < S; x++)
                            Next[(size_t)y * S + x] = 0.25 * (Level[c][(size_t)(2 * y) * P + 2 * x] + Level[c][(size_t)(2 * y) * P + 2 * x + 1]
                                + Level[c][(size_t)(2 * y + 1) * P + 2 * x] + Level[c][(size_t)(2 * y + 1) * P + 2 * x + 1]);
                    Level[c] = std::move(Next);
                }
            }
            const size_t Base = (size_t)MipOffset(N, m);
            for (size_t i = 0; i < (size_t)S * S; i++)
                for (int c = 0; c < 4; c++)
                    Set.Texels[(Base + i) * 4 + c] = Quantise(Level[c][i], Set.Channel[c].Min, Set.Channel[c].Max);
            if (m == 0)
            {
                for (int c = 0; c < 4; c++)
                {
                    double Sum = 0.0, Sum2 = 0.0;
                    const FChannel& C = Set.Channel[c];
                    for (size_t i = 0; i < (size_t)S * S; i++)
                    {
                        const size_t T = i * 4 + c;
                        const double V = (double)C.Min + ((double)C.Max - (double)C.Min) * (double)Set.Texels[T] / 65535.0;
                        Sum += V;
                        Sum2 += V * V;
                    }
                    const double Mean = Sum / ((double)S * S);
                    Set.Channel[c].Mean = (float)Mean;
                    Set.Channel[c].Std = (float)std::sqrt(std::max(Sum2 / ((double)S * S) - Mean * Mean, 0.0));
                }
            }
        }
    }

    /** The mean over mip 0 of F(decoded value of Channel), the texels Texels (the chain, as in
     *  FSet::Texels: four u16 a texel). */
    template <typename TFunc>
    double MeanOf(const FSet& Set, const uint16_t* Texels, int Channel, TFunc&& F)
    {
        const FChannel& C = Set.Channel[Channel];
        const size_t Count = (size_t)Set.N * Set.N;
        double Sum = 0.0;
        for (size_t i = 0; i < Count; i++)
        {
            const uint16_t Q = Texels[i * 4 + Channel];
            Sum += F((double)C.Min + ((double)C.Max - (double)C.Min) * (double)Q / 65535.0);
        }
        return Sum / (double)Count;
    }

    template <typename TFunc>
    double MeanOf(const FSet& Set, int Channel, TFunc&& F) { return MeanOf(Set, Set.Texels.data(), Channel, F); }

    /** Stage T5d3: the direction (radians, -pi/2 .. pi/2 from the tile's x toward its y, the texel
     *  rows) Channel's mip-0 slopes mostly face, from their structure tensor (the mean of g g^T over
     *  the tile, periodic central differences): half the angle of (Sxx - Syy, 2 Sxy). The normal of
     *  a set's ridges or slots (polar R's crevasses, G's sastrugi), defined up to a half turn. */
    inline double GrainAngleOf(const FSet& Set, const uint16_t* Texels, int Channel)
    {
        const uint32_t N = Set.N;
        auto At = [&](uint32_t x, uint32_t y) { return (double)Texels[((size_t)y * N + x) * 4 + Channel]; };
        double Sxx = 0.0, Syy = 0.0, Sxy = 0.0;
        for (uint32_t y = 0; y < N; y++)
            for (uint32_t x = 0; x < N; x++)
            {
                const double GX = At((x + 1) % N, y) - At((x + N - 1) % N, y);
                const double GY = At(x, (y + 1) % N) - At(x, (y + N - 1) % N);
                Sxx += GX * GX;
                Syy += GY * GY;
                Sxy += GX * GY;
            }
        return 0.5 * std::atan2(2.0 * Sxy, Sxx - Syy);
    }

    /** The mean over mip 0 of F(decoded value of Channel A, of Channel B). */
    template <typename TFunc>
    double MeanOf2(const FSet& Set, const uint16_t* Texels, int ChannelA, int ChannelB, TFunc&& F)
    {
        const FChannel& CA = Set.Channel[ChannelA];
        const FChannel& CB = Set.Channel[ChannelB];
        const size_t Count = (size_t)Set.N * Set.N;
        double Sum = 0.0;
        for (size_t i = 0; i < Count; i++)
        {
            const size_t T = i * 4;
            Sum += F((double)CA.Min + ((double)CA.Max - (double)CA.Min) * (double)Texels[T + ChannelA] / 65535.0,
                (double)CB.Min + ((double)CB.Max - (double)CB.Min) * (double)Texels[T + ChannelB] / 65535.0);
        }
        return Sum / (double)Count;
    }

    template <typename TFunc>
    double MeanOf2(const FSet& Set, int ChannelA, int ChannelB, TFunc&& F) { return MeanOf2(Set, Set.Texels.data(), ChannelA, ChannelB, F); }

    /** A periodic Gaussian blur (sigma in texels) of an N x N field, rows then columns. */
    template <typename TParallelFor>
    std::vector<double> BlurPeriodic(const std::vector<double>& In, uint32_t N, double Sigma, TParallelFor&& ParallelFor)
    {
        const int32_t Radius = (int32_t)std::ceil(3.0 * Sigma);
        std::vector<double> W((size_t)(2 * Radius + 1));
        double Sum = 0.0;
        for (int32_t k = -Radius; k <= Radius; k++) Sum += W[(size_t)(k + Radius)] = std::exp(-0.5 * (double)k * k / (Sigma * Sigma));
        for (double& V : W) V /= Sum;
        std::vector<double> Tmp((size_t)N * N), Out((size_t)N * N);
        const int32_t P = (int32_t)N;
        ParallelFor(P, [&](int32_t y) {
            for (int32_t x = 0; x < P; x++)
            {
                double A = 0.0;
                for (int32_t k = -Radius; k <= Radius; k++) A += W[(size_t)(k + Radius)] * In[(size_t)y * N + (size_t)Wrap(x + k, P)];
                Tmp[(size_t)y * N + x] = A;
            }
        });
        ParallelFor(P, [&](int32_t y) {
            for (int32_t x = 0; x < P; x++)
            {
                double A = 0.0;
                for (int32_t k = -Radius; k <= Radius; k++) A += W[(size_t)(k + Radius)] * Tmp[(size_t)Wrap(y + k, P) * N + x];
                Out[(size_t)y * N + x] = A;
            }
        });
        return Out;
    }

    /** The plugin's own `peaks` set, standing in for the eroded sets (Design/NoiseTiles2D_
     *  Addendum2.md) in their layout, without a simulation: R hip-roof pyramids (cellular border
     *  distance at a threshold slope, so planar faces and straight aretes, valleys on the cell
     *  borders) of three octaves, each pyramid's height varied by its cell; G the same blurred over
     *  an eighth of the massif spacing and lowered (rounded, gentler: soft rock); B R blurred over
     *  a quarter of it (the steering); A no flow (a constant). Heights in mip-0 texels. Massif
     *  spacing S = N / 16 texels. */
    template <typename TParallelFor>
    FSet GeneratePeaksStandIn(uint32_t Seed, uint32_t N, TParallelFor&& ParallelFor)
    {
        FSet Set;
        Set.Name = "peaks";
        Set.LookTag = "generated stand-in: pyramids, no erosion";
        Set.Seed = Seed;
        Set.N = N;
        constexpr uint32_t Massifs = 16;
        const double S = (double)N / Massifs;
        constexpr double Slope = 0.84;          // the hard-rock threshold slope (40 degrees)
        // The octaves' weights: the faces' slopes add up to about the threshold, and the relief
        // to about half a spacing at it (as an eroded range's: valleys a spacing apart).
        const double Weights[3] = { 0.6, 0.27, 0.12 };
        std::vector<double> Level[4];
        for (int c = 0; c < 4; c++) Level[c].assign((size_t)N * N, 0.0);
        ParallelFor((int32_t)N, [&](int32_t y) {
            for (uint32_t x = 0; x < N; x++)
            {
                double H = 0.0;
                for (int o = 0; o < 3; o++)
                {
                    FChannel C;
                    C.Kind = KindCellularBorder;
                    C.Seed = Mix(Seed + 11u * (uint32_t)o + 1u);
                    C.CellsPerTile = Massifs << o;
                    C.Jitter = 0.85f;
                    const double K = (double)C.CellsPerTile / (double)N;
                    const double Border = Evaluate(C, (double)x * K, (double)y * K);
                    C.Kind = KindCellularValue;
                    const double Value = Evaluate(C, (double)x * K, (double)y * K);
                    // Cell units to texels, at the threshold slope.
                    H += Weights[o] * (0.6 + 0.8 * Value) * Border * (S / (double)(1 << o)) * Slope;
                }
                Level[0][(size_t)y * N + x] = H;
            }
        });
        Level[1] = BlurPeriodic(Level[0], N, S / 8.0, ParallelFor);
        for (double& V : Level[1]) V *= 0.7;
        Level[2] = BlurPeriodic(Level[0], N, S / 4.0, ParallelFor);
        FChannel& R = Set.Channel[0];
        R.Kind = KindErodedHeight; R.Seed = Seed; R.CellsPerTile = Massifs;
        FChannel& G = Set.Channel[1];
        G = R;
        FChannel& B = Set.Channel[2];
        B.Kind = KindBlurred; B.Source = 0; B.CellsPerTile = 0;
        FChannel& A = Set.Channel[3];
        A.Kind = KindFlowAccumulation; A.Source = 0; A.CellsPerTile = 0;
        Finish(Set, Level);
        return Set;
    }

    /** The detail sets' layout (Design/NoiseTiles2D_Handoff.md section 6; the land's detail,
     *  N1: VoxelLandDetail.ush, and the layers' detail, VoxelLandPeaks.ush): R an fbm (soft rock's
     *  detail), G a ridged multifractal (hard rock's), B a billow (unused), A an eroded fbm (the
     *  detail layer), each DetailCells coarsest cells a side, DetailOctaves octaves, gain 0.5.
     *  Unitless: the plugin scales R and G to texel heights at an RMS slope (DetailSlopeSoft,
     *  DetailSlopeHard: a texel unit gradient, so 0.35 is 19 degrees), then within
     *  DetailRangeWaves of a coarsest wavelength of their mean (VoxelLand::PeakReach's bound);
     *  A to the detail layer's unit by its standard deviation (DetailStatsOf). */
    constexpr uint32_t DetailCells = 16;
    constexpr uint32_t DetailOctaves = 4;
    constexpr double DetailSlopeHard = 0.35;
    constexpr double DetailSlopeSoft = 0.22;
    constexpr double DetailRangeWaves = 0.1875;
    // The detail layer's unit: A over its standard deviation times this (the procedural fBm's
    // normalisation before N1). Its scales are 2^DetailOctaves apart in wavelength, each A's gain
    // to the power DetailOctaves lower (continuing the set's own spectrum); the face blend keeps
    // the variance (over the root of the weights' squares), so at a cube corner the three faces'
    // lookups can sum to sqrt(3) of one's bound.
    constexpr double DetailLayerSigma = 0.4;
    constexpr double DetailBlendBound = 1.7320508075688772;

    /** The plugin's own detail set in the detail sets' layout, standing in when the NoiseSets
     *  folder has none (a different hashing from the utility's, but the same kinds). */
    template <typename TParallelFor>
    FSet GenerateDetailStandIn(uint32_t Seed, uint32_t N, TParallelFor&& ParallelFor)
    {
        FSet Set;
        Set.Name = "detail";
        Set.LookTag = "generated stand-in: fbm, ridged, billow, eroded fbm";
        Set.Seed = Seed;
        Set.N = N;
        const uint32_t Kinds[4] = { KindFbm, KindRidged, KindBillow, KindErodedFbm };
        for (int c = 0; c < 4; c++)
        {
            FChannel& C = Set.Channel[c];
            C.Kind = Kinds[c];
            C.Seed = Mix(Seed + 0x9e3779b9u * (uint32_t)(c + 1));
            C.CellsPerTile = DetailCells;
            C.Octaves = DetailOctaves;
            C.Gain = 0.5f;
            C.ErosionK = Kinds[c] == KindErodedFbm ? 1.0f : 0.0f;
        }
        Generate(Set, ParallelFor);
        return Set;
    }

    /** Whether Set has the detail sets' layout: R an fbm, G a ridged multifractal, A an eroded fbm
     *  (B isn't read), at one coarsest lattice dividing N, its wavelength (N / cells) 16 to 512
     *  texels, each DetailOctaves octaves at lacunarity 2 (the detail layer's scales continue each
     *  other on that), A's gain within 0.3 .. 0.7. "" if so, else why not. */
    inline std::string CheckDetail(const FSet& Set)
    {
        const FChannel& R = Set.Channel[0];
        const FChannel& G = Set.Channel[1];
        const FChannel& A = Set.Channel[3];
        if (R.Kind != KindFbm || G.Kind != KindRidged || A.Kind != KindErodedFbm)
            return "R, G and A aren't fbm, ridged and eroded fbm (kinds " + std::to_string(R.Kind) + ", " + std::to_string(G.Kind) + ", "
                + std::to_string(A.Kind) + "): not a detail set?";
        const uint32_t Cells = R.CellsPerTile;
        if (G.CellsPerTile != Cells || A.CellsPerTile != Cells) return "R, G and A have different cell counts";
        if (Cells == 0u || Set.N % Cells != 0u) return "the cell count (" + std::to_string(Cells) + ") doesn't divide the size";
        if (Set.N / Cells < 16u || Set.N / Cells > 512u) return "the coarsest wavelength (" + std::to_string(Set.N / Cells) + " texels) isn't within 16 .. 512";
        for (const FChannel* C : { &R, &G, &A })
        {
            if (C->Octaves != DetailOctaves) return "a channel has " + std::to_string(C->Octaves) + " octaves (" + std::to_string(DetailOctaves) + " expected)";
            if (std::fabs(C->Lacunarity - 2.0f) > 1e-3f) return "a channel's lacunarity is " + std::to_string(C->Lacunarity) + " (2 expected)";
        }
        if (!(A.Gain >= 0.3f && A.Gain <= 0.7f)) return "A's gain " + std::to_string(A.Gain) + " isn't within 0.3 .. 0.7";
        return std::string();
    }

    /** What the plugin places and normalises a detail set with (CheckDetail's layout): hard
     *  rock's (G) and soft rock's (R) means (unit values) and ranges (Max - Min in texel heights:
     *  scaled to DetailSlopeHard and DetailSlopeSoft RMS slopes over mip 0, by central
     *  differences, and within DetailRangeWaves of a wavelength of their means); A's mean and
     *  standard deviation (unit values), the detail layer's gain from one scale to the next (A's
     *  gain to the power DetailOctaves) and its bound (its largest |unit|). */
    struct FDetailStats
    {
        double MeanHard = 0.0;
        double MeanSoft = 0.0;
        double RangeHard = 0.0;
        double RangeSoft = 0.0;
        double MeanA = 0.0;
        double StdA = 0.0;
        double ScaleGain = 0.0;
        double BoundA = 0.0;
    };

    inline FDetailStats DetailStatsOf(const FSet& Set, const uint16_t* Texels)
    {
        FDetailStats Out;
        const uint32_t N = Set.N;
        const size_t Count = (size_t)N * N;
        const double Spacing = (double)(N / std::max(Set.Channel[0].CellsPerTile, 1u));
        for (int c = 0; c < 2; c++)
        {
            const FChannel& C = Set.Channel[c];
            const double Span = (double)C.Max - (double)C.Min;
            auto At = [&](uint32_t x, uint32_t y) { return (double)Texels[((size_t)y * N + x) * 4 + c] / 65535.0; };
            double Sum = 0.0, Sum2 = 0.0, Lo = 1.0, Hi = 0.0;
            for (uint32_t y = 0; y < N; y++)
                for (uint32_t x = 0; x < N; x++)
                {
                    const double U = At(x, y);
                    const double GX = 0.5 * (At((x + 1) % N, y) - At((x + N - 1) % N, y));
                    const double GY = 0.5 * (At(x, (y + 1) % N) - At(x, (y + N - 1) % N));
                    Sum += U;
                    Sum2 += GX * GX + GY * GY;
                    Lo = std::min(Lo, U);
                    Hi = std::max(Hi, U);
                }
            const double Mean = Sum / (double)Count;
            // In decoded units, then to texel heights.
            const double Rms = std::sqrt(Sum2 / (double)Count) * Span;
            const double Spread = std::max(Mean - Lo, Hi - Mean) * Span;
            double Scale = (c == 1 ? DetailSlopeHard : DetailSlopeSoft) / std::max(Rms, 1e-12);
            if (Spread * Scale > DetailRangeWaves * Spacing) Scale = DetailRangeWaves * Spacing / std::max(Spread, 1e-12);
            (c == 1 ? Out.MeanHard : Out.MeanSoft) = Mean;
            (c == 1 ? Out.RangeHard : Out.RangeSoft) = Span * Scale;
        }
        double Sum = 0.0, Sum2 = 0.0, Lo = 1.0, Hi = 0.0;
        for (size_t i = 0; i < Count; i++)
        {
            const double U = (double)Texels[i * 4 + 3] / 65535.0;
            Sum += U;
            Sum2 += U * U;
            Lo = std::min(Lo, U);
            Hi = std::max(Hi, U);
        }
        Out.MeanA = Sum / (double)Count;
        Out.StdA = std::sqrt(std::max(Sum2 / (double)Count - Out.MeanA * Out.MeanA, 0.0));
        Out.ScaleGain = std::pow((double)Set.Channel[3].Gain, (double)DetailOctaves);
        // The B-splines are convex (each scale within the texels' range about the mean), the
        // scales' gains a geometric series, the face blend at most DetailBlendBound.
        Out.BoundA = Out.StdA > 0.0 && Out.ScaleGain < 1.0
            ? DetailLayerSigma * std::max(Out.MeanA - Lo, Hi - Out.MeanA) / Out.StdA / (1.0 - Out.ScaleGain) * DetailBlendBound : 0.0;
        return Out;
    }

    /** Whether Set has the `peaks` layout VoxelLandPeaks.ush reads (Design/NoiseTiles2D_
     *  Addendum2.md): R and G eroded heights of one tile (the same massif count, dividing N), B a
     *  blur of R, A R's flow accumulation; a massif spacing (N / cells) of 32 to 512 texels. ""
     *  if so, else why not. */
    inline std::string CheckPeaks(const FSet& Set)
    {
        const FChannel& R = Set.Channel[0];
        const FChannel& G = Set.Channel[1];
        const FChannel& B = Set.Channel[2];
        const FChannel& A = Set.Channel[3];
        if (R.Kind != KindErodedHeight || G.Kind != KindErodedHeight)
            return "R and G aren't eroded heights (kinds " + std::to_string(R.Kind) + ", " + std::to_string(G.Kind) + "): an older peaks layout?";
        if (B.Kind != KindBlurred || B.Source != 0) return "B isn't R blurred (kind " + std::to_string(B.Kind) + ")";
        if (A.Kind != KindFlowAccumulation || A.Source != 0) return "A isn't R's flow accumulation (kind " + std::to_string(A.Kind) + ")";
        const uint32_t Cells = R.CellsPerTile;
        if (G.CellsPerTile != Cells) return "R and G have different massif counts";
        if (Cells == 0u || Set.N % Cells != 0u) return "R's massif count (" + std::to_string(Cells) + ") doesn't divide the size";
        if (Set.N / Cells < 32u || Set.N / Cells > 512u) return "R's massif spacing (" + std::to_string(Set.N / Cells) + " texels) isn't within 32 .. 512";
        return std::string();
    }

    /** Whether Set has the `dunes` layout VoxelLandDunes.ush reads (stage T5c): R and G simulated
     *  heights of one dune field (sharp and rounded crests, the slip faces toward +x; the same
     *  crest count, dividing N), a crest spacing (N / cells) of 16 to 512 texels; B and A aren't
     *  read. "" if so, else why not. */
    inline std::string CheckDunes(const FSet& Set)
    {
        const FChannel& R = Set.Channel[0];
        const FChannel& G = Set.Channel[1];
        if (R.Kind != KindErodedHeight || G.Kind != KindErodedHeight)
            return "R and G aren't simulated heights (kinds " + std::to_string(R.Kind) + ", " + std::to_string(G.Kind) + "): not a dunes set?";
        const uint32_t Cells = R.CellsPerTile;
        if (G.CellsPerTile != Cells) return "R and G have different crest counts";
        if (Cells == 0u || Set.N % Cells != 0u) return "R's crest count (" + std::to_string(Cells) + ") doesn't divide the size";
        if (Set.N / Cells < 16u || Set.N / Cells > 512u) return "R's crest spacing (" + std::to_string(Set.N / Cells) + " texels) isn't within 16 .. 512";
        if (!(R.Max > R.Min) || !(G.Max > G.Min)) return "R or G is flat";
        return std::string();
    }

    /** Whether Set has the `sanddetail` layout's G VoxelLandDunes.ush reads (stage T5c): a
     *  simulated height (the megaripples, crests across +x) with a wavelength (N / its cells, which
     *  need not divide N) of 8 to 512 texels. "" if so, else why not. */
    inline std::string CheckSandDetail(const FSet& Set)
    {
        const FChannel& G = Set.Channel[1];
        if (G.Kind != KindErodedHeight) return "G isn't a simulated height (kind " + std::to_string(G.Kind) + "): not a sanddetail set?";
        if (G.CellsPerTile == 0u || Set.N < 8u * G.CellsPerTile || Set.N > 512u * G.CellsPerTile)
            return "G's wavelength (" + std::to_string(G.CellsPerTile > 0u ? Set.N / G.CellsPerTile : 0u) + " texels) isn't within 8 .. 512";
        if (!(G.Max > G.Min)) return "G is flat";
        return std::string();
    }

    /** Whether Set has the `polar` layout VoxelLandPolar.ush reads (stage T5d3): R crevasses, G
     *  sastrugi and B ice-wedge polygons, each a simulated height (A, sea ice, is stage T5e's: not
     *  checked) with a cell (N / its cells, which need not divide N) of 8 to 512 texels and some
     *  relief. "" if so, else why not. */
    inline std::string CheckPolar(const FSet& Set)
    {
        const char* Names[3] = { "R (crevasses)", "G (sastrugi)", "B (ice-wedge polygons)" };
        for (int c = 0; c < 3; c++)
        {
            const FChannel& C = Set.Channel[c];
            if (C.Kind != KindErodedHeight)
                return std::string(Names[c]) + " isn't a simulated height (kind " + std::to_string(C.Kind) + "): not a polar set?";
            if (C.CellsPerTile == 0u || Set.N < 8u * C.CellsPerTile || Set.N > 512u * C.CellsPerTile)
                return std::string(Names[c]) + "'s cell (" + std::to_string(C.CellsPerTile > 0u ? Set.N / C.CellsPerTile : 0u) + " texels) isn't within 8 .. 512";
            if (!(C.Max > C.Min)) return std::string(Names[c]) + " is flat";
        }
        return std::string();
    }

    /** A foam set (stage W3g): R lace, G grain, B whitecap, A streak. "" if usable, else why not. */
    inline std::string CheckFoam(const FSet& Set)
    {
        const uint32_t Want[4] = { KindFoamLace, KindFoamGrain, KindFoamWhitecap, KindFoamStreak };
        const char* Names[4] = { "R", "G", "B", "A" };
        for (int c = 0; c < 4; c++)
        {
            if (Set.Channel[c].Kind != Want[c])
                return std::string(Names[c]) + " has kind " + std::to_string(Set.Channel[c].Kind) + " (a foam set's is " + std::to_string(Want[c]) + ")";
        }
        if (Set.N < 64u) return "size " + std::to_string(Set.N) + " (a foam set needs at least 64)";
        return std::string();
    }

    // --- The noise generator's files (.vxnt v3) ----------------------------------------------
    //
    // Little-endian throughout (as every platform UE targets). As peaks_a.vxnt (NoiseTile2D 2.0)
    // lays it out; keep in step with the utility's byte-layout note:
    //   0    char[4]  "VXNT"
    //   4    u32      version (3)
    //   8    u32      N
    //   12   u32      mip count (log2 N + 1)
    //   16   u32      channel count (4)
    //   20   u32      encoding (0: UNORM16; float16 sets aren't read)
    //   24   u32      the set's seed
    //   28   u32      the texel data's offset (the header's size: 128 + 4 x 64 + 16 x mips)
    //   32   char[32] name (NUL-padded)
    //   64   char[64] look tag (NUL-padded)
    //   128  4 x 64   channels, R G B A: kind, seed, cells per tile, octaves (u32); lacunarity,
    //                 gain, jitter, erosion k (f32); source channel (i32, -1 none); decode min,
    //                 max, mean, standard deviation (f32); 12 bytes reserved
    //   384  mips x 16  per mip: size (u32), reserved (u32), offset in the file (u64)
    //   then the texels: the mips in order, each row-major, 4 u16 a texel (R, G, B, A); value =
    //   Min + (Max - Min) q / 65535, one range per channel for every mip.

    namespace VxntDetail
    {
        inline uint32_t U32(const uint8_t* P) { return (uint32_t)P[0] | ((uint32_t)P[1] << 8) | ((uint32_t)P[2] << 16) | ((uint32_t)P[3] << 24); }
        inline uint64_t U64(const uint8_t* P) { return (uint64_t)U32(P) | ((uint64_t)U32(P + 4) << 32); }
        inline float F32(const uint8_t* P) { const uint32_t U = U32(P); float F; std::memcpy(&F, &U, 4); return F; }
        inline std::string Text(const uint8_t* P, size_t Max)
        {
            size_t L = 0;
            while (L < Max && P[L] != 0) L++;
            return std::string((const char*)P, L);
        }
    }

    /** Reads a .vxnt v3 set's header (its first HeaderBytes bytes: at least 384 + 16 x mips;
     *  4 KB covers any size) into Out, all but the texels, and checks it against the file's size
     *  FileSize. Returns "" on success (OutDataOffset: where the texels start, ChainTexels(N) x 8
     *  bytes), else what's wrong (Out is then unchanged). Checks everything the sampler relies on:
     *  a power-of-two size, the whole chain, packed in order, the decode ranges. */
    inline std::string ReadVxntHeader(const uint8_t* Data, size_t HeaderBytes, uint64_t FileSize, FSet& Out, uint64_t& OutDataOffset)
    {
        using namespace VxntDetail;
        constexpr size_t Fixed = 128 + 4 * 64;
        if (!Data || HeaderBytes < Fixed || FileSize < Fixed) return "too short for a header";
        if (std::memcmp(Data, "VXNT", 4) != 0) return "not a .vxnt file";
        const uint32_t Version = U32(Data + 4);
        if (Version != 3u) return "version " + std::to_string(Version) + " (this reader reads 3)";
        const uint32_t N = U32(Data + 8);
        const uint32_t Mips = U32(Data + 12);
        const uint32_t Channels = U32(Data + 16);
        const uint32_t Encoding = U32(Data + 20);
        const uint32_t DataOffset = U32(Data + 28);
        // Up to 4096: the GPU buffer stays under 180 MB (a 2048 set is 45 MB).
        if (N < 4u || N > 4096u || (N & (N - 1u)) != 0u) return "size " + std::to_string(N) + " isn't a power of two in 4 .. 4096";
        uint32_t Full = 0;
        for (uint32_t S = N; S > 0; S >>= 1) Full++;
        if (Mips != Full) return std::to_string(Mips) + " mips (the whole chain is " + std::to_string(Full) + ")";
        if (Channels != 4u) return std::to_string(Channels) + " channels (4 expected)";
        if (Encoding != 0u) return "encoding " + std::to_string(Encoding) + " (only UNORM16, 0, is read)";
        const size_t Header = Fixed + 16 * (size_t)Mips;
        if (HeaderBytes < Header) return "the header was cut short";
        if (DataOffset < Header) return "the texel data's offset " + std::to_string(DataOffset) + " is inside the header";
        const uint64_t Texels = ChainTexels(N);
        if ((uint64_t)DataOffset + Texels * 8u > FileSize) return "the file is shorter than its mip chain";
        for (uint32_t m = 0; m < Mips; m++)
        {
            const uint8_t* P = Data + Fixed + 16 * (size_t)m;
            if (U32(P) != (N >> m)) return "mip " + std::to_string(m) + "'s size is " + std::to_string(U32(P));
            if (U64(P + 8) != (uint64_t)DataOffset + MipOffset(N, m) * 8u) return "mip " + std::to_string(m) + " isn't packed after the mip above";
        }
        FSet Set;
        Set.Name = Text(Data + 32, 32);
        Set.LookTag = Text(Data + 64, 64);
        Set.Seed = U32(Data + 24);
        Set.N = N;
        Set.Mips = Mips;
        for (int c = 0; c < 4; c++)
        {
            const uint8_t* P = Data + 128 + 64 * (size_t)c;
            FChannel& C = Set.Channel[c];
            C.Kind = U32(P);
            C.Seed = U32(P + 4);
            C.CellsPerTile = U32(P + 8);
            C.Octaves = U32(P + 12);
            C.Lacunarity = F32(P + 16);
            C.Gain = F32(P + 20);
            C.Jitter = F32(P + 24);
            C.ErosionK = F32(P + 28);
            C.Source = (int32_t)U32(P + 32);
            C.Min = F32(P + 36);
            C.Max = F32(P + 40);
            C.Mean = F32(P + 44);
            C.Std = F32(P + 48);
            if (C.Kind > KindFoamStreak) return "channel " + std::to_string(c) + " has kind " + std::to_string(C.Kind);
            if (!(C.Max > C.Min) || !std::isfinite(C.Min) || !std::isfinite(C.Max)) return "channel " + std::to_string(c) + " has no decode range";
        }
        Out = std::move(Set);
        OutDataOffset = DataOffset;
        return std::string();
    }

    /** Reads a whole .vxnt v3 file from its bytes into Out (ReadVxntHeader, then the texels). */
    inline std::string ReadVxnt(const uint8_t* Data, size_t Size, FSet& Out)
    {
        FSet Set;
        uint64_t DataOffset = 0;
        const std::string Error = ReadVxntHeader(Data, Size, Size, Set, DataOffset);
        if (!Error.empty()) return Error;
        // The texels as they lie (u16 little-endian: the platform's own order).
        Set.Texels.resize((size_t)ChainTexels(Set.N) * 4);
        std::memcpy(Set.Texels.data(), Data + DataOffset, Set.Texels.size() * 2);
        Out = std::move(Set);
        return std::string();
    }
}
