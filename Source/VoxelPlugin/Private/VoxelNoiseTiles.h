// VoxelNoiseTiles.h - Packed tiling noise sets (Design/NoiseTiles2D_Handoff.md and its Addendum 1):
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
// plugin's own generator (PeaksDefinition, Generate): the same channel kinds, defined as in the
// handoff (its hashing differs from the utility's, so its patterns do). Plain C++ without engine
// types: the harnesses run the same reader and generator.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace VoxelNoiseTiles
{
    /** Channel kinds (the handoff's section 3). */
    enum EKind : uint32_t
    {
        KindNone = 0,
        KindFbm = 1,
        KindRidged = 2,
        KindBillow = 3,
        KindCellularBorder = 4,
        KindCellularValue = 5,
        KindErodedFbm = 6,
    };

    struct FChannel
    {
        uint32_t Kind = KindNone;
        uint32_t Seed = 0;
        uint32_t CellsPerTile = 1;   // the coarsest lattice's cells along a side (a power of two)
        uint32_t Octaves = 1;
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

    // --- Generation (double precision, integer hashing: the same bytes on every machine) ------

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

    /** Fills Set's texels and statistics from its channels' definitions. ParallelFor(Count,
     *  Body) runs Body(i) for i in 0 .. Count - 1 (in any order, on any threads). */
    template <typename TParallelFor>
    void Generate(FSet& Set, TParallelFor&& ParallelFor)
    {
        const uint32_t N = Set.N;
        Set.Mips = 0;
        for (uint32_t S = N; S > 0; S >>= 1) Set.Mips++;
        Set.Texels.assign((size_t)ChainTexels(N) * 4, 0);
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

    /** The `peaks` set (the handoff's section 6, at half its size: N = 1024, the same texels per
     *  feature): R the cellular border distance (32 texels a cell), G a ridged multifractal, B a
     *  smooth fBm, A the cellular value of R's cells. */
    inline FSet PeaksDefinition(uint32_t Seed, uint32_t N = 1024)
    {
        FSet Set;
        Set.Name = "peaks";
        Set.Seed = Seed;
        Set.N = N;
        const uint32_t Cells = std::max(N / 32u, 1u);
        FChannel& R = Set.Channel[0];
        R.Kind = KindCellularBorder; R.Seed = Mix(Seed + 1u); R.CellsPerTile = Cells; R.Jitter = 0.85f;
        FChannel& G = Set.Channel[1];
        G.Kind = KindRidged; G.Seed = Mix(Seed + 2u); G.CellsPerTile = std::max(Cells / 4u, 1u); G.Octaves = 4; G.Gain = 0.5f;
        FChannel& B = Set.Channel[2];
        B.Kind = KindFbm; B.Seed = Mix(Seed + 3u); B.CellsPerTile = std::max(Cells / 8u, 1u); B.Octaves = 3; B.Gain = 0.5f;
        FChannel& A = Set.Channel[3];
        A = R;
        A.Kind = KindCellularValue;
        A.Source = 0;
        return Set;
    }

    /** Whether Set has the `peaks` layout VoxelLandPeaks.ush reads: R a cellular border, G a
     *  ridged multifractal, A the cellular value of R's own sites (the same seed, cells and
     *  jitter: constant over each pyramid; B is free), R's cells a power of two of 16 to 128
     *  texels (the peaks' mip levels are tuned for 32 and shift with it). "" if so, else why not. */
    inline std::string CheckPeaks(const FSet& Set)
    {
        const FChannel& R = Set.Channel[0];
        const FChannel& G = Set.Channel[1];
        const FChannel& A = Set.Channel[3];
        if (R.Kind != KindCellularBorder) return "R isn't cellular_border (kind " + std::to_string(R.Kind) + ")";
        if (G.Kind != KindRidged) return "G isn't ridged (kind " + std::to_string(G.Kind) + ")";
        if (A.Kind != KindCellularValue) return "A isn't cellular_value (kind " + std::to_string(A.Kind) + ")";
        if (A.Source != 0 || A.Seed != R.Seed || A.CellsPerTile != R.CellsPerTile || A.Jitter != R.Jitter) return "A doesn't share R's sites";
        const uint32_t Cells = R.CellsPerTile;
        if (Cells == 0u || (Cells & (Cells - 1u)) != 0u || Set.N % Cells != 0u || Set.N / Cells < 16u || Set.N / Cells > 128u)
            return "R's " + std::to_string(Cells) + " cells don't give a power of two of 16 to 128 texels each";
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
            C.Gain = F32(P + 20);
            C.Jitter = F32(P + 24);
            C.ErosionK = F32(P + 28);
            C.Source = (int32_t)U32(P + 32);
            C.Min = F32(P + 36);
            C.Max = F32(P + 40);
            C.Mean = F32(P + 44);
            C.Std = F32(P + 48);
            if (C.Kind > KindErodedFbm) return "channel " + std::to_string(c) + " has kind " + std::to_string(C.Kind);
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
