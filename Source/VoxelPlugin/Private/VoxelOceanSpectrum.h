// VoxelOceanSpectrum.h - The ocean's FFT cascades on the CPU (Design/OceanWaves.md sections 2
// and 7): the slot layout both sides build from, the CPU's FFT, and the long-band cascades the
// surface query samples. The spectrum, the dispersion and the butterflies are the shared code
// of VoxelOceanSpectrum.ush, which the GPU passes (VoxelOceanFFT.usf) run too, so the CPU
// evolves the same numbers the GPU draws.
//
// THE LAYOUT (MakeSlots): eight slots on the GPU, seven of 256^2 and the shortest band's of 512^2
// (stage W3z: SizeOfBand; VOXEL_OCEAN_FFT_LONG_SIZE and _FINE_SIZE, VoxelOceanSpectrum.ush).
// - Four bands with tile sides about 1,000, 250, 60 and 14 m, chosen non-commensurate (997,
//   241, 59, 13.7 by default). Band b keeps the modes with BoundaryK(b) <= |k| < BoundaryK(b+1):
//   the lower edge is BoundaryModes (4) times the band's own fundamental, so a band never uses
//   its sparse first rings, which the longer band covers; the top of the last band keeps
//   TopSamples (4) samples of its own grid per wavelength (10.7 cm on 512^2). The bottom of band
//   0 is its fundamental.
// - The two long bands (0, 1) at three sea states (winds of 5, 10 and 18 m/s by default): slots
//   0-5, state-major (slot = 2 state + band). The short bands (2, 3) once, at ShortWind: slots
//   6 and 7. Sea states of one band share its seed, so they hold the same waves at different
//   strengths and blend coherently.
//
// THE QUERY CASCADES (FQueryCascade): a slot on a smaller grid (bands 0-2 only hold modes up to
// about 18 grid steps, so 128^2 holds them exactly), as the six fields the query needs (height,
// horizontal displacement and their time derivatives), sampled by periodic Catmull-Rom with
// analytic gradients. The query uses the long bands by default (Design/OceanWaves.md section 9,
// decision 6); band 2 fits the same way when calm seas need it (it carries most of a sea below
// 5 m/s).
//
// Plain C++ (std containers, no engine types): it is included inside the plugin and by the
// harness (Tools/OceanHarness/spectrum_check.cpp). Compile without fused multiply-add
// contraction (MSVC /fp:precise, the UE default; -ffp-contract=off): the dispersion's exact
// test depends on it.

#pragma once

#include "VoxelShaderCompat.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace VoxelShader
{
#include "VoxelOceanSpectrum.ush"
}

namespace VoxelOceanSpectrum
{
    /** The long arrays' grid (slots 0-6, and the shortest band's copy at half its resolution in
     *  slice 7); the shortest band's own (stage W3z). */
    constexpr int GPUSize = VOXEL_OCEAN_FFT_LONG_SIZE;
    constexpr int FineSize = VOXEL_OCEAN_FFT_FINE_SIZE;
    constexpr int SlotCount = 8;
    constexpr int BandCount = 4;
    constexpr int StateCount = 3;
    constexpr double Pi = 3.14159265358979323846;
    constexpr double Gravity = 9.80665;
    constexpr double LoopPeriod = 1024.0;
    constexpr double Omega0 = 2.0 * Pi / LoopPeriod;
    static_assert(FineSize == 2 * GPUSize, "the long arrays' slice 7 is the fine grid's mip 1");

    /** A band's grid on the GPU (stage W3z): the shortest band's its own, finer. */
    inline constexpr int SizeOfBand(int Band) { return Band == BandCount - 1 ? FineSize : GPUSize; }

    /** The layout's settings (FVoxelOceanSettings feeds them in O6b). */
    struct FLayoutSettings
    {
        double TileLength[BandCount] = { 997.0, 241.0, 59.0, 13.7 };   // m
        double BoundaryModes = 4.0;     // a band's lower edge, in its own fundamentals
        double TopSamples = 4.0;        // grid samples per wavelength at the top of the last band
        double StateWind[StateCount] = { 5.0, 10.0, 18.0 };            // m/s, the long bands' sea states
        double ShortWind = 10.0;        // m/s, the short bands' spectrum
        double Fetch = 300000.0;        // m (capped at a fully developed sea)
        double Gamma = 3.3;             // JONSWAP peak enhancement
        double Swell = 0.3;             // Horvath's swell, 0-1
        double Chop[BandCount] = { 0.9, 0.9, 0.9, 0.9 };              // horizontal displacement per D
        /** The wave equalizer (stage W3k): gains on the height at the wavelengths 2^n m, n from
         *  VOXEL_OCEAN_EQ_LOW (25 cm) up, one per octave (1: as the spectrum). */
        double Gain[VOXEL_OCEAN_EQ_KNOTS] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        uint32_t PlanetSeed = 1u;
    };

    /** One slot: its shared params and what the GPU table adds. */
    struct FSlot
    {
        VoxelShader::FVoxelOceanSlotParams Params;
        float Chop = 0.9f;
        int Band = 0;
        int State = -1;             // -1: a short band (one spectrum at ShortWind)
        double KMin = 0.0;          // rad/m, the band's physical edges
        double KMax = 0.0;
    };

    /** The slot as the GPU's structured buffer holds it (FVoxelOceanSlotGPU, VoxelOceanFFT.usf). */
    struct FSlotGPU
    {
        float A[4];     // tile length, wind speed, fetch, gamma
        float B[4];     // swell, scale, 0, chop
        uint32_t C[4];  // MMin, MMax, seed, dispersion C2 (packed)
        float G[16];    // the equalizer's gains, knots 0-12 (13-15 unused, 0)
    };
    static_assert(sizeof(FSlotGPU) == 112, "FSlotGPU must match the shader's struct");
    static_assert(VOXEL_OCEAN_EQ_KNOTS <= 16, "the slot table holds 16 gains");

    inline double BoundaryK(const FLayoutSettings& S, int Band)
    {
        if (Band <= 0) return 2.0 * Pi / S.TileLength[0];
        if (Band >= BandCount) return 2.0 * Pi * double(SizeOfBand(BandCount - 1)) / (S.TileLength[BandCount - 1] * S.TopSamples);
        return S.BoundaryModes * 2.0 * Pi / S.TileLength[Band];
    }

    /** The seed of a band (shared by its sea states). */
    inline uint32_t BandSeed(uint32_t PlanetSeed, int Band)
    {
        return VoxelShader::VoxelOceanHash(PlanetSeed ^ VoxelShader::VoxelOceanHash(0x0cea0000u + uint32_t(Band)));
    }

    /** The integer band bound of a wavenumber on a tile: the smallest m with sqrt(m) dk >= K. */
    inline uint32_t BandBound(double K, double TileLength)
    {
        const double R = K * TileLength / (2.0 * Pi);
        double M = std::ceil(R * R - 1.0e-9);
        if (M < 1.0) M = 1.0;
        return uint32_t(M);
    }

    /** C2 as the shaders take it: a 24-bit mantissa | (exponent + 128) << 24, C2 = mant 2^exp. */
    inline uint32_t PackDispersion(double C2)
    {
        int E = 0;
        const double F = std::frexp(C2, &E);                // C2 = F 2^E, F in [0.5, 1)
        double Mant = std::round(F * 16777216.0);
        int Exp = E - 24;
        if (Mant >= 16777216.0) { Mant = 8388608.0; Exp += 1; }
        return uint32_t(Mant) | (uint32_t(Exp + 128) << 24);
    }

    inline double UnpackDispersion(uint32_t Packed)
    {
        return std::ldexp(double(Packed & 0xffffffu), int(Packed >> 24) - 128);
    }

    inline FSlot MakeSlot(const FLayoutSettings& S, int Band, int State, double Wind)
    {
        FSlot Slot;
        const double L = S.TileLength[Band];
        Slot.Band = Band;
        Slot.State = State;
        Slot.KMin = BoundaryK(S, Band);
        Slot.KMax = BoundaryK(S, Band + 1);
        Slot.Chop = float(S.Chop[Band]);
        const double C = Gravity * 2.0 * Pi / (L * Omega0 * Omega0);
        const uint32_t C2 = PackDispersion(C * C);
        // The band stays inside the grid's half (no Nyquist mode: m < (N/2)^2) and its
        // frequencies within the exact test's range (n <= VOXEL_OCEAN_MAX_MULTIPLE: C2 m below
        // (n + 1/2)^4). An inverted band is empty (MMax = MMin).
        uint32_t MMin = BandBound(Slot.KMin, L);
        uint32_t MMax = BandBound(Slot.KMax, L);
        const uint32_t Half = uint32_t(SizeOfBand(Band) / 2);
        MMax = std::min(MMax, Half * Half);
        const double Top = std::pow(double(VOXEL_OCEAN_MAX_MULTIPLE) + 0.5, 4.0) / UnpackDispersion(C2);
        MMax = uint32_t(std::min(double(MMax), std::floor(Top)));
        if (MMax < MMin) MMax = MMin;
        Slot.Params = VoxelShader::VoxelOceanMakeSlot(float(L), float(Wind), float(S.Fetch), float(S.Gamma),
            float(S.Swell), 1.0f, C2, MMin, MMax, BandSeed(S.PlanetSeed, Band));
        for (int i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++)
            Slot.Params.Gain[i] = float(std::max(S.Gain[i], 0.0));
        return Slot;
    }

    /** The eight slots (see THE LAYOUT). */
    inline std::vector<FSlot> MakeSlots(const FLayoutSettings& S)
    {
        std::vector<FSlot> Slots;
        for (int State = 0; State < StateCount; State++)
            for (int Band = 0; Band < 2; Band++)
                Slots.push_back(MakeSlot(S, Band, State, S.StateWind[State]));
        Slots.push_back(MakeSlot(S, 2, -1, S.ShortWind));
        Slots.push_back(MakeSlot(S, 3, -1, S.ShortWind));
        return Slots;
    }

    inline FSlotGPU ToGPU(const FSlot& Slot)
    {
        const VoxelShader::FVoxelOceanSlotParams& P = Slot.Params;
        FSlotGPU G;
        G.A[0] = P.TileLength; G.A[1] = P.WindSpeed; G.A[2] = P.Fetch; G.A[3] = P.Gamma;
        // B[2]: the foam's spread (sigma_J), filled by the FFT model from the slot's statistics.
        G.B[0] = P.Swell; G.B[1] = P.Scale; G.B[2] = 0.0f; G.B[3] = Slot.Chop;
        G.C[0] = P.MMin; G.C[1] = P.MMax; G.C[2] = P.Seed; G.C[3] = P.DispersionC2;
        for (int i = 0; i < 16; i++) G.G[i] = i < VOXEL_OCEAN_EQ_KNOTS ? P.Gain[i] : 0.0f;
        return G;
    }

    /** The ocean clock as the shaders take it: frac(t / T) 2^32 (t in s, any sign). */
    inline uint32_t TickOf(double Time)
    {
        double F = std::fmod(Time / LoopPeriod, 1.0);
        if (F < 0.0) F += 1.0;
        const double Ticks = std::floor(F * 4294967296.0);
        return Ticks >= 4294967296.0 ? 0u : uint32_t(Ticks);
    }

    /** What a slot's spectrum holds, from its modes (expected values over the random start):
     *  the height variance (m^2) and the slope variances along and across the wind. */
    struct FBandStats
    {
        double HeightVariance = 0.0;
        double SlopeVarianceX = 0.0;
        double SlopeVarianceY = 0.0;
        int Modes = 0;
    };

    inline FBandStats StatsOf(const FSlot& Slot)
    {
        using namespace VoxelShader;
        const FVoxelOceanSlotParams& P = Slot.Params;
        const FVoxelOceanJonswap J = VoxelOceanJonswapOf(P.WindSpeed, P.Fetch);
        const double Dk = 2.0 * Pi / P.TileLength;
        const int R = int(std::ceil(std::sqrt(double(P.MMax))));
        FBandStats Out;
        for (int Kj = -R; Kj <= R; Kj++)
        {
            for (int Ki = -R; Ki <= R; Ki++)
            {
                if (!VoxelOceanInBand(P, Ki, Kj)) continue;
                const double Kx = Ki * Dk, Ky = Kj * Dk;
                const double S = double(VoxelOceanSpectrumK(P, J, float(Kx), float(Ky))) * Dk * Dk * double(P.Scale) * double(P.Scale);
                Out.HeightVariance += S;
                Out.SlopeVarianceX += Kx * Kx * S;
                Out.SlopeVarianceY += Ky * Ky * S;
                Out.Modes++;
            }
        }
        return Out;
    }

    // -----------------------------------------------------------------------------------------
    // The CPU FFT
    // -----------------------------------------------------------------------------------------

    /** An inverse 2D FFT of size N (a power of two, 2-1024) over interleaved complex floats, in
     *  place, with the GPU's butterflies and twiddles (VoxelOceanButterflyOf, VoxelOceanTwiddle). */
    class FInverseFFT2D
    {
    public:
        explicit FInverseFFT2D(int InN) : N(InN)
        {
            Stages = 0;
            while ((1 << Stages) < N) Stages++;
            const uint32_t Half = uint32_t(N / 2);
            Butterflies.resize(size_t(Stages) * Half);
            uint32_t Ns = 1u;
            for (int Stage = 0; Stage < Stages; Stage++, Ns *= 2u)
                for (uint32_t J = 0; J < Half; J++)
                    Butterflies[size_t(Stage) * Half + J] = VoxelShader::VoxelOceanButterflyOf(Ns, J, Half);
            Twiddles.resize(Half);
            for (uint32_t J = 0; J < Half; J++) Twiddles[J] = VoxelShader::VoxelOceanTwiddle(J, Half);
            Line[0].resize(size_t(2 * N));
            Line[1].resize(size_t(2 * N));
        }

        int Size() const { return N; }

        /** Data: N x N complex numbers, row-major, (re, im) interleaved. RowModes >= 0: only the
         *  rows of signed mode |Kj| <= RowModes hold anything (a band's spectrum); the others
         *  are zero and stay zero, so the row pass skips them. */
        void Run(float* Data, int RowModes = -1)
        {
            for (int Row = 0; Row < N; Row++)
            {
                const int Mode = Row < N / 2 ? Row : N - Row;
                if (RowModes < 0 || Mode <= RowModes) RunLine(Data + size_t(2 * N) * Row, 1);
            }
            for (int Column = 0; Column < N; Column++) RunLine(Data + size_t(2) * Column, N);
        }

    private:
        void RunLine(float* Data, int Stride)
        {
            float* In = Line[0].data();
            for (int i = 0; i < N; i++)
            {
                In[2 * i] = Data[size_t(2) * i * Stride];
                In[2 * i + 1] = Data[size_t(2) * i * Stride + 1];
            }
            const uint32_t Half = uint32_t(N / 2);
            int Current = 0;
            for (int Stage = 0; Stage < Stages; Stage++)
            {
                const float* Src = Line[Current].data();
                float* Dst = Line[1 - Current].data();
                const VoxelShader::FVoxelOceanButterfly* B = &Butterflies[size_t(Stage) * Half];
                for (uint32_t J = 0; J < Half; J++)
                {
                    const float Ar = Src[2 * B[J].SrcA], Ai = Src[2 * B[J].SrcA + 1];
                    const float Br = Src[2 * B[J].SrcB], Bi = Src[2 * B[J].SrcB + 1];
                    const float Wr = Twiddles[B[J].TwiddleIndex].x, Wi = Twiddles[B[J].TwiddleIndex].y;
                    const float Tr = Br * Wr - Bi * Wi, Ti = Br * Wi + Bi * Wr;
                    Dst[2 * B[J].DstA] = Ar + Tr; Dst[2 * B[J].DstA + 1] = Ai + Ti;
                    Dst[2 * B[J].DstB] = Ar - Tr; Dst[2 * B[J].DstB + 1] = Ai - Ti;
                }
                Current = 1 - Current;
            }
            const float* Out = Line[Current].data();
            for (int i = 0; i < N; i++)
            {
                Data[size_t(2) * i * Stride] = Out[2 * i];
                Data[size_t(2) * i * Stride + 1] = Out[2 * i + 1];
            }
        }

        int N;
        int Stages;
        std::vector<VoxelShader::FVoxelOceanButterfly> Butterflies;
        std::vector<VoxelShader::float2> Twiddles;
        std::vector<float> Line[2];
    };

    // -----------------------------------------------------------------------------------------
    // The query's cascades
    // -----------------------------------------------------------------------------------------

    /** A value and its gradient (per m) from a sampled grid. */
    struct FSample
    {
        float Value = 0.0f;
        float Dx = 0.0f;
        float Dy = 0.0f;
    };

    /** Periodic Catmull-Rom on an N x N grid (row-major; element (i, j) at x = i L / N, y = j L / N),
     *  with the interpolant's gradient. U, V in grid units, any value. */
    inline FSample SampleCubic(const float* Grid, int N, double U, double V, double CellSize)
    {
        const double FU = std::floor(U), FV = std::floor(V);
        const float Tu = float(U - FU), Tv = float(V - FV);
        const int Mask = N - 1;
        const int I0 = int(int64_t(FU) & Mask), J0 = int(int64_t(FV) & Mask);
        // Catmull-Rom weights and their derivatives.
        auto Weights = [](float T, float W[4], float D[4])
        {
            const float T2 = T * T, T3 = T2 * T;
            W[0] = 0.5f * (-T3 + 2.0f * T2 - T);
            W[1] = 0.5f * (3.0f * T3 - 5.0f * T2 + 2.0f);
            W[2] = 0.5f * (-3.0f * T3 + 4.0f * T2 + T);
            W[3] = 0.5f * (T3 - T2);
            D[0] = 0.5f * (-3.0f * T2 + 4.0f * T - 1.0f);
            D[1] = 0.5f * (9.0f * T2 - 10.0f * T);
            D[2] = 0.5f * (-9.0f * T2 + 8.0f * T + 1.0f);
            D[3] = 0.5f * (3.0f * T2 - 2.0f * T);
        };
        float Wu[4], Du[4], Wv[4], Dv[4];
        Weights(Tu, Wu, Du);
        Weights(Tv, Wv, Dv);
        float Value = 0.0f, GU = 0.0f, GV = 0.0f;
        for (int b = 0; b < 4; b++)
        {
            const float* Row = Grid + size_t((J0 + b - 1) & Mask) * N;
            float RV = 0.0f, RD = 0.0f;
            for (int a = 0; a < 4; a++)
            {
                const float G = Row[(I0 + a - 1) & Mask];
                RV += Wu[a] * G;
                RD += Du[a] * G;
            }
            Value += Wv[b] * RV;
            GU += Wv[b] * RD;
            GV += Dv[b] * RV;
        }
        FSample S;
        S.Value = Value;
        S.Dx = float(GU / CellSize);
        S.Dy = float(GV / CellSize);
        return S;
    }

    /** The waves of one long-band slot at one time, for the query (m, m/s; chop applied). */
    struct FQuerySample
    {
        float Height = 0.0f;
        float DisplacementX = 0.0f;
        float DisplacementY = 0.0f;
        /** Gradients per m: d(height)/dx, dy; d(Dx)/dx, dy; d(Dy)/dx, dy. */
        float HeightDx = 0.0f, HeightDy = 0.0f;
        float DxDx = 0.0f, DxDy = 0.0f, DyDx = 0.0f, DyDy = 0.0f;
        float VelocityZ = 0.0f;
        float VelocityX = 0.0f;
        float VelocityY = 0.0f;
    };

    /** One slot's six query fields on an N x N grid at one tick. */
    class FQueryCascade
    {
    public:
        enum EField { Height, DispX, DispY, HeightT, DispXT, DispYT, FieldCount };

        FQueryCascade(const FSlot& InSlot, int InN) : Slot(InSlot), N(InN), FFT(InN)
        {
            for (int f = 0; f < FieldCount; f++) Fields[f].assign(size_t(N) * N, 0.0f);
            Packed[0].assign(size_t(2) * N * N, 0.0f);
            Packed[1].assign(size_t(2) * N * N, 0.0f);
            Packed[2].assign(size_t(2) * N * N, 0.0f);
            // The band's modes, their start spectrum and frequency: fixed for the slot.
            using namespace VoxelShader;
            const FVoxelOceanSlotParams& P = Slot.Params;
            const int R = int(std::ceil(std::sqrt(double(P.MMax))));
            for (int Kj = -R; Kj <= R; Kj++)
            {
                for (int Ki = -R; Ki <= R; Ki++)
                {
                    if (!VoxelOceanInBand(P, Ki, Kj)) continue;
                    FMode M;
                    M.Ki = Ki;
                    M.Kj = Kj;
                    M.H0 = VoxelOceanH0(P, Ki, Kj);
                    const float2 Minus = VoxelOceanH0(P, -Ki, -Kj);
                    M.H0MinusConj = float2(Minus.x, -Minus.y);
                    M.Multiple = VoxelOceanModeMultiple(P, Ki, Kj);
                    Modes.push_back(M);
                }
            }
            RowModes = R;
            bValid = 2 * R < N;
        }

        /** False when the band doesn't fit the grid (the grid must hold |K| < N / 2). */
        bool IsValid() const { return bValid; }
        int Size() const { return N; }
        const FSlot& GetSlot() const { return Slot; }
        const std::vector<float>& Field(EField F) const { return Fields[F]; }

        /** The fields at a tick (TickOf). Nothing for a band that doesn't fit (IsValid). */
        void Update(uint32_t Tick)
        {
            using namespace VoxelShader;
            if (!bValid) return;
            for (int p = 0; p < 3; p++) std::fill(Packed[p].begin(), Packed[p].end(), 0.0f);
            const float Dk = VOXEL_OCEAN_TWO_PI / Slot.Params.TileLength;
            for (const FMode& M : Modes)
            {
                const float Phase = VoxelOceanPhase(M.Multiple, Tick);
                const float Omega = float(M.Multiple) * VOXEL_OCEAN_OMEGA0;
                const FVoxelOceanCPUFields F = VoxelOceanCPUFieldsOf(M.H0, M.H0MinusConj, Phase, Omega,
                    float(M.Ki) * Dk, float(M.Kj) * Dk);
                const size_t E = size_t(2) * (size_t(M.Kj & (N - 1)) * N + size_t(M.Ki & (N - 1)));
                Packed[0][E] = F.W0.x; Packed[0][E + 1] = F.W0.y;
                Packed[1][E] = F.W1.x; Packed[1][E + 1] = F.W1.y;
                Packed[2][E] = F.W2.x; Packed[2][E + 1] = F.W2.y;
            }
            for (int p = 0; p < 3; p++) FFT.Run(Packed[p].data(), RowModes);
            const size_t Count = size_t(N) * N;
            for (size_t i = 0; i < Count; i++)
            {
                Fields[Height][i] = Packed[0][2 * i];
                Fields[DispX][i] = Packed[0][2 * i + 1];
                Fields[DispY][i] = Packed[1][2 * i];
                Fields[HeightT][i] = Packed[1][2 * i + 1];
                Fields[DispXT][i] = Packed[2][2 * i];
                Fields[DispYT][i] = Packed[2][2 * i + 1];
            }
        }

        /** The waves at tile-plane position (X, Y) (m, wind along +x), chop applied. */
        FQuerySample Sample(double X, double Y) const
        {
            const double Cell = double(Slot.Params.TileLength) / N;
            const double U = X / Cell, V = Y / Cell;
            const float Chop = Slot.Chop;
            FQuerySample Out;
            const FSample H = SampleCubic(Fields[Height].data(), N, U, V, Cell);
            const FSample Dx = SampleCubic(Fields[DispX].data(), N, U, V, Cell);
            const FSample Dy = SampleCubic(Fields[DispY].data(), N, U, V, Cell);
            Out.Height = H.Value;
            Out.HeightDx = H.Dx;
            Out.HeightDy = H.Dy;
            Out.DisplacementX = Chop * Dx.Value;
            Out.DisplacementY = Chop * Dy.Value;
            Out.DxDx = Chop * Dx.Dx;
            Out.DxDy = Chop * Dx.Dy;
            Out.DyDx = Chop * Dy.Dx;
            Out.DyDy = Chop * Dy.Dy;
            Out.VelocityZ = SampleCubic(Fields[HeightT].data(), N, U, V, Cell).Value;
            Out.VelocityX = Chop * SampleCubic(Fields[DispXT].data(), N, U, V, Cell).Value;
            Out.VelocityY = Chop * SampleCubic(Fields[DispYT].data(), N, U, V, Cell).Value;
            return Out;
        }

    private:
        struct FMode
        {
            int Ki = 0, Kj = 0;
            VoxelShader::float2 H0;
            VoxelShader::float2 H0MinusConj;
            int Multiple = 0;
        };

        FSlot Slot;
        int N;
        int RowModes = -1;
        bool bValid = false;
        FInverseFFT2D FFT;
        std::vector<FMode> Modes;
        std::vector<float> Fields[FieldCount];
        std::vector<float> Packed[3];
    };
}
