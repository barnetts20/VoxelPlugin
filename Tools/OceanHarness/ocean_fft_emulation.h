// ocean_fft_emulation.h - VoxelOceanFFT.usf's passes run in float on the CPU, with the shader's
// own loops and the shared code (init, time step, FFT rows and columns, assemble), plus the
// GPU's periodic bilinear lookup. Used by spectrum_check.cpp and spectrum_view.cpp. Stage W3z: each
// slot on its band's grid (SizeOfBand: 512^2 for the shortest band), and the long arrays' copy of it
// (its mip 1, LongCopy).
#pragma once
#include "VoxelOceanSpectrum.h"
#include <cmath>
#include <vector>

namespace OceanFFTEmulation
{
using namespace VoxelOceanSpectrum;
using VoxelShader::float2;

// ---------------------------------------------------------------------------------------------
// The GPU passes in float, as VoxelOceanFFT.usf runs them
// ---------------------------------------------------------------------------------------------

struct F4 { float x, y, z, w; };

inline int SignedMode(int Index, int N) { return Index < N / 2 ? Index : Index - N; }

inline F4 ButterflyApply(F4 A, F4 B, float2 W, float Sign)
{
    const float2 B0 = VoxelShader::VoxelOceanCMul(float2(B.x, B.y), W);
    const float2 B1 = VoxelShader::VoxelOceanCMul(float2(B.z, B.w), W);
    return { A.x + Sign * B0.x, A.y + Sign * B0.y, A.z + Sign * B1.x, A.w + Sign * B1.y };
}

// FFTCS on one line (Data[i * Stride]) of N.
inline void FFTLine(F4* Data, int Stride, int N)
{
    // (Up to 1024: the studies' own grids, e.g. GodotOceanWaves' 1024^2.)
    static F4 Shared[2][1024];
    static float2 Twiddles[512];
    const uint32_t Half = uint32_t(N / 2);
    uint32_t Stages = 0;
    while ((1 << Stages) < N) Stages++;
    for (uint32_t T = 0; T < Half; T++)
    {
        Shared[0][T] = Data[T * Stride];
        Shared[0][T + Half] = Data[(T + Half) * Stride];
        Twiddles[T] = VoxelShader::VoxelOceanTwiddle(T, Half);
    }
    uint32_t Ns = 1;
    for (uint32_t Stage = 0; Stage < Stages; Stage++)
    {
        const uint32_t In = Stage & 1u;
        for (uint32_t T = 0; T < Half; T++)
        {
            const VoxelShader::FVoxelOceanButterfly B = VoxelShader::VoxelOceanButterflyOf(Ns, T, Half);
            const float2 W = Twiddles[B.TwiddleIndex];
            const F4 A = Shared[In][B.SrcA], C = Shared[In][B.SrcB];
            Shared[1 - In][B.DstA] = ButterflyApply(A, C, W, 1.0f);
            Shared[1 - In][B.DstB] = ButterflyApply(A, C, W, -1.0f);
        }
        Ns *= 2;
    }
    // (The result in the buffer the last stage wrote, as FFTCS.)
    const uint32_t Out = Stages & 1u;
    for (uint32_t T = 0; T < Half; T++)
    {
        Data[T * Stride] = Shared[Out][T];
        Data[(T + Half) * Stride] = Shared[Out][T + Half];
    }
}

struct FGPUSlot
{
    std::vector<F4> Displacement;   // chop Dx, chop Dy, eta, chop Dxy
    std::vector<F4> Derivatives;    // eta_x, eta_y, chop Dxx, chop Dyy
    int N = GPUSize;                // its grid
};

// GridOverride: another grid than the band's (a study's; a power of two up to 1024).
inline FGPUSlot EmulateGPU(const FSlot& Slot, uint32_t Tick, int GridOverride = 0)
{
    using namespace VoxelShader;
    const int N = GridOverride > 0 ? GridOverride : SizeOfBand(Slot.Band);
    const FVoxelOceanSlotParams& P = Slot.Params;
    std::vector<F4> Spectrum(size_t(N) * N), Fields(size_t(2) * N * N);
    std::vector<uint32_t> Multiples(size_t(N) * N);
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++)
        {
            const int Ki = SignedMode(x, N), Kj = SignedMode(y, N);
            const float2 H0 = VoxelOceanH0(P, Ki, Kj), Hm = VoxelOceanH0(P, -Ki, -Kj);
            Spectrum[size_t(y) * N + x] = { H0.x, H0.y, Hm.x, -Hm.y };
            Multiples[size_t(y) * N + x] = VoxelOceanInBand(P, Ki, Kj) ? uint32_t(VoxelOceanModeMultiple(P, Ki, Kj)) : 0u;
        }
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++)
        {
            const int Ki = SignedMode(x, N), Kj = SignedMode(y, N);
            F4 O0 = { 0, 0, 0, 0 }, O1 = { 0, 0, 0, 0 };
            if (VoxelOceanInBand(P, Ki, Kj))
            {
                const F4 H = Spectrum[size_t(y) * N + x];
                const float Phase = VoxelOceanPhase(int(Multiples[size_t(y) * N + x]), Tick);
                const float2 Ht = VoxelOceanEvolve(float2(H.x, H.y), float2(H.z, H.w), Phase);
                const float Dk = VOXEL_OCEAN_TWO_PI / P.TileLength;
                const FVoxelOceanGPUFields F = VoxelOceanGPUFieldsOf(Ht, float(Ki) * Dk, float(Kj) * Dk);
                O0 = { F.Z0.x, F.Z0.y, F.Z1.x, F.Z1.y };
                O1 = { F.Z2.x, F.Z2.y, F.Z3.x, F.Z3.y };
            }
            Fields[size_t(y) * N + x] = O0;
            Fields[size_t(N) * N + size_t(y) * N + x] = O1;
        }
    for (int Slice = 0; Slice < 2; Slice++)
    {
        F4* Base = Fields.data() + size_t(Slice) * N * N;
        // The row pass skips the rows the band leaves empty, as FFTCS does.
        for (int Row = 0; Row < N; Row++)
        {
            const int Kj = SignedMode(Row, N);
            if (uint32_t(Kj * Kj) < P.MMax) FFTLine(Base + size_t(Row) * N, 1, N);
        }
        for (int Col = 0; Col < N; Col++) FFTLine(Base + Col, N, N);
    }
    FGPUSlot Out;
    Out.N = N;
    Out.Displacement.resize(size_t(N) * N);
    Out.Derivatives.resize(size_t(N) * N);
    const float Chop = Slot.Chop;
    for (size_t i = 0; i < size_t(N) * N; i++)
    {
        const F4 F0 = Fields[i], F1 = Fields[size_t(N) * N + i];
        Out.Displacement[i] = { Chop * F0.y, Chop * F0.z, F0.x, Chop * F1.w };
        Out.Derivatives[i] = { F0.w, F1.x, Chop * F1.y, Chop * F1.z };
    }
    return Out;
}

// The long arrays' copy of a slot (stage W3z): the shortest band's mip 1 (2 x 2 means; its texel i
// at the fine grid's 2 i + 1/2), the others as they are.
inline FGPUSlot LongCopy(const FGPUSlot& G)
{
    if (G.N == GPUSize) return G;
    const int N = G.N / 2;
    FGPUSlot Out;
    Out.N = N;
    Out.Displacement.resize(size_t(N) * N);
    Out.Derivatives.resize(size_t(N) * N);
    auto Mean = [&](const std::vector<F4>& T, int i, int j)
    {
        const F4 A = T[size_t(2 * j) * G.N + 2 * i], B = T[size_t(2 * j) * G.N + 2 * i + 1];
        const F4 C = T[size_t(2 * j + 1) * G.N + 2 * i], D = T[size_t(2 * j + 1) * G.N + 2 * i + 1];
        return F4{ 0.25f * (A.x + B.x + C.x + D.x), 0.25f * (A.y + B.y + C.y + D.y), 0.25f * (A.z + B.z + C.z + D.z), 0.25f * (A.w + B.w + C.w + D.w) };
    };
    for (int j = 0; j < N; j++)
        for (int i = 0; i < N; i++)
        {
            Out.Displacement[size_t(j) * N + i] = Mean(G.Displacement, i, j);
            Out.Derivatives[size_t(j) * N + i] = Mean(G.Derivatives, i, j);
        }
    return Out;
}

// Bilinear, periodic (the GPU's wrap sampler at mip 0), on the texture's own grid (its side the
// square root of its size), U and V in its texels.
inline F4 Bilinear(const std::vector<F4>& T, double U, double V)
{
    const int N = int(std::lround(std::sqrt(double(T.size()))));
    const double FU = std::floor(U), FV = std::floor(V);
    const float a = float(U - FU), b = float(V - FV);
    const int i0 = int(int64_t(FU) & (N - 1)), j0 = int(int64_t(FV) & (N - 1));
    const int i1 = (i0 + 1) & (N - 1), j1 = (j0 + 1) & (N - 1);
    auto At = [&](int i, int j) { return T[size_t(j) * N + i]; };
    const F4 p = At(i0, j0), q = At(i1, j0), r = At(i0, j1), s = At(i1, j1);
    auto L = [&](float P, float Q, float R, float S) { return (1 - b) * ((1 - a) * P + a * Q) + b * ((1 - a) * R + a * S); };
    return { L(p.x, q.x, r.x, s.x), L(p.y, q.y, r.y, s.y), L(p.z, q.z, r.z, s.z), L(p.w, q.w, r.w, s.w) };
}

}
