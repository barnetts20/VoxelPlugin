// VoxelOceanResolvedShare.h - How much of a slot's slope variance a lookup keeps at each mip
// (stage W6b; Design/DistantOcean.md): the GPU's lookup of a band's derivatives texture is a
// trilinear SampleLevel on its 2 x 2 box mips (VoxelOceanFFT.usf's MipCS), so a mode of the band
// reaches the surface filtered by the mip's box and the bilinear lookup at a point that falls
// anywhere between the mip's texels. Over the band's modes, weighed by their slope (S k_a^2 along
// axis a), the expected variance of the lookups at mip m is the share R_m of the band's; between
// mips (m + t) the lookup blends two filtered fields, its variance
//   (1 - t)^2 R_m + t^2 R_m+1 + 2 t (1 - t) X_m,
// X_m the two mips' covariance share. Per axis, separable: a mode (Ki, Kj) of a grid of N texels
// has phase steps theta = 2 pi Ki / N and 2 pi Kj / N between texels, the box of mip m averages
// 2^m of them, and the bilinear lookup at a uniformly random point
//   E |H_m|^2 = D_m(theta)^2 (2 + cos(2^m theta)) / 3,  D_m = sin(2^m theta / 2) / (2^m sin(theta / 2));
// the covariance E[H_m conj(H_m+1)] is taken numerically over the two mips' common period. The
// 2D lookup's response is the product of the axes' (the lookup's point independent per axis).
// ledger_check holds the table against the emulated textures' own lookups.
#pragma once
#include "VoxelOceanSpectrum.h"
#include <cmath>
#include <complex>
#include <vector>

namespace VoxelOceanSpectrum
{
    /** The mips a band's lookups reach (the shaders' VOXEL_OCEAN_FFT_TOP_MIP and FINE_TOP_MIP: at the top
     *  mip, one texel, the band is skipped). */
    inline int TopMipOfBand(int Band) { int M = 0; while ((1 << M) < SizeOfBand(Band)) M++; return M; }

    constexpr int ResolvedMips = 10;    // mips 0 to 9 (the fine band's top: 512 = 2^9)

    /** Per mip: the share kept along x and y (the band's downwind frame), and the covariance share
     *  with the next mip's lookups, along x and y. */
    struct FResolvedShare
    {
        double R[ResolvedMips][2] = {};
        double X[ResolvedMips][2] = {};
        /** The slope variance along x and y: the spectrum's expectation (StatsOf's, the shading's
         *  table's) and the slot's own draw of it (its modes' Gaussian amplitudes, as the textures
         *  hold them, over time: up to a fifth off for a band of few dominant modes). */
        double Expected[2] = {};
        double Drawn[2] = {};
        /** The clipmap's texels (VoxelOceanClip.usf): the lookups at lod h / 2 (h 0 to 19) taken at
         *  the texels of a grid of that lod's spacing and looked up there bilinearly again by the
         *  pixels (VoxelOceanPixelClip.ush): the share they keep, x and y. */
        double Clip[2 * ResolvedMips][2] = {};
    };

    namespace ResolvedShareDetail
    {
        // One axis at mip M: the lookup's mean response E|H_M|^2 and its covariance with mip M + 1's.
        inline void AxisResponse(double Theta, int M, double& P, std::complex<double>& C)
        {
            auto Box = [](double Th, int Mip) {
                const double W = std::ldexp(1.0, Mip);
                const double S = std::sin(0.5 * Th);
                return std::abs(S) < 1.0e-12 ? 1.0 : std::sin(0.5 * W * Th) / (W * S);
            };
            // The lookup at x (fine-texel units, texel i at i): mip M's texel j at 2^M j + (2^M - 1) / 2.
            auto H = [&](double X, int Mip) {
                const double W = std::ldexp(1.0, Mip);
                const double U = (X - 0.5 * (W - 1.0)) / W;
                const double J = std::floor(U), A = U - J;
                const double C0 = W * J + 0.5 * (W - 1.0), C1 = C0 + W;
                const std::complex<double> V = (1.0 - A) * std::polar(1.0, Theta * (C0 - X)) + A * std::polar(1.0, Theta * (C1 - X));
                return Box(Theta, Mip) * V;
            };
            const double D = Box(Theta, M);
            P = D * D * (2.0 + std::cos(std::ldexp(Theta, M))) / 3.0;
            // Over the common period (2^(M + 1) fine texels), finely enough for the bilinear kinks.
            const int Steps = 64;
            const double Period = std::ldexp(1.0, M + 1);
            std::complex<double> Sum(0.0, 0.0);
            for (int s = 0; s < Steps; s++)
            {
                const double X = (s + 0.5) / Steps * Period;
                Sum += H(X, M) * std::conj(H(X, M + 1));
            }
            C = Sum / double(Steps);
        }
    }

    /** The slot's table: from its modes (as StatsOf weighs them) and its band's grid. */
    inline FResolvedShare ResolvedShareOf(const FSlot& Slot)
    {
        using namespace VoxelShader;
        FResolvedShare Out;
        const FVoxelOceanSlotParams& P = Slot.Params;
        const int N = SizeOfBand(Slot.Band);
        const int Top = TopMipOfBand(Slot.Band);
        // Per axis and mode index: the responses at each mip.
        std::vector<double> PA(size_t(N) * ResolvedMips, 0.0);
        std::vector<std::complex<double>> CA(size_t(N) * ResolvedMips, 0.0);
        // (And the clip grid's bilinear factor per axis at the half lods.)
        std::vector<double> GA(size_t(N) * 2 * ResolvedMips, 0.0);
        for (int i = 0; i < N; i++)
        {
            const int K = i < N / 2 ? i : i - N;
            const double Theta = 2.0 * Pi * K / N;
            for (int h = 0; h < 2 * ResolvedMips; h++)
                GA[size_t(i) * 2 * ResolvedMips + h] = (2.0 + std::cos(std::exp2(0.5 * h) * Theta)) / 3.0;
            for (int m = 0; m < ResolvedMips; m++)
            {
                double Pm = 0.0;
                std::complex<double> Cm(0.0, 0.0);
                if (m < Top) ResolvedShareDetail::AxisResponse(Theta, m, Pm, Cm);
                // (At the top mip and past it the band is skipped: nothing; the last mip under
                // it blends toward nothing.)
                if (m + 1 >= Top) Cm = 0.0;
                PA[size_t(i) * ResolvedMips + m] = Pm;
                CA[size_t(i) * ResolvedMips + m] = Cm;
            }
        }
        const FVoxelOceanJonswap J = VoxelOceanJonswapOf(P.WindSpeed, P.Fetch);
        const double Dk = 2.0 * Pi / P.TileLength;
        const int R = int(std::ceil(std::sqrt(double(P.MMax))));
        double Total[2] = { 0.0, 0.0 };
        for (int Kj = -R; Kj <= R; Kj++)
            for (int Ki = -R; Ki <= R; Ki++)
            {
                if (!VoxelOceanInBand(P, Ki, Kj)) continue;
                const double Kx = Ki * Dk, Ky = Kj * Dk;
                const double S = double(VoxelOceanSpectrumK(P, J, float(Kx), float(Ky))) * Dk * Dk;
                const double W[2] = { Kx * Kx * S, Ky * Ky * S };
                const int Ix = Ki < 0 ? Ki + N : Ki, Iy = Kj < 0 ? Kj + N : Kj;
                for (int m = 0; m < ResolvedMips; m++)
                {
                    const double Pm = PA[size_t(Ix) * ResolvedMips + m] * PA[size_t(Iy) * ResolvedMips + m];
                    const double Xm = (CA[size_t(Ix) * ResolvedMips + m] * CA[size_t(Iy) * ResolvedMips + m]).real();
                    for (int a = 0; a < 2; a++) { Out.R[m][a] += W[a] * Pm; Out.X[m][a] += W[a] * Xm; }
                }
                // The clipmap's: the lookup at lod l (trilinear), then the clip grid's bilinear lookup
                // (spacing 2^l texels: per axis (2 + cos(2^l theta)) / 3 over the pixels' points).
                for (int h = 0; h < 2 * ResolvedMips; h++)
                {
                    const int M = h / 2;
                    const double T = 0.5 * (h & 1);
                    auto Pof = [&](int Mip) { return Mip < ResolvedMips ? PA[size_t(Ix) * ResolvedMips + Mip] * PA[size_t(Iy) * ResolvedMips + Mip] : 0.0; };
                    const double Xm = (CA[size_t(Ix) * ResolvedMips + M] * CA[size_t(Iy) * ResolvedMips + M]).real();
                    const double E = (1.0 - T) * (1.0 - T) * Pof(M) + T * T * Pof(M + 1) + 2.0 * T * (1.0 - T) * Xm;
                    const double C = GA[size_t(Ix) * 2 * ResolvedMips + h] * GA[size_t(Iy) * 2 * ResolvedMips + h];
                    for (int a = 0; a < 2; a++) Out.Clip[h][a] += W[a] * E * C;
                }
                Total[0] += W[0];
                Total[1] += W[1];
                // The draw: the mode's term and its conjugate's, |h0(k)|^2 + |h0(-k)|^2 (VoxelOceanH0
                // without the scale, as the weights here).
                const float2 H = VoxelOceanH0(P, Ki, Kj), Hm = VoxelOceanH0(P, -Ki, -Kj);
                const double D = (double(H.x) * H.x + double(H.y) * H.y + double(Hm.x) * Hm.x + double(Hm.y) * Hm.y)
                    / std::max(double(P.Scale) * double(P.Scale), 1.0e-30);
                Out.Drawn[0] += Kx * Kx * D;
                Out.Drawn[1] += Ky * Ky * D;
            }
        Out.Expected[0] = Total[0];
        Out.Expected[1] = Total[1];
        for (int m = 0; m < ResolvedMips; m++)
            for (int a = 0; a < 2; a++)
            {
                Out.R[m][a] = Total[a] > 0.0 ? Out.R[m][a] / Total[a] : 0.0;
                Out.X[m][a] = Total[a] > 0.0 ? Out.X[m][a] / Total[a] : 0.0;
            }
        for (int h = 0; h < 2 * ResolvedMips; h++)
            for (int a = 0; a < 2; a++)
                Out.Clip[h][a] = Total[a] > 0.0 ? Out.Clip[h][a] / Total[a] : 0.0;
        return Out;
    }

    /** The table's rows as the shaders take them (VoxelOceanSeaState.ush, VoxelOceanSlotUnresolved):
     *  per mip m (K_m, X_m, K_m+1, rho), both axes together over the expectation's variance: K the
     *  draw's variance kept at mip m, X its covariance with mip m + 1's, rho the draw's whole (each
     *  over the expectation's, x and y summed). The unresolved share at m + t is then
     *  rho - ((1 - t)^2 K_m + t^2 K_m+1 + 2 t (1 - t) X_m). */
    inline void PackResolvedRows(const FResolvedShare& T, float* Out)
    {
        const double E = std::max(T.Expected[0] + T.Expected[1], 1.0e-30);
        const double Wx = T.Drawn[0] / E, Wy = T.Drawn[1] / E;
        auto K = [&](int M) { return M < ResolvedMips ? Wx * T.R[M][0] + Wy * T.R[M][1] : 0.0; };
        for (int M = 0; M < ResolvedMips; M++)
        {
            Out[4 * M + 0] = float(K(M));
            Out[4 * M + 1] = float(Wx * T.X[M][0] + Wy * T.X[M][1]);
            Out[4 * M + 2] = float(K(M + 1));
            Out[4 * M + 3] = float(Wx + Wy);
        }
    }

    /** The clipmap's rows (VoxelOceanSlotUnresolvedClip): per mip m (C_m, C_m+1/2, C_m+1, rho), the
     *  clipmap's share kept at lods m, m + 1/2 and m + 1 (over the expectation's variance, as
     *  PackResolvedRows), interpolated quadratically between. */
    inline void PackClipRows(const FResolvedShare& T, float* Out)
    {
        const double E = std::max(T.Expected[0] + T.Expected[1], 1.0e-30);
        const double Wx = T.Drawn[0] / E, Wy = T.Drawn[1] / E;
        auto C = [&](int H) { return H < 2 * ResolvedMips ? Wx * T.Clip[H][0] + Wy * T.Clip[H][1] : 0.0; };
        for (int M = 0; M < ResolvedMips; M++)
        {
            Out[4 * M + 0] = float(C(2 * M));
            Out[4 * M + 1] = float(C(2 * M + 1));
            Out[4 * M + 2] = float(C(2 * M + 2));
            Out[4 * M + 3] = float(Wx + Wy);
        }
    }

    /** The clipmap's share kept at a texel lod (quadratic through the half steps), along axis a. */
    inline double ClipShareAt(const FResolvedShare& T, double Lod, int A)
    {
        Lod = std::clamp(Lod, 0.0, double(ResolvedMips - 1));
        const int M = std::min(int(std::floor(Lod)), ResolvedMips - 1);
        const double F = Lod - M;
        const double C0 = T.Clip[2 * M][A], CH = T.Clip[2 * M + 1][A], C1 = 2 * M + 2 < 2 * ResolvedMips ? T.Clip[2 * M + 2][A] : 0.0;
        return C0 * (1.0 - F) * (1.0 - 2.0 * F) + 4.0 * CH * F * (1.0 - F) + C1 * F * (2.0 * F - 1.0);
    }

    /** The share kept at a lookup's lod (the shaders' LodN) along axis a, as the GPU's trilinear lookup. */
    inline double ResolvedShareAt(const FResolvedShare& T, double Lod, int A)
    {
        if (Lod <= 0.0) return T.R[0][A];
        const int M = int(std::floor(Lod));
        if (M >= ResolvedMips - 1) return 0.0;
        const double F = Lod - M;
        return (1.0 - F) * (1.0 - F) * T.R[M][A] + F * F * T.R[M + 1][A] + 2.0 * F * (1.0 - F) * T.X[M][A];
    }
}
