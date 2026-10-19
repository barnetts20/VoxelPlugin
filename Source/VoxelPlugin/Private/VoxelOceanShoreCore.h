// VoxelOceanShoreCore.h - The shore field on the CPU (Design/OceanWaves.md section 5; stage O8):
// the shared code of VoxelOceanShore.ush (the texels, the sampling) and VoxelOceanShoreBuild.ush
// (the build, cell by cell), with the readers they take on this side, and the whole build for
// a machine without a GPU (a dedicated server). The GPU builds the same field with the same
// functions (VoxelOceanShoreBuild.usf; FVoxelOceanShoreField reads it back).
//
// Plain C++ (std containers, no engine types): included by the plugin (VoxelOceanShoreField.cpp,
// VoxelOceanWavesFFT.cpp) and by the harness (Tools/OceanHarness/shore_check.cpp).

#pragma once

#include "VoxelOceanCharts.h"

#include <cstdint>
#include <utility>
#include <vector>

namespace VoxelShader
{
    /** The field's texels as the sampling reads them (VoxelOceanShore.ush: its texture argument):
     *  6 faces of Side^2 texels' bits. */
    struct FVoxelOceanShoreReader
    {
        const uint32_t* Texels = nullptr;
        int Side = 0;

        uint Load(int Face, int X, int Y) const
        {
            return Texels[(size_t(Face) * size_t(Side) + size_t(Y)) * size_t(Side) + size_t(X)];
        }
    };

#define VOXEL_OCEAN_SHORE_PARAM const FVoxelOceanShoreReader& OceanShore
#define VOXEL_OCEAN_SHORE_ARG OceanShore
#define VOXEL_OCEAN_SHORE_LOAD(Face, X, Y) OceanShore.Load(Face, X, Y)
#include "VoxelOceanShore.ush"

    /** The build's inputs (VoxelOceanShoreBuild.ush): the bake's heights and attributes (the
     *  sea-connected bit, FVoxelLandBake::SeaBit), and the seeds of the pass before. */
    struct FVoxelOceanShoreBuildInputs
    {
        const float* Height = nullptr;
        const uint16_t* Attr = nullptr;
        const uint32_t* Seeds = nullptr;
        const uint32_t* Texels = nullptr;
    };

#define VOXEL_OCEAN_SHORE_BUILD_PARAM const FVoxelOceanShoreBuildInputs& ShoreBuild
#define VOXEL_OCEAN_SHORE_BUILD_ARG ShoreBuild
#define VOXEL_OCEAN_SHORE_BUILD_HEIGHT(Index) ShoreBuild.Height[Index]
#define VOXEL_OCEAN_SHORE_BUILD_SEA(Index) ((ShoreBuild.Attr[Index] & 0x800u) != 0u)
#define VOXEL_OCEAN_SHORE_BUILD_SEED(Index) ShoreBuild.Seeds[Index]
#define VOXEL_OCEAN_SHORE_BUILD_TEXEL(Index) ShoreBuild.Texels[Index]
#include "VoxelOceanShoreBuild.ush"
}

namespace VoxelOceanShoreCore
{
    constexpr int Pad = VOXEL_OCEAN_SHORE_PAD;

    inline int Side(int N) { return N + 2 * Pad; }
    inline int NumTexels(int N) { return 6 * Side(N) * Side(N); }

    /** The smallest cell's arc (m): the equal-angle cells vary 1.41x across a face, the largest at
     *  its centre (pi / 2 R / N). */
    inline double CellMinM(int N, double RadiusM) { return 1.5707963267948966 * RadiusM / double(N > 0 ? N : 1) / 1.4142135623730951; }

    /** The jumps' first step (cells) for a reach (m). */
    inline int FirstStep(int N, double RadiusM, double ReachM)
    {
        return VoxelShader::VoxelOceanShoreFirstStep(float(ReachM), float(CellMinM(N, RadiusM)));
    }

    /** The steps the jumps take: K, K / 2, ... 1, and 1 again. */
    inline std::vector<int> Steps(int N, double RadiusM, double ReachM)
    {
        std::vector<int> Out;
        for (int K = FirstStep(N, RadiusM, ReachM); K >= 1; K /= 2) Out.push_back(K);
        Out.push_back(1);
        return Out;
    }

    /** The smoothing passes after the pack (VoxelOceanShoreSmoothPass). */
    constexpr int SmoothPasses = 2;

    /** The whole build on the CPU: the texels (6 Side^2, VoxelOceanShore.ush) from the bake's
     *  heights and attributes (6 Side^2 each). ForEach(Count, Fn) runs Fn(i) for every i in
     *  [0, Count), in parallel or not. */
    template <typename FForEach>
    std::vector<uint32_t> Build(int N, double RadiusM, double ReachM, const float* Height, const uint16_t* Attr, FForEach&& ForEach)
    {
        const int Count = NumTexels(N);
        std::vector<uint32_t> A(size_t(Count), VOXEL_OCEAN_SHORE_NONE);
        std::vector<uint32_t> B(size_t(Count), VOXEL_OCEAN_SHORE_NONE);
        VoxelShader::FVoxelOceanShoreBuildInputs In;
        In.Height = Height;
        In.Attr = Attr;
        ForEach(Count, [&](int i) { A[size_t(i)] = VoxelShader::VoxelOceanShoreSeedPass(In, i, N); });
        for (const int K : Steps(N, RadiusM, ReachM))
        {
            In.Seeds = A.data();
            ForEach(Count, [&](int i) { B[size_t(i)] = VoxelShader::VoxelOceanShoreJumpPass(In, i, N, K); });
            std::swap(A, B);
        }
        In.Seeds = A.data();
        ForEach(Count, [&](int i) { B[size_t(i)] = VoxelShader::VoxelOceanShorePackPass(In, i, N, float(RadiusM), float(ReachM)); });
        for (int Pass = 0; Pass < SmoothPasses; Pass++)
        {
            In.Texels = B.data();
            ForEach(Count, [&](int i) { A[size_t(i)] = VoxelShader::VoxelOceanShoreSmoothPass(In, i, N, float(ReachM)); });
            std::swap(A, B);
        }
        return B;
    }

    /** The texel-space expansion around a reference point (VoxelOceanShore.ush, NEAR THE CAMERA:
     *  FVoxelOceanShoreFrame's 7 texels, 28 floats): in double, for the face holding the reference
     *  (planet frame, cm). The gradient analytic, the second derivatives by central differences of
     *  it 10 m apart. */
    inline void FrameOf(double RefX, double RefY, double RefZ, int N, float Out[28])
    {
        for (int i = 0; i < 28; i++) Out[i] = 0.0f;
        const double L = std::sqrt(RefX * RefX + RefY * RefY + RefZ * RefZ);
        if (N <= 0 || !(L > 0.0)) return;
        const double P[3] = { RefX, RefY, RefZ };
        int Axis = 0;
        if (std::fabs(P[1]) > std::fabs(P[Axis])) Axis = 1;
        if (std::fabs(P[2]) > std::fabs(P[Axis])) Axis = 2;
        const unsigned Face = unsigned(Axis * 2 + (P[Axis] < 0.0 ? 1 : 0));
        const int UA = int(VoxelShader::VoxelCubeUAxis(Face)), VA = int(VoxelShader::VoxelCubeVAxis(Face));
        const double NS = (Face & 1u) != 0u ? -1.0 : 1.0;
        const double Scale = 0.5 * double(N) * 4.0 / 3.141592653589793;
        // The texel coordinate along axis A (UA or VA) of a point and its gradient.
        auto Coord = [&](const double* Q, int A) { return (std::atan(Q[A] / (NS * Q[Axis])) * 4.0 / 3.141592653589793 + 1.0) * 0.5 * double(N) - 0.5 + double(Pad); };
        auto Grad = [&](const double* Q, int A, double* G)
            {
                const double Nn = NS * Q[Axis], R = Q[A] / Nn, K = Scale / (1.0 + R * R) / (Nn * Nn);
                for (int c = 0; c < 3; c++) G[c] = 0.0;
                G[A] += K * Nn;
                G[Axis] -= K * Q[A] * NS;
            };
        const double GX0 = Coord(P, UA), GY0 = Coord(P, VA);
        double GX[3], GY[3];
        Grad(P, UA, GX);
        Grad(P, VA, GY);
        // Second derivatives: the gradients 10 m either side along each axis.
        double HX[3][3], HY[3][3];
        const double H = 1000.0;
        for (int j = 0; j < 3; j++)
        {
            double Plus[3] = { P[0], P[1], P[2] }, Minus[3] = { P[0], P[1], P[2] };
            Plus[j] += H;
            Minus[j] -= H;
            double GP[3], GM[3];
            Grad(Plus, UA, GP); Grad(Minus, UA, GM);
            for (int i = 0; i < 3; i++) HX[i][j] = (GP[i] - GM[i]) / (2.0 * H);
            Grad(Plus, VA, GP); Grad(Minus, VA, GM);
            for (int i = 0; i < 3; i++) HY[i][j] = (GP[i] - GM[i]) / (2.0 * H);
        }
        auto Sym = [](double M[3][3], int i, int j) { return float(0.5 * (M[i][j] + M[j][i])); };
        const double BX = std::floor(GX0), BY = std::floor(GY0);
        float* T = Out;
        T[0] = 1.0f; T[1] = float(Face); T[2] = float(BX); T[3] = float(BY);
        T[4] = float(GX0 - BX); T[5] = float(GY0 - BY); T[6] = 0.0f; T[7] = 0.0f;
        T[8] = float(GX[0]); T[9] = float(GX[1]); T[10] = float(GX[2]); T[11] = 0.0f;
        T[12] = float(GY[0]); T[13] = float(GY[1]); T[14] = float(GY[2]); T[15] = 0.0f;
        T[16] = Sym(HX, 0, 0); T[17] = Sym(HX, 1, 1); T[18] = Sym(HX, 2, 2); T[19] = Sym(HX, 0, 1);
        T[20] = Sym(HX, 0, 2); T[21] = Sym(HX, 1, 2); T[22] = Sym(HY, 0, 0); T[23] = Sym(HY, 1, 1);
        T[24] = Sym(HY, 2, 2); T[25] = Sym(HY, 0, 1); T[26] = Sym(HY, 0, 2); T[27] = Sym(HY, 1, 2);
    }

    /** The frame as the shared code takes it. */
    inline VoxelShader::FVoxelOceanShoreFrame FrameFrom(const float T[28])
    {
        using VoxelShader::float4;
        return VoxelShader::VoxelOceanShoreFrameFrom(float4(T[0], T[1], T[2], T[3]), float4(T[4], T[5], T[6], T[7]), float4(T[8], T[9], T[10], T[11]),
            float4(T[12], T[13], T[14], T[15]), float4(T[16], T[17], T[18], T[19]), float4(T[20], T[21], T[22], T[23]), float4(T[24], T[25], T[26], T[27]));
    }

    /** The field at a direction (planet frame, any length). */
    inline VoxelShader::FVoxelOceanShoreSample Sample(const uint32_t* Texels, int N, double ReachM, double X, double Y, double Z)
    {
        const double L = std::sqrt(X * X + Y * Y + Z * Z);
        if (!Texels || N <= 0 || !(L > 0.0)) return VoxelShader::VoxelOceanShoreSampleNone(float(ReachM));
        VoxelShader::FVoxelOceanShoreReader Reader;
        Reader.Texels = Texels;
        Reader.Side = Side(N);
        return VoxelShader::VoxelOceanShoreFieldAt(Reader, VoxelShader::float3(float(X / L), float(Y / L), float(Z / L)), N, float(ReachM));
    }
}
