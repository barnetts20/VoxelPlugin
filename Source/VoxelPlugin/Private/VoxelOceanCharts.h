// VoxelOceanCharts.h - The waves' charts on the CPU (Design/OceanWaves.md section 3): the chart
// table the GPU reads (built once per planet, in double) and the same charts for the CPU query.
// The cell location, the candidates, the blend weights and the wind turn are the shared code of
// VoxelOceanCharts.ush, which the wave hook runs too. The table also holds the weather's field,
// the charts' headings and the model's constants (stage O7; VoxelOceanSeaState.ush), read on the
// CPU through FVoxelOceanTableReader with the hook's own code.
//
// Plain C++ (std containers, no engine types): included by the plugin (VoxelOceanWavesFFT.cpp)
// and by the harness (Tools/OceanHarness/chart_check.cpp).

#pragma once

#include "VoxelShaderCompat.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace VoxelShader
{
#include "VoxelCubeSphere.ush"
#include "VoxelOceanCharts.ush"

    /** A CPU copy of the table's texels as the shared functions read it (VoxelOceanSeaState.ush:
     *  their table argument, as the shaders' Texture2D). */
    struct FVoxelOceanTableReader
    {
        const float* Texels = nullptr;
        int Width = 0;

        float4 Load(int X, int Y) const
        {
            const float* P = Texels + (size_t(Y) * size_t(Width) + size_t(X)) * 4;
            return float4(P[0], P[1], P[2], P[3]);
        }
    };

#define VOXEL_OCEAN_TABLE_PARAM const FVoxelOceanTableReader& OceanCharts
#define VOXEL_OCEAN_TABLE_ARG OceanCharts
#define VOXEL_OCEAN_TABLE_LOAD(X, Y) OceanCharts.Load(X, Y)
#include "VoxelOceanSeaState.ush"
}

namespace VoxelOceanCharts
{
    /** A chart's nominal width (cm) the level is chosen for. */
    constexpr double TargetWidth = 4.0e6;
    constexpr int MinLevel = 2;
    constexpr int MaxLevel = 8;

    /** The chart level for a sea sphere of radius Radius (cm): cells about TargetWidth across
     *  (a face edge is pi / 2 R). */
    inline int LevelFor(double Radius)
    {
        const double Edge = 1.5707963267948966 * Radius;
        const int Level = int(std::lround(std::log2(std::max(Edge / TargetWidth, 1.0))));
        return std::min(std::max(Level, MinLevel), MaxLevel);
    }

    struct FVec { double X = 0, Y = 0, Z = 0; };
    inline FVec Add(FVec A, FVec B) { return { A.X + B.X, A.Y + B.Y, A.Z + B.Z }; }
    inline FVec Sub(FVec A, FVec B) { return { A.X - B.X, A.Y - B.Y, A.Z - B.Z }; }
    inline FVec Scale(FVec A, double S) { return { A.X * S, A.Y * S, A.Z * S }; }
    inline double Dot(FVec A, FVec B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
    inline FVec Cross(FVec A, FVec B) { return { A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X }; }
    inline FVec Normalize(FVec A) { const double L = std::sqrt(Dot(A, A)); return L > 0 ? Scale(A, 1.0 / L) : A; }

    /** The face axes of VoxelCubeSphere.h, in double. */
    inline FVec FaceAxis(int Face, int Which)  // 0: N, 1: U, 2: V
    {
        using namespace VoxelShader;
        const uint32_t F = uint32_t(Face);
        const uint32_t Axis = Which == 0 ? VoxelCubeNormalAxis(F) : (Which == 1 ? VoxelCubeUAxis(F) : VoxelCubeVAxis(F));
        const double Sign = Which == 0 ? double(VoxelCubeNormalSign(F)) : 1.0;
        FVec V;
        (Axis == 0 ? V.X : (Axis == 1 ? V.Y : V.Z)) = Sign;
        return V;
    }

    /** A value as a multiple of 4096 and the rest (both exact floats for |V| < 2^35 cm). */
    inline float SplitHigh(double V) { return float(std::floor(V / 4096.0) * 4096.0); }
    inline float SplitLow(double V) { return float(V - double(SplitHigh(V))); }

    /** The chart table (VoxelOceanCharts.ush, THE TABLE; VoxelOceanSeaState.ush, its other
     *  regions): RGBA floats. */
    struct FTable
    {
        int Level = 0;
        int N = 0;
        int Width = 0;
        int Height = 0;
        std::vector<float> Texels;      // Width x Height x 4

        bool IsValid() const { return N > 0 && !Texels.empty(); }
        float* Texel(int X, int Y) { return &Texels[(size_t(Y) * size_t(Width) + size_t(X)) * 4]; }
        const float* Texel(int X, int Y) const { return &Texels[(size_t(Y) * size_t(Width) + size_t(X)) * 4]; }
        VoxelShader::FVoxelOceanTableReader Reader() const
        {
            VoxelShader::FVoxelOceanTableReader R;
            R.Texels = Texels.data();
            R.Width = Width;
            return R;
        }
    };

    /** A chart's frame in double: its anchor's direction and its plane's axes. */
    inline void ChartFrame(int Face, int I, int J, int N, FVec& OutDir, FVec& OutE1, FVec& OutE2);

    inline FTable Build(double Radius)
    {
        FTable T;
        T.Level = LevelFor(Radius);
        T.N = 1 << T.Level;
        T.Width = VoxelShader::VoxelOceanTableWidth(T.N);
        T.Height = VoxelShader::VoxelOceanTableHeight(T.N);
        T.Texels.assign(size_t(T.Width) * T.Height * 4, 0.0f);
        for (int Face = 0; Face < 6; Face++)
        {
            for (int J = 0; J < T.N; J++)
            {
                for (int I = 0; I < T.N; I++)
                {
                    FVec Dir, E1, E2;
                    ChartFrame(Face, I, J, T.N, Dir, E1, E2);
                    const FVec A = Scale(Dir, Radius);
                    float* P = T.Texel(VOXEL_OCEAN_CHART_TEXELS * I, Face * T.N + J);
                    P[0] = SplitHigh(A.X); P[1] = SplitHigh(A.Y); P[2] = SplitHigh(A.Z); P[3] = float(E2.X);
                    P[4] = SplitLow(A.X); P[5] = SplitLow(A.Y); P[6] = SplitLow(A.Z); P[7] = float(E2.Y);
                    P[8] = float(E1.X); P[9] = float(E1.Y); P[10] = float(E1.Z); P[11] = float(E2.Z);
                    // The headings until the weather writes them: along E1, both keys alike.
                    P[12] = 1.0f; P[13] = 0.0f; P[14] = 1.0f; P[15] = 0.0f;
                    P[16] = 1.0f; P[17] = 0.0f; P[18] = 1.0f; P[19] = 0.0f;
                }
            }
        }
        return T;
    }

    inline void ChartFrame(int Face, int I, int J, int N, FVec& OutDir, FVec& OutE1, FVec& OutE2)
    {
        const FVec N0 = FaceAxis(Face, 0), U0 = FaceAxis(Face, 1), V0 = FaceAxis(Face, 2);
        const double S = -1.0 + (2.0 * I + 1.0) / N, Tc = -1.0 + (2.0 * J + 1.0) / N;
        OutDir = Normalize(Add(N0, Add(Scale(U0, std::tan(S * 0.78539816339744831)), Scale(V0, std::tan(Tc * 0.78539816339744831)))));
        // E1: the face's U made tangent; E2 = N x E1.
        OutE1 = Normalize(Sub(U0, Scale(OutDir, Dot(OutDir, U0))));
        OutE2 = Cross(OutDir, OutE1);
    }

    /** A chart as the CPU uses it: the shared struct (as the GPU reads it) and its anchor in
     *  double (the exact sum of the two parts). */
    struct FChartCPU
    {
        VoxelShader::FVoxelOceanChart Chart;
        FVec Anchor;
        FVec E1;
        FVec E2;
        /** Its first texel in the table (its headings at X + 3 and X + 4). */
        int X = 0;
        int Y = 0;
    };

    inline FChartCPU ChartAt(const FTable& T, int X, int Y)
    {
        const float* P = T.Texel(X, Y);
        FChartCPU C;
        C.X = X;
        C.Y = Y;
        C.Chart.AnchorHigh = VoxelShader::float3(P[0], P[1], P[2]);
        C.Chart.AnchorLow = VoxelShader::float3(P[4], P[5], P[6]);
        C.Chart.E1 = VoxelShader::float3(P[8], P[9], P[10]);
        C.Chart.E2 = VoxelShader::float3(P[3], P[7], P[11]);
        C.Anchor = { double(P[0]) + double(P[4]), double(P[1]) + double(P[5]), double(P[2]) + double(P[6]) };
        C.E1 = { P[8], P[9], P[10] };
        C.E2 = { P[3], P[7], P[11] };
        return C;
    }

    /** The charts a point (planet frame, cm) blends, as the hook finds them. */
    struct FPointCharts
    {
        int Count = 0;
        FChartCPU Charts[VOXEL_OCEAN_CHART_MAX];
        double Weight[VOXEL_OCEAN_CHART_MAX] = {};
    };

    inline FPointCharts ChartsAt(const FTable& T, const FVec& Point)
    {
        using namespace VoxelShader;
        const FVec D = Normalize(Point);
        const float3 Dir(float(D.X), float(D.Y), float(D.Z));
        const FVoxelOceanChartCandidates Cand = VoxelOceanChartCandidatesOf(Dir, T.N);
        const FVoxelOceanChartBlend B = VoxelOceanChartBlendOf(Cand.Count, VoxelOceanChartDistancesOf(Cand, Dir), T.N);
        FPointCharts Out;
        Out.Count = B.Count;
        for (int m = 0; m < B.Count; m++)
        {
            Out.Charts[m] = ChartAt(T, Cand.Texel[B.Index[m]].x, Cand.Texel[B.Index[m]].y);
            Out.Weight[m] = B.Weight[m];
        }
        return Out;
    }

    /** Chart-plane coordinates (m) of a point (planet frame, cm), in double (the exact sum of
     *  the GPU's base and offset). */
    inline void Coords(const FChartCPU& C, const FVec& Point, double& OutX, double& OutY)
    {
        const FVec Rel = Sub(Point, C.Anchor);
        OutX = Dot(Rel, C.E1) * 0.01;
        OutY = Dot(Rel, C.E2) * 0.01;
    }
}
