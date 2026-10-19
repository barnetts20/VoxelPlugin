// O6b: the waves' charts (VoxelOceanCharts.ush, VoxelOceanCharts.h), checked without the engine.
// - The level per planet size; the table's frames (orthonormal, tangent, anchors on the sphere,
//   the high and low parts exact).
// - Coverage: every point of the sphere has a chart; sum w^2 = 1; how many charts points blend.
// - Continuity: each chart's weight changes smoothly between points 1 m apart, everywhere and
//   along face edges and at cube corners in particular.
// - Precision: the GPU's chart coordinates (float, from the reference's high and low parts and a
//   point's offset from it) against the CPU's (double), for points up to 50 km from the reference.
// - The wind turn: the downwind direction against a reference in double.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private chart_check.cpp -o chart_check
#include "VoxelOceanCharts.h"

#include <cstdio>
#include <map>
#include <random>

using namespace VoxelOceanCharts;
using VoxelShader::float3;

static int Failures = 0;
static void Expect(bool Ok, const char* What)
{
    std::printf("  [%s] %s\n", Ok ? "ok" : "FAIL", What);
    if (!Ok) Failures++;
}

static FVec RandomDir(std::mt19937_64& Rng)
{
    std::normal_distribution<double> G;
    return Normalize({ G(Rng), G(Rng), G(Rng) });
}

static std::map<long, double> WeightMap(const FTable& T, const FVec& P)
{
    const FPointCharts C = ChartsAt(T, P);
    std::map<long, double> M;
    for (int m = 0; m < C.Count; m++)
    {
        const long Key = long(std::lround(C.Charts[m].Anchor.X)) * 1000003L + long(std::lround(C.Charts[m].Anchor.Y)) * 7919L + long(std::lround(C.Charts[m].Anchor.Z));
        M[Key] += C.Weight[m];
    }
    return M;
}

static double WeightJump(const FTable& T, const FVec& A, const FVec& B)
{
    const std::map<long, double> WA = WeightMap(T, A), WB = WeightMap(T, B);
    double Worst = 0;
    for (const auto& [K, W] : WA) { const auto It = WB.find(K); Worst = std::max(Worst, std::fabs(W - (It == WB.end() ? 0.0 : It->second))); }
    for (const auto& [K, W] : WB) { const auto It = WA.find(K); Worst = std::max(Worst, std::fabs(W - (It == WA.end() ? 0.0 : It->second))); }
    return Worst;
}

int main()
{
    const double R = 1.075e8;
    std::printf("Levels: 1,075 km -> %d, 3,000 km -> %d, 6,371 km -> %d, 300 km -> %d\n", LevelFor(R), LevelFor(3.0e8), LevelFor(6.371e8), LevelFor(3.0e7));
    const FTable T = Build(R);
    std::printf("Table: level %d, %d x %d texels (%.0f KB), chart about %.1f km\n", T.Level, T.Width, T.Height, T.Texels.size() * 4 / 1024.0, 1.5707963 * R / T.N / 1e5);

    // ---- Frames ------------------------------------------------------------------------------
    std::printf("\nThe table:\n");
    {
        double WorstOrtho = 0, WorstRadius = 0, WorstSplit = 0;
        for (int Y = 0; Y < 6 * T.N; Y++)
            for (int I = 0; I < T.N; I++)
            {
                const FChartCPU C = ChartAt(T, VOXEL_OCEAN_CHART_TEXELS * I, Y);
                const FVec N = Normalize(C.Anchor);
                WorstOrtho = std::max({ WorstOrtho, std::fabs(Dot(C.E1, C.E1) - 1), std::fabs(Dot(C.E2, C.E2) - 1), std::fabs(Dot(C.E1, C.E2)), std::fabs(Dot(C.E1, N)), std::fabs(Dot(C.E2, N)) });
                WorstRadius = std::max(WorstRadius, std::fabs(std::sqrt(Dot(C.Anchor, C.Anchor)) - R));
                for (int a = 0; a < 3; a++)
                {
                    const double H = (&C.Chart.AnchorHigh.x)[a], L = (&C.Chart.AnchorLow.x)[a];
                    WorstSplit = std::max(WorstSplit, std::fabs(std::fmod(H, 4096.0)) + (L < 0 || L >= 4096 ? 1.0 : 0.0));
                }
            }
        std::printf("  frames: worst |orthonormality or tangency error| %.2e; anchors off the sphere by up to %.3f cm; split defects %.0f\n", WorstOrtho, WorstRadius, WorstSplit);
        Expect(WorstOrtho < 1e-6 && WorstRadius < 0.01 && WorstSplit == 0, "frames orthonormal and tangent, anchors on the sphere, exact splits");
    }

    // ---- Coverage ----------------------------------------------------------------------------
    std::printf("\nCoverage:\n");
    {
        std::mt19937_64 Rng(1);
        int Hist[5] = {};
        double WorstNorm = 0;
        const int Count = 400000;
        for (int k = 0; k < Count; k++)
        {
            const FPointCharts C = ChartsAt(T, Scale(RandomDir(Rng), R));
            Hist[std::min(C.Count, 4)]++;
            double S = 0;
            for (int m = 0; m < C.Count; m++) S += C.Weight[m] * C.Weight[m];
            WorstNorm = std::max(WorstNorm, std::fabs(S - 1));
        }
        std::printf("  charts per point: 0: %d, 1: %.1f%%, 2: %.1f%%, 3: %.1f%%, 4: %.2f%%; worst |sum w^2 - 1| %.1e\n", Hist[0],
            100.0 * Hist[1] / Count, 100.0 * Hist[2] / Count, 100.0 * Hist[3] / Count, 100.0 * Hist[4] / Count, WorstNorm);
        Expect(Hist[0] == 0 && WorstNorm < 1e-5, "every point has a chart and sum w^2 = 1");
    }

    // ---- Continuity --------------------------------------------------------------------------
    std::printf("\nContinuity (weights between points 1 m apart):\n");
    {
        std::mt19937_64 Rng(2);
        double Worst[3] = {};
        const char* Where[3] = { "anywhere", "face edges", "cube corners" };
        for (int k = 0; k < 300000; k++)
        {
            const int Kind = k % 3;
            FVec D = RandomDir(Rng);
            if (Kind == 1)
            {
                // Onto a face edge: two components equal in magnitude, the largest.
                D = { 1.0, std::copysign(1.0, D.Y) * (1.0 + 1e-4 * std::uniform_real_distribution<double>(-1, 1)(Rng)), std::uniform_real_distribution<double>(-1, 1)(Rng) };
                const int Perm = int(Rng() % 3);
                if (Perm == 1) D = { D.Z, D.X, D.Y }; else if (Perm == 2) D = { D.Y, D.Z, D.X };
                if (Rng() & 1) D.X = -D.X;
                D = Normalize(D);
            }
            else if (Kind == 2)
            {
                std::uniform_real_distribution<double> U(-0.02, 0.02);
                D = Normalize({ ((Rng() & 1) ? 1.0 : -1.0) + U(Rng), ((Rng() & 1) ? 1.0 : -1.0) + U(Rng), ((Rng() & 1) ? 1.0 : -1.0) + U(Rng) });
            }
            const FVec P = Scale(D, R);
            const FVec Step = Scale(Normalize(Cross(D, RandomDir(Rng))), 100.0);
            Worst[Kind] = std::max(Worst[Kind], WeightJump(T, P, Add(P, Step)));
        }
        // A weight crosses its band (0.2 of a 53 km cell: about 10 km) at most 1 / 10,000 per metre,
        // more where sqrt(sum w^2) changes fast; a seam would jump by tenths.
        for (int k = 0; k < 3; k++) std::printf("  %s: largest weight change over 1 m %.2e\n", Where[k], Worst[k]);
        Expect(Worst[0] < 1e-3 && Worst[1] < 1e-3 && Worst[2] < 1e-3, "weights continuous everywhere, face edges and corners included");
    }

    // ---- Precision ---------------------------------------------------------------------------
    std::printf("\nPrecision of the GPU's lookups (float, base wrapped then offset) against the CPU's (double), band of 13.7 m:\n");
    {
        std::mt19937_64 Rng(3);
        const double L = 13.7;
        double WorstAbs[2] = {}, WorstRel[2] = {};
        for (int k = 0; k < 200000; k++)
        {
            const FVec Ref = Scale(RandomDir(Rng), R + std::uniform_real_distribution<double>(-500, 50000)(Rng));
            const double Far = (k & 1) ? 5.0e6 : 2.0e4;
            const FVec P = Normalize(Add(Ref, Scale(RandomDir(Rng), std::uniform_real_distribution<double>(0, Far)(Rng))));
            const FVec Point = Scale(P, R);
            // A neighbour 1 cm away: what the pixel-to-pixel pattern sees.
            const FVec Point2 = Add(Point, Scale(Normalize(Cross(P, RandomDir(Rng))), 1.0));
            const FPointCharts C = ChartsAt(T, Point);
            const float3 RefHigh(SplitHigh(Ref.X), SplitHigh(Ref.Y), SplitHigh(Ref.Z));
            const float3 RefLow(SplitLow(Ref.X), SplitLow(Ref.Y), SplitLow(Ref.Z));
            for (int m = 0; m < C.Count; m++)
            {
                double X, Y, X2, Y2;
                Coords(C.Charts[m], Point, X, Y);
                Coords(C.Charts[m], Point2, X2, Y2);
                auto Gpu = [&](const FVec& Pt)
                {
                    const FVec D = Sub(Pt, Ref);
                    const VoxelShader::FVoxelOceanChartCoords G = VoxelShader::VoxelOceanChartCoordsOf(C.Charts[m].Chart, RefHigh, RefLow, float3(float(D.X), float(D.Y), float(D.Z)));
                    return VoxelShader::VoxelOceanWrap(G.Base.x, G.Offset.x, float(1.0 / L));
                };
                auto Wrap = [](double V) { V -= std::floor(V); return V; };
                auto Dist = [&](double A, double B) { double d = std::fabs(A - B); d = std::min(d, 1.0 - d); return d * L; };
                const double U = Gpu(Point), U2 = Gpu(Point2);
                WorstAbs[k & 1] = std::max(WorstAbs[k & 1], Dist(U, Wrap(X / L)));
                // The neighbours' separation, as the GPU sees it, against the true one.
                const double TrueSep = (X2 - X) / L, GpuSep = U2 - U;
                double SepErr = std::fabs(GpuSep - TrueSep);
                SepErr = std::min(SepErr, std::fabs(SepErr - 1.0));
                WorstRel[k & 1] = std::max(WorstRel[k & 1], SepErr * L);
            }
        }
        std::printf("  points within 200 m of the reference: against the CPU up to %.2f mm (a chart's shared shift); between points 1 cm apart %.4f mm\n", WorstAbs[0] * 1000, WorstRel[0] * 1000);
        std::printf("  points up to 50 km away: against the CPU up to %.2f mm; between points 1 cm apart %.2f mm (the positions' own float precision there; pixels are tens of metres)\n", WorstAbs[1] * 1000, WorstRel[1] * 1000);
        Expect(WorstAbs[0] < 0.01 && WorstAbs[1] < 0.03, "lookups within 1 cm of the CPU's near the camera, 3 cm at 50 km");
        Expect(WorstRel[0] < 0.0002, "neighbouring points near the camera keep their separation to 0.2 mm (no pixel-to-pixel noise)");
    }

    // ---- Wind --------------------------------------------------------------------------------
    std::printf("\nWind turn:\n");
    {
        std::mt19937_64 Rng(4);
        double Worst = 0;
        for (int k = 0; k < 100000; k++)
        {
            const FVec P = Scale(RandomDir(Rng), R);
            const FPointCharts C = ChartsAt(T, P);
            const double H = std::uniform_real_distribution<double>(0, 6.283185307)(Rng);
            const FChartCPU& Ch = C.Charts[0];
            const FVec N = Normalize(Ch.Anchor);
            const FVec East = Normalize(Cross({ 0, 0, 1 }, N)), North = Cross(N, East);
            if (std::sqrt(Dot(Cross({ 0, 0, 1 }, N), Cross({ 0, 0, 1 }, N))) < 1e-4) continue;
            const FVec W = Add(Scale(East, std::sin(H)), Scale(North, std::cos(H)));
            const double X = Dot(W, Ch.E1), Y = Dot(W, Ch.E2), L = std::hypot(X, Y);
            const VoxelShader::float2 G = VoxelShader::VoxelOceanChartWind(Ch.Chart, VoxelShader::float2(float(std::sin(H)), float(std::cos(H))));
            Worst = std::max(Worst, std::hypot(G.x - X / L, G.y - Y / L));
        }
        std::printf("  downwind direction error up to %.1e\n", Worst);
        Expect(Worst < 1e-5, "the downwind direction matches the double reference");
    }

    std::printf("\n%s (%d failures)\n", Failures ? "FAILED" : "ALL OK", Failures);
    return Failures ? 1 : 0;
}
