// eroded_peaks.cpp - A test `peaks` set in Addendum 2's layout (Design/NoiseTiles2D_Addendum2.md),
// made by a simple erosion simulation: for checking the plugin's T3d peaks layer against real
// eroded data before the noise generator project's sets arrive, and as a reference for them.
// Not the production tool (that is the noise generator project's): D8 routing, no glaciers.
//
// The simulation, on a wrapped N x N grid in texel units (heights too, so a slope of 1 is 45 deg):
//   - base level: the Voronoi borders of 16 x 16 jittered massif cells (warped), held at 0;
//   - uplift: 1 elsewhere, scaled per massif by 0.5 .. 1.5 (summit heights vary);
//   - stream-power incision, implicit (Braun & Willett 2013): steepest-descent (D8) receivers
//     with wrapping, the stack from the base level up, E = K A^0.5 S;
//   - threshold hillslopes: h <= neighbour + Sc x distance (chamfer passes after each step);
//   - soft rock (G): a lower Sc and linear diffusion, the same uplift and base level.
// Then B = R blurred (sigma = spacing / 4), A = log2 of R's drainage area (texels).
//
// g++ -O2 -std=c++20 -I<Plugin>/Source/VoxelPlugin/Private eroded_peaks.cpp -o eroded_peaks
// eroded_peaks out.vxnt [N=1024] [seed=1]

#include "VoxelNoiseTiles.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <functional>
#include <string>

using namespace VoxelNoiseTiles;

static void Par(int32_t Count, const std::function<void(int32_t)>& F)
{
    const int T = std::max(1u, std::thread::hardware_concurrency());
    std::atomic<int32_t> Next{ 0 };
    std::vector<std::thread> Th;
    for (int t = 0; t < T; t++) Th.emplace_back([&]() { for (int32_t i; (i = Next.fetch_add(1)) < Count;) F(i); });
    for (auto& t : Th) t.join();
}

struct FSim
{
    int N = 0;
    std::vector<double> H, U;
    std::vector<uint8_t> Base;      // held at 0
    std::vector<int32_t> Recv, Stack;
    std::vector<double> Area;

    int Wrap(int I) const { return (I % N + N) % N; }
    int Id(int X, int Y) const { return Wrap(Y) * N + Wrap(X); }

    void Routing()
    {
        static const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 }, DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
        Recv.assign((size_t)N * N, 0);
        Par(N, [&](int32_t y) {
            for (int x = 0; x < N; x++)
            {
                const int i = y * N + x;
                int Best = i;
                double BestSlope = 0.0;
                if (!Base[i])
                    for (int k = 0; k < 8; k++)
                    {
                        const int j = Id(x + DX[k], y + DY[k]);
                        const double D = k < 4 ? 1.0 : 1.41421356;
                        const double S = (H[i] - H[j]) / D;
                        if (S > BestSlope) { BestSlope = S; Best = j; }
                    }
                Recv[i] = Best;
            }
        });
        // The stack: from the roots (base level and pits) up through the donors.
        std::vector<int32_t> Count((size_t)N * N + 1, 0), Donors((size_t)N * N);
        for (int i = 0; i < N * N; i++) if (Recv[i] != i) Count[Recv[i] + 1]++;
        for (int i = 0; i < N * N; i++) Count[i + 1] += Count[i];
        std::vector<int32_t> Fill(Count.begin(), Count.end() - 1);
        for (int i = 0; i < N * N; i++) if (Recv[i] != i) Donors[Fill[Recv[i]]++] = i;
        Stack.clear();
        Stack.reserve((size_t)N * N);
        std::vector<int32_t> Todo;
        for (int i = 0; i < N * N; i++)
            if (Recv[i] == i)
            {
                Todo.push_back(i);
                while (!Todo.empty())
                {
                    const int c = Todo.back();
                    Todo.pop_back();
                    Stack.push_back(c);
                    for (int k = Count[c]; k < Count[c + 1]; k++) Todo.push_back(Donors[k]);
                }
            }
        Area.assign((size_t)N * N, 1.0);
        for (int s = N * N - 1; s >= 0; s--)
        {
            const int i = Stack[s];
            if (Recv[i] != i) Area[Recv[i]] += Area[i];
        }
    }

    void Step(double Dt, double K, double Sc, double Kappa)
    {
        for (int i = 0; i < N * N; i++) if (!Base[i]) H[i] += U[i] * Dt;
        Routing();
        // Implicit stream power (n = 1), from the base up.
        for (int i : Stack)
        {
            const int r = Recv[i];
            if (r == i || Base[i]) continue;
            const int dx = std::abs(i % N - r % N), dy = std::abs(i / N - r / N);
            const double D = (dx == 0 || dy == 0) ? 1.0 : 1.41421356;
            const double F = K * Dt * std::sqrt(Area[i]) / D;
            H[i] = (H[i] + F * H[r]) / (1.0 + F);
        }
        // Linear diffusion (soft rock).
        if (Kappa > 0.0)
        {
            std::vector<double> Next(H);
            Par(N, [&](int32_t y) {
                for (int x = 0; x < N; x++)
                {
                    const int i = y * N + x;
                    if (Base[i]) continue;
                    const double L = H[Id(x + 1, y)] + H[Id(x - 1, y)] + H[Id(x, y + 1)] + H[Id(x, y - 1)] - 4.0 * H[i];
                    Next[i] = H[i] + Kappa * Dt * L;
                }
            });
            H.swap(Next);
        }
        // Threshold hillslopes: chamfer passes both ways (the wrap needs a second round).
        const double D1 = Sc, D2 = Sc * 1.41421356;
        for (int Round = 0; Round < 2; Round++)
        {
            for (int y = 0; y < N; y++)
                for (int x = 0; x < N; x++)
                {
                    double& h = H[(size_t)y * N + x];
                    h = std::min({ h, H[Id(x - 1, y)] + D1, H[Id(x, y - 1)] + D1, H[Id(x - 1, y - 1)] + D2, H[Id(x + 1, y - 1)] + D2 });
                }
            for (int y = N - 1; y >= 0; y--)
                for (int x = N - 1; x >= 0; x--)
                {
                    double& h = H[(size_t)y * N + x];
                    h = std::min({ h, H[Id(x + 1, y)] + D1, H[Id(x, y + 1)] + D1, H[Id(x + 1, y + 1)] + D2, H[Id(x - 1, y + 1)] + D2 });
                }
        }
    }
};

static void PutU32(std::vector<uint8_t>& B, size_t At, uint32_t V) { for (int k = 0; k < 4; k++) B[At + k] = (uint8_t)(V >> (8 * k)); }
static void PutF32(std::vector<uint8_t>& B, size_t At, float F) { uint32_t U; std::memcpy(&U, &F, 4); PutU32(B, At, U); }

/** Writes Set as .vxnt v3 (the layout VoxelNoiseTiles::ReadVxntHeader documents). */
static bool WriteVxnt(const FSet& Set, const char* Path)
{
    const size_t Header = 128 + 4 * 64 + 16 * (size_t)Set.Mips;
    std::vector<uint8_t> B(Header + Set.Texels.size() * 2, 0);
    std::memcpy(B.data(), "VXNT", 4);
    PutU32(B, 4, 3); PutU32(B, 8, Set.N); PutU32(B, 12, Set.Mips); PutU32(B, 16, 4); PutU32(B, 20, 0);
    PutU32(B, 24, Set.Seed); PutU32(B, 28, (uint32_t)Header);
    std::memcpy(B.data() + 32, Set.Name.c_str(), std::min<size_t>(Set.Name.size(), 31));
    std::memcpy(B.data() + 64, Set.LookTag.c_str(), std::min<size_t>(Set.LookTag.size(), 63));
    for (int c = 0; c < 4; c++)
    {
        const FChannel& C = Set.Channel[c];
        const size_t P = 128 + 64 * (size_t)c;
        PutU32(B, P, C.Kind); PutU32(B, P + 4, C.Seed); PutU32(B, P + 8, C.CellsPerTile); PutU32(B, P + 12, C.Octaves);
        PutF32(B, P + 16, 0.0f); PutF32(B, P + 20, C.Gain); PutF32(B, P + 24, C.Jitter); PutF32(B, P + 28, C.ErosionK);
        PutU32(B, P + 32, (uint32_t)C.Source); PutF32(B, P + 36, C.Min); PutF32(B, P + 40, C.Max); PutF32(B, P + 44, C.Mean); PutF32(B, P + 48, C.Std);
    }
    for (uint32_t m = 0; m < Set.Mips; m++)
    {
        const size_t P = 128 + 256 + 16 * (size_t)m;
        PutU32(B, P, Set.N >> m);
        const uint64_t Off = Header + MipOffset(Set.N, m) * 8;
        PutU32(B, P + 8, (uint32_t)Off); PutU32(B, P + 12, (uint32_t)(Off >> 32));
    }
    std::memcpy(B.data() + Header, Set.Texels.data(), Set.Texels.size() * 2);
    FILE* F = fopen(Path, "wb");
    if (!F) return false;
    const bool bOk = fwrite(B.data(), 1, B.size(), F) == B.size();
    fclose(F);
    return bOk;
}

int main(int argc, char** argv)
{
    if (argc < 2) { printf("eroded_peaks out.vxnt [N] [seed]\n"); return 1; }
    const int N = argc > 2 ? atoi(argv[2]) : 1024;
    const uint32_t Seed = argc > 3 ? (uint32_t)atoi(argv[3]) : 1u;
    constexpr int Massifs = 16;
    const double S = (double)N / Massifs;

    // Base level and uplift, from the massif cells (warped by a smooth fbm).
    FChannel Cell; Cell.Kind = KindCellularBorder; Cell.Seed = Mix(Seed + 101u); Cell.CellsPerTile = Massifs; Cell.Jitter = 0.85f;
    FChannel Warp; Warp.Kind = KindFbm; Warp.Seed = Mix(Seed + 202u); Warp.CellsPerTile = Massifs / 4; Warp.Octaves = 3; Warp.Gain = 0.5f;
    FChannel Warp2 = Warp; Warp2.Seed = Mix(Seed + 303u);
    std::vector<uint8_t> Base((size_t)N * N);
    std::vector<double> Uplift((size_t)N * N);
    Par(N, [&](int32_t y) {
        for (int x = 0; x < N; x++)
        {
            const double K = (double)Massifs / N;
            const double WX = x * K + 0.35 * Evaluate(Warp, x * (double)Warp.CellsPerTile / N, y * (double)Warp.CellsPerTile / N);
            const double WY = y * K + 0.35 * Evaluate(Warp2, x * (double)Warp.CellsPerTile / N, y * (double)Warp.CellsPerTile / N);
            const double Border = Evaluate(Cell, WX, WY);
            FChannel V = Cell; V.Kind = KindCellularValue;
            const double Value = Evaluate(V, WX, WY);
            Base[(size_t)y * N + x] = Border * S < 1.5 ? 1 : 0;
            Uplift[(size_t)y * N + x] = 0.5 + Value;
        }
    });

    auto Run = [&](double Sc, double Kappa, std::vector<double>* OutArea) {
        FSim Sim;
        Sim.N = N; Sim.Base = Base; Sim.U = Uplift;
        Sim.H.assign((size_t)N * N, 0.0);
        // A little initial roughness so the routing has somewhere to start.
        for (int i = 0; i < N * N; i++) Sim.H[i] = Base[i] ? 0.0 : 0.01 * (double)(Hash3((uint32_t)i, Seed, 7u) >> 8) / 16777216.0;
        const double Dt = 0.5, K = getenv("ERODE_K") ? atof(getenv("ERODE_K")) : 1.0;
        const int Steps = 260;
        for (int s = 0; s < Steps; s++)
        {
            Sim.Step(Dt, K, Sc, Kappa);
            if (s % 40 == 0 || s == Steps - 1)
            {
                double Hi = 0; for (double h : Sim.H) Hi = std::max(Hi, h);
                printf("  Sc %.2f step %d: max height %.2f texels\n", Sc, s, Hi);
                fflush(stdout);
            }
        }
        if (OutArea) { Sim.Routing(); *OutArea = Sim.Area; }
        return Sim.H;
    };

    printf("hard rock\n");
    std::vector<double> Area;
    std::vector<double> Hard = Run(0.84, 0.0, &Area);
    printf("soft rock\n");
    // Diffusion: kappa x Dt within 0.25 (the explicit 5-point step's stability).
    std::vector<double> Soft = Run(0.53, 0.4, nullptr);

    FSet Set;
    Set.Name = "peaks_test";
    Set.LookTag = "test erosion (D8 stream power, threshold slopes)";
    Set.Seed = Seed;
    Set.N = (uint32_t)N;
    std::vector<double> Level[4];
    Level[0] = Hard;
    Level[1] = Soft;
    Level[2] = BlurPeriodic(Hard, (uint32_t)N, S / 4.0, [](int32_t Count, auto&& Body) { Par(Count, Body); });
    Level[3].resize((size_t)N * N);
    for (size_t i = 0; i < Area.size(); i++) Level[3][i] = std::log2(Area[i]);
    FChannel& R = Set.Channel[0]; R.Kind = KindErodedHeight; R.Seed = Seed; R.CellsPerTile = Massifs;
    Set.Channel[1] = R;
    FChannel& B = Set.Channel[2]; B.Kind = KindBlurred; B.Source = 0; B.CellsPerTile = 0;
    FChannel& A = Set.Channel[3]; A.Kind = KindFlowAccumulation; A.Source = 0; A.CellsPerTile = 0;
    Finish(Set, Level);

    // Slopes (texel units), for the record.
    for (int c = 0; c < 2; c++)
    {
        const std::vector<double>& Hc = c == 0 ? Hard : Soft;
        std::vector<double> Sl;
        for (int y = 0; y < N; y += 2)
            for (int x = 0; x < N; x += 2)
            {
                auto At = [&](int X, int Y) { return Hc[(size_t)((Y % N + N) % N) * N + (X % N + N) % N]; };
                Sl.push_back(std::hypot(At(x + 1, y) - At(x - 1, y), At(x, y + 1) - At(x, y - 1)) * 0.5);
            }
        std::sort(Sl.begin(), Sl.end());
        auto Q = [&](double q) { return std::atan(Sl[(size_t)(q * (Sl.size() - 1))]) * 57.29578; };
        printf("%s: slope p50 %.1f p90 %.1f p99 %.1f deg; range %.2f .. %.2f texels\n", c == 0 ? "R (hard)" : "G (soft)", Q(0.5), Q(0.9), Q(0.99),
            Set.Channel[c].Min, Set.Channel[c].Max);
    }
    const std::string Why = CheckPeaks(Set);
    if (!Why.empty()) { printf("not a peaks set: %s\n", Why.c_str()); return 1; }
    if (!WriteVxnt(Set, argv[1])) { printf("couldn't write %s\n", argv[1]); return 1; }
    printf("wrote %s\n", argv[1]);
    return 0;
}
