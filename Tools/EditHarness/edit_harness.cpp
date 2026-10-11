// The terrain edits as shape operations (Design/EditsAndCaves.md, E1): the store's core
// (Source/VoxelPlugin/Private/VoxelEditCore.h) and the GPU's VoxelEditSample.ush compiled as C++,
// over a heightfield-like base density. Checks:
//   1. the GPU code (floats, lattice anchors, the hash table) against the CPU core (doubles);
//   2. compaction: the rules' effect on the surface, against the same edits kept in full;
//      stroke merging against spheres per hit (a deliberate change: the capsule fills the
//      waists between consecutive spheres);
//   3. the slope bound: the edited field's gradient against the base's;
//   4. size and cost: ops, cells, GPU bytes, CPU time per sample.
//
// g++ -O2 -std=c++20 -I<Source/VoxelPlugin/Public> -I<Source/VoxelPlugin/Private> -I<Shaders/Private> edit_harness.cpp
#include "VoxelShaderCompat.h"
#include "VoxelEditCore.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

namespace VoxelShader
{
    std::vector<uint> EditCells;
    std::vector<uint> EditOpList;
    std::vector<uint> EditOps;
    uint EditTableMask = 0;
    uint EditCellShift = 0;
    uint EditOpCount = 0;
    float2 HalfFinest;
#include "VoxelEditSample.ush"
}

using namespace VoxelEditCore;

namespace
{
    constexpr double H = 12.5;          // half finest cell (cm): 25 cm finest cells
    constexpr int32_t Shift = 6;        // 8 m index cells

    /** A rolling heightfield (cm) with slopes up to about 1: the base density is the height
     *  above it (positive in air). */
    double Ground(double X, double Y)
    {
        return 300.0 * std::sin(X * 0.0011) * std::cos(Y * 0.0013) + 120.0 * std::sin(X * 0.0047 + Y * 0.0031);
    }
    double Base(const double P[3]) { return P[2] - Ground(P[0], P[1]); }

    double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

    FState Empty()
    {
        FState S;
        S.HalfFinest = H;
        S.CellShift = Shift;
        return S;
    }

    struct FHit
    {
        EMode Mode;
        double P[3];
        double Radius;
        double Blend;
        bool bContinue;
    };

    /** A brush held while walking: hits along a wandering path at the ground, every Step cm. */
    void Walk(std::vector<FHit>& Hits, std::mt19937& Rng, EMode Mode, double X0, double Y0, double Length, double Step, double Radius, double Turn)
    {
        std::normal_distribution<double> G;
        double X = X0, Y = Y0, Heading = 0.3;
        for (double Done = 0.0; Done <= Length; Done += Step)
        {
            FHit Hit{ Mode, { X, Y, Ground(X, Y) - 0.3 * Radius }, Radius, 0.25 * Radius, Done > 0.0 };
            Hits.push_back(Hit);
            Heading += Turn * G(Rng);
            X += Step * std::cos(Heading);
            Y += Step * std::sin(Heading);
        }
    }

    FState Build(const std::vector<FHit>& Hits, bool bCompact, bool bStrokes, int32_t& OutDropped, double& OutMs)
    {
        FState S = Empty();
        FInsertOptions Options;
        Options.bCompact = bCompact;
        OutDropped = 0;
        const double T0 = Now();
        for (const FHit& Hit : Hits)
        {
            FInsertResult R;
            S = Insert(S, Hit.Mode, Hit.P, Hit.Radius, Hit.Blend, bStrokes && Hit.bContinue, Options, R);
            OutDropped += R.Dropped;
        }
        OutMs = (Now() - T0) * 1000.0;
        return S;
    }

    /** Points near the edited surface: each hit's centre plus random offsets within its reach. */
    std::vector<std::array<double, 3>> Probes(const std::vector<FHit>& Hits, std::mt19937& Rng, int PerHit)
    {
        std::uniform_real_distribution<double> U(-1.0, 1.0);
        std::vector<std::array<double, 3>> Out;
        for (const FHit& Hit : Hits)
            for (int i = 0; i < PerHit; i++)
            {
                const double R = 1.6 * Hit.Radius;
                Out.push_back({ Hit.P[0] + R * U(Rng), Hit.P[1] + R * U(Rng), Hit.P[2] + R * U(Rng) });
            }
        return Out;
    }

    /** The surface's change from A to B at the probes: sign flips, and the largest density change
     *  where either is within Band of zero (cm; with slopes near 1, about the surface's move). */
    void Compare(const FState& A, const FState& B, const std::vector<std::array<double, 3>>& Points, double Band, int64_t& OutFlips, double& OutMaxNear)
    {
        OutFlips = 0;
        OutMaxNear = 0.0;
        for (const auto& P : Points)
        {
            const double Base0 = Base(P.data());
            const double DA = A.Apply(P.data(), 0.0, Base0);
            const double DB = B.Apply(P.data(), 0.0, Base0);
            if ((DA > 0.0) != (DB > 0.0)) OutFlips++;
            if (std::fabs(DA) < Band || std::fabs(DB) < Band)
                OutMaxNear = (std::max)(OutMaxNear, std::fabs(DA - DB));
        }
    }

    void Upload(const FState& S, FGPUArrays& G)
    {
        Flatten(S, G);
        VoxelShader::EditCells = G.Cells;
        VoxelShader::EditOpList = G.OpList;
        VoxelShader::EditOps = G.Ops;
        VoxelShader::EditTableMask = G.TableMask;
        VoxelShader::EditCellShift = G.CellShift;
        VoxelShader::EditOpCount = G.OpCount;
        VoxelShader::HalfFinest = VoxelShader::float2((float)H, 0.0f);
    }
}

int main()
{
    std::mt19937 Rng(11);
    int Failures = 0;

    // Scenarios.
    struct FScenario { std::string Name; std::vector<FHit> Hits = {}; };
    std::vector<FScenario> Scenarios;
    {
        FScenario S{ "tunnel walk: carve held 60 m, a hit every 20 cm (12 m/s at 60 fps... or 3 m/s at 15 hits/s)" };
        Walk(S.Hits, Rng, EMode::Carve, 1000.0, 2000.0, 6000.0, 20.0, 300.0, 0.03);
        Scenarios.push_back(S);
    }
    {
        FScenario S{ "winding walk: carve held 40 m, turning, a hit every 10 cm" };
        Walk(S.Hits, Rng, EMode::Carve, -3000.0, 500.0, 4000.0, 10.0, 300.0, 0.15);
        Scenarios.push_back(S);
    }
    {
        FScenario S{ "clicks in place: 200 carves at one spot (held still), then 50 fills at another" };
        for (int i = 0; i < 200; i++) S.Hits.push_back({ EMode::Carve, { 0.0, 0.0, Ground(0.0, 0.0) }, 300.0, 75.0, i > 0 });
        for (int i = 0; i < 50; i++) S.Hits.push_back({ EMode::Fill, { 900.0, 0.0, Ground(900.0, 0.0) }, 300.0, 75.0, false });
        Scenarios.push_back(S);
    }
    {
        FScenario S{ "small digs, then a big dig over them, then fills over part" };
        std::uniform_real_distribution<double> U(-400.0, 400.0);
        for (int i = 0; i < 60; i++)
        {
            const double X = 5000.0 + U(Rng), Y = -2000.0 + U(Rng);
            S.Hits.push_back({ EMode::Carve, { X, Y, Ground(X, Y) }, 80.0, 20.0, false });
        }
        S.Hits.push_back({ EMode::Carve, { 5000.0, -2000.0, Ground(5000.0, -2000.0) }, 1000.0, 200.0, false });
        for (int i = 0; i < 20; i++)
        {
            const double X = 5000.0 + U(Rng), Y = -2000.0 + U(Rng);
            S.Hits.push_back({ EMode::Fill, { X, Y, Ground(X, Y) - 200.0 }, 150.0, 40.0, false });
        }
        Scenarios.push_back(S);
    }
    {
        FScenario S{ "random mix: 400 carves and fills, radii 1-4 m, clustered" };
        std::uniform_real_distribution<double> U(-1500.0, 1500.0), R(100.0, 400.0), C(0.0, 1.0);
        for (int i = 0; i < 400; i++)
        {
            const double X = -8000.0 + U(Rng), Y = 6000.0 + U(Rng), Radius = R(Rng);
            S.Hits.push_back({ C(Rng) < 0.7 ? EMode::Carve : EMode::Fill, { X, Y, Ground(X, Y) + 0.5 * U(Rng) * 0.1 }, Radius, 0.25 * Radius, false });
        }
        Scenarios.push_back(S);
    }

    {
        FScenario S{ "a held stroke swept between near and far spots (the stroke must break)" };
        for (int i = 0; i < 40; i++)
        {
            const double X = (i / 5) % 2 == 0 ? 0.0 + 10.0 * i : 5000.0 + 10.0 * i, Y = 200.0;
            S.Hits.push_back({ EMode::Carve, { X, Y, Ground(X, Y) }, 300.0, 75.0, i > 0 });
        }
        Scenarios.push_back(S);
    }
    {
        FScenario S{ "mixed blends: 300 edits, blends 0 to the radius" };
        std::uniform_real_distribution<double> U(-1200.0, 1200.0), R(100.0, 400.0), C(0.0, 1.0);
        for (int i = 0; i < 300; i++)
        {
            const double X = 9000.0 + U(Rng), Y = -9000.0 + U(Rng), Radius = R(Rng);
            S.Hits.push_back({ C(Rng) < 0.6 ? EMode::Carve : EMode::Fill, { X, Y, Ground(X, Y) + 0.1 * U(Rng) }, Radius, C(Rng) < 0.3 ? 0.0 : C(Rng) * Radius, false });
        }
        Scenarios.push_back(S);
    }
    {
        // Far from the origin (Earth radius): cells and anchors at planet-scale indices.
        FScenario S{ "the tunnel walk 6,400 km out" };
        for (FHit Hit : Scenarios[0].Hits)
        {
            Hit.P[0] += 6.4e8;
            Hit.P[1] -= 3.1e8;
            Hit.P[2] = Ground(Hit.P[0], Hit.P[1]) - 0.3 * Hit.Radius;
            S.Hits.push_back(Hit);
        }
        Scenarios.push_back(S);
    }

    for (const FScenario& Sc : Scenarios)
    {
        printf("\n== %s: %zu hits\n", Sc.Name.c_str(), Sc.Hits.size());
        int32_t Dropped = 0;
        double Ms = 0.0;
        const FState Full = Build(Sc.Hits, false, false, Dropped, Ms);
        const FState Rules = Build(Sc.Hits, true, false, Dropped, Ms);
        const int32_t RulesDropped = Dropped;
        const FState Strokes = Build(Sc.Hits, true, true, Dropped, Ms);
        const std::vector<std::array<double, 3>> Points = Probes(Sc.Hits, Rng, std::max(4, 40000 / (int)Sc.Hits.size()));

        int64_t Flips = 0;
        double Near = 0.0;
        // Hard edges (no blend): the rules must leave the surface exactly as the full list.
        {
            std::vector<FHit> Hard = Sc.Hits;
            for (FHit& Hit : Hard) Hit.Blend = 0.0;
            int32_t HardDropped = 0;
            double HardMs = 0.0;
            const FState HardFull = Build(Hard, false, false, HardDropped, HardMs);
            const FState HardRules = Build(Hard, true, false, HardDropped, HardMs);
            Compare(HardFull, HardRules, Points, 25.0, Flips, Near);
            printf("  hard edges, rules against full (%zu ops against %zu): %lld probes change side, largest change near the surface %.4f cm\n",
                HardRules.Ops.size(), HardFull.Ops.size(), (long long)Flips, Near);
            if (Flips > 0 || Near > 0.01) Failures++;   // the margin keeps even the values near the surface
        }
        Compare(Full, Rules, Points, 25.0, Flips, Near);
        printf("  ops: %zu in full; %zu after the repetition and containment rules (%d dropped by containment); %zu with stroke merging too (built in %.1f ms)\n",
            Full.Ops.size(), Rules.Ops.size(), RulesDropped, Strokes.Ops.size(), Ms);
        printf("  blended, rules against full: %lld of %zu probes change side; largest density change within 25 cm of the surface %.2f cm (repeats in the full list round the rims again each time)\n",
            (long long)Flips, Points.size(), Near);
        Compare(Full, Strokes, Points, 25.0, Flips, Near);
        printf("  strokes against spheres per hit: %lld change side; largest change near the surface %.2f cm (chords of the path instead of a sphere every hit; the full list also rounds every overlap again)\n", (long long)Flips, Near);

        // Capsules stay within the stroke's limit (a jump breaks the stroke).
        {
            double Longest = 0.0, Limit = 0.0;
            for (const FOp& Op : Strokes.Ops)
            {
                double L2 = 0.0;
                for (int a = 0; a < 3; a++) L2 += (Op.B[a] - Op.A[a]) * (Op.B[a] - Op.A[a]);
                Longest = std::max(Longest, std::sqrt(L2) / Op.Radius);
                Limit = 2.0;
            }
            printf("  longest capsule %.2f radii (limit %.1f)\n", Longest, Limit);
            if (Longest > Limit + 1e-6) Failures++;
        }
        // The cell-local field's sign against the full field's (every op everywhere), near the
        // surfaces: with mixed blends a later blend lifts what earlier edits left.
        {
            int64_t Wrong = 0, NearSurface = 0;
            for (const auto& P : Points)
            {
                const double B = Base(P.data());
                const double Everywhere = Strokes.ApplyWithin(P.data(), 1e30, 0.0, B);
                if (std::fabs(Everywhere) > 100.0) continue;
                NearSurface++;
                Wrong += (Strokes.Apply(P.data(), 0.0, B) > 0.0) != (Everywhere > 0.0) ? 1 : 0;
            }
            printf("  cell-local sign against the full field within 1 m of the surface: %lld wrong of %lld\n", (long long)Wrong, (long long)NearSurface);
            if (Wrong > 0) Failures++;
        }

        // GPU code against the CPU core, at lattice points near the edits.
        FGPUArrays G;
        Upload(Strokes, G);
        double Worst = 0.0;
        int64_t GPUFlips = 0;
        for (const auto& P : Points)
        {
            const int X = (int)std::llround(P[0] / H), Y = (int)std::llround(P[1] / H), Z = (int)std::llround(P[2] / H);
            const double Q[3] = { X * H, Y * H, Z * H };
            const double B = Base(Q);
            const double Cpu = Strokes.Apply(Q, 25.0, B);
            const double Gpu = VoxelShader::VoxelEditApply(X, Y, Z, 25.0f, (float)B);
            Worst = std::max(Worst, std::fabs(Cpu - Gpu));
            if (std::fabs(Cpu) > 0.05 && (Cpu > 0.0) != (Gpu > 0.0)) GPUFlips++;
        }
        printf("  GPU code against the CPU core: worst %.4f cm, %lld sign differences beyond 0.05 cm; %u table slots, %zu cells, %zu op refs, GPU %.1f KB\n",
            Worst, (long long)GPUFlips, G.TableMask + 1, Strokes.Cells.size(), G.OpList.size(),
            (G.Cells.size() * 4 + G.OpList.size() * 4 + G.Ops.size() * 4) / 1024.0);
        if (Worst > 0.5 || GPUFlips > 0) Failures++;

        // The slope bound: central differences over 2 cm, against the base's own.
        double MaxEdited = 0.0, MaxBase = 0.0;
        std::vector<double> Grads;
        for (const auto& P : Points)
        {
            double Ge[3], Gb[3];
            for (int a = 0; a < 3; a++)
            {
                double Pp[3] = { P[0], P[1], P[2] }, Pm[3] = { P[0], P[1], P[2] };
                Pp[a] += 1.0;
                Pm[a] -= 1.0;
                Ge[a] = (Strokes.Apply(Pp, 0.0, Base(Pp)) - Strokes.Apply(Pm, 0.0, Base(Pm))) * 0.5;
                Gb[a] = (Base(Pp) - Base(Pm)) * 0.5;
            }
            const double Le = std::sqrt(Ge[0] * Ge[0] + Ge[1] * Ge[1] + Ge[2] * Ge[2]);
            Grads.push_back(Le);
            MaxEdited = std::max(MaxEdited, Le);
            MaxBase = std::max(MaxBase, std::sqrt(Gb[0] * Gb[0] + Gb[1] * Gb[1] + Gb[2] * Gb[2]));
        }
        std::sort(Grads.begin(), Grads.end());
        const double P999 = Grads[(size_t)(0.999 * (Grads.size() - 1))];
        // The cell-local field jumps deep inside solid (where a cell doesn't list a carve that
        // raises it): the max shows those; the slope tests read ApplyWithin instead (below).
        printf("  slope of the cell-local field: p99.9 %.3f against the base's max %.3f (max %.1f: the jumps deep inside, where cells don't list a carve)\n",
            P999, MaxBase, MaxEdited);

        // The slope-bound test: a ball of radius R about c is free of surface when
        // |field(c)| > L R. Against the full field (every op applied everywhere), with the field
        // at c from c's cell (wrong deep inside) and from ApplyWithin (every op reaching the ball).
        {
            std::uniform_real_distribution<double> U(-1.0, 1.0), Rad(50.0, 2000.0);
            const double L = std::max(MaxBase, 1.0) * 1.02;
            int64_t Tests = 0, CellWrong = 0, WithinWrong = 0, WithinCulls = 0;
            for (int t = 0; t < 3000; t++)
            {
                const FHit& Hit = Sc.Hits[(size_t)t % Sc.Hits.size()];
                const double R = Rad(Rng);
                const double C[3] = { Hit.P[0] + 3.0 * R * U(Rng), Hit.P[1] + 3.0 * R * U(Rng), Hit.P[2] - std::fabs(6.0 * R * U(Rng)) };
                const double DCell = Strokes.Apply(C, 0.0, Base(C));
                const double DWithin = Strokes.ApplyWithin(C, R, 0.0, Base(C));
                const bool bCellCull = std::fabs(DCell) > L * R;
                const bool bWithinCull = std::fabs(DWithin) > L * R;
                if (!bCellCull && !bWithinCull) continue;
                Tests++;
                WithinCulls += bWithinCull ? 1 : 0;
                // Surface in the ball? The full field's sign at points in it.
                const double DC = Strokes.ApplyWithin(C, 1e30, 0.0, Base(C));
                bool bSurface = false;
                for (int k = 0; k < 400 && !bSurface; k++)
                {
                    double Q[3];
                    double Len = 0.0;
                    for (int a = 0; a < 3; a++) { Q[a] = U(Rng); Len += Q[a] * Q[a]; }
                    if (Len > 1.0) { k--; continue; }
                    for (int a = 0; a < 3; a++) Q[a] = C[a] + R * Q[a];
                    const double DQ = Strokes.ApplyWithin(Q, 1e30, 0.0, Base(Q));
                    bSurface = (DQ > 0.0) != (DC > 0.0);
                }
                CellWrong += (bCellCull && bSurface) ? 1 : 0;
                WithinWrong += (bWithinCull && bSurface) ? 1 : 0;
            }
            // ApplyWithin's cell gathering against visiting every op.
            double WorstGather = 0.0;
            for (int t = 0; t < 2000; t++)
            {
                const FHit& Hit = Sc.Hits[(size_t)t % Sc.Hits.size()];
                const double R = Rad(Rng);
                const double C[3] = { Hit.P[0] + 2.0 * R * U(Rng), Hit.P[1] + 2.0 * R * U(Rng), Hit.P[2] + 2.0 * R * U(Rng) };
                double Brute = Base(C);
                for (const FOp& Op : Strokes.Ops)
                    if (SegmentDistance(C, Op.A, Op.B) <= ReachAmong(Op, Strokes.MaxBlend) + R) Brute = ApplyOp(Op, C, 0.0, Brute);
                WorstGather = std::max(WorstGather, std::fabs(Brute - Strokes.ApplyWithin(C, R, 0.0, Base(C))));
            }
            if (WorstGather > 0.0) Failures++;
            printf("  ApplyWithin's cell gathering against every op: worst difference %.6f cm\n", WorstGather);
            printf("  slope-bound test: %lld balls proven empty by either; with the cell's field %lld of those hold surface, with ApplyWithin %lld (of %lld it proves)\n",
                (long long)Tests, (long long)CellWrong, (long long)WithinWrong, (long long)WithinCulls);
            if (WithinWrong > 0) Failures++;
        }

        // CPU cost per sample at the probes.
        const double T0 = Now();
        double Sink = 0.0;
        for (int r = 0; r < 5; r++)
            for (const auto& P : Points)
                Sink += Strokes.Apply(P.data(), 0.0, 0.0);
        printf("  CPU: %.0f ns a sample near the edits (%g)\n", (Now() - T0) * 1e9 / (5.0 * Points.size()), Sink * 0.0);
    }
    printf("\n%s\n", Failures ? "FAILURES" : "all checks passed");
    return Failures;
}
