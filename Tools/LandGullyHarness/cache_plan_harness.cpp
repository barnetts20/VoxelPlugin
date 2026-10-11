// The land surface cache's plan (Source/VoxelPlugin/Private/VoxelLandCachePlan.h, the code the
// engine runs) flown over a planet: a simulated atlas records what each job writes (which frame,
// which texel), and every frame checks that
//   - every texel of every level the state calls valid holds that level's frame and texel (the
//     toroidal strips, refills, re-centring into the spare tile and the swap);
//   - the camera's point is read from a valid level, and how fine (a gap: none, or coarser than
//     the footprint asks);
//   - the budget is kept.
// Flights: a slow walk, a fast flight, an orbital pass, a teleport.
//
// g++ -O2 -std=c++20 -I<Source/VoxelPlugin/Private> -I<Shaders/Private> cache_plan_harness.cpp
// cache_plan_harness RadiusKm
#include "VoxelLandCachePlan.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

using namespace VoxelLandCachePlan;

namespace
{
    uint64_t Key(const FLevel& L, int32_t I, int32_t J)
    {
        uint32_t Bits[4];
        std::memcpy(Bits, L.A, 12);
        uint64_t H = 1469598103934665603ull;
        auto Mix = [&H](uint64_t V) { H ^= V; H *= 1099511628211ull; };
        Mix(Bits[0]); Mix(Bits[1]); Mix(Bits[2]);
        Mix((uint64_t)(int64_t)L.TexelSize);
        Mix((uint64_t)(uint32_t)I); Mix((uint64_t)(uint32_t)J);
        return H | 1ull;   // 0: never written
    }

    struct FSim
    {
        FPlan Plan;
        std::vector<std::vector<uint64_t>> Tiles;
        std::mt19937 Rng{ 7 };
        int64_t Frames = 0, BadTexels = 0, NoLevel = 0, OverBudget = 0, Worst = 0;
        int64_t FinerMissing = 0;   // frames the camera reads a level coarser than its first valid could be (level 0)
        int32_t WorstLevel = 0;
        static constexpr int Arcs = 5;
        const double ArcDegrees[Arcs] = { 1.0, 10.0, 30.0, 45.0, 60.0 };
        int64_t Uncovered[Arcs] = {};
        int64_t Probes = 0;
        int64_t Reanchors = 0, Swaps = 0, ValidSum = 0;

        void Init(double SeaRadius)
        {
            Plan.Init(LevelsFor(SeaRadius), SeaRadius);
            Tiles.assign((size_t)TilesFor((int32_t)Plan.Levels.size()), std::vector<uint64_t>((size_t)N * N, 0));
        }

        int Slot(int32_t I) { return I & (N - 1); }

        int32_t Finest(const double Q[3]) const
        {
            for (int32_t k = 0; k < (int32_t)Plan.Levels.size(); k++)
            {
                const FLevel& L = Plan.Levels[k];
                if (!L.bValid || Dot(L.A, Q) <= 0.0) continue;
                double X, Y;
                PlaneOf(L, Q, Plan.SeaRadius, X, Y);
                const double U = X / L.TexelSize - L.OriginI - 0.5, V = Y / L.TexelSize - L.OriginJ - 0.5;
                if (U >= 0 && V >= 0 && U <= N - 1 && V <= N - 1)
                    return k;
            }
            return -1;
        }

        void Write(const FLevel& L, const FJob& Job)
        {
            for (int32_t j = 0; j < Job.Height; j++)
                for (int32_t i = 0; i < Job.Width; i++)
                {
                    const int32_t I = Job.I0 + i, J = Job.J0 + j;
                    Tiles[L.Tile][(size_t)Slot(J) * N + Slot(I)] = Key(L, I, J);
                }
        }

        bool Check(const FLevel& L, int32_t I, int32_t J) { return Tiles[L.Tile][(size_t)Slot(J) * N + Slot(I)] == Key(L, I, J); }

        void Step(const double D[3], const FSettings& Settings, bool bFull, bool bWarm)
        {
            std::vector<FJob> Jobs;
            const int32_t PendingBefore = Plan.PendingLevel;
            std::vector<FLevel> Before = Plan.Levels;
            Plan.Tick(D, Settings, Jobs);
            // The jobs write with the frames as they stand after the tick (as Dispatch does).
            int64_t Texels = 0;
            for (const FJob& Job : Jobs)
            {
                const FLevel& L = Job.Frame == (int32_t)Plan.Levels.size() ? Plan.Next : Plan.Levels[Job.Frame];
                Write(L, Job);
                Texels += (int64_t)Job.Width * Job.Height;
            }
            Worst = std::max(Worst, Texels);
            if (Texels > std::max<int64_t>(Settings.Budget, N)) OverBudget++;
            (void)PendingBefore;
            for (size_t k = 0; k < Before.size(); k++)
            {
                if (Before[k].Tile != Plan.Levels[k].Tile) Swaps++;
                else if (Before[k].bAnchored && (Before[k].A[0] != Plan.Levels[k].A[0] || Before[k].A[1] != Plan.Levels[k].A[1] || Before[k].A[2] != Plan.Levels[k].A[2]))
                    Reanchors++;
            }
            for (const FLevel& Lv : Plan.Levels) ValidSum += Lv.bValid ? 1 : 0;

            // Valid levels hold their windows (all texels now and then; a sample every frame).
            for (const FLevel& L : Plan.Levels)
            {
                if (!L.bValid) continue;
                if (bFull)
                {
                    for (int32_t j = 0; j < N; j++)
                        for (int32_t i = 0; i < N; i++)
                            BadTexels += Check(L, L.OriginI + i, L.OriginJ + j) ? 0 : 1;
                }
                else
                {
                    std::uniform_int_distribution<int32_t> U(0, N - 1);
                    for (int s = 0; s < 256; s++)
                    {
                        const int32_t I = L.OriginI + U(Rng), J = L.OriginJ + U(Rng);
                        if (!Check(L, I, J))
                        {
                            if (BadTexels++ < 3 && getenv("PLAN_DEBUG"))
                                printf("    frame %lld: level %d (tile %d, origin %d %d) texel %d %d wrong (pending %d, spare %d)\n", (long long)Frames,
                                    (int)(&L - Plan.Levels.data()), L.Tile, L.OriginI, L.OriginJ, I - L.OriginI, J - L.OriginJ, Plan.PendingLevel, Plan.SpareTile);
                        }
                    }
                }
            }
            // The tiles in use are distinct, and the spare is none of them.
            std::vector<int> Used(Tiles.size(), 0);
            for (const FLevel& L : Plan.Levels) Used[L.Tile]++;
            Used[Plan.SpareTile]++;
            for (int u : Used) if (u != 1) { BadTexels += 1000000; break; }

            // The camera's point: the finest valid level whose window holds it (as the sampler);
            // and points around it out to 60 degrees of arc (the far field).
            if (bWarm)
            {
                const int32_t Found = Finest(D);
                if (Found < 0) NoLevel++;
                if (Found > 0) FinerMissing++;
                WorstLevel = std::max(WorstLevel, Found);
                double E1[3], E2[3];
                const double Ref[3] = { 0.0, 0.0, 1.0 };
                E1[0] = Ref[1] * D[2] - Ref[2] * D[1]; E1[1] = Ref[2] * D[0] - Ref[0] * D[2]; E1[2] = Ref[0] * D[1] - Ref[1] * D[0];
                const double L1 = std::sqrt(E1[0] * E1[0] + E1[1] * E1[1] + E1[2] * E1[2]);
                for (double& V : E1) V /= L1;
                E2[0] = D[1] * E1[2] - D[2] * E1[1]; E2[1] = D[2] * E1[0] - D[0] * E1[2]; E2[2] = D[0] * E1[1] - D[1] * E1[0];
                std::uniform_real_distribution<double> Az(0.0, 6.283185307179586);
                for (int p = 0; p < 4; p++)
                {
                    const double Phi = Az(Rng);
                    for (int a = 0; a < Arcs; a++)
                    {
                        const double T = ArcDegrees[a] * 3.14159265358979 / 180.0;
                        double Q[3];
                        for (int c = 0; c < 3; c++) Q[c] = D[c] * std::cos(T) + (E1[c] * std::cos(Phi) + E2[c] * std::sin(Phi)) * std::sin(T);
                        Uncovered[a] += Finest(Q) < 0 ? 1 : 0;
                    }
                    Probes++;
                }
            }
            Frames++;
        }
    };

    void Rotate(double D[3], const double Axis[3], double Angle)
    {
        // Rodrigues.
        const double C = std::cos(Angle), S = std::sin(Angle);
        const double KD = Axis[0] * D[0] + Axis[1] * D[1] + Axis[2] * D[2];
        const double KxD[3] = { Axis[1] * D[2] - Axis[2] * D[1], Axis[2] * D[0] - Axis[0] * D[2], Axis[0] * D[1] - Axis[1] * D[0] };
        for (int a = 0; a < 3; a++) D[a] = D[a] * C + KxD[a] * S + Axis[a] * KD * (1 - C);
    }
}

int main(int argc, char** argv)
{
    const double RadiusKm = argc > 1 ? atof(argv[1]) : 1000.0;
    const double S = RadiusKm * 1e5;   // cm
    printf("planet %.0f km: %d levels, %d tiles\n", RadiusKm, LevelsFor(S), TilesFor(LevelsFor(S)));

    struct FFlight { const char* Name; double SpeedMPerFrame; double ArcDegrees; double TeleportDegrees; bool bTurnBack = false; };
    const FFlight Flights[] = {
        { "walk 2 m/frame (120 m/s), 0.5 deg", 2.0, 0.5, 0.0 },
        { "jet 20 m/frame (1.2 km/s), 25 deg", 20.0, 25.0, 0.0 },
        { "jet, 11 deg out and back (turning near the re-centring)", 50.0, 22.0, 0.0, true },
        { "flight 200 m/frame (12 km/s), 45 deg", 200.0, 45.0, 0.0 },
        { "orbit 130 m/frame (7.8 km/s), 30 deg", 130.0, 30.0, 0.0 },
        { "fast 2 km/frame, 60 deg", 2000.0, 60.0, 0.0 },
        { "teleport 90 deg, then 20 m/frame for 0.2 deg", 20.0, 0.2, 90.0 },
    };
    int Failures = 0;
    for (const FFlight& F : Flights)
    {
        FSim Sim;
        Sim.Init(S);
        FSettings Settings;
        double D[3] = { 0.3, -0.5, 0.81 };
        const double L = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
        for (double& V : D) V /= L;
        const double Axis0[3] = { 0.0, 0.0, 1.0 };
        double Axis[3] = { D[1] * Axis0[2] - D[2] * Axis0[1], D[2] * Axis0[0] - D[0] * Axis0[2], D[0] * Axis0[1] - D[1] * Axis0[0] };
        const double AL = std::sqrt(Axis[0] * Axis[0] + Axis[1] * Axis[1] + Axis[2] * Axis[2]);
        for (double& V : Axis) V /= AL;
        // Warm-up: every level written at the start.
        int Warm = 0;
        while (Warm < 200)
        {
            Sim.Step(D, Settings, false, false);
            bool bAll = true;
            for (const FLevel& Lv : Sim.Plan.Levels) bAll = bAll && Lv.bValid;
            Warm++;
            if (bAll) break;
        }
        const int WarmFrames = Warm;
        if (F.TeleportDegrees > 0.0)
        {
            Rotate(D, Axis, F.TeleportDegrees * 3.14159265358979 / 180.0);
            // The frames after a teleport aren't counted as gaps until everything is written again.
            int Refill = 0;
            for (; Refill < 200; Refill++)
            {
                Sim.Step(D, Settings, false, false);
                bool bAll = true;
                for (const FLevel& Lv : Sim.Plan.Levels) bAll = bAll && Lv.bValid;
                if (bAll) break;
            }
            printf("  teleport: all levels written again in %d frames\n", Refill + 1);
        }
        const double Step = F.SpeedMPerFrame * 100.0 / S;
        const int64_t Count = (int64_t)(F.ArcDegrees * 3.14159265358979 / 180.0 / Step);
        for (int64_t f = 0; f < Count; f++)
        {
            Rotate(D, Axis, F.bTurnBack && f >= Count / 2 ? -Step : Step);
            Sim.Step(D, Settings, f % 500 == 499, true);
        }
        Sim.Step(D, Settings, true, true);
        // The guarantee: everything within 45 degrees of arc (the coarsest level reaches 70 from a
        // centre at most 25 off); past that, while every level is within 10 degrees, to 60.
        const bool bOk = Sim.BadTexels == 0 && Sim.NoLevel == 0 && Sim.OverBudget == 0 && Sim.Uncovered[0] == 0 && Sim.Uncovered[1] == 0
            && Sim.Uncovered[2] == 0 && Sim.Uncovered[3] == 0;
        Failures += bOk ? 0 : 1;
        printf("%s %s: warm-up %d frames; %lld frames flown; bad texels %lld, frames with no level at the camera %lld, frames over budget %lld (worst %lld texels);"
               " camera read from a coarser level than 0 in %lld frames (worst level %d); %lld in-place re-anchors, %lld gapless re-centrings; %.1f levels valid on average\n",
            bOk ? "OK  " : "FAIL", F.Name, WarmFrames, (long long)Count, (long long)Sim.BadTexels, (long long)Sim.NoLevel, (long long)Sim.OverBudget,
            (long long)Sim.Worst, (long long)Sim.FinerMissing, Sim.WorstLevel, (long long)Sim.Reanchors, (long long)Sim.Swaps, (double)Sim.ValidSum / std::max<int64_t>(Sim.Frames, 1));
        printf("     far field uncovered (of %lld probes each): 1 deg %lld, 10 deg %lld, 30 deg %lld, 45 deg %lld, 60 deg %lld\n", (long long)Sim.Probes,
            (long long)Sim.Uncovered[0], (long long)Sim.Uncovered[1], (long long)Sim.Uncovered[2], (long long)Sim.Uncovered[3], (long long)Sim.Uncovered[4]);
    }
    return Failures;
}
