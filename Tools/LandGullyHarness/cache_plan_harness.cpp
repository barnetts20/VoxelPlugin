// The land surface cache's plan (Source/VoxelPlugin/Private/VoxelLandCachePlan.h, the code the
// engine runs) flown over a planet: a simulated atlas records what each job writes (which frame,
// which texel), and every frame checks that
//   - every texel of every level the state calls valid holds that level's frame and texel (the
//     toroidal strips, refills, re-centring into the spare tile and the swap);
//   - the camera's point is read from a valid level, and how fine (a gap: none, or coarser than
//     the footprint asks);
//   - the budget is kept;
//   - edits (MarkDirty, Design/EditsAndCaves.md E2): every texel of a valid level whose column
//     passes through an edit's box is written after the edit, a few frames on (the simulated
//     atlas records the edit count each texel was written at), and no frame's jobs write one
//     slot of a tile with two different texels (a window that moved and an edit's rectangle).
// Flights: a slow walk, a fast flight, an orbital pass, a teleport; brush-sized edits near the
// camera every 37 frames, one 100 km box, one on the far side of the planet.
//
// g++ -O2 -std=c++20 -I<Source/VoxelPlugin/Private> -I<Shaders/Private> cache_plan_harness.cpp
// cache_plan_harness RadiusKm
#include "VoxelLandCachePlan.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <unordered_map>

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
        std::vector<std::vector<uint32_t>> Versions;     // the edit count each texel was written at
        struct FEdit { double Min[3], Max[3]; uint32_t Version; int64_t Due; };
        std::vector<FEdit> Edits;
        uint32_t Version = 0;
        int64_t EditTexelsChecked = 0, Stale = 0, Aliased = 0, DirtyWritten = 0;
        std::unordered_map<uint64_t, uint64_t> FrameSlots;
        std::mt19937 Rng{ 7 };
        int64_t Frames = 0, BadTexels = 0, NoLevel = 0, OverBudget = 0, Worst = 0;
        int64_t FinerMissing = 0;   // frames the camera reads a level coarser than its first valid could be (level 0)
        int32_t WorstLevel = 0;
        static constexpr int Arcs = 5;
        const double ArcDegrees[Arcs] = { 1.0, 10.0, 30.0, 45.0, 60.0 };
        int64_t Uncovered[Arcs] = {};
        int64_t Probes = 0;
        int64_t Reanchors = 0, Swaps = 0, ValidSum = 0, EditCount = 0;

        void Init(double SeaRadius)
        {
            Plan.Init(LevelsFor(SeaRadius), SeaRadius);
            Tiles.assign((size_t)TilesFor((int32_t)Plan.Levels.size()), std::vector<uint64_t>((size_t)N * N, 0));
            Versions.assign(Tiles.size(), std::vector<uint32_t>((size_t)N * N, 0));
        }

        /** An edit: the box (planet frame, cm) changes, the next frame's jobs see it. */
        void AddEdit(const double Min[3], const double Max[3], int64_t SettleFrames)
        {
            Version++;
            FEdit E;
            for (int a = 0; a < 3; a++) { E.Min[a] = Min[a]; E.Max[a] = Max[a]; }
            E.Version = Version;
            E.Due = Frames + SettleFrames;
            Edits.push_back(E);
            Plan.MarkDirty(Min, Max);
        }

        /** Whether the ray from the planet's centre along Dir passes through the box. */
        static bool RayHits(const double Dir[3], const double Min[3], const double Max[3])
        {
            double T0 = 0.0, T1 = 1e30;
            for (int a = 0; a < 3; a++)
            {
                if (std::fabs(Dir[a]) < 1e-300)
                {
                    if (0.0 < Min[a] || 0.0 > Max[a]) return false;
                    continue;
                }
                double A = Min[a] / Dir[a], B = Max[a] / Dir[a];
                if (A > B) std::swap(A, B);
                T0 = std::max(T0, A);
                T1 = std::min(T1, B);
                if (T0 > T1) return false;
            }
            return true;
        }

        /** Texels of valid levels whose columns pass through the edit's box hold a write after it. */
        void CheckEdit(const FEdit& E)
        {
            const double C[3] = { 0.5 * (E.Min[0] + E.Max[0]), 0.5 * (E.Min[1] + E.Max[1]), 0.5 * (E.Min[2] + E.Max[2]) };
            const double Half = 0.5 * std::sqrt((E.Max[0] - E.Min[0]) * (E.Max[0] - E.Min[0]) + (E.Max[1] - E.Min[1]) * (E.Max[1] - E.Min[1]) + (E.Max[2] - E.Min[2]) * (E.Max[2] - E.Min[2]));
            for (const FLevel& L : Plan.Levels)
            {
                if (!L.bValid || Dot(L.A, C) <= 0.0) continue;
                double X, Y;
                PlaneOf(L, C, Plan.SeaRadius, X, Y);
                // Texels about the centre's (the box's reach on the plane, generously).
                const int64_t R = (int64_t)std::ceil(3.0 * Half / L.TexelSize) + 2;
                const int64_t CI = (int64_t)std::floor(X / L.TexelSize), CJ = (int64_t)std::floor(Y / L.TexelSize);
                const int64_t I0 = std::max(CI - R, (int64_t)L.OriginI), I1 = std::min(CI + R, (int64_t)L.OriginI + N - 1);
                const int64_t J0 = std::max(CJ - R, (int64_t)L.OriginJ), J1 = std::min(CJ + R, (int64_t)L.OriginJ + N - 1);
                for (int64_t J = J0; J <= J1; J++)
                    for (int64_t I = I0; I <= I1; I++)
                    {
                        const double U = (I + 0.5) * L.TexelSize, V = (J + 0.5) * L.TexelSize;
                        double Dir[3];
                        for (int r = 0; r < 3; r++) Dir[r] = L.Inverse[r][0] * U + L.Inverse[r][1] * V + L.Inverse[r][2] * Plan.SeaRadius;
                        if (!RayHits(Dir, E.Min, E.Max)) continue;
                        EditTexelsChecked++;
                        const size_t SlotIndex = (size_t)Slot((int32_t)J) * N + Slot((int32_t)I);
                        if (Versions[L.Tile][SlotIndex] < E.Version || !Check(L, (int32_t)I, (int32_t)J))
                        {
                            if (Stale++ < 10 && getenv("PLAN_DEBUG"))
                                printf("    frame %lld: edit %u (due %lld) stale at level %d texel %lld %lld (written at %u, key %s)\n", (long long)Frames, E.Version, (long long)E.Due,
                                    (int)(&L - Plan.Levels.data()), (long long)(I - L.OriginI), (long long)(J - L.OriginJ), Versions[L.Tile][SlotIndex],
                                    Check(L, (int32_t)I, (int32_t)J) ? "ok" : "wrong");
                        }
                    }
            }
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
                    const size_t SlotIndex = (size_t)Slot(J) * N + Slot(I);
                    const uint64_t K = Key(L, I, J);
                    // One frame's jobs run unordered: a slot written twice must get one texel.
                    auto [It, bNew] = FrameSlots.emplace(((uint64_t)L.Tile << 32) | SlotIndex, K);
                    if (!bNew && It->second != K) Aliased++;
                    Tiles[L.Tile][SlotIndex] = K;
                    Versions[L.Tile][SlotIndex] = Version;
                }
        }

        bool Check(const FLevel& L, int32_t I, int32_t J) { return Tiles[L.Tile][(size_t)Slot(J) * N + Slot(I)] == Key(L, I, J); }

        void Step(const double D[3], const FSettings& Settings, bool bFull, bool bWarm)
        {
            std::vector<FJob> Jobs;
            const int32_t PendingBefore = Plan.PendingLevel;
            std::vector<FLevel> Before = Plan.Levels;
            const int64_t DirtyBefore = Plan.DirtyTexels();
            Plan.Tick(D, Settings, Jobs);
            DirtyWritten += std::max<int64_t>(DirtyBefore - Plan.DirtyTexels(), 0);
            FrameSlots.clear();
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
            // The edits due now are written (and every one at the full checks).
            for (const FEdit& E : Edits)
                if (E.Due == Frames || (bFull && E.Due < Frames))
                    CheckEdit(E);
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
    const int Only = getenv("PLAN_FLIGHT") ? atoi(getenv("PLAN_FLIGHT")) : -1;
    for (const FFlight& F : Flights)
    {
        if (Only >= 0 && &F - Flights != Only) continue;
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
        std::uniform_real_distribution<double> Spread(-30000.0, 30000.0), Size(250.0, 1000.0);
        for (int64_t f = 0; f < Count; f++)
        {
            Rotate(D, Axis, F.bTurnBack && f >= Count / 2 ? -Step : Step);
            // Brush-sized edits within 300 m of the camera's ground point; a 100 km box once
            // (every level's window, rewritten in place); one on the far side (nothing to write).
            if (f % 37 == 5 || f == Count / 2 || f == Count / 3)
            {
                const bool bHuge = f == Count / 2;
                const double Sign = f == Count / 3 ? -1.0 : 1.0;
                const double Half = bHuge ? 5.0e6 : Size(Sim.Rng);
                double Min[3], Max[3];
                for (int a = 0; a < 3; a++)
                {
                    const double C = Sign * D[a] * S + (bHuge ? 0.0 : Spread(Sim.Rng));
                    Min[a] = C - Half;
                    Max[a] = C + Half;
                }
                Sim.AddEdit(Min, Max, bHuge ? 200 : 3);
                Sim.EditCount++;
            }
            Sim.Step(D, Settings, f % 500 == 499, true);
        }
        Sim.Step(D, Settings, true, true);
        // The guarantee: everything within 45 degrees of arc (the coarsest level reaches 70 from a
        // centre at most 25 off); past that, while every level is within 10 degrees, to 60.
        const bool bOk = Sim.BadTexels == 0 && Sim.NoLevel == 0 && Sim.OverBudget == 0 && Sim.Uncovered[0] == 0 && Sim.Uncovered[1] == 0
            && Sim.Uncovered[2] == 0 && Sim.Uncovered[3] == 0 && Sim.Stale == 0 && Sim.Aliased == 0;
        Failures += bOk ? 0 : 1;
        printf("%s %s: warm-up %d frames; %lld frames flown; bad texels %lld, frames with no level at the camera %lld, frames over budget %lld (worst %lld texels);"
               " camera read from a coarser level than 0 in %lld frames (worst level %d); %lld in-place re-anchors, %lld gapless re-centrings; %.1f levels valid on average\n",
            bOk ? "OK  " : "FAIL", F.Name, WarmFrames, (long long)Count, (long long)Sim.BadTexels, (long long)Sim.NoLevel, (long long)Sim.OverBudget,
            (long long)Sim.Worst, (long long)Sim.FinerMissing, Sim.WorstLevel, (long long)Sim.Reanchors, (long long)Sim.Swaps, (double)Sim.ValidSum / std::max<int64_t>(Sim.Frames, 1));
        printf("     edits: %lld; texels under them checked %lld, stale %lld; slots written twice with different texels %lld; %lld texels written again for edits\n",
            (long long)Sim.EditCount, (long long)Sim.EditTexelsChecked, (long long)Sim.Stale, (long long)Sim.Aliased, (long long)Sim.DirtyWritten);
        printf("     far field uncovered (of %lld probes each): 1 deg %lld, 10 deg %lld, 30 deg %lld, 45 deg %lld, 60 deg %lld\n", (long long)Sim.Probes,
            (long long)Sim.Uncovered[0], (long long)Sim.Uncovered[1], (long long)Sim.Uncovered[2], (long long)Sim.Uncovered[3], (long long)Sim.Uncovered[4]);
    }
    return Failures;
}
