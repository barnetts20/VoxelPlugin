// VoxelLandCachePlan.h - The land surface cache's plan (Design/LandSurfaceCache.md): which
// level is anchored where, which window each level shows, and what to write each frame within a
// texel budget. Plain C++ without engine types, so the harness runs the same code
// (Tools/LandGullyHarness/cache_harness.cpp); FVoxelLandCache (VoxelLandCache.cpp) wraps it,
// turns its jobs into compute passes and uploads the state it describes.
//
// Every frame (Tick, the camera's direction from the planet's centre):
//   0. A re-centring completed last frame takes its level's place (its tile and frame), before
//      any job is planned: a job writes with its level's frame and tile as they stand after the
//      plan, so neither changes once a job of the level's is planned. A re-centring the camera
//      has turned back from (nearer the level's own centre than the new one) is dropped.
//   1. Anchored where they are (and written again): a level never anchored, every level a jump
//      has left far behind (past HardReanchorDegrees), and a level the camera has left (past
//      voxel.Land.CacheReanchorDegrees) that isn't readable anyway.
//   2. A readable level the camera has left re-centres without a gap, one at a time (the
//      farthest, the coarsest of those): its new frame is written into the spare tile while its
//      own tile is still read; the two swap once the new one is complete (0).
//   3. The re-centring, with up to half the budget (it always progresses).
//   E. Texels an edit changed (MarkDirty: the edit's box, projected into every level's frame and
//      the re-centring's), written again in place: brush-sized rectangles fine to coarse with up
//      to half the budget, then large ones with up to a quarter; the level stays readable
//      meanwhile. The jobs are clipped to the windows as they stand after the plan (a window
//      that moves this frame doesn't write a texel it leaves).
//   4. Window moves of readable levels, coarse to fine (the cheap ones first): the strips the
//      move exposes, committed with their writes (a move the budget can't take waits; the old
//      window stays readable; a move of a window or more writes the level again).
//   5. Refills in place, coarse to fine (the material falls back to coarser levels meanwhile),
//      except of a level the camera crosses an eighth of each frame.
//   6. The re-centring again, with what's left.
// A level is readable (valid) only once its whole window is written; the state uploaded after a
// frame's writes describes exactly what they leave.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "VoxelLandCacheLayout.ush"

namespace VoxelLandCachePlan
{
    constexpr int32_t N = VOXEL_LAND_CACHE_N;

    /** The coarsest level reaches this far along the plane from its centre: tan 70 degrees of
     *  the sea-level radius (a re-anchor at 10 degrees leaves 60 degrees of arc about the camera:
     *  the horizon from 1,000 km up at the 1,000 km test scale). */
    constexpr double ReachTangent = 2.7474774194546216;   // tan 70 degrees

    /** Past this arc from its centre a level re-anchors at once, whatever the one-at-a-time
     *  rule: a teleport (the window would sit where the plane stretches, or behind the plane). */
    constexpr double HardReanchorDegrees = 25.0;

    /** Levels for a sea-level radius (cm): the coarsest reaches R tan 70 degrees along the plane,
     *  at least 1, at most VOXEL_LAND_CACHE_MAX_LEVELS. */
    inline int32_t LevelsFor(double SeaRadius)
    {
        const double FinestReach = 0.5 * N * (double)VOXEL_LAND_CACHE_T0;
        const double Reach = (std::max)(SeaRadius * ReachTangent, FinestReach);
        const int32_t Count = (int32_t)std::ceil(std::log2(Reach / FinestReach)) + 1;
        return std::clamp(Count, 1, (int32_t)VOXEL_LAND_CACHE_MAX_LEVELS);
    }

    /** Tiles the atlases need: one a level, and the spare a re-centring level is written into. */
    inline int32_t TilesFor(int32_t Levels) { return Levels + 1; }

    /** A rectangle of texels (global indices). */
    struct FRect
    {
        int32_t I0 = 0;
        int32_t J0 = 0;
        int32_t Width = 0;
        int32_t Height = 0;
    };

    /** A rectangle clipped to the window at (OI, OJ); false when nothing is left. */
    inline bool ClipToWindow(FRect& R, int32_t OI, int32_t OJ)
    {
        const int64_t I0 = (std::max)((int64_t)R.I0, (int64_t)OI);
        const int64_t J0 = (std::max)((int64_t)R.J0, (int64_t)OJ);
        const int64_t I1 = (std::min)((int64_t)R.I0 + R.Width, (int64_t)OI + N);
        const int64_t J1 = (std::min)((int64_t)R.J0 + R.Height, (int64_t)OJ + N);
        if (I1 <= I0 || J1 <= J0)
        {
            R.Width = 0;
            R.Height = 0;
            return false;
        }
        R = FRect{ (int32_t)I0, (int32_t)J0, (int32_t)(I1 - I0), (int32_t)(J1 - J0) };
        return true;
    }

    /** Dirty rectangles kept a level before they merge into their bounds. */
    constexpr size_t MaxDirtyRects = 16;

    /** A dirty rectangle written after the brush-sized ones (more than a sixteenth of a window). */
    inline bool IsLargeRect(const FRect& R) { return (int64_t)R.Width * R.Height > (int64_t)N * N / 16; }

    struct FLevel
    {
        int32_t Tile = 0;                 // the level's tile in the atlases
        bool bAnchored = false;
        float A[3] = { 0, 0, 0 };         // the projection centre (unit, planet frame): floats, as the shaders read them
        float E1[3] = { 0, 0, 0 };
        float E2[3] = { 0, 0, 0 };
        double Inverse[3][3] = {};        // the inverse of the rows E1, E2, A (exact, double)
        double TexelSize = 0.0;           // cm
        int32_t OriginI = 0;              // the window's first texel (global indices)
        int32_t OriginJ = 0;
        bool bValid = false;              // the whole window is written
        int32_t FillRow = 0;              // a refill's progress (rows of the window written)
        std::vector<FRect> Dirty;         // texels an edit changed, to write again (in the window)
    };

    /** A rectangle to write (global texel indices); Frame: the level, or Levels.size() for the
     *  re-centring level's new frame (Next). */
    struct FJob
    {
        int32_t Frame = 0;
        int32_t I0 = 0;
        int32_t J0 = 0;
        int32_t Width = 0;
        int32_t Height = 0;
    };

    struct FSettings
    {
        int64_t Budget = 131072;          // texels a frame (voxel.Land.CacheTexelsPerFrame)
        double ReanchorDegrees = 10.0;    // voxel.Land.CacheReanchorDegrees (1 to 20)
    };

    inline double Dot(const float V[3], const double D[3]) { return (double)V[0] * D[0] + (double)V[1] * D[1] + (double)V[2] * D[2]; }

    inline bool Invert(const double M[3][3], double Out[3][3])
    {
        const double C00 = M[1][1] * M[2][2] - M[1][2] * M[2][1];
        const double C01 = M[1][2] * M[2][0] - M[1][0] * M[2][2];
        const double C02 = M[1][0] * M[2][1] - M[1][1] * M[2][0];
        const double Det = M[0][0] * C00 + M[0][1] * C01 + M[0][2] * C02;
        if (std::fabs(Det) < 1e-12) return false;
        const double Inv = 1.0 / Det;
        Out[0][0] = C00 * Inv;
        Out[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) * Inv;
        Out[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * Inv;
        Out[1][0] = C01 * Inv;
        Out[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * Inv;
        Out[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) * Inv;
        Out[2][0] = C02 * Inv;
        Out[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) * Inv;
        Out[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * Inv;
        return true;
    }

    /** A level's frame about the unit direction D: the axes as floats (the material and the
     *  writer read the same values), the inverse of exactly those in double. */
    inline void Anchor(FLevel& Level, const double D[3])
    {
        auto Normalize = [](double V[3]) {
            const double L = std::sqrt(V[0] * V[0] + V[1] * V[1] + V[2] * V[2]);
            if (L > 0.0) { V[0] /= L; V[1] /= L; V[2] /= L; }
        };
        auto Cross = [](const double X[3], const double Y[3], double Out[3]) {
            Out[0] = X[1] * Y[2] - X[2] * Y[1];
            Out[1] = X[2] * Y[0] - X[0] * Y[2];
            Out[2] = X[0] * Y[1] - X[1] * Y[0];
        };
        double A[3] = { D[0], D[1], D[2] };
        Normalize(A);
        for (int a = 0; a < 3; a++) Level.A[a] = (float)A[a];
        const double Af[3] = { Level.A[0], Level.A[1], Level.A[2] };
        const double Reference[3] = { std::fabs(Af[2]) < 0.9 ? 0.0 : 1.0, 0.0, std::fabs(Af[2]) < 0.9 ? 1.0 : 0.0 };
        double E1[3], E2[3];
        Cross(Reference, Af, E1);
        Normalize(E1);
        for (int a = 0; a < 3; a++) Level.E1[a] = (float)E1[a];
        const double E1f[3] = { Level.E1[0], Level.E1[1], Level.E1[2] };
        Cross(Af, E1f, E2);
        Normalize(E2);
        for (int a = 0; a < 3; a++) Level.E2[a] = (float)E2[a];
        const double M[3][3] = {
            { Level.E1[0], Level.E1[1], Level.E1[2] },
            { Level.E2[0], Level.E2[1], Level.E2[2] },
            { Level.A[0], Level.A[1], Level.A[2] } };
        Invert(M, Level.Inverse);
        Level.bAnchored = true;
    }

    /** The plane coordinates of direction D on a level's grid (cm), in double: as
     *  VoxelLandCachePlaneOf with the same float axes. */
    inline void PlaneOf(const FLevel& Level, const double D[3], double SeaRadius, double& OutX, double& OutY)
    {
        const double PA = Dot(Level.A, D);
        OutX = SeaRadius * Dot(Level.E1, D) / PA;
        OutY = SeaRadius * Dot(Level.E2, D) / PA;
    }

    /** The window's first texel with direction D at its centre. Bounded before the conversion
     *  (Tick keeps the camera within 25 degrees of the centre; this guards a degenerate input). */
    inline void Centre(const FLevel& Level, const double D[3], double SeaRadius, int32_t& OutI, int32_t& OutJ)
    {
        double X = 0.0, Y = 0.0;
        PlaneOf(Level, D, SeaRadius, X, Y);
        const double Limit = (double)(1 << 28);
        auto Index = [Limit](double V) { return std::isfinite(V) ? (int32_t)std::clamp(std::floor(V), -Limit, Limit) : 0; };
        OutI = Index(X / Level.TexelSize) - N / 2;
        OutJ = Index(Y / Level.TexelSize) - N / 2;
    }

    class FPlan
    {
    public:
        std::vector<FLevel> Levels;
        /** A level re-centring without a gap: its new frame and window, written into the spare
         *  tile (Next.Tile) while the level's own tile is still read. -1: none. */
        int32_t PendingLevel = -1;
        FLevel Next;
        int32_t SpareTile = 0;
        double SeaRadius = 0.0;

        void Init(int32_t Count, double InSeaRadius)
        {
            *this = FPlan();
            SeaRadius = InSeaRadius;
            Levels.resize((size_t)Count);
            for (int32_t k = 0; k < Count; k++)
            {
                Levels[k].Tile = k;
                Levels[k].TexelSize = (double)VOXEL_LAND_CACHE_T0 * std::ldexp(1.0, k);
            }
            SpareTile = Count;
        }

        /** Everything is written again (a frame's writes couldn't run). */
        void InvalidateAll()
        {
            for (FLevel& Level : Levels)
            {
                Level.bValid = false;
                Level.FillRow = 0;
                Level.Dirty.clear();
            }
            PendingLevel = -1;
            Next.Dirty.clear();
        }

        /** An edit changed the density in the box (planet frame, cm): the texels whose columns
         *  pass through it, and a texel around them (the gradient's differences), are written
         *  again, in every level's window and the re-centring's (the next Tick plans them). */
        void MarkDirty(const double Min[3], const double Max[3])
        {
            for (FLevel& Level : Levels)
                MarkDirtyIn(Level, Min, Max);
            if (PendingLevel >= 0)
                MarkDirtyIn(Next, Min, Max);
        }

        /** Texels still to write again for edits (the levels' and the re-centring's). */
        int64_t DirtyTexels() const
        {
            int64_t Total = 0;
            auto Add = [&Total](const FLevel& Level) {
                for (const FRect& R : Level.Dirty) Total += (int64_t)R.Width * R.Height;
            };
            for (const FLevel& Level : Levels) Add(Level);
            if (PendingLevel >= 0) Add(Next);
            return Total;
        }

        /** The frame's jobs for the camera's direction D (any length; not zero). */
        void Tick(const double InD[3], const FSettings& Settings, std::vector<FJob>& Jobs)
        {
            const double Length = std::sqrt(InD[0] * InD[0] + InD[1] * InD[1] + InD[2] * InD[2]);
            if (!(Length > 0.0) || !std::isfinite(Length)) return;
            const double D[3] = { InD[0] / Length, InD[1] / Length, InD[2] / Length };
            const double Pi = 3.14159265358979323846;
            const int64_t Total = std::max<int64_t>(Settings.Budget, N);
            int64_t Budget = Total;
            const double ReanchorCos = std::cos(std::clamp(Settings.ReanchorDegrees, 1.0, 20.0) * Pi / 180.0);
            const double HardCos = std::cos(HardReanchorDegrees * Pi / 180.0);
            const int32_t Count = (int32_t)Levels.size();

            // The camera's travel since the last frame (cm along the sea sphere); a jump of more
            // than a degree is a teleport, not a speed.
            double Travel = 0.0;
            if (bHasLast)
            {
                const double Cos = std::clamp(D[0] * Last[0] + D[1] * Last[1] + D[2] * Last[2], -1.0, 1.0);
                const double Arc = std::acos(Cos);
                Travel = Arc < Pi / 180.0 ? Arc * SeaRadius : 0.0;
            }
            Last[0] = D[0]; Last[1] = D[1]; Last[2] = D[2];
            bHasLast = true;

            // 0. A re-centring the camera has turned back from (it's nearer the level's own centre
            // again) is dropped. One completed last frame takes the level's place, before anything
            // is written this frame (a job of the level's must write the frame and tile it was
            // planned with); the old tile is spare.
            if (PendingLevel >= 0 && Dot(Next.A, D) < Dot(Levels[PendingLevel].A, D))
                PendingLevel = -1;
            if (PendingLevel >= 0 && Next.FillRow >= N)
            {
                const int32_t OldTile = Levels[PendingLevel].Tile;
                Next.bValid = true;
                Levels[PendingLevel] = Next;
                SpareTile = OldTile;
                PendingLevel = -1;
            }

            // 1.
            for (int32_t k = 0; k < Count; k++)
            {
                FLevel& Level = Levels[k];
                const double Cos = Dot(Level.A, D);
                const bool bFar = Level.bAnchored && !(Cos >= HardCos);
                const bool bLeft = Level.bAnchored && !(Cos >= ReanchorCos);
                if (!Level.bAnchored || bFar || (bLeft && !Level.bValid))
                {
                    // A re-centring under way for it is moot (the spare tile stays spare).
                    if (PendingLevel == k)
                        PendingLevel = -1;
                    Anchor(Level, D);
                    Centre(Level, D, SeaRadius, Level.OriginI, Level.OriginJ);
                    Level.bValid = false;
                    Level.FillRow = 0;
                    Level.Dirty.clear();
                }
            }

            // 2. The farthest readable level the camera has left; levels anchored together leave
            // together, and then the coarsest goes first (it covers the most; a finer one missing
            // falls back to the next coarser).
            if (PendingLevel < 0)
            {
                double Farthest = ReanchorCos;
                for (int32_t k = Count - 1; k >= 0; k--)
                {
                    const double Cos = Dot(Levels[k].A, D);
                    if (Levels[k].bValid && Cos < Farthest)
                    {
                        Farthest = Cos;
                        PendingLevel = k;
                    }
                }
                if (PendingLevel >= 0)
                {
                    Next = FLevel();
                    Next.Tile = SpareTile;
                    Next.TexelSize = Levels[PendingLevel].TexelSize;
                    Anchor(Next, D);
                    Centre(Next, D, SeaRadius, Next.OriginI, Next.OriginJ);
                }
            }

            // The re-centring level's new frame, up to MaxRows rows of what's left (complete, it
            // takes the level's place at the start of the next frame: 0).
            auto FillPending = [&](int64_t MaxRows)
            {
                if (PendingLevel < 0) return;
                const int32_t Rows = (int32_t)std::min<int64_t>(std::min<int64_t>(N - Next.FillRow, Budget / N), MaxRows);
                if (Rows <= 0) return;
                Jobs.push_back(FJob{ Count, Next.OriginI, Next.OriginJ + Next.FillRow, N, Rows });
                Next.FillRow += Rows;
                Budget -= (int64_t)Rows * N;
            };

            // 3. The re-centring first, with up to half the budget: it always progresses (a
            // level that reaches 25 degrees first re-anchors with a gap).
            FillPending(std::max<int64_t>(Total / 2 / N, 1));

            // E. The edits' texels, after the re-centring's share: the brush-sized rectangles
            // first (fine to coarse, then the re-centring's), with up to half the budget, so a
            // brush near the camera shows at once whatever else is queued; then the large ones
            // (a reload's) with up to a quarter. Clipped again to the final windows at the end.
            std::vector<size_t> DirtyJobs;
            {
                int64_t DirtyBudget = 0;
                auto Drain = [&](FLevel& Level, int32_t Frame, bool bLarge)
                {
                    for (size_t r = 0; r < Level.Dirty.size() && DirtyBudget > 0;)
                    {
                        FRect& R = Level.Dirty[r];
                        if (!ClipToWindow(R, Level.OriginI, Level.OriginJ))
                        {
                            Level.Dirty.erase(Level.Dirty.begin() + (std::ptrdiff_t)r);
                            continue;
                        }
                        if (IsLargeRect(R) != bLarge)
                        {
                            r++;
                            continue;
                        }
                        const int32_t Rows = (int32_t)std::min<int64_t>(R.Height, DirtyBudget / R.Width);
                        if (Rows <= 0) break;
                        Jobs.push_back(FJob{ Frame, R.I0, R.J0, R.Width, Rows });
                        DirtyJobs.push_back(Jobs.size() - 1);
                        const int64_t Cost = (int64_t)Rows * R.Width;
                        DirtyBudget -= Cost;
                        Budget -= Cost;
                        R.J0 += Rows;
                        R.Height -= Rows;
                        if (R.Height <= 0)
                            Level.Dirty.erase(Level.Dirty.begin() + (std::ptrdiff_t)r);
                        else
                            r++;
                    }
                };
                for (int Pass = 0; Pass < 2; Pass++)
                {
                    DirtyBudget = std::min<int64_t>(Budget, Pass == 0 ? Total / 2 : Total / 4);
                    for (int32_t k = 0; k < Count; k++)
                        Drain(Levels[k], k, Pass == 1);
                    if (PendingLevel >= 0)
                        Drain(Next, Count, Pass == 1);
                }
            }

            // 4.
            for (int32_t k = Count - 1; k >= 0; k--)
            {
                FLevel& Level = Levels[k];
                if (!Level.bValid) continue;
                int32_t I = 0, J = 0;
                Centre(Level, D, SeaRadius, I, J);
                const int32_t DI = I - Level.OriginI;
                const int32_t DJ = J - Level.OriginJ;
                if (DI == 0 && DJ == 0) continue;
                const int32_t AI = DI < 0 ? -DI : DI;
                const int32_t AJ = DJ < 0 ? -DJ : DJ;
                if (AI >= N || AJ >= N)
                {
                    // Too far for strips: written again (a re-centring under way for it is
                    // dropped: step 1 re-anchors it in place if it's still off-centre).
                    Level.OriginI = I;
                    Level.OriginJ = J;
                    Level.bValid = false;
                    Level.FillRow = 0;
                    Level.Dirty.clear();
                    if (PendingLevel == k)
                        PendingLevel = -1;
                    continue;
                }
                const int64_t Cost = (int64_t)AI * N + (int64_t)AJ * (N - AI);
                if (Cost > Budget) continue;
                // The columns the new window gains (all its rows), then the rows (the columns
                // both windows share).
                if (DI != 0)
                    Jobs.push_back(FJob{ k, DI > 0 ? Level.OriginI + N : I, J, AI, N });
                if (DJ != 0)
                    Jobs.push_back(FJob{ k, (std::max)(I, Level.OriginI), DJ > 0 ? Level.OriginJ + N : J, N - AI, AJ });
                Level.OriginI = I;
                Level.OriginJ = J;
                Budget -= Cost;
            }

            // 5. Not a level the camera crosses an eighth of each frame: it would be stale before
            // it's readable, and the finest levels can't show at that speed anyway.
            for (int32_t k = Count - 1; k >= 0 && Budget >= N; k--)
            {
                FLevel& Level = Levels[k];
                if (Level.bValid) continue;
                if (Travel > Level.TexelSize * (N / 8)) continue;
                const int32_t Rows = (int32_t)std::min<int64_t>(N - Level.FillRow, Budget / N);
                if (Rows <= 0) continue;
                Jobs.push_back(FJob{ k, Level.OriginI, Level.OriginJ + Level.FillRow, N, Rows });
                Level.FillRow += Rows;
                Budget -= (int64_t)Rows * N;
                if (Level.FillRow >= N)
                    Level.bValid = true;
            }

            // 6.
            FillPending(N);

            // The edits' jobs clipped to the windows as they stand now (the frame the jobs write
            // with): a window that moved this frame shares a texel's slot between the texel it
            // left and the one it gained, and only the latter is written. Overlaps with this
            // frame's other jobs remain, writing the same values.
            for (size_t Index : DirtyJobs)
            {
                FJob& Job = Jobs[Index];
                const FLevel& Level = Job.Frame == Count ? Next : Levels[Job.Frame];
                FRect R{ Job.I0, Job.J0, Job.Width, Job.Height };
                ClipToWindow(R, Level.OriginI, Level.OriginJ);
                Job.I0 = R.I0;
                Job.J0 = R.J0;
                Job.Width = R.Width;
                Job.Height = R.Height;
            }
            Jobs.erase(std::remove_if(Jobs.begin(), Jobs.end(), [](const FJob& Job) { return Job.Width <= 0 || Job.Height <= 0; }), Jobs.end());
        }

    private:
        double Last[3] = { 0, 0, 0 };
        bool bHasLast = false;

        /** The box's texels in a level's frame (its corners projected along their rays: the
         *  projection of the box lies in their bounds), a texel around, within the window. Every
         *  corner past 84 degrees from the level's centre: none (a window reaches 73 at most);
         *  some: the whole window (a box that large is a reload, not a brush). */
        void MarkDirtyIn(FLevel& Level, const double Min[3], const double Max[3]) const
        {
            FRect R;
            if (!DirtyRectOf(Level, Min, Max, R)) return;
            // Coalesced: a rectangle inside one queued with it (both brush-sized, so written as
            // soon) adds nothing; past the limit, all merge into their bounds.
            for (const FRect& Q : Level.Dirty)
                if (IsLargeRect(Q) == IsLargeRect(R) && R.I0 >= Q.I0 && R.J0 >= Q.J0 && R.I0 + R.Width <= Q.I0 + Q.Width && R.J0 + R.Height <= Q.J0 + Q.Height)
                    return;
            Level.Dirty.push_back(R);
            if (Level.Dirty.size() > MaxDirtyRects)
            {
                int64_t I0 = Level.Dirty[0].I0, J0 = Level.Dirty[0].J0, I1 = I0 + Level.Dirty[0].Width, J1 = J0 + Level.Dirty[0].Height;
                for (const FRect& Q : Level.Dirty)
                {
                    I0 = (std::min)(I0, (int64_t)Q.I0); J0 = (std::min)(J0, (int64_t)Q.J0);
                    I1 = (std::max)(I1, (int64_t)Q.I0 + Q.Width); J1 = (std::max)(J1, (int64_t)Q.J0 + Q.Height);
                }
                Level.Dirty.assign(1, FRect{ (int32_t)I0, (int32_t)J0, (int32_t)(I1 - I0), (int32_t)(J1 - J0) });
            }
        }

    public:
        /** Some texel of the box (planet frame, cm; MarkDirty's rectangles) is still to be
         *  written again, in any level's window or the re-centring's (stage O9m: whether an
         *  edit's texels are all in what the writes so far leave). */
        bool IsDirtyIn(const double Min[3], const double Max[3]) const
        {
            auto Overlaps = [&](const FLevel& Level)
            {
                if (Level.Dirty.empty()) return false;
                FRect R;
                if (!DirtyRectOf(Level, Min, Max, R)) return false;
                for (const FRect& Q : Level.Dirty)
                    if (R.I0 < Q.I0 + Q.Width && Q.I0 < R.I0 + R.Width && R.J0 < Q.J0 + Q.Height && Q.J0 < R.J0 + R.Height)
                        return true;
                return false;
            };
            for (const FLevel& Level : Levels)
                if (Overlaps(Level)) return true;
            return PendingLevel >= 0 && Overlaps(Next);
        }

    private:
        /** MarkDirtyIn's rectangle for the box in a level's window; false: none. */
        bool DirtyRectOf(const FLevel& Level, const double Min[3], const double Max[3], FRect& Out) const
        {
            if (!Level.bAnchored) return false;
            double X0 = 0.0, X1 = 0.0, Y0 = 0.0, Y1 = 0.0;
            int32_t Behind = 0, Projected = 0;
            for (int c = 0; c < 8; c++)
            {
                const double P[3] = { (c & 1) ? Max[0] : Min[0], (c & 2) ? Max[1] : Min[1], (c & 4) ? Max[2] : Min[2] };
                const double Length = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
                if (!(Dot(Level.A, P) > 0.1 * Length))
                {
                    Behind++;
                    continue;
                }
                double X = 0.0, Y = 0.0;
                PlaneOf(Level, P, SeaRadius, X, Y);
                if (Projected++ == 0) { X0 = X1 = X; Y0 = Y1 = Y; }
                X0 = (std::min)(X0, X); X1 = (std::max)(X1, X);
                Y0 = (std::min)(Y0, Y); Y1 = (std::max)(Y1, Y);
            }
            if (Behind == 8) return false;
            const bool bWhole = Behind > 0;
            FRect R{ Level.OriginI, Level.OriginJ, N, N };
            if (!bWhole)
            {
                // Bounded before the conversion (the window is what's kept).
                const double Limit = (double)(1 << 29);
                auto Index = [Limit](double V) { return (int64_t)std::clamp(std::floor(V), -Limit, Limit); };
                const int64_t I0 = Index(X0 / Level.TexelSize) - 1, I1 = Index(X1 / Level.TexelSize) + 1;
                const int64_t J0 = Index(Y0 / Level.TexelSize) - 1, J1 = Index(Y1 / Level.TexelSize) + 1;
                if (I1 < Level.OriginI || J1 < Level.OriginJ || I0 >= (int64_t)Level.OriginI + N || J0 >= (int64_t)Level.OriginJ + N) return false;
                const int64_t CI0 = (std::max)(I0, (int64_t)Level.OriginI), CJ0 = (std::max)(J0, (int64_t)Level.OriginJ);
                const int64_t CI1 = (std::min)(I1, (int64_t)Level.OriginI + N - 1), CJ1 = (std::min)(J1, (int64_t)Level.OriginJ + N - 1);
                R = FRect{ (int32_t)CI0, (int32_t)CJ0, (int32_t)(CI1 - CI0 + 1), (int32_t)(CJ1 - CJ0 + 1) };
            }
            Out = R;
            return true;
        }
    };
}
