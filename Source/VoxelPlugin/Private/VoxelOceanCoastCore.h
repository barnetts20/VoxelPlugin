// VoxelOceanCoastCore.h - The coastal travel-time field on the CPU (stage O9h; Design/OceanWaves.md
// section 23; VoxelOceanCoast.ush has the layout and the sampling). The plan (which bake cells
// get a tile, how fine), the solve (from the fine cells' depths, which the GPU samples off the
// terrain: FVoxelOceanPages::SampleCoastDepths), the aprons and the packing into texels.
//
// THE SOLVE. Over the tiles' fine cells joined as one grid across tiles and face edges: the fast
// marching method (first arrivals, cell by cell in order of A), each cell's value from its
// already-known neighbours by the triangle update (the eikonal on the two neighbours' segment,
// with their 3D positions: exact on the cube-sphere's skewed, non-uniform cells and across face
// edges), slowness 1 / sqrt(held depth). B is carried along the same characteristic (B at the
// update's foot plus sqrt(held depth) times its length), both by the trapezoid rule. Its sources:
// water at least SeedDepth deep and water past Reach of the shore field's distance (each at its
// distance past that edge, so the edge's zero lies between the cells), and water on the tiles'
// edge toward the open sea. Land is crossed at the floor's slowness, as the shallowest water:
// the swash finds a phase up the beach that continues its shoreline's, and the interpolation
// near the shoreline sees no corner. Cells no ray reaches (cut off within the tiles) are untrusted.
//
// THE WINDOW (stage W7a). A planet's every coast at fine cells is tens of millions of cells (40
// million and half a minute on a 1,000 km planet), so by default the field covers only the coasts
// round the camera: the tiles within WindowRadius + WindowFade (the cover's ramp, applied by the
// shaders: VoxelOceanCoastWindowCover) + 1.25 WindowMove (the camera's travel before the next
// window) + the halo. The halo makes the window's values those of the whole planet's solve: a
// point's first arrival comes from sources within the reach of it (deep water, or water past
// Reach), so cutting the grid a halo past the cover changes no value the cover shows (coast_check
// measures it). The window's cut is closed (no sources there), unlike the tiles' edge toward the
// open sea. WindowRadius 0: the whole planet, as before.
//
// Plain C++ (std containers, no engine types): the plugin (VoxelOceanCoastField.cpp) and the
// harness (Tools/OceanHarness/coast_check.cpp).

#pragma once

#include "VoxelOceanShoreCore.h"

namespace VoxelShader
{
#include "VoxelOceanCoast.ush"
}

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace VoxelOceanCoastCore
{
    constexpr int Pad = VOXEL_OCEAN_SHORE_PAD;
    constexpr int Apron = VOXEL_OCEAN_COAST_APRON;
    constexpr double Pi = 3.141592653589793;
    /** The atlas's slices at most (a texture array's 2048 less the field's and the index's 12). */
    constexpr int MaxAtlasSlices = 2048 - 12;

    struct FSettings
    {
        double SeedDepthM = 40.0;        // the outer edge's depth
        double ReachM = 8000.0;          // the outer edge's farthest distance (the shore field's)
        double LandReachM = 3000.0;      // water on the field's land side kept within this
        double LowLandM = 2.0;           // a land-side bake cell has water near if a neighbour is below this
        double BudgetBytes = 128.0 * 1024.0 * 1024.0;   // the atlas's
        double MinCellM = 25.0;
        double MaxCellM = 150.0;
        int MaxR = 64;
        /** The fine cells sampled and solved at most (the solve's time and memory: about 3 us and
         *  25 bytes a cell on a large planet). */
        double MaxCells = 40.0e6;
        /** Stage W7a: the window round the camera (m: the full cover's radius; 0: the whole
         *  planet), the cover's ramp past it, the camera's travel before the next window, the halo
         *  past them (0: the larger of Reach and 4 km), the cells' size (m) and their most. */
        double WindowRadiusM = 16000.0;
        double WindowFadeM = 8000.0;
        double WindowMoveM = 4000.0;
        double WindowHaloM = 0.0;
        double WindowCellM = 40.0;
        double WindowMaxCells = 4.0e6;
        /** The cover's blend from one window to the next, and its fade in (s). */
        double WindowBlendSeconds = 1.5;

        bool IsWindowed() const { return WindowRadiusM > 0.0; }
        double HaloM() const { return WindowHaloM > 0.0 ? WindowHaloM : std::max(ReachM, 4000.0); }
        /** The tiles' radius round the window's centre. */
        double SolveRadiusM() const { return WindowRadiusM + WindowFadeM + 1.25 * WindowMoveM + HaloM(); }

        bool operator==(const FSettings& O) const
        {
            return SeedDepthM == O.SeedDepthM && ReachM == O.ReachM && LandReachM == O.LandReachM && LowLandM == O.LowLandM && BudgetBytes == O.BudgetBytes
                && MinCellM == O.MinCellM && MaxCellM == O.MaxCellM && MaxR == O.MaxR && MaxCells == O.MaxCells
                && WindowRadiusM == O.WindowRadiusM && WindowFadeM == O.WindowFadeM && WindowMoveM == O.WindowMoveM && WindowHaloM == O.WindowHaloM
                && WindowCellM == O.WindowCellM && WindowMaxCells == O.WindowMaxCells && WindowBlendSeconds == O.WindowBlendSeconds;
        }
        bool operator!=(const FSettings& O) const { return !(*this == O); }
    };

    /** Stage W7a: the window's centre (planet frame, unit), when the settings ask for one. */
    struct FWindow
    {
        bool bOn = false;
        double Centre[3] = { 0.0, 0.0, 1.0 };
    };

    /** A tile: bake cell (I, J) of a face (I, J from -Pad: past a face's edge, an apron tile). */
    struct FTile
    {
        int Face = 0;
        int I = 0;
        int J = 0;
    };

    struct FPlan
    {
        bool bValid = false;
        int N = 0;
        int R = 1;
        double RadiusM = 0.0;
        double CellM = 0.0;              // a fine cell's arc at a face's centre
        double BakeCellM = 0.0;
        FSettings Settings;
        /** Interior tiles first (InteriorTiles of them: solved), then the apron tiles. */
        std::vector<FTile> Tiles;
        int InteriorTiles = 0;
        /** Per index texel (6 Side^2, the shore field's layout): the tile, or -1. */
        std::vector<int> TileOf;
        int TilesPerRow = 0;
        int TilesPerColumn = 0;
        int AtlasSlices = 0;
        double AtlasBytes = 0.0;
        /** Candidates left out to fit the budget (the farthest from the coast). */
        int TrimmedTiles = 0;
        /** tan(pi/4 s) of the fine cells' centres from -Margin (TanTable[f + Margin]). */
        std::vector<double> TanTable;
        int TanMargin = 0;
        /** Stage W7a: the window (its centre), the atlas's slices the texture holds for any window
         *  (the same for every window of these settings: the texture is written in place), and per
         *  index texel the bake cells the window cut off (a tile's but outside: a closed edge, no
         *  sources). */
        FWindow Window;
        int CapacitySlices = 0;
        std::vector<uint8_t> Cut;

        int FineN() const { return N * R; }
        int CellsPerTile() const { return R * R; }
        size_t InteriorCells() const { return size_t(InteriorTiles) * size_t(R) * size_t(R); }
        int Stride() const { return R + 2 * Apron; }
        /** The coast's texels: the index's 6 slices and the atlas's. */
        int Slices() const { return 6 + std::max(AtlasSlices, CapacitySlices); }
    };

    // ---------------------------------------------------------------------------------------
    // Geometry (double)
    // ---------------------------------------------------------------------------------------

    struct FBasis
    {
        double Nrm[3];
        double U[3];
        double V[3];
    };

    inline FBasis BasisOf(int Face)
    {
        FBasis B;
        const unsigned F = unsigned(Face);
        const unsigned NA = VoxelShader::VoxelCubeNormalAxis(F), UA = VoxelShader::VoxelCubeUAxis(F), VA = VoxelShader::VoxelCubeVAxis(F);
        const double Sign = VoxelShader::VoxelCubeNormalSign(F);
        for (int k = 0; k < 3; k++)
        {
            B.Nrm[k] = (unsigned(k) == NA ? 1.0 : 0.0) * Sign;
            B.U[k] = unsigned(k) == UA ? 1.0 : 0.0;
            B.V[k] = unsigned(k) == VA ? 1.0 : 0.0;
        }
        return B;
    }

    inline const FBasis& Basis(int Face)
    {
        static const FBasis All[6] = { BasisOf(0), BasisOf(1), BasisOf(2), BasisOf(3), BasisOf(4), BasisOf(5) };
        return All[Face];
    }

    inline void Normalize(double D[3])
    {
        const double L = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
        if (L > 0.0) { D[0] /= L; D[1] /= L; D[2] /= L; }
    }

    /** A direction from a face's tangents of its coordinates. */
    inline void DirOfTan(int Face, double TU, double TV, double Out[3])
    {
        const FBasis& B = Basis(Face);
        for (int k = 0; k < 3; k++) Out[k] = B.Nrm[k] + B.U[k] * TU + B.V[k] * TV;
        Normalize(Out);
    }

    /** The face holding a direction and its equal-angle coordinates. */
    inline void Locate(const double D[3], int& Face, double& S, double& T)
    {
        int Axis = 0;
        if (std::fabs(D[1]) > std::fabs(D[Axis])) Axis = 1;
        if (std::fabs(D[2]) > std::fabs(D[Axis])) Axis = 2;
        Face = Axis * 2 + (D[Axis] < 0.0 ? 1 : 0);
        const FBasis& B = Basis(Face);
        const double Nn = D[0] * B.Nrm[0] + D[1] * B.Nrm[1] + D[2] * B.Nrm[2];
        const double Du = D[0] * B.U[0] + D[1] * B.U[1] + D[2] * B.U[2];
        const double Dv = D[0] * B.V[0] + D[1] * B.V[1] + D[2] * B.V[2];
        S = std::atan(Du / Nn) * 4.0 / Pi;
        T = std::atan(Dv / Nn) * 4.0 / Pi;
    }

    inline double TanOf(const FPlan& P, int F)
    {
        const int K = F + P.TanMargin;
        if (K >= 0 && K < int(P.TanTable.size())) return P.TanTable[size_t(K)];
        return std::tan(0.25 * Pi * (-1.0 + (2.0 * F + 1.0) / double(P.FineN())));
    }

    /** Fine cell (FI, FJ) of a face's centre direction (either may be past the face's edge: the
     *  face continued). */
    inline void FineDir(const FPlan& P, int Face, int FI, int FJ, double Out[3])
    {
        DirOfTan(Face, TanOf(P, FI), TanOf(P, FJ), Out);
    }

    /** The bake cell holding a fine cell (floor division). */
    inline int BakeOf(int F, int R) { return F >= 0 ? F / R : -((-F + R - 1) / R); }

    inline int IndexTexel(int N, int Face, int I, int J)
    {
        const int S = VoxelOceanShoreCore::Side(N);
        return (Face * S + J + Pad) * S + I + Pad;
    }

    /** Stage W7a: whether the window cut bake cell (I, J) of a face off (within the faces). */
    inline bool IsCut(const FPlan& P, int Face, int I, int J)
    {
        return !P.Cut.empty() && I >= 0 && J >= 0 && I < P.N && J < P.N && P.Cut[size_t(IndexTexel(P.N, Face, I, J))] != 0;
    }

    /** The centre direction of bake cell (I, J) of a face (double). */
    inline void BakeCellDir(int N, int Face, int I, int J, double Out[3])
    {
        const double TU = std::tan(0.25 * Pi * (-1.0 + (2.0 * I + 1.0) / double(N)));
        const double TV = std::tan(0.25 * Pi * (-1.0 + (2.0 * J + 1.0) / double(N)));
        DirOfTan(Face, TU, TV, Out);
    }

    // ---------------------------------------------------------------------------------------
    // The plan
    // ---------------------------------------------------------------------------------------

    /** Stage W7b: the bake cells that want a tile (the plan's candidates: the selection below), with
     *  their distance from the coast (m; the ring's a cell farther), and the reach as held. The
     *  same for every window of a shore field and these settings: the builder keeps them. */
    struct FCoastCell
    {
        float Distance;
        int Cell;          // (Face N + J) N + I
    };

    struct FCoastCells
    {
        bool bValid = false;
        double ReachM = 0.0;
        std::vector<FCoastCell> Cells;
    };

    inline FCoastCells CoastCells(int N, double RadiusM, const uint32_t* Shore, double ShoreReachM, const float* Height, const FSettings& Settings)
    {
        FCoastCells Out;
        if (N <= 0 || !Shore || !Height || !(RadiusM > 0.0)) return Out;
        const double Cell = 0.5 * Pi * RadiusM / double(N);
        const double ReachM = std::min(Settings.ReachM, ShoreReachM - 2.0 * Cell);
        if (!(ReachM > 0.0)) return Out;
        Out.ReachM = ReachM;
        Out.bValid = true;
        auto Distance = [&](int Face, int I, int J)
            {
                return double(VoxelShader::VoxelOceanShoreUnpack(Shore[size_t(IndexTexel(N, Face, I, J))], float(ShoreReachM)).Distance);
            };
        auto HeightAt = [&](int Face, int I, int J) { return double(Height[size_t(IndexTexel(N, Face, I, J))]); };
        // The interior candidates, nearest the coast first.
        std::vector<FCoastCell>& Candidates = Out.Cells;
        for (int Face = 0; Face < 6; Face++)
            for (int J = 0; J < N; J++)
                for (int I = 0; I < N; I++)
                {
                    const double D = Distance(Face, I, J);
                    // (Stage W5k: tolerant, better a tile too many than one missing: the bake's
                    // heights are cell means, and a coast finer than a cell, or bent across one,
                    // left a coastal cell without a tile (the shore's trains stopped at its edge).
                    // Two cells round instead of one, the coast's band 2.5 cells instead of 1.5,
                    // any cell whose neighbourhood holds both ground near the sea's level and
                    // shallow water, and below a ring round every chosen cell.)
                    double Lowest = 1.0e30, Highest = -1.0e30;
                    for (int dj = -2; dj <= 2; dj++)
                        for (int di = -2; di <= 2; di++)
                        {
                            // The apron's cells hold the neighbouring face's heights.
                            const double H = HeightAt(Face, I + di, J + dj);
                            Lowest = std::min(Lowest, H);
                            Highest = std::max(Highest, H);
                        }
                    const bool bCoast = std::fabs(D) < 2.5 * Cell;
                    const bool bSea = D > 0.0 && D < ReachM + 1.5 * Cell && Highest > -1.5 * Settings.SeedDepthM;
                    const bool bLowland = D <= 0.0 && D > -Settings.LandReachM && Lowest < Settings.LowLandM;
                    // (Not deep inland: land is crossed at the floor's slowness, and tiles far up it
                    // let rays come round to the shore across it, from the wrong side.)
                    const bool bStraddle = D > -2.5 * Cell && D < ReachM + 1.5 * Cell && Lowest < Settings.LowLandM && Highest > -1.5 * Settings.SeedDepthM;
                    if (bCoast || bSea || bLowland || bStraddle)
                        Candidates.push_back({ float(std::fabs(D)), (Face * N + J) * N + I });
                }
        if (Candidates.empty()) return Out;
        // Stage W5k: a ring of one cell round every candidate (within the faces; the aprons past
        // their edges follow below), after them in the order (a cell farther than its neighbour).
        {
            std::vector<uint8_t> Chosen(size_t(6) * size_t(N) * size_t(N), 0);
            for (const FCoastCell& C : Candidates) Chosen[size_t(C.Cell)] = 1;
            const size_t Before = Candidates.size();
            for (size_t k = 0; k < Before; k++)
            {
                const int Cell0 = Candidates[k].Cell;
                const int Face = Cell0 / (N * N), J = (Cell0 / N) % N, I = Cell0 % N;
                for (int dj = -1; dj <= 1; dj++)
                    for (int di = -1; di <= 1; di++)
                    {
                        const int I1 = I + di, J1 = J + dj;
                        if (I1 < 0 || J1 < 0 || I1 >= N || J1 >= N) continue;
                        const int Cell1 = (Face * N + J1) * N + I1;
                        if (Chosen[size_t(Cell1)]) continue;
                        // (The sea's side and the coast's band only: not up the land, as above.)
                        const double D1 = Distance(Face, I1, J1);
                        if (D1 < -2.5 * Cell) continue;
                        Chosen[size_t(Cell1)] = 1;
                        Candidates.push_back({ float(std::fabs(D1)) + float(Cell), Cell1 });
                    }
            }
        }
        return Out;
    }

    /** The bake cells near a coast and how fine their tiles are. Shore: the shore field's texels
     *  (VoxelOceanShore.ush, 6 Side^2) and its reach; Height: the land bake's heights (m, the same
     *  layout).
     *  - (Stage W5k: tolerant: 2.5 cells, two cells round, a cell whose neighbourhood holds both
     *    low ground and shallow water, a ring round every one chosen.)
     *  - A bake cell gets a tile at the coast (within 1.5 cells of it); on the sea side within
     *    Reach where some water nearby is shallower than 1.5 SeedDepth (the bake's heights are
     *    cell means: past that no fine cell is near the seed depth); on the land side within
     *    LandReach where some ground nearby lies below LowLand (a lowland's water).
     *  - Reach is held a little inside the shore field's own (past it the field's distance is
     *    capped, and every cell would seem within it).
     *  - The cells: the finest (largest R) the budget holds, cells of MinCellM to MaxCellM. Where
     *    even MaxCellM's don't fit, the tiles farthest from the coast are left out until they do
     *    (TrimmedTiles; the shore's waves fade out there: the field's cover).
     *  - Apron tiles for the bake's cells up to two past each face's edge, where the cell of the
     *    face holding them has a tile.
     *  - Stage W7a, a window (Settings.IsWindowed() and Window.bOn): only the candidates within
     *    SolveRadiusM of its centre, the nearest first; cells of WindowCellM (R the same for every
     *    window); at most WindowMaxCells of them (the farthest from the centre left out); the
     *    atlas's slices for the most tiles any window holds (CapacitySlices); the candidates left
     *    out marked Cut. */
    inline FPlan Plan(int N, double RadiusM, const uint32_t* Shore, double ShoreReachM, const float* Height, const FSettings& Settings,
        const FWindow& Window = FWindow(), const FCoastCells* Cached = nullptr)
    {
        FPlan P;
        P.N = N;
        P.RadiusM = RadiusM;
        P.Settings = Settings;
        const bool bWindow = Settings.IsWindowed() && Window.bOn;
        P.Window = Window;
        P.Window.bOn = bWindow;
        if (N <= 0 || !Shore || !Height || !(RadiusM > 0.0)) return P;
        const int S = VoxelOceanShoreCore::Side(N);
        P.BakeCellM = 0.5 * Pi * RadiusM / double(N);
        const double Cell = P.BakeCellM;
        const double ReachM = std::min(Settings.ReachM, ShoreReachM - 2.0 * Cell);
        if (!(ReachM > 0.0)) return P;
        P.Settings.ReachM = ReachM;
        // The interior candidates, nearest the coast first (stage W7b: as kept, or made now).
        FCoastCells Made;
        if (!Cached) Made = CoastCells(N, RadiusM, Shore, ShoreReachM, Height, Settings);
        const FCoastCells& Cells = Cached ? *Cached : Made;
        if (!Cells.bValid || Cells.Cells.empty()) return P;
        using FCandidate = FCoastCell;
        std::vector<FCandidate> Candidates = Cells.Cells;
        // Stage W7a: the window's candidates, by their distance from its centre (the rest are cut).
        std::vector<FCandidate> All;
        if (bWindow)
        {
            All = Candidates;
            Candidates.clear();
            const double Limit = Settings.SolveRadiusM() + Cell;
            for (const FCandidate& C : All)
            {
                const int Face = C.Cell / (N * N), J = (C.Cell / N) % N, I = C.Cell % N;
                double D[3];
                BakeCellDir(N, Face, I, J, D);
                const double Chord = std::sqrt((D[0] - Window.Centre[0]) * (D[0] - Window.Centre[0]) + (D[1] - Window.Centre[1]) * (D[1] - Window.Centre[1])
                    + (D[2] - Window.Centre[2]) * (D[2] - Window.Centre[2]));
                const double Along = RadiusM * 2.0 * std::asin(std::min(0.5 * Chord, 1.0));
                if (Along <= Limit) Candidates.push_back({ float(Along), C.Cell });
            }
        }
        std::stable_sort(Candidates.begin(), Candidates.end(), [](const FCandidate& A, const FCandidate& B) { return A.Distance < B.Distance; });
        std::vector<uint8_t> Selected(size_t(6) * size_t(N) * size_t(N), 0);
        // The tiles of the first Count candidates.
        // (Stage W7b: sparse, the selected cells and the index's texels set undone one by one: a
        // window's plan touches its own cells, not the planet's.)
        std::vector<int> Picked;
        std::vector<int> Indexed;
        P.TileOf.assign(size_t(6) * size_t(S) * size_t(S), -1);
        auto AddTile = [&](int Face, int I, int J)
            {
                const int X = IndexTexel(N, Face, I, J);
                P.TileOf[size_t(X)] = int(P.Tiles.size());
                Indexed.push_back(X);
                FTile T;
                T.Face = Face;
                T.I = I;
                T.J = J;
                P.Tiles.push_back(T);
            };
        auto Build = [&](size_t Count)
            {
                for (int c : Picked) Selected[size_t(c)] = 0;
                for (int X : Indexed) P.TileOf[size_t(X)] = -1;
                Picked.clear();
                Indexed.clear();
                for (size_t k = 0; k < Count; k++)
                {
                    Selected[size_t(Candidates[k].Cell)] = 1;
                    Picked.push_back(Candidates[k].Cell);
                }
                // (In the faces' order, as a scan would find them.)
                std::sort(Picked.begin(), Picked.end());
                P.Tiles.clear();
                for (int c : Picked) AddTile(c / (N * N), c % N, (c / N) % N);
                P.InteriorTiles = int(P.Tiles.size());
                // The aprons: the two cells past each face's edge (their rows and columns only).
                for (int Face = 0; Face < 6; Face++)
                    for (int J = -2; J < N + 2; J++)
                    {
                        const bool bRow = J < 0 || J >= N;
                        const int Side[4] = { -2, -1, N, N + 1 };
                        const int Count1 = bRow ? N + 4 : 4;
                        for (int k = 0; k < Count1; k++)
                        {
                            const int I = bRow ? k - 2 : Side[k];
                            const VoxelShader::FVoxelOceanShoreCell C = VoxelShader::VoxelOceanShoreCellOf(Face, I, J, N);
                            if (!Selected[(size_t(C.Face) * N + C.J) * N + C.I]) continue;
                            AddTile(Face, I, J);
                        }
                    }
            };
        // The atlas for R (its bytes; per row, per column and slices out).
        auto Layout = [&](int R, int& PerRow, int& PerColumn, int& Slices)
            {
                const int Stride = R + 2 * Apron;
                PerRow = std::max(S / Stride, 0);
                PerColumn = std::max(S / (Stride + 1), 0);
                const int PerSlice = PerRow * PerColumn;
                Slices = PerSlice > 0 ? int((P.Tiles.size() + size_t(PerSlice) - 1) / size_t(PerSlice)) : (1 << 30);
                return double(Slices) * double(S) * double(S) * 4.0;
            };
        auto Fits = [&](int R)
            {
                int PerRow, PerColumn, Slices;
                const double Bytes = Layout(R, PerRow, PerColumn, Slices);
                return PerRow > 0 && PerColumn > 0 && Bytes <= Settings.BudgetBytes && Slices <= MaxAtlasSlices
                    && double(P.InteriorTiles) * double(R) * double(R) <= Settings.MaxCells;
            };
        const int RMin = std::max(1, int(std::ceil(Cell / Settings.MaxCellM)));
        const int RMax = std::max(RMin, std::min(Settings.MaxR, int(std::floor(Cell / Settings.MinCellM))));
        // The candidates the window cut off (or that the budget left out), per index texel.
        auto MarkCut = [&]()
            {
                P.Cut.assign(size_t(6) * size_t(S) * size_t(S), 0);
                for (const FCandidate& C : All)
                    if (!Selected[size_t(C.Cell)])
                    {
                        const int Face = C.Cell / (N * N), J = (C.Cell / N) % N, I = C.Cell % N;
                        P.Cut[size_t(IndexTexel(N, Face, I, J))] = 1;
                    }
            };
        if (bWindow)
        {
            // Stage W7a: cells of WindowCellM, the same for every window (the texture keeps its
            // shape and the overlap its values); the most tiles the cells allow.
            const int RW = std::clamp(int(std::lround(Cell / std::max(Settings.WindowCellM, 1.0))), RMin, RMax);
            int PerRow = 0, PerColumn = 0, Unused = 0;
            Layout(RW, PerRow, PerColumn, Unused);
            const size_t PerSlice = size_t(std::max(PerRow, 0)) * size_t(std::max(PerColumn, 0));
            if (PerSlice == 0 || Candidates.empty()) { P.Tiles.clear(); P.InteriorTiles = 0; return P; }
            const size_t MaxInterior = std::max<size_t>(1, size_t(Settings.WindowMaxCells / (double(RW) * double(RW))));
            // (The aprons past face edges: at most a quarter more, near a cube corner.)
            const size_t Capacity = std::min(MaxInterior + MaxInterior / 4 + 64, size_t(MaxAtlasSlices) * PerSlice);
            P.CapacitySlices = int((Capacity + PerSlice - 1) / PerSlice);
            size_t Count = std::min(Candidates.size(), MaxInterior);
            Build(Count);
            while (P.Tiles.size() > Capacity && Count > 1)
            {
                Count = Count - std::max<size_t>(1, Count / 20);
                Build(Count);
            }
            P.TrimmedTiles = int(Candidates.size() - Count);
            MarkCut();
            P.R = RW;
            Layout(P.R, P.TilesPerRow, P.TilesPerColumn, P.AtlasSlices);
            // (The texture's: the slices any window may fill.)
            P.AtlasBytes = double(P.CapacitySlices) * double(S) * double(S) * 4.0;
        }
        else
        {
            Build(Candidates.size());
            if (!Fits(RMin))
            {
                // The most tiles that fit at the coarsest cells, nearest the coast first.
                size_t Lo = 0, Hi = Candidates.size();
                while (Hi - Lo > 1)
                {
                    const size_t Mid = (Lo + Hi) / 2;
                    Build(Mid);
                    if (Fits(RMin)) Lo = Mid; else Hi = Mid;
                }
                P.TrimmedTiles = int(Candidates.size() - Lo);
                Build(Lo);
                if (Lo == 0 || !Fits(RMin)) { P.Tiles.clear(); P.InteriorTiles = 0; return P; }
                // The field reaches to the farthest kept (the shaders fade the shore's waves by it).
                // A kept cell's edge toward a trimmed one can lie up to a cell nearer the coast than
                // the last kept centre: the reach a cell and a half inside it, so the cells along the
                // cut are Reach sources (past the cap: untrusted) and the cover fades before the edge.
                P.Settings.ReachM = std::min(P.Settings.ReachM, std::max(double(Candidates[Lo - 1].Distance) - 1.5 * Cell, 0.0));
            }
            int Chosen = RMin;
            for (int R = RMin + 1; R <= RMax && Fits(R); R++) Chosen = R;
            P.R = Chosen;
            P.AtlasBytes = Layout(P.R, P.TilesPerRow, P.TilesPerColumn, P.AtlasSlices);
        }
        P.CellM = Cell / double(P.R);
        // The fine cells' tangents, three bake cells past each face edge.
        P.TanMargin = 3 * P.R + 2;
        const int FN = P.FineN();
        P.TanTable.resize(size_t(FN + 2 * P.TanMargin));
        for (int k = 0; k < int(P.TanTable.size()); k++)
        {
            const int F = k - P.TanMargin;
            P.TanTable[size_t(k)] = std::tan(0.25 * Pi * (-1.0 + (2.0 * F + 1.0) / double(FN)));
        }
        P.bValid = true;
        return P;
    }

    /** The interior fine cells' centre directions in the order the depths come (tile by tile,
     *  rows of R): for the CPU's own sampling (the GPU's kernel computes the same). */
    inline void InteriorCellDir(const FPlan& P, size_t Cell, double Out[3])
    {
        const int RR = P.CellsPerTile();
        const int Tile = int(Cell / size_t(RR));
        const int Within = int(Cell - size_t(Tile) * size_t(RR));
        const FTile& T = P.Tiles[size_t(Tile)];
        FineDir(P, T.Face, T.I * P.R + Within % P.R, T.J * P.R + Within / P.R, Out);
    }

    // ---------------------------------------------------------------------------------------
    // The grid
    // ---------------------------------------------------------------------------------------

    /** Fine cell (FI, FJ) of a face (either may be past its edge) as an interior cell: its
     *  number, or -1 (no tile there). */
    inline int64_t InteriorCellAt(const FPlan& P, int Face, int FI, int FJ)
    {
        const int FN = P.FineN();
        if (FI < 0 || FJ < 0 || FI >= FN || FJ >= FN)
        {
            double D[3];
            FineDir(P, Face, FI, FJ, D);
            int F2;
            double S, T;
            Locate(D, F2, S, T);
            Face = F2;
            FI = std::clamp(int(std::floor((S + 1.0) * 0.5 * double(FN))), 0, FN - 1);
            FJ = std::clamp(int(std::floor((T + 1.0) * 0.5 * double(FN))), 0, FN - 1);
        }
        const int CI = FI / P.R, CJ = FJ / P.R;
        const int Tile = P.TileOf[size_t(IndexTexel(P.N, Face, CI, CJ))];
        if (Tile < 0 || Tile >= P.InteriorTiles) return -1;
        return int64_t(Tile) * P.CellsPerTile() + int64_t(FJ - CJ * P.R) * P.R + (FI - CI * P.R);
    }

    struct FCellId
    {
        int Face;
        int FI;
        int FJ;
    };

    inline FCellId CellIdOf(const FPlan& P, int64_t Cell)
    {
        const int RR = P.CellsPerTile();
        const int Tile = int(Cell / RR);
        const int Within = int(Cell - int64_t(Tile) * RR);
        const FTile& T = P.Tiles[size_t(Tile)];
        FCellId C;
        C.Face = T.Face;
        C.FI = T.I * P.R + Within % P.R;
        C.FJ = T.J * P.R + Within / P.R;
        return C;
    }

    /** The 4 neighbours (+I, +J, -I, -J) of an interior cell (-1: none). */
    inline void Neighbours(const FPlan& P, int64_t Cell, int64_t Out[4])
    {
        const FCellId C = CellIdOf(P, Cell);
        const int RR = P.CellsPerTile();
        const int Within = int(Cell % RR);
        const int LI = Within % P.R, LJ = Within / P.R;
        // Inside the tile: plain steps.
        Out[0] = LI + 1 < P.R ? Cell + 1 : InteriorCellAt(P, C.Face, C.FI + 1, C.FJ);
        Out[1] = LJ + 1 < P.R ? Cell + P.R : InteriorCellAt(P, C.Face, C.FI, C.FJ + 1);
        Out[2] = LI > 0 ? Cell - 1 : InteriorCellAt(P, C.Face, C.FI - 1, C.FJ);
        Out[3] = LJ > 0 ? Cell - P.R : InteriorCellAt(P, C.Face, C.FI, C.FJ - 1);
    }

    inline void CellPos(const FPlan& P, int64_t Cell, double Out[3])
    {
        const FCellId C = CellIdOf(P, Cell);
        FineDir(P, C.Face, C.FI, C.FJ, Out);
        for (int k = 0; k < 3; k++) Out[k] *= P.RadiusM;
    }

    // ---------------------------------------------------------------------------------------
    // The solve
    // ---------------------------------------------------------------------------------------

    struct FSolved
    {
        std::vector<float> A;            // interior cells
        std::vector<float> B;
        std::vector<uint8_t> Trust;
        size_t Seeds = 0;
        size_t Reached = 0;
        size_t Wet = 0;
        double Seconds = 0.0;
    };

    /** An indexed binary min-heap of cells by key. */
    struct FHeap
    {
        std::vector<int64_t> Items;
        float* Keys = nullptr;           // per cell (the caller's: the values themselves)
        std::vector<int32_t> Where;      // per cell: its place, or -1

        void Init(size_t Cells, float* InKeys)
        {
            Items.clear();
            Keys = InKeys;
            Where.assign(Cells, -1);
        }
        bool Empty() const { return Items.empty(); }
        void Swap(size_t a, size_t b)
        {
            std::swap(Items[a], Items[b]);
            Where[size_t(Items[a])] = int32_t(a);
            Where[size_t(Items[b])] = int32_t(b);
        }
        void Up(size_t i)
        {
            while (i > 0)
            {
                const size_t p = (i - 1) / 2;
                if (!(Keys[size_t(Items[i])] < Keys[size_t(Items[p])])) break;
                Swap(i, p);
                i = p;
            }
        }
        void Down(size_t i)
        {
            for (;;)
            {
                const size_t l = 2 * i + 1, r = l + 1;
                size_t m = i;
                if (l < Items.size() && Keys[size_t(Items[l])] < Keys[size_t(Items[m])]) m = l;
                if (r < Items.size() && Keys[size_t(Items[r])] < Keys[size_t(Items[m])]) m = r;
                if (m == i) break;
                Swap(i, m);
                i = m;
            }
        }
        /** Place a cell by its key (just lowered by the caller). */
        void Push(int64_t Cell)
        {
            const int32_t W = Where[size_t(Cell)];
            if (W >= 0)
            {
                Up(size_t(W));
                return;
            }
            Items.push_back(Cell);
            Where[size_t(Cell)] = int32_t(Items.size() - 1);
            Up(Items.size() - 1);
        }
        int64_t Pop()
        {
            const int64_t Top = Items[0];
            Swap(0, Items.size() - 1);
            Items.pop_back();
            Where[size_t(Top)] = -1;
            if (!Items.empty()) Down(0);
            return Top;
        }
    };

    struct FCandidate
    {
        float A;
        float B;
    };

    /** The eikonal's update of point C (slowness NC, B's rate RC) from the segment between known
     *  points PA (values AA, BA, slowness NA, rate RA) and PB (AB, BB, NB, RB): the smallest A over
     *  the segment's points X of A(X) + n |C - X|, A linear along it, n the mean of C's slowness
     *  and X's (the trapezoid rule: second order where the depth changes across a cell); none (A
     *  infinite) where the minimum is at an end (the edge updates cover those). */
    inline FCandidate TriangleUpdate(const double PC[3], double NC, double RC, const double PA[3], float AA, float BA, double NA, double RA,
        const double PB[3], float AB, float BB, double NB, double RB)
    {
        FCandidate Out;
        Out.A = std::numeric_limits<float>::infinity();
        Out.B = 0.0f;
        double E[3], W[3];
        for (int k = 0; k < 3; k++) { E[k] = PB[k] - PA[k]; W[k] = PC[k] - PA[k]; }
        const double EL2 = E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
        if (!(EL2 > 0.0)) return Out;
        const double EL = std::sqrt(EL2);
        const double Alpha = (W[0] * E[0] + W[1] * E[1] + W[2] * E[2]) / EL;
        const double W2 = W[0] * W[0] + W[1] * W[1] + W[2] * W[2];
        const double Beta = std::sqrt(std::max(W2 - Alpha * Alpha, 0.0));
        if (!(Beta > 0.0)) return Out;
        // The foot for a slowness, then again for the mean at that foot.
        double Lambda = 0.5, N = NC;
        for (int Pass = 0; Pass < 2; Pass++)
        {
            const double Q = (double(AB) - double(AA)) / N;
            if (std::fabs(Q) >= EL) return Out;
            const double X = (Q >= 0.0 ? 1.0 : -1.0) * std::fabs(Q) * Beta / std::sqrt(EL2 - Q * Q);
            Lambda = (Alpha - X) / EL;
            if (!(Lambda > 0.0 && Lambda < 1.0)) return Out;
            N = 0.5 * (NC + NA + Lambda * (NB - NA));
        }
        double Foot[3];
        for (int k = 0; k < 3; k++) Foot[k] = W[k] - Lambda * E[k];
        const double L = std::sqrt(Foot[0] * Foot[0] + Foot[1] * Foot[1] + Foot[2] * Foot[2]);
        const double R = 0.5 * (RC + RA + Lambda * (RB - RA));
        Out.A = float(double(AA) + Lambda * (double(AB) - double(AA)) + N * L);
        Out.B = float(double(BA) + Lambda * (double(BB) - double(BA)) + R * L);
        return Out;
    }

    /** The solve from the interior cells' depths (m; InteriorCells of them, in the plan's order).
     *  Shore and its reach: the shore field (the outer edge past Reach). Land is crossed at the
     *  floor's slowness (so the swash finds a phase up the beach that continues the shoreline's,
     *  and the interpolation near the shoreline sees no corner). */
    inline FSolved Solve(const FPlan& P, const float* DepthM, const uint32_t* Shore, double ShoreReachM)
    {
        const auto Start = std::chrono::steady_clock::now();
        FSolved Out;
        const size_t Cells = P.InteriorCells();
        Out.A.assign(Cells, std::numeric_limits<float>::infinity());
        Out.B.assign(Cells, 0.0f);
        Out.Trust.assign(Cells, 0);
        if (!P.bValid || Cells == 0) return Out;
        const FSettings& S = P.Settings;
        const int RR = P.CellsPerTile();
        // 0 far, 1 trial, 2 known.
        std::vector<uint8_t> State(Cells, 0);
        auto Wet = [&](int64_t c) { return DepthM[size_t(c)] > 0.0f; };
        auto Held = [&](int64_t c) { return double(VoxelShader::VoxelOceanCoastHeldDepth(DepthM[size_t(c)])); };
        auto Slow = [&](int64_t c) { return 1.0 / std::sqrt(Held(c)); };
        auto Rate = [&](int64_t c) { return std::sqrt(Held(c)); };
        auto Length = [](const double A[3], const double B[3]) { return std::sqrt((A[0] - B[0]) * (A[0] - B[0]) + (A[1] - B[1]) * (A[1] - B[1]) + (A[2] - B[2]) * (A[2] - B[2])); };
        // The shore field's distance at a bake cell, and finely at a direction (the outer edge
        // along Reach).
        auto BakeDistance = [&](int Face, int I, int J)
            {
                return double(VoxelShader::VoxelOceanShoreUnpack(Shore[size_t(IndexTexel(P.N, Face, I, J))], float(ShoreReachM)).Distance);
            };
        VoxelShader::FVoxelOceanShoreReader Reader;
        Reader.Texels = Shore;
        Reader.Side = VoxelOceanShoreCore::Side(P.N);
        FHeap Heap;
        Heap.Init(Cells, Out.A.data());
        // Sources past the cap (well inside the deep water) are flat there: untrusted, so the
        // cover fades from the outer edge.
        std::vector<uint8_t> DeepSource(Cells, 0);
        static const int Step[4][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
        const double Cap = 1.5 * P.CellM;
        // The sources, each at its distance past the outer edge (negative values), so the edge's
        // zero lies where the depth (or the distance) crosses it between cells rather than on the
        // cells' staircase.
        for (int Tile = 0; Tile < P.InteriorTiles; Tile++)
        {
            const FTile& T = P.Tiles[size_t(Tile)];
            const double TileDistance = BakeDistance(T.Face, T.I, T.J);
            const bool bNearReach = TileDistance > S.ReachM - 3.0 * P.BakeCellM;
            for (int k = 0; k < RR; k++)
            {
                const int64_t c = int64_t(Tile) * RR + k;
                if (!Wet(c)) continue;
                Out.Wet++;
                int64_t Nb[4];
                Neighbours(P, c, Nb);
                double PC[3];
                CellPos(P, c, PC);
                const double Dc = double(DepthM[size_t(c)]);
                double Past = -1.0;      // metres past the outer edge, or none
                if (Dc >= S.SeedDepthM)
                {
                    // The depth's gradient by central differences (one-sided at the tiles' edge).
                    double G2 = 0.0;
                    for (int Axis = 0; Axis < 2; Axis++)
                    {
                        const int64_t Fw = Nb[Axis], Bw = Nb[Axis + 2];
                        double Num = 0.0, Den = 0.0;
                        double PF[3], PB[3];
                        if (Fw >= 0 && Bw >= 0) { CellPos(P, Fw, PF); CellPos(P, Bw, PB); Num = double(DepthM[size_t(Fw)]) - double(DepthM[size_t(Bw)]); Den = Length(PF, PB); }
                        else if (Fw >= 0) { CellPos(P, Fw, PF); Num = double(DepthM[size_t(Fw)]) - Dc; Den = Length(PF, PC); }
                        else if (Bw >= 0) { CellPos(P, Bw, PB); Num = Dc - double(DepthM[size_t(Bw)]); Den = Length(PC, PB); }
                        if (Den > 0.0) G2 += (Num / Den) * (Num / Den);
                    }
                    const double G = std::sqrt(G2);
                    Past = G > 1.0e-9 ? std::min((Dc - S.SeedDepthM) / G, Cap) : Cap;
                }
                if (Past < 0.0 && bNearReach)
                {
                    double D[3] = { PC[0], PC[1], PC[2] };
                    Normalize(D);
                    const float Fine = VoxelShader::VoxelOceanShoreFieldAt(Reader, VoxelShader::float3(float(D[0]), float(D[1]), float(D[2])), P.N, float(ShoreReachM)).Distance;
                    if (double(Fine) >= S.ReachM) Past = std::min(double(Fine) - S.ReachM, Cap);
                }
                bool bEdge = false;
                if (Past < 0.0)
                {
                    // On the tiles' edge toward the open sea: a neighbour without a tile whose
                    // bake cell lies farther out (untrusted: the cover fades toward the edge).
                    const FCellId Id = CellIdOf(P, c);
                    for (int e = 0; e < 4 && Past < 0.0; e++)
                    {
                        if (Nb[e] >= 0) continue;
                        const int FI = Id.FI + Step[e][0], FJ = Id.FJ + Step[e][1];
                        const VoxelShader::FVoxelOceanShoreCell BC = VoxelShader::VoxelOceanShoreCellOf(Id.Face, BakeOf(FI, P.R), BakeOf(FJ, P.R), P.N);
                        // (Stage W7a: not across the window's cut, which is closed.)
                        if (IsCut(P, BC.Face, BC.I, BC.J)) continue;
                        if (BakeDistance(BC.Face, BC.I, BC.J) > TileDistance) { Past = 0.0; bEdge = true; }
                    }
                }
                if (Past >= 0.0)
                {
                    if (Past >= Cap || bEdge) DeepSource[size_t(c)] = 1;
                    Out.A[size_t(c)] = float(-Past * Slow(c));
                    Out.B[size_t(c)] = float(-Past * Rate(c));
                    State[size_t(c)] = 2;
                    Out.Seeds++;
                }
            }
        }
        // The update of a cell from its known neighbours.
        auto Update = [&](int64_t c)
            {
                int64_t Nb[4];
                Neighbours(P, c, Nb);
                double PC[3], PN[4][3], NN[4], RN[4];
                CellPos(P, c, PC);
                bool bKnown[4];
                for (int e = 0; e < 4; e++)
                {
                    bKnown[e] = Nb[e] >= 0 && State[size_t(Nb[e])] == 2;
                    if (!bKnown[e]) continue;
                    CellPos(P, Nb[e], PN[e]);
                    NN[e] = Slow(Nb[e]);
                    RN[e] = Rate(Nb[e]);
                }
                const double NC = Slow(c), RC = Rate(c);
                FCandidate Best;
                Best.A = std::numeric_limits<float>::infinity();
                Best.B = 0.0f;
                for (int e = 0; e < 4; e++)
                {
                    if (!bKnown[e]) continue;
                    const size_t a = size_t(Nb[e]);
                    const double L = Length(PC, PN[e]);
                    const float A = float(double(Out.A[a]) + 0.5 * (NC + NN[e]) * L);
                    if (A < Best.A) { Best.A = A; Best.B = float(double(Out.B[a]) + 0.5 * (RC + RN[e]) * L); }
                    const int f = (e + 1) & 3;
                    if (!bKnown[f]) continue;
                    const size_t b = size_t(Nb[f]);
                    const FCandidate T = TriangleUpdate(PC, NC, RC, PN[e], Out.A[a], Out.B[a], NN[e], RN[e], PN[f], Out.A[b], Out.B[b], NN[f], RN[f]);
                    if (T.A < Best.A) Best = T;
                }
                return Best;
            };
        auto Touch = [&](int64_t c)
            {
                int64_t Nb[4];
                Neighbours(P, c, Nb);
                for (int e = 0; e < 4; e++)
                {
                    const int64_t n = Nb[e];
                    if (n < 0 || State[size_t(n)] == 2) continue;
                    const FCandidate C = Update(n);
                    if (C.A < Out.A[size_t(n)])
                    {
                        Out.A[size_t(n)] = C.A;
                        Out.B[size_t(n)] = C.B;
                        State[size_t(n)] = 1;
                        Heap.Push(n);
                    }
                }
            };
        for (size_t c = 0; c < Cells; c++)
            if (State[c] == 2) Touch(int64_t(c));
        while (!Heap.Empty())
        {
            const int64_t c = Heap.Pop();
            if (State[size_t(c)] == 2) continue;
            State[size_t(c)] = 2;
            Touch(c);
        }
        for (size_t c = 0; c < Cells; c++)
        {
            if (State[c] == 2)
            {
                Out.Trust[c] = DeepSource[c] ? 0 : 1;
                if (DepthM[c] > 0.0f) Out.Reached++;
            }
            else
            {
                Out.A[c] = 0.0f;
                Out.B[c] = 0.0f;
            }
        }
        Out.Seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - Start).count();
        return Out;
    }

    // ---------------------------------------------------------------------------------------
    // The texels
    // ---------------------------------------------------------------------------------------

    struct FValue
    {
        float A = 0.0f;
        float B = 0.0f;
        float Trust = 0.0f;
        bool bSet = false;
    };

    /** The solved values at a direction, bilinear between the interior cells of the face holding
     *  it (for the aprons past face edges). */
    inline FValue ResampleAt(const FPlan& P, const FSolved& V, const double D[3])
    {
        FValue Out;
        int Face;
        double S, T;
        Locate(D, Face, S, T);
        const int FN = P.FineN();
        const double U = (S + 1.0) * 0.5 * double(FN) - 0.5, W = (T + 1.0) * 0.5 * double(FN) - 0.5;
        const int I0 = int(std::floor(U)), J0 = int(std::floor(W));
        const double FU = U - I0, FW = W - J0;
        double Sum = 0.0, A = 0.0, B = 0.0, Tr = 0.0;
        for (int dj = 0; dj < 2; dj++)
            for (int di = 0; di < 2; di++)
            {
                const double Wt = (di ? FU : 1.0 - FU) * (dj ? FW : 1.0 - FW);
                if (Wt <= 0.0) continue;
                const int64_t c = InteriorCellAt(P, Face, I0 + di, J0 + dj);
                if (c < 0) continue;
                Sum += Wt;
                A += Wt * double(V.A[size_t(c)]);
                B += Wt * double(V.B[size_t(c)]);
                Tr += Wt * double(V.Trust[size_t(c)]);
            }
        if (Sum <= 1.0e-9) return Out;
        Out.A = float(A / Sum);
        Out.B = float(B / Sum);
        Out.Trust = float(Tr / Sum) > 0.5f ? 1.0f : 0.0f;
        Out.bSet = true;
        return Out;
    }

    /** The coast's texels (the index's 6 slices, then the atlas's; Side^2 each). */
    inline std::vector<uint32_t> Pack(const FPlan& P, const FSolved& V)
    {
        const int S = VoxelOceanShoreCore::Side(P.N);
        std::vector<uint32_t> Out;
        if (!P.bValid) return Out;
        Out.assign(size_t(P.Slices()) * size_t(S) * size_t(S), 0u);
        auto At = [&](int Slice, int X, int Y) -> uint32_t& { return Out[(size_t(Slice) * size_t(S) + size_t(Y)) * size_t(S) + size_t(X)]; };
        // The index: the tile's number plus one as a float's value.
        for (size_t t = 0; t < P.Tiles.size(); t++)
        {
            const FTile& T = P.Tiles[t];
            const float Value = float(t + 1);
            uint32_t Bits;
            std::memcpy(&Bits, &Value, 4);
            At(VOXEL_OCEAN_COAST_INDEX_SLICE - 6 + T.Face, T.I + Pad, T.J + Pad) = Bits;
        }
        VoxelShader::FVoxelOceanCoastLayout L = VoxelShader::VoxelOceanCoastLayoutNone();
        L.Valid = 1.0f;
        L.N = P.N;
        L.R = P.R;
        L.TilesPerRow = P.TilesPerRow;
        L.TilesPerColumn = P.TilesPerColumn;
        const int Stride = P.Stride();
        const int FN = P.FineN();
        std::vector<FValue> Block(size_t(Stride) * size_t(Stride));
        for (size_t t = 0; t < P.Tiles.size(); t++)
        {
            const FTile& T = P.Tiles[t];
            // The block's values: the cells themselves within the face, resampled past its edges.
            for (int y = 0; y < Stride; y++)
                for (int x = 0; x < Stride; x++)
                {
                    FValue& B = Block[size_t(y) * Stride + x];
                    B = FValue();
                    const int FI = T.I * P.R + x - Apron, FJ = T.J * P.R + y - Apron;
                    if (FI >= 0 && FJ >= 0 && FI < FN && FJ < FN)
                    {
                        const int64_t c = InteriorCellAt(P, T.Face, FI, FJ);
                        if (c < 0) continue;
                        B.A = V.A[size_t(c)];
                        B.B = V.B[size_t(c)];
                        B.Trust = float(V.Trust[size_t(c)]);
                        B.bSet = true;
                    }
                    else
                    {
                        double D[3];
                        FineDir(P, T.Face, FI, FJ, D);
                        B = ResampleAt(P, V, D);
                    }
                }
            // Stage W5k: the cells next to a gap (toward a bake cell without a tile), the block's
            // set cells as they were before the gaps grow.
            std::vector<uint8_t> WasSet(Block.size(), 0);
            for (size_t b = 0; b < Block.size(); b++) WasSet[b] = Block[b].bSet ? 1 : 0;
            // Gaps (toward cells without a tile): grown from their set neighbours.
            for (int Pass = 0; Pass < 2 * Stride; Pass++)
            {
                bool bAny = false;
                std::vector<FValue> Next = Block;
                for (int y = 0; y < Stride; y++)
                    for (int x = 0; x < Stride; x++)
                    {
                        if (Block[size_t(y) * Stride + x].bSet) continue;
                        double Sum = 0.0, A = 0.0, B = 0.0;
                        static const int Step[4][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
                        for (int e = 0; e < 4; e++)
                        {
                            const int X1 = x + Step[e][0], Y1 = y + Step[e][1];
                            if (X1 < 0 || Y1 < 0 || X1 >= Stride || Y1 >= Stride) continue;
                            const FValue& Nb = Block[size_t(Y1) * Stride + X1];
                            if (!Nb.bSet) continue;
                            // Continued linearly from the neighbour's neighbour where it's set.
                            const int X2 = X1 + Step[e][0], Y2 = Y1 + Step[e][1];
                            double VA = Nb.A, VB = Nb.B;
                            if (X2 >= 0 && Y2 >= 0 && X2 < Stride && Y2 < Stride && Block[size_t(Y2) * Stride + X2].bSet)
                            {
                                VA = 2.0 * Nb.A - Block[size_t(Y2) * Stride + X2].A;
                                VB = 2.0 * Nb.B - Block[size_t(Y2) * Stride + X2].B;
                            }
                            Sum += 1.0;
                            A += VA;
                            B += VB;
                        }
                        if (Sum <= 0.0) continue;
                        FValue& G = Next[size_t(y) * Stride + x];
                        G.A = float(A / Sum);
                        G.B = float(B / Sum);
                        G.Trust = 0.0f;
                        G.bSet = true;
                        bAny = true;
                    }
                Block.swap(Next);
                if (!bAny) break;
            }
            // Stage W5k: THE COVER ENDS AT NOTHING AT A TILE'S EDGE. A point in a bake cell without a
            // tile has no field (cover 0), but the cover a tile's samples interpolate (Catmull-Rom over
            // its cells and its grown apron, untrusted) still stood near a half at its edge: the
            // shore's trains (their waves and foam) stopped there in a straight line, a cliff in the
            // mesh. The tile's own cells within EdgeCells of such a gap are untrusted too, so the
            // cover falls to 0 inside the tile, before its edge.
            {
                constexpr int EdgeCells = 2;
                std::vector<uint8_t> NearGap(Block.size(), 0);
                for (int y = 0; y < Stride; y++)
                    for (int x = 0; x < Stride; x++)
                    {
                        if (WasSet[size_t(y) * Stride + x]) continue;
                        for (int dy = -EdgeCells; dy <= EdgeCells; dy++)
                            for (int dx = -EdgeCells; dx <= EdgeCells; dx++)
                            {
                                const int X1 = x + dx, Y1 = y + dy;
                                if (X1 >= 0 && Y1 >= 0 && X1 < Stride && Y1 < Stride) NearGap[size_t(Y1) * Stride + X1] = 1;
                            }
                    }
                for (size_t b = 0; b < Block.size(); b++)
                    if (NearGap[b]) Block[b].Trust = 0.0f;
            }
            // The tile's ranges, then its codes.
            float MinA = std::numeric_limits<float>::infinity(), MaxA = -MinA, MinB = MinA, MaxB = -MinA;
            for (const FValue& B : Block)
            {
                if (!B.bSet) continue;
                MinA = std::min(MinA, B.A); MaxA = std::max(MaxA, B.A);
                MinB = std::min(MinB, B.B); MaxB = std::max(MaxB, B.B);
            }
            if (!(MinA <= MaxA)) { MinA = MaxA = 0.0f; MinB = MaxB = 0.0f; }
            const float SpanA = std::max(MaxA - MinA, 1.0e-3f), SpanB = std::max(MaxB - MinB, 1.0e-3f);
            const VoxelShader::int3 O = VoxelShader::VoxelOceanCoastTileOrigin(L, int(t));
            const int Slice = O.z - 6;
            for (int y = 0; y < Stride; y++)
                for (int x = 0; x < Stride; x++)
                {
                    const FValue& B = Block[size_t(y) * Stride + x];
                    const float A01 = B.bSet ? (B.A - MinA) / SpanA : 0.0f;
                    const float B01 = B.bSet ? (B.B - MinB) / SpanB : 0.0f;
                    At(Slice, O.x + x, O.y + y) = VoxelShader::VoxelOceanCoastPack(A01, B01, B.bSet && B.Trust > 0.5f);
                }
            const float Header[4] = { MinA, SpanA, MinB, SpanB };
            for (int k = 0; k < 4; k++)
            {
                uint32_t Bits;
                std::memcpy(&Bits, &Header[k], 4);
                At(Slice, O.x + k, O.y + Stride) = Bits;
            }
        }
        return Out;
    }

    /** Stage W7a: per face, the index texels the plan's tiles take (x0, y0, x1, y1 inclusive, the
     *  index's own coordinates, pad included; x0 > x1: none): the texture's index is written again
     *  there (and where the last window's were) when a window replaces another. */
    inline void IndexBoxes(const FPlan& P, int Out[6][4])
    {
        for (int f = 0; f < 6; f++) { Out[f][0] = Out[f][1] = 1 << 30; Out[f][2] = Out[f][3] = -1; }
        for (const FTile& T : P.Tiles)
        {
            int* B = Out[T.Face];
            B[0] = std::min(B[0], T.I + Pad);
            B[1] = std::min(B[1], T.J + Pad);
            B[2] = std::max(B[2], T.I + Pad);
            B[3] = std::max(B[3], T.J + Pad);
        }
    }

    /** The layout as the shaders take it (rows 19 and 20 of the chart table's constants; row 21
     *  holds the radius). */
    inline void LayoutRows(const FPlan& P, float Row0[4], float Row1[4])
    {
        Row0[0] = P.bValid ? 1.0f : 0.0f;
        Row0[1] = float(P.N);
        Row0[2] = float(P.R);
        Row0[3] = float(P.TilesPerRow);
        Row1[0] = float(P.TilesPerColumn);
        Row1[1] = float(P.CellM);
        Row1[2] = float(P.Settings.SeedDepthM);
        Row1[3] = float(P.Settings.ReachM);
    }

    /** The field at a direction (planet frame, any length), from the shore field's texels and the
     *  coast's: VoxelOceanCoastNear's blend across face edges, the fine coordinates in double
     *  (stage O9i: exact, as the shaders' near the camera; no texel-space expansion). */
    inline VoxelShader::FVoxelOceanCoastSample Sample(const uint32_t* ShoreTexels, const uint32_t* CoastTexels, int CoastSlices, int N, const float Row0[4],
        const float Row1[4], double RadiusM, double X, double Y, double Z)
    {
        using namespace VoxelShader;
        const double L = std::sqrt(X * X + Y * Y + Z * Z);
        if (!ShoreTexels || !CoastTexels || N <= 0 || !(L > 0.0)) return VoxelOceanCoastSampleNone();
        FVoxelOceanShoreReader Reader;
        Reader.Texels = ShoreTexels;
        Reader.Side = VoxelOceanShoreCore::Side(N);
        Reader.Coast = CoastTexels;
        Reader.CoastSlices = CoastSlices;
        const FVoxelOceanCoastLayout Layout = VoxelOceanCoastLayoutFrom(float4(Row0[0], Row0[1], Row0[2], Row0[3]),
            float4(Row1[0], Row1[1], Row1[2], Row1[3]), float(RadiusM));
        FVoxelOceanCoastSample Sum = VoxelOceanCoastSampleNone();
        if (Layout.Valid < 0.5f || Layout.N <= 0) return Sum;
        const double P[3] = { X / L, Y / L, Z / L };
        const float3 Dir = float3(float(P[0]), float(P[1]), float(P[2]));
        const float Band = 2.0f * VOXEL_OCEAN_SHORE_BLEND_CELLS / float(Layout.N);
        const double Fine = 0.5 * double(Layout.N) * double(Layout.R);
        float WeightAll = 0.0f;
        float WeightValid = 0.0f;
        for (int Axis = 0; Axis < 3; Axis++)
        {
            const double Along = std::fabs(P[Axis]);
            if (Along < 1.0e-3) continue;
            const unsigned Face = unsigned(Axis * 2 + (P[Axis] < 0.0 ? 1 : 0));
            const double S = std::atan(P[VoxelCubeUAxis(Face)] / Along) * (4.0 / 3.14159265358979323846);
            const double T = std::atan(P[VoxelCubeVAxis(Face)] / Along) * (4.0 / 3.14159265358979323846);
            const float Inside = float(1.0 - std::max(std::fabs(S), std::fabs(T)));
            const float W = smoothstep(-Band, Band, Inside);
            if (W <= 0.0f) continue;
            WeightAll += W;
            const double U = (S + 1.0) * Fine - 0.5;
            const double V = (T + 1.0) * Fine - 0.5;
            const FVoxelOceanCoastCoord CU = VoxelOceanCoastCoordOf(int(std::floor(U)), float(U - std::floor(U)));
            const FVoxelOceanCoastCoord CV = VoxelOceanCoastCoordOf(int(std::floor(V)), float(V - std::floor(V)));
            const FVoxelOceanCoastFace F = VoxelOceanCoastFaceSample(Reader, Layout, int(Face), CU, CV);
            if (F.Valid < 0.5f) continue;
            WeightValid += W;
            const float3 GU = VoxelOceanCoastAxisGradient(Dir, int(Face), VoxelCubeUAxis(Face), Layout);
            const float3 GV = VoxelOceanCoastAxisGradient(Dir, int(Face), VoxelCubeVAxis(Face), Layout);
            Sum.A += W * F.A;
            Sum.B += W * F.B;
            Sum.GradA = Sum.GradA + (GU * F.AU + GV * F.AV) * W;
            Sum.GradB = Sum.GradB + (GU * F.BU + GV * F.BV) * W;
            Sum.Cover += W * F.Trust;
        }
        if (WeightValid <= 0.0f) return VoxelOceanCoastSampleNone();
        const float Inv = 1.0f / WeightValid;
        Sum.A *= Inv;
        Sum.B *= Inv;
        Sum.GradA = Sum.GradA * Inv;
        Sum.GradB = Sum.GradB * Inv;
        Sum.Cover = saturate(Sum.Cover * Inv) * (WeightValid / std::max(WeightAll, 1.0e-6f));
        return Sum;
    }
}
