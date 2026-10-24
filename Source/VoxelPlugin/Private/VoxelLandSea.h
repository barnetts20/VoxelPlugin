// VoxelLandSea.h - The land bake's sea-connected cells (FVoxelLandBake::BuildSeaConnected;
// Design/OceanWaves.md section 5, stage O8): the cells below sea level that belong to the ocean or
// a sea joined to it, not to an inland basin. Plain C++ (no engine types), so the ocean harness
// checks the same code (Tools/OceanHarness/shore_check.cpp).
//
// A below-sea region is a set of interior cells under sea level, 4-connected, across face edges
// too (the equal-angle grids of two faces meet cell for cell: the apron cell past an edge lies on
// the next face's edge cell). A region is sea when it holds at least MinShare of all the
// below-sea cells; the largest always is. An apron cell takes the bit of the interior cell holding
// its centre, so a face's own reads (the shore field's seeds) see its neighbours' flags.
//
// Stage W5l, THE BRIDGE: a below-sea region within BridgeCells steps of the sea (4-connected steps
// through any cells, land included) is sea too. The bake's heights are one sample a cell, so a bay
// or a fjord whose mouth is narrower than a cell (or whose cells there sample the walls) came out cut
// off from the sea: an "inland basin", which the ocean drains (its surface sunk by its depth over
// the basins' margin: a wall of water across the bay) and culls past it. A true inland basin lies
// farther from the sea than that; one closer is taken as sea (better a basin flooded at the sea's
// level than a bay drained).
//
// Stage W5m, THE SPILL: a below-sea region the sea reaches through ground no higher than
// SpillLevel (m; 4-connected) is sea too: its spill level is at the sea's, within what one sample a
// cell can tell (a berm, a beach ridge, a dune line between a lagoon and the sea is narrower and
// lower than the cells that sample it). And THE LAGOONS: with Water (the bake's kept lakes' levels
// per cell, -3e38 where none), a cell under a kept lake whose level is at most SpillLevel counts as
// below sea level: the land fills a coastal depression to its spill, a lagoon at the sea's level
// whose bed the cells may sample above it, and the ocean drained and culled it (W5l's wall of water
// along a coast, the land's lake drawn in its place). A true inland basin's rim, or a lake's level,
// stands higher.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace VoxelLandSea
{
    struct FStats
    {
        int SeaCells = 0;       // interior cells marked
        int SeaRegions = 0;
        int BasinRegions = 0;
    };

    namespace Detail
    {
        // The cube-sphere's faces (VoxelCubeSphere.h): normal, U, V.
        inline void Face(int F, double* NUV)
        {
            static const double Faces[6][9] = {
                { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { -1, 0, 0, 0, 0, 1, 0, 1, 0 }, { 0, 1, 0, 0, 0, 1, 1, 0, 0 },
                { 0, -1, 0, 1, 0, 0, 0, 0, 1 }, { 0, 0, 1, 1, 0, 0, 0, 1, 0 }, { 0, 0, -1, 0, 1, 0, 1, 0, 0 } };
            for (int i = 0; i < 9; i++) NUV[i] = Faces[F][i];
        }

        inline double CellCentre(int I, int N) { return -1.0 + (2.0 * I + 1.0) / N; }

        // The interior cell (face, i, j) holding the centre of cell (I, J) of face F (I, J may be
        // past the face's edges).
        inline void CellOf(int F, int I, int J, int N, int& OutF, int& OutI, int& OutJ)
        {
            double A[9];
            Face(F, A);
            const double TU = std::tan(CellCentre(I, N) * 0.7853981633974483);
            const double TV = std::tan(CellCentre(J, N) * 0.7853981633974483);
            const double D[3] = { A[0] + A[3] * TU + A[6] * TV, A[1] + A[4] * TU + A[7] * TV, A[2] + A[5] * TU + A[8] * TV };
            int Best = 0;
            double BestDot = -1.0e30;
            for (int G = 0; G < 6; G++)
            {
                double B[9];
                Face(G, B);
                const double Dot = B[0] * D[0] + B[1] * D[1] + B[2] * D[2];
                if (Dot > BestDot) { BestDot = Dot; Best = G; }
            }
            double B[9];
            Face(Best, B);
            const double S = std::atan((B[3] * D[0] + B[4] * D[1] + B[5] * D[2]) / BestDot) / 0.7853981633974483;
            const double T = std::atan((B[6] * D[0] + B[7] * D[1] + B[8] * D[2]) / BestDot) / 0.7853981633974483;
            OutF = Best;
            OutI = std::min(std::max(int(std::floor((S + 1.0) * 0.5 * N)), 0), N - 1);
            OutJ = std::min(std::max(int(std::floor((T + 1.0) * 0.5 * N)), 0), N - 1);
        }
    }

    /** Sets Bit in Attr on the sea-connected cells and clears it elsewhere. Height and Attr: 6 faces
     *  of (N + 2 Pad)^2 cells (the bake's layout). */
    inline FStats Mark(int N, int Pad, const float* Height, uint16_t* Attr, uint16_t Bit, double MinShare, int BridgeCells = 3,
        const float* Water = nullptr, float SpillLevel = 3.0f)
    {
        FStats Stats;
        if (N <= 0 || !Height || !Attr) return Stats;
        const int S = N + 2 * Pad;
        auto Padded = [S, Pad](int F, int I, int J) { return (F * S + J + Pad) * S + I + Pad; };
        auto Interior = [N](int F, int I, int J) { return (F * N + J) * N + I; };

        // Across each face edge: edge e of face f, cell k along it, the interior cell next to it.
        std::vector<int> Across(size_t(24) * N);
        for (int F = 0; F < 6; F++)
        {
            for (int K = 0; K < N; K++)
            {
                const int Apron[4][2] = { { N, K }, { -1, K }, { K, N }, { K, -1 } };
                for (int E = 0; E < 4; E++)
                {
                    int F2 = 0, I2 = 0, J2 = 0;
                    Detail::CellOf(F, Apron[E][0], Apron[E][1], N, F2, I2, J2);
                    Across[size_t(F * 4 + E) * N + K] = Interior(F2, I2, J2);
                }
            }
        }

        // The below-sea regions, labelled by a flood from each unlabelled below-sea cell.
        const int Cells = 6 * N * N;
        std::vector<int> Label(size_t(Cells), -1);
        std::vector<int> Size;
        std::vector<int> Queue;
        Queue.reserve(1 << 16);
        long long Below = 0;
        auto IsBelow = [&](int Cell)
            {
                const int F = Cell / (N * N), J = (Cell / N) % N, I = Cell % N;
                const int P = Padded(F, I, J);
                // (Stage W5m: a lagoon's cells, under a kept lake at the sea's level, too.)
                return Height[P] < 0.0f || (Water && Water[P] > -1.0e30f && Water[P] <= SpillLevel);
            };
        // Stage W5m: ground the sea spills over (the spill's flood passes through it).
        auto IsLow = [&](int Cell)
            {
                const int F = Cell / (N * N), J = (Cell / N) % N, I = Cell % N;
                return Height[Padded(F, I, J)] <= SpillLevel || IsBelow(Cell);
            };
        for (int Start = 0; Start < Cells; Start++)
        {
            if (Label[size_t(Start)] >= 0 || !IsBelow(Start)) continue;
            const int Region = int(Size.size());
            Size.push_back(0);
            Label[size_t(Start)] = Region;
            Queue.clear();
            Queue.push_back(Start);
            for (size_t Head = 0; Head < Queue.size(); Head++)
            {
                const int Cell = Queue[Head];
                Size[size_t(Region)]++;
                const int F = Cell / (N * N), J = (Cell / N) % N, I = Cell % N;
                const int Next[4] = {
                    I + 1 < N ? Cell + 1 : Across[size_t(F * 4 + 0) * N + J],
                    I > 0 ? Cell - 1 : Across[size_t(F * 4 + 1) * N + J],
                    J + 1 < N ? Cell + N : Across[size_t(F * 4 + 2) * N + I],
                    J > 0 ? Cell - N : Across[size_t(F * 4 + 3) * N + I] };
                for (int E = 0; E < 4; E++)
                {
                    const int Other = Next[E];
                    if (Label[size_t(Other)] >= 0 || !IsBelow(Other)) continue;
                    Label[size_t(Other)] = Region;
                    Queue.push_back(Other);
                }
            }
            Below += Size[size_t(Region)];
        }

        // Sea: regions of at least MinShare of the below-sea cells, and the largest.
        std::vector<unsigned char> IsSea(Size.size(), 0);
        int Largest = -1;
        for (int r = 0; r < int(Size.size()); r++)
        {
            if (Largest < 0 || Size[size_t(r)] > Size[size_t(Largest)]) Largest = r;
            if (double(Size[size_t(r)]) >= MinShare * double(Below)) IsSea[size_t(r)] = 1;
        }
        if (Largest >= 0) IsSea[size_t(Largest)] = 1;
        // Stage W5l: the bridge (above), until no region joins (a joined one bridges on); stage W5m:
        // each pass first the spill (a flood from the sea through low ground).
        if (BridgeCells > 0 || SpillLevel >= 0.0f)
        {
            std::vector<int> Steps(size_t(Cells), -1);
            std::vector<unsigned char> Reached(size_t(Cells), 0);
            for (int Pass = 0; Pass < 8; Pass++)
            {
                bool bSpilled = false;
                if (SpillLevel >= 0.0f)
                {
                    std::fill(Reached.begin(), Reached.end(), 0);
                    Queue.clear();
                    for (int Cell = 0; Cell < Cells; Cell++)
                        if (Label[size_t(Cell)] >= 0 && IsSea[size_t(Label[size_t(Cell)])])
                        {
                            Reached[size_t(Cell)] = 1;
                            Queue.push_back(Cell);
                        }
                    for (size_t Head = 0; Head < Queue.size(); Head++)
                    {
                        const int Cell = Queue[Head];
                        const int F = Cell / (N * N), J = (Cell / N) % N, I = Cell % N;
                        const int Next[4] = {
                            I + 1 < N ? Cell + 1 : Across[size_t(F * 4 + 0) * N + J],
                            I > 0 ? Cell - 1 : Across[size_t(F * 4 + 1) * N + J],
                            J + 1 < N ? Cell + N : Across[size_t(F * 4 + 2) * N + I],
                            J > 0 ? Cell - N : Across[size_t(F * 4 + 3) * N + I] };
                        for (int E = 0; E < 4; E++)
                        {
                            const int Other = Next[E];
                            if (Reached[size_t(Other)] || !IsLow(Other)) continue;
                            Reached[size_t(Other)] = 1;
                            Queue.push_back(Other);
                            const int L = Label[size_t(Other)];
                            if (L >= 0 && !IsSea[size_t(L)])
                            {
                                IsSea[size_t(L)] = 1;
                                bSpilled = true;
                            }
                        }
                    }
                }
                if (BridgeCells <= 0)
                {
                    if (!bSpilled) break;
                    continue;
                }
                std::fill(Steps.begin(), Steps.end(), -1);
                Queue.clear();
                for (int Cell = 0; Cell < Cells; Cell++)
                    if (Label[size_t(Cell)] >= 0 && IsSea[size_t(Label[size_t(Cell)])])
                    {
                        Steps[size_t(Cell)] = 0;
                        Queue.push_back(Cell);
                    }
                bool bJoined = false;
                for (size_t Head = 0; Head < Queue.size(); Head++)
                {
                    const int Cell = Queue[Head];
                    if (Steps[size_t(Cell)] >= BridgeCells) continue;
                    const int F = Cell / (N * N), J = (Cell / N) % N, I = Cell % N;
                    const int Next[4] = {
                        I + 1 < N ? Cell + 1 : Across[size_t(F * 4 + 0) * N + J],
                        I > 0 ? Cell - 1 : Across[size_t(F * 4 + 1) * N + J],
                        J + 1 < N ? Cell + N : Across[size_t(F * 4 + 2) * N + I],
                        J > 0 ? Cell - N : Across[size_t(F * 4 + 3) * N + I] };
                    for (int E = 0; E < 4; E++)
                    {
                        const int Other = Next[E];
                        if (Steps[size_t(Other)] >= 0) continue;
                        Steps[size_t(Other)] = Steps[size_t(Cell)] + 1;
                        Queue.push_back(Other);
                        const int L = Label[size_t(Other)];
                        if (L >= 0 && !IsSea[size_t(L)])
                        {
                            IsSea[size_t(L)] = 1;
                            bJoined = true;
                        }
                    }
                }
                if (!bJoined && !bSpilled) break;
            }
        }
        for (unsigned char Sea : IsSea)
        {
            if (Sea) Stats.SeaRegions++;
            else Stats.BasinRegions++;
        }

        // The bit: interior cells from their region, apron cells from the interior cell holding them.
        for (int F = 0; F < 6; F++)
        {
            for (int J = -Pad; J < N + Pad; J++)
            {
                for (int I = -Pad; I < N + Pad; I++)
                {
                    const bool bInside = I >= 0 && I < N && J >= 0 && J < N;
                    int Cell = 0;
                    if (bInside)
                    {
                        Cell = Interior(F, I, J);
                    }
                    else
                    {
                        int F2 = 0, I2 = 0, J2 = 0;
                        Detail::CellOf(F, I, J, N, F2, I2, J2);
                        Cell = Interior(F2, I2, J2);
                    }
                    const bool bSea = Label[size_t(Cell)] >= 0 && IsSea[size_t(Label[size_t(Cell)])] != 0;
                    uint16_t& A = Attr[Padded(F, I, J)];
                    A = uint16_t(bSea ? (A | Bit) : (A & ~Bit));
                    if (bSea && bInside) Stats.SeaCells++;
                }
            }
        }
        return Stats;
    }
}
