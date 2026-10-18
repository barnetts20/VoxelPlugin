// rivers_r1_reference.h - R1's CPU river extraction and tile build (the plugin's VoxelLandRivers.h until
// R1b moved the build to the GPU), kept as the river harness's reference for the network.
//
// (R1's header follows.)
// VoxelLandRivers.h - The rivers of a land bake (Design/RiversLakes.md, R1): the network the
// bake's final routing makes, as lines with a floor height and a drainage area, and the per-tile
// lists the per-sample code looks them up in (VoxelLandRivers.ush).
//
// Derived from the bake when it is ready and whenever the river settings change (not cached:
// the bake caches the receivers; extraction and the tiles take well under a second). Plain C++
// without engine types: the harnesses run the same code.
//
// The network:
//   - River cells: interior land cells (with a receiver) outside the kept lakes, draining at
//     least MinAreaKm2. Every cell downstream of one is a river too (the drainage field spreads
//     on hillslopes, so it isn't strictly monotone below the channels' threshold).
//   - Nodes: the river cells; a terminal node where a river enters a lake (at the lake cell, the
//     floor at the lake's level) or the sea (at the sea cell, the floor at most 0); a start node
//     at each lake cell that drains out of its lake into a river (the floor at the lake's level).
//   - Floors: the bake's height at the node, made monotone (non-increasing) downstream.
//   - Chains: runs between sources, confluences, lake inlets and outlets and the sea; their inner
//     positions smoothed (1-2-1, 4 iterations, the ends fixed), then simplified (Douglas-Peucker
//     against the position, the floor and the drainage) into segments. The floor's tolerance is
//     3 m or 5% of the span's drop, whichever is larger: steep reaches are concave over their
//     length, and a fixed tolerance kept nearly every node in the mountains.
//
// The tiles: TileCells x TileCells cells of a face each (bake resolutions are multiples of 16:
// a partial last tile only a cell or two wide could be missed by the coverage's sampling), with the segments whose corridor (a
// radius by drainage from the caller, plus a margin) overlaps the tile, largest first, at most
// MaxPerTile. Each tile has a frame (its centre's direction and two tangents, float) and stores
// its entries' endpoints in that frame's gnomonic plane, in metres: x = R (p . t1) / (p . c).
// The per-sample code projects its point the same way and takes the distance through the
// plane's metric there (VoxelLandRivers.ush), so distances agree across tile and face edges to
// centimetres.
//
// The data layout (one float array, Data; offsets in floats):
//   frames    from 0:          12 per tile: c.xyz, 0, t1.xyz, 0, t2.xyz, 0
//   entries   from EntryBase:   8 per entry: x0, y0, dx, dy, 1 / (dx^2 + dy^2), segment, 0, 0
//   segments  from SegBase:     8 per segment: floor0, floor1, log2 area0, log2 area1 (m^2),
//                               s0 (m along its chain), length (m), the chain's length (m), flags
// and the tiles' first entries in Tiles (TileCount + 1 words).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace VoxelLandRiversR1
{
    constexpr uint32_t None = 0xffffffffu;
    constexpr int TileCells = 16;
    constexpr int MaxPerTile = 64;
    constexpr int FrameFloats = 12;
    constexpr int EntryFloats = 8;
    constexpr int SegFloats = 8;
    // Segment flags (the last float of a segment, exact).
    constexpr uint32_t FlagMouth = 1u;       // ends at the sea
    constexpr uint32_t FlagLakeInlet = 2u;   // ends in a lake
    constexpr uint32_t FlagLakeOutlet = 4u;  // starts at a lake's outlet
    constexpr uint32_t FlagSource = 8u;      // starts at a source

    /** The bake's grid (VoxelCubeSphere.ush; keep the face table equal). */
    struct FGrid
    {
        int N = 0;
        int Pad = 0;
        double RadiusM = 1.0;

        int Side() const { return N + 2 * Pad; }
        uint32_t NumCells() const { return 6u * (uint32_t)Side() * (uint32_t)Side(); }
        double CellArcM() const { return 1.5707963267948966 * RadiusM / (double)N; }

        uint32_t Encode(int Face, int I, int J) const
        {
            return (uint32_t)Face * (uint32_t)(Side() * Side()) + (uint32_t)(J + Pad) * (uint32_t)Side() + (uint32_t)(I + Pad);
        }

        void Decode(uint32_t Cell, int& Face, int& I, int& J) const
        {
            const uint32_t S2 = (uint32_t)(Side() * Side());
            Face = (int)(Cell / S2);
            const uint32_t Rest = Cell - (uint32_t)Face * S2;
            J = (int)(Rest / (uint32_t)Side()) - Pad;
            I = (int)(Rest % (uint32_t)Side()) - Pad;
        }

        bool IsInterior(uint32_t Cell) const
        {
            int F, I, J;
            Decode(Cell, F, I, J);
            return I >= 0 && J >= 0 && I < N && J < N;
        }

        /** Face f's normal axis and sign, U and V axes (VoxelCubeNormalAxis, ...). */
        static void Axes(int Face, int& NAxis, double& Sign, int& UAxis, int& VAxis)
        {
            static const int U[6] = { 1, 2, 2, 0, 0, 1 };
            static const int V[6] = { 2, 1, 0, 2, 1, 0 };
            NAxis = Face >> 1;
            Sign = (Face & 1) ? -1.0 : 1.0;
            UAxis = U[Face];
            VAxis = V[Face];
        }

        /** Unit direction of face coordinates S, T (VoxelCubeDirection, in double). */
        static void Direction(int Face, double S, double T, double Out[3])
        {
            int NA, UA, VA;
            double Sign;
            Axes(Face, NA, Sign, UA, VA);
            double D[3] = { 0.0, 0.0, 0.0 };
            D[NA] = Sign;
            D[UA] += std::tan(S * 0.78539816339744831);
            D[VA] += std::tan(T * 0.78539816339744831);
            const double L = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
            for (int k = 0; k < 3; k++) Out[k] = D[k] / L;
        }

        void CellDirection(uint32_t Cell, double Out[3]) const
        {
            int F, I, J;
            Decode(Cell, F, I, J);
            Direction(F, -1.0 + (2.0 * I + 1.0) / N, -1.0 + (2.0 * J + 1.0) / N, Out);
        }

        /** A direction's face and continuous cell coordinates (centres at whole numbers). */
        void Locate(const double D[3], int& Face, double& X, double& Y) const
        {
            const double AX = std::fabs(D[0]), AY = std::fabs(D[1]), AZ = std::fabs(D[2]);
            const int Axis = (AX >= AY && AX >= AZ) ? 0 : (AY >= AZ ? 1 : 2);
            Face = Axis * 2 + (D[Axis] < 0.0 ? 1 : 0);
            int NA, UA, VA;
            double Sign;
            Axes(Face, NA, Sign, UA, VA);
            const double Den = std::max(std::fabs(D[NA]), 1e-300);
            const double S = std::atan(D[UA] / Den) * 1.2732395447351627;
            const double T = std::atan(D[VA] / Den) * 1.2732395447351627;
            X = (S + 1.0) * 0.5 * N - 0.5;
            Y = (T + 1.0) * 0.5 * N - 0.5;
        }

        int TilesPerFace() const { return (N + TileCells - 1) / TileCells; }
        uint32_t TileCount() const { return 6u * (uint32_t)(TilesPerFace() * TilesPerFace()); }

        /** The tile holding a direction. */
        uint32_t TileOf(const double D[3]) const
        {
            int F;
            double X, Y;
            Locate(D, F, X, Y);
            const int I = std::clamp((int)std::floor(X + 0.5), 0, N - 1);
            const int J = std::clamp((int)std::floor(Y + 0.5), 0, N - 1);
            const int T = TilesPerFace();
            return (uint32_t)F * (uint32_t)(T * T) + (uint32_t)(J / TileCells) * (uint32_t)T + (uint32_t)(I / TileCells);
        }
    };

    struct FInput
    {
        FGrid Grid;
        const float* Height = nullptr;      // per cell (m)
        const float* Water = nullptr;       // per cell: the kept lake's level, or below -1e30
        const uint32_t* Receiver = nullptr; // per cell: the final routing's receiver, or None
        std::function<float(uint32_t)> FlowLog2;   // per cell: log2 of the drainage area (m^2)
    };

    struct FSettings
    {
        double MinAreaKm2 = 200.0;
        double SimplifyCells = 0.3;          // Douglas-Peucker's position tolerance (cells)
        double SimplifyFloorM = 3.0;         // and the floor's (m), or this share of the span's drop
        double SimplifyFloorShare = 0.05;
        double SimplifyLog2Area = 0.25;      // and the drainage's (log2)
    };

    struct FSegment
    {
        float Floor0 = 0.0f, Floor1 = 0.0f;
        float LogArea0 = 0.0f, LogArea1 = 0.0f;
        float S0 = 0.0f, Length = 0.0f, ChainLength = 0.0f;
        uint32_t Flags = 0;
    };

    struct FNetwork
    {
        std::vector<double> Node;            // 3 per node: unit direction
        std::vector<float> NodeFloor;
        std::vector<float> NodeLog2Area;
        std::vector<uint32_t> SegA, SegB;    // node indices
        std::vector<FSegment> Seg;
        // Statistics.
        uint32_t RiverCells = 0, Chains = 0, Mouths = 0, LakeInlets = 0, LakeOutlets = 0, Sources = 0, Confluences = 0;
        uint32_t RawNodes = 0;
        double TotalLengthKm = 0.0;
    };

    struct FTiles
    {
        int TilesPerFace = 0;
        std::vector<uint32_t> Tiles;         // TileCount + 1: each tile's first entry
        std::vector<float> Data;             // frames, entries, segments (see the top)
        uint32_t EntryBase = 0, SegBase = 0; // offsets in Data (floats; multiples of 4)
        uint32_t Entries = 0, Overflow = 0, MaxInTile = 0, TilesUsed = 0;
    };

    namespace Detail
    {
        inline void Normalize(double D[3])
        {
            const double L = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
            if (L > 0.0) for (int k = 0; k < 3; k++) D[k] /= L;
        }

        inline double Dot(const double* A, const double* B) { return A[0] * B[0] + A[1] * B[1] + A[2] * B[2]; }

        inline double ArcM(const double* A, const double* B, double R)
        {
            const double C[3] = { A[1] * B[2] - A[2] * B[1], A[2] * B[0] - A[0] * B[2], A[0] * B[1] - A[1] * B[0] };
            return std::atan2(std::sqrt(Dot(C, C)), Dot(A, B)) * R;
        }

        /** Douglas-Peucker on a chain's points (positions, floors, drainage): Keep marks the kept. */
        inline void Simplify(const std::vector<const double*>& P, const std::vector<float>& Floor, const std::vector<float>& Area,
            const std::vector<double>& S, double TolRad, double TolFloor, double TolFloorShare, double TolArea, std::vector<char>& Keep)
        {
            const size_t Count = P.size();
            Keep.assign(Count, 0);
            Keep[0] = Keep[Count - 1] = 1;
            std::vector<std::pair<size_t, size_t>> Stack;
            Stack.push_back({ 0, Count - 1 });
            while (!Stack.empty())
            {
                const auto [A, B] = Stack.back();
                Stack.pop_back();
                if (B <= A + 1) continue;
                // The floor's tolerance grows with the span's drop (a steep reach is concave
                // over its length; a flat one keeps its floor within the fixed tolerance).
                const double FloorTol = std::max(TolFloor, TolFloorShare * std::fabs((double)Floor[B] - (double)Floor[A]));
                // The chord A-B's plane normal (great circle), for the angular distance.
                double Nrm[3] = { P[A][1] * P[B][2] - P[A][2] * P[B][1], P[A][2] * P[B][0] - P[A][0] * P[B][2], P[A][0] * P[B][1] - P[A][1] * P[B][0] };
                const double NL = std::sqrt(Dot(Nrm, Nrm));
                double Worst = 0.0;
                size_t At = A;
                for (size_t k = A + 1; k < B; k++)
                {
                    const double T = S[B] > S[A] ? (S[k] - S[A]) / (S[B] - S[A]) : 0.0;
                    double E = 0.0;
                    if (NL > 1e-15) E = std::fabs(Dot(P[k], Nrm)) / NL / TolRad;
                    else E = std::acos(std::clamp(Dot(P[k], P[A]), -1.0, 1.0)) / TolRad;
                    E = std::max(E, std::fabs(Floor[k] - (Floor[A] + (Floor[B] - Floor[A]) * T)) / FloorTol);
                    E = std::max(E, std::fabs(Area[k] - (Area[A] + (Area[B] - Area[A]) * T)) / TolArea);
                    if (E > Worst) { Worst = E; At = k; }
                }
                if (Worst > 1.0)
                {
                    Keep[At] = 1;
                    Stack.push_back({ A, At });
                    Stack.push_back({ At, B });
                }
            }
        }
    }

    /** The network (see the top). */
    inline FNetwork Extract(const FInput& In, const FSettings& Settings)
    {
        using namespace Detail;
        FNetwork Out;
        const FGrid& G = In.Grid;
        const uint32_t Cells = G.NumCells();
        if (!In.Height || !In.Water || !In.Receiver || !In.FlowLog2 || G.N <= 0) return Out;
        const float MinLog2 = (float)std::log2(std::max(Settings.MinAreaKm2, 1e-6) * 1.0e6);
        auto IsLake = [&](uint32_t C) { return In.Water[C] > -1.0e30f; };
        // (A cell that is its own receiver has none: the routing's no-receiver case.)
        auto IsLand = [&](uint32_t C) { return In.Receiver[C] != None && In.Receiver[C] < Cells && In.Receiver[C] != C; };

        // River cells, and every land cell below one.
        std::vector<char> River(Cells, 0);
        for (uint32_t C = 0; C < Cells; C++)
            if (IsLand(C) && !IsLake(C) && G.IsInterior(C) && In.FlowLog2(C) >= MinLog2) River[C] = 1;
        for (uint32_t C = 0; C < Cells; C++)
        {
            if (!River[C]) continue;
            uint32_t Q = In.Receiver[C];
            for (int Guard = 0; Guard < 1 << 20 && Q < Cells && IsLand(Q) && !IsLake(Q) && !River[Q]; Guard++)
            {
                River[Q] = 1;
                Q = In.Receiver[Q];
            }
        }

        // Nodes: river cells, then terminals and lake-outlet starts.
        struct FRaw { uint32_t Cell; uint32_t Down; float Floor; float Area; uint32_t Kind; };   // Kind: 0 river, 1 mouth, 2 lake inlet, 3 outlet start
        std::vector<FRaw> Raw;
        // The river nodes are the river cells in increasing order: a cell's node by a binary
        // search (no per-cell table: a 2048 bake has 25 M cells); the terminals in a map.
        for (uint32_t C = 0; C < Cells; C++)
            if (River[C])
            {
                Raw.push_back({ C, None, In.Height[C], In.FlowLog2(C), 0u });
                Out.RiverCells++;
            }
        const size_t RiverNodes = Raw.size();
        auto NodeOf = [&](uint32_t C) {
            const auto It = std::lower_bound(Raw.begin(), Raw.begin() + (std::ptrdiff_t)RiverNodes, C, [](const FRaw& R, uint32_t V) { return R.Cell < V; });
            return (uint32_t)(It - Raw.begin());
        };
        std::unordered_map<uint32_t, uint32_t> TerminalOf;
        for (size_t n = 0; n < RiverNodes; n++)
        {
            const uint32_t Q = In.Receiver[Raw[n].Cell];
            if (Q >= Cells) continue;
            if (River[Q]) { Raw[n].Down = NodeOf(Q); continue; }
            // A terminal at Q: a lake (its level) or the sea (at most sea level).
            auto It = TerminalOf.find(Q);
            if (It == TerminalOf.end())
            {
                It = TerminalOf.emplace(Q, (uint32_t)Raw.size()).first;
                const bool bLake = IsLake(Q);
                Raw.push_back({ Q, None, bLake ? In.Water[Q] : std::min(In.Height[Q], 0.0f), Raw[n].Area, bLake ? 2u : 1u });
                if (bLake) Out.LakeInlets++; else Out.Mouths++;
            }
            const uint32_t T = It->second;
            Raw[n].Down = T;
            Raw[T].Area = std::max(Raw[T].Area, Raw[n].Area);
        }
        // Lake outlets: every lake cell draining out of its lake into a river (whatever the lake
        // table holds: it is capped).
        for (uint32_t O = 0; O < Cells; O++)
        {
            if (!IsLake(O) || !IsLand(O)) continue;
            const uint32_t Q = In.Receiver[O];
            if (Q >= Cells || IsLake(Q) || !River[Q]) continue;
            Raw.push_back({ O, NodeOf(Q), In.Water[O], In.FlowLog2(Q), 3u });
            Out.LakeOutlets++;
        }
        const uint32_t NumRaw = (uint32_t)Raw.size();
        Out.RawNodes = NumRaw;

        // Upstream counts, then the floors made monotone downstream (in topological order).
        std::vector<uint32_t> Up(NumRaw, 0);
        for (const FRaw& R : Raw) if (R.Down != None) Up[R.Down]++;
        {
            std::vector<uint32_t> Pending(Up), Queue;
            for (uint32_t n = 0; n < NumRaw; n++) if (Pending[n] == 0) Queue.push_back(n);
            for (size_t q = 0; q < Queue.size(); q++)
            {
                const uint32_t n = Queue[q];
                const uint32_t D = Raw[n].Down;
                if (D == None) continue;
                // A lake inlet keeps the lake's level; the others take the lowest from above.
                if (Raw[D].Kind != 2u) Raw[D].Floor = std::min(Raw[D].Floor, Raw[n].Floor);
                if (--Pending[D] == 0) Queue.push_back(D);
            }
        }

        // Positions.
        std::vector<double> Pos(3 * (size_t)NumRaw);
        for (uint32_t n = 0; n < NumRaw; n++) G.CellDirection(Raw[n].Cell, &Pos[3 * (size_t)n]);

        // Chains: from every node that isn't the single continuation of one above (sources,
        // confluences, outlet starts), down to the next confluence or end.
        std::vector<std::vector<uint32_t>> Chains;
        for (uint32_t n = 0; n < NumRaw; n++)
        {
            if (Up[n] == 1 && Raw[n].Kind != 3u) continue;
            if (Raw[n].Down == None) continue;
            if (Up[n] == 0 && Raw[n].Kind == 0u) Out.Sources++;
            if (Up[n] >= 2) Out.Confluences++;
            std::vector<uint32_t> Chain = { n };
            uint32_t m = Raw[n].Down;
            for (uint32_t Guard = 0; m != None && Guard < NumRaw; Guard++)
            {
                Chain.push_back(m);
                if (Up[m] != 1 || Raw[m].Down == None) break;
                m = Raw[m].Down;
            }
            Chains.push_back(std::move(Chain));
        }
        Out.Chains = (uint32_t)Chains.size();

        // Smoothing: each chain's inner nodes (they belong to that chain alone).
        for (const std::vector<uint32_t>& Chain : Chains)
        {
            if (Chain.size() < 3) continue;
            for (int Iter = 0; Iter < 4; Iter++)
            {
                std::vector<double> Next(3 * Chain.size());
                for (size_t k = 1; k + 1 < Chain.size(); k++)
                {
                    double* D = &Next[3 * k];
                    for (int a = 0; a < 3; a++)
                        D[a] = 0.25 * Pos[3 * (size_t)Chain[k - 1] + a] + 0.5 * Pos[3 * (size_t)Chain[k] + a] + 0.25 * Pos[3 * (size_t)Chain[k + 1] + a];
                    Normalize(D);
                }
                for (size_t k = 1; k + 1 < Chain.size(); k++)
                    for (int a = 0; a < 3; a++) Pos[3 * (size_t)Chain[k] + a] = Next[3 * k + a];
            }
        }

        // Simplification, and the output: the kept nodes once each, the segments between them.
        std::vector<uint32_t> OutNode(NumRaw, None);
        auto Emit = [&](uint32_t n) {
            if (OutNode[n] == None)
            {
                OutNode[n] = (uint32_t)(Out.NodeFloor.size());
                for (int a = 0; a < 3; a++) Out.Node.push_back(Pos[3 * (size_t)n + a]);
                Out.NodeFloor.push_back(Raw[n].Floor);
                Out.NodeLog2Area.push_back(Raw[n].Area);
            }
            return OutNode[n];
        };
        const double TolRad = Settings.SimplifyCells * G.CellArcM() / G.RadiusM;
        for (const std::vector<uint32_t>& Chain : Chains)
        {
            const size_t Count = Chain.size();
            std::vector<const double*> P(Count);
            std::vector<float> Floor(Count), Area(Count);
            std::vector<double> S(Count, 0.0);
            for (size_t k = 0; k < Count; k++)
            {
                P[k] = &Pos[3 * (size_t)Chain[k]];
                Floor[k] = Raw[Chain[k]].Floor;
                Area[k] = Raw[Chain[k]].Area;
                if (k > 0) S[k] = S[k - 1] + ArcM(P[k - 1], P[k], G.RadiusM);
            }
            std::vector<char> Keep;
            Simplify(P, Floor, Area, S, TolRad, Settings.SimplifyFloorM, Settings.SimplifyFloorShare, Settings.SimplifyLog2Area, Keep);
            const double ChainLength = S[Count - 1];
            Out.TotalLengthKm += ChainLength * 1e-3;
            size_t Prev = 0;
            const uint32_t StartKind = Raw[Chain[0]].Kind;
            const uint32_t EndKind = Raw[Chain[Count - 1]].Kind;
            for (size_t k = 1; k < Count; k++)
            {
                if (!Keep[k]) continue;
                FSegment Sg;
                Sg.Floor0 = Floor[Prev];
                Sg.Floor1 = Floor[k];
                Sg.LogArea0 = Area[Prev];
                Sg.LogArea1 = Area[k];
                Sg.S0 = (float)S[Prev];
                Sg.Length = (float)(S[k] - S[Prev]);
                Sg.ChainLength = (float)ChainLength;
                if (Prev == 0 && StartKind == 3u) Sg.Flags |= FlagLakeOutlet;
                if (Prev == 0 && StartKind == 0u && Up[Chain[0]] == 0) Sg.Flags |= FlagSource;
                if (k == Count - 1 && EndKind == 1u) Sg.Flags |= FlagMouth;
                if (k == Count - 1 && EndKind == 2u) Sg.Flags |= FlagLakeInlet;
                Out.SegA.push_back(Emit(Chain[Prev]));
                Out.SegB.push_back(Emit(Chain[k]));
                Out.Seg.push_back(Sg);
                Prev = k;
            }
        }
        return Out;
    }

    /** The tiles for a network (see the top). RadiusM: a segment's corridor reach (m) for its
     *  larger log2 drainage area (m^2); ParallelFor(Count, Body) runs Body(i) for i < Count. */
    inline FTiles BuildTiles(const FGrid& G, const FNetwork& Net, const std::function<double(float)>& RadiusM,
        const std::function<void(int, const std::function<void(int)>&)>& ParallelFor)
    {
        using namespace Detail;
        FTiles Out;
        const int T = G.TilesPerFace();
        Out.TilesPerFace = T;
        const uint32_t TileCount = G.TileCount();
        const uint32_t NumSegs = (uint32_t)Net.Seg.size();

        // The frames: each tile's centre and tangents, rounded to float as stored.
        std::vector<float> Frames((size_t)TileCount * FrameFloats, 0.0f);
        for (uint32_t t = 0; t < TileCount; t++)
        {
            const int Face = (int)(t / (uint32_t)(T * T));
            const int Rest = (int)(t % (uint32_t)(T * T));
            const int TJ = Rest / T, TI = Rest % T;
            const double CI = std::min((TI + 0.5) * TileCells, 0.5 * (G.N + TI * TileCells));   // a partial last tile's middle
            const double CJ = std::min((TJ + 0.5) * TileCells, 0.5 * (G.N + TJ * TileCells));
            double C[3];
            FGrid::Direction(Face, -1.0 + 2.0 * CI / G.N, -1.0 + 2.0 * CJ / G.N, C);
            int NA, UA, VA;
            double Sign;
            FGrid::Axes(Face, NA, Sign, UA, VA);
            double T1[3] = { 0.0, 0.0, 0.0 };
            T1[UA] = 1.0;
            const double CU = Dot(T1, C);
            for (int a = 0; a < 3; a++) T1[a] -= C[a] * CU;
            Normalize(T1);
            double T2[3] = { C[1] * T1[2] - C[2] * T1[1], C[2] * T1[0] - C[0] * T1[2], C[0] * T1[1] - C[1] * T1[0] };
            Normalize(T2);
            float* F = &Frames[(size_t)t * FrameFloats];
            for (int a = 0; a < 3; a++) { F[a] = (float)C[a]; F[4 + a] = (float)T1[a]; F[8 + a] = (float)T2[a]; }
        }

        // Each segment's tiles: points along it at half a cell, and around each a disc of its
        // reach plus a cell and a half, at three quarters of a cell.
        std::vector<std::vector<uint32_t>> SegTiles(NumSegs);
        const double Cell = G.CellArcM();
        ParallelFor((int)NumSegs, [&](int s) {
            const double* A = &Net.Node[3 * (size_t)Net.SegA[s]];
            const double* B = &Net.Node[3 * (size_t)Net.SegB[s]];
            const double Reach = RadiusM(std::max(Net.Seg[s].LogArea0, Net.Seg[s].LogArea1)) + 1.5 * Cell;
            const double Len = ArcM(A, B, G.RadiusM);
            const int Along = std::max(1, (int)std::ceil(Len / (0.5 * Cell)));
            const int Across = (int)std::ceil(Reach / (0.75 * Cell));
            std::vector<uint32_t>& List = SegTiles[s];
            for (int k = 0; k <= Along; k++)
            {
                const double U = (double)k / Along;
                double P[3] = { A[0] + (B[0] - A[0]) * U, A[1] + (B[1] - A[1]) * U, A[2] + (B[2] - A[2]) * U };
                Normalize(P);
                // A tangent frame at P.
                double E1[3] = { B[0] - A[0], B[1] - A[1], B[2] - A[2] };
                const double PE = Dot(E1, P);
                for (int a = 0; a < 3; a++) E1[a] -= P[a] * PE;
                if (Dot(E1, E1) < 1e-30) { E1[0] = P[1]; E1[1] = -P[0]; E1[2] = 0.0; if (Dot(E1, E1) < 1e-30) { E1[0] = 1.0; E1[1] = 0.0; } }
                Normalize(E1);
                double E2[3] = { P[1] * E1[2] - P[2] * E1[1], P[2] * E1[0] - P[0] * E1[2], P[0] * E1[1] - P[1] * E1[0] };
                for (int i = -Across; i <= Across; i++)
                    for (int j = -Across; j <= Across; j++)
                    {
                        const double X = i * 0.75 * Cell, Y = j * 0.75 * Cell;
                        if (X * X + Y * Y > (Reach + 0.75 * Cell) * (Reach + 0.75 * Cell)) continue;
                        double Q[3];
                        for (int a = 0; a < 3; a++) Q[a] = P[a] + (E1[a] * X + E2[a] * Y) / G.RadiusM;
                        Normalize(Q);
                        const uint32_t Tile = G.TileOf(Q);
                        if (std::find(List.begin(), List.end(), Tile) == List.end()) List.push_back(Tile);
                    }
            }
        });

        // Per tile, its segments, largest first (then by index), capped.
        std::vector<std::vector<uint32_t>> TileSegs(TileCount);
        for (uint32_t s = 0; s < NumSegs; s++)
            for (uint32_t Tile : SegTiles[s]) TileSegs[Tile].push_back(s);
        auto Larger = [&](uint32_t A, uint32_t B) {
            const float LA = std::max(Net.Seg[A].LogArea0, Net.Seg[A].LogArea1), LB = std::max(Net.Seg[B].LogArea0, Net.Seg[B].LogArea1);
            return LA != LB ? LA > LB : A < B;
        };
        Out.Tiles.assign(TileCount + 1, 0u);
        uint32_t Total = 0;
        for (uint32_t t = 0; t < TileCount; t++)
        {
            std::vector<uint32_t>& L = TileSegs[t];
            std::sort(L.begin(), L.end(), Larger);
            Out.MaxInTile = std::max(Out.MaxInTile, (uint32_t)L.size());
            if (L.size() > (size_t)MaxPerTile) { Out.Overflow += (uint32_t)(L.size() - MaxPerTile); L.resize(MaxPerTile); }
            if (!L.empty()) Out.TilesUsed++;
            Out.Tiles[t] = Total;
            Total += (uint32_t)L.size();
        }
        Out.Tiles[TileCount] = Total;
        Out.Entries = Total;

        // The data: frames, entries (in each tile's plane), segments.
        Out.EntryBase = TileCount * FrameFloats;
        Out.SegBase = Out.EntryBase + Total * EntryFloats;
        Out.Data.assign((size_t)Out.SegBase + (size_t)NumSegs * SegFloats, 0.0f);
        std::copy(Frames.begin(), Frames.end(), Out.Data.begin());
        ParallelFor((int)TileCount, [&](int t) {
            const float* F = &Frames[(size_t)t * FrameFloats];
            const double C[3] = { F[0], F[1], F[2] }, T1[3] = { F[4], F[5], F[6] }, T2[3] = { F[8], F[9], F[10] };
            auto Project = [&](const double* P, double& X, double& Y) {
                const double Den = Dot(P, C);
                X = G.RadiusM * Dot(P, T1) / Den;
                Y = G.RadiusM * Dot(P, T2) / Den;
            };
            const std::vector<uint32_t>& L = TileSegs[t];
            for (size_t k = 0; k < L.size(); k++)
            {
                const uint32_t s = L[k];
                double X0, Y0, X1, Y1;
                Project(&Net.Node[3 * (size_t)Net.SegA[s]], X0, Y0);
                Project(&Net.Node[3 * (size_t)Net.SegB[s]], X1, Y1);
                // Rounded to float first, so the stored inverse length matches the stored ends.
                const float FX0 = (float)X0, FY0 = (float)Y0, DX = (float)(X1 - X0), DY = (float)(Y1 - Y0);
                const float L2 = DX * DX + DY * DY;
                float* E = &Out.Data[Out.EntryBase + ((size_t)Out.Tiles[t] + k) * EntryFloats];
                E[0] = FX0; E[1] = FY0; E[2] = DX; E[3] = DY;
                E[4] = L2 > 0.0f ? 1.0f / L2 : 0.0f;
                E[5] = (float)s;
            }
        });
        for (uint32_t s = 0; s < NumSegs; s++)
        {
            const FSegment& Sg = Net.Seg[s];
            float* D = &Out.Data[Out.SegBase + (size_t)s * SegFloats];
            D[0] = Sg.Floor0; D[1] = Sg.Floor1; D[2] = Sg.LogArea0; D[3] = Sg.LogArea1;
            D[4] = Sg.S0; D[5] = Sg.Length; D[6] = Sg.ChainLength; D[7] = (float)Sg.Flags;
        }
        return Out;
    }

    /** A channel's width (m) for a log2 drainage area (m^2): 2.5 sqrt(km^2) (Design/RiversLakes.md 2.3). */
    inline double WidthM(double Log2Area, double Scale)
    {
        return Scale * 2.5 * std::sqrt(std::exp2(Log2Area) * 1.0e-6);
    }

    /** The valley's half-width (m) for a channel width: 400 m + 15 widths. */
    inline double ValleyHalfWidthM(double Width, double Scale)
    {
        return Scale * (400.0 + 15.0 * Width);
    }
}
