// river_build_cpu.h - The rivers' GPU build (VoxelLandRiverBuild.ush) compiled as C++ through
// VoxelShaderCompat.h and driven pass by pass in the order VoxelLandRiverBuild.cpp records them
// (the scans in C++), for the harnesses: the river harness, the gully harness's rivers, the
// material harness. Include after VoxelShaderCompat.h, with VoxelDensityCore.ush, VoxelDF64.ush
// and VoxelCubeSphere.ush included in namespace VoxelShader. Compile with -std=c++20 (the atomics).

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

namespace VoxelShader
{
    struct FRiverBuildShader
    {
        uint RiverN = 0, RiverPad = 4;
        float RiverRadiusM = 1.0e6f;
        const float2* RiverTan = nullptr;
        const float* RiverHeight = nullptr;
        const float* RiverWater = nullptr;
        const uint* RiverFlow = nullptr;
        const uint* RiverRec = nullptr;
        float RiverMinLog2 = 0.0f, RiverWidthScale = 1.0f, RiverValleyScale = 1.0f, RiverReachScale = 1.25f, RiverMarginCells = 0.25f;
        float RiverSimplifyCells = 0.3f, RiverSimplifyFloorM = 3.0f, RiverSimplifyFloorShare = 0.05f, RiverSimplifyLog2Area = 0.25f;
        uint RiverTilesPerFace = 0;
        uint RiverNodeCap = 0, RiverSegCap = 0, RiverCandCap = 0, RiverDataCap = 0;
        uint* RiverCounts = nullptr;
        uint* RiverCellFlags = nullptr;
        uint* RiverCellNode = nullptr;
        uint* NodeCell = nullptr; uint* NodeDown = nullptr; uint* NodePred = nullptr; uint* NodeUp = nullptr;
        uint* NodeKind = nullptr; uint* NodeFloor = nullptr; uint* NodeArea = nullptr; uint* NodeSeg = nullptr;
        float* NodePos = nullptr; float* NodePosOut = nullptr;
        uint* NodeJumpIn = nullptr; uint* NodeJumpOut = nullptr;
        float* NodeSIn = nullptr; float* NodeSOut = nullptr; float* NodeEndIn = nullptr; float* NodeEndOut = nullptr;
        uint* NodeIdxIn = nullptr; uint* NodeIdxOut = nullptr;
        uint* SegNode = nullptr; uint* SegEnd = nullptr;
        uint* TileStart = nullptr; uint* TileFill = nullptr; uint* TileBlock = nullptr; uint* TileCand = nullptr;
        uint* RiverTiles = nullptr;
        float* RiverData = nullptr;
#include "VoxelLandRiverBuild.ush"
    };
}

namespace RiverBuildCPU
{
    using VoxelShader::uint;

    /** The plugin's VoxelLandRiverBuild settings and caps (VoxelLandRiverBuild.h). */
    struct FSettings
    {
        int N = 0, Pad = 4;
        double RadiusM = 1.0e6;
        double MinAreaKm2 = 200.0, WidthScale = 1.0, ValleyScale = 1.0;
        uint32_t LandCells = 0;
    };

    struct FCaps
    {
        uint32_t Nodes = 0, Segments = 0, Candidates = 0, Data = 0;
    };

    /** Keep equal to VoxelLandRiverBuild::EstimateCaps (VoxelLandRiverBuild.cpp). */
    inline FCaps EstimateCaps(const FSettings& S)
    {
        const double Side = S.N + 2.0 * S.Pad;
        const double Cells = 6.0 * Side * Side;
        const double Land = S.LandCells > 0 ? (double)S.LandCells : Cells / 3.0;
        const double CellKm = 1.5707963267948966 * S.RadiusM / std::max(S.N, 1) * 1.0e-3;
        const double Ratio = std::max(S.MinAreaKm2, 1e-3) / std::max(CellKm * CellKm, 1e-9);
        const double Share = std::min(1.0, 0.5 * std::pow(std::max(Ratio, 1.0), -0.45));
        const double Tiles = 6.0 * std::pow(std::ceil(S.N / 8.0), 2.0);
        FCaps C;
        C.Nodes = (uint32_t)std::min(Cells, Land * Share * 1.1 + 4096.0);
        C.Segments = (uint32_t)(C.Nodes * 0.9 + 1024.0);
        C.Candidates = (uint32_t)(C.Segments * 4.0 + 4096.0);
        C.Data = (uint32_t)(C.Segments * (double)VOXEL_LAND_RIVER_SEG_FLOATS + std::min(Tiles, C.Candidates / 4.0) * 12.0 + C.Candidates * 7.0 + 4096.0);
        return C;
    }

    /** The tables, the counts, and the build's working arrays (for the checks). */
    struct FTables
    {
        int TilesPerFace = 0;
        std::vector<uint32_t> Tiles;
        std::vector<float> Data;            // trimmed
        uint32_t Counts[32] = {};
        bool bFits = false;
        int Builds = 0;                     // 1, or more when the estimate didn't fit
        FCaps Caps;                         // of the last build
        std::vector<uint32_t> SegNode, SegEnd, NodeCell, NodeKind, NodeDown, NodeUp, NodeIdx;
        std::vector<float> NodePos, NodeS;  // 4 a node
        std::vector<uint32_t> NodeFloor;
        uint32_t NumNodes = 0, NumSegments = 0;
    };

    template<typename F> inline void For(uint32_t Count, bool bParallel, F Fn)
    {
        if (!bParallel || Count < 4096) { for (uint32_t i = 0; i < Count; i++) Fn(i); return; }
        const unsigned T = std::max(1u, std::thread::hardware_concurrency());
        std::atomic<uint32_t> Next{ 0 };
        std::vector<std::thread> Th;
        for (unsigned t = 0; t < T; t++)
            Th.emplace_back([&]() { uint32_t i; while ((i = Next.fetch_add(1024)) < Count) for (uint32_t k = i; k < std::min(i + 1024, Count); k++) Fn(k); });
        for (auto& t : Th) t.join();
    }

    inline void Scan(uint32_t* Data, uint32_t Count, uint32_t* Counts, uint32_t Slot)
    {
        uint32_t Running = 0;
        for (uint32_t i = 0; i < Count; i++) { const uint32_t C = Data[i]; Data[i] = Running; Running += C; }
        Counts[Slot] = Running;
    }

    /** One build with the given caps (the plugin's AddPasses, in order). */
    inline FTables BuildOnce(const FSettings& S, const FCaps& Caps, const VoxelShader::float2* Tan, const float* Height, const float* Water,
        const uint32_t* FlowHalves, const uint32_t* Rec, bool bParallel)
    {
        using namespace VoxelShader;
        FRiverBuildShader B;
        B.RiverN = (uint)S.N; B.RiverPad = (uint)S.Pad; B.RiverRadiusM = (float)S.RadiusM;
        B.RiverTan = Tan; B.RiverHeight = Height; B.RiverWater = Water; B.RiverFlow = FlowHalves; B.RiverRec = Rec;
        B.RiverMinLog2 = (float)std::log2(std::max(S.MinAreaKm2, 1e-6) * 1.0e6);
        B.RiverWidthScale = (float)std::max(S.WidthScale, 0.1);
        B.RiverValleyScale = (float)std::max(S.ValleyScale, 0.1);
        const uint32_t T = (uint32_t)((S.N + 7) / 8);
        B.RiverTilesPerFace = T;
        const uint32_t TileCount = 6u * T * T;
        const uint32_t Side = (uint32_t)(S.N + 2 * S.Pad);
        const uint32_t Cells = 6u * Side * Side;
        B.RiverNodeCap = std::max(Caps.Nodes, 1u); B.RiverSegCap = std::max(Caps.Segments, 1u);
        B.RiverCandCap = std::max(Caps.Candidates, 1u); B.RiverDataCap = std::max(Caps.Data, 1u);
        const uint32_t NC = B.RiverNodeCap, SC = B.RiverSegCap;
        std::vector<uint32_t> Counts(32, 0u), CellFlags(Cells), CellNode(Cells);
        std::vector<uint32_t> NCell(NC), NDown(NC), NPred(NC), NUp(NC), NKind(NC), NFloor(NC), NArea(NC), NSeg(NC);
        std::vector<float> Pos[2] = { std::vector<float>(4 * (size_t)NC), std::vector<float>(4 * (size_t)NC) };
        std::vector<uint32_t> Jump[2] = { std::vector<uint32_t>(NC), std::vector<uint32_t>(NC) };
        std::vector<float> SV[2] = { std::vector<float>(2 * (size_t)NC), std::vector<float>(2 * (size_t)NC) };   // the places, then the meanders' phase
        std::vector<float> EV[2] = { std::vector<float>(NC), std::vector<float>(NC) };
        std::vector<uint32_t> IV[2] = { std::vector<uint32_t>(NC), std::vector<uint32_t>(NC) };
        std::vector<uint32_t> SNode(SC), SEnd(SC), TStart(TileCount, 0u), TFill(TileCount, 0u), TBlock(TileCount), TCand(B.RiverCandCap);
        std::vector<uint32_t> Tiles(TileCount + 1);
        std::vector<float> Data(B.RiverDataCap, 0.0f);
        B.RiverCounts = Counts.data(); B.RiverCellFlags = CellFlags.data(); B.RiverCellNode = CellNode.data();
        B.NodeCell = NCell.data(); B.NodeDown = NDown.data(); B.NodePred = NPred.data(); B.NodeUp = NUp.data();
        B.NodeKind = NKind.data(); B.NodeFloor = NFloor.data(); B.NodeArea = NArea.data(); B.NodeSeg = NSeg.data();
        B.SegNode = SNode.data(); B.SegEnd = SEnd.data(); B.TileStart = TStart.data(); B.TileFill = TFill.data();
        B.TileBlock = TBlock.data(); B.TileCand = TCand.data(); B.RiverTiles = Tiles.data(); B.RiverData = Data.data();
        auto Bind = [&](int P, int J, int SI, int EI, int II) {
            B.NodePos = Pos[P].data(); B.NodePosOut = Pos[1 - P].data();
            B.NodeJumpIn = Jump[J].data(); B.NodeJumpOut = Jump[1 - J].data();
            B.NodeSIn = SV[SI].data(); B.NodeSOut = SV[1 - SI].data();
            B.NodeEndIn = EV[EI].data(); B.NodeEndOut = EV[1 - EI].data();
            B.NodeIdxIn = IV[II].data(); B.NodeIdxOut = IV[1 - II].data();
        };
        Bind(0, 1, 1, 1, 1);
        // Cells.
        For(Cells, bParallel, [&](uint32_t c) { B.VoxelRiverSeedPass(c); });
        For(Cells, bParallel, [&](uint32_t c) { B.VoxelRiverClosurePass(c); });
        For(Cells, bParallel, [&](uint32_t c) { B.VoxelRiverRolesPass(c); });
        For(Cells, bParallel, [&](uint32_t c) { B.VoxelRiverNodeCountPass(c); });
        Scan(CellNode.data(), Cells, Counts.data(), 0);
        For(Cells, bParallel, [&](uint32_t c) { B.VoxelRiverNodeInitPass(c); });
        // Nodes (the link's predecessor writes race only where a node has several above it).
        For(NC, false, [&](uint32_t n) { B.VoxelRiverNodeLinkPass(n); });
        int P = 0;
        for (int k = 0; k < 4; k++) { Bind(P, 1, 1, 1, 1); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverSmoothPass(n); }); P ^= 1; }
        // Floors (jumps written to 0, then read from the current one).
        Bind(P, 1, 1, 1, 1); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverFloorInitPass(n); });
        int J = 0;
        for (int r = 0; r < 20; r++)
        {
            Bind(P, J, 1, 1, 1);
            For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverFloorPushPass(n); });
            For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverJumpPass(n); });
            J ^= 1;
        }
        Bind(P, J, 1, 1, 1); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverJumpCheckPass(n); });
        // Places along the chains (into S 0, Idx 0, Jump 0).
        Bind(P, 1, 1, 1, 1); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverChainInitPass(n); });
        int C = 0;
        for (int r = 0; r < 20; r++) { Bind(P, C, C, 1, C); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverChainJumpPass(n); }); C ^= 1; }
        Bind(P, C, C, 1, C); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverJumpCheckPass(n); });
        const int SFinal = C;
        // Chain lengths (into End 0, Jump 0).
        Bind(P, 1, SFinal, 1, SFinal); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverEndInitPass(n); });
        int E = 0;
        for (int r = 0; r < 20; r++) { Bind(P, E, SFinal, E, SFinal); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverEndJumpPass(n); }); E ^= 1; }
        Bind(P, E, SFinal, E, SFinal); For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverJumpCheckPass(n); });
        // Trunks (N1b): on the jump buffers, the tops left in Jump 0 for the segments.
        Bind(P, 0, SFinal, E, SFinal);
        For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverTrunkClearPass(n); });
        For(NC, false, [&](uint32_t n) { B.VoxelRiverTrunkAreaPass(n); });
        For(NC, false, [&](uint32_t n) { B.VoxelRiverTrunkMainPass(n); });
        For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverTrunkInitPass(n); });
        for (int r = 0; r < 20; r++) For(NC, false, [&](uint32_t n) { B.VoxelRiverTrunkJumpPass(n); });   // in place: one thread
        For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverTrunkCheckPass(n); });
        // Segments.
        For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverSegCountPass(n); });
        Scan(NSeg.data(), NC, Counts.data(), 1);
        For(NC, bParallel, [&](uint32_t n) { B.VoxelRiverSegmentPass(n); });
        // Tiles.
        For(SC, bParallel, [&](uint32_t s) { B.VoxelRiverCoverCountPass(s); });
        Scan(TStart.data(), TileCount, Counts.data(), 2);
        For(SC, bParallel, [&](uint32_t s) { B.VoxelRiverCoverFillPass(s); });
        For(TileCount, bParallel, [&](uint32_t t) { B.VoxelRiverTileSortPass(t); });
        Scan(TBlock.data(), TileCount, Counts.data(), 3);
        For(TileCount + 1, bParallel, [&](uint32_t t) { B.VoxelRiverTileWritePass(t); });

        FTables Out;
        Out.TilesPerFace = (int)T;
        Out.Caps = Caps;
        std::memcpy(Out.Counts, Counts.data(), sizeof(Out.Counts));
        const uint32_t Segs = Counts[1];
        const uint64_t Used = (uint64_t)VOXEL_LAND_RIVER_SEG_FLOATS * Segs + Counts[3];
        Out.bFits = Counts[0] <= NC && Segs <= SC && Counts[2] <= B.RiverCandCap && Used <= B.RiverDataCap && Counts[4] == 0;
        Out.Tiles = std::move(Tiles);
        Data.resize(std::min<uint64_t>(Used, Data.size()));
        Out.Data = std::move(Data);
        Out.NumNodes = std::min(Counts[0], NC);
        Out.NumSegments = std::min(Segs, SC);
        SNode.resize(Out.NumSegments); SEnd.resize(Out.NumSegments);
        Out.SegNode = std::move(SNode); Out.SegEnd = std::move(SEnd);
        NCell.resize(Out.NumNodes); NKind.resize(Out.NumNodes); NDown.resize(Out.NumNodes); NUp.resize(Out.NumNodes); NFloor.resize(Out.NumNodes);
        Out.NodeCell = std::move(NCell); Out.NodeKind = std::move(NKind); Out.NodeDown = std::move(NDown); Out.NodeUp = std::move(NUp);
        Out.NodeFloor = std::move(NFloor);
        Out.NodePos = std::move(Pos[P]); Out.NodePos.resize(4 * (size_t)Out.NumNodes);
        Out.NodeS = std::move(SV[SFinal]); Out.NodeS.resize(Out.NumNodes);
        Out.NodeIdx = std::move(IV[SFinal]); Out.NodeIdx.resize(Out.NumNodes);
        return Out;
    }

    /** The build as the plugin runs it: the estimate, again with the counts' sizes until it fits. */
    inline FTables Build(const FSettings& S, const VoxelShader::float2* Tan, const float* Height, const float* Water,
        const uint32_t* FlowHalves, const uint32_t* Rec, bool bParallel = true, const FCaps* Forced = nullptr)
    {
        FCaps Caps = Forced ? *Forced : EstimateCaps(S);
        FTables Out;
        for (int Attempt = 1; Attempt <= 6; Attempt++)   // VoxelLandRiverBuild::MaxBuilds
        {
            Out = BuildOnce(S, Caps, Tan, Height, Water, FlowHalves, Rec, bParallel);
            Out.Builds = Attempt;
            if (Out.bFits || Out.Counts[4] != 0) break;
            // Keep equal to VoxelLandRiverBuild::GrowCaps: a cut stage undercounts the later ones,
            // so they grow by its shortfall too.
            const uint32_t* N = Out.Counts;
            const double NodeShort = std::max(1.0, (double)N[0] / std::max(Caps.Nodes, 1u));
            const double SegShort = std::max(NodeShort, (double)N[1] * NodeShort / std::max(Caps.Segments, 1u));
            const double CandShort = std::max(SegShort, (double)N[2] * SegShort / std::max(Caps.Candidates, 1u));
            const double Used = (double)VOXEL_LAND_RIVER_SEG_FLOATS * N[1] + N[3];
            const double DataShort = std::max(CandShort, Used * CandShort / std::max(Caps.Data, 1u));
            const auto Grow = [](uint32_t Have, double Short) { return Short <= 1.0 ? Have : (uint32_t)std::min(Have * Short * 1.125 + 1024.0, 4294967295.0); };
            Caps.Nodes = Grow(Caps.Nodes, NodeShort);
            Caps.Segments = Grow(Caps.Segments, SegShort);
            Caps.Candidates = Grow(Caps.Candidates, CandShort);
            Caps.Data = Grow(Caps.Data, DataShort);
        }
        return Out;
    }

    /** The drainage as halves, two cells a word (the bake's FFloat16 and the GPU's f32tof16). */
    inline std::vector<uint32_t> PackFlow(const float* Flow, size_t Cells)
    {
        std::vector<uint32_t> Out((Cells + 1) / 2, 0u);
        for (size_t i = 0; i < Cells; i++) Out[i >> 1] |= VoxelShader::f32tof16(Flow[i]) << ((i & 1) * 16);
        return Out;
    }

    /** The bake's tan table (FVoxelLandBake::BuildTanTable). */
    inline std::vector<VoxelShader::float2> TanTable(int N, int Pad)
    {
        std::vector<VoxelShader::float2> Out(N + 2 * Pad);
        for (int i = 0; i < N + 2 * Pad; i++)
        {
            const double V = std::tan((-1.0 + (2.0 * (i - Pad) + 1.0) / N) * 0.78539816339744831);
            const float Hi = (float)V;
            Out[i] = VoxelShader::float2(Hi, (float)(V - (double)Hi));
        }
        return Out;
    }
}
