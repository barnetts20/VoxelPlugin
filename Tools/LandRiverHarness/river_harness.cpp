// River harness (R1b): the rivers' GPU build (VoxelLandRiverBuild.ush, through river_build_cpu.h)
// on a bake dump (the solve harness's dump mode or voxel.Land.ExportBake: Tag_height.f32,
// Tag_water.f32, Tag_flow.f32, Tag_rec.u32), checked against R1's CPU extraction
// (rivers_r1_reference.h) and against itself:
//   - the network: river cells, terminals, outlets, chains, sources, confluences, length;
//   - the floors against a sequential carve over the same nodes (bit for bit), the places along
//     the chains against a sequential walk (double), the decimated lines against R1's;
//   - the lookup (VoxelLandRivers.ush) against a brute-force search over every segment, at points
//     inside the corridors; its continuity across tile and face edges;
//   - the tables' sizes against the first build's estimate.
//
// g++ -O2 -std=c++20 -ffp-contract=off -pthread -I<Public> -I<Shaders/Private> -I. river_harness.cpp
//
// river_harness Tag N RadiusM [MinAreaKm2] [map]: map writes Tag_rivers_b.txt (segment ends'
// face coordinates and drainage, for a plot).
#include "VoxelShaderCompat.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <string>
#include <unordered_map>

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
#include "VoxelDF64.ush"
#include "VoxelCubeSphere.ush"
}
#include "river_build_cpu.h"
#include "rivers_r1_reference.h"

namespace VoxelShader
{
    struct FLookup
    {
        const uint* LandRiverTiles = nullptr;
        const float* LandRiverData = nullptr;
        uint LandN = 1;
        uint LandRiverTilesPerFace = 0;
        float LandRiverRadiusM = 1.0f, LandRiverWidthScale = 1.0f, LandRiverValleyScale = 1.0f;
#include "VoxelLandRivers.ush"
    };
}
using namespace VoxelShader;

template <class T> static std::vector<T> Load(const std::string& Name)
{
    FILE* F = fopen(Name.c_str(), "rb");
    if (!F) { printf("missing %s\n", Name.c_str()); exit(1); }
    fseek(F, 0, SEEK_END); const long L = ftell(F); fseek(F, 0, SEEK_SET);
    std::vector<T> V(L / sizeof(T));
    if (fread(V.data(), 1, L, F) != (size_t)L) { printf("short read %s\n", Name.c_str()); exit(1); }
    fclose(F);
    return V;
}

static double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

struct FD3 { double x, y, z; };
static FD3 Sub(FD3 a, FD3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
static double Dot(FD3 a, FD3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static FD3 Cross(FD3 a, FD3 b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
static FD3 Norm(FD3 a) { const double L = std::sqrt(Dot(a, a)); return { a.x / L, a.y / L, a.z / L }; }
static double Angle(FD3 a, FD3 b) { return std::atan2(std::sqrt(Dot(Cross(a, b), Cross(a, b))), Dot(a, b)); }

/** The distance (radians) from unit P to the great-circle arc A-B, and the place along it. */
static double ArcDistance(FD3 P, FD3 A, FD3 B, double* OutT = nullptr)
{
    const FD3 N = Cross(A, B);
    const double NL = std::sqrt(Dot(N, N));
    if (NL > 1e-15)
    {
        const FD3 Nn = { N.x / NL, N.y / NL, N.z / NL };
        if (Dot(Cross(A, P), Nn) >= 0.0 && Dot(Cross(P, B), Nn) >= 0.0)
        {
            if (OutT)
            {
                const FD3 Q = Norm(Sub(P, { Nn.x * Dot(P, Nn), Nn.y * Dot(P, Nn), Nn.z * Dot(P, Nn) }));
                *OutT = Angle(A, Q) / std::max(Angle(A, B), 1e-300);
            }
            return std::fabs(std::asin(std::clamp(Dot(P, Nn), -1.0, 1.0)));
        }
    }
    const double DA = Angle(P, A), DB = Angle(P, B);
    if (OutT) *OutT = DA <= DB ? 0.0 : 1.0;
    return std::min(DA, DB);
}

static float2 DF(double V) { const float Hi = (float)V; return float2(Hi, (float)(V - (double)Hi)); }

static double Percentile(std::vector<double> V, double Q)
{
    if (V.empty()) return 0.0;
    std::sort(V.begin(), V.end());
    return V[std::min(V.size() - 1, (size_t)(Q * (V.size() - 1) + 0.5))];
}

int main(int argc, char** argv)
{
    if (argc < 4) { printf("river_harness Tag N RadiusM [MinAreaKm2] [map]\n"); return 1; }
    const std::string Tag = argv[1];
    const int N = atoi(argv[2]);
    const double R = atof(argv[3]);
    const double MinArea = argc > 4 ? atof(argv[4]) : 200.0;
    const bool bMap = argc > 5 && std::string(argv[5]) == "map";
    const int Pad = 4;
    const std::vector<float> H = Load<float>(Tag + "_height.f32"), W = Load<float>(Tag + "_water.f32"), Fl = Load<float>(Tag + "_flow.f32");
    const std::vector<uint32_t> Rec = Load<uint32_t>(Tag + "_rec.u32");
    const size_t Cells = H.size();
    const std::vector<uint32_t> Halves = RiverBuildCPU::PackFlow(Fl.data(), Cells);
    const std::vector<float2> Tan = RiverBuildCPU::TanTable(N, Pad);
    uint32_t Land = 0;
    for (size_t c = 0; c < Cells; c++) Land += H[c] > 0.0f;

    RiverBuildCPU::FSettings S;
    S.N = N; S.Pad = Pad; S.RadiusM = R; S.MinAreaKm2 = MinArea; S.LandCells = Land;
    const RiverBuildCPU::FCaps Est = RiverBuildCPU::EstimateCaps(S);
    double T0 = Now();
    const RiverBuildCPU::FTables Tb = RiverBuildCPU::Build(S, Tan.data(), H.data(), W.data(), Halves.data(), Rec.data());
    const double BuildSeconds = Now() - T0;
    const uint32_t* C = Tb.Counts;
    const uint32_t Segs = Tb.NumSegments;
    printf("== R1b build, 6 x %d^2, R %.0f km, from %.0f km^2: %d build(s), %s, errors %u (emulated on the CPU in %.2f s)\n", N, R * 1e-3, MinArea,
        Tb.Builds, Tb.bFits ? "fits" : "DOESN'T FIT", C[4], BuildSeconds);
    printf("network: %u river cells, %u nodes -> %u segments (%.2f nodes a segment); %u chains, %u sources, %u confluences, %u mouths, %u lake inlets, %u outlets; %.0f km\n",
        C[5], C[0], C[1], (double)C[0] / std::max(1u, C[1]), C[6], C[7], C[8], C[9], C[10], C[11], C[12] * 1e-2);
    printf("tiles: %u used of %u, %u candidates (%.2f a segment), %u entries, max %u in a tile, %u dropped; data %.2f MB + tiles %.2f MB\n",
        C[13], 6 * Tb.TilesPerFace * Tb.TilesPerFace, C[2], (double)C[2] / std::max(1u, C[1]), C[16], C[14], C[15], Tb.Data.size() * 4e-6, Tb.Tiles.size() * 4e-6);
    printf("estimate (land %u cells): nodes %u (used %.0f%%), segments %u (%.0f%%), candidates %u (%.0f%%), data %u (%.0f%%)\n", Land,
        Est.Nodes, 100.0 * C[0] / Est.Nodes, Est.Segments, 100.0 * C[1] / Est.Segments, Est.Candidates, 100.0 * C[2] / Est.Candidates,
        Est.Data, 100.0 * (8.0 * C[1] + C[3]) / Est.Data);

    // --- R1's extraction on the same inputs (its flows rounded to halves, as the bake keeps them).
    VoxelLandRiversR1::FInput In;
    In.Grid.N = N; In.Grid.Pad = Pad; In.Grid.RadiusM = R;
    In.Height = H.data(); In.Water = W.data(); In.Receiver = Rec.data();
    In.FlowLog2 = [&](uint32_t c) { return f16tof32((Halves[c >> 1] >> ((c & 1) * 16)) & 0xffffu); };
    VoxelLandRiversR1::FSettings RS;
    RS.MinAreaKm2 = MinArea;
    T0 = Now();
    const VoxelLandRiversR1::FNetwork Ref = VoxelLandRiversR1::Extract(In, RS);
    printf("R1 reference: %u river cells, %zu segments, %u chains, %u sources, %u confluences, %u mouths, %u lake inlets, %u outlets; %.0f km (%.2f s)\n",
        Ref.RiverCells, Ref.Seg.size(), Ref.Chains, Ref.Sources, Ref.Confluences, Ref.Mouths, Ref.LakeInlets, Ref.LakeOutlets, Ref.TotalLengthKm, Now() - T0);
    int Fails = 0;
    auto Expect = [&](bool b, const char* What) { if (!b) { printf("  FAIL: %s\n", What); Fails++; } };
    Expect(C[5] == Ref.RiverCells, "river cells equal R1's");
    Expect(C[9] == Ref.Mouths && C[10] == Ref.LakeInlets && C[11] == Ref.LakeOutlets, "terminals and outlets equal R1's");
    Expect(C[6] == Ref.Chains && C[7] == Ref.Sources && C[8] == Ref.Confluences, "chains, sources and confluences equal R1's");
    Expect(Tb.bFits && C[4] == 0, "the build fits, no jump errors");
    Expect(C[15] == 0, "no tile overflow");

    // --- The floors: a sequential carve over the same nodes (Kahn, as R1), bit for bit.
    const uint32_t NN = Tb.NumNodes;
    {
        std::vector<float> Floor(NN);
        for (uint32_t n = 0; n < NN; n++)
        {
            const uint32_t c = Tb.NodeCell[n], K = Tb.NodeKind[n];
            Floor[n] = K == 0 ? H[c] : (K == 2 ? W[c] : (K == 1 ? std::min(H[c], 0.0f) : W[c]));
        }
        std::vector<uint32_t> Pending(NN, 0), Queue;
        for (uint32_t n = 0; n < NN; n++) if (Tb.NodeDown[n] != ~0u) Pending[Tb.NodeDown[n]]++;
        for (uint32_t n = 0; n < NN; n++) if (Pending[n] == 0) Queue.push_back(n);
        for (size_t q = 0; q < Queue.size(); q++)
        {
            const uint32_t n = Queue[q], D = Tb.NodeDown[n];
            if (D == ~0u) continue;
            if (Tb.NodeKind[D] != 2) Floor[D] = std::min(Floor[D], Floor[n]);
            if (--Pending[D] == 0) Queue.push_back(D);
        }
        uint32_t Differ = 0, Rising = 0;
        for (uint32_t n = 0; n < NN; n++)
        {
            const float G = VoxelShader::FRiverBuildShader().VoxelRiverFloorOf(Tb.NodeFloor[n]);
            Differ += G != Floor[n];
        }
        for (uint32_t s = 0; s < Segs; s++) Rising += Tb.Data[s * VOXEL_LAND_RIVER_SEG_FLOATS + 1] > Tb.Data[s * VOXEL_LAND_RIVER_SEG_FLOATS] + 1e-3f;
        printf("floors: %u of %u nodes differ from the sequential carve; %u segments rise downstream\n", Differ, NN, Rising);
        Expect(Differ == 0 && Rising == 0, "floors exact and monotone");
    }

    // --- R3's joints: each segment against the next in its chain (the one starting at its end,
    // when the chain goes on there): the turns, the grades and the meanders' phase agree.
    {
        const uint32_t F = VOXEL_LAND_RIVER_SEG_FLOATS;
        std::unordered_map<uint32_t, uint32_t> StartsAt;
        for (uint32_t s = 0; s < Segs; s++) StartsAt[Tb.SegNode[s]] = s;
        uint32_t Joints = 0, TurnOff = 0, GradeOff = 0, PhaseOff = 0, Knicks = 0;
        std::vector<double> Turns, Grades, Ratio, Lambda;
        for (uint32_t s = 0; s < Segs; s++)
        {
            const float* A = &Tb.Data[(size_t)s * F];
            const double G = std::max(A[0] - A[1], 0.0f) / std::max(A[5], 1.0f);
            Grades.push_back(G);
            if (A[13] > 0.0f) Lambda.push_back(A[5] / A[13]);
            const double Nb = std::max(std::max((double)A[10], (double)A[11]), 0.002);
            Ratio.push_back(G / Nb);
            Knicks += G > 0.02 && G > 3.0 * Nb;
            auto It = StartsAt.find(Tb.SegEnd[s]);
            if (It == StartsAt.end()) continue;
            const float* B = &Tb.Data[(size_t)It->second * F];
            // The chain goes on when the next one's place starts at this one's end.
            if (std::fabs(B[4] - (A[4] + A[5])) > 0.5f + 1e-4f * B[4]) continue;
            Joints++;
            Turns.push_back(std::fabs(A[9]));
            TurnOff += A[9] != B[8];
            GradeOff += std::fabs(B[10] - G) > 1e-6 + 1e-4 * G || std::fabs(A[11] - std::max(B[0] - B[1], 0.0f) / std::max(B[5], 1.0f)) > 1e-6 + 1e-4 * G;
            const double Ph = A[12] + A[13];
            PhaseOff += std::fabs((Ph - std::floor(Ph)) - B[12]) > 2e-3 && std::fabs(std::fabs((Ph - std::floor(Ph)) - B[12]) - 1.0) > 2e-3;
        }
        printf("joints in chains: %u; turns differing %u, grades %u, phases %u; turn p50 %.1f p90 %.1f p99 %.1f deg; grade p50 %.4f p90 %.4f p99 %.4f; "
            "over the neighbours' p99 %.1f, knickpoints (grade over 2%% and 3x both neighbours') %u; meander wavelength p50 %.0f m\n",
            Joints, TurnOff, GradeOff, PhaseOff, Percentile(Turns, 0.5) * 57.2958, Percentile(Turns, 0.9) * 57.2958, Percentile(Turns, 0.99) * 57.2958,
            Percentile(Grades, 0.5), Percentile(Grades, 0.9), Percentile(Grades, 0.99), Percentile(Ratio, 0.99), Knicks, Percentile(Lambda, 0.5));
        Expect(TurnOff == 0 && GradeOff == 0 && PhaseOff == 0, "joints agree (turns, grades, phase)");
    }

    // --- The places along the chains: a sequential walk (double) from each chain's start.
    auto NodeDir = [&](uint32_t n) { return Norm({ (double)Tb.NodePos[4 * n], (double)Tb.NodePos[4 * n + 1], (double)Tb.NodePos[4 * n + 2] }); };
    {
        std::vector<uint32_t> Pred(NN, ~0u);
        for (uint32_t n = 0; n < NN; n++) if (Tb.NodeDown[n] != ~0u) Pred[Tb.NodeDown[n]] = n;
        double WorstS = 0.0;
        uint32_t BadIdx = 0;
        for (uint32_t n = 0; n < NN; n++)
        {
            if (Tb.NodeUp[n] == 1) continue;   // chain starts only
            double Sv = 0.0;
            uint32_t Idx = 0, m = n;
            for (;;)
            {
                WorstS = std::max(WorstS, std::fabs(Sv - Tb.NodeS[m]) / std::max(1.0, Sv));
                BadIdx += Idx != Tb.NodeIdx[m];
                const uint32_t D = Tb.NodeDown[m];
                if (D == ~0u || Tb.NodeUp[D] != 1) break;
                Sv += Angle(NodeDir(m), NodeDir(D)) * R;
                Idx++;
                m = D;
            }
        }
        printf("places along the chains: worst relative error %.2e, %u wrong indices\n", WorstS, BadIdx);
        Expect(WorstS < 1e-4 && BadIdx == 0, "places along the chains");
    }

    // --- The decimated lines against R1's: points along each segment, their distance to R1's
    // nearest segment (a cell hash of R1's segments).
    {
        VoxelLandRiversR1::FGrid G{ N, Pad, R };
        std::unordered_map<uint64_t, std::vector<uint32_t>> Hash;
        auto Key = [&](const double D[3]) { int F; double X, Y; G.Locate(D, F, X, Y); return ((uint64_t)F << 40) | ((uint64_t)(int)std::floor(X / 2 + 1000) << 20) | (uint64_t)(int)std::floor(Y / 2 + 1000); };
        for (size_t s = 0; s < Ref.Seg.size(); s++)
            for (int k = 0; k <= 4; k++)
            {
                const double* A = &Ref.Node[3 * Ref.SegA[s]];
                const double* B = &Ref.Node[3 * Ref.SegB[s]];
                double P[3] = { A[0] + (B[0] - A[0]) * k / 4, A[1] + (B[1] - A[1]) * k / 4, A[2] + (B[2] - A[2]) * k / 4 };
                auto& V = Hash[Key(P)];
                if (std::find(V.begin(), V.end(), (uint32_t)s) == V.end()) V.push_back((uint32_t)s);
            }
        std::vector<double> Off;
        const double Cell = G.CellArcM();
        for (uint32_t s = 0; s < Segs; s += 3)
        {
            const FD3 A = NodeDir(Tb.SegNode[s]), B = NodeDir(Tb.SegEnd[s]);
            const FD3 P = Norm({ (A.x + B.x) * 0.5, (A.y + B.y) * 0.5, (A.z + B.z) * 0.5 });
            const double PD[3] = { P.x, P.y, P.z };
            int F; double X, Y; G.Locate(PD, F, X, Y);
            double Best = 1e30;
            for (int dx = -1; dx <= 1; dx++)
                for (int dy = -1; dy <= 1; dy++)
                {
                    const uint64_t K = ((uint64_t)F << 40) | ((uint64_t)((int)std::floor(X / 2 + 1000) + dx) << 20) | (uint64_t)((int)std::floor(Y / 2 + 1000) + dy);
                    auto It = Hash.find(K);
                    if (It == Hash.end()) continue;
                    for (uint32_t r : It->second)
                    {
                        const double* RA = &Ref.Node[3 * Ref.SegA[r]];
                        const double* RB = &Ref.Node[3 * Ref.SegB[r]];
                        Best = std::min(Best, ArcDistance(P, { RA[0], RA[1], RA[2] }, { RB[0], RB[1], RB[2] }) * R);
                    }
                }
            if (Best < 1e29) Off.push_back(Best / Cell);
        }
        printf("lines against R1's (segment midpoints, cells): p50 %.3f, p90 %.3f, p99 %.3f, max %.3f (%zu points)\n",
            Percentile(Off, 0.5), Percentile(Off, 0.9), Percentile(Off, 0.99), Percentile(Off, 1.0), Off.size());
    }

    // --- The lookup against a brute-force search, inside the corridors.
    FLookup L;
    L.LandRiverTiles = Tb.Tiles.data(); L.LandRiverData = Tb.Data.data(); L.LandN = (uint)N;
    L.LandRiverTilesPerFace = (uint)Tb.TilesPerFace; L.LandRiverRadiusM = (float)R;
    auto Reach = [&](uint32_t s) {
        const double LA = std::max(Tb.Data[s * VOXEL_LAND_RIVER_SEG_FLOATS + 2], Tb.Data[s * VOXEL_LAND_RIVER_SEG_FLOATS + 3]);
        return 1.25 * (400.0 + 15.0 * 2.5 * std::sqrt(std::exp2(LA) * 1e-6));
    };
    auto Lookup = [&](FD3 P) { return L.VoxelLandRiverAtDF(DF(P.x * R), DF(P.y * R), DF(P.z * R)); };
    {
        std::mt19937 Rng(5);
        std::vector<double> Diff;
        int Wrong = 0, Missing = 0, Points = 0;
        double Entries = 0.0;
        VoxelLandRiversR1::FGrid G{ N, Pad, R };
        std::vector<FD3> SegA(Segs), SegB(Segs);
        for (uint32_t q = 0; q < Segs; q++) { SegA[q] = NodeDir(Tb.SegNode[q]); SegB[q] = NodeDir(Tb.SegEnd[q]); }
        for (int t = 0; t < 4000; t++)
        {
            const uint32_t s = Rng() % Segs;
            const FD3 A = NodeDir(Tb.SegNode[s]), B = NodeDir(Tb.SegEnd[s]);
            const double U = std::uniform_real_distribution<double>(0, 1)(Rng);
            FD3 P = Norm({ A.x + (B.x - A.x) * U, A.y + (B.y - A.y) * U, A.z + (B.z - A.z) * U });
            // A random offset inside the corridor.
            const FD3 T1 = Norm(Cross(P, Sub(B, A)));
            const double Off = std::uniform_real_distribution<double>(-1, 1)(Rng) * Reach(s) * 0.95 / R;
            P = Norm({ P.x + T1.x * Off, P.y + T1.y * Off, P.z + T1.z * Off });
            // Every segment: the nearest, and the nearest whose corridor holds the point (its
            // tile must list that one, so the lookup is at most that far and at least the nearest).
            double Best = 1e30, BestIn = 1e30;
            uint32_t BestS = ~0u;
            for (uint32_t q = 0; q < Segs; q++)
            {
                const double D = ArcDistance(P, SegA[q], SegB[q]) * R;
                if (D < Best) { Best = D; BestS = q; }
                if (D <= Reach(q) && D < BestIn) BestIn = D;
            }
            const FLookup::FVoxelLandRiver Rv = Lookup(P);
            Points++;
            const uint32_t Tile = L.VoxelLandRiverTileOf(float3((float)P.x, (float)P.y, (float)P.z));
            if (Tb.Tiles[Tile + 1] > Tb.Tiles[Tile]) Entries += (Tb.Tiles[Tile + 1] - Tb.Tiles[Tile] - 12) / 7.0;
            if (!Rv.bFound) { Missing++; continue; }
            const double Dd = std::fabs((double)Rv.Distance - BestIn);
            Diff.push_back(Dd);
            if (Dd > 0.05 + 2e-4 * BestIn)
            {
                Wrong++;
                bool bListedS = false, bListedBest = false;
                for (uint32_t At = Tb.Tiles[Tile] + 12; At < Tb.Tiles[Tile + 1]; At += 7)
                {
                    bListedS |= (uint32_t)Tb.Data[At + 6] == s;
                    bListedBest |= (uint32_t)Tb.Data[At + 6] == BestS;
                }
                const double PD[3] = { P.x, P.y, P.z };
                int F; double X, Y; G.Locate(PD, F, X, Y);
                printf("  wrong: lookup %.2f m (segment %u), brute %.2f m (segment %u, reach %.0f), drawn from %u (reach %.0f, at %.2f m); tile %u (face %d at %.2f, %.2f) lists them: %d %d\n",
                    (double)Rv.Distance, Rv.Segment, Best, BestS, BestS != ~0u ? Reach(BestS) : 0.0, s, Reach(s),
                    ArcDistance(P, NodeDir(Tb.SegNode[s]), NodeDir(Tb.SegEnd[s])) * R, Tile, F, X, Y, bListedS ? 1 : 0, bListedBest ? 1 : 0);
            }
        }
        printf("lookup against brute force at %d points inside the corridors (against the nearest whose corridor holds the point): |difference| p50 %.4f m, p99 %.4f m, max %.3f m; wrong (beyond 5 cm + 2e-4 of the distance): %d; nothing found: %d; %.1f entries a lookup\n",
            Points, Percentile(Diff, 0.5), Percentile(Diff, 0.99), Percentile(Diff, 1.0), Wrong, Missing, Entries / Points);
        Expect(Missing == 0 && Wrong <= Points / 1000, "lookup equals brute force");
    }

    // --- Continuity: 1 m steps along lines across the tiles near rivers.
    {
        std::mt19937 Rng(9);
        double WorstJump = 0.0, WorstNear = 0.0, WorstMid = 0.0;
        int Crossings = 0;
        // Lines two tiles long through random segments' starts, along random directions.
        const int Half = (int)(VoxelLandRiversR1::FGrid{ N, Pad, R }.CellArcM() * 8.0);
        for (int t = 0; t < 100; t++)
        {
            const uint32_t s = Rng() % Segs;
            const FD3 A = NodeDir(Tb.SegNode[s]);
            const double Ph = std::uniform_real_distribution<double>(0, 6.283)(Rng);
            const FD3 E1 = Norm(Cross(A, { 0.3, 0.7, -0.2 })), E2 = Cross(A, E1);
            const FD3 Dir = { E1.x * std::cos(Ph) + E2.x * std::sin(Ph), E1.y * std::cos(Ph) + E2.y * std::sin(Ph), E1.z * std::cos(Ph) + E2.z * std::sin(Ph) };
            double Prev = -1.0;
            uint32_t PrevTile = ~0u;
            for (int k = -Half; k <= Half; k++)
            {
                const double Ang = k * 1.0 / R;
                const FD3 P = Norm({ A.x + Dir.x * Ang, A.y + Dir.y * Ang, A.z + Dir.z * Ang });
                const FLookup::FVoxelLandRiver Rv = Lookup(P);
                const uint32_t Tile = L.VoxelLandRiverTileOf(float3((float)P.x, (float)P.y, (float)P.z));
                if (Rv.bFound && Prev >= 0.0)
                {
                    const double J = std::fabs(Rv.Distance - Prev);
                    if (Tile != PrevTile)
                    {
                        Crossings++;
                        WorstJump = std::max(WorstJump, J);
                        if (Rv.Distance < 2000.0) WorstNear = std::max(WorstNear, J);
                        else if (Rv.Distance < 5000.0) WorstMid = std::max(WorstMid, J);
                    }
                }
                Prev = Rv.bFound ? Rv.Distance : -1.0;
                PrevTile = Tile;
            }
        }
        printf("continuity: %d tile crossings found on both sides, worst distance change over a 1 m step there %.4f m (within 2 km of the river %.4f, 2-5 km %.4f)\n", Crossings, WorstJump, WorstNear, WorstMid);
        Expect(WorstJump < 1.05, "continuous across tiles");
    }

    // --- N1b's trunks: at every confluence exactly one inflow carries on the outflow's key (the
    // largest), and a chain's segments share theirs.
    {
        const uint32_t F = VOXEL_LAND_RIVER_SEG_FLOATS;
        auto KeyOf = [&](uint32_t s) { return (uint32_t)Tb.Data[(size_t)s * F + 7] >> VOXEL_LAND_RIVER_TRUNK_SHIFT; };
        std::unordered_map<uint32_t, uint32_t> StartsAt;
        std::unordered_map<uint32_t, std::vector<uint32_t>> EndsAt;
        for (uint32_t s = 0; s < Segs; s++) { StartsAt[Tb.SegNode[s]] = s; EndsAt[Tb.SegEnd[s]].push_back(s); }
        uint32_t Confluences = 0, BadCarry = 0, NotLargest = 0, ChainBreaks = 0;
        std::unordered_map<uint32_t, int> Keys;
        for (uint32_t s = 0; s < Segs; s++) Keys[KeyOf(s)]++;
        for (const auto& [Node, Ins] : EndsAt)
        {
            const auto Out = StartsAt.find(Node);
            if (Out == StartsAt.end()) continue;
            if (Ins.size() == 1) { ChainBreaks += KeyOf(Ins[0]) != KeyOf(Out->second); continue; }
            Confluences++;
            int Carry = 0; float Largest = -1e30f, Carried = -1e30f;
            for (uint32_t In : Ins)
            {
                const float A = Tb.Data[(size_t)In * F + 3];
                Largest = std::max(Largest, A);
                if (KeyOf(In) == KeyOf(Out->second)) { Carry++; Carried = A; }
            }
            BadCarry += Carry != 1;
            NotLargest += Carry == 1 && Carried < Largest;
        }
        printf("trunks: %zu keys over %u segments; %u confluences: %u without exactly one inflow carrying on, %u carried by a smaller inflow; %u chain joints changing key\n",
            Keys.size(), Segs, Confluences, BadCarry, NotLargest, ChainBreaks);
        Expect(BadCarry == 0 && NotLargest == 0 && ChainBreaks == 0, "trunk keys");
    }
    if (bMap)
    {
        VoxelLandRiversR1::FGrid G{ N, Pad, R };
        FILE* F = fopen((Tag + "_rivers_b.txt").c_str(), "w");
        for (uint32_t s = 0; s < Segs; s++)
        {
            const FD3 A = NodeDir(Tb.SegNode[s]), B = NodeDir(Tb.SegEnd[s]);
            const double DA[3] = { A.x, A.y, A.z }, DB[3] = { B.x, B.y, B.z };
            int fa, fb; double xa, ya, xb, yb;
            G.Locate(DA, fa, xa, ya); G.Locate(DB, fb, xb, yb);
            fprintf(F, "%d %.3f %.3f %d %.3f %.3f %.3f %u\n", fa, xa, ya, fb, xb, yb, std::max(Tb.Data[s * VOXEL_LAND_RIVER_SEG_FLOATS + 2], Tb.Data[s * VOXEL_LAND_RIVER_SEG_FLOATS + 3]), (uint32_t)Tb.Data[s * VOXEL_LAND_RIVER_SEG_FLOATS + 7]);
        }
        fclose(F);
    }
    printf(Fails == 0 ? "all checks pass\n" : "%d checks FAILED\n", Fails);
    return Fails == 0 ? 0 : 1;
}
