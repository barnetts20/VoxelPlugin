// R2's carve (Shaders/Private/VoxelLandCarve.ush: the rivers' valleys and channels, the lakes'
// floors and shores) on a dumped bake with its rivers (gully_harness's planets: a dump with the
// receivers, the rivers built as the GPU builds them). Checks:
//   - centre: along the rivers' lines, the carved surface against the bed (floor less the
//     channel's depth), and how far the uncarved one stood above the floor;
//   - continuity: long lines through river corridors and lake shores, sampled every 0.5 m:
//     steps (a sample pair changing far more than its neighbours) and whether they straddle a
//     river tile's edge or a face edge;
//   - gradient: the land surface cache's gradient (VoxelLandCacheWriter.ush: the carve's scale
//     and own gradient on the terms' exact and differenced ones) against central differences
//     of the height, near rivers and lakes, carve on and off;
//   - backends: the global shaders' chain (VoxelLandHeightDF) against the CPU density's
//     (FVoxelDensityCPU::LandHeightAt, as gully_harness's Terms);
//   - cost: the CPU sample's time with and without the carve.
// Views: hillshades around rivers of a few sizes and a lake shore, carve on and off.
//
// g++ -O2 -std=c++20 -ffp-contract=off -pthread -I<Public> -I<Private> -I<Shaders/Private> carve_harness.cpp
// carve_harness Tag N RadiusM [check|views|cost]   (Tag_*.f32 and Tag_rec.u32 from `solve_harness N Tag dump RadiusM`)
#include <map>
#include <cstdlib>
// The valley walls' meeting (N1b), switched by `meet` views.
static float GMeetRound = getenv("MEET") ? (float)atof(getenv("MEET")) : 1.0f;
#define VOXEL_LAND_VALLEY_MEET_ROUND (GMeetRound)
#define main gully_main
#include "gully_harness.cpp"
#undef main

namespace VoxelShader
{
#include "VoxelLandCacheWriter.ush"
}

/** gully_harness's Terms with a given sampler (the carve switched by the caller). */
static FTerms TermsWith(const FPlanet& P, FSampler S, const double D[3], double Footprint)
{
    double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius };
    double Q[3], DW[3][3];
    for (int a = 0; a < 3; a++)
    {
        Q[a] = SP[a] + Evaluate(P.W[a], SP, 0.0, DW[a]) * P.WarpAmp;
        for (int b = 0; b < 3; b++) DW[a][b] *= P.WarpAmp;
    }
    const auto L = S.VoxelLandSurfaceDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), DF(Q[0]), DF(Q[1]), DF(Q[2]),
        float3((float)DW[0][0], (float)DW[0][1], (float)DW[0][2]), float3((float)DW[1][0], (float)DW[1][1], (float)DW[1][2]),
        float3((float)DW[2][0], (float)DW[2][1], (float)DW[2][2]), (float)std::max(Footprint, P.Finest), false);
    const float Wd = lerp(0.25f, 1.0f, smoothstep(-50.0f, 50.0f, L.Bake));
    const double Dd = (double)S.VoxelLandDetailDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), (float)std::max(Footprint, P.Finest), false).Unit;
    FTerms T;
    T.Bake = L.Bake; T.Hills = L.Hills; T.Peaks = L.Peaks; T.Gully = L.Gully; T.DetailScale = L.DetailScale; T.Carve = L.Carve;
    T.Detail = Dd * (double)((float)P.DetailAmp * Wd * L.DetailScale) * 0.01;
    T.Height = T.Bake + T.Hills + T.Peaks + T.Gully + T.Carve + T.Detail;
    return T;
}

/** The warped point's direction at D (unit). */
static void Warped(const FPlanet& P, const double D[3], double Out[3])
{
    double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius };
    for (int a = 0; a < 3; a++) Out[a] = SP[a] + Evaluate(P.W[a], SP, 0.0) * P.WarpAmp;
    Normalize(Out);
}

/** The point (unit direction) whose warped point lies in direction Target: fixed-point iteration
 *  on P = Target - W(P) (the warp moves 1.5 cells and changes over 6: it converges fast). */
static void Unwarp(const FPlanet& P, const double Target[3], double Out[3])
{
    // The direction D whose warped point's direction is Target: correct D by the miss each round
    // (the warp's Jacobian is near the identity). A fixed point of D = Target R - w(D R) instead
    // misses by about w's radial share times its tangential one (tens of metres at 1.5 cells).
    // Where the warp's Jacobian is far from it the step is halved until the miss shrinks; the
    // best point is kept.
    auto MissAt = [&](const double* X, double* E) {
        double SP[3] = { X[0] * P.SeaRadius, X[1] * P.SeaRadius, X[2] * P.SeaRadius }, W[3];
        for (int a = 0; a < 3; a++) W[a] = SP[a] + Evaluate(P.W[a], SP, 0.0) * P.WarpAmp;
        Normalize(W);
        double M = 0;
        for (int a = 0; a < 3; a++) { E[a] = Target[a] - W[a]; M += E[a] * E[a]; }
        return M;
    };
    double D[3] = { Target[0], Target[1], Target[2] }, E[3];
    double Miss = MissAt(D, E), Step = 1.0;
    for (int it = 0; it < 96 && Miss * P.SeaRadius * P.SeaRadius > 1.0e-6; it++)   // to 0.01 mm (cm units)
    {
        double X[3], EX[3];
        for (int a = 0; a < 3; a++) X[a] = D[a] + Step * E[a];
        Normalize(X);
        const double MX = MissAt(X, EX);
        if (MX < Miss) { for (int a = 0; a < 3; a++) { D[a] = X[a]; E[a] = EX[a]; } Miss = MX; Step = std::min(1.0, Step * 2.0); }
        else Step *= 0.5;
    }
    for (int a = 0; a < 3; a++) Out[a] = D[a];
}

static void Cross(const double A[3], const double B[3], double O[3])
{
    O[0] = A[1] * B[2] - A[2] * B[1]; O[1] = A[2] * B[0] - A[0] * B[2]; O[2] = A[0] * B[1] - A[1] * B[0];
}

/** The point Dist m from D (unit) along the tangent E (unit, tangent at D). */
static void Move(const FPlanet& P, const double D[3], const double E[3], double Dist, double Out[3])
{
    const double A = Dist * 100.0 / P.SeaRadius;
    for (int a = 0; a < 3; a++) Out[a] = D[a] * std::cos(A) + E[a] * std::sin(A);
    Normalize(Out);
}

struct FRiverSite
{
    double Q[3];        // on the line (the warped space)
    double Along[3];    // downstream (tangent)
    double Side[3];
    double Floor = 0, Log2Area = 0, Width = 0, Valley = 0, Depth = 0;
    uint32_t Node = 0;
    uint32_t Seg = 0;   // the lookup's segment there
};

/** River sites: the midpoints of the segments between consecutive nodes (every Stride-th node). */
static std::vector<FRiverSite> Sites(const FPlanet& P, uint32_t Stride)
{
    FSampler Smp = P.S;
    std::vector<FRiverSite> Out;
    const auto& R = P.Rivers;
    for (uint32_t n = 0; n < R.NumNodes; n += Stride)
    {
        const uint32_t Down = R.NodeDown[n];
        if (R.NodeKind[n] != 0 || Down >= R.NumNodes) continue;
        FRiverSite S;
        S.Node = n;
        double A[3], B[3];
        for (int a = 0; a < 3; a++) { A[a] = R.NodePos[4 * n + a]; B[a] = R.NodePos[4 * Down + a]; }
        for (int a = 0; a < 3; a++) S.Q[a] = 0.5 * (A[a] + B[a]);
        Normalize(S.Q);
        double T[3] = { B[0] - A[0], B[1] - A[1], B[2] - A[2] };
        const double Dot = T[0] * S.Q[0] + T[1] * S.Q[1] + T[2] * S.Q[2];
        for (int a = 0; a < 3; a++) T[a] -= Dot * S.Q[a];
        Normalize(T);
        for (int a = 0; a < 3; a++) S.Along[a] = T[a];
        Cross(S.Q, S.Along, S.Side);
        const auto L = Smp.VoxelLandRiverAtDF(DF(S.Q[0] * P.SeaRadius), DF(S.Q[1] * P.SeaRadius), DF(S.Q[2] * P.SeaRadius));
        // The lines are the decimated chains (R1b keeps an odd node only where it's needed), so a
        // node pair's midpoint can stand up to a few hundred metres off them: on a line only.
        if (!L.bFound || L.Distance > 1.0f) continue;
        S.Floor = L.Floor; S.Log2Area = L.Log2Area; S.Seg = L.Segment;
        S.Width = Smp.VoxelLandRiverWidth(L.Log2Area);
        S.Valley = Smp.VoxelLandRiverValleyHalfWidth(Smp.VoxelLandRiverBelt(L.Log2Area));
        S.Depth = Smp.VoxelLandRiverDepth(L.Log2Area);
        Out.push_back(S);
    }
    return Out;
}

static double Pct(std::vector<double> V, double Q)
{
    if (V.empty()) return 0;
    std::sort(V.begin(), V.end());
    return V[std::min((size_t)(Q * V.size()), V.size() - 1)];
}

/** Steps along a sampled line: a pair whose change is far beyond its neighbours'. */
struct FSteps
{
    int Count = 0;
    double Max = 0;
    int AtTileEdge = 0, AtFaceEdge = 0;
    int TileEdges = 0, FaceEdges = 0;     // crossed
    double MaxAtTileEdge = 0;             // the largest excess change at a crossing (m)
    std::vector<double> Excess;           // every pair's
};

static uint32_t TileOf(const FPlanet& P, const double D[3])
{
    double Q[3]; Warped(P, D, Q);
    FSampler Smp = P.S;
    return Smp.VoxelLandRiverTileOf(float3((float)Q[0], (float)Q[1], (float)Q[2]));
}

static int FaceOf(const double* D)
{
    const double X = std::fabs(D[0]), Y = std::fabs(D[1]), Z = std::fabs(D[2]);
    return X >= Y && X >= Z ? (D[0] > 0 ? 0 : 1) : (Y >= Z ? (D[1] > 0 ? 2 : 3) : (D[2] > 0 ? 4 : 5));
}

static void FindSteps(const FPlanet& P, const std::vector<std::array<double, 3>>& Dirs, const std::vector<double>& H, FSteps& Out, double Floor)
{
    std::vector<uint32_t> Tiles(Dirs.size());
    ParallelFor((int)Dirs.size(), [&](int k) { Tiles[k] = TileOf(P, Dirs[k].data()); });
    for (size_t i = 1; i + 2 < H.size(); i++)
    {
        const double D0 = H[i] - H[i - 1], D1 = H[i + 1] - H[i], D2 = H[i + 2] - H[i + 1];
        const double Expect = 0.5 * (D0 + D2);
        const double Excess = std::fabs(D1 - Expect);
        Out.Excess.push_back(Excess);
        if (Tiles[i] != Tiles[i + 1])
        {
            Out.TileEdges++;
            Out.MaxAtTileEdge = std::max(Out.MaxAtTileEdge, Excess);
        }
        double Wa[3], Wb[3]; Warped(P, Dirs[i].data(), Wa); Warped(P, Dirs[i + 1].data(), Wb);
        if (FaceOf(Wa) != FaceOf(Wb)) Out.FaceEdges++;
        if (Excess > Floor && std::fabs(D1) > 3.0 * std::max(std::fabs(D0), std::fabs(D2)))
        {
            Out.Count++;
            Out.Max = std::max(Out.Max, Excess);
            if (getenv("STEP_PRINT"))
                printf("    step %.3f m at %zu (dir %.9f %.9f %.9f): %.3f %.3f %.3f %.3f\n", Excess, i, Dirs[i][0], Dirs[i][1], Dirs[i][2], H[i - 1], H[i], H[i + 1], H[i + 2]);
            if (Tiles[i] != Tiles[i + 1]) Out.AtTileEdge++;
            if (FaceOf(Wa) != FaceOf(Wb)) Out.AtFaceEdge++;
        }
    }
}

/** A bake cell's direction (the solve's: the tan table). */
static void CellDir(const FPlanet& P, int c, double Out[3])
{
    const int Side = P.Side();
    const int f = c / (Side * Side), r = c % (Side * Side), j = r / Side - P.Pad, i = r % Side - P.Pad;
    const float2 TA = P.Tan[i + P.Pad], TB = P.Tan[j + P.Pad];
    const float3 Nf = VoxelCubeAxisVector(VoxelCubeNormalAxis(f)) * VoxelCubeNormalSign(f);
    const float3 Uf = VoxelCubeAxisVector(VoxelCubeUAxis(f)), Vf = VoxelCubeAxisVector(VoxelCubeVAxis(f));
    Out[0] = Nf.x + Uf.x * (TA.x + TA.y) + Vf.x * (TB.x + TB.y);
    Out[1] = Nf.y + Uf.y * (TA.x + TA.y) + Vf.y * (TB.x + TB.y);
    Out[2] = Nf.z + Uf.z * (TA.x + TA.y) + Vf.z * (TB.x + TB.y);
    Normalize(Out);
}

/** The lakes' edge cells (a lake cell beside a dry one). */
static std::vector<int> LakeEdges(const FPlanet& P)
{
    const int Side = P.Side(), N = P.N;
    std::vector<int> Edge;
    if (P.WaterF.size() != P.Height.size()) return Edge;
    for (int f = 0; f < 6; f++)
        for (int j = 1; j < N - 1; j++)
            for (int i = 1; i < N - 1; i++)
            {
                const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                if (!(P.WaterF[c] > -1e30f)) continue;
                if (!(P.WaterF[c + 1] > -1e30f) || !(P.WaterF[c - 1] > -1e30f) || !(P.WaterF[c + Side] > -1e30f) || !(P.WaterF[c - Side] > -1e30f)) Edge.push_back(c);
            }
    return Edge;
}

int main(int argc, char** argv)
{
    const std::string Tag = argc > 1 ? argv[1] : "r1";
    const int N = argc > 2 ? atoi(argv[2]) : 512;
    const double RadiusM = argc > 3 ? atof(argv[3]) : 1.0e6;
    const std::string Mode = argc > 4 ? argv[4] : "check";
    FPlanet P;
    Load(P, Tag, N, RadiusM, 30.0, HarnessGullyStrength());
    if (P.Rivers.Tiles.empty()) { printf("needs a dump with the receivers (Tag_rec.u32)\n"); return 1; }
    FSampler Off = P.S;
    Off.LandRiverCarve = 0.0f; Off.LandHasWater = 0;
    std::vector<FVoxelOctave> Gpu;
    SetGpuGlobals(P, Gpu);
    const std::vector<FRiverSite> All = Sites(P, 1);
    printf("%zu river sites (segment midpoints)\n", All.size());
    std::mt19937 Rng(41);

    if (Mode == "warpjac")
    {
        // The sampling warp's Jacobian on the sphere (tangent plane): its smaller singular value
        // (under 1: the warp squeezes there, so the carve's walls steepen by its inverse; 0: a
        // fold) and its determinant (negative: folded).
        std::mt19937 Rg(3);
        std::normal_distribution<double> G(0.0, 1.0);
        std::vector<double> Smin, Det;
        for (int k = 0; k < 200000; k++)
        {
            double D[3] = { G(Rg), G(Rg), G(Rg) }; Normalize(D);
            double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius }, DW[3][3];
            for (int a = 0; a < 3; a++) { Evaluate(P.W[a], SP, 0.0, DW[a]); for (int b = 0; b < 3; b++) DW[a][b] *= P.WarpAmp; }
            double E1[3], E2[3]; Frame(D, E1, E2);
            double J[2][2];
            for (int r = 0; r < 2; r++)
                for (int c = 0; c < 2; c++)
                {
                    const double* Er = r == 0 ? E1 : E2; const double* Ec = c == 0 ? E1 : E2;
                    double V = 0;
                    for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++) V += Er[a] * DW[a][b] * Ec[b];
                    J[r][c] = (r == c ? 1.0 : 0.0) + V;
                }
            const double Dt = J[0][0] * J[1][1] - J[0][1] * J[1][0];
            const double F2 = J[0][0] * J[0][0] + J[0][1] * J[0][1] + J[1][0] * J[1][0] + J[1][1] * J[1][1];
            const double Sm = std::sqrt(std::max(0.5 * (F2 - std::sqrt(std::max(F2 * F2 - 4.0 * Dt * Dt, 0.0))), 0.0));
            Smin.push_back(Sm); Det.push_back(Dt);
        }
        int Folded = 0, Squeezed = 0;
        for (double d : Det) Folded += d < 0;
        for (double v : Smin) Squeezed += v < 0.5;
        printf("warp (%.2f cells): smaller singular value p1 %.3f p5 %.3f p50 %.3f; %.1f%% squeezed under half, %.1f%% folded\n", P.WarpAmp / P.Cell,
            Pct(Smin, 0.01), Pct(Smin, 0.05), Pct(Smin, 0.5), 100.0 * Squeezed / Smin.size(), 100.0 * Folded / Det.size());
        return 0;
    }
    if (Mode == "slots")
    {
        // Cross-sections over the channels (every 13th river site, all sizes, the fade-in's too),
        // at the finest footprint and at coarser LODs: the channel's incision below its banks
        // (the lower of the highest ground 1..1.5 channel widths either side, less the lowest
        // within a width of the centre) over the channel's depth, and the steepest slope the carve
        // makes (over a metre, where it lowers the ground by over half a metre) against the
        // uncarved surface's there.
        for (double Fm : { P.Finest * 0.01, 25.0, 100.0 })
        {
            std::vector<double> Incision, Slope;
            int Slots = 0, Steep = 0, VerySteep = 0, Sections = 0;
            double WorstIncision = 0, WorstSlope = 0; int Printed = 0, PrintedSlots = 0;
            for (size_t i = 0; i < All.size(); i += 13)
            {
                const FRiverSite& S = All[i];
                if (S.Floor < 20) continue;
                double C[3]; Unwarp(P, S.Q, C);
                const double W = std::max(S.Width, 4.0);
                const double Reach = 3.0 * W + 40.0;
                const int Count = (int)(2.0 * Reach / 0.5) + 1;
                std::vector<double> H(Count), H0(Count);
                ParallelFor(Count, [&](int k) {
                    double D[3]; Move(P, C, S.Side, -Reach + k * 0.5, D);
                    H[k] = TermsWith(P, P.S, D, std::max(Fm * 100.0, P.Finest)).Height;
                    H0[k] = TermsWith(P, Off, D, std::max(Fm * 100.0, P.Finest)).Height;
                });
                Sections++;
                double Low = 1e30, BankL = -1e30, BankR = -1e30;
                for (int k = 0; k < Count; k++)
                {
                    const double X = -Reach + k * 0.5;
                    if (std::fabs(X) <= W) Low = std::min(Low, H[k]);
                    if (X <= -W && X >= -1.5 * W) BankL = std::max(BankL, H[k]);
                    if (X >= W && X <= 1.5 * W) BankR = std::max(BankR, H[k]);
                }
                const double Inc = (std::min(BankL, BankR) - Low) / std::max(S.Depth, 0.5);
                Incision.push_back(Inc);
                if (Inc > 3.0) Slots++;
                if (Inc > 8.0 && Fm < 10.0 && PrintedSlots++ < 5)
                {
                    const auto R = FSampler(P.S).VoxelLandRiverSegAt(S.Seg);
                    printf("  slot %.1f: area %.0f km^2, width %.0f, depth %.1f, floor %.0f, grade %.4f; across (m: carved/uncarved):", Inc, std::exp2(S.Log2Area) * 1e-6, S.Width, S.Depth,
                        S.Floor, R.Grade);
                    for (int k = 0; k < Count; k += std::max(Count / 24, 1)) printf(" %.0f: %.1f/%.1f", -Reach + k * 0.5, H[k], H0[k]);
                    printf("\n");
                }
                WorstIncision = std::max(WorstIncision, Inc);
                double MaxS = 0; int MaxK = 0;
                for (int k = 2; k < Count; k++)
                {
                    if (H0[k] - H[k] < 0.5 && H0[k - 2] - H[k - 2] < 0.5) continue;
                    const double Sl = std::fabs(H[k] - H[k - 2]) / 1.0;
                    const double Sl0 = std::fabs(H0[k] - H0[k - 2]) / 1.0;
                    if (Sl > Sl0 && Sl > MaxS) { MaxS = Sl; MaxK = k; }
                }
                if (MaxS > 4.0 && Fm > 10.0 && Printed++ < 6)
                {
                    const auto R = FSampler(P.S).VoxelLandRiverSegAt(S.Seg);
                    printf("  steep %.1f at %.1f m from the line: area %.0f km^2, width %.0f, valley %.0f, floor %.0f, grade %.4f (fall %.0f m); heights", MaxS, -Reach + MaxK * 0.5,
                        std::exp2(S.Log2Area) * 1e-6, S.Width, S.Valley, S.Floor, R.Grade, R.FallShare * (R.F0 - R.F1));
                    for (int q = std::max(MaxK - 8, 0); q <= std::min(MaxK + 6, Count - 1); q += 2) printf(" %.1f/%.1f", H[q], H0[q]);
                    printf("\n");
                    // The entries at the steep point: each one's carved height and distances.
                    double Dk[3]; Move(P, C, S.Side, -Reach + MaxK * 0.5, Dk);
                    double Qk[3]; Warped(P, Dk, Qk);
                    FSampler Smp = P.S;
                    double SPk[3] = { Dk[0] * P.SeaRadius, Dk[1] * P.SeaRadius, Dk[2] * P.SeaRadius }, QQ[3], DWk[3][3];
                    for (int a2 = 0; a2 < 3; a2++) { QQ[a2] = SPk[a2] + Evaluate(P.W[a2], SPk, 0.0, DWk[a2]) * P.WarpAmp; for (int b2 = 0; b2 < 3; b2++) DWk[a2][b2] *= P.WarpAmp; }
                    const auto Sf = Smp.VoxelLandSurfaceDF(DF(SPk[0]), DF(SPk[1]), DF(SPk[2]), DF(QQ[0]), DF(QQ[1]), DF(QQ[2]),
                        float3((float)DWk[0][0], (float)DWk[0][1], (float)DWk[0][2]), float3((float)DWk[1][0], (float)DWk[1][1], (float)DWk[1][2]),
                        float3((float)DWk[2][0], (float)DWk[2][1], (float)DWk[2][2]), (float)P.Finest, false);
                    const float S1v = Sf.Bake + Sf.Hills + Sf.Peaks + Sf.Gully;
                    const float2 QX = DF(QQ[0]), QY = DF(QQ[1]), QZ = DF(QQ[2]);
                    const uint Tile = Smp.VoxelLandRiverTileOf(normalize(float3(QX.x, QY.x, QZ.x)));
                    const uint F0 = Smp.LandRiverTiles[Tile], End = Smp.LandRiverTiles[Tile + 1];
                    const float* Dd = Smp.LandRiverData;
                    const float3 Cc(Dd[F0], Dd[F0 + 1], Dd[F0 + 2]), T1(Dd[F0 + 4], Dd[F0 + 5], Dd[F0 + 6]), T2(Dd[F0 + 8], Dd[F0 + 9], Dd[F0 + 10]);
                    const float2 DC = Smp.VoxelLandRiverDot(QX, QY, QZ, Cc);
                    const float X = VoxelDFDiv(Smp.VoxelLandRiverDot(QX, QY, QZ, T1), DC).x * Smp.LandRiverRadiusM;
                    const float Y = VoxelDFDiv(Smp.VoxelLandRiverDot(QX, QY, QZ, T2), DC).x * Smp.LandRiverRadiusM;
                    const float Hand = dot(cross(T1, T2), Cc) < 0.0f ? -1.0f : 1.0f;
                    printf("    surface %.1f, strata %.2f, carve %.1f:", S1v, Sf.Strata, Sf.Carve);
                    for (uint At = F0 + VOXEL_LAND_RIVER_FRAME_FLOATS; At < End; At += VOXEL_LAND_RIVER_ENTRY_FLOATS)
                    {
                        const float EX = X - Dd[At], EY = Y - Dd[At + 1], DX = Dd[At + 2], DY = Dd[At + 3];
                        const auto Nn = Smp.VoxelLandRiverNearAt(X, Y, EX, EY, DX, DY, Dd[At + 4], 1.0f / Smp.LandRiverRadiusM);
                        if (Nn.D * Nn.D > Dd[At + 5]) continue;
                        const auto R2 = Smp.VoxelLandRiverSegAt((uint)Dd[At + 6]);
                        const auto Pl = Smp.VoxelLandRiverPlaceAt(R2, Nn.U);
                        const auto Ch = Smp.VoxelLandRiverChannelAt(R2, Nn.U, X, Y, EX, EY, DX, DY, Hand, Nn.Stretch, 1.0f / Smp.LandRiverRadiusM, Sf.Strata);
                        const auto Sh = Smp.VoxelLandRiverShape(S1v, Nn.D, Ch.Distance, Pl.Floor, Pl.Log2Area, Pl.Grade, Pl.Pool, Sf.Strata, R2.StairKey, (float)P.Finest, false);
                        if (Sh.Height < S1v - 1.0f)
                            printf(" [seg %u u %.2f D %.0f Dc %.0f floor %.0f grade %.4f canyon %.2f -> %.1f]", (uint)Dd[At + 6], Nn.U, Nn.D, Ch.Distance, Pl.Floor, Pl.Grade,
                                Smp.VoxelLandRiverCanyon(Sf.Strata, Pl.Grade), Sh.Height);
                    }
                    printf("\n");
                }
                Slope.push_back(MaxS);
                Steep += MaxS > 2.0; VerySteep += MaxS > 4.0;
                WorstSlope = std::max(WorstSlope, MaxS);
            }
            printf("footprint %.0f m: %d sections; incision over the channel's depth p50 %.2f p99 %.2f max %.1f (%d over 3: slots); the carve's steepest slope p50 %.2f p99 %.2f max %.1f (%d over 2, %d over 4)\n",
                Fm, Sections, Pct(Incision, 0.5), Pct(Incision, 0.99), WorstIncision, Slots, Pct(Slope, 0.5), Pct(Slope, 0.99), WorstSlope, Steep, VerySteep);
        }
        return 0;
    }
    if (Mode == "shapegrad")
    {
        // The shape's analytic slopes (in S1, D, Dc) against central differences, over random
        // parameters (the strata's share too).
        FSampler Smp = P.S;
        std::mt19937 Rg(7);
        std::uniform_real_distribution<double> U01(0.0, 1.0);
        std::vector<double> ES, ED, EDc;
        int Bad = 0;
        for (int k = 0; k < 200000; k++)
        {
            const float La = (float)(27.6 + 9.0 * U01(Rg));
            const float Belt = Smp.VoxelLandRiverBelt(La);
            const float Valley = Smp.VoxelLandRiverValleyHalfWidth(Belt);
            const float D = (float)(U01(Rg) * 1.2 * Valley);
            const float Dc = (float)(U01(Rg) < 0.5 ? U01(Rg) * 2.0 * Smp.VoxelLandRiverWidth(La) : D);
            const float Floor = 500.0f;
            const float S1 = Floor + (float)((U01(Rg) - 0.2) * 1500.0);
            const float Grade = (float)std::pow(10.0, -3.5 + 2.5 * U01(Rg));
            const float Strata = U01(Rg) < 0.3 ? 0.0f : (U01(Rg) < 0.5 ? 1.0f : (float)U01(Rg));
            const uint Key = (uint)k * 2654435761u;
            const auto Sh = Smp.VoxelLandRiverShape(S1, D, Dc, Floor, La, Grade, 0.0f, Strata, Key, 100.0f, true);
            auto H = [&](float s1, float d, float dc) { return (double)Smp.VoxelLandRiverShape(s1, d, dc, Floor, La, Grade, 0.0f, Strata, Key, 100.0f, false).Height; };
            const double h = 0.01;
            const double FS = (H(S1 + 0.01f, D, Dc) - H(S1 - 0.01f, D, Dc)) / 0.02;
            const double FD = D > 0.02f ? (H(S1, D + 0.01f, Dc) - H(S1, D - 0.01f, Dc)) / 0.02 : Sh.DD;
            const double FDc = Dc > 0.02f ? (H(S1, D, Dc + 0.01f) - H(S1, D, Dc - 0.01f)) / 0.02 : Sh.DDc;
            (void)h;
            ES.push_back(std::fabs(FS - Sh.Scale)); ED.push_back(std::fabs(FD - Sh.DD)); EDc.push_back(std::fabs(FDc - Sh.DDc));
            if (std::fabs(FDc - Sh.DDc) > 0.03)
                printf("  Dc: fd %.4f analytic %.4f at Dc %.2f (half %.1f), D %.1f, S1-F %.2f, grade %.4f, strata %.2f, area %.1f\n", FDc, Sh.DDc, Dc, 0.5 * Smp.VoxelLandRiverWidth(La), D, S1 - Floor, Grade, Strata, La);
            if (std::fabs(FD - Sh.DD) > 0.05 && Bad++ < 8)
                printf("  D: fd %.4f analytic %.4f at D %.1f of valley %.0f, S1-F %.1f, grade %.4f, strata %.2f, area %.1f\n", FD, Sh.DD, D, Valley, S1 - Floor, Grade, Strata, La);
        }
        printf("shape slopes against differences: S1 p99 %.4f max %.4f; D p99 %.4f max %.4f; Dc p99 %.4f max %.4f (%d over 0.05 in D)\n",
            Pct(ES, 0.99), Pct(ES, 1.0), Pct(ED, 0.99), Pct(ED, 1.0), Pct(EDc, 0.99), Pct(EDc, 1.0), Bad);
        return 0;
    }
    if (Mode == "lakeseam")
    {
        // The lakes across the cube's face edges: lines from the lake cells on a face's edge rows
        // out across the edge, every metre, 3 cells each way; the weight's and the bed's (weight x
        // depth) largest change in a metre against what the weights' smoothstep allows.
        // The test planet has no lake on a face's edge, so lakes of 2 x 3 cells are put on every
        // edge (3 a side) at 8 m over their cells' highest, and every apron cell's level is taken
        // again from the cell holding its direction (VoxelSolveOutputPass's rule).
        const int Side = P.Side();
        auto CellOf = [&](const double* Dir) {
            int Axis = 0;
            for (int a = 1; a < 3; a++) if (std::fabs(Dir[a]) > std::fabs(Dir[Axis])) Axis = a;
            int f = 0;
            for (; f < 6; f++) if ((int)VoxelCubeNormalAxis(f) == Axis && VoxelCubeNormalSign(f) * Dir[Axis] > 0) break;
            const float3 Uf = VoxelCubeAxisVector(VoxelCubeUAxis(f)), Vf = VoxelCubeAxisVector(VoxelCubeVAxis(f));
            const double Nn = std::fabs(Dir[Axis]);
            const double TU = (Dir[0] * Uf.x + Dir[1] * Uf.y + Dir[2] * Uf.z) / Nn, TV = (Dir[0] * Vf.x + Dir[1] * Vf.y + Dir[2] * Vf.z) / Nn;
            const int i = std::clamp((int)std::lround((std::atan(TU) * 4.0 / M_PI + 1.0) * N * 0.5 - 0.5), 0, N - 1);
            const int j = std::clamp((int)std::lround((std::atan(TV) * 4.0 / M_PI + 1.0) * N * 0.5 - 0.5), 0, N - 1);
            return f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
        };
        for (int f = 0; f < 6; f++)
            for (int Edge = 0; Edge < 4; Edge++)
                for (int k = 1; k <= 3; k++)
                {
                    const int Along = k * N / 4 + 9 * f - 20;   // not where another face's are
                    std::vector<int> Cs;
                    for (int In = 0; In < 2; In++)
                        for (int d = -1; d <= 1; d++)
                        {
                            const int a = Edge < 2 ? (Edge == 0 ? In : N - 1 - In) : Along + d;
                            const int b = Edge < 2 ? Along + d : (Edge == 2 ? In : N - 1 - In);
                            Cs.push_back(f * Side * Side + (b + P.Pad) * Side + a + P.Pad);
                        }
                    float Top = -1e30f;
                    for (const int c : Cs) Top = std::max(Top, P.Height[c]);
                    for (const int c : Cs) P.WaterF[c] = Top + 8.0f;
                }
        for (int f = 0; f < 6; f++)
            for (int j = -P.Pad; j < N + P.Pad; j++)
                for (int i = -P.Pad; i < N + P.Pad; i++)
                {
                    if (i >= 0 && i < N && j >= 0 && j < N) continue;
                    const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                    double Dir[3]; CellDir(P, c, Dir);
                    P.WaterF[c] = P.WaterF[CellOf(Dir)];
                }
        FSampler Smp = P.S;
        int Lines = 0, Steps = 0;
        double WorstW = 0, WorstB = 0;
        for (int f = 0; f < 6; f++)
            for (int j = 0; j < N; j++)
                for (int i = 0; i < N; i++)
                {
                    const bool bEdgeI = i == 0 || i == N - 1, bEdgeJ = j == 0 || j == N - 1;
                    if (!bEdgeI && !bEdgeJ) continue;
                    const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                    if (!(P.WaterF[c] > -1e30f)) continue;
                    for (int e = 0; e < 2; e++)
                    {
                        if (e == 0 ? !bEdgeI : !bEdgeJ) continue;
                        // Toward the edge: from the cell one step in to this one.
                        const int In = e == 0 ? c + (i == 0 ? 1 : -1) : c + (j == 0 ? Side : -Side);
                        double A[3], B[3], T[3];
                        CellDir(P, In, A); CellDir(P, c, B);
                        for (int a = 0; a < 3; a++) T[a] = B[a] - A[a];
                        const double Dot = T[0] * B[0] + T[1] * B[1] + T[2] * B[2];
                        for (int a = 0; a < 3; a++) T[a] -= Dot * B[a];
                        Normalize(T);
                        const double Half = 3.0 * P.Cell * 0.01;
                        std::vector<double> VW, VB; std::vector<int> VF;
                        for (double t = -Half; t <= Half; t += 1.0)
                        {
                            double Q[3]; Move(P, B, T, t, Q);
                            const auto L = Smp.VoxelLandLakeAt(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius), false);
                            int Ax = 0;
                            for (int q = 1; q < 3; q++) if (std::fabs(Q[q]) > std::fabs(Q[Ax])) Ax = q;
                            VW.push_back(L.Weight); VB.push_back(L.Weight * L.Depth); VF.push_back(Ax * 2 + (Q[Ax] < 0));
                        }
                        // At the face change: the change over that metre less its neighbours' mean.
                        bool bStep = false;
                        for (size_t q = 2; q + 2 < VW.size(); q++)
                        {
                            if (VF[q] == VF[q - 1]) continue;
                            const double EW = std::fabs(VW[q] - VW[q - 1]) - 0.5 * (std::fabs(VW[q - 1] - VW[q - 2]) + std::fabs(VW[q + 1] - VW[q]));
                            const double EB = std::fabs(VB[q] - VB[q - 1]) - 0.5 * (std::fabs(VB[q - 1] - VB[q - 2]) + std::fabs(VB[q + 1] - VB[q]));
                            WorstW = std::max(WorstW, EW); WorstB = std::max(WorstB, EB);
                            if (EW > 0.001 || EB > 0.01) bStep = true;
                        }
                        Lines++;
                        Steps += bStep ? 1 : 0;
                    }
                }
        printf("lakes across face edges: %d lines from edge lake cells, %d with a step at the edge (the change over the metre across it, less its neighbours': weight over 0.001 or weight x depth over 1 cm); largest: weight %.5f, weight x depth %.4f m\n",
            Lines, Steps, WorstW, WorstB);
        return 0;
    }
    if (Mode == "lakegrad")
    {
        // The lake weight's and depth's analytic gradients against central differences.
        const std::vector<int> Edges = LakeEdges(P);
        FSampler Smp = P.S;
        std::vector<double> EW, ED, MW;
        for (int k = 0; k < 400; k++)
        {
            double LC[3]; CellDir(P, Edges[(size_t)(k * 7919) % Edges.size()], LC);
            double E1[3], E2[3]; Frame(LC, E1, E2);
            double T[3]; Move(P, LC, E1, ((k * 37) % 100 - 50) * 40.0, T);
            double D[3]; Move(P, T, E2, ((k * 53) % 100 - 50) * 40.0, D);
            const auto L = Smp.VoxelLandLakeAt(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), true);
            if (L.Weight <= 0.01 || L.Weight > 0.99) continue;
            double Fd[2], Fdd[2], An[2], And[2];
            Frame(D, E1, E2);
            for (int e = 0; e < 2; e++)
            {
                const double* E = e == 0 ? E1 : E2;
                double A[3], B[3]; Move(P, D, E, 1.0, A); Move(P, D, E, -1.0, B);
                const auto LA = Smp.VoxelLandLakeAt(DF(A[0] * P.SeaRadius), DF(A[1] * P.SeaRadius), DF(A[2] * P.SeaRadius), false);
                const auto LB = Smp.VoxelLandLakeAt(DF(B[0] * P.SeaRadius), DF(B[1] * P.SeaRadius), DF(B[2] * P.SeaRadius), false);
                Fd[e] = (LA.Weight - LB.Weight) / 2.0;
                Fdd[e] = (LA.Depth - LB.Depth) / 2.0;
                An[e] = L.WeightGrad.x * E[0] + L.WeightGrad.y * E[1] + L.WeightGrad.z * E[2];
                And[e] = L.DepthGrad.x * E[0] + L.DepthGrad.y * E[1] + L.DepthGrad.z * E[2];
            }
            EW.push_back(std::hypot(Fd[0] - An[0], Fd[1] - An[1]) / std::max(std::hypot(Fd[0], Fd[1]), 1e-9));
            ED.push_back(std::hypot(Fdd[0] - And[0], Fdd[1] - And[1]) / std::max(std::hypot(Fdd[0], Fdd[1]), 1e-9));
            if (ED.back() > 0.5) printf("  BAD weight %.3f: fd (%.3g %.3g) analytic (%.3g %.3g); depth %.1f fd (%.3g %.3g) analytic (%.3g %.3g)\n", L.Weight, Fd[0], Fd[1], An[0], An[1], L.Depth, Fdd[0], Fdd[1], And[0], And[1]);
            if (k < 6) printf("  weight %.3f: fd (%.3g %.3g) analytic (%.3g %.3g); depth %.1f fd (%.3g %.3g) analytic (%.3g %.3g)\n", L.Weight, Fd[0], Fd[1], An[0], An[1], L.Depth, Fdd[0], Fdd[1], And[0], And[1]);
        }
        printf("lake weight gradient relative error p50 %.4f p99 %.4f; depth p50 %.4f p99 %.4f (n %zu)\n", Pct(EW, 0.5), Pct(EW, 0.99), Pct(ED, 0.5), Pct(ED, 0.99), EW.size());
        return 0;
    }
    if (Mode == "lakeparts")
    {
        // At lake shores: each term's change against central differences: the carve's against
        // (scale - 1) x the surface's plus its own gradient; the hills' against their exact
        // gradient plus the unit times the amplitude's change.
        const std::vector<int> Edges = LakeEdges(P);
        std::vector<double> EC, EH, ES, EL;
        for (int k = 0; k < 600; k++)
        {
            double LC[3]; CellDir(P, Edges[(size_t)(k * 7919) % Edges.size()], LC);
            double LP[3]; Unwarp(P, LC, LP);
            double E1[3], E2[3]; Frame(LP, E1, E2);
            double T[3]; Move(P, LP, E1, ((k * 37) % 100 - 50) * 60.0, T);
            double D[3]; Move(P, T, E2, ((k * 53) % 100 - 50) * 60.0, D);
            const double F = std::max(P.Finest, 100.0);
            auto At = [&](const double* Dir, bool bG) { return VoxelLandHeightDF(DF(Dir[0] * P.SeaRadius), DF(Dir[1] * P.SeaRadius), DF(Dir[2] * P.SeaRadius), (float)F, bG); };
            const auto H = At(D, true);
            Frame(D, E1, E2);
            double Err[3] = { 0, 0, 0 }, Mag[3] = { 0, 0, 0 };
            for (int e = 0; e < 2; e++)
            {
                const double* E = e == 0 ? E1 : E2;
                double A[3], B[3]; Move(P, D, E, 0.1, A); Move(P, D, E, -0.1, B);
                const auto HA = At(A, false), HB = At(B, false);
                auto Fd = [&](double a, double b) { return (a - b) / 0.2; };
                const double SA = HA.Bake + HA.Hills + HA.Peaks + HA.Gully, SB = HB.Bake + HB.Hills + HB.Peaks + HB.Gully;
                const double GS = Fd(SA, SB), GC = Fd(HA.Carve, HB.Carve);
                const double CG = H.CarveGradient.x * E[0] + H.CarveGradient.y * E[1] + H.CarveGradient.z * E[2];
                const double PredC = ((double)H.CarveScale - 1.0) * GS + CG;
                Err[0] += (GC - PredC) * (GC - PredC); Mag[0] += GC * GC;
                // The hills: exact gradient plus the unit times the amplitude's change.
                const double GH = Fd(HA.Hills, HB.Hills);
                const double HG = H.HillGradient.x * E[0] + H.HillGradient.y * E[1] + H.HillGradient.z * E[2];
                const double PredH = HG + H.HillUnit * Fd(HA.HillAmplitudeM, HB.HillAmplitudeM);
                Err[1] += (GH - PredH) * (GH - PredH); Mag[1] += GH * GH;
                Err[2] += 0; Mag[2] += GS * GS;
            }
            EC.push_back(std::sqrt(Err[0])); EH.push_back(std::sqrt(Err[1])); ES.push_back(std::sqrt(Mag[2]));
            // The lake's piece alone: its height as a function of the point (the surface at each).
            {
                FSampler Smp = P.S;
                auto Piece = [&](const double* Dir, bool bG) {
                    double Q[3]; Warped(P, Dir, Q);
                    const FTerms TT = TermsWith(P, P.S, Dir, F);
                    const double S0 = TT.Bake + TT.Hills + TT.Peaks + TT.Gully;
                    const auto Lk = Smp.VoxelLandLakeAt(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius), bG);
                    return std::make_pair(S0, Smp.VoxelLandLakeShore(Lk, (float)S0, bG));
                };
                const auto C0 = Piece(D, true);
                if (C0.second.Height != C0.first)
                {
                    double ErrL = 0;
                    for (int e = 0; e < 2; e++)
                    {
                        const double* E = e == 0 ? E1 : E2;
                        double A[3], B[3]; Move(P, D, E, 0.1, A); Move(P, D, E, -0.1, B);
                        const auto CA = Piece(A, false), CB = Piece(B, false);
                        const double GS = (CA.first - CB.first) / 0.2, GL = ((double)CA.second.Height - CB.second.Height) / 0.2;
                        // The shore's own gradient is at the warped point: through the warp's Jacobian here.
                        double SPt[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius }, DW[3][3];
                        for (int a = 0; a < 3; a++) { Evaluate(P.W[a], SPt, 0.0, DW[a]); for (int b = 0; b < 3; b++) DW[a][b] *= P.WarpAmp; }
                        const float3 G = Smp.VoxelLandWarpedGradient(C0.second.Gradient, float3((float)DW[0][0], (float)DW[0][1], (float)DW[0][2]),
                            float3((float)DW[1][0], (float)DW[1][1], (float)DW[1][2]), float3((float)DW[2][0], (float)DW[2][1], (float)DW[2][2]), float3((float)D[0], (float)D[1], (float)D[2]));
                        const double Pred = C0.second.Scale * GS + (G.x * E[0] + G.y * E[1] + G.z * E[2]);
                        ErrL += (GL - Pred) * (GL - Pred);
                    }
                    EL.push_back(std::sqrt(ErrL));
                    if (std::sqrt(ErrL) > 0.02) printf("  lake piece err %.4f: height %.2f (from %.2f), scale %.3f\n", std::sqrt(ErrL), C0.second.Height, C0.first, C0.second.Scale);
                }
            }
            if (std::sqrt(Err[0]) > 0.02 || std::sqrt(Err[1]) > 0.02)
                printf("  carve err %.4f (|grad| %.4f) hills err %.4f (|grad| %.4f); lake w %.2f carve %.2f scale %.3f\n", std::sqrt(Err[0]), std::sqrt(Mag[0]), std::sqrt(Err[1]), std::sqrt(Mag[1]),
                    0.0, H.Carve, H.CarveScale);
        }
        printf("lake parts: carve's gradient error p50 %.5f p99 %.5f max %.4f; hills' p50 %.5f p99 %.5f max %.4f; surface slope p50 %.3f; the lake's piece p50 %.5f p99 %.5f max %.4f (n %zu)\n",
            Pct(EC, 0.5), Pct(EC, 0.99), Pct(EC, 1.0), Pct(EH, 0.5), Pct(EH, 0.99), Pct(EH, 1.0), Pct(ES, 0.5), Pct(EL, 0.5), Pct(EL, 0.99), Pct(EL, 1.0), EL.size());
        return 0;
    }
    if (Mode == "water")
    {
        // The water query's inputs (FVoxelDensityCPU::WaterAt's rules on the shared carve's
        // outputs): on the rivers' lines, the river's water over the ground and its depth; in the
        // lakes' cells, the lake over the ground.
        FSampler Smp = P.S;
        auto Surface = [&](const double* D) {
            double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius }, Q[3], DW[3][3];
            for (int a = 0; a < 3; a++) { Q[a] = SP[a] + Evaluate(P.W[a], SP, 0.0, DW[a]) * P.WarpAmp; for (int b = 0; b < 3; b++) DW[a][b] *= P.WarpAmp; }
            return Smp.VoxelLandSurfaceDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), DF(Q[0]), DF(Q[1]), DF(Q[2]),
                float3((float)DW[0][0], (float)DW[0][1], (float)DW[0][2]), float3((float)DW[1][0], (float)DW[1][1], (float)DW[1][2]),
                float3((float)DW[2][0], (float)DW[2][1], (float)DW[2][2]), (float)P.Finest, false);
        };
        int Lines = 0, Wet = 0, DryNone = 0, DryFaded = 0, DryOff = 0, DryAbove = 0, DryNoEntry = 0, DryLake = 0, Unconverged = 0; std::vector<double> Depth, Speed, AboveBy, NoneDist, Resid, SiteDist;
        for (size_t i = 0; i < All.size(); i += 11)
        {
            double D[3]; Unwarp(P, All[i].Q, D);
            const FTerms T = TermsWith(P, P.S, D, P.Finest);
            const auto L = Surface(D);
            Lines++;
            {
                const auto RL = Smp.VoxelLandRiverAtDF(DF(All[i].Q[0] * P.SeaRadius), DF(All[i].Q[1] * P.SeaRadius), DF(All[i].Q[2] * P.SeaRadius));
                SiteDist.push_back(RL.bFound ? RL.Distance : 1e9);
            }
            {
                double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius }, W[3];
                for (int a = 0; a < 3; a++) W[a] = SP[a] + Evaluate(P.W[a], SP, 0.0) * P.WarpAmp;
                Normalize(W);
                const double Off = std::sqrt(std::pow(W[0] - All[i].Q[0], 2) + std::pow(W[1] - All[i].Q[1], 2) + std::pow(W[2] - All[i].Q[2], 2)) * P.SeaRadius * 0.01;
                Resid.push_back(Off);
                if (Off > 1.0) { Unconverged++; Lines--; continue; }
            }
            if (!(L.River.WaterKind == VOXEL_LAND_WATER_RIVER && L.River.WaterLevel > T.Height))
            {
                // Why dry: no river in reach, off the channel (the site isn't on the line after the
                // unwarp), a channel faded out (near a source, or below the footprint's band), else
                // the ground above the water line.
                if (L.River.WaterKind != VOXEL_LAND_WATER_RIVER) { DryNone++; if (L.River.Distance > 1e30f) DryNoEntry++; else NoneDist.push_back(L.River.Distance); if (L.Lake.Weight > 0.0f) DryLake++; }
                else if (L.River.ChannelDepth < 0.99f * (float)All[i].Depth) DryFaded++;
                else if (L.River.Distance > 15.0f) DryOff++;
                else
                {
                    DryAbove++; AboveBy.push_back(T.Height - L.River.WaterLevel);
                    if (DryAbove <= 6)
                        printf("    above: site floor %.1f depth %.2f; carve floor-line %.1f cut %.2f grade %.4f dist %.2f; ground %.2f (surface %.2f carve %.2f detail %.2f)\n",
                            All[i].Floor, All[i].Depth, L.River.WaterLevel + (1.0f - VOXEL_LAND_RIVER_WATER_SHARE) * L.River.ChannelDepth, L.River.ChannelDepth, L.River.Grade, L.River.Distance,
                            T.Height, L.River.Height, L.Carve, T.Height - L.River.Height);
                }
            }
            if (L.River.WaterKind == VOXEL_LAND_WATER_RIVER && L.River.WaterLevel > T.Height)
            {
                Wet++;
                Depth.push_back(L.River.WaterLevel - T.Height);
                const double R = std::max(0.8 * L.River.ChannelDepth, 0.1), Sl = std::max((double)L.River.Grade, 1e-4);
                const double Nm = std::max(0.035, 0.39 * std::pow(Sl, 0.38) * std::pow(R, -0.16));
                Speed.push_back(std::clamp(std::pow(R, 2.0 / 3.0) * std::sqrt(Sl) / Nm, 0.05, 5.0));
            }
        }
        printf("rivers: %d of %d line points under a river's water: depth p10 %.2f p50 %.2f p90 %.2f m; speed p10 %.2f p50 %.2f p90 %.2f m/s\n",
            Wet, Lines, Pct(Depth, 0.1), Pct(Depth, 0.5), Pct(Depth, 0.9), Pct(Speed, 0.1), Pct(Speed, 0.5), Pct(Speed, 0.9));
        printf("  the sites' own distance to the nearest line: p50 %.2f p90 %.2f p99 %.2f m\n", Pct(SiteDist, 0.5), Pct(SiteDist, 0.9), Pct(SiteDist, 0.99));
        printf("  the unwarp's residual: p50 %.3f p99 %.3f max %.3f m (%d over 1 m left out: the warp folds there)\n", Pct(Resid, 0.5), Pct(Resid, 0.99), Pct(Resid, 1.0), Unconverged);
        printf("  no river: %d with no entry in reach, %d in a lake's weight; the rest at p10 %.1f p50 %.1f p90 %.1f m from a line\n", DryNoEntry, DryLake, Pct(NoneDist, 0.1), Pct(NoneDist, 0.5), Pct(NoneDist, 0.9));
        printf("  dry: %d off the channel (none in reach, or it meanders away from the line), %d in the fade-in from RiverMinArea (the valley and channel part-cut), %d over 15 m off the line, %d ground above the water (by p50 %.2f p90 %.2f m)\n", DryNone, DryFaded, DryOff, DryAbove, Pct(AboveBy, 0.5), Pct(AboveBy, 0.9));
        // Lakes: the lake cells' centres (the lakes at 512 are a few cells, with none inside).
        int Cells = 0, LakeWet = 0, LakeFolded = 0, LakeDry = 0; std::vector<double> LDepth;
        for (size_t c = 0; c < P.WaterF.size() && Cells < 3000; c++)
        {
            if (!(P.WaterF[c] > -1e30f)) continue;
            double Q[3]; CellDir(P, (int)c, Q);
            double D[3]; Unwarp(P, Q, D);
            {
                double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius }, W[3];
                for (int a = 0; a < 3; a++) W[a] = SP[a] + Evaluate(P.W[a], SP, 0.0) * P.WarpAmp;
                Normalize(W);
                if (std::sqrt(std::pow(W[0] - Q[0], 2) + std::pow(W[1] - Q[1], 2) + std::pow(W[2] - Q[2], 2)) * P.SeaRadius * 0.01 > 1.0) { LakeFolded++; continue; }
            }
            const FTerms T = TermsWith(P, P.S, D, P.Finest);
            const auto L = Surface(D);
            Cells++;
            if (!(L.Lake.Weight >= 0.5f && L.Lake.Level > T.Height) && LakeDry++ < 4)
                printf("    dry lake cell: weight %.3f level %.2f depth %.2f ground %.2f\n", L.Lake.Weight, L.Lake.Level, L.Lake.Depth, T.Height);
            if (L.Lake.Weight >= 0.5f && L.Lake.Level > T.Height) { LakeWet++; LDepth.push_back(L.Lake.Level - T.Height); }
        }
        printf("lakes: %d of %d lake cells' centres under their lake's level (%d where the warp folds left out): depth p10 %.1f p50 %.1f p90 %.1f m\n", LakeWet, Cells, LakeFolded, Pct(LDepth, 0.1), Pct(LDepth, 0.5), Pct(LDepth, 0.9));
        return 0;
    }
    if (Mode == "check")
    {
        // Centre: the carved surface on the line against the bed.
        {
            std::vector<double> OnBed, AboveFloor, Carve;
            int Below = 0;
            for (size_t i = 0; i < All.size(); i += 7)
            {
                const FRiverSite& S = All[i];
                double D[3]; Unwarp(P, S.Q, D);
                const FTerms On = TermsWith(P, P.S, D, P.Finest), Raw = TermsWith(P, Off, D, P.Finest);
                const double Bed = S.Floor - S.Depth;
                OnBed.push_back(On.Height - Bed);
                AboveFloor.push_back(Raw.Height - S.Floor);
                Carve.push_back(On.Height - Raw.Height);
                Below += Raw.Height < Bed ? 1 : 0;
            }
            printf("centre (%zu sites): carved height over the bed p1 %.2f p50 %.2f p99 %.2f m; uncarved over the floor p10 %.1f p50 %.1f p90 %.1f m (%d of them below the bed already); the carve p1 %.1f p50 %.1f p99 %.2f m\n",
                OnBed.size(), Pct(OnBed, 0.01), Pct(OnBed, 0.5), Pct(OnBed, 0.99), Pct(AboveFloor, 0.1), Pct(AboveFloor, 0.5), Pct(AboveFloor, 0.9), Below,
                Pct(Carve, 0.01), Pct(Carve, 0.5), Pct(Carve, 0.99));
        }
        // Continuity: lines through corridors (across and along), every 0.5 m.
        {
            const int Lines = getenv("LINES") ? atoi(getenv("LINES")) : 24;
            const double LenM = getenv("LINE_M") ? atof(getenv("LINE_M")) : 8000.0;
            FSteps Steps, StepsOff;
            long Samples = 0;
            std::uniform_real_distribution<double> U(0, 1);
            // Every third line from a river near a face's edge (its two largest components within 2%).
            std::vector<size_t> NearEdge;
            for (size_t i = 0; i < All.size(); i++)
            {
                double A[3] = { std::fabs(All[i].Q[0]), std::fabs(All[i].Q[1]), std::fabs(All[i].Q[2]) };
                std::sort(A, A + 3);
                if (A[1] > 0.98 * A[2]) NearEdge.push_back(i);
            }
            for (int l = 0; l < Lines; l++)
            {
                const bool bEdge = l % 3 == 2 && !NearEdge.empty();
                const FRiverSite& S = bEdge ? All[NearEdge[(size_t)(U(Rng) * NearEdge.size()) % NearEdge.size()]] : All[(size_t)(U(Rng) * All.size()) % All.size()];
                double C[3]; Unwarp(P, S.Q, C);
                // Across the river (half the lines), along it at an offset in the corridor (the rest).
                double E[3];
                double Start[3];
                if (l % 2 == 0)
                {
                    for (int a = 0; a < 3; a++) E[a] = S.Side[a];
                    Move(P, C, E, -0.5 * LenM, Start);
                }
                else
                {
                    const double Offset = (U(Rng) * 2 - 1) * S.Valley;
                    double O[3]; Move(P, C, S.Side, Offset, O);
                    for (int a = 0; a < 3; a++) E[a] = S.Along[a];
                    Move(P, O, E, -0.5 * LenM, Start);
                }
                // The line's tangent at its start (a great circle through Start along E).
                double T[3]; Cross(C, E, T); double E2[3]; Cross(T, Start, E2); Normalize(E2);
                const int Count = (int)(LenM / 0.5);
                std::vector<std::array<double, 3>> Dirs(Count);
                std::vector<double> H(Count), H0(Count);
                ParallelFor(Count, [&](int k) {
                    double D[3]; Move(P, Start, E2, k * 0.5, D);
                    Dirs[k] = { D[0], D[1], D[2] };
                    H[k] = TermsWith(P, P.S, D, P.Finest).Height;
                    H0[k] = TermsWith(P, Off, D, P.Finest).Height;
                });
                Samples += Count;
                FindSteps(P, Dirs, H, Steps, 0.02);
                FindSteps(P, Dirs, H0, StepsOff, 0.02);
            }
            printf("continuity: %ld samples on %d lines of %.0f m through corridors: %d steps over 2 cm (largest %.3f m; %d at a river tile's edge, %d at a face edge); uncarved %d (largest %.3f m)\n",
                Samples, Lines, LenM, Steps.Count, Steps.Max, Steps.AtTileEdge, Steps.AtFaceEdge, StepsOff.Count, StepsOff.Max);
            printf("  crossed %d river tile edges and %d face edges; the change's excess over its neighbours': largest at a tile edge %.4f m, everywhere p99.9 %.4f max %.4f m (uncarved max %.4f)\n",
                Steps.TileEdges, Steps.FaceEdges, Steps.MaxAtTileEdge, Pct(Steps.Excess, 0.999), Pct(Steps.Excess, 1.0), Pct(StepsOff.Excess, 1.0));
        }
        // Lakes: lines across shores (from a lake cell at the edge of its lake, outward).
        if (P.S.LandHasWater)
        {
            const int Side = P.Side();
            std::vector<int> Edge;
            for (int f = 0; f < 6; f++)
                for (int j = 1; j < N - 1; j++)
                    for (int i = 1; i < N - 1; i++)
                    {
                        const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                        if (!(P.WaterF[c] > -1e30f)) continue;
                        if (!(P.WaterF[c + 1] > -1e30f) || !(P.WaterF[c - 1] > -1e30f) || !(P.WaterF[c + Side] > -1e30f) || !(P.WaterF[c - Side] > -1e30f)) Edge.push_back(c);
                    }
            FSteps Steps;
            long Samples = 0;
            std::vector<double> Bench;
            const int Lines = std::min<int>(12, (int)Edge.size());
            for (int l = 0; l < Lines; l++)
            {
                const int c = Edge[(size_t)(l * 7919) % Edge.size()];
                const int f = c / (Side * Side), r = c % (Side * Side), j = r / Side - P.Pad, i = r % Side - P.Pad;
                // The cell's direction (VoxelSolveDirection: the tan table).
                const float2 TA = P.Tan[i + P.Pad], TB = P.Tan[j + P.Pad];
                const float3 Nf = VoxelCubeAxisVector(VoxelCubeNormalAxis(f)) * VoxelCubeNormalSign(f);
                const float3 Uf = VoxelCubeAxisVector(VoxelCubeUAxis(f)), Vf = VoxelCubeAxisVector(VoxelCubeVAxis(f));
                double Q[3] = { Nf.x + Uf.x * (TA.x + TA.y) + Vf.x * (TB.x + TB.y), Nf.y + Uf.y * (TA.x + TA.y) + Vf.y * (TB.x + TB.y), Nf.z + Uf.z * (TA.x + TA.y) + Vf.z * (TB.x + TB.y) };
                Normalize(Q);
                double C[3]; Unwarp(P, Q, C);
                double E1[3], E2[3]; Frame(C, E1, E2);
                double Start[3]; Move(P, C, l % 2 ? E1 : E2, -3000.0, Start);
                double T[3]; Cross(C, l % 2 ? E1 : E2, T); double E[3]; Cross(T, Start, E); Normalize(E);
                const int Count = 12000;
                std::vector<std::array<double, 3>> Dirs(Count);
                std::vector<double> H(Count);
                const float Level = P.WaterF[c];
                std::atomic<int> InBand{ 0 };
                ParallelFor(Count, [&](int k) {
                    double D[3]; Move(P, Start, E, k * 0.5, D);
                    Dirs[k] = { D[0], D[1], D[2] };
                    H[k] = TermsWith(P, P.S, D, P.Finest).Height;
                    if (std::fabs(H[k] - Level) < P.S.LandLakeShoreBand) InBand++;
                });
                Samples += Count;
                FindSteps(P, Dirs, H, Steps, 0.02);
                Bench.push_back(InBand.load() * 0.5);
            }
            printf("lakes: %zu shore cells; %ld samples on %d lines across shores: %d steps over 2 cm (largest %.3f m); ground within the band of the level along a line: median %.0f m\n",
                Edge.size(), Samples, Lines, Steps.Count, Steps.Max, Pct(Bench, 0.5));
        }
        // Gradient: the cache writer's against central differences, near rivers; and the
        // backends against each other there.
        const std::vector<int> Edges = LakeEdges(P);
        for (double F : { std::max(P.Finest, 100.0), 1000.0 })
        {
            for (int bCarve = 1; bCarve >= 0; bCarve--)
            {
                VoxelShader::LandRiverCarve = bCarve ? P.S.LandRiverCarve : 0.0f;
                VoxelShader::LandHasWater = bCarve ? P.S.LandHasWater : 0u;
                std::vector<double> Err, Back;
                std::uniform_real_distribution<double> U(0, 1);
                const int Points = 2000;
                std::vector<double> ErrV(Points), BackV(Points);
                ParallelFor(Points, [&](int k) {
                    std::mt19937 R2(1000 + k);
                    std::uniform_real_distribution<double> U2(0, 1);
                    const FRiverSite& S = All[(size_t)(U2(R2) * All.size()) % All.size()];
                    double C[3]; Unwarp(P, S.Q, C);
                    double O[3]; Move(P, C, S.Side, (U2(R2) * 2 - 1) * 1.2 * S.Valley, O);
                    double D[3]; Move(P, O, S.Along, (U2(R2) - 0.5) * 200.0, D);
                    if (k % 2 == 1 && !Edges.empty())
                    {
                        // Every other point at a lake's shore, within a few km.
                        double LC[3]; CellDir(P, Edges[(size_t)(U2(R2) * Edges.size()) % Edges.size()], LC);
                        double LP[3]; Unwarp(P, LC, LP);
                        double E1[3], E2[3]; Frame(LP, E1, E2);
                        double T[3]; Move(P, LP, E1, (U2(R2) * 2 - 1) * 3000.0, T);
                        Move(P, T, E2, (U2(R2) * 2 - 1) * 3000.0, D);
                    }
                    auto At = [&](const double* Dir) {
                        return VoxelLandHeightDF(DF(Dir[0] * P.SeaRadius), DF(Dir[1] * P.SeaRadius), DF(Dir[2] * P.SeaRadius), (float)F, false);
                    };
                    const FVoxelLandCacheSurface CS = VoxelLandCacheSurfaceAt(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), (float)F, 25.0f);
                    double E1[3], E2[3]; Frame(D, E1, E2);
                    double Ref[2], An[2];
                    for (int e = 0; e < 2; e++)
                    {
                        const double* E = e == 0 ? E1 : E2;
                        double A[3], B[3], A2[3], B2[3];
                        Move(P, D, E, 0.1, A); Move(P, D, E, -0.1, B); Move(P, D, E, 0.2, A2); Move(P, D, E, -0.2, B2);
                        const auto HA = At(A), HB = At(B), HA2 = At(A2), HB2 = At(B2);
                        const double G1 = ((HA.Height.x - HB.Height.x) + ((double)HA.Height.y - HB.Height.y)) * 0.01 / 0.2;
                        const double G2 = ((HA2.Height.x - HB2.Height.x) + ((double)HA2.Height.y - HB2.Height.y)) * 0.01 / 0.4;
                        Ref[e] = (4 * G1 - G2) / 3;
                        An[e] = CS.Gradient.x * E[0] + CS.Gradient.y * E[1] + CS.Gradient.z * E[2];
                    }
                    const double Dot = (1 + An[0] * Ref[0] + An[1] * Ref[1]) / (std::sqrt(1 + An[0] * An[0] + An[1] * An[1]) * std::sqrt(1 + Ref[0] * Ref[0] + Ref[1] * Ref[1]));
                    ErrV[k] = std::acos(std::min(1.0, Dot)) * 57.2957795;
                    if (getenv("GRAD_DEBUG") && bCarve && ErrV[k] > 2.0 && F < 500)
                    {
                        // The error's direction against the river's: along it, or across.
                        double Q[3]; Warped(P, D, Q);
                        FSampler Smp = P.S;
                        const auto R = Smp.VoxelLandRiverAtDF(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius));
                        const auto Lk = Smp.VoxelLandLakeAt(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius), false);
                        const double Ex = An[0] - Ref[0], Ey = An[1] - Ref[1];
                        double ErrV3[3]; for (int a = 0; a < 3; a++) ErrV3[a] = Ex * E1[a] + Ey * E2[a];
                        const double OnAlong = ErrV3[0] * S.Along[0] + ErrV3[1] * S.Along[1] + ErrV3[2] * S.Along[2];
                        const double OnSide = ErrV3[0] * S.Side[0] + ErrV3[1] * S.Side[1] + ErrV3[2] * S.Side[2];
                        const double Wd = R.bFound ? Smp.VoxelLandRiverWidth(R.Log2Area) : 0;
                        if (ErrV[k] > 45.0)
                        {
                            const FTerms TT = TermsWith(P, P.S, D, F);
                            const double S0 = TT.Bake + TT.Hills + TT.Peaks + TT.Gully;
                            const auto Lk2 = Smp.VoxelLandLakeAt(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius), true);
                            const auto Sh = Smp.VoxelLandLakeShore(Lk2, (float)S0, true);
                            const auto Rv = Smp.VoxelLandRiverCarveDF(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius), Sh.Height, 0.0f, float3(0.0f, 0.0f, 0.0f), (float)F, true);
                            printf("      S %.2f; lake w %.3f level %.2f depth %.2f |gw| %.3g |gl| %.3g |gd| %.3g; shore h %.2f scale %.3f |grad| %.3g; river h %.2f scale %.3f |grad| %.3g\n",
                                S0, Lk2.Weight, Lk2.Level, Lk2.Depth, std::sqrt(dot(Lk2.WeightGrad, Lk2.WeightGrad)), std::sqrt(dot(Lk2.LevelGrad, Lk2.LevelGrad)),
                                std::sqrt(dot(Lk2.DepthGrad, Lk2.DepthGrad)), Sh.Height, Sh.Scale, std::sqrt(dot(Sh.Gradient, Sh.Gradient)), Rv.Height, Rv.Scale,
                                std::sqrt(dot(Rv.Gradient, Rv.Gradient)));
                        }
                        double SPd[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius }, Qd[3], DWd[3][3];
                        for (int a = 0; a < 3; a++) { Qd[a] = SPd[a] + Evaluate(P.W[a], SPd, 0.0, DWd[a]) * P.WarpAmp; for (int b = 0; b < 3; b++) DWd[a][b] *= P.WarpAmp; }
                        const auto Sf = Smp.VoxelLandSurfaceDF(DF(SPd[0]), DF(SPd[1]), DF(SPd[2]), DF(Qd[0]), DF(Qd[1]), DF(Qd[2]),
                            float3((float)DWd[0][0], (float)DWd[0][1], (float)DWd[0][2]), float3((float)DWd[1][0], (float)DWd[1][1], (float)DWd[1][2]),
                            float3((float)DWd[2][0], (float)DWd[2][1], (float)DWd[2][2]), (float)F, true);
                        double StrataFd[2], StrataAn[2];
                        for (int e = 0; e < 2; e++)
                        {
                            const double* Ev = e == 0 ? E1 : E2;
                            double Da[3], Db[3]; Move(P, D, Ev, 1.0, Da); Move(P, D, Ev, -1.0, Db);
                            auto StrataAt = [&](const double* Dir) {
                                double Sp[3] = { Dir[0] * P.SeaRadius, Dir[1] * P.SeaRadius, Dir[2] * P.SeaRadius }, Qq[3], Dw[3][3];
                                for (int a = 0; a < 3; a++) { Qq[a] = Sp[a] + Evaluate(P.W[a], Sp, 0.0, Dw[a]) * P.WarpAmp; for (int b = 0; b < 3; b++) Dw[a][b] *= P.WarpAmp; }
                                return Smp.VoxelLandSurfaceDF(DF(Sp[0]), DF(Sp[1]), DF(Sp[2]), DF(Qq[0]), DF(Qq[1]), DF(Qq[2]),
                                    float3((float)Dw[0][0], (float)Dw[0][1], (float)Dw[0][2]), float3((float)Dw[1][0], (float)Dw[1][1], (float)Dw[1][2]),
                                    float3((float)Dw[2][0], (float)Dw[2][1], (float)Dw[2][2]), (float)F, true).Strata;
                            };
                            StrataFd[e] = (StrataAt(Da) - StrataAt(Db)) / 2.0;
                            {
                                auto CarvedAt = [&](const double* Dir) {
                                    double Sp[3] = { Dir[0] * P.SeaRadius, Dir[1] * P.SeaRadius, Dir[2] * P.SeaRadius }, Qq[3], Dw[3][3];
                                    for (int a = 0; a < 3; a++) { Qq[a] = Sp[a] + Evaluate(P.W[a], Sp, 0.0, Dw[a]) * P.WarpAmp; for (int b = 0; b < 3; b++) Dw[a][b] *= P.WarpAmp; }
                                    const auto L = Smp.VoxelLandSurfaceDF(DF(Sp[0]), DF(Sp[1]), DF(Sp[2]), DF(Qq[0]), DF(Qq[1]), DF(Qq[2]),
                                        float3((float)Dw[0][0], (float)Dw[0][1], (float)Dw[0][2]), float3((float)Dw[1][0], (float)Dw[1][1], (float)Dw[1][2]),
                                        float3((float)Dw[2][0], (float)Dw[2][1], (float)Dw[2][2]), (float)F, true);
                                    return (double)L.Bake + L.Hills + L.Peaks + L.Gully + L.Carve;
                                };
                                double Da2[3], Db2[3]; Move(P, D, Ev, 0.1, Da2); Move(P, D, Ev, -0.1, Db2);
                                printf("    carved surface slope e%d: fd %.4f, the carve's own gradient %.4f (scale %.3f)\n", e, (CarvedAt(Da2) - CarvedAt(Db2)) / 0.2,
                                    Sf.CarveGradient.x * Ev[0] + Sf.CarveGradient.y * Ev[1] + Sf.CarveGradient.z * Ev[2], Sf.CarveScale);
                            }
                            StrataAn[e] = Sf.StrataGradient.x * Ev[0] + Sf.StrataGradient.y * Ev[1] + Sf.StrataGradient.z * Ev[2];
                        }
                        printf("    strata gradient: fd (%.3g %.3g) analytic (%.3g %.3g)\n", StrataFd[0], StrataFd[1], StrataAn[0], StrataAn[1]);
                        printf("    err %.2f deg: ref (%.3f %.3f) analytic (%.3f %.3f); along %.4f across %.4f; river d %.0f m of valley %.0f m (width %.0f, area %.0f km^2, T %.2f, seg %u); lake %.2f; strata %.3f |grad| %.3g; carve %.1f scale %.3f |carve grad| %.3g\n",
                            ErrV[k], Ref[0], Ref[1], An[0], An[1], OnAlong, OnSide, R.Distance, R.bFound ? Smp.VoxelLandRiverValleyHalfWidth(Smp.VoxelLandRiverBelt(R.Log2Area)) : 0.0, Wd,
                            std::exp2(R.Log2Area) * 1e-6, R.T, R.Segment, Lk.Weight, Sf.Strata, std::sqrt(dot(Sf.StrataGradient, Sf.StrataGradient)), Sf.Carve, Sf.CarveScale,
                            std::sqrt(dot(Sf.CarveGradient, Sf.CarveGradient)));
                    }
                    // The backends: the global chain's height (above sea) against the CPU's.
                    const auto H = At(D);
                    const double Gpu = ((H.Height.x - LandHeightOffset.x) + ((double)H.Height.y - LandHeightOffset.y)) * 0.01;
                    FSampler SS = P.S;
                    if (!bCarve) { SS.LandRiverCarve = 0; SS.LandHasWater = 0; }
                    BackV[k] = std::fabs(Gpu - TermsWith(P, SS, D, F).Height);
                });
                printf("gradient near rivers and lakes, footprint %.0f m, carve %s: normal error (deg) p50 %.3f p90 %.3f p99 %.3f max %.2f; global chain against the CPU's |dh| p99 %.4f max %.4f m\n",
                    F * 0.01, bCarve ? "on " : "off", Pct(ErrV, 0.5), Pct(ErrV, 0.9), Pct(ErrV, 0.99), Pct(ErrV, 1.0), Pct(BackV, 0.99), Pct(BackV, 1.0));
            }
        }
        VoxelShader::LandRiverCarve = P.S.LandRiverCarve;
        VoxelShader::LandHasWater = P.S.LandHasWater;
    }
    else if (Mode == "cost")
    {
        // The CPU sample's cost, carve on and off: random land points, and points in corridors.
        std::uniform_real_distribution<double> U(0, 1);
        std::normal_distribution<double> G;
        std::vector<std::array<double, 3>> Land, Near;
        while (Land.size() < 20000)
        {
            double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
            if (TermsWith(P, Off, D, P.Finest).Bake > 0) Land.push_back({ D[0], D[1], D[2] });
        }
        for (int i = 0; i < 20000; i++)
        {
            const FRiverSite& S = All[(size_t)(U(Rng) * All.size()) % All.size()];
            double C[3]; Unwarp(P, S.Q, C);
            double O[3]; Move(P, C, S.Side, (U(Rng) * 2 - 1) * S.Valley, O);
            Near.push_back({ O[0], O[1], O[2] });
        }
        for (int Pass = 0; Pass < 2; Pass++)
            for (const auto* Set : { &Land, &Near })
                for (int bCarve = 0; bCarve < 2; bCarve++)
                {
                    const FSampler& S = bCarve ? P.S : Off;
                    const double T0 = Now();
                    double Sink = 0;
                    for (const auto& D : *Set) Sink += TermsWith(P, S, D.data(), P.Finest).Height;
                    if (Pass == 1)
                        printf("%s, carve %s: %.2f us a sample (%g)\n", Set == &Land ? "land" : "corridors", bCarve ? "on " : "off", (Now() - T0) / Set->size() * 1e6, Sink * 1e-30);
                }
    }
    else if (Mode == "diag")
    {
        // At the river of log2 area WANT (34) in a WIDTH-wide view: the carve (blue down), the
        // lakes' weight (red), and where the surface was below the nearest river's floor (green).
        const double Want = getenv("WANT") ? atof(getenv("WANT")) : 34.0;
        const FRiverSite* Best = nullptr;
        for (const auto& S : All)
            if (S.Log2Area > Want && S.Log2Area < Want + 1 && S.Floor > 50 && (!Best || std::fabs(S.Floor - 600) < std::fabs(Best->Floor - 600))) Best = &S;
        if (!Best) return 1;
        double C[3]; Unwarp(P, Best->Q, C);
        const double W = getenv("WIDTH") ? atof(getenv("WIDTH")) : 4.0 * Best->Valley;
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        double E1[3], E2[3]; Frame(C, E1, E2);
        std::vector<float3> Img(Px * Px);
        ParallelFor(Px * Px, [&](int k) {
            const int x = k % Px, y = k / Px;
            const double U = ((x + 0.5) / Px - 0.5) * W * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * W * 100.0 / P.SeaRadius;
            double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
            Normalize(D);
            FSampler Smp = P.S;
            const FTerms T = TermsWith(P, Smp, D, W / Px * 100.0);
            double Q[3]; Warped(P, D, Q);
            const auto L = Smp.VoxelLandLakeAt(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius), false);
            const auto R = Smp.VoxelLandRiverAtDF(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius));
            const double S1 = T.Bake + T.Hills + T.Peaks + T.Gully;
            const float c = (float)std::clamp(-T.Carve / 200.0, 0.0, 1.0);
            float3 Col(1.0f - c, 1.0f - 0.5f * c, 1.0f);
            if (L.Weight > 0) Col = Col * (1.0f - 0.6f * L.Weight) + float3(0.6f * L.Weight, 0, 0);
            if (R.bFound && S1 < R.Floor) Col = Col * 0.6f + float3(0, 0.4f, 0);
            Img[k] = Col;
        });
        WritePPM(Tag + "_diag.ppm", Px, Px, Img);
        // The steepest carved pixels: their terms.
        {
            std::vector<double> H(Px * Px), H0(Px * Px);
            ParallelFor(Px * Px, [&](int k) {
                const int x = k % Px, y = k / Px;
                const double U = ((x + 0.5) / Px - 0.5) * W * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * W * 100.0 / P.SeaRadius;
                double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
                Normalize(D);
                H[k] = TermsWith(P, P.S, D, W / Px * 100.0).Height;
                H0[k] = TermsWith(P, Off, D, W / Px * 100.0).Height;
            });
            const double Pix = W / Px;
            std::vector<std::pair<double, int>> Steep;
            for (int y = 1; y < Px - 1; y++)
                for (int x = 1; x < Px - 1; x++)
                {
                    const int k = y * Px + x;
                    const double S = std::hypot(H[k + 1] - H[k - 1], H[k + Px] - H[k - Px]) / (2 * Pix);
                    const double S0 = std::hypot(H0[k + 1] - H0[k - 1], H0[k + Px] - H0[k - Px]) / (2 * Pix);
                    Steep.push_back({ S - S0, k });
                }
            std::sort(Steep.rbegin(), Steep.rend());
            for (int i = 0; i < 5; i++)
            {
                const int k = Steep[i].second, x = k % Px, y = k / Px;
                const double U = ((x + 0.5) / Px - 0.5) * W * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * W * 100.0 / P.SeaRadius;
                double D[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
                Normalize(D);
                double Q[3]; Warped(P, D, Q);
                FSampler Smp = P.S;
                const auto R = Smp.VoxelLandRiverAtDF(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius));
                const FTerms T = TermsWith(P, P.S, D, Pix * 100.0);
                const double Wd = R.bFound ? Smp.VoxelLandRiverWidth(R.Log2Area) : 0;
                printf("  steeper by %.2f at pixel (%d, %d): height %.0f m (uncarved %.0f), carve %.0f, river %s: distance %.0f m, floor %.0f m, %.0f km^2, width %.0f m, valley %.0f m\n",
                    Steep[i].first, x, y, H[k], H0[k], T.Carve, R.bFound ? "found" : "none", R.Distance, R.Floor, std::exp2(R.Log2Area) * 1e-6, Wd,
                    R.bFound ? Smp.VoxelLandRiverValleyHalfWidth(Smp.VoxelLandRiverBelt(R.Log2Area)) : 0.0);
            }
        }
        printf("diag: river of %.0f km^2 at floor %.0f m\n", std::exp2(Best->Log2Area) * 1e-6, Best->Floor);
    }
    else if (Mode == "lakeviews")
    {
        // Hillshades at a few lakes' shores (the largest lakes' edge cells), carve on and off.
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        const int Side = P.Side();
        std::vector<std::pair<int, int>> Count;   // (lake cells at this level, an edge cell)
        std::map<float, std::pair<int, int>> Lakes;
        for (int f = 0; f < 6; f++)
            for (int j = 1; j < N - 1; j++)
                for (int i = 1; i < N - 1; i++)
                {
                    const int c = f * Side * Side + (j + P.Pad) * Side + i + P.Pad;
                    if (!(P.WaterF[c] > -1e30f)) continue;
                    auto& L = Lakes[P.WaterF[c]];
                    L.first++;
                    if (!(P.WaterF[c + 1] > -1e30f) && P.WaterF[c] > 50.0f) L.second = c;
                }
        for (auto& [Level, L] : Lakes) if (L.second) Count.push_back({ L.first, L.second });
        std::sort(Count.rbegin(), Count.rend());
        for (int v = 0; v < std::min<int>(3, (int)Count.size()); v++)
        {
            const int c = Count[v].second;
            const int f = c / (Side * Side), r = c % (Side * Side), j = r / Side - P.Pad, i = r % Side - P.Pad;
            const float2 TA = P.Tan[i + P.Pad], TB = P.Tan[j + P.Pad];
            const float3 Nf = VoxelCubeAxisVector(VoxelCubeNormalAxis(f)) * VoxelCubeNormalSign(f);
            const float3 Uf = VoxelCubeAxisVector(VoxelCubeUAxis(f)), Vf = VoxelCubeAxisVector(VoxelCubeVAxis(f));
            double Q[3] = { Nf.x + Uf.x * (TA.x + TA.y) + Vf.x * (TB.x + TB.y), Nf.y + Uf.y * (TA.x + TA.y) + Vf.y * (TB.x + TB.y), Nf.z + Uf.z * (TA.x + TA.y) + Vf.z * (TB.x + TB.y) };
            Normalize(Q);
            double C[3]; Unwarp(P, Q, C);
            for (double W : { 3000.0, 12000.0 })
                for (int bCarve = 0; bCarve < 2; bCarve++)
                {
                    const FSampler Keep = P.S;
                    if (!bCarve) { P.S.LandRiverCarve = 0; P.S.LandHasWater = 0; }
                    std::vector<double> H;
                    std::vector<float3> Img = Render(P, C, W, Px, 1, &H);
                    P.S = Keep;
                    // The water over it: blue where the ground is under the level.
                    for (size_t k = 0; k < H.size(); k++)
                        if (H[k] < P.WaterF[c]) Img[k] = Img[k] * 0.5f + float3(0.1f, 0.2f, 0.45f);
                    char Name[256]; snprintf(Name, sizeof Name, "%s_lake%d_%.0fm_%s.ppm", Tag.c_str(), v, W, bCarve ? "on" : "off");
                    WritePPM(Name, Px, Px, Img);
                    printf("%s: a lake of %d cells at %.1f m\n", Name, Count[v].first, P.WaterF[c]);
                }
        }
    }
    else if (Mode == "dcmap")
    {
        // Debug: around the largest gentle river's site, each entry's channel distance (min over
        // entries: dark in the channel), coloured by the winning segment; and a transect.
        FSampler Smp = P.S;
        const FRiverSite* Site = nullptr;
        for (const auto& S : All)
        {
            const auto R = Smp.VoxelLandRiverSegAt(S.Seg);
            const double Lambda = 11.0 * S.Width;
            const bool bInside = R.S0 > 3.0 * Lambda && R.ChainLength - R.S0 - R.Length > 3.0 * Lambda;
            if (S.Floor > 20 && std::max({ R.Grade, R.GradeAbove, R.GradeBelow }) < 0.002f && bInside && (!Site || S.Log2Area > Site->Log2Area)) Site = &S;
        }
        if (!Site) return 0;
        FSampler Off0 = P.S; Off0.LandRiverCarve = 0.0f; Off0.LandHasWater = 0;
        const double W = getenv("WIDTH") ? atof(getenv("WIDTH")) : 3000.0;
        const int Px = 300;
        double E1[3], E2[3]; Frame(Site->Q, E1, E2);
        std::vector<float3> Img(Px * Px);
        auto Eval = [&](const double* Q, float& BestDc, uint32_t& BestSeg, float& BestU, float& BestOff, float& BestAcross) {
            const float2 QX = DF(Q[0] * P.SeaRadius), QY = DF(Q[1] * P.SeaRadius), QZ = DF(Q[2] * P.SeaRadius);
            const uint Tile = Smp.VoxelLandRiverTileOf(normalize(float3(QX.x, QY.x, QZ.x)));
            const uint F = Smp.LandRiverTiles[Tile], End = Smp.LandRiverTiles[Tile + 1];
            BestDc = 1e9f; BestSeg = 0; BestU = 0; BestOff = 0; BestAcross = 0;
            if (F >= End) return;
            const float* Dd = Smp.LandRiverData;
            const float3 C(Dd[F], Dd[F + 1], Dd[F + 2]), T1(Dd[F + 4], Dd[F + 5], Dd[F + 6]), T2(Dd[F + 8], Dd[F + 9], Dd[F + 10]);
            const float2 DC = Smp.VoxelLandRiverDot(QX, QY, QZ, C);
            const float X = VoxelDFDiv(Smp.VoxelLandRiverDot(QX, QY, QZ, T1), DC).x * Smp.LandRiverRadiusM;
            const float Y = VoxelDFDiv(Smp.VoxelLandRiverDot(QX, QY, QZ, T2), DC).x * Smp.LandRiverRadiusM;
            const float Hand = dot(cross(T1, T2), C) < 0.0f ? -1.0f : 1.0f;
            for (uint At = F + VOXEL_LAND_RIVER_FRAME_FLOATS; At < End; At += VOXEL_LAND_RIVER_ENTRY_FLOATS)
            {
                const float EX = X - Dd[At], EY = Y - Dd[At + 1], DX = Dd[At + 2], DY = Dd[At + 3];
                const auto N = Smp.VoxelLandRiverNearAt(X, Y, EX, EY, DX, DY, Dd[At + 4], 1.0f / Smp.LandRiverRadiusM);
                if (N.D * N.D > Dd[At + 5]) continue;
                const auto R = Smp.VoxelLandRiverSegAt((uint)Dd[At + 6]);
                const auto Ch = Smp.VoxelLandRiverChannelAt(R, N.U, X, Y, EX, EY, DX, DY, Hand, N.Stretch, 1.0f / Smp.LandRiverRadiusM, 0.0f);
                const float Dc = Ch.Distance;
                if (Dc < BestDc) { BestDc = Dc; BestSeg = (uint)Dd[At + 6]; BestU = N.U; BestOff = Ch.U; BestAcross = N.D; }
            }
        };
        for (int j = 0; j < Px; j++)
            for (int i = 0; i < Px; i++)
            {
                double T[3], Q[3];
                Move(P, Site->Q, E1, (i + 0.5 - Px * 0.5) * W / Px, T);
                Move(P, T, E2, (j + 0.5 - Px * 0.5) * W / Px, Q);
                float Dc, U, Off, Ac; uint32_t Seg;
                Eval(Q, Dc, Seg, U, Off, Ac);
                const float G = std::min(Dc / (float)Site->Width, 1.0f);
                const uint32_t H = Seg * 2654435761u;
                const float3 Hue((H & 255) / 255.0f, ((H >> 8) & 255) / 255.0f, ((H >> 16) & 255) / 255.0f);
                Img[j * Px + i] = float3(Hue.x * 0.5f + 0.5f * G, Hue.y * 0.5f + 0.5f * G, Hue.z * 0.5f + 0.5f * G);
            }
        WritePPM(Tag + "_dcmap.ppm", Px, Px, Img);
        {
            const auto R = Smp.VoxelLandRiverSegAt(Site->Seg);
            printf("site seg %u: F %.1f %.1f A %.2f %.2f s0 %.0f L %.0f chain %.0f turns %.3f %.3f grades %.4f %.4f %.4f phase %.3f span %.3f; lambda %.0f\n", Site->Seg, R.F0, R.F1, R.A0, R.A1,
                R.S0, R.Length, R.ChainLength, R.Turn0, R.Turn1, R.GradeAbove, R.Grade, R.GradeBelow, R.Phase0, R.PhaseSpan, 11.0 * Site->Width);
        }
        for (int k = -24; k <= 24; k++)
        {
            double Q[3]; Move(P, Site->Q, Site->Side, k * Site->Width * 0.25, Q);
            float Dc, U, Off, Ac; uint32_t Seg;
            Eval(Q, Dc, Seg, U, Off, Ac);
            double D[3]; Unwarp(P, Q, D);
            const FTerms T = TermsWith(P, P.S, D, P.Finest), T0 = TermsWith(P, Off0, D, P.Finest);
            printf("  %+6.0f m: seg %u u %.3f channel's u %.3f channel distance %.1f (line %.1f); height %.2f (uncarved %.2f)\n", k * Site->Width * 0.25, Seg, U, Off, Dc, Ac, T.Height, T0.Height);
        }
    }
    else if (Mode == "r3views")
    {
        // R3's features, carve on and off: the largest river on a gentle reach (meanders), the
        // highest fall, and a river in the strata (a canyon). PX overrides the size.
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        FSampler Smp = P.S;
        auto Surface = [&](const double* D) {
            double SP[3] = { D[0] * P.SeaRadius, D[1] * P.SeaRadius, D[2] * P.SeaRadius }, Q[3], DW[3][3];
            for (int a = 0; a < 3; a++) { Q[a] = SP[a] + Evaluate(P.W[a], SP, 0.0, DW[a]) * P.WarpAmp; for (int b = 0; b < 3; b++) DW[a][b] *= P.WarpAmp; }
            return Smp.VoxelLandSurfaceDF(DF(SP[0]), DF(SP[1]), DF(SP[2]), DF(Q[0]), DF(Q[1]), DF(Q[2]),
                float3((float)DW[0][0], (float)DW[0][1], (float)DW[0][2]), float3((float)DW[1][0], (float)DW[1][1], (float)DW[1][2]),
                float3((float)DW[2][0], (float)DW[2][1], (float)DW[2][2]), (float)P.Finest, false);
        };
        const FRiverSite* Meander = nullptr;
        const FRiverSite* Fall = nullptr;
        const FRiverSite* Canyon = nullptr;
        double FallBest = 0;
        int Gentle = 0, Falls = 0, InStrata = 0;
        for (size_t i = 0; i < All.size(); i++)
        {
            const FRiverSite& S = All[i];
            const auto R = Smp.VoxelLandRiverSegAt(S.Seg);
            if (S.Floor < 20) continue;
            const double Smooth = std::max({ (double)R.Grade, (double)R.GradeAbove, (double)R.GradeBelow });
            const double Lambda = 11.0 * S.Width;
            const bool bInside = R.S0 > 3.0 * Lambda && R.ChainLength - R.S0 - R.Length > 3.0 * Lambda;
            if (Smooth < 0.002 && bInside) { Gentle++; if (!Meander || S.Log2Area > Meander->Log2Area) Meander = &S; }
            const double FallM = R.FallShare * (R.F0 - R.F1);
            if (FallM > 5.0) Falls++;
            if (FallM > FallBest) { FallBest = FallM; Fall = &S; }
            if (i % 5 == 0 && S.Log2Area > 29.0 && R.Grade > 0.005f && R.Grade < 0.03f && R.GradeAbove > 0.003f && R.GradeBelow > 0.003f)
            {
                double D[3]; Unwarp(P, S.Q, D);
                if (Surface(D).Strata > 0.95f) { InStrata++; if (!Canyon || S.Log2Area > Canyon->Log2Area) Canyon = &S; }
            }
        }
        printf("sites: %d on gentle reaches (under 0.2%% with their neighbours), %d falls over 5 m (highest %.0f m), %d sampled in the strata\n", Gentle, Falls, FallBest, InStrata);
        // The fall's profile along its segment: the carved surface on the line every 10 m.
        if (Fall)
        {
            const auto R = Smp.VoxelLandRiverSegAt(Fall->Seg);
            printf("fall: segment %u, %.0f m long, floors %.1f .. %.1f, lip at %.2f (+-%.3f), share %.2f\n", Fall->Seg, R.Length, R.F0, R.F1, R.LipU, R.LipHalf, R.FallShare);
            double Prev = 0;
            for (int k = -40; k <= 40; k++)
            {
                double Q[3]; Move(P, Fall->Q, Fall->Along, k * 10.0, Q);
                double D[3]; Unwarp(P, Q, D);
                const FTerms T = TermsWith(P, P.S, D, P.Finest);
                const auto L = Smp.VoxelLandRiverAtDF(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius));
                if (k % 4 == 0 || std::fabs(T.Height - Prev) > 3.0)
                    printf("  %+5d m: height %.2f (step %+.2f), the line's segment %u at %.3f, %.1f m off it\n", k * 10, T.Height, T.Height - Prev, L.Segment, L.T, L.Distance);
                Prev = T.Height;
            }
        }
        struct FView { const char* Name; const FRiverSite* Site; double W0, W1; };
        const FView Views[] = { { "meander", Meander, 3000.0, 9000.0 }, { "fall", Fall, 1200.0, 5000.0 }, { "canyon", Canyon, 3000.0, 10000.0 } };
        for (const FView& V : Views)
        {
            if (!V.Site) { printf("%s: none found\n", V.Name); continue; }
            double C[3]; Unwarp(P, V.Site->Q, C);
            for (double W : { V.W0, V.W1 })
                for (int bCarve = 0; bCarve < 2; bCarve++)
                {
                    const FSampler Keep = P.S;
                    if (!bCarve) { P.S.LandRiverCarve = 0; P.S.LandHasWater = 0; }
                    const std::vector<float3> Img = Render(P, C, W, Px, 1);
                    P.S = Keep;
                    char Name[256]; snprintf(Name, sizeof Name, "%s_r3_%s_%.0fm_%s.ppm", Tag.c_str(), V.Name, W, bCarve ? "on" : "off");
                    WritePPM(Name, Px, Px, Img);
                    const auto R = Smp.VoxelLandRiverSegAt(V.Site->Seg);
                    printf("%s: river of %.0f km^2 (width %.0f m, depth %.1f m, valley %.0f m), floor %.0f m, grade %.4f (neighbours %.4f %.4f), fall %.0f m\n", Name,
                        std::exp2(V.Site->Log2Area) * 1e-6, V.Site->Width, V.Site->Depth, V.Site->Valley, V.Site->Floor, R.Grade, R.GradeAbove, R.GradeBelow,
                        R.FallShare * (R.F0 - R.F1));
                }
        }
    }
    else if (Mode == "views")
    {
        // Hillshades around rivers of a few sizes (log2 area near 28, 31, 34: 270, 2,100,
        // 17,000 km^2) and at a lake's shore, carve on and off; WIDTH (m) and PX override.
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        for (double Want : { 28.0, 31.0, 34.0 })
        {
            const FRiverSite* Best = nullptr;
            for (const auto& S : All)
                if (S.Log2Area > Want && S.Log2Area < Want + 1 && S.Floor > 50 && (!Best || std::fabs(S.Floor - 600) < std::fabs(Best->Floor - 600))) Best = &S;
            if (!Best) continue;
            double C[3]; Unwarp(P, Best->Q, C);
            for (double Width : { 4.0 * Best->Valley, 12.0 * Best->Valley })
            {
                const double W = getenv("WIDTH") ? atof(getenv("WIDTH")) : Width;
                for (int bCarve = 0; bCarve < 2; bCarve++)
                {
                    const FSampler Keep = P.S;
                    if (!bCarve) { P.S.LandRiverCarve = 0; P.S.LandHasWater = 0; }
                    const std::vector<float3> Img = Render(P, C, W, Px, 1);
                    P.S = Keep;
                    char Name[256]; snprintf(Name, sizeof Name, "%s_carve_a%.0f_%.0fm_%s.ppm", Tag.c_str(), Want, W, bCarve ? "on" : "off");
                    WritePPM(Name, Px, Px, Img);
                    printf("%s: river of %.0f km^2 (width %.0f m, depth %.1f m, valley %.0f m), floor %.0f m\n", Name, std::exp2(Best->Log2Area) * 1e-6, Best->Width, Best->Depth, Best->Valley, Best->Floor);
                }
            }
        }
    }
    else if (Mode == "meet")
    {
        // Hillshades at confluences in hill country (two chains' valleys meeting), the walls'
        // meeting rounded (N1b) and not; MEET (a factor), WIDTH (m), PX, COUNT override.
        const int Px = getenv("PX") ? atoi(getenv("PX")) : 400;
        const int Count = getenv("COUNT") ? atoi(getenv("COUNT")) : 4;
        const auto& R = P.Rivers;
        std::vector<int> Ups(R.NumNodes, 0);
        for (uint32_t n = 0; n < R.NumNodes; n++)
            if (R.NodeDown[n] < R.NumNodes) Ups[R.NodeDown[n]]++;
        // Junctions on a line, on land well above the sea, of a river of 500 .. 20,000 km^2.
        std::vector<std::pair<double, uint32_t>> Pick;
        FSampler Smp = P.S;
        for (uint32_t n = 0; n < R.NumNodes; n++)
        {
            if (Ups[n] < 2 || R.NodeKind[n] != 0) continue;
            double Q[3] = { R.NodePos[4 * n], R.NodePos[4 * n + 1], R.NodePos[4 * n + 2] };
            Normalize(Q);
            const auto L = Smp.VoxelLandRiverAtDF(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius));
            if (!L.bFound || L.Distance > 5.0f || L.Floor < 200.0f || L.Log2Area < 29.0f || L.Log2Area > 34.0f) continue;
            double D[3]; Unwarp(P, Q, D);
            const double Relief = TermsWith(P, Off, D, P.Finest).Height - L.Floor;
            Pick.push_back({ -Relief, n });
        }
        std::sort(Pick.begin(), Pick.end());
        for (int k = 0; k < Count && k < (int)Pick.size(); k++)
        {
            const uint32_t n = Pick[(size_t)k * Pick.size() / Count].second;
            double Q[3] = { R.NodePos[4 * n], R.NodePos[4 * n + 1], R.NodePos[4 * n + 2] };
            Normalize(Q);
            const auto L = Smp.VoxelLandRiverAtDF(DF(Q[0] * P.SeaRadius), DF(Q[1] * P.SeaRadius), DF(Q[2] * P.SeaRadius));
            const double Valley = Smp.VoxelLandRiverValleyHalfWidth(Smp.VoxelLandRiverBelt(L.Log2Area));
            double C[3]; Unwarp(P, Q, C);
            const double W = getenv("WIDTH") ? atof(getenv("WIDTH")) : 3.0 * Valley;
            double MaxDiff = 0, SumDiff = 0;
            std::vector<float3> Imgs[2];
            std::vector<double> Hs[2];
            for (int bRound = 0; bRound < 2; bRound++)
            {
                GMeetRound = bRound ? (getenv("MEET") ? (float)atof(getenv("MEET")) : 1.0f) : 0.0f;
                Imgs[bRound] = Render(P, C, W, Px, 1, &Hs[bRound]);
                char Name[256]; snprintf(Name, sizeof Name, "%s_meet_%d_%s.ppm", Tag.c_str(), k, bRound ? "on" : "off");
                WritePPM(Name, Px, Px, Imgs[bRound]);
            }
            GMeetRound = getenv("MEET") ? (float)atof(getenv("MEET")) : 1.0f;
            size_t MaxAt = 0;
            for (size_t i = 0; i < Hs[0].size(); i++) { const double Dh = Hs[0][i] - Hs[1][i]; if (Dh > MaxDiff) { MaxDiff = Dh; MaxAt = i; } SumDiff += Dh; }
            // A closer view (a fifth of the width) where it lowers most: the crease.
            {
                double E1[3], E2[3]; Frame(C, E1, E2);
                const int x = (int)(MaxAt % Px), y = (int)(MaxAt / Px);
                const double U = ((x + 0.5) / Px - 0.5) * W * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * W * 100.0 / P.SeaRadius;
                double Z[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
                Normalize(Z);
                for (int bRound = 0; bRound < 2; bRound++)
                {
                    GMeetRound = bRound ? (getenv("MEET") ? (float)atof(getenv("MEET")) : 1.0f) : 0.0f;
                    const std::vector<float3> Img = Render(P, Z, 0.2 * W, Px, 1);
                    char Name[256]; snprintf(Name, sizeof Name, "%s_meetz_%d_%s.ppm", Tag.c_str(), k, bRound ? "on" : "off");
                    WritePPM(Name, Px, Px, Img);
                }
                GMeetRound = getenv("MEET") ? (float)atof(getenv("MEET")) : 1.0f;
            }
            // The cache writer's gradient against central differences where the rounding acts
            // (over 5 cm), rounded and not.
            double GradErr[2][3] = {};
            {
                double E1[3], E2[3]; Frame(C, E1, E2);
                std::vector<std::array<double, 3>> Pts;
                for (size_t i = 0; i < Hs[0].size(); i += 7)
                    if (Hs[0][i] - Hs[1][i] > 0.05)
                    {
                        const int x = (int)(i % Px), y = (int)(i / Px);
                        const double U = ((x + 0.5) / Px - 0.5) * W * 100.0 / P.SeaRadius, V = (0.5 - (y + 0.5) / Px) * W * 100.0 / P.SeaRadius;
                        double Z[3] = { C[0] + U * E1[0] + V * E2[0], C[1] + U * E1[1] + V * E2[1], C[2] + U * E1[2] + V * E2[2] };
                        Normalize(Z);
                        Pts.push_back({ Z[0], Z[1], Z[2] });
                    }
                for (int bRound = 0; bRound < 2 && !Pts.empty(); bRound++)
                {
                    GMeetRound = bRound ? (getenv("MEET") ? (float)atof(getenv("MEET")) : 1.0f) : 0.0f;
                    std::vector<double> Err(Pts.size());
                    const float F = 100.0f;
                    ParallelFor((int)Pts.size(), [&](int q) {
                        using namespace VoxelShader;
                        const double* D = Pts[q].data();
                        auto At = [&](const double* Dir) {
                            return VoxelLandHeightDF(DF(Dir[0] * P.SeaRadius), DF(Dir[1] * P.SeaRadius), DF(Dir[2] * P.SeaRadius), F, false);
                        };
                        const FVoxelLandCacheSurface CS = VoxelLandCacheSurfaceAt(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), F, 25.0f);
                        double F1[3], F2[3]; Frame(D, F1, F2);
                        double Ref[2], An[2];
                        for (int e = 0; e < 2; e++)
                        {
                            const double* E = e == 0 ? F1 : F2;
                            double A[3], B[3], A2[3], B2[3];
                            Move(P, D, E, 0.1, A); Move(P, D, E, -0.1, B); Move(P, D, E, 0.2, A2); Move(P, D, E, -0.2, B2);
                            const auto HA = At(A), HB = At(B), HA2 = At(A2), HB2 = At(B2);
                            const double G1 = ((HA.Height.x - HB.Height.x) + ((double)HA.Height.y - HB.Height.y)) * 0.01 / 0.2;
                            const double G2 = ((HA2.Height.x - HB2.Height.x) + ((double)HA2.Height.y - HB2.Height.y)) * 0.01 / 0.4;
                            Ref[e] = (4 * G1 - G2) / 3;
                            An[e] = CS.Gradient.x * E[0] + CS.Gradient.y * E[1] + CS.Gradient.z * E[2];
                        }
                        const double Dot = (1 + An[0] * Ref[0] + An[1] * Ref[1]) / (std::sqrt(1 + An[0] * An[0] + An[1] * An[1]) * std::sqrt(1 + Ref[0] * Ref[0] + Ref[1] * Ref[1]));
                        Err[q] = std::acos(std::min(1.0, Dot)) * 57.2957795;
                    });
                    GradErr[bRound][0] = Pct(Err, 0.5); GradErr[bRound][1] = Pct(Err, 0.99); GradErr[bRound][2] = Pct(Err, 1.0);
                }
                GMeetRound = getenv("MEET") ? (float)atof(getenv("MEET")) : 1.0f;
                printf("  %zu points where it rounds: normal error (deg) rounded p50 %.3f p99 %.3f max %.2f; not p50 %.3f p99 %.3f max %.2f\n",
                    Pts.size(), GradErr[1][0], GradErr[1][1], GradErr[1][2], GradErr[0][0], GradErr[0][1], GradErr[0][2]);
            }
            printf("%s_meet_%d: junction of %d at a river of %.0f km^2 (valley %.0f m), floor %.0f m, %.0f m wide: rounding lowers by up to %.1f m (mean %.2f m)\n",
                Tag.c_str(), k, Ups[n], std::exp2(L.Log2Area) * 1e-6, Valley, L.Floor, W, MaxDiff, Hs[0].empty() ? 0.0 : SumDiff / Hs[0].size());
        }
    }
    return 0;
}
