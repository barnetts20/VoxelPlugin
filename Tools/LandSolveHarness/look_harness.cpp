// Look harness (Design/SurfaceLook.md, T0): the shipped VoxelLandSolve.ush and VoxelLandPlates.ush
// compiled as C++, run in the GPU's pass sequence with the solve's settings given on the command
// line, and the bake's fields dumped for Tools/LandView/look.py (renders and measures).
//
// g++ -O2 -std=c++20 -ffp-contract=off -pthread -I<Public> -I<Shaders/Private> look_harness.cpp
// look_harness N OutTag [Name=Value ...]   (RadiusM, Seed, and the FSolve settings below)
// look_harness 0 x WarpFolds=1 [RadiusM=..] [Seed=..]   the plates' warp checked for folds
#include "VoxelShaderCompat.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <thread>
#include <random>
#include <algorithm>
#include <queue>
#include <cmath>
#include <atomic>
#include <chrono>
#include <string>

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
#include "VoxelDF64.ush"
#include "VoxelCubeSphere.ush"

    struct FPlates
    {
        const float* LandPlates = nullptr;
        uint LandPlateCount = 0;
        uint LandSeed = 0;
        float LandRadiusM = 1.0e6f;
        float LandWidthScale = 1.0f;
        float LandReliefScale = 1.0f;
        float LandShelfDepth = -140.0f;
        float LandAbyssDepth = -4200.0f;
        float LandCoastHeight = 50.0f;
        float LandInteriorHeight = 950.0f;
#include "VoxelLandPlates.ush"
    };

    struct FSolve
    {
        uint SolveN = 0, SolvePad = 4;
        float SolveRadiusM = 1.0e6f;
        const float2* SolveTan = nullptr;
        const float* BakeHeight = nullptr;
        const float* BakeUp = nullptr;
        const float* BakeCrust = nullptr;
        const uint* BakeAttr = nullptr;
        float SolvePlainsUplift = 400.0f, SolveRampWidth = 300.0f, SolveUpliftTime = 5.0e6f;
        float SolveKdt = 0.4f, SolveDt = 0.4f / 4.3e-7f, SolveEps = 0.01f;
        uint SolveSeed = 7;
        float* SolveH = nullptr; float* SolveW = nullptr; float* SolveWater = nullptr;
        uint* SolveRec = nullptr; uint* SolveSlot = nullptr; uint* SolveLabel = nullptr;
        uint* SlotCell = nullptr;
        float* SlotU = nullptr; float* SlotArea = nullptr; float* SlotDist = nullptr; float* SlotH0 = nullptr;
        float* SlotA = nullptr; float* SlotAOut = nullptr; float* SlotWOut = nullptr;
        float* JumpX = nullptr; float* JumpXOut = nullptr;
        uint* JumpY = nullptr; uint* JumpYOut = nullptr; uint* JumpP = nullptr; uint* JumpPOut = nullptr;
        uint* SolveFlags = nullptr;
        float SolveFactor = 1.0f, SolveReliefKnee = 1.0e9f, SolveReliefTop = 1.0e9f;
        float SolveMinLakeDepth = 10.0f, SolveMinLakeArea = 20.0e6f;
        float SolveJitter = 0.0f, SolveRouteNoise = 0.0f, SolveSpreadCells = 0.0f, SolveUndulation = 0.0f, SolveUpliftVariation = 0.0f, SolveErodibilityVariation = 0.0f, SolveVariationWavelength = 1.2e5f, SolveRoughness = 0.0f, SolveReliefScale = 0.0f;
        uint SolveFillDepressions = 1u, SolveModel = 0u;
        float SolvePlainsRelief = 0.0f, SolveIncision = 0.0f;
        float* SlotK = nullptr;
        uint* LakeArea = nullptr; uint* LakeDepth = nullptr; uint* LakeOutlet = nullptr;
        uint* LakeCount = nullptr; uint* LakeList = nullptr;
        uint SolveMaxLakes = 65536;
        float* OutHeight = nullptr; float* OutFlow = nullptr; float* OutWater = nullptr; uint* OutAttr = nullptr;
#include "VoxelLandSolve.ush"
    };
}
using namespace VoxelShader;

template<typename F> static void ParallelFor(int Count, F Fn)
{
    const int T = std::max(1u, std::thread::hardware_concurrency());
    std::atomic<int> Next{ 0 };
    std::vector<std::thread> Th;
    for (int t = 0; t < T; t++)
        Th.emplace_back([&]() { int i; while ((i = Next.fetch_add(256)) < Count) for (int k = i; k < std::min(i + 256, Count); k++) Fn(k); });
    for (auto& t : Th) t.join();
}

static double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

static std::vector<float> MakePlates(int K, uint32_t Seed)
{
    std::mt19937 Rng(Seed);
    std::normal_distribution<double> Normal(0.0, 1.0);
    std::uniform_real_distribution<double> Uni(0.0, 1.0);
    std::vector<float> Out(K * 8);
    for (int i = 0; i < K; i++)
    {
        const double Phi = std::acos(1.0 - 2.0 * (i + 0.5) / K);
        const double Th = M_PI * (1.0 + std::sqrt(5.0)) * (i + 0.5);
        double S[3] = { std::cos(Th) * std::sin(Phi), std::sin(Th) * std::sin(Phi), std::cos(Phi) };
        for (double& v : S) v += 0.25 * Normal(Rng);
        double L = std::sqrt(S[0] * S[0] + S[1] * S[1] + S[2] * S[2]);
        double Pole[3] = { Normal(Rng), Normal(Rng), Normal(Rng) };
        double PL = std::sqrt(Pole[0] * Pole[0] + Pole[1] * Pole[1] + Pole[2] * Pole[2]);
        const double Rate = 0.5 + Uni(Rng), Bias = -1.0 + 2.0 * Uni(Rng), Dens = Uni(Rng);
        for (int a = 0; a < 3; a++) { Out[i * 8 + a] = (float)(S[a] / L); Out[i * 8 + 4 + a] = (float)(Pole[a] / PL * Rate); }
        Out[i * 8 + 3] = (float)Bias;
        Out[i * 8 + 7] = (float)Dens;
    }
    return Out;
}

static float2 DF(double V) { const float Hi = (float)V; return float2(Hi, (float)(V - (double)Hi)); }

/** The plate pass on the padded grid: height, uplift, crust, attributes. */
struct FPlateBake
{
    int N = 256, Pad = 4;
    int Side() const { return N + 2 * Pad; }
    int Cells() const { return 6 * Side() * Side(); }
    std::vector<float> Height, Up, Crust;
    std::vector<uint> Attr;
    std::vector<float2> Tan;
    std::vector<float> Plates;
};

static void BakePlates(FPlateBake& B, uint32_t Seed, int K, float LandFraction, float RadiusM)
{
    B.Plates = MakePlates(K, Seed);
    FPlates P;
    P.LandPlates = B.Plates.data();
    P.LandPlateCount = K;
    P.LandSeed = Seed;
    P.LandRadiusM = RadiusM;
    const int S = B.Side(), N = B.N;
    const int Bins = 4096;
    std::vector<std::atomic<uint32_t>> Hist(Bins);
    ParallelFor(6 * N * N, [&](int Idx) {
        const int F = Idx / (N * N), R = Idx % (N * N), J = R / N, I = R % N;
        const float Su = VoxelCubeCellCentre(I, N), Tv = VoxelCubeCellCentre(J, N);
        const float3 D = VoxelCubeDirection(F, Su, Tv);
        const float T = P.VoxelLandRawCrust(D, P.VoxelLandNearestPlate(P.VoxelLandWarp(D)));
        const float A = std::tan(Su * 0.785398f), Bb = std::tan(Tv * 0.785398f);
        const float Solid = (1 + A * A) * (1 + Bb * Bb) / std::pow(1 + A * A + Bb * Bb, 1.5f);
        const int Bin = std::clamp((int)std::floor((T + 2.0f) / 4.0f * Bins), 0, Bins - 1);
        Hist[Bin].fetch_add((uint32_t)(Solid * 64.0f + 0.5f));
    });
    uint64_t Total = 0;
    for (auto& h : Hist) Total += h.load();
    const double Target = (1.0 - LandFraction) * (double)Total;
    uint64_t Cum = 0;
    float Threshold = 0;
    for (int b = 0; b < Bins; b++)
    {
        const uint64_t H = Hist[b].load();
        if (Cum + H >= Target && H > 0) { Threshold = (float)(-2.0 + 4.0 * (b + (Target - Cum) / (double)H) / Bins); break; }
        Cum += H;
    }
    B.Height.assign(B.Cells(), 0.f);
    B.Up.assign(B.Cells(), 0.f);
    B.Crust.assign(B.Cells(), 0.f);
    B.Attr.assign(B.Cells(), 0u);
    ParallelFor(B.Cells(), [&](int Idx) {
        const int F = Idx / (S * S), R = Idx % (S * S), J = R / S - B.Pad, I = R % S - B.Pad;
        const float3 D = VoxelCubeDirection(F, VoxelCubeCellCentre(I, N), VoxelCubeCellCentre(J, N));
        const auto E = P.VoxelLandElevationAt(D, Threshold);
        B.Height[Idx] = E.Base + E.Up - E.Down;
        B.Up[Idx] = E.Up;
        B.Crust[Idx] = E.Crust;
        B.Attr[Idx] = P.VoxelLandPackAttr(E.Crust, E.Class);
    });
    B.Tan.resize(S);
    for (int i = 0; i < S; i++)
        B.Tan[i] = DF(std::tan((-1.0 + (2.0 * (i - B.Pad) + 1.0) / N) * M_PI / 4.0));
}

/** The solve's buffers, owned. */
struct FSolveBuffers
{
    std::vector<float> H, W, Water, U, K, Area, Dist, H0, A, AOut, WOut, X, XOut;
    std::vector<uint> Rec, Slot, Label, Cell, Y, YOut, P, POut, Flags, LArea, LDepth, LOutlet, LCount, LList, OAttr;
    std::vector<float> OHeight, OFlow, OWater;

    void Bind(FSolve& S, int Cells)
    {
        auto Fz = [](std::vector<float>& V, int n) { V.assign(n, 0.f); };
        auto Uz = [](std::vector<uint>& V, int n) { V.assign(n, 0u); };
        Fz(H, Cells); Fz(W, Cells); Fz(Water, Cells); Uz(Rec, Cells); Uz(Slot, Cells); Uz(Label, Cells);
        for (auto* V : { &U, &K, &Area, &Dist, &H0, &A, &AOut, &WOut, &X, &XOut, &OHeight, &OFlow, &OWater }) Fz(*V, Cells);
        for (auto* V : { &Cell, &Y, &YOut, &P, &POut, &LArea, &LDepth, &LOutlet, &OAttr }) Uz(*V, Cells);
        Uz(Flags, 4); Uz(LCount, 1); Uz(LList, 65536 * 4);
        S.SolveH = H.data(); S.SolveW = W.data(); S.SolveWater = Water.data();
        S.SolveRec = Rec.data(); S.SolveSlot = Slot.data(); S.SolveLabel = Label.data();
        S.SlotCell = Cell.data(); S.SlotU = U.data(); S.SlotK = K.data(); S.SlotArea = Area.data(); S.SlotDist = Dist.data(); S.SlotH0 = H0.data();
        S.SlotA = A.data(); S.SlotAOut = AOut.data(); S.SlotWOut = WOut.data();
        S.JumpX = X.data(); S.JumpXOut = XOut.data(); S.JumpY = Y.data(); S.JumpYOut = YOut.data(); S.JumpP = P.data(); S.JumpPOut = POut.data();
        S.SolveFlags = Flags.data();
        S.LakeArea = LArea.data(); S.LakeDepth = LDepth.data(); S.LakeOutlet = LOutlet.data(); S.LakeCount = LCount.data(); S.LakeList = LList.data();
        S.OutHeight = OHeight.data(); S.OutFlow = OFlow.data(); S.OutWater = OWater.data(); S.OutAttr = OAttr.data();
    }
};

static void SwapJump(FSolve& S) { std::swap(S.JumpX, S.JumpXOut); std::swap(S.JumpY, S.JumpYOut); std::swap(S.JumpP, S.JumpPOut); }

/** Compaction in cell order (the GPU's scan gives the same slots). */
static uint Compact(FSolve& S, uint Domain)
{
    const uint Cells = S.VoxelSolveNumCells();
    uint Count = 0;
    for (uint c = 0; c < Cells; c++)
    {
        if (S.VoxelSolveInDomain(c, Domain)) { S.SolveSlot[c] = Count; S.SlotCell[Count] = c; Count++; }
        else S.SolveSlot[c] = VOXEL_SOLVE_NONE;
    }
    return Count;
}

/** In place until nothing changes (sequential, so it converges in few sweeps). */
template<typename F> static int UntilConverged(FSolve& S, F Pass, int Cap)
{
    for (int it = 0; it < Cap; it++)
    {
        S.SolveFlags[0] = 0;
        Pass();
        if (S.SolveFlags[0] == 0) return it + 1;
    }
    return -1;
}

struct FRunOptions
{
    int R = 20;          // pointer-jumping rounds
    int Kpd = 4;         // Jacobi refinements of the fill per step
    int Kacc = 16;       // accumulation iterations per step
    int MaxSteps = 150;
    float Tol = 0.02f;
    int MinSteps = 10;
    bool bReference = false;   // the sequential solve instead
};

struct FRunResult
{
    std::vector<float> Residuals;
    double Seconds = 0;
    uint Moving = 0, Land = 0, Errors = 0;
    float Factor = 1, P90Solved = 0, P90Target = 0;
    int Lakes = 0;
};

static float Percentile(std::vector<float> V, double Q)
{
    if (V.empty()) return 0;
    std::sort(V.begin(), V.end());
    return V[std::min(V.size() - 1, (size_t)(Q * V.size()))];
}

// --- Sequential reference (the prototype) --------------------------------------------------

/** Priority flood with Eps from every fixed cell (Barnes 2014): the least draining surface. */
static void ReferenceFill(FSolve& S, uint NumSlots, float Eps)
{
    const uint Cells = S.VoxelSolveNumCells();
    std::vector<char> Done(Cells, 0);
    typedef std::pair<float, uint> E;
    std::priority_queue<E, std::vector<E>, std::greater<E>> Q;
    for (uint c = 0; c < Cells; c++)
    {
        if (!S.VoxelSolveIsInterior(S.VoxelSolveDecode(c))) { Done[c] = 1; continue; }
        if (S.SolveSlot[c] == VOXEL_SOLVE_NONE) { Done[c] = 1; Q.push(E(S.SolveW[c], c)); }
    }
    for (uint s = 0; s < NumSlots; s++) S.SolveW[S.SlotCell[s]] = VOXEL_SOLVE_INF;
    while (!Q.empty())
    {
        const E Top = Q.top(); Q.pop();
        for (uint k = 0; k < 8; k++)
        {
            const uint M = S.VoxelSolveNeighbour(Top.second, k);
            if (M == VOXEL_SOLVE_NONE || Done[M]) continue;
            Done[M] = 1;
            S.SolveW[M] = std::max(S.SolveH[M], Top.first + Eps);
            Q.push(E(S.SolveW[M], M));
        }
    }
}

/** Accumulation in order of decreasing height, and the implicit update in increasing order. */
static void ReferenceAccumulate(FSolve& S, uint NumSlots, std::vector<uint>& Order)
{
    Order.resize(NumSlots);
    for (uint s = 0; s < NumSlots; s++) Order[s] = s;
    std::stable_sort(Order.begin(), Order.end(), [&](uint a, uint b) { return S.SolveH[S.SlotCell[a]] < S.SolveH[S.SlotCell[b]]; });
    for (uint s = 0; s < NumSlots; s++) S.SlotA[s] = S.SlotArea[s];
    for (int k = (int)NumSlots - 1; k >= 0; k--)
    {
        const uint s = Order[k], c = S.SlotCell[s], r = S.SolveRec[c];
        if (r != c && S.SolveSlot[r] != VOXEL_SOLVE_NONE) S.SlotA[S.SolveSlot[r]] += S.SlotA[s];
    }
}

static double ReferenceImplicit(FSolve& S, uint NumSlots, const std::vector<uint>& Order)
{
    double SumDh = 0;
    for (uint k = 0; k < NumSlots; k++)
    {
        const uint s = Order[k], c = S.SlotCell[s], r = S.SolveRec[c];
        const float Raised = S.SolveH[c] + S.SlotU[s] * S.SolveDt;
        float Hn;
        if (r == c) Hn = Raised;
        else
        {
            const float F = S.SolveKdt * S.SlotK[s] * std::sqrt(S.SlotA[s]) / std::max(S.SlotDist[s], 1.0f);
            const float Hr = S.SolveSlot[r] == VOXEL_SOLVE_NONE ? S.SolveW[r] : S.SolveH[r];
            Hn = (Raised + F * Hr) / (1.0f + F);
        }
        SumDh += std::fabs(Hn - S.SlotH0[s]);
        S.SolveH[c] = Hn;
    }
    return SumDh;
}

// --- The GPU's sequence ---------------------------------------------------------------------

static float ReliefTopScale = 1.3f;
static bool bReliefPower = false;

static FRunResult Run(FSolve& S, const FRunOptions& O, bool bLog)
{
    FRunResult Res;
    const double T0 = Now();
    const uint Cells = S.VoxelSolveNumCells();
    const uint M = Compact(S, 0);
    Res.Moving = M;
    for (uint c = 0; c < Cells; c++) S.VoxelSolveInitPass(c);
    S.SolveFlags[1] = 0;
    if (O.bReference) ReferenceFill(S, M, S.SolveEps);
    else
    {
        const int Its = UntilConverged(S, [&]() { for (uint s = 0; s < M; s++) S.VoxelSolveFillInPlacePass(s, M, S.SolveEps); }, 100000);
        if (bLog) printf("  initial fill: %d sweeps\n", Its);
    }
    for (uint s = 0; s < M; s++) S.VoxelSolveUndrainedPass(s, M);
    ParallelFor(M, [&](int s) { S.VoxelSolveReceiversPass(s, M, true); });
    double SumU = 0;
    for (uint s = 0; s < M; s++) SumU += S.SlotU[s];
    std::vector<uint> Order;
    for (int Step = 0; Step < O.MaxSteps; Step++)
    {
        if (Step > 0)
        {
            if (O.bReference) ReferenceFill(S, M, S.SolveEps);
            else
            {
                ParallelFor(M, [&](int s) { S.VoxelSolveTreeFillInitPass(s, M, S.SolveEps); });
                for (int r = 0; r < O.R; r++) { ParallelFor(M, [&](int s) { S.VoxelSolveTreeFillJumpPass(s, M, S.SolveEps); }); SwapJump(S); }
                ParallelFor(M, [&](int s) { S.VoxelSolveTreeFillFinishPass(s, M); });
                for (int k = 0; k < O.Kpd; k++)
                {
                    ParallelFor(M, [&](int s) { S.VoxelSolveFillJacobiPass(s, M, S.SolveEps); });
                    ParallelFor(M, [&](int s) { S.VoxelSolveFillCopyPass(s, M); });
                }
            }
            ParallelFor(M, [&](int s) { S.VoxelSolveReceiversPass(s, M, true); });
        }
        double SumDh = 0;
        if (O.bReference)
        {
            ReferenceAccumulate(S, M, Order);
            SumDh = ReferenceImplicit(S, M, Order);
        }
        else
        {
            for (int k = 0; k < O.Kacc; k++) { ParallelFor(M, [&](int s) { S.VoxelSolveAccumulateJacobiPass(s, M); }); std::swap(S.SlotA, S.SlotAOut); }
            ParallelFor(M, [&](int s) { S.VoxelSolveImplicitInitPass(s, M); });
            for (int r = 0; r < O.R; r++) { ParallelFor(M, [&](int s) { S.VoxelSolveImplicitJumpPass(s, M); }); SwapJump(S); }
            std::vector<float> Dh(M);
            ParallelFor(M, [&](int s) { Dh[s] = S.VoxelSolveImplicitFinishPass(s, M); });
            for (uint s = 0; s < M; s++) SumDh += Dh[s];
        }
        const float Residual = (float)(SumDh / std::max(SumU * S.SolveDt, 1e-30));
        Res.Residuals.push_back(Residual);
        if (bLog && (Step % 10 == 0)) printf("  step %3d residual %.4f\n", Step, Residual);
        if (Step + 1 >= O.MinSteps && Residual < O.Tol) break;
    }
    // Outlets, the normalisation.
    ParallelFor(M, [&](int s) { S.VoxelSolveOutletInitPass(s, M); });
    for (int r = 0; r < O.R; r++) { ParallelFor(M, [&](int s) { S.VoxelSolveOutletJumpPass(s, M); }); SwapJump(S); }
    std::vector<float> Solved, Target;
    Solved.clear(); Target.clear();
    for (uint s = 0; s < M; s++) { const auto R = S.VoxelSolveReliefOf(s); if (!R.bCounted) continue; Solved.push_back(R.Solved); Target.push_back(R.Target); }
    if (bLog)
    {
        for (double q : { 0.5, 0.75, 0.9, 0.95, 0.99, 0.999, 1.0 })
            printf("  quantile %.3f: solved %.0f m, target %.0f m\n", q, Percentile(Solved, std::min(q, 0.999999)), Percentile(Target, std::min(q, 0.999999)));
    }
    Res.P90Solved = Percentile(Solved, 0.9);
    Res.P90Target = Percentile(Target, 0.9);
    S.SolveFactor = std::clamp(Res.P90Target / std::max(Res.P90Solved, 1.0f), 0.01f, 100.0f);
    S.SolveReliefKnee = Res.P90Target;
    S.SolveReliefTop = std::max(ReliefTopScale * Percentile(Target, 0.999), 1.2f * Res.P90Target);
    if (bReliefPower)
    {
        // As VoxelLandSolve.cpp: the logarithm's scale that puts the solved 99.9th percentile on the top.
        const double D = S.SolveFactor * Percentile(Solved, 0.999) - S.SolveReliefKnee;
        const double Want = S.SolveReliefTop - S.SolveReliefKnee;
        double Sc = 1.0e9;
        if (D > Want && Want > 0.0)
        {
            double Lo = 1.0e-3 * Want, Hi = 1.0e3 * std::max(D, 1.0);
            for (int i = 0; i < 80; i++) { const double Mid = std::sqrt(Lo * Hi); (Mid * std::log(1.0 + D / Mid) > Want ? Hi : Lo) = Mid; }
            Sc = std::sqrt(Lo * Hi);
        }
        S.SolveReliefScale = (float)Sc;
        if (bLog) printf("  relief log scale %.0f m (solved p99.9 %.0f m above the knee, the top %.0f m above)\n", Sc, D, Want);
    }
    Res.Factor = S.SolveFactor;
    if (S.SolveModel == 1u)
        ParallelFor(M, [&](int s) { S.VoxelSolveSettleMarkPass(s, M); });
    ParallelFor(Cells, [&](int c) { S.VoxelSolveNormalisePass(c); });
    if (S.SolveModel == 1u)
    {
        // As the GPU job: the settle's tree fill along the last step's receivers, then the settle.
        ParallelFor(M, [&](int s) { S.VoxelSolveSettleInitPass(s, M, S.SolveEps); });
        for (int r = 0; r < O.R; r++) { ParallelFor(M, [&](int s) { S.VoxelSolveTreeFillJumpPass(s, M, S.SolveEps); }); SwapJump(S); }
        ParallelFor(M, [&](int s) { S.VoxelSolveTreeFillFinishPass(s, M); });
        ParallelFor(M, [&](int s) { S.VoxelSolveSettlePass(s, M); });
    }
    // Routing.
    const uint L = Compact(S, 1);
    Res.Land = L;
    for (uint c = 0; c < Cells; c++) S.VoxelSolveRouteInitPass(c, 0u);
    const int It0 = UntilConverged(S, [&]() { for (uint s = 0; s < L; s++) S.VoxelSolveFillInPlacePass(s, L, 0.0f); }, 100000);
    for (uint c = 0; c < Cells; c++) S.VoxelSolveRouteInitPass(c, 1u);
    const int It1 = UntilConverged(S, [&]() { for (uint s = 0; s < L; s++) S.VoxelSolveFillInPlacePass(s, L, S.SolveEps); }, 100000);
    ParallelFor(L, [&](int s) { S.VoxelSolveReceiversPass(s, L, false); });
    const int It2 = UntilConverged(S, [&]() { for (uint s = 0; s < L; s++) S.VoxelSolveAccumulateInPlacePass(s, L); }, 100000);
    int ItSpread = 0;
    if (S.SolveSpreadCells > 0.0f)
    {
        ParallelFor(L, [&](int s) { S.VoxelSolveSpreadInitPass(s, L); });
        ItSpread = UntilConverged(S, [&]() { for (uint s = 0; s < L; s++) S.VoxelSolveSpreadPass(s, L); }, 100000);
    }
    ParallelFor(L, [&](int s) { S.VoxelSolveLabelInitPass(s, L); });
    const int It3 = UntilConverged(S, [&]() { for (uint s = 0; s < L; s++) S.VoxelSolveLabelPass(s, L); }, 100000);
    for (uint c = 0; c < Cells; c++) { S.LakeArea[c] = 0; S.LakeDepth[c] = 0; S.LakeOutlet[c] = VOXEL_SOLVE_NONE; }
    S.LakeCount[0] = 0;
    for (uint s = 0; s < L; s++) S.VoxelSolveLakeStatsPass(s, L);
    for (uint s = 0; s < L; s++) S.VoxelSolveLakeFinishPass(s, L);
    ParallelFor(Cells, [&](int c) { S.VoxelSolveOutputPass(c); });
    Res.Lakes = (int)S.LakeCount[0];
    Res.Errors = S.SolveFlags[1];
    Res.Seconds = Now() - T0;
    if (bLog) printf("  routing sweeps: lakes fill %d, fill %d, accumulation %d, spread %d, labels %d\n", It0, It1, It2, ItSpread, It3);
    return Res;
}


/** The plates' warp (VoxelLandWarp) checked for folds: its Jacobian's determinant on the
 *  sphere's tangent plane at Count random directions (central differences), the share at or
 *  below 0 (a fold mirrors the boundary profiles' contours over a sliver) and the least. */
static void WarpFolds(float RadiusM, uint32_t Seed, int Count)
{
    FPlates P;
    P.LandRadiusM = RadiusM;
    P.LandSeed = Seed;
    std::mt19937 Random(3);
    std::normal_distribution<double> Gauss;
    int Folded = 0;
    double Least = 1.0e9;
    for (int k = 0; k < Count; k++)
    {
        double D[3] = { Gauss(Random), Gauss(Random), Gauss(Random) };
        const double L = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
        for (double& X : D) X /= L;
        // A tangent basis.
        double T1[3] = { -D[1], D[0], 0.0 };
        if (std::abs(D[2]) > 0.9) { T1[0] = 0.0; T1[1] = -D[2]; T1[2] = D[1]; }
        const double L1 = std::sqrt(T1[0] * T1[0] + T1[1] * T1[1] + T1[2] * T1[2]);
        for (double& X : T1) X /= L1;
        const double T2[3] = { D[1] * T1[2] - D[2] * T1[1], D[2] * T1[0] - D[0] * T1[2], D[0] * T1[1] - D[1] * T1[0] };
        const double H = 2.0e-4;   // radians
        auto Warp = [&](double A, double B) {
            return P.VoxelLandWarp(normalize(float3(float(D[0] + A * T1[0] + B * T2[0]), float(D[1] + A * T1[1] + B * T2[1]), float(D[2] + A * T1[2] + B * T2[2]))));
        };
        const float3 UA = Warp(H, 0.0), UB = Warp(-H, 0.0), VA = Warp(0.0, H), VB = Warp(0.0, -H);
        const double DU[3] = { (UA.x - UB.x) / (2 * H), (UA.y - UB.y) / (2 * H), (UA.z - UB.z) / (2 * H) };
        const double DV[3] = { (VA.x - VB.x) / (2 * H), (VA.y - VB.y) / (2 * H), (VA.z - VB.z) / (2 * H) };
        auto Dot = [](const double* X, const double* Y) { return X[0] * Y[0] + X[1] * Y[1] + X[2] * Y[2]; };
        const double Det = Dot(DU, T1) * Dot(DV, T2) - Dot(DU, T2) * Dot(DV, T1);
        Folded += Det <= 0.0 ? 1 : 0;
        Least = std::min(Least, Det);
    }
    printf("warp folds at radius %.0f m, seed %u: %.4f%% of %d directions, least determinant %.3f\n", RadiusM, Seed, 100.0 * Folded / Count, Count, Least);
}

int main(int argc, char** argv)
{
    const int N = argc > 1 ? atoi(argv[1]) : 256;
    const std::string Tag = argc > 2 ? argv[2] : "look";
    std::vector<std::pair<std::string, double>> Args;
    for (int a = 3; a < argc; a++)
    {
        const std::string S = argv[a];
        const size_t Eq = S.find('=');
        if (Eq == std::string::npos) continue;
        Args.push_back({ S.substr(0, Eq), atof(S.c_str() + Eq + 1) });
    }
    auto Get = [&](const char* Name, double Default) {
        for (const auto& A : Args) if (A.first == Name) return A.second;
        return Default;
    };
    const float RadiusM = (float)Get("RadiusM", 1.0e6);
    const uint32_t Seed = (uint32_t)Get("Seed", 7);
    if (Get("WarpFolds", 0.0) != 0.0)
    {
        WarpFolds(RadiusM, Seed, 400000);
        return 0;
    }
    FPlateBake B;
    B.N = N;
    double T = Now();
    BakePlates(B, Seed, 10, 0.30f, RadiusM);
    printf("plates 6x%d^2 in %.1f s\n", N, Now() - T);

    FSolve S;
    S.SolveN = N; S.SolvePad = B.Pad; S.SolveRadiusM = RadiusM; S.SolveSeed = Seed;
    S.SolveTan = B.Tan.data(); S.BakeHeight = B.Height.data(); S.BakeUp = B.Up.data(); S.BakeCrust = B.Crust.data(); S.BakeAttr = B.Attr.data();
    // The look test's settings (FSolve members), from the command line.
    S.SolveMinLakeDepth = (float)Get("MinLakeDepth", S.SolveMinLakeDepth);
    S.SolveMinLakeArea = (float)Get("MinLakeArea", S.SolveMinLakeArea);
    S.SolveJitter = (float)Get("Jitter", 0.0);
    S.SolveRouteNoise = (float)Get("RouteNoise", 0.0);
    S.SolveSpreadCells = (float)Get("SpreadCells", 0.0);
    S.SolveUndulation = (float)Get("Undulation", 0.0);
    S.SolveUpliftVariation = (float)Get("UpliftVariation", 0.0);
    S.SolveErodibilityVariation = (float)Get("ErodibilityVariation", 0.0);
    S.SolveVariationWavelength = (float)Get("VariationWavelength", 1.2e5);
    S.SolveRoughness = (float)Get("Roughness", 0.0);
    bReliefPower = Get("ReliefPower", 0.0) != 0.0;
    S.SolveFillDepressions = (uint)Get("FillDepressions", 1.0);
    S.SolveModel = (uint)Get("Model", 0.0);
    S.SolvePlainsRelief = (float)Get("PlainsRelief", 0.0);
    S.SolveIncision = (float)Get("Incision", 0.0);
    ReliefTopScale = (float)Get("ReliefTopScale", 1.3);
    FSolveBuffers Buf;
    const uint Cells = B.Cells();
    Buf.Bind(S, Cells);
    FRunOptions O;
    O.MaxSteps = (int)Get("MaxSteps", 150);
    const FRunResult R = Run(S, O, true);
    printf("  %zu steps, final residual %.4f, %.1f s; factor %.3f; %d lakes; errors 0x%x\n", R.Residuals.size(), R.Residuals.back(), R.Seconds, R.Factor, R.Lakes, R.Errors);
    auto Dump = [&](const std::string& Name, const void* Data, size_t Bytes) { FILE* F = fopen((Tag + "_" + Name).c_str(), "wb"); fwrite(Data, 1, Bytes, F); fclose(F); };
    Dump("height.f32", Buf.OHeight.data(), Cells * 4);
    Dump("flow.f32", Buf.OFlow.data(), Cells * 4);
    Dump("water.f32", Buf.OWater.data(), Cells * 4);
    Dump("rec.u32", Buf.Rec.data(), Cells * 4);
    Dump("plates.f32", B.Height.data(), Cells * 4);
    Dump("up.f32", B.Up.data(), Cells * 4);
    Dump("attr.u32", Buf.OAttr.data(), Cells * 4);
    FILE* J = fopen((Tag + ".json").c_str(), "w");
    fprintf(J, "{\"n\": %d, \"pad\": %d, \"radius_m\": %.1f, \"steps\": %zu, \"lakes\": %d, \"seconds\": %.1f}\n", N, B.Pad, RadiusM, R.Residuals.size(), R.Lakes, R.Seconds);
    fclose(J);
    return 0;
}
