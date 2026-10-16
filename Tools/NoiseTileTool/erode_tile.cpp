// erode_tile.cpp - Reference erosion for periodic tiles (Design/NoiseTiles2D_ErosionParadigm.md).
//
// The planet bake's erosion solve (Shaders/Private/VoxelLandSolve.ush) learned over stages T0 to
// T1b what makes simulated drainage look inorganic on a regular grid, and how to stop it. This
// file applies the same fixes to a wrapped N x N tile, on the CPU, in one readable place, so the
// noise generator project can port it (to its GPU compute or as is) instead of rediscovering them.
// Every fix has a switch, so each one's effect can be measured (the "legacy" preset turns them
// all off: an approximation of Tools/NoiseTileTool/eroded_peaks.cpp's method, which also fills
// no depressions and used K = 1, with its grid artifacts).
//
// Units: one texel horizontally, heights in the same unit (a gradient of 1 is 45 degrees).
// Scale: the parameters are tuned at a massif spacing S = N / Massifs of 64 texels. At another
// spacing the tool scales them as the continuous equations ask, so the landscape would be the
// same in units of S: uplift, starting height and roughness by s = S / 64, diffusion by s^2;
// the time step, the steps and the threshold slopes stay. K by sqrt(s), a correction for the
// grid (KScaleExponent): D8 drainage on unchanneled hillslopes grows with the path length, not
// the area. With it the relief scales by s; the slopes still come out a few degrees steeper at
// S = 128 than at 64 (the design doc, section 6). Preview at S = 64 for the network's look;
// judge slopes at the production size.
//
// The base landscape: a periodic Perlin fbm. Its lowest share is the base level (outlets held
// at 0: a periodic tile has no edge for water to leave by); above that, the field sets the
// starting surface and the uplift, so the relief follows the noise's massifs while every valley
// is the simulation's own. (Voronoi massif cells with their borders as outlets, the legacy
// mode, give straight polygonal trunk valleys: avoid.)
//
// Per step (section 4 of the design doc):
//   1. uplift:     H += U dt              U: the base field x a smooth variation field
//   2. fill:       W = priority flood     from the base level, each cell's descent Eps x a fixed
//                                         random factor x the step's length (random flat
//                                         routing: flats drain as wandering trees, not diagonals)
//                  H = W                  depressions fill with sediment
//   3. receivers:  steepest descent on W  between jittered node positions, each neighbour's
//                                         slope x a fixed random weight (Rho8-like)
//   4. area:       D8 accumulation        in reverse fill order (the fill order is topological)
//   5. incision:   implicit stream power  H_i = (H_i + F H_r) / (1 + F), F = K_i dt sqrt(A) / d
//                                         (Braun & Willett 2013, m = 0.5, n = 1), K_i a rock field
//   6. diffusion:  9-point isotropic      both rocks: sets the finest valley spacing above the
//                                         grid (hard a little, soft more: rounded crests);
//                                         substepped to stay stable
//   7. threshold:  eikonal limiter        |grad H| <= Sc_i, Sc_i a field: isotropic cones and
//                                         bowls, not the chamfer's octagonal facets
// After the steps: the output channels (Addendum 2): R hard rock, G soft rock derived from R
// (continues from it, so it shares R's drainage), B = R blurred, A = log2 of the
// multiple-flow-direction area.
//
// Build:  g++ -O2 -std=c++20 -I<dir with VoxelNoiseTiles.h> erode_tile.cpp -o erode_tile -pthread
// Run:    erode_tile out.vxnt [key=value ...]     (keys: FParams below; preset=legacy applies
//         first wherever it is given, the other keys then override it)
//         also writes out_R.pgm, out_G.pgm, out_A.pgm, out_R2x2.pgm (hillshades, binary PGM) and
//         out.json (parameters, the scaled values used, and the verification measures)

#include "VoxelNoiseTiles.h"
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include <map>
#include <algorithm>

using namespace VoxelNoiseTiles;

// --- Parameters --------------------------------------------------------------------------------

struct FParams
{
    int N = 1024;                 // tile side (texels, a power of two)
    uint32_t Seed = 1;
    int Massifs = 16;             // massif cells per side: spacing S = N / Massifs

    // The base landscape. 1 (default): a periodic Perlin fbm, the field p (0 at the base level,
    // 1 at its 99.5th percentile) sets the initial surface and the uplift; its lowest BaseShare
    // is the base level (held at 0): irregular lowland basins the drainage ends in. 0 (legacy):
    // warped Voronoi massif cells, their borders held at 0 as the trunk valleys (straight,
    // polygonal: inorganic), each cell's value its uplift.
    // 2 (hills, Design/NoiseTiles2D_Addendum3.md): a connected base network, the zero set of a
    // periodic Perlin |n| at BaseNetMassifs per side, warped, its lowest BaseShare of |n| held at
    // 0; p rises with |n| away from it (the trunk valleys are one network through the wrap, so
    // the tile has no closed basins: on a planet every valley drains into it).
    int BaseMode = 1;
    double BaseNetMassifs = 1.0;  // mode 2: the network's wavelength in massifs (1: about a massif apart)
    double BaseNetWarp = 0.35;    // mode 2: warp, in the network's wavelengths
    double BaseNetRamp = 0.3;     // mode 2: the uplift's rise from the network (a smoothstep over this share of
                                  // |n|'s range): small, an even upland the erosion dissects (hill country);
                                  // 1, a dome between network lines (radial, star-shaped hills)
    int BaseOctaves = 5;          // Perlin: octaves from a coarsest wavelength of 2 massifs
    double BaseShare = 0.08;      // Perlin: the share of the tile that is base level
    double UpliftScale = 2.0;     // Perlin: U = this x p (texels per time at S = 64)
    double InitialHeight = 4.0;   // Perlin: the starting surface, this x p (texels at S = 64)
    double BaseWidth = 1.5;       // Voronoi: half-width of the network (texels)
    double BaseWarp = 0.35;       // Voronoi: warp of the cells (massif cells)

    // Time.
    int Steps = 260;
    double Dt = 0.5;
    double K = 0.5;               // erodibility (texel units: F = K dt sqrt(A texels) / d)
    double KScaleExponent = 0.5;  // K x s^this (s = S / 64): D8 area on unchanneled hillslopes grows as
                                  // the path length (s), not the area (s^2), so incision there needs
                                  // sqrt(s) more to keep the landscape the same in units of S
    double ChannelArea = 0.0;     // channel heads: incision ramps in from half this drainage area to this (texels at S = 64, x s^2;
                                  // tried: faces turn planar, so 0); smaller areas are hillslopes
    double KappaHard = 0.25;      // hard-rock diffusion (texels^2 per time at S = 64): sets the finest valley spacing

    // The fixes (Design/SurfaceLook.md T1, T1b; the plugin's settings in brackets).
    double Jitter = 0.4;          // node offsets, cells [RoutingJitter]
    double RouteNoise = 1.0;      // receivers' slope weights 2^(+-this) [RoutingRandomness]
    double FlowBias = 0.0;        // directional ("flank") sets: receivers' slope weights x exp(this x c), c the
                                  // step's alignment with the tile's V axis: |cos| (either way along it), or
    int FlowBiasSigned = 0;       // with 1, cos toward -V (one way): valleys elongate along V
    int RandomFill = 1;           // random flat routing (the main T1b fix)
    double Eps = 1e-4;            // the fill's descent per cell (texels)
    double UpliftVariation = 0.5; // U x max(0, 1 + this x n1), n1 at 2 massifs [UpliftVariation]
    double ErodibilityVariation = 0.5; // K x exp(this x n2), n2 at 1 massif [ErodibilityVariation]
    double ThresholdVariation = 0.12;  // Sc x (1 + this x n3), n3 at a quarter massif
    double Roughness = 1.0;       // initial surface: fbm from half a massif, 4 octaves (texels at S = 64) [InitialRoughness]
    int Hillslope = 2;            // 2 eikonal limiter on both stencils, 1 axial stencil only, 0 chamfer (legacy)
    double SpreadCells = 16.0;    // A: MFD below this D8 area, D8 above [DrainageSpreadCells]

    // Rock.
    double ScHard = 0.84;         // 40 deg
    double ScSoft = 0.53;         // 28 deg
    double KappaSoft = 0.4;       // soft-rock diffusion (texels^2 per time at S = 64)
    double KSoft = 1.5;           // soft rock's erodibility over hard
    int SoftMode = 1;             // 1 derived from R (shared drainage), 0 a separate run (legacy)
    int SoftSteps = 100;          // steps of the derivation

    bool Set(const char* Key, const char* Value)
    {
        const double V = atof(Value);
        struct { const char* Name; double* D; int* I; } Table[] = {
            { "BaseWidth", &BaseWidth, nullptr }, { "BaseWarp", &BaseWarp, nullptr }, { "BaseShare", &BaseShare, nullptr },
            { "UpliftScale", &UpliftScale, nullptr }, { "InitialHeight", &InitialHeight, nullptr },
            { "BaseMode", nullptr, &BaseMode }, { "BaseNetMassifs", &BaseNetMassifs, nullptr }, { "BaseNetWarp", &BaseNetWarp, nullptr }, { "BaseNetRamp", &BaseNetRamp, nullptr }, { "BaseOctaves", nullptr, &BaseOctaves }, { "Dt", &Dt, nullptr }, { "K", &K, nullptr },
            { "Jitter", &Jitter, nullptr }, { "RouteNoise", &RouteNoise, nullptr }, { "FlowBias", &FlowBias, nullptr }, { "Eps", &Eps, nullptr },
            { "UpliftVariation", &UpliftVariation, nullptr }, { "ErodibilityVariation", &ErodibilityVariation, nullptr },
            { "ThresholdVariation", &ThresholdVariation, nullptr }, { "Roughness", &Roughness, nullptr },
            { "SpreadCells", &SpreadCells, nullptr }, { "ScHard", &ScHard, nullptr }, { "ScSoft", &ScSoft, nullptr },
            { "KappaSoft", &KappaSoft, nullptr }, { "ChannelArea", &ChannelArea, nullptr }, { "KScaleExponent", &KScaleExponent, nullptr }, { "KappaHard", &KappaHard, nullptr }, { "KSoft", &KSoft, nullptr },
            { "N", nullptr, &N }, { "Massifs", nullptr, &Massifs }, { "Steps", nullptr, &Steps }, { "RandomFill", nullptr, &RandomFill },
            { "Hillslope", nullptr, &Hillslope }, { "FlowBiasSigned", nullptr, &FlowBiasSigned }, { "SoftMode", nullptr, &SoftMode }, { "SoftSteps", nullptr, &SoftSteps },
        };
        if (!strcmp(Key, "Seed")) { Seed = (uint32_t)strtoul(Value, nullptr, 10); return true; }
        if (!strcmp(Key, "preset"))
        {
            if (strcmp(Value, "legacy")) return false;
            // Everything eroded_peaks.cpp lacked: the grid artifacts' baseline.
            BaseMode = 0; KappaHard = 0; Jitter = 0; RouteNoise = 0; RandomFill = 0; UpliftVariation = 0; ErodibilityVariation = 0;
            ThresholdVariation = 0; Roughness = 0; Hillslope = 0; SoftMode = 0; SpreadCells = 0;
            return true;
        }
        for (auto& E : Table)
            if (!strcmp(Key, E.Name)) { if (E.D) *E.D = V; else *E.I = (int)V; return true; }
        return false;
    }

    std::string Json() const
    {
        char B[2048];
        snprintf(B, sizeof(B),
            "{\"N\": %d, \"Seed\": %u, \"Massifs\": %d, \"Base\": \"%s\", \"BaseOctaves\": %d, \"BaseShare\": %g, \"UpliftScale\": %g, "
            "\"InitialHeight\": %g, \"BaseWidth\": %g, \"BaseWarp\": %g, \"Steps\": %d, \"Dt\": %g, \"K\": %g, \"ChannelArea\": %g, \"KappaHard\": %g, "
            "\"m\": 0.5, \"n\": 1, \"Jitter\": %g, \"RouteNoise\": %g, \"RandomFill\": %d, \"Eps\": %g, \"UpliftVariation\": %g, "
            "\"ErodibilityVariation\": %g, \"ThresholdVariation\": %g, \"Roughness\": %g, \"Hillslope\": \"%s\", \"SpreadCells\": %g, "
            "\"ScHard\": %g, \"ScSoft\": %g, \"KappaSoft\": %g, \"KSoft\": %g, \"SoftMode\": \"%s\", \"SoftSteps\": %d, "
            "\"BlurSigma\": %g, \"Routing\": \"%s\"}",
            N, Seed, Massifs, BaseMode == 2 ? "network" : (BaseMode ? "perlin" : "voronoi"), BaseOctaves, BaseShare, UpliftScale, InitialHeight, BaseWidth, BaseWarp, Steps, Dt, K, ChannelArea, KappaHard, Jitter, RouteNoise, RandomFill, Eps, UpliftVariation,
            ErodibilityVariation, ThresholdVariation, Roughness, Hillslope == 2 ? "eikonal (axial + diagonal)" : Hillslope == 1 ? "eikonal (axial)" : "chamfer", SpreadCells, ScHard, ScSoft,
            KappaSoft, KSoft, SoftMode ? "derived" : "separate", SoftSteps, (double)N / Massifs / 4.0,
            Jitter > 0 || RouteNoise > 0 ? "D8 on a jittered graph, randomised receivers" : "D8 steepest descent");
        return B;
    }
};

// --- Helpers -----------------------------------------------------------------------------------

static void Par(int32_t Count, const std::function<void(int32_t)>& F)
{
    const int T = std::max(1u, std::thread::hardware_concurrency());
    std::atomic<int32_t> Next{ 0 };
    std::vector<std::thread> Th;
    for (int t = 0; t < T; t++) Th.emplace_back([&]() { for (int32_t i; (i = Next.fetch_add(1)) < Count;) F(i); });
    for (auto& t : Th) t.join();
}

// Neighbour k of 8: axial first, then diagonal.
static const int DX[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
static const int DY[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };

/** -1 .. 1, a fixed hash of (A, B) and the seed. */
static double Uniform(uint32_t A, uint32_t B, uint32_t Seed)
{
    return (double)(Hash3(A, B, Seed) >> 8) * (2.0 / 16777216.0) - 1.0;
}

/** A periodic fbm over the tile, about -1 .. 1: CellsPerTile coarsest cells, Octaves halving. */
static std::vector<double> FbmField(int N, uint32_t Seed, uint32_t CellsPerTile, uint32_t Octaves)
{
    FChannel C;
    C.Kind = KindFbm; C.Seed = Mix(Seed); C.CellsPerTile = CellsPerTile; C.Octaves = Octaves; C.Gain = 0.5f;
    std::vector<double> F((size_t)N * N);
    Par(N, [&](int32_t y) {
        const double K = (double)CellsPerTile / N;
        for (int x = 0; x < N; x++) F[(size_t)y * N + x] = 1.4 * Evaluate(C, x * K, y * K);   // gradient noise spans about +-0.7
    });
    return F;
}

// --- The simulation ------------------------------------------------------------------------------

struct FSim
{
    const FParams& P;
    int N;
    std::vector<double> H, W, U, KField;
    std::vector<double> OX, OY;          // node offsets (the jittered graph)
    std::vector<double> FillFactor;      // the random fill's per-cell factor
    std::vector<uint8_t> Base;
    std::vector<int32_t> Recv, Order;    // Order: the fill's pop order (increasing W; topological)
    std::vector<double> Area, Dist;

    double ChannelArea = 0;              // P.ChannelArea scaled to the tile (texels, x s^2)
    double KTile = 0;                    // P.K scaled to the tile (x s^KScaleExponent)

    FSim(const FParams& InP) : P(InP), N(InP.N)
    {
        const double S = (double)InP.N / InP.Massifs / 64.0;
        ChannelArea = InP.ChannelArea * S * S;
        KTile = InP.K * std::pow(S, InP.KScaleExponent);
    }

    int Id(int X, int Y) const { return Wrap(Y, N) * N + Wrap(X, N); }
    int Nb(int i, int k) const { return Id(i % N + DX[k], i / N + DY[k]); }

    /** The distance between i's node and its neighbour k's (cells). */
    double NodeDist(int i, int k) const
    {
        const int j = Nb(i, k);
        return std::hypot(DX[k] + OX[j] - OX[i], DY[k] + OY[j] - OY[i]);
    }

    /** i's fixed weight for neighbour k in the receivers' choice. */
    double RouteWeight(int i, int k) const
    {
        double W = 1.0;
        if (P.RouteNoise > 0) W = std::exp2(P.RouteNoise * Uniform((uint32_t)i * 8u + (uint32_t)k, 0x6a09e667u, P.Seed));
        if (P.FlowBias != 0)
        {
            const double C = -DY[k] / std::hypot((double)DX[k], (double)DY[k]);   // toward -V
            W *= std::exp(P.FlowBias * (P.FlowBiasSigned ? C : std::fabs(C)));
        }
        return W;
    }

    void Init(const std::vector<uint8_t>& InBase, const std::vector<double>& InU, const std::vector<double>& InK,
        const std::vector<double>& InH)
    {
        const size_t NN = (size_t)N * N;
        Base = InBase; U = InU; KField = InK; H = InH;
        OX.assign(NN, 0.0); OY.assign(NN, 0.0); FillFactor.assign(NN, 1.0);
        for (size_t i = 0; i < NN; i++)
        {
            if (P.Jitter > 0)
            {
                const uint32_t Hs = Hash3((uint32_t)i, 0x2c1b3c6du, P.Seed);
                OX[i] = ((double)(Hs & 0xffffu) / 65535.0 - 0.5) * 2.0 * P.Jitter;
                OY[i] = ((double)(Hs >> 16) / 65535.0 - 0.5) * 2.0 * P.Jitter;
            }
            if (P.RandomFill)
                FillFactor[i] = std::exp2(2.0 * Uniform((uint32_t)i, 0x3c6ef372u, P.Seed));   // 1/4 .. 4
        }
    }

    /** Priority flood from the base-level network: W = the least surface that drains every cell
     *  by at least the descent per step; Order = the pop order. With RandomFill the descent into
     *  a cell is Eps x its factor (1/4 .. 4) x the step's length, so the least paths across a
     *  flat are a wandering, branching tree; without, Eps per step, and every flat drains along
     *  the chessboard-shortest paths: dead straight diagonals (Design/SurfaceLook.md, T1b). */
    void Fill()
    {
        const size_t NN = (size_t)N * N;
        W.assign(NN, 1e300);
        std::vector<uint8_t> Done(NN, 0);
        using E = std::pair<double, int32_t>;
        std::priority_queue<E, std::vector<E>, std::greater<E>> Q;
        for (size_t i = 0; i < NN; i++)
            if (Base[i]) { W[i] = H[i]; Q.push({ W[i], (int32_t)i }); }
        Order.clear();
        Order.reserve(NN);
        while (!Q.empty())
        {
            const auto [w, c] = Q.top();
            Q.pop();
            if (Done[c] || w > W[c]) continue;
            Done[c] = 1;
            Order.push_back(c);
            for (int k = 0; k < 8; k++)
            {
                const int n = Nb(c, k);
                if (Done[n] || Base[n]) continue;
                const double Step = P.RandomFill ? P.Eps * FillFactor[n] * NodeDist(c, k) : P.Eps;
                const double Cand = std::max(H[n], w + Step);
                if (Cand < W[n]) { W[n] = Cand; Q.push({ Cand, n }); }
            }
        }
    }

    /** Receivers on W: the steepest weighted descent between the jittered nodes. */
    void Receivers()
    {
        const size_t NN = (size_t)N * N;
        Recv.resize(NN);
        Dist.assign(NN, 1.0);
        Par(N, [&](int32_t y) {
            for (int x = 0; x < N; x++)
            {
                const int i = y * N + x;
                int Best = i;
                double BestSlope = 0.0, BestD = 1.0;
                if (!Base[i])
                    for (int k = 0; k < 8; k++)
                    {
                        const int j = Nb(i, k);
                        const double D = NodeDist(i, k);
                        const double S = (W[i] - W[j]) / D * RouteWeight(i, k);
                        if (S > BestSlope) { BestSlope = S; Best = j; BestD = D; }
                    }
                Recv[i] = Best;
                Dist[i] = BestD;
            }
        });
    }

    void Accumulate()
    {
        Area.assign((size_t)N * N, 1.0);
        for (size_t s = Order.size(); s-- > 0;)
        {
            const int i = Order[s];
            if (Recv[i] != i) Area[Recv[i]] += Area[i];
        }
    }

    /** Implicit stream power, receivers first (the fill order). */
    void Incise(double KScale)
    {
        for (int i : Order)
        {
            const int r = Recv[i];
            if (r == i || Base[i]) continue;
            double F = KTile * KScale * KField[i] * P.Dt * std::sqrt(Area[i]) / Dist[i];
            if (ChannelArea > 0)
            {
                const double T = std::clamp((Area[i] - 0.5 * ChannelArea) / (0.5 * ChannelArea), 0.0, 1.0);
                F *= T * T * (3.0 - 2.0 * T);
            }
            H[i] = (H[i] + F * H[r]) / (1.0 + F);
        }
    }

    /** Linear diffusion, the isotropic 9-point Laplacian, substepped within its stable range. */
    void Diffuse(double Kappa)
    {
        if (!(Kappa > 0)) return;
        // The 9-point stencil is stable for kappa dt <= 3/8 and monotone (no new extrema) for
        // kappa dt <= 0.3: the substeps keep within the latter.
        const int Sub = std::max(1, (int)std::ceil(Kappa * P.Dt / 0.3));
        const double KDt = Kappa * P.Dt / Sub;
        std::vector<double> Next(H.size());
        for (int s = 0; s < Sub; s++)
        {
            Par(N, [&](int32_t y) {
                for (int x = 0; x < N; x++)
                {
                    const int i = y * N + x;
                    if (Base[i]) { Next[i] = H[i]; continue; }
                    const double Ax = H[Id(x + 1, y)] + H[Id(x - 1, y)] + H[Id(x, y + 1)] + H[Id(x, y - 1)];
                    const double Di = H[Id(x + 1, y + 1)] + H[Id(x - 1, y + 1)] + H[Id(x + 1, y - 1)] + H[Id(x - 1, y - 1)];
                    Next[i] = H[i] + KDt * (4.0 * Ax + Di - 20.0 * H[i]) / 6.0;
                }
            });
            H.swap(Next);
        }
    }

    /** Threshold hillslopes: lower every cell until |grad H| <= Sc. The eikonal form (Godunov
     *  upwind, fast sweeping, iterated for the wrap) caps the slope in every direction alike, so a
     *  face below a summit is a cone; the chamfer form (legacy) caps it along 8 directions only,
     *  so faces come out as an octagon's planar facets aligned to the grid. */
    void Threshold(const std::vector<double>& Sc)
    {
        if (P.Hillslope == 0)
        {
            for (int Round = 0; Round < 2; Round++)
            {
                for (int y = 0; y < N; y++)
                    for (int x = 0; x < N; x++)
                    {
                        const int i = y * N + x;
                        const double D1 = Sc[i], D2 = Sc[i] * 1.41421356;
                        H[i] = std::min({ H[i], H[Id(x - 1, y)] + D1, H[Id(x, y - 1)] + D1, H[Id(x - 1, y - 1)] + D2, H[Id(x + 1, y - 1)] + D2 });
                    }
                for (int y = N - 1; y >= 0; y--)
                    for (int x = N - 1; x >= 0; x--)
                    {
                        const int i = y * N + x;
                        const double D1 = Sc[i], D2 = Sc[i] * 1.41421356;
                        H[i] = std::min({ H[i], H[Id(x + 1, y)] + D1, H[Id(x, y + 1)] + D1, H[Id(x + 1, y + 1)] + D2, H[Id(x - 1, y + 1)] + D2 });
                    }
            }
            return;
        }
        for (int Iter = 0;; Iter++)
        {
            if (Iter == 64)
            {
                // Never seen (5 to 7 iterations a step are usual): the slopes may exceed Sc a little.
                printf("  warning: the threshold limiter didn't converge in 64 iterations\n");
                break;
            }
            bool bChanged = false;
            for (int Sweep = 0; Sweep < 4; Sweep++)
            {
                const bool bXUp = (Sweep & 1) == 0, bYUp = (Sweep & 2) == 0;
                for (int yy = 0; yy < N; yy++)
                {
                    const int y = bYUp ? yy : N - 1 - yy;
                    for (int xx = 0; xx < N; xx++)
                    {
                        const int x = bXUp ? xx : N - 1 - xx;
                        const int i = y * N + x;
                        if (Base[i]) continue;
                        // The Godunov update on the axial stencil (spacing 1); with Hillslope 2
                        // also on the diagonal one (spacing sqrt 2), keeping the lower: each
                        // alone pulls cone faces toward its own axes (a 4-fold bias in the
                        // slopes' directions), their minimum nearly cancels it.
                        auto Godunov = [](double A, double B, double F) {
                            return std::fabs(A - B) >= F ? std::min(A, B) + F : 0.5 * (A + B + std::sqrt(2.0 * F * F - (A - B) * (A - B)));
                        };
                        const double F = Sc[i];
                        double T = Godunov(std::min(H[Id(x - 1, y)], H[Id(x + 1, y)]), std::min(H[Id(x, y - 1)], H[Id(x, y + 1)]), F);
                        if (P.Hillslope == 2)
                            T = std::min(T, Godunov(std::min(H[Id(x - 1, y - 1)], H[Id(x + 1, y + 1)]), std::min(H[Id(x - 1, y + 1)], H[Id(x + 1, y - 1)]), F * 1.41421356237));
                        if (T < H[i] - 1e-12) { H[i] = T; bChanged = true; }
                    }
                }
            }
            if (!bChanged) break;
        }
    }

    /** One step; returns the largest height change. */
    double Step(double KScale, double Kappa, const std::vector<double>& Sc)
    {
        const std::vector<double> Before = H;
        for (size_t i = 0; i < H.size(); i++) if (!Base[i]) H[i] += U[i] * P.Dt;
        Fill();
        H = W;
        Receivers();
        Accumulate();
        Incise(KScale);
        Diffuse(Kappa);
        Threshold(Sc);
        double Change = 0;
        for (size_t i = 0; i < H.size(); i++) Change = std::max(Change, std::fabs(H[i] - Before[i]));
        return Change;
    }

    /** The routing of the final surface (no erosion). */
    void Route()
    {
        Fill();
        Receivers();
        Accumulate();
    }

    /** The multiple-flow-direction area (Quinn et al. 1991, Freeman 1991): a cell draining
     *  fewer than SpreadCells D8 cells spreads its water over every lower neighbour by
     *  slope^1.1; a channel cell sends it all to its receiver. Hillslopes carry an even sheet
     *  and lines start where flow converges, instead of a comb of one-cell lines. */
    std::vector<double> SpreadArea() const
    {
        std::vector<double> A((size_t)N * N, 1.0);
        for (size_t s = Order.size(); s-- > 0;)
        {
            const int i = Order[s];
            if (Base[i]) continue;
            if (Area[i] >= P.SpreadCells)
            {
                A[Recv[i]] += A[i];
                continue;
            }
            double Wt[8], Sum = 0;
            for (int k = 0; k < 8; k++)
            {
                const double Drop = W[i] - W[Nb(i, k)];
                Wt[k] = Drop > 0 ? std::pow(Drop / NodeDist(i, k), 1.1) : 0.0;
                Sum += Wt[k];
            }
            if (Sum > 0)
                for (int k = 0; k < 8; k++) A[Nb(i, k)] += A[i] * Wt[k] / Sum;
        }
        return A;
    }
};

// --- Measures ----------------------------------------------------------------------------------

struct FMeasures
{
    double Continue = 0, Straight6 = 0;   // channel cells (D8 area > 64)
    int Longest = 0;
    double C4 = 0, C8 = 0;                // gradient orientation's 4- and 8-fold harmonics (0 isotropic)
    double SlopeP50 = 0, SlopeP90 = 0, SlopeP99 = 0;   // degrees
    double SeamRatio = 0;                 // the wrap edge's mean step over the whole tile's
    double SeamLo = 0, SeamHi = 0;        // the same ratio's 1st and 99th percentiles over the interior's rows and columns
};

/** The measures, their thresholds in units of the massif spacing (Scale = S / 64): channels
 *  drain more than 64 Scale^2 texels, straight runs are at least 6 Scale cells. */
static FMeasures Measure(const FSim& S, const std::vector<double>& H, double Scale)
{
    const int N = S.N;
    FMeasures M;
    const double ChannelMin = 64.0 * Scale * Scale;
    const int RunMin = std::max(2, (int)std::lround(6.0 * Scale)), RunCap = 64 * std::max(1, (int)std::lround(Scale));
    // Channels: does a cell's step continue its donor's direction; runs of the same step.
    std::vector<int8_t> StepK((size_t)N * N, -1);
    for (int i = 0; i < N * N; i++)
        if (S.Recv[i] != i)
            for (int k = 0; k < 8; k++)
                if (S.Nb(i, k) == S.Recv[i]) { StepK[i] = (int8_t)k; break; }
    size_t Pairs = 0, Same = 0, Chan = 0, Runs6 = 0;
    for (int i = 0; i < N * N; i++)
    {
        if (S.Area[i] <= ChannelMin || StepK[i] < 0 || S.Base[i]) continue;
        const int r = S.Recv[i];
        if (StepK[r] >= 0 && !S.Base[r]) { Pairs++; Same += StepK[r] == StepK[i]; }
        int Run = 1, c = i;
        while (Run < RunCap)
        {
            const int n = S.Recv[c];
            if (n == c || S.Base[n] || StepK[n] != StepK[i]) break;
            Run++;
            c = n;
        }
        Chan++;
        Runs6 += Run >= RunMin;
        M.Longest = std::max(M.Longest, Run);
    }
    M.Continue = Pairs ? (double)Same / Pairs : 0;
    M.Straight6 = Chan ? (double)Runs6 / Chan : 0;
    // Slopes and their orientation.
    std::vector<double> Sl;
    double C4r = 0, C4i = 0, C8r = 0, C8i = 0;
    size_t Steep = 0;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++)
        {
            const double GX = 0.5 * (H[S.Id(x + 1, y)] - H[S.Id(x - 1, y)]);
            const double GY = 0.5 * (H[S.Id(x, y + 1)] - H[S.Id(x, y - 1)]);
            const double G = std::hypot(GX, GY);
            Sl.push_back(G);
            if (G > 0.2)
            {
                const double T = std::atan2(GY, GX);
                C4r += std::cos(4 * T); C4i += std::sin(4 * T);
                C8r += std::cos(8 * T); C8i += std::sin(8 * T);
                Steep++;
            }
        }
    if (Steep) { M.C4 = std::hypot(C4r, C4i) / Steep; M.C8 = std::hypot(C8r, C8i) / Steep; }
    std::sort(Sl.begin(), Sl.end());
    auto Q = [&](double q) { return std::atan(Sl[(size_t)(q * (Sl.size() - 1))]) * 57.29578; };
    M.SlopeP50 = Q(0.5); M.SlopeP90 = Q(0.9); M.SlopeP99 = Q(0.99);
    // Seams: the mean step across each column boundary (x | x + 1) and row boundary; the wrap
    // edge's (x = N-1 | 0, y = N-1 | 0) against the whole tile's, and where the interior's
    // boundaries fall (a seamless tile's edge is inside that spread).
    std::vector<double> Col(N, 0.0), Row(N, 0.0);
    for (int t = 0; t < N; t++)
        for (int u = 0; u < N; u++)
        {
            Col[u] += std::fabs(H[S.Id(u + 1, t)] - H[S.Id(u, t)]) / N;
            Row[u] += std::fabs(H[S.Id(t, u + 1)] - H[S.Id(t, u)]) / N;
        }
    double All = 0;
    for (int u = 0; u < N; u++) All += (Col[u] + Row[u]) / (2.0 * N);
    M.SeamRatio = 0.5 * (Col[N - 1] + Row[N - 1]) / All;
    std::vector<double> Inner;
    for (int u = 0; u < N - 1; u++) { Inner.push_back(Col[u] / All); Inner.push_back(Row[u] / All); }
    std::sort(Inner.begin(), Inner.end());
    M.SeamLo = Inner[(size_t)(0.01 * (Inner.size() - 1))];
    M.SeamHi = Inner[(size_t)(0.99 * (Inner.size() - 1))];
    return M;
}

static double Correlation(const std::vector<double>& A, const std::vector<double>& B)
{
    double MA = 0, MB = 0;
    for (size_t i = 0; i < A.size(); i++) { MA += A[i]; MB += B[i]; }
    MA /= A.size(); MB /= B.size();
    double SAB = 0, SAA = 0, SBB = 0;
    for (size_t i = 0; i < A.size(); i++) { SAB += (A[i] - MA) * (B[i] - MB); SAA += (A[i] - MA) * (A[i] - MA); SBB += (B[i] - MB) * (B[i] - MB); }
    return SAB / std::sqrt(SAA * SBB);
}

/** Correlation of the two heightfields' slopes' directions where both are steep (shared
 *  valleys and ridges: near 1; unrelated networks: near 0). */
static double AspectAgreement(const FSim& S, const std::vector<double>& A, const std::vector<double>& B)
{
    const int N = S.N;
    double Sum = 0;
    size_t Count = 0;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++)
        {
            const double AX = A[S.Id(x + 1, y)] - A[S.Id(x - 1, y)], AY = A[S.Id(x, y + 1)] - A[S.Id(x, y - 1)];
            const double BX = B[S.Id(x + 1, y)] - B[S.Id(x - 1, y)], BY = B[S.Id(x, y + 1)] - B[S.Id(x, y - 1)];
            const double LA = std::hypot(AX, AY), LB = std::hypot(BX, BY);
            if (LA < 0.4 || LB < 0.4) continue;
            Sum += (AX * BX + AY * BY) / (LA * LB);
            Count++;
        }
    return Count ? Sum / Count : 0;
}

// --- Output ------------------------------------------------------------------------------------

static void WritePgm(const std::string& Path, int W, int Hgt, const std::vector<uint8_t>& Px)
{
    FILE* F = fopen(Path.c_str(), "wb");
    if (!F) return;
    fprintf(F, "P5\n%d %d\n255\n", W, Hgt);
    fwrite(Px.data(), 1, Px.size(), F);
    fclose(F);
}

/** A hillshade (light from the north-west at 40 degrees, texel units), Tiles x Tiles copies. */
static void WriteShade(const std::string& Path, const FSim& S, const std::vector<double>& H, int Tiles = 1)
{
    const int N = S.N, W = N * Tiles;
    std::vector<uint8_t> Px((size_t)W * W);
    const double El = 40.0 * 3.14159265 / 180.0;
    for (int y = 0; y < W; y++)
        for (int x = 0; x < W; x++)
        {
            const double GX = 0.5 * (H[S.Id(x + 1, y)] - H[S.Id(x - 1, y)]);
            const double GY = 0.5 * (H[S.Id(x, y + 1)] - H[S.Id(x, y - 1)]);
            const double Slope = std::atan(std::hypot(GX, GY));
            const double Aspect = std::atan2(-GX, GY);
            double L = 0;
            const double Az[3] = { 315, 225, 45 }, Wt[3] = { 0.6, 0.2, 0.2 };
            for (int a = 0; a < 3; a++)
                L += Wt[a] * std::max(0.0, std::sin(El) * std::cos(Slope) + std::cos(El) * std::sin(Slope) * std::cos(Az[a] * 3.14159265 / 180.0 - Aspect));
            Px[(size_t)y * W + x] = (uint8_t)std::clamp(L * 255.0, 0.0, 255.0);
        }
    WritePgm(Path, W, W, Px);
}

static void PutU32(std::vector<uint8_t>& B, size_t At, uint32_t V) { for (int k = 0; k < 4; k++) B[At + k] = (uint8_t)(V >> (8 * k)); }
static void PutF32(std::vector<uint8_t>& B, size_t At, float F) { uint32_t U; std::memcpy(&U, &F, 4); PutU32(B, At, U); }

/** Writes Set as .vxnt v3 (the layout VoxelNoiseTiles::ReadVxntHeader documents). */
static bool WriteVxnt(const FSet& Set, const char* Path)
{
    const size_t Header = 128 + 4 * 64 + 16 * (size_t)Set.Mips;
    std::vector<uint8_t> B(Header + Set.Texels.size() * 2, 0);
    std::memcpy(B.data(), "VXNT", 4);
    PutU32(B, 4, 3); PutU32(B, 8, Set.N); PutU32(B, 12, Set.Mips); PutU32(B, 16, 4); PutU32(B, 20, 0);
    PutU32(B, 24, Set.Seed); PutU32(B, 28, (uint32_t)Header);
    std::memcpy(B.data() + 32, Set.Name.c_str(), std::min<size_t>(Set.Name.size(), 31));
    std::memcpy(B.data() + 64, Set.LookTag.c_str(), std::min<size_t>(Set.LookTag.size(), 63));
    for (int c = 0; c < 4; c++)
    {
        const FChannel& C = Set.Channel[c];
        const size_t P = 128 + 64 * (size_t)c;
        PutU32(B, P, C.Kind); PutU32(B, P + 4, C.Seed); PutU32(B, P + 8, C.CellsPerTile); PutU32(B, P + 12, C.Octaves);
        PutF32(B, P + 16, 0.0f); PutF32(B, P + 20, C.Gain); PutF32(B, P + 24, C.Jitter); PutF32(B, P + 28, C.ErosionK);
        PutU32(B, P + 32, (uint32_t)C.Source); PutF32(B, P + 36, C.Min); PutF32(B, P + 40, C.Max); PutF32(B, P + 44, C.Mean); PutF32(B, P + 48, C.Std);
    }
    for (uint32_t m = 0; m < Set.Mips; m++)
    {
        const size_t P = 128 + 256 + 16 * (size_t)m;
        PutU32(B, P, Set.N >> m);
        const uint64_t Off = Header + MipOffset(Set.N, m) * 8;
        PutU32(B, P + 8, (uint32_t)Off); PutU32(B, P + 12, (uint32_t)(Off >> 32));
    }
    std::memcpy(B.data() + Header, Set.Texels.data(), Set.Texels.size() * 2);
    FILE* F = fopen(Path, "wb");
    if (!F) return false;
    const bool bOk = fwrite(B.data(), 1, B.size(), F) == B.size();
    fclose(F);
    return bOk;
}

// --- Main --------------------------------------------------------------------------------------

/** Joins the base cells (8-connected, wrapped) into one network: a multi-source Dijkstra from
 *  every piece at a step cost of (Cost + 0.02 x its spread), the cheapest join between each
 *  pair of touching regions, Kruskal's spanning tree over the pieces, and the chosen joins'
 *  paths back to their pieces marked as base. Returns the number of pieces found. */
static int ConnectBase(int N, const std::vector<double>& Cost, std::vector<uint8_t>& Base)
{
    const size_t NN = (size_t)N * N;
    auto At = [N](int x, int y) { x = (x % N + N) % N; y = (y % N + N) % N; return (size_t)y * N + x; };
    // Pieces.
    std::vector<int> Label(NN, -1);
    int Count = 0;
    for (size_t s = 0; s < NN; s++)
    {
        if (!Base[s] || Label[s] >= 0) continue;
        std::vector<size_t> Stack{ s };
        Label[s] = Count;
        while (!Stack.empty())
        {
            const size_t c = Stack.back(); Stack.pop_back();
            const int x = (int)(c % N), y = (int)(c / N);
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++)
                {
                    const size_t n = At(x + dx, y + dy);
                    if (Base[n] && Label[n] < 0) { Label[n] = Count; Stack.push_back(n); }
                }
        }
        Count++;
    }
    if (Count <= 1) return Count;
    double Spread = 0;
    for (double c : Cost) Spread = std::max(Spread, c);
    // Growth.
    std::vector<double> Dist(NN, 1e300);
    std::vector<int64_t> Parent(NN, -1);
    std::vector<int> Owner(Label);
    typedef std::pair<double, size_t> E;
    std::priority_queue<E, std::vector<E>, std::greater<E>> Q;
    for (size_t i = 0; i < NN; i++) if (Base[i]) { Dist[i] = 0; Q.push({ 0.0, i }); }
    while (!Q.empty())
    {
        const E Top = Q.top(); Q.pop();
        const size_t c = Top.second;
        if (Top.first > Dist[c]) continue;
        const int x = (int)(c % N), y = (int)(c / N);
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++)
            {
                if (!dx && !dy) continue;
                const size_t n = At(x + dx, y + dy);
                const double Step = (dx && dy) ? 1.41421356 : 1.0;
                const double D = Dist[c] + Step * (Cost[n] + 0.02 * Spread);
                if (D < Dist[n]) { Dist[n] = D; Parent[n] = (int64_t)c; Owner[n] = Owner[c]; Q.push({ D, n }); }
            }
    }
    // The cheapest join per pair of touching regions.
    struct FJoin { double W; size_t A, B; };
    std::map<std::pair<int, int>, FJoin> Joins;
    for (size_t c = 0; c < NN; c++)
    {
        const int x = (int)(c % N), y = (int)(c / N);
        for (int k = 0; k < 4; k++)
        {
            const size_t n = At(x + (k == 0) - (k == 1), y + (k == 2) - (k == 3));
            const int a = Owner[c], b = Owner[n];
            if (a == b || a < 0 || b < 0) continue;
            const double W = Dist[c] + Dist[n] + 0.5 * (Cost[c] + Cost[n]);
            const auto Key = std::make_pair(std::min(a, b), std::max(a, b));
            auto It = Joins.find(Key);
            if (It == Joins.end() || W < It->second.W) Joins[Key] = { W, c, n };
        }
    }
    std::vector<FJoin> Sorted;
    for (auto& J : Joins) Sorted.push_back(J.second);
    std::sort(Sorted.begin(), Sorted.end(), [](const FJoin& a, const FJoin& b) { return a.W < b.W; });
    std::vector<int> Root(Count);
    for (int i = 0; i < Count; i++) Root[i] = i;
    std::function<int(int)> Find = [&](int i) { return Root[i] == i ? i : (Root[i] = Find(Root[i])); };
    int Left = Count;
    for (const FJoin& J : Sorted)
    {
        const int a = Find(Owner[J.A]), b = Find(Owner[J.B]);
        if (a == b) continue;
        Root[a] = b;
        Left--;
        for (int64_t c : { (int64_t)J.A, (int64_t)J.B })
            for (; c >= 0 && !Base[(size_t)c]; c = Parent[(size_t)c]) Base[(size_t)c] = 2;
        if (Left == 1) break;
    }
    for (size_t i = 0; i < NN; i++) if (Base[i]) Base[i] = 1;
    return Count;
}

int main(int argc, char** argv)
{
    if (argc < 2) { printf("erode_tile out.vxnt [preset=legacy] [key=value ...]\n"); return 1; }
    FParams P;
    // Two passes: a preset first wherever it is given, then the other keys override it.
    for (int Pass = 0; Pass < 2; Pass++)
        for (int a = 2; a < argc; a++)
        {
            const char* Eq = strchr(argv[a], '=');
            if (!Eq) { printf("bad argument %s\n", argv[a]); return 1; }
            const std::string Key(argv[a], Eq - argv[a]);
            if ((Key == "preset") != (Pass == 0)) continue;
            char* End = nullptr;
            strtod(Eq + 1, &End);
            if (Key != "preset" && (End == Eq + 1 || *End != 0)) { printf("%s isn't a number\n", argv[a]); return 1; }
            if (!P.Set(Key.c_str(), Eq + 1)) { printf("unknown key or value %s\n", argv[a]); return 1; }
        }
    // Everything the plugin's reader and CheckPeaks will ask, before hours of simulation.
    {
        const char* Why = nullptr;
        if (P.N < 64 || P.N > 4096 || (P.N & (P.N - 1)) != 0) Why = "N must be a power of two in 64 .. 4096";
        else if (P.Massifs < 1 || P.N % P.Massifs != 0) Why = "Massifs must divide N";
        else if (P.N / P.Massifs < 32 || P.N / P.Massifs > 512) Why = "the massif spacing N / Massifs must be 32 .. 512 texels";
        else if (!(P.BaseShare > 0.001 && P.BaseShare < 0.5)) Why = "BaseShare must be in 0.001 .. 0.5 (the outlets)";
        else if (!(P.Jitter >= 0 && P.Jitter <= 0.45)) Why = "Jitter must be in 0 .. 0.45 (nodes must stay apart)";
        else if (P.Hillslope < 0 || P.Hillslope > 2 || P.BaseMode < 0 || P.BaseMode > 2 || P.SoftMode < 0 || P.SoftMode > 1 || P.RandomFill < 0 || P.RandomFill > 1)
            Why = "Hillslope and BaseMode are 0 .. 2; SoftMode and RandomFill are 0 or 1";
        else if (P.Steps < 1 || P.SoftSteps < 0 || !(P.Dt > 0) || !(P.K >= 0) || !(P.Eps > 0) || !(P.ScHard > 0) || !(P.ScSoft > 0) || P.KappaHard < 0 || P.KappaSoft < 0)
            Why = "Steps, Dt, Eps and the thresholds must be positive; K, the diffusions and SoftSteps not negative";
        if (Why) { printf("%s\n", Why); return 1; }
    }
    const int N = P.N;
    const size_t NN = (size_t)N * N;
    const double S = (double)N / P.Massifs;
    const double Scale = S / 64.0;   // the parameters' scale (see the top of the file)
    const auto T0 = std::chrono::steady_clock::now();

    // The base landscape and its base level. A periodic tile has no edge for water to leave by,
    // so some cells are held at 0 as outlets.
    std::vector<uint8_t> Base(NN);
    std::vector<double> U(NN), Start(NN, 0.0);
    if (P.BaseMode == 2)
    {
        const uint32_t Cells = (uint32_t)std::max(1.0, std::round(P.Massifs / std::max(P.BaseNetMassifs, 0.125)));
        const std::vector<double> Net = FbmField(N, P.Seed + 107u, Cells, 2);
        const std::vector<double> WX = FbmField(N, P.Seed + 108u, std::max(1u, Cells / 2), 2);
        const std::vector<double> WY = FbmField(N, P.Seed + 109u, std::max(1u, Cells / 2), 2);
        const std::vector<double> Env = FbmField(N, P.Seed + 101u, (uint32_t)std::max(1, P.Massifs / 2), 3);
        // The warped |n|: sampled from Net at an offset point (bilinear, wrapped).
        const double Reach = P.BaseNetWarp * (double)N / Cells;
        std::vector<double> D(NN);
        Par(N, [&](int32_t y) {
            for (int x = 0; x < N; x++)
            {
                const size_t i = (size_t)y * N + x;
                double fx = x + Reach * WX[i], fy = y + Reach * WY[i];
                fx -= std::floor(fx / N) * N; fy -= std::floor(fy / N) * N;
                const int x0 = (int)fx, y0 = (int)fy, x1 = (x0 + 1) % N, y1 = (y0 + 1) % N;
                const double tx = fx - x0, ty = fy - y0;
                const double V = (Net[(size_t)y0 * N + x0] * (1 - tx) + Net[(size_t)y0 * N + x1] * tx) * (1 - ty)
                    + (Net[(size_t)y1 * N + x0] * (1 - tx) + Net[(size_t)y1 * N + x1] * tx) * ty;
                D[i] = std::fabs(V);
            }
        });
        std::vector<double> Sorted(D);
        std::sort(Sorted.begin(), Sorted.end());
        const double Lo = Sorted[(size_t)(P.BaseShare * (NN - 1))], Hi = Sorted[(size_t)(0.995 * (NN - 1))];
        for (size_t i = 0; i < NN; i++) Base[i] = D[i] < Lo ? 1 : 0;
        // Level lines don't meet: the band is many loops and strands, and on a planet a loop at
        // one height with hills on both sides is a closed moat. Join them into one network: grow
        // every piece outward at a cost of |n| (so joins follow the low ground, near the zero
        // lines' saddles), then keep the cheapest joins between pieces as a spanning tree and
        // carve their paths.
        const int Joined = ConnectBase(N, D, Base);
        printf("  base network: %d pieces joined into one\n", Joined);
        for (size_t i = 0; i < NN; i++)
        {
            // Rising away from the network, its height varied by a massif-scale envelope.
            const double T = std::clamp((D[i] - Lo) / std::max((Hi - Lo) * P.BaseNetRamp, 1e-9), 0.0, 1.0);
            const double Pn = T * T * (3.0 - 2.0 * T) * std::clamp(0.7 + 0.5 * Env[i], 0.2, 1.2);
            if (Base[i]) continue;
            U[i] = P.UpliftScale * Scale * Pn;
            Start[i] = P.InitialHeight * Scale * Pn;
        }
    }
    else if (P.BaseMode == 1)
    {
        // Perlin fbm: its lowest BaseShare are the outlets (lowland basins of irregular shape,
        // which the erosion connects with dendritic valleys); above them p rises 0 .. 1 and sets
        // both the starting surface and the uplift, so the eroded relief follows the noise's
        // massifs while every valley is the simulation's own.
        const std::vector<double> F = FbmField(N, P.Seed + 101u, (uint32_t)std::max(1, P.Massifs / 2), (uint32_t)P.BaseOctaves);
        std::vector<double> Sorted(F);
        std::sort(Sorted.begin(), Sorted.end());
        const double Lo = Sorted[(size_t)(P.BaseShare * (NN - 1))], Hi = Sorted[(size_t)(0.995 * (NN - 1))];
        for (size_t i = 0; i < NN; i++)
        {
            const double Pn = std::clamp((F[i] - Lo) / std::max(Hi - Lo, 1e-9), 0.0, 1.0);
            Base[i] = F[i] < Lo ? 1 : 0;
            U[i] = P.UpliftScale * Scale * Pn;
            Start[i] = P.InitialHeight * Scale * Pn;
        }
    }
    else
    {
        // Voronoi massif cells (legacy): their warped borders are the base level, each cell's
        // value its uplift.
        FChannel Cell; Cell.Kind = KindCellularBorder; Cell.Seed = Mix(P.Seed + 101u); Cell.CellsPerTile = P.Massifs; Cell.Jitter = 0.85f;
        FChannel Warp; Warp.Kind = KindFbm; Warp.Seed = Mix(P.Seed + 202u); Warp.CellsPerTile = std::max(1, P.Massifs / 4); Warp.Octaves = 3; Warp.Gain = 0.5f;
        FChannel Warp2 = Warp; Warp2.Seed = Mix(P.Seed + 303u);
        Par(N, [&](int32_t y) {
            for (int x = 0; x < N; x++)
            {
                const double K = (double)P.Massifs / N, KW = (double)Warp.CellsPerTile / N;
                const double WX = x * K + P.BaseWarp * Evaluate(Warp, x * KW, y * KW);
                const double WY = y * K + P.BaseWarp * Evaluate(Warp2, x * KW, y * KW);
                FChannel V = Cell; V.Kind = KindCellularValue;
                Base[(size_t)y * N + x] = Evaluate(Cell, WX, WY) * S < P.BaseWidth ? 1 : 0;
                U[(size_t)y * N + x] = (0.5 + Evaluate(V, WX, WY)) * Scale;
            }
        });
    }

    // The fields (fixes: varied uplift, rock and threshold; multi-scale initial roughness).
    const uint32_t M = (uint32_t)P.Massifs;
    if (P.UpliftVariation > 0)
    {
        const std::vector<double> N1 = FbmField(N, P.Seed + 11u, std::max(1u, M / 2), 3);
        for (size_t i = 0; i < NN; i++) U[i] *= std::max(0.0, 1.0 + P.UpliftVariation * N1[i]);
    }
    std::vector<double> KField(NN, 1.0), ScHard(NN, P.ScHard), ScSoft(NN, P.ScSoft), H0(NN, 0.0);
    if (P.ErodibilityVariation > 0)
    {
        const std::vector<double> N2 = FbmField(N, P.Seed + 12u, M, 3);
        for (size_t i = 0; i < NN; i++) KField[i] = std::exp(P.ErodibilityVariation * N2[i]);
    }
    if (P.ThresholdVariation > 0)
    {
        const std::vector<double> N3 = FbmField(N, P.Seed + 13u, M * 4, 3);
        for (size_t i = 0; i < NN; i++)
        {
            ScHard[i] *= std::max(0.3, 1.0 + P.ThresholdVariation * N3[i]);
            ScSoft[i] *= std::max(0.3, 1.0 + P.ThresholdVariation * N3[i]);
        }
    }
    {
        std::vector<double> Rough;
        if (P.Roughness > 0) Rough = FbmField(N, P.Seed + 14u, M * 2, 4);
        for (size_t i = 0; i < NN; i++)
        {
            if (Base[i]) continue;
            // A trace of white noise breaks exact ties; the fbm gives the network somewhere to start.
            H0[i] = Start[i] + 0.01 * (Uniform((uint32_t)i, 7u, P.Seed) * 0.5 + 0.5) + (P.Roughness > 0 ? P.Roughness * Scale * (0.5 + 0.5 * Rough[i]) : 0.0);
        }
    }

    auto Run = [&](FSim& Sim, int Steps, double KScale, double Kappa, const std::vector<double>& Sc, const char* Label) {
        for (int s = 0; s < Steps; s++)
        {
            const double Change = Sim.Step(KScale, Kappa, Sc);
            if (s % 50 == 0 || s == Steps - 1)
            {
                double Hi = 0; for (double h : Sim.H) Hi = std::max(Hi, h);
                printf("  %s step %d: max height %.2f, largest change %.4f texels\n", Label, s, Hi, Change);
                fflush(stdout);
            }
        }
    };

    // Hard rock (R).
    FSim Hard(P);
    Hard.Init(Base, U, KField, H0);
    Run(Hard, P.Steps, 1.0, P.KappaHard * Scale * Scale, ScHard, "hard");
    // The routing of the final surface; R is its fill (W >= H): diffusion can leave shallow pits
    // on channel floors, and a set has no closed depressions off the outlets.
    Hard.Route();
    auto Pits = [&](const FSim& Sim, size_t& Count, double& Deepest) {
        Count = 0; Deepest = 0;
        for (size_t i = 0; i < NN; i++)
            if (Sim.W[i] > Sim.H[i]) { Count++; Deepest = std::max(Deepest, Sim.W[i] - Sim.H[i]); }
    };
    size_t PitsR = 0, PitsG = 0;
    double DeepR = 0, DeepG = 0;
    Pits(Hard, PitsR, DeepR);
    const std::vector<double> R = Hard.W;

    // Soft rock (G). Derived: R continues with soft rock's parameters, so it keeps R's drainage
    // network (its valleys and peaks sit in the same places; the plugin blends R and G per
    // sample). A separate run (legacy) diverges: the routing is chaotic in the parameters.
    FSim Soft(P);
    if (P.SoftMode == 1)
    {
        Soft.Init(Base, U, KField, R);
        Run(Soft, P.SoftSteps, P.KSoft, P.KappaSoft * Scale * Scale, ScSoft, "soft");
    }
    else
    {
        Soft.Init(Base, U, KField, H0);
        Run(Soft, P.Steps, 1.0, P.KappaSoft * Scale * Scale, ScSoft, "soft");
    }
    Soft.Route();
    Pits(Soft, PitsG, DeepG);
    const std::vector<double> G = Soft.W;

    // A: the spread area of R's routing. The base network is closed on a torus (no outlet), so
    // its own accumulation means nothing; it is marked as the tile's wettest ground (the p99 of
    // the channels').
    std::vector<double> Spread = P.SpreadCells > 0 ? Hard.SpreadArea() : Hard.Area;
    {
        std::vector<double> Ch;
        for (size_t i = 0; i < NN; i++) if (!Base[i] && Hard.Area[i] > 64.0 * Scale * Scale) Ch.push_back(Spread[i]);
        std::sort(Ch.begin(), Ch.end());
        const double Wet = Ch.empty() ? 1.0 : Ch[(size_t)(0.99 * (Ch.size() - 1))];
        for (size_t i = 0; i < NN; i++) if (Base[i]) Spread[i] = std::max(Spread[i], Wet);
    }

    // The set, named after the output file.
    const std::string Out = argv[1];
    const std::string Stem = Out.size() > 5 && Out.substr(Out.size() - 5) == ".vxnt" ? Out.substr(0, Out.size() - 5) : Out;
    FSet Set;
    Set.Name = Stem.substr(Stem.find_last_of("/\\") == std::string::npos ? 0 : Stem.find_last_of("/\\") + 1).substr(0, 31);
    Set.LookTag = "reference erosion (erode_tile.cpp)";
    Set.Seed = P.Seed;
    Set.N = (uint32_t)N;
    std::vector<double> Level[4];
    Level[0] = R;
    Level[1] = G;
    Level[2] = BlurPeriodic(R, (uint32_t)N, S / 4.0, [](int32_t Count, auto&& Body) { Par(Count, Body); });
    Level[3].resize(NN);
    for (size_t i = 0; i < NN; i++) Level[3][i] = std::log2(std::max(Spread[i], 1.0));
    const std::vector<double> LogArea = Level[3];   // Finish consumes Level
    // Fields that don't apply are zero (Addendum 2); R's and G's source is -1, none.
    for (FChannel& C : Set.Channel) { C.Octaves = 0; C.Gain = 0.0f; C.Jitter = 0.0f; C.ErosionK = 0.0f; }
    FChannel& CR = Set.Channel[0]; CR.Kind = KindErodedHeight; CR.Seed = P.Seed; CR.CellsPerTile = (uint32_t)P.Massifs; CR.Source = -1;
    Set.Channel[1] = CR;
    FChannel& CB = Set.Channel[2]; CB.Kind = KindBlurred; CB.Source = 0; CB.CellsPerTile = 0;
    FChannel& CA = Set.Channel[3]; CA.Kind = KindFlowAccumulation; CA.Source = 0; CA.CellsPerTile = 0;
    Finish(Set, Level);

    // Measures.
    const FMeasures MR = Measure(Hard, R, Scale);
    const FMeasures MG = Measure(Soft, G, Scale);
    const double Corr = Correlation(R, G), Aspect = AspectAgreement(Hard, R, G);
    const double Secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - T0).count();
    printf("R: channels continue %.3f, straight runs (>= 6 s cells) %.4f, longest %d; orientation C4 %.3f C8 %.3f; slope p50 %.1f p90 %.1f p99 %.1f deg; seam %.2f (interior %.2f .. %.2f)\n",
        MR.Continue, MR.Straight6, MR.Longest, MR.C4, MR.C8, MR.SlopeP50, MR.SlopeP90, MR.SlopeP99, MR.SeamRatio, MR.SeamLo, MR.SeamHi);
    printf("G: channels continue %.3f, straight runs (>= 6 s cells) %.4f, longest %d; orientation C4 %.3f C8 %.3f; slope p50 %.1f p90 %.1f p99 %.1f deg; seam %.2f (interior %.2f .. %.2f)\n",
        MG.Continue, MG.Straight6, MG.Longest, MG.C4, MG.C8, MG.SlopeP50, MG.SlopeP90, MG.SlopeP99, MG.SeamRatio, MG.SeamLo, MG.SeamHi);
    printf("R/G: height correlation %.3f, aspect agreement %.3f; pits filled: R %zu (deepest %.4f), G %zu (deepest %.4f) texels; %.0f s\n",
        Corr, Aspect, PitsR, DeepR, PitsG, DeepG, Secs);

    WriteShade(Stem + "_R.pgm", Hard, R);
    WriteShade(Stem + "_G.pgm", Hard, G);
    WriteShade(Stem + "_R2x2.pgm", Hard, R, 2);
    {
        std::vector<uint8_t> Px(NN);
        const double Top = std::log2((double)NN);
        for (size_t i = 0; i < NN; i++) Px[i] = (uint8_t)std::clamp(LogArea[i] / Top * 255.0, 0.0, 255.0);
        WritePgm(Stem + "_A.pgm", N, N, Px);
    }
    if (FILE* F = fopen((Stem + ".json").c_str(), "w"))
    {
        auto MJ = [](const FMeasures& M) {
            char B[512];
            snprintf(B, sizeof(B), "{\"continue\": %.4f, \"straight6\": %.4f, \"longest\": %d, \"C4\": %.4f, \"C8\": %.4f, \"slope_p50\": %.2f, \"slope_p90\": %.2f, \"slope_p99\": %.2f, \"seam\": %.3f, \"seam_interior_p1\": %.3f, \"seam_interior_p99\": %.3f}",
                M.Continue, M.Straight6, M.Longest, M.C4, M.C8, M.SlopeP50, M.SlopeP90, M.SlopeP99, M.SeamRatio, M.SeamLo, M.SeamHi);
            return std::string(B);
        };
        double UMax = 0;
        for (double u : U) UMax = std::max(UMax, u);
        fprintf(F, "{\"params\": %s,\n \"used\": {\"scale\": %.4f, \"uplift_max\": %.4f, \"kappa_hard\": %.4f, \"kappa_soft\": %.4f, \"K\": %.4f, \"blur_sigma\": %.3f},\n"
            " \"R\": %s,\n \"G\": %s,\n \"rg_correlation\": %.4f, \"rg_aspect\": %.4f,\n"
            " \"pits_filled\": {\"R\": %zu, \"R_deepest\": %.5f, \"G\": %zu, \"G_deepest\": %.5f}, \"seconds\": %.1f}\n",
            P.Json().c_str(), Scale, UMax, P.KappaHard * Scale * Scale, P.KappaSoft * Scale * Scale, Hard.KTile, S / 4.0,
            MJ(MR).c_str(), MJ(MG).c_str(), Corr, Aspect, PitsR, DeepR, PitsG, DeepG, Secs);
        fclose(F);
    }
    const std::string Why = CheckPeaks(Set);
    if (!Why.empty()) { printf("not a peaks set: %s\n", Why.c_str()); return 1; }
    if (!WriteVxnt(Set, argv[1])) { printf("couldn't write %s\n", argv[1]); return 1; }
    printf("wrote %s\n", argv[1]);
    return 0;
}
