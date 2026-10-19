// Harness for VoxelOceanTiles.h (stage O1): runs the shipped selection and tile geometry and
// checks the properties Design/OceanMeshing.md relies on.
#include "CoreMinimal.h"
#include VOT_HEADER
#include <cstdio>
#include <cstdlib>
#include <random>
#include <unordered_map>
#include <algorithm>
#include <utility>
#include <cmath>

using namespace VoxelOceanTiles;
using Key = FVoxelOceanTileKey;

struct FRun
{
    FSelectParams P;
    int32 Quads = 16;
    double MorphEnd0Value = 0.0;
    double StartFraction = 0.0;
};

static bool FindLeaf(const TSet<Key>& Sel, int32 MaxLevel, const FVector& Dir, Key& Out)
{
    int32 Face; double S, T;
    Locate(Dir, Face, S, T);
    for (int32 L = 0; L <= MaxLevel; L++)
    {
        const double N = (double)(1LL << L);
        const int32 I = (int32)FMath::Clamp(std::floor((S + 1.0) * 0.5 * N), 0.0, N - 1.0);
        const int32 J = (int32)FMath::Clamp(std::floor((T + 1.0) * 0.5 * N), 0.0, N - 1.0);
        const Key K(Face, L, I, J);
        if (Sel.Contains(K)) { Out = K; return true; }
    }
    return false;
}

static FVector WorldVertex(const Key& Tile, const FTileVertex& V, double Radius, bool bMorphed)
{
    FVector Origin; double Unit;
    PageFrame(PageOf(Tile), Radius, Origin, Unit);
    FVector P = Origin + FVector(V.Position.X, V.Position.Y, V.Position.Z) * Unit;
    if (bMorphed) P = P + FVector(V.Morph.X, V.Morph.Y, V.Morph.Z) * Unit;
    return P;
}

// Exact (double) grid point of a tile vertex.
static FVector ExactVertex(const Key& Tile, int32 Quads, int32 A, int32 B, double Radius)
{
    const int32 Bits = Tile.Level + FMath::FloorLog2((uint32)Quads);
    return Direction(Tile.Face, GridCoord((int64)Tile.I * Quads + A, Bits), GridCoord((int64)Tile.J * Quads + B, Bits)) * Radius;
}

static float WeightFloat(const FVector& Vertex, const FVector& Camera, int32 Level, double MorphEnd0Value, double StartFraction)
{
    // As the material: ToVertex in float, the band in float.
    const float Dx = (float)(Vertex.X - Camera.X), Dy = (float)(Vertex.Y - Camera.Y), Dz = (float)(Vertex.Z - Camera.Z);
    const float D = std::sqrt(Dx * Dx + Dy * Dy + Dz * Dz);
    if (Level <= 0) return 0.0f;
    const float End = (float)MorphEnd0Value * std::exp2(-(float)Level);
    const float Start = End * (float)StartFraction;
    float X = (D - Start) / (End - Start);
    X = X < 0.f ? 0.f : (X > 1.f ? 1.f : X);
    return X * X * (3.0f - 2.0f * X);
}

// A test wave: radial displacement from a sum of sines in the planet frame (cm), wavelengths
// from 3 m to 5 km, 1 m amplitude each. Any crack under it is what a wave model would show.
static FVector Displaced(const FVector& P)
{
    static const double K[4][3] = { { 0.0021, 0.0011, -0.0007 }, { -0.00013, 0.00011, 0.00009 }, { 0.000011, -0.000009, 0.000012 }, { -0.0000009, 0.0000008, 0.0000007 } };
    double H = 0.0;
    for (int32 i = 0; i < 4; i++) H += 100.0 * std::sin(K[i][0] * P.X + K[i][1] * P.Y + K[i][2] * P.Z + 0.7 * i);
    return P + P.GetSafeNormal() * H;
}

static double PointSegment(const FVector& P, const FVector& A, const FVector& B)
{
    const FVector E = B - A;
    const double L2 = E | E;
    const double T = L2 > 0.0 ? FMath::Clamp(((P - A) | E) / L2, 0.0, 1.0) : 0.0;
    return (P - (A + E * T)).Size();
}

// Where the material draws a vertex: the chained morph of VoxelOcean.ush (weights in float).
static FVector Displayed(const FRun& Run, const Key& Tile, const FTileVertex& V, const FVector& Camera, double* OutW1 = nullptr)
{
    FVector Origin; double Unit;
    PageFrame(PageOf(Tile), Run.P.Radius, Origin, Unit);
    const FVector Base = Origin + FVector(V.Position.X, V.Position.Y, V.Position.Z) * Unit;
    const FVector Parent = FVector(V.Morph.X, V.Morph.Y, V.Morph.Z) * Unit;
    const FVector Grand = FVector(V.Grand.X, V.Grand.Y, V.Grand.Z) * Unit;
    const float W0 = WeightFloat(Base, Camera, Tile.Level, Run.MorphEnd0Value, Run.StartFraction);
    const float W1 = WeightFloat(Base + Parent, Camera, Tile.Level - 1, Run.MorphEnd0Value, Run.StartFraction);
    if (OutW1) *OutW1 = W1;
#ifdef NO_CHAIN
    return Base + Parent * (double)W0;
#else
    return Base + (Parent + Grand * (double)W1) * (double)W0;
#endif
}

// As VoxelOcean.ush with the waves' fade by mesh spacing: the drawn point of a vertex under the
// test waves (radial at the morphed point), each wave faded by the vertex's spacing
// s_L 2^(W0 (1 + W1)) (1 from four spacings per wavelength, 0 at two).
static double ResolvedAt(double Wavelength, double Spacing) { return std::min(1.0, std::max(0.0, Wavelength / (2.0 * Spacing) - 1.0)); }
static FVector DrawnFaded(const FRun& Run, const Key& Tile, const FTileVertex& V, const FVector& Camera)
{
    static const double K[4][3] = { { 0.0021, 0.0011, -0.0007 }, { -0.00013, 0.00011, 0.00009 }, { 0.000011, -0.000009, 0.000012 }, { -0.0000009, 0.0000008, 0.0000007 } };
    FVector Origin; double Unit;
    PageFrame(PageOf(Tile), Run.P.Radius, Origin, Unit);
    const FVector Base = Origin + FVector(V.Position.X, V.Position.Y, V.Position.Z) * Unit;
    const FVector Parent = FVector(V.Morph.X, V.Morph.Y, V.Morph.Z) * Unit;
    const FVector Grand = FVector(V.Grand.X, V.Grand.Y, V.Grand.Z) * Unit;
    const float W0 = WeightFloat(Base, Camera, Tile.Level, Run.MorphEnd0Value, Run.StartFraction);
    const float W1 = W0 > 0.0f ? WeightFloat(Base + Parent, Camera, Tile.Level - 1, Run.MorphEnd0Value, Run.StartFraction) : 0.0f;
    const FVector M = Base + (Parent + Grand * (double)W1) * (double)W0;
    const double SpacingPerEnd = 1.0 / (MorphMargin * 2.0 * Run.P.LodDistanceFactor * Run.Quads);
    const double Spacing = Run.MorphEnd0Value * std::exp2((double)W0 * (1.0 + (double)W1) - Tile.Level) * SpacingPerEnd;
    double H = 0.0;
    const FVector Up = M.GetSafeNormal();
    FVector Horizontal(0.0, 0.0, 0.0);
    for (int32 i = 0; i < 4; i++)
    {
        const double Kn = std::sqrt(K[i][0] * K[i][0] + K[i][1] * K[i][1] + K[i][2] * K[i][2]);
        const double Rv = ResolvedAt(2.0 * UE_DOUBLE_PI / Kn, Spacing);
        const double Th = K[i][0] * M.X + K[i][1] * M.Y + K[i][2] * M.Z + 0.7 * i;
        H += 100.0 * Rv * std::sin(Th);
        // O4: Gerstner's horizontal part along the wave's tangent direction (choppiness 0.8).
        const FVector Kv(K[i][0], K[i][1], K[i][2]);
        const FVector Kt = Kv - Up * (Up | Kv);
        if (Kt.Size() > 1e-15) Horizontal = Horizontal + Kt.GetSafeNormal() * (0.8 * 100.0 * Rv * std::cos(Th));
    }
    return M + Up * H + Horizontal;
}

struct FStats
{
    int64 Cameras = 0, Tiles = 0, Edges = 0, BalanceViolations = 0, FineNotMorphed = 0, CoarseMorphing = 0, HorizonMisses = 0, Rays = 0;
    double MaxCrack = 0.0;           // cm, coarse morph at a shared edge times its offset
    double WorstCoarseRatio = 0.0;   // shared-edge distance / coarse morph end
    double ByLevel[32] = {};
    double MaxSameLevelMismatch = 0.0;
    double MaxSameLevelTargetMismatch = 0.0;   // cm, collapse targets of a shared vertex
    double MaxWaveCrack = 0.0;                 // cm, displaced fine vertex to displaced coarse edge
    double MaxWaveCrackRel = 0.0;              // of that, relative to the coarse spacing
    double MaxWaveCrackFine = 0.0;             // cm, fine tiles of level 10 and deeper
    double MaxFadedCrack = 0.0;                // cm, with the waves faded by mesh spacing
    double MaxFadedCrackFine = 0.0;
    int64 WaveSamples = 0, SeamSamples = 0;
    int64 ChainVertices = 0, ChainViolations = 0;   // level L - 2 morphing at a vertex's grand target
    double MaxReach = 0.0;                          // vertex distance / its level's morph end
    double MaxGrandMismatch = 0.0;                  // cm, grand targets of a shared seam vertex
    double MaxSeamDisplayed = 0.0;                  // cm, a shared seam vertex as drawn by both tiles
    int32 MaxTiles = 0;
};

static void RunCamera(const FRun& Run, const FVector& PrevCamera, const FVector& Camera, std::mt19937_64& Rng, FStats& St, bool bVerbose)
{
    TArray<Key> Tiles; TSet<Key> Split, PrevSplit; TArray<Key> PrevTiles;
    Select(PrevCamera, Run.P, nullptr, PrevTiles, PrevSplit);
    Select(Camera, Run.P, &PrevSplit, Tiles, Split);
    TSet<Key> Sel;
    for (const Key& K : Tiles) Sel.Add(K);
    St.Cameras++; St.Tiles += Tiles.Num(); St.MaxTiles = std::max(St.MaxTiles, Tiles.Num());

    const int32 Q = Run.Quads;
    for (const Key& Tile : Tiles)
    {
        // Sample just outside each edge at the midpoints of the tile's boundary vertices' spans.
        for (int32 Edge = 0; Edge < 4; Edge++)
        {
            for (int32 k = 0; k < 4; k++)
            {
                const double Along = (k + 0.5) / 4.0;
                const double N = (double)(1LL << Tile.Level);
                double S0 = -1.0 + 2.0 * Tile.I / N, S1 = -1.0 + 2.0 * (Tile.I + 1) / N;
                double T0 = -1.0 + 2.0 * Tile.J / N, T1 = -1.0 + 2.0 * (Tile.J + 1) / N;
                const double Eps = 1e-7 / N;
                double S, T;
                if (Edge == 0) { S = S0 - Eps; T = T0 + (T1 - T0) * Along; }
                else if (Edge == 1) { S = S1 + Eps; T = T0 + (T1 - T0) * Along; }
                else if (Edge == 2) { T = T0 - Eps; S = S0 + (S1 - S0) * Along; }
                else { T = T1 + Eps; S = S0 + (S1 - S0) * Along; }
                const FVector Dir = Direction(Tile.Face, S, T);
                Key Nb;
                if (!FindLeaf(Sel, Run.P.MaxLevel, Dir, Nb)) continue;
                St.Edges++;
                const int32 Diff = (int32)Tile.Level - (int32)Nb.Level;
                if (Diff > 1 || Diff < -1)
                {
                    St.BalanceViolations++;
                    if (bVerbose && St.BalanceViolations < 4)
                        std::printf("  balance: tile L%d (%d,%d,%d) next to L%d\n", Tile.Level, Tile.Face, Tile.I, Tile.J, Nb.Level);
                }
            }
        }
    }

    // Fine/coarse boundaries: every vertex of a fine tile on an edge shared with a coarser tile.
    for (const Key& Tile : Tiles)
    {
        if (Tile.Level == 0) continue;
        TArray<FTileVertex> Fine;
        BuildTile(Tile, Q, Run.P.Radius, Fine);
        for (int32 Edge = 0; Edge < 4; Edge++)
        {
            for (int32 i = 0; i <= Q; i++)
            {
                const int32 A = Edge == 0 ? 0 : (Edge == 1 ? Q : i);
                const int32 B = Edge == 2 ? 0 : (Edge == 3 ? Q : i);
                // The neighbour across this edge at this vertex: probe just outside it.
                const int32 Bits = Tile.Level + FMath::FloorLog2((uint32)Q);
                double S = GridCoord((int64)Tile.I * Q + A, Bits), T = GridCoord((int64)Tile.J * Q + B, Bits);
                const double Step = 2.0 / (double)(1LL << Bits);
                const double Off = 0.25 * Step;
                // Probe a quarter cell outside, beside the vertex along the edge (inside the span).
                const double Along = (i < Q ? 0.5 : -0.5) * Step;
                if (Edge == 0) { S -= Off; T += Along; } else if (Edge == 1) { S += Off; T += Along; }
                else if (Edge == 2) { T -= Off; S += Along; } else { T += Off; S += Along; }
                Key Nb;
                if (!FindLeaf(Sel, Run.P.MaxLevel, Direction(Tile.Face, S, T), Nb)) continue;
                if (Nb.Level + 1 != Tile.Level) continue;
                const FTileVertex& V = Fine[B * (Q + 1) + A];
                const FVector P = ExactVertex(Tile, Q, A, B, Run.P.Radius);
                const float WF = WeightFloat(P, Camera, Tile.Level, Run.MorphEnd0Value, Run.StartFraction);
                if (WF < 0.999999f) St.FineNotMorphed++;
                const float WC = WeightFloat(P, Camera, Nb.Level, Run.MorphEnd0Value, Run.StartFraction);
                const double CoarseStart = Run.MorphEnd0Value / (double)(1LL << Nb.Level);
                St.WorstCoarseRatio = std::max(St.WorstCoarseRatio, (P - Camera).Size() / CoarseStart);
                St.ByLevel[Nb.Level] = std::max(St.ByLevel[Nb.Level], (P - Camera).Size() / CoarseStart);
                if (WC > 1e-6f)
                {
                    // Crack: the coarse tile moves this point by WC times its own offset there.
                    // Locate the point in the coarse tile's grid.
                    TArray<FTileVertex> Coarse;
                    BuildTile(Nb, Q, Run.P.Radius, Coarse);
                    double Best = 1e300; int32 BestIdx = -1;
                    for (int32 v = 0; v < Coarse.Num(); v++)
                    {
                        const double Dd = (WorldVertex(Nb, Coarse[v], Run.P.Radius, false) - P).Size();
                        if (Dd < Best) { Best = Dd; BestIdx = v; }
                    }
                    const FTileVertex& CV = Coarse[BestIdx];
                    FVector Origin; double Unit; PageFrame(PageOf(Nb), Run.P.Radius, Origin, Unit);
                    const double Offset = FVector(CV.Morph.X, CV.Morph.Y, CV.Morph.Z).Size() * Unit;
                    if (Offset > 0.0)
                    {
                        St.CoarseMorphing++;
                        St.MaxCrack = std::max(St.MaxCrack, WC * Offset);
                    }
                }
                // The displaced surfaces: the fine vertex where the material puts it (float
                // weight, waves sampled at the morphed point) against the coarse edge through it.
                {
                    const FVector Fm = Displayed(Run, Tile, V, Camera);
                    TArray<FTileVertex> Coarse;
                    BuildTile(Nb, Q, Run.P.Radius, Coarse);
                    int32 B0 = -1, B1 = -1; double D0 = 1e300, D1 = 1e300;
                    for (int32 v = 0; v < Coarse.Num(); v++)
                    {
                        const double Dd = (WorldVertex(Nb, Coarse[v], Run.P.Radius, false) - P).Size();
                        if (Dd < D0) { D1 = D0; B1 = B0; D0 = Dd; B0 = v; }
                        else if (Dd < D1) { D1 = Dd; B1 = v; }
                    }
                    auto CoarseAt = [&](int32 v) { return Displaced(Displayed(Run, Nb, Coarse[v], Camera)); };
                    const double Spacing = (WorldVertex(Nb, Coarse[B0], Run.P.Radius, false) - WorldVertex(Nb, Coarse[B1], Run.P.Radius, false)).Size();
                    const double Crack = D0 < 1e-3 * Spacing ? (Displaced(Fm) - CoarseAt(B0)).Size()
                                                              : PointSegment(Displaced(Fm), CoarseAt(B0), CoarseAt(B1));
                    {
                        const FVector FF = DrawnFaded(Run, Tile, V, Camera);
                        const FVector C0 = DrawnFaded(Run, Nb, Coarse[B0], Camera), C1 = DrawnFaded(Run, Nb, Coarse[B1], Camera);
                        const double FC = D0 < 1e-3 * Spacing ? (FF - C0).Size() : PointSegment(FF, C0, C1);
                        St.MaxFadedCrack = std::max(St.MaxFadedCrack, FC);
                        if (Tile.Level >= 10) St.MaxFadedCrackFine = std::max(St.MaxFadedCrackFine, FC);
                    }
                    St.WaveSamples++;
                    St.MaxWaveCrack = std::max(St.MaxWaveCrack, Crack);
                    St.MaxWaveCrackRel = std::max(St.MaxWaveCrackRel, Crack / Spacing);
                    if (Tile.Level >= 10) St.MaxWaveCrackFine = std::max(St.MaxWaveCrackFine, Crack);
                }
            }
        }
    }

    // The two-level chain: wherever a vertex's grand target is, level L - 2 must not be morphing.
    for (int32 n = 0; n < 24 && n < Tiles.Num(); n++)
    {
        const Key& Tile = Tiles[(int32)(Rng() % (uint64)Tiles.Num())];
        if (Tile.Level < 2) continue;
        TArray<FTileVertex> Vs;
        BuildTile(Tile, Q, Run.P.Radius, Vs);
        FVector Origin; double Unit; PageFrame(PageOf(Tile), Run.P.Radius, Origin, Unit);
        for (const FTileVertex& V : Vs)
        {
            const FVector Base = WorldVertex(Tile, V, Run.P.Radius, false);
            const FVector GrandPoint = WorldVertex(Tile, V, Run.P.Radius, true) + FVector(V.Grand.X, V.Grand.Y, V.Grand.Z) * Unit;
            St.ChainVertices++;
            if (WeightFloat(GrandPoint, Camera, Tile.Level - 2, Run.MorphEnd0Value, Run.StartFraction) > 0.0f) St.ChainViolations++;
            St.MaxReach = std::max(St.MaxReach, (Base - Camera).Size() / (Run.MorphEnd0Value / (double)(1LL << Tile.Level)));
        }
    }

    // Same-level shared edge vertices (all four edges, across faces too): the same base point
    // and the same collapse target from both tiles, so seams stay closed at any weight.
    for (int32 n = 0; n < 12 && n < Tiles.Num(); n++)
    {
        const Key& Tile = Tiles[(int32)(Rng() % (uint64)Tiles.Num())];
        TArray<FTileVertex> Mine;
        BuildTile(Tile, Q, Run.P.Radius, Mine);
        const int32 Bits = Tile.Level + FMath::FloorLog2((uint32)Q);
        const double Step = 2.0 / (double)(1LL << Bits);
        for (int32 Edge = 0; Edge < 4; Edge++)
        {
            for (int32 i = 0; i <= Q; i++)
            {
                const int32 A = Edge == 0 ? 0 : (Edge == 1 ? Q : i);
                const int32 B = Edge == 2 ? 0 : (Edge == 3 ? Q : i);
                const FTileVertex& MV = Mine[B * (Q + 1) + A];
                const FVector P = WorldVertex(Tile, MV, Run.P.Radius, false);
                const FVector PT = WorldVertex(Tile, MV, Run.P.Radius, true);
                double S = GridCoord((int64)Tile.I * Q + A, Bits), T = GridCoord((int64)Tile.J * Q + B, Bits);
                const double Off = 0.25 * Step, Along = (i < Q ? 0.5 : -0.5) * Step;
                if (Edge == 0) { S -= Off; T += Along; } else if (Edge == 1) { S += Off; T += Along; }
                else if (Edge == 2) { T -= Off; S += Along; } else { T += Off; S += Along; }
                // Past the face's edge: Direction extrapolates, Locate finds the other face.
                Key Nb;
                if (!FindLeaf(Sel, Run.P.MaxLevel, Direction(Tile.Face, S, T), Nb) || Nb.Level != Tile.Level || Nb == Tile) continue;
                TArray<FTileVertex> Theirs;
                BuildTile(Nb, Q, Run.P.Radius, Theirs);
                double Best = 1e300; FVector BestT; int32 BestIdx = 0;
                for (int32 w = 0; w < Theirs.Num(); w++)
                {
                    const double Dd = (WorldVertex(Nb, Theirs[w], Run.P.Radius, false) - P).Size();
                    if (Dd < Best) { Best = Dd; BestT = WorldVertex(Nb, Theirs[w], Run.P.Radius, true); BestIdx = w; }
                }
                {
                    FVector O1, O2; double U1, U2;
                    PageFrame(PageOf(Tile), Run.P.Radius, O1, U1); PageFrame(PageOf(Nb), Run.P.Radius, O2, U2);
                    const FTileVertex& TV = Theirs[BestIdx];
                    const FVector G1 = PT + FVector(MV.Grand.X, MV.Grand.Y, MV.Grand.Z) * U1;
                    const FVector G2 = BestT + FVector(TV.Grand.X, TV.Grand.Y, TV.Grand.Z) * U2;
                    St.MaxGrandMismatch = std::max(St.MaxGrandMismatch, (G1 - G2).Size());
                    St.MaxSeamDisplayed = std::max(St.MaxSeamDisplayed, (Displayed(Run, Tile, MV, Camera) - Displayed(Run, Nb, TV, Camera)).Size());
                }
                St.SeamSamples++;
                St.MaxSameLevelMismatch = std::max(St.MaxSameLevelMismatch, Best);
                St.MaxSameLevelTargetMismatch = std::max(St.MaxSameLevelTargetMismatch, (BestT - PT).Size());
            }
        }
    }

    // Horizon: rays that hit the sea sphere must hit a selected tile.
    const double R0 = Camera.Size();
    for (int32 r = 0; r < 32; r++)
    {
        std::normal_distribution<double> G;
        FVector D(G(Rng), G(Rng), G(Rng));
        D = D.GetSafeNormal();
        const double B = Camera | D;
        const double C = R0 * R0 - Run.P.Radius * Run.P.Radius;
        const double Disc = B * B - C;
        if (Disc < 0.0) continue;
        double Tt = -B - std::sqrt(Disc);
        if (Tt < 0.0) Tt = -B + std::sqrt(Disc);
        if (Tt < 0.0) continue;
        St.Rays++;
        Key Hit;
        if (!FindLeaf(Sel, Run.P.MaxLevel, (Camera + D * Tt).GetSafeNormal(), Hit)) St.HorizonMisses++;
    }
}

// A fully collapsed tile, in exact grid indices: every triangle is degenerate or one of the
// parent's triangles, and each parent triangle appears exactly once.
static void CheckCollapse(int32 Q)
{
    int64 Tiles = 0, Bad = 0, Missing = 0, Twice = 0, Degenerate = 0;
    auto CheckTile = [&](const Key& Tile)
    {
        Tiles++;
        const Key Parent(Tile.Face, Tile.Level - 1, Tile.I >> 1, Tile.J >> 1);
        // Parent-tile grid indices of the fine tile's vertex (a, b) after collapse.
        auto Collapsed = [&](int32 A, int32 B)
        {
            int32 DA, DB;
            CollapseOffset(Tile, Q, A, B, DA, DB);
            const int32 FA = (Tile.I & 1) * Q + A + DA, FB = (Tile.J & 1) * Q + B + DB;
            if ((FA & 1) || (FB & 1)) Bad++;
            return std::make_pair(FA / 2, FB / 2);
        };
        std::unordered_map<int64, int32> Seen;
        auto PKey = [&](std::pair<int32, int32> V) { return (int64)V.second * 4096 + V.first; };
        for (int32 b = 0; b < Q; b++)
        {
            for (int32 a = 0; a < Q; a++)
            {
                int32 Idx[6];
                QuadTriangles(Q, a, b, false, Idx);
                for (int32 t = 0; t < 2; t++)
                {
                    std::pair<int32, int32> V[3];
                    for (int32 k = 0; k < 3; k++) V[k] = Collapsed(Idx[3 * t + k] % (Q + 1), Idx[3 * t + k] / (Q + 1));
                    if (V[0] == V[1] || V[1] == V[2] || V[0] == V[2]) { Degenerate++; continue; }
                    // Must be a parent triangle with the same winding: rotate to start at the min key.
                    int32 Pidx[6];
                    const int32 PA = std::min({ V[0].first, V[1].first, V[2].first }), PB = std::min({ V[0].second, V[1].second, V[2].second });
                    QuadTriangles(Q, PA, PB, false, Pidx);
                    bool bFound = false;
                    for (int32 pt = 0; pt < 2 && !bFound; pt++)
                    {
                        std::pair<int32, int32> W[3];
                        for (int32 k = 0; k < 3; k++) W[k] = std::make_pair(Pidx[3 * pt + k] % (Q + 1), Pidx[3 * pt + k] / (Q + 1));
                        for (int32 r = 0; r < 3; r++)
                            if (V[0] == W[r] && V[1] == W[(r + 1) % 3] && V[2] == W[(r + 2) % 3]) bFound = true;
                        if (bFound)
                        {
                            const int64 TKey = PKey(std::make_pair(PA, PB)) * 2 + pt;
                            if (++Seen[TKey] > 1) Twice++;
                        }
                    }
                    if (!bFound) Bad++;
                }
            }
        }
        // The fine tile covers a (Q/2)^2 block of parent quads: 2 triangles each.
        Missing += (int64)(Q / 2) * (Q / 2) * 2 - (int64)Seen.size();
    };
    for (int32 Face = 0; Face < 6; Face++)
        for (int32 L = 1; L <= 3; L++)
            for (int32 I = 0; I < (1 << L); I++)
                for (int32 J = 0; J < (1 << L); J++)
                    CheckTile(Key(Face, L, I, J));
    std::printf("Collapse (exact, %lld tiles, levels 1-3, every face): %lld bad triangles, %lld parent triangles missing, %lld covered twice, %lld degenerate\n",
        (long long)Tiles, (long long)Bad, (long long)Missing, (long long)Twice, (long long)Degenerate);
}

static void CheckGeometry(const FRun& Run)
{
    // Winding and exact parent coverage of fully morphed tiles.
    std::mt19937_64 Rng(7);
    int64 Inward = 0, Outward = 0, Triangles = 0;
    double MaxOffParent = 0.0, MaxOutside = 0.0, MaxRelative = 0.0, MaxFineLevels = 0.0;
    const int32 Q = Run.Quads;
    for (int32 n = 0; n < 600; n++)
    {
        const int32 Face = (int32)(Rng() % 6);
        const int32 L = 1 + (int32)(Rng() % 18);
        const int32 I = (int32)(Rng() % (1ull << L)), J = (int32)(Rng() % (1ull << L));
        const Key Tile(Face, L, I, J);
        TArray<FTileVertex> V;
        BuildTile(Tile, Q, Run.P.Radius, V);
        // Parent's triangles, from its exact grid.
        const Key Parent(Face, L - 1, I >> 1, J >> 1);
        auto ParentPoint = [&](int32 A, int32 B) { return ExactVertex(Parent, Q, A, B, Run.P.Radius); };
        for (int32 b = 0; b < Q; b++)
        {
            for (int32 a = 0; a < Q; a++)
            {
                int32 Idx[6];
                QuadTriangles(Q, a, b, false, Idx);
                for (int32 t = 0; t < 2; t++)
                {
                    const FVector P0 = WorldVertex(Tile, V[Idx[3 * t]], Run.P.Radius, false);
                    const FVector P1 = WorldVertex(Tile, V[Idx[3 * t + 1]], Run.P.Radius, false);
                    const FVector P2 = WorldVertex(Tile, V[Idx[3 * t + 2]], Run.P.Radius, false);
                    const double Dir = ((P1 - P0) ^ (P2 - P0)) | (P0 + P1 + P2);
                    Triangles++;
                    if (Dir < 0.0) Inward++; else Outward++;
                    // Fully morphed: the triangle must lie in one parent triangle.
                    const FVector M[3] = { WorldVertex(Tile, V[Idx[3 * t]], Run.P.Radius, true),
                        WorldVertex(Tile, V[Idx[3 * t + 1]], Run.P.Radius, true), WorldVertex(Tile, V[Idx[3 * t + 2]], Run.P.Radius, true) };
                    // The parent quad holding this fine quad, in the parent's grid.
                    const int32 PA = ((I & 1) * Q + a) / 2, PB = ((J & 1) * Q + b) / 2;
                    const FVector Q00 = ParentPoint(PA, PB), Q10 = ParentPoint(PA + 1, PB), Q01 = ParentPoint(PA, PB + 1), Q11 = ParentPoint(PA + 1, PB + 1);
                    const FVector Tri[2][3] = { { Q00, Q11, Q10 }, { Q00, Q01, Q11 } };
                    double BestOff = 1e300, BestOut = 1e300;
                    for (int32 pt = 0; pt < 2; pt++)
                    {
                        const FVector& A0 = Tri[pt][0]; const FVector E1 = Tri[pt][1] - A0, E2 = Tri[pt][2] - A0;
                        const FVector Nrm = (E1 ^ E2).GetSafeNormal();
                        double Off = 0.0, Out = 0.0;
                        for (int32 k = 0; k < 3; k++)
                        {
                            const FVector Rel = M[k] - A0;
                            Off = std::max(Off, std::fabs(Rel | Nrm));
                            // Barycentric (2D in the plane).
                            const double d00 = E1 | E1, d01 = E1 | E2, d11 = E2 | E2, d20 = Rel | E1, d21 = Rel | E2;
                            const double Den = d00 * d11 - d01 * d01;
                            const double Bv = (d11 * d20 - d01 * d21) / Den, Bw = (d00 * d21 - d01 * d20) / Den, Bu = 1.0 - Bv - Bw;
                            const double Len = std::sqrt(d00);
                            Out = std::max(Out, std::max(0.0, -std::min(Bu, std::min(Bv, Bw))) * Len);
                        }
                        if (Off + Out < BestOff + BestOut) { BestOff = Off; BestOut = Out; }
                    }
                    MaxOffParent = std::max(MaxOffParent, BestOff);
                    MaxOutside = std::max(MaxOutside, BestOut);
                    FVector Org; double Unit; PageFrame(PageOf(Tile), Run.P.Radius, Org, Unit);
                    MaxRelative = std::max(MaxRelative, (BestOff + BestOut) / Unit);
                    if (L >= 8) MaxFineLevels = std::max(MaxFineLevels, BestOff + BestOut);
                }
            }
        }
    }
    std::printf("Winding: %lld triangles, %lld inward, %lld outward (want all inward)\n", (long long)Triangles, (long long)Inward, (long long)Outward);
    std::printf("Fully morphed tiles on the parent's triangles: off-plane %.3g cm, outside %.3g cm (max over %d tiles); %.3g page units at worst (float rounding is 6e-8); %.3g cm at levels 8 and finer\n", MaxOffParent, MaxOutside, 600, MaxRelative, MaxFineLevels);

    // Cross-face edge vertices.
    double MaxFace = 0.0;
    for (int32 Face = 0; Face < 6; Face++)
    {
        for (int32 Edge = 0; Edge < 4; Edge++)
        {
            for (int32 k = 0; k <= 64; k++)
            {
                const double U = -1.0 + 2.0 * k / 64.0;
                const double S = Edge == 0 ? -1.0 : (Edge == 1 ? 1.0 : U), T = Edge == 2 ? -1.0 : (Edge == 3 ? 1.0 : U);
                const FVector D = Direction(Face, S, T);
                // The same point from the other faces that share it.
                for (int32 Other = 0; Other < 6; Other++)
                {
                    if (Other == Face) continue;
                    const FFace& F = GetFace(Other);
                    const double Nd = F.N | D;
                    if (Nd < 0.5) continue;
                    const double Os = Unwarp((F.U | D) / Nd), Ot = Unwarp((F.V | D) / Nd);
                    if (std::fabs(Os) > 1.0 + 1e-12 || std::fabs(Ot) > 1.0 + 1e-12) continue;
                    // Snap to the 64-grid of the other face and compare.
                    const double Ss = std::round((Os + 1.0) * 32.0) / 32.0 - 1.0, Ts = std::round((Ot + 1.0) * 32.0) / 32.0 - 1.0;
                    MaxFace = std::max(MaxFace, (Direction(Other, Ss, Ts) - D).Size() * Run.P.Radius);
                }
            }
        }
    }
    std::printf("Cross-face edge vertices: max mismatch %.3g cm\n", MaxFace);
}

int main(int Argc, char** Argv)
{
    const double Radius = 1.075e8;
    FRun Run;
    Run.Quads = 16;
    Run.P.Radius = Radius;
    Run.P.MaxLevel = LevelForSpacing(Radius, Run.Quads, 50.0);
    Run.P.HorizonHeight = 1000.0;
    const double Factors[] = { 2.25, 2.5, 3.0, 4.0 };
    const double Hysts[] = { 1.1 };
    const int32 Cameras = Argc > 1 ? std::atoi(Argv[1]) : 600;
    const double StartFractionArg = Argc > 2 ? std::atof(Argv[2]) : DefaultMorphStartFraction;
    std::printf("Radius %.0f cm, %d quads, MaxLevel %d (spacing %.1f cm)\n", Radius, Run.Quads, Run.P.MaxLevel,
        NominalArc(Run.P.MaxLevel) * Radius / Run.Quads);
    CheckCollapse(Run.Quads);
    CheckCollapse(8);
    CheckCollapse(32);
    CheckGeometry(Run);
    for (double H : Hysts) for (double F : Factors)
    {
        Run.P.Hysteresis = H;
        Run.P.LodDistanceFactor = F;
        Run.MorphEnd0Value = MorphEnd0(Radius, F);
        Run.StartFraction = StartFractionArg;
        std::printf("F %.2f: morph start fraction %.3f\n", F, Run.StartFraction);
        std::mt19937_64 Rng(1234);
        FStats St;
        std::uniform_real_distribution<double> U01(0.0, 1.0);
        std::normal_distribution<double> G;
        for (int32 c = 0; c < Cameras; c++)
        {
            // Direction: random, or near a face edge or corner.
            FVector D(G(Rng), G(Rng), G(Rng));
            const int32 Kind = c % 3;
            if (Kind == 1) D = FVector(1.0, 1.0, 0.0) + D * 0.02;
            if (Kind == 2) D = FVector(1.0, 1.0, 1.0) + D * 0.02;
            D = D.GetSafeNormal();
            const double Altitude = (c % 10 == 0) ? -1000.0 * U01(Rng) : std::pow(10.0, 2.0 + 7.0 * U01(Rng));   // -10 m .. 10,000 km
            const FVector Camera = D * (Radius + Altitude);
            // The previous camera: a step back of up to a tenth of the altitude (hysteresis state).
            const FVector Step = FVector(G(Rng), G(Rng), G(Rng)).GetSafeNormal() * (std::fabs(Altitude) * 0.1 * U01(Rng) + 100.0);
            RunCamera(Run, Camera + Step, Camera, Rng, St, false);
        }
        std::printf("H %.2f F %.1f: %lld cameras, %.0f tiles average (%d max), %lld edge samples: %lld balance violations, %lld fine edge vertices not fully morphed, "
                    "%lld coarse vertices morphing at a boundary (by up to %.3g cm, followed by the chain; worst distance / coarse morph end %.3f), same-level edge mismatch %.3g cm, %lld of %lld rays missed\n"
                    "  %lld seam samples: collapse targets differ by %.3g cm at most; %lld boundary samples under 1 m test waves: max crack %.3g cm (%.2g of the coarse spacing), %.3g cm at levels 10+\n"
                    "  with the waves faded by mesh spacing: max crack %.3g cm, %.3g cm at levels 10+\n"
                    "  chain: %lld vertices, %lld with level L-2 morphing at their grand target (want 0), farthest vertex %.3f of its level's morph end (chain needs < 2); seams: grand targets differ by %.3g cm, drawn positions by %.3g cm\n",
            H, F, (long long)St.Cameras, (double)St.Tiles / St.Cameras, St.MaxTiles, (long long)St.Edges, (long long)St.BalanceViolations, (long long)St.FineNotMorphed,
            (long long)St.CoarseMorphing, St.MaxCrack, St.WorstCoarseRatio, St.MaxSameLevelMismatch, (long long)St.HorizonMisses, (long long)St.Rays,
            (long long)St.SeamSamples, St.MaxSameLevelTargetMismatch, (long long)St.WaveSamples, St.MaxWaveCrack, St.MaxWaveCrackRel, St.MaxWaveCrackFine, St.MaxFadedCrack, St.MaxFadedCrackFine,
            (long long)St.ChainVertices, (long long)St.ChainViolations, St.MaxReach, St.MaxGrandMismatch, St.MaxSeamDisplayed);
        std::printf("  worst ratio by coarse level:");
        for (int32 l = 0; l < 20; l++) if (St.ByLevel[l] > 0) std::printf(" L%d %.3f", l, St.ByLevel[l]);
        std::printf("\n");
    }
    return 0;
}

