// coast_check.cpp - Stage O9h's coastal travel-time field offline (VoxelOceanCoastCore.h: the
// plan, the solve, the packing; VoxelOceanCoast.ush: the sampling the shaders and the queries
// share) on a synthetic planet whose depths are known exactly.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private coast_check.cpp -o coast_check
//   ./coast_check
//
// THE PLANET: a 300 km sea sphere, its land bake N 128 (3.7 km cells). A round continent (a cap
// 0.35 rad across, its centre near a cube corner so its coast crosses three faces' edges), its
// beach falling seaward along the arc from its coast at 1:30 on one side and 1:100 on the other
// (smoothly between, by the bearing), to 200 m deep. An island off it.

#include "VoxelShaderCompat.h"
#include "VoxelOceanCoastCore.h"
#include "VoxelLandSea.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace VoxelShader;

static int Fails = 0;
static void Check(bool bOk, const char* What, double Value, double Limit)
{
    std::printf("  [%s] %-74s %.6g (limit %.6g)\n", bOk ? "ok" : "FAIL", What, Value, Limit);
    if (!bOk) Fails++;
}

struct FDir { double X, Y, Z; };
static FDir Norm(FDir D) { const double L = std::sqrt(D.X * D.X + D.Y * D.Y + D.Z * D.Z); return { D.X / L, D.Y / L, D.Z / L }; }
static double Dot(FDir A, FDir B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
static FDir Cross(FDir A, FDir B) { return { A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X }; }
static FDir Scale(FDir A, double S) { return { A.X * S, A.Y * S, A.Z * S }; }
static FDir Add(FDir A, FDir B) { return { A.X + B.X, A.Y + B.Y, A.Z + B.Z }; }

static const double RadiusM = 3.0e5;
static FDir Centre, East, North;
static const double CapAngle = 0.35;
static FDir IslandCentre;
static const double IslandAngle = 0.012;

// The beach slope by bearing (1:30 toward East, 1:100 opposite).
static double SlopeAt(FDir D)
{
    FDir T = Add(D, Scale(Centre, -Dot(D, Centre)));
    const double L = std::sqrt(Dot(T, T));
    if (L < 1e-12) return 1.0 / 30.0;
    T = Scale(T, 1.0 / L);
    const double C = Dot(T, East);
    const double W = 0.5 + 0.5 * C;
    return std::exp(std::log(1.0 / 100.0) * (1.0 - W) + std::log(1.0 / 30.0) * W);
}

// Height (m) at a direction: the continent and the island, seaward depth along the arc.
static double HeightAt(FDir D)
{
    const double A = std::acos(std::clamp(Dot(D, Centre), -1.0, 1.0));
    const double X = (A - CapAngle) * RadiusM;           // seaward distance from the coast
    double H = X <= 0.0 ? std::min(-X * 0.01, 300.0) : -std::min(X * SlopeAt(D), 200.0);
    const double AI = std::acos(std::clamp(Dot(D, IslandCentre), -1.0, 1.0));
    const double XI = (AI - IslandAngle) * RadiusM;
    const double HI = XI <= 0.0 ? std::min(-XI * 0.02, 100.0) : -std::min(XI / 40.0, 200.0);
    return std::max(H, HI);
}

static double KOf(double K0, double D)
{
    // x tanh x = K0 D by Newton.
    const double Q = K0 * D;
    double X = Q < 1.0 ? std::sqrt(Q) : Q;
    for (int i = 0; i < 50; i++)
    {
        const double T = std::tanh(X);
        const double F = X * T - Q, DF = T + X * (1.0 - T * T);
        X -= F / DF;
    }
    return X / D;
}

int main()
{
    Centre = Norm({ 0.62, 0.58, 0.53 });
    East = Norm(Cross({ 0, 0, 1 }, Centre));
    North = Cross(Centre, East);
    // The island 6 km off the 1:30 side.
    {
        const double A = CapAngle + 6000.0 / RadiusM;
        IslandCentre = Norm(Add(Scale(Centre, std::cos(A)), Scale(East, std::sin(A))));
    }
    const int N = 128;
    const int Pad = VoxelOceanShoreCore::Pad;
    const int S = VoxelOceanShoreCore::Side(N);
    const int Count = VoxelOceanShoreCore::NumTexels(N);
    const double ReachM = 30000.0;
    std::printf("coast_check: sea sphere %.0f km, bake N %d (%.2f km cells)\n", RadiusM * 1e-3, N, 0.5 * 3.14159265 * RadiusM / N * 1e-3);

    // The bake's heights (apron cells at their own centres) and sea bits, then the shore field.
    std::vector<float> Height(static_cast<size_t>(Count));
    for (int F = 0; F < 6; F++)
        for (int J = -Pad; J < N + Pad; J++)
            for (int I = -Pad; I < N + Pad; I++)
            {
                const float3 V = VoxelOceanShoreCellDir(F, I, J, N);
                Height[size_t(VoxelOceanCoastCore::IndexTexel(N, F, I, J))] = float(HeightAt({ V.x, V.y, V.z }));
            }
    std::vector<uint16_t> Attr(static_cast<size_t>(Count), uint16_t(0));
    VoxelLandSea::Mark(N, Pad, Height.data(), Attr.data(), 0x800, 0.02);
    const std::vector<uint32_t> Shore = VoxelOceanShoreCore::Build(N, RadiusM, ReachM, Height.data(), Attr.data(),
        [](int Cnt, auto&& Fn) { for (int i = 0; i < Cnt; i++) Fn(i); });

    // -----------------------------------------------------------------------------------------
    std::printf("\n1. The plan\n");
    VoxelOceanCoastCore::FSettings Settings;
    Settings.SeedDepthM = 40.0;
    Settings.ReachM = 8000.0;
    Settings.BudgetBytes = 48.0 * 1024.0 * 1024.0;
    const VoxelOceanCoastCore::FPlan P = VoxelOceanCoastCore::Plan(N, RadiusM, Shore.data(), ReachM, Height.data(), Settings);
    std::printf("  %d tiles (%d interior), R %d: cells %.1f m, atlas %d slices %.1f MB, %zu interior cells\n", int(P.Tiles.size()), P.InteriorTiles, P.R, P.CellM,
        P.AtlasSlices, P.AtlasBytes / 1048576.0, P.InteriorCells());
    Check(P.bValid && P.AtlasBytes <= Settings.BudgetBytes, "the atlas fits the budget", P.AtlasBytes / 1048576.0, Settings.BudgetBytes / 1048576.0);
    Check(P.CellM >= Settings.MinCellM && P.CellM <= Settings.MaxCellM, "the cells within 25 to 150 m", P.CellM, Settings.MaxCellM);
    if (!P.bValid) { std::printf("\nFAILED: no plan\n"); return 1; }

    // The fine cells' depths, exact (the GPU samples the terrain's, smoothed over a cell).
    std::vector<float> Depth(P.InteriorCells());
    for (size_t c = 0; c < Depth.size(); c++)
    {
        double D[3];
        VoxelOceanCoastCore::InteriorCellDir(P, c, D);
        Depth[c] = float(-HeightAt({ D[0], D[1], D[2] }));
    }
    const VoxelOceanCoastCore::FSolved V = VoxelOceanCoastCore::Solve(P, Depth.data(), Shore.data(), ReachM);
    std::printf("  solve: %zu wet cells, %zu sources, %zu reached, %.2f s (%.2f us a cell)\n", V.Wet, V.Seeds, V.Reached, V.Seconds, V.Seconds * 1e6 / std::max<size_t>(Depth.size(), 1));
    Check(V.Reached > V.Wet * 95 / 100, "the rays reach the water (95% at least)", double(V.Reached) / double(std::max<size_t>(V.Wet, 1)), 0.95);
    const auto TP = std::chrono::steady_clock::now();
    const std::vector<uint32_t> Coast = VoxelOceanCoastCore::Pack(P, V);
    std::printf("  pack: %.2f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - TP).count());
    float Row0[4], Row1[4];
    VoxelOceanCoastCore::LayoutRows(P, Row0, Row1);
    auto SampleAt = [&](FDir D) { return VoxelOceanCoastCore::Sample(Shore.data(), Coast.data(), P.Slices(), N, Row0, Row1, RadiusM, D.X, D.Y, D.Z); };

    // A point Dist metres seaward of the continent's coast along bearing Theta (0 toward East).
    auto Seaward = [&](double Theta, double Dist)
        {
            const FDir T = Add(Scale(East, std::cos(Theta)), Scale(North, std::sin(Theta)));
            const double A = CapAngle + Dist / RadiusM;
            return Norm(Add(Scale(Centre, std::cos(A)), Scale(T, std::sin(A))));
        };

    // -----------------------------------------------------------------------------------------
    std::printf("\n2. The crests' spacing: the phase's gradient against the exact local wavenumber\n");
    {
        double Worst = 0.0, WorstD = 0.0, WorstT = 0.0, WorstTheta = 0.0, WorstAngle = 0.0, WorstFD = 0.0;
        int Points = 0;
        for (double Period : { 6.0, 9.0, 14.0 })
        {
            const double K0 = (2.0 * 3.14159265358979 / Period) * (2.0 * 3.14159265358979 / Period) / 9.81;
            // Up to a fifth of the deep wavelength (VoxelOceanCoast.ush: within 4% there), and 0.8 of the seed.
            const double Cut = std::min(0.2 * 2.0 * 3.14159265358979 / K0, 0.8 * Settings.SeedDepthM);
            for (int b = 0; b < 24; b++)
            {
                const double Theta = 0.6 + 2.0 * 3.14159265358979 * b / 24.0;   // off the island's line
                for (double Dist = 20.0; Dist < 6000.0; Dist *= 1.08)
                {
                    const FDir D = Seaward(Theta, Dist);
                    const double Dm = -HeightAt(D);
                    if (Dm < 2.5 || Dm > Cut) continue;
                    // Away from the island.
                    if (std::acos(std::clamp(Dot(D, IslandCentre), -1.0, 1.0)) * RadiusM < 4000.0) continue;
                    const FVoxelOceanCoastSample C = SampleAt(D);
                    if (C.Cover < 0.99f) continue;
                    const float4 Ph = VoxelOceanCoastPhase(C, float(K0));
                    const double G = std::sqrt(double(Ph.y) * Ph.y + double(Ph.z) * Ph.z + double(Ph.w) * Ph.w);
                    const double Exact = KOf(K0, double(VoxelOceanCoastHeldDepth(float(Dm))));
                    const double Err = std::fabs(G / Exact - 1.0);
                    // The gradient's direction against shoreward (toward the cap's centre).
                    const FDir Toward = Norm(Add(Centre, Scale(D, -Dot(D, Centre))));
                    const double Cos = (Ph.y * Toward.X + Ph.z * Toward.Y + Ph.w * Toward.Z) / std::max(G, 1e-12);
                    const double Angle = std::acos(std::clamp(Cos, -1.0, 1.0)) * 57.29578;
                    // The phase's own change along the ray (5 m either side) against the gradient.
                    const FDir D1 = Seaward(Theta, Dist - 5.0), D2 = Seaward(Theta, Dist + 5.0);
                    const double FD = (VoxelOceanCoastPhase(SampleAt(D1), float(K0)).x - VoxelOceanCoastPhase(SampleAt(D2), float(K0)).x) / 10.0;
                    const double FDErr = std::fabs(FD / (G * Cos) - 1.0);
                    WorstAngle = std::max(WorstAngle, Angle);
                    WorstFD = std::max(WorstFD, FDErr);
                    Points++;
                    if (Err > Worst) { Worst = Err; WorstD = Dm; WorstT = Period; WorstTheta = Theta; }
                }
            }
        }
        std::printf("  %d points 2.5 m deep to a fifth of the deep wavelength: worst %.2f%% (%.0f s, %.1f m deep, bearing %.2f)\n", Points, Worst * 100.0, WorstT, WorstD, WorstTheta);
        // 7%: the series' 4% at a fifth of the deep wavelength (VoxelOceanCoast.ush), and up to
        // 6% from the grid within 2.5 to 4 m of depth on the 1:30 beach (57 m cells).
        // (Stage W5k: 8%, was 7%: the tolerant selection's more tiles take this small planet's
        // atlas from 64 fine cells a tile to 63, 58.4 m cells against 57.5, and the worst point (a
        // 14 s swell 2.9 m deep) goes from 6.1% to 7.5%; the old selection at 63 gives the same.)
        Check(Points > 500 && Worst < 0.08, "the crests are spaced at the local wavelength (8%)", Worst, 0.08);
        Check(WorstAngle < 4.0, "the rays run shoreward, normal to a round coast (degrees)", WorstAngle, 4.0);
        Check(WorstFD < 0.03, "the phase's change along the ray is its gradient's (3%)", WorstFD, 0.03);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n3. Continuity along the coast (across tiles and face edges)\n");
    {
        double WorstStep = 0.0, WorstGrad = 0.0, MaxSpread = 0.0;
        int FaceChanges = 0;
        for (double Dist : { 60.0, 300.0, 900.0 })
        {
            const double K0 = (2.0 * 3.14159265358979 / 9.0) * (2.0 * 3.14159265358979 / 9.0) / 9.81;
            double Prev = 0.0, PrevG = 0.0, Lo = 1e30, Hi = -1e30;
            int PrevFace = -1;
            const int Steps = 40000;
            for (int k = 0; k <= Steps; k++)
            {
                const double Theta = 2.0 * 3.14159265358979 * k / Steps;
                const FDir D = Seaward(Theta, Dist);
                if (std::acos(std::clamp(Dot(D, IslandCentre), -1.0, 1.0)) * RadiusM < 9000.0) { PrevFace = -1; continue; }
                const FVoxelOceanFacePoint F = VoxelOceanLocate(float3(float(D.X), float(D.Y), float(D.Z)));
                const FVoxelOceanCoastSample C = SampleAt(D);
                const float4 Ph = VoxelOceanCoastPhase(C, float(K0));
                const double G = std::sqrt(double(Ph.y) * Ph.y + double(Ph.z) * Ph.z + double(Ph.w) * Ph.w);
                if (PrevFace >= 0)
                {
                    // Along the coast the phase barely changes (the rays are normal to it): any
                    // step shows as a jump against the tangential gradient's allowance.
                    const double StepM = 2.0 * 3.14159265358979 / Steps * std::sin(CapAngle) * RadiusM;
                    const double Allowed = 1.5 * G * StepM + 1.0e-3;
                    WorstStep = std::max(WorstStep, std::fabs(Ph.x - Prev) / Allowed);
                    WorstGrad = std::max(WorstGrad, std::fabs(G - PrevG) / std::max(G, 1e-6));
                    if (F.Face != PrevFace) FaceChanges++;
                }
                Lo = std::min(Lo, double(Ph.x)); Hi = std::max(Hi, double(Ph.x));
                Prev = Ph.x; PrevG = G; PrevFace = F.Face;
            }
            MaxSpread = std::max(MaxSpread, (Hi - Lo) / (2.0 * 3.14159265358979));
        }
        std::printf("  %d face crossings; the phase along a coast-parallel line varies by %.2f cycles\n", FaceChanges, MaxSpread);
        Check(FaceChanges >= 6, "the paths cross face edges", FaceChanges, 6);
        Check(WorstStep < 1.0, "no step in the phase (over its gradient's allowance)", WorstStep, 1.0);
        Check(WorstGrad < 0.02, "no jump in the crests' spacing (2% a step)", WorstGrad, 0.02);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n4. The texels against the solve\n");
    {
        double Worst = 0.0;
        int Points = 0;
        for (size_t c = 0; c < V.A.size(); c += 97)
        {
            if (!V.Trust[c] || !(Depth[c] > 2.0f) || !(Depth[c] < 0.9f * Settings.SeedDepthM)) continue;
            double D[3];
            VoxelOceanCoastCore::InteriorCellDir(P, c, D);
            // Away from the face edges' blend (there the neighbouring face's resampled apron joins in).
            const FVoxelOceanFacePoint F = VoxelOceanLocate(float3(float(D[0]), float(D[1]), float(D[2])));
            if (std::max(std::fabs(F.S), std::fabs(F.T)) > 1.0f - 4.0f / float(N)) continue;
            const FVoxelOceanCoastSample C = SampleAt({ D[0], D[1], D[2] });
            if (C.Cover < 0.99f) continue;
            // At a cell's centre Catmull-Rom returns the cell's value: the codes' rounding.
            Worst = std::max(Worst, std::fabs(double(C.A) - V.A[c]));
            Points++;
        }
        std::printf("  %d cells: A within %.4f m^1/2\n", Points, Worst);
        Check(Points > 100 && Worst < 0.05, "a cell's sample is its solved value away from face edges (A within 0.05 m^1/2: 0.02 rad at 6 s)", Worst, 0.05);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n5. Land and the island\n");
    {
        // Up the beach the phase runs on at the floor's slowness: its gradient continues the
        // shoreline's, trusted.
        double Worst = 0.0;
        for (int b = 0; b < 24; b++)
        {
            const double Theta = 0.6 + 2.0 * 3.14159265358979 * b / 24.0;
            for (double Up = 4.0 * P.CellM; Up < 12.0 * P.CellM; Up += P.CellM)
            {
                const FVoxelOceanCoastSample C = SampleAt(Seaward(Theta, -Up));
                const double G = std::sqrt(double(C.GradA.x) * C.GradA.x + double(C.GradA.y) * C.GradA.y + double(C.GradA.z) * C.GradA.z);
                Worst = std::max(Worst, std::fabs(G * std::sqrt(double(VOXEL_OCEAN_COAST_FLOOR)) - 1.0));
                if (C.Cover < 0.99f) Worst = 1e9;
            }
        }
        std::printf("  up the beach (4 to 12 cells): |grad A| sqrt(floor) within %.1f%% of 1\n", Worst * 100.0);
        Check(Worst < 0.1, "up the beach the phase runs on at the floor's wavenumber (10%), trusted", Worst, 0.1);
        // Between the island and the coast the rays from either end of the channel meet: the
        // phase stays continuous (its crests turn there), the spacing reported.
        double WorstLee = 0.0, WorstStep = 0.0;
        const double K0 = (2.0 * 3.14159265358979 / 9.0) * (2.0 * 3.14159265358979 / 9.0) / 9.81;
        for (double Dist = 300.0; Dist < 2000.0; Dist += 100.0)
        {
            double Prev = 0.0, PrevG = 0.0;
            bool bHave = false;
            for (double Theta = -0.06; Theta <= 0.06; Theta += 0.00005)
            {
                const FDir D = Seaward(Theta, Dist);
                const double Dm = -HeightAt(D);
                if (Dm < 2.5) { bHave = false; continue; }
                const FVoxelOceanCoastSample C = SampleAt(D);
                const float4 Ph = VoxelOceanCoastPhase(C, float(K0));
                const double G = std::sqrt(double(Ph.y) * Ph.y + double(Ph.z) * Ph.z + double(Ph.w) * Ph.w);
                WorstLee = std::max(WorstLee, std::fabs(G / KOf(K0, double(VoxelOceanCoastHeldDepth(float(Dm)))) - 1.0));
                if (bHave)
                {
                    const double StepM = 0.00005 * std::sin(CapAngle) * RadiusM;
                    WorstStep = std::max(WorstStep, std::fabs(Ph.x - Prev) / (1.5 * std::max(G, PrevG) * StepM + 1.0e-3));
                }
                Prev = Ph.x; PrevG = G; bHave = true;
            }
        }
        std::printf("  the channel between the island and the coast: the crests' spacing within %.1f%% of the local wavelength\n", WorstLee * 100.0);
        Check(WorstStep < 1.0, "across the channel where the rays meet, no step in the phase", WorstStep, 1.0);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n6. The budget, the reach, the cover past the outer edge\n");
    {
        // A budget too small for every candidate even at the coarsest cells: the farthest left out.
        VoxelOceanCoastCore::FSettings Small = Settings;
        Small.BudgetBytes = 1.0 * 1024.0 * 1024.0;
        const VoxelOceanCoastCore::FPlan PS = VoxelOceanCoastCore::Plan(N, RadiusM, Shore.data(), ReachM, Height.data(), Small);
        std::printf("  1 MB: %d tiles, %d left out, cells %.0f m, %.2f MB, reach %.1f km\n", int(PS.Tiles.size()), PS.TrimmedTiles, PS.CellM, PS.AtlasBytes / 1048576.0,
            PS.Settings.ReachM * 1e-3);
        Check(PS.bValid && PS.AtlasBytes <= Small.BudgetBytes && PS.TrimmedTiles > 0 && PS.Settings.ReachM < Settings.ReachM && PS.Settings.ReachM >= 0.0,
            "a small budget holds: the farthest tiles left out, the reach brought in", PS.AtlasBytes / 1048576.0, 1.0);
        // A reach past the shore field's own: held inside it.
        VoxelOceanCoastCore::FSettings Far = Settings;
        Far.ReachM = 2.0 * ReachM;
        const VoxelOceanCoastCore::FPlan PF = VoxelOceanCoastCore::Plan(N, RadiusM, Shore.data(), ReachM, Height.data(), Far);
        std::printf("  reach asked %.0f km (the shore field's %.0f km): held at %.1f km, %d tiles (%d at 8 km)\n", Far.ReachM * 1e-3, ReachM * 1e-3, PF.Settings.ReachM * 1e-3,
            int(PF.Tiles.size()), int(P.Tiles.size()));
        Check(PF.bValid && PF.Settings.ReachM < ReachM, "a reach past the shore field's is held inside it", PF.Settings.ReachM, ReachM);
        // Past the outer edge, well inside the deep water, the field isn't trusted (the cover fades).
        double DeepCover = 0.0;
        int DeepPoints = 0;
        for (int b = 0; b < 24; b++)
        {
            const double Theta = 0.6 + 2.0 * 3.14159265358979 * b / 24.0;
            for (double Dist = 500.0; Dist < 8000.0; Dist += 250.0)
            {
                const FDir D = Seaward(Theta, Dist);
                if (-HeightAt(D) < Settings.SeedDepthM + 20.0) continue;
                DeepCover = std::max(DeepCover, double(SampleAt(D).Cover));
                DeepPoints++;
            }
        }
        Check(DeepPoints > 50 && DeepCover < 0.05, "20 m past the seed depth the cover is gone", DeepCover, 0.05);
    }

    std::printf("\n7. The cover at a tile's edge toward a cell without one (stage W5k): it falls to nothing\n"
                "   inside the tile (the shore's trains stopped at the cell's edge, a step of about a half)\n");
    {
        double WorstStep = 0.0, AtEdge = 0.0;
        int Edges = 0;
        for (int t = 0; t < P.InteriorTiles && Edges < 400; t++)
        {
            const VoxelOceanCoastCore::FTile& T = P.Tiles[size_t(t)];
            if (T.I < 3 || T.J < 3 || T.I >= N - 3 || T.J >= N - 3) continue;
            static const int Dir4[4][2] = { { 1, 0 }, { 0, 1 }, { -1, 0 }, { 0, -1 } };
            for (int e = 0; e < 4; e++)
            {
                const int I1 = T.I + Dir4[e][0], J1 = T.J + Dir4[e][1];
                if (P.TileOf[size_t(VoxelOceanCoastCore::IndexTexel(N, T.Face, I1, J1))] >= 0) continue;
                // From the tile's centre to the neighbour's, along fine cells.
                double Prev = -1.0;
                const double R = double(P.R);
                for (int k = 0; k <= 400; k++)
                {
                    const double F = double(k) / 400.0;
                    const double FU = (T.I + 0.5 + Dir4[e][0] * F) * R - 0.5, FV = (T.J + 0.5 + Dir4[e][1] * F) * R - 0.5;
                    const double S = -1.0 + (2.0 * FU + 1.0) / (double(N) * R), Tt = -1.0 + (2.0 * FV + 1.0) / (double(N) * R);
                    // The face's point at (s, t) (equal-angle): the cube coordinates by tan.
                    double D[3];
                    VoxelOceanCoastCore::DirOfTan(T.Face, std::tan(0.25 * 3.14159265358979 * S), std::tan(0.25 * 3.14159265358979 * Tt), D);
                    const double C = SampleAt({ D[0], D[1], D[2] }).Cover;
                    if (Prev >= 0.0) WorstStep = std::max(WorstStep, std::abs(C - Prev));
                    if (k == 200) AtEdge = std::max(AtEdge, C);
                    Prev = C;
                }
                Edges++;
            }
        }
        // (Its slope per fine cell: the trust a bit per cell, Catmull-Rom between, so the cover
        // falls over a fine cell or two; before W5k it fell by about a half at the edge itself.)
        const double Slope = WorstStep * 400.0 / double(P.R);
        std::printf("  %d edges: the cover at the edge at most %.3f, its steepest fall %.2f a fine cell (%.0f m)\n", Edges, AtEdge, Slope, P.CellM);
        Check(Edges > 10 && AtEdge < 0.02, "nothing at a tile's edge toward a cell without one", AtEdge, 0.02);
        Check(Slope < 1.5, "falling over a fine cell or more, no step", Slope, 1.5);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n8. The window round the camera (stage W7a): its values the whole coast's, in a fraction of the time\n");
    {
        VoxelOceanCoastCore::FSettings W = Settings;
        W.WindowRadiusM = 16000.0;
        W.WindowFadeM = 8000.0;
        W.WindowMoveM = 4000.0;
        // (COAST_HALO=<m> tries another halo: on this planet's straight shelves even 2 km holds.)
        W.WindowHaloM = std::getenv("COAST_HALO") ? std::atof(std::getenv("COAST_HALO")) : 0.0;
        W.WindowCellM = P.CellM;
        W.WindowMaxCells = 1.0e9;
        const double Shown = W.WindowRadiusM + W.WindowFadeM + 1.25 * W.WindowMoveM;
        std::printf("  cover to %.0f km, tiles within %.0f km (halo %.0f km), cells %.1f m\n", (W.WindowRadiusM + W.WindowFadeM) * 1e-3, W.SolveRadiusM() * 1e-3,
            W.HaloM() * 1e-3, P.CellM);
        double WorstPlan = 0.0;
        int KeptSame = 0;
        const VoxelOceanCoastCore::FCoastCells Kept = VoxelOceanCoastCore::CoastCells(N, RadiusM, Shore.data(), ReachM, Height.data(), W);
        double WorstA = 0.0, WorstB = 0.0, WorstBelow = 0.0, WorstPhase = 0.0, WorstSampleA = 0.0, WorstSampleCover = 0.0, WorstTime = 0.0;
        size_t Compared = 0, MostCells = 0;
        int Windows = 0, BoxesOk = 0, SlicesOk = 0;
        const double RootK6 = std::sqrt(std::pow(2.0 * 3.14159265358979 / 6.0, 2.0) / 9.81);
        int FirstSlices = -1;
        for (int k = 0; k < 8; k++)
        {
            // On the coast, and 6 km out (some of them by the island, some across face edges).
            const double Theta = 0.37 + 0.785 * k;
            const FDir C = Seaward(Theta, (k % 2) ? 6000.0 : 0.0);
            VoxelOceanCoastCore::FWindow Win;
            Win.bOn = true;
            Win.Centre[0] = C.X; Win.Centre[1] = C.Y; Win.Centre[2] = C.Z;
            // (Stage W7b: from the coast's cells as the builder keeps them; the same plan as made afresh.)
            const auto TPlan = std::chrono::steady_clock::now();
            const VoxelOceanCoastCore::FPlan PW = VoxelOceanCoastCore::Plan(N, RadiusM, Shore.data(), ReachM, Height.data(), W, Win, &Kept);
            WorstPlan = std::max(WorstPlan, std::chrono::duration<double>(std::chrono::steady_clock::now() - TPlan).count());
            {
                const VoxelOceanCoastCore::FPlan Fresh = VoxelOceanCoastCore::Plan(N, RadiusM, Shore.data(), ReachM, Height.data(), W, Win);
                bool bSame = Fresh.Tiles.size() == PW.Tiles.size() && Fresh.InteriorTiles == PW.InteriorTiles && Fresh.Cut == PW.Cut && Fresh.TileOf == PW.TileOf;
                for (size_t t = 0; bSame && t < PW.Tiles.size(); t++)
                    bSame = Fresh.Tiles[t].Face == PW.Tiles[t].Face && Fresh.Tiles[t].I == PW.Tiles[t].I && Fresh.Tiles[t].J == PW.Tiles[t].J;
                if (bSame) KeptSame++;
            }
            if (!PW.bValid) { std::printf("  window %d: no plan\n", k); Fails++; continue; }
            Windows++;
            if (FirstSlices < 0) FirstSlices = PW.Slices();
            if (PW.Slices() == FirstSlices && PW.R == P.R) SlicesOk++;
            std::vector<float> DW(PW.InteriorCells());
            for (size_t c = 0; c < DW.size(); c++)
            {
                double D[3];
                VoxelOceanCoastCore::InteriorCellDir(PW, c, D);
                DW[c] = float(-HeightAt({ D[0], D[1], D[2] }));
            }
            const VoxelOceanCoastCore::FSolved VW = VoxelOceanCoastCore::Solve(PW, DW.data(), Shore.data(), ReachM);
            MostCells = std::max(MostCells, DW.size());
            WorstTime = std::max(WorstTime, VW.Seconds);
            // Every cell the cover can show (within the cover's reach of this centre or of a last
            // one within a move of it), against the whole coast's solve.
            for (size_t c = 0; c < DW.size(); c++)
            {
                const VoxelOceanCoastCore::FCellId Id = VoxelOceanCoastCore::CellIdOf(PW, int64_t(c));
                double D[3];
                VoxelOceanCoastCore::FineDir(PW, Id.Face, Id.FI, Id.FJ, D);
                const double Along = RadiusM * std::acos(std::clamp(D[0] * C.X + D[1] * C.Y + D[2] * C.Z, -1.0, 1.0));
                if (Along > Shown) continue;
                const int64_t g = VoxelOceanCoastCore::InteriorCellAt(P, Id.Face, Id.FI, Id.FJ);
                // (The water, and the beach up to 3 m where the swash runs: deep inland the rays
                // cross land at the floor's slowness from wherever, and nothing shows them.)
                if (g < 0 || !V.Trust[size_t(g)] || !(DW[c] > -3.0f)) continue;
                const double DA = double(VW.A[c]) - double(V.A[size_t(g)]);
                const double DB = double(VW.B[c]) - double(V.B[size_t(g)]);
                WorstA = std::max(WorstA, std::abs(DA));
                WorstB = std::max(WorstB, std::abs(DB));
                WorstBelow = std::max(WorstBelow, -DA);
                WorstPhase = std::max(WorstPhase, RootK6 * std::abs(DA));
                Compared++;
            }
            // The index's boxes hold every tile.
            int Boxes[6][4];
            VoxelOceanCoastCore::IndexBoxes(PW, Boxes);
            bool bBoxes = true;
            for (const VoxelOceanCoastCore::FTile& T : PW.Tiles)
            {
                const int* B = Boxes[T.Face];
                if (T.I + Pad < B[0] || T.I + Pad > B[2] || T.J + Pad < B[1] || T.J + Pad > B[3]) bBoxes = false;
            }
            if (bBoxes) BoxesOk++;
            // The texels, sampled within the full cover, against the whole coast's.
            const std::vector<uint32_t> CW = VoxelOceanCoastCore::Pack(PW, VW);
            float WR0[4], WR1[4];
            VoxelOceanCoastCore::LayoutRows(PW, WR0, WR1);
            for (int j = 0; j < 400; j++)
            {
                const double Bearing = 0.0157 * j * 37.0, Dist = W.WindowRadiusM * std::sqrt((j % 97) / 97.0);
                FDir T1 = Norm(Cross(C, { 0.3, 0.4, 0.86 }));
                FDir T2 = Cross(C, T1);
                const double A = Dist / RadiusM;
                const FDir Pt = Norm(Add(Scale(C, std::cos(A)), Scale(Add(Scale(T1, std::cos(Bearing)), Scale(T2, std::sin(Bearing))), std::sin(A))));
                const FVoxelOceanCoastSample G = SampleAt(Pt);
                const FVoxelOceanCoastSample L = VoxelOceanCoastCore::Sample(Shore.data(), CW.data(), PW.Slices(), N, WR0, WR1, RadiusM, Pt.X, Pt.Y, Pt.Z);
                if ((G.Cover <= 0.0f && L.Cover <= 0.0f) || HeightAt(Pt) > 3.0) continue;
                WorstSampleCover = std::max(WorstSampleCover, double(std::abs(G.Cover - L.Cover)));
                if (G.Cover > 0.5f) WorstSampleA = std::max(WorstSampleA, double(std::abs(G.A - L.A)));
            }
        }
        std::printf("  %d windows, %zu cells compared; the largest window %zu cells (the whole coast %zu), solved in %.2f s (the whole coast %.2f s), planned in %.3f s\n",
            Windows, Compared, MostCells, Depth.size(), WorstTime, V.Seconds, WorstPlan);
        std::printf("  the water and the beach: A within %.3g m^1/2 (B %.3g m^3/2; below the whole coast's by %.3g), a 6 s wave's phase within %.3g rad\n", WorstA, WorstB, WorstBelow, WorstPhase);
        Check(Windows == 8 && Compared > 10000, "every window planned, its shown cells compared", double(Compared), 10000.0);
        Check(WorstPhase < 1.0e-3, "the shown cells' phase the whole coast's (a 6 s wave)", WorstPhase, 1.0e-3);
        std::printf("  sampled within the full cover (the water and the beach): A within %.3g m^1/2, the cover within %.3g\n", WorstSampleA, WorstSampleCover);
        Check(RootK6 * WorstSampleA < 1.0e-3 && WorstSampleCover < 1.0e-3, "the texels sample as the whole coast's within the full cover (a 6 s wave's phase)",
            std::max(RootK6 * WorstSampleA, WorstSampleCover), 1.0e-3);
        Check(BoxesOk == Windows, "the index's boxes hold every tile", double(BoxesOk), double(Windows));
        Check(KeptSame == Windows, "a plan from the kept coast cells is the plan made afresh", double(KeptSame), double(Windows));
        Check(SlicesOk == Windows, "every window the same texture shape and cells", double(SlicesOk), double(Windows));
        Check(double(MostCells) < 0.3 * double(Depth.size()), "a window a fraction of the whole coast's cells", double(MostCells) / double(Depth.size()), 0.3);
    }

    std::printf("\n9. The window's cover (VoxelOceanCoastWindowCover): smooth, and a new window starts where the last one stood\n");
    {
        FVoxelOceanCoastLayout L = VoxelOceanCoastLayoutNone();
        L.RadiusM = float(RadiusM);
        L.WindowOn = 1.0f;
        L.Inner = 16000.0f;
        L.Outer = 24000.0f;
        const FDir C0 = Seaward(0.37, 0.0);
        const FDir T1 = Norm(Cross(C0, { 0.3, 0.4, 0.86 }));
        auto At = [&](double Dist) { const double A = Dist / RadiusM; return Norm(Add(Scale(C0, std::cos(A)), Scale(T1, std::sin(A)))); };
        const FDir C1 = At(4000.0);
        auto F3 = [](FDir D) { return float3(float(D.X), float(D.Y), float(D.Z)); };
        // The last window settled (its own centre both), the next one at blend 0 and 1.
        FVoxelOceanCoastLayout Last = L;
        Last.Centre = F3(C0); Last.CentreLast = F3(C0); Last.Blend = 1.0f; Last.Fade = 1.0f;
        FVoxelOceanCoastLayout Next = L;
        Next.Centre = F3(C1); Next.CentreLast = F3(C0); Next.Blend = 0.0f; Next.Fade = 1.0f;
        double WorstSwap = 0.0, WorstStep = 0.0, Inside = 1.0, Outside = 0.0;
        double Prev = -1.0;
        for (int k = 0; k <= 4000; k++)
        {
            const double Dist = -30000.0 + 15.0 * k;
            const FDir Pt = At(Dist);
            const double A = VoxelOceanCoastWindowCover(Last, F3(Pt)), B = VoxelOceanCoastWindowCover(Next, F3(Pt));
            WorstSwap = std::max(WorstSwap, std::abs(A - B));
            if (Prev >= 0.0) WorstStep = std::max(WorstStep, std::abs(A - Prev));
            Prev = A;
            if (std::abs(Dist) < 15900.0) Inside = std::min(Inside, A);
            if (std::abs(Dist) > 24100.0) Outside = std::max(Outside, A);
        }
        std::printf("  inside %.6f, outside %.6f, steepest %.4f per 15 m, the swap's step %.2g\n", Inside, Outside, WorstStep, WorstSwap);
        Check(Inside > 0.9999 && Outside < 1.0e-4, "full within the radius, none past the fade", std::max(1.0 - Inside, Outside), 1.0e-4);
        Check(WorstStep < 0.004, "smooth (8 km of fade: under 0.4% per 15 m)", WorstStep, 0.004);
        Check(WorstSwap < 1.0e-5, "a new window starts with the last one's cover", WorstSwap, 1.0e-5);
    }

    std::printf("\n%s: %d failure(s)\n", Fails ? "FAILED" : "PASSED", Fails);
    return Fails ? 1 : 0;
}
