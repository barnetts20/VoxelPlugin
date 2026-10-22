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
        Check(Points > 500 && Worst < 0.07, "the crests are spaced at the local wavelength (7%)", Worst, 0.07);
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

    std::printf("\n%s: %d failure(s)\n", Fails ? "FAILED" : "PASSED", Fails);
    return Fails ? 1 : 0;
}
