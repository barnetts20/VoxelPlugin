// The terrain edits in the land surface cache (Design/EditsAndCaves.md, E2): the writer's column
// search (Shaders/Private/VoxelLandCacheEdits.ush, compiled as C++ with the GPU's edit code)
// against a reference in double: the edit store's core (VoxelEditCore.h) applied to the exact
// base density of a rolling land on a sphere, sampled every 2 cm along each texel's column, the
// crossings refined, the normal from the full field's differences.
//
// Edits over a patch of land: craters, a mound, a held-brush trench, a tunnel 6 m down (an
// overhang), a pit dug deeper than the column, a wall, and edits narrower than coarse texels.
// For texels of levels 0, 2, 4 and 7 over them:
//   - the classification: the land's own (no change), one surface, or flagged (several
//     crossings, none, or steeper than 75 degrees) against the reference's;
//   - for one surface: the height's error (cm) and the normal's (degrees);
//   - a column far from every edit costs nothing (the cell bounds' early out).
// Planets of 1,000 and 6,400 km (the double-float path to the lattice).
//
// g++ -O2 -std=c++20 -I<Source/VoxelPlugin/Public> -I<Source/VoxelPlugin/Private> -I<Shaders/Private> cache_edit_harness.cpp
#include "VoxelShaderCompat.h"
#include "VoxelEditCore.h"
#include "VoxelLandCachePlan.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace VoxelShader
{
    std::vector<uint> EditCells;
    std::vector<uint> EditOpList;
    std::vector<uint> EditOps;
    uint EditTableMask = 0;
    uint EditCellShift = 0;
    uint EditOpCount = 0;
    int3 EditCellMin;
    int3 EditCellMax;
    float2 HalfFinest;
    int64_t CellLookups = 0;    // the harness's count (VoxelEditFindCell calls)
#include "VoxelDensityCore.ush"
#include "VoxelDF64.ush"
#define VoxelEditFindCell VoxelEditFindCellCounted
#include "VoxelEditSample.ush"
#undef VoxelEditFindCell
    inline uint VoxelEditFindCell(int X, int Y, int Z) { CellLookups++; return VoxelEditFindCellCounted(X, Y, Z); }
#include "VoxelLandCacheEdits.ush"
}

using namespace VoxelEditCore;

namespace
{
    constexpr double H = 12.5;          // half finest cell (cm)
    constexpr int32_t Shift = 6;        // 8 m index cells

    struct FPlanet
    {
        double S = 0.0;                 // sea-level radius (cm)
        double Site[3] = { 0, 0, 0 };   // the edits' patch (unit)
        double E1[3] = { 0, 0, 0 }, E2[3] = { 0, 0, 0 };

        /** The land (m about sea level) at a unit direction: rolling, slopes up to about 0.3. */
        double HeightM(const double U[3]) const
        {
            const double X = U[0] * S * 0.01, Y = U[1] * S * 0.01, Z = U[2] * S * 0.01;   // metres
            return 40.0 + 12.0 * std::sin(X * 0.004 + 0.3) * std::cos(Y * 0.005) + 1.5 * std::sin(Z * 0.05 + Y * 0.03) + 0.4 * std::sin(X * 0.21 - Z * 0.17);
        }
        /** The base density at P (cm, positive in air): the height above the land. */
        double Base(const double P[3]) const
        {
            const double R = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
            const double U[3] = { P[0] / R, P[1] / R, P[2] / R };
            return R - (S + 100.0 * HeightM(U));
        }
        /** The land's gradient (tangent, m/m) at U: the base density's is Up - this. */
        void Gradient(const double U[3], double Out[3]) const
        {
            const double R = S + 100.0 * HeightM(U);
            const double P[3] = { U[0] * R, U[1] * R, U[2] * R };
            const double E = 1.0;
            double G[3];
            for (int a = 0; a < 3; a++)
            {
                double A[3] = { P[0], P[1], P[2] }, B[3] = { P[0], P[1], P[2] };
                A[a] += E;
                B[a] -= E;
                G[a] = (Base(A) - Base(B)) / (2.0 * E);
            }
            // G = Up - Gradient: the tangent part, negated.
            const double GU = G[0] * U[0] + G[1] * U[1] + G[2] * U[2];
            for (int a = 0; a < 3; a++) Out[a] = -(G[a] - U[a] * GU);
        }
        /** A point on the land at tangent offsets (cm) from the site, and Depth cm below it. */
        void Ground(double X, double Y, double Depth, double Out[3]) const
        {
            double D[3];
            for (int a = 0; a < 3; a++) D[a] = Site[a] * S + E1[a] * X + E2[a] * Y;
            const double L = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
            const double U[3] = { D[0] / L, D[1] / L, D[2] / L };
            const double R = S + 100.0 * HeightM(U) - Depth;
            for (int a = 0; a < 3; a++) Out[a] = U[a] * R;
        }
    };

    struct FEdit { EMode Mode; double X, Y, Depth; double Radius, Blend; bool bContinue; };

    /** The reference: the edited field along the column at Dir (unit) from Lo to Hi m about sea
     *  level. State 0 (unchanged), 1 (one surface), 2 (flagged), the topmost crossing, normal. */
    struct FReference { int State = 0; double Height = 0.0; double Normal[3] = { 0, 0, 0 }; double Facing = 1.0; int Crossings = 0; };

    FReference Reference(const FPlanet& Planet, const FState& Edits, const double Dir[3], double LandM, double Footprint, double Step)
    {
        FReference Out;
        auto Field = [&](const double P[3]) { return Edits.Apply(P, Footprint, Planet.Base(P)); };
        auto At = [&](double M, double P[3]) { for (int a = 0; a < 3; a++) P[a] = Dir[a] * (Planet.S + 100.0 * M); };
        bool bChanged = false, bBelow = false, bBottom = false;
        int Upper = -1;
        const int Count = 2400;   // 2 cm over +-24 m
        for (int k = 0; k <= Count; k++)
        {
            const double M = LandM - 24.0 + 48.0 * k / Count;
            double P[3];
            At(M, P);
            const double B = Planet.Base(P), F = Field(P);
            bChanged = bChanged || std::fabs(F - B) > 1e-3;
            const bool bAir = F > 0.0;
            if (k == 0) bBottom = bAir;
            else if (bAir != bBelow) { Out.Crossings++; if (bAir) Upper = k - 1; }
            bBelow = bAir;
        }
        if (!bChanged) return Out;
        Out.State = 2;
        if (Upper < 0) return Out;
        double Lo = LandM - 24.0 + 48.0 * Upper / Count, Hi = LandM - 24.0 + 48.0 * (Upper + 1) / Count;
        for (int b = 0; b < 40; b++)
        {
            const double Mid = 0.5 * (Lo + Hi);
            double P[3];
            At(Mid, P);
            (Field(P) > 0.0 ? Hi : Lo) = Mid;
        }
        Out.Height = 0.5 * (Lo + Hi);
        double P[3];
        At(Out.Height, P);
        double G[3];
        for (int a = 0; a < 3; a++)
        {
            double A[3] = { P[0], P[1], P[2] }, B[3] = { P[0], P[1], P[2] };
            A[a] += Step;
            B[a] -= Step;
            G[a] = (Field(A) - Field(B)) / (2.0 * Step);
        }
        const double L = std::sqrt(G[0] * G[0] + G[1] * G[1] + G[2] * G[2]);
        for (int a = 0; a < 3; a++) Out.Normal[a] = G[a] / L;
        Out.Facing = Out.Normal[0] * Dir[0] + Out.Normal[1] * Dir[1] + Out.Normal[2] * Dir[2];
        if (Out.Crossings == 1 && !bBottom && Out.Facing >= 0.2588190) Out.State = 1;
        return Out;
    }

    VoxelShader::float2 Split(double V) { const float Hi = (float)V; return VoxelShader::float2(Hi, (float)(V - (double)Hi)); }

    int Run(double RadiusKm)
    {
        FPlanet Planet;
        Planet.S = RadiusKm * 1e5;
        const double Site[3] = { 0.48, -0.62, 0.62 };
        const double L = std::sqrt(Site[0] * Site[0] + Site[1] * Site[1] + Site[2] * Site[2]);
        for (int a = 0; a < 3; a++) Planet.Site[a] = Site[a] / L;
        VoxelLandCachePlan::FLevel Frame;
        VoxelLandCachePlan::Anchor(Frame, Planet.Site);
        for (int a = 0; a < 3; a++) { Planet.E1[a] = Frame.E1[a]; Planet.E2[a] = Frame.E2[a]; }

        // The edits (tangent offsets from the site, cm).
        std::vector<FEdit> List;
        List.push_back({ EMode::Carve, 0.0, 0.0, 0.0, 300.0, 75.0, false });             // crater
        List.push_back({ EMode::Carve, 700.0, 100.0, 50.0, 400.0, 100.0, false });       // crater, overlapping
        List.push_back({ EMode::Fill, -900.0, 300.0, -100.0, 350.0, 90.0, false });      // mound
        for (int i = 0; i <= 60; i++)                                                    // trench, held brush
            List.push_back({ EMode::Carve, -2000.0 + 20.0 * i, -1500.0 + 8.0 * i, 100.0, 250.0, 60.0, i > 0 });
        for (int i = 0; i <= 50; i++)                                                    // tunnel 6 m down: an overhang
            List.push_back({ EMode::Carve, 2500.0 + 20.0 * i, -1200.0, 600.0, 200.0, 50.0, i > 0 });
        for (int i = 0; i < 12; i++)                                                     // pit 30 m deep
            List.push_back({ EMode::Carve, -2500.0, 2000.0, 250.0 * i, 300.0, 75.0, false });
        for (int i = 0; i < 8; i++)                                                      // a wall
            List.push_back({ EMode::Fill, 1800.0, 1500.0 + 100.0 * i, -200.0, 150.0, 0.0, false });
        List.push_back({ EMode::Carve, 3000.0, 2500.0, 0.0, 40.0, 10.0, false });       // narrow: closes at coarse texels

        FState Edits;
        Edits.HalfFinest = H;
        Edits.CellShift = Shift;
        for (const FEdit& E : List)
        {
            double P[3];
            Planet.Ground(E.X, E.Y, E.Depth, P);
            FInsertResult R;
            Edits = Insert(Edits, E.Mode, P, E.Radius, E.Blend, E.bContinue, FInsertOptions(), R);
        }
        FGPUArrays G;
        Flatten(Edits, G);
        VoxelShader::EditCells = G.Cells;
        VoxelShader::EditOpList = G.OpList;
        VoxelShader::EditOps = G.Ops;
        VoxelShader::EditTableMask = G.TableMask;
        VoxelShader::EditCellShift = G.CellShift;
        VoxelShader::EditOpCount = G.OpCount;
        VoxelShader::HalfFinest = Split(H);
        bool bAny = false;
        for (size_t s = 0; s + 3 < G.Cells.size(); s += 4)
        {
            if (G.Cells[s + 3] == 0u) continue;
            const int X = (int)G.Cells[s], Y = (int)G.Cells[s + 1], Z = (int)G.Cells[s + 2];
            if (!bAny) { VoxelShader::EditCellMin = VoxelShader::int3(X, Y, Z); VoxelShader::EditCellMax = VoxelShader::EditCellMin; bAny = true; }
            VoxelShader::EditCellMin = VoxelShader::int3(std::min(VoxelShader::EditCellMin.x, X), std::min(VoxelShader::EditCellMin.y, Y), std::min(VoxelShader::EditCellMin.z, Z));
            VoxelShader::EditCellMax = VoxelShader::int3(std::max(VoxelShader::EditCellMax.x, X), std::max(VoxelShader::EditCellMax.y, Y), std::max(VoxelShader::EditCellMax.z, Z));
        }
        printf("planet %.0f km: %zu edits as %zu operations, %u GPU ops\n", RadiusKm, List.size(), Edits.Ops.size(), G.OpCount);

        int Failures = 0;
        const int Levels[] = { 0, 2, 4, 7 };
        for (int K : Levels)
        {
            const double T = 25.0 * std::ldexp(1.0, K);
            // Texels over the patch (about 70 m square), at most 200 a side.
            const int Side = std::min(200, (int)std::ceil(7000.0 / T));
            int64_t Count[3][3] = {};   // [reference][ours]
            int64_t Matched = 0, Mismatched = 0, Texels = 0;
            double WorstHeight = 0.0, SumHeight = 0.0, WorstAngle = 0.0, SumAngle = 0.0;
            int64_t Surfaces = 0, Near = 0, Same = 0;
            double LandWorstHeight = 0.0, LandWorstAngle = 0.0;
            for (int j = 0; j < Side; j++)
                for (int i = 0; i < Side; i++)
                {
                    // The texel's centre on the sea sphere, as the writer finds it (double here).
                    const int64_t I = (int64_t)std::floor(-3500.0 / T) + i, J = (int64_t)std::floor(-3000.0 / T) + j;
                    const double U = (I + 0.5) * T, V = (J + 0.5) * T;
                    double D[3];
                    for (int r = 0; r < 3; r++) D[r] = Frame.Inverse[r][0] * U + Frame.Inverse[r][1] * V + Frame.Inverse[r][2] * Planet.S;
                    const double DL = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
                    const double Dir[3] = { D[0] / DL, D[1] / DL, D[2] / DL };
                    const double LandM = Planet.HeightM(Dir);
                    double Grad[3];
                    Planet.Gradient(Dir, Grad);
                    const double Step = std::min(0.5 * T, 100.0);
                    const FReference Ref = Reference(Planet, Edits, Dir, LandM, T, Step);
                    const VoxelShader::FVoxelLandCacheEdited Ours = VoxelShader::VoxelLandCacheEditColumn(
                        Split(Dir[0] * Planet.S), Split(Dir[1] * Planet.S), Split(Dir[2] * Planet.S), Split(Planet.S),
                        (float)LandM, VoxelShader::float3((float)Grad[0], (float)Grad[1], (float)Grad[2]), (float)T);
                    Texels++;
                    Count[Ref.State][Ours.State]++;
                    if (Ref.State == (int)Ours.State) Matched++;
                    else do
                    {
                        // At the thresholds (75 degrees, a crossing pair thinner than the
                        // writer's 1 m samples) the two may differ.
                        bool bThreshold = std::fabs(Ref.Facing - 0.2588190) < 0.05 || Ref.Crossings > 1;
                        // Ours keeping the land where the reference finds one surface: the same
                        // when that surface is the land's (an edit's change deep in solid, or
                        // finer than the writer's 1 m samples).
                        if (Ref.State == 1 && Ours.State == 0)
                        {
                            double N[3];
                            for (int a = 0; a < 3; a++) N[a] = Dir[a] - Grad[a];
                            const double NL = std::sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]);
                            const double Angle = std::acos(std::clamp((N[0] * Ref.Normal[0] + N[1] * Ref.Normal[1] + N[2] * Ref.Normal[2]) / NL, -1.0, 1.0)) * 57.29577951308232;
                            const double DH = std::fabs(Ref.Height - LandM) * 100.0;
                            LandWorstHeight = std::max(LandWorstHeight, DH);
                            LandWorstAngle = std::max(LandWorstAngle, Angle);
                            if (DH < 1.0 && Angle < 1.0) { Same++; continue; }
                        }
                        (bThreshold ? Near : Mismatched)++;
                        if (!bThreshold && Mismatched <= 3 && getenv("CACHE_EDIT_DEBUG"))
                            printf("    level %d texel %lld %lld: reference %d (crossings %d, facing %.3f, %.3f m above the land), ours %u\n", K, (long long)I, (long long)J, Ref.State, Ref.Crossings, Ref.Facing, Ref.Height - LandM, Ours.State);
                    } while (false);
                    if (Ref.State == 1 && Ours.State == 1)
                    {
                        const double DH = std::fabs(Ours.Height - Ref.Height) * 100.0;
                        WorstHeight = std::max(WorstHeight, DH);
                        SumHeight += DH;
                        // Our normal from the stored gradient (as the material: Up - Gradient).
                        double N[3];
                        const double GU = Ours.Gradient.x * Dir[0] + Ours.Gradient.y * Dir[1] + Ours.Gradient.z * Dir[2];
                        for (int a = 0; a < 3; a++) N[a] = Dir[a] - (Ours.Gradient[a] - Dir[a] * GU);
                        const double NL = std::sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]);
                        const double Cos = std::clamp((N[0] * Ref.Normal[0] + N[1] * Ref.Normal[1] + N[2] * Ref.Normal[2]) / NL, -1.0, 1.0);
                        const double Angle = std::acos(Cos) * 57.29577951308232;
                        WorstAngle = std::max(WorstAngle, Angle);
                        SumAngle += Angle;
                        Surfaces++;
                    }
                }
            const bool bOk = Mismatched == 0 && WorstHeight < 1.0 && WorstAngle < 3.0;
            Failures += bOk ? 0 : 1;
            printf("%s level %d (%.0f cm texels, %d x %d): land %lld / %lld / %lld, one surface %lld / %lld / %lld, flagged %lld / %lld / %lld (reference's row: ours unchanged / one / flagged);"
                   " %lld differ at a threshold, %lld otherwise\n",
                bOk ? "OK  " : "FAIL", K, T, Side, Side,
                (long long)Count[0][0], (long long)Count[0][1], (long long)Count[0][2], (long long)Count[1][0], (long long)Count[1][1], (long long)Count[1][2],
                (long long)Count[2][0], (long long)Count[2][1], (long long)Count[2][2], (long long)Near, (long long)Mismatched);
            printf("     the land kept where the reference finds a surface: %lld texels, its surface off the land's by at most %.3f cm and %.3f deg\n",
                (long long)Same, LandWorstHeight, LandWorstAngle);
            printf("     one surface: height error worst %.3f cm (mean %.4f), normal worst %.3f deg (mean %.4f) over %lld texels\n", WorstHeight,
                SumHeight / std::max<int64_t>(Surfaces, 1), WorstAngle, SumAngle / std::max<int64_t>(Surfaces, 1), (long long)Surfaces);
        }

        // A column far from every edit: the cell bounds turn it away without a table lookup.
        {
            double P[3];
            Planet.Ground(500000.0, 0.0, 0.0, P);
            const double R = std::sqrt(P[0] * P[0] + P[1] * P[1] + P[2] * P[2]);
            const double Dir[3] = { P[0] / R, P[1] / R, P[2] / R };
            VoxelShader::CellLookups = 0;
            const VoxelShader::FVoxelLandCacheEdited Far = VoxelShader::VoxelLandCacheEditColumn(Split(Dir[0] * Planet.S), Split(Dir[1] * Planet.S), Split(Dir[2] * Planet.S),
                Split(Planet.S), (float)Planet.HeightM(Dir), VoxelShader::float3(0.0f, 0.0f, 0.0f), 25.0f);
            const bool bOk = Far.State == 0u && VoxelShader::CellLookups == 0;
            Failures += bOk ? 0 : 1;
            printf("%s a column 5 km away: state %u, %lld table lookups\n", bOk ? "OK  " : "FAIL", Far.State, (long long)VoxelShader::CellLookups);
        }
        return Failures;
    }
}

int main()
{
    int Failures = 0;
    Failures += Run(1000.0);
    Failures += Run(6400.0);
    return Failures;
}
