// shore_check.cpp - Stage O8's shared code offline: the sea-connected flood (VoxelLandSea.h, as
// FVoxelLandBake::BuildSeaConnected runs it), the shore field's build and sampling
// (VoxelOceanShore.ush, VoxelOceanShoreBuild.ush through VoxelOceanShoreCore.h: the same functions
// the GPU passes run) on a synthetic planet, and the shoaling maths (VoxelOceanShoal.ush).
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private shore_check.cpp -o shore_check
//   ./shore_check [N]        (N cells per face edge, default 96)
//
// THE PLANET: heights from a seeded sum of smooth waves on the sphere (about 60% below sea level,
// continents and islands), plus a pit carved into a continent (an inland basin below sea level),
// on a 1,075 km sphere.

#include "VoxelShaderCompat.h"
#include "VoxelOceanShoreCore.h"
#include "VoxelLandSea.h"

namespace VoxelShader
{
#include "VoxelOceanFFTWaves.ush"
#include "VoxelOceanShoal.ush"
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace VoxelShader;

static int Fails = 0;
static void Check(bool bOk, const char* What, double Value, double Limit)
{
    std::printf("  [%s] %-70s %.6g (limit %.6g)\n", bOk ? "ok" : "FAIL", What, Value, Limit);
    if (!bOk) Fails++;
}

// ---------------------------------------------------------------------------------------------
// The synthetic planet
// ---------------------------------------------------------------------------------------------

struct FDir { double X, Y, Z; };
static FDir Norm(FDir D) { const double L = std::sqrt(D.X * D.X + D.Y * D.Y + D.Z * D.Z); return { D.X / L, D.Y / L, D.Z / L }; }
static double Dot(FDir A, FDir B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }

static FDir CellDirD(int F, int I, int J, int N)
{
    double A[9];
    VoxelLandSea::Detail::Face(F, A);
    const double TU = std::tan((-1.0 + (2.0 * I + 1.0) / N) * 0.7853981633974483);
    const double TV = std::tan((-1.0 + (2.0 * J + 1.0) / N) * 0.7853981633974483);
    return Norm({ A[0] + A[3] * TU + A[6] * TV, A[1] + A[4] * TU + A[7] * TV, A[2] + A[5] * TU + A[8] * TV });
}

struct FWave { FDir K; double Phase, Amp; };
static std::vector<FWave> Waves;
static FDir PitCentre;
static double PitRadius = 0.0;

static double HeightAt(FDir D)
{
    double H = -350.0;
    for (const FWave& W : Waves)
        H += W.Amp * std::sin(Dot(W.K, D) + W.Phase);
    if (PitRadius > 0.0)
    {
        const double C = Dot(D, PitCentre);
        const double A = std::acos(std::min(1.0, C));
        if (A < 2.0 * PitRadius) H -= 4000.0 * std::exp(-0.5 * (A / PitRadius) * (A / PitRadius) * 4.0);
    }
    return H;
}

int main(int Argc, char** Argv)
{
    const int N = Argc > 1 ? std::max(16, std::atoi(Argv[1])) : 96;
    const double RadiusM = 1.075e6;
    const int Pad = VoxelOceanShoreCore::Pad;
    const int S = VoxelOceanShoreCore::Side(N);
    const int Count = VoxelOceanShoreCore::NumTexels(N);
    const double CellM = 1.5707963267948966 * RadiusM / N;
    const double ReachM = 40.0 * CellM;
    std::printf("shore_check: N %d (cell %.1f km at a face centre), reach %.0f km\n", N, CellM * 1e-3, ReachM * 1e-3);

    std::mt19937 Rng(12345);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    for (int w = 0; w < 24; w++)
    {
        FDir K = Norm({ U(Rng), U(Rng), U(Rng) });
        const double F = 1.5 + 6.0 * std::abs(U(Rng));
        Waves.push_back({ { K.X * F, K.Y * F, K.Z * F }, 3.14159 * U(Rng), 900.0 / F });
    }

    // Heights (apron cells at their own centres, as the bake).
    std::vector<float> Height(static_cast<size_t>(Count));
    auto Index = [&](int F, int I, int J) { return (F * S + J + Pad) * S + I + Pad; };
    for (int F = 0; F < 6; F++)
        for (int J = -Pad; J < N + Pad; J++)
            for (int I = -Pad; I < N + Pad; I++)
                Height[size_t(Index(F, I, J))] = float(HeightAt(CellDirD(F, I, J, N)));

    // A pit: the land cell farthest from the sea (coarse search), carved below sea level.
    {
        double Best = -1.0;
        for (int F = 0; F < 6; F++)
            for (int J = 0; J < N; J += 2)
                for (int I = 0; I < N; I += 2)
                {
                    const double H = Height[size_t(Index(F, I, J))];
                    if (H > Best) { Best = H; PitCentre = CellDirD(F, I, J, N); }
                }
        PitRadius = 3.0 * CellM / RadiusM;
        for (int F = 0; F < 6; F++)
            for (int J = -Pad; J < N + Pad; J++)
                for (int I = -Pad; I < N + Pad; I++)
                    Height[size_t(Index(F, I, J))] = float(HeightAt(CellDirD(F, I, J, N)));
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n1. The sea-connected flood (VoxelLandSea::Mark)\n");
    std::vector<uint16_t> Attr(size_t(Count), uint16_t(0x0123));
    const auto T0 = std::chrono::steady_clock::now();
    const VoxelLandSea::FStats Stats = VoxelLandSea::Mark(N, Pad, Height.data(), Attr.data(), 0x800, 0.02);
    const double FloodMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T0).count();
    int Below = 0, SeaInterior = 0, Wrong = 0, ApronWrong = 0, OtherBits = 0, PitSea = 0, PitCells = 0;
    for (int F = 0; F < 6; F++)
        for (int J = -Pad; J < N + Pad; J++)
            for (int I = -Pad; I < N + Pad; I++)
            {
                const int Id = Index(F, I, J);
                const bool bSea = (Attr[size_t(Id)] & 0x800) != 0;
                if ((Attr[size_t(Id)] & ~0x800) != 0x0123) OtherBits++;
                const bool bInside = I >= 0 && I < N && J >= 0 && J < N;
                if (bInside)
                {
                    if (Height[size_t(Id)] < 0.0f) Below++;
                    if (bSea) SeaInterior++;
                    if (bSea && !(Height[size_t(Id)] < 0.0f)) Wrong++;
                    const FDir D = CellDirD(F, I, J, N);
                    if (std::acos(std::min(1.0, Dot(D, PitCentre))) < 1.0 * PitRadius && Height[size_t(Id)] < 0.0f)
                    {
                        PitCells++;
                        if (bSea) PitSea++;
                    }
                }
                else
                {
                    int F2, I2, J2;
                    VoxelLandSea::Detail::CellOf(F, I, J, N, F2, I2, J2);
                    if (bSea != ((Attr[size_t(Index(F2, I2, J2))] & 0x800) != 0)) ApronWrong++;
                }
            }
    std::printf("  %d below-sea interior cells, %d marked sea in %d regions; %d inland basins; %.1f ms\n",
        Below, SeaInterior, Stats.SeaRegions, Stats.BasinRegions, FloodMs);
    Check(Stats.SeaCells == SeaInterior, "the stats count the marked interior cells", std::abs(Stats.SeaCells - SeaInterior), 0);
    Check(Wrong == 0, "no cell above sea level marked", Wrong, 0);
    Check(OtherBits == 0, "the other attribute bits untouched", OtherBits, 0);
    Check(ApronWrong == 0, "apron cells carry their interior cell's bit", ApronWrong, 0);
    Check(PitCells > 0 && PitSea == 0, "the pit (an inland basin below sea level) is not sea", PitSea, 0);
    Check(PitCells > 0, "the pit has cells below sea level", PitCells, 1);
    Check(Stats.BasinRegions >= 1, "at least one inland basin found", Stats.BasinRegions, 1);
    Check(double(SeaInterior) > 0.9 * double(Below - PitCells), "the oceans are sea (most below-sea cells)", double(SeaInterior) / std::max(1, Below), 0.9);

    // Flood again: idempotent.
    {
        std::vector<uint16_t> Again = Attr;
        VoxelLandSea::Mark(N, Pad, Height.data(), Again.data(), 0x800, 0.02);
        Check(Again == Attr, "marking again changes nothing", Again == Attr ? 0 : 1, 0);
    }

    // Stage W5l: a bay cut off from the sea by one cell of land (a mouth narrower than a cell) is
    // sea by the bridge, not an inland basin (the ocean drained it into a wall of water).
    {
        std::vector<float> Bay = Height;
        int Found = -1, FF = 0, FI = 0, FJ = 0;
        for (int F = 0; F < 6 && Found < 0; F++)
            for (int J = 8; J < N - 8 && Found < 0; J++)
                for (int I = 8; I < N - 8 && Found < 0; I++)
                {
                    // A sea cell with four land cells east of it, two rows of them.
                    bool bOk = Height[size_t(Index(F, I, J))] < -20.0f && (Attr[size_t(Index(F, I, J))] & 0x800) != 0;
                    for (int k = 1; k <= 4 && bOk; k++)
                        bOk = Height[size_t(Index(F, I + k, J))] > 1.0f && Height[size_t(Index(F, I + k, J + 1))] > 1.0f;
                    if (bOk) { Found = 1; FF = F; FI = I; FJ = J; }
                }
        if (Found > 0)
        {
            // The barrier one cell, the bay behind it two by two.
            Bay[size_t(Index(FF, FI + 1, FJ))] = 300.0f;
            Bay[size_t(Index(FF, FI + 1, FJ + 1))] = 300.0f;
            for (int k = 2; k <= 3; k++)
            {
                Bay[size_t(Index(FF, FI + k, FJ))] = -50.0f;
                Bay[size_t(Index(FF, FI + k, FJ + 1))] = -50.0f;
            }
            std::vector<uint16_t> A0(size_t(Count), 0), A3(size_t(Count), 0);
            VoxelLandSea::Mark(N, Pad, Bay.data(), A0.data(), 0x800, 0.02, 0);
            VoxelLandSea::Mark(N, Pad, Bay.data(), A3.data(), 0x800, 0.02);
            const bool bCutOff = (A0[size_t(Index(FF, FI + 2, FJ))] & 0x800) == 0;
            const bool bJoined = (A3[size_t(Index(FF, FI + 2, FJ))] & 0x800) != 0;
            std::printf("  a bay behind one cell of land: %s without the bridge, %s with it (its sea cell %s)\n", bCutOff ? "a basin" : "sea", bJoined ? "sea" : "a basin",
                (A3[size_t(Index(FF, FI, FJ))] & 0x800) != 0 ? "sea" : "not sea");
            Check(bCutOff && bJoined, "a bay cut off by a mouth narrower than a cell is sea (the bridge)", bJoined ? 0 : 1, 0);
        }
        else
        {
            Check(false, "found a coast for the bay test", 0, 1);
        }
    }

    // Stage W5m: the spill and the lagoons. A lagoon behind a berm wider than the bridge: sea when the
    // berm is low (the sea spills over: its level the sea's within what the cells resolve), a basin
    // behind a high one; a coastal lake at the sea's level (the bake's kept lake, its cells sampled
    // above sea level) is sea with the lakes' levels.
    {
        int Found = -1, FF = 0, FI = 0, FJ = 0;
        for (int F = 0; F < 6 && Found < 0; F++)
            for (int J = 8; J < N - 8 && Found < 0; J++)
                for (int I = 8; I < N - 16 && Found < 0; I++)
                {
                    bool bOk = Height[size_t(Index(F, I, J))] < -20.0f && (Attr[size_t(Index(F, I, J))] & 0x800) != 0;
                    for (int k = 1; k <= 9 && bOk; k++)
                        for (int r = 0; r <= 1 && bOk; r++)
                            bOk = Height[size_t(Index(F, I + k, J + r))] > 1.0f;
                    if (bOk) { Found = 1; FF = F; FI = I; FJ = J; }
                }
        if (Found > 0)
        {
            // A berm of five cells (past the bridge's three) east of the sea cell, the lagoon behind
            // it three cells long; the rows either side stay land.
            auto Lagoon = [&](float Berm, float Bed) {
                std::vector<float> H = Height;
                // (Walled round: the rows either side and the cell past it high, so only the berm joins it.)
                for (int k = 1; k <= 9; k++) { H[size_t(Index(FF, FI + k, FJ - 1))] = 50.0f; H[size_t(Index(FF, FI + k, FJ + 2))] = 50.0f; }
                for (int r = 0; r <= 1; r++) H[size_t(Index(FF, FI + 9, FJ + r))] = 50.0f;
                for (int r = 0; r <= 1; r++)
                {
                    for (int k = 1; k <= 5; k++) H[size_t(Index(FF, FI + k, FJ + r))] = Berm;
                    for (int k = 6; k <= 8; k++) H[size_t(Index(FF, FI + k, FJ + r))] = Bed;
                }
                return H;
            };
            auto SeaAt = [&](const std::vector<float>& H, const float* Water, int I) {
                std::vector<uint16_t> A(size_t(Count), 0);
                VoxelLandSea::Mark(N, Pad, H.data(), A.data(), 0x800, 0.02, 3, Water);
                return (A[size_t(Index(FF, I, FJ))] & 0x800) != 0;
            };
            const std::vector<float> Low = Lagoon(2.0f, -4.0f), High = Lagoon(20.0f, -4.0f);
            const bool bLow = SeaAt(Low, nullptr, FI + 7), bHigh = SeaAt(High, nullptr, FI + 7);
            std::printf("  a lagoon behind five cells of berm: %s at 2 m, %s at 20 m\n", bLow ? "sea" : "a basin", bHigh ? "sea" : "a basin");
            Check(bLow && !bHigh, "a lagoon behind a low berm is sea (the spill), behind a high one a basin", bLow && !bHigh ? 0 : 1, 0);
            // The coastal lake: its cells sampled at 1 m, the bake's lake level 1.5 m, behind the low berm.
            const std::vector<float> Lake = Lagoon(2.0f, 1.0f);
            std::vector<float> Water(size_t(Count), -3.0e38f);
            for (int r = 0; r <= 1; r++)
                for (int k = 6; k <= 8; k++) Water[size_t(Index(FF, FI + k, FJ + r))] = 1.5f;
            const bool bWithout = SeaAt(Lake, nullptr, FI + 7), bWith = SeaAt(Lake, Water.data(), FI + 7);
            std::vector<float> WaterHigh = Water;
            for (float& W : WaterHigh) if (W > -1.0e30f) W = 25.0f;
            const bool bHighLake = SeaAt(Lake, WaterHigh.data(), FI + 7);
            std::printf("  a coastal lake at 1.5 m (its cells at 1 m): %s without the lakes' levels, %s with them; a lake at 25 m: %s\n",
                bWithout ? "sea" : "land", bWith ? "sea" : "land", bHighLake ? "sea" : "land");
            Check(!bWithout && bWith && !bHighLake, "a lake at the sea's level next to it is sea (a lagoon), a higher lake not", (!bWithout && bWith && !bHighLake) ? 0 : 1, 0);
        }
        else
        {
            Check(false, "found a coast for the lagoon test", 0, 1);
        }
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n2. The field's build (the shared passes, on the CPU)\n");
    const auto T1 = std::chrono::steady_clock::now();
    const std::vector<uint32_t> Texels = VoxelOceanShoreCore::Build(N, RadiusM, ReachM, Height.data(), Attr.data(),
        [](int Num, auto&& Fn) { for (int i = 0; i < Num; i++) Fn(i); });
    const double BuildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T1).count();
    const std::vector<int> Steps = VoxelOceanShoreCore::Steps(N, RadiusM, ReachM);
    std::printf("  %d texels, %zu jump passes (first step %d), %.0f ms single-threaded\n", Count, Steps.size(), Steps[0], BuildMs);

    // Every texel is a normal finite float.
    int Abnormal = 0;
    for (uint32_t Bits : Texels)
    {
        const float V = asfloat(Bits);
        if (!std::isnormal(V)) Abnormal++;
    }
    Check(Abnormal == 0, "every texel's bits are a normal finite float", Abnormal, 0);

    // The coast points, unquantized, for the exact distances.
    std::vector<FDir> Coast;
    for (int F = 0; F < 6; F++)
        for (int J = 0; J < N; J++)
            for (int I = 0; I < N; I++)
            {
                const int Id = Index(F, I, J);
                const bool bSea = (Attr[size_t(Id)] & 0x800) != 0;
                for (int E = 0; E < 4; E++)
                {
                    const int DI = E == 0 ? 1 : (E == 1 ? -1 : 0), DJ = E == 2 ? 1 : (E == 3 ? -1 : 0);
                    const int Other = Index(F, I + DI, J + DJ);
                    if (((Attr[size_t(Other)] & 0x800) != 0) == bSea) continue;
                    const double H = Height[size_t(Id)], HN = Height[size_t(Other)];
                    const double T = ((H < 0.0) == (HN < 0.0)) ? 0.5 : std::clamp(H / (H - HN), 0.0, 1.0);
                    const FDir A = CellDirD(F, I, J, N), B = CellDirD(F, I + DI, J + DJ, N);
                    Coast.push_back(Norm({ A.X + (B.X - A.X) * T, A.Y + (B.Y - A.Y) * T, A.Z + (B.Z - A.Z) * T }));
                }
            }
    std::printf("  %zu coast crossings\n", Coast.size());

    // Every k-th texel against the exact nearest coast point.
    double MaxErr = 0.0, SumErr = 0.0, MaxSignErr = 0.0;
    int Checked = 0, Far = 0, Over = 0, SignWrong = 0, NoCoastNear = 0, NearChecked = 0;
    double NearMax = 0.0;
    const double Quantum = CellM / 32.0;   // the seed's fraction step, about
    for (int Id = 0; Id < Count; Id += 7)
    {
        const int F = Id / (S * S), Rest = Id % (S * S), J = Rest / S - Pad, I = Rest % S - Pad;
        const FDir P = CellDirD(F, I, J, N);
        double Best = 1e30;
        for (const FDir& C : Coast)
        {
            const double DX = P.X - C.X, DY = P.Y - C.Y, DZ = P.Z - C.Z;
            Best = std::min(Best, DX * DX + DY * DY + DZ * DZ);
        }
        const double Exact = 2.0 * std::asin(0.5 * std::sqrt(Best)) * RadiusM;
        const FVoxelOceanShoreTexel T = VoxelOceanShoreUnpack(Texels[size_t(Id)], float(ReachM));
        const bool bSea = (Attr[size_t(Id)] & 0x800) != 0;
        if ((T.Distance > 0.0f) != bSea && std::abs(T.Distance) > 0.0f) SignWrong++;
        if (Exact > 0.9 * ReachM)
        {
            Far++;
            if (T.Coast > 0.5f && std::abs(T.Distance) < 0.8 * ReachM) NoCoastNear++;
            continue;
        }
        const double Err = std::abs(std::abs(T.Distance) - Exact);
        // Within 2.5 cells the build blends to the heights' own zero level (the linearised
        // terrain, not the crossings' polyline): checked on its own below.
        if (Exact < 2.5 * CellM)
        {
            NearMax = std::max(NearMax, Err / CellM);
            NearChecked++;
            continue;
        }
        if (Err > 2.0 * Quantum) Over++;
        MaxErr = std::max(MaxErr, Err);
        SumErr += Err;
        Checked++;
        MaxSignErr = std::max(MaxSignErr, Err / CellM);
    }
    std::printf("  %d texels from 2.5 cells to the reach checked (%d far), mean error %.2f m, max %.1f m (%.3f cells); %d nearer, up to %.3f cells off the crossings\n",
        Checked, Far, SumErr / std::max(1, Checked), MaxErr, MaxErr / CellM, NearChecked, NearMax);
    Check(NearMax < 0.5, "near the coast (the heights' zero level) within half a cell of the crossings' polyline", NearMax, 0.5);
    Check(SignWrong == 0, "the distance's sign is the cell's flag", SignWrong, 0);
    // A cell keeps one crossing (its nearest), and so does its neighbour across it: a crossing
    // both sides keep another one for is lost (concave corners), a fraction of a cell away.
    Check(double(Over) / std::max(1, Checked) < 0.015, "share of texels beyond 2 seed steps of the exact distance", double(Over) / std::max(1, Checked), 0.015);
    Check(MaxErr < 0.35 * CellM, "the largest distance error (m; JFA's rare misses)", MaxErr, 0.35 * CellM);
    Check(NoCoastNear == 0, "texels beyond the reach don't claim a near coast", NoCoastNear, 0);

    // Directions toward the coast: along minus the distance's gradient on the sea side.
    {
        int Tested = 0, Bad = 0;
        double SumCos = 0.0;
        for (int F = 0; F < 6; F++)
            for (int J = 1; J < N - 1; J += 3)
                for (int I = 1; I < N - 1; I += 3)
                {
                    const FVoxelOceanShoreTexel T = VoxelOceanShoreUnpack(Texels[size_t(Index(F, I, J))], float(ReachM));
                    if (T.Coast < 0.5f || T.Distance < 3.0 * CellM || T.Distance > 0.7 * ReachM) continue;
                    const double DX = VoxelOceanShoreUnpack(Texels[size_t(Index(F, I + 1, J))], float(ReachM)).Distance
                        - VoxelOceanShoreUnpack(Texels[size_t(Index(F, I - 1, J))], float(ReachM)).Distance;
                    const double DY = VoxelOceanShoreUnpack(Texels[size_t(Index(F, I, J + 1))], float(ReachM)).Distance
                        - VoxelOceanShoreUnpack(Texels[size_t(Index(F, I, J - 1))], float(ReachM)).Distance;
                    const double L = std::sqrt(DX * DX + DY * DY);
                    if (L < 1e-6) continue;
                    // Grid axes are close to the face's U, V near the face centre; elsewhere skewed: loose test.
                    const double C = -(DX * T.DirU + DY * T.DirV) / L;
                    SumCos += C;
                    Tested++;
                    if (C < 0.5) Bad++;
                }
        std::printf("  directions: %d texels, mean cosine with the distance's descent %.3f\n", Tested, SumCos / std::max(1, Tested));
        Check(double(Bad) / std::max(1, Tested) < 0.03, "share of directions more than 60 degrees off the descent", double(Bad) / std::max(1, Tested), 0.03);
    }

    // Slopes: decoded against the seed's own cells.
    {
        double MaxRel = 0.0, SmoothedRatio = 1.0;
        int Tested = 0, DistanceChanged = 0;
        // The seeds again (the build's last jump), for the exact slopes.
        std::vector<uint32_t> A(static_cast<size_t>(Count)), B(static_cast<size_t>(Count));
        FVoxelOceanShoreBuildInputs In;
        In.Height = Height.data();
        In.Attr = Attr.data();
        for (int i = 0; i < Count; i++) A[size_t(i)] = VoxelOceanShoreSeedPass(In, i, N);
        for (int K : Steps)
        {
            In.Seeds = A.data();
            for (int i = 0; i < Count; i++) B[size_t(i)] = VoxelOceanShoreJumpPass(In, i, N, K);
            std::swap(A, B);
        }
        In.Seeds = A.data();
        for (int Id = 0; Id < Count; Id += 13)
        {
            if (A[size_t(Id)] == VOXEL_OCEAN_SHORE_NONE) continue;
            const FVoxelOceanShoreSeedParts Pt = VoxelOceanShoreSeedPartsOf(A[size_t(Id)], N);
            const int2 St = VoxelOceanShoreEdgeStep(Pt.Edge);
            const double H0 = Height[size_t(Index(Pt.Face, Pt.I, Pt.J))], H1 = Height[size_t(Index(Pt.Face, Pt.I + St.x, Pt.J + St.y))];
            const FDir P0 = CellDirD(Pt.Face, Pt.I, Pt.J, N), P1 = CellDirD(Pt.Face, Pt.I + St.x, Pt.J + St.y, N);
            const double Apart = std::acos(std::min(1.0, Dot(P0, P1))) * RadiusM;
            const double Exact = std::abs(H1 - H0) / Apart;
            // The pack's own texel (before the smoothing) where it is the seed's alone (2.5 cells
            // out and more): the slope's code.
            const uint32_t Packed = VoxelOceanShorePackPass(In, Id, N, float(RadiusM), float(ReachM));
            const FVoxelOceanShoreTexel T = VoxelOceanShoreUnpack(Packed, float(ReachM));
            const FVoxelOceanShoreTexel Final = VoxelOceanShoreUnpack(Texels[size_t(Id)], float(ReachM));
            // The smoothing keeps the distance's bits.
            if ((Packed & 0xffffu) != (Texels[size_t(Id)] & 0xffffu)) DistanceChanged++;
            if (Final.Coast > 0.5f) SmoothedRatio = std::max(SmoothedRatio, double(std::max(Final.Slope / T.Slope, T.Slope / Final.Slope)));
            if (Exact < 0.0025 || Exact > 0.9 || std::abs(T.Distance) < 2.5 * CellM) continue;
            MaxRel = std::max(MaxRel, std::abs(T.Slope - Exact) / Exact);
            Tested++;
        }
        std::printf("  slopes: %d texels; the smoothing moves a slope by a factor up to %.2f\n", Tested, SmoothedRatio);
        Check(MaxRel < 0.06, "slopes decoded within the code's step (6 bits of log2 over 9 octaves)", MaxRel, 0.06);
        Check(DistanceChanged == 0, "the smoothing passes keep every distance", DistanceChanged, 0);
    }

    // The packing round trip.
    {
        double MaxD = 0.0;
        std::mt19937 R2(7);
        for (int i = 0; i < 100000; i++)
        {
            const float D = float(ReachM * (2.0 * std::uniform_real_distribution<double>(0, 1)(R2) - 1.0));
            const float Ang = float(6.2831853 * std::uniform_real_distribution<double>(0, 1)(R2));
            const uint32_t Bits = VoxelOceanShorePack(D, float(ReachM), std::cos(Ang), std::sin(Ang), 0.03f, true);
            const FVoxelOceanShoreTexel T = VoxelOceanShoreUnpack(Bits, float(ReachM));
            const double Step = 2.0 * ReachM * std::sqrt(std::abs(D) / ReachM) / 32767.0 + 1e-6;
            MaxD = std::max(MaxD, std::abs(T.Distance - D) / Step);
        }
        Check(MaxD <= 0.75, "packed distances within half a code step (in steps)", MaxD, 0.75);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n3. Sampling (VoxelOceanShoreFieldAt): continuity across face edges and corners\n");
    {
        auto SampleAt = [&](FDir D) { return VoxelOceanShoreCore::Sample(Texels.data(), N, ReachM, D.X, D.Y, D.Z); };
        // Pairs of points a tiny step apart: across face edges, and the same inside faces.
        const double Eps = 1e-6;
        double MaxEdgeJump = 0.0, MaxInsideJump = 0.0, MaxEdgeDir = 0.0;
        std::mt19937 R3(99);
        for (int t = 0; t < 20000; t++)
        {
            // A point on an edge: s = 1 on a random face.
            const int F = int(R3() % 6);
            const double T = 2.0 * std::uniform_real_distribution<double>(0, 1)(R3) - 1.0;
            double A[9];
            VoxelLandSea::Detail::Face(F, A);
            const double TV = std::tan(T * 0.7853981633974483);
            const FDir OnEdge = Norm({ A[0] + A[3] + A[6] * TV, A[1] + A[4] + A[7] * TV, A[2] + A[5] + A[8] * TV });
            const FDir Across = Norm({ A[3] - A[0], A[4] - A[1], A[5] - A[2] });   // tangent across the edge (roughly)
            const FDir P0 = Norm({ OnEdge.X - Across.X * Eps, OnEdge.Y - Across.Y * Eps, OnEdge.Z - Across.Z * Eps });
            const FDir P1 = Norm({ OnEdge.X + Across.X * Eps, OnEdge.Y + Across.Y * Eps, OnEdge.Z + Across.Z * Eps });
            const FVoxelOceanShoreSample S0 = SampleAt(P0), S1 = SampleAt(P1);
            MaxEdgeJump = std::max(MaxEdgeJump, double(std::abs(S0.Distance - S1.Distance)));
            const float3 DD = S0.Direction - S1.Direction;
            MaxEdgeDir = std::max(MaxEdgeDir, double(length(DD)));
            // Inside a face.
            const double TI = 2.0 * std::uniform_real_distribution<double>(0, 1)(R3) - 1.0;
            const FDir In0 = CellDirD(F, int((T * 0.5 + 0.5) * (N - 1)), int((TI * 0.5 + 0.5) * (N - 1)), N);
            const FDir In1 = Norm({ In0.X + Across.X * 2 * Eps, In0.Y + Across.Y * 2 * Eps, In0.Z + Across.Z * 2 * Eps });
            MaxInsideJump = std::max(MaxInsideJump, double(std::abs(SampleAt(In0).Distance - SampleAt(In1).Distance)));
        }
        // Cube corners: around each corner, 8 points a tiny step away.
        double MaxCorner = 0.0;
        for (int C = 0; C < 8; C++)
        {
            const FDir Corner = Norm({ (C & 1) ? 1.0 : -1.0, (C & 2) ? 1.0 : -1.0, (C & 4) ? 1.0 : -1.0 });
            float Lo = 1e30f, Hi = -1e30f;
            for (int k = 0; k < 16; k++)
            {
                const double Ang = 6.2831853 * k / 16.0;
                FDir E1 = Norm({ -Corner.Y, Corner.X, 0.0 });
                FDir E2 = { Corner.Y * E1.Z - Corner.Z * E1.Y, Corner.Z * E1.X - Corner.X * E1.Z, Corner.X * E1.Y - Corner.Y * E1.X };
                const FDir P = Norm({ Corner.X + (E1.X * std::cos(Ang) + E2.X * std::sin(Ang)) * Eps, Corner.Y + (E1.Y * std::cos(Ang) + E2.Y * std::sin(Ang)) * Eps,
                    Corner.Z + (E1.Z * std::cos(Ang) + E2.Z * std::sin(Ang)) * Eps });
                const float D = SampleAt(P).Distance;
                Lo = std::min(Lo, D);
                Hi = std::max(Hi, D);
            }
            MaxCorner = std::max(MaxCorner, double(Hi - Lo));
        }
        std::printf("  a 1e-6 step: across edges %.3f m (directions %.4f), inside faces %.3f m, around corners %.3f m\n",
            MaxEdgeJump, MaxEdgeDir, MaxInsideJump, MaxCorner);
        Check(MaxEdgeJump < 3.0, "no jump across face edges (m, over a 1 m step)", MaxEdgeJump, 3.0);
        Check(MaxEdgeDir < 0.01, "no direction jump across face edges", MaxEdgeDir, 0.01);
        Check(MaxCorner < 3.0, "no jump around cube corners (m)", MaxCorner, 3.0);

        // The sampled distance against the exact one, at random points (bilinear between texels).
        double MaxS = 0.0, SumS = 0.0;
        int NS = 0;
        for (int t = 0; t < 3000; t++)
        {
            const FDir P = Norm({ U(Rng), U(Rng), U(Rng) });
            double Best = 1e30;
            for (const FDir& C : Coast)
            {
                const double DX = P.X - C.X, DY = P.Y - C.Y, DZ = P.Z - C.Z;
                Best = std::min(Best, DX * DX + DY * DY + DZ * DZ);
            }
            const double Exact = 2.0 * std::asin(0.5 * std::sqrt(Best)) * RadiusM;
            if (Exact > 0.8 * ReachM || Exact < 2.0 * CellM) continue;
            const double Err = std::abs(std::abs(SampleAt(P).Distance) - Exact);
            MaxS = std::max(MaxS, Err);
            SumS += Err;
            NS++;
        }
        std::printf("  sampled against exact (2 cells to the reach): mean %.1f m, max %.1f m (%.2f cells)\n", SumS / std::max(1, NS), MaxS, MaxS / CellM);
        Check(MaxS < 0.5 * CellM, "sampled distances within half a cell of the exact ones", MaxS, 0.5 * CellM);

        // Directions: unit away from the midlines, tangent.
        int Tangent = 0, Long = 0, Tested = 0;
        for (int t = 0; t < 3000; t++)
        {
            const FDir P = Norm({ U(Rng), U(Rng), U(Rng) });
            const FVoxelOceanShoreSample Sm = SampleAt(P);
            if (std::abs(Sm.Distance) > 0.8 * ReachM) continue;
            Tested++;
            const double Rad = Sm.Direction.x * P.X + Sm.Direction.y * P.Y + Sm.Direction.z * P.Z;
            if (std::abs(Rad) > 1e-4) Tangent++;
            if (length(Sm.Direction) > 1.0001f) Long++;
        }
        Check(Tangent == 0, "sampled directions tangent", Tangent, 0);
        Check(Long == 0, "sampled directions at most unit length", Long, 0);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n3b. The coastline (stage O9c: VoxelOceanShoreCore::Coastline, the mesh's split bias)\n");
    {
        const std::vector<double> Segs = VoxelOceanShoreCore::Coastline(Texels.data(), N, ReachM);
        const size_t NumSegs = Segs.size() / 6;
        auto SampleAt = [&](FDir D) { return VoxelOceanShoreCore::Sample(Texels.data(), N, ReachM, D.X, D.Y, D.Z); };
        // The ends lie on the field's zero line (its bilinear reads along the texels' edges); within
        // two cells of a face's edge the faces' own fields differ by part of a cell.
        auto NearEdge = [&](const double* P)
        {
            const FVoxelOceanFacePoint FP = VoxelOceanLocate(float3(float(P[0]), float(P[1]), float(P[2])));
            return std::max(std::abs(FP.S), std::abs(FP.T)) > 1.0f - 4.0f / float(N);
        };
        double WorstEnd = 0.0, WorstEdgeEnd = 0.0;
        for (size_t i = 0; i < NumSegs; i++)
            for (int e = 0; e < 2; e++)
            {
                const double* P = &Segs[i * 6 + e * 3];
                const double D = std::abs(double(SampleAt({ P[0], P[1], P[2] }).Distance));
                if (NearEdge(P)) WorstEdgeEnd = std::max(WorstEdgeEnd, D);
                else WorstEnd = std::max(WorstEnd, D);
            }
        std::printf("  %zu segments; their ends within %.1f m (%.4f cells) of the field's zero line, %.3f cells by a face's edge\n", NumSegs, WorstEnd,
            WorstEnd / CellM, WorstEdgeEnd / CellM);
        Check(NumSegs > 100, "a coastline found", double(NumSegs), 100.0);
        Check(WorstEnd < 0.01 * CellM, "the segments' ends on the field's zero line (0.01 cell)", WorstEnd / CellM, 0.01);
        Check(WorstEdgeEnd < 0.5 * CellM, "by a face's edge, within the faces' own difference (0.5 cell)", WorstEdgeEnd / CellM, 0.5);
        // Every point of the zero line lies near a segment: points where the field is near zero,
        // found along random great-circle walks.
        std::mt19937 R4(7);
        std::normal_distribution<double> G4;
        double WorstGap = 0.0, WorstEdgeGap = 0.0;
        int Found = 0;
        for (int w = 0; w < 200 && Found < 600; w++)
        {
            FDir A = Norm({ G4(R4), G4(R4), G4(R4) }), B = Norm({ G4(R4), G4(R4), G4(R4) });
            double Last = SampleAt(A).Distance;
            auto At = [&](double T) { return Norm({ A.X + (B.X - A.X) * T, A.Y + (B.Y - A.Y) * T, A.Z + (B.Z - A.Z) * T }); };
            for (int k = 1; k <= 4000; k++)
            {
                const double T = k / 4000.0;
                const double D = SampleAt(At(T)).Distance;
                if ((D < 0.0) != (Last < 0.0) && std::abs(D) < 0.05 * CellM)
                {
                    // Onto the zero line: bisection between the two samples.
                    double Lo = (k - 1) / 4000.0, Hi = T;
                    const bool bLoSea = Last >= 0.0;
                    for (int b = 0; b < 30; b++)
                    {
                        const double Mid = 0.5 * (Lo + Hi);
                        if ((SampleAt(At(Mid)).Distance >= 0.0) == bLoSea) Lo = Mid; else Hi = Mid;
                    }
                    const FDir P = At(0.5 * (Lo + Hi));
                    double Best = 1e30;
                    for (size_t i = 0; i < NumSegs; i++)
                    {
                        const double* S0 = &Segs[i * 6];
                        const double E[3] = { S0[3] - S0[0], S0[4] - S0[1], S0[5] - S0[2] };
                        const double Wv[3] = { P.X - S0[0], P.Y - S0[1], P.Z - S0[2] };
                        const double L2 = E[0] * E[0] + E[1] * E[1] + E[2] * E[2];
                        const double U = L2 > 0.0 ? std::clamp((Wv[0] * E[0] + Wv[1] * E[1] + Wv[2] * E[2]) / L2, 0.0, 1.0) : 0.0;
                        const double DX = Wv[0] - E[0] * U, DY = Wv[1] - E[1] * U, DZ = Wv[2] - E[2] * U;
                        Best = std::min(Best, DX * DX + DY * DY + DZ * DZ);
                    }
                    const double Pd[3] = { P.X, P.Y, P.Z };
                    double& Worst = NearEdge(Pd) ? WorstEdgeGap : WorstGap;
                    Worst = std::max(Worst, std::sqrt(Best) * RadiusM);
                    Found++;
                }
                Last = D;
            }
        }
        std::printf("  %d points of the zero line: the nearest segment within %.1f m (%.4f cells), %.3f cells by a face's edge\n", Found, WorstGap,
            WorstGap / CellM, WorstEdgeGap / CellM);
        Check(Found > 50 && WorstGap < 0.1 * CellM, "the zero line covered by the segments (0.1 cell)", WorstGap / CellM, 0.1);
        Check(WorstEdgeGap < 0.5 * CellM, "by a face's edge, within the faces' own difference (0.5 cell)", WorstEdgeGap / CellM, 0.5);
    }

    // -----------------------------------------------------------------------------------------
    std::printf("\n4. Shoaling maths (VoxelOceanShoal.ush)\n");
    {
        // The dispersion against bisection in double.
        double MaxRel = 0.0;
        for (double Y = 1e-6; Y < 6.0; Y *= 1.01)
        {
            double Lo = 1e-12, Hi = Y + 20.0;
            for (int it = 0; it < 200; it++) { const double M = 0.5 * (Lo + Hi); (M * std::tanh(M) > Y ? Hi : Lo) = M; }
            const double X = 0.5 * (Lo + Hi);
            const FVoxelOceanDispersion D = VoxelOceanDispersionOf(float(Y));
            MaxRel = std::max(MaxRel, std::abs(D.X - X) / X);
        }
        Check(MaxRel < 2e-4, "k_d d within 2e-4 of the exact dispersion (float)", MaxRel, 2e-4);
        const FVoxelOceanDispersion Deep = VoxelOceanDispersionOf(10.0f);
        Check(Deep.R == 1.0f && Deep.N == 0.5f, "deep water: r 1, n 0.5", std::abs(Deep.R - 1.0) + std::abs(Deep.N - 0.5), 0);
        const FVoxelOceanDispersion Near = VoxelOceanDispersionOf(5.99f);
        Check(std::abs(Near.R - 1.0f) < 1e-4 && std::abs(Near.N - 0.5f) < 1e-4, "continuous into deep water at k d 6", std::abs(Near.R - 1.0) + std::abs(Near.N - 0.5), 1e-4);
        const FVoxelOceanDispersion Shallow = VoxelOceanDispersionOf(0.001f);
        Check(std::abs(Shallow.R * std::sqrt(0.001) - 1.0) < 0.01 && std::abs(Shallow.N - 1.0f) < 0.01, "shallow water: r 1 / sqrt(k d), n 1", std::abs(Shallow.R * std::sqrt(0.001) - 1.0), 0.01);

        // The warp integral G against numerical integration of r - 1.
        auto RExact = [](double Y)
            {
                double Lo = 1e-14, Hi = Y + 20.0;
                for (int it = 0; it < 200; it++) { const double M = 0.5 * (Lo + Hi); (M * std::tanh(M) > Y ? Hi : Lo) = M; }
                return 0.5 * (Lo + Hi) / Y;
            };
        double MaxG = 0.0;
        for (double Y0 : { 0.0617, 0.08, 0.1, 0.2, 0.35, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 4.5 })
        {
            // Simpson on [Y0, 12] in log-spaced pieces.
            double G = 0.0;
            const int M = 20000;
            const double A = std::log(Y0), B = std::log(12.0);
            for (int k = 0; k < M; k++)
            {
                const double U0 = A + (B - A) * k / M, U1 = A + (B - A) * (k + 1) / M, Um = 0.5 * (U0 + U1);
                auto F = [&](double Uv) { const double Y = std::exp(Uv); return (RExact(Y) - 1.0) * Y; };
                G += (U1 - U0) / 6.0 * (F(U0) + 4.0 * F(Um) + F(U1));
            }
            MaxG = std::max(MaxG, std::abs(VoxelOceanWarpOf(float(Y0)).G - G));
        }
        Check(MaxG < 2e-4, "G(y) against numerical integration of r - 1", MaxG, 2e-4);
        Check(std::abs(VoxelOceanWarpOf(1e-9f).G - (VoxelOceanWarpOf(0.0617f).G + 3.0f * 0.0617f)) < 1e-4, "G continues linearly below the stretch cap", 0, 1e-4);
        // dG/dy = -(r - 1): the stretch is the warp's own derivative (a lookup's local wavenumber).
        double MaxDer = 0.0;
        for (double Y = 0.01; Y < 5.5; Y *= 1.15)
        {
            const double H = 1e-3 * std::max(Y, 0.05);
            const double Der = -(VoxelOceanWarpOf(float(Y + H)).G - VoxelOceanWarpOf(float(Y - H)).G) / (2.0 * H);
            MaxDer = std::max(MaxDer, std::abs(Der - VoxelOceanWarpOf(float(Y)).Stretch));
        }
        Check(MaxDer < 3e-3, "the stretch is minus G's derivative (finite differences)", MaxDer, 3e-3);
        Check(VoxelOceanWarpOf(6.0f).G == 0.0f && VoxelOceanWarpOf(6.0f).Stretch == 0.0f, "no warp in deep water", 0, 0);

        // The bands at a point.
        FVoxelOceanSeaState Sea;
        Sea.Wind = 10.0f; Sea.Swell = 0.0f; Sea.Long = VoxelOceanStateWeights(10.0f); Sea.Short2 = 1.0f; Sea.Short3 = 1.0f;
        const float4 Tiles(997.0f, 241.0f, 59.0f, 13.7f);
        const float4 Sigma(0.4f, 0.12f, 0.03f, 0.005f);
        FVoxelOceanShoalSettings St;
        St.LongKeep = 1.0f;
        St.Floor = 0.0f;
        St.Breaking = 0.78f; St.ChopGrowth = 1.0f; St.Warp = 1.0f; St.Fade = 0.0f; St.MinSlope = 0.005f;
        FVoxelOceanShoreSample Shore;
        Shore.Distance = 1.0e6f; Shore.Slope = 0.02f; Shore.Direction = float3(1, 0, 0); Shore.Weight = 1.0f;
        const FVoxelOceanShoal DeepSea = VoxelOceanShoalOf(4000.0f, Shore, true, Sea, Tiles, Sigma, St);
        Check(DeepSea.Active == 0.0f, "deep water far from the coast: nothing to do", DeepSea.Active, 0);
        const FVoxelOceanShoal Unknown = VoxelOceanShoalOf(1.0e30f, Shore, false, Sea, Tiles, Sigma, St);
        Check(Unknown.Active == 0.0f, "unknown depth and no field: nothing to do", Unknown.Active, 0);
        FVoxelOceanShoalSettings Off = St;
        Off.Breaking = 0.0f;
        Check(VoxelOceanShoalOf(1.0f, Shore, true, Sea, Tiles, Sigma, Off).Active == 0.0f, "breaking index 0: shoaling off", 0, 0);

        // Toward the beach along a uniform slope: amplitude falls to 0 at the shoreline, the waves
        // never exceed the breaking cap, wind sea (TMA) shrinks, swell first grows.
        bool bMonoTma = true, bCap = true, bZero = true;
        float PrevA = 2.0f, MaxSwellGain = 0.0f;
        FVoxelOceanSeaState SwellSea = Sea;
        SwellSea.Wind = 2.0f; SwellSea.Swell = 12.0f; SwellSea.Long = VoxelOceanStateWeights(12.0f); SwellSea.Short2 = 0.1f; SwellSea.Short3 = 0.1f;
        for (float D = 60.0f; D > 0.001f; D *= 0.97f)
        {
            Shore.Distance = D / 0.02f;
            const FVoxelOceanShoal W = VoxelOceanShoalOf(D, Shore, true, Sea, Tiles, Sigma, St);
            const float A0 = W.Amp.x;
            if (A0 > PrevA + 1e-5f) bMonoTma = false;
            PrevA = A0;
            float V = 0.0f;
            for (int B = 0; B < 4; B++) { const float A = VoxelOceanShoalComponent(W.Amp, B) * VoxelOceanShoalComponent(Sigma, B); V += A * A; }
            if (4.0f * std::sqrt(V) > 0.78f * D * 1.0001f) bCap = false;
            const FVoxelOceanShoal Sw = VoxelOceanShoalOf(D, Shore, true, SwellSea, Tiles, Sigma, St);
            MaxSwellGain = std::max(MaxSwellGain, Sw.Amp.x);
        }
        const FVoxelOceanShoal AtShore = VoxelOceanShoalOf(0.0f, Shore, true, Sea, Tiles, Sigma, St);
        bZero = AtShore.Amp.x == 0.0f && AtShore.Amp.w == 0.0f;
        Check(bMonoTma, "wind sea's long band shrinks toward the beach (TMA, breaking)", bMonoTma ? 0 : 1, 0);
        Check(bCap, "the significant height stays under 0.78 d", bCap ? 0 : 1, 0);
        Check(bZero, "no waves at the shoreline (d 0)", bZero ? 0 : 1, 0);
        Check(MaxSwellGain > 1.05f && MaxSwellGain <= VOXEL_OCEAN_SHOAL_REACH_HEIGHT, "swell grows before it breaks, within the reach's allowance", MaxSwellGain, VOXEL_OCEAN_SHOAL_REACH_HEIGHT);
        // Horizontal growth.
        const FVoxelOceanShoal Mid = VoxelOceanShoalOf(3.0f, Shore, true, Sea, Tiles, Sigma, St);
        Check(Mid.Chop.x > 1.5f && Mid.Chop.x <= VOXEL_OCEAN_SHOAL_MAX_CHOP, "orbits flatten in shallow water (chop x 1 / tanh k_d d)", Mid.Chop.x, VOXEL_OCEAN_SHOAL_MAX_CHOP);
        // Continuity in depth.
        double MaxJump = 0.0;
        for (float D = 0.05f; D < 300.0f; D *= 1.001f)
        {
            Shore.Distance = 5000.0f;
            const FVoxelOceanShoal A = VoxelOceanShoalOf(D, Shore, true, Sea, Tiles, Sigma, St);
            const FVoxelOceanShoal B = VoxelOceanShoalOf(D * 1.001f, Shore, true, Sea, Tiles, Sigma, St);
            for (int k = 0; k < 4; k++)
            {
                MaxJump = std::max(MaxJump, double(std::abs(VoxelOceanShoalComponent(A.Amp, k) - VoxelOceanShoalComponent(B.Amp, k))));
                MaxJump = std::max(MaxJump, double(std::abs(VoxelOceanShoalComponent(A.Chop, k) - VoxelOceanShoalComponent(B.Chop, k))));
            }
        }
        Check(MaxJump < 0.01, "factors continuous in depth (0.1% steps)", MaxJump, 0.01);
        // The fade.
        FVoxelOceanShoalSettings Faded = St;
        Faded.Fade = 100.0f;
        Shore.Distance = 50.0f;
        const FVoxelOceanShoal Fd = VoxelOceanShoalOf(1.0f, Shore, true, Sea, Tiles, Sigma, Faded);   // d / slope = 50 m
        const FVoxelOceanShoal Nf = VoxelOceanShoalOf(1.0f, Shore, true, Sea, Tiles, Sigma, St);
        Check(std::abs(Fd.Amp.y / Nf.Amp.y - 0.5f) < 0.01f, "the fade at half its distance: one half", Fd.Amp.y / Nf.Amp.y, 0.5);
        // Stage W3g: with the fade on, the factors stay continuous through the depth where the
        // longest band starts to shoal (the fade used to switch on there all at once).
        {
            FVoxelOceanShoalSettings Far = St;
            Far.Fade = 1.0e6f;
            double MaxFadeJump = 0.0;
            for (float D = 1.0f; D < 3000.0f; D *= 1.001f)
            {
                Shore.Distance = 5000.0f;
                const FVoxelOceanShoal A = VoxelOceanShoalOf(D, Shore, true, Sea, Tiles, Sigma, Far);
                const FVoxelOceanShoal B = VoxelOceanShoalOf(D * 1.001f, Shore, true, Sea, Tiles, Sigma, Far);
                for (int k = 0; k < 4; k++)
                    MaxFadeJump = std::max(MaxFadeJump, double(std::abs(VoxelOceanShoalComponent(A.Amp, k) - VoxelOceanShoalComponent(B.Amp, k))));
            }
            Check(MaxFadeJump < 0.01, "with the fade on, continuous where the longest band starts to shoal (0.1% steps)", MaxFadeJump, 0.01);
        }
        // Stage W3g: the floor. No band under Floor of its height, on land too; continuous through
        // the shoreline.
        {
            FVoxelOceanShoalSettings Fl = St;
            Fl.Floor = 0.15f;
            Shore.Distance = 50.0f;
            float Least = 1.0f;
            double MaxFloorJump = 0.0;
            for (float D = -5.0f; D < 50.0f; D += 0.005f)
            {
                const FVoxelOceanShoal A = VoxelOceanShoalOf(D, Shore, true, Sea, Tiles, Sigma, Fl);
                const FVoxelOceanShoal B = VoxelOceanShoalOf(D + 0.005f, Shore, true, Sea, Tiles, Sigma, Fl);
                for (int k = 0; k < 4; k++)
                {
                    Least = std::min(Least, VoxelOceanShoalComponent(A.Amp, k));
                    MaxFloorJump = std::max(MaxFloorJump, double(std::abs(VoxelOceanShoalComponent(A.Amp, k) - VoxelOceanShoalComponent(B.Amp, k))));
                }
            }
            Check(std::abs(Least - 0.15f) < 1e-4f, "the floor: no band under 0.15 of its height, on land either", Least, 0.15);
            Check(MaxFloorJump < 0.02, "with the floor, continuous through the shoreline (5 mm steps)", MaxFloorJump, 0.02);
        }

        // The warp: U(s) = G(k m s) / (m k); its derivative along s is minus the stretch.
        double MaxW = 0.0;
        for (float Sd = 5.0f; Sd < 3000.0f; Sd *= 1.2f)
        {
            Shore.Distance = Sd;
            const FVoxelOceanShoal A = VoxelOceanShoalOf(1.0e30f, Shore, true, Sea, Tiles, Sigma, St);
            Shore.Distance = Sd * 1.001f;
            const FVoxelOceanShoal B = VoxelOceanShoalOf(1.0e30f, Shore, true, Sea, Tiles, Sigma, St);
            for (int k = 0; k < 4; k++)
            {
                const double Der = -(VoxelOceanShoalComponent(B.Shift, k) - VoxelOceanShoalComponent(A.Shift, k)) / (0.001 * Sd);
                MaxW = std::max(MaxW, std::abs(Der - VoxelOceanShoalComponent(A.Stretch, k)) / (1.0 + VoxelOceanShoalComponent(A.Stretch, k)));
            }
        }
        Check(MaxW < 5e-3, "a band's shift falls along s at its stretch (the lookup's wavenumber)", MaxW, 5e-3);

        // The sample transform against a warped plane wave's finite differences.
        {
            // f(x) = a sin(k . (x + U(x . n) n)) with U' = s (a constant stretch over the step).
            const float Kx = 0.3f, Ky = 0.2f, Am = 1.3f, Stretch = 0.7f;
            const float2 E(0.6f, 0.8f);
            auto Warped = [&](double X, double Y)
                {
                    const double Along = X * E.x + Y * E.y;
                    const double Xw = X + Stretch * Along * E.x, Yw = Y + Stretch * Along * E.y;
                    return Am * std::sin(Kx * Xw + Ky * Yw);
                };
            const double X0 = 1.7, Y0 = -0.4, H = 1e-4;
            const double Gx = (Warped(X0 + H, Y0) - Warped(X0 - H, Y0)) / (2 * H), Gy = (Warped(X0, Y0 + H) - Warped(X0, Y0 - H)) / (2 * H);
            FVoxelOceanFFTSample Raw = VoxelOceanFFTSampleZero();
            const double Along = X0 * E.x + Y0 * E.y;
            const double Xw = X0 + Stretch * Along * E.x, Yw = Y0 + Stretch * Along * E.y;
            Raw.Height = float(Am * std::sin(Kx * Xw + Ky * Yw));
            Raw.Slope = float2(float(Am * Kx * std::cos(Kx * Xw + Ky * Yw)), float(Am * Ky * std::cos(Kx * Xw + Ky * Yw)));
            const FVoxelOceanFFTSample T = VoxelOceanShoalSample(Raw, 1.0f, 1.0f, Stretch, E);
            const double Err = std::abs(T.Slope.x - Gx) + std::abs(T.Slope.y - Gy);
            Check(Err < 1e-3, "the shoaled slope is the warped wave's gradient", Err, 1e-3);
        }
    }

    std::printf("\n%s: %d failure(s)\n", Fails ? "FAILED" : "PASSED", Fails);
    return Fails ? 1 : 0;
}
