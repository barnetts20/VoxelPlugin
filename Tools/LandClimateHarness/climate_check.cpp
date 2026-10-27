// climate_check.cpp - Stage T5a: the land climate's shared code (VoxelLandClimate.ush, as the
// terrain material and APlanetActor::GetClimateAt run it) and its map builder
// (VoxelLandClimateGrid.h), outside the engine.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private \
//       -I<plugin>/Shaders/Private climate_check.cpp -o climate_check && ./climate_check
//
// Checks: Ohmura's line, the biotemperature, continuity (no step in any share as the height,
// slope or latitude moves), the slope and facing terms, the map's taps (exact at cell centres,
// continuous across longitude 0, clamped at the poles), the coast distance against great-circle
// distances, the precipitation's calibration (the land's median), and a table of the provisional
// climate on a synthetic planet: the snow line, deserts and plants by latitude.
#include "VoxelShaderCompat.h"
namespace VoxelShader
{
#include "VoxelLandClimate.ush"
}
#include "VoxelLandClimateGrid.h"
#include <cstdio>
#include <cstdlib>
#include <random>

using namespace VoxelShader;

static int Failures = 0;
#define CHECK(Cond, ...) do { if (!(Cond)) { Failures++; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static FVoxelLandClimateLook DefaultLook(float Breakup = 0.0f)
{
    FVoxelLandClimateLook L;
    L.LapseRate = 6.5f;
    L.Season = 0.15f;
    L.AspectWarming = 2.0f;
    L.BreakupAmplitude = Breakup;
    L.BreakupScale = 60.0f;
    L.SnowWidth = 1.5f;
    return L;
}

static FVoxelLandClimateCell Cell(float T, float P, float A, float C = 0.0f)
{
    FVoxelLandClimateCell X;
    X.SeaTemperature = T;
    X.Precipitation = P;
    X.Seasonality = A;
    X.Continentality = C;
    return X;
}

static float3 Dir(double LatDeg, double LonDeg)
{
    const double La = LatDeg * 3.14159265358979 / 180.0, Lo = LonDeg * 3.14159265358979 / 180.0;
    return float3((float)(std::cos(La) * std::cos(Lo)), (float)(std::cos(La) * std::sin(Lo)), (float)std::sin(La));
}

/** The snow line (m): the lowest height where the glacier share reaches a half (flat, no breakup). */
static float SnowLine(const FVoxelLandClimateCell& C, const float3& Up, double RadiusM)
{
    const FVoxelLandClimateLook L = DefaultLook();
    for (float H = 0.0f; H <= 9000.0f; H += 10.0f)
    {
        const float3 P = Up * (float)(RadiusM + H);
        if (VoxelLandClimateAt(C, L, P, Up, Up, H).Glacier >= 0.5f) return H;
    }
    return 9999.0f;
}

int main()
{
    // --- Ohmura's line ---
    CHECK(std::fabs(VoxelLandClimateEquilibriumSummer(645.0f)) < 1e-3f, "Ohmura at 645 mm: %f", VoxelLandClimateEquilibriumSummer(645.0f));
    CHECK(std::fabs(VoxelLandClimateEquilibriumSummer(2500.0f) - 5.38f) < 0.02f, "Ohmura at 2500 mm: %f", VoxelLandClimateEquilibriumSummer(2500.0f));
    {
        float Last = -100.0f;
        for (float P = 0.0f; P <= 8000.0f; P += 25.0f)
        {
            const float T = VoxelLandClimateEquilibriumSummer(P);
            // Check the fit itself where it holds.
            if (P <= 5000.0f) CHECK(std::fabs(645.0f + 296.0f * T + 9.0f * T * T - P) < 0.5f, "Ohmura residual at %f", P);
            CHECK(T >= Last - 1e-5f, "Ohmura not monotonic at %f", P);
            Last = T;
        }
    }
    // --- Biotemperature ---
    CHECK(std::fabs(VoxelLandClimateBiotemperature(15.0f, 5.0f) - 15.0f) < 1e-4f, "bio mid");
    CHECK(VoxelLandClimateBiotemperature(-20.0f, 5.0f) == 0.0f, "bio cold");
    CHECK(std::fabs(VoxelLandClimateBiotemperature(40.0f, 2.0f) - 30.0f) < 1e-4f, "bio hot");

    // --- Continuity: every share over height, slope and latitude, with the breakup on ---
    {
        const double R = 1.0e6;
        const FVoxelLandClimateLook L = DefaultLook(1.5f);
        float MaxStep = 0.0f;
        for (int Lat = -80; Lat <= 80; Lat += 20)
        {
            const float3 Up = Dir(Lat, 30.0);
            const FVoxelLandClimateCell C = Cell(27.0f - 52.0f * (float)std::pow(std::fabs(std::sin(Lat * 3.14159265 / 180.0)), 4.0), 900.0f, 6.0f);
            FVoxelLandClimatePoint Prev = VoxelLandClimateAt(C, L, Up * (float)R, Up, Up, 0.0f);
            for (float H = 1.0f; H <= 8000.0f; H += 1.0f)
            {
                // Along the ground too (a metre a step: the breakup's octaves are 60 and 18.6 m).
                const float3 Side = normalize(cross(Up, float3(0.0f, 0.0f, 1.0f)) + float3(1e-3f, 0.0f, 0.0f));
                const float3 P = Up * (float)(R + H) + Side * H;
                const FVoxelLandClimatePoint Q = VoxelLandClimateAt(C, L, P, Up, Up, H);
                MaxStep = std::max(MaxStep, std::fabs(Q.Snow - Prev.Snow));
                MaxStep = std::max(MaxStep, std::fabs(Q.Glacier - Prev.Glacier));
                MaxStep = std::max(MaxStep, std::fabs(Q.Desert - Prev.Desert));
                MaxStep = std::max(MaxStep, std::fabs(Q.Vegetation - Prev.Vegetation));
                Prev = Q;
            }
        }
        std::printf("continuity: the largest change of a share over a metre (height and ground, breakup on): %.4f\n", MaxStep);
        CHECK(MaxStep < 0.08f, "a share jumps by %f over a metre", MaxStep);
    }
    // Slope: snow off past 55 degrees, full under 40; ice to 50, gone past 65.
    {
        const float3 Up = Dir(45.0, 0.0);
        const float3 East = normalize(cross(float3(0.0f, 0.0f, 1.0f), Up));
        const FVoxelLandClimateCell C = Cell(-20.0f, 1000.0f, 6.0f);
        const FVoxelLandClimateLook L = DefaultLook();
        auto At = [&](float Deg)
        {
            const float A = Deg * 3.14159265f / 180.0f;
            return VoxelLandClimateAt(C, L, Up * 1.0e6f, Up, Up * std::cos(A) + East * std::sin(A), 0.0f);
        };
        CHECK(At(30.0f).Snow > 0.99f && At(56.0f).Snow < 0.01f, "snow slope: %f %f", At(30.0f).Snow, At(56.0f).Snow);
        CHECK(At(45.0f).Glacier > 0.99f && At(66.0f).Glacier < 0.01f, "ice slope: %f %f", At(45.0f).Glacier, At(66.0f).Glacier);
        CHECK(At(60.0f).Glacier > At(60.0f).Snow, "exposed ice on steep glacier faces");
    }
    // Facing: in the north a slope facing south (toward the equator) is warmer; in the south, one facing north.
    {
        const FVoxelLandClimateCell C = Cell(10.0f, 800.0f, 6.0f);
        const FVoxelLandClimateLook L = DefaultLook();
        for (int Hemisphere = -1; Hemisphere <= 1; Hemisphere += 2)
        {
            const float3 Up = Dir(45.0 * Hemisphere, 0.0);
            const float3 North = normalize(float3(0.0f, 0.0f, 1.0f) - Up * Up.z);
            const float A = 30.0f * 3.14159265f / 180.0f;
            const float TNorth = VoxelLandClimateAt(C, L, Up * 1.0e6f, Up, Up * std::cos(A) + North * std::sin(A), 0.0f).Temperature;
            const float TSouth = VoxelLandClimateAt(C, L, Up * 1.0e6f, Up, Up * std::cos(A) - North * std::sin(A), 0.0f).Temperature;
            const float Equatorward = Hemisphere > 0 ? TSouth - TNorth : TNorth - TSouth;
            CHECK(std::fabs(Equatorward - 2.0f) < 0.01f, "facing (hemisphere %d): %f", Hemisphere, Equatorward);
        }
    }

    // --- The taps ---
    {
        const int W = VOXEL_LAND_CLIMATE_WIDTH, H = VOXEL_LAND_CLIMATE_HEIGHT;
        int Bad = 0;
        for (int J = 0; J < H; J += 7)
            for (int I = 0; I < W; I += 13)
            {
                double X, Y, Z;
                VoxelLandClimateGrid::CellDirection(I, J, W, H, X, Y, Z);
                const FVoxelLandClimateTaps T = VoxelLandClimateTapsOf(float3((float)X, (float)Y, (float)Z));
                const bool bX = (T.X0 == I && T.FX < 1e-3f) || (T.X1 == I && T.FX > 1.0f - 1e-3f);
                const bool bY = (T.Y0 == J && T.FY < 1e-3f) || (T.Y1 == J && T.FY > 1.0f - 1e-3f);
                if (!bX || !bY) Bad++;
            }
        CHECK(Bad == 0, "taps off the cell centres: %d", Bad);
        // Across longitude 0: the last column and the first blend.
        const FVoxelLandClimateTaps A = VoxelLandClimateTapsOf(Dir(10.0, 0.0));
        CHECK(A.X0 == W - 1 && A.X1 == 0 && std::fabs(A.FX - 0.5f) < 1e-3f, "seam: %d %d %f", A.X0, A.X1, A.FX);
        const FVoxelLandClimateTaps N = VoxelLandClimateTapsOf(float3(0.0f, 0.0f, 1.0f));
        CHECK(N.Y0 == H - 1 && N.Y1 == H - 1, "north pole: %d %d", N.Y0, N.Y1);
        const FVoxelLandClimateTaps S = VoxelLandClimateTapsOf(float3(0.0f, 0.0f, -1.0f));
        CHECK(S.Y0 == 0 && S.FY == 0.0f, "south pole: %d %f", S.Y0, S.FY);
    }

    // --- A synthetic planet: continents as spherical caps ---
    const int W = VOXEL_LAND_CLIMATE_WIDTH, H = VOXEL_LAND_CLIMATE_HEIGHT;
    const double R = 1.0e6;
    std::vector<float> Land((size_t)(W * H), 0.0f);
    {
        std::mt19937 Rng(7);
        std::uniform_real_distribution<double> U(-1.0, 1.0);
        std::vector<std::array<double, 4>> Caps;
        for (int k = 0; k < 9; k++)
        {
            double x = U(Rng), y = U(Rng), z = U(Rng) * 0.8;
            const double L = std::sqrt(x * x + y * y + z * z);
            Caps.push_back({ x / L, y / L, z / L, 0.35 + 0.25 * (U(Rng) + 1.0) * 0.5 });
        }
        for (int J = 0; J < H; J++)
            for (int I = 0; I < W; I++)
            {
                double X, Y, Z;
                VoxelLandClimateGrid::CellDirection(I, J, W, H, X, Y, Z);
                for (const auto& C : Caps)
                    if (std::acos(std::min(1.0, X * C[0] + Y * C[1] + Z * C[2])) < C[3]) Land[(size_t)(J * W + I)] = 1.0f;
            }
    }
    double LandShare = 0.0;
    for (float L : Land) LandShare += L;
    LandShare /= (double)Land.size();
    std::printf("synthetic planet: land %.0f%%\n", 100.0 * LandShare);

    // --- The coast distance against great-circle distances (64 x 32 subset of cells) ---
    std::vector<float> Coast((size_t)(W * H));
    VoxelLandClimateGrid::CoastDistance(Land.data(), W, H, R, Coast.data());
    {
        std::vector<std::array<double, 3>> SeaDirs;
        for (int J = 0; J < H; J++)
            for (int I = 0; I < W; I++)
                if (Land[(size_t)(J * W + I)] < 0.5f)
                {
                    double X, Y, Z;
                    VoxelLandClimateGrid::CellDirection(I, J, W, H, X, Y, Z);
                    SeaDirs.push_back({ X, Y, Z });
                }
        double WorstLow = 1.0, WorstHigh = 1.0;
        int Tested = 0;
        for (int J = 4; J < H; J += 8)
            for (int I = 3; I < W; I += 8)
            {
                const int C = J * W + I;
                if (Land[(size_t)C] < 0.5f) continue;
                double X, Y, Z;
                VoxelLandClimateGrid::CellDirection(I, J, W, H, X, Y, Z);
                double Best = 1e30;
                for (const auto& S : SeaDirs)
                    Best = std::min(Best, std::acos(std::min(1.0, X * S[0] + Y * S[1] + Z * S[2])) * R);
                if (Best < 50.0e3) continue;   // within a few cells the grid's steps dominate
                const double Ratio = Coast[(size_t)C] / Best;
                WorstLow = std::min(WorstLow, Ratio);
                WorstHigh = std::max(WorstHigh, Ratio);
                Tested++;
            }
        std::printf("coast distance: grid / great circle over %d inland cells: %.3f .. %.3f\n", Tested, WorstLow, WorstHigh);
        CHECK(Tested > 50 && WorstLow > 0.97 && WorstHigh < 1.13, "coast distance off: %f %f", WorstLow, WorstHigh);
    }

    // --- The map: provisional, and a sim's precipitation scaled to the land's median ---
    FVoxelLandClimateGridParams Params;
    Params.RadiusM = R;
    std::vector<float> Cells((size_t)(W * H * 4));
    const bool bSimUsed = VoxelLandClimateGrid::BuildCells(Land.data(), Coast.data(), nullptr, W, H, Params, Cells.data());
    CHECK(!bSimUsed, "provisional reported as the sim's");
    auto LandMedian = [&](const std::vector<float>& Out)
    {
        std::vector<double> V;
        for (int C = 0; C < W * H; C++) if (Land[(size_t)C] >= 0.5f) V.push_back(Out[(size_t)(4 * C + 1)]);
        std::nth_element(V.begin(), V.begin() + (std::ptrdiff_t)(V.size() / 2), V.end());
        return V[V.size() / 2];
    };
    CHECK(std::fabs(LandMedian(Cells) - 650.0) < 6.5, "provisional land median %f", LandMedian(Cells));
    {
        std::vector<float> Sim((size_t)(W * H * 4), 0.0f);
        std::mt19937 Rng(3);
        std::uniform_real_distribution<float> U(0.0f, 0.3f);
        for (int C = 0; C < W * H; C++) { Sim[(size_t)(4 * C)] = U(Rng); Sim[(size_t)(4 * C + 3)] = 0.05f; }
        std::vector<float> Out((size_t)(W * H * 4));
        CHECK(VoxelLandClimateGrid::BuildCells(Land.data(), Coast.data(), Sim.data(), W, H, Params, Out.data()), "sim not used");
        CHECK(std::fabs(LandMedian(Out) - 650.0) < 6.5, "sim land median %f", LandMedian(Out));
        CHECK(std::fabs(Out[0] - (27.0f - 52.0f * (float)std::pow(std::fabs(VoxelLandClimateGrid::RowMu(0, H)), 4.0) + 8.0f * (float)std::tanh(3.0 / 8.0))) < 1e-3f, "sim warmth (60 x 0.05, soft-capped at 8 C)");
        for (float& V : Sim) V = 0.0f;
        CHECK(!VoxelLandClimateGrid::BuildCells(Land.data(), Coast.data(), Sim.data(), W, H, Params, Out.data()), "an all-zero sim must fall back");
    }

    // --- The provisional climate by latitude (land cells): the table ---
    std::printf("\nprovisional climate on the synthetic planet (land; flat ground; Earth-like settings):\n");
    std::printf("  lat | T sea  swing coast/inland | P median mm | snow line m (coast, inland) | desert%% plants (sea level)\n");
    for (int Band = -75; Band <= 75; Band += 15)
    {
        double Sum = 0.0, N = 0.0, Desert = 0.0, Plants = 0.0;
        std::vector<double> P;
        float SwingCoast = 1e9f, SwingInland = -1e9f, LineCoast = 0.0f, LineInland = 0.0f;
        int IC = -1, II = -1;
        float CoastBest = 1e30f, InlandBest = -1.0f;
        for (int J = 0; J < H; J++)
        {
            const double Lat = std::asin(VoxelLandClimateGrid::RowMu(J, H)) * 180.0 / 3.14159265358979;
            if (std::fabs(Lat - Band) > 7.5) continue;
            for (int I = 0; I < W; I++)
            {
                const int C = J * W + I;
                if (Land[(size_t)C] < 0.5f) continue;
                const FVoxelLandClimateCell X = Cell(Cells[(size_t)(4 * C)], Cells[(size_t)(4 * C + 1)], Cells[(size_t)(4 * C + 2)], Cells[(size_t)(4 * C + 3)]);
                double DX, DY, DZ;
                VoxelLandClimateGrid::CellDirection(I, J, W, H, DX, DY, DZ);
                const float3 Up((float)DX, (float)DY, (float)DZ);
                const FVoxelLandClimatePoint Q = VoxelLandClimateAt(X, DefaultLook(), Up * (float)R, Up, Up, 0.0f);
                Sum += X.SeaTemperature; N += 1.0; P.push_back(X.Precipitation);
                Desert += Q.Desert > 0.5f ? 1.0 : 0.0; Plants += Q.Vegetation;
                // The snow line's cells: the coast-most and the most inland near the band's centre.
                if (std::fabs(Lat - Band) < 1.5)
                {
                    if (Coast[(size_t)C] < CoastBest) { CoastBest = Coast[(size_t)C]; IC = C; }
                    if (Coast[(size_t)C] > InlandBest) { InlandBest = Coast[(size_t)C]; II = C; }
                }
            }
        }
        if (N < 1.0 || IC < 0 || II < 0) { std::printf("  %3d | (no land)\n", Band); continue; }
        std::nth_element(P.begin(), P.begin() + (std::ptrdiff_t)(P.size() / 2), P.end());
        auto LineOf = [&](int C, float& Swing)
        {
            const FVoxelLandClimateCell X = Cell(Cells[(size_t)(4 * C)], Cells[(size_t)(4 * C + 1)], Cells[(size_t)(4 * C + 2)], Cells[(size_t)(4 * C + 3)]);
            Swing = X.Seasonality;
            double DX, DY, DZ;
            VoxelLandClimateGrid::CellDirection(C % W, C / W, W, H, DX, DY, DZ);
            return SnowLine(X, float3((float)DX, (float)DY, (float)DZ), R);
        };
        LineCoast = LineOf(IC, SwingCoast);
        LineInland = LineOf(II, SwingInland);
        std::printf("  %3d | %5.1f  %4.1f / %4.1f           | %6.0f      | %5.0f, %5.0f                | %5.0f%%  %.2f\n", Band, Sum / N, SwingCoast, SwingInland,
            P[P.size() / 2], LineCoast, LineInland, 100.0 * Desert / N, Plants / N);
    }

    // T5d2: the sim's warmth without its terrain's part: a high equatorial plateau the sim reads as
    // cold (its air thins over it) keeps no cold anomaly; a real anomaly at sea level survives;
    // the cap holds.
    {
        const int W = 512, H = 256;
        std::vector<float> Sim((size_t)W * H * 4, 0.0f), Height((size_t)W * H, 0.0f), Land2((size_t)W * H, 0.0f);
        std::vector<double> Warm((size_t)W * H);
        for (int J = 0; J < H; J++)
            for (int I = 0; I < W; I++)
            {
                const size_t C = (size_t)J * W + I;
                const bool bPlateau = std::abs(J - H / 2) < 20 && I > 100 && I < 180;
                Height[C] = bPlateau ? 3000.0f : 0.0f;
                // Land: the plateau and a low plain beside it on the row (the trough is sea).
                Land2[C] = bPlateau || (std::abs(J - H / 2) < 20 && I >= 180 && I < 300) ? 1.0f : 0.0f;
                // The sim: -0.25 over the plateau (about -15 C at 60 C a unit), and a real cold
                // trough at sea level elsewhere on the row.
                const bool bTrough = std::abs(J - H / 2) < 20 && I > 300 && I < 340;
                Sim[4 * C + 3] = bPlateau ? -0.25f : (bTrough ? -0.1f : 0.0f);
            }
        FVoxelLandClimateGridParams P;
        VoxelLandClimateGrid::SimWarmth(Sim.data(), Height.data(), Land2.data(), W, H, P, Warm.data());
        const double Plateau = Warm[(size_t)(H / 2) * W + 140];
        const double Trough = Warm[(size_t)(H / 2) * W + 320];
        const double Plain = Warm[(size_t)(H / 2) * W + 450];
        std::printf("sim warmth: plateau %.1f C (raw %.1f), trough %.1f C (raw %.1f), plain %.1f C\n", Plateau, 60.0 * -0.25, Trough, 60.0 * -0.1, Plain);
        CHECK(std::fabs(Plateau - Plain) < 1.5, "the plateau keeps no terrain-made cold (%.1f vs %.1f C)", Plateau, Plain);
        CHECK(Trough < Plain - 3.5, "a sea-level cold trough survives (%.1f vs %.1f C)", Trough, Plain);
        VoxelLandClimateGrid::SimWarmth(Sim.data(), nullptr, nullptr, W, H, P, Warm.data());
        double Lo = 0.0;
        for (double V : Warm) Lo = std::min(Lo, V);
        CHECK(Lo >= -8.0 && Lo < -7.0, "the cap: the raw plateau soft-clamped to 8 C (%.2f)", Lo);
    }

    // T5d2: the ice sheets' thickness (VoxelLandClimateGrid::IceSheet): a polar cap 20 degrees
    // across, on a 1000 km planet: zero outside, Nye's profile in, the cap.
    {
        const int W = 512, H = 256;
        const double R = 1.0e6;
        std::vector<float> Iced((size_t)W * H, 0.0f), Height((size_t)W * H, 500.0f), Out((size_t)W * H * 4);
        for (int J = 0; J < H; J++)
            if (VoxelLandClimateGrid::RowMu(J, H) > std::sin(70.0 * 3.14159265358979 / 180.0))
                for (int I = 0; I < W; I++) Iced[(size_t)J * W + I] = 1.0f;
        VoxelLandClimateGrid::IceSheet(Iced.data(), Height.data(), W, H, R, Out.data());
        // The land chain's thickness from the signed distance (VoxelLandIce.ush).
        auto Thick = [](double D, double Cap) { return std::min(std::sqrt(2.0e5 / (917.0 * 9.81) * std::max(D, 0.0)), Cap); };
        const float Pole = (float)Thick(Out[4 * ((size_t)(H - 1) * W)], 3000.0);
        const float Outside = (float)Thick(Out[4 * ((size_t)(H / 2) * W)], 3000.0);
        // The top row's centre: its arc from the last row not iced, less half a cell.
        int Last = 0;
        for (int J = 0; J < H; J++) if (Iced[(size_t)J * W] < 0.5f) Last = J;
        const double Arc = R * (std::asin(VoxelLandClimateGrid::RowMu(H - 1, H)) - std::asin(VoxelLandClimateGrid::RowMu(Last, H)));
        const double Expected = std::sqrt(2.0e5 / (917.0 * 9.81) * (Arc - 0.5 * R * std::sqrt(4.0 * 3.14159265358979 / (W * H))));
        std::printf("ice sheet: pole %.0f m (Nye %.0f m), outside %.0f m, base %.0f m\n", Pole, Expected, Outside, Out[4 * ((size_t)(H - 1) * W) + 1]);
        CHECK(Outside == 0.0f, "no ice outside the cap");
        CHECK(std::fabs(Pole - Expected) < 0.06 * Expected, "the pole's thickness follows Nye's profile (%.0f vs %.0f)", Pole, Expected);
        CHECK(Thick(Out[4 * ((size_t)(H - 1) * W)], 1000.0) == 1000.0, "the cap");
        // The margin: the signed distance changes sign between the last row out and the first in.
        int First = H - 1;
        for (int J = 0; J < H; J++) if (Iced[(size_t)J * W] >= 0.5f) { First = J; break; }
        CHECK(Out[4 * ((size_t)First * W)] > 0.0f && Out[4 * ((size_t)(First - 1) * W)] < 0.0f, "the signed distance's sign flips at the margin");
    }

    // T5d: the glacier margin (the land chain's glacial shares) against the material's glacier on
    // flat ground without the breakup: the same line.
    {
        double Worst = 0.0;
        const FVoxelLandClimateLook L = DefaultLook();
        for (float T = -30.0f; T <= 30.0f; T += 1.5f)
            for (float P = 50.0f; P <= 3000.0f; P *= 1.7f)
                for (float A = 0.0f; A <= 20.0f; A += 4.0f)
                    for (float Hm = 0.0f; Hm <= 6000.0f; Hm += 500.0f)
                    {
                        const FVoxelLandClimateCell C = Cell(T, P, A);
                        const float3 Up = Dir(45.0, 10.0);
                        const FVoxelLandClimatePoint Pt = VoxelLandClimateAt(C, L, Up * 1.0e6f, Up, Up, Hm);
                        const float M = VoxelLandClimateGlacierMargin(C, L.LapseRate, Hm, 0.0f);
                        const float W = std::max(L.SnowWidth, 0.05f);
                        Worst = std::max(Worst, (double)std::fabs(Pt.Glacier - VoxelLandClimateRamp(-W, W, M)));
                    }
        std::printf("glacier margin vs the material's glacier (flat, no breakup): worst %.2e\n", Worst);
        CHECK(Worst < 1e-4, "the land chain's ice line is the material's");
    }

    // T5c: the sand's drift (VoxelLandClimateGrid::BuildDrift) and the ergs' noise.
    {
        const int W = 512, H = 256;
        std::vector<float> Out((size_t)W * H * 4);
        // The provisional winds: the trades toward the west and the equator, steady; the
        // westerlies toward the east and the pole; none at the belts' edges.
        VoxelLandClimateGrid::BuildDrift(nullptr, W, H, 2, Out.data());
        auto RowAt = [&](double LatDeg) { return std::min(std::max((int)std::floor((std::sin(LatDeg * 3.14159265358979 / 180.0) + 1.0) * 0.5 * H), 0), H - 1); };
        auto At = [&](double LatDeg, int I) { return &Out[4 * ((size_t)RowAt(LatDeg) * W + I)]; };
        const float* T15 = At(15.0, 10);
        const float* T45 = At(45.0, 10);
        const float* S15 = At(-15.0, 10);
        const float* E0 = At(0.0, 10);
        std::printf("drift, provisional: 15N (%.2f, %.2f) %.2f; 45N (%.2f, %.2f) %.2f; 15S (%.2f, %.2f) %.2f; equator steadiness %.3f\n",
            T15[0], T15[1], T15[2], T45[0], T45[1], T45[2], S15[0], S15[1], S15[2], E0[2]);
        CHECK(T15[0] < -0.8f && T15[1] < -0.2f && T15[2] > 0.75f, "the northern trades blow toward the west-south-west, steady");
        CHECK(T45[0] > 0.8f && T45[1] > 0.2f && T45[2] > 0.3f && T45[2] < 0.7f, "the northern westerlies blow toward the east-north-east, variable");
        CHECK(S15[0] < -0.8f && S15[1] > 0.2f, "the southern trades blow toward the west-north-west");
        CHECK(E0[2] < 0.05f, "the doldrums have no resultant");
        float MaxSteady = 0.0f, MinLen = 2.0f, MaxLen = 0.0f;
        for (size_t i = 0; i < (size_t)W * H; i++)
        {
            MaxSteady = std::max(MaxSteady, Out[4 * i + 2]);
            const float L = std::sqrt(Out[4 * i] * Out[4 * i] + Out[4 * i + 1] * Out[4 * i + 1]);
            if (Out[4 * i + 2] > 0.01f) { MinLen = std::min(MinLen, L); MaxLen = std::max(MaxLen, L); }
        }
        CHECK(MaxSteady <= 1.0f && MinLen > 0.999f && MaxLen < 1.001f, "unit directions where there's a resultant (%.4f .. %.4f), steadiness at most 1", MinLen, MaxLen);
        // A sim: one wind toward the east everywhere (steadiness 1); then a field whose direction
        // turns every cell (the blur's resultant cancels: low steadiness); then the wrap at
        // longitude 0 (a wind toward the north in the last column only reaches the first).
        std::vector<float> Sim((size_t)W * H * 4, 0.0f);
        for (size_t i = 0; i < (size_t)W * H; i++) { Sim[4 * i] = 2.0f; Sim[4 * i + 2] = 2.0f; Sim[4 * i + 3] = 1.26f; }
        VoxelLandClimateGrid::BuildDrift(Sim.data(), W, H, 2, Out.data());
        CHECK(std::fabs(Out[0] - 1.0f) < 1e-5f && std::fabs(Out[1]) < 1e-5f && std::fabs(Out[2] - 1.0f) < 1e-5f, "one wind: east, steadiness 1");
        std::mt19937 Rng(3);
        std::uniform_real_distribution<float> Ang(0.0f, 6.2831853f);
        for (size_t i = 0; i < (size_t)W * H; i++) { const float A = Ang(Rng); Sim[4 * i] = std::cos(A); Sim[4 * i + 1] = std::sin(A); Sim[4 * i + 2] = 1.0f; }
        VoxelLandClimateGrid::BuildDrift(Sim.data(), W, H, 2, Out.data());
        double Mean = 0.0;
        for (size_t i = 0; i < (size_t)W * H; i++) Mean += Out[4 * i + 2];
        Mean /= (double)W * H;
        std::printf("drift, random directions a cell: mean steadiness %.3f (a 5 x 5 box of unit vectors: about 0.18)\n", Mean);
        CHECK(Mean < 0.3, "winds that disagree have a low steadiness");
        std::fill(Sim.begin(), Sim.end(), 0.0f);
        for (int J = 0; J < H; J++) { float* C = &Sim[4 * ((size_t)J * W + W - 1)]; C[1] = 1.0f; C[2] = 1.0f; }
        VoxelLandClimateGrid::BuildDrift(Sim.data(), W, H, 2, Out.data());
        CHECK(Out[4 * ((size_t)100 * W + 1) + 1] > 0.99f && Out[4 * ((size_t)100 * W + 3) + 2] == 0.0f, "the filter wraps at longitude 0 (and reaches two cells)");
        // The ergs' noise: within -1..1, and its quantiles spread (the threshold picks a share).
        double Lo = 9.0, Hi = -9.0;
        for (int i = 0; i < 20000; i++)
        {
            const double Z = 1.0 - 2.0 * (i + 0.5) / 20000.0, Rr = std::sqrt(1.0 - Z * Z), A = 2.39996323 * i;
            const float N = VoxelLandClimateErgNoise(float3((float)(Rr * std::cos(A)), (float)(Rr * std::sin(A)), (float)Z), 16.0f);
            Lo = std::min(Lo, (double)N); Hi = std::max(Hi, (double)N);
        }
        std::printf("ergs' noise at 16: %.3f .. %.3f\n", Lo, Hi);
        CHECK(Lo >= -1.0 && Hi <= 1.0 && Hi - Lo > 0.8, "the ergs' noise spans most of -1..1");
    }

    // --- T5e: the ice shelf (VoxelLandShelf.h) ---
    {
        FVoxelLandShelfParams SP;
        SP.bOn = true;
        SP.Seed = 1234u;
        SP.FrontNoise = std::getenv("SHELF_NOISE") ? std::atof(std::getenv("SHELF_NOISE")) : 5.0;
        // The noise: within its bound, and smooth (no step between neighbours a thousandth apart).
        double Lo = 9.0, Hi = -9.0, Step = 0.0;
        for (int i = 0; i < 40000; i++)
        {
            const double Z = 1.0 - 2.0 * (i + 0.5) / 40000.0, Rr = std::sqrt(1.0 - Z * Z), A = 2.39996323 * i;
            const double X = Rr * std::cos(A), Y = Rr * std::sin(A);
            const double F = VoxelLandShelf::FrontFbm(X, Y, Z, SP.Seed, 256.0);
            const double F2 = VoxelLandShelf::FrontFbm(X + 1.0e-3, Y, Z, SP.Seed, 256.0);
            Lo = std::min(Lo, F); Hi = std::max(Hi, F); Step = std::max(Step, std::fabs(F2 - F));
        }
        std::printf("shelf front noise: %.3f .. %.3f, largest change over 1e-3: %.4f\n", Lo, Hi, Step);
        CHECK(Lo > -1.05 && Hi < 1.05 && Hi - Lo > 0.6, "the front's noise within its bound and spread");
        CHECK(Step < 0.05, "the front's noise smooth");

        // The map's share by latitude (the provisional map above, Earth's latitude curve).
        FVoxelLandClimateGridParams ShelfParams = Params;
        ShelfParams.Shelf = SP;
        std::vector<float> ShelfCells((size_t)(W * H * 4));
        VoxelLandClimateGrid::BuildCells(Land.data(), Coast.data(), nullptr, W, H, ShelfParams, ShelfCells.data());
        std::printf("shelf share in the map by latitude (north; the year's mean at sea level):");
        double ShareAt[19] = {};
        for (int Band = 0; Band < 18; Band++)
        {
            double Sum = 0.0, TSum = 0.0;
            int Count = 0;
            for (int J = 0; J < H; J++)
            {
                const double Lat = std::asin(VoxelLandClimateGrid::RowMu(J, H)) * 180.0 / 3.14159265358979;
                if (Lat < 45.0 + 2.5 * Band || Lat >= 47.5 + 2.5 * Band) continue;
                for (int I = 0; I < W; I++)
                {
                    const int C = J * W + I;
                    Sum += std::max(-(double)ShelfCells[4 * C + 3], 0.0);
                    TSum += ShelfCells[4 * C + 0];
                    Count++;
                }
            }
            ShareAt[Band] = Count ? Sum / Count : 0.0;
            if (Band % 2 == 0) std::printf(" %.1f: %.2f (%.0f C)", 45.0 + 2.5 * Band, ShareAt[Band], Count ? TSum / Count : 0.0);
        }
        std::printf("\n");
        CHECK(ShareAt[0] == 0.0 && ShareAt[4] == 0.0, "no shelf at 45-57 degrees");
        CHECK(ShareAt[15] > 0.99, "the shelf full near the pole (82.5-85 degrees: the map's last rows): %f", ShareAt[15]);
        // Land cells outside it keep their continentality; inside, minus the share.
        int Kept = 0, Wrong = 0;
        for (int C = 0; C < W * H; C++)
        {
            if (Land[(size_t)C] < 0.5f || ShelfCells[4 * C + 3] < 0.0f) continue;
            Kept++;
            if (std::fabs(ShelfCells[4 * C + 3] - Cells[4 * C + 3]) > 1e-6f) Wrong++;
        }
        CHECK(Kept > 0 && Wrong == 0, "the land's continentality kept outside the shelf (%d of %d)", Wrong, Kept);

        // The bake's shelf along meridians: the year's mean from the map (bilinear, as the chain),
        // over an abyss, a shelf sea, low land and a mountain.
        auto TAt = [&](double X, double Y, double Z)
        {
            const FVoxelLandClimateTaps T = VoxelLandClimateTapsOf(float3((float)X, (float)Y, (float)Z));
            auto At = [&](int I, int J) { return (double)ShelfCells[4 * (J * W + I) + 0]; };
            const double A = At(T.X0, T.Y0) * (1.0 - T.FX) + At(T.X1, T.Y0) * T.FX;
            const double B = At(T.X0, T.Y1) * (1.0 - T.FX) + At(T.X1, T.Y1) * T.FX;
            return A * (1.0 - T.FY) + B * T.FY;
        };
        const double MaxF = 256.0;   // a 1024 bake
        double WorstJump = 0.0, FrontLatLo = 90.0, FrontLatHi = 0.0, TopLo = 1e9, TopHi = -1e9;
        int Meridians = 0;
        for (int M = 0; M < 72; M++)
        {
            const double Lon = M * 5.0 * 3.14159265358979 / 180.0;
            double Prev = 0.0;
            bool bFront = false;
            for (int K = 0; K <= 4000; K++)
            {
                const double Lat = (55.0 + 35.0 * K / 4000.0) * 3.14159265358979 / 180.0;
                const double X = std::cos(Lat) * std::cos(Lon), Y = std::cos(Lat) * std::sin(Lon), Z = std::sin(Lat);
                const VoxelLandShelf::FCell C = VoxelLandShelf::Apply(X, Y, Z, TAt(X, Y, Z), -3000.0, SP, MaxF);
                if (K > 0) WorstJump = std::max(WorstJump, std::fabs(C.Height - Prev));
                if (!bFront && C.Height > 0.0) { bFront = true; FrontLatLo = std::min(FrontLatLo, Lat * 180.0 / 3.14159265358979); FrontLatHi = std::max(FrontLatHi, Lat * 180.0 / 3.14159265358979); }
                if (C.Raised > 0.99) { TopLo = std::min(TopLo, C.Height); TopHi = std::max(TopHi, C.Height); }
                Prev = C.Height;
            }
            Meridians += bFront ? 1 : 0;
        }
        // (A step of 35/4000 degrees: about 1 km on Earth, a tenth of a 1024 bake's cell.)
        std::printf("shelf over a 3000 m abyss, 72 meridians: front at %.1f .. %.1f degrees, top %.1f .. %.1f m, largest step between samples %.0f m\n",
            FrontLatLo, FrontLatHi, TopLo, TopHi, WorstJump);
        CHECK(Meridians == 72, "every meridian reaches the shelf");
        CHECK(FrontLatLo > 62.0 && FrontLatHi < 80.0 && FrontLatHi - FrontLatLo > 2.0, "the front near the -15 C line (about 72 degrees)");
        CHECK(TopLo > 0.5 * SP.Height - 1.0 && TopHi < SP.Height + SP.Rise + 2.0 * SP.Roll + 1.0, "the top within its bounds");
        // Ground higher than the top stands through; low land is raised to it; deep in, the sea floor
        // is the draft's (or the top).
        {
            const double Lat = 85.0 * 3.14159265358979 / 180.0;
            const double X = std::cos(Lat), Y = 0.0, Z = std::sin(Lat);
            const double T = TAt(X, Y, Z);
            const VoxelLandShelf::FCell Mountain = VoxelLandShelf::Apply(X, Y, Z, T, 900.0, SP, MaxF);
            const VoxelLandShelf::FCell Low = VoxelLandShelf::Apply(X, Y, Z, T, 10.0, SP, MaxF);
            const VoxelLandShelf::FCell Sea = VoxelLandShelf::Apply(X, Y, Z, T, -3000.0, SP, MaxF);
            std::printf("shelf at 85 degrees: a 900 m mountain %.1f (raised %.2f), 10 m land %.1f (%.2f), the abyss %.1f (%.2f)\n",
                Mountain.Height, Mountain.Raised, Low.Height, Low.Raised, Sea.Height, Sea.Raised);
            CHECK(Mountain.Height == 900.0 && Mountain.Raised == 0.0, "a mountain stands through the shelf");
            CHECK(std::fabs(Low.Height - Sea.Height) < 1e-6 && Low.Raised > 0.99, "low land raised to the shelf's top");
        }
        // The chain's and the material's share (VoxelLandClimateShelfOf): on the shelf's heights in
        // its zone, none on the sea floor or high ground, and the material white there.
        {
            const FVoxelLandClimateCell Zone = Cell(-18.0f, 200.0f, 12.0f, -1.0f);
            const float OnShelf = VoxelLandClimateShelfOf(Zone, 60.0f).Share;
            const float Floor = VoxelLandClimateShelfOf(Zone, -300.0f).Share;
            const float High = VoxelLandClimateShelfOf(Zone, 600.0f).Share;
            const float Outside = VoxelLandClimateShelfOf(Cell(-18.0f, 200.0f, 12.0f, 0.4f), 60.0f).Share;
            // Its slope against a difference.
            const float D = (VoxelLandClimateShelfOf(Zone, 200.5f).Share - VoxelLandClimateShelfOf(Zone, 199.5f).Share);
            const float S = VoxelLandClimateShelfOf(Zone, 200.0f).Slope;
            const FVoxelLandClimatePoint P = VoxelLandClimateAt(Zone, DefaultLook(), Dir(80.0, 0.0) * 1.0e6f, Dir(80.0, 0.0), Dir(80.0, 0.0), 60.0f);
            std::printf("shelf share: on it %.2f, sea floor %.2f, at 600 m %.2f, outside the zone %.2f; slope %.5f (difference %.5f); the material's snow %.2f\n",
                OnShelf, Floor, High, Outside, S, D, P.Snow);
            CHECK(OnShelf == 1.0f && Floor == 0.0f && High == 0.0f && Outside == 0.0f, "the shelf's share by height and zone");
            CHECK(std::fabs(S - D) < 1e-4f, "the shelf share's slope");
            CHECK(P.Snow > 0.99f && P.Vegetation < 0.01f, "the shelf white");
        }
        // Outside its reach, nothing changes.
        {
            const double Lat = 50.0 * 3.14159265358979 / 180.0;
            const double X = std::cos(Lat), Y = 0.0, Z = std::sin(Lat);
            const VoxelLandShelf::FCell C = VoxelLandShelf::Apply(X, Y, Z, TAt(X, Y, Z), -3000.0, SP, MaxF);
            CHECK(C.Height == -3000.0 && C.Raised == 0.0 && C.Influence == 0.0, "no shelf at 50 degrees");
        }
        // The front's skirt: a few cells in front of the front the floor has come up toward the draft.
        // A look: the north polar cap from above (orthographic, 60 degrees out), over an abyss
        // with continents (a coarse noise) and a ridge of mountains.
        {
            const int Wd = 800;
            std::vector<unsigned char> Img((size_t)Wd * Wd * 3, 0);
            for (int y = 0; y < Wd; y++)
                for (int x = 0; x < Wd; x++)
                {
                    const double U = (x + 0.5) / Wd * 2.0 - 1.0, V = 1.0 - (y + 0.5) / Wd * 2.0;
                    const double Rho = std::sqrt(U * U + V * V) * std::sin(30.0 * 3.14159265358979 / 180.0);
                    if (Rho >= 1.0) continue;
                    const double Z = std::sqrt(1.0 - Rho * Rho);
                    const double Scale = Rho > 0.0 ? Rho / std::sqrt(U * U + V * V) : 0.0;
                    const double X = U * Scale, Y = V * Scale;
                    const double Cont = VoxelLandShelf::Fbm(X, Y, Z, 99u, 3.0, 5, 256.0);
                    double H0 = Cont > 0.12 ? 50.0 + 4000.0 * (Cont - 0.12) : -3000.0 + 2000.0 * (Cont + 0.3);
                    H0 = std::min(H0, -100.0 + 0.0 * H0) * (Cont > 0.12 ? 0.0 : 1.0) + (Cont > 0.12 ? H0 : 0.0);
                    const VoxelLandShelf::FCell C = VoxelLandShelf::Apply(X, Y, Z, TAt(X, Y, Z), H0, SP, MaxF);
                    unsigned char* Px = &Img[((size_t)y * Wd + x) * 3];
                    if (C.Height <= 0.0)
                    {
                        const double D = std::min(-C.Height / 3000.0, 1.0);
                        Px[0] = (unsigned char)(20 + 30 * (1 - D)); Px[1] = (unsigned char)(40 + 80 * (1 - D)); Px[2] = (unsigned char)(90 + 120 * (1 - D));
                    }
                    else if (C.Raised > 0.5)
                    {
                        const double G = std::min(std::max((C.Height - 20.0) / 100.0, 0.0), 1.0);
                        Px[0] = (unsigned char)(200 + 55 * G); Px[1] = (unsigned char)(210 + 45 * G); Px[2] = 255;
                    }
                    else
                    {
                        const double G = std::min(C.Height / 1500.0, 1.0);
                        Px[0] = (unsigned char)(90 + 100 * G); Px[1] = (unsigned char)(110 + 60 * G); Px[2] = (unsigned char)(60 + 80 * G);
                    }
                }
            FILE* F = std::fopen("shelf_cap.ppm", "wb");
            if (F)
            {
                std::fprintf(F, "P6\n%d %d\n255\n", Wd, Wd);
                std::fwrite(Img.data(), 1, Img.size(), F);
                std::fclose(F);
                std::printf("look: shelf_cap.ppm (the north polar cap to 60 degrees: sea blue, shelf white, land green-brown)\n");
            }
        }
    }

    std::printf("\n%s (%d failures)\n", Failures == 0 ? "ALL OK" : "FAILED", Failures);
    return Failures == 0 ? 0 : 1;
}
