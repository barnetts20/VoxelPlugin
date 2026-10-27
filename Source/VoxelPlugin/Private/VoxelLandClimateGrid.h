// VoxelLandClimateGrid.h - The land's climate map, built on the weather sim's grid (stage T5a,
// Design/LandClimate.md; the layout in Shaders/Private/VoxelLandClimate.ush): plain C++ on
// arrays, so the climate harness (Tools/LandClimateHarness) runs the plugin's own code.
//
// A cell holds the year's mean temperature at sea level (C), the precipitation (mm a year), the
// seasons' swing (the summer's mean less the year's, C) and the continentality (0..1). From:
//   - the land's share per cell (the land bake, VoxelLandClimate.cpp) and the distance to the sea
//     (CoastDistance: shortest paths over the grid);
//   - the atmosphere's climate (FAtmosphereClimateMap, resampled to this grid) when its means have
//     settled: its warmth gives the temperature's anomaly, its precipitation (net condensation,
//     arbitrary units) is scaled so the land's median is LandPrecipitation mm a year (and
//     PrecipitationContrast widens or narrows the spread about it);
//   - else a provisional model: a zonal profile (the equator's rain belt, the subtropics' dry belt,
//     the mid-latitudes' storm tracks, the dry poles; the shape of the observed zonal means) times
//     a continentality factor (drier inland), scaled the same way.
// The seasons aren't simulated: the swing comes from the latitude (the axial tilt's effect,
// Seasonality) and the continentality (inland swings wider: Earth's 45 degrees north is about
// 6 C at the coast, 12 C deep inland).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "VoxelLandShelf.h"

/** The settings the map depends on (FVoxelLandClimateSettings and the planet's climate). */
struct FVoxelLandClimateGridParams
{
    double RadiusM = 1.0e6;                // the sea's radius
    float EquatorTemperature = 27.0f;      // C, at sea level (APlanetActor::ClimateEquatorTemperature)
    float PoleTemperature = -25.0f;        // C
    float LatitudeExponent = 4.0f;         // the profile's |sin(latitude)| exponent (ClimateLatitudeExponent)
    float WarmthScale = 60.0f;             // C per unit of the sim's warmth (ClimateWarmthScale)
    float LandPrecipitation = 650.0f;      // mm a year: the land's median
    float PrecipitationContrast = 1.0f;    // the spread's exponent about the median
    float Seasonality = 1.0f;              // the seasons' swing, 1 Earth's
    float MaxWarmthAnomaly = 8.0f;         // C: the sim's warmth soft-clamped to this (0: no clamp)
    FVoxelLandShelfParams Shelf;           // T5e: the ice shelf (its share in w)
};

namespace VoxelLandClimateGrid
{
    /** A row's sin(latitude), cell centres (row 0 the south). */
    inline double RowMu(int J, int H)
    {
        return -1.0 + 2.0 * ((double)J + 0.5) / (double)H;
    }

    /** A column's longitude (radians), cell centres. */
    inline double ColumnLongitude(int I, int W)
    {
        return 2.0 * 3.14159265358979323846 * ((double)I + 0.5) / (double)W;
    }

    /** The planet-local unit direction of a cell's centre. */
    inline void CellDirection(int I, int J, int W, int H, double& OutX, double& OutY, double& OutZ)
    {
        const double Mu = RowMu(J, H);
        const double Cos = std::sqrt(std::max(1.0 - Mu * Mu, 0.0));
        const double Lon = ColumnLongitude(I, W);
        OutX = Cos * std::cos(Lon);
        OutY = Cos * std::sin(Lon);
        OutZ = Mu;
    }

    /** Smooth ramp, 0 at E0 to 1 at E1 (either order). */
    inline double Ramp(double E0, double E1, double X)
    {
        const double T = std::min(std::max((X - E0) / (E1 - E0), 0.0), 1.0);
        return T * T * (3.0 - 2.0 * T);
    }

    /** The provisional zonal precipitation's shape (relative) at sin(latitude) Mu: the rain belt at
     *  the equator, the dry belt near 25 degrees, the storm tracks near 48, dry toward the poles
     *  (the observed zonal means' shape; GPCP: about 2,000, 700, 1,000 and 200 mm). */
    inline double ZonalPrecipitation(double Mu)
    {
        const double Lat = std::fabs(std::asin(std::min(std::max(Mu, -1.0), 1.0))) * (180.0 / 3.14159265358979323846);
        const double Belt = 0.25 + 1.9 * std::exp(-(Lat / 11.0) * (Lat / 11.0)) + 0.85 * std::exp(-((Lat - 48.0) / 14.0) * ((Lat - 48.0) / 14.0));
        return Belt * (1.0 - 0.6 * Ramp(60.0, 90.0, Lat));
    }

    /** The continentality (0 at the coast toward 1 inland) at a distance from the sea, on a
     *  planet of RadiusM: 1 - exp(-d / (0.16 R)), about 1,000 km on Earth. */
    inline double Continentality(double DistanceM, double RadiusM)
    {
        return 1.0 - std::exp(-DistanceM / std::max(0.16 * RadiusM, 1.0));
    }

    /** The seasons' swing (C) at sin(latitude) Mu and a continentality. */
    inline double Swing(double Mu, double Continental, double Seasonality)
    {
        const double A = std::fabs(Mu);
        return std::max(Seasonality, 0.0) * (1.0 + 7.0 * A + 8.0 * Continental * std::sqrt(A));
    }

    /** The great-circle distance (m) between two cells' centres on a sphere of RadiusM. */
    inline double CellArc(int I0, int J0, int I1, int J1, int W, int H, double RadiusM)
    {
        double X0, Y0, Z0, X1, Y1, Z1;
        CellDirection(I0, J0, W, H, X0, Y0, Z0);
        CellDirection(I1, J1, W, H, X1, Y1, Z1);
        const double Dot = std::min(std::max(X0 * X1 + Y0 * Y1 + Z0 * Z1, -1.0), 1.0);
        return std::acos(Dot) * RadiusM;
    }

    /** The distance (m) from every cell to the nearest sea cell (land share under a half), over
     *  the grid's sixteen neighbours (the eight and the knight's moves, each its great-circle arc:
     *  within a few percent of the true distance in every direction), wrapping in longitude and
     *  across the poles, on a sphere of RadiusM; sea cells 0. Without any sea, half the
     *  circumference everywhere. */
    inline void CoastDistance(const float* Land, int W, int H, double RadiusM, float* Out)
    {
        const int N = W * H;
        std::vector<double> Lat((size_t)H);
        for (int J = 0; J < H; J++)
            Lat[(size_t)J] = std::asin(RowMu(J, H));
        std::vector<double> Dist((size_t)N, 1.0e30);
        typedef std::pair<double, int> FItem;
        std::priority_queue<FItem, std::vector<FItem>, std::greater<FItem>> Queue;
        for (int C = 0; C < N; C++)
        {
            if (Land[C] < 0.5f)
            {
                Dist[(size_t)C] = 0.0;
                Queue.push(FItem(0.0, C));
            }
        }
        if (Queue.empty())
        {
            for (int C = 0; C < N; C++) Out[C] = (float)(3.14159265358979323846 * RadiusM);
            return;
        }
        auto Relax = [&](int To, double D)
        {
            if (D < Dist[(size_t)To])
            {
                Dist[(size_t)To] = D;
                Queue.push(FItem(D, To));
            }
        };
        // The steps' arcs depend on the row only (and the step): a table per row.
        static const int Steps[16][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }, { -1, -1 },
            { 2, 1 }, { -2, 1 }, { 2, -1 }, { -2, -1 }, { 1, 2 }, { -1, 2 }, { 1, -2 }, { -1, -2 } };
        std::vector<double> Arcs((size_t)H * 16, 0.0);
        for (int J = 0; J < H; J++)
            for (int S = 0; S < 16; S++)
            {
                const int JN = J + Steps[S][1];
                Arcs[(size_t)J * 16 + (size_t)S] = JN >= 0 && JN < H ? CellArc(W / 2, J, W / 2 + Steps[S][0], JN, W, H, RadiusM) : 0.0;
            }
        while (!Queue.empty())
        {
            const FItem Top = Queue.top();
            Queue.pop();
            const int C = Top.second;
            if (Top.first > Dist[(size_t)C]) continue;
            const int I = C % W;
            const int J = C / W;
            for (int S = 0; S < 16; S++)
            {
                const int JN = J + Steps[S][1];
                if (JN < 0 || JN >= H) continue;
                Relax(JN * W + (I + Steps[S][0] + 2 * W) % W, Top.first + Arcs[(size_t)J * 16 + (size_t)S]);
            }
            if (J == 0 || J == H - 1)
            {
                // Over the pole: the cell half way round on the same row.
                const double Pole = J == H - 1 ? 0.5 * 3.14159265358979323846 : -0.5 * 3.14159265358979323846;
                Relax(J * W + (I + W / 2) % W, Top.first + 2.0 * std::fabs(Pole - Lat[(size_t)J]) * RadiusM);
            }
        }
        for (int C = 0; C < N; C++)
            Out[C] = (float)Dist[(size_t)C];
    }

    /** The median of the values where Mask is at least a half (all of them if none is). */
    inline double MaskedMedian(const std::vector<double>& Values, const float* Mask)
    {
        std::vector<double> Pick;
        Pick.reserve(Values.size());
        for (size_t C = 0; C < Values.size(); C++)
            if (Mask[C] >= 0.5f) Pick.push_back(Values[C]);
        if (Pick.empty()) Pick = Values;
        if (Pick.empty()) return 0.0;
        const size_t Mid = Pick.size() / 2;
        std::nth_element(Pick.begin(), Pick.begin() + (std::ptrdiff_t)Mid, Pick.end());
        return Pick[Mid];
    }

    /** The mean of the values where Mask is at least a half (all of them if none is). */
    inline double MaskedMean(const std::vector<double>& Values, const float* Mask)
    {
        double Sum = 0.0, Count = 0.0;
        for (size_t C = 0; C < Values.size(); C++)
            if (Mask[C] >= 0.5f) { Sum += Values[C]; Count += 1.0; }
        if (Count <= 0.0)
            for (size_t C = 0; C < Values.size(); C++) { Sum += Values[C]; Count += 1.0; }
        return Count > 0.0 ? Sum / Count : 0.0;
    }

    /** Stage T5d2: the sim's warmth per cell (Sim's w, W x H x 4) without its terrain's part, in C:
     *  the sim sees the ground (stage A9) and its air columns thin over high land, which its warmth
     *  reads as cold, while the land's own lapse rate cools it again by its height (a high continent
     *  came out glaciated at the equator). The part that follows the ground's height (Height: m, W x
     *  H; null: none) is fitted over the sphere (least squares, equal-area cells) and removed where
     *  it makes high ground colder, each row's mean kept (the warmth is an anomaly about its
     *  latitude's): the slope fitted within the rows, over the land (Land: W x H shares, null: all
     *  cells); then times WarmthScale, soft-clamped to MaxWarmthAnomaly (tanh). Out: W x H. */
    inline void SimWarmth(const float* Sim, const float* Height, const float* Land, int W, int H, const FVoxelLandClimateGridParams& P, double* Out)
    {
        const int N = W * H;
        for (int C = 0; C < N; C++)
            Out[C] = (double)Sim[4 * C + 3];
        if (Height)
        {
            // Within the rows (each row's means taken out: the warmth is an anomaly about its
            // latitude's) and over the land (the sea's 0 m would read the sim's land-sea contrast as
            // the height's).
            std::vector<double> RowX((size_t)H, 0.0), RowY((size_t)H, 0.0);
            for (int J = 0; J < H; J++)
            {
                double SX = 0.0, SY = 0.0, Count = 0.0;
                for (int I = 0; I < W; I++)
                {
                    const int C = J * W + I;
                    if (Land && Land[C] < 0.5f) continue;
                    SX += std::max((double)Height[C], 0.0);
                    SY += Out[C];
                    Count += 1.0;
                }
                RowX[(size_t)J] = Count > 0.0 ? SX / Count : 0.0;
                RowY[(size_t)J] = Count > 0.0 ? SY / Count : 0.0;
            }
            double SXX = 0.0, SXY = 0.0;
            for (int C = 0; C < N; C++)
            {
                if (Land && Land[C] < 0.5f) continue;
                const int J = C / W;
                const double X = std::max((double)Height[C], 0.0) - RowX[(size_t)J];
                SXX += X * X;
                SXY += X * (Out[C] - RowY[(size_t)J]);
            }
            const double Slope = SXX > 1.0e-9 ? SXY / SXX : 0.0;
            if (Slope < 0.0)
            {
                // Removed about each row's mean height over all its cells (the row's mean warmth kept).
                for (int J = 0; J < H; J++)
                {
                    double RowMean = 0.0;
                    for (int I = 0; I < W; I++) RowMean += std::max((double)Height[J * W + I], 0.0);
                    RowMean /= W;
                    for (int I = 0; I < W; I++)
                        Out[J * W + I] -= Slope * (std::max((double)Height[J * W + I], 0.0) - RowMean);
                }
            }
        }
        const double A = std::max((double)P.MaxWarmthAnomaly, 0.0);
        for (int C = 0; C < N; C++)
        {
            const double T = (double)P.WarmthScale * Out[C];
            Out[C] = A > 0.0 ? A * std::tanh(T / A) : T;
        }
    }

    /** The map's cells (Out: W x H x 4, the layout above) from the land's share and the distance
     *  to the sea (W x H each), and the sim's climate resampled to this grid (Sim: W x H x 4, as
     *  FAtmosphereClimateMap's cells: x precipitation, w warmth; null for the provisional model).
     *  False when the sim's precipitation was all zero (the provisional model's then). Height
     *  (T5d2; m, W x H, null: none): the land's mean height a cell, for SimWarmth. */
    inline bool BuildCells(const float* Land, const float* Coast, const float* Sim, int W, int H, const FVoxelLandClimateGridParams& P, float* Out,
        const float* Height = nullptr)
    {
        const int N = W * H;
        std::vector<double> Raw((size_t)N, 0.0);
        bool bSim = Sim != nullptr;
        if (bSim)
        {
            for (int C = 0; C < N; C++)
                Raw[(size_t)C] = std::max((double)Sim[4 * C + 0], 0.0);
        }
        double Scale = bSim ? MaskedMedian(Raw, Land) : 0.0;
        if (bSim && Scale <= 1.0e-9)
            Scale = MaskedMean(Raw, Land);
        if (bSim && Scale <= 1.0e-9)
            bSim = false;
        if (!bSim)
        {
            for (int J = 0; J < H; J++)
            {
                const double Zonal = ZonalPrecipitation(RowMu(J, H));
                for (int I = 0; I < W; I++)
                {
                    const int C = J * W + I;
                    const double Inland = 0.3 + 0.7 * std::exp(-(double)Coast[C] / std::max(0.19 * P.RadiusM, 1.0));
                    Raw[(size_t)C] = Zonal * Inland;
                }
            }
            Scale = MaskedMedian(Raw, Land);
        }
        const double Median = std::max((double)P.LandPrecipitation, 0.0);
        const double Contrast = std::max((double)P.PrecipitationContrast, 0.05);
        std::vector<double> Warm((size_t)N, 0.0);
        if (bSim)
            SimWarmth(Sim, Height, Land, W, H, P, Warm.data());
        for (int J = 0; J < H; J++)
        {
            const double Mu = RowMu(J, H);
            const double Zonal = (double)P.EquatorTemperature + ((double)P.PoleTemperature - (double)P.EquatorTemperature)
                * std::pow(std::fabs(Mu), std::max((double)P.LatitudeExponent, 0.1));
            for (int I = 0; I < W; I++)
            {
                const int C = J * W + I;
                const double Warmth = Warm[(size_t)C];
                const double Relative = Scale > 1.0e-12 ? Raw[(size_t)C] / Scale : 1.0;
                const double Rain = std::min(Median * std::pow(std::max(Relative, 0.0), Contrast), 20000.0);
                const double Continental = Land[C] >= 0.5f ? Continentality((double)Coast[C], P.RadiusM) : 0.0;
                Out[4 * C + 0] = (float)(Zonal + Warmth);
                Out[4 * C + 1] = (float)Rain;
                Out[4 * C + 2] = (float)Swing(Mu, Continental, (double)P.Seasonality);
                // T5e: within the ice shelf's reach (VoxelLandShelf.h, the front's octaves the map
                // resolves), minus its share: the material's whitening, the polar layer's ice and
                // the rivers' fade (VoxelLandClimate.ush VoxelLandClimateShelfOf). The continentality
                // otherwise (only the debug view reads it).
                double Shelf = 0.0;
                if (!VoxelLandShelf::IsOutside(Zonal + Warmth, P.Shelf))
                {
                    double X = 0.0, Y = 0.0, Z = 0.0;
                    CellDirection(I, J, W, H, X, Y, Z);
                    Shelf = VoxelLandShelf::MapShare(VoxelLandShelf::PhiAt(X, Y, Z, Zonal + Warmth, P.Shelf, VoxelLandShelf::MapMaxFrequency));
                }
                Out[4 * C + 3] = Shelf > 0.0 ? (float)-Shelf : (float)Continental;
            }
        }
        return bSim;
    }

    /** Stage T5c: the sand's drift on the grid (VoxelLandDunes.ush's entries after the cells): per
     *  cell x east, y north (the resultant drift's unit direction, or 0 where there is none), z its
     *  steadiness (Fryberger's RDP / DP, 0..1), w 0. Sim (W x H x 4, FAtmosphereClimateMap::Drift
     *  resampled: x, y the resultant drift, z the drift potential, w the mean speed): the resultant
     *  and the potential box-filtered over BlurCells each way first (wrapping in longitude, clamped
     *  at the poles' rows), so the direction turns over a few hundred kilometres and the steadiness
     *  is the region's (where the winds of neighbouring cells disagree, it falls). Null: the
     *  provisional winds (the three cells of each hemisphere: trades toward the west and the
     *  equator, westerlies toward the east and the pole, polar easterlies; the resultant zero at
     *  the doldrums, the horse latitudes and the polar fronts, the trades steadiest). */
    inline void BuildDrift(const float* Sim, int W, int H, int BlurCells, float* Out)
    {
        const int N = W * H;
        if (!Sim)
        {
            for (int J = 0; J < H; J++)
            {
                const double Lat = std::asin(std::min(std::max(RowMu(J, H), -1.0), 1.0));
                const double S6 = std::sin(6.0 * Lat);
                const double U = -std::sin(6.0 * std::fabs(Lat));
                const double V = -0.4 * S6;
                const double L = std::sqrt(U * U + V * V);
                const double Deg = std::fabs(Lat) * (180.0 / 3.14159265358979323846);
                const double Steady = std::fabs(S6) * (0.85 + (0.5 - 0.85) * Ramp(25.0, 35.0, Deg));
                for (int I = 0; I < W; I++)
                {
                    float* C = Out + 4 * ((size_t)J * W + I);
                    C[0] = L > 1e-6 ? (float)(U / L) : 0.0f;
                    C[1] = L > 1e-6 ? (float)(V / L) : 0.0f;
                    C[2] = (float)Steady;
                    C[3] = 0.0f;
                }
            }
            return;
        }
        // The box filter, separable: along the rows (wrapping), then the columns (clamped).
        const int R = std::max(BlurCells, 0);
        std::vector<double> A((size_t)N * 3, 0.0), B((size_t)N * 3, 0.0);
        for (int J = 0; J < H; J++)
            for (int I = 0; I < W; I++)
            {
                double Sum[3] = { 0.0, 0.0, 0.0 };
                for (int K = -R; K <= R; K++)
                {
                    const int X = ((I + K) % W + W) % W;
                    const float* C = Sim + 4 * ((size_t)J * W + X);
                    for (int c = 0; c < 3; c++) Sum[c] += (double)C[c];
                }
                for (int c = 0; c < 3; c++) A[3 * ((size_t)J * W + I) + c] = Sum[c];
            }
        for (int J = 0; J < H; J++)
            for (int I = 0; I < W; I++)
            {
                double Sum[3] = { 0.0, 0.0, 0.0 };
                for (int K = -R; K <= R; K++)
                {
                    const int Y = std::min(std::max(J + K, 0), H - 1);
                    for (int c = 0; c < 3; c++) Sum[c] += A[3 * ((size_t)Y * W + I) + c];
                }
                for (int c = 0; c < 3; c++) B[3 * ((size_t)J * W + I) + c] = Sum[c];
            }
        for (int i = 0; i < N; i++)
        {
            const double X = B[3 * (size_t)i], Y = B[3 * (size_t)i + 1], DP = B[3 * (size_t)i + 2];
            const double L = std::sqrt(X * X + Y * Y);
            float* C = Out + 4 * (size_t)i;
            const bool bAny = L > 1e-12 && DP > 1e-12;
            C[0] = bAny ? (float)(X / L) : 0.0f;
            C[1] = bAny ? (float)(Y / L) : 0.0f;
            C[2] = bAny ? (float)std::min(std::max(L / DP, 0.0), 1.0) : 0.0f;
            C[3] = 0.0f;
        }
    }

    /** Stage T5d2: the ice sheets on the grid (the land chain's third block of entries): per cell
     *  x the signed distance into the ice (m: positive inside, negative outside; the margin lies
     *  half a cell from the centres either side of it), y the ground's mean height (m; Height's),
     *  z, w 0. The land chain makes the thickness from the distance (VoxelLandIce.ush: Nye's
     *  plastic profile, h = sqrt(2 tau / (rho g) d), tau 100 kPa: 470 m 10 km in, 1.5 km at 100 km,
     *  capped), after its B-spline read: a distance is close to linear across the margin, so the
     *  spline keeps the margin where it is (a thickness stepping from 0 to hundreds of metres at the
     *  first iced cell would spread a cell beyond it). Iced (W x H, 0..1): the share of the cell the
     *  climate ices; above a half it is ice. The poles' rows (and two next to them, fading) take
     *  their rows' means: the grid's rows close in on the pole, where the read would otherwise fan
     *  out by longitude. */
    inline void IceSheet(const float* Iced, const float* Height, int W, int H, double RadiusM, float* Out)
    {
        const int N = W * H;
        std::vector<float> Inside((size_t)N, 0.0f), Outside((size_t)N, 0.0f), NotIced((size_t)N, 0.0f);
        for (int C = 0; C < N; C++) NotIced[(size_t)C] = 1.0f - Iced[C];
        CoastDistance(Iced, W, H, RadiusM, Inside.data());
        CoastDistance(NotIced.data(), W, H, RadiusM, Outside.data());
        // The cells' equal area: (4 pi R^2) / (W H); half its side is the margin's offset.
        const double HalfCell = 0.5 * RadiusM * std::sqrt(4.0 * 3.14159265358979323846 / ((double)W * (double)H));
        for (int C = 0; C < N; C++)
        {
            float* E = Out + 4 * (size_t)C;
            E[0] = Iced[C] >= 0.5f ? (float)((double)Inside[(size_t)C] - HalfCell) : (float)(HalfCell - (double)Outside[(size_t)C]);
            E[1] = Height ? std::max(Height[C], 0.0f) : 0.0f;
            E[2] = 0.0f;
            E[3] = 0.0f;
        }
        // The poles: each row's mean, fading back to the row's own over the next two.
        for (int Side = 0; Side < 2; Side++)
            for (int K = 0; K < 3 && K < H; K++)
            {
                const int J = Side == 0 ? K : H - 1 - K;
                const double Keep = (double)K / 3.0;
                for (int c = 0; c < 2; c++)
                {
                    double Mean = 0.0;
                    for (int I = 0; I < W; I++) Mean += Out[4 * ((size_t)J * W + I) + c];
                    Mean /= W;
                    for (int I = 0; I < W; I++)
                    {
                        float& V = Out[4 * ((size_t)J * W + I) + c];
                        V = (float)(Mean + Keep * ((double)V - Mean));
                    }
                }
            }
    }
}
