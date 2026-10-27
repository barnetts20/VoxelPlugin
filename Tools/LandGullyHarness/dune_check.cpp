// T5c harness: the dunes (VoxelLandDunes.ush) compiled as C++ with the authored dune and sand
// detail sets (read from their mip no larger than 1024, as the pack does): continuity over the
// wind's direction (the bins) and its steadiness (sharp to rounded, the many winds), the gradient
// against differences, the blend's range against DuneBlendBound, and hillshade looks.
// T5d: with a glacial set, the glacial layer in the peaks' place (VoxelLandPeaksShapeDF, the
// generated peaks set beside it) on a synthetic mountain slope under a uniform climate:
// continuity up the slope through the ice line and the glacial maximum's line, the gradient
// (the ice line's character gradient included) against differences, the range against
// VoxelLand::PeakReach's rule, and looks.
//
// g++ -O2 -std=c++20 -ffp-contract=off -pthread -I<Public> -I<Private> -I<Shaders/Private> dune_check.cpp
// T5d2: with an ice sheet set, the ice sheet over a synthetic land (VoxelLandIceSurfaceDF): continuity
// across the margin and the nunataks, the gradient (Scale x the land's + its own) against
// differences, and a look.
//
// T5d3: with a polar set, the polar surfaces (VoxelLandPolarDF): direction sweeps, lines at fine
// steps, the gradient (the shape's and the shares' known changes) against differences over a
// synthetic ground (a glacier line on a slope, an ice sheet's margin, the permafrost's ramp), the
// range against VoxelLand::PolarReach's rule, the footprint fade, and looks.
//
// dune_check dunes.vxnt sanddetail.vxnt OutPrefix [glacial.vxnt [icesheet.vxnt [polar.vxnt]]]
#include "VoxelShaderCompat.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <string>
#include "VoxelNoiseTiles.h"
#include "VoxelLandClimateGrid.h"

namespace VoxelShader
{
#include "VoxelDensityCore.ush"
#include "VoxelDF64.ush"
#include "VoxelCubeSphere.ush"
    struct FHost
    {
        const float* LandHeight = nullptr;
        const float2* LandTanTable = nullptr;
        const uint* LandFlow = nullptr;
        const uint* LandForm = nullptr;
        const uint* LandRelief = nullptr;
        const uint* LandProvince = nullptr;
        const uint* LandRiverTiles = nullptr;
        const float* LandRiverData = nullptr;
        const uint* LandNoise = nullptr;
        uint LandN = 1, LandPad = 0, LandHasFlow = 0, LandHasForm = 0;
        float LandHillRelief = 0.0f;
        float2 LandHillFrequency;
        uint LandHillOctaves = 0, LandHillSeed = 0;
        float LandPeakStrength = 0.0f, LandPeakTexelsPerCell = 0.0f, LandPeakSpacingM = 0.0f, LandPeakReliefM = 0.0f, LandPeakReliefStart = 500.0f, LandPeakReliefEnd = 1500.0f;
        float LandPeakMeanR = 0.0f, LandPeakMeanG = 0.0f, LandPeakScaleR = 1.0f, LandPeakScaleG = 1.0f, LandPeakScaleB = 1.0f;
        uint LandPeakSeed = 0, LandNoiseN = 0, LandNoiseMips = 0;
        uint LandDetailBase = 0, LandDetailN = 0, LandDetailMips = 0;
        float LandPeakDetailTexelsPerCell = 0.0f, LandDetailMeanHard = 0.0f, LandDetailMeanSoft = 0.0f;
        float LandPeakDetailScaleHard = 0.0f, LandPeakDetailScaleSoft = 0.0f, LandPeakDetailFade = 0.0f;
        float LandHillTexelsPerCell = 0.0f, LandHillScaleR = 1.0f, LandHillScaleG = 1.0f, LandHillScaleB = 1.0f, LandHillMeanR = 0.0f, LandHillMeanG = 0.0f, LandHillSpacingM = 0.0f;
        uint LandHillBase = 0, LandHillN = 0, LandHillMips = 0;
        float LandHillSwellTexelsPerCell = 0.0f, LandHillSwellM = 0.0f, LandHillSwellScaleG = 1.0f;
        float LandHillDetailTexelsPerCell = 0.0f, LandHillDetailScaleHard = 0.0f, LandHillDetailScaleSoft = 0.0f, LandHillDetailFade = 0.0f, LandHillFoothill = 0.0f;
        float LandStrataTexelsPerCell = 0.0f, LandStrataScaleR = 1.0f, LandStrataScaleG = 1.0f, LandStrataScaleB = 1.0f, LandStrataMeanR = 0.0f, LandStrataMeanG = 0.0f, LandStrataSpacingM = 0.0f;
        uint LandStrataBase = 0, LandStrataN = 0, LandStrataMips = 0, LandStrataSeed = 0;
        float LandStrataRelief = 0.0f, LandStrataFlat = 0.0f, LandStrataThreshold = 2.0f, LandStrataWidth = 0.05f;
        float LandStrataDetailTexelsPerCell = 0.0f, LandStrataDetailScaleHard = 0.0f, LandStrataDetailScaleSoft = 0.0f, LandStrataDetailFade = 0.0f;
        uint LandRiverTilesPerFace = 0;
        float LandRiverRadiusM = 1.0f, LandRiverWidthScale = 1.0f, LandRiverValleyScale = 1.0f;
        const float* LandWater = nullptr;   // R2: the carve (VoxelLandCarve.ush)
        uint LandHasWater = 0;
        float LandRiverCarve = 0.0f, LandRiverDepthScale = 1.0f, LandRiverFloodplainScale = 1.0f, LandRiverMinLog2Area = 0.0f, LandLakeShoreBand = 0.0f, LandRiverMeander = 0.0f, LandRiverFalls = 0.0f;
        float LandDetailTexelsPerCell = 0.0f, LandDetailMeanA = 0.0f, LandDetailScaleA = 0.0f, LandDetailScaleGain = 0.0f, LandDetailFade = 0.0f;   // N1: the detail layer
        uint LandDetailScales = 0, LandDetailSeed = 0;
        const float4* LandClimate = nullptr;   // T5c
        uint LandHasClimate = 0, LandDuneBase = 0, LandDuneN = 0, LandDuneMips = 0, LandDuneSeed = 0, LandDuneDetailBase = 0, LandDuneDetailN = 0, LandDuneDetailMips = 0;
        float LandClimateLapse = 6.5f, LandDuneTexelsPerCell = 0.0f, LandDuneScaleR = 1.0f, LandDuneScaleG = 1.0f, LandDuneMeanR = 0.0f, LandDuneMeanG = 0.0f, LandDuneRelief = 0.0f;
        float LandDuneErgFrequency = 1.0f, LandDuneErgThreshold = 2.0f, LandDuneErgWidth = 0.06f, LandDuneReliefMax = 400.0f;
        float LandDuneDetailTexelsPerCell = 0.0f, LandDuneDetailScale = 0.0f, LandDuneDetailMean = 0.0f, LandDuneDetailFade = 0.0f;
        uint LandGlacialBase = 0, LandGlacialN = 0, LandGlacialMips = 0, LandGlacialSeed = 0;   // T5d
        float LandGlacialTexelsPerCell = 0.0f, LandGlacialScaleR = 1.0f, LandGlacialScaleG = 1.0f, LandGlacialScaleB = 1.0f, LandGlacialMeanR = 0.0f, LandGlacialMeanG = 0.0f;
        float LandGlacialReliefM = 0.0f, LandGlacialStrength = 0.0f, LandGlacialSpacingM = 0.0f, LandGlacialCooling = 0.0f, LandGlacialDetailScaleHard = 0.0f, LandGlacialDetailScaleSoft = 0.0f;
        float LandIceMaxThickness = 0.0f, LandIceBlend = 0.0f, LandIceSwellTexelsPerCell = 0.0f, LandIceMeanG = 0.0f, LandIceSwellM = 0.0f, LandIceSwellSpacingM = 0.0f;   // T5d2
        uint LandIceBase = 0, LandIceN = 0, LandIceMips = 0, LandIceSeed = 0;
        uint LandPolarBase = 0, LandPolarN = 0, LandPolarMips = 0, LandPolarSeed = 0;   // T5d3
        float LandPolarCrevasseTexelsPerCell = 0.0f, LandPolarCrevasseM = 0.0f, LandPolarCrevasseMean = 0.0f, LandPolarCrevasseFade = 0.0f;
        float LandPolarSastrugiTexelsPerCell = 0.0f, LandPolarSastrugiM = 0.0f, LandPolarSastrugiMean = 0.0f, LandPolarSastrugiFade = 0.0f;
        float LandPolarPolygonTexelsPerCell = 0.0f, LandPolarPolygonM = 0.0f, LandPolarPolygonMean = 0.0f, LandPolarPolygonFade = 0.0f;
        float LandPolarPermafrostT = -6.0f, LandPolarCrevasseTurn = 0.0f, LandPolarSastrugiTurn = 0.0f;
        uint LandPolarDuneBase = 0, LandPolarDuneN = 0, LandPolarDuneMips = 0;
        float LandPolarDuneTexelsPerCell = 0.0f, LandPolarDuneM = 0.0f, LandPolarDuneMean = 0.0f, LandPolarDuneFade = 0.0f;
        float LandCellM = 1.0f;
        float LandGullyStrength = 0.0f;
        uint LandGullyOctaves = 0;
        float LandGullyFlowGain = 0.0f, LandGullyDetail = 1.0f;
        uint LandGullySeed = 0;
#include "VoxelLandSample.ush"
#include "VoxelLandGully.ush"
    };
}
using namespace VoxelShader;
using FVoxelLandDuneSite = FHost::FVoxelLandDuneSite;
using FVoxelLandPeaks = FHost::FVoxelLandPeaks;
using FVoxelLandIceSurface = FHost::FVoxelLandIceSurface;

struct FChain
{
    VoxelNoiseTiles::FSet Header;
    std::vector<uint32_t> Words;
};

// As VoxelLandBake.cpp's Read with MaxN: the chain from the first mip no larger than MaxN.
static bool ReadChain(const char* Path, uint32_t MaxN, FChain& Out)
{
    FILE* F = fopen(Path, "rb");
    if (!F) { printf("can't open %s\n", Path); return false; }
    fseek(F, 0, SEEK_END);
    const long Size = ftell(F);
    fseek(F, 0, SEEK_SET);
    std::vector<uint8_t> Head(4096);
    if (fread(Head.data(), 1, Head.size(), F) != Head.size()) { fclose(F); return false; }
    uint64_t DataOffset = 0;
    const std::string Why = VoxelNoiseTiles::ReadVxntHeader(Head.data(), Head.size(), (uint64_t)Size, Out.Header, DataOffset);
    if (!Why.empty()) { printf("%s: %s\n", Path, Why.c_str()); fclose(F); return false; }
    uint32_t Skip = 0;
    while ((Out.Header.N >> Skip) > MaxN && Skip + 1 < Out.Header.Mips) Skip++;
    const uint64_t Skipped = VoxelNoiseTiles::ChainTexels(Out.Header.N) - VoxelNoiseTiles::ChainTexels(Out.Header.N >> Skip);
    const float Scale = 1.0f / (float)(1u << Skip);
    Out.Header.N >>= Skip;
    Out.Header.Mips -= Skip;
    for (auto& C : Out.Header.Channel)
        if (C.Kind == VoxelNoiseTiles::KindErodedHeight || C.Kind == VoxelNoiseTiles::KindBlurred) { C.Min *= Scale; C.Max *= Scale; C.Mean *= Scale; C.Std *= Scale; }
    const uint64_t Texels = VoxelNoiseTiles::ChainTexels(Out.Header.N);
    Out.Words.resize(2 * Texels);
    fseek(F, (long)(DataOffset + Skipped * 8), SEEK_SET);
    const bool bOk = fread(Out.Words.data(), 8, Texels, F) == Texels;
    fclose(F);
    printf("%s: %u^2 from mip %u, %u mips\n", Path, Out.Header.N, Skip, Out.Header.Mips);
    return bOk;
}

static double UnitMean(const FChain& C, int Channel)
{
    const VoxelNoiseTiles::FSet& S = C.Header;
    const uint16_t* T = (const uint16_t*)C.Words.data();
    const size_t N = (size_t)S.N * S.N;
    double Sum = 0.0;
    for (size_t i = 0; i < N; i++) Sum += T[i * 4 + Channel] / 65535.0;
    return Sum / (double)N;
}

static float2 DF(double V) { const float Hi = (float)V; return float2(Hi, (float)(V - (double)Hi)); }

struct FSetup
{
    FHost H;
    std::vector<uint32_t> Words;
    std::vector<float2> Tan;
    std::vector<float4> Climate;
    double RadiusCm = 0.0;
};

static void Setup(FSetup& S, const FChain& Dunes, const FChain& Sand, bool bDetail)
{
    // A pack of the two chains in rows of the wider's N.
    const uint32_t RowN = std::max(Dunes.Header.N, Sand.Header.N);
    auto Append = [&](const FChain& C) {
        const uint64_t Count = VoxelNoiseTiles::ChainTexels(C.Header.N);
        const uint64_t Rows = (Count + RowN - 1) / RowN;
        const uint32_t Base = (uint32_t)(S.Words.size() / 2);
        S.Words.resize(S.Words.size() + 2 * Rows * RowN, 0u);
        memcpy(S.Words.data() + 2 * (size_t)Base, C.Words.data(), Count * 8);
        return Base;
    };
    const uint32_t DuneBase = Append(Dunes);
    const uint32_t SandBase = Append(Sand);
    FHost& H = S.H;
    H.LandNoise = S.Words.data();
    // A 1000 km planet, 1024 bake cells a face edge.
    S.RadiusCm = 1.0e8;
    H.LandN = 1024;
    H.LandPad = 4;
    // FVoxelLandBake::BuildTanTable.
    S.Tan.resize(H.LandN + 2 * H.LandPad);
    for (size_t i = 0; i < S.Tan.size(); i++)
        S.Tan[i] = DF(std::tan((-1.0 + (2.0 * ((double)i - H.LandPad) + 1.0) / H.LandN) * 3.14159265358979323846 * 0.25));
    H.LandTanTable = S.Tan.data();
    H.LandCellM = (float)(3.14159265358979 * 0.5 * S.RadiusCm * 0.01 / H.LandN);
    const double SpacingM = 600.0;
    const auto& DC = Dunes.Header.Channel;
    const uint32_t SpacingTexels = Dunes.Header.N / DC[0].CellsPerTile;
    const double TexelM = SpacingM / SpacingTexels;
    const float Range = std::max(DC[0].Max - DC[0].Min, DC[1].Max - DC[1].Min);
    H.LandDuneTexelsPerCell = (float)(H.LandCellM / TexelM);
    H.LandDuneBase = DuneBase; H.LandDuneN = Dunes.Header.N; H.LandDuneMips = Dunes.Header.Mips;
    H.LandDuneScaleR = (DC[0].Max - DC[0].Min) / Range;
    H.LandDuneScaleG = (DC[1].Max - DC[1].Min) / Range;
    H.LandDuneMeanR = (float)UnitMean(Dunes, 0);
    H.LandDuneMeanG = (float)UnitMean(Dunes, 1);
    H.LandDuneRelief = (float)(Range * TexelM);
    H.LandDuneSeed = 12345u;
    H.LandHasClimate = 1u;
    if (bDetail)
    {
        const auto& G = Sand.Header.Channel[1];
        const double Wavelength = 24.0;
        const double DTexelM = Wavelength / ((double)Sand.Header.N / G.CellsPerTile);
        H.LandDuneDetailTexelsPerCell = (float)(H.LandCellM / DTexelM);
        H.LandDuneDetailBase = SandBase; H.LandDuneDetailN = Sand.Header.N; H.LandDuneDetailMips = Sand.Header.Mips;
        H.LandDuneDetailScale = (float)((G.Max - G.Min) * DTexelM / H.LandDuneRelief);
        H.LandDuneDetailMean = (float)UnitMean(Sand, 1);
        H.LandDuneDetailFade = std::floor(std::log2((double)Sand.Header.N / G.CellsPerTile)) - 1.0f;
    }
    printf("dunes: %.0f m relief at %.0f m (%.1f texels a cell), means R %.3f G %.3f; detail %.2f (unit), %.1f texels a cell\n",
        H.LandDuneRelief, SpacingM, H.LandDuneTexelsPerCell, H.LandDuneMeanR, H.LandDuneMeanG, H.LandDuneDetailScale, H.LandDuneDetailTexelsPerCell);
}

// A point on the +X face near (U, V) metres from a reference spot, and its tangent frame.
struct FPoint { double X, Y, Z; };
static FPoint At(const FSetup& S, double Um, double Vm)
{
    // The reference: 20 degrees off the face centre each way.
    const double U0 = 0.31, V0 = -0.22;
    const double R = S.RadiusCm;
    double X = 1.0, Y = U0, Z = V0;
    const double L = std::sqrt(X * X + Y * Y + Z * Z);
    X /= L; Y /= L; Z /= L;
    // East and north at the reference.
    double EX = -Y, EY = X, EZ = 0.0;
    const double EL = std::sqrt(EX * EX + EY * EY);
    EX /= EL; EY /= EL;
    const double NX = Y * EZ - Z * EY, NY = Z * EX - X * EZ, NZ = X * EY - Y * EX;
    double PX = X * R + (EX * Um + NX * Vm) * 100.0, PY = Y * R + (EY * Um + NY * Vm) * 100.0, PZ = Z * R + (EZ * Um + NZ * Vm) * 100.0;
    const double PL = std::sqrt(PX * PX + PY * PY + PZ * PZ) / R;
    return { PX / PL, PY / PL, PZ / PL };
}

static FVoxelLandDuneSite Site(const FPoint& P, double AngleDeg, double Steadiness)
{
    const double L = std::sqrt(P.X * P.X + P.Y * P.Y + P.Z * P.Z);
    const double X = P.X / L, Y = P.Y / L, Z = P.Z / L;
    double EX = -Y, EY = X, EZ = 0.0;
    const double EL = std::sqrt(EX * EX + EY * EY);
    EX /= EL; EY /= EL;
    const double NX = Y * EZ - Z * EY, NY = Z * EX - X * EZ, NZ = X * EY - Y * EX;
    const double A = AngleDeg * 3.14159265358979 / 180.0;
    FVoxelLandDuneSite Out;
    Out.Share = 1.0f;
    Out.Wind = float3((float)(EX * std::cos(A) + NX * std::sin(A)), (float)(EY * std::cos(A) + NY * std::sin(A)), (float)(EZ * std::cos(A) + NZ * std::sin(A)));
    Out.Steadiness = (float)Steadiness;
    return Out;
}

static FVoxelLandPeaks Eval(const FSetup& S, const FPoint& P, const FVoxelLandDuneSite& Site, float FootprintCm)
{
    FHost H = S.H;
    return H.VoxelLandDunesDF(DF(P.X), DF(P.Y), DF(P.Z), Site, H.LandDuneRelief, FootprintCm, true);
}

static void WritePPM(const std::string& Path, int W, int Hh, const std::vector<float>& Height, double CellM)
{
    FILE* F = fopen(Path.c_str(), "wb");
    fprintf(F, "P6\n%d %d\n255\n", W, Hh);
    for (int y = 0; y < Hh; y++)
        for (int x = 0; x < W; x++)
        {
            auto Hgt = [&](int i, int j) { i = std::clamp(i, 0, W - 1); j = std::clamp(j, 0, Hh - 1); return Height[(size_t)j * W + i]; };
            const double DX = (Hgt(x + 1, y) - Hgt(x - 1, y)) / (2.0 * CellM);
            const double DY = (Hgt(x, y - 1) - Hgt(x, y + 1)) / (2.0 * CellM);
            // Sun from the north-west, 35 degrees up.
            double NX = -DX, NY = -DY, NZ = 1.0;
            const double NL = std::sqrt(NX * NX + NY * NY + NZ * NZ);
            const double LX = -0.58, LY = 0.58, LZ = 0.57;
            const double Shade = std::max((NX * LX + NY * LY + NZ * LZ) / NL, 0.0);
            const unsigned char C[3] = { (unsigned char)std::min(255.0, 70 + 185 * Shade), (unsigned char)std::min(255.0, 55 + 160 * Shade), (unsigned char)std::min(255.0, 35 + 120 * Shade) };
            fwrite(C, 1, 3, F);
        }
    fclose(F);
}

// --- T5d: the glacial layer -------------------------------------------------------------------

struct FGlacialSetup
{
    FHost H;
    std::vector<uint32_t> Words;
    std::vector<float2> Tan;
    std::vector<float4> Climate;
    double RadiusCm = 1.0e8;
    double SlopeN = 0.3;      // the synthetic bake's slope, rising to the north
    double Base = 1200.0;     // its height (m) at the reference
};

// The synthetic bake's height (m) at (U, V) metres from the reference.
static double GlacialBake(const FGlacialSetup& G, double Vm) { return G.Base + G.SlopeN * Vm; }

static void GlacialSetupAll(FGlacialSetup& G, const FChain& Glacial, double Cooling)
{
    // The peaks: the generated stand-in at 1024 (texel 0), then the glacial chain.
    VoxelNoiseTiles::FSet Peaks = VoxelNoiseTiles::GeneratePeaksStandIn(0x7065616bu, 1024, [](int Count, auto&& Body) { for (int i = 0; i < Count; i++) Body(i); });
    const uint32_t RowN = std::max(Peaks.N, Glacial.Header.N);
    auto Append = [&](const uint32_t* Data, uint32_t N) {
        const uint64_t Count = VoxelNoiseTiles::ChainTexels(N);
        const uint64_t Rows = (Count + RowN - 1) / RowN;
        const uint32_t Base = (uint32_t)(G.Words.size() / 2);
        G.Words.resize(G.Words.size() + 2 * Rows * RowN, 0u);
        memcpy(G.Words.data() + 2 * (size_t)Base, Data, Count * 8);
        return Base;
    };
    Append((const uint32_t*)Peaks.Texels.data(), Peaks.N);
    const uint32_t GBase = Append(Glacial.Words.data(), Glacial.Header.N);
    FHost& H = G.H;
    H.LandNoise = G.Words.data();
    H.LandN = 1024;
    H.LandPad = 4;
    G.Tan.resize(H.LandN + 2 * H.LandPad);
    for (size_t i = 0; i < G.Tan.size(); i++)
        G.Tan[i] = DF(std::tan((-1.0 + (2.0 * ((double)i - H.LandPad) + 1.0) / H.LandN) * 3.14159265358979323846 * 0.25));
    H.LandTanTable = G.Tan.data();
    H.LandCellM = (float)(3.14159265358979 * 0.5 * G.RadiusCm * 0.01 / H.LandN);
    H.LandHasForm = 1u;
    H.LandHasFlow = 0u;
    // The peaks at 6 km (as the plugin's defaults).
    auto UnitMeanSet = [](const VoxelNoiseTiles::FSet& S, const uint16_t* T, int c) {
        return VoxelNoiseTiles::MeanOf(S, T, c, [&](double V) { return (V - S.Channel[c].Min) / std::max((double)S.Channel[c].Max - S.Channel[c].Min, 1e-12); });
    };
    {
        const double TexelM = 6000.0 / (Peaks.N / Peaks.Channel[0].CellsPerTile);
        const float Range = std::max(Peaks.Channel[0].Max - Peaks.Channel[0].Min, Peaks.Channel[1].Max - Peaks.Channel[1].Min);
        H.LandPeakStrength = 1.0f;
        H.LandPeakTexelsPerCell = (float)(H.LandCellM / TexelM);
        H.LandPeakSeed = 99u;
        H.LandPeakSpacingM = 6000.0f;
        H.LandPeakReliefM = (float)(Range * TexelM);
        H.LandPeakMeanR = (float)UnitMeanSet(Peaks, (const uint16_t*)G.Words.data(), 0);
        H.LandPeakMeanG = (float)UnitMeanSet(Peaks, (const uint16_t*)G.Words.data(), 1);
        H.LandPeakScaleR = (Peaks.Channel[0].Max - Peaks.Channel[0].Min) / Range;
        H.LandPeakScaleG = (Peaks.Channel[1].Max - Peaks.Channel[1].Min) / Range;
        H.LandPeakScaleB = (Peaks.Channel[2].Max - Peaks.Channel[2].Min) / Range;
        H.LandNoiseN = Peaks.N;
        H.LandNoiseMips = Peaks.Mips;
    }
    {
        const auto& C = Glacial.Header.Channel;
        const double TexelM = 6000.0 / (Glacial.Header.N / C[0].CellsPerTile);
        const float Range = std::max(C[0].Max - C[0].Min, C[1].Max - C[1].Min);
        H.LandGlacialTexelsPerCell = (float)(H.LandCellM / TexelM);
        H.LandGlacialBase = GBase; H.LandGlacialN = Glacial.Header.N; H.LandGlacialMips = Glacial.Header.Mips;
        H.LandGlacialScaleR = (C[0].Max - C[0].Min) / Range;
        H.LandGlacialScaleG = (C[1].Max - C[1].Min) / Range;
        H.LandGlacialScaleB = (C[2].Max - C[2].Min) / Range;
        H.LandGlacialMeanR = (float)UnitMean(Glacial, 0);
        H.LandGlacialMeanG = (float)UnitMean(Glacial, 1);
        H.LandGlacialReliefM = (float)(Range * TexelM);
        H.LandGlacialStrength = 1.0f;
        H.LandGlacialSeed = 4242u;
        H.LandGlacialSpacingM = 6000.0f;
        H.LandGlacialCooling = (float)Cooling;
    }
    // A uniform climate: -2 C at sea level, 800 mm, an 8 C swing: the ice line near 840 m.
    const int W = VOXEL_LAND_CLIMATE_WIDTH, Hh = VOXEL_LAND_CLIMATE_HEIGHT;
    G.Climate.assign((size_t)2 * W * Hh, float4(0.0f, 0.0f, 0.0f, 0.0f));
    for (int i = 0; i < W * Hh; i++) G.Climate[i] = float4(-2.0f, 800.0f, 8.0f, 0.5f);
    H.LandClimate = G.Climate.data();
    H.LandHasClimate = 1u;
    H.LandClimateLapse = 6.5f;
    printf("glacial: %.0f m relief (peaks %.0f m), troughs every %.0f m, %.1f texels a cell; the glacial maximum %.1f C colder\n",
        H.LandGlacialReliefM, H.LandPeakReliefM, H.LandGlacialSpacingM, H.LandGlacialTexelsPerCell, Cooling);
}

static FVoxelLandPeaks GlacialEval(const FGlacialSetup& G, double Um, double Vm, float FootprintCm)
{
    FHost H = G.H;
    FSetup Dummy;
    Dummy.RadiusCm = G.RadiusCm;
    const FPoint P = At(Dummy, Um, Vm);
    // The bake's gradient: SlopeN to the north at P.
    const double L = std::sqrt(P.X * P.X + P.Y * P.Y + P.Z * P.Z);
    const double X = P.X / L, Y = P.Y / L, Z = P.Z / L;
    double EX = -Y, EY = X;
    const double EL = std::sqrt(EX * EX + EY * EY); EX /= EL; EY /= EL;
    const float3 North((float)(-Z * EY), (float)(Z * EX), (float)(X * EY - Y * EX));
    const float3 BakeGradient = North * (float)G.SlopeN;
    return H.VoxelLandPeaksShapeDF(DF(P.X), DF(P.Y), DF(P.Z), float2(0.9f, 0.6f), float3(0.0f, 0.0f, 0.0f), 2500.0f, 0.0f,
        (float)GlacialBake(G, Vm), BakeGradient, FootprintCm, true);
}

static int GlacialChecks(const char* Path, const std::string& Prefix)
{
    FChain Glacial;
    if (!ReadChain(Path, 1024, Glacial)) return 1;
    std::string Why = VoxelNoiseTiles::CheckPeaks(Glacial.Header);
    printf("CheckPeaks (glacial): %s\n", Why.empty() ? "ok" : Why.c_str());
    int Fails = Why.empty() ? 0 : 1;
    FGlacialSetup G;
    GlacialSetupAll(G, Glacial, 2.0);
    // 1. Up the slope (300 .. 1500 m of bake height: the glacial maximum's line near 530 m, the
    // ice line near 840 m), 0.25 m steps: no jumps.
    for (const double U : { 0.0, 1777.0 })
    {
        double MaxJump = 0.0, Where = 0.0, Sum = 0.0;
        const int Steps = 16000;
        auto H = [&](int i) { const double V = -3000.0 + i * 0.25; const FVoxelLandPeaks P = GlacialEval(G, U, V, 100.0f); return (double)P.Unit * P.AmplitudeM; };
        double Prev = H(0);
        for (int i = 1; i <= Steps; i++)
        {
            const double Hh = H(i);
            const double J = std::fabs(Hh - Prev);
            Sum += J;
            if (J > MaxJump) { MaxJump = J; Where = -3000.0 + i * 0.25; }
            Prev = Hh;
        }
        const double Mean = Sum / Steps;
        const bool bOk = MaxJump < 12.0 * Mean + 0.01;
        Fails += bOk ? 0 : 1;
        printf("up the slope at %.0f m: mean step %.4f m, largest %.4f m at %.1f m (bake %.0f m) %s\n", U, Mean, MaxJump, Where, GlacialBake(G, Where), bOk ? "ok" : "FAIL");
    }
    // 2. The gradient against differences (0.25 m), through the switch and the ice line; then on a
    // low coast (bake 0 .. 60 m), where the two layers' coast fades differ, the switch's middle
    // there (GlacialEraCooling 5.3 C).
    auto GradientCheck = [&](FGlacialSetup& G, double V0, double V1, const char* Name)
    {
        std::mt19937 Rng(5);
        std::uniform_real_distribution<double> PU(-4000.0, 4000.0), PV(V0, V1);
        double SumErr = 0.0, SumMag = 0.0;
        for (int i = 0; i < 2000; i++)
        {
            const double U = PU(Rng), V = PV(Rng), Hm = 0.25;
            const FVoxelLandPeaks D = GlacialEval(G, U, V, 100.0f);
            auto Hgt = [&](double u, double v) { const FVoxelLandPeaks P = GlacialEval(G, u, v, 100.0f); return (double)P.Unit * P.AmplitudeM; };
            const double HE = (Hgt(U + Hm, V) - Hgt(U - Hm, V)) / (2.0 * Hm);
            const double HN = (Hgt(U, V + Hm) - Hgt(U, V - Hm)) / (2.0 * Hm);
            // What the hosts add: Unit times the amplitude's change (by differences).
            auto Amp = [&](double u, double v) { return (double)GlacialEval(G, u, v, 100.0f).AmplitudeM; };
            const double AE = (Amp(U + Hm, V) - Amp(U - Hm, V)) / (2.0 * Hm) * D.Unit;
            const double AN = (Amp(U, V + Hm) - Amp(U, V - Hm)) / (2.0 * Hm) * D.Unit;
            FSetup Dummy; Dummy.RadiusCm = G.RadiusCm;
            const FPoint P = At(Dummy, U, V);
            const double L = std::sqrt(P.X * P.X + P.Y * P.Y + P.Z * P.Z);
            const double X = P.X / L, Y = P.Y / L, Z = P.Z / L;
            double EX = -Y, EY = X;
            const double EL = std::sqrt(EX * EX + EY * EY); EX /= EL; EY /= EL;
            const double NX = -Z * EY, NY = Z * EX, NZ = X * EY - Y * EX;
            const double GE = D.Gradient.x * EX + D.Gradient.y * EY + AE;
            const double GN = D.Gradient.x * NX + D.Gradient.y * NY + D.Gradient.z * NZ + AN;
            SumErr += std::sqrt((GE - HE) * (GE - HE) + (GN - HN) * (GN - HN));
            SumMag += std::sqrt(HE * HE + HN * HN);
        }
        printf("%s: gradient (+ Unit x the amplitude's change, as the hosts add it) vs differences: mean error %.4f of the mean slope %.4f\n",
            Name, SumErr / 2000.0, SumMag / 2000.0);
        const bool bOk = SumErr / SumMag < 0.02;
        Fails += bOk ? 0 : 1;
        printf("  %s\n", bOk ? "ok" : "FAIL");
    };
    GradientCheck(G, -3000.0, 1000.0, "the slope");
    {
        FGlacialSetup C;
        GlacialSetupAll(C, Glacial, 5.3);
        C.Base = 25.0;
        C.SlopeN = 0.02;
        GradientCheck(C, -1200.0, 1700.0, "the coast");
    }
    // 3. The range against PeakReach's rule: sqrt 2 x sqrt 3 x the larger relief (no detail here).
    {
        std::mt19937 Rng(9);
        std::uniform_real_distribution<double> PU(-40000.0, 40000.0), PV(-4000.0, 2000.0);
        double Lo = 1e9, Hi = -1e9;
        for (int i = 0; i < 100000; i++)
        {
            const FVoxelLandPeaks P = GlacialEval(G, PU(Rng), PV(Rng), 100.0f);
            const double Hh = (double)P.Unit * P.AmplitudeM;
            Lo = std::min(Lo, Hh); Hi = std::max(Hi, Hh);
        }
        const double Bound = 1.01 * std::sqrt(2.0) * std::sqrt(3.0) * std::max(G.H.LandGlacialReliefM, G.H.LandPeakReliefM);
        const bool bOk = Hi <= Bound && Lo >= -Bound;
        Fails += bOk ? 0 : 1;
        printf("range: %.0f .. %.0f m; the bound +-%.0f m %s\n", Lo, Hi, Bound, bOk ? "ok" : "FAIL");
    }
    // 4. Looks: 16 km square, 25 m cells, the slope's bake under it: the peaks low down, the
    // carved bedrock above the glacial maximum's line, the ice above today's.
    {
        const int W = 640;
        const double Size = 16000.0;
        std::vector<float> Hgt((size_t)W * W);
        for (int y = 0; y < W; y++)
            for (int x = 0; x < W; x++)
            {
                const double U = (x + 0.5) / W * Size - 0.5 * Size, V = 0.5 * Size - (y + 0.5) / W * Size - 2000.0;
                const FVoxelLandPeaks P = GlacialEval(G, U, V, 1250.0f);
                Hgt[(size_t)y * W + x] = (float)(GlacialBake(G, V) + (double)P.Unit * P.AmplitudeM);
            }
        WritePPM(Prefix + "_glacial.ppm", W, W, Hgt, Size / W);
        printf("look: %s_glacial.ppm (north up: the bake rises from %.0f m at the bottom to %.0f m at the top)\n", Prefix.c_str(),
            GlacialBake(G, -0.5 * Size - 2000.0), GlacialBake(G, 0.5 * Size - 2000.0));
    }
    return Fails;
}

// --- T5d2: the ice sheets ----------------------------------------------------------------------

static double IceLand(double U, double V) { return 500.0 + 450.0 * std::sin(U / 1700.0) * std::cos(V / 1300.0) + 0.02 * V; }
static void IceLandGradient(double U, double V, double& GE, double& GN)
{
    GE = 450.0 / 1700.0 * std::cos(U / 1700.0) * std::cos(V / 1300.0);
    GN = -450.0 / 1300.0 * std::sin(U / 1700.0) * std::sin(V / 1300.0) + 0.02;
}

static int IceChecks(const char* Path, const std::string& Prefix)
{
    FChain Ice;
    if (!ReadChain(Path, 1024, Ice)) return 1;
    std::string Why = VoxelNoiseTiles::CheckPeaks(Ice.Header);
    printf("CheckPeaks (icesheet): %s\n", Why.empty() ? "ok" : Why.c_str());
    int Fails = Why.empty() ? 0 : 1;
    FSetup S;
    S.RadiusCm = 1.0e8;
    FHost& H = S.H;
    S.Words = Ice.Words;
    H.LandNoise = S.Words.data();
    H.LandN = 1024;
    H.LandPad = 4;
    S.Tan.resize(H.LandN + 2 * H.LandPad);
    for (size_t i = 0; i < S.Tan.size(); i++)
        S.Tan[i] = DF(std::tan((-1.0 + (2.0 * ((double)i - H.LandPad) + 1.0) / H.LandN) * 3.14159265358979323846 * 0.25));
    H.LandTanTable = S.Tan.data();
    H.LandCellM = (float)(3.14159265358979 * 0.5 * S.RadiusCm * 0.01 / H.LandN);
    // The climate: the ice block, thickening east of a margin near the reference (longitude 17.2).
    const int W = VOXEL_LAND_CLIMATE_WIDTH, Hh = VOXEL_LAND_CLIMATE_HEIGHT;
    std::vector<float4> Climate((size_t)3 * W * Hh, float4(0.0f, 0.0f, 0.0f, 0.0f));
    for (int J = 0; J < Hh; J++)
        for (int I = 0; I < W; I++)
        {
            // The signed distance into the ice: east of longitude 16.8 (m along the latitude).
            const double Lon = (I + 0.5) / W * 360.0;
            const double Mu = VoxelLandClimateGrid::RowMu(J, Hh);
            const double Cos = std::sqrt(std::max(1.0 - Mu * Mu, 0.0));
            Climate[(size_t)2 * W * Hh + (size_t)J * W + I] = float4((float)((Lon - 16.8) * 3.14159265358979 / 180.0 * S.RadiusCm * 0.01 * Cos), 300.0f, 0.0f, 0.0f);
        }
    H.LandClimate = Climate.data();
    H.LandHasClimate = 1u;
    H.LandIceMaxThickness = 2000.0f;
    H.LandIceBlend = 60.0f;
    {
        const auto& G = Ice.Header.Channel[1];
        const double TexelM = 30000.0 / (Ice.Header.N / G.CellsPerTile);
        H.LandIceSwellTexelsPerCell = (float)(H.LandCellM / TexelM);
        H.LandIceBase = 0u; H.LandIceN = Ice.Header.N; H.LandIceMips = Ice.Header.Mips;
        H.LandIceMeanG = (float)UnitMean(Ice, 1);
        H.LandIceSwellM = 60.0f;
        H.LandIceSeed = 777u;
        H.LandIceSwellSpacingM = 30000.0f;
    }
    // The surface at (U, V): the land through the ice.
    auto Eval = [&](double U, double V, float3* OutGrad) {
        FHost Hc = H;
        const FPoint P = At(S, U, V);
        const double L = std::sqrt(P.X * P.X + P.Y * P.Y + P.Z * P.Z);
        const float3 N((float)(P.X / L), (float)(P.Y / L), (float)(P.Z / L));
        const float3 Zero(0.0f, 0.0f, 0.0f);
        const double A = IceLand(U, V);
        double EX = -N.y, EY = N.x;
        const double EL = std::sqrt(EX * EX + EY * EY); EX /= EL; EY /= EL;
        const float3 East((float)EX, (float)EY, 0.0f);
        const float3 North((float)(-N.z * EY), (float)(N.z * EX), (float)(N.x * EY - N.y * EX));
        double GE, GN;
        IceLandGradient(U, V, GE, GN);
        const float3 LandGrad = East * (float)GE + North * (float)GN;
        // The land is the bake here (its height and gradient, the land gate's).
        const FVoxelLandIceSurface R = Hc.VoxelLandIceSurfaceDF(DF(P.X), DF(P.Y), DF(P.Z), Zero, Zero, Zero, N, (float)A, (float)A, (float)A, LandGrad, 100.0f, true);
        if (OutGrad)
            *OutGrad = LandGrad * R.Scale + R.Gradient;
        return (double)R.Height;
    };
    // 1. Across the margin (east-west) and over the nunataks, 0.25 m steps: no jumps.
    {
        double MaxJump = 0.0, Where = 0.0, Sum = 0.0;
        const int Steps = 40000;
        double Prev = Eval(-5000.0, 300.0, nullptr);
        for (int i = 1; i <= Steps; i++)
        {
            const double U = -5000.0 + i * 0.25;
            const double Hgt = Eval(U, 300.0, nullptr);
            const double J = std::fabs(Hgt - Prev);
            Sum += J;
            if (J > MaxJump) { MaxJump = J; Where = U; }
            Prev = Hgt;
        }
        const double Mean = Sum / Steps;
        const bool bOk = MaxJump < 12.0 * Mean + 0.01;
        Fails += bOk ? 0 : 1;
        printf("ice, across the margin: mean step %.4f m, largest %.4f m at %.1f m %s\n", Mean, MaxJump, Where, bOk ? "ok" : "FAIL");
    }
    // 2. The gradient against differences.
    {
        std::mt19937 Rng(13);
        std::uniform_real_distribution<double> PU(-5000.0, 5000.0), PV(-5000.0, 5000.0);
        double SumErr = 0.0, SumMag = 0.0;
        for (int i = 0; i < 3000; i++)
        {
            // 2 m: the heights are floats near 1000 m (a 0.25 m difference is mostly their rounding).
            const double U = PU(Rng), V = PV(Rng), Hm = 2.0;
            float3 G;
            Eval(U, V, &G);
            const double HE = (Eval(U + Hm, V, nullptr) - Eval(U - Hm, V, nullptr)) / (2.0 * Hm);
            const double HN = (Eval(U, V + Hm, nullptr) - Eval(U, V - Hm, nullptr)) / (2.0 * Hm);
            const FPoint P = At(S, U, V);
            const double L = std::sqrt(P.X * P.X + P.Y * P.Y + P.Z * P.Z);
            const double X = P.X / L, Y = P.Y / L, Z = P.Z / L;
            double EX = -Y, EY = X;
            const double EL = std::sqrt(EX * EX + EY * EY); EX /= EL; EY /= EL;
            const double NX = -Z * EY, NY = Z * EX, NZ = X * EY - Y * EX;
            const double GE = G.x * EX + G.y * EY, GN = G.x * NX + G.y * NY + G.z * NZ;
            const double Err = std::sqrt((GE - HE) * (GE - HE) + (GN - HN) * (GN - HN));
            SumErr += Err;
            SumMag += std::sqrt(HE * HE + HN * HN);
            if (std::getenv("ICE_CREASE") && Err > 0.01) printf("  err %.4f at (%.0f, %.0f): analytic (%.4f, %.4f) diff (%.4f, %.4f)\n", Err, U, V, GE, GN, HE, HN);
        }
        const bool bOk = SumErr / SumMag < 0.02;
        Fails += bOk ? 0 : 1;
        printf("ice, gradient vs differences: mean error %.4f of the mean slope %.4f %s\n", SumErr / 3000.0, SumMag / 3000.0, bOk ? "ok" : "FAIL");
    }
    // Debug: the slope's largest jump along a row (a crease).
    if (std::getenv("ICE_CREASE"))
    {
        double Prev = Eval(-6000.0, 0.0, nullptr), PrevD = 0.0, Worst = 0.0, Where = 0.0;
        for (int i = 1; i < 12000; i++)
        {
            const double U = -6000.0 + i;
            const double Hc = Eval(U, 0.0, nullptr);
            const double D = Hc - Prev;
            if (i > 1 && std::fabs(D - PrevD) > Worst) { Worst = std::fabs(D - PrevD); Where = U; }
            PrevD = D; Prev = Hc;
        }
        printf("crease: largest slope change %.5f at %.0f m\n", Worst, Where);
        for (double U = Where - 4.0; U <= Where + 4.0; U += 1.0) printf("  %.0f: %.4f\n", U, Eval(U, 0.0, nullptr));
    }
    // 3. A look: 12 km square, the margin left of the middle, the sheet thickening eastward.
    {
        const int Wd = 600;
        const double Size = 12000.0;
        std::vector<float> Hgt((size_t)Wd * Wd);
        for (int y = 0; y < Wd; y++)
            for (int x = 0; x < Wd; x++)
            {
                const double U = (x + 0.5) / Wd * Size - 0.5 * Size, V = 0.5 * Size - (y + 0.5) / Wd * Size;
                Hgt[(size_t)y * Wd + x] = (float)Eval(U, V, nullptr);
            }
        WritePPM(Prefix + "_ice.ppm", Wd, Wd, Hgt, Size / Wd);
        printf("look: %s_ice.ppm\n", Prefix.c_str());
    }
    return Fails;
}

// --- T5d3: the polar surfaces -------------------------------------------------------------------

// The polar host: the set's chain, a uniform climate (cold, a glacier line on a slope), the drift.
struct FPolarSetup
{
    FSetup S;
    std::vector<float4> Climate;
    double Spacing[3] = { 60.0, 4.0, 20.0 };
    double Reach[2] = { 0.0, 0.0 };   // up, down (m): VoxelLand::PolarReach's rule
};

static void PolarSetupAll(FPolarSetup& PS, const FChain& Polar, const FChain& Dunes)
{
    FSetup& S = PS.S;
    FHost& H = S.H;
    S.RadiusCm = 1.0e8;
    S.Words = Polar.Words;
    // The dune set's chain after it (the snow dunes).
    const uint32_t DuneBase = (uint32_t)(S.Words.size() / 2);
    S.Words.insert(S.Words.end(), Dunes.Words.begin(), Dunes.Words.end());
    H.LandNoise = S.Words.data();
    H.LandN = 1024;
    H.LandPad = 4;
    S.Tan.resize(H.LandN + 2 * H.LandPad);
    for (size_t i = 0; i < S.Tan.size(); i++)
        S.Tan[i] = DF(std::tan((-1.0 + (2.0 * ((double)i - H.LandPad) + 1.0) / H.LandN) * 3.14159265358979323846 * 0.25));
    H.LandTanTable = S.Tan.data();
    H.LandCellM = (float)(3.14159265358979 * 0.5 * S.RadiusCm * 0.01 / H.LandN);
    H.LandPolarBase = 0u; H.LandPolarN = Polar.Header.N; H.LandPolarMips = Polar.Header.Mips;
    H.LandPolarSeed = 4242u;
    float* TPC[3] = { &H.LandPolarCrevasseTexelsPerCell, &H.LandPolarSastrugiTexelsPerCell, &H.LandPolarPolygonTexelsPerCell };
    float* M[3] = { &H.LandPolarCrevasseM, &H.LandPolarSastrugiM, &H.LandPolarPolygonM };
    float* Mean[3] = { &H.LandPolarCrevasseMean, &H.LandPolarSastrugiMean, &H.LandPolarPolygonMean };
    float* Fade[3] = { &H.LandPolarCrevasseFade, &H.LandPolarSastrugiFade, &H.LandPolarPolygonFade };
    for (int c = 0; c < 3; c++)
    {
        const auto& C = Polar.Header.Channel[c];
        const double ST = (double)Polar.Header.N / C.CellsPerTile;
        const double TexelM = PS.Spacing[c] / ST;
        *TPC[c] = (float)(H.LandCellM / TexelM);
        const double TexelF = (double)H.LandCellM / (double)*TPC[c];
        *M[c] = (float)((C.Max - C.Min) * TexelF);
        *Mean[c] = (float)UnitMean(Polar, c);
        *Fade[c] = (float)std::log2(std::max(ST, 2.0));
        PS.Reach[0] += (C.Max - C.Min) * TexelM * (1.0 - *Mean[c]);
        PS.Reach[1] += (C.Max - C.Min) * TexelM * *Mean[c];
        printf("polar %c: cell %.1f texels, %.3f m texels, %.2f m range, mean %.3f, fade level %.2f\n", "RGB"[c], ST, TexelF, *M[c], *Mean[c], *Fade[c]);
    }
    // The snow dunes: the dune set's G, 8 m at 400 m (the defaults).
    {
        const auto& G = Dunes.Header.Channel[1];
        const double ST = (double)Dunes.Header.N / Dunes.Header.Channel[0].CellsPerTile;
        const double TexelM = 400.0 / ST;
        H.LandPolarDuneBase = DuneBase; H.LandPolarDuneN = Dunes.Header.N; H.LandPolarDuneMips = Dunes.Header.Mips;
        H.LandPolarDuneTexelsPerCell = (float)(H.LandCellM / TexelM);
        H.LandPolarDuneM = 8.0f;
        H.LandPolarDuneMean = (float)UnitMean(Dunes, 1);
        H.LandPolarDuneFade = (float)std::log2(std::max(ST, 2.0));
        PS.Reach[0] += 8.0 * (1.0 - H.LandPolarDuneMean);
        PS.Reach[1] += 8.0 * H.LandPolarDuneMean;
        printf("snow dunes: %.1f texels a crest, %.2f m texels, mean %.3f (G range %.1f texels)\n", ST, TexelM, H.LandPolarDuneMean, G.Max - G.Min);
    }
    PS.Reach[0] *= 1.01 * 2.449489742783178;
    PS.Reach[1] *= 1.01 * 2.449489742783178;
    H.LandPolarPermafrostT = -6.0f;
    // The grains, as the pack measures them.
    H.LandPolarCrevasseTurn = (float)VoxelNoiseTiles::GrainAngleOf(Polar.Header, (const uint16_t*)Polar.Words.data(), 0);
    double Axis = VoxelNoiseTiles::GrainAngleOf(Polar.Header, (const uint16_t*)Polar.Words.data(), 1) - 0.5 * 3.14159265358979;
    if (Axis <= -0.5 * 3.14159265358979) Axis += 3.14159265358979;
    H.LandPolarSastrugiTurn = (float)Axis;
    printf("polar grains: crevasses' normal %.1f deg, sastrugi's axis %.1f deg\n", H.LandPolarCrevasseTurn * 57.29578, H.LandPolarSastrugiTurn * 57.29578);
    // The climate: every cell -1 C at sea level, 500 mm, a swing of 8 C (the glacier line near
    // 1000 m), a drift to the east at steadiness 0.6. No ice block (the harness passes the sheet).
    const int W = VOXEL_LAND_CLIMATE_WIDTH, Hh = VOXEL_LAND_CLIMATE_HEIGHT;
    PS.Climate.assign((size_t)3 * W * Hh, float4(0.0f, 0.0f, 0.0f, 0.0f));
    for (size_t i = 0; i < (size_t)W * Hh; i++)
    {
        PS.Climate[i] = float4(-1.0f, 500.0f, 8.0f, 0.5f);
        PS.Climate[(size_t)W * Hh + i] = float4(1.0f, 0.0f, 0.6f, 0.0f);
    }
    H.LandClimate = PS.Climate.data();
    H.LandHasClimate = 1u;
    H.LandClimateLapse = 6.5f;
}

// East and north at a point.
static void EastNorth(const FPoint& P, float3& N, float3& East, float3& North)
{
    const double L = std::sqrt(P.X * P.X + P.Y * P.Y + P.Z * P.Z);
    N = float3((float)(P.X / L), (float)(P.Y / L), (float)(P.Z / L));
    double EX = -N.y, EY = N.x;
    const double EL = std::sqrt(EX * EX + EY * EY); EX /= EL; EY /= EL;
    East = float3((float)EX, (float)EY, 0.0f);
    North = float3((float)(-N.z * EY), (float)(N.z * EX), (float)(N.x * EY - N.y * EX));
}

// The synthetic ground: a bake rising north at 0.1 (the glacier line across it), an ice sheet's
// fade rising east over 4 km (300 m thick, its own slope 0.05 west-down... east-up), the steering
// the bake's. Its polar surfaces at (U, V), and their gradient (east, north).
// The sheet's own slope in the synthetic ground (0.012: its flat interior, snow dunes; 0.05: its
// marginal crevasse zone).
static double GPolarSheetSlope = 0.012;

static FHost::FVoxelLandPolarSite PolarSiteAt(const FPolarSetup& PS, double U, double V, bool bGradients)
{
    FHost Hc = PS.S.H;
    const FPoint P = At(PS.S, U, V);
    float3 N, East, North;
    EastNorth(P, N, East, North);
    const float Bake = (float)(1000.0 + 0.1 * V);
    const float3 BakeGrad = North * 0.1f;
    const double T = std::clamp((U + 2000.0) / 4000.0, 0.0, 1.0);
    const float Fade = (float)(T * T * (3.0 - 2.0 * T));
    const float3 FadeGrad = East * (float)(6.0 * T * (1.0 - T) / 4000.0);
    const float3 IceSlope = East * (float)GPolarSheetSlope;
    // The surface: the bake plus the sheet's 300 m where it covers.
    return Hc.VoxelLandPolarSiteAt(N, Bake, BakeGrad, BakeGrad, Bake + 300.0f * Fade, Fade, IceSlope, FadeGrad, 1.0f, bGradients);
}

static double PolarEval(const FPolarSetup& PS, double U, double V, float Footprint, double* GE, double* GN)
{
    FHost Hc = PS.S.H;
    const FPoint P = At(PS.S, U, V);
    const FHost::FVoxelLandPolarSite Site = PolarSiteAt(PS, U, V, GE != nullptr);
    const FHost::FVoxelLandPolar R = Hc.VoxelLandPolarDF(DF(P.X), DF(P.Y), DF(P.Z), Site, Footprint, GE != nullptr);
    if (GE)
    {
        float3 N, East, North;
        EastNorth(P, N, East, North);
        *GE = dot(R.Gradient, East);
        *GN = dot(R.Gradient, North);
    }
    return R.Height;
}

// A fixed site: one feature at full share, its direction at AngleDeg from east.
static FHost::FVoxelLandPolarSite PolarFixed(const FPoint& P, int Feature, double AngleDeg)
{
    float3 N, East, North;
    EastNorth(P, N, East, North);
    const double A = AngleDeg * 3.14159265358979 / 180.0;
    const float3 D = East * (float)std::cos(A) + North * (float)std::sin(A);
    FHost::FVoxelLandPolarSite S;
    S.Crevasse = Feature == 0 ? 1.0f : 0.0f;
    S.Sastrugi = Feature == 1 ? 1.0f : 0.0f;
    S.Polygon = Feature == 2 ? 1.0f : 0.0f;
    S.SnowDune = Feature == 3 ? 1.0f : 0.0f;
    S.Flow = D;
    S.Wind = D;
    S.CrevasseGradient = S.SastrugiGradient = S.PolygonGradient = S.SnowDuneGradient = float3(0.0f, 0.0f, 0.0f);
    return S;
}

static int PolarChecks(const char* Path, const FChain& Dunes, const std::string& Prefix)
{
    FChain Polar;
    if (!ReadChain(Path, 1024, Polar)) return 1;
    const std::string Why = VoxelNoiseTiles::CheckPolar(Polar.Header);
    printf("CheckPolar: %s\n", Why.empty() ? "ok" : Why.c_str());
    int Fails = Why.empty() ? 0 : 1;
    FPolarSetup PS;
    PolarSetupAll(PS, Polar, Dunes);
    printf("polar reach: up %.2f m, down %.2f m\n", PS.Reach[0], PS.Reach[1]);
    const char* Names[4] = { "crevasses", "sastrugi", "polygons", "snow dunes" };
    // 1. Direction sweeps (the turns either side) for the directed features: largest step in 0.02 deg.
    for (int f = 0; f < 4; f++)
    {
        if (f == 2) continue;
        FHost Hc = PS.S.H;
        const FPoint P = At(PS.S, 137.0, -61.0);
        double Prev = 0.0, MaxStep = 0.0, Range = 0.0;
        for (int i = 0; i <= 18000; i++)
        {
            const double H = Hc.VoxelLandPolarDF(DF(P.X), DF(P.Y), DF(P.Z), PolarFixed(P, f, i * 0.02), 5.0f, false).Height;
            if (i > 0) MaxStep = std::max(MaxStep, std::fabs(H - Prev));
            Range = std::max(Range, std::fabs(H));
            Prev = H;
        }
        const double M = f == 0 ? Hc.LandPolarCrevasseM : (f == 1 ? Hc.LandPolarSastrugiM : Hc.LandPolarDuneM);
        const bool bOk = MaxStep < 0.01 * M;
        Fails += bOk ? 0 : 1;
        printf("%s, direction sweep: largest step %.5f m (of %.2f m range) %s\n", Names[f], MaxStep, M, bOk ? "ok" : "FAIL");
    }
    // 2. Lines through each feature at full share in small steps: no seams (largest step against the mean).
    for (int f = 0; f < 4; f++)
    {
        FHost Hc = PS.S.H;
        const double Step = f == 1 ? 0.02 : (f == 3 ? 1.0 : 0.1);
        const int Steps = 40000;
        double Prev = 0.0, Sum = 0.0, MaxJ = 0.0;
        for (int i = 0; i <= Steps; i++)
        {
            const FPoint P = At(PS.S, -1500.0 + i * Step * 0.8, 300.0 + i * Step * 0.6);
            const double H = Hc.VoxelLandPolarDF(DF(P.X), DF(P.Y), DF(P.Z), PolarFixed(P, f, 25.0), 2.0f, false).Height;
            if (i > 0) { const double J = std::fabs(H - Prev); Sum += J; MaxJ = std::max(MaxJ, J); }
            Prev = H;
        }
        const double Mean = Sum / Steps;
        // Crevasse walls are near vertical: a step there is the wall's slope times the step.
        const double Slope = MaxJ / Step;
        const bool bOk = Slope < 20.0;
        Fails += bOk ? 0 : 1;
        printf("%s, %.2f m steps: mean %.5f m, largest %.5f m (slope %.2f) %s\n", Names[f], Step, Mean, MaxJ, Slope, bOk ? "ok" : "FAIL");
    }
    // 3. The gradient (the shape's and the shares' known changes) against differences, over the
    // synthetic ground (the glacier line, the sheet's margin, the permafrost's ramp): its flat
    // interior, then its marginal crevasse zone.
    for (const double SheetSlope : { 0.012, 0.05 })
    {
        GPolarSheetSlope = SheetSlope;
        printf("the sheet's own slope %.3f:\n", SheetSlope);
        std::mt19937 Rng(7);
        std::uniform_real_distribution<double> PU(-3000.0, 3000.0), PV(-4000.0, 4000.0);
        double SumErr[4] = { 0, 0, 0, 0 }, SumMag[4] = { 0, 0, 0, 0 };
        int Count[4] = { 0, 0, 0, 0 };
        const int N = 4000;
        for (int i = 0; i < N; i++)
        {
            const double U = PU(Rng), V = PV(Rng), Hm = 0.02;
            double GE, GN;
            PolarEval(PS, U, V, 2.0f, &GE, &GN);
            const double HE = (PolarEval(PS, U + Hm, V, 2.0f, nullptr, nullptr) - PolarEval(PS, U - Hm, V, 2.0f, nullptr, nullptr)) / (2.0 * Hm);
            const double HN = (PolarEval(PS, U, V + Hm, 2.0f, nullptr, nullptr) - PolarEval(PS, U, V - Hm, 2.0f, nullptr, nullptr)) / (2.0 * Hm);
            const FHost::FVoxelLandPolarSite Site = PolarSiteAt(PS, U, V, false);
            const bool bIn[4] = { Site.Crevasse > 0.0f, Site.Sastrugi > 0.0f, Site.Polygon > 0.0f && Site.Crevasse <= 0.0f && Site.Sastrugi <= 0.0f,
                Site.SnowDune > 0.0f };
            for (int k = 0; k < 4; k++)
            {
                if (!bIn[k]) continue;
                SumErr[k] += std::sqrt((GE - HE) * (GE - HE) + (GN - HN) * (GN - HN));
                SumMag[k] += std::sqrt(HE * HE + HN * HN);
                Count[k]++;
            }
        }
        const char* Where[4] = { "with crevasses", "with sastrugi", "polygons only", "with snow dunes" };
        for (int k = 0; k < 4; k++)
        {
            if (Count[k] == 0) { printf("  gradient %s: no samples\n", Where[k]); if (k != 3 || SheetSlope < 0.02) Fails++; continue; }
            const double Rel = SumErr[k] / std::max(SumMag[k], 1e-12);
            const bool bOk = Rel < 0.02;
            Fails += bOk ? 0 : 1;
            printf("  gradient %s (%d samples): mean error %.5f of the mean slope %.4f (%.2f%%) %s\n", Where[k], Count[k], SumErr[k] / Count[k],
                SumMag[k] / Count[k], 100.0 * Rel, bOk ? "ok" : "FAIL");
        }
    }
    GPolarSheetSlope = 0.012;
    // 3b. Nothing on the sea floor (the review's case): a bed 30 m under a sea cold enough to ice
    // its coast, flat and steep.
    {
        FHost Hc = PS.S.H;
        const FPoint P = At(PS.S, 0.0, 0.0);
        float3 N, East, North;
        EastNorth(P, N, East, North);
        // Colder cells: the glacier line under sea level.
        std::vector<float4> Cold = PS.Climate;
        for (size_t i = 0; i < (size_t)VOXEL_LAND_CLIMATE_WIDTH * VOXEL_LAND_CLIMATE_HEIGHT; i++) Cold[i].x = -15.0f;
        Hc.LandClimate = Cold.data();
        double Worst = 0.0;
        for (const float Slope : { 0.0f, 0.2f })
        {
            const FHost::FVoxelLandPolarSite S = Hc.VoxelLandPolarSiteAt(N, -30.0f, North * Slope, North * Slope, -30.0f, 0.0f, float3(0.0f, 0.0f, 0.0f),
                float3(0.0f, 0.0f, 0.0f), 1.0f, false);
            Worst = std::max(Worst, (double)std::max(std::max(S.Crevasse, S.Sastrugi), std::max(S.SnowDune, S.Polygon)));
        }
        const FHost::FVoxelLandPolarSite OnLand = Hc.VoxelLandPolarSiteAt(N, 200.0f, North * 0.2f, North * 0.2f, 200.0f, 0.0f, float3(0.0f, 0.0f, 0.0f),
            float3(0.0f, 0.0f, 0.0f), 1.0f, false);
        const bool bOk = Worst == 0.0 && OnLand.Crevasse > 0.0f;
        Fails += bOk ? 0 : 1;
        printf("the sea floor under a cold sea: largest share %.3f (on land beside it, crevasses %.2f) %s\n", Worst, OnLand.Crevasse, bOk ? "ok" : "FAIL");
    }
    // 4. The range against the bound, all features at full share, random directions and points.
    {
        std::mt19937 Rng(99);
        std::uniform_real_distribution<double> PU(-200000.0, 200000.0), PA(0.0, 360.0);
        double Lo = 0.0, Hi = 0.0;
        FHost Hc = PS.S.H;
        for (int i = 0; i < 200000; i++)
        {
            const FPoint P = At(PS.S, PU(Rng), PU(Rng));
            FHost::FVoxelLandPolarSite S = PolarFixed(P, 0, PA(Rng));
            S.Sastrugi = 1.0f;
            S.Polygon = 1.0f;
            S.SnowDune = 1.0f;
            const FHost::FVoxelLandPolarSite S2 = PolarFixed(P, 1, PA(Rng));
            S.Wind = S2.Wind;
            const double H = Hc.VoxelLandPolarDF(DF(P.X), DF(P.Y), DF(P.Z), S, 2.0f, false).Height;
            Lo = std::min(Lo, H); Hi = std::max(Hi, H);
        }
        const bool bOk = Hi <= PS.Reach[0] && -Lo <= PS.Reach[1];
        Fails += bOk ? 0 : 1;
        printf("range, all three at full share: %.2f .. %.2f m within -%.2f .. %.2f %s\n", Lo, Hi, PS.Reach[1], PS.Reach[0], bOk ? "ok" : "FAIL");
    }
    // 5. The footprint fade: by the fade level, nothing; a coarse footprint costs no lookups.
    {
        FHost Hc = PS.S.H;
        const FPoint P = At(PS.S, 11.0, 7.0);
        FHost::FVoxelLandPolarSite S = PolarFixed(P, 0, 10.0);
        S.Sastrugi = 1.0f; S.Polygon = 1.0f; S.SnowDune = 1.0f;
        const double Far = Hc.VoxelLandPolarDF(DF(P.X), DF(P.Y), DF(P.Z), S, 1.0e6f, false).Height;
        const bool bOk = Far == 0.0;
        Fails += bOk ? 0 : 1;
        printf("at a 10 km footprint: %.6f m %s\n", Far, bOk ? "ok" : "FAIL");
    }
    // 6. Looks: each feature alone, then the synthetic ground's transitions.
    auto Look = [&](const char* Name, double Size, int Wd, int Feature, double Angle) {
        std::vector<float> Hgt((size_t)Wd * Wd);
        FHost Hc = PS.S.H;
        for (int y = 0; y < Wd; y++)
            for (int x = 0; x < Wd; x++)
            {
                const double U = (x + 0.5) / Wd * Size - 0.5 * Size, V = 0.5 * Size - (y + 0.5) / Wd * Size;
                const FPoint P = At(PS.S, U, V);
                Hgt[(size_t)y * Wd + x] = Feature < 0 ? (float)PolarEval(PS, U, V, (float)(Size / Wd * 100.0), nullptr, nullptr)
                    : (float)Hc.VoxelLandPolarDF(DF(P.X), DF(P.Y), DF(P.Z), PolarFixed(P, Feature, Angle), (float)(Size / Wd * 100.0), false).Height;
            }
        const std::string File = Prefix + "_polar_" + Name + ".ppm";
        WritePPM(File, Wd, Wd, Hgt, Size / Wd);
        printf("look: %s\n", File.c_str());
    };
    Look("crevasses", 800.0, 600, 0, 30.0);
    Look("sastrugi", 60.0, 600, 1, 30.0);
    Look("polygons", 200.0, 600, 2, 0.0);
    Look("snowdunes", 3000.0, 600, 3, 30.0);
    // The synthetic ground's shares (red crevasses, green sastrugi, blue polygons): the glacier line
    // across the middle (north up), the ice sheet's margin from x -2 km to 2 km.
    {
        const int Wd = 400;
        const double Size = 8000.0;
        FILE* F = fopen((Prefix + "_polar_shares.ppm").c_str(), "wb");
        fprintf(F, "P6\n%d %d\n255\n", Wd, Wd);
        for (int y = 0; y < Wd; y++)
            for (int x = 0; x < Wd; x++)
            {
                const double U = (x + 0.5) / Wd * Size - 0.5 * Size, V = 0.5 * Size - (y + 0.5) / Wd * Size;
                const FHost::FVoxelLandPolarSite Site = PolarSiteAt(PS, U, V, false);
                const unsigned char C[3] = { (unsigned char)(255 * Site.Crevasse), (unsigned char)(255 * std::max(Site.Sastrugi, Site.SnowDune)), (unsigned char)(255 * Site.Polygon) };
                fwrite(C, 1, 3, F);
            }
        fclose(F);
        printf("look: %s_polar_shares.ppm\n", Prefix.c_str());
    }
    return Fails;
}

int main(int argc, char** argv)
{
    if (argc < 4) { printf("dune_check dunes.vxnt sanddetail.vxnt OutPrefix\n"); return 1; }
    FChain Dunes, Sand;
    if (!ReadChain(argv[1], 1024, Dunes) || !ReadChain(argv[2], 1024, Sand)) return 1;
    std::string Why = VoxelNoiseTiles::CheckDunes(Dunes.Header);
    printf("CheckDunes: %s\n", Why.empty() ? "ok" : Why.c_str());
    Why = VoxelNoiseTiles::CheckSandDetail(Sand.Header);
    printf("CheckSandDetail: %s\n", Why.empty() ? "ok" : Why.c_str());
    FSetup S;
    Setup(S, Dunes, Sand, true);
    int Fails = 0;

    // 1. Continuity over the wind's direction: a fixed point, the angle in 0.02 degree steps.
    for (const double Steady : { 0.9, 0.5, 0.25, 0.05 })
    {
        const FPoint P = At(S, 123.0, -77.0);
        double MaxJump = 0.0, Where = 0.0, Lo = 1e9, Hi = -1e9;
        float Prev = Eval(S, P, Site(P, 0.0, Steady), 100.0f).Unit;
        for (int i = 1; i <= 18000; i++)
        {
            const double A = i * 0.02;
            const float U = Eval(S, P, Site(P, A, Steady), 100.0f).Unit;
            if (std::fabs(U - Prev) > MaxJump) { MaxJump = std::fabs(U - Prev); Where = A; }
            Lo = std::min(Lo, (double)U); Hi = std::max(Hi, (double)U);
            Prev = U;
        }
        // A smooth function of the angle moves at most about its range x pi/180 x 0.02 x (bins/2pi
        // x 2pi) a step: well under 1e-2 of a unit.
        const bool bOk = MaxJump < 0.01;
        Fails += bOk ? 0 : 1;
        printf("angle sweep, steadiness %.2f: unit %.3f .. %.3f, largest step %.5f at %.2f deg %s\n", Steady, Lo, Hi, MaxJump, Where, bOk ? "ok" : "FAIL");
    }
    // 2. Continuity over the steadiness (sharp to rounded, the many winds in), 0.001 steps.
    for (const double Angle : { 10.0, 100.0, 211.25 })
    {
        const FPoint P = At(S, -340.0, 55.0);
        double MaxJump = 0.0, Where = 0.0;
        float Prev = Eval(S, P, Site(P, Angle, 0.0), 100.0f).Unit;
        for (int i = 1; i <= 1000; i++)
        {
            const float U = Eval(S, P, Site(P, Angle, i * 0.001), 100.0f).Unit;
            if (std::fabs(U - Prev) > MaxJump) { MaxJump = std::fabs(U - Prev); Where = i * 0.001; }
            Prev = U;
        }
        const bool bOk = MaxJump < 0.02;
        Fails += bOk ? 0 : 1;
        printf("steadiness sweep, wind %.2f deg: largest step %.5f at %.3f %s\n", Angle, MaxJump, Where, bOk ? "ok" : "FAIL");
    }
    // 3. The gradient against central differences (1 m) along east and north.
    {
        std::mt19937 Rng(7);
        std::uniform_real_distribution<double> Pos(-3000.0, 3000.0), Ang(0.0, 360.0), St(0.0, 1.0);
        double Worst = 0.0, SumErr = 0.0, SumMag = 0.0;
        for (int i = 0; i < 2000; i++)
        {
            const double U = Pos(Rng), V = Pos(Rng), A = Ang(Rng), Sd = St(Rng);
            const FPoint P = At(S, U, V);
            const FVoxelLandDuneSite Si = Site(P, A, Sd);
            const FVoxelLandPeaks D = Eval(S, P, Si, 100.0f);
            const double Hm = 0.25;
            const double HE = (Eval(S, At(S, U + Hm, V), Si, 100.0f).Unit - Eval(S, At(S, U - Hm, V), Si, 100.0f).Unit) * S.H.LandDuneRelief / (2.0 * Hm);
            const double HN = (Eval(S, At(S, U, V + Hm), Si, 100.0f).Unit - Eval(S, At(S, U, V - Hm), Si, 100.0f).Unit) * S.H.LandDuneRelief / (2.0 * Hm);
            // The analytic gradient's east and north components at P.
            const double L = std::sqrt(P.X * P.X + P.Y * P.Y + P.Z * P.Z);
            const double X = P.X / L, Y = P.Y / L, Z = P.Z / L;
            double EX = -Y, EY = X;
            const double EL = std::sqrt(EX * EX + EY * EY); EX /= EL; EY /= EL;
            const double NX = -Z * EY, NY = Z * EX, NZ = X * EY - Y * EX;
            const double GE = D.Gradient.x * EX + D.Gradient.y * EY;
            const double GN = D.Gradient.x * NX + D.Gradient.y * NY + D.Gradient.z * NZ;
            const double Err = std::sqrt((GE - HE) * (GE - HE) + (GN - HN) * (GN - HN));
            const double Mag = std::sqrt(HE * HE + HN * HN);
            SumErr += Err; SumMag += Mag;
            Worst = std::max(Worst, Err / std::max(Mag, 0.05));
        }
        const bool bOk = SumErr / SumMag < 0.05;
        Fails += bOk ? 0 : 1;
        printf("gradient vs differences: mean error %.4f of the mean slope %.4f (worst relative %.3f) %s\n", SumErr / 2000.0, SumMag / 2000.0, Worst, bOk ? "ok" : "FAIL");
    }
    // 3b. Continuity in space: north-south and east-west lines 5 km long at 0.25 m, a fixed wind
    // (the lattice's cells, the bake's cells, the face's B-spline segments).
    for (int Line = 0; Line < 4; Line++)
    {
        const bool bNS = (Line & 1) == 0;
        const double Steady = Line < 2 ? 0.9 : 0.0;
        double MaxJump = 0.0, Where = 0.0, SumStep = 0.0;
        const int Steps = 20000;
        auto PointAt = [&](int i) { const double T = -2500.0 + i * 0.25; return bNS ? At(S, 0.0, T) : At(S, T, 0.0); };
        float Prev = Eval(S, PointAt(0), Site(PointAt(0), 30.0, Steady), 100.0f).Unit;
        for (int i = 1; i <= Steps; i++)
        {
            const FPoint P = PointAt(i);
            const float U = Eval(S, P, Site(P, 30.0, Steady), 100.0f).Unit;
            const double J = std::fabs(U - Prev);
            SumStep += J;
            if (J > MaxJump) { MaxJump = J; Where = -2500.0 + i * 0.25; }
            Prev = U;
        }
        const double Mean = SumStep / Steps;
        const bool bOk = MaxJump < 12.0 * Mean + 1e-4;
        Fails += bOk ? 0 : 1;
        printf("%s line, steadiness %.1f: mean step %.5f, largest %.5f at %.2f m %s\n", bNS ? "north-south" : "east-west", Steady, Mean, MaxJump, Where, bOk ? "ok" : "FAIL");
    }
    // 4. The range over random points, winds and steadiness, against the bound.
    {
        std::mt19937 Rng(11);
        std::uniform_real_distribution<double> Pos(-50000.0, 50000.0), Ang(0.0, 360.0), St(0.0, 1.0);
        double Lo = 1e9, Hi = -1e9;
        for (int i = 0; i < 200000; i++)
        {
            const FPoint P = At(S, Pos(Rng), Pos(Rng));
            const float U = Eval(S, P, Site(P, Ang(Rng), St(Rng)), 100.0f).Unit;
            Lo = std::min(Lo, (double)U); Hi = std::max(Hi, (double)U);
        }
        const double B = 3.872983346207417;
        const double M = std::max(S.H.LandDuneMeanR * S.H.LandDuneScaleR, S.H.LandDuneMeanG * S.H.LandDuneScaleG);
        const double Detail = S.H.LandDuneDetailScale;
        const double Up = B * (1.0 + Detail), Down = -((B - 1.0) * M + B * Detail);
        const bool bOk = Hi <= Up && Lo >= Down;
        Fails += bOk ? 0 : 1;
        printf("range: unit %.3f .. %.3f (x %.0f m: %.0f .. %.0f m); the bound %.3f .. %.3f %s\n", Lo, Hi, S.H.LandDuneRelief, Lo * S.H.LandDuneRelief,
            Hi * S.H.LandDuneRelief, Down, Up, bOk ? "ok" : "FAIL");
    }
    // 5. Looks: 3 km square patches, 4 m cells, the wind at 30 degrees, by steadiness; and a
    // wind turning across the patch (the bins' blend); and a 120 m close-up with the detail.
    {
        auto Look = [&](const std::string& Name, double SizeM, int W, auto&& SiteAt, float FootprintCm) {
            std::vector<float> Hgt((size_t)W * W);
            for (int y = 0; y < W; y++)
                for (int x = 0; x < W; x++)
                {
                    const double U = (x + 0.5) / W * SizeM - 0.5 * SizeM, V = 0.5 * SizeM - (y + 0.5) / W * SizeM;
                    const FPoint P = At(S, U, V);
                    Hgt[(size_t)y * W + x] = Eval(S, P, SiteAt(P, U, V), FootprintCm).Unit * S.H.LandDuneRelief;
                }
            WritePPM(std::string(argv[3]) + "_" + Name + ".ppm", W, W, Hgt, SizeM / W);
        };
        for (const double Steady : { 0.95, 0.6, 0.3, 0.0 })
        {
            char Name[64];
            snprintf(Name, sizeof(Name), "steady%02d", (int)std::lround(Steady * 100));
            Look(Name, 3000.0, 600, [&](const FPoint& P, double, double) { return Site(P, 30.0, Steady); }, 400.0f);
        }
        Look("turning", 6000.0, 600, [&](const FPoint& P, double U, double) { return Site(P, 90.0 * (U + 3000.0) / 6000.0, 0.9); }, 800.0f);
        Look("closeup", 120.0, 600, [&](const FPoint& P, double, double) { return Site(P, 30.0, 0.9); }, 20.0f);
    }
    if (argc > 4)
        Fails += GlacialChecks(argv[4], argv[3]);
    if (argc > 5)
        Fails += IceChecks(argv[5], argv[3]);
    if (argc > 6)
        Fails += PolarChecks(argv[6], Dunes, argv[3]);
    printf(Fails ? "%d FAILED\n" : "ALL OK\n", Fails);
    return Fails ? 1 : 0;
}
