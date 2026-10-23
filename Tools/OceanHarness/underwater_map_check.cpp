// O6b: the underwater pass's surface map (VoxelUnderwaterMap.ush, VoxelUnderwaterMap.h) against
// the exact search it replaces (VoxelOceanHeightAtFFT, ported in hook_emulation.h), over emulated
// cascades and a turned planet, a camera under water.
// - Heights: the map against the search unmipped (what the pass used to draw) and mipped at each
//   level's texel (the map's own interpolation), level by level.
// - The waterline: points within 30 cm of the camera (the near plane).
// - Continuity: a walk outward from the camera across every level's edge.
// - Normals: the map's (central differences) against the hook's, mipped alike.
// - Exits: the pass's march on the map against the old march on the search, rays up through the
//   surface.
// - Cost: the map's points and the old search's per pixel.
// - Foam (stage O9c): the foam map's reads (VoxelUnderwaterMapFoam) against a synthetic foam field
//   (accuracy, range, continuity across the levels' edges, none past the coarsest), and the map
//   pass's surface point (the foot less the waves' horizontal displacement there, where the foam
//   is evaluated) against the inversion.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -IUEStandIn -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private -I. underwater_map_check.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWavesFFT.cpp
//       ../../Source/VoxelPlugin/Private/VoxelOceanWeather.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWeatherTable.cpp -o underwater_map_check
//   ./underwater_map_check [wind m/s, default 12]
#include "hook_emulation.h"
#include "VoxelUnderwaterMap.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <unordered_map>

using namespace VoxelShader;
using namespace HookEmulation;

// ---- The map, emulated: texels computed on first read ----------------------------------------

struct FMapEmulation
{
    const FTextures* Tx = nullptr;
    FParams W;
    float3 Centre;
    double Radius = 0, CameraHeight = 0;
    VoxelUnderwaterMap::FLevels Levels;
    std::unordered_map<long long, float> Cache;
    long long Computed = 0;

    float3 Anchor(int L) const { return float3(Levels.Anchor[L].X, Levels.Anchor[L].Y, Levels.Anchor[L].Z); }
    float Texel(int L) const { return Levels.Anchor[L].W; }
    float3 E1(int L) const { return float3(Levels.E1[L].X, Levels.E1[L].Y, Levels.E1[L].Z); }
    float3 E2(int L) const { return float3(Levels.E2[L].X, Levels.E2[L].Y, Levels.E2[L].Z); }

    float Load(int L, int X, int Y);
    // A synthetic foam field (stage O9c): 0 to 1, wavelengths of 3 m and 5 m (its steepest change
    // FoamSlope per cm), read at a map point as UnderwaterMapCS writes its texel.
    static float FoamAt(float3 P)
    {
        return 0.5f + 0.5f * std::sin(P.x * 0.0020944f + P.y * 0.0005f) * std::sin(P.y * 0.0012566f - P.z * 0.0003f);
    }
    float LoadFoam(int L, int X, int Y) const;
};
static constexpr float FoamSlope = 0.5f * (0.0021535f + 0.0012920f);
static FMapEmulation* GMap = nullptr;

namespace VoxelShader
{
#define VOXEL_UNDERWATER_MAP_LOAD(Level, X, Y) GMap->Load(Level, X, Y)
#define VOXEL_UNDERWATER_MAP_ANCHOR(Level) GMap->Anchor(Level)
#define VOXEL_UNDERWATER_MAP_TEXEL(Level) GMap->Texel(Level)
#define VOXEL_UNDERWATER_MAP_E1(Level) GMap->E1(Level)
#define VOXEL_UNDERWATER_MAP_E2(Level) GMap->E2(Level)
#define VOXEL_UNDERWATER_MAP_FOAM_LOAD(Level, X, Y) GMap->LoadFoam(Level, X, Y)
#include "VoxelUnderwaterMap.ush"
}

float FMapEmulation::LoadFoam(int L, int X, int Y) const
{
    return FoamAt(VoxelUnderwaterMapPoint(Anchor(L), Texel(L), E1(L), E2(L), X, Y));
}

// UnderwaterMapCS, ported.
float FMapEmulation::Load(int L, int X, int Y)
{
    const long long Key = ((long long)L << 40) | ((long long)X << 20) | (long long)Y;
    auto It = Cache.find(Key);
    if (It != Cache.end()) return It->second;
    const float3 P = VoxelUnderwaterMapPoint(Anchor(L), Texel(L), E1(L), E2(L), X, Y);
    FPoint Out;
    const float Above = HookHeightAt(*Tx, P, Centre, Radius, CameraHeight, W, Out, 3, 1u, Texel(L));
    const float Height = SphereHeight(P, float(Radius), float(CameraHeight), normalize(Centre * -1.0f));
    Computed++;
    return Cache[Key] = Height - Above;
}

// ---- Helpers ---------------------------------------------------------------------------------

static int Failures = 0;
static void Expect(bool Ok, const char* What)
{
    std::printf("  [%s] %s\n", Ok ? "ok" : "FAIL", What);
    if (!Ok) Failures++;
}
static FVector RandomDir(std::mt19937_64& Rng)
{
    std::normal_distribution<double> G;
    return FVector(G(Rng), G(Rng), G(Rng)).GetSafeNormal();
}
static float3 F3(const FVector& V) { return float3(float(V.X), float(V.Y), float(V.Z)); }
static float Degrees(float3 A, float3 B) { return std::acos(std::clamp(dot(normalize(A), normalize(B)), -1.0f, 1.0f)) * 57.29578f; }

// VoxelUnderwater.ush's shell exit, ported.
static float ShellExit(float B, float Height, float Radius, float Offset)
{
    const float C = (Height - Offset) * (2.0f * Radius + Height + Offset);
    const float Root = std::sqrt(std::max(B * B - C, 0.0f));
    return B > 0.0f ? -C / std::max(B + Root, 1.0e-6f) : Root - B;
}

// VoxelUnderwaterExit, ported: Above(T) the height above the surface at Start + Dir T.
template<class FAbove>
static float Exit(float3 Start, float3 Dir, float3 Centre, float Radius, float CameraHeight, float MaxHeight, int Steps, int Bisections, FAbove Above)
{
    const float Height = SphereHeight(Start, Radius, CameraHeight, normalize(Centre * -1.0f));
    const float B = dot(Start - Centre, Dir);
    const float Shell = MaxHeight + 1.0f;
    if (Height < -Shell && B < 0.0f) return 3.0e38f;
    const float Lower = Height < -Shell ? ShellExit(B, Height, Radius, -Shell) : 0.0f;
    float Upper = std::max(ShellExit(B, Height, Radius, Shell), Lower);
    if (Height >= -Shell && B < 0.0f)
    {
        const float Inner = (Height + Shell) * (2.0f * Radius + Height - Shell);
        const float Disc = B * B - Inner;
        if (Disc >= 0.0f) Upper = std::min(Upper, Inner / std::max(std::sqrt(Disc) - B, 1.0e-6f));
    }
    float Below = Lower, AboveT = Upper;
    bool bFound = false;
    for (int Step = 1; Step <= Steps; Step++)
    {
        const float X = float(Step) / float(Steps);
        const float T = Lower + (Upper - Lower) * X * X;
        if (Above(T) > 0.0f) { AboveT = T; bFound = true; break; }
        Below = T;
    }
    if (!bFound) return 3.0e38f;
    for (int i = 0; i < Bisections; i++)
    {
        const float Mid = 0.5f * (Below + AboveT);
        if (Above(Mid) > 0.0f) AboveT = Mid; else Below = Mid;
    }
    return 0.5f * (Below + AboveT);
}

int main(int argc, char** argv)
{
    const double Wind = argc > 1 ? std::atof(argv[1]) : 12.0;
    const double Radius = 1.075e8;
    FVoxelOceanFFTSettings Settings;
    Settings.Seed = 7;
    Settings.WindSpeed = Wind;
    const FVoxelOceanWavesFFT Model(Settings, Radius);
    FVoxelOceanWaveContext Context;
    Context.Time = 1234.0 / 15.0;
    Context.Radius = Radius;
    Context.Environment.WindSpeed = Wind;
    Context.Environment.WindHeading = 0.7;
    const auto Weather = ConstantWeather(Model, Wind, 0.7);
    Context.Weather = Weather;
    FTextures Tx;
    Tx.Charts = &Weather->Table;
    const uint32_t Tick = VoxelOceanSpectrum::TickOf(Context.Time);
    for (int s = 0; s < VoxelOceanSpectrum::SlotCount; s++)
    {
        const OceanFFTEmulation::FGPUSlot G = OceanFFTEmulation::LongCopy(OceanFFTEmulation::EmulateGPU(Model.GetSlots()[s], Tick));
        Tx.Displacement.Build(s, G.Displacement);
        Tx.Derivatives.Build(s, G.Derivatives);
    }
    std::printf("Wind %.1f m/s, reach %.0f cm\n", Wind, Model.MaxHeight());

    // A turned planet; the camera 1.5 m under the sea, near a cube edge (two faces' charts).
    const FQuat Rot = [] { const double a = 0.9, s = std::sin(a / 2); const FVector Ax = FVector(0.3, -0.8, 0.5).GetSafeNormal(); return FQuat(Ax.X * s, Ax.Y * s, Ax.Z * s, std::cos(a / 2)); }();
    const FTransform PlanetToWorld(Rot, FVector(1.0e7, -3.0e6, 2.0e5));
    const FVector CameraDir = FVector(1.0, 0.97, 0.31).GetSafeNormal();
    const double CameraHeight = -150.0;
    const FVector CameraPlanet = CameraDir * (Radius + CameraHeight);
    FVoxelOceanWaveShaderParams SP;
    Model.GetShaderParams(Context, CameraPlanet, PlanetToWorld, SP);
    FParams W = ParamsOf(SP);
    const float3 Centre = F3(Rot.RotateVector(-CameraPlanet));

    FMapEmulation Map;
    Map.Tx = &Tx; Map.W = W; Map.Centre = Centre; Map.Radius = Radius; Map.CameraHeight = CameraHeight;
    Map.Levels = VoxelUnderwaterMap::MakeLevels(CameraPlanet, Radius, Rot);
    GMap = &Map;

    auto FootOf = [&](float3 Q, float3& Up) {
        const float H = SphereHeight(Q, float(Radius), float(CameraHeight), normalize(Centre * -1.0f));
        Up = normalize(Q - Centre);
        return Q - Up * H;
    };
    // The exact search's surface height above the sphere at Q's radial (Footprint as asked).
    auto Exact = [&](float3 Q, float Footprint, FPoint& Out) {
        const float Above = HookHeightAt(Tx, Q, Centre, Radius, CameraHeight, W, Out, 3, 3u, Footprint);
        return SphereHeight(Q, float(Radius), float(CameraHeight), normalize(Centre * -1.0f)) - Above;
    };
    // A point on the sea at a lateral distance along a random tangent direction.
    const float3 CamUp = normalize(Centre * -1.0f);
    auto SeaPoint = [&](std::mt19937_64& Rng, double Distance) {
        FVector T = RandomDir(Rng);
        const FVector U(CamUp.x, CamUp.y, CamUp.z);
        T = (T - U * (T | U)).GetSafeNormal();
        const FVector P = U * (-CameraHeight + std::uniform_real_distribution<double>(-100, 100)(Rng)) + T * Distance;
        return F3(P);
    };

    // ---- Levels --------------------------------------------------------------------------------
    std::printf("\nThe levels (texel, half-width; the camera's offset from each anchor in texels):\n");
    for (int L = 0; L < VoxelUnderwaterMap::Levels; L++)
    {
        const float2 P = VoxelUnderwaterMapCoords(Map.Anchor(L), Map.Texel(L), Map.E1(L), Map.E2(L), float3(0, 0, 0));
        std::printf("  level %d: texel %.0f cm, half-width %.1f m; camera at (%.2f, %.2f) from the centre\n", L, Map.Texel(L),
            Map.Texel(L) * 128.0 / 100.0, P.x - 127.5f, P.y - 127.5f);
    }
    bool bFrames = true;
    for (int L = 0; L < VoxelUnderwaterMap::Levels; L++)
    {
        const float3 A = Map.Anchor(L), E1 = Map.E1(L), E2 = Map.E2(L);
        float3 Up;
        const float3 Foot = FootOf(A, Up);
        bFrames &= length(A - Foot) < 0.05f && std::fabs(dot(E1, Up)) < 1e-4f && std::fabs(dot(E2, Up)) < 1e-4f && std::fabs(dot(E1, E2)) < 1e-5f;
    }
    Expect(bFrames, "each anchor lies on the sphere, its axes tangent and orthonormal");

    // ---- Heights by level -----------------------------------------------------------------------
    std::printf("\nHeights: the map against the search (unmipped; mipped at the level's texel):\n");
    std::mt19937_64 Rng(11);
    double WorstL0 = 0, WorstInterp[VoxelUnderwaterMap::Levels] = {};
    for (int L = 0; L < VoxelUnderwaterMap::Levels; L++)
    {
        const double Inner = L == 0 ? 0.0 : 0.25 * Map.Texel(L) * 112.0, Outer = Map.Texel(L) * 112.0;
        double MaxExact = 0, MaxMip = 0, SumExact = 0;
        const int Count = 150;
        for (int k = 0; k < Count; k++)
        {
            const float3 Q = SeaPoint(Rng, std::uniform_real_distribution<double>(Inner, Outer)(Rng));
            float3 Up;
            const float3 Foot = FootOf(Q, Up);
            const float M = VoxelUnderwaterMapHeight(Foot);
            FPoint Out;
            const float E0 = Exact(Q, 0.0f, Out);
            const float EM = Exact(Q, Map.Texel(L), Out);
            MaxExact = std::max(MaxExact, (double)std::fabs(M - E0));
            MaxMip = std::max(MaxMip, (double)std::fabs(M - EM));
            SumExact += (M - E0) * (M - E0);
        }
        std::printf("  level %d (%.0f-%.0f m out): unmipped up to %.1f mm (rms %.1f mm); mipped alike up to %.1f mm\n", L, Inner / 100, Outer / 100,
            MaxExact * 10, std::sqrt(SumExact / Count) * 10, MaxMip * 10);
        if (L == 0) WorstL0 = MaxExact;
        WorstInterp[L] = MaxMip;
    }
    Expect(WorstL0 < 0.5, "level 0 matches the unmipped search within 5 mm");
    bool bInterp = true;
    for (int L = 0; L < VoxelUnderwaterMap::Levels; L++) bInterp &= WorstInterp[L] < 0.08 * Model.MaxHeight();
    Expect(bInterp, "every level matches the search mipped alike within 8% of the reach (bilinear between texels)");

    // ---- The waterline ----------------------------------------------------------------------------
    std::printf("\nThe waterline (points within 30 cm of the camera, the near plane):\n");
    {
        double Worst = 0;
        for (int k = 0; k < 300; k++)
        {
            const FVector D = RandomDir(Rng) * std::uniform_real_distribution<double>(0, 30)(Rng);
            const float3 Q = F3(D);
            float3 Up;
            const float3 Foot = FootOf(Q, Up);
            FPoint Out;
            Worst = std::max(Worst, (double)std::fabs(VoxelUnderwaterMapHeight(Foot) - Exact(Q, 0.0f, Out)));
        }
        std::printf("  the map's surface against the search: up to %.2f mm\n", Worst * 10);
        Expect(Worst < 0.3, "the waterline within 3 mm of the old search");
    }

    // ---- Continuity -------------------------------------------------------------------------------
    std::printf("\nContinuity: a walk outward to 1 km, steps of 1 cm to 50 cm, across every level's edge:\n");
    {
        FVector T = RandomDir(Rng);
        const FVector U(CamUp.x, CamUp.y, CamUp.z);
        T = (T - U * (T | U)).GetSafeNormal();
        double WorstSlope = 0, Past = 0;
        float Last = 0;
        double LastD = 0;
        bool bFirst = true;
        for (double D = 0; D < 100000.0; D += std::max(1.0, D / 2000.0))
        {
            const float3 Q = F3(U * (-CameraHeight) + T * D);
            float3 Up;
            const float H = VoxelUnderwaterMapHeight(FootOf(Q, Up));
            if (!bFirst) WorstSlope = std::max(WorstSlope, std::fabs(H - Last) / (D - LastD));
            bFirst = false;
            Last = H;
            LastD = D;
            if (D > 94000.0) Past = std::max(Past, (double)std::fabs(H));
        }
        std::printf("  steepest step %.3f cm per cm; past the coarsest level (940 m+) the map's height %.1f cm (the sphere)\n", WorstSlope, Past);
        Expect(WorstSlope < 1.0, "no step at the levels' edges (every step under a slope of 1)");
    }

    // ---- Foam (stage O9c) ---------------------------------------------------------------------------
    std::printf("\nFoam: the foam map's reads against a synthetic field; the map pass's surface point:\n");
    {
        // Accuracy where the finest levels hold it (the field is resolved there).
        double WorstNear = 0;
        for (int k = 0; k < 400; k++)
        {
            const float3 Q = SeaPoint(Rng, std::uniform_real_distribution<double>(0.0, 2000.0)(Rng));
            float3 Up;
            const float3 Foot = FootOf(Q, Up);
            WorstNear = std::max(WorstNear, (double)std::fabs(VoxelUnderwaterMapFoam(Foot) - FMapEmulation::FoamAt(Foot)));
        }
        std::printf("  within 20 m of the camera: the map's foam within %.4f of the field\n", WorstNear);
        Expect(WorstNear < 0.01, "the foam map reads the field within 0.01 where it resolves it");
        // A walk outward across every level's edge: in range, no step, none past the coarsest.
        FVector T = RandomDir(Rng);
        const FVector U(CamUp.x, CamUp.y, CamUp.z);
        T = (T - U * (T | U)).GetSafeNormal();
        double WorstSlope = 0, Past = 0, Low = 1, High = 0;
        float Last = 0;
        double LastD = 0;
        bool bFirst = true;
        for (double D = 0; D < 100000.0; D += std::max(1.0, D / 2000.0))
        {
            const float3 Q = F3(U * (-CameraHeight) + T * D);
            float3 Up;
            const float F = VoxelUnderwaterMapFoam(FootOf(Q, Up));
            Low = std::min(Low, (double)F);
            High = std::max(High, (double)F);
            if (!bFirst) WorstSlope = std::max(WorstSlope, std::fabs(F - Last) / (D - LastD));
            bFirst = false;
            Last = F;
            LastD = D;
            if (D > 94000.0) Past = std::max(Past, (double)F);
        }
        std::printf("  a walk to 1 km: foam %.3f to %.3f, steepest change %.5f per cm (the field's %.5f); past the coarsest level %.3f\n",
            Low, High, WorstSlope, FoamSlope, Past);
        Expect(Low >= 0.0 && High <= 1.0, "the foam stays within 0 to 1");
        Expect(WorstSlope < 2.0 * FoamSlope, "no step at the levels' edges (under twice the field's steepest change)");
        Expect(Past == 0.0, "no foam past the coarsest level");
        // The surface point UnderwaterMapCS evaluates the foam at: the foot less the horizontal
        // displacement the inversion found there; its own waves carry it back over the foot.
        double WorstBack = 0, Moved = 0;
        for (int k = 0; k < 60; k++)
        {
            const float3 Q = SeaPoint(Rng, std::uniform_real_distribution<double>(0.0, 50000.0)(Rng));
            float3 Up;
            const float3 Foot = FootOf(Q, Up);
            FPoint Out;
            HookHeightAt(Tx, Q, Centre, Radius, CameraHeight, W, Out, 3, 1u, 0.0f);
            const float3 Point = Foot - Rotate(W.Rotation, Out.R.Displacement);
            Moved = std::max(Moved, (double)length(Point - Foot));
            const FPoint Back = HookEvaluate(Tx, Point, normalize(Point - Centre), 0.0f, 0.0f, W, 1u, Deep, ShoreAt(Tx, Point, normalize(Point - Centre), W));
            float3 R = Point + Rotate(W.Rotation, Back.R.Displacement) - Foot;
            R = R - Up * dot(Up, R);
            WorstBack = std::max(WorstBack, (double)length(R));
        }
        std::printf("  the foam's surface point (up to %.0f cm from the foot), carried by its own waves, lands within %.2f cm of the foot\n", Moved, WorstBack);
        Expect(WorstBack < 2.0 && Moved > 10.0, "the map pass evaluates the foam at the surface point above (within 2 cm; the waves move it)");
    }

    // ---- Normals ----------------------------------------------------------------------------------
    std::printf("\nNormals: the map's against the hook's (mipped alike):\n");
    {
        double WorstNear = 0, WorstAll = 0;
        for (int L = 0; L < 3; L++)
        {
            const double Inner = L == 0 ? 0.0 : 0.25 * Map.Texel(L) * 112.0, Outer = Map.Texel(L) * 112.0;
            double Worst = 0, Sum = 0;
            const int Count = 100;
            for (int k = 0; k < Count; k++)
            {
                const float3 Q = SeaPoint(Rng, std::uniform_real_distribution<double>(Inner, Outer)(Rng));
                float3 Up;
                const float3 Foot = FootOf(Q, Up);
                const float3 N = VoxelUnderwaterMapNormal(Foot, Up);
                FPoint Out;
                Exact(Q, Map.Texel(L), Out);
                const float3 HookN = Rotate(W.Rotation, Out.R.Normal);
                const double A = Degrees(N, HookN);
                Worst = std::max(Worst, A);
                Sum += A;
            }
            std::printf("  level %d: up to %.2f deg (mean %.2f deg)\n", L, Worst, Sum / Count);
            if (L == 0) WorstNear = Sum / Count;
            WorstAll = std::max(WorstAll, Sum / Count);
        }
        Expect(WorstNear < 1.0 && WorstAll < 3.0, "the underside's normals: mean under 1 deg near the camera, 3 deg out to 40 m");
    }

    // ---- Exits ------------------------------------------------------------------------------------
    std::printf("\nExits: rays up through the surface, the map's march (32 + 10) against the search's (16 + 6):\n");
    {
        double WorstPoint = 0, WorstHeight = 0, Sum = 0;
        int Count = 0, Disagree = 0;
        const float MaxHeight = W.RefLow.W;
        for (int k = 0; k < 150; k++)
        {
            FVector D = RandomDir(Rng);
            const FVector U(CamUp.x, CamUp.y, CamUp.z);
            if ((D | U) < 0.15) D = (D + U * (0.15 - (D | U) + std::uniform_real_distribution<double>(0, 0.8)(Rng))).GetSafeNormal();
            const float3 Dir = F3(D);
            const float3 Start(0, 0, 0);
            auto MapAbove = [&](float T) {
                const float3 Q = Start + Dir * T;
                float3 Up;
                const float3 Foot = FootOf(Q, Up);
                return SphereHeight(Q, float(Radius), float(CameraHeight), normalize(Centre * -1.0f)) - VoxelUnderwaterMapHeight(Foot);
            };
            auto SearchAbove = [&](float T) {
                FPoint Out;
                return HookHeightAt(Tx, Start + Dir * T, Centre, Radius, CameraHeight, W, Out, 3, 1u, 0.0f);
            };
            const float TM = Exit(Start, Dir, Centre, float(Radius), float(CameraHeight), MaxHeight, 32, 10, MapAbove);
            const float TS = Exit(Start, Dir, Centre, float(Radius), float(CameraHeight), MaxHeight, 16, 6, SearchAbove);
            if (TM > 1e37f || TS > 1e37f) { Disagree += (TM > 1e37f) != (TS > 1e37f); continue; }
            Count++;
            const double Gap = std::fabs(TM - TS) * std::fabs(dot(Dir, CamUp));
            WorstPoint = std::max(WorstPoint, (double)std::fabs(TM - TS));
            WorstHeight = std::max(WorstHeight, Gap);
            Sum += std::fabs(TM - TS);
            // The search's exit point measured against the exact surface (both within their
            // bisection's width of it).
        }
        std::printf("  %d rays: exits apart by up to %.1f cm along the ray (mean %.2f cm), %.1f cm in height; %d rays where one leaves and the other doesn't\n",
            Count, WorstPoint, Sum / std::max(Count, 1), WorstHeight, Disagree);
        Expect(Disagree == 0 && WorstHeight < 5.0, "the map's exits match the old search's (5 cm in height)");
    }

    // ---- Cost -------------------------------------------------------------------------------------
    std::printf("\nCost:\n");
    {
        const double MapPoints = double(VoxelUnderwaterMap::Size) * VoxelUnderwaterMap::Size * VoxelUnderwaterMap::Levels;
        const double Pixels = 1920.0 * 1080.0;
        std::printf("  the map: %.0f searches a view (%.2f per 1080p pixel), where the old pass ran up to 24 a pixel: %.0fx fewer\n",
            MapPoints, MapPoints / Pixels, 24.0 * Pixels / MapPoints);
        std::printf("  (this check computed %lld of its points)\n", Map.Computed);
    }

    std::printf("\n%s (%d failures)\n", Failures ? "FAILED" : "ALL OK", Failures);
    return Failures ? 1 : 0;
}
