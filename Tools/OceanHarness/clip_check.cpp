// clip_check.cpp - Stage W1: the wave clipmap's levels (VoxelOceanClipmap.h, compiled unchanged) and
// the shader's camera-relative projection (VoxelOceanClip.ush's VoxelOceanClipPlaneOffset and
// VoxelOceanClipPointOf, mirrored here in float).
//
// 1. PROJECTION: for planets of 1,000 km and Earth's radius and cameras from 2 m to 200 km up, every
//    level's window corners and random texels: the shader's float sphere point against the double
//    one, and the plane offset back (texels).
// 2. FLIGHT: a camera descending from orbit to the sea and flying along it at speed, 60 frames a
//    second, with the static layer's writes and the waves' staggering as FVoxelOceanPages::
//    UpdateClipmap plans them (mirrored), and each slot's storage simulated texel by texel. Every
//    frame: each level's static layer holds, at every window texel's toroidal place, that very
//    texel (this plane, this level); every point a user samples (within the reach around the
//    camera, its bilinear footprint) finds waves evaluated for that texel. Also the static texels
//    written per frame and the re-anchors.
// 3. EDITS: every texel within an edit's reach lies in its rectangle (RectAround).
// 4. RINGS (stage W2; VoxelOceanRingLayout.ush, compiled as C++ through VoxelShaderCompat.h, and
//    MakeRings): for cameras anywhere on the snap grids and many heights, every level's edge vertex
//    is bitwise the next level's vertex there (offset, scale, level: no cracks); random points in
//    the outermost square are covered by exactly one drawn triangle (the holes, the morph's
//    collapse, the diagonals); every vertex samples its clipmap level inside its reach, before the
//    fade.
//
//   g++ -O2 -std=c++20 -IClipStandIn -I<plugin>/Source/VoxelPlugin/Private -I<plugin>/Source/VoxelPlugin/Public
//       -I<plugin>/Shaders/Private clip_check.cpp -o clip_check

#include "ClipStandIn/CoreMinimal.h"
#include "VoxelOceanClipmap.h"
#include "VoxelShaderCompat.h"
namespace VoxelShader
{
#include "VoxelOceanRingLayout.ush"
}
#include <cstdio>
#include <random>
#include <vector>

using namespace VoxelOceanClipmap;

// ---- The shader's float projection (VoxelOceanClip.ush) ----
struct F3 { float x, y, z; };
static F3 f3(const FVector& v) { return { (float)v.X, (float)v.Y, (float)v.Z }; }
static float dotf(F3 a, F3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static F3 add(F3 a, F3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
static F3 mul(F3 a, float s) { return { a.x * s, a.y * s, a.z * s }; }

struct FShaderFrame
{
    F3 A, E1, E2;
    float Radius, Along, Height, CamS1, CamS2;
};

static FShaderFrame ShaderFrameOf(const FFrame& Frame)
{
    FVector4f Rows[InfoRows];
    FIntPoint Ev[MaxLevels];
    bool Valid[MaxLevels];
    for (int32 i = 0; i < Frame.Count; i++) { Ev[i] = Frame.Levels[i].Origin; Valid[i] = true; }
    MakeInfoRows(Frame, Ev, Valid, Rows);
    FShaderFrame F;
    F.A = { Rows[0].X, Rows[0].Y, Rows[0].Z }; F.Radius = Rows[0].W;
    F.E1 = { Rows[1].X, Rows[1].Y, Rows[1].Z }; F.Along = Rows[1].W;
    F.E2 = { Rows[2].X, Rows[2].Y, Rows[2].Z }; F.Height = Rows[2].W;
    F.CamS1 = Rows[3].X; F.CamS2 = Rows[3].Y;
    return F;
}

static void PlaneOffset(const FShaderFrame& F, F3 X, float& D1, float& D2)
{
    const float XA = dotf(X, F.A);
    const float Den = std::max(F.Along + XA, 1.0f);
    D1 = (dotf(X, F.E1) - F.CamS1 * XA) * (F.Radius / Den);
    D2 = (dotf(X, F.E2) - F.CamS2 * XA) * (F.Radius / Den);
}

static F3 PointOf(const FShaderFrame& F, float D1, float D2)
{
    const F3 U = add(add(F.A, mul(F.E1, F.CamS1)), mul(F.E2, F.CamS2));
    const float N = std::sqrt(dotf(U, U));
    const F3 Dv = mul(add(mul(F.E1, D1), mul(F.E2, D2)), 1.0f / F.Radius);
    const float UD = dotf(U, Dv);
    const float DD = dotf(Dv, Dv);
    const float M = std::sqrt(std::max(N * N + 2.0f * UD + DD, 1.0e-12f));
    const float NMinusM = -(2.0f * UD + DD) / (N + M);
    const F3 Diff = add(mul(Dv, 1.0f / M), mul(U, NMinusM / (M * N)));
    return add(mul(Diff, F.Radius), mul(U, -F.Height / N));
}

static int Fails = 0;
#define EXPECT(c, ...) do { if (!(c)) { Fails++; std::printf("FAIL: " __VA_ARGS__); std::printf("\n"); } } while (0)

static void ProjectionCheck()
{
    std::mt19937 Rng(7);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    const double Radii[] = { 1.0e8, 6.371e8 };
    const double Heights[] = { 200.0, 5000.0, 1.0e5, 2.0e6, 2.0e7, 2.0e8, 2.0e9 };
    for (double R : Radii)
    {
        double WorstCm = 0.0, WorstRel = 0.0, WorstBack = 0.0, WorstFirstCm = 0.0;
        int Levels = 0;
        for (double H : Heights)
        {
            for (int c = 0; c < 20; c++)
            {
                const FVector Dir = FVector(U(Rng), U(Rng), U(Rng)).GetSafeNormal();
                // A plane anchored up to the re-anchor angle away.
                const FVector Off = (FVector(U(Rng), U(Rng), U(Rng)) ^ Dir).GetSafeNormal();
                const double Angle = FMath::DegreesToRadians(ReanchorDegrees * 0.99 * (c % 5) / 4.0);
                const FVector AnchorDir = (Dir * std::cos(Angle) + Off * std::sin(Angle)).GetSafeNormal();
                FFrame Seed = MakeFrame(AnchorDir * (R + H), R, 128, 12, 20.0, nullptr);
                const FFrame Frame = MakeFrame(Dir * (R + H), R, 128, 12, 20.0, &Seed);
                EXPECT(Frame.Plane.Id == Seed.Plane.Id, "the plane re-anchored within the angle");
                const FShaderFrame SF = ShaderFrameOf(Frame);
                EXPECT(Frame.Count > 0, "no levels at %.0f km up", H * 1e-5);
                // Every window within the plane's useful part.
                for (int32 i = 0; i < Frame.Count; i++)
                {
                    const FLevel& L = Frame.Levels[i];
                    const double Far = std::max(std::fabs((double)L.Origin.X), std::fabs((double)L.Origin.X + Frame.Size))
                        + std::max(std::fabs((double)L.Origin.Y), std::fabs((double)L.Origin.Y + Frame.Size));
                    EXPECT(Far * L.Spacing <= 2.0 * R * std::tan(FMath::DegreesToRadians(MaxPlaneDegrees)) + 4.0 * L.Spacing,
                        "a window past the plane's limit (%.0f km up, level %d)", H * 1e-5, i);
                }
                Levels = std::max(Levels, Frame.Count);
                for (int32 i = 0; i < Frame.Count; i++)
                {
                    const FLevel& L = Frame.Levels[i];
                    const FVector2D InW = CameraInWindow(Frame, L.Spacing, L.Origin);
                    for (int t = 0; t < 40; t++)
                    {
                        double I, J;
                        if (t < 4) { I = (t & 1) ? Frame.Size - 1 : 0; J = (t & 2) ? Frame.Size - 1 : 0; }
                        else { I = std::floor((U(Rng) * 0.5 + 0.5) * Frame.Size); J = std::floor((U(Rng) * 0.5 + 0.5) * Frame.Size); }
                        const double D1 = (I + 0.5 - InW.X) * L.Spacing, D2 = (J + 0.5 - InW.Y) * L.Spacing;
                        // The double reference: the sphere point at the camera's plane point plus D.
                        const FVector Exact = SphereOf(Frame.Plane, R, FVector2D(Frame.CameraPlane.X + D1, Frame.CameraPlane.Y + D2)) - Frame.Camera;
                        const F3 X = PointOf(SF, (float)((I + 0.5f - (float)InW.X) * (float)L.Spacing), (float)((J + 0.5f - (float)InW.Y) * (float)L.Spacing));
                        const FVector Xd(X.x, X.y, X.z);
                        const double Err = (Xd - Exact).Size();
                        WorstCm = std::max(WorstCm, Err);
                        if (i == 0 && H <= 5000.0) WorstFirstCm = std::max(WorstFirstCm, Err);
                        WorstRel = std::max(WorstRel, Err / L.Spacing);
                        float B1, B2;
                        PlaneOffset(SF, f3(Exact), B1, B2);
                        WorstBack = std::max(WorstBack, std::max(std::fabs(B1 - D1), std::fabs(B2 - D2)) / L.Spacing);
                    }
                }
            }
        }
        std::printf("projection, radius %.0f km: up to %d levels; the shader's point within %.4f cm (%.2e texels; the first level's within %.4f cm from 50 m up or lower),\n"
            "  the offset back within %.2e texels\n", R * 1e-5, Levels, WorstCm, WorstRel, WorstFirstCm, WorstBack);
        EXPECT(WorstRel < 1.0e-3 && WorstBack < 1.0e-3, "projection precision");
    }
}

// ---- The flight, with UpdateClipmap's plan mirrored ----
struct FSimSlot
{
    bool bStatic = false; uint32 Plane = 0; int32 K = MIN_int32; FIntPoint Origin;
    bool bEvaluated = false; uint32 EvalPlane = 0; int32 EvalK = MIN_int32; FIntPoint EvalOrigin;
    // Storage: per toroidal texel, the absolute texel (and plane, level) its static layer and its
    // waves were written for.
    std::vector<int64> StaticAbs, WaveAbs;
};

static int64 Tag(uint32 Plane, int32 K, int32 I, int32 J)
{
    return ((int64)Plane << 52) ^ ((int64)(K + 64) << 44) ^ (((int64)I & 0x3fffff) << 22) ^ ((int64)J & 0x3fffff);
}

static void WriteRect(FSimSlot& S, const FFrame& F, const FLevel& L, const FRect& R, bool bWave)
{
    for (int32 j = R.J0; j < R.J0 + R.Height; j++)
        for (int32 i = R.I0; i < R.I0 + R.Width; i++)
        {
            const int32 AI = L.Origin.X + i, AJ = L.Origin.Y + j;
            const int32 SI = PositiveMod(AI, F.Size), SJ = PositiveMod(AJ, F.Size);
            (bWave ? S.WaveAbs : S.StaticAbs)[(size_t)SJ * F.Size + SI] = Tag(F.Plane.Id, L.K, AI, AJ);
        }
}

static void FlightCheck(double R, double DescentSpeed)
{
    const int32 Size = 128, MaxCount = 12;
    std::vector<FSimSlot> Slots(MaxLevels);
    for (FSimSlot& S : Slots) { S.StaticAbs.assign((size_t)Size * Size, -1); S.WaveAbs.assign((size_t)Size * Size, -1); }
    FFrame Prev; bool bHas = false;
    uint64 Turn = 0;
    int64 StaticTotal = 0, StaticMax = 0, Frames = 0, Reanchors = 0, BadStatic = 0, BadWave = 0, MaxDue = 0, WholeLater = 0, StaticMaxLater = 0;
    double WorstSlack = 0.0;
    std::mt19937 Rng(11);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    FVector Dir = FVector(0.3, 0.8, 0.52).GetSafeNormal();
    FVector Heading = (FVector(0, 0, 1) ^ Dir).GetSafeNormal();
    // Descend from 200 km to 2 m over 3 minutes while flying at 300 m/s, then 2 minutes at the sea
    // at 30 m/s with turns.
    const int TotalFrames = 60 * 300;
    for (int f = 0; f < TotalFrames; f++)
    {
        const double T = f / 60.0;
        const double Height = T < 180.0 ? 2.0e7 * std::pow(1.0e-5, T / 180.0) : 200.0 + 100.0 * std::sin(T);
        const double Speed = T < 180.0 ? DescentSpeed : 3000.0;   // cm/s
        Heading = (Heading + (Dir ^ Heading) * (0.02 * std::sin(T * 0.3))).GetSafeNormal();
        Dir = (Dir + Heading * (Speed / 60.0 / R)).GetSafeNormal();
        Heading = (Heading - Dir * (Heading | Dir)).GetSafeNormal();
        const FVector Camera = Dir * (R + Height);
        const FFrame F = MakeFrame(Camera, R, Size, MaxCount, 20.0, bHas ? &Prev : nullptr);
        if (!F.IsValid()) { EXPECT(false, "no frame"); continue; }
        if (bHas && F.Plane.Id != Prev.Plane.Id) Reanchors++;
        // The static layer (as UpdateClipmap: inputs unchanged here).
        int64 Texels = 0;
        bool bWhole[MaxLevels] = {};
        for (int32 i = 0; i < F.Count; i++)
        {
            const FLevel& L = F.Levels[i];
            FSimSlot& S = Slots[L.Slot];
            const bool bSame = S.bStatic && S.Plane == F.Plane.Id && S.K == L.K;
            if (!bSame) { WriteRect(S, F, L, FRect{ 0, 0, Size, Size }, false); Texels += (int64)Size * Size; bWhole[i] = true; if (f > 0) WholeLater++; }
            else
            {
                FRect G[2];
                const int32 NG = GainedStrips(S.Origin, L.Origin, Size, G);
                for (int32 g = 0; g < NG; g++) { WriteRect(S, F, L, G[g], false); Texels += (int64)G[g].Width * G[g].Height; if (G[g].Width == Size && G[g].Height == Size) bWhole[i] = true; }
            }
            S.bStatic = true; S.Plane = F.Plane.Id; S.K = L.K; S.Origin = L.Origin;
        }
        StaticTotal += Texels; StaticMax = std::max(StaticMax, Texels); Frames++;
        if (f > 0 && !(bHas && F.Plane.Id != Prev.Plane.Id)) StaticMaxLater = std::max(StaticMaxLater, Texels);
        // The waves due.
        const uint64 Now = Turn++;
        int64 NumDue = 0;
        FIntPoint Evaluated[MaxLevels];
        for (int32 i = 0; i < F.Count; i++)
        {
            const FLevel& L = F.Levels[i];
            FSimSlot& S = Slots[L.Slot];
            const bool bSameLevel = S.bEvaluated && S.EvalPlane == F.Plane.Id && S.EvalK == L.K;
            const bool bNear = bSameLevel && FMath::Abs(S.EvalOrigin.X - L.Origin.X) <= EvaluatedSlack && FMath::Abs(S.EvalOrigin.Y - L.Origin.Y) <= EvaluatedSlack;
            const uint64 Period = i < 4 ? 1u : (i < 8 ? 2u : 4u);
            if (bWhole[i] || !bNear || Now % Period == (uint64)i % Period)
            {
                // The waves read the static layer at every window texel: it must be this texel's.
                for (int32 j = 0; j < Size; j++)
                    for (int32 ii = 0; ii < Size; ii++)
                    {
                        const int32 AI = L.Origin.X + ii, AJ = L.Origin.Y + j;
                        if (S.StaticAbs[(size_t)PositiveMod(AJ, Size) * Size + PositiveMod(AI, Size)] != Tag(F.Plane.Id, L.K, AI, AJ)) BadStatic++;
                    }
                WriteRect(S, F, L, FRect{ 0, 0, Size, Size }, true);
                S.bEvaluated = true; S.EvalPlane = F.Plane.Id; S.EvalK = L.K; S.EvalOrigin = L.Origin;
                NumDue++;
            }
            Evaluated[i] = S.EvalOrigin;
        }
        MaxDue = std::max(MaxDue, NumDue);
        // The users' samples: points within the reach around the camera, their bilinear
        // neighbours, in every level (as the info's rows place them).
        const double Reach = 0.5 * Size - 2.0 - 3.5;
        for (int32 i = 0; i < F.Count; i++)
        {
            const FLevel& L = F.Levels[i];
            const FSimSlot& S = Slots[L.Slot];
            const FVector2D InEval = CameraInWindow(F, L.Spacing, Evaluated[i]);
            WorstSlack = std::max(WorstSlack, std::max(std::fabs(InEval.X - 0.5 * Size), std::fabs(InEval.Y - 0.5 * Size)));
            const FVector2D Cam = F.CameraPlane / L.Spacing;
            for (int t = 0; t < 24; t++)
            {
                const double OX = t < 4 ? ((t & 1) ? Reach : -Reach) : U(Rng) * Reach;
                const double OY = t < 4 ? ((t & 2) ? Reach : -Reach) : U(Rng) * Reach;
                // Bilinear: the texels whose centres surround the point.
                const double PX = Cam.X + OX - 0.5, PY = Cam.Y + OY - 0.5;
                for (int c = 0; c < 4; c++)
                {
                    const int32 AI = (int32)std::floor(PX) + (c & 1), AJ = (int32)std::floor(PY) + (c >> 1);
                    if (S.WaveAbs[(size_t)PositiveMod(AJ, Size) * Size + PositiveMod(AI, Size)] != Tag(F.Plane.Id, L.K, AI, AJ)) BadWave++;
                }
            }
        }
        Prev = F; bHas = true;
    }
    std::printf("flight, radius %.0f km, descent at %.0f m/s: %lld frames, %lld re-anchors; static texels %.0f a frame on average, %lld at most\n"
        "  (%lld at most on a frame without a re-anchor after the first; %lld whole levels later on); up to %lld levels due a frame;\n"
        "  the camera at most %.2f texels from an evaluated window's centre (slack %.1f); %lld static and %lld wave texels wrong\n",
        R * 1e-5, DescentSpeed * 0.01, (long long)Frames, (long long)Reanchors, (double)StaticTotal / std::max<int64>(Frames, 1), (long long)StaticMax,
        (long long)StaticMaxLater, (long long)WholeLater, (long long)MaxDue, WorstSlack, OriginSlack + EvaluatedSlack, (long long)BadStatic, (long long)BadWave);
    EXPECT(BadStatic == 0 && BadWave == 0, "storage consistency");
    EXPECT(WorstSlack <= OriginSlack + EvaluatedSlack + 1e-9, "slack");
}

static void EditCheck()
{
    const double R = 1.0e8;
    const FVector Camera = FVector(0.2, 0.9, 0.4).GetSafeNormal() * (R + 1000.0);
    const FFrame F = MakeFrame(Camera, R, 128, 12, 20.0, nullptr);
    std::mt19937 Rng(3);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    int64 Missed = 0, Checked = 0;
    for (int e = 0; e < 200; e++)
    {
        const FVector Dir = (Camera.GetSafeNormal() + FVector(U(Rng), U(Rng), U(Rng)) * 2.0e-4).GetSafeNormal();
        const FVector Center = Dir * R;
        const double Reach = 500.0 + 3000.0 * (U(Rng) * 0.5 + 0.5);
        for (int32 i = 0; i < F.Count; i++)
        {
            const FLevel& L = F.Levels[i];
            const FRect Rect = RectAround(F, L, Center, Reach, 2.0);
            for (int t = 0; t < 50; t++)
            {
                // A point within the reach (on the sphere), its window texel.
                const FVector Q = (Center + FVector(U(Rng), U(Rng), U(Rng)) * (Reach * 0.57)).GetSafeNormal() * R;
                if ((Q - Center).Size() > Reach) continue;
                const FVector2D P = PlaneOf(F.Plane, R, Q.GetSafeNormal()) / L.Spacing;
                const int32 I = (int32)std::floor(P.X) - L.Origin.X, J = (int32)std::floor(P.Y) - L.Origin.Y;
                if (I < 0 || J < 0 || I >= F.Size || J >= F.Size) continue;
                Checked++;
                if (I < Rect.I0 || J < Rect.J0 || I >= Rect.I0 + Rect.Width || J >= Rect.J0 + Rect.Height) Missed++;
            }
        }
    }
    std::printf("edits: %lld texels within an edit's reach checked, %lld outside its rectangle\n", (long long)Checked, (long long)Missed);
    EXPECT(Missed == 0 && Checked > 0, "edit rectangles");
}

// ---- The rings ----
static VoxelShader::FVoxelOceanRingLevel RingLevelOf(const FRingRow& Row)
{
    VoxelShader::FVoxelOceanRingLevel L;
    L.CamX = (float)Row.CamX;
    L.CamY = (float)Row.CamY;
    L.Spacing = (float)Row.Spacing;
    L.HoleX = Row.HoleX;
    L.HoleY = Row.HoleY;
    L.CoarseX = Row.CoarseX;
    L.CoarseY = Row.CoarseY;
    return L;
}

static void RingCheck(int32 Size)
{
    const int32 Half = RingHalf(Size);
    const float MorphStart = (float)Half * (2.0f / 3.0f), MorphEnd = (float)Half - 2.0f;
    const double Fade = 0.5 * Size - 2.0 - 3.5 - Size / 16.0;   // texels: VoxelOceanClip.ush's reach less the fade
    std::mt19937 Rng(5);
    std::uniform_real_distribution<double> U(-1.0, 1.0);
    int64 EdgeChecked = 0, EdgeMismatch = 0, Covered = 0, Uncovered = 0, Overlapped = 0, ReachWorst = 0;
    double WorstReach = 0.0;
    const double R = 1.0e8;
    for (int c = 0; c < 60; c++)
    {
        const double Height = std::pow(10.0, 2.0 + 5.0 * (c % 20) / 19.0);
        const FVector Dir = FVector(0.3 + 1e-3 * U(Rng), 0.8 + 1e-3 * U(Rng), 0.52).GetSafeNormal();
        const FFrame F = MakeFrame(Dir * (R + Height), R, Size, 12, 20.0, nullptr);
        if (!F.IsValid()) continue;
        FRingRow Rows[MaxLevels];
        MakeRings(F, F.Count, Rows);
        VoxelShader::FVoxelOceanRingLevel L[MaxLevels];
        for (int32 i = 0; i < MaxLevels; i++) L[i] = RingLevelOf(Rows[i]);
        auto PointOf = [&](int32 X, int32 Y, int32 I) { return VoxelShader::VoxelOceanRingPointOf(X, Y, I, F.Count, Half, MorphStart, MorphEnd, L[I], L[std::min(I + 1, MaxLevels - 1)]); };
        // 1. The edges.
        for (int32 i = 0; i + 1 < F.Count; i++)
        {
            for (int32 t = -Half; t <= Half; t++)
            {
                const int32 E[4][2] = { { t, -Half }, { t, Half }, { -Half, t }, { Half, t } };
                for (const auto& V : E)
                {
                    const VoxelShader::FVoxelOceanRingPoint A = PointOf(V[0], V[1], i);
                    const int32 XC = V[0] - (V[0] & 1), YC = V[1] - (V[1] & 1);
                    const VoxelShader::FVoxelOceanRingPoint B = PointOf(XC / 2 + Rows[i].CoarseX, YC / 2 + Rows[i].CoarseY, i + 1);
                    EdgeChecked++;
                    if (!(A.Valid == 1 && B.Valid == 1 && A.Level == B.Level && A.OffsetX == B.OffsetX && A.OffsetY == B.OffsetY && A.Scale == B.Scale)) EdgeMismatch++;
                }
            }
        }
        // 2. Coverage: the drawn triangles' plane positions (cm from the camera).
        struct FTri { double X[3], Y[3]; };
        std::vector<FTri> Tris;
        const int32 Side = 2 * Half + 1;
        for (int32 i = 0; i < F.Count; i++)
        {
            std::vector<VoxelShader::FVoxelOceanRingPoint> P((size_t)Side * Side);
            for (int32 y = -Half; y <= Half; y++)
                for (int32 x = -Half; x <= Half; x++)
                {
                    P[(size_t)(y + Half) * Side + (x + Half)] = PointOf(x, y, i);
                    const VoxelShader::FVoxelOceanRingPoint& Q = P[(size_t)(y + Half) * Side + (x + Half)];
                    if (!Q.Valid) continue;
                    // 3. The reach: its offset in its sampled level's texels.
                    const double T = std::max(std::fabs((double)Q.OffsetX), std::fabs((double)Q.OffsetY)) / (0.5 * L[Q.Level].Spacing);
                    WorstReach = std::max(WorstReach, T);
                    if (T > Fade) ReachWorst++;
                }
            auto At = [&](int32 x, int32 y) -> const VoxelShader::FVoxelOceanRingPoint& { return P[(size_t)(y + Half) * Side + (x + Half)]; };
            for (int32 y = -Half; y < Half; y++)
                for (int32 x = -Half; x < Half; x++)
                {
                    const bool bToward11 = (x >= 0) == (y >= 0);
                    const int32 Tri[2][3][2] = { { { x, y }, { x + 1, y + 1 }, { x + 1, y } }, { { x, y }, { x, y + 1 }, { x + 1, y + 1 } } };
                    const int32 TriB[2][3][2] = { { { x, y }, { x, y + 1 }, { x + 1, y } }, { { x + 1, y }, { x, y + 1 }, { x + 1, y + 1 } } };
                    for (int k = 0; k < 2; k++)
                    {
                        const auto& V = bToward11 ? Tri[k] : TriB[k];
                        bool bValid = true;
                        FTri T3;
                        for (int v = 0; v < 3; v++)
                        {
                            const VoxelShader::FVoxelOceanRingPoint& Q = At(V[v][0], V[v][1]);
                            bValid &= Q.Valid == 1;
                            T3.X[v] = Q.OffsetX;
                            T3.Y[v] = Q.OffsetY;
                        }
                        if (bValid) Tris.push_back(T3);
                    }
                }
        }
        const int32 O = F.Count - 1;
        for (int t = 0; t < 400; t++)
        {
            // A point within the outermost square, less a cell.
            const double G = Rows[O].Spacing;
            const double PX = (U(Rng) * (Half - 1) - Rows[O].CamX) * G, PY = (U(Rng) * (Half - 1) - Rows[O].CamY) * G;
            int Hits = 0;
            for (const FTri& T3 : Tris)
            {
                const double A2 = (T3.X[1] - T3.X[0]) * (T3.Y[2] - T3.Y[0]) - (T3.Y[1] - T3.Y[0]) * (T3.X[2] - T3.X[0]);
                if (std::fabs(A2) < 1e-9) continue;
                double W[3];
                for (int v = 0; v < 3; v++)
                {
                    const int a = (v + 1) % 3, b = (v + 2) % 3;
                    W[v] = ((T3.X[b] - T3.X[a]) * (PY - T3.Y[a]) - (T3.Y[b] - T3.Y[a]) * (PX - T3.X[a])) / A2;
                }
                if (W[0] > 0 && W[1] > 0 && W[2] > 0) Hits++;
            }
            if (Hits == 1) Covered++;
            else if (Hits == 0) Uncovered++;
            else Overlapped++;
        }
    }
    std::printf("rings (levels of %d texels, half %d cells): %lld edge vertices checked, %lld not the next level's own; %lld points covered once, %lld uncovered, %lld covered twice or more;\n"
        "  every vertex within %.1f texels of the camera in its level (the fade starts at %.1f), %lld past it\n",
        Size, Half, (long long)EdgeChecked, (long long)EdgeMismatch, (long long)Covered, (long long)Uncovered, (long long)Overlapped, WorstReach, Fade, (long long)ReachWorst);
    EXPECT(EdgeMismatch == 0 && Uncovered == 0 && Overlapped == 0 && ReachWorst == 0, "rings");
}

int main()
{
    ProjectionCheck();
    FlightCheck(1.0e8, 30000.0);
    FlightCheck(6.371e8, 30000.0);
    FlightCheck(1.0e8, 300000.0);
    EditCheck();
    RingCheck(64);
    RingCheck(128);
    RingCheck(256);
    std::printf(Fails ? "clip_check: %d FAILED\n" : "clip_check: all passed\n", Fails);
    return Fails ? 1 : 0;
}
