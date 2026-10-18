// The land surface cache (Design/LandSurfaceCache.md) as C++: the shipped VoxelLandCache.ush
// (geometry), VoxelLandCacheWriter.ush (what a texel holds) and VoxelLandCacheSample.ush (the
// material's read), on an eroded bake (gully_harness's planets). Checks:
//   - the writer's inverse (texel -> point on the sea sphere) against the forward map (point ->
//     plane coordinates), at every level, far from the anchor too;
//   - written levels sampled at random points against the surface computed at those points:
//     the height and normal the cache's interpolation loses;
//   - the plan's arithmetic (levels for a radius; the origin's exact float split).
//
// g++ -O2 -std=c++20 -ffp-contract=off -pthread -I<Public> -I<Private> -I<Shaders/Private> cache_harness.cpp
// (the plan itself: cache_plan_harness.cpp)
// cache_harness Tag N RadiusM
#define main gully_main
#include "gully_harness.cpp"
#undef main
#include "VoxelLandCachePlan.h"

namespace VoxelShader
{
    struct float4
    {
        float x, y, z, w;
        float4() : x(0), y(0), z(0), w(0) {}
        float4(float X, float Y, float Z, float W) : x(X), y(Y), z(Z), w(W) {}
    };

    /** A texture as the material sees it (Load), four floats a texel. */
    struct Texture2D
    {
        const float* Data = nullptr;
        int Width = 0, Height = 0;
        float4 Load(int3 P) const
        {
            if (P.x < 0 || P.y < 0 || P.x >= Width || P.y >= Height || P.z != 0) { fprintf(stderr, "texel out of range %d %d\n", P.x, P.y); exit(2); }
            const float* T = Data + 4 * ((size_t)P.y * Width + P.x);
            return float4(T[0], T[1], T[2], T[3]);
        }
    };

#include "VoxelLandCache.ush"
#include "VoxelLandCacheWriter.ush"
#include "VoxelLandCacheSample.ush"
}

/** A level as VoxelLandCache.cpp plans it: float axes, the double inverse. */
struct FLevelPlan
{
    float A[3], E1[3], E2[3];
    double Inv[3][3];
    double T;
    int OI, OJ;
};

static void Cross(const double* A, const double* B, double* O) { O[0] = A[1] * B[2] - A[2] * B[1]; O[1] = A[2] * B[0] - A[0] * B[2]; O[2] = A[0] * B[1] - A[1] * B[0]; }

static bool Invert3(const double M[3][3], double Out[3][3])
{
    const double C00 = M[1][1] * M[2][2] - M[1][2] * M[2][1], C01 = M[1][2] * M[2][0] - M[1][0] * M[2][2], C02 = M[1][0] * M[2][1] - M[1][1] * M[2][0];
    const double Det = M[0][0] * C00 + M[0][1] * C01 + M[0][2] * C02;
    if (std::fabs(Det) < 1e-12) return false;
    const double I = 1.0 / Det;
    Out[0][0] = C00 * I; Out[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) * I; Out[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) * I;
    Out[1][0] = C01 * I; Out[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) * I; Out[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) * I;
    Out[2][0] = C02 * I; Out[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) * I; Out[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) * I;
    return true;
}

static FLevelPlan Plan(const double* AnchorDir, const double* Camera, double SeaRadius, int K)
{
    FLevelPlan L;
    double A[3] = { AnchorDir[0], AnchorDir[1], AnchorDir[2] }; Normalize(A);
    for (int a = 0; a < 3; a++) L.A[a] = (float)A[a];
    const double Af[3] = { L.A[0], L.A[1], L.A[2] };
    const double Ref[3] = { std::fabs(Af[2]) < 0.9 ? 0.0 : 1.0, 0.0, std::fabs(Af[2]) < 0.9 ? 1.0 : 0.0 };
    double E1[3]; Cross(Ref, Af, E1); Normalize(E1);
    for (int a = 0; a < 3; a++) L.E1[a] = (float)E1[a];
    const double E1f[3] = { L.E1[0], L.E1[1], L.E1[2] };
    double E2[3]; Cross(Af, E1f, E2); Normalize(E2);
    for (int a = 0; a < 3; a++) L.E2[a] = (float)E2[a];
    const double M[3][3] = { { L.E1[0], L.E1[1], L.E1[2] }, { L.E2[0], L.E2[1], L.E2[2] }, { L.A[0], L.A[1], L.A[2] } };
    Invert3(M, L.Inv);
    L.T = VOXEL_LAND_CACHE_T0 * std::pow(2.0, K);
    // The window centred on the camera (VoxelLandCache::Centre).
    double C[3] = { Camera[0], Camera[1], Camera[2] }; Normalize(C);
    const double PA = C[0] * L.A[0] + C[1] * L.A[1] + C[2] * L.A[2];
    const double X = SeaRadius * (C[0] * L.E1[0] + C[1] * L.E1[1] + C[2] * L.E1[2]) / PA;
    const double Y = SeaRadius * (C[0] * L.E2[0] + C[1] * L.E2[1] + C[2] * L.E2[2]) / PA;
    L.OI = (int)std::floor(X / L.T) - VOXEL_LAND_CACHE_N / 2;
    L.OJ = (int)std::floor(Y / L.T) - VOXEL_LAND_CACHE_N / 2;
    return L;
}

static void InverseRows(const FLevelPlan& L, float3* High, float3* Low)
{
    for (int r = 0; r < 3; r++)
    {
        const float H[3] = { (float)L.Inv[r][0], (float)L.Inv[r][1], (float)L.Inv[r][2] };
        High[r] = float3(H[0], H[1], H[2]);
        Low[r] = float3((float)(L.Inv[r][0] - H[0]), (float)(L.Inv[r][1] - H[1]), (float)(L.Inv[r][2] - H[2]));
    }
}

int main(int argc, char** argv)
{
    const std::string Tag = argc > 1 ? argv[1] : "m1024";
    const int N = argc > 2 ? atoi(argv[2]) : 1024;
    const double RadiusM = argc > 3 ? atof(argv[3]) : 1.0e6;
    FPlanet P;
    Load(P, Tag, N, RadiusM, 30.0, HarnessGullyStrength());
    std::vector<FVoxelOctave> Gpu;
    SetGpuGlobals(P, Gpu);
    const double S = P.SeaRadius;
    const float2 SeaDF = DF(S);

    // Levels for a radius: the plan's (FVoxelLandCache::LevelsFor).
    auto LevelsFor = [](double R) { return VoxelLandCachePlan::LevelsFor(R); };
    printf("levels: 1,000 km %d, 5,000 km %d, Earth %d, 20,000 km %d, 50,000 km %d (max %d); this planet %d\n",
        LevelsFor(1e8), LevelsFor(5e8), LevelsFor(6.371e8), LevelsFor(2e9), LevelsFor(5e9), VOXEL_LAND_CACHE_MAX_LEVELS, LevelsFor(S));
    const int Levels = LevelsFor(S);

    // 1. The inverse against the forward map: random texels of every level's window, the
    // camera up to 10 degrees from the anchor.
    std::mt19937 Rng(5);
    std::normal_distribution<double> G;
    std::uniform_real_distribution<double> U(0.0, 1.0);
    double WorstTexel = 0, WorstRadius = 0;
    for (int Trial = 0; Trial < 200; Trial++)
    {
        double Anchor[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(Anchor);
        double E[3] = { G(Rng), G(Rng), G(Rng) };
        double Camera[3]; const double Angle = U(Rng) * 10.0 * M_PI / 180.0;
        double Dot = E[0] * Anchor[0] + E[1] * Anchor[1] + E[2] * Anchor[2];
        for (int a = 0; a < 3; a++) E[a] -= Dot * Anchor[a];
        Normalize(E);
        for (int a = 0; a < 3; a++) Camera[a] = Anchor[a] * std::cos(Angle) + E[a] * std::sin(Angle);
        for (int K = 0; K < Levels; K++)
        {
            const FLevelPlan L = Plan(Anchor, Camera, S, K);
            float3 High[3], Low[3]; InverseRows(L, High, Low);
            for (int s = 0; s < 8; s++)
            {
                const int I = L.OI + (int)(U(Rng) * VOXEL_LAND_CACHE_N), J = L.OJ + (int)(U(Rng) * VOXEL_LAND_CACHE_N);
                const FVoxelLandCachePoint Pt = VoxelLandCacheTexelPoint(I, J, (float)L.T, High[0], Low[0], High[1], Low[1], High[2], Low[2], SeaDF);
                const double R = std::sqrt(std::pow((double)Pt.X.x + Pt.X.y, 2) + std::pow((double)Pt.Y.x + Pt.Y.y, 2) + std::pow((double)Pt.Z.x + Pt.Z.y, 2));
                WorstRadius = std::max(WorstRadius, std::fabs(R - S));
                if ((double)Pt.X.x * L.A[0] + (double)Pt.Y.x * L.A[1] + (double)Pt.Z.x * L.A[2] <= 0) continue;
                const FVoxelLandCachePlane Pl = VoxelLandCachePlaneOf(Pt.X, Pt.Y, Pt.Z, float3(L.A[0], L.A[1], L.A[2]),
                    float3(L.E1[0], L.E1[1], L.E1[2]), float3(L.E2[0], L.E2[1], L.E2[2]), SeaDF);
                const double U0 = VoxelLandCacheTexelCoord(Pl.X, L.OI, (float)L.T) - 0.5f, V0 = VoxelLandCacheTexelCoord(Pl.Y, L.OJ, (float)L.T) - 0.5f;
                WorstTexel = std::max(WorstTexel, std::max(std::fabs(U0 - (I - L.OI)), std::fabs(V0 - (J - L.OJ))));
            }
        }
    }
    printf("inverse against forward: worst %.2g texels off, worst %.2g cm off the sea sphere (%d levels, the camera up to 10 degrees from the anchor)\n",
        WorstTexel, WorstRadius, Levels);

    // 2. Written levels sampled at points against the surface there. Levels 0, 4 and 8, the
    // camera 6 degrees from the anchor.
    // The camera in the mountains (the steepest of a few thousand land spots), its levels'
    // anchor 6 degrees away.
    double Camera[3] = { 0, 0, 1 }, Anchor[3];
    {
        double BestSlope = -1;
        std::mt19937 R2(77);
        std::normal_distribution<double> G2;
        for (int t = 0; t < 4000; t++)
        {
            double D[3] = { G2(R2), G2(R2), G2(R2) }; Normalize(D);
            const FVoxelLandHeight H = VoxelLandHeightDF(DF(D[0] * P.SeaRadius), DF(D[1] * P.SeaRadius), DF(D[2] * P.SeaRadius), 100.0f, true);
            const double Sl = std::sqrt(H.BakeGradient.x * H.BakeGradient.x + H.BakeGradient.y * H.BakeGradient.y + H.BakeGradient.z * H.BakeGradient.z);
            // CACHE_SPOT=hills: the hilliest instead (the hills' amplitude, T4a).
            const double Score = getenv("CACHE_SPOT") && std::string(getenv("CACHE_SPOT")) == "hills" ? (double)H.HillAmplitudeM * 0.001 : Sl;
            if (H.Bake > 300 && Score > BestSlope) { BestSlope = Score; for (int a = 0; a < 3; a++) Camera[a] = D[a]; }
        }
        double E[3] = { 0.9, 0.2, -0.1 };
        const double Dot = E[0] * Camera[0] + E[1] * Camera[1] + E[2] * Camera[2];
        for (int a = 0; a < 3; a++) E[a] -= Dot * Camera[a];
        Normalize(E);
        const double Angle = 6.0 * M_PI / 180.0;
        for (int a = 0; a < 3; a++) Anchor[a] = Camera[a] * std::cos(Angle) + E[a] * std::sin(Angle);
        printf("test spot: %s %.2f\n", getenv("CACHE_SPOT") ? "hill amplitude (km)" : "bake slope", BestSlope);
    }
    const int Rows = (VoxelLandCachePlan::TilesFor(Levels) + VOXEL_LAND_CACHE_COLUMNS - 1) / VOXEL_LAND_CACHE_COLUMNS;
    // Level 8 in the spare tile (as after a re-centring): the sampler reads each level's tile
    // from the state.
    auto TileOf = [Levels](int K) { return K == 8 ? Levels : K; };
    const int AW = VOXEL_LAND_CACHE_COLUMNS * VOXEL_LAND_CACHE_N, AH = Rows * VOXEL_LAND_CACHE_N;
    std::vector<float> HeightAtlas(4 * (size_t)AW * AH, 0.0f), SlopeAtlas(4 * (size_t)AW * AH, 0.0f), FieldsAtlas(4 * (size_t)AW * AH, 0.0f);
    std::vector<float> State(4 * VOXEL_LAND_CACHE_STATE_WIDTH * (VOXEL_LAND_CACHE_MAX_LEVELS + 1), 0.0f);
    State[0] = (float)Levels; State[1] = VOXEL_LAND_CACHE_N; State[2] = VOXEL_LAND_CACHE_T0;
    std::vector<FLevelPlan> Plans(Levels);
    for (int K = 0; K < Levels; K++) Plans[K] = Plan(Anchor, Camera, S, K);
    const int Written[] = { 0, 4, 8 };
    for (int K : Written)
    {
        const FLevelPlan& L = Plans[K];
        float3 High[3], Low[3]; InverseRows(L, High, Low);
        const double T0 = Now();
        ParallelFor(VOXEL_LAND_CACHE_N, [&](int Row) {
            for (int Col = 0; Col < VOXEL_LAND_CACHE_N; Col++)
            {
                const int I = L.OI + Col, J = L.OJ + Row;
                const FVoxelLandCachePoint Pt = VoxelLandCacheTexelPoint(I, J, (float)L.T, High[0], Low[0], High[1], Low[1], High[2], Low[2], SeaDF);
                const FVoxelLandCacheSurface Sf = VoxelLandCacheSurfaceAt(Pt.X, Pt.Y, Pt.Z, (float)L.T, std::max(VOXEL_LAND_CACHE_T0, 0.5f * (float)L.T));
                const int2 At = VoxelLandCacheAtlasTexel(TileOf(K), VoxelLandCacheSlot(I), VoxelLandCacheSlot(J));
                const size_t Index = 4 * ((size_t)At.y * AW + At.x);
                HeightAtlas[Index] = Sf.Height;
                SlopeAtlas[Index] = Sf.Gradient.x; SlopeAtlas[Index + 1] = Sf.Gradient.y; SlopeAtlas[Index + 2] = Sf.Gradient.z; SlopeAtlas[Index + 3] = Sf.Flow;
                // Level 0: a lake 3 m above every texel (relative, as the writer stores it).
                FieldsAtlas[Index + 2] = K == 0 ? 3.0f : VOXEL_LAND_CACHE_NO_LAKE; FieldsAtlas[Index + 3] = 1.0f;
            }
        });
        float* Row = &State[4 * VOXEL_LAND_CACHE_STATE_WIDTH * (1 + K)];
        Row[0] = L.A[0]; Row[1] = L.A[1]; Row[2] = L.A[2]; Row[3] = 1.0f;
        Row[4] = L.E1[0]; Row[5] = L.E1[1]; Row[6] = L.E1[2]; Row[7] = (float)L.T;
        Row[8] = L.E2[0]; Row[9] = L.E2[1]; Row[10] = L.E2[2]; Row[11] = (float)TileOf(K);
        const int LowI = L.OI & 4095, LowJ = L.OJ & 4095;
        Row[12] = (float)(L.OI - LowI); Row[13] = (float)LowI; Row[14] = (float)(L.OJ - LowJ); Row[15] = (float)LowJ;
        printf("level %2d written (%.0f cm texels, +-%.1f km): %.1f s on %u threads\n", K, L.T, L.T * VOXEL_LAND_CACHE_N * 0.5e-5, Now() - T0,
            std::max(1u, std::thread::hardware_concurrency()));
    }
    VoxelShader::Texture2D HT{ HeightAtlas.data(), AW, AH }, ST{ SlopeAtlas.data(), AW, AH }, FT{ FieldsAtlas.data(), AW, AH },
        StT{ State.data(), VOXEL_LAND_CACHE_STATE_WIDTH, VOXEL_LAND_CACHE_MAX_LEVELS + 1 };

    for (int K : Written)
    {
        const FLevelPlan& L = Plans[K];
        std::vector<double> EH, EN, Slope;
        int Wrong = 0;
        for (int s = 0; s < 4000; s++)
        {
            // A point within the inner half of the window around the camera.
            double E1[3], E2[3]; Frame(Camera, E1, E2);
            const double Reach = 0.25 * VOXEL_LAND_CACHE_N * L.T;
            const double A = (U(Rng) * 2 - 1) * Reach, B = (U(Rng) * 2 - 1) * Reach;
            double D[3]; for (int a = 0; a < 3; a++) D[a] = Camera[a] * S + E1[a] * A + E2[a] * B;
            Normalize(D);
            const float2 X = DF(D[0] * S), Y = DF(D[1] * S), Z = DF(D[2] * S);
            // A footprint whose level is K exactly (no blend toward K + 1).
            const FVoxelLandCacheSample C = VoxelLandCacheSampleAt(HT, ST, FT, StT, X, Y, Z, SeaDF, (float)L.T);
            if (!C.bValid || std::fabs(C.Level - K) > 1e-3)
            {
                if (Wrong++ < 3)
                {
                    const FVoxelLandCacheHit Hh = VoxelLandCacheLocate(StT, K, X, Y, Z, SeaDF);
                    printf("    not level %d: valid %d level %.3f, hit %d u %.1f v %.1f edge %.1f\n", K, (int)C.bValid, C.Level, (int)Hh.bHit, Hh.U, Hh.V, Hh.Edge);
                }
                continue;
            }
            const FVoxelLandCacheSurface R = VoxelLandCacheSurfaceAt(X, Y, Z, (float)L.T, std::max(VOXEL_LAND_CACHE_T0, 0.5f * (float)L.T));
            EH.push_back(std::fabs(C.Height - R.Height) * 100.0);
            const float3 Up = normalize(float3(X.x, Y.x, Z.x));
            auto Nrm = [&](float3 Gr) { const float3 Tg = Gr - Up * dot(Gr, Up); return normalize(Up - Tg); };
            const float3 NA = Nrm(C.Gradient), NB = Nrm(R.Gradient);
            EN.push_back(std::acos(std::min(1.0f, dot(NA, NB))) * 57.2957795);
            Slope.push_back(std::acos(std::min(1.0f, dot(NB, Up))) * 57.2957795);
        }
        std::sort(EH.begin(), EH.end()); std::sort(EN.begin(), EN.end()); std::sort(Slope.begin(), Slope.end());
        auto Q = [](const std::vector<double>& V, double q) { return V.empty() ? -1.0 : V[std::min((size_t)(q * V.size()), V.size() - 1)]; };
        printf("level %2d sampled against the surface: height error median %.3f p99 %.3f cm; normal error median %.3f p90 %.3f p99 %.3f deg; slope median %.1f deg; %d of 4000 not read from level %d\n",
            K, Q(EH, 0.5), Q(EH, 0.99), Q(EN, 0.5), Q(EN, 0.9), Q(EN, 0.99), Q(Slope, 0.5), Wrong, K);
    }

    // 3. A point outside level 0's window falls to a written coarser level.
    {
        double E1[3], E2[3]; Frame(Camera, E1, E2);
        double D[3]; for (int a = 0; a < 3; a++) D[a] = Camera[a] * S + E1[a] * 20000.0;   // 200 m: past level 0 (+-64 m)
        Normalize(D);
        const FVoxelLandCacheSample C = VoxelLandCacheSampleAt(HT, ST, FT, StT, DF(D[0] * S), DF(D[1] * S), DF(D[2] * S), SeaDF, VOXEL_LAND_CACHE_T0);
        printf("a point 200 m out at a level-0 footprint reads level %.2f (expected 4: level 0 doesn't reach, 1-3 aren't written)\n", C.Level);
    }

    // 4. The lake's level comes back absolute (the nearest texel's height + 3 m), and is absent
    // on a level without one.
    {
        const float2 X = DF(Camera[0] * S), Y = DF(Camera[1] * S), Z = DF(Camera[2] * S);
        const FVoxelLandCacheSample C0 = VoxelLandCacheSampleAt(HT, ST, FT, StT, X, Y, Z, SeaDF, VOXEL_LAND_CACHE_T0);
        const FVoxelLandCacheSample C4 = VoxelLandCacheSampleAt(HT, ST, FT, StT, X, Y, Z, SeaDF, (float)Plans[4].T);
        printf("lake at the camera: level %.2f height %.3f m lake %.3f m (lake - height %.3f, expected about 3); level %.2f lake %s\n",
            C0.Level, C0.Height, C0.Fields.z, C0.Fields.z - C0.Height, C4.Level, C4.Fields.z <= VOXEL_LAND_CACHE_NO_LAKE + 1.0f ? "none (expected)" : "PRESENT (wrong)");
    }

    // 5. Texels not yet written (w 0) fall through to the next written level.
    {
        const FLevelPlan& L = Plans[0];
        for (int j = 0; j < VOXEL_LAND_CACHE_N; j++)
            for (int i = 0; i < VOXEL_LAND_CACHE_N; i++)
            {
                const int2 At = VoxelLandCacheAtlasTexel(0, VoxelLandCacheSlot(L.OI + i), VoxelLandCacheSlot(L.OJ + j));
                FieldsAtlas[4 * ((size_t)At.y * AW + At.x) + 3] = 0.0f;
            }
        const FVoxelLandCacheSample C = VoxelLandCacheSampleAt(HT, ST, FT, StT, DF(Camera[0] * S), DF(Camera[1] * S), DF(Camera[2] * S), SeaDF, VOXEL_LAND_CACHE_T0);
        printf("level 0 unwritten: the camera's point reads level %.2f (expected 4)\n", C.Level);
    }
    return 0;
}
