// The terrain material's land evaluation (VoxelLandMaterialAccess.ush: texture loads and the
// constants texture) against the global shaders' (buffers and uniforms), as C++: both compile
// the same shared land code (VoxelLandSample/Gully/Plates.ush, VoxelDensitySample.ush,
// VoxelLandDebugColor.ush), one through each set of accessors. The textures are packed here as
// VoxelLandMaterial::BuildFieldTexels and BuildConstantTexels pack them (layout VoxelLandMaterialLayout.ush). The two
// must agree bit for bit.
//
// g++ -O2 -std=c++20 -ffp-contract=off -pthread -I<Public> -I<Shaders/Private> material_harness.cpp material_side.cpp
// material_harness Tag N RadiusM   (Tag_*.f32 from `solve_harness N Tag dump RadiusM`)
#define main gully_main
#include "gully_harness.cpp"
#undef main

#include "VoxelLandMaterialLayout.ush"
#include "material_side.h"

static float HalfToFloat(uint16_t H) { return VoxelShader::f16tof32(H); }

int main(int argc, char** argv)
{
    const std::string Tag = argc > 1 ? argv[1] : "m1024";
    const int N = argc > 2 ? atoi(argv[2]) : 1024;
    const double RadiusM = argc > 3 ? atof(argv[3]) : 1.0e6;
    FPlanet P;
    Load(P, Tag, N, RadiusM, 30.0, 0.22);
    std::vector<FVoxelOctave> Gpu;
    SetGpuGlobals(P, Gpu);
    auto W = ReadFile(Tag + "_water.f32");
    std::vector<float> Water(W.size() / 4); std::memcpy(Water.data(), W.data(), W.size());
    const int Side = P.Side(), Cells = (int)P.Height.size();
    // Attributes: synthetic (the dump has none), every bit pattern the decode must keep.
    std::vector<uint16_t> Attr(Cells);
    for (int i = 0; i < Cells; i++) Attr[i] = (uint16_t)((i * 2654435761u) >> 19) & 0x1fffu;

    // The textures, packed as BuildTextureData does.
    std::vector<float> HeightTex(4 * (size_t)Cells), FieldTex(4 * (size_t)Cells);
    for (int i = 0; i < Cells; i++)
    {
        HeightTex[4 * (size_t)i] = P.Height[i];
        const uint16_t FlowBits = (uint16_t)((P.FlowPacked[i >> 1] >> ((i & 1) * 16)) & 0xffffu);
        FieldTex[4 * (size_t)i] = HalfToFloat(FlowBits);
        const float L = Water[i];
        FieldTex[4 * (size_t)i + 1] = HalfToFloat(ToHalf(L > -1.0e30f ? std::clamp(L - P.Height[i], VOXEL_LAND_MATERIAL_NO_LAKE + 1000.0f, 65000.0f) : VOXEL_LAND_MATERIAL_NO_LAKE));
        FieldTex[4 * (size_t)i + 2] = (float)(Attr[i] & 0xffu);
        FieldTex[4 * (size_t)i + 3] = (float)(Attr[i] >> 8);
    }
    const int TanBase = VOXEL_LAND_MC_TEXELS, TanTexels = ((int)P.Tan.size() + 1) / 2;
    const int OctaveBase = TanBase + TanTexels, PlateBase = OctaveBase + (int)Gpu.size() * VOXEL_LAND_MATERIAL_OCTAVE_TEXELS;
    const int Texels = PlateBase, Rows = (Texels + VOXEL_LAND_MATERIAL_CONST_WIDTH - 1) / VOXEL_LAND_MATERIAL_CONST_WIDTH;
    std::vector<float> Const(4 * (size_t)Rows * VOXEL_LAND_MATERIAL_CONST_WIDTH, 0.0f);
    float* S = Const.data();
    auto PutU32 = [](float* D, int I, uint32_t V) { D[I] = (float)(V & 0xffffu); D[I + 1] = (float)(V >> 16); };
    S[VOXEL_LAND_MC_N] = (float)N; S[VOXEL_LAND_MC_PAD] = (float)P.Pad; S[VOXEL_LAND_MC_HAS_FLOW] = 1; S[VOXEL_LAND_MC_CELL_M] = P.S.LandCellM;
    S[VOXEL_LAND_MC_GULLY_STRENGTH] = P.S.LandGullyStrength; S[VOXEL_LAND_MC_GULLY_OCTAVES] = (float)P.S.LandGullyOctaves;
    S[VOXEL_LAND_MC_GULLY_FLOW_GAIN] = P.S.LandGullyFlowGain; S[VOXEL_LAND_MC_GULLY_DETAIL] = P.S.LandGullyDetail;
    PutU32(S, VOXEL_LAND_MC_GULLY_SEED, P.S.LandGullySeed);
    S[VOXEL_LAND_MC_WARP_OCTAVES] = (float)VoxelShader::LandWarpOctaves; S[VOXEL_LAND_MC_WARP_INV_NORM] = VoxelShader::LandWarpInvNorm;
    S[VOXEL_LAND_MC_WARP_AMPLITUDE] = VoxelShader::LandWarpAmplitude; S[VOXEL_LAND_MC_DETAIL_AMPLITUDE] = VoxelShader::LandDetailAmplitude;
    S[VOXEL_LAND_MC_HEIGHT_OFFSET] = VoxelShader::LandHeightOffset.x; S[VOXEL_LAND_MC_HEIGHT_OFFSET + 1] = VoxelShader::LandHeightOffset.y;
    S[VOXEL_LAND_MC_PLANET_RADIUS] = VoxelShader::PlanetRadius.x; S[VOXEL_LAND_MC_PLANET_RADIUS + 1] = VoxelShader::PlanetRadius.y;
    S[VOXEL_LAND_MC_NOISE_AMPLITUDE] = VoxelShader::NoiseAmplitude.x; S[VOXEL_LAND_MC_NOISE_AMPLITUDE + 1] = VoxelShader::NoiseAmplitude.y;
    S[VOXEL_LAND_MC_INV_NORM_DETAIL] = VoxelShader::InvNormDetail.x; S[VOXEL_LAND_MC_INV_NORM_DETAIL + 1] = VoxelShader::InvNormDetail.y;
    S[VOXEL_LAND_MC_RIDGE_GAIN] = VoxelShader::RidgeGain; S[VOXEL_LAND_MC_SLOPE_DAMPING] = VoxelShader::SlopeDamping;
    S[VOXEL_LAND_MC_CONTINENT_OCTAVES] = 0; S[VOXEL_LAND_MC_MOUNTAIN_OCTAVES] = 0; S[VOXEL_LAND_MC_DETAIL_OCTAVES] = (float)VoxelShader::DetailOctaves;
    S[VOXEL_LAND_MC_LAND_MODE] = 1; S[VOXEL_LAND_MC_PLATE_COUNT] = 0; S[VOXEL_LAND_MC_HAS_HYDRO] = 1;
    S[VOXEL_LAND_MC_TAN_BASE] = (float)TanBase; S[VOXEL_LAND_MC_OCTAVE_BASE] = (float)OctaveBase; S[VOXEL_LAND_MC_PLATE_BASE] = (float)PlateBase;
    S[VOXEL_LAND_MC_PLATE_RADIUS_M] = (float)RadiusM; S[VOXEL_LAND_MC_PLATE_WIDTH_SCALE] = 1.0f;
    S[VOXEL_LAND_MC_SIDE] = (float)Side;
    for (size_t i = 0; i < P.Tan.size(); i++) { float* T = S + 4 * (TanBase + i / 2) + 2 * (i & 1); T[0] = P.Tan[i].x; T[1] = P.Tan[i].y; }
    for (size_t o = 0; o < Gpu.size(); o++)
    {
        const FVoxelOctave& O = Gpu[o];
        float* T = S + 4 * (OctaveBase + o * VOXEL_LAND_MATERIAL_OCTAVE_TEXELS);
        T[0] = O.Frequency.x; T[1] = O.Frequency.y; T[2] = O.OffsetX.x; T[3] = O.OffsetX.y;
        T[4] = O.OffsetY.x; T[5] = O.OffsetY.y; T[6] = O.OffsetZ.x; T[7] = O.OffsetZ.y;
        T[8] = O.Wavelength; T[9] = O.Amplitude;
        PutU32(T, 10, O.Seed); PutU32(T, 12, O.Shift);
        PutU32(T, 14, O.OffsetCellX); PutU32(T, 16, O.OffsetCellY); PutU32(T, 18, O.OffsetCellZ);
        PutU32(T, 20, O.OffsetFracX); PutU32(T, 22, O.OffsetFracY); PutU32(T, 24, O.OffsetFracZ);
    }
    FMaterialTextures Tex;
    Tex.Height = HeightTex.data(); Tex.Fields = FieldTex.data(); Tex.Constants = Const.data(); Tex.Side = Side; Tex.Rows = Rows;

    // Decode checks: every attribute and lake level, every octave field.
    int AttrBad = 0, WaterBad = 0, Lakes = 0;
    float WorstLake = 0;
    for (int i = 0; i < Cells; i++)
    {
        AttrBad += MaterialAttr(Tex, (uint32_t)i) != Attr[i];
        const float Wm = MaterialWater(Tex, (uint32_t)i);
        const bool bLake = Water[i] > -1e30f;
        WaterBad += bLake != (Wm > -1e30f);
        if (bLake && Wm > -1e30f)
        {
            // Half precision of the level above the cell: 2^-11 of it, and float rounding.
            const float Err = std::fabs(Wm - Water[i]), Above = std::fabs(Water[i] - P.Height[i]);
            WorstLake = std::max(WorstLake, Err);
            Lakes++;
            WaterBad += Err > Above * (1.0f / 2048.0f) + std::fabs(Water[i]) * 1.0e-6f + 1.0e-4f;
        }
    }
    int OctBad = 0;
    for (size_t o = 0; o < Gpu.size(); o++)
    {
        unsigned char A[80];
        MaterialOctave(Tex, (uint32_t)o, A);
        OctBad += std::memcmp(A, &Gpu[o], 80) != 0;   // every field, the padding zero on both
    }
    printf("decode: attributes wrong %d of %d, lake levels wrong %d (of %d; worst %.4f m), octaves wrong %d of %zu\n", AttrBad, Cells, WaterBad, Lakes, WorstLake, OctBad, Gpu.size());

    // The full land chain through both accessors.
    std::mt19937 Rng(31);
    std::normal_distribution<double> G;
    int Mismatch = 0, Count = 0, ColorBad = 0, HeightChanged = 0;
    for (int i = 0; i < 20000; i++)
    {
        double D[3] = { G(Rng), G(Rng), G(Rng) }; Normalize(D);
        const float F = (float)(P.Finest * (1 << (i % 10)));
        const float2 X = DF(D[0] * P.SeaRadius), Y = DF(D[1] * P.SeaRadius), Z = DF(D[2] * P.SeaRadius);
        const VoxelShader::FVoxelLandHeight A = VoxelShader::VoxelLandHeightDF(X, Y, Z, F, true);
        const float XA[2] = { X.x, X.y }, YA[2] = { Y.x, Y.y }, ZA[2] = { Z.x, Z.y };
        const FMaterialHeight B = MaterialLandHeight(Tex, XA, YA, ZA, F, true);
        const bool bSame = A.Height.x == B.Height[0] && A.Height.y == B.Height[1] && A.Bake == B.Bake && A.Gully == B.Gully && A.Detail == B.Detail
            && A.Flow == B.Flow && A.GullyScale == B.GullyScale && A.DetailScale == B.DetailScale
            && A.BakeGradient.x == B.BakeGradient[0] && A.BakeGradient.y == B.BakeGradient[1] && A.BakeGradient.z == B.BakeGradient[2]
            && A.Warped.x == B.Warped[0] && A.Warped.y == B.Warped[1] && A.Warped.z == B.Warped[2]
            && A.DetailGradient.x == B.DetailGradient[0] && A.DetailGradient.y == B.DetailGradient[1] && A.DetailGradient.z == B.DetailGradient[2]
            && A.DetailUnit == B.DetailUnit && A.DetailAmplitudeM == B.DetailAmplitudeM;
        // The exact detail path (bGradient) leaves the height as the density's own path has it.
        const VoxelShader::FVoxelLandHeight A0 = VoxelShader::VoxelLandHeightDF(X, Y, Z, F, false);
        HeightChanged += A0.Height.x != A.Height.x || A0.Height.y != A.Height.y || A0.Detail != A.Detail;
        if (!bSame && Mismatch < 5)
            printf("  differs at %d: height %.9g/%.9g vs %.9g/%.9g, bake %.9g vs %.9g, gully %.9g vs %.9g, detail %.9g vs %.9g\n", i,
                A.Height.x, A.Height.y, B.Height[0], B.Height[1], A.Bake, B.Bake, A.Gully, B.Gully, A.Detail, B.Detail);
        Mismatch += !bSame;
        Count++;
        // Every debug view's colour is finite and in range.
        for (uint32_t Mode = 1; Mode < 12; Mode++)
        {
            float C[3];
            MaterialDebugColor(Tex, Mode, B, F * 0.01f, C);
            for (float V : C) ColorBad += !(std::isfinite(V) && V >= 0.0f && V <= 1.001f);
        }
    }
    printf("land chain through the material's accessors against the global shaders': %d of %d points differ in any bit; %d debug colours out of range; height with the exact detail gradient against without: %d differ\n", Mismatch, Count, ColorBad, HeightChanged);

    // The pixel's planet-frame position from the view (VoxelLandFramePoint), against doubles:
    // planets anywhere within 10^9 cm of the world origin, rotated, cameras near their surface.
    {
        std::mt19937 R2(41);
        std::uniform_real_distribution<double> U(-1.0, 1.0);
        double WorstFloatAxes = 0, WorstTrue = 0;
        for (int i = 0; i < 20000; i++)
        {
            const double Origin[3] = { U(R2) * 1e9, U(R2) * 1e9, U(R2) * 1e9 };
            // A random rotation: axes from a random unit quaternion.
            double Qt[4] = { U(R2), U(R2), U(R2), U(R2) }; const double Ql = std::sqrt(Qt[0]*Qt[0]+Qt[1]*Qt[1]+Qt[2]*Qt[2]+Qt[3]*Qt[3]);
            for (double& q : Qt) q /= Ql;
            const double w = Qt[0], x = Qt[1], y = Qt[2], z = Qt[3];
            const double Ax[3][3] = { { 1 - 2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y) }, { 2*(x*y-w*z), 1 - 2*(x*x+z*z), 2*(y*z+w*x) }, { 2*(x*z+w*y), 2*(y*z-w*x), 1 - 2*(x*x+y*y) } };
            double Dir[3] = { U(R2), U(R2), U(R2) }; Normalize(Dir);
            double Cam[3], Pix[3];
            for (int a = 0; a < 3; a++) { Cam[a] = Origin[a] + Dir[a] * (P.SeaRadius + 3e5); }
            const double Off[3] = { U(R2) * 1e5, U(R2) * 1e5, U(R2) * 1e5 };
            for (int a = 0; a < 3; a++) Pix[a] = Cam[a] + Off[a];
            float TW[3], PH[3], PL[3], OH[3], OL[3], AF[3][3], AL[3][3];
            for (int a = 0; a < 3; a++)
            {
                const double Pv = -Cam[a];
                PH[a] = (float)Pv; PL[a] = (float)(Pv - (double)PH[a]);
                OH[a] = (float)Origin[a]; OL[a] = (float)(Origin[a] - (double)OH[a]);
                TW[a] = (float)(Pix[a] + Pv);
                for (int b = 0; b < 3; b++) { AF[a][b] = (float)Ax[a][b]; AL[a][b] = (float)(Ax[a][b] - (double)AF[a][b]); }
            }
            double Out[3];
            MaterialFramePoint(TW, PH, PL, OH, OL, AF[0], AF[1], AF[2], AL[0], AL[1], AL[2], Out);
            for (int a = 0; a < 3; a++)
            {
                // The world position the GPU has: the translated float, back in world.
                double ExactF = 0, ExactT = 0;
                for (int b = 0; b < 3; b++)
                {
                    const double Dw = ((double)TW[b] - ((double)PH[b] + PL[b])) - Origin[b];
                    ExactF += Dw * ((double)AF[a][b] + AL[a][b]);
                    ExactT += Dw * Ax[a][b];
                }
                WorstFloatAxes = std::max(WorstFloatAxes, std::fabs(Out[a] - ExactF));
                WorstTrue = std::max(WorstTrue, std::fabs(Out[a] - ExactT));
            }
        }
        printf("pixel in the planet frame from the view: worst %.2g cm against its double-float axes in doubles, %.2g cm against the exact rotation\n",
            WorstFloatAxes, WorstTrue);
    }

    // Close up: Gully + Detail along tangent lines in 1 cm steps at a 1 cm footprint, the
    // material's path against the global one (a staircase there is what screen-space
    // derivatives turn into speckle).
    for (int Line = 0; Line < 4; Line++)
    {
        double D[3] = { 0.013 + 0.31 * Line, 0.021 - 0.17 * Line, 1.0 - 0.2 * Line }; Normalize(D);
        double E1[3], E2[3]; Frame(D, E1, E2);
        float Worst = 0; int Flat = 0, Steps = 0; double SumD2A = 0, SumD2B = 0;
        float PrevA = 0, PrevB = 0, PrevA2 = 0, PrevB2 = 0;
        for (int k = 0; k < 2000; k++)
        {
            double Q[3]; for (int a = 0; a < 3; a++) Q[a] = D[a] * P.SeaRadius + E1[a] * (k * 1.0);
            const float2 X = DF(Q[0]), Y = DF(Q[1]), Z = DF(Q[2]);
            const VoxelShader::FVoxelLandHeight A = VoxelShader::VoxelLandHeightDF(X, Y, Z, 1.0f, true);
            const float XA[2] = { X.x, X.y }, YA[2] = { Y.x, Y.y }, ZA[2] = { Z.x, Z.y };
            const FMaterialHeight B = MaterialLandHeight(Tex, XA, YA, ZA, 1.0f, true);
            const float RA = A.Gully + A.Detail, RB = B.Gully + B.Detail;
            Worst = std::max(Worst, std::fabs(RA - RB));
            if (k > 0) { Steps++; Flat += RB == PrevB; }
            if (k > 1) { const double a2 = RA - 2 * PrevA + PrevA2, b2 = RB - 2 * PrevB + PrevB2; SumD2A += a2 * a2; SumD2B += b2 * b2; }
            PrevA2 = PrevA; PrevB2 = PrevB; PrevA = RA; PrevB = RB;
        }
        printf("close-up line %d (up %.2f %.2f %.2f): gully+detail worst difference %.4f m, %d of %d 1 cm steps unchanged, rms 2nd difference %.3g mm (global %.3g mm)\n",
            Line, D[0], D[1], D[2], Worst, Flat, Steps, 1000 * std::sqrt(SumD2B / (Steps - 1)), 1000 * std::sqrt(SumD2A / (Steps - 1)));
    }
    return Mismatch || AttrBad || WaterBad || OctBad || ColorBad || HeightChanged ? 1 : 0;
}
