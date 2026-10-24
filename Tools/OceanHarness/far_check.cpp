// far_check.cpp - The far ocean's light, term by term (stage W6a; Design/DistantOcean.md): a pixel
// whose waves are all unresolved (the mean normal, every slope in the roughness; past ledger_check's
// 30 m footprints), shaded as VoxelOceanSurfacePass.usf shades it, through VoxelOceanShading.ush
// compiled as C++:
// 1. the equivalent albedo (pi radiance over the sun's irradiance on the horizontal) of each term
//    at views from nadir to the limb: the sky's reflection (StandInSky at the mean normal's
//    reflection times the mean Fresnel), the sun's glint, the water's body (our single scattering),
//    Godot's water light (GodotWaterLight 1, in the body's place), the foam (its coverage times its
//    albedo, Lambertian);
// 2. the plane albedo of each (the equivalent albedo over every view, cos weighted): the number to
//    hold against the measured clear-sky ocean's (about 0.06 at a high sun: Payne 1972, Jin et al.
//    2004; the sky's share included);
// 3. the sky's reflection against a Monte Carlo of the facets the eye sees (each reflecting the sky
//    in its own mirror direction, Smith-masked): what W6c (the sky over the slopes) would change.
// At physical heights and at the Godot preset's (2.83 x the heights: 8 x the slope variance), with
// the slope variance ledger_check measures (the bands' spectrum: 0.0308 at 10 m/s, 0.0381 at 18).
// A report: its checks only the integration's own consistency.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Shaders/Private far_check.cpp -o far_check

#include "VoxelShaderCompat.h"
namespace VoxelShader
{
#include "VoxelOceanShading.ush"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

using namespace VoxelShader;

static int Fails = 0;
static void Check(bool bOk, const char* What, double Value, double Limit)
{
    std::printf("  [%s] %-80s %.6g (limit %.6g)\n", bOk ? "ok" : "FAIL", What, Value, Limit);
    if (!bOk) Fails++;
}

namespace
{
    constexpr double Pi = 3.14159265358979323846;
    struct FRGB { double R = 0, G = 0, B = 0; };
    double Luma(const FRGB& C) { return 0.2126 * C.R + 0.7152 * C.G + 0.0722 * C.B; }

    // The defaults (VoxelOceanSettings.h).
    const FRGB SkyZenith = { 0.035, 0.07, 0.14 }, SkyHorizon = { 0.09, 0.11, 0.13 };
    const FRGB Absorption = { 0.35, 0.07, 0.03 }, Scattering = { 0.02, 0.03, 0.035 };   // per m
    const FRGB GodotColor = { 0.0103, 0.0194, 0.0273 };
    const FRGB FoamAlbedo = { 0.492, 0.407, 0.342 };
    constexpr double PhaseG = 0.8;
    constexpr double ShortWave = 0.001;     // ShortWaveSlopeVariance

    struct FV { double X, Y, Z; };
    FV Make(double X, double Y, double Z) { return { X, Y, Z }; }
    double Dot(FV A, FV B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
    FV Norm(FV A) { const double L = std::sqrt(Dot(A, A)); return { A.X / L, A.Y / L, A.Z / L }; }
    float3 F3(FV A) { return float3(float(A.X), float(A.Y), float(A.Z)); }
    FV Reflect(FV I, FV N) { const double D = 2.0 * Dot(I, N); return { I.X - D * N.X, I.Y - D * N.Y, I.Z - D * N.Z }; }
    // HLSL's refract (I toward the surface, Eta n1/n2); zero past the critical angle.
    FV Refract(FV I, FV N, double Eta)
    {
        const double C = Dot(N, I);
        const double K = 1.0 - Eta * Eta * (1.0 - C * C);
        if (K < 0.0) return { 0, 0, 0 };
        const double S = Eta * C + std::sqrt(K);
        return { Eta * I.X - S * N.X, Eta * I.Y - S * N.Y, Eta * I.Z - S * N.Z };
    }

    FRGB Sky(FV Dir)
    {
        const double H = std::sqrt(std::clamp(Dir.Z, 0.0, 1.0));
        return { SkyHorizon.R + (SkyZenith.R - SkyHorizon.R) * H, SkyHorizon.G + (SkyZenith.G - SkyHorizon.G) * H, SkyHorizon.B + (SkyZenith.B - SkyHorizon.B) * H };
    }
    FRGB SkyIrradiance() { return { Pi * (0.2 * SkyHorizon.R + 0.8 * SkyZenith.R), Pi * (0.2 * SkyHorizon.G + 0.8 * SkyZenith.G), Pi * (0.2 * SkyHorizon.B + 0.8 * SkyZenith.B) }; }

    struct FTerms
    {
        double Sky = 0, Glint = 0, Body = 0, Godot = 0, Foam = 0;     // equivalent albedos (luma)
        double Fresnel = 0;
    };

    // The far pixel's terms (VoxelOceanSurfacePass.usf with N the mean normal), sun at zenith angle
    // ThetaS (azimuth 0), eye at ThetaV and azimuth Phi from the sun's.
    FTerms Shade(double S2, double ThetaS, double ThetaV, double Phi, double Coverage)
    {
        const FV Up = Make(0, 0, 1), Tx = Make(1, 0, 0), Ty = Make(0, 1, 0);
        const FV L = Make(std::sin(ThetaS), 0, std::cos(ThetaS));
        FV V = Make(std::sin(ThetaV) * std::cos(Phi), std::sin(ThetaV) * std::sin(Phi), std::cos(ThetaV));
        FV N = Up;
        const double CosS = L.Z;
        FTerms T;
        // Reflected.
        FV R = Reflect(Make(-V.X, -V.Y, -V.Z), N);
        if (R.Z < 0) R.Z = -R.Z;
        const double F = VoxelOceanMeanFresnel(float(Dot(V, N)), float(std::sqrt(S2)));
        T.Fresnel = F;
        const FRGB Sk = Sky(R);
        T.Sky = Pi * Luma(Sk) * F / CosS;
        const double SunAbove = VoxelOceanSunAbove(float(CosS));
        const double G = SunAbove * std::min(double(VoxelOceanGlint(F3(L), F3(V), F3(N), F3(Tx), F3(Ty), float(S2), float(S2))), 1000.0);
        T.Glint = Pi * G / CosS;
        // The body (deep water).
        const double Eta = 1.0 / VOXEL_OCEAN_WATER_IOR;
        const FV Tv = Refract(Make(-V.X, -V.Y, -V.Z), N, Eta);
        const double MuView = std::max(-Tv.Z, 0.05);
        const FV Ts = Refract(Make(-L.X, -L.Y, -L.Z), Up, Eta);
        const double MuSun = std::max(-Ts.Z, 0.05);
        const double SunIn = SunAbove * (1.0 - VoxelOceanFresnel(float(CosS))) * std::max(CosS, 0.0) / MuSun;
        const double SunPhase = VoxelOceanPhaseHG(float(Ts.X * -Tv.X + Ts.Y * -Tv.Y + Ts.Z * -Tv.Z), float(PhaseG));
        const FRGB SI = SkyIrradiance();
        const double Out = (1.0 - F) / (VOXEL_OCEAN_WATER_IOR * VOXEL_OCEAN_WATER_IOR);
        auto Body = [&](double S, double A, double SkyI) {
            const double Sc = S * 0.01, Ex = (A + S) * 0.01;
            return double(VoxelOceanWaterBody(float(Sc), float(Ex), -1.0f, float(MuView), float(MuSun), float(SunPhase), float(SunIn), float(SkyI * (1.0 - VOXEL_OCEAN_DIFFUSE_FRESNEL)), 0.0f)) * Out;
        };
        const FRGB B = { Body(Scattering.R, Absorption.R, SI.R), Body(Scattering.G, Absorption.G, SI.G), Body(Scattering.B, Absorption.B, SI.B) };
        T.Body = Pi * Luma(B) / CosS;
        // Godot's water light (its constant terms; the crest's height term at the mean, 2.5 m).
        {
            const double NV = std::max(Dot(N, V), 2.0e-5), NL = std::max(Dot(N, L), 2.0e-5);
            const double MaskA = 0.4 / (NV * std::sqrt(1.0 - 0.16));
            const double LightMask = MaskA < 1.6 ? (1.0 - 1.259 * MaskA + 0.396 * MaskA * MaskA) / (3.535 * MaskA + 2.181 * MaskA * MaskA) : 0.0;
            const double Back = std::max(-Dot(L, V), 0.0);
            const double Away = 0.5 - 0.5 * Dot(L, N);
            const double SssHeight = 2.5 * Back * Back * Back * Back * Away * Away * Away;
            const double SssNear = 0.5 * NV * NV;
            const double Sss = (SssHeight + SssNear) / (1.0 + LightMask);
            const FRGB Tint = { 0.9, 1.15, 0.85 };
            auto Ch = [&](double C, double Ti, double SkyI) { return C * (SunAbove * (Sss * Ti + 0.5 * NL) + SkyI / Pi) * (1.0 - F); };
            const FRGB GL = { Ch(GodotColor.R, Tint.R, SI.R), Ch(GodotColor.G, Tint.G, SI.G), Ch(GodotColor.B, Tint.B, SI.B) };
            T.Godot = Pi * Luma(GL) / CosS;
        }
        // The foam: Lambertian, its coverage.
        T.Foam = Coverage * Luma(FoamAlbedo) * (SunAbove * CosS + Luma(SI)) / CosS;
        return T;
    }

    // The plane albedo: the equivalent albedo over every view, cos weighted (1/pi int A cos dOmega).
    FTerms PlaneAlbedo(double S2, double ThetaS, double Coverage)
    {
        FTerms Sum;
        const int NT = 180, NP = 180;
        for (int i = 0; i < NT; i++)
        {
            const double Th = (i + 0.5) / NT * 0.5 * Pi;
            for (int j = 0; j < NP; j++)
            {
                const double Ph = (j + 0.5) / NP * 2.0 * Pi;
                const FTerms T = Shade(S2, ThetaS, Th, Ph, Coverage);
                const double W = std::cos(Th) * std::sin(Th) * (0.5 * Pi / NT) * (2.0 * Pi / NP) / Pi;
                Sum.Sky += T.Sky * W; Sum.Glint += T.Glint * W; Sum.Body += T.Body * W; Sum.Godot += T.Godot * W; Sum.Foam += T.Foam * W;
            }
        }
        return Sum;
    }

    // The sky's reflection over the facets the eye sees (Monte Carlo): slopes drawn from the
    // Gaussian, each facet weighed by its area toward the eye and Smith's masking, reflecting the
    // sky in its own mirror direction with its own Fresnel; a reflection into the sea (or masked by
    // the next wave, Smith's G1 on the way out) left out: Masked. Unmasked: as the shader folds it.
    void SkyReference(double S2, double ThetaV, double& Unmasked, double& Masked, double& MeanF)
    {
        std::mt19937 Rng(11);
        std::normal_distribution<double> Gauss(0.0, std::sqrt(S2));
        const FV V = Make(-std::sin(ThetaV), 0, std::cos(ThetaV));
        double WSum = 0, U = 0, M = 0, FS = 0;
        const int Count = 400000;
        for (int n = 0; n < Count; n++)
        {
            const double Sx = Gauss(Rng), Sy = Gauss(Rng);
            const FV Nf = Norm(Make(-Sx, -Sy, 1.0));
            const double VN = Dot(V, Nf);
            if (VN <= 0) continue;
            const double W = VN / Nf.Z;     // the facet's area seen from the eye, per unit of the mean
            FV R = Reflect(Make(-V.X, -V.Y, -V.Z), Nf);
            const double F = VoxelOceanFresnel(float(VN));
            WSum += W;
            FS += W * F;
            const FV RF = R.Z < 0 ? Make(R.X, R.Y, -R.Z) : R;
            U += W * F * Luma(Sky(RF));
            if (R.Z > 0)
            {
                const double G1 = 1.0 / (1.0 + VoxelOceanLambda(float(R.Z), float(S2)));
                M += W * F * Luma(Sky(R)) * G1;
            }
        }
        Unmasked = Pi * U / WSum;
        Masked = Pi * M / WSum;
        MeanF = FS / WSum;
    }
}

int main()
{
    struct FCase { const char* Name; double Wind; double Total; double Coverage; };
    // The bands' slope variance (ledger_check), the coverage: the physical sea's (Monahan: about 1%
    // at 10 m/s, 7% at 18), the Godot preset's (foam_check section 3: 12-16% at 10 m/s; at 18 an
    // estimate).
    const FCase Cases[4] = {
        { "physical 10 m/s", 10.0, 0.0308, 0.01 },
        { "Godot    10 m/s", 10.0, 8.0 * 0.0308, 0.14 },
        { "physical 18 m/s", 18.0, 0.0381, 0.07 },
        { "Godot    18 m/s", 18.0, 8.0 * 0.0381, 0.30 },
    };
    const double Suns[3] = { 10.0, 40.0, 70.0 };
    const double Views[6] = { 0.0, 30.0, 60.0, 75.0, 85.0, 89.0 };
    const double Deg = Pi / 180.0;

    for (const FCase& C : Cases)
    {
        const double S2 = std::max(0.5 * (C.Total + ShortWave), 1.0e-5);
        std::printf("\n==== %s: slope variance %.4f (Cox-Munk clean 0.003 + 0.00512 U: %.4f), %.4f per axis; foam coverage %.0f%% ====\n",
            C.Name, C.Total + ShortWave, 0.003 + 0.00512 * C.Wind, S2, 100.0 * C.Coverage);
        for (double Sun : Suns)
        {
            const FTerms A = PlaneAlbedo(S2, Sun * Deg, C.Coverage);
            // (Over all of the light, the sun's and the sky's on the horizontal, as voxel.Ocean.DebugShade
            // draws it and as a satellite's albedo is: the sun's share of it.)
            const double SunShare = std::cos(Sun * Deg) / (std::cos(Sun * Deg) + Luma(SkyIrradiance()));
            std::printf("  sun %2.0f deg from zenith: plane albedo sky %.4f  glint %.4f  body %.4f  | Godot light %.4f  foam %.4f | ours %.4f, with Godot's light %.4f (over the sky's light too: %.4f, %.4f)\n",
                Sun, A.Sky, A.Glint, A.Body, A.Godot, A.Foam, A.Sky + A.Glint + A.Body + A.Foam, A.Sky + A.Glint + A.Godot + A.Foam,
                SunShare * (A.Sky + A.Glint + A.Body + A.Foam), SunShare * (A.Sky + A.Glint + A.Godot + A.Foam));
            std::printf("     %-24s %8s %8s %8s %8s %8s %8s\n", "view (deg; toward the sun's glint)", "sky", "glint", "body", "Godot", "foam", "Fresnel");
            for (double View : Views)
            {
                const FTerms T = Shade(S2, Sun * Deg, View * Deg, Pi, C.Coverage);
                std::printf("     %-24.0f %8.4f %8.4f %8.4f %8.4f %8.4f %8.4f\n", View, T.Sky, T.Glint, T.Body, T.Godot, T.Foam, T.Fresnel);
            }
        }
        std::printf("  the sky's reflection, the shader's (one direction, the mean Fresnel) against the facets' (Monte Carlo), sun 40 deg (equivalent albedo):\n");
        for (double View : Views)
        {
            double U = 0, M = 0, MF = 0;
            SkyReference(S2, View * Deg, U, M, MF);
            const FTerms T = Shade(S2, 40.0 * Deg, View * Deg, Pi, C.Coverage);
            const double Shader = T.Sky * std::cos(40.0 * Deg);   // (back to radiance pi L, the sun's term aside)
            std::printf("     view %4.0f: shader %.4f, facets unmasked %.4f (%+.0f%%), masked %.4f (%+.0f%%); mean Fresnel fit %.4f, facets' %.4f\n",
                View, Shader, U, 100.0 * (Shader / U - 1.0), M, 100.0 * (Shader / std::max(M, 1e-9) - 1.0), T.Fresnel, MF);
        }
    }

    // The integration's own consistency: a smooth surface's sky term against Fresnel times the sky.
    {
        const FTerms T = Shade(1.0e-5, 40.0 * Deg, 30.0 * Deg, Pi, 0.0);
        const FV R = Make(std::sin(30.0 * Deg), 0, std::cos(30.0 * Deg));
        const double Want = Pi * Luma(Sky(R)) * VoxelOceanFresnel(float(std::cos(30.0 * Deg))) / std::cos(40.0 * Deg);
        std::printf("\n");
        Check(std::abs(T.Sky / Want - 1.0) < 0.02, "a smooth sea reflects the sky by Schlick's Fresnel (within 2%)", T.Sky / Want, 1.0);
    }
    std::printf("\n%s (%d failures)\n", Fails == 0 ? "ALL OK" : "FAILED", Fails);
    return Fails == 0 ? 0 : 1;
}
