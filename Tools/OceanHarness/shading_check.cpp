// shading_check.cpp - The ocean's light model (stage W3; VoxelOceanShading.ush) compiled as C++
// through VoxelShaderCompat.h, against references computed independently:
//
// 1. The sun's glitter without masking: its hemispherical reflectance (the shader's function
//    integrated over every eye direction) against a Monte Carlo of mirror facets drawn from the
//    same Gaussian slopes (each facet's intercepted light times its Fresnel factor). This checks
//    the slope density's normalisation and the 4 cos V cos^4 H factor.
// 2. Smith's Lambda (Walter et al.'s rational fit) against the exact Gaussian form with erfc.
// 3. The mean Fresnel factor (Bruneton's fit) against a Monte Carlo of the facets the eye sees,
//    weighed by their projected area. The fit is approximate: reported, with a loose bound.
// 4. The water's single scattering (VoxelOceanWaterBody) against a numerical march down the ray.
//
//   g++ -O2 -std=c++20 -I../../Source/VoxelPlugin/Public -I../../Shaders/Private shading_check.cpp -o shading_check && ./shading_check

#include "VoxelShaderCompat.h"
namespace VoxelShader
{
#include "VoxelOceanShading.ush"
}

#include <cmath>
#include <cstdio>
#include <random>

using namespace VoxelShader;

static int Failures = 0;

static void Expect(bool bOk, const char* What)
{
    if (!bOk)
    {
        std::printf("FAIL: %s\n", What);
        Failures++;
    }
}

static float3 Normalize3(double X, double Y, double Z)
{
    const double L = std::sqrt(X * X + Y * Y + Z * Z);
    return float3((float)(X / L), (float)(Y / L), (float)(Z / L));
}

int main()
{
    const float3 N(0.0f, 0.0f, 1.0f);
    const float3 Tx(1.0f, 0.0f, 0.0f);
    const float3 Ty(0.0f, 1.0f, 0.0f);
    std::mt19937_64 Rng(12345);
    std::normal_distribution<double> Gauss(0.0, 1.0);

    // -- 1. The glitter's reflectance -----------------------------------------------------------
    {
        double Worst = 0.0;
        for (double ThetaL : { 0.0, 30.0, 60.0, 75.0 })
        {
            const double TL = ThetaL * M_PI / 180.0;
            const float3 L((float)std::sin(TL), 0.0f, (float)std::cos(TL));
            for (double S2 : { 0.001, 0.005, 0.02, 0.05 })
            {
                const double Sigma = std::sqrt(S2);
                // The shader's function over every eye direction, in the half vector's slope
                // space: dw_V = 4 (V.H) cos^3(H) dzeta.
                const int Steps = 600;
                const double Range = 7.0 * Sigma;
                const double D = 2.0 * Range / Steps;
                double Integral = 0.0;
                for (int j = 0; j < Steps; j++)
                    for (int i = 0; i < Steps; i++)
                    {
                        const double ZX = -Range + (i + 0.5) * D;
                        const double ZY = -Range + (j + 0.5) * D;
                        const float3 H = Normalize3(-ZX, -ZY, 1.0);
                        const double LH = dot(L, H);
                        if (LH <= 0.0) continue;
                        const float3 V((float)(2.0 * LH * H.x - L.x), (float)(2.0 * LH * H.y - L.y), (float)(2.0 * LH * H.z - L.z));
                        if (V.z <= 0.0f) continue;
                        const double G = VoxelOceanGlintUnmasked(L, normalize(V), N, Tx, Ty, (float)S2, (float)S2);
                        const double CosH = H.z;
                        Integral += G * V.z * 4.0 * LH * CosH * CosH * CosH * D * D;
                    }
                const double Analytic = Integral / L.z;
                // Mirror facets: each intercepts the beam by its area toward the sun, reflects its
                // Fresnel share into the upper hemisphere.
                const int Samples = 2000000;
                double Sum = 0.0;
                for (int k = 0; k < Samples; k++)
                {
                    const float3 M = Normalize3(-Sigma * Gauss(Rng), -Sigma * Gauss(Rng), 1.0);
                    const double LM = dot(L, M);
                    if (LM <= 0.0) continue;
                    const double VZ = 2.0 * LM * M.z - L.z;
                    if (VZ <= 0.0) continue;
                    Sum += VoxelOceanFresnel((float)LM) * LM / M.z;
                }
                const double Reference = Sum / Samples / L.z;
                const double Error = std::fabs(Analytic - Reference) / Reference;
                Worst = std::max(Worst, Error);
                std::printf("glint  sun %4.0f deg  var %.3f: shader %.5f  facets %.5f  (%.2f%%)\n", ThetaL, S2, Analytic, Reference, 100.0 * Error);
            }
        }
        Expect(Worst < 0.02, "the glitter's reflectance within 2% of the facets'");
    }

    // -- 2. Smith's Lambda ------------------------------------------------------------------------
    {
        double Worst = 0.0;
        for (double S2 : { 0.001, 0.01, 0.05, 0.2 })
            for (int k = 1; k < 200; k++)
            {
                const double C = k / 200.0;
                const double A = C / std::sqrt((1.0 - C * C) * 2.0 * S2);
                const double Exact = std::max(0.0, (std::exp(-A * A) - A * std::sqrt(M_PI) * std::erfc(A)) / (2.0 * A * std::sqrt(M_PI)));
                const double Fit = VoxelOceanLambda((float)C, (float)S2);
                Worst = std::max(Worst, std::fabs(Fit - Exact) / (1.0 + Exact));
            }
        std::printf("lambda fit: worst error %.4f (relative to 1 + Lambda)\n", Worst);
        Expect(Worst < 0.01, "Lambda's fit within 1% of 1 + Lambda");
    }

    // -- 3. The mean Fresnel factor ------------------------------------------------------------------
    {
        double Worst = 0.0;
        for (double CosV : { 1.0, 0.7, 0.4, 0.2, 0.1, 0.05 })
            for (double Sigma : { 0.0, 0.05, 0.1, 0.2, 0.3 })
            {
                const double SinV = std::sqrt(1.0 - CosV * CosV);
                const float3 V((float)SinV, 0.0f, (float)CosV);
                double Reference = VoxelOceanFresnel((float)CosV);
                if (Sigma > 0.0)
                {
                    double Num = 0.0, Den = 0.0;
                    for (int k = 0; k < 400000; k++)
                    {
                        const float3 M = Normalize3(-Sigma * Gauss(Rng), -Sigma * Gauss(Rng), 1.0);
                        const double VM = dot(V, M);
                        if (VM <= 0.0) continue;
                        const double W = VM / M.z;
                        Num += W * VoxelOceanFresnel((float)VM);
                        Den += W;
                    }
                    Reference = Num / Den;
                }
                const double Fit = VoxelOceanMeanFresnel((float)CosV, (float)Sigma);
                Worst = std::max(Worst, std::fabs(Fit - Reference));
                std::printf("fresnel cos %.2f sigma %.2f: fit %.4f  facets %.4f\n", CosV, Sigma, Fit, Reference);
            }
        std::printf("mean fresnel fit: worst error %.4f\n", Worst);
        Expect(Worst < 0.08, "the mean Fresnel fit within 0.08 of the facets'");
    }

    // -- 4. The water's single scattering ---------------------------------------------------------------
    {
        std::uniform_real_distribution<double> U(0.0, 1.0);
        double Worst = 0.0;
        for (int Case = 0; Case < 200; Case++)
        {
            const double Sca = 0.0001 + 0.001 * U(Rng);       // per cm
            const double Ext = Sca + 0.004 * U(Rng);
            const double MuV = 0.66 + 0.34 * U(Rng);
            const double MuS = 0.66 + 0.34 * U(Rng);
            const double Phase = 0.01 + 0.3 * U(Rng);
            const double Esun = 1.0 + 10.0 * U(Rng);
            const double Esky = 0.5 * U(Rng);
            const double Bed = U(Rng);
            const double Path = Case % 4 == 0 ? -1.0 : 5000.0 * U(Rng);
            const double Shader = VoxelOceanWaterBody((float)Sca, (float)Ext, (float)Path, (float)MuV, (float)MuS, (float)Phase, (float)Esun, (float)Esky, (float)Bed);
            // The march: at x along the eye's ray (depth x MuV) the sun's beam has come x MuV / MuS,
            // the sky's diffuse light x MuV / 0.75; the light scattered at x comes back x.
            const double End = Path < 0.0 ? 40.0 / Ext : Path;
            const int Steps = 200000;
            const double Dx = End / Steps;
            double March = 0.0;
            for (int k = 0; k < Steps; k++)
            {
                const double X = (k + 0.5) * Dx;
                const double Back = std::exp(-Ext * X);
                const double SunIn = Esun * std::exp(-Ext * X * MuV / MuS);
                const double SkyIn = (Esky / 0.75) * std::exp(-Ext * X * MuV / 0.75);
                March += Sca * (Phase * SunIn + SkyIn / (4.0 * M_PI)) * Back * Dx;
            }
            if (Path >= 0.0) March += Bed * std::exp(-Ext * Path) * std::exp(-Ext * Path * MuV / MuS);
            const double Error = std::fabs(Shader - March) / std::max(March, 1.0e-6);
            Worst = std::max(Worst, Error);
        }
        std::printf("water body: worst error %.5f against the march (200 cases)\n", Worst);
        Expect(Worst < 1.0e-3, "the water's light within 0.1% of the march");
    }

    std::printf(Failures == 0 ? "shading_check: all passed\n" : "shading_check: %d failed\n", Failures);
    return Failures == 0 ? 0 : 1;
}
