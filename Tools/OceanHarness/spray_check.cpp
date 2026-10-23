// spray_check.cpp - The spray's maths (stage W3x; VoxelOceanSpray.ush, GodotOceanWaves' sea spray as the
// GPU runs it), checked without the engine:
// - Godot's shaping: exp_impulse peaks at 1 at 1 / k;
// - the birth: none under the foam it rises from or on a surface too steep; the largest splash on the
//   densest, flattest foam (20 m by 8.5 m at size 1), a quarter of it just over the threshold; its
//   life within 2.25 to 3 s;
// - the motion: thrown from about half a metre under the surface to half a metre over it at a fifth
//   of its life, and sinking back under it (20 m by its end);
// - the shape: its width from nothing growing as log(1 + t), its height an impulse peaking at a third
//   of its life; a shorter-lived one smaller;
// - the look: the fade in over its first tenth and out after; the dissolve; the fade near the camera;
// - the budget: the splashes alive in a stormy view against the default ring.
//
//   g++ -O2 -std=c++20 -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Shaders/Private spray_check.cpp -o spray_check

#include "VoxelShaderCompat.h"

namespace VoxelShader
{
#include "VoxelOceanSpray.ush"
}

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace VoxelShader;

static int Failures = 0;
static void Expect(bool bOk, const char* What, double Value)
{
    std::printf("  [%s] %-74s %.4g\n", bOk ? "ok" : "FAIL", What, Value);
    if (!bOk) Failures++;
}

static FVoxelOceanSprayParticle At(FVoxelOceanSprayParticle P, float T)
{
    P.Age = T * P.Life;
    return P;
}

int main()
{
    std::printf("Godot's shaping:\n");
    {
        double Worst = 0.0;
        for (float K : { 3.0f, 10.0f })
        {
            Worst = std::max(Worst, std::fabs(VoxelOceanSprayImpulse(1.0f / K, K) - 1.0));
            for (float X = 0.0f; X <= 1.0f; X += 0.01f) Worst = std::max(Worst, double(VoxelOceanSprayImpulse(X, K)) - 1.0);
        }
        Expect(Worst < 1e-5, "exp_impulse(x, k) = k x e^(1 - k x) peaks at 1 where x = 1 / k", Worst);
    }

    std::printf("\nThe birth:\n");
    {
        const float3 Rest(0.0f, 0.0f, 0.0f);
        Expect(VoxelOceanSprayBirth(Rest, 0.85f, 1.0f, 0.9f, 1.0f, 0.5f, 0.5f).Life == 0.0f, "none under the foam it rises from (0.85 against 0.9)", 0.0);
        Expect(VoxelOceanSprayBirth(Rest, 1.2f, 0.85f, 0.9f, 1.0f, 0.5f, 0.5f).Life == 0.0f, "none on a surface too steep (its normal 0.85 of the vertical)", 0.0);
        const FVoxelOceanSprayParticle Big = VoxelOceanSprayBirth(Rest, 1.2f, 1.0f, 0.9f, 1.0f, 0.0f, 0.5f);
        Expect(std::fabs(Big.Width - 2000.0f) < 1e-3f && std::fabs(Big.Height - 850.0f) < 1e-3f, "the largest on the densest, flattest foam: 20 m by 8.5 m (cm)", Big.Width);
        const FVoxelOceanSprayParticle Small = VoxelOceanSprayBirth(Rest, 0.9001f, 1.0f, 0.9f, 1.0f, 0.0f, 0.5f);
        Expect(std::fabs(Small.Width / Big.Width - 0.25f) < 0.01f, "a quarter of it just over the threshold", Small.Width / Big.Width);
        const FVoxelOceanSprayParticle Steepish = VoxelOceanSprayBirth(Rest, 1.2f, 0.9f, 0.9f, 1.0f, 0.0f, 0.5f);
        Expect(Steepish.Life > 0.0f && Steepish.Height < 0.25f * Big.Height, "flattened on a steeper surface (its normal 0.9: under a quarter as tall)", Steepish.Height / Big.Height);
        const float L0 = VoxelOceanSprayBirth(Rest, 1.2f, 1.0f, 0.9f, 1.0f, 0.0f, 0.5f).Life, L1 = VoxelOceanSprayBirth(Rest, 1.2f, 1.0f, 0.9f, 1.0f, 1.0f, 0.5f).Life;
        Expect(std::fabs(L0 - 3.0f) < 1e-5f && std::fabs(L1 - 2.25f) < 1e-5f, "living 2.25 to 3 s (s)", L1);
        Expect(std::fabs(VoxelOceanSprayBirth(Rest, 1.2f, 1.0f, 0.9f, 2.0f, 0.0f, 0.5f).Width - 2.0f * Big.Width) < 1e-2f, "the size setting scales it", 2.0);
    }

    std::printf("\nThe motion (its rise over the surface it rides):\n");
    {
        const FVoxelOceanSprayParticle P = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), 1.2f, 1.0f, 0.9f, 1.0f, 0.0f, 0.5f);
        const float R0 = VoxelOceanSprayRise(At(P, 0.0f)), RPeak = VoxelOceanSprayRise(At(P, 0.18f)), REnd = VoxelOceanSprayRise(At(P, 1.0f));
        float Top = -1e9f, TopT = 0.0f;
        for (float T = 0.0f; T <= 1.0f; T += 0.001f) { const float R = VoxelOceanSprayRise(At(P, T)); if (R > Top) { Top = R; TopT = T; } }
        std::printf("  from %.0f cm, up to %.0f cm at %.2f of its life, to %.0f cm at its end\n", R0, Top, TopT, REnd);
        Expect(R0 < -40.0f && R0 > -60.0f, "starting about half a metre under the surface (cm)", R0);
        Expect(std::fabs(Top - 50.0f) < 0.5f && std::fabs(TopT - 0.18f) < 0.01f && std::fabs(RPeak - 50.0f) < 0.5f, "thrown up to half a metre over it at 0.18 of its life (cm)", Top);
        Expect(REnd < -1500.0f, "and sinking back under it (cm)", REnd);
    }

    std::printf("\nThe shape:\n");
    {
        const FVoxelOceanSprayParticle P = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), 1.2f, 1.0f, 0.9f, 1.0f, 0.0f, 0.5f);
        const float2 S0 = VoxelOceanSpraySizeNow(At(P, 0.0f)), SThird = VoxelOceanSpraySizeNow(At(P, 1.0f / 3.0f)), SEnd = VoxelOceanSpraySizeNow(At(P, 1.0f));
        Expect(S0.x == 0.0f && S0.y == 0.0f, "nothing at its birth", 0.0);
        Expect(std::fabs(SThird.y - P.Height) < 1.0f, "its height at its most at a third of its life (cm)", SThird.y);
        Expect(std::fabs(SEnd.x - P.Width * std::log(2.0f)) < 1.0f && SEnd.y < 0.45f * P.Height, "its width log 2 of its largest by its end, its height fallen under half (cm)", SEnd.x);
        const FVoxelOceanSprayParticle Short = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), 1.2f, 1.0f, 0.9f, 1.0f, 1.0f, 0.5f);
        Expect(std::fabs(VoxelOceanSpraySizeNow(At(Short, 0.5f)).x / VoxelOceanSpraySizeNow(At(P, 0.5f)).x - 0.5625f) < 1e-3f,
            "a shorter-lived one smaller: its life's share squared (0.75^2)", VoxelOceanSpraySizeNow(At(Short, 0.5f)).x / VoxelOceanSpraySizeNow(At(P, 0.5f)).x);
    }

    std::printf("\nThe look:\n");
    {
        const FVoxelOceanSprayParticle P = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), 1.2f, 1.0f, 0.9f, 1.0f, 0.0f, 0.5f);
        Expect(VoxelOceanSprayFade(At(P, 0.0f)) == 0.0f && std::fabs(VoxelOceanSprayFade(At(P, 0.1f)) - 1.0f) < 1e-5f && VoxelOceanSprayFade(At(P, 0.6f)) < 0.05f,
            "it fades in over its first tenth and out after", VoxelOceanSprayFade(At(P, 0.6f)));
        Expect(std::fabs(VoxelOceanSprayDissolve(1.0f, 0.5f, 0.25f) - 0.5f) < 1e-6f && VoxelOceanSprayDissolve(0.2f, 0.2f, 0.5f) == 0.0f,
            "the dissolve: max((fade + seed) / 2 - noise, 0)", VoxelOceanSprayDissolve(1.0f, 0.5f, 0.25f));
        Expect(std::fabs(VoxelOceanSprayNear(2500.0f) - (1.0f - std::exp(-1.0f))) < 1e-5f && VoxelOceanSprayNear(0.0f) == 0.0f,
            "faded near the camera (1 - e^(-0.04 d): 0.63 at 25 m)", VoxelOceanSprayNear(2500.0f));
        FVoxelOceanSprayParticle Q = P;
        Q = VoxelOceanSprayAge(Q, 2.0f);
        Expect(Q.Life > 0.0f && std::fabs(Q.Age - 2.0f) < 1e-6f, "it ages", Q.Age);
        Q = VoxelOceanSprayAge(Q, 1.5f);
        Expect(Q.Life == 0.0f && VoxelOceanSprayFade(Q) == 0.0f, "and is gone at its life's end", 0.0);
    }

    std::printf("\nThe budget (informational and checked):\n");
    {
        // Born within half the reach (150 m at 300 m), on dense foam over 30% of the sea (a storm),
        // living 2.625 s on average: the splashes alive and born a frame at 60 Hz.
        const double Area = 3.14159265 * 150.0 * 150.0, Dense = 0.3, Life = 2.625;
        const double Rate = VOXEL_OCEAN_SPRAY_CANDIDATES * Dense * Area;
        const double Alive = Rate * Life;
        std::printf("  a storm's view: %.0f born a second, %.0f alive\n", Rate, Alive);
        Expect(Alive < 64000.0, "the default ring (64,000) holds a storm's splashes", Alive);
        Expect(Rate / 60.0 < 8000.0, "within the frame's cap (an eighth of the ring)", Rate / 60.0);
    }

    std::printf("\n%s (%d failures)\n", Failures == 0 ? "ALL OK" : "FAILED", Failures);
    return Failures ? 1 : 0;
}
