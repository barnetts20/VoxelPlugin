// spray_check.cpp - The spray's maths (stage W3o; VoxelOceanSpray.ush, as the GPU runs it), checked
// without the engine:
// - the step is exact for linear drag: one step of 0.1 s against 10,000 small ones;
// - a puff takes the wind's speed (spume within a second) and falls at its terminal speed;
// - the trajectories: how high spume and splashes rise, how far the wind carries spume;
// - the birth rates: none in light air, the spume's rising from 8 to 20 m/s, the splashes';
// - the look: the size grows, the opacity fades in and out;
// - the budget: the particles alive in a stormy view against the default ring (32,768).
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
    std::printf("  [%s] %-70s %.4g\n", bOk ? "ok" : "FAIL", What, Value);
    if (!bOk) Failures++;
}

static double Len(float3 V) { return std::sqrt(double(V.x) * V.x + double(V.y) * V.y + double(V.z) * V.z); }

int main()
{
    const float3 Up(0.0f, 0.0f, 1.0f), Down(0.0f, 0.0f, -1.0f);
    const float3 Wind(2000.0f, 0.0f, 0.0f);     // 20 m/s along x

    std::printf("The step:\n");
    {
        double Worst = 0.0;
        for (float Kind : { 0.0f, 1.0f })
        {
            FVoxelOceanSprayParticle A = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), Up, Wind, float3(0.0f, 1.0f, 0.0f), Kind, 0.5f, 0.5f, 0.5f, 0.5f);
            A.Life = 100.0f;
            FVoxelOceanSprayParticle B = A;
            A = VoxelOceanSprayStep(A, Wind, Down, 0.1f);
            // The reference: 10,000 explicit steps of the same ODE in double.
            double P[3] = { B.Position.x, B.Position.y, B.Position.z }, V[3] = { B.Velocity.x, B.Velocity.y, B.Velocity.z };
            const double Tau = VoxelOceanSprayTau(Kind), G = VOXEL_OCEAN_SPRAY_GRAVITY * VoxelOceanSprayFall(Kind);
            const double W[3] = { Wind.x * VOXEL_OCEAN_SPRAY_WIND_SHARE, Wind.y * VOXEL_OCEAN_SPRAY_WIND_SHARE, Wind.z * VOXEL_OCEAN_SPRAY_WIND_SHARE };
            const int N = 10000;
            const double H = 0.1 / N;
            for (int i = 0; i < N; i++)
                for (int c = 0; c < 3; c++)
                {
                    const double Acc = (W[c] - V[c]) / Tau - (c == 2 ? G : 0.0);
                    P[c] += V[c] * H + 0.5 * Acc * H * H;
                    V[c] += Acc * H;
                }
            const double DP = std::sqrt((A.Position.x - P[0]) * (A.Position.x - P[0]) + (A.Position.y - P[1]) * (A.Position.y - P[1]) + (A.Position.z - P[2]) * (A.Position.z - P[2]));
            Worst = std::max(Worst, DP);
        }
        Expect(Worst < 0.05, "one 0.1 s step against 10,000 small ones (cm)", Worst);
    }

    std::printf("\nThe motion:\n");
    {
        FVoxelOceanSprayParticle S = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), Up, Wind, float3(0.0f, 0.0f, 0.0f), 0.0f, 0.5f, 0.0f, 0.5f, 0.5f);
        S.Life = 100.0f;
        for (int i = 0; i < 600; i++) S = VoxelOceanSprayStep(S, Wind, Down, 1.0f / 60.0f);
        const double Felt = Wind.x * VOXEL_OCEAN_SPRAY_WIND_SHARE;
        Expect(std::fabs(S.Velocity.x - Felt) < 0.03 * Felt, "spume takes the wind it feels within ten seconds (x share)", S.Velocity.x / Felt);
        FVoxelOceanSprayParticle Q = S;
        for (int i = 0; i < 600; i++) Q = VoxelOceanSprayStep(Q, float3(0.0f, 0.0f, 0.0f), Down, 1.0f / 60.0f);
        const double Fall = -Q.Velocity.z, Want = VOXEL_OCEAN_SPRAY_TAU0 * VOXEL_OCEAN_SPRAY_GRAVITY * VOXEL_OCEAN_SPRAY_FALL0;
        Expect(std::fabs(Fall - Want) < 0.01 * Want, "in still air it falls at its terminal speed (cm/s)", Fall);

        // Trajectories over their lives, the randoms at their middles and ends.
        double SpumeTop = 0.0, SpumeCarry = 0.0, SplashTop = 0.0, SplashLow = 1e9;
        for (float U : { 0.0f, 0.5f, 1.0f })
        {
            FVoxelOceanSprayParticle A = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), Up, Wind, float3(0.0f, 0.0f, 0.0f), 0.0f, U, U, U, U);
            double Top = 0.0;
            while (A.Life > 0.0f) { A = VoxelOceanSprayStep(A, Wind, Down, 1.0f / 60.0f); Top = std::max(Top, double(A.Position.z)); if (A.Life > 0.0f) SpumeCarry = std::max(SpumeCarry, double(A.Position.x)); }
            SpumeTop = std::max(SpumeTop, Top);
            FVoxelOceanSprayParticle B = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), Up, Wind, float3(0.0f, 1.0f, 0.0f), 1.0f, U, U, U, U);
            double TopB = 0.0;
            while (B.Life > 0.0f) { B = VoxelOceanSprayStep(B, Wind, Down, 1.0f / 60.0f); TopB = std::max(TopB, double(B.Position.z)); }
            SplashTop = std::max(SplashTop, TopB);
            SplashLow = std::min(SplashLow, TopB);
        }
        std::printf("  spume rises up to %.2f m and is carried up to %.1f m at 20 m/s; splashes rise %.2f to %.2f m\n",
            SpumeTop / 100.0, SpumeCarry / 100.0, SplashLow / 100.0, SplashTop / 100.0);
        // (Stage W3r: thrown up off the crest, then taken by the wind: W3q's drifted 7 m and rose
        // half a metre at most.)
        Expect(SpumeTop > 15.0 && SpumeTop < 90.0, "spume rises 15 to 90 cm (cm)", SpumeTop);
        Expect(SpumeCarry < 200.0, "the wind carries it under 2 m in a 20 m/s gale over its life (cm)", SpumeCarry);
        {
            // The uplift first: a fifth of a second in, it has risen more than it has drifted.
            FVoxelOceanSprayParticle A = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), Up, Wind, float3(0.0f, 0.0f, 0.0f), 0.0f, 1.0f, 0.5f, 0.5f, 0.5f);
            for (int i = 0; i < 12; i++) A = VoxelOceanSprayStep(A, Wind, Down, 1.0f / 60.0f);
            Expect(A.Position.z > A.Position.x, "uplift first: at 0.2 s it has risen more than drifted (20 m/s)", A.Position.z / std::max(A.Position.x, 1e-3f));
        }
        Expect(SplashLow > 5.0 && SplashTop < 120.0, "splashes rise 5 cm to 1.2 m (cm)", SplashTop);
    }

    std::printf("\nThe rates:\n");
    {
        Expect(VoxelOceanSpraySpumeRate(1.0f, 3.0f, 0.5f) == 0.0f, "no spume in light air (3 m/s)", VoxelOceanSpraySpumeRate(1.0f, 3.0f, 0.5f));
        const float R6 = VoxelOceanSpraySpumeRate(1.0f, 6.0f, 0.5f), R12 = VoxelOceanSpraySpumeRate(1.0f, 12.0f, 0.5f), R20 = VoxelOceanSpraySpumeRate(1.0f, 20.0f, 0.5f);
        std::printf("  per m^2 of whitecap per s: %.3f at 6 m/s, %.3f at 12, %.3f at 20\n", R6, R12, R20);
        Expect(R6 > 0.0f && R6 < R12 && R12 < R20, "rising with the wind", R20);
        Expect(std::fabs(R20 - (VOXEL_OCEAN_SPRAY_BREAK_RATE + VOXEL_OCEAN_SPRAY_SPUME_RATE)) < 1e-6f, "the full rate from 20 m/s", R20);
        Expect(VoxelOceanSpraySpumeRate(0.0f, 30.0f, 0.5f) == 0.0f, "none without whitecaps", 0.0);
        // Stage W3s: the core's threshold a setting: lower spawns from more of the whitecaps.
        Expect(VoxelOceanSpraySpumeRate(0.45f, 30.0f, 0.2f) > 0.5f * VoxelOceanSpraySpumeRate(1.0f, 30.0f, 0.2f), "SprayCore 0.2: most of the rate from density 0.45", VoxelOceanSpraySpumeRate(0.45f, 30.0f, 0.2f));
        Expect(VoxelOceanSpraySpumeRate(0.45f, 30.0f, 0.5f) == 0.0f, "none from a whitecap's margin (under the core's threshold, 0.5 here)", VoxelOceanSpraySpumeRate(0.45f, 30.0f, 0.5f));
        Expect(VoxelOceanSpraySplashRate(0.0f) == 0.0f && VoxelOceanSpraySplashRate(1.0f) == VOXEL_OCEAN_SPRAY_SPLASH_RATE, "splashes by the contact's core", VoxelOceanSpraySplashRate(1.0f));
    }

    std::printf("\nThe look:\n");
    {
        FVoxelOceanSprayParticle P = VoxelOceanSprayBirth(float3(0.0f, 0.0f, 0.0f), Up, Wind, float3(0.0f, 0.0f, 0.0f), 0.0f, 0.5f, 0.5f, 0.5f, 0.5f);
        const float Size0 = VoxelOceanSpraySize(P), Op0 = VoxelOceanSprayOpacity(P);
        float Peak = 0.0f, SizeEnd = 0.0f, OpLate = 1.0f;
        while (P.Life > 0.0f)
        {
            Peak = std::max(Peak, VoxelOceanSprayOpacity(P));
            SizeEnd = VoxelOceanSpraySize(P);
            if (P.Age > 0.9f * P.Life) OpLate = std::min(OpLate, VoxelOceanSprayOpacity(P));
            P = VoxelOceanSprayStep(P, Wind, Down, 1.0f / 120.0f);
        }
        Expect(Op0 == 0.0f && std::fabs(Peak - VOXEL_OCEAN_SPRAY_OPACITY0) < 0.01f, "fades in from nothing to its peak", Peak);
        Expect(OpLate < 0.3f * Peak, "and out by its end (under 0.3 of its peak in its last tenth)", OpLate / Peak);
        Expect(SizeEnd > 1.4f * Size0 && SizeEnd < 1.6f * Size0, "a puff spreads to 1.5 times its size", SizeEnd / Size0);
        Expect(VoxelOceanSprayOpacity(P) == 0.0f, "dead: nothing", 0.0);
    }

    std::printf("\nThe budget (informational and checked):\n");
    {
        // A 25 m/s storm: whitecaps cover 8.8% of the sea (stage W3i); the sea within the spawn's
        // reach (300 m by default) seen from near the surface, about half a disc.
        const double Area = 0.5 * 3.14159265 * 300.0 * 300.0;
        // (Stage W3r: from the whitecaps' cores; taken as all of their area, the most it can be.)
        const double Born = Area * 0.088 * VoxelOceanSpraySpumeRate(1.0f, 25.0f, 0.5f);
        const double Alive = Born * 0.275;  // the spume's mean life (0.15 + 0.25 / 2)
        std::printf("  a 25 m/s storm in view: at most %.0f puffs born a second, %.0f alive (ring 65,536; %.0f born a frame at 60 Hz, cap 8,192)\n", Born, Alive, Born / 60.0);
        Expect(Alive < 65536.0, "the default ring holds a storm's spray", Alive);
        Expect(Born / 60.0 < 8192.0, "within the frame's cap", Born / 60.0);
    }

    std::printf("\n%s (%d failures)\n", Failures == 0 ? "ALL OK" : "FAILED", Failures);
    return Failures == 0 ? 0 : 1;
}
