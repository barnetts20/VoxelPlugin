// surf_check.cpp - Stage O9a's shared code offline: the surf (VoxelOceanSurf.ush, the same
// function the wave hook and the CPU model run) on a uniform beach: its derivatives against finite
// differences, continuity in depth, time (across the clock's wrap) and sea state, breaking, the
// bores' steep fronts, the swash climbing the beach and falling below the still level, the
// handover, the sets, the resolution fade, the reach. Stage O9b2: the trains' height (root mean
// square), the bores' index, the swash's variance and its hold within its reach, the mesh's
// least front.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private surf_check.cpp -o surf_check

#include "VoxelShaderCompat.h"
#include "VoxelOceanShoreCore.h"

namespace VoxelShader
{
#include "VoxelOceanFFTWaves.ush"
#include "VoxelOceanShoal.ush"
#include "VoxelOceanSurf.ush"
}

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace VoxelShader;

static int Fails = 0;
static void Check(bool bOk, const char* What, double Value, double Limit)
{
    std::printf("  [%s] %-74s %.6g (limit %.6g)\n", bOk ? "ok" : "FAIL", What, Value, Limit);
    if (!bOk) Fails++;
}

static FVoxelOceanSurfSettings Settings(float Sets = 1.0f, float Skew = 0.85f, float Swash = 1.0f, float Handover = 2.5f, float Strength = 1.0f,
    float Reach = 10.0f)
{
    return VoxelOceanSurfSettingsOf(float4(Strength, Sets, Swash, Skew), float4(Handover, 0.0f, 0.0f, Reach), 0.78f);
}

// A point d metres deep on a beach of slope M, at Along metres along the coast (the noises).
static FVoxelOceanSurfInput Input(float D, float T, float Hs = 1.5f, float Speed = 12.0f, float M = 0.02f, float Along = 0.0f)
{
    FVoxelOceanSurfInput In;
    In.DepthM = D;
    In.Slope = M;
    In.Agree = 1.0f;
    In.ShoreM = D / M;
    In.Hs = Hs;
    In.Speed = Speed;
    // A planet-frame point (cm): 1,000 km out, the coast along y, landward along -x; split into
    // whole units of 4096 cm and the rest, as the CPU model splits a site.
    const double P[3] = { 1.0e8 - 100.0 * double(D) / double(M), 200000.0 + 100.0 * double(Along), 30000.0 };
    int U[3];
    float L[3];
    for (int i = 0; i < 3; i++)
    {
        const double Units = std::floor(P[i] / 4096.0);
        U[i] = int(Units);
        L[i] = float(P[i] - Units * 4096.0);
    }
    In.Units = int3(U[0], U[1], U[2]);
    In.Low = float3(L[0], L[1], L[2]);
    In.Time = T;
    In.SpacingM = 0.0f;
    In.FootprintM = 0.0f;
    In.BedSlope = VOXEL_OCEAN_BED_UNKNOWN;
    In.BedDepthM = 1.0e30f;
    In.ForNormals = 0.0f;
    return In;
}

int main()
{
    const FVoxelOceanSurfSettings S = Settings();

    std::printf("1. Derivatives against finite differences\n");
    {
        double WorstG = 0.0, WorstR = 0.0, Scale = 0.0, RateScale = 0.0;
        for (int k = 0; k < 4000; k++)
        {
            const float D = -0.5f + 4.5f * float(k % 400) / 400.0f + 0.0013f;
            const float T = 13.7f + 0.37f * float(k / 400);
            const float M = 0.02f;
            const float H = 0.0005f;  // m of depth (2.5 cm along the beach)
            const FVoxelOceanSurf C = VoxelOceanSurfOf(Input(D, T), S);
            const FVoxelOceanSurf A = VoxelOceanSurfOf(Input(D + H, T), S);
            const FVoxelOceanSurf B = VoxelOceanSurfOf(Input(D - H, T), S);
            // Landward the depth falls by M per metre (the noise point moves with it).
            const double FD = -(A.Height - B.Height) / (2.0 * H) * M;
            const FVoxelOceanSurf TA = VoxelOceanSurfOf(Input(D, T + 0.0005f), S);
            const FVoxelOceanSurf TB = VoxelOceanSurfOf(Input(D, T - 0.0005f), S);
            const double FR = (TA.Height - TB.Height) / 0.001;
            // Smooth stretches only (a fold of the noise's cell or a near-vertical front is
            // sampled too coarsely by the differences).
            if (std::abs(A.Height - 2.0 * C.Height + B.Height) > 2.0e-5 || std::abs(TA.Height - 2.0 * C.Height + TB.Height) > 2.0e-5) continue;
            WorstG = std::max(WorstG, std::abs(C.Gradient - FD));
            WorstR = std::max(WorstR, std::abs(C.Rate - FR));
            Scale = std::max(Scale, std::abs(FD));
            RateScale = std::max(RateScale, std::abs(FR));
        }
        std::printf("  steepest gradient met %.4f\n", Scale);
        Check(WorstG < 0.02 * std::max(Scale, 0.01) + 2.0e-4, "the gradient is the height's along the beach (landward, -m d/dd)", WorstG, 0.02 * std::max(Scale, 0.01) + 2.0e-4);
        std::printf("  fastest rate met %.3f m/s\n", RateScale);
        Check(WorstR < 0.01 * RateScale + 1.0e-3, "the rate is the height's in time (m/s)", WorstR, 0.01 * RateScale + 1.0e-3);
    }

    std::printf("\n2. Continuity\n");
    {
        // Every step within 1.5 times what the steeper gradient at its ends allows (a bore's front
        // is steep, not a jump).
        double WorstD = 0.0, Steepest = 0.0;
        for (int t = 0; t < 40; t++)
        {
            const float T = 100.0f + 2.3f * float(t);
            FVoxelOceanSurf Previous = VoxelOceanSurfOf(Input(-2.0f, T), S);
            for (int k = 1; k <= 20000; k++)
            {
                const float D = -2.0f + 12.0f * float(k) / 20000.0f;   // 0.6 mm of depth a step (3 cm along the beach)
                const FVoxelOceanSurf H = VoxelOceanSurfOf(Input(D, T), S);
                const double Allowed = 1.5 * std::max(std::abs(H.Gradient), std::abs(Previous.Gradient)) * 0.03 + 2.0e-4;
                WorstD = std::max(WorstD, std::abs(H.Height - Previous.Height) / Allowed);
                Steepest = std::max(Steepest, double(std::abs(H.Gradient)));
                Previous = H;
            }
        }
        std::printf("  steepest slope met %.2f\n", Steepest);
        Check(WorstD < 1.0, "no jump in depth from the beach to past the handover (step over its allowance)", WorstD, 1.0);
        double WorstW = 0.0;
        for (int k = 0; k < 200; k++)
        {
            const float D = -0.5f + 0.03f * float(k);
            const FVoxelOceanSurf A = VoxelOceanSurfOf(Input(D, VOXEL_OCEAN_SURF_WRAP - 0.001f), S);
            const FVoxelOceanSurf B = VoxelOceanSurfOf(Input(D, 0.001f), S);
            WorstW = std::max(WorstW, double(std::abs(A.Height - B.Height)));
        }
        Check(WorstW < 0.01, "no jump across the clock's wrap (m over 2 ms, at up to 2.5 m/s)", WorstW, 0.01);
        double WorstS = 0.0;
        for (int k = 0; k < 3000; k++)
        {
            const float Speed = 3.0f + 22.0f * float(k) / 3000.0f;
            const float Next = 3.0f + 22.0f * float(k + 1) / 3000.0f;
            for (int j = 0; j < 5; j++)
            {
                const float D = 0.2f + 0.7f * float(j);
                WorstS = std::max(WorstS, double(std::abs(VoxelOceanSurfOf(Input(D, 57.0f, 1.5f, Speed), S).Height - VoxelOceanSurfOf(Input(D, 57.0f, 1.5f, Next), S).Height)));
            }
        }
        Check(WorstS < 0.01, "no jump as the sea state's period crosses a train's (m per 7 mm/s of speed)", WorstS, 0.01);
    }

    std::printf("\n3. Breaking, bores and swash on a 1:50 beach (Hs 1.5 m, 12 m/s: 9 s)\n");
    {
        const FVoxelOceanSurfSettings Flat = Settings(0.0f);
        double WorstCap = 0.0, Outside = 0.0;
        for (int j = 0; j < 30; j++)
        {
            const float D = 0.1f + 0.1f * float(j);
            float Lo = 1e9f, Hi = -1e9f;
            for (int k = 0; k < 2000; k++)
            {
                const float H = VoxelOceanSurfOf(Input(D, 0.01f * float(k)), Flat).Height;
                Lo = std::min(Lo, H);
                Hi = std::max(Hi, H);
            }
            // The wave's range, the swash's level blended in near the shoreline aside.
            if (D > 2.0f) WorstCap = std::max(WorstCap, double((Hi - Lo) / (VOXEL_OCEAN_SURF_INNER * 0.78f * D)));
            if (j == 29) Outside = Hi - Lo;
        }
        Check(WorstCap <= 1.0, "a wave's height stays under 0.55 d in the surf zone (the bores' index; ratio)", WorstCap, 1.0);
        // At 3 m (0.63 of the 4.8 m handover): the RMS height 1.06 m shoaled (x 1.07), capped, by
        // the handover's share there (0.68).
        Check(Outside > 0.6 && Outside < 0.9, "waves of the deep RMS height by the handover's share at 3 m deep (m)", Outside, 0.6);

        // The bore's front: the rise faster than the fall at 1.5 m deep.
        double Rise = 0.0, Fall = 0.0;
        for (int k = 0; k < 4000; k++)
        {
            const FVoxelOceanSurf R = VoxelOceanSurfOf(Input(1.5f, 0.005f * float(k)), Flat);
            Rise = std::max(Rise, double(R.Rate));
            Fall = std::max(Fall, double(-R.Rate));
        }
        std::printf("  at 1.5 m deep: rising up to %.2f m/s, falling up to %.2f m/s\n", Rise, Fall);
        Check(Rise > 4.0 * Fall, "a breaking wave's front is steep (rise over fall)", Rise / std::max(Fall, 1e-9), 4.0);

        // The swash: the level over the beach climbs above the still level and falls below it.
        float Lo = 1e9f, Hi = -1e9f;
        double Spread = 0.0;
        for (int k = 0; k < 4000; k++)
        {
            const float T = 0.05f * float(k);
            const float A = VoxelOceanSurfOf(Input(-0.05f, T), Flat).Height;
            const float B = VoxelOceanSurfOf(Input(-0.25f, T), Flat).Height;
            Spread = std::max(Spread, double(std::abs(A - B)));
            Lo = std::min(Lo, A);
            Hi = std::max(Hi, A);
        }
        std::printf("  the shoreline's level: %.2f to %.2f m (run-up to %.0f m up the beach, back %.0f m)\n", Lo, Hi, Hi / 0.02f, -Lo / 0.02f);
        Check(Hi > 0.15f && Hi < 1.0f, "the swash runs up the beach (its highest level, m)", Hi, 0.15);
        Check(Lo < -0.02f, "the backwash falls below the still level (the wet sand shows; m)", Lo, -0.02);
        Check(Spread < 0.01, "the water over the beach is level within its reach (the noises' drift over 10 m; m)", Spread, 0.01);
        // Steeper beach, higher run-up (Stockdon).
        float HiSteep = -1e9f;
        for (int k = 0; k < 4000; k++)
            HiSteep = std::max(HiSteep, VoxelOceanSurfOf(Input(-0.05f, 0.05f * float(k), 1.5f, 12.0f, 0.08f), Flat).Height);
        Check(HiSteep > Hi, "a steeper beach runs up higher (m)", HiSteep, Hi);
        // No swash where the field's directions disagree, nor far inland.
        float Disagree = 0.0f, Inland = 0.0f;
        for (int k = 0; k < 2000; k++)
        {
            FVoxelOceanSurfInput A = Input(-0.3f, 0.05f * float(k));
            A.Agree = 0.0f;
            Disagree = std::max(Disagree, std::abs(VoxelOceanSurfOf(A, Flat).Height));
            FVoxelOceanSurfInput B = Input(-0.3f, 0.05f * float(k));
            B.ShoreM = -3500.0f;
            Inland = std::max(Inland, std::abs(VoxelOceanSurfOf(B, Flat).Height));
        }
        Check(Disagree == 0.0f, "no swash where the field's directions disagree (m)", Disagree, 0.0);
        Check(Inland == 0.0f, "no swash 3.5 km inland by the field (low ground behind the beach; m)", Inland, 0.0);

        // Stage O9b2: the swash's swings by the variance of Stockdon's, held within the reach.
        // On a 1:200 beach, Hs 2 m, 15 m/s (11 s): unheld, the infragravity swing alone would be
        // (sqrt 2 / 4) 0.06 sqrt(Hs L0) = 0.26 m, 52 m up and down the beach.
        auto Swing = [](const FVoxelOceanSurfSettings& Set, float M, float& OutSetup)
        {
            float SLo = 1e9f, SHi = -1e9f;
            double Sum = 0.0;
            const int N = 7200;
            for (int k = 0; k < N; k++)
            {
                const float L = VoxelOceanSurfOf(Input(-0.01f, 0.25f * float(k), 2.0f, 15.0f, M), Set).Height;
                SLo = std::min(SLo, L);
                SHi = std::max(SHi, L);
                Sum += L;
            }
            OutSetup = float(Sum / N);
            return 0.5f * (SHi - SLo);
        };
        float SetupHeld = 0.0f, SetupFree = 0.0f, SetupSteep = 0.0f;
        const float Held = Swing(Settings(), 0.005f, SetupHeld);
        const float Free = Swing(Settings(1.0f, 0.85f, 1.0f, 2.5f, 1.0f, 0.0f), 0.005f, SetupFree);
        const float Steep = Swing(Settings(), 0.1f, SetupSteep);
        std::printf("  1:200, Hs 2 m: swings %.2f m held (%.1f m up and down the beach), %.2f m unheld (%.1f m); 1:10: %.2f m (%.1f m)\n",
            Held, Held / 0.005f, Free, Free / 0.005f, Steep, Steep / 0.1f);
        Check(Held / 0.005f <= 10.5f, "the swash stays within its reach up and down the beach (10 m; m)", Held / 0.005f, 10.5);
        Check(Free / 0.005f > 30.0f, "unheld it would sweep much farther on a gentle beach (m)", Free / 0.005f, 30.0);
        Check(Steep / 0.1f <= 10.5f && Steep / 0.1f > 2.0f, "a steep beach's swash runs a few metres (m)", Steep / 0.1f, 10.5);
        // Without the sets the incident swing alone is (sqrt 2 / 4) 0.75 b sqrt(Hs L0) at most
        // (Stockdon's significant swash, as a swing of its variance), on a steep beach.
        float SetupPlain = 0.0f;
        const float Plain = Swing(Settings(0.0f, 0.85f, 1.0f, 2.5f, 1.0f, 0.0f), 0.05f, SetupPlain);
        const float L0 = 9.80665f * 11.26f * 11.26f / 6.2831853f;
        const float Expected = VOXEL_OCEAN_SURF_SWING * (0.75f * 0.05f + 0.06f) * std::sqrt(2.0f * L0);
        Check(Plain <= 1.02f * Expected && Plain > 0.7f * Expected, "the swings are Stockdon's swash as a swing of its variance (m)", Plain, Expected);
    }

    std::printf("\n4. The handover, the sets, the resolution\n");
    {
        // No jump at the handover's edge with a strong swash and a short handover.
        const FVoxelOceanSurfSettings Odd = Settings(1.0f, 0.85f, 3.0f, 1.0f);
        double Worst = 0.0;
        for (int t = 0; t < 50; t++)
        {
            const float T = 7.0f * float(t);
            for (int j = 0; j < 4; j++)
            {
                const float Hs = 0.2f + 0.4f * float(j);
                const float Reach = std::max(1.0f * Hs / 0.78f, 2.0f);
                const float A = VoxelOceanSurfOf(Input(Reach - 1.0e-4f, T, Hs), Odd).Height;
                const float B = VoxelOceanSurfOf(Input(Reach + 1.0e-4f, T, Hs), Odd).Height;
                Worst = std::max(Worst, double(std::abs(A - B)));
            }
        }
        Check(Worst < 1.0e-3, "no jump at the handover's edge (swash 3, handover 1; m)", Worst, 1.0e-3);
    }
    {
        const FVoxelOceanSurf Far = VoxelOceanSurfOf(Input(6.0f, 10.0f), S);
        const FVoxelOceanSurf Near = VoxelOceanSurfOf(Input(0.5f, 10.0f), S);
        Check(Far.Active < 0.5f && Far.LongKeep == 1.0f, "past the handover (4.8 m here) no surf, the long bands whole", Far.LongKeep, 1.0);
        Check(Near.LongKeep < 1.0e-3f, "in the surf zone the long bands are handed over", Near.LongKeep, 1.0e-3);
        FVoxelOceanSurfInput Between = Input(0.5f, 10.0f);
        Between.Agree = 0.25f;
        Check(std::abs(VoxelOceanSurfOf(Between, S).LongKeep - std::sqrt(1.0f - 0.0625f)) < 1e-5f, "where the field's directions disagree the surf takes less",
            VoxelOceanSurfOf(Between, S).LongKeep, std::sqrt(1.0 - 0.0625));

        // Wave to wave at 3 m deep: crests alike without sets, varied with them.
        // At a train's own period (9.09 s: 12.11 m/s), one train.
        const float OneTrain = (VOXEL_OCEAN_SURF_WRAP / 396.0f) / VOXEL_OCEAN_SURF_PERIOD_PER_SPEED;
        auto Crests = [OneTrain](const FVoxelOceanSurfSettings& Set, float Along, std::vector<float>& Out)
        {
            Out.clear();
            float A = VoxelOceanSurfOf(Input(3.0f, 0.0f, 1.5f, OneTrain, 0.02f, Along), Set).Height, B = A;
            for (int k = 2; k < 36000; k++)
            {
                const float C = VoxelOceanSurfOf(Input(3.0f, 0.05f * float(k), 1.5f, OneTrain, 0.02f, Along), Set).Height;
                if (B > A && B >= C) Out.push_back(B);
                A = B;
                B = C;
            }
        };
        std::vector<float> Plain, WithSets;
        Crests(Settings(0.0f), 0.0f, Plain);
        Crests(S, 0.0f, WithSets);
        const auto [PMin, PMax] = std::minmax_element(Plain.begin(), Plain.end());
        const auto [SMin, SMax] = std::minmax_element(WithSets.begin(), WithSets.end());
        std::printf("  %zu crests in 30 min: %.2f to %.2f m without sets, %.2f to %.2f m with\n", WithSets.size(), *PMin, *PMax, *SMin, *SMax);
        Check(*PMax / *PMin < 1.05f, "without sets the waves stay alike (at one train's period)", *PMax / *PMin, 1.05);
        Check(*SMax / *SMin > 1.8f, "with sets some waves are much larger than others", *SMax / *SMin, 1.8);

        // Along the coast 3 km apart the sets don't come together: the correlation of the waves'
        // one-minute energy (mean removed) over half an hour, the mean over 10 pairs.
        double SumCorr = 0.0;
        for (int p = 0; p < 10; p++)
        {
            const float From = 5000.0f * float(p);
            std::vector<double> EA, EB;
            for (int w = 0; w < 30; w++)
            {
                double A2 = 0.0, B2 = 0.0;
                for (int k = 0; k < 300; k++)
                {
                    const float T = 60.0f * float(w) + 0.2f * float(k);
                    const double A = VoxelOceanSurfOf(Input(3.0f, T, 1.5f, 12.0f, 0.02f, From), S).Height;
                    const double B = VoxelOceanSurfOf(Input(3.0f, T, 1.5f, 12.0f, 0.02f, From + 3000.0f), S).Height;
                    A2 += A * A;
                    B2 += B * B;
                }
                EA.push_back(A2);
                EB.push_back(B2);
            }
            double MA = 0.0, MB = 0.0;
            for (int w = 0; w < 30; w++) { MA += EA[w] / 30.0; MB += EB[w] / 30.0; }
            double Same = 0.0, NA = 0.0, NB = 0.0;
            for (int w = 0; w < 30; w++) { Same += (EA[w] - MA) * (EB[w] - MB); NA += (EA[w] - MA) * (EA[w] - MA); NB += (EB[w] - MB) * (EB[w] - MB); }
            SumCorr += std::abs(Same / std::sqrt(NA * NB));
        }
        Check(SumCorr / 10.0 < 0.5, "3 km apart along the coast the sets come at other times (mean correlation)", SumCorr / 10.0, 0.5);

        // A coarse mesh keeps the levels, not the waves it can't hold.
        FVoxelOceanSurfInput Coarse = Input(1.0f, 21.0f);
        Coarse.SpacingM = 200.0f;
        const FVoxelOceanSurf C = VoxelOceanSurfOf(Coarse, S);
        FVoxelOceanSurfInput OnLand = Input(-0.1f, 21.0f);
        OnLand.SpacingM = 200.0f;
        Check(std::abs(C.Height) < 0.5f, "a 200 m spacing drops the trains' waves (m left)", std::abs(C.Height), 0.5);
        Check(VoxelOceanSurfOf(OnLand, S).Height == VoxelOceanSurfOf(Input(-0.1f, 21.0f), S).Height, "and keeps the swash's level", 0, 0);

        // Stage O9b2: the mesh keeps a front of 4 spacings at least. At 1 m spacing on the
        // 1:50 beach, the height's steepest slope along the beach (over the surf zone, through a
        // wave period) stays under a half cosine's over 4 spacings of the local wave height; the
        // unlimited surface is steeper.
        const FVoxelOceanSurfSettings Flat = Settings(0.0f);
        double WorstMesh = 0.0, SteepMesh = 0.0, SteepFree = 0.0;
        for (int k = 0; k < 90; k++)
        {
            const float T = 31.0f + 0.1f * float(k);
            for (int j = 0; j < 1200; j++)
            {
                const float X0 = 15.0f + 0.05f * float(j);      // metres from the shoreline
                const float X1 = X0 + 0.05f;
                FVoxelOceanSurfInput A = Input(0.02f * X0, T), B = Input(0.02f * X1, T);
                A.SpacingM = 1.0f;
                B.SpacingM = 1.0f;
                const double Slope = std::abs(VoxelOceanSurfOf(A, Flat).Height - VoxelOceanSurfOf(B, Flat).Height) / 0.05;
                // The local height bound: the bores' index times the depth.
                const double Bound = 1.5707963 * (VOXEL_OCEAN_SURF_INNER * 0.78 * 0.02 * X1) / (VOXEL_OCEAN_SURF_MESH_FRONT * 1.0);
                WorstMesh = std::max(WorstMesh, Slope / Bound);
                SteepMesh = std::max(SteepMesh, Slope);
                SteepFree = std::max(SteepFree, std::abs(VoxelOceanSurfOf(Input(0.02f * X0, T), Flat).Height - VoxelOceanSurfOf(Input(0.02f * X1, T), Flat).Height) / 0.05);
            }
        }
        std::printf("  steepest slope along the beach: %.2f at 1 m spacing, %.2f unlimited\n", SteepMesh, SteepFree);
        Check(WorstMesh < 1.1, "the mesh keeps a front of at least 4 spacings (slope over a half cosine's)", WorstMesh, 1.1);

        // And no wave under 4 spacings: at 4 m spacing a point 0.3 m deep (a 9 s wave's local
        // wavelength there is about 15 m) keeps none of the trains' wave.
        double Short = 0.0;
        for (int k = 0; k < 200; k++)
        {
            FVoxelOceanSurfInput A = Input(0.3f, 0.05f * float(k));
            A.SpacingM = 4.0f;
            FVoxelOceanSurfInput Lv = Input(0.3f, 0.05f * float(k));
            Lv.SpacingM = 1.0e6f;
            Short = std::max(Short, double(std::abs(VoxelOceanSurfOf(A, Flat).Height - VoxelOceanSurfOf(Lv, Flat).Height)));
        }
        Check(Short < 1.0e-6, "a wave under 4 spacings leaves the mesh (m of wave left at 4 m spacing, 15 m waves)", Short, 1.0e-6);

        // Why the depth the waves feel is smoothed (stage O9b2, VoxelOceanPage.usf): terrain
        // detail of 10 cm at a 3 m wavelength on the 1:50 beach, vertices 0.5 m apart. The
        // steepest step from one vertex to the next, over the surf zone through a wave period,
        // against the smooth beach's.
        double StepRough = 0.0, StepSmooth = 0.0;
        for (int k = 0; k < 90; k++)
        {
            const float T = 31.0f + 0.1f * float(k);
            float PrevR = 0.0f, PrevS = 0.0f;
            for (int j = 0; j < 300; j++)
            {
                const float X = 10.0f + 0.5f * float(j);
                const float Detail = 0.1f * std::sin(6.2831853f * X / 3.0f + 1.3f);
                FVoxelOceanSurfInput R = Input(0.02f * X + Detail, T), Sm = Input(0.02f * X, T);
                R.SpacingM = 0.5f;
                Sm.SpacingM = 0.5f;
                const float HR = VoxelOceanSurfOf(R, Flat).Height, HS = VoxelOceanSurfOf(Sm, Flat).Height;
                if (j > 0)
                {
                    StepRough = std::max(StepRough, double(std::abs(HR - PrevR)));
                    StepSmooth = std::max(StepSmooth, double(std::abs(HS - PrevS)));
                }
                PrevR = HR;
                PrevS = HS;
            }
        }
        std::printf("  steepest vertex step (0.5 m apart): %.3f m over a smooth beach, %.3f m with 10 cm of 3 m detail\n", StepSmooth, StepRough);
        Check(StepRough > 2.0 * StepSmooth, "terrain detail the depth keeps breaks the crests into spikes (step ratio)", StepRough / StepSmooth, 2.0);
    }

    std::printf("\n4b. The bed (stage O9b2): the bottom's own slope against the field's\n");
    {
        const FVoxelOceanSurfSettings Flat = Settings(0.0f);
        // The codes: magnitude within 3.5%, the sign, level, unknown; three to a float.
        double WorstCode = 0.0;
        bool bSigns = true;
        for (int k = 0; k <= 2000; k++)
        {
            const float M = std::exp2(-11.0f + 12.0f * float(k) / 2000.0f);
            const float Up = VoxelOceanBedSlopeOf(VoxelOceanBedCode(M)), Down = VoxelOceanBedSlopeOf(VoxelOceanBedCode(-M));
            WorstCode = std::max(WorstCode, double(std::abs(Up / M - 1.0f)));
            bSigns = bSigns && Up > 0.0f && Down == -Up;
        }
        const float Packed = VoxelOceanBedPack(VoxelOceanBedCode(0.03f), VoxelOceanBedCode(-0.2f), 0u);
        const bool bPack = VoxelOceanBedCodeAt(Packed, 0) == VoxelOceanBedCode(0.03f) && VoxelOceanBedCodeAt(Packed, 1) == VoxelOceanBedCode(-0.2f)
            && VoxelOceanBedCodeAt(Packed, 2) == 0u && VoxelOceanBedSlopeOf(0u) == VOXEL_OCEAN_BED_UNKNOWN && VoxelOceanBedSlopeOf(VoxelOceanBedCode(0.0f)) == 0.0f;
        Check(WorstCode < 0.035, "the bed's slope codes within 3.5% (0.0005 to 2)", WorstCode, 0.035);
        Check(bSigns && bPack, "the codes keep the sign, level and unknown, three to a float", bSigns && bPack ? 1 : 0, 1);

        // A beach 2.5 times as steep as the field says (1:20 against 1:50): the gradient is the
        // height's along it.
        auto OnBed = [](float X, float T, float Bed, float Spacing)
        {
            FVoxelOceanSurfInput In = Input(Bed * X, T);
            In.BedSlope = Bed;
            In.SpacingM = Spacing;
            In.FootprintM = Spacing;
            return In;
        };
        double WorstG = 0.0, SteepG = 0.0;
        for (int k = 0; k < 400; k++)
        {
            const float T = 13.0f + 0.37f * float(k);
            const float X = 2.0f + 0.11f * float(k % 97);
            const float H = 0.002f;
            const float Fd = (VoxelOceanSurfOf(OnBed(X - H, T, 0.05f, 0.0f), Flat).Height - VoxelOceanSurfOf(OnBed(X + H, T, 0.05f, 0.0f), Flat).Height) / (2.0f * H);
            const float G = VoxelOceanSurfOf(OnBed(X, T, 0.05f, 0.0f), Flat).Gradient;
            WorstG = std::max(WorstG, double(std::abs(Fd - G)));
            SteepG = std::max(SteepG, double(std::abs(G)));
        }
        Check(WorstG < 0.02 * std::max(SteepG, 1e-3), "on a bed steeper than the field's the gradient is the height's along it", WorstG, 0.02 * SteepG);

        // The waves' length on the ground: 0.3 m deep a 9 s wave is about 15 m long by the field's
        // slope, a quarter of it on a bed 4 times as steep: under 4 spacings of a 1 m mesh, so
        // none of it in the mesh (where the field's slope alone would keep some).
        double KeptSteep = 0.0, KeptField = 0.0;
        for (int k = 0; k < 200; k++)
        {
            const float T = 0.05f * float(k);
            FVoxelOceanSurfInput A = OnBed(0.3f / 0.08f, T, 0.08f, 1.0f), Lv = OnBed(0.3f / 0.08f, T, 0.08f, 1.0e6f);
            KeptSteep = std::max(KeptSteep, double(std::abs(VoxelOceanSurfOf(A, Flat).Height - VoxelOceanSurfOf(Lv, Flat).Height)));
            FVoxelOceanSurfInput B = Input(0.3f, T), Bv = Input(0.3f, T);
            B.SpacingM = 1.0f;
            Bv.SpacingM = 1.0e6f;
            KeptField = std::max(KeptField, double(std::abs(VoxelOceanSurfOf(B, Flat).Height - VoxelOceanSurfOf(Bv, Flat).Height)));
        }
        std::printf("  0.3 m deep, 1 m spacing: wave kept %.3f m on the field's slope, %.3f m on a bed 4 times as steep\n", KeptField, KeptSteep);
        Check(KeptSteep < 1.0e-6 && KeptField > 0.05, "a wave too short on the ground for the mesh leaves it (m kept)", KeptSteep, 1.0e-6);

        // Behind a bar (the bottom rising seaward) and on a flat, the trains fade out and the long
        // bands take their energy back.
        FVoxelOceanSurfInput Back = Input(2.0f, 10.0f), Level = Input(2.0f, 10.0f), Normal = Input(2.0f, 10.0f);
        Back.BedSlope = -0.02f;
        Level.BedSlope = 0.0008f;
        Normal.BedSlope = 0.02f;
        const FVoxelOceanSurf RB = VoxelOceanSurfOf(Back, Flat), RL = VoxelOceanSurfOf(Level, Flat), RN = VoxelOceanSurfOf(Normal, Flat);
        std::printf("  2 m deep, long bands kept: %.2f on the field's slope, %.2f behind a bar, %.2f on a flat\n", RN.LongKeep, RB.LongKeep, RL.LongKeep);
        Check(RB.LongKeep == 1.0f && RL.LongKeep == 1.0f && RB.Breaking == 0.0f && RL.Breaking == 0.0f && RN.LongKeep < 0.5f,
            "behind a bar and on a flat no train breaks, and the long bands carry the waves (stage O9b3)", RB.LongKeep, 1.0);
        double WaveBack = 0.0;
        for (int k = 0; k < 400; k++)
        {
            FVoxelOceanSurfInput A = Input(2.0f, 0.05f * float(k)), B = Input(2.0f, 0.05f * float(k));
            A.BedSlope = -0.02f;
            B.BedSlope = -0.02f;
            B.Time = 0.05f * float(k) + 0.5f;
            WaveBack = std::max(WaveBack, double(std::abs(VoxelOceanSurfOf(A, Flat).Height - VoxelOceanSurfOf(B, Flat).Height)));
        }
        Check(WaveBack < 0.02, "behind a bar no train moves (m in 0.5 s; the setup's drift aside)", WaveBack, 0.02);
    }

    std::printf("\n4c. Stage O9b3: unshown energy, the sheet's reach, the inland fade, the drain\n");
    {
        const FVoxelOceanSurfSettings Flat = Settings(0.0f);
        // Trains too short for the mesh hand their energy back to the long bands.
        FVoxelOceanSurfInput Fine = Input(2.0f, 10.0f), Coarse = Input(2.0f, 10.0f);
        Coarse.SpacingM = 50.0f;
        const float KeepFine = VoxelOceanSurfOf(Fine, Flat).LongKeep, KeepCoarse = VoxelOceanSurfOf(Coarse, Flat).LongKeep;
        std::printf("  2 m deep, long bands kept: %.2f where the trains are drawn, %.2f at a 50 m spacing\n", KeepFine, KeepCoarse);
        Check(KeepFine < 0.5f && KeepCoarse > 0.999f, "trains the mesh can't draw leave their energy with the long bands", KeepCoarse, 0.999);

        // The sheet on a plain 5 cm above the sea, flat (bed 0.0005): none of it there, where the
        // field's 1:50 beach would have flooded it; on the beach itself it still runs up.
        double OnPlain = 0.0, OnBeach = 0.0;
        for (int k = 0; k < 2000; k++)
        {
            FVoxelOceanSurfInput P = Input(-0.05f, 0.25f * float(k));
            P.BedSlope = 0.0005f;
            OnPlain = std::max(OnPlain, double(std::abs(VoxelOceanSurfOf(P, Flat).Height)));
            FVoxelOceanSurfInput B = Input(-0.05f, 0.25f * float(k));
            B.BedSlope = 0.02f;
            OnBeach = std::max(OnBeach, double(VoxelOceanSurfOf(B, Flat).Height));
        }
        std::printf("  5 cm above the sea: sheet up to %.3f m on a flat plain, %.3f m on the 1:50 beach\n", OnPlain, OnBeach);
        Check(OnPlain == 0.0 && OnBeach > 0.1, "the swash's sheet doesn't cover a plain near sea level, still runs up a beach", OnPlain, 0.0);

        // The inland fade on both sides of the still level: no step across it 2.2 km inland.
        double Step = 0.0;
        for (int k = 0; k < 2000; k++)
        {
            FVoxelOceanSurfInput A = Input(-0.001f, 0.25f * float(k)), B = Input(0.001f, 0.25f * float(k));
            A.ShoreM = -2200.0f;
            B.ShoreM = -2200.0f;
            Step = std::max(Step, double(std::abs(VoxelOceanSurfOf(A, Flat).Height - VoxelOceanSurfOf(B, Flat).Height)));
        }
        Check(Step < 0.01, "no step where the ground crosses the still level inland (m)", Step, 0.01);

        // The drain: nothing within the bake's margin (two thirds of the basins'), the waves gone
        // and the surface under the bottom past the basins' margin, continuous between, its slope
        // the drop's rise landward (the depth falling by the bed's slope).
        const FVoxelOceanSurfDrain Near = VoxelOceanSurfDrainOf(-600.0f, 1.0f, 0.02f, 1000.0f), Far = VoxelOceanSurfDrainOf(-1000.0f, 1.0f, 0.02f, 1000.0f);
        const FVoxelOceanSurfDrain None = VoxelOceanSurfDrainOf(-5000.0f, 1.0f, 0.02f, 0.0f);
        double DrainStep = 0.0, DrainSlope = 0.0;
        for (int k = 0; k < 2000; k++)
        {
            // Landward s metres past the start: the field's distance -666.7 - s, the depth 2 - 0.02 s.
            const float Sx = 0.17f * float(k);
            auto At = [](float X) { return VoxelOceanSurfDrainOf(-666.7f - X, 2.0f - 0.02f * X, 0.02f, 1000.0f); };
            const FVoxelOceanSurfDrain A = At(Sx), B = At(Sx + 0.17f), M = At(Sx + 0.085f);
            DrainStep = std::max(DrainStep, double(std::abs(A.Drop - B.Drop)));
            DrainSlope = std::max(DrainSlope, double(std::abs((B.Drop - A.Drop) / 0.17f - M.Slope)));
        }
        Check(Near.Keep == 1.0f && Near.Drop == 0.0f && Far.Keep == 0.0f && Far.Drop >= 1.0f + VOXEL_OCEAN_SURF_DRAIN_DEPTH && None.Keep == 1.0f && DrainStep < 0.02,
            "the drain: none within the bake's margin, below the bottom past the basins', smooth between", DrainStep, 0.02);
        Check(DrainSlope < 2.0e-3, "the drain's slope is its drop's rise landward (finite differences, the kink where the bottom meets sea level aside)", DrainSlope, 2.0e-3);

        // On a flat (the bed 0.001 against the field's 0.02) the swash hardly moves.
        float FLo = 1e9f, FHi = -1e9f;
        for (int k = 0; k < 4000; k++)
        {
            FVoxelOceanSurfInput P = Input(-0.001f, 0.25f * float(k));
            P.BedSlope = 0.001f;
            const float L = VoxelOceanSurfOf(P, Flat).Height;
            FLo = std::min(FLo, L);
            FHi = std::max(FHi, L);
        }
        std::printf("  on a flat: the level swings %.3f m\n", FHi - FLo);
        Check(FHi - FLo < 0.03f, "on a flat the swash's level hardly moves (m)", FHi - FLo, 0.03);
    }

    std::printf("\n5. The reach: the surf's highest against the FFT model's allowance\n");
    {
        // The model allows 1.6 x 5 sigma of the whole sea (VoxelOceanWavesFFT); the long bands'
        // Hs = 4 sigma of theirs: 2 Hs at the least.
        double Highest = 0.0;
        for (int b = 0; b < 3; b++)
        {
            const float M = b == 0 ? 0.005f : (b == 1 ? 0.02f : 0.1f);
            for (int j = 0; j < 60; j++)
                for (int k = 0; k < 1800; k++)
                {
                    const float D = -0.5f + 0.1f * float(j);
                    Highest = std::max(Highest, double(VoxelOceanSurfOf(Input(D, 0.5f * float(k), 2.0f, 15.0f, M, 37.0f * float(j)), S).Height));
                }
        }
        std::printf("  highest %.2f m for Hs 2 m (beaches 1:200 to 1:10)\n", Highest);
        Check(Highest < VoxelOceanSurfMaxHeight(2.0f, S), "within the surf's bound (VoxelOceanSurfMaxHeight, the model's reach)", Highest, VoxelOceanSurfMaxHeight(2.0f, S));
        // Strength and swash 2.
        const FVoxelOceanSurfSettings Strong = Settings(1.0f, 0.85f, 2.0f, 2.5f, 2.0f);
        double HighestStrong = 0.0;
        for (int b = 0; b < 3; b++)
        {
            const float M = b == 0 ? 0.005f : (b == 1 ? 0.02f : 0.1f);
            for (int j = 0; j < 60; j++)
                for (int k = 0; k < 1800; k++)
                {
                    const float D = -0.5f + 0.15f * float(j);
                    HighestStrong = std::max(HighestStrong, double(VoxelOceanSurfOf(Input(D, 0.5f * float(k), 2.0f, 24.0f, M, 37.0f * float(j)), Strong).Height));
                }
        }
        std::printf("  strength and swash 2: highest %.2f m (bound %.2f m)\n", HighestStrong, VoxelOceanSurfMaxHeight(2.0f, Strong));
        Check(HighestStrong < VoxelOceanSurfMaxHeight(2.0f, Strong), "within the surf's bound at strength and swash 2", HighestStrong, VoxelOceanSurfMaxHeight(2.0f, Strong));
    }

    std::printf("\n%s: %d failure(s)\n", Fails ? "FAILED" : "PASSED", Fails);
    return Fails ? 1 : 0;
}
