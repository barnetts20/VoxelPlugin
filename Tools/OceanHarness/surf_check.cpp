// surf_check.cpp - Stage O9a's shared code offline: the surf (VoxelOceanSurf.ush, the same
// function the wave hook and the CPU model run) on a uniform beach: its derivatives against finite
// differences, continuity in depth, time (across the clock's wrap) and sea state, breaking, the
// bores' steep fronts, the swash climbing the beach and falling below the still level, the
// handover, the sets, the resolution fade, the reach.
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

static FVoxelOceanSurfSettings Settings(float Sets = 1.0f, float Skew = 0.85f, float Swash = 1.0f, float Handover = 2.5f, float Strength = 1.0f)
{
    return VoxelOceanSurfSettingsOf(float4(Strength, Sets, Swash, Skew), float4(Handover, 0.0f, 0.0f, 0.0f), 0.78f);
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
            if (D > 2.0f) WorstCap = std::max(WorstCap, double((Hi - Lo) / (0.78f * D)));
            if (j == 29) Outside = Hi - Lo;
        }
        Check(WorstCap <= 1.0, "a wave's height stays under 0.78 d in the surf zone (ratio)", WorstCap, 1.0);
        Check(Outside > 0.8, "waves of about the deep height come in at 3 m deep (m)", Outside, 0.8);

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
            const float B = VoxelOceanSurfOf(Input(-0.8f, T), Flat).Height;
            Spread = std::max(Spread, double(std::abs(A - B)));
            Lo = std::min(Lo, A);
            Hi = std::max(Hi, A);
        }
        std::printf("  the shoreline's level: %.2f to %.2f m (run-up to %.0f m up the beach, back %.0f m)\n", Lo, Hi, Hi / 0.02f, -Lo / 0.02f);
        Check(Hi > 0.3f && Hi < 2.0f, "the swash runs up the beach (its highest level, m)", Hi, 0.3);
        Check(Lo < -0.02f, "the backwash falls below the still level (the wet sand shows; m)", Lo, -0.02);
        Check(Spread < 0.01, "the water over the beach is level (within the noises' drift over 37 m; m)", Spread, 0.01);
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
