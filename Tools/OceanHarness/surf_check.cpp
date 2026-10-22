// surf_check.cpp - The shore's waves offline (VoxelOceanSurf.ush, the same function the wave hook
// and the CPU model run; stage O9i): the directed trains from a coastal travel-time field on a
// uniform beach (the field integrated here as the bake's solve would along straight rays: A =
// integral ds / sqrt(held depth), B = integral sqrt(held depth) ds, from the seed depth in).
// Checks: the composition (the trains' share by the field's depth past the cutoff, the FFT's floor,
// the cover, the reach), the crests' spacing against the exact local wavelength, the derivatives
// against finite differences (space and time), continuity in depth and across the clock's wrap,
// the swash (its range, its mean, continuity at the still level), the swash's fade on land by the
// mesh's spacing and the footprint, the trains' fade by the mesh, the bores' least front, the foam
// (bores, whitecaps, the slope variance for the open sea's), no field, and the bound.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private surf_check.cpp -o surf_check

#include "VoxelShaderCompat.h"
#include "VoxelOceanShoreCore.h"
#include "VoxelOceanCoastCore.h"

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
    std::printf("  [%s] %-78s %.6g (limit %.6g)\n", bOk ? "ok" : "FAIL", What, Value, Limit);
    if (!bOk) Fails++;
}

static FVoxelOceanSurfSettings Settings(float Sets = 1.0f, float Skew = 0.85f, float Swash = 1.0f, float Strength = 1.0f, float Reach = 10.0f,
    float Cutoff = 0.2f, float Floor = 0.25f, float Foam = 1.0f)
{
    return VoxelOceanSurfSettingsOf(float4(Strength, Sets, Swash, Skew), float4(0.0f, Foam, 1.0f, Reach), float4(Cutoff, Floor, 0.0f, 0.0f), 0.78f);
}

// A uniform beach of slope M: the depth M x at x metres offshore (below 0 on land), the coastal
// field along straight rays from the seed depth (40 m) in, tabulated every Step metres.
struct FBeach
{
    double M = 0.02;
    double Seed = 40.0;
    double Step = 0.05;
    double X0 = -200.0;     // the table's start (on land)
    std::vector<double> A, B;

    explicit FBeach(double InM) : M(InM)
    {
        const double XSeed = Seed / M;
        const int Count = int((XSeed - X0) / Step) + 2;
        A.assign(size_t(Count), 0.0);
        B.assign(size_t(Count), 0.0);
        // From the seed inward (the trapezoid rule on the held depth).
        for (int i = Count - 2; i >= 0; i--)
        {
            const double Xa = X0 + Step * i, Xb = Xa + Step;
            if (Xb > XSeed) { A[size_t(i)] = 0.0; B[size_t(i)] = 0.0; continue; }
            const double Ha = VoxelOceanCoastHeldDepth(float(M * Xa)), Hb = VoxelOceanCoastHeldDepth(float(M * Xb));
            A[size_t(i)] = A[size_t(i + 1)] + 0.5 * Step * (1.0 / std::sqrt(Ha) + 1.0 / std::sqrt(Hb));
            B[size_t(i)] = B[size_t(i + 1)] + 0.5 * Step * (std::sqrt(Ha) + std::sqrt(Hb));
        }
    }
    double Lerp(const std::vector<double>& T, double X) const
    {
        const double U = std::clamp((X - X0) / Step, 0.0, double(T.size() - 2));
        const int I = int(U);
        const double F = U - I;
        return T[size_t(I)] + (T[size_t(I + 1)] - T[size_t(I)]) * F;
    }
};

// The offshore direction in the planet frame (the points below run along -X offshore).
static const float3 Off(-1.0f, 0.0f, 0.0f);

// A point x metres offshore on Beach (along the coast Along metres: the noises), at time T.
static FVoxelOceanSurfInput Input(const FBeach& Beach, double X, float T, float Hs = 1.5f, float Speed = 12.0f, float Along = 0.0f)
{
    FVoxelOceanSurfInput In;
    const float D = float(Beach.M * X);
    In.DepthM = D;
    In.Slope = float(Beach.M);
    In.ShoreM = float(X);
    In.Hs = Hs;
    In.Speed = Speed;
    // A planet-frame point (cm): 1,000 km out, the coast along y, offshore along -x; split into
    // whole units of 4096 cm and the rest, as the CPU model splits a site.
    const double P[3] = { 1.0e8 - 100.0 * X, 200000.0 + 100.0 * double(Along), 30000.0 };
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
    In.BedSlope = float(Beach.M);
    In.BedDepthM = 1.0e30f;
    In.ForNormals = 0.0f;
    // The field: A and B grow shoreward (their gradients point shoreward, -Off).
    const double Held = VoxelOceanCoastHeldDepth(D);
    const bool bIn = X < Beach.Seed / Beach.M;
    In.CoastA = float(Beach.Lerp(Beach.A, X));
    In.CoastB = float(Beach.Lerp(Beach.B, X));
    In.CoastGradA = bIn ? Off * float(-1.0 / std::sqrt(Held)) : float3(0.0f, 0.0f, 0.0f);
    In.CoastGradB = bIn ? Off * float(-std::sqrt(Held)) : float3(0.0f, 0.0f, 0.0f);
    In.CoastCover = 1.0f;
    In.CoastSeedM = float(Beach.Seed);
    In.CoastReachM = 8000.0f;
    return In;
}

// The surf's slope offshore (d height / dx): its phase's along Off, the depth's (up the bed is
// shoreward here) and the inland fade's (landward is shoreward).
static double Offshore(const FVoxelOceanSurf& R) { return double(dot(R.GradientPhase, Off)) - R.GradientBed - R.Gradient; }

// The exact wavenumber at depth D for angular frequency W.
static double ExactK(double W, double D)
{
    double K = W * W / 9.80665;
    K = std::max(K, W / std::sqrt(9.80665 * D));
    for (int i = 0; i < 60; i++)
    {
        const double T = std::tanh(K * D);
        const double F = 9.80665 * K * T - W * W;
        const double DF = 9.80665 * (T + K * D * (1.0 - T * T));
        K -= F / DF;
    }
    return K;
}

int main()
{
    const FVoxelOceanSurfSettings S = Settings();
    const FBeach Beach(0.02);

    std::printf("== the composition (Hs 1.5 m, 12 m/s: peak period %.2f s)\n", VOXEL_OCEAN_SURF_PERIOD_PER_SPEED * 12.0);
    {
        const double Period = VOXEL_OCEAN_SURF_PERIOD_PER_SPEED * 12.0;
        const double Cut = std::min(0.2 * 9.80665 * Period * Period / (2.0 * 3.14159265), 0.8 * 40.0);
        std::printf("  cutoff %.1f m (fully in from %.1f m)\n", Cut, 0.4 * Cut);
        double DeepShare = 0.0, DeepKeep = 0.0, ShallowShare = 0.0, ShallowLow = 1.0, LowKeep2 = 1.0, Rise = 0.0, EnergyErr = 0.0;
        double Last = -1.0;
        for (int i = 0; i < 4000; i++)
        {
            const double D = 0.05 + 0.0125 * i;    // to 50 m
            const FVoxelOceanSurf R = VoxelOceanSurfOf(Input(Beach, D / Beach.M, 100.0f), S);
            const double Share = R.Debug.x;
            if (D > Cut * 1.02) { DeepShare = std::max(DeepShare, Share); DeepKeep = std::max(DeepKeep, std::abs(1.0 - R.LongKeep)); }
            if (D < 0.4 * Cut * 0.98) { ShallowShare = std::max(ShallowShare, std::abs(Share - 0.75)); ShallowLow = std::min(ShallowLow, Share); }
            LowKeep2 = std::min(LowKeep2, double(R.LongKeep) * R.LongKeep);
            EnergyErr = std::max(EnergyErr, std::abs(double(R.LongKeep) * R.LongKeep + Share - 1.0));
            if (Last >= 0.0) Rise = std::max(Rise, Share - Last);
            Last = Share;
        }
        Check(DeepShare == 0.0, "past the cutoff: no trains (their share)", DeepShare, 0.0);
        Check(DeepKeep == 0.0, "past the cutoff: the long bands whole", DeepKeep, 0.0);
        Check(ShallowShare < 1.0e-4, "within 0.4 of the cutoff: the trains take 1 - floor (0.75)", ShallowShare, 1.0e-4);
        Check(LowKeep2 > 0.25 - 1.0e-4, "the FFT never under its floor (LongKeep^2)", LowKeep2, 0.25);
        Check(EnergyErr < 1.0e-4, "energy: LongKeep^2 + share = 1 (resolved)", EnergyErr, 1.0e-4);
        Check(Rise <= 1.0e-6, "the share never falls shoreward (monotone in depth)", Rise, 1.0e-6);
        // The floor's setting.
        double AllFFT = 0.0, NoFloor = 0.0;
        for (int i = 0; i < 100; i++)
        {
            const double X = 10.0 + 4.0 * i;
            AllFFT = std::max(AllFFT, double(VoxelOceanSurfOf(Input(Beach, X, 100.0f), Settings(1, 0.85f, 1, 1, 10, 0.2f, 1.0f)).Debug.x));
            NoFloor = std::max(NoFloor, double(VoxelOceanSurfOf(Input(Beach, X, 100.0f), Settings(1, 0.85f, 1, 1, 10, 0.2f, 0.0f)).Debug.x));
        }
        Check(AllFFT == 0.0, "floor 1: no trains", AllFFT, 0.0);
        Check(std::abs(NoFloor - 1.0) < 1.0e-5, "floor 0: the trains take all of it shallow", NoFloor, 1.0);
        // Cutoff 0: none.
        const FVoxelOceanSurf C0 = VoxelOceanSurfOf(Input(Beach, 50.0, 100.0f), Settings(1, 0.85f, 1, 1, 10, 0.0f));
        Check(C0.Debug.x < 1.0e-6, "cutoff 0: no trains", C0.Debug.x, 0.0);
        // No field: nothing (no trains, no swash).
        FVoxelOceanSurfInput In = Input(Beach, 20.0, 100.0f);
        In.CoastCover = 0.0f;
        const FVoxelOceanSurf N = VoxelOceanSurfOf(In, S);
        Check(N.Active == 0.0f && N.LongKeep == 1.0f && N.Height == 0.0f, "no coastal field: no surf (the FFT whole)", N.Active, 0.0);
        // The swash fades in with the cover (no line where the field ends along a coast).
        double Faint = 0.0;
        for (int k = 0; k < 600; k++)
        {
            FVoxelOceanSurfInput F = Input(Beach, -0.05 / 0.02, 0.5f * k);
            F.CoastCover = 1.0e-3f;
            Faint = std::max(Faint, std::abs(double(VoxelOceanSurfOf(F, S).Height)));
        }
        Check(Faint < 1.0e-4, "the swash at a cover of 0.001 (the field's end): next to nothing", Faint, 1.0e-4);
        // Half the cover: half the share.
        In.CoastCover = 0.5f;
        const FVoxelOceanSurf Half = VoxelOceanSurfOf(In, S);
        Check(std::abs(Half.Debug.x - 0.375) < 1.0e-4, "half the field's cover: half the share", Half.Debug.x, 0.375);
        // The field's reach: faded from 0.7 of it, none past it.
        double PastReach = 0.0;
        for (int i = 0; i < 50; i++)
        {
            FVoxelOceanSurfInput R = Input(Beach, 20.0, 100.0f);
            R.ShoreM = 8000.0f + 10.0f * i;
            PastReach = std::max(PastReach, double(VoxelOceanSurfOf(R, S).Debug.x));
        }
        Check(PastReach == 0.0, "past the field's reach: no trains", PastReach, 0.0);
        FVoxelOceanSurfInput Mid = Input(Beach, 20.0, 100.0f);
        Mid.ShoreM = 0.85f * 8000.0f;
        const double MidShare = VoxelOceanSurfOf(Mid, S).Debug.x;
        Check(MidShare > 0.3 && MidShare < 0.45, "at 0.85 of the reach: half faded", MidShare, 0.375);
    }

    std::printf("\n== the crests' spacing (the field's phase against the exact local wavelength)\n");
    {
        for (int b = 0; b < 3; b++)
        {
            const double M = b == 0 ? 0.01 : (b == 1 ? 0.02 : 0.05);
            const FBeach Bch(M);
            double Worst = 0.0, WorstD = 0.0;
            for (int t = 0; t < VOXEL_OCEAN_SURF_TRAINS; t++)
            {
                const double Cycles = VoxelOceanSurfCycles(t);
                const double W = 2.0 * 3.14159265358979 * Cycles / VOXEL_OCEAN_SURF_WRAP;
                const double L0 = 2.0 * 3.14159265358979 * 9.80665 / (W * W);
                for (int i = 0; i < 200; i++)
                {
                    const double D = 2.0 + (0.2 * L0 - 2.0) * i / 199.0;
                    if (D > 0.8 * Bch.Seed) continue;
                    const FVoxelOceanSurfInput In = Input(Bch, D / M, 100.0f);
                    const FVoxelOceanSurfTrain T = VoxelOceanSurfTrainOf(t, In, S, 1.5f, 0.0f, 0.0f, 0.0f, float(M), 0.0f, 1.0f);
                    // The spacing from the phase itself: the distance between crests along the ray.
                    const double Exact = 2.0 * 3.14159265358979 / ExactK(W, D);
                    const double Err = std::abs(T.Wavelength / Exact - 1.0);
                    if (Err > Worst) { Worst = Err; WorstD = D; }
                }
            }
            char What[128];
            std::snprintf(What, sizeof(What), "beach 1:%.0f: within the series' error, 2 m to a fifth of the deep wavelength (worst at %.1f m)", 1.0 / M, WorstD);
            Check(Worst < 0.06, What, Worst, 0.06);
        }
        // The phase's own change along the ray matches its gradient (no seam in the field's use).
        double Worst = 0.0;
        for (int i = 0; i < 400; i++)
        {
            const double X = 100.0 + 3.0 * i;
            const FVoxelOceanSurfTrain A = VoxelOceanSurfTrainOf(2, Input(Beach, X - 0.25, 100.0f), S, 1.5f, 0, 0, 0, 0.02f, 0, 1);
            const FVoxelOceanSurfTrain B = VoxelOceanSurfTrainOf(2, Input(Beach, X + 0.25, 100.0f), S, 1.5f, 0, 0, 0, 0.02f, 0, 1);
            const FVoxelOceanSurfTrain C = VoxelOceanSurfTrainOf(2, Input(Beach, X, 100.0f), S, 1.5f, 0, 0, 0, 0.02f, 0, 1);
            const double Numeric = (B.Phase - A.Phase) / 0.5;
            Worst = std::max(Worst, std::abs(Numeric / double(dot(C.GradPhase, Off)) - 1.0));
        }
        Check(Worst < 0.01, "the phase's change offshore against its gradient", Worst, 0.01);
    }

    std::printf("\n== derivatives against finite differences (sets 0; resolved)\n");
    {
        const FVoxelOceanSurfSettings S0 = Settings(0.0f);
        double SumErr = 0.0, SumRef = 0.0, SumErrT = 0.0, SumRefT = 0.0;
        for (int b = 0; b < 2; b++)
        {
            const FBeach Bch(b == 0 ? 0.02 : 0.05);
            for (int i = 0; i < 300; i++)
                for (int k = 0; k < 12; k++)
                {
                    const double D = -0.3 + 0.07 * i;
                    const double X = D / Bch.M;
                    const float T = 17.0f + 3.7f * k;
                    const double H = 0.002;
                    const double Up = VoxelOceanSurfOf(Input(Bch, X + H, T), S0).Height;
                    const double Dn = VoxelOceanSurfOf(Input(Bch, X - H, T), S0).Height;
                    const FVoxelOceanSurf R = VoxelOceanSurfOf(Input(Bch, X, T), S0);
                    const double Numeric = (Up - Dn) / (2.0 * H);
                    // Skip the profile's kinks (the crest and the trough of a skewed bore).
                    if (std::abs(Numeric - Offshore(R)) > 0.2 * std::abs(Numeric) + 0.01)
                    {
                        const double Up2 = VoxelOceanSurfOf(Input(Bch, X + 0.5 * H, T), S0).Height;
                        const double Dn2 = VoxelOceanSurfOf(Input(Bch, X - 0.5 * H, T), S0).Height;
                        const double Numeric2 = (Up2 - Dn2) / H;
                        if (std::abs(Numeric2 - Numeric) > 0.05 * std::abs(Numeric) + 1.0e-3) continue;
                    }
                    SumErr += (Numeric - Offshore(R)) * (Numeric - Offshore(R));
                    SumRef += Numeric * Numeric;
                    const double Tu = VoxelOceanSurfOf(Input(Bch, X, T + 0.002f), S0).Height;
                    const double Td = VoxelOceanSurfOf(Input(Bch, X, T - 0.002f), S0).Height;
                    const double NumericT = (Tu - Td) / 0.004;
                    SumErrT += (NumericT - R.Rate) * (NumericT - R.Rate);
                    SumRefT += NumericT * NumericT;
                }
        }
        Check(std::sqrt(SumErr / SumRef) < 0.05, "the slope offshore (phase, bed, inland fade): rms error / rms slope", std::sqrt(SumErr / SumRef), 0.05);
        Check(std::sqrt(SumErrT / SumRefT) < 0.02, "the rate: rms error / rms rate", std::sqrt(SumErrT / SumRefT), 0.02);
        // With the sets: the rate (the groups and the drift).
        double E = 0.0, Ref = 0.0;
        for (int i = 0; i < 200; i++)
            for (int k = 0; k < 10; k++)
            {
                const double X = (0.2 + 0.1 * i) / 0.02;
                const float T = 31.0f + 11.3f * k;
                const FVoxelOceanSurf R = VoxelOceanSurfOf(Input(Beach, X, T, 1.5f, 12.0f, 50.0f), S);
                const double Num = (VoxelOceanSurfOf(Input(Beach, X, T + 0.002f, 1.5f, 12.0f, 50.0f), S).Height
                    - VoxelOceanSurfOf(Input(Beach, X, T - 0.002f, 1.5f, 12.0f, 50.0f), S).Height) / 0.004;
                E += (Num - R.Rate) * (Num - R.Rate);
                Ref += Num * Num;
            }
        Check(std::sqrt(E / Ref) < 0.03, "the rate with the sets: rms error / rms rate", std::sqrt(E / Ref), 0.03);
    }

    std::printf("\n== continuity (across the still level, in depth, across the clock's wrap)\n");
    {
        double Step = 0.0, AtZero = 0.0;
        for (int k = 0; k < 400; k++)
        {
            const float T = 0.75f * k;
            double Prev = VoxelOceanSurfOf(Input(Beach, -1.5 / 0.02, T), S).Height;
            for (int i = 1; i <= 2000; i++)
            {
                const double X = (-1.5 + 0.002 * i) / 0.02;    // 10 cm steps from 1.5 m up the beach to 2.5 m deep
                const double H = VoxelOceanSurfOf(Input(Beach, X, T), S).Height;
                Step = std::max(Step, std::abs(H - Prev));
                Prev = H;
            }
            AtZero = std::max(AtZero, std::abs(double(VoxelOceanSurfOf(Input(Beach, 0.0005, T), S).Height) - VoxelOceanSurfOf(Input(Beach, -0.0005, T), S).Height));
        }
        Check(Step < 0.06, "no step over 10 cm along the beach (1.5 m up to 2.5 m deep, 300 s)", Step, 0.06);
        Check(AtZero < 0.002, "across the still level", AtZero, 0.002);
        double Wrap = 0.0;
        for (int i = 0; i < 200; i++)
        {
            const double X = (0.1 + 0.1 * i) / 0.02;
            const FVoxelOceanSurf A = VoxelOceanSurfOf(Input(Beach, X, VOXEL_OCEAN_SURF_WRAP - 0.001f), S);
            const FVoxelOceanSurf B = VoxelOceanSurfOf(Input(Beach, X, 0.0f), S);
            Wrap = std::max(Wrap, std::abs(double(A.Height) - B.Height - 0.001 * A.Rate));
        }
        Check(Wrap < 0.003, "across the clock's wrap", Wrap, 0.003);
    }

    std::printf("\n== the swash (Hs 1.5 m, 12 m/s; beach 1:20, reach 10 m)\n");
    {
        const FBeach Steep(0.05);
        double Lo = 1e9, Hi = -1e9, Sum = 0.0;
        int Count = 0;
        for (int k = 0; k < 6000; k++)
        {
            const float T = 0.1f * k;
            const double H = VoxelOceanSurfOf(Input(Steep, -0.01 / 0.05, T), S).Height;
            Lo = std::min(Lo, H);
            Hi = std::max(Hi, H);
            Sum += H;
            Count++;
        }
        std::printf("  the shoreline's level over 600 s: %.2f to %.2f m, mean %.2f m\n", Lo, Hi, Sum / Count);
        Check(Hi - Lo > 0.2, "the shoreline's level swings (run-up and run-down)", Hi - Lo, 0.2);
        Check(Sum / Count > 0.02, "its mean stands above the still level (setup)", Sum / Count, 0.02);
        Check(Hi < 0.5 * 10.0 * 0.05 * 2.0 + 0.3 * 1.5 * 1.3 + 0.01, "within the reach's hold and the setup's cap", Hi, 0.5 * 10.0 * 0.05 * 2.0 + 0.3 * 1.5 * 1.3);
        // Far up the beach (past three reaches): the still level.
        double Far = 0.0;
        for (int k = 0; k < 600; k++)
            Far = std::max(Far, std::abs(double(VoxelOceanSurfOf(Input(Steep, -35.0, 1.0f * k), S).Height)));
        Check(Far == 0.0, "past three reaches up the beach: the still level", Far, 0.0);
        // The bores reach the swash: the shoreline's level rises with the crests (correlated with
        // the sea's surface a little offshore, a lag later).
        double Best = -1.0;
        for (int Lag = 0; Lag < 80; Lag++)
        {
            double SAB = 0, SA = 0, SB = 0, SAA = 0, SBB = 0;
            for (int k = 0; k < 3000; k++)
            {
                const float T = 0.2f * k;
                const double A = VoxelOceanSurfOf(Input(Steep, 1.0 / 0.05, T), S).Height;
                const double B = VoxelOceanSurfOf(Input(Steep, -0.01 / 0.05, T + 0.1f * Lag), S).Height;
                SAB += A * B; SA += A; SB += B; SAA += A * A; SBB += B * B;
            }
            const double N = 3000.0;
            const double C = (SAB / N - SA / N * SB / N) / std::sqrt(std::max((SAA / N - SA * SA / (N * N)) * (SBB / N - SB * SB / (N * N)), 1e-12));
            Best = std::max(Best, C);
        }
        Check(Best > 0.4, "the swash follows the bores (best lagged correlation with 1 m deep)", Best, 0.4);
    }

    std::printf("\n== the swash on land by the mesh (stage O9i: a far mesh leaves the still level)\n");
    {
        const FBeach Steep(0.05);
        auto LandRange = [&](float Spacing, float Footprint, float ForNormals)
        {
            double Most = 0.0;
            for (int k = 0; k < 600; k++)
            {
                FVoxelOceanSurfInput In = Input(Steep, -5.0, 0.5f * k);
                In.SpacingM = Spacing;
                In.FootprintM = Footprint;
                In.ForNormals = ForNormals;
                Most = std::max(Most, std::abs(double(VoxelOceanSurfOf(In, S).Height)));
            }
            return Most;
        };
        const double Near = LandRange(0.0f, 0.0f, 0.0f);
        const double Fine = LandRange(2.0f, 0.0f, 0.0f);
        const double Coarse = LandRange(20.0f, 0.0f, 0.0f);
        const double CoarseN = LandRange(0.0f, 30.0f, 1.0f);
        std::printf("  5 m up the beach: %.3f m (no limit), %.3f m (2 m spacing), %.3f m (20 m), %.3f m (30 m footprint)\n", Near, Fine, Coarse, CoarseN);
        Check(Near > 0.05, "near: the sheet runs up the beach", Near, 0.05);
        Check(std::abs(Fine - Near) < 1.0e-6, "2 m spacing carries a 60 m sheet whole", std::abs(Fine - Near), 1.0e-6);
        Check(Coarse == 0.0, "20 m spacing: the still level (the terrain draws the waterline)", Coarse, 0.0);
        Check(CoarseN == 0.0, "30 m footprint (the normals, 2 a sheet's width): the still level", CoarseN, 0.0);
    }

    std::printf("\n== the trains by the mesh and the footprint\n");
    {
        // 8 m deep: the trains' wavelengths about 75 to 85 m; a 25 m spacing carries neither.
        FVoxelOceanSurfInput In = Input(Beach, 8.0 / 0.02, 50.0f);
        In.SpacingM = 25.0f;
        double Most = 0.0;
        for (int k = 0; k < 300; k++)
        {
            In.Time = 0.37f * k;
            Most = std::max(Most, std::abs(double(VoxelOceanSurfOf(In, S).Height)));
        }
        const FVoxelOceanSurf R = VoxelOceanSurfOf(In, S);
        Check(Most < 1.0e-6, "25 m spacing at 8 m deep: no trains in the height", Most, 1.0e-6);
        // Stage W3f: the composition is by place: the long bands keep 1 - share whatever the mesh
        // or the footprint shows of the trains.
        Check(std::abs(double(R.LongKeep) * R.LongKeep + R.Debug.x - 1.0) < 1.0e-5, "and the long bands keep 1 - share (composition by place)",
            std::abs(double(R.LongKeep) * R.LongKeep + R.Debug.x - 1.0), 1.0e-5);
        In.SpacingM = 0.0f;
        In.FootprintM = 45.0f;
        In.ForNormals = 1.0f;
        const FVoxelOceanSurf N = VoxelOceanSurfOf(In, S);
        Check(std::abs(double(N.LongKeep) - R.LongKeep) < 1.0e-6, "45 m footprint (the normals): the same composition", std::abs(double(N.LongKeep) - R.LongKeep), 1.0e-6);
        Check(dot(N.GradientPhase, N.GradientPhase) < 1.0e-12, "and no trains in the normals", std::sqrt(dot(N.GradientPhase, N.GradientPhase)), 0.0);
        // The same at every spacing.
        double Spread = 0.0;
        for (int i = 0; i <= 200; i++)
        {
            FVoxelOceanSurfInput P = Input(Beach, 6.0 / 0.02, 50.0f);
            P.SpacingM = 0.1f * float(i);
            const FVoxelOceanSurf Ri = VoxelOceanSurfOf(P, S);
            Spread = std::max(Spread, std::abs(double(Ri.LongKeep) * Ri.LongKeep + Ri.Debug.x - 1.0));
        }
        Check(Spread < 1.0e-5, "at every spacing from 0 to 20 m: the long bands keep 1 - share", Spread, 1.0e-5);
        // The bores' least front at a 2 m spacing: no slope steeper than a half cosine over 4
        // spacings of the wave's range.
        const FVoxelOceanSurfSettings S0 = Settings(0.0f);
        double Worst = 0.0;
        for (int k = 0; k < 60; k++)
        {
            const float T = 1.3f * k;
            double MaxSlope = 0.0, Lo = 1e9, Hi = -1e9;
            for (int i = 0; i < 4000; i++)
            {
                const double X = (0.8 + 0.0005 * i) / 0.02;     // 0.8 to 2.8 m deep (2.5 cm steps)
                FVoxelOceanSurfInput A = Input(Beach, X, T), B = Input(Beach, X + 0.025, T);
                A.SpacingM = 2.0f; B.SpacingM = 2.0f;
                const double HA = VoxelOceanSurfOf(A, S0).Height, HB = VoxelOceanSurfOf(B, S0).Height;
                MaxSlope = std::max(MaxSlope, std::abs(HB - HA) / 0.025);
                Lo = std::min(Lo, HA);
                Hi = std::max(Hi, HA);
            }
            Worst = std::max(Worst, MaxSlope * 4.0 * 2.0 / std::max(Hi - Lo, 1e-6));
        }
        Check(Worst < 1.6 * 1.25, "the bores' fronts at 2 m spacing: slope x 4 spacings / range (pi / 2 a half cosine)", Worst, 1.6 * 1.25);
    }

    std::printf("\n== the foam\n");
    {
        double Bore = 0.0, Deep = 0.0, Cap = 0.0, Calm = 0.0;
        for (int k = 0; k < 600; k++)
        {
            const float T = 0.5f * k;
            Bore = std::max(Bore, double(VoxelOceanSurfOf(Input(Beach, 1.0 / 0.02, T), S).Foam));
            Deep = std::max(Deep, double(VoxelOceanSurfOf(Input(Beach, 9.0 / 0.02, T, 0.8f, 8.0f), S).Foam));
            // A train's crests by their steepness alone (no bores' carpet): the 5 s train 10 m deep.
            FVoxelOceanSurfInput In = Input(Beach, 10.0 / 0.02, T);
            Cap = std::max(Cap, double(VoxelOceanSurfTrainOf(0, In, S, 4.0f, 0.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f).Foam));
            Calm = std::max(Calm, double(VoxelOceanSurfTrainOf(0, In, S, 1.0f, 0.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f).Foam));
        }
        std::printf("  the most: %.2f (bores 1 m deep, Hs 1.5), %.2f (9 m deep, Hs 0.8); a 5 s train's whitecaps 10 m deep: %.2f (Hs 4), %.2f (Hs 1)\n", Bore, Deep, Cap, Calm);
        Check(Bore > 0.5, "the bores make white water in the surf zone", Bore, 0.5);
        Check(Deep < 0.15, "a small sea past its surf zone: little", Deep, 0.15);
        Check(Cap > 0.3, "a steep train's crests whiten (whitecaps, past the bores' carpet)", Cap, 0.3);
        Check(Calm == 0.0, "a gentle one's don't", Calm, 0.0);
        const FVoxelOceanSurf In = VoxelOceanSurfOf(Input(Beach, 5.0 / 0.02, 10.0f), S);
        const FVoxelOceanSurf Out = VoxelOceanSurfOf(Input(Beach, 45.0 / 0.02, 10.0f), S);
        Check(In.SteepVar > 1.0e-4, "the trains' slope variance for the open sea's foam where they are", In.SteepVar, 1.0e-4);
        Check(Out.SteepVar == 0.0f, "none past the cutoff", Out.SteepVar, 0.0);
        // The swash's sheet: foamy where thin.
        double Sheet = 0.0;
        for (int k = 0; k < 300; k++)
            Sheet = std::max(Sheet, double(VoxelOceanSurfOf(Input(Beach, -0.02 / 0.02, 1.0f * k), S).Foam));
        Check(Sheet > 0.5, "the swash's thin sheet", Sheet, 0.5);
        const FVoxelOceanSurf NoFoam = VoxelOceanSurfOf(Input(Beach, 1.0 / 0.02, 10.0f), Settings(1, 0.85f, 1, 1, 10, 0.2f, 0.25f, 0.0f));
        Check(NoFoam.Foam == 0.0f, "surf foam 0: none", NoFoam.Foam, 0.0);
    }

    std::printf("\n== the bound\n");
    {
        for (int Strong = 0; Strong < 2; Strong++)
        {
            const FVoxelOceanSurfSettings SS = Strong ? Settings(1.0f, 0.85f, 2.0f, 2.0f) : S;
            const float Speed = Strong ? 24.0f : 15.0f;
            double Highest = 0.0;
            for (int b = 0; b < 3; b++)
            {
                const FBeach Bch(b == 0 ? 0.005 : (b == 1 ? 0.02 : 0.1));
                for (int j = 0; j < 80; j++)
                    for (int k = 0; k < 900; k++)
                    {
                        const double D = -0.5 + 0.15 * j;
                        Highest = std::max(Highest, double(VoxelOceanSurfOf(Input(Bch, D / Bch.M, 0.9f * k, 2.0f, Speed, 37.0f * j), SS).Height));
                    }
            }
            const double Bound = VoxelOceanSurfMaxHeight(2.0f, SS);
            std::printf("  %s: highest %.2f m for Hs 2 m (bound %.2f m)\n", Strong ? "strength and swash 2" : "defaults", Highest, Bound);
            Check(Highest < Bound, "within the surf's bound (VoxelOceanSurfMaxHeight, the model's reach)", Highest, Bound);
        }
    }

    std::printf("\n%s: %d failure(s)\n", Fails ? "FAILED" : "PASSED", Fails);
    return Fails ? 1 : 0;
}
