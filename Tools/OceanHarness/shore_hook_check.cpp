// O8: the shoaled FFT waves, hook against model, near a coast, without the engine.
// - The planet: a round continent (a spherical cap, heights rising 1 m in 100 m across its
//   coast) on the 1,075 km sea; its land bake at N 256 marked sea-connected (VoxelLandSea.h) and
//   its shore field built with the shared passes (VoxelOceanShoreCore.h).
// - The hook: hook_emulation.h (VoxelOceanWaves.ush's FFT path ported, with VoxelOceanShoal.ush
//   and the field's sampling), the depth the terrain's at the point (as the mesh carries it).
// - The model: VoxelOceanWavesFFT.cpp unchanged, sites carrying the same depth and the field's
//   sample (as FillSites and FVoxelOceanShoreField::Sample give them).
// Checks: drawn against queried waves near the coast; shoaling at work (amplitude, the warp); the
// waves shorter toward the coast; no seams; the underwater waterline (VoxelOceanHeightAtFFTShore)
// against the query's inversion; the reach.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -IUEStandIn -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
//       -I<plugin>/Shaders/Private -I. shore_hook_check.cpp <plugin>/Source/VoxelPlugin/Private/VoxelOceanWavesFFT.cpp
//       <plugin>/Source/VoxelPlugin/Private/VoxelOceanWeather.cpp <plugin>/Source/VoxelPlugin/Private/VoxelOceanWeatherTable.cpp -o shore_hook_check
#include "hook_emulation.h"
#include "VoxelLandSea.h"

#include <chrono>
#include <cstdio>
#include <random>
#include <thread>

using namespace VoxelShader;
using namespace HookEmulation;

static int Failures = 0;
static void Expect(bool Ok, const char* What)
{
    std::printf("  [%s] %s\n", Ok ? "ok" : "FAIL", What);
    if (!Ok) Failures++;
}

static float3 F3(const FVector& V) { return float3(float(V.X), float(V.Y), float(V.Z)); }

// Two unit tangents at a unit vector.
static void Axes(const FVector& U, FVector& E1, FVector& E2)
{
    E1 = (std::fabs(U.X) < 0.9 ? FVector(1, 0, 0) : FVector(0, 1, 0));
    E1 = (E1 - U * (U | E1)).GetSafeNormal();
    E2 = U ^ E1;
}

// ---------------------------------------------------------------------------------------------
// The planet: a round continent
// ---------------------------------------------------------------------------------------------

static const double Radius = 1.075e8;                 // cm
static const FVector Continent = FVector(0.4, -0.3, 0.866).GetSafeNormal();
static const double ContinentAngle = 0.3;             // rad
static const double CoastSlope = 0.01;

// The terrain's height (m above sea level) at a direction: rising CoastSlope across the coast,
// held within +-3000 m.
static double HeightAt(const FVector& Dir)
{
    const double Theta = std::acos(std::clamp(Dir | Continent, -1.0, 1.0));
    return std::clamp(CoastSlope * (ContinentAngle - Theta) * Radius * 0.01, -3000.0, 3000.0);
}

// The depth (cm, positive under water), as the mesh would carry it.
static float DepthAt(const FVector& Dir) { return float(-HeightAt(Dir) * 100.0); }

// A point s metres seaward of the coast, Along radians around it.
static FVector CoastPoint(double Along, double SeawardM)
{
    FVector E1, E2;
    Axes(Continent, E1, E2);
    const double Theta = ContinentAngle + SeawardM * 100.0 / Radius;
    const FVector Around = E1 * std::cos(Along) + E2 * std::sin(Along);
    return (Continent * std::cos(Theta) + Around * std::sin(Theta)).GetSafeNormal();
}

// ---------------------------------------------------------------------------------------------

struct FField
{
    int N = 256;
    double ReachM = 40000.0;
    std::vector<uint32_t> Texels;

    FVoxelOceanShore Sample(const FVector& P) const
    {
        const FVoxelOceanShoreSample S = VoxelOceanShoreCore::Sample(Texels.data(), N, ReachM, P.X, P.Y, P.Z);
        FVoxelOceanShore Out;
        Out.Distance = double(S.Distance) * 100.0;
        Out.Direction = FVector(S.Direction.x, S.Direction.y, S.Direction.z);
        Out.Slope = S.Slope;
        return Out;
    }
};

static FVoxelOceanWaveResult EvaluateAt(const FVoxelOceanWavesFFT& Model, const FVoxelOceanWaveContext& Context, const FField& Field, const FVector& Dir)
{
    FVoxelOceanWaveSite Site;
    Site.Point = Dir * Radius;
    Site.Up = Dir;
    Site.Environment = Context.Environment;
    Site.Depth = DepthAt(Dir);
    Site.Shore = Field.Sample(Site.Point);
    FVoxelOceanWaveResult R;
    Model.Evaluate(Context, TConstArrayView<FVoxelOceanWaveSite>(&Site, 1), TArrayView<FVoxelOceanWaveResult>(&R, 1));
    return R;
}

// VoxelOceanQuery::Run's inversion (ported), each step's site filled as FillSites does.
struct FQueryOut { double Height; bool bSettled; };
static FQueryOut Query(const FVoxelOceanWavesFFT& Model, const FVoxelOceanWaveContext& Context, const FField& Field, const FVector& Point)
{
    const FVector U = Point.GetSafeNormal();
    const FVector Foot = U * Radius;
    FVector X = Foot;
    FVoxelOceanWaveResult R = EvaluateAt(Model, Context, Field, X.GetSafeNormal());
    bool bDone = false;
    for (int Step = 0; Step < 8 && !bDone; Step++)
    {
        const FVector Up = X.GetSafeNormal();
        FVector Residual = X + (R.Displacement - Up * R.Height) - Foot;
        Residual -= U * (U | Residual);
        if (Residual.Size() < 1.0e-3) { bDone = true; break; }
        FVector E1 = (std::fabs(U.X) < 0.9 ? FVector(1, 0, 0) : FVector(0, 1, 0)); E1 = (E1 - U * (U | E1)).GetSafeNormal();
        const FVector E2 = U ^ E1;
        auto Apply = [&R](const FVector& V) { return FVector(R.HorizontalGradient[0] | V, R.HorizontalGradient[1] | V, R.HorizontalGradient[2] | V); };
        const FVector JE1 = Apply(E1), JE2 = Apply(E2);
        const double A11 = 1.0 + (E1 | JE1), A12 = E1 | JE2, A21 = E2 | JE1, A22 = 1.0 + (E2 | JE2);
        const double Det = std::max(A11 * A22 - A12 * A21, 0.05);
        const double R1 = E1 | Residual, R2 = E2 | Residual;
        X = (X - (E1 * ((A22 * R1 - A12 * R2) / Det) + E2 * ((A11 * R2 - A21 * R1) / Det))).GetSafeNormal() * Radius;
        R = EvaluateAt(Model, Context, Field, X.GetSafeNormal());
    }
    if (!bDone)
    {
        const FVector Up = X.GetSafeNormal();
        FVector Residual = X + (R.Displacement - Up * R.Height) - Foot;
        Residual -= U * (U | Residual);
        bDone = Residual.Size() < 1.0e-3;
    }
    return { Point.Size() - Radius - R.Height, bDone };
}

int main()
{
    // ---- The bake and the field ------------------------------------------------------------------
    FField Field;
    const int N = Field.N, Pad = VoxelOceanShoreCore::Pad, S = VoxelOceanShoreCore::Side(N);
    std::vector<float> Height(static_cast<size_t>(VoxelOceanShoreCore::NumTexels(N)));
    std::vector<uint16_t> Attr(Height.size(), 0);
    for (int F = 0; F < 6; F++)
        for (int J = -Pad; J < N + Pad; J++)
            for (int I = -Pad; I < N + Pad; I++)
            {
                double A[9];
                VoxelLandSea::Detail::Face(F, A);
                const double TU = std::tan((-1.0 + (2.0 * I + 1.0) / N) * 0.7853981633974483), TV = std::tan((-1.0 + (2.0 * J + 1.0) / N) * 0.7853981633974483);
                const FVector D = FVector(A[0] + A[3] * TU + A[6] * TV, A[1] + A[4] * TU + A[7] * TV, A[2] + A[5] * TU + A[8] * TV).GetSafeNormal();
                Height[size_t((F * S + J + Pad) * S + I + Pad)] = float(HeightAt(D));
            }
    const VoxelLandSea::FStats Sea = VoxelLandSea::Mark(N, Pad, Height.data(), Attr.data(), 0x800, 0.02);
    const auto T0 = std::chrono::steady_clock::now();
    Field.Texels = VoxelOceanShoreCore::Build(N, Radius * 0.01, Field.ReachM, Height.data(), Attr.data(), [](int Count, auto&& Fn)
        {
            const int Threads = std::max(1u, std::thread::hardware_concurrency());
            std::vector<std::thread> Pool;
            for (int t = 0; t < Threads; t++)
                Pool.emplace_back([&, t] { for (int i = t; i < Count; i += Threads) Fn(i); });
            for (std::thread& Th : Pool) Th.join();
        });
    std::printf("Planet: a continent of radius %.0f km; bake N %d (cells %.1f km), %d sea cells; shore field built in %.0f ms\n",
        ContinentAngle * Radius * 1e-5, N, 1.5708 * Radius * 1e-5 / N, Sea.SeaCells,
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T0).count());

    // The field's distance against the true one (the coast is a circle).
    {
        double Max = 0.0;
        for (int k = 0; k < 200; k++)
        {
            const double Sd = 100.0 + 50.0 * k;
            const FVector P = CoastPoint(0.37 * k, Sd);
            Max = std::max(Max, std::abs(Field.Sample(P * Radius).Distance * 0.01 - Sd));
        }
        std::printf("  field distance against the true one (0.1 to 10 km out): up to %.0f m\n", Max);
        Expect(Max < 0.5 * 1.5708 * Radius * 1e-2 / N, "the field finds the coast within half a bake cell");
    }

    // Along the coast at a constant true distance: how much the field's distance and direction
    // wobble (the seeds are crossings a cell apart; the warp moves the waves' lookup by
    // kilometres along the direction, so its swings tear the waves sideways).
    {
        const double CellM = 1.5708 * Radius * 1e-2 / N;
        const double Offsets[] = { -3.0, -1.0, -0.3, 0.3, 1.0, 2.0, 3.0, 5.0 };   // within the reach (6 cells)
        double WorstNear = 0.0, WorstFar = 0.0, WorstDistance = 0.0, WorstStep = 0.0, WorstLength = 1.0;
        for (const double Cells : Offsets)
        {
            double Angle = 0.0, Distance = 0.0, Step = 0.0, Shortest = 1.0;
            FVector Previous = FVector::ZeroVector;
            const int Count = 4000;
            for (int k = 0; k <= Count; k++)
            {
                const double Along = 6.283185307179586 * k / Count;   // ~0.5 km apart
                const FVector P = CoastPoint(Along, Cells * CellM);
                const FVoxelOceanShore Sh = Field.Sample(P * Radius);
                FVector True = Continent - P * (P | Continent);
                True = True.GetSafeNormal();
                const double Length = Sh.Direction.Size();
                Shortest = std::min(Shortest, Length);
                const FVector Dir = Sh.Direction / std::max(Length, 1e-9);
                Angle = std::max(Angle, std::acos(std::clamp(Dir | True, -1.0, 1.0)) * 57.29577951308232);
                Distance = std::max(Distance, std::abs(Sh.Distance * 0.01 - Cells * CellM));
                if (k > 0) Step = std::max(Step, std::acos(std::clamp(Dir | Previous, -1.0, 1.0)) * 57.29577951308232);
                Previous = Dir;
            }
            std::printf("  %5.1f cells out: direction off by up to %.2f deg (%.3f deg per 0.5 km step, length at least %.3f), distance by up to %.0f m\n",
                Cells, Angle, Step, Shortest, Distance);
            (std::abs(Cells) <= 1.0 ? WorstNear : WorstFar) = std::max(std::abs(Cells) <= 1.0 ? WorstNear : WorstFar, Angle);
            WorstDistance = std::max(WorstDistance, Distance / CellM);
            WorstStep = std::max(WorstStep, Step);
            WorstLength = std::min(WorstLength, Shortest);
        }
        // The angle's code is 1.4 degrees; the smoothing's window takes in a curved coast's turn.
        // (Before the gradient and the smoothing: up to 55 degrees off near the coast, turning 11
        // degrees between samples, its length down to 0.46.)
        Expect(WorstNear < 4.0 && WorstFar < 4.0, "the direction along a round coast stays within 4 degrees of the true one, near it and away");
        Expect(WorstStep < 0.5, "the direction turns less than half a degree between samples 0.5 km apart along the coast");
        Expect(WorstLength > 0.95, "the direction keeps its length along the coast (the warp's full weight)");
        Expect(WorstDistance < 0.15, "the distance along the coast stays within 0.15 cells of the true one");
    }

    // The texel-space expansion around a reference against the exact (double) lookup, by distance.
    {
        const FVector Ref = CoastPoint(2.0, 800.0) * (Radius + 500.0);
        float T[28];
        VoxelOceanShoreCore::FrameOf(Ref.X, Ref.Y, Ref.Z, N, T);
        const FVoxelOceanShoreFrame Frame = VoxelOceanShoreCore::FrameFrom(T);
        double Worst1 = 0.0, Worst4 = 0.0;
        std::mt19937_64 R(3);
        for (int k = 0; k < 4000; k++)
        {
            const double Range = k < 2000 ? 100000.0 : 400000.0;   // 1 km, 4 km
            FVector E1, E2;
            Axes(Ref.GetSafeNormal(), E1, E2);
            const double A = std::uniform_real_distribution<double>(0, 6.2832)(R), Rr = Range * std::sqrt(std::uniform_real_distribution<double>(0, 1)(R));
            const FVector Point = (Ref + E1 * (Rr * std::cos(A)) + E2 * (Rr * std::sin(A))).GetSafeNormal() * Radius;
            const FVector D = Point - Ref;
            // The texel coordinate both ways (on the reference's face).
            const int Face = int(T[1] + 0.5f);
            double Pp[3] = { Point.X, Point.Y, Point.Z };
            const int UA = int(VoxelCubeUAxis(unsigned(Face))), VA = int(VoxelCubeVAxis(unsigned(Face))), Ax = Face >> 1;
            const double NS = (Face & 1) ? -1.0 : 1.0;
            const double GX = (std::atan(Pp[UA] / (NS * Pp[Ax])) * 4.0 / 3.141592653589793 + 1.0) * 0.5 * N - 0.5 + Pad;
            const double GY = (std::atan(Pp[VA] / (NS * Pp[Ax])) * 4.0 / 3.141592653589793 + 1.0) * 0.5 * N - 0.5 + Pad;
            const float3 Df = F3(D);
            const double EX = Frame.BaseX + double(Frame.FracX + VoxelOceanShoreFrameDelta(Frame.GradX, Frame.HessXDiag, Frame.HessXOff, Df));
            const double EY = Frame.BaseY + double(Frame.FracY + VoxelOceanShoreFrameDelta(Frame.GradY, Frame.HessYDiag, Frame.HessYOff, Df));
            const double Err = std::sqrt((EX - GX) * (EX - GX) + (EY - GY) * (EY - GY)) * 1.5708 * Radius / N;   // cm
            (k < 2000 ? Worst1 : Worst4) = std::max(k < 2000 ? Worst1 : Worst4, Err);
        }
        std::printf("  texel-space expansion against the exact lookup: up to %.2f mm within 1 km, %.1f mm within 4 km\n", Worst1 * 10.0, Worst4 * 10.0);
        Expect(Worst1 < 0.2 && Worst4 < 5.0, "the expansion places the field within 2 mm a kilometre out, 5 cm at 4 km");
    }

    // ---- The model and the hook ------------------------------------------------------------------
    for (int Case = 0; Case < 2; Case++)
    {
        // A wind sea, then a swell on a light wind.
        FVoxelOceanEnvironment E;
        E.WindSpeed = Case == 0 ? 12.0 : 3.0;
        E.WindHeading = 0.6;
        E.SwellSpeed = Case == 0 ? 0.0 : 14.0;
        E.SwellHeading = 2.0;
        const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> Source(MakeShared<FVoxelOceanConstantWeather, ESPMode::ThreadSafe>(E));
        FVoxelOceanWeatherTiming Timing;
        Timing.HeadingStepDegrees = 0.0;
        FVoxelOceanFFTSettings Settings;
        Settings.Seed = 11;
        Settings.WindSpeed = FVoxelOceanWavesFFT::ReachWind(std::max(E.WindSpeed, E.SwellSpeed), 0.0, false);
        Settings.ShoreFade = 0.0;   // the shoaling up to the beach
        const FVoxelOceanWavesFFT Model(Settings, Radius);
        const FVoxelOceanWeatherKey Key = VoxelOceanWeatherTable::BuildKey(*Source, Model.GetCharts().N, Timing, 0);
        const TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe> Table = VoxelOceanWeatherTable::MakeTable(Model.GetCharts(), Key, Key, Key, Model.GetTableConstants(), Source, Timing);
        FVoxelOceanWaveContext Context;
        Context.Time = 40.0;
        Context.Radius = Radius;
        Context.Weather = Table;
        Context.ShoreCells = N;
        Context.ShoreReachM = Field.ReachM;
        FTextures Tx;
        Tx.Charts = &Table->Table;
        Tx.Shore = Field.Texels.data();
        Tx.ShoreN = N;
        const uint32_t Tick = VoxelOceanSpectrum::TickOf(Context.Time);
        for (int s = 0; s < VoxelOceanSpectrum::SlotCount; s++)
        {
            const OceanFFTEmulation::FGPUSlot G = OceanFFTEmulation::EmulateGPU(Model.GetSlots()[s], Tick);
            Tx.Displacement.Build(s, G.Displacement);
            Tx.Derivatives.Build(s, G.Derivatives);
        }
        const FQuat Rot = [] { const double a = 0.7, sn = std::sin(a / 2); const FVector Ax = FVector(0.2, 0.9, -0.3).GetSafeNormal(); return FQuat(Ax.X * sn, Ax.Y * sn, Ax.Z * sn, std::cos(a / 2)); }();
        const FTransform PlanetToWorld(Rot, FVector(3.0e6, 1.0e7, -2.0e5));
        std::printf("\n%s (wind %.0f m/s, swell %.0f m/s): reach %.0f cm up, %.0f cm sideways\n",
            Case == 0 ? "Wind sea" : "Swell", E.WindSpeed, E.SwellSpeed, Model.MaxHeight(), Model.MaxHorizontal());

        // Drawn (the hook, mip 0, the long bands) against queried (Evaluate) near the coast.
        std::mt19937_64 Rng(5 + Case);
        double EH = 0.0, MH = 0.0, SumAmp = 0.0, MaxShift = 0.0, MaxHeightSeen = 0.0;
        int Count = 0, Active = 0;
        for (int k = 0; k < 1500; k++)
        {
            const double Sd = std::exp(std::uniform_real_distribution<double>(std::log(5.0), std::log(6000.0))(Rng));
            const FVector Dir = CoastPoint(std::uniform_real_distribution<double>(0, 6.2832)(Rng), Sd);
            const FVector Ref = (Dir * Radius + FVector(1, 2, 3).GetSafeNormal() * 3000.0).GetSafeNormal() * (Radius + 300.0);
            FVoxelOceanWaveShaderParams SP;
            Model.GetShaderParams(Context, Ref, PlanetToWorld, SP);
            FParams W = ParamsOf(SP);
            W.bLongOnly = true;
            const FVector PW = PlanetToWorld.TransformPositionNoScale(Dir * Radius) - PlanetToWorld.TransformPositionNoScale(Ref);
            const FVector UpW = PlanetToWorld.TransformVectorNoScale(Dir);
            const FShore Sh = ShoreAt(Tx, F3(PW), F3(UpW), W);
            const FPoint Hook = HookEvaluate(Tx, F3(PW), F3(UpW), 0.0f, 0.0f, W, 3u, DepthAt(Dir), Sh);
            const FVoxelOceanWaveResult R = EvaluateAt(Model, Context, Field, Dir);
            EH = std::max(EH, std::abs(double(Hook.R.Height) - R.Height));
            MH = std::max(MH, std::abs(R.Height));
            MaxHeightSeen = std::max(MaxHeightSeen, std::abs(R.Height));
            if (Hook.Shoal.Active > 0.0f)
            {
                Active++;
                SumAmp += Hook.Shoal.Amp.x;
                MaxShift = std::max(MaxShift, double(Hook.Shoal.Shift.x));
            }
            Count++;
        }
        std::printf("  %d points 5 m to 6 km off the coast: drawn against queried height up to %.1f mm of waves up to %.2f m; shoaling at %d, long band's mean amplitude %.2f, its largest move %.0f m\n",
            Count, EH * 10.0, MH * 0.01, Active, SumAmp / std::max(1, Active), MaxShift);
        Expect(EH < 40.0 * std::max(1.0, MH / 300.0), "drawn and queried shoaled waves agree within 4 cm per 3 m of waves");
        Expect(Active > Count / 2 && MaxShift > 50.0, "the shoaling and the warp are at work near the coast");
        Expect(MaxHeightSeen <= Model.MaxHeight(), "the shoaled waves stay within the model's reach");

        // Shorter waves toward the coast: zero crossings of the long bands along transects toward
        // the shore, near (3 to 10 m deep) against far (40 m deep and more).
        {
            double Near = 0.0, Far = 0.0;
            const int Transects = 24;
            for (int t = 0; t < Transects; t++)
            {
                const double Along = 0.26 * t;
                auto Crossings = [&](double From, double To)
                    {
                        int Count2 = 0;
                        double Prev = 0.0;
                        const int Steps = int((To - From) / 1.0);
                        for (int i = 0; i <= Steps; i++)
                        {
                            const FVoxelOceanWaveResult R = EvaluateAt(Model, Context, Field, CoastPoint(Along, From + i * 1.0));
                            if (i > 0 && (R.Height > 0.0) != (Prev > 0.0)) Count2++;
                            Prev = R.Height;
                        }
                        return double(Count2) / (To - From);
                    };
                Near += Crossings(300.0, 1000.0);
                Far += Crossings(4000.0, 4700.0);
            }
            std::printf("  zero crossings per km along the way to the coast: %.1f near (3-10 m deep), %.1f far (40-47 m deep)\n", Near / Transects * 1000.0, Far / Transects * 1000.0);
            Expect(Near > 1.15 * Far, "the waves are shorter toward the coast (the warp)");
        }

        // Continuity of the hook (all bands, mip 0): 1 mm steps anywhere near the coast.
        {
            double Worst = 0.0, Steepest = 0.0;
            std::mt19937_64 R2(9);
            const FVector Ref = CoastPoint(1.0, 500.0) * (Radius + 200.0);
            FVoxelOceanWaveShaderParams SP;
            Model.GetShaderParams(Context, Ref, PlanetToWorld, SP);
            const FParams W = ParamsOf(SP);
            for (int k = 0; k < 3000; k++)
            {
                const FVector Dir = CoastPoint(1.0 + std::uniform_real_distribution<double>(-0.001, 0.001)(R2), std::uniform_real_distribution<double>(5.0, 3000.0)(R2));
                FVector T1, T2;
                Axes(Dir, T1, T2);
                const FVector Dir2 = (Dir * Radius + T1 * 0.1).GetSafeNormal();
                // The radial as the shader forms it: the point from the reference less the planet
                // centre from it, in float (quantized to a few cm on this sphere).
                const float3 CentreW = F3(PlanetToWorld.TransformPositionNoScale(FVector(0, 0, 0)) - PlanetToWorld.TransformPositionNoScale(Ref));
                auto HeightOf = [&](const FVector& D)
                    {
                        const FVector PW = PlanetToWorld.TransformPositionNoScale(D * Radius) - PlanetToWorld.TransformPositionNoScale(Ref);
                        const float3 UpW = normalize(F3(PW) - CentreW);
                        const FPoint P = HookEvaluate(Tx, F3(PW), UpW, 0.0f, 0.0f, W, 3u, DepthAt(D), ShoreAt(Tx, F3(PW), UpW, W));
                        Steepest = std::max(Steepest, double(length(float3(P.R.Gradient.x, P.R.Gradient.y, P.R.Gradient.z))));
                        return double(P.R.Height);
                    };
                Worst = std::max(Worst, std::abs(HeightOf(Dir) - HeightOf(Dir2)));
            }
            std::printf("  continuity: up to %.3f mm per 1 mm step (the steepest slope met %.2f)\n", Worst, Steepest);
            Expect(Worst <= 1.5 * std::max(Steepest, 0.05) * 0.1 + 0.05, "no seam (every step within 1.5 times the steepest slope)");
        }

        // The surf (stage O9a): drawn against queried from 60 m up the beach to 400 m out (at the
        // emulated cascades' time); over a minute the swash wets and dries the beach (the model).
        {
            std::mt19937_64 R4(29 + Case);
            double Worst = 0.0, Highest = 0.0;
            int Wet = 0, Dry = 0;
            const double BeachUp = 15.0;    // m up the beach: 0.15 m above the still level
            FVoxelOceanWaveContext SurfContext = Context;
            for (int k = 0; k < 600; k++)
            {
                SurfContext.Time = Context.Time + 0.1 * k;
                const double Sd = std::uniform_real_distribution<double>(-60.0, 400.0)(R4);
                const FVector Dir = CoastPoint(std::uniform_real_distribution<double>(0, 6.2832)(R4), Sd);
                const FVector Ref = (Dir * Radius + FVector(1, 2, 3).GetSafeNormal() * 300.0).GetSafeNormal() * (Radius + 200.0);
                FVoxelOceanWaveShaderParams SP;
                Model.GetShaderParams(Context, Ref, PlanetToWorld, SP);
                FParams W = ParamsOf(SP);
                W.bLongOnly = true;
                const FVector PW = PlanetToWorld.TransformPositionNoScale(Dir * Radius) - PlanetToWorld.TransformPositionNoScale(Ref);
                const FVector UpW = PlanetToWorld.TransformVectorNoScale(Dir);
                const FPoint Hook = HookEvaluate(Tx, F3(PW), F3(UpW), 0.0f, 0.0f, W, 3u, DepthAt(Dir), ShoreAt(Tx, F3(PW), F3(UpW), W));
                const FVoxelOceanWaveResult R = EvaluateAt(Model, Context, Field, Dir);
                Worst = std::max(Worst, std::abs(double(Hook.R.Height) - R.Height));
                Highest = std::max(Highest, R.Height);
                // The beach point 15 m up: wet when the level stands over the terrain.
                const FVector Beach = CoastPoint(0.37 * k, -BeachUp);
                const FVoxelOceanWaveResult B = EvaluateAt(Model, SurfContext, Field, Beach);
                (B.Height > -DepthAt(Beach) ? Wet : Dry)++;
            }
            std::printf("  surf from 60 m up the beach to 400 m out: drawn against queried up to %.1f mm, highest %.2f m; a beach point 0.15 m up wet %d of %d times\n",
                Worst * 10.0, Highest * 0.01, Wet, Wet + Dry);
            Expect(Worst < 2.0, "drawn and queried surf agree within 2 cm");
            Expect(Highest <= Model.MaxHeight(), "the surf stays within the model's reach");
            Expect(Wet > 30 && Dry > 30, "the swash wets and dries the beach");
        }

        // The underwater waterline (VoxelOceanHeightAtFFTShore, the depth at the foot) against the
        // query's inversion near the coast.
        {
            std::mt19937_64 R3(13);
            double Worst = 0.0;
            int Settled = 0, Total = 0;
            for (int k = 0; k < 200; k++)
            {
                const FVector Dir = CoastPoint(std::uniform_real_distribution<double>(0, 6.28)(R3), std::exp(std::uniform_real_distribution<double>(std::log(20.0), std::log(3000.0))(R3)));
                const FVector Camera = Dir * (Radius + 150.0);
                const FVector Q = (Dir * Radius + FVector(0.3, 0.5, 0.1).GetSafeNormal() * 200.0).GetSafeNormal() * (Radius - 30.0);
                FVoxelOceanWaveShaderParams SP;
                Model.GetShaderParams(Context, Camera, PlanetToWorld, SP);
                FParams W = ParamsOf(SP);
                W.bLongOnly = true;
                const FVector QW = PlanetToWorld.TransformVectorNoScale(Q - Camera);
                const FVector CentreW = PlanetToWorld.TransformVectorNoScale(-Camera);
                FPoint Out;
                // The depth at each step's point (the pass's probe: VOXEL_OCEAN_HEIGHT_DEPTH).
                const FQuat ToWorld = PlanetToWorld.GetRotation();
                auto DepthOf = [&](float3 X) { return DepthAt((Camera + ToWorld.UnrotateVector(FVector(X.x, X.y, X.z))).GetSafeNormal()); };
                const float HookAbove = HookHeightAt(Tx, F3(QW), F3(CentreW), Radius, Camera.Size() - Radius, W, Out, 3, 1u, 0.0f, DepthAt(Q.GetSafeNormal()), DepthOf);
                const FQueryOut QO = Query(Model, Context, Field, Q);
                Settled += QO.bSettled ? 1 : 0;
                Total++;
                Worst = std::max(Worst, std::abs(double(HookAbove) - QO.Height));
            }
            std::printf("  the waterline against the query: up to %.1f mm; %d of %d queries settled\n", Worst * 10.0, Settled, Total);
            Expect(Settled >= Total * 95 / 100, "the query settles near the coast");
            Expect(Worst < 6.0, "the underwater waterline matches the query within 6 cm (the depth read at each step)");
        }
    }

    std::printf("\n%s (%d failures)\n", Failures ? "FAILED" : "ALL OK", Failures);
    return Failures ? 1 : 0;
}
