// O6b, O7: the FFT wave hook against the FFT model, in the procedural weather, without the engine.
// - The hook: VoxelOceanWaves.ush's FFT path (VoxelOceanFFTEvaluate and its lookups) ported line
//   for line, over emulated cascade textures (VoxelOceanFFT.usf's passes in float, their mips
//   averaged as MipCS, trilinear wrapping lookups) and the weather's table; the shared code
//   (charts, sea state, headings, detiling, the sum, the normal) is the shaders' own.
// - The model: VoxelOceanWavesFFT.cpp compiled unchanged (UEStandIn/ stands in for the engine),
//   with the weather (VoxelOceanWeather.cpp, VoxelOceanWeatherTable.cpp) at a stormy moment.
// Checks:
// - The drawn surface (the hook at mip 0) against the query's waves (the model's Evaluate): height,
//   horizontal displacement, normal, Jacobian, at random points and references, the planet
//   turned; once the headings have settled and while they turn.
// - Continuity of the hook across chart borders, face edges and cube corners (settled, turning).
// - The query's inversion (VoxelOceanQuery::Run's Newton steps, ported) on the FFT waves, and the
//   underwater pass's waterline (VoxelOceanHeightAtFFT, ported) against it.
// - Detiling off.
// - The weather: storm seas against calm ones; no jump when a key's interval gives way to the
//   next (query and hook); the reach's steps.
// - The waves' reach (MaxHeight); the model's cost.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -IUEStandIn -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private -I. hook_check.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWavesFFT.cpp
//       ../../Source/VoxelPlugin/Private/VoxelOceanWeather.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWeatherTable.cpp -o hook_check
#include "hook_emulation.h"
#include "VoxelOceanWeather.h"
#include "VoxelOceanWeatherTable.h"

#include <chrono>
#include <cstdio>
#include <random>

using namespace VoxelShader;
using namespace HookEmulation;

static int Failures = 0;
static void Expect(bool Ok, const char* What)
{
    std::printf("  [%s] %s\n", Ok ? "ok" : "FAIL", What);
    if (!Ok) Failures++;
}

// ---------------------------------------------------------------------------------------------
// The query's inversion (VoxelOceanQuery::Run, ported): the surface point on a point's radial
// ---------------------------------------------------------------------------------------------

struct FQueryOut { double Height; FVector Surface; bool bSettled; };

static FQueryOut Query(const FVoxelOceanWavesFFT& Model, const FVoxelOceanWaveContext& Context, const FVector& Point)
{
    const double Radius = Context.Radius;
    const FVector U = Point.GetSafeNormal();
    const FVector Foot = U * Radius;
    FVoxelOceanWaveSite Site;
    Site.Point = Foot;
    Site.Up = U;
    Site.Environment = Context.Environment;
    FVoxelOceanWaveResult R;
    Model.Evaluate(Context, TConstArrayView<FVoxelOceanWaveSite>(&Site, 1), TArrayView<FVoxelOceanWaveResult>(&R, 1));
    bool bDone = false;
    for (int Step = 0; Step < 8 && !bDone; Step++)
    {
        FVector Residual = Site.Point + (R.Displacement - Site.Up * R.Height) - Foot;
        Residual -= U * (U | Residual);
        if (Residual.Size() < 1.0e-3) { bDone = true; break; }
        FVector E1 = (std::fabs(U.X) < 0.9 ? FVector(1, 0, 0) : FVector(0, 1, 0)); E1 = (E1 - U * (U | E1)).GetSafeNormal();
        const FVector E2 = U ^ E1;
        auto Apply = [&R](const FVector& V) { return FVector(R.HorizontalGradient[0] | V, R.HorizontalGradient[1] | V, R.HorizontalGradient[2] | V); };
        const FVector JE1 = Apply(E1), JE2 = Apply(E2);
        const double A11 = 1.0 + (E1 | JE1), A12 = E1 | JE2, A21 = E2 | JE1, A22 = 1.0 + (E2 | JE2);
        const double Det = std::max(A11 * A22 - A12 * A21, 0.05);
        const double R1 = E1 | Residual, R2 = E2 | Residual;
        const FVector Delta = E1 * ((A22 * R1 - A12 * R2) / Det) + E2 * ((A11 * R2 - A21 * R1) / Det);
        Site.Point = (Site.Point - Delta).GetSafeNormal() * Radius;
        Site.Up = Site.Point.GetSafeNormal();
        Model.Evaluate(Context, TConstArrayView<FVoxelOceanWaveSite>(&Site, 1), TArrayView<FVoxelOceanWaveResult>(&R, 1));
    }
    if (!bDone)
    {
        FVector Residual = Site.Point + (R.Displacement - Site.Up * R.Height) - Foot;
        Residual -= U * (U | Residual);
        bDone = Residual.Size() < 1.0e-3;
    }
    return { Point.Size() - Radius - R.Height, Site.Point + R.Displacement, bDone };
}

// ---------------------------------------------------------------------------------------------

static FVector RandomDir(std::mt19937_64& Rng)
{
    std::normal_distribution<double> G;
    return FVector(G(Rng), G(Rng), G(Rng)).GetSafeNormal();
}
static float3 F3(const FVector& V) { return float3(float(V.X), float(V.Y), float(V.Z)); }
static FVector D3(float3 V) { return FVector(V.x, V.y, V.z); }

static FVoxelOceanWaveResult EvaluateOne(const FVoxelOceanWavesFFT& Model, const FVoxelOceanWaveContext& Context, const FVector& Dir, double Radius)
{
    FVoxelOceanWaveSite Site;
    Site.Point = Dir * Radius;
    Site.Up = Dir;
    Site.Environment = Context.Environment;
    FVoxelOceanWaveResult R;
    Model.Evaluate(Context, TConstArrayView<FVoxelOceanWaveSite>(&Site, 1), TArrayView<FVoxelOceanWaveResult>(&R, 1));
    return R;
}

int main()
{
    const double Radius = 1.075e8;
    using FTablePtr = TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe>;
    using FSourcePtr = TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>;

    // ---- The weather at a stormy key ------------------------------------------------------------
    FVoxelOceanWeatherSettings WeatherSettings;
    WeatherSettings.Seed = 3;
    const auto Weather = MakeShared<FVoxelOceanProceduralWeather, ESPMode::ThreadSafe>(WeatherSettings, Radius);
    FVoxelOceanWeatherTiming Timing;
    // A key when some storm is near its peak.
    int64 K = 0;
    FVoxelOceanStorm Storm;
    {
        TArray<FVoxelOceanStorm> Storms;
        for (int64 k = 1; k < 200 && K == 0; k++)
        {
            Weather->StormsAt(double(k) * Timing.KeySeconds, Storms);
            for (const FVoxelOceanStorm& S : Storms)
                if (S.Strength > 0.75) { K = k; Storm = S; break; }
        }
    }
    const VoxelOceanCharts::FTable Charts = VoxelOceanCharts::Build(Radius);
    const FVoxelOceanWeatherKey KPrev = VoxelOceanWeatherTable::BuildKey(*Weather, Charts.N, Timing, K - 1);
    const FVoxelOceanWeatherKey KCur = VoxelOceanWeatherTable::BuildKey(*Weather, Charts.N, Timing, K);
    const FVoxelOceanWeatherKey KNext = VoxelOceanWeatherTable::BuildKey(*Weather, Charts.N, Timing, K + 1);
    const FVoxelOceanWeatherKey KAfter = VoxelOceanWeatherTable::BuildKey(*Weather, Charts.N, Timing, K + 2);

    FVoxelOceanFFTSettings Settings;
    Settings.Seed = 7;
    Settings.WindSpeed = FVoxelOceanWavesFFT::ReachWind(std::max(KCur.MaxLong, KNext.MaxLong), 0.0, false);
    Settings.bQueryShortWaves = false;
    const FVoxelOceanWavesFFT Model(Settings, Radius);
    // The table while the key's headings turn (30 s), and the same moment with them settled.
    FVoxelOceanWeatherTiming Settled = Timing;
    Settled.FadeSeconds = 1.0;
    const FTablePtr Turning = VoxelOceanWeatherTable::MakeTable(Charts, KPrev, KCur, KNext, Model.GetTableConstants(), FSourcePtr(Weather), Timing);
    const FTablePtr Calm = VoxelOceanWeatherTable::MakeTable(Charts, KPrev, KCur, KNext, Model.GetTableConstants(), FSourcePtr(Weather), Settled);
    std::printf("Weather: key %lld (%.0f s), a storm of strength %.2f at latitude %.0f; greatest wind %.1f m/s, long-wave speed %.1f m/s\n",
        (long long)K, double(K) * Timing.KeySeconds, Storm.Strength, std::asin(Storm.Centre.Z) * 57.3, Turning->MaxWind, Turning->MaxLong);
    std::printf("Model: reach wind %.0f m/s, reach %.0f cm up, %.0f cm sideways; sigma at 10 m/s %.2f m, at the reach wind %.2f m\n",
        Settings.WindSpeed, Model.MaxHeight(), Model.MaxHorizontal(), Model.SigmaAt(10.0), Model.SigmaAt(Settings.WindSpeed));

    // The context 15 s into the key (a 15 Hz step: the query's frames there are exact), and the
    // GPU's textures at the same tick.
    FVoxelOceanWaveContext Context;
    Context.Time = std::round((double(K) * Timing.KeySeconds + 15.0) * 15.0) / 15.0;
    Context.Radius = Radius;
    Context.Weather = Calm;
    FVoxelOceanWaveContext TurnContext = Context;
    TurnContext.Weather = Turning;
    std::printf("Turning: the headings %.2f of the way to the key's\n", Turning->Fade(Context.Time));
    const uint32_t Tick = VoxelOceanSpectrum::TickOf(Context.Time);
    FTextures Tx;
    Tx.Charts = &Calm->Table;
    FTextures TxTurning;
    TxTurning.Charts = &Turning->Table;
    {
        const auto T0 = std::chrono::steady_clock::now();
        for (int s = 0; s < VoxelOceanSpectrum::SlotCount; s++)
        {
            const OceanFFTEmulation::FGPUSlot G = OceanFFTEmulation::EmulateGPU(Model.GetSlots()[s], Tick);
            Tx.Displacement.Build(s, G.Displacement);
            Tx.Derivatives.Build(s, G.Derivatives);
        }
        TxTurning.Displacement = Tx.Displacement;
        TxTurning.Derivatives = Tx.Derivatives;
        std::printf("GPU passes emulated for 8 slots in %.0f ms\n", std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T0).count());
    }

    // The planet turned and placed somewhere in the world.
    const FQuat Rot = [] { const double a = 0.9, s = std::sin(a / 2); const FVector Ax = FVector(0.3, -0.8, 0.5).GetSafeNormal(); return FQuat(Ax.X * s, Ax.Y * s, Ax.Z * s, std::cos(a / 2)); }();
    const FTransform PlanetToWorld(Rot, FVector(1.0e7, -3.0e6, 2.0e5));

    // ---- The drawn surface against the query's waves ------------------------------------------
    std::printf("\nThe hook (mip 0, the long bands) against the model's Evaluate:\n");
    for (int Pass = 0; Pass < 2; Pass++)
    {
        const FVoxelOceanWaveContext& C = Pass == 0 ? Context : TurnContext;
        const FTextures& T = Pass == 0 ? Tx : TxTurning;
        std::mt19937_64 Rng(1);
        double EH = 0, ED = 0, EN = 0, EJ = 0, MH = 0, SumEH2 = 0;
        int Count = 0;
        for (int k = 0; k < 2000; k++)
        {
            // Half the points near the storm.
            FVector Dir = RandomDir(Rng);
            if (k % 2) Dir = (Storm.Centre + RandomDir(Rng) * (Storm.Radius * 1.5)).GetSafeNormal();
            const FVector Point = Dir * Radius;
            const FVector Ref = (Dir * Radius + RandomDir(Rng) * std::uniform_real_distribution<double>(0, 30000)(Rng)).GetSafeNormal() * (Radius + std::uniform_real_distribution<double>(200, 5000)(Rng));
            FVoxelOceanWaveShaderParams SP;
            Model.GetShaderParams(C, Ref, PlanetToWorld, SP);
            FParams W = ParamsOf(SP);
            W.bLongOnly = true;   // the query leaves the short bands out (decision 6)
            const FPoint G = HookEvaluate(T, F3(Rot.RotateVector(Point - Ref)), F3(Rot.RotateVector(Dir)), 0, 0, W, 3u);
            const FVoxelOceanWaveResult R = EvaluateOne(Model, C, Dir, Radius);
            const FVector HorizontalCPU = R.Displacement - Dir * R.Height;
            EH = std::max(EH, std::fabs(G.R.Height - R.Height));
            SumEH2 += (G.R.Height - R.Height) * (G.R.Height - R.Height);
            ED = std::max(ED, (D3(G.R.Displacement) - HorizontalCPU).Size());
            const double Cos = std::clamp(D3(G.R.Normal).GetSafeNormal() | R.Normal, -1.0, 1.0);
            EN = std::max(EN, std::acos(Cos) * 57.29578);
            EJ = std::max(EJ, std::fabs(G.R.Jacobian - R.Jacobian));
            MH = std::max(MH, std::fabs(R.Height));
            Count++;
        }
        std::printf("  %s: %d points: height up to %.1f mm (rms %.1f mm) of waves up to %.2f m; displacement up to %.1f mm; normal up to %.2f deg; Jacobian up to %.3f\n",
            Pass == 0 ? "settled" : "turning", Count, EH * 10, std::sqrt(SumEH2 / Count) * 10, MH / 100, ED * 10, EN, EJ);
        const double Scale = std::max(1.0, MH / 300.0);   // bilinear against cubic grows with the waves
        Expect(EH < 4.0 * Scale && ED < 4.0 * Scale, Pass == 0 ? "settled: drawn and queried waves agree within 4 cm per 3 m of waves (bilinear against cubic lookups)"
            : "turning: drawn and queried waves agree as well");
        Expect(EN < 2.0 && EJ < 0.06 * Scale, Pass == 0 ? "settled: normals within 2 degrees, Jacobians within 0.06" : "turning: normals and Jacobians as well");
    }

    // ---- Continuity ----------------------------------------------------------------------------
    std::printf("\nContinuity of the hook (all bands) across chart borders, face edges and corners:\n");
    for (int Pass = 0; Pass < 2; Pass++)
    {
        const FVoxelOceanWaveContext& C = Pass == 0 ? Context : TurnContext;
        const FTextures& T = Pass == 0 ? Tx : TxTurning;
        std::mt19937_64 Rng(2);
        double Worst[3] = {}, WorstSlope = 0;
        const char* Where[3] = { "anywhere", "face edges", "cube corners" };
        for (int k = 0; k < 6000; k++)
        {
            const int Kind = k % 3;
            FVector D = RandomDir(Rng);
            if (Kind == 1)
            {
                D = FVector(1.0, std::copysign(1.0, D.Y) * (1.0 + 2e-5 * std::uniform_real_distribution<double>(-1, 1)(Rng)), std::uniform_real_distribution<double>(-1, 1)(Rng));
                const int Perm = int(Rng() % 3);
                if (Perm == 1) D = FVector(D.Z, D.X, D.Y); else if (Perm == 2) D = FVector(D.Y, D.Z, D.X);
                if (Rng() & 1) D.X = -D.X;
                D = D.GetSafeNormal();
            }
            else if (Kind == 2)
            {
                std::uniform_real_distribution<double> U(-0.003, 0.003);
                D = FVector(((Rng() & 1) ? 1.0 : -1.0) + U(Rng), ((Rng() & 1) ? 1.0 : -1.0) + U(Rng), ((Rng() & 1) ? 1.0 : -1.0) + U(Rng)).GetSafeNormal();
            }
            const FVector P0 = D * Radius;
            const FVector Step = (D ^ RandomDir(Rng)).GetSafeNormal() * 0.1;   // 1 mm
            const FVector P1 = (P0 + Step).GetSafeNormal() * Radius;
            const FVector Ref = P0 + D * 500.0;
            FVoxelOceanWaveShaderParams SP;
            Model.GetShaderParams(C, Ref, PlanetToWorld, SP);
            const FParams W = ParamsOf(SP);
            const FPoint A = HookEvaluate(T, F3(Rot.RotateVector(P0 - Ref)), F3(Rot.RotateVector(D)), 0, 0, W, 3u);
            const FPoint B = HookEvaluate(T, F3(Rot.RotateVector(P1 - Ref)), F3(Rot.RotateVector(P1.GetSafeNormal())), 0, 0, W, 3u);
            Worst[Kind] = std::max(Worst[Kind], (double)std::fabs(A.R.Height - B.R.Height));
            WorstSlope = std::max(WorstSlope, (double)length(A.R.Gradient));
        }
        std::printf("  %s: anywhere %.3f mm, face edges %.3f mm, cube corners %.3f mm per 1 mm (the steepest slope met %.2f)\n",
            Pass == 0 ? "settled" : "turning", Worst[0] * 10, Worst[1] * 10, Worst[2] * 10, WorstSlope);
        const double Bound = std::max(0.25, 0.15 * WorstSlope);   // under 1.5 slopes
        Expect(Worst[0] < Bound && Worst[1] < Bound && Worst[2] < Bound, Pass == 0 ? "settled: no seam (every step within 1.5 times the steepest slope)"
            : "turning: no seam either");
    }

    // ---- The query's inversion and the waterline -----------------------------------------------
    std::printf("\nThe query's inversion on the FFT waves, and the underwater waterline against it:\n");
    {
        std::mt19937_64 Rng(3);
        int Settled = 0, Count = 0;
        double WorstRadial = 0, WorstWaterline = 0, WorstSteps = 0, WorstMask = 0;
        for (int k = 0; k < 400; k++)
        {
            FVector Dir = RandomDir(Rng);
            if (k % 2) Dir = (Storm.Centre + RandomDir(Rng) * (Storm.Radius * 1.5)).GetSafeNormal();
            const FVector Point = Dir * (Radius + std::uniform_real_distribution<double>(-500, 500)(Rng));
            const FQueryOut Q = Query(Model, Context, Point);
            Settled += Q.bSettled;
            Count++;
            const FVector S = Q.Surface;
            WorstRadial = std::max(WorstRadial, (S - Dir * (S | Dir)).Size());
            // The underwater pass's height of the same point (from a camera 2 m away).
            const FVector Ref = Point + RandomDir(Rng) * 200.0;
            FVoxelOceanWaveShaderParams SP;
            Model.GetShaderParams(Context, Ref, PlanetToWorld, SP);
            FParams W = ParamsOf(SP);
            W.bLongOnly = true;
            FPoint Out;
            const float H = HookHeightAt(Tx, F3(Rot.RotateVector(Point - Ref)), F3(Rot.RotateVector(-Ref)), Radius, Ref.Size() - Radius, W, Out);
            WorstWaterline = std::max(WorstWaterline, std::fabs(H - Q.Height));
            const float H8 = HookHeightAt(Tx, F3(Rot.RotateVector(Point - Ref)), F3(Rot.RotateVector(-Ref)), Radius, Ref.Size() - Radius, W, Out, 8);
            WorstSteps = std::max(WorstSteps, (double)std::fabs(H8 - H));
            const float HD = HookHeightAt(Tx, F3(Rot.RotateVector(Point - Ref)), F3(Rot.RotateVector(-Ref)), Radius, Ref.Size() - Radius, W, Out, 3, 1u);
            WorstMask = std::max(WorstMask, (double)std::fabs(HD - H));
        }
        std::printf("  %d of %d queries settled; surface points off the radial by up to %.3f mm; the waterline against the query up to %.1f mm (3 steps against 8: %.2f mm; height-only against full: %.3f mm)\n",
            Settled, Count, WorstRadial * 10, WorstWaterline * 10, WorstSteps * 10, WorstMask * 10);
        Expect(Settled == Count && WorstRadial < 0.01, "every query settles on its radial (0.1 mm)");
        Expect(WorstWaterline < 6.0, "the underwater waterline matches the query within 6 cm (storm seas included)");
        Expect(WorstSteps < 0.5, "the waterline's steps converge: 3 against 8 within 5 mm");
        Expect(WorstMask < 0.01, "the height-only search gives the full search's height");
    }

    // ---- Detiling off --------------------------------------------------------------------------
    std::printf("\nDetiling off (cell 0):\n");
    {
        FVoxelOceanFFTSettings S2 = Settings;
        S2.DetileCell = 0.0;
        const FVoxelOceanWavesFFT M2(S2, Radius);
        std::mt19937_64 Rng(4);
        double EH = 0;
        for (int k = 0; k < 500; k++)
        {
            const FVector Dir = RandomDir(Rng);
            const FVector Ref = Dir * (Radius + 1000.0);
            FVoxelOceanWaveShaderParams SP;
            M2.GetShaderParams(Context, Ref, PlanetToWorld, SP);
            FParams W = ParamsOf(SP);
            W.bLongOnly = true;
            const FPoint G = HookEvaluate(Tx, F3(Rot.RotateVector(Dir * Radius - Ref)), F3(Rot.RotateVector(Dir)), 0, 0, W, 3u);
            const FVoxelOceanWaveResult R = EvaluateOne(M2, Context, Dir, Radius);
            EH = std::max(EH, std::fabs(G.R.Height - R.Height));
        }
        std::printf("  height up to %.1f mm\n", EH * 10);
        Expect(EH < 6.0, "drawn and queried waves agree with detiling off");
    }

    // ---- The weather ----------------------------------------------------------------------------
    std::printf("\nThe weather:\n");
    {
        // Storm seas against calm: the waves' spread near the storm and far from it.
        std::mt19937_64 Rng(6);
        double Near = 0, Far = 0;
        int NearCount = 0, FarCount = 0;
        for (int k = 0; k < 3000; k++)
        {
            const bool bNear = k % 2 == 0;
            const FVector Dir = bNear ? (Storm.Centre + RandomDir(Rng) * (Storm.Radius * 0.5)).GetSafeNormal() : RandomDir(Rng);
            const double D = std::acos(std::min(1.0, Dir | Storm.Centre));
            if (!bNear && D < 4.0 * Storm.Radius) continue;
            const FVoxelOceanEnvironment E = Weather->Sample(Dir * Radius, Context.Time);
            if (!bNear && (E.WindSpeed > 8.0 || E.SwellSpeed > 8.0)) continue;   // a calm spot
            const FVoxelOceanWaveResult R = EvaluateOne(Model, Context, Dir, Radius);
            (bNear ? Near : Far) += R.Height * R.Height;
            (bNear ? NearCount : FarCount)++;
        }
        const double SNear = std::sqrt(Near / std::max(NearCount, 1)), SFar = std::sqrt(Far / std::max(FarCount, 1));
        std::printf("  the waves' rms height: %.2f m within half a storm radius, %.2f m in calm spots far away (%d and %d points)\n", SNear / 100, SFar / 100, NearCount, FarCount);
        Expect(SNear > 2.5 * SFar, "storm seas tower over calm ones");

        // A turn keeps the sea's height: settled against turning (same time, same waves), on the
        // charts that turn, across them and close to their anchors (where the turned field is
        // nearly the same field).
        {
            struct FTurned { VoxelOceanCharts::FVec Anchor; };
            std::vector<FTurned> Turned;
            const FVoxelOceanTableReader Reader = Turning->Table.Reader();
            const int N = Charts.N;
            for (int Face = 0; Face < 6; Face++)
                for (int J = 0; J < N; J++)
                    for (int I = 0; I < N; I++)
                        if (VoxelOceanChartTurned(VoxelOceanChartHeadingsAt(Reader, int2(VOXEL_OCEAN_CHART_TEXELS * I, Face * N + J))))
                        {
                            VoxelOceanCharts::FVec A, E1, E2;
                            VoxelOceanCharts::ChartFrame(Face, I, J, N, A, E1, E2);
                            Turned.push_back({ A });
                        }
            double SumSettled[2] = {}, SumTurning[2] = {};
            for (int k = 0; k < 4000 && !Turned.empty(); k++)
            {
                const int Where = k % 2;
                const VoxelOceanCharts::FVec A = Turned[Rng() % Turned.size()].Anchor;
                const FVector Anchor(A.X, A.Y, A.Z);
                const double Out = Where == 0 ? std::uniform_real_distribution<double>(0, 20000e2)(Rng) : std::uniform_real_distribution<double>(0, 300e2)(Rng);
                const FVector Dir = (Anchor + (RandomDir(Rng) ^ Anchor).GetSafeNormal() * (Out / Radius)).GetSafeNormal();
                const FVoxelOceanWaveResult S = EvaluateOne(Model, Context, Dir, Radius);
                const FVoxelOceanWaveResult T = EvaluateOne(Model, TurnContext, Dir, Radius);
                SumSettled[Where] += S.Height * S.Height;
                SumTurning[Where] += T.Height * T.Height;
            }
            const double RatioSea = std::sqrt(SumTurning[0] / std::max(SumSettled[0], 1e-9)), RatioAnchor = std::sqrt(SumTurning[1] / std::max(SumSettled[1], 1e-9));
            std::printf("  mid-turn against settled on the %d turning charts, rms height: %.3f across them, %.3f within 300 m of their anchors\n",
                (int)Turned.size(), RatioSea, RatioAnchor);
            Expect(!Turned.empty() && std::fabs(RatioSea - 1.0) < 0.06 && std::fabs(RatioAnchor - 1.0) < 0.08,
                "a turn keeps the sea's height (a group that didn't turn is looked up once; near anchors a plain average)");
        }

        // The next interval: no jump where key k's table gives way to key k + 1's.
        const FTablePtr NextTable = VoxelOceanWeatherTable::MakeTable(Charts, KCur, KNext, KAfter, Model.GetTableConstants(), FSourcePtr(Weather), Timing);
        const double Switch = double(K + 1) * Timing.KeySeconds;
        FVoxelOceanWaveContext Before = Context, After = Context;
        Before.Weather = Turning; Before.Time = Switch - 1.0e-4;
        After.Weather = NextTable; After.Time = Switch + 1.0e-4;
        double WorstQuery = 0, WorstHook = 0;
        for (int k = 0; k < 300; k++)
        {
            FVector Dir = RandomDir(Rng);
            if (k % 2) Dir = (Storm.Centre + RandomDir(Rng) * (Storm.Radius * 1.5)).GetSafeNormal();
            const FVoxelOceanWaveResult A = EvaluateOne(Model, Before, Dir, Radius);
            const FVoxelOceanWaveResult B = EvaluateOne(Model, After, Dir, Radius);
            WorstQuery = std::max(WorstQuery, (A.Displacement - B.Displacement).Size());
            // The hook over the same textures, each side's table and parameters.
            const FVector Ref = Dir * (Radius + 1000.0);
            FVoxelOceanWaveShaderParams SA, SB;
            Model.GetShaderParams(Before, Ref, PlanetToWorld, SA);
            Model.GetShaderParams(After, Ref, PlanetToWorld, SB);
            FTextures TA = Tx, TB = Tx;
            TA.Charts = &Turning->Table;
            TB.Charts = &NextTable->Table;
            const FPoint GA = HookEvaluate(TA, F3(Rot.RotateVector(Dir * Radius - Ref)), F3(Rot.RotateVector(Dir)), 0, 0, ParamsOf(SA), 3u);
            const FPoint GB = HookEvaluate(TB, F3(Rot.RotateVector(Dir * Radius - Ref)), F3(Rot.RotateVector(Dir)), 0, 0, ParamsOf(SB), 3u);
            WorstHook = std::max(WorstHook, (D3(GA.R.Displacement) - D3(GB.R.Displacement)).Size() + std::fabs(GA.R.Height - GB.R.Height));
        }
        std::printf("  across the switch to the next key's table: the query moves %.3f mm, the hook %.3f mm\n", WorstQuery * 10, WorstHook * 10);
        Expect(WorstQuery < 0.1 && WorstHook < 0.1, "no jump when the next key's table takes over (the field and the headings carry on)");

        const bool bSteps = FVoxelOceanWavesFFT::ReachWind(13.1, 0.0, false) == 14.0 && FVoxelOceanWavesFFT::ReachWind(13.0, 14.0, true) == 14.0
            && FVoxelOceanWavesFFT::ReachWind(9.9, 14.0, true) == 10.0 && FVoxelOceanWavesFFT::ReachWind(14.5, 14.0, true) == 16.0;
        Expect(bSteps, "the reach's wind covers the weather, in 2 m/s steps up and back down 4 m/s below");
    }

    // ---- Reach and cost ------------------------------------------------------------------------
    std::printf("\nReach and cost:\n");
    {
        std::mt19937_64 Rng(5);
        double Highest = 0;
        const int Count = 20000;
        std::vector<FVoxelOceanWaveSite> Sites(Count);
        std::vector<FVoxelOceanWaveResult> Results(Count);
        for (int k = 0; k < Count; k++)
        {
            FVector Dir = RandomDir(Rng);
            if (k % 2) Dir = (Storm.Centre + RandomDir(Rng) * (Storm.Radius * 0.5)).GetSafeNormal();
            Sites[k].Point = Dir * Radius; Sites[k].Up = Dir; Sites[k].Environment = Context.Environment;
        }
        const auto T0 = std::chrono::steady_clock::now();
        Model.Evaluate(Context, TConstArrayView<FVoxelOceanWaveSite>(Sites.data(), Count), TArrayView<FVoxelOceanWaveResult>(Results.data(), Count));
        const double Us = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - T0).count() / Count;
        for (const FVoxelOceanWaveResult& R : Results)
            Highest = std::max(Highest, std::fabs(R.Height));
        // New 15 Hz frames: for a point in a calm (only the states it weighs) and in the storm.
        FVoxelOceanWaveSite CalmSite, StormSite;
        for (int k = 0; k < Count; k += 2)
        {
            const FVoxelOceanEnvironment E = Weather->Sample(Sites[k].Point, Context.Time);
            if (E.WindSpeed < 4.0 && E.SwellSpeed < 4.0) { CalmSite = Sites[k]; break; }
        }
        StormSite = Sites[1];
        FVoxelOceanWaveResult Scratch;
        FVoxelOceanWaveContext Later = Context;
        Later.Time += 10.0;
        const auto T1 = std::chrono::steady_clock::now();
        Model.Evaluate(Later, TConstArrayView<FVoxelOceanWaveSite>(&CalmSite, 1), TArrayView<FVoxelOceanWaveResult>(&Scratch, 1));
        const double CalmMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T1).count();
        Later.Time += 10.0;
        const auto T2 = std::chrono::steady_clock::now();
        Model.Evaluate(Later, TConstArrayView<FVoxelOceanWaveSite>(&StormSite, 1), TArrayView<FVoxelOceanWaveResult>(&Scratch, 1));
        const double StormMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T2).count();
        std::printf("  highest of %d points (half in the storm) %.0f cm against the reach %.0f cm; Evaluate %.1f us per point (frames cached); two new 15 Hz frames %.1f ms in a calm, %.1f ms in the storm (the slots weighed only)\n",
            Count, Highest, Model.MaxHeight(), Us, CalmMs, StormMs);
        Expect(Highest < Model.MaxHeight(), "the waves stay within the model's reach");
    }

    std::printf("\n%s (%d failures)\n", Failures ? "FAILED" : "ALL OK", Failures);
    return Failures ? 1 : 0;
}
