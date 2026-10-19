// O7: the procedural weather (VoxelOceanWeather.cpp) and its table (VoxelOceanWeatherTable.cpp,
// read through VoxelOceanSeaState.ush as the hook reads it).
// - The bands: the zonal wind by latitude (easterly trades, westerlies, polar easterlies).
// - Storms: alive, drifting, cyclonic (counter-clockwise in the north), their peak wind.
// - Swell: away from the storms, outside their cores.
// - Smoothness and determinism: in space and time, the same numbers twice.
// - The table: keys pure in their index, the field's lookups against the source (in time too),
//   continuity across face edges, the headings (steps, swell steering the long bands), the
//   short-band and slope constants.
// - Cost: a key's build.
//
//   g++ -O2 -std=c++20 -ffp-contract=off -IUEStandIn -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private -I. weather_check.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWeather.cpp
//       ../../Source/VoxelPlugin/Private/VoxelOceanWeatherTable.cpp -o weather_check
#include "UEStandIn/CoreMinimal.h"
#include "VoxelOceanWeather.h"
#include "VoxelOceanWeatherTable.h"

#include <chrono>
#include <cstdio>
#include <random>

static int Failures = 0;
static void Expect(bool Ok, const char* What)
{
    std::printf("  [%s] %s\n", Ok ? "ok" : "FAIL", What);
    if (!Ok) Failures++;
}
static FVector RandomDir(std::mt19937_64& Rng)
{
    std::normal_distribution<double> G;
    return FVector(G(Rng), G(Rng), G(Rng)).GetSafeNormal();
}
static FVector FromLatLon(double LatDeg, double LonDeg)
{
    const double La = LatDeg * 3.14159265358979 / 180.0, Lo = LonDeg * 3.14159265358979 / 180.0;
    return FVector(std::cos(La) * std::cos(Lo), std::cos(La) * std::sin(Lo), std::sin(La));
}
static FVector WindVector(const FVector& Dir, const FVoxelOceanEnvironment& E)
{
    return VoxelOceanWeather::FromSpeedHeading(Dir, E.WindSpeed, E.WindHeading);
}

int main()
{
    const double Radius = 1.075e8;
    FVoxelOceanWeatherSettings Settings;
    Settings.Seed = 3;
    const auto Source = MakeShared<FVoxelOceanProceduralWeather, ESPMode::ThreadSafe>(Settings, Radius);
    std::mt19937_64 Rng(1);

    // ---- Bands ----------------------------------------------------------------------------------
    std::printf("The bands (the zonal wind's mean over longitudes, storms aside; + toward the east):\n");
    {
        double Mean[7] = {};
        const double Lats[7] = { 0, 15, 30, 45, 60, 75, -45 };
        for (int b = 0; b < 7; b++)
        {
            for (int k = 0; k < 360; k++)
            {
                const FVector Dir = FromLatLon(Lats[b], k);
                FVector East, North;
                VoxelOceanWeather::EastNorth(Dir, East, North);
                Mean[b] += (Source->BackgroundWind(Dir, 1000.0) | East) / 360.0;
            }
        }
        std::printf("  0: %.1f, 15: %.1f, 30: %.1f, 45: %.1f, 60: %.1f, 75: %.1f, -45: %.1f m/s\n", Mean[0], Mean[1], Mean[2], Mean[3], Mean[4], Mean[5], Mean[6]);
        Expect(Mean[1] < -4.0 && Mean[3] > 6.0 && Mean[5] < -2.0 && Mean[6] > 6.0, "easterly trades, westerlies in both hemispheres, polar easterlies");
        Expect(std::fabs(Mean[0]) < 2.5 && std::fabs(Mean[2]) < 2.5 && std::fabs(Mean[4]) < 2.5, "calms at the equator, 30 and 60 degrees (the noise aside)");
    }

    // ---- Storms ---------------------------------------------------------------------------------
    std::printf("\nStorms:\n");
    {
        TArray<FVoxelOceanStorm> Storms;
        double Time = 0.0;
        // A moment when some storm is near its peak.
        int Best = -1;
        for (double T = 0.0; T < 20000.0 && Best < 0; T += 60.0)
        {
            Source->StormsAt(T, Storms);
            for (int s = 0; s < Storms.Num(); s++)
                if (Storms[s].Strength > 0.75) { Best = s; Time = T; break; }
        }
        Expect(Best >= 0, "a storm reaches most of its strength");
        if (Best >= 0)
        {
            const FVoxelOceanStorm S = Storms[Best];
            std::printf("  storm %d at %.0f s: strength %.2f, radius %.0f km, centre at latitude %.1f\n", Best, Time, S.Strength, S.Radius * Radius * 1e-5,
                std::asin(S.Centre.Z) * 180.0 / 3.14159265);
            // Around its centre: the strongest wind and its turn.
            FVector East, North;
            VoxelOceanWeather::EastNorth(S.Centre, East, North);
            double Peak = 0, Turn = 0;
            int Count = 0;
            for (int a = 0; a < 36; a++)
            {
                const double Ang = a * 3.14159265 / 18.0;
                for (double R = 0.1; R <= 1.0; R += 0.05)
                {
                    const FVector Off = (East * std::cos(Ang) + North * std::sin(Ang)) * (R * S.Radius);
                    const FVector P = (S.Centre + Off).GetSafeNormal();
                    const FVoxelOceanEnvironment E = Source->SampleWeather(P * Radius, Time);
                    Peak = std::max(Peak, E.WindSpeed);
                    if (std::fabs(R - 0.35) < 0.03)
                    {
                        // The wind's sense around the centre: (centre x point) . wind > 0 is counter-clockwise.
                        Turn += ((S.Centre ^ P) | WindVector(P, E)) > 0 ? 1 : -1;
                        Count++;
                    }
                }
            }
            const double Sense = Turn / std::max(Count, 1) * (S.Centre.Z >= 0 ? 1.0 : -1.0);
            std::printf("  its peak wind %.1f m/s; turning the right way at %.0f%% of the points a third of its radius out\n", Peak, 50.0 * (Sense + 1.0));
            Expect(Peak > 0.7 * Settings.StormWind * S.Strength && Peak < 1.6 * Settings.StormWind + 15.0, "its wind peaks near StormWind times its strength");
            Expect(Sense > 0.9, "cyclonic: counter-clockwise in the north, clockwise in the south");
            const double StormValue = Source->SampleWeather(S.Centre * Radius, Time).Storm;
            Expect(StormValue > 0.6 * S.Strength, "its storm value at the centre follows its strength");

            // Drift: the centre moves, smoothly, over minutes.
            TArray<FVoxelOceanStorm> Later;
            Source->StormsAt(Time + 600.0, Later);
            const double Moved = std::acos(std::min(1.0, Later[Best].Centre | S.Centre)) * Radius * 1e-5;
            std::printf("  in 10 minutes it drifts %.2f km\n", Moved);
            Expect(Moved > 0.05 && Moved < 20.0, "storms drift (km per 10 minutes, not jumps)");

            // Swell outside it points away.
            int Away = 0, Total = 0;
            for (int a = 0; a < 36; a++)
            {
                const double Ang = a * 3.14159265 / 18.0;
                const FVector P = (S.Centre + (East * std::cos(Ang) + North * std::sin(Ang)) * (2.5 * S.Radius)).GetSafeNormal();
                const FVoxelOceanEnvironment E = Source->SampleWeather(P * Radius, Time);
                const FVector Swell = VoxelOceanWeather::FromSpeedHeading(P, E.SwellSpeed, E.SwellHeading);
                const FVector Out = ((S.Centre ^ P) ^ P).GetSafeNormal();
                Away += (Swell | Out) > 0 ? 1 : 0;
                Total++;
            }
            std::printf("  swell 2.5 radii out runs away from it at %d of %d points\n", Away, Total);
            Expect(Away >= Total * 3 / 4, "swell runs away from the storm");
        }
    }

    // ---- Smoothness and determinism ---------------------------------------------------------------
    std::printf("\nSmoothness and determinism:\n");
    {
        double WorstSpace = 0, WorstTime = 0, Diff = 0;
        for (int k = 0; k < 2000; k++)
        {
            const FVector Dir = RandomDir(Rng);
            const double T = std::uniform_real_distribution<double>(0, 1e5)(Rng);
            const FVoxelOceanEnvironment A = Source->SampleWeather(Dir * Radius, T);
            const FVector Step = ((Dir ^ RandomDir(Rng)).GetSafeNormal()) * (1000.0e2 / Radius);   // 1 km
            const FVoxelOceanEnvironment B = Source->SampleWeather((Dir + Step).GetSafeNormal() * Radius, T);
            const FVoxelOceanEnvironment C = Source->SampleWeather(Dir * Radius, T + 10.0);
            const FVoxelOceanEnvironment D = Source->SampleWeather(Dir * Radius, T);
            WorstSpace = std::max(WorstSpace, (WindVector(Dir, A) - WindVector(Dir, B)).Size());
            WorstTime = std::max(WorstTime, (WindVector(Dir, A) - WindVector(Dir, C)).Size());
            Diff = std::max(Diff, std::fabs(A.WindSpeed - D.WindSpeed) + std::fabs(A.SwellSpeed - D.SwellSpeed) + std::fabs(A.Storm - D.Storm));
        }
        std::printf("  the wind changes by up to %.3f m/s over 1 km and %.3f m/s over 10 s; repeated samples differ by %.1e\n", WorstSpace, WorstTime, Diff);
        Expect(WorstSpace < 1.0 && WorstTime < 0.5, "smooth in space and time");
        Expect(Diff == 0.0, "the same inputs give the same weather");
    }

    // ---- The table ------------------------------------------------------------------------------
    std::printf("\nThe table:\n");
    const VoxelOceanCharts::FTable Charts = VoxelOceanCharts::Build(Radius);
    FVoxelOceanWeatherTiming Timing;
    const int64 K = 40;
    const auto T0 = std::chrono::steady_clock::now();
    const FVoxelOceanWeatherKey KPrev = VoxelOceanWeatherTable::BuildKey(*Source, Charts.N, Timing, K - 1);
    const double BuildMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - T0).count();
    const FVoxelOceanWeatherKey KCur = VoxelOceanWeatherTable::BuildKey(*Source, Charts.N, Timing, K);
    const FVoxelOceanWeatherKey KNext = VoxelOceanWeatherTable::BuildKey(*Source, Charts.N, Timing, K + 1);
    const FVoxelOceanWeatherKey KAgain = VoxelOceanWeatherTable::BuildKey(*Source, Charts.N, Timing, K);
    Expect(KAgain.Field == KCur.Field && KAgain.Headings == KCur.Headings, "a key is a pure function of its index");
    FVoxelOceanTableConstants Constants;
    for (int i = 0; i < VOXEL_OCEAN_TABLE_CONSTANTS * 4; i++) Constants.Texels[i] = 0.1f * float(i + 1);
    const auto Table = VoxelOceanWeatherTable::MakeTable(Charts, KPrev, KCur, KNext, Constants,
        TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>(Source), Timing);
    std::printf("  a key builds in %.0f ms (one thread; %d field cells, %d charts); greatest wind %.1f m/s, long-wave speed %.1f m/s\n",
        BuildMs, 6 * VOXEL_OCEAN_FIELD_SIDE * VOXEL_OCEAN_FIELD_SIDE, 6 * Charts.N * Charts.N, Table->MaxWind, Table->MaxLong);
    {
        const VoxelShader::FVoxelOceanTableReader Reader = Table->Table.Reader();
        using VoxelShader::float3;
        // The field's lookups against the source, at the keys and between them.
        double WorstKey = 0, WorstMid = 0, Scale = 0, SumKey = 0;
        for (int k = 0; k < 2000; k++)
        {
            const FVector Dir = RandomDir(Rng);
            const float3 D((float)Dir.X, (float)Dir.Y, (float)Dir.Z);
            const double TK = double(K) * Timing.KeySeconds;
            const VoxelShader::float2 F0 = VoxelShader::VoxelOceanFieldAt(Reader, D, Charts.N, 0.0f);
            const VoxelShader::float2 FH = VoxelShader::VoxelOceanFieldAt(Reader, D, Charts.N, 0.5f);
            const FVoxelOceanEnvironment E0 = Source->SampleWeather(Dir * Radius, TK);
            const FVoxelOceanEnvironment E1 = Source->SampleWeather(Dir * Radius, TK + Timing.KeySeconds);
            const FVoxelOceanEnvironment EH = Source->SampleWeather(Dir * Radius, TK + 0.5 * Timing.KeySeconds);
            WorstKey = std::max(WorstKey, std::fabs(F0.x - E0.WindSpeed));
            SumKey += (F0.x - E0.WindSpeed) * (F0.x - E0.WindSpeed);
            WorstMid = std::max(WorstMid, std::fabs(FH.x - EH.WindSpeed));
            Scale = std::max(Scale, std::fabs(E1.WindSpeed - E0.WindSpeed));
        }
        std::printf("  the field's wind against the source: up to %.2f m/s at a key (rms %.2f; bilinear over %.0f km cells, the worst in storm cores), %.2f m/s halfway (keys %.2f m/s apart at most)\n",
            WorstKey, std::sqrt(SumKey / 2000.0), 1.5707963 * Radius * 1e-5 / VOXEL_OCEAN_FIELD_CELLS, WorstMid, Scale);
        Expect(WorstKey < 3.0 && std::sqrt(SumKey / 2000.0) < 0.3 && WorstMid < 3.0, "the field follows the source (within its cells' and keys' interpolation: 3 m/s in storm cores, rms 0.3)");

        // Across face edges: a walk over every edge of face 0's neighbours.
        double WorstStep = 0;
        for (int k = 0; k < 4000; k++)
        {
            const FVector Dir = RandomDir(Rng);
            const FVector Next = (Dir + ((Dir ^ RandomDir(Rng)).GetSafeNormal()) * (100.0e2 / Radius)).GetSafeNormal();   // 100 m
            const VoxelShader::float2 A = VoxelShader::VoxelOceanFieldAt(Reader, float3((float)Dir.X, (float)Dir.Y, (float)Dir.Z), Charts.N, 0.3f);
            const VoxelShader::float2 B = VoxelShader::VoxelOceanFieldAt(Reader, float3((float)Next.X, (float)Next.Y, (float)Next.Z), Charts.N, 0.3f);
            WorstStep = std::max(WorstStep, (double)std::fabs(A.x - B.x) + std::fabs(A.y - B.y));
        }
        // Points straddling the face edges.
        double WorstEdge = 0;
        for (int k = 0; k < 4000; k++)
        {
            FVector Dir = RandomDir(Rng);
            // Pushed onto an edge: two components equal in magnitude.
            const int A = k % 3, B = (k + 1) % 3;
            Dir[B] = (Dir[B] >= 0 ? 1.0 : -1.0) * std::fabs(Dir[A]);
            Dir = Dir.GetSafeNormal();
            FVector Side = FVector(0, 0, 0);
            Side[A] = Dir[A] >= 0 ? 1.0 : -1.0;
            Side[B] = Dir[B] >= 0 ? -1.0 : 1.0;
            const FVector P0 = (Dir + Side * (10.0e2 / Radius)).GetSafeNormal(), P1 = (Dir - Side * (10.0e2 / Radius)).GetSafeNormal();
            const VoxelShader::float2 V0 = VoxelShader::VoxelOceanFieldAt(Reader, float3((float)P0.X, (float)P0.Y, (float)P0.Z), Charts.N, 0.3f);
            const VoxelShader::float2 V1 = VoxelShader::VoxelOceanFieldAt(Reader, float3((float)P1.X, (float)P1.Y, (float)P1.Z), Charts.N, 0.3f);
            WorstEdge = std::max(WorstEdge, (double)std::fabs(V0.x - V1.x) + std::fabs(V0.y - V1.y));
        }
        std::printf("  the field changes by up to %.3f m/s over 100 m anywhere, %.3f m/s across a face edge (20 m)\n", WorstStep, WorstEdge);
        Expect(WorstStep < 0.3 && WorstEdge < 0.2, "the field is continuous, face edges included");

        // The headings: whole steps, swell steering the long bands, prev and cur from their keys.
        int Swung = 0, OffStep = 0, Turned = 0, SwellLed = 0;
        const double Step = Timing.HeadingStepDegrees * 3.14159265358979 / 180.0;
        for (int Face = 0; Face < 6; Face++)
            for (int J = 0; J < Charts.N; J++)
                for (int I = 0; I < Charts.N; I++)
                {
                    const VoxelShader::FVoxelOceanChartHeadings H = VoxelShader::VoxelOceanChartHeadingsAt(Reader, VoxelShader::int2(VOXEL_OCEAN_CHART_TEXELS * I, Face * Charts.N + J));
                    const double A = std::atan2(H.LongCurrent.y, H.LongCurrent.x) / Step;
                    OffStep += std::fabs(A - std::round(A)) > 1e-4 ? 1 : 0;
                    Turned += VoxelShader::VoxelOceanChartTurned(H) ? 1 : 0;
                    SwellLed += (H.LongCurrent.x != H.ShortCurrent.x || H.LongCurrent.y != H.ShortCurrent.y) ? 1 : 0;
                    Swung++;
                }
        std::printf("  %d charts: %d turn between these keys (5 minutes), %d have their long waves led apart by swell; %d off a whole step\n", Swung, Turned, SwellLed, OffStep);
        Expect(OffStep == 0, "headings are whole steps");
        Expect(Turned < Swung / 2, "most charts keep their headings from one key to the next");
        Expect(SwellLed > 0, "swell steers some charts' long waves");

        // The constants as written.
        const VoxelShader::float4 C2 = Reader.Load(VoxelShader::VoxelOceanConstantsX(Charts.N), 2);
        Expect(std::fabs(C2.x - 0.9f) < 1e-6f, "the constants sit where the shaders read them");
    }

    // ---- Timing ---------------------------------------------------------------------------------
    std::printf("\nThe interval's clock:\n");
    {
        const double TK = double(K) * Timing.KeySeconds;
        const bool bOk = std::fabs(Table->Alpha(TK) - 0.0) < 1e-9 && std::fabs(Table->Alpha(TK + 150.0) - 0.5) < 1e-9 && Table->Fade(TK) == 0.0
            && std::fabs(Table->Fade(TK + 15.0) - 0.5) < 1e-9 && Table->Fade(TK + 40.0) == 1.0
            && VoxelOceanWeatherTable::KeyIndexAt(*Source, Timing, TK + 10.0) == K;
        Expect(bOk, "Alpha runs the key's interval, Fade its first 30 s, keys found by time");
    }

    std::printf("\n%s (%d failures)\n", Failures ? "FAILED" : "ALL OK", Failures);
    return Failures ? 1 : 0;
}
