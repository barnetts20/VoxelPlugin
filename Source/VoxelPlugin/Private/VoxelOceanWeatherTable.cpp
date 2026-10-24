#include "VoxelOceanWeatherTable.h"
#include "VoxelOceanWeather.h"
#include "Async/ParallelFor.h"

#include <atomic>
#include <cmath>

namespace VoxelOceanWeatherTableDetail
{
    using VoxelOceanCharts::FVec;

    constexpr int32 Side = VOXEL_OCEAN_FIELD_SIDE;
    constexpr int32 Cells = VOXEL_OCEAN_FIELD_CELLS;

    /** A field cell's direction (i, j from -1 to Cells: the border past the face's edges). */
    FVec FieldDirection(int32 Face, int32 I, int32 J)
    {
        using namespace VoxelOceanCharts;
        const FVec N0 = FaceAxis(Face, 0), U0 = FaceAxis(Face, 1), V0 = FaceAxis(Face, 2);
        const double S = -1.0 + (2.0 * I + 1.0) / Cells, T = -1.0 + (2.0 * J + 1.0) / Cells;
        return Normalize(Add(N0, Add(Scale(U0, std::tan(S * 0.78539816339744831)), Scale(V0, std::tan(T * 0.78539816339744831)))));
    }

    FVector ToVector(const FVec& V) { return FVector(V.X, V.Y, V.Z); }

    /** A tangent vector given by speed and heading at Dir, in a chart's plane: its angle from E1
     *  turned to whole steps, as (cos, sin). */
    void Heading(const FVector& Dir, double Speed, double HeadingRadians, const FVec& E1, const FVec& E2, double Step, float& OutX, float& OutY)
    {
        const FVector V = VoxelOceanWeather::FromSpeedHeading(Dir, FMath::Max(Speed, 1.0e-3), HeadingRadians);
        double Angle = std::atan2(V.X * E2.X + V.Y * E2.Y + V.Z * E2.Z, V.X * E1.X + V.Y * E1.Y + V.Z * E1.Z);
        if (Step > 0.0) Angle = std::floor(Angle / Step + 0.5) * Step;
        OutX = (float)std::cos(Angle);
        OutY = (float)std::sin(Angle);
    }

    double SmoothStep(double X)
    {
        const double T = FMath::Clamp(X, 0.0, 1.0);
        return T * T * (3.0 - 2.0 * T);
    }
}

namespace VWT = VoxelOceanWeatherTableDetail;

double FVoxelOceanWeatherTable::Alpha(double OceanTime) const
{
    const double Key = FMath::Max(Timing.KeySeconds, 1.0);
    return FMath::Clamp((WeatherTime(OceanTime) - double(Index) * Key) / Key, 0.0, 1.0);
}

double FVoxelOceanWeatherTable::Fade(double OceanTime) const
{
    const double Key = FMath::Max(Timing.KeySeconds, 1.0);
    // A turn ends within its key (the next key's starts from this one's headings); a weather
    // clock that stands still has no turn under way.
    const double FadeSeconds = FMath::Min(Timing.FadeSeconds, Key);
    if (FadeSeconds <= 0.0 || WeatherTime(OceanTime + 1.0) == WeatherTime(OceanTime)) return 1.0;
    const double Since = WeatherTime(OceanTime) - double(Index) * Key;
    return VWT::SmoothStep(Since / FadeSeconds);
}

int64 VoxelOceanWeatherTable::KeyIndexAt(const IVoxelOceanEnvironmentSource& Source, const FVoxelOceanWeatherTiming& Timing, double OceanTime)
{
    return (int64)std::floor(Source.WeatherTime(OceanTime) / FMath::Max(Timing.KeySeconds, 1.0));
}

FVoxelOceanWeatherKey VoxelOceanWeatherTable::BuildKey(const IVoxelOceanEnvironmentSource& Source, int32 ChartCells,
    const FVoxelOceanWeatherTiming& Timing, int64 Index)
{
    using VoxelOceanCharts::FVec;
    FVoxelOceanWeatherKey Key;
    Key.Index = Index;
    const int32 N = FMath::Max(ChartCells, 1);
    const double Time = double(Index) * FMath::Max(Timing.KeySeconds, 1.0);
    const double Step = FMath::Max(Timing.HeadingStepDegrees, 0.0) * 3.14159265358979323846 / 180.0;

    // Every point the key samples: the field's cells, then the charts' anchors.
    const int32 FieldPoints = 6 * VWT::Side * VWT::Side;
    const int32 ChartPoints = 6 * N * N;
    TArray<FVector> Points;
    Points.SetNum(FieldPoints + ChartPoints);
    for (int32 Face = 0; Face < 6; Face++)
        for (int32 J = 0; J < VWT::Side; J++)
            for (int32 I = 0; I < VWT::Side; I++)
                Points[(Face * VWT::Side + J) * VWT::Side + I] = VWT::ToVector(VWT::FieldDirection(Face, I - 1, J - 1));
    for (int32 Face = 0; Face < 6; Face++)
        for (int32 J = 0; J < N; J++)
            for (int32 I = 0; I < N; I++)
            {
                FVec Dir, E1, E2;
                VoxelOceanCharts::ChartFrame(Face, I, J, N, Dir, E1, E2);
                Points[FieldPoints + (Face * N + J) * N + I] = VWT::ToVector(Dir);
            }

    // Sampled in chunks (a source shares work within a batch).
    TArray<FVoxelOceanEnvironment> Samples;
    Samples.SetNum(Points.Num());
    constexpr int32 Chunk = 512;
    const int32 Chunks = (Points.Num() + Chunk - 1) / Chunk;
    ParallelFor(Chunks, [&Source, &Points, &Samples, Time](int32 C)
        {
            const int32 First = C * Chunk;
            const int32 Count = FMath::Min(Chunk, Points.Num() - First);
            Source.SampleWeatherBatch(TConstArrayView<FVector>(Points.GetData() + First, Count), Time,
                TArrayView<FVoxelOceanEnvironment>(Samples.GetData() + First, Count));
        });

    Key.Field.assign(size_t(FieldPoints) * 2, 0.0f);
    for (int32 P = 0; P < FieldPoints; P++)
    {
        const FVoxelOceanEnvironment& E = Samples[P];
        Key.Field[size_t(P) * 2] = (float)FMath::Max(E.WindSpeed, 0.0);
        Key.Field[size_t(P) * 2 + 1] = (float)FMath::Max(E.SwellSpeed, 0.0);
        Key.MaxWind = FMath::Max(Key.MaxWind, E.WindSpeed);
        Key.MaxLong = FMath::Max(Key.MaxLong, FMath::Max(E.WindSpeed, E.SwellSpeed));
    }
    Key.Headings.assign(size_t(ChartPoints) * 4, 0.0f);
    for (int32 Face = 0; Face < 6; Face++)
        for (int32 J = 0; J < N; J++)
            for (int32 I = 0; I < N; I++)
            {
                const int32 C = (Face * N + J) * N + I;
                FVec Dir, E1, E2;
                VoxelOceanCharts::ChartFrame(Face, I, J, N, Dir, E1, E2);
                const FVector D = VWT::ToVector(Dir);
                const FVoxelOceanEnvironment& E = Samples[FieldPoints + C];
                float* H = &Key.Headings[size_t(C) * 4];
                if (E.SwellSpeed > E.WindSpeed)
                    VWT::Heading(D, E.SwellSpeed, E.SwellHeading, E1, E2, Step, H[0], H[1]);
                else
                    VWT::Heading(D, E.WindSpeed, E.WindHeading, E1, E2, Step, H[0], H[1]);
                VWT::Heading(D, E.WindSpeed, E.WindHeading, E1, E2, Step, H[2], H[3]);
            }
    return Key;
}

TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe> VoxelOceanWeatherTable::MakeTable(const VoxelOceanCharts::FTable& Charts,
    const FVoxelOceanWeatherKey& Previous, const FVoxelOceanWeatherKey& Current, const FVoxelOceanWeatherKey& Next,
    const FVoxelOceanTableConstants& Constants, const TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe>& Source,
    const FVoxelOceanWeatherTiming& Timing)
{
    TSharedPtr<FVoxelOceanWeatherTable, ESPMode::ThreadSafe> Out = MakeShared<FVoxelOceanWeatherTable, ESPMode::ThreadSafe>();
    Out->Table = Charts;
    Out->Source = Source;
    Out->Timing = Timing;
    Out->Index = Current.Index;
    static std::atomic<uint64> NextSerial { 1 };
    Out->Serial = NextSerial.fetch_add(1);
    Out->MaxWind = FMath::Max(Current.MaxWind, Next.MaxWind);
    Out->MaxLong = FMath::Max(Current.MaxLong, Next.MaxLong);
    VoxelOceanCharts::FTable& T = Out->Table;
    const int32 N = T.N;
    // The headings.
    for (int32 Face = 0; Face < 6; Face++)
        for (int32 J = 0; J < N; J++)
            for (int32 I = 0; I < N; I++)
            {
                const size_t C = size_t((Face * N + J) * N + I) * 4;
                float* P = T.Texel(VOXEL_OCEAN_CHART_TEXELS * I + 3, Face * N + J);
                for (int32 k = 0; k < 4; k++)
                {
                    P[k] = Previous.Headings[C + k];
                    P[4 + k] = Current.Headings[C + k];
                }
            }
    // The field: key k's in xy, key k + 1's in zw.
    const int32 X0 = VoxelShader::VoxelOceanFieldX(N);
    for (int32 Face = 0; Face < 6; Face++)
        for (int32 J = 0; J < VWT::Side; J++)
            for (int32 I = 0; I < VWT::Side; I++)
            {
                const size_t F = size_t((Face * VWT::Side + J) * VWT::Side + I) * 2;
                float* P = T.Texel(X0 + I, Face * VWT::Side + J);
                P[0] = Current.Field[F];
                P[1] = Current.Field[F + 1];
                P[2] = Next.Field[F];
                P[3] = Next.Field[F + 1];
            }
    // The constants.
    const int32 CX = VoxelShader::VoxelOceanConstantsX(N);
    for (int32 Row = 0; Row < VOXEL_OCEAN_TABLE_CONSTANTS; Row++)
    {
        float* P = T.Texel(CX, Row);
        for (int32 k = 0; k < 4; k++) P[k] = Constants.Texels[Row * 4 + k];
    }
    // Stage W6b: the resolved shares.
    for (int32 Row = 0; Row < VOXEL_OCEAN_TABLE_RESOLVED_TEXELS; Row++)
    {
        float* P = T.Texel(CX, VOXEL_OCEAN_TABLE_RESOLVED_ROW + Row);
        for (int32 k = 0; k < 4; k++) P[k] = Constants.Resolved[Row * 4 + k];
    }
    return Out;
}
