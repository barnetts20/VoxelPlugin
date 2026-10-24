#include "VoxelOceanWaves.h"
#include "FDensitySampleCompositor.h"
#include "HAL/IConsoleManager.h"

static TAutoConsoleVariable<int32> CVarVoxelOceanWaveModel(
    TEXT("voxel.Ocean.WaveModel"), 2,
    TEXT("The ocean's waves: 0 a flat sea; 1 the test model (three Gerstner waves: voxel.Ocean.TestWaveHeight, TestWaveLength, TestWaveChop), to check the surface, the underwater view and the queries under displacement; 2 (default) the FFT cascades (stage O6b; the ocean settings' Waves category)."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelOceanTestWaveHeight(
    TEXT("voxel.Ocean.TestWaveHeight"), 100.0f,
    TEXT("Amplitude of each of the test model's three waves (cm)."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelOceanTestWaveLength(
    TEXT("voxel.Ocean.TestWaveLength"), 3000.0f,
    TEXT("Wavelength of the test model's longest wave (cm); the others are 0.61 and 0.37 of it."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelOceanTestWaveChop(
    TEXT("voxel.Ocean.TestWaveChop"), 0.5f,
    TEXT("Choppiness of the test model: horizontal displacement per vertical amplitude, 0 (sines) to 1 (sharp Gerstner crests). Clamped so the surface never folds."),
    ECVF_Default);

namespace VoxelOceanWavesDetail
{
    constexpr double Gravity = 980.665;            // cm/s^2, deep-water dispersion
    /** The test model's fixed planet directions and wavelengths relative to the longest. */
    const FVector TestDirections[3] = { FVector(0.97, 0.29, -0.19), FVector(-0.36, 0.89, 0.27), FVector(0.18, -0.53, 0.83) };
    constexpr double TestScales[3] = { 1.0, 0.61, 0.37 };
    /** The most the horizontal displacement may compress the surface (sum of C A |k|). */
    constexpr double MaxCompression = 0.95;
    /** The query's inversion: steps, tolerance (cm), and the least determinant a Newton step
     *  divides by (as VoxelOceanHeightAt). */
    constexpr int32 QuerySteps = 8;
    constexpr double QueryTolerance = 1.0e-3;
    constexpr double QueryMinJacobian = 0.05;
    /** The bed's slope (stage O9b2; VoxelOceanPage.usf's VOXEL_OCEAN_BED_*): measured where the
     *  depth lies within these (cm), over at least this step either side (cm). */
    constexpr double BedLand = -10000.0;
    constexpr double BedSea = 6000.0;
    constexpr double BedMinStep = 100.0;
    constexpr double BedMaxFootprint = 100000.0;
    /** Stage W6h: the terrain's own depth for the felt depth (VoxelOceanSurfFeltDepth): sampled where
     *  the depth the waves feel lies under this (cm: within the felt depth's blend, 3 m, and on the
     *  land side), at this footprint (cm: the wave clipmap's finer levels' texels, whose static
     *  layer holds it for the drawn waves). */
    constexpr double FeltBelow = 300.0;
    constexpr double FeltFootprint = 100.0;
}

namespace VOW = VoxelOceanWavesDetail;

// ---------------------------------------------------------------------------
// Test model
// ---------------------------------------------------------------------------

int32 VoxelOceanWaves::ChosenModel()
{
    return FMath::Clamp(CVarVoxelOceanWaveModel.GetValueOnGameThread(), 0, 2);
}

FVoxelOceanWavesTest::FSettings FVoxelOceanWavesTest::ReadConsole()
{
    FSettings Settings;
    if (VoxelOceanWaves::ChosenModel() != 1) return Settings;
    Settings.Amplitude = FMath::Max(0.0, (double)CVarVoxelOceanTestWaveHeight.GetValueOnGameThread());
    Settings.Wavelength = FMath::Max(10.0, (double)CVarVoxelOceanTestWaveLength.GetValueOnGameThread());
    Settings.Choppiness = FMath::Clamp((double)CVarVoxelOceanTestWaveChop.GetValueOnGameThread(), 0.0, 1.0);
    return Settings;
}

FVoxelOceanWavesTest::FVoxelOceanWavesTest(const FSettings& InSettings)
    : Settings(InSettings)
{
    double SumK = 0.0;
    for (int32 w = 0; w < 3; w++)
    {
        const double Length = FMath::Max(Settings.Wavelength, 10.0) * VOW::TestScales[w];
        const double Magnitude = 2.0 * UE_DOUBLE_PI / Length;
        K[w] = VOW::TestDirections[w].GetSafeNormal() * Magnitude;
        Omega[w] = FMath::Sqrt(VOW::Gravity * Magnitude);
        SumK += Magnitude;
    }
    // No fold: the horizontal displacement's gradient stays below MaxCompression everywhere.
    const double Limit = Settings.Amplitude > 0.0 ? VOW::MaxCompression / (Settings.Amplitude * SumK) : 0.0;
    Chop = FMath::Clamp(FMath::Min(Settings.Choppiness, Limit), 0.0, 1.0);
}

void FVoxelOceanWavesTest::Evaluate(const FVoxelOceanWaveContext& Context, TConstArrayView<FVoxelOceanWaveSite> Sites,
    TArrayView<FVoxelOceanWaveResult> Out) const
{
    const double A = Settings.Amplitude;
    for (int32 i = 0; i < Sites.Num(); i++)
    {
        const FVoxelOceanWaveSite& Site = Sites[i];
        FVoxelOceanWaveResult& R = Out[i];
        R = FVoxelOceanWaveResult();
        const FVector& U = Site.Up;
        R.Normal = U;
        if (A <= 0.0) continue;

        double Height = 0.0;
        FVector Horizontal = FVector::ZeroVector;
        FVector G = FVector::ZeroVector;
        FVector T[3];
        double Ws[3];
        for (int32 w = 0; w < 3; w++)
        {
            const FVector Kt3 = K[w] - U * (U | K[w]);
            const double Kt = Kt3.Size();
            T[w] = Kt > 1.0e-12 ? Kt3 / Kt : FVector::ZeroVector;
            const double Theta = (K[w] | Site.Point) + Omega[w] * Context.Time;
            const double S = FMath::Sin(Theta);
            const double C = FMath::Cos(Theta);
            Height += A * S;
            Horizontal += T[w] * (Chop * A * C);
            G += Kt3 * (A * C);
            Ws[w] = Chop * A * Kt * S;
            // d theta / dt = omega.
            R.Velocity += U * (A * Omega[w] * C) - T[w] * (Chop * A * Omega[w] * S);
            // dD_h / dx = -Ws T T^T (tangent plane).
            for (int32 b = 0; b < 3; b++)
                R.HorizontalGradient[b] -= T[w] * (Ws[w] * T[w][b]);
        }
        R.Height = Height;
        R.Displacement = U * Height + Horizontal;

        // As VoxelOceanWaves.ush: N = Up det(M) - ((1 - tr S) g + S g), M = I - S.
        const double Trace = Ws[0] + Ws[1] + Ws[2];
        double TraceSq = 0.0;
        for (int32 a = 0; a < 3; a++)
        {
            for (int32 b = 0; b < 3; b++)
            {
                const double D = T[a] | T[b];
                TraceSq += Ws[a] * Ws[b] * D * D;
            }
        }
        R.Jacobian = 1.0 - Trace + 0.5 * (Trace * Trace - TraceSq);
        const FVector SG = T[0] * (Ws[0] * (T[0] | G)) + T[1] * (Ws[1] * (T[1] | G)) + T[2] * (Ws[2] * (T[2] | G));
        const FVector N = U * R.Jacobian - ((1.0 - Trace) * G + SG);
        R.Normal = (R.Jacobian > 0.0 ? N : N + U * (2.0 * FMath::Abs(R.Jacobian) + 1.0e-3)).GetSafeNormal();
    }
}

void FVoxelOceanWavesTest::GetShaderParams(const FVoxelOceanWaveContext& Context, const FVector& Reference,
    const FTransform& PlanetToWorld, FVoxelOceanWaveShaderParams& Out) const
{
    Out = FVoxelOceanWaveShaderParams();
    Out.Shape = FVector4f((float)Chop, (float)Context.Environment.Storm, (float)Context.Environment.WindSpeed,
        (float)Context.Environment.WindHeading);
    if (Settings.Amplitude <= 0.0) return;
    Out.Amplitude = (float)Settings.Amplitude;
    for (int32 w = 0; w < 3; w++)
    {
        // k . (point - reference) is small and exact near the reference; the phase there carries
        // the rest, formed and wrapped in double.
        const double Phase = FMath::Fmod((K[w] | Reference) + Omega[w] * Context.Time, 2.0 * UE_DOUBLE_PI);
        const FVector KWorld = PlanetToWorld.TransformVectorNoScale(K[w]);
        Out.Wave[w] = FVector4f((float)KWorld.X, (float)KWorld.Y, (float)KWorld.Z, (float)Phase);
    }
}

// ---------------------------------------------------------------------------
// Frame
// ---------------------------------------------------------------------------

void FVoxelOceanWaveFrame::ShaderParams(const FVector& Reference, FVoxelOceanWaveShaderParams& Out) const
{
    Out = FVoxelOceanWaveShaderParams();
    Out.Shape = FVector4f(0.0f, (float)Context.Environment.Storm, (float)Context.Environment.WindSpeed,
        (float)Context.Environment.WindHeading);
    if (Model.IsValid())
        Model->GetShaderParams(Context, Reference, PlanetToWorld, Out);
}

void FVoxelOceanWaveFrame::FillSites(TArrayView<FVoxelOceanWaveSite> Sites, double Time) const
{
    const EVoxelOceanWaveInputs Required = Model.IsValid() ? Model->RequiredInputs() : EVoxelOceanWaveInputs::None;
    for (FVoxelOceanWaveSite& Site : Sites)
    {
        Site.Up = Site.Point.GetSafeNormal();
        Site.Depth = VoxelOceanWaves::Deep;
        Site.BedSlope = VoxelOceanWaves::BedUnknown;
        Site.BedRise = FVector::ZeroVector;
        Site.BedDepth = VoxelOceanWaves::Deep;
        Site.Shore = FVoxelOceanShore();
        Site.Environment = Context.Environment;
        if (EnumHasAnyFlags(Required, EVoxelOceanWaveInputs::Shore) && Shore.IsValid())
            Site.Shore = Shore->Sample(Site.Point);
        if (EnumHasAnyFlags(Required, EVoxelOceanWaveInputs::Environment) && Environment.IsValid())
            Site.Environment = Environment->Sample(Site.Point, Time);
    }
    if (EnumHasAnyFlags(Required, EVoxelOceanWaveInputs::Depth) && Density.IsValid() && Sites.Num() > 0)
    {
        // One batch through the compositor (the density with the terrain's edits).
        TArray<double> X, Y, Z;
        TArray<float> Values;
        X.SetNumUninitialized(Sites.Num());
        Y.SetNumUninitialized(Sites.Num());
        Z.SetNumUninitialized(Sites.Num());
        Values.SetNumZeroed(Sites.Num());
        for (int32 i = 0; i < Sites.Num(); i++)
        {
            X[i] = Sites[i].Point.X;
            Y[i] = Sites[i].Point.Y;
            Z[i] = Sites[i].Point.Z;
        }
        // The depth the waves feel: the terrain's detail finer than the depth footprint left out,
        // less the point's height over the sea sphere (as the drawn surface's, VoxelOceanPage.usf).
        const double Footprint = FMath::Max(DepthFootprintCm, 0.0);
        const double SeaRadius = Context.Radius;
        Density->Sample(FSampleInput(X.GetData(), Y.GetData(), Z.GetData(), Sites.Num(), Footprint), Values.GetData());
        for (int32 i = 0; i < Sites.Num(); i++)
            Sites[i].Depth = Values[i] - (SeaRadius > 0.0 ? Sites[i].Point.Size() - SeaRadius : 0.0);
        // Stage W6h: the terrain's own depth where the felt depth can differ from it (the drawn
        // waves', VoxelOceanSurfFeltDepth: within its blend of the coast, and on the land side,
        // where the smoothing averaged the high ground behind the beach into the shallows).
        {
            TArray<int32> Near;
            TArray<double> FX, FY, FZ;
            for (int32 i = 0; i < Sites.Num(); i++)
            {
                if (Sites[i].Depth >= VoxelOceanWavesDetail::FeltBelow || Sites[i].Depth <= VoxelOceanWavesDetail::BedLand) continue;
                Near.Add(i);
                FX.Add(Sites[i].Point.X);
                FY.Add(Sites[i].Point.Y);
                FZ.Add(Sites[i].Point.Z);
            }
            if (Near.Num() > 0)
            {
                TArray<float> FValues;
                FValues.SetNumZeroed(Near.Num());
                Density->Sample(FSampleInput(FX.GetData(), FY.GetData(), FZ.GetData(), Near.Num(), FMath::Min(Footprint, VoxelOceanWavesDetail::FeltFootprint)),
                    FValues.GetData());
                for (int32 k = 0; k < Near.Num(); k++)
                    Sites[Near[k]].BedDepth = FValues[k] - (SeaRadius > 0.0 ? Sites[Near[k]].Point.Size() - SeaRadius : 0.0);
            }
        }
        // An inland basin below sea level keeps the terrain's depth (stage O9b3): the model drains
        // the surface below the ground toward it (VoxelOceanSurfDrainOf), as the pages' waves do.
        // The bed's slope (stage O9b2): the depth's fall over the footprint either side along the
        // field's landward direction and across it (stage O9e: the whole slope's steepness,
        // VoxelOceanBedWhole, and the way it rises), where the depth lies near any surf (as
        // VoxelOceanPage.usf's).
        if (EnumHasAnyFlags(Required, EVoxelOceanWaveInputs::Shore) && Footprint <= VoxelOceanWavesDetail::BedMaxFootprint)
        {
            const double Step = FMath::Max(Footprint, VoxelOceanWavesDetail::BedMinStep);
            TArray<int32> Measured;
            TArray<double> BX, BY, BZ;
            for (int32 i = 0; i < Sites.Num(); i++)
            {
                const FVoxelOceanWaveSite& Site = Sites[i];
                const double L2 = Site.Shore.Direction.SizeSquared();
                if (!Site.Shore.HasField() || L2 < 0.01 || Site.Depth <= VoxelOceanWavesDetail::BedLand || Site.Depth >= VoxelOceanWavesDetail::BedSea)
                    continue;
                const FVector Along = Site.Shore.Direction * (Step / FMath::Sqrt(L2));
                const FVector Cross = FVector::CrossProduct(Site.Point.GetSafeNormal(), Site.Shore.Direction);
                const double CrossLength = Cross.Size();
                const FVector Across = CrossLength > 1.0e-3 * FMath::Sqrt(L2) ? Cross * (Step / CrossLength) : FVector::ZeroVector;
                for (int32 Side = 0; Side < 4; Side++)
                {
                    const FVector P = Side == 0 ? Site.Point - Along : (Side == 1 ? Site.Point + Along : (Side == 2 ? Site.Point - Across : Site.Point + Across));
                    BX.Add(P.X);
                    BY.Add(P.Y);
                    BZ.Add(P.Z);
                }
                Measured.Add(i);
            }
            if (Measured.Num() > 0)
            {
                TArray<float> BValues;
                BValues.SetNumZeroed(BX.Num());
                Density->Sample(FSampleInput(BX.GetData(), BY.GetData(), BZ.GetData(), BX.Num(), Footprint), BValues.GetData());
                for (int32 k = 0; k < Measured.Num(); k++)
                {
                    auto Lifted = [&](int32 j) { return (double)BValues[j] - (SeaRadius > 0.0 ? FVector(BX[j], BY[j], BZ[j]).Size() - SeaRadius : 0.0); };
                    const double AlongSlope = (Lifted(4 * k) - Lifted(4 * k + 1)) / (2.0 * Step);
                    const double AcrossSlope = (Lifted(4 * k + 2) - Lifted(4 * k + 3)) / (2.0 * Step);
                    // The steepness (VoxelOceanBedWhole, VoxelOceanShore.ush: the two steps are
                    // orthogonal here) and the direction it rises in (toward the shallows).
                    const double Whole = FMath::Sqrt(AlongSlope * AlongSlope + AcrossSlope * AcrossSlope);
                    Sites[Measured[k]].BedSlope = Whole;
                    const FVector& Dir = Sites[Measured[k]].Shore.Direction;
                    const FVector E = Dir.GetSafeNormal();
                    const FVector F = FVector::CrossProduct(Sites[Measured[k]].Point.GetSafeNormal(), Dir).GetSafeNormal();
                    Sites[Measured[k]].BedRise = Whole > 1.0e-9 ? ((E * AlongSlope + F * AcrossSlope) / Whole) : FVector::ZeroVector;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Query
// ---------------------------------------------------------------------------

void VoxelOceanQuery::Run(const FVoxelOceanWaveFrame& Frame, TConstArrayView<FVector> PlanetPoints,
    TArrayView<FVoxelOceanSample> Out, EVoxelOceanQuery Flags, double Time)
{
    const int32 Count = FMath::Min(PlanetPoints.Num(), Out.Num());
    if (Count <= 0) return;
    const double Radius = Frame.Radius();
    FVoxelOceanWaveContext Context = Frame.Context;
    if (Time >= 0.0)
        Context.Time = Time;

    TArray<FVoxelOceanWaveSite> Sites;
    TArray<FVoxelOceanWaveResult> Results;
    TArray<FVector> Feet;
    TArray<uint8> Done;
    Sites.SetNum(Count);
    Results.SetNum(Count);
    Feet.SetNumUninitialized(Count);
    Done.SetNumZeroed(Count);
    for (int32 i = 0; i < Count; i++)
    {
        const FVector U = PlanetPoints[i].GetSafeNormal();
        Feet[i] = U * Radius;
        Sites[i].Point = Feet[i];
    }

    const bool bWaves = Frame.HasWaves();
    Frame.FillSites(Sites, Context.Time);
    if (bWaves)
        Frame.Model->Evaluate(Context, Sites, Results);

    // The surface point on each query point's radial: X + D_h(X) = foot, by Newton steps in the
    // tangent plane at the foot (the plain step where the surface nearly folds), X kept on the
    // sphere. Without horizontal displacement X is the foot.
    if (bWaves && Frame.MaxHorizontal() > 0.0)
    {
        for (int32 Step = 0; Step < VOW::QuerySteps; Step++)
        {
            bool bAny = false;
            for (int32 i = 0; i < Count; i++)
            {
                if (Done[i]) continue;
                const FVector U = Feet[i] / Radius;
                const FVoxelOceanWaveResult& R = Results[i];
                const FVector HorizontalD = R.Displacement - Sites[i].Up * R.Height;
                FVector Residual = Sites[i].Point + HorizontalD - Feet[i];
                Residual -= U * (U | Residual);
                if (Residual.Size() < VOW::QueryTolerance)
                {
                    Done[i] = 1;
                    continue;
                }
                // The tangent basis at the foot and the 2x2 system (I + J) delta = residual.
                FVector E1, E2;
                U.FindBestAxisVectors(E1, E2);
                auto Apply = [&R](const FVector& V)
                    {
                        return FVector(R.HorizontalGradient[0] | V, R.HorizontalGradient[1] | V, R.HorizontalGradient[2] | V);
                    };
                const FVector JE1 = Apply(E1);
                const FVector JE2 = Apply(E2);
                const double A11 = 1.0 + (E1 | JE1), A12 = E1 | JE2;
                const double A21 = E2 | JE1, A22 = 1.0 + (E2 | JE2);
                const double Det = FMath::Max(A11 * A22 - A12 * A21, VOW::QueryMinJacobian);
                const double R1 = E1 | Residual, R2 = E2 | Residual;
                const FVector Delta = E1 * ((A22 * R1 - A12 * R2) / Det) + E2 * ((A11 * R2 - A21 * R1) / Det);
                Sites[i].Point = (Sites[i].Point - Delta).GetSafeNormal() * Radius;
                bAny = true;
            }
            if (!bAny) break;
            Frame.FillSites(Sites, Context.Time);
            Frame.Model->Evaluate(Context, Sites, Results);
        }
        // The last step's points: settled or not.
        for (int32 i = 0; i < Count; i++)
        {
            if (Done[i]) continue;
            const FVector U = Feet[i] / Radius;
            FVector Residual = Sites[i].Point + (Results[i].Displacement - Sites[i].Up * Results[i].Height) - Feet[i];
            Residual -= U * (U | Residual);
            Done[i] = Residual.Size() < VOW::QueryTolerance ? 1 : 0;
        }
    }

    TArray<FVector> SurfacePoints;
    if (EnumHasAnyFlags(Flags, EVoxelOceanQuery::Depth))
        SurfacePoints.SetNumUninitialized(Count);
    for (int32 i = 0; i < Count; i++)
    {
        FVoxelOceanSample& S = Out[i];
        S = FVoxelOceanSample();
        const FVoxelOceanWaveResult& R = Results[i];
        const double Distance = PlanetPoints[i].Size();
        if (bWaves)
        {
            S.Height = Distance - Radius - R.Height;
            S.SurfaceLocation = Sites[i].Point + R.Displacement;
            S.Normal = R.Normal;
            S.Velocity = R.Velocity;
            S.Jacobian = R.Jacobian;
            S.bAmbiguous = R.Jacobian <= 0.0 || (Frame.MaxHorizontal() > 0.0 && !Done[i]);
        }
        else
        {
            S.Height = Distance - Radius;
            S.SurfaceLocation = Feet[i];
            S.Normal = Feet[i] / FMath::Max(Radius, 1.0);
        }
        if (SurfacePoints.Num() > 0)
            SurfacePoints[i] = S.SurfaceLocation;
    }

    // No water in an inland basin (stage O8): the shore field at the surface point's foot.
    if (Frame.Shore.IsValid())
    {
        for (int32 i = 0; i < Count; i++)
        {
            const FVoxelOceanShore At = Frame.Shore->Sample(Feet[i]);
            Out[i].bWater = !(At.HasField() && At.Distance < -Frame.ShoreMarginCm);
        }
    }

    if (SurfacePoints.Num() > 0 && Frame.Density.IsValid())
    {
        TArray<double> X, Y, Z;
        TArray<float> Values;
        X.SetNumUninitialized(Count);
        Y.SetNumUninitialized(Count);
        Z.SetNumUninitialized(Count);
        Values.SetNumZeroed(Count);
        for (int32 i = 0; i < Count; i++)
        {
            X[i] = SurfacePoints[i].X;
            Y[i] = SurfacePoints[i].Y;
            Z[i] = SurfacePoints[i].Z;
        }
        Frame.Density->Sample(FSampleInput(X.GetData(), Y.GetData(), Z.GetData(), Count), Values.GetData());
        for (int32 i = 0; i < Count; i++)
            Out[i].Depth = Values[i];
    }
}
