#include "VoxelOceanWavesFFT.h"
#include "VoxelOceanResolvedShare.h"
#include "VoxelOceanShoreCore.h"
#include "Misc/ScopeLock.h"

#include <cmath>

// The shared sum of the lookups (VoxelOceanFFTWaves.ush) and the shoaling (VoxelOceanShoal.ush),
// after the charts and the shore field VoxelOceanShoreCore.h brought in.
namespace VoxelShader
{
#include "VoxelOceanFFTWaves.ush"
#include "VoxelOceanShoal.ush"
#include "VoxelOceanSurf.ush"
#include "VoxelOceanFeltDepth.ush"
#include "VoxelOceanFoam.ush"
}

namespace VoxelOceanWavesFFTDetail
{
    constexpr int32 QuerySize = 128;
    /** Query cascade updates per second of the ocean clock (interpolated between). */
    constexpr double QueryRate = 15.0;
    constexpr int32 CachedFrames = 4;
    /** The waves' reach in standard deviations of the height (crests over a large area). */
    constexpr double ReachSigmas = 5.0;
    /** The lowest wind a short band's spectrum is built for (calmer seas scale it down). */
    constexpr double MinShortWind = 0.5;

    VoxelShader::FVoxelOceanFFTSample ToSample(const VoxelOceanSpectrum::FQuerySample& Q)
    {
        VoxelShader::FVoxelOceanFFTSample S;
        S.Height = 100.0f * Q.Height;
        S.Displacement = VoxelShader::float2(100.0f * Q.DisplacementX, 100.0f * Q.DisplacementY);
        S.Slope = VoxelShader::float2(Q.HeightDx, Q.HeightDy);
        S.Jxx = Q.DxDx;
        S.Jyy = Q.DyDy;
        S.Jxy = 0.5f * (Q.DxDy + Q.DyDx);
        S.VelocityUp = 100.0f * Q.VelocityZ;
        S.VelocityH = VoxelShader::float2(100.0f * Q.VelocityX, 100.0f * Q.VelocityY);
        return S;
    }

    /** The surf's settings as the shaders get them (VoxelOceanSurfSettingsOf; the chart table's
     *  constants rows 15, 16 and 18): strength (0 off), sets, swash, skew; the waves' chop (stage W3h), surf foam, foam
     *  coverage, the swash's reach (m); the directed trains' cutoff (deep wavelengths) and the
     *  FFT's floor (stage O9i), the waves' floor (stage W3g; the debug view in w, the surface's). */
    void SurfRows(const FVoxelOceanFFTSettings& Settings, VoxelShader::float4& Row0, VoxelShader::float4& Row1, VoxelShader::float4& Row2)
    {
        Row2 = VoxelShader::float4((float)FMath::Clamp(Settings.SurfCutoff, 0.0, 0.5), (float)FMath::Clamp(Settings.SurfFFTFloor, 0.0, 1.0),
            (float)FMath::Clamp(Settings.ShoreWaveFloor, 0.0, 1.0), 0.0f);
        Row0 = VoxelShader::float4((float)FMath::Max(Settings.SurfStrength, 0.0), (float)FMath::Clamp(Settings.SurfSets, 0.0, 1.0),
            (float)FMath::Max(Settings.SurfSwash, 0.0), (float)FMath::Clamp(Settings.SurfSkew, 0.0, 0.95));
        // (Stage W3v: x the foam's fresh gain, z its whitecap threshold, both written live by the
        // surface, VoxelOceanFoam.ush.)
        Row1 = VoxelShader::float4(0.0f, (float)FMath::Max(Settings.SurfFoam, 0.0), (float)FMath::Clamp(Settings.FoamWhitecap, 0.0, 1.0),
            (float)FMath::Max(Settings.SurfSwashReach, 1.0));
    }

    /** The surf's clock: the ocean clock modulo its wrap (the trains' periods divide it). */
    float SurfClock(double Time)
    {
        const double Wrap = (double)VOXEL_OCEAN_SURF_WRAP;
        const double T = std::fmod(Time, Wrap);
        return (float)(T < 0.0 ? T + Wrap : T);
    }
}

namespace VOF = VoxelOceanWavesFFTDetail;

double FVoxelOceanWavesFFT::ReachWind(double MaxLong, double Current, bool bHasCurrent)
{
    MaxLong = FMath::Max(MaxLong, 0.0);
    if (bHasCurrent && MaxLong <= Current && MaxLong > Current - ReachSlack) return Current;
    return FMath::Max(std::ceil(MaxLong / ReachStep), 1.0) * ReachStep;
}

FVoxelOceanWavesFFT::FVoxelOceanWavesFFT(const FVoxelOceanFFTSettings& InSettings, double InRadius)
    : Settings(InSettings)
    , Radius(InRadius)
{
    using namespace VoxelOceanSpectrum;
    FLayoutSettings Layout;
    Layout.PlanetSeed = VoxelShader::VoxelOceanHash((uint32)Settings.Seed ^ 0x0cea51edu);
    Layout.Fetch = FMath::Max(Settings.FetchKm, 1.0) * 1000.0;
    Layout.Swell = FMath::Clamp(Settings.Swell, 0.0, 1.0);
    for (int32 b = 0; b < BandCount; b++)
        Layout.Chop[b] = FMath::Max(Settings.Chop, 0.0);
    static_assert(FVoxelOceanFFTSettings::EqualizerKnots == VOXEL_OCEAN_EQ_KNOTS, "the equalizer's knots must match the shaders'");
    for (int32 i = 0; i < VOXEL_OCEAN_EQ_KNOTS; i++)
        Layout.Gain[i] = FMath::Max(Settings.Gain[i], 0.0);
    Layout.ShortWind = ShortReferenceWind;
    Tiles = FVector4f((float)Layout.TileLength[0], (float)Layout.TileLength[1], (float)Layout.TileLength[2], (float)Layout.TileLength[3]);

    Slots = MakeSlots(Layout);
    for (const FSlot& Slot : Slots)
        SlotTable.Add(ToGPU(Slot));

    // What each slot holds.
    for (int32 Slot = 0; Slot < SlotCount; Slot++)
    {
        const FBandStats Stats = StatsOf(Slots[Slot]);
        SlotSigma[Slot] = FMath::Sqrt(Stats.HeightVariance);
        SlotSlope[Slot] = FMath::Sqrt(Stats.SlopeVarianceX + Stats.SlopeVarianceY);
        // (Stage W3v: unused; the retired per-slot foam's spread.)
        SlotTable[Slot].B[2] = 0.0f;
    }
    // The table's constants: the slope deviations, and the short bands' height deviation at
    // each knot against their reference's.
    float* C = Constants.Texels;
    for (int32 Slot = 0; Slot < SlotCount; Slot++)
        C[Slot] = (float)SlotSlope[Slot];
    for (int32 Band = 2; Band < BandCount; Band++)
    {
        const double Reference = FMath::Max(SlotSigma[4 + Band], 1.0e-12);
        float* Curve = C + (Band == 2 ? 8 : 16);
        for (int32 K = 0; K < VOXEL_OCEAN_SHORT_KNOTS; K++)
        {
            const double U = (double)VoxelShader::VoxelOceanShortKnot(K);
            double Ratio = 0.0;
            if (U > 0.0)
            {
                FLayoutSettings At = Layout;
                At.ShortWind = FMath::Max(U, VOF::MinShortWind);
                const std::vector<FSlot> AtSlots = MakeSlots(At);
                Ratio = FMath::Sqrt(StatsOf(AtSlots[4 + Band]).HeightVariance) / Reference;
                if (U < VOF::MinShortWind) Ratio *= FMath::Square(U / VOF::MinShortWind);
            }
            Curve[K] = (float)Ratio;
        }
    }

    // The slots' height deviations (m): the breaking's (stage O8).
    for (int32 Slot = 0; Slot < SlotCount; Slot++)
        C[24 + Slot] = (float)SlotSigma[Slot];

    // Stage W6b: each slot's share of its slope variance its lookups keep per mip (the GPU's box mips
    // and trilinear lookups, from its modes; VoxelOceanResolvedShare.h), for the unresolved
    // roughness at any footprint (VoxelOceanSlotUnresolved).
    // The clipmap's texels' second set (their grid's own bilinear lookup on top), from row
    // VOXEL_OCEAN_TABLE_CLIP_ROW (VoxelOceanSlotUnresolvedClip).
    static_assert(VOXEL_OCEAN_TABLE_RESOLVED_TEXELS == 2 * SlotCount * ResolvedMips && VOXEL_OCEAN_RESOLVED_MIPS == ResolvedMips
        && VOXEL_OCEAN_TABLE_CLIP_ROW == VOXEL_OCEAN_TABLE_RESOLVED_ROW + SlotCount * ResolvedMips, "the resolved shares' rows must match the shaders'");
    for (int32 Slot = 0; Slot < SlotCount; Slot++)
    {
        const FResolvedShare Share = ResolvedShareOf(Slots[Slot]);
        PackResolvedRows(Share, Constants.Resolved + 4 * ResolvedMips * Slot);
        PackClipRows(Share, Constants.Resolved + 4 * ResolvedMips * (SlotCount + Slot));
    }

    // The reach, at the reach's wind; shoaled waves grow (stage O8).
    MaxHeightCm = HeightBoundCm(FMath::Max(Settings.WindSpeed, 0.0));
    MaxHorizontalCm = FMath::Max(Settings.Chop, 0.0) * VOF::ReachSigmas * SigmaAt(FMath::Max(Settings.WindSpeed, 0.0)) * 100.0;
    if (Settings.ShoreBreaking > 0.0)
        MaxHorizontalCm *= VOXEL_OCEAN_SHOAL_REACH_HORIZONTAL;

    Charts = VoxelOceanCharts::Build(Radius);
    // An exact float in the shaders (a 24-bit integer).
    DetileSeed = VoxelShader::VoxelOceanHash((uint32)Settings.Seed ^ 0xde711eu) & 0xffffffu;

    // The query's slots: every long slot (the weather may weigh any state), band 2 on request.
    for (int32 State = 0; State < StateCount; State++)
        for (int32 Band = 0; Band < 2; Band++)
            QuerySlots.Add({ 2 * State + Band, Band, State });
    if (Settings.bQueryShortWaves)
        QuerySlots.Add({ 6, 2, -1 });
    for (const FQuerySlot& Q : QuerySlots)
    {
        TSharedPtr<FQueryCascade, ESPMode::ThreadSafe> Cascade = MakeShared<FQueryCascade, ESPMode::ThreadSafe>(Slots[Q.Slot], VOF::QuerySize);
        check(Cascade->IsValid());
        Prototypes.Add(Cascade);
    }
}

double FVoxelOceanWavesFFT::HeightBoundCm(double Wind) const
{
    double Bound = VOF::ReachSigmas * SigmaAt(Wind) * 100.0;
    if (Settings.ShoreBreaking > 0.0)
    {
        Bound *= VOXEL_OCEAN_SHOAL_REACH_HEIGHT;
        // The surf (stage O9a) at its most: the long bands' significant height taken as 4 sigma
        // of the whole sea (at least theirs), the short bands' 3 sigma on top.
        VoxelShader::float4 Row0, Row1, Row2;
        VOF::SurfRows(Settings, Row0, Row1, Row2);
        const VoxelShader::FVoxelOceanSurfSettings Surf = VoxelShader::VoxelOceanSurfSettingsOf(Row0, Row1, Row2, (float)Settings.ShoreBreaking);
        const double Sigma = SigmaAt(Wind);
        const double SurfCm = ((double)VoxelShader::VoxelOceanSurfMaxHeight((float)(4.0 * Sigma), Surf) + 3.0 * Sigma) * 100.0;
        Bound = FMath::Max(Bound, SurfCm);
    }
    return Bound;
}

double FVoxelOceanWavesFFT::MaxHeightAt(const FVoxelOceanWaveContext& Context, const FVector& PlanetPoint) const
{
    // Stage W6i: the reach for the sea state at the point (its wind or swell, the larger, at either
    // of the weather interval's keys), no more than the planet's. The underwater pass's skip test:
    // the planet's reach is the stormiest sea's anywhere, so a camera metres over a calm sea ran the
    // whole pass (its surface map's million FFT evaluations) every frame.
    using namespace VoxelShader;
    const FVoxelOceanWeatherTable* Weather = Context.Weather.Get();
    if (!Weather || Weather->Table.N != Charts.N) return MaxHeightCm;
    const FVoxelOceanTableReader Reader = Weather->Table.Reader();
    const VoxelOceanCharts::FVec Dir = VoxelOceanCharts::Normalize({ PlanetPoint.X, PlanetPoint.Y, PlanetPoint.Z });
    const float3 D((float)Dir.X, (float)Dir.Y, (float)Dir.Z);
    double Wind = 0.0;
    for (int32 Key = 0; Key < 2; Key++)
    {
        const FVoxelOceanSeaState Sea = VoxelOceanSeaStateAt(Reader, D, Charts.N, (float)Key);
        Wind = FMath::Max(Wind, (double)FMath::Max(Sea.Wind, Sea.Swell));
    }
    return FMath::Min(MaxHeightCm, HeightBoundCm(Wind));
}

double FVoxelOceanWavesFFT::ShortScale(int32 Band, double Wind) const
{
    // The hook's curve (VoxelOceanShortScale), on the constants.
    const float* Curve = Constants.Texels + (Band == 2 ? 8 : 16);
    const double Last = VoxelShader::VoxelOceanShortKnot(VOXEL_OCEAN_SHORT_KNOTS - 1);
    if (Wind >= Last) return Curve[VOXEL_OCEAN_SHORT_KNOTS - 1];
    int32 K = 0;
    while (K < VOXEL_OCEAN_SHORT_KNOTS - 2 && Wind >= VoxelShader::VoxelOceanShortKnot(K + 1)) K++;
    const double U0 = VoxelShader::VoxelOceanShortKnot(K), U1 = VoxelShader::VoxelOceanShortKnot(K + 1);
    return Curve[K] + (Curve[K + 1] - Curve[K]) * FMath::Clamp((Wind - U0) / (U1 - U0), 0.0, 1.0);
}

double FVoxelOceanWavesFFT::SigmaAt(double Wind) const
{
    using namespace VoxelOceanSpectrum;
    const VoxelShader::float3 W = VoxelShader::VoxelOceanStateWeights((float)Wind);
    const double Weight[3] = { W.x, W.y, W.z };
    double Variance = 0.0;
    for (int32 Band = 0; Band < 2; Band++)
    {
        double Sigma = 0.0;
        for (int32 State = 0; State < StateCount; State++)
            Sigma += Weight[State] * SlotSigma[2 * State + Band];
        Variance += Sigma * Sigma;
    }
    // The short bands at their largest for any wind up to this one.
    for (int32 Band = 2; Band < BandCount; Band++)
    {
        double Scale = ShortScale(Band, Wind);
        for (int32 K = 0; K < VOXEL_OCEAN_SHORT_KNOTS && VoxelShader::VoxelOceanShortKnot(K) <= Wind; K++)
            Scale = FMath::Max(Scale, (double)Constants.Texels[(Band == 2 ? 8 : 16) + K]);
        Variance += FMath::Square(Scale * SlotSigma[4 + Band]);
    }
    return FMath::Sqrt(Variance);
}

TSharedPtr<const FVoxelOceanWavesFFT::FFrame, ESPMode::ThreadSafe> FVoxelOceanWavesFFT::GetFrame(int64 Index, uint32 Needed) const
{
    auto Missing = [Needed](const FFrame& Frame)
    {
        uint32 Bits = 0;
        for (int32 q = 0; q < Frame.Cascades.Num(); q++)
            if ((Needed & (1u << q)) != 0u && !Frame.Cascades[q].IsValid()) Bits |= 1u << q;
        return Bits;
    };
    TSharedPtr<const FFrame, ESPMode::ThreadSafe> Have;
    {
        FScopeLock Lock(&CacheLock);
        for (const TSharedPtr<const FFrame, ESPMode::ThreadSafe>& Frame : Cache)
        {
            if (Frame->Index == Index) { Have = Frame; break; }
        }
    }
    if (Have.IsValid() && Missing(*Have) == 0u) return Have;

    // The missing slots, computed outside the lock (other threads keep reading the cache); two
    // threads asking for the same slots at once both compute them, and the first one in is kept.
    TSharedPtr<FFrame, ESPMode::ThreadSafe> Fuller = MakeShared<FFrame, ESPMode::ThreadSafe>();
    Fuller->Index = Index;
    Fuller->Cascades.SetNum(Prototypes.Num());
    if (Have.IsValid())
        for (int32 q = 0; q < Prototypes.Num(); q++) Fuller->Cascades[q] = Have->Cascades[q];
    const uint32 Tick = VoxelOceanSpectrum::TickOf((double)Index / VOF::QueryRate);
    for (int32 q = 0; q < Prototypes.Num(); q++)
    {
        if ((Needed & (1u << q)) == 0u || Fuller->Cascades[q].IsValid()) continue;
        TSharedPtr<VoxelOceanSpectrum::FQueryCascade, ESPMode::ThreadSafe> Cascade = MakeShared<VoxelOceanSpectrum::FQueryCascade, ESPMode::ThreadSafe>(*Prototypes[q]);
        Cascade->Update(Tick);
        Fuller->Cascades[q] = Cascade;
    }
    FScopeLock Lock(&CacheLock);
    for (int32 i = 0; i < Cache.Num(); i++)
    {
        if (Cache[i]->Index != Index) continue;
        // Someone else's frame went in meanwhile: theirs with ours merged in.
        for (int32 q = 0; q < Prototypes.Num(); q++)
            if (!Fuller->Cascades[q].IsValid()) Fuller->Cascades[q] = Cache[i]->Cascades[q];
        Cache[i] = Fuller;
        return Fuller;
    }
    Cache.Add(Fuller);
    if (Cache.Num() > VOF::CachedFrames)
        Cache.RemoveAt(0);
    return Fuller;
}

void FVoxelOceanWavesFFT::Evaluate(const FVoxelOceanWaveContext& Context, TConstArrayView<FVoxelOceanWaveSite> Sites,
    TArrayView<FVoxelOceanWaveResult> Out) const
{
    using namespace VoxelShader;
    const int32 Count = FMath::Min(Sites.Num(), Out.Num());
    if (Count <= 0) return;
    const float DetileCell = (float)FMath::Max(Settings.DetileCell, 0.0);

    // The weather: the table's (as the GPU's), or the context's constants.
    const FVoxelOceanWeatherTable* Weather = Context.Weather.Get();
    if (Weather && Weather->Table.N != Charts.N) Weather = nullptr;
    const FVoxelOceanTableReader Reader = Weather ? Weather->Table.Reader() : FVoxelOceanTableReader();
    const float WeatherAlpha = Weather ? (float)Weather->Alpha(Context.Time) : 0.0f;
    const float Fade = Weather ? (float)Weather->Fade(Context.Time) : 1.0f;

    // Each site's sea state first: the query cascades are computed only for the slots weighed.
    TArray<FVoxelOceanSeaState> Seas;
    Seas.SetNum(Count);
    uint32 Needed = 0;
    for (int32 i = 0; i < Count; i++)
    {
        const FVoxelOceanWaveSite& Site = Sites[i];
        FVoxelOceanSeaState& Sea = Seas[i];
        if (Weather)
        {
            const VoxelOceanCharts::FVec Dir = VoxelOceanCharts::Normalize({ Site.Point.X, Site.Point.Y, Site.Point.Z });
            Sea = VoxelOceanSeaStateAt(Reader, float3((float)Dir.X, (float)Dir.Y, (float)Dir.Z), Charts.N, WeatherAlpha);
        }
        else
        {
            const double Speed = FMath::Max(Site.Environment.WindSpeed, 0.0);
            Sea.Wind = (float)Speed;
            Sea.Swell = 0.0f;
            Sea.Long = VoxelOceanStateWeights((float)Speed);
            Sea.Short2 = (float)ShortScale(2, Speed);
            Sea.Short3 = (float)ShortScale(3, Speed);
        }
        const float StateWeight[3] = { Sea.Long.x, Sea.Long.y, Sea.Long.z };
        for (int32 q = 0; q < QuerySlots.Num(); q++)
            if ((QuerySlots[q].State >= 0 ? StateWeight[QuerySlots[q].State] : Sea.Short2) > 0.0f) Needed |= 1u << q;
    }

    // The two 15 Hz frames around the time, and how far between.
    const double Step = Context.Time * VOF::QueryRate;
    const double FloorStep = FMath::FloorToDouble(Step);
    const float Alpha = (float)(Step - FloorStep);
    const TSharedPtr<const FFrame, ESPMode::ThreadSafe> F0 = GetFrame((int64)FloorStep, Needed);
    const TSharedPtr<const FFrame, ESPMode::ThreadSafe> F1 = GetFrame((int64)FloorStep + 1, Needed);

    auto Lookup = [&F0, &F1, Alpha](int32 Q, double X, double Y)
    {
        const FVoxelOceanFFTSample A = VOF::ToSample(F0->Cascades[Q]->Sample(X, Y));
        const FVoxelOceanFFTSample B = VOF::ToSample(F1->Cascades[Q]->Sample(X, Y));
        return VoxelOceanFFTSampleMad(VoxelOceanFFTSampleMad(VoxelOceanFFTSampleZero(), 1.0f - Alpha, A), Alpha, B);
    };

    // The shoaling's settings (as the shaders get them, GetShaderParams) and the bands' tiles.
    FVoxelOceanShoalSettings ShoalSettings;
    ShoalSettings.Breaking = (float)FMath::Max(Settings.ShoreBreaking, 0.0);
    ShoalSettings.ChopGrowth = (float)FMath::Max(Settings.ShoreChopGrowth, 0.0);
    ShoalSettings.Warp = (float)FMath::Max(Settings.ShoreWarp, 0.0);
    ShoalSettings.Fade = (float)FMath::Max(Settings.ShoreFade, 0.0);
    ShoalSettings.MinSlope = (float)FMath::Max(Settings.ShoreMinSlope, 1.0e-4);
    ShoalSettings.LongKeep = 1.0f;
    ShoalSettings.Floor = (float)FMath::Clamp(Settings.ShoreWaveFloor, 0.0, 1.0);
    const float4 BandTiles(Tiles.X, Tiles.Y, Tiles.Z, Tiles.W);
    // The surf's settings (as the shaders get them, VOF::SurfRows) and the clock it runs on.
    float4 SurfRow0, SurfRow1, SurfRow2;
    VOF::SurfRows(Settings, SurfRow0, SurfRow1, SurfRow2);
    const FVoxelOceanSurfSettings SurfSettings = VoxelOceanSurfSettingsOf(SurfRow0, SurfRow1, SurfRow2, ShoalSettings.Breaking);
    const float SurfTime = VOF::SurfClock(Context.Time);
    const float* Sig = Constants.Texels + 24;
    const float4 SlotSigma0(Sig[0], Sig[1], Sig[2], Sig[3]);
    const float4 SlotSigma1(Sig[4], Sig[5], Sig[6], Sig[7]);

    for (int32 i = 0; i < Count; i++)
    {
        const FVoxelOceanWaveSite& Site = Sites[i];
        FVoxelOceanWaveResult& R = Out[i];
        R = FVoxelOceanWaveResult();
        R.Normal = Site.Up;
        const VoxelOceanCharts::FVec P = { Site.Point.X, Site.Point.Y, Site.Point.Z };
        const VoxelOceanCharts::FPointCharts PointCharts = VoxelOceanCharts::ChartsAt(Charts, P);

        // The site's shoaling (VoxelOceanShoal.ush, as the hook's VoxelOceanFFTShoalAt).
        FVoxelOceanShoal Shoal = VoxelOceanShoalNone();
        FVoxelOceanSurf Surf = VoxelOceanSurfNone();
        float3 SurfDir(0.0f, 0.0f, 0.0f);
        FVoxelOceanSurfDrain Drain;
        Drain.Keep = 1.0f;
        Drain.Drop = 0.0f;
        Drain.Slope = 0.0f;
        Drain.SlopeBed = 0.0f;
        float3 DrainDir(0.0f, 0.0f, 0.0f);
        // The bed's rise (stage O9e; the site's, the field's landward direction where unknown).
        const double RiseL = Site.BedRise.Size();
        const bool bRise = RiseL > 0.5;
        const float3 Rise = bRise ? float3((float)(Site.BedRise.X / RiseL), (float)(Site.BedRise.Y / RiseL), (float)(Site.BedRise.Z / RiseL)) : float3(0.0f, 0.0f, 0.0f);
        // Stage W6h: the depth the waves feel, as the drawn waves' (VoxelOceanSurfFeltDepth, stage W5e):
        // the terrain's own where it lies lower near the coast (an inland basin's as land, as the
        // clipmap's static layer has it). Until W6h the query's waves (and the buoyancy on them)
        // stopped at the smoothed depth's line offshore, where the drawn ones run on to the beach.
        const bool bBasinSite = Site.Shore.HasField() && Context.ShoreMarginM > 0.0 && Site.Shore.Distance * 0.01 < -Context.ShoreMarginM;
        const bool bBed = !bBasinSite && Site.BedDepth < 0.5 * VoxelOceanWaves::Deep;
        const double FeltCm = Site.Depth < 0.5 * VoxelOceanWaves::Deep
            ? (double)VoxelOceanSurfFeltDepth((float)Site.Depth, bBed ? (float)Site.BedDepth : 1.0e30f, 100.0f * VOXEL_OCEAN_SURF_FELT_BLEND) : Site.Depth;
        // The drain toward inland basins (stage O9b3), as the hook: with shoaling and the field.
        if (ShoalSettings.Breaking > 0.0f && Context.ShoreCells > 0 && Site.Shore.HasField())
        {
            const float Bed = Site.BedSlope > VoxelOceanWaves::BedUnknown * 0.1 ? VoxelOceanBedSlopeOf(VoxelOceanBedCode((float)Site.BedSlope)) : (float)Site.Shore.Slope;
            Drain = VoxelOceanSurfDrainOf((float)(Site.Shore.Distance * 0.01),
                FeltCm < 0.5 * VoxelOceanWaves::Deep ? (float)(FeltCm * 0.01) : 1.0e30f, Bed, (float)Context.ShoreMarginM);
            const double L = Site.Shore.Direction.Size();
            if (L > 1.0e-6)
                DrainDir = float3((float)(Site.Shore.Direction.X / L), (float)(Site.Shore.Direction.Y / L), (float)(Site.Shore.Direction.Z / L));
        }
        // (Stage W3g: the drain sinks the sea but no longer stills it, as the hook.)
        const bool bStill = false;
        if (ShoalSettings.Breaking > 0.0f)
        {
            const bool bField = Context.ShoreCells > 0 && Site.Shore.HasField();
            FVoxelOceanShoreSample Field;
            Field.Distance = bField ? (float)(Site.Shore.Distance * 0.01) : 1.0e30f;
            Field.Slope = (float)Site.Shore.Slope;
            Field.Direction = bField ? float3((float)Site.Shore.Direction.X, (float)Site.Shore.Direction.Y, (float)Site.Shore.Direction.Z) : float3(0.0f, 0.0f, 0.0f);
            Field.Weight = 1.0f;
            const float DepthM = FeltCm < 0.5 * VoxelOceanWaves::Deep ? (float)(FeltCm * 0.01) : 1.0e30f;
            const float4 Sigma = VoxelOceanShoalBandSigma(SlotSigma0, SlotSigma1, Seas[i]);
            // The surf (VoxelOceanSurf.ush, as the hook's VoxelOceanFFTSurfAt), first: its handover
            // goes into the shoaling, before the long bands' cap.
            if (bField && SurfSettings.Strength > 0.0f && DepthM < 1.0e20f && !bStill)
            {
                const float Agree = dot(Field.Direction, Field.Direction);
                FVoxelOceanSurfInput In;
                In.DepthM = DepthM;
                In.Slope = FMath::Max(Field.Slope, ShoalSettings.MinSlope);
                In.ShoreM = Field.Distance;
                In.Hs = 4.0f * FMath::Sqrt(Sigma.x * Sigma.x + Sigma.y * Sigma.y);
                In.Speed = FMath::Max(Seas[i].Wind, Seas[i].Swell);
                // The point for the noises: whole units of 4096 cm and the rest (as the hook's split).
                const double UX = std::floor(Site.Point.X / (double)VOXEL_OCEAN_SURF_UNIT);
                const double UY = std::floor(Site.Point.Y / (double)VOXEL_OCEAN_SURF_UNIT);
                const double UZ = std::floor(Site.Point.Z / (double)VOXEL_OCEAN_SURF_UNIT);
                In.Units = int3((int)UX, (int)UY, (int)UZ);
                In.Low = float3((float)(Site.Point.X - UX * VOXEL_OCEAN_SURF_UNIT), (float)(Site.Point.Y - UY * VOXEL_OCEAN_SURF_UNIT),
                    (float)(Site.Point.Z - UZ * VOXEL_OCEAN_SURF_UNIT));
                In.Time = SurfTime;
                In.SpacingM = 0.0f;
                In.FootprintM = 0.0f;
                // The bed's slope, coded as the mesh codes it (VoxelOceanBedCode, stage O9b2); the
                // swash's foam isn't queried.
                In.BedSlope = Site.BedSlope > VoxelOceanWaves::BedUnknown * 0.1 ? VoxelOceanBedSlopeOf(VoxelOceanBedCode((float)Site.BedSlope)) : VOXEL_OCEAN_BED_UNKNOWN;
                In.BedDepthM = bBed ? (float)(Site.BedDepth * 0.01) : 1.0e30f;
                In.ForNormals = 0.0f;
                // The coastal field (stage O9i; the site's shore source's, planet frame).
                In.CoastA = (float)Site.Shore.CoastA;
                In.CoastB = (float)Site.Shore.CoastB;
                In.CoastGradA = float3((float)Site.Shore.CoastGradA.X, (float)Site.Shore.CoastGradA.Y, (float)Site.Shore.CoastGradA.Z);
                In.CoastGradB = float3((float)Site.Shore.CoastGradB.X, (float)Site.Shore.CoastGradB.Y, (float)Site.Shore.CoastGradB.Z);
                In.CoastCover = (float)Site.Shore.CoastCover;
                In.CoastSeedM = (float)Site.Shore.CoastSeedM;
                In.CoastReachM = (float)Site.Shore.CoastReachM;
                Surf = VoxelOceanSurfOf(In, SurfSettings);
                SurfDir = Agree > 1.0e-12f ? Field.Direction * (1.0f / FMath::Sqrt(Agree)) : float3(0.0f, 0.0f, 0.0f);
            }
            FVoxelOceanShoalSettings SiteSettings = ShoalSettings;
            SiteSettings.LongKeep = Surf.LongKeep;
            Shoal = VoxelOceanShoalOf(DepthM, Field, bField, Seas[i], BandTiles, Sigma, SiteSettings);
        }
        const bool bShoal = Shoal.Active > 0.0f;

        // The slots' weights from the site's sea state.
        const FVoxelOceanSeaState& Sea = Seas[i];
        float2 WindEastNorth(0.0f, 0.0f);
        if (!Weather)
        {
            const double Speed = FMath::Max(Site.Environment.WindSpeed, 0.0);
            WindEastNorth = float2((float)(Speed * FMath::Sin(Site.Environment.WindHeading)), (float)(Speed * FMath::Cos(Site.Environment.WindHeading)));
        }
        const float StateWeight[3] = { Sea.Long.x, Sea.Long.y, Sea.Long.z };
        float SlotWeight[VoxelOceanSpectrum::SlotCount] = {};
        for (int32 q = 0; q < QuerySlots.Num(); q++)
            SlotWeight[q] = QuerySlots[q].State >= 0 ? StateWeight[QuerySlots[q].State] : Sea.Short2;

        FVoxelOceanFFTAccum Sum = VoxelOceanFFTAccumZero();
        for (int32 m = 0; m < PointCharts.Count && !bStill; m++)
        {
            const VoxelOceanCharts::FChartCPU& Chart = PointCharts.Charts[m];
            double X, Y;
            VoxelOceanCharts::Coords(Chart, P, X, Y);
            // Each band's own detiling lattice (stage O9b3, as the hook), the short bands' too.
            float2 Shifts[4][3];
            float CopyWeights[4][3];
            for (int32 b = 0; b < 4; b++)
            {
                const FVoxelOceanDetile Detile = VoxelOceanDetileOfBand(float2((float)X, (float)Y), DetileCell, DetileSeed, b, BandTiles);
                Shifts[b][0] = Detile.Offset0; Shifts[b][1] = Detile.Offset1; Shifts[b][2] = Detile.Offset2;
                CopyWeights[b][0] = Detile.Weight.x; CopyWeights[b][1] = Detile.Weight.y; CopyWeights[b][2] = Detile.Weight.z;
            }

            // Its headings: the table's (turning while the key's fade runs), or the constant wind's.
            FVoxelOceanChartHeadings H;
            if (Weather)
            {
                H = VoxelOceanChartHeadingsAt(Reader, int2(Chart.X, Chart.Y));
            }
            else
            {
                const float2 Wind = VoxelOceanChartWind(Chart.Chart, WindEastNorth);
                H.LongPrevious = Wind; H.ShortPrevious = Wind; H.LongCurrent = Wind; H.ShortCurrent = Wind;
            }
            // A group that turned blends its two orientations (VoxelOceanTurnWeights, as the hook).
            const float Distance = (float)std::sqrt(X * X + Y * Y);
            const float2 WLong = VoxelOceanTurnWeights(Fade, Distance, H.LongPrevious, H.LongCurrent, VoxelOceanLongCorrelationLength(Sea));
            const float2 WShort = VoxelOceanTurnWeights(Fade, Distance, H.ShortPrevious, H.ShortCurrent, VOXEL_OCEAN_SHORT_CORRELATION_LENGTH);
            for (int32 o = 0; o < 2; o++)
            {
                const float LongW = o == 0 ? WLong.x : WLong.y;
                const float ShortW = o == 0 ? WShort.x : WShort.y;
                if (LongW <= 0.0f && ShortW <= 0.0f) continue;
                const float2 LongWind = o == 0 ? H.LongCurrent : H.LongPrevious;
                const float2 ShortWind = o == 0 ? H.ShortCurrent : H.ShortPrevious;
                const double LX = LongWind.x * X + LongWind.y * Y, LY = -LongWind.y * X + LongWind.x * Y;
                const double SX = ShortWind.x * X + ShortWind.y * Y, SY = -ShortWind.y * X + ShortWind.x * Y;
                // The shore direction in the chart's plane, then in each group's downwind frame.
                const float2 ShorePlane(dot(Shoal.Dir, Chart.Chart.E1), dot(Shoal.Dir, Chart.Chart.E2));
                const float2 LongShore = VoxelOceanToWind(ShorePlane, LongWind);
                const float2 ShortShore = VoxelOceanToWind(ShorePlane, ShortWind);
                // Each band's lookups summed, moved by its warp, then shoaled (as the hook).
                FVoxelOceanFFTSample BandSum[4] = { VoxelOceanFFTSampleZero(), VoxelOceanFFTSampleZero(), VoxelOceanFFTSampleZero(), VoxelOceanFFTSampleZero() };
                for (int32 q = 0; q < QuerySlots.Num(); q++)
                {
                    const FQuerySlot& Slot = QuerySlots[q];
                    const float W = SlotWeight[q];
                    if (W <= 0.0f) continue;
                    const int32 B = Slot.Band;
                    if (bShoal && VoxelOceanShoalComponent(Shoal.Amp, B) <= 0.0f) continue;
                    const float2 E = B < 2 ? LongShore : ShortShore;
                    const float Move = bShoal ? VoxelOceanShoalComponent(Shoal.Shift, B) : 0.0f;
                    const double MX = (double)(E.x * Move), MY = (double)(E.y * Move);
                    if (B < 2)
                    {
                        if (LongW <= 0.0f) continue;
                        for (int32 c = 0; c < 3; c++)
                        {
                            if (CopyWeights[B][c] <= 0.0f) continue;
                            BandSum[B] = VoxelOceanFFTSampleMad(BandSum[B], W * CopyWeights[B][c], Lookup(q, LX + Shifts[B][c].x + MX, LY + Shifts[B][c].y + MY));
                        }
                    }
                    else if (ShortW > 0.0f)
                    {
                        for (int32 c = 0; c < 3; c++)
                        {
                            if (CopyWeights[B][c] <= 0.0f) continue;
                            BandSum[B] = VoxelOceanFFTSampleMad(BandSum[B], W * CopyWeights[B][c], Lookup(q, SX + Shifts[B][c].x + MX, SY + Shifts[B][c].y + MY));
                        }
                    }
                }
                FVoxelOceanFFTSample LongSum = VoxelOceanFFTSampleZero();
                FVoxelOceanFFTSample ShortSum = VoxelOceanFFTSampleZero();
                for (int32 B = 0; B < 4; B++)
                {
                    const float2 E = B < 2 ? LongShore : ShortShore;
                    const FVoxelOceanFFTSample Band = bShoal ? VoxelOceanShoalSample(BandSum[B], VoxelOceanShoalComponent(Shoal.Amp, B),
                        VoxelOceanShoalComponent(Shoal.Chop, B), VoxelOceanShoalComponent(Shoal.Stretch, B), E) : BandSum[B];
                    if (B < 2) LongSum = VoxelOceanFFTSampleMad(LongSum, 1.0f, Band);
                    else ShortSum = VoxelOceanFFTSampleMad(ShortSum, 1.0f, Band);
                }
                const float Weight = (float)PointCharts.Weight[m];
                Sum = VoxelOceanFFTAccumAdd(Sum, Chart.Chart, LongWind, Weight * LongW, LongSum);
                Sum = VoxelOceanFFTAccumAdd(Sum, Chart.Chart, ShortWind, Weight * ShortW, ShortSum);
            }
        }

        // The surf's height, gradient and rate (cm), as the hook.
        if (Surf.Active > 0.5f)
        {
            Sum.Height += Surf.Height * 100.0f;
            Sum.Gradient = Sum.Gradient + SurfDir * Surf.Gradient + (bRise ? Rise : SurfDir) * Surf.GradientBed + Surf.GradientPhase;
            Sum.VelocityUp += Surf.Rate * 100.0f;
        }
        // The drain toward inland basins (stage O9b3), as the hook.
        if (Drain.Keep < 1.0f)
        {
            Sum.Height -= Drain.Drop * 100.0f;
            Sum.Gradient = Sum.Gradient - DrainDir * Drain.Slope - (bRise ? Rise : DrainDir) * Drain.SlopeBed;
        }
        // Thin water (stage O9f), as the hook (the query's spacing: none, the least band).
        if (ShoalSettings.Breaking > 0.0f && Context.ShoreCells > 0 && Site.Shore.HasField() && FeltCm < 0.5 * VoxelOceanWaves::Deep)
        {
            // (Stage W6h: over the lower of the felt depth and the terrain's own, as the hook.)
            const double Bottom = bBed ? FMath::Min(FeltCm, Site.BedDepth) : FeltCm;
            Sum.Height -= 100.0f * VoxelOceanSurfThinDrop((Sum.Height + (float)Bottom) * 0.01f, 0.0f);
        }
        const float3 Up((float)Site.Up.X, (float)Site.Up.Y, (float)Site.Up.Z);
        const FVoxelOceanFFTResult W = VoxelOceanFFTFinish(Sum, Up);
        R.Height = W.Height;
        R.Displacement = Site.Up * (double)W.Height + FVector(W.Displacement.x, W.Displacement.y, W.Displacement.z);
        R.Normal = FVector(W.Normal.x, W.Normal.y, W.Normal.z).GetSafeNormal();
        R.Velocity = FVector(W.Velocity.x, W.Velocity.y, W.Velocity.z);
        R.Jacobian = W.Jacobian;
        R.HorizontalGradient[0] = FVector(Sum.J11, Sum.J12, Sum.J13);
        R.HorizontalGradient[1] = FVector(Sum.J12, Sum.J22, Sum.J23);
        R.HorizontalGradient[2] = FVector(Sum.J13, Sum.J23, Sum.J33);
    }
}

void FVoxelOceanWavesFFT::GetShaderParams(const FVoxelOceanWaveContext& Context, const FVector& Reference,
    const FTransform& PlanetToWorld, FVoxelOceanWaveShaderParams& Out) const
{
    Out = FVoxelOceanWaveShaderParams();
    // x: the chop (stage W3z: the foam's unresolved compression, VoxelOceanFoam.ush), y: the weather's
    // way through its interval, z: the headings' turn (VoxelOceanSeaState.ush).
    const FVoxelOceanWeatherTable* Weather = Context.Weather.Get();
    Out.Shape = FVector4f((float)FMath::Max(Settings.Chop, 0.0), Weather ? (float)Weather->Alpha(Context.Time) : 0.0f, Weather ? (float)Weather->Fade(Context.Time) : 1.0f, 0.0f);
    Out.FFT = FVector4f(1.0f, (float)Charts.N, (float)FMath::Max(Settings.DetileCell, 0.0), (float)DetileSeed);
    Out.RefHigh = FVector4f(VoxelOceanCharts::SplitHigh(Reference.X), VoxelOceanCharts::SplitHigh(Reference.Y), VoxelOceanCharts::SplitHigh(Reference.Z), Settings.MipBias);
    Out.RefLow = FVector4f(VoxelOceanCharts::SplitLow(Reference.X), VoxelOceanCharts::SplitLow(Reference.Y), VoxelOceanCharts::SplitLow(Reference.Z), (float)MaxHeightCm);
    const FQuat Q = PlanetToWorld.GetRotation();
    Out.Rotation = FVector4f((float)Q.X, (float)Q.Y, (float)Q.Z, (float)Q.W);
    Out.Tiles = Tiles;
    // The shore (stage O8; VoxelOceanShoalSettingsOf, VoxelOceanShoreAt).
    Out.States = FVector4f((float)FMath::Max(Context.ShoreCells, 0), (float)FMath::Max(Settings.ShoreBreaking, 0.0),
        (float)FMath::Max(Settings.ShoreChopGrowth, 0.0), (float)FMath::Max(Settings.ShoreWarp, 0.0));
    Out.SlopeVar = FVector4f((float)FMath::Max(Settings.ShoreFade, 0.0), (float)FMath::Max(Settings.ShoreMinSlope, 1.0e-4),
        (float)FMath::Max(Context.ShoreReachM, 0.0), VOF::SurfClock(Context.Time));
    // The surf (stage O9a; VoxelOceanSurfSettingsOf): the chart table's rows 15, 16 and 18.
    {
        VoxelShader::float4 Row0, Row1, Row2;
        VOF::SurfRows(Settings, Row0, Row1, Row2);
        Out.Surf[0] = FVector4f(Row0.x, Row0.y, Row0.z, Row0.w);
        Out.Surf[1] = FVector4f(Row1.x, Row1.y, Row1.z, Row1.w);
        Out.Surf[2] = FVector4f(Row2.x, Row2.y, Row2.z, Row2.w);
    }
    // The field's texel-space expansion around the reference (VoxelOceanShore.ush, NEAR THE CAMERA).
    if (Context.ShoreCells > 0)
    {
        float Frame[28];
        VoxelOceanShoreCore::FrameOf(Reference.X, Reference.Y, Reference.Z, Context.ShoreCells, Frame);
        for (int32 i = 0; i < 7; i++)
            Out.ShoreFrame[i] = FVector4f(Frame[4 * i], Frame[4 * i + 1], Frame[4 * i + 2], Frame[4 * i + 3]);
        // The basins' margin (m) in row 9's spare z, for the drain (stage O9b3).
        Out.ShoreFrame[1].Z = (float)FMath::Max(Context.ShoreMarginM, 0.0);
    }
}
