#include "VoxelOceanProfile.h"

#include "AtmosphereGenerator.h"

namespace
{
    FAtmosphereDraw MakeDraw(const TCHAR* Path, EAtmosphereDrawDistribution Distribution, float Min, float Max)
    {
        FAtmosphereDraw Entry;
        Entry.Path = Path;
        Entry.Distribution = Distribution;
        Entry.Min = Min;
        Entry.Max = Max;
        return Entry;
    }
}

void UVoxelOceanProfile::AddStarterDraws()
{
#if WITH_EDITOR
    Modify();
#endif
    const auto HasPath = [this](const FString& Path)
        {
            return Draws.Draws.ContainsByPredicate([&Path](const FAtmosphereDraw& Entry) { return Entry.Path == Path; });
        };
    const auto HasGroup = [this](FName Name)
        {
            return Draws.Choices.ContainsByPredicate([Name](const FAtmosphereChoiceGroup& Group) { return Group.Name == Name; });
        };

    // The palettes: the water's colour (its albedo: what deep water looks like), the foam's.
    if (!HasGroup(TEXT("Water")))
    {
        FAtmosphereChoiceGroup& Group = Draws.Choices.AddDefaulted_GetRef();
        Group.Name = TEXT("Water");
        Group.ScaleMin = 0.85f;
        Group.ScaleMax = 1.15f;
        Group.Mutation = 0.08f;
        FAtmospherePaletteColour& Colour = Group.Colours.AddDefaulted_GetRef();
        Colour.Path = TEXT("WaterColor");
        Colour.Options = {
            FLinearColor(0.054f, 0.30f, 0.538f),   // the default teal
            FLinearColor(0.03f, 0.16f, 0.45f),     // deep ocean blue
            FLinearColor(0.10f, 0.48f, 0.56f),     // tropical shallows
            FLinearColor(0.09f, 0.40f, 0.26f),     // green, coastal
            FLinearColor(0.025f, 0.10f, 0.32f),    // navy
            FLinearColor(0.10f, 0.22f, 0.12f),     // murky olive
        };
    }
    if (!HasGroup(TEXT("Foam")))
    {
        FAtmosphereChoiceGroup& Group = Draws.Choices.AddDefaulted_GetRef();
        Group.Name = TEXT("Foam");
        Group.Mutation = 0.05f;
        FAtmospherePaletteColour& Colour = Group.Colours.AddDefaulted_GetRef();
        Colour.Path = TEXT("FoamAlbedo");
        Colour.Options = {
            FLinearColor(0.492f, 0.407f, 0.342f),  // GodotOceanWaves' beige
            FLinearColor(0.75f, 0.75f, 0.75f),     // white
            FLinearColor(0.62f, 0.66f, 0.70f),     // cool white
        };
    }

    if (!HasPath(TEXT("WaterTurbidity")))
        Draws.Draws.Add(MakeDraw(TEXT("WaterTurbidity"), EAtmosphereDrawDistribution::LogUniform, 0.015f, 0.08f));

    // The wave equalizer: one height for the whole spectrum, then its shape knot by knot.
    static const TCHAR* Knots[] = { TEXT("Gain25cm"), TEXT("Gain50cm"), TEXT("Gain1m"), TEXT("Gain2m"), TEXT("Gain4m"), TEXT("Gain8m"),
        TEXT("Gain16m"), TEXT("Gain32m"), TEXT("Gain64m"), TEXT("Gain128m"), TEXT("Gain256m"), TEXT("Gain512m"), TEXT("Gain1024m") };
    for (const TCHAR* Knot : Knots)
    {
        const FString Path = FString(TEXT("WaveEqualizer.")) + Knot;
        if (HasPath(Path)) continue;
        FAtmosphereDraw Height = MakeDraw(*Path, EAtmosphereDrawDistribution::Uniform, 1.0f, 2.83f);
        Height.Link = TEXT("OceanWaveHeight");
        Draws.Draws.Add(Height);
        FAtmosphereDraw Shape;
        Shape.Path = Path;
        Shape.Distribution = EAtmosphereDrawDistribution::Jitter;
        Shape.Amount = 0.2f;
        Shape.bRelative = true;
        Draws.Draws.Add(Shape);
    }

    if (!HasPath(TEXT("WaveChop")))
        Draws.Draws.Add(MakeDraw(TEXT("WaveChop"), EAtmosphereDrawDistribution::Uniform, 0.7f, 1.2f));
    if (!HasPath(TEXT("FoamWhitecap")))
    {
        // Higher seas break more: the threshold falls as the heights rise.
        FAtmosphereDraw Whitecap = MakeDraw(TEXT("FoamWhitecap"), EAtmosphereDrawDistribution::Uniform, 0.4f, 0.66f);
        Whitecap.Link = TEXT("OceanWaveHeight");
        Whitecap.bInvert = true;
        Draws.Draws.Add(Whitecap);
    }
    if (!HasPath(TEXT("AtmosphereMeanWind")))
        Draws.Draws.Add(MakeDraw(TEXT("AtmosphereMeanWind"), EAtmosphereDrawDistribution::LogUniform, 4.5f, 11.0f));
    if (!HasPath(TEXT("AtmosphereSwellShare")))
        Draws.Draws.Add(MakeDraw(TEXT("AtmosphereSwellShare"), EAtmosphereDrawDistribution::Uniform, 0.4f, 0.8f));
    if (!HasPath(TEXT("RainAmount")))
        Draws.Draws.Add(MakeDraw(TEXT("RainAmount"), EAtmosphereDrawDistribution::Uniform, 0.5f, 1.5f));
}

FVoxelOceanSettings VoxelOceanSeeding::Generate(int32 Seed, const FVoxelOceanSettings& Base, const UVoxelOceanProfile* Profile,
    const TArray<FString>& Locks, const FString& SweepPath, float SweepPosition, TArray<FAtmosphereDrawRecord>& OutReport)
{
    OutReport.Reset();
    FVoxelOceanSettings Out = Base;
    if (!Profile) return Out;
    FAtmosphereGenerateOptions Options;
    Options.Locks = Locks;
    Options.SweepPath = SweepPath;
    Options.SweepPosition = SweepPosition;
    AtmosphereHarness::ApplyDrawSet(Seed, AtmosphereHarness::EStream::External, FVoxelOceanSettings::StaticStruct(), &Out,
        Profile->Draws, Options, TEXT("Ocean."), OutReport);
    for (const FString& Path : Locks)
    {
        FAtmosphereDrawRecord& Line = OutReport.AddDefaulted_GetRef();
        Line.Path = TEXT("Ocean.") + Path;
        Line.Source = EAtmosphereDrawSource::Lock;
    }
    return Out;
}
