#include "PlanetAtmosphereActor.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/PostProcessComponent.h"
#include "Engine/VolumeTexture.h"
#include "Engine/TextureRenderTarget2DArray.h"
#include "GasGiantSimSubsystem.h"
#include "GasGiantSimTypes.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MaterialTypes.h"

// ─────────────────────────────────────────────────────────────────────────────
// Default material and texture assets
//
// Constructor defaults for the soft references. Overridable per instance, so a
// relocated asset is a data edit rather than a code one.
// ─────────────────────────────────────────────────────────────────────────────

static const TCHAR* MatPath_Preprocess = TEXT("/VoxelPlugin/Material/MT_UCA_Preprocess_Inst.MT_UCA_Preprocess_Inst");
static const TCHAR* MatPath_Terrestrial = TEXT("/VoxelPlugin/Material/MT_UCA_Default_Inst.MT_UCA_Default_Inst");
static const TCHAR* MatPath_GasGiant = TEXT("/CloudAtmosphere/Material/MT_UGA_Default_Inst.MT_UGA_Default_Inst");
static const TCHAR* MatPath_Postprocess = TEXT("/VoxelPlugin/Material/MT_UCA_Postprocess_Inst.MT_UCA_Postprocess_Inst");

// ---------------------------------------------------------------------------
// Checked parameter pushes
//
// SetXParameterValue ON A NAME THE MATERIAL DOES NOT HAVE IS A SILENT NO-OP.
// No warning, no log, no return value -- the handle simply stops working, which
// reads as a shader bug and costs an afternoon. Every silent failure in this
// system so far has been a misspelled or renamed parameter.
//
// These wrappers check every push, editor-only, and are otherwise the same
// call. The point is that a rename is caught on the next rebuild rather than on
// the next screenshot.
//
// WARNED ONCE PER NAME, BECAUSE THE PUSH RUNS EVERY TICK. Unfiltered this would
// be a line per missing parameter per frame, which is not a diagnostic but a
// flood that buries the next one.
//
// RebuildMaterialInstances CLEARS THE FILTER, so the button in the details panel
// is the retrigger. Only the ACTIVE model's parameters are pushed, so covering
// both means pressing it, flipping PlanetType, and pressing it again.
// ---------------------------------------------------------------------------

#if WITH_EDITOR
static TSet<FString> GWarnedMaterialParameters;

// THE PARAMETER SET, NOT A PARAMETER VALUE. GetXParameterValue answers "is this
// overridden anywhere in the chain", which is false for a parameter that exists
// in the base material and has never been touched in the instance -- a true
// answer to a question nobody asked, and a false alarm for the one that matters.
// GetAllXParameterInfo enumerates what the material HAS.
//
// Cached per MID because the enumeration allocates, and cleared alongside the
// warning filter so a rebuilt material is re-read rather than remembered.
static TMap<const UMaterialInstanceDynamic*, TSet<FName>> GKnownScalarNames;
static TMap<const UMaterialInstanceDynamic*, TSet<FName>> GKnownVectorNames;
static TMap<const UMaterialInstanceDynamic*, TSet<FName>> GKnownTextureNames;

enum class EAtmoParamKind : uint8 { Scalar, Vector, Texture };

static bool MaterialHasParameter(UMaterialInstanceDynamic* MID, EAtmoParamKind Kind, FName Name)
{
    TMap<const UMaterialInstanceDynamic*, TSet<FName>>& Cache =
        Kind == EAtmoParamKind::Scalar ? GKnownScalarNames
        : Kind == EAtmoParamKind::Vector ? GKnownVectorNames
        : GKnownTextureNames;

    TSet<FName>* Known = Cache.Find(MID);

    if (!Known)
    {
        TArray<FMaterialParameterInfo> Infos;
        TArray<FGuid> Guids;

        switch (Kind)
        {
        case EAtmoParamKind::Scalar:  MID->GetAllScalarParameterInfo(Infos, Guids); break;
        case EAtmoParamKind::Vector:  MID->GetAllVectorParameterInfo(Infos, Guids); break;
        default:                      MID->GetAllTextureParameterInfo(Infos, Guids); break;
        }

        Known = &Cache.Add(MID);

        for (const FMaterialParameterInfo& Info : Infos)
        {
            Known->Add(Info.Name);
        }
    }

    return Known->Contains(Name);
}

static void WarnMissingParameter(const UMaterialInstanceDynamic* MID, const TCHAR* Kind, FName Name)
{
    const FString Key = FString::Printf(TEXT("%s.%s"),
        MID ? *MID->GetName() : TEXT("null"), *Name.ToString());

    if (GWarnedMaterialParameters.Contains(Key))
    {
        return;
    }

    GWarnedMaterialParameters.Add(Key);

    UE_LOG(LogTemp, Warning,
        TEXT("PlanetAtmosphereActor: material '%s' has no %s parameter '%s' -- push ignored"),
        MID ? *MID->GetName() : TEXT("null"), Kind, *Name.ToString());
}
#endif

static void SetScalarChecked(UMaterialInstanceDynamic* MID, FName Name, float Value)
{
    if (!MID) return;

#if WITH_EDITOR
    if (!MaterialHasParameter(MID, EAtmoParamKind::Scalar, Name))
    {
        WarnMissingParameter(MID, TEXT("scalar"), Name);
    }
#endif

    MID->SetScalarParameterValue(Name, Value);
}

static void SetVectorChecked(UMaterialInstanceDynamic* MID, FName Name, const FLinearColor& Value)
{
    if (!MID) return;

#if WITH_EDITOR
    if (!MaterialHasParameter(MID, EAtmoParamKind::Vector, Name))
    {
        WarnMissingParameter(MID, TEXT("vector"), Name);
    }
#endif

    MID->SetVectorParameterValue(Name, Value);
}

static void SetTextureChecked(UMaterialInstanceDynamic* MID, FName Name, UTexture* Value)
{
    if (!MID) return;

#if WITH_EDITOR
    if (!MaterialHasParameter(MID, EAtmoParamKind::Texture, Name))
    {
        WarnMissingParameter(MID, TEXT("texture"), Name);
    }
#endif

    MID->SetTextureParameterValue(Name, Value);
}

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

APlanetAtmosphereActor::APlanetAtmosphereActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    AtmosphereRoot = CreateDefaultSubobject<USceneComponent>(TEXT("AtmosphereRoot"));
    SetRootComponent(AtmosphereRoot);

    // Default radius = max(OceanRadius, PlanetRadius) at planet defaults:
    // PlanetRadius(100M) + SeaLevel(0.5) * NoiseAmplitude(15M) = 107,500,000 cm
    SetActorScale3D(FVector(107500000.0));

    // Set on the CDO rather than left to member initialisers, so the details
    // panel's reset-to-default gives each type ITS defaults. Member
    // initialisers can only serve one of the two.
    // The terrestrial sets are the member initialisers; the gas giant sets are
    // deltas against them, per group. Everything not overridden is identical
    // today and free to diverge -- carrying two instances is what makes that a
    // value edit rather than a code change.
    GasGiantGeometry = FAtmosphereGeometryParams::MakeGasGiantDefaults();
    GasGiantAtmosphereScattering = FAtmosphereAirScatteringParams::MakeGasGiantDefaults();
    GasGiantCloudScattering = FAtmosphereCloudScatteringParams::MakeGasGiantDefaults();
    GasGiantRaymarch = FAtmosphereRaymarchParams::MakeGasGiantDefaults();

    PreprocessMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_Preprocess));
    TerrestrialMarchMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_Terrestrial));
    GasGiantMarchMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_GasGiant));
    PostprocessMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_Postprocess));

    // Default source assets, so both models display something in the details
    // panel and a fresh actor renders. A deck with no volumes is not a subtle
    // failure -- the carves and the erosion both go to their neutral values and
    // the deck comes out as a smooth shell.
    static ConstructorHelpers::FObjectFinder<UVolumeTexture> DefaultCloudTexture(
        TEXT("/VoxelPlugin/VolumeTextures/Textures/VT_PerlinWorley_Balanced"));
    if (DefaultCloudTexture.Succeeded())
    {
        Terrestrial.CloudVolumeTexture = DefaultCloudTexture.Object;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("PlanetAtmosphereActor: Failed to load default cloud volume texture"));
    }

    static ConstructorHelpers::FObjectFinder<UVolumeTexture> DefaultDetailVolume(
        TEXT("/CloudAtmosphere/Noise/VT_PerlinWorley_S8_128"));
    if (DefaultDetailVolume.Succeeded())
    {
        GasGiantDeck.DetailVolume = DefaultDetailVolume.Object;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("PlanetAtmosphereActor: Failed to load default deck detail volume"));
    }

    static ConstructorHelpers::FObjectFinder<UVolumeTexture> DefaultStructureVolume(
        TEXT("/UniverseNoisePack/128/VT_Worley_F1_S8"));
    if (DefaultStructureVolume.Succeeded())
    {
        GasGiantDeck.StructureVolume = DefaultStructureVolume.Object;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("PlanetAtmosphereActor: Failed to load default deck structure volume"));
    }

    static ConstructorHelpers::FObjectFinder<UGasGiantSimConfig> DefaultSimConfig(
        TEXT("/CloudAtmosphere/NoiseRecipes/GasGiantSimScratch"));
    if (DefaultSimConfig.Succeeded())
    {
        Simulation.Config = DefaultSimConfig.Object;
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("PlanetAtmosphereActor: Failed to load default sim config"));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::BeginPlay()
{
    Super::BeginPlay();

    if (PlanetType == EPlanetAtmosphereType::GasGiant &&
        Simulation.bStartOnBeginPlay)
    {
        StartGasGiantSimulation();
    }
}

void APlanetAtmosphereActor::Destroyed()
{
    DestroyChildActors();
    Super::Destroyed();
}

void APlanetAtmosphereActor::BeginDestroy()
{
    DestroyChildActors();
    Super::BeginDestroy();
}

void APlanetAtmosphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (!GetWorld() || GetWorld()->IsPreviewWorld()) return;

    if (!bInitialized)
    {
        bPendingInitialize = true;
    }
}

void APlanetAtmosphereActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (bPendingInitialize)
    {
        bPendingInitialize = false;
        Initialize();
    }

    if (bInitialized)
    {
        UpdateMaterialParameters();
        UpdateLightFromRotation();
    }
}

#if WITH_EDITOR
void APlanetAtmosphereActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);

    const FName Changed = PropertyChangedEvent.GetPropertyName();

    // PlanetType selects the material, not just the parameter set, so it cannot
    // take effect through the per-tick sweep alone.
    const bool bTypeChanged =
        Changed == GET_MEMBER_NAME_CHECKED(APlanetAtmosphereActor, PlanetType) ||
        Changed == GET_MEMBER_NAME_CHECKED(APlanetAtmosphereActor, TerrestrialMarchMaterial) ||
        Changed == GET_MEMBER_NAME_CHECKED(APlanetAtmosphereActor, GasGiantMarchMaterial);

    if (bInitialized && bTypeChanged)
    {
        RebuildMaterialInstances();
        return;
    }

    if (bInitialized)
    {
        UpdateMaterialParameters();
        UpdateLightFromRotation();
    }
}

bool APlanetAtmosphereActor::CanEditChange(const FProperty* InProperty) const
{
    if (!Super::CanEditChange(InProperty))
        return false;

    if (bIsPlanetOwned && InProperty)
    {
        const FName PropName = InProperty->GetFName();
        // Lock location and scale — driven by the planet.
        // Rotation remains editable (controls light direction).
        if (PropName == TEXT("RelativeLocation") || PropName == TEXT("RelativeScale3D"))
            return false;
    }

    return true;
}

void APlanetAtmosphereActor::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
}

void APlanetAtmosphereActor::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
}

void APlanetAtmosphereActor::PostEditMove(bool bFinished)
{
    if (bIsPlanetOwned)
    {
        // Snap location back — it follows the planet. Rotation is intentionally left alone
        // (controls light direction). Scale is absolute and planet-driven.
        if (USceneComponent* Root = GetRootComponent())
        {
            Root->SetRelativeLocation(FVector::ZeroVector);
        }
    }
    Super::PostEditMove(bFinished);
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Planet integration
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport)
{
    if (!bIsPlanetOwned) return;

    if (USceneComponent* Root = GetRootComponent())
    {
        // Lock location and scale. Rotation is intentionally left alone (light direction).
        // Using _Direct setters avoids firing TransformUpdated recursively.
        bool bLocationDirty = !Root->GetRelativeLocation().IsNearlyZero(0.01);
        bool bScaleDirty = !Root->GetComponentScale().Equals(PlanetDrivenScale, 0.01);

        if (bLocationDirty || bScaleDirty)
        {
            Root->SetRelativeLocation_Direct(FVector::ZeroVector);
            Root->SetRelativeScale3D_Direct(PlanetDrivenScale);
            Root->UpdateComponentToWorld();
        }
    }
}

void APlanetAtmosphereActor::InitializeFromPlanet(USceneComponent* InAttachParent,
    FVector InScale)
{
    bIsPlanetOwned = true;

    // Apply scale BEFORE binding the transform guard — prevents the old guard
    // from reverting a scale change that the planet actor intends.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.RemoveAll(this);

    if (!InScale.IsZero())
        SetActorScale3D(InScale);

    PlanetDrivenScale = GetActorScale3D();

    // Re-bind the guard now that PlanetDrivenScale reflects the new scale.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.AddUObject(this, &APlanetAtmosphereActor::OnTransformUpdated);

    // Planet actor handles spawn + attach. Just run our init.
    bPendingInitialize = false;
    Initialize();
}

// ─────────────────────────────────────────────────────────────────────────────
// Initialize
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::Initialize()
{
    SpawnChildActors();
    CreateMaterialInstances();
    UpdateMaterialParameters();
    UpdateLightFromRotation();
    bInitialized = true;
}

void APlanetAtmosphereActor::RebuildMaterialInstances()
{
#if WITH_EDITOR
    // The parameter-check retrigger. Cleared before the push, so every missing
    // name reports again rather than staying silent from the first run.
    GWarnedMaterialParameters.Reset();
    GKnownScalarNames.Reset();
    GKnownVectorNames.Reset();
    GKnownTextureNames.Reset();
#endif

    CreateMaterialInstances();
    UpdateMaterialParameters();
}

// ─────────────────────────────────────────────────────────────────────────────
// Child actor management
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::SpawnChildActors()
{
    UWorld* World = GetWorld();
    if (!World) return;

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    // --- Post-Process Volume ---
    if (!PostProcessVolume)
    {
        PostProcessVolume = World->SpawnActor<APostProcessVolume>(
            APostProcessVolume::StaticClass(),
            GetActorTransform(),
            SpawnParams);

        if (PostProcessVolume)
        {
            PostProcessVolume->bUnbound = true;
            PostProcessVolume->BlendWeight = 1.0f;

            if (USceneComponent* PPRoot = PostProcessVolume->GetRootComponent())
            {
                PPRoot->AttachToComponent(AtmosphereRoot,
                    FAttachmentTransformRules::KeepWorldTransform);
            }
        }
    }

    // --- Directional Light ---
    if (!SunLight)
    {
        SunLight = World->SpawnActor<ADirectionalLight>(
            ADirectionalLight::StaticClass(),
            GetActorTransform(),
            SpawnParams);

        if (SunLight)
        {
            SunLight->GetComponent()->SetMobility(EComponentMobility::Movable);

            if (USceneComponent* LightRoot = SunLight->GetRootComponent())
            {
                LightRoot->AttachToComponent(AtmosphereRoot,
                    FAttachmentTransformRules::KeepWorldTransform);
            }
        }
    }
}

void APlanetAtmosphereActor::DestroyChildActors()
{
    // Released before the actor goes, or a pooled planet leaves the sim
    // stepping with nothing sampling it.
    if (bStartedSimulation)
    {
        if (UWorld* World = GetWorld())
        {
            if (UGasGiantSimSubsystem* Sim = World->GetSubsystem<UGasGiantSimSubsystem>())
            {
                Sim->StopSimulation();
            }
        }
        bStartedSimulation = false;
    }

    if (PostProcessVolume)
    {
        PostProcessVolume->Destroy();
        PostProcessVolume = nullptr;
    }
    if (SunLight)
    {
        SunLight->Destroy();
        SunLight = nullptr;
    }
    MID_Preprocess = nullptr;
    MID_Atmosphere = nullptr;
    MID_Postprocess = nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// Material instances
// ─────────────────────────────────────────────────────────────────────────────

UMaterialInterface* APlanetAtmosphereActor::LoadMaterialAsset(const TSoftObjectPtr<UMaterialInterface>& Ref, const TCHAR* Label)
{
    if (Ref.IsNull())
    {
        UE_LOG(LogTemp, Warning, TEXT("PlanetAtmosphereActor: %s material is unset."), Label);
        return nullptr;
    }

    UMaterialInterface* Loaded = Ref.LoadSynchronous();
    if (!Loaded)
    {
        UE_LOG(LogTemp, Warning, TEXT("PlanetAtmosphereActor: %s material failed to load from '%s'."),
            Label, *Ref.ToSoftObjectPath().ToString());
    }
    return Loaded;
}

void APlanetAtmosphereActor::CreateMaterialInstances()
{
    if (!PostProcessVolume) return;

    // THE MODEL IS CHOSEN ONCE, HERE, and recorded in BuiltType. The parameter
    // sweep dispatches on BuiltType rather than PlanetType so a type change
    // that has not been rebuilt yet cannot push one model's parameters at the
    // other model's material, which would do nothing and log nothing.
    const bool bGasGiant = (PlanetType == EPlanetAtmosphereType::GasGiant);

    UMaterialInterface* BasePre = LoadMaterialAsset(PreprocessMaterial, TEXT("Preprocess"));
    UMaterialInterface* BaseAtmo = bGasGiant
        ? LoadMaterialAsset(GasGiantMarchMaterial, TEXT("Gas giant march"))
        : LoadMaterialAsset(TerrestrialMarchMaterial, TEXT("Terrestrial march"));
    UMaterialInterface* BasePost = LoadMaterialAsset(PostprocessMaterial, TEXT("Postprocess"));

    if (!BasePre || !BaseAtmo || !BasePost)
    {
        return;
    }

    MID_Preprocess = UMaterialInstanceDynamic::Create(BasePre, this, TEXT("MID_Preprocess"));
    MID_Atmosphere = UMaterialInstanceDynamic::Create(BaseAtmo, this, TEXT("MID_Atmosphere"));
    MID_Postprocess = UMaterialInstanceDynamic::Create(BasePost, this, TEXT("MID_Postprocess"));

    BuiltType = PlanetType;

    // Order is the pipeline order: preprocess, march, composite. Rebuilt rather
    // than assigned by index, so a stale instance cannot survive a swap and
    // write the same UserSceneTexture as its replacement.
    FPostProcessSettings& Settings = PostProcessVolume->Settings;
    Settings.WeightedBlendables.Array.Empty();
    Settings.WeightedBlendables.Array.Add(FWeightedBlendable(1.0f, MID_Preprocess));
    Settings.WeightedBlendables.Array.Add(FWeightedBlendable(1.0f, MID_Atmosphere));
    Settings.WeightedBlendables.Array.Add(FWeightedBlendable(1.0f, MID_Postprocess));
}

// ─────────────────────────────────────────────────────────────────────────────
// Material parameter update
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::UpdateMaterialParameters()
{
    if (!MID_Atmosphere || !MID_Postprocess) return;

    const FVector PlanetCenter = GetActorLocation();
    const float PlanetRadius = static_cast<float>(GetActorScale3D().GetMax());

    // Light direction from relative rotation — treated as world-space direction
    // regardless of parent rotation. The user/gizmo sets relative rotation directly.
    const FVector LightDir = GetRootComponent()->GetRelativeRotation().Vector();

    // Selected once and threaded through, so the shell radius the composite
    // blurs against cannot come from a different instance than the one the
    // march planned with.
    const FAtmosphereCommonView Common = GetCommonParams();

    ApplyCommonParams(Common, PlanetRadius, PlanetCenter, LightDir);

    if (BuiltType == EPlanetAtmosphereType::GasGiant)
    {
        ApplyGasGiantParams(Common, PlanetRadius);
    }
    else
    {
        ApplyTerrestrialParams(Common);
    }

    // --- Postprocess (slot 2) ---
    //
    // One material for both models, so every blur parameter comes from
    // Environment and none of it is per-model.
    //
    // EVERY ARGUMENT Atmo_Composite TAKES IS PUSHED FROM HERE, and nothing else
    // is. The pass reads the atmosphere buffer and the depth buffer; it does not
    // need to know where the planet is, which is why the centre and radius that
    // used to go with these are gone.

    SetScalarChecked(MID_Postprocess, TEXT("Blur Radius"), static_cast<float>(Composite.BlurRadius));
    SetScalarChecked(MID_Postprocess, TEXT("Blur Falloff Factor"), Composite.BlurFalloffFactor);
    SetScalarChecked(MID_Postprocess, TEXT("Depth Sharpness"), Composite.DepthSharpness);
    SetScalarChecked(MID_Postprocess, TEXT("Depth Tap Scale"), Composite.DepthTapScale);
    SetScalarChecked(MID_Postprocess, TEXT("Blur Weight"), Composite.BlurWeight);
}

void APlanetAtmosphereActor::ApplyCommonParams(const FAtmosphereCommonView& Common, float PlanetRadius,
    const FVector& PlanetCenter, const FVector& LightDir)
{
    SetVectorChecked(MID_Atmosphere, TEXT("Planet Center"),
        FLinearColor(PlanetCenter.X, PlanetCenter.Y, PlanetCenter.Z, 0.0f));
    SetScalarChecked(MID_Atmosphere, TEXT("Planet Radius"), PlanetRadius);
    SetScalarChecked(MID_Atmosphere, TEXT("Atmosphere Height Scale"), Common.Geometry.HeightScale);
    SetScalarChecked(MID_Atmosphere, TEXT("Atmosphere Floor Offset"), Common.Geometry.FloorOffset);

    SetVectorChecked(MID_Atmosphere, TEXT("Light Direction"),
        FLinearColor(LightDir.X, LightDir.Y, LightDir.Z, 0.0f));
    SetVectorChecked(MID_Atmosphere, TEXT("Light Color"), LightColor);

    SetVectorChecked(MID_Atmosphere, TEXT("Rayleigh Beta"), Common.AirScattering.RayleighBeta);

    SetVectorChecked(MID_Atmosphere, TEXT("Mie Beta"), Common.AirScattering.MieBeta);

    SetScalarChecked(MID_Atmosphere, TEXT("Mie G"), Common.AirScattering.MieG);
    SetVectorChecked(MID_Atmosphere, TEXT("Atmosphere Absorption Beta"), Common.AirScattering.AbsorptionBeta);

    SetScalarChecked(MID_Atmosphere, TEXT("Atmosphere Absorption Falloff"), Common.AirScattering.AbsorptionFalloff);
    SetVectorChecked(MID_Atmosphere, TEXT("Atmosphere Ambient"), Common.AirScattering.Ambient);

    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Ambient"), Common.CloudScattering.Ambient);
    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Phase Params"), Common.CloudScattering.PhaseParams);

    SetScalarChecked(MID_Atmosphere, TEXT("Atmosphere Steps"), Common.Raymarch.AtmosphereSteps);
    SetScalarChecked(MID_Atmosphere, TEXT("Atmosphere Light Steps"), Common.Raymarch.AtmosphereLightSteps);
    SetScalarChecked(MID_Atmosphere, TEXT("Step Scale Factor"), Common.Raymarch.StepScaleFactor);
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Steps"), Common.Raymarch.CloudSteps);
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Light Steps"), Common.Raymarch.CloudLightSteps);
    SetScalarChecked(MID_Atmosphere, TEXT("Light Step Texels"), Common.Raymarch.LightStepTexels);
    SetScalarChecked(MID_Atmosphere, TEXT("View Step Pixels"), Common.Raymarch.ViewStepPixels);
}

void APlanetAtmosphereActor::ApplyTerrestrialParams(const FAtmosphereCommonView& Common)
{
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Outer Height Scale"),
        Terrestrial.GetOuterHeightScale(Common.Geometry.HeightScale));
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Inner Height Scale"),
        Terrestrial.GetInnerHeightScale(Common.Geometry.HeightScale));

    if (Terrestrial.CloudVolumeTexture)
    {
        SetTextureChecked(MID_Atmosphere, TEXT("Cloud Volume Texture"), Terrestrial.CloudVolumeTexture);
    }

    SetVectorChecked(MID_Atmosphere, TEXT("Animation Weights"), Terrestrial.AnimationWeights);
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Coverage"), Terrestrial.CloudCoverage);
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Density Multiplier"), Terrestrial.CloudDensityMultiplier);
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Height Curve Min"), Terrestrial.CloudHeightCurveMin);
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Height Curve Max"), Terrestrial.CloudHeightCurveMax);
    SetScalarChecked(MID_Atmosphere, TEXT("Cloud Noise Frequency"), Terrestrial.CloudNoiseFrequency);
    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Noise Weights"), Terrestrial.CloudNoiseWeights);
    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Noise Invert"), Terrestrial.CloudNoiseInvert);
    SetScalarChecked(MID_Atmosphere, TEXT("Detail Noise Frequency"), Terrestrial.GetDetailNoiseFrequency());
    SetVectorChecked(MID_Atmosphere, TEXT("Detail Noise Weights"), Terrestrial.DetailNoiseWeights);
    SetVectorChecked(MID_Atmosphere, TEXT("Detail Noise Invert"), Terrestrial.DetailNoiseInvert);
    SetScalarChecked(MID_Atmosphere, TEXT("Detail Erode Strength"), Terrestrial.DetailErodeStrength);

    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Beta"), Terrestrial.CloudBeta);
    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Absorption Beta"), Terrestrial.CloudAbsorptionBeta);
}

void APlanetAtmosphereActor::ApplyGasGiantParams(const FAtmosphereCommonView& Common, float PlanetRadius)
{
    // THE MATERIAL COMPOSES THE SHADER'S FLOAT4s FROM INDIVIDUAL SCALARS via
    // Convert nodes, for instance-editor clarity. So there is no "Profile" or
    // "Scales" parameter to set -- pushing one does nothing and logs nothing.
    // The derivations still happen here; only the last hop is per-component.

    // The flow target is created at runtime, so it arrives through the config
    // rather than as a migrated asset. Its sampler must be WRAP U, CLAMP V:
    // the sim grid is a cylinder, and wrapping V joins the north pole to the
    // south, which reads as a simulation bug rather than a sampler one.
    if (Simulation.Config && Simulation.Config->FlowTarget)
    {
        SetTextureChecked(MID_Atmosphere, TEXT("flowField"), Simulation.Config->FlowTarget);
    }

    if (GasGiantDeck.DetailVolume)
    {
        SetTextureChecked(MID_Atmosphere, TEXT("detailVolume"), GasGiantDeck.DetailVolume);
    }

    if (GasGiantDeck.StructureVolume)
    {
        SetTextureChecked(MID_Atmosphere, TEXT("structureVolume"), GasGiantDeck.StructureVolume);
    }

    // -- Profile ------------------------------------------------------------
    //
    // AtmosphereThickness is the only absolute length the field reads, and the
    // unit every height in the deck is a fraction of. The deck has no shell of
    // its own: the anchors place it inside the air, so sizing the air does not
    // resize the deck.
    //
    // GradientThickness is the span the density profile occupies below each
    // column's own top; DeckBackstop is the backstop under it, and also the fine
    // band's lower edge, so the march's step sizing follows the anchors rather
    // than the extinction.

    const FLinearColor Profile = GasGiantDeck.GetProfile(PlanetRadius, Common.Geometry.HeightScale);

    SetScalarChecked(MID_Atmosphere, TEXT("AtmosphereThickness"), Profile.R);
    SetScalarChecked(MID_Atmosphere, TEXT("DeckBackstop"), Profile.G);
    SetScalarChecked(MID_Atmosphere, TEXT("VortexThreshold"), Profile.B);
    SetScalarChecked(MID_Atmosphere, TEXT("GradientThickness"), Profile.A);

    // -- Scales -------------------------------------------------------------

    const FLinearColor Scales = GasGiantDeck.GetScales();

    SetScalarChecked(MID_Atmosphere, TEXT("DetailScale"), Scales.R);
    SetScalarChecked(MID_Atmosphere, TEXT("StructureScale"), Scales.G);
    SetScalarChecked(MID_Atmosphere, TEXT("DetailWarpInherit"), Scales.B);
    SetScalarChecked(MID_Atmosphere, TEXT("StructureWarpInherit"), Scales.A);

    // -- Warps --------------------------------------------------------------

    const FLinearColor Warps = GasGiantDeck.GetWarps();

    SetScalarChecked(MID_Atmosphere, TEXT("WarpTime"), Warps.R);
    SetScalarChecked(MID_Atmosphere, TEXT("DetailWarp"), Warps.G);
    SetScalarChecked(MID_Atmosphere, TEXT("BandBias"), Warps.B);
    SetScalarChecked(MID_Atmosphere, TEXT("TurbulenceFloor"), Warps.A);

    // -- Detail weights -----------------------------------------------------
    //
    // xyz are renormalized by their sum in the shader, so changing the balance
    // between them does not change how much cloud there is.

    // Each layer's Worley ladder, coarse to fine, plus its own amount. The
    // ladder is renormalized shader-side, so these set the spectrum and the w
    // sets the strength.
    const FLinearColor DetailNoise = GasGiantDeck.GetDetailNoise();
    const FLinearColor StructureNoise = GasGiantDeck.GetStructureNoise();

    SetVectorChecked(MID_Atmosphere, TEXT("DetailNoise"), DetailNoise);
    SetVectorChecked(MID_Atmosphere, TEXT("StructureNoise"), StructureNoise);

    SetScalarChecked(MID_Atmosphere, TEXT("EdgeBias"), GasGiantDeck.EdgeBias);

    // -- Relief -------------------------------------------------------------
    //
    // Fractions of GradientThickness, so relief moves the profile rather than
    // stretching it. GetTopMax() sums them the same way GG_TopBounds does and
    // the cull radius follows it; nothing bounds them below, since the backstop
    // catches whatever they cut.

    const FLinearColor Relief = GasGiantDeck.GetRelief();

    SetScalarChecked(MID_Atmosphere, TEXT("DeckTop"), Relief.R);
    SetScalarChecked(MID_Atmosphere, TEXT("BandRelief"), Relief.G);
    SetScalarChecked(MID_Atmosphere, TEXT("PressureRelief"), Relief.B);
    SetScalarChecked(MID_Atmosphere, TEXT("StormTowerRelief"), Relief.A);

    // -- Layers -------------------------------------------------------------

    SetScalarChecked(MID_Atmosphere, TEXT("DeckSlope"), GasGiantDeck.DeckSlope);

    const FLinearColor Crossfade = GasGiantDeck.GetCrossfade();

    // AUTHORED IN SIMULATED SECONDS, PUSHED IN REAL ONES. TimeScale is simulated
    // time per real second, so the flow's own motion speeds up with it -- and
    // noise the flow is supposed to be carrying has to speed up by the same
    // factor or the two visibly come apart the moment the sim speed is touched.
    //
    // A frozen sim freezes the crossfade with it, which is the right answer
    // rather than an edge case: nothing is advecting, so nothing should travel.
    const float SimTimeScale = Simulation.Config ? Simulation.Config->TimeScale : 1.0f;

    SetScalarChecked(MID_Atmosphere, TEXT("Crossfade Period"),
        Crossfade.R / FMath::Max(SimTimeScale, KINDA_SMALL_NUMBER));
    SetScalarChecked(MID_Atmosphere, TEXT("Crossfade Detail"), Crossfade.G);
    SetScalarChecked(MID_Atmosphere, TEXT("Crossfade Structure"), Crossfade.B);
    SetScalarChecked(MID_Atmosphere, TEXT("Structure Shadows"), Crossfade.A);

    // -- Fade ranges --------------------------------------------------------
    //
    // Atmosphere thicknesses from the camera. Composed into FadeRanges by the
    // material, like the other float4s.

    const FLinearColor FadeRanges = GasGiantDeck.GetFadeRanges();

    SetScalarChecked(MID_Atmosphere, TEXT("DetailFadeNear"), FadeRanges.R);
    SetScalarChecked(MID_Atmosphere, TEXT("DetailFadeFar"), FadeRanges.G);
    SetScalarChecked(MID_Atmosphere, TEXT("StructureFadeNear"), FadeRanges.B);
    SetScalarChecked(MID_Atmosphere, TEXT("StructureFadeFar"), FadeRanges.A);

    // -- Loose field scalars ------------------------------------------------

    SetScalarChecked(MID_Atmosphere, TEXT("BandSharpness"), GasGiantDeck.BandSharpness);
    SetScalarChecked(MID_Atmosphere, TEXT("ReliefThinning"), GasGiantDeck.ReliefThinning);
    SetScalarChecked(MID_Atmosphere, TEXT("DetailVertical"), GasGiantDeck.GetDetailVertical(Common.Geometry.HeightScale));
    SetScalarChecked(MID_Atmosphere, TEXT("StructureVertical"), GasGiantDeck.GetStructureVertical(Common.Geometry.HeightScale));
    SetScalarChecked(MID_Atmosphere, TEXT("DetailErosion"), GasGiantDeck.DetailErosion);
    SetScalarChecked(MID_Atmosphere, TEXT("DetailRelief"), GasGiantDeck.DetailRelief);
    SetScalarChecked(MID_Atmosphere, TEXT("StructureRelief"), GasGiantDeck.StructureRelief);
    SetScalarChecked(MID_Atmosphere, TEXT("StructureErosion"), GasGiantDeck.StructureErosion);
    SetScalarChecked(MID_Atmosphere, TEXT("ErosionDepth"), GasGiantDeck.ErosionDepth);
    SetScalarChecked(MID_Atmosphere, TEXT("DensityCurve"), GasGiantDeck.DensityCurve);
    SetScalarChecked(MID_Atmosphere, TEXT("RotationWeight"), GasGiantDeck.RotationWeight);

    // The sim's clock, not the world's. Requires the material's Time parameter
    // to feed the Custom node directly -- wired through a multiply against an
    // engine Time node, this value is ignored and the field advects against
    // world time, which diverges the moment the sim pauses or restores.
    SetScalarChecked(MID_Atmosphere, TEXT("Time"), GetGasGiantTime());

    // -- Local frame --------------------------------------------------------
    //
    // The planet's axes in world space, as three rows. The field is defined
    // with the spin axis on Z; the march runs world-oriented.

    const FVector AxisX = GetActorForwardVector();
    const FVector AxisY = GetActorRightVector();
    const FVector AxisZ = GetActorUpVector();

    SetVectorChecked(MID_Atmosphere, TEXT("localAxisX"), FLinearColor(AxisX.X, AxisX.Y, AxisX.Z, 0.0f));
    SetVectorChecked(MID_Atmosphere, TEXT("localAxisY"), FLinearColor(AxisY.X, AxisY.Y, AxisY.Z, 0.0f));
    SetVectorChecked(MID_Atmosphere, TEXT("localAxisZ"), FLinearColor(AxisZ.X, AxisZ.Y, AxisZ.Z, 0.0f));

    // -- Scattering ---------------------------------------------------------

    SetVectorChecked(MID_Atmosphere, TEXT("ScatterNeg"), GasGiantScatter.ScatterNegative);
    SetVectorChecked(MID_Atmosphere, TEXT("ScatterPos"), GasGiantScatter.ScatterPositive);
    SetVectorChecked(MID_Atmosphere, TEXT("ScatterBase"), GasGiantScatter.ScatterBase);
    SetScalarChecked(MID_Atmosphere, TEXT("BandScale"), GasGiantScatter.BandScale);

    // Terminator shaping. One float4 in the shader, four scalars here, because
    // they are tuned against each other: the softness sets the terminator's
    // width and the power crushes the tail the forward lobe leaks through it.
    const FLinearColor LobeParams = GasGiantScatter.GetLobeParams();

    SetScalarChecked(MID_Atmosphere, TEXT("TerminatorSoftness"), LobeParams.R);
    SetScalarChecked(MID_Atmosphere, TEXT("AmbientTerminator"), LobeParams.G);
    SetScalarChecked(MID_Atmosphere, TEXT("MieLobeDecay"), LobeParams.B);
    SetScalarChecked(MID_Atmosphere, TEXT("LobeShadowPower"), LobeParams.A);

    // Solved from DeckOpticalDepth against the path a vertical ray takes down a
    // column with no relief, so retuning the shell leaves the deck's opacity
    // where it was authored.
    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Beta"),
        GasGiantScatter.GetCloudBeta(GasGiantDeck.GetDeckBase(), GasGiantDeck.GetNominalFloor()));
    SetVectorChecked(MID_Atmosphere, TEXT("Cloud Absorption Beta"),
        GasGiantScatter.GetCloudAbsorptionBeta(GasGiantDeck.GetDeckBase(), GasGiantDeck.GetNominalFloor()));
}

// ─────────────────────────────────────────────────────────────────────────────
// Gas giant simulation
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::StartGasGiantSimulation()
{
    if (!Simulation.Config)
    {
        UE_LOG(LogTemp, Warning,
            TEXT("PlanetAtmosphereActor: gas giant with no SimConfig. The deck will render against "
                "an unbound flow field, which looks like flat horizontal stripes."));
        return;
    }

    UWorld* World = GetWorld();
    if (!World) return;

    UGasGiantSimSubsystem* Sim = World->GetSubsystem<UGasGiantSimSubsystem>();
    if (!Sim) return;

    Sim->StartSimulation(Simulation.Config);
    bStartedSimulation = true;
}

float APlanetAtmosphereActor::GetGasGiantTime() const
{
    if (const UWorld* World = GetWorld())
    {
        if (const UGasGiantSimSubsystem* Sim = World->GetSubsystem<UGasGiantSimSubsystem>())
        {
            return Sim->GetSimulatedTime();
        }
    }
    return 0.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Light update
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::OrientToStar(const FVector& StarWorldPos)
{
    // Point the atmosphere's forward at the star, then let the existing rotation->light
    // sync propagate it to the directional light + raymarch MIDs. If illumination ends
    // up inverted, negate ToStar: a directional light's forward is the *travel*
    // direction (away from the star), not the direction toward it.
    //
    // PITFALL for gas giants: this also rotates the planet's local frame, which
    // is what localAxisX/Y/Z carry. Aiming the actor at a moving star therefore
    // spins the deck's spin axis with it. A planet whose axis must stay fixed
    // needs the light on a separate transform from the field's frame.
    const FVector ToStar = StarWorldPos - GetActorLocation();
    if (ToStar.IsNearlyZero()) return;
    SetActorRotation(ToStar.Rotation());
    UpdateLightFromRotation();
}

void APlanetAtmosphereActor::UpdateLightFromRotation()
{
    if (!SunLight) return;

    UDirectionalLightComponent* LightComp = SunLight->GetComponent();
    if (!LightComp) return;

    // Directional light faces opposite the light direction vector
    const FVector LightDir = GetRootComponent()->GetRelativeRotation().Vector();
    const FRotator SunRotation = (-LightDir).Rotation();
    SunLight->SetActorRotation(SunRotation);

    // Extract color and intensity from LightColor.
    // RGB = normalized color, magnitude of RGB = intensity multiplier.
    const FVector ColorVec(LightColor.R, LightColor.G, LightColor.B);
    const float Magnitude = ColorVec.Size();

    if (Magnitude > KINDA_SMALL_NUMBER)
    {
        const FLinearColor NormalizedColor(
            LightColor.R / Magnitude,
            LightColor.G / Magnitude,
            LightColor.B / Magnitude, 1.0f);
        LightComp->SetLightColor(NormalizedColor);
        LightComp->SetIntensity(Magnitude);
    }
    else
    {
        LightComp->SetLightColor(FLinearColor::White);
        LightComp->SetIntensity(0.0f);
    }
}

void APlanetAtmosphereActor::SetAtmosphereActive(bool bActive)
{
    // bEnabled drops the volume out of the post-process chain entirely -- the ray march
    // stops. Unbound volumes affect the whole camera regardless of actor visibility, so
    // this is what makes a pooled/dormant planet cost zero atmosphere GPU.
    if (PostProcessVolume) PostProcessVolume->bEnabled = bActive;
}