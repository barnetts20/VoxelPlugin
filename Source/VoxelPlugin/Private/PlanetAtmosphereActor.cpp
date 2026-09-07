#include "PlanetAtmosphereActor.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/PostProcessComponent.h"
#include "Engine/VolumeTexture.h"
#include "Engine/TextureRenderTarget2DArray.h"
#include "GasGiantSimSubsystem.h"
#include "GasGiantSimTypes.h"

// ─────────────────────────────────────────────────────────────────────────────
// Default material and texture assets
//
// Constructor defaults for the soft references. Overridable per instance, so a
// relocated asset is a data edit rather than a code one.
// ─────────────────────────────────────────────────────────────────────────────

static const TCHAR* MatPath_Preprocess = TEXT("/VoxelPlugin/Material/MT_UCA_Preprocess_Inst.MT_UCA_Preprocess_Inst");
static const TCHAR* MatPath_Terrestrial = TEXT("/VoxelPlugin/Material/MT_UCA_Default_Inst.MT_UCA_Default_Inst");
static const TCHAR* MatPath_GasGiant = TEXT("/VoxelPlugin/Material/MT_UGA_Default_Inst.MT_UGA_Default_Inst");
static const TCHAR* MatPath_Postprocess = TEXT("/VoxelPlugin/Material/MT_UCA_Postprocess_Inst.MT_UCA_Postprocess_Inst");

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

    PreprocessMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_Preprocess));
    TerrestrialMarchMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_Terrestrial));
    GasGiantMarchMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_GasGiant));
    PostprocessMaterial = TSoftObjectPtr<UMaterialInterface>(FSoftObjectPath(MatPath_Postprocess));

    // Load default cloud volume texture so it displays in the editor details panel.
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
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::BeginPlay()
{
    Super::BeginPlay();

    if (PlanetType == EPlanetAtmosphereType::GasGiant &&
        GasGiantDeck.bStartSimulationOnBeginPlay)
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

    ApplySharedParams(PlanetRadius, PlanetCenter, LightDir);

    if (BuiltType == EPlanetAtmosphereType::GasGiant)
    {
        ApplyGasGiantParams(PlanetRadius);
    }
    else
    {
        ApplyTerrestrialParams();
    }

    // --- Postprocess (slot 2) ---

    MID_Postprocess->SetVectorParameterValue(TEXT("Atmosphere Center"),
        FLinearColor(PlanetCenter.X, PlanetCenter.Y, PlanetCenter.Z, 0.0f));
    MID_Postprocess->SetScalarParameterValue(TEXT("Atmosphere Radius"), Shared.GetAtmosphereRadius(PlanetRadius));
    MID_Postprocess->SetScalarParameterValue(TEXT("Blur Falloff Factor"), Shared.BlurFalloffFactor);
    MID_Postprocess->SetScalarParameterValue(TEXT("MaxW"), Shared.MaxBlurWeight);
    MID_Postprocess->SetScalarParameterValue(TEXT("MinW"), Shared.GetMinBlurWeight());
}

void APlanetAtmosphereActor::ApplySharedParams(float PlanetRadius, const FVector& PlanetCenter, const FVector& LightDir)
{
    MID_Atmosphere->SetVectorParameterValue(TEXT("Planet Center"),
        FLinearColor(PlanetCenter.X, PlanetCenter.Y, PlanetCenter.Z, 0.0f));
    MID_Atmosphere->SetScalarParameterValue(TEXT("Planet Radius"), PlanetRadius);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Height Scale"), Shared.AtmosphereHeightScale);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Floor Offset"), Shared.AtmosphereFloorOffset);

    MID_Atmosphere->SetVectorParameterValue(TEXT("Light Direction"),
        FLinearColor(LightDir.X, LightDir.Y, LightDir.Z, 0.0f));
    MID_Atmosphere->SetVectorParameterValue(TEXT("Light Color"), Shared.LightColor);

    MID_Atmosphere->SetVectorParameterValue(TEXT("Rayleigh Beta"), Shared.RayleighBeta);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Rayleigh Height"), Shared.RayleighHeight);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Mie Beta"), Shared.MieBeta);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Mie Height"), Shared.MieHeight);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Mie G"), Shared.MieG);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Atmosphere Absorption Beta"), Shared.AtmosphereAbsorptionBeta);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Absorption Height"), Shared.AtmosphereAbsorptionHeight);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Absorption Falloff"), Shared.AtmosphereAbsorptionFalloff);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Atmosphere Ambient"), Shared.AtmosphereAmbient);

    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Steps"), Shared.AtmosphereSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Light Steps"), Shared.AtmosphereLightSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Step Scale Factor"), Shared.StepScaleFactor);
}

void APlanetAtmosphereActor::ApplyTerrestrialParams()
{
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Outer Height Scale"),
        Terrestrial.GetOuterHeightScale(Shared.AtmosphereHeightScale));
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Inner Height Scale"),
        Terrestrial.GetInnerHeightScale(Shared.AtmosphereHeightScale));

    if (Terrestrial.CloudVolumeTexture)
    {
        MID_Atmosphere->SetTextureParameterValue(TEXT("Cloud Volume Texture"), Terrestrial.CloudVolumeTexture);
    }

    MID_Atmosphere->SetVectorParameterValue(TEXT("Animation Weights"), Terrestrial.AnimationWeights);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Coverage"), Terrestrial.CloudCoverage);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Density Multiplier"), Terrestrial.CloudDensityMultiplier);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Height Curve Min"), Terrestrial.CloudHeightCurveMin);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Height Curve Max"), Terrestrial.CloudHeightCurveMax);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Noise Frequency"), Terrestrial.CloudNoiseFrequency);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Noise Weights"), Terrestrial.CloudNoiseWeights);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Noise Invert"), Terrestrial.CloudNoiseInvert);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Detail Noise Frequency"), Terrestrial.GetDetailNoiseFrequency());
    MID_Atmosphere->SetVectorParameterValue(TEXT("Detail Noise Weights"), Terrestrial.DetailNoiseWeights);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Detail Noise Invert"), Terrestrial.DetailNoiseInvert);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Detail Erode Strength"), Terrestrial.DetailErodeStrength);

    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Beta"), Terrestrial.CloudBeta);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Absorption Beta"), Terrestrial.CloudAbsorptionBeta);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Ambient"), Terrestrial.CloudAmbient);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Phase Params"), Terrestrial.CloudPhaseParams);

    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Steps"), Terrestrial.CloudSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Light Steps"), Terrestrial.CloudLightSteps);
}

void APlanetAtmosphereActor::ApplyGasGiantParams(float PlanetRadius)
{
    // THE MATERIAL COMPOSES THE SHADER'S FLOAT4s FROM INDIVIDUAL SCALARS via
    // Convert nodes, for instance-editor clarity. So there is no "Profile" or
    // "Scales" parameter to set -- pushing one does nothing and logs nothing.
    // The derivations still happen here; only the last hop is per-component.

    // The flow target is created at runtime, so it arrives through the config
    // rather than as a migrated asset. Its sampler must be WRAP U, CLAMP V:
    // the sim grid is a cylinder, and wrapping V joins the north pole to the
    // south, which reads as a simulation bug rather than a sampler one.
    if (GasGiantDeck.SimConfig && GasGiantDeck.SimConfig->FlowTarget)
    {
        MID_Atmosphere->SetTextureParameterValue(TEXT("flowField"), GasGiantDeck.SimConfig->FlowTarget);
    }

    if (GasGiantDeck.PackedDetail)
    {
        MID_Atmosphere->SetTextureParameterValue(TEXT("packedDetail"), GasGiantDeck.PackedDetail);
    }

    // -- Profile ------------------------------------------------------------
    //
    // ShellThickness is the deck's depth in world units, solved so the highest
    // the deck can reach lands at DeckTopFraction of the atmosphere height. The
    // only absolute length in the field, and the one that silently turns the
    // deck into a film when authored against a scale-derived planet radius.

    const FLinearColor Profile = GasGiantDeck.GetProfile(PlanetRadius, Shared.AtmosphereHeightScale);

    MID_Atmosphere->SetScalarParameterValue(TEXT("ShellThickness"), Profile.R);
    MID_Atmosphere->SetScalarParameterValue(TEXT("DensityRamp"), Profile.G);
    MID_Atmosphere->SetScalarParameterValue(TEXT("VortexThreshold"), Profile.B);
    MID_Atmosphere->SetScalarParameterValue(TEXT("CoreDensity"), Profile.A);

    // -- Scales -------------------------------------------------------------

    const FLinearColor Scales = GasGiantDeck.GetScales();

    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailScale"), Scales.R);
    MID_Atmosphere->SetScalarParameterValue(TEXT("PackedNoise"), Scales.G);
    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailWarpInherit"), Scales.B);
    MID_Atmosphere->SetScalarParameterValue(TEXT("PackedWarpInherit"), Scales.A);

    // -- Warps --------------------------------------------------------------

    const FLinearColor Warps = GasGiantDeck.GetWarps();

    MID_Atmosphere->SetScalarParameterValue(TEXT("WarpTime"), Warps.R);
    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailWarp"), Warps.G);
    MID_Atmosphere->SetScalarParameterValue(TEXT("BandBias"), Warps.B);
    MID_Atmosphere->SetScalarParameterValue(TEXT("TurbulenceFloor"), Warps.A);

    // -- Detail weights -----------------------------------------------------
    //
    // xyz are renormalized by their sum in the shader, so changing the balance
    // between them does not change how much cloud there is.

    MID_Atmosphere->SetScalarParameterValue(TEXT("RidgeWeight"), GasGiantDeck.DetailWeights.R);
    MID_Atmosphere->SetScalarParameterValue(TEXT("FluffWeight"), GasGiantDeck.DetailWeights.G);
    MID_Atmosphere->SetScalarParameterValue(TEXT("WispWeight"), GasGiantDeck.DetailWeights.B);
    MID_Atmosphere->SetScalarParameterValue(TEXT("EdgeBias"), GasGiantDeck.DetailWeights.A);

    // -- Relief -------------------------------------------------------------
    //
    // Shell fractions. GetTopMax() sums these the same way GG_TopBounds does,
    // and the thickness above was solved against that sum -- so retuning any of
    // them keeps the cull radius where DeckTopFraction says it should be.

    MID_Atmosphere->SetScalarParameterValue(TEXT("DeckBaseHeight"), GasGiantDeck.Relief.R);
    MID_Atmosphere->SetScalarParameterValue(TEXT("BandRelief"), GasGiantDeck.Relief.G);
    MID_Atmosphere->SetScalarParameterValue(TEXT("PressureLift"), GasGiantDeck.Relief.B);
    MID_Atmosphere->SetScalarParameterValue(TEXT("StormTowerHeight"), GasGiantDeck.Relief.A);

    // -- Layers -------------------------------------------------------------

    MID_Atmosphere->SetScalarParameterValue(TEXT("FlowLayer"), static_cast<float>(GasGiantDeck.FlowLayer));
    MID_Atmosphere->SetScalarParameterValue(TEXT("DeepFlowLayer"), static_cast<float>(GasGiantDeck.DeepFlowLayer));
    MID_Atmosphere->SetScalarParameterValue(TEXT("DeckSlope"), GasGiantDeck.DeckSlope);

    // -- Fade ranges --------------------------------------------------------
    //
    // Atmosphere thicknesses from the camera. Composed into FadeRanges by the
    // material, like the other float4s.

    const FLinearColor FadeRanges = GasGiantDeck.GetFadeRanges();

    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailFadeNear"), FadeRanges.R);
    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailFadeFar"), FadeRanges.G);
    MID_Atmosphere->SetScalarParameterValue(TEXT("PackedFadeNear"), FadeRanges.B);
    MID_Atmosphere->SetScalarParameterValue(TEXT("PackedFadeFar"), FadeRanges.A);

    // -- Loose field scalars ------------------------------------------------

    MID_Atmosphere->SetScalarParameterValue(TEXT("BandSharpness"), GasGiantDeck.BandSharpness);
    MID_Atmosphere->SetScalarParameterValue(TEXT("ReliefThinning"), GasGiantDeck.ReliefThinning);
    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailVertical"), GasGiantDeck.GetDetailVertical());
    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailErosion"), GasGiantDeck.DetailErosion);
    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailRelief"), GasGiantDeck.DetailRelief);
    MID_Atmosphere->SetScalarParameterValue(TEXT("PackedRelief"), GasGiantDeck.PackedRelief);
    MID_Atmosphere->SetScalarParameterValue(TEXT("PackedErosion"), GasGiantDeck.PackedErosion);
    MID_Atmosphere->SetScalarParameterValue(TEXT("DetailDepth"), GasGiantDeck.GetDetailDepth());
    MID_Atmosphere->SetScalarParameterValue(TEXT("DensityCurve"), GasGiantDeck.DensityCurve);
    MID_Atmosphere->SetScalarParameterValue(TEXT("RigidRate"), GasGiantDeck.RigidRate);

    // The sim's clock, not the world's. Requires the material's Time parameter
    // to feed the Custom node directly -- wired through a multiply against an
    // engine Time node, this value is ignored and the field advects against
    // world time, which diverges the moment the sim pauses or restores.
    MID_Atmosphere->SetScalarParameterValue(TEXT("Time"), GetGasGiantTime());

    // -- Local frame --------------------------------------------------------
    //
    // The planet's axes in world space, as three rows. The field is defined
    // with the spin axis on Z; the march runs world-oriented.

    const FVector AxisX = GetActorForwardVector();
    const FVector AxisY = GetActorRightVector();
    const FVector AxisZ = GetActorUpVector();

    MID_Atmosphere->SetVectorParameterValue(TEXT("localAxisX"), FLinearColor(AxisX.X, AxisX.Y, AxisX.Z, 0.0f));
    MID_Atmosphere->SetVectorParameterValue(TEXT("localAxisY"), FLinearColor(AxisY.X, AxisY.Y, AxisY.Z, 0.0f));
    MID_Atmosphere->SetVectorParameterValue(TEXT("localAxisZ"), FLinearColor(AxisZ.X, AxisZ.Y, AxisZ.Z, 0.0f));

    // -- Scattering ---------------------------------------------------------

    MID_Atmosphere->SetVectorParameterValue(TEXT("ScatterNeg"), GasGiantScatter.ScatterNegative);
    MID_Atmosphere->SetVectorParameterValue(TEXT("ScatterPos"), GasGiantScatter.ScatterPositive);
    MID_Atmosphere->SetVectorParameterValue(TEXT("ScatterBase"), GasGiantScatter.ScatterBase);
    MID_Atmosphere->SetScalarParameterValue(TEXT("BandScale"), GasGiantScatter.BandScale);

    // Solved from DeckOpticalDepth against the same TopMax the shell thickness
    // was solved against, so retuning Relief or CoreDensity leaves the deck's
    // opacity where it was authored.
    const float TopMax = GasGiantDeck.GetTopMax();

    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Beta"),
        GasGiantScatter.GetCloudBeta(GasGiantDeck.DeckTopFraction, TopMax, GasGiantDeck.CoreDensity));
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Absorption Beta"),
        GasGiantScatter.GetCloudAbsorptionBeta(GasGiantDeck.DeckTopFraction, TopMax, GasGiantDeck.CoreDensity));
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Ambient"), GasGiantScatter.CloudAmbient);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Phase Params"), GasGiantScatter.CloudPhaseParams);

    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Steps"), GasGiantDeck.CloudSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Light Steps"), GasGiantDeck.CloudLightSteps);
}

// ─────────────────────────────────────────────────────────────────────────────
// Gas giant simulation
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::StartGasGiantSimulation()
{
    if (!GasGiantDeck.SimConfig)
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

    Sim->StartSimulation(GasGiantDeck.SimConfig);
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
    const FVector ColorVec(Shared.LightColor.R, Shared.LightColor.G, Shared.LightColor.B);
    const float Magnitude = ColorVec.Size();

    if (Magnitude > KINDA_SMALL_NUMBER)
    {
        const FLinearColor NormalizedColor(
            Shared.LightColor.R / Magnitude,
            Shared.LightColor.G / Magnitude,
            Shared.LightColor.B / Magnitude, 1.0f);
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