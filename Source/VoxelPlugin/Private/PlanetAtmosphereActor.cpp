#include "PlanetAtmosphereActor.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/PostProcessComponent.h"
#include "Engine/VolumeTexture.h"

// ─────────────────────────────────────────────────────────────────────────────
// Material asset paths (plugin Content folder)
// ─────────────────────────────────────────────────────────────────────────────

static const TCHAR* MatPath_Preprocess = TEXT("/VoxelPlugin/Material/MT_UCA_Preprocess_Inst.MT_UCA_Preprocess_Inst");
static const TCHAR* MatPath_Atmosphere = TEXT("/VoxelPlugin/Material/MT_UCA_Default_Inst.MT_UCA_Default_Inst");
static const TCHAR* MatPath_Postprocess = TEXT("/VoxelPlugin/Material/MT_UCA_Postprocess_Inst.MT_UCA_Postprocess_Inst");
static const TCHAR* DefaultVolumeTexturePath = TEXT("/VoxelPlugin/VolumeTextures/Textures/VT_PerlinWorley_Balanced.VT_PerlinWorley_Balanced");

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
    // PlanetRadius(100M) + SeaLevel(0.5) * NoiseAmplitude(25M) = 112,500,000 cm
    SetActorScale3D(FVector(112500000.0));
}

// ─────────────────────────────────────────────────────────────────────────────
// Lifecycle
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::BeginPlay()
{
    Super::BeginPlay();
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

    if (bInitialized)
    {
        UpdateMaterialParameters();
        UpdateLightFromRotation();
    }
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Planet integration
// ─────────────────────────────────────────────────────────────────────────────

void APlanetAtmosphereActor::InitializeFromPlanet(USceneComponent* InAttachParent)
{
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

UMaterialInterface* APlanetAtmosphereActor::LoadMaterialAsset(const TCHAR* Path)
{
    return Cast<UMaterialInterface>(
        StaticLoadObject(UMaterialInterface::StaticClass(), nullptr, Path));
}

void APlanetAtmosphereActor::CreateMaterialInstances()
{
    if (!PostProcessVolume) return;

    UMaterialInterface* BasePre = LoadMaterialAsset(MatPath_Preprocess);
    UMaterialInterface* BaseAtmo = LoadMaterialAsset(MatPath_Atmosphere);
    UMaterialInterface* BasePost = LoadMaterialAsset(MatPath_Postprocess);

    if (!BasePre || !BaseAtmo || !BasePost)
    {
        UE_LOG(LogTemp, Warning, TEXT("PlanetAtmosphereActor: Failed to load one or more base materials."));
        return;
    }

    MID_Preprocess = UMaterialInstanceDynamic::Create(BasePre, this, TEXT("MID_Preprocess"));
    MID_Atmosphere = UMaterialInstanceDynamic::Create(BaseAtmo, this, TEXT("MID_Atmosphere"));
    MID_Postprocess = UMaterialInstanceDynamic::Create(BasePost, this, TEXT("MID_Postprocess"));

    // Load default volume texture if none assigned
    if (!CloudVolumeTexture)
    {
        CloudVolumeTexture = Cast<UVolumeTexture>(
            StaticLoadObject(UVolumeTexture::StaticClass(), nullptr, DefaultVolumeTexturePath));
    }

    // Assign to post-process volume blendables in exact order: preprocess, atmosphere, postprocess
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

    // ── Atmosphere material (slot 1) ──

    // Transform / geometry
    MID_Atmosphere->SetVectorParameterValue(TEXT("Planet Center"),
        FLinearColor(PlanetCenter.X, PlanetCenter.Y, PlanetCenter.Z, 0.0f));
    MID_Atmosphere->SetScalarParameterValue(TEXT("Planet Radius"), PlanetRadius);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Height Scale"), AtmosphereHeightScale);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Outer Height Scale"), CloudOuterHeightScale);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Inner Height Scale"), CloudInnerHeightScale);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Floor Offset"), AtmosphereFloorOffset);

    // Light direction
    MID_Atmosphere->SetVectorParameterValue(TEXT("Light Direction"),
        FLinearColor(LightDir.X, LightDir.Y, LightDir.Z, 0.0f));

    // Light color
    MID_Atmosphere->SetVectorParameterValue(TEXT("Light Color"), LightColor);

    // Scattering
    MID_Atmosphere->SetVectorParameterValue(TEXT("Rayleigh Beta"), RayleighBeta);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Rayleigh Height"), RayleighHeight);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Mie Beta"), MieBeta);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Mie Height"), MieHeight);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Mie G"), MieG);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Atmosphere Absorption Beta"), AtmosphereAbsorptionBeta);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Absorption Height"), AtmosphereAbsorptionHeight);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Absorption Falloff"), AtmosphereAbsorptionFalloff);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Atmosphere Ambient"), AtmosphereAmbient);

    // Cloud shape
    if (CloudVolumeTexture)
    {
        MID_Atmosphere->SetTextureParameterValue(TEXT("Cloud Volume Texture"), CloudVolumeTexture);
    }
    MID_Atmosphere->SetVectorParameterValue(TEXT("Animation Weights"), AnimationWeights);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Coverage"), CloudCoverage);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Density Multiplier"), CloudDensityMultiplier);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Height Curve Max"), CloudHeightCurveMax);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Height Curve Min"), CloudHeightCurveMin);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Noise Frequency"), CloudNoiseFrequency);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Noise Invert"), CloudNoiseInvert);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Noise Weights"), CloudNoiseWeights);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Detail Erode Strength"), DetailErodeStrength);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Detail Noise Frequency"), DetailNoiseFrequency);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Detail Noise Invert"), DetailNoiseInvert);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Detail Noise Weights"), DetailNoiseWeights);

    // Cloud lighting
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Beta"), CloudBeta);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Absorption Beta"), CloudAbsorptionBeta);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Ambient"), CloudAmbient);
    MID_Atmosphere->SetVectorParameterValue(TEXT("Cloud Phase Params"), CloudPhaseParams);

    // Ray marching
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Steps"), AtmosphereSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Steps"), CloudSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Atmosphere Light Steps"), AtmosphereLightSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Cloud Light Steps"), CloudLightSteps);
    MID_Atmosphere->SetScalarParameterValue(TEXT("Step Scale Factor"), StepScaleFactor);

    // ── Postprocess material (slot 2) ──

    const float AtmosphereRadius = PlanetRadius * (1.0f + AtmosphereHeightScale);

    MID_Postprocess->SetVectorParameterValue(TEXT("Atmosphere Center"),
        FLinearColor(PlanetCenter.X, PlanetCenter.Y, PlanetCenter.Z, 0.0f));
    MID_Postprocess->SetScalarParameterValue(TEXT("Atmosphere Radius"), AtmosphereRadius);
    MID_Postprocess->SetScalarParameterValue(TEXT("Blur Falloff Factor"), BlurFalloffFactor);
    MID_Postprocess->SetScalarParameterValue(TEXT("MaxW"), MaxW);
    MID_Postprocess->SetScalarParameterValue(TEXT("MinW"), MinW);
}

// ─────────────────────────────────────────────────────────────────────────────
// Light update
// ─────────────────────────────────────────────────────────────────────────────

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