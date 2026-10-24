#include "OceanSphereActor.h"
#include "PlanetActor.h"
#include "VoxelPlugin.h"
#include "VoxelOceanSurface.h"
#include "VoxelDensityGPU.h"
#include "VoxelBrickTerrain.h"
#include "VoxelTerrainStats.h"
#include "VoxelUnderwaterViewExtension.h"
#include "VoxelOceanSurfacePass.h"
#include "VoxelOceanRingDraw.h"
#include "VoxelOceanFoamTexture.h"
#include "Components/DirectionalLightComponent.h"
#include "UObject/UObjectIterator.h"
#include "VoxelOceanCascades.h"
#include "VoxelOceanWeather.h"
#include "VoxelOceanShoreField.h"
#include "VoxelOceanCoastField.h"
#include "VoxelOceanLandCacheRHI.h"
#include "VoxelLandCache.h"
#include "VoxelLandBake.h"
#include "Engine/Texture2D.h"
#include "Engine/Texture2DArray.h"
#include "RealtimeMeshComponent.h"
#include "Materials/Material.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "MaterialDomain.h"
#include "Misc/App.h"
#include "Misc/ScopeRWLock.h"
#include "RenderingThread.h"
#include "SceneViewExtension.h"
#include "DrawDebugHelpers.h"
#include "Engine/World.h"
#include "GameFramework/GameStateBase.h"
#include "HAL/IConsoleManager.h"

DECLARE_CYCLE_STAT(TEXT("Ocean: actor tick"), STAT_VoxelOceanActorTick, STATGROUP_VoxelTerrain);

static TAutoConsoleVariable<int32> CVarVoxelOceanDepthFromLandCache(
    TEXT("voxel.Ocean.DepthFromLandCache"), 1,
    TEXT("The ocean's depths (its tiles, the wave clipmap's static layer, the depth probe) read the terrain's land surface cache where it holds the point (stage O9m): 0 the density everywhere, as before."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugOceanQuery(
    TEXT("voxel.Debug.OceanQuery"), 0,
    TEXT("N > 0: a (2N + 1) x (2N + 1) grid of QuerySurface points around the camera, drawn where the query puts the surface (green; red where it folds or the search didn't settle), with their normals (blue) and velocities (yellow)."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelDebugOceanQuerySpacing(
    TEXT("voxel.Debug.OceanQuerySpacing"), 150.0f,
    TEXT("Spacing (cm) of the voxel.Debug.OceanQuery grid."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugOceanWeather(
    TEXT("voxel.Debug.OceanWeather"), 0,
    TEXT("N > 0: the weather on a (2N + 1) x (2N + 1) grid around the camera's foot (voxel.Debug.OceanWeatherSpacing apart): the wind (white arrows, 1 km per m/s), the swell (cyan) and the storms' centres and radii (red circles)."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelDebugOceanWeatherSpacing(
    TEXT("voxel.Debug.OceanWeatherSpacing"), 20.0f,
    TEXT("Spacing (km) of the voxel.Debug.OceanWeather grid."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugOceanShore(
    TEXT("voxel.Debug.OceanShore"), 0,
    TEXT("N > 0: the shore field (stage O8) on a (2N + 1) x (2N + 1) grid around the camera's foot (voxel.Debug.OceanShoreSpacing apart): toward the coast (arrows, 100 m long; green on the sea side, brown on land, red in an inland basin), with the distance and beach slope printed at the centre."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelDebugOceanShoreSpacing(
    TEXT("voxel.Debug.OceanShoreSpacing"), 500.0f,
    TEXT("Spacing (m) of the voxel.Debug.OceanShore grid."),
    ECVF_Default);

namespace OceanSphereActorDetail
{
    // The water's optics on a Single Layer Water surface (the underwater pass takes them
    // directly).
    const FName AbsorptionName(TEXT("VoxelOceanAbsorption"));
    const FName ScatteringName(TEXT("VoxelOceanScattering"));
    const FName PhaseGName(TEXT("VoxelOceanPhaseG"));
}

AOceanSphereActor::AOceanSphereActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    // Default scale = ocean radius matching APlanetActor defaults:
    // PlanetRadius (100M) + SeaLevel (0.5) * NoiseAmplitude (15M) = 107,500,000 cm.
    SetActorScale3D(FVector(107500000.0));

    MeshAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("MeshAttachmentRoot"));
    MeshAttachmentRoot->SetupAttachment(GetRootComponent());
    MeshAttachmentRoot->SetAbsolute(false, false, true);
}

void AOceanSphereActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    // A planet's ocean is initialized by the planet (InitializeFromPlanet).
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;
    if (!GetWorld() || GetWorld()->IsPreviewWorld()) return;
    if (!bInitialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
        Initialize();
}

void AOceanSphereActor::BeginPlay()
{
    Super::BeginPlay();
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;
    // An ocean spawned at run time was built by its construction already.
    if (!bInitialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
        Initialize();
}

void AOceanSphereActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ReleaseUnderwater();
    ReleaseSurfacePass();
    if (ShoreBuilder.IsValid())
        ShoreBuilder->Reset();
    if (CoastBuilder.IsValid())
        CoastBuilder->Reset();
    CoastField.Reset();
    {
        FWriteScopeLock Lock(QueryLock);
        QueryFrame.Reset();
    }
    UnbindTerrain();
    if (Surface.IsValid())
        Surface->Reset();
    bInitialized = false;
    Super::EndPlay(EndPlayReason);
}

void AOceanSphereActor::Destroyed()
{
    ReleaseUnderwater();
    ReleaseSurfacePass();
    {
        FWriteScopeLock Lock(QueryLock);
        QueryFrame.Reset();
    }
    UnbindTerrain();
    if (Surface.IsValid())
        Surface->Reset();
    bInitialized = false;
    Super::Destroyed();
}

void AOceanSphereActor::BeginDestroy()
{
    // An editor world unloads without EndPlay or Destroyed.
    ReleaseUnderwater();
    ReleaseSurfacePass();
    Super::BeginDestroy();
}

#if WITH_EDITOR
void AOceanSphereActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    if (!bInitialized) return;

    const FName PropName = PropertyChangedEvent.GetPropertyName();

    if (PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, OceanMaterial))
    {
        SetOceanMaterial(OceanMaterial);
        return;
    }

    // The settings: only what changed applies (SetOceanSettings). A change inside the struct
    // reports the struct as the member property.
    const FName MemberName = PropertyChangedEvent.MemberProperty ? PropertyChangedEvent.MemberProperty->GetFName() : PropName;
    if (MemberName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, OceanSettings))
    {
        const FVoxelOceanSettings Edited = OceanSettings;
        SetOceanSettings(Edited, PropertyChangedEvent.ChangeType == EPropertyChangeType::Interactive);
        return;
    }

    // Slider drags rebuild once, on release.
    if (PropertyChangedEvent.ChangeType == EPropertyChangeType::Interactive) return;

    // Anything else (an undo reports no property): the settings again, applying only what
    // differs from what the surface was built with.
    if (SurfaceMaterialInstance && OceanMaterial && SurfaceMaterialInstance->Parent != OceanMaterial)
        SetOceanMaterial(OceanMaterial);
    {
        const FVoxelOceanSettings Current = OceanSettings;
        SetOceanSettings(Current);
    }

    // A standalone ocean's density follows the terrain's settings.
    const bool bDensityChanged = !bIsPlanetOwned && PropertyChangedEvent.MemberProperty
        && (PropertyChangedEvent.MemberProperty->GetFName() == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, SurfaceSettings)
            || PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, TerrainPlanetRadius)
            || PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, NoiseAmplitudeRatio));
    if (bDensityChanged)
        Initialize();
}

bool AOceanSphereActor::CanEditChange(const FProperty* InProperty) const
{
    if (!Super::CanEditChange(InProperty))
        return false;

    if (bIsPlanetOwned && InProperty)
    {
        const FName PropName = InProperty->GetFName();
        static const FName TransformNames[] = {
            TEXT("RelativeLocation"), TEXT("RelativeRotation"), TEXT("RelativeScale3D"),
        };
        for (const FName& Name : TransformNames)
        {
            if (PropName == Name) return false;
        }
    }

    return true;
}

void AOceanSphereActor::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
}

void AOceanSphereActor::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
}

void AOceanSphereActor::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
}

void AOceanSphereActor::PostEditMove(bool bFinished)
{
    if (bIsPlanetOwned)
    {
        if (USceneComponent* Root = GetRootComponent())
        {
            Root->SetRelativeLocation(FVector::ZeroVector);
            Root->SetRelativeRotation(FRotator::ZeroRotator);
        }
    }
    Super::PostEditMove(bFinished);
}
#endif

bool AOceanSphereActor::ShouldTickIfViewportsOnly() const
{
    return bInitialized;
}

UMaterialInterface* AOceanSphereActor::DefaultOceanMaterial()
{
    UMaterialInterface* PluginMaterial = Cast<UMaterialInterface>(StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
        TEXT("/VoxelPlugin/MT_Ocean.MT_Ocean"), nullptr, LOAD_NoWarn | LOAD_Quiet));
    return PluginMaterial ? PluginMaterial : UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
}

void AOceanSphereActor::SetOceanMaterial(UMaterialInterface* InMaterial)
{
    OceanMaterial = InMaterial ? InMaterial : DefaultOceanMaterial();
    if (!bInitialized) return;
    SurfaceMaterialInstance = UMaterialInstanceDynamic::Create(OceanMaterial, this);
    SurfaceMaterialInstance->SetFlags(RF_Transient);
    ApplyOptics();
    if (Surface.IsValid())
        Surface->SetMaterial(SurfaceMaterialInstance);
}

void AOceanSphereActor::ApplyOptics()
{
    using namespace OceanSphereActorDetail;
    if (!SurfaceMaterialInstance) return;
    SurfaceMaterialInstance->SetVectorParameterValue(AbsorptionName, OceanSettings.Absorption);
    SurfaceMaterialInstance->SetVectorParameterValue(ScatteringName, OceanSettings.Scattering);
    SurfaceMaterialInstance->SetScalarParameterValue(PhaseGName, OceanSettings.PhaseG);
}

void AOceanSphereActor::SetOceanSettings(const FVoxelOceanSettings& InSettings, bool bInteractive)
{
    OceanSettings = InSettings;
    if (!bInteractive)
        OceanSettings.Sanitize();
    if (!bInitialized) return;   // the next initialization takes them

    // Live, even while a slider moves: the optics on the surface material. The underwater
    // view, its optics and the environment are read every tick.
    ApplyOptics();
    if (bInteractive) return;

    // On release: the mesh rebuilds; the LOD factor refills the surface; the cull rewrites the
    // tiles on screen (GatherInputs reads AppliedSettings).
    if (!OceanSettings.SameMesh(AppliedSettings))
    {
        InitializeInternal(Compositor);
        return;
    }
    if (OceanSettings.LodDistanceFactor != AppliedSettings.LodDistanceFactor && Surface.IsValid())
        Surface->SetLodDistanceFactor(OceanSettings.LodDistanceFactor);
    AppliedSettings = OceanSettings;
}

void AOceanSphereActor::CreateUnderwater()
{
    // A server or a commandlet draws nothing. One extension per world the actor lives in.
    UWorld* World = GetWorld();
    if (UnderwaterExtension.IsValid() || !World || !FApp::CanEverRender()) return;
    UnderwaterExtension = FSceneViewExtensions::NewExtension<FVoxelUnderwaterViewExtension>(World);
}

void AOceanSphereActor::UpdateUnderwater()
{
    if (!UnderwaterExtension.IsValid()) return;

    // A hidden ocean (disabled or parked by its planet) has no underwater view.
    bool bVisible = !IsHidden();
#if WITH_EDITOR
    if (const UWorld* World = GetWorld(); World && !World->IsGameWorld())
        bVisible = bVisible && !IsHiddenEd();
#endif
    FVoxelUnderwaterParams Params;
    TSharedPtr<const FVoxelOceanWaveFrame, ESPMode::ThreadSafe> Frame;
    {
        FReadScopeLock Lock(QueryLock);
        Frame = QueryFrame;
    }
    const bool bOn = OceanSettings.bUnderwaterView && bVisible && bInitialized && Frame.IsValid();
    UnderwaterExtension->SetEnabled(bOn);
    if (!bOn) return;

    // The render thread needs the waves, not the density.
    Params.Waves = *Frame;
    Params.Waves.Density.Reset();
    const FVoxelOceanSettings& S = OceanSettings;
    Params.Absorption = FVector3f(S.Absorption.R, S.Absorption.G, S.Absorption.B);
    Params.Scattering = FVector3f(S.Scattering.R, S.Scattering.G, S.Scattering.B);
    Params.Light = FVector3f(S.UnderwaterLight.R, S.UnderwaterLight.G, S.UnderwaterLight.B);
    Params.Meniscus = FVector4f(S.MeniscusColor.R, S.MeniscusColor.G, S.MeniscusColor.B, (float)FMath::Max(S.MeniscusWidth, 0.0));
    // The FFT model's textures (kept alive here; released after the extension).
    if (const FVoxelOceanCascadeTextures* Textures = Surface.IsValid() ? Surface->GetCascadeTextures() : nullptr)
    {
        Params.Displacement = Textures->Displacement->GetResource();
        Params.Derivatives = Textures->Derivatives->GetResource();
        Params.Charts = Textures->Charts->GetResource();
        Params.Foam = Textures->Foam ? Textures->Foam->GetResource() : nullptr;
    }
    // The foam seen from below (stage O9c), with the foam on.
    Params.FoamColor = FVector4f(S.UnderwaterFoamColor.R, S.UnderwaterFoamColor.G, S.UnderwaterFoamColor.B, S.bFoam ? 1.0f : 0.0f);
    // The shore (stage O8): the field's texture, the depth probe, the inland basins' margin.
    Params.Shore = ShoreRHI;
    Params.Probe = Surface.IsValid() ? Surface->GetProbeState() : nullptr;
    Params.ShoreMarginCm = Frame->ShoreMarginCm;
    UnderwaterExtension->SetFrame_GameThread(Params);
}

void AOceanSphereActor::ReleaseUnderwater()
{
    if (!UnderwaterExtension.IsValid()) return;
    UnderwaterExtension->SetEnabled(false);
    // The last reference goes on the render thread, after the frames that may still use it.
    ENQUEUE_RENDER_COMMAND(VoxelUnderwaterRelease)(
        [Extension = MoveTemp(UnderwaterExtension)](FRHICommandListImmediate&) mutable
        {
            Extension.Reset();
        });
    UnderwaterExtension.Reset();
}

void AOceanSphereActor::CreateSurfacePass()
{
    // A server or a commandlet draws nothing. One per world the actor lives in.
    UWorld* World = GetWorld();
    if (SurfacePass.IsValid() || !World || !FApp::CanEverRender()) return;
    SurfacePass = FSceneViewExtensions::NewExtension<FVoxelOceanSurfacePass>(World);
    bSurfacePassOn = false;
}

UDirectionalLightComponent* AOceanSphereActor::FindSunLight()
{
    UWorld* World = GetWorld();
    if (!World) return nullptr;
    const double Now = FPlatformTime::Seconds();
    UDirectionalLightComponent* Current = SunLight.Get();
    if (Current && Current->GetWorld() == World && Now - SunLightSearchTime < 1.0) return Current;
    SunLightSearchTime = Now;
    UDirectionalLightComponent* Best = nullptr;
    bool bBestAtmosphere = false;
    float BestBrightness = -1.0f;
    for (TObjectIterator<UDirectionalLightComponent> It; It; ++It)
    {
        UDirectionalLightComponent* Light = *It;
        if (!Light || Light->GetWorld() != World || !Light->IsRegistered() || !Light->IsVisible() || !Light->bAffectsWorld) continue;
        const bool bAtmosphere = Light->IsUsedAsAtmosphereSunLight() && Light->GetAtmosphereSunLightIndex() == 0;
        const float Brightness = Light->GetColoredLightBrightness().GetLuminance();
        if (!Best || (bAtmosphere && !bBestAtmosphere) || (bAtmosphere == bBestAtmosphere && Brightness > BestBrightness))
        {
            Best = Light;
            bBestAtmosphere = bAtmosphere;
            BestBrightness = Brightness;
        }
    }
    SunLight = Best;
    return Best;
}

void AOceanSphereActor::UpdateSurfacePass()
{
    if (!SurfacePass.IsValid()) return;
    bool bVisible = !IsHidden();
#if WITH_EDITOR
    if (const UWorld* World = GetWorld(); World && !World->IsGameWorld())
        bVisible = bVisible && !IsHiddenEd();
#endif
    const TSharedPtr<IVoxelOceanRingDraw, ESPMode::ThreadSafe> Rings = Surface.IsValid() ? Surface->GetRingDraw() : nullptr;
    const bool bOn = OceanSettings.bOwnShading && bVisible && bInitialized && Rings.IsValid();
    SurfacePass->SetEnabled(bOn);
    if (!bOn)
    {
        // Off: the render thread lets go of the rings (their buffers).
        if (bSurfacePassOn) SurfacePass->SetFrame_GameThread(FVoxelOceanShadingParams());
        bSurfacePassOn = false;
        return;
    }
    bSurfacePassOn = true;

    const FVoxelOceanSettings& S = OceanSettings;
    FVoxelOceanShadingParams Params;
    Params.Rings = Rings;
    Params.Absorption = FVector3f(S.Absorption.R, S.Absorption.G, S.Absorption.B);
    Params.Scattering = FVector3f(S.Scattering.R, S.Scattering.G, S.Scattering.B);
    Params.PhaseG = S.PhaseG;
    // The stand-in light (stage W3a): the world's sun, and a gradient sky by its illuminance.
    if (UDirectionalLightComponent* Sun = FindSunLight())
    {
        const FLinearColor E = Sun->GetColoredLightBrightness();
        const float Luminance = FMath::Max(E.GetLuminance(), 0.0f);
        Params.SunDirection = FVector3f(-Sun->GetDirection().GetSafeNormal());
        Params.SunIlluminance = FVector3f(E.R, E.G, E.B);
        Params.SkyZenith = FVector3f(S.StandInSkyZenith.R, S.StandInSkyZenith.G, S.StandInSkyZenith.B) * Luminance;
        Params.SkyHorizon = FVector3f(S.StandInSkyHorizon.R, S.StandInSkyHorizon.G, S.StandInSkyHorizon.B) * Luminance;
    }
    else if (!bWarnedNoSun)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] %s: no directional light in the world, so the ocean's own pass has no sun or sky to shade with until the atmosphere's light is connected (stage W3b). Add one, or turn off bOwnShading for the material."),
            *GetName());
        bWarnedNoSun = true;
    }
    // The sub-mesh roughness is the waves below the shortest band (21 cm): the equalizer's
    // shortest gain scales it (on the height, so squared on the slope variance).
    Params.ShortWaveSlopeVariance = S.ShortWaveSlopeVariance * FMath::Square(S.WaveEqualizer.Gain25cm);
    Params.Refraction = S.Refraction;
    Params.bFoam = S.bFoam;
    Params.FoamAlbedo = FVector3f(S.FoamAlbedo.R, S.FoamAlbedo.G, S.FoamAlbedo.B);
    // The foam's set (stage W3g): read once a name (tried once; a failure says why, and the
    // procedural lace stands in).
    // (Tried again when the name changes, or the foam is turned on after starting off.)
    if (!bFoamTextureTried || FoamTextureName != S.FoamSet || (S.bFoam && !bFoamTextureForFoam))
    {
        bFoamTextureTried = true;
        bFoamTextureForFoam = S.bFoam;
        FoamTextureName = S.FoamSet;
        FoamTexture.Reset();
        if (!FoamTextureName.IsEmpty() && S.bFoam)
        {
            FString Error;
            FoamTexture = VoxelOceanFoamTexture::Load(FoamTextureName, Error);
            if (!FoamTexture.IsValid())
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] %s: the foam set couldn't be read (%s); the procedural lace stands in."), *GetName(), *Error);
            }
            // (Else it reads in the background; the procedural lace until its textures exist.)
        }
    }
    Params.FoamTexture = FoamTexture;
    Params.FoamTileM = S.FoamTile;
    Params.CrestGlow = S.CrestGlow;
    Params.GodotWaterLight = S.GodotWaterLight;
    Params.GodotWaterColor = FVector3f(S.GodotWaterColor.R, S.GodotWaterColor.G, S.GodotWaterColor.B);
    Params.WhitecapRelief = S.WhitecapRelief;
    Params.IntersectionFoam = S.IntersectionFoam;
    Params.IntersectionFoamWidthM = S.IntersectionFoamWidth;
    // The spray (stage W3o): the wind at the camera drives it (the weather varies over kilometres).
    Params.SprayAmount = S.bSpray ? S.SprayAmount : 0.0f;
    Params.SprayReachM = S.SprayDistance;
    Params.SprayParticles = S.SprayParticles;
    Params.SprayFoam = S.SprayFoam;
    Params.SpraySize = S.SpraySize;
    Params.SprayOpacity = S.SprayOpacity;
    Params.SprayBreakup = S.SprayBreakup;
    Params.FoamOpacity = S.FoamOpacity;
    Params.FoamThinOpacity = S.FoamThinOpacity;
    Params.FoamFalloff = S.FoamFalloff;
    Params.FoamSoftness = S.FoamSoftness;
    Params.FoamDetail = S.FoamDetail;
    Params.FoamDetailDistanceM = S.FoamDetailDistance;
    Params.FoamFadeDistanceM = S.FoamFadeDistance;
    Params.bCubicFiltering = S.bCubicFiltering;
    Params.Anisotropy = S.WaveAnisotropy;
    if (Params.SprayAmount > 0.0f && bHasCamera)
    {
        FVector Wind, Swell;
        double Storm = 0.0;
        const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
        if (GetWeatherAt(NoScaleTransform.TransformPosition(CameraPosition), Wind, Storm, Swell))
            Params.SprayWind = FVector3f(Wind);
    }
    SurfacePass->SetFrame_GameThread(Params);
}

void AOceanSphereActor::ReleaseSurfacePass()
{
    if (!SurfacePass.IsValid()) return;
    SurfacePass->SetEnabled(false);
    // Off the renderer's hook, and the last reference, on the render thread after the frames that
    // may still use it.
    ENQUEUE_RENDER_COMMAND(VoxelOceanSurfacePassRelease)(
        [Pass = MoveTemp(SurfacePass)](FRHICommandListImmediate&) mutable
        {
            Pass->Unregister_RenderThread();
            Pass.Reset();
        });
    SurfacePass.Reset();
    bSurfacePassOn = false;
}

bool AOceanSphereActor::QuerySurface(TConstArrayView<FVector> WorldPoints, TArrayView<FVoxelOceanSample> Out,
    EVoxelOceanQuery Flags, double Time) const
{
    const int32 Count = FMath::Min(WorldPoints.Num(), Out.Num());
    TSharedPtr<const FVoxelOceanWaveFrame, ESPMode::ThreadSafe> Frame;
    {
        FReadScopeLock Lock(QueryLock);
        Frame = QueryFrame;
    }
    if (!Frame.IsValid() || Frame->Radius() <= 0.0)
    {
        // Not initialized: the plain sphere around the actor on the game thread (the actor's
        // state isn't safe to read elsewhere); elsewhere far above any water, so a caller that
        // ignores the result applies no buoyancy.
        const bool bGameThread = IsInGameThread();
        for (int32 i = 0; i < Count; i++)
        {
            Out[i] = FVoxelOceanSample();
            Out[i].Height = bGameThread ? (WorldPoints[i] - GetActorLocation()).Size() - OceanRadius : VoxelOceanWaves::Deep;
        }
        return false;
    }

    // World to the planet frame and back through the snapshot's transform.
    TArray<FVector, TInlineAllocator<16>> Points;
    Points.SetNumUninitialized(Count);
    for (int32 i = 0; i < Count; i++)
        Points[i] = Frame->PlanetToWorld.InverseTransformPositionNoScale(WorldPoints[i]);
    VoxelOceanQuery::Run(*Frame, Points, Out.Slice(0, Count), Flags, Time);
    for (int32 i = 0; i < Count; i++)
    {
        FVoxelOceanSample& S = Out[i];
        S.SurfaceLocation = Frame->PlanetToWorld.TransformPositionNoScale(S.SurfaceLocation);
        S.Normal = Frame->PlanetToWorld.TransformVectorNoScale(S.Normal);
        S.Velocity = Frame->PlanetToWorld.TransformVectorNoScale(S.Velocity);
    }
    return true;
}

FVoxelOceanSample AOceanSphereActor::QuerySurfaceAt(const FVector& WorldLocation, bool bWithDepth) const
{
    FVoxelOceanSample Sample;
    QuerySurface(MakeArrayView(&WorldLocation, 1), MakeArrayView(&Sample, 1),
        bWithDepth ? EVoxelOceanQuery::Depth : EVoxelOceanQuery::None);
    return Sample;
}

double AOceanSphereActor::GetHeightAboveSurface(const FVector& WorldLocation) const
{
    return QuerySurfaceAt(WorldLocation, false).Height;
}

bool AOceanSphereActor::IsSubmerged(const FVector& WorldLocation) const
{
    return GetHeightAboveSurface(WorldLocation) < 0.0;
}

double AOceanSphereActor::GetOceanTime() const
{
    // The server's clock where there is one, so every machine draws and queries the same waves.
    const UWorld* World = GetWorld();
    if (!World) return 0.0;
    const AGameStateBase* GameState = World->GetGameState();
    const double Seconds = GameState ? (double)GameState->GetServerWorldTimeSeconds() : (double)World->GetTimeSeconds();
    return Seconds * FMath::Max(OceanSettings.OceanTimeScale, 0.0);
}

void AOceanSphereActor::UpdateWeatherSource()
{
    const FVoxelOceanSettings& S = OceanSettings;
    const FVoxelOceanSettings& Was = WeatherSourceSettings;
    const bool bSame = WeatherSource.IsValid() && WeatherSourceRadius == OceanRadius && S.bProceduralWeather == Was.bProceduralWeather
        && (S.bProceduralWeather
            ? (S.WeatherSeed == Was.WeatherSeed && S.TradeWind == Was.TradeWind && S.Westerly == Was.Westerly && S.PolarWind == Was.PolarWind
                && S.WindNoise == Was.WindNoise && S.WindNoiseSize == Was.WindNoiseSize && S.StormCount == Was.StormCount
                && S.StormWind == Was.StormWind && S.StormRadius == Was.StormRadius && S.StormHours == Was.StormHours
                && S.SwellStrength == Was.SwellStrength && S.SwellRange == Was.SwellRange
                && S.WeatherTimeScale == Was.WeatherTimeScale && S.WeatherTimeOffset == Was.WeatherTimeOffset)
            : (S.WindSpeed == Was.WindSpeed && S.WindHeading == Was.WindHeading && S.StormIntensity == Was.StormIntensity));
    if (bSame) return;
    if (S.bProceduralWeather)
    {
        FVoxelOceanWeatherSettings W;
        W.Seed = S.WeatherSeed;
        W.TradeWind = S.TradeWind;
        W.Westerly = S.Westerly;
        W.PolarWind = S.PolarWind;
        W.NoiseWind = S.WindNoise;
        W.NoiseSizeKm = S.WindNoiseSize;
        W.StormCount = S.StormCount;
        W.StormWind = S.StormWind;
        W.StormRadiusKm = S.StormRadius;
        W.StormHours = S.StormHours;
        W.SwellStrength = S.SwellStrength;
        W.SwellRangeKm = S.SwellRange;
        W.TimeScale = S.WeatherTimeScale;
        W.TimeOffset = S.WeatherTimeOffset;
        WeatherSource = MakeShared<FVoxelOceanProceduralWeather, ESPMode::ThreadSafe>(W, FMath::Max(OceanRadius, 1.0));
    }
    else
    {
        WeatherSource = MakeShared<FVoxelOceanConstantWeather, ESPMode::ThreadSafe>(GetEnvironment());
    }
    WeatherSourceSettings = S;
    WeatherSourceRadius = OceanRadius;
}

bool AOceanSphereActor::GetWeatherAt(const FVector& WorldLocation, FVector& Wind, double& Storm, FVector& Swell) const
{
    Wind = FVector::ZeroVector;
    Swell = FVector::ZeroVector;
    Storm = 0.0;
    if (!WeatherSource.IsValid()) return false;
    const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
    const FVector Planet = NoScaleTransform.InverseTransformPosition(WorldLocation);
    const FVector Dir = Planet.GetSafeNormal();
    const FVoxelOceanEnvironment E = WeatherSource->Sample(Planet, GetOceanTime());
    Wind = NoScaleTransform.TransformVectorNoScale(VoxelOceanWeather::FromSpeedHeading(Dir, E.WindSpeed, E.WindHeading));
    Swell = NoScaleTransform.TransformVectorNoScale(VoxelOceanWeather::FromSpeedHeading(Dir, E.SwellSpeed, E.SwellHeading));
    Storm = E.Storm;
    return true;
}

void AOceanSphereActor::DrawWeatherDebug() const
{
#if ENABLE_DRAW_DEBUG
    const int32 N = FMath::Clamp(CVarVoxelDebugOceanWeather.GetValueOnGameThread(), 0, 32);
    UWorld* World = GetWorld();
    if (N <= 0 || !World || !bHasCamera || !WeatherSource.IsValid()) return;
    const double Spacing = FMath::Max(0.1, (double)CVarVoxelDebugOceanWeatherSpacing.GetValueOnGameThread()) * 1.0e5;
    const double Time = GetOceanTime();
    const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
    const FVector Up = CameraPosition.GetSafeNormal();
    FVector E1, E2;
    Up.FindBestAxisVectors(E1, E2);
    // Arrows drawn a little above the sea, 1 km long per m/s.
    const double Lift = 2000.0;
    for (int32 j = -N; j <= N; j++)
    {
        for (int32 i = -N; i <= N; i++)
        {
            const FVector Dir = (Up * OceanRadius + E1 * (i * Spacing) + E2 * (j * Spacing)).GetSafeNormal();
            const FVoxelOceanEnvironment E = WeatherSource->Sample(Dir * OceanRadius, Time);
            const FVector Base = NoScaleTransform.TransformPosition(Dir * (OceanRadius + Lift));
            const FVector Wind = NoScaleTransform.TransformVectorNoScale(VoxelOceanWeather::FromSpeedHeading(Dir, E.WindSpeed, E.WindHeading));
            const FVector Swell = NoScaleTransform.TransformVectorNoScale(VoxelOceanWeather::FromSpeedHeading(Dir, E.SwellSpeed, E.SwellHeading));
            const FColor WindColor = FLinearColor::LerpUsingHSV(FLinearColor::White, FLinearColor::Red, (float)E.Storm).ToFColor(true);
            DrawDebugDirectionalArrow(World, Base, Base + Wind * 1.0e5, 5000.0f, WindColor, false, -1.0f, SDPG_Foreground, 300.0f);
            if (E.SwellSpeed > 0.1)
                DrawDebugDirectionalArrow(World, Base, Base + Swell * 1.0e5, 5000.0f, FColor::Cyan, false, -1.0f, SDPG_Foreground, 150.0f);
        }
    }
    // The source is procedural when the settings it was made for say so.
    if (const FVoxelOceanProceduralWeather* Procedural = static_cast<const FVoxelOceanProceduralWeather*>(WeatherSourceSettings.bProceduralWeather ? WeatherSource.Get() : nullptr))
    {
        TArray<FVoxelOceanStorm> Storms;
        Procedural->StormsAt(Procedural->WeatherTime(Time), Storms);
        for (const FVoxelOceanStorm& Storm : Storms)
        {
            if (Storm.Strength < 0.05) continue;
            const FVector Centre = NoScaleTransform.TransformPosition(Storm.Centre * (OceanRadius + Lift));
            FVector A, B;
            Storm.Centre.FindBestAxisVectors(A, B);
            DrawDebugCircle(World, Centre, (float)(Storm.Radius * OceanRadius), 64, FColor(255, (uint8)(255 * (1.0 - Storm.Strength)), 0), false, -1.0f, SDPG_Foreground, 2000.0f,
                NoScaleTransform.TransformVectorNoScale(A), NoScaleTransform.TransformVectorNoScale(B), false);
        }
    }
#endif
}

FVoxelOceanEnvironment AOceanSphereActor::GetEnvironment() const
{
    FVoxelOceanEnvironment Environment;
    Environment.WindSpeed = FMath::Max(OceanSettings.WindSpeed, 0.0);
    Environment.WindHeading = FMath::DegreesToRadians(OceanSettings.WindHeading);
    Environment.Storm = FMath::Clamp(OceanSettings.StormIntensity, 0.0, 1.0);
    return Environment;
}

void AOceanSphereActor::UpdateShoreField()
{
    if (!ShoreBuilder.IsValid())
        ShoreBuilder = MakeShared<FVoxelOceanShoreBuilder>();
    // The planet's land bake (none for a noise planet or a standalone ocean without one).
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake;
    if (Compositor.IsValid() && Compositor->GetBaseDensity().IsValid())
        Bake = Compositor->GetBaseDensity()->GetLandBake();
    // As last applied: a slider drag doesn't start a build every frame.
    const double ReachM = FMath::Max(AppliedSettings.ShoreFieldReach, 1.0) * 1000.0;
    const bool bHave = ShoreField.IsValid() && Bake.IsValid() && ShoreField->BakeKey == Bake->Key && ShoreField->N == Bake->N
        && ShoreField->RadiusCm == OceanRadius && ShoreField->ReachM == ReachM;
    if (!Bake.IsValid())
    {
        if (ShoreField.IsValid() || !ShoreBuilder->IsDone()) ShoreBuilder->Reset();
        ShoreField.Reset();
        ShoreTexture = nullptr;
        ShoreResource = nullptr;
        ShoreRHI.Reset();
        return;
    }
    if (!bHave && !ShoreBuilder->IsFor(Bake.Get(), OceanRadius, ReachM))
        ShoreBuilder->Start(Bake, OceanRadius, ReachM);
    ShoreBuilder->Tick();
    if (!bHave && ShoreBuilder->IsDone() && ShoreBuilder->IsFor(Bake.Get(), OceanRadius, ReachM))
    {
        const TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> Made = ShoreBuilder->GetResult();
        if (Made.IsValid() && Made != ShoreField)
        {
            ShoreField = Made;
            ShoreTexture = Made->MakeTexture();
            ShoreResource = ShoreTexture ? ShoreTexture->GetResource() : nullptr;
            ShoreRHI = FVoxelOceanShoreRHI::Make(ShoreTexture.Get(), Made->N, Made->ReachM);
        }
        else if (!Made.IsValid() && ShoreField.IsValid())
        {
            // The rebuild failed: no field rather than one for another bake, radius or reach.
            ShoreField.Reset();
            ShoreTexture = nullptr;
            ShoreResource = nullptr;
            ShoreRHI.Reset();
        }
    }
    // The texture's resource made again (it is ours alone, but a device reset or an editor
    // action can): its render-thread handle again.
    if (ShoreField.IsValid() && ShoreTexture && ShoreTexture->GetResource() != ShoreResource)
    {
        ShoreResource = ShoreTexture->GetResource();
        ShoreRHI = FVoxelOceanShoreRHI::Make(ShoreTexture.Get(), ShoreField->N, ShoreField->ReachM);
    }
}

void AOceanSphereActor::UpdateCoastField()
{
    if (!CoastBuilder.IsValid())
        CoastBuilder = MakeShared<FVoxelOceanCoastBuilder>();
    const FVoxelOceanSettings& S = AppliedSettings;
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake;
    if (Compositor.IsValid() && Compositor->GetBaseDensity().IsValid())
        Bake = Compositor->GetBaseDensity()->GetLandBake();
    // A field for another shore field (made again) is dropped: its texels went with that texture.
    if (CoastField.IsValid() && CoastField->Shore != ShoreField)
        CoastField.Reset();
    const bool bWant = S.bShoaling && S.bCoastField && ShoreField.IsValid() && ShoreTexture && Bake.IsValid() && Surface.IsValid() && FApp::CanEverRender();
    if (!bWant)
    {
        if (!CoastBuilder->IsDone()) CoastBuilder->Reset();
        if (CoastField.IsValid())
        {
            // The shore field's texture again without the coast's slices.
            CoastField.Reset();
            ShoreTexture = ShoreField.IsValid() ? ShoreField->MakeTexture() : nullptr;
            ShoreResource = ShoreTexture ? ShoreTexture->GetResource() : nullptr;
            ShoreRHI = ShoreField.IsValid() ? FVoxelOceanShoreRHI::Make(ShoreTexture.Get(), ShoreField->N, ShoreField->ReachM) : nullptr;
        }
        return;
    }
    VoxelOceanCoastCore::FSettings Settings;
    Settings.SeedDepthM = FMath::Max(S.CoastSeedDepth, 1.0);
    Settings.ReachM = FMath::Max(S.CoastReach, 0.1) * 1000.0;
    Settings.BudgetBytes = FMath::Max(S.CoastMemoryMB, 1.0) * 1024.0 * 1024.0;
    const bool bHave = CoastField.IsValid() && CoastField->Requested == Settings;
    if (!bHave && !CoastBuilder->IsFor(ShoreField.Get(), Settings))
        CoastBuilder->Start(ShoreField, Bake, Settings);
    CoastBuilder->Tick([this](const TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>& Job, int64 First, int32 Count)
        {
            return Surface.IsValid() && Surface->SampleCoastDepths(Job, First, Count);
        });
    if (!bHave && CoastBuilder->IsDone() && CoastBuilder->IsFor(ShoreField.Get(), Settings))
    {
        const TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe> Made = CoastBuilder->GetResult();
        if (Made.IsValid() && Made != CoastField && Made->Shore == ShoreField)
        {
            CoastField = Made;
            ShoreTexture = ShoreField->MakeTexture(Made->Texels.data(), Made->Slices);
            ShoreResource = ShoreTexture ? ShoreTexture->GetResource() : nullptr;
            ShoreRHI = FVoxelOceanShoreRHI::Make(ShoreTexture.Get(), ShoreField->N, ShoreField->ReachM);
        }
    }
}

void AOceanSphereActor::DrawShoreDebug() const
{
#if ENABLE_DRAW_DEBUG
    const int32 N = FMath::Clamp(CVarVoxelDebugOceanShore.GetValueOnGameThread(), 0, 32);
    UWorld* World = GetWorld();
    if (N <= 0 || !World || !bHasCamera || !ShoreField.IsValid()) return;
    const double Spacing = FMath::Max(1.0, (double)CVarVoxelDebugOceanShoreSpacing.GetValueOnGameThread()) * 100.0;
    const double Margin = ShoreField->MarginCm(AppliedSettings.ShoreBasinMargin) * VoxelOceanWaves::BasinDrainSpan;
    const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
    const FVector Up = CameraPosition.GetSafeNormal();
    FVector E1, E2;
    Up.FindBestAxisVectors(E1, E2);
    const double Lift = 300.0;
    for (int32 j = -N; j <= N; j++)
    {
        for (int32 i = -N; i <= N; i++)
        {
            const FVector Dir = (Up * OceanRadius + E1 * (i * Spacing) + E2 * (j * Spacing)).GetSafeNormal();
            const FVoxelOceanShore S = ShoreField->Sample(Dir * OceanRadius);
            const FVector Base = NoScaleTransform.TransformPosition(Dir * (OceanRadius + Lift));
            const FVector Toward = NoScaleTransform.TransformVectorNoScale(S.Direction);
            const FColor Color = S.Distance >= 0.0 ? FColor::Green : (S.Distance < -Margin ? FColor::Red : FColor(150, 100, 40));
            DrawDebugDirectionalArrow(World, Base, Base + Toward * 10000.0, 1500.0f, Color, false, -1.0f, SDPG_Foreground, 60.0f);
            if (i == 0 && j == 0)
            {
                DrawDebugString(World, Base + NoScaleTransform.TransformVectorNoScale(Up) * 500.0,
                    FString::Printf(TEXT("shore %.0f m, slope %.4f"), S.Distance * 0.01, S.Slope), nullptr, FColor::White, 0.0f, true);
            }
        }
    }
#endif
}

void AOceanSphereActor::PublishWaveFrame()
{
    TSharedPtr<FVoxelOceanWaveFrame, ESPMode::ThreadSafe> Frame = MakeShared<FVoxelOceanWaveFrame, ESPMode::ThreadSafe>();
    if (!Surface.IsValid() || !Surface->GetWaveFrame(*Frame)) return;
    // Depth inputs and depth queries read the planet density (planet frame).
    Frame->Density = Compositor;
    FWriteScopeLock Lock(QueryLock);
    QueryFrame = Frame;
}

void AOceanSphereActor::DrawQueryDebug() const
{
#if ENABLE_DRAW_DEBUG
    const int32 N = FMath::Clamp(CVarVoxelDebugOceanQuery.GetValueOnGameThread(), 0, 32);
    UWorld* World = GetWorld();
    if (N <= 0 || !World || !bHasCamera) return;
    // A grid on the sea sphere around the camera's foot, in the planet frame.
    const double Spacing = FMath::Max(1.0, (double)CVarVoxelDebugOceanQuerySpacing.GetValueOnGameThread());
    const FVector Up = CameraPosition.GetSafeNormal();
    FVector E1, E2;
    Up.FindBestAxisVectors(E1, E2);
    const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
    TArray<FVector> Points;
    for (int32 j = -N; j <= N; j++)
    {
        for (int32 i = -N; i <= N; i++)
        {
            const FVector Planet = (Up * OceanRadius + E1 * (i * Spacing) + E2 * (j * Spacing)).GetSafeNormal() * OceanRadius;
            Points.Add(NoScaleTransform.TransformPosition(Planet));
        }
    }
    TArray<FVoxelOceanSample> Samples;
    Samples.SetNum(Points.Num());
    if (!QuerySurface(Points, Samples)) return;
    for (const FVoxelOceanSample& S : Samples)
    {
        DrawDebugPoint(World, S.SurfaceLocation, 8.0f, S.bAmbiguous ? FColor::Red : FColor::Green, false, -1.0f, SDPG_Foreground);
        DrawDebugLine(World, S.SurfaceLocation, S.SurfaceLocation + S.Normal * 50.0, FColor::Blue, false, -1.0f, SDPG_Foreground);
        DrawDebugLine(World, S.SurfaceLocation, S.SurfaceLocation + S.Velocity * 0.25, FColor::Yellow, false, -1.0f, SDPG_Foreground);
    }
#endif
}

void AOceanSphereActor::SetActorHiddenInGame(bool bNewHidden)
{
    Super::SetActorHiddenInGame(bNewHidden);
    // A hidden ocean (disabled or parked by its planet) has no underwater view.
    if (bNewHidden && UnderwaterExtension.IsValid())
        UnderwaterExtension->SetEnabled(false);
    if (bNewHidden && SurfacePass.IsValid())
        SurfacePass->SetEnabled(false);
}

void AOceanSphereActor::Initialize()
{
    // Standalone: a density of its own from the terrain settings (no edits).
    const TSharedPtr<FDensitySampleCompositor> Own = VoxelDensity::MakeCompositor(SurfaceSettings, TerrainPlanetRadius,
        TerrainPlanetRadius * NoiseAmplitudeRatio, nullptr);
    InitializeInternal(Own);
}

void AOceanSphereActor::OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport)
{
    if (!bIsPlanetOwned) return;

    if (USceneComponent* Root = GetRootComponent())
    {
        if (!Root->GetRelativeLocation().IsNearlyZero(0.01)
            || !Root->GetRelativeRotation().IsNearlyZero(0.01)
            || !Root->GetComponentScale().Equals(PlanetDrivenScale, 0.01))
        {
            Root->SetRelativeLocation_Direct(FVector::ZeroVector);
            Root->SetRelativeRotation_Direct(FRotator::ZeroRotator);
            Root->SetRelativeScale3D_Direct(PlanetDrivenScale);
            Root->UpdateComponentToWorld();
        }
    }
}

void AOceanSphereActor::InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
    USceneComponent* InAttachParent, FVector InScale)
{
    bIsPlanetOwned = true;

    // Apply scale BEFORE binding the transform guard, so the old guard doesn't revert a scale
    // change the planet intends.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.RemoveAll(this);

    if (!InScale.IsZero())
        SetActorScale3D(InScale);

    PlanetDrivenScale = GetActorScale3D();

    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.AddUObject(this, &AOceanSphereActor::OnTransformUpdated);

    // The pages follow the planet's position and rotation directly.
    if (InAttachParent && MeshAttachmentRoot)
    {
        MeshAttachmentRoot->AttachToComponent(InAttachParent,
            FAttachmentTransformRules::KeepWorldTransform);
    }
    InitializeInternal(InCompositor);
}

void AOceanSphereActor::CleanSceneRoot()
{
    if (!MeshAttachmentRoot) return;
    const TArray<TObjectPtr<USceneComponent>> Attached = MeshAttachmentRoot->GetAttachChildren();
    for (const TObjectPtr<USceneComponent>& Child : Attached)
    {
        if (URealtimeMeshComponent* MeshComponent = Cast<URealtimeMeshComponent>(Child))
            MeshComponent->DestroyComponent();
    }
}

void AOceanSphereActor::InitializeInternal(TSharedPtr<FDensitySampleCompositor> InCompositor)
{
    bInitialized = false;
    if (Surface.IsValid())
        Surface->Reset();
    CleanSceneRoot();
    Compositor = InCompositor;

    OceanRadius = GetActorScale3D().GetMax();
    // 8, 16 or 32 quads per tile edge, and the other settings within their limits.
    OceanSettings.Sanitize();
    OceanSettings.LodDistanceFactor = FMath::Max(OceanSettings.LodDistanceFactor, VoxelOceanTiles::MinLodDistanceFactor);
    MaxLevel = VoxelOceanTiles::LevelForSpacing(OceanRadius, OceanSettings.TileQuads, OceanSettings.FinestSpacing);
    ActualSpacing = VoxelOceanTiles::NominalArc(MaxLevel) * OceanRadius / OceanSettings.TileQuads;

    if (!OceanMaterial)
        OceanMaterial = DefaultOceanMaterial();
    SurfaceMaterialInstance = UMaterialInstanceDynamic::Create(OceanMaterial, this);
    SurfaceMaterialInstance->SetFlags(RF_Transient);

    // The density the depth is sampled from: the compositor's own, as the terrain builds it.
    GPUDensity.Reset();
    if (Compositor.IsValid() && Compositor->GetBaseDensity().IsValid())
    {
        const TSharedRef<FVoxelDensityGPUData> Data = MakeShared<FVoxelDensityGPUData>();
        Compositor->GetBaseDensity()->BuildGPUData(*Data);
        GPUDensity = Data;
    }
    AppliedSettings = OceanSettings;

    // The FFT cascades' textures: the arrays once, the chart table again for a new radius (the
    // underwater pass keeps its own references to the old one's RHI texture until replaced).
    if (FApp::CanEverRender() && (!WaveDisplacement || !WaveDerivatives || !WaveFoam || !WaveCharts || !WaveFineDisplacement
        || !WaveFineDerivatives || !WaveFineFoam || WaveChartsRadius != OceanRadius))
    {
        FVoxelOceanCascadeTextures Made;
        Made.Displacement = WaveDisplacement.Get();
        Made.Derivatives = WaveDerivatives.Get();
        Made.Foam = WaveFoam.Get();
        Made.FineDisplacement = WaveFineDisplacement.Get();
        Made.FineDerivatives = WaveFineDerivatives.Get();
        Made.FineFoam = WaveFineFoam.Get();
        if (VoxelOceanCascades::MakeTextures(this, VoxelOceanCharts::Build(OceanRadius), Made))
        {
            WaveDisplacement = Made.Displacement;
            WaveDerivatives = Made.Derivatives;
            WaveFoam = Made.Foam;
            WaveFineDisplacement = Made.FineDisplacement;
            WaveFineDerivatives = Made.FineDerivatives;
            WaveFineFoam = Made.FineFoam;
            WaveCharts = Made.Charts;
            WaveChartsRadius = OceanRadius;
        }
    }

    FVoxelOceanSurfaceSettings Settings;
    Settings.Radius = OceanRadius;
    Settings.TileQuads = OceanSettings.TileQuads;
    Settings.MaxLevel = MaxLevel;
    Settings.LodDistanceFactor = OceanSettings.LodDistanceFactor;
    Settings.SpareBufferSets = OceanSettings.SpareBufferSets;
    Settings.bWavesInCompute = OceanSettings.bWavesInCompute;
    Settings.Clip.bOn = OceanSettings.bWaveClipmap;
    Settings.Clip.Size = OceanSettings.ClipSize;
    Settings.Clip.Levels = OceanSettings.ClipLevels;
    Settings.Clip.FinestCm = OceanSettings.ClipFinestSpacing;
    Settings.Clip.bRings = OceanSettings.bWaveRings;
    Settings.Clip.FarCells = OceanSettings.FarSphereCells;
    Settings.Clip.bOwnPass = OceanSettings.bOwnShading;
    Settings.Material = SurfaceMaterialInstance.Get();
    if (WaveChartsRadius == OceanRadius)
    {
        Settings.CascadeTextures.Displacement = WaveDisplacement.Get();
        Settings.CascadeTextures.Derivatives = WaveDerivatives.Get();
        Settings.CascadeTextures.Charts = WaveCharts.Get();
        Settings.CascadeTextures.Foam = WaveFoam.Get();
        Settings.CascadeTextures.FineDisplacement = WaveFineDisplacement.Get();
        Settings.CascadeTextures.FineDerivatives = WaveFineDerivatives.Get();
        Settings.CascadeTextures.FineFoam = WaveFineFoam.Get();
    }
    if (!Surface.IsValid())
        Surface = MakeShared<FVoxelOceanSurface>();
    Surface->Initialize(this, MeshAttachmentRoot, Settings);
    ApplyOptics();
    CreateUnderwater();
    CreateSurfacePass();

    LastInitScale = GetActorScale3D();
    bInitialized = true;
    // The underwater pass gets the new textures now, not on the next tick.
    UpdateUnderwater();
}

void AOceanSphereActor::Tick(float DeltaTime)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanActorTick);
    Super::Tick(DeltaTime);
    if (!bInitialized || !Surface.IsValid()) return;

    if (UWorld* World = GetWorld())
    {
        const TArray<FVector>& Views = World->ViewLocationsRenderedLastFrame;
        if (Views.Num() > 0)
        {
            // The planet frame: position and rotation only (the surface is built at world scale).
            const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
            CameraPosition = NoScaleTransform.InverseTransformPosition(Views[0]);
            bHasCamera = true;
        }
    }
    AAdaptiveVoxelActor* Terrain = FindTerrain();
    if (Terrain != BoundTerrain.Get())
        BindTerrain(Terrain);
    UpdateWeatherSource();
    UpdateShoreField();
    UpdateLandCache(Terrain);
    Surface->SetInputs(GatherInputs(Terrain));
    UpdateCoastField();
    Surface->Tick(CameraPosition);
    PublishWaveFrame();

    // The view from under water: every view decides for itself on the render thread.
    UpdateUnderwater();
    // The ocean's own pass (stage W3), with the rings this frame.
    UpdateSurfacePass();
    DrawQueryDebug();
    DrawWeatherDebug();
    DrawShoreDebug();
}

AAdaptiveVoxelActor* AOceanSphereActor::FindTerrain() const
{
    const APlanetActor* Planet = Cast<APlanetActor>(GetOwner());
    return Planet ? Planet->GetTerrainActor() : nullptr;
}

void AOceanSphereActor::BindTerrain(AAdaptiveVoxelActor* Terrain)
{
    UnbindTerrain();
    if (!Terrain) return;
    TerrainEditHandle = Terrain->OnTerrainEdited().AddUObject(this, &AOceanSphereActor::HandleTerrainEdit);
    BoundTerrain = Terrain;
}

void AOceanSphereActor::UnbindTerrain()
{
    if (AAdaptiveVoxelActor* Terrain = BoundTerrain.Get())
        Terrain->OnTerrainEdited().Remove(TerrainEditHandle);
    TerrainEditHandle.Reset();
    BoundTerrain.Reset();
}

void AOceanSphereActor::HandleTerrainEdit(uint32 Sequence, const FVector& Center, double Radius)
{
    // The terrain's actor space without scale is this ocean's planet frame (both sit on the
    // planet's root).
    if (Surface.IsValid())
        Surface->NotifyEdit(Sequence, Center, Radius);
}

FVoxelOceanSurfaceInputs AOceanSphereActor::GatherInputs(const AAdaptiveVoxelActor* Terrain) const
{
    FVoxelOceanSurfaceInputs Inputs;
    Inputs.Density.Density = GPUDensity;
    // As last applied, so a slider drag doesn't write the tiles again every frame.
    Inputs.LandCullMargin = AppliedSettings.bCullUnderLand ? AppliedSettings.LandCullMargin : -1.0;
    // The depth the waves feel (stage O9b2): as last applied too (a change writes every tile).
    Inputs.Density.DepthFootprintCm = AppliedSettings.bShoaling ? AppliedSettings.ShoreDepthSmoothing * 100.0 : 0.0;
    Inputs.Time = GetOceanTime();
    Inputs.Environment = GetEnvironment();
    const FVoxelOceanSettings& S = OceanSettings;
    // The weather and how the waves key it.
    Inputs.Weather = WeatherSource;
    Inputs.WeatherTiming.KeySeconds = S.WeatherKeySeconds;
    Inputs.WeatherTiming.HeadingStepDegrees = S.WaveHeadingStep;
    Inputs.WeatherTiming.FadeSeconds = S.WaveTurnSeconds;
    // The FFT waves' settings (the reach's wind is the surface's to choose).
    Inputs.Waves.Seed = S.WaveSeed;
    Inputs.Waves.FetchKm = S.WaveFetch;
    Inputs.Waves.Chop = S.WaveChop;
    Inputs.Waves.Swell = S.WaveSwell;
    static_assert(FVoxelOceanWaveEqualizer::Knots == FVoxelOceanFFTSettings::EqualizerKnots, "the equalizer's settings and the wave model's must match");
    for (int32 i = 0; i < FVoxelOceanFFTSettings::EqualizerKnots; i++)
        Inputs.Waves.Gain[i] = S.WaveEqualizer.Get(i);
    Inputs.Waves.DetileCell = S.WaveDetileCell;
    Inputs.Waves.bQueryShortWaves = S.bQueryShortWaves;
    Inputs.Waves.MipBias = S.WaveMipBias;
    // The shore (stage O8): shoaling's settings (a breaking index of 0 turns it off), the field.
    Inputs.Waves.ShoreBreaking = S.bShoaling ? S.ShoreBreaking : 0.0;
    Inputs.Waves.ShoreChopGrowth = S.ShoreChopGrowth;
    Inputs.Waves.ShoreWarp = S.ShoreWarp;
    Inputs.Waves.ShoreFade = S.ShoreFadeDistance;
    Inputs.Waves.ShoreWaveFloor = S.ShoreWaveFloor;
    Inputs.Waves.ShoreMinSlope = S.ShoreMinSlope;
    // The surf (stage O9a): a strength of 0 turns it off.
    Inputs.Waves.SurfStrength = S.bShoaling && S.bSurf ? S.SurfStrength : 0.0;
    Inputs.Waves.SurfSets = S.SurfSets;
    Inputs.Waves.SurfSwash = S.SurfSwash;
    Inputs.Waves.SurfSkew = S.SurfSkew;
    Inputs.Waves.SurfSwashReach = S.SurfSwashReach;
    Inputs.Waves.SurfCutoff = S.SurfCutoff;
    Inputs.Waves.SurfFFTFloor = S.SurfFFTFloor;
    // The mesh's split bias near shores (stage O9c; live).
    Inputs.ShoreDetail.Detail = S.ShoreDetail;
    Inputs.ShoreDetail.BandM = S.ShoreDetailBand;
    Inputs.ShoreDetail.ReachM = S.ShoreDetailReach;
    // The foam (stage W3v): a whitecap threshold of 0 makes none.
    Inputs.Waves.FoamWhitecap = S.bFoam ? S.FoamWhitecap : 0.0;
    Inputs.Waves.FoamAmount = S.FoamAmount;
    Inputs.Waves.FoamRamp = S.FoamRamp;
    Inputs.Waves.FineFoamWhitecap = S.bFoam ? S.FineFoamWhitecap : 0.0;
    Inputs.Waves.FineFoamAmount = S.FineFoamAmount;
    Inputs.Waves.UpdateRate = S.WaveUpdateRate;
    Inputs.Waves.FoamDetailLevels = S.FoamDetailLevels;
    Inputs.Waves.SurfFoamLinger = S.SurfFoamLinger;
    Inputs.Waves.FoamBreakup = S.FoamBreakup;
    // (Stage W5f: the clipmap cuts its foam only where the own pass fades it out.)
    Inputs.Waves.FoamFadeDistance = S.bOwnShading ? S.FoamFadeDistance : 0.0;
    Inputs.Waves.SurfFoam = S.bFoam ? S.SurfFoam : 0.0;
    // The field reaches the queries wherever it exists (a server builds it on the CPU, and has
    // no texture); its texture and the pages' cull only where it was made.
    if (ShoreField.IsValid())
    {
        Inputs.Shore = ShoreField;
        // The basins' margin past the sea's drain (stage O9b3): from the bake's margin to 1.5 times it
        // the sea sinks below the ground, and only past that is the basin land.
        Inputs.Density.ShoreMarginCm = ShoreField->MarginCm(AppliedSettings.ShoreBasinMargin) * VoxelOceanWaves::BasinDrainSpan;
        if (ShoreRHI.IsValid())
        {
            Inputs.ShoreTexture = ShoreTexture.Get();
            Inputs.Density.Shore = ShoreRHI;
            // The coastal field's layout (stage O9h) with the texture that holds its texels.
            if (CoastField.IsValid() && CoastField->Shore == ShoreField)
                Inputs.Coast = CoastField;
        }
    }
    if (Terrain && Terrain->ActualPrecision > 0.0)
    {
        // The terrain's lattice (its edit tree is addressed in it) and slope bound.
        Inputs.Density.HalfFinest = 0.5 * Terrain->ActualPrecision;
        Inputs.SlopeBound = Terrain->CollisionSlopeBound;
        if (const TSharedPtr<FVoxelBrickTerrain> Bricks = Terrain->GetBrickTerrain())
        {
            // The slope bound the terrain was built with: a slider still moving doesn't write
            // the ocean's tiles again every frame.
            Inputs.SlopeBound = Bricks->GetSettings().SlopeBound;
            // A sequence only with the data that holds it (a rebuilding terrain drops its data
            // before its sequence).
            Inputs.Density.Edits = Bricks->GetGPUEdits();
            Inputs.EditSequence = Inputs.Density.Edits.IsValid() ? Bricks->GetGPUEditsSequence() : 0;
        }
    }
    else
    {
        // Standalone: any lattice whose indices fit 32 bits; no edits.
        Inputs.Density.HalfFinest = FMath::Max(25.0, OceanRadius * 1.0e-9);
        Inputs.SlopeBound = 3.5;
    }
    // The land surface cache (stage O9m): its texels follow the edit data, so an edit counts as
    // in the depths once both carry it.
    Inputs.WrittenEditSequence = Inputs.EditSequence;
    if (LandCacheRHI.IsValid())
    {
        Inputs.Density.LandCache = LandCacheRHI;
        Inputs.Density.LandEditSequence = LandCacheEditSequence;
        Inputs.WrittenEditSequence = FMath::Min(Inputs.EditSequence, LandCacheEditSequence);
    }
    return Inputs;
}

void AOceanSphereActor::UpdateLandCache(const AAdaptiveVoxelActor* Terrain)
{
    const FVoxelLandCache* Cache = Terrain && CVarVoxelOceanDepthFromLandCache.GetValueOnGameThread() != 0 && FApp::CanEverRender()
        ? Terrain->GetLandCache() : nullptr;
    // Its heights are about its own sea level; one far from this ocean's isn't this planet's.
    if (Cache && FMath::Abs(Cache->GetSeaRadius() - OceanRadius) > 0.01 * OceanRadius) Cache = nullptr;
    // Written for the camera, or not for long: a cache left behind (its material unbound, the
    // terrain not drawn) is let go after two seconds' worth of frames. Meanwhile its edits don't
    // settle, so the ocean's wait for them.
    LandCacheIdleFrames = Cache && Cache->IsLive() ? 0 : LandCacheIdleFrames + 1;
    if (!Cache || !Cache->GetTextures().IsValid() || LandCacheIdleFrames > 120)
    {
        LandCacheRHI.Reset();
        LandCacheEditSequence = 0;
        LandCacheIdleFrames = FMath::Min(LandCacheIdleFrames, 1000);
        return;
    }
    if (!LandCacheRHI.IsValid() || !LandCacheRHI->IsFor(Cache->GetTextures(), Cache->GetSeaRadius(), OceanRadius))
        LandCacheRHI = FVoxelOceanLandCacheRHI::Make(Cache->GetTextures(), Cache->GetSeaRadius(), OceanRadius);
    LandCacheEditSequence = Cache->GetSettledEditSequence();
}
