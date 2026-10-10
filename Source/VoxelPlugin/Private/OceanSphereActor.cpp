#include "OceanSphereActor.h"
#include "PlanetActor.h"
#include "VoxelPlugin.h"
#include "VoxelOceanSurface.h"
#include "VoxelDensityGPU.h"
#include "VoxelBrickTerrain.h"
#include "VoxelTerrainStats.h"
#include "VoxelUnderwaterViewExtension.h"
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

static TAutoConsoleVariable<int32> CVarVoxelDebugOceanQuery(
    TEXT("voxel.Debug.OceanQuery"), 0,
    TEXT("N > 0: a (2N + 1) x (2N + 1) grid of QuerySurface points around the camera, drawn where the query puts the surface (green; red where it folds or the search didn't settle), with their normals (blue) and velocities (yellow)."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelDebugOceanQuerySpacing(
    TEXT("voxel.Debug.OceanQuerySpacing"), 150.0f,
    TEXT("Spacing (cm) of the voxel.Debug.OceanQuery grid."),
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
    Initialize();
}

void AOceanSphereActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    ReleaseUnderwater();
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

    if (PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, LodDistanceFactor))
    {
        if (PropertyChangedEvent.ChangeType != EPropertyChangeType::Interactive && Surface.IsValid())
            Surface->SetLodDistanceFactor(LodDistanceFactor);
        return;
    }

    // The tiles on screen are written again with the new cull (GatherInputs), once per release.
    if (PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, LandCullMargin))
    {
        if (PropertyChangedEvent.ChangeType != EPropertyChangeType::Interactive)
            AppliedLandCullMargin = LandCullMargin;
        return;
    }
    if (PropName == GET_MEMBER_NAME_CHECKED(AOceanSphereActor, bCullUnderLand))
        return;

    // The underwater pass reads the switch and the optics every tick, the waves the environment
    // and the clock.
    static const TSet<FName> TickProps = {
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, bUnderwaterView),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, WindSpeed),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, WindHeading),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, StormIntensity),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, OceanTimeScale),
    };
    if (TickProps.Contains(PropName))
        return;
    // Optics apply live (struct members report the struct as the member property).
    static const TSet<FName> OpticsProps = {
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, Absorption),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, Scattering),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, PhaseG),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, UnderwaterLight),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, MeniscusWidth),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, MeniscusColor),
    };
    const FName MemberName = PropertyChangedEvent.MemberProperty ? PropertyChangedEvent.MemberProperty->GetFName() : PropName;
    if (OpticsProps.Contains(PropName) || OpticsProps.Contains(MemberName))
    {
        ApplyOptics();
        return;
    }
    // The mesh rebuilds; slider drags rebuild once, on release.
    if (PropertyChangedEvent.ChangeType == EPropertyChangeType::Interactive) return;
    static const TSet<FName> MeshProps = {
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, FinestSpacing),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, TileQuads),
        GET_MEMBER_NAME_CHECKED(AOceanSphereActor, SpareBufferSets),
    };
    if (MeshProps.Contains(PropName))
    {
        InitializeInternal(Compositor);
        return;
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
    SurfaceMaterialInstance->SetVectorParameterValue(AbsorptionName, Absorption);
    SurfaceMaterialInstance->SetVectorParameterValue(ScatteringName, Scattering);
    SurfaceMaterialInstance->SetScalarParameterValue(PhaseGName, PhaseG);
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
    const bool bOn = bUnderwaterView && bVisible && bInitialized && Frame.IsValid();
    UnderwaterExtension->SetEnabled(bOn);
    if (!bOn) return;

    // The render thread needs the waves, not the density.
    Params.Waves = *Frame;
    Params.Waves.Density.Reset();
    Params.Absorption = FVector3f(Absorption.R, Absorption.G, Absorption.B);
    Params.Scattering = FVector3f(Scattering.R, Scattering.G, Scattering.B);
    Params.Light = FVector3f(UnderwaterLight.R, UnderwaterLight.G, UnderwaterLight.B);
    Params.Meniscus = FVector4f(MeniscusColor.R, MeniscusColor.G, MeniscusColor.B, (float)FMath::Max(MeniscusWidth, 0.0));
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
    return Seconds * FMath::Max(OceanTimeScale, 0.0);
}

FVoxelOceanEnvironment AOceanSphereActor::GetEnvironment() const
{
    FVoxelOceanEnvironment Environment;
    Environment.WindSpeed = FMath::Max(WindSpeed, 0.0);
    Environment.WindHeading = FMath::DegreesToRadians(WindHeading);
    Environment.Storm = FMath::Clamp(StormIntensity, 0.0, 1.0);
    return Environment;
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
    // 8, 16 or 32 quads per tile edge.
    TileQuads = 1 << FMath::Clamp(FMath::RoundToInt(FMath::Log2((double)FMath::Max(TileQuads, 1))), 3, 5);
    LodDistanceFactor = FMath::Max(LodDistanceFactor, VoxelOceanTiles::MinLodDistanceFactor);
    MaxLevel = VoxelOceanTiles::LevelForSpacing(OceanRadius, TileQuads, FinestSpacing);
    ActualSpacing = VoxelOceanTiles::NominalArc(MaxLevel) * OceanRadius / TileQuads;

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
    AppliedLandCullMargin = LandCullMargin;

    FVoxelOceanSurfaceSettings Settings;
    Settings.Radius = OceanRadius;
    Settings.TileQuads = TileQuads;
    Settings.MaxLevel = MaxLevel;
    Settings.LodDistanceFactor = LodDistanceFactor;
    Settings.SpareBufferSets = SpareBufferSets;
    Settings.Material = SurfaceMaterialInstance.Get();
    if (!Surface.IsValid())
        Surface = MakeShared<FVoxelOceanSurface>();
    Surface->Initialize(this, MeshAttachmentRoot, Settings);
    ApplyOptics();
    CreateUnderwater();

    LastInitScale = GetActorScale3D();
    bInitialized = true;
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
    Surface->SetInputs(GatherInputs(Terrain));
    Surface->Tick(CameraPosition);
    PublishWaveFrame();

    // The view from under water: every view decides for itself on the render thread.
    UpdateUnderwater();
    DrawQueryDebug();
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
    Inputs.LandCullMargin = bCullUnderLand ? AppliedLandCullMargin : -1.0;
    Inputs.Time = GetOceanTime();
    Inputs.Environment = GetEnvironment();
    if (Terrain && Terrain->ActualPrecision > 0.0)
    {
        // The terrain's lattice (its edit tree is addressed in it) and slope bound.
        Inputs.Density.HalfFinest = 0.5 * Terrain->ActualPrecision;
        Inputs.SlopeBound = Terrain->CollisionSlopeBound;
        if (const TSharedPtr<FVoxelBrickTerrain> Bricks = Terrain->GetBrickTerrain())
        {
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
    return Inputs;
}
