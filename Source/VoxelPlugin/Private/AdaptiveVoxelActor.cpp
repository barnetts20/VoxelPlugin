#include "AdaptiveVoxelActor.h"
#include "PlanetActor.h"
#include "VoxelPlugin.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include "FDensitySampleCompositor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "Camera/PlayerCameraManager.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeLock.h"
#include "FSparseEditStore.h"
#include "RealtimeMeshComponent.h"
#include "VoxelTerrainStats.h"
#include "VoxelLandMaterial.h"
#include "VoxelLandBake.h"
#include "VoxelLandCacheLayout.ush"
#include "Engine/Texture2DDynamic.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/Engine.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/World.h"
#include "Misc/App.h"

DECLARE_CYCLE_STAT(TEXT("Actor tick"), STAT_VoxelActorTick, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Collision tick"), STAT_VoxelCollisionTick, STATGROUP_VoxelTerrain);

// An edit task taking longer than this (ms) is logged.
static constexpr double EditLogThresholdMs = 50.0;

#if !UE_BUILD_SHIPPING
static TAutoConsoleVariable<int32> CVarVoxelDebugEditStretch(
    TEXT("voxel.Debug.EditStretch"), 1,
    TEXT("The debug brush (E carves, Q fills) held down. 1 (default): one capsule stretched from where it was pressed to a point kept at that distance along the view, any direction, up to 100 brush radii (300 m) long, until released (drag out a tunnel by walking or turning; a press more than 50 radii away strokes instead). 0: a stroke of segments along the surface the trace hits."),
    ECVF_Default);
#endif

AAdaptiveVoxelActor::AAdaptiveVoxelActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    SurfaceMaterial = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);

    // Default scale defines planet radius in world units (cm).
    // 100,000,000 cm = 1000 km radius planet (matches APlanetActor default).
    SetActorScale3D(FVector(100000000.0));

    BrickAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("BrickAttachmentRoot"));
    BrickAttachmentRoot->SetupAttachment(GetRootComponent());
    BrickAttachmentRoot->SetAbsolute(false, false, true);
}

int32 AAdaptiveVoxelActor::MaxDepthFor(double RootExtent, double InTargetPrecision)
{
    // At depth D the finest cell is 2 * RootExtent / 2^D.
    const double Ratio = 2.0 * RootExtent / FMath::Max(InTargetPrecision, 1.0);
    return FMath::Clamp((int32)FMath::CeilToInt(FMath::Log2(Ratio)), 6, MaxLatticeDepth);
}

void AAdaptiveVoxelActor::BeginDestroy()
{
    IsDestroyed = true;
    if (LandMaterialTickHandle.IsValid())
    {
        FWorldDelegates::OnWorldPostActorTick.Remove(LandMaterialTickHandle);
        LandMaterialTickHandle.Reset();
    }
    if (UWorld* World = GetWorld())
        World->GetTimerManager().ClearAllTimersForObject(this);
    {
        FScopeLock Lock(&EditQueue->CS);
        EditQueue->Notices.Empty();
    }
    Super::BeginDestroy();
}

void AAdaptiveVoxelActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    // When owned by a PlanetActor, skip self-init - planet calls InitializeFromPlanet.
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;

    if (GetWorld() && !GetWorld()->IsPreviewWorld() && bTickInEditor)
    {
        if (!Initialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
        {
            Initialize();
        }
    }
}

#if WITH_EDITOR
void AAdaptiveVoxelActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    if (!Initialized) return;

    FName PropName = PropertyChangedEvent.GetPropertyName();
    // A slider still moving: everything below rebuilds, so it waits for the release.
    const bool bInteractive = PropertyChangedEvent.ChangeType == EPropertyChangeType::Interactive;

    // Changes to the density or the lattice rebuild everything.
    static const TSet<FName> RebuildProps = {
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, NoiseAmplitudeRatio),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, TargetPrecision),
    };

    // A change inside SurfaceSettings reports the inner field as the property; the member
    // property is the struct.
    // Slider drags rebuild once, on release.
    const bool bSurfaceChanged = !bIsPlanetOwned && !bInteractive
        && PropertyChangedEvent.MemberProperty
        && PropertyChangedEvent.MemberProperty->GetFName() == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, SurfaceSettings);

    if ((RebuildProps.Contains(PropName) && !bInteractive) || bSurfaceChanged)
    {
        // A planet's terrain shares the planet's compositor, whose edit store depth follows
        // TargetPrecision: the planet rebuilds both.
        if (APlanetActor* Planet = bIsPlanetOwned ? Cast<APlanetActor>(GetOwner()) : nullptr)
            Planet->RequestRebuild();
        else
            Initialize();
        return;
    }

    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, SurfaceMaterial))
    {
        SetSurfaceMaterial(SurfaceMaterial);
        return;
    }

    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, BrickMorphParameters))
    {
        if (BrickTerrain.IsValid())
            BrickTerrain->SetMorphParameters(BrickMorphParameters.Get());
        return;
    }

    if (bInteractive) return;
    const double ActorPlanetRadius = GetActorScale3D().GetMax();
    const double ActorRootExtent = VoxelDensity::RootExtentFor(ActorPlanetRadius, ActorPlanetRadius * NoiseAmplitudeRatio);

    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, BrickLodDistanceFactor)
        || PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, BrickCellsPerBrick))
    {
        // InitializeBrickTerrain snaps BrickCellsPerBrick to the size it builds (8-64).
        InitializeBrickTerrain(ActorRootExtent);
        return;
    }

    // The slope bound drives both the collision bricks and the rendered terrain's empty test
    // (and the ocean's land cull, which follows it every tick): both restart.
    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, CollisionSlopeBound))
    {
        InitializeCollision(ActorRootExtent);
        InitializeBrickTerrain(ActorRootExtent);
        return;
    }

    // The collision lattice rebuilds only the collision bricks.
    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, CollisionCellScale))
        InitializeCollision(ActorRootExtent);
}

bool AAdaptiveVoxelActor::CanEditChange(const FProperty* InProperty) const
{
    if (!Super::CanEditChange(InProperty))
        return false;

    if (bIsPlanetOwned && InProperty)
    {
        const FName PropName = InProperty->GetFName();
        // Lock all transform components - position, rotation, scale are driven by the planet.
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

void AAdaptiveVoxelActor::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
}

void AAdaptiveVoxelActor::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
}

void AAdaptiveVoxelActor::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
}

void AAdaptiveVoxelActor::PostEditMove(bool bFinished)
{
    if (bIsPlanetOwned)
    {
        // Snap all transforms back - location and rotation follow parent, scale is planet-driven.
        if (USceneComponent* Root = GetRootComponent())
        {
            Root->SetRelativeLocation(FVector::ZeroVector);
            Root->SetRelativeRotation(FRotator::ZeroRotator);
        }
    }
    Super::PostEditMove(bFinished);
}
#endif

void AAdaptiveVoxelActor::BeginPlay()
{
    Super::BeginPlay();
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;
    // A terrain spawned at run time was built by its construction already.
    if (!Initialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
        Initialize();
}

void AAdaptiveVoxelActor::CleanSceneRoot()
{
    // Components left from an earlier run (saved with the level, or copied from the editor
    // world into PIE) would otherwise keep rendering or colliding without anything tracking
    // them.
    if (!BrickAttachmentRoot) return;
    const TArray<TObjectPtr<USceneComponent>> Attached = BrickAttachmentRoot->GetAttachChildren();
    for (const TObjectPtr<USceneComponent>& Child : Attached)
    {
        if (URealtimeMeshComponent* MeshComponent = Cast<URealtimeMeshComponent>(Child))
            MeshComponent->DestroyComponent();
    }
}

UMaterialInterface* AAdaptiveVoxelActor::DefaultSurfaceMaterial()
{
    UMaterialInterface* PluginMaterial = Cast<UMaterialInterface>(StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
        TEXT("/VoxelPlugin/MT_TriPlanar.MT_TriPlanar"), nullptr, LOAD_NoWarn | LOAD_Quiet));
    return PluginMaterial ? PluginMaterial : UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
}

void AAdaptiveVoxelActor::SetSurfaceMaterial(UMaterialInterface* InMaterial)
{
    SurfaceMaterial = InMaterial ? InMaterial : DefaultSurfaceMaterial();
    if (BrickTerrain.IsValid())
    {
        BrickTerrain->SetMaterial(ResolveBrickMaterial());
        BrickTerrain->SetMaterialLandViews(bLandMaterialBound);
    }
}

bool AAdaptiveVoxelActor::ResolveLandCache(const TSharedPtr<const FVoxelDensityCPU>& Density)
{
    if (!Density.IsValid()) return false;
    if (LandCache.IsValid() && LandCache->IsInitialized() && LandCacheDensity.Pin() == Density)
        return true;
    // A failure is remembered (as the land textures'), so a later call doesn't try again.
    if (LandCacheFailed.Pin() == Density) return false;
    // The textures fit the levels of the bake's radius (kept across densities with the same).
    const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake = Density->GetLandBake();
    if (!Bake.IsValid()) return false;
    const int32 Levels = FVoxelLandCache::LevelsFor(Bake->SeaRadius);
    const int32 Capacity = LandCacheHeight ? (LandCacheHeight->SizeY / VOXEL_LAND_CACHE_N) * VOXEL_LAND_CACHE_COLUMNS : 0;
    if (!LandCacheHeight || !LandCacheSlope || !LandCacheFields || !LandCacheState || Capacity < FVoxelLandCache::TilesFor(Levels))
    {
        FVoxelLandCacheTextures Made;
        if (!FVoxelLandCache::MakeTextures(Levels, this, Made))
        {
            ReleaseLandCache();
            LandCacheFailed = Density;
            return false;
        }
        LandCacheHeight = Made.Height;
        LandCacheSlope = Made.Slope;
        LandCacheFields = Made.Fields;
        LandCacheState = Made.State;
    }
    FVoxelLandCacheTextures Textures;
    Textures.Height = LandCacheHeight;
    Textures.Slope = LandCacheSlope;
    Textures.Fields = LandCacheFields;
    Textures.State = LandCacheState;
    if (!LandCache.IsValid())
        LandCache = MakeUnique<FVoxelLandCache>();
    if (!LandCache->Initialize(*Density, Textures))
    {
        ReleaseLandCache();
        LandCacheFailed = Density;
        return false;
    }
    LandCacheDensity = Density;
    return true;
}

void AAdaptiveVoxelActor::ReleaseLandCache()
{
    if (LandCache.IsValid())
        LandCache->Reset();
    // The instance lets go of the atlases too (so they can be collected).
    if (LandMaterialInstance)
    {
        LandMaterialInstance->SetTextureParameterValue(FVoxelLandCache::HeightTextureName, nullptr);
        LandMaterialInstance->SetTextureParameterValue(FVoxelLandCache::SlopeTextureName, nullptr);
        LandMaterialInstance->SetTextureParameterValue(FVoxelLandCache::FieldsTextureName, nullptr);
        LandMaterialInstance->SetTextureParameterValue(FVoxelLandCache::StateTextureName, nullptr);
    }
    LandCacheDensity.Reset();
    LandCacheHeight = nullptr;
    LandCacheSlope = nullptr;
    LandCacheFields = nullptr;
    LandCacheState = nullptr;
}

void AAdaptiveVoxelActor::ReleaseLandTextures()
{
    ReleaseLandCache();
    LandHeightTexture = nullptr;
    LandFieldTexture = nullptr;
    LandConstantTexture = nullptr;
    LandTexturesBake.Reset();
    LandTexturesDensity.Reset();
    // The instance lets go of them too (the frame parameters are set again every frame).
    if (LandMaterialInstance)
        LandMaterialInstance->ClearParameterValues();
}

UMaterialInterface* AAdaptiveVoxelActor::ResolveBrickMaterial()
{
    bLandMaterialBound = false;
    if (!SurfaceMaterial)
    {
        ReleaseLandTextures();
        LandMaterialInstance = nullptr;
        LandMaterialParent = nullptr;
        return SurfaceMaterial;
    }
    // The bricks always draw an instance: with the land node off (VoxelLandParams.z 0) unless the
    // textures are bound, whatever the material's own default.
    if (!LandMaterialInstance || LandMaterialParent != SurfaceMaterial)
    {
        LandMaterialInstance = UMaterialInstanceDynamic::Create(SurfaceMaterial, this);
        LandMaterialInstance->SetFlags(RF_Transient | RF_DuplicateTransient);
        LandMaterialParent = SurfaceMaterial;
    }

    // The textures, on a terrestrial planet whose material has the land node: the bake's (kept
    // while the bake is, across densities rebuilt with other sample settings) and the density's
    // constants. A failure is remembered, so a later call doesn't build them again.
    const TSharedPtr<const FVoxelDensityCPU> Density = DensityCompositor.IsValid() ? DensityCompositor->GetBaseDensity() : nullptr;
    const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake = Density.IsValid() ? Density->GetLandBake() : nullptr;
    if (!FApp::CanEverRender() || !Bake.IsValid() || !VoxelLandMaterial::HasLandNode(SurfaceMaterial) || LandTexturesFailed.Pin() == Density)
    {
        ReleaseLandTextures();
    }
    else
    {
        bool bFailed = false;
        if (LandTexturesBake.Pin() != Bake || !LandHeightTexture || !LandFieldTexture)
        {
            UTexture2DDynamic* Height = nullptr;
            UTexture2DDynamic* Fields = nullptr;
            bFailed = !VoxelLandMaterial::MakeBakeTextures(Bake, Height, Fields);
            LandHeightTexture = Height;
            LandFieldTexture = Fields;
            LandTexturesBake = Bake;
            LandTexturesDensity.Reset();
        }
        if (!bFailed && (LandTexturesDensity.Pin() != Density || !LandConstantTexture))
        {
            LandConstantTexture = VoxelLandMaterial::MakeConstantTexture(*Density);
            LandTexturesDensity = Density;
            bFailed = !LandConstantTexture;
        }
        if (bFailed)
        {
            ReleaseLandTextures();
            LandTexturesFailed = Density;
        }
        else
        {
            VoxelLandMaterial::SetTextureParameters(LandMaterialInstance, LandHeightTexture, LandFieldTexture, LandConstantTexture);
            bLandMaterialBound = true;
            // The land surface cache the node reads by default (without it, the analytic node).
            if (ResolveLandCache(Density))
                LandCache->SetTextureParameters(LandMaterialInstance);
        }
    }

    // The frame every frame, once all actors (the planet among them) have moved.
    if (!LandMaterialTickHandle.IsValid())
        LandMaterialTickHandle = FWorldDelegates::OnWorldPostActorTick.AddUObject(this, &AAdaptiveVoxelActor::OnWorldPostActorTick);
    UpdateLandMaterial();
    return LandMaterialInstance;
}

void AAdaptiveVoxelActor::OnWorldPostActorTick(UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
    if (World == GetWorld() && !IsDestroyed)
        UpdateLandMaterial(true);
}

void AAdaptiveVoxelActor::UpdateLandMaterial(bool bTickCache)
{
    if (!LandMaterialInstance) return;
    const FTransform PlanetFrame(GetActorRotation(), GetActorLocation());
    // The land surface cache follows the primary camera, kept warm whenever it's bound (so
    // voxel.Land.MaterialSource switches back to it at once): from the end of the actors' tick
    // only, when the cameras have moved. Without the write shader or the textures' resources,
    // the material evaluates the land itself.
    const bool bCache = bLandMaterialBound && LandCache.IsValid() && LandCache->IsWritable();
    if (bCache && bTickCache)
    {
        // The edits the cache's writes apply: the mesh's (the cache rewrites an edit's texels
        // once these carry it, so both change together).
        if (BrickTerrain.IsValid())
            LandCache->SetEdits(BrickTerrain->GetGPUEdits(), BrickTerrain->GetGPUEditsSequence());
        FVector CameraPlanet;
        if (GetLandCacheCamera(PlanetFrame, CameraPlanet))
            LandCache->Tick(CameraPlanet);
        if (VoxelLandMaterial::ShowsCacheLevels() && GEngine)
            GEngine->AddOnScreenDebugMessage((uint64)(UPTRINT)this + 0x1a7dcac4eull, 0.0f, FColor::Cyan, LandCache->Describe());
    }
    // The planet's frame as it renders this frame (every actor has moved); the material places
    // each pixel from its own view.
    VoxelLandMaterial::SetFrameParameters(LandMaterialInstance, PlanetFrame, FVoxelBrickTerrain::GetMaterialLandView(), LandSurfaceTolerance,
        LandNearMeshNormal * ActualPrecision, bLandMaterialBound, bCache);
}

bool AAdaptiveVoxelActor::GetLandCacheCamera(const FTransform& PlanetFrame, FVector& OutCameraPlanet) const
{
    const UWorld* World = GetWorld();
    if (!World) return false;
    // A game (and play in editor): the first local player's camera, this frame's (the cameras
    // have updated by the end of the actors' tick).
    if (World->IsGameWorld())
    {
        if (const APlayerCameraManager* Camera = UGameplayStatics::GetPlayerCameraManager(World, 0))
        {
            OutCameraPlanet = PlanetFrame.InverseTransformPositionNoScale(Camera->GetCameraLocation());
            return true;
        }
    }
    // The editor (or no player): of the views the last frame rendered, the one nearest the
    // planet's surface (the viewport looking at it, not a far one framing the planet).
    const double Sea = LandCache.IsValid() ? LandCache->GetSeaRadius() : 0.0;
    double Best = TNumericLimits<double>::Max();
    bool bFound = false;
    for (const FVector& View : World->ViewLocationsRenderedLastFrame)
    {
        const FVector Local = PlanetFrame.InverseTransformPositionNoScale(View);
        const double Distance = FMath::Abs(Local.Size() - Sea);
        if (Distance < Best)
        {
            Best = Distance;
            OutCameraPlanet = Local;
            bFound = true;
        }
    }
    return bFound;
}

void AAdaptiveVoxelActor::SetDebugSeaRadius(double InSeaRadius)
{
    DebugSeaRadius = FMath::Max(InSeaRadius, 0.0);
    if (BrickTerrain.IsValid())
        BrickTerrain->SetDebugSeaRadius(DebugSeaRadius);
}

void AAdaptiveVoxelActor::Initialize()
{
    Initialized = false;

    // The plugin's triplanar material for standalone use, when none is assigned (the
    // constructor sets the engine default as a fallback).
    if (!SurfaceMaterial || SurfaceMaterial == UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface))
        SurfaceMaterial = DefaultSurfaceMaterial();

    // The terrain is built at world scale in actor-local space; the actor's scale is the
    // planet radius.
    const double ActorPlanetRadius = GetActorScale3D().GetMax();
    const double ActorNoiseAmplitude = ActorPlanetRadius * NoiseAmplitudeRatio;
    const double ActorRootExtent = VoxelDensity::RootExtentFor(ActorPlanetRadius, ActorNoiseAmplitude);
    const int32 Depth = MaxDepthFor(ActorRootExtent, TargetPrecision);
    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(FVector::ZeroVector, ActorRootExtent, Depth);
    Rebuild(VoxelDensity::MakeCompositor(SurfaceSettings, ActorPlanetRadius, ActorNoiseAmplitude, EditStore,
        VoxelDensity::TerrainFinestCell(ActorPlanetRadius, ActorNoiseAmplitude, Depth)));
}

void AAdaptiveVoxelActor::Rebuild(TSharedPtr<FDensitySampleCompositor> Compositor)
{
    Initialized = false;
    while (EditQueue->bRunning)
        FPlatformProcess::Sleep(0.001f);
    {
        FScopeLock Lock(&EditQueue->CS);
        EditQueue->Notices.Empty();
    }
    CleanSceneRoot();

    const double ActorPlanetRadius = GetActorScale3D().GetMax();
    const double ActorRootExtent = VoxelDensity::RootExtentFor(ActorPlanetRadius, ActorPlanetRadius * NoiseAmplitudeRatio);
    MaxDepth = MaxDepthFor(ActorRootExtent, TargetPrecision);
    ActualPrecision = 2.0 * ActorRootExtent / FMath::Pow(2.0, (double)MaxDepth);

    DensityCompositor = Compositor;
    InitializeCollision(ActorRootExtent);
    InitializeBrickTerrain(ActorRootExtent);
    LastInitScale = GetActorScale3D();
    Initialized = true;
}

void AAdaptiveVoxelActor::OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport)
{
    if (!bIsPlanetOwned) return;

    // Snap everything back: location and rotation follow parent, scale is planet-driven.
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

void AAdaptiveVoxelActor::InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
    USceneComponent* InAttachParent, FVector InScale)
{
    Initialized = false;
    bIsPlanetOwned = true;

    // Apply scale BEFORE binding the transform guard - prevents the old guard
    // from reverting a scale change that the planet actor intends.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.RemoveAll(this);

    if (!InScale.IsZero())
        SetActorScale3D(InScale);

    PlanetDrivenScale = GetActorScale3D();

    // Re-bind the guard now that PlanetDrivenScale reflects the new scale.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.AddUObject(this, &AAdaptiveVoxelActor::OnTransformUpdated);

    // The bricks follow the planet's position and rotation directly.
    if (InAttachParent && BrickAttachmentRoot)
    {
        BrickAttachmentRoot->AttachToComponent(InAttachParent,
            FAttachmentTransformRules::KeepWorldTransform);
    }

    // The planet's compositor: shared density and edit store.
    Rebuild(InCompositor);
}

void AAdaptiveVoxelActor::InitializeCollision(double RootExtent)
{
    if (!CollisionBricks.IsValid())
        CollisionBricks = MakeShared<FVoxelCollisionBricks>();

    // The finest terrain lattice: corners at -RootExtent + k * ActualPrecision on each axis.
    FVoxelBrickGrid Grid;
    Grid.Origin = FVector(-RootExtent);
    Grid.CellSize = ActualPrecision * FMath::Max(1, CollisionCellScale);
    Grid.CellsPerBrick = 16;
    CollisionBricks->Initialize(this, BrickAttachmentRoot, DensityCompositor, Grid, CollisionSlopeBound);
}

void AAdaptiveVoxelActor::InitializeBrickTerrain(double RootExtent)
{
    if (!BrickTerrain.IsValid())
        BrickTerrain = MakeShared<FVoxelBrickTerrain>();

    // Bricks of 2^k cells: level MaxDepth - k is a single brick covering the whole root.
    const int32 BrickLog2 = FMath::Clamp(FMath::RoundToInt(FMath::Log2((double)FMath::Max(BrickCellsPerBrick, 1))), 3, 6);
    // The property shows the size built (8, 16, 32 or 64), not a value rounded away.
    BrickCellsPerBrick = 1 << BrickLog2;
    FVoxelBrickTerrainSettings Settings;
    Settings.Origin = FVector(-RootExtent);
    Settings.FinestCell = ActualPrecision;
    Settings.CellsPerBrick = 1 << BrickLog2;
    Settings.MaxLevel = FMath::Max(0, MaxDepth - BrickLog2);
    Settings.LodDistanceFactor = BrickLodDistanceFactor;
    Settings.SlopeBound = CollisionSlopeBound;
    Settings.TriplanarUVScale = 0.0001 * (80000000.0 / FMath::Max(GetActorScale3D().GetMax(), 1.0));
    Settings.Material = ResolveBrickMaterial();
    Settings.bMaterialLandViews = bLandMaterialBound;
    Settings.DebugSeaRadius = DebugSeaRadius;
    // The plugin's collection when none is assigned (the planet spawns this actor).
    if (!BrickMorphParameters)
        BrickMorphParameters = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), nullptr,
            TEXT("/VoxelPlugin/MPC_VoxelMorph.MPC_VoxelMorph"), nullptr, LOAD_NoWarn | LOAD_Quiet));
    Settings.MorphParameters = BrickMorphParameters.Get();
    BrickTerrain->Initialize(this, BrickAttachmentRoot, DensityCompositor, Settings);
}

void AAdaptiveVoxelActor::LogEditHit(const FHitResult& Hit, const FVector& LocalHit) const
{
    static IConsoleVariable* DebugCollision = IConsoleManager::Get().FindConsoleVariable(TEXT("voxel.Debug.Collision"));
    if (!DebugCollision || DebugCollision->GetInt() <= 0 || !CollisionBricks.IsValid()) return;
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelCollision] Edit trace: %s"), *CollisionBricks->DescribeHit(Hit.GetComponent(), LocalHit));
}

void AAdaptiveVoxelActor::TickCollision(UWorld* World)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelCollisionTick);
    if (!CollisionBricks.IsValid() || !World) return;

    TArray<FEditNotice> Notices;
    {
        FScopeLock Lock(&EditQueue->CS);
        Notices = MoveTemp(EditQueue->Notices);
        EditQueue->Notices.Reset();
    }
    for (const FEditNotice& Notice : Notices)
    {
        // The collision bricks and the listeners (the ocean) take a sphere and add their own
        // reach of twice its radius: half the box's half diagonal covers the box, and half the
        // edit index cell's diagonal the values around it that the cells listing the edit
        // change (deep inside solid or air; FSparseEditStore::Apply).
        const TSharedPtr<FSparseEditStore> Store = DensityCompositor.IsValid() ? DensityCompositor->GetEditStore() : nullptr;
        const double IndexDiagonal = Store.IsValid() ? Store->GetIndexCellSize() * FMath::Sqrt(3.0) : 0.0;
        const FVector Center = Notice.Bounds.GetCenter();
        const double Radius = 0.5 * (Notice.Bounds.GetExtent().Size() + IndexDiagonal);
        CollisionBricks->NotifyEdit(Notice.Sequence, Center, Radius);
        if (BrickTerrain.IsValid())
            BrickTerrain->NotifyEdit(Notice.Sequence, Notice.Bounds);
        // The land surface cache's columns through the box (its frame is the actor's).
        if (LandCache.IsValid())
            LandCache->NotifyEdit(Notice.Sequence, Notice.Bounds);
        TerrainEdited.Broadcast(Notice.Sequence, Center, Radius);
    }

    TArray<FVoxelCollisionSource> Sources;
    if (bEnableSurfaceCollision)
    {
        const double MaxRadius = FMath::Max(MaxCollisionDistance, WalkBaseCollisionDistance);
        auto RadiusFor = [this, MaxRadius](double Speed)
            {
                return FMath::Min(WalkBaseCollisionDistance + Speed * CollisionCookLeadTime, MaxRadius);
            };

        const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
        for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
        {
            const APlayerController* PC = It->Get();
            const APawn* Pawn = PC ? PC->GetPawn() : nullptr;
            if (!Pawn) continue;
            // A pawn moved by setting its location reports no velocity; the local camera's
            // smoothed speed covers that case.
            const double Speed = FMath::Max((double)Pawn->GetVelocity().Size(), PC->IsLocalController() ? PlayerSpeed : 0.0);
            Sources.Add({ NoScaleTransform.InverseTransformPosition(Pawn->GetActorLocation()), RadiusFor(Speed) });
        }
        if (Sources.Num() == 0 && bHasSpeedSample)
            Sources.Add({ CameraPosition, RadiusFor(PlayerSpeed) });
    }

    CollisionBricks->Tick(Sources, CollisionHysteresis);
    if (BrickAttachmentRoot)
        CollisionBricks->DrawDebug(World, BrickAttachmentRoot->GetComponentTransform());
}

void AAdaptiveVoxelActor::RunEditUpdateTask(EVoxelEditMode Mode, FVector InEditCenter, double InEditRadius, double InEditBlend, bool bContinueStroke, bool bStretch)
{
    if (EditQueue->bRunning || IsDestroyed || !DensityCompositor.IsValid()) return;
    const TSharedPtr<FSparseEditStore> Store = DensityCompositor->GetEditStore();
    if (!Store.IsValid()) return;

    EditQueue->bRunning = true;
    FFunctionGraphTask::CreateAndDispatchWhenReady([Queue = EditQueue, Store, Mode, InEditCenter, InEditRadius, InEditBlend, bContinueStroke, bStretch]()
        {
            const double StartTime = FPlatformTime::Seconds();
            const FVoxelEditResult Result = Store->ApplyEdit(Mode, InEditCenter, InEditRadius, InEditBlend, bContinueStroke, bStretch);

            // The collision bricks and the brick terrain rebuild what this edit reaches (nothing
            // when it changed nothing: repeated, or the brush stayed put).
            if (Result.bChanged)
            {
                FScopeLock Lock(&Queue->CS);
                Queue->Notices.Add({ Result.Sequence, Result.Bounds });
            }
            if (Result.Dropped > 0)
            {
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelEdit] Edit %u made %d earlier edits irrelevant; the store holds %d."), Result.Sequence, Result.Dropped, Store->Num());
            }

            const double ElapsedMs = (FPlatformTime::Seconds() - StartTime) * 1000.0;
            if (ElapsedMs > EditLogThresholdMs)
            {
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelEdit] Edit applied in %.2f ms"), ElapsedMs);
            }

            // Cleared last: Rebuild waits on the flag and then clears the notices, so a
            // notice for a replaced store never lands after that clear.
            Queue->bRunning = false;
        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

void AAdaptiveVoxelActor::RunEndStrokeTask()
{
    if (EditQueue->bRunning || IsDestroyed || !DensityCompositor.IsValid()) return;
    const TSharedPtr<FSparseEditStore> Store = DensityCompositor->GetEditStore();
    if (!Store.IsValid()) return;

    EditQueue->bRunning = true;
    FFunctionGraphTask::CreateAndDispatchWhenReady([Queue = EditQueue, Store]()
        {
            // The surface doesn't change: no notice.
            const FVoxelEditResult Result = Store->EndStroke();
            if (Result.Dropped > 0)
            {
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelEdit] The stretched stroke made %d earlier edits irrelevant; the store holds %d."), Result.Dropped, Store->Num());
            }
            Queue->bRunning = false;
        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

bool AAdaptiveVoxelActor::ShouldTickIfViewportsOnly() const
{
    return bTickInEditor && Initialized;
}

void AAdaptiveVoxelActor::Tick(float DeltaTime)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelActorTick);
    Super::Tick(DeltaTime);
    if (!Initialized) return;

    // Cache cam data
    auto world = GetWorld();
    if (world != nullptr)
    {
        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0)
        {
            // Convert world-space camera to actor-local space.
            // Position and rotation only: the terrain is built at world scale.
            FVector WorldCamPos = viewLocations[0];
            FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
            this->CameraPosition = NoScaleTransform.InverseTransformPosition(WorldCamPos);

            // Smoothed player speed (cm/s) for the collision lead distance. Smoothing avoids
            // a single stutter frame briefly inflating the collision shell.
            if (bHasSpeedSample && DeltaTime > KINDA_SMALL_NUMBER)
            {
                const double Inst = (CameraPosition - PrevSpeedSamplePos).Size() / DeltaTime;
                PlayerSpeed = FMath::FInterpTo(PlayerSpeed, Inst, DeltaTime, 4.0);
            }
            PrevSpeedSamplePos = CameraPosition;
            bHasSpeedSample = true;
        }

        TickCollision(world);
        if (BrickTerrain.IsValid())
            BrickTerrain->Tick(CameraPosition);

#if !UE_BUILD_SHIPPING
        if (world->IsGameWorld())
            TickDebugEdits(world);
#endif
    }
}

#if !UE_BUILD_SHIPPING
void AAdaptiveVoxelActor::TickDebugEdits(UWorld* World)
{
    // Example of the edit flow; a real tool would run this off the tick. Only this terrain's
    // collision bricks count as a hit, so with several planets each edits its own surface.
    APlayerController* PC = UGameplayStatics::GetPlayerController(World, 0);
    if (!PC) return;
    const bool bCarve = PC->IsInputKeyDown(EKeys::E);
    const bool bFill = PC->IsInputKeyDown(EKeys::Q);
    const bool bStretch = CVarVoxelDebugEditStretch.GetValueOnGameThread() != 0;
    if (!bCarve && !bFill)
    {
        // Released: the next press starts a new stroke; a stretched one is final now.
        if (DebugStrokeMode >= 0 && bDebugStretching)
            bDebugStrokeEndPending = true;
        DebugStrokeMode = -1;
        bDebugStretching = false;
        if (bDebugStrokeEndPending && !EditQueue->bRunning)
        {
            RunEndStrokeTask();
            bDebugStrokeEndPending = false;
        }
        return;
    }
    // The previous edit is still being written: the stroke goes on with the next one.
    if (EditQueue->bRunning) return;
    // A stroke released and pressed again before its end ran: ended first.
    if (bDebugStrokeEndPending)
    {
        RunEndStrokeTask();
        bDebugStrokeEndPending = false;
        return;
    }

    const EVoxelEditMode Mode = bCarve ? EVoxelEditMode::Carve : EVoxelEditMode::Fill;
    const double EditRadius = 300.0;
    const double EditBlend = 0.25 * EditRadius;
    const float DebugDrawTime = 0.1f;
    const FColor Color = bCarve ? FColor::Red : FColor::Green;

    const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
    const FVector Start = NoScaleTransform.TransformPosition(CameraPosition);
    const FVector Direction = PC->GetControlRotation().Vector();

    // Stretching: the capsule's end at the pressed distance along the view (no trace: the
    // trace would hit the end of the tunnel being dug, and run away from the brush).
    if (bDebugStretching && DebugStrokeMode == (int32)Mode)
    {
        const FVector Point = Start + Direction * DebugStretchDistance;
        RunEditUpdateTask(Mode, NoScaleTransform.InverseTransformPosition(Point), EditRadius, EditBlend, true, true);
        DrawDebugSphere(World, Point, (float)EditRadius, 32, Color, false, DebugDrawTime);
        return;
    }
    // The other key while stretching: that stroke ends first.
    if (bDebugStretching && DebugStrokeMode >= 0)
    {
        RunEndStrokeTask();
        DebugStrokeMode = -1;
        bDebugStretching = false;
        return;
    }

    const FVector End = Start + Direction * (GetActorScale3D().GetMax() * 3.0);
    FHitResult Hit;
    if (!World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility) || Hit.GetActor() != this)
    {
        // Off the terrain: the stroke breaks (it would otherwise bridge the gap).
        DebugStrokeMode = -1;
        return;
    }

    const FVector LocalHit = NoScaleTransform.InverseTransformPosition(Hit.ImpactPoint);
    LogEditHit(Hit, LocalHit);
    // Stretching starts with a sphere where the trace hits, within 50 brush radii (farther, a
    // turn of the view would swing the capsule's end by hundreds of metres); otherwise a
    // segment stroke goes on along the surface the trace hits.
    const double Distance = (Hit.ImpactPoint - Start).Size();
    const bool bContinueStroke = DebugStrokeMode == (int32)Mode;
    RunEditUpdateTask(Mode, LocalHit, EditRadius, EditBlend, bContinueStroke, false);
    DebugStrokeMode = (int32)Mode;
    bDebugStretching = bStretch && Distance <= 50.0 * EditRadius;
    DebugStretchDistance = Distance;
    DrawDebugSphere(World, Hit.ImpactPoint, (float)EditRadius, 32, Color, false, DebugDrawTime);
}
#endif
