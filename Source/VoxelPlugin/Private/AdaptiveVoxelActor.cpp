#include "AdaptiveVoxelActor.h"
#include "PlanetActor.h"
#include "VoxelPlugin.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include "FDensitySampleCompositor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeLock.h"
#include "FSparseEditStore.h"
#include "RealtimeMeshComponent.h"
#include "VoxelTerrainStats.h"
#include "VoxelLandMaterial.h"
#include "Engine/Texture2DDynamic.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/World.h"
#include "Misc/App.h"

DECLARE_CYCLE_STAT(TEXT("Actor tick"), STAT_VoxelActorTick, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Collision tick"), STAT_VoxelCollisionTick, STATGROUP_VoxelTerrain);

// An edit task taking longer than this (ms) is logged.
static constexpr double EditLogThresholdMs = 50.0;

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

void AAdaptiveVoxelActor::ReleaseLandTextures()
{
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
        UpdateLandMaterial();
}

void AAdaptiveVoxelActor::UpdateLandMaterial()
{
    if (!LandMaterialInstance) return;
    // The reference point: the camera the last frame rendered from (any point near the pixels
    // keeps their offsets small; the editor's viewports count, ticking or not).
    const UWorld* World = GetWorld();
    if (World && World->ViewLocationsRenderedLastFrame.Num() > 0)
    {
        LandReferenceWorld = World->ViewLocationsRenderedLastFrame[0];
        bHasLandReference = true;
    }
    // Until a view exists the node stays off: from the planet's centre the pixels' offsets
    // would be too large for floats.
    VoxelLandMaterial::SetFrameParameters(LandMaterialInstance, FTransform(GetActorRotation(), GetActorLocation()), LandReferenceWorld,
        FVoxelBrickTerrain::GetMaterialLandView(), LandSurfaceTolerance, bLandMaterialBound && bHasLandReference);
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
        CollisionBricks->NotifyEdit(Notice.Sequence, Notice.Center, Notice.Radius);
        if (BrickTerrain.IsValid())
            BrickTerrain->NotifyEdit(Notice.Sequence, Notice.Center, Notice.Radius);
        TerrainEdited.Broadcast(Notice.Sequence, Notice.Center, Notice.Radius);
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

void AAdaptiveVoxelActor::RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    if (EditQueue->bRunning || IsDestroyed || !DensityCompositor.IsValid()) return;
    const TSharedPtr<FSparseEditStore> Store = DensityCompositor->GetEditStore();
    if (!Store.IsValid()) return;

    EditQueue->bRunning = true;
    FFunctionGraphTask::CreateAndDispatchWhenReady([Queue = EditQueue, Store, InEditCenter, InEditRadius, InEditStrength, InEditResolution]()
        {
            const double StartTime = FPlatformTime::Seconds();
            Store->ApplySphericalEdit(InEditCenter, InEditRadius, InEditStrength, Store->GetDepthForBrushRadius(InEditRadius, InEditResolution));

            // The collision bricks and the brick terrain rebuild what this edit reaches.
            {
                FScopeLock Lock(&Queue->CS);
                Queue->Notices.Add({ Store->GetLastSequence(), InEditCenter, InEditRadius });
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
    if (!PC || EditQueue->bRunning) return;
    const bool bAdd = PC->IsInputKeyDown(EKeys::E);
    const bool bRemove = PC->IsInputKeyDown(EKeys::Q);
    if (!bAdd && !bRemove) return;

    const double EditRadius = 300.0;
    const double EditStrength = 600.0;
    const int32 EditResolution = 3;
    const float DebugDrawTime = 0.1f;

    const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
    const FVector Start = NoScaleTransform.TransformPosition(CameraPosition);
    const FVector End = Start + PC->GetControlRotation().Vector() * (GetActorScale3D().GetMax() * 3.0);
    FHitResult Hit;
    if (!World->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility) || Hit.GetActor() != this) return;

    const FVector LocalHit = NoScaleTransform.InverseTransformPosition(Hit.ImpactPoint);
    LogEditHit(Hit, LocalHit);
    RunEditUpdateTask(LocalHit, EditRadius, bAdd ? EditStrength : -EditStrength, EditResolution);
    DrawDebugSphere(World, Hit.ImpactPoint, (float)EditRadius, 32, bAdd ? FColor::Red : FColor::Green, false, DebugDrawTime);
}
#endif
