#include "PlanetActor.h"
#include "PlanetGravityZone.h"
#include "Kismet/GameplayStatics.h"

APlanetActor::APlanetActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    PlanetRoot = CreateDefaultSubobject<USceneComponent>(TEXT("PlanetRoot"));
    SetRootComponent(PlanetRoot);

    // Default planet radius: 1000 km = 100,000,000 cm
    SetActorScale3D(FVector(100000000.0));

#if WITH_EDITORONLY_DATA
    // Dragging the planet with the gizmo reconstructs it once, on release (a scale drag
    // would otherwise rebuild the terrain and ocean every frame).
    bRunConstructionScriptOnDrag = false;
#endif
}

void APlanetActor::BeginPlay()
{
    Super::BeginPlay();
    // bPendingInitialize defaults to true - first Tick calls Initialize.
}

void APlanetActor::Destroyed()
{
    // Destroyed() fires when the actor is explicitly deleted - both in-editor
    // and at runtime. The world is still valid here, so Destroy() on children works.
    DestroyChildActors();
    Super::Destroyed();
}

void APlanetActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    DestroyChildActors();
    Super::EndPlay(EndPlayReason);
}

void APlanetActor::BeginDestroy()
{
    // Fallback - catch anything EndPlay missed (shouldn't happen, but safe).
    DestroyChildActors();
    Super::BeginDestroy();
}

void APlanetActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (!GetWorld() || GetWorld()->IsPreviewWorld()) return;

    // A slider or a gizmo drag still moving: the release flags the work once.
    if (bInteractiveEdit) return;
    FlagPendingWork();
}

void APlanetActor::FlagPendingWork()
{
    // Flag for deferred work in Tick - never spawn actors during OnConstruction.
    if (!bInitialized
        || !GetActorScale3D().Equals(LastInitScale, 0.01)
        || NoiseAmplitudeRatio != LastNoiseAmplitudeRatio
        || SurfaceSettings.GetSettingsHash() != LastSurfaceSettingsHash)
    {
        bPendingInitialize = true;
    }
    else if (SeaLevel != LastSeaLevel
        || bEnableOcean != bLastEnableOcean
        || bEnableGravity != bLastEnableGravity)
    {
        bPendingOceanUpdate = true;
    }
}

#if WITH_EDITOR
void APlanetActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    // The construction rerun inside Super sees whether a slider is still moving.
    const bool bInteractive = PropertyChangedEvent.ChangeType == EPropertyChangeType::Interactive;
    bInteractiveEdit = bInteractive;
    Super::PostEditChangeProperty(PropertyChangedEvent);
    bInteractiveEdit = false;
    if (!bInitialized) return;
    // A PIE world doesn't rerun construction after an edit; flagging twice is harmless.
    if (!bInteractive)
        FlagPendingWork();

    const FName PropName = PropertyChangedEvent.GetPropertyName();
    const FName MemberName = PropertyChangedEvent.MemberProperty ? PropertyChangedEvent.MemberProperty->GetFName() : PropName;

    // The terrain material changes in place (no rebuild).
    if (PropName == GET_MEMBER_NAME_CHECKED(APlanetActor, TerrainMaterial))
    {
        if (TerrainActor)
            TerrainActor->SetSurfaceMaterial(TerrainMaterial);
        return;
    }

    // So does the ocean's.
    if (PropName == GET_MEMBER_NAME_CHECKED(APlanetActor, OceanMaterial))
    {
        if (OceanActor)
            OceanActor->SetOceanMaterial(OceanMaterial);
        return;
    }

    // The ocean's settings apply in place, as far as they can (a change inside the struct
    // reports the struct as the member property); slider drags apply on release.
    if (MemberName == GET_MEMBER_NAME_CHECKED(APlanetActor, OceanSettings))
    {
        // The planet shows what the ocean builds (TileQuads snapped, limits) once released.
        if (!bInteractive)
            OceanSettings.Sanitize();
        if (OceanActor)
            OceanActor->SetOceanSettings(OceanSettings, bInteractive);
        return;
    }

    // Scale, NoiseAmplitudeRatio, SurfaceSettings, SeaLevel, and toggle changes are handled
    // by FlagPendingWork via the bPendingInitialize / bPendingOceanUpdate flags. Anything else
    // (an undo reports no property) forwards the children's settings again.
    if (!bInteractive)
        ForwardChildSettings();
}

void APlanetActor::PostEditMove(bool bFinished)
{
    // A gizmo drag (also on a Blueprint planet, whose asset may rerun construction on drag)
    // flags the work once, when it ends.
    bInteractiveEdit = !bFinished;
    Super::PostEditMove(bFinished);
    bInteractiveEdit = false;
}
#endif

void APlanetActor::ForwardChildSettings()
{
    if (TerrainActor && TerrainMaterial && TerrainActor->SurfaceMaterial != TerrainMaterial)
        TerrainActor->SetSurfaceMaterial(TerrainMaterial);
    if (OceanActor)
    {
        if (OceanMaterial && OceanActor->OceanMaterial != OceanMaterial)
            OceanActor->SetOceanMaterial(OceanMaterial);
        OceanActor->SetOceanSettings(OceanSettings);
    }
}

void APlanetActor::SetOceanSettings(const FVoxelOceanSettings& InSettings)
{
    OceanSettings = InSettings;
    OceanSettings.Sanitize();
    if (OceanActor && bInitialized)
        OceanActor->SetOceanSettings(OceanSettings);
}

void APlanetActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Deferred init - safe to spawn actors here.
    if (bPendingInitialize)
    {
        bPendingInitialize = false;
        bPendingOceanUpdate = false;
        Initialize();
    }
    else if (bPendingOceanUpdate)
    {
        bPendingOceanUpdate = false;
        const double PlanetRadius = GetActorScale3D().GetMax();
        const double NoiseAmplitude = PlanetRadius * NoiseAmplitudeRatio;
        const double OceanRadius = ComputeOceanRadius(PlanetRadius, NoiseAmplitude);
        // The same configuration as the first initialization: the ocean keeps its material and
        // settings, and the gravity zone follows the new sea level. A gravity toggle alone
        // leaves the ocean as it is.
        if (SeaLevel != LastSeaLevel || bEnableOcean != bLastEnableOcean)
            ConfigureOcean(PlanetRadius, NoiseAmplitude, OceanRadius);
        ConfigureGravity(PlanetRadius, OceanRadius);
        RecordAppliedState();
    }
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void APlanetActor::Initialize()
{
    const double PlanetRadius = GetActorScale3D().GetMax();
    const double NoiseAmplitude = PlanetRadius * NoiseAmplitudeRatio;
    const double RootExtent = VoxelDensity::RootExtentFor(PlanetRadius, NoiseAmplitude);
    const double OceanRadius = ComputeOceanRadius(PlanetRadius, NoiseAmplitude);

    SpawnChildActors();
    if (!TerrainActor) return;

    // The terrain lattice depth the terrain actor derives in Rebuild: the edit store and the
    // compositor's finest footprint must match it.
    const int32 TempMaxDepth = AAdaptiveVoxelActor::MaxDepthFor(RootExtent, TerrainActor->TargetPrecision);

    TSharedPtr<FDensitySampleCompositor> NewCompositor =
        BuildCompositor(PlanetRadius, NoiseAmplitude, RootExtent, TempMaxDepth);
    SharedCompositor = NewCompositor;

    // --- Terrain actor ---
    TerrainActor->SetSurfaceMaterial(TerrainMaterial);
    TerrainActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
    TerrainActor->SurfaceSettings = SurfaceSettings;
    TerrainActor->InitializeFromPlanet(SharedCompositor, PlanetRoot, FVector(PlanetRadius));
    // Re-activate after a pool park: shown, ticking, and with collision (dormancy dropped it).
    TerrainActor->SetActorHiddenInGame(false);
    TerrainActor->SetActorTickEnabled(true);
    TerrainActor->SetActorEnableCollision(true);

    ConfigureOcean(PlanetRadius, NoiseAmplitude, OceanRadius);
    ConfigureGravity(PlanetRadius, OceanRadius);

    RecordAppliedState();
    bInitialized = true;
}

void APlanetActor::ConfigureOcean(double PlanetRadius, double NoiseAmplitude, double OceanRadius)
{
    if (!OceanActor) return;
    if (!bEnableOcean)
    {
        OceanActor->SetActorHiddenInGame(true);
        OceanActor->SetActorTickEnabled(false);
#if WITH_EDITOR
        // Hidden in game doesn't hide it in the editor's viewports.
        OceanActor->SetIsTemporarilyHiddenInEditor(true);
#endif
        return;
    }
    OceanActor->OceanMaterial = OceanMaterial;
    OceanActor->TerrainPlanetRadius = PlanetRadius;
    OceanActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
    OceanActor->SurfaceSettings = SurfaceSettings;
    // Before the initialization, which builds the surface with them.
    OceanActor->OceanSettings = OceanSettings;
    OceanActor->InitializeFromPlanet(SharedCompositor, PlanetRoot, FVector(OceanRadius));
    OceanActor->SetActorHiddenInGame(false);
    OceanActor->SetActorTickEnabled(true);
#if WITH_EDITOR
    OceanActor->SetIsTemporarilyHiddenInEditor(false);
#endif
}

void APlanetActor::ConfigureGravity(double PlanetRadius, double OceanRadius)
{
    if (!GravityZone) return;
    if (!bEnableGravity)
    {
        GravityZone->SetActorHiddenInGame(true);
        GravityZone->SetActorTickEnabled(false);
        GravityZone->SetActorEnableCollision(false);
        return;
    }
    // Inner radius = max of ocean/planet radius, so full gravity is reached at the visible
    // surface; set on every update, so a sea level change or a late enable sizes it too.
    GravityZone->SetActorScale3D(FVector(FMath::Max(OceanRadius, PlanetRadius)));
    GravityZone->InitializeFromPlanet();
    GravityZone->SetActorHiddenInGame(false);
    GravityZone->SetActorTickEnabled(true);
    GravityZone->SetActorEnableCollision(true);
}

void APlanetActor::RecordAppliedState()
{
    LastInitScale = GetActorScale3D();
    LastSeaLevel = SeaLevel;
    LastNoiseAmplitudeRatio = NoiseAmplitudeRatio;
    LastSurfaceSettingsHash = SurfaceSettings.GetSettingsHash();
    bLastEnableOcean = bEnableOcean;
    bLastEnableGravity = bEnableGravity;
}

// ---------------------------------------------------------------------------
// Compositor
// ---------------------------------------------------------------------------

TSharedPtr<FDensitySampleCompositor> APlanetActor::BuildCompositor(
    double PlanetRadius, double NoiseAmplitude, double RootExtent, int32 MaxDepth)
{
    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(
        FVector::ZeroVector, RootExtent, MaxDepth);

    return VoxelDensity::MakeCompositor(SurfaceSettings, PlanetRadius, NoiseAmplitude, EditStore,
        VoxelDensity::TerrainFinestCell(PlanetRadius, NoiseAmplitude, MaxDepth));
}

// ---------------------------------------------------------------------------
// Child actor management
// ---------------------------------------------------------------------------

void APlanetActor::SpawnChildActors()
{
    UWorld* World = GetWorld();
    if (!World) return;

    FActorSpawnParameters SpawnParams;
    SpawnParams.Owner = this;
    SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

    // A child another planet owns (this planet is a copy of it) is not ours to configure.
    if (TerrainActor && TerrainActor->GetOwner() != this) TerrainActor = nullptr;
    if (OceanActor && OceanActor->GetOwner() != this) OceanActor = nullptr;
    if (GravityZone && GravityZone->GetOwner() != this) GravityZone = nullptr;

    if (!TerrainActor)
    {
        TerrainActor = World->SpawnActor<AAdaptiveVoxelActor>(
            AAdaptiveVoxelActor::StaticClass(),
            FTransform(GetActorRotation(), GetActorLocation()),
            SpawnParams);
        if (USceneComponent* ChildRoot = TerrainActor->GetRootComponent())
        {
            ChildRoot->SetAbsolute(false, false, true);
            ChildRoot->AttachToComponent(PlanetRoot,
                FAttachmentTransformRules::KeepWorldTransform);
        }
    }

    if (!OceanActor)
    {
        OceanActor = World->SpawnActor<AOceanSphereActor>(
            AOceanSphereActor::StaticClass(),
            FTransform(GetActorRotation(), GetActorLocation()),
            SpawnParams);
        if (USceneComponent* ChildRoot = OceanActor->GetRootComponent())
        {
            ChildRoot->SetAbsolute(false, false, true);
            ChildRoot->AttachToComponent(PlanetRoot,
                FAttachmentTransformRules::KeepWorldTransform);
        }
    }

    if (!GravityZone)
    {
        GravityZone = World->SpawnActor<APlanetGravityZone>(
            APlanetGravityZone::StaticClass(),
            FTransform(GetActorRotation(), GetActorLocation()),
            SpawnParams);
        if (USceneComponent* ChildRoot = GravityZone->GetRootComponent())
        {
            ChildRoot->SetAbsolute(false, false, true);
            ChildRoot->AttachToComponent(PlanetRoot,
                FAttachmentTransformRules::KeepWorldTransform);
        }
    }

    // Load default materials if none set - paths reference plugin Content folder
    if (!TerrainMaterial)
    {
        TerrainMaterial = Cast<UMaterialInterface>(
            StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
                TEXT("/VoxelPlugin/MT_TriPlanar.MT_TriPlanar")));
    }
    if (!OceanMaterial)
    {
        OceanMaterial = Cast<UMaterialInterface>(
            StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
                TEXT("/VoxelPlugin/MT_Ocean.MT_Ocean")));
    }
}

void APlanetActor::DestroyChildActors()
{
    // Only children this planet spawned (see SpawnChildActors).
    if (TerrainActor && TerrainActor->GetOwner() == this)
        TerrainActor->Destroy();
    TerrainActor = nullptr;
    if (OceanActor && OceanActor->GetOwner() == this)
        OceanActor->Destroy();
    OceanActor = nullptr;
    if (GravityZone && GravityZone->GetOwner() == this)
        GravityZone->Destroy();
    GravityZone = nullptr;
}

double APlanetActor::ComputeOceanRadius(double PlanetRadius, double NoiseAmplitude) const
{
    return FMath::Max(1.0, PlanetRadius + SeaLevel * NoiseAmplitude);
}

// ---------------------------------------------------------------------------
// Pooling: dormancy + reconfigure (IPooledActor, proxy-carried)
// ---------------------------------------------------------------------------

void APlanetActor::SetDormant(bool bDormant)
{
    const bool bActive = !bDormant;

    SetActorHiddenInGame(bDormant);
    SetActorTickEnabled(bActive);

    if (TerrainActor)
    {
        TerrainActor->SetActorHiddenInGame(bDormant);
        TerrainActor->SetActorTickEnabled(bActive);   // stops the brick LOD updates and edits
        TerrainActor->SetActorEnableCollision(bActive); // its collision bricks stop updating while parked
    }
    if (OceanActor)
    {
        // Hidden and not ticking: no selection, no commits, no underwater pass. Its pages stay
        // allocated, so waking draws at once.
        const bool bOn = bActive && bEnableOcean;
        OceanActor->SetActorHiddenInGame(!bOn);
        OceanActor->SetActorTickEnabled(bOn);
    }
    if (GravityZone)
    {
        const bool bOn = bActive && bEnableGravity;
        GravityZone->SetActorHiddenInGame(!bOn);
        GravityZone->SetActorTickEnabled(bOn);
        GravityZone->SetActorEnableCollision(bOn);
    }
}

void APlanetActor::SetStarWorldPosition(const FVector& StarWorldPos)
{
    StarWorldPosition = StarWorldPos;
    bHasStarWorldPosition = true;
}

void APlanetActor::OnReturnToPool()
{
    SetDormant(true);
}

void APlanetActor::OnAcquired(double WorldRadius)
{
    SetActorScale3D(FVector(WorldRadius));

    // Terrain geometry is a function of radius, so only rebuild when it actually
    // changed; a re-acquired planet at the same radius (warm revisit) skips the
    // rebuild entirely and just un-parks.
    const double BuiltRadius = LastInitScale.GetMax();
    const bool bNeedsRebuild = !bInitialized ||
        !FMath::IsNearlyEqual(BuiltRadius, WorldRadius, FMath::Max(1.0, BuiltRadius * 1e-4));

    if (bNeedsRebuild)
    {
        // Wake this actor's tick so its next Tick runs Initialize(), which reconfigures
        // and re-activates every child at the new scale (children stay parked until then).
        SetActorHiddenInGame(false);
        SetActorTickEnabled(true);
        bPendingInitialize = true;
    }
    else
    {
        SetDormant(false);   // warm revisit: geometry matches, just un-park
    }
}
