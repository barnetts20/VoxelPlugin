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
}

void APlanetActor::BeginPlay()
{
    Super::BeginPlay();
    // bPendingInitialize defaults to true — first Tick calls Initialize.
}

void APlanetActor::Destroyed()
{
    // Destroyed() fires when the actor is explicitly deleted — both in-editor
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
    // Fallback — catch anything EndPlay missed (shouldn't happen, but safe).
    DestroyChildActors();
    Super::BeginDestroy();
}

void APlanetActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (!GetWorld() || GetWorld()->IsPreviewWorld()) return;

    // Flag for deferred work in Tick — never spawn actors during OnConstruction.
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
    Super::PostEditChangeProperty(PropertyChangedEvent);
    if (!bInitialized) return;

    FName PropName = PropertyChangedEvent.GetPropertyName();

    // Material swaps — push to child actor and reinitialize via the public
    // InitializeFromPlanet entry point (reuses existing compositor).
    if (PropName == GET_MEMBER_NAME_CHECKED(APlanetActor, TerrainMaterial))
    {
        if (TerrainActor && TerrainMaterial)
        {
            TerrainActor->SurfaceMaterial = TerrainMaterial;
            TerrainActor->InitializeFromPlanet(SharedCompositor, PlanetRoot);
        }
        return;
    }

    if (PropName == GET_MEMBER_NAME_CHECKED(APlanetActor, OceanMaterial))
    {
        if (OceanActor && OceanMaterial)
        {
            OceanActor->OceanMaterial = OceanMaterial;
            OceanActor->InitializeFromPlanet(SharedCompositor, PlanetRoot);
        }
        return;
    }

    // Scale, NoiseAmplitudeRatio, SurfaceSettings, SeaLevel, and toggle changes are handled
    // by OnConstruction (which also fires on property edits) via the
    // bPendingInitialize / bPendingOceanUpdate flags.
}
#endif

void APlanetActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    // Deferred init — safe to spawn actors here.
    if (bPendingInitialize)
    {
        bPendingInitialize = false;
        bPendingOceanUpdate = false;
        Initialize();
    }
    else if (bPendingOceanUpdate)
    {
        bPendingOceanUpdate = false;
        double PlanetRadius = GetActorScale3D().GetMax();
        double NoiseAmplitude = PlanetRadius * NoiseAmplitudeRatio;

        if (bEnableOcean && OceanActor)
        {
            double NewOceanRadius = ComputeOceanRadius(PlanetRadius, NoiseAmplitude);
            OceanActor->TerrainPlanetRadius = PlanetRadius;
            OceanActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
            OceanActor->InitializeFromPlanet(SharedCompositor, PlanetRoot, FVector(NewOceanRadius));
            OceanActor->SetActorHiddenInGame(false);
            OceanActor->SetActorTickEnabled(true);
        }
        else if (OceanActor)
        {
            OceanActor->SetActorHiddenInGame(true);
            OceanActor->SetActorTickEnabled(false);
        }

        // Toggle gravity zone visibility
        if (bEnableGravity && GravityZone)
        {
            GravityZone->SetActorHiddenInGame(false);
            GravityZone->SetActorTickEnabled(true);
            GravityZone->SetActorEnableCollision(true);
        }
        else if (GravityZone)
        {
            GravityZone->SetActorHiddenInGame(true);
            GravityZone->SetActorTickEnabled(false);
            GravityZone->SetActorEnableCollision(false);
        }

        LastSeaLevel = SeaLevel;
        bLastEnableOcean = bEnableOcean;
        bLastEnableGravity = bEnableGravity;
    }
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void APlanetActor::Initialize()
{
    double PlanetRadius = GetActorScale3D().GetMax();
    double NoiseAmplitude = PlanetRadius * NoiseAmplitudeRatio;
    double RootExtent = VoxelDensity::RootExtentFor(PlanetRadius, NoiseAmplitude);
    double OceanRadiusValue = ComputeOceanRadius(PlanetRadius, NoiseAmplitude);

    SpawnChildActors();

    // Max depth for the edit store (mirrors AdaptiveVoxelActor::Initialize logic)
    int32 TempMaxDepth;
    {
        double Ratio = 2.0 * RootExtent / TerrainActor->TargetPrecision;
        TempMaxDepth = FMath::Clamp(
            (int32)FMath::CeilToInt(FMath::Log2(Ratio)),
            TerrainActor->MinDepth, AAdaptiveVoxelActor::MaxKeyDepth);
    }

    TSharedPtr<FDensitySampleCompositor> NewCompositor =
        BuildCompositor(PlanetRadius, NoiseAmplitude, RootExtent, TempMaxDepth);
    SharedCompositor = NewCompositor;

    // --- Terrain actor ---
    if (TerrainMaterial)
        TerrainActor->SurfaceMaterial = TerrainMaterial;
    TerrainActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
    TerrainActor->SurfaceSettings = SurfaceSettings;
    TerrainActor->InitializeFromPlanet(SharedCompositor, PlanetRoot, FVector(PlanetRadius));
    TerrainActor->SetActorHiddenInGame(false);
    TerrainActor->SetActorTickEnabled(true);   // re-activate after a pool park

    // --- Ocean actor ---
    if (bEnableOcean && OceanActor)
    {
        if (OceanMaterial)
            OceanActor->OceanMaterial = OceanMaterial;
        OceanActor->TerrainPlanetRadius = PlanetRadius;
        OceanActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
        OceanActor->SurfaceSettings = SurfaceSettings;
        OceanActor->InitializeFromPlanet(SharedCompositor, PlanetRoot, FVector(OceanRadiusValue));
        OceanActor->SetActorHiddenInGame(false);
        OceanActor->SetActorTickEnabled(true);
    }
    else if (OceanActor)
    {
        OceanActor->SetActorHiddenInGame(true);
        OceanActor->SetActorTickEnabled(false);
    }

    // --- Gravity zone ---
    if (bEnableGravity && GravityZone)
    {
        // Inner radius = max of ocean/planet radius, so full gravity is reached at
        // the visible surface.
        double GravityInnerRadius = FMath::Max(OceanRadiusValue, PlanetRadius);
        GravityZone->SetActorScale3D(FVector(GravityInnerRadius));
        GravityZone->InitializeFromPlanet();
        GravityZone->SetActorHiddenInGame(false);
        GravityZone->SetActorTickEnabled(true);
        GravityZone->SetActorEnableCollision(true);
    }
    else if (GravityZone)
    {
        GravityZone->SetActorHiddenInGame(true);
        GravityZone->SetActorTickEnabled(false);
        GravityZone->SetActorEnableCollision(false);
    }

    LastInitScale = GetActorScale3D();
    LastSeaLevel = SeaLevel;
    LastNoiseAmplitudeRatio = NoiseAmplitudeRatio;
    LastSurfaceSettingsHash = SurfaceSettings.GetSettingsHash();
    bLastEnableOcean = bEnableOcean;
    bLastEnableGravity = bEnableGravity;
    bInitialized = true;
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

    // Load default materials if none set — paths reference plugin Content folder
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
    if (TerrainActor)
    {
        TerrainActor->Destroy();
        TerrainActor = nullptr;
    }
    if (OceanActor)
    {
        OceanActor->Destroy();
        OceanActor = nullptr;
    }
    if (GravityZone)
    {
        GravityZone->Destroy();
        GravityZone = nullptr;
    }
}

double APlanetActor::ComputeOceanRadius(double PlanetRadius, double NoiseAmplitude) const
{
    return PlanetRadius + SeaLevel * NoiseAmplitude;
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
        TerrainActor->SetActorTickEnabled(bActive);   // gate the adaptive LOD meshing pump
        TerrainActor->SetActorEnableCollision(bActive); // its collision bricks stop updating while parked
    }
    if (OceanActor)
    {
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
