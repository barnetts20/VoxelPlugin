#include "PlanetActor.h"
#include "PlanetGravityZone.h"
#include "VoxelLandBake.h"
#include "VoxelPlugin.h"
#include "EngineUtils.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "Kismet/GameplayStatics.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"

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
    const bool bTerrestrial = Generator == EVoxelSurfaceGenerator::Terrestrial;
    if (!bInitialized
        || !GetActorScale3D().Equals(LastInitScale, 0.01)
        || NoiseAmplitudeRatio != LastNoiseAmplitudeRatio
        || Generator != LastGenerator
        || (!bTerrestrial && SurfaceSettings.GetSettingsHash() != LastSurfaceSettingsHash)
        || (bTerrestrial && TerrestrialSettings.GetSettingsHash() != LastTerrestrialHash))
    {
        bPendingInitialize = true;
    }
    else if ((!bTerrestrial && SeaLevel != LastSeaLevel)
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
        // A terrestrial planet is built once its bake is ready; until then it shows what it
        // showed (nothing on the first build).
        if (Generator == EVoxelSurfaceGenerator::Terrestrial && !TickLandBake())
            return;
        bPendingInitialize = false;
        bPendingOceanUpdate = false;
        Initialize();
    }
    else if (bPendingOceanUpdate)
    {
        // A terrestrial planet not built yet has nothing to update: its build applies the toggles.
        if (Generator == EVoxelSurfaceGenerator::Terrestrial && !LandBake.IsValid())
        {
            bPendingOceanUpdate = false;
            return;
        }
        bPendingOceanUpdate = false;
        double PlanetRadius = 0.0;
        double NoiseAmplitude = 0.0;
        double OceanRadius = 0.0;
        ComputeShape(PlanetRadius, NoiseAmplitude, OceanRadius);
        // The same configuration as the first initialization: the ocean keeps its material and
        // settings, and the gravity zone follows the new sea level. A gravity toggle alone
        // leaves the ocean as it is.
        const bool bSeaLevelChanged = Generator != EVoxelSurfaceGenerator::Terrestrial && SeaLevel != LastSeaLevel;
        if (bSeaLevelChanged || bEnableOcean != bLastEnableOcean)
            ConfigureOcean(PlanetRadius, NoiseAmplitude, OceanRadius);
        // The elevation debug view measures from the new sea level.
        if (bSeaLevelChanged && TerrainActor)
            TerrainActor->SetDebugSeaRadius(OceanRadius);
        ConfigureGravity(PlanetRadius, OceanRadius);
        RecordAppliedState();
    }
}

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

bool APlanetActor::TickLandBake()
{
    const double SeaRadius = GetActorScale3D().GetMax();
    const uint32 Key = VoxelLand::BakeKey(TerrestrialSettings, SeaRadius);
    // The peaks' shared noise set: the density takes it when it is built, so it is read or
    // generated in the background first (the game thread never waits for it), and held here
    // from then on (it can't go before the density takes it).
    auto NoiseReady = [this]()
    {
        if (TerrestrialSettings.PeakStrength <= 0.0) return true;
        const FString Name = VoxelLand::PeakNoiseNameFor(TerrestrialSettings);
        if (VoxelLand::TryGetPeakNoise(Name, PeakNoise)) return true;
        VoxelLand::PrewarmPeakNoise(Name);
        return false;
    };
    if (LandBake.IsValid() && LandBake->Key == Key && LandBake->N == VoxelLand::BakeResolutionFor(TerrestrialSettings))
    {
        // Only the per-sample settings changed: the bake is reused.
        LandBaker.Reset();
        return NoiseReady();
    }
    if (!LandBaker.IsValid() || LandBaker->GetKey() != Key)
        LandBaker = MakeShared<FVoxelLandBaker>(TerrestrialSettings, SeaRadius);
    LandBaker->Tick();
    if (!LandBaker->IsDone())
        return false;
    LandBake = LandBaker->GetResult();
    LandBaker.Reset();
    return LandBake.IsValid() && NoiseReady();
}

float APlanetActor::GetBakeProgress() const
{
    if (Generator != EVoxelSurfaceGenerator::Terrestrial) return 1.0f;
    if (LandBaker.IsValid()) return LandBaker->GetProgress();
    return LandBake.IsValid() && LandBake->Key == VoxelLand::BakeKey(TerrestrialSettings, GetActorScale3D().GetMax()) ? 1.0f : 0.0f;
}

void APlanetActor::ComputeShape(double& OutPlanetRadius, double& OutNoiseAmplitude, double& OutOceanRadius) const
{
    const double Radius = GetActorScale3D().GetMax();
    if (Generator == EVoxelSurfaceGenerator::Terrestrial && LandBake.IsValid())
    {
        // Heights about sea level, the actor's radius; the lattice spans the bake's range,
        // within the NoiseAmplitudeRatio ceiling.
        VoxelLand::ShapeFor(TerrestrialSettings, *LandBake, OutPlanetRadius, OutNoiseAmplitude);
        const double Ceiling = Radius * NoiseAmplitudeRatio;
        if (OutNoiseAmplitude > Ceiling)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[Planet] %s: the terrestrial heights span %.0f m, more than NoiseAmplitudeRatio allows (%.0f m); they are clamped."),
                *GetName(), OutNoiseAmplitude * 0.01, Ceiling * 0.01);
            OutNoiseAmplitude = Ceiling;
        }
        OutOceanRadius = FMath::Max(1.0, Radius);
        return;
    }
    OutPlanetRadius = Radius;
    OutNoiseAmplitude = Radius * NoiseAmplitudeRatio;
    OutOceanRadius = ComputeOceanRadius(OutPlanetRadius, OutNoiseAmplitude);
}

double APlanetActor::ChildNoiseAmplitudeRatio(double PlanetRadius, double NoiseAmplitude) const
{
    if (Generator == EVoxelSurfaceGenerator::Terrestrial && LandBake.IsValid())
        return NoiseAmplitude / FMath::Max(PlanetRadius, 1.0);
    return NoiseAmplitudeRatio;
}

void APlanetActor::Initialize()
{
    if (Generator != EVoxelSurfaceGenerator::Terrestrial)
    {
        // The noise surface needs no bake: release one a terrestrial build left.
        LandBake.Reset();
        LandBaker.Reset();
        PeakNoise.Reset();
    }
    double PlanetRadius = 0.0;
    double NoiseAmplitude = 0.0;
    double OceanRadius = 0.0;
    ComputeShape(PlanetRadius, NoiseAmplitude, OceanRadius);
    const double RootExtent = VoxelDensity::RootExtentFor(PlanetRadius, NoiseAmplitude);

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
    TerrainActor->NoiseAmplitudeRatio = ChildNoiseAmplitudeRatio(PlanetRadius, NoiseAmplitude);
    TerrainActor->SurfaceSettings = SurfaceSettings;
    TerrainActor->SetDebugSeaRadius(OceanRadius);
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
    OceanActor->NoiseAmplitudeRatio = ChildNoiseAmplitudeRatio(PlanetRadius, NoiseAmplitude);
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
    LastGenerator = Generator;
    LastTerrestrialHash = TerrestrialSettings.GetSettingsHash();
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

    const double Finest = VoxelDensity::TerrainFinestCell(PlanetRadius, NoiseAmplitude, MaxDepth);
    if (Generator == EVoxelSurfaceGenerator::Terrestrial && LandBake.IsValid())
        return VoxelDensity::MakeTerrestrialCompositor(TerrestrialSettings, LandBake, PlanetRadius, NoiseAmplitude, EditStore, Finest);
    return VoxelDensity::MakeCompositor(SurfaceSettings, PlanetRadius, NoiseAmplitude, EditStore, Finest);
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

// ---------------------------------------------------------------------------
// voxel.Land.ExportBake
// ---------------------------------------------------------------------------

namespace PlanetActorLandExport
{
    void Run(const TArray<FString>& Args, UWorld* World)
    {
        int32 Count = 0;
        for (TActorIterator<APlanetActor> It(World); It; ++It)
        {
            const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake = It->GetLandBake();
            if (!Bake.IsValid()) continue;
            const FString Base = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("VoxelLand"), FString::Printf(TEXT("Export_%s_%08x"), *It->GetName(), Bake->Key));
            IFileManager::Get().MakeDirectory(*FPaths::GetPath(Base), true);
            TArray<uint8> Bytes;
            Bytes.SetNumUninitialized(Bake->Height.Num() * sizeof(float));
            FMemory::Memcpy(Bytes.GetData(), Bake->Height.GetData(), Bytes.Num());
            FFileHelper::SaveArrayToFile(Bytes, *(Base + TEXT("_height.f32")));
            Bytes.SetNumUninitialized(Bake->Attr.Num() * sizeof(uint16));
            FMemory::Memcpy(Bytes.GetData(), Bake->Attr.GetData(), Bytes.Num());
            FFileHelper::SaveArrayToFile(Bytes, *(Base + TEXT("_attr.u16")));
            // The landform (VoxelLandPackForm: hilliness low byte, character high), a u32 per
            // cell as the harnesses read it (Tag_form.u32).
            if (Bake->Form.Num() == Bake->Height.Num())
            {
                TArray<uint32> Form;
                Form.SetNumUninitialized(Bake->Form.Num());
                for (int32 i = 0; i < Form.Num(); i++)
                    Form[i] = Bake->Form[i];
                Bytes.SetNumUninitialized(Form.Num() * sizeof(uint32));
                FMemory::Memcpy(Bytes.GetData(), Form.GetData(), Bytes.Num());
                FFileHelper::SaveArrayToFile(Bytes, *(Base + TEXT("_form.u32")));
            }
            // The drainage (log2 m^2) and the lake levels, as f32, when the bake is eroded.
            if (Bake->Flow.Num() == Bake->Height.Num() && Bake->Water.Num() == Bake->Height.Num())
            {
                TArray<float> Flow;
                Flow.SetNumUninitialized(Bake->Flow.Num());
                for (int32 i = 0; i < Flow.Num(); i++)
                    Flow[i] = Bake->Flow[i].GetFloat();
                Bytes.SetNumUninitialized(Flow.Num() * sizeof(float));
                FMemory::Memcpy(Bytes.GetData(), Flow.GetData(), Bytes.Num());
                FFileHelper::SaveArrayToFile(Bytes, *(Base + TEXT("_flow.f32")));
                Bytes.SetNumUninitialized(Bake->Water.Num() * sizeof(float));
                FMemory::Memcpy(Bytes.GetData(), Bake->Water.GetData(), Bytes.Num());
                FFileHelper::SaveArrayToFile(Bytes, *(Base + TEXT("_water.f32")));
            }
            FString LakesJson;
            for (const FVoxelLandLake& Lake : Bake->Lakes)
            {
                LakesJson += FString::Printf(TEXT("%s{\"cell\": %u, \"level_m\": %.3f, \"outlet\": %u, \"area_km2\": %.3f}"),
                    LakesJson.IsEmpty() ? TEXT("") : TEXT(", "), Lake.Cell, Lake.Level, Lake.Outlet, Lake.AreaKm2);
            }
            const FString Json = FString::Printf(TEXT("{\"n\": %d, \"pad\": %d, \"sea_radius_cm\": %.17g, \"key\": \"%08x\", \"min_m\": %.3f, \"max_m\": %.3f, \"crust_threshold\": %.6f, \"eroded\": %s, \"solve_steps\": %d, \"solve_residual\": %.5f, \"relief_factor\": %.4f, \"lakes\": [%s]}\n"),
                Bake->N, FVoxelLandBake::Pad, Bake->SeaRadius, Bake->Key, Bake->MinHeight, Bake->MaxHeight, Bake->CrustThreshold,
                Bake->bEroded ? TEXT("true") : TEXT("false"), Bake->SolveSteps, Bake->SolveResidual, Bake->ReliefFactor, *LakesJson);
            FFileHelper::SaveStringToFile(Json, *(Base + TEXT(".json")));
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Exported %s's bake to %s_*."), *It->GetName(), *Base);
            Count++;
        }
        if (Count == 0)
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] No terrestrial planet with a bake in this world."));
    }

    FAutoConsoleCommandWithWorldAndArgs Command(
        TEXT("voxel.Land.ExportBake"),
        TEXT("Writes every terrestrial planet's bake (heights f32, attributes u16, the landform u32, drainage and lake levels f32 when eroded, a json header with the lakes) to Saved/VoxelLand/Export_* (Tools/LandView/view_bake.py renders them)."),
        FConsoleCommandWithWorldAndArgsDelegate::CreateStatic(&Run));
}
