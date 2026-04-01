#include "PlanetActor.h"
#include "PlanetAtmosphereActor.h"
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

void APlanetActor::BeginDestroy()
{
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
        || NoiseAmplitudeRatio != LastNoiseAmplitudeRatio)
    {
        bPendingInitialize = true;
    }
    else if (SeaLevel != LastSeaLevel
        || bEnableOcean != bLastEnableOcean
        || bEnableAtmosphere != bLastEnableAtmosphere
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

    // Scale, NoiseAmplitudeRatio, SeaLevel, and toggle changes are handled
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

        // Toggle atmosphere visibility
        if (bEnableAtmosphere && AtmosphereActor)
        {
            AtmosphereActor->SetActorHiddenInGame(false);
            AtmosphereActor->SetActorTickEnabled(true);
        }
        else if (AtmosphereActor)
        {
            AtmosphereActor->SetActorHiddenInGame(true);
            AtmosphereActor->SetActorTickEnabled(false);
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
        bLastEnableAtmosphere = bEnableAtmosphere;
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
    double RootExtent = (PlanetRadius + NoiseAmplitude) * 1.05;
    double OceanRadiusValue = ComputeOceanRadius(PlanetRadius, NoiseAmplitude);

    SpawnChildActors();

    // Build the shared noise + compositor
    Noise = FastNoise::NewFromEncodedNodeTree(
        "GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/"
        "ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAA"
        "AAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4B"
        "EwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAA"
        "AAAIA/AAAAAD8AAACAvwAAAAA/");

    // Compute chunk/max depth for the edit store (mirrors AdaptiveVoxelActor::Initialize logic)
    int32 TempChunkDepth, TempMaxDepth;
    {
        double Ratio = 2.0 * RootExtent / TerrainActor->TargetPrecision;
        TempMaxDepth = FMath::Clamp(
            (int32)FMath::CeilToInt(FMath::Log2(Ratio)),
            TerrainActor->MinDepth, AAdaptiveVoxelActor::MaxKeyDepth);

        constexpr double FloatEps = 1.19e-7;
        double ChunkRatio = RootExtent * FloatEps / 1.0;
        TempChunkDepth = FMath::Clamp(
            (ChunkRatio > 1.0) ? (int32)FMath::CeilToInt(FMath::Log2(ChunkRatio)) : 2, 2, 5);
    }

    TSharedPtr<FDensitySampleCompositor> NewCompositor =
        BuildCompositor(PlanetRadius, NoiseAmplitude, RootExtent, TempChunkDepth, TempMaxDepth);
    SharedCompositor = NewCompositor;

    // --- Terrain actor ---
    if (TerrainMaterial)
        TerrainActor->SurfaceMaterial = TerrainMaterial;
    TerrainActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
    TerrainActor->InitializeFromPlanet(SharedCompositor, PlanetRoot, FVector(PlanetRadius));

    // --- Ocean actor ---
    if (bEnableOcean && OceanActor)
    {
        if (OceanMaterial)
            OceanActor->OceanMaterial = OceanMaterial;
        OceanActor->TerrainPlanetRadius = PlanetRadius;
        OceanActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
        OceanActor->InitializeFromPlanet(SharedCompositor, PlanetRoot, FVector(OceanRadiusValue));
        OceanActor->SetActorHiddenInGame(false);
        OceanActor->SetActorTickEnabled(true);
    }
    else if (OceanActor)
    {
        OceanActor->SetActorHiddenInGame(true);
        OceanActor->SetActorTickEnabled(false);
    }

    // --- Atmosphere actor ---
    if (bEnableAtmosphere && AtmosphereActor)
    {
        // Scale = max(OceanRadius, PlanetRadius) — the visible surface floor
        double AtmosphereScale = FMath::Max(OceanRadiusValue, PlanetRadius);
        AtmosphereActor->InitializeFromPlanet(PlanetRoot, FVector(AtmosphereScale));
        AtmosphereActor->SetActorHiddenInGame(false);
        AtmosphereActor->SetActorTickEnabled(true);
    }
    else if (AtmosphereActor)
    {
        AtmosphereActor->SetActorHiddenInGame(true);
        AtmosphereActor->SetActorTickEnabled(false);
    }

    // --- Gravity zone ---
    if (bEnableGravity && GravityZone)
    {
        // Inner radius = atmosphere scale (max of ocean/planet), so full
        // gravity is reached at the visible surface.
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
    bLastEnableOcean = bEnableOcean;
    bLastEnableAtmosphere = bEnableAtmosphere;
    bLastEnableGravity = bEnableGravity;
    bInitialized = true;
}

// ---------------------------------------------------------------------------
// Compositor
// ---------------------------------------------------------------------------

TSharedPtr<FDensitySampleCompositor> APlanetActor::BuildCompositor(
    double PlanetRadius, double NoiseAmplitude, double RootExtent,
    int32 ChunkDepth, int32 MaxDepth)
{
    auto HeightmapLayer = [NoiseNode = Noise, PlanetRadius, NoiseAmplitude]
    (const FSampleInput& Input, float* DensityOut)
        {
            int32 Count = Input.Num();
            double InvNoiseAmplitude = 1.0 / NoiseAmplitude;
            double LayerRootExtent = (PlanetRadius + NoiseAmplitude) * 1.05;

            constexpr int32 StackLimit = 64;
            float  StackPX[StackLimit], StackPY[StackLimit], StackPZ[StackLimit], StackNoise[StackLimit];
            double StackDist[StackLimit];
            TArray<float>  HeapPX, HeapPY, HeapPZ, HeapNoise;
            TArray<double> HeapDist;

            float* PX; float* PY; float* PZ; float* NoiseOut; double* Distances;
            if (Count <= StackLimit)
            {
                PX = StackPX; PY = StackPY; PZ = StackPZ; NoiseOut = StackNoise; Distances = StackDist;
            }
            else
            {
                HeapPX.SetNumUninitialized(Count); HeapPY.SetNumUninitialized(Count);
                HeapPZ.SetNumUninitialized(Count); HeapNoise.SetNumUninitialized(Count);
                HeapDist.SetNumUninitialized(Count);
                PX = HeapPX.GetData(); PY = HeapPY.GetData(); PZ = HeapPZ.GetData();
                NoiseOut = HeapNoise.GetData(); Distances = HeapDist.GetData();
            }

            for (int32 i = 0; i < Count; i++)
            {
                double px = Input.X[i], py = Input.Y[i], pz = Input.Z[i];
                double Dist = FMath::Sqrt(px * px + py * py + pz * pz);
                Distances[i] = Dist;
                double InvDist = (Dist > 1e-10) ? (LayerRootExtent / Dist) : 0.0;
                PX[i] = (float)(px * InvDist * InvNoiseAmplitude);
                PY[i] = (float)(py * InvDist * InvNoiseAmplitude);
                PZ[i] = (float)(pz * InvDist * InvNoiseAmplitude);
            }

            NoiseNode->GenPositionArray3D(NoiseOut, Count, PX, PY, PZ, 0, 0, 0, 0);

            for (int32 i = 0; i < Count; i++)
            {
                double Clamped = FMath::Clamp((double)NoiseOut[i], -1.0, 1.0);
                double Height = (Clamped + 1.0) * 0.5 * NoiseAmplitude;
                DensityOut[i] = (float)(Distances[i] - (PlanetRadius + Height));
            }
        };

    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(
        FVector::ZeroVector, RootExtent, ChunkDepth, MaxDepth);

    TSharedPtr<FDensitySampleCompositor> Comp = MakeShared<FDensitySampleCompositor>(EditStore);
    Comp->AddSampleLayer(HeightmapLayer);
    return Comp;
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

    if (!AtmosphereActor)
    {
        AtmosphereActor = World->SpawnActor<APlanetAtmosphereActor>(
            APlanetAtmosphereActor::StaticClass(),
            FTransform(GetActorRotation(), GetActorLocation()),
            SpawnParams);
        if (USceneComponent* ChildRoot = AtmosphereActor->GetRootComponent())
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
    if (AtmosphereActor)
    {
        AtmosphereActor->Destroy();
        AtmosphereActor = nullptr;
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