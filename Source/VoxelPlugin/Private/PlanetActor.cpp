#include "PlanetActor.h"
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
    else if (SeaLevel != LastSeaLevel || bEnableOcean != bLastEnableOcean)
    {
        bPendingOceanUpdate = true;
    }
}

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
            // Set the ocean actor's own scale = its actual radius.
            // Its Initialize reads scale to derive OceanRadius.
            OceanActor->SetActorScale3D(FVector(NewOceanRadius));
            OceanActor->TerrainPlanetRadius = PlanetRadius;
            OceanActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
            OceanActor->InitializeFromPlanet(OceanActor->GetCompositor());
            OceanActor->SetActorHiddenInGame(false);
            OceanActor->SetActorTickEnabled(true);
        }
        else if (OceanActor)
        {
            OceanActor->SetActorHiddenInGame(true);
            OceanActor->SetActorTickEnabled(false);
        }

        LastSeaLevel = SeaLevel;
        bLastEnableOcean = bEnableOcean;
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

    TSharedPtr<FDensitySampleCompositor> SharedCompositor =
        BuildCompositor(PlanetRadius, NoiseAmplitude, RootExtent, TempChunkDepth, TempMaxDepth);

    // --- Terrain actor ---
    // Set its scale = actual planet radius. Its Initialize reads GetActorScale3D().
    TerrainActor->SetActorScale3D(FVector(PlanetRadius));
    TerrainActor->SurfaceMaterial = TerrainMaterial;
    TerrainActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
    TerrainActor->InitializeFromPlanet(SharedCompositor);

    // --- Ocean actor ---
    if (bEnableOcean && OceanActor)
    {
        OceanActor->SetActorScale3D(FVector(OceanRadiusValue));
        OceanActor->OceanMaterial = OceanMaterial;
        OceanActor->TerrainPlanetRadius = PlanetRadius;
        OceanActor->NoiseAmplitudeRatio = NoiseAmplitudeRatio;
        OceanActor->InitializeFromPlanet(SharedCompositor);
        OceanActor->SetActorHiddenInGame(false);
        OceanActor->SetActorTickEnabled(true);
    }
    else if (OceanActor)
    {
        OceanActor->SetActorHiddenInGame(true);
        OceanActor->SetActorTickEnabled(false);
    }

    LastInitScale = GetActorScale3D();
    LastSeaLevel = SeaLevel;
    LastNoiseAmplitudeRatio = NoiseAmplitudeRatio;
    bLastEnableOcean = bEnableOcean;
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
        // Position/rotation inherit from planet. Scale is absolute so
        // SetActorScale3D sets the actual world-space radius, not relative to parent.
        if (USceneComponent* Root = TerrainActor->GetRootComponent())
            Root->SetAbsolute(false, false, true);
        TerrainActor->AttachToActor(this, FAttachmentTransformRules::KeepWorldTransform);
    }

    if (!OceanActor)
    {
        OceanActor = World->SpawnActor<AOceanSphereActor>(
            AOceanSphereActor::StaticClass(),
            FTransform(GetActorRotation(), GetActorLocation()),
            SpawnParams);
        if (USceneComponent* Root = OceanActor->GetRootComponent())
            Root->SetAbsolute(false, false, true);
        OceanActor->AttachToActor(this, FAttachmentTransformRules::KeepWorldTransform);
    }

    // Load default materials if none set
    if (!TerrainMaterial)
    {
        TerrainMaterial = Cast<UMaterialInterface>(
            StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
                TEXT("/Game/Materials/MT_TriPlanar.MT_TriPlanar")));
    }
    if (!OceanMaterial)
    {
        OceanMaterial = Cast<UMaterialInterface>(
            StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
                TEXT("/Game/Materials/MT_Ocean.MT_Ocean")));
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
}

double APlanetActor::ComputeOceanRadius(double PlanetRadius, double NoiseAmplitude) const
{
    return PlanetRadius + SeaLevel * NoiseAmplitude;
}