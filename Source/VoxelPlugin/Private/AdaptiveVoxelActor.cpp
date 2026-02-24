#include "AdaptiveVoxelActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

using namespace RealtimeMesh;

// Sets default values
AAdaptiveVoxelActor::AAdaptiveVoxelActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    CameraPosition = FVector(0, 0, 0);
    Material = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
}

void AAdaptiveVoxelActor::BeginDestroy()
{
    IsDestroyed = true;

    // Safely clear all timers when the actor is destroyed to prevent dangling executions
    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearAllTimersForObject(this);
    }

    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Super::BeginDestroy();
}

void AAdaptiveVoxelActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    // Ensure we are not in a preview world (prevents running in editor mode)
    if (GetWorld() && !GetWorld()->IsPreviewWorld() && TickInEditor)
    {
        InitializeChunks();
    }
}

void AAdaptiveVoxelActor::BeginPlay()
{
    Super::BeginPlay();
    InitializeChunks();
}

void AAdaptiveVoxelActor::CleanSceneRoot()
{
    auto destroyComponentArray = GetRootComponent()->GetAttachChildren();
    for (TObjectPtr<USceneComponent> child : destroyComponentArray)
    {
        URealtimeMeshComponent* meshComponent = Cast<URealtimeMeshComponent>(child);
        if (meshComponent)
        {
            meshComponent->DestroyComponent();
        }
    }
}

void AAdaptiveVoxelActor::InitializeChunks()
{
    CleanSceneRoot();
    // Spherenoise - Example SDF applys perline noise to a sphere with domain shifting and noise scaling to help precision
    auto DensityFunction = [this](FVector Position, FVector AnchorCenter) -> double {
        // 1. High-precision local offset relative to the Chunk Anchor
        FVector LocalOffset = Position - AnchorCenter;

        // 2. Consistent Domain Translation
        double NoiseScale = Size * 0.1;

        // We find the 'Base Domain Coordinate' of the Anchor. 
        // Fmod handles the periodicity (repeating every 256 units).
        double Periodicity = 256;
        FVector DomainBase(
            FMath::Fmod(AnchorCenter.X / NoiseScale, Periodicity),
            FMath::Fmod(AnchorCenter.Y / NoiseScale, Periodicity),
            FMath::Fmod(AnchorCenter.Z / NoiseScale, Periodicity)
        );

        // 3. Local Domain Offset
        // This is a small number (e.g., within the chunk's extent)
        FVector DomainLocal = LocalOffset / NoiseScale;

        // Summing two small numbers preserves the precision lost at 10^8
        FVector FinalSample = DomainBase + DomainLocal;
        float NoiseVal = FMath::PerlinNoise3D(FinalSample) * (float)(Size * 0.1);

        // 4. Planet-Relative Distance (Stay in Double)
        FVector PlanetRelativeP = Position - GetActorLocation();
        double RealDist = PlanetRelativeP.Size(); // RelativePos is small enough now

        return RealDist - (Size * 0.85 + (double)NoiseVal);
    };

    //AdaptiveOctree.Reset();
    //EditStore.Reset();
    // Adaptive octree meshes the implicit structure
    EditStore = MakeShared<FSparseEditStore>(GetActorLocation(), Size, ChunkDepth, MaxDepth);
    AdaptiveOctree = MakeShared<FAdaptiveOctree>(DensityFunction, GetActorLocation(), Size, ChunkDepth, MinDepth, MaxDepth);
    // Store for user edits


    AdaptiveOctree->InitializeMeshChunks(this, Material);
    Initialized = true;

    // Use Unreal's TimerManager to safely schedule repeating tasks
    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearTimer(DataUpdateTimerHandle);
        World->GetTimerManager().ClearTimer(MeshUpdateTimerHandle);
        World->GetTimerManager().SetTimer(DataUpdateTimerHandle, this, &AAdaptiveVoxelActor::RunDataUpdateTask, MinDataUpdateInterval, true);
        World->GetTimerManager().SetTimer(MeshUpdateTimerHandle, this, &AAdaptiveVoxelActor::RunMeshUpdateTask, MinMeshUpdateInterval, true);
    }
}

void AAdaptiveVoxelActor::RunDataUpdateTask()
{
    if (DataUpdateIsRunning || IsDestroyed) return;

    DataUpdateIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);

    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis]()
        {
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;

            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                FVector CurrentCamPos = Self->CameraPosition;
                FVector Velocity = (CurrentCamPos - Self->LastLodUpdatePosition);
                FVector PredictedPos = CurrentCamPos + (Velocity * Self->VelocityLookAheadFactor);
                Self->AdaptiveOctree->UpdateLOD(PredictedPos, Self->ScreenSpaceThreshold, Self->CameraFOV);
                Self->LastLodUpdatePosition = Self->CameraPosition;
            }

            Self->DataUpdateIsRunning = false;

        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundThreadNormalTask);
}

void AAdaptiveVoxelActor::RunMeshUpdateTask()
{
    if (MeshUpdateIsRunning || IsDestroyed) return;

    MeshUpdateIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);

    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis]()
        {
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;

            {
                FRWScopeLock ReadLock(Self->OctreeLock, SLT_ReadOnly);
                Self->AdaptiveOctree->UpdateMesh();
            }

            Self->MeshUpdateIsRunning = false;

        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundThreadNormalTask);
}

bool AAdaptiveVoxelActor::ShouldTickIfViewportsOnly() const
{
    return TickInEditor;
}

void AAdaptiveVoxelActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!Initialized) return;

    // Cache cam data
    auto world = GetWorld();
    if (world != nullptr)
    {
        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0)
        {
            this->CameraPosition = viewLocations[0];

            APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(world, 0);
            if (CamManager)
            {
                CameraFOV = CamManager->GetFOVAngle();
            }
        }
    }
}

TSharedPtr<FAdaptiveOctree> AAdaptiveVoxelActor::GetOctree()
{
    return AdaptiveOctree;
}