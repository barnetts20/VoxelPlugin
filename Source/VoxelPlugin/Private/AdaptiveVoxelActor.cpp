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
        Initialize();
    }
}

void AAdaptiveVoxelActor::BeginPlay()
{
    Super::BeginPlay();
    Initialize();
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

void AAdaptiveVoxelActor::Initialize()
{
    CleanSceneRoot();

    // Spherenoise - Example SDF applies perlin noise to a sphere with domain shifting and noise scaling to help precision
    auto DensityFunction = [this](FVector Position, FVector AnchorCenter) -> double {
        double NoiseScale = Size * 0.1;
        float NoiseVal = FMath::PerlinNoise3D(Position/NoiseScale) * (float)(Size * 0.1);
        FVector PlanetRelativeP = Position - GetActorLocation();
        double RealDist = PlanetRelativeP.Size();

        return RealDist - (Size * 0.9 + (double)NoiseVal);
    };

    // Store for user edits
    EditStore = MakeShared<FSparseEditStore>(GetActorLocation(), Size, ChunkDepth, MaxDepth);
    // Adaptive octree meshes the implicit structure
    AdaptiveOctree = MakeShared<FAdaptiveOctree>(this, Material, DensityFunction, EditStore, GetActorLocation(), Size, ChunkDepth, MinDepth, MaxDepth);

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

        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
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

        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundHiPriTask);
}


void AAdaptiveVoxelActor::RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);
    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, InEditCenter, InEditRadius, InEditStrength, InEditResolution]()
        {
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;

            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                //Updates data and chunk state of tree
                Self->AdaptiveOctree->ApplyEdit(InEditCenter, InEditRadius, InEditStrength, InEditResolution);
                //Dispatches component updates to game thread
                //Self->RunMeshUpdateTask();
            }
        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

bool AAdaptiveVoxelActor::ShouldTickIfViewportsOnly() const
{
    return TickInEditor && Initialized;
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
    if (world->IsGameWorld())
    {
        APlayerController* PC = UGameplayStatics::GetPlayerController(world, 0);
        if (PC && PC->WasInputKeyJustPressed(EKeys::E))
        {
            FVector Start = CameraPosition;
            FVector Forward = PC->GetControlRotation().Vector();
            FVector End = Start + Forward * Size;

            FHitResult Hit;
            if (world->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            {
                RunEditUpdateTask(Hit.ImpactPoint, 500.0, 1000.0, 3); 
                DrawDebugSphere(world, Hit.ImpactPoint, 500.0, 12, FColor::Red, false, 2.0f);
            }
        }
        if (PC && PC->WasInputKeyJustPressed(EKeys::Q))
        {
            FVector Start = CameraPosition;
            FVector Forward = PC->GetControlRotation().Vector();
            FVector End = Start + Forward * Size;

            FHitResult Hit;
            if (world->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            {
                RunEditUpdateTask(Hit.ImpactPoint, 500.0, -1000.0, 3);
                DrawDebugSphere(world, Hit.ImpactPoint, 500.0, 12, FColor::Green, false, 2.0f);
            }
        }
    }
}

TSharedPtr<FAdaptiveOctree> AAdaptiveVoxelActor::GetOctree()
{
    return AdaptiveOctree;
}