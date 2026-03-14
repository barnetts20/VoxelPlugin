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
    SurfaceMaterial = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
    OceanMaterial = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);

    // Default scale defines planet radius in world units (cm).
    // 80,000,000 cm = 800 km radius planet.
    SetActorScale3D(FVector(80000000.0));

    // Mesh chunks attach to this component instead of the actor root.
    // bAbsoluteScale prevents actor scale from being inherited by mesh geometry.
    MeshAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("MeshAttachmentRoot"));
    MeshAttachmentRoot->SetupAttachment(GetRootComponent());
    MeshAttachmentRoot->SetAbsolute(false, false, true);
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

    // Lock all scale axes to the max component to keep the planet spherical
    FVector CurrentScale = GetActorScale3D();
    double MaxScale = FMath::Max3(FMath::Abs(CurrentScale.X), FMath::Abs(CurrentScale.Y), FMath::Abs(CurrentScale.Z));
    if (MaxScale < 1.0) MaxScale = 1.0;
    SetActorScale3D(FVector(MaxScale));

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
    auto destroyComponentArray = MeshAttachmentRoot->GetAttachChildren();
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

    Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");

    auto DensityFunction = [this](int Count, const float* XPos, const float* YPos, const float* ZPos, float* OutNoise) {
        Noise->GenPositionArray3D(OutNoise, Count, XPos, YPos, ZPos, 0, 0, 0, 0);
        };

    // PlanetRadius is the max component of actor scale (all axes locked to same value).
    // This is the minimum possible surface elevation — noise only adds height above it.
    double ActorPlanetRadius = GetActorScale3D().GetMax();
    double ActorNoiseAmplitude = ActorPlanetRadius * NoiseAmplitudeRatio;
    double ActorRootExtent = (ActorPlanetRadius + ActorNoiseAmplitude) * 1.05;

    // Everything is built in actor-local space (origin 0,0,0).
    // MeshAttachmentRoot handles world placement by following the actor.
    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(FVector::ZeroVector, ActorRootExtent, ChunkDepth, MaxDepth);

    FOctreeParams Params;
    Params.ParentActor = this;
    Params.MeshAttachmentRoot = MeshAttachmentRoot;
    Params.SurfaceMaterial = SurfaceMaterial;
    Params.OceanMaterial = OceanMaterial;
    Params.NoiseFunction = DensityFunction;
    Params.EditStore = EditStore;
    Params.Center = FVector::ZeroVector;
    Params.PlanetRadius = ActorPlanetRadius;
    Params.NoiseAmplitude = ActorNoiseAmplitude;
    Params.SeaLevelCoefficient = SeaLevelCoefficient;
    Params.ChunkDepth = ChunkDepth;
    Params.MinDepth = MinDepth;
    Params.MaxDepth = MaxDepth;

    AdaptiveOctree = MakeShared<FAdaptiveOctree>(Params);

    Initialized = true;

    // Clear both timers only DataUpdate timer is used now as the chain starter
    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearTimer(DataUpdateTimerHandle);
    }

    RunDataUpdateTask();
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

            double t0 = FPlatformTime::Seconds();
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                FVector CurrentCamPos = Self->CameraPosition;
                FVector Velocity = (CurrentCamPos - Self->LastLodUpdatePosition);
                FVector PredictedPos = CurrentCamPos + (Velocity * Self->VelocityLookAheadFactor);
                Self->AdaptiveOctree->UpdateLOD(PredictedPos, Self->ScreenSpaceThreshold, Self->CameraFOV);
                Self->LastLodUpdatePosition = Self->CameraPosition;
            }
            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > 100.0)
                UE_LOG(LogTemp, Log, TEXT("[Pipeline] DataUpdate (UpdateLOD): %.2fms"), elapsed);

            Self->DataUpdateIsRunning = false;
            Self->RunMeshUpdateTask();

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

            double t0 = FPlatformTime::Seconds();
            {
                FRWScopeLock ReadLock(Self->OctreeLock, SLT_ReadOnly);
                Self->AdaptiveOctree->UpdateMesh();
            }
            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > 10.0)
                UE_LOG(LogTemp, Log, TEXT("[Pipeline] MeshUpdate (UpdateMesh): %.2fms"), elapsed);

            Self->MeshUpdateIsRunning = false;

            // Chain back to data update after a minimum interval delay
            AsyncTask(ENamedThreads::GameThread, [WeakThis]()
                {
                    AAdaptiveVoxelActor* Self = WeakThis.Get();
                    if (!Self || Self->IsDestroyed) return;
                    if (UWorld* World = Self->GetWorld())
                    {
                        World->GetTimerManager().SetTimer(
                            Self->DataUpdateTimerHandle,
                            Self,
                            &AAdaptiveVoxelActor::RunDataUpdateTask,
                            Self->MinDataUpdateInterval,
                            false
                        );
                    }
                });

        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

void AAdaptiveVoxelActor::RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    if (EditUpdateIsRunning || IsDestroyed) return;

    EditUpdateIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);
    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, InEditCenter, InEditRadius, InEditStrength, InEditResolution]()
        {
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                Self->AdaptiveOctree->ApplyEdit(InEditCenter, InEditRadius, InEditStrength, InEditResolution);
                Self->AdaptiveOctree->UpdateMesh();
            }
            Self->EditUpdateIsRunning = false;
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
            // Convert world-space camera to actor-local space (octree is built at origin).
            FVector WorldCamPos = viewLocations[0];
            this->CameraPosition = WorldCamPos - GetActorLocation();

            APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(world, 0);
            if (CamManager)
            {
                CameraFOV = CamManager->GetFOVAngle();
            }
        }
    }

    //Example of edit flow, would want to move off tick for actual implementation
    if (world->IsGameWorld())
    {
        // Edit traces need world-space camera, not actor-local
        FVector WorldCamPos = CameraPosition + GetActorLocation();
        double TraceDistance = GetActorScale3D().GetMax() * 3.0;
        double InEditRadius = 300;
        double InEditStrength = 300 * 2;
        int InEditResolution = 3;
        float DebugDrawTime = .1f;
        APlayerController* PC = UGameplayStatics::GetPlayerController(world, 0);
        if (PC && PC->IsInputKeyDown(EKeys::E) && !EditUpdateIsRunning)
        {
            FVector Start = WorldCamPos;
            FVector Forward = PC->GetControlRotation().Vector();
            FVector End = Start + Forward * TraceDistance;

            FHitResult Hit;
            if (world->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            {
                FVector LocalHit = Hit.ImpactPoint - GetActorLocation();
                RunEditUpdateTask(LocalHit, InEditRadius, InEditStrength, InEditResolution);
                DrawDebugSphere(world, Hit.ImpactPoint, InEditRadius, 32, FColor::Red, false, DebugDrawTime);
            }
        }
        if (PC && PC->IsInputKeyDown(EKeys::Q) && !EditUpdateIsRunning)
        {
            FVector Start = WorldCamPos;
            FVector Forward = PC->GetControlRotation().Vector();
            FVector End = Start + Forward * TraceDistance;

            FHitResult Hit;
            if (world->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            {
                FVector LocalHit = Hit.ImpactPoint - GetActorLocation();
                RunEditUpdateTask(LocalHit, InEditRadius, -InEditStrength, 3);
                DrawDebugSphere(world, Hit.ImpactPoint, InEditRadius, 32, FColor::Green, false, DebugDrawTime);
            }
        }
    }
}