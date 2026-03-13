#include "AdaptiveVoxelActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

using namespace RealtimeMesh;

AAdaptiveVoxelActor::AAdaptiveVoxelActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    CameraPosition = FVector(0, 0, 0);
    SurfaceMaterial = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
    OceanMaterial = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
}

void AAdaptiveVoxelActor::BeginDestroy()
{
    IsDestroyed = true;

    if (UWorld* World = GetWorld())
        World->GetTimerManager().ClearAllTimersForObject(this);

    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Super::BeginDestroy();
}

void AAdaptiveVoxelActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (GetWorld() && !GetWorld()->IsPreviewWorld() && TickInEditor)
        Initialize();
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
            meshComponent->DestroyComponent();
    }
}

void AAdaptiveVoxelActor::Initialize()
{
    CleanSceneRoot();

    Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");

    auto DensityFunction = [this](int Count, const float* XPos, const float* YPos, const float* ZPos, float* OutNoise) {
        Noise->GenPositionArray3D(OutNoise, Count, XPos, YPos, ZPos, 0, 0, 0, 0);
        };

    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(GetActorLocation(), Size, ChunkDepth, MaxDepth);
    AdaptiveOctree = MakeShared<FAdaptiveOctree>(this, SurfaceMaterial, OceanMaterial, DensityFunction, EditStore, GetActorLocation(), Size, ChunkDepth, MinDepth, MaxDepth);

    Initialized = true;

    if (UWorld* World = GetWorld())
        World->GetTimerManager().ClearTimer(DataUpdateTimerHandle);

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
                FRWScopeLock ReadLock(Self->OctreeLock, SLT_ReadOnly);
                FVector CurrentCamPos = Self->CameraPosition;
                FVector Velocity = CurrentCamPos - Self->LastLodUpdatePosition;
                FVector PredictedPos = CurrentCamPos + Velocity * Self->VelocityLookAheadFactor;
                Self->AdaptiveOctree->UpdateLOD(PredictedPos, Self->ScreenSpaceThreshold, Self->CameraFOV);
                Self->LastLodUpdatePosition = Self->CameraPosition;
            }
            UE_LOG(LogTemp, Log, TEXT("[Pipeline] DataUpdate (UpdateLOD): %.2fms"), (FPlatformTime::Seconds() - t0) * 1000.0);

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
            UE_LOG(LogTemp, Log, TEXT("[Pipeline] MeshUpdate (UpdateMesh): %.2fms"), (FPlatformTime::Seconds() - t0) * 1000.0);

            Self->MeshUpdateIsRunning = false;
            Self->RunDataCleanupTask();

        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundHiPriTask);
}

void AAdaptiveVoxelActor::RunDataCleanupTask()
{
    if (DataCleanupIsRunning || IsDestroyed) return;
    DataCleanupIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);

    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis]()
        {
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;

            double t0 = FPlatformTime::Seconds();
            Self->AdaptiveOctree->CleanupData();
            UE_LOG(LogTemp, Log, TEXT("[Pipeline] DataCleanup (PruneCorners): %.2fms"), (FPlatformTime::Seconds() - t0) * 1000.0);

            Self->DataCleanupIsRunning = false;

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

        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundHiPriTask);
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

    auto world = GetWorld();
    if (world != nullptr)
    {
        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0)
        {
            CameraPosition = viewLocations[0];

            APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(world, 0);
            if (CamManager)
                CameraFOV = CamManager->GetFOVAngle();
        }
    }

    if (world->IsGameWorld())
    {
        double InEditRadius = 300;
        double InEditStrength = 300 * 2;
        int InEditResolution = 3;
        float DebugDrawTime = .1f;

        APlayerController* PC = UGameplayStatics::GetPlayerController(world, 0);
        if (PC && PC->IsInputKeyDown(EKeys::E) && !EditUpdateIsRunning)
        {
            FVector Start = CameraPosition;
            FVector End = Start + PC->GetControlRotation().Vector() * Size;
            FHitResult Hit;
            if (world->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            {
                RunEditUpdateTask(Hit.ImpactPoint, InEditRadius, InEditStrength, InEditResolution);
                DrawDebugSphere(world, Hit.ImpactPoint, InEditRadius, 32, FColor::Red, false, DebugDrawTime);
            }
        }

        if (PC && PC->IsInputKeyDown(EKeys::Q) && !EditUpdateIsRunning)
        {
            FVector Start = CameraPosition;
            FVector End = Start + PC->GetControlRotation().Vector() * Size;
            FHitResult Hit;
            if (world->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            {
                RunEditUpdateTask(Hit.ImpactPoint, InEditRadius, -InEditStrength, InEditResolution);
                DrawDebugSphere(world, Hit.ImpactPoint, InEditRadius, 32, FColor::Green, false, DebugDrawTime);
            }
        }
    }
}