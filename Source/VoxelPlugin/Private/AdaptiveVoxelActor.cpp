#include "AdaptiveVoxelActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include <FDensitySampleCompositor.h>

using namespace RealtimeMesh;

// Sets default values
AAdaptiveVoxelActor::AAdaptiveVoxelActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    CameraPosition = FVector(0, 0, 0);
    SurfaceMaterial = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);

    // Default scale defines planet radius in world units (cm).
    // 80,000,000 cm = 800 km radius planet.
    SetActorScale3D(FVector(80000000.0));

    // Mesh chunks attach to this component.
    // Inherits actor position and rotation, but uses absolute scale (1,1,1)
    // since the octree is built at world scale. Scale changes trigger reconstruction.
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

    if (GetWorld() && !GetWorld()->IsPreviewWorld() && TickInEditor)
    {
        // Only reconstruct if not yet initialized or scale changed.
        // Position/rotation are handled live by the actor transform.
        if (!Initialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
        {
            Initialize();
        }
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
    // Stop any in-flight async work from the old octree
    Initialized = false;

    // Wait for any running tasks to complete before tearing down
    while (DataUpdateIsRunning || MeshUpdateIsRunning || EditUpdateIsRunning)
    {
        FPlatformProcess::Sleep(0.001f);
    }

    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearTimer(DataUpdateTimerHandle);
    }

    // Destroy old octree first � this releases all chunk shared pointers
    if (AdaptiveOctree.IsValid())
    {
        AdaptiveOctree->Clear();
        AdaptiveOctree.Reset();
    }

    // Now clean up any components that were already attached
    CleanSceneRoot();

    // Octree is built at world scale in actor-local space (origin 0,0,0).
    // Actor scale determines planet radius. Collision data is baked at this scale.
    // MeshAttachmentRoot handles world placement via position/rotation only (absolute scale).
    double ActorPlanetRadius = GetActorScale3D().GetMax();
    double ActorNoiseAmplitude = ActorPlanetRadius * NoiseAmplitudeRatio;
    double ActorRootExtent = (ActorPlanetRadius + ActorNoiseAmplitude) * 1.05;
    
    //Composes a density sampling layer that treats the input noise node as if it was a heightmap
    //FastNoise::SmartNode<> Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");


    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(FVector::ZeroVector, ActorRootExtent, ChunkDepth, MaxDepth);

    TSharedPtr<FDensitySampleCompositor> Compositor = MakeShared<FDensitySampleCompositor>(EditStore);
    Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");
    auto HeightmapLayer = FCompositorExamples::HeightMapLayer(ActorPlanetRadius, NoiseAmplitudeRatio, Noise);
    Compositor->AddSampleLayer(HeightmapLayer);

    FOctreeParams Params;
    Params.ParentActor = this;
    Params.MeshAttachmentRoot = MeshAttachmentRoot;
    Params.SurfaceMaterial = SurfaceMaterial;
    Params.Compositor = Compositor;
    Params.PlanetRadius = ActorPlanetRadius;
    Params.NoiseAmplitude = ActorNoiseAmplitude;
    Params.ChunkDepth = ChunkDepth;
    Params.MinDepth = MinDepth;
    Params.MaxDepth = MaxDepth;

    AdaptiveOctree = MakeShared<FAdaptiveOctree>(Params);

    LastInitScale = GetActorScale3D();
    Initialized = true;

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
            // Convert world-space camera to actor-local space.
            // Position and rotation only � octree is built at world scale, not normalized.
            FVector WorldCamPos = viewLocations[0];
            FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
            this->CameraPosition = NoScaleTransform.InverseTransformPosition(WorldCamPos);

            APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(world, 0);
            if (CamManager)
            {
                CameraFOV = CamManager->GetFOVAngle();
            }
        }

        //Example of edit flow, would want to move off tick for actual implementation
        if (world->IsGameWorld())
        {
            FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
            FVector WorldCamPos = NoScaleTransform.TransformPosition(CameraPosition);
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
                    FVector LocalHit = NoScaleTransform.InverseTransformPosition(Hit.ImpactPoint);
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
                    FVector LocalHit = NoScaleTransform.InverseTransformPosition(Hit.ImpactPoint);
                    RunEditUpdateTask(LocalHit, InEditRadius, -InEditStrength, 3);
                    DrawDebugSphere(world, Hit.ImpactPoint, InEditRadius, 32, FColor::Green, false, DebugDrawTime);
                }
            }
        }
    }
}