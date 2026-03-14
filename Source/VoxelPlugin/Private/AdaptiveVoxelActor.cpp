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
    // Inherits actor transform (position, rotation, scale) so normalized-space geometry
    // is placed and scaled to world size by the actor transform.
    MeshAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("MeshAttachmentRoot"));
    MeshAttachmentRoot->SetupAttachment(GetRootComponent());
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

    // Lock all scale axes to the max component to keep the planet spherical.
    // Only override if axes actually differ to avoid fighting the editor drag.
    //FVector CurrentScale = GetActorScale3D();
    //double MaxScale = FMath::Max3(FMath::Abs(CurrentScale.X), FMath::Abs(CurrentScale.Y), FMath::Abs(CurrentScale.Z));
    //if (MaxScale < 1.0) MaxScale = 1.0;
    //if (!CurrentScale.Equals(FVector(MaxScale), 0.01))
    //{
    //    SetActorScale3D(FVector(MaxScale));
    //}

    // Only initialize once. The octree is built in normalized space, so
    // position/rotation/scale changes are all handled by the actor transform
    // without requiring reconstruction.
    if (GetWorld() && !GetWorld()->IsPreviewWorld() && TickInEditor && !Initialized)
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

    // Octree is built in normalized space: PlanetRadius = 1.0.
    // Actor scale transforms the mesh to world size.
    // NoiseAmplitude is relative to the unit radius.
    double NormalizedPlanetRadius = 1.0;
    double NormalizedNoiseAmplitude = NoiseAmplitudeRatio;
    double NormalizedRootExtent = (NormalizedPlanetRadius + NormalizedNoiseAmplitude) * 1.05;

    // Everything is built in actor-local normalized space (origin 0,0,0).
    // MeshAttachmentRoot handles world placement and scaling via the actor transform.
    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(FVector::ZeroVector, NormalizedRootExtent, ChunkDepth, MaxDepth);

    FOctreeParams Params;
    Params.ParentActor = this;
    Params.MeshAttachmentRoot = MeshAttachmentRoot;
    Params.SurfaceMaterial = SurfaceMaterial;
    Params.OceanMaterial = OceanMaterial;
    Params.NoiseFunction = DensityFunction;
    Params.EditStore = EditStore;
    Params.Center = FVector::ZeroVector;
    Params.PlanetRadius = NormalizedPlanetRadius;
    Params.NoiseAmplitude = NormalizedNoiseAmplitude;
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
            // Convert world-space camera to normalized actor-local space.
            // Full inverse transform includes position, rotation, AND scale
            // since the octree is built in normalized space (radius ~1.0).
            FVector WorldCamPos = viewLocations[0];
            this->CameraPosition = GetActorTransform().InverseTransformPosition(WorldCamPos);

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
        // Convert between normalized local space and world space using full actor transform
        FTransform ActorTransform = GetActorTransform();
        FVector WorldCamPos = ActorTransform.TransformPosition(CameraPosition);
        double ActorScale = GetActorScale3D().GetMax();
        double TraceDistance = ActorScale * 3.0;

        // World-space edit parameters, converted to normalized space for the octree
        double WorldEditRadius = 300;
        double WorldEditStrength = 300 * 2;
        double LocalEditRadius = WorldEditRadius / ActorScale;
        double LocalEditStrength = WorldEditStrength / ActorScale;
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
                FVector LocalHit = ActorTransform.InverseTransformPosition(Hit.ImpactPoint);
                RunEditUpdateTask(LocalHit, LocalEditRadius, LocalEditStrength, InEditResolution);
                DrawDebugSphere(world, Hit.ImpactPoint, WorldEditRadius, 32, FColor::Red, false, DebugDrawTime);
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
                FVector LocalHit = ActorTransform.InverseTransformPosition(Hit.ImpactPoint);
                RunEditUpdateTask(LocalHit, LocalEditRadius, -LocalEditStrength, 3);
                DrawDebugSphere(world, Hit.ImpactPoint, WorldEditRadius, 32, FColor::Green, false, DebugDrawTime);
            }
        }
    }
}