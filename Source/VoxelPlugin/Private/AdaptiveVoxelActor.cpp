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

    // Spherenoise - Example SDF applies perlin noise to a sphere
    //auto DensityFunction = [this](FVector Position, FVector AnchorCenter) -> double {
    //    double NoiseScale = Size * 0.1;
    //    float NoiseVal = FMath::PerlinNoise3D(Position/NoiseScale) * (float)(Size * 0.1);
    //    FVector PlanetRelativeP = Position - GetActorLocation();
    //    double RealDist = PlanetRelativeP.Size();

    //    return RealDist - (Size * 0.9 + (double)NoiseVal);
    //};

    //Spherenoise 2 - Example SDF applies perline noise to a sphere as if it was a height map
    //auto DensityFunction = [this](FVector Position, FVector AnchorCenter) -> double {
    //    FVector PlanetRelative = Position - GetActorLocation();
    //    double RealDist = PlanetRelative.Size();
    //    FVector Direction = PlanetRelative.GetSafeNormal();

    //    // Sample noise at the surface direction, not at the 3D position
    //    FVector NoiseSamplePos = Direction * Size;
    //    double NoiseScale = Size * 0.1;
    //    float NoiseVal = FMath::PerlinNoise3D(NoiseSamplePos / NoiseScale) * (float)(Size * 0.1);

    //    double SurfaceRadius = Size * 0.9 + (double)NoiseVal;
    //    return RealDist - SurfaceRadius;
    //};
    Noise = FastNoise::NewFromEncodedNodeTree("HgAZABoAAR8AGwAkABgAAAAXAAAAgL8AAIA/AAAAAAAAgD8RAAQAAAAAAABAEAAAAAA/EwAAAAA/BwABJQAAAIA/AACAP7geBUAAAIA/GwAZAP//AAAAAACAPwDNzExAAf//BQAAAAAAPwAAACBCARcAAAAAAAAAgD8AAMA/AAAAPw0ABAAAAAAAAEALAAEAAAAAAAAAAQAAAAAAAAAAAACAPwAAAAA/AAAAAAAAAACAPwEaAAEgAP//CQAB//8MAAAAAIA/Af//DAABGwD//wYAAAAAAD8AAAAAAA==");
    //Fast Noise example
    auto DensityFunction = [this](FVector Position, FVector AnchorCenter) -> double {
        FVector PlanetRelative = Position - GetActorLocation();
        double RealDist = PlanetRelative.Size();
        FVector Direction = PlanetRelative.GetSafeNormal();

        // Sample noise at the surface direction, not at the 3D position
        FVector NoiseSamplePos = Direction * Size;
        double NoiseScale = Size * 0.1;
        auto samplePos = NoiseSamplePos / NoiseScale;
        float NoiseVal = Noise->GenSingle3D(samplePos.X, samplePos.Y, samplePos.Z, 666) * (float)(Size * 0.1);// FMath::PerlinNoise3D(NoiseSamplePos / NoiseScale)* (float)(Size * 0.1);

        double SurfaceRadius = Size * 0.9 + (double)NoiseVal;
        return RealDist - SurfaceRadius;
    };


    // Store for user edits
    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(GetActorLocation(), Size, ChunkDepth, MaxDepth);
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
            this->CameraPosition = viewLocations[0];

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
        double InEditRadius = 300;
        double InEditStrength = 300 * 2;
        int InEditResolution = 3;
        float DebugDrawTime = .1f;
        APlayerController* PC = UGameplayStatics::GetPlayerController(world, 0);
        if (PC && PC->IsInputKeyDown(EKeys::E) && !EditUpdateIsRunning)
        {
            FVector Start = CameraPosition;
            FVector Forward = PC->GetControlRotation().Vector();
            FVector End = Start + Forward * Size;

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
            FVector Forward = PC->GetControlRotation().Vector();
            FVector End = Start + Forward * Size;

            FHitResult Hit;
            if (world->LineTraceSingleByChannel(Hit, Start, End, ECC_Visibility))
            {
                RunEditUpdateTask(Hit.ImpactPoint, InEditRadius, -InEditStrength, 3);
                DrawDebugSphere(world, Hit.ImpactPoint, InEditRadius, 32, FColor::Green, false, DebugDrawTime);
            }
        }
    }
}
