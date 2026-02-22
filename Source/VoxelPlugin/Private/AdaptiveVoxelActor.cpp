#include "AdaptiveVoxelActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"

using namespace RealtimeMesh;

// ============================================================================
// CONSTRUCTION (Game Thread)
// ============================================================================

AAdaptiveVoxelActor::AAdaptiveVoxelActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    CameraPosition = FVector(0, 0, 0);
    Material = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);
}

// ============================================================================
// LIFECYCLE (Game Thread)
// ============================================================================

void AAdaptiveVoxelActor::BeginPlay()
{
    Super::BeginPlay();
    InitializeChunks();
}

void AAdaptiveVoxelActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    if (GetWorld() && !GetWorld()->IsPreviewWorld() && TickInEditor)
    {
        InitializeChunks();
    }
}

void AAdaptiveVoxelActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    IsDestroyed = true;

    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearAllTimersForObject(this);
    }

    {
        FRWScopeLock WriteLock(OctreeLock, SLT_Write);
        if (AdaptiveOctree.IsValid())
        {
            AdaptiveOctree->Clear();
            AdaptiveOctree.Reset();
        }
    }

    Super::EndPlay(EndPlayReason);
}

void AAdaptiveVoxelActor::BeginDestroy()
{
    IsDestroyed = true;

    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearAllTimersForObject(this);
    }

    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Super::BeginDestroy();
}

bool AAdaptiveVoxelActor::ShouldTickIfViewportsOnly() const
{
    return TickInEditor;
}

// ============================================================================
// INITIALIZATION (Game Thread)
// ============================================================================

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

    auto DensityFunction = [this](FVector Position, FVector AnchorCenter) -> double {
        FVector LocalOffset = Position - AnchorCenter;

        double NoiseScale = Size * 0.2;
        double Periodicity = 256;
        FVector DomainBase(
            FMath::Fmod(AnchorCenter.X / NoiseScale, Periodicity),
            FMath::Fmod(AnchorCenter.Y / NoiseScale, Periodicity),
            FMath::Fmod(AnchorCenter.Z / NoiseScale, Periodicity)
        );

        FVector DomainLocal = LocalOffset / NoiseScale;
        FVector FinalSample = DomainBase + DomainLocal;
        float NoiseVal = FMath::PerlinNoise3D(FinalSample) * (float)(Size * 0.02);

        FVector PlanetRelativeP = Position - GetActorLocation();
        double RealDist = PlanetRelativeP.Size();

        return RealDist - (Size * 0.85 + (double)NoiseVal);
        };

    AdaptiveOctree.Reset();

    // Construct octree (data only — no UObject creation)
    AdaptiveOctree = MakeShared<FAdaptiveOctree>(DensityFunction, GetActorLocation(), Size, ChunkDepth, MinDepth, MaxDepth);

    // Create mesh chunk entries and UObject components (Game Thread)
    AdaptiveOctree->InitializeMeshChunks(this, Material);
    Initialized = true;

    // Schedule update tasks
    if (UWorld* World = GetWorld())
    {
        World->GetTimerManager().ClearTimer(DataUpdateTimerHandle);
        World->GetTimerManager().ClearTimer(MeshUpdateTimerHandle);
        World->GetTimerManager().SetTimer(DataUpdateTimerHandle, this, &AAdaptiveVoxelActor::RunDataUpdateTask, MinDataUpdateInterval, true);
        World->GetTimerManager().SetTimer(MeshUpdateTimerHandle, this, &AAdaptiveVoxelActor::RunMeshUpdateTask, MinMeshUpdateInterval, true);
    }
}

// ============================================================================
// TICK (Game Thread) — Camera cache + pending chunk creation
// ============================================================================

void AAdaptiveVoxelActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!Initialized) return;

    // Cache camera data
    auto world = GetWorld();
    if (world != nullptr)
    {
        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0)
        {
            CameraPosition = viewLocations[0];

            APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(world, 0);
            if (CamManager)
            {
                CameraFOV = CamManager->GetFOVAngle();
            }
        }
    }

    // Create any pending mesh chunks that need UObject components (Game Thread only)
    if (AdaptiveOctree.IsValid() && AdaptiveOctree->PendingNewChunks.Num() > 0)
    {
        FRWScopeLock WriteLock(OctreeLock, SLT_Write);
        AdaptiveOctree->CreatePendingMeshChunks();
    }

    if (world->IsGameWorld())
    {
        APlayerController* PC = UGameplayStatics::GetPlayerController(world, 0);
        if (PC && PC->IsInputKeyDown(EKeys::E))
        {
            FVector CamLoc;
            FRotator CamRot;
            PC->GetPlayerViewPoint(CamLoc, CamRot);

            FHitResult Hit;
            FVector TraceEnd = CamLoc + CamRot.Vector() * (Size * 3.0);

            FCollisionQueryParams Params;
            Params.bTraceComplex = true;

            if (world->LineTraceSingleByChannel(Hit, CamLoc, TraceEnd, ECC_Visibility, Params))
            {
                if (!PendingEditLocation.IsSet())
                {
                    PendingEditLocation = Hit.ImpactPoint;
                    PendingBrushRadius = 400;
                    PendingStrength = 800;
                }
            }
        }
    }
}

// ============================================================================
// DATA UPDATE TASK (Background Thread) — LOD + Edits
// ============================================================================

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

                if (!Self->AdaptiveOctree.IsValid()) return;

                // Process pending edits
                if (Self->PendingEditLocation.IsSet())
                {
                    Self->AdaptiveOctree->ApplyEdit(
                        Self->PendingEditLocation.GetValue(),
                        Self->PendingBrushRadius,
                        Self->PendingStrength);
                    Self->PendingEditLocation.Reset();
                }

                // LOD update
                FVector CurrentCamPos = Self->CameraPosition;
                FVector Velocity = (CurrentCamPos - Self->LastLodUpdatePosition);
                FVector PredictedPos = CurrentCamPos + (Velocity * Self->VelocityLookAheadFactor);
                Self->AdaptiveOctree->UpdateLOD(PredictedPos, Self->ScreenSpaceThreshold, Self->CameraFOV);
                Self->LastLodUpdatePosition = Self->CameraPosition;
            }

            Self->DataUpdateIsRunning = false;
        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundHiPriTask);
}

// ============================================================================
// MESH UPDATE TASK (Game Thread dispatch via AsyncTask internally)
// ============================================================================

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

                if (!Self->AdaptiveOctree.IsValid()) return;

                // UpdateMesh iterates chunks and calls UpdateComponent,
                // which dispatches to game thread via AsyncTask internally
                Self->AdaptiveOctree->UpdateMesh();
            }

            Self->MeshUpdateIsRunning = false;
        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundHiPriTask);
}

// ============================================================================
// ACCESSORS
// ============================================================================

TSharedPtr<FAdaptiveOctree> AAdaptiveVoxelActor::GetOctree()
{
    return AdaptiveOctree;
}