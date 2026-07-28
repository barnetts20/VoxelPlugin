#include "AdaptiveVoxelActor.h"
#include "PlanetActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include "FDensitySampleCompositor.h"

//Pipeline log thresholds... log if the pipelines take longer than the set value in MS
//0 will always log pipeline execution times.
const float DATA_LOG_THRESHOLD = 100;
const float MESH_LOG_THRESHOLD = 50;
const float EDIT_LOG_THRESHOLD = 50;

// Sets default values
AAdaptiveVoxelActor::AAdaptiveVoxelActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    CameraPosition = FVector(0, 0, 0);
    SurfaceMaterial = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);

    // Default scale defines planet radius in world units (cm).
    // 100,000,000 cm = 1000 km radius planet (matches APlanetActor default).
    SetActorScale3D(FVector(100000000.0));

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

    // Drop any queued applies so their chunk refs release and nothing is applied to a
    // dying actor. IsDestroyed (set above) also gates DrainPendingApply.
    {
        FScopeLock Lock(&PendingApplyCS);
        PendingApply.Empty();
        PendingDestroy.Empty();
        PendingCollision.Empty();
    }

    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Super::BeginDestroy();
}

void AAdaptiveVoxelActor::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    // When owned by a PlanetActor, skip self-init — planet calls InitializeFromPlanet.
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;

    if (GetWorld() && !GetWorld()->IsPreviewWorld() && bTickInEditor)
    {
        if (!Initialized || !GetActorScale3D().Equals(LastInitScale, 0.01))
        {
            Initialize();
        }
    }
}

#if WITH_EDITOR
void AAdaptiveVoxelActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    if (!Initialized) return;

    FName PropName = PropertyChangedEvent.GetPropertyName();

    // Level 0 — LOD params read live each tick, no rebuild needed:
    // VelocityLookAheadFactor, MinDataUpdateInterval

    // Structural changes require full octree rebuild:
    static const TSet<FName> RebuildProps = {
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, NoiseAmplitudeRatio),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, MinDepth),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, TargetPrecision),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, PrecisionDepthFloor),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, SurfaceMaterial),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, ScreenSpaceThreshold),
    };

    if (RebuildProps.Contains(PropName))
    {
        Initialize();
    }
}

bool AAdaptiveVoxelActor::CanEditChange(const FProperty* InProperty) const
{
    if (!Super::CanEditChange(InProperty))
        return false;

    if (bIsPlanetOwned && InProperty)
    {
        const FName PropName = InProperty->GetFName();
        // Lock all transform components — position, rotation, scale are driven by the planet.
        static const FName TransformNames[] = {
            TEXT("RelativeLocation"), TEXT("RelativeRotation"), TEXT("RelativeScale3D"),
        };
        for (const FName& Name : TransformNames)
        {
            if (PropName == Name) return false;
        }
    }

    return true;
}

void AAdaptiveVoxelActor::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
}

void AAdaptiveVoxelActor::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
}

void AAdaptiveVoxelActor::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
    if (bIsPlanetOwned) return;
    Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
}

void AAdaptiveVoxelActor::PostEditMove(bool bFinished)
{
    if (bIsPlanetOwned)
    {
        // Snap all transforms back — location and rotation follow parent, scale is planet-driven.
        if (USceneComponent* Root = GetRootComponent())
        {
            Root->SetRelativeLocation(FVector::ZeroVector);
            Root->SetRelativeRotation(FRotator::ZeroRotator);
        }
    }
    Super::PostEditMove(bFinished);
}
#endif

void AAdaptiveVoxelActor::BeginPlay()
{
    Super::BeginPlay();
    if (GetOwner() && GetOwner()->IsA<APlanetActor>()) return;
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

    // Drop queued applies referencing the old octree's chunks. The busy-wait above has
    // already fenced any in-flight mesh task's enqueue (it enqueues before clearing
    // MeshUpdateIsRunning), so nothing repopulates this after the clear.
    {
        FScopeLock Lock(&PendingApplyCS);
        PendingApply.Empty();
        PendingDestroy.Empty();
        PendingCollision.Empty();
    }

    // Now clean up any components that were already attached
    CleanSceneRoot();

    // Load default plugin material if none assigned by the user.
    // The constructor sets SurfaceMaterial to the engine default as a safe fallback;
    // here we upgrade to the plugin's triplanar material for standalone use.
    if (!SurfaceMaterial || SurfaceMaterial == UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface))
    {
        UMaterialInterface* PluginMaterial = Cast<UMaterialInterface>(
            StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
                TEXT("/VoxelPlugin/MT_TriPlanar.MT_Surface")));
        if (PluginMaterial)
            SurfaceMaterial = PluginMaterial;
    }

    // Octree is built at world scale in actor-local space (origin 0,0,0).
    // Actor scale determines planet radius. Collision data is baked at this scale.
    // MeshAttachmentRoot handles world placement via position/rotation only (absolute scale).
    double ActorPlanetRadius = GetActorScale3D().GetMax();
    double ActorNoiseAmplitude = ActorPlanetRadius * NoiseAmplitudeRatio;
    double ActorRootExtent = (ActorPlanetRadius + ActorNoiseAmplitude) * 1.05;

    // Compute MaxDepth from TargetPrecision.
    // At depth D, voxel extent = RootExtent / 2^D, vertex spacing ≈ 2 * extent.
    // Solve for D: D = ceil(log2(2 * RootExtent / TargetPrecision))
    {
        double Ratio = 2.0 * ActorRootExtent / TargetPrecision;
        int32 IdealDepth = (int32)FMath::CeilToInt(FMath::Log2(Ratio));
        MaxDepth = FMath::Clamp(IdealDepth, MinDepth, MaxKeyDepth);
        ActualPrecision = 2.0 * ActorRootExtent / FMath::Pow(2.0, (double)MaxDepth);
    }

    // Build the chunk cut at the shallow floor (MinChunkDepth). Float-precision jitter at
    // this coarse depth is expected and is recovered near the camera by the chunk-cut pass
    // (FAdaptiveOctree::UpdateChunkCut), which promotes chunks toward MaxChunkDepth
    // wherever the surface is close enough for the jitter to be visible.
    {
        ChunkDepth = FMath::Clamp(MinChunkDepth, 2, MaxChunkDepth);
        MinDepth = FMath::Max(MinDepth, ChunkDepth);
    }

    //Composes a density sampling layer that treats the input noise node as if it was a heightmap
    Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");
    auto HeightmapLayer = [NoiseNode = Noise, PlanetRadius = ActorPlanetRadius, NoiseAmplitude = ActorNoiseAmplitude](const FSampleInput& Input, float* DensityOut) {
        int32 Count = Input.Num();

        double InvNoiseAmplitude = 1.0 / NoiseAmplitude;
        double RootExtent = (PlanetRadius + NoiseAmplitude) * 1.05;

        // Stack buffers for the common small-count paths (8, 19 samples).
        // Only heap-allocate for large bulk calls (ReconstructSubtree).
        constexpr int32 StackLimit = 64;

        float  StackPX[StackLimit], StackPY[StackLimit], StackPZ[StackLimit], StackNoise[StackLimit];
        double StackDist[StackLimit];

        TArray<float>  HeapPX, HeapPY, HeapPZ, HeapNoise;
        TArray<double> HeapDist;

        float* PX; float* PY; float* PZ; float* NoiseOut;
        double* Distances;

        if (Count <= StackLimit)
        {
            PX = StackPX; PY = StackPY; PZ = StackPZ; NoiseOut = StackNoise;
            Distances = StackDist;
        }
        else
        {
            HeapPX.SetNumUninitialized(Count);
            HeapPY.SetNumUninitialized(Count);
            HeapPZ.SetNumUninitialized(Count);
            HeapNoise.SetNumUninitialized(Count);
            HeapDist.SetNumUninitialized(Count);
            PX = HeapPX.GetData(); PY = HeapPY.GetData(); PZ = HeapPZ.GetData();
            NoiseOut = HeapNoise.GetData(); Distances = HeapDist.GetData();
        }

        // Project positions onto sphere surface for noise sampling.
        // Cache Dist to avoid recomputing sqrt in the density loop.
        for (int32 i = 0; i < Count; i++)
        {
            double px = Input.X[i], py = Input.Y[i], pz = Input.Z[i];
            double Dist = FMath::Sqrt(px * px + py * py + pz * pz);
            Distances[i] = Dist;
            double InvDist = (Dist > 1e-10) ? (RootExtent / Dist) : 0.0;
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

    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(FVector::ZeroVector, ActorRootExtent, ChunkDepth, MaxDepth);

    TSharedPtr<FDensitySampleCompositor> Compositor = MakeShared<FDensitySampleCompositor>(EditStore);
    Compositor->AddSampleLayer(HeightmapLayer);

    // Store params for deferred construction on the background thread.
    PendingParams = MakeShared<FOctreeParams>();
    PendingParams->ParentActor = this;
    PendingParams->MeshAttachmentRoot = MeshAttachmentRoot;
    PendingParams->SurfaceMaterial = SurfaceMaterial;
    PendingParams->Compositor = Compositor;
    PendingParams->PlanetRadius = ActorPlanetRadius;
    PendingParams->NoiseAmplitude = ActorNoiseAmplitude;
    PendingParams->ChunkDepth = ChunkDepth;
    PendingParams->MaxChunkDepth = MaxChunkDepth;
    PendingParams->ChunkPrecisionThreshold = ChunkPrecisionThreshold;
    PendingParams->ChunkDemoteHysteresis = ChunkDemoteHysteresis;
    PendingParams->MaxChunkDepthDelta = MaxChunkDepthDelta;
    PendingParams->MinDepth = MinDepth;
    PendingParams->MaxDepth = MaxDepth;
    PendingParams->PrecisionDepthFloor = PrecisionDepthFloor;

    LastInitScale = GetActorScale3D();
    Initialized = true;

    RunDataUpdateTask();
}

void AAdaptiveVoxelActor::OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport)
{
    if (!bIsPlanetOwned) return;

    // Snap everything back: location and rotation follow parent, scale is planet-driven.
    if (USceneComponent* Root = GetRootComponent())
    {
        if (!Root->GetRelativeLocation().IsNearlyZero(0.01)
            || !Root->GetRelativeRotation().IsNearlyZero(0.01)
            || !Root->GetComponentScale().Equals(PlanetDrivenScale, 0.01))
        {
            Root->SetRelativeLocation_Direct(FVector::ZeroVector);
            Root->SetRelativeRotation_Direct(FRotator::ZeroRotator);
            Root->SetRelativeScale3D_Direct(PlanetDrivenScale);
            Root->UpdateComponentToWorld();
        }
    }
}

void AAdaptiveVoxelActor::InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
    USceneComponent* InAttachParent, FVector InScale)
{
    // Same teardown as Initialize
    Initialized = false;
    bIsPlanetOwned = true;

    // Apply scale BEFORE binding the transform guard — prevents the old guard
    // from reverting a scale change that the planet actor intends.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.RemoveAll(this);

    if (!InScale.IsZero())
        SetActorScale3D(InScale);

    PlanetDrivenScale = GetActorScale3D();

    // Re-bind the guard now that PlanetDrivenScale reflects the new scale.
    if (USceneComponent* Root = GetRootComponent())
        Root->TransformUpdated.AddUObject(this, &AAdaptiveVoxelActor::OnTransformUpdated);
    while (DataUpdateIsRunning || MeshUpdateIsRunning || EditUpdateIsRunning)
        FPlatformProcess::Sleep(0.001f);

    if (UWorld* World = GetWorld())
        World->GetTimerManager().ClearTimer(DataUpdateTimerHandle);

    if (AdaptiveOctree.IsValid())
    {
        AdaptiveOctree->Clear();
        AdaptiveOctree.Reset();
    }

    // Drop queued applies referencing the old octree's chunks (see Initialize).
    {
        FScopeLock Lock(&PendingApplyCS);
        PendingApply.Empty();
        PendingDestroy.Empty();
        PendingCollision.Empty();
    }

    CleanSceneRoot();

    // Re-parent MeshAttachmentRoot to the planet's component hierarchy.
    // This ensures mesh chunks follow the planet's position/rotation directly.
    if (InAttachParent && MeshAttachmentRoot)
    {
        MeshAttachmentRoot->AttachToComponent(InAttachParent,
            FAttachmentTransformRules::KeepWorldTransform);
    }

    // Scale = planet radius, same as standalone Initialize
    double ActorPlanetRadius = GetActorScale3D().GetMax();
    double ActorNoiseAmplitude = ActorPlanetRadius * NoiseAmplitudeRatio;
    double ActorRootExtent = (ActorPlanetRadius + ActorNoiseAmplitude) * 1.05;

    // Auto-derive MaxDepth
    {
        double Ratio = 2.0 * ActorRootExtent / TargetPrecision;
        int32 IdealDepth = (int32)FMath::CeilToInt(FMath::Log2(Ratio));
        MaxDepth = FMath::Clamp(IdealDepth, MinDepth, MaxKeyDepth);
        ActualPrecision = 2.0 * ActorRootExtent / FMath::Pow(2.0, (double)MaxDepth);
    }

    // Build the chunk cut at the shallow floor (MinChunkDepth); precision is recovered
    // near the camera by UpdateChunkCut.
    {
        ChunkDepth = FMath::Clamp(MinChunkDepth, 2, MaxChunkDepth);
        MinDepth = FMath::Max(MinDepth, ChunkDepth);
    }

    // Use the planet-provided compositor (shared noise + edit store)
    PendingParams = MakeShared<FOctreeParams>();
    PendingParams->ParentActor = this;
    PendingParams->MeshAttachmentRoot = MeshAttachmentRoot;
    PendingParams->SurfaceMaterial = SurfaceMaterial;
    PendingParams->Compositor = InCompositor;
    PendingParams->PlanetRadius = ActorPlanetRadius;
    PendingParams->NoiseAmplitude = ActorNoiseAmplitude;
    PendingParams->ChunkDepth = ChunkDepth;
    PendingParams->MaxChunkDepth = MaxChunkDepth;
    PendingParams->ChunkPrecisionThreshold = ChunkPrecisionThreshold;
    PendingParams->ChunkDemoteHysteresis = ChunkDemoteHysteresis;
    PendingParams->MaxChunkDepthDelta = MaxChunkDepthDelta;
    PendingParams->MinDepth = MinDepth;
    PendingParams->MaxDepth = MaxDepth;
    PendingParams->PrecisionDepthFloor = PrecisionDepthFloor;

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

            // Deferred construction — build octree on the background thread
            if (Self->PendingParams.IsValid())
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                Self->AdaptiveOctree = MakeShared<FAdaptiveOctree>(*Self->PendingParams);
                Self->PendingParams.Reset();
            }

            if (!Self->AdaptiveOctree.IsValid())
            {
                Self->DataUpdateIsRunning = false;
                return;
            }

            double t0 = FPlatformTime::Seconds();
            TArray<TSharedPtr<FMeshChunk>> RemovedChunks;
            TArray<TSharedPtr<FMeshChunk>> CollisionChanges;
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                FVector CurrentCamPos = Self->CameraPosition;
                FVector Velocity = (CurrentCamPos - Self->LastLodUpdatePosition);
                FVector PredictedPos = CurrentCamPos + (Velocity * Self->VelocityLookAheadFactor);

                Self->AdaptiveOctree->UpdateLOD(PredictedPos, Self->ScreenSpaceThreshold, Self->CameraFOV);
                Self->LastLodUpdatePosition = Self->CameraPosition;

                // Chunk-cut pass: promote chunks to finer origins where float jitter would
                // show, and demote them back as the camera leaves. Uses the ACTUAL camera
                // position (precision is about where you are, not where you're heading). New
                // chunks are dirty and flow through the mesh pass; retired chunks (coarse
                // parents on promote, fine children on demote) come back in RemovedChunks for
                // GT destruction.
                Self->AdaptiveOctree->UpdateChunkCut(CurrentCamPos, Self->CameraFOV, RemovedChunks);

                // Collision pass: decide which chunks should carry a collision body based on
                // near distance + the speed-scaled lead, so the cooked set follows the player.
                const double OnDist = Self->bEnableSurfaceCollision
                    ? (Self->WalkBaseCollisionDistance + Self->PlayerSpeed * Self->CollisionCookLeadTime)
                    : 0.0;
                const double OffDist = OnDist * Self->CollisionHysteresis;
                Self->AdaptiveOctree->UpdateChunkCollision(CurrentCamPos, OnDist, OffDist, CollisionChanges);
            }
            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > DATA_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] DataUpdate: %.2fms"), elapsed);

            if (RemovedChunks.Num() > 0 || CollisionChanges.Num() > 0)
            {
                FScopeLock Lock(&Self->PendingApplyCS);
                if (RemovedChunks.Num() > 0)     Self->PendingDestroy.Append(MoveTemp(RemovedChunks));
                if (CollisionChanges.Num() > 0)  Self->PendingCollision.Append(MoveTemp(CollisionChanges));
            }

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

            // Collect the dirty chunks under the read lock -- this is cheap (it just
            // gathers shared refs). The actual RealtimeMesh apply is deferred to the game
            // thread and throttled, so we neither hold the octree lock across component
            // work nor fire one game-thread task per chunk (the old spawn-time burst).
            double t0 = FPlatformTime::Seconds();
            TArray<TSharedPtr<FMeshChunk>> DirtyChunks;
            {
                FRWScopeLock ReadLock(Self->OctreeLock, SLT_ReadOnly);
                if (Self->AdaptiveOctree.IsValid())
                    Self->AdaptiveOctree->CollectDirtyChunks(DirtyChunks);
            }

            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > MESH_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] MeshUpdate: %.2fms"), elapsed);

            // Enqueue for the game-thread drain BEFORE clearing the running flag, so the
            // teardown busy-wait in Initialize (which waits on MeshUpdateIsRunning) fences
            // the enqueue -- a re-init can't clear PendingApply and then have this task add
            // stale chunks after it.
            Self->EnqueueDirtyChunksForApply(DirtyChunks);

            Self->MeshUpdateIsRunning = false;

            // Chain the next data update on the normal interval. The apply is decoupled --
            // Tick drains PendingApply at frame rate under a time budget -- so it's no
            // longer gated to this loop's slower cadence (the old spawn-population bottleneck).
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
                            false);
                    }
                });

        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

void AAdaptiveVoxelActor::EnqueueDirtyChunksForApply(const TArray<TSharedPtr<FMeshChunk>>& DirtyChunks)
{
    FScopeLock Lock(&PendingApplyCS);
    for (const TSharedPtr<FMeshChunk>& Chunk : DirtyChunks)
    {
        if (Chunk.IsValid() && !Chunk->bApplyQueued)
        {
            Chunk->bApplyQueued = true;
            PendingApply.Add(Chunk);
        }
    }
}

void AAdaptiveVoxelActor::DrainPendingApply()
{
    if (!Initialized || IsDestroyed) return;

    const double BudgetSeconds = FMath::Max(0.1, ChunkApplyBudgetMs) / 1000.0;
    const double StartTime = FPlatformTime::Seconds();

    for (;;)
    {
        TSharedPtr<FMeshChunk> Chunk;
        {
            FScopeLock Lock(&PendingApplyCS);
            if (PendingApply.Num() == 0) break;
            Chunk = PendingApply.Pop(EAllowShrinking::No);
            if (Chunk.IsValid()) Chunk->bApplyQueued = false;
        }

        // Apply WITHOUT the octree lock -- matching the original per-chunk apply.
        // RegisterComponent / UpdateSectionGroup must run on the game thread but don't
        // touch the octree; taking the octree read lock here would block the whole drain on
        // the data pass's write lock (a multi-ms UpdateLOD, and the full initial build),
        // reintroducing the game-thread stall we're removing. ApplyToComponent reads the
        // chunk's stream, which a concurrent data pass could be rewriting -- a narrow,
        // pre-existing race (a chunk's stream is only rewritten when its LOD changes, and
        // RealtimeMesh copies the stream on update). If it ever bites, the fix is to hand
        // the drain an immutable stream snapshot at mesh time rather than to lock here.
        if (Chunk.IsValid() && Chunk->IsDirty)
            Chunk->ApplyToComponent();

        // Budget checked after applying at least one chunk, so the drain always makes
        // progress even with a tiny budget.
        if (FPlatformTime::Seconds() - StartTime > BudgetSeconds) break;
    }
}

// A chunk is "settled" once whatever it will show is showing: it has applied (IsDirty
// cleared) or is empty. A chunk retired before it ever applied is settled once its OWN
// replacements are settled -- this follows a fast promote/demote cascade to the bottom so
// we never release a coarse chunk while a hole would remain.
static bool IsChunkSettled(const FMeshChunk* Chunk)
{
    if (!Chunk) return true;
    if (!Chunk->IsDirty) return true;            // applied at least once, or empty
    if (Chunk->bRetired)                         // superseded before applying -- follow the chain
    {
        for (const TSharedPtr<FMeshChunk>& Rep : Chunk->ReplacedBy)
            if (!IsChunkSettled(Rep.Get())) return false;
        return true;
    }
    return false;                                // still waiting on its first apply
}

void AAdaptiveVoxelActor::DrainPendingDestroy()
{
    if (IsDestroyed) return;

    TArray<TSharedPtr<FMeshChunk>> ToProcess;
    {
        FScopeLock Lock(&PendingApplyCS);
        if (PendingDestroy.Num() == 0) return;
        ToProcess = MoveTemp(PendingDestroy);
    }

    // Release a retired chunk only once every chunk that replaces it is live. Until then it
    // keeps rendering its last mesh, so the promote/demote swap never opens a hole. Chunks
    // that aren't ready yet are re-queued for a later frame.
    TArray<TSharedPtr<FMeshChunk>> Deferred;
    for (const TSharedPtr<FMeshChunk>& Chunk : ToProcess)
    {
        if (!Chunk.IsValid()) continue;

        bool bReplacementsLive = true;
        for (const TSharedPtr<FMeshChunk>& Rep : Chunk->ReplacedBy)
        {
            if (!IsChunkSettled(Rep.Get())) { bReplacementsLive = false; break; }
        }

        if (bReplacementsLive)
        {
            Chunk->ReplacedBy.Reset();
            Chunk->ReleaseComponent();
        }
        else
        {
            Deferred.Add(Chunk);
        }
    }

    if (Deferred.Num() > 0)
    {
        FScopeLock Lock(&PendingApplyCS);
        PendingDestroy.Append(MoveTemp(Deferred));
    }
}

void AAdaptiveVoxelActor::DrainPendingCollision()
{
    if (IsDestroyed) return;

    TArray<TSharedPtr<FMeshChunk>> ToProcess;
    {
        FScopeLock Lock(&PendingApplyCS);
        if (PendingCollision.Num() == 0) return;
        ToProcess = MoveTemp(PendingCollision);
    }

    // Few chunks cross the collision threshold per pass, so this is drained fully rather
    // than time-sliced. RealtimeMesh async-cooks, so the flip itself is cheap; the cook cost
    // is amortized by the async worker and covered by the lead distance.
    for (const TSharedPtr<FMeshChunk>& Chunk : ToProcess)
    {
        if (Chunk.IsValid() && !Chunk->bRetired && Chunk->bWantsCollision != Chunk->bCollisionActive)
            Chunk->SetCollisionActive(Chunk->bWantsCollision);
    }
}

void AAdaptiveVoxelActor::RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    if (EditUpdateIsRunning || IsDestroyed) return;

    EditUpdateIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);
    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, InEditCenter, InEditRadius, InEditStrength, InEditResolution]()
        {
            double t0 = FPlatformTime::Seconds();
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;

            TArray<TSharedPtr<FMeshChunk>> DirtyChunks;
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                Self->AdaptiveOctree->ApplyEdit(InEditCenter, InEditRadius, InEditStrength, InEditResolution);
                Self->AdaptiveOctree->CollectDirtyChunks(DirtyChunks);
            }

            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > EDIT_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] EditUpdate: %.2fms"), elapsed);

            Self->EditUpdateIsRunning = false;

            // Enqueue for the same game-thread drain as the mesh loop; Tick applies within
            // the per-frame budget. Edits touch few chunks, so they typically drain in the
            // next frame or two.
            Self->EnqueueDirtyChunksForApply(DirtyChunks);

        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

bool AAdaptiveVoxelActor::ShouldTickIfViewportsOnly() const
{
    return bTickInEditor && Initialized;
}

void AAdaptiveVoxelActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!Initialized) return;

    // Drain a per-frame time slice of queued chunk applies (component creation + mesh
    // push). Decoupled from the data/mesh loop so it runs at frame rate.
    DrainPendingApply();

    // Destroy components of chunks retired by the chunk-cut pass. After the applies above
    // so the finer replacement chunks get a chance to appear first.
    DrainPendingDestroy();

    // Flip collision on/off for chunks the collision pass re-gated (cook + query).
    DrainPendingCollision();

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

            // Smoothed player speed (cm/s) for the collision lead distance. Smoothing avoids
            // a single stutter frame briefly inflating the collision shell.
            if (bHasSpeedSample && DeltaTime > KINDA_SMALL_NUMBER)
            {
                const double Inst = (CameraPosition - PrevSpeedSamplePos).Size() / DeltaTime;
                PlayerSpeed = FMath::FInterpTo(PlayerSpeed, Inst, DeltaTime, 4.0);
            }
            PrevSpeedSamplePos = CameraPosition;
            bHasSpeedSample = true;

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