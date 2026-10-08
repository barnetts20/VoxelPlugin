#include "AdaptiveVoxelActor.h"
#include "PlanetActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include "FDensitySampleCompositor.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "HAL/IConsoleManager.h"
#include "VoxelTerrainStats.h"

DECLARE_CYCLE_STAT(TEXT("Actor tick"), STAT_VoxelActorTick, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Octree: apply chunks"), STAT_VoxelOctreeApply, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Octree: destroy chunks"), STAT_VoxelOctreeDestroy, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Octree: data task (worker)"), STAT_VoxelOctreeDataTask, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Octree: mesh task (worker)"), STAT_VoxelOctreeMeshTask, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Collision tick"), STAT_VoxelCollisionTick, STATGROUP_VoxelTerrain);

static TAutoConsoleVariable<int32> CVarVoxelRenderBricks(
    TEXT("voxel.Render.Bricks"), 1,
    TEXT("1 (default): render terrain from LOD bricks (VoxelBrickTerrain) instead of the octree; the octree's chunks are hidden and its LOD passes paused. 0: the octree."),
    ECVF_Default);

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

    BrickAttachmentRoot = CreateDefaultSubobject<USceneComponent>(TEXT("BrickAttachmentRoot"));
    BrickAttachmentRoot->SetupAttachment(GetRootComponent());
    BrickAttachmentRoot->SetAbsolute(false, false, true);
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
        PendingEditNotices.Empty();
        DebugStaleBounds.Empty();
        DebugFoldedBounds.Empty();
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
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, SurfaceMaterial),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, ScreenSpaceThreshold),
    };

    // A change inside SurfaceSettings reports the inner field as the property; the member
    // property is the struct.
    // Slider drags rebuild once, on release.
    const bool bSurfaceChanged = !bIsPlanetOwned && PropertyChangedEvent.ChangeType != EPropertyChangeType::Interactive
        && PropertyChangedEvent.MemberProperty
        && PropertyChangedEvent.MemberProperty->GetFName() == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, SurfaceSettings);

    if (RebuildProps.Contains(PropName) || bSurfaceChanged)
    {
        Initialize();
        return;
    }

    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, BrickMorphParameters))
    {
        if (BrickTerrain.IsValid())
            BrickTerrain->SetMorphParameters(BrickMorphParameters.Get());
        return;
    }

    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, BrickLodDistanceFactor)
        || PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, BrickCellsPerBrick))
    {
        const double ActorPlanetRadius = GetActorScale3D().GetMax();
        InitializeBrickTerrain(VoxelDensity::RootExtentFor(ActorPlanetRadius, ActorPlanetRadius * NoiseAmplitudeRatio));
        return;
    }

    // Collision lattice changes rebuild only the collision bricks.
    if (PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, CollisionCellScale)
        || PropName == GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, CollisionSlopeBound))
    {
        const double ActorPlanetRadius = GetActorScale3D().GetMax();
        InitializeCollision(VoxelDensity::RootExtentFor(ActorPlanetRadius, ActorPlanetRadius * NoiseAmplitudeRatio));
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
    // Both roots: components left from an earlier run (saved with the level, or copied from the
    // editor world into PIE before they were transient) would otherwise keep rendering or
    // colliding without anything tracking them.
    for (USceneComponent* AttachRoot : { MeshAttachmentRoot.Get(), BrickAttachmentRoot.Get() })
    {
        if (!AttachRoot) continue;
        const TArray<TObjectPtr<USceneComponent>> Attached = AttachRoot->GetAttachChildren();
        for (const TObjectPtr<USceneComponent>& Child : Attached)
        {
            if (URealtimeMeshComponent* MeshComponent = Cast<URealtimeMeshComponent>(Child))
                MeshComponent->DestroyComponent();
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

    // Destroy old octree first - this releases all chunk shared pointers
    if (AdaptiveOctree.IsValid())
    {
        AdaptiveOctree->Clear();
        AdaptiveOctree.Reset();
    }

    // Drop queued applies referencing the old octree's chunks. The busy-wait above has
    // already fenced every task's enqueue (each enqueues before clearing its running
    // flag), so nothing repopulates this after the clear.
    {
        FScopeLock Lock(&PendingApplyCS);
        PendingApply.Empty();
        PendingDestroy.Empty();
        PendingEditNotices.Empty();
        DebugStaleBounds.Empty();
        DebugFoldedBounds.Empty();
    }
    ApplyHeldFrames = 0;

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
    double ActorRootExtent = VoxelDensity::RootExtentFor(ActorPlanetRadius, ActorNoiseAmplitude);

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

    TSharedPtr<FSparseEditStore> EditStore = MakeShared<FSparseEditStore>(FVector::ZeroVector, ActorRootExtent, MaxDepth);
    TSharedPtr<FDensitySampleCompositor> Compositor = VoxelDensity::MakeCompositor(SurfaceSettings, ActorPlanetRadius, ActorNoiseAmplitude, EditStore,
        VoxelDensity::TerrainFinestCell(ActorPlanetRadius, ActorNoiseAmplitude, MaxDepth));
    DensityCompositor = Compositor;
    InitializeCollision(ActorRootExtent);
    InitializeBrickTerrain(ActorRootExtent);
    // The root's visibility is saved with the level; match it to the current render mode.
    if (MeshAttachmentRoot)
        MeshAttachmentRoot->SetVisibility(!bBrickRenderActive, true);

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
        PendingEditNotices.Empty();
        DebugStaleBounds.Empty();
        DebugFoldedBounds.Empty();
    }
    ApplyHeldFrames = 0;

    CleanSceneRoot();

    // Re-parent MeshAttachmentRoot to the planet's component hierarchy.
    // This ensures mesh chunks follow the planet's position/rotation directly.
    if (InAttachParent && MeshAttachmentRoot)
    {
        MeshAttachmentRoot->AttachToComponent(InAttachParent,
            FAttachmentTransformRules::KeepWorldTransform);
    }
    if (InAttachParent && BrickAttachmentRoot)
    {
        BrickAttachmentRoot->AttachToComponent(InAttachParent,
            FAttachmentTransformRules::KeepWorldTransform);
    }

    // Scale = planet radius, same as standalone Initialize
    double ActorPlanetRadius = GetActorScale3D().GetMax();
    double ActorNoiseAmplitude = ActorPlanetRadius * NoiseAmplitudeRatio;
    double ActorRootExtent = VoxelDensity::RootExtentFor(ActorPlanetRadius, ActorNoiseAmplitude);

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
    DensityCompositor = InCompositor;
    InitializeCollision(ActorRootExtent);
    InitializeBrickTerrain(ActorRootExtent);
    // The root's visibility is saved with the level; match it to the current render mode.
    if (MeshAttachmentRoot)
        MeshAttachmentRoot->SetVisibility(!bBrickRenderActive, true);
    PendingParams->PlanetRadius = ActorPlanetRadius;
    PendingParams->NoiseAmplitude = ActorNoiseAmplitude;
    PendingParams->ChunkDepth = ChunkDepth;
    PendingParams->MaxChunkDepth = MaxChunkDepth;
    PendingParams->ChunkPrecisionThreshold = ChunkPrecisionThreshold;
    PendingParams->ChunkDemoteHysteresis = ChunkDemoteHysteresis;
    PendingParams->MaxChunkDepthDelta = MaxChunkDepthDelta;
    PendingParams->MinDepth = MinDepth;
    PendingParams->MaxDepth = MaxDepth;

    LastInitScale = GetActorScale3D();
    Initialized = true;
    RunDataUpdateTask();
}

void AAdaptiveVoxelActor::RunDataUpdateTask()
{
    if (DataUpdateIsRunning || IsDestroyed) return;
    // Paused while the brick terrain renders; the first pass still builds the octree, which
    // edits need.
    if (bBrickRenderActive && AdaptiveOctree.IsValid() && !PendingParams.IsValid()) return;

    DataUpdateIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);

    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis]()
        {
            SCOPE_CYCLE_COUNTER(STAT_VoxelOctreeDataTask);
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

            const bool bDebug = CVarVoxelDebugFlicker.GetValueOnAnyThread() > 0;
            bool bSeamChecked = false;
            TArray<FBox> StaleBounds;
            int32 DroppedInterior = 0;
            int32 DroppedFace = 0;
            int32 VertexlessQuads = 0;
            int32 FoldedTris = 0;
            TArray<FBox> FoldedBounds;

            double t0 = FPlatformTime::Seconds();
            TArray<TSharedPtr<FMeshChunk>> RemovedChunks;
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                Self->AdaptiveOctree->BeginPass();

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

                if (bDebug)
                {
                    Self->AdaptiveOctree->ConsumeDebugCounters(DroppedInterior, DroppedFace, VertexlessQuads, FoldedTris);
                    Self->AdaptiveOctree->FindFoldedChunks(FoldedBounds);
                    const uint32 Interval = (uint32)FMath::Max(1, CVarVoxelDebugSeamCheckInterval.GetValueOnAnyThread());
                    if (++Self->DebugDataPassCount % Interval == 0)
                    {
                        Self->AdaptiveOctree->FindStaleSeams(StaleBounds);
                        bSeamChecked = true;
                    }
                }
            }
            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > DATA_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] DataUpdate: %.2fms"), elapsed);

            if (bDebug)
            {
                if (DroppedInterior > 0 || DroppedFace > 0)
                    UE_LOG(LogTemp, Warning, TEXT("[VoxelFlicker] Dropped edges (a neighbour >2 levels coarser): %d interior, %d on chunk faces"), DroppedInterior, DroppedFace);
                if (bSeamChecked && StaleBounds.Num() > 0)
                    UE_LOG(LogTemp, Warning, TEXT("[VoxelFlicker] Stale seams: %d chunks meshed against outdated neighbours"), StaleBounds.Num());
                if (VertexlessQuads > 0)
                    UE_LOG(LogTemp, Warning, TEXT("[VoxelFlicker] Quads skipped for a leaf without vertex data: %d"), VertexlessQuads);
                if (FoldedTris > 0)
                    UE_LOG(LogTemp, Warning, TEXT("[VoxelFlicker] Folded triangles meshed this pass: %d (%d chunks now hold folds)"), FoldedTris, FoldedBounds.Num());
            }

            if (RemovedChunks.Num() > 0 || bSeamChecked || bDebug)
            {
                FScopeLock Lock(&Self->PendingApplyCS);
                if (RemovedChunks.Num() > 0)     Self->PendingDestroy.Append(MoveTemp(RemovedChunks));
                if (bSeamChecked)
                {
                    Self->DebugStaleBounds = MoveTemp(StaleBounds);
                    Self->DebugStaleBoundsTime = FPlatformTime::Seconds();
                }
                if (bDebug)
                    Self->DebugFoldedBounds = MoveTemp(FoldedBounds);
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
            SCOPE_CYCLE_COUNTER(STAT_VoxelOctreeMeshTask);
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;

            // Collect the dirty chunks under the read lock -- this is cheap (it just
            // gathers shared refs). The RealtimeMesh apply happens on the game thread in
            // Tick, so the octree lock is never held across component work.
            double t0 = FPlatformTime::Seconds();
            const bool bDebug = CVarVoxelDebugFlicker.GetValueOnAnyThread() > 0;
            const bool bLogStats = bDebug && (t0 - Self->DebugLastStatsTime) > 1.0;
            FOctreeDebugStats Stats;
            TArray<TSharedPtr<FMeshChunk>> DirtyChunks;
            {
                FRWScopeLock ReadLock(Self->OctreeLock, SLT_ReadOnly);
                if (Self->AdaptiveOctree.IsValid())
                {
                    Self->AdaptiveOctree->CollectDirtyChunks(DirtyChunks);
                    if (bLogStats)
                        Self->AdaptiveOctree->GetDebugStats(Stats);
                }
            }

            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > MESH_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] MeshUpdate: %.2fms"), elapsed);

            // Enqueue for the game-thread drain BEFORE clearing the running flag, so the
            // teardown busy-wait in Initialize (which waits on MeshUpdateIsRunning) fences
            // the enqueue -- a re-init can't clear PendingApply and then have this task add
            // stale chunks after it.
            Self->EnqueueDirtyChunksForApply(DirtyChunks);

            if (bLogStats)
            {
                Self->DebugLastStatsTime = t0;
                int32 NumQueued = 0;
                {
                    FScopeLock Lock(&Self->PendingApplyCS);
                    NumQueued = Self->PendingApply.Num();
                }
                const uint64 OldestAge = Stats.OldestDirtyPublishFrame > 0 ? (GFrameCounter - Stats.OldestDirtyPublishFrame) : 0;
                UE_LOG(LogTemp, Log, TEXT("[VoxelFlicker] Chunks %d, dirty %d (oldest unapplied: %llu frames), queued %d"),
                    Stats.NumChunks, Stats.NumDirty, OldestAge, NumQueued);
            }

            Self->MeshUpdateIsRunning = false;

            // Chain the next data update on the normal interval. The apply is decoupled --
            // Tick drains PendingApply at frame rate -- so it isn't gated to this loop's
            // slower cadence.
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
    SCOPE_CYCLE_COUNTER(STAT_VoxelOctreeApply);
    if (!Initialized || IsDestroyed) return;

    TArray<TSharedPtr<FMeshChunk>> Queue;
    {
        FScopeLock Lock(&PendingApplyCS);
        if (PendingApply.Num() == 0) return;
        Queue = MoveTemp(PendingApply);
        PendingApply.Reset();
    }

    // Capture each chunk's latest stream once; that exact stream is what gets applied, so
    // a stream published meanwhile can't slip into the wrong group. FirstPass..LastPass
    // spans the passes the chunk has unapplied work from.
    struct FQueued
    {
        TSharedPtr<FMeshChunk> Chunk;
        TSharedPtr<FMeshStreamData> Stream;
        uint32 FirstPass;
        uint32 LastPass;
    };
    TArray<FQueued> Entries;
    Entries.Reserve(Queue.Num());
    for (TSharedPtr<FMeshChunk>& Chunk : Queue)
    {
        if (!Chunk.IsValid()) continue;
        if (Chunk->bRetired || !Chunk->IsDirty())
        {
            // Leave the queue, then re-check: a publish landing in between either sees the
            // cleared flag and re-enqueues, or is caught here and kept.
            {
                FScopeLock Lock(&PendingApplyCS);
                Chunk->bApplyQueued = false;
            }
            if (Chunk->bRetired || !Chunk->IsDirty()) continue;
            FScopeLock Lock(&PendingApplyCS);
            if (Chunk->bApplyQueued) continue;
            Chunk->bApplyQueued = true;
        }
        TSharedPtr<FMeshStreamData> Stream = Chunk->GetPublished();
        if (!Stream.IsValid()) continue;
        const uint32 LastPass = Stream->PassId;
        const uint32 First = Chunk->FirstUnappliedPass.load(std::memory_order_relaxed);
        Entries.Add({ Chunk, Stream, (First != 0 && First < LastPass) ? First : LastPass, LastPass });
    }
    Entries.StableSort([](const FQueued& A, const FQueued& B)
        {
            return A.FirstPass != B.FirstPass ? A.FirstPass < B.FirstPass : A.LastPass < B.LastPass;
        });

    const double BudgetSeconds = FMath::Max(0.1, ChunkApplyBudgetMs) / 1000.0;
    const double StartTime = FPlatformTime::Seconds();

    // 1. Create missing components within the budget: grouped work first, oldest first,
    //    since a group waits for all of its components; then the initial build, which
    //    blocks nothing. A new component's section group is empty, so creating it early
    //    shows nothing.
    for (int32 Round = 0; Round < 2; Round++)
    {
        const bool bBuildRound = Round == 1;
        for (FQueued& Entry : Entries)
        {
            if (FPlatformTime::Seconds() - StartTime > BudgetSeconds) break;
            if ((Entry.FirstPass == FAdaptiveOctree::BuildPassId) != bBuildRound) continue;
            if (Entry.Chunk->NeedsComponentFor(Entry.Stream))
                Entry.Chunk->EnsureComponent();
        }
    }

    auto ApplyEntry = [this](FQueued& Entry)
        {
            {
                // Cleared before the apply: a stream published during it re-enqueues.
                FScopeLock Lock(&PendingApplyCS);
                Entry.Chunk->bApplyQueued = false;
            }
            Entry.Chunk->ApplyStream(Entry.Stream);
            Entry.Chunk.Reset();
        };

    // 2. The initial build has no seams on screen to keep consistent yet: its chunks fill
    //    in as their components appear. (They sort first.)
    int32 First = 0;
    while (First < Entries.Num() && Entries[First].FirstPass == FAdaptiveOctree::BuildPassId)
    {
        if (!Entries[First].Chunk->NeedsComponentFor(Entries[First].Stream))
            ApplyEntry(Entries[First]);
        First++;
    }

    // 3. Everything later applies in groups: entries whose pass ranges overlap belong
    //    together (a chunk with work from passes 10..12 ties those passes), and a group
    //    applies in one frame once all its components exist. A waiting group holds back
    //    newer ones, whose seams may depend on it. After MaxApplyHoldFrames consecutive
    //    frames of waiting, every group's ready chunks go up anyway.
    constexpr int32 MaxApplyHoldFrames = 30;
    const bool bDebug = CVarVoxelDebugFlicker.GetValueOnGameThread() > 0;
    const bool bReleaseHeld = ApplyHeldFrames >= MaxApplyHoldFrames;
    bool bHeadWaiting = false;
    int32 GroupStart = First;
    while (GroupStart < Entries.Num())
    {
        const uint32 GroupPass = Entries[GroupStart].FirstPass;
        uint32 GroupLastPass = Entries[GroupStart].LastPass;
        int32 GroupEnd = GroupStart;
        bool bReady = true;
        while (GroupEnd < Entries.Num() && Entries[GroupEnd].FirstPass <= GroupLastPass)
        {
            GroupLastPass = FMath::Max(GroupLastPass, Entries[GroupEnd].LastPass);
            if (Entries[GroupEnd].Chunk->NeedsComponentFor(Entries[GroupEnd].Stream)) bReady = false;
            GroupEnd++;
        }

        if (!bReady)
        {
            if (GroupStart == First) bHeadWaiting = true;
            if (!bReleaseHeld) break;
            if (bDebug)
                UE_LOG(LogTemp, Warning, TEXT("[VoxelFlicker] Passes %u-%u waited %d frames for components; applying the ready part"),
                    GroupPass, GroupLastPass, ApplyHeldFrames);
            for (int32 i = GroupStart; i < GroupEnd; i++)
                if (!Entries[i].Chunk->NeedsComponentFor(Entries[i].Stream))
                    ApplyEntry(Entries[i]);
            GroupStart = GroupEnd;
            continue;
        }

        if (bDebug && GroupStart == First && ApplyHeldFrames > 0)
        {
            UE_LOG(LogTemp, Log, TEXT("[VoxelFlicker] Passes %u-%u applied (%d chunks) after waiting %d frames for components"),
                GroupPass, GroupLastPass, GroupEnd - GroupStart, ApplyHeldFrames);
        }

        for (int32 i = GroupStart; i < GroupEnd; i++)
            ApplyEntry(Entries[i]);
        GroupStart = GroupEnd;
    }

    // Consecutive frames the oldest group has been waiting for components.
    ApplyHeldFrames = bHeadWaiting ? ApplyHeldFrames + 1 : 0;

    // Return the rest to the queue (still marked queued).
    FScopeLock Lock(&PendingApplyCS);
    for (FQueued& Entry : Entries)
    {
        if (Entry.Chunk.IsValid())
            PendingApply.Add(MoveTemp(Entry.Chunk));
    }
}

// A replacement chunk covers its space once its render proxy has a mesh (IsDisplayed), or
// immediately when it was applied with nothing to show. A chunk retired before it ever
// applied is settled once its OWN replacements are settled -- this follows a fast
// promote/demote cascade to the bottom so we never release a coarse chunk while a hole
// would remain.
static bool IsChunkSettled(const FMeshChunk* Chunk)
{
    if (!Chunk) return true;
    if (Chunk->IsDisplayed()) return true;
    if (!Chunk->IsDirty() && !Chunk->IsInitialized) return true;   // applied, nothing to show
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
    SCOPE_CYCLE_COUNTER(STAT_VoxelOctreeDestroy);
    if (IsDestroyed) return;

    TArray<TSharedPtr<FMeshChunk>> ToProcess;
    {
        FScopeLock Lock(&PendingApplyCS);
        if (PendingDestroy.Num() == 0) return;
        ToProcess = MoveTemp(PendingDestroy);
    }

    // Release a retired chunk only once every chunk that replaces it is on screen. Until
    // then it keeps rendering its last mesh, so the promote/demote swap never opens a hole.
    // Chunks that aren't ready yet are re-queued for a later frame; MaxRetireWaitFrames after
    // all replacements have applied, one is released anyway, so a display report that never
    // arrives can't pin old geometry.
    constexpr int32 MaxRetireWaitFrames = 120;
    TArray<TSharedPtr<FMeshChunk>> Deferred;
    for (const TSharedPtr<FMeshChunk>& Chunk : ToProcess)
    {
        if (!Chunk.IsValid()) continue;

        bool bReplacementsLive = true;
        bool bReplacementsApplied = true;
        for (const TSharedPtr<FMeshChunk>& Rep : Chunk->ReplacedBy)
        {
            if (!IsChunkSettled(Rep.Get())) bReplacementsLive = false;
            if (Rep.IsValid() && !Rep->bRetired && Rep->AppliedGen.load(std::memory_order_acquire) == 0) bReplacementsApplied = false;
        }
        // The timeout only runs once every replacement has been applied: it guards against
        // a display report that never arrives, not against applies still queued.
        if (!bReplacementsLive && bReplacementsApplied && ++Chunk->RetireWaitFrames > MaxRetireWaitFrames)
            bReplacementsLive = true;

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

void AAdaptiveVoxelActor::InitializeCollision(double RootExtent)
{
    if (!CollisionBricks.IsValid())
        CollisionBricks = MakeShared<FVoxelCollisionBricks>();

    // The finest octree lattice: corners at -RootExtent + k * ActualPrecision on each axis.
    FVoxelBrickGrid Grid;
    Grid.Origin = FVector(-RootExtent);
    Grid.CellSize = ActualPrecision * FMath::Max(1, CollisionCellScale);
    Grid.CellsPerBrick = 16;
    CollisionBricks->Initialize(this, BrickAttachmentRoot, DensityCompositor, Grid, CollisionSlopeBound);
}

void AAdaptiveVoxelActor::InitializeBrickTerrain(double RootExtent)
{
    if (!BrickTerrain.IsValid())
        BrickTerrain = MakeShared<FVoxelBrickTerrain>();

    // Bricks of 2^k cells: level MaxDepth - k is a single brick covering the whole root.
    const int32 BrickLog2 = FMath::Clamp(FMath::RoundToInt(FMath::Log2((double)FMath::Max(BrickCellsPerBrick, 1))), 3, 6);
    FVoxelBrickTerrainSettings Settings;
    Settings.Origin = FVector(-RootExtent);
    Settings.FinestCell = ActualPrecision;
    Settings.CellsPerBrick = 1 << BrickLog2;
    Settings.MaxLevel = FMath::Max(0, MaxDepth - BrickLog2);
    Settings.LodDistanceFactor = BrickLodDistanceFactor;
    Settings.SlopeBound = CollisionSlopeBound;
    Settings.TriplanarUVScale = 0.0001 * (80000000.0 / FMath::Max(GetActorScale3D().GetMax(), 1.0));
    Settings.Material = SurfaceMaterial;
    // The plugin's collection when none is assigned (the planet spawns this actor).
    if (!BrickMorphParameters)
        BrickMorphParameters = Cast<UMaterialParameterCollection>(StaticLoadObject(UMaterialParameterCollection::StaticClass(), nullptr,
            TEXT("/VoxelPlugin/MPC_VoxelMorph.MPC_VoxelMorph"), nullptr, LOAD_NoWarn | LOAD_Quiet));
    Settings.MorphParameters = BrickMorphParameters.Get();
    BrickTerrain->Initialize(this, BrickAttachmentRoot, DensityCompositor, Settings);
}

void AAdaptiveVoxelActor::SetBrickRender(bool bOn)
{
    bBrickRenderActive = bOn;
    if (MeshAttachmentRoot)
        MeshAttachmentRoot->SetVisibility(!bOn, true);

    if (!bOn)
    {
        if (BrickTerrain.IsValid())
            BrickTerrain->Reset();
        // Resume the octree's LOD passes (a pass still in flight continues the chain itself).
        // The camera moved while paused; don't predict the first pass from that jump.
        LastLodUpdatePosition = CameraPosition;
        if (!DataUpdateIsRunning && !MeshUpdateIsRunning)
            RunDataUpdateTask();
    }
}

void AAdaptiveVoxelActor::LogEditHit(const FHitResult& Hit, const FVector& LocalHit) const
{
    static IConsoleVariable* DebugCollision = IConsoleManager::Get().FindConsoleVariable(TEXT("voxel.Debug.Collision"));
    if (!DebugCollision || DebugCollision->GetInt() <= 0 || !CollisionBricks.IsValid()) return;
    UE_LOG(LogTemp, Log, TEXT("[VoxelCollision] Edit trace: %s"), *CollisionBricks->DescribeHit(Hit.GetComponent(), LocalHit));
}

void AAdaptiveVoxelActor::TickCollision(UWorld* World)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelCollisionTick);
    if (!CollisionBricks.IsValid() || !World) return;

    TArray<FEditNotice> Notices;
    {
        FScopeLock Lock(&PendingApplyCS);
        Notices = MoveTemp(PendingEditNotices);
        PendingEditNotices.Reset();
    }
    for (const FEditNotice& Notice : Notices)
    {
        CollisionBricks->NotifyEdit(Notice.Sequence, Notice.Center, Notice.Radius);
        if (bBrickRenderActive && BrickTerrain.IsValid())
            BrickTerrain->NotifyEdit(Notice.Sequence, Notice.Center, Notice.Radius);
    }

    TArray<FVoxelCollisionSource> Sources;
    if (bEnableSurfaceCollision)
    {
        const double MaxRadius = FMath::Max(MaxCollisionDistance, WalkBaseCollisionDistance);
        auto RadiusFor = [this, MaxRadius](double Speed)
            {
                return FMath::Min(WalkBaseCollisionDistance + Speed * CollisionCookLeadTime, MaxRadius);
            };

        const FTransform NoScaleTransform(GetActorRotation(), GetActorLocation());
        for (FConstPlayerControllerIterator It = World->GetPlayerControllerIterator(); It; ++It)
        {
            const APlayerController* PC = It->Get();
            const APawn* Pawn = PC ? PC->GetPawn() : nullptr;
            if (!Pawn) continue;
            // A pawn moved by setting its location reports no velocity; the local camera's
            // smoothed speed covers that case.
            const double Speed = FMath::Max((double)Pawn->GetVelocity().Size(), PC->IsLocalController() ? PlayerSpeed : 0.0);
            Sources.Add({ NoScaleTransform.InverseTransformPosition(Pawn->GetActorLocation()), RadiusFor(Speed) });
        }
        if (Sources.Num() == 0 && bHasSpeedSample)
            Sources.Add({ CameraPosition, RadiusFor(PlayerSpeed) });
    }

    CollisionBricks->Tick(Sources, CollisionHysteresis);
    if (BrickAttachmentRoot)
        CollisionBricks->DrawDebug(World, BrickAttachmentRoot->GetComponentTransform());
}

void AAdaptiveVoxelActor::RunDeferredOctreeEdits()
{
    TArray<FEditNotice> Edits;
    {
        FScopeLock Lock(&PendingApplyCS);
        Edits = MoveTemp(DeferredOctreeEdits);
        DeferredOctreeEdits.Reset();
    }
    if (Edits.Num() == 0 || IsDestroyed) return;

    EditUpdateIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);
    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, Edits = MoveTemp(Edits)]()
        {
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;
            TArray<TSharedPtr<FMeshChunk>> DirtyChunks;
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                if (Self->AdaptiveOctree.IsValid())
                {
                    Self->AdaptiveOctree->BeginPass();
                    for (const FEditNotice& Edit : Edits)
                        Self->AdaptiveOctree->RefreshEdit(Edit.Center, Edit.Radius);
                    Self->AdaptiveOctree->CollectDirtyChunks(DirtyChunks);
                }
            }
            Self->EnqueueDirtyChunksForApply(DirtyChunks);
            Self->EditUpdateIsRunning = false;
        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

void AAdaptiveVoxelActor::RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    if (EditUpdateIsRunning || IsDestroyed) return;

    EditUpdateIsRunning = true;
    TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);
    const bool bStoreOnly = bBrickRenderActive;
    FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, InEditCenter, InEditRadius, InEditStrength, InEditResolution, bStoreOnly]()
        {
            double t0 = FPlatformTime::Seconds();
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;

            TArray<TSharedPtr<FMeshChunk>> DirtyChunks;
            bool bApplied = false;
            const TSharedPtr<FSparseEditStore> Store = Self->DensityCompositor.IsValid() ? Self->DensityCompositor->GetEditStore() : nullptr;
            if (bStoreOnly && Store.IsValid())
            {
                // The brick terrain and the collision read the edit store alone: no octree
                // lock or rebuild. The octree catches up if it renders again.
                Store->ApplySphericalEdit(InEditCenter, InEditRadius, InEditStrength, Store->GetDepthForBrushRadius(InEditRadius, InEditResolution));
                bApplied = true;
                FScopeLock Lock(&Self->PendingApplyCS);
                Self->DeferredOctreeEdits.Add({ Store->GetLastSequence(), InEditCenter, InEditRadius });
            }
            else
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                // The octree is built by the first data pass; an edit before that is dropped.
                if (Self->AdaptiveOctree.IsValid())
                {
                    Self->AdaptiveOctree->BeginPass();
                    Self->AdaptiveOctree->ApplyEdit(InEditCenter, InEditRadius, InEditStrength, InEditResolution);
                    Self->AdaptiveOctree->CollectDirtyChunks(DirtyChunks);
                    bApplied = true;
                }
            }

            // The collision bricks and the brick terrain rebuild what this edit reaches.
            if (bApplied && Store.IsValid())
            {
                FScopeLock Lock(&Self->PendingApplyCS);
                Self->PendingEditNotices.Add({ Store->GetLastSequence(), InEditCenter, InEditRadius });
            }

            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > EDIT_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] EditUpdate: %.2fms"), elapsed);

            // Enqueue before clearing the running flag: Initialize waits on the flag and then
            // clears the queue, so chunks of a replaced octree never land after that clear.
            Self->EnqueueDirtyChunksForApply(DirtyChunks);
            Self->EditUpdateIsRunning = false;

        }, TStatId(), nullptr, ENamedThreads::AnyNormalThreadHiPriTask);
}

bool AAdaptiveVoxelActor::ShouldTickIfViewportsOnly() const
{
    return bTickInEditor && Initialized;
}

void AAdaptiveVoxelActor::Tick(float DeltaTime)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelActorTick);
    Super::Tick(DeltaTime);
    if (!Initialized) return;

    const bool bWantBricks = CVarVoxelRenderBricks.GetValueOnGameThread() > 0;
    if (bWantBricks != bBrickRenderActive)
        SetBrickRender(bWantBricks);

    // The octree's chunks wait in their queues while the brick terrain renders, so none
    // appears under the hidden root.
    if (!bBrickRenderActive)
    {
        // Edits made while the bricks rendered reach the octree first.
        if (!EditUpdateIsRunning)
            RunDeferredOctreeEdits();

        // Create components and swap in finished passes (component creation is time-sliced).
        DrainPendingApply();

        // Destroy components of chunks retired by the chunk-cut pass. After the applies
        // above so the finer replacement chunks get a chance to appear first.
        DrainPendingDestroy();
    }

    // Cache cam data
    auto world = GetWorld();
    if (world != nullptr)
    {
        // Flicker diagnostics: outline chunks the last stale-seam check flagged (red) and
        // chunks whose current mesh holds folded triangles (yellow).
        if (CVarVoxelDebugFlicker.GetValueOnGameThread() > 0 && MeshAttachmentRoot)
        {
            TArray<FBox> StaleBounds;
            TArray<FBox> FoldedBounds;
            {
                FScopeLock Lock(&PendingApplyCS);
                if (FPlatformTime::Seconds() - DebugStaleBoundsTime < 2.0)
                    StaleBounds = DebugStaleBounds;
                FoldedBounds = DebugFoldedBounds;
            }
            const FTransform& ToWorld = MeshAttachmentRoot->GetComponentTransform();
            for (const FBox& Bounds : StaleBounds)
            {
                DrawDebugBox(world, ToWorld.TransformPosition(Bounds.GetCenter()), Bounds.GetExtent(),
                    ToWorld.GetRotation(), FColor::Red, false, -1.f, 0, 50.f);
            }
            for (const FBox& Bounds : FoldedBounds)
            {
                DrawDebugBox(world, ToWorld.TransformPosition(Bounds.GetCenter()), Bounds.GetExtent(),
                    ToWorld.GetRotation(), FColor::Yellow, false, -1.f, 0, 30.f);
            }
        }

        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0)
        {
            // Convert world-space camera to actor-local space.
            // Position and rotation only - octree is built at world scale, not normalized.
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

        TickCollision(world);
        if (bBrickRenderActive && BrickTerrain.IsValid())
            BrickTerrain->Tick(CameraPosition);

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
                    LogEditHit(Hit, LocalHit);
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
                    LogEditHit(Hit, LocalHit);
                    RunEditUpdateTask(LocalHit, InEditRadius, -InEditStrength, 3);
                    DrawDebugSphere(world, Hit.ImpactPoint, InEditRadius, 32, FColor::Green, false, DebugDrawTime);
                }
            }
        }
    }
}
