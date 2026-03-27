#include "AdaptiveVoxelActor.h"
#include "PlanetActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include "TimerManager.h"
#include <FDensitySampleCompositor.h>

using namespace RealtimeMesh;

//Pipeline log thresholds... log if the pipelines take longer than the set value in MS
const float DATA_LOG_THRESHOLD = 100;
const float MESH_LOG_THRESHOLD = 50;
const float EDIT_LOG_THRESHOLD = 0;

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
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, ChunkCullingMode),
        GET_MEMBER_NAME_CHECKED(AAdaptiveVoxelActor, VolumeSdfRadius),
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

    // Now clean up any components that were already attached
    CleanSceneRoot();

    // Load default plugin material if none assigned by the user.
    // The constructor sets SurfaceMaterial to the engine default as a safe fallback;
    // here we upgrade to the plugin's triplanar material for standalone use.
    if (!SurfaceMaterial || SurfaceMaterial == UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface))
    {
        UMaterialInterface* PluginMaterial = Cast<UMaterialInterface>(
            StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
                TEXT("/VoxelPlugin/MT_TriPlanar.MT_TriPlanar")));
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

    // Compute ChunkDepth from float precision requirements.
    // Chunk-local vertex positions are FVector3f. The float precision at the
    // chunk extent must be below a fixed threshold for artifact-free rendering.
    // FloatPrecision = ChunkExtent * FLT_EPSILON ≈ (RootExtent / 2^D) * 1.19e-7
    {
        constexpr double FloatEps = 1.19e-7;
        constexpr double MaxFloatError = 1.0; // cm — max acceptable vertex jitter
        double Ratio = ActorRootExtent * FloatEps / MaxFloatError;
        int32 IdealChunkDepth = (Ratio > 1.0) ? (int32)FMath::CeilToInt(FMath::Log2(Ratio)) : 2;
        ChunkDepth = FMath::Clamp(IdealChunkDepth, 2, 5);

        MinDepth = FMath::Max(MinDepth, ChunkDepth);

        double ChunkExtent = ActorRootExtent / FMath::Pow(2.0, (double)ChunkDepth);
        double FloatPrec = ChunkExtent * FloatEps;
        if (FloatPrec > MaxFloatError)
        {
            UE_LOG(LogTemp, Warning,
                TEXT("[Octree] Float precision (%.2fcm) exceeds %.0fcm at ChunkDepth %d. "
                    "Reduce planet scale to avoid vertex jitter."),
                FloatPrec, MaxFloatError, ChunkDepth);
        }
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
    PendingParams->MinDepth = MinDepth;
    PendingParams->MaxDepth = MaxDepth;
    PendingParams->PrecisionDepthFloor = PrecisionDepthFloor;
    PendingParams->ChunkCullingMode = (ChunkCullingMode == EChunkCullingMode::Volume)
        ? EOctreeChunkCulling::Volume : EOctreeChunkCulling::Surface;
    PendingParams->VolumeSdfRadius = VolumeSdfRadius;

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
    USceneComponent* InAttachParent)
{
    // Same teardown as Initialize
    Initialized = false;
    bIsPlanetOwned = true;
    PlanetDrivenScale = GetActorScale3D();

    // Guard transforms — bind once to snap back any editor-driven changes.
    if (USceneComponent* Root = GetRootComponent())
    {
        Root->TransformUpdated.RemoveAll(this);
        Root->TransformUpdated.AddUObject(this, &AAdaptiveVoxelActor::OnTransformUpdated);
    }
    while (DataUpdateIsRunning || MeshUpdateIsRunning || EditUpdateIsRunning)
        FPlatformProcess::Sleep(0.001f);

    if (UWorld* World = GetWorld())
        World->GetTimerManager().ClearTimer(DataUpdateTimerHandle);

    if (AdaptiveOctree.IsValid())
    {
        AdaptiveOctree->Clear();
        AdaptiveOctree.Reset();
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

    // Auto-derive ChunkDepth
    {
        constexpr double FloatEps = 1.19e-7;
        constexpr double MaxFloatError = 1.0;
        double Ratio = ActorRootExtent * FloatEps / MaxFloatError;
        int32 IdealChunkDepth = (Ratio > 1.0) ? (int32)FMath::CeilToInt(FMath::Log2(Ratio)) : 2;
        ChunkDepth = FMath::Clamp(IdealChunkDepth, 2, 5);
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
    PendingParams->MinDepth = MinDepth;
    PendingParams->MaxDepth = MaxDepth;
    PendingParams->PrecisionDepthFloor = PrecisionDepthFloor;
    PendingParams->ChunkCullingMode = (ChunkCullingMode == EChunkCullingMode::Volume)
        ? EOctreeChunkCulling::Volume : EOctreeChunkCulling::Surface;
    PendingParams->VolumeSdfRadius = VolumeSdfRadius;

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
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                FVector CurrentCamPos = Self->CameraPosition;
                FVector Velocity = (CurrentCamPos - Self->LastLodUpdatePosition);
                FVector PredictedPos = CurrentCamPos + (Velocity * Self->VelocityLookAheadFactor);
                Self->AdaptiveOctree->UpdateLOD(PredictedPos, Self->ScreenSpaceThreshold, Self->CameraFOV);
                Self->LastLodUpdatePosition = Self->CameraPosition;
            }
            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > DATA_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] DataUpdate: %.2fms"), elapsed);

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
            if (elapsed > MESH_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] MeshUpdate: %.2fms"), elapsed);

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
            double t0 = FPlatformTime::Seconds();
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                Self->AdaptiveOctree->ApplyEdit(InEditCenter, InEditRadius, InEditStrength, InEditResolution);
                Self->AdaptiveOctree->UpdateMesh();
            }

            double elapsed = (FPlatformTime::Seconds() - t0) * 1000.0;
            if (elapsed > EDIT_LOG_THRESHOLD) UE_LOG(LogTemp, Log, TEXT("[Pipeline] EditUpdate: %.2fms"), elapsed);

            Self->EditUpdateIsRunning = false;

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