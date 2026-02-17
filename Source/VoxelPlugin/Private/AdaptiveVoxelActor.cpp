#include "AdaptiveVoxelActor.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"

using namespace RealtimeMesh;
    
// Sets default values
AAdaptiveVoxelActor::AAdaptiveVoxelActor()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;
    CameraPosition = FVector(0, 0, 0);
    ChunkExtent = Size / FMath::Pow(2.0, (double)MinDepth);
    LodDistanceThreshold = ChunkExtent * 2.0 * 1e-6;
    Material = UMaterial::GetDefaultMaterial(EMaterialDomain::MD_Surface);

    //Spherenoise - Example SDF applys perline noise to a sphere with domain shifting and noise scaling to help precision
    auto DensityFunction = [this](FVector Position, FVector AnchorCenter) -> double {
        // 1. High-precision local offset relative to the Chunk Anchor
        FVector LocalOffset = Position - AnchorCenter;

        // 2. Consistent Domain Translation
        double NoiseScale = Size * 0.1;

        // We find the 'Base Domain Coordinate' of the Anchor. 
        // Fmod handles the periodicity (repeating every 256 units).
        double Periodicity = 4096 * 8;
        FVector DomainBase(
            FMath::Fmod(AnchorCenter.X / NoiseScale, Periodicity),
            FMath::Fmod(AnchorCenter.Y / NoiseScale, Periodicity),
            FMath::Fmod(AnchorCenter.Z / NoiseScale, Periodicity)
        );

        // 3. Local Domain Offset
        // This is a small number (e.g., within the chunk's extent)
        FVector DomainLocal = LocalOffset / NoiseScale;

        // Summing two small numbers preserves the precision lost at 10^8
        FVector FinalSample = DomainBase + DomainLocal;
        float NoiseVal = FMath::PerlinNoise3D(FinalSample) * (float)(Size * 0.1);

        // 4. Planet-Relative Distance (Stay in Double)
        FVector PlanetRelativeP = Position - GetActorLocation();
        double RealDist = PlanetRelativeP.Size(); // RelativePos is small enough now

        return RealDist - (Size * 0.85 + (double)NoiseVal);
    };

    //Adaptive octree meshes the implicit structure
    AdaptiveOctree = MakeShared<FAdaptiveOctree>(DensityFunction, GetActorLocation(), Size, MinDepth, MaxDepth);
    //Sparsetree for user edits
    SparseOctree = MakeShared<FSparseOctree>();
}

void AAdaptiveVoxelActor::BeginDestroy() {
    IsDestroyed = true;
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Super::BeginDestroy();
}

void AAdaptiveVoxelActor::OnConstruction(const FTransform& Transform) {
    Super::OnConstruction(Transform);
    // Ensure we are not in a preview world (prevents running in editor mode)
    if (GetWorld() && !GetWorld()->IsPreviewWorld() && TickInEditor)
    {
        InitializeChunks();
    }
}

void AAdaptiveVoxelActor::BeginPlay()
{
    Super::BeginPlay();
    InitializeChunks();
}

void AAdaptiveVoxelActor::CleanSceneRoot() {
    auto destroyComponentArray = GetRootComponent()->GetAttachChildren();
    for (TObjectPtr<USceneComponent> child : destroyComponentArray) {
        URealtimeMeshComponent* meshComponent = Cast<URealtimeMeshComponent>(child);
        if (meshComponent) {
            meshComponent->DestroyComponent();
        }
    }
}

void AAdaptiveVoxelActor::InitializeChunks() {
    CleanSceneRoot();
    AdaptiveOctree->InitializeMeshChunks(this, Material);
    Initialized = true;
    ScheduleDataUpdate(MinDataUpdateInterval);
    ScheduleMeshUpdate(MinMeshUpdateInterval);
}

void AAdaptiveVoxelActor::ScheduleDataUpdate(float IntervalInSeconds)
{
    if (!DataUpdateIsRunning && !IsDestroyed)
    {
        DataUpdateIsRunning = true;
        TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);

        FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, IntervalInSeconds]()
        {
            AAdaptiveVoxelActor* Self = WeakThis.Get();
            if (!Self || Self->IsDestroyed) return;
            {
                FRWScopeLock WriteLock(Self->OctreeLock, SLT_Write);
                FVector CurrentCamPos = Self->CameraPosition;
                FVector Velocity = (CurrentCamPos - Self->LastLodUpdatePosition);
                FVector PredictedPos = CurrentCamPos + (Velocity * 2);
                Self->AdaptiveOctree->UpdateLOD(PredictedPos, Self->ScreenSpaceThreshold, Self->CameraFOV);
                Self->LastLodUpdatePosition = Self->CameraPosition;
            }

            Self->DataUpdateIsRunning = false;
            FTSTicker::GetCoreTicker().AddTicker(
                FTickerDelegate::CreateLambda([WeakThis, IntervalInSeconds](float) {
                    if (auto* S = WeakThis.Get()) S->ScheduleDataUpdate(IntervalInSeconds);
                    return false;
                    }), IntervalInSeconds);
        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundThreadNormalTask);
    }
}

void AAdaptiveVoxelActor::ScheduleMeshUpdate(float IntervalInSeconds)
{
    if (!MeshUpdateIsRunning && !IsDestroyed)
    {
        MeshUpdateIsRunning = true;
        TWeakObjectPtr<AAdaptiveVoxelActor> WeakThis(this);
        FFunctionGraphTask::CreateAndDispatchWhenReady([WeakThis, IntervalInSeconds]()
            {
                AAdaptiveVoxelActor* Self = WeakThis.Get();
                if (!Self || Self->IsDestroyed) return;
                {
                    FRWScopeLock ReadLock(Self->OctreeLock, SLT_ReadOnly);
                    Self->AdaptiveOctree->UpdateMesh();
                }
                Self->MeshUpdateIsRunning = false;
                FTSTicker::GetCoreTicker().AddTicker(
                    FTickerDelegate::CreateLambda([WeakThis, IntervalInSeconds](float) {
                        if (auto* S = WeakThis.Get()) S->ScheduleMeshUpdate(IntervalInSeconds);
                        return false;
                        }), IntervalInSeconds);
            }, TStatId(), nullptr, ENamedThreads::AnyBackgroundThreadNormalTask);
    }
}

bool AAdaptiveVoxelActor::ShouldTickIfViewportsOnly() const
{
    return TickInEditor;
}

void AAdaptiveVoxelActor::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!Initialized) return;

    //Cache cam data
    auto world = GetWorld();
    if (world != nullptr) {
        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0) {
            this->CameraPosition = viewLocations[0];

            APlayerCameraManager* CamManager = UGameplayStatics::GetPlayerCameraManager(world, 0);
            if (CamManager) {
                CameraFOV = CamManager->GetFOVAngle();
            }
        }
    }
}

TSharedPtr<FAdaptiveOctree> AAdaptiveVoxelActor::GetOctree()
{
    return AdaptiveOctree;
}