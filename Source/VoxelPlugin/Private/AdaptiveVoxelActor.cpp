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

    //Basic density function
    //Torus - Warp Tunnel
    //auto DensityFunction = [](FVector Position) -> double
    //    {
    //        double MajorRadius = 40000000.0; // Distance from the center to the ring
    //        double MinorRadius = 38000000.0; // Tube radius

    //        FVector2D q(FVector2D(Position.X, Position.Y).Size() - MajorRadius, Position.Z);
    //        return q.Size() - MinorRadius;
    //    };
    //Torus
    //auto DensityFunction = [](FVector Position) -> double
    //    {
    //        double MajorRadius = 5000000.0; // Distance from the center to the ring
    //        double MinorRadius = 2000000.0; // Tube radius

    //        FVector2D q(FVector2D(Position.X, Position.Y).Size() - MajorRadius, Position.Z);
    //        return q.Size() - MinorRadius;
    //    };
    //Cube
    //auto DensityFunction = [](FVector Position) -> double
    //    {
    //        FVector CubeSize(20000000.0, 20000000.0, 20000000.0); // Half-size of the cube

    //        FVector q = Position.GetAbs() - CubeSize;
    //        return FMath::Max(q.X, FMath::Max(q.Y, q.Z));
    //    };

    //sphere
    //auto densityfunction = [](fvector position) -> double
    //    {
    //        double sphereradius = 10000000.0;
    //        return position.size() - sphereradius;
    //    };
    //spherenoise
    auto DensityFunction = [&](FVector Position) -> double
        {
            double SphereRadius = 9713713.0;
            float NoiseValue = (FMath::PerlinNoise3D(Position / 977999.0)) * 497997.0;// +(FMath::PerlinNoise3D(Position / 100000.0)) * 100000;
            return Position.Size() - (SphereRadius - NoiseValue);
        };
    //Adaptive Octree Picks out the Implicit Structure
    AdaptiveOctree = MakeShared<FAdaptiveOctree>(DensityFunction, GetActorLocation(), 15001357.0, ChunkDepth, MinDepth, MaxDepth);
    //Sparsetree for user edits
    SparseOctree = MakeShared<FSparseOctree>();
}

void AAdaptiveVoxelActor::BeginDestroy() {
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

// Called when the game starts or when spawned
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
    ScheduleDataUpdate(.1);
    ScheduleMeshUpdate(.1);
}

void AAdaptiveVoxelActor::ScheduleDataUpdate(float IntervalInSeconds)
{
    if (!DataUpdateIsRunning)
    {
        DataUpdateIsRunning = true;
        FGraphEventRef Task = FFunctionGraphTask::CreateAndDispatchWhenReady([this, IntervalInSeconds]()
        {
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            {
                FRWScopeLock WriteLock(OctreeLock, SLT_Write);
                AdaptiveOctree->UpdateLOD(CameraPosition, LodFactor);
            }
            //***********END IMPLEMENTATION BLOCK***************
            //***********END IMPLEMENTATION BLOCK***************
            //***********END IMPLEMENTATION BLOCK***************

            DataUpdateIsRunning = false;
            FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([this, IntervalInSeconds](float DeltaTime)
            {
                ScheduleDataUpdate(IntervalInSeconds);
                return false;
            }), IntervalInSeconds);
        }, TStatId(), nullptr, ENamedThreads::AnyBackgroundThreadNormalTask);
    }
}
void AAdaptiveVoxelActor::ScheduleMeshUpdate(float IntervalInSeconds)
{
    if (!MeshUpdateIsRunning)
    {
        MeshUpdateIsRunning = true;
        FGraphEventRef Task = FFunctionGraphTask::CreateAndDispatchWhenReady([this, IntervalInSeconds]()
        {
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            {
                FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
                AdaptiveOctree->UpdateMesh();
            }
            //***********END IMPLEMENTATION BLOCK***************
            //***********END IMPLEMENTATION BLOCK***************
            //***********END IMPLEMENTATION BLOCK***************

            MeshUpdateIsRunning = false;
            FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([this, IntervalInSeconds](float DeltaTime)
            {
                ScheduleMeshUpdate(IntervalInSeconds);
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
        }
    }
}

TSharedPtr<FAdaptiveOctree> AAdaptiveVoxelActor::GetOctree()
{
    return AdaptiveOctree;
}