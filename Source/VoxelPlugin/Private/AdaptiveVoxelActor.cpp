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
    //auto DensityFunction = [this](FVector Position) -> double
    //    {
    //        double MajorRadius = Size * .5; // Distance from the center to the ring
    //        double MinorRadius = Size * .2; // Tube radius

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
    //auto DensityFunction = [&](FVector Position) -> double
    //    {
    //        double SphereRadius = Size * .85;
    //        float NoiseValue = (FMath::PerlinNoise3D(Position / (Size * .2))) * Size * .1;// +(FMath::PerlinNoise3D(Position / 100000.0)) * 100000;
    //        return Position.Size() - (SphereRadius + NoiseValue);
    //    };
    //Torus noise
    //auto DensityFunction = [this](FVector Position) -> double
    //{
    //        double MajorRadius = Size * .5; // Distance from the center to the ring
    //        double MinorRadius = Size * .2; // Tube radius
    //        float NoiseValue = (FMath::PerlinNoise3D(Position / (Size * .2))) * Size * .1;// +(FMath::PerlinNoise3D(Position / 100000.0)) * 100000;
    //        FVector2D q(FVector2D(Position.X, Position.Y).Size() - MajorRadius + NoiseValue, Position.Z);
    //        return q.Size() - MinorRadius + NoiseValue;
    //};
    auto DensityFunction = [&](FVector Position) -> double
        {
            double SphereRadius = Size * .85;

            // Normalize the position to get the direction
            FVector Direction = Position.GetSafeNormal();

            // Create a wave pattern based on latitude and longitude
            double latitude = FMath::Asin(Direction.Z);
            double longitude = FMath::Atan2(Direction.Y, Direction.X);

            // Create topographical variation using sin/cos
            double frequency1 = 12.0;  // Controls how many waves around the sphere
            double frequency2 = 8.0;  // Secondary wave pattern
            double amplitude = Size * 0.1;  // Same scale as the Perlin noise was using

            double variation = amplitude * (
                0.5 * FMath::Sin(frequency1 * latitude) * FMath::Cos(frequency1 * longitude) +
                0.3 * FMath::Sin(frequency2 * latitude * 2.0) * FMath::Cos(frequency2 * longitude * 0.5) +
                0.2 * FMath::Sin(frequency1 * longitude * 0.7)
                );

            // Return signed distance
            return Position.Size() - (SphereRadius + variation);
        };
    //Adaptive Octree Picks out the Implicit Structure
    AdaptiveOctree = MakeShared<FAdaptiveOctree>(DensityFunction, GetActorLocation(), Size, ChunkDepth, MinDepth, MaxDepth);
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
    ScheduleDataUpdate(.05);
    ScheduleMeshUpdate(.1);

    FVector quadrants[8];
    quadrants[0] = FVector(1, 1, 1);
    quadrants[1] = FVector(-1, 1, 1);
    quadrants[2] = FVector(1, -1, 1);
    quadrants[3] = FVector(1, 1, -1);
    quadrants[4] = FVector(-1, -1, 1);
    quadrants[5] = FVector(1, -1, -1);
    quadrants[6] = FVector(-1, 1, -1);
    quadrants[7] = FVector(-1, -1, -1);

    FVector oneVec = FVector(1, 1, 1);
    double scl = 1000000000;
    DrawDebugBox(GetWorld(), quadrants[0] * scl, scl * oneVec, FColor::Red, true, 1000);
    DrawDebugBox(GetWorld(), quadrants[1] * scl, scl * oneVec, FColor::Green, true, 1000);
    DrawDebugBox(GetWorld(), quadrants[2] * scl, scl * oneVec, FColor::Blue, true, 1000);
    DrawDebugBox(GetWorld(), quadrants[3] * scl, scl * oneVec, FColor::Purple, true, 1000);
    DrawDebugBox(GetWorld(), quadrants[4] * scl, scl * oneVec, FColor::Orange, true, 1000);
    DrawDebugBox(GetWorld(), quadrants[5] * scl, scl * oneVec, FColor::Yellow, true, 1000);
    DrawDebugBox(GetWorld(), quadrants[6] * scl, scl * oneVec, FColor::Magenta, true, 1000);
    DrawDebugBox(GetWorld(), quadrants[7] * scl, scl * oneVec, FColor::Cyan, true, 1000);
}

void AAdaptiveVoxelActor::ScheduleDataUpdate(float IntervalInSeconds)
{
    if (!DataUpdateIsRunning && !IsDestroyed)
    {
        DataUpdateIsRunning = true;

        FGraphEventRef Task = FFunctionGraphTask::CreateAndDispatchWhenReady([this, IntervalInSeconds]()
        {
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            {
                FRWScopeLock WriteLock(OctreeLock, SLT_Write);
                if (IsDestroyed) return;
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
    if (!MeshUpdateIsRunning && !IsDestroyed)
    {
        MeshUpdateIsRunning = true;
        FGraphEventRef Task = FFunctionGraphTask::CreateAndDispatchWhenReady([this, IntervalInSeconds]()
        {
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            //**********BEGIN IMPLEMENTATION BLOCK***************
            {
                FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
                if(IsDestroyed) return;
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