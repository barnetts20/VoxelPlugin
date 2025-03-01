#include "AdaptiveVoxelActorMT.h"
#include "DrawDebugHelpers.h"
#include "Kismet/GameplayStatics.h"
#include <FVoxelStructure.h>

using namespace RealtimeMesh;

// Sets default values
AAdaptiveVoxelActorMT::AAdaptiveVoxelActorMT()
{
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.bStartWithTickEnabled = true;

    CameraData.Position = FVector::ZeroVector;
    CameraData.Direction = FVector(1, 0, 0);
    CameraData.FOV = 90;

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
    int ChunkMinMaxDepth[3] = { ChunkDepth, MinDepth, MaxDepth };
    AdaptiveOctree = MakeShared<FAdaptiveOctreeMT>(DensityFunction, GetActorLocation(), 15001357.0, ChunkMinMaxDepth);
    //Sparsetree for user edits
    SparseOctree = MakeShared<FSparseOctree>();
}

void AAdaptiveVoxelActorMT::BeginDestroy() {
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Super::BeginDestroy();
}

void AAdaptiveVoxelActorMT::OnConstruction(const FTransform& Transform) {
    Super::OnConstruction(Transform);
    // Ensure we are not in a preview world (prevents running in editor mode)
    if (GetWorld() && !GetWorld()->IsPreviewWorld() && TickInEditor)
    {
        InitializeChunks();
    }
}

void AAdaptiveVoxelActorMT::BeginPlay()
{
    Super::BeginPlay();
    InitializeChunks();
}

void AAdaptiveVoxelActorMT::CleanSceneRoot() {
    auto destroyComponentArray = GetRootComponent()->GetAttachChildren();
    for (TObjectPtr<USceneComponent> child : destroyComponentArray) {
        URealtimeMeshComponent* meshComponent = Cast<URealtimeMeshComponent>(child);
        if (meshComponent) {
            meshComponent->DestroyComponent();
        }
    }
}

void AAdaptiveVoxelActorMT::InitializeChunks() {
    CleanSceneRoot();
    AdaptiveOctree->InitializeChunks(this, Material);

    Initialized = true;
    ScheduleDataUpdate(.1);
    //ScheduleMeshUpdate(.1);
}

void AAdaptiveVoxelActorMT::ScheduleDataUpdate(float IntervalInSeconds)
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
                    //FRWScopeLock WriteLock(OctreeLock, SLT_Write);
                    AdaptiveOctree->UpdateLOD(CameraData, LodFactor);
                }
                //{
                //    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
                //    //FCriticalSection CriticalSection;
                //    //ChunksToUpdate.Empty();
                //    //ChunksToUpdate.Reserve(Chunks.Num());
                //    ParallelFor(Chunks.Num(), [&](int32 idx)
                //    {
                //        if (Chunks[idx]->UpdateData(CameraPosition, LodFactor)) {
                //            //FScopeLock Lock(&CriticalSection);
                //            //ChunksToUpdate.Add(Chunks[idx]);
                //        }
                //    });
                //}
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
//void AAdaptiveVoxelActorMT::ScheduleMeshUpdate(float IntervalInSeconds)
//{
//    if (!MeshUpdateIsRunning)
//    {
//        MeshUpdateIsRunning = true;
//        FGraphEventRef Task = FFunctionGraphTask::CreateAndDispatchWhenReady([this, IntervalInSeconds]()
//            {
//                //**********BEGIN IMPLEMENTATION BLOCK***************
//                //**********BEGIN IMPLEMENTATION BLOCK***************
//                //**********BEGIN IMPLEMENTATION BLOCK***************
//                {
//                    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
//                    AdaptiveOctree->();
//                }
//                //{
//                //    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
//                //    ParallelFor(Chunks.Num(), [&](int32 idx)
//                //    {
//                //        //if (ChunksToUpdate[idx].IsValid()) {
//                //            Chunks[idx]->UpdateMeshData();
//                //            Chunks[idx]->UpdateComponent();
//                //        //}
//                //    });
//                //    //ChunksToUpdate.Empty();
//                //}
//                //***********END IMPLEMENTATION BLOCK***************
//                //***********END IMPLEMENTATION BLOCK***************
//                //***********END IMPLEMENTATION BLOCK***************
//
//                MeshUpdateIsRunning = false;
//                FTSTicker::GetCoreTicker().AddTicker(FTickerDelegate::CreateLambda([this, IntervalInSeconds](float DeltaTime)
//                    {
//                        ScheduleMeshUpdate(IntervalInSeconds);
//                        return false;
//                    }), IntervalInSeconds);
//            }, TStatId(), nullptr, ENamedThreads::AnyBackgroundThreadNormalTask);
//    }
//}

bool AAdaptiveVoxelActorMT::ShouldTickIfViewportsOnly() const
{
    return TickInEditor;
}

void AAdaptiveVoxelActorMT::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);
    if (!Initialized) return;

    //Cache cam data
    auto world = GetWorld();
    if (world != nullptr) {
        auto viewLocations = world->ViewLocationsRenderedLastFrame;
        if (viewLocations.Num() > 0) {
            this->CameraData.Position = viewLocations[0];
        }
    }
}

TSharedPtr<FAdaptiveOctreeMT> AAdaptiveVoxelActorMT::GetOctree()
{
    return AdaptiveOctree;
}

// Draws the Dual Contouring Points of Surface Nodes
//void AAdaptiveVoxelActor::DrawDebugSurfaceNodes()
//{
//    const bool DrawNodeBounds = true;
//    const bool DrawEdgeCrossings = false;
//    const bool DrawNormalVector = false;
//    const bool DrawDualContourPoint = false;
//    if (!AdaptiveOctree) return;
//
//    for (TSharedPtr<FAdaptiveOctreeNode> Node : CachedSurfaceNodes)
//    {
//        if (!Node.IsValid()) continue;
//        if(DrawNodeBounds) DrawDebugBox(GetWorld(), Node->Center, FVector(Node->Extent), FColor::Blue, false, .3f);
//        if(DrawEdgeCrossings) {
//            for (FNodeEdge& anEdge : Node->Edges) {
//                if (anEdge.SignChange) {
//                    DrawDebugPoint(GetWorld(), anEdge.Corners[0].Position, 8.0f, anEdge.Corners[0].Density > 0 ? FColor::Green : FColor::Red, false, .3f);
//                    DrawDebugPoint(GetWorld(), anEdge.Corners[1].Position, 8.0f, anEdge.Corners[1].Density > 0 ? FColor::Green : FColor::Red, false, .3f);
//                    DrawDebugPoint(GetWorld(), anEdge.ZeroCrossingPoint, 5.0f, FColor::Green, false, .3f);
//                }
//            }
//        }
//        if(DrawNormalVector) DrawDebugLine(GetWorld(), Node->DualContourPosition, Node->DualContourPosition + (Node->DualContourNormal * Node->Extent * .5), FColor(Node->DualContourNormal.X * 255, Node->DualContourNormal.Y * 255, Node->DualContourNormal.Z * 255), false, .3f);
//        if(DrawDualContourPoint) DrawDebugPoint(GetWorld(), Node->DualContourPosition, 10.0f, FColor::Purple, false, .3f);
//    }
//}