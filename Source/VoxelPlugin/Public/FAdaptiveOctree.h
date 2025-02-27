#pragma once

#include "CoreMinimal.h"
#include "FAdaptiveOctreeNode.h"

/**
 * Adaptive Octree for LOD-based voxel meshing.
 */
struct VOXELPLUGIN_API FAdaptiveOctree
{
private:
    FRWLock OctreeLock;
    TFunction<double(FVector)> DensityFunction;
    TSharedPtr<FAdaptiveOctreeNode> Root;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    double RootExtent;

public:
    // Constructor
    FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth);

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);

    bool UpdateLOD(FVector CameraPosition, double LodFactor);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetLeaves();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetChunks();

    FAdaptiveOctreeFlatNode GetSurfaceNodeByPoint(FVector Position);
    TArray<FAdaptiveOctreeFlatNode> SampleSurfaceNodesAroundEdge(const FNodeEdge& Edge);
    void Clear(); //TODO: Need to test this
    //TODO: Need to make a destructor that safely disposes references/pointers and locks
};