#pragma once

#include "CoreMinimal.h"
#include <FMeshingStructs.h>
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
    bool MeshChunksInitialized = false;
    TArray<FMeshChunk> MeshChunks;
    double RootExtent;

public:
    // Constructor
    FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth);

    void InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial);

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);

    void UpdateMeshChunkStreamData(FMeshChunk& InChunk);

    bool UpdateLOD(FVector CameraPosition, double LodFactor);
    void UpdateMesh();

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetLeaves();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetChunks();

    FAdaptiveOctreeFlatNode GetSurfaceNodeByPoint(FVector Position);
    TArray<FAdaptiveOctreeFlatNode> SampleSurfaceNodesAroundEdge(const FNodeEdge& Edge);
    void Clear(); //TODO: Need to test this
    //TODO: Need to make a destructor that safely disposes references/pointers and locks
};