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
    ARealtimeMeshActor* ParentActor;
    FRWLock OctreeLock;
    TFunction<double(FVector, FVector)> DensityFunction;
    TSharedPtr<FAdaptiveOctreeNode> Root;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    TArray<TSharedPtr<FMeshChunk>> MeshChunks;
    int ChunkDepth;
    bool MeshChunksInitialized = false;
    double RootExtent;

public:
    // Constructor
    FAdaptiveOctree(TFunction<double(FVector, FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth);

    void InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial);

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    FVector QuantizePosition(const FVector& P, double GridSize = 1.0);
    
    FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);
    
    void UpdateLOD(FVector CameraPosition, double LodFactor);

    void UpdateMesh();

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetLeaves();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetChunks();

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);
    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(const FNodeEdge& Edge);
    TSharedPtr<FAdaptiveOctreeNode> FindNeighborLeafAtEdge(TSharedPtr<FAdaptiveOctreeNode> Node, int PerpendicularAxis, bool PositiveDirection, const FVector& ZeroCrossingPoint);
    void Clear(); //TODO: Need to test this
    //TODO: Need to make a destructor that safely disposes references/pointers and locks
};