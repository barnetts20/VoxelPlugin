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
    TFunction<double(FVector)> DensityFunction;
    TSharedPtr<FAdaptiveOctreeNode> Root;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    TArray<TSharedPtr<FMeshChunk>> MeshChunks;
    int ChunkDepth = 4;
    UMaterialInterface* Material;
    TMap<FAdaptiveOctreeNode*, int32> ChunkNodeToIndex;
    bool MeshChunksInitialized = false;
    double RootExtent;

public:
    // Constructor
    FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    void InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial);

    bool HasReachedChunkDepth();

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    uint32 ComputePositionHash(const FVector& Position, float GridSize);

    void UpdateLOD(FVector CameraPosition, double LodFactor);
    void UpdateMesh();

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetLeaves();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetChunks();

    FVector CalculateSurfaceNormal(const FVector& Position);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetNodesAtDepth(int Depth);
    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(const FNodeEdge& Edge);
    void Clear(); //TODO: Need to test this
    //TODO: Need to make a destructor that safely disposes references/pointers and locks
};