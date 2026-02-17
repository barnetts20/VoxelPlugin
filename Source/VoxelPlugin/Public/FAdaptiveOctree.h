#pragma once

#include "CoreMinimal.h"
#include <FMeshingStructs.h>
#include "FAdaptiveOctreeNode.h"
/**
 * Adaptive Octree for LOD-based voxel meshing.
 */
inline const FVector Directions[6] =
{
    FVector(1, 0, 0),
    FVector(-1, 0, 0),
    FVector(0, 1, 0),
    FVector(0, -1, 0),
    FVector(0, 0, 1),
    FVector(0, 0, -1)
};

struct VOXELPLUGIN_API FAdaptiveOctree
{
private:
    TFunction<double(FVector, FVector)> DensityFunction;
    TSharedPtr<FAdaptiveOctreeNode> Root;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    TArray<TSharedPtr<FMeshChunk>> MeshChunks;
    int ChunkDepth;
    bool MeshChunksInitialized = false;
    double RootExtent;

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);
    
    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);
    
    static FVector QuantizePosition(const FVector& P, double GridSize = 1.0);
    
    static FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(const FNodeEdge& Edge);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();

public:
    // Constructor
    FAdaptiveOctree(TFunction<double(FVector, FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth);

    void InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial);
    
    void UpdateLOD(FVector CameraPosition, double LodFactor);

    void UpdateMesh();

    void Clear(); //TODO: Need to test this
    //TODO: Need to make a destructor that safely disposes references/pointers and locks
};