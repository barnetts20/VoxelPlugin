#pragma once

#include "CoreMinimal.h"
#include "FMeshingStructs.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctreeNode.h"

/**
 * Adaptive Octree for LOD-based voxel meshing.
 */

struct VOXELPLUGIN_API FAdaptiveOctree
{
private:
    static inline const FVector Directions[6] =
    {
        FVector(1, 0, 0),
        FVector(-1, 0, 0),
        FVector(0, 1, 0),
        FVector(0, -1, 0),
        FVector(0, 0, 1),
        FVector(0, 0, -1)
    };

    TFunction<void(int, const float*, const float*, const float*, float*)> DensityFunction;

    TSharedPtr<FSparseEditStore> EditStore;

    TSharedPtr<FAdaptiveOctreeNode> Root;

    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;

    TWeakObjectPtr<UMaterialInterface> CachedMaterial;

    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;

    bool MeshChunksInitialized = false;

    double RootExtent;

    double ChunkExtent;

    int ChunkDepth;

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);

    void PopulateChunks();

    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    void ProcessChunkData(TSharedPtr<FAdaptiveOctreeNode> ChunkNode);

    void FinalizeSubtree(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius);

    void ComputeNormalsFromMap(TArray<FCornerSample>& Samples, const TMap<FIntVector, int32>& CornerMap, FVector PlanetCenter);

    void ReconstructSubtree(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    static FVector QuantizePosition(const FVector& P, double GridSize = 1.0);
    
    static FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(const FNodeEdge& Edge);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);

    TSharedPtr<FAdaptiveOctreeNode> GetChunkNodeByPoint(FVector Position);

    void GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FNodeEdge>& OutEdges);

    void ComputeNodeData(TSharedPtr<FAdaptiveOctreeNode> Node);

public:
    FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial, TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    void GatherUniqueCorners(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& CornerMap, double QuantizeGrid, FVector EditCenter, double SearchRadius);

    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void UpdateMesh();

    void Clear();
    void UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged);
};