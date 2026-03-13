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

    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;

    TWeakObjectPtr<UMaterialInterface> CachedOceanMaterial;

    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;

    bool MeshChunksInitialized = false;

    double RootExtent;

    double ChunkExtent;

    int ChunkDepth;

    void SplitToDepth(FAdaptiveOctreeNode* Node, int InMinDepth);

    void PopulateChunks();

    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    void FinalizeSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius);

    void ComputeNormalsFromMap(TArray<FCornerSample>& Samples, const TMap<FIntVector, int32>& CornerMap, FVector PlanetCenter);

    void ReconstructSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    static FVector QuantizePosition(const FVector& P, double GridSize = 1.0);

    static FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);

    FEdgeNeighbors SampleNodesAroundEdge(const FNodeEdge& Edge);

    FAdaptiveOctreeNode* GetLeafNodeByPoint(FVector Position);

    FAdaptiveOctreeNode* GetChunkNodeByPoint(FVector Position);

    void SplitAndComputeChildren(FAdaptiveOctreeNode* Node);

    void ComputeNodeData(FAdaptiveOctreeNode* Node);

public:
    FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InSurfaceMaterial, UMaterialInterface* InOceanMaterial, TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    void GatherUniqueCorners(FAdaptiveOctreeNode* Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& CornerMap, double QuantizeGrid, FVector EditCenter, double SearchRadius);

    void UpdateLodRecursive(FAdaptiveOctreeNode* Node, FVector CameraPosition, double InScreenSpaceThreshold, double InFOVScale, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged);

    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void UpdateMesh();

    void Clear();

    void GatherLeafEdges(FAdaptiveOctreeNode* Node, TArray<FNodeEdge>& OutEdges, TMap<FEdgeKey, int32>& EdgeMap);
};
