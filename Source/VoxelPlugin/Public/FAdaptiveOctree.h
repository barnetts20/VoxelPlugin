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
    
    TSparseArray<FCornerData> CornerPool;
    TMap<FIntVector, int32> CornerMap;

    TSparseArray<FOctreeEdge> EdgePool;
    TMap<FEdgeKey, int32> EdgeLookupMap;
    // Thread safety for concurrent LOD splits
    FCriticalSection CornerPoolLock;
    FCriticalSection EdgePoolLock;

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

    void DebugDrawEdgeData(const FEdgeVertexData& EdgeData, const FVector& chunkCenter);

    static FVector QuantizePosition(const FVector& P, double GridSize = 1.0);
    
    static FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(int32 EdgeIndex);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);

    TSharedPtr<FAdaptiveOctreeNode> GetChunkNodeByPoint(FVector Position);

    void GatherLeafEdges(
        TSharedPtr<FAdaptiveOctreeNode> Node,
        TArray<int32>& OutEdges
    );

    void ComputeNodeData(TSharedPtr<FAdaptiveOctreeNode> Node);

public:

    FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InSurfaceMaterial, UMaterialInterface* InOceanMaterial, TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    void GatherUniqueCorners(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& InCornerMap, double QuantizeGrid, FVector EditCenter, double SearchRadius);

    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);
    
    int32 AcquireCorner(const FVector& InPos);
    void ReleaseCorner(int32 InCornerIndex);

    int32 AcquireEdge(int32 C0, int32 C1, TSharedPtr<FAdaptiveOctreeNode> RequestingNode);
    void ReleaseEdge(int32 InEdgeIndex);

    void SortNodesCircular(TArray<TSharedPtr<FAdaptiveOctreeNode>, TInlineAllocator<4>>& Nodes, int32 Axis);
    void RegisterNodeEdges(TSharedPtr<FAdaptiveOctreeNode> Node);
    
    void SetCornerDensity(int32 Index, double NewDensity);
    void SetCornerNormal(int32 Index, const FVector& NewNormal);

    // In FAdaptiveOctree.h
    const FCornerData& GetCornerData(int32 Index) const
    {
        // Basic safety check for debugging
        check(CornerPool.IsValidIndex(Index));
        return CornerPool[Index];
    }

    const FOctreeEdge& GetEdgeData(int32 Index) const
    {
        // Basic safety check for debugging
        check(EdgePool.IsValidIndex(Index));
        return EdgePool[Index];
    }

    void UpdateMesh();

    void Clear();
    void UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, bool& OutChanged);
};


