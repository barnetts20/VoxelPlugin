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

    TSharedPtr<FNodeStructureProvider> StructureProvider;

    void GatherNodesInSphere(TSharedPtr<FAdaptiveOctreeNode> Node, const FVector& Center, double Radius, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNodes);

    TSharedPtr<FAdaptiveOctreeNode> Root;

    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;

    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;

    TWeakObjectPtr<UMaterialInterface> CachedOceanMaterial;

    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;

    bool MeshChunksInitialized = false;

    double RootExtent;

    double ChunkExtent;

    int ChunkDepth;

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNewNodes);

    void PopulateChunks();

    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    static FVector QuantizePosition(const FVector& P, double GridSize = 1.0);

    static FVector2f ComputeTriplanarUV(FVector Position, FVector3f Normal);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);

    TSharedPtr<FAdaptiveOctreeNode> GetChunkNodeByPoint(FVector Position);

    void GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FVoxelEdge*>& OutEdges);

    void SampleNodesAroundEdge(FVoxelEdge* Edge, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNodes);

    void UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNewNodes, bool& OutChanged);

public:
    FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InSurfaceMaterial, UMaterialInterface* InOceanMaterial, TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void UpdateMesh();

    void Clear();
    void UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNewNodes, bool& OutChanged);
};