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


    TFunction<double(FVector, FVector)> DensityFunction;
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
    
    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);
    
    void ReconstructAffectedNodes(TSharedPtr<FAdaptiveOctreeNode> InNode, FVector InEditCenter, double InReconstructRadius);

    static FVector QuantizePosition(const FVector& P, double GridSize = 1.0);
    
    static FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(const FNodeEdge& Edge);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);

    TSharedPtr<FAdaptiveOctreeNode> GetChunkNodeByPoint(FVector Position);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();

public:
    // Constructor
    FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial, TFunction<double(FVector, FVector)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    void PopulateChunks();
    
    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    void ReconstructSubtree(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius);

    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void UpdateMesh();

    void Clear(); //TODO: Need to test this

    void GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FNodeEdge>& OutEdges);

    //TODO: Need to make a destructor that safely disposes references/pointers and locks - LIFECYCLE MANAGEMENT, SPAWN/DESPAWN STABILITY AND OPTIMIZATION
};