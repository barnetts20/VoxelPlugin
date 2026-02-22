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
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> MeshChunks;

    bool MeshChunksInitialized = false;
    double RootExtent;
    double ChunkExtent;

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);
    
    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);
    
    static FVector QuantizePosition(const FVector& P, double GridSize = 1.0);
    
    static FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(const FNodeEdge& Edge);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();

public:
    int32 ChunkDepth = 0;
    ARealtimeMeshActor* CachedParentActor = nullptr;
    UMaterialInterface* CachedMaterial = nullptr;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> PendingNewChunks;

    void ApplyEdit(FVector BrushCenter, double BrushRadius, double Strength);
    void UpdateAffectedChunks(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius);
    void GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FNodeEdge>& OutEdges);
    TSharedPtr<FAdaptiveOctreeNode> GetNodeByPointAtDepth(FVector Position, int TargetDepth);
    bool HasSurfaceNeighbor(TSharedPtr<FAdaptiveOctreeNode> Node);
    void CreatePendingMeshChunks();

    // Constructor
    FAdaptiveOctree(TFunction<double(FVector, FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    void TestEdit(FVector Center);

    void ReconstructAffectedLeaves(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius);

    void InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial);
    
    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void UpdateMesh();

    void Clear();
};

inline uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}