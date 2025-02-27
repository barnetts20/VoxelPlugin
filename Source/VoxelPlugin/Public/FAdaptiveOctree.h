#pragma once
struct FMeshChunk;

#include "CoreMinimal.h"
#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"

/**
 * Adaptive Octree for LOD-based voxel meshing.
 */
struct VOXELPLUGIN_API FAdaptiveOctree : public TSharedFromThis<FAdaptiveOctree>
{
private:
    FRWLock OctreeLock;
    TFunction<double(FVector)> DensityFunction;
    TSharedPtr<FAdaptiveOctreeNode> Root;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> ChunkNodes; // The actual data nodes
    bool ChunksInitialized = false;
    TArray<FMeshChunk> Chunks; // Chunk data and mesh wrapper
    double RootExtent;

public:
    // Constructor
    FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth);

    void InitializeChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial);

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth); //Write Lock
    bool UpdateLOD(FVector CameraPosition, double LodFactor); //Write Lock
    void UpdateMesh();
    void Clear(); //Write Lock

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetLeaves(); //Read Lock
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes(); //Read Lock
    TArray<FMeshChunk> GetChunks(); //Read Lock
    FAdaptiveOctreeFlatNode GetSurfaceNodeByPoint(FVector Position); //Read Lock
    TArray<FAdaptiveOctreeFlatNode> SampleSurfaceNodesAroundEdge(const FNodeEdge& Edge); //Read Lock

    //TODO: Need to make a destructor that safely disposes references/pointers and locks
};