#pragma once

#include "CoreMinimal.h"
#include "FMeshingStructs.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctreeNode.h"
#include "FCornerProvider.h"

struct VOXELPLUGIN_API FAdaptiveOctree
{
private:
    static inline const FVector Directions[6] =
    {
        FVector(1, 0, 0),  FVector(-1, 0, 0),
        FVector(0, 1, 0),  FVector(0, -1, 0),
        FVector(0, 0, 1),  FVector(0, 0, -1)
    };

    TSharedPtr<FAdaptiveOctreeNode> Root;
    TSharedPtr<FCornerProvider> CornerProvider;

    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;
    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;
    TWeakObjectPtr<UMaterialInterface> CachedOceanMaterial;

    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;

    bool MeshChunksInitialized = false;

    double RootExtent;
    double ChunkExtent;

    void SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth);

    void PopulateChunks();

    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    void GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FNodeEdge>& OutEdges);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> SampleNodesAroundEdge(const FNodeEdge& Edge);

    TSharedPtr<FAdaptiveOctreeNode> GetLeafNodeByPoint(FVector Position);

    TSharedPtr<FAdaptiveOctreeNode> GetChunkNodeByPoint(FVector Position);

    static FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);



public:
    FAdaptiveOctree(
        ARealtimeMeshActor* InParentActor,
        UMaterialInterface* InSurfaceMaterial,
        UMaterialInterface* InOceanMaterial,
        TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction,
        TSharedPtr<FSparseEditStore> InEditStore,
        FVector InCenter,
        double InRootExtent,
        int InChunkDepth,
        int InMinDepth,
        int InMaxDepth);

    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, bool& OutChanged);

    void UpdateMesh();

    void CleanupData();

    void Clear();
};