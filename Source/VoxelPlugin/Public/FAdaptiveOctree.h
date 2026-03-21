#pragma once

#include "CoreMinimal.h"
#include "FMeshingStructs.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctreeNode.h"
#include <FDensitySampleCompositor.h>

struct VOXELPLUGIN_API FOctreeParams {
    // --- Rendering ---
    ARealtimeMeshActor* ParentActor = nullptr;
    USceneComponent* MeshAttachmentRoot = nullptr;
    UMaterialInterface* SurfaceMaterial = nullptr;

    // --- Data Dependencies ---
    TSharedPtr<FDensitySampleCompositor> Compositor;

    // Planet radius: the minimum possible surface elevation.
    // Noise can only push the surface outward from this radius.
    double PlanetRadius = 1000.0;

    // Maximum displacement that noise can apply to the surface.
    // Surface elevation ranges from PlanetRadius to PlanetRadius + NoiseAmplitude.
    double NoiseAmplitude = 100.0;

    // Buffer factor applied when computing root extent from PlanetRadius + NoiseAmplitude.
    // Ensures the octree bounds always contain all possible surface points.
    double RootExtentBuffer = 1.05;

    // --- Tree Structure ---
    int ChunkDepth = 4;
    int MinDepth = 7;
    int MaxDepth = 21;

    // Depth beyond which noise sampling is replaced by trilinear interpolation
    // from parent corner densities. The noise loses precision past this depth,
    // but deeper splits still provide geometric detail for editing.
    int PrecisionDepthFloor = 21;
};

/**
 * Adaptive Octree for LOD-based voxel meshing.
 */

struct VOXELPLUGIN_API FAdaptiveOctree
{
private:
    TSharedPtr<FDensitySampleCompositor> Compositor;

    TSharedPtr<FAdaptiveOctreeNode> Root;

    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;

    TWeakObjectPtr<USceneComponent> CachedMeshAttachRoot;

    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;

    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;

    bool MeshChunksInitialized = false;

    double RootExtent;

    double ChunkExtent;

    int ChunkDepth;

    // Beyond this depth, noise sampling is replaced by interpolation from parent densities.
    int PrecisionDepthFloor;

    // Centralized terrain parameters -- derived from FOctreeParams at construction
    double PlanetRadius;    // Minimum surface radius (noise only adds elevation)
    double NoiseAmplitude;  // Maximum noise displacement above PlanetRadius

    // Triplanar UV scale: produces consistent UV tiling regardless of tree scale.
    // Calibrated so that ~8000 UV wraps occur across the planet diameter.
    double TriplanarUVScale;

    void SplitToDepth(FAdaptiveOctreeNode* Node, int InMinDepth);

    void PopulateChunks();

    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    void FinalizeSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius);

    void ReconstructSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    FVector2f ComputeTriplanarUV(FVector Position, FVector Normal) const;

    FEdgeNeighbors SampleNodesAroundEdge(const FNodeEdge& Edge);

    FAdaptiveOctreeNode* GetLeafNodeByPoint(FVector Position);

    FAdaptiveOctreeNode* GetChunkNodeByPoint(FVector Position);

    void SplitAndComputeChildren(FAdaptiveOctreeNode* Node);

    void ComputeNodeData(FAdaptiveOctreeNode* Node);

public:
    explicit FAdaptiveOctree(const FOctreeParams& Params);

    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    void GatherUniqueCorners(FAdaptiveOctreeNode* Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& CornerMap, double QuantizeGrid, FVector EditCenter, double SearchRadius);

    void UpdateLodRecursive(FAdaptiveOctreeNode* Node, FVector CameraPosition, double ThresholdSq, double MergeThresholdSq, double InFOVScale, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged);

    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void UpdateMesh();

    void Clear();

    void GatherLeafEdges(FAdaptiveOctreeNode* Node, TArray<FNodeEdge>& OutEdges, TMap<FEdgeKey, int32>& EdgeMap);

    // Public accessors for shader/atmosphere/rendering use
    double GetPlanetRadius() const { return PlanetRadius; }
    double GetNoiseAmplitude() const { return NoiseAmplitude; }
    FVector GetPlanetCenter() const { return Root.IsValid() ? Root->Center : FVector::ZeroVector; }
};