#pragma once

#include "CoreMinimal.h"
#include "FMeshingStructs.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctreeNode.h"

//TODO: COMPLETE PARAM STRUCT, CHANGE CONSTRUCTOR
struct VOXELPLUGIN_API FOctreeParams {
    ARealtimeMeshActor* InParentActor; 
    UMaterialInterface* InSurfaceMaterial; 
    UMaterialInterface* InOceanMaterial; 
    TFunction<void(int, const float*, const float*, const float*, float*)> NoiseFunction;
    TSharedPtr<FSparseEditStore> InEditStore; 
    FVector InCenter; 

    double PlanetRadius; //min surface radius in real world units
    double InRootExtent = 1.2; //compute a buffer larger than planet radius, dont directly expose
    double SeaLevel; // should be relative to planet radius 0 at planet radius
    double NoiseScale;

    int InChunkDepth; 
    int InMinDepth; 
    int InMaxDepth;
};

/**
 * Adaptive Octree for LOD-based voxel meshing.
 */

struct VOXELPLUGIN_API FAdaptiveOctree
{
private:
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

    // Centralized terrain parameters -- derived from RootExtent at construction
    double PlanetRadius;    // RootExtent * 0.9 -- base sphere surface
    double OceanRadius;     // Sea level radius (initially == PlanetRadius)
    double NoiseScale;      // RootExtent * 0.1 -- noise displacement amplitude

    // Composite density sample: combines sphere SDF, noise height, and edit store.
    // Dist: distance from planet center to the sample point.
    // NoiseHeight: raw noise output for this point.
    // Position: world-space position for edit store lookup.
    float ComputeDensity(double Dist, float NoiseHeight, FVector Position) const
    {
        double Height = (double)NoiseHeight * NoiseScale;
        return (float)(Dist - (PlanetRadius + Height) + EditStore->Sample(Position));
    }

    // Compute noise sampling coordinates for a world-space position.
    // Projects onto the unit sphere surface and scales by noise frequency.
    void ComputeNoisePosition(FVector WorldPos, float& OutX, float& OutY, float& OutZ) const
    {
        FVector PlanetRel = WorldPos - Root->Center;
        double Dist = PlanetRel.Size();
        FVector Dir = (Dist > 1e-10) ? (PlanetRel / Dist) : FVector::UpVector;
        FVector SurfacePos = Dir * RootExtent;
        double InvNoiseScale = 1.0 / NoiseScale;
        OutX = (float)(SurfacePos.X * InvNoiseScale);
        OutY = (float)(SurfacePos.Y * InvNoiseScale);
        OutZ = (float)(SurfacePos.Z * InvNoiseScale);
    }

    void SplitToDepth(FAdaptiveOctreeNode* Node, int InMinDepth);

    void PopulateChunks();

    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    void FinalizeSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius);

    void ReconstructSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius);

    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

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

    // Public accessors for shader/atmosphere/rendering use
    double GetPlanetRadius() const { return PlanetRadius; }
    double GetOceanRadius() const { return OceanRadius; }
    double GetNoiseScale() const { return NoiseScale; }
    FVector GetPlanetCenter() const { return Root.IsValid() ? Root->Center : FVector::ZeroVector; }
};