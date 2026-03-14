#pragma once

#include "CoreMinimal.h"
#include "FMeshingStructs.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctreeNode.h"

struct VOXELPLUGIN_API FOctreeParams {
    // --- Rendering ---
    ARealtimeMeshActor* ParentActor = nullptr;
    USceneComponent* MeshAttachmentRoot = nullptr;
    UMaterialInterface* SurfaceMaterial = nullptr;
    UMaterialInterface* OceanMaterial = nullptr;

    // --- Data Dependencies ---
    TFunction<void(int, const float*, const float*, const float*, float*)> NoiseFunction;
    TSharedPtr<FSparseEditStore> EditStore;

    // --- Spatial ---
    FVector Center = FVector::ZeroVector;

    // Planet radius: the minimum possible surface elevation.
    // Noise can only push the surface outward from this radius.
    double PlanetRadius = 1000.0;

    // Maximum displacement that noise can apply to the surface.
    // Surface elevation ranges from PlanetRadius to PlanetRadius + NoiseAmplitude.
    double NoiseAmplitude = 100.0;

    // Sea level as a coefficient of NoiseAmplitude.
    //  0.0 = sea level at PlanetRadius (no ocean)
    //  0.5 = sea level halfway through the noise range (~50/50 land/ocean)
    //  1.0 = sea level at PlanetRadius + NoiseAmplitude (fully submerged)
    // Values outside [0,1] are valid: -1 ensures no ocean, 2 gives deep ocean.
    double SeaLevelCoefficient = 0.5;

    // Buffer factor applied when computing root extent from PlanetRadius + NoiseAmplitude.
    // Ensures the octree bounds always contain all possible surface points.
    double RootExtentBuffer = 1.05;

    // --- Tree Structure ---
    int ChunkDepth = 4;
    int MinDepth = 7;
    int MaxDepth = 18;
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

    TWeakObjectPtr<USceneComponent> CachedMeshAttachRoot;

    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;

    TWeakObjectPtr<UMaterialInterface> CachedOceanMaterial;

    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;

    bool MeshChunksInitialized = false;

    double RootExtent;

    double ChunkExtent;

    int ChunkDepth;

    // Centralized terrain parameters -- derived from FOctreeParams at construction
    double PlanetRadius;    // Minimum surface radius (noise only adds elevation)
    double OceanRadius;     // Sea level radius, derived from SeaLevelCoefficient
    double NoiseAmplitude;  // Maximum noise displacement above PlanetRadius

    // Composite density sample: combines sphere SDF, noise height, and edit store.
    // Dist: distance from planet center to the sample point.
    // NoiseHeight: raw noise output in [-1, 1] range.
    // Position: world-space position for edit store lookup.
    //
    // Noise is remapped from [-1,1] to [0,1] so that the surface ranges from
    // PlanetRadius (noise=-1) to PlanetRadius+NoiseAmplitude (noise=+1).
    // This guarantees PlanetRadius is the minimum possible surface elevation.
    float ComputeDensity(double Dist, float NoiseHeight, FVector Position) const
    {
        double Remapped = ((double)NoiseHeight + 1.0) * 0.5; // [-1,1] -> [0,1]
        double Height = Remapped * NoiseAmplitude;
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
        double InvNoiseAmplitude = 1.0 / NoiseAmplitude;
        OutX = (float)(SurfacePos.X * InvNoiseAmplitude);
        OutY = (float)(SurfacePos.Y * InvNoiseAmplitude);
        OutZ = (float)(SurfacePos.Z * InvNoiseAmplitude);
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
    explicit FAdaptiveOctree(const FOctreeParams& Params);

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
    double GetNoiseAmplitude() const { return NoiseAmplitude; }
    FVector GetPlanetCenter() const { return Root.IsValid() ? Root->Center : FVector::ZeroVector; }
};