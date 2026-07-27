// FAdaptiveOctree.h — The adaptive octree: manages tree construction, LOD-driven
// split/merge, density evaluation, edit application, and dual-contour mesh generation.
//
// Lifecycle:
//   1. Construction: builds tree to ChunkDepth, samples density, populates mesh chunks.
//   2. Per-frame: UpdateLOD splits/merges leaves by screen-space size, UpdateMesh pushes
//      dirty chunks to RealtimeMesh components on the game thread.
//   3. Edits: ApplyEdit writes to the sparse edit store, reconstructs affected subtrees,
//      enforces LOD splits for newly-surfaced nodes, and rebuilds chunk meshes.

#pragma once

#include "CoreMinimal.h"
#include "FMeshingStructs.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctreeNode.h"
#include "FDensitySampleCompositor.h"

/** Construction parameters for FAdaptiveOctree. Aggregates rendering targets,
 *  density compositor, terrain geometry parameters, and tree depth bounds. */
struct VOXELPLUGIN_API FOctreeParams {
    // --- Rendering ---
    ARealtimeMeshActor* ParentActor = nullptr;
    USceneComponent* MeshAttachmentRoot = nullptr;
    UMaterialInterface* SurfaceMaterial = nullptr;

    // --- Data Dependencies ---
    TSharedPtr<FDensitySampleCompositor> Compositor;

    /** Planet radius: the minimum possible surface elevation.
     *  Noise can only push the surface outward from this radius. */
    double PlanetRadius = 1000.0;

    /** Maximum displacement that noise can apply to the surface.
     *  Surface elevation ranges from PlanetRadius to PlanetRadius + NoiseAmplitude. */
    double NoiseAmplitude = 100.0;

    /** Buffer factor applied when computing root extent from PlanetRadius + NoiseAmplitude.
     *  Ensures the octree bounds always contain all possible surface points. */
    double RootExtentBuffer = 1.05;

    // --- Tree Structure ---

    /** Depth of the (currently uniform) chunk cut: the tree is split to here at build and
     *  every node at this depth becomes a chunk root. Also the floor for the future
     *  variable-depth chunk cut. */
    int ChunkDepth = 4;

    /** Deepest a chunk root may sit — the near-camera precision ceiling for the
     *  chunk-cut pass. ChunkDepth is the shallow floor the tree is built at; chunks
     *  promote toward this as the camera approaches. */
    int MaxChunkDepth = 10;

    /** Screen-space tolerance for chunk float-precision jitter (fraction of the view).
     *  A chunk promotes to finer children when its worst-case FVector3f jitter
     *  (Extent * FLT_EPSILON) would project larger than this at the camera's near
     *  distance. Smaller = promote sooner (finer origins, more components). */
    double ChunkPrecisionThreshold = 5e-4;

    /** Demote hysteresis: a promoted chunk only collapses back once its parent would be
     *  precise to ChunkPrecisionThreshold * this (< 1 => parent must be comfortably below
     *  the promote trigger). Combined with the 2x parent/child extent ratio this gives a
     *  stable dead zone so a chunk on a distance boundary doesn't create/destroy every
     *  frame. */
    double ChunkDemoteHysteresis = 0.5;

    int MinDepth = 8;
    int MaxDepth = 22;

    /** Depth beyond which noise sampling is replaced by trilinear interpolation
     *  from parent corner densities. Noise loses float precision past this depth,
     *  but deeper splits still provide geometric detail for editing. */
    int PrecisionDepthFloor = 19;
};

/** Adaptive octree for LOD-driven dual-contour voxel meshing.
 *
 *  Manages a hierarchy of FAdaptiveOctreeNode from a single root down to leaf nodes
 *  that produce mesh geometry. The tree is spatially partitioned at ChunkDepth into
 *  mesh chunks, each backed by a RealtimeMesh component. Below ChunkDepth, leaves
 *  are split and merged per-frame based on screen-space projected size.
 *
 *  Density is evaluated through an FDensitySampleCompositor (noise layers + edit store).
 *  Edits trigger a localized reconstruction pipeline: re-sample affected corners,
 *  propagate densities past the precision floor, recompute normals/edges/QEF, enforce
 *  LOD splits for newly-surfaced nodes, and rebuild mesh streams for dirty chunks. */
struct VOXELPLUGIN_API FAdaptiveOctree
{
private:
    // --- Core State ---
    TSharedPtr<FDensitySampleCompositor> Compositor;
    TSharedPtr<FAdaptiveOctreeNode> Root;

    // --- Rendering References (cached from FOctreeParams) ---
    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;
    TWeakObjectPtr<USceneComponent> CachedMeshAttachRoot;
    TWeakObjectPtr<UMaterialInterface> CachedSurfaceMaterial;

    // --- Chunk Management ---

    /** Maps chunk-depth nodes to their mesh chunk. Keyed by shared pointer so chunk
     *  identity is tied to the node's lifetime. */
    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;
    bool MeshChunksInitialized = false;

    // --- Tree Geometry ---
    double RootExtent;
    double ChunkExtent;
    int ChunkDepth;
    int MaxChunkDepth;
    double ChunkPrecisionThreshold;
    double ChunkDemoteHysteresis;
    int PrecisionDepthFloor;

    // --- Terrain Parameters (derived from FOctreeParams) ---
    double PlanetRadius;
    double NoiseAmplitude;

    /** Triplanar UV scale calibrated to produce consistent tiling across planet scales.
     *  Derived at construction: 0.0001 * (80M / PlanetRadius). */
    double TriplanarUVScale;

    // --- Cached LOD State ---
    // Saved from the last UpdateLOD pass so the edit pipeline can split newly-surfaced
    // nodes at the correct LOD depth without a redundant LOD evaluation.
    FVector CachedCameraPosition = FVector::ZeroVector;
    double CachedFOVScale = 1.0;
    double CachedThresholdSq = 0.004;

    // =====================================================================
    // Construction & Initial Build
    // =====================================================================

    /** Recursively splits the tree from root to InMinDepth (ChunkDepth), computing
     *  density at each level. Only recurses into children where CouldContainSurface
     *  is true, skipping deep interior/exterior nodes. */
    void SplitToDepth(FAdaptiveOctreeNode* Node, int InMinDepth);

    /** After SplitToDepth, builds mesh chunks for all relevant chunk-depth nodes
     *  and populates the ChunkMap. Runs edge gathering and mesh stream building in parallel. */
    void PopulateChunks();

    /** Recursively collects all chunk-depth nodes where CouldContainSurface is true.
     *  This single walk replaces both the old Surface-mode GetSurfaceChunks + neighbor
     *  buffer probing and the Volume-mode CollectVolumeChunks sphere-AABB test. */
    void CollectNearSurfaceChunks(FAdaptiveOctreeNode* Node, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out);

    // =====================================================================
    // Edit Pipeline
    // =====================================================================

    /** Updates the ChunkMap after an edit: marks existing surface chunks dirty,
     *  creates new chunks for newly-surfaced nodes, and adds face-adjacent neighbor
     *  buffer chunks for seamless cross-chunk meshing. */
    void UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks);

    /** Full reconstruction of an edited subtree. Gathers unique corners, re-samples
     *  density in one bulk compositor call, propagates deep densities past the precision
     *  floor, and finalizes all nodes (normals, edges, QEF, surface flags). */
    void ReconstructSubtree(FAdaptiveOctreeNode* Node);

    /** Post-reconstruction pass: walks edited leaves and splits any that are now
     *  CouldContainSurface but weren't previously split. Uses cached LOD state. */
    void EnforceSplitsInSubtree(FAdaptiveOctreeNode* Node);

    /** Bottom-up finalization: recomputes normals, sign-change edges, dual contour
     *  position, and surface flags for all edited nodes in a subtree. */
    void FinalizeSubtree(FAdaptiveOctreeNode* Node);

    /** Top-down density propagation for nodes at or beyond PrecisionDepthFloor.
     *  Parent corners have fresh noise+edit data; children get noise interpolated
     *  from the 27-point grid with edit deltas re-sampled at full resolution. */
    void PropagateDeepDensities(FAdaptiveOctreeNode* Node);

    /** Builds the 27-point subdivision grid from a parent's corners (stripping edit
     *  contributions), interpolates noise-only densities to midpoints, then re-samples
     *  edit deltas at full resolution for all 27 points. Assigns results to children. */
    void InterpolateChildrenWithEdits(FAdaptiveOctreeNode* Node);

    // =====================================================================
    // Chunk cut (variable-depth chunk roots)
    // =====================================================================

    /** True if the node's worst-case FVector3f jitter (Extent * FLT_EPSILON) would project
     *  larger than sqrt(ThresholdSq) at the camera's near distance (closest point of the
     *  node's AABB, floored). Mirrors the screen-space form of EvaluateSplit. Promote uses
     *  the precision threshold; demote uses a tighter (hysteresis-scaled) threshold on the
     *  parent so a chunk parked on a distance boundary doesn't thrash. */
    bool ChunkJitterExceeds(FAdaptiveOctreeNode* Node, FVector CameraPosition, double FOVScale, double ThresholdSq) const;

    /** True if all 8 children of Node exist and are chunk roots -- i.e. Node sits exactly
     *  one level above the chunk-cut frontier and is a candidate to collapse into. */
    bool AllChildrenAreChunkRoots(FAdaptiveOctreeNode* Node) const;

    /** Builds and meshes a fresh FMeshChunk whose origin is Node->Center, gathering Node's
     *  subtree leaves. Shared by promote (per surface child) and demote (the parent). */
    TSharedPtr<FMeshChunk> BuildMeshChunkFor(FAdaptiveOctreeNode* Node);

    /** Promotes one chunk root into its children: retires the coarse chunk (handed to
     *  OutRemovedChunks for game-thread component destruction), marks all 8 children as
     *  chunk roots, and builds + meshes a new FMeshChunk for each surface-bearing child.
     *  Caller must hold the write lock and the node must be a non-leaf chunk root. */
    void PromoteChunk(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks);

    /** Collapses a parent's 8 chunk-root children back into the parent: retires the child
     *  chunks (handed to OutRemovedChunks), clears their flags, marks the parent a chunk
     *  root, and builds + meshes a single FMeshChunk for it (if it bears surface). Caller
     *  must hold the write lock; the parent must satisfy AllChildrenAreChunkRoots. */
    void DemoteChunk(TSharedPtr<FAdaptiveOctreeNode> ParentNode, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks);

    // =====================================================================
    // LOD
    // =====================================================================

    /** Recursive per-chunk LOD pass: splits leaves that exceed the screen-space threshold,
     *  merges leaves that fall below it, and collects sign-change edges from all leaves. */
    void UpdateLodRecursive(FAdaptiveOctreeNode* Node, FVector CameraPosition, double ThresholdSq, double MergeThresholdSq, double InFOVScale, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged);

    /** Recursively collects sign-change edges from all leaf nodes in a subtree,
     *  deduplicating by FEdgeKey and keeping the smallest (finest-resolution) edge. */
    void GatherLeafEdges(FAdaptiveOctreeNode* Node, TArray<FNodeEdge>& OutEdges, TMap<FEdgeKey, int32>& EdgeMap);

    /** Deduplicates and appends edges to an output array. When a duplicate key exists,
     *  the smaller (finer-resolution) edge replaces the larger one. */
    static void AppendUniqueEdges(const TArray<FNodeEdge>& InAppendEdges, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap);

    // =====================================================================
    // Meshing
    // =====================================================================

    /** Builds mesh stream data for a chunk: finds edge neighbors via biased traversal,
     *  filters edges, builds vertex/triangle arrays, and writes to RealtimeMesh streams. */
    void UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk);

    /** Computes triplanar UV coordinates from a world position and surface normal. */
    FVector2f ComputeTriplanarUV(FVector Position, FVector3f Normal) const;

    /** Finds up to 4 leaf nodes sharing an edge by traversing from root with directional
     *  bias on the two axes perpendicular to the edge. Handles LOD boundaries where
     *  neighbors may be at different depths. */
    FEdgeNeighbors SampleNodesAroundEdge(const FNodeEdge& Edge);

    // =====================================================================
    // Node Data
    // =====================================================================

    /** Splits a node and computes all child data: builds the 27-point subdivision grid,
     *  samples density (or interpolates past precision floor), computes grid normals,
     *  assigns corners to children, and finalizes each child. */
    void SplitAndComputeChildren(FAdaptiveOctreeNode* Node);

    /** Samples density at a node's 8 corners via the compositor and finalizes the node. */
    void ComputeNodeData(FAdaptiveOctreeNode* Node);

    // =====================================================================
    // Traversal Helpers
    // =====================================================================

    /** Walks from root to the leaf containing the given position. */
    FAdaptiveOctreeNode* GetLeafNodeByPoint(FVector Position);

    /** Walks from root to the chunk-depth node containing the given position. */
    FAdaptiveOctreeNode* GetChunkNodeByPoint(FVector Position);

    /** Collects unique corners from edited nodes for batched density re-sampling.
     *  Deduplicates by quantized position so shared corners are sampled once. */
    void GatherUniqueCorners(FAdaptiveOctreeNode* Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& CornerMap, double QuantizeGrid);

public:
    /** Constructs the octree: initializes from params, builds tree to ChunkDepth,
     *  samples initial density, and creates all mesh chunks. */
    explicit FAdaptiveOctree(const FOctreeParams& Params);

    /** Applies an additive spherical brush edit. Writes to the edit store, reconstructs
     *  affected subtrees, enforces LOD splits, and rebuilds dirty chunk meshes. */
    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    /** Per-frame LOD update: evaluates split/merge for all chunks in parallel based on
     *  camera distance and screen-space threshold, then rebuilds mesh streams for changed chunks. */
    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    /** Chunk-cut pass: keeps each chunk root at the depth where its float-precision jitter
     *  is just sub-pixel at the camera's near distance. Promotes chunks to finer children
     *  where jitter would show (near the camera) and demotes finer children back into their
     *  parent once the region is comfortably far (a tighter, hysteresis-scaled threshold on
     *  the parent prevents thrashing at the boundary). One level of change per chunk per
     *  pass, so the near-camera funnel refines and the wake collapses gradually. Retired
     *  chunks (coarse parents on promote, fine children on demote) are appended to
     *  OutRemovedChunks for the caller to destroy on the game thread; new chunks are marked
     *  dirty and flow through the normal CollectDirtyChunks/apply path. Caller holds the
     *  write lock. */
    void UpdateChunkCut(FVector InCameraPosition, double InCameraFOV, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks);

    /** Collects all currently-dirty mesh chunks (holding a shared ref to each) into
     *  OutDirtyChunks, for the caller to apply on the game thread under a per-frame
     *  throttle. Replaces the old UpdateMesh, which pushed every dirty chunk to the game
     *  thread at once via a per-chunk AsyncTask -- an unthrottled RegisterComponent burst
     *  on first build. */
    void CollectDirtyChunks(TArray<TSharedPtr<FMeshChunk>>& OutDirtyChunks) const;

    /** Destroys the entire tree and all mesh chunks. */
    void Clear();

    // --- Public Accessors ---
    double GetPlanetRadius() const { return PlanetRadius; }
    double GetNoiseAmplitude() const { return NoiseAmplitude; }
    FVector GetPlanetCenter() const { return Root.IsValid() ? Root->Center : FVector::ZeroVector; }
};