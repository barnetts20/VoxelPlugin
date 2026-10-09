// FAdaptiveOctree.h — The adaptive octree: manages tree construction, LOD-driven
// split/merge, density evaluation, edit application, and dual-contour mesh generation.
//
// Lifecycle:
//   1. Construction: builds tree to ChunkDepth, samples density, populates mesh chunks.
//   2. Per-frame: UpdateLOD splits/merges leaves by screen-space size, keeps the leaves
//      2:1 balanced, re-places the vertices around the changes, and re-meshes the changed
//      chunks together with their neighbours; the owning actor collects dirty chunks and
//      applies them on the game thread.
//   3. Edits: ApplyEdit writes to the sparse edit store, reconstructs the chunk roots the
//      brush touches, enforces LOD splits for newly-surfaced nodes, and re-meshes those
//      chunks and their neighbours.
//
// Every operation that publishes meshes runs inside a pass (BeginPass). The pass id is
// stamped on each published stream so the actor can apply a pass's chunks together.

#pragma once

#include "CoreMinimal.h"
#include "HAL/IConsoleManager.h"
#include "FMeshingStructs.h"
#include "FSparseEditStore.h"
#include "FAdaptiveOctreeNode.h"
#include "FDensitySampleCompositor.h"
#include <atomic>

/** Flicker diagnostics (0 off, 1 statistics, stale-seam check and debug bounds). */
extern TAutoConsoleVariable<int32> CVarVoxelDebugFlicker;

/** Vertex colour of newly meshed chunks (0 constant, 1 chunk depth, 2 meshing pass). */
extern TAutoConsoleVariable<int32> CVarVoxelDebugChunkColor;

/** With flicker diagnostics on, run the stale-seam check every N data passes. */
extern TAutoConsoleVariable<int32> CVarVoxelDebugSeamCheckInterval;

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

    // --- Tree Structure ---

    /** Depth the chunk cut is built at: the tree is split to here at build and every node
     *  at this depth becomes a chunk root. Also the floor the chunk-cut pass never demotes
     *  below. */
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

    /** Restricted-octree balance: the largest depth difference allowed between face-adjacent
     *  chunk roots. The mesher can only stitch a boundary whose leaf sizes differ by <= ~2
     *  levels, and a chunk floors its leaves at its own depth, so an unbounded chunk-depth
     *  gap opens cracks. The chunk-cut pass force-promotes a chunk that is more than this
     *  many levels coarser than a neighbor and refuses a demote that would exceed it. 1 is
     *  the safe 2:1 balance; 2 allows a steeper (cheaper) funnel and is usually still
     *  crack-free. */
    int MaxChunkDepthDelta = 1;

    int MinDepth = 8;
    int MaxDepth = 22;
};

/** Debug statistics over the chunk map (FAdaptiveOctree::GetDebugStats). */
struct VOXELPLUGIN_API FOctreeDebugStats {
    int32 NumChunks = 0;
    int32 NumDirty = 0;
    /** GFrameCounter of the oldest publish still waiting to be applied (0 if none). */
    uint64 OldestDirtyPublishFrame = 0;
};

/** Adaptive octree for LOD-driven dual-contour voxel meshing.
 *
 *  Manages a hierarchy of FAdaptiveOctreeNode from a single root down to leaf nodes
 *  that produce mesh geometry. The tree is spatially partitioned at chunk roots into
 *  mesh chunks, each backed by a RealtimeMesh component. Below the chunk roots, leaves
 *  are split and merged per pass based on screen-space projected size.
 *
 *  A chunk's mesh depends on the leaves of its neighbours (boundary faces take vertices
 *  from both sides), so whenever a chunk's leaves change, it is re-meshed together with
 *  every adjacent chunk in the same pass.
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

    /** Maps surface-bearing chunk roots to their mesh chunk. Keyed by shared pointer so
     *  chunk identity is tied to the node's lifetime. */
    TMap<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>> ChunkMap;
    bool MeshChunksInitialized = false;

    /** Id of the current pass, stamped on every stream published during it. */
    uint32 CurrentPassId = BuildPassId;

public:
    /** Pass id of the initial build (constructor). */
    static constexpr uint32 BuildPassId = 1;

private:

    /** Edges the mesher dropped because fewer than 3 leaves were usable around them
     *  (a neighbour more than two levels coarser). Counted only with flicker diagnostics
     *  on, once per mesh of the chunk holding the edge; split by whether the edge lies on
     *  a chunk face. */
    std::atomic<int32> DroppedInteriorEdges{ 0 };
    std::atomic<int32> DroppedFaceEdges{ 0 };

    /** Quads skipped because a leaf around the edge had no vertex data, and triangles that
     *  face against their vertex normals. Counted only with flicker diagnostics on. */
    std::atomic<int32> VertexlessQuads{ 0 };
    std::atomic<int32> FoldedTriangles{ 0 };

    // --- Tree Geometry ---
    double RootExtent;
    double ChunkExtent;
    int ChunkDepth;
    int MaxChunkDepth;
    double ChunkPrecisionThreshold;
    double ChunkDemoteHysteresis;
    int MaxChunkDepthDelta;

    /** Footprint (cm) passed with every density sample: the cell size at MaxDepth. One value
     *  for all depths, so a corner shared by nodes of different depths has one density and
     *  the minimal-edge stitching sees consistent signs; only octaves finer than the finest
     *  cell are left out. */
    double SampleFootprint;

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

    /** Recursively collects all chunk roots where CouldContainSurface is true. */
    void CollectNearSurfaceChunks(FAdaptiveOctreeNode* Node, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out);

    // =====================================================================
    // Edit Pipeline
    // =====================================================================

    /** Collects every chunk root (surface or not) whose AABB intersects the sphere. Chunk
     *  roots form an anti-chain, so the walk stops at each. */
    void CollectChunkRootsInSphere(FAdaptiveOctreeNode* Node, const FVector& SphereCenter, double SphereRadius, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out) const;

    /** Full reconstruction of an edited subtree. Gathers unique corners, re-samples
     *  density in one bulk compositor call, propagates deep densities past the precision
     *  floor, and finalizes all nodes (normals, edges, QEF, surface flags). */
    void ReconstructSubtree(FAdaptiveOctreeNode* Node);

    /** Post-reconstruction pass: walks edited leaves and splits any that are now
     *  CouldContainSurface but weren't previously split. Uses cached LOD state. Appends
     *  every node it splits to OutSplitNodes. */
    void EnforceSplitsInSubtree(FAdaptiveOctreeNode* Node, TArray<FAdaptiveOctreeNode*>& OutSplitNodes);

    /** Bottom-up finalization: recomputes normals, sign-change edges, dual contour
     *  position, and surface flags for all edited nodes in a subtree. */
    void FinalizeSubtree(FAdaptiveOctreeNode* Node);

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

    /** Collects every chunk root in the subtree (surface AND non-surface), stopping at each
     *  since chunk roots form an anti-chain. The chunk-cut pass iterates this rather than
     *  ChunkMap: non-surface chunk roots have no FMeshChunk (aren't in ChunkMap) but must
     *  still be demotable, or a promoted parent with all-non-surface children can never
     *  collapse and deadlocks the cut. */
    void CollectAllChunkRoots(FAdaptiveOctreeNode* Node, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out) const;

    /** True if any chunk root just outside Node's six faces is deeper than DepthThreshold.
     *  Samples each face at 2^(MaxChunkDepthDelta+1) per axis -- fine enough that a
     *  delta-violating (>= threshold+1) neighbour can't hide between samples -- and
     *  short-circuits on the first hit. Used to keep the chunk cut restricted: a chunk more
     *  than MaxChunkDepthDelta shallower than a neighbour must refine, and a demote that
     *  would breach the delta is refused. */
    bool AnyFaceNeighborChunkDeeperThan(FAdaptiveOctreeNode* Node, int DepthThreshold);

    /** Creates an FMeshChunk whose origin is Node->Center with Node's subtree edges
     *  gathered, not yet meshed. Callers add every new chunk to ChunkMap before meshing
     *  any, since the mesher consults the map (OwnsSharedEdge). */
    TSharedPtr<FMeshChunk> CreateMeshChunk(FAdaptiveOctreeNode* Node);

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

    /** Recursive per-chunk LOD step, safe to run on chunks in parallel: splits leaves that
     *  exceed the screen-space threshold (appended to OutSplitNodes) and collects parents
     *  whose children are all leaves and fall below the merge threshold
     *  (OutMergeCandidates). Merging is left to the serial step. */
    void UpdateLodSplits(FAdaptiveOctreeNode* Node, FVector CameraPosition, double ThresholdSq, double MergeThresholdSq, double InFOVScale, TArray<FAdaptiveOctreeNode*>& OutSplitNodes, TArray<FAdaptiveOctreeNode*>& OutMergeCandidates);

    /** True if collapsing Parent's children keeps the leaves balanced: no surface leaf
     *  touching Parent is more than one level finer than it, and if Parent has crossings
     *  of its own, no leaf touching it is more than one level coarser. */
    bool CanMergeBalanced(FAdaptiveOctreeNode* Parent);

    /** Restores 2:1 balance around Seeds (a split node seeds its children, a leaf seeds
     *  itself): every leaf touching a surface leaf ends at most one level coarser than it,
     *  splitting coarser leaves (and repeating for their children) as needed. Leaves above
     *  the chunk cut are left alone. Adds the chunk roots and regions it changes. */
    void BalanceAfterSplits(const TArray<FAdaptiveOctreeNode*>& SplitNodes, TSet<FAdaptiveOctreeNode*>& InOutChangedRoots, TArray<FBox>& InOutChangedRegions);

    /** Re-places a leaf's vertex from its own sign-change edges plus the sign-change edges of
     *  same-size or finer touching leaves that lie on its faces. Writes only Leaf. */
    void ComputeLeafVertex(FAdaptiveOctreeNode* Leaf);

    /** ComputeLeafVertex for every leaf (under a chunk root) touching a changed region, in
     *  parallel. Adds the chunk roots of leaves whose vertex moved. */
    void RefreshVertices(const TArray<FBox>& ChangedRegions, TSet<FAdaptiveOctreeNode*>& OutVertexChangedRoots);

    /** Common end of a pass that changed leaves: refreshes vertices around the changed
     *  regions, creates chunks for changed roots that gained surface, regathers changed
     *  chunks' edges from their current leaves, and meshes them with their neighbours. */
    void FinishPass(TSet<FAdaptiveOctreeNode*>& ChangedRoots, const TArray<FBox>& ChangedRegions);

    /** Recursively collects sign-change edges from all leaf nodes in a subtree,
     *  deduplicating by FEdgeKey and keeping the smallest (finest-resolution) edge. */
    void GatherLeafEdges(FAdaptiveOctreeNode* Node, TArray<FNodeEdge>& OutEdges, TMap<FEdgeKey, int32>& EdgeMap);

    /** Deduplicates and appends edges to an output array. When a duplicate key exists,
     *  the smaller (finer-resolution) edge replaces the larger one. */
    static void AppendUniqueEdges(const TArray<FNodeEdge>& InAppendEdges, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap);

    // =====================================================================
    // Meshing
    // =====================================================================

    /** Re-meshes the given chunk roots that have mesh chunks (their ChunkEdges must be
     *  current), plus every adjacent surface chunk whose seam inputs changed
     *  (ComputeSeamHash differs from its SeamHash), in one parallel pass. */
    void MeshChunksAndNeighbours(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& ChangedNodes);

    /** Adds every chunk root touching Node's AABB (faces, edges and corners) to Out, by a
     *  tree walk against the AABB grown by a hair. */
    void CollectAdjacentChunkRoots(FAdaptiveOctreeNode* Node, TSet<FAdaptiveOctreeNode*>& Out);

    /** Re-derives a chunk's seam summary from the current tree: the sum, over its edges
     *  lying on its faces, of HashExternalNeighbours. Equals Chunk.SeamHash while the
     *  leaves its boundary faces were built from are unchanged. */
    uint32 ComputeSeamHash(FAdaptiveOctreeNode* ChunkNode, const FMeshChunk& Chunk);

    /** Builds mesh stream data for a chunk: finds edge neighbors via biased traversal,
     *  filters edges, builds vertex/triangle arrays, publishes the stream, and records the
     *  chunk's SeamHash. */
    void UpdateMeshChunkStreamData(FAdaptiveOctreeNode* ChunkNode, TSharedPtr<FMeshChunk> InChunk);

    /** An edge lying on a boundary between chunks can be a sign-change edge of equal-size
     *  leaves in several chunks. Exactly one emits its faces: the chunk whose root path
     *  sorts first (FMortonIndex::PathLess). True if ChunkNode is that chunk, or the only
     *  chunk holding the edge. */
    bool OwnsSharedEdge(const FNodeEdge& Edge, const FEdgeNeighbors& Neighbors, const FAdaptiveOctreeNode* ChunkNode);

    /** Hash of the leaves around an edge that lie outside the chunk (path, dual-contour
     *  vertex and normal, sign-change edge keys); 0 and bOutHasExternal false if all lie
     *  inside. Summed per
     *  chunk into SeamHash. */
    static uint32 HashExternalNeighbours(const FEdgeNeighbors& Neighbors, const FMortonIndex& ChunkIndex, bool& bOutHasExternal);

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

    /** Walks from root to the chunk root containing the given position (nullptr if the
     *  path ends at a leaf above the chunk cut). */
    FAdaptiveOctreeNode* GetChunkNodeByPoint(FVector Position);

    /** Collects unique corners from edited nodes for batched density re-sampling.
     *  Deduplicates by quantized position so shared corners are sampled once. */
    void GatherUniqueCorners(FAdaptiveOctreeNode* Node, TArray<FCornerSample>& Samples, TMap<FInt64Vector, int32>& CornerMap, double QuantizeGrid);

public:
    /** Constructs the octree: initializes from params, builds tree to ChunkDepth,
     *  samples initial density, and creates all mesh chunks. */
    explicit FAdaptiveOctree(const FOctreeParams& Params);

    /** Starts a new pass; streams published until the next call carry its id. Caller
     *  holds the write lock. */
    uint32 BeginPass() { return ++CurrentPassId; }

    /** Applies an additive spherical brush edit: writes it to the edit store, then
     *  RefreshEdit. */
    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);

    /** The octree side of an edit already in the edit store: reconstructs every chunk root
     *  the brush touches, enforces LOD splits, and re-meshes those chunks and their
     *  neighbours. Caller holds the write lock. */
    void RefreshEdit(FVector InEditCenter, double InEditRadius);

    /** Per-pass LOD update: evaluates split/merge for all chunks in parallel based on
     *  camera distance and screen-space threshold, then re-meshes the changed chunks and
     *  their neighbours. */
    void UpdateLOD(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    /** Chunk-cut pass: keeps each chunk root at the depth where its float-precision jitter
     *  is just sub-pixel at the camera's near distance. Promotes chunks to finer children
     *  where jitter would show (near the camera) and demotes finer children back into their
     *  parent once the region is comfortably far (a tighter, hysteresis-scaled threshold on
     *  the parent prevents thrashing at the boundary). One level of change per chunk per
     *  pass, so the near-camera funnel refines and the wake collapses gradually. Retired
     *  chunks (coarse parents on promote, fine children on demote) are appended to
     *  OutRemovedChunks for the caller to destroy on the game thread; new chunks are dirty
     *  and flow through the normal CollectDirtyChunks/apply path. Leaves are unchanged, so
     *  neighbouring chunks keep valid meshes. Caller holds the write lock. */
    void UpdateChunkCut(FVector InCameraPosition, double InCameraFOV, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks);

    /** Collects all currently-dirty mesh chunks (holding a shared ref to each) into
     *  OutDirtyChunks, for the caller to apply on the game thread. */
    void CollectDirtyChunks(TArray<TSharedPtr<FMeshChunk>>& OutDirtyChunks) const;

    // --- Diagnostics (flicker) ---

    /** Re-derives every chunk's seam inputs and appends the bounds (actor-local) of chunks
     *  whose current mesh was built against different neighbour leaves -- a stale seam.
     *  Costs about as much as meshing every chunk once. Caller holds the write lock. */
    void FindStaleSeams(TArray<FBox>& OutStaleBounds);

    /** Chunk and dirty counts and the oldest unapplied publish. Caller holds a lock. */
    void GetDebugStats(FOctreeDebugStats& Out) const;

    /** Appends the bounds (actor-local) of chunks whose last mesh has folded triangles. */
    void FindFoldedChunks(TArray<FBox>& OutFoldedBounds) const;

    /** Returns and resets the mesher's diagnostic counters. */
    void ConsumeDebugCounters(int32& OutDroppedInterior, int32& OutDroppedFace, int32& OutVertexless, int32& OutFolded)
    {
        OutDroppedInterior = DroppedInteriorEdges.exchange(0, std::memory_order_relaxed);
        OutDroppedFace = DroppedFaceEdges.exchange(0, std::memory_order_relaxed);
        OutVertexless = VertexlessQuads.exchange(0, std::memory_order_relaxed);
        OutFolded = FoldedTriangles.exchange(0, std::memory_order_relaxed);
    }

    /** Destroys the entire tree and all mesh chunks. */
    void Clear();

    // --- Public Accessors ---
    double GetPlanetRadius() const { return PlanetRadius; }
    double GetNoiseAmplitude() const { return NoiseAmplitude; }
    FVector GetPlanetCenter() const { return Root.IsValid() ? Root->Center : FVector::ZeroVector; }
};
