// FSparseEditStore.h — Sparse octree storing per-node density edits (brush strokes).
// Edits are applied at a target depth and stored as corner density deltas on
// sparse nodes. At sample time, all nodes along the path from root to the query
// point contribute their trilinearly-interpolated delta, which is summed into
// the final SDF density by the compositor.

#pragma once

#include "CoreMinimal.h"
#include "FOctreeConstants.h"
#include "FNodeStructs.h"

/** A single node in the sparse edit octree. Only allocated where brush strokes
 *  have reached. CornerDensities stores additive density deltas at the 8 corners
 *  (same layout as OctreeConstants::Offsets). HasEdits is true if this node was
 *  directly written to by a brush stroke (vs. just existing as a path ancestor). */
struct FSparseEditNode
{
    double CornerDensities[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    bool HasEdits = false;
    TSharedPtr<FSparseEditNode> Children[8];
};

/** Sparse octree that stores user-applied density edits (e.g. terrain sculpting).
 *  Mirrors the spatial bounds of the main adaptive octree but only allocates nodes
 *  where edits exist. Edits are written at a target depth determined by brush radius
 *  and applied as additive density deltas with smoothstep falloff.
 *
 *  Queried in two ways:
 *    - Sample(): trilinear interpolation of all edit deltas at a world position,
 *      called per-corner during density evaluation.
 *    - HasEditsAlongPath(): fast path check using a morton index, used to skip
 *      unedited subtrees during octree reconstruction. */
class FSparseEditStore
{
public:
    FSparseEditStore(FVector InCenter, double InExtent, int InChunkDepth, int InMaxDepth);

    /** Returns the additive density delta at the given world position by walking
     *  from root to the deepest node containing Position, summing trilinearly-
     *  interpolated corner deltas at each level that has edits. */
    double Sample(FVector Position) const;

    /** Returns true if the edit store has any nodes along the path defined by
     *  the given morton index. Walks the edit tree level by level - if any
     *  node along the path exists and has edits, or if the path can be walked
     *  to the target depth, edits exist in that region. */
    bool HasEditsAlongPath(const FMortonIndex& Index) const;

    /** Computes the edit-tree depth at which a brush of the given radius should
     *  write its edits. Finds the depth whose node extent best matches the brush
     *  radius, then adds SubdivisionLevels for finer detail within the stroke. */
    int GetDepthForBrushRadius(double BrushRadius, int SubdivisionLevels) const;

    /** Applies an additive spherical brush edit with smoothstep falloff.
     *  Returns the centers of all chunk-depth nodes that overlap the brush,
     *  so the caller knows which chunks need reconstruction. */
    TArray<FVector> ApplySphericalEdit(FVector BrushCenter, double Radius, double Strength, int Depth);

    /** Destroys the entire edit tree, freeing all sparse nodes. */
    void Clear();

private:
    TSharedPtr<FSparseEditNode> Root;
    FVector Center;
    double Extent;
    int ChunkDepth;
    int MaxDepth;

    /** Returns the child octant index (0-7) for a position relative to a node center.
     *  Bit 0 = X >= center, bit 1 = Y >= center, bit 2 = Z >= center. */
    int GetChildIndex(FVector Position, FVector NodeCenter) const;

    /** Trilinearly interpolates the 8 corner density deltas of a node at the given position. */
    double InterpolateCorners(const TSharedPtr<FSparseEditNode>& Node, FVector NodeCenter, double NodeExtent, FVector Position) const;

    /** Returns the world position of a corner within a node. */
    FVector GetCornerPosition(FVector NodeCenter, double NodeExtent, int CornerIndex) const;

    /** Recursive brush application. Descends from root to TargetDepth, creating sparse
     *  nodes as needed, writing corner deltas with smoothstep falloff at the target depth.
     *  Collects affected chunk centers into OutAffectedChunks along the way. */
    void ApplyEditRecursive(TSharedPtr<FSparseEditNode> Node, FVector NodeCenter, double NodeExtent,
        FVector BrushCenter, double BrushRadius, double Strength,
        int CurrentDepth, int TargetDepth, TArray<FVector>& OutAffectedChunks);
};