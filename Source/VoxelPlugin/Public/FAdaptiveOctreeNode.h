// FAdaptiveOctreeNode.h - A single node in the adaptive octree. Stores corner
// density/normal data, dual contour vertex position, sign-change edges, and
// LOD split/merge state. Nodes form a tree via Parent/Children pointers and
// track their path from root via a morton index.

#pragma once

#include "CoreMinimal.h"
#include "FNodeStructs.h"

/** Index constants for the DepthBounds[3] array, which stores the three
 *  depth limits inherited from the tree root at construction time. */
namespace EDepthBound
{
    static constexpr int32 ChunkDepth = 0;
    static constexpr int32 MinDepth = 1;
    static constexpr int32 MaxDepth = 2;
};

/** A single node in the adaptive octree, storing spatial bounds, 8-corner SDF
 *  density/normal data, sign-change edges, and the dual contour vertex.
 *
 *  Nodes are either leaves (renderable) or interior (subdivided into 8 children).
 *  The LOD system splits and merges nodes based on screen-space projected size.
 *  Surface detection propagates up the tree so that ancestor nodes know whether
 *  their subtree contains geometry. */
struct VOXELPLUGIN_API FAdaptiveOctreeNode : public TSharedFromThis<FAdaptiveOctreeNode>
{
private:
    /** Solves the QEF (Quadric Error Function) from sign-change edges to place
     *  the dual contour vertex. Falls back to mass point when the QEF drifts
     *  too far, with a smoothstep blend in between. */
    void ComputeDualContourPosition();

    /** Trilinearly interpolates corner normals at an arbitrary point within the node.
     *  Used to evaluate surface normals at sign-change edge crossing points for QEF input. */
    FVector GetInterpolatedNormal(FVector P);

    bool bIsLeaf = true;

    /** Edges whose two corners have opposite density signs (surface crossings).
     *  Built by FinalizeFromExistingCorners, consumed by the mesher via GetSignChangeEdges(). */
    TArray<FNodeEdge> SignChangeEdges;

public:
    /** Morton index encoding the path from root to this node (3 bits per level). */
    FMortonIndex Index;

    TWeakPtr<FAdaptiveOctreeNode> Parent;
    TSharedPtr<FAdaptiveOctreeNode> Children[8];

    /** SDF density, world position, and gradient normal at each of the 8 corners.
     *  Corner layout follows OctreeConstants::Offsets (bit 0=X, bit 1=Y, bit 2=Z). */
    FNodeCorner Corners[8];

    /** Depth limits inherited from the tree root, indexed by EDepthBound.
     *  [ChunkDepth]: depth at which the tree is spatially partitioned into mesh chunks.
     *  [MinDepth]: minimum depth the LOD system will maintain regardless of distance.
     *  [MaxDepth]: absolute maximum subdivision depth. */
    uint8 DepthBounds[3];

    /** World-space center and half-extent of this node's AABB. */
    FVector Center;
    double Extent;

    /** Center of the chunk-depth ancestor that owns this node. Used to compute
     *  chunk-local vertex positions during meshing. */
    FVector ChunkCenter;

    /** Dual contour vertex position and normal for this node. Computed by the QEF
     *  solver from sign-change edges. Used as the mesh vertex when this node is a leaf. */
    FVector DualContourPosition;
    FVector3f DualContourNormal = FVector3f(0, 0, 1);

    /** True if this node has at least one sign-change edge (density crosses zero). */
    bool IsSurfaceNode = false;

    /** True if the node has sign changes (IsSurfaceNode) OR if any corner density
     *  is close enough to zero that the surface could exist between corners at
     *  finer resolution. Used by the LOD system to avoid skipping near-surface nodes. */
    bool CouldContainSurface = false;

    /** True if this node once had descendants with edit-created surface that
     *  its own corners can't resolve. Set during merge, inherited during split,
     *  so the LOD system re-splits to the correct depth when the camera returns. */
    bool bHasEditedDescendants = false;

    /** When true, prevents the LOD system from merging this node regardless of distance. */
    bool LodOverride = false;

    bool IsLeaf() const;
    bool IsRoot() const;

    /** Returns true if the node has sign changes or any corner density is within
     *  proximity of the zero crossing. Threshold is sqrt(3) * Extent (the node's
     *  space diagonal), ensuring corners one full diagonal away are considered. */
    bool IsNearSurface() const
    {
        if (IsSurfaceNode) return true;
        const float Threshold = (float)(Extent * 1.74);
        for (int i = 0; i < 8; i++)
        {
            if (FMath::Abs(Corners[i].Density) < Threshold)
                return true;
        }
        return false;
    }

    /** Screen-space split test. Returns true if the node's projected size exceeds
     *  the threshold at the given distance. Respects MinDepth (always split) and
     *  MaxDepth (never split) bounds. */
    static bool EvaluateSplit(double Extent, double DistSq, double FOVScale, double ThresholdSq, int Depth, int MaxDepth, int MinDepth)
    {
        if (Depth >= MaxDepth) return false;
        if (Depth < MinDepth) return true;
        double lhs = 2.0 * Extent * FOVScale;
        return (lhs * lhs) > (ThresholdSq * DistSq);
    }

    /** Screen-space merge test. Returns true if the node's projected size is below
     *  the merge threshold. Never merges at or below ChunkDepth or below MinDepth. */
    static bool EvaluateMerge(double Extent, double DistSq, double FOVScale, double MergeThresholdSq, int Depth, int ChunkDepth, int MinDepth)
    {
        if (Depth <= ChunkDepth) return false;
        if (Depth < MinDepth) return false;
        double lhs = 2.0 * Extent * FOVScale;
        return (lhs * lhs) < (MergeThresholdSq * DistSq);
    }

    /** Instance split/merge tests that include back-face culling (split) and
     *  LodOverride (merge). ThresholdSq/MergeThresholdSq are precomputed once per LOD pass. */
    bool ShouldSplit(FVector InCameraPosition, double ThresholdSq, double InFOVScale);
    bool ShouldMerge(FVector InCameraPosition, double MergeThresholdSq, double InFOVScale);

    /** Creates 8 children, transitioning this node from leaf to interior.
     *  Children inherit DepthBounds and get their ChunkCenter set to this node's
     *  center if this node is at ChunkDepth (starting a new chunk). */
    void Split();

    /** Destroys all descendants, transitioning this node back to a leaf.
     *  Preserves bHasEditedDescendants from children so edit regions are
     *  re-split when the camera returns. */
    void Merge();

    /** Collects all chunk-depth nodes with surface geometry in this subtree. */
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceChunks();

    /** Computes the normalized position on a sphere of given radius.
     *  Projects DualContourPosition onto the sphere surface. */
    FVector ComputeNormalizedPosition(double InRadius) const;

    /** Returns this node's sign-change edges (surface crossings). */
    TArray<FNodeEdge>& GetSignChangeEdges();

    /** Recomputes normals (unless skipped), sign-change edges, dual contour position,
     *  and surface flags from the current corner densities. Propagates IsSurfaceNode
     *  and CouldContainSurface up to ancestors. Called after corner data is populated
     *  (initial construction, LOD split, or edit reconstruction). */
    void FinalizeFromExistingCorners(bool bSkipNormals = false);

    /** Root constructor. Creates the top-level node centered at origin with the
     *  given extent and depth bounds. */
    FAdaptiveOctreeNode(double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    /** Child constructor. Creates a child node at the given octant of InParent.
     *  InAnchorCenter is the chunk center to inherit (or parent's center if at chunk depth). */
    FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 InChildIndex, FVector InAnchorCenter);
};