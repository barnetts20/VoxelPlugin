// FSparseEditStore.h - Sparse octree storing per-node density edits (brush strokes).
// Edits are applied at a target depth and stored as corner density deltas on
// sparse nodes. At sample time, all nodes along the path from root to the query
// point contribute their trilinearly-interpolated delta, which is summed into
// the final SDF density by the compositor.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include <atomic>

/** A single node in the sparse edit octree. Only allocated where brush strokes
 *  have reached. CornerDensities stores additive density deltas at the 8 corners
 *  (corner c: bit 0 +X, bit 1 +Y, bit 2 +Z; children the same). HasEdits is true if this node was
 *  directly written to by a brush stroke (vs. just existing as a path ancestor). */
struct FSparseEditNode
{
    double CornerDensities[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    bool HasEdits = false;
    TSharedPtr<FSparseEditNode> Children[8];
};

/** One brush operation, as recorded in the edit log. The log is the authoritative record of
 *  edits (persistence and, later, replication replay it); the sparse tree is built from it. */
struct FVoxelEditOp
{
    uint32 Sequence = 0;
    FVector Center = FVector::ZeroVector;
    double Radius = 0.0;
    double Strength = 0.0;   // added to density at the centre; negative adds material
    int32 Depth = 0;         // edit-tree depth the op was written at
};

/** Sparse octree that stores user-applied density edits (e.g. terrain sculpting).
 *  Covers the terrain lattice's root box but only allocates nodes
 *  where edits exist. Edits are written at a target depth determined by brush radius
 *  and applied as additive density deltas with smoothstep falloff.
 *
 *  Copy-on-write: a published tree is never modified. An edit copies every node on the
 *  paths it writes (untouched subtrees stay shared) and then publishes the new root.
 *  Queries pin the current root once and walk it without locks, so terrain and ocean
 *  can sample from any thread while an edit is being written.
 *
 *  Queried by SampleAdd() (a batch of positions), Sample() (one position) and
 *  HasEditsInBox() (a box; against a snapshot for many queries from any thread). */
class FSparseEditStore
{
public:
    FSparseEditStore(FVector InCenter, double InExtent, int InMaxDepth);

    /** Returns the additive density delta at the given world position by walking
     *  from root to the deepest node containing Position, summing trilinearly-
     *  interpolated corner deltas at each level that has edits. */
    double Sample(FVector Position) const;

    /** Adds the edit delta at each of Count positions (double SoA) to DensityInOut, all
     *  from one snapshot of the tree. Parallel above 128 positions. */
    void SampleAdd(const double* X, const double* Y, const double* Z, int32 Count, float* DensityInOut) const;

    /** True if any node holding edits overlaps Box (a node's edits reach its whole box). */
    bool HasEditsInBox(const FBox& Box) const;

    /** HasEditsInBox against a GetRootSnapshot snapshot, without the lock. */
    bool HasEditsInBox(const FSparseEditNode* Snapshot, const FBox& Box) const
    {
        return HasEditsInBoxRecursive(Snapshot, Center, Extent, Box);
    }

    /** Sequence number of the latest edit (0 if none). */
    uint32 GetLastSequence() const { return LastSequence.load(std::memory_order_acquire); }

    /** Computes the edit-tree depth at which a brush of the given radius should
     *  write its edits. Finds the depth whose node extent best matches the brush
     *  radius, then adds SubdivisionLevels for finer detail within the stroke. */
    int GetDepthForBrushRadius(double BrushRadius, int SubdivisionLevels) const;

    /** Applies an additive spherical brush edit with smoothstep falloff, appends it to the
     *  edit log and publishes the new tree. The caller works out which terrain chunks the
     *  brush touches from its own tree. */
    void ApplySphericalEdit(FVector BrushCenter, double Radius, double Strength, int Depth);

    /** Copy of the edit log, oldest first. */
    TArray<FVoxelEditOp> GetEditLog() const;

    /** Replaces the store's contents with the result of applying Ops in order (e.g. a loaded
     *  save). Sequence numbers continue after the last replayed op. */
    void ReplayEditLog(const TArray<FVoxelEditOp>& Ops);

    /** Publishes an empty tree and clears the log. */
    void Clear();

    /** The current published tree (immutable; null when empty), for copies elsewhere such
     *  as the GPU's flattened tree. */
    TSharedPtr<const FSparseEditNode> GetRootSnapshot() const { return PinRoot(); }
    FVector GetCenter() const { return Center; }
    double GetExtent() const { return Extent; }

private:
    /** Root of the published tree. Read and replaced under RootCS; the tree behind it is
     *  immutable. */
    TSharedPtr<FSparseEditNode> Root;
    mutable FCriticalSection RootCS;

    /** Serializes edits (each builds on the latest published root) and guards EditLog. */
    mutable FCriticalSection WriteCS;
    TArray<FVoxelEditOp> EditLog;
    uint32 NextSequence = 1;
    std::atomic<uint32> LastSequence{ 0 };

    /** Writes one op into a private copy of the tree and publishes it. Caller holds WriteCS. */
    void ApplyOpLocked(const FVoxelEditOp& Op);

    FVector Center;
    double Extent;
    int MaxDepth;

    /** The current published root (keeps that tree alive while the caller walks it). */
    TSharedPtr<FSparseEditNode> PinRoot() const;

    static bool HasEditsInBoxRecursive(const FSparseEditNode* Node, const FVector& NodeCenter, double NodeExtent, const FBox& Box);

    /** Sample() against a pinned tree. */
    double SampleTree(const FSparseEditNode* TreeRoot, const FVector& Position) const;

    /** Returns the child octant index (0-7) for a position relative to a node center.
     *  Bit 0 = X >= center, bit 1 = Y >= center, bit 2 = Z >= center. */
    int GetChildIndex(FVector Position, FVector NodeCenter) const;

    /** Trilinearly interpolates the 8 corner density deltas of a node at the given position. */
    double InterpolateCorners(const FSparseEditNode& Node, FVector NodeCenter, double NodeExtent, FVector Position) const;

    /** Returns the world position of a corner within a node. */
    FVector GetCornerPosition(FVector NodeCenter, double NodeExtent, int CornerIndex) const;

    /** Recursive brush application on an unpublished copy. Node is already a private copy;
     *  each child the brush reaches is copied (or created) before it is descended into.
     *  Writes corner deltas with smoothstep falloff at the target depth. */
    void ApplyEditRecursive(FSparseEditNode& Node, FVector NodeCenter, double NodeExtent,
        FVector BrushCenter, double BrushRadius, double Strength,
        int CurrentDepth, int TargetDepth);
};
