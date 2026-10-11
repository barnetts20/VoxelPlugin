// FSparseEditStore.h - The terrain's edits as shape operations (Design/EditsAndCaves.md, E1):
// an ordered list of carves and fills of capsules (spheres when the two centres coincide), each
// with a blend radius, applied to the density as it stands, and a sparse index of 8 m cells
// listing the edits that reach them. The core (shapes, smooth min/max, the index, compaction)
// is plain C++ in Private/VoxelEditCore.h; the GPU applies the same edits through
// VoxelEditSample.ush (FVoxelEditGPUData).
//
// Copy-on-write: a published state is never modified. An edit builds a new state on a worker
// and publishes it; queries pin the current state once and read it without locks, so terrain,
// collision and ocean can sample from any thread while an edit is being written.
//
// (The name is the old store's: this replaced the octree of additive density deltas.)

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include <atomic>

namespace VoxelEditCore { struct FState; }

/** How an edit changes the terrain. */
enum class EVoxelEditMode : uint8
{
    Carve,   // air inside the shape (dig)
    Fill     // solid inside the shape (build)
};

/** One edit, as the store keeps it (the record persistence saves and replays). A capsule from A
 *  to B in the store's frame (cm), a sphere when A == B. */
struct FVoxelEditOp
{
    uint32 Sequence = 0;
    EVoxelEditMode Mode = EVoxelEditMode::Carve;
    FVector A = FVector::ZeroVector;
    FVector B = FVector::ZeroVector;
    double Radius = 0.0;
    double Blend = 0.0;   // smooth blend radius with the density it edits (cm)
};

/** What an edit did. */
struct FVoxelEditResult
{
    bool bChanged = false;    // false: nothing changed (repeated, or the brush stayed put)
    uint32 Sequence = 0;      // the store's sequence after it
    FBox Bounds = FBox(ForceInit);   // where the surface may have moved (bChanged)
    int32 Dropped = 0;        // earlier edits it made irrelevant (removed)
};

class VOXELPLUGIN_API FSparseEditStore
{
public:
    /** The store covers the terrain lattice's root box (Center, half extent Extent) whose finest
     *  lattice has 2^MaxDepth half cells a half extent: HalfFinest = Extent / 2^MaxDepth. The
     *  index's cells are about 8 m, a power of two of half cells (the GPU shifts lattice
     *  indices). */
    FSparseEditStore(FVector InCenter, double InExtent, int InMaxDepth);
    ~FSparseEditStore();

    /** Applies the edits to the densities at Count positions (double SoA, the actor's frame):
     *  each edit, in order, carves or fills the density as it stands. Footprint (cm, the cell
     *  size the samples are meshed at; 0 exact) shrinks edits narrower than it so they close
     *  smoothly at coarse levels of detail. Parallel above 128 positions. Exact in sign
     *  everywhere and in value near every surface; deep inside solid or air the value is the
     *  cell's (see ApplyWithin). */
    void Apply(const double* X, const double* Y, const double* Z, int32 Count, float* DensityInOut, double Footprint) const;

    /** Apply with every edit reaching within Radius of each position (in order), for tests
     *  that prove a ball of that radius free of surface from one sample (|d| > slope bound x
     *  Radius): that field is slope-bounded and has the full field's sign in the ball. */
    void ApplyWithin(const double* X, const double* Y, const double* Z, int32 Count, float* DensityInOut, double Footprint, double Radius) const;

    /** True if any edit's reach overlaps Box. */
    bool HasEditsInBox(const FBox& Box) const;

    /** Sequence number of the latest change (0 if none). */
    uint32 GetLastSequence() const { return LastSequence.load(std::memory_order_acquire); }

    /** Adds an edit: a sphere at Center, or (bContinueStroke, the same brush held since the
     *  previous call) the stroke from the previous centre to this one, merged into the
     *  previous capsule while the brush moves straight. Earlier edits it makes irrelevant are
     *  removed. Thread safe (edits are serialised). */
    FVoxelEditResult ApplyEdit(EVoxelEditMode Mode, const FVector& Center, double Radius, double Blend, bool bContinueStroke);

    /** The current edits, oldest first (compacted: what persistence saves). */
    TArray<FVoxelEditOp> GetEditLog() const;

    /** Replaces the store's contents with Ops, applied in order as they are (e.g. a loaded
     *  save). Sequence numbers continue after the last op's. */
    void ReplayEditLog(const TArray<FVoxelEditOp>& Ops);

    /** Publishes an empty store. */
    void Clear();

    /** The current published state (immutable), for copies elsewhere such as the GPU's buffers
     *  (FVoxelEditGPUData::Build). */
    TSharedPtr<const VoxelEditCore::FState, ESPMode::ThreadSafe> GetSnapshot() const;

    int32 Num() const;
    FVector GetCenter() const { return Center; }
    double GetExtent() const { return Extent; }
    double GetHalfFinest() const { return HalfFinest; }
    /** The index cells' size (cm): about 8 m, a power of two of half cells. */
    double GetIndexCellSize() const { return IndexCellSize; }

private:
    TSharedPtr<const VoxelEditCore::FState, ESPMode::ThreadSafe> PinState() const;
    void Publish(TSharedPtr<const VoxelEditCore::FState, ESPMode::ThreadSafe> NewState);

    /** The published state. Read and replaced under StateCS; the state behind it is immutable. */
    TSharedPtr<const VoxelEditCore::FState, ESPMode::ThreadSafe> State;
    mutable FCriticalSection StateCS;

    /** Serialises edits (each builds on the latest published state). */
    mutable FCriticalSection WriteCS;
    std::atomic<uint32> LastSequence{ 0 };

    FVector Center;
    double Extent;
    double HalfFinest;
    double IndexCellSize;
};
