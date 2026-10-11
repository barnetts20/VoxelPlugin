// FDensitySampleCompositor.h — Composites multiple density sample layers and the edit store's
// edits into a final SDF density value. Layers are evaluated in parallel when multiple are
// present, then summed. The edits (FSparseEditStore: carves and fills of shapes) apply last, in
// order, to the summed density.

#pragma once

#include "CoreMinimal.h"
#include "Async/ParallelFor.h"
#include "FSparseEditStore.h"

class FVoxelDensityCPU;

/** Non-owning view of a batch of world-space sample positions (double SoA, actor-local) and
 *  the footprint they will be meshed at. Callers own the backing storage. The footprint (cm,
 *  the cell size of the samples) lets the density leave out noise octaves too fine to show;
 *  0 keeps every octave. */
struct VOXELPLUGIN_API FSampleInput
{
    const double* X;
    const double* Y;
    const double* Z;
    int32 Count;
    double Footprint;

    int32 Num() const { return Count; }

    FVector GetPosition(int32 Index) const
    {
        return FVector(X[Index], Y[Index], Z[Index]);
    }

    FSampleInput(const double* InX, const double* InY, const double* InZ, int32 InCount, double InFootprint = 0.0)
        : X(InX), Y(InY), Z(InZ), Count(InCount), Footprint(InFootprint) {}
};

/** Composites one or more density sample layers with an optional edit store into final SDF
 *  density values (negative = solid).
 *
 *  Layers are TFunctions that write density values for a batch of positions.
 *  When multiple layers exist they are evaluated in parallel and summed.
 *  The edits always apply last, to the sum.
 *
 *  Built by VoxelDensity::MakeCompositor and shared between the planet's terrain and ocean,
 *  so both sample the same density field (including user edits). Sampling is safe from any
 *  thread while an edit is being written: the edit store publishes copy-on-write. */
class VOXELPLUGIN_API FDensitySampleCompositor
{
public:
    FDensitySampleCompositor() {}
    FDensitySampleCompositor(TSharedPtr<FSparseEditStore> InEditStore) : EditStore(InEditStore) {}

    TSharedPtr<FSparseEditStore> GetEditStore() { return EditStore; }
    TSharedPtr<const FSparseEditStore> GetEditStore() const { return EditStore; }

    /** The CPU density this compositor was built on (VoxelDensity::MakeCompositor), for
     *  consumers that evaluate the same field elsewhere (the GPU path); null otherwise. */
    TSharedPtr<const FVoxelDensityCPU> GetBaseDensity() const { return BaseDensity; }
    void SetBaseDensity(TSharedPtr<const FVoxelDensityCPU> InDensity) { BaseDensity = MoveTemp(InDensity); }

    /** The smallest footprint the edits are applied at (the base density's MinFootprint), as
     *  the GPU raises every footprint to it: collision and render agree on edits narrower
     *  than it. */
    void SetEditFootprintFloor(double InFootprint) { EditFootprintFloor = FMath::Max(InFootprint, 0.0); }

    /** Registers a density sample layer. Layers are evaluated in registration order
     *  and their outputs are summed. */
    void AddSampleLayer(TFunction<void(const FSampleInput&, float*)> Layer)
    {
        SampleLayers.Add(MoveTemp(Layer));
    }

    /** Evaluates all layers and the edits, writing final density values to DensityOut.
     *  Single-layer fast path avoids temp allocation and ParallelFor overhead. */
    void Sample(const FSampleInput& Input, float* DensityOut) const
    {
        if (!SampleLayersInto(Input, DensityOut)) return;
        if (EditStore.IsValid())
            EditStore->Apply(Input.X, Input.Y, Input.Z, Input.Count, DensityOut, FMath::Max(Input.Footprint, EditFootprintFloor));
    }

    /** Sample for a test that proves a ball of Radius about each position free of surface from
     *  that one value (|d| > slope bound x Radius): every edit reaching the ball applies, not
     *  just those of the position's index cell (FSparseEditStore::ApplyWithin), and at its full
     *  size (no shrinking for the footprint: a brick tree culls a brick's finer levels with it,
     *  where a narrow edit does show). */
    void SampleWithin(const FSampleInput& Input, double Radius, float* DensityOut) const
    {
        if (!SampleLayersInto(Input, DensityOut)) return;
        if (EditStore.IsValid())
            EditStore->ApplyWithin(Input.X, Input.Y, Input.Z, Input.Count, DensityOut, 0.0, Radius);
    }

    /** Applies only the edits to existing density values (DensityInOut). */
    void SampleEditsOnly(const FSampleInput& Input, float* DensityInOut) const
    {
        if (Input.Num() <= 0 || !EditStore.IsValid()) return;
        EditStore->Apply(Input.X, Input.Y, Input.Z, Input.Count, DensityInOut, FMath::Max(Input.Footprint, EditFootprintFloor));
    }

private:
    TArray<TFunction<void(const FSampleInput&, float*)>> SampleLayers;
    TSharedPtr<FSparseEditStore> EditStore;
    TSharedPtr<const FVoxelDensityCPU> BaseDensity;
    double EditFootprintFloor = 0.0;

    /** The layers' sum; false when there is nothing to write. */
    bool SampleLayersInto(const FSampleInput& Input, float* DensityOut) const
    {
        const int32 Count = Input.Num();
        if (Count <= 0 || SampleLayers.Num() <= 0) return false;

        // Fast path: single layer - no temp allocation, no ParallelFor overhead
        if (SampleLayers.Num() == 1)
        {
            SampleLayers[0](Input, DensityOut);
            return true;
        }

        // Multi-layer path
        TArray<TArray<float>> LayerOutputs;
        LayerOutputs.SetNum(SampleLayers.Num());
        for (auto& L : LayerOutputs) L.SetNumUninitialized(Count);

        ParallelFor(SampleLayers.Num(), [&](int32 LayerIdx) {
            SampleLayers[LayerIdx](Input, LayerOutputs[LayerIdx].GetData());
            });

        FMemory::Memcpy(DensityOut, LayerOutputs[0].GetData(), Count * sizeof(float));
        for (int32 LayerIdx = 1; LayerIdx < SampleLayers.Num(); LayerIdx++)
            for (int32 i = 0; i < Count; i++)
                DensityOut[i] += LayerOutputs[LayerIdx][i];
        return true;
    }
};
