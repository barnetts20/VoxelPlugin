// FDensitySampleCompositor.h — Composites multiple density sample layers and
// edit-store deltas into a final SDF density value. Layers are evaluated in
// parallel when multiple are present, then summed. The edit store is applied
// additively on top.

#pragma once

#include "CoreMinimal.h"
#include "FSparseEditStore.h"

/** Non-owning view into three SoA (struct-of-arrays) float streams representing
 *  batched world-space sample positions. Callers own the backing storage
 *  (stack arrays, TArrays, etc). Passed to density sample layers and the
 *  edit store for batched evaluation. */
struct VOXELPLUGIN_API FSampleInput
{
    float* X;
    float* Y;
    float* Z;
    int32 Count;

    int32 Num() const { return Count; }

    FVector GetPosition(int32 Index) const
    {
        return FVector(X[Index], Y[Index], Z[Index]);
    }

    FSampleInput(float* InX, float* InY, float* InZ, int32 InCount)
        : X(InX), Y(InY), Z(InZ), Count(InCount) {}
};

/** Composites one or more density sample layers (e.g. sphere SDF, noise heightmap)
 *  with an optional sparse edit store into final SDF density values.
 *
 *  Layers are TFunctions that write density values for a batch of positions.
 *  When multiple layers exist they are evaluated in parallel and summed.
 *  The edit store is always applied last as an additive delta.
 *
 *  Shared between PlanetActor's terrain and ocean subsystems so that both
 *  sample the same density field (including user edits). */
class VOXELPLUGIN_API FDensitySampleCompositor
{
public:
    FDensitySampleCompositor() {}
    FDensitySampleCompositor(TSharedPtr<FSparseEditStore> InEditStore) : EditStore(InEditStore) {}

    TSharedPtr<FSparseEditStore> GetEditStore() { return EditStore; }

    /** Registers a density sample layer. Layers are evaluated in registration order
     *  and their outputs are summed. */
    void AddSampleLayer(TFunction<void(const FSampleInput&, float*)> Layer)
    {
        SampleLayers.Add(MoveTemp(Layer));
    }

    /** Evaluates all layers and the edit store, writing final density values to DensityOut.
     *  Single-layer fast path avoids temp allocation and ParallelFor overhead. */
    void Sample(const FSampleInput& Input, float* DensityOut) const
    {
        int32 Count = Input.Num();
        if (Count <= 0 || SampleLayers.Num() <= 0) return;

        // Fast path: single layer - no temp allocation, no ParallelFor overhead
        if (SampleLayers.Num() == 1)
        {
            SampleLayers[0](Input, DensityOut);
        }
        else
        {
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
        }

        ApplyEditDeltas(Input, DensityOut, Count);
    }

    /** Applies only edit-store deltas to existing density values (DensityInOut).
     *  Used for nodes beyond PrecisionDepthFloor where noise has insufficient precision
     *  but edits must still be applied on top of interpolated parent densities. */
    void SampleEditsOnly(const FSampleInput& Input, float* DensityInOut) const
    {
        int32 Count = Input.Num();
        if (Count <= 0) return;

        ApplyEditDeltas(Input, DensityInOut, Count);
    }

private:
    TArray<TFunction<void(const FSampleInput&, float*)>> SampleLayers;
    TSharedPtr<FSparseEditStore> EditStore;

    /** Shared helper: additively applies edit-store deltas to an existing density buffer.
     *  Uses ParallelFor for batches larger than 128 samples. */
    void ApplyEditDeltas(const FSampleInput& Input, float* DensityInOut, int32 Count) const
    {
        if (!EditStore.IsValid()) return;

        if (Count > 128)
        {
            ParallelFor(Count, [&](int32 i) {
                DensityInOut[i] += EditStore->Sample(Input.GetPosition(i));
                });
        }
        else
        {
            for (int32 i = 0; i < Count; i++)
                DensityInOut[i] += EditStore->Sample(Input.GetPosition(i));
        }
    }
};