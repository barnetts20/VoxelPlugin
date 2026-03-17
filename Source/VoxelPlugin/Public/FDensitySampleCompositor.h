// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "FastNoise/FastNoise.h"
#include "FSparseEditStore.h"

/**
 * Non-owning view into three SoA float streams.
 * Callers own the backing storage (stack arrays, TArrays, etc).
 */
struct VOXELPLUGIN_API FSampleInput
{
    float* X;
    float* Y;
    float* Z;
    int32 Count;

    int32 Num() const { return Count; }

    const float* XData() const { return X; }
    const float* YData() const { return Y; }
    const float* ZData() const { return Z; }

    FVector GetPosition(int32 Index) const
    {
        return FVector(X[Index], Y[Index], Z[Index]);
    }

    FSampleInput(float* InX, float* InY, float* InZ, int32 InCount)
        : X(InX), Y(InY), Z(InZ), Count(InCount) {}
};

class VOXELPLUGIN_API FDensitySampleCompositor
{
private:
    TArray<TFunction<void(const FSampleInput&, float*)>> SampleLayers;
    TSharedPtr<FSparseEditStore> EditStore;
public:

    TSharedPtr<FSparseEditStore> GetEditStore() {
        return EditStore;
    }

    void AddSampleLayer(TFunction<void(const FSampleInput&, float*)> Layer)
    {
        SampleLayers.Add(MoveTemp(Layer));
    }

    void Sample(const FSampleInput& Input, float* DensityOut) const
    {
        int32 Count = Input.Num();
        if (Count <= 0 || SampleLayers.Num() <= 0) return;

        // Fast path: single layer — no temp allocation, no ParallelFor overhead
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

        // Edit store — scalar loop for small batches, parallel only when worth it
        if (EditStore.IsValid())
        {
            if (Count > 128)
            {
                ParallelFor(Count, [&](int32 i) {
                    DensityOut[i] += EditStore->Sample(Input.GetPosition(i));
                    });
            }
            else
            {
                for (int32 i = 0; i < Count; i++)
                    DensityOut[i] += EditStore->Sample(Input.GetPosition(i));
            }
        }
    }

    FDensitySampleCompositor() {};
    FDensitySampleCompositor(TSharedPtr<FSparseEditStore> InEditStore) : EditStore(InEditStore) {};
};

struct VOXELPLUGIN_API FCompositorExamples {
    static TFunction<void(const FSampleInput&, float*)> HeightMapLayer(double ActorPlanetRadius, double ActorNoiseAmplitude, FastNoise::SmartNode<> InNoise);
};