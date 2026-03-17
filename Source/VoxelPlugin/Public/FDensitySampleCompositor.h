// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>
#include "FSparseEditStore.h"

using namespace RealtimeMesh;

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
    TFunction<void(const FSampleInput&, float*)> GetHeightMapExample(double ActorPlanetRadius, double ActorNoiseAmplitude) {
        //Composes a density sampling layer that treats the input noise node as if it was a heightmap
        FastNoise::SmartNode<> Noise = FastNoise::NewFromEncodedNodeTree("GQAgAB8AEwCamRk+DQAMAAAAAAAAQAcAAAAAAD8AAAAAAAAAAAA/AAAAAD8AAAAAvwAAAAA/ARsAFwCamRk+AAAAPwAAAAAAAAA/IAAgABMAAABAQBsAJAACAAAADQAIAAAAAAAAQAsAAQAAAAAAAAABAAAAAAAAAAAAAIA/AAAAAD8AAAAAAAAAAIA/AAAAAAAAmpmZPgCamRk+AM3MTD4BEwDNzEw+IAAfABcAAACAvwAAgD8AAIDAAAAAPw8AAQAAAAAAAED//wEAAAAAAD8AAAAAAAAAAIA/AAAAAD8AAACAvwAAAAA/");
        auto HeightmapLayer = [NoiseNode = Noise, PlanetRadius = ActorPlanetRadius, NoiseAmplitude = ActorNoiseAmplitude](const FSampleInput& Input, float* DensityOut) {
            int32 Count = Input.Num();

            double InvNoiseAmplitude = 1.0 / NoiseAmplitude;
            double RootExtent = (PlanetRadius + NoiseAmplitude) * 1.05;

            // Stack buffers for the common small-count paths (8, 19 samples).
            // Only heap-allocate for large bulk calls (ReconstructSubtree).
            constexpr int32 StackLimit = 64;

            float  StackPX[StackLimit], StackPY[StackLimit], StackPZ[StackLimit], StackNoise[StackLimit];
            double StackDist[StackLimit];

            TArray<float>  HeapPX, HeapPY, HeapPZ, HeapNoise;
            TArray<double> HeapDist;

            float* PX; float* PY; float* PZ; float* NoiseOut;
            double* Distances;

            if (Count <= StackLimit)
            {
                PX = StackPX; PY = StackPY; PZ = StackPZ; NoiseOut = StackNoise;
                Distances = StackDist;
            }
            else
            {
                HeapPX.SetNumUninitialized(Count);
                HeapPY.SetNumUninitialized(Count);
                HeapPZ.SetNumUninitialized(Count);
                HeapNoise.SetNumUninitialized(Count);
                HeapDist.SetNumUninitialized(Count);
                PX = HeapPX.GetData(); PY = HeapPY.GetData(); PZ = HeapPZ.GetData();
                NoiseOut = HeapNoise.GetData(); Distances = HeapDist.GetData();
            }

            // Project positions onto sphere surface for noise sampling.
            // Cache Dist to avoid recomputing sqrt in the density loop.
            for (int32 i = 0; i < Count; i++)
            {
                double px = Input.X[i], py = Input.Y[i], pz = Input.Z[i];
                double Dist = FMath::Sqrt(px * px + py * py + pz * pz);
                Distances[i] = Dist;
                double InvDist = (Dist > 1e-10) ? (RootExtent / Dist) : 0.0;
                PX[i] = (float)(px * InvDist * InvNoiseAmplitude);
                PY[i] = (float)(py * InvDist * InvNoiseAmplitude);
                PZ[i] = (float)(pz * InvDist * InvNoiseAmplitude);
            }

            NoiseNode->GenPositionArray3D(NoiseOut, Count, PX, PY, PZ, 0, 0, 0, 0);

            for (int32 i = 0; i < Count; i++)
            {
                double Clamped = FMath::Clamp((double)NoiseOut[i], -1.0, 1.0);
                double Height = (Clamped + 1.0) * 0.5 * NoiseAmplitude;
                DensityOut[i] = (float)(Distances[i] - (PlanetRadius + Height));
            }
        };
        return HeightmapLayer;
    }
};