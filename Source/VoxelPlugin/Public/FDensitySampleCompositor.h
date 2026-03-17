// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <FSparseEditStore.h>

struct VOXELPLUGIN_API FSampleInput
{
    TArray<float> X;
    TArray<float> Y;
    TArray<float> Z;

    int32 Num() const { return X.Num(); }

    const float* XData() const { return X.GetData(); }
    const float* YData() const { return Y.GetData(); }
    const float* ZData() const { return Z.GetData(); }

    FVector GetPosition(int32 Index) const
    {
        return FVector(X[Index], Y[Index], Z[Index]);
    }

    // Construct empty, sized — caller fills arrays directly (octree path)
    explicit FSampleInput(int32 Count)
    {
        X.SetNumUninitialized(Count);
        Y.SetNumUninitialized(Count);
        Z.SetNumUninitialized(Count);
    }

    // Construct from FVector span — parallel for if large enough
    explicit FSampleInput(TArrayView<const FVector> Positions)
    {
        int32 Count = Positions.Num();
        X.SetNumUninitialized(Count);
        Y.SetNumUninitialized(Count);
        Z.SetNumUninitialized(Count);

        // Parallel worth it above some threshold
        if (Count > 8192)
        {
            ParallelFor(Count, [&](int32 i)
                {
                    X[i] = (float)Positions[i].X;
                    Y[i] = (float)Positions[i].Y;
                    Z[i] = (float)Positions[i].Z;
                });
        }
        else
        {
            for (int32 i = 0; i < Count; i++)
            {
                X[i] = (float)Positions[i].X;
                Y[i] = (float)Positions[i].Y;
                Z[i] = (float)Positions[i].Z;
            }
        }
    }

    // TArray overload just delegates to the view
    explicit FSampleInput(const TArray<FVector>& Positions)
        : FSampleInput(TArrayView<const FVector>(Positions)) {}
};

class VOXELPLUGIN_API FDensitySampleCompositor
{
public:
	TSharedPtr<FSparseEditStore> EditStore;
    virtual void SampleBase(FSampleInput InSampleInput,
        float* DensityOut) const = 0;

    void Sample(FSampleInput InSampleInput, float* DensityOut) const {
        SampleBase(InSampleInput, DensityOut);

        //Sampling the edit store is more expensive than just building arrays, so we just always parallelize
        ParallelFor(InSampleInput.Num(), [&](int32 i) {
            DensityOut[i] += EditStore->Sample(InSampleInput.GetPosition(i));
        });
    }

	FDensitySampleCompositor();
	~FDensitySampleCompositor();
};
