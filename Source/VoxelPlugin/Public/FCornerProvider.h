// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <FSparseEditStore.h>

struct FAdaptiveOctreeNode;

struct VOXELPLUGIN_API FVoxelCorner : public TSharedFromThis<FVoxelCorner> {
    FInt64Vector Position = FInt64Vector::ZeroValue;
    FVector3f Normal = FVector3f(0.0,0.0,1.0);
    double Density = 0;
    bool bIsInitialized = false;

    FVoxelCorner(FVector InPosition) {
        Position = QuantizeKey(InPosition);
    }

    FVoxelCorner(FInt64Vector InPosition) {
        Position = InPosition;
    }

    FORCEINLINE FVector GetPosition() const {
        return FVector((double)Position.X, (double)Position.Y, (double)Position.Z);
    }

    static FORCEINLINE FInt64Vector QuantizeKey(FVector InPosition) {
        return FInt64Vector(
            (int64)FMath::RoundToDouble(InPosition.X),
            (int64)FMath::RoundToDouble(InPosition.Y),
            (int64)FMath::RoundToDouble(InPosition.Z)
        );
    }
};
/**
 * 
 */
class VOXELPLUGIN_API FCornerProvider
{
public:
    TMap<FInt64Vector, TWeakPtr<FVoxelCorner>> CornerLookup;
    FRWLock CornerLock;

    TFunction<void(int32, const float*, const float*, const float*, float*)> NoiseFunction;
    TSharedPtr<FSparseEditStore> EditStore;

    FVector Center;
    double RootExtent;
    double SurfaceExtent;
    double SeaExtent;

    TSharedPtr<FVoxelCorner> GetOrCreateCorner(FVector InPosition);

    TSharedPtr<FVoxelCorner> GetOrCreateCorner(FInt64Vector InKey);

    void SampleCorners(TArray<TSharedPtr<FVoxelCorner>>& InCorners);

    void ProvisionCorners(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes);

    void UpdateCorners(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes);

    void ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution, const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InAffectedNodes);

    void PruneExpiredCorners();

    FORCEINLINE double CompositeSample(const FVector& P, double RawNoise) const
    {
        double dx = P.X - Center.X;
        double dy = P.Y - Center.Y;
        double dz = P.Z - Center.Z;
        double Dist = FMath::Sqrt(dx * dx + dy * dy + dz * dz);

        const double NoiseAmplitude = SurfaceExtent * 0.05; // TODO: expose as property
        return (Dist - SurfaceExtent) - (RawNoise * NoiseAmplitude) + EditStore->Sample(P);
    }

    FCornerProvider(
        TSharedPtr<FSparseEditStore> InEditStore,
        TFunction<void(int32, const float*, const float*, const float*, float*)> InNoiseFunction,
        FVector InCenter,
        double InRootExtent,
        double InSeaLevel,
        double InSurfaceLevel)
        : EditStore(InEditStore)
        , NoiseFunction(InNoiseFunction)
        , Center(InCenter)
        , RootExtent(InRootExtent)
        , SeaExtent(InSeaLevel* InRootExtent)
        , SurfaceExtent(InSurfaceLevel* InRootExtent)
    {
    }
    ~FCornerProvider() = default;
};
