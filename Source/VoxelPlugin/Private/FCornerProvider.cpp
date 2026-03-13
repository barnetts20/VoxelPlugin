// Fill out your copyright notice in the Description page of Project Settings.

#include "FAdaptiveOctreeNode.h"
#include "FCornerProvider.h"

TSharedPtr<FVoxelCorner> FCornerProvider::GetOrCreateCorner(FVector InPosition)
{
    auto cornerKey = FVoxelCorner::QuantizeKey(InPosition);
    return GetOrCreateCorner(cornerKey);
}

TSharedPtr<FVoxelCorner> FCornerProvider::GetOrCreateCorner(FInt64Vector InKey)
{
    {
        FReadScopeLock ReadLock(CornerLock);
        TWeakPtr<FVoxelCorner>* Found = CornerLookup.Find(InKey);
        if (Found)
        {
            TSharedPtr<FVoxelCorner> Pinned = Found->Pin();
            if (Pinned.IsValid()) return Pinned;
        }
    }

    FWriteScopeLock WriteLock(CornerLock);
    TWeakPtr<FVoxelCorner>* Found = CornerLookup.Find(InKey);
    if (Found)
    {
        TSharedPtr<FVoxelCorner> Pinned = Found->Pin();
        if (Pinned.IsValid()) return Pinned;
    }

    TSharedPtr<FVoxelCorner> NewCorner = MakeShared<FVoxelCorner>(InKey);
    CornerLookup.Add(InKey, NewCorner);
    return NewCorner;
}

// Private helper — samples a flat list of corners, writes Density + Normal
void FCornerProvider::SampleCorners(TArray<TSharedPtr<FVoxelCorner>>& InCorners)
{
    int32 N = InCorners.Num();
    if (N == 0) return;

    const float Epsilon = 1.0f;
    const float InvSurface = (float)(1.0 / SurfaceExtent);
    int32 TotalCount = N * 7;

    TArray<float> X, Y, Z, NX, NY, NZ, OutDensities;
    X.SetNumUninitialized(TotalCount);   Y.SetNumUninitialized(TotalCount);   Z.SetNumUninitialized(TotalCount);
    NX.SetNumUninitialized(TotalCount); NY.SetNumUninitialized(TotalCount); NZ.SetNumUninitialized(TotalCount);
    OutDensities.SetNum(TotalCount);

    ParallelFor(N, [&](int32 i)
        {
            FVector P = InCorners[i]->GetPosition();
            int32 Base = i * 7;

            X[Base] = P.X; Y[Base] = P.Y; Z[Base] = P.Z;
            X[Base + 1] = P.X + Epsilon; X[Base + 2] = P.X - Epsilon; Y[Base + 1] = P.Y; Y[Base + 2] = P.Y; Z[Base + 1] = P.Z; Z[Base + 2] = P.Z;
            Y[Base + 3] = P.Y + Epsilon; Y[Base + 4] = P.Y - Epsilon; X[Base + 3] = P.X; X[Base + 4] = P.X; Z[Base + 3] = P.Z; Z[Base + 4] = P.Z;
            Z[Base + 5] = P.Z + Epsilon; Z[Base + 6] = P.Z - Epsilon; X[Base + 5] = P.X; X[Base + 6] = P.X; Y[Base + 5] = P.Y; Y[Base + 6] = P.Y;

            for (int32 j = 0; j < 7; j++)
            {
                NX[Base + j] = X[Base + j] * InvSurface;
                NY[Base + j] = Y[Base + j] * InvSurface;
                NZ[Base + j] = Z[Base + j] * InvSurface;
            }
        });

    NoiseFunction(TotalCount, NX.GetData(), NY.GetData(), NZ.GetData(), OutDensities.GetData());

    ParallelFor(N, [&](int32 i)
        {
            int32 Base = i * 7;
            auto Sample = [&](int32 Offset) -> double {
                return CompositeSample(
                    FVector(X[Base + Offset], Y[Base + Offset], Z[Base + Offset]),
                    (double)OutDensities[Base + Offset]);
                };

            InCorners[i]->Density = Sample(0);
            InCorners[i]->Normal = FVector3f(
                (float)(Sample(1) - Sample(2)),
                (float)(Sample(3) - Sample(4)),
                (float)(Sample(5) - Sample(6))
            ).GetSafeNormal();
            InCorners[i]->bIsInitialized = true;
        });
}

void FCornerProvider::ProvisionCorners(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes)
{
    // 1. Acquire corners for all nodes, collect unsampled ones
    TArray<TSharedPtr<FVoxelCorner>> NewCorners;

    for (const auto& Node : InNodes)
    {
        for (int32 i = 0; i < 8; i++)
        {
            TSharedPtr<FVoxelCorner> Corner = GetOrCreateCorner(Node->GetCornerPosition(i));
            Node->Corners[i] = Corner;
            if (!Corner->bIsInitialized)
                NewCorners.AddUnique(Corner);
        }
    }

    // 2. Sample any corners we haven't seen before
    SampleCorners(NewCorners);

    // 3. Finalize nodes
    ParallelFor(InNodes.Num(), [&](int32 i)
        {
            InNodes[i]->ComputeDualContourPosition();
            InNodes[i]->ComputeNormalizedPosition(SeaExtent);
            InNodes[i]->bIsInitialized = true;
        });
}

void FCornerProvider::UpdateCorners(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes)
{
    // Collect unique corners across all affected nodes
    TSet<TSharedPtr<FVoxelCorner>> UniqueCorners;
    for (const auto& Node : InNodes)
        for (int32 i = 0; i < 8; i++)
            if (Node->Corners[i].IsValid())
                UniqueCorners.Add(Node->Corners[i]);

    TArray<TSharedPtr<FVoxelCorner>> CornersToSample = UniqueCorners.Array();
    SampleCorners(CornersToSample);

    ParallelFor(InNodes.Num(), [&](int32 i)
        {
            InNodes[i]->ComputeDualContourPosition();
            InNodes[i]->ComputeNormalizedPosition(SeaExtent);
        });
}

void FCornerProvider::ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution, const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InAffectedNodes)
{
    int32 Depth = EditStore->GetDepthForBrushRadius(InEditRadius, InEditResolution);
    EditStore->ApplySphericalEdit(InEditCenter, InEditRadius, InEditStrength, Depth);
    UpdateCorners(InAffectedNodes);
}

void FCornerProvider::PruneExpiredCorners()
{
    FWriteScopeLock WriteLock(CornerLock);
    for (auto It = CornerLookup.CreateIterator(); It; ++It)
        if (!It.Value().IsValid()) It.RemoveCurrent();
}
