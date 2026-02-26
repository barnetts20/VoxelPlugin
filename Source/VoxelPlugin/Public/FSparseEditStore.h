// FSparseEditStore.h
#pragma once
#include "CoreMinimal.h"

struct FSparseEditNode
{
    double CornerDensities[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    bool HasEdits = false;
    TSharedPtr<FSparseEditNode> Children[8];
};

class FSparseEditStore
{
public:
    FSparseEditStore(FVector InCenter, double InExtent, int InChunkDepth, int InMaxDepth);

    double Sample(FVector Position) const;
    int GetDepthForBrushRadius(double BrushRadius, int SubdivisionLevels) const;
    void ApplySphericalEdit(FVector BrushCenter, double Radius, double Strength, int Depth);
    const TArray<FVector>& GetAffectedChunkCenters() const;
    void Clear();
    bool HasEdits() const;
    int MaxDepth;
    FVector Center;
    double Extent;
    int ChunkDepth;

private:
    TSharedPtr<FSparseEditNode> Root;

    TArray<FVector> AffectedChunkCenters;

    static inline const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1,  1, -1), FVector(1,  1, -1),
        FVector(-1, -1,  1), FVector(1, -1,  1),
        FVector(-1,  1,  1), FVector(1,  1,  1)
    };

    int GetChildIndex(FVector Position, FVector NodeCenter) const;
    double InterpolateCorners(const TSharedPtr<FSparseEditNode>& Node, FVector NodeCenter, double NodeExtent, FVector Position) const;
    FVector GetCornerPosition(FVector NodeCenter, double NodeExtent, int CornerIndex) const;
    void ApplyEditRecursive(TSharedPtr<FSparseEditNode> Node, FVector NodeCenter, double NodeExtent, FVector BrushCenter, double BrushRadius, double Strength, int CurrentDepth, int TargetDepth);
};