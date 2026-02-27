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
    
    TArray<FVector> ApplySphericalEdit(FVector BrushCenter, double Radius, double Strength, int Depth);
    
    void Clear();

private:
    static inline const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1,  1, -1), FVector(1,  1, -1),
        FVector(-1, -1,  1), FVector(1, -1,  1),
        FVector(-1,  1,  1), FVector(1,  1,  1)
    };

    TSharedPtr<FSparseEditNode> Root;
    
    int MaxDepth;
    
    FVector Center;
    
    double Extent;
    
    int ChunkDepth;

    TArray<FVector> AffectedChunkCenters;

    int GetChildIndex(FVector Position, FVector NodeCenter) const;
    
    double InterpolateCorners(const TSharedPtr<FSparseEditNode>& Node, FVector NodeCenter, double NodeExtent, FVector Position) const;
    
    FVector GetCornerPosition(FVector NodeCenter, double NodeExtent, int CornerIndex) const;
    
    void ApplyEditRecursive(TSharedPtr<FSparseEditNode> Node, FVector NodeCenter, double NodeExtent, FVector BrushCenter, double BrushRadius, double Strength, int CurrentDepth, int TargetDepth);
};