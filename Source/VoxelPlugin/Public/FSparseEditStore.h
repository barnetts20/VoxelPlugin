// FSparseEditStore.h
#pragma once
#include "CoreMinimal.h"
#include "FOctreeConstants.h"
#include "FNodeStructs.h"

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

    // Returns true if the edit store has any nodes along the path defined by
    // the given morton index. Walks the edit tree level by level — if any
    // node along the path exists and has edits, or if the path can be walked
    // to the target depth, edits exist in that region.
    bool HasEditsAlongPath(const FMortonIndex& Index) const;

    int GetDepthForBrushRadius(double BrushRadius, int SubdivisionLevels) const;

    TArray<FVector> ApplySphericalEdit(FVector BrushCenter, double Radius, double Strength, int Depth);

    void Clear();

private:
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