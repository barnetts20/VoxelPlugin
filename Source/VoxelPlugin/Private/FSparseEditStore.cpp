#include "FSparseEditStore.h"

// ============================================================================
// CONSTRUCTION
// ============================================================================

FSparseEditStore::FSparseEditStore(FVector InCenter, double InExtent, int InChunkDepth, int InMaxDepth)
    : Center(InCenter), Extent(InExtent), ChunkDepth(InChunkDepth), MaxDepth(InMaxDepth) {}

// ============================================================================
// SAMPLING (Thread Safe — read only)
// ============================================================================

double FSparseEditStore::Sample(FVector Position) const
{
    if (!Root) return 0.0;

    TSharedPtr<FSparseEditNode> Current = Root;
    FVector NodeCenter = Center;
    double NodeExtent = Extent;
    double Sum = 0.0;

    while (Current)
    {
        if (Current->HasEdits)
            Sum += InterpolateCorners(Current, NodeCenter, NodeExtent, Position);

        int ChildIndex = GetChildIndex(Position, NodeCenter);
        if (!Current->Children[ChildIndex]) break;

        NodeExtent *= 0.5;
        NodeCenter = NodeCenter + Offsets[ChildIndex] * NodeExtent;
        Current = Current->Children[ChildIndex];
    }

    return Sum;
}

double FSparseEditStore::InterpolateCorners(const TSharedPtr<FSparseEditNode>& Node, FVector NodeCenter, double NodeExtent, FVector Position) const
{
    double tx = (Position.X - (NodeCenter.X - NodeExtent)) / (2.0 * NodeExtent);
    double ty = (Position.Y - (NodeCenter.Y - NodeExtent)) / (2.0 * NodeExtent);
    double tz = (Position.Z - (NodeCenter.Z - NodeExtent)) / (2.0 * NodeExtent);

    tx = FMath::Clamp(tx, 0.0, 1.0);
    ty = FMath::Clamp(ty, 0.0, 1.0);
    tz = FMath::Clamp(tz, 0.0, 1.0);

    double c00 = FMath::Lerp(Node->CornerDensities[0], Node->CornerDensities[1], tx);
    double c01 = FMath::Lerp(Node->CornerDensities[2], Node->CornerDensities[3], tx);
    double c10 = FMath::Lerp(Node->CornerDensities[4], Node->CornerDensities[5], tx);
    double c11 = FMath::Lerp(Node->CornerDensities[6], Node->CornerDensities[7], tx);

    double c0 = FMath::Lerp(c00, c01, ty);
    double c1 = FMath::Lerp(c10, c11, ty);

    return FMath::Lerp(c0, c1, tz);
}

// ============================================================================
// EDIT APPLICATION (Background Thread)
// ============================================================================

TArray<FVector> FSparseEditStore::ApplySphericalEdit(FVector BrushCenter, double Radius, double Strength, int Depth)
{
    Depth = FMath::Clamp(Depth, 0, MaxDepth);
    AffectedChunkCenters.Empty();

    if (!Root)
        Root = MakeShared<FSparseEditNode>();

    ApplyEditRecursive(Root, Center, Extent, BrushCenter, Radius, Strength, 0, Depth);
    return AffectedChunkCenters;
}

void FSparseEditStore::ApplyEditRecursive(TSharedPtr<FSparseEditNode> Node, FVector NodeCenter, double NodeExtent, FVector BrushCenter, double BrushRadius, double Strength, int CurrentDepth, int TargetDepth) {
    // AABB-sphere overlap check
    FVector ClosestPoint;
    ClosestPoint.X = FMath::Clamp(BrushCenter.X, NodeCenter.X - NodeExtent, NodeCenter.X + NodeExtent);
    ClosestPoint.Y = FMath::Clamp(BrushCenter.Y, NodeCenter.Y - NodeExtent, NodeCenter.Y + NodeExtent);
    ClosestPoint.Z = FMath::Clamp(BrushCenter.Z, NodeCenter.Z - NodeExtent, NodeCenter.Z + NodeExtent);

    if (FVector::DistSquared(ClosestPoint, BrushCenter) > BrushRadius * BrushRadius)
        return;

    // Record chunk-depth nodes for affected chunk tracking
    if (CurrentDepth == ChunkDepth)
        AffectedChunkCenters.AddUnique(NodeCenter);

    // Write edit at target depth
    if (CurrentDepth == TargetDepth)
    {
        Node->HasEdits = true;
        for (int i = 0; i < 8; i++)
        {
            FVector CornerPos = GetCornerPosition(NodeCenter, NodeExtent, i);
            double Dist = FVector::Dist(CornerPos, BrushCenter);
            double Falloff = FMath::Clamp(1.0 - (Dist / BrushRadius), 0.0, 1.0);
            Falloff = Falloff * Falloff * (3.0 - 2.0 * Falloff); // Smoothstep
            Node->CornerDensities[i] += Strength * Falloff;
        }
        return;
    }

    // Recurse into children
    double ChildExtent = NodeExtent * 0.5;
    for (int i = 0; i < 8; i++)
    {
        FVector ChildCenter = NodeCenter + Offsets[i] * ChildExtent;

        FVector ChildClosest;
        ChildClosest.X = FMath::Clamp(BrushCenter.X, ChildCenter.X - ChildExtent, ChildCenter.X + ChildExtent);
        ChildClosest.Y = FMath::Clamp(BrushCenter.Y, ChildCenter.Y - ChildExtent, ChildCenter.Y + ChildExtent);
        ChildClosest.Z = FMath::Clamp(BrushCenter.Z, ChildCenter.Z - ChildExtent, ChildCenter.Z + ChildExtent);

        if (FVector::DistSquared(ChildClosest, BrushCenter) > BrushRadius * BrushRadius)
            continue;

        if (!Node->Children[i])
            Node->Children[i] = MakeShared<FSparseEditNode>();

        ApplyEditRecursive(Node->Children[i], ChildCenter, ChildExtent,
            BrushCenter, BrushRadius, Strength, CurrentDepth + 1, TargetDepth);
    }
}

// ============================================================================
// QUERIES
// ============================================================================

int FSparseEditStore::GetDepthForBrushRadius(double BrushRadius, int SubdivisionLevels) const
{
    int Depth = 0;
    double NodeExtent = Extent;
    while (Depth < MaxDepth - SubdivisionLevels && NodeExtent > BrushRadius)
    {
        NodeExtent *= 0.5;
        Depth++;
    }
    return Depth + SubdivisionLevels;
}

const TArray<FVector>& FSparseEditStore::GetAffectedChunkCenters() const
{
    return AffectedChunkCenters;
}

bool FSparseEditStore::HasEdits() const
{
    return Root.IsValid();
}

// ============================================================================
// UTILITY
// ============================================================================

int FSparseEditStore::GetChildIndex(FVector Position, FVector NodeCenter) const
{
    int Index = 0;
    if (Position.X >= NodeCenter.X) Index |= 1;
    if (Position.Y >= NodeCenter.Y) Index |= 2;
    if (Position.Z >= NodeCenter.Z) Index |= 4;
    return Index;
}

FVector FSparseEditStore::GetCornerPosition(FVector NodeCenter, double NodeExtent, int CornerIndex) const
{
    return NodeCenter + Offsets[CornerIndex] * NodeExtent;
}

void FSparseEditStore::Clear()
{
    Root.Reset();
    AffectedChunkCenters.Empty();
}