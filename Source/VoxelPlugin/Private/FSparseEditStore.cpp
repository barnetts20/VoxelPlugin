#include "FSparseEditStore.h"
#include "Async/ParallelFor.h"
#include "Misc/ScopeLock.h"

namespace
{
    /** Child (or corner) c's direction from a node's centre: bit 0 +X, bit 1 +Y, bit 2 +Z. */
    const FVector EditChildOffsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1), FVector(-1, 1, -1), FVector(1, 1, -1),
        FVector(-1, -1, 1), FVector(1, -1, 1), FVector(-1, 1, 1), FVector(1, 1, 1) };
}

FSparseEditStore::FSparseEditStore(FVector InCenter, double InExtent, int InMaxDepth)
    : Center(InCenter), Extent(InExtent), MaxDepth(InMaxDepth) {}

TSharedPtr<FSparseEditNode> FSparseEditStore::PinRoot() const
{
    FScopeLock Lock(&RootCS);
    return Root;
}

double FSparseEditStore::Sample(FVector Position) const
{
    const TSharedPtr<FSparseEditNode> Tree = PinRoot();
    return SampleTree(Tree.Get(), Position);
}

void FSparseEditStore::SampleAdd(const double* X, const double* Y, const double* Z, int32 Count, float* DensityInOut) const
{
    if (Count <= 0) return;

    const TSharedPtr<FSparseEditNode> Tree = PinRoot();
    const FSparseEditNode* TreeRoot = Tree.Get();
    if (!TreeRoot) return;

    if (Count > 128)
    {
        ParallelFor(Count, [&](int32 i) {
            DensityInOut[i] += (float)SampleTree(TreeRoot, FVector(X[i], Y[i], Z[i]));
            });
    }
    else
    {
        for (int32 i = 0; i < Count; i++)
            DensityInOut[i] += (float)SampleTree(TreeRoot, FVector(X[i], Y[i], Z[i]));
    }
}

double FSparseEditStore::SampleTree(const FSparseEditNode* TreeRoot, const FVector& Position) const
{
    // Raw pointers: the caller pins the tree, and published trees never change. Copying
    // shared pointers here would contend on the root's reference count across workers.
    const FSparseEditNode* Current = TreeRoot;
    FVector NodeCenter = Center;
    double NodeExtent = Extent;
    double Sum = 0.0;

    while (Current)
    {
        if (Current->HasEdits)
            Sum += InterpolateCorners(*Current, NodeCenter, NodeExtent, Position);

        int ChildIndex = GetChildIndex(Position, NodeCenter);
        const FSparseEditNode* Child = Current->Children[ChildIndex].Get();
        if (!Child) break;

        NodeExtent *= 0.5;
        NodeCenter = NodeCenter + EditChildOffsets[ChildIndex] * NodeExtent;
        Current = Child;
    }

    return Sum;
}

bool FSparseEditStore::HasEditsInBox(const FBox& Box) const
{
    const TSharedPtr<FSparseEditNode> Tree = PinRoot();
    return HasEditsInBoxRecursive(Tree.Get(), Center, Extent, Box);
}

bool FSparseEditStore::HasEditsInBoxRecursive(const FSparseEditNode* Node, const FVector& NodeCenter, double NodeExtent, const FBox& Box)
{
    if (!Node) return false;
    if (Box.Min.X > NodeCenter.X + NodeExtent || Box.Max.X < NodeCenter.X - NodeExtent
        || Box.Min.Y > NodeCenter.Y + NodeExtent || Box.Max.Y < NodeCenter.Y - NodeExtent
        || Box.Min.Z > NodeCenter.Z + NodeExtent || Box.Max.Z < NodeCenter.Z - NodeExtent)
        return false;
    if (Node->HasEdits) return true;

    const double ChildExtent = NodeExtent * 0.5;
    for (int i = 0; i < 8; i++)
    {
        if (HasEditsInBoxRecursive(Node->Children[i].Get(), NodeCenter + EditChildOffsets[i] * ChildExtent, ChildExtent, Box))
            return true;
    }
    return false;
}

void FSparseEditStore::ApplySphericalEdit(FVector BrushCenter, double Radius, double Strength, int Depth)
{
    FVoxelEditOp Op;
    Op.Center = BrushCenter;
    Op.Radius = Radius;
    Op.Strength = Strength;
    Op.Depth = FMath::Clamp(Depth, 0, MaxDepth);

    FScopeLock WriteLock(&WriteCS);
    Op.Sequence = NextSequence++;
    EditLog.Add(Op);
    ApplyOpLocked(Op);
    LastSequence.store(Op.Sequence, std::memory_order_release);
}

void FSparseEditStore::ApplyOpLocked(const FVoxelEditOp& Op)
{
    const TSharedPtr<FSparseEditNode> OldRoot = PinRoot();
    TSharedPtr<FSparseEditNode> NewRoot = OldRoot.IsValid()
        ? MakeShared<FSparseEditNode>(*OldRoot)
        : MakeShared<FSparseEditNode>();

    ApplyEditRecursive(*NewRoot, Center, Extent, Op.Center, Op.Radius, Op.Strength, 0, FMath::Clamp(Op.Depth, 0, MaxDepth));

    FScopeLock Lock(&RootCS);
    Root = NewRoot;
}

TArray<FVoxelEditOp> FSparseEditStore::GetEditLog() const
{
    FScopeLock WriteLock(&WriteCS);
    return EditLog;
}

void FSparseEditStore::ReplayEditLog(const TArray<FVoxelEditOp>& Ops)
{
    FScopeLock WriteLock(&WriteCS);
    {
        FScopeLock Lock(&RootCS);
        Root.Reset();
    }
    EditLog.Reset();
    NextSequence = 1;
    for (const FVoxelEditOp& Op : Ops)
    {
        EditLog.Add(Op);
        NextSequence = FMath::Max(NextSequence, Op.Sequence + 1);
        ApplyOpLocked(Op);
    }
    LastSequence.store(NextSequence - 1, std::memory_order_release);
}

void FSparseEditStore::ApplyEditRecursive(FSparseEditNode& Node, FVector NodeCenter, double NodeExtent,
    FVector BrushCenter, double BrushRadius, double Strength,
    int CurrentDepth, int TargetDepth)
{
    // AABB-sphere overlap check
    FVector ClosestPoint;
    ClosestPoint.X = FMath::Clamp(BrushCenter.X, NodeCenter.X - NodeExtent, NodeCenter.X + NodeExtent);
    ClosestPoint.Y = FMath::Clamp(BrushCenter.Y, NodeCenter.Y - NodeExtent, NodeCenter.Y + NodeExtent);
    ClosestPoint.Z = FMath::Clamp(BrushCenter.Z, NodeCenter.Z - NodeExtent, NodeCenter.Z + NodeExtent);

    if (FVector::DistSquared(ClosestPoint, BrushCenter) > BrushRadius * BrushRadius)
        return;

    // Write edit at target depth
    if (CurrentDepth == TargetDepth)
    {
        Node.HasEdits = true;
        for (int i = 0; i < 8; i++)
        {
            FVector CornerPos = GetCornerPosition(NodeCenter, NodeExtent, i);
            double Dist = FVector::Dist(CornerPos, BrushCenter);
            double Falloff = FMath::Clamp(1.0 - (Dist / BrushRadius), 0.0, 1.0);
            Falloff = Falloff * Falloff * (3.0 - 2.0 * Falloff); // Smoothstep
            Node.CornerDensities[i] += Strength * Falloff;
        }
        return;
    }

    // Recurse into children
    double ChildExtent = NodeExtent * 0.5;
    for (int i = 0; i < 8; i++)
    {
        FVector ChildCenter = NodeCenter + EditChildOffsets[i] * ChildExtent;

        FVector ChildClosest;
        ChildClosest.X = FMath::Clamp(BrushCenter.X, ChildCenter.X - ChildExtent, ChildCenter.X + ChildExtent);
        ChildClosest.Y = FMath::Clamp(BrushCenter.Y, ChildCenter.Y - ChildExtent, ChildCenter.Y + ChildExtent);
        ChildClosest.Z = FMath::Clamp(BrushCenter.Z, ChildCenter.Z - ChildExtent, ChildCenter.Z + ChildExtent);

        if (FVector::DistSquared(ChildClosest, BrushCenter) > BrushRadius * BrushRadius)
            continue;

        // The child may belong to a published tree: write to a copy.
        Node.Children[i] = Node.Children[i].IsValid()
            ? MakeShared<FSparseEditNode>(*Node.Children[i])
            : MakeShared<FSparseEditNode>();

        ApplyEditRecursive(*Node.Children[i], ChildCenter, ChildExtent,
            BrushCenter, BrushRadius, Strength, CurrentDepth + 1, TargetDepth);
    }
}

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
    return NodeCenter + EditChildOffsets[CornerIndex] * NodeExtent;
}

double FSparseEditStore::InterpolateCorners(const FSparseEditNode& Node, FVector NodeCenter, double NodeExtent, FVector Position) const
{
    double tx = (Position.X - (NodeCenter.X - NodeExtent)) / (2.0 * NodeExtent);
    double ty = (Position.Y - (NodeCenter.Y - NodeExtent)) / (2.0 * NodeExtent);
    double tz = (Position.Z - (NodeCenter.Z - NodeExtent)) / (2.0 * NodeExtent);

    tx = FMath::Clamp(tx, 0.0, 1.0);
    ty = FMath::Clamp(ty, 0.0, 1.0);
    tz = FMath::Clamp(tz, 0.0, 1.0);

    double c00 = FMath::Lerp(Node.CornerDensities[0], Node.CornerDensities[1], tx);
    double c01 = FMath::Lerp(Node.CornerDensities[2], Node.CornerDensities[3], tx);
    double c10 = FMath::Lerp(Node.CornerDensities[4], Node.CornerDensities[5], tx);
    double c11 = FMath::Lerp(Node.CornerDensities[6], Node.CornerDensities[7], tx);

    double c0 = FMath::Lerp(c00, c01, ty);
    double c1 = FMath::Lerp(c10, c11, ty);

    return FMath::Lerp(c0, c1, tz);
}

void FSparseEditStore::Clear()
{
    FScopeLock WriteLock(&WriteCS);
    EditLog.Reset();
    NextSequence = 1;
    LastSequence.store(0, std::memory_order_release);
    FScopeLock Lock(&RootCS);
    Root.Reset();
}
