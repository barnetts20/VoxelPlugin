#include "FAdaptiveOctreeNode.h"

bool FAdaptiveOctreeNode::IsLeaf() { return bIsLeaf; }
bool FAdaptiveOctreeNode::IsRoot() { return !Parent.IsValid(); }

// Root Constructor
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InExtent, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    Center = InCenter;
    Extent = FMath::Max(InExtent, 0.0);
    Density = 0.0;
    DepthBounds[0] = InMinDepth;
    DepthBounds[1] = InMaxDepth;

    for (int i = 0; i < 8; i++) {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        Corners.Add(FNodeCorner(i, CornerPosition, DensityFunction(CornerPosition)));
        Children[i] = nullptr;
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]]);
        Edges.Add(anEdge);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    ComputeDualContourPosition();
}

// Child Constructor
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex)
{
    Parent = InParent;
    DensityFunction = InDensityFunction;
    Center = InParent->Center + Offsets[ChildIndex] * (InParent->Extent * 0.5);
    Extent = InParent->Extent * 0.5;
    Density = 0.0;
    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];

    for (int i = 0; i < 8; i++) {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        Corners.Add(FNodeCorner(i, CornerPosition, DensityFunction(CornerPosition)));
        Children[i] = nullptr;
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]]);
        Edges.Add(anEdge);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(ChildIndex);

    ComputeDualContourPosition();
}

void FAdaptiveOctreeNode::Split()
{
    if (!bIsLeaf) return;
    for (uint8 i = 0; i < 8; i++)
    {
        Children[i] = MakeShared<FAdaptiveOctreeNode>(DensityFunction, AsShared(), i);

        if (Children[i]->IsSurfaceNode)
        {
            TSharedPtr<FAdaptiveOctreeNode> Ancestor = AsShared();
            while (Ancestor.IsValid() && !Ancestor->IsSurfaceNode)
            {
                Ancestor->IsSurfaceNode = true;
                Ancestor = Ancestor->Parent.Pin();
            }
        }
    }
    bIsLeaf = false;
}

void FAdaptiveOctreeNode::Merge()
{
    if (bIsLeaf) return;

    TArray<TSharedPtr<FAdaptiveOctreeNode>> NodesToDelete;
    NodesToDelete.Append(Children);

    while (NodesToDelete.Num() > 0)
    {
        TSharedPtr<FAdaptiveOctreeNode> Node = NodesToDelete.Pop();
        if (!Node.IsValid()) continue;

        if (!Node->bIsLeaf) {
            NodesToDelete.Append(Node->Children);
        }

        for (int i = 0; i < 8; i++) {
            Node->Children[i].Reset();
        }
        Node.Reset();
    }
    bIsLeaf = true;
}

bool FAdaptiveOctreeNode::ShouldSplit(FVector InCameraPosition, double InLodDistanceFactor)
{
    return TreeIndex.Num() < DepthBounds[0]
        || (FVector::Dist(DualContourPosition, InCameraPosition) < Extent * (InLodDistanceFactor + TreeIndex.Num())
            && TreeIndex.Num() < DepthBounds[1]);
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector InCameraPosition, double InLodDistanceFactor)
{
    if (LodOverride) return false;
    return (FVector::Dist(DualContourPosition, InCameraPosition) > Extent * (InLodDistanceFactor + TreeIndex.Num()))
        && TreeIndex.Num() >= DepthBounds[0];
}

void AppendUniqueEdges(TArray<FNodeEdge> InAppendEdges, TArray<FNodeEdge>& OutNodeEdges) {
    for (FNodeEdge& anEdge : InAppendEdges) {
        bool found = false;
        for (int i = 0; i < OutNodeEdges.Num(); i++) {
            if (OutNodeEdges[i] == anEdge) {
                if (anEdge.Distance < OutNodeEdges[i].Distance) {
                    OutNodeEdges[i] = anEdge;
                }
                found = true;
                break;
            }
        }
        if (!found) {
            OutNodeEdges.Add(anEdge);
        }
    }
}

void FAdaptiveOctreeNode::UpdateLod(FVector InCameraPosition, double InLodDistanceFactor, TArray<FNodeEdge>& OutNodeEdges, bool& OutChanged)
{
    if (IsLeaf())
    {
        if (ShouldSplit(InCameraPosition, InLodDistanceFactor))
        {
            OutChanged = true;
            Split();
            // One level per frame — don't recurse into new children
            for (TSharedPtr<FAdaptiveOctreeNode> aChild : Children) {
                AppendUniqueEdges(aChild->GetSignChangeEdges(), OutNodeEdges);
            }
            return;
        }
        else if (Parent.IsValid() && Parent.Pin()->ShouldMerge(InCameraPosition, InLodDistanceFactor) && TreeIndex.Last() == 7)
        {
            OutChanged = true;
            auto parentPtr = Parent.Pin();
            parentPtr->Merge();
            AppendUniqueEdges(parentPtr->GetSignChangeEdges(), OutNodeEdges);
            return;
        }
        else
        {
            AppendUniqueEdges(GetSignChangeEdges(), OutNodeEdges);
            return;
        }
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            Children[i]->UpdateLod(InCameraPosition, InLodDistanceFactor, OutNodeEdges, OutChanged);
        }
    }
}

TArray<FNodeEdge> FAdaptiveOctreeNode::GetSurfaceEdges()
{
    TArray<FNodeEdge> SurfaceEdges;
    TQueue<TSharedPtr<FAdaptiveOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(this->AsShared());

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);
        if (!CurrentNode) continue;

        if (CurrentNode->IsLeaf() && CurrentNode->IsSurfaceNode)
        {
            AppendUniqueEdges(CurrentNode->GetSignChangeEdges(), SurfaceEdges);
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (CurrentNode->Children[i]) NodeQueue.Enqueue(CurrentNode->Children[i]);
            }
        }
    }

    return SurfaceEdges;
}

TArray<FNodeEdge> FAdaptiveOctreeNode::GetSignChangeEdges()
{
    return SignChangeEdges;
}

void FAdaptiveOctreeNode::ComputeDualContourPosition()
{
    if (SignChangeEdges.Num() == 0)
    {
        DualContourNormal = FVector::ZeroVector;
        DualContourPosition = Center;
        IsSurfaceNode = false;
        return;
    }

    IsSurfaceNode = true;

    FQEF Qef;
    double h = Extent * 0.01;

    for (FNodeEdge& Edge : SignChangeEdges)
    {
        FVector P = Edge.ZeroCrossingPoint;

        double dx = DensityFunction(P + FVector(h, 0, 0)) - DensityFunction(P - FVector(h, 0, 0));
        double dy = DensityFunction(P + FVector(0, h, 0)) - DensityFunction(P - FVector(0, h, 0));
        double dz = DensityFunction(P + FVector(0, 0, h)) - DensityFunction(P - FVector(0, 0, h));

        FVector Normal = FVector(dx, dy, dz).GetSafeNormal();
        if (Normal.IsNearlyZero()) continue;

        Qef.AddPlane(P, Normal);
    }

    double Error = 0.0;
    DualContourPosition = Qef.Solve(Center, Extent, &Error);

    double ndx = DensityFunction(DualContourPosition + FVector(h, 0, 0)) - DensityFunction(DualContourPosition - FVector(h, 0, 0));
    double ndy = DensityFunction(DualContourPosition + FVector(0, h, 0)) - DensityFunction(DualContourPosition - FVector(0, h, 0));
    double ndz = DensityFunction(DualContourPosition + FVector(0, 0, h)) - DensityFunction(DualContourPosition - FVector(0, 0, h));

    DualContourNormal = FVector(ndx, ndy, ndz).GetSafeNormal();

    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal.X = (Corners[1].Density + Corners[3].Density + Corners[5].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[2].Density + Corners[4].Density + Corners[6].Density);
        DualContourNormal.Y = (Corners[2].Density + Corners[3].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[4].Density + Corners[5].Density);
        DualContourNormal.Z = (Corners[4].Density + Corners[5].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[2].Density + Corners[3].Density);
        DualContourNormal.Normalize();
    }
}

void FAdaptiveOctreeNode::DrawAndLogNode()
{
    FVector NodeCenter = Center;
    double NodeExtent = Extent;
    FVector DCPosition = DualContourPosition;

    AsyncTask(ENamedThreads::GameThread, [NodeCenter, NodeExtent, DCPosition]() {
        UWorld* World = GEngine->GetWorldContexts()[0].World();
        if (!World) return;
        DrawDebugBox(World, NodeCenter, FVector(NodeExtent), FQuat::Identity, FColor::Green, true, 1000.0f, 0, 50000.0f);
        DrawDebugPoint(World, DCPosition, 10.0, FColor::Red, true, 1000.0f, 0);
        });
}