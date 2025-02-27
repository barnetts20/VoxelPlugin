#include "FAdaptiveOctreeNode.h"
#include "FSparseOctree.h"

bool FAdaptiveOctreeNode::IsLeaf()
{
    return bIsLeaf;
}

bool FAdaptiveOctreeNode::IsRoot()
{
    return !Parent.IsValid();
}

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
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]], 0);
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
    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(ChildIndex);

    for (int i = 0; i < 8; i++) {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        Corners.Add(FNodeCorner(i, CornerPosition, DensityFunction(CornerPosition)));
        Children[i] = nullptr;
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]], TreeIndex.Num());
        Edges.Add(anEdge);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    //Look up user density alterations in Sparse octree for this index? Would actually need to happen before all density evaluations.
    //Each node can calculate its own density alteration via the sparse octree stored values, as well as its composite density modification
    //By accumulating its parent node density alterations, this final value is applied to the density calculated by the density function before
   
    ComputeDualContourPosition();
}

void FAdaptiveOctreeNode::Split()
{
    if (!bIsLeaf) return; // Already split
    for (uint8 i = 0; i < 8; i++)
    {
        Children[i] = MakeShared<FAdaptiveOctreeNode>(DensityFunction, AsShared(), i);
    }
    bIsLeaf = false;
}

void FAdaptiveOctreeNode::Merge()
{
    if (bIsLeaf) return; // Already merged

    TArray<TSharedPtr<FAdaptiveOctreeNode>> NodesToDelete;
    NodesToDelete.Append(Children); // Add children to cleanup list

    while (NodesToDelete.Num() > 0)
    {
        TSharedPtr<FAdaptiveOctreeNode> Node = NodesToDelete.Pop();
        if (!Node.IsValid()) continue;

        // Add children of this node to the stack before clearing
        if (!Node->bIsLeaf) {
            NodesToDelete.Append(Node->Children);
        }
            
        // Properly clear shared pointers
        for (int i = 0; i < 8; i++) {
            Node->Children[i].Reset();
        }
        Node.Reset();
    }
    bIsLeaf = true;
}


bool FAdaptiveOctreeNode::ShouldSplit(FVector InCameraPosition, double InLodDistanceFactor)
{
    return TreeIndex.Num() < DepthBounds[0] || (FVector::Dist(DualContourPosition, InCameraPosition) < Extent * (InLodDistanceFactor + TreeIndex.Num()) && TreeIndex.Num() < DepthBounds[1]);
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector InCameraPosition, double InLodDistanceFactor)
{
    bool CanMerge = true;
    for (const auto& Child : Children)
    {
        if (!Child.IsValid() || !Child->IsLeaf())
        {
            CanMerge = false;
            break; 
        }
    }

    return CanMerge && (FVector::Dist(DualContourPosition, InCameraPosition) > Extent * (InLodDistanceFactor + TreeIndex.Num())) && TreeIndex.Num() >= DepthBounds[0];
}

bool FAdaptiveOctreeNode::UpdateLod(FVector InCameraPosition, double InLodDistanceFactor)
{
    if (IsLeaf())
    {
        if (ShouldSplit(InCameraPosition, InLodDistanceFactor))
        {
            Split();
            return true; // A split occurred
        }
        else if (Parent.IsValid() && Parent.Pin()->ShouldMerge(InCameraPosition, InLodDistanceFactor) && TreeIndex.Last() == 7)
        {
            Parent.Pin()->Merge();
            return true; // A merge occurred
        }
    }
    else
    {
        bool bAnyChanges = false;
        for (int i = 0; i < 8; i++)
        {
            if (Children[i]->UpdateLod(InCameraPosition, InLodDistanceFactor))
            {
                bAnyChanges = true;
            }
        }
        return bAnyChanges; // Return true if any child changed
    }

    return false; // No changes occurred
}

// Retrieves all surface nodes for meshing
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
            auto edges = CurrentNode->GetSignChangeEdges();
            for (auto edge : edges) {
                SurfaceEdges.AddUnique(edge);
            }
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

// Retrieves all surface nodes for meshing
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctreeNode::GetSurfaceNodes()
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> SurfaceNodes;
    TQueue<TSharedPtr<FAdaptiveOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(this->AsShared());

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode) continue;

        if (CurrentNode->IsLeaf() && CurrentNode->IsSurfaceNode)
        {
            SurfaceNodes.Add(CurrentNode->AsShared());
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (CurrentNode->Children[i]) NodeQueue.Enqueue(CurrentNode->Children[i]);
            }
        }
    }

    return SurfaceNodes;
}
//bool FAdaptiveOctreeNode::ContainsOverlappingEdge(FNodeEdge InEdgeToCheck)
//{
//    for (auto edge : Edges) {
//        if (edge.Axis != InEdgeToCheck.Axis) return false;
//        
//        if(edge.Axis == 0 && edge.ZeroCrossingPoint.Y == InEdgeToCheck.ZeroCrossingPoint.Y )
//    }
//    return true;
//}
//TODO: Lets try converting this lod method
//void QuadTreeNode::TrySetLod() {
//    if (this->IsInitialized && this->IsLeaf()) {
//        double k = 8;
//        double fov = this->ParentActor->GetCameraFOV();
//        FVector lastCamPos = this->ParentActor->GetLastCameraPosition();
//        auto lastCamRot = this->ParentActor->GetLastCameraRotation();
//
//        //Since we are doing origin rebasing frequently, the actors location can "change" arbitrarily and needs to be accounted for
//        FVector planetCenter = this->ParentActor->GetActorLocation();
//        FVector adjustedCentroid = this->NodeCentroid * this->ParentActor->GetActorScale().X + planetCenter;
//        auto parentCenter = adjustedCentroid;
//        auto parentSize = this->MaxNodeRadius * this->ParentActor->GetActorScale().X;
//
//        double planetRadius = FVector::Distance(this->ParentActor->GetActorLocation(), adjustedCentroid);
//
//        auto parent = this->Parent;
//
//        if (parent.IsValid()) {
//            parentCenter = parent.Pin()->NodeCentroid * this->ParentActor->GetActorScale().X + planetCenter;
//            parentSize = parent.Pin()->MaxNodeRadius * this->ParentActor->GetActorScale().X;
//        }
//
//        double d1 = FVector::Distance(lastCamPos, adjustedCentroid);
//        double d2 = FVector::Distance(lastCamPos, parentCenter);
//        if (this->GetDepth() < this->MinDepth || (this->GetDepth() < this->MaxDepth && k * this->MaxNodeRadius * this->ParentActor->GetActorScale().X > s(d1, fov))) {
//            this->CanMerge = false;
//            if (this->LastRenderedState) {
//                this->AsyncSplit(this->AsShared());
//            }
//        }
//        else if ((parent.IsValid() && parent.Pin()->GetDepth() >= this->MinDepth) && k * parentSize < s(d2, fov)) {
//            this->CanMerge = true;
//            parent.Pin()->TryMerge();
//        }
//        else {
//            this->CanMerge = false;
//        }
//
//    }
//}

TArray<FNodeEdge>& FAdaptiveOctreeNode::GetSignChangeEdges()
{
    return SignChangeEdges;
}

// Compute Dual Contour Position
void FAdaptiveOctreeNode::ComputeDualContourPosition()
{
    TArray<FVector> SurfaceCrossings;
    for (FNodeEdge anEdge : SignChangeEdges) {
        SurfaceCrossings.Add(anEdge.ZeroCrossingPoint);
    }

    if (SignChangeEdges.Num() > 0) {
        DualContourPosition = Algo::Accumulate(SurfaceCrossings, FVector::ZeroVector) / SurfaceCrossings.Num();
        
        DualContourNormal = FVector(0, 0, 0);
        DualContourNormal.X = (Corners[1].Density + Corners[3].Density + Corners[5].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[2].Density + Corners[4].Density + Corners[6].Density);
        DualContourNormal.Y = (Corners[2].Density + Corners[3].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[4].Density + Corners[5].Density);
        DualContourNormal.Z = (Corners[4].Density + Corners[5].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[2].Density + Corners[3].Density);

        if (!DualContourNormal.IsNearlyZero(KINDA_SMALL_NUMBER))
        {
            DualContourNormal.Normalize();
        }

        IsSurfaceNode = true;
    }
    else {
        DualContourNormal = FVector(0, 0, 0);
        DualContourPosition = Center;
        IsSurfaceNode = false;
    }
}
