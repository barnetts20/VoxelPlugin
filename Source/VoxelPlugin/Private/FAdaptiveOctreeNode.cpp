#include "FAdaptiveOctreeNode.h"

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
    if (LodOverride) return false;
    //if (!CanMergeBalanced()) return false;  // <-- add this
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
            BalancedSplit();
            if (fullUpdate) {
                for (TSharedPtr<FAdaptiveOctreeNode> aChild : Children) {
                    aChild->UpdateLod(InCameraPosition, InLodDistanceFactor, OutNodeEdges, OutChanged);
                }
            }
            else {
                for (TSharedPtr<FAdaptiveOctreeNode> aChild : Children) {
                    AppendUniqueEdges(aChild->GetSignChangeEdges(), OutNodeEdges);
                }
                return;
            }

        }
        else if (Parent.IsValid() && Parent.Pin()->ShouldMerge(InCameraPosition, InLodDistanceFactor) && TreeIndex.Last() == 7)
        {
            OutChanged = true;
            auto parentPtr = Parent.Pin();
            parentPtr->Merge();
            if (fullUpdate) {
                parentPtr->UpdateLod(InCameraPosition, InLodDistanceFactor, OutNodeEdges, OutChanged);
            }
            else {
                AppendUniqueEdges(parentPtr->GetSignChangeEdges(), OutNodeEdges);
                return;
            }
            
        }
        else {
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

TArray<FNodeEdge> FAdaptiveOctreeNode::GetSignChangeEdges()
{
    //TArray<FNodeEdge> returnEdges;
    //for (int i = 0; i < 6; i++) {
    //    if (Edges[i].SignChange) returnEdges.Add(Edges[i]);
    //}
    //return returnEdges;
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

    // Build QEF from sign-change edges.
    // Each edge contributes a plane: (zero-crossing point, surface normal at that point).
    FQEF Qef;

    // Step size for finite-difference normal estimation — scales with cell size
    double h = Extent * 0.01;

    for (FNodeEdge& Edge : SignChangeEdges)
    {
        FVector P = Edge.ZeroCrossingPoint;

        // Estimate surface normal at the zero-crossing via central differences
        // on the density function. This gives much better normals than the 
        // discrete corner gradient, especially for curved surfaces.
        double dx = DensityFunction(P + FVector(h, 0, 0)) - DensityFunction(P - FVector(h, 0, 0));
        double dy = DensityFunction(P + FVector(0, h, 0)) - DensityFunction(P - FVector(0, h, 0));
        double dz = DensityFunction(P + FVector(0, 0, h)) - DensityFunction(P - FVector(0, 0, h));

        FVector Normal = FVector(dx, dy, dz).GetSafeNormal();

        // If normal estimation failed (e.g., density is constant), skip this plane
        if (Normal.IsNearlyZero()) continue;

        Qef.AddPlane(P, Normal);
    }

    // Solve QEF — result is clamped to cell bounds internally
    double Error = 0.0;
    DualContourPosition = Qef.Solve(Center, Extent, &Error);

    // Compute the node normal as the average of the edge normals
    // (or we could use the gradient at the DC position itself)
    double ndx = DensityFunction(DualContourPosition + FVector(h, 0, 0)) - DensityFunction(DualContourPosition - FVector(h, 0, 0));
    double ndy = DensityFunction(DualContourPosition + FVector(0, h, 0)) - DensityFunction(DualContourPosition - FVector(0, h, 0));
    double ndz = DensityFunction(DualContourPosition + FVector(0, 0, h)) - DensityFunction(DualContourPosition - FVector(0, 0, h));

    DualContourNormal = FVector(ndx, ndy, ndz).GetSafeNormal();

    // Fallback: if normal is zero (flat region), use the discrete corner gradient
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
    // Capture necessary information for drawing
    FVector NodeCenter = Center;
    double NodeExtent = Extent;
    FVector DCPosition = DualContourPosition;
    TArray<FNodeCorner> NodeCorners = Corners;
    int32 TreeDepth = TreeIndex.Num();

    // Dispatch drawing task to the game thread
    AsyncTask(ENamedThreads::GameThread, [NodeCenter, NodeExtent, DCPosition, NodeCorners, TreeDepth]() {
        // Get world for drawing debug shapes
        UWorld* World = GEngine->GetWorldContexts()[0].World();
        if (!World) return;

        // Set duration and color based on tree depth
        float Duration = 1.0f; // 1 second display
        FColor BoxColor = FColor::Green;

        // Calculate the min/max corners of the box
        FVector Min = NodeCenter - FVector(NodeExtent);
        FVector Max = NodeCenter + FVector(NodeExtent);

        // Draw the bounding box
        DrawDebugBox(World, NodeCenter, FVector(NodeExtent), FQuat::Identity, BoxColor, true, Duration, 0, 5000.0f);

        // Draw the dual contour position
        DrawDebugPoint(World, DCPosition, 10.0, FColor::Red, true, Duration, 0);

        // Log information about the node
        UE_LOG(LogTemp, Display, TEXT("Node at depth %d:"), TreeDepth);
        UE_LOG(LogTemp, Display, TEXT("  Center: (%f, %f, %f)"), NodeCenter.X, NodeCenter.Y, NodeCenter.Z);
        UE_LOG(LogTemp, Display, TEXT("  Extent: %f"), NodeExtent);
        UE_LOG(LogTemp, Display, TEXT("  Dual Contour Position: (%f, %f, %f)"),
            DCPosition.X, DCPosition.Y, DCPosition.Z);

        // Log corner information
        for (int i = 0; i < NodeCorners.Num(); i++) {
            UE_LOG(LogTemp, Display, TEXT("  Corner %d: Pos=(%f, %f, %f), Density=%f"),
                i, NodeCorners[i].Position.X, NodeCorners[i].Position.Y, NodeCorners[i].Position.Z,
                NodeCorners[i].Density);
        }
        });
}

TSharedPtr<FAdaptiveOctreeNode> FAdaptiveOctreeNode::FindNeighbor(int Direction)
{
    // If we're root, no neighbor in this direction
    if (!Parent.IsValid()) return nullptr;

    auto ParentPtr = Parent.Pin();
    if (!ParentPtr.IsValid()) return nullptr;

    // What child index are we?
    uint8 MyChildIndex = TreeIndex.Last();

    // Check if the neighbor is a sibling (shares the same parent)
    // A child is on the +X face if bit 0 is set, -X if bit 0 is clear, etc.
    // Direction 0 (+X): we need bit 0 to be 0 (so mirror is within same parent)
    // Direction 1 (-X): we need bit 0 to be 1

    int AxisBit = Direction / 2;  // 0 for X, 1 for Y, 2 for Z
    bool PositiveDir = (Direction % 2 == 0);
    bool ChildOnPositiveSide = (MyChildIndex & (1 << AxisBit)) != 0;

    if (PositiveDir != ChildOnPositiveSide) {
        // Neighbor is a sibling — just flip the relevant bit
        uint8 SiblingIndex = MyChildIndex ^ (1 << AxisBit);
        return ParentPtr->Children[SiblingIndex];
    }

    // Neighbor is NOT a sibling — go up to parent and find parent's neighbor
    TSharedPtr<FAdaptiveOctreeNode> ParentNeighbor = ParentPtr->FindNeighbor(Direction);

    if (!ParentNeighbor.IsValid()) return nullptr;

    // If parent's neighbor is a leaf, it IS the neighbor (larger cell)
    if (ParentNeighbor->IsLeaf()) return ParentNeighbor;

    // Otherwise, descend into parent's neighbor to find the adjacent child
    // We want the child on the opposite face
    uint8 MirroredChild = MirrorChild[Direction][MyChildIndex];
    return ParentNeighbor->Children[MirroredChild];
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctreeNode::GetFaceNeighborLeaves(int Direction)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Result;

    TSharedPtr<FAdaptiveOctreeNode> Neighbor = FindNeighbor(Direction);
    if (!Neighbor.IsValid()) return Result;

    // Collect all leaf descendants of Neighbor that touch our face
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Stack;
    Stack.Push(Neighbor);

    while (Stack.Num() > 0) {
        auto Current = Stack.Pop();
        if (!Current.IsValid()) continue;

        if (Current->IsLeaf()) {
            Result.Add(Current);
        }
        else {
            // Only descend into children that are on the face adjacent to us
            int OppDir = OppositeDir[Direction];
            for (int i = 0; i < 4; i++) {
                int ChildIdx = FaceChildren[OppDir][i];
                if (Current->Children[ChildIdx].IsValid()) {
                    Stack.Push(Current->Children[ChildIdx]);
                }
            }
        }
    }

    return Result;
}

void FAdaptiveOctreeNode::BalancedSplit()
{
    if (!bIsLeaf) return;

    // Split ourselves first
    Split();

    // Now check all 6 face neighbors
    // If any neighbor is a leaf AND coarser than us (same depth = they'd be
    // 1 level coarser than our new children), force-split them too
    for (int Dir = 0; Dir < 6; Dir++) {
        TSharedPtr<FAdaptiveOctreeNode> Neighbor = FindNeighbor(Dir);

        if (Neighbor.IsValid() && Neighbor->IsLeaf()) {
            // Neighbor is at our depth (same level as us pre-split).
            // Our children are now 1 level deeper than neighbor — that's fine (diff = 1).
            // But if our CHILDREN split later, they'd need neighbor to also be split.
            // We don't need to do anything here — the constraint is checked when 
            // children try to split.

            // However, if neighbor is coarser than us (fewer TreeIndex entries),
            // our children would be 2+ levels finer — must split neighbor.
            if (Neighbor->TreeIndex.Num() < TreeIndex.Num()) {
                // Recursive balanced split on the coarser neighbor
                Neighbor->Split();
            }
        }
    }
}

bool FAdaptiveOctreeNode::CanMergeBalanced()
{
    // Standard merge checks
    if (bIsLeaf) return false;

    for (const auto& Child : Children) {
        if (!Child.IsValid() || !Child->IsLeaf()) return false;
    }

    // Balance check: after merging, we'd be a leaf at our depth.
    // Check that no face-adjacent leaf is 2+ levels deeper than us.
    for (int Dir = 0; Dir < 6; Dir++) {
        TArray<TSharedPtr<FAdaptiveOctreeNode>> NeighborLeaves = GetFaceNeighborLeaves(Dir);
        for (auto& NLeaf : NeighborLeaves) {
            // If any neighbor leaf is 2+ levels deeper than we would be after merge,
            // merging would violate the balance constraint
            if (NLeaf->TreeIndex.Num() > TreeIndex.Num() + 1) {
                return false;
            }
        }
    }

    return true;
}