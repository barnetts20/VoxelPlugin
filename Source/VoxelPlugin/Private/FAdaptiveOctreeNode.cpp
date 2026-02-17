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
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TFunction<double(FVector, FVector)>* InDensityFunction, FVector InCenter, double InExtent, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    Center = InCenter;
    AnchorCenter = InCenter;
    Extent = FMath::Max(InExtent, 0.0);

    DepthBounds[0] = InMinDepth;
    DepthBounds[1] = InMaxDepth;

    // Use a conservative h for the root
    double h = 0.1 * Extent;

    for (int i = 0; i < 8; i++) {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        double d = (*DensityFunction)(CornerPosition, AnchorCenter);

        // Forward difference is fine for the root as it's usually empty/air
        double dx = (*DensityFunction)(CornerPosition + FVector(h, 0, 0), AnchorCenter) - d;
        double dy = (*DensityFunction)(CornerPosition + FVector(0, h, 0), AnchorCenter) - d;
        double dz = (*DensityFunction)(CornerPosition + FVector(0, 0, h), AnchorCenter) - d;

        FVector grad(dx, dy, dz);
        if (!grad.Normalize()) { grad = FVector::UpVector; }

        Corners[i] = FNodeCorner(CornerPosition, d, grad);
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]]);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    ComputeDualContourPosition();
}

// Child Constructor
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TFunction<double(FVector, FVector)>* InDensityFunction, TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex, FVector InAnchorCenter)
{
    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(ChildIndex);

    Parent = InParent;
    DensityFunction = InDensityFunction;
    AnchorCenter = InAnchorCenter;

    // Set spatial bounds first
    Extent = InParent->Extent * 0.5;
    Center = InParent->Center + Offsets[ChildIndex] * Extent;

    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];

    // DERIVE CHUNK EXTENT: Using the TreeIndex depth relative to ChunkDepth
    // ChunkDepth should be a member or accessible variable (e.g., 5)
    double ChunkExtent = Extent * FMath::Pow(2.0, (double)(TreeIndex.Num() - ChunkDepth));

    // STABLE GRADIENT STEP: Node detail (10%) but clamped to Chunk stability (1e-6)
    double h = FMath::Max(0.1 * Extent, ChunkExtent * 2 * 1e-6);

    for (int i = 0; i < 8; i++)
    {
        FVector CornerPos = Center + (Offsets[i] * Extent);
        double d = (*DensityFunction)(CornerPos, AnchorCenter);

        // CENTRAL DIFFERENCE: Much smoother for planetary lighting
        double dx = (*DensityFunction)(CornerPos + FVector(h, 0, 0), AnchorCenter) -
            (*DensityFunction)(CornerPos - FVector(h, 0, 0), AnchorCenter);
        double dy = (*DensityFunction)(CornerPos + FVector(0, h, 0), AnchorCenter) -
            (*DensityFunction)(CornerPos - FVector(0, h, 0), AnchorCenter);
        double dz = (*DensityFunction)(CornerPos + FVector(0, 0, h), AnchorCenter) -
            (*DensityFunction)(CornerPos - FVector(0, 0, h), AnchorCenter);

        FVector Normal(dx, dy, dz);
        if (!Normal.Normalize()) { Normal = FVector::UpVector; }

        Corners[i] = FNodeCorner(CornerPos, d, Normal);
    }

    for (int i = 0; i < 12; i++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[i][0]], Corners[EdgePairs[i][1]]);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    ComputeDualContourPosition();
}

FVector FAdaptiveOctreeNode::GetInterpolatedNormal(FVector P)
{
    // Normalize P to [0, 1] range within the cell
    double tx = (P.X - (Center.X - Extent)) / (2.0 * Extent);
    double ty = (P.Y - (Center.Y - Extent)) / (2.0 * Extent);
    double tz = (P.Z - (Center.Z - Extent)) / (2.0 * Extent);

    tx = FMath::Clamp(tx, 0.0, 1.0);
    ty = FMath::Clamp(ty, 0.0, 1.0);
    tz = FMath::Clamp(tz, 0.0, 1.0);

    // Trilinear interpolation of the 8 corner normals
    FVector n00 = FMath::Lerp(Corners[0].Normal, Corners[1].Normal, tx);
    FVector n01 = FMath::Lerp(Corners[2].Normal, Corners[3].Normal, tx);
    FVector n10 = FMath::Lerp(Corners[4].Normal, Corners[5].Normal, tx);
    FVector n11 = FMath::Lerp(Corners[6].Normal, Corners[7].Normal, tx);

    FVector n0 = FMath::Lerp(n00, n01, ty);
    FVector n1 = FMath::Lerp(n10, n11, ty);

    FVector FinalNormal = FMath::Lerp(n0, n1, tz);
    return FinalNormal.GetSafeNormal();
}

void FAdaptiveOctreeNode::Split()
{
    if (!bIsLeaf) return;

    // Determine the anchor for children
    // If THIS node is at the ChunkDepth, its children will use its center as their anchor.
    // Otherwise, they inherit the current anchor.
    FVector NextAnchor = (TreeIndex.Num() == ChunkDepth) ? Center : AnchorCenter;

    for (uint8 i = 0; i < 8; i++)
    {
        // Update constructor to take NextAnchor
        Children[i] = MakeShared<FAdaptiveOctreeNode>(DensityFunction, AsShared(), i, NextAnchor);

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

void AppendUniqueEdges(const TArray<FNodeEdge>& InAppendEdges, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap)
{
    for (const FNodeEdge& anEdge : InAppendEdges)
    {
        FEdgeKey Key(anEdge);
        int32* ExistingIdx = EdgeMap.Find(Key);

        if (ExistingIdx)
        {
            // Keep the one with the smaller Distance (same logic as before)
            if (anEdge.Distance < OutNodeEdges[*ExistingIdx].Distance)
            {
                OutNodeEdges[*ExistingIdx] = anEdge;
            }
        }
        else
        {
            int32 NewIdx = OutNodeEdges.Num();
            OutNodeEdges.Add(anEdge);
            EdgeMap.Add(Key, NewIdx);
        }
    }
}

void FAdaptiveOctreeNode::UpdateLod(FVector InCameraPosition, double InLodDistanceFactor, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged)
{
    if (IsLeaf())
    {
        if (ShouldSplit(InCameraPosition, InLodDistanceFactor))
        {
            OutChanged = true;
            Split();
            for (TSharedPtr<FAdaptiveOctreeNode> aChild : Children) {
                AppendUniqueEdges(aChild->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            }
            return;
        }
        else if (Parent.IsValid() && Parent.Pin()->ShouldMerge(InCameraPosition, InLodDistanceFactor) && TreeIndex.Last() == 7)
        {
            OutChanged = true;
            auto parentPtr = Parent.Pin();
            parentPtr->Merge();
            AppendUniqueEdges(parentPtr->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            return;
        }
        else {
            AppendUniqueEdges(GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            return;
        }
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            Children[i]->UpdateLod(InCameraPosition, InLodDistanceFactor, OutNodeEdges, EdgeMap, OutChanged);
        }
    }
}

// Retrieves all surface nodes for meshing
// Update this in FAdaptiveOctreeNode.cpp
TArray<FNodeEdge> FAdaptiveOctreeNode::GetSurfaceEdges()
{
    // If we are a leaf, we already HAVE our edges. No need to traverse.
    if (IsLeaf())
    {
        return SignChangeEdges;
    }

    // If we are NOT a leaf (e.g. a Chunk Root), only then do we gather from children
    TArray<FNodeEdge> SurfaceEdges;
    // ... (Your existing TQueue logic is fine here, BUT only if !IsLeaf()) ...
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

TArray<FNodeEdge>& FAdaptiveOctreeNode::GetSignChangeEdges()
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
    FQEF Qef;

    FVector MassPoint = FVector::ZeroVector;
    double MaxDensityRange = 0.0;
    int32 ValidEdges = 0;

    for (FNodeEdge& Edge : SignChangeEdges)
    {
        double Denom = Edge.Corners[1].Density - Edge.Corners[0].Density;
        if (FMath::Abs(Denom) < 1e-15) continue;

        MaxDensityRange = FMath::Max(MaxDensityRange, FMath::Abs(Denom));

        double T = (0 - Edge.Corners[0].Density) / Denom;
        T = FMath::Clamp(T, 0.0, 1.0);

        FVector P = Edge.Corners[0].Position + T * (Edge.Corners[1].Position - Edge.Corners[0].Position);
        MassPoint += P;

        FVector Normal = GetInterpolatedNormal(P);
        Qef.AddPlane(P, Normal);
        ValidEdges++;
    }

    if (ValidEdges == 0)
    {
        DualContourPosition = Center;
        DualContourNormal = FVector::ZeroVector;
        return;
    }

    MassPoint /= (double)ValidEdges;

    // When density variation is below float precision, the zero-crossing
    // T values are pure noise. The mass point averages out the jitter
    // and gives a stable position. Skip the QEF entirely.
    double PrecisionThreshold = 1e-5;
    if (MaxDensityRange < PrecisionThreshold)
    {
        DualContourPosition = MassPoint;
    }
    else
    {
        double Error = 0.0;
        FVector CalculatedPos = Qef.Solve(Center, Extent, &Error);

        if (FVector::DistSquared(CalculatedPos, MassPoint) > (Extent * Extent) * 2.0)
        {
            DualContourPosition = MassPoint;
        }
        else
        {
            DualContourPosition = CalculatedPos;
        }
    }

    // Standard bounding box clamp
    DualContourPosition.X = FMath::Clamp(DualContourPosition.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(DualContourPosition.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(DualContourPosition.Z, Center.Z - Extent, Center.Z + Extent);

    DualContourNormal = Qef.GetAverageNormal();

    // Final Normal Fallback
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
    TArray<FNodeCorner> NodeCorners(Corners, 8);
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