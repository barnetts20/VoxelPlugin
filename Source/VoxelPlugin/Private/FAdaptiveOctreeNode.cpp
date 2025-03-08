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
    bool CanMerge = true;
    for (const auto& Child : Children)
    {
        if (!Child.IsValid() || !Child->IsLeaf())
        {
            CanMerge = false;
            break; 
        }
    }
    if (LodOverride) return false;
    return CanMerge && (FVector::Dist(DualContourPosition, InCameraPosition) > Extent * (InLodDistanceFactor + TreeIndex.Num())) && TreeIndex.Num() >= DepthBounds[0];
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
        // If not found, add it to the array
        if (!found) {
            OutNodeEdges.Add(anEdge);
        }
    }
}


//void AppendUniqueEdges(TArray<FNodeEdge> InAppendEdges, TArray<FNodeEdge>& OutNodeEdges) {
//    for (FNodeEdge& anEdge : InAppendEdges) {
//        //int32 cEdge = OutNodeEdges.Find(anEdge);
//        //if (cEdge != INDEX_NONE) {
//        //    //Replace with smallest edge
//        //    if (OutNodeEdges[cEdge].Size > anEdge.Size) {
//        //        OutNodeEdges[cEdge] = anEdge;
//        //    }
//        //}
//        //else {
//        //    OutNodeEdges.Add(anEdge);
//        //}
//        OutNodeEdges.AddUnique(anEdge);
//    }
//}

void FAdaptiveOctreeNode::UpdateLod(FVector InCameraPosition, double InLodDistanceFactor, TArray<FNodeEdge>& OutNodeEdges, bool& OutChanged)
{
    if (IsLeaf())
    {
        if (ShouldSplit(InCameraPosition, InLodDistanceFactor))
        {
            OutChanged = true;
            Split();
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

// Compute Dual Contour Position, currently just naive surface nets
void FAdaptiveOctreeNode::ComputeDualContourPosition()
{

    if (SignChangeEdges.Num() > 0){
        DualContourPosition = FVector::ZeroVector;
        for (FNodeEdge& anEdge : SignChangeEdges) {
            DualContourPosition += anEdge.ZeroCrossingPoint;
        }
        DualContourPosition = DualContourPosition / (double)SignChangeEdges.Num();

        DualContourNormal = FVector(0, 0, 0);
        DualContourNormal.X = (Corners[1].Density + Corners[3].Density + Corners[5].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[2].Density + Corners[4].Density + Corners[6].Density);
        DualContourNormal.Y = (Corners[2].Density + Corners[3].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[4].Density + Corners[5].Density);
        DualContourNormal.Z = (Corners[4].Density + Corners[5].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[2].Density + Corners[3].Density);

        DualContourNormal.Normalize();
        IsSurfaceNode = true;
    }
    else {
        DualContourNormal = FVector(0, 0, 0);
        DualContourPosition = Center;
        IsSurfaceNode = false;
    }
}

bool FAdaptiveOctreeNode::RefineDualContour(const FNodeEdge& RefEdge)
{
    // If already a surface node, no need to refine
    if (IsSurfaceNode) {
        return true;
    }

    // Get the midpoints of all edges and sample density there
    TArray<FNodeCorner> MidpointSamples;
    TArray<FNodeEdge> RefinedEdges;

    // Iterate through all 12 edges of the cube
    for (int i = 0; i < 12; i++) {
        int v0Idx = EdgePairs[i][0];
        int v1Idx = EdgePairs[i][1];

        // Get the midpoint of this edge
        FVector MidPoint = (Corners[v0Idx].Position + Corners[v1Idx].Position) * 0.5f;
        double MidDensity = DensityFunction(MidPoint);

        // Create a new corner sample for this midpoint
        FNodeCorner MidCorner(12 + i, MidPoint, MidDensity); // Index starts at 12 for midpoints
        MidpointSamples.Add(MidCorner);

        // Create edges between midpoints and original corners
        FNodeEdge Edge1(Corners[v0Idx], MidCorner);
        FNodeEdge Edge2(MidCorner, Corners[v1Idx]);

        // Add edges if they cross the surface
        if (Edge1.SignChange) {
            RefinedEdges.Add(Edge1);
        }
        if (Edge2.SignChange) {
            RefinedEdges.Add(Edge2);
        }
    }

    // If we have surface crossings in the refined edges
    if (RefinedEdges.Num() > 0) {
        // We need to handle the case where an original edge now has multiple crossings
        TMap<int32, FNodeEdge> BestEdges;

        for (const FNodeEdge& Edge : RefinedEdges) {
            // Determine which original edge this refined edge belongs to
            int32 OriginalEdgeIndex = -1;

            // Check which original edge's corners match this refined edge's corners
            for (int i = 0; i < 12; i++) {
                int v0 = EdgePairs[i][0];
                int v1 = EdgePairs[i][1];

                // If this refined edge connects an original corner to a midpoint
                if ((Edge.Corners[0].CornerIndex < 8 && Edge.Corners[1].CornerIndex >= 12) ||
                    (Edge.Corners[1].CornerIndex < 8 && Edge.Corners[0].CornerIndex >= 12)) {

                    // Get the original corner index
                    int OrigCornerIdx = (Edge.Corners[0].CornerIndex < 8) ? Edge.Corners[0].CornerIndex : Edge.Corners[1].CornerIndex;

                    // If this corner is part of the original edge
                    if (OrigCornerIdx == v0 || OrigCornerIdx == v1) {
                        OriginalEdgeIndex = i;
                        break;
                    }
                }
            }

            // If we found which original edge this belongs to
            if (OriginalEdgeIndex != -1) {
                // Calculate distance to the reference edge's zero crossing
                float DistToRefZero = FVector::Distance(Edge.ZeroCrossingPoint, RefEdge.ZeroCrossingPoint);

                // If we haven't seen this original edge yet, or this crossing is closer to ref edge
                if (!BestEdges.Contains(OriginalEdgeIndex) ||
                    DistToRefZero < FVector::Distance(BestEdges[OriginalEdgeIndex].ZeroCrossingPoint, RefEdge.ZeroCrossingPoint)) {

                    BestEdges.Add(OriginalEdgeIndex, Edge);
                }
            }
        }

        // Now use the best edges to compute a refined dual contour position
        TArray<FVector> RefinedCrossings;
        for (const auto& EdgePair : BestEdges) {
            RefinedCrossings.Add(EdgePair.Value.ZeroCrossingPoint);
        }

        // If we have crossings, update the dual contour position
        if (RefinedCrossings.Num() > 0) {
            DualContourPosition = Algo::Accumulate(RefinedCrossings, FVector::ZeroVector) / RefinedCrossings.Num();

            // Recompute normal using both original corners and midpoint samples
            TArray<FNodeCorner> AllSamples = Corners;
            AllSamples.Append(MidpointSamples);

            // Calculate gradient using all samples
            FVector Gradient = FVector::ZeroVector;
            for (const FNodeCorner& Sample : AllSamples) {
                // Weight by inverse distance from cell center
                float Weight = 1.0f / FMath::Max(0.01f, FVector::Distance(Sample.Position, Center));
                Gradient += (Sample.Position - Center) * Sample.Density * Weight;
            }

            // Normalize to get normal direction
            DualContourNormal = -Gradient.GetSafeNormal();

            // Mark as surface node
            IsSurfaceNode = true;

            // Keep track of the refined edges for future reference
            SignChangeEdges.Empty();
            for (const auto& EdgePair : BestEdges) {
                SignChangeEdges.Add(EdgePair.Value);
            }

            return true;
        }
    }

    // No valid dual contour point found
    return false;
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