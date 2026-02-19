#include "FAdaptiveOctreeNode.h"

// Root Constructor
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TFunction<double(FVector, FVector)>* InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    EditStore = InEditStore;
    Center = InCenter;
    AnchorCenter = InCenter;

    Extent = FMath::Max(InExtent, 0.0);

    DepthBounds[0] = InChunkDepth;
    DepthBounds[1] = InMaxDepth;
    DepthBounds[2] = InMinDepth;

    ComputeCornerDensity();
    ComputeDualContourPosition();
}

// Child Constructor
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex, FVector InAnchorCenter)
{
    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(ChildIndex);

    Parent = InParent;
    DensityFunction = InParent->DensityFunction;
    EditStore = InParent->EditStore;
    AnchorCenter = InAnchorCenter;

    // Set spatial bounds first
    Extent = InParent->Extent * 0.5;
    Center = InParent->Center + Offsets[ChildIndex] * Extent;

    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];
    DepthBounds[2] = InParent->DepthBounds[2];
    // DERIVE CHUNK EXTENT: Using the TreeIndex depth relative to ChunkDepth
    // ChunkDepth should be a member or accessible variable (e.g., 5)
    ComputeCornerDensity();
    ComputeDualContourPosition();
}

bool FAdaptiveOctreeNode::IsLeaf()
{
    return bIsLeaf;
}

bool FAdaptiveOctreeNode::IsRoot()
{
    return !Parent.IsValid();
}

double FAdaptiveOctreeNode::SampleDensity(FVector InPosition, FVector InAnchorCenter) {
    double base = (*DensityFunction)(InPosition, InAnchorCenter);
    double edit = EditStore->Sample(InPosition);
    return base + edit;
}

void FAdaptiveOctreeNode::ComputeCornerDensity() {
    SignChangeEdges.Empty();
    // Use a conservative h for the root
    double ChunkExtent = Extent * FMath::Pow(2.0, (double)(TreeIndex.Num() - DepthBounds[0]));
    double h = FMath::Max(0.1 * Extent, ChunkExtent * 2 * 1e-6);

    for (int i = 0; i < 8; i++) {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        double d = SampleDensity(CornerPosition, AnchorCenter);

        // Forward difference is fine for the root as it's usually empty/air
        double dx = SampleDensity(CornerPosition + FVector(h, 0, 0), AnchorCenter) - d;
        double dy = SampleDensity(CornerPosition + FVector(0, h, 0), AnchorCenter) - d;
        double dz = SampleDensity(CornerPosition + FVector(0, 0, h), AnchorCenter) - d;

        FVector grad(dx, dy, dz);
        if (!grad.Normalize()) { grad = FVector::UpVector; }

        Corners[i] = FNodeCorner(CornerPosition, d, grad);
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]]);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }
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
    FVector NextAnchor = (TreeIndex.Num() == DepthBounds[0]) ? Center : AnchorCenter;

    for (uint8 i = 0; i < 8; i++)
    {
        // Update constructor to take NextAnchor
        Children[i] = MakeShared<FAdaptiveOctreeNode>(AsShared(), i, NextAnchor);

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

    for (int i = 0; i < 8; i++)
    {
        if (Children[i].IsValid())
        {
            Children[i]->Merge(); // Recursively clean grandchildren
            Children[i].Reset();
        }
    }
    bIsLeaf = true;
}


bool FAdaptiveOctreeNode::ShouldSplit(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    int Depth = TreeIndex.Num();
    if (Depth >= DepthBounds[1]) return false;
    //TODO: We can probably reap a pretty substantial optimization by checking if the chunk is on the far side of the planet by enough to be expected to have no visible parts
    if (Depth < DepthBounds[2]) return true;
    
    double Distance = FMath::Max(FVector::Dist(DualContourPosition, InCameraPosition), 1.0);
    double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));
    double AngularSize = ((2.0 * Extent) / Distance) * FOVScale;

    return AngularSize > InScreenSpaceThreshold;
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    int Depth = TreeIndex.Num();
    if (LodOverride) return false;
    if (Depth <= DepthBounds[0]) return false;
    if (Depth < DepthBounds[2]) return false;
    double Distance = FMath::Max(FVector::Dist(DualContourPosition, InCameraPosition), 1.0);
    double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));
    double AngularSize = ((2.0 * Extent) / Distance) * FOVScale;

    return AngularSize < InScreenSpaceThreshold * 0.5;
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

void FAdaptiveOctreeNode::UpdateLod(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged)
{
    if (IsLeaf())
    {
        if (ShouldSplit(InCameraPosition, InScreenSpaceThreshold, InCameraFOV))
        {
            OutChanged = true;
            Split();
            for (TSharedPtr<FAdaptiveOctreeNode> aChild : Children) {
                AppendUniqueEdges(aChild->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            }
            return;
        }
        else if (Parent.IsValid() && Parent.Pin()->ShouldMerge(InCameraPosition, InScreenSpaceThreshold, InCameraFOV) && TreeIndex.Last() == 7)
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
            Children[i]->UpdateLod(InCameraPosition, InScreenSpaceThreshold, InCameraFOV, OutNodeEdges, EdgeMap, OutChanged);
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

    ensure(false);
    return TArray<FNodeEdge>();
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

        FVector P = Edge.ZeroCrossingPoint;
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
