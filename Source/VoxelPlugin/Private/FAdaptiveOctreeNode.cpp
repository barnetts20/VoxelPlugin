#include "FAdaptiveOctreeNode.h"

// ============================================================================
// CONSTRUCTORS
// ============================================================================

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

FAdaptiveOctreeNode::FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex, FVector InAnchorCenter)
{
    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(ChildIndex);

    Parent = InParent;
    DensityFunction = InParent->DensityFunction;
    EditStore = InParent->EditStore;
    AnchorCenter = InAnchorCenter;

    Extent = InParent->Extent * 0.5;
    Center = InParent->Center + Offsets[ChildIndex] * Extent;

    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];
    DepthBounds[2] = InParent->DepthBounds[2];

    ComputeCornerDensity();
    ComputeDualContourPosition();
}

// ============================================================================
// STATE QUERIES
// ============================================================================

bool FAdaptiveOctreeNode::IsLeaf() { return bIsLeaf; }
bool FAdaptiveOctreeNode::IsRoot() { return !Parent.IsValid(); }

// ============================================================================
// DENSITY SAMPLING (Thread Safe)
// ============================================================================

double FAdaptiveOctreeNode::SampleDensity(FVector InPosition, FVector InAnchorCenter)
{
    return (*DensityFunction)(InPosition, InAnchorCenter) + EditStore->Sample(InPosition);
}

// ============================================================================
// CORNER & EDGE COMPUTATION (Background Thread)
// ============================================================================

void FAdaptiveOctreeNode::ComputeCornerDensity()
{
    SignChangeEdges.Empty();

    // Sample density at 8 corners only (8 calls, not 32)
    for (int i = 0; i < 8; i++)
    {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        double d = SampleDensity(CornerPosition, AnchorCenter);
        Corners[i] = FNodeCorner(CornerPosition, d, FVector::ZeroVector);
    }

    // Derive normals from corner densities — no extra sampling
    // Offsets layout: 0(-,-,-) 1(+,-,-) 2(-,+,-) 3(+,+,-) 4(-,-,+) 5(+,-,+) 6(-,+,+) 7(+,+,+)
    for (int i = 0; i < 8; i++)
    {
        // Neighbors along each axis (flip bit 0 for X, bit 1 for Y, bit 2 for Z)
        double dx = Corners[i ^ 1].Density - Corners[i].Density;
        double dy = Corners[i ^ 2].Density - Corners[i].Density;
        double dz = Corners[i ^ 4].Density - Corners[i].Density;

        // Correct sign: if this corner is on the positive side of that axis, flip
        if (i & 1) dx = -dx;
        if (i & 2) dy = -dy;
        if (i & 4) dz = -dz;

        FVector grad(dx, dy, dz);
        if (!grad.Normalize()) grad = FVector::UpVector;
        Corners[i].Normal = grad;
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]]);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    IsSurfaceNode = !SignChangeEdges.IsEmpty();
}

FVector FAdaptiveOctreeNode::GetInterpolatedNormal(FVector P)
{
    double tx = (P.X - (Center.X - Extent)) / (2.0 * Extent);
    double ty = (P.Y - (Center.Y - Extent)) / (2.0 * Extent);
    double tz = (P.Z - (Center.Z - Extent)) / (2.0 * Extent);

    tx = FMath::Clamp(tx, 0.0, 1.0);
    ty = FMath::Clamp(ty, 0.0, 1.0);
    tz = FMath::Clamp(tz, 0.0, 1.0);

    FVector n00 = FMath::Lerp(Corners[0].Normal, Corners[1].Normal, tx);
    FVector n01 = FMath::Lerp(Corners[2].Normal, Corners[3].Normal, tx);
    FVector n10 = FMath::Lerp(Corners[4].Normal, Corners[5].Normal, tx);
    FVector n11 = FMath::Lerp(Corners[6].Normal, Corners[7].Normal, tx);

    FVector n0 = FMath::Lerp(n00, n01, ty);
    FVector n1 = FMath::Lerp(n10, n11, ty);

    return FMath::Lerp(n0, n1, tz).GetSafeNormal();
}

// ============================================================================
// DUAL CONTOUR POSITION (Background Thread)
// ============================================================================

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
    int32 ValidEdges = 0;

    for (FNodeEdge& Edge : SignChangeEdges)
    {
        double Denom = Edge.Corners[1].Density - Edge.Corners[0].Density;
        if (FMath::Abs(Denom) < 1e-15) continue;

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

    // Solve QEF
    double Error = 0.0;
    FVector CalculatedPos = Qef.Solve(Center, Extent, &Error);

    // Clamp QEF solution to cell bounds
    FVector ClampedPos;
    ClampedPos.X = FMath::Clamp(CalculatedPos.X, Center.X - Extent, Center.X + Extent);
    ClampedPos.Y = FMath::Clamp(CalculatedPos.Y, Center.Y - Extent, Center.Y + Extent);
    ClampedPos.Z = FMath::Clamp(CalculatedPos.Z, Center.Z - Extent, Center.Z + Extent);

    // Blend between QEF and mass point based on error magnitude
    double ErrorThreshold = Extent * 0.1;
    double MaxError = Extent * 1.0;
    double ErrorAlpha = FMath::Clamp((Error - ErrorThreshold) / (MaxError - ErrorThreshold), 0.0, 1.0);
    ErrorAlpha = ErrorAlpha * ErrorAlpha * (3.0 - 2.0 * ErrorAlpha); // Smoothstep

    FVector BlendedPos = FMath::Lerp(ClampedPos, MassPoint, ErrorAlpha);

    // Sign preservation: verify vertex is near surface
    double VertexDensity = SampleDensity(BlendedPos, AnchorCenter);
    if (FMath::Abs(VertexDensity) > Extent * 0.5)
    {
        BlendedPos = MassPoint;
    }

    // Final clamp
    DualContourPosition.X = FMath::Clamp(BlendedPos.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(BlendedPos.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(BlendedPos.Z, Center.Z - Extent, Center.Z + Extent);

    // Normal
    DualContourNormal = Qef.GetAverageNormal();

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

// ============================================================================
// LOD TREE OPERATIONS (Background Thread)
// ============================================================================

void FAdaptiveOctreeNode::Split()
{
    if (!bIsLeaf) return;

    FVector NextAnchor = (TreeIndex.Num() == DepthBounds[0]) ? Center : AnchorCenter;

    for (uint8 i = 0; i < 8; i++)
    {
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
            Children[i]->Merge();
            Children[i].Reset();
        }
    }
    bIsLeaf = true;
}

bool FAdaptiveOctreeNode::ShouldSplit(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    int Depth = TreeIndex.Num();
    if (Depth >= DepthBounds[1]) return false;
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
            if (anEdge.Distance < OutNodeEdges[*ExistingIdx].Distance)
                OutNodeEdges[*ExistingIdx] = anEdge;
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
            for (TSharedPtr<FAdaptiveOctreeNode> aChild : Children)
                AppendUniqueEdges(aChild->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
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
        else
        {
            AppendUniqueEdges(GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            return;
        }
    }
    else
    {
        for (int i = 0; i < 8; i++)
            Children[i]->UpdateLod(InCameraPosition, InScreenSpaceThreshold, InCameraFOV, OutNodeEdges, EdgeMap, OutChanged);
    }
}

// ============================================================================
// SURFACE DATA ACCESSORS (Thread Safe)
// ============================================================================

TArray<FNodeEdge> FAdaptiveOctreeNode::GetSurfaceEdges()
{
    if (IsLeaf())
        return SignChangeEdges;

    ensure(false);
    return TArray<FNodeEdge>();
}

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
                if (CurrentNode->Children[i])
                    NodeQueue.Enqueue(CurrentNode->Children[i]);
            }
        }
    }

    return SurfaceNodes;
}

TArray<FNodeEdge>& FAdaptiveOctreeNode::GetSignChangeEdges()
{
    return SignChangeEdges;
}

void FAdaptiveOctreeNode::Reconstruct()
{
    ComputeCornerDensity();
    ComputeDualContourPosition();

    if (IsSurfaceNode)
    {
        TSharedPtr<FAdaptiveOctreeNode> Ancestor = Parent.Pin();
        while (Ancestor.IsValid() && !Ancestor->IsSurfaceNode)
        {
            Ancestor->IsSurfaceNode = true;
            Ancestor = Ancestor->Parent.Pin();
        }
    }
}