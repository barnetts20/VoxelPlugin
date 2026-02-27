#include "FAdaptiveOctreeNode.h"
#include "FAdaptiveOctree.h"

const bool FAdaptiveOctreeNode::IsLeaf()
{
    return bIsLeaf;
}

const bool FAdaptiveOctreeNode::IsRoot()
{
    return !Parent.IsValid();
}

FAdaptiveOctreeNode::FAdaptiveOctreeNode(FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    ChunkDepth = InChunkDepth;
    Center = InCenter;
    AnchorCenter = InCenter;
    TreeCenter = InCenter;
    Extent = FMath::Max(InExtent, 0.0);

    DepthBounds[0] = InChunkDepth;
    DepthBounds[1] = InMaxDepth;
    DepthBounds[2] = InMinDepth;

    for (int i = 0; i < 8; i++)
    {
        Corners[i].Position = Center + (Offsets[i] * Extent);
    }
}

FAdaptiveOctreeNode::FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex, FVector InAnchorCenter)
{
    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(ChildIndex);

    ChunkDepth = InParent->ChunkDepth;

    Parent = InParent;
    TreeCenter = InParent->TreeCenter;
    AnchorCenter = InAnchorCenter;

    // Set spatial bounds first
    Extent = InParent->Extent * 0.5;
    Center = InParent->Center + Offsets[ChildIndex] * Extent;

    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];
    DepthBounds[2] = InParent->DepthBounds[2];

    for (int i = 0; i < 8; i++)
    {
        Corners[i].Position = Center + (Offsets[i] * Extent);
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

void FAdaptiveOctreeNode::FinalizeFromExistingCorners()
{
    SignChangeEdges.Reset();

    // Compute normals from local corners if not already set by map
    for (int i = 0; i < 8; i++)
    {
        double d = Corners[i].Density;
        int idxX = i ^ 1;
        int idxY = i ^ 2;
        int idxZ = i ^ 4;

        double dx = (Offsets[i].X < 0) ? (Corners[idxX].Density - d) : (d - Corners[idxX].Density);
        double dy = (Offsets[i].Y < 0) ? (Corners[idxY].Density - d) : (d - Corners[idxY].Density);
        double dz = (Offsets[i].Z < 0) ? (Corners[idxZ].Density - d) : (d - Corners[idxZ].Density);

        FVector Normal(dx, dy, dz);
        if (!Normal.Normalize())
            Normal = (Corners[i].Position - AnchorCenter).GetSafeNormal();

        Corners[i].Normal = Normal;
    }

    for (int i = 0; i < 12; i++)
    {
        FNodeEdge NewEdge(Corners[EdgePairs[i][0]], Corners[EdgePairs[i][1]]);
        if (NewEdge.SignChange)
            SignChangeEdges.Add(NewEdge);
    }
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
        Children[i] = MakeShared<FAdaptiveOctreeNode>( AsShared(), i, NextAnchor);
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

TArray<FNodeEdge>& FAdaptiveOctreeNode::GetSignChangeEdges()
{
    return SignChangeEdges;
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctreeNode::GetSurfaceChunks()
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> SurfaceNodes;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Stack;
    Stack.Reserve(128);
    Stack.Add(AsShared());

    while (Stack.Num() > 0)
    {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Stack.Pop(EAllowShrinking::No);

        if (!CurrentNode || CurrentNode->TreeIndex.Num() > ChunkDepth) continue;

        if (CurrentNode->TreeIndex.Num() == ChunkDepth && CurrentNode->IsSurfaceNode)
        {
            SurfaceNodes.Add(CurrentNode);
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (CurrentNode->Children[i]) Stack.Add(CurrentNode->Children[i]);
            }
        }
    }

    return SurfaceNodes;
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

    if (TreeIndex.Num() > DepthPrecisionFloor)
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

    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal = (DualContourPosition - TreeCenter).GetSafeNormal();
    }

    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal = FVector(0, 0, 1);
    }
}
