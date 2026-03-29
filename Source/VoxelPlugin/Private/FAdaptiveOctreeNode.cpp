#include "FAdaptiveOctreeNode.h"
#include "FAdaptiveOctree.h"
#include "FQEF.h"
#include "FOctreeConstants.h"

bool FAdaptiveOctreeNode::IsLeaf() const
{
    return bIsLeaf;
}

bool FAdaptiveOctreeNode::IsRoot() const
{
    return !Parent.IsValid();
}

FAdaptiveOctreeNode::FAdaptiveOctreeNode(double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    Center = FVector::ZeroVector;
    ChunkCenter = Center;
    DualContourPosition = Center;

    Extent = FMath::Max(InExtent, 0.0);

    DepthBounds[EDepthBound::ChunkDepth] = (uint8)InChunkDepth;
    DepthBounds[EDepthBound::MinDepth] = (uint8)InMinDepth;
    DepthBounds[EDepthBound::MaxDepth] = (uint8)InMaxDepth;

    for (int i = 0; i < 8; i++)
    {
        Corners[i].Position = Center + (OctreeConstants::Offsets[i] * Extent);
    }
}

FAdaptiveOctreeNode::FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex, FVector InAnchorCenter)
{
    Index = InParent->Index;
    Index.PushChild(ChildIndex);

    Parent = InParent;
    ChunkCenter = InAnchorCenter;

    Extent = InParent->Extent * 0.5;
    Center = InParent->Center + OctreeConstants::Offsets[ChildIndex] * Extent;
    DualContourPosition = Center;

    DepthBounds[EDepthBound::ChunkDepth] = InParent->DepthBounds[EDepthBound::ChunkDepth];
    DepthBounds[EDepthBound::MinDepth] = InParent->DepthBounds[EDepthBound::MinDepth];
    DepthBounds[EDepthBound::MaxDepth] = InParent->DepthBounds[EDepthBound::MaxDepth];

    for (int i = 0; i < 8; i++)
    {
        Corners[i].Position = Center + (OctreeConstants::Offsets[i] * Extent);
    }
}

FVector FAdaptiveOctreeNode::GetInterpolatedNormal(FVector P)
{
    double tx = (P.X - (Center.X - Extent)) / (2.0 * Extent);
    double ty = (P.Y - (Center.Y - Extent)) / (2.0 * Extent);
    double tz = (P.Z - (Center.Z - Extent)) / (2.0 * Extent);

    float ftx = (float)FMath::Clamp(tx, 0.0, 1.0);
    float fty = (float)FMath::Clamp(ty, 0.0, 1.0);
    float ftz = (float)FMath::Clamp(tz, 0.0, 1.0);

    // Trilinear interpolation of corner normals (FVector3f), then widen to
    // FVector for the QEF which operates in double precision.
    FVector3f n00 = FMath::Lerp(Corners[0].Normal, Corners[1].Normal, ftx);
    FVector3f n01 = FMath::Lerp(Corners[2].Normal, Corners[3].Normal, ftx);
    FVector3f n10 = FMath::Lerp(Corners[4].Normal, Corners[5].Normal, ftx);
    FVector3f n11 = FMath::Lerp(Corners[6].Normal, Corners[7].Normal, ftx);

    FVector3f n0 = FMath::Lerp(n00, n01, fty);
    FVector3f n1 = FMath::Lerp(n10, n11, fty);

    FVector3f FinalNormal = FMath::Lerp(n0, n1, ftz);
    return FVector(FinalNormal).GetSafeNormal();
}

void FAdaptiveOctreeNode::FinalizeFromExistingCorners(bool bSkipNormals)
{
    SignChangeEdges.Reset();

    // Compute per-corner normals from finite differences of neighbor densities.
    // Each corner's gradient is estimated using the three axis-adjacent corners
    // (found by flipping one bit of the corner index).
    if (!bSkipNormals)
    {
        for (int i = 0; i < 8; i++)
        {
            double d = Corners[i].Density;
            int idxX = i ^ 1;
            int idxY = i ^ 2;
            int idxZ = i ^ 4;

            double dx = (OctreeConstants::Offsets[i].X < 0) ? (Corners[idxX].Density - d) : (d - Corners[idxX].Density);
            double dy = (OctreeConstants::Offsets[i].Y < 0) ? (Corners[idxY].Density - d) : (d - Corners[idxY].Density);
            double dz = (OctreeConstants::Offsets[i].Z < 0) ? (Corners[idxZ].Density - d) : (d - Corners[idxZ].Density);

            FVector Normal(dx, dy, dz);
            if (!Normal.Normalize())
                Normal = (Corners[i].Position - ChunkCenter).GetSafeNormal();
            Corners[i].Normal = FVector3f(Normal);
        }
    }

    // Build sign-change edges from all 12 cube edges
    for (int i = 0; i < 12; i++)
    {
        FNodeEdge NewEdge(Corners[OctreeConstants::EdgePairs[i][0]], Corners[OctreeConstants::EdgePairs[i][1]]);
        if (NewEdge.SignChange)
            SignChangeEdges.Add(NewEdge);
    }

    ComputeDualContourPosition();

    // CouldContainSurface: true if this node has sign changes or corners near zero.
    // Edit persistence is handled separately by bHasEditedDescendants.
    CouldContainSurface = IsNearSurface();

    // Propagate surface flags up to ancestors so the LOD system knows
    // which subtrees contain geometry without traversing them.
    if (IsSurfaceNode)
    {
        TSharedPtr<FAdaptiveOctreeNode> Ancestor = Parent.Pin();
        while (Ancestor.IsValid() && !Ancestor->IsSurfaceNode)
        {
            Ancestor->IsSurfaceNode = true;
            Ancestor->CouldContainSurface = true;
            Ancestor = Ancestor->Parent.Pin();
        }
    }
    else if (CouldContainSurface)
    {
        TSharedPtr<FAdaptiveOctreeNode> Ancestor = Parent.Pin();
        while (Ancestor.IsValid() && !Ancestor->CouldContainSurface)
        {
            Ancestor->CouldContainSurface = true;
            Ancestor = Ancestor->Parent.Pin();
        }
    }
}

void FAdaptiveOctreeNode::Split()
{
    if (!bIsLeaf) return;

    // At chunk depth, children start a new chunk — anchor to this node's center.
    // Otherwise inherit the existing chunk center from the parent.
    FVector NextAnchor = (Index.Depth == DepthBounds[EDepthBound::ChunkDepth]) ? Center : ChunkCenter;

    for (uint8 i = 0; i < 8; i++)
    {
        Children[i] = MakeShared<FAdaptiveOctreeNode>(AsShared(), i, NextAnchor);
    }
    bIsLeaf = false;
}

void FAdaptiveOctreeNode::Merge()
{
    if (bIsLeaf) return;

    // Before destroying children, check if any had edit-created surface
    // that this node's corners can't resolve. Preserve that knowledge
    // so the LOD system re-splits when the camera returns.
    for (int i = 0; i < 8; i++)
    {
        if (Children[i].IsValid())
        {
            if (Children[i]->bHasEditedDescendants || Children[i]->CouldContainSurface)
                bHasEditedDescendants = true;
            Children[i]->Merge();
            Children[i].Reset();
        }
    }
    bIsLeaf = true;
}

bool FAdaptiveOctreeNode::ShouldSplit(FVector InCameraPosition, double ThresholdSq, double InFOVScale)
{
    // Back-face cull: skip nodes on the far side of the planet.
    // The dot product is negative when the node is behind the planet center
    // relative to the camera. The threshold (0.04) allows a ~11 degree margin.
    double Dot = FVector::DotProduct(InCameraPosition, Center);
    if (Dot < 0.0)
    {
        double CamDistSq = InCameraPosition.SizeSquared();
        double NodeDistSq = Center.SizeSquared();
        if ((Dot * Dot) > (0.04 * CamDistSq * NodeDistSq)) return false;
    }

    double SurfaceDistSq = FVector::DistSquared(DualContourPosition, InCameraPosition);

    return EvaluateSplit(Extent, SurfaceDistSq, InFOVScale, ThresholdSq, Index.Depth, DepthBounds[EDepthBound::MaxDepth], DepthBounds[EDepthBound::MinDepth]);
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector InCameraPosition, double MergeThresholdSq, double InFOVScale)
{
    if (LodOverride) return false;

    double SurfaceDistSq = FVector::DistSquared(DualContourPosition, InCameraPosition);
    return EvaluateMerge(Extent, SurfaceDistSq, InFOVScale, MergeThresholdSq, Index.Depth, DepthBounds[EDepthBound::ChunkDepth], DepthBounds[EDepthBound::MinDepth]);
}

TArray<FNodeEdge>& FAdaptiveOctreeNode::GetSignChangeEdges()
{
    return SignChangeEdges;
}

FVector FAdaptiveOctreeNode::ComputeNormalizedPosition(double InRadius) const
{
    return DualContourPosition.GetSafeNormal() * InRadius;
}

void FAdaptiveOctreeNode::ComputeDualContourPosition()
{
    if (SignChangeEdges.Num() == 0)
    {
        DualContourNormal = FVector3f::ZeroVector;
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
        FVector P = Edge.ZeroCrossingPoint;
        MassPoint += P;

        FVector Normal = GetInterpolatedNormal(P);
        Qef.AddPlane(P, Normal);
        ValidEdges++;
    }

    MassPoint /= (double)ValidEdges;

    double Error = 0.0;
    FVector CalculatedPos = Qef.Solve(Center, Extent, &Error);

    double QefDistSq = FVector::DistSquared(CalculatedPos, MassPoint);
    double MaxDistSq = Extent * Extent;

    // Blend factor from spatial drift (existing logic)
    double tDist = 0.0;
    if (QefDistSq > MaxDistSq)
    {
        tDist = 1.0;
    }
    else if (QefDistSq > MaxDistSq * 0.25)
    {
        tDist = (QefDistSq - MaxDistSq * 0.25) / (MaxDistSq * 0.75);
        tDist = tDist * tDist * (3.0 - 2.0 * tDist);
    }

    // Blend factor from QEF error (surface too complex for this LOD).
    // Normalized error > 1.0 means the half-planes disagree by more than
    // the node's own extent — a strong signal that the vertex is unreliable.
    double ErrorIntolerance = 1;
    double NormalizedError = Error / FMath::Max(MaxDistSq, 1e-12);
    double tError = FMath::Clamp(NormalizedError * ErrorIntolerance, 0.0, 1.0);
    tError = tError * tError * (3.0 - 2.0 * tError);

    // Whichever reason is stronger drives the blend toward mass point
    double t = FMath::Max(tDist, tError);

    DualContourPosition = FMath::Lerp(CalculatedPos, MassPoint, t);

    // Final clamp to node bounds
    DualContourPosition.X = FMath::Clamp(DualContourPosition.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(DualContourPosition.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(DualContourPosition.Z, Center.Z - Extent, Center.Z + Extent);

    DualContourNormal = FVector3f(Qef.GetAverageNormal());
}