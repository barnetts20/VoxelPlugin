#include "FAdaptiveOctreeNode.h"
#include "FAdaptiveOctree.h"
#include <FQEF.h>
#include <FOctreeConstants.h>

const bool FAdaptiveOctreeNode::IsLeaf() const
{
    return bIsLeaf;
}

const bool FAdaptiveOctreeNode::IsRoot() const
{
    return !Parent.IsValid();
}

FAdaptiveOctreeNode::FAdaptiveOctreeNode(double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    Center = FVector::ZeroVector;
    ChunkCenter = Center;
    DualContourPosition = Center;

    Extent = FMath::Max(InExtent, 0.0);

    DepthBounds[0] = (uint8)InChunkDepth;
    DepthBounds[1] = (uint8)InMinDepth;
    DepthBounds[2] = (uint8)InMaxDepth;

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

    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];
    DepthBounds[2] = InParent->DepthBounds[2];

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

    for (int i = 0; i < 12; i++)
    {
        FNodeEdge NewEdge(Corners[OctreeConstants::EdgePairs[i][0]], Corners[OctreeConstants::EdgePairs[i][1]]);
        if (NewEdge.SignChange)
            SignChangeEdges.Add(NewEdge);
    }

    ComputeDualContourPosition();

    // Update CouldContainSurface: OR with current state so merge-inherited
    // flags survive re-finalization (parent corners may be too coarse to detect
    // edit-created surface that children contained before merging)
    CouldContainSurface = CouldContainSurface || IsNearSurface();

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

    FVector NextAnchor = (Index.Depth == DepthBounds[0]) ? Center : ChunkCenter;

    for (uint8 i = 0; i < 8; i++)
    {
        Children[i] = MakeShared<FAdaptiveOctreeNode>(AsShared(), i, NextAnchor);
    }
    bIsLeaf = false;
}

void FAdaptiveOctreeNode::Merge()
{
    if (bIsLeaf) return;

    // Before destroying children, inherit their surface flags.
    // If any child had surface or near-surface data (possibly from edits
    // at finer resolution than our corners can detect), preserve that
    // knowledge so the LOD system re-splits us when the camera returns.
    for (int i = 0; i < 8; i++)
    {
        if (Children[i].IsValid())
        {
            if (Children[i]->CouldContainSurface)
                CouldContainSurface = true;
            if (Children[i]->IsSurfaceNode)
                IsSurfaceNode = true;
            Children[i]->Merge();
            Children[i].Reset();
        }
    }
    bIsLeaf = true;
}

bool FAdaptiveOctreeNode::ShouldSplit(FVector InCameraPosition, double ThresholdSq, double InFOVScale)
{
    // Back-face cull: skip nodes on the far side of the planet
    double Dot = FVector::DotProduct(InCameraPosition, Center);
    if (Dot < 0.0)
    {
        double CamDistSq = InCameraPosition.SizeSquared();
        double NodeDistSq = Center.SizeSquared();
        if ((Dot * Dot) > (0.04 * CamDistSq * NodeDistSq)) return false;
    }

    double SurfaceDistSq = FVector::DistSquared(DualContourPosition, InCameraPosition);

    return EvaluateSplit(Extent, SurfaceDistSq, InFOVScale, ThresholdSq, Index.Depth, DepthBounds[2], DepthBounds[1]);
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector InCameraPosition, double MergeThresholdSq, double InFOVScale)
{
    if (LodOverride) return false;

    double SurfaceDistSq = FVector::DistSquared(DualContourPosition, InCameraPosition);
    return EvaluateMerge(Extent, SurfaceDistSq, InFOVScale, MergeThresholdSq, Index.Depth, DepthBounds[0], DepthBounds[1]);
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

        if (!CurrentNode || CurrentNode->Index.Depth > DepthBounds[0]) continue;

        if (CurrentNode->Index.Depth == DepthBounds[0] && CurrentNode->IsSurfaceNode)
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
        // Don't clear IsSurfaceNode — it may have been inherited from merged children
        // that contained edit-created surface too fine for this node's corners to detect
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

    //TODO: BRING BACK ERROR INTEGRATION, PROBLEM WAS VERTICALITY IN SURFACE DRIVING WAY TO MANY SPLITS WHEN THE LOD FLATTENS THE ELEVATION AXIS
    double Error = 0.0;
    FVector CalculatedPos = Qef.Solve(Center, Extent, &Error);
    double QefDistSq = FVector::DistSquared(CalculatedPos, MassPoint);
    double MaxDistSq = Extent * Extent;
    if (QefDistSq > MaxDistSq)
    {
        DualContourPosition = MassPoint;
    }
    else if (QefDistSq > MaxDistSq * 0.25) // Start blending at 50% of max distance
    {
        // Smoothly lerp from QEF to MassPoint over the 0.25-1.0 range
        double t = (QefDistSq - MaxDistSq * 0.25) / (MaxDistSq * 0.75);
        t = t * t * (3.0 - 2.0 * t); // Smoothstep
        DualContourPosition = FMath::Lerp(CalculatedPos, MassPoint, t);
    }
    else
    {
        DualContourPosition = CalculatedPos;
    }


    DualContourPosition.X = FMath::Clamp(DualContourPosition.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(DualContourPosition.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(DualContourPosition.Z, Center.Z - Extent, Center.Z + Extent);

    // Cache ocean surface projection of the dual contour position.
    // When ocean is disabled, OceanPosition == DualContourPosition
    // so the min() in LOD distance is a no-op.

    DualContourNormal = FVector3f(Qef.GetAverageNormal());
}