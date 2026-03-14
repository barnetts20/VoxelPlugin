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

FAdaptiveOctreeNode::FAdaptiveOctreeNode(FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    Center = InCenter;
    ChunkCenter = InCenter;
    Extent = FMath::Max(InExtent, 0.0);

    DepthBounds[0] = (uint8)InChunkDepth;
    DepthBounds[1] = (uint8)InMaxDepth;
    DepthBounds[2] = (uint8)InMinDepth;

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

void FAdaptiveOctreeNode::FinalizeFromExistingCorners(FVector TreeCenter, double OceanRadius, bool bSkipNormals)
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

    ComputeDualContourPosition(TreeCenter, OceanRadius);

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

bool FAdaptiveOctreeNode::ShouldSplit(FVector TreeCenter, FVector InCameraPosition, double InScreenSpaceThreshold, double InFOVScale)
{
    FVector ToCam = InCameraPosition - TreeCenter;
    FVector ToNode = Center - TreeCenter;
    double Dot = FVector::DotProduct(ToCam, ToNode);
    if (Dot < 0.0)
    {
        double DotSq = Dot * Dot;
        double ThreshSq = 0.04 * ToCam.SizeSquared() * ToNode.SizeSquared();
        if (DotSq > ThreshSq) return false;
    }

    // Use closer of terrain surface and ocean surface for LOD distance
    double DistSq = FMath::Min(
        FVector::DistSquared(DualContourPosition, InCameraPosition),
        FVector::DistSquared(OceanPosition, InCameraPosition));
    DistSq = FMath::Max(DistSq, 1e-12);

    return EvaluateSplit(Extent, DistSq, InFOVScale, InScreenSpaceThreshold, Index.Depth, DepthBounds[1], DepthBounds[2]);
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector TreeCenter, FVector InCameraPosition, double InScreenSpaceThreshold, double InFOVScale)
{
    if (LodOverride) return false;

    double DistSq = FMath::Min(
        FVector::DistSquared(DualContourPosition, InCameraPosition),
        FVector::DistSquared(OceanPosition, InCameraPosition));
    DistSq = FMath::Max(DistSq, 1e-12);

    return EvaluateMerge(Extent, DistSq, InFOVScale, InScreenSpaceThreshold, Index.Depth, DepthBounds[0], DepthBounds[2]);
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

FVector FAdaptiveOctreeNode::ComputeNormalizedPosition(FVector TreeCenter, double InRadius) const
{
    FVector DirFromCenter = (DualContourPosition - TreeCenter).GetSafeNormal();
    return TreeCenter + DirFromCenter * InRadius;
}

void FAdaptiveOctreeNode::ComputeDualContourPosition(FVector TreeCenter, double OceanRadius)
{
    if (SignChangeEdges.Num() == 0)
    {
        DualContourNormal = FVector3f::ZeroVector;
        DualContourPosition = Center;
        OceanPosition = Center;
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

    if (ValidEdges == 0)
    {
        DualContourPosition = Center;
        OceanPosition = Center;
        DualContourNormal = FVector3f::ZeroVector;
        return;
    }

    MassPoint /= (double)ValidEdges;

    if (Index.Depth > DepthPrecisionFloor)
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

    DualContourPosition.X = FMath::Clamp(DualContourPosition.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(DualContourPosition.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(DualContourPosition.Z, Center.Z - Extent, Center.Z + Extent);

    // Cache ocean surface projection of the dual contour position
    FVector DirFromCenter = (DualContourPosition - TreeCenter);
    double DistFromCenter = DirFromCenter.Size();
    if (DistFromCenter > 1e-10)
        OceanPosition = TreeCenter + (DirFromCenter / DistFromCenter) * OceanRadius;
    else
        OceanPosition = DualContourPosition;

    // Compute normal in double precision, store as FVector3f
    FVector ComputedNormal = Qef.GetAverageNormal();

    if (ComputedNormal.IsNearlyZero())
    {
        ComputedNormal.X = (Corners[1].Density + Corners[3].Density + Corners[5].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[2].Density + Corners[4].Density + Corners[6].Density);
        ComputedNormal.Y = (Corners[2].Density + Corners[3].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[4].Density + Corners[5].Density);
        ComputedNormal.Z = (Corners[4].Density + Corners[5].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[2].Density + Corners[3].Density);
        ComputedNormal.Normalize();
    }

    if (ComputedNormal.IsNearlyZero())
    {
        ComputedNormal = (DualContourPosition - TreeCenter).GetSafeNormal();
    }

    DualContourNormal = FVector3f(ComputedNormal);
}