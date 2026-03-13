#include "FAdaptiveOctreeNode.h"
#include "FAdaptiveOctree.h"

FAdaptiveOctreeNode::FAdaptiveOctreeNode(FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    Center = InCenter;
    AnchorCenter = InCenter;
    TreeCenter = InCenter;
    Extent = FMath::Max(InExtent, 0.0);

    DepthBounds[0] = InChunkDepth;
    DepthBounds[1] = InMaxDepth;
    DepthBounds[2] = InMinDepth;
}

FAdaptiveOctreeNode::FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex, FVector InAnchorCenter)
{
    Index = InParent->Index;
    Index.PushChild(ChildIndex);

    Parent = InParent;
    TreeCenter = InParent->TreeCenter;
    AnchorCenter = InAnchorCenter;

    // Set spatial bounds first
    Extent = InParent->Extent * 0.5;
    Center = InParent->Center + OctreeConstants::Offsets[ChildIndex] * Extent;

    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];
    DepthBounds[2] = InParent->DepthBounds[2];
}

//FVector FAdaptiveOctreeNode::GetInterpolatedNormal(FVector P)
//{
//    // Normalize P to [0, 1] range within the cell
//    double tx = (P.X - (Center.X - Extent)) / (2.0 * Extent);
//    double ty = (P.Y - (Center.Y - Extent)) / (2.0 * Extent);
//    double tz = (P.Z - (Center.Z - Extent)) / (2.0 * Extent);
//
//    tx = FMath::Clamp(tx, 0.0, 1.0);
//    ty = FMath::Clamp(ty, 0.0, 1.0);
//    tz = FMath::Clamp(tz, 0.0, 1.0);
//
//    // Trilinear interpolation of the 8 corner normals
//    FVector n00 = FMath::Lerp(Corners[0].Normal, Corners[1].Normal, tx);
//    FVector n01 = FMath::Lerp(Corners[2].Normal, Corners[3].Normal, tx);
//    FVector n10 = FMath::Lerp(Corners[4].Normal, Corners[5].Normal, tx);
//    FVector n11 = FMath::Lerp(Corners[6].Normal, Corners[7].Normal, tx);
//
//    FVector n0 = FMath::Lerp(n00, n01, ty);
//    FVector n1 = FMath::Lerp(n10, n11, ty);
//
//    FVector FinalNormal = FMath::Lerp(n0, n1, tz);
//    return FinalNormal.GetSafeNormal();
//}
//
//void FAdaptiveOctreeNode::FinalizeFromExistingCorners()
//{
//    SignChangeEdges.Reset();
//
//    // Compute normals from local corners if not already set by map
//    for (int i = 0; i < 8; i++)
//    {
//        double d = Corners[i].Density;
//        int idxX = i ^ 1;
//        int idxY = i ^ 2;
//        int idxZ = i ^ 4;
//
//        double dx = (OctreeConstants::Offsets[i].X < 0) ? (Corners[idxX].Density - d) : (d - Corners[idxX].Density);
//        double dy = (OctreeConstants::Offsets[i].Y < 0) ? (Corners[idxY].Density - d) : (d - Corners[idxY].Density);
//        double dz = (OctreeConstants::Offsets[i].Z < 0) ? (Corners[idxZ].Density - d) : (d - Corners[idxZ].Density);
//
//        FVector Normal(dx, dy, dz);
//        if (!Normal.Normalize())
//            Normal = (Corners[i].Position - AnchorCenter).GetSafeNormal();
//
//        Corners[i].Normal = Normal;
//    }
//
//    for (int i = 0; i < 12; i++)
//    {
//        FNodeEdge NewEdge(Corners[OctreeConstants::EdgePairs[i][0]], Corners[OctreeConstants::EdgePairs[i][1]]);
//        if (NewEdge.SignChange)
//            SignChangeEdges.Add(NewEdge);
//    }
//    ComputeDualContourPosition();
//
//    if (bIsSurfaceNode)
//    {
//        TSharedPtr<FAdaptiveOctreeNode> Ancestor = Parent.Pin();
//        while (Ancestor.IsValid() && !Ancestor->bIsSurfaceNode)
//        {
//            Ancestor->bIsSurfaceNode = true;
//            Ancestor = Ancestor->Parent.Pin();
//        }
//    }
//}

void FAdaptiveOctreeNode::Split()
{
    if (!bIsLeaf) return;

    // Determine the anchor for children
    // If THIS node is at the ChunkDepth, its children will use its center as their anchor.
    // Otherwise, they inherit the current anchor.
    FVector NextAnchor = (Index.Depth == DepthBounds[0]) ? Center : AnchorCenter;

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
    FVector ToCam = (InCameraPosition - TreeCenter).GetSafeNormal();
    FVector ToNode = (Center - TreeCenter).GetSafeNormal();
    float Dot = FVector::DotProduct(ToCam, ToNode);
    // Node is on the far hemisphere — don't split further
    if (Dot < -0.2f) return false; // small negative threshold for horizon margin

    int Depth = Index.Depth;
    if (Depth >= DepthBounds[1]) return false;
    if (Depth < DepthBounds[2]) return true;
    
    double Distance = FMath::Max(FVector::Dist(DualContourPosition, InCameraPosition), 1.0);
    //double Distance2 = FMath::Max(FVector::Dist(NormalizedPosition, InCameraPosition), 1.0);
    //double Distance = FMath::Min(Distance, Distance2);
    double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));
    double AngularSize = ((2.0 * Extent) / Distance) * FOVScale;

    return AngularSize > InScreenSpaceThreshold;
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    int Depth = Index.Depth;
    if (LodOverride) return false;
    if (Depth <= DepthBounds[0]) return false;
    if (Depth < DepthBounds[2]) return false;
    double Distance = FMath::Max(FVector::Dist(DualContourPosition, InCameraPosition), 1.0);
    //double Distance2 = FMath::Max(FVector::Dist(NormalizedPosition, InCameraPosition), 1.0);
    //double Distance = FMath::Min(Distance, Distance2);
    double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));
    double AngularSize = ((2.0 * Extent) / Distance) * FOVScale;

    return AngularSize < InScreenSpaceThreshold * 0.5;
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

        if (CurrentNode->Index.Depth == DepthBounds[0] && CurrentNode->IsSurface())
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

void FAdaptiveOctreeNode::ComputeNormalizedPosition(double InRadius) {
    FVector DirFromCenter = (DualContourPosition - TreeCenter).GetSafeNormal();
    NormalizedPosition = TreeCenter + DirFromCenter * InRadius;
}

void FAdaptiveOctreeNode::ComputeDualContourPosition()
{
    FQEF Qef;
    FVector MassPoint = FVector::ZeroVector;
    int32 ValidEdges = 0;

    for (int32 i = 0; i < 12; i++)
    {
        const FVoxelCorner* A = Corners[OctreeConstants::EdgePairs[i][0]].Get();
        const FVoxelCorner* B = Corners[OctreeConstants::EdgePairs[i][1]].Get();
        if (!A || !B) continue;

        double dA = A->Density;
        double dB = B->Density;
        if ((dA <= 0.0) == (dB <= 0.0)) continue;

        double Denom = dA - dB;
        double t = FMath::Abs(Denom) < 1e-12 ? 0.5 : FMath::Clamp(dA / Denom, 0.0, 1.0);

        FVector PA = A->GetPosition();
        FVector PB = B->GetPosition();
        FVector CrossingPoint = PA + t * (PB - PA);

        FVector Normal = FVector(FMath::Lerp(A->Normal, B->Normal, (float)t)).GetSafeNormal();

        Qef.AddPlane(CrossingPoint, Normal);
        MassPoint += CrossingPoint;
        ValidEdges++;
    }

    if (ValidEdges == 0)
    {
        DualContourPosition = Center;
        DualContourNormal = FVector::ZeroVector;
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
        DualContourPosition = (FVector::DistSquared(CalculatedPos, MassPoint) > Extent * Extent * 2.0)
            ? MassPoint
            : CalculatedPos;
    }

    DualContourPosition.X = FMath::Clamp(DualContourPosition.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(DualContourPosition.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(DualContourPosition.Z, Center.Z - Extent, Center.Z + Extent);

    DualContourNormal = Qef.GetAverageNormal();

    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal.X = (Corners[1]->Density + Corners[3]->Density + Corners[5]->Density + Corners[7]->Density) -
            (Corners[0]->Density + Corners[2]->Density + Corners[4]->Density + Corners[6]->Density);
        DualContourNormal.Y = (Corners[2]->Density + Corners[3]->Density + Corners[6]->Density + Corners[7]->Density) -
            (Corners[0]->Density + Corners[1]->Density + Corners[4]->Density + Corners[5]->Density);
        DualContourNormal.Z = (Corners[4]->Density + Corners[5]->Density + Corners[6]->Density + Corners[7]->Density) -
            (Corners[0]->Density + Corners[1]->Density + Corners[2]->Density + Corners[3]->Density);
        DualContourNormal.Normalize();
    }

    if (DualContourNormal.IsNearlyZero())
        DualContourNormal = (DualContourPosition - TreeCenter).GetSafeNormal();
}
