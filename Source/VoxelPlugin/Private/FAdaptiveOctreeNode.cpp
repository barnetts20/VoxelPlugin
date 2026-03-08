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
}

FAdaptiveOctreeNode::FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex, FVector InAnchorCenter)
{
    Index = InParent->Index;
    Index.PushChild(ChildIndex);

    ChunkDepth = InParent->ChunkDepth;

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

FVector FAdaptiveOctreeNode::GetInterpolatedNormal(FVector P, FAdaptiveOctree* InOwner)
{
    // Normalize P to [0, 1] range within the cell
    FCornerData LocalCorners[8];
    for (int i = 0; i < 8; i++)
    {
        LocalCorners[i] = InOwner->GetCornerData(Corners[i]);
    }

    double tx = (P.X - (Center.X - Extent)) / (2.0 * Extent);
    double ty = (P.Y - (Center.Y - Extent)) / (2.0 * Extent);
    double tz = (P.Z - (Center.Z - Extent)) / (2.0 * Extent);

    tx = FMath::Clamp(tx, 0.0, 1.0);
    ty = FMath::Clamp(ty, 0.0, 1.0);
    tz = FMath::Clamp(tz, 0.0, 1.0);

    // Trilinear interpolation of the 8 corner normals
    FVector n00 = FMath::Lerp(LocalCorners[0].Normal, LocalCorners[1].Normal, tx);
    FVector n01 = FMath::Lerp(LocalCorners[2].Normal, LocalCorners[3].Normal, tx);
    FVector n10 = FMath::Lerp(LocalCorners[4].Normal, LocalCorners[5].Normal, tx);
    FVector n11 = FMath::Lerp(LocalCorners[6].Normal, LocalCorners[7].Normal, tx);

    FVector n0 = FMath::Lerp(n00, n01, ty);
    FVector n1 = FMath::Lerp(n10, n11, ty);

    FVector FinalNormal = FMath::Lerp(n0, n1, tz);
    return FinalNormal.GetSafeNormal();
}

bool FAdaptiveOctreeNode::ContainsCorner(const FVector& P) const
{
    return
        P.X >= Center.X - Extent && P.X <= Center.X + Extent &&
        P.Y >= Center.Y - Extent && P.Y <= Center.Y + Extent &&
        P.Z >= Center.Z - Extent && P.Z <= Center.Z + Extent;
}

bool FAdaptiveOctreeNode::ContainsEdge(
    int32 EdgeIndex,
    FAdaptiveOctree* Owner) const
{
    if (Owner == nullptr)
    {
        return false;
    }

    const FOctreeEdge& Edge = Owner->GetEdgeData(EdgeIndex);

    const FVector& P0 =
        Owner->GetCornerData(Edge.Key.C0).Position;

    const FVector& P1 =
        Owner->GetCornerData(Edge.Key.C1).Position;

    if (!ContainsCorner(P0))
    {
        return false;
    }

    if (!ContainsCorner(P1))
    {
        return false;
    }

    return true;
}

void FAdaptiveOctreeNode::FinalizeFromExistingCorners(FAdaptiveOctree* InOwner)
{

    // 1. GATHER (Read-only phase)
    FCornerData LocalCorners[8];
    for (int i = 0; i < 8; i++)
    {
        LocalCorners[i] = InOwner->GetCornerData(Corners[i]);
    }

    // Compute normals from local corners if not already set by map
    for (int i = 0; i < 8; i++)
    {
        double d = LocalCorners[i].Density;
        int idxX = i ^ 1;
        int idxY = i ^ 2;
        int idxZ = i ^ 4;

        double dx = (OctreeConstants::Offsets[i].X < 0) ? (LocalCorners[idxX].Density - d) : (d - LocalCorners[idxX].Density);
        double dy = (OctreeConstants::Offsets[i].Y < 0) ? (LocalCorners[idxY].Density - d) : (d - LocalCorners[idxY].Density);
        double dz = (OctreeConstants::Offsets[i].Z < 0) ? (LocalCorners[idxZ].Density - d) : (d - LocalCorners[idxZ].Density);

        FVector Normal(dx, dy, dz);
        if (!Normal.Normalize())
            Normal = (LocalCorners[i].Position - AnchorCenter).GetSafeNormal();

        InOwner->SetCornerNormal(Corners[i], Normal);
    }

    SignChangeEdges.Reset();
    for (int i = 0; i < 12; i++)
    {
        int32 IdxA = Corners[OctreeConstants::EdgePairs[i][0]];
        int32 IdxB = Corners[OctreeConstants::EdgePairs[i][1]];

        FNodeEdge NewEdge(IdxA, IdxB, LocalCorners[OctreeConstants::EdgePairs[i][0]], LocalCorners[OctreeConstants::EdgePairs[i][1]]);
        if (NewEdge.SignChange) SignChangeEdges.Add(NewEdge);
    }

    ComputeDualContourPosition(InOwner);

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

TSharedPtr<FAdaptiveOctreeNode>
FAdaptiveOctreeNode::GetLeafForEdge(
    int32 EdgeIndex,
    FAdaptiveOctree* Owner)
{
    if (!Owner)
        return nullptr;

    // If leaf, best match
    if (bIsLeaf)
    {
        return AsShared();
    }

    // Search children
    for (int i = 0; i < 8; i++)
    {
        TSharedPtr<FAdaptiveOctreeNode> Child = Children[i];

        if (!Child.IsValid())
            continue;

        if (Child->ContainsEdge(EdgeIndex, Owner))
        {
            return Child->GetLeafForEdge(EdgeIndex, Owner);
        }
    }

    // fallback
    return AsShared();
}

void FAdaptiveOctreeNode::Split(FAdaptiveOctree* InOwner)
{
    if (!bIsLeaf) return;

    FVector NextAnchor = (Index.Depth == DepthBounds[0]) ? Center : AnchorCenter;

    // Every child corner position is Center + {-1,0,+1} * Extent on each axis.
    // AcquireCorner handles dedup — parent corners already exist in the map,
    // so they just get AddRef'd. New interior points get allocated.

    for (uint8 i = 0; i < 8; i++)
    {
        Children[i] = MakeShared<FAdaptiveOctreeNode>(AsShared(), i, NextAnchor);

        double ChildExtent = Children[i]->Extent;  // already set by child constructor
        FVector ChildCenter = Children[i]->Center;

        for (int j = 0; j < 8; j++)
        {
            FVector CornerPos = ChildCenter + OctreeConstants::Offsets[j] * ChildExtent;
            Children[i]->Corners[j] = InOwner->AcquireCorner(CornerPos);
        }
    }

    bIsLeaf = false;
}

void FAdaptiveOctreeNode::Merge(FAdaptiveOctree* InOwner)
{
    if (bIsLeaf) return;

    for (int i = 0; i < 8; i++)
    {
        if (Children[i].IsValid())
        {
            // Recursive merge first to clean up the bottom of the tree
            Children[i]->Merge(InOwner);

            // Clean up the corners of the child we are about to delete
            for (int j = 0; j < 8; j++)
            {
                int32 CornerIdx = Children[i]->Corners[j];

                // Use INDEX_NONE check instead of null check
                if (CornerIdx != INDEX_NONE)
                {
                    // ReleaseCorner handles the RefCount decrement and 
                    // potential removal from the TSparseArray/Map
                    InOwner->ReleaseCorner(CornerIdx);

                    // Mark as invalid to prevent double-release 
                    // (though the node is being deleted anyway)
                    Children[i]->Corners[j] = INDEX_NONE;
                }
            }

            // Finally, delete the child node
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

        if (!CurrentNode || CurrentNode->Index.Depth > ChunkDepth) continue;

        if (CurrentNode->Index.Depth == ChunkDepth && CurrentNode->IsSurfaceNode)
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

void FAdaptiveOctreeNode::ComputeDualContourPosition(FAdaptiveOctree* InOwner)
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
        double Denom = InOwner->GetCornerData(Edge.Corners[1]).Density - InOwner->GetCornerData(Edge.Corners[0]).Density;
        //if (FMath::Abs(Denom) < 1e-15) continue;

        FVector P = Edge.ZeroCrossingPoint;
        MassPoint += P;

        FVector Normal = GetInterpolatedNormal(P, InOwner);
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

    // Standard bounding box clamp
    DualContourPosition.X = FMath::Clamp(DualContourPosition.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(DualContourPosition.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(DualContourPosition.Z, Center.Z - Extent, Center.Z + Extent);

    DualContourNormal = Qef.GetAverageNormal();

    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal.X = (InOwner->GetCornerData(Corners[1]).Density + InOwner->GetCornerData(Corners[3]).Density + InOwner->GetCornerData(Corners[5]).Density + InOwner->GetCornerData(Corners[7]).Density) -
            (InOwner->GetCornerData(Corners[0]).Density + InOwner->GetCornerData(Corners[2]).Density + InOwner->GetCornerData(Corners[4]).Density + InOwner->GetCornerData(Corners[6]).Density);
        DualContourNormal.Y = (InOwner->GetCornerData(Corners[2]).Density + InOwner->GetCornerData(Corners[3]).Density + InOwner->GetCornerData(Corners[6]).Density + InOwner->GetCornerData(Corners[7]).Density) -
            (InOwner->GetCornerData(Corners[0]).Density + InOwner->GetCornerData(Corners[1]).Density + InOwner->GetCornerData(Corners[4]).Density + InOwner->GetCornerData(Corners[5]).Density);
        DualContourNormal.Z = (InOwner->GetCornerData(Corners[4]).Density + InOwner->GetCornerData(Corners[5]).Density + InOwner->GetCornerData(Corners[6]).Density + InOwner->GetCornerData(Corners[7]).Density) -
            (InOwner->GetCornerData(Corners[0]).Density + InOwner->GetCornerData(Corners[1]).Density + InOwner->GetCornerData(Corners[2]).Density + InOwner->GetCornerData(Corners[3]).Density);
        DualContourNormal.Normalize();
    }

    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal = (DualContourPosition - TreeCenter).GetSafeNormal();
    }

    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal = (DualContourPosition - TreeCenter).GetSafeNormal();
    }
}
