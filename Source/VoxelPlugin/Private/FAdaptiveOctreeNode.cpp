#include "FAdaptiveOctreeNode.h"
#include "FAdaptiveOctree.h"
#include "FNodeStructureProvider.h"

const bool FAdaptiveOctreeNode::IsLeaf()
{
    return !Children[0].IsValid();;
}

const bool FAdaptiveOctreeNode::IsRoot()
{
    return !Parent.IsValid();
}

const bool FAdaptiveOctreeNode::IsSurface() {
    if (!bDataReady) return false;
    for (int i = 0; i < 12; i++) {
        if (Edges[i]->GetSignChange() != 0) return true;
    }
    return false;
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

FVector3f FAdaptiveOctreeNode::GetInterpolatedNormal(FVector P)
{
    // Normalize P to [0, 1] range within the cell
    double tx = (P.X - (Center.X - Extent)) / (2.0 * Extent);
    double ty = (P.Y - (Center.Y - Extent)) / (2.0 * Extent);
    double tz = (P.Z - (Center.Z - Extent)) / (2.0 * Extent);

    tx = FMath::Clamp(tx, 0.0, 1.0);
    ty = FMath::Clamp(ty, 0.0, 1.0);
    tz = FMath::Clamp(tz, 0.0, 1.0);

    // Trilinear interpolation of the 8 corner normals
    FVector3f n00 = FMath::Lerp(Corners[0]->GetNormal(), Corners[1]->GetNormal(), tx);
    FVector3f n01 = FMath::Lerp(Corners[2]->GetNormal(), Corners[3]->GetNormal(), tx);
    FVector3f n10 = FMath::Lerp(Corners[4]->GetNormal(), Corners[5]->GetNormal(), tx);
    FVector3f n11 = FMath::Lerp(Corners[6]->GetNormal(), Corners[7]->GetNormal(), tx);

    FVector3f n0 = FMath::Lerp(n00, n01, ty);
    FVector3f n1 = FMath::Lerp(n10, n11, ty);

    FVector3f FinalNormal = FMath::Lerp(n0, n1, tz);
    return FinalNormal.GetSafeNormal();
}

void FAdaptiveOctreeNode::Split()
{
    if (!IsLeaf()) return;

    // Determine the anchor for children
    // If THIS node is at the ChunkDepth, its children will use its center as their anchor.
    // Otherwise, they inherit the current anchor.
    FVector NextAnchor = (Index.Depth == DepthBounds[0]) ? Center : AnchorCenter;

    for (uint8 i = 0; i < 8; i++)
    {
        // Update constructor to take NextAnchor
        Children[i] = MakeShared<FAdaptiveOctreeNode>( AsShared(), i, NextAnchor);
    }
}

void FAdaptiveOctreeNode::Merge()
{
    if (IsLeaf()) return;

    for (int i = 0; i < 8; i++)
    {
        Children[i].Reset();
    }
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

TArray<FVoxelEdge*> FAdaptiveOctreeNode::GetSignChangeEdges() const
{
    TArray<FVoxelEdge*> CollectedEdges;
    for (int i = 0; i < 12; i++) {
        if(Edges[i] && Edges[i]->GetSignChange()) CollectedEdges.Add(Edges[i]);
    }
    return CollectedEdges;
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

        if (CurrentNode->Index.Depth == ChunkDepth && CurrentNode->IsSurface())
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
    TArray<FVoxelEdge*> SignChangeEdges = GetSignChangeEdges();
    if (SignChangeEdges.Num() == 0)
    {
        DualContourNormal = FVector3f::ZeroVector;
        DualContourPosition = Center;
        return;
    }

    FQEF Qef;
    FVector MassPoint = FVector::ZeroVector;
    int32 ValidEdges = 0;

    for (FVoxelEdge* Edge : SignChangeEdges)
    {
        // 1. Get the pre-calculated zero crossing from the edge
        // This replaces the manual d0/d1 math you had before
        FVector P = Edge->GetZeroCrossingPoint();
        MassPoint += P;

        // 2. Use the edge's analytic normal 
        // (Either interpolated from corners or via DensityFunction gradient)
        FVector3f Normal = Edge->GetAverageNormal();

        Qef.AddPlane(P, Normal);
        ValidEdges++;
    }

    if (ValidEdges == 0)
    {
        DualContourPosition = Center;
        DualContourNormal = FVector3f::ZeroVector;
        return;
    }

    MassPoint /= (double)ValidEdges;

    // Use MassPoint for deep nodes to avoid QEF jitter, otherwise solve
    if (Index.Depth > DepthPrecisionFloor)
    {
        DualContourPosition = MassPoint;
    }
    else
    {
        double Error = 0.0;
        FVector CalculatedPos = Qef.Solve(Center, Extent, &Error);

        // Heuristic: If QEF result is wildly outside the "cluster", fallback to MassPoint
        if (FVector::DistSquared(CalculatedPos, MassPoint) > (Extent * Extent) * 2.0)
        {
            DualContourPosition = MassPoint;
        }
        else
        {
            DualContourPosition = CalculatedPos;
        }
    }

    // Standard bounding box clamp - keeps the vertex inside the Octree cell
    DualContourPosition.X = FMath::Clamp(DualContourPosition.X, Center.X - Extent, Center.X + Extent);
    DualContourPosition.Y = FMath::Clamp(DualContourPosition.Y, Center.Y - Extent, Center.Y + Extent);
    DualContourPosition.Z = FMath::Clamp(DualContourPosition.Z, Center.Z - Extent, Center.Z + Extent);

    // Finalize Normal
    DualContourNormal = Qef.GetAverageNormal();

    // Fallback 1: Central Difference from corners if QEF normal failed
    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal.X = (Corners[1]->GetDensity() + Corners[3]->GetDensity() + Corners[5]->GetDensity() + Corners[7]->GetDensity()) -
            (Corners[0]->GetDensity() + Corners[2]->GetDensity() + Corners[4]->GetDensity() + Corners[6]->GetDensity());
        DualContourNormal.Y = (Corners[2]->GetDensity() + Corners[3]->GetDensity() + Corners[6]->GetDensity() + Corners[7]->GetDensity()) -
            (Corners[0]->GetDensity() + Corners[1]->GetDensity() + Corners[4]->GetDensity() + Corners[5]->GetDensity());
        DualContourNormal.Z = (Corners[4]->GetDensity() + Corners[5]->GetDensity() + Corners[6]->GetDensity() + Corners[7]->GetDensity()) -
            (Corners[0]->GetDensity() + Corners[1]->GetDensity() + Corners[2]->GetDensity() + Corners[3]->GetDensity());
        DualContourNormal.Normalize();
    }

    // Fallback 2: Radial normal (Planetary up-vector)
    if (DualContourNormal.IsNearlyZero())
    {
        DualContourNormal = (FVector3f)(DualContourPosition - TreeCenter).GetSafeNormal();
    }
}

FAdaptiveOctreeNode::~FAdaptiveOctreeNode()
{
    // Release Corners
    for (int i = 0; i < 8; ++i)
    {
        if (Corners[i])
        {
            Corners[i]->Release();
            Corners[i] = nullptr;
        }
    }

    // Release Edges
    for (int i = 0; i < 12; ++i)
    {
        if (Edges[i])
        {
            Edges[i]->Release();
            Edges[i] = nullptr;
        }
    }

    // Release Faces
    for (int i = 0; i < 6; ++i)
    {
        if (Faces[i])
        {
            Faces[i]->Release();
            Faces[i] = nullptr;
        }
    }
}
