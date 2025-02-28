// Fill out your copyright notice in the Description page of Project Settings.


#include "FAdaptiveOctreeNodeMT.h"

FAdaptiveOctreeNodeMT::FAdaptiveOctreeNodeMT(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InExtent, int InMinMaxDepth[2])
{
    // Initialize member variables
    bIsLeaf = true;
    bIsRoot = true;
    bIsSurface = false;
    bNeedsMeshUpdate = true;
    DensityFunction = InDensityFunction;
    Extent = InExtent;
    // Initialize depth constraints
    MinMaxDepth[0] = InMinMaxDepth[0];
    MinMaxDepth[1] = InMinMaxDepth[1];

    // Initialize tree index (root node has empty index)
    TreeIndex.Empty();

    Center = FSamplePosition(8, InCenter, FVector::ZeroVector, DensityFunction(InCenter));
    // Initialize corners with sample positions
    for (int i = 0; i < 8; i++) {
        FVector CornerPos = Center.Position + FVoxelStructures::CornerOffsets[i] * Extent;
        double SampleDensity = DensityFunction(CornerPos);
        // Initialize with a default normal - will be computed properly later
        Corners[i] = FSamplePosition(i, CornerPos, FVector::ZeroVector, SampleDensity);
    }
    ComputeSampleNormals();

    // Initialize edges
    for (int i = 0; i < 12; i++) {
        FSamplePosition EndPoints[2] = {
            Corners[FVoxelStructures::EdgePairs[i][0]],
            Corners[FVoxelStructures::EdgePairs[i][1]]
        };
        Edges[i] = FVoxelEdge(i, EndPoints);
    }

    // Initialize faces
    for (int i = 0; i < 6; i++) {
        FVoxelEdge FaceEdges[4] = {
            Edges[FVoxelStructures::FaceEdges[i][0]],
            Edges[FVoxelStructures::FaceEdges[i][1]],
            Edges[FVoxelStructures::FaceEdges[i][2]],
            Edges[FVoxelStructures::FaceEdges[i][3]]
        };
        Faces[i] = FQuadFace(i, FaceEdges);
        FaceNeighborMap.Add(i);
    }

    // Initialize base tetrahedra for all nodes
    for (int i = 0; i < 6; i++) {
        Tetra[i] = FTetrahedron(i, Faces[i], Center);
    }

    // Check if this is a surface node by examining density signs
    double FirstDensity = Center.Density;
    bool AllSameSigns = true;
    for (int i = 1; i < 8; i++) {
        if ((FirstDensity < 0) != (Corners[i].Density < 0)) {
            AllSameSigns = false;
            break;
        }
    }

    // If not all samples have the same sign, this is a surface node
    bIsSurface = !AllSameSigns;
}

FAdaptiveOctreeNodeMT::FAdaptiveOctreeNodeMT(TFunction<double(FVector)> InDensityFunction, TSharedPtr<FAdaptiveOctreeNodeMT> InParent, uint8 InChildIndex)
{
    // Set parent relationship
    Parent = InParent;
    // Initialize member variables
    bIsLeaf = true;
    bIsRoot = false;
    bIsSurface = false;
    bNeedsMeshUpdate = true;
    DensityFunction = InDensityFunction;
    Extent = InParent->Extent * 0.5;

    // Initialize depth constraints (inherit from parent)
    MinMaxDepth[0] = InParent->MinMaxDepth[0];
    MinMaxDepth[1] = InParent->MinMaxDepth[1];

    // Initialize tree index (append child index to parent's index)
    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(InChildIndex);

    // Initialize the center
    FVector CenterPos = InParent->Center.Position + FVoxelStructures::CornerOffsets[InChildIndex] * Extent;
    Center = FSamplePosition(8, CenterPos, FVector::ZeroVector, DensityFunction(CenterPos));

    // Initialize corners with sample positions
    for (int i = 0; i < 8; i++) {
        FVector CornerPos = Center.Position + FVoxelStructures::CornerOffsets[i] * Extent;
        double SampleDensity = DensityFunction(CornerPos);
        // Initialize with a default normal - will be computed properly later
        Corners[i] = FSamplePosition(i, CornerPos, FVector::ZeroVector, SampleDensity);
    }

    // Compute normals for all sample positions
    ComputeSampleNormals();

    // Initialize edges
    for (int i = 0; i < 12; i++) {
        FSamplePosition EndPoints[2] = {
            Corners[FVoxelStructures::EdgePairs[i][0]],
            Corners[FVoxelStructures::EdgePairs[i][1]]
        };
        Edges[i] = FVoxelEdge(i, EndPoints);
    }

    // Initialize faces
    for (int i = 0; i < 6; i++) {
        FVoxelEdge FaceEdges[4] = {
            Edges[FVoxelStructures::FaceEdges[i][0]],
            Edges[FVoxelStructures::FaceEdges[i][1]],
            Edges[FVoxelStructures::FaceEdges[i][2]],
            Edges[FVoxelStructures::FaceEdges[i][3]]
        };
        Faces[i] = FQuadFace(i, FaceEdges);
    }

    // Initialize base tetrahedra for all nodes
    for (int i = 0; i < 6; i++) {
        Tetra[i] = FTetrahedron(i, Faces[i], Center);
    }

    // Check if this is a surface node by examining density signs
    double FirstDensity = Center.Density;
    bool AllSameSigns = true;
    for (int i = 0; i < 8; i++) {
        if ((FirstDensity < 0) != (Corners[i].Density < 0)) {
            AllSameSigns = false;
            break;
        }
    }

    // If not all samples have the same sign, this is a surface node
    bIsSurface = !AllSameSigns;
}

void FAdaptiveOctreeNodeMT::ComputeSampleNormals()
{
    // Calculate normal for center sample using central differences
    FVector CenterGradient = FVector::ZeroVector;

    // For the center, we can use pairs of corner samples to estimate gradient
    // X-axis gradient (using average of 4 pairs of corners across the X axis)
    double dX = 0.0;
    dX += Corners[1].Density - Corners[0].Density; // Front-bottom edge
    dX += Corners[3].Density - Corners[2].Density; // Front-top edge
    dX += Corners[5].Density - Corners[4].Density; // Back-bottom edge
    dX += Corners[7].Density - Corners[6].Density; // Back-top edge
    dX /= 4.0 * (2.0 * Extent); // Normalize by distance between opposite corners

    // Y-axis gradient
    double dY = 0.0;
    dY += Corners[2].Density - Corners[0].Density; // Front-left edge
    dY += Corners[3].Density - Corners[1].Density; // Front-right edge
    dY += Corners[6].Density - Corners[4].Density; // Back-left edge
    dY += Corners[7].Density - Corners[5].Density; // Back-right edge
    dY /= 4.0 * (2.0 * Extent);

    // Z-axis gradient
    double dZ = 0.0;
    dZ += Corners[4].Density - Corners[0].Density; // Left-bottom edge
    dZ += Corners[5].Density - Corners[1].Density; // Right-bottom edge
    dZ += Corners[6].Density - Corners[2].Density; // Left-top edge
    dZ += Corners[7].Density - Corners[3].Density; // Right-top edge
    dZ /= 4.0 * (2.0 * Extent);

    // Density gradient points from negative to positive, so we negate it for normal direction
    CenterGradient = FVector(-dX, -dY, -dZ).GetSafeNormal();
    Center.Normal = CenterGradient;

    // Calculate normals for corner samples
    for (int i = 0; i < 8; i++)
    {
        // For corners, we can use adjacent corners and center to estimate gradient
        FVector CornerGradient = FVector::ZeroVector;
        int OppositeCornerIndex = 7 - i; // Opposite corner in the cube

        // Component-wise differences
        double diffX = 0.0, diffY = 0.0, diffZ = 0.0;
        int countX = 0, countY = 0, countZ = 0;

        // Check neighbors in the X direction
        int xNeighbor = i ^ 1; // Flip the least significant bit to get X neighbor
        if (xNeighbor != i) {
            diffX += Corners[xNeighbor].Density - Corners[i].Density;
            countX++;
        }

        // Check neighbors in the Y direction
        int yNeighbor = i ^ 2; // Flip the second bit to get Y neighbor
        if (yNeighbor != i) {
            diffY += Corners[yNeighbor].Density - Corners[i].Density;
            countY++;
        }

        // Check neighbors in the Z direction
        int zNeighbor = i ^ 4; // Flip the third bit to get Z neighbor
        if (zNeighbor != i) {
            diffZ += Corners[zNeighbor].Density - Corners[i].Density;
            countZ++;
        }

        // Also use the center for additional precision
        diffX += (Center.Density - Corners[i].Density) * (Center.Position.X - Corners[i].Position.X) / (Center.Position - Corners[i].Position).Size();
        diffY += (Center.Density - Corners[i].Density) * (Center.Position.Y - Corners[i].Position.Y) / (Center.Position - Corners[i].Position).Size();
        diffZ += (Center.Density - Corners[i].Density) * (Center.Position.Z - Corners[i].Position.Z) / (Center.Position - Corners[i].Position).Size();
        countX++; countY++; countZ++;

        // Normalize by count and distance
        diffX /= countX * (2.0 * Extent / countX);
        diffY /= countY * (2.0 * Extent / countY);
        diffZ /= countZ * (2.0 * Extent / countZ);

        // Density gradient points from negative to positive, so we negate it for normal direction
        CornerGradient = FVector(-diffX, -diffY, -diffZ).GetSafeNormal();
        Corners[i].Normal = CornerGradient;
    }
}

bool FAdaptiveOctreeNodeMT::IsLeaf() const
{
    return bIsLeaf;
}

bool FAdaptiveOctreeNodeMT::IsRoot() const
{
    return bIsRoot;
}

bool FAdaptiveOctreeNodeMT::IsSurface() const
{
    return bIsSurface;
}

bool FAdaptiveOctreeNodeMT::IsSurfaceLeaf() const
{
    return bIsSurface && bIsLeaf;
}

bool FAdaptiveOctreeNodeMT::ShouldSplit()
{
    return TreeIndex.Num() < MinMaxDepth[0] || (FVector::Dist(Center.Position, CameraData.Position) < Extent * (LODFactor + TreeIndex.Num()) && TreeIndex.Num() < MinMaxDepth[1]);
}

void FAdaptiveOctreeNodeMT::Split()
{
    if (!bIsLeaf) return; // Already split
    for (uint8 i = 0; i < 8; i++)
    {
        Children[i] = MakeShared<FAdaptiveOctreeNodeMT>(DensityFunction, AsShared(), i);
    }
    bIsLeaf = false;
}

bool FAdaptiveOctreeNodeMT::ShouldMerge()
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
    return CanMerge && (FVector::Dist(Center.Position, CameraData.Position) > Extent * (LODFactor + TreeIndex.Num())) && TreeIndex.Num() >= MinMaxDepth[0];
}

void FAdaptiveOctreeNodeMT::Merge()
{
    if (bIsLeaf) return; // Already merged
    TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> NodesToDelete;
    NodesToDelete.Append(Children); // Add children to cleanup list
    while (NodesToDelete.Num() > 0)
    {
        TSharedPtr<FAdaptiveOctreeNodeMT> Node = NodesToDelete.Pop();
        if (!Node.IsValid()) continue;
        if (!Node->bIsLeaf) {
            NodesToDelete.Append(Node->Children);
        }
        for (int i = 0; i < 8; i++) {
            Node->Children[i].Reset();
        }
        Node.Reset();
    }
    bIsLeaf = true;
}

void FAdaptiveOctreeNodeMT::SplitToDepth(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, int InDepth)
{
    if (!InNode.IsValid()) return;

    if (InNode->TreeIndex.Num() < InDepth)
    {
        InNode->Split();
        for (int i = 0; i < 8; i++)
        {
            if (InNode->Children[i])
            {
                SplitToDepth(InNode->Children[i], InDepth);
            }
        }
    }
}

TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> FAdaptiveOctreeNodeMT::GetSurfaceLeafNodes(TSharedPtr<FAdaptiveOctreeNodeMT> InNode)
{
    TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> SurfaceLeafNodes;
    TQueue<TSharedPtr<FAdaptiveOctreeNodeMT>> NodeQueue;
    NodeQueue.Enqueue(InNode);
    while (!NodeQueue.IsEmpty()) {
        TSharedPtr<FAdaptiveOctreeNodeMT> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);
        if (!CurrentNode) continue;
        if (CurrentNode->IsSurfaceLeaf()) {
            SurfaceLeafNodes.Add(CurrentNode);
        }
        if (!CurrentNode->IsLeaf()) {
            for (const TSharedPtr<FAdaptiveOctreeNodeMT>& child : CurrentNode->Children) {
                NodeQueue.Enqueue(child);
            }
        }
    }
    return SurfaceLeafNodes;
}

TSharedPtr<FAdaptiveOctreeNodeMT> FAdaptiveOctreeNodeMT::SampleSurfaceLeafByPosition(FVector SamplePosition) {
    //TODO: crawl up parent heirarchy until we find a node containing the position, nullptr if not contained at root
    //TODO: crawl back down heirarchy until we either find a surface leaf contianing the position or nullptr if none was found
}

void FAdaptiveOctreeNodeMT::UpdateNeighbors()
{
    bool NeighborsChanged = false;
    //TODO: Sample neigbors update faceidx/neighbor map
    
    if (NeighborsChanged) bNeedsMeshUpdate = true;
}

FInternalMeshBuffer FAdaptiveOctreeNodeMT::ComputeGeometry()
{
    if (bNeedsMeshUpdate) {
        //TODO: Update mesh data
        bNeedsMeshUpdate = false;
    }
    return MeshData;
}

bool FAdaptiveOctreeNodeMT::UpdateLOD(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FCameraInfo InCameraData, double InLODFactor)
{
    InNode->CameraData = InCameraData;
    InNode->LODFactor = InLODFactor;
    if (InNode->IsLeaf())
    {
        if (InNode->ShouldSplit())
        {
            InNode->Split();
            return true; // A split occurred
        }
        else if (InNode->Parent.IsValid() && InNode->TreeIndex.Last() == 7)
        {
            auto tParent = InNode->Parent.Pin();
            tParent->CameraData = InCameraData;
            tParent->LODFactor = InLODFactor;

            if (tParent->ShouldMerge()) {
                tParent->Merge();
                return true;
            }
        }
    }
    else
    {
        bool bAnyChanges = false;
        for (int i = 0; i < 8; i++)
        {
            if (InNode->Children[i]->FAdaptiveOctreeNodeMT::UpdateLOD(InNode->Children[i], InCameraData, InLODFactor))
            {
                bAnyChanges = true;
            }
        }
        return bAnyChanges; // Return true if any child changed
    }

    return false; // No changes occurred
}

void FAdaptiveOctreeNodeMT::UpdateNeighborData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode)
{
}

void FAdaptiveOctreeNodeMT::RegenerateMeshData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FInternalMeshBuffer& OutMeshBuffer)
{
}


