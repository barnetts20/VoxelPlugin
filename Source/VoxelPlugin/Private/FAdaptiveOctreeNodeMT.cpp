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
    NodeMinMax[0] = Center.Position - FVector(Extent);
    NodeMinMax[1] = Center.Position + FVector(Extent);
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
        NeighborCenters[i] = FVoxelStructures::FaceAxes[i] * 2 * Extent + Center.Position;
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
    NodeMinMax[0] = Center.Position - FVector(Extent);
    NodeMinMax[1] = Center.Position + FVector(Extent);
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
        NeighborCenters[i] = FVoxelStructures::FaceAxes[i] * 2 * Extent + Center.Position; 
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

bool FAdaptiveOctreeNodeMT::ContainsPosition(FVector InPosition) {  
    return (InPosition.X >= NodeMinMax[0].X && InPosition.X <= NodeMinMax[1].X 
         && InPosition.Y >= NodeMinMax[0].Y && InPosition.Y <= NodeMinMax[1].Y 
         && InPosition.Z >= NodeMinMax[0].Z && InPosition.Z <= NodeMinMax[1].Z);
}

bool FAdaptiveOctreeNodeMT::ShouldSplit()
{
    return ShouldSplit(Center.Position);
}

bool FAdaptiveOctreeNodeMT::ShouldSplit(const FVector VirtualCenter)
{
    // Return true if splitting would occur at the Virtual Center
    return TreeIndex.Num() < MinMaxDepth[0] || (FVector::Dist(VirtualCenter, CameraData.Position) < Extent * (LODFactor + TreeIndex.Num()) && TreeIndex.Num() < MinMaxDepth[1]);
}

void FAdaptiveOctreeNodeMT::Split()
{
    if (!bIsLeaf) return;
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
    if (bIsLeaf) return;
    TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> NodesToDelete;
    NodesToDelete.Append(Children);
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

TSharedPtr<FAdaptiveOctreeNodeMT> FAdaptiveOctreeNodeMT::SampleLeafByPosition(FVector SamplePosition) {
    TSharedPtr<FAdaptiveOctreeNodeMT> ContainingNode = AsShared();

    while (ContainingNode.IsValid() && !ContainingNode->ContainsPosition(SamplePosition)) {
        ContainingNode = ContainingNode->Parent.Pin();
    }

    while (ContainingNode.IsValid() && !ContainingNode->IsLeaf()) {
        // Determine which child would contain the position
        uint8 ChildIdx = 0;
        if (SamplePosition.X >= ContainingNode->Center.Position.X) ChildIdx |= 1;
        if (SamplePosition.Y >= ContainingNode->Center.Position.Y) ChildIdx |= 2;
        if (SamplePosition.Z >= ContainingNode->Center.Position.Z) ChildIdx |= 4;

        ContainingNode = ContainingNode->Children[ChildIdx];
    }
    return ContainingNode;
}

bool FAdaptiveOctreeNodeMT::UpdateNeighbors()
{
    bool bNeighborChanged = false;
    
    for (int i = 0; i < 6; i++) {
        bool bNeighborSplit = ShouldSplit(NeighborCenters[i]);
        if (NeighborLODChanges[i] != bNeighborSplit) {
            NeighborLODChanges[i] = bNeighborSplit;
            bNeighborChanged = true;
        }
    }

    if(bNeighborChanged) bNeedsMeshUpdate = true;

    return bNeighborChanged;
}

FInternalMeshBuffer FAdaptiveOctreeNodeMT::ComputeGeometry()
{
    if (bNeedsMeshUpdate) {
        // Clear any existing mesh data
        MeshData.Positions.Empty();
        MeshData.Normals.Empty();
        MeshData.Triangles.Empty();

        // We already have sample positions for all corners and center
        // No need to recalculate densities

        // Create all 12 edges of the cube
        TArray<FVoxelEdge> CubeEdges;
        CubeEdges.SetNum(12);

        for (int i = 0; i < 12; i++) {
            FSamplePosition EdgeCorners[2] = {
                Corners[FVoxelStructures::EdgePairs[i][0]],
                Corners[FVoxelStructures::EdgePairs[i][1]]
            };

            FVoxelEdge Edge(i, EdgeCorners);
            CubeEdges[i] = Edge;
        }

        // Create the 6 faces of the cube
        TArray<FQuadFace> CubeFaces;
        CubeFaces.SetNum(6);

        for (int i = 0; i < 6; i++) {
            FVoxelEdge FaceEdges[4] = {
                CubeEdges[FVoxelStructures::FaceEdges[i][0]],
                CubeEdges[FVoxelStructures::FaceEdges[i][1]],
                CubeEdges[FVoxelStructures::FaceEdges[i][2]],
                CubeEdges[FVoxelStructures::FaceEdges[i][3]]
            };

            CubeFaces[i] = FQuadFace(i, FaceEdges);
        }

        // Process each tetrahedron (6 per cube)
        for (int tetraIdx = 0; tetraIdx < 6; tetraIdx++) {
            // Get corner indices for this tetrahedron
            FSamplePosition TetraCorners[4] = {
                Corners[FVoxelStructures::TetrahedronCorners[tetraIdx][0]],
                Corners[FVoxelStructures::TetrahedronCorners[tetraIdx][1]],
                Corners[FVoxelStructures::TetrahedronCorners[tetraIdx][2]],
                Center // Using pre-calculated center
            };

            // Create the tetrahedron edges (6 per tetrahedron)
            TArray<FVoxelEdge> TetraEdges;
            TetraEdges.SetNum(6);

            for (int edgeIdx = 0; edgeIdx < 6; edgeIdx++) {
                int corner1Idx = FVoxelStructures::TetrahedronEdges[tetraIdx][edgeIdx][0];
                int corner2Idx = FVoxelStructures::TetrahedronEdges[tetraIdx][edgeIdx][1];

                // Handle both cube edges and edges to the center
                FSamplePosition EdgeCorners[2];

                // Map corner indices correctly
                if (corner1Idx < 8) {
                    EdgeCorners[0] = Corners[corner1Idx];
                }
                else {
                    EdgeCorners[0] = Center;
                }

                if (corner2Idx < 8) {
                    EdgeCorners[1] = Corners[corner2Idx];
                }
                else {
                    EdgeCorners[1] = Center;
                }

                // Create the edge (this will calculate zero crossing if needed)
                TetraEdges[edgeIdx] = FVoxelEdge(edgeIdx, EdgeCorners);
            }

            // Determine the case from the tetrahedron lookup table
            uint8 caseIndex = 0;
            for (int i = 0; i < 4; i++) {
                if (TetraCorners[i].Density < 0.0)
                    caseIndex |= (1 << i);
            }

            // Check if this case produces triangles
            const int* triangleIndices = FVoxelStructures::TetrahedronTriTable[caseIndex];
            if (triangleIndices[0] != -1) {
                // This case produces triangles
                int vertexOffset = MeshData.Positions.Num();
                int numTriangleVertices = 0;

                // Count how many triangle vertices we need to add
                for (int i = 0; triangleIndices[i] != -1; i++) {
                    numTriangleVertices++;
                }

                // For each edge in the triangulation
                for (int i = 0; i < numTriangleVertices; i++) {
                    int edgeIdx = triangleIndices[i];
                    FVoxelEdge& edge = TetraEdges[edgeIdx];

                    // Add vertex at zero crossing point
                    MeshData.Positions.Add(edge.ZeroCrossingPoint);
                    MeshData.Normals.Add(edge.Normal);
                }

                // Add triangles (every three vertices form a triangle)
                for (int i = 0; i < numTriangleVertices / 3; i++) {
                    // Add triangle indices in CCW order
                    MeshData.Triangles.Add(vertexOffset + i * 3);
                    MeshData.Triangles.Add(vertexOffset + i * 3 + 1);
                    MeshData.Triangles.Add(vertexOffset + i * 3 + 2);
                }
            }
        }

        // Handle LOD transitions if needed TODO: SKIPPING FOR NOW I WANT TO SEE THE CRACKS BEFORE WE START MESSING WITH IT
        //for (int faceIdx = 0; faceIdx < 6; faceIdx++) {
        //    if (NeighborLODChanges[faceIdx]) {
        //        // This face has an LOD transition
        //        StitchLODTransition(faceIdx, Corners, CubeEdges, CubeFaces[faceIdx]);
        //    }
        //}

        bNeedsMeshUpdate = false;
    }

    return MeshData;
}

void FAdaptiveOctreeNodeMT::StitchLODTransition(int faceIdx, const FSamplePosition InCorners[8], const TArray<FVoxelEdge>& CubeEdges, const FQuadFace& Face)
{
    // Get the four corners of the face
    const int* cornerIndices = FVoxelStructures::FaceCorners[faceIdx];

    // Get mid-points of each edge of the face
    FVector edgeMidPoints[4];
    for (int i = 0; i < 4; i++) {
        int edgeIdx = FVoxelStructures::FaceEdges[faceIdx][i];
        edgeMidPoints[i] = CubeEdges[edgeIdx].MidPoint;
    }

    // Get face mid-point
    FVector faceMidPoint = Face.MidPoint;

    // For density at mid-points, interpolate from corner values
    double edgeMidDensities[4];
    for (int i = 0; i < 4; i++) {
        int edgeIdx = FVoxelStructures::FaceEdges[faceIdx][i];
        int cornerA = FVoxelStructures::EdgePairs[edgeIdx][0];
        int cornerB = FVoxelStructures::EdgePairs[edgeIdx][1];
        edgeMidDensities[i] = (InCorners[cornerA].Density + InCorners[cornerB].Density) * 0.5f;
    }

    // Density at face center (average of corner densities)
    double faceCenterDensity = 0.0;
    for (int i = 0; i < 4; i++) {
        faceCenterDensity += InCorners[cornerIndices[i]].Density;
    }
    faceCenterDensity *= 0.25f;

    // Create face center vertex
    int faceCenterIdx = MeshData.Positions.Num();
    MeshData.Positions.Add(faceMidPoint);

    // Calculate normal at face center (average of corner normals)
    FVector faceNormal = FVector::ZeroVector;
    for (int i = 0; i < 4; i++) {
        faceNormal += InCorners[cornerIndices[i]].Normal;
    }
    faceNormal = faceNormal.GetSafeNormal();
    MeshData.Normals.Add(faceNormal);

    // Process each edge of the face
    int edgeMidIndices[4];
    for (int i = 0; i < 4; i++) {
        edgeMidIndices[i] = MeshData.Positions.Num();
        MeshData.Positions.Add(edgeMidPoints[i]);

        // Interpolate normal at edge midpoint
        int edgeIdx = FVoxelStructures::FaceEdges[faceIdx][i];
        int cornerA = FVoxelStructures::EdgePairs[edgeIdx][0];
        int cornerB = FVoxelStructures::EdgePairs[edgeIdx][1];
        FVector edgeNormal = (InCorners[cornerA].Normal + InCorners[cornerB].Normal).GetSafeNormal();
        MeshData.Normals.Add(edgeNormal);
    }

    // Now create connecting triangles for the stitching pattern
    // This is a simple pattern that connects each corner to adjacent edge midpoints and face center
    for (int i = 0; i < 4; i++) {
        int cornerVertexIdx = -1;
        int cornerIdx = cornerIndices[i];

        // First, check if we already have a vertex for this corner
        bool cornerHasIsosurfaceIntersection = false;
        for (int j = 0; j < MeshData.Positions.Num(); j++) {
            // This would need a proper way to detect if the position matches the corner
            // For simplicity, we'll add the corner anyway
            if (InCorners[cornerIdx].Position.Equals(MeshData.Positions[j], 0.001f)) {
                cornerVertexIdx = j;
                cornerHasIsosurfaceIntersection = true;
                break;
            }
        }

        // If not already in the mesh, add it
        if (cornerVertexIdx == -1) {
            cornerVertexIdx = MeshData.Positions.Num();
            MeshData.Positions.Add(InCorners[cornerIdx].Position);
            MeshData.Normals.Add(InCorners[cornerIdx].Normal);
        }

        // Get indices of adjacent edge midpoints
        int nextEdge = i;
        int prevEdge = (i + 3) % 4; // "Previous" edge in the face loop

        // Create triangles for this corner
        // Triangle 1: corner -> edge midpoint -> face center
        MeshData.Triangles.Add(cornerVertexIdx);
        MeshData.Triangles.Add(edgeMidIndices[nextEdge]);
        MeshData.Triangles.Add(faceCenterIdx);

        // Triangle 2: corner -> face center -> previous edge midpoint
        MeshData.Triangles.Add(cornerVertexIdx);
        MeshData.Triangles.Add(faceCenterIdx);
        MeshData.Triangles.Add(edgeMidIndices[prevEdge]);
    }
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
            for (TSharedPtr<FAdaptiveOctreeNodeMT> Child : InNode->Children) {
                if(Child->IsSurfaceLeaf()) Child->UpdateNeighbors();
            }
            return true;
        }
        else if (InNode->Parent.IsValid() && InNode->TreeIndex.Last() == 7)
        {
            auto tParent = InNode->Parent.Pin();
            tParent->CameraData = InCameraData;
            tParent->LODFactor = InLODFactor;

            if (tParent->ShouldMerge()) {
                tParent->Merge();
                if (tParent->IsSurfaceLeaf()) tParent->UpdateNeighbors();
                return true;
            }
        }
        else {
            if(InNode->IsSurfaceLeaf()) InNode->UpdateNeighbors();
        }
    }
    else
    {
        bool bAnyChanges = false;
        for (int i = 0; i < 8; i++)
        {
            if (FAdaptiveOctreeNodeMT::UpdateLOD(InNode->Children[i], InCameraData, InLODFactor))
            {
                bAnyChanges = true;
            }
        }
        return bAnyChanges;
    }
    return false;
}

void FAdaptiveOctreeNodeMT::RegenerateMeshData(TSharedPtr<FAdaptiveOctreeNodeMT> InNode, FInternalMeshBuffer& OutMeshBuffer)
{
    TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> SurfaceNodes = FAdaptiveOctreeNodeMT::GetSurfaceLeafNodes(InNode);
    
    TArray<FInternalMeshBuffer> PerNodeMeshBuffers;
    PerNodeMeshBuffers.SetNum(SurfaceNodes.Num());
    //TODO: PARALLEL FOR HERE
    ParallelFor(SurfaceNodes.Num(), [&](int32 idx) {
        if (SurfaceNodes[idx]->bNeedsMeshUpdate) SurfaceNodes[idx]->ComputeGeometry();

        PerNodeMeshBuffers[idx].Positions.Append(SurfaceNodes[idx]->MeshData.Positions);
        PerNodeMeshBuffers[idx].Normals.Append(SurfaceNodes[idx]->MeshData.Normals);
        PerNodeMeshBuffers[idx].Triangles.Append(SurfaceNodes[idx]->MeshData.Triangles);
    });
    for (auto mb : PerNodeMeshBuffers) {
        OutMeshBuffer.Append(mb);
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

