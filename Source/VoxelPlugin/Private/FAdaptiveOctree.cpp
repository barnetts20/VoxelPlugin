#include "FAdaptiveOctree.h"

// Constructor
FAdaptiveOctree::FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    RootExtent = InRootExtent;
    Root = MakeShared<FAdaptiveOctreeNode>(InDensityFunction, InCenter, InRootExtent, InMinDepth, InMaxDepth);
    {
        FRWScopeLock WriteLock(OctreeLock, SLT_Write);
        SplitToDepth(Root, ChunkDepth);
    }
    Chunks = GetSurfaceNodes();
}

void FAdaptiveOctree::InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial) {
    for (auto chunk : Chunks) {
        TSharedPtr<FMeshChunk> newChunk = MakeShared<FMeshChunk>();
        auto flatNode = FAdaptiveOctreeFlatNode(chunk);
        newChunk->Initialize(InParentActor, InMaterial, flatNode.Center, flatNode.Extent);
        newChunk->ChunkEdges = chunk->GetSurfaceEdges();
        MeshChunks.Add(newChunk);
        UpdateMeshChunkStreamData(newChunk);
    }
    MeshChunksInitialized = true;
}

void FAdaptiveOctree::SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth)
{
    if (!Node.IsValid()) return;

    if (Node->TreeIndex.Num() < InMinDepth)
    {
        Node->Split();
        for (int i = 0; i < 8; i++)
        {
            if (Node->Children[i])
            {
                SplitToDepth(Node->Children[i], InMinDepth);
            }
        }
    }
}

FVector2f ComputeTriplanarUV(FVector Position, FVector Normal)
{
    FVector2f UV;
    FVector AbsNormal = Normal.GetAbs();

    if (AbsNormal.X > AbsNormal.Y && AbsNormal.X > AbsNormal.Z)
    {
        UV = FVector2f(Position.Y, Position.Z) * 0.0001f;
    }
    else if (AbsNormal.Y > AbsNormal.X && AbsNormal.Y > AbsNormal.Z)
    {
        UV = FVector2f(Position.X, Position.Z) * 0.0001f;
    }
    else
    {
        UV = FVector2f(Position.X, Position.Y) * 0.0001f;
    }

    return UV;
};

struct FMeshVertex
{
    FVector Position;
    FVector Normal;
    FColor Color;
    FVector2f UV;

    bool operator==(const FMeshVertex& Other) const
    {
        // Ensure vertex equality comparison based on position and normal
        // You might want to adjust epsilon values based on your needs
        const float PositionEpsilon = 0.0001f;
        const float NormalEpsilon = 0.001f;

        return Position.Equals(Other.Position, PositionEpsilon) &&
            Normal.Equals(Other.Normal, NormalEpsilon);
    }
};

// Hash function for our vertex structure
uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    // Create a hash based on the position components
    // You can adjust this hashing method to be more or less sensitive
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}

void FAdaptiveOctree::UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk)
{
    auto PositionStream = InChunk->ChunkMeshData->GetPositionStream();
    auto TangentStream = InChunk->ChunkMeshData->GetTangentStream();
    auto ColorStream = InChunk->ChunkMeshData->GetColorStream();
    auto TexCoordStream = InChunk->ChunkMeshData->GetTexCoordStream();
    auto TriangleStream = InChunk->ChunkMeshData->GetTriangleStream();
    auto PolygroupStream = InChunk->ChunkMeshData->GetPolygroupStream();

    TArray<FNodeEdge> edgesToProcess = InChunk->ChunkEdges;

    // Reset all streams
    PositionStream.Empty();
    TangentStream.Empty();
    ColorStream.Empty();
    TexCoordStream.Empty();
    TriangleStream.Empty();
    PolygroupStream.Empty();

    // Map to store unique vertices and their indices
    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> Triangles;

    // Process all edges and generate triangles in parallel
    // Need critical sections for thread safety when modifying shared data structures
    FCriticalSection VertexMapLock;
    FCriticalSection TrianglesLock;

    // First, collect all valid edge data in parallel
    struct FEdgeVertexData
    {
        TArray<FMeshVertex> Vertices;
        bool IsValid;
    };

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(edgesToProcess.Num());

    ParallelFor(edgesToProcess.Num(), [&](int32 edgeIdx) {
        auto currentEdge = edgesToProcess[edgeIdx];
        TArray<FAdaptiveOctreeFlatNode> nodesToMesh = SampleSurfaceNodesAroundEdge(currentEdge);

        if (!InChunk->ShouldProcessEdge(currentEdge, nodesToMesh)) {
            AllEdgeData[edgeIdx].IsValid = false;
            return;
        }

        AllEdgeData[edgeIdx].IsValid = true;
        AllEdgeData[edgeIdx].Vertices.SetNumZeroed(nodesToMesh.Num());

        // Fill vertex data
        for (int i = 0; i < nodesToMesh.Num(); i++) {
            AllEdgeData[edgeIdx].Vertices[i].Position = nodesToMesh[i].DualContourPosition;
            AllEdgeData[edgeIdx].Vertices[i].Normal = nodesToMesh[i].DualContourNormal;
            AllEdgeData[edgeIdx].Vertices[i].Color = FColor::Green; // Replace with encoded density function
            AllEdgeData[edgeIdx].Vertices[i].UV = ComputeTriplanarUV(
                nodesToMesh[i].DualContourPosition,
                nodesToMesh[i].DualContourNormal
            );
        }
        });

    // Now process the collected data and build the mesh
    for (int32 edgeIdx = 0; edgeIdx < AllEdgeData.Num(); edgeIdx++) {
        if (!AllEdgeData[edgeIdx].IsValid)
            continue;

        const TArray<FMeshVertex>& EdgeVertices = AllEdgeData[edgeIdx].Vertices;

        // Get or add indices for each vertex
        TArray<int32> VertexIndices;
        VertexIndices.SetNumZeroed(EdgeVertices.Num());

        // Lock when accessing shared structures
        {
            FScopeLock Lock(&VertexMapLock);

            for (int i = 0; i < EdgeVertices.Num(); i++) {
                int32* ExistingIndex = VertexMap.Find(EdgeVertices[i]);

                if (ExistingIndex) {
                    // Use existing vertex index
                    VertexIndices[i] = *ExistingIndex;
                }
                else {
                    // Add new vertex
                    int32 NewIndex = UniqueVertices.Num();
                    UniqueVertices.Add(EdgeVertices[i]);
                    VertexMap.Add(EdgeVertices[i], NewIndex);
                    VertexIndices[i] = NewIndex;
                }
            }
        }

        // Add triangles
        if (EdgeVertices.Num() >= 3) {
            FScopeLock Lock(&TrianglesLock);

            // First triangle
            Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[1], VertexIndices[2]));

            // Second triangle if we have 4 vertices
            if (EdgeVertices.Num() == 4) {
                Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[3]));
            }
        }
    }

    // Now populate the streams with our unique vertices and triangles
    PositionStream.SetNumUninitialized(UniqueVertices.Num());
    TangentStream.SetNumUninitialized(UniqueVertices.Num());
    ColorStream.SetNumUninitialized(UniqueVertices.Num());
    TexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    TriangleStream.SetNumUninitialized(Triangles.Num());
    PolygroupStream.SetNumUninitialized(Triangles.Num());

    // Fill vertex data streams
    for (int32 VertIdx = 0; VertIdx < UniqueVertices.Num(); VertIdx++) {
        const FMeshVertex& Vertex = UniqueVertices[VertIdx];

        PositionStream.Set(VertIdx, Vertex.Position);

        FRealtimeMeshTangentsHighPrecision Tangent;
        Tangent.SetNormal(FVector3f(Vertex.Normal));
        TangentStream.Set(VertIdx, Tangent);

        ColorStream.Set(VertIdx, Vertex.Color);
        TexCoordStream.Set(VertIdx, Vertex.UV);
    }

    // Fill triangle data streams
    for (int32 TriIdx = 0; TriIdx < Triangles.Num(); TriIdx++) {
        TriangleStream.Set(TriIdx, Triangles[TriIdx]);
        PolygroupStream.Set(TriIdx, 0); // All in the same polygroup
    }

    // Update the mesh with our new data
    //InChunk->UpdateMeshData(*InChunk->ChunkMeshData);
    InChunk->IsDirty = true;
}

// Optional: Add a method to pre-compute a deterministic hash for a position to ensure consistent vertex ordering
uint32 FAdaptiveOctree::ComputePositionHash(const FVector& Position, float GridSize)
{
    // Snap the position to a grid to ensure deterministic results
    FVector SnappedPos = FVector(
        FMath::RoundToFloat(Position.X / GridSize) * GridSize,
        FMath::RoundToFloat(Position.Y / GridSize) * GridSize,
        FMath::RoundToFloat(Position.Z / GridSize) * GridSize
    );

    // Compute a hash based on the snapped position
    return FCrc::MemCrc32(&SnappedPos, sizeof(FVector));
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double LodFactor)
{
    if (!MeshChunksInitialized) return;
    {
        FRWScopeLock WriteLock(OctreeLock, SLT_Write);
        ParallelFor(Chunks.Num(), [&](int32 idx)
        {
            TArray<FNodeEdge> tChunkEdges;
            if (Chunks[idx]->UpdateLod(CameraPosition, LodFactor, tChunkEdges)) MeshChunks[idx]->ChunkEdges = tChunkEdges;
        });
        ParallelFor(MeshChunks.Num(), [&](int32 idx)
        {
            UpdateMeshChunkStreamData(MeshChunks[idx]);
        });
    }
}

void FAdaptiveOctree::UpdateMesh()
{
    if (!MeshChunksInitialized) return;
    {
        FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
        for (auto mChunk : MeshChunks) {
            mChunk->UpdateComponent();
        }
    }
}

// Retrieves all leaf nodes
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetLeaves()
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Leaves;
    if (!Root.IsValid()) return Leaves;

    TQueue<TSharedPtr<FAdaptiveOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(Root->AsShared());

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode.IsValid()) continue;

        if (CurrentNode->IsLeaf())
        {
            Leaves.Add(CurrentNode->AsShared());
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (CurrentNode->Children[i]) NodeQueue.Enqueue(CurrentNode->Children[i]);
            }
        }
    }

    return Leaves;
}

// Retrieves all surface nodes for meshing
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetSurfaceNodes()
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    if (!Root.IsValid()) return TArray<TSharedPtr<FAdaptiveOctreeNode>>();
    return Root->GetSurfaceNodes();
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetChunks()
{
    return Chunks;
}

FVector FAdaptiveOctree::CalculateSurfaceNormal(const FVector& Position)
{
    const double h = 0.01; // Small step size

    double dx = (DensityFunction(Position + FVector(h, 0, 0)) -
        DensityFunction(Position - FVector(h, 0, 0))) / (2.0 * h);
    double dy = (DensityFunction(Position + FVector(0, h, 0)) -
        DensityFunction(Position - FVector(0, h, 0))) / (2.0 * h);
    double dz = (DensityFunction(Position + FVector(0, 0, h)) -
        DensityFunction(Position - FVector(0, 0, h))) / (2.0 * h);

    FVector Gradient(dx, dy, dz);

    // Density functions typically return positive values outside and negative inside
    // So the gradient points outward from the surface - negate if you want normal pointing out
    if (DensityFunction(Position) > 0)
    {
        Gradient = -Gradient; // Flip direction if outside the surface
    }

    return Gradient.GetSafeNormal();
}

TArray<FAdaptiveOctreeFlatNode> FAdaptiveOctree::SampleSurfaceNodesAroundEdge(const FNodeEdge& Edge)
{
    //FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly); //May need to change to a write lock since it could potentially modify internal node data
    TArray<TSharedPtr<FAdaptiveOctreeNode>> SurfaceNodes;
    double SmallOffset = 0.001;

    // Compute perpendicular vectors
    FVector Perp1 = FVector::CrossProduct(Edge.EdgeDirection, FVector(1, 0, 0)).GetSafeNormal();
    if (Perp1.IsNearlyZero()) Perp1 = FVector::CrossProduct(Edge.EdgeDirection, FVector(0, 1, 0)).GetSafeNormal();
    FVector Perp2 = FVector::CrossProduct(Edge.EdgeDirection, Perp1).GetSafeNormal();

    // Generate sample offsets
    FVector OffsetA = Edge.ZeroCrossingPoint + (Perp1 * SmallOffset) + (Perp2 * SmallOffset);
    FVector OffsetB = Edge.ZeroCrossingPoint - (Perp1 * SmallOffset) + (Perp2 * SmallOffset);
    FVector OffsetC = Edge.ZeroCrossingPoint - (Perp1 * SmallOffset) - (Perp2 * SmallOffset);
    FVector OffsetD = Edge.ZeroCrossingPoint + (Perp1 * SmallOffset) - (Perp2 * SmallOffset);

    // Query octree for valid surface nodes
    TSharedPtr<FAdaptiveOctreeNode> NodeA = GetLeafNodeByPoint(OffsetA);
    TSharedPtr<FAdaptiveOctreeNode> NodeB = GetLeafNodeByPoint(OffsetB);
    TSharedPtr<FAdaptiveOctreeNode> NodeC = GetLeafNodeByPoint(OffsetC);
    TSharedPtr<FAdaptiveOctreeNode> NodeD = GetLeafNodeByPoint(OffsetD);

    //if (NodeA.IsValid()) SurfaceNodes.AddUnique(NodeA);
    //if (NodeB.IsValid()) SurfaceNodes.AddUnique(NodeB);
    //if (NodeC.IsValid()) SurfaceNodes.AddUnique(NodeC);
    //if (NodeD.IsValid()) SurfaceNodes.AddUnique(NodeD);

    TArray<FAdaptiveOctreeFlatNode> returnNodes;
    if (NodeA.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeA));
    if (NodeB.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeB));
    if (NodeC.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeC));
    if (NodeD.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeD));

    return returnNodes;
    //for (auto node : SurfaceNodes) {
    //    if (!node->IsSurfaceNode) {
    //        node->LodOverride = true; 
    //        node->Split();
    //    }
    //}

    //SurfaceNodes.Empty();

    //NodeA = GetLeafNodeByPoint(OffsetA);
    //NodeB = GetLeafNodeByPoint(OffsetB);
    //NodeC = GetLeafNodeByPoint(OffsetC);
    //NodeD = GetLeafNodeByPoint(OffsetD);

    //if (NodeA.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeA));
    //if (NodeB.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeB));
    //if (NodeC.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeC));
    //if (NodeD.IsValid()) returnNodes.AddUnique(FAdaptiveOctreeFlatNode(NodeD));

    //return returnNodes;
}

TSharedPtr<FAdaptiveOctreeNode> FAdaptiveOctree::GetLeafNodeByPoint(FVector Position)
{
    //FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;

    while (CurrentNode.IsValid())
    {
        if (CurrentNode->IsLeaf())
        {
            return CurrentNode;
        }

        int ChildIndex = 0;
        if (Position.X >= CurrentNode->Center.X) ChildIndex |= 1;
        if (Position.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
        if (Position.Z >= CurrentNode->Center.Z) ChildIndex |= 4;

        if (CurrentNode->Children[ChildIndex].IsValid())
        {
            CurrentNode = CurrentNode->Children[ChildIndex];
        }
        else
        {
            break;
        }
    }

    return nullptr;
}

// Clears the entire octree
void FAdaptiveOctree::Clear()
{
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Root.Reset();
}
