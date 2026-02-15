#include "FAdaptiveOctree.h"

// Constructor
FAdaptiveOctree::FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    RootExtent = InRootExtent;
    Root = MakeShared<FAdaptiveOctreeNode>(InDensityFunction, InCenter, InRootExtent, InMinDepth, InMaxDepth);
    {
        SplitToDepth(Root, InMinDepth);
    }
    //Chunks = GetLeaves();
    Chunks = GetNodesAtDepth(ChunkDepth);
}

void FAdaptiveOctree::InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial) {
    ParentActor = InParentActor;
    for (auto chunk : Chunks) {
        TSharedPtr<FMeshChunk> newChunk = MakeShared<FMeshChunk>();
        newChunk->Initialize(InParentActor, InMaterial, chunk->Center, chunk->Extent);
        newChunk->ChunkEdges = chunk->GetSurfaceEdges();
        UpdateMeshChunkStreamData(newChunk);
        MeshChunks.Add(newChunk);
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

FORCEINLINE FVector QuantizePosition(const FVector& P, double GridSize = 1.0)
{
    return FVector(
        FMath::RoundToDouble(P.X / GridSize) * GridSize,
        FMath::RoundToDouble(P.Y / GridSize) * GridSize,
        FMath::RoundToDouble(P.Z / GridSize) * GridSize
    );
}

struct FMeshVertex
{
    FVector Position;       // Quantized position (chunk-local)
    FVector OriginalPosition; // Unquantized for actual mesh output
    FVector Normal;
    FColor Color;
    FVector2f UV;

    bool operator==(const FMeshVertex& Other) const
    {
        // Exact comparison on quantized position — no epsilon needed
        return Position == Other.Position;
    }
};

uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    // Hash the quantized position — guaranteed consistent with operator==
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

    // Collect all valid edge data in parallel
    struct FEdgeVertexData
    {
        TArray<FMeshVertex> Vertices;
        TOptional<FNodeEdge> Edge;
        bool IsValid;
    };

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(InChunk->ChunkEdges.Num());

    double QuantizationGrid = InChunk->ChunkExtent * 0.0001;

    ParallelFor(InChunk->ChunkEdges.Num(), [&](int32 edgeIdx) {
        auto& currentEdge = InChunk->ChunkEdges[edgeIdx];
        TArray<TSharedPtr<FAdaptiveOctreeNode>> nodesToMesh = SampleNodesAroundEdge(currentEdge);
        if (!InChunk->ShouldProcessEdge(currentEdge, nodesToMesh)) {
            AllEdgeData[edgeIdx].IsValid = false;
            return;
        }

        double MaxVertexDistance = currentEdge.Size * 5.0;
        TArray<TSharedPtr<FAdaptiveOctreeNode>> validNodes;
        for (auto& node : nodesToMesh) {
            double dist = FVector::Dist(node->DualContourPosition, currentEdge.ZeroCrossingPoint);
            if (dist <= MaxVertexDistance) {
                validNodes.Add(node);
            }
        }

        if (validNodes.Num() < 3) {
            AllEdgeData[edgeIdx].IsValid = false;
            return;
        }

        AllEdgeData[edgeIdx].IsValid = true;
        AllEdgeData[edgeIdx].Edge = currentEdge;
        AllEdgeData[edgeIdx].Vertices.SetNumZeroed(validNodes.Num());

        for (int i = 0; i < validNodes.Num(); i++) {
            FVector WorldPos = validNodes[i]->DualContourPosition;
            FVector LocalPos = WorldPos - InChunk->ChunkCenter;
            FVector n = validNodes[i]->DualContourNormal;

            AllEdgeData[edgeIdx].Vertices[i].Position = QuantizePosition(LocalPos, QuantizationGrid);
            AllEdgeData[edgeIdx].Vertices[i].OriginalPosition = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = n;
            AllEdgeData[edgeIdx].Vertices[i].Color = FColor::Green;
            AllEdgeData[edgeIdx].Vertices[i].UV = ComputeTriplanarUV(WorldPos, n);
        }
        });

    // Build mesh from collected data (single-threaded for TMap)
    for (int32 edgeIdx = 0; edgeIdx < AllEdgeData.Num(); edgeIdx++) {
        if (!AllEdgeData[edgeIdx].IsValid)
            continue;

        const auto& currentEdge = AllEdgeData[edgeIdx].Edge.GetValue();
        const TArray<FMeshVertex>& EdgeVertices = AllEdgeData[edgeIdx].Vertices;

        TArray<int32> VertexIndices;
        VertexIndices.SetNumZeroed(EdgeVertices.Num());

        for (int i = 0; i < EdgeVertices.Num(); i++) {
            int32* ExistingIndex = VertexMap.Find(EdgeVertices[i]);

            if (ExistingIndex) {
                VertexIndices[i] = *ExistingIndex;
            }
            else {
                int32 NewIndex = UniqueVertices.Num();
                UniqueVertices.Add(EdgeVertices[i]);
                VertexMap.Add(EdgeVertices[i], NewIndex);
                VertexIndices[i] = NewIndex;
            }
        }

        bool FlipWinding = (currentEdge.Corners[0].Density < 0);

        if (EdgeVertices.Num() >= 3
            && VertexIndices[0] != VertexIndices[1]
            && VertexIndices[1] != VertexIndices[2]
            && VertexIndices[0] != VertexIndices[2])
        {
            if (FlipWinding) {
                Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[1]));
            }
            else {
                Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[1], VertexIndices[2]));
            }

            if (EdgeVertices.Num() == 4
                && VertexIndices[0] != VertexIndices[3]
                && VertexIndices[2] != VertexIndices[3])
            {
                if (FlipWinding) {
                    Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[3], VertexIndices[2]));
                }
                else {
                    Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[3]));
                }
            }
        }
    }

    // Populate streams
    PositionStream.SetNumUninitialized(UniqueVertices.Num());
    TangentStream.SetNumUninitialized(UniqueVertices.Num());
    ColorStream.SetNumUninitialized(UniqueVertices.Num());
    TexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    TriangleStream.SetNumUninitialized(Triangles.Num());
    PolygroupStream.SetNumUninitialized(Triangles.Num());

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
        const FMeshVertex& Vertex = UniqueVertices[VertIdx];
        PositionStream.Set(VertIdx, Vertex.OriginalPosition);
        FRealtimeMeshTangentsHighPrecision Tangent;
        Tangent.SetNormal(FVector3f(Vertex.Normal));
        TangentStream.Set(VertIdx, Tangent);
        ColorStream.Set(VertIdx, Vertex.Color);
        TexCoordStream.Set(VertIdx, Vertex.UV);
        });
    ParallelFor(Triangles.Num(), [&](int32 TriIdx) {
        TriangleStream.Set(TriIdx, Triangles[TriIdx]);
        PolygroupStream.Set(TriIdx, 0);
        });
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
    if (!MeshChunksInitialized) return;;

    // Start timing
    double StartTime = FPlatformTime::Seconds();

    {
        // Array to track which chunks were modified
        TArray<int32> ChunksModified;
        ChunksModified.SetNumZeroed(Chunks.Num());

        ParallelFor(Chunks.Num(), [&](int32 idx)
        {
            TArray<FNodeEdge> tChunkEdges;
            bool tChanged = false;
            Chunks[idx]->UpdateLod(CameraPosition, LodFactor, tChunkEdges, tChanged);
            if (tChanged) {
                MeshChunks[idx]->ChunkEdges = tChunkEdges;
                // Mark this chunk as modified (using 1 instead of true)
                FPlatformAtomics::InterlockedExchange(&ChunksModified[idx], 1);
            }
        });

        // Only update the chunks that were modified
        ParallelFor(MeshChunks.Num(), [&](int32 i)
        {
            if (ChunksModified[i] != 0) {
                UpdateMeshChunkStreamData(MeshChunks[i]);
            }
        });
    }
}

void FAdaptiveOctree::UpdateMesh()
{
    if (!MeshChunksInitialized) return;
    {
        for (auto mChunk : MeshChunks) {
            mChunk->UpdateComponent();
        }
    }
}

// Retrieves all leaf nodes
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetLeaves()
{
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
    if (!Root.IsValid()) return TArray<TSharedPtr<FAdaptiveOctreeNode>>();
    return Root->GetSurfaceNodes();
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetChunks()
{
    return Chunks;
}

FVector FAdaptiveOctree::CalculateSurfaceNormal(const FVector& Position)
{
    // Scale step size relative to the root extent for numerical stability
    const double h = RootExtent * 1e-6;

    double dx = (DensityFunction(Position + FVector(h, 0, 0)) -
        DensityFunction(Position - FVector(h, 0, 0))) / (2.0 * h);
    double dy = (DensityFunction(Position + FVector(0, h, 0)) -
        DensityFunction(Position - FVector(0, h, 0))) / (2.0 * h);
    double dz = (DensityFunction(Position + FVector(0, 0, h)) -
        DensityFunction(Position - FVector(0, 0, h))) / (2.0 * h);

    FVector Gradient(dx, dy, dz);
    if (DensityFunction(Position) > 0)
    {
        Gradient = -Gradient;
    }
    return Gradient.GetSafeNormal();
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    static const FVector AxisVectors[3] = {
        FVector(1, 0, 0),
        FVector(0, 1, 0),
        FVector(0, 0, 1)
    };

    FVector Perp1 = AxisVectors[(Edge.Axis + 1) % 3];
    FVector Perp2 = AxisVectors[(Edge.Axis + 2) % 3];
    FVector ZCP = Edge.ZeroCrossingPoint;

    // Try multiple offset scales. Start with a reasonable default,
    // then try smaller and larger if we don't get enough unique nodes.
    // The offset needs to be:
    //   - Large enough to cross into all 4 neighboring cells
    //   - Small enough to not overshoot into non-adjacent cells
    static const double OffsetScales[] = { 0.1, 0.25, 0.01, 0.4 };

    for (double Scale : OffsetScales)
    {
        double Offset = Edge.Size * Scale;

        TSharedPtr<FAdaptiveOctreeNode> NodeA = GetLeafNodeByPoint(ZCP + (Perp1 + Perp2) * Offset);
        TSharedPtr<FAdaptiveOctreeNode> NodeB = GetLeafNodeByPoint(ZCP + (-Perp1 + Perp2) * Offset);
        TSharedPtr<FAdaptiveOctreeNode> NodeC = GetLeafNodeByPoint(ZCP + (-Perp1 - Perp2) * Offset);
        TSharedPtr<FAdaptiveOctreeNode> NodeD = GetLeafNodeByPoint(ZCP + (Perp1 - Perp2) * Offset);

        TArray<TSharedPtr<FAdaptiveOctreeNode>> Nodes;
        if (NodeA.IsValid()) Nodes.AddUnique(NodeA);
        if (NodeB.IsValid()) Nodes.AddUnique(NodeB);
        if (NodeC.IsValid()) Nodes.AddUnique(NodeC);
        if (NodeD.IsValid()) Nodes.AddUnique(NodeD);

        if (Nodes.Num() >= 3)
        {
            return Nodes;
        }
    }

    // Last resort: sample at the midpoint of the edge instead of the
    // zero-crossing point. The ZCP might be right on a cell boundary,
    // but the edge midpoint is guaranteed to be at the center of a cell face.
    FVector EdgeMidpoint = (Edge.Corners[0].Position + Edge.Corners[1].Position) * 0.5;
    double Offset = Edge.Size * 0.1;

    TSharedPtr<FAdaptiveOctreeNode> NodeA = GetLeafNodeByPoint(EdgeMidpoint + (Perp1 + Perp2) * Offset);
    TSharedPtr<FAdaptiveOctreeNode> NodeB = GetLeafNodeByPoint(EdgeMidpoint + (-Perp1 + Perp2) * Offset);
    TSharedPtr<FAdaptiveOctreeNode> NodeC = GetLeafNodeByPoint(EdgeMidpoint + (-Perp1 - Perp2) * Offset);
    TSharedPtr<FAdaptiveOctreeNode> NodeD = GetLeafNodeByPoint(EdgeMidpoint + (Perp1 - Perp2) * Offset);

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Nodes;
    if (NodeA.IsValid()) Nodes.AddUnique(NodeA);
    if (NodeB.IsValid()) Nodes.AddUnique(NodeB);
    if (NodeC.IsValid()) Nodes.AddUnique(NodeC);
    if (NodeD.IsValid()) Nodes.AddUnique(NodeD);

    return Nodes;
}

TSharedPtr<FAdaptiveOctreeNode> FAdaptiveOctree::GetLeafNodeByPoint(FVector Position)
{
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

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetNodesAtDepth(int Depth)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Result;
    TQueue<TSharedPtr<FAdaptiveOctreeNode>> Queue;
    Queue.Enqueue(Root);

    while (!Queue.IsEmpty())
    {
        TSharedPtr<FAdaptiveOctreeNode> Node;
        Queue.Dequeue(Node);
        if (!Node.IsValid()) continue;

        if (Node->TreeIndex.Num() == Depth)
        {
            if (Node->IsSurfaceNode)
            {
                Result.Add(Node);
            }
        }
        else if (Node->TreeIndex.Num() < Depth && !Node->IsLeaf())
        {
            for (int i = 0; i < 8; i++)
            {
                if (Node->Children[i].IsValid())
                    Queue.Enqueue(Node->Children[i]);
            }
        }
    }

    return Result;
}
// Clears the entire octree
void FAdaptiveOctree::Clear()
{
    Root.Reset();
}
