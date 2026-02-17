#include "FAdaptiveOctree.h"

// Constructor
FAdaptiveOctree::FAdaptiveOctree(TFunction<double(FVector, FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    RootExtent = InRootExtent;
    ChunkDepth = InChunkDepth;
    Root = MakeShared<FAdaptiveOctreeNode>(&DensityFunction, InCenter, InRootExtent, InMinDepth, InMaxDepth);
    {
        SplitToDepth(Root, ChunkDepth);
    }
    Chunks = GetSurfaceNodes(); //Get the nodes containing surface
    TArray<TSharedPtr<FAdaptiveOctreeNode>> NeighborChunks;

    for (auto& ChunkNode : Chunks)
    {
        if (!ChunkNode.IsValid())
            continue;

        const double Offset = ChunkNode->Extent * 2.0;

        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + Directions[i] * Offset;

            TSharedPtr<FAdaptiveOctreeNode> NeighborNode = GetLeafNodeByPoint(NeighborPos);

            if (!NeighborNode.IsValid())
                continue;

            if (!NeighborNode->IsSurfaceNode)
            {
                NeighborChunks.AddUnique(NeighborNode);
            }
        }
    }

    Chunks.Append(NeighborChunks);
}

void FAdaptiveOctree::InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial) {
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

FVector2f FAdaptiveOctree::ComputeTriplanarUV(FVector Position, FVector Normal)
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

FVector FAdaptiveOctree::QuantizePosition(const FVector& P, double GridSize)
{
    return FVector(
        FMath::RoundToDouble(P.X / GridSize) * GridSize,
        FMath::RoundToDouble(P.Y / GridSize) * GridSize,
        FMath::RoundToDouble(P.Z / GridSize) * GridSize
    );
}

uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    // Hash the quantized position — guaranteed consistent with operator==
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}

void FAdaptiveOctree::UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk)
{
    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> Triangles;

    struct FEdgeVertexData {
        TArray<FMeshVertex> Vertices;
        TOptional<FNodeEdge> Edge;
        bool IsValid;
    };

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(InChunk->ChunkEdges.Num());

    double QuantizationGrid = InChunk->ChunkExtent * 2 * 1e-6;

    ParallelFor(InChunk->ChunkEdges.Num(), [&](int32 edgeIdx) {
        const FNodeEdge currentEdge = InChunk->ChunkEdges[edgeIdx];
        TArray<TSharedPtr<FAdaptiveOctreeNode>> nodesToMesh = SampleNodesAroundEdge(currentEdge);

        if (!InChunk->ShouldProcessEdge(currentEdge, nodesToMesh)) {
            AllEdgeData[edgeIdx].IsValid = false;
            return;
        }

        AllEdgeData[edgeIdx].IsValid = true;
        AllEdgeData[edgeIdx].Edge = currentEdge;
        AllEdgeData[edgeIdx].Vertices.SetNumZeroed(nodesToMesh.Num());

        for (int i = 0; i < nodesToMesh.Num(); i++) {
            FVector WorldPos = nodesToMesh[i]->DualContourPosition;

            double LX = (double)WorldPos.X - (double)InChunk->ChunkCenter.X;
            double LY = (double)WorldPos.Y - (double)InChunk->ChunkCenter.Y;
            double LZ = (double)WorldPos.Z - (double)InChunk->ChunkCenter.Z;
            FVector LocalPos(LX, LY, LZ);

            FVector n = nodesToMesh[i]->DualContourNormal;

            // Store quantized position for the normal calculation loop
            AllEdgeData[edgeIdx].Vertices[i].Position = QuantizePosition(LocalPos, QuantizationGrid);
            AllEdgeData[edgeIdx].Vertices[i].OriginalPosition = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = n; 
            AllEdgeData[edgeIdx].Vertices[i].Color = FColor::Green;
            AllEdgeData[edgeIdx].Vertices[i].UV = ComputeTriplanarUV(WorldPos, n);
        }
    });

    // Topology Build
    for (int32 edgeIdx = 0; edgeIdx < AllEdgeData.Num(); edgeIdx++) {
        if (!AllEdgeData[edgeIdx].IsValid) continue;

        const auto& currentEdge = AllEdgeData[edgeIdx].Edge.GetValue();
        const TArray<FMeshVertex>& EdgeVertices = AllEdgeData[edgeIdx].Vertices;

        TArray<int32> VertexIndices;
        VertexIndices.SetNumZeroed(EdgeVertices.Num());

        for (int i = 0; i < EdgeVertices.Num(); i++) {
            int32* ExistingIndex = VertexMap.Find(EdgeVertices[i]);
            if (ExistingIndex) {
                VertexIndices[i] = *ExistingIndex;
            } else {
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
            } else {
                Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[1], VertexIndices[2]));
            }

            if (EdgeVertices.Num() == 4
                && VertexIndices[0] != VertexIndices[3]
                && VertexIndices[2] != VertexIndices[3])
            {
                if (FlipWinding) {
                    Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[3], VertexIndices[2]));
                } else {
                    Triangles.Add(FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[3]));
                }
            }
        }
    }


    InChunk->ChunkMeshData->ResetStreams();

    auto PositionStream = InChunk->ChunkMeshData->GetPositionStream();
    auto TangentStream = InChunk->ChunkMeshData->GetTangentStream();
    auto ColorStream = InChunk->ChunkMeshData->GetColorStream();
    auto TexCoordStream = InChunk->ChunkMeshData->GetTexCoordStream();
    auto TriangleStream = InChunk->ChunkMeshData->GetTriangleStream();
    auto PolygroupStream = InChunk->ChunkMeshData->GetPolygroupStream();

    PositionStream.SetNumUninitialized(UniqueVertices.Num());
    TangentStream.SetNumUninitialized(UniqueVertices.Num());
    ColorStream.SetNumUninitialized(UniqueVertices.Num());
    TexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    TriangleStream.SetNumUninitialized(Triangles.Num());
    PolygroupStream.SetNumUninitialized(Triangles.Num());

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
        FMeshVertex& Vertex = UniqueVertices[VertIdx];
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

    InChunk->IsDirty = (Triangles.Num() > 0);
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double LodFactor)
{
    if (!MeshChunksInitialized) return;

    TArray<int32> ChunksModified;
    ChunksModified.SetNumZeroed(Chunks.Num());

    ParallelFor(Chunks.Num(), [&](int32 idx)
        {
            TArray<FNodeEdge> tChunkEdges;
            TMap<FEdgeKey, int32> tEdgeMap;
            bool tChanged = false;
            Chunks[idx]->UpdateLod(CameraPosition, LodFactor, tChunkEdges, tEdgeMap, tChanged);
            if (tChanged) {
                MeshChunks[idx]->ChunkEdges = tChunkEdges;
                FPlatformAtomics::InterlockedExchange(&ChunksModified[idx], 1);
            }
        });

    ParallelFor(MeshChunks.Num(), [&](int32 i)
        {
            if (ChunksModified[i] != 0) {
                UpdateMeshChunkStreamData(MeshChunks[i]);
            }
        });
}

void FAdaptiveOctree::UpdateMesh()
{
    if (!MeshChunksInitialized) return;
    {
        for (auto mChunk : MeshChunks) {
            if(mChunk->IsDirty) mChunk->UpdateComponent(mChunk);
        }
    }
}

// Retrieves all surface nodes for meshing
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetSurfaceNodes()
{
    if (!Root.IsValid()) return TArray<TSharedPtr<FAdaptiveOctreeNode>>();
    return Root->GetSurfaceNodes();
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Nodes;

    // Calculate the exact mathematical midpoint of the edge
    FVector Origin = Edge.Corners[0].Position;
    FVector End = Edge.Corners[1].Position;
    FVector Midpoint = (Origin + End) * 0.5;
    double ChunkExtent = Root->Extent / (2 * ChunkDepth);
    // Helper lambda for biased top-down traversal
    auto GetLeafWithBias = [&](bool BiasPerp1, bool BiasPerp2) -> TSharedPtr<FAdaptiveOctreeNode> {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;

        while (CurrentNode.IsValid() && !CurrentNode->IsLeaf()) {
            int ChildIndex = 0;

            // Adaptive epsilon based on current node size to handle floating-point drift
            double Epsilon = ChunkExtent * 2 * 1e-6;

            // Helper to check which side of the splitting plane the point falls on
            auto CheckSide = [&](int AxisIndex, bool PositiveBias) {
                double Diff = Midpoint[AxisIndex] - CurrentNode->Center[AxisIndex];

                // If the midpoint is exactly on the boundary, force it into the biased quadrant
                if (FMath::Abs(Diff) <= Epsilon) {
                    return PositiveBias;
                }
                // Otherwise, follow the actual spatial coordinate
                return Diff > 0.0;
                };

            // Route the traversal based on the axis the edge runs along
            if (Edge.Axis == 0) { // X-axis edge
                if (Midpoint.X >= CurrentNode->Center.X) ChildIndex |= 1;
                if (CheckSide(1, BiasPerp1)) ChildIndex |= 2; // Y is Perp1
                if (CheckSide(2, BiasPerp2)) ChildIndex |= 4; // Z is Perp2
            }
            else if (Edge.Axis == 1) { // Y-axis edge
                if (CheckSide(0, BiasPerp2)) ChildIndex |= 1; // X is Perp2
                if (Midpoint.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
                if (CheckSide(2, BiasPerp1)) ChildIndex |= 4; // Z is Perp1
            }
            else if (Edge.Axis == 2) { // Z-axis edge
                if (CheckSide(0, BiasPerp1)) ChildIndex |= 1; // X is Perp1
                if (CheckSide(1, BiasPerp2)) ChildIndex |= 2; // Y is Perp2
                if (Midpoint.Z >= CurrentNode->Center.Z) ChildIndex |= 4;
            }

            CurrentNode = CurrentNode->Children[ChildIndex];
        }
        return CurrentNode;
        };

    // Topologically find the 4 neighbors using your exact N0->N3 ordering
    // N0: +Perp1, +Perp2
    TSharedPtr<FAdaptiveOctreeNode> N0 = GetLeafWithBias(true, true);
    // N1: -Perp1, +Perp2
    TSharedPtr<FAdaptiveOctreeNode> N1 = GetLeafWithBias(false, true);
    // N2: -Perp1, -Perp2
    TSharedPtr<FAdaptiveOctreeNode> N2 = GetLeafWithBias(false, false);
    // N3: +Perp1, -Perp2
    TSharedPtr<FAdaptiveOctreeNode> N3 = GetLeafWithBias(true, false);

    // AddUnique ensures that if a neighbor is a lower LOD (larger cell), 
    // it is only added once, while strictly preserving the rotational winding order.
    if (N0.IsValid()) Nodes.AddUnique(N0);
    if (N1.IsValid()) Nodes.AddUnique(N1);
    if (N2.IsValid()) Nodes.AddUnique(N2);
    if (N3.IsValid()) Nodes.AddUnique(N3);

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

// Clears the entire octree
void FAdaptiveOctree::Clear()
{
    Root.Reset();
}
