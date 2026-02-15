#include "FAdaptiveOctree.h"

// Constructor — lightweight, no splitting
FAdaptiveOctree::FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    RootExtent = InRootExtent;
    ChunkDepth = InChunkDepth;
    Root = MakeShared<FAdaptiveOctreeNode>(InDensityFunction, InCenter, InRootExtent, InMinDepth, InMaxDepth);
}

void FAdaptiveOctree::InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial)
{
    ParentActor = InParentActor;
    Material = InMaterial;
    MeshChunksInitialized = true;
}

bool FAdaptiveOctree::HasReachedChunkDepth()
{
    TSharedPtr<FAdaptiveOctreeNode> Node = Root;
    for (int d = 0; d < ChunkDepth; d++)
    {
        if (!Node.IsValid() || Node->IsLeaf()) return false;
        Node = Node->Children[0];
    }
    return true;
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double LodFactor)
{
    if (!MeshChunksInitialized) return;

    // Phase 1: Split/merge
    TArray<FNodeEdge> UnusedEdges;
    bool TreeChanged = false;
    Root->UpdateLod(CameraPosition, LodFactor, UnusedEdges, TreeChanged);

    // Phase 2: Wait for chunk depth
    if (!HasReachedChunkDepth()) return;

    // Phase 3: Discover new surface chunks
    TArray<TSharedPtr<FAdaptiveOctreeNode>> NewChunkNodes;
    TArray<TSharedPtr<FAdaptiveOctreeNode>> CurrentChunkNodes = GetNodesAtDepth(ChunkDepth);
    for (auto& ChunkNode : CurrentChunkNodes)
    {
        if (ChunkNodeToIndex.Contains(ChunkNode.Get())) continue;
        NewChunkNodes.Add(ChunkNode);
    }

    if (NewChunkNodes.Num() > 0)
    {
        AsyncTask(ENamedThreads::GameThread, [this, NewChunkNodes]()
            {
                for (auto& ChunkNode : NewChunkNodes)
                {
                    if (ChunkNodeToIndex.Contains(ChunkNode.Get())) continue;

                    TSharedPtr<FMeshChunk> NewChunk = MakeShared<FMeshChunk>();
                    NewChunk->Initialize(ParentActor, Material, ChunkNode->Center, ChunkNode->Extent);

                    int32 Idx = Chunks.Num();
                    Chunks.Add(ChunkNode);
                    MeshChunks.Add(NewChunk);
                    ChunkNodeToIndex.Add(ChunkNode.Get(), Idx);
                }
            });
        return; // Wait for game thread to create components
    }

    // Phase 4: Re-mesh if tree changed
    if (TreeChanged)
    {
        ParallelFor(MeshChunks.Num(), [&](int32 i)
            {
                MeshChunks[i]->ChunkEdges = Chunks[i]->GetSurfaceEdges();
                UpdateMeshChunkStreamData(MeshChunks[i]);
            });
    }
}

void FAdaptiveOctree::UpdateMesh()
{
    if (!MeshChunksInitialized) return;
    for (auto& mChunk : MeshChunks)
    {
        mChunk->UpdateComponent();
    }
}

// ============================================================
// Mesh generation (existing edge-based pipeline)
// ============================================================

FVector2f ComputeTriplanarUV(FVector Position, FVector Normal)
{
    FVector AbsNormal = Normal.GetAbs();
    if (AbsNormal.X > AbsNormal.Y && AbsNormal.X > AbsNormal.Z)
        return FVector2f(Position.Y, Position.Z) * 0.0001f;
    else if (AbsNormal.Y > AbsNormal.X && AbsNormal.Y > AbsNormal.Z)
        return FVector2f(Position.X, Position.Z) * 0.0001f;
    else
        return FVector2f(Position.X, Position.Y) * 0.0001f;
}

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
    FVector Position;
    FVector OriginalPosition;
    FVector Normal;
    FColor Color;
    FVector2f UV;

    bool operator==(const FMeshVertex& Other) const
    {
        return Position == Other.Position;
    }
};

uint32 GetTypeHash(const FMeshVertex& Vertex)
{
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

    PositionStream.Empty();
    TangentStream.Empty();
    ColorStream.Empty();
    TexCoordStream.Empty();
    TriangleStream.Empty();
    PolygroupStream.Empty();

    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> Triangles;

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

// ============================================================
// Edge sampling (existing implementation)
// ============================================================

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    static const FVector AxisVectors[3] = {
        FVector(1, 0, 0), FVector(0, 1, 0), FVector(0, 0, 1)
    };

    FVector Perp1 = AxisVectors[(Edge.Axis + 1) % 3];
    FVector Perp2 = AxisVectors[(Edge.Axis + 2) % 3];
    FVector ZCP = Edge.ZeroCrossingPoint;

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

        if (Nodes.Num() >= 3) return Nodes;
    }

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

// ============================================================
// Tree queries
// ============================================================

TSharedPtr<FAdaptiveOctreeNode> FAdaptiveOctree::GetLeafNodeByPoint(FVector Position)
{
    TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;
    while (CurrentNode.IsValid())
    {
        if (CurrentNode->IsLeaf()) return CurrentNode;

        int ChildIndex = 0;
        if (Position.X >= CurrentNode->Center.X) ChildIndex |= 1;
        if (Position.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
        if (Position.Z >= CurrentNode->Center.Z) ChildIndex |= 4;

        if (CurrentNode->Children[ChildIndex].IsValid())
            CurrentNode = CurrentNode->Children[ChildIndex];
        else
            break;
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
                Result.Add(Node);
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

void FAdaptiveOctree::Clear()
{
    Root.Reset();
}