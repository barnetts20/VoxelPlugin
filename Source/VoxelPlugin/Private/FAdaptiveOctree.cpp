#include "FAdaptiveOctree.h"

// Constructor
FAdaptiveOctree::FAdaptiveOctree(TFunction<double(FVector, FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    RootExtent = InRootExtent;
    ChunkDepth = InChunkDepth;
    EditStore = MakeShared<FSparseEditStore>(InCenter, InRootExtent, InMaxDepth, InChunkDepth);

    Root = MakeShared<FAdaptiveOctreeNode>(&DensityFunction, EditStore, InCenter, InRootExtent, InChunkDepth, InMinDepth, InMaxDepth);
    SplitToDepth(Root, InChunkDepth);

    ChunkExtent = Root->Extent / (2 * InChunkDepth);

    // Build initial chunks list with neighbors
    Chunks = GetSurfaceNodes();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> NeighborChunks;
    for (auto& ChunkNode : Chunks)
    {
        if (!ChunkNode.IsValid()) continue;
        const double Offset = ChunkNode->Extent * 2.0;
        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + Directions[i] * Offset;
            TSharedPtr<FAdaptiveOctreeNode> NeighborNode = GetLeafNodeByPoint(NeighborPos);
            if (NeighborNode.IsValid() && !NeighborNode->IsSurfaceNode)
                NeighborChunks.AddUnique(NeighborNode);
        }
    }
    Chunks.Append(NeighborChunks);
}

void FAdaptiveOctree::TestEdit(FVector Center)
{
    double OctantExtent = RootExtent * 0.5;
    FVector OctantCenter = Root->Center + FVector(OctantExtent, OctantExtent, OctantExtent);
    float BrushStrength = -1;
    //int InsertDepth = EditStore->GetDepthForBrushRadius(OctantExtent, 4);
    ApplyEdit(OctantCenter, OctantExtent, OctantExtent * BrushStrength);
}

void FAdaptiveOctree::ApplyEdit(FVector BrushCenter, double BrushRadius, double Strength)
{
    int Depth = EditStore->GetDepthForBrushRadius(BrushRadius, 4);
    EditStore->ApplySphericalEdit(BrushCenter, BrushRadius, Strength, Depth);

    double ReconstructRadius = BrushRadius * 1.5;
    ReconstructAffectedLeaves(Root, BrushCenter, ReconstructRadius);
    UpdateAffectedChunks(Root, BrushCenter, ReconstructRadius);
}

void FAdaptiveOctree::UpdateAffectedChunks(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius)
{
    if (!Node.IsValid()) return;

    FVector Closest;
    Closest.X = FMath::Clamp(EditCenter.X, Node->Center.X - Node->Extent, Node->Center.X + Node->Extent);
    Closest.Y = FMath::Clamp(EditCenter.Y, Node->Center.Y - Node->Extent, Node->Center.Y + Node->Extent);
    Closest.Z = FMath::Clamp(EditCenter.Z, Node->Center.Z - Node->Extent, Node->Center.Z + Node->Extent);

    if (FVector::Dist(Closest, EditCenter) > SearchRadius)
        return;

    if (Node->TreeIndex.Num() == ChunkDepth)
    {
        bool InMap = MeshChunks.Contains(Node);
        bool NeedsMesh = Node->IsSurfaceNode || HasSurfaceNeighbor(Node);

        if (!NeedsMesh && InMap)
        {
            MeshChunks[Node]->ChunkMeshData->ResetStreams();
            MeshChunks[Node]->IsDirty = true;
            MeshChunks.Remove(Node);
        }
        else if (InMap)
        {
            TArray<FNodeEdge> NewEdges;
            GatherLeafEdges(Node, NewEdges);
            MeshChunks[Node]->ChunkEdges = NewEdges;
            UpdateMeshChunkStreamData(MeshChunks[Node]);
        }
        else if (NeedsMesh)
        {
            PendingNewChunks.AddUnique(Node);
        }
        return;
    }

    if (Node->TreeIndex.Num() < ChunkDepth)
    {
        for (int i = 0; i < 8; i++)
            UpdateAffectedChunks(Node->Children[i], EditCenter, SearchRadius);
    }
}

void FAdaptiveOctree::ReconstructAffectedLeaves(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius)
{
    if (!Node.IsValid()) return;

    FVector Closest;
    Closest.X = FMath::Clamp(EditCenter.X, Node->Center.X - Node->Extent, Node->Center.X + Node->Extent);
    Closest.Y = FMath::Clamp(EditCenter.Y, Node->Center.Y - Node->Extent, Node->Center.Y + Node->Extent);
    Closest.Z = FMath::Clamp(EditCenter.Z, Node->Center.Z - Node->Extent, Node->Center.Z + Node->Extent);

    if (FVector::Dist(Closest, EditCenter) > SearchRadius)
        return;

    if (Node->IsLeaf() || Node->TreeIndex.Num() >= ChunkDepth) {
        Node->ComputeCornerDensity();
        Node->ComputeDualContourPosition();

        if (Node->IsLeaf() && Node->IsSurfaceNode)
        {
            TSharedPtr<FAdaptiveOctreeNode> Ancestor = Node->Parent.Pin();
            while (Ancestor.IsValid() && !Ancestor->IsSurfaceNode)
            {
                Ancestor->IsSurfaceNode = true;
                Ancestor = Ancestor->Parent.Pin();
            }
        }
    }
    if (!Node->IsLeaf())
    {
        for (int i = 0; i < 8; i++)
            ReconstructAffectedLeaves(Node->Children[i], EditCenter, SearchRadius);
    }
}

//void FAdaptiveOctree::InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial)
//{
//    CachedParentActor = InParentActor;
//    CachedMaterial = InMaterial;
//    MeshChunks.Empty();
//    for (auto& Chunk : Chunks)
//    {
//        TSharedPtr<FMeshChunk> NewMeshChunk = MakeShared<FMeshChunk>();
//        NewMeshChunk->ChunkMeshData = MakeShared<FMeshStreamData>();
//        NewMeshChunk->ChunkExtent = Chunk->Extent;
//        NewMeshChunk->ChunkCenter = Chunk->Center;
//        MeshChunks.Add(Chunk, NewMeshChunk);
//    }
//    MeshChunksInitialized = true;
//}

void FAdaptiveOctree::CreatePendingMeshChunks()
{
    for (auto& Chunk : PendingNewChunks)
    {
        TSharedPtr<FMeshChunk> NewMeshChunk = MakeShared<FMeshChunk>();
        NewMeshChunk->ChunkMeshData = MakeShared<FMeshStreamData>();
        NewMeshChunk->ChunkExtent = Chunk->Extent;
        NewMeshChunk->ChunkCenter = Chunk->Center;
        TArray<FNodeEdge> NewEdges;
        GatherLeafEdges(Chunk, NewEdges);
        NewMeshChunk->ChunkEdges = NewEdges;
        UpdateMeshChunkStreamData(NewMeshChunk);

        MeshChunks.Add(Chunk, NewMeshChunk);
    }
    PendingNewChunks.Empty();
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
}

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

    double QuantizationGrid = InChunk->ChunkExtent * 1e-6;

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

    InChunk->IsDirty = true;// (Triangles.Num() > 0);
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    for (auto& Pair : MeshChunks)
    {
        TArray<FNodeEdge> tChunkEdges;
        TMap<FEdgeKey, int32> tEdgeMap;
        bool tChanged = false;
        Pair.Key->UpdateLod(CameraPosition, InScreenSpaceThreshold, InCameraFOV, tChunkEdges, tEdgeMap, tChanged);
        if (tChanged)
        {
            Pair.Value->ChunkEdges = tChunkEdges;
            UpdateMeshChunkStreamData(Pair.Value);
        }
    }
}

void FAdaptiveOctree::UpdateMesh()
{
    if (!MeshChunksInitialized) return;
    for (auto& Pair : MeshChunks)
    {
        if (Pair.Value->IsDirty){
            Pair.Value->UpdateComponent(Pair.Value, Pair.Key, CachedParentActor, CachedMaterial);
        }
    }
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetSurfaceNodes()
{
    if (!Root.IsValid()) return TArray<TSharedPtr<FAdaptiveOctreeNode>>();
    return Root->GetSurfaceNodes();
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Nodes;

    FVector Origin = Edge.Corners[0].Position;
    FVector End = Edge.Corners[1].Position;
    FVector Midpoint = (Origin + End) * 0.5;

    auto GetLeafWithBias = [&](bool BiasPerp1, bool BiasPerp2) -> TSharedPtr<FAdaptiveOctreeNode> {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;

        while (CurrentNode.IsValid() && !CurrentNode->IsLeaf()) {
            int ChildIndex = 0;

            double Epsilon = ChunkExtent * 1e-6;

            auto CheckSide = [&](int AxisIndex, bool PositiveBias) {
                double Diff = Midpoint[AxisIndex] - CurrentNode->Center[AxisIndex];
                if (FMath::Abs(Diff) <= Epsilon) {
                    return PositiveBias;
                }
                return Diff > 0.0;
                };

            if (Edge.Axis == 0) {
                if (Midpoint.X >= CurrentNode->Center.X) ChildIndex |= 1;
                if (CheckSide(1, BiasPerp1)) ChildIndex |= 2;
                if (CheckSide(2, BiasPerp2)) ChildIndex |= 4;
            }
            else if (Edge.Axis == 1) {
                if (CheckSide(0, BiasPerp2)) ChildIndex |= 1;
                if (Midpoint.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
                if (CheckSide(2, BiasPerp1)) ChildIndex |= 4;
            }
            else if (Edge.Axis == 2) {
                if (CheckSide(0, BiasPerp1)) ChildIndex |= 1;
                if (CheckSide(1, BiasPerp2)) ChildIndex |= 2;
                if (Midpoint.Z >= CurrentNode->Center.Z) ChildIndex |= 4;
            }

            CurrentNode = CurrentNode->Children[ChildIndex];
        }
        return CurrentNode;
        };

    TSharedPtr<FAdaptiveOctreeNode> N0 = GetLeafWithBias(true, true);
    TSharedPtr<FAdaptiveOctreeNode> N1 = GetLeafWithBias(false, true);
    TSharedPtr<FAdaptiveOctreeNode> N2 = GetLeafWithBias(false, false);
    TSharedPtr<FAdaptiveOctreeNode> N3 = GetLeafWithBias(true, false);

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

TSharedPtr<FAdaptiveOctreeNode> FAdaptiveOctree::GetNodeByPointAtDepth(FVector Position, int TargetDepth)
{
    TSharedPtr<FAdaptiveOctreeNode> Current = Root;
    while (Current.IsValid() && Current->TreeIndex.Num() < TargetDepth)
    {
        int ChildIndex = 0;
        if (Position.X >= Current->Center.X) ChildIndex |= 1;
        if (Position.Y >= Current->Center.Y) ChildIndex |= 2;
        if (Position.Z >= Current->Center.Z) ChildIndex |= 4;

        if (!Current->Children[ChildIndex].IsValid())
            return nullptr;

        Current = Current->Children[ChildIndex];
    }
    return Current;
}

bool FAdaptiveOctree::HasSurfaceNeighbor(TSharedPtr<FAdaptiveOctreeNode> Node)
{
    const double Offset = Node->Extent * 2.0;
    for (int i = 0; i < 6; i++)
    {
        FVector NeighborPos = Node->Center + Directions[i] * Offset;
        TSharedPtr<FAdaptiveOctreeNode> Neighbor = GetNodeByPointAtDepth(NeighborPos, ChunkDepth);
        if (Neighbor.IsValid() && Neighbor->IsSurfaceNode)
            return true;
    }
    return false;
}

void FAdaptiveOctree::GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FNodeEdge>& OutEdges)
{
    if (!Node.IsValid()) return;
    if (Node->IsLeaf())
    {
        OutEdges.Append(Node->GetSignChangeEdges());
        return;
    }
    for (int i = 0; i < 8; i++)
        GatherLeafEdges(Node->Children[i], OutEdges);
}

void FAdaptiveOctree::Clear()
{
    // Iterate through MeshChunks and break the UObject link
    for (auto& Chunk : MeshChunks)
    {
        if (Chunk.Value.IsValid())
        {
            // Null out the reference to the RealtimeMeshComponent
            Chunk.Value->ChunkRtComponent = nullptr;
            Chunk.Value->ChunkRtMesh = nullptr;
        }
    }
    MeshChunks.Empty();
    Root.Reset();
}