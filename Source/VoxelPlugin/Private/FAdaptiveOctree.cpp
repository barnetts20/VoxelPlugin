#include "FAdaptiveOctree.h"

FAdaptiveOctree::FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial, TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    EditStore = InEditStore;
    RootExtent = InRootExtent;
    ChunkExtent = InRootExtent / FMath::Pow(2.0, (double)InChunkDepth);
    CachedParentActor = InParentActor;
    CachedMaterial = InMaterial;
    ChunkDepth = InChunkDepth;

    Root = MakeShared<FAdaptiveOctreeNode>( InCenter, InRootExtent, InChunkDepth, InMinDepth, InMaxDepth);
    ComputeNodeData(Root);
    SplitToDepth(Root, InChunkDepth);

    PopulateChunks();
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
                ComputeNodeData(Node->Children[i]);
                SplitToDepth(Node->Children[i], InMinDepth);
            }
        }
    }
}

void FAdaptiveOctree::PopulateChunks()
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks = Root->GetSurfaceChunks();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> NeighborChunks;
    for (auto& ChunkNode : Chunks)
    {
        if (!ChunkNode.IsValid()) continue;

        const double Offset = ChunkNode->Extent * 2.0;
        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + Directions[i] * Offset;
            TSharedPtr<FAdaptiveOctreeNode> NeighborNode = GetLeafNodeByPoint(NeighborPos);
            if (!NeighborNode.IsValid()) continue;
            if (!NeighborNode->IsSurfaceNode)
                NeighborChunks.AddUnique(NeighborNode);
        }
    }
    Chunks.Append(NeighborChunks);

    // Process all chunk data in bulk -- parallel
    ParallelFor(Chunks.Num(), [&](int32 i) {
        ReconstructSubtree(Chunks[i], FVector::ZeroVector, 0);
        });

    // Build mesh chunks -- parallel edge gather + stream build
    TArray<TSharedPtr<FMeshChunk>> NewChunks;
    NewChunks.SetNum(Chunks.Num());

    ParallelFor(Chunks.Num(), [&](int32 i) {
        NewChunks[i] = MakeShared<FMeshChunk>();
        NewChunks[i]->CachedParentActor = CachedParentActor;
        NewChunks[i]->CachedMaterial = CachedMaterial;
        NewChunks[i]->InitializeData(Chunks[i]->Center, Chunks[i]->Extent);

        TArray<FNodeEdge> Edges;
        GatherLeafEdges(Chunks[i], Edges);
        NewChunks[i]->ChunkEdges = Edges;
        UpdateMeshChunkStreamData(NewChunks[i]);
        NewChunks[i]->IsDirty = (NewChunks[i]->ChunkMeshData->GetPositionStream().Num() > 0);
    });

    // Serial -- add to map
    for (int32 i = 0; i < Chunks.Num(); i++)
        ChunkMap.Add(Chunks[i], NewChunks[i]);

    MeshChunksInitialized = true;
}

void FAdaptiveOctree::UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks)
{
    TSharedPtr<FMeshChunk>* Found = ChunkMap.Find(ChunkNode);

    if (Found && *Found)
    {
        if (!ChunkNode->IsSurfaceNode)
        {
            (*Found)->ChunkMeshData->ResetStreams();
            (*Found)->IsDirty = true;
        }
        else
        {
            OutDirtyChunks.Add({ ChunkNode, *Found });
        }
    }
    else if (ChunkNode->IsSurfaceNode)
    {
        TSharedPtr<FMeshChunk> NewChunk = MakeShared<FMeshChunk>();
        NewChunk->CachedParentActor = CachedParentActor;
        NewChunk->CachedMaterial = CachedMaterial;
        NewChunk->InitializeData(ChunkNode->Center, ChunkNode->Extent);
        ChunkMap.Add(ChunkNode, NewChunk);
        OutDirtyChunks.Add({ ChunkNode, NewChunk });

        // Buffer neighbors
        const double Offset = ChunkNode->Extent * 2.0;
        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + Directions[i] * Offset;
            TSharedPtr<FAdaptiveOctreeNode> Neighbor = GetLeafNodeByPoint(NeighborPos);

            if (!Neighbor.IsValid()) continue;
            if (Neighbor->IsSurfaceNode) continue;
            if (ChunkMap.Contains(Neighbor)) continue;

            TSharedPtr<FMeshChunk> NeighborChunk = MakeShared<FMeshChunk>();
            NeighborChunk->CachedParentActor = CachedParentActor;
            NeighborChunk->CachedMaterial = CachedMaterial;
            NeighborChunk->InitializeData(Neighbor->Center, Neighbor->Extent);
            ChunkMap.Add(Neighbor, NeighborChunk);
            OutDirtyChunks.Add({ Neighbor, NeighborChunk });
        }
    }
}

void FAdaptiveOctree::ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    int depth = EditStore->GetDepthForBrushRadius(InEditRadius, InEditResolution);
    TArray<FVector> AffectedChunkCenters = EditStore->ApplySphericalEdit(InEditCenter, InEditRadius, InEditStrength, depth);
    double ReconstructRadius = InEditRadius * 1.5;

    // Resolve chunk nodes
    TArray<TSharedPtr<FAdaptiveOctreeNode>> ChunkNodes;
    for (const FVector& Center : AffectedChunkCenters)
    {
        TSharedPtr<FAdaptiveOctreeNode> ChunkNode = GetChunkNodeByPoint(Center);
        if (ChunkNode.IsValid())
            ChunkNodes.Add(ChunkNode);
    }

    // Stage 1: Parallel — recompute all node data
    ParallelFor(ChunkNodes.Num(), [&](int32 i) {
        ReconstructSubtree(ChunkNodes[i], InEditCenter, ReconstructRadius);
    });

    // Stage 2: Serial — update chunk map
    TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>> DirtyChunks;
    for (auto& ChunkNode : ChunkNodes)
    {
        UpdateChunkMap(ChunkNode, DirtyChunks);
    }

    // Stage 3: Parallel — gather edges + rebuild mesh streams
    ParallelFor(DirtyChunks.Num(), [&](int32 i) {
        TArray<FNodeEdge> NewEdges;
        GatherLeafEdges(DirtyChunks[i].Key, NewEdges);
        DirtyChunks[i].Value->ChunkEdges = NewEdges;
        UpdateMeshChunkStreamData(DirtyChunks[i].Value);
        });
}

void FAdaptiveOctree::GatherUniqueCorners(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& CornerMap, double QuantizeGrid, FVector EditCenter, double SearchRadius)
{
    if (!Node.IsValid()) return;

    if (SearchRadius > 0)
    {
        FVector Closest;
        Closest.X = FMath::Clamp(EditCenter.X, Node->Center.X - Node->Extent, Node->Center.X + Node->Extent);
        Closest.Y = FMath::Clamp(EditCenter.Y, Node->Center.Y - Node->Extent, Node->Center.Y + Node->Extent);
        Closest.Z = FMath::Clamp(EditCenter.Z, Node->Center.Z - Node->Extent, Node->Center.Z + Node->Extent);

        if (FVector::DistSquared(Closest, EditCenter) > SearchRadius * SearchRadius)
            return;
    }

    // Collect corners at ALL depths
    for (int i = 0; i < 8; i++)
    {
        FVector Pos = Node->Corners[i].Position;
        FIntVector Key(
            FMath::RoundToInt(Pos.X / QuantizeGrid),
            FMath::RoundToInt(Pos.Y / QuantizeGrid),
            FMath::RoundToInt(Pos.Z / QuantizeGrid));

        int32* Existing = CornerMap.Find(Key);
        if (Existing)
        {
            Samples[*Existing].Targets.Add(&Node->Corners[i].Density);
        }
        else
        {
            int32 NewIdx = Samples.Num();
            FCornerSample Sample;
            Sample.Position = Pos;
            Sample.Targets.Add(&Node->Corners[i].Density);
            Samples.Add(Sample);
            CornerMap.Add(Key, NewIdx);
        }
    }

    if (Node->IsLeaf()) return;

    for (int i = 0; i < 8; i++)
        GatherUniqueCorners(Node->Children[i], Samples, CornerMap, QuantizeGrid, EditCenter, SearchRadius);
}

void FAdaptiveOctree::FinalizeSubtree(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius)
{
    if (!Node.IsValid()) return;

    if (SearchRadius > 0)
    {
        FVector Closest;
        Closest.X = FMath::Clamp(EditCenter.X, Node->Center.X - Node->Extent, Node->Center.X + Node->Extent);
        Closest.Y = FMath::Clamp(EditCenter.Y, Node->Center.Y - Node->Extent, Node->Center.Y + Node->Extent);
        Closest.Z = FMath::Clamp(EditCenter.Z, Node->Center.Z - Node->Extent, Node->Center.Z + Node->Extent);

        if (FVector::DistSquared(Closest, EditCenter) > SearchRadius * SearchRadius)
            return;
    }

    // Children first
    if (!Node->IsLeaf())
    {
        for (int i = 0; i < 8; i++)
            FinalizeSubtree(Node->Children[i], EditCenter, SearchRadius);
    }

    // Then this node
    Node->FinalizeFromExistingCorners();
}

void FAdaptiveOctree::ReconstructSubtree(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius)
{
    // 1. Gather unique corners from affected nodes
    TMap<FIntVector, int32> CornerMap;
    TArray<FCornerSample> Samples;
    Samples.Reserve(4096);
    CornerMap.Reserve(4096);
    double QuantizeGrid = ChunkExtent * 1e-6;

    GatherUniqueCorners(Node, Samples, CornerMap, QuantizeGrid, EditCenter, SearchRadius);

    if (Samples.Num() == 0) return;

    // 2. Build position arrays + pre-compute distances
    int32 Count = Samples.Num();
    TArray<float> XPos, YPos, ZPos, NoiseOut;
    XPos.SetNumUninitialized(Count);
    YPos.SetNumUninitialized(Count);
    ZPos.SetNumUninitialized(Count);
    NoiseOut.SetNumUninitialized(Count);

    double NoiseScale = RootExtent * 0.1;
    FVector PlanetCenter = Root->Center;

    for (int32 i = 0; i < Count; i++)
    {
        FVector PlanetRel = Samples[i].Position - PlanetCenter;
        Samples[i].Dist = PlanetRel.Size();
        FVector Dir = PlanetRel / Samples[i].Dist; // Cheaper than GetSafeNormal since we already have Size
        FVector SurfacePos = Dir * RootExtent;
        XPos[i] = (float)(SurfacePos.X / NoiseScale);
        YPos[i] = (float)(SurfacePos.Y / NoiseScale);
        ZPos[i] = (float)(SurfacePos.Z / NoiseScale);
    }

    // 3. One bulk noise call
    DensityFunction(Count, XPos.GetData(), YPos.GetData(), ZPos.GetData(), NoiseOut.GetData());

    // 4. SDF + edits -- uses pre-computed distances
    for (int32 i = 0; i < Count; i++)
    {
        double Height = (double)NoiseOut[i] * NoiseScale;
        double EditDensity = EditStore->Sample(Samples[i].Position);
        Samples[i].Density = Samples[i].Dist - (RootExtent * 0.9 + Height) + EditDensity;
    }

    // 5. Write back densities to all nodes that share each corner
    for (int32 i = 0; i < Count; i++)
    {
        for (double* Target : Samples[i].Targets)
            *Target = Samples[i].Density;
    }

    // 6. Finalize edges/QEF
    FinalizeSubtree(Node, EditCenter, SearchRadius);
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

void FAdaptiveOctree::UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk)
{
    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> Triangles;
    
    VertexMap.Reserve(1024);
    UniqueVertices.Reserve(1024);
    Triangles.Reserve(2048);

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

            double LX = WorldPos.X - InChunk->ChunkCenter.X;
            double LY = WorldPos.Y - InChunk->ChunkCenter.Y;
            double LZ = WorldPos.Z - InChunk->ChunkCenter.Z;
            FVector LocalPos(LX, LY, LZ);

            AllEdgeData[edgeIdx].Vertices[i].Position = QuantizePosition(LocalPos, QuantizationGrid);
            AllEdgeData[edgeIdx].Vertices[i].OriginalPosition = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = nodesToMesh[i]->DualContourNormal;
            AllEdgeData[edgeIdx].Vertices[i].Color = FColor::Green;
        }
    });

    for (int32 edgeIdx = 0; edgeIdx < AllEdgeData.Num(); edgeIdx++) {
        if (!AllEdgeData[edgeIdx].IsValid) continue;

        const auto& currentEdge = AllEdgeData[edgeIdx].Edge.GetValue();
        const TArray<FMeshVertex>& EdgeVertices = AllEdgeData[edgeIdx].Vertices;

        int32 VertexIndices[4];
        int32 NumEdgeVerts = EdgeVertices.Num();

        for (int i = 0; i < NumEdgeVerts; i++) {
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

    TSharedPtr<FMeshStreamData> UpdatedData = MakeShared<FMeshStreamData>();
    UpdatedData->MeshGroupKey = InChunk->ChunkMeshData->MeshGroupKey;
    UpdatedData->MeshSectionKey = InChunk->ChunkMeshData->MeshSectionKey;

    auto PositionStream = UpdatedData->GetPositionStream();
    auto TangentStream = UpdatedData->GetTangentStream();
    auto ColorStream = UpdatedData->GetColorStream();
    auto TexCoordStream = UpdatedData->GetTexCoordStream();
    auto TriangleStream = UpdatedData->GetTriangleStream();
    auto PolygroupStream = UpdatedData->GetPolygroupStream();

    PositionStream.SetNumUninitialized(UniqueVertices.Num());
    TangentStream.SetNumUninitialized(UniqueVertices.Num());
    ColorStream.SetNumUninitialized(UniqueVertices.Num());
    TexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    TriangleStream.SetNumUninitialized(Triangles.Num());
    PolygroupStream.SetNumUninitialized(Triangles.Num());

    FVector ChunkCenter = InChunk->ChunkCenter;

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
        FMeshVertex& Vertex = UniqueVertices[VertIdx];
        FVector WorldPos = FVector(Vertex.OriginalPosition) + ChunkCenter;

        PositionStream.Set(VertIdx, Vertex.OriginalPosition);
        FRealtimeMeshTangentsHighPrecision Tangent;
        Tangent.SetNormal(FVector3f(Vertex.Normal));
        TangentStream.Set(VertIdx, Tangent);
        ColorStream.Set(VertIdx, Vertex.Color);
        TexCoordStream.Set(VertIdx, ComputeTriplanarUV(WorldPos, Vertex.Normal));
        });

    ParallelFor(Triangles.Num(), [&](int32 TriIdx) {
        TriangleStream.Set(TriIdx, Triangles[TriIdx]);
        PolygroupStream.Set(TriIdx, 0);
        });

    InChunk->ChunkMeshData = UpdatedData;
    InChunk->IsDirty = (Triangles.Num() > 0);
}

void AppendUniqueEdges(const TArray<FNodeEdge>& InAppendEdges, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap)
{
    for (const FNodeEdge& anEdge : InAppendEdges)
    {
        FEdgeKey Key(anEdge);
        int32* ExistingIdx = EdgeMap.Find(Key);

        if (ExistingIdx)
        {
            // Keep the one with the smaller Distance (same logic as before)
            if (anEdge.Distance < OutNodeEdges[*ExistingIdx].Distance)
            {
                OutNodeEdges[*ExistingIdx] = anEdge;
            }
        }
        else
        {
            int32 NewIdx = OutNodeEdges.Num();
            OutNodeEdges.Add(anEdge);
            EdgeMap.Add(Key, NewIdx);
        }
    }
}

void FAdaptiveOctree::UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged)
{
    if (Node->IsLeaf())
    {
        if (Node->ShouldSplit(CameraPosition, InScreenSpaceThreshold, InCameraFOV))
        {
            OutChanged = true;
            Node->Split();
            for (int i = 0; i < 8; i++)
            {
                ComputeNodeData(Node->Children[i]);
                AppendUniqueEdges(Node->Children[i]->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            }
            return;
        }
        else if (Node->Parent.IsValid() && Node->Parent.Pin()->ShouldMerge(CameraPosition, InScreenSpaceThreshold, InCameraFOV) && Node->TreeIndex.Last() == 7)
        {
            OutChanged = true;
            auto ParentPtr = Node->Parent.Pin();
            ParentPtr->Merge();
            AppendUniqueEdges(ParentPtr->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            return;
        }
        else
        {
            AppendUniqueEdges(Node->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            return;
        }
    }
    else
    {
        for (int i = 0; i < 8; i++)
            UpdateLodRecursive(Node->Children[i], CameraPosition, InScreenSpaceThreshold, InCameraFOV, OutNodeEdges, EdgeMap, OutChanged);
    }
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    ChunkMap.GenerateKeyArray(Chunks);
    int32 NumChunks = Chunks.Num();

    TArray<TSharedPtr<FMeshChunk>> ModifiedChunks;
    ModifiedChunks.SetNumZeroed(NumChunks);

    ParallelFor(Chunks.Num(), [&](int32 idx)
        {
            TArray<FNodeEdge> tChunkEdges;
            TMap<FEdgeKey, int32> tEdgeMap;
            bool tChanged = false;
            UpdateLodRecursive(Chunks[idx], CameraPosition, InScreenSpaceThreshold, InCameraFOV, tChunkEdges, tEdgeMap, tChanged);
            if (tChanged) {
                auto MeshChunk = ChunkMap[Chunks[idx]];
                MeshChunk->ChunkEdges = tChunkEdges;
                ModifiedChunks[idx] = MeshChunk;
            }
        });

    ParallelFor(NumChunks, [&](int32 i)
        {
            if (ModifiedChunks[i].IsValid()) {
                UpdateMeshChunkStreamData(ModifiedChunks[i]);
            }
        });
}

void FAdaptiveOctree::UpdateMesh()
{
    if (!MeshChunksInitialized) return;

    // Use a reference to avoid incrementing shared pointer ref counts unnecessarily during iteration
    for (auto& It : ChunkMap)
    {
        TSharedPtr<FMeshChunk> Chunk = It.Value;

        if (!Chunk.IsValid()) continue;

        if (Chunk->IsDirty)
        {
            Chunk->UpdateComponent(Chunk);
        }
    }
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

void FAdaptiveOctree::ComputeNodeData(TSharedPtr<FAdaptiveOctreeNode> Node)
{
    float xPos[8], yPos[8], zPos[8], noiseOut[8];
    double dists[8];

    double NoiseScale = RootExtent * 0.1;
    FVector PlanetCenter = Root->Center;

    for (int i = 0; i < 8; i++)
    {
        FVector PlanetRel = Node->Corners[i].Position - PlanetCenter;
        dists[i] = PlanetRel.Size();
        FVector Dir = PlanetRel / dists[i];
        FVector SurfacePos = Dir * RootExtent;
        xPos[i] = (float)(SurfacePos.X / NoiseScale);
        yPos[i] = (float)(SurfacePos.Y / NoiseScale);
        zPos[i] = (float)(SurfacePos.Z / NoiseScale);
    }

    DensityFunction(8, xPos, yPos, zPos, noiseOut);

    for (int i = 0; i < 8; i++)
    {
        double height = (double)noiseOut[i] * NoiseScale;
        Node->Corners[i].Density = dists[i] - (RootExtent * 0.9 + height) + EditStore->Sample(Node->Corners[i].Position);
    }

    Node->FinalizeFromExistingCorners();
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Nodes;

    // Calculate the exact mathematical midpoint of the edge
    FVector Origin = Edge.Corners[0].Position;
    FVector End = Edge.Corners[1].Position;
    FVector Midpoint = (Origin + End) * 0.5;
    
    // Adaptive epsilon based on current node size to handle floating-point drift
    double Epsilon = ChunkExtent * 1e-6;

    // Helper lambda for biased top-down traversal
    auto GetLeafWithBias = [&](bool BiasPerp1, bool BiasPerp2) -> TSharedPtr<FAdaptiveOctreeNode> {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;

        while (CurrentNode.IsValid() && !CurrentNode->IsLeaf()) {
            int ChildIndex = 0;

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

TSharedPtr<FAdaptiveOctreeNode> FAdaptiveOctree::GetChunkNodeByPoint(FVector Position)
{
    TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;
    while (CurrentNode.IsValid())
    {
        if (CurrentNode->TreeIndex.Num() == ChunkDepth)
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

void FAdaptiveOctree::Clear()
{
    Root.Reset();
}