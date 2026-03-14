#include "FAdaptiveOctree.h"

FAdaptiveOctree::FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InSurfaceMaterial, UMaterialInterface* InOceanMaterial, TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    EditStore = InEditStore;
    RootExtent = InRootExtent;
    ChunkExtent = InRootExtent / FMath::Pow(2.0, (double)InChunkDepth);
    CachedParentActor = InParentActor;
    CachedSurfaceMaterial = InSurfaceMaterial;
    CachedOceanMaterial = InOceanMaterial;
    ChunkDepth = InChunkDepth;

    Root = MakeShared<FAdaptiveOctreeNode>(InCenter, InRootExtent, InChunkDepth, InMinDepth, InMaxDepth);
    ComputeNodeData(Root.Get());
    SplitToDepth(Root.Get(), InChunkDepth);

    PopulateChunks();
}

void FAdaptiveOctree::SplitToDepth(FAdaptiveOctreeNode* Node, int InMinDepth)
{
    if (!Node) return;

    if (Node->Index.Depth < InMinDepth)
    {
        SplitAndComputeChildren(Node);
        for (int i = 0; i < 8; i++)
        {
            if (Node->Children[i])
            {
                SplitToDepth(Node->Children[i].Get(), InMinDepth);
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
            FAdaptiveOctreeNode* NeighborNode = GetLeafNodeByPoint(NeighborPos);
            if (!NeighborNode) continue;
            if (!NeighborNode->IsSurfaceNode)
                NeighborChunks.AddUnique(NeighborNode->AsShared());
        }
    }
    Chunks.Append(NeighborChunks);

    // Process all chunk data in bulk -- parallel
    ParallelFor(Chunks.Num(), [&](int32 i) {
        ReconstructSubtree(Chunks[i].Get(), FVector::ZeroVector, 0);
        });

    // Build mesh chunks -- parallel edge gather + stream build
    TArray<TSharedPtr<FMeshChunk>> NewChunks;
    NewChunks.SetNum(Chunks.Num());

    ParallelFor(Chunks.Num(), [&](int32 i) {
        NewChunks[i] = MakeShared<FMeshChunk>();
        NewChunks[i]->CachedParentActor = CachedParentActor;
        NewChunks[i]->CachedSurfaceMaterial = CachedSurfaceMaterial;
        NewChunks[i]->CachedOceanMaterial = CachedOceanMaterial;
        NewChunks[i]->InitializeData(Chunks[i]->Center, Chunks[i]->Extent);

        TArray<FNodeEdge> Edges;
        TMap<FEdgeKey, int32> EdgeMap;
        GatherLeafEdges(Chunks[i].Get(), Edges, EdgeMap);
        NewChunks[i]->ChunkEdges = Edges;
        UpdateMeshChunkStreamData(NewChunks[i]);
        NewChunks[i]->IsDirty = (NewChunks[i]->SurfaceMeshData->GetPositionStream().Num() > 0);
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
            (*Found)->SurfaceMeshData->ResetStreams();
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
        NewChunk->CachedSurfaceMaterial = CachedSurfaceMaterial;
        NewChunk->CachedOceanMaterial = CachedOceanMaterial;
        NewChunk->InitializeData(ChunkNode->Center, ChunkNode->Extent);
        ChunkMap.Add(ChunkNode, NewChunk);
        OutDirtyChunks.Add({ ChunkNode, NewChunk });

        // Buffer neighbors
        const double Offset = ChunkNode->Extent * 2.0;
        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + Directions[i] * Offset;
            FAdaptiveOctreeNode* NeighborRaw = GetLeafNodeByPoint(NeighborPos);

            if (!NeighborRaw) continue;
            if (NeighborRaw->IsSurfaceNode) continue;

            // Only convert to TSharedPtr at the ChunkMap boundary
            TSharedPtr<FAdaptiveOctreeNode> Neighbor = NeighborRaw->AsShared();
            if (ChunkMap.Contains(Neighbor)) continue;

            TSharedPtr<FMeshChunk> NeighborChunk = MakeShared<FMeshChunk>();
            NeighborChunk->CachedParentActor = CachedParentActor;
            NeighborChunk->CachedSurfaceMaterial = CachedSurfaceMaterial;
            NeighborChunk->CachedOceanMaterial = CachedOceanMaterial;
            NeighborChunk->InitializeData(NeighborRaw->Center, NeighborRaw->Extent);
            ChunkMap.Add(Neighbor, NeighborChunk);
            OutDirtyChunks.Add({ Neighbor, NeighborChunk });
        }
    }
}

void FAdaptiveOctree::ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    int depth = EditStore->GetDepthForBrushRadius(InEditRadius, InEditResolution);
    TArray<FVector> AffectedChunkCenters = EditStore->ApplySphericalEdit(InEditCenter, InEditRadius, InEditStrength, depth);
    double ReconstructRadius = InEditRadius * 2.5;

    // Resolve chunk nodes -- collect as shared ptrs since we need them for ChunkMap operations
    TArray<TSharedPtr<FAdaptiveOctreeNode>> ChunkNodes;
    for (const FVector& Center : AffectedChunkCenters)
    {
        FAdaptiveOctreeNode* ChunkNodeRaw = GetChunkNodeByPoint(Center);
        if (ChunkNodeRaw)
            ChunkNodes.Add(ChunkNodeRaw->AsShared());
    }

    // Stage 1: Parallel -- recompute all node data
    ParallelFor(ChunkNodes.Num(), [&](int32 i) {
        ReconstructSubtree(ChunkNodes[i].Get(), InEditCenter, ReconstructRadius);
        });

    // Stage 2: Serial -- update chunk map
    TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>> DirtyChunks;
    for (auto& ChunkNode : ChunkNodes)
    {
        UpdateChunkMap(ChunkNode, DirtyChunks);
    }

    // Stage 3: Parallel -- gather edges + rebuild mesh streams
    ParallelFor(DirtyChunks.Num(), [&](int32 i) {
        TArray<FNodeEdge> NewEdges;
        TMap<FEdgeKey, int32> EdgeMap;
        GatherLeafEdges(DirtyChunks[i].Key.Get(), NewEdges, EdgeMap);
        DirtyChunks[i].Value->ChunkEdges = NewEdges;
        UpdateMeshChunkStreamData(DirtyChunks[i].Value);
        });
}

void FAdaptiveOctree::GatherUniqueCorners(FAdaptiveOctreeNode* Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& CornerMap, double QuantizeGrid, FVector EditCenter, double SearchRadius)
{
    if (!Node) return;

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
        GatherUniqueCorners(Node->Children[i].Get(), Samples, CornerMap, QuantizeGrid, EditCenter, SearchRadius);
}

void FAdaptiveOctree::FinalizeSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius)
{
    if (!Node) return;

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
            FinalizeSubtree(Node->Children[i].Get(), EditCenter, SearchRadius);
    }

    // Then this node
    Node->FinalizeFromExistingCorners(Root->Center);
}

void FAdaptiveOctree::ReconstructSubtree(FAdaptiveOctreeNode* Node, FVector EditCenter, double SearchRadius)
{
    // 1. Gather unique corners from affected nodes
    TMap<FIntVector, int32> CornerMap;
    TArray<FCornerSample> Samples;
    Samples.Reserve(4096);
    CornerMap.Reserve(4096);
    double QuantizeGrid = ChunkExtent * 1e-8;

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

    ParallelFor(Count, [&](int32 i)
        {
            FVector PlanetRel = Samples[i].Position - PlanetCenter;
            Samples[i].Dist = PlanetRel.Size();
            FVector Dir = PlanetRel / Samples[i].Dist;
            FVector SurfacePos = Dir * RootExtent;
            XPos[i] = (float)(SurfacePos.X / NoiseScale);
            YPos[i] = (float)(SurfacePos.Y / NoiseScale);
            ZPos[i] = (float)(SurfacePos.Z / NoiseScale);
        });

    // 3. One bulk noise call
    DensityFunction(Count, XPos.GetData(), YPos.GetData(), ZPos.GetData(), NoiseOut.GetData());

    // 4. SDF + edits -- uses pre-computed distances
    ParallelFor(Count, [&](int32 i)
        {
            double Height = (double)NoiseOut[i] * NoiseScale;
            double EditDensity = EditStore->Sample(Samples[i].Position);
            Samples[i].Density = Samples[i].Dist - (RootExtent * 0.9 + Height) + EditDensity;
        });

    // 5. Write back densities to all nodes that share each corner
    for (int32 i = 0; i < Count; i++)
    {
        float FinalDensity = (float)Samples[i].Density;
        for (float* Target : Samples[i].Targets)
            *Target = FinalDensity;
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

void FAdaptiveOctree::UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk)
{
    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> SrfTriangles;
    TArray<FIndex3UI> OcnTriangles;

    VertexMap.Reserve(1024);
    UniqueVertices.Reserve(1024);
    SrfTriangles.Reserve(2048);

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(InChunk->ChunkEdges.Num());

    double OceanRadius = RootExtent * 0.9;
    double OceanTriThreshold = -1000;

    ParallelFor(InChunk->ChunkEdges.Num(), [&](int32 edgeIdx) {
        const FNodeEdge currentEdge = InChunk->ChunkEdges[edgeIdx];
        FEdgeNeighbors neighbors = SampleNodesAroundEdge(currentEdge);

        if (!InChunk->ShouldProcessEdge(currentEdge, neighbors)) {
            AllEdgeData[edgeIdx].IsValid = false;
            return;
        }

        AllEdgeData[edgeIdx].IsValid = true;
        AllEdgeData[edgeIdx].Edge = currentEdge;
        AllEdgeData[edgeIdx].Vertices.SetNumZeroed(neighbors.Count);

        FVector PlanetCenter = Root->Center;
        double NormRadius = Root->Extent * 0.9;
        for (int i = 0; i < neighbors.Count; i++) {
            FAdaptiveOctreeNode* NodePtr = neighbors.Nodes[i];
            FVector LocalPos(NodePtr->DualContourPosition - InChunk->ChunkCenter);
            FVector WorldPos = NodePtr->DualContourPosition;
            FVector NormPos = NodePtr->ComputeNormalizedPosition(PlanetCenter, NormRadius);
            FVector NormLocalPos = NormPos - InChunk->ChunkCenter;
            double Dist = FVector::Dist(WorldPos, PlanetCenter);

            AllEdgeData[edgeIdx].Vertices[i].Position = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].NormalizedPosition = NormLocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = FVector(NodePtr->DualContourNormal);
            AllEdgeData[edgeIdx].Vertices[i].Depth = (float)(OceanRadius - Dist);
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

        if (NumEdgeVerts >= 3
            && VertexIndices[0] != VertexIndices[1]
            && VertexIndices[1] != VertexIndices[2]
            && VertexIndices[0] != VertexIndices[2])
        {
            FIndex3UI tri0 = FlipWinding
                ? FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[1])
                : FIndex3UI(VertexIndices[0], VertexIndices[1], VertexIndices[2]);
            SrfTriangles.Add(tri0);

            if (UniqueVertices[tri0.V0].Depth > OceanTriThreshold ||
                UniqueVertices[tri0.V1].Depth > OceanTriThreshold ||
                UniqueVertices[tri0.V2].Depth > OceanTriThreshold)
                OcnTriangles.Add(tri0);

            if (NumEdgeVerts == 4
                && VertexIndices[0] != VertexIndices[3]
                && VertexIndices[2] != VertexIndices[3])
            {
                FIndex3UI tri1 = FlipWinding
                    ? FIndex3UI(VertexIndices[0], VertexIndices[3], VertexIndices[2])
                    : FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[3]);
                SrfTriangles.Add(tri1);

                if (UniqueVertices[tri1.V0].Depth > OceanTriThreshold ||
                    UniqueVertices[tri1.V1].Depth > OceanTriThreshold ||
                    UniqueVertices[tri1.V2].Depth > OceanTriThreshold)
                    OcnTriangles.Add(tri1);
            }
        }
    }

    // Compact ocean vertices -- only include vertices actually referenced by OcnTriangles

    TSharedPtr<FMeshStreamData> UpdatedSurfaceData = MakeShared<FMeshStreamData>();
    UpdatedSurfaceData->MeshGroupKey = InChunk->SurfaceMeshData->MeshGroupKey;
    UpdatedSurfaceData->MeshSectionKey = InChunk->SurfaceMeshData->MeshSectionKey;

    TSharedPtr<FMeshStreamData> UpdatedOceanData = MakeShared<FMeshStreamData>();
    UpdatedOceanData->MeshGroupKey = InChunk->OceanMeshData->MeshGroupKey;
    UpdatedOceanData->MeshSectionKey = InChunk->OceanMeshData->MeshSectionKey;

    auto SrfPositionStream = UpdatedSurfaceData->GetPositionStream();
    auto SrfTangentStream = UpdatedSurfaceData->GetTangentStream();
    auto SrfColorStream = UpdatedSurfaceData->GetColorStream();
    auto SrfTexCoordStream = UpdatedSurfaceData->GetTexCoordStream();
    auto SrfTriangleStream = UpdatedSurfaceData->GetTriangleStream();
    auto SrfPolygroupStream = UpdatedSurfaceData->GetPolygroupStream();

    auto OcnPositionStream = UpdatedOceanData->GetPositionStream();
    auto OcnTangentStream = UpdatedOceanData->GetTangentStream();
    auto OcnColorStream = UpdatedOceanData->GetColorStream();
    auto OcnTexCoordStream = UpdatedOceanData->GetTexCoordStream();
    auto OcnTriangleStream = UpdatedOceanData->GetTriangleStream();
    auto OcnPolygroupStream = UpdatedOceanData->GetPolygroupStream();

    SrfPositionStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTangentStream.SetNumUninitialized(UniqueVertices.Num());
    SrfColorStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTriangleStream.SetNumUninitialized(SrfTriangles.Num());
    SrfPolygroupStream.SetNumUninitialized(SrfTriangles.Num());

    OcnPositionStream.SetNumUninitialized(UniqueVertices.Num());
    OcnTangentStream.SetNumUninitialized(UniqueVertices.Num());
    OcnColorStream.SetNumUninitialized(UniqueVertices.Num());
    OcnTexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    OcnTriangleStream.SetNumUninitialized(OcnTriangles.Num());
    OcnPolygroupStream.SetNumUninitialized(OcnTriangles.Num());

    FVector ChunkCenter = InChunk->ChunkCenter;

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
        FMeshVertex& Vertex = UniqueVertices[VertIdx];
        FVector WorldPos = FVector(Vertex.Position) + ChunkCenter;

        SrfPositionStream.Set(VertIdx, Vertex.Position);
        FRealtimeMeshTangentsHighPrecision Tangent;
        Tangent.SetNormal(FVector3f(Vertex.Normal));
        SrfTangentStream.Set(VertIdx, Tangent);
        SrfColorStream.Set(VertIdx, Vertex.Color);
        SrfTexCoordStream.Set(VertIdx, ComputeTriplanarUV(WorldPos, Vertex.Normal));
        });

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
        FMeshVertex& Vertex = UniqueVertices[VertIdx];
        FVector WorldPos = FVector(Vertex.Position) + ChunkCenter;

        OcnPositionStream.Set(VertIdx, Vertex.NormalizedPosition);
        FRealtimeMeshTangentsHighPrecision OcnTangent;
        FVector OcnNormal = (FVector(Vertex.NormalizedPosition) + ChunkCenter - Root->Center).GetSafeNormal();
        OcnTangent.SetNormal(FVector3f(OcnNormal));
        OcnTangentStream.Set(VertIdx, OcnTangent);

        float absDepth = FMath::Max(Vertex.Depth, 0.0f);
        uint32 intDepth = (uint32)FMath::Min(absDepth * 1000.0f, 4294967295.0f);
        uint8 R = (intDepth >> 24) & 0xFF;
        uint8 G = (intDepth >> 16) & 0xFF;
        uint8 B = (intDepth >> 8) & 0xFF;
        uint8 A = intDepth & 0xFF;
        OcnColorStream.Set(VertIdx, FColor(R, G, B, A));
        OcnTexCoordStream.Set(VertIdx, ComputeTriplanarUV(FVector(Vertex.NormalizedPosition) + ChunkCenter, OcnNormal));
        });

    ParallelFor(SrfTriangles.Num(), [&](int32 TriIdx) {
        SrfTriangleStream.Set(TriIdx, SrfTriangles[TriIdx]);
        SrfPolygroupStream.Set(TriIdx, 0);
        });

    ParallelFor(OcnTriangles.Num(), [&](int32 TriIdx) {
        OcnTriangleStream.Set(TriIdx, OcnTriangles[TriIdx]);
        OcnPolygroupStream.Set(TriIdx, 0);
        });

    InChunk->SurfaceMeshData = UpdatedSurfaceData;
    if (UpdatedOceanData->GetTriangleStream().Num() > 0)
        InChunk->OceanMeshData = UpdatedOceanData;

    InChunk->IsDirty = (SrfTriangles.Num() > 0);
}

void AppendUniqueEdges(const TArray<FNodeEdge>& InAppendEdges, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap)
{
    for (const FNodeEdge& anEdge : InAppendEdges)
    {
        const FEdgeKey& Key = anEdge.CachedKey;
        int32* ExistingIdx = EdgeMap.Find(Key);

        if (ExistingIdx)
        {
            if (anEdge.Size < OutNodeEdges[*ExistingIdx].Size)
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

void FAdaptiveOctree::UpdateLodRecursive(FAdaptiveOctreeNode* Node, FVector CameraPosition, double InScreenSpaceThreshold, double InFOVScale, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged)
{
    if (Node->IsLeaf())
    {
        if (Node->ShouldSplit(Root->Center, CameraPosition, InScreenSpaceThreshold, InFOVScale))
        {
            OutChanged = true;
            SplitAndComputeChildren(Node);
            for (int i = 0; i < 8; i++)
                AppendUniqueEdges(Node->Children[i]->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            return;
        }
        else if (Node->Index.LastChild() == 7 && Node->Parent.IsValid())
        {
            FAdaptiveOctreeNode* ParentPtr = Node->Parent.Pin().Get();
            if (ParentPtr && ParentPtr->ShouldMerge(CameraPosition, InScreenSpaceThreshold, InFOVScale))
            {
                OutChanged = true;
                ParentPtr->Merge();
                AppendUniqueEdges(ParentPtr->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
                return;
            }
        }

        AppendUniqueEdges(Node->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
        return;
    }

    for (int i = 0; i < 8; i++)
        if (Node->Children[i])
            UpdateLodRecursive(Node->Children[i].Get(), CameraPosition, InScreenSpaceThreshold, InFOVScale, OutNodeEdges, EdgeMap, OutChanged);
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    ChunkMap.GenerateKeyArray(Chunks);
    int32 NumChunks = Chunks.Num();

    TArray<TSharedPtr<FMeshChunk>> ModifiedChunks;
    ModifiedChunks.SetNumZeroed(NumChunks);

    // Pre-compute once for entire pass
    double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));

    double t0 = FPlatformTime::Seconds();
    ParallelFor(NumChunks, [&](int32 idx) {
        FAdaptiveOctreeNode* ChunkNode = Chunks[idx].Get();
        double DistSq = FMath::Max(FVector::DistSquared(ChunkNode->Center, CameraPosition), 1.0);
        bool CouldSplit = FAdaptiveOctreeNode::EvaluateSplit(ChunkNode->Extent, DistSq, FOVScale, InScreenSpaceThreshold, ChunkNode->Index.Depth, ChunkNode->DepthBounds[1], ChunkNode->DepthBounds[2]);
        double SmallestExtent = ChunkNode->Extent / (double)(1 << (ChunkNode->DepthBounds[1] - ChunkNode->Index.Depth));
        bool CouldMerge = FAdaptiveOctreeNode::EvaluateMerge(SmallestExtent, DistSq, FOVScale, InScreenSpaceThreshold, ChunkNode->DepthBounds[1], ChunkNode->DepthBounds[0], ChunkNode->DepthBounds[2]);

        if (!CouldSplit && !CouldMerge) return; //Early out

        TArray<FNodeEdge> tChunkEdges;
        TMap<FEdgeKey, int32> tEdgeMap;
        bool tChanged = false;
        UpdateLodRecursive(ChunkNode, CameraPosition, InScreenSpaceThreshold, FOVScale, tChunkEdges, tEdgeMap, tChanged);

        if (!tChanged) return;

        auto MeshChunk = ChunkMap[Chunks[idx]];
        MeshChunk->ChunkEdges = tChunkEdges;
        ModifiedChunks[idx] = MeshChunk;
        });

    ParallelFor(ModifiedChunks.Num(), [&](int32 idx)
        {
            if (ModifiedChunks[idx].IsValid())
                UpdateMeshChunkStreamData(ModifiedChunks[idx]);
        });

    double t1 = FPlatformTime::Seconds();
    if ((t1 - t0) * 1000.0 > 100.0)
    {
        UE_LOG(LogTemp, Log, TEXT("[LOD] LODPass: %.2fms"),
            (t1 - t0) * 1000.0);
    }
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

void FAdaptiveOctree::GatherLeafEdges(FAdaptiveOctreeNode* Node, TArray<FNodeEdge>& OutEdges, TMap<FEdgeKey, int32>& EdgeMap)
{
    if (!Node) return;
    if (Node->IsLeaf())
    {
        AppendUniqueEdges(Node->GetSignChangeEdges(), OutEdges, EdgeMap);
        return;
    }
    for (int i = 0; i < 8; i++)
        GatherLeafEdges(Node->Children[i].Get(), OutEdges, EdgeMap);
}

void FAdaptiveOctree::SplitAndComputeChildren(FAdaptiveOctreeNode* Node)
{
    static const int32 ChildCornerSources[8][8] = {
        {  0,  8, 12, 24, 16, 22, 20, 26 },
        {  8,  1, 24, 13, 22, 17, 26, 21 },
        { 12, 24,  2,  9, 20, 26, 18, 23 },
        { 24, 13,  9,  3, 26, 21, 23, 19 },
        { 16, 22, 20, 26,  4, 10, 14, 25 },
        { 22, 17, 26, 21, 10,  5, 25, 15 },
        { 20, 26, 18, 23, 14, 25,  6, 11 },
        { 26, 21, 23, 19, 25, 15, 11,  7 },
    };

    static const int8 GridCoords[27][3] = {
        {-1,-1,-1},{+1,-1,-1},{-1,+1,-1},{+1,+1,-1},
        {-1,-1,+1},{+1,-1,+1},{-1,+1,+1},{+1,+1,+1},
        { 0,-1,-1},{ 0,+1,-1},{ 0,-1,+1},{ 0,+1,+1},
        {-1, 0,-1},{+1, 0,-1},{-1, 0,+1},{+1, 0,+1},
        {-1,-1, 0},{+1,-1, 0},{-1,+1, 0},{+1,+1, 0},
        {-1, 0, 0},{+1, 0, 0},{ 0,-1, 0},{ 0,+1, 0},
        { 0, 0,-1},{ 0, 0,+1},{ 0, 0, 0}
    };

    // CoordToGrid[x+1][y+1][z+1]
    static const int8 CoordToGrid[3][3][3] = {
        { {  0, 16,  4 }, { 12, 20, 14 }, {  2, 18,  6 } },
        { {  8, 22, 10 }, { 24, 26, 25 }, {  9, 23, 11 } },
        { {  1, 17,  5 }, { 13, 21, 15 }, {  3, 19,  7 } },
    };

    static const int32 FaceCorners[6][4] = {
        { 0, 2, 4, 6 },
        { 1, 3, 5, 7 },
        { 0, 1, 4, 5 },
        { 2, 3, 6, 7 },
        { 0, 1, 2, 3 },
        { 4, 5, 6, 7 },
    };

    Node->Split();

    // --- Stage 1: Build grid positions ---
    FVector GridPositions[27];
    double  GridDensities[27];

    // G0-G7: parent corners
    for (int i = 0; i < 8; i++)
    {
        GridPositions[i] = Node->Corners[i].Position;
        GridDensities[i] = Node->Corners[i].Density;
    }

    // G8-G19: edge midpoints
    for (int i = 0; i < 12; i++)
    {
        int a = OctreeConstants::EdgePairs[i][0];
        int b = OctreeConstants::EdgePairs[i][1];
        GridPositions[8 + i] = (Node->Corners[a].Position + Node->Corners[b].Position) * 0.5;
    }

    // G20-G25: face centers
    for (int i = 0; i < 6; i++)
    {
        GridPositions[20 + i] = (
            Node->Corners[FaceCorners[i][0]].Position +
            Node->Corners[FaceCorners[i][1]].Position +
            Node->Corners[FaceCorners[i][2]].Position +
            Node->Corners[FaceCorners[i][3]].Position) * 0.25;
    }

    // G26: body center
    GridPositions[26] = Node->Center;

    // --- Stage 2: Sample 19 new densities ---
    const int32 NewCount = 19;
    float XPos[19], YPos[19], ZPos[19], NoiseOut[19];

    double NoiseScale = RootExtent * 0.1;
    FVector PlanetCenter = Root->Center;

    for (int32 i = 0; i < NewCount; i++)
    {
        FVector PlanetRel = GridPositions[8 + i] - PlanetCenter;
        double Dist = PlanetRel.Size();
        FVector Dir = (Dist > 1e-10) ? (PlanetRel / Dist) : FVector::UpVector;
        FVector SurfacePos = Dir * RootExtent;
        XPos[i] = (float)(SurfacePos.X / NoiseScale);
        YPos[i] = (float)(SurfacePos.Y / NoiseScale);
        ZPos[i] = (float)(SurfacePos.Z / NoiseScale);
    }

    DensityFunction(NewCount, XPos, YPos, ZPos, NoiseOut);

    for (int32 i = 0; i < NewCount; i++)
    {
        double Height = (double)NoiseOut[i] * NoiseScale;
        FVector PlanetRel = GridPositions[8 + i] - PlanetCenter;
        double Dist = PlanetRel.Size();
        GridDensities[8 + i] = Dist - (RootExtent * 0.9 + Height) + EditStore->Sample(GridPositions[8 + i]);
    }

    // --- Stage 3: Compute 27 normals from grid gradients ---
    FVector GridNormals[27];

    for (int32 gi = 0; gi < 27; gi++)
    {
        int8 cx = GridCoords[gi][0];
        int8 cy = GridCoords[gi][1];
        int8 cz = GridCoords[gi][2];

        double dX;
        if (cx <= 0)
        {
            int32 ngi = CoordToGrid[cx + 2][cy + 1][cz + 1];
            dX = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = CoordToGrid[cx][cy + 1][cz + 1];
            dX = -(GridDensities[ngi] - GridDensities[gi]);
        }

        double dY;
        if (cy <= 0)
        {
            int32 ngi = CoordToGrid[cx + 1][cy + 2][cz + 1];
            dY = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = CoordToGrid[cx + 1][cy][cz + 1];
            dY = -(GridDensities[ngi] - GridDensities[gi]);
        }

        double dZ;
        if (cz <= 0)
        {
            int32 ngi = CoordToGrid[cx + 1][cy + 1][cz + 2];
            dZ = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = CoordToGrid[cx + 1][cy + 1][cz];
            dZ = -(GridDensities[ngi] - GridDensities[gi]);
        }

        FVector Normal(dX, dY, dZ);
        if (!Normal.Normalize())
            Normal = (GridPositions[gi] - PlanetCenter).GetSafeNormal();
        GridNormals[gi] = Normal;
    }

    // --- Stage 4: Assign to children and finalize ---
    for (int ci = 0; ci < 8; ci++)
    {
        for (int k = 0; k < 8; k++)
        {
            int32 gi = ChildCornerSources[ci][k];
            Node->Children[ci]->Corners[k].Position = GridPositions[gi];
            Node->Children[ci]->Corners[k].Density = (float)GridDensities[gi];
            Node->Children[ci]->Corners[k].Normal = FVector3f(GridNormals[gi]);
        }
        Node->Children[ci]->FinalizeFromExistingCorners(Root->Center, true); // normals already computed from grid
    }
}

void FAdaptiveOctree::ComputeNodeData(FAdaptiveOctreeNode* Node)
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
        Node->Corners[i].Density = (float)(dists[i] - (RootExtent * 0.9 + height) + EditStore->Sample(Node->Corners[i].Position));
    }

    Node->FinalizeFromExistingCorners(Root->Center);
}

FEdgeNeighbors FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    FEdgeNeighbors Result;

    // Calculate the exact mathematical midpoint of the edge
    FVector Origin = Edge.Corners[0].Position;
    FVector End = Edge.Corners[1].Position;
    FVector Midpoint = (Origin + End) * 0.5;

    double Epsilon = ChunkExtent * 1e-8;

    // Helper lambda for biased top-down traversal -- uses raw pointers throughout
    auto GetLeafWithBias = [&](bool BiasPerp1, bool BiasPerp2) -> FAdaptiveOctreeNode* {
        FAdaptiveOctreeNode* CurrentNode = Root.Get();

        while (CurrentNode && !CurrentNode->IsLeaf()) {
            int ChildIndex = 0;

            auto CheckSide = [&](int AxisIndex, bool PositiveBias) {
                double Diff = Midpoint[AxisIndex] - CurrentNode->Center[AxisIndex];
                if (FMath::Abs(Diff) <= Epsilon) {
                    return PositiveBias;
                }
                return Diff > 0.0;
                };

            if (Edge.Axis == 0) { // X-axis edge
                if (Midpoint.X >= CurrentNode->Center.X) ChildIndex |= 1;
                if (CheckSide(1, BiasPerp1)) ChildIndex |= 2;
                if (CheckSide(2, BiasPerp2)) ChildIndex |= 4;
            }
            else if (Edge.Axis == 1) { // Y-axis edge
                if (CheckSide(0, BiasPerp2)) ChildIndex |= 1;
                if (Midpoint.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
                if (CheckSide(2, BiasPerp1)) ChildIndex |= 4;
            }
            else if (Edge.Axis == 2) { // Z-axis edge
                if (CheckSide(0, BiasPerp1)) ChildIndex |= 1;
                if (CheckSide(1, BiasPerp2)) ChildIndex |= 2;
                if (Midpoint.Z >= CurrentNode->Center.Z) ChildIndex |= 4;
            }

            FAdaptiveOctreeNode* Child = CurrentNode->Children[ChildIndex].Get();
            if (!Child) return CurrentNode; // Shouldn't happen in normal operation
            CurrentNode = Child;
        }
        return CurrentNode;
        };

    FAdaptiveOctreeNode* N0 = GetLeafWithBias(true, true);
    FAdaptiveOctreeNode* N1 = GetLeafWithBias(false, true);
    FAdaptiveOctreeNode* N2 = GetLeafWithBias(false, false);
    FAdaptiveOctreeNode* N3 = GetLeafWithBias(true, false);

    double MaxNodeExtent = Edge.Size * 2.0;
    if (N0 && N0->Extent <= MaxNodeExtent) Result.AddUnique(N0);
    if (N1 && N1->Extent <= MaxNodeExtent) Result.AddUnique(N1);
    if (N2 && N2->Extent <= MaxNodeExtent) Result.AddUnique(N2);
    if (N3 && N3->Extent <= MaxNodeExtent) Result.AddUnique(N3);

    return Result;
}

FAdaptiveOctreeNode* FAdaptiveOctree::GetLeafNodeByPoint(FVector Position)
{
    FAdaptiveOctreeNode* CurrentNode = Root.Get();
    while (CurrentNode)
    {
        if (CurrentNode->IsLeaf())
        {
            return CurrentNode;
        }

        int ChildIndex = 0;
        if (Position.X >= CurrentNode->Center.X) ChildIndex |= 1;
        if (Position.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
        if (Position.Z >= CurrentNode->Center.Z) ChildIndex |= 4;

        FAdaptiveOctreeNode* Child = CurrentNode->Children[ChildIndex].Get();
        if (Child)
        {
            CurrentNode = Child;
        }
        else
        {
            break;
        }
    }

    return nullptr;
}

FAdaptiveOctreeNode* FAdaptiveOctree::GetChunkNodeByPoint(FVector Position)
{
    FAdaptiveOctreeNode* CurrentNode = Root.Get();
    while (CurrentNode)
    {
        if (CurrentNode->Index.Depth == ChunkDepth)
        {
            return CurrentNode;
        }

        int ChildIndex = 0;
        if (Position.X >= CurrentNode->Center.X) ChildIndex |= 1;
        if (Position.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
        if (Position.Z >= CurrentNode->Center.Z) ChildIndex |= 4;

        FAdaptiveOctreeNode* Child = CurrentNode->Children[ChildIndex].Get();
        if (Child)
        {
            CurrentNode = Child;
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
