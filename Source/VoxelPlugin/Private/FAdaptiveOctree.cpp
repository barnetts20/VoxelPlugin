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

    Root = MakeShared<FAdaptiveOctreeNode>( InCenter, InRootExtent, InChunkDepth, InMinDepth, InMaxDepth);
    ComputeNodeData(Root);
    SplitToDepth(Root, InChunkDepth);

    PopulateChunks();
}

void FAdaptiveOctree::SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth)
{
    if (!Node.IsValid()) return;

    if (Node->Index.Depth < InMinDepth)
    {
        SplitAndComputeChildren(Node);// ->Split();
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
        NewChunks[i]->CachedSurfaceMaterial = CachedSurfaceMaterial;
        NewChunks[i]->CachedOceanMaterial = CachedOceanMaterial;
        NewChunks[i]->InitializeData(Chunks[i]->Center, Chunks[i]->Extent);

        TArray<FNodeEdge> Edges;
        GatherLeafEdges(Chunks[i], Edges);
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
            TSharedPtr<FAdaptiveOctreeNode> Neighbor = GetLeafNodeByPoint(NeighborPos);

            if (!Neighbor.IsValid()) continue;
            if (Neighbor->IsSurfaceNode) continue;
            if (ChunkMap.Contains(Neighbor)) continue;

            TSharedPtr<FMeshChunk> NeighborChunk = MakeShared<FMeshChunk>();
            NeighborChunk->CachedParentActor = CachedParentActor;
            NeighborChunk->CachedSurfaceMaterial = CachedSurfaceMaterial;
            NeighborChunk->CachedOceanMaterial = CachedOceanMaterial;
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
    double ReconstructRadius = InEditRadius * 2.5;

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
    Node->ComputeNormalizedPosition(Root->Extent * .9);
}

void FAdaptiveOctree::ReconstructSubtree(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius)
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
    TArray<FIndex3UI> SrfTriangles;
    TArray<FIndex3UI> OcnTriangles;
    
    VertexMap.Reserve(1024);
    UniqueVertices.Reserve(1024);
    SrfTriangles.Reserve(2048);

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(InChunk->ChunkEdges.Num());

    double QuantizationGrid = InChunk->ChunkExtent * 1e-8;
    double OceanRadius = RootExtent * 0.9;

    double OceanTriThreshold = -1000;

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
            FVector PlanetCenter = Root->Center;
            FVector LocalPos(nodesToMesh[i]->DualContourPosition - InChunk->ChunkCenter);
            FVector WorldPos = nodesToMesh[i]->DualContourPosition;
            FVector DirFromCenter = (WorldPos - PlanetCenter).GetSafeNormal();
            FVector NormLocalPos = (nodesToMesh[i]->NormalizedPosition) - InChunk->ChunkCenter;
            double Dist = FVector::Dist(WorldPos, PlanetCenter);

            AllEdgeData[edgeIdx].Vertices[i].Position = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].OriginalPosition = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].NormalizedPosition = NormLocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = nodesToMesh[i]->DualContourNormal;
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

        if (EdgeVertices.Num() >= 3
            && VertexIndices[0] != VertexIndices[1]
            && VertexIndices[1] != VertexIndices[2]
            && VertexIndices[0] != VertexIndices[2])
        {
            FIndex3UI tri0 = (FlipWinding ? FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[1]) : FIndex3UI(VertexIndices[0], VertexIndices[1], VertexIndices[2]));
            SrfTriangles.Add(tri0);

            if (UniqueVertices[tri0.V0].Depth > OceanTriThreshold ||
                UniqueVertices[tri0.V1].Depth > OceanTriThreshold ||
                UniqueVertices[tri0.V2].Depth > OceanTriThreshold) {
                OcnTriangles.Add(tri0);
            }

            FIndex3UI tri1;
            if (EdgeVertices.Num() == 4
                && VertexIndices[0] != VertexIndices[3]
                && VertexIndices[2] != VertexIndices[3])
            {
                tri1 = (FlipWinding ? FIndex3UI(VertexIndices[0], VertexIndices[3], VertexIndices[2]) : FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[3]));
                SrfTriangles.Add(tri1);

                if (UniqueVertices[tri1.V0].Depth > OceanTriThreshold ||
                    UniqueVertices[tri1.V1].Depth > OceanTriThreshold ||
                    UniqueVertices[tri1.V2].Depth > OceanTriThreshold) {
                    OcnTriangles.Add(tri1);
                }
            }
        }
    }

    TSharedPtr<FMeshStreamData> UpdatedSurfaceData = MakeShared<FMeshStreamData>();
    UpdatedSurfaceData->MeshGroupKey = InChunk->SurfaceMeshData->MeshGroupKey;
    UpdatedSurfaceData->MeshSectionKey = InChunk->SurfaceMeshData->MeshSectionKey;

    TSharedPtr<FMeshStreamData> UpdatedOceanData = MakeShared<FMeshStreamData>();
    UpdatedOceanData->MeshGroupKey = InChunk->OceanMeshData->MeshGroupKey;
    UpdatedOceanData->MeshSectionKey = InChunk->OceanMeshData->MeshSectionKey;

    //Surface streams
    auto SrfPositionStream = UpdatedSurfaceData->GetPositionStream();
    auto SrfTangentStream = UpdatedSurfaceData->GetTangentStream();
    auto SrfColorStream = UpdatedSurfaceData->GetColorStream();
    auto SrfTexCoordStream = UpdatedSurfaceData->GetTexCoordStream();
    auto SrfTriangleStream = UpdatedSurfaceData->GetTriangleStream();
    auto SrfPolygroupStream = UpdatedSurfaceData->GetPolygroupStream();

    //Ocean streams
    auto OcnPositionStream = UpdatedOceanData->GetPositionStream();
    auto OcnTangentStream = UpdatedOceanData->GetTangentStream();
    auto OcnColorStream = UpdatedOceanData->GetColorStream();
    auto OcnTexCoordStream = UpdatedOceanData->GetTexCoordStream();
    auto OcnTriangleStream = UpdatedOceanData->GetTriangleStream();
    auto OcnPolygroupStream = UpdatedOceanData->GetPolygroupStream();

    //Init surface count
    SrfPositionStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTangentStream.SetNumUninitialized(UniqueVertices.Num());
    SrfColorStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTriangleStream.SetNumUninitialized(SrfTriangles.Num());
    SrfPolygroupStream.SetNumUninitialized(SrfTriangles.Num());

    //Init ocean count
    OcnPositionStream.SetNumUninitialized(UniqueVertices.Num());
    OcnTangentStream.SetNumUninitialized(UniqueVertices.Num());
    OcnColorStream.SetNumUninitialized(UniqueVertices.Num());
    OcnTexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    OcnTriangleStream.SetNumUninitialized(OcnTriangles.Num());
    OcnPolygroupStream.SetNumUninitialized(OcnTriangles.Num());

    FVector ChunkCenter = InChunk->ChunkCenter;

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
        FMeshVertex& Vertex = UniqueVertices[VertIdx];
        FVector WorldPos = FVector(Vertex.OriginalPosition) + ChunkCenter;

        SrfPositionStream.Set(VertIdx, Vertex.OriginalPosition);
        FRealtimeMeshTangentsHighPrecision Tangent;
        Tangent.SetNormal(FVector3f(Vertex.Normal));
        SrfTangentStream.Set(VertIdx, Tangent);
        SrfColorStream.Set(VertIdx, Vertex.Color);
        SrfTexCoordStream.Set(VertIdx, ComputeTriplanarUV(WorldPos, Vertex.Normal));

        // Ocean — same index, different position/normal
        OcnPositionStream.Set(VertIdx, Vertex.NormalizedPosition);
        FRealtimeMeshTangentsHighPrecision OcnTangent;
        FVector OcnNormal = (FVector(Vertex.NormalizedPosition) + ChunkCenter - Root->Center).GetSafeNormal();
        OcnTangent.SetNormal(FVector3f(OcnNormal));
        OcnTangentStream.Set(VertIdx, OcnTangent);

        // Encode
        float absDepth = FMath::Max(Vertex.Depth, 0.0f);
        bool isUnderwater = (Vertex.Depth >= 0);
        uint32 intDepth = (uint32)FMath::Min(absDepth * 1000.0f, 4294967295.0f); // mm precision, full uint32 range

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
    InChunk->OceanMeshData = UpdatedOceanData;

    InChunk->IsDirty = (SrfTriangles.Num() > 0);
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
            SplitAndComputeChildren(Node);
            for (int i = 0; i < 8; i++)
                AppendUniqueEdges(Node->Children[i]->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
            return;
        }
        else if (Node->Parent.IsValid() &&
            Node->Parent.Pin()->ShouldMerge(CameraPosition, InScreenSpaceThreshold, InCameraFOV) &&
            Node->Index.LastChild() == 7)
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

    // --- SHORT CIRCUIT ---
    // If this entire subtree is too far away to split or too close to merge,
    // just collect its leaf edges without recursing into children
    double Distance = FMath::Max(FVector::Dist(Node->Center, CameraPosition), 1.0);
    double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));

    // Largest node in subtree is this node — if it wouldn't split, nothing deeper will split
    double AngularSizeThisNode = (2.0 * Node->Extent / Distance) * FOVScale;

    // Smallest node in subtree is at MaxDepth — if it wouldn't merge, nothing deeper will merge  
    double SmallestExtent = Node->Extent / FMath::Pow(2.0, (double)(Node->DepthBounds[1] - Node->Index.Depth));
    double AngularSizeSmallest = (2.0 * SmallestExtent / Distance) * FOVScale;

    bool SubtreeCouldSplit = AngularSizeThisNode > InScreenSpaceThreshold;
    bool SubtreeCouldMerge = AngularSizeSmallest < InScreenSpaceThreshold * 0.5;

    if (!SubtreeCouldSplit && !SubtreeCouldMerge)
    {
        return;
    }

    // Subtree could change — recurse normally
    for (int i = 0; i < 8; i++)
        if (Node->Children[i])
            UpdateLodRecursive(Node->Children[i], CameraPosition, InScreenSpaceThreshold, InCameraFOV, OutNodeEdges, EdgeMap, OutChanged);
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    ChunkMap.GenerateKeyArray(Chunks);
    int32 NumChunks = Chunks.Num();

    TArray<TSharedPtr<FMeshChunk>> ModifiedChunks;
    ModifiedChunks.SetNumZeroed(NumChunks);

    double t0 = FPlatformTime::Seconds();

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

    double t1 = FPlatformTime::Seconds();

    int32 NumModified = 0;
    ParallelFor(NumChunks, [&](int32 i)
        {
            if (ModifiedChunks[i].IsValid()) {
                UpdateMeshChunkStreamData(ModifiedChunks[i]);
                FPlatformAtomics::InterlockedIncrement(&NumModified);
            }
        });

    double t2 = FPlatformTime::Seconds();

    if ((t2 - t0) * 1000.0 > 100.0) // only log if anything interesting happened
    {
        UE_LOG(LogTemp, Log, TEXT("[LOD] LODPass+EdgeBuild: %.2fms | StreamBuild: %.2fms | ModifiedChunks: %d / %d"),
            (t1 - t0) * 1000.0, (t2 - t1) * 1000.0, NumModified, NumChunks);
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

void FAdaptiveOctree::SplitAndComputeChildren(TSharedPtr<FAdaptiveOctreeNode> Node)
{
    // Verified lookup table: ChildCornerSources[childIdx][cornerIdx] = G index (0-26)
    static const int32 ChildCornerSources[8][8] = {
        {  0,  8, 12, 24, 16, 22, 20, 26 }, // child 0: octant (-1,-1,-1)
        {  8,  1, 24, 13, 22, 17, 26, 21 }, // child 1: octant (+1,-1,-1)
        { 12, 24,  2,  9, 20, 26, 18, 23 }, // child 2: octant (-1,+1,-1)
        { 24, 13,  9,  3, 26, 21, 23, 19 }, // child 3: octant (+1,+1,-1)
        { 16, 22, 20, 26,  4, 10, 14, 25 }, // child 4: octant (-1,-1,+1)
        { 22, 17, 26, 21, 10,  5, 25, 15 }, // child 5: octant (+1,-1,+1)
        { 20, 26, 18, 23, 14, 25,  6, 11 }, // child 6: octant (-1,+1,+1)
        { 26, 21, 23, 19, 25, 15, 11,  7 }, // child 7: octant (+1,+1,+1)
    };

    Node->Split();

    // Build the 27-point grid positions
    FVector GridPositions[27];
    double  GridDensities[27];

    // G0-G7: parent corners, positions and densities already known
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
    // Faces defined by which axis is fixed and which sign
    // -X(0): corners 0,2,4,6   +X(1): corners 1,3,5,7
    // -Y(2): corners 0,1,4,5   +Y(3): corners 2,3,6,7
    // -Z(4): corners 0,1,2,3   +Z(5): corners 4,5,6,7
    static const int32 FaceCorners[6][4] = {
        { 0, 2, 4, 6 }, // G20: -X face
        { 1, 3, 5, 7 }, // G21: +X face
        { 0, 1, 4, 5 }, // G22: -Y face
        { 2, 3, 6, 7 }, // G23: +Y face
        { 0, 1, 2, 3 }, // G24: -Z face
        { 4, 5, 6, 7 }, // G25: +Z face
    };
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

    // Sample only the 19 new points (G8-G26)
    const int32 NewCount = 19;
    TArray<float> XPos, YPos, ZPos, NoiseOut;
    XPos.SetNumUninitialized(NewCount);
    YPos.SetNumUninitialized(NewCount);
    ZPos.SetNumUninitialized(NewCount);
    NoiseOut.SetNumUninitialized(NewCount);

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

    DensityFunction(NewCount, XPos.GetData(), YPos.GetData(), ZPos.GetData(), NoiseOut.GetData());

    for (int32 i = 0; i < NewCount; i++)
    {
        double Height = (double)NoiseOut[i] * NoiseScale;
        FVector PlanetRel = GridPositions[8 + i] - PlanetCenter;
        double Dist = PlanetRel.Size();
        GridDensities[8 + i] = Dist - (RootExtent * 0.9 + Height) + EditStore->Sample(GridPositions[8 + i]);
    }

    // Assign corners to children from the grid, then finalize
    for (int ci = 0; ci < 8; ci++)
    {
        for (int k = 0; k < 8; k++)
        {
            int32 gi = ChildCornerSources[ci][k];
            Node->Children[ci]->Corners[k].Position = GridPositions[gi];
            Node->Children[ci]->Corners[k].Density = GridDensities[gi];
        }
        Node->Children[ci]->FinalizeFromExistingCorners();
        Node->Children[ci]->ComputeNormalizedPosition(Root->Extent * 0.9);
    }
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
    Node->ComputeNormalizedPosition(Root->Extent * .9);
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Nodes;

    // Calculate the exact mathematical midpoint of the edge
    FVector Origin = Edge.Corners[0].Position;
    FVector End = Edge.Corners[1].Position;
    FVector Midpoint = (Origin + End) * 0.5;
    
    double Epsilon = ChunkExtent * 1e-8;

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
        if (CurrentNode->Index.Depth == ChunkDepth)
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