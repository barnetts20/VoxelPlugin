#include "FAdaptiveOctree.h"
#include <FOctreeConstants.h>

FAdaptiveOctree::FAdaptiveOctree(const FOctreeParams& Params)
{
    DensityFunction = Params.NoiseFunction;
    EditStore = Params.EditStore;
    CachedParentActor = Params.ParentActor;
    CachedMeshAttachRoot = Params.MeshAttachmentRoot;
    CachedSurfaceMaterial = Params.SurfaceMaterial;
    CachedOceanMaterial = Params.OceanMaterial;
    ChunkDepth = Params.ChunkDepth;

    // Core terrain parameters from params
    PlanetRadius = Params.PlanetRadius;
    NoiseAmplitude = Params.NoiseAmplitude;

    // Derive root extent: must contain all possible surface points
    RootExtent = (PlanetRadius + NoiseAmplitude) * Params.RootExtentBuffer;
    ChunkExtent = RootExtent / FMath::Pow(2.0, (double)Params.ChunkDepth);

    // Configure edge key quantization for this tree's scale.
    // Maps the smallest possible corner spacing to ~1 grid unit.
    FEdgeKey::InvGridSize = FMath::Pow(2.0, (double)Params.MaxDepth) / RootExtent;

    bEnableOcean = Params.bEnableOcean;

    // Ocean radius derived from sea level coefficient.
    OceanRadius = PlanetRadius + (Params.SeaLevelCoefficient * NoiseAmplitude);

    // UV scale: 0.0001 was calibrated for PlanetRadius=80M. Keep the same visual tiling.
    TriplanarUVScale = 0.0001 * (80000000.0 / FMath::Max(PlanetRadius, 1.0));

    Root = MakeShared<FAdaptiveOctreeNode>(Params.Center, RootExtent, Params.ChunkDepth, Params.MinDepth, Params.MaxDepth);
    ComputeNodeData(Root.Get());
    SplitToDepth(Root.Get(), Params.ChunkDepth);

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
            FVector NeighborPos = ChunkNode->Center + OctreeConstants::Directions[i] * Offset;
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
        NewChunks[i]->CachedMeshAttachRoot = CachedMeshAttachRoot;
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
        NewChunk->CachedMeshAttachRoot = CachedMeshAttachRoot;
        NewChunk->CachedSurfaceMaterial = CachedSurfaceMaterial;
        NewChunk->CachedOceanMaterial = CachedOceanMaterial;
        NewChunk->InitializeData(ChunkNode->Center, ChunkNode->Extent);
        ChunkMap.Add(ChunkNode, NewChunk);
        OutDirtyChunks.Add({ ChunkNode, NewChunk });

        // Buffer neighbors
        const double Offset = ChunkNode->Extent * 2.0;
        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + OctreeConstants::Directions[i] * Offset;
            FAdaptiveOctreeNode* NeighborRaw = GetLeafNodeByPoint(NeighborPos);

            if (!NeighborRaw) continue;
            if (NeighborRaw->IsSurfaceNode) continue;

            // Only convert to TSharedPtr at the ChunkMap boundary
            TSharedPtr<FAdaptiveOctreeNode> Neighbor = NeighborRaw->AsShared();
            if (ChunkMap.Contains(Neighbor)) continue;

            TSharedPtr<FMeshChunk> NeighborChunk = MakeShared<FMeshChunk>();
            NeighborChunk->CachedParentActor = CachedParentActor;
            NeighborChunk->CachedMeshAttachRoot = CachedMeshAttachRoot;
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
    Node->FinalizeFromExistingCorners(Root->Center, OceanRadius, bEnableOcean);
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

    FVector PlanetCenter = Root->Center;

    ParallelFor(Count, [&](int32 i)
        {
            FVector PlanetRel = Samples[i].Position - PlanetCenter;
            Samples[i].Dist = PlanetRel.Size();
            ComputeNoisePosition(Samples[i].Position, XPos[i], YPos[i], ZPos[i]);
        });

    // 3. One bulk noise call
    DensityFunction(Count, XPos.GetData(), YPos.GetData(), ZPos.GetData(), NoiseOut.GetData());

    // 4. SDF + edits
    ParallelFor(Count, [&](int32 i)
        {
            Samples[i].Density = ComputeDensity(Samples[i].Dist, NoiseOut[i], Samples[i].Position);
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

FVector2f FAdaptiveOctree::ComputeTriplanarUV(FVector Position, FVector Normal) const
{
    FVector2f UV;
    FVector AbsNormal = Normal.GetAbs();
    float Scale = (float)TriplanarUVScale;

    if (AbsNormal.X > AbsNormal.Y && AbsNormal.X > AbsNormal.Z)
    {
        UV = FVector2f(Position.Y, Position.Z) * Scale;
    }
    else if (AbsNormal.Y > AbsNormal.X && AbsNormal.Y > AbsNormal.Z)
    {
        UV = FVector2f(Position.X, Position.Z) * Scale;
    }
    else
    {
        UV = FVector2f(Position.X, Position.Y) * Scale;
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

    double OceanTriThreshold = -NoiseAmplitude;

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
        for (int i = 0; i < neighbors.Count; i++) {
            FAdaptiveOctreeNode* NodePtr = neighbors.Nodes[i];
            FVector LocalPos(NodePtr->DualContourPosition - InChunk->ChunkCenter);

            AllEdgeData[edgeIdx].Vertices[i].Position = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = FVector(NodePtr->DualContourNormal);
            AllEdgeData[edgeIdx].Vertices[i].Color = FColor::Green;

            if (bEnableOcean)
            {
                FVector OceanLocalPos = NodePtr->OceanPosition - InChunk->ChunkCenter;
                double Dist = FVector::Dist(NodePtr->DualContourPosition, PlanetCenter);
                AllEdgeData[edgeIdx].Vertices[i].NormalizedPosition = OceanLocalPos;
                AllEdgeData[edgeIdx].Vertices[i].Depth = (float)(OceanRadius - Dist);
            }
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

            if (bEnableOcean &&
                (UniqueVertices[tri0.V0].Depth > OceanTriThreshold ||
                UniqueVertices[tri0.V1].Depth > OceanTriThreshold ||
                UniqueVertices[tri0.V2].Depth > OceanTriThreshold))
                OcnTriangles.Add(tri0);

            if (NumEdgeVerts == 4
                && VertexIndices[0] != VertexIndices[3]
                && VertexIndices[2] != VertexIndices[3])
            {
                FIndex3UI tri1 = FlipWinding
                    ? FIndex3UI(VertexIndices[0], VertexIndices[3], VertexIndices[2])
                    : FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[3]);
                SrfTriangles.Add(tri1);

                if (bEnableOcean &&
                    (UniqueVertices[tri1.V0].Depth > OceanTriThreshold ||
                    UniqueVertices[tri1.V1].Depth > OceanTriThreshold ||
                    UniqueVertices[tri1.V2].Depth > OceanTriThreshold))
                    OcnTriangles.Add(tri1);
            }
        }
    }

    TSharedPtr<FMeshStreamData> UpdatedSurfaceData = MakeShared<FMeshStreamData>();
    UpdatedSurfaceData->MeshGroupKey = InChunk->SurfaceMeshData->MeshGroupKey;
    UpdatedSurfaceData->MeshSectionKey = InChunk->SurfaceMeshData->MeshSectionKey;

    auto SrfPositionStream = UpdatedSurfaceData->GetPositionStream();
    auto SrfTangentStream = UpdatedSurfaceData->GetTangentStream();
    auto SrfColorStream = UpdatedSurfaceData->GetColorStream();
    auto SrfTexCoordStream = UpdatedSurfaceData->GetTexCoordStream();
    auto SrfTriangleStream = UpdatedSurfaceData->GetTriangleStream();
    auto SrfPolygroupStream = UpdatedSurfaceData->GetPolygroupStream();

    SrfPositionStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTangentStream.SetNumUninitialized(UniqueVertices.Num());
    SrfColorStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTriangleStream.SetNumUninitialized(SrfTriangles.Num());
    SrfPolygroupStream.SetNumUninitialized(SrfTriangles.Num());

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

    ParallelFor(SrfTriangles.Num(), [&](int32 TriIdx) {
        SrfTriangleStream.Set(TriIdx, SrfTriangles[TriIdx]);
        SrfPolygroupStream.Set(TriIdx, 0);
        });

    InChunk->SurfaceMeshData = UpdatedSurfaceData;

    // Ocean mesh pipeline — only when ocean is enabled
    if (bEnableOcean)
    {
        TSharedPtr<FMeshStreamData> UpdatedOceanData = MakeShared<FMeshStreamData>();
        UpdatedOceanData->MeshGroupKey = InChunk->OceanMeshData->MeshGroupKey;
        UpdatedOceanData->MeshSectionKey = InChunk->OceanMeshData->MeshSectionKey;

        auto OcnPositionStream = UpdatedOceanData->GetPositionStream();
        auto OcnTangentStream = UpdatedOceanData->GetTangentStream();
        auto OcnColorStream = UpdatedOceanData->GetColorStream();
        auto OcnTexCoordStream = UpdatedOceanData->GetTexCoordStream();
        auto OcnTriangleStream = UpdatedOceanData->GetTriangleStream();
        auto OcnPolygroupStream = UpdatedOceanData->GetPolygroupStream();

        OcnPositionStream.SetNumUninitialized(UniqueVertices.Num());
        OcnTangentStream.SetNumUninitialized(UniqueVertices.Num());
        OcnColorStream.SetNumUninitialized(UniqueVertices.Num());
        OcnTexCoordStream.SetNumUninitialized(UniqueVertices.Num());
        OcnTriangleStream.SetNumUninitialized(OcnTriangles.Num());
        OcnPolygroupStream.SetNumUninitialized(OcnTriangles.Num());

        ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
            FMeshVertex& Vertex = UniqueVertices[VertIdx];

            OcnPositionStream.Set(VertIdx, Vertex.NormalizedPosition);
            FRealtimeMeshTangentsHighPrecision OcnTangent;
            FVector OcnNormal = (FVector(Vertex.NormalizedPosition) + ChunkCenter - Root->Center).GetSafeNormal();
            OcnTangent.SetNormal(FVector3f(OcnNormal));
            OcnTangentStream.Set(VertIdx, OcnTangent);

            float absDepth = FMath::Max(Vertex.Depth, 0.0f);
            float DepthEncodeScale = (float)(1000.0 * (80000000.0 / FMath::Max(PlanetRadius, 1.0)));
            uint32 intDepth = (uint32)FMath::Min(absDepth * DepthEncodeScale, 4294967295.0f);
            uint8 R = (intDepth >> 24) & 0xFF;
            uint8 G = (intDepth >> 16) & 0xFF;
            uint8 B = (intDepth >> 8) & 0xFF;
            uint8 A = intDepth & 0xFF;
            OcnColorStream.Set(VertIdx, FColor(R, G, B, A));
            OcnTexCoordStream.Set(VertIdx, ComputeTriplanarUV(FVector(Vertex.NormalizedPosition) + ChunkCenter, OcnNormal));
            });

        ParallelFor(OcnTriangles.Num(), [&](int32 TriIdx) {
            OcnTriangleStream.Set(TriIdx, OcnTriangles[TriIdx]);
            OcnPolygroupStream.Set(TriIdx, 0);
            });

        InChunk->OceanMeshData = UpdatedOceanData;
    }

    InChunk->IsDirty = true;
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

void FAdaptiveOctree::UpdateLodRecursive(FAdaptiveOctreeNode* Node, FVector CameraPosition, double ThresholdSq, double MergeThresholdSq, double InFOVScale, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged)
{
    if (Node->IsLeaf())
    {
        if (Node->ShouldSplit(Root->Center, CameraPosition, ThresholdSq, InFOVScale))
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
            if (ParentPtr && ParentPtr->ShouldMerge(Root->Center, CameraPosition, MergeThresholdSq, InFOVScale))
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
            UpdateLodRecursive(Node->Children[i].Get(), CameraPosition, ThresholdSq, MergeThresholdSq, InFOVScale, OutNodeEdges, EdgeMap, OutChanged);
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
    double ThresholdSq = InScreenSpaceThreshold * InScreenSpaceThreshold;
    double MergeThresholdSq = (InScreenSpaceThreshold * 0.5) * (InScreenSpaceThreshold * 0.5);

    double t0 = FPlatformTime::Seconds();
    ParallelFor(NumChunks, [&](int32 idx) {
        FAdaptiveOctreeNode* ChunkNode = Chunks[idx].Get();
        double DistSq = FMath::Max(FVector::DistSquared(ChunkNode->Center, CameraPosition), 1e-12);
        bool CouldSplit = FAdaptiveOctreeNode::EvaluateSplit(ChunkNode->Extent, DistSq, FOVScale, ThresholdSq, ChunkNode->Index.Depth, ChunkNode->DepthBounds[1], ChunkNode->DepthBounds[2]);
        double SmallestExtent = ChunkNode->Extent / (double)(1 << (ChunkNode->DepthBounds[1] - ChunkNode->Index.Depth));
        bool CouldMerge = FAdaptiveOctreeNode::EvaluateMerge(SmallestExtent, DistSq, FOVScale, MergeThresholdSq, ChunkNode->DepthBounds[1], ChunkNode->DepthBounds[0], ChunkNode->DepthBounds[2]);

        if (!CouldSplit && !CouldMerge) return; //Early out

        TArray<FNodeEdge> tChunkEdges;
        TMap<FEdgeKey, int32> tEdgeMap;
        bool tChanged = false;
        UpdateLodRecursive(ChunkNode, CameraPosition, ThresholdSq, MergeThresholdSq, FOVScale, tChunkEdges, tEdgeMap, tChanged);

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
            Node->Corners[OctreeConstants::FaceCorners[i][0]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][1]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][2]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][3]].Position) * 0.25;
    }

    // G26: body center
    GridPositions[26] = Node->Center;

    // --- Stage 2: Sample 19 new densities ---
    const int32 NewCount = 19;
    float XPos[19], YPos[19], ZPos[19], NoiseOut[19];

    FVector PlanetCenter = Root->Center;
    double GridDists[19];

    for (int32 i = 0; i < NewCount; i++)
    {
        FVector PlanetRel = GridPositions[8 + i] - PlanetCenter;
        GridDists[i] = PlanetRel.Size();
        ComputeNoisePosition(GridPositions[8 + i], XPos[i], YPos[i], ZPos[i]);
    }

    DensityFunction(NewCount, XPos, YPos, ZPos, NoiseOut);

    for (int32 i = 0; i < NewCount; i++)
    {
        GridDensities[8 + i] = ComputeDensity(GridDists[i], NoiseOut[i], GridPositions[8 + i]);
    }

    // --- Stage 3: Compute 27 normals from grid gradients ---
    FVector GridNormals[27];

    for (int32 gi = 0; gi < 27; gi++)
    {
        int8 cx = OctreeConstants::GridCoords[gi][0];
        int8 cy = OctreeConstants::GridCoords[gi][1];
        int8 cz = OctreeConstants::GridCoords[gi][2];

        double dX;
        if (cx <= 0)
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 2][cy + 1][cz + 1];
            dX = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx][cy + 1][cz + 1];
            dX = -(GridDensities[ngi] - GridDensities[gi]);
        }

        double dY;
        if (cy <= 0)
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy + 2][cz + 1];
            dY = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy][cz + 1];
            dY = -(GridDensities[ngi] - GridDensities[gi]);
        }

        double dZ;
        if (cz <= 0)
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy + 1][cz + 2];
            dZ = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy + 1][cz];
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
            int32 gi = OctreeConstants::ChildCornerSources[ci][k];
            Node->Children[ci]->Corners[k].Position = GridPositions[gi];
            Node->Children[ci]->Corners[k].Density = (float)GridDensities[gi];
            Node->Children[ci]->Corners[k].Normal = FVector3f(GridNormals[gi]);
        }
        Node->Children[ci]->FinalizeFromExistingCorners(Root->Center, OceanRadius, bEnableOcean, true); // normals already computed from grid
    }
}

void FAdaptiveOctree::ComputeNodeData(FAdaptiveOctreeNode* Node)
{
    float xPos[8], yPos[8], zPos[8], noiseOut[8];
    double dists[8];

    FVector PlanetCenter = Root->Center;

    for (int i = 0; i < 8; i++)
    {
        FVector PlanetRel = Node->Corners[i].Position - PlanetCenter;
        dists[i] = PlanetRel.Size();
        ComputeNoisePosition(Node->Corners[i].Position, xPos[i], yPos[i], zPos[i]);
    }

    DensityFunction(8, xPos, yPos, zPos, noiseOut);

    for (int i = 0; i < 8; i++)
    {
        Node->Corners[i].Density = ComputeDensity(dists[i], noiseOut[i], Node->Corners[i].Position);
    }

    Node->FinalizeFromExistingCorners(Root->Center, OceanRadius, bEnableOcean);
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
    ChunkMap.Empty();
    Root.Reset();
    MeshChunksInitialized = false;
}