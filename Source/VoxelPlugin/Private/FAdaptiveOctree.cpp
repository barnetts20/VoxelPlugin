#include "FAdaptiveOctree.h"
#include "FOctreeConstants.h"

FAdaptiveOctree::FAdaptiveOctree(const FOctreeParams& Params)
{
    CachedParentActor = Params.ParentActor;
    CachedMeshAttachRoot = Params.MeshAttachmentRoot;
    CachedSurfaceMaterial = Params.SurfaceMaterial;
    Compositor = Params.Compositor;
    ChunkDepth = Params.ChunkDepth;
    MaxChunkDepth = Params.MaxChunkDepth;
    ChunkPrecisionThreshold = Params.ChunkPrecisionThreshold;
    ChunkDemoteHysteresis = Params.ChunkDemoteHysteresis;
    MaxChunkDepthDelta = FMath::Max(1, Params.MaxChunkDepthDelta);
    PrecisionDepthFloor = Params.PrecisionDepthFloor;

    // Core terrain parameters from params
    PlanetRadius = Params.PlanetRadius;
    NoiseAmplitude = Params.NoiseAmplitude;

    // Derive root extent: must contain all possible surface points
    RootExtent = (PlanetRadius + NoiseAmplitude) * Params.RootExtentBuffer;
    ChunkExtent = RootExtent / FMath::Pow(2.0, (double)Params.ChunkDepth);

    // Configure edge key quantization for this tree's scale.
    // Maps the smallest possible corner spacing to ~1 grid unit.
    FEdgeKey::InvGridSize = FMath::Pow(2.0, (double)Params.MaxDepth) / RootExtent;

    // UV scale: 0.0001 was calibrated for PlanetRadius=80M. Keep the same visual tiling.
    TriplanarUVScale = 0.0001 * (80000000.0 / FMath::Max(PlanetRadius, 1.0));

    Root = MakeShared<FAdaptiveOctreeNode>(RootExtent, Params.ChunkDepth, Params.MinDepth, Params.MaxDepth);
    ComputeNodeData(Root.Get());
    SplitToDepth(Root.Get(), Params.ChunkDepth);

    PopulateChunks();
}

void FAdaptiveOctree::SplitToDepth(FAdaptiveOctreeNode* Node, int InMinDepth)
{
    if (!Node || Node->Index.Depth >= InMinDepth) return;

    SplitAndComputeChildren(Node);
    for (int i = 0; i < 8; i++)
    {
        if (Node->Children[i] && Node->Children[i]->CouldContainSurface)
            SplitToDepth(Node->Children[i].Get(), InMinDepth);
    }
}

void FAdaptiveOctree::CollectNearSurfaceChunks(FAdaptiveOctreeNode* Node, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out)
{
    if (!Node) return;
    if (!Node->CouldContainSurface) return;

    if (Node->bIsChunkRoot)
    {
        Out.Add(Node->AsShared());
        return;
    }

    if (Node->IsLeaf()) return;

    for (int i = 0; i < 8; i++)
        CollectNearSurfaceChunks(Node->Children[i].Get(), Out);
}

void FAdaptiveOctree::PopulateChunks()
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    CollectNearSurfaceChunks(Root.Get(), Chunks);

    // Note: densities, normals, edges, QEF, and surface flags are already computed
    // by SplitToDepth -> SplitAndComputeChildren -> FinalizeFromExistingCorners.
    // No reconstruction pass needed for initial build.

    // Build mesh chunks -- parallel edge gather + stream build
    TArray<TSharedPtr<FMeshChunk>> NewChunks;
    NewChunks.SetNum(Chunks.Num());

    ParallelFor(Chunks.Num(), [&](int32 i) {
        NewChunks[i] = MakeShared<FMeshChunk>();
        NewChunks[i]->CachedParentActor = CachedParentActor;
        NewChunks[i]->CachedMeshAttachRoot = CachedMeshAttachRoot;
        NewChunks[i]->CachedSurfaceMaterial = CachedSurfaceMaterial;
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
        if (!ChunkNode->CouldContainSurface)
        {
            (*Found)->SurfaceMeshData->ResetStreams();
            (*Found)->IsDirty = true;
        }
        else
        {
            OutDirtyChunks.Add({ ChunkNode, *Found });
        }
    }
    else if (ChunkNode->CouldContainSurface)
    {
        TSharedPtr<FMeshChunk> NewChunk = MakeShared<FMeshChunk>();
        NewChunk->CachedParentActor = CachedParentActor;
        NewChunk->CachedMeshAttachRoot = CachedMeshAttachRoot;
        NewChunk->CachedSurfaceMaterial = CachedSurfaceMaterial;
        NewChunk->InitializeData(ChunkNode->Center, ChunkNode->Extent);
        ChunkMap.Add(ChunkNode, NewChunk);
        OutDirtyChunks.Add({ ChunkNode, NewChunk });
    }
}

void FAdaptiveOctree::ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    int depth = Compositor->GetEditStore()->GetDepthForBrushRadius(InEditRadius, InEditResolution);
    TArray<FVector> AffectedChunkCenters = Compositor->GetEditStore()->ApplySphericalEdit(InEditCenter, InEditRadius, InEditStrength, depth);

    // Resolve chunk nodes -- collect as shared ptrs since we need them for ChunkMap operations
    TArray<TSharedPtr<FAdaptiveOctreeNode>> ChunkNodes;
    for (const FVector& Center : AffectedChunkCenters)
    {
        FAdaptiveOctreeNode* ChunkNodeRaw = GetChunkNodeByPoint(Center);
        if (ChunkNodeRaw)
            ChunkNodes.Add(ChunkNodeRaw->AsShared());
    }

    // Propagate bHasEditedDescendants through the main tree using the brush
    // sphere as the spatial filter. Only nodes whose AABB overlaps the brush
    // get flagged. This replaces the per-node HasEditsAlongPath queries in
    // the reconstruction pipeline with a single up-front pass.
    double BrushRadiusSq = InEditRadius * InEditRadius;
    TFunction<void(FAdaptiveOctreeNode*)> FlagEditedNodes = [&](FAdaptiveOctreeNode* N)
        {
            if (!N) return;

            // Sphere-AABB overlap test
            FVector Closest(
                FMath::Clamp(InEditCenter.X, N->Center.X - N->Extent, N->Center.X + N->Extent),
                FMath::Clamp(InEditCenter.Y, N->Center.Y - N->Extent, N->Center.Y + N->Extent),
                FMath::Clamp(InEditCenter.Z, N->Center.Z - N->Extent, N->Center.Z + N->Extent));
            if (FVector::DistSquared(Closest, InEditCenter) > BrushRadiusSq)
                return;

            N->bHasEditedDescendants = true;

            if (!N->IsLeaf())
            {
                for (int i = 0; i < 8; i++)
                    FlagEditedNodes(N->Children[i].Get());
            }
        };

    for (auto& ChunkNode : ChunkNodes)
        FlagEditedNodes(ChunkNode.Get());

    // Also flag ancestors up to root so EnforceSplitsInSubtree can enter.
    for (auto& ChunkNode : ChunkNodes)
    {
        FAdaptiveOctreeNode* Current = ChunkNode->Parent.IsValid() ? ChunkNode->Parent.Pin().Get() : nullptr;
        while (Current && !Current->bHasEditedDescendants)
        {
            Current->bHasEditedDescendants = true;
            Current = Current->Parent.IsValid() ? Current->Parent.Pin().Get() : nullptr;
        }
    }

    // Stage 1: Parallel -- recompute all node data (densities, normals, surface flags)
    ParallelFor(ChunkNodes.Num(), [&](int32 i) {
        ReconstructSubtree(ChunkNodes[i].Get());
        });

    // Stage 2: Enforce splits -- edits may create surface in previously empty nodes.
    // Must run after ReconstructSubtree (densities updated) but before UpdateChunkMap
    // so the chunk map sees the complete tree state including new nodes.
    for (auto& ChunkNode : ChunkNodes)
    {
        EnforceSplitsInSubtree(ChunkNode.Get());
    }

    // Stage 3: Serial -- update chunk map (picks up newly surfaced chunks)
    TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>> DirtyChunks;
    for (auto& ChunkNode : ChunkNodes)
    {
        UpdateChunkMap(ChunkNode, DirtyChunks);
    }

    // Stage 4: Parallel -- gather edges + rebuild mesh streams
    ParallelFor(DirtyChunks.Num(), [&](int32 i) {
        TArray<FNodeEdge> NewEdges;
        TMap<FEdgeKey, int32> EdgeMap;
        GatherLeafEdges(DirtyChunks[i].Key.Get(), NewEdges, EdgeMap);
        DirtyChunks[i].Value->ChunkEdges = NewEdges;
        UpdateMeshChunkStreamData(DirtyChunks[i].Value);
        });
}

void FAdaptiveOctree::GatherUniqueCorners(FAdaptiveOctreeNode* Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& CornerMap, double QuantizeGrid)
{
    if (!Node) return;

    if (!Node->bHasEditedDescendants)
        return;

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

    if (Node->IsLeaf() || Node->Index.Depth >= PrecisionDepthFloor) return;

    for (int i = 0; i < 8; i++)
        GatherUniqueCorners(Node->Children[i].Get(), Samples, CornerMap, QuantizeGrid);
}

void FAdaptiveOctree::FinalizeSubtree(FAdaptiveOctreeNode* Node)
{
    if (!Node) return;

    if (!Node->bHasEditedDescendants)
        return;

    // Children first
    if (!Node->IsLeaf())
    {
        for (int i = 0; i < 8; i++)
            FinalizeSubtree(Node->Children[i].Get());
    }

    // Then this node
    Node->FinalizeFromExistingCorners();
}

void FAdaptiveOctree::PropagateDeepDensities(FAdaptiveOctreeNode* Node)
{
    if (!Node || Node->IsLeaf()) return;

    if (!Node->bHasEditedDescendants)
        return;

    if (Node->Index.Depth >= PrecisionDepthFloor)
    {
        // This node is at or past the floor � interpolate its children's densities
        // from this node's corners, then apply edit-store deltas on top.
        InterpolateChildrenWithEdits(Node);

        // Recurse into children that have their own children
        for (int i = 0; i < 8; i++)
            if (Node->Children[i].IsValid())
                PropagateDeepDensities(Node->Children[i].Get());
    }
    else
    {
        // Haven't reached the floor yet � keep walking down
        for (int i = 0; i < 8; i++)
            PropagateDeepDensities(Node->Children[i].Get());
    }
}

void FAdaptiveOctree::InterpolateChildrenWithEdits(FAdaptiveOctreeNode* Node)
{
    if (!Node || Node->IsLeaf()) return;

    FVector GridPositions[27];
    double  GridDensities[27];

    // G0-G7: parent corners � strip edit contribution to get noise-only baseline
    float ParentSX[8], ParentSY[8], ParentSZ[8], ParentEdits[8];
    for (int i = 0; i < 8; i++)
    {
        GridPositions[i] = Node->Corners[i].Position;
        ParentSX[i] = (float)GridPositions[i].X;
        ParentSY[i] = (float)GridPositions[i].Y;
        ParentSZ[i] = (float)GridPositions[i].Z;
        ParentEdits[i] = 0.f;
    }

    if (Compositor.IsValid())
    {
        // Get edit-only values at parent positions so we can subtract them
        FSampleInput ParentInput(ParentSX, ParentSY, ParentSZ, 8);
        Compositor->SampleEditsOnly(ParentInput, ParentEdits);
    }

    // Noise-only baseline for parent corners
    for (int i = 0; i < 8; i++)
        GridDensities[i] = Node->Corners[i].Density - ParentEdits[i];

    // Interpolate noise-only baseline to midpoints
    for (int i = 0; i < 12; i++)
    {
        int a = OctreeConstants::EdgePairs[i][0];
        int b = OctreeConstants::EdgePairs[i][1];
        GridPositions[8 + i] = (Node->Corners[a].Position + Node->Corners[b].Position) * 0.5;
        GridDensities[8 + i] = (GridDensities[a] + GridDensities[b]) * 0.5;
    }

    for (int i = 0; i < 6; i++)
    {
        GridPositions[20 + i] = (
            Node->Corners[OctreeConstants::FaceCorners[i][0]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][1]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][2]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][3]].Position) * 0.25;
        GridDensities[20 + i] = (
            GridDensities[OctreeConstants::FaceCorners[i][0]] +
            GridDensities[OctreeConstants::FaceCorners[i][1]] +
            GridDensities[OctreeConstants::FaceCorners[i][2]] +
            GridDensities[OctreeConstants::FaceCorners[i][3]]) * 0.25;
    }

    GridPositions[26] = Node->Center;
    GridDensities[26] = 0.0;
    for (int i = 0; i < 8; i++)
        GridDensities[26] += GridDensities[i];
    GridDensities[26] *= 0.125;

    // Sample edit store at full resolution for all 27 positions and add back
    if (Compositor.IsValid())
    {
        float SX[27], SY[27], SZ[27], Edits[27];
        for (int32 i = 0; i < 27; i++)
        {
            SX[i] = (float)GridPositions[i].X;
            SY[i] = (float)GridPositions[i].Y;
            SZ[i] = (float)GridPositions[i].Z;
            Edits[i] = 0.f;
        }
        FSampleInput AllInput(SX, SY, SZ, 27);
        Compositor->SampleEditsOnly(AllInput, Edits);
        for (int32 i = 0; i < 27; i++)
            GridDensities[i] += Edits[i];
    }

    // Assign to children
    for (int ci = 0; ci < 8; ci++)
    {
        if (!Node->Children[ci].IsValid()) continue;
        for (int k = 0; k < 8; k++)
        {
            int32 gi = OctreeConstants::ChildCornerSources[ci][k];
            Node->Children[ci]->Corners[k].Density = (float)GridDensities[gi];
        }
    }
}

void FAdaptiveOctree::ReconstructSubtree(FAdaptiveOctreeNode* Node)
{
    // 1. Gather unique corners from affected nodes
    TMap<FIntVector, int32> CornerMap;
    TArray<FCornerSample> Samples;
    Samples.Reserve(4096);
    CornerMap.Reserve(4096);
    double QuantizeGrid = ChunkExtent * 1e-8;

    GatherUniqueCorners(Node, Samples, CornerMap, QuantizeGrid);

    if (Samples.Num() == 0) return;

    // 2. Build position arrays
    int32 Count = Samples.Num();
    TArray<float> SX, SY, SZ;
    SX.SetNumUninitialized(Count);
    SY.SetNumUninitialized(Count);
    SZ.SetNumUninitialized(Count);

    for (int32 i = 0; i < Count; i++)
    {
        FVector curPos = Samples[i].Position;
        SX[i] = (float)curPos.X;
        SY[i] = (float)curPos.Y;
        SZ[i] = (float)curPos.Z;
    }

    // 3. One bulk noise call
    TArray<float> SampleOut;
    SampleOut.SetNumUninitialized(Count);

    FSampleInput SampleIn(SX.GetData(), SY.GetData(), SZ.GetData(), Count);
    Compositor->Sample(SampleIn, SampleOut.GetData());

    // 4. Write back densities to all nodes that share each corner
    for (int32 i = 0; i < Count; i++)
    {
        float FinalDensity = SampleOut[i];
        for (float* Target : Samples[i].Targets)
            *Target = FinalDensity;
    }

    // 5. Propagate densities into children beyond the precision depth floor.
    //    Floor-level nodes now have fresh noise+edit densities from step 4.
    //    Deeper children get noise interpolated from parent corners, with edit-store
    //    contributions stripped before interpolation and re-sampled at full resolution.
    PropagateDeepDensities(Node);

    // 6. Finalize edges/QEF
    FinalizeSubtree(Node);
}

void FAdaptiveOctree::EnforceSplitsInSubtree(FAdaptiveOctreeNode* Node)
{
    if (!Node) return;

    if (!Node->bHasEditedDescendants)
        return;

    if (Node->IsLeaf())
    {
        bool bNeedsSplit = false;

        if (Node->bHasEditedDescendants || Node->CouldContainSurface)
        {
            bool bLodWantsSplit = Node->ShouldSplit(CachedCameraPosition, CachedThresholdSq, CachedFOVScale);
            bool bBelowMinDepth = Node->Index.Depth < Node->DepthBounds[EDepthBound::MinDepth];
            bNeedsSplit = bLodWantsSplit || bBelowMinDepth;
        }

        // Respect max depth
        if (bNeedsSplit && Node->Index.Depth < Node->DepthBounds[EDepthBound::MaxDepth])
        {
            SplitAndComputeChildren(Node);

            // Recurse into new children — they might need further splitting
            for (int i = 0; i < 8; i++)
            {
                if (Node->Children[i].IsValid())
                    EnforceSplitsInSubtree(Node->Children[i].Get());
            }
        }
        return;
    }

    // Non-leaf: recurse into children
    for (int i = 0; i < 8; i++)
    {
        if (Node->Children[i].IsValid())
            EnforceSplitsInSubtree(Node->Children[i].Get());
    }
}

FVector2f FAdaptiveOctree::ComputeTriplanarUV(FVector Position, FVector3f Normal) const
{
    FVector2f UV;
    FVector3f AbsNormal = Normal.GetAbs();
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
}

void FAdaptiveOctree::UpdateMeshChunkStreamData(TSharedPtr<FMeshChunk> InChunk)
{
    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> SrfTriangles;

    VertexMap.Reserve(1024);
    UniqueVertices.Reserve(1024);
    SrfTriangles.Reserve(2048);

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(InChunk->ChunkEdges.Num());

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

        for (int i = 0; i < neighbors.Count; i++) {
            FAdaptiveOctreeNode* NodePtr = neighbors.Nodes[i];
            FVector LocalPos(NodePtr->DualContourPosition - InChunk->ChunkCenter);

            AllEdgeData[edgeIdx].Vertices[i].Position = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = NodePtr->DualContourNormal;
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

            if (NumEdgeVerts == 4
                && VertexIndices[0] != VertexIndices[3]
                && VertexIndices[2] != VertexIndices[3])
            {
                FIndex3UI tri1 = FlipWinding
                    ? FIndex3UI(VertexIndices[0], VertexIndices[3], VertexIndices[2])
                    : FIndex3UI(VertexIndices[0], VertexIndices[2], VertexIndices[3]);
                SrfTriangles.Add(tri1);
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
        Tangent.SetNormal(Vertex.Normal);
        SrfTangentStream.Set(VertIdx, Tangent);
        SrfColorStream.Set(VertIdx, Vertex.Color);
        SrfTexCoordStream.Set(VertIdx, ComputeTriplanarUV(WorldPos, Vertex.Normal));
        });

    ParallelFor(SrfTriangles.Num(), [&](int32 TriIdx) {
        SrfTriangleStream.Set(TriIdx, SrfTriangles[TriIdx]);
        SrfPolygroupStream.Set(TriIdx, 0);
        });

    // Publish the freshly-built stream. Under the per-chunk lock so a concurrent game-thread
    // ApplyToComponent captures either the old or new stream whole, never a torn pointer.
    {
        FScopeLock Lock(&InChunk->MeshDataCS);
        InChunk->SurfaceMeshData = UpdatedSurfaceData;
    }

    InChunk->IsDirty = true;
}

void FAdaptiveOctree::AppendUniqueEdges(const TArray<FNodeEdge>& InAppendEdges, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap)
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
        // Only nodes near the surface or with edit history can split —
        // deep interior/exterior nodes are skipped but all leaves still
        // contribute edges and participate in merge checks
        if ((Node->CouldContainSurface || Node->bHasEditedDescendants) && Node->ShouldSplit(CameraPosition, ThresholdSq, InFOVScale))
        {
            OutChanged = true;
            SplitAndComputeChildren(Node);
            // Gather from full subtree in case children inherited CouldContainSurface
            // and will be split further on subsequent LOD passes
            for (int i = 0; i < 8; i++)
                GatherLeafEdges(Node->Children[i].Get(), OutNodeEdges, EdgeMap);
            return;
        }
        else if (Node->Index.LastChild() == 7 && Node->Parent.IsValid() && !Node->bIsChunkRoot)
        {
            // NOTE: !Node->bIsChunkRoot is load-bearing. This branch merges Node's PARENT,
            // destroying all 8 of the parent's children. If Node is a chunk root, its parent
            // is above the chunk cut and its siblings are the other chunk roots -- merging it
            // would free live ChunkMap nodes (and, since chunk siblings are LOD'd in parallel,
            // free nodes other threads are still walking). Coarsening the chunk cut is the
            // demote pass's job alone; LOD stops at the chunk root, which is the merge floor.
            FAdaptiveOctreeNode* ParentPtr = Node->Parent.Pin().Get();
            if (ParentPtr && ParentPtr->ShouldMerge(CameraPosition, MergeThresholdSq, InFOVScale))
            {
                // Verify ALL siblings are leaves before merging
                bool bAllSiblingsLeaves = true;
                for (int i = 0; i < 8; i++)
                {
                    if (ParentPtr->Children[i].IsValid() && !ParentPtr->Children[i]->IsLeaf())
                    {
                        bAllSiblingsLeaves = false;
                        break;
                    }
                }

                if (bAllSiblingsLeaves)
                {
                    OutChanged = true;
                    ParentPtr->Merge();
                    AppendUniqueEdges(ParentPtr->GetSignChangeEdges(), OutNodeEdges, EdgeMap);
                    return;
                }
            }
        }

        // ALL leaves contribute edges — not just surface ones
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

    // Cache for edit pipeline — EnforceSplitsInSubtree needs these
    CachedCameraPosition = CameraPosition;
    CachedFOVScale = FOVScale;
    CachedThresholdSq = ThresholdSq;

    ParallelFor(NumChunks, [&](int32 idx) {
        FAdaptiveOctreeNode* ChunkNode = Chunks[idx].Get();
        double DistSq = FMath::Max(FVector::DistSquared(ChunkNode->Center, CameraPosition), 1e-12);
        bool CouldSplit = FAdaptiveOctreeNode::EvaluateSplit(
            ChunkNode->Extent, DistSq, FOVScale, ThresholdSq,
            ChunkNode->Index.Depth,
            ChunkNode->DepthBounds[EDepthBound::MaxDepth],    // MaxDepth param
            ChunkNode->DepthBounds[EDepthBound::MinDepth]);
        double SmallestExtent = ChunkNode->Extent / (double)(1 << (ChunkNode->DepthBounds[EDepthBound::MaxDepth] - ChunkNode->Index.Depth));
        bool CouldMerge = FAdaptiveOctreeNode::EvaluateMerge(
            SmallestExtent, DistSq, FOVScale, MergeThresholdSq,
            ChunkNode->DepthBounds[EDepthBound::MaxDepth],    // Depth = deepest possible node
            ChunkNode->DepthBounds[EDepthBound::ChunkDepth],  // ChunkDepth
            ChunkNode->DepthBounds[EDepthBound::MinDepth]);   // MinDepth

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

    ParallelFor(ModifiedChunks.Num(), [&](int32 idx) {
        if (ModifiedChunks[idx].IsValid())
            UpdateMeshChunkStreamData(ModifiedChunks[idx]);
        });
}

void FAdaptiveOctree::CollectDirtyChunks(TArray<TSharedPtr<FMeshChunk>>& OutDirtyChunks) const
{
    if (!MeshChunksInitialized) return;

    for (const auto& It : ChunkMap)
    {
        const TSharedPtr<FMeshChunk>& Chunk = It.Value;
        if (Chunk.IsValid() && Chunk->IsDirty)
            OutDirtyChunks.Add(Chunk);
    }
}

// ---------------------------------------------------------------------------
// Chunk cut
// ---------------------------------------------------------------------------

bool FAdaptiveOctree::ChunkJitterExceeds(FAdaptiveOctreeNode* Node, FVector CameraPosition, double FOVScale, double ThresholdSq) const
{
    // Near distance = distance from the camera to the closest point of the node's AABB,
    // floored so that standing on the surface (near-dist -> 0) doesn't demand infinite
    // depth. This is the key difference from the LOD test, which uses center distance:
    // precision is about how close the *surface under your feet* is, not the chunk center
    // (which can be half a planet away for a coarse chunk).
    const FVector C = Node->Center;
    const double  E = Node->Extent;
    const FVector Closest(
        FMath::Clamp(CameraPosition.X, C.X - E, C.X + E),
        FMath::Clamp(CameraPosition.Y, C.Y - E, C.Y + E),
        FMath::Clamp(CameraPosition.Z, C.Z - E, C.Z + E));

    constexpr double MinNearDist = 100.0; // cm floor (~eye height)
    const double NearDistSq = FMath::Max(FVector::DistSquared(Closest, CameraPosition), MinNearDist * MinNearDist);

    // Worst-case FVector3f rounding error at this node's extent: vertices are stored
    // chunk-local, so the largest coordinate magnitude is ~Extent and the absolute error
    // there is Extent * FLT_EPSILON.
    constexpr double FloatEps = 1.1920929e-7;
    const double Jitter = E * FloatEps;

    // Same screen-space form as EvaluateSplit (2 * size * FOVScale, compared against
    // threshold * distance), but the "size" is the jitter magnitude.
    const double lhs = 2.0 * Jitter * FOVScale;
    return (lhs * lhs) > ThresholdSq * NearDistSq;
}

bool FAdaptiveOctree::AllChildrenAreChunkRoots(FAdaptiveOctreeNode* Node) const
{
    if (!Node || Node->IsLeaf()) return false;
    for (int i = 0; i < 8; i++)
    {
        const FAdaptiveOctreeNode* Child = Node->Children[i].Get();
        if (!Child || !Child->bIsChunkRoot) return false;
    }
    return true;
}

void FAdaptiveOctree::CollectAllChunkRoots(FAdaptiveOctreeNode* Node, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out) const
{
    if (!Node) return;
    if (Node->bIsChunkRoot) { Out.Add(Node->AsShared()); return; }  // anti-chain: nothing deeper is a root
    if (Node->IsLeaf()) return;
    for (int i = 0; i < 8; i++)
        CollectAllChunkRoots(Node->Children[i].Get(), Out);
}

bool FAdaptiveOctree::AnyFaceNeighborChunkDeeperThan(FAdaptiveOctreeNode* Node, int DepthThreshold)
{
    const FVector Ctr = Node->Center;
    const double  E = Node->Extent;
    const double  Out = E * 1.01;   // just past the face plane

    // Resolve neighbours finely enough to catch one that VIOLATES the balance delta. A
    // neighbour up to (delta+1) levels finer than Node has 2^(delta+1) cells across the
    // face, so sample that many per axis. The old 2x2 (one-level-finer) grid let a small
    // deep patch hide between samples -> missed skirt / missed demote-refusal -> unstitched
    // >delta boundary -> cracks.
    const int    N = 1 << (MaxChunkDepthDelta + 1);
    const double Cell = (2.0 * E) / (double)N;
    const double Start = -E + 0.5 * Cell;

    static const FVector FaceNormals[6] = {
        FVector(1, 0, 0), FVector(-1, 0, 0),
        FVector(0, 1, 0), FVector(0, -1, 0),
        FVector(0, 0, 1), FVector(0, 0, -1) };

    for (int f = 0; f < 6; f++)
    {
        const FVector Nrm = FaceNormals[f];
        FVector T1, T2;
        if (FMath::Abs(Nrm.X) > 0.5) { T1 = FVector(0, 1, 0); T2 = FVector(0, 0, 1); }
        else if (FMath::Abs(Nrm.Y) > 0.5) { T1 = FVector(1, 0, 0); T2 = FVector(0, 0, 1); }
        else { T1 = FVector(1, 0, 0); T2 = FVector(0, 1, 0); }

        for (int a = 0; a < N; a++)
            for (int b = 0; b < N; b++)
            {
                const FVector P = Ctr + Nrm * Out + T1 * (Start + a * Cell) + T2 * (Start + b * Cell);
                if (const FAdaptiveOctreeNode* Nb = GetChunkNodeByPoint(P))
                    if ((int)Nb->Index.Depth > DepthThreshold)
                        return true;   // found a delta-violating neighbour; that's all we need
            }
    }
    return false;
}

TSharedPtr<FMeshChunk> FAdaptiveOctree::BuildMeshChunkFor(FAdaptiveOctreeNode* Node)
{
    TSharedPtr<FMeshChunk> NewChunk = MakeShared<FMeshChunk>();
    NewChunk->CachedParentActor = CachedParentActor;
    NewChunk->CachedMeshAttachRoot = CachedMeshAttachRoot;
    NewChunk->CachedSurfaceMaterial = CachedSurfaceMaterial;
    NewChunk->InitializeData(Node->Center, Node->Extent);

    TArray<FNodeEdge> Edges;
    TMap<FEdgeKey, int32> EdgeMap;
    GatherLeafEdges(Node, Edges, EdgeMap);
    NewChunk->ChunkEdges = Edges;
    UpdateMeshChunkStreamData(NewChunk);
    NewChunk->IsDirty = (NewChunk->SurfaceMeshData->GetPositionStream().Num() > 0);
    return NewChunk;
}

void FAdaptiveOctree::PromoteChunk(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks)
{
    // Pull the coarse chunk out of the map and clear its flag. It stays alive (still
    // rendering) in OutRemovedChunks until its finer children have applied.
    TSharedPtr<FMeshChunk> OldChunk;
    ChunkMap.RemoveAndCopyValue(ChunkNode, OldChunk);
    ChunkNode->bIsChunkRoot = false;

    // Every child becomes a chunk root so chunk lookups/edits still resolve inside the
    // promoted region, but only surface-bearing children get a meshed FMeshChunk.
    TArray<TSharedPtr<FMeshChunk>> Created;
    for (int i = 0; i < 8; i++)
    {
        const TSharedPtr<FAdaptiveOctreeNode>& Child = ChunkNode->Children[i];
        if (!Child.IsValid()) continue;

        Child->bIsChunkRoot = true;
        if (!Child->CouldContainSurface) continue;

        TSharedPtr<FMeshChunk> NewChunk = BuildMeshChunkFor(Child.Get());
        ChunkMap.Add(Child, NewChunk);
        Created.Add(NewChunk);
    }

    // Retire the coarse chunk, remembering the children that now cover it so the destroy
    // drain won't release it until they're live. bRetired also stops the apply drain from
    // (re)creating a component for it if it was still queued.
    if (OldChunk.IsValid())
    {
        OldChunk->bRetired = true;
        OldChunk->ReplacedBy = MoveTemp(Created);
        OutRemovedChunks.Add(OldChunk);
    }
}

void FAdaptiveOctree::DemoteChunk(TSharedPtr<FAdaptiveOctreeNode> ParentNode, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks)
{
    // Build the parent chunk first (re-meshing its whole subtree against its own coarser
    // origin, which also re-covers any non-surface sub-regions the children skipped) so we
    // can point the retired children at it. GatherLeafEdges walks the subtree directly and
    // doesn't care that the children are still in the map at this instant.
    ParentNode->bIsChunkRoot = true;
    TSharedPtr<FMeshChunk> ParentChunk;
    if (ParentNode->CouldContainSurface)
    {
        ParentChunk = BuildMeshChunkFor(ParentNode.Get());
        ChunkMap.Add(ParentNode, ParentChunk);
    }

    // Retire the 8 child chunk roots. They stay alive (still rendering their finer meshes)
    // until the parent chunk has applied, so the collapse never opens a hole.
    for (int i = 0; i < 8; i++)
    {
        const TSharedPtr<FAdaptiveOctreeNode>& Child = ParentNode->Children[i];
        if (!Child.IsValid()) continue;

        Child->bIsChunkRoot = false;
        TSharedPtr<FMeshChunk> OldChunk;
        if (ChunkMap.RemoveAndCopyValue(Child, OldChunk) && OldChunk.IsValid())
        {
            OldChunk->bRetired = true;
            if (ParentChunk.IsValid())
                OldChunk->ReplacedBy = { ParentChunk };
            OutRemovedChunks.Add(OldChunk);
        }
    }
}

void FAdaptiveOctree::UpdateChunkCut(FVector InCameraPosition, double InCameraFOV, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks)
{
    if (!MeshChunksInitialized) return;

    const double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));
    const double PromoteThresholdSq = ChunkPrecisionThreshold * ChunkPrecisionThreshold;
    const double DemoteThresholdSq = FMath::Square(ChunkPrecisionThreshold * ChunkDemoteHysteresis);

    // Snapshot ALL chunk roots (not just ChunkMap's surface ones) so non-surface chunk
    // roots can trigger their parent's demote -- otherwise a promoted parent whose children
    // all came out non-surface never collapses and deadlocks the cut. Each chunk changes at
    // most one level per pass, spreading the work over frames.
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    CollectAllChunkRoots(Root.Get(), Chunks);

    // Parents already collapsed this pass, so their (snapshotted) siblings skip the retry.
    TSet<FAdaptiveOctreeNode*> DemotedParents;

    for (const TSharedPtr<FAdaptiveOctreeNode>& C : Chunks)
    {
        // May have been retired by a sibling's demote earlier in this loop.
        if (!C.IsValid() || !C->bIsChunkRoot) continue;

        const int Depth = (int)C->Index.Depth;

        // Promotion is only worthwhile if it actually produces a surface (meshable) child;
        // otherwise it just mints non-surface roots (and, since those demote right back,
        // oscillates). A chunk with no surface-bearing child has nothing to refine -- it
        // only participates in demote.
        bool bHasSurfaceChild = false;
        if (!C->IsLeaf())
            for (int i = 0; i < 8; i++)
            {
                const FAdaptiveOctreeNode* Ch = C->Children[i].Get();
                if (Ch && Ch->CouldContainSurface) { bHasSurfaceChild = true; break; }
            }
        const bool bCanGoDeeper = bHasSurfaceChild && (Depth < MaxChunkDepth);

        // --- Promote (precision): too coarse for its near distance. ---
        if (bCanGoDeeper &&
            ChunkJitterExceeds(C.Get(), InCameraPosition, FOVScale, PromoteThresholdSq))
        {
            PromoteChunk(C, OutRemovedChunks);
            continue;
        }

        // --- Promote (balance): never sit more than MaxChunkDepthDelta levels coarser than
        //     a face neighbour, or the leaf floors differ too much for the mesher to stitch
        //     and the boundary cracks. Force-refine toward the finer neighbour. ---
        if (bCanGoDeeper &&
            AnyFaceNeighborChunkDeeperThan(C.Get(), Depth + MaxChunkDepthDelta))
        {
            PromoteChunk(C, OutRemovedChunks);
            continue;
        }

        // --- Demote: collapse C and its 7 siblings back into the parent once the parent
        //     (one step coarser) is comfortably precise by the tighter demote threshold. ---
        TSharedPtr<FAdaptiveOctreeNode> Parent = C->Parent.Pin();
        if (!Parent.IsValid()) continue;
        const int ParentDepth = (int)Parent->Index.Depth;
        if (ParentDepth < ChunkDepth) continue;                      // never collapse below the build floor
        if (DemotedParents.Contains(Parent.Get())) continue;
        if (!AllChildrenAreChunkRoots(Parent.Get())) continue;       // must be the frontier
        if (ChunkJitterExceeds(Parent.Get(), InCameraPosition, FOVScale, DemoteThresholdSq)) continue; // parent still needs the detail

        // Balance: don't coarsen if it would leave the parent more than MaxChunkDepthDelta
        // levels shallower than a still-fine neighbour (that's the crack we're avoiding).
        // The finer neighbour collapses first; this parent follows on a later pass.
        if (AnyFaceNeighborChunkDeeperThan(Parent.Get(), ParentDepth + MaxChunkDepthDelta)) continue;

        DemoteChunk(Parent, OutRemovedChunks);
        DemotedParents.Add(Parent.Get());
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

    // --- Stage 2: Compute 19 new densities ---
    // Beyond PrecisionDepthFloor, noise lacks precision. Strip edit contributions
    // from parent corners, interpolate noise-only baseline, then sample edit store
    // at full resolution for all 27 grid positions.
    if (Node->Index.Depth >= PrecisionDepthFloor)
    {
        // Strip edit contribution from parent corners
        float ParentSX[8], ParentSY[8], ParentSZ[8], ParentEdits[8];
        for (int i = 0; i < 8; i++)
        {
            ParentSX[i] = (float)GridPositions[i].X;
            ParentSY[i] = (float)GridPositions[i].Y;
            ParentSZ[i] = (float)GridPositions[i].Z;
            ParentEdits[i] = 0.f;
        }
        if (Compositor.IsValid())
        {
            FSampleInput ParentInput(ParentSX, ParentSY, ParentSZ, 8);
            Compositor->SampleEditsOnly(ParentInput, ParentEdits);
        }
        for (int i = 0; i < 8; i++)
            GridDensities[i] -= ParentEdits[i];

        // Interpolate noise-only baseline
        for (int i = 0; i < 12; i++)
        {
            int a = OctreeConstants::EdgePairs[i][0];
            int b = OctreeConstants::EdgePairs[i][1];
            GridDensities[8 + i] = (GridDensities[a] + GridDensities[b]) * 0.5;
        }
        for (int i = 0; i < 6; i++)
        {
            GridDensities[20 + i] = (
                GridDensities[OctreeConstants::FaceCorners[i][0]] +
                GridDensities[OctreeConstants::FaceCorners[i][1]] +
                GridDensities[OctreeConstants::FaceCorners[i][2]] +
                GridDensities[OctreeConstants::FaceCorners[i][3]]) * 0.25;
        }
        GridDensities[26] = 0.0;
        for (int i = 0; i < 8; i++)
            GridDensities[26] += GridDensities[i];
        GridDensities[26] *= 0.125;

        // Sample edit store at full resolution for all 27 positions
        if (Compositor.IsValid())
        {
            float SX[27], SY[27], SZ[27], Edits[27];
            for (int32 i = 0; i < 27; i++)
            {
                SX[i] = (float)GridPositions[i].X;
                SY[i] = (float)GridPositions[i].Y;
                SZ[i] = (float)GridPositions[i].Z;
                Edits[i] = 0.f;
            }
            FSampleInput AllInput(SX, SY, SZ, 27);
            Compositor->SampleEditsOnly(AllInput, Edits);
            for (int32 i = 0; i < 27; i++)
                GridDensities[i] += Edits[i];
        }
    }
    else
    {
        const int32 Count = 19;
        float SX[19], SY[19], SZ[19], SampleOut[19];

        for (int32 i = 0; i < Count; i++)
        {
            FVector curPos = GridPositions[8 + i];
            SX[i] = (float)curPos.X;
            SY[i] = (float)curPos.Y;
            SZ[i] = (float)curPos.Z;
        }

        FSampleInput SampleIn(SX, SY, SZ, Count);
        Compositor->Sample(SampleIn, SampleOut);

        for (int32 i = 0; i < Count; i++)
        {
            GridDensities[8 + i] = SampleOut[i];
        }
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
        if (!Normal.Normalize()) Normal = GridPositions[gi].GetSafeNormal();
        GridNormals[gi] = Normal;
    }

    // --- Stage 4: Assign to children and finalize ---
    bool bParentHasEditedDescendants = Node->bHasEditedDescendants;

    for (int ci = 0; ci < 8; ci++)
    {
        for (int k = 0; k < 8; k++)
        {
            int32 gi = OctreeConstants::ChildCornerSources[ci][k];
            Node->Children[ci]->Corners[k].Position = GridPositions[gi];
            Node->Children[ci]->Corners[k].Density = (float)GridDensities[gi];
            Node->Children[ci]->Corners[k].Normal = FVector3f(GridNormals[gi]);
        }
        Node->Children[ci]->FinalizeFromExistingCorners(true); // normals already computed from grid

        // If the parent had edited descendants, check each child against the edit
        // store directly — only flag children whose region actually contains edits.
        if (bParentHasEditedDescendants && Compositor.IsValid())
        {
            auto EditStore = Compositor->GetEditStore();
            if (EditStore.IsValid() && EditStore->HasEditsAlongPath(Node->Children[ci]->Index))
            {
                Node->Children[ci]->bHasEditedDescendants = true;
                Node->Children[ci]->CouldContainSurface = true;
            }
        }
    }
}

void FAdaptiveOctree::ComputeNodeData(FAdaptiveOctreeNode* Node)
{
    float SX[8], SY[8], SZ[8], SampleOut[8];

    for (int i = 0; i < 8; i++)
    {
        FVector curPos = Node->Corners[i].Position;
        SX[i] = (float)curPos.X;
        SY[i] = (float)curPos.Y;
        SZ[i] = (float)curPos.Z;
    }

    FSampleInput SampleIn(SX, SY, SZ, 8);
    Compositor->Sample(SampleIn, SampleOut);

    for (int i = 0; i < 8; i++)
    {
        Node->Corners[i].Density = SampleOut[i];
    }

    Node->FinalizeFromExistingCorners();
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
        if (CurrentNode->bIsChunkRoot)
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