#include "FAdaptiveOctree.h"

FAdaptiveOctree::FAdaptiveOctree(
    ARealtimeMeshActor* InParentActor,
    UMaterialInterface* InSurfaceMaterial,
    UMaterialInterface* InOceanMaterial,
    TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction,
    TSharedPtr<FSparseEditStore> InEditStore,
    FVector InCenter,
    double InRootExtent,
    int InChunkDepth,
    int InMinDepth,
    int InMaxDepth)
{
    RootExtent = InRootExtent;
    ChunkExtent = InRootExtent / FMath::Pow(2.0, (double)InChunkDepth);
    CachedParentActor = InParentActor;
    CachedSurfaceMaterial = InSurfaceMaterial;
    CachedOceanMaterial = InOceanMaterial;

    CornerProvider = MakeShared<FCornerProvider>(
        InEditStore,
        InDensityFunction,
        InCenter,
        InRootExtent,
        0.9,         // SeaLevel fraction
        0.9          // SurfaceLevel fraction
    );

    Root = MakeShared<FAdaptiveOctreeNode>(InCenter, InRootExtent, InChunkDepth, InMinDepth, InMaxDepth);
    TArray<TSharedPtr<FAdaptiveOctreeNode>> InitialLeaves;
    SplitToDepth(Root, InChunkDepth, InitialLeaves);
    CornerProvider->ProvisionCorners(InitialLeaves);
    PopulateChunks();
}

void FAdaptiveOctree::SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNewLeaves)
{
    if (!Node.IsValid()) return;
    if (Node->Index.Depth < InMinDepth)
    {
        Node->Split();
        for (int i = 0; i < 8; i++)
        {
            if (Node->Children[i])
                SplitToDepth(Node->Children[i], InMinDepth, OutNewLeaves);
        }
    }
    else
    {
        OutNewLeaves.Add(Node); // at target depth, this is a leaf
    }
}

void FAdaptiveOctree::PopulateChunks()
{
    // 1. Corners already provisioned — GetSurfaceChunks works correctly
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks = Root->GetSurfaceChunks();

    // 2. Gather non-surface neighbors as buffer chunks
    TArray<TSharedPtr<FAdaptiveOctreeNode>> NeighborChunks;
    for (auto& ChunkNode : Chunks)
    {
        if (!ChunkNode.IsValid()) continue;
        const double Offset = ChunkNode->Extent * 2.0;
        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + Directions[i] * Offset;
            TSharedPtr<FAdaptiveOctreeNode> Neighbor = GetLeafNodeByPoint(NeighborPos);
            if (!Neighbor.IsValid()) continue;
            if (!Neighbor->IsSurface())
                NeighborChunks.AddUnique(Neighbor);
        }
    }
    Chunks.Append(NeighborChunks);

    // 3. Build mesh chunks
    TArray<TSharedPtr<FMeshChunk>> NewChunks;
    NewChunks.SetNum(Chunks.Num());

    ParallelFor(Chunks.Num(), [&](int32 i)
        {
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

    // 4. Serial add to map
    for (int32 i = 0; i < Chunks.Num(); i++)
        ChunkMap.Add(Chunks[i], NewChunks[i]);

    MeshChunksInitialized = true;
}

void FAdaptiveOctree::UpdateChunkMap(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>>& OutDirtyChunks)
{
    TSharedPtr<FMeshChunk>* Found = ChunkMap.Find(ChunkNode);

    if (Found && *Found)
    {
        if (!ChunkNode->IsSurface())
        {
            (*Found)->SurfaceMeshData->ResetStreams();
            (*Found)->IsDirty = true;
        }
        else
        {
            OutDirtyChunks.Add({ ChunkNode, *Found });
        }
    }
    else if (ChunkNode->IsSurface())
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
            if (Neighbor->IsSurface()) continue;
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
    // 1. Gather affected chunk nodes from the edit radius
    TArray<TSharedPtr<FAdaptiveOctreeNode>> AffectedChunkNodes;
    double SearchRadius = InEditRadius * 2.5;
    {
        TArray<TSharedPtr<FAdaptiveOctreeNode>> Stack;
        Stack.Add(Root);
        while (Stack.Num() > 0)
        {
            auto Node = Stack.Pop(EAllowShrinking::No);
            if (!Node.IsValid()) continue;

            FVector Closest;
            Closest.X = FMath::Clamp(InEditCenter.X, Node->Center.X - Node->Extent, Node->Center.X + Node->Extent);
            Closest.Y = FMath::Clamp(InEditCenter.Y, Node->Center.Y - Node->Extent, Node->Center.Y + Node->Extent);
            Closest.Z = FMath::Clamp(InEditCenter.Z, Node->Center.Z - Node->Extent, Node->Center.Z + Node->Extent);
            if (FVector::DistSquared(Closest, InEditCenter) > SearchRadius * SearchRadius) continue;

            if (Node->Index.Depth == Node->DepthBounds[0])
            {
                AffectedChunkNodes.AddUnique(Node);
                continue;
            }

            if (Node->IsLeaf())
            {
                AffectedChunkNodes.AddUnique(Node);
                continue;
            }

            for (int i = 0; i < 8; i++)
                if (Node->Children[i]) Stack.Add(Node->Children[i]);
        }
    }

    // 2. Gather all leaf nodes under affected chunks
    TArray<TSharedPtr<FAdaptiveOctreeNode>> AffectedLeafNodes;
    for (auto& ChunkNode : AffectedChunkNodes)
    {
        TArray<TSharedPtr<FAdaptiveOctreeNode>> Stack;
        Stack.Add(ChunkNode);
        while (Stack.Num() > 0)
        {
            auto Node = Stack.Pop(EAllowShrinking::No);
            if (!Node.IsValid()) continue;
            if (Node->IsLeaf()) AffectedLeafNodes.Add(Node);
            else for (int i = 0; i < 8; i++)
                if (Node->Children[i]) Stack.Add(Node->Children[i]);
        }
    }

    // 3. Apply edit through provider — writes edit store + resamples affected corners + finalizes nodes
    CornerProvider->ApplyEdit(InEditCenter, InEditRadius, InEditStrength, InEditResolution, AffectedLeafNodes);

    // 4. Update chunk map
    TArray<TPair<TSharedPtr<FAdaptiveOctreeNode>, TSharedPtr<FMeshChunk>>> DirtyChunks;
    for (auto& ChunkNode : AffectedChunkNodes)
        UpdateChunkMap(ChunkNode, DirtyChunks);

    // 5. Rebuild mesh streams for dirty chunks
    ParallelFor(DirtyChunks.Num(), [&](int32 i)
        {
            TArray<FNodeEdge> NewEdges;
            GatherLeafEdges(DirtyChunks[i].Key, NewEdges);
            DirtyChunks[i].Value->ChunkEdges = NewEdges;
            UpdateMeshChunkStreamData(DirtyChunks[i].Value);
        });
}

void FAdaptiveOctree::GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FNodeEdge>& OutEdges)
{
    TSet<FNodeEdge> EdgeSet;
    GatherLeafEdgesRecursive(Node, EdgeSet);
    OutEdges = EdgeSet.Array();
}

void FAdaptiveOctree::GatherLeafEdgesRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, TSet<FNodeEdge>& OutEdges)
{
    if (!Node.IsValid()) return;
    if (Node->IsLeaf())
    {
        for (int32 i = 0; i < 12; i++)
        {
            const FVoxelCorner* A = Node->Corners[OctreeConstants::EdgePairs[i][0]].Get();
            const FVoxelCorner* B = Node->Corners[OctreeConstants::EdgePairs[i][1]].Get();
            if (!A || !B) continue;
            if ((A->Density <= 0.0) == (B->Density <= 0.0)) continue;
            OutEdges.Add(FNodeEdge(*A, *B));
        }
        return;
    }
    for (int i = 0; i < 8; i++)
        GatherLeafEdgesRecursive(Node->Children[i], OutEdges);
}

void FAdaptiveOctree::UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, bool& OutChanged, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNewLeaves)
{
    if (Node->IsLeaf())
    {
        if (Node->ShouldSplit(CameraPosition, InScreenSpaceThreshold, InCameraFOV))
        {
            OutChanged = true;
            Node->Split();
            for (int i = 0; i < 8; i++)
                if (Node->Children[i]) OutNewLeaves.Add(Node->Children[i]);
            return;
        }
        else if (Node->Parent.IsValid() &&
            Node->Parent.Pin()->ShouldMerge(CameraPosition, InScreenSpaceThreshold, InCameraFOV) &&
            Node->Index.LastChild() == 7)
        {
            OutChanged = true;
            Node->Parent.Pin()->Merge();
            return;
        }
        return;
    }

    for (int i = 0; i < 8; i++)
        if (Node->Children[i])
            UpdateLodRecursive(Node->Children[i], CameraPosition, InScreenSpaceThreshold, InCameraFOV, OutChanged, OutNewLeaves);
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    ChunkMap.GenerateKeyArray(Chunks);
    int32 NumChunks = Chunks.Num();

    TArray<TSharedPtr<FMeshChunk>> ModifiedChunks;
    ModifiedChunks.SetNumZeroed(NumChunks);

    ParallelFor(NumChunks, [&](int32 idx)
    {
        bool bChanged = false;
        TArray<TSharedPtr<FAdaptiveOctreeNode>> NewLeaves;

        UpdateLodRecursive(Chunks[idx], CameraPosition, InScreenSpaceThreshold, InCameraFOV, bChanged, NewLeaves);

        if (!bChanged) return;

        // Provision new leaves for this chunk immediately
        if (NewLeaves.Num() > 0)
            CornerProvider->ProvisionCorners(NewLeaves);

        // Rebuild this chunk's mesh
        TArray<FNodeEdge> ChunkEdges;
        GatherLeafEdges(Chunks[idx], ChunkEdges);
        auto MeshChunk = ChunkMap[Chunks[idx]];
        MeshChunk->ChunkEdges = ChunkEdges;
        ModifiedChunks[idx] = MeshChunk;
    });

    ParallelFor(NumChunks, [&](int32 i)
    {
        if (ModifiedChunks[i].IsValid())
            UpdateMeshChunkStreamData(ModifiedChunks[i]);
    });
}

void FAdaptiveOctree::UpdateMesh()
{
    if (!MeshChunksInitialized) return;

    for (auto& It : ChunkMap)
    {
        TSharedPtr<FMeshChunk> Chunk = It.Value;
        if (!Chunk.IsValid()) continue;
        if (Chunk->IsDirty)
            Chunk->UpdateComponent(Chunk);
    }
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Nodes;

    FVector Midpoint = (Edge.Positions[0] + Edge.Positions[1]) * 0.5;
    double Epsilon = .5;

    auto GetLeafWithBias = [&](bool BiasPerp1, bool BiasPerp2) -> TSharedPtr<FAdaptiveOctreeNode>
        {
            TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;
            while (CurrentNode.IsValid() && !CurrentNode->IsLeaf())
            {
                int ChildIndex = 0;
                auto CheckSide = [&](int AxisIndex, bool PositiveBias)
                    {
                        double Diff = Midpoint[AxisIndex] - CurrentNode->Center[AxisIndex];
                        if (FMath::Abs(Diff) <= Epsilon) return PositiveBias;
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
                else {
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

TSharedPtr<FAdaptiveOctreeNode> FAdaptiveOctree::GetChunkNodeByPoint(FVector Position)
{
    TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;
    while (CurrentNode.IsValid())
    {
        if (CurrentNode->Index.Depth == CurrentNode->DepthBounds[0]) return CurrentNode;

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

FVector2f FAdaptiveOctree::ComputeTriplanarUV(FVector Position, FVector Normal)
{
    FVector AbsNormal = Normal.GetAbs();
    if (AbsNormal.X > AbsNormal.Y && AbsNormal.X > AbsNormal.Z)
        return FVector2f(Position.Y, Position.Z) * 0.0001f;
    else if (AbsNormal.Y > AbsNormal.X && AbsNormal.Y > AbsNormal.Z)
        return FVector2f(Position.X, Position.Z) * 0.0001f;
    else
        return FVector2f(Position.X, Position.Y) * 0.0001f;
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

    double OceanRadius = RootExtent * 0.9;
    double OceanTriThreshold = -1000;

    ParallelFor(InChunk->ChunkEdges.Num(), [&](int32 edgeIdx)
        {
            const FNodeEdge currentEdge = InChunk->ChunkEdges[edgeIdx];
            TArray<TSharedPtr<FAdaptiveOctreeNode>> nodesToMesh = SampleNodesAroundEdge(currentEdge);

            if (!InChunk->ShouldProcessEdge(currentEdge, nodesToMesh))
            {
                AllEdgeData[edgeIdx].IsValid = false;
                return;
            }

            AllEdgeData[edgeIdx].IsValid = true;
            AllEdgeData[edgeIdx].Edge = currentEdge;
            AllEdgeData[edgeIdx].Vertices.SetNumZeroed(nodesToMesh.Num());

            for (int i = 0; i < nodesToMesh.Num(); i++)
            {
                FVector PlanetCenter = Root->Center;
                FVector LocalPos = nodesToMesh[i]->DualContourPosition - InChunk->ChunkCenter;
                FVector WorldPos = nodesToMesh[i]->DualContourPosition;
                FVector NormLocalPos = nodesToMesh[i]->NormalizedPosition - InChunk->ChunkCenter;
                double Dist = FVector::Dist(WorldPos, PlanetCenter);

                AllEdgeData[edgeIdx].Vertices[i].Position = LocalPos;
                AllEdgeData[edgeIdx].Vertices[i].NormalizedPosition = NormLocalPos;
                AllEdgeData[edgeIdx].Vertices[i].Normal = nodesToMesh[i]->DualContourNormal;
                AllEdgeData[edgeIdx].Vertices[i].Depth = (float)(OceanRadius - Dist);
                AllEdgeData[edgeIdx].Vertices[i].Color = FColor::Green;
            }
        });

    for (int32 edgeIdx = 0; edgeIdx < AllEdgeData.Num(); edgeIdx++)
    {
        if (!AllEdgeData[edgeIdx].IsValid) continue;

        const auto& currentEdge = AllEdgeData[edgeIdx].Edge.GetValue();
        const TArray<FMeshVertex>& EdgeVertices = AllEdgeData[edgeIdx].Vertices;

        int32 VertexIndices[4];
        int32 NumEdgeVerts = EdgeVertices.Num();

        for (int i = 0; i < NumEdgeVerts; i++)
        {
            int32* ExistingIndex = VertexMap.Find(EdgeVertices[i]);
            if (ExistingIndex)
            {
                VertexIndices[i] = *ExistingIndex;
            }
            else
            {
                int32 NewIndex = UniqueVertices.Num();
                UniqueVertices.Add(EdgeVertices[i]);
                VertexMap.Add(EdgeVertices[i], NewIndex);
                VertexIndices[i] = NewIndex;
            }
        }

        bool FlipWinding = (currentEdge.Densities[0] < 0);

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

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx)
        {
            FMeshVertex& Vertex = UniqueVertices[VertIdx];
            FVector WorldPos = FVector(Vertex.Position) + ChunkCenter;

            SrfPositionStream.Set(VertIdx, Vertex.Position);
            FRealtimeMeshTangentsHighPrecision Tangent;
            Tangent.SetNormal(FVector3f(Vertex.Normal));
            SrfTangentStream.Set(VertIdx, Tangent);
            SrfColorStream.Set(VertIdx, Vertex.Color);
            SrfTexCoordStream.Set(VertIdx, ComputeTriplanarUV(WorldPos, Vertex.Normal));

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

    ParallelFor(SrfTriangles.Num(), [&](int32 TriIdx)
        {
            SrfTriangleStream.Set(TriIdx, SrfTriangles[TriIdx]);
            SrfPolygroupStream.Set(TriIdx, 0);
        });

    ParallelFor(OcnTriangles.Num(), [&](int32 TriIdx)
        {
            OcnTriangleStream.Set(TriIdx, OcnTriangles[TriIdx]);
            OcnPolygroupStream.Set(TriIdx, 0);
        });

    InChunk->SurfaceMeshData = UpdatedSurfaceData;
    InChunk->OceanMeshData = UpdatedOceanData;
    InChunk->IsDirty = (SrfTriangles.Num() > 0);
}

void FAdaptiveOctree::CleanupData()
{
    CornerProvider->PruneExpiredCorners();
}

void FAdaptiveOctree::Clear()
{
    Root.Reset();
}