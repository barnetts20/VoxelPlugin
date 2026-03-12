#include "FAdaptiveOctree.h"
#include "FNodeStructureProvider.h"

FAdaptiveOctree::FAdaptiveOctree(ARealtimeMeshActor* InParentActor, UMaterialInterface* InSurfaceMaterial, UMaterialInterface* InOceanMaterial, TFunction<void(int, const float*, const float*, const float*, float*)> InDensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InRootExtent, int InChunkDepth, int InMinDepth, int InMaxDepth)
{
    RootExtent = InRootExtent;
    ChunkExtent = InRootExtent / FMath::Pow(2.0, (double)InChunkDepth);
    CachedParentActor = InParentActor;
    CachedSurfaceMaterial = InSurfaceMaterial;
    CachedOceanMaterial = InOceanMaterial;
    ChunkDepth = InChunkDepth;
    double SeaLevel = .9; //Later on we will expose this to the BP, for now we can just hard code
    double SurfaceLevel = .9; //Later on we will expose this to the BP, for now we can just hard code
    StructureProvider = MakeShared<FNodeStructureProvider>(InEditStore, InDensityFunction, InRootExtent, SeaLevel, SurfaceLevel, InCenter);

    Root = MakeShared<FAdaptiveOctreeNode>(InCenter, InRootExtent, InChunkDepth, InMinDepth, InMaxDepth);

    //Split
    TArray<TSharedPtr<FAdaptiveOctreeNode>> splitNodes;
    splitNodes.Add(Root);
    SplitToDepth(Root, InChunkDepth, splitNodes);
    StructureProvider->PopulateNodeStructure(splitNodes);

    //Populate surface chunks
    PopulateChunks();
}

void FAdaptiveOctree::GatherNodesInSphere(TSharedPtr<FAdaptiveOctreeNode> Node, const FVector& Center, double Radius, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNodes)
{
    if (!Node.IsValid()) return;

    // AABB vs Sphere check
    // Node->Extent is half-size. Check if the sphere overlaps this node's box.
    double DistSq = 0;
    for (int i = 0; i < 3; i++)
    {
        double v = Center[i];
        double min = Node->Center[i] - Node->Extent;
        double max = Node->Center[i] + Node->Extent;

        if (v < min) DistSq += (min - v) * (min - v);
        else if (v > max) DistSq += (v - max) * (v - max);
    }

    if (DistSq <= (Radius * Radius))
    {
        OutNodes.Add(Node);

        // If not a leaf, recurse into children
        if (!Node->IsLeaf())
        {
            for (int i = 0; i < 8; i++)
            {
                GatherNodesInSphere(Node->Children[i], Center, Radius, OutNodes);
            }
        }
    }
}

void FAdaptiveOctree::SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InTargetDepth, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNewNodes)
{
    if (!Node.IsValid() || Node->Index.Depth >= InTargetDepth) return;

    // 1. Perform the structural split
    Node->Split();

    // 2. Track the children and recurse
    for (int i = 0; i < 8; i++)
    {
        if (Node->Children[i].IsValid())
        {
            // Add to the batch for the Provider
            OutNewNodes.Add(Node->Children[i]);

            // Continue splitting until we hit target depth
            SplitToDepth(Node->Children[i], InTargetDepth, OutNewNodes);
        }
    }
}

void FAdaptiveOctree::PopulateChunks()
{
    // 1. Identify which nodes act as our render-chunk roots
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks = Root->GetSurfaceChunks();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> NeighborChunks;

    UE_LOG(LogTemp, Warning, TEXT("[Voxel] PopulateChunks: %d surface chunks found at ChunkDepth %d"), Chunks.Num(), ChunkDepth);

    // Count nodes at chunk depth to verify the tree structure
    int32 TotalReadyNodes = 0, TotalSignChangeEdges = 0;
    for (auto& ChunkNode : Chunks)
    {
        if (!ChunkNode.IsValid()) continue;
        if (ChunkNode->bDataReady) TotalReadyNodes++;
        for (int i = 0; i < 12; i++)
            if (ChunkNode->Edges[i] && ChunkNode->Edges[i]->GetSignChange()) TotalSignChangeEdges++;
    }
    UE_LOG(LogTemp, Warning, TEXT("[Voxel] PopulateChunks: %d/%d chunks have bDataReady, %d total sign-change edges across chunk-depth nodes"),
        TotalReadyNodes, Chunks.Num(), TotalSignChangeEdges);

    for (auto& ChunkNode : Chunks)
    {
        if (!ChunkNode.IsValid()) continue;

        const double Offset = ChunkNode->Extent * 2.0;
        for (int i = 0; i < 6; i++)
        {
            FVector NeighborPos = ChunkNode->Center + Directions[i] * Offset;
            TSharedPtr<FAdaptiveOctreeNode> NeighborNode = GetLeafNodeByPoint(NeighborPos);

            if (!NeighborNode.IsValid()) continue;

            // If the neighbor isn't already a surface chunk, we add it to ensure 
            // the edges between "solid" and "empty" chunks are correctly manifold.
            if (!NeighborNode->IsSurface())
                NeighborChunks.AddUnique(NeighborNode);
        }
    }
    Chunks.Append(NeighborChunks);

    // 2. Prepare the mesh chunk objects
    TArray<TSharedPtr<FMeshChunk>> NewChunks;
    NewChunks.SetNum(Chunks.Num());

    // 3. Serial meshing - UpdateMeshChunkStreamData reads shared edge/face/node data
    // that is not safe to access from multiple threads simultaneously.
    for (int32 i = 0; i < Chunks.Num(); i++) {
        NewChunks[i] = MakeShared<FMeshChunk>();
        NewChunks[i]->CachedParentActor = CachedParentActor;
        NewChunks[i]->CachedSurfaceMaterial = CachedSurfaceMaterial;
        NewChunks[i]->CachedOceanMaterial = CachedOceanMaterial;
        NewChunks[i]->InitializeData(Chunks[i]->Center, Chunks[i]->Extent);

        TArray<FVoxelEdge*> Edges;
        GatherLeafEdges(Chunks[i], Edges);
        NewChunks[i]->ChunkEdges = Edges;

        UpdateMeshChunkStreamData(NewChunks[i]);

        NewChunks[i]->IsDirty = (NewChunks[i]->SurfaceMeshData->GetPositionStream().Num() > 0);
    }

    // 4. Update the map (Serial)
    int32 DirtyChunks = 0, TotalTris = 0, TotalEdgesProcessed = 0, TotalSignChange = 0;
    for (int32 i = 0; i < Chunks.Num(); i++)
    {
        ChunkMap.Add(Chunks[i], NewChunks[i]);
        if (NewChunks[i]->IsDirty) DirtyChunks++;
        if (auto* TriStream = NewChunks[i]->SurfaceMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles))
            TotalTris += TriStream->Num();
        TotalEdgesProcessed += NewChunks[i]->ChunkEdges.Num();
        for (auto* Edge : NewChunks[i]->ChunkEdges)
            if (Edge && Edge->GetSignChange()) TotalSignChange++;
    }
    UE_LOG(LogTemp, Warning, TEXT("[Voxel] PopulateChunks done: %d chunks, %d dirty, %d total leaf edges, %d sign-change edges, %d triangles"),
        Chunks.Num(), DirtyChunks, TotalEdgesProcessed, TotalSignChange, TotalTris);

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
    // 1. Gather affected nodes and delegate edit + resample to the provider
    TArray<TSharedPtr<FAdaptiveOctreeNode>> AffectedNodes;
    GatherNodesInSphere(Root, InEditCenter, InEditRadius * 1.25, AffectedNodes);
    StructureProvider->ApplyEdit(InEditCenter, InEditRadius, InEditStrength, InEditResolution, AffectedNodes);

    // 2. Find chunk-depth nodes within the affected radius and rebuild their mesh data
    TArray<TSharedPtr<FAdaptiveOctreeNode>> DirtyChunkNodes;
    for (auto& Node : AffectedNodes)
    {
        if (Node.IsValid() && Node->Index.Depth == ChunkDepth)
        {
            DirtyChunkNodes.AddUnique(Node);
        }
    }

    for (auto& ChunkNode : DirtyChunkNodes)
    {
        TSharedPtr<FMeshChunk>* Found = ChunkMap.Find(ChunkNode);
        if (Found && Found->IsValid())
        {
            (*Found)->ChunkEdges.Reset();
            GatherLeafEdges(ChunkNode, (*Found)->ChunkEdges);
            UpdateMeshChunkStreamData(*Found);
        }
    }
}

FVector2f FAdaptiveOctree::ComputeTriplanarUV(FVector Position, FVector3f Normal)
{
    FVector2f UV;
    FVector3f AbsNormal = Normal.GetAbs();

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
    // --- DIAGNOSTIC COUNTERS ---
    std::atomic<int32> TotalEdges(0);
    std::atomic<int32> SkippedEdges(0); // < 3 nodes
    std::atomic<int32> ThreeNodeEdges(0);
    std::atomic<int32> FourNodeEdges(0);
    std::atomic<int32> MoreThanFourEdges(0);

    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> SrfTriangles;
    TArray<FIndex3UI> OcnTriangles;

    VertexMap.Reserve(1024);
    UniqueVertices.Reserve(1024);
    SrfTriangles.Reserve(2048);

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(InChunk->ChunkEdges.Num());

    double OceanRadius = RootExtent * 0.8;
    double OceanTriThreshold = -1000;

    ParallelFor(InChunk->ChunkEdges.Num(), [&](int32 edgeIdx) {
        FVoxelEdge* CurrentEdge = InChunk->ChunkEdges[edgeIdx];
        if (!CurrentEdge || !CurrentEdge->GetSignChange())
        {
            AllEdgeData[edgeIdx].IsValid = false;
            return;
        }

        TotalEdges++;

        TArray<TSharedPtr<FAdaptiveOctreeNode>> NodesToMesh;
        SampleNodesAroundEdge(CurrentEdge, NodesToMesh);

        int32 NodeCount = NodesToMesh.Num();

        // Log specific counts for this edge
        if (NodeCount < 3) SkippedEdges++;
        else if (NodeCount == 3) ThreeNodeEdges++;
        else if (NodeCount == 4) FourNodeEdges++;
        else MoreThanFourEdges++;

        if (NodeCount < 3)
        {
            AllEdgeData[edgeIdx].IsValid = false;
            return;
        }

        AllEdgeData[edgeIdx].IsValid = true;
        AllEdgeData[edgeIdx].Edge = CurrentEdge;
        AllEdgeData[edgeIdx].Vertices.SetNumZeroed(NodeCount);

        for (int i = 0; i < NodeCount; i++)
        {
            FVector LocalPos = NodesToMesh[i]->DualContourPosition - InChunk->ChunkCenter;
            FVector WorldPos = NodesToMesh[i]->DualContourPosition;
            FVector NormLocalPos = NodesToMesh[i]->NormalizedPosition - InChunk->ChunkCenter;
            double Dist = FVector::Dist(WorldPos, Root->Center);

            AllEdgeData[edgeIdx].Vertices[i].Position = LocalPos;
            AllEdgeData[edgeIdx].Vertices[i].NormalizedPosition = NormLocalPos;
            AllEdgeData[edgeIdx].Vertices[i].Normal = NodesToMesh[i]->DualContourNormal;
            AllEdgeData[edgeIdx].Vertices[i].Depth = (float)(OceanRadius - Dist);
            AllEdgeData[edgeIdx].Vertices[i].Color = FColor::Green;
        }
        });

    // --- LOG THE RESULTS ---
    if (TotalEdges > 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("[Voxel Mesh] Chunk at %s processed %d crossing edges:"),
            *InChunk->ChunkCenter.ToString(), TotalEdges.load());
        UE_LOG(LogTemp, Warning, TEXT("   - Quads (4 nodes): %d"), FourNodeEdges.load());
        UE_LOG(LogTemp, Warning, TEXT("   - Tris  (3 nodes): %d"), ThreeNodeEdges.load());
        UE_LOG(LogTemp, Error, TEXT("   - Holes ( <3 nodes): %d  <-- THIS CAUSES MISSING TRIS"), SkippedEdges.load());
        if (MoreThanFourEdges > 0)
            UE_LOG(LogTemp, Error, TEXT("   - Non-Manifold (>4 nodes): %d"), MoreThanFourEdges.load());
    }

    for (int32 edgeIdx = 0; edgeIdx < AllEdgeData.Num(); edgeIdx++)
    {
        if (!AllEdgeData[edgeIdx].IsValid) continue;

        const TArray<FMeshVertex>& EdgeVertices = AllEdgeData[edgeIdx].Vertices;
        const int32 NumVerts = EdgeVertices.Num();

        // Resolve or insert vertices, collecting indices in winding order
        int32 VertexIndices[4];
        for (int i = 0; i < NumVerts; i++)
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

        // Winding order is guaranteed by SampleNodesAroundEdge (CCW around edge axis).
        // Emit as fan from vertex 0: tri0 = (0,1,2), tri1 = (0,2,3) for quad case.
        // Degenerate check: skip any triangle where two indices are the same
        // (can happen at LOD boundaries where two slots resolved to the same coarse node).
        auto EmitTri = [&](int32 A, int32 B, int32 C)
            {
                if (A == B || B == C || A == C) return;
                FIndex3UI Tri(VertexIndices[A], VertexIndices[B], VertexIndices[C]);
                SrfTriangles.Add(Tri);
                if (UniqueVertices[Tri.V0].Depth > OceanTriThreshold ||
                    UniqueVertices[Tri.V1].Depth > OceanTriThreshold ||
                    UniqueVertices[Tri.V2].Depth > OceanTriThreshold)
                {
                    OcnTriangles.Add(Tri);
                }
            };
            short Sign = (*AllEdgeData[edgeIdx].Edge)->GetSignChange();
            if (Sign > 0) // solid -> empty: standard winding
            {
                EmitTri(0, 1, 2);
                if (NumVerts == 4) EmitTri(0, 2, 3);
            }
            else // empty -> solid: reverse winding
            {
                EmitTri(0, 2, 1);
                if (NumVerts == 4) EmitTri(0, 3, 2);
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
        FVector WorldPos = FVector(Vertex.Position) + ChunkCenter;

        SrfPositionStream.Set(VertIdx, Vertex.Position);
        FRealtimeMeshTangentsHighPrecision Tangent;
        Tangent.SetNormal(FVector3f(Vertex.Normal));
        SrfTangentStream.Set(VertIdx, Tangent);
        SrfColorStream.Set(VertIdx, Vertex.Color);
        SrfTexCoordStream.Set(VertIdx, ComputeTriplanarUV(WorldPos, Vertex.Normal));

        // Ocean same index, different position/normal
        OcnPositionStream.Set(VertIdx, Vertex.NormalizedPosition);
        FRealtimeMeshTangentsHighPrecision OcnTangent;
        FVector3f OcnNormal = FVector3f(Vertex.NormalizedPosition + ChunkCenter - Root->Center).GetSafeNormal();
        OcnTangent.SetNormal(FVector3f(OcnNormal));
        OcnTangentStream.Set(VertIdx, OcnTangent);

        // Encode
        float absDepth = FMath::Max(Vertex.Depth, 0.0f);
        bool isUnderwater = (Vertex.Depth >= 0);
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
    InChunk->OceanMeshData = UpdatedOceanData;

    InChunk->IsDirty = (SrfTriangles.Num() > 0);
}

void FAdaptiveOctree::UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNewNodes, bool& OutChanged)
{
    if (!Node.IsValid()) return;

    if (Node->IsLeaf())
    {
        if (Node->ShouldSplit(CameraPosition, InScreenSpaceThreshold, InCameraFOV))
        {
            OutChanged = true;
            Node->Split();
            for (int i = 0; i < 8; i++)
            {
                OutNewNodes.Add(Node->Children[i]);
            }
        }
    }
    else
    {
        // Check merge before recursing --- only trigger on child 7 to avoid 8x redundant checks
        if (Node->ShouldMerge(CameraPosition, InScreenSpaceThreshold, InCameraFOV))
        {
            OutChanged = true;
            Node->Merge();
            // Parent node retains its existing structure data --- no provider call needed
            return;
        }

        for (int i = 0; i < 8; i++)
        {
            UpdateLodRecursive(Node->Children[i], CameraPosition, InScreenSpaceThreshold, InCameraFOV, OutNewNodes, OutChanged);
        }
    }
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    ChunkMap.GenerateKeyArray(Chunks);
    int32 NumChunks = Chunks.Num();

    // Per-chunk results --- pre-allocated for parallel write safety
    TArray<TArray<TSharedPtr<FAdaptiveOctreeNode>>> PerChunkNewNodes;
    TArray<bool> ChunkChanged;
    PerChunkNewNodes.SetNum(NumChunks);
    ChunkChanged.SetNum(NumChunks);
    for (int32 i = 0; i < NumChunks; i++) ChunkChanged[i] = false;

    // Stage 1: Parallel LOD update --- splits/merges only, no provider calls yet
    ParallelFor(NumChunks, [&](int32 idx)
        {
            UpdateLodRecursive(Chunks[idx], CameraPosition, InScreenSpaceThreshold, InCameraFOV, PerChunkNewNodes[idx], ChunkChanged[idx]);
        });

    // Stage 2: Batch all new nodes from splits to the provider (serial collect, then one call)
    TArray<TSharedPtr<FAdaptiveOctreeNode>> AllNewNodes;
    for (int32 i = 0; i < NumChunks; i++)
    {
        AllNewNodes.Append(PerChunkNewNodes[i]);
    }
    if (AllNewNodes.Num() > 0)
    {
        StructureProvider->PopulateNodeStructure(AllNewNodes);
    }

    // Stage 3: Rebuild mesh stream data for any chunk that changed
    TArray<TSharedPtr<FMeshChunk>> DirtyChunks;
    for (int32 i = 0; i < NumChunks; i++)
    {
        if (ChunkChanged[i])
        {
            TSharedPtr<FMeshChunk> MeshChunk = ChunkMap[Chunks[i]];
            if (MeshChunk.IsValid())
            {
                MeshChunk->ChunkEdges.Reset();
                GatherLeafEdges(Chunks[i], MeshChunk->ChunkEdges);
                DirtyChunks.Add(MeshChunk);
            }
        }
    }

    ParallelFor(DirtyChunks.Num(), [&](int32 i)
        {
            UpdateMeshChunkStreamData(DirtyChunks[i]);
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

void FAdaptiveOctree::GatherLeafEdges(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FVoxelEdge*>& OutEdges)
{
    if (!Node.IsValid()) return;

    if (Node->IsLeaf())
    {
        auto nodeEdges = Node->GetSignChangeEdges();
        for (auto edge : nodeEdges) {
            OutEdges.AddUnique(edge);
        }
        return;
    }

    for (int i = 0; i < 8; i++)
        GatherLeafEdges(Node->Children[i], OutEdges);
}

void FAdaptiveOctree::SampleNodesAroundEdge(FVoxelEdge* Edge, TArray<TSharedPtr<FAdaptiveOctreeNode>>& OutNodes)
{
    if (!Edge) return;

    FVoxelFace* Faces[4] = { nullptr, nullptr, nullptr, nullptr };
    int32 FilledCount = 0;

    // Seed from this edge's own connected faces
    for (int32 i = 0; i < 4; i++)
    {
        Faces[i] = const_cast<FVoxelFace*>(Edge->GetConnectedFace(i));
        if (Faces[i]) FilledCount++;
    }

    // Climb the edge parent chain to fill remaining empty slots
    if (FilledCount < 4)
    {
        FVoxelEdge* ParentEdge = Edge->GetParent();
        while (ParentEdge && FilledCount < 4)
        {
            for (int32 i = 0; i < 4; i++)
            {
                if (!Faces[i])
                {
                    FVoxelFace* ParentFace = const_cast<FVoxelFace*>(ParentEdge->GetConnectedFace(i));
                    if (ParentFace)
                    {
                        Faces[i] = ParentFace;
                        FilledCount++;
                    }
                }
            }
            ParentEdge = ParentEdge->GetParent();
        }
    }

    // T-junction fallback: climb adjacent filled slot's face parent chain
    // Adjacent slots in the cycle: 0<->1<->2<->3<->0
    for (int32 i = 0; i < 4; i++)
    {
        if (Faces[i]) continue;

        const int32 AdjacentSlots[2] = { (i + 1) % 4, (i + 3) % 4 };
        for (int32 adj : AdjacentSlots)
        {
            if (!Faces[adj]) continue;

            FVoxelFace* CurrFace = Faces[adj]->GetParent();
            while (CurrFace)
            {
                // Pick the node slot using the same world-space logic as RegisterNode:
                // the node whose center is on the same side of the face plane as the edge.
                double EdgeCoord = Edge->GetMinCorner()->GetPosition()[CurrFace->Axis];
                double FaceCoord = CurrFace->Key.Min->GetPosition()[CurrFace->Axis];
                int32 NodeSlot = (EdgeCoord < FaceCoord) ? 0 : 1;

                TSharedPtr<FAdaptiveOctreeNode> ParentFaceNodes[2];
                CurrFace->GetNodes(ParentFaceNodes);
                if (ParentFaceNodes[NodeSlot].IsValid())
                {
                    Faces[i] = CurrFace;
                    FilledCount++;
                    break;
                }
                CurrFace = CurrFace->GetParent();
            }
            if (Faces[i]) break;
        }
    }

    // Collect nodes - for each face, pick the node on the same side as the edge
    // using the same world-space comparison as RegisterNode.
    for (int32 SlotIdx = 0; SlotIdx < 4; SlotIdx++)
    {
        FVoxelFace* Face = Faces[SlotIdx];
        if (!Face) continue;

        double EdgeCoord = Edge->GetMinCorner()->GetPosition()[Face->Axis];
        double FaceCoord = Face->Key.Min->GetPosition()[Face->Axis];
        int32 NodeSlot = (EdgeCoord < FaceCoord) ? 0 : 1;

        TSharedPtr<FAdaptiveOctreeNode> FaceNodes[2];
        Face->GetNodes(FaceNodes);
        if (FaceNodes[NodeSlot].IsValid())
        {
            OutNodes.AddUnique(FaceNodes[NodeSlot]);
        }
    }
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