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
    for (int i = 0; i < 8; i++)
    {
        FVector CornerPos = Root->Center + OctreeConstants::Offsets[i] * Root->Extent;
        Root->Corners[i] = AcquireCorner(CornerPos);
    }
    ComputeNodeData(Root);
    SplitToDepth(Root, InChunkDepth);

    PopulateChunks();
}

int32 FAdaptiveOctree::AcquireCorner(const FVector& Position)
{
    FIntVector Key = CornerKeyUtils::Quantize(Position);

    // Lock required because UpdateLOD calls Split() inside a ParallelFor
    FScopeLock Lock(&CornerPoolLock);

    int32* ExistingIdx = CornerMap.Find(Key);
    if (ExistingIdx)
    {
        CornerPool[*ExistingIdx].RefCount++;
        return *ExistingIdx;
    }

    FCornerData NewData;
    NewData.Position = Position;
    NewData.Density = 0.0;
    NewData.Normal = FVector::ZeroVector;
    NewData.RefCount = 1;

    int32 NewIdx = CornerPool.Add(NewData);
    CornerMap.Add(Key, NewIdx);
    return NewIdx;
}

void FAdaptiveOctree::ReleaseCorner(int32 CornerIndex)
{
    if (CornerIndex == INDEX_NONE) return;

    FScopeLock Lock(&CornerPoolLock);

    check(CornerPool.IsValidIndex(CornerIndex));
    FCornerData& Corner = CornerPool[CornerIndex];

    check(Corner.RefCount > 0);
    Corner.RefCount--;

    if (Corner.RefCount <= 0)
    {
        FIntVector Key = CornerKeyUtils::Quantize(Corner.Position);
        CornerMap.Remove(Key);
        CornerPool.RemoveAt(CornerIndex);
    }
}

// Updated AcquireEdge signature
int32 FAdaptiveOctree::AcquireEdge(int32 C0, int32 C1, TSharedPtr<FAdaptiveOctreeNode> RequestingNode) {
    FEdgeKey Key(C0, C1);
    
    FScopeLock Lock(&EdgePoolLock);
    if (int32* ExistingIndex = EdgeLookupMap.Find(Key)) {
        // Just add the pointer directly
        EdgePool[*ExistingIndex].ConnectedNodes.AddUnique(RequestingNode);
        return *ExistingIndex;
    }

    int32 NewIndex = EdgePool.Add(FOctreeEdge());
    FOctreeEdge& NewEdge = EdgePool[NewIndex];
    
    NewEdge.Key = Key;

    // The axis is the index where the coordinates are not equal
    const FVector& P0 = CornerPool[C0].Position;
    const FVector& P1 = CornerPool[C1].Position;

    int axis = 2;
    // The axis is the index where the coordinates are not equal
    if (FMath::Abs(P0.X - P1.X) > 1e-4) axis = 0; // X-Axis
    if (FMath::Abs(P0.Y - P1.Y) > 1e-4) axis = 1; // Y-Axis

    NewEdge.Axis = axis; // Derived from CornerPool positions
    
    const auto& Cor0 = CornerPool[C0];
    const auto& Cor1 = CornerPool[C1];
    
    NewEdge.bIsSignChange = (Cor0.Density * Cor1.Density < 0);
    NewEdge.bStartIsInside = (Cor0.Density < 0);
    NewEdge.ConnectedNodes.Add(RequestingNode);

    EdgeLookupMap.Add(Key, NewIndex);
    return NewIndex;
}

void FAdaptiveOctree::SortNodesCircular(TArray<TSharedPtr<FAdaptiveOctreeNode>, TInlineAllocator<4>>& Nodes, int32 Axis) {
    if (Nodes.Num() < 3) return;

    // We sort based on their position relative to the edge axis
    // If Axis is X (0), we sort by Angle in the YZ plane.
    int32 A1 = (Axis + 1) % 3;
    int32 A2 = (Axis + 2) % 3;

    FVector2D Center(0, 0);
    for (const auto& Node : Nodes) {
        Center.X += Node->Center[A1];
        Center.Y += Node->Center[A2];
    }
    Center /= Nodes.Num();

    Nodes.Sort([&](const TSharedPtr<FAdaptiveOctreeNode>& A, const TSharedPtr<FAdaptiveOctreeNode>& B) {
        float AngleA = FMath::Atan2(A->Center[A2] - Center.Y, A->Center[A1] - Center.X);
        float AngleB = FMath::Atan2(B->Center[A2] - Center.Y, B->Center[A1] - Center.X);
        return AngleA < AngleB;
        });
}
// Used during initial ComputeNodeData and ApplyEdit
void FAdaptiveOctree::SetCornerDensity(int32 Index, double NewDensity)
{
    FScopeLock Lock(&CornerPoolLock);
    if (CornerPool.IsValidIndex(Index))
    {
        CornerPool[Index].Density = NewDensity;
    }
}

// Used during FinalizeFromExistingCorners
void FAdaptiveOctree::SetCornerNormal(int32 Index, const FVector& NewNormal)
{
    FScopeLock Lock(&CornerPoolLock);
    if (CornerPool.IsValidIndex(Index))
    {
        CornerPool[Index].Normal = NewNormal;
    }
}

void FAdaptiveOctree::SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth)
{
    if (!Node.IsValid()) return;

    if (Node->Index.Depth < InMinDepth)
    {
        Node->Split(this);
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

        TArray<int32> EdgeIndices;
        GatherLeafEdges(Chunks[i], EdgeIndices);

        NewChunks[i]->ChunkEdges = EdgeIndices;

        UpdateMeshChunkStreamData(NewChunks[i]);

        NewChunks[i]->IsDirty =
            (NewChunks[i]->SurfaceMeshData->GetPositionStream().Num() > 0);
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
        TArray<int32> NewEdges;
        GatherLeafEdges(DirtyChunks[i].Key, NewEdges);
        DirtyChunks[i].Value->ChunkEdges = NewEdges;
        UpdateMeshChunkStreamData(DirtyChunks[i].Value);
        });
}

void FAdaptiveOctree::GatherLeafEdges(
    TSharedPtr<FAdaptiveOctreeNode> Node,
    TArray<int32>& OutEdges
)
{
    if (!Node.IsValid())
    {
        return;
    }

    if (Node->IsLeaf())
    {
        for (int32 EdgeIndex : Node->Edges)
        {
            OutEdges.Add(EdgeIndex);
        }

        return;
    }

    for (int32 i = 0; i < 8; i++)
    {
        if (Node->Children[i].IsValid())
        {
            GatherLeafEdges(
                Node->Children[i],
                OutEdges
            );
        }
    }
}
void FAdaptiveOctree::GatherUniqueCorners(TSharedPtr<FAdaptiveOctreeNode> Node, TArray<FCornerSample>& Samples, TMap<FIntVector, int32>& InCornerMap, double QuantizeGrid, FVector EditCenter, double SearchRadius)
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
        FVector Pos = CornerPool[Node->Corners[i]].Position;
        FIntVector Key(
            FMath::RoundToInt(Pos.X / QuantizeGrid),
            FMath::RoundToInt(Pos.Y / QuantizeGrid),
            FMath::RoundToInt(Pos.Z / QuantizeGrid));

        int32* Existing = InCornerMap.Find(Key);
        if (Existing)
        {
            Samples[*Existing].Targets.Add(&CornerPool[Node->Corners[i]].Density);
        }
        else
        {
            int32 NewIdx = Samples.Num();
            FCornerSample Sample;
            Sample.Position = Pos;
            Sample.Targets.Add(&CornerPool[Node->Corners[i]].Density);
            Samples.Add(Sample);
            InCornerMap.Add(Key, NewIdx);
        }
    }

    if (Node->IsLeaf()) return;

    for (int i = 0; i < 8; i++)
        GatherUniqueCorners(Node->Children[i], Samples, InCornerMap, QuantizeGrid, EditCenter, SearchRadius);
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
    Node->FinalizeFromExistingCorners(this);
    Node->ComputeNormalizedPosition(Root->Extent * .9);
}

void FAdaptiveOctree::ReconstructSubtree(TSharedPtr<FAdaptiveOctreeNode> Node, FVector EditCenter, double SearchRadius)
{
    // 1. Gather unique corners from affected nodes
    TMap<FIntVector, int32> TempCornerMap;
    TArray<FCornerSample> Samples;
    Samples.Reserve(4096);
    TempCornerMap.Reserve(4096);
    double QuantizeGrid = ChunkExtent * 1e-8;

    GatherUniqueCorners(Node, Samples, TempCornerMap, QuantizeGrid, EditCenter, SearchRadius);

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
    if (!InChunk.IsValid())
    {
        return;
    }

    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;

    TArray<FIndex3UI> SrfTriangles;
    TArray<FIndex3UI> OcnTriangles;

    VertexMap.Reserve(1024);
    UniqueVertices.Reserve(1024);
    SrfTriangles.Reserve(2048);

    double OceanRadius = RootExtent * 0.9;
    double OceanTriThreshold = -1000;


    // ============================================
    // iterate chunk edges ONLY
    // ============================================

    for (int32 EdgeIndex : InChunk->ChunkEdges)
    {
        if (!EdgePool.IsValidIndex(EdgeIndex))
        {
            continue;
        }

        FOctreeEdge& Edge = EdgePool[EdgeIndex];

        if (!Edge.bIsSignChange)
        {
            continue;
        }

        if (Edge.ConnectedNodes.Num() < 3)
        {
            continue;
        }


        // fix bowtie

        SortNodesCircular(
            Edge.ConnectedNodes,
            Edge.Axis
        );


        int32 VertexIndices[4];

        int32 NumEdgeVerts =
            FMath::Min(
                Edge.ConnectedNodes.Num(),
                4
            );


        for (int32 i = 0; i < NumEdgeVerts; i++)
        {
            TSharedPtr<FAdaptiveOctreeNode> Node =
                Edge.ConnectedNodes[i];

            if (!Node.IsValid())
            {
                continue;
            }

            FMeshVertex NewVert;

            FVector WorldPos =
                Node->DualContourPosition;

            NewVert.Position =
                WorldPos
                - InChunk->ChunkCenter;

            NewVert.OriginalPosition =
                NewVert.Position;

            NewVert.NormalizedPosition =
                Node->NormalizedPosition
                - InChunk->ChunkCenter;

            NewVert.Normal =
                Node->DualContourNormal;


            double DistFromCenter =
                FVector::Dist(
                    WorldPos,
                    Root->Center
                );


            NewVert.Depth =
                (float)(
                    OceanRadius
                    - DistFromCenter
                    );


            NewVert.Color =
                FColor::Green;


            int32* ExistingIndex =
                VertexMap.Find(
                    NewVert
                );


            if (ExistingIndex)
            {
                VertexIndices[i] =
                    *ExistingIndex;
            }
            else
            {
                int32 NewIndex =
                    UniqueVertices.Num();

                UniqueVertices.Add(
                    NewVert
                );

                VertexMap.Add(
                    NewVert,
                    NewIndex
                );

                VertexIndices[i] =
                    NewIndex;
            }
        }


        bool bReverseWinding =
            !Edge.bStartIsInside;


        if (NumEdgeVerts >= 3)
        {
            FIndex3UI tri0 =
                bReverseWinding
                ?
                FIndex3UI(
                    VertexIndices[0],
                    VertexIndices[2],
                    VertexIndices[1]
                )
                :
                FIndex3UI(
                    VertexIndices[0],
                    VertexIndices[1],
                    VertexIndices[2]
                );

            SrfTriangles.Add(tri0);

            if (
                UniqueVertices[
                    tri0.V0
                ].Depth
        >
                        OceanTriThreshold
                        )
            {
                OcnTriangles.Add(tri0);
            }


            if (NumEdgeVerts == 4)
            {
                FIndex3UI tri1 =
                    bReverseWinding
                    ?
                    FIndex3UI(
                        VertexIndices[0],
                        VertexIndices[3],
                        VertexIndices[2]
                    )
                    :
                    FIndex3UI(
                        VertexIndices[0],
                        VertexIndices[2],
                        VertexIndices[3]
                    );

                SrfTriangles.Add(tri1);

                if (
                    UniqueVertices[
                        tri1.V0
                    ].Depth
        >
                            OceanTriThreshold
                            )
                {
                    OcnTriangles.Add(tri1);
                }
            }
        }
    }


    // ============================================
    // build streams
    // ============================================

    TSharedPtr<FMeshStreamData> UpdatedSurfaceData =
        MakeShared<FMeshStreamData>();

    UpdatedSurfaceData->MeshGroupKey =
        InChunk->SurfaceMeshData->MeshGroupKey;

    UpdatedSurfaceData->MeshSectionKey =
        InChunk->SurfaceMeshData->MeshSectionKey;


    TSharedPtr<FMeshStreamData> UpdatedOceanData =
        MakeShared<FMeshStreamData>();

    UpdatedOceanData->MeshGroupKey =
        InChunk->OceanMeshData->MeshGroupKey;

    UpdatedOceanData->MeshSectionKey =
        InChunk->OceanMeshData->MeshSectionKey;


    auto SrfPositionStream =
        UpdatedSurfaceData->GetPositionStream();

    auto SrfTangentStream =
        UpdatedSurfaceData->GetTangentStream();

    auto SrfColorStream =
        UpdatedSurfaceData->GetColorStream();

    auto SrfTexCoordStream =
        UpdatedSurfaceData->GetTexCoordStream();

    auto SrfTriangleStream =
        UpdatedSurfaceData->GetTriangleStream();

    auto SrfPolygroupStream =
        UpdatedSurfaceData->GetPolygroupStream();


    auto OcnPositionStream =
        UpdatedOceanData->GetPositionStream();

    auto OcnTangentStream =
        UpdatedOceanData->GetTangentStream();

    auto OcnColorStream =
        UpdatedOceanData->GetColorStream();

    auto OcnTexCoordStream =
        UpdatedOceanData->GetTexCoordStream();

    auto OcnTriangleStream =
        UpdatedOceanData->GetTriangleStream();

    auto OcnPolygroupStream =
        UpdatedOceanData->GetPolygroupStream();


    SrfPositionStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    SrfTangentStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    SrfColorStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    SrfTexCoordStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    SrfTriangleStream.SetNumUninitialized(
        SrfTriangles.Num()
    );

    SrfPolygroupStream.SetNumUninitialized(
        SrfTriangles.Num()
    );


    OcnPositionStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    OcnTangentStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    OcnColorStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    OcnTexCoordStream.SetNumUninitialized(
        UniqueVertices.Num()
    );

    OcnTriangleStream.SetNumUninitialized(
        OcnTriangles.Num()
    );

    OcnPolygroupStream.SetNumUninitialized(
        OcnTriangles.Num()
    );


    FVector ChunkCenter =
        InChunk->ChunkCenter;


    ParallelFor(
        UniqueVertices.Num(),
        [&](int32 VertIdx)
        {
            FMeshVertex& Vertex =
                UniqueVertices[VertIdx];

            FVector WorldPos =
                FVector(
                    Vertex.OriginalPosition
                )
                + ChunkCenter;


            SrfPositionStream.Set(
                VertIdx,
                Vertex.OriginalPosition
            );


            FRealtimeMeshTangentsHighPrecision Tangent;

            Tangent.SetNormal(
                FVector3f(
                    Vertex.Normal
                )
            );

            SrfTangentStream.Set(
                VertIdx,
                Tangent
            );


            SrfColorStream.Set(
                VertIdx,
                Vertex.Color
            );


            SrfTexCoordStream.Set(
                VertIdx,
                ComputeTriplanarUV(
                    WorldPos,
                    FVector(
                        Vertex.Normal
                    )
                )
            );


            OcnPositionStream.Set(
                VertIdx,
                Vertex.NormalizedPosition
            );


            FVector OcnNormal =
                (
                    FVector(
                        Vertex.NormalizedPosition
                    )
                    + ChunkCenter
                    - Root->Center
                    ).GetSafeNormal();


            FRealtimeMeshTangentsHighPrecision OcnTangent;

            OcnTangent.SetNormal(
                FVector3f(
                    OcnNormal
                )
            );


            OcnTangentStream.Set(
                VertIdx,
                OcnTangent
            );


            float absDepth =
                FMath::Max(
                    Vertex.Depth,
                    0.0f
                );


            uint32 intDepth =
                (uint32)
                FMath::Min(
                    absDepth * 1000.0f,
                    4294967295.0f
                );


            OcnColorStream.Set(
                VertIdx,
                FColor(
                    (intDepth >> 24) & 0xFF,
                    (intDepth >> 16) & 0xFF,
                    (intDepth >> 8) & 0xFF,
                    intDepth & 0xFF
                )
            );


            OcnTexCoordStream.Set(
                VertIdx,
                ComputeTriplanarUV(
                    FVector(
                        Vertex.NormalizedPosition
                    )
                    + ChunkCenter,
                    OcnNormal
                )
            );
        }
    );


    for (int32 i = 0; i < SrfTriangles.Num(); i++)
    {
        SrfTriangleStream.Set(
            i,
            SrfTriangles[i]
        );

        SrfPolygroupStream.Set(
            i,
            0
        );
    }


    for (int32 i = 0; i < OcnTriangles.Num(); i++)
    {
        OcnTriangleStream.Set(
            i,
            OcnTriangles[i]
        );

        OcnPolygroupStream.Set(
            i,
            0
        );
    }


    InChunk->SurfaceMeshData =
        UpdatedSurfaceData;

    InChunk->OceanMeshData =
        UpdatedOceanData;

    InChunk->IsDirty =
        (
            SrfTriangles.Num()
    > 0
            );
}

void FAdaptiveOctree::RegisterNodeEdges(TSharedPtr<FAdaptiveOctreeNode> Node)
{
    if (!Node.IsValid() || !Node->IsLeaf()) return;

    // Use your existing GetSignChangeEdges() but immediately process them
    TArray<FNodeEdge> NodeEdges = Node->GetSignChangeEdges();

    for (const FNodeEdge& Edge : NodeEdges)
    {
        // This calls the AcquireEdge function already in your file
        AcquireEdge(Edge.Corners[0], Edge.Corners[1], Node);
    }
}
void FAdaptiveOctree::UpdateLodRecursive(TSharedPtr<FAdaptiveOctreeNode> Node, FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV, bool& OutChanged)
{
    if (Node->IsLeaf())
    {
        if (Node->ShouldSplit(CameraPosition, InScreenSpaceThreshold, InCameraFOV))
        {
            OutChanged = true;
            Node->Split(this);
            for (int i = 0; i < 8; i++)
            {
                ComputeNodeData(Node->Children[i]);
                // NEW: Register children's edges to the pool
                RegisterNodeEdges(Node->Children[i]);
            }
        }
        else if (Node->Parent.IsValid() && Node->Parent.Pin()->ShouldMerge(CameraPosition, InScreenSpaceThreshold, InCameraFOV) && Node->Index.LastChild() == 7)
        {
            OutChanged = true;
            auto ParentPtr = Node->Parent.Pin();
            ParentPtr->Merge(this);
            // NEW: Register parent's new leaf edges
            RegisterNodeEdges(ParentPtr);
        }
        else
        {
            // NEW: Stable leaf stays in the pool
            RegisterNodeEdges(Node);
        }
    }
    else
    {
        for (int i = 0; i < 8; i++)
            UpdateLodRecursive(Node->Children[i], CameraPosition, InScreenSpaceThreshold, InCameraFOV, OutChanged);
    }
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    // 1. Clear the old edge state
    {
        FScopeLock Lock(&EdgePoolLock);
        EdgePool.Empty();
        EdgeLookupMap.Empty();
    }

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    ChunkMap.GenerateKeyArray(Chunks);

    // 2. Parallel Registration
    ParallelFor(Chunks.Num(), [&](int32 idx)
        {
            bool tChanged = false;
            UpdateLodRecursive(Chunks[idx], CameraPosition, InScreenSpaceThreshold, InCameraFOV, tChanged);

            // Note: UpdateMeshChunkStreamData will now be called in a second pass
        });

    // 3. Rebuild Streams
    ParallelFor(Chunks.Num(), [&](int32 i)
        {
            UpdateMeshChunkStreamData(ChunkMap[Chunks[i]]);
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

void FAdaptiveOctree::ComputeNodeData(TSharedPtr<FAdaptiveOctreeNode> Node)
{
    float xPos[8], yPos[8], zPos[8], noiseOut[8];
    double dists[8];

    double NoiseScale = RootExtent * 0.1;
    FVector PlanetCenter = Root->Center;

    for (int i = 0; i < 8; i++)
    {
        FVector PlanetRel = CornerPool[Node->Corners[i]].Position - PlanetCenter;
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
        CornerPool[Node->Corners[i]].Density = dists[i] - (RootExtent * 0.9 + height) + EditStore->Sample(CornerPool[Node->Corners[i]].Position);
    }

    Node->FinalizeFromExistingCorners(this);
    Node->ComputeNormalizedPosition(Root->Extent * .9);
}

TArray<TSharedPtr<FAdaptiveOctreeNode>>
FAdaptiveOctree::SampleNodesAroundEdge(int32 EdgeIndex)
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Result;

    if (!EdgePool.IsValidIndex(EdgeIndex))
    {
        return Result;
    }

    const FOctreeEdge& Edge = EdgePool[EdgeIndex];

    for (TSharedPtr<FAdaptiveOctreeNode> NodePtr : Edge.ConnectedNodes)
    {
        if (!NodePtr.IsValid())
        {
            continue;
        }

        TSharedPtr<FAdaptiveOctreeNode> Leaf =
            NodePtr->GetLeafForEdge(EdgeIndex, this);

        if (Leaf.IsValid())
        {
            Result.AddUnique(Leaf);
        }
    }

    return Result;
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