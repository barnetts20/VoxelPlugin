#include "FAdaptiveOctree.h"

// Constructor
FAdaptiveOctree::FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    RootExtent = InRootExtent;
    Root = MakeShared<FAdaptiveOctreeNode>(InDensityFunction, InCenter, InRootExtent, InMinDepth, InMaxDepth);
    SplitToDepth(Root, ChunkDepth);
    Chunks = GetSurfaceNodes();
}

void FAdaptiveOctree::InitializeMeshChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial) {
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    for (auto chunk : Chunks) {
        FMeshChunk newChunk;
        newChunk.Initialize(InParentActor, InMaterial, FAdaptiveOctreeFlatNode(chunk));
        newChunk.ChunkEdges = chunk->GetSurfaceEdges();
        UpdateMeshChunkStreamData(newChunk);
        newChunk.UpdateComponent();
        MeshChunks.Add(newChunk);
    }
    MeshChunksInitialized = true;
}

void FAdaptiveOctree::SplitToDepth(TSharedPtr<FAdaptiveOctreeNode> Node, int InMinDepth)
{
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
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

FVector2f ComputeTriplanarUV(FVector Position, FVector Normal)
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

void FAdaptiveOctree::UpdateMeshChunkStreamData(FMeshChunk& InChunk) {
    FMeshStreamData ChunkMeshData;
    auto PositionStream = ChunkMeshData.GetPositionStream();
    auto TangentStream = ChunkMeshData.GetTangentStream();
    auto ColorStream = ChunkMeshData.GetColorStream();
    auto TexCoordStream = ChunkMeshData.GetTexCoordStream();
    auto TriangleStream = ChunkMeshData.GetTriangleStream();
    auto PolygroupStream = ChunkMeshData.GetPolygroupStream();
    int idx = 0;
    int triIdx = 0;

    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    for (FNodeEdge currentEdge : InChunk.ChunkEdges){
        TArray<FAdaptiveOctreeFlatNode> nodesToMesh = SampleSurfaceNodesAroundEdge(currentEdge);
        if (!InChunk.ShouldProcessEdge(currentEdge, nodesToMesh)) continue;
        // Add first three vertices
        int32 IndexA = idx++;
        int32 IndexB = idx++;
        int32 IndexC = idx++;

        PositionStream.Add(nodesToMesh[0].DualContourPosition);
        FRealtimeMeshTangentsHighPrecision tan0;
        tan0.SetNormal(FVector3f(nodesToMesh[0].DualContourNormal));
        TangentStream.Add(tan0);
        ColorStream.Add(FColor::Green);
        TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[0].DualContourPosition, nodesToMesh[0].DualContourNormal));

        PositionStream.Add(nodesToMesh[1].DualContourPosition);
        FRealtimeMeshTangentsHighPrecision tan1;
        tan1.SetNormal(FVector3f(nodesToMesh[1].DualContourNormal));
        TangentStream.Add(tan1);
        ColorStream.Add(FColor::Green);
        TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[1].DualContourPosition, nodesToMesh[1].DualContourNormal));

        PositionStream.Add(nodesToMesh[2].DualContourPosition);
        FRealtimeMeshTangentsHighPrecision tan2;
        tan2.SetNormal(FVector3f(nodesToMesh[2].DualContourNormal));
        TangentStream.Add(tan2);
        ColorStream.Add(FColor::Green);
        TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[2].DualContourPosition, nodesToMesh[2].DualContourNormal));

        TriangleStream.Add(FIndex3UI(IndexA, IndexB, IndexC));
        PolygroupStream.Add(0);

        triIdx++;

        // Handle the fourth vertex if a quad exists
        if (nodesToMesh.Num() == 4)
        {
            int32 IndexD = idx++;

            PositionStream.Add(nodesToMesh[3].DualContourPosition);
            FRealtimeMeshTangentsHighPrecision tan3;
            tan3.SetNormal(FVector3f(nodesToMesh[3].DualContourNormal));
            TangentStream.Add(tan3);
            ColorStream.Add(FColor::Green);
            TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[3].DualContourPosition, nodesToMesh[3].DualContourNormal));

            TriangleStream.Add(FIndex3UI(IndexA, IndexC, IndexD));
            PolygroupStream.Add(0);
            triIdx++;
        }
    }
    PositionStream.SetNumUninitialized(idx);
    TangentStream.SetNumUninitialized(idx);
    ColorStream.SetNumUninitialized(idx);
    TexCoordStream.SetNumUninitialized(idx);
    TriangleStream.SetNumUninitialized(triIdx);
    PolygroupStream.SetNumUninitialized(triIdx);

    InChunk.UpdateMeshData(ChunkMeshData);
}

bool FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double LodFactor)
{
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    bool wasUpdated = false;
    if(Root)
    ParallelFor(Chunks.Num(), [&](int32 idx)
    {
        TArray<FNodeEdge> tChunkEdges;
        if (Chunks[idx]->UpdateLod(CameraPosition, LodFactor, tChunkEdges)) {
            wasUpdated = true;
            MeshChunks[idx].ChunkEdges = tChunkEdges;
        }
    });

    return wasUpdated;
}

void FAdaptiveOctree::UpdateMesh()
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    ParallelFor(MeshChunks.Num(), [&](int32 idx) {
        UpdateMeshChunkStreamData(MeshChunks[idx]);
        MeshChunks[idx].UpdateComponent();
    });
}

// Retrieves all leaf nodes
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetLeaves()
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Leaves;
    if (!Root.IsValid()) return Leaves;

    TQueue<TSharedPtr<FAdaptiveOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(Root->AsShared());

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode.IsValid()) continue;

        if (CurrentNode->IsLeaf())
        {
            Leaves.Add(CurrentNode->AsShared());
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (CurrentNode->Children[i]) NodeQueue.Enqueue(CurrentNode->Children[i]);
            }
        }
    }

    return Leaves;
}

// Retrieves all surface nodes for meshing
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetSurfaceNodes()
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    if (!Root.IsValid()) return TArray<TSharedPtr<FAdaptiveOctreeNode>>();
    return Root->GetSurfaceNodes();
}

TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctree::GetChunks()
{
    return Chunks;
}

TArray<FAdaptiveOctreeFlatNode> FAdaptiveOctree::SampleSurfaceNodesAroundEdge(const FNodeEdge& Edge)
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    TArray<FAdaptiveOctreeFlatNode> SurfaceNodes;
    double SmallOffset = 0.001;

    // Compute perpendicular vectors
    FVector Perp1 = FVector::CrossProduct(Edge.EdgeDirection, FVector(1, 0, 0)).GetSafeNormal();
    if (Perp1.IsNearlyZero()) Perp1 = FVector::CrossProduct(Edge.EdgeDirection, FVector(0, 1, 0)).GetSafeNormal();
    FVector Perp2 = FVector::CrossProduct(Edge.EdgeDirection, Perp1).GetSafeNormal();

    // Generate sample offsets
    FVector OffsetA = Edge.ZeroCrossingPoint + (Perp1 * SmallOffset) + (Perp2 * SmallOffset);
    FVector OffsetB = Edge.ZeroCrossingPoint - (Perp1 * SmallOffset) + (Perp2 * SmallOffset);
    FVector OffsetC = Edge.ZeroCrossingPoint - (Perp1 * SmallOffset) - (Perp2 * SmallOffset);
    FVector OffsetD = Edge.ZeroCrossingPoint + (Perp1 * SmallOffset) - (Perp2 * SmallOffset);

    // Query octree for valid surface nodes
    FAdaptiveOctreeFlatNode NodeA = GetSurfaceNodeByPoint(OffsetA);
    FAdaptiveOctreeFlatNode NodeB = GetSurfaceNodeByPoint(OffsetB);
    FAdaptiveOctreeFlatNode NodeC = GetSurfaceNodeByPoint(OffsetC);
    FAdaptiveOctreeFlatNode NodeD = GetSurfaceNodeByPoint(OffsetD);

    if (NodeA.IsValid) SurfaceNodes.AddUnique(NodeA);
    if (NodeB.IsValid) SurfaceNodes.AddUnique(NodeB);
    if (NodeC.IsValid) SurfaceNodes.AddUnique(NodeC);
    if (NodeD.IsValid) SurfaceNodes.AddUnique(NodeD);

    return SurfaceNodes;
}

FAdaptiveOctreeFlatNode FAdaptiveOctree::GetSurfaceNodeByPoint(FVector Position)
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    TSharedPtr<FAdaptiveOctreeNode> CurrentNode = Root;

    while (CurrentNode.IsValid())
    {
        if (CurrentNode->IsLeaf())
        {
            return CurrentNode->IsSurfaceNode ? FAdaptiveOctreeFlatNode(CurrentNode) : FAdaptiveOctreeFlatNode();
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

    return FAdaptiveOctreeFlatNode();
}

// Clears the entire octree
void FAdaptiveOctree::Clear()
{
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Root.Reset();
}
