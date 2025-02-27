#include "FAdaptiveOctree.h"

// Constructor
FAdaptiveOctree::FAdaptiveOctree(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InRootExtent, int ChunkDepth, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    RootExtent = InRootExtent;
    Root = MakeShared<FAdaptiveOctreeNode>(InDensityFunction, InCenter, InRootExtent, InMinDepth, InMaxDepth);
    SplitToDepth(Root, ChunkDepth);
    ChunkNodes = GetSurfaceNodes();
}

void FAdaptiveOctree::InitializeChunks(ARealtimeMeshActor* InParentActor, UMaterialInterface* InMaterial) {
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    Chunks.Empty();
    for (auto aChunk : ChunkNodes) {
        FMeshChunk NewChunk;
        NewChunk.Initialize(InParentActor, InMaterial, aChunk, AsShared());
        Chunks.Add(NewChunk);
    }
    ChunksInitialized = true;
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


bool FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double LodFactor)
{
    if (!ChunksInitialized) return false;
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    bool wasUpdated = false;
    if(Root)
    ParallelFor(ChunkNodes.Num(), [&](int32 idx)
    {
        TArray<FNodeEdge> OutChunkEdges;
        if (ChunkNodes[idx]->UpdateLod(CameraPosition, LodFactor, OutChunkEdges)) {
            wasUpdated = true;
            Chunks[idx].ChunkEdges = OutChunkEdges;
            Chunks[idx].IsDirty = true;
        }
    });
    return wasUpdated;
}
void FAdaptiveOctree::UpdateMesh() {
    if (!ChunksInitialized) return;
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    ParallelFor(Chunks.Num(), [&](int32 idx)
    {
        if (Chunks[idx].IsDirty) {
            Chunks[idx].UpdateMeshData();
            Chunks[idx].UpdateComponent();
        }
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

TArray<FMeshChunk> FAdaptiveOctree::GetChunks()
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    return Chunks;
}

TArray<FAdaptiveOctreeFlatNode> FAdaptiveOctree::SampleSurfaceNodesAroundEdge(const FNodeEdge& Edge)
{
    FRWScopeLock ReadLock(OctreeLock, SLT_ReadOnly);
    TArray<FAdaptiveOctreeFlatNode> SurfaceNodes;
    double SmallOffset = 0.001;// *(Edge.Corners[0].Position - Edge.Corners[1].Position).Size();

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

    return  FAdaptiveOctreeFlatNode();
}

// Clears the entire octree
void FAdaptiveOctree::Clear()
{
    FRWScopeLock WriteLock(OctreeLock, SLT_Write);
    Root.Reset();
}
