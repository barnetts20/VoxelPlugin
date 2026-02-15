#include "FSparseOctree.h"

FSparseOctree::FSparseOctree() : Precision(1), MaxDepth(62), MaxExtent(FInt64Coordinate::MaxCoord){
    Root = MakeShared<FSparseOctreeNode>();
}

void FSparseOctree::SetMaxContainedScale(double InMaxContainedSize)
{
    if (InMaxContainedSize <= 0)
    {
        MaxContainedScale = 0;
        MaxExtent = FInt64Coordinate::MaxCoord;
        MaxDepth = 62;
        return;
    }

    // Store the user-defined max contained world scale
    MaxContainedScale = InMaxContainedSize;

    // Convert to internal coordinate scale (uint64 units)
    uint64 InternalSize = static_cast<uint64>(MaxContainedScale / GetPrecision());
    uint64 NewExtent = FMath::RoundUpToPowerOfTwo(InternalSize);

    // Update the octree's internal max extent
    MaxExtent = NewExtent;
    MaxDepth = static_cast<int32>(FMath::Log2(static_cast<double>(MaxExtent))) - 1;
    MaxDepth = FMath::Clamp(MaxDepth, 0, MAXIMUM_POSSIBLE_DEPTH);

    UE_LOG(LogTemp, Log, TEXT("SetMaxContainedScale: MaxContainedScale = %f, InternalSize = %llu, MaxExtent = %lld, MaxDepth = %d, Precision = %f"),
        MaxContainedScale, InternalSize, MaxExtent, MaxDepth, GetPrecision());
}
double FSparseOctree::GetMaxContainedScale() {
    return MaxContainedScale;
}
void FSparseOctree::SetPrecision(double InPrecision)
{
    if (InPrecision <= 0)
    {
        UE_LOG(LogTemp, Error, TEXT("SetPrecision failed: Precision must be greater than zero."));
        return;
    }

    // Store precision internally as half of user input
    Precision = InPrecision * 0.5;

    // If `MaxContainedScale` is set, recalculate `MaxExtent` and `MaxDepth`
    if (MaxContainedScale > 0)
    {
        SetMaxContainedScale(MaxContainedScale); // Reapply scaling constraints
    }

    UE_LOG(LogTemp, Log, TEXT("SetPrecision: New Precision = %f, Internal Precision = %f, MaxExtent = %lld, MaxDepth = %d"),
        GetPrecision(), GetPrecision(), MaxExtent, MaxDepth);
}
double FSparseOctree::GetPrecision() {
    return Precision * 2;
}
double FSparseOctree::GetContainerWorldScale()
{
    return MaxExtent * 2 * Precision;
}
int64 FSparseOctree::GetMaxExtent()
{
    return MaxExtent;
}
int FSparseOctree::GetMaxDepth()
{
    return MaxDepth;
}

bool FSparseOctree::ValidatePosition(FInt64Coordinate InCoord)
{
    return FMath::Abs(InCoord.X) <= MaxExtent && FMath::Abs(InCoord.Y) <= MaxExtent && FMath::Abs(InCoord.Z) <= MaxExtent;
}
int FSparseOctree::ValidateDepth(int InDepth)
{
    return FMath::Clamp(InDepth, 0, MaxDepth);
}
TArray<uint8> FSparseOctree::ValidateTreeIndex(TArray<uint8> InTreeIndex)
{
    if (InTreeIndex.Num() > MaxDepth)
    {
        InTreeIndex.SetNum(MaxDepth); // Trim excess indices to fit within `MaxDepth`
    }
    return MoveTemp(InTreeIndex);
}

void FSparseOctree::SetOctreeOffset(FVector WorldOffset)
{
    OctreeOffset = FInt64Coordinate::ToInt64Position(WorldOffset, GetPrecision());
}
void FSparseOctree::SetOctreeOffset(FInt64Coordinate InternalOffset)
{
    OctreeOffset = InternalOffset;
}
FInt64Coordinate FSparseOctree::GetOctreeOffsetInternal() {
    return OctreeOffset;
}
FVector FSparseOctree::GetOctreeOffsetWorld() {
    return FInt64Coordinate::ToWorldPosition(OctreeOffset, GetPrecision());
}

FVector FSparseOctree::ConvertToWorldPosition(FInt64Coordinate InternalPosition) {
    return FInt64Coordinate::ToWorldPosition(FInt64Coordinate::FromDoubleVector(InternalPosition.ToDoubleVector() + OctreeOffset.ToDoubleVector()), GetPrecision());
}
FInt64Coordinate FSparseOctree::ConvertToInternalPosition(FVector WorldPosition) {
    return FInt64Coordinate::FromDoubleVector(FInt64Coordinate::ToInt64Position(WorldPosition, GetPrecision()).ToDoubleVector() - OctreeOffset.ToDoubleVector());
}
double FSparseOctree::ConvertToWorldDistance(uint64 InternalDistance)
{
    return static_cast<double>(InternalDistance) * Precision;
}
double FSparseOctree::GetWorldHalfScaleForDepth(int depth)
{
    return static_cast<double>(GetInternalHalfScaleForDepth(depth)) * Precision;
}
int64 FSparseOctree::GetInternalHalfScaleForDepth(int depth)
{
    depth = FMath::Clamp(depth, 0, MAXIMUM_POSSIBLE_DEPTH);
    return FInt64Coordinate::MaxCoord >> depth;;
}
uint64 FSparseOctree::ConvertToInternalDistance(double InternalDistance)
{
    return static_cast<double>(InternalDistance) / Precision;
}
uint8 FSparseOctree::GetChildIndexForPosition(TSharedPtr<FSparseOctreeNode> Node, FInt64Coordinate Position) const
{
    FInt64Coordinate Center = Node->GetCenter();

    return (Position.X >= Center.X ? 1 : 0) |
        (Position.Y >= Center.Y ? 2 : 0) |
        (Position.Z >= Center.Z ? 4 : 0);
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::PopulateSphereInOctree(FVector Center, int32 TargetDepth, double SphereRadiusWorld, TSharedPtr<FVoxelData> Data)
{
    TArray<TSharedPtr<FSparseOctreeNode>> InsertedNodes; // Store inserted nodes

    // Convert sphere center from world to internal space
    FInt64Coordinate SphereCenter = ConvertToInternalPosition(Center);

    // Convert sphere radius to octree's integer scale
    double SphereRadius = SphereRadiusWorld / GetPrecision();

    struct NodeEntry
    {
        TSharedPtr<FSparseOctreeNode> Node;
        double NodeRadius;
        double DistanceToCenter;
    };

    TArray<NodeEntry> NodeStack;
    NodeStack.Add({ Root, sqrt(3.0) * Root->GetHalfScale(), FInt64Coordinate::DDist(Root->GetCenter(), SphereCenter) });

    while (NodeStack.Num() > 0)
    {
        NodeEntry Entry = NodeStack.Pop();

        if (!Entry.Node.IsValid()) continue;

        double HalfSize = Entry.Node->GetHalfScale();

        // Skip entirely outside nodes
        if (Entry.DistanceToCenter - Entry.NodeRadius > SphereRadius)
        {
            continue;
        }

        // Fully contained? Just set voxel data and store the node
        if (Entry.DistanceToCenter + Entry.NodeRadius <= SphereRadius || Entry.Node->GetHalfScale() <= (Root->GetHalfScale() >> TargetDepth))
        {
            Entry.Node->SetPayload(Data);
            InsertedNodes.Add(Entry.Node); // Store inserted node
            continue;
        }

        // Otherwise, we need to subdivide
        for (uint8 ChildIndex = 0; ChildIndex < 8; ++ChildIndex)
        {
            TSharedPtr<FSparseOctreeNode> Child = Entry.Node->InsertChild(ChildIndex);
            if (Child.IsValid())
            {
                FInt64Coordinate ChildCenter = Child->GetCenter();
                double ChildRadius = sqrt(3.0) * Child->GetHalfScale();
                double ChildDistance = FInt64Coordinate::DDist(ChildCenter, SphereCenter);

                NodeStack.Add({ Child, ChildRadius, ChildDistance });
            }
        }
    }

    return InsertedNodes; // Return all inserted nodes
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::PopulatePointsInOctree()
{
    TArray<TSharedPtr<FSparseOctreeNode>> InsertedNodes; // Store inserted nodes
    TArray<FVector> Positions;
    TArray<TSharedPtr<FVoxelData>> Data;
    TArray<int32> Depths;
    double scale = 100000000;
    FRandomStream rStream = FRandomStream(1);
    for (int i = 0; i < 1000; i++) {
        Positions.Add(rStream.GetUnitVector() * rStream.FRand() * scale);
        Depths.Add(MaxDepth);
        Data.Add(MakeShared<FVoxelData>(1, 1));
    }

    return BulkInsertPointData(Positions,Depths,Data); // Return all inserted nodes
}
//Insert Methods
TSharedPtr<FSparseOctreeNode> FSparseOctree::InsertData(TArray<uint8> TreeIndex, TSharedPtr<FVoxelData> Data)
{
    TreeIndex = ValidateTreeIndex(TreeIndex);
    //Root Node case
    if (TreeIndex.Num() == 0) {
        if (Data)
            Data->Position = Root->GetCenter();
        Root->SetPayload(Data);
        return Root;
    }
    TSharedPtr<FSparseOctreeNode> CurrentNode = Root;
    for (uint8 index : TreeIndex) {
        TSharedPtr<FSparseOctreeNode> NextNode = CurrentNode->GetChildren()[index];
        if (!NextNode) {
            NextNode = CurrentNode->InsertChild(index);
        }
        CurrentNode = NextNode;
    }
    if (Data)
        Data->Position = CurrentNode->GetCenter();
    CurrentNode->SetPayload(Data);
    return CurrentNode;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkInsertData(TArray<TArray<uint8>> TreeIndicies, TArray<TSharedPtr<FVoxelData>> Data)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ReturnNodes;
    for (int i = 0; i < TreeIndicies.Num(); i++) {

        ReturnNodes.Add(InsertData(TreeIndicies[i], Data[i]));
    }
    return ReturnNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkInsertData(TArray<TArray<uint8>> TreeIndicies, TSharedPtr<FVoxelData> Data)
{
    return TArray<TSharedPtr<FSparseOctreeNode>>();
}
TSharedPtr<FSparseOctreeNode> FSparseOctree::InsertDataAtPoint(FVector WorldPosition, int32 Depth, TSharedPtr<FVoxelData> Data)
{
    FInt64Coordinate InternalPosition = ConvertToInternalPosition(WorldPosition);
    return InsertDataAtPoint(InternalPosition, Depth, Data);
}
TSharedPtr<FSparseOctreeNode> FSparseOctree::InsertDataAtPoint(FInt64Coordinate InternalPosition, int32 Depth, TSharedPtr<FVoxelData> Data)
{
    // Ensure the Octree and Root exist
    if (!Root.IsValid() || !ValidatePosition(InternalPosition))
    {
        return nullptr;
    }

    Depth = ValidateDepth(Depth);

    if (Data)
        Data->Position = InternalPosition;
    // Start from root and traverse down to the correct depth
    TSharedPtr<FSparseOctreeNode> CurrentNode = Root;
    for (int32 CurrentDepth = 0; CurrentDepth < Depth; CurrentDepth++)
    {
        uint8 ChildIndex = GetChildIndexForPosition(CurrentNode, InternalPosition);

        TSharedPtr<FSparseOctreeNode> NextNode = CurrentNode->GetChildren()[ChildIndex];
        if (!NextNode)
        {
            NextNode = CurrentNode->InsertChild(ChildIndex);
        }
        CurrentNode = NextNode;
    }
    CurrentNode->SetPayload(Data);
    return CurrentNode;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkInsertPointData(TArray<FVector> Positions, TArray<int32> Depths, TArray<TSharedPtr<FVoxelData>> Data)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ReturnNodes;

    for (int32 i = 0; i < Positions.Num(); i++)
    {
        FInt64Coordinate InternalPos = ConvertToInternalPosition(Positions[i]);
        ReturnNodes.Add(InsertDataAtPoint(InternalPos, Depths[i], Data[i]));
    }

    return ReturnNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkInsertPointData(TArray<FInt64Coordinate> Positions, TArray<int32> Depths, TArray<TSharedPtr<FVoxelData>> Data)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ReturnNodes;

    for (int32 i = 0; i < Positions.Num(); i++)
    {
        ReturnNodes.Add(InsertDataAtPoint(Positions[i], Depths[i], Data[i]));
    }

    return ReturnNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkInsertPointData(TArray<FVector> Positions, int32 Depth, TSharedPtr<FVoxelData> Data)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ReturnNodes;

    for (int32 i = 0; i < Positions.Num(); i++)
    {
        FInt64Coordinate InternalPos = ConvertToInternalPosition(Positions[i]);
        ReturnNodes.Add(InsertDataAtPoint(InternalPos, Depth, Data));
    }

    return ReturnNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkInsertPointData(TArray<FInt64Coordinate> Positions, int32 Depth, TSharedPtr<FVoxelData> Data)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ReturnNodes;

    for (int32 i = 0; i < Positions.Num(); i++)
    {
        ReturnNodes.Add(InsertDataAtPoint(Positions[i], Depth, Data));
    }

    return ReturnNodes;
}
//Fetch Methods
TSharedPtr<FSparseOctreeNode> FSparseOctree::GetNode(TArray<uint8> TreeIndex) {
    TreeIndex = ValidateTreeIndex(TreeIndex);
    if (TreeIndex.Num() == 0) {
        return Root;
    }
    TSharedPtr<FSparseOctreeNode> CurrentNode = Root;
    for (uint8 index : TreeIndex) {
        TSharedPtr<FSparseOctreeNode> NextNode = CurrentNode->GetChildren()[index];
        if (!NextNode) {
            return nullptr;
        }
        CurrentNode = NextNode;
    }
    return CurrentNode;
}
TSharedPtr<FSparseOctreeNode> FSparseOctree::GetRootNode()
{
    return Root;
}
TSharedPtr<FSparseOctreeNode> FSparseOctree::GetNodeContainingPointAtDepth(FVector WorldPosition, int32 Depth)
{
    FInt64Coordinate InternalPosition = ConvertToInternalPosition(WorldPosition);
    return GetNodeContainingPointAtDepth(InternalPosition, Depth);
}
TSharedPtr<FSparseOctreeNode> FSparseOctree::GetNodeContainingPointAtDepth(FInt64Coordinate InternalPosition, int32 Depth)
{
    // Ensure the Octree and Root exist
    if (!Root.IsValid() || !ValidatePosition(InternalPosition))
    {
        return nullptr;
    }
    
    Depth = ValidateDepth(Depth);

    // Start from root and traverse down to the correct depth
    TSharedPtr<FSparseOctreeNode> CurrentNode = Root;
    for (int32 CurrentDepth = 0; CurrentDepth < Depth; CurrentDepth++)
    {
        uint8 ChildIndex = GetChildIndexForPosition(CurrentNode, InternalPosition);

        TSharedPtr<FSparseOctreeNode> NextNode = CurrentNode->GetChildren()[ChildIndex];
        if (!NextNode)
        {
            return nullptr; // Node does not exist at this depth
        }
        CurrentNode = NextNode;
    }

    return CurrentNode;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllNodes(TSharedPtr<FSparseOctreeNode> InNode, bool bOnlyOccupied)
{
    TArray<TSharedPtr<FSparseOctreeNode>> Nodes;

    if (!InNode.IsValid())
    {
        return Nodes;
    }

    if (!bOnlyOccupied || InNode->HasPayload())
    {
        Nodes.Add(InNode);
    }

    for (const TSharedPtr<FSparseOctreeNode>& Child : InNode->GetChildren())
    {
        if (Child.IsValid())
        {
            TArray<TSharedPtr<FSparseOctreeNode>> ChildNodes = GetAllNodes(Child, bOnlyOccupied);
            Nodes.Append(ChildNodes);
        }
    }

    return Nodes;
}

TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkGetNodes(TArray<TArray<uint8>> TreeIndices, bool bOnlyOccupied)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ResultNodes;

    for (const TArray<uint8>& Index : TreeIndices)
    {
        TSharedPtr<FSparseOctreeNode> Node = GetNode(Index);
        if (Node.IsValid() && (!bOnlyOccupied || Node->HasPayload()))
        {
            ResultNodes.Add(Node);
        }
    }

    return ResultNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetNodesContainingPoint(FVector WorldPosition, bool bOnlyOccupied)
{
    return GetNodesContainingPoint(ConvertToInternalPosition(WorldPosition), bOnlyOccupied);
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetNodesContainingPoint(FInt64Coordinate InternalPosition, bool bOnlyOccupied)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ContainingNodes;

    if (!Root.IsValid() || !ValidatePosition(InternalPosition))
    {
        return ContainingNodes;
    }

    TSharedPtr<FSparseOctreeNode> CurrentNode = Root;
    while (CurrentNode.IsValid())
    {
        if (!bOnlyOccupied || CurrentNode->HasPayload())
        {
            ContainingNodes.Add(CurrentNode);
        }

        uint8 ChildIndex = GetChildIndexForPosition(CurrentNode, InternalPosition);
        TSharedPtr<FSparseOctreeNode> NextNode = CurrentNode->GetChildren()[ChildIndex];

        if (!NextNode.IsValid())
        {
            break;
        }

        CurrentNode = NextNode;
    }

    return ContainingNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkGetNodeContainingPointAtDepth(TArray<FVector> WorldPositions, TArray<int32> Depths, bool bOnlyOccupied)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ResultNodes;

    for (int32 i = 0; i < WorldPositions.Num(); i++)
    {
        TSharedPtr<FSparseOctreeNode> Node = GetNodeContainingPointAtDepth(ConvertToInternalPosition(WorldPositions[i]), Depths[i]);
        if (Node.IsValid() && (!bOnlyOccupied || Node->HasPayload()))
        {
            ResultNodes.Add(Node);
        }
    }

    return ResultNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::BulkGetNodeContainingPointAtDepth(TArray<FInt64Coordinate> InternalCoordinates, TArray<int32> Depths, bool bOnlyOccupied)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ResultNodes;

    for (int32 i = 0; i < InternalCoordinates.Num(); i++)
    {
        TSharedPtr<FSparseOctreeNode> Node = GetNodeContainingPointAtDepth(InternalCoordinates[i], Depths[i]);
        if (Node.IsValid() && (!bOnlyOccupied || Node->HasPayload()))
        {
            ResultNodes.Add(Node);
        }
    }

    return ResultNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllNodesInRadius(FVector WorldPosition, double Range, bool bOnlyOccupied)
{
    FInt64Coordinate InternalPos = ConvertToInternalPosition(WorldPosition);
    return GetAllNodesInRadius(InternalPos, static_cast<uint64>(Range / GetPrecision() + .1), bOnlyOccupied);
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllNodesInRadius(FInt64Coordinate InternalPosition, uint64 Range, bool bOnlyOccupied)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ResultNodes;
    if (!Root.IsValid() || !ValidatePosition(InternalPosition))
    {
        return ResultNodes;
    }

    TQueue<TSharedPtr<FSparseOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(Root);

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FSparseOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode.IsValid())
        {
            continue;
        }

        // Compute internal distance without offset artifacts
        double Distance = FVector::Dist(InternalPosition.ToDoubleVector(), CurrentNode->GetCenter().ToDoubleVector());

        if (Distance <= static_cast<double>(Range))
        {
            if (!bOnlyOccupied || CurrentNode->HasPayload())
            {
                ResultNodes.Add(CurrentNode);
            }
        }

        for (TSharedPtr<FSparseOctreeNode> Child : CurrentNode->GetChildren())
        {
            if (Child.IsValid())
            {
                NodeQueue.Enqueue(Child);
            }
        }
    }

    return ResultNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllNodesInRadiusAtDepth(FVector WorldPosition, double Range, int32 Depth, bool bOnlyOccupied)
{
    FInt64Coordinate InternalPos = ConvertToInternalPosition(WorldPosition);
    return GetAllNodesInRadiusAtDepth(InternalPos, static_cast<uint64>(Range / GetPrecision()), Depth, bOnlyOccupied);
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllNodesInRadiusAtDepth(FInt64Coordinate InternalPosition, uint64 Range, int32 Depth, bool bOnlyOccupied)
{
    TArray<TSharedPtr<FSparseOctreeNode>> ResultNodes;
    if (!Root.IsValid() || !ValidatePosition(InternalPosition))
    {
        return ResultNodes;
    }

    Depth = ValidateDepth(Depth);

    TQueue<TSharedPtr<FSparseOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(Root);

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FSparseOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode.IsValid())
        {
            continue;
        }

        // Ensure we are at the correct depth
        if (CurrentNode->GetIndex().Num() == Depth)
        {
            double Distance = FVector::Dist(InternalPosition.ToDoubleVector(), CurrentNode->GetCenter().ToDoubleVector());

            if (Distance <= static_cast<double>(Range))
            {
                if (!bOnlyOccupied || CurrentNode->HasPayload())
                {
                    ResultNodes.Add(CurrentNode);
                }
            }
            continue;
        }

        for (TSharedPtr<FSparseOctreeNode> Child : CurrentNode->GetChildren())
        {
            if (Child.IsValid())
            {
                NodeQueue.Enqueue(Child);
            }
        }
    }

    return ResultNodes;
}

TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllLeaves(TSharedPtr<FSparseOctreeNode> InNode)
{
    TArray<TSharedPtr<FSparseOctreeNode>> LeafNodes;
    if (!InNode.IsValid()) return LeafNodes;

    TQueue<TSharedPtr<FSparseOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(InNode);

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FSparseOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode.IsValid()) continue;

        // If the node has no children, it is a leaf
        if (CurrentNode->IsLeaf())
        {
            LeafNodes.Add(CurrentNode);
        }
        else
        {
            // Enqueue all valid children
            for (TSharedPtr<FSparseOctreeNode> Child : CurrentNode->GetChildren())
            {
                if (Child.IsValid())
                {
                    NodeQueue.Enqueue(Child);
                }
            }
        }
    }

    return LeafNodes;
}
TSharedPtr<FSparseOctreeNode> FSparseOctree::GetLeafAtPoint(FVector WorldPosition)
{
    FInt64Coordinate InternalPos = ConvertToInternalPosition(WorldPosition);
    return GetLeafAtPoint(InternalPos);
}
TSharedPtr<FSparseOctreeNode> FSparseOctree::GetLeafAtPoint(FInt64Coordinate InternalPosition)
{
    if (!Root.IsValid() || !ValidatePosition(InternalPosition)) return nullptr;

    TSharedPtr<FSparseOctreeNode> CurrentNode = Root;
    while (CurrentNode.IsValid() && !CurrentNode->IsLeaf())
    {
        uint8 ChildIndex = GetChildIndexForPosition(CurrentNode, InternalPosition);
        TSharedPtr<FSparseOctreeNode> NextNode = CurrentNode->GetChildren()[ChildIndex];

        if (!NextNode.IsValid()) break;
        CurrentNode = NextNode;
    }

    return CurrentNode;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetLeavesAtPoints(TArray<FVector> WorldPositions)
{
    TArray<TSharedPtr<FSparseOctreeNode>> LeafNodes;
    for (const FVector& Position : WorldPositions)
    {
        TSharedPtr<FSparseOctreeNode> LeafNode = GetLeafAtPoint(Position);
        if (LeafNode.IsValid())
        {
            LeafNodes.Add(LeafNode);
        }
    }
    return LeafNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetLeavesAtPoints(TArray<FInt64Coordinate> InternalPositions)
{
    TArray<TSharedPtr<FSparseOctreeNode>> LeafNodes;
    for (const FInt64Coordinate& Position : InternalPositions)
    {
        TSharedPtr<FSparseOctreeNode> LeafNode = GetLeafAtPoint(Position);
        if (LeafNode.IsValid())
        {
            LeafNodes.Add(LeafNode);
        }
    }
    return LeafNodes;
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllLeavesInRadius(FVector WorldPosition, double Range)
{
    FInt64Coordinate InternalPos = ConvertToInternalPosition(WorldPosition);
    return GetAllLeavesInRadius(InternalPos, static_cast<uint64>(Range / GetPrecision() + 0.1));
}
TArray<TSharedPtr<FSparseOctreeNode>> FSparseOctree::GetAllLeavesInRadius(FInt64Coordinate InternalPosition, uint64 Range)
{
    TArray<TSharedPtr<FSparseOctreeNode>> LeafNodes;
    if (!Root.IsValid() || !ValidatePosition(InternalPosition))
    {
        return LeafNodes;
    }

    TQueue<TSharedPtr<FSparseOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(Root);

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FSparseOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode.IsValid()) continue;

        double Distance = FVector::Dist(InternalPosition.ToDoubleVector(), CurrentNode->GetCenter().ToDoubleVector());

        if (Distance <= static_cast<double>(Range))
        {
            if (CurrentNode->IsLeaf())
            {
                LeafNodes.Add(CurrentNode);
            }
        }

        for (TSharedPtr<FSparseOctreeNode> Child : CurrentNode->GetChildren())
        {
            if (Child.IsValid())
            {
                NodeQueue.Enqueue(Child);
            }
        }
    }

    return LeafNodes;
}

// Delete payload data at a specific index and clean up the tree structure
void FSparseOctree::DeleteData(TArray<uint8> TreeIndex)
{
    TreeIndex = ValidateTreeIndex(TreeIndex);
    TSharedPtr<FSparseOctreeNode> NodeToDelete = GetNode(TreeIndex);
    if (NodeToDelete.IsValid() && NodeToDelete->HasPayload())
    {
        NodeToDelete->ClearPayload();
        if (NodeToDelete->IsLeaf()) {
            CleanUpTree(NodeToDelete);
        }
    }
}
void FSparseOctree::BulkDeleteData(TArray<TArray<uint8>> TreeIndices)
{
    for (const TArray<uint8>& Index : TreeIndices)
    {
        DeleteData(Index);
    }
}
void FSparseOctree::DeleteDataAtPoint(FVector WorldPosition)
{
    FInt64Coordinate InternalPosition = ConvertToInternalPosition(WorldPosition);
    DeleteDataAtPoint(InternalPosition);
}
void FSparseOctree::DeleteDataAtPoint(FInt64Coordinate InternalPosition)
{
    if (!ValidatePosition(InternalPosition)) return;
    TArray<TSharedPtr<FSparseOctreeNode>> NodesToDelete = GetNodesContainingPoint(InternalPosition, true);
    for (TSharedPtr<FSparseOctreeNode> aNode : NodesToDelete) {
        aNode->ClearPayload();
        if (aNode->IsLeaf()) {
            CleanUpTree(aNode);
        }
    }
}
void FSparseOctree::BulkDeleteDataAtPoint(TArray<FVector> WorldPositions)
{
    for (FVector Position : WorldPositions)
    {
        DeleteDataAtPoint(Position);
    }
}
void FSparseOctree::BulkDeleteDataAtPoint(TArray<FInt64Coordinate> InternalPositions)
{
    for (FInt64Coordinate Position : InternalPositions)
    {
        DeleteDataAtPoint(Position);
    }
}
void FSparseOctree::DeleteDataAtPointAndDepth(FVector WorldPosition, int32 Depth)
{
    FInt64Coordinate InternalPosition = ConvertToInternalPosition(WorldPosition);
    DeleteDataAtPointAndDepth(InternalPosition, Depth);
}
void FSparseOctree::DeleteDataAtPointAndDepth(FInt64Coordinate InternalPosition, int32 Depth)
{
    if (!ValidatePosition(InternalPosition)) return;
    Depth = ValidateDepth(Depth);
    TSharedPtr<FSparseOctreeNode> NodeToDelete = GetNodeContainingPointAtDepth(InternalPosition, Depth);
    if (NodeToDelete.IsValid() && NodeToDelete->HasPayload())
    {
        NodeToDelete->ClearPayload();
        if (NodeToDelete->IsLeaf()) {
            CleanUpTree(NodeToDelete);
        }
    }
}
void FSparseOctree::BulkDeleteAtPointAndDepth(TArray<FVector> Positions, TArray<int32> Depths)
{
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        DeleteDataAtPointAndDepth(Positions[i], Depths[i]);
    }
}
void FSparseOctree::BulkDeleteAtPointAndDepth(TArray<FInt64Coordinate> Positions, TArray<int32> Depths)
{
    for (int32 i = 0; i < Positions.Num(); ++i)
    {
        DeleteDataAtPointAndDepth(Positions[i], Depths[i]);
    }
}
void FSparseOctree::CleanUpTree(TSharedPtr<FSparseOctreeNode> Node)
{
    TSharedPtr<FSparseOctreeNode> CurrentNode = Node;

    // Traverse upwards through the tree to prune empty nodes
    while (CurrentNode.IsValid())
    {
        // If the node is a leaf and has no payload or children with payload, remove it
        if (CurrentNode->IsLeaf() && !CurrentNode->HasPayload())
        {
            TSharedPtr<FSparseOctreeNode> ParentNode = CurrentNode->GetParent();
            if (ParentNode.IsValid())
            {
                uint8 ChildIndex = GetChildIndexForPosition(ParentNode, CurrentNode->GetCenter());
                ParentNode->RemoveChild(ChildIndex);
            }
        }

        // Move up the tree
        CurrentNode = CurrentNode->GetParent();
    }
}