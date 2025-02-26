#pragma once

#include "CoreMinimal.h"
#include "FSparseOctreeNode.h"
#include "FInt64Coordinate.h"

struct VOXELPLUGIN_API FSparseOctree : public TSharedFromThis<FSparseOctree>
{
public:
	TSharedPtr<FSparseOctreeNode> Root;

	FSparseOctree();

	static const int MAXIMUM_POSSIBLE_DEPTH = 62;

	void SetMaxContainedScale(double InMaxContainedScale); //Scales the MaxExtent and MaxDepth such that the octree is the minimum bounded size in powers of 2 that can contain the provided world scale
	double GetMaxContainedScale();// Returns the current MaxContainerScale value
	void SetPrecision(double InPrecision); //Set precision, if max contained scale is set, recalculate max extent, max depth
	double GetPrecision(); //Returns the precision
	double GetContainerWorldScale();//Returns the size in world scale of the octree container
	int64 GetMaxExtent(); //Gets the max extent, together with the origin offset defines the valid coordinate range for inserts
	int GetMaxDepth(); //Gets the depth at which Node centers have an integer size of 2 on all axes, corresponds to node centers being spaced 2*Precision world space units apart

	bool ValidatePosition(FInt64Coordinate InCoord);
	int ValidateDepth(int InDepth);
	TArray<uint8> ValidateTreeIndex(TArray<uint8> InTreeIndex);

	void SetOctreeOffset(FVector WorldOffset);
	void SetOctreeOffset(FInt64Coordinate InternalOffset);
	FInt64Coordinate GetOctreeOffsetInternal();
	FVector GetOctreeOffsetWorld();
	FVector ConvertToWorldPosition(FInt64Coordinate InternalPosition);
	FInt64Coordinate ConvertToInternalPosition(FVector WorldPosition);
	uint64 ConvertToInternalDistance(double WorldDistance);
	double ConvertToWorldDistance(uint64 InternalDistance);
	double GetWorldHalfScaleForDepth(int depth);
	int64 GetInternalHalfScaleForDepth(int depth);
	uint8 GetChildIndexForPosition(TSharedPtr<FSparseOctreeNode> Node, FInt64Coordinate Position) const;

	//Tree Modification methods 
	TSharedPtr<FSparseOctreeNode> InsertData(TArray<uint8> TreeIndex, TSharedPtr<FVoxelData> Data);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkInsertData(TArray<TArray<uint8>> TreeIndicies, TArray<TSharedPtr<FVoxelData>> Data);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkInsertData(TArray<TArray<uint8>> TreeIndicies, TSharedPtr<FVoxelData> Data);
	TSharedPtr<FSparseOctreeNode> InsertDataAtPoint(FVector WorldPosition, int32 Depth, TSharedPtr<FVoxelData> Data);
	TSharedPtr<FSparseOctreeNode> InsertDataAtPoint(FInt64Coordinate InternalPosition, int32 Depth, TSharedPtr<FVoxelData> Data);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkInsertPointData(TArray<FVector> Positions, TArray<int32> Depths, TArray<TSharedPtr<FVoxelData>> Data);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkInsertPointData(TArray<FInt64Coordinate> Positions, TArray<int32> Depths, TArray<TSharedPtr<FVoxelData>> Data);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkInsertPointData(TArray<FVector> Positions, int32 Depth, TSharedPtr<FVoxelData> Data);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkInsertPointData(TArray<FInt64Coordinate> Positions, int32 Depth, TSharedPtr<FVoxelData> Data);

	//Complex Insertion Examples
	TArray<TSharedPtr<FSparseOctreeNode>> PopulateSphereInOctree(FVector Center, int32 TargetDepth, double SphereRadius, TSharedPtr<FVoxelData> Data);

	//Basic Fetching
	TSharedPtr<FSparseOctreeNode> GetRootNode();
	TSharedPtr<FSparseOctreeNode> GetNode(TArray<uint8> TreeIndex); //Return the node at the given index, assuming it exists
	TSharedPtr<FSparseOctreeNode> GetNodeContainingPointAtDepth(FVector WorldPosition, int32 Depth); //Return the node at a given depth containing the given point - Decorator
	TSharedPtr<FSparseOctreeNode> GetNodeContainingPointAtDepth(FInt64Coordinate InternalCoordinate, int32 Depth); //Return the node at a given depth containing the given point
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllNodes(TSharedPtr<FSparseOctreeNode> InNode, bool bOnlyOccupied = false);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkGetNodes(TArray<TArray<uint8>> TreeIndices, bool bOnlyOccupied = false);
	TArray<TSharedPtr<FSparseOctreeNode>> GetNodesContainingPoint(FVector WorldPosition, bool bOnlyOccupied = false);
	TArray<TSharedPtr<FSparseOctreeNode>> GetNodesContainingPoint(FInt64Coordinate WorldPosition, bool bOnlyOccupied = false);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkGetNodeContainingPointAtDepth(TArray<FVector> WorldPositions, TArray<int32> Depths, bool bOnlyOccupied = false);
	TArray<TSharedPtr<FSparseOctreeNode>> BulkGetNodeContainingPointAtDepth(TArray<FInt64Coordinate> InternalCoordinates, TArray<int32> Depths, bool bOnlyOccupied = false);
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllNodesInRadius(FVector WorldPosition, double Range, bool bOnlyOccupied = false); //Get all nodes in range of point - Decorator
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllNodesInRadius(FInt64Coordinate InternalPosition, uint64 Range, bool bOnlyOccupied = false); //Get all nodes in range of point
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllNodesInRadiusAtDepth(FVector WorldPosition, double Range, int32 Depth, bool bOnlyOccupied = false); //Get all nodes in range of point at the specified depth - Decorator
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllNodesInRadiusAtDepth(FInt64Coordinate InternalPosition, uint64 Range, int32 Depth, bool bOnlyOccupied = false); //Get all nodes in range of point at the specified depth

	//Leaf Fetching
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllLeaves(TSharedPtr<FSparseOctreeNode> InNode); //Return every leaf node that includes or is a descendant of the given node
	TSharedPtr<FSparseOctreeNode> GetLeafAtPoint(FVector WorldPosition); //Get the leaf node containing the point - Decorator
	TSharedPtr<FSparseOctreeNode> GetLeafAtPoint(FInt64Coordinate InternalPosition); //Get the leaf node containing the point
	TArray<TSharedPtr<FSparseOctreeNode>> GetLeavesAtPoints(TArray<FVector> WorldPositions);//Get all leaf nodes containing any of the points - Decorator
	TArray<TSharedPtr<FSparseOctreeNode>> GetLeavesAtPoints(TArray<FInt64Coordinate> InternalPositions); //Get all leaf nodes containing any of the points
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllLeavesInRadius(FVector WorldPosition, double Range); //Return every leaf node in the given range from the given point - Decorator
	TArray<TSharedPtr<FSparseOctreeNode>> GetAllLeavesInRadius(FInt64Coordinate InternalPosition, uint64 Range); //Return every leaf node in the given range from the given point

	//Delete methods should handle recursive tree clean up as well
	void DeleteData(TArray<uint8> TreeIndex); // Deletes the payload at the given index if it exists, if a deletion occurs cleans up the tree structure
	void BulkDeleteData(TArray<TArray<uint8>> TreeIndicies); //Bulk delete by index, then clean up the tree 
	void DeleteDataAtPoint(FVector WorldPosition); // Coordinate adapter decorator
	void DeleteDataAtPoint(FInt64Coordinate InternalPosition); //Deletes the data in any node containing the Position at all depths, then clean up the tree
	void BulkDeleteDataAtPoint(TArray<FVector> WorldPositions);
	void BulkDeleteDataAtPoint(TArray<FInt64Coordinate> InternalPositions); // Bulk version
	void DeleteDataAtPointAndDepth(FVector WorldPosition, int32 Depth);
	void DeleteDataAtPointAndDepth(FInt64Coordinate InternalPosition, int32 Depth); // Deletes the data at the given depth containing the position, then clean up the tree
	void BulkDeleteAtPointAndDepth(TArray<FVector> Positions, TArray<int32> Depths);
	void BulkDeleteAtPointAndDepth(TArray<FInt64Coordinate> Positions, TArray<int32> Depths);
	void CleanUpTree(TSharedPtr<FSparseOctreeNode> Node);

private:
	double Precision = .00000005; //Our min node center precision at our deepest depth is precise to 2x the internal precision
	int MaxDepth = 62; //This is the max depth assuming we are using the full int64 coordinate space
	int64 MaxExtent = FInt64Coordinate::MaxCoord; //By default our max extent is the max side of our coordinate space
	double MaxContainedScale; //If this is unset or 0, it will be ignored. Otherwise it represents the world scale that must be containable by the tree

	FInt64Coordinate OctreeOffset;
	//The origin offset of the tree
};

