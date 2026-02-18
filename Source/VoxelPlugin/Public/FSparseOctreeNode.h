#pragma once

#include "CoreMinimal.h"
#include <FVoxelData.h>
#include "FInt64Coordinate.h"

//TODO: REFACTOR TO DOUBLE PRECISION OR REPLACE, THIS MAY BE OVERENGINEERED
struct VOXELPLUGIN_API FSparseOctreeNode : public TSharedFromThis<FSparseOctreeNode>
{

public:
	static const int8 MAX_DEPTH = 62;
	//Parent/Root traversal
	TSharedPtr<FSparseOctreeNode> GetParent() const {
		return (Parent.IsValid() ? Parent.Pin() : nullptr);
	}

	TSharedPtr<FSparseOctreeNode> GetRoot() {
		TSharedPtr<FSparseOctreeNode> node = AsShared();
		while (node->Parent.IsValid()) {
			node = node->Parent.Pin();
		}
		return node;
	}

	int32 GetDepth() {
		return Index.Num() - 1;
	}
	//Child Operations
	TSharedPtr<FSparseOctreeNode> InsertChild(uint8 InIndex)
	{
		if (InIndex >= 8) return nullptr;
		if (!Children[InIndex].IsValid())
		{
			Children[InIndex] = MakeShared<FSparseOctreeNode>(AsShared(), InIndex);
		}
		return Children[InIndex];
	}

	void RemoveChild(uint8 InIndex)
	{
		if (InIndex >= 8 || !Children[InIndex].IsValid()) return;
		Children[InIndex]->RemoveAllChildren();
		Children[InIndex].Reset();
		Children[InIndex] = nullptr;
	}

	void RemoveAllChildren()
	{
		for (int i = 0; i < 8; i++)
		{
			if (Children[i].IsValid())
			{
				Children[i]->RemoveAllChildren();
				Children[i].Reset();
				Children[i] = nullptr;
			}
		}
	}

	const TArray<TSharedPtr<FSparseOctreeNode>>& GetChildren() const{
		return Children;
	}

	bool IsLeaf() const
	{
		for (const TSharedPtr<FSparseOctreeNode>& Child : Children)
		{
			if (Child.IsValid()) return false;
		}
		return true;
	}

	//Data Operations
	void ClearPayload() {
		Payload.Reset();
	}

	const TSharedPtr<FVoxelData> GetPayload() const {
		return Payload;
	}

	void SetPayload(TSharedPtr<FVoxelData> InPayload) {
		Payload = InPayload;
	}

	bool HasPayload() const {
		return Payload.IsValid();
	}

	const TArray<uint8>& GetIndex() const{
		return Index;
	}

	//Position
	const FInt64Coordinate GetCenter() const {
		return Center;
	}

	const int64 GetHalfScale() const {
		return HalfScale;
	}

	const bool IsPointInsideNode(FInt64Coordinate InPosition) const
	{
		return (InPosition.X >= Center.X - HalfScale && InPosition.X < Center.X + HalfScale &&
			InPosition.Y >= Center.Y - HalfScale && InPosition.Y < Center.Y + HalfScale &&
			InPosition.Z >= Center.Z - HalfScale && InPosition.Z < Center.Z + HalfScale);
	}


	//Root Constructor
	FSparseOctreeNode() : Index(), Center(0,0,0), Parent(nullptr) {
		Children.SetNum(8);
		Index.Add(0);
		HalfScale = FInt64Coordinate::MaxCoord;
	};

	FSparseOctreeNode(TSharedPtr<FSparseOctreeNode> InParent, uint8 ChildIndex) : Index(), Parent(InParent)
	{
		Children.SetNum(8);
		Index.Append(InParent->Index);
		Index.Add(ChildIndex);
		HalfScale = InParent->GetHalfScale() / 2;

		FInt64Coordinate offsetVector = FInt64Coordinate(
			(ChildIndex & 1) ? HalfScale : -HalfScale, //X
			(ChildIndex & 2) ? HalfScale : -HalfScale, //Y
			(ChildIndex & 4) ? HalfScale : -HalfScale  //Z
		);
		Center = InParent->GetCenter() + offsetVector;
	};

	~FSparseOctreeNode() {};

private:
	TArray<uint8> Index;
	FInt64Coordinate Center;
	int64 HalfScale;
	TWeakPtr<FSparseOctreeNode> Parent;
	TArray<TSharedPtr<FSparseOctreeNode>> Children;
	TSharedPtr<FVoxelData> Payload;
};
