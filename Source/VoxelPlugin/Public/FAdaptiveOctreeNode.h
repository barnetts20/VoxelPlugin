#pragma once

#include "CoreMinimal.h"

struct VOXELPLUGIN_API FNodeCorner {
    int CornerIndex;
    FVector Position;
    double Density;
    FNodeCorner(int InIndex, FVector InPosition, double InDensity) : CornerIndex(InIndex), Position(InPosition), Density(InDensity) {};
    FNodeCorner() : CornerIndex(-1), Position(FVector::ZeroVector), Density(0) {};
};

struct VOXELPLUGIN_API FNodeEdge
{
    FNodeCorner Corners[2];     // The corners associated with this edge
    double Size;
    bool SignChange;            // Does this edge contain a sign change
    double Distance;
    FVector EdgeDirection;      // Precomputed normalized edge direction
    int Axis;                   // Axis-aligned indicator (0 = X, 1 = Y, 2 = Z)
    FVector ZeroCrossingPoint;  // Position where sign flips

    // Constructor
    FNodeEdge(FNodeCorner InCorner1, FNodeCorner InCorner2)    
    {
        Corners[0] = InCorner1;
        Corners[1] = InCorner2;
        SignChange = (InCorner1.Density < 0) != (InCorner2.Density < 0);
        // Determine which corner is positive and which is negative
        FNodeCorner PosCorner = (InCorner1.Density > InCorner2.Density) ? InCorner1 : InCorner2;
        FNodeCorner NegCorner = (InCorner1.Density > InCorner2.Density) ? InCorner2 : InCorner1;
        Distance = FVector::Dist(InCorner1.Position, InCorner2.Position);
        // Compute edge direction: Always point from positive to negative
        EdgeDirection = (NegCorner.Position - PosCorner.Position).GetSafeNormal();
        Size = FVector::Dist(Corners[0].Position, Corners[1].Position);
        Axis = FMath::Abs(InCorner1.Position.X - InCorner2.Position.X) > 0 ? 0 : (FMath::Abs(InCorner1.Position.Y - InCorner2.Position.Y) > 0 ? 1 : 2);
        if (SignChange) {
            ZeroCrossingPoint = InCorner1.Position + FMath::Abs(InCorner1.Density) / (FMath::Abs(InCorner1.Density) + FMath::Abs(InCorner2.Density)) * (InCorner2.Position - InCorner1.Position);
        }
        else {
            ZeroCrossingPoint = (InCorner1.Position + InCorner2.Position) / 2.0;
        }

    }

    bool IsCongruent(const FNodeEdge& Other) const {
        return
            Corners[0].Position.Equals(Other.Corners[0].Position, .01)
            || Corners[0].Position.Equals(Other.Corners[1].Position, .01)
            || Corners[1].Position.Equals(Other.Corners[0].Position, .01)
            || Corners[1].Position.Equals(Other.Corners[1].Position, .01)
            &&
            Axis == Other.Axis;
    }

    // Equality operator for ensuring uniqueness
    bool operator==(const FNodeEdge& Other) const
    {
        return IsCongruent(Other) && Other.EdgeDirection == EdgeDirection;
    }
};

struct VOXELPLUGIN_API FAdaptiveOctreeNode : public TSharedFromThis<FAdaptiveOctreeNode>
{
private:
    TFunction<double(FVector)> DensityFunction;
    void ComputeDualContourPosition();
    bool bIsLeaf = true;
    
    // Static arrays
    inline static const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1, 1, -1), FVector(1, 1, -1),
        FVector(-1, -1, 1), FVector(1, -1, 1),
        FVector(-1, 1, 1), FVector(1, 1, 1)
    };
    inline static const int SharedCorners[3][4] = {
        {0, 2, 4, 6}, // +X or -X aligned
        {0, 1, 4, 5}, // +Y or -Y aligned
        {0, 1, 2, 3}  // +Z or -Z aligned
    };
    inline static const int EdgePairs[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7}, // X-axis edges
        {0, 2}, {1, 3}, {4, 6}, {5, 7}, // Y-axis edges
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Z-axis edges
    };

public:
    TArray<uint8> TreeIndex;
    TWeakPtr<FAdaptiveOctreeNode> Parent;
    TSharedPtr<FAdaptiveOctreeNode> Children[8];
    int DepthBounds[2];

    FVector Center;
    double Extent;
    double Density;
    FVector DualContourPosition;
    FVector DualContourNormal;
    bool IsSurfaceNode;
    bool LodOverride = false; //If true prevent merge ops
    TArray<FNodeCorner> Corners;
    TArray<FNodeEdge> Edges;
    TArray<FNodeEdge> SignChangeEdges;

    bool IsLeaf();
    bool IsRoot();

    void Split();
    bool ShouldSplit(FVector InCameraPosition, double InLodDistanceFactor);
    void Merge();
    bool ShouldMerge(FVector InCameraPosition, double InLodDistanceFactor);    
    bool fullUpdate = true; //Force recursion until all nodes match the lod 
    void UpdateLod(FVector InCameraPosition, double InLodDistanceFactor, TArray<FNodeEdge>& OutEdges, bool& OutChanged);

    TArray<FNodeEdge> GetSurfaceEdges();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();
    TArray<FNodeEdge> GetSignChangeEdges();
    bool RefineDualContour(const FNodeEdge& InNeighborZeroCrossing);

    void DrawAndLogNode();

    // Root Constructor
    FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InExtent, int InMinDepth, int InMaxDepth);

    // Child Constructor
    FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex);
};

struct VOXELPLUGIN_API FAdaptiveOctreeFlatNode {
    // Basic node data
    TArray<uint8> TreeIndex;

    FVector Center;
    double Extent;
    double Density;

    FVector DualContourPosition;
    FVector DualContourNormal;
    TArray<FNodeCorner> Corners;
    TArray<FNodeEdge> Edges;
    TArray<FNodeEdge> SignChangeEdges;

    bool IsSurfaceNode;
    bool IsLeaf;
    bool IsValid;

    // Constructor - creates an empty/invalid flat node
    FAdaptiveOctreeFlatNode()
        : Center(FVector::ZeroVector)
        , Extent(0)
        , Density(0)
        , DualContourPosition(FVector::ZeroVector)
        , DualContourNormal(FVector::ZeroVector)
        , IsSurfaceNode(false)
        , IsLeaf(true)
        , IsValid(false)
    {};

    FAdaptiveOctreeFlatNode(TSharedPtr<FAdaptiveOctreeNode> InNode) 
        : Center(FVector::ZeroVector)
        , Extent(0)
        , Density(0)
        , DualContourPosition(FVector::ZeroVector)
        , DualContourNormal(FVector::ZeroVector)
        , IsSurfaceNode(false)
        , IsLeaf(true)
        , IsValid(InNode.IsValid())
    {
        if (IsValid) {
            TreeIndex = InNode->TreeIndex;
            Center = InNode->Center;
            Extent = InNode->Extent;
            Density = InNode->Density;
            DualContourPosition = InNode->DualContourPosition;
            DualContourNormal = InNode->DualContourNormal;
            Corners = InNode->Corners;
            Edges = InNode->Edges;
            SignChangeEdges = InNode->SignChangeEdges;
            IsSurfaceNode = InNode->IsSurfaceNode;
            IsLeaf = InNode->IsLeaf();
        }
    };

    // Equality operators for collections
    bool operator==(const FAdaptiveOctreeFlatNode& Other) const
    {
        return TreeIndex == Other.TreeIndex;
    }
};