#pragma once
#include "CoreMinimal.h"

struct VOXELPLUGIN_API FNodeCorner {
    int CornerIndex;
    FVector Position;
    double Density;
    FNodeCorner(int InIndex, FVector InPosition, double InDensity) : CornerIndex(InIndex), Position(InPosition), Density(InDensity) {};
    FNodeCorner() {};
};

struct VOXELPLUGIN_API FNodeEdge
{
    FNodeCorner Corners[2];     // The corners associated with this edge
    bool SignChange;            // Does this edge contain a sign change
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

        // Compute edge direction: Always point from positive to negative
        EdgeDirection = (NegCorner.Position - PosCorner.Position).GetSafeNormal();

        Axis = FMath::Abs(InCorner1.Position.X - InCorner2.Position.X) > 0 ? 0 : (FMath::Abs(InCorner1.Position.Y - InCorner2.Position.Y) > 0 ? 1 : 2);
        ZeroCrossingPoint = InCorner1.Position + FMath::Abs(InCorner1.Density) / (FMath::Abs(InCorner1.Density) + FMath::Abs(InCorner2.Density)) * (InCorner2.Position - InCorner1.Position);
    }

    // Equality operator for ensuring uniqueness
    bool operator==(const FNodeEdge& Other) const
    {
        return ZeroCrossingPoint.Equals(Other.ZeroCrossingPoint, KINDA_SMALL_NUMBER) &&
            EdgeDirection.Equals(Other.EdgeDirection, KINDA_SMALL_NUMBER) &&
            Axis == Other.Axis &&
            SignChange == Other.SignChange;
    }
};

/**
 * Adaptive Octree Node struct for dynamic LOD-based meshing.
 */
struct VOXELPLUGIN_API FAdaptiveOctreeNode : public TSharedFromThis<FAdaptiveOctreeNode>
{
private:
    TFunction<double(FVector)> DensityFunction;
    void ComputeDualContourPosition();
    bool bIsLeaf = true;
    
    // Static array for child offsets
    inline static const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1, 1, -1), FVector(1, 1, -1),
        FVector(-1, -1, 1), FVector(1, -1, 1),
        FVector(-1, 1, 1), FVector(1, 1, 1)
    };

    // Maps face-aligned edges to their 4 corner indices
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

    TArray<FNodeCorner> Corners;
    TArray<FNodeEdge> Edges;
    TArray<FNodeEdge> SignChangeEdges;

    bool IsLeaf();
    bool IsRoot();

    void Split();
    bool ShouldSplit(FVector InCameraPosition, double InLodDistanceFactor);
    void Merge();
    bool ShouldMerge(FVector InCameraPosition, double InLodDistanceFactor);
    
    bool UpdateLod(FVector InCameraPosition, double InLodDistanceFactor);

    TArray<FNodeEdge> GetSurfaceEdges();
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceNodes();

    TArray<FNodeEdge>& GetSignChangeEdges();
    // Root Constructor
    FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InExtent, int InMinDepth, int InMaxDepth);

    // Child Constructor
    FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex);
};