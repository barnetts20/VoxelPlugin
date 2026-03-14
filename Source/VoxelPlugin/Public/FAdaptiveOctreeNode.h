#pragma once

#include "CoreMinimal.h"
#include <FNodeStructs.h>

struct VOXELPLUGIN_API FAdaptiveOctreeNode : public TSharedFromThis<FAdaptiveOctreeNode>
{
private:
    void ComputeDualContourPosition(FVector TreeCenter);

    FVector GetInterpolatedNormal(FVector P);

    bool bIsLeaf = true;

    static constexpr int DepthPrecisionFloor = 20;

public:
    FMortonIndex Index;

    TWeakPtr<FAdaptiveOctreeNode> Parent;

    TSharedPtr<FAdaptiveOctreeNode> Children[8];

    FNodeCorner Corners[8];

    TArray<FNodeEdge> SignChangeEdges;

    uint8 DepthBounds[3];

    FVector Center;

    FVector ChunkCenter;

    double Extent;

    FVector DualContourPosition;

    FVector3f DualContourNormal;

    bool IsSurfaceNode = false;

    bool LodOverride = false;

    const bool IsLeaf() const;

    const bool IsRoot() const;

    static bool EvaluateSplit(double Extent, double DistSq, double FOVScale, double Threshold, int Depth, int MaxDepth, int MinDepth)
    {
        if (Depth >= MaxDepth) return false;
        if (Depth < MinDepth) return true;
        double lhs = 2.0 * Extent * FOVScale;
        return (lhs * lhs) > (Threshold * Threshold * DistSq);
    }

    static bool EvaluateMerge(double Extent, double DistSq, double FOVScale, double Threshold, int Depth, int ChunkDepth, int MinDepth)
    {
        if (Depth <= ChunkDepth) return false;
        if (Depth < MinDepth) return false;
        double lhs = 2.0 * Extent * FOVScale;
        double rhs = Threshold * 0.5;
        return (lhs * lhs) < (rhs * rhs * DistSq);
    }

    // TreeCenter passed explicitly -- no longer stored per-node
    bool ShouldSplit(FVector TreeCenter, FVector InCameraPosition, double InScreenSpaceThreshold, double InFOVScale);

    bool ShouldMerge(FVector InCameraPosition, double InScreenSpaceThreshold, double InFOVScale);

    void Split();

    void Merge();

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceChunks();

    // Computes normalized position on the fly -- no longer stored on the node
    FVector ComputeNormalizedPosition(FVector TreeCenter, double InRadius) const;

    TArray<FNodeEdge>& GetSignChangeEdges();

    // TreeCenter passed explicitly for fallback normal computation
    void FinalizeFromExistingCorners(FVector TreeCenter, bool bSkipNormals = false);

    // Root Constructor
    FAdaptiveOctreeNode(FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    // Child Constructor
    FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 InChildIndex, FVector InAnchorCenter);
};
