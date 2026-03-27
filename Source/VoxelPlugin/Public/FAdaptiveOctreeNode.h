#pragma once

#include "CoreMinimal.h"
#include <FNodeStructs.h>

struct VOXELPLUGIN_API FAdaptiveOctreeNode : public TSharedFromThis<FAdaptiveOctreeNode>
{
private:
    void ComputeDualContourPosition();

    FVector GetInterpolatedNormal(FVector P);

    bool bIsLeaf = true;

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

    FVector3f DualContourNormal = FVector3f(0, 0, 1);

    bool IsSurfaceNode = false;

    // True if the node has sign changes (IsSurfaceNode) OR if any corner density
    // is close enough to zero that the surface could exist between corners at
    // finer resolution. Used by the LOD system to avoid skipping near-surface nodes.
    bool CouldContainSurface = false;

    bool LodOverride = false;

    const bool IsLeaf() const;

    const bool IsRoot() const;

    // Returns true if the node has sign changes or any corner is within
    // proximity of the zero crossing. Threshold scales with node extent.
    bool IsNearSurface() const
    {
        if (IsSurfaceNode) return true;
        const float Threshold = (float)(Extent * 3);
        for (int i = 0; i < 8; i++)
        {
            if (FMath::Abs(Corners[i].Density) < Threshold)
                return true;
        }
        return false;
    }

    static bool EvaluateSplit(double Extent, double DistSq, double FOVScale, double ThresholdSq, int Depth, int MaxDepth, int MinDepth)
    {
        if (Depth >= MaxDepth) return false;
        if (Depth < MinDepth) return true;
        double lhs = 2.0 * Extent * FOVScale;
        return (lhs * lhs) > (ThresholdSq * DistSq);
    }

    static bool EvaluateMerge(double Extent, double DistSq, double FOVScale, double MergeThresholdSq, int Depth, int ChunkDepth, int MinDepth)
    {
        if (Depth <= ChunkDepth) return false;
        if (Depth < MinDepth) return false;
        double lhs = 2.0 * Extent * FOVScale;
        return (lhs * lhs) < (MergeThresholdSq * DistSq);
    }

    // ThresholdSq / MergeThresholdSq precomputed once per LOD pass
    bool ShouldSplit(FVector InCameraPosition, double ThresholdSq, double InFOVScale);

    bool ShouldMerge(FVector InCameraPosition, double MergeThresholdSq, double InFOVScale);

    void Split();

    void Merge();

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceChunks();

    // Computes normalized position on the fly -- no longer stored on the node
    FVector ComputeNormalizedPosition(double InRadius) const;

    TArray<FNodeEdge>& GetSignChangeEdges();

    void FinalizeFromExistingCorners(bool bSkipNormals = false);

    // Root Constructor
    FAdaptiveOctreeNode(double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    // Child Constructor
    FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 InChildIndex, FVector InAnchorCenter);
};