#pragma once

#include "CoreMinimal.h"
#include "FOceanSharedStructs.h"
#include "FDensitySampleCompositor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

class AOceanSphereActor;

// ---------------------------------------------------------------------------
// FOceanStreamData
// ---------------------------------------------------------------------------
struct VOXELPLUGIN_API FOceanStreamData
{
    FRealtimeMeshSectionGroupKey MeshGroupKey;
    FRealtimeMeshSectionKey      MeshSectionKey;
    FRealtimeMeshStreamSet       MeshStream;

    FOceanStreamData()
    {
        MeshStream.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
        MeshStream.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>());
        MeshStream.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>());
        MeshStream.AddStream(FRealtimeMeshStreams::PolyGroups, GetRealtimeMeshBufferLayout<uint16>());
        MeshStream.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2DHalf>());
        MeshStream.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>());
    }

    TRealtimeMeshStreamBuilder<FVector, FVector3f> GetPositionStream() {
        return TRealtimeMeshStreamBuilder<FVector, FVector3f>(*MeshStream.Find(FRealtimeMeshStreams::Position));
    }
    TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision> GetTangentStream() {
        return TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision>(*MeshStream.Find(FRealtimeMeshStreams::Tangents));
    }
    TRealtimeMeshStreamBuilder<TIndex3<uint32>> GetTriangleStream() {
        return TRealtimeMeshStreamBuilder<TIndex3<uint32>>(*MeshStream.Find(FRealtimeMeshStreams::Triangles));
    }
    TRealtimeMeshStreamBuilder<uint32, uint16> GetPolygroupStream() {
        return TRealtimeMeshStreamBuilder<uint32, uint16>(*MeshStream.Find(FRealtimeMeshStreams::PolyGroups));
    }
    TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf> GetTexCoordStream() {
        return TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf>(*MeshStream.Find(FRealtimeMeshStreams::TexCoords));
    }
    TRealtimeMeshStreamBuilder<FColor> GetColorStream() {
        return TRealtimeMeshStreamBuilder<FColor>(*MeshStream.Find(FRealtimeMeshStreams::Color));
    }
};

// ---------------------------------------------------------------------------
// FOceanMeshChunk
// ---------------------------------------------------------------------------
struct VOXELPLUGIN_API FOceanMeshChunk
{
    TWeakObjectPtr<AOceanSphereActor> CachedOwner;
    FVector ChunkCenter;

    TSharedPtr<FOceanStreamData> InnerMeshData;
    TSharedPtr<FOceanStreamData> EdgeMeshData;

    bool IsDirty = false;
    bool IsInitialized = false;

    TWeakObjectPtr<URealtimeMeshSimple>    ChunkRtMesh;
    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;

    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey(0);

    void InitializeData(FVector InCenter);
    void InitializeComponent(AOceanSphereActor* InOwner);
    void DestroyComponent(AOceanSphereActor* InOwner);
};

// ---------------------------------------------------------------------------
// FOceanQuadTreeNode
// ---------------------------------------------------------------------------
class VOXELPLUGIN_API FOceanQuadTreeNode : public TSharedFromThis<FOceanQuadTreeNode>
{
public:
    FOceanQuadTreeNode(
        AOceanSphereActor* InOwner,
        FCubeTransform                      InFaceTransform,
        FQuadIndex                          InIndex,
        FVector                             InCubeCenter,
        double                              InSize,
        double                              InOceanRadius,
        int32                               InMinDepth,
        int32                               InMaxDepth,
        int32                               InChunkDepth,
        TSharedPtr<FDensitySampleCompositor> InCompositor
    );

    AOceanSphereActor* Owner = nullptr;
    TWeakPtr<FOceanQuadTreeNode>            Parent;
    TArray<TSharedPtr<FOceanQuadTreeNode>>  Children;

    // Density compositor — shared across entire tree, set once at root construction
    TSharedPtr<FDensitySampleCompositor> Compositor;

    FQuadIndex      Index;
    FCubeTransform  FaceTransform;

    int32   MinDepth;
    int32   MaxDepth;
    int32   ChunkDepth;

    FVector CubeCenter;
    FVector SphereCenter;
    FVector ChunkAnchorCenter;

    double  OceanRadius;
    double  Size;
    double  HalfSize;
    double  QuarterSize;
    double  WorldExtent = 0.0;
    double  NodeBoundRadius = 0.0;

    int32 NeighborLods[4] = { 0, 0, 0, 0 };

    // Corner densities: 4 corners in face-local order matching CubeCenter offsets.
    // Index layout matches child order: BL=0, TL=1, BR=2, TR=3
    // (same as EChildPosition enum: BOTTOM_LEFT=0, TOP_LEFT=1, BOTTOM_RIGHT=2, TOP_RIGHT=3)
    float CornerDensities[4] = { 0.f, 0.f, 0.f, 0.f };
    bool bDensitySampled = false;

    // Vertex data
    TArray<FVector>   Vertices;
    TArray<FVector3f> Normals;
    TArray<FVector2f> TexCoords;
    TArray<float>     VertexDepths;   // per-vertex depth, interpolated from corner densities
    TArray<FIndex3UI> AllTriangles;
    TArray<int32>     PatchTriangleIndices;
    TArray<FIndex3UI> EdgeTriangles;

    bool HasGenerated = false;
    bool IsRestructuring = false;
    bool CanMerge = false;

    // Tree traversal
    static void CollectLeaves(TSharedPtr<FOceanQuadTreeNode> Root,
        TArray<TSharedPtr<FOceanQuadTreeNode>>& Out);

    // LOD
    bool TrySetLod(FVector CameraPos, double ThresholdSq, double MergeThresholdSq, double FOVScale);

    // Neighbor stitching
    bool CheckNeighbors();

    static void Split(TSharedPtr<FOceanQuadTreeNode> Node);
    static void Merge(TSharedPtr<FOceanQuadTreeNode> Node);
    void SplitToDepth(int32 TargetDepth);
    void TryMerge();
    static void RemoveChildren(TSharedPtr<FOceanQuadTreeNode> Node);

    bool ShouldSplit(double DistSq, double FOVScale, double ThresholdSq) const;
    bool ShouldMerge(double ParentDistSq, double ParentFOVScale, double MergeThresholdSq) const;

    // Density sampling
    // Samples all 4 corners in a single batch call and stores results in CornerDensities.
    void SampleCornerDensities();

    // Called on Split: samples only the 5 new positions (4 edge midpoints + face center)
    // and distributes all 4 corner densities to each of the 4 children without redundant samples.
    static void SplitAndDistributeDensities(TSharedPtr<FOceanQuadTreeNode> Parent);

    // Bilinearly interpolates CornerDensities at a given face-local UV in [0,1]^2.
    // U maps to the face's AxisMap[0] direction, V to AxisMap[1].
    float InterpolateDensity(float U, float V) const;

    // Mesh data
    void GenerateMeshData();
    void RebuildEdgeTriangles();

    // Encodes a float depth value into FColor RGBA (4x8-bit fixed point).
    // Range is clamped to [DepthRangeMin, DepthRangeMax].
    // Reconstruct in shader: depth = (R/255 + G/255^2 + B/255^3 + A/255^4) * Range + Min
    static FColor EncodeDepthToColor(float Depth, float DepthRangeMin, float DepthRangeMax);

    // Returns true if all 4 corner densities are below the threshold,
    // meaning no ocean surface passes through this node.
    // Only meaningful after SampleCornerDensities() or SplitAndDistributeDensities() has run.
    bool IsFullySubmerged(float Threshold) const
    {
        if (!bDensitySampled) return false;
        return CornerDensities[0] < Threshold
            && CornerDensities[1] < Threshold
            && CornerDensities[2] < Threshold
            && CornerDensities[3] < Threshold;
    }

    bool  IsLeaf()  const { return Children.Num() == 0; }
    int32 GetDepth() const { return Index.GetDepth(); }

private:
    FVector ProjectToSphere(FVector CubeOffset) const;
    int32   FaceResolution() const;

    // Returns the face-local UV [0,1]^2 for a vertex given its cube-space offset from CubeCenter.
    // U = AxisMap[0] component mapped from [-HalfSize, HalfSize] to [0, 1]
    // V = AxisMap[1] component mapped from [-HalfSize, HalfSize] to [0, 1]
    FVector2f ComputeFaceUV(const FVector& CubeOffset) const;
};