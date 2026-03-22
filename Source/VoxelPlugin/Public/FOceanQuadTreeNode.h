#pragma once

#include "CoreMinimal.h"
#include "FOceanSharedStructs.h"
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
        // UV0: x = depth (cm), y = unused. Full float so depth interpolates correctly on the GPU.
        MeshStream.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2f>());
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
    TRealtimeMeshStreamBuilder<FVector2f> GetTexCoordStream() {
        return TRealtimeMeshStreamBuilder<FVector2f>(*MeshStream.Find(FRealtimeMeshStreams::TexCoords));
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
    bool IsCulled = false;

    TWeakObjectPtr<URealtimeMeshSimple>    ChunkRtMesh;
    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;

    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey(0);

    void InitializeData(FVector InCenter);
    void InitializeComponent(AOceanSphereActor* InOwner);
    void DestroyComponent(AOceanSphereActor* InOwner);
};

// ---------------------------------------------------------------------------
// FOceanMeshGrid
//
// Static triangle index cache for one FaceResolution value.
// Interior patch triangles are identical for every node at the same resolution.
// Edge stitch triangles vary only by which 4 neighbors are coarser �
// 16 combinations encoded as: bLeft | (bRight<<1) | (bTop<<2) | (bBottom<<3).
// Built once on first use, reused for every node rebuild.
// ---------------------------------------------------------------------------
struct VOXELPLUGIN_API FOceanMeshGrid
{
    // Interior non-edge, non-virtual triangles.
    // Indices into the (Res+2)*(Res+2) extended vertex grid.
    // Winding applied per-node at emit time using FaceTransform.bFlipWinding.
    TArray<FIndex3UI> PatchTriangles;

    // Edge stitch triangles for each of the 16 neighbor LOD flag combinations.
    TArray<FIndex3UI> EdgeTriangles[16];

    void Build(int32 Res);

private:
    static void BuildEdgeVariant(int32 ExtRes, int32 tRes, uint8 Flags,
        TArray<FIndex3UI>& Out);
};

// ---------------------------------------------------------------------------
// FOceanQuadTreeNode  �  pure topology + LOD, no mesh/vertex data.
// ---------------------------------------------------------------------------
class VOXELPLUGIN_API FOceanQuadTreeNode : public TSharedFromThis<FOceanQuadTreeNode>
{
public:
    FOceanQuadTreeNode(
        AOceanSphereActor* InOwner,
        FCubeTransform      InFaceTransform,
        FQuadIndex          InIndex,
        FVector             InCubeCenter,
        double              InSize,
        double              InOceanRadius,
        int32               InMinDepth,
        int32               InMaxDepth,
        int32               InChunkDepth
    );

    AOceanSphereActor* Owner = nullptr;
    TWeakPtr<FOceanQuadTreeNode>           Parent;
    TArray<TSharedPtr<FOceanQuadTreeNode>> Children;

    FQuadIndex     Index;
    FCubeTransform FaceTransform;

    int32 MinDepth;
    int32 MaxDepth;
    int32 ChunkDepth;

    FVector CubeCenter;
    FVector SphereCenter;
    FVector ChunkAnchorCenter;

    double OceanRadius;
    double Size;
    double HalfSize;
    double QuarterSize;
    double WorldExtent = 0.0;

    int32 NeighborLods[4] = { 0, 0, 0, 0 };

    // Max compositor depth (cm) sampled at the corners of this node's face.
    // Cheap proxy for culling � avoids full vertex rebuild just to check visibility.
    // Starts at FLT_MAX so chunks are never culled before their first real mesh build.
    // Updated with actual vertex data during RebuildChunkStreamData.
    float MaxVertexDepth = FLT_MAX;

    bool CanMerge = false;
    bool IsRestructuring = false;

    static void CollectLeaves(TSharedPtr<FOceanQuadTreeNode> Root,
        TArray<TSharedPtr<FOceanQuadTreeNode>>& Out);

    bool TrySetLod(FVector CameraPos, double ThresholdSq, double MergeThresholdSq, double FOVScale);
    bool CheckNeighbors();

    static void Split(TSharedPtr<FOceanQuadTreeNode> Node);
    static void Merge(TSharedPtr<FOceanQuadTreeNode> Node);
    void        SplitToDepth(int32 TargetDepth);
    void        TryMerge();
    static void RemoveChildren(TSharedPtr<FOceanQuadTreeNode> Node);

    bool ShouldSplit(double DistSq, double FOVScale, double ThresholdSq) const;
    bool ShouldMerge(double ParentDistSq, double ParentFOVScale, double MergeThresholdSq) const;

    // Samples the compositor at the 4 face corners to update MaxVertexDepth.
    void SampleMaxDepth();

    bool  IsLeaf()   const { return Children.Num() == 0; }
    int32 GetDepth() const { return Index.GetDepth(); }
};