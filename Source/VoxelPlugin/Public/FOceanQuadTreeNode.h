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
    TWeakPtr<FOceanQuadTreeNode>            Parent;
    TArray<TSharedPtr<FOceanQuadTreeNode>>  Children;

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

    // Corner densities for this node's face quad — populated by PushDensityToChildren.
    // Layout: BL=0 (U-,V-), TL=1 (U-,V+), BR=2 (U+,V-), TR=3 (U+,V+).
    float CornerDensities[4] = { 0.f, 0.f, 0.f, 0.f };
    bool  bDensitySampled = false;

    // Vertex data
    TArray<FVector>   Vertices;
    TArray<FVector3f> Normals;
    TArray<FVector2f> TexCoords;
    TArray<FColor>    VertexColors; // depth encoded: R=bits[31:24], G=[23:16], B=[15:8], A=[7:0]
    TArray<FIndex3UI> AllTriangles;
    TArray<int32>     PatchTriangleIndices;
    TArray<FIndex3UI> EdgeTriangles;

    bool HasGenerated = false;
    bool IsRestructuring = false;
    bool CanMerge = false;

    static void CollectLeaves(TSharedPtr<FOceanQuadTreeNode> Root,
        TArray<TSharedPtr<FOceanQuadTreeNode>>& Out);

    bool TrySetLod(FVector CameraPos, double ThresholdSq, double MergeThresholdSq, double FOVScale);
    bool CheckNeighbors();

    static void Split(TSharedPtr<FOceanQuadTreeNode> Node);
    static void Merge(TSharedPtr<FOceanQuadTreeNode> Node);
    void SplitToDepth(int32 TargetDepth);
    void TryMerge();
    static void RemoveChildren(TSharedPtr<FOceanQuadTreeNode> Node);

    bool ShouldSplit(double DistSq, double FOVScale, double ThresholdSq) const;
    bool ShouldMerge(double ParentDistSq, double ParentFOVScale, double MergeThresholdSq) const;

    void GenerateMeshData();
    void RebuildEdgeTriangles();

    bool  IsLeaf()   const { return Children.Num() == 0; }
    int32 GetDepth() const { return Index.GetDepth(); }

private:
    FVector ProjectToSphere(FVector CubeOffset) const;
    int32   FaceResolution() const;
};