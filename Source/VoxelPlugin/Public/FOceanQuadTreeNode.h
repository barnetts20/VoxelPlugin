// FOceanQuadTreeNode.h — Ocean quadtree node, mesh chunk, stream data, and
// static triangle grid cache. The quadtree tiles a cube-sphere with 6 face
// roots, each subdivided into a quadtree that drives LOD and mesh generation.
//
// Cube space uses a unit cube: each face root has Size=2.0, HalfSize=1.0.
// Positions in cube space are only used to compute direction vectors via
// GetSafeNormal() before projection onto the sphere at OceanRadius.
// WorldExtent = HalfSize * OceanRadius gives the world-space angular extent.

#pragma once

#include "CoreMinimal.h"
#include "FOceanSharedStructs.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

// Required by RealtimeMesh stream builder types. Narrower scoping is not feasible
// due to the template-heavy API surface used throughout this file.
using namespace RealtimeMesh;

class AOceanSphereActor;

/** Wrapper around a RealtimeMesh FRealtimeMeshStreamSet, pre-configured with the
 *  six vertex streams needed for ocean rendering. Similar to FMeshStreamData but
 *  uses full-precision FVector2f for UV0 (UV0.x = terrain SDF density at the ocean
 *  surface, interpolated on the GPU for depth-based shading). */
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
        // UV0: x = terrain SDF density at ocean surface (positive = underwater), y = unused.
        // Full float precision so depth interpolates correctly on the GPU.
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

/** A spatial subdivision of the ocean quadtree used for mesh generation.
 *  Each chunk owns two stream sets: inner (non-edge triangles) and edge
 *  (stitch triangles that adapt to neighbor LOD differences).
 *
 *  Lifecycle mirrors FMeshChunk: InitializeData sets spatial bounds and stream keys,
 *  InitializeComponent lazily creates the RealtimeMesh component on the game thread. */
struct VOXELPLUGIN_API FOceanMeshChunk
{
    TWeakObjectPtr<AOceanSphereActor> CachedOwner;
    FVector ChunkCenter;

    TSharedPtr<FOceanStreamData> InnerMeshData;
    TSharedPtr<FOceanStreamData> EdgeMeshData;

    bool IsDirty = false;
    bool IsInitialized = false;

    /** When true, the chunk's terrain SDF density indicates no ocean is visible
     *  (terrain fully above water). The mesh component is hidden but not destroyed. */
    bool IsCulled = false;

    TWeakObjectPtr<URealtimeMeshSimple>    ChunkRtMesh;
    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;
    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey(0);

    void InitializeData(FVector InCenter);
    void InitializeComponent(AOceanSphereActor* InOwner);
    void DestroyComponent(AOceanSphereActor* InOwner);
};

/** Static triangle index cache for one FaceResolution value.
 *
 *  Interior patch triangles are identical for every node at the same resolution.
 *  Edge stitch triangles vary only by which of the 4 neighbors are coarser --
 *  16 combinations encoded as a bitfield: bLeft | (bRight<<1) | (bTop<<2) | (bBottom<<3).
 *
 *  Built once on first use via Build(), then reused for every node rebuild.
 *  Indices reference the (Res+2)*(Res+2) extended vertex grid, which includes
 *  a 1-cell virtual border for neighbor-matched edge stitching. */
struct VOXELPLUGIN_API FOceanMeshGrid
{
    /** Interior non-edge, non-virtual triangles. Winding is default (CCW);
     *  flipped per-node at emit time using FaceTransform.bFlipWinding. */
    TArray<FIndex3UI> PatchTriangles;

    /** Edge stitch triangles for each of the 16 neighbor LOD flag combinations. */
    TArray<FIndex3UI> EdgeTriangles[16];

    void Build(int32 Res);

private:
    static void BuildEdgeVariant(int32 ExtRes, int32 tRes, uint8 Flags,
        TArray<FIndex3UI>& Out);
};

/** A single node in the ocean quadtree. Manages topology and LOD decisions
 *  but holds no mesh/vertex data -- that lives in FOceanMeshChunk.
 *
 *  Each node maps a rectangular region on one of the 6 cube faces to a
 *  spherical patch at OceanRadius. Cube space uses a unit cube (root Size=2.0)
 *  so positions are pure direction vectors before sphere projection.
 *  The quadtree splits/merges based on screen-space projected size, with
 *  back-face culling for far-side nodes.
 *
 *  Owner is a raw pointer -- nodes cannot outlive their owning AOceanSphereActor
 *  since the actor holds the root shared pointers for all 6 face trees. */
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

    /** Owning actor. Raw pointer is safe -- nodes are destroyed when the actor
     *  resets or destroys its face root shared pointers. */
    AOceanSphereActor* Owner = nullptr;

    TWeakPtr<FOceanQuadTreeNode>           Parent;
    TArray<TSharedPtr<FOceanQuadTreeNode>> Children;

    /** Quadtree path index (encodes face + depth + quadrant path). */
    FQuadIndex     Index;
    FCubeTransform FaceTransform;

    int32 MinDepth;
    int32 MaxDepth;
    int32 ChunkDepth;

    /** Position of this node's center on the unit cube face (cube space). */
    FVector CubeCenter;

    /** CubeCenter projected onto the sphere at OceanRadius (world space). */
    FVector SphereCenter;

    /** Sphere center of the chunk-depth ancestor that owns this node's mesh data. */
    FVector ChunkAnchorCenter;

    double OceanRadius;
    double Size;         // Full width of this node on the cube face.
    double HalfSize;
    double QuarterSize;

    /** World-space extent for screen-space LOD calculations.
     *  Cube-space HalfSize scaled to world: HalfSize * OceanRadius.
     *  At the root (HalfSize=1.0), WorldExtent equals OceanRadius. */
    double WorldExtent = 0.0;

    /** Depth of the neighbor node in each direction (LEFT, RIGHT, UP, DOWN).
     *  Used to select the correct edge stitch variant (coarser neighbor = halved
     *  edge resolution). Only the 2 outer edges per quadrant position are checked
     *  by CheckNeighbors -- inner edges always match the sibling's depth. */
    int32 NeighborLods[4] = { 0, 0, 0, 0 };

    /** Max terrain SDF density sampled at this node's face corners.
     *  Positive = terrain is below ocean (underwater), negative = terrain above ocean.
     *  Used as a cheap culling proxy: if all corners are below the cull threshold,
     *  the chunk is hidden (terrain fully exposed, no ocean visible).
     *  Initialized to FLT_MAX so chunks are never culled before their first mesh build. */
    float MaxVertexDepth = FLT_MAX;

    bool CanMerge = false;
    bool IsRestructuring = false;

    /** Collects all leaf nodes in the subtree rooted at Root (iterative DFS). */
    static void CollectLeaves(TSharedPtr<FOceanQuadTreeNode> Root,
        TArray<TSharedPtr<FOceanQuadTreeNode>>& Out);

    /** Evaluates split/merge for a leaf node. Returns true if the tree structure changed. */
    bool TrySetLod(FVector CameraPos, double ThresholdSq, double MergeThresholdSq, double FOVScale);

    /** Checks the 2 outer-facing neighbor edges for this node's quadrant position.
     *  Only outer edges can have LOD mismatches -- inner edges share a sibling at
     *  the same depth. Updates NeighborLods and returns true if any changed
     *  (triggering edge stitch rebuild). */
    bool CheckNeighbors();

    static void Split(TSharedPtr<FOceanQuadTreeNode> Node);
    static void Merge(TSharedPtr<FOceanQuadTreeNode> Node);
    void        SplitToDepth(int32 TargetDepth);
    void        TryMerge();
    static void RemoveChildren(TSharedPtr<FOceanQuadTreeNode> Node);

    bool ShouldSplit(double DistSq, double FOVScale, double ThresholdSq) const;
    bool ShouldMerge(double ParentDistSq, double ParentFOVScale, double MergeThresholdSq) const;

    /** Samples the terrain compositor at the 4 face corners to update MaxVertexDepth.
     *  Cheap proxy for visibility -- avoids a full vertex rebuild just to check culling. */
    void SampleMaxDepth();

    bool  IsLeaf()   const { return Children.Num() == 0; }
    int32 GetDepth() const { return Index.GetDepth(); }
};