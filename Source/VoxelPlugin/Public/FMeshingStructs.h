#pragma once

// Forward declarations
struct FAdaptiveOctree;
struct FAdaptiveOctreeNode;
struct FNodeEdge;
struct FNodeCorner;
struct FAdaptiveOctreeFlatNode;

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

struct VOXELPLUGIN_API FMeshStreamData {
    FRealtimeMeshSectionGroupKey MeshGroupKey;
    FRealtimeMeshSectionKey MeshSectionKey;
    FRealtimeMeshStreamSet MeshStream;

    FMeshStreamData();

    TRealtimeMeshStreamBuilder<FVector, FVector3f> GetPositionStream();
    TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision> GetTangentStream();
    TRealtimeMeshStreamBuilder<TIndex3<uint32>> GetTriangleStream();
    TRealtimeMeshStreamBuilder<uint32, uint16> GetPolygroupStream();
    TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf> GetTexCoordStream();
    TRealtimeMeshStreamBuilder<FColor> GetColorStream();

    void ResetStreams();
};

struct VOXELPLUGIN_API FMeshChunk {
    //FRWLock ChunkDataLock = FRWLock(0);
    TSharedPtr<FAdaptiveOctree> OwningOctree;
    FVector ChunkCenter;
    double ChunkExtent;
    TArray<FNodeEdge> ChunkEdges;
    TSharedPtr<FMeshStreamData> ChunkMeshData;

    // Mesh components
    bool IsDirty = false;
    URealtimeMeshSimple* ChunkRtMesh;
    URealtimeMeshComponent* ChunkRtComponent;
    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey::FRealtimeMeshLODKey(0);
    FRealtimeMeshSectionConfig SecConfig = FRealtimeMeshSectionConfig(0);

    void Initialize(ARealtimeMeshActor* InParentActor, UMaterialInterface* Material,
        TSharedPtr<FAdaptiveOctreeNode> InChunkNode, TSharedPtr<FAdaptiveOctree> InTree);

    FVector2f ComputeTriplanarUV(FVector Position, FVector Normal);
    bool IsEdgeOnChunkFace(const FNodeEdge& Edge, int Coef);
    bool IsNodeInChunk(FAdaptiveOctreeFlatNode Node);
    bool ShouldProcessEdge(const FNodeEdge& Edge, const TArray<FAdaptiveOctreeFlatNode>& SampledNodes);
    void UpdateMeshData();
    void UpdateComponent();
};