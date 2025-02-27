#pragma once

#include "FSparseOctree.h"
#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

struct VOXELPLUGIN_API FMeshStreamData {
    FRealtimeMeshSectionGroupKey MeshGroupKey;
    FRealtimeMeshSectionKey MeshSectionKey;
    FRealtimeMeshStreamSet MeshStream;

    FMeshStreamData() {
        TRealtimeMeshStreamBuilder<FVector, FVector3f> PositionBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>()));
        TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision> TangentBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>()));
        TRealtimeMeshStreamBuilder<TIndex3<uint32>> TrianglesBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>()));
        TRealtimeMeshStreamBuilder<uint32, uint16> PolygroupsBuilder(MeshStream.AddStream(FRealtimeMeshStreams::PolyGroups, GetRealtimeMeshBufferLayout<uint16>()));
        TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf> TexCoordsBuilder(MeshStream.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2DHalf>()));
        TRealtimeMeshStreamBuilder<FColor> ColorBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>()));
    }

    TRealtimeMeshStreamBuilder<FVector, FVector3f> GetPositionStream() {
        return TRealtimeMeshStreamBuilder<FVector, FVector3f>(*MeshStream.Find((FRealtimeMeshStreams::Position)));
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

    void ResetStreams() {
        if (auto* Position = MeshStream.Find(FRealtimeMeshStreams::Position)) Position->Empty();
        if (auto* Tangent = MeshStream.Find(FRealtimeMeshStreams::Tangents)) Tangent->Empty();
        if (auto* Triangle = MeshStream.Find(FRealtimeMeshStreams::Triangles)) Triangle->Empty();
        if (auto* Polygroup = MeshStream.Find(FRealtimeMeshStreams::PolyGroups)) Polygroup->Empty();
        if (auto* TexCoord = MeshStream.Find(FRealtimeMeshStreams::TexCoords)) TexCoord->Empty();
        if (auto* Color = MeshStream.Find(FRealtimeMeshStreams::Color)) Color->Empty();
    }
};

struct VOXELPLUGIN_API FMeshChunk {
    //Data Model Info
    FAdaptiveOctreeFlatNode ChunkNode;
    TArray<FNodeEdge> ChunkEdges;
    TSharedPtr<FMeshStreamData> ChunkMeshData;

    //Mesh Stuff
    bool IsDirty = false;
    TSharedPtr<FRWLock> MeshDataLock = MakeShared<FRWLock>();
    URealtimeMeshSimple* ChunkRtMesh;
    URealtimeMeshComponent* ChunkRtComponent;
    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey::FRealtimeMeshLODKey(0);
    FRealtimeMeshSectionConfig SecConfig = FRealtimeMeshSectionConfig(0);

    void Initialize(ARealtimeMeshActor* InParentActor, UMaterialInterface* Material, FAdaptiveOctreeFlatNode InChunkNode) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        //OwningOctree = InTree;
        ChunkNode = InChunkNode;
        ChunkMeshData = MakeShared<FMeshStreamData>();
        ChunkMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName("MeshGroup"));
        ChunkMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(ChunkMeshData->MeshGroupKey, 0);

        ChunkRtMesh = NewObject<URealtimeMeshSimple>(InParentActor);
        ChunkRtMesh->SetCollisionConfig(cConfig);
        ChunkRtMesh->SetupMaterialSlot(0, "Material");

        ChunkRtComponent = NewObject<URealtimeMeshComponent>(InParentActor, URealtimeMeshComponent::StaticClass());
        ChunkRtComponent->RegisterComponent();

        ChunkRtComponent->SetMaterial(0, Material);
        ChunkRtComponent->AttachToComponent(InParentActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        ChunkRtComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh);
        ChunkRtComponent->SetRenderCustomDepth(true);

        ChunkRtMesh->CreateSectionGroup(ChunkMeshData->MeshGroupKey, ChunkMeshData->MeshStream).Wait();
        ChunkRtMesh->UpdateSectionConfig(ChunkMeshData->MeshSectionKey, SecConfig, true).Wait();
        ChunkRtMesh->ClearInternalFlags(EInternalObjectFlags::Async);
    };

    bool IsEdgeOnChunkFace(const FNodeEdge& Edge, int Coef) {
        // Pre-compute face position vector and tolerance once
        const float tolerance = 0.001f * ChunkNode.Extent;
        const FVector facePos = ChunkNode.Center + (Coef * FVector(ChunkNode.Extent));

        // Calculate perpendicular axes using modular arithmetic
        const int axis1 = (Edge.Axis + 1) % 3;  // First perpendicular axis
        const int axis2 = (Edge.Axis + 2) % 3;  // Second perpendicular axis

        // Check if edge's zero-crossing lies on the face for either perpendicular axis
        return FMath::Abs(Edge.ZeroCrossingPoint[axis1] - facePos[axis1]) < tolerance ||
            FMath::Abs(Edge.ZeroCrossingPoint[axis2] - facePos[axis2]) < tolerance;
    }

    bool IsNodeInChunk(const FAdaptiveOctreeFlatNode& Node) {
        // Calculate extents once
        const FVector extentVec(ChunkNode.Extent);
        FVector distFromCenter = (Node.Center - ChunkNode.Center).GetAbs();
        return distFromCenter.X <= ChunkNode.Extent &&
            distFromCenter.Y <= ChunkNode.Extent &&
            distFromCenter.Z <= ChunkNode.Extent;
    }

    bool ShouldProcessEdge(const FNodeEdge& Edge, const TArray<FAdaptiveOctreeFlatNode>& SampledNodes) {
        // If we don't have enough nodes to form a triangle bail out
        if (SampledNodes.Num() < 3) return false;
        // If it's not on a stitching face, mesh it
        if (!IsEdgeOnChunkFace(Edge, -1)) return true;
        // For each node that is outside the chunk bounds, add its edges to test against
        TArray<FNodeEdge> testEdges;
        for (auto& node : SampledNodes) {
            if (!IsNodeInChunk(node)) testEdges.Append(node.SignChangeEdges);
        }
        // If the other chunks edges do not contain the edge, mesh it
        // Otherwise it will be handled by the external chunk
        return !testEdges.Contains(Edge);
    }
    //Update all chunk mesh data in async 2
    void UpdateMeshData(FMeshStreamData newMeshData) {
        FRWScopeLock WriteLock(*MeshDataLock, SLT_Write);
        ChunkMeshData->MeshStream = FRealtimeMeshStreamSet(newMeshData.MeshStream);
        IsDirty = true;
    }

    void UpdateComponent() {
        FRWScopeLock ReadLock(*MeshDataLock, SLT_ReadOnly);
        if (!IsDirty) return;
        if (ChunkMeshData) ChunkRtMesh->UpdateSectionGroup(ChunkMeshData->MeshGroupKey, ChunkMeshData->MeshStream).Then([this](TFuture<ERealtimeMeshProxyUpdateStatus> update) {
            ChunkRtMesh->UpdateSectionConfig(ChunkMeshData->MeshSectionKey, SecConfig, true);
        });
        IsDirty = false;
    }
};
