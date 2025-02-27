#pragma once
#include "FAdaptiveOctree.h"
#include "FSparseOctree.h"
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
    FRWLock ChunkDataLock = FRWLock(0);

    TSharedPtr<FAdaptiveOctree> OwningOctree;
    TSharedPtr<FAdaptiveOctreeNode> ChunkNode;
    //TArray<TSharedPtr<FAdaptiveOctreeNode>> ChunkSurfaceNodes;
    TArray<FNodeEdge> ChunkEdges;
    TSharedPtr<FMeshStreamData> ChunkMeshData;
    URealtimeMeshSimple* ChunkRtMesh;
    URealtimeMeshComponent* ChunkRtComponent;
    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey::FRealtimeMeshLODKey(0);
    FRealtimeMeshSectionConfig SecConfig = FRealtimeMeshSectionConfig(0);

    void Initialize(ARealtimeMeshActor* InParentActor, UMaterialInterface* Material, TSharedPtr<FAdaptiveOctreeNode> InChunkNode, TSharedPtr<FAdaptiveOctree> InTree) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        OwningOctree = InTree;
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

    FVector2f ComputeTriplanarUV(FVector Position, FVector Normal)
    {
        FVector2f UV;
        FVector AbsNormal = Normal.GetAbs();

        if (AbsNormal.X > AbsNormal.Y && AbsNormal.X > AbsNormal.Z)
        {
            UV = FVector2f(Position.Y, Position.Z) * 0.0001f;
        }
        else if (AbsNormal.Y > AbsNormal.X && AbsNormal.Y > AbsNormal.Z)
        {
            UV = FVector2f(Position.X, Position.Z) * 0.0001f;
        }
        else
        {
            UV = FVector2f(Position.X, Position.Y) * 0.0001f;
        }

        return UV;
    };

    // Improved helper to check if an edge is on a negative chunk face
    bool IsEdgeOnChunkFace(const FNodeEdge& Edge, int Coef)
    {
        // Get negative face coordinates
        FVector negFace = ChunkNode->Center + (Coef * FVector(ChunkNode->Extent));

        // Tolerance scaled to chunk size
        const float tolerance = 0.001f * ChunkNode->Extent;

        // Check based on edge axis and position
        if (Edge.Axis == 0) // X-aligned edge
        {
            // Check if Y or Z matches negative face
            return FMath::Abs(Edge.ZeroCrossingPoint.Y - negFace.Y) < tolerance ||
                FMath::Abs(Edge.ZeroCrossingPoint.Z - negFace.Z) < tolerance;
        }
        else if (Edge.Axis == 1) // Y-aligned edge
        {
            // Check if X or Z matches negative face
            return FMath::Abs(Edge.ZeroCrossingPoint.X - negFace.X) < tolerance ||
                FMath::Abs(Edge.ZeroCrossingPoint.Z - negFace.Z) < tolerance;
        }
        else if (Edge.Axis == 2) // Z-aligned edge
        {
            // Check if X or Y matches negative face
            return FMath::Abs(Edge.ZeroCrossingPoint.X - negFace.X) < tolerance ||
                FMath::Abs(Edge.ZeroCrossingPoint.Y - negFace.Y) < tolerance;
        }

        return false;
    }

    // Simplified helper to check if a node is within this chunk
    bool IsNodeInChunk(TSharedPtr<FAdaptiveOctreeNode> Node)
    {
        FVector chunkMin = ChunkNode->Center - FVector(ChunkNode->Extent);
        FVector chunkMax = ChunkNode->Center + FVector(ChunkNode->Extent);

        // Use the node's center to determine if it's in the chunk
        return (Node->Center.X >= chunkMin.X && Node->Center.X <= chunkMax.X &&
            Node->Center.Y >= chunkMin.Y && Node->Center.Y <= chunkMax.Y &&
            Node->Center.Z >= chunkMin.Z && Node->Center.Z <= chunkMax.Z);
    }

    bool ShouldProcessEdge(const FNodeEdge& Edge, const TArray<TSharedPtr<FAdaptiveOctreeNode>>& SampledNodes)
    {
        //If we dont have enough nodes to form a triangle bail out
        if (SampledNodes.Num() < 3) return false;
        bool isOnNegChunkFace = IsEdgeOnChunkFace(Edge, -1);
        if (!isOnNegChunkFace) return true;

        // If we are on the chunk boundary, we need a case to handle LOD differences between Chunks
        // Specifically, we can end up missing geometry in cases where there are only 3 connected nodes
        //if (SampledNodes.Num() == 3)
        //{
            // Find which node is outside this chunk
            TSharedPtr<FAdaptiveOctreeNode> outsideNode = nullptr;
            TArray<FNodeEdge> testEdges;
            for (auto& node : SampledNodes)
            {
                // Check if node is outside chunk bounds
                if (!IsNodeInChunk(node))
                {

                    testEdges.Append(node->GetSignChangeEdges());
                    break;
                }
            }

            for (const FNodeEdge testEdge : testEdges) {
                if (testEdge == Edge) return false;
            }
            //If we are on the outside edge, and we cannot find the edge in the neighbor chunk, it is a stitching node
            return true;
    }
    //Update all chunk mesh data in async 2
    void UpdateMeshData() {
        ChunkMeshData->ResetStreams();
        auto PositionStream = ChunkMeshData->GetPositionStream();
        auto TangentStream = ChunkMeshData->GetTangentStream();
        auto ColorStream = ChunkMeshData->GetColorStream();
        auto TexCoordStream = ChunkMeshData->GetTexCoordStream();
        auto TriangleStream = ChunkMeshData->GetTriangleStream();
        auto PolygroupStream = ChunkMeshData->GetPolygroupStream();
        int idx = 0;
        int triIdx = 0;

        for (auto currentEdge : ChunkEdges){
            //Data only queries could return flat structs instead of recursive nodes perhaps
            TArray<TSharedPtr<FAdaptiveOctreeNode>> nodesToMesh = OwningOctree->SampleSurfaceNodesAroundEdge(currentEdge);
            if (!ShouldProcessEdge(currentEdge, nodesToMesh)) continue;
            //if (nodesToMesh.Num() < 3 
            //    ||!ShouldProcessEdge(currentEdge, nodesToMesh)
            //    ) continue; // Need at least 3 nodes to form a triangle       
            //if (nodesToMesh.Num() == 4) {
            //    int minLod = 9999;
            //    for (auto node : nodesToMesh) {
            //        minLod = FMath::Min(minLod, node->TreeIndex.Num());
            //    }
            //    if (currentEdge.LOD != minLod) continue;
            //}
            // Add first three vertices
            int32 IndexA = idx++;
            int32 IndexB = idx++;
            int32 IndexC = idx++;

            PositionStream.Add(nodesToMesh[0]->DualContourPosition);
            FRealtimeMeshTangentsHighPrecision tan0;
            tan0.SetNormal(FVector3f(nodesToMesh[0]->DualContourNormal));
            TangentStream.Add(tan0);
            ColorStream.Add(FColor::Green);
            TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[0]->DualContourPosition, nodesToMesh[0]->DualContourNormal));

            PositionStream.Add(nodesToMesh[1]->DualContourPosition);
            FRealtimeMeshTangentsHighPrecision tan1;
            tan1.SetNormal(FVector3f(nodesToMesh[1]->DualContourNormal));
            TangentStream.Add(tan1);
            ColorStream.Add(FColor::Green);
            TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[1]->DualContourPosition, nodesToMesh[1]->DualContourNormal));

            PositionStream.Add(nodesToMesh[2]->DualContourPosition);
            FRealtimeMeshTangentsHighPrecision tan2;
            tan2.SetNormal(FVector3f(nodesToMesh[2]->DualContourNormal));
            TangentStream.Add(tan2);
            ColorStream.Add(FColor::Green);
            TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[2]->DualContourPosition, nodesToMesh[2]->DualContourNormal));

            TriangleStream.Add(FIndex3UI(IndexA, IndexB, IndexC));
            PolygroupStream.Add(0);

            triIdx++;

            // Handle the fourth vertex if a quad exists
            if (nodesToMesh.Num() == 4)
            {
                int32 IndexD = idx++;

                PositionStream.Add(nodesToMesh[3]->DualContourPosition);
                FRealtimeMeshTangentsHighPrecision tan3;
                tan3.SetNormal(FVector3f(nodesToMesh[3]->DualContourNormal));
                TangentStream.Add(tan3);
                ColorStream.Add(FColor::Green);
                TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[3]->DualContourPosition, nodesToMesh[3]->DualContourNormal));

                TriangleStream.Add(FIndex3UI(IndexA, IndexC, IndexD));
                PolygroupStream.Add(0);
                triIdx++;
            }
        }
        PositionStream.SetNumUninitialized(idx);
        TangentStream.SetNumUninitialized(idx);
        ColorStream.SetNumUninitialized(idx);
        TexCoordStream.SetNumUninitialized(idx);
        TriangleStream.SetNumUninitialized(triIdx);
        PolygroupStream.SetNumUninitialized(triIdx);
    };

    //Iterate all chunks data updates in async 1
    //This should perform update to LOD, recalculation of implicit data and caching of the Edges for this chunk
    bool UpdateData(FVector InCameraPosition, double InLodFactor) {
        bool Updated = false;
        {
            if (ChunkNode && ChunkNode->UpdateLod(InCameraPosition, InLodFactor)) {
                ChunkEdges = ChunkNode->GetSurfaceEdges();
                Updated = true;
            }
        }
        return Updated;
    };

    void UpdateComponent() {
        if (ChunkMeshData) ChunkRtMesh->UpdateSectionGroup(ChunkMeshData->MeshGroupKey, ChunkMeshData->MeshStream).Then([this](TFuture<ERealtimeMeshProxyUpdateStatus> update) {
            ChunkRtMesh->UpdateSectionConfig(ChunkMeshData->MeshSectionKey, SecConfig, true);
        });
    }
};
