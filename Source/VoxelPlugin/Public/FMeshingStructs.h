#pragma once

#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

struct VOXELPLUGIN_API FCornerSample {
    FVector Position;           // World position
    double Density;             // Final SDF density (computed in double, truncated to float on writeback)
    TArray<float*> Targets;     // Pointers to all Corners[i].Density that share this position
};

struct VOXELPLUGIN_API FMeshVertex
{
    FVector Position;       // Quantized position (chunk-local)
    FVector NormalizedPosition; // Projected onto planet radius sphere surface
    FVector Normal;
    FColor Color;
    double Depth;
    FVector2f UV;

    bool operator==(const FMeshVertex& Other) const
    {
        // Exact comparison on quantized position -- no epsilon needed
        return Position == Other.Position;
    }
};

struct VOXELPLUGIN_API FEdgeVertexData {
    TArray<FMeshVertex> Vertices;
    TOptional<FNodeEdge> Edge;
    bool IsValid;
};

struct VOXELPLUGIN_API FMeshStreamData {
    FRealtimeMeshSectionGroupKey MeshGroupKey;
    FRealtimeMeshSectionKey MeshSectionKey;
    FRealtimeMeshStreamSet MeshStream;

    FMeshStreamData() {
        MeshStream.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
        MeshStream.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>());
        MeshStream.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>());
        MeshStream.AddStream(FRealtimeMeshStreams::PolyGroups, GetRealtimeMeshBufferLayout<uint16>());
        MeshStream.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2DHalf>());
        MeshStream.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>());
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
    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;
    TWeakObjectPtr<USceneComponent> CachedMeshAttachRoot;

    TWeakObjectPtr <UMaterialInterface> CachedSurfaceMaterial;

    //Data Model Info
    FVector ChunkCenter;

    double ChunkExtent;

    TArray<FNodeEdge> ChunkEdges;

    TSharedPtr<FMeshStreamData> SurfaceMeshData;

    //Mesh Stuff
    bool IsDirty = false;

    bool IsInitialized = false;

    TWeakObjectPtr<URealtimeMeshSimple> ChunkRtMesh;

    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;

    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey::FRealtimeMeshLODKey(0);

    void InitializeData(FVector InCenter, double InExtent) {
        SurfaceMeshData = MakeShared<FMeshStreamData>();

        ChunkCenter = InCenter;
        ChunkExtent = InExtent;

        SurfaceMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName("SURFACE"));
        SurfaceMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(SurfaceMeshData->MeshGroupKey, 0);
    };

    void InitializeComponent(ARealtimeMeshActor* InParentActor, USceneComponent* InAttachRoot, UMaterialInterface* InSurfaceMaterial) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        ChunkRtMesh = NewObject<URealtimeMeshSimple>(InParentActor);
        ChunkRtMesh->SetCollisionConfig(cConfig);
        ChunkRtMesh->SetupMaterialSlot(0, "Surface Material");

        ChunkRtComponent = NewObject<URealtimeMeshComponent>(InParentActor, URealtimeMeshComponent::StaticClass());
        ChunkRtComponent->RegisterComponent();

        ChunkRtComponent->SetMaterial(0, InSurfaceMaterial);

        ChunkRtComponent->AttachToComponent(InAttachRoot, FAttachmentTransformRules::KeepRelativeTransform);
        ChunkRtComponent->SetRelativeLocation(ChunkCenter);
        ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        ChunkRtComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh.Get());
        ChunkRtComponent->SetRenderCustomDepth(true);
        ChunkRtComponent->SetVisibleInRayTracing(false);

        ChunkRtMesh->CreateSectionGroup(SurfaceMeshData->MeshGroupKey, FRealtimeMeshStreamSet());

        IsInitialized = true;
    }

    bool ShouldProcessEdge(const FNodeEdge& Edge, const FEdgeNeighbors& Neighbors) {
        if (Neighbors.Count < 3) return false;

        double OwnerExtent = Edge.Size * 0.5;
        for (int32 i = 0; i < Neighbors.Count; i++)
        {
            if (Neighbors.Nodes[i]->Extent < OwnerExtent * 0.9)
                return false;
        }
        return true;
    }

    void UpdateComponent(TSharedPtr<FMeshChunk> Self) {
        AsyncTask(ENamedThreads::GameThread, [Self]() {
            // 1. Lazy Init
            if (!Self->IsInitialized) {
                ARealtimeMeshActor* Parent = Self->CachedParentActor.Get();
                USceneComponent* AttachRoot = Self->CachedMeshAttachRoot.Get();
                UMaterialInterface* SurfaceMaterial = Self->CachedSurfaceMaterial.Get();
                if (!Parent || !AttachRoot || !SurfaceMaterial) return;
                Self->InitializeComponent(Parent, AttachRoot, SurfaceMaterial);
            }

            // 2. Ensure valid component pointers
            URealtimeMeshSimple* MeshPtr = Self->ChunkRtMesh.Get();
            URealtimeMeshComponent* CompPtr = Self->ChunkRtComponent.Get();
            if (!MeshPtr || !CompPtr) return;
            if (!Self->IsDirty) return;

            // 3. Surface update
            auto* SrfTriStream = Self->SurfaceMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles);
            int32 SrfNumTris = SrfTriStream ? SrfTriStream->Num() : 0;
            if (SrfNumTris <= 0)
            {
                MeshPtr->UpdateSectionGroup(Self->SurfaceMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
            }
            else
            {
                MeshPtr->UpdateSectionGroup(Self->SurfaceMeshData->MeshGroupKey, Self->SurfaceMeshData->MeshStream);
                FRealtimeMeshSectionConfig SrfConfig(0); // material slot 0
                MeshPtr->UpdateSectionConfig(Self->SurfaceMeshData->MeshSectionKey, SrfConfig, true);
            }

            Self->IsDirty = false;
            });
    }
};

FORCEINLINE uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}