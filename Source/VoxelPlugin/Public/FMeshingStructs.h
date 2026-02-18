#pragma once

#include "FSparseOctree.h"
#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

struct VOXELPLUGIN_API FMeshVertex
{
    FVector Position;       // Quantized position (chunk-local)
    FVector OriginalPosition; // Unquantized for actual mesh output
    FVector Normal;
    FColor Color;
    FVector2f UV;

    bool operator==(const FMeshVertex& Other) const
    {
        // Exact comparison on quantized position — no epsilon needed
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
    //Data Model Info
    FVector ChunkCenter;
    double ChunkExtent;
    TArray<FNodeEdge> ChunkEdges;
    TSharedPtr<FMeshStreamData> ChunkMeshData;

    //Mesh Stuff
    bool IsDirty = false;
    bool IsInitialized = false;
    
    TWeakObjectPtr<URealtimeMeshSimple> ChunkRtMesh;
    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;

    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey::FRealtimeMeshLODKey(0);
    FRealtimeMeshSectionConfig SecConfig = FRealtimeMeshSectionConfig(0);

    void Initialize(ARealtimeMeshActor* InParentActor, UMaterialInterface* Material, FVector InCenter, double InExtent) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        ChunkMeshData = MakeShared<FMeshStreamData>();

        ChunkCenter = InCenter;
        ChunkExtent = InExtent;

        ChunkRtMesh = NewObject<URealtimeMeshSimple>(InParentActor);
        ChunkRtMesh->SetCollisionConfig(cConfig);
        ChunkRtMesh->SetupMaterialSlot(0, "Material");

        ChunkRtComponent = NewObject<URealtimeMeshComponent>(InParentActor, URealtimeMeshComponent::StaticClass());
        ChunkRtComponent->RegisterComponent();

        ChunkRtComponent->SetMaterial(0, Material);
        
        ChunkRtComponent->AttachToComponent(InParentActor->GetRootComponent(),FAttachmentTransformRules::KeepRelativeTransform);
        ChunkRtComponent->SetRelativeLocation(InCenter);

        ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        ChunkRtComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh.Get());
        ChunkRtComponent->SetRenderCustomDepth(true);
        
        ChunkMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName(ChunkCenter.ToCompactString()));
        ChunkRtMesh->CreateSectionGroup(ChunkMeshData->MeshGroupKey, ChunkMeshData->MeshStream);
        ChunkMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(ChunkMeshData->MeshGroupKey, 0);

        IsInitialized = true;
    };

    bool ShouldProcessEdge(const FNodeEdge& Edge, const TArray<TSharedPtr<FAdaptiveOctreeNode>>& SampledNodes) {
        if (SampledNodes.Num() < 3) return false;

        double OwnerExtent = Edge.Size * 0.5;
        for (auto& Node : SampledNodes)
        {
            if (Node->Extent < OwnerExtent * 0.9)
                return false;
        }
        return true;
    }

    void UpdateComponent(TSharedPtr<FMeshChunk> Self) {
        AsyncTask(ENamedThreads::GameThread, [Self]() {
            // 1. Initial State Check
            if (!Self->IsInitialized || !Self->IsDirty) return;

            // 2. Resolve Weak Pointers to Raw Pointers
            // This is the "One-Time Resolution" that fixes the conversion error
            URealtimeMeshSimple* MeshPtr = Self->ChunkRtMesh.Get();
            URealtimeMeshComponent* CompPtr = Self->ChunkRtComponent.Get();

            // 3. Safety Check: If the Actor/Component was destroyed, bail out
            if (!MeshPtr || !CompPtr) return;

            // 4. Ghost Mesh Check
            auto* PositionStream = Self->ChunkMeshData->MeshStream.Find(FRealtimeMeshStreams::Position);
            if (!PositionStream || PositionStream->Num() == 0) {
                // Clear the section by passing an empty StreamSet
                MeshPtr->UpdateSectionGroup(Self->ChunkMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
                Self->IsDirty = false;
                return;
            }

            // 5. Standard Update
            Self->IsDirty = false;

            // Use the resolved MeshPtr instead of the WeakPtr wrapper
            MeshPtr->UpdateSectionGroup(Self->ChunkMeshData->MeshGroupKey, Self->ChunkMeshData->MeshStream);
            MeshPtr->UpdateSectionConfig(Self->ChunkMeshData->MeshSectionKey, Self->SecConfig, true);
        });
    }
};