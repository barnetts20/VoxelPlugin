#pragma once

#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

struct VOXELPLUGIN_API FCornerSample {
    FVector Position;           // World position
    double Density;             // Final SDF density
    double Dist;
    float NoiseValue;           // Raw noise result
    TArray<double*> Targets;    // Pointers to all Corners[i].Density that share this position
};

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
    TWeakObjectPtr<ARealtimeMeshActor> CachedParentActor;
    
    TWeakObjectPtr <UMaterialInterface> CachedMaterial;
    
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

    void InitializeData(FVector InCenter, double InExtent) {
        ChunkMeshData = MakeShared<FMeshStreamData>();
        ChunkCenter = InCenter;
        ChunkExtent = InExtent;
        ChunkMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName(ChunkCenter.ToCompactString()));
        ChunkMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(ChunkMeshData->MeshGroupKey, 0);
    };

    void InitializeComponent(ARealtimeMeshActor* InParentActor, UMaterialInterface* Material) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        ChunkRtMesh = NewObject<URealtimeMeshSimple>(InParentActor);
        ChunkRtMesh->SetCollisionConfig(cConfig);
        ChunkRtMesh->SetupMaterialSlot(0, "Material");
        ChunkRtComponent = NewObject<URealtimeMeshComponent>(InParentActor, URealtimeMeshComponent::StaticClass());
        ChunkRtComponent->RegisterComponent();
        ChunkRtComponent->SetMaterial(0, Material);
        ChunkRtComponent->AttachToComponent(InParentActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        ChunkRtComponent->SetRelativeLocation(ChunkCenter);
        ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        ChunkRtComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh.Get());
        ChunkRtComponent->SetRenderCustomDepth(true);
        ChunkRtComponent->SetVisibleInRayTracing(false);
        ChunkRtMesh->CreateSectionGroup(ChunkMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
        IsInitialized = true;
    }

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
            // 1. Lazy Init
            if (!Self->IsInitialized) {
                ARealtimeMeshActor* Parent = Self->CachedParentActor.Get();
                UMaterialInterface* Material = Self->CachedMaterial.Get();
                if (!Parent || !Material) return;
                Self->InitializeComponent(Parent, Material);
            }

            // 2. Ensure valid component pointers
            URealtimeMeshSimple* MeshPtr = Self->ChunkRtMesh.Get();
            URealtimeMeshComponent* CompPtr = Self->ChunkRtComponent.Get();
            if (!MeshPtr || !CompPtr) return;

            // 3. The Zero-Triangle Guard
            auto* PositionStream = Self->ChunkMeshData->MeshStream.Find(FRealtimeMeshStreams::Position);
            int32 NumVerts = PositionStream ? PositionStream->Num() : 0;
            if (NumVerts < 3) {
                // If it was previously initialized but now has no data (e.g., deleted via terraforming)
                MeshPtr->UpdateSectionGroup(Self->ChunkMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
                Self->IsDirty = false;
                return;
            }

            // 4. Normal Mesh Update Case
            if (!Self->IsDirty) return;
            // Use the resolved MeshPtr instead of the WeakPtr wrapper
            MeshPtr->UpdateSectionGroup(Self->ChunkMeshData->MeshGroupKey, Self->ChunkMeshData->MeshStream);
            MeshPtr->UpdateSectionConfig(Self->ChunkMeshData->MeshSectionKey, Self->SecConfig, true);

            Self->IsDirty = false;
        });
    }
};

FORCEINLINE uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}