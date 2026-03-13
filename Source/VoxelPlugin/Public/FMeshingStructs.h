#pragma once

#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

struct VOXELPLUGIN_API FNodeEdge
{
    FVector Positions[2];
    double Densities[2];
    int32 Axis;
    bool SignChange;
    FVector ZeroCrossingPoint;

    FNodeEdge() = default;

    FNodeEdge(const FVoxelCorner& A, const FVoxelCorner& B)
    {
        Positions[0] = A.GetPosition();
        Positions[1] = B.GetPosition();
        Densities[0] = A.Density;
        Densities[1] = B.Density;

        SignChange = (A.Density <= 0.0) != (B.Density <= 0.0);

        FVector Delta = (Positions[1] - Positions[0]).GetAbs();
        if (Delta.X > Delta.Y && Delta.X > Delta.Z) Axis = 0;
        else if (Delta.Y > Delta.X && Delta.Y > Delta.Z) Axis = 1;
        else Axis = 2;

        if (SignChange)
        {
            double Denom = Densities[0] - Densities[1];
            double t = FMath::Abs(Denom) < 1e-12 ? 0.5 : FMath::Clamp(Densities[0] / Denom, 0.0, 1.0);
            ZeroCrossingPoint = Positions[0] + t * (Positions[1] - Positions[0]);
        }
        else
        {
            ZeroCrossingPoint = (Positions[0] + Positions[1]) * 0.5;
        }
    }

    bool operator==(const FNodeEdge& Other) const
    {
        return Axis == Other.Axis &&
            ((Positions[0].Equals(Other.Positions[0], 0.01) && Positions[1].Equals(Other.Positions[1], 0.01)) ||
                (Positions[0].Equals(Other.Positions[1], 0.01) && Positions[1].Equals(Other.Positions[0], 0.01)));
    }
};

struct VOXELPLUGIN_API FMeshVertex
{
    FVector Position;           // Quantized position (chunk-local)
    FVector NormalizedPosition; // Projected onto planet radius sphere surface
    FVector Normal;
    FColor Color;
    double Depth;
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
    
    TWeakObjectPtr <UMaterialInterface> CachedSurfaceMaterial;
    TWeakObjectPtr <UMaterialInterface> CachedOceanMaterial;
    
    //Data Model Info
    FVector ChunkCenter;
    
    double ChunkExtent;
    
    TArray<FNodeEdge> ChunkEdges;
    
    TSharedPtr<FMeshStreamData> SurfaceMeshData;

    TSharedPtr<FMeshStreamData> OceanMeshData;

    //Mesh Stuff
    bool IsDirty = false;
    
    bool IsInitialized = false;
    
    TWeakObjectPtr<URealtimeMeshSimple> ChunkRtMesh;
    
    TWeakObjectPtr<URealtimeMeshComponent> ChunkRtComponent;

    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey::FRealtimeMeshLODKey(0);
    
    FRealtimeMeshSectionConfig SecConfig = FRealtimeMeshSectionConfig(0);

    void InitializeData(FVector InCenter, double InExtent) {
        SurfaceMeshData = MakeShared<FMeshStreamData>();
        OceanMeshData = MakeShared<FMeshStreamData>();

        ChunkCenter = InCenter;
        ChunkExtent = InExtent;

        SurfaceMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName("SURFACE"));
        SurfaceMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(SurfaceMeshData->MeshGroupKey, 0);

        OceanMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName("OCEAN"));
        OceanMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(OceanMeshData->MeshGroupKey, 0);
    };

    void InitializeComponent(ARealtimeMeshActor* InParentActor, UMaterialInterface* InSurfaceMaterial, UMaterialInterface* InOceanMaterial) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        ChunkRtMesh = NewObject<URealtimeMeshSimple>(InParentActor);
        ChunkRtMesh->SetCollisionConfig(cConfig);
        ChunkRtMesh->SetupMaterialSlot(0, "Surface Material");
        ChunkRtMesh->SetupMaterialSlot(1, "Ocean Material");

        ChunkRtComponent = NewObject<URealtimeMeshComponent>(InParentActor, URealtimeMeshComponent::StaticClass());
        ChunkRtComponent->RegisterComponent();
        
        ChunkRtComponent->SetMaterial(0, InSurfaceMaterial);
        ChunkRtComponent->SetMaterial(1, InOceanMaterial);

        ChunkRtComponent->AttachToComponent(InParentActor->GetRootComponent(), FAttachmentTransformRules::KeepRelativeTransform);
        ChunkRtComponent->SetRelativeLocation(ChunkCenter);
        ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
        ChunkRtComponent->SetCollisionResponseToAllChannels(ECollisionResponse::ECR_Block);
        ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh.Get());
        ChunkRtComponent->SetRenderCustomDepth(true);
        ChunkRtComponent->SetVisibleInRayTracing(false);

        ChunkRtMesh->CreateSectionGroup(SurfaceMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
        ChunkRtMesh->CreateSectionGroup(OceanMeshData->MeshGroupKey, FRealtimeMeshStreamSet());

        IsInitialized = true;
    }

    bool ShouldProcessEdge(const FNodeEdge& Edge, const TArray<TSharedPtr<FAdaptiveOctreeNode>>& SampledNodes) {
        if (SampledNodes.Num() < 3) return false;

        double OwnerExtent = FVector::Dist(Edge.Positions[0], Edge.Positions[1]) * 0.5;
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
                UMaterialInterface* SurfaceMaterial = Self->CachedSurfaceMaterial.Get();
                UMaterialInterface* OceanMaterial = Self->CachedOceanMaterial.Get();
                if (!Parent || !SurfaceMaterial || !OceanMaterial) return;
                Self->InitializeComponent(Parent, SurfaceMaterial, OceanMaterial);
            }

            // 2. Ensure valid component pointers
            URealtimeMeshSimple* MeshPtr = Self->ChunkRtMesh.Get();
            URealtimeMeshComponent* CompPtr = Self->ChunkRtComponent.Get();
            if (!MeshPtr || !CompPtr) return;
            if (!Self->IsDirty) return;

            // 3. Surface update
            auto* SrfTriStream = Self->SurfaceMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles);
            int32 SrfNumTris = SrfTriStream ? SrfTriStream->Num() : 0;
            if (SrfNumTris <= 0) {
                MeshPtr->UpdateSectionGroup(Self->SurfaceMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
            }
            else {
                MeshPtr->UpdateSectionGroup(Self->SurfaceMeshData->MeshGroupKey, Self->SurfaceMeshData->MeshStream);
                FRealtimeMeshSectionConfig SrfConfig(0); // material slot 0
                MeshPtr->UpdateSectionConfig(Self->SurfaceMeshData->MeshSectionKey, SrfConfig, true);
            }

            // 4. Ocean update
            auto* OcnTriStream = Self->OceanMeshData->MeshStream.Find(FRealtimeMeshStreams::Triangles);
            int32 OcnNumTris = OcnTriStream ? OcnTriStream->Num() : 0;
            if (OcnNumTris <= 0) {
                MeshPtr->UpdateSectionGroup(Self->OceanMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
            }
            else {
                MeshPtr->UpdateSectionGroup(Self->OceanMeshData->MeshGroupKey, Self->OceanMeshData->MeshStream);
                FRealtimeMeshSectionConfig OcnConfig(1); // material slot 1
                OcnConfig.bIsVisible = true;
                OcnConfig.bCastsShadow = false;
                MeshPtr->UpdateSectionConfig(Self->OceanMeshData->MeshSectionKey, OcnConfig, false);
            }

            Self->IsDirty = false;
        });
    }
};

FORCEINLINE uint32 GetTypeHash(const FMeshVertex& Vertex)
{
    return FCrc::MemCrc32(&Vertex.Position, sizeof(FVector));
}