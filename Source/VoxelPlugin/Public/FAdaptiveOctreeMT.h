// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <FMeshingStructs.h>
#include <FAdaptiveOctreeNodeMT.h>


struct VOXELPLUGIN_API FMeshChunkMT {
    //Data Model Info
    FVector ChunkCenter;
    double ChunkExtent;
    TSharedPtr<FMeshStreamData> ChunkMeshData;

    //Mesh Stuff
    bool IsDirty = false;

    URealtimeMeshSimple* ChunkRtMesh;
    URealtimeMeshComponent* ChunkRtComponent;
    FRealtimeMeshLODKey LODKey = FRealtimeMeshLODKey::FRealtimeMeshLODKey(0);
    FRealtimeMeshSectionConfig SecConfig = FRealtimeMeshSectionConfig(0);

    void Initialize(ARealtimeMeshActor* InParentActor, UMaterialInterface* Material, FVector InCenter, double InExtent) {
        FRealtimeMeshCollisionConfiguration cConfig;
        cConfig.bShouldFastCookMeshes = false;
        cConfig.bUseComplexAsSimpleCollision = true;
        cConfig.bDeformableMesh = false;
        cConfig.bUseAsyncCook = true;

        ChunkCenter = InCenter;
        ChunkExtent = InExtent;
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

        ChunkRtMesh->CreateSectionGroup(ChunkMeshData->MeshGroupKey, ChunkMeshData->MeshStream);
        ChunkRtMesh->ClearInternalFlags(EInternalObjectFlags::Async);
    };

    void UpdateMeshData(FInternalMeshBuffer InInternalBuffer) {
        int NumVerts = InInternalBuffer.Positions.Num();
        int NumTris = InInternalBuffer.Triangles.Num() / 3;
        FMeshStreamData MeshStream;
        auto PositionStream = MeshStream.GetPositionStream();
        auto TangentStream  = MeshStream.GetTangentStream();
        auto TexCoordStream = MeshStream.GetTexCoordStream();
        auto ColorStream = MeshStream.GetColorStream();
        auto TriangleStream = MeshStream.GetTriangleStream();
        auto PolygroupStream = MeshStream.GetPolygroupStream();
        
        //TODO::DEDUPLICATE HERE FInternalMeshBuffer InInternalBuffer
        //Update num verts

        // Now populate the streams with our unique vertices and triangles
        PositionStream.SetNumUninitialized(NumVerts);
        TangentStream.SetNumUninitialized(NumVerts);
        ColorStream.SetNumUninitialized(NumVerts);
        TexCoordStream.SetNumUninitialized(NumVerts);
        TriangleStream.SetNumUninitialized(NumTris);
        PolygroupStream.SetNumUninitialized(NumTris);

        for (int i = 0; i < NumVerts; i++) {
            
            PositionStream.Set(i, InInternalBuffer.Positions[i]);
            FRealtimeMeshTangentsHighPrecision tan;
            tan.SetNormal(FVector3f(InInternalBuffer.Normals[i]));
            TangentStream.Set(i, tan);
            TexCoordStream.Set(i, FVector2f(0,0));//TODO: REPLACE WITH PROPER TRIPLANAR UV
            ColorStream.Set(i, FColor::Green);//TODO::REPLACE WITH SAMPLE DENSITY... MAY BEED TO ADD TO INTERNAL BUFFER STRUCT
        }

        for (int i = 0; i < NumTris; i++) {
            int Idx0 = i * 3;
            int Idx1 = Idx0 + 1;
            int Idx2 = Idx1 + 1;

            TriangleStream.Set(i, FIndex3UI(Idx0, Idx1, Idx2));
            PolygroupStream.Set(i, 0);
        }

        UpdateMeshData(MeshStream);
    }

    //Update all chunk mesh data in async 2
    void UpdateMeshData(FMeshStreamData newMeshData) {
        ChunkMeshData->MeshStream = FRealtimeMeshStreamSet(newMeshData.MeshStream);
        IsDirty = true;
    }

    void UpdateComponent() {
        if (!IsDirty) return;
        if (ChunkMeshData) ChunkRtMesh->UpdateSectionGroup(ChunkMeshData->MeshGroupKey, ChunkMeshData->MeshStream)
            .Then([this](TFuture<ERealtimeMeshProxyUpdateStatus> update) {
            ChunkRtMesh->UpdateSectionConfig(ChunkMeshData->MeshSectionKey, SecConfig, true);
                });
        IsDirty = false;
    }
};
/**
 * 
 */
class VOXELPLUGIN_API FAdaptiveOctreeMT
{
public:
    TFunction<double(FVector)> DensityFunction;
    TSharedPtr<FAdaptiveOctreeNodeMT> Root;
    TArray<TSharedPtr<FAdaptiveOctreeNodeMT>> ChunkNodes;
    TArray<TSharedPtr<FMeshChunkMT>> MeshChunks;
    bool MeshChunksInitialized = false;

    bool UpdateLOD(FCameraInfo InCameraData, double InLODFactor);
    
    //block these actions if chunks not init
    //bool UpdateMesh(); //Updates the relevant nodes meshs, maps to MeshChunk buffer, and then updates the RTM

    void InitializeChunks(ARealtimeMeshActor* InParent, UMaterialInterface* InMaterial);

	FAdaptiveOctreeMT(TFunction<double(FVector)> InDensityFunction, FVector InCenterPosition, double InExtent, int InChunkMinMaxDepths[3]);

	~FAdaptiveOctreeMT();
};
