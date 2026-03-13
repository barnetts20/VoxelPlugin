#pragma once

#include "FAdaptiveOctreeNode.h"
#include "RealtimeMeshActor.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>

using namespace RealtimeMesh;

struct VOXELPLUGIN_API FNodeEdge
{
    FVoxelCorner* Corners[2];
    double Size;
    bool SignChange;
    double Distance;
    FVector EdgeDirection;
    int Axis;
    FVector ZeroCrossingPoint;

    // Constructor
    FNodeEdge() : Size(0), SignChange(false), Distance(0), Axis(0) {}

    FNodeEdge(FVoxelCorner* InCorner1, FVoxelCorner* InCorner2)
    {
        Corners[0] = InCorner1;
        Corners[1] = InCorner2;

        double d1 = InCorner1->Density;
        double d2 = InCorner2->Density;

        SignChange = (d1 < 0) != (d2 < 0);

        // Determine which corner is positive and which is negative
        FVoxelCorner* PosCorner = (d1 > d2) ? InCorner1 : InCorner2;
        FVoxelCorner* NegCorner = (d1 > d2) ? InCorner2 : InCorner1;

        Size = FVector::Dist(InCorner1->GetPosition(), InCorner2->GetPosition());
        Distance = Size;

        // Compute edge direction: Always point from positive to negative
        EdgeDirection = (NegCorner->GetPosition() - PosCorner->GetPosition()).GetSafeNormal();

        // Safe axis detection: find the axis with the largest delta
        // This is immune to floating point noise at large coordinates
        FVector Delta = (InCorner2->GetPosition() - InCorner1->GetPosition()).GetAbs();
        if (Delta.X > Delta.Y && Delta.X > Delta.Z)
            Axis = 0;
        else if (Delta.Y > Delta.X && Delta.Y > Delta.Z)
            Axis = 1;
        else
            Axis = 2;

        if (SignChange) {
            // Stable zero-crossing interpolation
            // t = d1 / (d1 - d2) gives the parametric position along the edge
            // where the sign change occurs. Clamp to [0,1] for safety.
            double Denominator = d1 - d2;
            if (FMath::Abs(Denominator) < 1e-12) {
                // Both densities essentially equal — place at midpoint
                ZeroCrossingPoint = (InCorner1->GetPosition() + InCorner2->GetPosition()) * 0.5;
            }
            else {
                double t = d1 / Denominator;
                t = FMath::Clamp(t, 0.0, 1.0);
                ZeroCrossingPoint = InCorner1->GetPosition() + t * (InCorner2->GetPosition() - InCorner1->GetPosition());
            }
        }
        else {
            ZeroCrossingPoint = (InCorner1->GetPosition() + InCorner2->GetPosition()) * 0.5;
        }
    }

    bool IsCongruent(const FNodeEdge& Other) const {
        // Both corners must match (in either order) AND axis must match
        bool CornersMatch =
            (Corners[0]->GetPosition().Equals(Other.Corners[0]->GetPosition(), .01)
                && Corners[1]->GetPosition().Equals(Other.Corners[1]->GetPosition(), .01))
            || (Corners[0]->GetPosition().Equals(Other.Corners[1]->GetPosition(), .01)
                && Corners[1]->GetPosition().Equals(Other.Corners[0]->GetPosition(), .01));

        return CornersMatch && Axis == Other.Axis;
    }

    // Equality operator for ensuring uniqueness
    bool operator==(const FNodeEdge& Other) const
    {
        return IsCongruent(Other) && Other.EdgeDirection == EdgeDirection;
    }
};

struct VOXELPLUGIN_API FEdgeKey
{
    int64 X0, Y0, Z0;  // Quantized corner 0 (sorted so min corner is always first)
    int64 X1, Y1, Z1;  // Quantized corner 1
    int32 Axis;

    // Quantization grid — 0.001 is well within the 0.01 epsilon used in IsCongruent
    static constexpr double GridSize = 0.001;

    static int64 Quantize(double V)
    {
        return FMath::RoundToInt64(V / GridSize);
    }

    FEdgeKey() = default;

    FEdgeKey(const FNodeEdge& Edge)
    {
        int64 ax = Quantize(Edge.Corners[0]->Position.X);
        int64 ay = Quantize(Edge.Corners[0]->Position.Y);
        int64 az = Quantize(Edge.Corners[0]->Position.Z);
        int64 bx = Quantize(Edge.Corners[1]->Position.X);
        int64 by = Quantize(Edge.Corners[1]->Position.Y);
        int64 bz = Quantize(Edge.Corners[1]->Position.Z);

        // Canonical ordering: ensure (corner0 < corner1) so order doesn't matter
        if (ax < bx || (ax == bx && ay < by) || (ax == bx && ay == by && az < bz))
        {
            X0 = ax; Y0 = ay; Z0 = az;
            X1 = bx; Y1 = by; Z1 = bz;
        }
        else
        {
            X0 = bx; Y0 = by; Z0 = bz;
            X1 = ax; Y1 = ay; Z1 = az;
        }

        Axis = Edge.Axis;
    }

    bool operator==(const FEdgeKey& Other) const
    {
        return X0 == Other.X0 && Y0 == Other.Y0 && Z0 == Other.Z0
            && X1 == Other.X1 && Y1 == Other.Y1 && Z1 == Other.Z1
            && Axis == Other.Axis;
    }
};

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