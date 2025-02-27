#include "FMeshingStructs.h"
#include "FAdaptiveOctree.h"
#include "FAdaptiveOctreeNode.h"

// FMeshStreamData implementation
FMeshStreamData::FMeshStreamData() {
    TRealtimeMeshStreamBuilder<FVector, FVector3f> PositionBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>()));
    TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision> TangentBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>()));
    TRealtimeMeshStreamBuilder<TIndex3<uint32>> TrianglesBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>()));
    TRealtimeMeshStreamBuilder<uint32, uint16> PolygroupsBuilder(MeshStream.AddStream(FRealtimeMeshStreams::PolyGroups, GetRealtimeMeshBufferLayout<uint16>()));
    TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf> TexCoordsBuilder(MeshStream.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<FVector2DHalf>()));
    TRealtimeMeshStreamBuilder<FColor> ColorBuilder(MeshStream.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>()));
}

TRealtimeMeshStreamBuilder<FVector, FVector3f> FMeshStreamData::GetPositionStream() {
    return TRealtimeMeshStreamBuilder<FVector, FVector3f>(*MeshStream.Find((FRealtimeMeshStreams::Position)));
}

TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision> FMeshStreamData::GetTangentStream() {
    return TRealtimeMeshStreamBuilder<FRealtimeMeshTangentsHighPrecision, FRealtimeMeshTangentsNormalPrecision>(*MeshStream.Find(FRealtimeMeshStreams::Tangents));
}

TRealtimeMeshStreamBuilder<TIndex3<uint32>> FMeshStreamData::GetTriangleStream() {
    return TRealtimeMeshStreamBuilder<TIndex3<uint32>>(*MeshStream.Find(FRealtimeMeshStreams::Triangles));
}

TRealtimeMeshStreamBuilder<uint32, uint16> FMeshStreamData::GetPolygroupStream() {
    return TRealtimeMeshStreamBuilder<uint32, uint16>(*MeshStream.Find(FRealtimeMeshStreams::PolyGroups));
}

TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf> FMeshStreamData::GetTexCoordStream() {
    return TRealtimeMeshStreamBuilder<FVector2f, FVector2DHalf>(*MeshStream.Find(FRealtimeMeshStreams::TexCoords));
}

TRealtimeMeshStreamBuilder<FColor> FMeshStreamData::GetColorStream() {
    return TRealtimeMeshStreamBuilder<FColor>(*MeshStream.Find(FRealtimeMeshStreams::Color));
}

void FMeshStreamData::ResetStreams() {
    if (auto* Position = MeshStream.Find(FRealtimeMeshStreams::Position)) Position->Empty();
    if (auto* Tangent = MeshStream.Find(FRealtimeMeshStreams::Tangents)) Tangent->Empty();
    if (auto* Triangle = MeshStream.Find(FRealtimeMeshStreams::Triangles)) Triangle->Empty();
    if (auto* Polygroup = MeshStream.Find(FRealtimeMeshStreams::PolyGroups)) Polygroup->Empty();
    if (auto* TexCoord = MeshStream.Find(FRealtimeMeshStreams::TexCoords)) TexCoord->Empty();
    if (auto* Color = MeshStream.Find(FRealtimeMeshStreams::Color)) Color->Empty();
}

// FMeshChunk implementation
void FMeshChunk::Initialize(ARealtimeMeshActor* InParentActor, UMaterialInterface* Material,
    TSharedPtr<FAdaptiveOctreeNode> InChunkNode, TSharedPtr<FAdaptiveOctree> InTree) {
    FRealtimeMeshCollisionConfiguration cConfig;
    cConfig.bShouldFastCookMeshes = false;
    cConfig.bUseComplexAsSimpleCollision = true;
    cConfig.bDeformableMesh = false;
    cConfig.bUseAsyncCook = true;

    OwningOctree = InTree;
    ChunkCenter = InChunkNode->Center;
    ChunkExtent = InChunkNode->Extent;
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
}

FVector2f FMeshChunk::ComputeTriplanarUV(FVector Position, FVector Normal) {
    FVector2f UV;
    FVector AbsNormal = Normal.GetAbs();

    if (AbsNormal.X > AbsNormal.Y && AbsNormal.X > AbsNormal.Z) {
        UV = FVector2f(Position.Y, Position.Z) * 0.0001f;
    }
    else if (AbsNormal.Y > AbsNormal.X && AbsNormal.Y > AbsNormal.Z) {
        UV = FVector2f(Position.X, Position.Z) * 0.0001f;
    }
    else {
        UV = FVector2f(Position.X, Position.Y) * 0.0001f;
    }

    return UV;
}

bool FMeshChunk::IsEdgeOnChunkFace(const FNodeEdge& Edge, int Coef) {
    // Get face coordinates
    FVector negFace = ChunkCenter + (Coef * FVector(ChunkExtent));

    // Tolerance scaled to chunk size
    const float tolerance = 0.001f * ChunkExtent;

    // Check based on edge axis and position
    if (Edge.Axis == 0) { // X-aligned edge
        // Check if Y or Z matches face
        return FMath::Abs(Edge.ZeroCrossingPoint.Y - negFace.Y) < tolerance ||
            FMath::Abs(Edge.ZeroCrossingPoint.Z - negFace.Z) < tolerance;
    }
    else if (Edge.Axis == 1) { // Y-aligned edge
        // Check if X or Z matches face
        return FMath::Abs(Edge.ZeroCrossingPoint.X - negFace.X) < tolerance ||
            FMath::Abs(Edge.ZeroCrossingPoint.Z - negFace.Z) < tolerance;
    }
    else if (Edge.Axis == 2) { // Z-aligned edge
        // Check if X or Y matches face
        return FMath::Abs(Edge.ZeroCrossingPoint.X - negFace.X) < tolerance ||
            FMath::Abs(Edge.ZeroCrossingPoint.Y - negFace.Y) < tolerance;
    }

    return false;
}

bool FMeshChunk::IsNodeInChunk(FAdaptiveOctreeFlatNode Node) {
    FVector chunkMin = ChunkCenter - FVector(ChunkExtent);
    FVector chunkMax = ChunkCenter + FVector(ChunkExtent);

    // Use the node's center to determine if it's in the chunk
    return (Node.Center.X >= chunkMin.X && Node.Center.X <= chunkMax.X &&
        Node.Center.Y >= chunkMin.Y && Node.Center.Y <= chunkMax.Y &&
        Node.Center.Z >= chunkMin.Z && Node.Center.Z <= chunkMax.Z);
}

bool FMeshChunk::ShouldProcessEdge(const FNodeEdge& Edge, const TArray<FAdaptiveOctreeFlatNode>& SampledNodes) {
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

void FMeshChunk::UpdateMeshData() {
    ChunkMeshData->ResetStreams();
    auto PositionStream = ChunkMeshData->GetPositionStream();
    auto TangentStream = ChunkMeshData->GetTangentStream();
    auto ColorStream = ChunkMeshData->GetColorStream();
    auto TexCoordStream = ChunkMeshData->GetTexCoordStream();
    auto TriangleStream = ChunkMeshData->GetTriangleStream();
    auto PolygroupStream = ChunkMeshData->GetPolygroupStream();
    int idx = 0;
    int triIdx = 0;

    for (auto currentEdge : ChunkEdges) {
        // Data only queries could return flat structs instead of recursive nodes perhaps
        TArray<FAdaptiveOctreeFlatNode> nodesToMesh = OwningOctree->SampleSurfaceNodesAroundEdge(currentEdge);
        if (!ShouldProcessEdge(currentEdge, nodesToMesh)) continue;

        int32 IndexA = idx++;
        int32 IndexB = idx++;
        int32 IndexC = idx++;

        PositionStream.Add(nodesToMesh[0].DualContourPosition);
        FRealtimeMeshTangentsHighPrecision tan0;
        tan0.SetNormal(FVector3f(nodesToMesh[0].DualContourNormal));
        TangentStream.Add(tan0);
        ColorStream.Add(FColor::Green);
        TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[0].DualContourPosition, nodesToMesh[0].DualContourNormal));

        PositionStream.Add(nodesToMesh[1].DualContourPosition);
        FRealtimeMeshTangentsHighPrecision tan1;
        tan1.SetNormal(FVector3f(nodesToMesh[1].DualContourNormal));
        TangentStream.Add(tan1);
        ColorStream.Add(FColor::Green);
        TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[1].DualContourPosition, nodesToMesh[1].DualContourNormal));

        PositionStream.Add(nodesToMesh[2].DualContourPosition);
        FRealtimeMeshTangentsHighPrecision tan2;
        tan2.SetNormal(FVector3f(nodesToMesh[2].DualContourNormal));
        TangentStream.Add(tan2);
        ColorStream.Add(FColor::Green);
        TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[2].DualContourPosition, nodesToMesh[2].DualContourNormal));

        TriangleStream.Add(FIndex3UI(IndexA, IndexB, IndexC));
        PolygroupStream.Add(0);

        triIdx++;

        // Handle the fourth vertex if a quad exists
        if (nodesToMesh.Num() == 4) {
            int32 IndexD = idx++;

            PositionStream.Add(nodesToMesh[3].DualContourPosition);
            FRealtimeMeshTangentsHighPrecision tan3;
            tan3.SetNormal(FVector3f(nodesToMesh[3].DualContourNormal));
            TangentStream.Add(tan3);
            ColorStream.Add(FColor::Green);
            TexCoordStream.Add(ComputeTriplanarUV(nodesToMesh[3].DualContourPosition, nodesToMesh[3].DualContourNormal));

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
}

void FMeshChunk::UpdateComponent() {
    if (ChunkMeshData) {
        ChunkRtMesh->UpdateSectionGroup(ChunkMeshData->MeshGroupKey, ChunkMeshData->MeshStream).Then(
            [this](TFuture<ERealtimeMeshProxyUpdateStatus> update) {
                ChunkRtMesh->UpdateSectionConfig(ChunkMeshData->MeshSectionKey, SecConfig, true);
            });
    }
}