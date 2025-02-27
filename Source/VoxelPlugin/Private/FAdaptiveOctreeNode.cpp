#include "FAdaptiveOctreeNode.h"
#include "FSparseOctree.h"
#include "FAdaptiveOctree.h"

bool FAdaptiveOctreeNode::IsLeaf()
{
    return bIsLeaf;
}

bool FAdaptiveOctreeNode::IsRoot()
{
    return !Parent.IsValid();
}

// Root Constructor
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, FVector InCenter, double InExtent, int InMinDepth, int InMaxDepth)
{
    DensityFunction = InDensityFunction;
    Center = InCenter;
    Extent = FMath::Max(InExtent, 0.0);
    Density = 0.0;
    DepthBounds[0] = InMinDepth;
    DepthBounds[1] = InMaxDepth;

    for (int i = 0; i < 8; i++) {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        Corners.Add(FNodeCorner(i, CornerPosition, DensityFunction(CornerPosition)));
        Children[i] = nullptr;
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]], 0);
        Edges.Add(anEdge);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    ComputeDualContourPosition();
}

// Child Constructor
FAdaptiveOctreeNode::FAdaptiveOctreeNode(TFunction<double(FVector)> InDensityFunction, TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 ChildIndex)
{
    Parent = InParent;
    DensityFunction = InDensityFunction;
    Center = InParent->Center + Offsets[ChildIndex] * (InParent->Extent * 0.5);
    Extent = InParent->Extent * 0.5;
    Density = 0.0;
    DepthBounds[0] = InParent->DepthBounds[0];
    DepthBounds[1] = InParent->DepthBounds[1];
    TreeIndex = InParent->TreeIndex;
    TreeIndex.Add(ChildIndex);

    for (int i = 0; i < 8; i++) {
        FVector CornerPosition = Center + Offsets[i] * Extent;
        Corners.Add(FNodeCorner(i, CornerPosition, DensityFunction(CornerPosition)));
        Children[i] = nullptr;
    }

    for (int EdgeIndex = 0; EdgeIndex < 12; EdgeIndex++)
    {
        FNodeEdge anEdge = FNodeEdge(Corners[EdgePairs[EdgeIndex][0]], Corners[EdgePairs[EdgeIndex][1]], TreeIndex.Num());
        Edges.Add(anEdge);
        if (anEdge.SignChange) SignChangeEdges.Add(anEdge);
    }

    //Look up user density alterations in Sparse octree for this index? Would actually need to happen before all density evaluations.
    //Each node can calculate its own density alteration via the sparse octree stored values, as well as its composite density modification
    //By accumulating its parent node density alterations, this final value is applied to the density calculated by the density function before
   
    ComputeDualContourPosition();
}

void FAdaptiveOctreeNode::Split()
{
    if (!bIsLeaf) return; // Already split
    for (uint8 i = 0; i < 8; i++)
    {
        Children[i] = MakeShared<FAdaptiveOctreeNode>(DensityFunction, AsShared(), i);
    }
    bIsLeaf = false;
}

void FAdaptiveOctreeNode::Merge()
{
    if (bIsLeaf) return; // Already merged

    TArray<TSharedPtr<FAdaptiveOctreeNode>> NodesToDelete;
    NodesToDelete.Append(Children); // Add children to cleanup list

    while (NodesToDelete.Num() > 0)
    {
        TSharedPtr<FAdaptiveOctreeNode> Node = NodesToDelete.Pop();
        if (!Node.IsValid()) continue;

        // Add children of this node to the stack before clearing
        if (!Node->bIsLeaf) {
            NodesToDelete.Append(Node->Children);
        }
            
        // Properly clear shared pointers
        for (int i = 0; i < 8; i++) {
            Node->Children[i].Reset();
        }
        Node.Reset();
    }
    bIsLeaf = true;
}


bool FAdaptiveOctreeNode::ShouldSplit(FVector InCameraPosition, double InLodDistanceFactor)
{
    return TreeIndex.Num() < DepthBounds[0] || (FVector::Dist(DualContourPosition, InCameraPosition) < Extent * (InLodDistanceFactor + TreeIndex.Num()) && TreeIndex.Num() < DepthBounds[1]);
}

bool FAdaptiveOctreeNode::ShouldMerge(FVector InCameraPosition, double InLodDistanceFactor)
{
    bool CanMerge = true;
    for (const auto& Child : Children)
    {
        if (!Child.IsValid() || !Child->IsLeaf())
        {
            CanMerge = false;
            break; 
        }
    }

    return CanMerge && (FVector::Dist(DualContourPosition, InCameraPosition) > Extent * (InLodDistanceFactor + TreeIndex.Num())) && TreeIndex.Num() >= DepthBounds[0];
}

void AppendUniqueEdges(TArray<FNodeEdge>& OutEdges, TArray<FNodeEdge> AppendEdges) {
    for (FNodeEdge anEdge : AppendEdges) {
        OutEdges.AddUnique(anEdge);
    }
}

bool FAdaptiveOctreeNode::UpdateLod(FVector InCameraPosition, double InLodDistanceFactor, TArray<FNodeEdge>& OutEdges)
{
    if (IsLeaf())
    {
        if (ShouldSplit(InCameraPosition, InLodDistanceFactor))
        {
            Split(); //Push child sign change edges into return array
            for (TSharedPtr<FAdaptiveOctreeNode> Child : Children) {
                AppendUniqueEdges(OutEdges, Child->GetSignChangeEdges());
            }
            return true; // A split occurred
        }
        else if (Parent.IsValid() && Parent.Pin()->ShouldMerge(InCameraPosition, InLodDistanceFactor) && TreeIndex.Last() == 7)
        {
            Parent.Pin()->Merge(); //Push Parent sign change edges into return array
            AppendUniqueEdges(OutEdges, Parent.Pin()->GetSignChangeEdges());
            return true; // A merge occurred
        }
        else {
            AppendUniqueEdges(OutEdges, GetSignChangeEdges());
        }
    }
    else
    {
        bool bAnyChanges = false;
        for (int i = 0; i < 8; i++)
        {
            if (Children[i]->UpdateLod(InCameraPosition, InLodDistanceFactor, OutEdges))
            {
                bAnyChanges = true;
            }
        }
        return bAnyChanges; // Return true if any child changed
    }
    return false; // No changes occurred
}

// Retrieves all surface nodes for meshing
TArray<FNodeEdge> FAdaptiveOctreeNode::GetSurfaceEdges()
{
    TArray<FNodeEdge> SurfaceEdges;
    TQueue<TSharedPtr<FAdaptiveOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(this->AsShared());

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode) continue;

        if (CurrentNode->IsLeaf() && CurrentNode->IsSurfaceNode)
        {
            auto edges = CurrentNode->GetSignChangeEdges();
            for (auto edge : edges) {
                SurfaceEdges.AddUnique(edge);
            }
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (CurrentNode->Children[i]) NodeQueue.Enqueue(CurrentNode->Children[i]);
            }
        }
    }

    return SurfaceEdges;
}

// Retrieves all surface nodes for meshing
TArray<TSharedPtr<FAdaptiveOctreeNode>> FAdaptiveOctreeNode::GetSurfaceNodes()
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> SurfaceNodes;
    TQueue<TSharedPtr<FAdaptiveOctreeNode>> NodeQueue;
    NodeQueue.Enqueue(this->AsShared());

    while (!NodeQueue.IsEmpty())
    {
        TSharedPtr<FAdaptiveOctreeNode> CurrentNode;
        NodeQueue.Dequeue(CurrentNode);

        if (!CurrentNode) continue;

        if (CurrentNode->IsLeaf() && CurrentNode->IsSurfaceNode)
        {
            SurfaceNodes.Add(CurrentNode->AsShared());
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (CurrentNode->Children[i]) NodeQueue.Enqueue(CurrentNode->Children[i]);
            }
        }
    }

    return SurfaceNodes;
}
//bool FAdaptiveOctreeNode::ContainsOverlappingEdge(FNodeEdge InEdgeToCheck)
//{
//    for (auto edge : Edges) {
//        if (edge.Axis != InEdgeToCheck.Axis) return false;
//        
//        if(edge.Axis == 0 && edge.ZeroCrossingPoint.Y == InEdgeToCheck.ZeroCrossingPoint.Y )
//    }
//    return true;
//}
//TODO: Lets try converting this lod method
//void QuadTreeNode::TrySetLod() {
//    if (this->IsInitialized && this->IsLeaf()) {
//        double k = 8;
//        double fov = this->ParentActor->GetCameraFOV();
//        FVector lastCamPos = this->ParentActor->GetLastCameraPosition();
//        auto lastCamRot = this->ParentActor->GetLastCameraRotation();
//
//        //Since we are doing origin rebasing frequently, the actors location can "change" arbitrarily and needs to be accounted for
//        FVector planetCenter = this->ParentActor->GetActorLocation();
//        FVector adjustedCentroid = this->NodeCentroid * this->ParentActor->GetActorScale().X + planetCenter;
//        auto parentCenter = adjustedCentroid;
//        auto parentSize = this->MaxNodeRadius * this->ParentActor->GetActorScale().X;
//
//        double planetRadius = FVector::Distance(this->ParentActor->GetActorLocation(), adjustedCentroid);
//
//        auto parent = this->Parent;
//
//        if (parent.IsValid()) {
//            parentCenter = parent.Pin()->NodeCentroid * this->ParentActor->GetActorScale().X + planetCenter;
//            parentSize = parent.Pin()->MaxNodeRadius * this->ParentActor->GetActorScale().X;
//        }
//
//        double d1 = FVector::Distance(lastCamPos, adjustedCentroid);
//        double d2 = FVector::Distance(lastCamPos, parentCenter);
//        if (this->GetDepth() < this->MinDepth || (this->GetDepth() < this->MaxDepth && k * this->MaxNodeRadius * this->ParentActor->GetActorScale().X > s(d1, fov))) {
//            this->CanMerge = false;
//            if (this->LastRenderedState) {
//                this->AsyncSplit(this->AsShared());
//            }
//        }
//        else if ((parent.IsValid() && parent.Pin()->GetDepth() >= this->MinDepth) && k * parentSize < s(d2, fov)) {
//            this->CanMerge = true;
//            parent.Pin()->TryMerge();
//        }
//        else {
//            this->CanMerge = false;
//        }
//
//    }
//}

TArray<FNodeEdge>& FAdaptiveOctreeNode::GetSignChangeEdges()
{
    return SignChangeEdges;
}

// Compute Dual Contour Position
void FAdaptiveOctreeNode::ComputeDualContourPosition()
{
    TArray<FVector> SurfaceCrossings;
    for (FNodeEdge anEdge : SignChangeEdges) {
        SurfaceCrossings.Add(anEdge.ZeroCrossingPoint);
    }

    if (SignChangeEdges.Num() > 0) {
        DualContourPosition = Algo::Accumulate(SurfaceCrossings, FVector::ZeroVector) / SurfaceCrossings.Num();
        
        DualContourNormal = FVector(0, 0, 0);
        DualContourNormal.X = (Corners[1].Density + Corners[3].Density + Corners[5].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[2].Density + Corners[4].Density + Corners[6].Density);
        DualContourNormal.Y = (Corners[2].Density + Corners[3].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[4].Density + Corners[5].Density);
        DualContourNormal.Z = (Corners[4].Density + Corners[5].Density + Corners[6].Density + Corners[7].Density) -
            (Corners[0].Density + Corners[1].Density + Corners[2].Density + Corners[3].Density);

        if (!DualContourNormal.IsNearlyZero(KINDA_SMALL_NUMBER))
        {
            DualContourNormal.Normalize();
        }

        IsSurfaceNode = true;
    }
    else {
        DualContourNormal = FVector(0, 0, 0);
        DualContourPosition = Center;
        IsSurfaceNode = false;
    }
}

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
    // Pre-compute face position and tolerance
    const float tolerance = 0.001f * ChunkExtent;
    const float facePos = ChunkCenter[Edge.Axis] + (Coef * ChunkExtent);

    // Check if the edge's zero-crossing point lies on this face
    // Only need to check the coordinates perpendicular to the edge axis
    const int axis1 = (Edge.Axis + 1) % 3;  // First perpendicular axis
    const int axis2 = (Edge.Axis + 2) % 3;  // Second perpendicular axis

    return FMath::Abs(Edge.ZeroCrossingPoint[axis1] - (ChunkCenter[axis1] + Coef * ChunkExtent)) < tolerance ||
        FMath::Abs(Edge.ZeroCrossingPoint[axis2] - (ChunkCenter[axis2] + Coef * ChunkExtent)) < tolerance;
}

bool FMeshChunk::IsNodeInChunk(const FAdaptiveOctreeFlatNode& Node) {
    // Calculate extents once
    const FVector extentVec(ChunkExtent);
    FVector distFromCenter = (Node.Center - ChunkCenter).GetAbs();
    return distFromCenter.X <= ChunkExtent && 
           distFromCenter.Y <= ChunkExtent && 
           distFromCenter.Z <= ChunkExtent;
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