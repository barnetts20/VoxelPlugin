#include "FOceanQuadTreeNode.h"
#include "OceanSphereActor.h"
#include "Async/Async.h"

// ===========================================================================
// FOceanMeshChunk
// ===========================================================================

void FOceanMeshChunk::InitializeData(FVector InCenter)
{
    ChunkCenter = InCenter;
    InnerMeshData = MakeShared<FOceanStreamData>();
    InnerMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName("INNER"));
    InnerMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(InnerMeshData->MeshGroupKey, 0);
    EdgeMeshData = MakeShared<FOceanStreamData>();
    EdgeMeshData->MeshGroupKey = FRealtimeMeshSectionGroupKey::Create(LODKey, FName("EDGE"));
    EdgeMeshData->MeshSectionKey = FRealtimeMeshSectionKey::CreateForPolyGroup(EdgeMeshData->MeshGroupKey, 0);
}

void FOceanMeshChunk::InitializeComponent(AOceanSphereActor* InOwner)
{
    FRealtimeMeshCollisionConfiguration CConfig;
    CConfig.bShouldFastCookMeshes = false;
    CConfig.bUseComplexAsSimpleCollision = false;
    CConfig.bDeformableMesh = false;
    CConfig.bUseAsyncCook = false;

    ChunkRtMesh = NewObject<URealtimeMeshSimple>(InOwner);
    ChunkRtMesh->SetCollisionConfig(CConfig);
    ChunkRtMesh->SetupMaterialSlot(0, "OceanMaterial");

    ChunkRtComponent = NewObject<URealtimeMeshComponent>(InOwner, URealtimeMeshComponent::StaticClass());
    ChunkRtComponent->RegisterComponent();
    ChunkRtComponent->SetMaterial(0, InOwner->OceanMaterial);
    ChunkRtComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // Attach to MeshAttachmentRoot (absolute scale, inherits position + rotation).
    // Use relative location so the component moves with the actor automatically �
    // no world location bake needed, no re-init on actor movement.
    ChunkRtComponent->AttachToComponent(InOwner->GetMeshAttachmentRoot(), FAttachmentTransformRules::KeepRelativeTransform);
    ChunkRtComponent->SetRelativeLocation(ChunkCenter);

    ChunkRtComponent->SetRealtimeMesh(ChunkRtMesh.Get());
    ChunkRtComponent->SetRenderCustomDepth(true);
    ChunkRtComponent->SetVisibleInRayTracing(false);

    ChunkRtMesh->CreateSectionGroup(InnerMeshData->MeshGroupKey, FRealtimeMeshStreamSet());
    ChunkRtMesh->CreateSectionGroup(EdgeMeshData->MeshGroupKey, FRealtimeMeshStreamSet());

    IsInitialized = true;
}

void FOceanMeshChunk::DestroyComponent(AOceanSphereActor* InOwner)
{
    if (IsInitialized && ChunkRtComponent.IsValid())
    {
        InOwner->RemoveOwnedComponent(ChunkRtComponent.Get());
        ChunkRtComponent->DestroyComponent();
        ChunkRtComponent = nullptr;
        ChunkRtMesh = nullptr;
    }
    IsInitialized = false;
}

// ===========================================================================
// FOceanMeshGrid
// ===========================================================================

void FOceanMeshGrid::Build(int32 Res)
{
    const int32 ExtRes = Res + 2;
    const int32 tRes = ExtRes - 1;

    // --- Interior patch triangles (default winding, flipped at emit time if needed) ---
    PatchTriangles.Reset();
    for (int32 ix = 0; ix < tRes; ++ix)
    {
        for (int32 iy = 0; iy < tRes; ++iy)
        {
            bool bIsVirtual = (ix == 0 || iy == 0 || ix == tRes - 1 || iy == tRes - 1);
            bool bIsEdge = (ix == 1 || iy == 1 || ix == tRes - 2 || iy == tRes - 2);
            if (bIsVirtual || bIsEdge) continue;

            int32 tl = ix * ExtRes + iy, tr = tl + 1, bl = tl + ExtRes, br = bl + 1;
            if ((ix + iy) % 2 == 0)
            {
                PatchTriangles.Add(FIndex3UI(tl, bl, br));
                PatchTriangles.Add(FIndex3UI(tl, br, tr));
            }
            else
            {
                PatchTriangles.Add(FIndex3UI(tl, bl, tr));
                PatchTriangles.Add(FIndex3UI(tr, bl, br));
            }
        }
    }

    // --- 16 edge stitch variants ---
    for (uint8 flags = 0; flags < 16; ++flags)
        BuildEdgeVariant(ExtRes, Res, flags, EdgeTriangles[flags]);
}

void FOceanMeshGrid::BuildEdgeVariant(int32 ExtRes, int32 tRes, uint8 Flags,
    TArray<FIndex3UI>& Out)
{
    Out.Reset();

    bool bLeft = (Flags & 0x1) != 0;
    bool bRight = (Flags & 0x2) != 0;
    bool bTop = (Flags & 0x4) != 0;
    bool bBottom = (Flags & 0x8) != 0;

    FIndex3UI topOdd, bottomOdd, leftOdd, rightOdd;

    for (int32 i = 1; i < tRes; ++i)
    {
        { // TOP
            int32 x = i, y = 1;
            int32 tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { tl, tl + 1, bl, bl + 1 };
            if (x % 2 != 0) { topOdd = FIndex3UI(quad[3], quad[0], quad[2]); if (x != 1) Out.Add(FIndex3UI(quad[0], quad[3], quad[1])); }
            else {
                if (bTop) { topOdd[2] = quad[2]; Out.Add(topOdd); }
                else { Out.Add(topOdd); Out.Add(FIndex3UI(quad[0], quad[2], quad[1])); }
                if (x != tRes - 1) Out.Add(FIndex3UI(quad[1], quad[2], quad[3]));
            }
        }
        { // BOTTOM
            int32 x = i, y = tRes - 1;
            int32 tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { bl, bl + 1, tl, tl + 1 };
            if (x % 2 != 0) { bottomOdd = FIndex3UI(quad[3], quad[0], quad[1]); if (x != 1) Out.Add(FIndex3UI(quad[3], quad[2], quad[0])); }
            else {
                if (bBottom) { bottomOdd[2] = quad[1]; Out.Add(bottomOdd); }
                else { Out.Add(bottomOdd); Out.Add(FIndex3UI(quad[1], quad[3], quad[2])); }
                if (x != tRes - 1) Out.Add(FIndex3UI(quad[0], quad[1], quad[2]));
            }
        }
        { // LEFT
            int32 y = i, x = 1;
            int32 tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { tl, bl, tl + 1, bl + 1 };
            if (y % 2 != 0) { leftOdd = FIndex3UI(quad[0], quad[3], quad[2]); if (y != 1) Out.Add(FIndex3UI(quad[3], quad[0], quad[1])); }
            else {
                if (bLeft) { leftOdd[2] = quad[2]; Out.Add(leftOdd); }
                else { Out.Add(leftOdd); Out.Add(FIndex3UI(quad[2], quad[0], quad[1])); }
                if (y != tRes - 1) Out.Add(FIndex3UI(quad[2], quad[1], quad[3]));
            }
        }
        { // RIGHT
            int32 y = i, x = tRes - 1;
            int32 tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { tl + 1, bl + 1, tl, bl };
            if (y % 2 != 0) { rightOdd = FIndex3UI(quad[0], quad[3], quad[1]); if (y != 1) Out.Add(FIndex3UI(quad[2], quad[3], quad[0])); }
            else {
                if (bRight) { rightOdd[2] = quad[1]; Out.Add(rightOdd); }
                else { Out.Add(rightOdd); Out.Add(FIndex3UI(quad[3], quad[1], quad[2])); }
                if (y != tRes - 1) Out.Add(FIndex3UI(quad[1], quad[0], quad[2]));
            }
        }
    }
}

// ===========================================================================
// FOceanQuadTreeNode
// ===========================================================================

FOceanQuadTreeNode::FOceanQuadTreeNode(
    AOceanSphereActor* InOwner,
    FCubeTransform      InFaceTransform,
    FQuadIndex          InIndex,
    FVector             InCubeCenter,
    double              InSize,
    double              InOceanRadius,
    int32               InMinDepth,
    int32               InMaxDepth,
    int32               InChunkDepth)
    : Owner(InOwner)
    , Index(InIndex)
    , FaceTransform(InFaceTransform)
    , MinDepth(InMinDepth)
    , MaxDepth(InMaxDepth)
    , ChunkDepth(InChunkDepth)
    , CubeCenter(InCubeCenter)
    , OceanRadius(InOceanRadius)
    , Size(InSize)
{
    HalfSize = Size * 0.5;
    QuarterSize = HalfSize * 0.5;
    SphereCenter = CubeCenter.GetSafeNormal() * OceanRadius;
    WorldExtent = HalfSize * (OceanRadius / 500.0);

    int32 d = Index.GetDepth();
    NeighborLods[0] = d; NeighborLods[1] = d; NeighborLods[2] = d; NeighborLods[3] = d;
}

void FOceanQuadTreeNode::CollectLeaves(
    TSharedPtr<FOceanQuadTreeNode>          Root,
    TArray<TSharedPtr<FOceanQuadTreeNode>>& Out)
{
    TArray<TSharedPtr<FOceanQuadTreeNode>> Stack;
    Stack.Push(Root);
    while (Stack.Num() > 0)
    {
        TSharedPtr<FOceanQuadTreeNode> Node = Stack.Pop(EAllowShrinking::No);
        if (!Node.IsValid()) continue;
        if (Node->IsLeaf()) { Out.Add(Node); }
        else
        {
            for (int32 i = Node->Children.Num() - 1; i >= 0; --i)
                if (Node->Children[i].IsValid())
                    Stack.Add(Node->Children[i]);
        }
    }
}

// ---------------------------------------------------------------------------
// LOD
// ---------------------------------------------------------------------------

bool FOceanQuadTreeNode::ShouldSplit(double DistSq, double FOVScale, double ThresholdSq) const
{
    if (GetDepth() >= MaxDepth) return false;
    if (GetDepth() < MinDepth)  return true;
    double lhs = 2.0 * WorldExtent * FOVScale;
    return (lhs * lhs) > (ThresholdSq * DistSq);
}

bool FOceanQuadTreeNode::ShouldMerge(double ParentDistSq, double ParentFOVScale, double MergeThresholdSq) const
{
    if (!Parent.IsValid()) return false;
    TSharedPtr<FOceanQuadTreeNode> P = Parent.Pin();
    if (P->GetDepth() < MinDepth)    return false;
    if (P->GetDepth() <= ChunkDepth) return false;
    double lhs = 2.0 * P->WorldExtent * ParentFOVScale;
    return (lhs * lhs) < (MergeThresholdSq * ParentDistSq);
}

bool FOceanQuadTreeNode::TrySetLod(FVector CameraPos, double ThresholdSq, double MergeThresholdSq, double FOVScale)
{
    if (!IsLeaf()) return false;

    // CameraPos and SphereCenter are both in actor-local space � direct distance.
    // No GetActorLocation() needed since Tick converts the camera before passing it here.
    double DistSq = FMath::Max(FVector::DistSquared(CameraPos, SphereCenter), 1e-12);

    // Back-face cull: dot product of camera direction vs node direction, both local.
    double Dot = FVector::DotProduct(CameraPos, SphereCenter);
    if (Dot < 0.0 && GetDepth() >= MinDepth)
    {
        double CamDistSq = CameraPos.SizeSquared();
        double NodeDistSq = SphereCenter.SizeSquared();
        if ((Dot * Dot) > (0.04 * CamDistSq * NodeDistSq))
        {
            CanMerge = true;
            if (Index.GetQuadrant() == 3 && Parent.IsValid())
                Parent.Pin()->TryMerge();
            return !IsLeaf();
        }
    }

    if (ShouldSplit(DistSq, FOVScale, ThresholdSq))
    {
        CanMerge = false;
        FOceanQuadTreeNode::Split(AsShared());
        return true;
    }
    else if (ShouldMerge(DistSq, FOVScale, MergeThresholdSq))
    {
        CanMerge = true;
        if (Index.GetQuadrant() == 3 && Parent.IsValid())
        {
            Parent.Pin()->TryMerge();
            return !Parent.Pin()->IsLeaf() ? false : true;
        }
    }
    else { CanMerge = false; }

    return false;
}

// ---------------------------------------------------------------------------
// Split / Merge
// ---------------------------------------------------------------------------

void FOceanQuadTreeNode::Split(TSharedPtr<FOceanQuadTreeNode> inNode)
{
    if (!inNode.IsValid() || !inNode->IsLeaf() || inNode->IsRestructuring) return;
    inNode->IsRestructuring = true;

    FVector2d childOffsets[4] = {
        FVector2d(-inNode->QuarterSize, -inNode->QuarterSize),
        FVector2d(-inNode->QuarterSize,  inNode->QuarterSize),
        FVector2d(inNode->QuarterSize, -inNode->QuarterSize),
        FVector2d(inNode->QuarterSize,  inNode->QuarterSize)
    };

    for (int32 i = 0; i < 4; ++i)
    {
        FVector childCube = inNode->CubeCenter;
        childCube[inNode->FaceTransform.AxisMap[0]] += inNode->FaceTransform.AxisDir[0] * childOffsets[i].X;
        childCube[inNode->FaceTransform.AxisMap[1]] += inNode->FaceTransform.AxisDir[1] * childOffsets[i].Y;

        auto Child = MakeShared<FOceanQuadTreeNode>(
            inNode->Owner, inNode->FaceTransform, inNode->Index.GetChildIndex(i),
            childCube, inNode->HalfSize, inNode->OceanRadius,
            inNode->MinDepth, inNode->MaxDepth, inNode->ChunkDepth);

        Child->Parent = inNode.ToWeakPtr();
        Child->ChunkAnchorCenter = inNode->ChunkAnchorCenter;
        inNode->Children.Add(Child);
    }

    inNode->IsRestructuring = false;
}

void FOceanQuadTreeNode::TryMerge()
{
    if (IsLeaf()) return;
    for (auto& Child : Children)
        if (!Child->CanMerge) return;
    FOceanQuadTreeNode::Merge(AsShared());
}

void FOceanQuadTreeNode::Merge(TSharedPtr<FOceanQuadTreeNode> inNode)
{
    if (!inNode.IsValid() || inNode->IsLeaf() || inNode->IsRestructuring) return;
    inNode->IsRestructuring = true;
    RemoveChildren(inNode);
    inNode->IsRestructuring = false;
}

void FOceanQuadTreeNode::RemoveChildren(TSharedPtr<FOceanQuadTreeNode> InNode)
{
    if (!InNode.IsValid()) return;
    for (auto& Child : InNode->Children)
    {
        if (Child.IsValid())
            RemoveChildren(Child);
    }
    InNode->Children.Reset();
}

void FOceanQuadTreeNode::SplitToDepth(int32 TargetDepth)
{
    if (GetDepth() >= TargetDepth) return;

    FVector2d childOffsets[4] = {
        FVector2d(-QuarterSize, -QuarterSize),
        FVector2d(-QuarterSize,  QuarterSize),
        FVector2d(QuarterSize, -QuarterSize),
        FVector2d(QuarterSize,  QuarterSize)
    };

    for (int32 i = 0; i < 4; ++i)
    {
        FVector childCube = CubeCenter;
        childCube[FaceTransform.AxisMap[0]] += FaceTransform.AxisDir[0] * childOffsets[i].X;
        childCube[FaceTransform.AxisMap[1]] += FaceTransform.AxisDir[1] * childOffsets[i].Y;

        auto Child = MakeShared<FOceanQuadTreeNode>(
            Owner, FaceTransform, Index.GetChildIndex(i),
            childCube, HalfSize, OceanRadius,
            MinDepth, MaxDepth, ChunkDepth);

        Child->Parent = AsShared().ToWeakPtr();

        if (Child->GetDepth() == ChunkDepth)
            Child->ChunkAnchorCenter = Child->SphereCenter;
        else
            Child->ChunkAnchorCenter = ChunkAnchorCenter;

        Children.Add(Child);
    }

    for (auto& Child : Children)
        Child->SplitToDepth(TargetDepth);
}

// ---------------------------------------------------------------------------
// Neighbors
// ---------------------------------------------------------------------------

bool FOceanQuadTreeNode::CheckNeighbors()
{
    uint8 myQuadrant = (uint8)Index.GetQuadrant();
    bool  bChanged = false;

    auto Check = [&](EdgeOrientation Edge)
        {
            TSharedPtr<FOceanQuadTreeNode> Neighbor = Owner->GetNodeByIndex(Index.GetNeighborIndex(Edge));
            if (!Neighbor.IsValid()) return;
            int32 d = Neighbor->GetDepth();
            if (NeighborLods[(uint8)Edge] != d) { NeighborLods[(uint8)Edge] = d; bChanged = true; }
        };

    switch (myQuadrant)
    {
    case (uint8)EChildPosition::BOTTOM_LEFT:  Check(EdgeOrientation::LEFT);  Check(EdgeOrientation::UP);   break;
    case (uint8)EChildPosition::TOP_LEFT:     Check(EdgeOrientation::LEFT);  Check(EdgeOrientation::DOWN); break;
    case (uint8)EChildPosition::BOTTOM_RIGHT: Check(EdgeOrientation::RIGHT); Check(EdgeOrientation::UP);   break;
    case (uint8)EChildPosition::TOP_RIGHT:    Check(EdgeOrientation::RIGHT); Check(EdgeOrientation::DOWN); break;
    }

    return bChanged;
}

// ---------------------------------------------------------------------------
// SampleMaxDepth
// ---------------------------------------------------------------------------

void FOceanQuadTreeNode::SampleMaxDepth()
{
    if (!Owner) return;
    FDensitySampleCompositor* Comp = Owner->GetCompositor().Get();
    if (!Comp) return;

    // Sample the 4 face corners — cheap proxy for MaxVertexDepth.
    const int32  U = FaceTransform.AxisMap[0];
    const int32  V = FaceTransform.AxisMap[1];
    const int32  DU = FaceTransform.AxisDir[0];
    const int32  DV = FaceTransform.AxisDir[1];

    float SX[4], SY[4], SZ[4], Out[4];
    for (int32 c = 0; c < 4; ++c)
    {
        FVector Corner = CubeCenter;
        Corner[U] += DU * HalfSize * ((c & 1) ? 1.0 : -1.0);
        Corner[V] += DV * HalfSize * ((c & 2) ? 1.0 : -1.0);
        FVector S = Corner.GetSafeNormal() * OceanRadius;
        SX[c] = (float)S.X;
        SY[c] = (float)S.Y;
        SZ[c] = (float)S.Z;
    }
    FSampleInput Input(SX, SY, SZ, 4);
    Comp->Sample(Input, Out);
    MaxVertexDepth = FMath::Max(FMath::Max(Out[0], Out[1]), FMath::Max(Out[2], Out[3]));
}