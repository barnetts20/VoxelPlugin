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
    CConfig.bUseAsyncCook = true;

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
    , CubeCenter(InCubeCenter)
    , MinDepth(InMinDepth)
    , MaxDepth(InMaxDepth)
    , ChunkDepth(InChunkDepth)
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

    for (int32 i = 0; i < 4; ++i)
        inNode->Children[i]->GenerateMeshData();

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
    {
        Child->GenerateMeshData();
        Child->SplitToDepth(TargetDepth);
    }
}

// ---------------------------------------------------------------------------
// Neighbors
// ---------------------------------------------------------------------------

bool FOceanQuadTreeNode::CheckNeighbors()
{
    if (!HasGenerated) return false;

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
// Mesh data generation
// ---------------------------------------------------------------------------

int32 FOceanQuadTreeNode::FaceResolution() const
{
    return Owner ? Owner->FaceResolution : 3;
}

FVector FOceanQuadTreeNode::ProjectToSphere(FVector CubeOffset) const
{
    FVector WorldCube = CubeCenter + CubeOffset;
    return WorldCube.GetSafeNormal() * OceanRadius - ChunkAnchorCenter;
}

float FOceanQuadTreeNode::InterpolateDensity(double normX, double normY) const
{
    // Bilinear interpolation of CornerDensities at face-local (normX, normY).
    // normX and normY are in [-HalfSize, +HalfSize].
    // u=0 at U- edge (BL/TL), u=1 at U+ edge (BR/TR).
    // v=0 at V- edge (BL/BR), v=1 at V+ edge (TL/TR).
    float u = (float)FMath::Clamp((normX + HalfSize) / Size, 0.0, 1.0);
    float v = (float)FMath::Clamp((normY + HalfSize) / Size, 0.0, 1.0);

    // BL=0 (u=0,v=0), TL=1 (u=0,v=1), BR=2 (u=1,v=0), TR=3 (u=1,v=1)
    float bottom = FMath::Lerp(CornerDensities[0], CornerDensities[2], u); // lerp BL→BR
    float top = FMath::Lerp(CornerDensities[1], CornerDensities[3], u); // lerp TL→TR
    return FMath::Lerp(bottom, top, v);
}

void FOceanQuadTreeNode::GenerateMeshData()
{
    const int32  Res = FaceResolution();
    const int32  ExtRes = Res + 2;
    const double Step = Size / (double)(Res - 1);
    const int32  Total = ExtRes * ExtRes;

    Vertices.Reset();    Normals.Reset();      TexCoords.Reset();
    VertexColors.Reset(); AllTriangles.Reset(); PatchTriangleIndices.Reset();
    EdgeTriangles.Reset();
    NodeBoundRadius = 0.0;

    Vertices.Reserve(Total);
    Normals.Reserve(Total);
    TexCoords.Reserve(Total);
    VertexColors.Reserve(Total);

    // --- Stage 1: build vertex positions and collect sphere-space positions for density sampling ---
    TArray<FVector> SpherePositions;
    SpherePositions.Reserve(Total);

    for (int32 ix = 0; ix < ExtRes; ++ix)
    {
        for (int32 iy = 0; iy < ExtRes; ++iy)
        {
            double normX = -HalfSize + Step * (ix - 1);
            double normY = -HalfSize + Step * (iy - 1);

            FVector CubeOffset = FVector::ZeroVector;
            CubeOffset[FaceTransform.AxisMap[0]] = FaceTransform.AxisDir[0] * normX;
            CubeOffset[FaceTransform.AxisMap[1]] = FaceTransform.AxisDir[1] * normY;

            // ProjectToSphere gives chunk-local position. Add ChunkAnchorCenter
            // to get the actual world-scale sphere position for compositor sampling.
            FVector LocalPos = ProjectToSphere(CubeOffset);
            FVector SpherePos = LocalPos + ChunkAnchorCenter;
            SpherePositions.Add(SpherePos);

            Vertices.Add(LocalPos);
            Normals.Add(FVector3f(SpherePos.GetSafeNormal()));

            FVector Dir = SpherePos.GetSafeNormal();
            TexCoords.Add(FVector2f(
                (FMath::Atan2(Dir.Y, Dir.X) + PI) / (2.f * PI),
                FMath::Acos(FMath::Clamp((float)Dir.Z, -1.f, 1.f)) / PI));

            if (ix > 0 && iy > 0 && ix < ExtRes - 1 && iy < ExtRes - 1)
                NodeBoundRadius = FMath::Max(NodeBoundRadius, LocalPos.Size());
        }
    }

    // --- Stage 2: batch-sample density for all vertices via the compositor ---
    // One compositor call per node — no per-vertex interpolation artifacts,
    // no discontinuities at chunk or node boundaries.
    FDensitySampleCompositor* Comp = Owner ? Owner->GetCompositor().Get() : nullptr;
    if (Comp)
    {
        TArray<float> SX, SY, SZ, DensityOut;
        SX.SetNumUninitialized(Total);
        SY.SetNumUninitialized(Total);
        SZ.SetNumUninitialized(Total);
        DensityOut.SetNumUninitialized(Total);

        for (int32 i = 0; i < Total; ++i)
        {
            SX[i] = (float)SpherePositions[i].X;
            SY[i] = (float)SpherePositions[i].Y;
            SZ[i] = (float)SpherePositions[i].Z;
        }

        FSampleInput Input(SX.GetData(), SY.GetData(), SZ.GetData(), Total);
        Comp->Sample(Input, DensityOut.GetData());

        for (int32 i = 0; i < Total; ++i)
        {
            float d = DensityOut[i];
            uint32 Bits;
            FMemory::Memcpy(&Bits, &d, sizeof(float));
            VertexColors.Add(FColor(
                (uint8)(Bits & 0xFF),
                (uint8)((Bits >> 8) & 0xFF),
                (uint8)((Bits >> 16) & 0xFF),
                (uint8)((Bits >> 24) & 0xFF)
            ));
        }
    }
    else
    {
        // No compositor available — fill with zero density (sea level)
        const float Zero = 0.f;
        uint32 Bits; FMemory::Memcpy(&Bits, &Zero, sizeof(float));
        FColor ZeroColor((uint8)(Bits & 0xFF), (uint8)((Bits >> 8) & 0xFF), (uint8)((Bits >> 16) & 0xFF), (uint8)((Bits >> 24) & 0xFF));
        for (int32 i = 0; i < Total; ++i)
            VertexColors.Add(ZeroColor);
    }

    const int32 tRes = ExtRes - 1;
    for (int32 ix = 0; ix < tRes; ++ix)
    {
        for (int32 iy = 0; iy < tRes; ++iy)
        {
            int32 tl = ix * ExtRes + iy, tr = tl + 1, bl = tl + ExtRes, br = bl + 1;
            bool bIsVirtual = (ix == 0 || iy == 0 || ix == tRes - 1 || iy == tRes - 1);
            bool bIsEdge = (ix == 1 || iy == 1 || ix == tRes - 2 || iy == tRes - 2);

            TArray<FIndex3UI> Tris;
            if ((ix + iy) % 2 == 0) { Tris.Add(FIndex3UI(tl, bl, br)); Tris.Add(FIndex3UI(tl, br, tr)); }
            else { Tris.Add(FIndex3UI(tl, bl, tr)); Tris.Add(FIndex3UI(tr, bl, br)); }

            for (FIndex3UI T : Tris)
            {
                int32 Idx = FaceTransform.bFlipWinding
                    ? AllTriangles.Add(FIndex3UI(T.V0, T.V2, T.V1))
                    : AllTriangles.Add(T);
                if (!bIsVirtual && !bIsEdge) PatchTriangleIndices.Add(Idx);
            }
        }
    }

    HasGenerated = true;
    RebuildEdgeTriangles();
}

void FOceanQuadTreeNode::RebuildEdgeTriangles()
{
    if (!HasGenerated) return;

    const int32 Res = FaceResolution();
    const int32 ExtRes = Res + 2;
    const int32 tRes = Res;

    bool bLeft = GetDepth() > NeighborLods[(uint8)EdgeOrientation::LEFT];
    bool bRight = GetDepth() > NeighborLods[(uint8)EdgeOrientation::RIGHT];
    bool bTop = GetDepth() > NeighborLods[(uint8)EdgeOrientation::UP];
    bool bBottom = GetDepth() > NeighborLods[(uint8)EdgeOrientation::DOWN];

    EdgeTriangles.Reset();
    FIndex3UI topOdd, bottomOdd, leftOdd, rightOdd;

    for (int32 i = 1; i < tRes; ++i)
    {
        { // TOP
            int32 x = i, y = 1, tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { tl,tl + 1,bl,bl + 1 };
            if (x % 2 != 0) { topOdd = FIndex3UI(quad[3], quad[0], quad[2]); if (x != 1)EdgeTriangles.Add(FIndex3UI(quad[0], quad[3], quad[1])); }
            else {
                if (bTop) { topOdd[2] = quad[2]; EdgeTriangles.Add(topOdd); }
                else { EdgeTriangles.Add(topOdd); EdgeTriangles.Add(FIndex3UI(quad[0], quad[2], quad[1])); }
                if (x != tRes - 1)EdgeTriangles.Add(FIndex3UI(quad[1], quad[2], quad[3]));
            }
        }
        { // BOTTOM
            int32 x = i, y = tRes - 1, tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { bl,bl + 1,tl,tl + 1 };
            if (x % 2 != 0) { bottomOdd = FIndex3UI(quad[3], quad[0], quad[1]); if (x != 1)EdgeTriangles.Add(FIndex3UI(quad[3], quad[2], quad[0])); }
            else {
                if (bBottom) { bottomOdd[2] = quad[1]; EdgeTriangles.Add(bottomOdd); }
                else { EdgeTriangles.Add(bottomOdd); EdgeTriangles.Add(FIndex3UI(quad[1], quad[3], quad[2])); }
                if (x != tRes - 1)EdgeTriangles.Add(FIndex3UI(quad[0], quad[1], quad[2]));
            }
        }
        { // LEFT
            int32 y = i, x = 1, tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { tl,bl,tl + 1,bl + 1 };
            if (y % 2 != 0) { leftOdd = FIndex3UI(quad[0], quad[3], quad[2]); if (y != 1)EdgeTriangles.Add(FIndex3UI(quad[3], quad[0], quad[1])); }
            else {
                if (bLeft) { leftOdd[2] = quad[2]; EdgeTriangles.Add(leftOdd); }
                else { EdgeTriangles.Add(leftOdd); EdgeTriangles.Add(FIndex3UI(quad[2], quad[0], quad[1])); }
                if (y != tRes - 1)EdgeTriangles.Add(FIndex3UI(quad[2], quad[1], quad[3]));
            }
        }
        { // RIGHT
            int32 y = i, x = tRes - 1, tl = x * ExtRes + y, bl = (x + 1) * ExtRes + y;
            int32 quad[4] = { tl + 1,bl + 1,tl,bl };
            if (y % 2 != 0) { rightOdd = FIndex3UI(quad[0], quad[3], quad[1]); if (y != 1)EdgeTriangles.Add(FIndex3UI(quad[2], quad[3], quad[0])); }
            else {
                if (bRight) { rightOdd[2] = quad[1]; EdgeTriangles.Add(rightOdd); }
                else { EdgeTriangles.Add(rightOdd); EdgeTriangles.Add(FIndex3UI(quad[3], quad[1], quad[2])); }
                if (y != tRes - 1)EdgeTriangles.Add(FIndex3UI(quad[1], quad[0], quad[2]));
            }
        }
    }
}