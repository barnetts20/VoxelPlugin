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
    FCubeTransform                      InFaceTransform,
    FQuadIndex                          InIndex,
    FVector                             InCubeCenter,
    double                              InSize,
    double                              InOceanRadius,
    int32                               InMinDepth,
    int32                               InMaxDepth,
    int32                               InChunkDepth,
    TSharedPtr<FDensitySampleCompositor> InCompositor)
    : Owner(InOwner)
    , Compositor(InCompositor)
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

// ---------------------------------------------------------------------------
// Density helpers
// ---------------------------------------------------------------------------

// Corner positions in face-local 2D, matching EChildPosition order:
//   0=BL(-,-), 1=TL(-,+), 2=BR(+,-), 3=TR(+,+)
// Returns cube-space 3D offset from CubeCenter for corner i.
static FVector GetCornerCubeOffset(const FOceanQuadTreeNode* Node, int32 i)
{
    // i bit 0 = X axis sign (0=neg, 1=pos), i bit 1 = Y axis sign (0=neg, 1=pos)
    double sx = (i & 2) ? Node->HalfSize : -Node->HalfSize;
    double sy = (i & 1) ? Node->HalfSize : -Node->HalfSize;
    FVector Offset = FVector::ZeroVector;
    Offset[Node->FaceTransform.AxisMap[0]] = Node->FaceTransform.AxisDir[0] * sx;
    Offset[Node->FaceTransform.AxisMap[1]] = Node->FaceTransform.AxisDir[1] * sy;
    return Offset;
}

// Corner world positions (on the cube surface, not projected)
static FVector GetCornerWorldPos(const FOceanQuadTreeNode* Node, int32 i)
{
    return Node->CubeCenter + GetCornerCubeOffset(Node, i);
}

void FOceanQuadTreeNode::SampleCornerDensities()
{
    if (!Compositor.IsValid()) return;

    float SX[4], SY[4], SZ[4], Out[4];
    for (int32 i = 0; i < 4; ++i)
    {
        // Project corner onto sphere surface for sampling — same projection as vertices
        FVector CubePos = GetCornerWorldPos(this, i);
        FVector SpherePos = CubePos.GetSafeNormal() * OceanRadius;
        SX[i] = (float)SpherePos.X;
        SY[i] = (float)SpherePos.Y;
        SZ[i] = (float)SpherePos.Z;
    }

    FSampleInput Input(SX, SY, SZ, 4);
    Compositor->Sample(Input, Out);

    for (int32 i = 0; i < 4; ++i)
        CornerDensities[i] = Out[i];

    bDensitySampled = true;
}

void FOceanQuadTreeNode::SplitAndDistributeDensities(TSharedPtr<FOceanQuadTreeNode> Parent)
{
    // Parent must already have valid corner densities
    if (!Parent.IsValid() || !Parent->bDensitySampled || !Parent->Compositor.IsValid()) return;
    if (Parent->Children.Num() != 4) return;

    // Parent corners in face-local order (BL=0, TL=1, BR=2, TR=3)
    // On split, each child quadrant covers one quarter of the parent face.
    // 
    // Parent corner layout:          Child layout (same face, child i covers quadrant i):
    //   TL(1) --- TR(3)                 Child1(TL): corners TL_new, TL_par, Center, Left_mid
    //    |           |                  ...etc
    //   BL(0) --- BR(2)
    //
    // The 5 NEW positions we need to sample (not inherited from parent corners):
    //   EdgeMid[0] = midpoint of BL-BR  (bottom edge mid)
    //   EdgeMid[1] = midpoint of TL-TR  (top edge mid)
    //   EdgeMid[2] = midpoint of BL-TL  (left edge mid)
    //   EdgeMid[3] = midpoint of BR-TR  (right edge mid)
    //   Center     = face center (midpoint of all 4 corners)
    //
    // Each child needs 4 corners. Mapping:
    //   Child 0 (BL quadrant): BL_par(0), BotMid(E0), LeftMid(E2), Center(C)
    //   Child 1 (TL quadrant): LeftMid(E2), Center(C), TL_par(1), TopMid(E1)
    //   Child 2 (BR quadrant): BotMid(E0), BR_par(2), Center(C), RightMid(E3)
    //   Child 3 (TR quadrant): Center(C), RightMid(E3), TopMid(E1), TR_par(3)
    //
    // Child corner order is same as parent (BL=0,TL=1,BR=2,TR=3) for their own sub-face.

    TSharedPtr<FDensitySampleCompositor> Comp = Parent->Compositor;

    // Compute the 5 new cube-space positions and project to sphere for sampling
    FVector ParentCornerWorld[4];
    for (int32 i = 0; i < 4; ++i)
        ParentCornerWorld[i] = GetCornerWorldPos(Parent.Get(), i);

    FVector NewCubePos[5];
    NewCubePos[0] = (ParentCornerWorld[0] + ParentCornerWorld[2]) * 0.5; // BotMid (BL-BR)
    NewCubePos[1] = (ParentCornerWorld[1] + ParentCornerWorld[3]) * 0.5; // TopMid (TL-TR)
    NewCubePos[2] = (ParentCornerWorld[0] + ParentCornerWorld[1]) * 0.5; // LeftMid (BL-TL)
    NewCubePos[3] = (ParentCornerWorld[2] + ParentCornerWorld[3]) * 0.5; // RightMid (BR-TR)
    NewCubePos[4] = (ParentCornerWorld[0] + ParentCornerWorld[1] + ParentCornerWorld[2] + ParentCornerWorld[3]) * 0.25; // Center

    // Project to sphere surface for density sampling
    float SX[5], SY[5], SZ[5], Out[5];
    for (int32 i = 0; i < 5; ++i)
    {
        FVector SpherePos = NewCubePos[i].GetSafeNormal() * Parent->OceanRadius;
        SX[i] = (float)SpherePos.X;
        SY[i] = (float)SpherePos.Y;
        SZ[i] = (float)SpherePos.Z;
    }

    FSampleInput Input(SX, SY, SZ, 5);
    Comp->Sample(Input, Out);

    // Named references for clarity
    const float D_BL = Parent->CornerDensities[0];
    const float D_TL = Parent->CornerDensities[1];
    const float D_BR = Parent->CornerDensities[2];
    const float D_TR = Parent->CornerDensities[3];
    const float D_BotM = Out[0];
    const float D_TopM = Out[1];
    const float D_LeftM = Out[2];
    const float D_RightM = Out[3];
    const float D_Ctr = Out[4];

    // Assign densities to each child (BL=0, TL=1, BR=2, TR=3 within child's own face)
    // Child 0 occupies BL quadrant of parent
    Parent->Children[0]->CornerDensities[0] = D_BL;
    Parent->Children[0]->CornerDensities[1] = D_LeftM;
    Parent->Children[0]->CornerDensities[2] = D_BotM;
    Parent->Children[0]->CornerDensities[3] = D_Ctr;
    Parent->Children[0]->bDensitySampled = true;

    // Child 1 occupies TL quadrant of parent
    Parent->Children[1]->CornerDensities[0] = D_LeftM;
    Parent->Children[1]->CornerDensities[1] = D_TL;
    Parent->Children[1]->CornerDensities[2] = D_Ctr;
    Parent->Children[1]->CornerDensities[3] = D_TopM;
    Parent->Children[1]->bDensitySampled = true;

    // Child 2 occupies BR quadrant of parent
    Parent->Children[2]->CornerDensities[0] = D_BotM;
    Parent->Children[2]->CornerDensities[1] = D_Ctr;
    Parent->Children[2]->CornerDensities[2] = D_BR;
    Parent->Children[2]->CornerDensities[3] = D_RightM;
    Parent->Children[2]->bDensitySampled = true;

    // Child 3 occupies TR quadrant of parent
    Parent->Children[3]->CornerDensities[0] = D_Ctr;
    Parent->Children[3]->CornerDensities[1] = D_TopM;
    Parent->Children[3]->CornerDensities[2] = D_RightM;
    Parent->Children[3]->CornerDensities[3] = D_TR;
    Parent->Children[3]->bDensitySampled = true;
}

FVector2f FOceanQuadTreeNode::ComputeFaceUV(const FVector& CubeOffset) const
{
    // CubeOffset is relative to CubeCenter, in [-HalfSize, HalfSize] on both face axes.
    // Map to [0, 1] UV.
    double AxisU = CubeOffset[FaceTransform.AxisMap[0]] * FaceTransform.AxisDir[0];
    double AxisV = CubeOffset[FaceTransform.AxisMap[1]] * FaceTransform.AxisDir[1];
    float U = (float)FMath::Clamp((AxisU / Size) + 0.5, 0.0, 1.0);
    float V = (float)FMath::Clamp((AxisV / Size) + 0.5, 0.0, 1.0);
    return FVector2f(U, V);
}

float FOceanQuadTreeNode::InterpolateDensity(float U, float V) const
{
    // Bilinear interpolation across CornerDensities[BL=0, TL=1, BR=2, TR=3]
    // U = 0 => left side (BL/TL), U = 1 => right side (BR/TR)
    // V = 0 => bottom (BL/BR), V = 1 => top (TL/TR)
    float BotInterp = FMath::Lerp(CornerDensities[0], CornerDensities[2], U); // BL -> BR
    float TopInterp = FMath::Lerp(CornerDensities[1], CornerDensities[3], U); // TL -> TR
    return FMath::Lerp(BotInterp, TopInterp, V);
}

FColor FOceanQuadTreeNode::EncodeDepthToColor(float Depth, float DepthRangeMin, float DepthRangeMax)
{
    // Map depth to [0, 1] normalized
    float Range = DepthRangeMax - DepthRangeMin;
    float Normalized = (Range > 1e-6f) ? FMath::Clamp((Depth - DepthRangeMin) / Range, 0.f, 1.f) : 0.5f;

    // Encode as 32-bit fixed point split across RGBA channels.
    // R = most significant byte, A = least significant byte.
    // To reconstruct in shader: depth = (R/255 + G/255^2 + B/255^3 + A/255^4) * Range + Min
    // This gives ~0.000015 precision across the range, enough for ocean depth rendering.
    uint32 Fixed = (uint32)(Normalized * 0xFFFFFFFF);
    uint8 R = (uint8)((Fixed >> 24) & 0xFF);
    uint8 G = (uint8)((Fixed >> 16) & 0xFF);
    uint8 B = (uint8)((Fixed >> 8) & 0xFF);
    uint8 A = (uint8)(Fixed & 0xFF);
    return FColor(R, G, B, A);
}

// ---------------------------------------------------------------------------
// Tree traversal
// ---------------------------------------------------------------------------

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

    FVector ActorLoc = Owner->GetActorLocation();
    FVector WorldSphereCenter = SphereCenter + ActorLoc;
    double  DistSq = FMath::Max(FVector::DistSquared(CameraPos, WorldSphereCenter), 1e-12);

    // Back-face cull
    double Dot = FVector::DotProduct(CameraPos - ActorLoc, SphereCenter);
    if (Dot < 0.0 && GetDepth() >= MinDepth)
    {
        double CamDistSq = FVector::DistSquared(CameraPos, ActorLoc);
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
            inNode->MinDepth, inNode->MaxDepth, inNode->ChunkDepth,
            inNode->Compositor);

        Child->Parent = inNode.ToWeakPtr();
        Child->ChunkAnchorCenter = inNode->ChunkAnchorCenter;
        inNode->Children.Add(Child);
    }

    // Distribute corner densities to children (5 new samples + 4 inherited)
    if (inNode->bDensitySampled)
        SplitAndDistributeDensities(inNode);

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
            MinDepth, MaxDepth, ChunkDepth,
            Compositor);

        Child->Parent = AsShared().ToWeakPtr();

        if (Child->GetDepth() == ChunkDepth)
            Child->ChunkAnchorCenter = Child->SphereCenter;
        else
            Child->ChunkAnchorCenter = ChunkAnchorCenter;

        Children.Add(Child);
    }

    // Distribute densities before generating mesh data so children have valid corners
    if (bDensitySampled)
        SplitAndDistributeDensities(AsShared());

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

void FOceanQuadTreeNode::GenerateMeshData()
{
    const int32  Res = FaceResolution();
    const int32  ExtRes = Res + 2;
    const double Step = Size / (double)(Res - 1);

    Vertices.Reset(); Normals.Reset(); TexCoords.Reset(); VertexDepths.Reset();
    AllTriangles.Reset(); PatchTriangleIndices.Reset();
    EdgeTriangles.Reset();
    NodeBoundRadius = 0.0;

    Vertices.Reserve(ExtRes * ExtRes);
    Normals.Reserve(ExtRes * ExtRes);
    TexCoords.Reserve(ExtRes * ExtRes);
    VertexDepths.Reserve(ExtRes * ExtRes);

    for (int32 ix = 0; ix < ExtRes; ++ix)
    {
        for (int32 iy = 0; iy < ExtRes; ++iy)
        {
            double normX = -HalfSize + Step * (ix - 1);
            double normY = -HalfSize + Step * (iy - 1);

            FVector CubeOffset = FVector::ZeroVector;
            CubeOffset[FaceTransform.AxisMap[0]] = FaceTransform.AxisDir[0] * normX;
            CubeOffset[FaceTransform.AxisMap[1]] = FaceTransform.AxisDir[1] * normY;

            FVector LocalPos = ProjectToSphere(CubeOffset);
            Vertices.Add(LocalPos);

            FVector WorldPos = LocalPos + SphereCenter;
            Normals.Add(FVector3f(WorldPos.GetSafeNormal()));

            FVector Dir = WorldPos.GetSafeNormal();
            TexCoords.Add(FVector2f(
                (FMath::Atan2(Dir.Y, Dir.X) + PI) / (2.f * PI),
                FMath::Acos(FMath::Clamp((float)Dir.Z, -1.f, 1.f)) / PI));

            // Interpolate depth from corner densities using face-local UV
            float Depth = 0.f;
            if (bDensitySampled)
            {
                FVector2f UV = ComputeFaceUV(CubeOffset);
                Depth = InterpolateDensity(UV.X, UV.Y);
            }
            VertexDepths.Add(Depth);

            if (ix > 0 && iy > 0 && ix < ExtRes - 1 && iy < ExtRes - 1)
                NodeBoundRadius = FMath::Max(NodeBoundRadius, LocalPos.Size());
        }
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