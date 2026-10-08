#include "FAdaptiveOctree.h"
#include "FOctreeConstants.h"
#include "VoxelDensity.h"

TAutoConsoleVariable<int32> CVarVoxelDebugFlicker(
    TEXT("voxel.Debug.Flicker"),
    0,
    TEXT("Flicker diagnostics for voxel terrain.\n")
    TEXT(" 0: off\n")
    TEXT(" 1: log apply, dirty-chunk and dropped-edge statistics; run the stale-seam check\n")
    TEXT("    (voxel.Debug.SeamCheckInterval) and draw the bounds of stale chunks"),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarVoxelDebugChunkColor(
    TEXT("voxel.Debug.ChunkColor"),
    0,
    TEXT("Vertex colour of newly meshed voxel chunks (the material must show vertex colour).\n")
    TEXT(" 0: constant\n")
    TEXT(" 1: by chunk-root depth\n")
    TEXT(" 2: by the pass that meshed the chunk (chunks applied together share a colour)"),
    ECVF_Default);

TAutoConsoleVariable<int32> CVarVoxelDebugSeamCheckInterval(
    TEXT("voxel.Debug.SeamCheckInterval"),
    10,
    TEXT("With voxel.Debug.Flicker on, run the stale-seam check every N terrain data passes."),
    ECVF_Default);

namespace VoxelOctreeDetail
{
    FColor ChunkDebugColor(int32 Mode, uint8 RootDepth, uint32 PassId)
    {
        if (Mode == 1) return FLinearColor::MakeFromHSV8((uint8)(RootDepth * 37), 220, 255).ToFColor(true);
        if (Mode == 2) return FLinearColor::MakeFromHSV8((uint8)(PassId * 47), 220, 255).ToFColor(true);
        return FColor::Green;
    }

    /** Adds every chunk root under Node whose AABB intersects [BoxMin, BoxMax], except
     *  Exclude. Chunk roots form an anti-chain, so the walk stops at each. */
    void CollectChunkRootsInBox(FAdaptiveOctreeNode* Node, const FVector& BoxMin, const FVector& BoxMax,
        const FAdaptiveOctreeNode* Exclude, TSet<FAdaptiveOctreeNode*>& Out)
    {
        if (!Node) return;
        const FVector C = Node->Center;
        const double E = Node->Extent;
        if (C.X + E < BoxMin.X || C.X - E > BoxMax.X ||
            C.Y + E < BoxMin.Y || C.Y - E > BoxMax.Y ||
            C.Z + E < BoxMin.Z || C.Z - E > BoxMax.Z) return;

        if (Node->bIsChunkRoot)
        {
            if (Node != Exclude) Out.Add(Node);
            return;
        }
        if (Node->IsLeaf()) return;
        for (int i = 0; i < 8; i++)
            CollectChunkRootsInBox(Node->Children[i].Get(), BoxMin, BoxMax, Exclude, Out);
    }

    /** Adds every leaf under Node whose AABB intersects [BoxMin, BoxMax]. With
     *  bSurfaceOnly, skips subtrees without surface (IsSurfaceNode is set on every ancestor
     *  of a surface leaf) and returns only leaves with sign-change edges of their own. */
    void CollectLeavesInBox(FAdaptiveOctreeNode* Node, const FVector& BoxMin, const FVector& BoxMax,
        bool bSurfaceOnly, TArray<FAdaptiveOctreeNode*>& Out)
    {
        if (!Node) return;
        if (bSurfaceOnly && !Node->IsSurfaceNode) return;
        const FVector C = Node->Center;
        const double E = Node->Extent;
        if (C.X + E < BoxMin.X || C.X - E > BoxMax.X ||
            C.Y + E < BoxMin.Y || C.Y - E > BoxMax.Y ||
            C.Z + E < BoxMin.Z || C.Z - E > BoxMax.Z) return;

        if (Node->IsLeaf())
        {
            if (!bSurfaceOnly || Node->HasOwnSurface()) Out.Add(Node);
            return;
        }
        for (int i = 0; i < 8; i++)
            CollectLeavesInBox(Node->Children[i].Get(), BoxMin, BoxMax, bSurfaceOnly, Out);
    }

    FBox NodeBox(const FAdaptiveOctreeNode* Node, double Grow = 0.0)
    {
        const FVector Half(Node->Extent + Grow);
        return FBox(Node->Center - Half, Node->Center + Half);
    }

    /** Relative slack for comparing octree positions that went through different sums. */
    double PositionTolerance(double Extent)
    {
        return FMath::Max(Extent * 1e-6, 1e-4);
    }

    /** True if both endpoints of the edge lie in the same face plane of the box and inside
     *  the box (within Tol). */
    bool IsEdgeOnBoxBoundary(const FNodeEdge& Edge, const FVector& BoxMin, const FVector& BoxMax, double Tol)
    {
        const FVector& A = Edge.Corners[0].Position;
        const FVector& B = Edge.Corners[1].Position;
        for (int32 Axis = 0; Axis < 3; Axis++)
        {
            if (A[Axis] < BoxMin[Axis] - Tol || A[Axis] > BoxMax[Axis] + Tol) return false;
            if (B[Axis] < BoxMin[Axis] - Tol || B[Axis] > BoxMax[Axis] + Tol) return false;
        }
        for (int32 Axis = 0; Axis < 3; Axis++)
        {
            if (Axis == Edge.Axis) continue;
            const bool bOnMin = FMath::Abs(A[Axis] - BoxMin[Axis]) <= Tol && FMath::Abs(B[Axis] - BoxMin[Axis]) <= Tol;
            const bool bOnMax = FMath::Abs(A[Axis] - BoxMax[Axis]) <= Tol && FMath::Abs(B[Axis] - BoxMax[Axis]) <= Tol;
            if (bOnMin || bOnMax) return true;
        }
        return false;
    }

    /** True if the edge lies in one of the box's face planes (within a quarter of its
     *  length on one of its two perpendicular axes). Only such an edge can have leaves of
     *  another chunk around it: leaves tile a chunk on its own power-of-two lattice. */
    bool IsOnBoxFace(const FNodeEdge& Edge, const FVector& BoxMin, const FVector& BoxMax)
    {
        const FVector Mid = (Edge.Corners[0].Position + Edge.Corners[1].Position) * 0.5;
        const double Tol = Edge.Size * 0.25;
        for (int32 Axis = 0; Axis < 3; Axis++)
        {
            if (Axis == Edge.Axis) continue;
            if (FMath::Abs(Mid[Axis] - BoxMin[Axis]) <= Tol || FMath::Abs(Mid[Axis] - BoxMax[Axis]) <= Tol)
                return true;
        }
        return false;
    }
}

using namespace VoxelOctreeDetail;

FAdaptiveOctree::FAdaptiveOctree(const FOctreeParams& Params)
{
    CachedParentActor = Params.ParentActor;
    CachedMeshAttachRoot = Params.MeshAttachmentRoot;
    CachedSurfaceMaterial = Params.SurfaceMaterial;
    Compositor = Params.Compositor;
    ChunkDepth = Params.ChunkDepth;
    MaxChunkDepth = Params.MaxChunkDepth;
    ChunkPrecisionThreshold = Params.ChunkPrecisionThreshold;
    ChunkDemoteHysteresis = Params.ChunkDemoteHysteresis;
    MaxChunkDepthDelta = FMath::Max(1, Params.MaxChunkDepthDelta);

    // Core terrain parameters from params
    PlanetRadius = Params.PlanetRadius;
    NoiseAmplitude = Params.NoiseAmplitude;

    // Derive root extent: must contain all possible surface points
    RootExtent = VoxelDensity::RootExtentFor(PlanetRadius, NoiseAmplitude);
    ChunkExtent = RootExtent / FMath::Pow(2.0, (double)Params.ChunkDepth);

    // Configure edge key quantization for this tree's scale.
    // Maps the smallest possible corner spacing to ~1 grid unit.
    FEdgeKey::InvGridSize = FMath::Pow(2.0, (double)Params.MaxDepth) / RootExtent;
    SampleFootprint = VoxelDensity::TerrainFinestCell(PlanetRadius, NoiseAmplitude, Params.MaxDepth);

    // UV scale: 0.0001 was calibrated for PlanetRadius=80M. Keep the same visual tiling.
    TriplanarUVScale = 0.0001 * (80000000.0 / FMath::Max(PlanetRadius, 1.0));

    Root = MakeShared<FAdaptiveOctreeNode>(RootExtent, Params.ChunkDepth, Params.MinDepth, Params.MaxDepth);
    ComputeNodeData(Root.Get());
    SplitToDepth(Root.Get(), Params.ChunkDepth);

    PopulateChunks();
}

void FAdaptiveOctree::SplitToDepth(FAdaptiveOctreeNode* Node, int InMinDepth)
{
    if (!Node || Node->Index.Depth >= InMinDepth) return;

    SplitAndComputeChildren(Node);
    for (int i = 0; i < 8; i++)
    {
        if (Node->Children[i] && Node->Children[i]->CouldContainSurface)
            SplitToDepth(Node->Children[i].Get(), InMinDepth);
    }
}

void FAdaptiveOctree::CollectNearSurfaceChunks(FAdaptiveOctreeNode* Node, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out)
{
    if (!Node) return;
    if (!Node->CouldContainSurface) return;

    if (Node->bIsChunkRoot)
    {
        Out.Add(Node->AsShared());
        return;
    }

    if (Node->IsLeaf()) return;

    for (int i = 0; i < 8; i++)
        CollectNearSurfaceChunks(Node->Children[i].Get(), Out);
}

void FAdaptiveOctree::PopulateChunks()
{
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    CollectNearSurfaceChunks(Root.Get(), Chunks);

    // Densities, normals, edges, QEF and surface flags were computed by SplitToDepth.
    // Every chunk is in the map before any is meshed: the mesher checks the map to decide
    // which chunk emits a shared boundary edge.
    TArray<TSharedPtr<FMeshChunk>> NewChunks;
    NewChunks.SetNum(Chunks.Num());

    ParallelFor(Chunks.Num(), [&](int32 i) {
        NewChunks[i] = CreateMeshChunk(Chunks[i].Get());
        });

    for (int32 i = 0; i < Chunks.Num(); i++)
        ChunkMap.Add(Chunks[i], NewChunks[i]);

    ParallelFor(Chunks.Num(), [&](int32 i) {
        UpdateMeshChunkStreamData(Chunks[i].Get(), NewChunks[i]);
        });

    MeshChunksInitialized = true;
}

void FAdaptiveOctree::CollectChunkRootsInSphere(FAdaptiveOctreeNode* Node, const FVector& SphereCenter, double SphereRadius, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out) const
{
    if (!Node) return;

    const FVector Closest(
        FMath::Clamp(SphereCenter.X, Node->Center.X - Node->Extent, Node->Center.X + Node->Extent),
        FMath::Clamp(SphereCenter.Y, Node->Center.Y - Node->Extent, Node->Center.Y + Node->Extent),
        FMath::Clamp(SphereCenter.Z, Node->Center.Z - Node->Extent, Node->Center.Z + Node->Extent));
    if (FVector::DistSquared(Closest, SphereCenter) > SphereRadius * SphereRadius) return;

    if (Node->bIsChunkRoot) { Out.Add(Node->AsShared()); return; }
    if (Node->IsLeaf()) return;

    for (int i = 0; i < 8; i++)
        CollectChunkRootsInSphere(Node->Children[i].Get(), SphereCenter, SphereRadius, Out);
}

void FAdaptiveOctree::ApplyEdit(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution)
{
    TSharedPtr<FSparseEditStore> EditStore = Compositor.IsValid() ? Compositor->GetEditStore() : nullptr;
    if (!EditStore.IsValid()) return;

    const int EditDepth = EditStore->GetDepthForBrushRadius(InEditRadius, InEditResolution);
    EditStore->ApplySphericalEdit(InEditCenter, InEditRadius, InEditStrength, EditDepth);
    RefreshEdit(InEditCenter, InEditRadius);
}

void FAdaptiveOctree::RefreshEdit(FVector InEditCenter, double InEditRadius)
{
    // Every chunk root the brush touches, at whatever depth the chunk cut has it.
    TArray<TSharedPtr<FAdaptiveOctreeNode>> ChunkNodes;
    CollectChunkRootsInSphere(Root.Get(), InEditCenter, InEditRadius, ChunkNodes);
    if (ChunkNodes.Num() == 0) return;

    // Flag the nodes whose AABB overlaps the brush; the reconstruction below only walks
    // flagged nodes.
    double BrushRadiusSq = InEditRadius * InEditRadius;
    TFunction<void(FAdaptiveOctreeNode*)> FlagEditedNodes = [&](FAdaptiveOctreeNode* N)
        {
            if (!N) return;

            // Sphere-AABB overlap test
            FVector Closest(
                FMath::Clamp(InEditCenter.X, N->Center.X - N->Extent, N->Center.X + N->Extent),
                FMath::Clamp(InEditCenter.Y, N->Center.Y - N->Extent, N->Center.Y + N->Extent),
                FMath::Clamp(InEditCenter.Z, N->Center.Z - N->Extent, N->Center.Z + N->Extent));
            if (FVector::DistSquared(Closest, InEditCenter) > BrushRadiusSq)
                return;

            N->bHasEditedDescendants = true;

            if (!N->IsLeaf())
            {
                for (int i = 0; i < 8; i++)
                    FlagEditedNodes(N->Children[i].Get());
            }
        };

    for (auto& ChunkNode : ChunkNodes)
        FlagEditedNodes(ChunkNode.Get());

    // Also flag ancestors up to root so EnforceSplitsInSubtree can enter.
    for (auto& ChunkNode : ChunkNodes)
    {
        FAdaptiveOctreeNode* Current = ChunkNode->Parent.IsValid() ? ChunkNode->Parent.Pin().Get() : nullptr;
        while (Current && !Current->bHasEditedDescendants)
        {
            Current->bHasEditedDescendants = true;
            Current = Current->Parent.IsValid() ? Current->Parent.Pin().Get() : nullptr;
        }
    }

    // Stage 1: Parallel -- recompute all node data (densities, normals, surface flags).
    // ChunkNodes are distinct chunk roots (an anti-chain), so the subtrees are disjoint.
    ParallelFor(ChunkNodes.Num(), [&](int32 i) {
        ReconstructSubtree(ChunkNodes[i].Get());
        });

    // Stage 2: Enforce splits -- edits may create surface in previously empty nodes.
    // Must run after ReconstructSubtree (densities updated).
    TArray<FAdaptiveOctreeNode*> SplitNodes;
    for (auto& ChunkNode : ChunkNodes)
    {
        EnforceSplitsInSubtree(ChunkNode.Get(), SplitNodes);
    }

    // Every leaf the brush reaches changed (its crossings may have appeared or moved); their
    // neighbours' vertices may use their edges.
    TArray<FAdaptiveOctreeNode*> Touched;
    {
        TArray<FAdaptiveOctreeNode*> InBox;
        const FVector BrushHalf(InEditRadius);
        CollectLeavesInBox(Root.Get(), InEditCenter - BrushHalf, InEditCenter + BrushHalf, false, InBox);
        for (FAdaptiveOctreeNode* Leaf : InBox)
            if (GetChunkNodeByPoint(Leaf->Center))
                Touched.Add(Leaf);
    }
    TSet<FAdaptiveOctreeNode*> ChangedRoots;
    TArray<FBox> ChangedRegions;
    for (auto& ChunkNode : ChunkNodes)
        ChangedRoots.Add(ChunkNode.Get());
    for (FAdaptiveOctreeNode* Leaf : Touched)
        ChangedRegions.Add(NodeBox(Leaf));

    // Stage 3: keep the leaves 2:1 balanced around the new detail and the changed crossings.
    TArray<FAdaptiveOctreeNode*> BalanceSeeds = SplitNodes;
    BalanceSeeds.Append(Touched);
    BalanceAfterSplits(BalanceSeeds, ChangedRoots, ChangedRegions);

    // Stage 4: vertices, chunk map, edges and meshes for everything that changed.
    FinishPass(ChangedRoots, ChangedRegions);
}

void FAdaptiveOctree::GatherUniqueCorners(FAdaptiveOctreeNode* Node, TArray<FCornerSample>& Samples, TMap<FInt64Vector, int32>& CornerMap, double QuantizeGrid)
{
    if (!Node) return;

    if (!Node->bHasEditedDescendants)
        return;

    // Collect corners at ALL depths
    for (int i = 0; i < 8; i++)
    {
        FVector Pos = Node->Corners[i].Position;
        FInt64Vector Key(
            (int64)FMath::RoundToDouble(Pos.X / QuantizeGrid),
            (int64)FMath::RoundToDouble(Pos.Y / QuantizeGrid),
            (int64)FMath::RoundToDouble(Pos.Z / QuantizeGrid));

        int32* Existing = CornerMap.Find(Key);
        if (Existing)
        {
            Samples[*Existing].Targets.Add(&Node->Corners[i].Density);
        }
        else
        {
            int32 NewIdx = Samples.Num();
            FCornerSample Sample;
            Sample.Position = Pos;
            Sample.Targets.Add(&Node->Corners[i].Density);
            Samples.Add(Sample);
            CornerMap.Add(Key, NewIdx);
        }
    }

    if (Node->IsLeaf()) return;

    for (int i = 0; i < 8; i++)
        GatherUniqueCorners(Node->Children[i].Get(), Samples, CornerMap, QuantizeGrid);
}

void FAdaptiveOctree::FinalizeSubtree(FAdaptiveOctreeNode* Node)
{
    if (!Node) return;

    if (!Node->bHasEditedDescendants)
        return;

    // Children first
    if (!Node->IsLeaf())
    {
        for (int i = 0; i < 8; i++)
            FinalizeSubtree(Node->Children[i].Get());
    }

    // Then this node
    Node->FinalizeFromExistingCorners();
}

void FAdaptiveOctree::ReconstructSubtree(FAdaptiveOctreeNode* Node)
{
    // 1. Gather unique corners from affected nodes
    TMap<FInt64Vector, int32> CornerMap;
    TArray<FCornerSample> Samples;
    Samples.Reserve(4096);
    CornerMap.Reserve(4096);
    // Corners lie on multiples of the MaxDepth node extent (keys up to +-2^MaxDepth).
    double QuantizeGrid = SampleFootprint * 0.5;

    GatherUniqueCorners(Node, Samples, CornerMap, QuantizeGrid);

    if (Samples.Num() == 0) return;

    // 2. Build position arrays
    int32 Count = Samples.Num();
    TArray<double> SX, SY, SZ;
    SX.SetNumUninitialized(Count);
    SY.SetNumUninitialized(Count);
    SZ.SetNumUninitialized(Count);

    for (int32 i = 0; i < Count; i++)
    {
        SX[i] = Samples[i].Position.X;
        SY[i] = Samples[i].Position.Y;
        SZ[i] = Samples[i].Position.Z;
    }

    // 3. One bulk density call
    TArray<float> SampleOut;
    SampleOut.SetNumUninitialized(Count);

    FSampleInput SampleIn(SX.GetData(), SY.GetData(), SZ.GetData(), Count, SampleFootprint);
    Compositor->Sample(SampleIn, SampleOut.GetData());

    // 4. Write back densities to all nodes that share each corner
    for (int32 i = 0; i < Count; i++)
    {
        float FinalDensity = SampleOut[i];
        for (float* Target : Samples[i].Targets)
            *Target = FinalDensity;
    }

    // 5. Finalize edges/QEF
    FinalizeSubtree(Node);
}

void FAdaptiveOctree::EnforceSplitsInSubtree(FAdaptiveOctreeNode* Node, TArray<FAdaptiveOctreeNode*>& OutSplitNodes)
{
    if (!Node) return;

    if (!Node->bHasEditedDescendants)
        return;

    if (Node->IsLeaf())
    {
        bool bNeedsSplit = false;

        if (Node->bHasEditedDescendants || Node->CouldContainSurface)
        {
            bool bLodWantsSplit = Node->ShouldSplit(CachedCameraPosition, CachedThresholdSq, CachedFOVScale);
            bool bBelowMinDepth = Node->Index.Depth < Node->DepthBounds[EDepthBound::MinDepth];
            bNeedsSplit = bLodWantsSplit || bBelowMinDepth;
        }

        // Respect max depth
        if (bNeedsSplit && Node->Index.Depth < Node->DepthBounds[EDepthBound::MaxDepth])
        {
            SplitAndComputeChildren(Node);
            OutSplitNodes.Add(Node);

            // Recurse into new children — they might need further splitting
            for (int i = 0; i < 8; i++)
            {
                if (Node->Children[i].IsValid())
                    EnforceSplitsInSubtree(Node->Children[i].Get(), OutSplitNodes);
            }
        }
        return;
    }

    // Non-leaf: recurse into children
    for (int i = 0; i < 8; i++)
    {
        if (Node->Children[i].IsValid())
            EnforceSplitsInSubtree(Node->Children[i].Get(), OutSplitNodes);
    }
}

FVector2f FAdaptiveOctree::ComputeTriplanarUV(FVector Position, FVector3f Normal) const
{
    FVector2f UV;
    FVector3f AbsNormal = Normal.GetAbs();
    float Scale = (float)TriplanarUVScale;

    if (AbsNormal.X > AbsNormal.Y && AbsNormal.X > AbsNormal.Z)
    {
        UV = FVector2f(Position.Y, Position.Z) * Scale;
    }
    else if (AbsNormal.Y > AbsNormal.X && AbsNormal.Y > AbsNormal.Z)
    {
        UV = FVector2f(Position.X, Position.Z) * Scale;
    }
    else
    {
        UV = FVector2f(Position.X, Position.Y) * Scale;
    }

    return UV;
}

void FAdaptiveOctree::MeshChunksAndNeighbours(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& ChangedNodes)
{
    if (ChangedNodes.Num() == 0) return;

    TSet<FAdaptiveOctreeNode*> Changed;
    for (const TSharedPtr<FAdaptiveOctreeNode>& Node : ChangedNodes)
        Changed.Add(Node.Get());

    TSet<FAdaptiveOctreeNode*> Adjacent;
    for (FAdaptiveOctreeNode* Node : Changed)
        CollectAdjacentChunkRoots(Node, Adjacent);

    // Changed chunks always re-mesh; a neighbour only if the leaves its boundary faces
    // use are no longer the ones it was meshed against.
    TArray<FAdaptiveOctreeNode*> MeshNodes;
    TArray<TSharedPtr<FMeshChunk>> MeshChunks;
    TArray<FAdaptiveOctreeNode*> CheckNodes;
    TArray<TSharedPtr<FMeshChunk>> CheckChunks;
    for (FAdaptiveOctreeNode* Node : Changed)
    {
        if (const TSharedPtr<FMeshChunk>* Found = ChunkMap.Find(Node->AsShared()))
        {
            MeshNodes.Add(Node);
            MeshChunks.Add(*Found);
        }
    }
    for (FAdaptiveOctreeNode* Node : Adjacent)
    {
        if (Changed.Contains(Node)) continue;
        if (const TSharedPtr<FMeshChunk>* Found = ChunkMap.Find(Node->AsShared()))
        {
            CheckNodes.Add(Node);
            CheckChunks.Add(*Found);
        }
    }

    TArray<uint8> SeamChanged;
    SeamChanged.SetNumZeroed(CheckNodes.Num());
    ParallelFor(CheckNodes.Num(), [&](int32 i) {
        SeamChanged[i] = (ComputeSeamHash(CheckNodes[i], *CheckChunks[i]) != CheckChunks[i]->SeamHash) ? 1 : 0;
        });
    for (int32 i = 0; i < CheckNodes.Num(); i++)
    {
        if (SeamChanged[i])
        {
            MeshNodes.Add(CheckNodes[i]);
            MeshChunks.Add(CheckChunks[i]);
        }
    }

    ParallelFor(MeshNodes.Num(), [&](int32 i) {
        UpdateMeshChunkStreamData(MeshNodes[i], MeshChunks[i]);
        });
}

void FAdaptiveOctree::CollectAdjacentChunkRoots(FAdaptiveOctreeNode* Node, TSet<FAdaptiveOctreeNode*>& Out)
{
    // Grown by a hair so roots that only touch a face, edge or corner intersect.
    const FVector Grow(Node->Extent * (1.0 + 1e-6));
    CollectChunkRootsInBox(Root.Get(), Node->Center - Grow, Node->Center + Grow, Node, Out);
}

uint32 FAdaptiveOctree::HashExternalNeighbours(const FEdgeNeighbors& Neighbors, const FMortonIndex& ChunkIndex, bool& bOutHasExternal)
{
    uint32 Hash = 0x9E3779B9u;
    bOutHasExternal = false;
    for (int32 i = 0; i < Neighbors.Count; i++)
    {
        const FAdaptiveOctreeNode* N = Neighbors.Nodes[i];
        if (N->Index.IsWithin(ChunkIndex)) continue;

        bOutHasExternal = true;
        uint32 NodeHash = GetTypeHash(N->Index);
        NodeHash = FCrc::MemCrc32(&N->DualContourPosition, sizeof(FVector), NodeHash);
        NodeHash = FCrc::MemCrc32(&N->DualContourNormal, sizeof(FVector3f), NodeHash);
        NodeHash = HashCombine(NodeHash, N->bHasVertexData ? 1u : 0u);
        uint32 EdgeKeys = 0;
        for (const FNodeEdge& Edge : N->GetSignChangeEdges())
            EdgeKeys ^= Edge.CachedKey.CachedHash;
        NodeHash = HashCombine(NodeHash, EdgeKeys);
        Hash = HashCombine(Hash, NodeHash);
    }
    return bOutHasExternal ? Hash : 0;
}

uint32 FAdaptiveOctree::ComputeSeamHash(FAdaptiveOctreeNode* ChunkNode, const FMeshChunk& Chunk)
{
    const FVector BoxMin = ChunkNode->Center - FVector(ChunkNode->Extent);
    const FVector BoxMax = ChunkNode->Center + FVector(ChunkNode->Extent);
    uint32 Seam = 0;
    for (const FNodeEdge& Edge : Chunk.ChunkEdges)
    {
        if (!IsOnBoxFace(Edge, BoxMin, BoxMax)) continue;
        bool bHasExternal = false;
        Seam += HashExternalNeighbours(SampleNodesAroundEdge(Edge), ChunkNode->Index, bHasExternal);
    }
    return Seam;
}

bool FAdaptiveOctree::OwnsSharedEdge(const FNodeEdge& Edge, const FEdgeNeighbors& Neighbors, const FAdaptiveOctreeNode* ChunkNode)
{
    for (int32 i = 0; i < Neighbors.Count; i++)
    {
        const FAdaptiveOctreeNode* N = Neighbors.Nodes[i];
        if (N->Index.IsWithin(ChunkNode->Index)) continue;

        // Does the leaf across the boundary hold this exact edge (same size, same key)?
        bool bHoldsEdge = false;
        for (const FNodeEdge& Other : N->GetSignChangeEdges())
        {
            if (Other.CachedKey == Edge.CachedKey) { bHoldsEdge = true; break; }
        }
        if (!bHoldsEdge) continue;

        // Hand the edge over only to a chunk that is actually meshed.
        FAdaptiveOctreeNode* OtherChunk = GetChunkNodeByPoint(N->Center);
        if (!OtherChunk || OtherChunk == ChunkNode) continue;
        if (!FMortonIndex::PathLess(OtherChunk->Index, ChunkNode->Index)) continue;
        if (ChunkMap.Contains(OtherChunk->AsShared()))
            return false;
    }
    return true;
}

void FAdaptiveOctree::UpdateMeshChunkStreamData(FAdaptiveOctreeNode* ChunkNode, TSharedPtr<FMeshChunk> InChunk)
{
    TMap<FMeshVertex, int32> VertexMap;
    TArray<FMeshVertex> UniqueVertices;
    TArray<FIndex3UI> SrfTriangles;

    VertexMap.Reserve(1024);
    UniqueVertices.Reserve(1024);
    SrfTriangles.Reserve(2048);

    const FMortonIndex ChunkIndex = ChunkNode->Index;
    const FVector BoxMin = ChunkNode->Center - FVector(ChunkNode->Extent);
    const FVector BoxMax = ChunkNode->Center + FVector(ChunkNode->Extent);
    const bool bCountDrops = CVarVoxelDebugFlicker.GetValueOnAnyThread() > 0;
    const FColor VertexColor = ChunkDebugColor(CVarVoxelDebugChunkColor.GetValueOnAnyThread(), ChunkIndex.Depth, CurrentPassId);

    TArray<FEdgeVertexData> AllEdgeData;
    AllEdgeData.SetNum(InChunk->ChunkEdges.Num());

    ParallelFor(InChunk->ChunkEdges.Num(), [&](int32 edgeIdx) {
        const FNodeEdge& currentEdge = InChunk->ChunkEdges[edgeIdx];
        FEdgeVertexData& EdgeData = AllEdgeData[edgeIdx];
        FEdgeNeighbors neighbors = SampleNodesAroundEdge(currentEdge);
        bool bHasExternal = false;
        const uint32 BoundaryHash = HashExternalNeighbours(neighbors, ChunkIndex, bHasExternal);
        if (IsOnBoxFace(currentEdge, BoxMin, BoxMax))
            EdgeData.BoundaryHash = BoundaryHash;

        if (!InChunk->ShouldProcessEdge(currentEdge, neighbors)) {
            if (bCountDrops && neighbors.Count < 3)
            {
                if (IsOnBoxFace(currentEdge, BoxMin, BoxMax))
                    DroppedFaceEdges.fetch_add(1, std::memory_order_relaxed);
                else
                    DroppedInteriorEdges.fetch_add(1, std::memory_order_relaxed);
            }
            return;
        }

        // Only an edge with leaves outside the chunk can be held by another chunk too.
        if (bHasExternal && !OwnsSharedEdge(currentEdge, neighbors, ChunkNode))
            return;

        // Every leaf around a crossing should carry a vertex built from it (own or boundary
        // edges). One without any would put its cell center in the quad: skip and count.
        for (int i = 0; i < neighbors.Count; i++)
        {
            if (!neighbors.Nodes[i]->bHasVertexData)
            {
                if (bCountDrops)
                    VertexlessQuads.fetch_add(1, std::memory_order_relaxed);
                return;
            }
        }

        EdgeData.IsValid = true;
        EdgeData.Edge = currentEdge;
        EdgeData.Vertices.SetNumZeroed(neighbors.Count);

        for (int i = 0; i < neighbors.Count; i++) {
            FAdaptiveOctreeNode* NodePtr = neighbors.Nodes[i];
            FVector LocalPos(NodePtr->DualContourPosition - InChunk->ChunkCenter);

            EdgeData.Vertices[i].Position = LocalPos;
            EdgeData.Vertices[i].Normal = NodePtr->DualContourNormal;
            EdgeData.Vertices[i].Color = VertexColor;
        }
        });

    uint32 SeamHash = 0;
    int32 NumFolded = 0;
    for (int32 edgeIdx = 0; edgeIdx < AllEdgeData.Num(); edgeIdx++) {
        SeamHash += AllEdgeData[edgeIdx].BoundaryHash;
        if (!AllEdgeData[edgeIdx].IsValid) continue;

        const auto& currentEdge = AllEdgeData[edgeIdx].Edge.GetValue();
        const TArray<FMeshVertex>& EdgeVertices = AllEdgeData[edgeIdx].Vertices;

        int32 VertexIndices[4];
        int32 NumEdgeVerts = EdgeVertices.Num();

        for (int i = 0; i < NumEdgeVerts; i++) {
            int32* ExistingIndex = VertexMap.Find(EdgeVertices[i]);
            if (ExistingIndex) {
                VertexIndices[i] = *ExistingIndex;
            }
            else {
                int32 NewIndex = UniqueVertices.Num();
                UniqueVertices.Add(EdgeVertices[i]);
                VertexMap.Add(EdgeVertices[i], NewIndex);
                VertexIndices[i] = NewIndex;
            }
        }

        bool FlipWinding = (currentEdge.Corners[0].Density < 0);

        // Corners of the quad in cyclic order around the edge. A non-planar quad splits
        // along the diagonal whose two triangles fold least.
        int32 Quad[4] = { VertexIndices[0], VertexIndices[1], NumEdgeVerts > 2 ? VertexIndices[2] : -1, NumEdgeVerts > 3 ? VertexIndices[3] : -1 };
        bool bSplitB = false;
        if (NumEdgeVerts == 4)
        {
            const FVector P0 = EdgeVertices[0].Position, P1 = EdgeVertices[1].Position;
            const FVector P2 = EdgeVertices[2].Position, P3 = EdgeVertices[3].Position;
            const double FoldA = FVector::DotProduct(((P1 - P0) ^ (P2 - P0)).GetSafeNormal(), ((P2 - P0) ^ (P3 - P0)).GetSafeNormal());
            const double FoldB = FVector::DotProduct(((P1 - P0) ^ (P3 - P0)).GetSafeNormal(), ((P2 - P1) ^ (P3 - P1)).GetSafeNormal());
            bSplitB = FoldB > FoldA;
        }

        // Split A: (0,1,2)(0,2,3); split B: (0,1,3)(1,2,3). Both keep the cyclic winding.
        int32 Tris[2][3];
        int32 NumTris = 0;
        if (NumEdgeVerts == 3)
        {
            Tris[0][0] = 0; Tris[0][1] = 1; Tris[0][2] = 2; NumTris = 1;
        }
        else if (NumEdgeVerts == 4 && !bSplitB)
        {
            Tris[0][0] = 0; Tris[0][1] = 1; Tris[0][2] = 2;
            Tris[1][0] = 0; Tris[1][1] = 2; Tris[1][2] = 3; NumTris = 2;
        }
        else if (NumEdgeVerts == 4)
        {
            Tris[0][0] = 0; Tris[0][1] = 1; Tris[0][2] = 3;
            Tris[1][0] = 1; Tris[1][1] = 2; Tris[1][2] = 3; NumTris = 2;
        }

        for (int32 t = 0; t < NumTris; t++)
        {
            const int32 A = Quad[Tris[t][0]], B = Quad[Tris[t][1]], C = Quad[Tris[t][2]];
            if (A == B || B == C || A == C) continue;   // two corners share a leaf
            SrfTriangles.Add(FlipWinding ? FIndex3UI(A, C, B) : FIndex3UI(A, B, C));

            if (bCountDrops)
            {
                const FVector& PA = UniqueVertices[A].Position;
                const FVector& PB = UniqueVertices[B].Position;
                const FVector& PC = UniqueVertices[C].Position;
                FVector Face = (PB - PA) ^ (PC - PA);
                if (FlipWinding) Face = -Face;
                // Triangles are wound so this cross product points into the solid, against
                // the outward vertex normals; one that points with them is folded over.
                const FVector Smooth = FVector(UniqueVertices[A].Normal + UniqueVertices[B].Normal + UniqueVertices[C].Normal);
                if (FVector::DotProduct(Face, Smooth) > 0.0)
                    NumFolded++;
            }
        }
    }
    InChunk->DebugFoldedTriangles = NumFolded;
    if (NumFolded > 0)
        FoldedTriangles.fetch_add(NumFolded, std::memory_order_relaxed);

    TSharedPtr<FMeshStreamData> UpdatedSurfaceData = InChunk->MakeStream();

    auto SrfPositionStream = UpdatedSurfaceData->GetPositionStream();
    auto SrfTangentStream = UpdatedSurfaceData->GetTangentStream();
    auto SrfColorStream = UpdatedSurfaceData->GetColorStream();
    auto SrfTexCoordStream = UpdatedSurfaceData->GetTexCoordStream();
    auto SrfTriangleStream = UpdatedSurfaceData->GetTriangleStream();
    auto SrfPolygroupStream = UpdatedSurfaceData->GetPolygroupStream();

    SrfPositionStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTangentStream.SetNumUninitialized(UniqueVertices.Num());
    SrfColorStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTexCoordStream.SetNumUninitialized(UniqueVertices.Num());
    SrfTriangleStream.SetNumUninitialized(SrfTriangles.Num());
    SrfPolygroupStream.SetNumUninitialized(SrfTriangles.Num());

    FVector ChunkCenter = InChunk->ChunkCenter;

    ParallelFor(UniqueVertices.Num(), [&](int32 VertIdx) {
        FMeshVertex& Vertex = UniqueVertices[VertIdx];
        FVector WorldPos = FVector(Vertex.Position) + ChunkCenter;

        SrfPositionStream.Set(VertIdx, Vertex.Position);
        FRealtimeMeshTangentsHighPrecision Tangent;
        Tangent.SetNormal(Vertex.Normal);
        SrfTangentStream.Set(VertIdx, Tangent);
        SrfColorStream.Set(VertIdx, Vertex.Color);
        SrfTexCoordStream.Set(VertIdx, ComputeTriplanarUV(WorldPos, Vertex.Normal));
        });

    ParallelFor(SrfTriangles.Num(), [&](int32 TriIdx) {
        SrfTriangleStream.Set(TriIdx, SrfTriangles[TriIdx]);
        SrfPolygroupStream.Set(TriIdx, 0);
        });

    InChunk->SeamHash = SeamHash;
    InChunk->Publish(UpdatedSurfaceData, CurrentPassId);
}

void FAdaptiveOctree::AppendUniqueEdges(const TArray<FNodeEdge>& InAppendEdges, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap)
{
    for (const FNodeEdge& anEdge : InAppendEdges)
    {
        const FEdgeKey& Key = anEdge.CachedKey;
        int32* ExistingIdx = EdgeMap.Find(Key);

        if (ExistingIdx)
        {
            if (anEdge.Size < OutNodeEdges[*ExistingIdx].Size)
            {
                OutNodeEdges[*ExistingIdx] = anEdge;
            }
        }
        else
        {
            int32 NewIdx = OutNodeEdges.Num();
            OutNodeEdges.Add(anEdge);
            EdgeMap.Add(Key, NewIdx);
        }
    }
}

void FAdaptiveOctree::UpdateLodSplits(FAdaptiveOctreeNode* Node, FVector CameraPosition, double ThresholdSq, double MergeThresholdSq, double InFOVScale, TArray<FAdaptiveOctreeNode*>& OutSplitNodes, TArray<FAdaptiveOctreeNode*>& OutMergeCandidates)
{
    if (Node->IsLeaf())
    {
        // Only nodes near the surface, with edit history, or with merged-away surface can
        // split -- deep interior/exterior nodes are skipped.
        const bool bMaySplit = Node->CouldContainSurface || Node->bHasEditedDescendants || Node->bHadSurfaceDescendants;
        if (bMaySplit && Node->ShouldSplit(CameraPosition, ThresholdSq, InFOVScale))
        {
            SplitAndComputeChildren(Node);
            OutSplitNodes.Add(Node);
            return;
        }

        // NOTE: !Node->bIsChunkRoot is load-bearing. A merge destroys all 8 children of
        // Node's PARENT. If Node is a chunk root, its parent is above the chunk cut and its
        // siblings are other chunk roots, possibly being walked by other threads. Coarsening
        // the chunk cut is the demote pass's job; LOD stops at the chunk root.
        if (Node->Parent.IsValid() && !Node->bIsChunkRoot && Node->Index.LastChild() == 7)
        {
            FAdaptiveOctreeNode* ParentPtr = Node->Parent.Pin().Get();
            if (ParentPtr && ParentPtr->ShouldMerge(CameraPosition, MergeThresholdSq, InFOVScale))
            {
                bool bAllSiblingsLeaves = true;
                for (int i = 0; i < 8; i++)
                {
                    if (ParentPtr->Children[i].IsValid() && !ParentPtr->Children[i]->IsLeaf())
                    {
                        bAllSiblingsLeaves = false;
                        break;
                    }
                }
                if (bAllSiblingsLeaves)
                    OutMergeCandidates.Add(ParentPtr);
            }
        }
        return;
    }

    for (int i = 0; i < 8; i++)
        if (Node->Children[i])
            UpdateLodSplits(Node->Children[i].Get(), CameraPosition, ThresholdSq, MergeThresholdSq, InFOVScale, OutSplitNodes, OutMergeCandidates);
}

void FAdaptiveOctree::UpdateLOD(FVector CameraPosition, double InScreenSpaceThreshold, double InCameraFOV)
{
    if (!MeshChunksInitialized) return;

    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    ChunkMap.GenerateKeyArray(Chunks);
    int32 NumChunks = Chunks.Num();

    // Pre-compute once for entire pass
    double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));
    double ThresholdSq = InScreenSpaceThreshold * InScreenSpaceThreshold;
    double MergeThresholdSq = (InScreenSpaceThreshold * 0.5) * (InScreenSpaceThreshold * 0.5);

    // Cache for edit pipeline — EnforceSplitsInSubtree needs these
    CachedCameraPosition = CameraPosition;
    CachedFOVScale = FOVScale;
    CachedThresholdSq = ThresholdSq;

    // 1. Parallel per chunk: splits, and merge candidates. Merges look outside the chunk
    //    (balance), so they wait for the serial step.
    TArray<TArray<FAdaptiveOctreeNode*>> SplitsPerChunk;
    TArray<TArray<FAdaptiveOctreeNode*>> MergesPerChunk;
    SplitsPerChunk.SetNum(NumChunks);
    MergesPerChunk.SetNum(NumChunks);

    ParallelFor(NumChunks, [&](int32 idx) {
        FAdaptiveOctreeNode* ChunkNode = Chunks[idx].Get();
        double DistSq = FMath::Max(FVector::DistSquared(ChunkNode->Center, CameraPosition), 1e-12);
        bool CouldSplit = FAdaptiveOctreeNode::EvaluateSplit(
            ChunkNode->Extent, DistSq, FOVScale, ThresholdSq,
            ChunkNode->Index.Depth,
            ChunkNode->DepthBounds[EDepthBound::MaxDepth],    // MaxDepth param
            ChunkNode->DepthBounds[EDepthBound::MinDepth]);
        double SmallestExtent = ChunkNode->Extent / (double)(1 << (ChunkNode->DepthBounds[EDepthBound::MaxDepth] - ChunkNode->Index.Depth));
        bool CouldMerge = FAdaptiveOctreeNode::EvaluateMerge(
            SmallestExtent, DistSq, FOVScale, MergeThresholdSq,
            ChunkNode->DepthBounds[EDepthBound::MaxDepth],    // Depth = deepest possible node
            ChunkNode->DepthBounds[EDepthBound::ChunkDepth],  // ChunkDepth
            ChunkNode->DepthBounds[EDepthBound::MinDepth]);   // MinDepth

        if (!CouldSplit && !CouldMerge) return; //Early out

        UpdateLodSplits(ChunkNode, CameraPosition, ThresholdSq, MergeThresholdSq, FOVScale, SplitsPerChunk[idx], MergesPerChunk[idx]);
        });

    TSet<FAdaptiveOctreeNode*> ChangedRoots;
    TArray<FBox> ChangedRegions;
    TArray<FAdaptiveOctreeNode*> SplitNodes;
    for (int32 idx = 0; idx < NumChunks; idx++)
    {
        if (SplitsPerChunk[idx].Num() == 0) continue;
        ChangedRoots.Add(Chunks[idx].Get());
        for (FAdaptiveOctreeNode* Split : SplitsPerChunk[idx])
        {
            SplitNodes.Add(Split);
            ChangedRegions.Add(NodeBox(Split));
        }
    }

    // 2. Serial: split whatever the new detail left more than one level coarser.
    BalanceAfterSplits(SplitNodes, ChangedRoots, ChangedRegions);

    // 3. Serial: merges that keep the leaves balanced (checked against the balanced tree,
    //    so a merge never has to be split again by this pass's balancing).
    for (int32 idx = 0; idx < NumChunks; idx++)
    {
        for (FAdaptiveOctreeNode* Candidate : MergesPerChunk[idx])
        {
            if (Candidate->IsLeaf()) continue;
            bool bAllLeaves = true;
            for (int i = 0; i < 8; i++)
                if (Candidate->Children[i].IsValid() && !Candidate->Children[i]->IsLeaf()) { bAllLeaves = false; break; }
            if (!bAllLeaves || !CanMergeBalanced(Candidate)) continue;

            Candidate->Merge();
            ChangedRoots.Add(Chunks[idx].Get());
            ChangedRegions.Add(NodeBox(Candidate));
        }
    }

    // 4. Vertices, edges and meshes for everything that changed.
    FinishPass(ChangedRoots, ChangedRegions);
}

bool FAdaptiveOctree::CanMergeBalanced(FAdaptiveOctreeNode* Parent)
{
    const int32 ParentDepth = Parent->Index.Depth;
    const bool bOwnSurface = Parent->HasOwnSurface();

    TArray<FAdaptiveOctreeNode*> Touching;
    const FBox Box = NodeBox(Parent, PositionTolerance(Parent->Extent));
    CollectLeavesInBox(Root.Get(), Box.Min, Box.Max, false, Touching);

    for (const FAdaptiveOctreeNode* N : Touching)
    {
        if (N->Index.IsWithin(Parent->Index)) continue;
        const int32 Depth = N->Index.Depth;
        // A finer surface leaf next to the merged node would be two levels away from it.
        if (Depth > ParentDepth + 1 && N->HasOwnSurface()) return false;
        // The merged node's own crossings need neighbours at most one level coarser.
        if (bOwnSurface && Depth < ParentDepth - 1) return false;
    }
    return true;
}

void FAdaptiveOctree::BalanceAfterSplits(const TArray<FAdaptiveOctreeNode*>& SplitNodes, TSet<FAdaptiveOctreeNode*>& InOutChangedRoots, TArray<FBox>& InOutChangedRegions)
{
    // Restricted octree: every leaf touching a surface leaf is at most one level coarser.
    // Quads around a surface leaf's edges then join leaves of at most two adjacent depths,
    // and a coarser leaf's vertex can be built from its finer neighbours' edges.
    TArray<FAdaptiveOctreeNode*> Work;
    for (FAdaptiveOctreeNode* Seed : SplitNodes)
    {
        if (Seed->IsLeaf()) { Work.Add(Seed); continue; }
        for (int i = 0; i < 8; i++)
            if (Seed->Children[i].IsValid())
                Work.Add(Seed->Children[i].Get());
    }

    TArray<FAdaptiveOctreeNode*> Touching;
    while (Work.Num() > 0)
    {
        FAdaptiveOctreeNode* Leaf = Work.Pop(EAllowShrinking::No);
        if (!Leaf->IsLeaf() || !Leaf->HasOwnSurface()) continue;

        const int32 Depth = Leaf->Index.Depth;
        Touching.Reset();
        const FBox Box = NodeBox(Leaf, PositionTolerance(Leaf->Extent));
        CollectLeavesInBox(Root.Get(), Box.Min, Box.Max, false, Touching);

        bool bSplitAny = false;
        for (FAdaptiveOctreeNode* N : Touching)
        {
            if (N == Leaf || !N->IsLeaf()) continue;
            if ((int32)N->Index.Depth + 1 >= Depth) continue;
            if (N->Index.Depth >= N->DepthBounds[EDepthBound::MaxDepth]) continue;

            // Only leaves of meshed chunks: leaves above the chunk cut, and chunk roots
            // without surface, are never LOD'd back down. Vertices don't depend on balance
            // (ComputeLeafVertex takes boundary edges from any finer leaf).
            FAdaptiveOctreeNode* NRoot = GetChunkNodeByPoint(N->Center);
            if (!NRoot || !ChunkMap.Contains(NRoot->AsShared())) continue;

            SplitAndComputeChildren(N);
            bSplitAny = true;
            InOutChangedRoots.Add(NRoot);
            InOutChangedRegions.Add(NodeBox(N));
            for (int i = 0; i < 8; i++)
                if (N->Children[i].IsValid())
                    Work.Add(N->Children[i].Get());
        }
        if (bSplitAny)
            Work.Add(Leaf);
    }
}

void FAdaptiveOctree::ComputeLeafVertex(FAdaptiveOctreeNode* Leaf)
{
    const double Tol = PositionTolerance(Leaf->Extent);
    const FBox Box = NodeBox(Leaf);
    const FBox Search = NodeBox(Leaf, Tol);

    TArray<FAdaptiveOctreeNode*> Touching;
    CollectLeavesInBox(Root.Get(), Search.Min, Search.Max, true, Touching);

    TArray<FNodeEdge> Boundary;
    for (const FAdaptiveOctreeNode* N : Touching)
    {
        if (N == Leaf || N->Index.Depth < Leaf->Index.Depth) continue;
        for (const FNodeEdge& Edge : N->GetSignChangeEdges())
        {
            if (!IsEdgeOnBoxBoundary(Edge, Box.Min, Box.Max, Tol)) continue;

            bool bKnown = false;
            for (const FNodeEdge& Own : Leaf->GetSignChangeEdges())
                if (Own.CachedKey == Edge.CachedKey) { bKnown = true; break; }
            for (int32 i = 0; i < Boundary.Num() && !bKnown; i++)
                if (Boundary[i].CachedKey == Edge.CachedKey) bKnown = true;
            if (!bKnown)
                Boundary.Add(Edge);
        }
    }

    Leaf->SolveVertexWithBoundaryEdges(Boundary);
}

void FAdaptiveOctree::RefreshVertices(const TArray<FBox>& ChangedRegions, TSet<FAdaptiveOctreeNode*>& OutVertexChangedRoots)
{
    // Leaves inside or touching a changed region: their own edges or their neighbours'
    // edges may have changed.
    TArray<FAdaptiveOctreeNode*> Candidates;
    for (const FBox& Region : ChangedRegions)
    {
        const double Tol = PositionTolerance(Region.GetExtent().GetMax());
        CollectLeavesInBox(Root.Get(), Region.Min - FVector(Tol), Region.Max + FVector(Tol), false, Candidates);
    }

    TSet<FAdaptiveOctreeNode*> Unique;
    TArray<FAdaptiveOctreeNode*> Leaves;
    Leaves.Reserve(Candidates.Num());
    for (FAdaptiveOctreeNode* Leaf : Candidates)
    {
        // Leaves above the chunk cut are included: a quad at a chunk's edge can use one.
        bool bAlready = false;
        Unique.Add(Leaf, &bAlready);
        if (!bAlready)
            Leaves.Add(Leaf);
    }

    TArray<uint8> Moved;
    Moved.SetNumZeroed(Leaves.Num());
    ParallelFor(Leaves.Num(), [&](int32 i) {
        FAdaptiveOctreeNode* Leaf = Leaves[i];
        const FVector OldPos = Leaf->DualContourPosition;
        const FVector3f OldNormal = Leaf->DualContourNormal;
        const bool bOldData = Leaf->bHasVertexData;
        ComputeLeafVertex(Leaf);
        Moved[i] = (OldPos != Leaf->DualContourPosition || OldNormal != Leaf->DualContourNormal
            || bOldData != Leaf->bHasVertexData) ? 1 : 0;
        });

    for (int32 i = 0; i < Leaves.Num(); i++)
    {
        if (!Moved[i]) continue;
        if (FAdaptiveOctreeNode* LeafRoot = GetChunkNodeByPoint(Leaves[i]->Center))
            OutVertexChangedRoots.Add(LeafRoot);
    }
}

void FAdaptiveOctree::FinishPass(TSet<FAdaptiveOctreeNode*>& ChangedRoots, const TArray<FBox>& ChangedRegions)
{
    if (ChangedRoots.Num() == 0 && ChangedRegions.Num() == 0) return;

    // Leaves whose vertex moved re-mesh their chunk (their neighbours follow through the
    // seam check); only roots whose leaves changed regather their edges.
    TSet<FAdaptiveOctreeNode*> VertexChanged;
    RefreshVertices(ChangedRegions, VertexChanged);

    // A changed root that now bears surface gets a chunk; existing chunks regather their
    // edges from the current leaves. New chunks enter the map before anything is meshed.
    TArray<TSharedPtr<FAdaptiveOctreeNode>> ChangedNodes;
    TArray<FAdaptiveOctreeNode*> GatherNodes;
    TArray<TSharedPtr<FMeshChunk>> GatherChunks;
    for (FAdaptiveOctreeNode* ChangedRoot : ChangedRoots)
    {
        const TSharedPtr<FAdaptiveOctreeNode> Key = ChangedRoot->AsShared();
        ChangedNodes.Add(Key);
        if (TSharedPtr<FMeshChunk>* Found = ChunkMap.Find(Key))
        {
            GatherNodes.Add(ChangedRoot);
            GatherChunks.Add(*Found);
        }
        else if (ChangedRoot->CouldContainSurface)
        {
            ChunkMap.Add(Key, CreateMeshChunk(ChangedRoot));
        }
    }
    for (FAdaptiveOctreeNode* MovedRoot : VertexChanged)
    {
        if (!ChangedRoots.Contains(MovedRoot))
            ChangedNodes.Add(MovedRoot->AsShared());
    }

    ParallelFor(GatherNodes.Num(), [&](int32 i) {
        TArray<FNodeEdge> Edges;
        TMap<FEdgeKey, int32> EdgeMap;
        GatherLeafEdges(GatherNodes[i], Edges, EdgeMap);
        GatherChunks[i]->ChunkEdges = MoveTemp(Edges);
        });

    MeshChunksAndNeighbours(ChangedNodes);
}

void FAdaptiveOctree::CollectDirtyChunks(TArray<TSharedPtr<FMeshChunk>>& OutDirtyChunks) const
{
    if (!MeshChunksInitialized) return;

    for (const auto& It : ChunkMap)
    {
        const TSharedPtr<FMeshChunk>& Chunk = It.Value;
        if (Chunk.IsValid() && Chunk->IsDirty())
            OutDirtyChunks.Add(Chunk);
    }
}

// ---------------------------------------------------------------------------
// Chunk cut
// ---------------------------------------------------------------------------

bool FAdaptiveOctree::ChunkJitterExceeds(FAdaptiveOctreeNode* Node, FVector CameraPosition, double FOVScale, double ThresholdSq) const
{
    // Near distance = distance from the camera to the closest point of the node's AABB,
    // floored so that standing on the surface (near-dist -> 0) doesn't demand infinite
    // depth. This is the key difference from the LOD test, which uses center distance:
    // precision is about how close the *surface under your feet* is, not the chunk center
    // (which can be half a planet away for a coarse chunk).
    const FVector C = Node->Center;
    const double  E = Node->Extent;
    const FVector Closest(
        FMath::Clamp(CameraPosition.X, C.X - E, C.X + E),
        FMath::Clamp(CameraPosition.Y, C.Y - E, C.Y + E),
        FMath::Clamp(CameraPosition.Z, C.Z - E, C.Z + E));

    constexpr double MinNearDist = 100.0; // cm floor (~eye height)
    const double NearDistSq = FMath::Max(FVector::DistSquared(Closest, CameraPosition), MinNearDist * MinNearDist);

    // Worst-case FVector3f rounding error at this node's extent: vertices are stored
    // chunk-local, so the largest coordinate magnitude is ~Extent and the absolute error
    // there is Extent * FLT_EPSILON.
    constexpr double FloatEps = 1.1920929e-7;
    const double Jitter = E * FloatEps;

    // Same screen-space form as EvaluateSplit (2 * size * FOVScale, compared against
    // threshold * distance), but the "size" is the jitter magnitude.
    const double lhs = 2.0 * Jitter * FOVScale;
    return (lhs * lhs) > ThresholdSq * NearDistSq;
}

bool FAdaptiveOctree::AllChildrenAreChunkRoots(FAdaptiveOctreeNode* Node) const
{
    if (!Node || Node->IsLeaf()) return false;
    for (int i = 0; i < 8; i++)
    {
        const FAdaptiveOctreeNode* Child = Node->Children[i].Get();
        if (!Child || !Child->bIsChunkRoot) return false;
    }
    return true;
}

void FAdaptiveOctree::CollectAllChunkRoots(FAdaptiveOctreeNode* Node, TArray<TSharedPtr<FAdaptiveOctreeNode>>& Out) const
{
    if (!Node) return;
    if (Node->bIsChunkRoot) { Out.Add(Node->AsShared()); return; }  // anti-chain: nothing deeper is a root
    if (Node->IsLeaf()) return;
    for (int i = 0; i < 8; i++)
        CollectAllChunkRoots(Node->Children[i].Get(), Out);
}

bool FAdaptiveOctree::AnyFaceNeighborChunkDeeperThan(FAdaptiveOctreeNode* Node, int DepthThreshold)
{
    const FVector Ctr = Node->Center;
    const double  E = Node->Extent;
    const double  Out = E * 1.01;   // just past the face plane

    // Resolve neighbours finely enough to catch one that VIOLATES the balance delta. A
    // neighbour up to (delta+1) levels finer than Node has 2^(delta+1) cells across the
    // face, so sample that many per axis; a coarser grid lets a small deep patch hide
    // between samples.
    const int    N = 1 << (MaxChunkDepthDelta + 1);
    const double Cell = (2.0 * E) / (double)N;
    const double Start = -E + 0.5 * Cell;

    static const FVector FaceNormals[6] = {
        FVector(1, 0, 0), FVector(-1, 0, 0),
        FVector(0, 1, 0), FVector(0, -1, 0),
        FVector(0, 0, 1), FVector(0, 0, -1) };

    for (int f = 0; f < 6; f++)
    {
        const FVector Nrm = FaceNormals[f];
        FVector T1, T2;
        if (FMath::Abs(Nrm.X) > 0.5) { T1 = FVector(0, 1, 0); T2 = FVector(0, 0, 1); }
        else if (FMath::Abs(Nrm.Y) > 0.5) { T1 = FVector(1, 0, 0); T2 = FVector(0, 0, 1); }
        else { T1 = FVector(1, 0, 0); T2 = FVector(0, 1, 0); }

        for (int a = 0; a < N; a++)
            for (int b = 0; b < N; b++)
            {
                const FVector P = Ctr + Nrm * Out + T1 * (Start + a * Cell) + T2 * (Start + b * Cell);
                if (const FAdaptiveOctreeNode* Nb = GetChunkNodeByPoint(P))
                    if ((int)Nb->Index.Depth > DepthThreshold)
                        return true;   // found a delta-violating neighbour; that's all we need
            }
    }
    return false;
}

TSharedPtr<FMeshChunk> FAdaptiveOctree::CreateMeshChunk(FAdaptiveOctreeNode* Node)
{
    TSharedPtr<FMeshChunk> NewChunk = MakeShared<FMeshChunk>();
    NewChunk->CachedParentActor = CachedParentActor;
    NewChunk->CachedMeshAttachRoot = CachedMeshAttachRoot;
    NewChunk->CachedSurfaceMaterial = CachedSurfaceMaterial;
    NewChunk->InitializeData(Node->Center, Node->Extent);

    TArray<FNodeEdge> Edges;
    TMap<FEdgeKey, int32> EdgeMap;
    GatherLeafEdges(Node, Edges, EdgeMap);
    NewChunk->ChunkEdges = Edges;
    return NewChunk;
}

void FAdaptiveOctree::PromoteChunk(TSharedPtr<FAdaptiveOctreeNode> ChunkNode, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks)
{
    // Pull the coarse chunk out of the map and clear its flag. It stays alive (still
    // rendering) in OutRemovedChunks until its finer children have applied.
    TSharedPtr<FMeshChunk> OldChunk;
    ChunkMap.RemoveAndCopyValue(ChunkNode, OldChunk);
    ChunkNode->bIsChunkRoot = false;

    // Every child becomes a chunk root so chunk lookups/edits still resolve inside the
    // promoted region. All are flagged before any is meshed, so each child's mesher sees
    // its siblings as chunks when deciding which chunk emits a shared boundary edge.
    for (int i = 0; i < 8; i++)
    {
        if (ChunkNode->Children[i].IsValid())
            ChunkNode->Children[i]->bIsChunkRoot = true;
    }

    // Only surface-bearing children get a meshed FMeshChunk; all are in the map before
    // any is meshed.
    TArray<TSharedPtr<FMeshChunk>> Created;
    TArray<FAdaptiveOctreeNode*> CreatedNodes;
    for (int i = 0; i < 8; i++)
    {
        const TSharedPtr<FAdaptiveOctreeNode>& Child = ChunkNode->Children[i];
        if (!Child.IsValid() || !Child->CouldContainSurface) continue;

        TSharedPtr<FMeshChunk> NewChunk = CreateMeshChunk(Child.Get());
        ChunkMap.Add(Child, NewChunk);
        Created.Add(NewChunk);
        CreatedNodes.Add(Child.Get());
    }
    for (int32 i = 0; i < Created.Num(); i++)
        UpdateMeshChunkStreamData(CreatedNodes[i], Created[i]);

    // Retire the coarse chunk, remembering the children that now cover it so the destroy
    // drain won't release it until they're live. bRetired also stops the apply drain from
    // (re)creating a component for it if it was still queued.
    if (OldChunk.IsValid())
    {
        OldChunk->bRetired = true;
        OldChunk->ReplacedBy = MoveTemp(Created);
        OutRemovedChunks.Add(OldChunk);
    }
}

void FAdaptiveOctree::DemoteChunk(TSharedPtr<FAdaptiveOctreeNode> ParentNode, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks)
{
    // Move the chunk-root flag up first, so the parent's mesher sees the whole subtree as
    // one chunk. Then build the parent chunk (re-meshing its subtree against its own
    // coarser origin, which also re-covers any non-surface sub-regions the children
    // skipped) so the retired children can point at it.
    for (int i = 0; i < 8; i++)
    {
        if (ParentNode->Children[i].IsValid())
            ParentNode->Children[i]->bIsChunkRoot = false;
    }
    ParentNode->bIsChunkRoot = true;

    TSharedPtr<FMeshChunk> ParentChunk;
    if (ParentNode->CouldContainSurface)
    {
        ParentChunk = CreateMeshChunk(ParentNode.Get());
        ChunkMap.Add(ParentNode, ParentChunk);
        UpdateMeshChunkStreamData(ParentNode.Get(), ParentChunk);
    }

    // Retire the 8 child chunks. They stay alive (still rendering their finer meshes)
    // until the parent chunk has applied, so the collapse never opens a hole.
    for (int i = 0; i < 8; i++)
    {
        const TSharedPtr<FAdaptiveOctreeNode>& Child = ParentNode->Children[i];
        if (!Child.IsValid()) continue;

        TSharedPtr<FMeshChunk> OldChunk;
        if (ChunkMap.RemoveAndCopyValue(Child, OldChunk) && OldChunk.IsValid())
        {
            OldChunk->bRetired = true;
            if (ParentChunk.IsValid())
                OldChunk->ReplacedBy = { ParentChunk };
            OutRemovedChunks.Add(OldChunk);
        }
    }
}

void FAdaptiveOctree::UpdateChunkCut(FVector InCameraPosition, double InCameraFOV, TArray<TSharedPtr<FMeshChunk>>& OutRemovedChunks)
{
    if (!MeshChunksInitialized) return;

    const double FOVScale = 1.0 / FMath::Tan(FMath::DegreesToRadians(InCameraFOV * 0.5));
    const double PromoteThresholdSq = ChunkPrecisionThreshold * ChunkPrecisionThreshold;
    const double DemoteThresholdSq = FMath::Square(ChunkPrecisionThreshold * ChunkDemoteHysteresis);

    // Snapshot ALL chunk roots (not just ChunkMap's surface ones) so non-surface chunk
    // roots can trigger their parent's demote -- otherwise a promoted parent whose children
    // all came out non-surface never collapses and deadlocks the cut. Each chunk changes at
    // most one level per pass, spreading the work over frames.
    TArray<TSharedPtr<FAdaptiveOctreeNode>> Chunks;
    CollectAllChunkRoots(Root.Get(), Chunks);

    // Parents already collapsed this pass, so their (snapshotted) siblings skip the retry.
    TSet<FAdaptiveOctreeNode*> DemotedParents;

    for (const TSharedPtr<FAdaptiveOctreeNode>& C : Chunks)
    {
        // May have been retired by a sibling's demote earlier in this loop.
        if (!C.IsValid() || !C->bIsChunkRoot) continue;

        const int Depth = (int)C->Index.Depth;

        // Promotion is only worthwhile if it actually produces a surface (meshable) child;
        // otherwise it just mints non-surface roots (and, since those demote right back,
        // oscillates). A chunk with no surface-bearing child has nothing to refine -- it
        // only participates in demote.
        bool bHasSurfaceChild = false;
        if (!C->IsLeaf())
            for (int i = 0; i < 8; i++)
            {
                const FAdaptiveOctreeNode* Ch = C->Children[i].Get();
                if (Ch && Ch->CouldContainSurface) { bHasSurfaceChild = true; break; }
            }
        const bool bCanGoDeeper = bHasSurfaceChild && (Depth < MaxChunkDepth);

        // --- Promote (precision): too coarse for its near distance. ---
        if (bCanGoDeeper &&
            ChunkJitterExceeds(C.Get(), InCameraPosition, FOVScale, PromoteThresholdSq))
        {
            PromoteChunk(C, OutRemovedChunks);
            continue;
        }

        // --- Promote (balance): never sit more than MaxChunkDepthDelta levels coarser than
        //     a face neighbour, or the leaf floors differ too much for the mesher to stitch
        //     and the boundary cracks. Force-refine toward the finer neighbour. ---
        if (bCanGoDeeper &&
            AnyFaceNeighborChunkDeeperThan(C.Get(), Depth + MaxChunkDepthDelta))
        {
            PromoteChunk(C, OutRemovedChunks);
            continue;
        }

        // --- Demote: collapse C and its 7 siblings back into the parent once the parent
        //     (one step coarser) is comfortably precise by the tighter demote threshold. ---
        TSharedPtr<FAdaptiveOctreeNode> Parent = C->Parent.Pin();
        if (!Parent.IsValid()) continue;
        const int ParentDepth = (int)Parent->Index.Depth;
        if (ParentDepth < ChunkDepth) continue;                      // never collapse below the build floor
        if (DemotedParents.Contains(Parent.Get())) continue;
        if (!AllChildrenAreChunkRoots(Parent.Get())) continue;       // must be the frontier
        if (ChunkJitterExceeds(Parent.Get(), InCameraPosition, FOVScale, DemoteThresholdSq)) continue; // parent still needs the detail

        // Balance: don't coarsen if it would leave the parent more than MaxChunkDepthDelta
        // levels shallower than a still-fine neighbour (that's the crack we're avoiding).
        // The finer neighbour collapses first; this parent follows on a later pass.
        if (AnyFaceNeighborChunkDeeperThan(Parent.Get(), ParentDepth + MaxChunkDepthDelta)) continue;

        DemoteChunk(Parent, OutRemovedChunks);
        DemotedParents.Add(Parent.Get());
    }
}

void FAdaptiveOctree::FindStaleSeams(TArray<FBox>& OutStaleBounds)
{
    if (!MeshChunksInitialized) return;

    TArray<FAdaptiveOctreeNode*> Nodes;
    TArray<FMeshChunk*> Meshes;
    Nodes.Reserve(ChunkMap.Num());
    Meshes.Reserve(ChunkMap.Num());
    for (const auto& It : ChunkMap)
    {
        if (!It.Key.IsValid() || !It.Value.IsValid()) continue;
        Nodes.Add(It.Key.Get());
        Meshes.Add(It.Value.Get());
    }

    TArray<uint8> Stale;
    Stale.SetNumZeroed(Nodes.Num());
    ParallelFor(Nodes.Num(), [&](int32 i) {
        Stale[i] = (ComputeSeamHash(Nodes[i], *Meshes[i]) != Meshes[i]->SeamHash) ? 1 : 0;
        });

    for (int32 i = 0; i < Nodes.Num(); i++)
    {
        if (Stale[i])
            OutStaleBounds.Add(FBox(Nodes[i]->Center - FVector(Nodes[i]->Extent), Nodes[i]->Center + FVector(Nodes[i]->Extent)));
    }
}

void FAdaptiveOctree::FindFoldedChunks(TArray<FBox>& OutFoldedBounds) const
{
    for (const auto& It : ChunkMap)
    {
        if (!It.Key.IsValid() || !It.Value.IsValid() || It.Value->DebugFoldedTriangles == 0) continue;
        OutFoldedBounds.Add(NodeBox(It.Key.Get()));
    }
}

void FAdaptiveOctree::GetDebugStats(FOctreeDebugStats& Out) const
{
    for (const auto& It : ChunkMap)
    {
        const FMeshChunk* Chunk = It.Value.Get();
        if (!Chunk) continue;
        Out.NumChunks++;
        if (!Chunk->IsDirty()) continue;
        Out.NumDirty++;
        const uint64 Frame = Chunk->PublishFrame.load(std::memory_order_relaxed);
        if (Out.OldestDirtyPublishFrame == 0 || Frame < Out.OldestDirtyPublishFrame)
            Out.OldestDirtyPublishFrame = Frame;
    }
}

void FAdaptiveOctree::GatherLeafEdges(FAdaptiveOctreeNode* Node, TArray<FNodeEdge>& OutEdges, TMap<FEdgeKey, int32>& EdgeMap)
{
    if (!Node) return;
    if (Node->IsLeaf())
    {
        AppendUniqueEdges(Node->GetSignChangeEdges(), OutEdges, EdgeMap);
        return;
    }
    for (int i = 0; i < 8; i++)
        GatherLeafEdges(Node->Children[i].Get(), OutEdges, EdgeMap);
}

void FAdaptiveOctree::SplitAndComputeChildren(FAdaptiveOctreeNode* Node)
{
    Node->Split();

    // --- Stage 1: Build grid positions ---
    FVector GridPositions[27];
    double  GridDensities[27];

    // G0-G7: parent corners
    for (int i = 0; i < 8; i++)
    {
        GridPositions[i] = Node->Corners[i].Position;
        GridDensities[i] = Node->Corners[i].Density;
    }

    // G8-G19: edge midpoints
    for (int i = 0; i < 12; i++)
    {
        int a = OctreeConstants::EdgePairs[i][0];
        int b = OctreeConstants::EdgePairs[i][1];
        GridPositions[8 + i] = (Node->Corners[a].Position + Node->Corners[b].Position) * 0.5;
    }

    // G20-G25: face centers
    for (int i = 0; i < 6; i++)
    {
        GridPositions[20 + i] = (
            Node->Corners[OctreeConstants::FaceCorners[i][0]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][1]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][2]].Position +
            Node->Corners[OctreeConstants::FaceCorners[i][3]].Position) * 0.25;
    }

    // G26: body center
    GridPositions[26] = Node->Center;

    // --- Stage 2: Compute 19 new densities (the 8 parent corners are reused) ---
    {
        const int32 Count = 19;
        double SX[19], SY[19], SZ[19];
        float SampleOut[19];

        for (int32 i = 0; i < Count; i++)
        {
            SX[i] = GridPositions[8 + i].X;
            SY[i] = GridPositions[8 + i].Y;
            SZ[i] = GridPositions[8 + i].Z;
        }

        FSampleInput SampleIn(SX, SY, SZ, Count, SampleFootprint);
        Compositor->Sample(SampleIn, SampleOut);

        for (int32 i = 0; i < Count; i++)
        {
            GridDensities[8 + i] = SampleOut[i];
        }
    }

    // --- Stage 3: Compute 27 normals from grid gradients ---
    FVector GridNormals[27];

    for (int32 gi = 0; gi < 27; gi++)
    {
        int8 cx = OctreeConstants::GridCoords[gi][0];
        int8 cy = OctreeConstants::GridCoords[gi][1];
        int8 cz = OctreeConstants::GridCoords[gi][2];

        double dX;
        if (cx <= 0)
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 2][cy + 1][cz + 1];
            dX = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx][cy + 1][cz + 1];
            dX = -(GridDensities[ngi] - GridDensities[gi]);
        }

        double dY;
        if (cy <= 0)
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy + 2][cz + 1];
            dY = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy][cz + 1];
            dY = -(GridDensities[ngi] - GridDensities[gi]);
        }

        double dZ;
        if (cz <= 0)
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy + 1][cz + 2];
            dZ = GridDensities[ngi] - GridDensities[gi];
        }
        else
        {
            int32 ngi = OctreeConstants::CoordToGrid[cx + 1][cy + 1][cz];
            dZ = -(GridDensities[ngi] - GridDensities[gi]);
        }

        FVector Normal(dX, dY, dZ);
        if (!Normal.Normalize()) Normal = GridPositions[gi].GetSafeNormal();
        GridNormals[gi] = Normal;
    }

    // --- Stage 4: Assign to children and finalize ---
    bool bParentHasEditedDescendants = Node->bHasEditedDescendants;

    for (int ci = 0; ci < 8; ci++)
    {
        for (int k = 0; k < 8; k++)
        {
            int32 gi = OctreeConstants::ChildCornerSources[ci][k];
            Node->Children[ci]->Corners[k].Position = GridPositions[gi];
            Node->Children[ci]->Corners[k].Density = (float)GridDensities[gi];
            Node->Children[ci]->Corners[k].Normal = FVector3f(GridNormals[gi]);
        }
        Node->Children[ci]->FinalizeFromExistingCorners(true); // normals already computed from grid

        // If the parent had edited descendants, check each child against the edit
        // store directly — only flag children whose region actually contains edits.
        if (bParentHasEditedDescendants && Compositor.IsValid())
        {
            auto EditStore = Compositor->GetEditStore();
            if (EditStore.IsValid() && EditStore->HasEditsAlongPath(Node->Children[ci]->Index))
            {
                Node->Children[ci]->bHasEditedDescendants = true;
                Node->Children[ci]->CouldContainSurface = true;
            }
        }
    }
}

void FAdaptiveOctree::ComputeNodeData(FAdaptiveOctreeNode* Node)
{
    double SX[8], SY[8], SZ[8];
    float SampleOut[8];

    for (int i = 0; i < 8; i++)
    {
        SX[i] = Node->Corners[i].Position.X;
        SY[i] = Node->Corners[i].Position.Y;
        SZ[i] = Node->Corners[i].Position.Z;
    }

    FSampleInput SampleIn(SX, SY, SZ, 8, SampleFootprint);
    Compositor->Sample(SampleIn, SampleOut);

    for (int i = 0; i < 8; i++)
    {
        Node->Corners[i].Density = SampleOut[i];
    }

    Node->FinalizeFromExistingCorners();
}

FEdgeNeighbors FAdaptiveOctree::SampleNodesAroundEdge(const FNodeEdge& Edge)
{
    FEdgeNeighbors Result;

    // Calculate the exact mathematical midpoint of the edge
    FVector Origin = Edge.Corners[0].Position;
    FVector End = Edge.Corners[1].Position;
    FVector Midpoint = (Origin + End) * 0.5;

    double Epsilon = ChunkExtent * 1e-8;

    // Helper lambda for biased top-down traversal -- uses raw pointers throughout
    auto GetLeafWithBias = [&](bool BiasPerp1, bool BiasPerp2) -> FAdaptiveOctreeNode* {
        FAdaptiveOctreeNode* CurrentNode = Root.Get();

        while (CurrentNode && !CurrentNode->IsLeaf()) {
            int ChildIndex = 0;

            auto CheckSide = [&](int AxisIndex, bool PositiveBias) {
                double Diff = Midpoint[AxisIndex] - CurrentNode->Center[AxisIndex];
                if (FMath::Abs(Diff) <= Epsilon) {
                    return PositiveBias;
                }
                return Diff > 0.0;
                };

            if (Edge.Axis == 0) { // X-axis edge
                if (Midpoint.X >= CurrentNode->Center.X) ChildIndex |= 1;
                if (CheckSide(1, BiasPerp1)) ChildIndex |= 2;
                if (CheckSide(2, BiasPerp2)) ChildIndex |= 4;
            }
            else if (Edge.Axis == 1) { // Y-axis edge
                if (CheckSide(0, BiasPerp2)) ChildIndex |= 1;
                if (Midpoint.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
                if (CheckSide(2, BiasPerp1)) ChildIndex |= 4;
            }
            else if (Edge.Axis == 2) { // Z-axis edge
                if (CheckSide(0, BiasPerp1)) ChildIndex |= 1;
                if (CheckSide(1, BiasPerp2)) ChildIndex |= 2;
                if (Midpoint.Z >= CurrentNode->Center.Z) ChildIndex |= 4;
            }

            FAdaptiveOctreeNode* Child = CurrentNode->Children[ChildIndex].Get();
            if (!Child) return CurrentNode; // Shouldn't happen in normal operation
            CurrentNode = Child;
        }
        return CurrentNode;
        };

    FAdaptiveOctreeNode* N0 = GetLeafWithBias(true, true);
    FAdaptiveOctreeNode* N1 = GetLeafWithBias(false, true);
    FAdaptiveOctreeNode* N2 = GetLeafWithBias(false, false);
    FAdaptiveOctreeNode* N3 = GetLeafWithBias(true, false);

    double MaxNodeExtent = Edge.Size * 2.0;
    if (N0 && N0->Extent <= MaxNodeExtent) Result.AddUnique(N0);
    if (N1 && N1->Extent <= MaxNodeExtent) Result.AddUnique(N1);
    if (N2 && N2->Extent <= MaxNodeExtent) Result.AddUnique(N2);
    if (N3 && N3->Extent <= MaxNodeExtent) Result.AddUnique(N3);

    return Result;
}

FAdaptiveOctreeNode* FAdaptiveOctree::GetLeafNodeByPoint(FVector Position)
{
    FAdaptiveOctreeNode* CurrentNode = Root.Get();
    while (CurrentNode)
    {
        if (CurrentNode->IsLeaf())
        {
            return CurrentNode;
        }

        int ChildIndex = 0;
        if (Position.X >= CurrentNode->Center.X) ChildIndex |= 1;
        if (Position.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
        if (Position.Z >= CurrentNode->Center.Z) ChildIndex |= 4;

        FAdaptiveOctreeNode* Child = CurrentNode->Children[ChildIndex].Get();
        if (Child)
        {
            CurrentNode = Child;
        }
        else
        {
            break;
        }
    }

    return nullptr;
}

FAdaptiveOctreeNode* FAdaptiveOctree::GetChunkNodeByPoint(FVector Position)
{
    FAdaptiveOctreeNode* CurrentNode = Root.Get();
    while (CurrentNode)
    {
        if (CurrentNode->bIsChunkRoot)
        {
            return CurrentNode;
        }

        int ChildIndex = 0;
        if (Position.X >= CurrentNode->Center.X) ChildIndex |= 1;
        if (Position.Y >= CurrentNode->Center.Y) ChildIndex |= 2;
        if (Position.Z >= CurrentNode->Center.Z) ChildIndex |= 4;

        FAdaptiveOctreeNode* Child = CurrentNode->Children[ChildIndex].Get();
        if (Child)
        {
            CurrentNode = Child;
        }
        else
        {
            break;
        }
    }

    return nullptr;
}

void FAdaptiveOctree::Clear()
{
    ChunkMap.Empty();
    Root.Reset();
    MeshChunksInitialized = false;
}
