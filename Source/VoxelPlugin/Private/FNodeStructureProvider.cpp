// Fill out your copyright notice in the Description page of Project Settings.


#include "FNodeStructureProvider.h"

//FCORNER STUFF
const double FVoxelCorner::GetDensity() {
    FReadScopeLock ReadLock(Mutex);
    return Density;
}

const FVector3f FVoxelCorner::GetNormal() {
    FReadScopeLock ReadLock(Mutex);
    return Normal;
}

void FVoxelCorner::SetDensity(double InDensity) {
    FWriteScopeLock WriteLock(Mutex);
    Density = InDensity;
}

void FVoxelCorner::SetNormal(FVector3f InNormal) {
    FWriteScopeLock WriteLock(Mutex);
    Normal = InNormal;
}

void FVoxelCorner::SetDensityAndNormal(double InDensity, FVector3f InNormal) {
    FWriteScopeLock WriteLock(Mutex);
    Density = InDensity;
    Normal = InNormal;
}

//FEDGE STUFF
const short FVoxelEdge::GetSignChange() const
{
    // Ensure we aren't dereferencing null corners
    if (!Key.A || !Key.B) return 0;

    double D0 = Key.A->GetDensity();
    double D1 = Key.B->GetDensity();

    // Standard Dual Contouring crossing: 
    // Sign is different if (D0 > 0 && D1 <= 0) or (D0 <= 0 && D1 > 0)
    bool bIn0 = D0 <= 0.0;
    bool bIn1 = D1 <= 0.0;

    if (bIn0 == bIn1) return 0; // Both same side, no crossing.

    return bIn1 ? -1 : 1; // Return direction for potential manifold orientation
}

const FVector FVoxelEdge::GetZeroCrossingPoint() const {
    double d0 = Key.A->GetDensity();
    double d1 = Key.B->GetDensity();
    double t = d0 / (d0 - d1);
    return FMath::Lerp(Key.A->GetPosition(), Key.B->GetPosition(), t);
}

FVector3f FVoxelEdge::GetAverageNormal() const
{
    // 1. Get densities to determine the interpolation weight
    double d0 = Key.A->GetDensity();
    double d1 = Key.B->GetDensity();

    // Avoid division by zero if densities are identical (rare)
    if (FMath::IsNearlyEqual(d0, d1))
    {
        return Key.A->GetNormal();
    }

    // 2. Calculate the 't' value (where the surface crosses the edge)
    // t = 0 means surface is at Corner A, t = 1 means surface is at Corner B
    float t = static_cast<float>(FMath::Clamp(d0 / (d0 - d1), 0.0, 1.0));

    // 3. Linearly interpolate the pre-sampled normals from the corners
    FVector3f NormalA = Key.A->GetNormal();
    FVector3f NormalB = Key.B->GetNormal();

    FVector3f Result = FMath::Lerp(NormalA, NormalB, t);

    // 4. Ensure it's a unit vector for the QEF solver
    Result.Normalize();

    return Result;
}

const FVoxelFace* FVoxelEdge::GetConnectedFace(uint8 idx) {
    return ConnectedFaces[idx];
}

void FVoxelEdge::RegisterConnectedFace(FVoxelFace* InFace)
{
    FWriteScopeLock WriteLock(Mutex);

    int32 U, V;
    if (Axis == 0) { U = 1; V = 2; }
    else if (Axis == 1) { U = 2; V = 0; }
    else { U = 0; V = 1; }

    FVector EdgeCorner = Key.A->GetPosition();
    FVector FaceCenter = InFace->GetCenter();

    double DiffU = FaceCenter[U] - EdgeCorner[U];
    double DiffV = FaceCenter[V] - EdgeCorner[V];

    int32 Slot;
    if (!FMath::IsNearlyZero(DiffU))
    {
        Slot = (DiffU < 0.0) ? 0 : 2;
    }
    else
    {
        Slot = (DiffV < 0.0) ? 1 : 3;
    }

    // Count how many slots are filled before and after
    int32 FilledBefore = 0;
    for (int32 i = 0; i < 4; i++) if (ConnectedFaces[i]) FilledBefore++;

    ConnectedFaces[Slot] = InFace;

    int32 FilledAfter = 0;
    for (int32 i = 0; i < 4; i++) if (ConnectedFaces[i]) FilledAfter++;

    if (FilledAfter == FilledBefore)
    {
        UE_LOG(LogTemp, Error, TEXT("[Edge] COLLISION: face registered into already-occupied slot %d on axis=%d, DiffU=%f DiffV=%f"), Slot, (int32)Axis, DiffU, DiffV);
    }
}

//FACE STUFF
void FVoxelFace::RegisterNode(TSharedPtr<FAdaptiveOctreeNode> InNode)
{
    FWriteScopeLock WriteLock(Mutex);

    // Get the coordinate of the node center on the face's normal axis

    // Axis 0: X, 1: Y, 2: Z
    double NodeCoord = InNode->Center[Axis];

    // All corners on the face share this coord
    double FaceCoord = Key.Min->GetPosition()[Axis];

    // Slot 0: Node is on the "Negative" side (Min)
    // Slot 1: Node is on the "Positive" side (Max)
    if (NodeCoord < FaceCoord)
    {
        Nodes[0] = InNode;
    }
    else
    {
        Nodes[1] = InNode;
    }
}

void FVoxelFace::GetNodes(TSharedPtr<FAdaptiveOctreeNode> OutNodes[2])
{
    OutNodes[0] = Nodes[0].IsValid() ? Nodes[0].Pin() : nullptr;
    OutNodes[1] = Nodes[1].IsValid() ? Nodes[1].Pin() : nullptr;
}

TSharedPtr<FAdaptiveOctreeNode> FVoxelFace::GetNode(int index)
{
    return Nodes[index].IsValid() ? Nodes[index].Pin() : nullptr;
}

//STRUCTURE PROVIDER
FNodeStructureProvider::FNodeStructureProvider(TSharedPtr<FSparseEditStore> InEditStore, TFunction<void(int32 Count, const float* X, const float* Y, const float* Z, float* OutDensities)> InDensityFunction, double InRootExtent, double InSeaLevel, double InSurfaceLevel, FVector InCenter)
{
    EditStore = InEditStore;
    NoiseFunction = InDensityFunction;
    RootExtent = InRootExtent;
    SeaLevel = InRootExtent * InSeaLevel;
    SurfaceLevel = InRootExtent * InSurfaceLevel; // convert proportion to absolute world-space radius
    Center = InCenter;
}

FNodeStructureProvider::~FNodeStructureProvider()
{
}

//Initializes new node structural data/pointers
void FNodeStructureProvider::PopulateNodeStructure(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes)
{
    // Corners that need their first-ever sample
    TArray<FVoxelCorner*> NewCorners;

    // 1. ACQUIRE & ASSIGN
    for (const auto& Node : InNodes)
    {
        for (int32 i = 0; i < 8; ++i)
        {
            FVector Pos = Node->GetCornerPosition(i);
            FVoxelCorner* Corner = GetOrCreateCorner(Pos);

            Node->Corners[i] = Corner;
            Corner->AddRef();

            // If RefCount is 1, this thread is the first one to ever 
            // assign this corner to a node. It's our responsibility to sample it.
            if (Corner->GetRefCount() == 1)
            {
                NewCorners.Add(Corner);
            }
        }
    }

    // 2. THE 7N SUPER-BATCH - BATCH TOGETHER BOTH THE CORNER DENSITY CALL AND THE NEEDED SAMPLES FOR COMPUTING THE NORMAL
    // THIS ALLOWS US TO SUBMIT THEM ALL AT ONCE TO THE SIMD CALL INSTEAD OF NEEDING MULTIPLE STAGES
    if (NewCorners.Num() > 0)
    {
        int32 N = NewCorners.Num();
        const float Epsilon = 1; // 1cm
        const float InvSurface = (float)(1.0 / SurfaceLevel); // normalize to unit-sphere for FastNoise

        int32 TotalCount = N * 7;
        TArray<float> X, Y, Z;           // world-space positions used by CompositeSample
        TArray<float> NX, NY, NZ;        // unit-sphere positions passed to FastNoise
        TArray<float> OutDensities;
        X.SetNumUninitialized(TotalCount);   Y.SetNumUninitialized(TotalCount);   Z.SetNumUninitialized(TotalCount);
        NX.SetNumUninitialized(TotalCount); NY.SetNumUninitialized(TotalCount); NZ.SetNumUninitialized(TotalCount);
        OutDensities.SetNum(TotalCount);

        for (int32 i = 0; i < N; ++i)
        {
            FVector P = NewCorners[i]->GetPosition();
            int32 Base = i * 7;

            // World-space positions (center + gradient offsets)
            X[Base] = P.X; Y[Base] = P.Y; Z[Base] = P.Z;
            X[Base + 1] = P.X + Epsilon; X[Base + 2] = P.X - Epsilon; Y[Base + 1] = P.Y; Y[Base + 2] = P.Y; Z[Base + 1] = P.Z; Z[Base + 2] = P.Z;
            Y[Base + 3] = P.Y + Epsilon; Y[Base + 4] = P.Y - Epsilon; X[Base + 3] = P.X; X[Base + 4] = P.X; Z[Base + 3] = P.Z; Z[Base + 4] = P.Z;
            Z[Base + 5] = P.Z + Epsilon; Z[Base + 6] = P.Z - Epsilon; X[Base + 5] = P.X; X[Base + 6] = P.X; Y[Base + 5] = P.Y; Y[Base + 6] = P.Y;

            // Normalized positions for FastNoise (planet surface maps to unit sphere)
            for (int32 j = 0; j < 7; ++j)
            {
                NX[Base + j] = X[Base + j] * InvSurface;
                NY[Base + j] = Y[Base + j] * InvSurface;
                NZ[Base + j] = Z[Base + j] * InvSurface;
            }
        }

        // SIMD Density Pass sample at unit-sphere coordinates
        NoiseFunction(TotalCount, NX.GetData(), NY.GetData(), NZ.GetData(), OutDensities.GetData());

        // RESOLVE & STORE CompositeSample uses world-space X/Y/Z for the SDF, OutDensities for noise
        ParallelFor(N, [&](int32 i)
            {
                int32 Base = i * 7;

                auto Sample = [&](int32 Offset) -> double
                    {
                        return CompositeSample(
                            FVector(X[Base + Offset], Y[Base + Offset], Z[Base + Offset]),
                            (double)OutDensities[Base + Offset]);
                    };

                double D0 = Sample(0);
                FVector3f Gradient(
                    (float)(Sample(1) - Sample(2)),
                    (float)(Sample(3) - Sample(4)),
                    (float)(Sample(5) - Sample(6))
                );
                NewCorners[i]->SetDensityAndNormal(D0, Gradient);
            });
    }

    // --- 4. EDGES ---
    for (const auto& Node : InNodes)
    {
        // Pin the parent once per node
        TSharedPtr<FAdaptiveOctreeNode> ParentNode = Node->Parent.Pin();
        uint8 ChildIdx = Node->Index.LastChild();

        for (int32 i = 0; i < 12; ++i)
        {
            FVoxelCorner* C1 = Node->Corners[OctreeConstants::EdgePairs[i][0]];
            FVoxelCorner* C2 = Node->Corners[OctreeConstants::EdgePairs[i][1]];

            FVoxelEdge* ParentEdge = nullptr;
            if (ParentNode.IsValid())
            {
                int32 P_EdgeIdx = OctreeConstants::ChildToParentEdgeMap[ChildIdx][i];
                if (P_EdgeIdx != -1)
                {
                    // Access the parent's edge array through the pinned pointer
                    ParentEdge = ParentNode->Edges[P_EdgeIdx];
                }
            }

            FVoxelEdge* Edge = GetOrCreateEdge(C1, C2, ParentEdge);
            Node->Edges[i] = Edge;
            Edge->AddRef();
        }
    }

    // --- 5. FACES ---
    for (const auto& Node : InNodes)
    {
        // Pin the parent once per node
        TSharedPtr<FAdaptiveOctreeNode> ParentNode = Node->Parent.Pin();
        int32 ChildIdx = Node->Index.LastChild();

        for (int32 i = 0; i < 6; ++i)
        {
            const int32* FE = OctreeConstants::FaceEdges[i];
            FVoxelEdge* F_Edges[4] = {
                Node->Edges[FE[0]],
                Node->Edges[FE[1]],
                Node->Edges[FE[2]],
                Node->Edges[FE[3]]
            };

            short Axis = OctreeConstants::FaceAxisBit[i];

            FVoxelFace* ParentFace = nullptr;
            if (ParentNode.IsValid())
            {
                int32 P_FaceIdx = OctreeConstants::ChildToParentFaceMap[ChildIdx][i];
                if (P_FaceIdx != -1)
                {
                    ParentFace = ParentNode->Faces[P_FaceIdx];
                }
            }

            // GetOrCreateFace now handles the initial registration
            FVoxelFace* Face = GetOrCreateFace(F_Edges, Axis, ParentFace, Node);

            Node->Faces[i] = Face;
            Face->AddRef();
        }
        Node->ComputeDualContourPosition();
        Node->ComputeNormalizedPosition(SeaLevel);
        Node->bDataReady = true;
    }
}

void FNodeStructureProvider::UpdateNodeStructure(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes)
{
    if (InNodes.Num() == 0) return;

    // 1. Gather Unique Existing Corners
    TSet<FVoxelCorner*> UniqueCorners;
    for (const auto& Node : InNodes)
    {
        for (int32 i = 0; i < 8; ++i)
        {
            if (Node->Corners[i]) UniqueCorners.Add(Node->Corners[i]);
        }
    }

    if (UniqueCorners.Num() == 0) return;

    // 2. Prepare 7N Batch
    TArray<FVoxelCorner*> CornerArray = UniqueCorners.Array();
    int32 N = CornerArray.Num();
    const float Epsilon = 1;

    int32 TotalCount = N * 7;
    TArray<float> X, Y, Z;          // world-space used by CompositeSample
    TArray<float> NX, NY, NZ;       // unit-sphere passed to FastNoise
    TArray<float> RawNoise;
    X.SetNumUninitialized(TotalCount);   Y.SetNumUninitialized(TotalCount);   Z.SetNumUninitialized(TotalCount);
    NX.SetNumUninitialized(TotalCount); NY.SetNumUninitialized(TotalCount); NZ.SetNumUninitialized(TotalCount);
    RawNoise.SetNum(TotalCount);

    const float InvSurface = (float)(1.0 / SurfaceLevel);

    for (int32 i = 0; i < N; ++i)
    {
        FVector P = CornerArray[i]->GetPosition();
        int32 Base = i * 7;
        X[Base] = P.X; Y[Base] = P.Y; Z[Base] = P.Z;
        X[Base + 1] = P.X + Epsilon; X[Base + 2] = P.X - Epsilon; Y[Base + 1] = P.Y; Y[Base + 2] = P.Y; Z[Base + 1] = P.Z; Z[Base + 2] = P.Z;
        Y[Base + 3] = P.Y + Epsilon; Y[Base + 4] = P.Y - Epsilon; X[Base + 3] = P.X; X[Base + 4] = P.X; Z[Base + 3] = P.Z; Z[Base + 4] = P.Z;
        Z[Base + 5] = P.Z + Epsilon; Z[Base + 6] = P.Z - Epsilon; X[Base + 5] = P.X; X[Base + 6] = P.X; Y[Base + 5] = P.Y; Y[Base + 6] = P.Y;

        for (int32 j = 0; j < 7; ++j)
        {
            NX[Base + j] = X[Base + j] * InvSurface;
            NY[Base + j] = Y[Base + j] * InvSurface;
            NZ[Base + j] = Z[Base + j] * InvSurface;
        }
    }

    // 3. SIMD Pass sample at unit-sphere coordinates
    NoiseFunction(TotalCount, NX.GetData(), NY.GetData(), NZ.GetData(), RawNoise.GetData());

    // 4. Resolve & Set
    ParallelFor(N, [&](int32 i)
        {
            int32 Base = i * 7;

            auto Sample = [&](int32 Offset) -> double
                {
                    return CompositeSample(
                        FVector(X[Base + Offset], Y[Base + Offset], Z[Base + Offset]),
                        (double)RawNoise[Base + Offset]);
                };

            double D0 = Sample(0);
            FVector3f Gradient(
                (float)(Sample(1) - Sample(2)),
                (float)(Sample(3) - Sample(4)),
                (float)(Sample(5) - Sample(6))
            );

            CornerArray[i]->SetDensityAndNormal(D0, Gradient);
        });

    // 5. Finalize Nodes
    for (const auto& Node : InNodes)
    {
        Node->ComputeDualContourPosition();
        Node->bDataReady = true;
    }
}

void FNodeStructureProvider::ApplyEdit(FVector InCenter, double InRadius, double InStrength, int InResolution, const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InAffectedNodes)
{
    // 1. Write the edit into the sparse store at the appropriate depth
    int32 Depth = EditStore->GetDepthForBrushRadius(InRadius, InResolution);
    EditStore->ApplySphericalEdit(InCenter, InRadius, InStrength, Depth);

    // 2. Resample all affected nodes EditStore->Sample is now baked into GetFinalD
    //    inside UpdateNodeStructure, so a straight resample pass is all we need
    UpdateNodeStructure(InAffectedNodes);
}

FVoxelCorner* FNodeStructureProvider::Internal_CreateCorner(const FInt64Vector& InKey)
{
    FVoxelCorner* NewCorner = nullptr;

    if (FreeCornerIndices.Num() > 0)
    {
        int32 RecycledIdx = FreeCornerIndices.Pop();
        NewCorner = &CornerStore[RecycledIdx];
        // Placement new re-initializes the existing allocation (resets atomic, mutex)
        new (NewCorner) FVoxelCorner(InKey);
    }
    else
    {
        NewCorner = new FVoxelCorner(InKey);
        CornerStore.Add(NewCorner);
    }

    // Register in the map
    CornerLookup.Add(InKey, NewCorner);

    return NewCorner;
}

FVoxelCorner* FNodeStructureProvider::GetOrCreateCorner(const FVector& InPosition)
{
    FInt64Vector Key = FVoxelCorner::QuantizeKey(InPosition);

    // 1. SHARED READ: Check if it exists without blocking other threads
    {
        FReadScopeLock ReadLock(CornerLock);
        if (FVoxelCorner* const* Found = CornerLookup.Find(Key))
        {
            return *Found;
        }
    }

    // 2. EXCLUSIVE WRITE: We think it's new, so prepare to allocate
    FWriteScopeLock WriteLock(CornerLock);

    // 3. DOUBLE-CHECK: Did another thread create it while we were waiting for the WriteLock?
    if (FVoxelCorner* const* Found = CornerLookup.Find(Key))
    {
        return *Found;
    }

    // 4. ALLOCATE: Actually create the entry
    return Internal_CreateCorner(Key);
}

FVoxelEdge* FNodeStructureProvider::Internal_CreateEdge(const FVoxelEdgeKey& InKey, FVoxelEdge* InParent)
{
    FVoxelEdge* NewEdge = nullptr;

    if (FreeEdgeIndices.Num() > 0)
    {
        int32 RecycledIdx = FreeEdgeIndices.Pop();
        NewEdge = &EdgeStore[RecycledIdx];
        new (NewEdge) FVoxelEdge(InKey, InParent);
    }
    else
    {
        NewEdge = new FVoxelEdge(InKey, InParent);
        EdgeStore.Add(NewEdge);
    }

    // Register in map
    EdgeLookup.Add(InKey, NewEdge);

    return NewEdge;
}

FVoxelEdge* FNodeStructureProvider::GetOrCreateEdge(FVoxelCorner* InC1, FVoxelCorner* InC2, FVoxelEdge* InParent)
{
    FVoxelEdgeKey SearchKey = FVoxelEdgeKey::Generate(InC1, InC2);

    {
        FReadScopeLock ReadLock(EdgeLock);
        if (FVoxelEdge* const* Found = EdgeLookup.Find(SearchKey)) return *Found;
    }

    FWriteScopeLock WriteLock(EdgeLock);
    if (FVoxelEdge* const* Found = EdgeLookup.Find(SearchKey)) return *Found;

    return Internal_CreateEdge(SearchKey, InParent);
}

FVoxelFace* FNodeStructureProvider::Internal_CreateFace(FVoxelEdge* InEdges[4], int32 InAxis, FVoxelFace* InParent, TSharedPtr<FAdaptiveOctreeNode> InitiatingNode)
{
    FVoxelFace* NewFace = nullptr;

    // 1. Construct Face using Placement New
    // The FFace constructor handles edge/corner sorting and key generation
    if (FreeFaceIndices.Num() > 0)
    {
        int32 RecycledIdx = FreeFaceIndices.Pop();
        NewFace = &FaceStore[RecycledIdx];
        new (NewFace) FVoxelFace(InEdges, static_cast<short>(InAxis), InParent);
    }
    else
    {
        NewFace = new FVoxelFace(InEdges, static_cast<short>(InAxis), InParent);
        FaceStore.Add(NewFace);
    }

    // 2. Immediate Binding of the creator node
    NewFace->RegisterNode(InitiatingNode);

    // 3. Register in map using the key generated by the constructor
    FaceLookup.Add(NewFace->Key, NewFace);

    return NewFace;
}

FVoxelFace* FNodeStructureProvider::GetOrCreateFace(FVoxelEdge* InEdges[4], int32 Axis, FVoxelFace* InParent, TSharedPtr<FAdaptiveOctreeNode> InitiatingNode)
{
    FVoxelFaceKey SearchKey = FVoxelFaceKey::Generate(InEdges, static_cast<short>(Axis));

    {
        FReadScopeLock ReadLock(FaceLock);
        if (FVoxelFace* const* Found = FaceLookup.Find(SearchKey))
        {
            (*Found)->RegisterNode(InitiatingNode);
            return *Found;
        }
    }

    FWriteScopeLock WriteLock(FaceLock);
    if (FVoxelFace* const* Found = FaceLookup.Find(SearchKey))
    {
        (*Found)->RegisterNode(InitiatingNode);
        return *Found;
    }

    return Internal_CreateFace(InEdges, Axis, InParent, InitiatingNode);
}

void FNodeStructureProvider::PruneUnusedStructures()
{
}