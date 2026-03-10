// Fill out your copyright notice in the Description page of Project Settings.


#include "FNodeStructureProvider.h"

//FCORNER STUFF
const double FCorner::GetDensity() {
    FReadScopeLock ReadLock(Mutex);
    return Density;
}

const FVector3f FCorner::GetNormal() {
    FReadScopeLock ReadLock(Mutex);
    return Normal;
}

void FCorner::SetDensity(double InDensity) {
    FWriteScopeLock WriteLock(Mutex);
    Density = InDensity;
}

void FCorner::SetNormal(FVector3f InNormal) {
    FWriteScopeLock WriteLock(Mutex);
    Normal = InNormal;
}

void FCorner::SetDensityAndNormal(double InDensity, FVector3f InNormal) {
    FWriteScopeLock WriteLock(Mutex);
    Density = InDensity;
    Normal = InNormal;
}

//FEDGE STUFF
const short FEdge::GetSignChange() const {
    return FMath::Sign(Key.A->GetDensity() - Key.B->GetDensity());
}

const FVector FEdge::GetZeroCrossingPoint() const {
    double d0 = Key.A->GetDensity();
    double d1 = Key.B->GetDensity();
    double t = d0 / (d0 - d1);
    return FMath::Lerp(Key.A->GetPosition(), Key.B->GetPosition(), t);
}

const FFace* FEdge::GetConnectedFace(uint8 idx) {
    FReadScopeLock ReadLock(Mutex);
    return ConnectedFaces[idx];
}

void FEdge::RegisterConnectedFace(FFace* InFace) {
    FWriteScopeLock WriteLock(Mutex);

    // 1. Identify the two axes perpendicular to this edge
    // If Axis is 0 (X), O1 is 1 (Y) and O2 is 2 (Z)
    int32 O1 = (Axis + 1) % 3;
    int32 O2 = (Axis + 2) % 3;

    // 2. Get the position of the face relative to the edge
    FVector EdgePos = Key.A->GetPosition();
    FVector FacePos = InFace->Key.Min->GetPosition();

    // 3. Determine the quadrant (Binary 00, 01, 10, 11)
    bool bHigherO1 = FacePos[O1] >= EdgePos[O1];
    bool bHigherO2 = FacePos[O2] >= EdgePos[O2];

    // Map to 0-3 index
    int32 Slot = (bHigherO1 ? 1 : 0) | (bHigherO2 ? 2 : 0);

    ConnectedFaces[Slot] = InFace;
}

//FACE STUFF
void FFace::RegisterNode(TSharedPtr<FAdaptiveOctreeNode> InNode)
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

void FFace::GetNodes(TSharedPtr<FAdaptiveOctreeNode> OutNodes[2])
{
    FReadScopeLock ReadLock(Mutex);
    OutNodes[0] = Nodes[0].IsValid() ? Nodes[0].Pin() : nullptr;
    OutNodes[1] = Nodes[1].IsValid() ? Nodes[1].Pin() : nullptr;
}

//STRUCTURE PROVIDER
FNodeStructureProvider::FNodeStructureProvider()
{
}

FNodeStructureProvider::~FNodeStructureProvider()
{
}

void FNodeStructureProvider::PopulateNodeStructure(const TArray<TSharedPtr<class FAdaptiveOctreeNode>>& InNodes)
{
    // Corners that need their first-ever sample
    TArray<FCorner*> NewCorners;

    // 1. ACQUIRE & ASSIGN
    for (const auto& Node : InNodes)
    {
        for (int32 i = 0; i < 8; ++i)
        {
            FVector Pos = Node->GetCornerPosition(i);
            FCorner* Corner = GetOrCreateCorner(Pos);

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
        const float Epsilon = 1.0f; // 1cm

        int32 TotalCount = N * 7;
        TArray<float> X, Y, Z, OutDensities;
        X.SetNumUninitialized(TotalCount);
        Y.SetNumUninitialized(TotalCount);
        Z.SetNumUninitialized(TotalCount);
        OutDensities.SetNum(TotalCount);

        for (int32 i = 0; i < N; ++i)
        {
            FVector P = NewCorners[i]->GetPosition();
            int32 Base = i * 7;

            // Center + 6 Gradient offsets
            X[Base] = P.X; Y[Base] = P.Y; Z[Base] = P.Z;

            // X-axis
            X[Base + 1] = P.X + Epsilon; X[Base + 2] = P.X - Epsilon;
            Y[Base + 1] = P.Y; Y[Base + 2] = P.Y; Z[Base + 1] = P.Z; Z[Base + 2] = P.Z;
            // Y-axis
            Y[Base + 3] = P.Y + Epsilon; Y[Base + 4] = P.Y - Epsilon;
            X[Base + 3] = P.X; X[Base + 4] = P.X; Z[Base + 3] = P.Z; Z[Base + 4] = P.Z;
            // Z-axis
            Z[Base + 5] = P.Z + Epsilon; Z[Base + 6] = P.Z - Epsilon;
            X[Base + 5] = P.X; X[Base + 6] = P.X; Y[Base + 5] = P.Y; Y[Base + 6] = P.Y;
        }

        // SIMD Density Pass - this is why we pack the normal samples in addition to the corner samples - so we can get them all in one simd call
        DensityFunction(TotalCount, X.GetData(), Y.GetData(), Z.GetData(), OutDensities.GetData());

        // 3. RESOLVE & STORE
        for (int32 i = 0; i < N; ++i)
        {
            int32 Base = i * 7;

            auto GetFinalD = [&](int32 Offset) {
                FVector SampleP(X[Base + Offset], Y[Base + Offset], Z[Base + Offset]);
                return OutDensities[Base + Offset] + EditStore->Sample(SampleP);
            };
            FVector Gradient(
                GetFinalD(1) - GetFinalD(2),
                GetFinalD(3) - GetFinalD(4),
                GetFinalD(5) - GetFinalD(6)
            );
            //Final set unless the corner is edited later
            NewCorners[i]->SetDensityAndNormal(GetFinalD(0), (FVector3f)Gradient);
        }
    }

    // --- 4. EDGES ---
    for (const auto& Node : InNodes)
    {
        // Pin the parent once per node
        TSharedPtr<FAdaptiveOctreeNode> ParentNode = Node->Parent.Pin();
        uint8 ChildIdx = Node->Index.LastChild();

        for (int32 i = 0; i < 12; ++i)
        {
            FCorner* C1 = Node->Corners[OctreeConstants::EdgePairs[i][0]];
            FCorner* C2 = Node->Corners[OctreeConstants::EdgePairs[i][1]];

            FEdge* ParentEdge = nullptr;
            if (ParentNode.IsValid())
            {
                int32 P_EdgeIdx = OctreeConstants::ChildToParentEdgeMap[ChildIdx][i];
                if (P_EdgeIdx != -1)
                {
                    // Access the parent's edge array through the pinned pointer
                    ParentEdge = ParentNode->Edges[P_EdgeIdx];
                }
            }

            FEdge* Edge = GetOrCreateEdge(C1, C2, ParentEdge);
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
            FEdge* F_Edges[4] = {
                Node->Edges[FE[0]],
                Node->Edges[FE[1]],
                Node->Edges[FE[2]],
                Node->Edges[FE[3]]
            };

            short Axis = OctreeConstants::FaceAxisBit[i];

            FFace* ParentFace = nullptr;
            if (ParentNode.IsValid())
            {
                int32 P_FaceIdx = OctreeConstants::ChildToParentFaceMap[ChildIdx][i];
                if (P_FaceIdx != -1)
                {
                    ParentFace = ParentNode->Faces[P_FaceIdx];
                }
            }

            // GetOrCreateFace now handles the initial registration
            FFace* Face = GetOrCreateFace(F_Edges, Axis, ParentFace, Node);

            Node->Faces[i] = Face;
            Face->AddRef();

            // Handle the neighbor registration for existing faces
            if (Face->GetRefCount() > 1)
            {
                Face->RegisterNode(Node);
            }
        }
    }
}

void FNodeStructureProvider::UpdateCorners(const TArray<FCorner*>& CornersToUpdate)
{
}

FCorner* FNodeStructureProvider::Internal_CreateCorner(const FInt64Vector& InKey)
{
    FCorner* NewCorner = nullptr;

    if (FreeCornerIndices.Num() > 0)
    {
        int32 RecycledIdx = FreeCornerIndices.Pop();
        NewCorner = &CornerStore[RecycledIdx];

        // CORRECT: Placement New. 
        // Re-initializes the existing memory at 'NewCorner' address.
        // This properly resets the atomic RefCount and the Mutex.
        new (NewCorner) FCorner(InKey);
    }
    else
    {
        // AddElement returns the index of the newly constructed item.
        // We pass the Key-based constructor here.
        int32 NewIdx = CornerStore.AddElement(FCorner(InKey));
        NewCorner = &CornerStore[NewIdx];
    }

    // Register in the map
    CornerLookup.Add(InKey, NewCorner);

    return NewCorner;
}

FCorner* FNodeStructureProvider::GetOrCreateCorner(const FVector& InPosition)
{
    FInt64Vector Key = FCorner::QuantizeKey(InPosition);

    // 1. SHARED READ: Check if it exists without blocking other threads
    {
        FReadScopeLock ReadLock(CornerLock);
        if (FCorner* const* Found = CornerLookup.Find(Key))
        {
            return *Found;
        }
    }

    // 2. EXCLUSIVE WRITE: We think it's new, so prepare to allocate
    FWriteScopeLock WriteLock(CornerLock);

    // 3. DOUBLE-CHECK: Did another thread create it while we were waiting for the WriteLock?
    if (FCorner* const* Found = CornerLookup.Find(Key))
    {
        return *Found;
    }

    // 4. ALLOCATE: Actually create the entry
    return Internal_CreateCorner(Key);
}

FEdge* FNodeStructureProvider::Internal_CreateEdge(const FEdgeKey& InKey, FEdge* InParent)
{
    FEdge* NewEdge = nullptr;

    if (FreeEdgeIndices.Num() > 0)
    {
        int32 RecycledIdx = FreeEdgeIndices.Pop();
        NewEdge = &EdgeStore[RecycledIdx];

        // Use Placement New to reset atomics and run sorting logic
        new (NewEdge) FEdge(InKey, InParent);
    }
    else
    {
        // AddElement constructs the FEdge in-place within the chunk
        int32 NewIdx = EdgeStore.AddElement(FEdge(InKey, InParent));
        NewEdge = &EdgeStore[NewIdx];
    }

    // Register in map
    EdgeLookup.Add(InKey, NewEdge);

    return NewEdge;
}

FEdge* FNodeStructureProvider::GetOrCreateEdge(FCorner* InC1, FCorner* InC2, FEdge* InParent)
{
    FEdgeKey SearchKey = FEdgeKey::Generate(InC1, InC2);

    {
        FReadScopeLock ReadLock(EdgeLock);
        if (FEdge* const* Found = EdgeLookup.Find(SearchKey)) return *Found;
    }

    FWriteScopeLock WriteLock(EdgeLock);
    if (FEdge* const* Found = EdgeLookup.Find(SearchKey)) return *Found;

    return Internal_CreateEdge(SearchKey, InParent);
}

FFace* FNodeStructureProvider::Internal_CreateFace(FEdge* InEdges[4], int32 InAxis, FFace* InParent, TSharedPtr<FAdaptiveOctreeNode> InitiatingNode)
{
    FFace* NewFace = nullptr;

    // 1. Construct Face using Placement New
    // The FFace constructor handles edge/corner sorting and key generation
    if (FreeFaceIndices.Num() > 0)
    {
        int32 RecycledIdx = FreeFaceIndices.Pop();
        NewFace = &FaceStore[RecycledIdx];
        new (NewFace) FFace(InEdges, static_cast<short>(InAxis), InParent);
    }
    else
    {
        int32 NewIdx = FaceStore.AddElement(FFace(InEdges, static_cast<short>(InAxis), InParent));
        NewFace = &FaceStore[NewIdx];
    }

    // 2. Immediate Binding of the creator node
    NewFace->RegisterNode(InitiatingNode);

    // 3. Register in map using the key generated by the constructor
    FaceLookup.Add(NewFace->Key, NewFace);

    return NewFace;
}

FFace* FNodeStructureProvider::GetOrCreateFace(FEdge* InEdges[4], int32 Axis, FFace* InParent, TSharedPtr<FAdaptiveOctreeNode> InitiatingNode)
{
    // 1. Generate the spatial handle
    FCorner* KeyCorners[4] = {
        InEdges[0]->GetMinCorner(), InEdges[0]->GetMaxCorner(),
        InEdges[2]->GetMinCorner(), InEdges[2]->GetMaxCorner()
    };
    FFaceKey SearchKey = FFaceKey::Generate(KeyCorners, static_cast<short>(Axis));

    // 2. Thread-safe lookup
    {
        FReadScopeLock ReadLock(FaceLock);
        if (FFace* const* Found = FaceLookup.Find(SearchKey))
        {
            FFace* ExistingFace = *Found;
            // IMPORTANT: Register the node here if it's the second visitor
            ExistingFace->RegisterNode(InitiatingNode);
            return ExistingFace;
        }
    }

    FWriteScopeLock WriteLock(FaceLock);

    // Double-check for race conditions
    if (FFace* const* Found = FaceLookup.Find(SearchKey))
    {
        FFace* ExistingFace = *Found;
        ExistingFace->RegisterNode(InitiatingNode);
        return ExistingFace;
    }

    // 3. Creation path (Internal_CreateFace already handles registration)
    return Internal_CreateFace(InEdges, Axis, InParent, InitiatingNode);
}

void FNodeStructureProvider::PruneUnusedStructures()
{
}