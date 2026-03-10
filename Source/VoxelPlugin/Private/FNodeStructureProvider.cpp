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
FCorner* FNodeStructureProvider::Internal_CreateCorner(const FInt64Vector& InKey)
{
    FCorner NewCorner = nullptr;

    // A. Check if we can recycle a deleted corner's memory
    if (FreeCornerIndices.Num() > 0)
    {
        int32 RecycledIdx = FreeCornerIndices.Pop();
        NewCorner = &CornerStore[RecycledIdx];

        // Placement new or manual reset to ensure atomics/mutexes are fresh
        NewCorner = FCorner(InKey);
    }
    else
    {
        // B. Grow the ChunkedArray
        int32 NewIdx = CornerStore.AddElement(FCorner(InPosition));
        NewCorner = &CornerStore[NewIdx];
    }

    // Register in the map so others can find it
    CornerLookup.Add(InKey, NewCorner);

    return NewCorner;
}

FEdge* FNodeStructureProvider::Internal_CreateEdge(FCorner* InC1, FCorner* InC2, const FEdgeKey& InKey)
{
    return nullptr;
}

FFace* FNodeStructureProvider::Internal_CreateFace(FCorner* InC1, FCorner* InC2, int32 InAxis, const FFaceKey& InKey)
{
    return nullptr;
}

FNodeStructureProvider::FNodeStructureProvider()
{
}

FNodeStructureProvider::~FNodeStructureProvider()
{
}

void FNodeStructureProvider::PopulateNodeStructure(const TArray<TSharedPtr<class FAdaptiveOctreeNode>>& InNodes)
{
}

void FNodeStructureProvider::UpdateCorners(const TArray<FCorner*>& CornersToUpdate)
{
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
    return Internal_CreateCorner(InPosition, Key);
}

FEdge* FNodeStructureProvider::GetOrCreateEdge(FCorner* InC1, FCorner* InC2)
{
    return nullptr;
}

FFace* FNodeStructureProvider::GetOrCreateFace(FCorner* InC1, FCorner* InC2, int32 Axis)
{
    return nullptr;
}

void FNodeStructureProvider::PruneUnusedStructures()
{
}