// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <FAdaptiveOctreeNode.h>
#include <FSparseEditStore.h>

struct VOXELPLUGIN_API FCorner
{
private:
    std::atomic<int> RefCount = 0;
    FRWLock Mutex;
    FInt64Vector PositionKey = FInt64Vector::ZeroValue;
    FVector3f Normal = FVector3f(1, 0, 0);
    double Density = 0.0;

public:

    FCorner() {};

    FCorner(FVector InPosition) {
        PositionKey = FCorner::QuantizeKey(InPosition);
    }

    FCorner(FInt64Vector InKey) {
        PositionKey = InKey;
    }

    FORCEINLINE const FInt64Vector GetKey() const{
        return PositionKey;
    }

    FORCEINLINE FVector GetPosition() const
    {
        // Best way: Explicitly use the FVector constructor with double casts
        return FVector(
            static_cast<double>(PositionKey.X),
            static_cast<double>(PositionKey.Y),
            static_cast<double>(PositionKey.Z)
        );
    }

    const double GetDensity();
    
    const FVector3f GetNormal();
    
    void SetDensity(double InDensity);
    
    void SetNormal(FVector3f InNormal);

    void SetDensityAndNormal(double InDensity, FVector3f InNormal);

    static FORCEINLINE FInt64Vector QuantizeKey(const FVector& Position) {
        return FInt64Vector(
            (int64)FMath::RoundToDouble(Position.X),
            (int64)FMath::RoundToDouble(Position.Y),
            (int64)FMath::RoundToDouble(Position.Z)
        );
    }

    FORCEINLINE void AddRef() {
        RefCount.fetch_add(1, std::memory_order_relaxed);
    }

    FORCEINLINE bool Release() {
        return RefCount.fetch_sub(1, std::memory_order_acq_rel) <= 1;
    }

    FORCEINLINE int32 GetRefCount() const { 
        return RefCount.load(); 
    }
};

struct VOXELPLUGIN_API FEdgeKey
{
    FCorner* A; //Min Corner
    FCorner* B; //Max Corner

    FEdgeKey() : A(nullptr), B(nullptr) {}

    FEdgeKey(FCorner* InA, FCorner* InB)
    {
        A = InA; B = InB;
    }

    bool operator==(const FEdgeKey& Other) const
    {
        return A == Other.A && B == Other.B;
    }

    static FEdgeKey Generate(FCorner* A, FCorner* B) {
        FInt64Vector RawDelta = A->GetKey() - B->GetKey();

        int64 Magnitude = 0;
        bool bSwap = false;

        if (RawDelta.X != 0) {
            Magnitude = RawDelta.X;
            bSwap = RawDelta.X < 0;
        }
        else if (RawDelta.Y != 0) {
            Magnitude = RawDelta.Y;
            bSwap = RawDelta.Y < 0;
        }
        else {
            Magnitude = RawDelta.Z;
            bSwap = RawDelta.Z < 0;
        }

        FCorner* MinC = bSwap ? B : A;
        FCorner* MaxC = bSwap ? A : B;

        return FEdgeKey(MinC, MaxC);
    }

    // This stays inside the struct but behaves as a global function
    friend FORCEINLINE uint32 GetTypeHash(const FEdgeKey& Key)
    {
        // Use PointerHash for the actual pointer values
        return HashCombine(PointerHash(Key.A), PointerHash(Key.B));
    }
};

struct VOXELPLUGIN_API FEdge
{
private:
    std::atomic<int> RefCount = 0;
    FRWLock Mutex;
    FEdgeKey Key;

    double Size = 0.0;
    short Axis = 0;

    FEdge* Parent = nullptr;
    //Connected faces at the same level, for neighbor finding, need to deterministically populate order
    struct FFace* ConnectedFaces[4] = { nullptr, nullptr, nullptr, nullptr };

public:

    FEdge(FCorner* InCorner1, FCorner* InCorner2, FEdge* InParent) : Parent(InParent) {
        FInt64Vector RawDelta = InCorner2->GetKey() - InCorner1->GetKey();

        int64 Magnitude = 0;
        bool bSwap = false;

        if (RawDelta.X != 0) {
            Axis = 0;
            Magnitude = RawDelta.X;
            bSwap = RawDelta.X < 0;
        }
        else if (RawDelta.Y != 0) {
            Axis = 1;
            Magnitude = RawDelta.Y;
            bSwap = RawDelta.Y < 0;
        }
        else {
            Axis = 2;
            Magnitude = RawDelta.Z;
            bSwap = RawDelta.Z < 0;
        }

        FCorner* MinC = bSwap ? InCorner2 : InCorner1;
        FCorner* MaxC = bSwap ? InCorner1 : InCorner2;

        Key = FEdgeKey(MinC, MaxC);
        Size = static_cast<double>(FMath::Abs(Magnitude));
    }

    FEdge(FEdgeKey InKey, FEdge* InParent) : FEdge(InKey.A, InKey.B, InParent) {};

    FORCEINLINE const FEdgeKey GetKey() const {
        return Key;
    }

    FORCEINLINE FCorner* GetMinCorner() {
        return Key.A;
    }

    FORCEINLINE const FCorner* GetMinCorner() const{
        return Key.A;
    }
    
    FORCEINLINE FCorner* GetMaxCorner() {
        return Key.B;
    }

    FORCEINLINE const FCorner* GetMaxCorner() const{
        return Key.B;
    }

    FORCEINLINE const double GetSize() const {
        return Size;
    }

    FORCEINLINE const short GetAxis() const {
        return Axis;
    }

    const short GetSignChange() const;

    const FVector GetZeroCrossingPoint() const;

    const FFace* GetConnectedFace(uint8 idx);

    void RegisterConnectedFace(FFace* InFace);

    FORCEINLINE void AddRef() {
        RefCount.fetch_add(1, std::memory_order_relaxed);
    }

    FORCEINLINE bool Release() {
        return RefCount.fetch_sub(1, std::memory_order_acq_rel) <= 1;
    }

    FORCEINLINE int32 GetRefCount() const {
        return RefCount.load();
    }
};

struct VOXELPLUGIN_API FFaceKey
{
    FCorner* Min;
    FCorner* Max;
    int32 Axis; // 0, 1, or 2

    FFaceKey() : Min(nullptr), Max(nullptr), Axis(-1) {}
    FFaceKey(FCorner* InMin, FCorner* InMax, short InAxis)
        : Min(InMin), Max(InMax), Axis(InAxis) {}

    static FFaceKey Generate(FCorner* InCorners[4], short InAxis) {
        // 1. Identify plane axes (U and V)
        int32 U = (InAxis + 1) % 3;
        int32 V = (InAxis + 2) % 3;

        // 2. Sort the 4 corners to find the absolute Min and Max for the FFaceKey.
        // In our quantized 1cm grid, Min is the corner with the lowest U and V.
        FCorner* MinC = InCorners[0];
        FCorner* MaxC = InCorners[0];

        for (int i = 1; i < 4; i++)
        {
            const FInt64Vector& K = InCorners[i]->GetKey();
            const FInt64Vector& MK = MinC->GetKey();
            const FInt64Vector& XK = MaxC->GetKey();

            // Standard 2D Sort: Compare U, then V
            if (K[U] < MK[U] || (K[U] == MK[U] && K[V] < MK[V])) MinC = InCorners[i];
            if (K[U] > XK[U] || (K[U] == XK[U] && K[V] > XK[V])) MaxC = InCorners[i];
        }

        // 3. Set the Key and AddRef the anchors
        return FFaceKey(MinC, MaxC, InAxis);
    }

    bool operator==(const FFaceKey& Other) const {
        return Min == Other.Min && Max == Other.Max && Axis == Other.Axis;
    }

    friend FORCEINLINE uint32 GetTypeHash(const FFaceKey& Key) {
        uint32 Hash = HashCombine(PointerHash(Key.Min), PointerHash(Key.Max));
        return HashCombine(Hash, GetTypeHash(Key.Axis));
    }
};

struct VOXELPLUGIN_API FFace {
    std::atomic<int> RefCount = 0;
    FRWLock Mutex;
    FFaceKey Key;

    FCorner* Corners[4];
    FEdge* Edges[4];

    // The nodes at THIS LOD level
    TWeakPtr<FAdaptiveOctreeNode> Nodes[2];

    // One-way upward link
    FFace* Parent = nullptr;

    short Axis; // 0, 1, or 2 (X, Y, or Z normal)

    FFace() : Parent(nullptr) {
        Nodes[0] = Nodes[1] = nullptr;
        for (int i = 0; i < 4; i++) { Corners[i] = nullptr; Edges[i] = nullptr; }
    }

    FFace(FEdge* InEdges[4], short InAxis, FFace* InParent) : Axis(InAxis), Parent(InParent)
    {
        int32 U = (Axis + 1) % 3;
        int32 V = (Axis + 2) % 3;

        // FIX: We need a reference point for "Min" and "Max" before the Key exists.
        // We'll just find the min/max coordinates from the input edges.
        int64 MinU = MAX_int64, MaxU = MIN_int64;
        int64 MinV = MAX_int64, MaxV = MIN_int64;

        for (int i = 0; i < 4; ++i) {
            const FInt64Vector& K1 = InEdges[i]->GetMinCorner()->GetKey();
            const FInt64Vector& K2 = InEdges[i]->GetMaxCorner()->GetKey();
            MinU = FMath::Min(MinU, FMath::Min(K1[U], K2[U]));
            MaxU = FMath::Max(MaxU, FMath::Max(K1[U], K2[U]));
            MinV = FMath::Min(MinV, FMath::Min(K1[V], K2[V]));
            MaxV = FMath::Max(MaxV, FMath::Max(K1[V], K2[V]));
        }

        // 1. CANONICAL EDGE SORTING
        for (int32 i = 0; i < 4; ++i)
        {
            FEdge* E = InEdges[i];
            FInt64Vector P = E->GetMinCorner()->GetKey();

            if (E->GetAxis() == U) {
                // Lower V is Bottom (0), Higher V is Top (2)
                Edges[(P[V] == MinV) ? 0 : 2] = E;
            }
            else {
                // Higher U is Right (1), Lower U is Left (3)
                Edges[(P[U] == MaxU) ? 1 : 3] = E;
            }
        }

        // 2. CORNER EXTRACTION (Winding: BL -> BR -> TR -> TL)
        Corners[0] = Edges[0]->GetMinCorner(); // (MinU, MinV)
        Corners[1] = Edges[0]->GetMaxCorner(); // (MaxU, MinV)
        Corners[2] = Edges[2]->GetMaxCorner(); // (MaxU, MaxV)
        Corners[3] = Edges[2]->GetMinCorner(); // (MinU, MaxV)

        // 3. NOW GENERATE KEY
        Key = FFaceKey(Corners[0], Corners[2], Axis);

        // 4. EDGE-LINKING
        for (int32 i = 0; i < 4; i++) {
            Edges[i]->RegisterConnectedFace(this);
        }
    }

    void RegisterNode(TSharedPtr<FAdaptiveOctreeNode> InNode);

    void GetNodes(TSharedPtr<FAdaptiveOctreeNode> OutNodes[2]);

    FORCEINLINE void AddRef() {
        RefCount.fetch_add(1, std::memory_order_relaxed);
    }

    FORCEINLINE bool Release() {
        return RefCount.fetch_sub(1, std::memory_order_acq_rel) <= 1;
    }

    FORCEINLINE int32 GetRefCount() const {
        return RefCount.load();
    }
};

class VOXELPLUGIN_API FNodeStructureProvider : public TSharedFromThis<FNodeStructureProvider>
{
private:
    TSharedPtr<FSparseEditStore> EditStore;
    /** Mutexes marked mutable to allow locking in const-pointer scenarios */
    mutable FRWLock CornerLock;
    mutable FRWLock EdgeLock;
    mutable FRWLock FaceLock;

    /** Memory-stable storage for primitives */
    TChunkedArray<FCorner> CornerStore;
    TChunkedArray<FEdge> EdgeStore;
    TChunkedArray<FFace> FaceStore;

    /** Fast lookups for existing primitives */
    TMap<FInt64Vector, FCorner*> CornerLookup;
    TMap<FEdgeKey, FEdge*> EdgeLookup;
    TMap<FFaceKey, FFace*> FaceLookup;

    /** Free lists for recycling memory in planetary LOD shifting */
    TArray<int32> FreeCornerIndices;
    TArray<int32> FreeEdgeIndices;
    TArray<int32> FreeFaceIndices;

    /** * Internal Factory Methods:
     * These assume the caller is already holding the appropriate WriteLock.
     */
    FCorner* Internal_CreateCorner(const FInt64Vector& InKey);
    FEdge* Internal_CreateEdge(const FEdgeKey& InKey, FEdge* InParent);
    FFace* Internal_CreateFace(FEdge* InEdges[4], int32 InAxis, FFace* InParent, TSharedPtr<FAdaptiveOctreeNode> InNode);

public:
    FNodeStructureProvider();
    ~FNodeStructureProvider();

    /** SIMD Density Function: Handed in from the Planet Actor or Volume */
    TFunction<void(int32 Count, const float* X, const float* Y, const float* Z, float* OutDensities)> DensityFunction;

    /** High-level batch population for Octree Splits */
    void PopulateNodeStructure(const TArray<TSharedPtr<class FAdaptiveOctreeNode>>& InNodes);

    /** Batch update for runtime terrain edits */
    void UpdateCorners(const TArray<FCorner*>& CornersToUpdate);

    /** Thread-safe Acquisition API */
    FCorner* GetOrCreateCorner(const FVector& InPosition);
    FEdge* GetOrCreateEdge(FCorner* InC1, FCorner* InC2, FEdge* InParent);
    FFace* GetOrCreateFace(FEdge* InEdges[4], int32 InAxis, FFace* InFace, TSharedPtr<FAdaptiveOctreeNode> InNode);

    /** * Cleanup: Optional method to prune the TChunkedArrays
     * using the FreeLists when RefCounts hit zero.
     */
    void PruneUnusedStructures();
};
