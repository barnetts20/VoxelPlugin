// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include <FAdaptiveOctreeNode.h>
#include <FSparseEditStore.h>

struct VOXELPLUGIN_API FVoxelCorner
{
private:
    std::atomic<int> RefCount = 0;
    FRWLock Mutex;
    FInt64Vector PositionKey = FInt64Vector::ZeroValue;
    FVector3f Normal = FVector3f(1, 0, 0);
    double Density = 0.0;

public:

    FVoxelCorner() {};

    FVoxelCorner(FVector InPosition) {
        PositionKey = FVoxelCorner::QuantizeKey(InPosition);
    }

    FVoxelCorner(FInt64Vector InKey) {
        PositionKey = InKey;
    }

    FORCEINLINE const FInt64Vector GetKey() const {
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

struct VOXELPLUGIN_API FVoxelEdgeKey
{
    FVoxelCorner* A; //Min Corner
    FVoxelCorner* B; //Max Corner

    FVoxelEdgeKey() : A(nullptr), B(nullptr) {}

    FVoxelEdgeKey(FVoxelCorner* InA, FVoxelCorner* InB)
    {
        A = InA; B = InB;
    }

    bool operator==(const FVoxelEdgeKey& Other) const
    {
        return A == Other.A && B == Other.B;
    }

    static FVoxelEdgeKey Generate(FVoxelCorner* A, FVoxelCorner* B) {
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

        FVoxelCorner* MinC = bSwap ? B : A;
        FVoxelCorner* MaxC = bSwap ? A : B;

        return FVoxelEdgeKey(MinC, MaxC);
    }

    // This stays inside the struct but behaves as a global function
    friend FORCEINLINE uint32 GetTypeHash(const FVoxelEdgeKey& Key)
    {
        // Use PointerHash for the actual pointer values
        return HashCombine(PointerHash(Key.A), PointerHash(Key.B));
    }
};

struct VOXELPLUGIN_API FVoxelEdge
{
private:
    std::atomic<int> RefCount = 0;
    FRWLock Mutex;
    FVoxelEdgeKey Key;

    double Size = 0.0;
    short Axis = 0;

    FVoxelEdge* Parent = nullptr;
    //Connected faces at the same level, for neighbor finding, need to deterministically populate order
    struct FVoxelFace* ConnectedFaces[4] = { nullptr, nullptr, nullptr, nullptr };

public:

    FVoxelEdge(FVoxelCorner* InCorner1, FVoxelCorner* InCorner2, FVoxelEdge* InParent) : Parent(InParent) {
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

        FVoxelCorner* MinC = bSwap ? InCorner2 : InCorner1;
        FVoxelCorner* MaxC = bSwap ? InCorner1 : InCorner2;

        Key = FVoxelEdgeKey(MinC, MaxC);
        Size = static_cast<double>(FMath::Abs(Magnitude));
    }

    FVoxelEdge(FVoxelEdgeKey InKey, FVoxelEdge* InParent) : FVoxelEdge(InKey.A, InKey.B, InParent) {};

    FORCEINLINE const FVoxelEdgeKey GetKey() const {
        return Key;
    }

    FORCEINLINE FVoxelCorner* GetMinCorner() {
        return Key.A;
    }

    FORCEINLINE const FVoxelCorner* GetMinCorner() const {
        return Key.A;
    }

    FORCEINLINE FVoxelCorner* GetMaxCorner() {
        return Key.B;
    }

    FORCEINLINE const FVoxelCorner* GetMaxCorner() const {
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

    // In FNodeStructureProvider.cpp

    FVector3f GetAverageNormal() const;

    FVoxelEdge* GetParent() {
        return Parent;
    }

    const FVoxelFace* GetConnectedFace(uint8 idx);

    void RegisterConnectedFace(FVoxelFace* InFace);

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

struct VOXELPLUGIN_API FVoxelFaceKey
{
    FVoxelCorner* Min;
    FVoxelCorner* Max;
    int32 Axis; // 0, 1, or 2

    FVoxelFaceKey() : Min(nullptr), Max(nullptr), Axis(-1) {}
    FVoxelFaceKey(FVoxelCorner* InMin, FVoxelCorner* InMax, short InAxis)
        : Min(InMin), Max(InMax), Axis(InAxis) {}

    static FVoxelFaceKey Generate(FVoxelCorner* InCorners[4], short InAxis) {
        // 1. Identify plane axes (U and V)
        int32 U = (InAxis + 1) % 3;
        int32 V = (InAxis + 2) % 3;

        // 2. Sort the 4 corners to find the absolute Min and Max for the FFaceKey.
        // In our quantized 1cm grid, Min is the corner with the lowest U and V.
        FVoxelCorner* MinC = InCorners[0];
        FVoxelCorner* MaxC = InCorners[0];

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
        return FVoxelFaceKey(MinC, MaxC, InAxis);
    }

    bool operator==(const FVoxelFaceKey& Other) const {
        return Min == Other.Min && Max == Other.Max && Axis == Other.Axis;
    }

    friend FORCEINLINE uint32 GetTypeHash(const FVoxelFaceKey& Key) {
        uint32 Hash = HashCombine(PointerHash(Key.Min), PointerHash(Key.Max));
        return HashCombine(Hash, GetTypeHash(Key.Axis));
    }
};

struct VOXELPLUGIN_API FVoxelFace {
    std::atomic<int> RefCount = 0;
    FRWLock Mutex;
    FVoxelFaceKey Key;

    FVoxelCorner* Corners[4];
    FVoxelEdge* Edges[4];

    // The nodes at THIS LOD level
    TWeakPtr<FAdaptiveOctreeNode> Nodes[2];

    short Axis; // 0, 1, or 2 (X, Y, or Z normal)

    // One-way upward link
    FVoxelFace* Parent = nullptr;

    FVoxelFace() : Parent(nullptr) {
        Nodes[0] = Nodes[1] = nullptr;
        for (int i = 0; i < 4; i++) { Corners[i] = nullptr; Edges[i] = nullptr; }
    }

    FVoxelFace(FVoxelEdge* InEdges[4], short InAxis, FVoxelFace* InParent) : Axis(InAxis), Parent(InParent)
    {
        int32 U = (Axis + 1) % 3;
        int32 V = (Axis + 2) % 3;

        // 1. CANONICAL EDGE SORTING
        // Partition into U-axis edges and V-axis edges, then sort each pair by position.
        FVoxelEdge* UEdges[2] = { nullptr, nullptr }; // will become Slot 0 (bottom) and Slot 2 (top)
        FVoxelEdge* VEdges[2] = { nullptr, nullptr }; // will become Slot 3 (left) and Slot 1 (right)
        int32 UCount = 0, VCount = 0;

        for (int32 i = 0; i < 4; ++i)
        {
            if (InEdges[i]->GetAxis() == U)
            {
                if (UCount < 2) UEdges[UCount++] = InEdges[i];
            }
            else
            {
                if (VCount < 2) VEdges[VCount++] = InEdges[i];
            }
        }

        if (UCount != 2 || VCount != 2)
        {
            // Malformed face input — leave edges null, caller will detect via null corners
            return;
        }

        // Sort U-edges by their V coordinate: lower V = bottom (slot 0), higher V = top (slot 2)
        if (UEdges[0]->GetMinCorner()->GetKey()[V] > UEdges[1]->GetMinCorner()->GetKey()[V])
        {
            Swap(UEdges[0], UEdges[1]);
        }
        Edges[0] = UEdges[0]; // bottom
        Edges[2] = UEdges[1]; // top

        // Sort V-edges by their U coordinate: lower U = left (slot 3), higher U = right (slot 1)
        if (VEdges[0]->GetMinCorner()->GetKey()[U] > VEdges[1]->GetMinCorner()->GetKey()[U])
        {
            Swap(VEdges[0], VEdges[1]);
        }
        Edges[3] = VEdges[0]; // left
        Edges[1] = VEdges[1]; // right

        // 2. CORNER EXTRACTION (Winding: BL -> BR -> TR -> TL)
        Corners[0] = Edges[0]->GetMinCorner(); // (MinU, MinV)
        Corners[1] = Edges[0]->GetMaxCorner(); // (MaxU, MinV)
        Corners[2] = Edges[2]->GetMaxCorner(); // (MaxU, MaxV)
        Corners[3] = Edges[2]->GetMinCorner(); // (MinU, MaxV)

        // 3. NOW GENERATE KEY
        Key = FVoxelFaceKey(Corners[0], Corners[2], Axis);

        // 4. EDGE-LINKING
        for (int32 i = 0; i < 4; i++) {
            Edges[i]->RegisterConnectedFace(this);
        }
    }

    void RegisterNode(TSharedPtr<FAdaptiveOctreeNode> InNode);

    void GetNodes(TSharedPtr<FAdaptiveOctreeNode> OutNodes[2]);

    TSharedPtr<FAdaptiveOctreeNode> GetNode(int index);

    FVoxelFace* GetParent() {
        return Parent;
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

class VOXELPLUGIN_API FNodeStructureProvider : public TSharedFromThis<FNodeStructureProvider>
{
private:
    TSharedPtr<FSparseEditStore> EditStore;
    /** Mutexes marked mutable to allow locking in const-pointer scenarios */
    mutable FRWLock CornerLock;
    mutable FRWLock EdgeLock;
    mutable FRWLock FaceLock;

    FVector Center;
    double RootExtent;
    double SurfaceLevel;
    double SeaLevel;
    /** Memory-stable storage for primitives */
    TIndirectArray<FVoxelCorner> CornerStore;
    TIndirectArray<FVoxelEdge> EdgeStore;
    TIndirectArray<FVoxelFace> FaceStore;

    /** Fast lookups for existing primitives */
    TMap<FInt64Vector, FVoxelCorner*> CornerLookup;
    TMap<FVoxelEdgeKey, FVoxelEdge*> EdgeLookup;
    TMap<FVoxelFaceKey, FVoxelFace*> FaceLookup;

    /** Free lists for recycling memory in planetary LOD shifting */
    TArray<int32> FreeCornerIndices;
    TArray<int32> FreeEdgeIndices;
    TArray<int32> FreeFaceIndices;

    /** * Internal Factory Methods:
     * These assume the caller is already holding the appropriate WriteLock.
     */
    FVoxelCorner* Internal_CreateCorner(const FInt64Vector& InKey);
    FVoxelEdge* Internal_CreateEdge(const FVoxelEdgeKey& InKey, FVoxelEdge* InParent);
    FVoxelFace* Internal_CreateFace(FVoxelEdge* InEdges[4], int32 InAxis, FVoxelFace* InParent, TSharedPtr<FAdaptiveOctreeNode> InNode);

public:
    FNodeStructureProvider(TSharedPtr<FSparseEditStore> InEditStore, TFunction<void(int32 Count, const float* X, const float* Y, const float* Z, float* OutDensities)> InDensityFunction, double InRootExtent, double InSeaLevel, double InSurfaceLevel, FVector InActorCenter);
    ~FNodeStructureProvider();

    /** SIMD Density Function: Handed in from the Planet Actor or Volume */
    TFunction<void(int32 Count, const float* X, const float* Y, const float* Z, float* OutDensities)> NoiseFunction;

    /** High-level batch population for Octree Splits */
    void PopulateNodeStructure(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes);
    void UpdateNodeStructure(const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InNodes);

    /** Apply a spherical terrain edit and resample all affected nodes */
    void ApplyEdit(FVector InCenter, double InRadius, double InStrength, int InResolution, const TArray<TSharedPtr<FAdaptiveOctreeNode>>& InAffectedNodes);

    /** Thread-safe Acquisition API */
    FVoxelCorner* GetOrCreateCorner(const FVector& InPosition);
    FVoxelEdge* GetOrCreateEdge(FVoxelCorner* InC1, FVoxelCorner* InC2, FVoxelEdge* InParent);
    FVoxelFace* GetOrCreateFace(FVoxelEdge* InEdges[4], int32 InAxis, FVoxelFace* InFace, TSharedPtr<FAdaptiveOctreeNode> InNode);

    FORCEINLINE double CompositeSample(const FVector& P, double RawNoise) const
    {
        double dx = P.X - Center.X;
        double dy = P.Y - Center.Y;
        double dz = P.Z - Center.Z;
        double Dist = FMath::Sqrt(dx * dx + dy * dy + dz * dz);
        return (Dist - SurfaceLevel) - RawNoise + EditStore->Sample(P);
    }

    /** * Cleanup: Optional method to prune the TChunkedArrays
     * using the FreeLists when RefCounts hit zero.
     */
    void PruneUnusedStructures();
};