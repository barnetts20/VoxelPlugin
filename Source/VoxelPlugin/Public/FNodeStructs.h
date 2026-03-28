// FNodeStructs.h — Core data types for octree node geometry: corners, edges, 
// morton indices, and edge neighbor tracking.

#pragma once

#include "CoreMinimal.h"

struct FAdaptiveOctreeNode;

/** A single corner of an octree node, storing its world position,
 *  SDF density value, and surface normal. Each node has 8 corners
 *  indexed per OctreeConstants::Offsets (bit 0 = X, bit 1 = Y, bit 2 = Z). */
struct VOXELPLUGIN_API FNodeCorner {
    FVector Position;
    float Density;
    FVector3f Normal;

    FNodeCorner() : Position(0), Density(0), Normal(0, 0, 1) {}
    FNodeCorner(FVector InPos, float InDensity, FVector3f InNormal) : Position(InPos), Density(InDensity), Normal(InNormal) {}
};

/** Quantized, canonically-ordered key that uniquely identifies an edge in the octree.
 *  Two endpoints are quantized to integer grid coordinates and stored in a consistent
 *  order (lexicographic on (X,Y,Z)) so that the same edge discovered from different
 *  neighboring nodes produces an identical key. The hash is computed once at construction
 *  and cached for fast TMap lookups during edge deduplication. */
struct VOXELPLUGIN_API FEdgeKey
{
    int32 X0, Y0, Z0;
    int32 X1, Y1, Z1;
    uint8 Axis;
    uint32 CachedHash;

    // Set once by FAdaptiveOctree at construction.
    // Value: 2^MaxDepth / RootExtent - maps the smallest possible edge to ~1 grid unit.
    static inline double InvGridSize = 1.0;

    /** Snaps a world-space coordinate to the integer grid defined by InvGridSize. */
    static int32 Quantize(double V)
    {
        return FMath::RoundToInt32(V * InvGridSize);
    }

    FEdgeKey() : X0(0), Y0(0), Z0(0), X1(0), Y1(0), Z1(0), Axis(0), CachedHash(0) {}

    /** Quantizes both endpoint positions, stores them in canonical (lexicographic) order,
     *  and precomputes the hash. */
    void BuildFromCorners(const FVector& PosA, const FVector& PosB, uint8 InAxis)
    {
        int32 ax = Quantize(PosA.X);
        int32 ay = Quantize(PosA.Y);
        int32 az = Quantize(PosA.Z);
        int32 bx = Quantize(PosB.X);
        int32 by = Quantize(PosB.Y);
        int32 bz = Quantize(PosB.Z);

        if (ax < bx || (ax == bx && ay < by) || (ax == bx && ay == by && az < bz))
        {
            X0 = ax; Y0 = ay; Z0 = az;
            X1 = bx; Y1 = by; Z1 = bz;
        }
        else
        {
            X0 = bx; Y0 = by; Z0 = bz;
            X1 = ax; Y1 = ay; Z1 = az;
        }

        Axis = InAxis;

        uint32 Hash = ::GetTypeHash(X0);
        Hash = HashCombine(Hash, ::GetTypeHash(Y0));
        Hash = HashCombine(Hash, ::GetTypeHash(Z0));
        Hash = HashCombine(Hash, ::GetTypeHash(X1));
        Hash = HashCombine(Hash, ::GetTypeHash(Y1));
        Hash = HashCombine(Hash, ::GetTypeHash(Z1));
        Hash = HashCombine(Hash, ::GetTypeHash((int32)Axis));
        CachedHash = Hash;
    }

    bool operator==(const FEdgeKey& Other) const
    {
        return X0 == Other.X0 && Y0 == Other.Y0 && Z0 == Other.Z0
            && X1 == Other.X1 && Y1 == Other.Y1 && Z1 == Other.Z1
            && Axis == Other.Axis;
    }
};

/** An axis-aligned edge between two node corners, carrying the data needed for
 *  dual contouring: whether a sign change (surface crossing) exists, the interpolated
 *  zero-crossing point along the edge, and a prebuilt FEdgeKey for deduplication.
 *  Edges are always axis-aligned since they connect corners differing in exactly one bit. */
struct VOXELPLUGIN_API FNodeEdge
{
    FNodeCorner Corners[2];
    double Size;            // Distance between the two corner positions.
    bool SignChange;        // True if the density changes sign across this edge (surface crossing).
    uint8 Axis;             // 0 = X, 1 = Y, 2 = Z.
    FVector ZeroCrossingPoint;  // Linearly interpolated surface crossing position, or midpoint if no sign change.
    FEdgeKey CachedKey;

    FNodeEdge() : Size(0), SignChange(false), Axis(0) {}

    /** Constructs the edge from two corners: detects sign change, determines axis,
     *  computes the zero-crossing point via linear interpolation of densities,
     *  and builds the cached edge key. */
    FNodeEdge(const FNodeCorner& InCorner1, const FNodeCorner& InCorner2)
    {
        Corners[0] = InCorner1;
        Corners[1] = InCorner2;

        double d1 = InCorner1.Density;
        double d2 = InCorner2.Density;

        SignChange = (d1 < 0) != (d2 < 0);
        Size = FVector::Dist(InCorner1.Position, InCorner2.Position);

        // Determine which axis this edge is aligned to from the dominant delta component.
        FVector Delta = (InCorner2.Position - InCorner1.Position).GetAbs();
        if (Delta.X > Delta.Y && Delta.X > Delta.Z)
            Axis = 0;
        else if (Delta.Y > Delta.X && Delta.Y > Delta.Z)
            Axis = 1;
        else
            Axis = 2;

        if (SignChange) {
            double Denominator = d1 - d2;
            if (FMath::Abs(Denominator) < 1e-12) {
                ZeroCrossingPoint = (InCorner1.Position + InCorner2.Position) * 0.5;
            }
            else {
                double t = d1 / Denominator;
                t = FMath::Clamp(t, 0.0, 1.0);
                ZeroCrossingPoint = InCorner1.Position + t * (InCorner2.Position - InCorner1.Position);
            }
        }
        else {
            ZeroCrossingPoint = (InCorner1.Position + InCorner2.Position) * 0.5;
        }

        CachedKey.BuildFromCorners(InCorner1.Position, InCorner2.Position, Axis);
    }
};

/** 128-bit morton index encoding a path from root to a node in the octree.
 *  Each level stores a 3-bit child index (0-7), supporting up to depth 42
 *  (3 bits * 42 = 126 bits across two uint64s). Lo holds depths 0-20,
 *  Hi holds depths 21-41. Both halves are always compared and hashed so
 *  there is no branching in the hot path; Hi is simply 0 for shallow trees.
 *  Used as the key for the FSparseEditStore path lookups. */
struct VOXELPLUGIN_API FMortonIndex {
    uint64 Lo = 0;
    uint64 Hi = 0;
    uint8 Depth = 0;

    /** Appends a child index (0-7) to the path, advancing depth by one level. */
    void PushChild(uint8 ChildIndex) {
        if (Depth < 21)
            Lo |= (uint64(ChildIndex & 0x7) << (Depth * 3));
        else
            Hi |= (uint64(ChildIndex & 0x7) << ((Depth - 21) * 3));
        Depth++;
    }

    /** Returns the child index at the given level of the path. */
    uint8 GetChildAtLevel(uint8 Level) const {
        if (Level < 21)
            return (Lo >> (Level * 3)) & 0x7;
        else
            return (Hi >> ((Level - 21) * 3)) & 0x7;
    }

    /** Returns the child index at the deepest level of this path. */
    uint8 LastChild() const {
        return GetChildAtLevel(Depth - 1);
    }

    bool operator==(const FMortonIndex& Other) const {
        return Lo == Other.Lo && Hi == Other.Hi && Depth == Other.Depth;
    }

    friend uint32 GetTypeHash(const FMortonIndex& Index) {
        uint32 Hash = ::GetTypeHash(Index.Lo);
        Hash = HashCombine(Hash, ::GetTypeHash(Index.Hi));
        Hash = HashCombine(Hash, ::GetTypeHash(Index.Depth));
        return Hash;
    }
};

/** Fixed-capacity set of up to 4 leaf nodes sharing an edge.
 *  In a regular octree, at most 4 leaf nodes can border a single edge.
 *  Used during dual contouring to find the nodes whose QEF vertices
 *  form a quad (or degenerate tri) across a sign-change edge. */
struct FEdgeNeighbors {
    FAdaptiveOctreeNode* Nodes[4] = { nullptr, nullptr, nullptr, nullptr };
    int32 Count = 0;

    void AddUnique(FAdaptiveOctreeNode* Node)
    {
        if (!Node) return;
        for (int32 i = 0; i < Count; i++)
        {
            if (Nodes[i] == Node) return;
        }
        if (Count < 4)
        {
            Nodes[Count++] = Node;
        }
    }
};

FORCEINLINE uint32 GetTypeHash(const FEdgeKey& Key)
{
    return Key.CachedHash;
}