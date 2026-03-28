// FOceanSharedStructs.h — Cube-sphere topology types shared between
// FOceanQuadTreeNode and AOceanSphereActor: face/edge enums, cube-face
// transform tables, and the FQuadIndex path encoding for quadtree traversal.

#pragma once

#include "CoreMinimal.h"
#include "FOceanSharedStructs.generated.h"

/** Edge direction within a quadtree face (2D local space). */
UENUM(BlueprintType)
enum class EdgeOrientation : uint8
{
    LEFT = 0,
    RIGHT = 1,
    UP = 2,
    DOWN = 3
};

/** Identifies one of the 6 cube faces that tile the sphere. */
UENUM(BlueprintType)
enum class EFaceDirection : uint8
{
    X_POS = 0  UMETA(DisplayName = "X+"),
    X_NEG = 1  UMETA(DisplayName = "X-"),
    Y_POS = 2  UMETA(DisplayName = "Y+"),
    Y_NEG = 3  UMETA(DisplayName = "Y-"),
    Z_POS = 4  UMETA(DisplayName = "Z+"),
    Z_NEG = 5  UMETA(DisplayName = "Z-")
};

/** Quadrant position of a child within its parent quad.
 *  Bit 0 = Y (0=bottom, 1=top), bit 1 = X (0=left, 1=right). */
enum VOXELPLUGIN_API EChildPosition
{
    BOTTOM_LEFT = 0,   // 0b00
    TOP_LEFT = 1,   // 0b01
    BOTTOM_RIGHT = 2,   // 0b10
    TOP_RIGHT = 3    // 0b11
};

/** Describes how a quadtree path maps when crossing from one cube face to
 *  an adjacent face. QuadrantRemap permutes the 4 child indices; FlipX/FlipY
 *  mirror coordinates to maintain consistent winding across the seam. */
struct VOXELPLUGIN_API FaceTransition
{
    uint8 TargetFace;
    uint8 QuadrantRemap[4];
    bool  bFlipX = false;
    bool  bFlipY = false;
};

/** Per-face transform that maps 2D quadtree coordinates to 3D cube/world space.
 *  AxisMap selects which world axes correspond to the face's local U, V, and normal.
 *  AxisDir flips axes as needed. NeighborEdgeMap and FaceTransitions define the
 *  topology for cross-face neighbor lookups. bFlipWinding controls triangle winding
 *  so outward-facing normals are consistent across all 6 faces.
 *
 *  The static FaceTransforms[6] table is defined in FOceanSharedStructs.cpp. */
struct VOXELPLUGIN_API FCubeTransform
{
    FIntVector3      AxisMap;           // [0]=U axis, [1]=V axis, [2]=normal axis in world space
    FIntVector3      AxisDir;           // Sign (+1/-1) for each mapped axis
    EdgeOrientation  NeighborEdgeMap[4]; // Which edge the neighbor sees when looking back
    FaceTransition   FaceTransitions[4]; // Cross-face transition for LEFT, RIGHT, UP, DOWN
    bool             bFlipWinding;      // True if this face needs reversed triangle winding

    static const FCubeTransform FaceTransforms[6];
};

/** Encodes a path through the quadtree as a packed uint64.
 *
 *  Layout: sentinel bits 0b11 sit at the bottom; each depth level prepends 2 bits
 *  (child quadrant index 0-3). Supports up to depth 31 (62 path bits + 2 sentinel).
 *  Combined with a FaceId to uniquely identify any node across all 6 cube faces.
 *
 *  Provides navigation: GetChildIndex, GetParentIndex, GetNeighborIndex (including
 *  cross-face transitions via FCubeTransform::FaceTransitions). */
struct VOXELPLUGIN_API FQuadIndex
{
    uint64 EncodedPath;
    uint8  FaceId;
    static constexpr uint64 SentinelBits = 0b11ULL;

    explicit FQuadIndex(uint8 InFaceId)
        : EncodedPath(SentinelBits), FaceId(InFaceId) {}

    FQuadIndex(uint64 InPath, uint8 InFaceId)
        : EncodedPath(InPath), FaceId(InFaceId) {}

    /** Returns the depth by counting 2-bit groups above the sentinel. */
    uint8 GetDepth() const
    {
        uint64 path = EncodedPath >> 2;
        uint8  depth = 0;
        while (path != 0) { depth++; path >>= 2; }
        return depth;
    }

    bool IsRoot() const { return GetDepth() == 0; }

    /** Returns the quadrant index (0-3) at the given level of the path. */
    uint8 GetQuadrantAtDepth(uint8 Level) const
    {
        uint8 depth = GetDepth();
        if (Level >= depth) return 0;
        uint8 shiftAmount = (depth - Level - 1) * 2;
        return (EncodedPath >> shiftAmount) & 0x3;
    }

    /** Returns the quadrant of this node within its parent. */
    uint8 GetQuadrant() const
    {
        if (IsRoot()) return 0;
        return GetQuadrantAtDepth(GetDepth() - 1);
    }

    /** Returns the index of a child at the given quadrant. */
    FQuadIndex GetChildIndex(uint8 InChildIndex) const
    {
        if (GetDepth() >= 31) return *this;
        uint64 newPath = (EncodedPath << 2) | (InChildIndex & 0x3);
        return FQuadIndex(newPath, FaceId);
    }

    /** Returns the index of this node's parent. */
    FQuadIndex GetParentIndex() const
    {
        if (IsRoot()) return *this;
        return FQuadIndex(EncodedPath >> 2, FaceId);
    }

    uint64 GetQuadrantPath()                       const { return EncodedPath >> 2; }
    uint64 MakeEncodedPath(uint64 QuadrantPath)    const { return (QuadrantPath << 2) | SentinelBits; }

    /** Mirrors a quadrant across the given edge (flips the X or Y bit). */
    uint8 ReflectQuadrant(uint8 quadrant, EdgeOrientation Direction) const
    {
        switch (Direction)
        {
        case EdgeOrientation::LEFT:
        case EdgeOrientation::RIGHT: return quadrant ^ 0x2;
        case EdgeOrientation::UP:
        case EdgeOrientation::DOWN:  return quadrant ^ 0x1;
        default:                     return quadrant;
        }
    }

    /** Applies X/Y flip to a quadrant index for cross-face remapping. */
    uint8 ApplyFlip(uint8 quadrant, bool FlipX, bool FlipY) const
    {
        if (FlipX) quadrant ^= 0x2;
        if (FlipY) quadrant ^= 0x1;
        return quadrant;
    }

    /** Finds the neighbor node in the given direction. Walks up the tree until
     *  finding a non-edge ancestor, reflects the quadrant, then walks back down.
     *  If the edge is at the cube face boundary, applies the cross-face transition
     *  from FCubeTransform::FaceTransitions to remap the entire path onto the
     *  adjacent face. */
    FQuadIndex GetNeighborIndex(EdgeOrientation Direction) const
    {
        bool isFaceEdge = false;

        FQuadIndex current = *this;
        while (!current.IsRoot())
        {
            uint8 quadrant = current.GetQuadrant();
            bool  quadAtEdge = false;

            switch (Direction)
            {
            case EdgeOrientation::LEFT:  quadAtEdge = (quadrant & 0x2) == 0; break;
            case EdgeOrientation::RIGHT: quadAtEdge = (quadrant & 0x2) != 0; break;
            case EdgeOrientation::UP:    quadAtEdge = (quadrant & 0x1) == 0; break;
            case EdgeOrientation::DOWN:  quadAtEdge = (quadrant & 0x1) != 0; break;
            }

            if (!quadAtEdge) { isFaceEdge = false; break; }

            current = current.GetParentIndex();
            isFaceEdge = true;
        }

        if (isFaceEdge || IsRoot())
        {
            TArray<uint8> path;
            current = *this;
            while (!current.IsRoot())
            {
                path.Insert(current.GetQuadrant(), 0);
                current = current.GetParentIndex();
            }

            const FaceTransition& transition =
                FCubeTransform::FaceTransforms[FaceId].FaceTransitions[(uint8)Direction];

            TArray<uint8> remappedPath;
            for (uint8 q : path)
                remappedPath.Add(ApplyFlip(transition.QuadrantRemap[q], transition.bFlipX, transition.bFlipY));

            FQuadIndex result(transition.TargetFace);
            for (uint8 rq : remappedPath)
                result = result.GetChildIndex(rq);

            return result;
        }

        uint8 quadrant = GetQuadrant();
        bool  atEdge = false;
        switch (Direction)
        {
        case EdgeOrientation::LEFT:  atEdge = (quadrant & 0x2) == 0; break;
        case EdgeOrientation::RIGHT: atEdge = (quadrant & 0x2) != 0; break;
        case EdgeOrientation::UP:    atEdge = (quadrant & 0x1) == 0; break;
        case EdgeOrientation::DOWN:  atEdge = (quadrant & 0x1) != 0; break;
        }

        if (!atEdge)
        {
            uint64 neighborPath = EncodedPath;
            neighborPath ^= (Direction == EdgeOrientation::LEFT ||
                Direction == EdgeOrientation::RIGHT) ? 0x2ULL : 0x1ULL;
            return FQuadIndex(neighborPath, FaceId);
        }

        FQuadIndex parent = GetParentIndex();
        FQuadIndex neighborParent = parent.GetNeighborIndex(Direction);
        uint8      reflected = ReflectQuadrant(quadrant, Direction);
        return neighborParent.GetChildIndex(reflected);
    }

    bool operator==(const FQuadIndex& Other) const
    {
        return FaceId == Other.FaceId && EncodedPath == Other.EncodedPath;
    }

    friend uint32 GetTypeHash(const FQuadIndex& ID)
    {
        return HashCombine(GetTypeHash(ID.FaceId),
            GetTypeHash(uint32(ID.EncodedPath ^ (ID.EncodedPath >> 32))));
    }

    bool IsValid() const { return EncodedPath != 0; }

    FString ToString() const
    {
        FString Result = FString::Printf(TEXT("Face: %d, Depth: %d, Path: "), FaceId, GetDepth());
        for (int32 i = 0; i < GetDepth(); i++)
            Result += FString::Printf(TEXT("%d"), GetQuadrantAtDepth(i));
        Result += FString::Printf(TEXT(" (Encoded: 0x%llX)"), EncodedPath);
        return Result;
    }
};