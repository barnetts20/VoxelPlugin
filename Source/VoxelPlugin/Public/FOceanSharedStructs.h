#pragma once

#include "CoreMinimal.h"
#include <RealtimeMeshCore.h>
#include <RealtimeMeshSimple.h>
#include "FOceanSharedStructs.generated.h"

using namespace RealtimeMesh;
// ---------------------------------------------------------------------------
// FOceanSharedStructs.h
//
// Cube-sphere topology types shared between FOceanQuadTreeNode and
// AOceanSphereActor.  Extracted from PlanetSharedStructs; all planet-
// specific noise / parameter structs have been removed.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Edge / face orientation enums
// ---------------------------------------------------------------------------

UENUM(BlueprintType)
enum class EdgeOrientation : uint8
{
    LEFT = 0,
    RIGHT = 1,
    UP = 2,
    DOWN = 3
};

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

enum VOXELPLUGIN_API EChildPosition
{
    BOTTOM_LEFT = 0,   // 0b00
    TOP_LEFT = 1,   // 0b01
    BOTTOM_RIGHT = 2,   // 0b10
    TOP_RIGHT = 3    // 0b11
};

// ---------------------------------------------------------------------------
// Face topology
// ---------------------------------------------------------------------------

struct VOXELPLUGIN_API FaceTransition
{
    uint8 TargetFace;
    uint8 QuadrantRemap[4];
    bool  bFlipX = false;
    bool  bFlipY = false;
};

struct VOXELPLUGIN_API FCubeTransform
{
    FIntVector3      AxisMap;
    FIntVector3      AxisDir;
    EdgeOrientation  NeighborEdgeMap[4];
    FaceTransition   FaceTransitions[4];
    bool             bFlipWinding;

    static const FCubeTransform FaceTransforms[6];
};

// ---------------------------------------------------------------------------
// FQuadIndex
//
// Encodes a path through the quadtree as a packed uint64.
// Sentinel bits 0b11 sit at the bottom; each depth level appends 2 bits.
// Supports up to depth 31 (62 path bits + 2 sentinel bits = 64 bits).
// ---------------------------------------------------------------------------

struct VOXELPLUGIN_API FQuadIndex
{
    uint64 EncodedPath;
    uint8  FaceId;
    static constexpr uint64 SentinelBits = 0b11ULL;

    explicit FQuadIndex(uint8 InFaceId)
        : EncodedPath(SentinelBits), FaceId(InFaceId) {}

    FQuadIndex(uint64 InPath, uint8 InFaceId)
        : EncodedPath(InPath), FaceId(InFaceId) {}

    uint8 GetDepth() const
    {
        uint64 path = EncodedPath >> 2;
        uint8  depth = 0;
        while (path != 0) { depth++; path >>= 2; }
        return depth;
    }

    bool IsRoot() const { return GetDepth() == 0; }

    uint8 GetQuadrantAtDepth(uint8 Level) const
    {
        uint8 depth = GetDepth();
        if (Level >= depth) return 0;
        uint8 shiftAmount = (depth - Level - 1) * 2;
        return (EncodedPath >> shiftAmount) & 0x3;
    }

    uint8 GetQuadrant() const
    {
        if (IsRoot()) return 0;
        return GetQuadrantAtDepth(GetDepth() - 1);
    }

    FQuadIndex GetChildIndex(uint8 InChildIndex) const
    {
        if (GetDepth() >= 31) return *this;
        uint64 newPath = (EncodedPath << 2) | (InChildIndex & 0x3);
        return FQuadIndex(newPath, FaceId);
    }

    FQuadIndex GetParentIndex() const
    {
        if (IsRoot()) return *this;
        return FQuadIndex(EncodedPath >> 2, FaceId);
    }

    uint64 GetQuadrantPath()                       const { return EncodedPath >> 2; }
    uint64 MakeEncodedPath(uint64 QuadrantPath)    const { return (QuadrantPath << 2) | SentinelBits; }

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

    uint8 ApplyFlip(uint8 quadrant, bool FlipX, bool FlipY) const
    {
        if (FlipX) quadrant ^= 0x2;
        if (FlipY) quadrant ^= 0x1;
        return quadrant;
    }

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