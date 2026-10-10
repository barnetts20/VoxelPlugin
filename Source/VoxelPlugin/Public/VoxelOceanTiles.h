// VoxelOceanTiles.h - The ocean's cube-sphere quadtree (Design/OceanMeshing.md section 2): tile
// keys, the equal-angle face mapping (from VoxelCubeSphere.h), the CDLOD selection, and a
// tile's vertices with their morph targets. Pure functions of their inputs, in double; any
// thread.
//
// A face has an outward normal N and axes U, V with U x V = N. Face coordinates s (along U) and
// t (along V) run over [-1, 1] and map to normalize(N + tan(pi s / 4) U + tan(pi t / 4) V): the
// equal-angle mapping, whose cells vary 1.41x in edge length across a face (2.1x for the plain
// cube). A level-L tile (I, J) covers s in [-1 + 2I / 2^L, -1 + 2(I + 1) / 2^L], t likewise. Its
// vertex (a, b) lies on grid line I * Q + a (and J * Q + b) of level L + log2(Q), Q being the
// quads per tile edge, so a vertex shared by two tiles, or by a tile and its parent, is computed
// from the same exact face coordinates.
//
// Selection: a node of level L splits while the camera is nearer than F * Arc(L) * R to it
// (F the LOD distance factor, Arc(L) the nominal tile arc, R the sea radius), Hysteresis times
// that if the previous selection split it. Edge-adjacent tiles then differ by at most one level
// (the harness finds violations at F = 0.6 and none from 1).
//
// Morph (CDLOD collapse, as the terrain's vertices collapse into their parents): every odd
// vertex of a tile slides along the grid onto an even neighbour, a vertex of its parent
// (CollapseOffset). Fully collapsed, a tile is exactly its parent's triangles plus degenerate
// ones (every quad is split along the same diagonal (a, b) - (a + 1, b + 1), the way odd
// vertices slide), without T-junctions.
//
// The targets are chained, as the terrain's (VoxelMorph.ush): a level-L vertex blends, with
// weight W(L) at its own position, toward what level L - 1 draws there at that moment: its
// collapse target, itself blended toward its own target (the grandparent grid) with W(L - 1)
// at the target's position. W(L) runs from 0 at the start fraction (voxel.Ocean.MorphStart) of MorphEnd(L) to 1 at
// MorphEnd(L) = MorphMargin * 2 * F * Arc(L) * R from the camera the selection was built for,
// the nearest a coarser unsplit tile can be; with the start at one half the levels' bands are
// contiguous, so the mesh morphs continuously with distance. On a boundary with a coarser tile
// the finer side is fully collapsed onto the coarser tile's vertices as that tile draws them,
// so the boundary is closed at any weights and for any displacement sampled at the morphed
// points (waves). Two levels of chain suffice while every level-L vertex is nearer than where
// level L - 2 starts to morph (2 MorphEnd(L)); F of at least MinLodDistanceFactor keeps it so
// (the harness checks).

#pragma once

#include "CoreMinimal.h"
#include "VoxelCubeSphere.h"

/** A tile of the ocean quadtree; also a page of tiles (VoxelOceanTiles::PageOf). */
struct FVoxelOceanTileKey
{
    int32 I = 0;       // along the face's U axis, 0 .. 2^Level - 1
    int32 J = 0;       // along V
    uint8 Face = 0;
    uint8 Level = 0;

    FVoxelOceanTileKey() = default;
    FVoxelOceanTileKey(int32 InFace, int32 InLevel, int32 InI, int32 InJ)
        : I(InI), J(InJ), Face((uint8)InFace), Level((uint8)InLevel)
    {
    }

    /** Quadrant bit 0: the upper half in I, bit 1: in J. */
    FVoxelOceanTileKey Child(int32 Quadrant) const
    {
        return FVoxelOceanTileKey(Face, Level + 1, I * 2 + (Quadrant & 1), J * 2 + ((Quadrant >> 1) & 1));
    }

    bool operator==(const FVoxelOceanTileKey& Other) const
    {
        return I == Other.I && J == Other.J && Face == Other.Face && Level == Other.Level;
    }

    bool operator!=(const FVoxelOceanTileKey& Other) const { return !(*this == Other); }

    friend uint32 GetTypeHash(const FVoxelOceanTileKey& Key)
    {
        return HashCombine(HashCombine(::GetTypeHash(Key.I), ::GetTypeHash(Key.J)), ::GetTypeHash(((uint32)Key.Face << 8) | (uint32)Key.Level));
    }
};

namespace VoxelOceanTiles
{
    /** A page holds the tiles of one level under one node PageShift levels up: 4x4 tiles. */
    constexpr int32 PageShift = 2;
    constexpr int32 PageTiles = 1 << PageShift;
    constexpr int32 PageSlots = PageTiles * PageTiles;

    /** Deepest tile level (keys hold 32-bit tile indices; grid lines need 2^(Level + log2 Q)
     *  to stay exact in double). */
    constexpr int32 MaxTileLevel = 24;

    /** The morph is complete this fraction of the way to the nearest coarser tile (float
     *  margin in the shader). */
    constexpr double MorphMargin = 0.98;

    /** Smallest LOD distance factor: below about 1.9 a tile's vertices can reach where its
     *  grandparent level morphs, beyond the two-level chain. */
    constexpr double MinLodDistanceFactor = 2.25;

    /** Where a level's morph starts by default, as a fraction of its end: one half makes the
     *  levels' bands contiguous. Valid from 0.5 (overlapping bands would need a longer chain) to
     *  below 1. */
    constexpr double DefaultMorphStartFraction = 0.5;

    // The mapping lives in VoxelCubeSphere.h, shared with the land bake.
    using VoxelCubeSphere::FFace;
    using VoxelCubeSphere::GetFace;
    using VoxelCubeSphere::Warp;
    using VoxelCubeSphere::Unwarp;
    using VoxelCubeSphere::GridCoord;
    using VoxelCubeSphere::Direction;
    using VoxelCubeSphere::Locate;

    /** Angle between two vectors (radians), accurate for small angles. */
    inline double AngleBetween(const FVector& A, const FVector& B)
    {
        return FMath::Atan2((A ^ B).Size(), A | B);
    }

    /** Arc of a level-L tile edge through the face centre, on the unit sphere. */
    inline double NominalArc(int32 Level)
    {
        return (UE_DOUBLE_PI * 0.5) / (double)(1LL << Level);
    }

    /** Centre direction and angular radius of a cap holding the tile (its corners, and so its
     *  great-circle edges). */
    inline void Cap(const FVoxelOceanTileKey& Key, FVector& OutCenter, double& OutAngle)
    {
        OutCenter = Direction(Key.Face, GridCoord(2 * (int64)Key.I + 1, Key.Level + 1), GridCoord(2 * (int64)Key.J + 1, Key.Level + 1));
        double Angle = 0.0;
        for (int32 c = 0; c < 4; c++)
        {
            const FVector Corner = Direction(Key.Face, GridCoord((int64)Key.I + (c & 1), Key.Level), GridCoord((int64)Key.J + (c >> 1), Key.Level));
            Angle = FMath::Max(Angle, AngleBetween(OutCenter, Corner));
        }
        OutAngle = Angle;
    }

    /** Distance from Point (planet frame) to a cap of the sphere of radius Radius. */
    inline double DistanceToCap(const FVector& Point, const FVector& CapCenter, double CapAngle, double Radius)
    {
        const double R0 = Point.Size();
        const double Theta = AngleBetween(Point, CapCenter);
        if (Theta <= CapAngle) return FMath::Abs(R0 - Radius);
        const double Half = FMath::Sin(0.5 * (Theta - CapAngle));
        return FMath::Sqrt(FMath::Square(R0 - Radius) + 4.0 * R0 * Radius * Half * Half);
    }

    /** False when the whole cap, raised by Height, is below the horizon seen from Point. */
    inline bool IsAboveHorizon(const FVector& Point, const FVector& CapCenter, double CapAngle, double Radius, double Height)
    {
        const double R0 = Point.Size();
        if (R0 <= Radius + Height) return true;
        const double Reach = FMath::Acos(Radius / R0) + FMath::Acos(Radius / (Radius + Height));
        return AngleBetween(Point, CapCenter) - CapAngle <= Reach;
    }

    struct FSelectParams
    {
        double Radius = 0.0;              // sea sphere (cm)
        int32 MaxLevel = 0;
        double LodDistanceFactor = 2.5;
        double Hysteresis = 1.1;          // a split node merges beyond this times its range
        double HorizonHeight = 1000.0;    // tiles are kept over the horizon up to this height (cm)
    };

    /** A node of this level splits while the camera is nearer than this (cm). */
    inline double SplitRange(const FSelectParams& Params, int32 Level)
    {
        return Params.LodDistanceFactor * NominalArc(Level) * Params.Radius;
    }

    /** Where level 0's morph ends (cm); level L's is 2^-L times it. */
    inline double MorphEnd0(double Radius, double LodDistanceFactor)
    {
        return MorphMargin * 2.0 * LodDistanceFactor * NominalArc(0) * Radius;
    }

    /** A valid morph start fraction (DefaultMorphStartFraction). */
    inline double ClampMorphStartFraction(double StartFraction)
    {
        return FMath::Clamp(StartFraction, 0.5, 0.95);
    }

    /** The morph weight of a level-L vertex Distance from the selection camera, as the
     *  material computes it (VoxelOcean.ush, in float there). */
    inline double MorphWeight(double Distance, int32 Level, double InMorphEnd0, double StartFraction)
    {
        if (Level <= 0) return 0.0;
        const double End = InMorphEnd0 / (double)(1LL << Level);
        const double Start = End * StartFraction;
        const double X = FMath::Clamp((Distance - Start) / (End - Start), 0.0, 1.0);
        return X * X * (3.0 - 2.0 * X);
    }

    /** CDLOD selection for Camera (planet frame, cm): appends the selected tiles to OutTiles and
     *  the split nodes to OutSplit. PrevSplit: the nodes the previous selection split (they
     *  merge only beyond Hysteresis times their range). */
    inline void Select(const FVector& Camera, const FSelectParams& Params, const TSet<FVoxelOceanTileKey>* PrevSplit,
        TArray<FVoxelOceanTileKey>& OutTiles, TSet<FVoxelOceanTileKey>& OutSplit)
    {
        TArray<FVoxelOceanTileKey> Stack;
        for (int32 Face = 0; Face < 6; Face++)
            Stack.Add(FVoxelOceanTileKey(Face, 0, 0, 0));
        while (Stack.Num() > 0)
        {
            const FVoxelOceanTileKey Node = Stack.Pop();
            FVector Center;
            double Angle = 0.0;
            Cap(Node, Center, Angle);
            if (!IsAboveHorizon(Camera, Center, Angle, Params.Radius, Params.HorizonHeight)) continue;

            double Range = SplitRange(Params, Node.Level);
            if (PrevSplit && PrevSplit->Contains(Node))
                Range *= Params.Hysteresis;
            if (Node.Level < Params.MaxLevel && DistanceToCap(Camera, Center, Angle, Params.Radius) < Range)
            {
                OutSplit.Add(Node);
                for (int32 Quadrant = 0; Quadrant < 4; Quadrant++)
                    Stack.Add(Node.Child(Quadrant));
            }
            else
            {
                OutTiles.Add(Node);
            }
        }
    }

    /** The page holding a tile: its level, and the tile's coordinates shifted by PageShift. */
    inline FVoxelOceanTileKey PageOf(const FVoxelOceanTileKey& Tile)
    {
        return FVoxelOceanTileKey(Tile.Face, Tile.Level, Tile.I >> PageShift, Tile.J >> PageShift);
    }

    /** A page's origin on the sphere (planet frame, cm) and its unit (cm): the centre and nominal
     *  arc of its node, PageShift levels above its tiles (the face for levels 0 and 1). */
    inline void PageFrame(const FVoxelOceanTileKey& Page, double Radius, FVector& OutOrigin, double& OutUnit)
    {
        const int32 NodeLevel = FMath::Max((int32)Page.Level - PageShift, 0);
        OutOrigin = Direction(Page.Face, GridCoord(2 * (int64)Page.I + 1, NodeLevel + 1), GridCoord(2 * (int64)Page.J + 1, NodeLevel + 1)) * Radius;
        OutUnit = NominalArc(NodeLevel) * Radius;
    }

    struct FTileVertex
    {
        FVector Point;        // on the sea sphere, planet frame (cm)
        FVector3f Position;   // from the page origin, page units
        FVector3f Morph;      // to the morph target (the parent vertex it collapses into), page units
        FVector3f Grand;      // from the morph target to the target's own (the grandparent grid), page units
        FVector3f Normal;     // radial
        FVector3f Tangent;    // the face's U axis on the tangent plane
        /** The morph target and the target's own target as vertices of this tile (BuildTile
         *  order): both lie in the tile, so values sampled per vertex (the terrain depth) are
         *  gathered for them from the tile itself. */
        int32 TargetIndex = 0;
        int32 GrandIndex = 0;
    };

    /** Which way an odd vertex on a face's edge collapses (-1 toward lower grid lines, +1
     *  toward higher): toward the edge's cube corner that is lower in (x, y, z) order. The two
     *  faces sharing the edge find the same corner, from exact integer vectors, so a vertex
     *  shared across the edge collapses to the same point from both. Edge: 0 is s = -1, 1 is
     *  s = +1 (the vertex runs along t); 2 is t = -1, 3 is t = +1 (it runs along s). */
    inline int32 FaceEdgeCollapse(int32 Face, int32 Edge)
    {
        const FFace& F = GetFace(Face);
        FVector Low, High;
        if (Edge < 2)
        {
            const FVector Fixed = F.N + F.U * (Edge == 0 ? -1.0 : 1.0);
            Low = Fixed - F.V;
            High = Fixed + F.V;
        }
        else
        {
            const FVector Fixed = F.N + F.V * (Edge == 2 ? -1.0 : 1.0);
            Low = Fixed - F.U;
            High = Fixed + F.U;
        }
        // Low is the corner at the lower grid lines; it wins unless High is lower in (x, y, z).
        if (High.X != Low.X) return High.X < Low.X ? 1 : -1;
        if (High.Y != Low.Y) return High.Y < Low.Y ? 1 : -1;
        return High.Z < Low.Z ? 1 : -1;
    }

    /** The grid offsets (each -1, 0 or +1) that take vertex (A, B) of a level-Level tile (I, J)
     *  to its collapse target on the parent's grid: (0, 0) for an even vertex. An odd vertex
     *  slides along the grid onto an even neighbour toward lower lines (the quads' shared
     *  diagonal runs the same way, so a fully collapsed tile is its parent's triangles plus
     *  degenerate ones); on a face's edge, toward FaceEdgeCollapse's corner. */
    inline void CollapseOffset(const FVoxelOceanTileKey& Tile, int32 Quads, int32 A, int32 B, int32& OutDA, int32& OutDB)
    {
        OutDA = 0;
        OutDB = 0;
        if (Tile.Level == 0) return;
        const bool bOddA = (A & 1) != 0;
        const bool bOddB = (B & 1) != 0;
        if (!bOddA && !bOddB) return;
        const int64 Lines = (int64)Quads << Tile.Level;
        const int64 GA = (int64)Tile.I * Quads + A;
        const int64 GB = (int64)Tile.J * Quads + B;
        if (bOddA && bOddB)
        {
            OutDA = -1;
            OutDB = -1;
        }
        else if (bOddA)
        {
            // Along s; on the face's edge t = -1 or t = +1 the corner decides.
            OutDA = GB == 0 ? FaceEdgeCollapse(Tile.Face, 2) : (GB == Lines ? FaceEdgeCollapse(Tile.Face, 3) : -1);
        }
        else
        {
            OutDB = GA == 0 ? FaceEdgeCollapse(Tile.Face, 0) : (GA == Lines ? FaceEdgeCollapse(Tile.Face, 1) : -1);
        }
    }

    /** The (Quads + 1)^2 vertices of a tile, row by row along U (index b * (Quads + 1) + a). */
    inline void BuildTile(const FVoxelOceanTileKey& Tile, int32 Quads, double Radius, TArray<FTileVertex>& Out)
    {
        const int32 N = Quads + 1;
        const int32 Bits = (int32)Tile.Level + FMath::FloorLog2((uint32)Quads);
        FVector Origin;
        double Unit = 1.0;
        PageFrame(PageOf(Tile), Radius, Origin, Unit);
        const double InvUnit = 1.0 / Unit;

        TArray<FVector> Points;
        Points.Reserve(N * N);
        for (int32 b = 0; b < N; b++)
        {
            for (int32 a = 0; a < N; a++)
            {
                Points.Add(Direction(Tile.Face, GridCoord((int64)Tile.I * Quads + a, Bits), GridCoord((int64)Tile.J * Quads + b, Bits)) * Radius);
            }
        }
        auto At = [&Points, N](int32 A, int32 B) -> const FVector& { return Points[B * N + A]; };

        const FFace& F = GetFace(Tile.Face);
        Out.Reset(N * N);
        for (int32 b = 0; b < N; b++)
        {
            for (int32 a = 0; a < N; a++)
            {
                const FVector& P = At(a, b);
                // The parent's grid: even lines are its vertices; an odd vertex slides onto
                // one of them (CollapseOffset). Level 0 has no parent.
                int32 DA = 0, DB = 0;
                CollapseOffset(Tile, Quads, a, b, DA, DB);
                const FVector& Target = At(a + DA, b + DB);
                // The target as a vertex of the parent tile, and where it collapses in turn. A
                // tile's edge lines are even in its parent's grid too (Quads / 2 is even), so the
                // grand target stays in the tile.
                FVector Grand = Target;
                int32 GrandA = a + DA, GrandB = b + DB;
                if (Tile.Level >= 2)
                {
                    const FVoxelOceanTileKey Parent(Tile.Face, Tile.Level - 1, Tile.I >> 1, Tile.J >> 1);
                    const int32 PA = ((Tile.I & 1) * Quads + a + DA) / 2;
                    const int32 PB = ((Tile.J & 1) * Quads + b + DB) / 2;
                    int32 GA = 0, GB = 0;
                    CollapseOffset(Parent, Quads, PA, PB, GA, GB);
                    if (GA != 0 || GB != 0)
                    {
                        Grand = Direction(Tile.Face, GridCoord((int64)Parent.I * Quads + PA + GA, Bits - 1),
                            GridCoord((int64)Parent.J * Quads + PB + GB, Bits - 1)) * Radius;
                        GrandA = FMath::Clamp(2 * (PA + GA) - (Tile.I & 1) * Quads, 0, Quads);
                        GrandB = FMath::Clamp(2 * (PB + GB) - (Tile.J & 1) * Quads, 0, Quads);
                    }
                }
                const FVector D = P / Radius;
                FTileVertex Vertex;
                Vertex.Point = P;
                Vertex.TargetIndex = (b + DB) * N + (a + DA);
                Vertex.GrandIndex = GrandB * N + GrandA;
                Vertex.Position = FVector3f((P - Origin) * InvUnit);
                Vertex.Morph = FVector3f((Target - P) * InvUnit);
                Vertex.Grand = FVector3f((Grand - Target) * InvUnit);
                Vertex.Normal = FVector3f(D);
                Vertex.Tangent = FVector3f((F.U - D * (F.U | D)).GetSafeNormal());
                Out.Add(Vertex);
            }
        }
    }

    /** The longest edge between neighbouring vertices of a tile BuildTile made (cm). */
    inline double MaxSpacing(TConstArrayView<FTileVertex> Vertices, int32 Quads)
    {
        const int32 N = Quads + 1;
        double Max = 0.0;
        for (int32 b = 0; b < N; b++)
        {
            for (int32 a = 0; a < N; a++)
            {
                const FVector& P = Vertices[b * N + a].Point;
                if (a + 1 < N) Max = FMath::Max(Max, FVector::DistSquared(P, Vertices[b * N + a + 1].Point));
                if (b + 1 < N) Max = FMath::Max(Max, FVector::DistSquared(P, Vertices[(b + 1) * N + a].Point));
            }
        }
        return FMath::Sqrt(Max);
    }

    /** The two triangles of quad (a, b) of a tile, as vertex indices of BuildTile's order, wound
     *  so UE draws the outward side: (B - A) x (C - A) points into the sphere, as the old
     *  ocean's tables did. bFlip reverses them (voxel.Ocean.FlipWinding). */
    inline void QuadTriangles(int32 Quads, int32 A, int32 B, bool bFlip, int32 (&Out)[6])
    {
        const int32 N = Quads + 1;
        const int32 V00 = B * N + A;
        const int32 V10 = V00 + 1;
        const int32 V01 = V00 + N;
        const int32 V11 = V01 + 1;
        Out[0] = V00; Out[1] = bFlip ? V10 : V11; Out[2] = bFlip ? V11 : V10;
        Out[3] = V00; Out[4] = bFlip ? V11 : V01; Out[5] = bFlip ? V01 : V11;
    }

    /** The finest level for a spacing between vertices (cm). */
    inline int32 LevelForSpacing(double Radius, int32 Quads, double Spacing)
    {
        const double Ratio = NominalArc(0) * Radius / (FMath::Max(Spacing, 1.0) * FMath::Max(Quads, 1));
        return FMath::Clamp((int32)FMath::CeilToInt(FMath::Log2(FMath::Max(Ratio, 1.0))), 0, MaxTileLevel);
    }
}
