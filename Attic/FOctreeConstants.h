// FOctreeConstants.h - Static lookup tables for octree child, edge, face, and grid topology.

#pragma once

#include "CoreMinimal.h"

/**
 * Static lookup tables encoding the spatial topology of an octree node.
 *
 * Corner index convention: bit 0 = +X, bit 1 = +Y, bit 2 = +Z.
 *   Corner 0 = (-X, -Y, -Z), Corner 7 = (+X, +Y, +Z).
 * Children follow the same bit convention for their position within the parent.
 *
 * The 27-point subdivision grid extends the 8 corners with edge midpoints (12),
 * face centers (6), and the body center (1). This grid is used to propagate
 * density and position data from parent to children without resampling the
 * density field, enabling O(1) child initialization during octree splits.
 */
struct VOXELPLUGIN_API OctreeConstants {

    /** Unit axis-aligned direction vectors for the 6 cube faces.
     *  Order: +X, -X, +Y, -Y, +Z, -Z.
     *  Used to locate face-adjacent neighbor chunks at the same depth. */
    static inline const FVector Directions[6] =
    {
        FVector(1, 0, 0),
        FVector(-1, 0, 0),
        FVector(0, 1, 0),
        FVector(0, -1, 0),
        FVector(0, 0, 1),
        FVector(0, 0, -1)
    };

    /** Sign offsets for the 8 corners/children within a node.
     *  Multiply by the node's half-extent and add to center to get corner
     *  world positions, or child center positions at the next depth.
     *  Index matches the bit convention: bit 0 = X, bit 1 = Y, bit 2 = Z. */
    static inline const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1, 1, -1),  FVector(1, 1, -1),
        FVector(-1, -1, 1),  FVector(1, -1, 1),
        FVector(-1, 1, 1),   FVector(1, 1, 1)
    };

    /** The 12 edges of a cube as pairs of corner indices.
     *  Rows 0-3: X-aligned edges (corners differ in bit 0).
     *  Rows 4-7: Y-aligned edges (corners differ in bit 1).
     *  Rows 8-11: Z-aligned edges (corners differ in bit 2).
     *  Used to construct FNodeEdge structs for sign-change detection and dual contouring. */
    static inline const int EdgePairs[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7},
        {0, 2}, {1, 3}, {4, 6}, {5, 7},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    /** Maps each child's 8 local corners to their source in the parent's 27-point grid.
     *  ChildCornerSources[childIdx][localCorner] = grid index (0-26).
     *  When a parent splits, each child inherits corner data directly from these
     *  grid points - the 8 parent corners, 12 edge midpoints, 6 face centers,
     *  and 1 body center are shared across children with no redundant sampling. */
    static inline const int32 ChildCornerSources[8][8] = {
    {  0,  8, 12, 24, 16, 22, 20, 26 },
    {  8,  1, 24, 13, 22, 17, 26, 21 },
    { 12, 24,  2,  9, 20, 26, 18, 23 },
    { 24, 13,  9,  3, 26, 21, 23, 19 },
    { 16, 22, 20, 26,  4, 10, 14, 25 },
    { 22, 17, 26, 21, 10,  5, 25, 15 },
    { 20, 26, 18, 23, 14, 25,  6, 11 },
    { 26, 21, 23, 19, 25, 15, 11,  7 },
    };

    /** Signed coordinates for all 27 points of the subdivision grid.
     *  Each component is -1, 0, or +1 relative to the node center.
     *    Indices  0-7:  corners (all components +/-1).
     *    Indices  8-19: edge midpoints (one component is 0).
     *    Indices 20-25: face centers (two components are +/-1, one is 0).
     *    Index    26:   body center (0, 0, 0).
     *  Used with CoordToGrid for neighbor-shared grid point lookups
     *  during GatherUniqueCorners. */
    static inline const int8 GridCoords[27][3] = {
        {-1,-1,-1},{+1,-1,-1},{-1,+1,-1},{+1,+1,-1},
        {-1,-1,+1},{+1,-1,+1},{-1,+1,+1},{+1,+1,+1},
        { 0,-1,-1},{ 0,+1,-1},{ 0,-1,+1},{ 0,+1,+1},
        {-1, 0,-1},{+1, 0,-1},{-1, 0,+1},{+1, 0,+1},
        {-1,-1, 0},{+1,-1, 0},{-1,+1, 0},{+1,+1, 0},
        {-1, 0, 0},{+1, 0, 0},{ 0,-1, 0},{ 0,+1, 0},
        { 0, 0,-1},{ 0, 0,+1},{ 0, 0, 0}
    };

    /** Inverse lookup of GridCoords: maps a signed offset (x,y,z) in {-1, 0, +1}
     *  to its grid index 0-26. Access as CoordToGrid[x+1][y+1][z+1].
     *  Used during GatherUniqueCorners to find the grid index of a neighbor-shared
     *  point by stepping one unit along an axis from a known grid coordinate. */
    static inline const int8 CoordToGrid[3][3][3] = {
        { {  0, 16,  4 }, { 12, 20, 14 }, {  2, 18,  6 } },
        { {  8, 22, 10 }, { 24, 26, 25 }, {  9, 23, 11 } },
        { {  1, 17,  5 }, { 13, 21, 15 }, {  3, 19,  7 } },
    };

    /** The 4 corner indices belonging to each of the 6 cube faces.
     *  Order: -X, +X, -Y, +Y, -Z, +Z (grouped by which bit is constant).
     *  Used to compute face-center positions and densities (grid indices 20-25)
     *  by averaging the 4 corners of each face during subdivision grid construction. */
    static inline const int32 FaceCorners[6][4] = {
        { 0, 2, 4, 6 },
        { 1, 3, 5, 7 },
        { 0, 1, 4, 5 },
        { 2, 3, 6, 7 },
        { 0, 1, 2, 3 },
        { 4, 5, 6, 7 },
    };
};