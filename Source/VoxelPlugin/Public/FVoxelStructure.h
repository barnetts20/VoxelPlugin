//TODO: FIGURE OUT HOW MUCH OF THIS IS NEEDED, IF ANY, CLEAN UP AND CENTRALIZE AND STRUCTS WE ENDED UP NEEDING
#pragma once
#include "CoreMinimal.h"

/// <summary>
/// Sample position structure representing a sampled point in the density field
/// </summary>
struct VOXELPLUGIN_API FSamplePosition {
public:
    int SampleIndex;
    FVector Position;
    FVector Normal;
    double Density;

    FSamplePosition() {};
    FSamplePosition(int InSampleIndex, FVector Position, FVector Normal, double Density);
};

/// <summary>
/// Edge structure representing a connection between two sample positions
/// </summary>
struct VOXELPLUGIN_API FVoxelEdge {
    int EdgeIndex;
    FSamplePosition EndPoints[2];
    double Length;
    FVector MidPoint;
    FVector ZeroCrossingPoint;
    FVector Normal;
    bool SignChange;

    FVoxelEdge() {};
    FVoxelEdge(int InEdgeIndex, FSamplePosition InEndPoints[2]);
};

/// <summary>
/// Face structure representing a 4 sided face
/// </summary>
struct VOXELPLUGIN_API FQuadFace {
    int FaceIndex;                  // 0-5 corresponding to the 6 faces of a cube
    FVoxelEdge Edges[4];                 // 4 edges of the face
    FSamplePosition Corners[4];     // 4 sample positions at the corners of the face
    FVector MidPoint;               // Center of the face

    FQuadFace() {};
    FQuadFace(int InFaceIndex, FVoxelEdge Edges[4]);
};

/// <summary>
/// Face structure representing a 3 sided face
/// </summary>
struct VOXELPLUGIN_API FTriFace {
    int FaceIndex;                  // 0-5 corresponding to the 6 faces of a cube
    FVoxelEdge Edges[3];                 // 4 edges of the face
    FSamplePosition Corners[3];     // 4 sample positions at the corners of the face
    FVector MidPoint;               // Center of the face

    FTriFace() {};
    FTriFace(int InFaceIndex, FVoxelEdge Edges[3]);
};

/// <summary>
/// Tetrahedron structure representing a 3D tetrahedron for marching tetrahedra algorithm
/// Defined as a base face plus an apex point (often the center of the node)
/// </summary>
struct VOXELPLUGIN_API FTetrahedron {
    int TetraIndex;
    FQuadFace BaseFace;             // The quadrilateral base face of the tetrahedron, contains 4 of our 8 edges 
    FTriFace TriFaces[3];           // The triangular "side" faces of the tetrahedron
    FSamplePosition Apex;           // Apex point of the tetrahedron ("top" of the pyramid)
    FVoxelEdge ConnectingEdges[4];       // The extra 4 edges connecting the base to the apex

    FTetrahedron() {};
    FTetrahedron(int InTetraIndex, FQuadFace BaseFace, FSamplePosition Apex);
};

/// <summary>
/// Structure to encapsulate cached mesh data for a node, will eventually be recombined into realtime mesh buffers
/// </summary>
struct VOXELPLUGIN_API FInternalMeshBuffer {
    TArray<FVector> Positions;
    TArray<FVector> Normals;
    TArray<int32> Triangles;

    void Append(FInternalMeshBuffer InBuffer) {
        Positions.Append(InBuffer.Positions);
        Normals.Append(InBuffer.Normals);
        Triangles.Append(InBuffer.Triangles);
    }
};

struct VOXELPLUGIN_API FCameraInfo {
    FVector Position = FVector::ZeroVector;
    FVector Direction = FVector(1,0,0);
    double FOV = 0;
};


/// <summary>
/// Static class for voxel-related constants, lookup tables, and generic functions
/// </summary>
class VOXELPLUGIN_API FVoxelStructures
{
public:
    /// <summary>
    /// Corner offsets from cell origin (for a unit cube centered at origin)
    /// Ordered as:
    /// 0: (-1, -1, -1) | 1: (1, -1, -1)
    /// 2: (-1,  1, -1) | 3: (1,  1, -1)
    /// 4: (-1, -1,  1) | 5: (1, -1,  1)
    /// 6: (-1,  1,  1) | 7: (1,  1,  1)
    /// </summary>
    static const FVector CornerOffsets[8];

    /// <summary>
    /// Edge pairs representing the 12 edges of a cube
    /// Each pair contains the indices of the two corners that the edge connects
    /// </summary>
    static const int EdgePairs[12][2];

    /// <summary>
    /// Maps a EFaceAxis enum value to its FVector
    /// </summary>
    static const FVector FaceAxes[6];

    /// <summary>
    /// Face corner indices - each face has 4 corners
    /// Indices refer to the 8 corners of the cube
    /// </summary>
    static const int FaceCorners[6][4];

    /// <summary>
    /// Face-edge connections - each face has 4 edges
    /// Indices refer to the 12 edges of the cube
    /// </summary>
    static const int FaceEdges[6][4];

    /// <summary>
    /// Tetrahedron decomposition
    /// This array maps each of the 6 tetrahedra to its 4 corner indices
    /// Each tetrahedron is formed by 3 corners of a cube face plus the center point
    /// </summary>
    static const int TetrahedronCorners[6][4];

    /// <summary>
    /// Tetrahedron edge connections
    /// For each of the 6 tetrahedra, lists its 6 edges
    /// Each edge is defined by the indices of the two corner indices it connects
    /// </summary>
    static const int TetrahedronEdges[6][6][2];

    /// <summary>
    /// Lookup table for marching tetrahedra edge cases
    /// Each row is a bit pattern for which corners are inside/outside the isosurface
    /// Each value is a bit pattern for which edges should be intersected
    /// </summary>
    static const uint8 TetrahedronEdgeTable[16];

    /// <summary>
    /// Marching Tetrahedra triangle table
    /// For each of the 16 possible inside/outside configurations (0-15),
    /// lists the edges to use to form triangles
    /// Format: list of edge indices (0-5) followed by -1 as a terminator
    /// </summary>
    static const int TetrahedronTriTable[16][7];
};

// Define the static corner offsets
inline const FVector FVoxelStructures::CornerOffsets[8] = {
    FVector(-1, -1, -1), FVector(1, -1, -1),
    FVector(-1,  1, -1), FVector(1,  1, -1),
    FVector(-1, -1,  1), FVector(1, -1,  1),
    FVector(-1,  1,  1), FVector(1,  1,  1)
};

// Define the edge pairs
inline const int FVoxelStructures::EdgePairs[12][2] = {
    {0, 1}, {2, 3}, {4, 5}, {6, 7}, // X-axis edges
    {0, 2}, {1, 3}, {4, 6}, {5, 7}, // Y-axis edges
    {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Z-axis edges
};

// Define the face corner indices
inline const int FVoxelStructures::FaceCorners[6][4] = {
    {0, 2, 6, 4}, // -X face (X_NEGATIVE)
    {1, 3, 7, 5}, // +X face (X_POSITIVE)
    {0, 1, 5, 4}, // -Y face (Y_NEGATIVE)
    {2, 3, 7, 6}, // +Y face (Y_POSITIVE)
    {0, 1, 3, 2}, // -Z face (Z_NEGATIVE)
    {4, 5, 7, 6}  // +Z face (Z_POSITIVE)
};

// Define the face-edge connections
inline const int FVoxelStructures::FaceEdges[6][4] = {
    {4, 10, 6, 8},   // -X face (X_NEGATIVE)
    {5, 11, 7, 9},   // +X face (X_POSITIVE)
    {0, 9, 2, 8},    // -Y face (Y_NEGATIVE)
    {1, 11, 3, 10},  // +Y face (Y_POSITIVE)
    {0, 5, 1, 4},    // -Z face (Z_NEGATIVE)
    {2, 7, 3, 6}     // +Z face (Z_POSITIVE)
};

inline const FVector FVoxelStructures::FaceAxes[6] = {
    FVector(-1, 0, 0),
    FVector( 1, 0, 0),
    FVector( 0,-1, 0),
    FVector( 0, 1, 0),
    FVector( 0, 0,-1),
    FVector( 0, 0, 1)
};
// Define the tetrahedron corner indices
// For each of the 6 tetrahedra, lists the 4 corner indices
// Format: [3 corners of the face, center point(8)]
inline const int FVoxelStructures::TetrahedronCorners[6][4] = {
    {0, 2, 6, 8}, // -X face: corners 0,2,6 + center
    {1, 3, 7, 8}, // +X face: corners 1,3,7 + center
    {0, 1, 5, 8}, // -Y face: corners 0,1,5 + center
    {2, 3, 7, 8}, // +Y face: corners 2,3,7 + center
    {0, 1, 3, 8}, // -Z face: corners 0,1,3 + center
    {4, 5, 7, 8}  // +Z face: corners 4,5,7 + center
};

// Define the tetrahedron edges for each tetrahedron
// For each of the 6 tetrahedra, lists its 6 edge connections
inline const int FVoxelStructures::TetrahedronEdges[6][6][2] = {
    // Tetrahedron 0: corners 0,2,6,8
    {
        {0, 2}, {2, 6}, {6, 0}, // Edges of the face
        {0, 8}, {2, 8}, {6, 8}  // Edges to center
    },
    // Tetrahedron 1: corners 1,3,7,8
    {
        {1, 3}, {3, 7}, {7, 1}, // Edges of the face
        {1, 8}, {3, 8}, {7, 8}  // Edges to center
    },
    // Tetrahedron 2: corners 0,1,5,8
    {
        {0, 1}, {1, 5}, {5, 0}, // Edges of the face
        {0, 8}, {1, 8}, {5, 8}  // Edges to center
    },
    // Tetrahedron 3: corners 2,3,7,8
    {
        {2, 3}, {3, 7}, {7, 2}, // Edges of the face
        {2, 8}, {3, 8}, {7, 8}  // Edges to center
    },
    // Tetrahedron 4: corners 0,1,3,8
    {
        {0, 1}, {1, 3}, {3, 0}, // Edges of the face
        {0, 8}, {1, 8}, {3, 8}  // Edges to center
    },
    // Tetrahedron 5: corners 4,5,7,8
    {
        {4, 5}, {5, 7}, {7, 4}, // Edges of the face
        {4, 8}, {5, 8}, {7, 8}  // Edges to center
    }
};

// Define the tetrahedron edge table
inline const uint8 FVoxelStructures::TetrahedronEdgeTable[16] = {
    0x00, 0x0D, 0x13, 0x1E, 0x26, 0x2B, 0x35, 0x38,
    0x38, 0x35, 0x2B, 0x26, 0x1E, 0x13, 0x0D, 0x00
};

// Define the marching tetrahedra triangle table
// Each row corresponds to a case in the edge table
// Values: Edge indices to use (0-5) followed by -1 as a terminator
inline const int FVoxelStructures::TetrahedronTriTable[16][7] = {
    {-1, -1, -1, -1, -1, -1, -1}, // Case 0: All outside or all inside
    { 0,  2,  3, -1, -1, -1, -1}, // Case 1: V0 inside
    { 0,  4,  1, -1, -1, -1, -1}, // Case 2: V1 inside
    { 2,  3,  1,  1,  3,  4, -1}, // Case 3: V0,V1 inside
    { 1,  5,  2, -1, -1, -1, -1}, // Case 4: V2 inside
    { 0,  3,  5,  0,  5,  1, -1}, // Case 5: V0,V2 inside
    { 0,  4,  5,  0,  5,  2, -1}, // Case 6: V1,V2 inside
    { 3,  5,  4, -1, -1, -1, -1}, // Case 7: V0,V1,V2 inside
    { 3,  5,  4, -1, -1, -1, -1}, // Case 8: V3 inside
    { 0,  2,  5,  0,  5,  4, -1}, // Case 9: V0,V3 inside
    { 0,  4,  3,  0,  3,  5, -1}, // Case 10: V1,V3 inside
    { 2,  5,  1, -1, -1, -1, -1}, // Case 11: V0,V1,V3 inside
    { 1,  3,  4,  1,  4,  5, -1}, // Case 12: V2,V3 inside
    { 0,  1,  4, -1, -1, -1, -1}, // Case 13: V0,V2,V3 inside
    { 0,  3,  2, -1, -1, -1, -1}, // Case 14: V1,V2,V3 inside
    {-1, -1, -1, -1, -1, -1, -1}  // Case 15: All inside
};