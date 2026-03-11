#pragma once

#include "CoreMinimal.h"

struct FVoxelCorner;
struct FVoxelEdge;
struct FVoxelFace;

struct VOXELPLUGIN_API OctreeConstants {
    inline static const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1, 1, -1),  FVector(1, 1, -1),
        FVector(-1, -1, 1),  FVector(1, -1, 1),
        FVector(-1, 1, 1),   FVector(1, 1, 1)
    };

    inline static const int EdgePairs[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7},
        {0, 2}, {1, 3}, {4, 6}, {5, 7},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    inline static const int FaceMap[6][4] = {
        {1, 3, 5, 7}, // XPos
        {0, 2, 4, 6}, // XNeg
        {2, 3, 6, 7}, // YPos
        {0, 1, 4, 5}, // YNeg
        {4, 5, 6, 7}, // ZPos
        {0, 1, 2, 3}  // ZNeg
    };

    enum EFace : uint8 {
        XPos = 0, XNeg = 1,
        YPos = 2, YNeg = 3,
        ZPos = 4, ZNeg = 5
    };

    inline static const uint8 FaceAxisBit[6] = {
        0, 0, 1, 1, 2, 2
    };

    inline static const bool FaceIsPositive[6] = {
        true, false, true, false, true, false
    };

    // ChildToParentEdgeMap[ChildIndex][ChildEdgeIndex] -> ParentEdgeIndex (-1 = no parent edge)
    // Edge axis groups: X-axis edges 0-3, Y-axis edges 4-7, Z-axis edges 8-11
    // EdgePairs: {0,1},{2,3},{4,5},{6,7} = X-axis; {0,2},{1,3},{4,6},{5,7} = Y-axis; {0,4},{1,5},{2,6},{3,7} = Z-axis
    static constexpr int32 ChildToParentEdgeMap[8][12] = {
        // Child 0 (0,0,0): Min corner --- touches parent edges on X-,Y-,Z- faces
        {  0, -1,  2, -1,  4, -1, -1, -1,  8, -1, 10, -1 },
        // Child 1 (1,0,0): touches parent edges on X+,Y-,Z- faces
        {  0, -1,  3, -1, -1,  5, -1, -1,  9, -1, -1, 11 },
        // Child 2 (0,1,0): touches parent edges on X-,Y+,Z- faces
        { -1,  1,  2, -1,  6, -1, -1, -1, -1,  8, 10, -1 },
        // Child 3 (1,1,0): touches parent edges on X+,Y+,Z- faces
        { -1,  1,  3, -1, -1,  7, -1, -1, -1,  9, -1, 11 },
        // Child 4 (0,0,1): touches parent edges on X-,Y-,Z+ faces
        {  0, -1, -1,  2,  4, -1, -1, -1,  8, -1, 10, -1 },
        // Child 5 (1,0,1): touches parent edges on X+,Y-,Z+ faces
        {  0, -1, -1,  3, -1,  5, -1, -1,  9, -1, -1, 11 },
        // Child 6 (0,1,1): touches parent edges on X-,Y+,Z+ faces
        { -1,  1, -1,  2,  6, -1, -1, -1, -1,  8, 10, -1 },
        // Child 7 (1,1,1): Max corner --- touches parent edges on X+,Y+,Z+ faces
        { -1,  1, -1,  3, -1,  7, -1, -1, -1,  9, -1, 11 },
    };

    static constexpr int32 ChildToParentFaceMap[8][6] = {
        // Child 0 (0,0,0): Min corner child
        { -1,  1, -1,  3, -1,  5 }, // Touching Parent's X-, Y-, Z-
        // Child 1 (1,0,0)
        {  0, -1, -1,  3, -1,  5 }, // Touching Parent's X+, Y-, Z-
        // Child 2 (0,1,0)
        { -1,  1,  2, -1, -1,  5 }, // Touching Parent's X-, Y+, Z-
        // Child 3 (1,1,0)
        {  0, -1,  2, -1, -1,  5 }, // Touching Parent's X+, Y+, Z-
        // Child 4 (0,0,1)
        { -1,  1, -1,  3,  4, -1 }, // Touching Parent's X-, Y-, Z+
        // Child 5 (1,0,1)
        {  0, -1, -1,  3,  4, -1 }, // Touching Parent's X+, Y-, Z+
        // Child 6 (0,1,1)
        { -1,  1,  2, -1,  4, -1 }, // Touching Parent's X-, Y+, Z+
        // Child 7 (1,1,1): Max corner child
        {  0, -1,  2, -1,  4, -1 }  // Touching Parent's X+, Y+, Z+
    };

    // FaceEdges[FaceIndex][4] -> Node Edge Indices (0-11)
    // Edges 0-3: X-axis, 4-7: Y-axis, 8-11: Z-axis
    static constexpr int32 FaceEdges[6][4] = {
        {  5,  7,  9, 11 },  // 0: XPos - Y and Z axis edges on X+ face
        {  4,  6,  8, 10 },  // 1: XNeg - Y and Z axis edges on X- face
        {  1,  3, 10, 11 },  // 2: YPos - X and Z axis edges on Y+ face
        {  0,  2,  8,  9 },  // 3: YNeg - X and Z axis edges on Y- face
        {  2,  3,  6,  7 },  // 4: ZPos - X and Y axis edges on Z+ face
        {  0,  1,  4,  5 },  // 5: ZNeg - X and Y axis edges on Z- face
    };

    inline static const uint8 OuterChildren[6][4] = {
        {1, 3, 5, 7},
        {0, 2, 4, 6},
        {2, 3, 6, 7},
        {0, 1, 4, 5},
        {4, 5, 6, 7},
        {0, 1, 2, 3},
    };

    inline static const uint8 InnerChildren[6][4] = {
        {0, 2, 4, 6},
        {1, 3, 5, 7},
        {0, 1, 4, 5},
        {2, 3, 6, 7},
        {0, 1, 2, 3},
        {4, 5, 6, 7},
    };
};

struct VOXELPLUGIN_API FMortonIndex {
    uint64 Low = 0;   // Bits 0-63:   levels 0-20
    uint64 High = 0;  // Bits 64-127: levels 21-41
    uint8 Depth = 0;

    void PushChild(uint8 ChildIndex) {
        uint8 BitOffset = Depth * 3;
        if (BitOffset < 64) {
            // Might straddle the boundary
            Low |= (uint64(ChildIndex & 0x7) << BitOffset);
            if (BitOffset > 61) {
                // Bits spill into High
                High |= (uint64(ChildIndex & 0x7) >> (64 - BitOffset));
            }
        }
        else {
            High |= (uint64(ChildIndex & 0x7) << (BitOffset - 64));
        }
        Depth++;
    }

    uint8 GetChildAtLevel(uint8 Level) const {
        uint8 BitOffset = Level * 3;
        if (BitOffset < 64) {
            if (BitOffset > 61) {
                // Straddles boundary
                uint8 LowBits = (Low >> BitOffset) & 0x7;
                uint8 HighBits = (High << (64 - BitOffset)) & 0x7;
                return (LowBits | HighBits) & 0x7;
            }
            return (Low >> BitOffset) & 0x7;
        }
        return (High >> (BitOffset - 64)) & 0x7;
    }

    uint8 LastChild() const {
        return GetChildAtLevel(Depth - 1);
    }

    void PopChild() {
        Depth--;
        uint8 BitOffset = Depth * 3;
        if (BitOffset < 64) {
            Low &= ~(uint64(0x7) << BitOffset);
            if (BitOffset > 61) {
                High &= ~(uint64(0x7) >> (64 - BitOffset));
            }
        }
        else {
            High &= ~(uint64(0x7) << (BitOffset - 64));
        }
    }

    // Common ancestor depth between two indices
    uint8 CommonDepth(const FMortonIndex& Other) const {
        uint8 MaxCheck = FMath::Min(Depth, Other.Depth);
        for (uint8 i = 0; i < MaxCheck; i++) {
            if (GetChildAtLevel(i) != Other.GetChildAtLevel(i))
                return i;
        }
        return MaxCheck;
    }

    bool operator==(const FMortonIndex& Other) const {
        return Low == Other.Low && High == Other.High && Depth == Other.Depth;
    }

    // For use as TMap key
    friend uint32 GetTypeHash(const FMortonIndex& Index) {
        uint32 Hash = ::GetTypeHash(Index.Low);
        Hash = HashCombine(Hash, ::GetTypeHash(Index.High));
        Hash = HashCombine(Hash, ::GetTypeHash(Index.Depth));
        return Hash;
    }
};

struct VOXELPLUGIN_API FQEF
{
    // Accumulated ATA matrix (symmetric 3x3, stored as 6 unique values)
    double ATA_00, ATA_01, ATA_02, ATA_11, ATA_12, ATA_22;

    // Accumulated ATb vector
    double ATb_X, ATb_Y, ATb_Z;

    // Accumulated bTb scalar (for error computation)
    double BTB;

    // Number of planes added
    int32 PlaneCount;

    // Mass point (average of intersection points --- used as fallback and bias)
    FVector MassPoint;

    // Accumulated normal (sum of normals passed to AddPlane --- for average normal output)
    FVector3f AccumulatedNormal;

    FQEF()
    {
        Reset();
    }

    void Reset()
    {
        ATA_00 = ATA_01 = ATA_02 = 0.0;
        ATA_11 = ATA_12 = 0.0;
        ATA_22 = 0.0;
        ATb_X = ATb_Y = ATb_Z = 0.0;
        BTB = 0.0;
        PlaneCount = 0;
        MassPoint = FVector::ZeroVector;
        AccumulatedNormal = FVector3f::ZeroVector;
    }

    // Add a plane constraint: the solution should be close to InPoint
    // along the direction InNormal
    void AddPlane(const FVector& InPoint, const FVector3f& InNormal)
    {
        // The plane equation is: dot(Normal, X - Point) = 0
        // Which expands to: dot(Normal, X) = dot(Normal, Point)
        // In matrix form: N * X = d, where N is the normal row and d = dot(N, P)

        double nx = InNormal.X;
        double ny = InNormal.Y;
        double nz = InNormal.Z;
        double d = nx * InPoint.X + ny * InPoint.Y + nz * InPoint.Z;

        // Accumulate ATA (outer product of normal with itself)
        ATA_00 += nx * nx;
        ATA_01 += nx * ny;
        ATA_02 += nx * nz;
        ATA_11 += ny * ny;
        ATA_12 += ny * nz;
        ATA_22 += nz * nz;

        // Accumulate ATb
        ATb_X += nx * d;
        ATb_Y += ny * d;
        ATb_Z += nz * d;

        // Accumulate bTb
        BTB += d * d;

        // Accumulate mass point and normal
        MassPoint += InPoint;
        AccumulatedNormal += InNormal;
        PlaneCount++;
    }

    // Returns the average normal from all planes added to this QEF.
    // Returns zero vector if no planes were added.
    FVector3f GetAverageNormal() const
    {
        if (PlaneCount == 0) return FVector3f::ZeroVector;
        return AccumulatedNormal.GetSafeNormal();
    }

    // Solve the QEF, returning the minimizing position.
    // InCellCenter and InCellExtent are used for clamping.
    // OutError receives the QEF error value if non-null.
    FVector Solve(const FVector& InCellCenter, double InCellExtent, double* OutError = nullptr) const
    {
        if (PlaneCount == 0)
        {
            if (OutError) *OutError = 0.0;
            return InCellCenter;
        }

        FVector AvgMassPoint = MassPoint / (double)PlaneCount;

        // Solve relative to mass point for numerical stability.
        // Instead of solving ATA * x = ATb directly, solve:
        //   ATA * (x - masspoint) = ATb - ATA * masspoint
        // This keeps the numbers small.

        double rhs_x = ATb_X - (ATA_00 * AvgMassPoint.X + ATA_01 * AvgMassPoint.Y + ATA_02 * AvgMassPoint.Z);
        double rhs_y = ATb_Y - (ATA_01 * AvgMassPoint.X + ATA_11 * AvgMassPoint.Y + ATA_12 * AvgMassPoint.Z);
        double rhs_z = ATb_Z - (ATA_02 * AvgMassPoint.X + ATA_12 * AvgMassPoint.Y + ATA_22 * AvgMassPoint.Z);

        // Solve ATA * delta = rhs using pseudoinverse with SVD threshold
        FVector Delta = SolveSVD(rhs_x, rhs_y, rhs_z);

        FVector Result = AvgMassPoint + Delta;

        // Clamp to cell bounds
        FVector MinBound = InCellCenter - FVector(InCellExtent);
        FVector MaxBound = InCellCenter + FVector(InCellExtent);
        Result.X = FMath::Clamp(Result.X, MinBound.X, MaxBound.X);
        Result.Y = FMath::Clamp(Result.Y, MinBound.Y, MaxBound.Y);
        Result.Z = FMath::Clamp(Result.Z, MinBound.Z, MaxBound.Z);

        if (OutError)
        {
            // Error = xT * ATA * x - 2 * xT * ATb + bTb
            *OutError = ComputeError(Result);
        }

        return Result;
    }

private:
    // Solve ATA * x = rhs using eigendecomposition of the 3x3 symmetric matrix.
    // Singular/near-singular eigenvalues are clamped, which gives us the 
    // pseudoinverse behavior we need for degenerate cases.
    FVector SolveSVD(double rhs_x, double rhs_y, double rhs_z) const
    {
        // For a 3x3 symmetric matrix, we use iterative Jacobi eigendecomposition.
        // This is simple, robust, and more than fast enough for 3x3.

        // Copy ATA into a working matrix
        double a[3][3] = {
            {ATA_00, ATA_01, ATA_02},
            {ATA_01, ATA_11, ATA_12},
            {ATA_02, ATA_12, ATA_22}
        };

        // Eigenvector matrix (starts as identity)
        double v[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        };

        // Jacobi iterations
        for (int iter = 0; iter < 20; iter++)
        {
            // Find largest off-diagonal element
            int p = 0, q = 1;
            double maxVal = FMath::Abs(a[0][1]);
            if (FMath::Abs(a[0][2]) > maxVal) { p = 0; q = 2; maxVal = FMath::Abs(a[0][2]); }
            if (FMath::Abs(a[1][2]) > maxVal) { p = 1; q = 2; maxVal = FMath::Abs(a[1][2]); }

            if (maxVal < 1e-12) break; // Converged

            // Compute rotation
            double theta = 0.5 * FMath::Atan2(2.0 * a[p][q], a[p][p] - a[q][q]);
            double c = FMath::Cos(theta);
            double s = FMath::Sin(theta);

            // Apply Givens rotation to A: A' = G^T * A * G
            double newA[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    newA[i][j] = a[i][j];

            newA[p][p] = c * c * a[p][p] + 2 * s * c * a[p][q] + s * s * a[q][q];
            newA[q][q] = s * s * a[p][p] - 2 * s * c * a[p][q] + c * c * a[q][q];
            newA[p][q] = newA[q][p] = 0.0; // This is what we're zeroing

            int r = 3 - p - q; // The other index
            newA[p][r] = c * a[p][r] + s * a[q][r];
            newA[r][p] = newA[p][r];
            newA[q][r] = -s * a[p][r] + c * a[q][r];
            newA[r][q] = newA[q][r];

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    a[i][j] = newA[i][j];

            // Update eigenvectors
            for (int i = 0; i < 3; i++)
            {
                double vip = v[i][p];
                double viq = v[i][q];
                v[i][p] = c * vip + s * viq;
                v[i][q] = -s * vip + c * viq;
            }
        }

        // Eigenvalues are now on the diagonal of a
        // Solve using pseudoinverse: x = V * diag(1/eigenvalue) * V^T * rhs
        // Clamp small eigenvalues to avoid amplifying noise

        double eigenvalues[3] = { a[0][0], a[1][1], a[2][2] };

        // Threshold: eigenvalues below this fraction of the largest are treated as zero
        double maxEigen = FMath::Max3(FMath::Abs(eigenvalues[0]),
            FMath::Abs(eigenvalues[1]),
            FMath::Abs(eigenvalues[2]));
        double threshold = maxEigen * 0.1; // 10% threshold --- fairly aggressive clamping

        // V^T * rhs
        double vtRhs[3];
        for (int i = 0; i < 3; i++)
        {
            vtRhs[i] = v[0][i] * rhs_x + v[1][i] * rhs_y + v[2][i] * rhs_z;
        }

        // Apply pseudoinverse of eigenvalues
        double scaled[3];
        for (int i = 0; i < 3; i++)
        {
            if (FMath::Abs(eigenvalues[i]) > threshold)
                scaled[i] = vtRhs[i] / eigenvalues[i];
            else
                scaled[i] = 0.0; // Singular direction --- collapse to mass point
        }

        // V * scaled
        FVector result;
        result.X = v[0][0] * scaled[0] + v[0][1] * scaled[1] + v[0][2] * scaled[2];
        result.Y = v[1][0] * scaled[0] + v[1][1] * scaled[1] + v[1][2] * scaled[2];
        result.Z = v[2][0] * scaled[0] + v[2][1] * scaled[1] + v[2][2] * scaled[2];

        return result;
    }

    double ComputeError(const FVector& Point) const
    {
        double x = Point.X, y = Point.Y, z = Point.Z;
        double xTATAx = x * (ATA_00 * x + ATA_01 * y + ATA_02 * z)
            + y * (ATA_01 * x + ATA_11 * y + ATA_12 * z)
            + z * (ATA_02 * x + ATA_12 * y + ATA_22 * z);
        double xTATb = x * ATb_X + y * ATb_Y + z * ATb_Z;
        return xTATAx - 2.0 * xTATb + BTB;
    }
};

struct VOXELPLUGIN_API FAdaptiveOctreeNode : public TSharedFromThis<FAdaptiveOctreeNode>
{
private:
    FVector3f GetInterpolatedNormal(FVector P);

    int DepthPrecisionFloor = 20;

public:
    FMortonIndex Index;
    TWeakPtr<FAdaptiveOctreeNode> Parent;
    TSharedPtr<FAdaptiveOctreeNode> Children[8];
    FVoxelCorner* Corners[8];
    FVoxelEdge* Edges[12];
    FVoxelFace* Faces[6];

    bool bDataReady = false;

    int ChunkDepth = 4;
    int DepthBounds[3];

    // Octree center
    FVector TreeCenter;

    //Chunk level parent center, used to construct the mesh at world origin in higher precision, 
    //then move the entire chunk back to chunk center - yeilds higher precision mesh
    FVector AnchorCenter;

    FVector Center;

    double Extent;

    FVector DualContourPosition;

    FVector3f DualContourNormal;

    //Position projected onto the sphere, used to construct ocean mesh
    FVector NormalizedPosition;

    // Turns off lod updates, useful for disabling lod during parallax movement, 
    // or for a warp effect -> could disable the lod, set an override position, let the lod update for the position the 
    // warp will END at, then warp the player to the point with no pop in
    // placeholder, not using this yet
    bool LodOverride = false;

    const bool IsLeaf();

    const bool IsSurface();

    const bool IsRoot();

    bool ShouldSplit(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    bool ShouldMerge(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void Split();

    void Merge();

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceChunks();

    void ComputeNormalizedPosition(double InRadius);

    TArray<struct FVoxelEdge*> GetSignChangeEdges() const;

    void ComputeDualContourPosition();

    FORCEINLINE FVector GetCornerPosition(int32 CornerIndex) const
    {
        // Uses your existing OctreeConstants::Offsets to derive world position
        return Center + (OctreeConstants::Offsets[CornerIndex] * Extent);
    }

    // Root Constructor
    FAdaptiveOctreeNode(FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    // Child Constructor
    FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 InChildIndex, FVector InAnchorCenter);

    ~FAdaptiveOctreeNode();
};
