#pragma once

#include "FSparseEditStore.h"
#include "CoreMinimal.h"

struct VOXELPLUGIN_API FNodeCorner {
    FVector Position;
    double Density;
    FVector Normal;

    FNodeCorner() : Position(0), Density(0), Normal(0, 0, 1) {}

    FNodeCorner(FVector InPos, double InDensity, FVector InNormal) : Position(InPos), Density(InDensity), Normal(InNormal) {}
};

struct VOXELPLUGIN_API FNodeEdge
{
    FNodeCorner Corners[2];
    double Size;
    bool SignChange;
    double Distance;
    FVector EdgeDirection;
    int Axis;  
    FVector ZeroCrossingPoint;

    // Constructor
    FNodeEdge() : Size(0), SignChange(false), Distance(0), Axis(0) {}

    FNodeEdge(FNodeCorner InCorner1, FNodeCorner InCorner2)
    {
        Corners[0] = InCorner1;
        Corners[1] = InCorner2;

        double d1 = InCorner1.Density;
        double d2 = InCorner2.Density;

        SignChange = (d1 < 0) != (d2 < 0);

        // Determine which corner is positive and which is negative
        FNodeCorner PosCorner = (d1 > d2) ? InCorner1 : InCorner2;
        FNodeCorner NegCorner = (d1 > d2) ? InCorner2 : InCorner1;

        Size = FVector::Dist(InCorner1.Position, InCorner2.Position);
        Distance = Size;

        // Compute edge direction: Always point from positive to negative
        EdgeDirection = (NegCorner.Position - PosCorner.Position).GetSafeNormal();

        // Safe axis detection: find the axis with the largest delta
        // This is immune to floating point noise at large coordinates
        FVector Delta = (InCorner2.Position - InCorner1.Position).GetAbs();
        if (Delta.X > Delta.Y && Delta.X > Delta.Z)
            Axis = 0;
        else if (Delta.Y > Delta.X && Delta.Y > Delta.Z)
            Axis = 1;
        else
            Axis = 2;

        if (SignChange) {
            // Stable zero-crossing interpolation
            // t = d1 / (d1 - d2) gives the parametric position along the edge
            // where the sign change occurs. Clamp to [0,1] for safety.
            double Denominator = d1 - d2;
            if (FMath::Abs(Denominator) < 1e-12) {
                // Both densities essentially equal — place at midpoint
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
    }

    bool IsCongruent(const FNodeEdge& Other) const {
        // Both corners must match (in either order) AND axis must match
        bool CornersMatch =
            (Corners[0].Position.Equals(Other.Corners[0].Position, .01)
                && Corners[1].Position.Equals(Other.Corners[1].Position, .01))
            || (Corners[0].Position.Equals(Other.Corners[1].Position, .01)
                && Corners[1].Position.Equals(Other.Corners[0].Position, .01));

        return CornersMatch && Axis == Other.Axis;
    }

    // Equality operator for ensuring uniqueness
    bool operator==(const FNodeEdge& Other) const
    {
        return IsCongruent(Other) && Other.EdgeDirection == EdgeDirection;
    }
};

struct VOXELPLUGIN_API FEdgeKey
{
    int64 X0, Y0, Z0;  // Quantized corner 0 (sorted so min corner is always first)
    int64 X1, Y1, Z1;  // Quantized corner 1
    int32 Axis;

    // Quantization grid — 0.001 is well within the 0.01 epsilon used in IsCongruent
    static constexpr double GridSize = 0.001;

    static int64 Quantize(double V)
    {
        return FMath::RoundToInt64(V / GridSize);
    }

    FEdgeKey() = default;

    FEdgeKey(const FNodeEdge& Edge)
    {
        int64 ax = Quantize(Edge.Corners[0].Position.X);
        int64 ay = Quantize(Edge.Corners[0].Position.Y);
        int64 az = Quantize(Edge.Corners[0].Position.Z);
        int64 bx = Quantize(Edge.Corners[1].Position.X);
        int64 by = Quantize(Edge.Corners[1].Position.Y);
        int64 bz = Quantize(Edge.Corners[1].Position.Z);

        // Canonical ordering: ensure (corner0 < corner1) so order doesn't matter
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

        Axis = Edge.Axis;
    }

    bool operator==(const FEdgeKey& Other) const
    {
        return X0 == Other.X0 && Y0 == Other.Y0 && Z0 == Other.Z0
            && X1 == Other.X1 && Y1 == Other.Y1 && Z1 == Other.Z1
            && Axis == Other.Axis;
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

    // Mass point (average of intersection points — used as fallback and bias)
    FVector MassPoint;

    // Accumulated normal (sum of normals passed to AddPlane — for average normal output)
    FVector AccumulatedNormal;

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
        AccumulatedNormal = FVector::ZeroVector;
    }

    // Add a plane constraint: the solution should be close to InPoint
    // along the direction InNormal
    void AddPlane(const FVector& InPoint, const FVector& InNormal)
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
    FVector GetAverageNormal() const
    {
        if (PlaneCount == 0) return FVector::ZeroVector;
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
        double threshold = maxEigen * 0.1; // 10% threshold — fairly aggressive clamping

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
                scaled[i] = 0.0; // Singular direction — collapse to mass point
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
    TFunction<double(FVector, FVector)>* DensityFunction;

    TSharedPtr<FSparseEditStore> EditStore;
    
    void ComputeDualContourPosition();
    
    FVector GetInterpolatedNormal(FVector P);
    
    double SampleDensity(FVector Position);
    
    bool bIsLeaf = true;
    
    int DepthPrecisionFloor = 18;

    // Static arrays, offset and edge pair lookup tables
    inline static const FVector Offsets[8] = {
        FVector(-1, -1, -1), FVector(1, -1, -1),
        FVector(-1, 1, -1), FVector(1, 1, -1),
        FVector(-1, -1, 1), FVector(1, -1, 1),
        FVector(-1, 1, 1), FVector(1, 1, 1)
    };

    inline static const int EdgePairs[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7}, // X-axis edges
        {0, 2}, {1, 3}, {4, 6}, {5, 7}, // Y-axis edges
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // Z-axis edges
    };

public:
    TArray<uint8> TreeIndex;
    
    TWeakPtr<FAdaptiveOctreeNode> Parent;
    
    TSharedPtr<FAdaptiveOctreeNode> Children[8];
    
    FNodeCorner Corners[8];
    
    TArray<FNodeEdge> SignChangeEdges;
    
    int ChunkDepth = 4;
    
    int DepthBounds[3];
    
    FVector AnchorCenter;
    
    FVector Center;
    
    double Extent;
    
    FVector DualContourPosition;
    
    FVector DualContourNormal;
    
    bool IsSurfaceNode;
    
    bool LodOverride = false;

    bool IsLeaf();

    bool IsRoot();
    
    bool ShouldSplit(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    bool ShouldMerge(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV);

    void Split();

    void Merge();

    void UpdateLod(FVector InCameraPosition, double InScreenSpaceThreshold, double InCameraFOV, TArray<FNodeEdge>& OutNodeEdges, TMap<FEdgeKey, int32>& EdgeMap, bool& OutChanged);
    
    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceChunks();

    TArray<FNodeEdge>& GetSignChangeEdges();

    // Root Constructor
    FAdaptiveOctreeNode(TFunction<double(FVector, FVector)>* DensityFunction, TSharedPtr<FSparseEditStore> InEditStore, FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    // Child Constructor
    FAdaptiveOctreeNode(TFunction<double(FVector, FVector)>* DensityFunction, TSharedPtr<FSparseEditStore> InEditStore, TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 InChildIndex, FVector InAnchorCenter);

    void ComputeNodeData();
};

FORCEINLINE uint32 GetTypeHash(const FEdgeKey& Key)
{
    uint32 Hash = GetTypeHash(Key.X0);
    Hash = HashCombine(Hash, GetTypeHash(Key.Y0));
    Hash = HashCombine(Hash, GetTypeHash(Key.Z0));
    Hash = HashCombine(Hash, GetTypeHash(Key.X1));
    Hash = HashCombine(Hash, GetTypeHash(Key.Y1));
    Hash = HashCombine(Hash, GetTypeHash(Key.Z1));
    Hash = HashCombine(Hash, GetTypeHash(Key.Axis));
    return Hash;
}