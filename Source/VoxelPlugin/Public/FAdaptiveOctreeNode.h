#pragma once

#include "CoreMinimal.h"

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
};

struct VOXELPLUGIN_API FNodeCorner {
    FVector Position;
    float Density;
    FVector3f Normal;

    FNodeCorner() : Position(0), Density(0), Normal(0, 0, 1) {}
    FNodeCorner(FVector InPos, float InDensity, FVector3f InNormal) : Position(InPos), Density(InDensity), Normal(InNormal) {}
};

struct VOXELPLUGIN_API FEdgeKey
{
    int32 X0, Y0, Z0;
    int32 X1, Y1, Z1;
    uint8 Axis;
    uint32 CachedHash;

    static constexpr double InvGridSize = 1.0;

    static int32 Quantize(double V)
    {
        return FMath::RoundToInt32(V * InvGridSize);
    }

    FEdgeKey() : X0(0), Y0(0), Z0(0), X1(0), Y1(0), Z1(0), Axis(0), CachedHash(0) {}

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

struct VOXELPLUGIN_API FNodeEdge
{
    FNodeCorner Corners[2];
    double Size;
    bool SignChange;
    uint8 Axis;
    FVector ZeroCrossingPoint;
    FEdgeKey CachedKey;

    FNodeEdge() : Size(0), SignChange(false), Axis(0) {}

    FNodeEdge(const FNodeCorner& InCorner1, const FNodeCorner& InCorner2)
    {
        Corners[0] = InCorner1;
        Corners[1] = InCorner2;

        double d1 = InCorner1.Density;
        double d2 = InCorner2.Density;

        SignChange = (d1 < 0) != (d2 < 0);
        Size = FVector::Dist(InCorner1.Position, InCorner2.Position);

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

// Single uint64 morton index -- supports up to depth 21 (3 bits * 21 = 63 bits)
struct VOXELPLUGIN_API FMortonIndex {
    uint64 Code = 0;
    uint8 Depth = 0;

    void PushChild(uint8 ChildIndex) {
        uint8 BitOffset = Depth * 3;
        Code |= (uint64(ChildIndex & 0x7) << BitOffset);
        Depth++;
    }

    uint8 GetChildAtLevel(uint8 Level) const {
        return (Code >> (Level * 3)) & 0x7;
    }

    uint8 LastChild() const {
        return GetChildAtLevel(Depth - 1);
    }

    bool operator==(const FMortonIndex& Other) const {
        return Code == Other.Code && Depth == Other.Depth;
    }

    friend uint32 GetTypeHash(const FMortonIndex& Index) {
        uint32 Hash = ::GetTypeHash(Index.Code);
        Hash = HashCombine(Hash, ::GetTypeHash(Index.Depth));
        return Hash;
    }
};

struct VOXELPLUGIN_API FQEF
{
    double ATA_00, ATA_01, ATA_02, ATA_11, ATA_12, ATA_22;
    double ATb_X, ATb_Y, ATb_Z;
    double BTB;
    int32 PlaneCount;
    FVector MassPoint;
    FVector AccumulatedNormal;

    FQEF() { Reset(); }

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

    void AddPlane(const FVector& InPoint, const FVector& InNormal)
    {
        double nx = InNormal.X, ny = InNormal.Y, nz = InNormal.Z;
        double d = nx * InPoint.X + ny * InPoint.Y + nz * InPoint.Z;

        ATA_00 += nx * nx; ATA_01 += nx * ny; ATA_02 += nx * nz;
        ATA_11 += ny * ny; ATA_12 += ny * nz; ATA_22 += nz * nz;
        ATb_X += nx * d; ATb_Y += ny * d; ATb_Z += nz * d;
        BTB += d * d;

        MassPoint += InPoint;
        AccumulatedNormal += InNormal;
        PlaneCount++;
    }

    FVector GetAverageNormal() const
    {
        if (PlaneCount == 0) return FVector::ZeroVector;
        return AccumulatedNormal.GetSafeNormal();
    }

    FVector Solve(const FVector& InCellCenter, double InCellExtent, double* OutError = nullptr) const
    {
        if (PlaneCount == 0)
        {
            if (OutError) *OutError = 0.0;
            return InCellCenter;
        }

        FVector AvgMassPoint = MassPoint / (double)PlaneCount;

        double rhs_x = ATb_X - (ATA_00 * AvgMassPoint.X + ATA_01 * AvgMassPoint.Y + ATA_02 * AvgMassPoint.Z);
        double rhs_y = ATb_Y - (ATA_01 * AvgMassPoint.X + ATA_11 * AvgMassPoint.Y + ATA_12 * AvgMassPoint.Z);
        double rhs_z = ATb_Z - (ATA_02 * AvgMassPoint.X + ATA_12 * AvgMassPoint.Y + ATA_22 * AvgMassPoint.Z);

        FVector Delta = SolveSVD(rhs_x, rhs_y, rhs_z);
        FVector Result = AvgMassPoint + Delta;

        FVector MinBound = InCellCenter - FVector(InCellExtent);
        FVector MaxBound = InCellCenter + FVector(InCellExtent);
        Result.X = FMath::Clamp(Result.X, MinBound.X, MaxBound.X);
        Result.Y = FMath::Clamp(Result.Y, MinBound.Y, MaxBound.Y);
        Result.Z = FMath::Clamp(Result.Z, MinBound.Z, MaxBound.Z);

        if (OutError) *OutError = ComputeError(Result);
        return Result;
    }

private:
    FVector SolveSVD(double rhs_x, double rhs_y, double rhs_z) const
    {
        double a[3][3] = {
            {ATA_00, ATA_01, ATA_02},
            {ATA_01, ATA_11, ATA_12},
            {ATA_02, ATA_12, ATA_22}
        };
        double v[3][3] = { {1,0,0}, {0,1,0}, {0,0,1} };

        for (int iter = 0; iter < 20; iter++)
        {
            int p = 0, q = 1;
            double maxVal = FMath::Abs(a[0][1]);
            if (FMath::Abs(a[0][2]) > maxVal) { p = 0; q = 2; maxVal = FMath::Abs(a[0][2]); }
            if (FMath::Abs(a[1][2]) > maxVal) { p = 1; q = 2; maxVal = FMath::Abs(a[1][2]); }
            if (maxVal < 1e-12) break;

            double theta = 0.5 * FMath::Atan2(2.0 * a[p][q], a[p][p] - a[q][q]);
            double c = FMath::Cos(theta);
            double s = FMath::Sin(theta);

            double newA[3][3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    newA[i][j] = a[i][j];

            newA[p][p] = c * c * a[p][p] + 2 * s * c * a[p][q] + s * s * a[q][q];
            newA[q][q] = s * s * a[p][p] - 2 * s * c * a[p][q] + c * c * a[q][q];
            newA[p][q] = newA[q][p] = 0.0;

            int r = 3 - p - q;
            newA[p][r] = c * a[p][r] + s * a[q][r]; newA[r][p] = newA[p][r];
            newA[q][r] = -s * a[p][r] + c * a[q][r]; newA[r][q] = newA[q][r];

            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    a[i][j] = newA[i][j];

            for (int i = 0; i < 3; i++)
            {
                double vip = v[i][p], viq = v[i][q];
                v[i][p] = c * vip + s * viq;
                v[i][q] = -s * vip + c * viq;
            }
        }

        double eigenvalues[3] = { a[0][0], a[1][1], a[2][2] };
        double maxEigen = FMath::Max3(FMath::Abs(eigenvalues[0]), FMath::Abs(eigenvalues[1]), FMath::Abs(eigenvalues[2]));
        double threshold = maxEigen * 0.1;

        double vtRhs[3];
        for (int i = 0; i < 3; i++)
            vtRhs[i] = v[0][i] * rhs_x + v[1][i] * rhs_y + v[2][i] * rhs_z;

        double scaled[3];
        for (int i = 0; i < 3; i++)
            scaled[i] = (FMath::Abs(eigenvalues[i]) > threshold) ? vtRhs[i] / eigenvalues[i] : 0.0;

        return FVector(
            v[0][0] * scaled[0] + v[0][1] * scaled[1] + v[0][2] * scaled[2],
            v[1][0] * scaled[0] + v[1][1] * scaled[1] + v[1][2] * scaled[2],
            v[2][0] * scaled[0] + v[2][1] * scaled[1] + v[2][2] * scaled[2]);
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
    void ComputeDualContourPosition(FVector TreeCenter);

    FVector GetInterpolatedNormal(FVector P);

    bool bIsLeaf = true;

    static constexpr int DepthPrecisionFloor = 20;

public:
    FMortonIndex Index;

    TWeakPtr<FAdaptiveOctreeNode> Parent;

    TSharedPtr<FAdaptiveOctreeNode> Children[8];

    FNodeCorner Corners[8];

    TArray<FNodeEdge> SignChangeEdges;

    uint8 DepthBounds[3];

    FVector Center;

    FVector ChunkCenter;

    double Extent;

    FVector DualContourPosition;

    FVector3f DualContourNormal;

    bool IsSurfaceNode = false;

    bool LodOverride = false;

    const bool IsLeaf() const;

    const bool IsRoot() const;

    static bool EvaluateSplit(double Extent, double DistSq, double FOVScale, double Threshold, int Depth, int MaxDepth, int MinDepth)
    {
        if (Depth >= MaxDepth) return false;
        if (Depth < MinDepth) return true;
        double lhs = 2.0 * Extent * FOVScale;
        return (lhs * lhs) > (Threshold * Threshold * DistSq);
    }

    static bool EvaluateMerge(double Extent, double DistSq, double FOVScale, double Threshold, int Depth, int ChunkDepth, int MinDepth)
    {
        if (Depth <= ChunkDepth) return false;
        if (Depth < MinDepth) return false;
        double lhs = 2.0 * Extent * FOVScale;
        double rhs = Threshold * 0.5;
        return (lhs * lhs) < (rhs * rhs * DistSq);
    }

    // TreeCenter passed explicitly -- no longer stored per-node
    bool ShouldSplit(FVector TreeCenter, FVector InCameraPosition, double InScreenSpaceThreshold, double InFOVScale);

    bool ShouldMerge(FVector InCameraPosition, double InScreenSpaceThreshold, double InFOVScale);

    void Split();

    void Merge();

    TArray<TSharedPtr<FAdaptiveOctreeNode>> GetSurfaceChunks();

    // Computes normalized position on the fly -- no longer stored on the node
    FVector ComputeNormalizedPosition(FVector TreeCenter, double InRadius) const;

    TArray<FNodeEdge>& GetSignChangeEdges();

    // TreeCenter passed explicitly for fallback normal computation
    void FinalizeFromExistingCorners(FVector TreeCenter, bool bSkipNormals = false);

    // Root Constructor
    FAdaptiveOctreeNode(FVector InCenter, double InExtent, int InChunkDepth, int InMinDepth, int InMaxDepth);

    // Child Constructor
    FAdaptiveOctreeNode(TSharedPtr<FAdaptiveOctreeNode> InParent, uint8 InChildIndex, FVector InAnchorCenter);
};

// Lightweight fixed-size result for SampleNodesAroundEdge
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
