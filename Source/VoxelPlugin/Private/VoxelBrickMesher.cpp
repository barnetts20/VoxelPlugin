#include "VoxelBrickMesher.h"
#include "FDensitySampleCompositor.h"
#include "FQEF.h"

namespace
{
    constexpr double Sqrt3 = 1.7320508075688772;

    /** Perpendicular axes of an edge along each axis, in the order the octree mesher walks
     *  the four cells around an edge (so triangles get the same winding). */
    constexpr int32 PerpA[3] = { 1, 2, 0 };
    constexpr int32 PerpB[3] = { 2, 0, 1 };

    /** Dense samples of one brick: points -2..N+1 on each axis. */
    struct FBrickSamples
    {
        int32 N = 0;
        int32 P = 0;
        double H = 1.0;
        FVector Min;
        TArray<float> D;

        int32 Index(int32 I, int32 J, int32 K) const { return ((K + 2) * P + (J + 2)) * P + (I + 2); }
        float At(int32 I, int32 J, int32 K) const { return D[Index(I, J, K)]; }

        /** Central-difference gradient in density per cm; valid for points -1..N. */
        FVector Gradient(int32 I, int32 J, int32 K) const
        {
            return FVector(
                At(I + 1, J, K) - At(I - 1, J, K),
                At(I, J + 1, K) - At(I, J - 1, K),
                At(I, J, K + 1) - At(I, J, K - 1)) / (2.0 * H);
        }
    };

    void SamplePoints(const FDensitySampleCompositor& Density, const TArray<double>& X, const TArray<double>& Y, const TArray<double>& Z,
        double Footprint, TArray<float>& Out)
    {
        Out.SetNumUninitialized(X.Num());
        Density.Sample(FSampleInput(X.GetData(), Y.GetData(), Z.GetData(), X.Num(), Footprint), Out.GetData());
    }

    /** True if the coarse test proves the brick's crossing region (points -1..N) empty. */
    bool IsProvablyEmpty(const FDensitySampleCompositor& Density, const FVector& Min, int32 N, double H, double Slope, int32& InOutSamples)
    {
        // One sample at the centre of the region.
        {
            const double Half = 0.5 * (N + 1) * H;
            const FVector Centre = Min + FVector(Half - H);
            double CX = Centre.X, CY = Centre.Y, CZ = Centre.Z;
            float DC = 0.0f;
            Density.Sample(FSampleInput(&CX, &CY, &CZ, 1, H), &DC);
            InOutSamples += 1;
            if (FMath::Abs((double)DC) > Slope * Half * Sqrt3)
                return true;
        }

        // Every 4th lattice point from -4 to N+4: each point of the region lies within 2*sqrt(3)
        // cells of one of them.
        const int32 C = N / 4 + 3;
        TArray<double> X, Y, Z;
        X.SetNumUninitialized(C * C * C);
        Y.SetNumUninitialized(C * C * C);
        Z.SetNumUninitialized(C * C * C);
        int32 n = 0;
        for (int32 K = 0; K < C; K++)
            for (int32 J = 0; J < C; J++)
                for (int32 I = 0; I < C; I++, n++)
                {
                    X[n] = Min.X + (4 * I - 4) * H;
                    Y[n] = Min.Y + (4 * J - 4) * H;
                    Z[n] = Min.Z + (4 * K - 4) * H;
                }
        TArray<float> D;
        SamplePoints(Density, X, Y, Z, H, D);
        InOutSamples += D.Num();

        const double Margin = Slope * 2.0 * Sqrt3 * H;
        bool bPositive = false, bNegative = false;
        for (const float V : D)
        {
            if (FMath::Abs((double)V) <= Margin) return false;
            (V < 0.0f ? bNegative : bPositive) = true;
        }
        return !(bPositive && bNegative);
    }

    /** Dual-contour vertex of the cell whose min corner is lattice point (I,J,K), solved in
     *  cell units ([0,1] per axis) the same way as FAdaptiveOctreeNode::SolveVertex. */
    bool SolveCell(const FBrickSamples& S, int32 I, int32 J, int32 K, FVector& OutLocal, FVector& OutNormal)
    {
        FQEF Qef;
        FVector MassPoint = FVector::ZeroVector;
        int32 Count = 0;

        for (int32 Axis = 0; Axis < 3; Axis++)
        {
            const int32 A = PerpA[Axis], B = PerpB[Axis];
            for (int32 e = 0; e < 4; e++)
            {
                int32 C0[3] = { I, J, K };
                C0[A] += (e & 1);
                C0[B] += (e >> 1);
                int32 C1[3] = { C0[0], C0[1], C0[2] };
                C1[Axis] += 1;

                const float D0 = S.At(C0[0], C0[1], C0[2]);
                const float D1 = S.At(C1[0], C1[1], C1[2]);
                if ((D0 < 0.0f) == (D1 < 0.0f)) continue;

                const double Denominator = (double)D0 - (double)D1;
                const double T = FMath::Abs(Denominator) < 1e-12 ? 0.5 : FMath::Clamp((double)D0 / Denominator, 0.0, 1.0);

                FVector P((double)(C0[0] - I), (double)(C0[1] - J), (double)(C0[2] - K));
                P[Axis] += T;

                FVector N = FMath::Lerp(S.Gradient(C0[0], C0[1], C0[2]), S.Gradient(C1[0], C1[1], C1[2]), T);
                if (!N.Normalize())
                    N = (S.Min + FVector(C0[0], C0[1], C0[2]) * S.H).GetSafeNormal();

                Qef.AddPlane(P, N);
                MassPoint += P;
                Count++;
            }
        }
        if (Count == 0) return false;

        MassPoint /= (double)Count;
        const FVector CellCentre(0.5);
        double Error = 0.0;
        const FVector Solved = Qef.Solve(CellCentre, 0.5, &Error);

        // Blend toward the mass point when the solution drifts or the planes disagree.
        constexpr double MaxDistSq = 0.25;
        const double DistSq = FVector::DistSquared(Solved, MassPoint);
        double TDist = 0.0;
        if (DistSq > MaxDistSq)
            TDist = 1.0;
        else if (DistSq > MaxDistSq * 0.25)
        {
            TDist = (DistSq - MaxDistSq * 0.25) / (MaxDistSq * 0.75);
            TDist = TDist * TDist * (3.0 - 2.0 * TDist);
        }
        double TError = FMath::Clamp(Error / MaxDistSq, 0.0, 1.0);
        TError = TError * TError * (3.0 - 2.0 * TError);

        OutLocal = FMath::Lerp(Solved, MassPoint, FMath::Max(TDist, TError));
        OutLocal.X = FMath::Clamp(OutLocal.X, 0.0, 1.0);
        OutLocal.Y = FMath::Clamp(OutLocal.Y, 0.0, 1.0);
        OutLocal.Z = FMath::Clamp(OutLocal.Z, 0.0, 1.0);

        OutNormal = Qef.AccumulatedNormal;
        if (!OutNormal.Normalize())
            OutNormal = (S.Min + (FVector(I, J, K) + OutLocal) * S.H).GetSafeNormal();
        return true;
    }
}

void VoxelBrickMesher::AddSkirts(FVoxelBrickMesh& Mesh, float Depth)
{
    // Directed edges keyed by their sorted ends; an undirected edge used once is open.
    struct FEdge
    {
        uint64 Key;
        int32 From;
        int32 To;
    };
    const int32 NumTris = Mesh.Indices.Num() / 3;
    TArray<FEdge> Edges;
    Edges.Reserve(NumTris * 3);
    for (int32 t = 0; t < NumTris; t++)
    {
        for (int32 e = 0; e < 3; e++)
        {
            const int32 A = Mesh.Indices[3 * t + e];
            const int32 B = Mesh.Indices[3 * t + (e + 1) % 3];
            const uint64 Lo = (uint64)(uint32)FMath::Min(A, B);
            const uint64 Hi = (uint64)(uint32)FMath::Max(A, B);
            Edges.Add({ (Lo << 32) | Hi, A, B });
        }
    }
    Edges.Sort([](const FEdge& X, const FEdge& Y) { return X.Key < Y.Key; });

    TArray<int32> SkirtVertex;
    SkirtVertex.Init(INDEX_NONE, Mesh.Positions.Num());
    auto GetSkirtVertex = [&](int32 V) -> int32
        {
            if (SkirtVertex[V] == INDEX_NONE)
            {
                SkirtVertex[V] = Mesh.Positions.Num();
                const FVector3f Normal = Mesh.Normals[V];
                const FVector3f Pos = Mesh.Positions[V];
                Mesh.Positions.Add(FVector3f(Pos.X - Normal.X * Depth, Pos.Y - Normal.Y * Depth, Pos.Z - Normal.Z * Depth));
                Mesh.Normals.Add(Normal);
            }
            return SkirtVertex[V];
        };

    for (int32 i = 0; i < Edges.Num();)
    {
        int32 j = i + 1;
        while (j < Edges.Num() && Edges[j].Key == Edges[i].Key) j++;
        if (j - i == 1)
        {
            // The surface runs From->To along this edge; the skirt runs To->From, so the strip
            // keeps the surface's orientation as it folds into the solid.
            const int32 A = Edges[i].From, B = Edges[i].To;
            const int32 SA = GetSkirtVertex(A), SB = GetSkirtVertex(B);
            Mesh.Indices.Add(B); Mesh.Indices.Add(A); Mesh.Indices.Add(SA);
            Mesh.Indices.Add(B); Mesh.Indices.Add(SA); Mesh.Indices.Add(SB);
        }
        i = j;
    }
}

void VoxelBrickMesher::MeshBrick(const FDensitySampleCompositor& Density, const FVoxelBrickGrid& Grid,
    const FInt64Vector& Brick, double SlopeBound, bool bNoSkip, FVoxelBrickMesh& Out, FVoxelBrickStats& OutStats)
{
    Out.Positions.Reset();
    Out.Normals.Reset();
    Out.Indices.Reset();
    OutStats = FVoxelBrickStats();

    const int32 N = FMath::Max(4, Grid.CellsPerBrick / 4 * 4);
    const double H = Grid.CellSize;
    const FVector Min = Grid.BrickMin(Brick);

    if (!bNoSkip && IsProvablyEmpty(Density, Min, N, H, FMath::Max(SlopeBound, 1.0), OutStats.Samples))
    {
        OutStats.bSkipped = true;
        return;
    }

    // Dense samples, points -2..N+1.
    FBrickSamples S;
    S.N = N;
    S.P = N + 4;
    S.H = H;
    S.Min = Min;
    {
        const int32 Count = S.P * S.P * S.P;
        TArray<double> X, Y, Z;
        X.SetNumUninitialized(Count);
        Y.SetNumUninitialized(Count);
        Z.SetNumUninitialized(Count);
        for (int32 K = -2; K < N + 2; K++)
            for (int32 J = -2; J < N + 2; J++)
                for (int32 I = -2; I < N + 2; I++)
                {
                    const int32 Idx = S.Index(I, J, K);
                    X[Idx] = Min.X + I * H;
                    Y[Idx] = Min.Y + J * H;
                    Z[Idx] = Min.Z + K * H;
                }
        SamplePoints(Density, X, Y, Z, H, S.D);
        OutStats.Samples += Count;
    }

    // Vertex index per cell -1..N-1 (INDEX_NONE until solved, -2 for no vertex).
    const int32 CellsPerAxis = N + 1;
    TArray<int32> CellVertex;
    CellVertex.Init(INDEX_NONE, CellsPerAxis * CellsPerAxis * CellsPerAxis);
    auto GetVertex = [&](int32 I, int32 J, int32 K) -> int32
        {
            int32& Slot = CellVertex[((K + 1) * CellsPerAxis + (J + 1)) * CellsPerAxis + (I + 1)];
            if (Slot == INDEX_NONE)
            {
                FVector Local, Normal;
                if (SolveCell(S, I, J, K, Local, Normal))
                {
                    Slot = Out.Positions.Num();
                    Out.Positions.Add(FVector3f((FVector(I, J, K) + Local) * H));
                    Out.Normals.Add(FVector3f(Normal));
                }
                else
                {
                    Slot = -2;
                }
            }
            return Slot;
        };

    // Owned edges: lower end at points 0..N-1 on every axis.
    float MaxGradient = 0.0f;
    for (int32 Axis = 0; Axis < 3; Axis++)
    {
        const int32 A = PerpA[Axis], B = PerpB[Axis];
        for (int32 K = 0; K < N; K++)
            for (int32 J = 0; J < N; J++)
                for (int32 I = 0; I < N; I++)
                {
                    int32 Hi[3] = { I, J, K };
                    Hi[Axis] += 1;
                    const float D0 = S.At(I, J, K);
                    const float D1 = S.At(Hi[0], Hi[1], Hi[2]);
                    if ((D0 < 0.0f) == (D1 < 0.0f)) continue;

                    MaxGradient = FMath::Max(MaxGradient, (float)S.Gradient(I, J, K).Size());

                    // The four cells around the edge, in the octree mesher's order:
                    // (+A,+B), (-A,+B), (-A,-B), (+A,-B).
                    int32 Quad[4];
                    bool bValid = true;
                    for (int32 c = 0; c < 4; c++)
                    {
                        int32 Cell[3] = { I, J, K };
                        if (c == 1 || c == 2) Cell[A] -= 1;
                        if (c >= 2) Cell[B] -= 1;
                        Quad[c] = GetVertex(Cell[0], Cell[1], Cell[2]);
                        bValid &= Quad[c] >= 0;
                    }
                    if (!bValid) continue;

                    // Same split choice and winding as FAdaptiveOctree::UpdateMeshChunkStreamData.
                    const FVector P0(Out.Positions[Quad[0]]), P1(Out.Positions[Quad[1]]);
                    const FVector P2(Out.Positions[Quad[2]]), P3(Out.Positions[Quad[3]]);
                    const double FoldA = FVector::DotProduct(((P1 - P0) ^ (P2 - P0)).GetSafeNormal(), ((P2 - P0) ^ (P3 - P0)).GetSafeNormal());
                    const double FoldB = FVector::DotProduct(((P1 - P0) ^ (P3 - P0)).GetSafeNormal(), ((P2 - P1) ^ (P3 - P1)).GetSafeNormal());
                    const bool bSplitB = FoldB > FoldA;
                    const int32 Tris[2][3] = {
                        { 0, 1, bSplitB ? 3 : 2 },
                        { bSplitB ? 1 : 0, 2, 3 } };
                    const bool bFlip = D0 < 0.0f;
                    for (int32 t = 0; t < 2; t++)
                    {
                        const int32 VA = Quad[Tris[t][0]], VB = Quad[Tris[t][1]], VC = Quad[Tris[t][2]];
                        Out.Indices.Add(VA);
                        Out.Indices.Add(bFlip ? VC : VB);
                        Out.Indices.Add(bFlip ? VB : VC);
                    }
                }
    }
    OutStats.MaxGradient = MaxGradient;
}
