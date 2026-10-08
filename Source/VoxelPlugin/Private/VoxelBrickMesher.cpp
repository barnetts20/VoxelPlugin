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

    /** True if lattice samples prove a stitched brick's face region empty: the closed box
     *  (points 0..N), whose edges are the only ones it emits faces for. Every 4th point
     *  first, then every 2nd (each point of the box lies within 2 * sqrt(3) and sqrt(3)
     *  cells of one of them). The extra cell allows for points the mesher samples at a finer
     *  neighbour's footprint. */
    bool IsStitchedProvablyEmpty(const FDensitySampleCompositor& Density, const FVector& Min, int32 N, double H, double Slope, int32& InOutSamples)
    {
        bool bPositive = false, bNegative = false;
        for (const int32 Step : { 4, 2 })
        {
            const int32 C = N / Step + 1;
            TArray<double> X, Y, Z;
            X.SetNumUninitialized(C * C * C);
            Y.SetNumUninitialized(C * C * C);
            Z.SetNumUninitialized(C * C * C);
            int32 n = 0;
            for (int32 K = 0; K < C; K++)
                for (int32 J = 0; J < C; J++)
                    for (int32 I = 0; I < C; I++, n++)
                    {
                        X[n] = Min.X + Step * I * H;
                        Y[n] = Min.Y + Step * J * H;
                        Z[n] = Min.Z + Step * K * H;
                    }
            TArray<float> D;
            SamplePoints(Density, X, Y, Z, H, D);
            InOutSamples += D.Num();

            const double Margin = Slope * 0.5 * Step * Sqrt3 * H + H;
            bool bClear = true;
            for (const float V : D)
            {
                (V < 0.0f ? bNegative : bPositive) = true;
                bClear &= FMath::Abs((double)V) > Margin;
            }
            if (bPositive && bNegative) return false;
            if (bClear) return true;
        }
        return false;
    }

    /** QEF solution in cell units ([0,1] per axis), blended toward the mass point when it
     *  drifts or the planes disagree, as in FAdaptiveOctreeNode::SolveVertex. */
    FVector BlendedVertex(const FQEF& Qef, const FVector& MassPoint)
    {
        const FVector CellCentre(0.5);
        double Error = 0.0;
        const FVector Solved = Qef.Solve(CellCentre, 0.5, &Error);

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

        FVector Local = FMath::Lerp(Solved, MassPoint, FMath::Max(TDist, TError));
        Local.X = FMath::Clamp(Local.X, 0.0, 1.0);
        Local.Y = FMath::Clamp(Local.Y, 0.0, 1.0);
        Local.Z = FMath::Clamp(Local.Z, 0.0, 1.0);
        return Local;
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

        OutLocal = BlendedVertex(Qef, MassPoint / (double)Count);
        OutNormal = Qef.AccumulatedNormal;
        if (!OutNormal.Normalize())
            OutNormal = (S.Min + (FVector(I, J, K) + OutLocal) * S.H).GetSafeNormal();
        return true;
    }

    // ---------------------------------------------------------------------------------------
    // Stitched meshing. Coordinates are integers in half-cells of the brick's level ("units"),
    // relative to the brick's min corner: the brick spans [0, 2N], its cells are 2 units, a
    // finer neighbour's 1 and a coarser neighbour's 4. Levels are relative to the brick's.
    // ---------------------------------------------------------------------------------------

    constexpr double ProbeOffset = 0.25;   // units off an edge or face; stays inside one leaf

    struct FStitchCell
    {
        int32 Level;      // -1, 0 or +1
        FIntVector Min;   // units
        int32 Size;       // units
    };

    struct FStitchEdge
    {
        FIntVector Lo;
        int32 Axis;
        int32 Length;   // units
    };

    /** A minimal edge this brick emits, with its four cells in the octree mesher's order. */
    struct FStitchFace
    {
        int32 Cells[4];
        bool bFlip;
    };

    class FStitcher
    {
    public:
        FStitcher(const FDensitySampleCompositor& InDensity, const FVoxelBrickGrid& Grid, const FInt64Vector& Brick, const int8 (&InLevels)[27], uint32 InMeshed)
            : Density(InDensity), N(Grid.CellsPerBrick), CellSize(Grid.CellSize), Unit(0.5 * Grid.CellSize), Origin(Grid.Origin), Meshed(InMeshed | (1u << 13))
        {
            MinIndex = FInt64Vector(Brick.X * 2 * N, Brick.Y * 2 * N, Brick.Z * 2 * N);
            for (int32 i = 0; i < 27; i++)
                Levels[i] = FMath::Clamp((int32)InLevels[i], -1, 1);
            Levels[13] = 0;
        }

        int32 Samples = 0;

        /** World position of a lattice point. Built from its global index so every brick
         *  that samples the point passes the density the same coordinates. */
        FVector Position(int32 X, int32 Y, int32 Z) const
        {
            return Origin + FVector((double)(MinIndex.X + X), (double)(MinIndex.Y + Y), (double)(MinIndex.Z + Z)) * Unit;
        }

        FVector BrickMin() const { return Position(0, 0, 0); }

        FIntVector BlockAt(const FIntVector& P) const
        {
            return FIntVector(BlockOf(P.X), BlockOf(P.Y), BlockOf(P.Z));
        }

        /** True if the block (offsets -1..1) holds a brick that is meshed. */
        bool IsMeshed(const FIntVector& Block) const
        {
            return (Meshed >> ((Block.Z + 1) * 9 + (Block.Y + 1) * 3 + (Block.X + 1))) & 1;
        }

        /** Level of the leaf owning lattice point P; leaves are half-open boxes. */
        int32 OwnerLevel(const FIntVector& P) const
        {
            const FIntVector Block = BlockAt(P);
            return Levels[(Block.Z + 1) * 9 + (Block.Y + 1) * 3 + (Block.X + 1)];
        }

        /** Cell containing a point off the lattice (a probe). */
        FStitchCell CellAt(const FVector& Probe) const
        {
            const FIntVector Floor(FMath::FloorToInt32(Probe.X), FMath::FloorToInt32(Probe.Y), FMath::FloorToInt32(Probe.Z));
            FStitchCell Cell;
            Cell.Level = OwnerLevel(Floor);
            Cell.Size = 1 << (Cell.Level + 1);
            Cell.Min = FIntVector(FloorTo(Floor.X, Cell.Size), FloorTo(Floor.Y, Cell.Size), FloorTo(Floor.Z, Cell.Size));
            return Cell;
        }

        /** Samples the brick's own lattice, points -2..N+1 (even units -4..2N+2). */
        void Prefetch()
        {
            DenseCount = N + 4;
            DenseValues.SetNumUninitialized(DenseCount * DenseCount * DenseCount);
            TArray<FIntVector> Points;
            Points.Reserve(DenseValues.Num());
            for (int32 k = 0; k < DenseCount; k++)
                for (int32 j = 0; j < DenseCount; j++)
                    for (int32 i = 0; i < DenseCount; i++)
                        Points.Add(FIntVector(2 * i - 4, 2 * j - 4, 2 * k - 4));
            SampleBatch(Points, [this](int32 Index, float Value) { DenseValues[Index] = Value; });
        }

        /** Queues a lattice point off the prefetched lattice for the next Flush. */
        void Request(const FIntVector& P)
        {
            if (DenseIndex(P) != INDEX_NONE || Sparse.Contains(P)) return;
            Sparse.Add(P, 0.0f);
            Pending.Add(P);
        }

        void RequestGradient(const FIntVector& P, int32 Step)
        {
            for (int32 Axis = 0; Axis < 3; Axis++)
            {
                FIntVector Q = P;
                Q[Axis] += Step;
                Request(Q);
                Q[Axis] -= 2 * Step;
                Request(Q);
            }
        }

        /** Samples every queued point. */
        void Flush()
        {
            SampleBatch(Pending, [this](int32 Index, float Value) { Sparse.Add(Pending[Index], Value); });
            Pending.Reset();
        }

        /** Density at a prefetched or flushed lattice point, band-limited to the cell size of
         *  the leaf owning it, so every brick that samples the point gets the same value. */
        float Sample(const FIntVector& P) const
        {
            const int32 Dense = DenseIndex(P);
            if (Dense != INDEX_NONE)
                return DenseValues[Dense];
            const float* Found = Sparse.Find(P);
            check(Found);
            return Found ? *Found : 0.0f;
        }

        /** Central-difference gradient in density per cm, with a step of Step units. */
        FVector Gradient(const FIntVector& P, int32 Step) const
        {
            return FVector(
                Sample(P + FIntVector(Step, 0, 0)) - Sample(P - FIntVector(Step, 0, 0)),
                Sample(P + FIntVector(0, Step, 0)) - Sample(P - FIntVector(0, Step, 0)),
                Sample(P + FIntVector(0, 0, Step)) - Sample(P - FIntVector(0, 0, Step))) / (2.0 * Step * Unit);
        }

        /** The minimal edges on a cell's boundary, whose crossings its QEF uses: a cell edge
         *  that a finer cell touches is replaced by its halves, and a face shared with finer
         *  cells adds the four half-edges crossing it. A coarse cell next to finer ones so
         *  solves from the crossings the finer cells use. */
        void CellEdges(const FStitchCell& Cell, TArray<FStitchEdge>& Out) const
        {
            const int32 S = Cell.Size;
            const int32 H = S / 2;
            for (int32 Axis = 0; Axis < 3; Axis++)
            {
                const int32 A = PerpA[Axis], B = PerpB[Axis];
                for (int32 e = 0; e < 4; e++)
                {
                    FIntVector Lo = Cell.Min;
                    Lo[A] += (e & 1) * S;
                    Lo[B] += (e >> 1) * S;
                    bool bSplit = false;
                    for (int32 q = 0; q < 4 && H >= 1 && !bSplit; q++)
                    {
                        FVector Probe(Lo);
                        Probe[Axis] += 0.5 * S;
                        Probe[A] += (q & 1) ? ProbeOffset : -ProbeOffset;
                        Probe[B] += (q & 2) ? ProbeOffset : -ProbeOffset;
                        bSplit = CellAt(Probe).Level < Cell.Level;
                    }
                    if (bSplit)
                    {
                        FIntVector Mid = Lo;
                        Mid[Axis] += H;
                        Out.Add({ Lo, Axis, H });
                        Out.Add({ Mid, Axis, H });
                    }
                    else
                    {
                        Out.Add({ Lo, Axis, S });
                    }
                }
            }
            for (int32 Face = 0; Face < 6 && H >= 1; Face++)
            {
                const int32 Normal = Face / 2;
                const int32 Side = Face % 2;
                FVector Probe = FVector(Cell.Min) + FVector(0.5 * S);
                Probe[Normal] = Cell.Min[Normal] + Side * S + (Side ? ProbeOffset : -ProbeOffset);
                if (CellAt(Probe).Level >= Cell.Level) continue;

                // The face's two midlines, each in two halves.
                for (int32 Along = 0; Along < 3; Along++)
                {
                    if (Along == Normal) continue;
                    const int32 Across = 3 - Normal - Along;
                    FIntVector Lo = Cell.Min;
                    Lo[Normal] += Side * S;
                    Lo[Across] += H;
                    FIntVector Mid = Lo;
                    Mid[Along] += H;
                    Out.Add({ Lo, Along, H });
                    Out.Add({ Mid, Along, H });
                }
            }
        }

        static FIntVector EdgeEnd(const FStitchEdge& Edge)
        {
            FIntVector Hi = Edge.Lo;
            Hi[Edge.Axis] += Edge.Length;
            return Hi;
        }

        bool Crosses(const FStitchEdge& Edge) const
        {
            return (Sample(Edge.Lo) < 0.0f) != (Sample(EdgeEnd(Edge)) < 0.0f);
        }

        /** Dual-contour vertex of a cell from the crossings of its edges, appended to Out;
         *  INDEX_NONE without crossings. Every quantity here scales exactly with the unit
         *  size, so bricks of different levels computing the same cell get the same vertex. */
        int32 SolveVertex(const FStitchCell& Cell, const FStitchEdge* Edges, int32 NumEdges, FVoxelBrickMesh& Out) const
        {
            const int32 S = Cell.Size;
            FQEF Qef;
            FVector MassPoint = FVector::ZeroVector;
            int32 Count = 0;
            for (int32 i = 0; i < NumEdges; i++)
            {
                const FStitchEdge& Edge = Edges[i];
                const FIntVector Hi = EdgeEnd(Edge);
                const float D0 = Sample(Edge.Lo);
                const float D1 = Sample(Hi);
                if ((D0 < 0.0f) == (D1 < 0.0f)) continue;

                const double Denominator = (double)D0 - (double)D1;
                const double T = FMath::Abs(Denominator) < 1e-12 ? 0.5 : FMath::Clamp((double)D0 / Denominator, 0.0, 1.0);
                FVector P(Edge.Lo - Cell.Min);
                P[Edge.Axis] += T * Edge.Length;
                P /= (double)S;

                FVector Normal = FMath::Lerp(Gradient(Edge.Lo, Edge.Length), Gradient(Hi, Edge.Length), T);
                if (!Normal.Normalize())
                    Normal = Position(Edge.Lo.X, Edge.Lo.Y, Edge.Lo.Z).GetSafeNormal();

                Qef.AddPlane(P, Normal);
                MassPoint += P;
                Count++;
            }
            if (Count == 0) return INDEX_NONE;

            const FVector Local = FVector(Cell.Min) + BlendedVertex(Qef, MassPoint / (double)Count) * S;
            FVector VertexNormal = Qef.AccumulatedNormal;
            if (!VertexNormal.Normalize())
                VertexNormal = (BrickMin() + Local * Unit).GetSafeNormal();
            Out.Positions.Add(FVector3f(Local * Unit));
            Out.Normals.Add(FVector3f(VertexNormal));
            return Out.Positions.Num() - 1;
        }

    private:
        const FDensitySampleCompositor& Density;
        int32 N;
        double CellSize;
        double Unit;
        FVector Origin;
        FInt64Vector MinIndex;
        uint32 Meshed;
        int32 Levels[27];
        TArray<float> DenseValues;
        int32 DenseCount = 0;
        TMap<FIntVector, float> Sparse;
        TArray<FIntVector> Pending;

        int32 BlockOf(int32 X) const { return X < 0 ? -1 : (X >= 2 * N ? 1 : 0); }
        static int32 FloorTo(int32 X, int32 Step) { return (X >= 0 ? X / Step : -((-X + Step - 1) / Step)) * Step; }

        int32 DenseIndex(const FIntVector& P) const
        {
            if ((P.X & 1) | (P.Y & 1) | (P.Z & 1)) return INDEX_NONE;
            const int32 I = (P.X + 4) / 2, J = (P.Y + 4) / 2, K = (P.Z + 4) / 2;
            if (P.X < -4 || P.Y < -4 || P.Z < -4 || I >= DenseCount || J >= DenseCount || K >= DenseCount) return INDEX_NONE;
            return (K * DenseCount + J) * DenseCount + I;
        }

        /** Samples Points in one batch per owning level (the footprint) and hands each value
         *  to Store with the point's index. */
        template<typename FStore>
        void SampleBatch(const TArray<FIntVector>& Points, FStore&& Store)
        {
            TArray<double> X, Y, Z;
            TArray<int32> Slots;
            TArray<float> D;
            for (int32 Group = -1; Group <= 1; Group++)
            {
                X.Reset(); Y.Reset(); Z.Reset(); Slots.Reset();
                for (int32 i = 0; i < Points.Num(); i++)
                {
                    if (OwnerLevel(Points[i]) != Group) continue;
                    const FVector P = Position(Points[i].X, Points[i].Y, Points[i].Z);
                    X.Add(P.X); Y.Add(P.Y); Z.Add(P.Z);
                    Slots.Add(i);
                }
                if (X.Num() == 0) continue;
                D.SetNumUninitialized(X.Num());
                const double Footprint = Group < 0 ? 0.5 * CellSize : (Group > 0 ? 2.0 * CellSize : CellSize);
                Density.Sample(FSampleInput(X.GetData(), Y.GetData(), Z.GetData(), X.Num(), Footprint), D.GetData());
                Samples += X.Num();
                for (int32 i = 0; i < Slots.Num(); i++)
                    Store(Slots[i], D[i]);
            }
        }
    };
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

bool VoxelBrickMesher::IsBrickProvablyEmpty(const FDensitySampleCompositor& Density, const FVoxelBrickGrid& Grid,
    const FInt64Vector& Brick, double SlopeBound, int32& InOutSamples)
{
    FVoxelBrickGrid BrickGrid = Grid;
    BrickGrid.CellsPerBrick = FMath::Max(8, Grid.CellsPerBrick / 4 * 4);
    const int8 Levels[27] = {};
    const FStitcher Stitcher(Density, BrickGrid, Brick, Levels, 0u);
    return IsStitchedProvablyEmpty(Density, Stitcher.BrickMin(), BrickGrid.CellsPerBrick, Grid.CellSize, FMath::Max(SlopeBound, 1.0), InOutSamples);
}

void VoxelBrickMesher::MeshBrickStitched(const FDensitySampleCompositor& Density, const FVoxelBrickGrid& Grid,
    const FInt64Vector& Brick, const int8 (&NeighbourLevels)[27], uint32 MeshedNeighbours, double SlopeBound, bool bNoSkip,
    FVoxelBrickMesh& Out, FVoxelBrickStats& OutStats)
{
    Out.Positions.Reset();
    Out.Normals.Reset();
    Out.Indices.Reset();
    OutStats = FVoxelBrickStats();

    FVoxelBrickGrid BrickGrid = Grid;
    BrickGrid.CellsPerBrick = FMath::Max(8, Grid.CellsPerBrick / 4 * 4);
    const int32 N = BrickGrid.CellsPerBrick;
    FStitcher Stitcher(Density, BrickGrid, Brick, NeighbourLevels, MeshedNeighbours);

    if (!bNoSkip && IsStitchedProvablyEmpty(Density, Stitcher.BrickMin(), N, Grid.CellSize, FMath::Max(SlopeBound, 1.0), OutStats.Samples))
    {
        OutStats.bSkipped = true;
        return;
    }
    Stitcher.Prefetch();

    // 1. Minimal edges to emit. Candidates are the edges of this level touching the brick
    //    (lower end at cells 0..N-1 along the edge, 0..N across it). Each minimal edge is
    //    emitted once, by the meshed brick of its level that is last (z, then y, then x)
    //    among the bricks holding its cells of that level.
    TArray<FStitchFace> Faces;
    TArray<FStitchCell> Cells;
    TMap<FIntVector4, int32> CellIndex;
    float MaxGradient = 0.0f;
    for (int32 Axis = 0; Axis < 3; Axis++)
    {
        const int32 A = PerpA[Axis], B = PerpB[Axis];
        for (int32 K = 0; K <= N; K++)
            for (int32 J = 0; J <= N; J++)
                for (int32 I = 0; I <= N; I++)
                {
                    const FIntVector Lo(2 * I, 2 * J, 2 * K);
                    if (Lo[Axis] == 2 * N) continue;
                    const FStitchEdge Edge{ Lo, Axis, 2 };
                    if (!Stitcher.Crosses(Edge)) continue;

                    // The four cells around the edge: (+A,+B), (-A,+B), (-A,-B), (+A,-B).
                    FStitchCell Around[4];
                    bool bMinimal = true;
                    FIntVector Owner(-2, -2, -2);
                    for (int32 c = 0; c < 4 && bMinimal; c++)
                    {
                        FVector Probe(Lo);
                        Probe[Axis] += 1.0;
                        Probe[A] += (c == 1 || c == 2) ? -ProbeOffset : ProbeOffset;
                        Probe[B] += (c >= 2) ? -ProbeOffset : ProbeOffset;
                        Around[c] = Stitcher.CellAt(Probe);
                        bMinimal = Around[c].Level >= 0;
                        const FIntVector Block = Stitcher.BlockAt(Around[c].Min);
                        const bool bLater = Block.Z > Owner.Z || (Block.Z == Owner.Z && (Block.Y > Owner.Y || (Block.Y == Owner.Y && Block.X > Owner.X)));
                        if (Around[c].Level == 0 && bLater && Stitcher.IsMeshed(Block))
                            Owner = Block;
                    }
                    if (!bMinimal || Owner != FIntVector(0, 0, 0)) continue;

                    MaxGradient = FMath::Max(MaxGradient, (float)Stitcher.Gradient(Lo, 2).Size());

                    FStitchFace& Face = Faces.Emplace_GetRef();
                    Face.bFlip = Stitcher.Sample(Lo) < 0.0f;
                    for (int32 c = 0; c < 4; c++)
                    {
                        const FIntVector4 Key(Around[c].Min.X, Around[c].Min.Y, Around[c].Min.Z, Around[c].Level);
                        if (const int32* Found = CellIndex.Find(Key))
                        {
                            Face.Cells[c] = *Found;
                        }
                        else
                        {
                            Face.Cells[c] = Cells.Add(Around[c]);
                            CellIndex.Add(Key, Face.Cells[c]);
                        }
                    }
                }
    }
    OutStats.MaxGradient = MaxGradient;
    if (Faces.Num() == 0)
    {
        OutStats.Samples += Stitcher.Samples;
        return;
    }

    // 2. The cells' edges, then the samples they need off the prefetched lattice in two
    //    batches: edge ends, then gradients at the ends of crossed edges.
    TArray<FStitchEdge> Edges;
    TArray<int32> EdgeStart;
    EdgeStart.SetNumUninitialized(Cells.Num() + 1);
    for (int32 c = 0; c < Cells.Num(); c++)
    {
        EdgeStart[c] = Edges.Num();
        Stitcher.CellEdges(Cells[c], Edges);
    }
    EdgeStart[Cells.Num()] = Edges.Num();

    for (const FStitchEdge& Edge : Edges)
    {
        Stitcher.Request(Edge.Lo);
        Stitcher.Request(FStitcher::EdgeEnd(Edge));
    }
    Stitcher.Flush();
    for (const FStitchEdge& Edge : Edges)
    {
        if (!Stitcher.Crosses(Edge)) continue;
        Stitcher.RequestGradient(Edge.Lo, Edge.Length);
        Stitcher.RequestGradient(FStitcher::EdgeEnd(Edge), Edge.Length);
    }
    Stitcher.Flush();
    OutStats.Samples += Stitcher.Samples;

    // 3. Vertices, then faces. Where a coarser cell covers two neighbouring quadrants of an
    //    edge, the quad collapses to a triangle.
    TArray<int32> CellVertex;
    CellVertex.SetNumUninitialized(Cells.Num());
    for (int32 c = 0; c < Cells.Num(); c++)
        CellVertex[c] = Stitcher.SolveVertex(Cells[c], Edges.GetData() + EdgeStart[c], EdgeStart[c + 1] - EdgeStart[c], Out);

    for (const FStitchFace& Face : Faces)
    {
        int32 Ring[4];
        int32 Count = 0;
        bool bValid = true;
        for (int32 c = 0; c < 4; c++)
        {
            const int32 V = CellVertex[Face.Cells[c]];
            bValid &= V >= 0;
            if (Count == 0 || Ring[Count - 1] != V)
                Ring[Count++] = V;
        }
        if (Count > 1 && Ring[Count - 1] == Ring[0]) Count--;
        if (!bValid || Count < 3) continue;

        auto Emit = [&Out, &Face](int32 VA, int32 VB, int32 VC)
            {
                Out.Indices.Add(VA);
                Out.Indices.Add(Face.bFlip ? VC : VB);
                Out.Indices.Add(Face.bFlip ? VB : VC);
            };
        if (Count == 3)
        {
            Emit(Ring[0], Ring[1], Ring[2]);
            continue;
        }

        // Same split choice and winding as FAdaptiveOctree::UpdateMeshChunkStreamData.
        const FVector P0(Out.Positions[Ring[0]]), P1(Out.Positions[Ring[1]]);
        const FVector P2(Out.Positions[Ring[2]]), P3(Out.Positions[Ring[3]]);
        const double FoldA = FVector::DotProduct(((P1 - P0) ^ (P2 - P0)).GetSafeNormal(), ((P2 - P0) ^ (P3 - P0)).GetSafeNormal());
        const double FoldB = FVector::DotProduct(((P1 - P0) ^ (P3 - P0)).GetSafeNormal(), ((P2 - P1) ^ (P3 - P1)).GetSafeNormal());
        if (FoldB > FoldA)
        {
            Emit(Ring[0], Ring[1], Ring[3]);
            Emit(Ring[1], Ring[2], Ring[3]);
        }
        else
        {
            Emit(Ring[0], Ring[1], Ring[2]);
            Emit(Ring[0], Ring[2], Ring[3]);
        }
    }
}
