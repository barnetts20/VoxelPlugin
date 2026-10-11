// VoxelEditCore.h - The terrain edits as shape operations (Design/EditsAndCaves.md, E1): plain
// C++ without engine types, shared by the edit store (FSparseEditStore), the GPU data builder
// (FVoxelEditGPUData, VoxelEditSample.ush on the GPU) and the harness
// (Tools/EditHarness/edit_harness.cpp).
//
// An edit carves or fills a capsule (a sphere when its two centres coincide) into the density as
// it stands, with a blend radius k:
//   Carve:  d = smax(d, -s(p), k)        Fill:  d = smin(d, s(p), k)
// s is the capsule's signed distance (negative inside), the density is positive in air (cm), and
// smin/smax are the quadratic polynomial smooth min/max. They mix the two gradients with convex
// weights, so the result is never steeper than the steeper of the density and the shape (1): the
// brick tree's slope-bound empty test holds in edited regions.
//
// Edits apply in order (a fill after a carve refills it). A sample looks up its 8 m cell of a
// sparse index (cells hold the edits whose reach touches them, in order) and applies those.
// That gives the full field's sign everywhere (an edit changes the sign only within its reach)
// and its value near every surface, but not its value deep inside solid or air: a carve raises
// the density of the solid around it as far as it is closer than the original surface, and
// cells away from the carve don't apply it. Tests that prove a region free of surface from one
// sample (the brick trees' slope-bound tests) use ApplyWithin, which applies every edit
// reaching the region: that field is slope-bounded and has the full field's sign there.
// Below a footprint F (cm), an edit narrower than F shrinks by max(0, F - r): it closes smoothly
// at coarse levels of detail instead of aliasing (a constant offset: the slope bound holds).
//
// Compaction keeps the list small (Insert):
//   - stroke continuation: a held brush extends one capsule while it moves straight (up to two
//     radii long), adds a segment where it turns or the capsule is full, and adds nothing while
//     it stays put;
//   - repetition: an edit equal to an earlier one of the same mode, with nothing of the other
//     mode after that one overlapping it, adds nothing (max is idempotent);
//   - containment: an earlier edit lying, with its blend reach, inside a later edit's core
//     (shrunk by that one's blend and two cells) no longer reaches the surface, whatever the
//     modes and whatever lies between: it is dropped.
// With hard edges (no blend) these leave the surface exactly as the full list would; with
// blends, the full list would round a repeated edit's rim again each time, which the compacted
// list doesn't (Tools/EditHarness measures both).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace VoxelEditCore
{
    enum class EMode : uint8_t
    {
        Carve = 0,   // air inside the shape
        Fill = 1     // solid inside the shape
    };

    /** One edit: a capsule from A to B (a sphere when A == B), in the store's frame (cm). */
    struct FOp
    {
        uint32_t Sequence = 0;
        EMode Mode = EMode::Carve;
        double A[3] = { 0, 0, 0 };
        double B[3] = { 0, 0, 0 };
        double Radius = 0.0;
        double Blend = 0.0;          // k (cm)
    };

    /** How far past its shape an edit can move the surface: the quadratic smooth max changes the
     *  sign of the density only within 1.25 k of the shape. */
    inline double Reach(const FOp& Op) { return Op.Radius + 1.25 * Op.Blend; }

    /** A blend rounded up to a power of two (cm): the state's MaxBlend steps by these. */
    inline double BlendStep(double Blend)
    {
        return Blend > 0.0 ? std::exp2(std::ceil(std::log2(Blend))) : 0.0;
    }

    /** How far past its shape an edit can matter to the field's sign among other edits whose
     *  largest blend is MaxBlend: a later edit's blend adds up to k/4 to whatever the earlier
     *  ones left, so an earlier edit within that of the surface still decides its sign. */
    inline double ReachAmong(const FOp& Op, double MaxBlend) { return Op.Radius + (std::max)(1.25 * Op.Blend, 0.25 * MaxBlend); }

    struct FAabb
    {
        double Min[3] = { 0, 0, 0 };
        double Max[3] = { 0, 0, 0 };
        bool bValid = false;

        void Add(const FAabb& Other)
        {
            if (!Other.bValid) return;
            for (int a = 0; a < 3; a++)
            {
                Min[a] = bValid ? (std::min)(Min[a], Other.Min[a]) : Other.Min[a];
                Max[a] = bValid ? (std::max)(Max[a], Other.Max[a]) : Other.Max[a];
            }
            bValid = true;
        }
        bool Intersects(const FAabb& Other) const
        {
            if (!bValid || !Other.bValid) return false;
            for (int a = 0; a < 3; a++)
                if (Min[a] > Other.Max[a] || Max[a] < Other.Min[a]) return false;
            return true;
        }
    };

    inline FAabb BoundsOf(const FOp& Op, double MaxBlend = 0.0)
    {
        FAabb Box;
        const double R = ReachAmong(Op, MaxBlend);
        for (int a = 0; a < 3; a++)
        {
            Box.Min[a] = (std::min)(Op.A[a], Op.B[a]) - R;
            Box.Max[a] = (std::max)(Op.A[a], Op.B[a]) + R;
        }
        Box.bValid = true;
        return Box;
    }

    /** Distance from P to the segment A-B. */
    inline double SegmentDistance(const double P[3], const double A[3], const double B[3])
    {
        double PA[3], BA[3];
        double Dot = 0.0, Len2 = 0.0;
        for (int a = 0; a < 3; a++)
        {
            PA[a] = P[a] - A[a];
            BA[a] = B[a] - A[a];
            Dot += PA[a] * BA[a];
            Len2 += BA[a] * BA[a];
        }
        const double H = Len2 > 0.0 ? (std::clamp)(Dot / Len2, 0.0, 1.0) : 0.0;
        double D2 = 0.0;
        for (int a = 0; a < 3; a++)
        {
            const double V = PA[a] - BA[a] * H;
            D2 += V * V;
        }
        return std::sqrt(D2);
    }

    /** The quadratic polynomial smooth min (k 0: min). */
    inline double SmoothMin(double X, double Y, double K)
    {
        if (!(K > 0.0)) return (std::min)(X, Y);
        const double H = (std::max)(K - std::fabs(X - Y), 0.0) / K;
        return (std::min)(X, Y) - H * H * K * 0.25;
    }

    /** One edit applied to density D at P, for a footprint (cm; 0: exact). */
    inline double ApplyOp(const FOp& Op, const double P[3], double Footprint, double D)
    {
        const double S = SegmentDistance(P, Op.A, Op.B) - Op.Radius + (std::max)(0.0, Footprint - Op.Radius);
        return Op.Mode == EMode::Carve ? -SmoothMin(-D, S, Op.Blend) : SmoothMin(D, S, Op.Blend);
    }

    /** Outer's core (its shape shrunk by its blend and Margin) holds Inner's whole reach. The
     *  margin keeps Inner's effect off Outer's surface: there the two lists' values (not their
     *  signs) would differ, and the mesh interpolates the values. */
    inline bool Contains(const FOp& Outer, const FOp& Inner, double Margin, double MaxBlend)
    {
        const double Room = Outer.Radius - Outer.Blend - Margin - ReachAmong(Inner, MaxBlend);
        if (Room < 0.0) return false;
        // The distance to Outer's segment is convex along Inner's: the ends bound it.
        return SegmentDistance(Inner.A, Outer.A, Outer.B) <= Room && SegmentDistance(Inner.B, Outer.A, Outer.B) <= Room;
    }

    /** Same mode, radius and blend, both centres within Tolerance. */
    inline bool SameShape(const FOp& X, const FOp& Y, double Tolerance)
    {
        if (X.Mode != Y.Mode || std::fabs(X.Radius - Y.Radius) > 1e-6 * (1.0 + X.Radius) || std::fabs(X.Blend - Y.Blend) > 1e-6 * (1.0 + X.Blend))
            return false;
        double DA = 0.0, DB = 0.0;
        for (int a = 0; a < 3; a++)
        {
            DA += (X.A[a] - Y.A[a]) * (X.A[a] - Y.A[a]);
            DB += (X.B[a] - Y.B[a]) * (X.B[a] - Y.B[a]);
        }
        return DA <= Tolerance * Tolerance && DB <= Tolerance * Tolerance;
    }

    /** An index cell (cells are 2^CellShift lattice points a side; lattice indices are 32-bit,
     *  so cell coordinates are too). */
    struct FCell
    {
        int32_t X = 0;
        int32_t Y = 0;
        int32_t Z = 0;
        bool operator==(const FCell& Other) const { return X == Other.X && Y == Other.Y && Z == Other.Z; }
        bool operator<(const FCell& Other) const { return X != Other.X ? X < Other.X : (Y != Other.Y ? Y < Other.Y : Z < Other.Z); }
    };

    inline uint32_t CellHash(int32_t X, int32_t Y, int32_t Z)
    {
        return ((uint32_t)X * 73856093u) ^ ((uint32_t)Y * 19349663u) ^ ((uint32_t)Z * 83492791u);
    }

    struct FCellHasher
    {
        size_t operator()(const FCell& C) const { return (size_t)CellHash(C.X, C.Y, C.Z); }
    };

    inline FCell MakeCell(int64_t X, int64_t Y, int64_t Z)
    {
        FCell C;
        C.X = (int32_t)X;
        C.Y = (int32_t)Y;
        C.Z = (int32_t)Z;
        return C;
    }

    /** The edits and their index: immutable once published (copy-on-write in the store). */
    struct FState
    {
        double HalfFinest = 1.0;                 // the lattice's half cell (cm)
        int32_t CellShift = 6;                   // index cells of 2^CellShift half cells
        std::vector<FOp> Ops;                    // in order
        std::unordered_map<FCell, std::vector<uint32_t>, FCellHasher> Cells;   // cell -> op indices, ascending
        FAabb Bounds;                             // every op's reach (Reindex)
        double MaxBlend = 0.0;                    // at least every op's blend: a power of two (cm) that
                                                  // only grows (Insert), so reaches change rarely
        uint32_t LastSequence = 0;

        // The stroke a continued edit extends: the op it last changed (-1: none) and the last
        // brush centre.
        int32_t StrokeOp = -1;
        double StrokeEnd[3] = { 0, 0, 0 };

        double CellSize() const { return HalfFinest * (double)(1ll << CellShift); }

        int64_t CellOf(double V) const { return (int64_t)std::floor(V / CellSize()); }

        /** The ops a point's cell lists (null: none). */
        const std::vector<uint32_t>* ListAt(const double P[3]) const
        {
            if (Ops.empty()) return nullptr;
            const auto It = Cells.find(MakeCell(CellOf(P[0]), CellOf(P[1]), CellOf(P[2])));
            return It == Cells.end() ? nullptr : &It->second;
        }

        double Apply(const double P[3], double Footprint, double D) const
        {
            const std::vector<uint32_t>* List = ListAt(P);
            if (!List) return D;
            for (const uint32_t Index : *List)
                D = ApplyOp(Ops[Index], P, Footprint, D);
            return D;
        }

        /** Density D at P with every op whose reach comes within Radius of P applied, in order
         *  (not just P's cell's): the field that decides whether a ball of that radius about P
         *  holds surface. Within the ball its sign is the full field's (an op changes the sign
         *  only within its reach), and it is as slope-bounded as the full field, so
         *  |result| > L * Radius proves the ball free of surface. Evaluating P's cell alone
         *  would not: a carve deeper than its distance raises the density of the solid around
         *  it (max(d, -s)), far past the cells that list it. */
        double ApplyWithin(const double P[3], double Radius, double Footprint, double D) const
        {
            if (Ops.empty()) return D;
            FAabb Ball;
            for (int a = 0; a < 3; a++)
            {
                Ball.Min[a] = P[a] - Radius;
                Ball.Max[a] = P[a] + Radius;
            }
            Ball.bValid = true;
            if (!Bounds.Intersects(Ball)) return D;
            // The ops of the cells the ball overlaps (an op reaching a point is listed in its
            // cell), or every op when that's fewer to visit. (A scratch list a thread: this runs
            // per sample in the brick trees' descent.)
            thread_local std::vector<uint32_t> Found;
            if (!CollectInBox(Ball, Found))
            {
                Found.resize(Ops.size());
                for (uint32_t i = 0; i < (uint32_t)Ops.size(); i++) Found[i] = i;
            }
            for (const uint32_t Index : Found)
            {
                const FOp& Op = Ops[Index];
                if (SegmentDistance(P, Op.A, Op.B) <= ReachAmong(Op, MaxBlend) + Radius)
                    D = ApplyOp(Op, P, Footprint, D);
            }
            return D;
        }

        /** The ops listed in the cells Box overlaps, ascending and unique; false (Out empty)
         *  when that is more cells than ops (visit every op instead). */
        bool CollectInBox(const FAabb& Box, std::vector<uint32_t>& Out) const
        {
            Out.clear();
            // Measured before any conversion: a huge box must not overflow the cell indices.
            double Span = 1.0;
            for (int a = 0; a < 3; a++)
                Span *= (Box.Max[a] - Box.Min[a]) / CellSize() + 2.0;
            if (!(Span <= 4096.0)) return false;
            const int64_t Lo[3] = { CellOf(Box.Min[0]), CellOf(Box.Min[1]), CellOf(Box.Min[2]) };
            const int64_t Hi[3] = { CellOf(Box.Max[0]), CellOf(Box.Max[1]), CellOf(Box.Max[2]) };
            const double Count = (double)(Hi[0] - Lo[0] + 1) * (double)(Hi[1] - Lo[1] + 1) * (double)(Hi[2] - Lo[2] + 1);
            if (Count > (double)(std::max)(Cells.size(), Ops.size()) || Count > 4096.0) return false;
            for (int64_t Z = Lo[2]; Z <= Hi[2]; Z++)
                for (int64_t Y = Lo[1]; Y <= Hi[1]; Y++)
                    for (int64_t X = Lo[0]; X <= Hi[0]; X++)
                    {
                        const auto It = Cells.find(MakeCell(X, Y, Z));
                        if (It != Cells.end())
                            Out.insert(Out.end(), It->second.begin(), It->second.end());
                    }
            std::sort(Out.begin(), Out.end());
            Out.erase(std::unique(Out.begin(), Out.end()), Out.end());
            return true;
        }

        bool AnyInBox(const FAabb& Box) const
        {
            if (Ops.empty() || !Box.bValid || !Bounds.Intersects(Box)) return false;
            thread_local std::vector<uint32_t> Found;
            if (CollectInBox(Box, Found))
            {
                for (const uint32_t Index : Found)
                    if (BoundsOf(Ops[Index], MaxBlend).Intersects(Box)) return true;
                return false;
            }
            for (const FOp& Op : Ops)
                if (BoundsOf(Op, MaxBlend).Intersects(Box)) return true;
            return false;
        }

        /** Rebuilds the index from Ops. A cell lists an op when the cell's centre is within the
         *  op's reach plus the cell's half diagonal of its segment. */
        void Reindex()
        {
            Cells.clear();
            Bounds = FAabb();
            for (const FOp& Op : Ops)
                MaxBlend = (std::max)(MaxBlend, BlendStep(Op.Blend));
            for (const FOp& Op : Ops)
                Bounds.Add(BoundsOf(Op, MaxBlend));
            const double Size = CellSize();
            const double HalfDiagonal = 0.5 * Size * std::sqrt(3.0);
            for (uint32_t i = 0; i < (uint32_t)Ops.size(); i++)
            {
                const FOp& Op = Ops[i];
                const FAabb Box = BoundsOf(Op, MaxBlend);
                const int64_t Lo[3] = { CellOf(Box.Min[0]), CellOf(Box.Min[1]), CellOf(Box.Min[2]) };
                const int64_t Hi[3] = { CellOf(Box.Max[0]), CellOf(Box.Max[1]), CellOf(Box.Max[2]) };
                const double Limit = ReachAmong(Op, MaxBlend) + HalfDiagonal;
                for (int64_t Z = Lo[2]; Z <= Hi[2]; Z++)
                    for (int64_t Y = Lo[1]; Y <= Hi[1]; Y++)
                        for (int64_t X = Lo[0]; X <= Hi[0]; X++)
                        {
                            const double C[3] = { ((double)X + 0.5) * Size, ((double)Y + 0.5) * Size, ((double)Z + 0.5) * Size };
                            if (SegmentDistance(C, Op.A, Op.B) <= Limit)
                                Cells[MakeCell(X, Y, Z)].push_back(i);
                        }
            }
        }
    };

    struct FInsertResult
    {
        bool bChanged = false;       // the field changed (a new or extended op)
        FAabb Dirty;                  // where the surface may have moved
        int32_t Dropped = 0;         // earlier ops removed by containment
    };

    struct FInsertOptions
    {
        bool bCompact = true;                    // false: append only (the harness's reference)
        double StrokeTolerance = 0.02;           // of the radius: straight-line and stay-put tolerance
        double MaxSegment = 2.0;                 // of the radius: a stroke's capsule grows no longer
                                                 // (an extension rewrites its whole length's surface)
    };

    /** The state after one more edit (a sphere at Centre), continuing the previous call's stroke
     *  when bContinue. */
    inline FState Insert(const FState& Old, EMode Mode, const double Centre[3], double Radius, double Blend, bool bContinue,
        const FInsertOptions& Options, FInsertResult& Out)
    {
        // The edits and the stroke, without the index (rebuilt below).
        FState S;
        S.HalfFinest = Old.HalfFinest;
        S.CellShift = Old.CellShift;
        S.Ops = Old.Ops;
        S.LastSequence = Old.LastSequence;
        S.MaxBlend = Old.MaxBlend;
        S.StrokeOp = Old.StrokeOp;
        for (int a = 0; a < 3; a++) S.StrokeEnd[a] = Old.StrokeEnd[a];
        Out = FInsertResult();
        Radius = (std::max)(Radius, 0.0);
        Blend = (std::clamp)(Blend, 0.0, Radius);
        const double Tolerance = Options.StrokeTolerance * Radius;

        FOp New;
        New.Mode = Mode;
        New.Radius = Radius;
        New.Blend = Blend;
        for (int a = 0; a < 3; a++) New.A[a] = New.B[a] = Centre[a];

        int32_t Changed = -1;   // the op added or extended
        const bool bStroke = Options.bCompact && bContinue && S.StrokeOp >= 0 && S.StrokeOp == (int32_t)S.Ops.size() - 1;
        if (bStroke)
        {
            FOp& Last = S.Ops[S.StrokeOp];
            double Move2 = 0.0;
            for (int a = 0; a < 3; a++) Move2 += (Centre[a] - S.StrokeEnd[a]) * (Centre[a] - S.StrokeEnd[a]);
            // A jump (the brush swept to another spot) breaks the stroke: a segment longer than
            // a capsule may be would carve or fill straight through what lies between.
            const double MaxLength = Options.MaxSegment * Radius;
            const bool bSameBrush = Last.Mode == Mode && std::fabs(Last.Radius - Radius) <= 1e-6 * (1.0 + Radius) && std::fabs(Last.Blend - Blend) <= 1e-6 * (1.0 + Blend)
                && Move2 <= MaxLength * MaxLength;
            if (bSameBrush && Move2 <= Tolerance * Tolerance)
            {
                // Stayed put: the brush already covers it.
                S.Cells = Old.Cells;
                S.Bounds = Old.Bounds;
                S.MaxBlend = Old.MaxBlend;
                return S;
            }
            if (bSameBrush)
            {
                double LastLen2 = 0.0;
                for (int a = 0; a < 3; a++) LastLen2 += (Last.B[a] - Last.A[a]) * (Last.B[a] - Last.A[a]);
                if (LastLen2 <= Tolerance * Tolerance)
                {
                    // A sphere becomes the capsule to the new centre (it holds the sphere).
                    for (int a = 0; a < 3; a++) Last.B[a] = Centre[a];
                    Changed = S.StrokeOp;
                }
                else
                {
                    // Straight on (the old end within tolerance of the new axis, and ahead): the
                    // capsule extends; otherwise a new segment from the old end.
                    double Ahead = 0.0, Length2 = 0.0;
                    for (int a = 0; a < 3; a++)
                    {
                        Ahead += (Centre[a] - Last.B[a]) * (Last.B[a] - Last.A[a]);
                        Length2 += (Centre[a] - Last.A[a]) * (Centre[a] - Last.A[a]);
                    }
                    if (Ahead > 0.0 && Length2 <= MaxLength * MaxLength && SegmentDistance(Last.B, Last.A, Centre) <= Tolerance)
                    {
                        for (int a = 0; a < 3; a++) Last.B[a] = Centre[a];
                        Changed = S.StrokeOp;
                    }
                }
                if (Changed < 0)
                {
                    for (int a = 0; a < 3; a++) New.A[a] = S.StrokeEnd[a];
                }
            }
        }

        if (Changed < 0 && Options.bCompact)
        {
            // Repetition: the same edit as an earlier one, with no op of the other mode after it
            // overlapping it, changes nothing.
            const double Blends = (std::max)(Old.MaxBlend, Blend);
            const FAabb NewBox = BoundsOf(New, Blends);
            for (int32_t i = (int32_t)S.Ops.size() - 1; i >= 0; i--)
            {
                const FOp& Op = S.Ops[i];
                if (Op.Mode != Mode && BoundsOf(Op, Blends).Intersects(NewBox)) break;
                if (SameShape(Op, New, Tolerance))
                {
                    S.StrokeOp = i == (int32_t)S.Ops.size() - 1 ? i : -1;
                    for (int a = 0; a < 3; a++) S.StrokeEnd[a] = Centre[a];
                    S.Cells = Old.Cells;
                S.Bounds = Old.Bounds;
                S.MaxBlend = Old.MaxBlend;
                    return S;
                }
            }
        }

        if (Changed >= 0)
        {
            // The extended op's old bounds lie inside its new ones.
            S.Ops[Changed].Sequence = S.LastSequence + 1;
        }
        else
        {
            New.Sequence = S.LastSequence + 1;
            S.Ops.push_back(New);
            Changed = (int32_t)S.Ops.size() - 1;
        }
        S.LastSequence++;
        const FOp Latest = S.Ops[Changed];
        Out.bChanged = true;

        // Containment: earlier ops inside the latest's core no longer reach the surface.
        if (Options.bCompact)
        {
            std::vector<FOp> Kept;
            Kept.reserve(S.Ops.size());
            for (int32_t i = 0; i < (int32_t)S.Ops.size(); i++)
            {
                if (i != Changed && Contains(Latest, S.Ops[i], 4.0 * S.HalfFinest, (std::max)(Old.MaxBlend, Blend)))
                {
                    Out.Dropped++;
                    continue;
                }
                Kept.push_back(S.Ops[i]);
            }
            if (Out.Dropped > 0)
                S.Ops.swap(Kept);
        }
        S.StrokeOp = (int32_t)S.Ops.size() - 1;
        for (int a = 0; a < 3; a++) S.StrokeEnd[a] = Centre[a];
        S.Reindex();
        // A larger blend than any before (rare: MaxBlend only grows, in powers of two) widens
        // every op's reach: their cells change too.
        Out.Dirty = BoundsOf(Latest, S.MaxBlend);
        if (S.MaxBlend > Old.MaxBlend)
            Out.Dirty.Add(S.Bounds);
        return S;
    }

    /** The edits as VoxelEditSample.ush reads them. */
    struct FGPUArrays
    {
        std::vector<uint32_t> Cells;     // 4 per slot: x, y, z, (offset << 12) | count
        std::vector<uint32_t> OpList;
        std::vector<uint32_t> Ops;       // 16 per op (floats as their bits, the anchor as ints)
        uint32_t TableMask = 0;
        uint32_t CellShift = 0;
        uint32_t OpCount = 0;
        int32_t Truncated = 0;           // cells holding more than 4095 ops (the rest dropped)
    };

    constexpr uint32_t GPUOpStride = 16;
    constexpr uint32_t GPUCountBits = 12;

    inline void Flatten(const FState& S, FGPUArrays& Out)
    {
        Out = FGPUArrays();
        Out.CellShift = (uint32_t)S.CellShift;
        Out.OpCount = (uint32_t)S.Ops.size();
        if (S.Ops.empty())
        {
            // One empty slot and one op's worth of zeros: the buffers are never empty.
            Out.Cells.assign(4, 0u);
            Out.OpList.assign(1, 0u);
            Out.Ops.assign(GPUOpStride, 0u);
            return;
        }

        const double H = S.HalfFinest;
        auto Bits = [](float V) { uint32_t U; std::memcpy(&U, &V, 4); return U; };
        Out.Ops.assign((size_t)S.Ops.size() * GPUOpStride, 0u);
        for (size_t i = 0; i < S.Ops.size(); i++)
        {
            const FOp& Op = S.Ops[i];
            uint32_t* O = &Out.Ops[i * GPUOpStride];
            for (int a = 0; a < 3; a++)
            {
                const int32_t Anchor = (int32_t)std::llround(Op.A[a] / H);
                O[a] = (uint32_t)Anchor;
                O[4 + a] = Bits((float)(Op.A[a] - (double)Anchor * H));
                O[8 + a] = Bits((float)(Op.B[a] - (double)Anchor * H));
            }
            O[3] = Op.Mode == EMode::Fill ? 1u : 0u;
            O[7] = Bits((float)Op.Radius);
            O[11] = Bits((float)Op.Blend);
        }

        uint32_t Slots = 16;
        while (Slots < 2 * (uint32_t)S.Cells.size()) Slots *= 2;
        Out.TableMask = Slots - 1;
        Out.Cells.assign((size_t)Slots * 4, 0u);
        // Cells in a fixed order, so the same state flattens to the same buffers.
        std::vector<FCell> Keys;
        Keys.reserve(S.Cells.size());
        for (const auto& Pair : S.Cells) Keys.push_back(Pair.first);
        std::sort(Keys.begin(), Keys.end());
        for (const FCell& Key : Keys)
        {
            const std::vector<uint32_t>& List = S.Cells.at(Key);
            const int32_t X = Key.X, Y = Key.Y, Z = Key.Z;
            uint32_t Count = (uint32_t)List.size();
            const uint32_t MaxCount = (1u << GPUCountBits) - 1u;
            if (Count > MaxCount)
            {
                Count = MaxCount;
                Out.Truncated++;
            }
            const uint32_t First = (uint32_t)Out.OpList.size();
            if (First >= (1u << (32u - GPUCountBits)))
            {
                // The list's offsets don't fit: the cell holds no edits on the GPU.
                Out.Truncated++;
                continue;
            }
            Out.OpList.insert(Out.OpList.end(), List.begin(), List.begin() + Count);
            uint32_t Slot = CellHash(X, Y, Z) & Out.TableMask;
            while (Out.Cells[(size_t)Slot * 4 + 3] != 0u)
                Slot = (Slot + 1) & Out.TableMask;
            Out.Cells[(size_t)Slot * 4 + 0] = (uint32_t)X;
            Out.Cells[(size_t)Slot * 4 + 1] = (uint32_t)Y;
            Out.Cells[(size_t)Slot * 4 + 2] = (uint32_t)Z;
            Out.Cells[(size_t)Slot * 4 + 3] = (First << GPUCountBits) | Count;
        }
    }
}
