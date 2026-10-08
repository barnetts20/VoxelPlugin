// VoxelRangeAllocator.h - First-fit ranges of elements in a buffer, for the brick pages
// (VoxelBrickPages.cpp): the mesh cache's regions and bricks, and the vertices and triangles of
// each page buffer set. Free ranges are kept sorted by start and merged.

#pragma once

#include "CoreMinimal.h"

class FVoxelRangeAllocator
{
public:
    /** Everything free: Capacity elements. */
    void Reset(int32 InCapacity)
    {
        Capacity = FMath::Max(0, InCapacity);
        Free.Reset();
        NumFree = Capacity;
        if (Capacity > 0)
            Free.Add(FIntPoint(0, Capacity));
    }

    /** Adds free room at the end. */
    void Grow(int32 NewCapacity)
    {
        if (NewCapacity <= Capacity) return;
        const int32 Old = Capacity;
        Capacity = NewCapacity;
        Release(Old, NewCapacity - Old);
    }

    /** True when Allocate(Count) would succeed. */
    bool CanAllocate(int32 Count) const
    {
        if (Count <= 0) return true;
        for (const FIntPoint& Range : Free)
        {
            if (Range.Y >= Count) return true;
        }
        return false;
    }

    /** The start of Count elements at the lowest place they fit, or -1. Zero elements take
     *  no room (start 0, never released). */
    int32 Allocate(int32 Count)
    {
        if (Count <= 0) return 0;
        for (int32 i = 0; i < Free.Num(); i++)
        {
            FIntPoint& Range = Free[i];
            if (Range.Y < Count) continue;
            const int32 Start = Range.X;
            Range.X += Count;
            Range.Y -= Count;
            if (Range.Y == 0)
                Free.RemoveAt(i);
            NumFree -= Count;
            return Start;
        }
        return -1;
    }

    /** Frees Count elements from Start, allocated before. */
    void Release(int32 Start, int32 Count)
    {
        if (Count <= 0) return;
        // The first free range after Start.
        int32 Lo = 0;
        int32 Hi = Free.Num();
        while (Lo < Hi)
        {
            const int32 Mid = (Lo + Hi) / 2;
            if (Free[Mid].X < Start)
                Lo = Mid + 1;
            else
                Hi = Mid;
        }
        checkSlow(Lo == 0 || Free[Lo - 1].X + Free[Lo - 1].Y <= Start);
        checkSlow(Lo == Free.Num() || Start + Count <= Free[Lo].X);
        NumFree += Count;
        const bool bJoinPrev = Lo > 0 && Free[Lo - 1].X + Free[Lo - 1].Y == Start;
        const bool bJoinNext = Lo < Free.Num() && Start + Count == Free[Lo].X;
        if (bJoinPrev && bJoinNext)
        {
            Free[Lo - 1].Y += Count + Free[Lo].Y;
            Free.RemoveAt(Lo);
        }
        else if (bJoinPrev)
        {
            Free[Lo - 1].Y += Count;
        }
        else if (bJoinNext)
        {
            Free[Lo].X = Start;
            Free[Lo].Y += Count;
        }
        else
        {
            Free.Insert(FIntPoint(Start, Count), Lo);
        }
    }

    /** One past the last allocated element, 0 when everything is free. */
    int32 End() const
    {
        if (Free.Num() > 0 && Free.Last().X + Free.Last().Y == Capacity)
            return Free.Last().X;
        return Capacity;
    }

    int32 GetCapacity() const { return Capacity; }
    int32 GetNumFree() const { return NumFree; }
    int32 GetNumUsed() const { return Capacity - NumFree; }
    const TArray<FIntPoint>& GetFree() const { return Free; }

    /** Ranges (start, count) less every element of Remove, and less everything from Limit on.
     *  Neither list needs to be sorted; Ranges comes back sorted. */
    static void Subtract(TArray<FIntPoint>& Ranges, TArray<FIntPoint> Remove, int32 Limit)
    {
        Ranges.Sort([](const FIntPoint& A, const FIntPoint& B) { return A.X < B.X; });
        Remove.Sort([](const FIntPoint& A, const FIntPoint& B) { return A.X < B.X; });
        TArray<FIntPoint> Out;
        int32 r = 0;
        for (const FIntPoint& Range : Ranges)
        {
            int32 Start = Range.X;
            const int32 Stop = FMath::Min(Range.X + Range.Y, Limit);
            while (r < Remove.Num() && Remove[r].X + Remove[r].Y <= Start)
                r++;
            for (int32 k = r; k < Remove.Num() && Start < Stop && Remove[k].X < Stop; k++)
            {
                if (Remove[k].X > Start)
                    Out.Add(FIntPoint(Start, Remove[k].X - Start));
                Start = FMath::Max(Start, Remove[k].X + Remove[k].Y);
            }
            if (Start < Stop)
                Out.Add(FIntPoint(Start, Stop - Start));
        }
        Ranges = MoveTemp(Out);
    }

private:
    TArray<FIntPoint> Free;   // X start, Y count: sorted, disjoint, never touching
    int32 Capacity = 0;
    int32 NumFree = 0;
};
