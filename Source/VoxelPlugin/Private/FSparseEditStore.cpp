#include "FSparseEditStore.h"
#include "VoxelEditCore.h"
#include "Async/ParallelFor.h"
#include "Misc/ScopeLock.h"

namespace
{
    using FStatePtr = TSharedPtr<const VoxelEditCore::FState, ESPMode::ThreadSafe>;

    FBox ToBox(const VoxelEditCore::FAabb& Box, const FVector& Center)
    {
        if (!Box.bValid) return FBox(ForceInit);
        return FBox(FVector(Box.Min[0], Box.Min[1], Box.Min[2]) + Center, FVector(Box.Max[0], Box.Max[1], Box.Max[2]) + Center);
    }

    /** Index cells: the power of two of half cells nearest 8 m. */
    int32 CellShiftFor(double HalfFinest)
    {
        const double Cells = 800.0 / FMath::Max(HalfFinest, 1e-6);
        return FMath::Clamp((int32)FMath::RoundToDouble(FMath::Log2(FMath::Max(Cells, 1.0))), 0, 20);
    }
}

FSparseEditStore::FSparseEditStore(FVector InCenter, double InExtent, int InMaxDepth)
    : Center(InCenter), Extent(InExtent), HalfFinest(InExtent / FMath::Pow(2.0, (double)FMath::Max(InMaxDepth, 0)))
{
    TSharedRef<VoxelEditCore::FState, ESPMode::ThreadSafe> Empty = MakeShared<VoxelEditCore::FState, ESPMode::ThreadSafe>();
    Empty->HalfFinest = HalfFinest;
    Empty->CellShift = CellShiftFor(HalfFinest);
    IndexCellSize = Empty->CellSize();
    State = Empty;
}

FSparseEditStore::~FSparseEditStore() = default;

FStatePtr FSparseEditStore::PinState() const
{
    FScopeLock Lock(&StateCS);
    return State;
}

FStatePtr FSparseEditStore::GetSnapshot() const
{
    return PinState();
}

void FSparseEditStore::Publish(FStatePtr NewState)
{
    const uint32 Sequence = NewState->LastSequence;
    {
        FScopeLock Lock(&StateCS);
        State = MoveTemp(NewState);
    }
    LastSequence.store(Sequence, std::memory_order_release);
}

int32 FSparseEditStore::Num() const
{
    const FStatePtr Pinned = PinState();
    return (int32)Pinned->Ops.size();
}

void FSparseEditStore::Apply(const double* X, const double* Y, const double* Z, int32 Count, float* DensityInOut, double Footprint) const
{
    if (Count <= 0) return;
    const FStatePtr Pinned = PinState();
    const VoxelEditCore::FState& S = *Pinned;
    if (S.Ops.empty()) return;
    auto One = [&](int32 i)
        {
            const double P[3] = { X[i] - Center.X, Y[i] - Center.Y, Z[i] - Center.Z };
            DensityInOut[i] = (float)S.Apply(P, Footprint, (double)DensityInOut[i]);
        };
    if (Count > 128)
        ParallelFor(Count, One);
    else
        for (int32 i = 0; i < Count; i++) One(i);
}

void FSparseEditStore::ApplyWithin(const double* X, const double* Y, const double* Z, int32 Count, float* DensityInOut, double Footprint, double Radius) const
{
    if (Count <= 0) return;
    const FStatePtr Pinned = PinState();
    const VoxelEditCore::FState& S = *Pinned;
    if (S.Ops.empty()) return;
    auto One = [&](int32 i)
        {
            const double P[3] = { X[i] - Center.X, Y[i] - Center.Y, Z[i] - Center.Z };
            DensityInOut[i] = (float)S.ApplyWithin(P, Radius, Footprint, (double)DensityInOut[i]);
        };
    if (Count > 128)
        ParallelFor(Count, One);
    else
        for (int32 i = 0; i < Count; i++) One(i);
}

bool FSparseEditStore::HasEditsInBox(const FBox& Box) const
{
    if (!Box.IsValid) return false;
    const FStatePtr Pinned = PinState();
    VoxelEditCore::FAabb Local;
    for (int32 a = 0; a < 3; a++)
    {
        Local.Min[a] = Box.Min[a] - Center[a];
        Local.Max[a] = Box.Max[a] - Center[a];
    }
    Local.bValid = true;
    return Pinned->AnyInBox(Local);
}

FVoxelEditResult FSparseEditStore::ApplyEdit(EVoxelEditMode Mode, const FVector& InCenter, double Radius, double Blend, bool bContinueStroke)
{
    FScopeLock WriteLock(&WriteCS);
    const FStatePtr Old = PinState();
    const double P[3] = { InCenter.X - Center.X, InCenter.Y - Center.Y, InCenter.Z - Center.Z };
    VoxelEditCore::FInsertResult Inserted;
    TSharedRef<VoxelEditCore::FState, ESPMode::ThreadSafe> New = MakeShared<VoxelEditCore::FState, ESPMode::ThreadSafe>(
        VoxelEditCore::Insert(*Old, Mode == EVoxelEditMode::Fill ? VoxelEditCore::EMode::Fill : VoxelEditCore::EMode::Carve, P, Radius, Blend,
            bContinueStroke, VoxelEditCore::FInsertOptions(), Inserted));

    FVoxelEditResult Result;
    Result.bChanged = Inserted.bChanged;
    Result.Dropped = Inserted.Dropped;
    Result.Bounds = ToBox(Inserted.Dirty, Center);
    // Unchanged edits still move the stroke on (the next continued call starts there); a
    // brush held still changes nothing at all, and publishes nothing.
    const bool bStrokeMoved = New->StrokeOp != Old->StrokeOp || New->StrokeEnd[0] != Old->StrokeEnd[0]
        || New->StrokeEnd[1] != Old->StrokeEnd[1] || New->StrokeEnd[2] != Old->StrokeEnd[2];
    if (Inserted.bChanged || bStrokeMoved)
        Publish(New);
    Result.Sequence = GetLastSequence();
    return Result;
}

TArray<FVoxelEditOp> FSparseEditStore::GetEditLog() const
{
    const FStatePtr Pinned = PinState();
    TArray<FVoxelEditOp> Out;
    Out.Reserve((int32)Pinned->Ops.size());
    for (const VoxelEditCore::FOp& Op : Pinned->Ops)
    {
        FVoxelEditOp& E = Out.AddDefaulted_GetRef();
        E.Sequence = Op.Sequence;
        E.Mode = Op.Mode == VoxelEditCore::EMode::Fill ? EVoxelEditMode::Fill : EVoxelEditMode::Carve;
        E.A = FVector(Op.A[0], Op.A[1], Op.A[2]) + Center;
        E.B = FVector(Op.B[0], Op.B[1], Op.B[2]) + Center;
        E.Radius = Op.Radius;
        E.Blend = Op.Blend;
    }
    return Out;
}

void FSparseEditStore::ReplayEditLog(const TArray<FVoxelEditOp>& Ops)
{
    FScopeLock WriteLock(&WriteCS);
    TSharedRef<VoxelEditCore::FState, ESPMode::ThreadSafe> New = MakeShared<VoxelEditCore::FState, ESPMode::ThreadSafe>();
    New->HalfFinest = HalfFinest;
    New->CellShift = CellShiftFor(HalfFinest);
    uint32 Last = GetLastSequence();
    for (const FVoxelEditOp& E : Ops)
    {
        VoxelEditCore::FOp Op;
        Op.Sequence = E.Sequence;
        Op.Mode = E.Mode == EVoxelEditMode::Fill ? VoxelEditCore::EMode::Fill : VoxelEditCore::EMode::Carve;
        for (int32 a = 0; a < 3; a++)
        {
            Op.A[a] = E.A[a] - Center[a];
            Op.B[a] = E.B[a] - Center[a];
        }
        Op.Radius = FMath::Max(E.Radius, 0.0);
        Op.Blend = FMath::Clamp(E.Blend, 0.0, Op.Radius);
        New->Ops.push_back(Op);
        Last = FMath::Max(Last, E.Sequence);
    }
    // A replay is a change even when the sequences are old ones: consumers rebuild.
    New->LastSequence = Last + 1;
    New->Reindex();
    Publish(New);
}

void FSparseEditStore::Clear()
{
    FScopeLock WriteLock(&WriteCS);
    TSharedRef<VoxelEditCore::FState, ESPMode::ThreadSafe> New = MakeShared<VoxelEditCore::FState, ESPMode::ThreadSafe>();
    New->HalfFinest = HalfFinest;
    New->CellShift = CellShiftFor(HalfFinest);
    New->LastSequence = GetLastSequence() + 1;
    Publish(New);
}
