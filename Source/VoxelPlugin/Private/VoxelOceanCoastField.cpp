#include "VoxelOceanCoastField.h"
#include "VoxelOceanShoreField.h"
#include "VoxelLandBake.h"
#include "VoxelPlugin.h"
#include "Async/Async.h"
#include "HAL/PlatformTime.h"

namespace VoxelOceanCoastFieldDetail
{
    /** Fine cells a GPU sampling chunk takes, and chunks in flight at once. */
    constexpr int32 ChunkCells = 1 << 20;
    constexpr int32 MaxInFlight = 2;
}

namespace VOCF = VoxelOceanCoastFieldDetail;

// ---------------------------------------------------------------------------
// Field
// ---------------------------------------------------------------------------

bool FVoxelOceanCoastField::IsValid() const
{
    const int64 Side = VoxelOceanShoreCore::Side(N);
    return N > 0 && R > 0 && Slices > 6 && (int64)Texels.size() == (int64)Slices * Side * Side && Shore.IsValid() && Shore->IsValid() && Shore->N == N;
}

VoxelShader::FVoxelOceanCoastSample FVoxelOceanCoastField::SampleCoast(const FVector& PlanetPoint) const
{
    if (!IsValid()) return VoxelShader::VoxelOceanCoastSampleNone();
    const float Row0[4] = { Rows[0].X, Rows[0].Y, Rows[0].Z, Rows[0].W };
    const float Row1[4] = { Rows[1].X, Rows[1].Y, Rows[1].Z, Rows[1].W };
    return VoxelOceanCoastCore::Sample(Shore->Texels.GetData(), Texels.data(), Slices, N, Row0, Row1, RadiusCm * 0.01, PlanetPoint.X, PlanetPoint.Y, PlanetPoint.Z);
}

FVoxelOceanShore FVoxelOceanCoastField::Sample(const FVector& PlanetPoint) const
{
    if (!Shore.IsValid()) return FVoxelOceanShore();
    FVoxelOceanShore Out = Shore->Sample(PlanetPoint);
    if (!IsValid() || !Out.HasField()) return Out;
    // The layout's (rows 19 and 20: valid, ..., cells; seed depth, reach), as the shaders'.
    Out.CoastSeedM = Rows[1].Z;
    Out.CoastReachM = Rows[1].W;
    if (Rows[0].X < 0.5f || Out.Distance * 0.01 >= (double)Rows[1].W + 2.0 * (double)Rows[1].Y) return Out;
    const VoxelShader::FVoxelOceanCoastSample C = SampleCoast(PlanetPoint);
    Out.CoastA = C.A;
    Out.CoastB = C.B;
    Out.CoastGradA = FVector(C.GradA.x, C.GradA.y, C.GradA.z);
    Out.CoastGradB = FVector(C.GradB.x, C.GradB.y, C.GradB.z);
    Out.CoastCover = C.Cover;
    return Out;
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

FVoxelOceanCoastBuilder::FVoxelOceanCoastBuilder() = default;
FVoxelOceanCoastBuilder::~FVoxelOceanCoastBuilder() = default;

void FVoxelOceanCoastBuilder::Reset()
{
    // A worker still running finishes into its own shared state, which nothing reads after this.
    Phase = EPhase::Idle;
    Shore.Reset();
    Bake.Reset();
    Plan.Reset();
    Planned = TFuture<void>();
    Job.Reset();
    Issued = 0;
    Solved = TFuture<TSharedPtr<FVoxelOceanCoastField, ESPMode::ThreadSafe>>();
    Result.Reset();
    bForThis = false;
}

bool FVoxelOceanCoastBuilder::IsFor(const FVoxelOceanShoreField* InShore, const VoxelOceanCoastCore::FSettings& InSettings) const
{
    return bForThis && Shore.Get() == InShore && Settings == InSettings;
}

void FVoxelOceanCoastBuilder::Start(const TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe>& InShore,
    const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& InBake, const VoxelOceanCoastCore::FSettings& InSettings)
{
    Reset();
    Shore = InShore;
    Bake = InBake;
    Settings = InSettings;
    bForThis = true;
    StartSeconds = FPlatformTime::Seconds();
    if (!Shore.IsValid() || !Shore->IsValid() || !Bake.IsValid() || Bake->N != Shore->N || Bake->Height.Num() != Bake->NumCells()
        || FVoxelLandBake::Pad != VoxelOceanCoastCore::Pad)
        return;
    Plan = MakeShared<VoxelOceanCoastCore::FPlan, ESPMode::ThreadSafe>();
    TSharedPtr<VoxelOceanCoastCore::FPlan, ESPMode::ThreadSafe> OutPlan = Plan;
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> InField = Shore;
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> InLand = Bake;
    const VoxelOceanCoastCore::FSettings S = Settings;
    Planned = Async(EAsyncExecution::ThreadPool, [OutPlan, InField, InLand, S]()
        {
            *OutPlan = VoxelOceanCoastCore::Plan(InField->N, InField->RadiusCm * 0.01, InField->Texels.GetData(), InField->ReachM, InLand->Height.GetData(), S);
        });
    Phase = EPhase::Planning;
}

void FVoxelOceanCoastBuilder::Tick(const FSampler& Sampler)
{
    if (Phase == EPhase::Planning)
    {
        if (!Planned.IsReady()) return;
        if (!Plan->bValid)
        {
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field: no coast within the shore field (or no tile fits the budget); none."));
            Phase = EPhase::Idle;
            return;
        }
        Job = MakeShared<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>();
        Job->R = Plan->R;
        Job->N = Plan->N;
        Job->FootprintCm = Plan->CellM * 100.0;
        Job->Cells = (int64)Plan->InteriorCells();
        Job->Tiles.SetNumUninitialized(Plan->InteriorTiles * 3);
        for (int32 t = 0; t < Plan->InteriorTiles; t++)
        {
            const VoxelOceanCoastCore::FTile& T = Plan->Tiles[size_t(t)];
            Job->Tiles[3 * t] = T.Face;
            Job->Tiles[3 * t + 1] = T.I;
            Job->Tiles[3 * t + 2] = T.J;
        }
        // (The budget holds the cells well under int32's range: 4 bytes a texel.)
        Job->DepthCm.SetNumZeroed((int32)FMath::Min<int64>(Job->Cells, MAX_int32));
        Issued = 0;
        Phase = EPhase::Sampling;
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field: %d tiles (%d past face edges), %d x %d cells of %.0f m, %lld cells to sample; atlas %.1f MB; reach %.1f km."),
            int32(Plan->Tiles.size()), int32(Plan->Tiles.size()) - Plan->InteriorTiles, Plan->R, Plan->R, Plan->CellM, Job->Cells, Plan->AtlasBytes / 1048576.0,
            Plan->Settings.ReachM * 1.0e-3);
        if (Plan->TrimmedTiles > 0)
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] Coastal field: %d bake cells farthest from the coast left out to fit CoastMemoryMB (%.0f MB) at %.0f m cells; the field reaches %.1f km."),
                Plan->TrimmedTiles, Plan->Settings.BudgetBytes / 1048576.0, Plan->CellM, Plan->Settings.ReachM * 1.0e-3);
    }
    if (Phase == EPhase::Sampling)
    {
        if (Job->Failed.load(std::memory_order_acquire) != 0)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] Coastal field: the GPU's depth sampling failed; none."));
            Phase = EPhase::Idle;
            return;
        }
        while (Issued < Job->Cells && Job->InFlight.load(std::memory_order_acquire) < VOCF::MaxInFlight)
        {
            const int32 Count = (int32)FMath::Min<int64>(VOCF::ChunkCells, Job->Cells - Issued);
            Job->InFlight.fetch_add(1, std::memory_order_acq_rel);
            if (!Sampler || !Sampler(Job, Issued, Count))
            {
                Job->InFlight.fetch_sub(1, std::memory_order_acq_rel);
                return;
            }
            Issued += Count;
        }
        if (Job->Received.load(std::memory_order_acquire) < Job->Cells) return;
        TSharedPtr<VoxelOceanCoastCore::FPlan, ESPMode::ThreadSafe> InPlan = Plan;
        TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe> InJob = Job;
        TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> InField = Shore;
        const uint32 Key = Bake.IsValid() ? Bake->Key : 0u;
        const double Started = StartSeconds;
        const VoxelOceanCoastCore::FSettings Requested = Settings;
        // On its own thread (it can take a minute on a large planet: not the pool's).
        Solved = Async(EAsyncExecution::Thread, [InPlan, InJob, InField, Key, Started, Requested]() -> TSharedPtr<FVoxelOceanCoastField, ESPMode::ThreadSafe>
            {
                // The depths to metres in place.
                float* DepthM = InJob->DepthCm.GetData();
                for (int64 c = 0; c < InJob->Cells; c++) DepthM[c] *= 0.01f;
                VoxelOceanCoastCore::FSolved V = VoxelOceanCoastCore::Solve(*InPlan, DepthM, InField->Texels.GetData(), InField->ReachM);
                InJob->DepthCm.Empty();
                std::vector<uint32_t> Texels = VoxelOceanCoastCore::Pack(*InPlan, V);
                V = VoxelOceanCoastCore::FSolved();
                TSharedPtr<FVoxelOceanCoastField, ESPMode::ThreadSafe> Field = MakeShared<FVoxelOceanCoastField, ESPMode::ThreadSafe>();
                Field->N = InPlan->N;
                Field->R = InPlan->R;
                Field->Slices = InPlan->Slices();
                Field->RadiusCm = InField->RadiusCm;
                Field->CellM = InPlan->CellM;
                Field->BakeKey = Key;
                Field->Settings = InPlan->Settings;
                Field->Requested = Requested;
                float Row0[4], Row1[4];
                VoxelOceanCoastCore::LayoutRows(*InPlan, Row0, Row1);
                Field->Rows[0] = FVector4f(Row0[0], Row0[1], Row0[2], Row0[3]);
                Field->Rows[1] = FVector4f(Row1[0], Row1[1], Row1[2], Row1[3]);
                Field->Rows[2] = FVector4f((float)(InField->RadiusCm * 0.01), 0.0f, 0.0f, 0.0f);
                Field->Texels = MoveTemp(Texels);
                Field->TrimmedTiles = InPlan->TrimmedTiles;
                Field->Shore = InField;
                Field->Tiles = int32(InPlan->Tiles.size());
                Field->InteriorTiles = InPlan->InteriorTiles;
                Field->AtlasBytes = InPlan->AtlasBytes;
                Field->SolveSeconds = V.Seconds;
                Field->BuildSeconds = FPlatformTime::Seconds() - Started;
                return Field;
            });
        Job.Reset();
        // The plan's tables go with the solve (it holds its own reference).
        Plan.Reset();
        Phase = EPhase::Solving;
        return;
    }
    if (Phase == EPhase::Solving)
    {
        if (!Solved.IsReady()) return;
        TSharedPtr<FVoxelOceanCoastField, ESPMode::ThreadSafe> Field = Solved.Get();
        Phase = EPhase::Idle;
        if (!Field.IsValid() || !Field->IsValid())
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] Coastal field: the solve came back incomplete; none."));
            return;
        }
        Result = Field;
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field of bake %08x: %d tiles, cells of %.0f m, %.1f MB, in %.1f s (solve %.1f s)."),
            Field->BakeKey, Field->Tiles, Field->CellM, Field->AtlasBytes / 1048576.0, Field->BuildSeconds, Field->SolveSeconds);
    }
}
