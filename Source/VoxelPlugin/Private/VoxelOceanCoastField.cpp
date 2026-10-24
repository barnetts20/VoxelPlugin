#include "VoxelOceanCoastField.h"
#include "VoxelOceanShoreField.h"
#include "VoxelLandBake.h"
#include "VoxelPlugin.h"
#include "Async/Async.h"
#include "HAL/PlatformTime.h"
#include "Engine/Texture2DArray.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "TextureResource.h"

namespace VoxelOceanCoastFieldDetail
{
    /** Fine cells a GPU sampling chunk takes, and chunks in flight at once (stage W7a: smaller
     *  chunks, more of them in flight: each is a terrain evaluation per cell, spread over frames). */
    constexpr int32 ChunkCells = 1 << 18;
    constexpr int32 MaxInFlight = 4;

    /** The fields' serials (game thread). */
    uint32 NextSerial = 1;
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

void FVoxelOceanCoastField::WindowRows(double Now, FVector4f Out[3]) const
{
    const double Blend = BlendSeconds > 0.0 ? FMath::Clamp((Now - BlendStart) / BlendSeconds, 0.0, 1.0) : 1.0;
    const double Fade = BlendSeconds > 0.0 ? FMath::Clamp((Now - FadeStart) / BlendSeconds, 0.0, 1.0) : 1.0;
    // (Smoothly in time as well: no step in the cover's rate at either end.)
    auto Ease = [](double T) { return T * T * (3.0 - 2.0 * T); };
    Out[0] = FVector4f(Rows[2].X, (float)Ease(Blend), (float)Ease(Fade), bWindow ? 1.0f : 0.0f);
    Out[1] = FVector4f((float)Centre.X, (float)Centre.Y, (float)Centre.Z, (float)InnerM);
    Out[2] = FVector4f((float)CentreLast.X, (float)CentreLast.Y, (float)CentreLast.Z, (float)OuterM);
}

VoxelShader::FVoxelOceanCoastSample FVoxelOceanCoastField::SampleCoast(const FVector& PlanetPoint) const
{
    if (!IsValid()) return VoxelShader::VoxelOceanCoastSampleNone();
    const float Row0[4] = { Rows[0].X, Rows[0].Y, Rows[0].Z, Rows[0].W };
    const float Row1[4] = { Rows[1].X, Rows[1].Y, Rows[1].Z, Rows[1].W };
    VoxelShader::FVoxelOceanCoastSample S = VoxelOceanCoastCore::Sample(Shore->Texels.GetData(), Texels.data(), Slices, N, Row0, Row1, RadiusCm * 0.01,
        PlanetPoint.X, PlanetPoint.Y, PlanetPoint.Z);
    // Stage W7a: the window's cover and the fade in, as the shaders' (VoxelOceanCoastNear).
    FVector4f W[3];
    WindowRows(FPlatformTime::Seconds(), W);
    using namespace VoxelShader;
    const FVoxelOceanCoastLayout L = VoxelOceanCoastLayoutWindow(VoxelOceanCoastLayoutFrom(float4(Row0[0], Row0[1], Row0[2], Row0[3]),
        float4(Row1[0], Row1[1], Row1[2], Row1[3]), float(RadiusCm * 0.01)), float4(W[0].X, W[0].Y, W[0].Z, W[0].W), float4(W[1].X, W[1].Y, W[1].Z, W[1].W),
        float4(W[2].X, W[2].Y, W[2].Z, W[2].W));
    const FVector D = PlanetPoint.GetSafeNormal();
    S.Cover *= VoxelOceanCoastWindowCover(L, float3((float)D.X, (float)D.Y, (float)D.Z));
    return S;
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

bool FVoxelOceanCoastField::WriteInto(UTexture2DArray* Texture, const TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe>& Made, const FVoxelOceanCoastField* Last)
{
    check(IsInGameThread());
    if (!Texture || !Made.IsValid() || !Made->IsValid()) return false;
    FTextureResource* Resource = Texture->GetResource();
    if (!Resource) return false;
    const int32 Side = VoxelOceanShoreCore::Side(Made->N);
    // The texture's slice, the rectangle (texels), all from Made's texels (its slice 0 the
    // texture's 6).
    struct FRect { int32 Slice, X, Y, W, H; };
    TArray<FRect> Rects;
    for (int32 f = 0; f < 6; f++)
    {
        int32 B[4] = { Made->IndexBox[f][0], Made->IndexBox[f][1], Made->IndexBox[f][2], Made->IndexBox[f][3] };
        if (Last && Last->N == Made->N && Last->IndexBox[f][0] <= Last->IndexBox[f][2])
        {
            B[0] = FMath::Min(B[0], Last->IndexBox[f][0]);
            B[1] = FMath::Min(B[1], Last->IndexBox[f][1]);
            B[2] = FMath::Max(B[2], Last->IndexBox[f][2]);
            B[3] = FMath::Max(B[3], Last->IndexBox[f][3]);
        }
        B[0] = FMath::Max(B[0], 0);
        B[1] = FMath::Max(B[1], 0);
        B[2] = FMath::Min(B[2], Side - 1);
        B[3] = FMath::Min(B[3], Side - 1);
        if (B[0] > B[2] || B[1] > B[3]) continue;
        Rects.Add({ VOXEL_OCEAN_COAST_INDEX_SLICE + f, B[0], B[1], B[2] - B[0] + 1, B[3] - B[1] + 1 });
    }
    for (int32 s = 0; s < Made->UsedAtlasSlices && VOXEL_OCEAN_COAST_ATLAS_SLICE - 6 + s < Made->Slices; s++)
        Rects.Add({ VOXEL_OCEAN_COAST_ATLAS_SLICE + s, 0, 0, Side, Side });
    ENQUEUE_RENDER_COMMAND(VoxelOceanCoastWrite)([Resource, Made, Rects = MoveTemp(Rects), Side](FRHICommandListImmediate& RHICmdList)
        {
            FRHITexture* Dest = Resource->TextureRHI;
            if (!Dest) return;
            // Each rectangle into a staging texture of its own, then copied into its slice.
            TArray<FTextureRHIRef> Staging;
            Staging.Reserve(Rects.Num());
            for (const FRect& R : Rects)
            {
                const FRHITextureCreateDesc Desc = FRHITextureCreateDesc::Create2D(TEXT("VoxelOceanCoastStaging"), R.W, R.H, PF_R32_FLOAT)
                    .SetFlags(ETextureCreateFlags::ShaderResource)
                    .SetInitialState(ERHIAccess::SRVMask);
                FTextureRHIRef Tex = RHICmdList.CreateTexture(Desc);
                const uint32* First = Made->Texels.data() + ((size_t)(R.Slice - 6) * (size_t)Side + (size_t)R.Y) * (size_t)Side + (size_t)R.X;
                RHICmdList.UpdateTexture2D(Tex, 0, FUpdateTextureRegion2D(0, 0, 0, 0, (uint32)R.W, (uint32)R.H), (uint32)Side * 4u, (const uint8*)First);
                Staging.Add(Tex);
            }
            FRDGBuilder GraphBuilder(RHICmdList);
            FRDGTextureRef DestRDG = RegisterExternalTexture(GraphBuilder, Dest, TEXT("Voxel.OceanShore"));
            for (int32 i = 0; i < Rects.Num(); i++)
            {
                const FRect& R = Rects[i];
                FRDGTextureRef Source = RegisterExternalTexture(GraphBuilder, Staging[i].GetReference(), TEXT("Voxel.OceanCoastStaging"));
                FRHICopyTextureInfo Copy;
                Copy.Size = FIntVector(R.W, R.H, 1);
                Copy.DestPosition = FIntVector(R.X, R.Y, 0);
                Copy.SourceSliceIndex = 0;
                Copy.DestSliceIndex = (uint32)R.Slice;
                Copy.NumSlices = 1;
                AddCopyTexturePass(GraphBuilder, Source, DestRDG, Copy);
            }
            GraphBuilder.SetTextureAccessFinal(DestRDG, ERHIAccess::SRVMask);
            GraphBuilder.Execute();
        });
    return true;
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
    Previous.Reset();
    bForThis = false;
}

bool FVoxelOceanCoastBuilder::IsFor(const FVoxelOceanShoreField* InShore, const VoxelOceanCoastCore::FSettings& InSettings) const
{
    return bForThis && Shore.Get() == InShore && Settings == InSettings;
}

void FVoxelOceanCoastBuilder::Start(const TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe>& InShore,
    const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& InBake, const VoxelOceanCoastCore::FSettings& InSettings, const FVector3d& CentreDir,
    const TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe>& InPrevious)
{
    Reset();
    Shore = InShore;
    Bake = InBake;
    Settings = InSettings;
    Centre = CentreDir.GetSafeNormal();
    if (Centre.IsNearlyZero()) Centre = FVector3d::ZAxisVector;
    Previous = InPrevious;
    bForThis = true;
    StartSeconds = FPlatformTime::Seconds();
    if (!Shore.IsValid() || !Shore->IsValid() || !Bake.IsValid() || Bake->N != Shore->N || Bake->Height.Num() != Bake->NumCells()
        || FVoxelLandBake::Pad != VoxelOceanCoastCore::Pad)
        return;
    Plan = MakeShared<VoxelOceanCoastCore::FPlan, ESPMode::ThreadSafe>();
    TSharedPtr<VoxelOceanCoastCore::FPlan, ESPMode::ThreadSafe> OutPlan = Plan;
    // Stage W7b: the coast's cells as kept (the same shore field, the same selection), else found
    // now and kept once the plan is in.
    auto SameSelection = [](const VoxelOceanCoastCore::FSettings& A, const VoxelOceanCoastCore::FSettings& B)
        {
            return A.SeedDepthM == B.SeedDepthM && A.ReachM == B.ReachM && A.LandReachM == B.LandReachM && A.LowLandM == B.LowLandM;
        };
    TSharedPtr<const VoxelOceanCoastCore::FCoastCells, ESPMode::ThreadSafe> Cached;
    if (KeptCells.IsValid() && KeptShore.Pin() == Shore && SameSelection(KeptSettings, Settings))
        Cached = KeptCells;
    MadeCells.Reset();
    TSharedPtr<VoxelOceanCoastCore::FCoastCells, ESPMode::ThreadSafe> OutCells;
    if (!Cached.IsValid())
        OutCells = MadeCells = MakeShared<VoxelOceanCoastCore::FCoastCells, ESPMode::ThreadSafe>();
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> InField = Shore;
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> InLand = Bake;
    const VoxelOceanCoastCore::FSettings S = Settings;
    VoxelOceanCoastCore::FWindow Window;
    Window.bOn = S.IsWindowed();
    Window.Centre[0] = Centre.X;
    Window.Centre[1] = Centre.Y;
    Window.Centre[2] = Centre.Z;
    // (Stage W7b: on a thread of its own: behind the terrain's work in the pool a plan once waited
    // 40 s.)
    Planned = Async(EAsyncExecution::Thread, [OutPlan, OutCells, Cached, InField, InLand, S, Window]()
        {
            const int32 N = InField->N;
            const double RadiusM = InField->RadiusCm * 0.01;
            // (Stage W7d: the heights the shore field was made from: the terrain's where sampled.)
            const float* Heights = InField->Height.Num() == InLand->Height.Num() ? InField->Height.GetData() : InLand->Height.GetData();
            if (OutCells.IsValid())
                *OutCells = VoxelOceanCoastCore::CoastCells(N, RadiusM, InField->Texels.GetData(), InField->ReachM, Heights, S);
            const VoxelOceanCoastCore::FCoastCells* Cells = Cached.IsValid() ? Cached.Get() : OutCells.Get();
            *OutPlan = VoxelOceanCoastCore::Plan(N, RadiusM, InField->Texels.GetData(), InField->ReachM, Heights, S, Window, Cells);
        });
    Phase = EPhase::Planning;
}

void FVoxelOceanCoastBuilder::Tick(const FSampler& Sampler)
{
    if (Phase == EPhase::Planning)
    {
        if (!Planned.IsReady()) return;
        if (MadeCells.IsValid())
        {
            KeptCells = MadeCells;
            KeptShore = Shore;
            KeptSettings = Settings;
            MadeCells.Reset();
        }
        if (!Plan->bValid)
        {
            // (A window with no coast in it is common, out at sea: quietly.)
            // (Braced: UE_LOG is a block of its own.)
            if (Settings.IsWindowed())
            {
                UE_LOG(LogVoxelPlugin, Verbose, TEXT("[VoxelOcean] Coastal field: no coast within the window; none."));
            }
            else
            {
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field: no coast within the shore field (or no tile fits the budget); none."));
            }
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
        SampleStartSeconds = FPlatformTime::Seconds();
        if (Plan->Window.bOn)
        {
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field: window of %.1f km (tiles within %.1f km), %d tiles (%d past face edges), %d x %d cells of %.0f m, %lld cells to sample; atlas %.1f MB; reach %.1f km; planned in %.2f s."),
                (Settings.WindowRadiusM + Settings.WindowFadeM) * 1.0e-3, Settings.SolveRadiusM() * 1.0e-3, int32(Plan->Tiles.size()), int32(Plan->Tiles.size()) - Plan->InteriorTiles,
                Plan->R, Plan->R, Plan->CellM, Job->Cells, Plan->AtlasBytes / 1048576.0, Plan->Settings.ReachM * 1.0e-3, SampleStartSeconds - StartSeconds);
            if (Plan->TrimmedTiles > 0)
                UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field: %d bake cells farthest from the window's centre left out to fit CoastWindowCells (%.1f million) at %.0f m cells."),
                    Plan->TrimmedTiles, Settings.WindowMaxCells * 1.0e-6, Plan->CellM);
        }
        else
        {
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field: %d tiles (%d past face edges), %d x %d cells of %.0f m, %lld cells to sample; atlas %.1f MB; reach %.1f km."),
                int32(Plan->Tiles.size()), int32(Plan->Tiles.size()) - Plan->InteriorTiles, Plan->R, Plan->R, Plan->CellM, Job->Cells, Plan->AtlasBytes / 1048576.0,
                Plan->Settings.ReachM * 1.0e-3);
            if (Plan->TrimmedTiles > 0)
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] Coastal field: %d bake cells farthest from the coast left out to fit CoastMemoryMB (%.0f MB) at %.0f m cells; the field reaches %.1f km."),
                    Plan->TrimmedTiles, Plan->Settings.BudgetBytes / 1048576.0, Plan->CellM, Plan->Settings.ReachM * 1.0e-3);
        }
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
        const double SampleSeconds = FPlatformTime::Seconds() - SampleStartSeconds;
        const VoxelOceanCoastCore::FSettings Requested = Settings;
        // On its own thread (it can take a minute on a large planet without the window: not the
        // pool's).
        Solved = Async(EAsyncExecution::Thread, [InPlan, InJob, InField, Key, Started, SampleSeconds, Requested]() -> TSharedPtr<FVoxelOceanCoastField, ESPMode::ThreadSafe>
            {
                // The depths to metres in place.
                float* DepthM = InJob->DepthCm.GetData();
                for (int64 c = 0; c < InJob->Cells; c++) DepthM[c] *= 0.01f;
                VoxelOceanCoastCore::FSolved V = VoxelOceanCoastCore::Solve(*InPlan, DepthM, InField->Texels.GetData(), InField->ReachM);
                const double SolveSeconds = V.Seconds;
                InJob->DepthCm.Empty();
                const double PackStart = FPlatformTime::Seconds();
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
                Field->Rows[2] = FVector4f((float)(InField->RadiusCm * 0.01), 1.0f, 1.0f, 0.0f);
                Field->Texels = MoveTemp(Texels);
                Field->TrimmedTiles = InPlan->TrimmedTiles;
                Field->Shore = InField;
                Field->Tiles = int32(InPlan->Tiles.size());
                Field->InteriorTiles = InPlan->InteriorTiles;
                Field->AtlasBytes = InPlan->AtlasBytes;
                Field->Cells = (int64)InPlan->InteriorCells();
                VoxelOceanCoastCore::IndexBoxes(*InPlan, Field->IndexBox);
                Field->UsedAtlasSlices = InPlan->AtlasSlices;
                Field->bWindow = InPlan->Window.bOn;
                Field->Centre = FVector3d(InPlan->Window.Centre[0], InPlan->Window.Centre[1], InPlan->Window.Centre[2]);
                Field->CentreLast = Field->Centre;
                Field->InnerM = Requested.WindowRadiusM;
                Field->OuterM = Requested.WindowRadiusM + FMath::Max(Requested.WindowFadeM, 1.0);
                Field->SampleSeconds = SampleSeconds;
                Field->SolveSeconds = SolveSeconds;
                Field->PackSeconds = FPlatformTime::Seconds() - PackStart;
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
            Previous.Reset();
            return;
        }
        // Stage W7a: the cover's blend from the field shown now (a window near this one, settled)
        // or the fade in (the first field, or one far from the last).
        const double Now = FPlatformTime::Seconds();
        Field->Serial = VOCF::NextSerial++;
        Field->BlendSeconds = FMath::Max(Settings.WindowBlendSeconds, 0.0);
        const double RadiusM = Field->RadiusCm * 0.01;
        const bool bNear = Previous.IsValid() && Previous->IsValid() && Previous->Shore == Field->Shore && Previous->bWindow && Field->bWindow
            && (Previous->Centre - Field->Centre).Size() * RadiusM <= 2.0 * Settings.WindowMoveM && Previous->IsSettled(Now);
        if (bNear)
        {
            Field->CentreLast = Previous->Centre;
            Field->BlendStart = Now;
            Field->FadeStart = Previous->FadeStart;
        }
        else
        {
            Field->CentreLast = Field->Centre;
            Field->BlendStart = -1.0e30;
            Field->FadeStart = Now;
        }
        Previous.Reset();
        Result = Field;
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Coastal field of bake %08x: %d tiles, %lld cells of %.0f m, %.1f MB, in %.2f s (plan and wait %.2f s, sampling %.2f s, solve %.2f s, pack %.2f s)."),
            Field->BakeKey, Field->Tiles, Field->Cells, Field->CellM, Field->AtlasBytes / 1048576.0, Field->BuildSeconds,
            FMath::Max(Field->BuildSeconds - Field->SampleSeconds - Field->SolveSeconds - Field->PackSeconds, 0.0), Field->SampleSeconds, Field->SolveSeconds, Field->PackSeconds);
    }
}
