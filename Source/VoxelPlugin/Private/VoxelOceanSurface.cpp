#include "VoxelOceanSurface.h"
#include "VoxelPlugin.h"
#include "VoxelTerrainStats.h"
#include "HAL/IConsoleManager.h"
#include "Async/ParallelFor.h"
#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Engine/World.h"
#include "Misc/App.h"
#include "VoxelDensityGPU.h"

DECLARE_CYCLE_STAT(TEXT("Ocean: select"), STAT_VoxelOceanSelect, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Ocean: build tiles"), STAT_VoxelOceanBuild, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Ocean: tiles"), STAT_VoxelOceanTiles, STATGROUP_VoxelTerrain);

static TAutoConsoleVariable<int32> CVarVoxelOceanMorph(
    TEXT("voxel.Ocean.Morph"), 1,
    TEXT("Ocean morph between tile levels: 0 off, 1 by distance, 2 fully morphed (every tile shows the grid two levels coarser, for checking)."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelOceanMorphStart(
    TEXT("voxel.Ocean.MorphStart"), 0.5f,
    TEXT("Where each ocean level starts to morph, as a fraction of where it ends (0.5 to 0.95). 0.5: the levels' bands touch and the mesh morphs continuously with distance; higher: narrower bands."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelOceanFlipWinding(
    TEXT("voxel.Ocean.FlipWinding"), 0,
    TEXT("Reverses the ocean's triangle winding. A diagnostic: with 0 the ocean must be visible from above (and culled from below with a one-sided material). Rebuilds the ocean."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugOcean(
    TEXT("voxel.Debug.Ocean"), 0,
    TEXT("Once a second: the ocean's tiles, pages, commits and their waits."),
    ECVF_Default);

namespace VoxelOceanSurfaceDetail
{
    const FName CameraHighName(TEXT("VoxelOceanCameraHigh"));
    const FName CameraLowName(TEXT("VoxelOceanCameraLow"));
    const FName ParamsName(TEXT("VoxelOceanParams"));
    const FName WaveNames[3] = { FName(TEXT("VoxelOceanWave0")), FName(TEXT("VoxelOceanWave1")), FName(TEXT("VoxelOceanWave2")) };
    const FName CenterName(TEXT("VoxelOceanCenter"));
    const FName ShapeName(TEXT("VoxelOceanShape"));
    constexpr double Hysteresis = 1.1;
    constexpr int32 ParallelBuildTiles = 8;        // fewer tiles are built on the game thread
    /** How far a point of a drawn triangle can be from the vertices whose depth culls it, in
     *  tile spacings: the triangle itself, and its corners' morph to the grandparent grid. */
    constexpr double CullReachSpacings = 6.0;
    constexpr double NoCull = 3.0e38;
}

namespace VOS = VoxelOceanSurfaceDetail;

FVoxelOceanSurface::FVoxelOceanSurface() = default;

FVoxelOceanSurface::~FVoxelOceanSurface()
{
    Reset();
}

void FVoxelOceanSurface::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelOceanSurfaceSettings& InSettings)
{
    Reset();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Settings = InSettings;
    Settings.LodDistanceFactor = FMath::Max(Settings.LodDistanceFactor, VoxelOceanTiles::MinLodDistanceFactor);
    // 8, 16 or 32 quads per tile edge (the page shader holds at most 33 x 33 vertices).
    Settings.TileQuads = 1 << FMath::Clamp(FMath::RoundToInt(FMath::Log2((double)FMath::Max(Settings.TileQuads, 1))), 3, 5);
    bFlipWinding = CVarVoxelOceanFlipWinding.GetValueOnGameThread() > 0;
    bReportedMaterial = false;
    bReportedShape = false;
    // A server, or a commandlet, draws nothing.
    bAvailable = FApp::CanEverRender() && InOwner && InAttachRoot && Settings.Radius > 0.0;
    if (!bAvailable) return;
    Pages = MakeUnique<FVoxelOceanPages>();
    Pages->Initialize(InOwner, InAttachRoot, Settings.Material.Get(), Settings.Radius, Settings.TileQuads,
        VoxelOceanTiles::MorphEnd0(Settings.Radius, Settings.LodDistanceFactor), bFlipWinding);
    if (bHasInputs)
        Pages->SetDensityInputs(Inputs.Density);
}

void FVoxelOceanSurface::Reset()
{
    if (Pages.IsValid())
        Pages->Reset();
    Pages.Reset();
    Committed.Reset();
    CommittedSplit.Reset();
    PendingEdits.Reset();
    bRewriteAll = false;
    bHasSelectionCamera = false;
    bFirstFill = true;
}

void FVoxelOceanSurface::SetInputs(const FVoxelOceanSurfaceInputs& InInputs)
{
    // New edit data with a newer edit is explained by the pending edits (NotifyEdit); edit data
    // that appears, goes, or is replaced without a newer edit (the terrain rebuilt) is not.
    const bool bEditsReplaced = InInputs.Density.Edits.IsValid() != Inputs.Density.Edits.IsValid()
        || (InInputs.Density.Edits != Inputs.Density.Edits && InInputs.EditSequence <= Inputs.EditSequence);
    const bool bChanged = !bHasInputs || bEditsReplaced
        || InInputs.Density.Density != Inputs.Density.Density
        || InInputs.Density.HalfFinest != Inputs.Density.HalfFinest
        || InInputs.SlopeBound != Inputs.SlopeBound
        || InInputs.LandCullMargin != Inputs.LandCullMargin;
    // Tiles written before any inputs, or for other ones, are written again.
    bRewriteAll |= bChanged && Committed.Num() > 0;
    Inputs = InInputs;
    bHasInputs = true;
    if (Pages.IsValid())
        Pages->SetDensityInputs(Inputs.Density);
}

void FVoxelOceanSurface::NotifyEdit(uint32 Sequence, const FVector& Center, double Radius)
{
    // Nothing drawn (a server): nothing to write again.
    if (!bAvailable) return;
    // As the terrain's reach (two radii and two finest cells), and a vertex's step to the
    // lattice point it samples.
    FPendingEdit& Edit = PendingEdits.AddDefaulted_GetRef();
    Edit.Sequence = Sequence;
    Edit.Center = Center;
    Edit.Reach = 2.0 * Radius + 6.0 * Inputs.Density.HalfFinest;
}

void FVoxelOceanSurface::SetMaterial(UMaterialInstanceDynamic* InMaterial)
{
    Settings.Material = InMaterial;
    bReportedMaterial = false;
    bReportedShape = false;
    if (Pages.IsValid())
        Pages->SetMaterial(InMaterial);
    UpdateMaterial();
}

void FVoxelOceanSurface::SetLodDistanceFactor(double InFactor)
{
    Settings.LodDistanceFactor = FMath::Max(InFactor, VoxelOceanTiles::MinLodDistanceFactor);
    if (!Pages.IsValid()) return;
    // The tiles on screen were selected for the old factor and its morph bands: they are
    // cleared, and the surface fills in again for the new one.
    Pages->Commit(TConstArrayView<FVoxelOceanPages::FTileWrite>(), Committed.Array());
    Pages->SetMorphEnd0(VoxelOceanTiles::MorphEnd0(Settings.Radius, Settings.LodDistanceFactor));
    Committed.Reset();
    CommittedSplit.Reset();
    bFirstFill = true;
}

VoxelOceanTiles::FSelectParams FVoxelOceanSurface::SelectParams() const
{
    VoxelOceanTiles::FSelectParams Params;
    Params.Radius = Settings.Radius;
    Params.MaxLevel = Settings.MaxLevel;
    Params.LodDistanceFactor = Settings.LodDistanceFactor;
    Params.Hysteresis = VOS::Hysteresis;
    Params.HorizonHeight = Settings.HorizonHeight;
    return Params;
}

void FVoxelOceanSurface::BuildWrites(TConstArrayView<FVoxelOceanTileKey> Tiles, TArray<FVoxelOceanPages::FTileWrite>& Out) const
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanBuild);
    Out.SetNum(Tiles.Num());
    const int32 Quads = Settings.TileQuads;
    const double Radius = Settings.Radius;
    const double HalfFinest = FMath::Max(Inputs.Density.HalfFinest, 1.0e-3);
    const double MinFootprint = Inputs.Density.Density.IsValid() ? Inputs.Density.Density->MinFootprint : 0.0;
    const double SlopeBound = FMath::Max(Inputs.SlopeBound, 0.0);
    const double Margin = Inputs.LandCullMargin;
    // The waves reach up and sideways over land the cull would otherwise leave dry.
    const double WaveHeight = WaveModel.IsValid() ? WaveModel->MaxHeight() : 0.0;
    const double WaveReach = WaveModel.IsValid() ? WaveModel->MaxHorizontal() : 0.0;
    ParallelFor(Tiles.Num(), [&Tiles, &Out, Quads, Radius, HalfFinest, MinFootprint, SlopeBound, Margin, WaveHeight, WaveReach](int32 Index)
        {
            TArray<VoxelOceanTiles::FTileVertex> Vertices;
            VoxelOceanTiles::BuildTile(Tiles[Index], Quads, Radius, Vertices);
            FVoxelOceanPages::FTileWrite& Write = Out[Index];
            Write.Tile = Tiles[Index];
            // Detail finer than the tile's spacing is left out of its depth. A triangle is culled
            // where the land stands above the sea, at its corners, by more than the margin and
            // the slope bound over the triangle's reach (with the step to the lattice point).
            const double Spacing = VoxelOceanTiles::MaxSpacing(Vertices, Quads);
            Write.Footprint = (float)FMath::Max(Spacing, MinFootprint);
            Write.CullHeight = Margin < 0.0 ? (float)VOS::NoCull
                : (float)(Margin + WaveHeight + SlopeBound * (VOS::CullReachSpacings * Spacing + 2.0 * HalfFinest + WaveReach));
            Write.Words.SetNumUninitialized(Vertices.Num() * FVoxelOceanPages::VertexWords);
            for (int32 v = 0; v < Vertices.Num(); v++)
            {
                const FVector Lattice = Vertices[v].Point / HalfFinest;
                const FIntVector Index3(
                    (int32)FMath::Clamp(FMath::RoundToDouble(Lattice.X), -2147483000.0, 2147483000.0),
                    (int32)FMath::Clamp(FMath::RoundToDouble(Lattice.Y), -2147483000.0, 2147483000.0),
                    (int32)FMath::Clamp(FMath::RoundToDouble(Lattice.Z), -2147483000.0, 2147483000.0));
                FVoxelOceanPages::PackVertex(Vertices[v], Index3, &Write.Words[v * FVoxelOceanPages::VertexWords]);
            }
        }, Tiles.Num() < VOS::ParallelBuildTiles ? EParallelForFlags::ForceSingleThread : EParallelForFlags::None);
}

void FVoxelOceanSurface::ApplyRewrites()
{
    if (Committed.Num() == 0)
    {
        // Tiles written later sample the edits the inputs hold by then.
        bRewriteAll = false;
        PendingEdits.RemoveAll([this](const FPendingEdit& Edit) { return Edit.Sequence <= Inputs.EditSequence; });
        return;
    }
    TArray<FVoxelOceanTileKey> ToWrite;
    if (bRewriteAll)
    {
        ToWrite = Committed.Array();
        bRewriteAll = false;
        // The edits the inputs hold are in these writes.
        PendingEdits.RemoveAll([this](const FPendingEdit& Edit) { return Edit.Sequence <= Inputs.EditSequence; });
    }
    else
    {
        TArray<FPendingEdit> Ready;
        for (int32 i = 0; i < PendingEdits.Num(); i++)
        {
            if (PendingEdits[i].Sequence > Inputs.EditSequence) continue;
            Ready.Add(PendingEdits[i]);
            PendingEdits.RemoveAtSwap(i--);
        }
        if (Ready.Num() == 0) return;
        for (const FVoxelOceanTileKey& Tile : Committed)
        {
            FVector CapCenter;
            double CapAngle = 0.0;
            VoxelOceanTiles::Cap(Tile, CapCenter, CapAngle);
            for (const FPendingEdit& Edit : Ready)
            {
                if (VoxelOceanTiles::DistanceToCap(Edit.Center, CapCenter, CapAngle, Settings.Radius) <= Edit.Reach)
                {
                    ToWrite.Add(Tile);
                    break;
                }
            }
        }
    }
    if (ToWrite.Num() == 0) return;
    TArray<FVoxelOceanPages::FTileWrite> Writes;
    BuildWrites(ToWrite, Writes);
    Pages->Commit(Writes, TConstArrayView<FVoxelOceanTileKey>());
    Perf.Rewritten += ToWrite.Num();
}

void FVoxelOceanSurface::Tick(const FVector& Camera)
{
    // The waves before anything drawn: a server has no pages but answers queries with the same
    // waves as every client.
    UpdateWaveModel();
    if (!bAvailable || !Pages.IsValid()) return;
    const double Now = FPlatformTime::Seconds();
    Perf.Frames++;

    // A winding change rewrites every tile.
    if ((CVarVoxelOceanFlipWinding.GetValueOnGameThread() > 0) != bFlipWinding)
    {
        bFlipWinding = !bFlipWinding;
        Pages->Initialize(Owner.Get(), AttachRoot.Get(), Settings.Material.Get(), Settings.Radius, Settings.TileQuads,
            VoxelOceanTiles::MorphEnd0(Settings.Radius, Settings.LodDistanceFactor), bFlipWinding);
        Committed.Reset();
        CommittedSplit.Reset();
        bFirstFill = true;
    }

    Pages->Flush(Settings.SpareBufferSets);
    ApplyRewrites();

    TArray<FVoxelOceanTileKey> Tiles;
    TSet<FVoxelOceanTileKey> Split;
    {
        SCOPE_CYCLE_COUNTER(STAT_VoxelOceanSelect);
        const double Start = FPlatformTime::Seconds();
        VoxelOceanTiles::Select(Camera, SelectParams(), &CommittedSplit, Tiles, Split);
        const double Ms = (FPlatformTime::Seconds() - Start) * 1000.0;
        Perf.SelectMs += Ms;
        Perf.MaxSelectMs = FMath::Max(Perf.MaxSelectMs, Ms);
    }
    Perf.Selected += Tiles.Num();

    // The same tiles: this camera is as good as the one they were selected for.
    bool bSame = !bFirstFill && Tiles.Num() == Committed.Num();
    for (int32 i = 0; bSame && i < Tiles.Num(); i++)
        bSame = Committed.Contains(Tiles[i]);
    if (bSame)
    {
        SelectionCamera = Camera;
        bHasSelectionCamera = true;
        CommittedSplit = MoveTemp(Split);
        Perf.Wait = 0;
        UpdateMaterial();
        LogPerf(Now);
        return;
    }

    Perf.Changes++;
    const double CommitStart = FPlatformTime::Seconds();
    TSet<FVoxelOceanTileKey> NewSet;
    NewSet.Reserve(Tiles.Num());
    TArray<FVoxelOceanTileKey> Added;
    TSet<FVoxelOceanTileKey> NeededPages;
    for (const FVoxelOceanTileKey& Tile : Tiles)
    {
        NewSet.Add(Tile);
        if (Committed.Contains(Tile)) continue;
        Added.Add(Tile);
        NeededPages.Add(VoxelOceanTiles::PageOf(Tile));
    }
    TArray<FVoxelOceanTileKey> Removed;
    for (const FVoxelOceanTileKey& Tile : Committed)
    {
        if (!NewSet.Contains(Tile))
            Removed.Add(Tile);
    }

    // The whole change in one frame, once every page it writes to is ready; until then the
    // tiles on screen and their camera stay. The first fill takes what is ready.
    const bool bReady = Pages->PreparePages(NeededPages);
    if (!bReady && !bFirstFill)
    {
        Perf.Waits++;
        Perf.Wait++;
        Perf.MaxWait = FMath::Max(Perf.MaxWait, Perf.Wait);
        UpdateMaterial();
        LogPerf(Now);
        return;
    }
    Perf.Wait = 0;

    TArray<FVoxelOceanTileKey> ToWrite;
    if (bReady)
    {
        ToWrite = MoveTemp(Added);
    }
    else
    {
        for (const FVoxelOceanTileKey& Tile : Added)
        {
            if (Pages->IsPageReady(VoxelOceanTiles::PageOf(Tile)))
                ToWrite.Add(Tile);
        }
    }
    TArray<FVoxelOceanPages::FTileWrite> Writes;
    BuildWrites(ToWrite, Writes);
    Pages->Commit(Writes, Removed);

    if (bReady)
    {
        Committed = MoveTemp(NewSet);
        bFirstFill = false;
    }
    else
    {
        for (const FVoxelOceanTileKey& Tile : Removed)
            Committed.Remove(Tile);
        for (const FVoxelOceanTileKey& Tile : ToWrite)
            Committed.Add(Tile);
    }
    CommittedSplit = MoveTemp(Split);
    SelectionCamera = Camera;
    bHasSelectionCamera = true;
    SET_DWORD_STAT(STAT_VoxelOceanTiles, Committed.Num());

    const double CommitMs = (FPlatformTime::Seconds() - CommitStart) * 1000.0;
    Perf.CommitMs += CommitMs;
    Perf.MaxCommitMs = FMath::Max(Perf.MaxCommitMs, CommitMs);
    UpdateMaterial();
    LogPerf(Now);
}

void FVoxelOceanSurface::UpdateWaveModel()
{
    const FVoxelOceanWavesTest::FSettings Next = FVoxelOceanWavesTest::ReadConsole();
    if (WaveModel.IsValid() == (Next.Amplitude > 0.0) && Next == WaveSettings) return;
    const double OldHeight = WaveModel.IsValid() ? WaveModel->MaxHeight() : 0.0;
    const double OldReach = WaveModel.IsValid() ? WaveModel->MaxHorizontal() : 0.0;
    WaveSettings = Next;
    WaveModel.Reset();
    if (Next.Amplitude > 0.0)
        WaveModel = MakeShared<FVoxelOceanWavesTest, ESPMode::ThreadSafe>(Next);
    const double NewHeight = WaveModel.IsValid() ? WaveModel->MaxHeight() : 0.0;
    const double NewReach = WaveModel.IsValid() ? WaveModel->MaxHorizontal() : 0.0;
    // The land cull allows for the waves' reach (BuildWrites).
    if (Inputs.LandCullMargin >= 0.0 && (OldHeight != NewHeight || OldReach != NewReach))
        bRewriteAll |= Committed.Num() > 0;
}

bool FVoxelOceanSurface::GetWaveFrame(FVoxelOceanWaveFrame& Out) const
{
    const USceneComponent* Root = AttachRoot.Get();
    if (!Root || Settings.Radius <= 0.0) return false;
    // The attach root carries the planet's position and rotation at scale 1.
    const FTransform& RootTransform = Root->GetComponentTransform();
    Out = FVoxelOceanWaveFrame();
    Out.PlanetToWorld = FTransform(RootTransform.GetRotation(), RootTransform.GetLocation());
    Out.Context.Time = Inputs.Time;
    Out.Context.Radius = Settings.Radius;
    Out.Context.Environment = Inputs.Environment;
    Out.Model = WaveModel;
    return true;
}

void FVoxelOceanSurface::UpdateMaterial()
{
    USceneComponent* Root = AttachRoot.Get();
    UMaterialInstanceDynamic* Material = Settings.Material.Get();
    FVoxelOceanWaveFrame Frame;
    if (!Root || !Material || !bHasSelectionCamera || !GetWaveFrame(Frame)) return;

    // The camera as a float and the rest, so the material subtracts it at full precision; the
    // planet centre from it (displacements along the exact radial).
    const FVector CameraWorld = Frame.PlanetToWorld.TransformPosition(SelectionCamera);
    const FVector3f High(CameraWorld);
    const FVector Low = CameraWorld - FVector(High);
    const FVector Center = Frame.PlanetToWorld.GetLocation() - CameraWorld;
    const int32 Mode = FMath::Clamp(CVarVoxelOceanMorph.GetValueOnGameThread(), 0, 2);
    const double StartFraction = VoxelOceanTiles::ClampMorphStartFraction(CVarVoxelOceanMorphStart.GetValueOnGameThread());
    // w: a level's vertex spacing per unit of its morph end (MorphEnd0 2^-L), for the waves' fade
    // with the mesh (VoxelOcean.ush).
    const double SpacingPerMorphEnd = 1.0 / (VoxelOceanTiles::MorphMargin * 2.0 * Settings.LodDistanceFactor * FMath::Max(Settings.TileQuads, 1));

    Material->SetVectorParameterValue(VOS::CameraHighName, FLinearColor(High.X, High.Y, High.Z, 0.0f));
    Material->SetVectorParameterValue(VOS::CameraLowName, FLinearColor((float)Low.X, (float)Low.Y, (float)Low.Z, 0.0f));
    // The waves from the selection camera (the underwater pass forms the same from each view's
    // camera).
    FVoxelOceanWaveShaderParams Waves;
    Frame.ShaderParams(SelectionCamera, Waves);
    Material->SetVectorParameterValue(VOS::ParamsName, FLinearColor((float)Mode, (float)StartFraction, Waves.Amplitude, (float)SpacingPerMorphEnd));
    Material->SetVectorParameterValue(VOS::CenterName, FLinearColor((float)Center.X, (float)Center.Y, (float)Center.Z, 0.0f));
    Material->SetVectorParameterValue(VOS::ShapeName, FLinearColor(Waves.Shape.X, Waves.Shape.Y, Waves.Shape.Z, Waves.Shape.W));
    if (Waves.Amplitude > 0.0f)
    {
        for (int32 w = 0; w < 3; w++)
            Material->SetVectorParameterValue(VOS::WaveNames[w], FLinearColor(Waves.Wave[w].X, Waves.Wave[w].Y, Waves.Wave[w].Z, Waves.Wave[w].W));
    }

    if (!bReportedMaterial)
    {
        // Ask the parent: the instance reports its own overrides (just set above) whether or
        // not the material has the parameters.
        FLinearColor Probe;
        UMaterialInterface* Parent = Material->Parent;
        const bool bWired = Parent
            && Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::CameraHighName), Probe)
            && Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::CameraLowName), Probe)
            && Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::ParamsName), Probe);
        if (!bWired)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] %s has no vector parameters VoxelOceanCameraHigh, VoxelOceanCameraLow and VoxelOceanParams: the ocean's levels won't morph (stage O2 notes, material setup)."),
                *GetNameSafe(Parent));
        }
        else if (!Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::WaveNames[0]), Probe)
            || !Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::CenterName), Probe))
        {
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] %s has no vector parameters VoxelOceanWave0 to VoxelOceanWave2 and VoxelOceanCenter: voxel.Ocean.TestWaves shows nothing (stage O2 notes, material setup)."),
                *GetNameSafe(Parent));
        }
        bReportedMaterial = true;
    }
    if (!bReportedShape && Waves.Amplitude > 0.0f)
    {
        FLinearColor Probe;
        UMaterialInterface* Parent = Material->Parent;
        if (Parent && !Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::ShapeName), Probe))
        {
            UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] %s has no vector parameter VoxelOceanShape: the waves move only up and down (stage O4 notes, material setup)."),
                *GetNameSafe(Parent));
        }
        bReportedShape = true;
    }
}

void FVoxelOceanSurface::LogPerf(double Now)
{
    if (Perf.Start == 0.0) Perf.Start = Now;
    if (Now - Perf.Start < 1.0) return;
    const FString PagePerf = Pages.IsValid() ? Pages->TakePerf() : FString();
    if (CVarVoxelDebugOcean.GetValueOnGameThread() > 0 && Pages.IsValid())
    {
        const double PerFrame = 1.0 / FMath::Max(1, Perf.Frames);
        const double PerChange = 1.0 / FMath::Max(1, Perf.Changes - Perf.Waits);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] %d frames in %.2f s. Tiles: %.0f selected, %d on screen%s (levels 0-%d, factor %.2f). Select %.2f ms average, %.2f max; commit %.2f ms average, %.2f max; %d frames with a change, %d of them waiting for buffer sets (%d at most in a row); %d tiles written again for depth changes, %d edits waiting for the terrain's GPU data. %s %s"),
            Perf.Frames, Now - Perf.Start, (double)Perf.Selected * PerFrame, Committed.Num(), bFirstFill ? TEXT(" (filling)") : TEXT(""),
            Settings.MaxLevel, Settings.LodDistanceFactor,
            Perf.SelectMs * PerFrame, Perf.MaxSelectMs, Perf.CommitMs * PerChange, Perf.MaxCommitMs, Perf.Changes, Perf.Waits, Perf.MaxWait,
            Perf.Rewritten, PendingEdits.Num(), *Pages->Describe(), *PagePerf);
    }
    const int32 Wait = Perf.Wait;
    Perf = FPerf();
    Perf.Start = Now;
    Perf.Wait = Wait;
}
