#include "VoxelOceanSurface.h"
#include "VoxelPlugin.h"
#include "VoxelTerrainStats.h"
#include "HAL/IConsoleManager.h"
#include "Async/ParallelFor.h"
#include "Components/SceneComponent.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Misc/App.h"

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
    constexpr double Hysteresis = 1.1;
    constexpr int32 ParallelBuildTiles = 8;        // fewer tiles are built on the game thread
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
    bFlipWinding = CVarVoxelOceanFlipWinding.GetValueOnGameThread() > 0;
    bReportedMaterial = false;
    // A server, or a commandlet, draws nothing.
    bAvailable = FApp::CanEverRender() && InOwner && InAttachRoot && Settings.Radius > 0.0;
    if (!bAvailable) return;
    Pages = MakeUnique<FVoxelOceanPages>();
    Pages->Initialize(InOwner, InAttachRoot, Settings.Material.Get(), Settings.Radius, Settings.TileQuads,
        VoxelOceanTiles::MorphEnd0(Settings.Radius, Settings.LodDistanceFactor), bFlipWinding);
}

void FVoxelOceanSurface::Reset()
{
    if (Pages.IsValid())
        Pages->Reset();
    Pages.Reset();
    Committed.Reset();
    CommittedSplit.Reset();
    bHasSelectionCamera = false;
    bFirstFill = true;
}

void FVoxelOceanSurface::SetMaterial(UMaterialInstanceDynamic* InMaterial)
{
    Settings.Material = InMaterial;
    bReportedMaterial = false;
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
    ParallelFor(Tiles.Num(), [&Tiles, &Out, Quads, Radius](int32 Index)
        {
            TArray<VoxelOceanTiles::FTileVertex> Vertices;
            VoxelOceanTiles::BuildTile(Tiles[Index], Quads, Radius, Vertices);
            FVoxelOceanPages::FTileWrite& Write = Out[Index];
            Write.Tile = Tiles[Index];
            Write.Words.SetNumUninitialized(Vertices.Num() * FVoxelOceanPages::VertexWords);
            for (int32 v = 0; v < Vertices.Num(); v++)
                FVoxelOceanPages::PackVertex(Vertices[v], &Write.Words[v * FVoxelOceanPages::VertexWords]);
        }, Tiles.Num() < VOS::ParallelBuildTiles ? EParallelForFlags::ForceSingleThread : EParallelForFlags::None);
}

void FVoxelOceanSurface::Tick(const FVector& Camera)
{
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

void FVoxelOceanSurface::UpdateMaterial()
{
    UMaterialInstanceDynamic* Material = Settings.Material.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!Material || !Root || !bHasSelectionCamera) return;

    // The camera as a float and the rest, so the material subtracts it at full precision.
    const FVector World = Root->GetComponentTransform().TransformPosition(SelectionCamera);
    const FVector3f High(World);
    const FVector Low = World - FVector(High);
    const int32 Mode = FMath::Clamp(CVarVoxelOceanMorph.GetValueOnGameThread(), 0, 2);
    const double StartFraction = VoxelOceanTiles::ClampMorphStartFraction(CVarVoxelOceanMorphStart.GetValueOnGameThread());
    Material->SetVectorParameterValue(VOS::CameraHighName, FLinearColor(High.X, High.Y, High.Z, 0.0f));
    Material->SetVectorParameterValue(VOS::CameraLowName, FLinearColor((float)Low.X, (float)Low.Y, (float)Low.Z, 0.0f));
    Material->SetVectorParameterValue(VOS::ParamsName, FLinearColor((float)Mode, (float)StartFraction, 0.0f, 0.0f));

    if (!bReportedMaterial)
    {
        // Ask the parent: the instance reports its own overrides (just set above) whether or not
        // the material has the parameters.
        FLinearColor Probe;
        UMaterialInterface* Parent = Material->Parent;
        const bool bWired = Parent
            && Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::CameraHighName), Probe)
            && Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::CameraLowName), Probe)
            && Parent->GetVectorParameterValue(FHashedMaterialParameterInfo(VOS::ParamsName), Probe);
        if (!bWired)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] %s has no vector parameters VoxelOceanCameraHigh, VoxelOceanCameraLow and VoxelOceanParams: the ocean's levels won't morph (stage O1 notes, material setup)."),
                *GetNameSafe(Material->Parent));
        }
        bReportedMaterial = true;
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
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] %d frames in %.2f s. Tiles: %.0f selected, %d on screen%s (levels 0-%d, factor %.2f). Select %.2f ms average, %.2f max; commit %.2f ms average, %.2f max; %d frames with a change, %d of them waiting for buffer sets (%d at most in a row). %s %s"),
            Perf.Frames, Now - Perf.Start, (double)Perf.Selected * PerFrame, Committed.Num(), bFirstFill ? TEXT(" (filling)") : TEXT(""),
            Settings.MaxLevel, Settings.LodDistanceFactor,
            Perf.SelectMs * PerFrame, Perf.MaxSelectMs, Perf.CommitMs * PerChange, Perf.MaxCommitMs, Perf.Changes, Perf.Waits, Perf.MaxWait,
            *Pages->Describe(), *PagePerf);
    }
    const int32 Wait = Perf.Wait;
    Perf = FPerf();
    Perf.Start = Now;
    Perf.Wait = Wait;
}
