// VoxelOceanSurface.h - The ocean surface of one ocean actor (Design/OceanMeshing.md sections 2
// and 3): every frame the CDLOD selection of the cube-sphere quadtree for the camera
// (VoxelOceanTiles), committed to the GPU pages (FVoxelOceanPages) in one frame when it
// changes, and the camera the committed selection was built for handed to the material for
// the morph (VoxelOcean.ush).
//
// A change waits, with the previous selection and its camera kept on screen, until every page
// it writes to has a buffer set ready; so the morph is always evaluated against the camera of
// the selection on screen, and boundaries between levels stay closed however long a commit
// waits. Only the first fill commits page by page, as pages get ready.
//
// Tiles sample the terrain depth when written (FVoxelOceanPages). A tile on screen is written
// again where the depth under it changed: all of them when the density, the lattice or the
// cull settings change, and those in an edit's reach once the terrain's GPU edit data holds
// that edit (the tile keeps its slot; nothing else moves).
//
// The waves (stage O6b): the surface holds the wave model the console chooses
// (voxel.Ocean.WaveModel) and, for the FFT model, drives its GPU cascades every tick
// (FVoxelOceanCascades, into the textures its settings carry) and hands the material the model's
// parameters and the textures.
//
// The weather (stage O7): for the FFT model the surface keys the inputs' environment source
// (FVoxelOceanWeatherKeys) into the weather table of the current interval, uploads it into the
// chart texture when it changes, and hands it to every frame (FVoxelOceanWaveContext::Weather) so
// the queries read what the GPU reads. The model's reach follows the weather's greatest
// long-wave speed, in steps.
//
// The shore (stage O8): the surface hands the shore field to the frames (the queries' shore
// source), its texture to the material and to the pages (the inland-basin cull), and keeps the
// depth probe around the camera for the underwater pass.

#pragma once

#include "CoreMinimal.h"
#include "Async/Future.h"
#include "VoxelOceanTiles.h"
#include "VoxelOceanPages.h"
#include "VoxelOceanWaves.h"

class AActor;
class USceneComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;
class UTexture2D;
class UTexture2DArray;
class FVoxelOceanCascades;
class FVoxelOceanWavesFFT;
class FVoxelOceanWeatherKeys;
class FVoxelOceanShoreField;
class FVoxelOceanCoastField;
struct FVoxelOceanCoastDepthJob;
struct FVoxelOceanProbeState;

/** The FFT cascades' textures (VoxelOceanCascades::MakeTextures): written every tick, read by
 *  the surface material and the underwater pass. Their owner keeps them alive. */
struct VOXELPLUGIN_API FVoxelOceanCascadeTextures
{
    UTexture2DArray* Displacement = nullptr;
    UTexture2DArray* Derivatives = nullptr;
    UTexture2D* Charts = nullptr;
    /** The open sea's foam per slot (stage O9b; W3w: the fine foam in slice 7). */
    UTexture2DArray* Foam = nullptr;
    /** Stage W3z: the shortest band's own 512^2 cascade, one slice each (its displacement,
     *  derivatives and fine foam; the arrays above hold its mip 1 in slice 7). */
    UTexture2DArray* FineDisplacement = nullptr;
    UTexture2DArray* FineDerivatives = nullptr;
    UTexture2DArray* FineFoam = nullptr;

    bool IsValid() const { return Displacement && Derivatives && Charts && Foam && FineDisplacement && FineDerivatives && FineFoam; }
};

/** The split bias near shores (stage O9c; VoxelOceanTiles::FShoreBias): within BandM of the coast
 *  (the shore field's distance) the mesh is as fine as if the camera were Detail times closer,
 *  the bias fading over the camera's own distance outside it and saving at most (1 - 1 / Detail)
 *  ReachM of distance. Detail 1: off. */
struct FVoxelOceanShoreDetail
{
    double Detail = 3.0;
    double BandM = 300.0;
    double ReachM = 500.0;
};

/** What the surface samples the terrain with, refreshed every frame (AOceanSphereActor). */
struct FVoxelOceanSurfaceInputs
{
    FVoxelOceanDensityInputs Density;
    uint32 EditSequence = 0;          // the newest edit Density.Edits holds
    /** The newest edit the depths carry (stage O9m): EditSequence, or with the land surface cache
     *  (Density.LandCache) the older of it and the cache's (its texels follow the edit data). */
    uint32 WrittenEditSequence = 0;
    double SlopeBound = 3.5;          // the steepest density gradient assumed (the land cull's reach)
    /** How far (cm) the land must stand above the sea, beyond what the slope bound allows for
     *  over a morphed triangle, before the triangle is culled. Negative: no cull. */
    double LandCullMargin = 500.0;
    /** The ocean clock (s) and the environment constants, this frame (no rewrite on change). */
    double Time = 0.0;
    FVoxelOceanEnvironment Environment;
    /** The weather (a procedural or constant source, VoxelOceanWeather.h; the same object until
     *  its settings change: a new one starts its keys over) and how the FFT model keys it. */
    TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> Weather;
    FVoxelOceanWeatherTiming WeatherTiming;
    /** The FFT model's settings (its wind speed, the reach's, is chosen here from the weather). */
    FVoxelOceanFFTSettings Waves;
    /** The shore field (stage O8; null: none) and its texture (the material's VoxelOceanShore).
     *  Density.Shore carries the texture to the pages, Density.ShoreMarginCm the inland basins'
     *  margin. */
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> Shore;
    UTexture2DArray* ShoreTexture = nullptr;
    /** The split bias near shores (stage O9c; live: the next selection follows it). */
    FVoxelOceanShoreDetail ShoreDetail;
    /** The coastal travel-time field (stage O9h; null: none): its layout goes to the shaders with
     *  the frame (rows 19 to 21 of the chart table's constants), its texels in ShoreTexture. */
    TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe> Coast;
};

/** Inputs of the ocean surface, fixed until the next Initialize (LodDistanceFactor and the
 *  material change live). */
struct FVoxelOceanSurfaceSettings
{
    double Radius = 0.0;                  // sea sphere (cm)
    int32 TileQuads = 16;                 // quads per tile edge: 8, 16 or 32
    int32 MaxLevel = 0;                   // finest tile level
    double LodDistanceFactor = 2.5;       // a node splits within this many tile sizes of the camera
    double HorizonHeight = 1000.0;        // tiles stay drawn over the horizon up to this height (cm)
    int32 SpareBufferSets = 8;            // buffer sets kept ready for commits
    /** The ocean's material instance: receives the selection camera every frame. */
    TWeakObjectPtr<UMaterialInstanceDynamic> Material;
    /** The FFT cascades' textures (none: the FFT waves stay off screen; queries still have them). */
    FVoxelOceanCascadeTextures CascadeTextures;
    /** The waves in compute (stage O9j): the pages displace their vertices once a frame and the
     *  material's vertex node passes them through (VoxelOceanParams.x 3). */
    bool bWavesInCompute = true;
    /** The wave clipmap (stage O9k; with the waves in compute): the composed sea evaluated once a
     *  frame on camera-centred levels the vertices and the material's pixel node sample. */
    FVoxelOceanClipSettings Clip;
};

class VOXELPLUGIN_API FVoxelOceanSurface
{
public:
    FVoxelOceanSurface();
    ~FVoxelOceanSurface();

    /** Game thread. Drops every tile and starts over. Without a renderer nothing is drawn. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelOceanSurfaceSettings& InSettings);

    /** Game thread. Destroys every page. */
    void Reset();

    /** Game thread. A new material instance for every page, without rebuilding. */
    void SetMaterial(UMaterialInstanceDynamic* InMaterial);

    /** Game thread. This frame's waves: the planet frame, the clock, the environment and the
     *  wave model (false before Initialize, or without an attach root). Density is left for
     *  the caller. */
    bool GetWaveFrame(FVoxelOceanWaveFrame& Out) const;

    /** Game thread. A new LOD distance factor (clamped to MinLodDistanceFactor); the next
     *  selection follows it. */
    void SetLodDistanceFactor(double InFactor);

    /** Game thread, before Tick. The terrain inputs; tiles on screen are written again when
     *  the density, the lattice or the cull settings change. */
    void SetInputs(const FVoxelOceanSurfaceInputs& InInputs);

    /** Game thread. An edit of the terrain (planet frame): the tiles on screen within its reach
     *  are written again once the inputs' edit data holds it. */
    void NotifyEdit(uint32 Sequence, const FVector& Center, double Radius);

    /** Game thread, every frame. Camera in the planet frame (actor space without scale). */
    void Tick(const FVector& Camera);

    const FVoxelOceanSurfaceSettings& GetSettings() const { return Settings; }

    /** Tiles on screen. */
    int32 NumTiles() const { return Committed.Num(); }

    /** The cascades' textures while the FFT model draws them (the underwater pass binds them);
     *  null otherwise. */
    const FVoxelOceanCascadeTextures* GetCascadeTextures() const;

    /** The weather's table of this frame (the FFT model's; null otherwise). */
    TSharedPtr<const FVoxelOceanWeatherTable, ESPMode::ThreadSafe> GetWeatherTable() const;

    /** The depth probe's result for the underwater pass (render thread data; null without pages). */
    TSharedPtr<FVoxelOceanProbeState, ESPMode::ThreadSafe> GetProbeState() const;

    /** Stage W3: the rings for the ocean's own pass (render thread object), while they draw the
     *  surface for it; null otherwise (the tiles, or the rings with the material). */
    TSharedPtr<IVoxelOceanRingDraw, ESPMode::ThreadSafe> GetRingDraw() const;

    /** Game thread. The coastal field's depth sampling (stage O9h; FVoxelOceanPages::
     *  SampleCoastDepths with this frame's terrain inputs). False: not yet. */
    bool SampleCoastDepths(const TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>& Job, int64 First, int32 Count);

private:
    VoxelOceanTiles::FSelectParams SelectParams() const;
    /** Builds the tiles' vertex words (in parallel when there are many). */
    /** The tiles' writes, their vertices' coast codes from Coast (stage O9c; null: none). */
    void BuildWrites(TConstArrayView<FVoxelOceanTileKey> Tiles, TArray<FVoxelOceanPages::FTileWrite>& Out,
        const TSharedPtr<const VoxelOceanTiles::FCoastSet, ESPMode::ThreadSafe>& Coast) const;
    /** Writes again the tiles on screen whose depth changed (SetInputs, NotifyEdit). */
    void ApplyRewrites();
    /** Writes the selection camera (world, as a float and the rest) and the morph parameters to
     *  the material instance. */
    /** The rings' land cull this frame (stage W2; FVoxelOceanDisplaceFrame::RingCull). */
    FVector4f RingCull() const;
    void UpdateMaterial();
    void LogPerf(double Now);
    /** The wave model the console chooses (voxel.Ocean.WaveModel) with this frame's settings and
     *  wind; a change of its reach writes the tiles on screen again (the land cull allows for
     *  it) and resizes the pages' bounds. */
    void UpdateWaveModel();
    /** The FFT model's weather table for this frame's time, uploaded into the chart texture when
     *  it changes. */
    void UpdateWeather();
    /** The depth probe around the camera (VoxelOceanDepthProbe.h) while the camera is near the
     *  sea, sampled again when a grid moves or the terrain changes. */
    void UpdateProbe(const FVector& Camera);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    FVoxelOceanSurfaceSettings Settings;
    TUniquePtr<FVoxelOceanPages> Pages;

    FVoxelOceanSurfaceInputs Inputs;
    /** Null: a flat sea. */
    TSharedPtr<const IVoxelOceanWaveModel, ESPMode::ThreadSafe> WaveModel;
    /** The same model when it is the FFT one. */
    TSharedPtr<const FVoxelOceanWavesFFT, ESPMode::ThreadSafe> FFTModel;
    /** The console's choice the model was built for (0 flat, 1 test, 2 FFT), and its settings. */
    int32 WaveModelKind = -1;
    FVoxelOceanWavesTest::FSettings WaveSettings;
    FVoxelOceanFFTSettings FFTSettings;
    double FFTRadius = 0.0;
    /** The waves' reach (cm, up and sideways) the land cull allows for (BuildWrites): the model's
     *  with headroom, so a changing wind doesn't write every tile again at each of its steps. */
    double CullHeight = 0.0;
    double CullReach = 0.0;
    /** The FFT model's GPU cascades (none without textures or a renderer). */
    TUniquePtr<FVoxelOceanCascades> Cascades;
    /** The FFT model's weather keys and table; what the chart texture holds. */
    TUniquePtr<FVoxelOceanWeatherKeys> WeatherKeys;
    uint64 UploadedTable = 0;       // its serial (FVoxelOceanWeatherTable::Serial)
    const UTexture2D* UploadedTexture = nullptr;
    /** The shore texture the material holds. */
    const UTexture2DArray* BoundShoreTexture = nullptr;
    /** The shore field's expansion, the surf's settings, the split bias and the coastal field's
     *  layout last written into the chart texture (rows 8 to 21 of its constants), and the table
     *  upload they were written after. */
    FVector4f WrittenShoreFrame[16];
    uint64 ShoreFrameUploads = 0;   // ChartUploads when it was written
    uint64 ChartUploads = 0;        // the weather's full uploads into the chart texture
    const UTexture2D* ShoreFrameTexture = nullptr;
    /** Writes the shore field's expansion around the selection camera into the chart texture. */
    void WriteShoreFrame(const FVoxelOceanWaveShaderParams& Waves);
    /** What the depth probe last sampled: its grids' anchors (VoxelOceanDepthProbe::Levels) and
     *  the inputs. */
    FVector ProbeAnchors[4];
    const void* ProbeDensity = nullptr;
    const void* ProbeEdits = nullptr;
    const void* ProbeShore = nullptr;
    uint32 ProbeSequence = 0;
    const void* ProbeLandCache = nullptr;
    double ProbeHalfFinest = 0.0;
    double ProbeDepthFootprint = 0.0;
    bool bProbed = false;
    bool bClipBound = false;
    bool bTexturesBound = false;
    /** Stage W3w: the ocean clock's time the cascades were last updated at (their fixed rate). */
    double CascadeTime = 0.0;
    bool bCascadeTime = false;
    /** Stage W3y: the cascades stepped this tick (the clipmap's regular evaluation follows them). */
    bool bCascadesStepped = true;
    bool bReportedFFT = false;
    bool bHasInputs = false;
    bool bRewriteAll = false;
    struct FPendingEdit
    {
        uint32 Sequence = 0;
        FVector Center = FVector::ZeroVector;   // planet frame
        double Reach = 0.0;                     // cm: a vertex nearer than this may sample it
    };
    TArray<FPendingEdit> PendingEdits;

    /** Tiles on screen and the nodes their selection split (the hysteresis); the camera they
     *  were selected for (planet frame). */
    TSet<FVoxelOceanTileKey> Committed;
    TSet<FVoxelOceanTileKey> CommittedSplit;
    FVector SelectionCamera = FVector::ZeroVector;
    bool bHasSelectionCamera = false;
    /** The split bias the tiles on screen were selected with (the chart table's row 17: Detail,
     *  band and reach in cm; Detail 1 none), and whether the material can follow one (the FFT
     *  waves drawn, the shore texture bound, the material has its parameter). */
    FVector4f SelectionBias = FVector4f(1.0f, 0.0f, 0.0f, 0.0f);
    bool bShoreDetailReady = false;
    /** The bias the next selection takes (SelectParams), as the chart table's row 17 holds it. */
    FVector4f BiasRow() const;
    /** The coast the split bias measures from (VoxelOceanTiles::FCoastSet, from the shore field:
     *  made off the game thread for each new field), the field it was made from, the build
     *  running; and the coast the tiles on screen carry the codes of (a new coast writes them
     *  all again with the next selection). */
    TSharedPtr<const VoxelOceanTiles::FCoastSet, ESPMode::ThreadSafe> Coast;
    const void* CoastField = nullptr;
    TFuture<TSharedPtr<const VoxelOceanTiles::FCoastSet, ESPMode::ThreadSafe>> CoastBuild;
    const void* CoastBuildField = nullptr;
    TSharedPtr<const VoxelOceanTiles::FCoastSet, ESPMode::ThreadSafe> SelectionCoast;
    /** Per node, the least coast distance the material sees over its cap (CoastCapBound), for
     *  Coast: nodes are selected again every frame. */
    mutable TMap<FVoxelOceanTileKey, double> CoastBounds;
    mutable TMap<FVoxelOceanTileKey, double> CoastBoundsOld;
    static double CoastBoundOf(const void* Context, const FVoxelOceanTileKey& Node, const FVector& CenterDir, double Angle);
    /** Starts or takes the coast set's build for the inputs' shore field. */
    void UpdateCoast();
    /** Nothing complete on screen yet: commits go page by page as pages get ready. */
    bool bFirstFill = true;
    bool bFlipWinding = false;
    bool bAvailable = false;
    bool bReportedMaterial = false;
    bool bReportedShape = false;

    struct FPerf
    {
        double Start = 0.0;
        int32 Frames = 0;
        double SelectMs = 0.0;
        double MaxSelectMs = 0.0;
        double CommitMs = 0.0;
        double MaxCommitMs = 0.0;
        int32 Changes = 0;             // frames whose selection differed from the one on screen
        int32 Rewritten = 0;           // tiles written again for depth changes
        int32 Waits = 0;               // of those, frames waiting for buffer sets
        int32 MaxWait = 0;
        int32 Wait = 0;
        int64 Selected = 0;
    };
    FPerf Perf;
};
