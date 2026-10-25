// VoxelOceanPages.h - The GPU pages of the ocean surface (Design/OceanMeshing.md section 3).
//
// A page holds up to 16 tiles of one level, the 4x4 under one quadtree node
// (VoxelOceanTiles::PageOf), in fixed-size slots of one RealtimeMesh buffer set: tile vertices
// and triangles are written by an on-demand compute provider (VoxelOceanPage.usf) and drawn
// indirect, up to the highest slot in use. Vertices are in page units (the component carries
// the page's unit as its scale), so every buffer set has the same size, a unit cube bounds any
// page, and a pooled set serves any page.
//
// A commit writes and clears tiles; every page it touches runs in the same frame's graph, so
// the surface switches from one selection to the next between two frames. A page that needs a
// buffer set first waits until one is registered and placed (PreparePages).
//
// Writing a tile also samples the terrain under it (stage O2): the planet density, with the
// terrain's edits, at the lattice point nearest each vertex, in the page's compute run (the
// terrain's own VoxelDensitySample.ush and VoxelEditSample.ush). The depth goes to the
// vertices' texture coordinates, morphed with them, and a triangle the land stands well
// above at every corner is written degenerate (the land cull). With the shore field (stage O8) a
// vertex in an inland basin below sea level counts as land. The same run's shader samples the
// depth probe the underwater pass reads (ProbeDepth).

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanTiles.h"
#include "UObject/StrongObjectPtr.h"
#include <atomic>

class AActor;
class USceneComponent;
class UMaterialInterface;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class FVoxelOceanPageProvider;
class FVoxelOceanInputCache;
class FTextureResource;
class UTexture2D;
class UTexture2DArray;
struct FVoxelOceanDisplaceState;
struct FVoxelOceanClipState;
struct FVoxelOceanClipPlan;
struct FVoxelOceanRingSet;
struct FVoxelDensityGPUData;
struct FVoxelEditGPUData;
struct FVoxelOceanShoreRHI;
struct FVoxelOceanLandCacheRHI;
struct FVoxelOceanProbeState;
struct FVoxelOceanCoastDepthJob;
class IVoxelOceanRingDraw;
namespace VoxelOceanDepthProbe { struct FLevels; }

/** What the page writes sample the terrain depth from. */
struct FVoxelOceanDensityInputs
{
    /** The planet density without edits (FVoxelDensityCPU::BuildGPUData). Null: no terrain,
     *  every vertex deep water. */
    TSharedPtr<const FVoxelDensityGPUData> Density;
    /** The edit tree as the terrain meshes with it. Null: no edits. */
    TSharedPtr<const FVoxelEditGPUData> Edits;
    /** The lattice step (cm): a vertex samples the point of the terrain lattice nearest it,
     *  counted in these from the planet centre (the edit tree is addressed the same way). */
    double HalfFinest = 50.0;
    /** The shore field's texture (stage O8, FVoxelOceanShoreField): a vertex in an inland basin
     *  (its field distance below minus ShoreMarginCm) is land. Null: no field, no such cull. */
    TSharedPtr<const FVoxelOceanShoreRHI, ESPMode::ThreadSafe> Shore;
    double ShoreMarginCm = 0.0;
    /** Stage W7a: the coastal field the shore texture holds (FVoxelOceanCoastField::Serial; 0:
     *  none). A window written into the texture in place keeps Shore: this tells the clipmap's
     *  static layer to write again. */
    uint32 CoastSerial = 0;
    /** The depth the waves feel (stage O9b2): the vertices' and the probe's depth leave out
     *  terrain detail finer than this (cm; the tile's own footprint where that is coarser). The
     *  land cull keeps the tile's footprint. 0: the tile's footprint. */
    double DepthFootprintCm = 0.0;
    /** The terrain's land surface cache (stage O9m; null: none, or not written for the camera):
     *  the depths read it wherever it holds the point, the density elsewhere. */
    TSharedPtr<const FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe> LandCache;
    /** The newest edit the cache's texels carry (FVoxelLandCache::GetSettledEditSequence). */
    uint32 LandEditSequence = 0;
};

/** The waves in compute (stage O9j; VoxelOceanDisplace.usf): what every page's per-frame
 *  displacement takes, as the material gets it this frame (FVoxelOceanSurface::UpdateMaterial). */
struct FVoxelOceanDisplaceFrame
{
    /** False: no displacement this frame (the pages keep what they last drew). */
    bool bOn = false;
    /** The planet frame in world space (the pages' attach root, without scale) and the selection
     *  camera in world space: the pages' vertices are morphed and waved from it. */
    FTransform PlanetToWorld = FTransform::Identity;
    FVector CameraWorld = FVector::ZeroVector;
    /** Level 0's morph end (world cm; the pages' own, set by SetDisplaceFrame). */
    double MorphEnd0 = 0.0;
    /** VoxelOceanParams with its real morph mode (x), the test waves, the planet centre from the
     *  selection camera, the FFT model's vectors (States.x 0 without the shore texture). */
    FVector4f Params = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f Wave[3] = { FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f), FVector4f(0.0f, 0.0f, 0.0f, 0.0f) };
    FVector3f Center = FVector3f::ZeroVector;
    FVector4f Shape = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f FFT = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f RefHigh = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f RefLow = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f Rotation = FVector4f(0.0f, 0.0f, 0.0f, 1.0f);
    FVector4f Tiles = FVector4f(1.0f, 1.0f, 1.0f, 1.0f);
    FVector4f States = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f SlopeVar = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    /** The rings' land cull (stage W2): x the height the land must stand above the sea at a vertex
     *  (cm, the margin and the waves'; negative: no cull), y the slope bound, z the reach a
     *  triangle adds besides its cells (cm: the waves' horizontal reach, the lattice step), w how
     *  far the far sphere sinks under the rings (cm). */
    FVector4f RingCull = FVector4f(-1.0f, 0.0f, 0.0f, 0.0f);
    /** The material's textures (null: stand-ins, and FFT.x 0). Read by the render command only. */
    FTextureResource* Displacement = nullptr;
    FTextureResource* Derivatives = nullptr;
    FTextureResource* Charts = nullptr;
    FTextureResource* Shore = nullptr;
    /** Stage W3u: the cascades' foam envelopes (VoxelOceanCascades' foam texture; null: none), for
     *  the ocean's own pass's foam per pixel. */
    FTextureResource* Foam = nullptr;
    /** Stage W3z: the shortest band's own cascade (VoxelOceanCascades' fine arrays; null: none),
     *  for the ocean's own pass. */
    FTextureResource* FineDisplacement = nullptr;
    FTextureResource* FineDerivatives = nullptr;
    FTextureResource* FineFoam = nullptr;
};

/** The wave clipmap (stages O9k, W1; VoxelOceanClip.ush, VoxelOceanClipmap.h): the composed sea
 *  evaluated on up to Levels camera-centred levels of Size^2 texels (as many as reach past the
 *  horizon), the first's spacing Finest cm or coarser with the camera's height, from a static
 *  layer of what the terrain and the shore give each texel (written as the windows reach it); the
 *  pages' vertices (with the waves in compute) and the material's pixel node (VoxelOceanPixelClip)
 *  sample them. */
struct FVoxelOceanClipSettings
{
    bool bOn = true;
    int32 Size = 128;
    int32 Levels = 12;
    double FinestCm = 20.0;
    /** Stage W2: the mesh as rings on the clipmap's levels and a far sphere (VoxelOceanRings.usf)
     *  instead of the tile pages, whenever the clipmap runs; the far sphere's cells a face edge. */
    bool bRings = true;
    int32 FarCells = 128;
    /** Stage W6f: above this camera height (cm) the rings draw nothing and the far sphere the
     *  whole sea (their waves under a pixel; the outermost ring's square, its cells stretched by
     *  the plane, sagged below the sphere and showed from orbit). Back from 0.9 of it. 0: always. */
    double RingsMaxHeightCm = 4.0e6;
    /** Stage W6f part 2: above this camera height (cm), or once the clipmap's plane bounds its
     *  coarsest level short of the horizon, the clipmap is off (no static layer, waves or foam
     *  passes): the far sphere is shaded by the sea state's statistics alone. Back from 0.9 of the
     *  height it went off at. 0: by the plane alone. */
    double ClipMaxHeightCm = 3.0e7;
    /** Stage W3: the rings drawn by the ocean's own pass (no mesh component; GetRingDraw) instead
     *  of with the ocean material. */
    bool bOwnPass = true;

    bool operator==(const FVoxelOceanClipSettings& O) const
    {
        return bOn == O.bOn && Size == O.Size && Levels == O.Levels && FinestCm == O.FinestCm && bRings == O.bRings && FarCells == O.FarCells
            && bOwnPass == O.bOwnPass && RingsMaxHeightCm == O.RingsMaxHeightCm && ClipMaxHeightCm == O.ClipMaxHeightCm;
    }
    bool operator!=(const FVoxelOceanClipSettings& O) const { return !(*this == O); }
};

/** Game thread. The pages of one ocean surface. */
class FVoxelOceanPages
{
public:
    /** One tile to write: its vertices, VertexWords words each, in VoxelOceanTiles::BuildTile
     *  order (PackVertex). */
    struct FTileWrite
    {
        FVoxelOceanTileKey Tile;
        TArray<uint32> Words;
        /** The density is sampled leaving out detail finer than this (cm): the tile's spacing. */
        float Footprint = 0.0f;
        /** A triangle is culled when the land stands above the sea by more than this (cm) at
         *  each corner (FLT_MAX: never). */
        float CullHeight = 0.0f;
        /** The level's nominal spacing (cm; stage O9b2): the depth the waves feel leaves out
         *  detail finer than it (or DepthFootprintCm), the same for every tile of the level. */
        float Nominal = 0.0f;
    };

    /** Words per vertex in a tile write: position (3 floats), morph offset (3 floats), packed
     *  tangent and packed normal (snorm8 x4), the target's own morph offset (3 floats), the
     *  lattice point sampled for the depth (3 ints), the tile indices of the morph target and of
     *  its own target (16 bits each), and the split bias's coast codes of the vertex and its
     *  target (stage O9c: VoxelOceanTiles::CoastPack, a float; TexCoord 2 y). */
    static constexpr int32 VertexWords = 16;

    FVoxelOceanPages();
    ~FVoxelOceanPages();

    /** Drops every page; MorphEnd0 (world units) goes to the components' custom primitive data. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, UMaterialInterface* InMaterial, double InRadius, int32 InTileQuads,
        double InMorphEnd0, bool bInFlipWinding);

    /** Destroys every page and buffer set. */
    void Reset();

    /** The waves in compute (stage O9j): each page displaces its vertices every frame
     *  (VoxelOceanDisplace.usf) from SetDisplaceFrame's inputs, and the material's vertex node
     *  passes them through (VoxelOceanParams.x 3). A change drops every page (they register
     *  again with the driver's every-frame trigger, or on demand). */
    void SetWavesInCompute(bool bInCompute);
    bool IsWavesInCompute() const { return bWavesInCompute; }

    /** Game thread, once a frame with the waves in compute: this frame's displacement inputs, for
     *  every page's run in this frame's rendering. */
    void SetDisplaceFrame(const FVoxelOceanDisplaceFrame& Frame);

    /** The wave clipmap's settings (stage O9k): makes its textures (the material binds them:
     *  GetClipTextures). Off: none, the vertices evaluate the waves themselves. */
    void SetClipmap(const FVoxelOceanClipSettings& InSettings);
    bool IsClipmapOn() const { return ClipSettings.bOn && ClipDisplacement.IsValid(); }
    UTexture2DArray* GetClipDisplacement() const { return ClipDisplacement.Get(); }
    UTexture2DArray* GetClipNormal() const { return ClipNormal.Get(); }
    UTexture2D* GetClipInfo() const { return ClipInfo.Get(); }

    /** Game thread, once a frame after SetDisplaceFrame, with the clipmap on: the levels around
     *  the selection camera (planet frame) evaluated with Waves (the material's parameters and
     *  textures) and the open sea's foam texture; their beds sampled again where they moved or the
     *  terrain changed. */
    /** OceanTime: the ocean's clock (s); FoamDecayPerSecond and FoamGrowthPerSecond: the foam's
     *  rates (stage W3v, VoxelOceanFoam.ush: GodotOceanWaves' rule; a decay of 0: no foam buffer).
     *  bWavesStepped (stage W3y): the cascades stepped this frame; else only the levels that must
     *  be (new, edited, moved past their slack) are evaluated, the others wait for the next step. */
    /** FoamDetailLevels (stage W4c): how many levels around the camera keep the foam's fine history
     *  (VoxelOceanClip.usf, ClipFineFoamCS; 0: none, at most VoxelOceanClipmap::FineMaxLevels).
     *  FoamCutCm (stage W5f): texels further than this from the camera (on the plane) skip the
     *  foam (the own pass has faded it out there; 0: none). */
    void UpdateClipmap(const FVector& CameraPlanet, const FVoxelOceanDisplaceFrame& Waves, FTextureResource* Foam,
        double OceanTime = 0.0, double FoamDecayPerSecond = 0.0, double FoamGrowthPerSecond = 0.0, bool bWavesStepped = true,
        int32 FoamDetailLevels = 0, double SurfFoamDecayPerSecond = 0.4, double FoamBreakup = 0.0, double FoamCutCm = 0.0,
        bool bFineFromClip = true);

    /** Game thread (stage W1). An edit's sphere (planet frame, cm) that the depths now carry (the
     *  surface's: once the terrain's edit data, and the land surface cache, hold it): the static
     *  layer writes it again. */
    void NotifyClipEdit(const FVector& Center, double Reach);
    /** Game thread. The static layer and the waves written again everywhere (the edits replaced). */
    void InvalidateClip();
    /** The clipmap's last second (the surface's log): levels, static texels written, re-anchors. */
    FString TakeClipPerf();

    /** Stage W2: the rings can draw the surface (the setting on, the clipmap on with the waves in
     *  compute, the clipmap's size big enough for rings). */
    bool CanDrawRings() const;
    /** Game thread. The rings on (made: one mesh for every level and the far sphere, which the
     *  clipmap's frames then fill; the tile pages are the surface's to drop) or off (destroyed). */
    void SetRings(bool bOn);
    bool IsRingsOn() const;
    /** Game thread, every frame with the rings on (after the clipmap's update): their origin and
     *  frame around the camera (planet frame), their land cull (FVoxelOceanDisplaceFrame::
     *  RingCull). Without this frame's clipmap the far sphere still draws. */
    void UpdateRings(const FVector& CameraPlanet, const FVector4f& Cull);
    /** Stage W3: the rings for the ocean's own pass (render thread object; null unless the rings
     *  are on for the own pass). */
    TSharedPtr<IVoxelOceanRingDraw, ESPMode::ThreadSafe> GetRingDraw() const;

    /** Changes the material of every buffer set, pooled ones included. */
    void SetMaterial(UMaterialInterface* InMaterial);

    /** Changes where level 0's morph ends (world units) on every buffer set. */
    void SetMorphEnd0(double InMorphEnd0);

    /** How far (cm) the waves move a vertex in any direction (the wave model's reach, its
     *  vertical and horizontal parts together): a page's bounds grow past the unit cube when its
     *  own slack (0.23 of its unit) is less. */
    void SetWaveReach(double InReach);

    /** The density and edits the next writes sample. Writes already pushed keep theirs. */
    void SetDensityInputs(const FVoxelOceanDensityInputs& InInputs);
    const FVoxelOceanDensityInputs& GetDensityInputs() const { return DensityInputs; }

    /** True when every page in InPages has a buffer set ready to draw in this frame; otherwise
     *  gets more ready (within voxel.Ocean.PageCreates new ones per frame) and returns false. */
    bool PreparePages(const TSet<FVoxelOceanTileKey>& InPages);

    /** True when the page has a buffer set ready to draw in this frame. */
    bool IsPageReady(const FVoxelOceanTileKey& Page) const;

    /** Writes and clears tiles. Every page touched runs in this frame's graph. The pages Writes
     *  go to must be ready (PreparePages). */
    void Commit(TConstArrayView<FTileWrite> Writes, TConstArrayView<FVoxelOceanTileKey> Clears);

    /** Once per frame, before any commit: registration, kicks, pooling released buffer sets,
     *  dropping unused empty pages, and keeping SpareTarget buffer sets ready. */
    void Flush(int32 SpareTarget);

    /** Samples the terrain depth on the depth probe's grids (VoxelOceanDepthProbe.h) with the
     *  density inputs set now, in a graph of its own; the result replaces the probe state's on the
     *  render thread. Radius: the sea sphere (cm). */
    void ProbeDepth(const VoxelOceanDepthProbe::FLevels& Levels, double InRadius);

    /** Game thread. Samples the terrain's depth at Count of a coastal field job's fine cells from
     *  First (stage O9h, VoxelOceanCoastField.h: VoxelOceanPage.usf's CoastDepthCS), read back
     *  into the job on the render thread. False when it can't yet (no terrain density, or no
     *  rendering): nothing is enqueued. */
    bool SampleCoastDepths(const TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>& Job, int64 First, int32 Count, double InRadius);

    /** The depth probe's result (render thread data; the underwater pass reads it). */
    const TSharedPtr<FVoxelOceanProbeState, ESPMode::ThreadSafe>& GetProbeState() const { return ProbeState; }

    /** Pages, tiles and buffer sets, for the debug log. */
    FString Describe() const;

    /** voxel.Debug.Ocean: work since the last call, and resets it. */
    FString TakePerf();

    /** Vertex words of a tile write for one vertex; Lattice: the lattice point its depth is
     *  sampled at; Coast: its coast codes (VoxelOceanTiles::CoastPack). */
    static void PackVertex(const VoxelOceanTiles::FTileVertex& Vertex, const FIntVector& Lattice, float Coast, uint32* OutWords);

private:
    struct FInstance
    {
        TWeakObjectPtr<URealtimeMeshSimple> Mesh;
        TWeakObjectPtr<URealtimeMeshComponent> Component;
        TSharedPtr<FVoxelOceanPageProvider, ESPMode::ThreadSafe> Provider;
        TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> bBuffersReady;
        uint64 Handle = 0;             // compute registration, 0 until registered
        uint64 CreatedFrame = 0;
        uint64 PushedVersion = 0;      // the newest state sent to the provider
        uint64 LastKickFrame = 0;
        uint64 RegisteredFrame = 0;
        uint64 ReadyFrame = 0;         // not filled before (a moved instance waits a frame)

        bool IsValid() const { return Provider.IsValid(); }
    };

    struct FPage
    {
        FInstance Instance;
        /** The tile in each slot (FreeSlot when none). Tiles take the lowest free slot, and a
         *  page draws up to its highest slot in use. */
        FVoxelOceanTileKey Slots[VoxelOceanTiles::PageSlots];
        int32 NumTiles = 0;
        uint64 WantedFrame = 0;        // the last frame a commit wanted this page

        FPage()
        {
            for (FVoxelOceanTileKey& Slot : Slots)
                Slot = FreeSlot();
        }
    };

    /** The key of a free slot (no tile has level 255). */
    static FVoxelOceanTileKey FreeSlot() { return FVoxelOceanTileKey(0, 255, 0, 0); }

    /** A buffer set emptied by a commit: pooled once that run is recorded. */
    struct FReleasing
    {
        FInstance Instance;
        uint64 Version = 0;
        uint64 Frame = 0;
    };

    FInstance CreateInstance();
    static void DestroyInstance(FInstance& Instance);
    /** Moves and scales the instance onto the page, with the page's level for the morph. */
    void PlaceInstance(const FVoxelOceanTileKey& PageKey, FInstance& Instance) const;
    /** A pooled buffer set or a new one (within the creation budget), placed on the page;
     *  invalid when none could be had this frame. */
    FInstance AcquireInstance(const FVoxelOceanTileKey& PageKey);
    void PoolInstance(FInstance& Instance);
    static bool IsReady(const FInstance& Instance);
    void Advance(FInstance& Instance) const;
    /** Sends a state (write and clear jobs, slots drawn) to the instance's provider. */
    void Push(const FVoxelOceanTileKey& PageKey, FInstance& Instance, TArray<uint32>&& Jobs, TArray<uint32>&& Words, int32 DrawSlots);
    void BeginCreates();
    void CollectCounters(const FInstance& Instance);
    static int32 FindSlot(const FPage& Page, const FVoxelOceanTileKey& Tile);
    static int32 DrawSlotsOf(const FPage& Page);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    TWeakObjectPtr<UMaterialInterface> Material;
    double Radius = 0.0;
    int32 TileQuads = 16;
    double MorphEnd0 = 0.0;
    double WaveReach = 0.0;
    bool bFlipWinding = false;
    bool bWavesInCompute = false;
    /** The displacement's inputs this frame (render thread; stage O9j). */
    TSharedPtr<FVoxelOceanDisplaceState, ESPMode::ThreadSafe> DisplaceState;
    /** The wave clipmap (stages O9k, W1): its settings, textures, its plan (the levels, what each
     *  slot holds), the density inputs' version (the static layer is written again when it
     *  changes), its render state. */
    FVoxelOceanClipSettings ClipSettings;
    TStrongObjectPtr<UTexture2DArray> ClipDisplacement;
    TStrongObjectPtr<UTexture2DArray> ClipNormal;
    /** Stage W3h: the whitecaps' density per level; stage W3u: PF_FloatRGBA, with the foam, the
     *  surf's zone and the open sea's source (VoxelOceanClip.usf; the own pass reads it). */
    TStrongObjectPtr<UTexture2DArray> ClipWhitecap;
    TStrongObjectPtr<UTexture2D> ClipInfo;
    TUniquePtr<FVoxelOceanClipPlan> ClipPlan;
    /** Stage W6f: the rings off (the camera above RingsMaxHeightCm; hysteresis). */
    bool bRingsHigh = false;
    /** Stage W6f part 2: the clipmap off, and the height it went off at. */
    bool bClipHigh = false;
    double ClipOffHeightCm = 0.0;
    /** The rings (stage W2; null: off). */
    TUniquePtr<FVoxelOceanRingSet> Rings;
    void DestroyRings();
    /** Game thread, once a frame with the clipmap's: registers the rings' provider once it can
     *  run, moves their origin when the camera is far from it, samples the far sphere's land again
     *  when the terrain changed. Returns the camera from the origin (planet frame, cm). */
    FVector AdvanceRings(const FVector& CameraPlanet);
    uint32 DensityVersion = 1;
    TSharedPtr<FVoxelOceanClipState, ESPMode::ThreadSafe> ClipState;
    FVoxelOceanDensityInputs DensityInputs;
    /** Stand-ins for missing inputs: a density with no terrain (every vertex deep water) and
     *  no edits. */
    TSharedPtr<const FVoxelDensityGPUData> NoDensity;
    TSharedPtr<const FVoxelEditGPUData> NoEdits;
    /** The inputs uploaded once for every page's runs (render thread). */
    TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe> InputCache;
    /** The depth probe's last result (render thread). */
    TSharedPtr<FVoxelOceanProbeState, ESPMode::ThreadSafe> ProbeState;
    TMap<FVoxelOceanTileKey, FPage> Pages;
    TArray<FInstance> Pool;
    TArray<FReleasing> Releasing;
    uint64 NextVersion = 1;
    int32 CreatesThisFrame = 0;
    uint64 CreatesFrame = 0;
    uint32 WarnedMissing = 0;

    struct FPerf
    {
        int32 Frames = 0;
        int32 Commits = 0;
        int32 TilesWritten = 0;
        int32 TilesCleared = 0;
        int32 PagesPushed = 0;
        int32 Created = 0;
        double CreateMs = 0.0;
        double MaxCreateMs = 0.0;
        int32 Recycled = 0;
        int64 Calls = 0;
        int64 IdleCalls = 0;
        int64 Runs = 0;
        int64 RenderMicros = 0;
    };
    FPerf Perf;
};
