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
// above at every corner is written degenerate (the land cull).

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanTiles.h"
#include <atomic>

class AActor;
class USceneComponent;
class UMaterialInterface;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class FVoxelOceanPageProvider;
class FVoxelOceanInputCache;
struct FVoxelDensityGPUData;
struct FVoxelEditGPUData;

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
    };

    /** Words per vertex in a tile write: position (3 floats), morph offset (3 floats), packed
     *  tangent and packed normal (snorm8 x4), the target's own morph offset (3 floats), the
     *  lattice point sampled for the depth (3 ints), and the tile indices of the morph target
     *  and of its own target (16 bits each). */
    static constexpr int32 VertexWords = 15;

    FVoxelOceanPages();
    ~FVoxelOceanPages();

    /** Drops every page; MorphEnd0 (world units) goes to the components' custom primitive data. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, UMaterialInterface* InMaterial, double InRadius, int32 InTileQuads,
        double InMorphEnd0, bool bInFlipWinding);

    /** Destroys every page and buffer set. */
    void Reset();

    /** Changes the material of every buffer set, pooled ones included. */
    void SetMaterial(UMaterialInterface* InMaterial);

    /** Changes where level 0's morph ends (world units) on every buffer set. */
    void SetMorphEnd0(double InMorphEnd0);

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

    /** Pages, tiles and buffer sets, for the debug log. */
    FString Describe() const;

    /** voxel.Debug.Ocean: work since the last call, and resets it. */
    FString TakePerf();

    /** Vertex words of a tile write for one vertex; Lattice: the lattice point its depth is
     *  sampled at. */
    static void PackVertex(const VoxelOceanTiles::FTileVertex& Vertex, const FIntVector& Lattice, uint32* OutWords);

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
    bool bFlipWinding = false;
    FVoxelOceanDensityInputs DensityInputs;
    /** Stand-ins for missing inputs: a density with no terrain (every vertex deep water) and
     *  no edits. */
    TSharedPtr<const FVoxelDensityGPUData> NoDensity;
    TSharedPtr<const FVoxelEditGPUData> NoEdits;
    /** The inputs uploaded once for every page's runs (render thread). */
    TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe> InputCache;
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
