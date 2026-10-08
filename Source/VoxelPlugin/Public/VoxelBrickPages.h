// VoxelBrickPages.h - The GPU backend of the brick terrain (Design/GPUPath.md stage c). Bricks
// that may hold surface are grouped into pages of 4x4x4 bricks of one level; each page is one
// RealtimeMesh component whose compute-writable buffers a compute provider fills in the frame:
// the stitched mesher (VoxelBrickMesherGPU) meshes every member brick and the page passes
// (VoxelBrickPageGPU.usf) pack the result and write the indirect draw arguments. One draw per
// page; nothing is read back except a few counters used to size the buffers.

#pragma once

#include "CoreMinimal.h"
#include "VoxelBrickTerrain.h"
#include "VoxelBrickMesherGPU.h"
#include <atomic>

class AActor;
class USceneComponent;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class FVoxelBrickPageProvider;
struct FVoxelBrickPageGPUInputs;

/** What a page's compute run lacked. Runs missing a Required item are skipped. */
enum class EVoxelPageMissing : uint32
{
    MesherShaders = 1 << 0,
    PageShaders = 1 << 1,
    Positions = 1 << 2,
    Tangents = 1 << 3,
    TexCoords = 1 << 4,
    Colors = 1 << 5,
    Triangles = 1 << 6,
    Args = 1 << 7,
    Capacity = 1 << 8,
    Required = MesherShaders | PageShaders | Positions | Triangles | Args | Capacity
};

/** What one run of a page's provider produced (read back a few frames later). */
struct FVoxelBrickPageStats
{
    uint64 Version = 0;
    uint32 VerticesNeeded = 0;
    uint32 TrianglesNeeded = 0;
    uint32 Errors = 0;             // OR of the mesher's error bits over the page's bricks
    uint32 VerticesWritten = 0;
    uint32 TrianglesWritten = 0;
    uint32 BricksDropped = 0;      // did not fit the page buffers
    uint32 VertexCapacity = 0;     // as the provider used them
    uint32 TriangleCapacity = 0;
    int32 Bricks = 0;
};

/** Game thread. The pages of one brick terrain. */
class FVoxelBrickPages
{
public:
    FVoxelBrickPages();
    ~FVoxelBrickPages();

    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelBrickTerrainSettings& InSettings);

    /** Destroys every page. */
    void Reset();

    /** Shows Key meshed with these neighbour levels from the next Flush. Returns the page
     *  version that holds it (see IsShown). */
    uint64 SetBrick(const FVoxelBrickKey& Key, const int8 (&NeighbourLevels)[27], uint32 MeshedNeighbours);
    void RemoveBrick(const FVoxelBrickKey& Key);

    /** True once the page holding Key has drawn Version or later. */
    bool IsShown(const FVoxelBrickKey& Key, uint64 Version) const;

    /** Once per frame: sends changed pages to the GPU with this density and these edits,
     *  creates, grows and swaps page buffers, collects the read-back counters. */
    void Flush(const TSharedPtr<const FVoxelDensityGPUData>& Density, const TSharedPtr<const FVoxelEditGPUData>& Edits);

    /** Pages, buffer memory and the latest counters, for the debug log. */
    FString Describe() const;

    /** The mesher settings for this terrain's lattice. */
    const FVoxelBrickMesherGPUSettings& GetMesherSettings() const { return MesherSettings; }

private:
    struct FInstance
    {
        TWeakObjectPtr<URealtimeMeshSimple> Mesh;
        TWeakObjectPtr<URealtimeMeshComponent> Component;
        TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe> Provider;
        TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> bBuffersReady;
        uint64 Handle = 0;             // compute registration, 0 until registered
        int32 VertexCapacity = 0;
        int32 TriangleCapacity = 0;
        uint64 CreatedFrame = 0;
        uint64 PushedVersion = 0;      // the newest state sent to the provider
        uint64 LastKickFrame = 0;
        uint64 RegisteredFrame = 0;
        bool bVisible = false;         // section shown (at registration)

        bool IsValid() const { return Provider.IsValid(); }
    };

    struct FMember
    {
        int8 Levels[27] = {};
        uint32 Meshed = 0;
    };

    struct FPage
    {
        TMap<FVoxelBrickKey, FMember> Members;
        FInstance Current;             // drawn
        FInstance Next;                // larger or smaller buffers, replaces Current once drawn
        uint64 NextVersion = 1;        // handed out by SetBrick, published by Flush
        uint64 Published = 0;
        bool bDirty = false;
        FVoxelBrickPageStats Stats;    // latest read back (from Current, or the instance it replaced)
        bool bStatsFromCurrent = false;
        bool bAwaitingStats = false;
    };

    static FVoxelBrickKey PageOf(const FVoxelBrickKey& Key);
    FInstance CreateInstance(const FVoxelBrickKey& PageKey, int32 VertexCapacity, int32 TriangleCapacity);
    static void DestroyInstance(FInstance& Instance);
    FIntPoint WantedCapacity(const FPage& Page) const;
    void Push(const FVoxelBrickKey& PageKey, const FPage& Page, FInstance& Instance) const;
    void Advance(FInstance& Instance) const;
    static FString DescribeMissing(uint32 Bits);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    FVoxelBrickTerrainSettings Settings;
    FVoxelBrickMesherGPUSettings MesherSettings;
    TMap<FVoxelBrickKey, FPage> Pages;   // keyed by level and page coordinate
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TSharedRef<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe> GPUInputs;
    uint32 WarnedMissing = 0;   // EVoxelPageMissing bits already logged
};
