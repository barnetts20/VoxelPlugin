#include "VoxelBrickPages.h"
#include "VoxelPlugin.h"
#include "HAL/IConsoleManager.h"
#include "Misc/ScopeLock.h"
#include "RealtimeMeshSimple.h"
#include "RealtimeMeshComponent.h"
#include "Compute/RealtimeMeshComputeProvider.h"
#include "GameFramework/Actor.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "RenderingThread.h"
#include "SceneView.h"
#include "SceneViewExtension.h"
#include "Misc/App.h"
#include "Engine/Engine.h"
#include "Misc/ScopeExit.h"
#include "VoxelTerrainStats.h"

DECLARE_CYCLE_STAT(TEXT("Pages: flush"), STAT_VoxelPagesFlush, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: prepare"), STAT_VoxelPagesPrepare, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: commit"), STAT_VoxelPagesCommit, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: create buffer set"), STAT_VoxelPagesCreate, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: provider (render thread)"), STAT_VoxelPagesProvider, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: staging (render thread)"), STAT_VoxelPagesStaging, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: runs"), STAT_VoxelPagesRuns, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: bricks meshed"), STAT_VoxelPagesBricksMeshed, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: idle provider calls"), STAT_VoxelPagesIdleCalls, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: buffer sets created"), STAT_VoxelPagesCreated, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: buffer sets recycled"), STAT_VoxelPagesRecycled, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Pages"), STAT_VoxelPagesCount, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Pages: pooled buffer sets"), STAT_VoxelPagesPooled, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Pages: staged bricks"), STAT_VoxelPagesStaged, STATGROUP_VoxelTerrain);
DECLARE_GPU_STAT(VoxelBrickMesher);
DECLARE_GPU_STAT(VoxelBrickPageWrite);

using namespace RealtimeMesh;

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelStageParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, MesherCounters)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, MesherVertices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, MesherIndices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, StageBricks)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, CacheVertices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, CacheIndices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, StageEntries)
    SHADER_PARAMETER(uint32, NumBricks)
    SHADER_PARAMETER(uint32, VertexCap)
    SHADER_PARAMETER(uint32, TriangleCap)
    SHADER_PARAMETER(uint32, VertexStride)
    SHADER_PARAMETER(uint32, RegionVertexStart)
    SHADER_PARAMETER(uint32, RegionVertices)
    SHADER_PARAMETER(uint32, RegionTriangleStart)
    SHADER_PARAMETER(uint32, RegionTriangles)
END_SHADER_PARAMETER_STRUCT()

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelWriteParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, CachedVertices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, CachedIndices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, Jobs)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float>, OutPositions)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTangents)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutColors)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutIndices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutArgs)
    SHADER_PARAMETER(uint32, NumJobs)
    SHADER_PARAMETER(uint32, TriangleEnd)
    SHADER_PARAMETER(uint32, VertexCapacity)
    SHADER_PARAMETER(uint32, TriangleCapacity)
    SHADER_PARAMETER(uint32, Color)
    SHADER_PARAMETER(uint32, StreamMask)
END_SHADER_PARAMETER_STRUCT()

#define VOXEL_PAGE_SHADER(ShaderName, ParametersType) \
    class ShaderName : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(ShaderName); \
        using FParameters = ParametersType; \
        SHADER_USE_PARAMETER_STRUCT(ShaderName, FGlobalShader); \
        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
        { \
            return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
        } \
    };

VOXEL_PAGE_SHADER(FVoxelPageStageCS, FVoxelStageParameters)
VOXEL_PAGE_SHADER(FVoxelPageWriteCS, FVoxelWriteParameters)
#undef VOXEL_PAGE_SHADER

IMPLEMENT_GLOBAL_SHADER(FVoxelPageStageCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "StageCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageWriteCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "WriteCS", SF_Compute);

static TAutoConsoleVariable<int32> CVarVoxelBrickPageCreates(
    TEXT("voxel.Render.BrickPageCreates"), 4,
    TEXT("Brick page buffer sets created per frame at most (recycled ones don't count)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelBrickMeshBatches(
    TEXT("voxel.Render.BrickMeshBatches"), 2,
    TEXT("Brick mesher batches sent per frame (128 bricks of 16 cells, fewer of larger ones), on async compute. The bricks a task changes are meshed at this rate ahead of their commit; four times as fast while nothing is on screen."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelBrickMeshBatchesBusy(
    TEXT("voxel.Render.BrickMeshBatchesBusy"), 4,
    TEXT("Brick mesher batches sent per frame while a newer task result waits for the commit being meshed (the terrain is behind the camera). At least voxel.Render.BrickMeshBatches."),
    ECVF_Default);

namespace VoxelBrickPagesDetail
{
    /** What a page's compute run lacked. Runs missing a Required item are skipped. */
    enum class EMissing : uint32
    {
        PageShaders = 1 << 0,
        Positions = 1 << 1,
        Tangents = 1 << 2,
        TexCoords = 1 << 3,
        Colors = 1 << 4,
        Triangles = 1 << 5,
        Args = 1 << 6,
        Capacity = 1 << 7,
        Required = PageShaders | Positions | Triangles | Args | Capacity
    };

    constexpr int32 PageShift = 3;                   // pages of 8x8x8 bricks
    constexpr int32 BatchCells = 128 * 4096;         // cells per mesher batch (128 16-cell bricks)
    constexpr int32 MaxPooled = 32;                  // emptied buffer sets kept for reuse
    constexpr int32 PoolSlack = 2;                   // a pooled set may be this many classes larger on each count
    /** Page bounds beyond its bricks (cells of its level): the morph moves vertices toward
     *  coarser cells' vertices, no farther than the mesher reads. */
    constexpr double BoundsMarginCells = (double)VoxelBrickMesherGPU::TargetReachCells;
    constexpr int32 MinVertices = 4096;
    constexpr int32 MinTriangles = 8192;
    constexpr int32 SlabVertices = 1 << 16;          // a buffer set's size limit: 4 MB of vertices
    constexpr int32 SlabTriangles = 1 << 17;         // and 1.5 MB of triangles
    constexpr uint64 KickFrames = 3;                 // frames before a run that didn't happen is asked for again
    constexpr uint64 UnusedFrames = 60;              // an unused spare is pooled after
    constexpr uint64 EmptyingTimeoutFrames = 60;     // an emptying run not recorded by then: destroyed instead
    constexpr uint64 CacheTimeoutFrames = 600;       // cache ranges of runs never recorded are freed after
    constexpr double CapacityMargin = 1.25;          // over the counts a new buffer set holds
    constexpr double PlanarVertices = 1.0;           // a brick before any read back: N^2 vertices
    constexpr double PlanarTriangles = 2.0;          // and 2 N^2 triangles
    constexpr int32 CreateRampFrames = 4;            // a waiting commit raises the creation budget every this many frames
    constexpr int32 MaxCreateRamp = 3;               // creation costs about 0.3 ms each on the game thread
    constexpr double UnknownBrickWeight = 1.5;       // a brick not yet meshed counts this many level averages
    constexpr double AverageBlend = 0.05;            // per surface brick read back
    constexpr double RegionMargin = 1.25;            // a batch's cache region over its expected counts
    constexpr int32 RegionAlign = 256;
    constexpr int32 MaxBatchesInFlight = 16;
    constexpr int32 FirstFillBatchScale = 4;
    constexpr int32 InitialCacheVertices = 1 << 18;
    constexpr int32 InitialCacheTriangles = 1 << 19;
    constexpr int32 MaxCacheVertices = 1 << 23;
    constexpr int32 MaxCacheTriangles = 1 << 24;
    constexpr uint64 CachePeakFrames = 300;          // window of the cache's peak use
    constexpr double CacheShrinkMargin = 1.5;        // an idle cache shrinks to a power of two above this times its peak
    constexpr uint32 CacheWords = 22;                // VOXEL_CACHE_WORDS (VoxelBrickPageGPU.usf)
    constexpr uint32 EntryWords = 8;                 // VOXEL_ENTRY_WORDS
    constexpr uint32 JobWords = 8;                   // VOXEL_JOB_WORDS
    constexpr int32 TexCoordChannels = 8;
    constexpr int32 BytesPerVertex = 12 + 8 + 64 + 4;   // position, two packed normals, eight float2 UVs, colour
    constexpr int32 BytesPerTriangle = 12;

    const FName PageGroupName(TEXT("BRICKPAGE"));

    /** Bricks per mesher batch: the scratch buffers grow with the cells per brick (about 75 MB
     *  per batch at 16 cells). */
    int32 BatchBricksFor(const FVoxelBrickMesherGPUSettings& Settings)
    {
        return FMath::Clamp(BatchCells / FMath::Cube(VoxelBrickMesherGPU::CellsFor(Settings)), 8, 256);
    }

    /** The capacity class holding Count: classes 2k and 2k - 1 are 2^k and 1.5 * 2^(k-1). */
    int32 SizeClass(int32 Count)
    {
        const int32 K = (int32)FMath::CeilLogTwo((uint32)FMath::Max(Count, 4));
        return Count <= (3 << (K - 2)) ? 2 * K - 1 : 2 * K;
    }

    int32 ClassSize(int32 Class)
    {
        return (Class & 1) ? 3 << (Class / 2 - 1) : 1 << (Class / 2);
    }

    uint64 PoolKey(int32 VertexClass, int32 TriangleClass)
    {
        return ((uint64)(uint32)VertexClass << 32) | (uint64)(uint32)TriangleClass;
    }

    FVector PageOrigin(const FVoxelBrickTerrainSettings& Settings, const FVoxelBrickKey& PageKey)
    {
        return Settings.GridFor(PageKey.Level).BrickMin(PageKey.Coord * (1LL << PageShift));
    }

    FRealtimeMeshBufferSetKey GroupKey()
    {
        return FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), PageGroupName);
    }

    FRealtimeMeshSectionKey SectionKey()
    {
        return FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey(), 0);
    }

    /** Where level 0's morph band ends (cm; level l's is 2^l times it, VoxelMorph.ush). With F
     *  = LodDistanceFactor, B the brick size and h the cell of a level, a level-l vertex can
     *  meet level l + 1 (a seam, or a merge) no nearer than 2F * B, less a cell diagonal for
     *  cells just outside a brick, so it must be fully blended from 2F * B - 2 * sqrt(3) * h.
     *  With chained targets the band can start where level l - 1's ends. */
    double MorphBandEnd0(const FVoxelBrickTerrainSettings& InSettings)
    {
        const double Cell = InSettings.FinestCell;
        return 2.0 * InSettings.LodDistanceFactor * InSettings.CellsPerBrick * Cell - 2.0 * UE_DOUBLE_SQRT_3 * Cell;
    }

    bool ArePageShadersReady()
    {
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        return ShaderMap
            && ShaderMap->GetShader(&FVoxelPageStageCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageWriteCS::GetStaticType(), 0).IsValid();
    }

    FRDGBufferRef OneWord(FRDGBuilder& GraphBuilder, const TCHAR* Name)
    {
        static const uint32 Zero = 0u;
        return CreateStructuredBuffer(GraphBuilder, Name, sizeof(uint32), 1, &Zero, sizeof(uint32));
    }
}

namespace VBP = VoxelBrickPagesDetail;

/** Render thread. One terrain's mesher work: the density inputs on the GPU, the mesh cache, and
 *  the batches on their way through it (queued, meshed, read back). */
struct FVoxelBrickPageGPU
{
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TRefCountPtr<FRDGPooledBuffer> Octaves;
    TRefCountPtr<FRDGPooledBuffer> EditCorners;
    TRefCountPtr<FRDGPooledBuffer> EditChildren;
    TRefCountPtr<FRDGPooledBuffer> EditFlags;

    /** The cache: VBP::CacheWords words per vertex, 3 per triangle. The game thread places
     *  everything in it and asks for its size. */
    TRefCountPtr<FRDGPooledBuffer> CacheVertices;
    TRefCountPtr<FRDGPooledBuffer> CacheIndices;
    int32 CacheVertexCapacity = 0;
    int32 CacheTriangleCapacity = 0;
    int32 WantedVertexCapacity = 0;
    int32 WantedTriangleCapacity = 0;

    struct FWork
    {
        uint32 Id = 0;
        TArray<FVoxelMesherBrickGPU> Bricks;
        TArray<FVector4f> Places;                 // two per brick (StageBricks)
        FIntPoint Vertices = FIntPoint::ZeroValue;   // cache regions: start, count
        FIntPoint Triangles = FIntPoint::ZeroValue;
        TSharedPtr<const FVoxelDensityGPUData> Density;
        TSharedPtr<const FVoxelEditGPUData> Edits;
        FVoxelBrickMesherGPUSettings Settings;
        FVoxelBrickMesherGPUDebugView DebugView;
        TRefCountPtr<FRDGPooledBuffer> Entries;
        TSharedPtr<FRHIGPUBufferReadback> Readback;
    };
    TArray<FWork> Queued;       // from the game thread, for the next graph
    TArray<FWork> Dispatched;   // meshed in an earlier graph; their entries are copied in the next
    TArray<FWork> Reading;

    struct FResult
    {
        uint32 Id = 0;
        TArray<uint32> Entries;   // VBP::EntryWords per brick, empty if the readback failed
    };
    FCriticalSection ResultsCS;
    TArray<FResult> Results;      // to the game thread

    uint32 LastFrame = ~0u;
    std::atomic<int64> StageMicros{ 0 };
    std::atomic<int32> BricksMeshed{ 0 };

    VoxelBrickMesherGPU::FInputs GetInputs(FRDGBuilder& GraphBuilder, const TSharedPtr<const FVoxelDensityGPUData>& InDensity,
        const TSharedPtr<const FVoxelEditGPUData>& InEdits)
    {
        VoxelBrickMesherGPU::FInputs Inputs;
        if (InDensity != Density || InEdits != Edits || !Octaves.IsValid())
        {
            Inputs = VoxelBrickMesherGPU::UploadInputs(GraphBuilder, *InDensity, *InEdits);
            Octaves = GraphBuilder.ConvertToExternalBuffer(Inputs.Octaves);
            EditCorners = GraphBuilder.ConvertToExternalBuffer(Inputs.EditCorners);
            EditChildren = GraphBuilder.ConvertToExternalBuffer(Inputs.EditChildren);
            EditFlags = GraphBuilder.ConvertToExternalBuffer(Inputs.EditFlags);
            Density = InDensity;
            Edits = InEdits;
            return Inputs;
        }
        Inputs.Octaves = GraphBuilder.RegisterExternalBuffer(Octaves);
        Inputs.EditCorners = GraphBuilder.RegisterExternalBuffer(EditCorners);
        Inputs.EditChildren = GraphBuilder.RegisterExternalBuffer(EditChildren);
        Inputs.EditFlags = GraphBuilder.RegisterExternalBuffer(EditFlags);
        Inputs.EditNodeCount = (uint32)Edits->Flags.Num();
        Inputs.EditRootExtent = Edits->RootExtent;
        return Inputs;
    }

    /** Once per frame, from the first view family rendered (FVoxelBrickMeshExtension). */
    void Run_RenderThread(FRDGBuilder& GraphBuilder, uint32 FrameNumber);

    /** Nothing in the cache is in use: new buffers of these sizes in the next graph (graphs
     *  already recorded keep the old ones). */
    void ShrinkCache(int32 Vertices, int32 Triangles)
    {
        CacheVertices.SafeRelease();
        CacheIndices.SafeRelease();
        CacheVertexCapacity = 0;
        CacheTriangleCapacity = 0;
        WantedVertexCapacity = Vertices;
        WantedTriangleCapacity = Triangles;
    }

private:
    void Poll();
    void Grow(FRDGBuilder& GraphBuilder);
};

void FVoxelBrickPageGPU::Poll()
{
    for (int32 i = 0; i < Reading.Num(); i++)
    {
        FWork& Work = Reading[i];
        if (!Work.Readback->IsReady()) continue;
        FResult Result;
        Result.Id = Work.Id;
        const int32 Words = Work.Bricks.Num() * (int32)VBP::EntryWords;
        if (const uint32* Data = (const uint32*)Work.Readback->Lock((uint32)(Words * sizeof(uint32))))
        {
            Result.Entries.Append(Data, Words);
            Work.Readback->Unlock();
        }
        {
            FScopeLock Lock(&ResultsCS);
            Results.Add(MoveTemp(Result));
        }
        Reading.RemoveAt(i--);
    }
}

void FVoxelBrickPageGPU::Grow(FRDGBuilder& GraphBuilder)
{
    // A larger buffer holding what the old one held: the game thread's places stay valid.
    auto GrowOne = [&GraphBuilder](TRefCountPtr<FRDGPooledBuffer>& Buffer, int32& Capacity, int32 Wanted, uint32 Words, const TCHAR* Name)
        {
            if (Wanted <= Capacity) return;
            FRDGBufferRef New = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), (uint32)Wanted * Words), Name);
            if (Buffer.IsValid() && Capacity > 0)
                AddCopyBufferPass(GraphBuilder, New, 0, GraphBuilder.RegisterExternalBuffer(Buffer), 0, (uint64)Capacity * Words * sizeof(uint32));
            Buffer = GraphBuilder.ConvertToExternalBuffer(New);
            Capacity = Wanted;
        };
    GrowOne(CacheVertices, CacheVertexCapacity, WantedVertexCapacity, VBP::CacheWords, TEXT("Voxel.MeshCacheVertices"));
    GrowOne(CacheIndices, CacheTriangleCapacity, WantedTriangleCapacity, 3u, TEXT("Voxel.MeshCacheIndices"));
}

void FVoxelBrickPageGPU::Run_RenderThread(FRDGBuilder& GraphBuilder, uint32 FrameNumber)
{
    if (FrameNumber == LastFrame) return;
    LastFrame = FrameNumber;
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesStaging);
    const uint64 StartCycles = FPlatformTime::Cycles64();
    ON_SCOPE_EXIT
    {
        StageMicros.fetch_add((int64)(FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - StartCycles) * 1.0e6), std::memory_order_relaxed);
    };

    Poll();
    Grow(GraphBuilder);

    // Entries meshed in an earlier graph are copied here, after its async work has joined.
    for (FWork& Work : Dispatched)
    {
        Work.Readback = MakeShared<FRHIGPUBufferReadback>(TEXT("Voxel.StageEntriesReadback"));
        AddEnqueueCopyPass(GraphBuilder, Work.Readback.Get(), GraphBuilder.RegisterExternalBuffer(Work.Entries), (uint32)(Work.Bricks.Num() * VBP::EntryWords * sizeof(uint32)));
        Work.Entries.SafeRelease();
        Reading.Add(MoveTemp(Work));
    }
    Dispatched.Reset();

    if (Queued.Num() == 0 || !CacheVertices.IsValid() || !CacheIndices.IsValid()
        || !VoxelBrickMesherGPU::AreShadersReady() || !VBP::ArePageShadersReady())
        return;

    // Async compute: nothing in this frame reads what these passes write.
    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelBrickMesher, "VoxelBrickMesh staging (%d batches)", Queued.Num());
    FRDGBufferRef CacheV = GraphBuilder.RegisterExternalBuffer(CacheVertices);
    FRDGBufferRef CacheI = GraphBuilder.RegisterExternalBuffer(CacheIndices);
    TShaderMapRef<FVoxelPageStageCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
    for (FWork& Work : Queued)
    {
        const int32 NumBricks = Work.Bricks.Num();
        const VoxelBrickMesherGPU::FInputs Inputs = GetInputs(GraphBuilder, Work.Density, Work.Edits);
        const VoxelBrickMesherGPU::FOutputs Outputs = VoxelBrickMesherGPU::AddMesherPasses(GraphBuilder, *Work.Density, Inputs, Work.Settings,
            Work.Bricks, ERDGPassFlags::AsyncCompute, Work.DebugView);

        FRDGBufferDesc EntriesDesc = FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), (uint32)NumBricks * VBP::EntryWords);
        EntriesDesc.Usage |= EBufferUsageFlags::SourceCopy;
        FRDGBufferRef Entries = GraphBuilder.CreateBuffer(EntriesDesc, TEXT("Voxel.StageEntries"));

        FVoxelStageParameters* Parameters = GraphBuilder.AllocParameters<FVoxelStageParameters>();
        Parameters->MesherCounters = GraphBuilder.CreateSRV(Outputs.Counters);
        Parameters->MesherVertices = GraphBuilder.CreateSRV(Outputs.Vertices);
        Parameters->MesherIndices = GraphBuilder.CreateSRV(Outputs.Indices);
        Parameters->StageBricks = GraphBuilder.CreateSRV(CreateStructuredBuffer(GraphBuilder, TEXT("Voxel.StageBricks"), sizeof(FVector4f),
            Work.Places.Num(), Work.Places.GetData(), sizeof(FVector4f) * Work.Places.Num()));
        Parameters->CacheVertices = GraphBuilder.CreateUAV(CacheV);
        Parameters->CacheIndices = GraphBuilder.CreateUAV(CacheI);
        Parameters->StageEntries = GraphBuilder.CreateUAV(Entries);
        Parameters->NumBricks = (uint32)NumBricks;
        Parameters->VertexCap = Outputs.Caps.VertexCap;
        Parameters->TriangleCap = Outputs.Caps.TriangleCap;
        Parameters->VertexStride = VoxelBrickMesherGPU::VertexStride;
        Parameters->RegionVertexStart = (uint32)Work.Vertices.X;
        Parameters->RegionVertices = (uint32)Work.Vertices.Y;
        Parameters->RegionTriangleStart = (uint32)Work.Triangles.X;
        Parameters->RegionTriangles = (uint32)Work.Triangles.Y;
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelBrickMesh stage %d bricks", NumBricks), ERDGPassFlags::AsyncCompute,
            Shader, Parameters, FIntVector(NumBricks, 1, 1));

        Work.Entries = GraphBuilder.ConvertToExternalBuffer(Entries);
        BricksMeshed.fetch_add(NumBricks, std::memory_order_relaxed);
        Dispatched.Add(MoveTemp(Work));
    }
    Queued.Reset();
}

/** Runs a terrain's mesher batches in the scene's graph, early in the frame, where async
 *  compute overlaps the rendering. */
class FVoxelBrickMeshExtension final : public FSceneViewExtensionBase
{
public:
    FVoxelBrickMeshExtension(const FAutoRegister& AutoRegister, const TSharedRef<FVoxelBrickPageGPU, ESPMode::ThreadSafe>& InGPU)
        : FSceneViewExtensionBase(AutoRegister)
        , GPU(InGPU)
    {
    }

    virtual void SetupViewFamily(FSceneViewFamily& InViewFamily) override {}
    virtual void SetupView(FSceneViewFamily& InViewFamily, FSceneView& InView) override {}
    virtual void BeginRenderViewFamily(FSceneViewFamily& InViewFamily) override {}

    virtual void PreRenderViewFamily_RenderThread(FRDGBuilder& GraphBuilder, FSceneViewFamily& InViewFamily) override
    {
        GPU->Run_RenderThread(GraphBuilder, InViewFamily.FrameNumber);
    }

private:
    TSharedRef<FVoxelBrickPageGPU, ESPMode::ThreadSafe> GPU;
};

/** Writes one buffer set's commits when the compute driver runs it (render thread). */
class FVoxelBrickPageProvider final : public IRealtimeMeshComputeProvider
{
public:
    /** A commit's work: VBP::JobWords per job (VoxelBrickPageGPU.usf WriteCS). */
    struct FState
    {
        TArray<uint32> Jobs;
        uint32 TriangleEnd = 0;
        uint32 Color = 0xffffffffu;
        uint64 Version = 0;
    };

    explicit FVoxelBrickPageProvider(const TSharedRef<FVoxelBrickPageGPU, ESPMode::ThreadSafe>& InGPU)
        : GPU(InGPU)
    {
    }

    /** States not yet run accumulate: each is written in order, so none is lost. */
    void AddState_RenderThread(FState&& InState)
    {
        States.Add(MoveTemp(InState));
    }

    virtual void BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context) override;

    /** The newest state version recorded into a frame's graph. */
    std::atomic<uint64> ShownVersion{ 0 };
    /** Runs that found no shaders or a required stream missing, runs recorded, and the
     *  OR of the VBP::EMissing bits seen (optional streams missing don't skip a run). */
    std::atomic<uint32> SkippedRuns{ 0 };
    std::atomic<uint32> Runs{ 0 };
    std::atomic<uint32> Missing{ 0 };
    /** For voxel.Debug.BrickPerf, taken and reset by Flush. */
    std::atomic<uint32> Calls{ 0 };
    std::atomic<uint32> IdleCalls{ 0 };
    std::atomic<uint32> PerfRuns{ 0 };
    std::atomic<uint64> RenderMicros{ 0 };

private:
    TSharedRef<FVoxelBrickPageGPU, ESPMode::ThreadSafe> GPU;
    TArray<FState> States;
};

void FVoxelBrickPageProvider::BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesProvider);
    const uint64 StartCycles = FPlatformTime::Cycles64();
    ON_SCOPE_EXIT
    {
        RenderMicros.fetch_add((uint64)(FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - StartCycles) * 1.0e6), std::memory_order_relaxed);
    };
    Calls.fetch_add(1, std::memory_order_relaxed);
    if (States.Num() == 0)
    {
        IdleCalls.fetch_add(1, std::memory_order_relaxed);
        INC_DWORD_STAT(STAT_VoxelPagesIdleCalls);
        return;
    }

    // Every stream is written as 32-bit words and packed in the shader.
    FRDGBufferUAVRef Positions = Context.GetStreamUAV(FRealtimeMeshStreams::Position, PF_R32_FLOAT);
    FRDGBufferUAVRef Tangents = Context.GetStreamUAV(FRealtimeMeshStreams::Tangents, PF_R32_UINT);
    FRDGBufferUAVRef TexCoords = Context.GetStreamUAV(FRealtimeMeshStreams::TexCoords, PF_R32_UINT);
    FRDGBufferUAVRef Colors = Context.GetStreamUAV(FRealtimeMeshStreams::Color, PF_R32_UINT);
    FRDGBufferUAVRef Triangles = Context.GetStreamUAV(FRealtimeMeshStreams::Triangles, PF_R32_UINT);
    FRDGBufferUAVRef Args = Context.GetIndirectArgsUAV();
    uint32 MissingNow = 0;
    MissingNow |= VBP::ArePageShadersReady() ? 0u : (uint32)VBP::EMissing::PageShaders;
    MissingNow |= Positions ? 0u : (uint32)VBP::EMissing::Positions;
    MissingNow |= Tangents ? 0u : (uint32)VBP::EMissing::Tangents;
    MissingNow |= TexCoords ? 0u : (uint32)VBP::EMissing::TexCoords;
    MissingNow |= Colors ? 0u : (uint32)VBP::EMissing::Colors;
    MissingNow |= Triangles ? 0u : (uint32)VBP::EMissing::Triangles;
    MissingNow |= Args ? 0u : (uint32)VBP::EMissing::Args;
    if (Context.GetVertexCapacity() <= 0 || Context.GetIndexCapacity() <= 0)
        MissingNow |= (uint32)VBP::EMissing::Capacity;
    Missing.fetch_or(MissingNow, std::memory_order_relaxed);
    if (MissingNow & (uint32)VBP::EMissing::Required)
    {
        SkippedRuns.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    Runs.fetch_add(1, std::memory_order_relaxed);
    PerfRuns.fetch_add(1, std::memory_order_relaxed);
    INC_DWORD_STAT(STAT_VoxelPagesRuns);

    // Streams the driver doesn't expose are written to a scratch word and left alone.
    const uint32 StreamMask = (Tangents ? 1u : 0u) | (TexCoords ? 2u : 0u) | (Colors ? 4u : 0u);
    auto Scratch = [&GraphBuilder]()
        {
            return GraphBuilder.CreateUAV(GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 1), TEXT("Voxel.PageScratch")), PF_R32_UINT);
        };
    Tangents = Tangents ? Tangents : Scratch();
    TexCoords = TexCoords ? TexCoords : Scratch();
    Colors = Colors ? Colors : Scratch();

    FRDGBufferRef CacheV = GPU->CacheVertices.IsValid() ? GraphBuilder.RegisterExternalBuffer(GPU->CacheVertices) : VBP::OneWord(GraphBuilder, TEXT("Voxel.NoCache"));
    FRDGBufferRef CacheI = GPU->CacheIndices.IsValid() ? GraphBuilder.RegisterExternalBuffer(GPU->CacheIndices) : VBP::OneWord(GraphBuilder, TEXT("Voxel.NoCache"));
    const uint32 VertexCapacity = (uint32)FMath::Max(0, Context.GetVertexCapacity());
    const uint32 TriangleCapacity = (uint32)FMath::Max(0, Context.GetIndexCapacity() / 3);

    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelBrickPageWrite, "VoxelBrickPage write");
    TShaderMapRef<FVoxelPageWriteCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
    for (const FState& State : States)
    {
        const uint32 NumJobs = (uint32)State.Jobs.Num() / VBP::JobWords;
        FVoxelWriteParameters* Parameters = GraphBuilder.AllocParameters<FVoxelWriteParameters>();
        Parameters->CachedVertices = GraphBuilder.CreateSRV(CacheV);
        Parameters->CachedIndices = GraphBuilder.CreateSRV(CacheI);
        Parameters->Jobs = GraphBuilder.CreateSRV(NumJobs > 0
            ? CreateStructuredBuffer(GraphBuilder, TEXT("Voxel.PageJobs"), sizeof(uint32), State.Jobs.Num(), State.Jobs.GetData(), sizeof(uint32) * State.Jobs.Num())
            : VBP::OneWord(GraphBuilder, TEXT("Voxel.PageJobs")));
        Parameters->OutPositions = Positions;
        Parameters->OutTangents = Tangents;
        Parameters->OutTexCoords = TexCoords;
        Parameters->OutColors = Colors;
        Parameters->OutIndices = Triangles;
        Parameters->OutArgs = Args;
        Parameters->NumJobs = NumJobs;
        Parameters->TriangleEnd = State.TriangleEnd;
        Parameters->VertexCapacity = VertexCapacity;
        Parameters->TriangleCapacity = TriangleCapacity;
        Parameters->Color = State.Color;
        Parameters->StreamMask = StreamMask;
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelBrickPage write %u jobs", NumJobs), Shader, Parameters,
            FIntVector((int32)FMath::Max(1u, NumJobs), 1, 1));
    }
    ShownVersion.store(States.Last().Version, std::memory_order_release);
    States.Reset();
}

FVoxelBrickPages::FVoxelBrickPages()
    : GPU(MakeShared<FVoxelBrickPageGPU, ESPMode::ThreadSafe>())
{
    CacheVertices.Reset(VBP::InitialCacheVertices);
    CacheTriangles.Reset(VBP::InitialCacheTriangles);
}

FVoxelBrickPages::~FVoxelBrickPages()
{
    Reset();
    MeshExtension.Reset();
}

void FVoxelBrickPages::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelBrickTerrainSettings& InSettings)
{
    Reset();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Settings = InSettings;
    // The level-MaxLevel brick spans the root: 2^MaxDepth finest cells.
    MesherSettings.FinestCell = Settings.FinestCell;
    MesherSettings.CellsPerBrick = Settings.CellsPerBrick;
    MesherSettings.MaxDepth = Settings.MaxLevel + FMath::FloorLog2(FMath::Max(1, Settings.CellsPerBrick));
    const VoxelBrickMesherGPU::FCaps Caps = VoxelBrickMesherGPU::CapsFor(MesherSettings);
    MaxSlab = FIntPoint(FMath::Max(VBP::SlabVertices, 2 * (int32)FMath::RoundUpToPowerOfTwo(Caps.VertexCap)),
        FMath::Max(VBP::SlabTriangles, 2 * (int32)FMath::RoundUpToPowerOfTwo(Caps.TriangleCap)));
    Density.Reset();
    Edits.Reset();
    if (!MeshExtension.IsValid() && GEngine && FApp::CanEverRender())
        MeshExtension = FSceneViewExtensions::NewExtension<FVoxelBrickMeshExtension>(GPU);
}

void FVoxelBrickPages::Reset()
{
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        for (FSlab& Slab : Pair.Value.Slabs)
            DestroyInstance(Slab.Instance);
        for (FInstance& Spare : Pair.Value.Spares)
            DestroyInstance(Spare);
    }
    Pages.Reset();
    for (TPair<uint64, TArray<FInstance>>& Pair : Pool)
    {
        for (FInstance& Instance : Pair.Value)
            DestroyInstance(Instance);
    }
    Pool.Reset();
    NumPooled = 0;
    for (FReleasing& Entry : Releasing)
        DestroyInstance(Entry.Instance);
    Releasing.Reset();
    LevelAverages.Reset();
    WaitingSince = 0;

    // Whatever is being meshed is dropped; results of batches already sent are ignored.
    Staged.Reset();
    MeshQueue.Reset();
    QueueHead = 0;
    NumStagedReady = 0;
    Batches.Reset();
    CacheReleases.Reset();
    CacheVertices.Reset(CacheVertices.GetCapacity());
    CacheTriangles.Reset(CacheTriangles.GetCapacity());
    CachePeak = FIntPoint::ZeroValue;
    CachePeakBefore = FIntPoint::ZeroValue;
    CachePeakFrame = GFrameCounter;
    NoShrinkUntil = 0;
    NewPages.Reset();
    bFirstFill = false;
    bBusy = false;
    ENQUEUE_RENDER_COMMAND(VoxelBrickPagesReset)([GPURef = GPU](FRHICommandListImmediate&)
        {
            GPURef->Queued.Reset();
            FScopeLock Lock(&GPURef->ResultsCS);
            GPURef->Results.Reset();
        });
}

void FVoxelBrickPages::SetMaterial(UMaterialInterface* InMaterial)
{
    Settings.Material = InMaterial;
    auto Apply = [InMaterial](FInstance& Instance)
        {
            if (URealtimeMeshComponent* Component = Instance.Component.Get())
                Component->SetMaterial(0, InMaterial);
        };
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        for (FSlab& Slab : Pair.Value.Slabs)
            Apply(Slab.Instance);
        for (FInstance& Spare : Pair.Value.Spares)
            Apply(Spare);
    }
    for (TPair<uint64, TArray<FInstance>>& Pair : Pool)
    {
        for (FInstance& Instance : Pair.Value)
            Apply(Instance);
    }
    for (FReleasing& Entry : Releasing)
        Apply(Entry.Instance);
}

FVoxelBrickKey FVoxelBrickPages::PageOf(const FVoxelBrickKey& Key)
{
    FVoxelBrickKey Page = Key;
    Page.Coord = FInt64Vector(FVoxelBrickKey::FloorShift(Key.Coord.X, VBP::PageShift), FVoxelBrickKey::FloorShift(Key.Coord.Y, VBP::PageShift),
        FVoxelBrickKey::FloorShift(Key.Coord.Z, VBP::PageShift));
    return Page;
}

FVoxelBrickPages::FInstance FVoxelBrickPages::CreateInstance(int32 VertexCapacity, int32 TriangleCapacity)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesCreate);
    INC_DWORD_STAT(STAT_VoxelPagesCreated);
    const double StartTime = FPlatformTime::Seconds();
    ON_SCOPE_EXIT
    {
        const double Ms = (FPlatformTime::Seconds() - StartTime) * 1000.0;
        Perf.Created++;
        Perf.CreateMs += Ms;
        Perf.MaxCreateMs = FMath::Max(Perf.MaxCreateMs, Ms);
    };
    FInstance Instance;
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!OwnerActor || !Root) return Instance;

    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetupMaterialSlot(0, "Surface Material");
    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->bHiddenInSceneCapture = true;
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->RegisterComponent();
    Component->SetMaterial(0, Settings.Material.Get());
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetRealtimeMesh(Mesh);
    Component->SetVisibleInRayTracing(false);
    // For the morph (VoxelMorph.ush): where level 0's band ends in world units. Data 0, the
    // level, is set with the page (PlaceInstance).
    Component->SetCustomPrimitiveDataFloat(1, (float)(VBP::MorphBandEnd0(Settings) * Root->GetComponentScale().X));

    // The uploaded streams only size the buffers; the provider writes them. Two corner
    // positions give the bounds, the same for every level (positions are in level units):
    // bricks reach a few cells past their box, and the morph moves vertices toward the
    // vertices of coarser cells, within VBP::BoundsMarginCells of the brick.
    FRealtimeMeshStreamSet Streams;
    FRealtimeMeshStream& Positions = Streams.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
    Positions.SetNumZeroed(VertexCapacity);
    const double Margin = VBP::BoundsMarginCells * Settings.FinestCell;
    const double Extent = (double)(1LL << VBP::PageShift) * Settings.CellsPerBrick * Settings.FinestCell;
    TArrayView<FVector3f> PositionView = Positions.GetArrayView<FVector3f>();
    PositionView[0] = FVector3f((float)-Margin);
    PositionView[1] = FVector3f((float)(Extent + Margin));
    // Nothing draws these before the provider writes them: no need to clear them here.
    Streams.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>()).SetNumUninitialized(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<TRealtimeMeshTexCoords<FVector2f, VBP::TexCoordChannels>>()).SetNumUninitialized(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>()).SetNumUninitialized(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>()).SetNumZeroed(TriangleCapacity);
    // No PolyGroups stream: the buffer set then makes one section over every vertex and index.
    // (Sections built from polygroups take the vertex range the indices use, and drop a
    // section whose indices are all zero, which leaves the section key without a section.)

    Instance.bBuffersReady = MakeShared<std::atomic<bool>, ESPMode::ThreadSafe>(false);
    const TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> Ready = Instance.bBuffersReady;
    Mesh->CreateBufferSet(VBP::GroupKey(), MoveTemp(Streams), FRealtimeMeshBufferSetConfig(ERealtimeMeshSectionDrawType::Static, true))
        .Next([Ready](ERealtimeMeshProxyUpdateStatus) { Ready->store(true, std::memory_order_release); });
    Mesh->UpdateSectionConfig(VBP::SectionKey(), FRealtimeMeshSectionConfig(0), false);
    // The section stays hidden until the provider is registered: until then it draws its whole
    // (zero) range. Once registered it draws indirect from args the driver creates zeroed, so
    // nothing until the first run. The component must stay visible: a hidden component has no
    // render proxy, and the driver runs providers only for meshes with one.
    Mesh->SetSectionVisibility(VBP::SectionKey(), false);

    Instance.Mesh = Mesh;
    Instance.Component = Component;
    Instance.Provider = MakeShared<FVoxelBrickPageProvider, ESPMode::ThreadSafe>(GPU);
    Instance.VertexCapacity = VertexCapacity;
    Instance.TriangleCapacity = TriangleCapacity;
    Instance.CreatedFrame = GFrameCounter;
    return Instance;
}

void FVoxelBrickPages::DestroyInstance(FInstance& Instance)
{
    if (Instance.Handle != 0)
        UnregisterRealtimeMeshComputeProvider(Instance.Handle);
    if (URealtimeMeshComponent* Component = Instance.Component.Get())
        Component->DestroyComponent();
    Instance = FInstance();
}

void FVoxelBrickPages::PlaceInstance(const FVoxelBrickKey& PageKey, FInstance& Instance) const
{
    URealtimeMeshComponent* Component = Instance.Component.Get();
    if (!Component) return;
    const double Scale = (double)(1LL << PageKey.Level);
    Component->SetRelativeTransform(FTransform(FQuat::Identity, VBP::PageOrigin(Settings, PageKey), FVector(Scale)));
    Component->SetCustomPrimitiveDataFloat(0, (float)PageKey.Level);
}

bool FVoxelBrickPages::IsReady(const FInstance& Instance)
{
    return Instance.IsValid() && Instance.Handle != 0 && GFrameCounter > Instance.RegisteredFrame && GFrameCounter >= Instance.ReadyFrame;
}

TArray<FVoxelBrickPages::FInstance>* FVoxelBrickPages::FindPooled(FIntPoint Capacity)
{
    // Smallest total first.
    const int32 VC = VBP::SizeClass(Capacity.X);
    const int32 TC = VBP::SizeClass(Capacity.Y);
    for (int32 Extra = 0; Extra <= 2 * VBP::PoolSlack; Extra++)
    {
        for (int32 DV = FMath::Max(0, Extra - VBP::PoolSlack); DV <= FMath::Min(Extra, VBP::PoolSlack); DV++)
        {
            TArray<FInstance>* Free = Pool.Find(VBP::PoolKey(VC + DV, TC + Extra - DV));
            if (Free && Free->Num() > 0) return Free;
        }
    }
    return nullptr;
}

FVoxelBrickPages::FInstance FVoxelBrickPages::AcquireInstance(const FVoxelBrickKey& PageKey, FIntPoint Capacity)
{
    while (TArray<FInstance>* Free = FindPooled(Capacity))
    {
        FInstance Instance = Free->Pop();
        NumPooled--;
        if (!Instance.Component.IsValid() || !Instance.Mesh.IsValid())
        {
            DestroyInstance(Instance);
            continue;
        }
        // Moved now, filled from the next frame on, so the move reaches the render thread no
        // later than the new contents.
        PlaceInstance(PageKey, Instance);
        Instance.ReadyFrame = GFrameCounter + 1;
        Perf.Recycled++;
        INC_DWORD_STAT(STAT_VoxelPagesRecycled);
        return Instance;
    }
    // A new one is registered with empty contents, so it is ready when a commit needs it.
    FInstance Instance = CreateInstance(Capacity.X, Capacity.Y);
    if (Instance.IsValid())
    {
        PlaceInstance(PageKey, Instance);
        Push(PageKey, Instance, TArray<uint32>(), 0);
    }
    return Instance;
}

void FVoxelBrickPages::PoolInstance(FInstance& Instance)
{
    if (!Instance.IsValid()) return;
    if (!Instance.Component.IsValid() || NumPooled >= VBP::MaxPooled)
    {
        DestroyInstance(Instance);
        return;
    }
    Pool.FindOrAdd(VBP::PoolKey(VBP::SizeClass(Instance.VertexCapacity), VBP::SizeClass(Instance.TriangleCapacity))).Add(MoveTemp(Instance));
    NumPooled++;
    Instance = FInstance();
}

FIntPoint FVoxelBrickPages::RoundCapacity(double Vertices, double Triangles) const
{
    // MaxSlab is a power of two, so a class size too.
    const int32 V = (int32)FMath::Clamp(FMath::CeilToDouble(Vertices * VBP::CapacityMargin), (double)VBP::MinVertices, (double)MaxSlab.X);
    const int32 T = (int32)FMath::Clamp(FMath::CeilToDouble(Triangles * VBP::CapacityMargin), (double)VBP::MinTriangles, (double)MaxSlab.Y);
    return FIntPoint(VBP::ClassSize(VBP::SizeClass(V)), VBP::ClassSize(VBP::SizeClass(T)));
}

int32 FVoxelBrickPages::CreateBudget() const
{
    // The longer a commit waits for buffer sets, the more may be created per frame.
    const int32 Base = FMath::Max(1, CVarVoxelBrickPageCreates.GetValueOnGameThread());
    const int32 Waited = WaitingSince != 0 ? (int32)FMath::Min<uint64>(GFrameCounter - WaitingSince, 64) : 0;
    return Base * FMath::Min(1 + Waited / VBP::CreateRampFrames, VBP::MaxCreateRamp);
}

void FVoxelBrickPages::BeginCreates()
{
    if (CreatesFrame == GFrameCounter) return;
    CreatesFrame = GFrameCounter;
    CreatesThisFrame = 0;
}

bool FVoxelBrickPages::AddSpare(const FVoxelBrickKey& PageKey, FPage& Page, FIntPoint Capacity)
{
    Page.SpareUsedFrame = GFrameCounter;
    if (!FindPooled(Capacity) && CreatesThisFrame >= CreateBudget()) return false;
    const int32 CreatedBefore = Perf.Created;
    FInstance Instance = AcquireInstance(PageKey, Capacity);
    CreatesThisFrame += Perf.Created - CreatedBefore;
    if (!Instance.IsValid()) return false;
    Page.Spares.Add(MoveTemp(Instance));
    Perf.SparesAdded++;
    return true;
}

void FVoxelBrickPages::PrepareNewPages()
{
    BeginCreates();
    for (const TPair<FVoxelBrickKey, FIntPoint>& Entry : NewPages)
    {
        FPage& Page = Pages.FindOrAdd(Entry.Key);
        if (Page.Spares.Num() > 0)
            Page.SpareUsedFrame = GFrameCounter;
        else if (Page.Slabs.Num() == 0)
            AddSpares(Entry.Key, Page, Entry.Value, FIntPoint::ZeroValue);
    }
}

void FVoxelBrickPages::AddSpares(const FVoxelBrickKey& PageKey, FPage& Page, FIntPoint Counts, FIntPoint MinCounts)
{
    // Sets of at most MaxSlab each: a large need takes several.
    FIntPoint Left = Counts;
    while (Left.X > 0 || Left.Y > 0)
    {
        const FIntPoint Capacity = RoundCapacity(FMath::Max(Left.X, MinCounts.X), FMath::Max(Left.Y, MinCounts.Y));
        if (!AddSpare(PageKey, Page, Capacity)) return;
        Left -= Capacity;
    }
}

FIntPoint FVoxelBrickPages::ExpectedCounts(const FVoxelBrickKey& Key) const
{
    // What it held before, else the level's average surface brick (the largest level average
    // before its first read back), else a planar brick; the latter with a margin.
    if (const FPage* Page = Pages.Find(PageOf(Key)))
    {
        const FMember* Member = Page->Members.Find(Key);
        if (Member && Member->Triangles > 0)
            return FIntPoint(Member->Vertices, Member->Triangles);
    }
    const FVector2D* LevelAverage = LevelAverages.Find(Key.Level);
    FVector2D Average = LevelAverage ? *LevelAverage : FVector2D::ZeroVector;
    if (Average.X <= 0.0)
    {
        for (const TPair<int32, FVector2D>& Pair : LevelAverages)
            Average = Average.ComponentMax(Pair.Value);
    }
    if (Average.X <= 0.0)
    {
        const double Square = (double)Settings.CellsPerBrick * Settings.CellsPerBrick;
        Average = FVector2D(VBP::PlanarVertices * Square, VBP::PlanarTriangles * Square);
    }
    Average *= VBP::UnknownBrickWeight;
    return FIntPoint(FMath::CeilToInt(Average.X), FMath::CeilToInt(Average.Y));
}

void FVoxelBrickPages::Push(const FVoxelBrickKey& PageKey, FInstance& Instance, TArray<uint32>&& Jobs, int32 TriangleEnd)
{
    if (!Instance.IsValid()) return;
    FVoxelBrickPageProvider::FState State;
    State.Jobs = MoveTemp(Jobs);
    State.TriangleEnd = (uint32)FMath::Max(0, TriangleEnd);
    State.Color = (CVarVoxelDebugLevelColor.GetValueOnGameThread() > 0 ? FVoxelBrickTerrain::LevelColor(PageKey.Level) : FColor::White).DWColor();
    State.Version = NextVersion++;
    Instance.PushedVersion = State.Version;
    // The state reaches the provider before the dirty mark: both are render commands.
    ENQUEUE_RENDER_COMMAND(VoxelBrickPagePush)([Provider = Instance.Provider, State = MoveTemp(State)](FRHICommandListImmediate&) mutable
        {
            Provider->AddState_RenderThread(MoveTemp(State));
        });
    if (Instance.Handle != 0)
    {
        MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
        Instance.LastKickFrame = GFrameCounter;
    }
}

void FVoxelBrickPages::Advance(FInstance& Instance) const
{
    if (!Instance.IsValid()) return;
    URealtimeMeshSimple* Mesh = Instance.Mesh.Get();
    if (!Mesh) return;

    if (Instance.Handle == 0)
    {
        // Registered once the buffers exist and the component has a proxy (two frames after
        // its creation); the driver runs an OnDemand provider when registered.
        if (!Instance.bBuffersReady->load(std::memory_order_acquire) || GFrameCounter < Instance.CreatedFrame + 2 || Instance.PushedVersion == 0)
            return;
        // Shown first: the driver binds the indirect args to the section proxy, and a later
        // section update from the data side would rebuild that proxy without them.
        Mesh->SetSectionVisibility(VBP::SectionKey(), true);
        FRealtimeMeshComputeRegistration Registration;
        Registration.Mesh = Mesh->GetMesh();
        Registration.BufferSetKey = VBP::GroupKey();
        Registration.SectionKey = VBP::SectionKey();
        Registration.Provider = Instance.Provider;
        // On demand: a commit marks a provider when its buffer set changes, so idle ones cost
        // the driver nothing.
        Registration.Trigger = ERealtimeMeshComputeTrigger::OnDemand;
        Registration.bUseIndirect = true;
        Instance.Handle = RegisterRealtimeMeshComputeProvider(Registration);
        Instance.LastKickFrame = GFrameCounter;
        Instance.RegisteredFrame = GFrameCounter;
        return;
    }

    // A run that didn't happen (no proxy yet, shaders still compiling) is asked for again.
    if (Instance.Provider->ShownVersion.load(std::memory_order_acquire) < Instance.PushedVersion && GFrameCounter > Instance.LastKickFrame + VBP::KickFrames)
    {
        MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
        Instance.LastKickFrame = GFrameCounter;
    }
}

void FVoxelBrickPages::DiscardStaged(const FStaged& Brick)
{
    // A brick still being meshed is freed with its batch's region.
    if (Brick.State != EStage::Ready) return;
    CacheVertices.Release(Brick.CacheVertex, Brick.Vertices);
    CacheTriangles.Release(Brick.CacheTriangle, Brick.Triangles);
}

void FVoxelBrickPages::Stage(TConstArrayView<FVoxelBrickPageChange> Changes, bool bInFirstFill)
{
    for (const TPair<FVoxelBrickKey, FStaged>& Pair : Staged)
        DiscardStaged(Pair.Value);
    Staged.Reset();
    MeshQueue.Reset();
    QueueHead = 0;
    NumStagedReady = 0;
    bFirstFill = bInFirstFill;
    for (const FVoxelBrickPageChange& Change : Changes)
    {
        if (Change.bRemove) continue;
        FStaged& Brick = Staged.Add(Change.Key);
        Brick.Stamp = ++NextStamp;
        Brick.Expected = ExpectedCounts(Change.Key);
        Brick.bMeshable = VoxelBrickMesherGPU::MakeBrick(Change.Key, Change.Levels, Change.Meshed, MesherSettings, Brick.Brick);
        if (!Brick.bMeshable)
        {
            // Out of the GPU lattice's range: nothing to draw.
            Brick.State = EStage::Ready;
            NumStagedReady++;
            continue;
        }
        MeshQueue.Add(FQueued{ Change.Key, Brick.Stamp });
    }
    // Coarse levels first (they cover the most), a page's bricks together.
    MeshQueue.Sort([](const FQueued& A, const FQueued& B)
        {
            if (A.Key.Level != B.Key.Level) return A.Key.Level > B.Key.Level;
            const FInt64Vector PA = PageOf(A.Key).Coord;
            const FInt64Vector PB = PageOf(B.Key).Coord;
            if (PA.X != PB.X) return PA.X < PB.X;
            if (PA.Y != PB.Y) return PA.Y < PB.Y;
            return PA.Z < PB.Z;
        });

    // Pages without buffer sets, sized from the expected counts (without their margin).
    TMap<FVoxelBrickKey, FVector2D> PageCounts;
    for (const FQueued& Item : MeshQueue)
    {
        const FVoxelBrickKey PageKey = PageOf(Item.Key);
        const FPage* Page = Pages.Find(PageKey);
        if (Page && Page->Slabs.Num() > 0) continue;
        const FIntPoint Counts = Staged.FindChecked(Item.Key).Expected;
        PageCounts.FindOrAdd(PageKey, FVector2D::ZeroVector) += FVector2D(Counts.X, Counts.Y) / VBP::UnknownBrickWeight;
    }
    NewPages.Reset(PageCounts.Num());
    for (const TPair<FVoxelBrickKey, FVector2D>& Pair : PageCounts)
        NewPages.Emplace(Pair.Key, FIntPoint(FMath::CeilToInt(Pair.Value.X), FMath::CeilToInt(Pair.Value.Y)));
}

void FVoxelBrickPages::TakeBatchResults()
{
    TArray<FVoxelBrickPageGPU::FResult> Results;
    {
        FScopeLock Lock(&GPU->ResultsCS);
        Results = MoveTemp(GPU->Results);
        GPU->Results.Reset();
    }
    for (const FVoxelBrickPageGPU::FResult& Result : Results)
    {
        FBatch Batch;
        if (!Batches.RemoveAndCopyValue(Result.Id, Batch)) continue;
        const int32 NumBricks = Batch.Keys.Num();
        const bool bRead = Result.Entries.Num() >= NumBricks * (int32)VBP::EntryWords;
        if (!bRead && !bWarnedReadback)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelBricks] A brick mesher readback returned nothing (%d of %d words); its %d bricks are meshed again. (Logged once.)"),
                Result.Entries.Num(), NumBricks * (int32)VBP::EntryWords, NumBricks);
            bWarnedReadback = true;
        }
        TArray<FIntPoint> KeptVertices;
        TArray<FIntPoint> KeptTriangles;
        for (int32 b = 0; b < NumBricks; b++)
        {
            FStaged* Brick = Staged.Find(Batch.Keys[b]);
            if (!Brick || Brick->Stamp != Batch.Stamps[b] || Brick->State != EStage::Meshing) continue;
            const uint32* E = bRead ? &Result.Entries[b * (int32)VBP::EntryWords] : nullptr;
            const bool bInside = E && E[5] != 0u
                && (int32)E[0] >= Batch.Vertices.X && (int64)E[0] + E[1] <= (int64)Batch.Vertices.X + Batch.Vertices.Y
                && (int32)E[2] >= Batch.Triangles.X && (int64)E[2] + E[3] <= (int64)Batch.Triangles.X + Batch.Triangles.Y;
            if (!bInside)
            {
                // Its batch's region had no room left (or nothing was read): meshed again, with
                // room for what it has.
                Brick->State = EStage::Queued;
                if (E)
                    Brick->Expected = FIntPoint((int32)E[6], (int32)E[7]);
                MeshQueue.Insert(FQueued{ Batch.Keys[b], Brick->Stamp }, QueueHead);
                Perf.BricksBack++;
                continue;
            }
            Brick->State = EStage::Ready;
            NumStagedReady++;
            INC_DWORD_STAT(STAT_VoxelPagesBricksMeshed);
            Brick->Errors = E[4];
            Brick->Triangles = (int32)E[3];
            Brick->Vertices = Brick->Triangles > 0 ? (int32)E[1] : 0;
            Brick->CacheVertex = (int32)E[0];
            Brick->CacheTriangle = (int32)E[2];
            if (Brick->Triangles <= 0) continue;
            KeptVertices.Add(FIntPoint(Brick->CacheVertex, Brick->Vertices));
            KeptTriangles.Add(FIntPoint(Brick->CacheTriangle, Brick->Triangles));
            FVector2D& Average = LevelAverages.FindOrAdd(Batch.Keys[b].Level, FVector2D::ZeroVector);
            const FVector2D Counts((double)Brick->Vertices, (double)Brick->Triangles);
            Average = Average.X <= 0.0 ? Counts : Average + (Counts - Average) * VBP::AverageBlend;
        }
        // The rest of the regions is free again.
        TArray<FIntPoint> FreeVertices = { Batch.Vertices };
        TArray<FIntPoint> FreeTriangles = { Batch.Triangles };
        FVoxelRangeAllocator::Subtract(FreeVertices, MoveTemp(KeptVertices), MAX_int32);
        FVoxelRangeAllocator::Subtract(FreeTriangles, MoveTemp(KeptTriangles), MAX_int32);
        for (const FIntPoint& Range : FreeVertices)
            CacheVertices.Release(Range.X, Range.Y);
        for (const FIntPoint& Range : FreeTriangles)
            CacheTriangles.Release(Range.X, Range.Y);
    }
}

void FVoxelBrickPages::DispatchBatches()
{
    if (!Density.IsValid() || !Edits.IsValid()) return;
    const int32 Base = FMath::Max(0, CVarVoxelBrickMeshBatches.GetValueOnGameThread());
    const int32 Budget = bFirstFill ? Base * VBP::FirstFillBatchScale
        : (bBusy ? FMath::Max(Base, CVarVoxelBrickMeshBatchesBusy.GetValueOnGameThread()) : Base);
    const int32 BatchBricks = VBP::BatchBricksFor(MesherSettings);
    const VoxelBrickMesherGPU::FCaps Caps = VoxelBrickMesherGPU::CapsFor(MesherSettings);
    // The land debug view the bricks sent now are coloured with (their copies say so).
    FVoxelBrickMesherGPUDebugView DebugView;
    DebugView.Mode = FVoxelBrickTerrain::GetVertexLandView(Settings.bMaterialLandViews);
    DebugView.SeaRadius = Settings.DebugSeaRadius;
    for (int32 Sent = 0; Sent < Budget && QueueHead < MeshQueue.Num() && Batches.Num() < VBP::MaxBatchesInFlight; Sent++)
    {
        FBatch Batch;
        FVoxelBrickPageGPU::FWork Work;
        FIntPoint Need(0, 0);
        while (QueueHead < MeshQueue.Num() && Batch.Keys.Num() < BatchBricks)
        {
            const FQueued Item = MeshQueue[QueueHead++];
            FStaged* Brick = Staged.Find(Item.Key);
            if (!Brick || Brick->Stamp != Item.Stamp || Brick->State != EStage::Queued) continue;
            Brick->State = EStage::Meshing;
            Brick->bDebugColor = DebugView.Mode != 0;
            Batch.Keys.Add(Item.Key);
            Batch.Stamps.Add(Item.Stamp);
            Need += FIntPoint(FMath::Min(Brick->Expected.X, (int32)Caps.VertexCap), FMath::Min(Brick->Expected.Y, (int32)Caps.TriangleCap));

            // Page-local placement (cm) and level scale, then the page's triplanar UV origin.
            const FVoxelBrickKey PageKey = PageOf(Item.Key);
            const FInt64Vector Local = Item.Key.Coord - PageKey.Coord * (1LL << VBP::PageShift);
            const double BrickSize = Settings.GridFor(Item.Key.Level).BrickSize();
            const FVector UVOrigin = VBP::PageOrigin(Settings, PageKey) * Settings.TriplanarUVScale;
            Work.Bricks.Add(Brick->Brick);
            Work.Places.Add(FVector4f((float)(Local.X * BrickSize), (float)(Local.Y * BrickSize), (float)(Local.Z * BrickSize),
                (float)(1.0 / (double)(1LL << Item.Key.Level))));
            Work.Places.Add(FVector4f((float)FMath::Frac(UVOrigin.X), (float)FMath::Frac(UVOrigin.Y), (float)FMath::Frac(UVOrigin.Z),
                (float)Settings.TriplanarUVScale));
        }
        const int32 NumBricks = Batch.Keys.Num();
        if (NumBricks == 0) break;

        // A region with room for the expected counts and a margin, from the cache (grown when
        // full; the render thread keeps what it holds).
        auto Reserve = [](FVoxelRangeAllocator& Cache, int32 Wanted, int32 Initial, int32 Max)
            {
                int32 Start = Cache.Allocate(Wanted);
                if (Start < 0)
                {
                    Cache.Grow(FMath::Min(Max, FMath::Max3(Initial, Cache.GetCapacity() * 2, Cache.GetCapacity() + Wanted)));
                    Start = Cache.Allocate(Wanted);
                }
                return Start;
            };
        const int32 RegionV = FMath::Clamp(Align((int32)(Need.X * VBP::RegionMargin), VBP::RegionAlign), VBP::RegionAlign, NumBricks * (int32)Caps.VertexCap);
        const int32 RegionT = FMath::Clamp(Align((int32)(Need.Y * VBP::RegionMargin), VBP::RegionAlign), VBP::RegionAlign, NumBricks * (int32)Caps.TriangleCap);
        const int32 StartV = Reserve(CacheVertices, RegionV, VBP::InitialCacheVertices, VBP::MaxCacheVertices);
        const int32 StartT = StartV >= 0 ? Reserve(CacheTriangles, RegionT, VBP::InitialCacheTriangles, VBP::MaxCacheTriangles) : -1;
        if (StartV < 0 || StartT < 0)
        {
            // The cache is at its limit: wait for commits to free some.
            if (StartV >= 0)
                CacheVertices.Release(StartV, RegionV);
            for (int32 b = NumBricks - 1; b >= 0; b--)
            {
                Staged.FindChecked(Batch.Keys[b]).State = EStage::Queued;
                MeshQueue.Insert(FQueued{ Batch.Keys[b], Batch.Stamps[b] }, QueueHead);
            }
            break;
        }
        Batch.Vertices = FIntPoint(StartV, RegionV);
        Batch.Triangles = FIntPoint(StartT, RegionT);
        Work.Id = ++NextBatchId;
        Work.Vertices = Batch.Vertices;
        Work.Triangles = Batch.Triangles;
        Work.Density = Density;
        Work.Edits = Edits;
        Work.Settings = MesherSettings;
        Work.DebugView = DebugView;
        Batches.Add(Work.Id, MoveTemp(Batch));
        ENQUEUE_RENDER_COMMAND(VoxelBrickStage)([GPURef = GPU, Work = MoveTemp(Work), WantV = CacheVertices.GetCapacity(), WantT = CacheTriangles.GetCapacity()](FRHICommandListImmediate&) mutable
            {
                GPURef->WantedVertexCapacity = FMath::Max(GPURef->WantedVertexCapacity, WantV);
                GPURef->WantedTriangleCapacity = FMath::Max(GPURef->WantedTriangleCapacity, WantT);
                GPURef->Queued.Add(MoveTemp(Work));
            });
        Perf.Batches++;
        Perf.BricksSent += NumBricks;
    }
    if (QueueHead >= MeshQueue.Num())
    {
        MeshQueue.Reset();
        QueueHead = 0;
    }
}

bool FVoxelBrickPages::PreparePages(TConstArrayView<FVoxelBrickPageChange> Changes)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesPrepare);
    BeginCreates();
    TMap<FVoxelBrickKey, TArray<const FVoxelBrickPageChange*>> ByPage;
    for (const FVoxelBrickPageChange& Change : Changes)
        ByPage.FindOrAdd(PageOf(Change.Key)).Add(&Change);

    bool bMeshed = true;
    bool bReady = true;
    for (const TPair<FVoxelBrickKey, TArray<const FVoxelBrickPageChange*>>& Pair : ByPage)
    {
        bool bAdds = false;
        bool bPageMeshed = true;
        FIntPoint Estimate(0, 0);
        for (const FVoxelBrickPageChange* Change : Pair.Value)
        {
            if (Change->bRemove) continue;
            bAdds = true;
            const FStaged* Brick = Staged.Find(Change->Key);
            if (!Brick)
            {
                // Never staged (the terrain stages every commit): meshed now.
                FStaged& New = Staged.Add(Change->Key);
                New.Stamp = ++NextStamp;
                New.Expected = ExpectedCounts(Change->Key);
                New.bMeshable = VoxelBrickMesherGPU::MakeBrick(Change->Key, Change->Levels, Change->Meshed, MesherSettings, New.Brick);
                New.State = New.bMeshable ? EStage::Queued : EStage::Ready;
                NumStagedReady += New.bMeshable ? 0 : 1;
                if (New.bMeshable)
                    MeshQueue.Add(FQueued{ Change->Key, New.Stamp });
                Brick = &New;
            }
            // Exact counts, or the expected ones without their margin.
            bPageMeshed &= Brick->State == EStage::Ready;
            Estimate += Brick->State == EStage::Ready ? FIntPoint(Brick->Vertices, Brick->Triangles)
                : FIntPoint(FMath::CeilToInt(Brick->Expected.X / VBP::UnknownBrickWeight), FMath::CeilToInt(Brick->Expected.Y / VBP::UnknownBrickWeight));
        }
        if (!bAdds) continue;   // removals only: always ready
        FPage& Page = Pages.FindOrAdd(Pair.Key);
        if (!bPageMeshed)
        {
            // A new page gets its first buffer set while its bricks are meshed.
            bMeshed = false;
            if (Page.Spares.Num() > 0)
                Page.SpareUsedFrame = GFrameCounter;
            else if (Page.Slabs.Num() == 0)
                AddSpares(Pair.Key, Page, Estimate, FIntPoint::ZeroValue);
            continue;
        }
        bool bUsesSpare = false;
        const FIntPoint Overflow = PlanPage(Pair.Key, Page, Pair.Value, false, nullptr, &bUsesSpare);
        if (bUsesSpare)
        {
            Page.SpareUsedFrame = GFrameCounter;
            for (const FInstance& Spare : Page.Spares)
                bReady &= IsReady(Spare);
        }
        if (Overflow.X <= 0 && Overflow.Y <= 0) continue;
        // More spares, each at least as large as what the slabs hold (a growing page needs a
        // few sets, not one per commit).
        FIntPoint Held(0, 0);
        for (const FSlab& Slab : Page.Slabs)
            Held += FIntPoint(Slab.Vertices.GetNumUsed(), Slab.Triangles.GetNumUsed());
        AddSpares(Pair.Key, Page, Overflow, Held);
        bReady = false;
    }
    bPrepareMeshed = bMeshed;
    // Only a wait for buffer sets raises the creation budget.
    WaitingSince = !bMeshed || bReady ? 0 : (WaitingSince != 0 ? WaitingSince : GFrameCounter);
    return bMeshed && bReady;
}

bool FVoxelBrickPages::PrepareCommit(TConstArrayView<FVoxelBrickPageChange> Changes, bool bWhole)
{
    // Changes are everything staged: until the mesher is done, only new pages need work.
    if (bWhole && !IsMeshed())
    {
        SCOPE_CYCLE_COUNTER(STAT_VoxelPagesPrepare);
        PrepareNewPages();
        bPrepareMeshed = false;
        WaitingSince = 0;
        Perf.MeshWaits++;
        return false;
    }
    const bool bReady = PreparePages(Changes);
    if (!bReady)
    {
        Perf.MeshWaits += bPrepareMeshed ? 0 : 1;
        Perf.SlabWaits += bPrepareMeshed ? 1 : 0;
    }
    return bReady;
}

FIntPoint FVoxelBrickPages::PlanPage(const FVoxelBrickKey& PageKey, FPage& Page, TConstArrayView<const FVoxelBrickPageChange*> PageChanges, bool bApply,
    FCacheRelease* Release, bool* bOutUsesSpare)
{
    // The slabs' space as the plan changes it, then the spares (empty).
    const int32 NumSlabs = Page.Slabs.Num();
    TArray<FPlanSlab> Plan;
    Plan.SetNum(NumSlabs + Page.Spares.Num());
    for (int32 s = 0; s < NumSlabs; s++)
    {
        Plan[s].Vertices = Page.Slabs[s].Vertices;
        Plan[s].Triangles = Page.Slabs[s].Triangles;
    }
    for (int32 i = 0; i < Page.Spares.Num(); i++)
    {
        Plan[NumSlabs + i].Vertices.Reset(Page.Spares[i].VertexCapacity);
        Plan[NumSlabs + i].Triangles.Reset(Page.Spares[i].TriangleCapacity);
    }
    auto SlotOf = [&Page, NumSlabs](uint32 Id)
        {
            for (int32 s = 0; s < NumSlabs; s++)
            {
                if (Page.Slabs[s].Id == Id) return s;
            }
            return (int32)INDEX_NONE;
        };

    // Departing bricks (removed, or replaced by their new mesh) free their ranges first.
    for (const FVoxelBrickPageChange* Change : PageChanges)
    {
        const FMember* Member = Page.Members.Find(Change->Key);
        const int32 s = Member && Member->Slab != 0 ? SlotOf(Member->Slab) : INDEX_NONE;
        if (s == INDEX_NONE) continue;
        FPlanSlab& Slot = Plan[s];
        Slot.Vertices.Release(Member->VertexStart, Member->Vertices);
        Slot.Triangles.Release(Member->TriangleStart, Member->Triangles);
        Slot.Freed.Add(FIntPoint(Member->TriangleStart, Member->Triangles));
        Slot.Left++;
        Slot.bTouched = true;
    }

    // New bricks: the lowest free ranges of the first slab with room.
    FIntPoint Overflow(0, 0);
    TArray<int32> PlacedSlot;
    TArray<FIntPoint> PlacedStart;
    PlacedSlot.Init(INDEX_NONE, PageChanges.Num());
    PlacedStart.Init(FIntPoint::ZeroValue, PageChanges.Num());
    for (int32 c = 0; c < PageChanges.Num(); c++)
    {
        const FStaged* Brick = PageChanges[c]->bRemove ? nullptr : Staged.Find(PageChanges[c]->Key);
        if (!Brick || Brick->Triangles <= 0) continue;
        for (int32 s = 0; s < Plan.Num(); s++)
        {
            FPlanSlab& Slot = Plan[s];
            if (!Slot.Vertices.CanAllocate(Brick->Vertices) || !Slot.Triangles.CanAllocate(Brick->Triangles)) continue;
            const int32 V = Slot.Vertices.Allocate(Brick->Vertices);
            const int32 T = Slot.Triangles.Allocate(Brick->Triangles);
            Slot.Placed.Add(FIntPoint(T, Brick->Triangles));
            Slot.Joined++;
            Slot.bTouched = true;
            if (bApply)
            {
                Slot.Jobs.Append({ 0u, (uint32)Brick->CacheVertex, (uint32)V, (uint32)Brick->Vertices,
                    (uint32)Brick->CacheTriangle, (uint32)T, (uint32)Brick->Triangles, Brick->bDebugColor ? 1u : 0u });
            }
            PlacedSlot[c] = s;
            PlacedStart[c] = FIntPoint(V, T);
            break;
        }
        if (PlacedSlot[c] == INDEX_NONE)
            Overflow += FIntPoint(Brick->Vertices, Brick->Triangles);
    }
    if (bOutUsesSpare)
    {
        *bOutUsesSpare = false;
        for (int32 s = NumSlabs; s < Plan.Num(); s++)
            *bOutUsesSpare |= Plan[s].bTouched;
    }
    if (!bApply) return Overflow;

    if (Overflow.X > 0 || Overflow.Y > 0)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelBricks] A page commit had no room for %d vertices and %d triangles: those bricks are not drawn until they change."), Overflow.X, Overflow.Y);
    }
    // Spares that take bricks become slabs, in plan order; SlabOf maps plan slots to slabs.
    TArray<int32> SlabOf;
    SlabOf.SetNum(Plan.Num());
    for (int32 s = 0; s < NumSlabs; s++)
        SlabOf[s] = s;
    TArray<FInstance> Unused;
    for (int32 i = 0; i < Page.Spares.Num(); i++)
    {
        const int32 s = NumSlabs + i;
        if (!Plan[s].bTouched)
        {
            SlabOf[s] = INDEX_NONE;
            Unused.Add(MoveTemp(Page.Spares[i]));
            continue;
        }
        FSlab& Slab = Page.Slabs.AddDefaulted_GetRef();
        Slab.Instance = MoveTemp(Page.Spares[i]);
        Slab.Id = ++Page.NextSlabId;
        SlabOf[s] = Page.Slabs.Num() - 1;
    }
    Page.Spares = MoveTemp(Unused);

    for (int32 c = 0; c < PageChanges.Num(); c++)
    {
        const FVoxelBrickPageChange* Change = PageChanges[c];
        if (Change->bRemove)
        {
            Page.Members.Remove(Change->Key);
            continue;
        }
        FMember& Member = Page.Members.FindOrAdd(Change->Key);
        Member = FMember();
        const FStaged* Brick = Staged.Find(Change->Key);
        if (!Brick) continue;
        Member.Errors = Brick->Errors;
        if (PlacedSlot[c] != INDEX_NONE)
        {
            Member.Slab = Page.Slabs[SlabOf[PlacedSlot[c]]].Id;
            Member.VertexStart = PlacedStart[c].X;
            Member.Vertices = Brick->Vertices;
            Member.TriangleStart = PlacedStart[c].Y;
            Member.Triangles = Brick->Triangles;
        }
        // Its cache ranges are freed once the copy is recorded.
        if (Release && Brick->Vertices > 0)
            Release->Vertices.Add(FIntPoint(Brick->CacheVertex, Brick->Vertices));
        if (Release && Brick->Triangles > 0)
            Release->Triangles.Add(FIntPoint(Brick->CacheTriangle, Brick->Triangles));
        Staged.Remove(Change->Key);
        NumStagedReady--;
    }

    // Each touched slab: its new space, the departed triangles no new brick covers cleared,
    // and the triangles drawn. A slab left without bricks is emptied by that run. Slots go
    // by descending slab, so removing one leaves the others' indices.
    for (int32 s = Plan.Num() - 1; s >= 0; s--)
    {
        FPlanSlab& Slot = Plan[s];
        if (!Slot.bTouched) continue;
        FSlab& Slab = Page.Slabs[SlabOf[s]];
        Slab.Vertices = MoveTemp(Slot.Vertices);
        Slab.Triangles = MoveTemp(Slot.Triangles);
        Slab.NumMembers += Slot.Joined - Slot.Left;
        const int32 End = Slab.Triangles.End();
        FVoxelRangeAllocator::Subtract(Slot.Freed, MoveTemp(Slot.Placed), End);
        for (const FIntPoint& Range : Slot.Freed)
            Slot.Jobs.Append({ 1u, 0u, 0u, 0u, 0u, (uint32)Range.X, (uint32)Range.Y, 0u });
        Perf.CommitSlabs++;
        Perf.CommitJobs += Slot.Jobs.Num() / (int32)VBP::JobWords;
        Push(PageKey, Slab.Instance, MoveTemp(Slot.Jobs), End);
        if (Release)
            Release->Runs.Emplace(Slab.Instance.Provider, Slab.Instance.PushedVersion);
        if (Slab.NumMembers > 0) continue;
        FReleasing& Entry = Releasing.AddDefaulted_GetRef();
        Entry.Instance = MoveTemp(Slab.Instance);
        Entry.Version = Entry.Instance.PushedVersion;
        Entry.Frame = GFrameCounter;
        Page.Slabs.RemoveAt(SlabOf[s]);
    }
    return Overflow;
}

void FVoxelBrickPages::Commit(TConstArrayView<FVoxelBrickPageChange> Changes)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesCommit);
    TMap<FVoxelBrickKey, TArray<const FVoxelBrickPageChange*>> ByPage;
    for (const FVoxelBrickPageChange& Change : Changes)
        ByPage.FindOrAdd(PageOf(Change.Key)).Add(&Change);

    FCacheRelease Release;
    Release.Frame = GFrameCounter;
    for (const TPair<FVoxelBrickKey, TArray<const FVoxelBrickPageChange*>>& Pair : ByPage)
    {
        FPage* Page = Pages.Find(Pair.Key);
        if (!Page)
        {
            bool bAdds = false;
            for (const FVoxelBrickPageChange* Change : Pair.Value)
                bAdds |= !Change->bRemove;
            if (!bAdds) continue;
            Page = &Pages.Add(Pair.Key);
        }
        PlanPage(Pair.Key, *Page, Pair.Value, true, &Release);
        if (Page->Members.Num() == 0 && Page->Slabs.Num() == 0)
        {
            for (FInstance& Spare : Page->Spares)
                PoolInstance(Spare);
            Pages.Remove(Pair.Key);
        }
    }
    if (Release.Vertices.Num() > 0 || Release.Triangles.Num() > 0)
        CacheReleases.Add(MoveTemp(Release));
    Perf.Commits++;
    Perf.CommitChanges += Changes.Num();
    Perf.CommitPages += ByPage.Num();
    Perf.MaxCommitChanges = FMath::Max(Perf.MaxCommitChanges, Changes.Num());
}

void FVoxelBrickPages::Flush(const TSharedPtr<const FVoxelDensityGPUData>& InDensity, const TSharedPtr<const FVoxelEditGPUData>& InEdits)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesFlush);
    const double StartTime = FPlatformTime::Seconds();
    ON_SCOPE_EXIT
    {
        const double Ms = (FPlatformTime::Seconds() - StartTime) * 1000.0;
        Perf.Frames++;
        Perf.FlushMs += Ms;
        Perf.MaxFlushMs = FMath::Max(Perf.MaxFlushMs, Ms);
    };
    Density = InDensity;
    Edits = InEdits;
    BeginCreates();
    auto CollectCounters = [this](const FInstance& Instance)
        {
            if (!Instance.IsValid()) return;
            FVoxelBrickPageProvider& Provider = *Instance.Provider;
            Perf.Calls += Provider.Calls.exchange(0, std::memory_order_relaxed);
            Perf.IdleCalls += Provider.IdleCalls.exchange(0, std::memory_order_relaxed);
            Perf.Runs += Provider.PerfRuns.exchange(0, std::memory_order_relaxed);
            Perf.RenderMicros += Provider.RenderMicros.exchange(0, std::memory_order_relaxed);
        };

    TakeBatchResults();

    // Cache ranges whose copies are recorded (or whose runs are gone).
    for (int32 r = 0; r < CacheReleases.Num(); r++)
    {
        FCacheRelease& Entry = CacheReleases[r];
        bool bDone = GFrameCounter > Entry.Frame + VBP::CacheTimeoutFrames;
        // A run never recorded may still copy from these ranges: the cache keeps its size.
        if (bDone && Entry.Runs.Num() > 0)
            NoShrinkUntil = GFrameCounter + VBP::CacheTimeoutFrames;
        for (int32 i = 0; i < Entry.Runs.Num() && !bDone; i++)
        {
            const TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe> Provider = Entry.Runs[i].Key.Pin();
            if (!Provider.IsValid() || Provider->ShownVersion.load(std::memory_order_acquire) >= Entry.Runs[i].Value)
                Entry.Runs.RemoveAtSwap(i--);
        }
        if (!bDone && Entry.Runs.Num() > 0) continue;
        for (const FIntPoint& Range : Entry.Vertices)
            CacheVertices.Release(Range.X, Range.Y);
        for (const FIntPoint& Range : Entry.Triangles)
            CacheTriangles.Release(Range.X, Range.Y);
        CacheReleases.RemoveAtSwap(r--);
    }

    // Emptied buffer sets, pooled once their emptying run is recorded.
    for (int32 r = 0; r < Releasing.Num(); r++)
    {
        FReleasing& Entry = Releasing[r];
        Advance(Entry.Instance);
        CollectCounters(Entry.Instance);
        if (Entry.Instance.IsValid() && Entry.Instance.Provider->ShownVersion.load(std::memory_order_acquire) >= Entry.Version)
            PoolInstance(Entry.Instance);
        else if (!Entry.Instance.IsValid() || GFrameCounter > Entry.Frame + VBP::EmptyingTimeoutFrames)
            DestroyInstance(Entry.Instance);
        else
            continue;
        Releasing.RemoveAtSwap(r--);
    }

    // Registration and kicks; unused spares, and pages left with nothing.
    TArray<FVoxelBrickKey> Emptied;
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        FPage& Page = Pair.Value;
        for (FSlab& Slab : Page.Slabs)
        {
            Advance(Slab.Instance);
            CollectCounters(Slab.Instance);
            const uint32 MissingBits = Slab.Instance.IsValid() ? Slab.Instance.Provider->Missing.load(std::memory_order_relaxed) : 0u;
            if (MissingBits != 0 && (MissingBits & ~WarnedMissing) != 0)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelBricks] A brick page's compute run is missing: %s%s."), *DescribeMissing(MissingBits),
                    (MissingBits & (uint32)VBP::EMissing::Required) ? TEXT(" (run skipped, retried)") : TEXT(" (written elsewhere, run continues)"));
                WarnedMissing |= MissingBits;
            }
        }
        const bool bUnused = GFrameCounter > Page.SpareUsedFrame + VBP::UnusedFrames;
        for (FInstance& Spare : Page.Spares)
        {
            Advance(Spare);
            CollectCounters(Spare);
            if (bUnused)
                PoolInstance(Spare);
        }
        if (bUnused)
            Page.Spares.Reset();
        if (Page.Members.Num() == 0 && Page.Slabs.Num() == 0 && Page.Spares.Num() == 0)
            Emptied.Add(Pair.Key);
    }
    for (const FVoxelBrickKey& Key : Emptied)
        Pages.Remove(Key);
    for (TPair<uint64, TArray<FInstance>>& Pair : Pool)
    {
        for (FInstance& Instance : Pair.Value)
        {
            Advance(Instance);
            CollectCounters(Instance);
        }
    }

    DispatchBatches();
    ShrinkCache();
    SET_DWORD_STAT(STAT_VoxelPagesCount, Pages.Num());
    SET_DWORD_STAT(STAT_VoxelPagesPooled, NumPooled);
    SET_DWORD_STAT(STAT_VoxelPagesStaged, Staged.Num());
}

void FVoxelBrickPages::ShrinkCache()
{
    const FIntPoint Used(CacheVertices.GetNumUsed(), CacheTriangles.GetNumUsed());
    CachePeak = CachePeak.ComponentMax(Used);
    Perf.CacheUsed = Perf.CacheUsed.ComponentMax(Used);
    if (GFrameCounter >= CachePeakFrame + VBP::CachePeakFrames)
    {
        CachePeakBefore = CachePeak;
        CachePeak = Used;
        CachePeakFrame = GFrameCounter;
    }
    // Only an empty cache shrinks: nothing staged, being meshed or waiting to be copied.
    if (Used.X > 0 || Used.Y > 0 || Batches.Num() > 0 || CacheReleases.Num() > 0 || GFrameCounter < NoShrinkUntil) return;
    const FIntPoint Peak = CachePeak.ComponentMax(CachePeakBefore);
    auto Target = [](int32 PeakUsed, int32 Initial, int32 Current)
        {
            const uint32 Wanted = (uint32)FMath::CeilToInt(PeakUsed * VBP::CacheShrinkMargin);
            return FMath::Min(Current, FMath::Max(Initial, (int32)FMath::RoundUpToPowerOfTwo(FMath::Max(Wanted, 1u))));
        };
    const int32 V = Target(Peak.X, VBP::InitialCacheVertices, CacheVertices.GetCapacity());
    const int32 T = Target(Peak.Y, VBP::InitialCacheTriangles, CacheTriangles.GetCapacity());
    if (V == CacheVertices.GetCapacity() && T == CacheTriangles.GetCapacity()) return;
    CacheVertices.Reset(V);
    CacheTriangles.Reset(T);
    Perf.CacheShrinks++;
    ENQUEUE_RENDER_COMMAND(VoxelBrickCacheShrink)([GPURef = GPU, V, T](FRHICommandListImmediate&)
        {
            GPURef->ShrinkCache(V, T);
        });
}

FString FVoxelBrickPages::TakePerf()
{
    int32 Instances = 0;
    int64 Bytes = 0, Triangles = 0;
    for (const TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        auto Count = [&Instances, &Bytes](const FInstance& Instance)
            {
                if (!Instance.IsValid()) return;
                Instances++;
                Bytes += (int64)Instance.VertexCapacity * VBP::BytesPerVertex + (int64)Instance.TriangleCapacity * VBP::BytesPerTriangle;
            };
        for (const FSlab& Slab : Pair.Value.Slabs)
        {
            Count(Slab.Instance);
            Triangles += Slab.Triangles.GetNumUsed();
        }
        for (const FInstance& Spare : Pair.Value.Spares)
            Count(Spare);
    }
    const int64 Meshed = GPU->BricksMeshed.exchange(0, std::memory_order_relaxed);
    const int64 StageMicros = GPU->StageMicros.exchange(0, std::memory_order_relaxed);
    const double PerFrame = 1.0 / FMath::Max(1, Perf.Frames);
    const double PerCommit = 1.0 / FMath::Max(1, Perf.Commits);
    auto CacheMB = [](int64 NumVertices, int64 NumTriangles)
        {
            return ((double)NumVertices * VBP::CacheWords * 4.0 + (double)NumTriangles * 12.0) / (1024.0 * 1024.0);
        };
    const FString Out = FString::Printf(TEXT("Pages: %d pages, %d buffer sets (+%d pooled), %.1f MB, %lld triangles. Commits: %d (%.0f changes, %.0f pages, %.0f buffer sets written and %.0f jobs on average; %d changes at most), %d frames waiting for the mesher, %d for buffer sets. Mesher per frame: %.2f batches, %.1f bricks sent, %.1f meshed (%d sent again), %.2f ms render thread; cache %.1f MB (%.1f MB used at most, %d shrinks), %d bricks staged. Providers per frame: %.1f calls (%.1f idle), %.2f runs, %.2f ms render thread; buffer sets %.2f created (%.2f ms each, %.2f ms max), %.2f recycled, %d spares added. Flush %.2f ms average, %.2f ms max."),
        Pages.Num(), Instances, NumPooled, Bytes / (1024.0 * 1024.0), Triangles,
        Perf.Commits, Perf.CommitChanges * PerCommit, Perf.CommitPages * PerCommit, Perf.CommitSlabs * PerCommit, Perf.CommitJobs * PerCommit, Perf.MaxCommitChanges,
        Perf.MeshWaits, Perf.SlabWaits,
        Perf.Batches * PerFrame, Perf.BricksSent * PerFrame, Meshed * PerFrame, Perf.BricksBack, StageMicros * PerFrame / 1000.0,
        CacheMB(CacheVertices.GetCapacity(), CacheTriangles.GetCapacity()), CacheMB(Perf.CacheUsed.X, Perf.CacheUsed.Y), Perf.CacheShrinks, Staged.Num(),
        Perf.Calls * PerFrame, Perf.IdleCalls * PerFrame, Perf.Runs * PerFrame, Perf.RenderMicros * PerFrame / 1000.0,
        Perf.Created * PerFrame, Perf.Created > 0 ? Perf.CreateMs / Perf.Created : 0.0, Perf.MaxCreateMs, Perf.Recycled * PerFrame, Perf.SparesAdded,
        Perf.FlushMs * PerFrame, Perf.MaxFlushMs);
    Perf = FPerf();
    return Out;
}

FString FVoxelBrickPages::DescribeMissing(uint32 Bits)
{
    // VBP::EMissing, bit by bit.
    static const TCHAR* Names[] = { TEXT("page shaders"), TEXT("Position stream"), TEXT("Tangents stream"),
        TEXT("TexCoords stream"), TEXT("Color stream"), TEXT("Triangles stream"), TEXT("indirect args"), TEXT("capacity") };
    FString Out;
    for (int32 i = 0; i < UE_ARRAY_COUNT(Names); i++)
    {
        if (Bits & (1u << i))
            Out += FString::Printf(TEXT("%s%s"), Out.IsEmpty() ? TEXT("") : TEXT(", "), Names[i]);
    }
    return Out;
}

FString FVoxelBrickPages::Describe() const
{
    int32 Slabs = 0, Spares = 0, Members = 0, Unregistered = 0, Lagging = 0;
    uint32 Runs = 0, Skipped = 0, MissingBits = 0, Errors = 0;
    int64 Bytes = 0, Triangles = 0;
    for (const TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        const FPage& Page = Pair.Value;
        Members += Page.Members.Num();
        for (const TPair<FVoxelBrickKey, FMember>& Member : Page.Members)
            Errors |= Member.Value.Errors;
        auto Count = [&](const FInstance& Instance)
            {
                if (!Instance.IsValid()) return;
                Bytes += (int64)Instance.VertexCapacity * VBP::BytesPerVertex + (int64)Instance.TriangleCapacity * VBP::BytesPerTriangle;
                Unregistered += Instance.Handle == 0 ? 1 : 0;
                Lagging += Instance.Provider->ShownVersion.load(std::memory_order_relaxed) < Instance.PushedVersion ? 1 : 0;
                Runs += Instance.Provider->Runs.load(std::memory_order_relaxed);
                Skipped += Instance.Provider->SkippedRuns.load(std::memory_order_relaxed);
                MissingBits |= Instance.Provider->Missing.load(std::memory_order_relaxed);
            };
        for (const FSlab& Slab : Page.Slabs)
        {
            Slabs++;
            Count(Slab.Instance);
            Triangles += Slab.Triangles.GetNumUsed();
        }
        Spares += Page.Spares.Num();
        for (const FInstance& Spare : Page.Spares)
            Count(Spare);
    }
    int32 Queued = 0, Meshing = 0;
    for (const TPair<FVoxelBrickKey, FStaged>& Pair : Staged)
    {
        Queued += Pair.Value.State == EStage::Queued ? 1 : 0;
        Meshing += Pair.Value.State == EStage::Meshing ? 1 : 0;
    }
    return FString::Printf(TEXT("GPU pages: %d (%d bricks, %d buffer sets and %d spares, %d not yet registered, %d not yet drawing their newest state), %u runs, %u skipped%s%s, %.1f MB of buffers, %lld triangles held, %d buffer sets being emptied, %d pooled. Staged: %d bricks (%d queued, %d meshing, %d ready), %d batches in flight; cache %d of %d vertices and %d of %d triangles used. Mesher errors 0x%x"),
        Pages.Num(), Members, Slabs, Spares, Unregistered, Lagging, Runs, Skipped, MissingBits ? TEXT(", missing: ") : TEXT(""), *DescribeMissing(MissingBits),
        Bytes / (1024.0 * 1024.0), Triangles, Releasing.Num(), NumPooled,
        Staged.Num(), Queued, Meshing, NumStagedReady, Batches.Num(),
        CacheVertices.GetNumUsed(), CacheVertices.GetCapacity(), CacheTriangles.GetNumUsed(), CacheTriangles.GetCapacity(), Errors);
}
