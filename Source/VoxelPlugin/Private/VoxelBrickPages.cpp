#include "VoxelBrickPages.h"
#include "FAdaptiveOctree.h"
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
#include "Misc/ScopeExit.h"
#include "VoxelTerrainStats.h"

DECLARE_CYCLE_STAT(TEXT("Pages: flush"), STAT_VoxelPagesFlush, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: create buffer set"), STAT_VoxelPagesCreate, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: push state"), STAT_VoxelPagesPush, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: provider (render thread)"), STAT_VoxelPagesProvider, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Pages: frame batch (render thread)"), STAT_VoxelPagesBatch, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: re-mesh runs"), STAT_VoxelPagesRemeshRuns, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: bricks meshed"), STAT_VoxelPagesBricksMeshed, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: bricks copied"), STAT_VoxelPagesBricksReused, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: idle provider calls"), STAT_VoxelPagesIdleCalls, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: buffer sets created"), STAT_VoxelPagesCreated, STATGROUP_VoxelTerrain);
DECLARE_DWORD_COUNTER_STAT(TEXT("Pages: buffer sets recycled"), STAT_VoxelPagesRecycled, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Pages"), STAT_VoxelPagesCount, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Pages: pooled buffer sets"), STAT_VoxelPagesPooled, STATGROUP_VoxelTerrain);
DECLARE_GPU_STAT(VoxelBrickMesh);

using namespace RealtimeMesh;

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelPageParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, MesherCounters)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, MesherVertices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, MesherIndices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, BrickOffsets)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, PageState)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, Verdict)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, CommitVerdict)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, BrickBase)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float>, OutPositions)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTangents)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutColors)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutIndices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutArgs)
    SHADER_PARAMETER(uint32, NumBricks)
    SHADER_PARAMETER(uint32, VertexCap)
    SHADER_PARAMETER(uint32, TriangleCap)
    SHADER_PARAMETER(uint32, VertexCapacity)
    SHADER_PARAMETER(uint32, TriangleCapacity)
    SHADER_PARAMETER(uint32, Color)
    SHADER_PARAMETER(uint32, StreamMask)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, BrickTable)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OldTable)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OldPositions)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OldTangents)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OldTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, OldIndices)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, BrickMap)
    SHADER_PARAMETER(uint32, VertexStride)
    SHADER_PARAMETER(FVector3f, UVOffset)
    SHADER_PARAMETER(float, UVScale)
    SHADER_PARAMETER(uint32, GroupsX)
END_SHADER_PARAMETER_STRUCT()

#define VOXEL_PAGE_SHADER(ShaderName) \
    class ShaderName : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(ShaderName); \
        using FParameters = FVoxelPageParameters; \
        SHADER_USE_PARAMETER_STRUCT(ShaderName, FGlobalShader); \
        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
        { \
            return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
        } \
    };

VOXEL_PAGE_SHADER(FVoxelPageOffsetsCS)
VOXEL_PAGE_SHADER(FVoxelPageVerticesCS)
VOXEL_PAGE_SHADER(FVoxelPageTrianglesCS)
VOXEL_PAGE_SHADER(FVoxelPageReuseOffsetsCS)
VOXEL_PAGE_SHADER(FVoxelPageReuseVerticesCS)
VOXEL_PAGE_SHADER(FVoxelPageReuseTrianglesCS)
VOXEL_PAGE_SHADER(FVoxelPageArgsCS)
#undef VOXEL_PAGE_SHADER

IMPLEMENT_GLOBAL_SHADER(FVoxelPageOffsetsCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "OffsetsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageVerticesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "VerticesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageTrianglesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "TrianglesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageReuseOffsetsCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "ReuseOffsetsCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageReuseVerticesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "ReuseVerticesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageReuseTrianglesCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "ReuseTrianglesCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelPageArgsCS, "/Plugin/VoxelPlugin/Private/VoxelBrickPageGPU.usf", "ArgsCS", SF_Compute);

static TAutoConsoleVariable<int32> CVarVoxelBrickPageCreates(
    TEXT("voxel.Render.BrickPageCreates"), 4,
    TEXT("Brick page buffer sets created per frame at most (recycled ones don't count)."),
    ECVF_Default);

namespace VoxelBrickPagesDetail
{
    constexpr int32 PageShift = 2;                   // pages of 4x4x4 bricks
    constexpr int32 BatchCells = 128 * 4096;         // cells per mesher dispatch set in the frame batch (128 16-cell bricks)
    constexpr int32 MaxPooled = 64;                  // emptied buffer sets kept for reuse
    constexpr int32 PoolSlack = 2;                   // a pooled set may be up to 2^this times larger on each count
    constexpr double BoundsMarginCells = 12.0;       // page bounds beyond its bricks (cells of its level), with the morph
    constexpr int32 MinVertices = 4096;
    constexpr int32 MinTriangles = 8192;
    constexpr int32 MaxVertices = 1 << 22;
    constexpr int32 MaxTriangles = 1 << 22;
    constexpr uint64 KickFrames = 3;                 // frames before a run that didn't happen is asked for again
    constexpr uint64 UnusedFrames = 60;              // an empty page or an unused replacement is released after
    constexpr double CapacityMargin = 1.25;          // over the estimated contents
    constexpr int32 CreateRampFrames = 4;            // a waiting commit raises the creation budget every this many frames
    constexpr int32 MaxCreateRamp = 3;               // creation costs about 0.25 ms each on the game thread
    constexpr double UnknownBrickWeight = 1.5;       // a brick not yet meshed counts this many level averages
    constexpr double AverageBlend = 0.05;            // per surface brick read back
    constexpr int32 MaxPendingReadbacks = 4;
    constexpr uint32 PageStateWords = 9;             // VoxelBrickPageGPU.usf PageState
    constexpr uint64 VerifyTimeoutFrames = 60;       // a commit not read back by then counts as drawn
    constexpr int32 BytesPerVertex = 12 + 8 + 40 + 4;   // position, two packed normals, five float2 UVs, colour
    constexpr int32 TexCoordChannels = 5;            // VoxelBrickPageGPU.usf
    constexpr int32 BytesPerTriangle = 12;

    const FName PageGroupName(TEXT("BRICKPAGE"));

    /** Bricks per mesher dispatch set in the frame batch: the scratch buffers grow with the
     *  cells per brick (about 50 MB at the default). */
    int32 BatchBricksFor(const FVoxelBrickMesherGPUSettings& Settings)
    {
        return FMath::Clamp(BatchCells / FMath::Cube(VoxelBrickMesherGPU::CellsFor(Settings)), 8, 256);
    }

    uint64 PoolKey(int32 Level, int32 VertexCapacity, int32 TriangleCapacity)
    {
        return ((uint64)(uint32)Level << 48) ^ ((uint64)(uint32)FMath::FloorLog2((uint32)FMath::Max(1, VertexCapacity)) << 24)
            ^ (uint64)(uint32)FMath::FloorLog2((uint32)FMath::Max(1, TriangleCapacity));
    }

    int64 FloorShift(int64 Value, int32 Levels)
    {
        return Value >= 0 ? (Value >> Levels) : -((-Value + (1LL << Levels) - 1) >> Levels);
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
            && ShaderMap->GetShader(&FVoxelPageOffsetsCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageVerticesCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageTrianglesCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageReuseOffsetsCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageReuseVerticesCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageReuseTrianglesCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelPageArgsCS::GetStaticType(), 0).IsValid();
    }

    template<typename TShader>
    void AddPagePass(FRDGBuilder& GraphBuilder, const FVoxelPageParameters& Template, uint32 NumThreads, uint32 GroupSize, const TCHAR* Name)
    {
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(NumThreads, GroupSize), 1u);
        const uint32 GroupsX = FMath::Min(NumGroups, 32768u);
        const uint32 GroupsY = FMath::DivideAndRoundUp(NumGroups, GroupsX);
        FVoxelPageParameters* Parameters = GraphBuilder.AllocParameters<FVoxelPageParameters>();
        *Parameters = Template;
        Parameters->GroupsX = GroupsX;
        TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelBrickPage %s", Name), Shader, Parameters, FIntVector((int32)GroupsX, (int32)GroupsY, 1));
    }
}

namespace VBP = VoxelBrickPagesDetail;

/** Render thread. The density octaves and edit tree on the GPU, shared by one terrain's pages
 *  and uploaded again only when the data changes. */
struct FVoxelBrickPageGPUInputs
{
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TRefCountPtr<FRDGPooledBuffer> Octaves;
    TRefCountPtr<FRDGPooledBuffer> EditCorners;
    TRefCountPtr<FRDGPooledBuffer> EditChildren;
    TRefCountPtr<FRDGPooledBuffer> EditFlags;

    VoxelBrickMesherGPU::FInputs Get(FRDGBuilder& GraphBuilder, const TSharedPtr<const FVoxelDensityGPUData>& InDensity,
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

    /** Providers given a new state since the last frame batch. */
    TArray<TWeakPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>> Pending;
    /** Once per frame, by the first provider the driver runs (all providers record into the
     *  same graph): the bricks every pending provider must mesh, meshed together, then every
     *  pending provider's placement, with one verdict for all of them. */
    uint32 BatchFrame = ~0u;
    const FRDGBuilder* BatchGraph = nullptr;
    TArray<VoxelBrickMesherGPU::FOutputs> BatchOutputs;
    std::atomic<int64> BatchBricks{ 0 };   // meshed in frame batches (for the perf log)

    void BuildFrameBatch(FRDGBuilder& GraphBuilder, uint32 FrameNumber);
};

/** Meshes one page's bricks into its buffers when the compute driver runs it (render thread). */
class FVoxelBrickPageProvider final : public IRealtimeMeshComputeProvider
{
public:
    struct FState
    {
        TArray<FVoxelMesherBrickGPU> Bricks;
        TArray<FVector4f> Offsets;               // brick min from the page origin
        TArray<FVoxelBrickKey> Keys;             // per brick
        TArray<uint32> Stamps;                   // per brick: changes whenever it must be meshed again
        bool bFull = false;                      // mesh every brick (an instance's first state)
        TSharedPtr<const FVoxelDensityGPUData> Density;
        TSharedPtr<const FVoxelEditGPUData> Edits;
        FVoxelBrickMesherGPUSettings Settings;
        FVector3f UVOffset = FVector3f::ZeroVector;
        float UVScale = 0.0f;
        uint32 Color = 0xffffffffu;
        int32 VertexCapacity = 0;
        int32 TriangleCapacity = 0;
        uint64 Version = 0;
    };

    explicit FVoxelBrickPageProvider(const TSharedRef<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe>& InInputs)
        : Inputs(InInputs)
    {
    }

    void SetState_RenderThread(FState&& InState)
    {
        State = MoveTemp(InState);
        bStatePushed = true;
    }

    /** Render thread: Self has a new state for the next frame batch. */
    static void QueueForBatch(const TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>& Self)
    {
        Self->Inputs->Pending.Add(Self);
    }

    /** A re-mesh of the current state: the bricks copied from the buffers (previous row and
     *  member index, in pairs) and the members meshed, with where the frame batch put them. */
    struct FPlan
    {
        uint32 Frame = ~0u;                 // the frame batch that meshes them, if any
        const FRDGBuilder* Graph = nullptr;
        uint64 Version = 0;
        TArray<uint32> Reuse;
        TArray<int32> Mesh;
        TArray<int32> MeshOutput;           // per meshed member: frame batch output, -1 none
        TArray<int32> MeshRow;              // and its row there
    };

    /** False when the buffers already hold the current state. */
    bool MakePlan(FPlan& Out) const
    {
        if (!State.Density.IsValid() || !State.Edits.IsValid() || !bStatePushed || MeshedVersion == State.Version) return false;

        Out = FPlan();
        Out.Version = State.Version;
        // Bricks the buffers hold at the same stamp are copied; the rest are meshed. Nothing is
        // copied while the last run's verdict is unknown (the buffers may hold either table).
        const bool bCanReuse = !State.bFull && Table.IsValid() && TentativeVersion == 0;
        for (int32 i = 0; i < State.Bricks.Num(); i++)
        {
            const int32* Row = bCanReuse ? TableRows.Find(State.Keys[i]) : nullptr;
            if (Row && TableStamps[*Row] == State.Stamps[i])
            {
                Out.Reuse.Add((uint32)*Row);
                Out.Reuse.Add((uint32)i);
            }
            else
            {
                Out.Mesh.Add(i);
            }
        }
        Out.MeshOutput.Init(-1, Out.Mesh.Num());
        Out.MeshRow.Init(0, Out.Mesh.Num());
        return true;
    }

    FPlan Plan;

    /** Where a re-mesh run's bricks go, recorded by the frame batch (or by the run itself when
     *  it meshes its own): the page counters, the new table, the verdict it shares, and per
     *  source (the copied bricks, then each mesher output used) the brick map, offsets and
     *  placement. */
    struct FSource
    {
        VoxelBrickMesherGPU::FOutputs Meshed;
        int32 Count = 0;
        FRDGBufferRef Map = nullptr;
        FRDGBufferRef Offsets = nullptr;
        FRDGBufferRef Base = nullptr;
    };
    struct FLayout
    {
        uint32 Frame = ~0u;
        const FRDGBuilder* Graph = nullptr;
        uint64 Version = 0;
        FRDGBufferRef PageState = nullptr;
        FRDGBufferRef NewTable = nullptr;
        FRDGBufferRef Verdict = nullptr;
        FRDGBufferRef ReuseMap = nullptr;
        FRDGBufferRef ReuseBase = nullptr;
        TArray<FSource> Sources;
    };
    FLayout Layout;

    /** Records the placement of Plan's bricks (copied, then meshed in Outputs). */
    void AddPlacement(FRDGBuilder& GraphBuilder, uint32 Frame, TConstArrayView<VoxelBrickMesherGPU::FOutputs> Outputs, FRDGBufferRef Verdict);

    virtual void BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context) override;

    void PollStats_RenderThread()
    {
        for (int32 i = 0; i < Readbacks.Num(); i++)
        {
            FPendingReadback& Pending = Readbacks[i];
            if (!Pending.Readback->IsReady() || (Pending.TableReadback.IsValid() && !Pending.TableReadback->IsReady())) continue;
            if (const uint32* Data = (const uint32*)Pending.Readback->Lock(VBP::PageStateWords * sizeof(uint32)))
            {
                FVoxelBrickPageStats Read;
                Read.Version = Pending.Version;
                Read.bCommitted = Data[8] != 0u;
                Read.VerticesNeeded = Data[0];
                Read.TrianglesNeeded = Data[1];
                Read.Errors = Data[2];
                Read.VerticesWritten = Data[3];
                Read.TrianglesWritten = Data[4];
                Read.BricksDropped = Data[5];
                Read.VertexCapacity = Data[6];
                Read.TriangleCapacity = Data[7];
                Read.Bricks = Pending.Bricks;
                Pending.Readback->Unlock();
                if (Pending.TableReadback.IsValid())
                {
                    const int32 Members = Pending.Keys.Num();
                    if (const uint32* Rows = (const uint32*)Pending.TableReadback->Lock(4 * Members * sizeof(uint32)))
                    {
                        Read.Keys = MoveTemp(Pending.Keys);
                        Read.Stamps = MoveTemp(Pending.Stamps);
                        Read.Counts.SetNumUninitialized(2 * Members);
                        for (int32 m = 0; m < Members; m++)
                        {
                            Read.Counts[2 * m] = Rows[4 * m + 1];
                            Read.Counts[2 * m + 1] = Rows[4 * m + 3];
                        }
                        Pending.TableReadback->Unlock();
                    }
                }
                // A refused run left the buffers as they were, so the table copies come from is
                // the previous one again; if that is uncertain too, nothing is copied next run.
                if (TentativeVersion != 0 && Read.Version == PrevMeshedVersion)
                    bPrevValid = Read.bCommitted;
                if (Read.Version == TentativeVersion)
                {
                    if (!Read.bCommitted && bPrevValid)
                    {
                        Table = MoveTemp(PrevTable);
                        TableRows = MoveTemp(PrevTableRows);
                        TableStamps = MoveTemp(PrevTableStamps);
                        MeshedVersion = PrevMeshedVersion;
                    }
                    else if (!Read.bCommitted)
                    {
                        Table.SafeRelease();
                        TableRows.Reset();
                        TableStamps.Reset();
                        MeshedVersion = 0;
                    }
                    PrevTable.SafeRelease();
                    PrevTableRows.Reset();
                    PrevTableStamps.Reset();
                    TentativeVersion = 0;
                }
                FScopeLock Lock(&StatsCS);
                if (!bNewStats || Read.Version >= Stats.Version)
                {
                    Stats = Read;
                    bNewStats = true;
                }
            }
            Readbacks.RemoveAt(i--);
        }
    }

    /** Game thread. The latest counters, once each. */
    bool TakeStats(FVoxelBrickPageStats& Out)
    {
        FScopeLock Lock(&StatsCS);
        if (!bNewStats) return false;
        Out = Stats;
        bNewStats = false;
        return true;
    }

    /** The newest state version recorded into a frame's graph. */
    std::atomic<uint64> ShownVersion{ 0 };
    /** Runs that found no shaders or a required stream missing, runs recorded, and the
     *  OR of EVoxelPageMissing bits seen (optional streams missing don't skip a run). */
    std::atomic<uint32> SkippedRuns{ 0 };
    std::atomic<uint32> Runs{ 0 };
    std::atomic<uint32> Missing{ 0 };
    /** For voxel.Debug.BrickPerf, taken and reset by Flush: calls from the driver, calls
     *  that did nothing, re-mesh runs, bricks meshed and copied, and the render-thread time
     *  spent here. */
    std::atomic<uint32> Calls{ 0 };
    std::atomic<uint32> IdleCalls{ 0 };
    std::atomic<uint32> PerfRuns{ 0 };
    std::atomic<uint32> BricksMeshed{ 0 };
    std::atomic<uint32> BricksReused{ 0 };
    std::atomic<uint64> RenderMicros{ 0 };

private:
    struct FPendingReadback
    {
        TUniquePtr<FRHIGPUBufferReadback> Readback;
        TUniquePtr<FRHIGPUBufferReadback> TableReadback;   // the members' rows
        TArray<FVoxelBrickKey> Keys;
        TArray<uint32> Stamps;
        uint64 Version = 0;
        int32 Bricks = 0;
    };

    TSharedRef<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe> Inputs;
    FState State;
    // Render thread: the version the buffers hold.
    uint64 MeshedVersion = 0;
    // Where each brick of the meshed state sits in the buffers (BrickTable rows, in the
    // state's order), and the stamp it was meshed at.
    TRefCountPtr<FRDGPooledBuffer> Table;
    TMap<FVoxelBrickKey, int32> TableRows;
    TArray<uint32> TableStamps;
    // Until the verdict of the run at TentativeVersion is read back: the table and version
    // before it, restored if it was refused.
    TRefCountPtr<FRDGPooledBuffer> PrevTable;
    TMap<FVoxelBrickKey, int32> PrevTableRows;
    TArray<uint32> PrevTableStamps;
    uint64 PrevMeshedVersion = 0;
    uint64 TentativeVersion = 0;
    bool bPrevValid = false;
    bool bStatePushed = false;   // State has not been meshed since SetState_RenderThread
    TArray<FPendingReadback> Readbacks;
    FCriticalSection StatsCS;

    friend struct FVoxelBrickPageGPUInputs;
    FVoxelBrickPageStats Stats;
    bool bNewStats = false;
};

void FVoxelBrickPageGPUInputs::BuildFrameBatch(FRDGBuilder& GraphBuilder, uint32 FrameNumber)
{
    if (BatchFrame == FrameNumber && BatchGraph == &GraphBuilder) return;
    BatchFrame = FrameNumber;
    BatchGraph = &GraphBuilder;
    BatchOutputs.Reset();
    if (!VoxelBrickMesherGPU::AreShadersReady() || !VBP::ArePageShadersReady()) return;

    // Every pending provider's state: planned, its new bricks meshed (one set of mesher passes
    // per density and edit set), then placed. All placements share one verdict, so a commit
    // is drawn whole or not at all.
    TArray<TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>> Planned;
    for (const TWeakPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>& Weak : Pending)
    {
        const TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe> Provider = Weak.Pin();
        if (!Provider.IsValid() || (Provider->Plan.Frame == FrameNumber && Provider->Plan.Graph == &GraphBuilder)) continue;
        if (!Provider->MakePlan(Provider->Plan)) continue;
        Provider->Plan.Frame = FrameNumber;
        Provider->Plan.Graph = &GraphBuilder;
        Planned.Add(Provider);
    }
    Pending.Reset();
    if (Planned.Num() == 0) return;

    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelBrickMesh, "VoxelBrickMesh frame batch (%d pages)", Planned.Num());
    TArray<bool> Grouped;
    Grouped.Init(false, Planned.Num());
    int64 Meshed = 0;
    for (int32 First = 0; First < Planned.Num(); First++)
    {
        if (Grouped[First]) continue;
        const FVoxelBrickPageProvider::FState& Lead = Planned[First]->State;
        const int32 RowsPerOutput = VBP::BatchBricksFor(Lead.Settings);
        const int32 OutputBase = BatchOutputs.Num();
        TArray<FVoxelMesherBrickGPU> Bricks;
        for (int32 p = First; p < Planned.Num(); p++)
        {
            FVoxelBrickPageProvider& Provider = *Planned[p];
            if (Grouped[p] || Provider.State.Density != Lead.Density || Provider.State.Edits != Lead.Edits) continue;
            Grouped[p] = true;
            FVoxelBrickPageProvider::FPlan& ProviderPlan = Provider.Plan;
            for (int32 k = 0; k < ProviderPlan.Mesh.Num(); k++)
            {
                ProviderPlan.MeshOutput[k] = OutputBase + Bricks.Num() / RowsPerOutput;
                ProviderPlan.MeshRow[k] = Bricks.Num() % RowsPerOutput;
                Bricks.Add(Provider.State.Bricks[ProviderPlan.Mesh[k]]);
            }
        }
        if (Bricks.Num() == 0) continue;
        const VoxelBrickMesherGPU::FInputs MesherInputs = Get(GraphBuilder, Lead.Density, Lead.Edits);
        for (int32 Start = 0; Start < Bricks.Num(); Start += RowsPerOutput)
        {
            const int32 Count = FMath::Min(RowsPerOutput, Bricks.Num() - Start);
            BatchOutputs.Add(VoxelBrickMesherGPU::AddMesherPasses(GraphBuilder, *Lead.Density, MesherInputs, Lead.Settings,
                TConstArrayView<FVoxelMesherBrickGPU>(Bricks.GetData() + Start, Count)));
        }
        Meshed += Bricks.Num();
    }

    FRDGBufferRef Verdict = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 1), TEXT("Voxel.PageVerdict"));
    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Verdict), 1u);
    for (const TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>& Provider : Planned)
        Provider->AddPlacement(GraphBuilder, FrameNumber, BatchOutputs, Verdict);
    BatchBricks += Meshed;
}

void FVoxelBrickPageProvider::AddPlacement(FRDGBuilder& GraphBuilder, uint32 Frame, TConstArrayView<VoxelBrickMesherGPU::FOutputs> Outputs, FRDGBufferRef Verdict)
{
    Layout = FLayout();
    Layout.Frame = Frame;
    Layout.Graph = &GraphBuilder;
    Layout.Version = State.Version;
    Layout.Verdict = Verdict;
    Layout.PageState = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), VBP::PageStateWords), TEXT("Voxel.PageState"));
    Layout.NewTable = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 4 * FMath::Max(1, State.Bricks.Num())), TEXT("Voxel.PageBrickTable"));

    FVoxelPageParameters Template;
    Template.PageState = GraphBuilder.CreateUAV(Layout.PageState);
    Template.Verdict = GraphBuilder.CreateUAV(Verdict);
    Template.BrickTable = GraphBuilder.CreateUAV(Layout.NewTable);
    Template.VertexCapacity = (uint32)FMath::Max(0, State.VertexCapacity);
    Template.TriangleCapacity = (uint32)FMath::Max(0, State.TriangleCapacity);
    AddClearUAVPass(GraphBuilder, Template.PageState, 0u);

    // Copied bricks first (in the previous table's rows), then each mesher output's.
    const uint32 NumReused = (uint32)Plan.Reuse.Num() / 2u;
    if (NumReused > 0)
    {
        Layout.ReuseMap = CreateStructuredBuffer(GraphBuilder, TEXT("Voxel.PageReuseMap"), sizeof(uint32), Plan.Reuse.Num(), Plan.Reuse.GetData(), sizeof(uint32) * Plan.Reuse.Num());
        Layout.ReuseBase = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 3 * NumReused), TEXT("Voxel.PageBrickBase"));
        FVoxelPageParameters Pass = Template;
        Pass.OldTable = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Table));
        Pass.BrickMap = GraphBuilder.CreateSRV(Layout.ReuseMap);
        Pass.BrickBase = GraphBuilder.CreateUAV(Layout.ReuseBase);
        Pass.NumBricks = NumReused;
        VBP::AddPagePass<FVoxelPageReuseOffsetsCS>(GraphBuilder, Pass, 1u, 1u, TEXT("Reuse offsets"));
    }
    for (int32 Output = 0; Output < Outputs.Num(); Output++)
    {
        TArray<uint32> Map;
        TArray<FVector4f> Offsets;
        for (int32 k = 0; k < Plan.Mesh.Num(); k++)
        {
            if (Plan.MeshOutput[k] != Output) continue;
            Map.Add((uint32)Plan.MeshRow[k]);
            Map.Add((uint32)Plan.Mesh[k]);
            Offsets.Add(State.Offsets[Plan.Mesh[k]]);
        }
        if (Offsets.Num() == 0) continue;
        FSource& Source = Layout.Sources.AddDefaulted_GetRef();
        Source.Meshed = Outputs[Output];
        Source.Count = Offsets.Num();
        Source.Map = CreateStructuredBuffer(GraphBuilder, TEXT("Voxel.PageBrickMap"), sizeof(uint32), Map.Num(), Map.GetData(), sizeof(uint32) * Map.Num());
        Source.Offsets = CreateStructuredBuffer(GraphBuilder, TEXT("Voxel.PageBrickOffsets"), sizeof(FVector4f), Source.Count, Offsets.GetData(), sizeof(FVector4f) * Source.Count);
        Source.Base = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 3 * Source.Count), TEXT("Voxel.PageBrickBase"));
        FVoxelPageParameters Pass = Template;
        Pass.MesherCounters = GraphBuilder.CreateSRV(Source.Meshed.Counters);
        Pass.BrickMap = GraphBuilder.CreateSRV(Source.Map);
        Pass.BrickBase = GraphBuilder.CreateUAV(Source.Base);
        Pass.NumBricks = (uint32)Source.Count;
        Pass.VertexCap = Source.Meshed.Caps.VertexCap;
        Pass.TriangleCap = Source.Meshed.Caps.TriangleCap;
        VBP::AddPagePass<FVoxelPageOffsetsCS>(GraphBuilder, Pass, 1u, 1u, TEXT("Offsets"));
    }
}

void FVoxelBrickPageProvider::BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesProvider);
    const uint64 StartCycles = FPlatformTime::Cycles64();
    ON_SCOPE_EXIT
    {
        RenderMicros.fetch_add((uint64)(FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - StartCycles) * 1.0e6), std::memory_order_relaxed);
    };
    Calls.fetch_add(1, std::memory_order_relaxed);
    {
        SCOPE_CYCLE_COUNTER(STAT_VoxelPagesBatch);
        Inputs->BuildFrameBatch(GraphBuilder, Context.GetFrame().FrameNumber);
    }
    if (!State.Density.IsValid() || !State.Edits.IsValid()) return;

    // Run when a commit pushed a new state; only a pushed state is meshed (after a refused
    // run the buffers keep the previous one until the game thread commits again).
    if (!bStatePushed || MeshedVersion == State.Version)
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
    MissingNow |= VoxelBrickMesherGPU::AreShadersReady() ? 0u : (uint32)EVoxelPageMissing::MesherShaders;
    MissingNow |= VBP::ArePageShadersReady() ? 0u : (uint32)EVoxelPageMissing::PageShaders;
    MissingNow |= Positions ? 0u : (uint32)EVoxelPageMissing::Positions;
    MissingNow |= Tangents ? 0u : (uint32)EVoxelPageMissing::Tangents;
    MissingNow |= TexCoords ? 0u : (uint32)EVoxelPageMissing::TexCoords;
    MissingNow |= Colors ? 0u : (uint32)EVoxelPageMissing::Colors;
    MissingNow |= Triangles ? 0u : (uint32)EVoxelPageMissing::Triangles;
    MissingNow |= Args ? 0u : (uint32)EVoxelPageMissing::Args;
    if (Context.GetVertexCapacity() <= 0 || Context.GetIndexCapacity() <= 0)
        MissingNow |= (uint32)EVoxelPageMissing::Capacity;
    Missing.fetch_or(MissingNow, std::memory_order_relaxed);
    if (MissingNow & (uint32)EVoxelPageMissing::Required)
    {
        SkippedRuns.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    Runs.fetch_add(1, std::memory_order_relaxed);
    PerfRuns.fetch_add(1, std::memory_order_relaxed);
    INC_DWORD_STAT(STAT_VoxelPagesRemeshRuns);

    // The placement the frame batch recorded for this state, or, for a run the batch didn't
    // plan (pushed before registration, or asked for again), a plan made now, meshed and
    // placed here with its own verdict.
    const uint32 Frame = Context.GetFrame().FrameNumber;
    if (Layout.Frame != Frame || Layout.Graph != &GraphBuilder || Layout.Version != State.Version)
    {
        RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelBrickMesh, "VoxelBrickPage own mesh");
        MakePlan(Plan);
        Plan.Frame = Frame;
        Plan.Graph = &GraphBuilder;
        TArray<VoxelBrickMesherGPU::FOutputs> OwnOutputs;
        TArray<FVoxelMesherBrickGPU> OwnBricks;
        for (int32 k = 0; k < Plan.Mesh.Num(); k++)
        {
            Plan.MeshOutput[k] = OwnBricks.Num() / VoxelBrickMesherGPU::MaxBricks;
            Plan.MeshRow[k] = OwnBricks.Num() % VoxelBrickMesherGPU::MaxBricks;
            OwnBricks.Add(State.Bricks[Plan.Mesh[k]]);
        }
        if (OwnBricks.Num() > 0)
        {
            const VoxelBrickMesherGPU::FInputs MesherInputs = Inputs->Get(GraphBuilder, State.Density, State.Edits);
            for (int32 Start = 0; Start < OwnBricks.Num(); Start += VoxelBrickMesherGPU::MaxBricks)
            {
                const int32 Count = FMath::Min(VoxelBrickMesherGPU::MaxBricks, OwnBricks.Num() - Start);
                OwnOutputs.Add(VoxelBrickMesherGPU::AddMesherPasses(GraphBuilder, *State.Density, MesherInputs, State.Settings,
                    TConstArrayView<FVoxelMesherBrickGPU>(OwnBricks.GetData() + Start, Count)));
            }
        }
        FRDGBufferRef OwnVerdict = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 1), TEXT("Voxel.PageVerdict"));
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(OwnVerdict), 1u);
        AddPlacement(GraphBuilder, Frame, OwnOutputs, OwnVerdict);
    }

    // Streams the driver doesn't expose are written to a scratch word and left alone.
    const uint32 StreamMask = (Tangents ? 1u : 0u) | (TexCoords ? 2u : 0u) | (Colors ? 4u : 0u);
    auto Scratch = [&GraphBuilder]()
        {
            return GraphBuilder.CreateUAV(GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 1), TEXT("Voxel.PageScratch")), PF_R32_UINT);
        };
    Tangents = Tangents ? Tangents : Scratch();
    TexCoords = TexCoords ? TexCoords : Scratch();
    Colors = Colors ? Colors : Scratch();

    // The smaller of what the page asked for and what the driver allocated. The index
    // capacity counts indices.
    const uint32 VertexCapacity = (uint32)FMath::Max(0, FMath::Min(State.VertexCapacity, Context.GetVertexCapacity()));
    const uint32 TriangleCapacity = (uint32)FMath::Max(0, FMath::Min(State.TriangleCapacity, Context.GetIndexCapacity() / 3));

    FVoxelPageParameters Template;
    Template.OutPositions = Positions;
    Template.OutTangents = Tangents;
    Template.OutTexCoords = TexCoords;
    Template.OutColors = Colors;
    Template.OutIndices = Triangles;
    Template.OutArgs = Args;
    Template.VertexCapacity = VertexCapacity;
    Template.TriangleCapacity = TriangleCapacity;
    Template.Color = State.Color;
    Template.StreamMask = StreamMask;
    Template.UVOffset = State.UVOffset;
    Template.UVScale = State.UVScale;
    Template.VertexStride = VoxelBrickMesherGPU::VertexStride;

    {
        RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelBrickMesh, "VoxelBrickPage Mesh");
        Template.PageState = GraphBuilder.CreateUAV(Layout.PageState);
        Template.CommitVerdict = GraphBuilder.CreateSRV(Layout.Verdict);
        const int32 NumMembers = State.Bricks.Num();
        const VoxelBrickMesherGPU::FCaps Caps = VoxelBrickMesherGPU::CapsFor(State.Settings);
        const uint32 NumReused = (uint32)Plan.Reuse.Num() / 2u;
        if (NumReused > 0)
        {
            // Snapshots: the passes below overwrite what they copy from. Streams the driver
            // doesn't expose are not written either: a word stands in.
            auto Snapshot = [&GraphBuilder, &Context](const FRealtimeMeshStreamKey& Key, const TCHAR* Name)
                {
                    FRDGBufferRef Stream = Context.GetStreamBuffer(Key);
                    FRDGBufferRef Copy = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32),
                        Stream ? FMath::Max<uint32>(1u, Stream->Desc.GetSize() / sizeof(uint32)) : 1u), Name);
                    if (Stream)
                        AddCopyBufferPass(GraphBuilder, Copy, Stream);
                    else
                        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Copy), 0u);
                    return GraphBuilder.CreateSRV(Copy);
                };
            FVoxelPageParameters Pass = Template;
            Pass.OldPositions = Snapshot(FRealtimeMeshStreams::Position, TEXT("Voxel.PageOldPositions"));
            Pass.OldTangents = Snapshot(FRealtimeMeshStreams::Tangents, TEXT("Voxel.PageOldTangents"));
            Pass.OldTexCoords = Snapshot(FRealtimeMeshStreams::TexCoords, TEXT("Voxel.PageOldTexCoords"));
            Pass.OldIndices = Snapshot(FRealtimeMeshStreams::Triangles, TEXT("Voxel.PageOldIndices"));
            Pass.OldTable = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Table));
            Pass.BrickMap = GraphBuilder.CreateSRV(Layout.ReuseMap);
            Pass.BrickBase = GraphBuilder.CreateUAV(Layout.ReuseBase);
            Pass.NumBricks = NumReused;
            Pass.VertexCap = Caps.VertexCap;
            Pass.TriangleCap = Caps.TriangleCap;
            VBP::AddPagePass<FVoxelPageReuseVerticesCS>(GraphBuilder, Pass, NumReused * Caps.VertexCap, 64u, TEXT("Reuse vertices"));
            VBP::AddPagePass<FVoxelPageReuseTrianglesCS>(GraphBuilder, Pass, NumReused * Caps.TriangleCap, 64u, TEXT("Reuse triangles"));
        }
        for (const FSource& Source : Layout.Sources)
        {
            FVoxelPageParameters Pass = Template;
            Pass.MesherCounters = GraphBuilder.CreateSRV(Source.Meshed.Counters);
            Pass.MesherVertices = GraphBuilder.CreateSRV(Source.Meshed.Vertices);
            Pass.MesherIndices = GraphBuilder.CreateSRV(Source.Meshed.Indices);
            Pass.BrickOffsets = GraphBuilder.CreateSRV(Source.Offsets);
            Pass.BrickMap = GraphBuilder.CreateSRV(Source.Map);
            Pass.BrickBase = GraphBuilder.CreateUAV(Source.Base);
            Pass.NumBricks = (uint32)Source.Count;
            Pass.VertexCap = Source.Meshed.Caps.VertexCap;
            Pass.TriangleCap = Source.Meshed.Caps.TriangleCap;
            VBP::AddPagePass<FVoxelPageVerticesCS>(GraphBuilder, Pass, (uint32)Source.Count * Source.Meshed.Caps.VertexCap, 64u, TEXT("Vertices"));
            VBP::AddPagePass<FVoxelPageTrianglesCS>(GraphBuilder, Pass, (uint32)Source.Count * Source.Meshed.Caps.TriangleCap, 64u, TEXT("Triangles"));
        }
        VBP::AddPagePass<FVoxelPageArgsCS>(GraphBuilder, Template, 1u, 1u, TEXT("Draw arguments"));

        if (Readbacks.Num() >= VBP::MaxPendingReadbacks)
            Readbacks.RemoveAt(0);
        FPendingReadback& Pending = Readbacks.AddDefaulted_GetRef();
        Pending.Readback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.PageStateReadback"));
        Pending.Version = State.Version;
        Pending.Bricks = NumMembers;
        AddEnqueueCopyPass(GraphBuilder, Pending.Readback.Get(), Layout.PageState, VBP::PageStateWords * sizeof(uint32));
        if (NumMembers > 0)
        {
            Pending.TableReadback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.PageTableReadback"));
            Pending.Keys = State.Keys;
            Pending.Stamps = State.Stamps;
            AddEnqueueCopyPass(GraphBuilder, Pending.TableReadback.Get(), Layout.NewTable, 4 * NumMembers * sizeof(uint32));
        }

        // The new table, tentatively: a refused run (read back) restores the previous one,
        // which is what the buffers hold if the run before was drawn.
        bPrevValid = TentativeVersion == 0;
        PrevTable = Table;
        PrevTableRows = TableRows;
        PrevTableStamps = TableStamps;
        PrevMeshedVersion = MeshedVersion;
        TentativeVersion = State.Version;
        Table = GraphBuilder.ConvertToExternalBuffer(Layout.NewTable);
        TableRows.Reset();
        for (int32 i = 0; i < NumMembers; i++)
            TableRows.Add(State.Keys[i], i);
        TableStamps = State.Stamps;
        BricksMeshed.fetch_add((uint32)Plan.Mesh.Num(), std::memory_order_relaxed);
        BricksReused.fetch_add(NumReused, std::memory_order_relaxed);
        INC_DWORD_STAT_BY(STAT_VoxelPagesBricksMeshed, Plan.Mesh.Num());
        INC_DWORD_STAT_BY(STAT_VoxelPagesBricksReused, NumReused);
        Plan = FPlan();
        Layout = FLayout();
        MeshedVersion = State.Version;
        bStatePushed = false;
    }

    ShownVersion.store(State.Version, std::memory_order_release);
}

FVoxelBrickPages::FVoxelBrickPages()
    : GPUInputs(MakeShared<FVoxelBrickPageGPUInputs, ESPMode::ThreadSafe>())
{
}

FVoxelBrickPages::~FVoxelBrickPages()
{
    Reset();
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
    Density.Reset();
    Edits.Reset();
}

void FVoxelBrickPages::Reset()
{
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        DestroyInstance(Pair.Value.Current);
        DestroyInstance(Pair.Value.Next);
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
}

FVoxelBrickKey FVoxelBrickPages::PageOf(const FVoxelBrickKey& Key)
{
    FVoxelBrickKey Page;
    Page.Level = Key.Level;
    Page.Coord = FInt64Vector(VBP::FloorShift(Key.Coord.X, VBP::PageShift), VBP::FloorShift(Key.Coord.Y, VBP::PageShift), VBP::FloorShift(Key.Coord.Z, VBP::PageShift));
    return Page;
}

FVoxelBrickPages::FInstance FVoxelBrickPages::CreateInstance(const FVoxelBrickKey& PageKey, int32 VertexCapacity, int32 TriangleCapacity)
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

    FVoxelBrickGrid Grid;
    Grid.Origin = Settings.Origin;
    Grid.CellSize = Settings.FinestCell * (double)(1LL << PageKey.Level);
    Grid.CellsPerBrick = Settings.CellsPerBrick;
    const FVector PageOrigin = Grid.BrickMin(PageKey.Coord * (1LL << VBP::PageShift));

    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetupMaterialSlot(0, "Surface Material");
    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->bHiddenInSceneCapture = true;
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->RegisterComponent();
    Component->SetMaterial(0, Settings.Material.Get());
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetRelativeLocation(PageOrigin);
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetRealtimeMesh(Mesh);
    Component->SetRenderCustomDepth(true);
    Component->SetVisibleInRayTracing(false);
    // For the morph in the material (VoxelMorph.ush): the page's level, and where level 0's
    // band ends in world units. A buffer set is only ever reused for pages of its level.
    Component->SetCustomPrimitiveDataFloat(0, (float)PageKey.Level);
    Component->SetCustomPrimitiveDataFloat(1, (float)(VBP::MorphBandEnd0(Settings) * Root->GetComponentScale().X));

    // The uploaded streams only size the buffers; the provider writes them. Two corner
    // positions give the page its bounds: bricks reach a few cells past their box, and the
    // morph (VoxelMorph.ush) moves vertices toward coarser cells up to 8 cells across.
    FRealtimeMeshStreamSet Streams;
    FRealtimeMeshStream& Positions = Streams.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
    Positions.SetNumZeroed(VertexCapacity);
    const double Margin = VBP::BoundsMarginCells * Grid.CellSize;
    const double Extent = (double)(1LL << VBP::PageShift) * Grid.BrickSize();
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
    Instance.Provider = MakeShared<FVoxelBrickPageProvider, ESPMode::ThreadSafe>(GPUInputs);
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

bool FVoxelBrickPages::IsReady(const FInstance& Instance)
{
    return Instance.IsValid() && Instance.Handle != 0 && GFrameCounter > Instance.RegisteredFrame && GFrameCounter >= Instance.ReadyFrame;
}

TArray<FVoxelBrickPages::FInstance>* FVoxelBrickPages::FindPooled(int32 Level, FIntPoint Capacity)
{
    // Smallest total first.
    for (int32 Extra = 0; Extra <= 2 * VBP::PoolSlack; Extra++)
    {
        for (int32 DV = FMath::Max(0, Extra - VBP::PoolSlack); DV <= FMath::Min(Extra, VBP::PoolSlack); DV++)
        {
            TArray<FInstance>* Free = Pool.Find(VBP::PoolKey(Level, Capacity.X << DV, Capacity.Y << (Extra - DV)));
            if (Free && Free->Num() > 0) return Free;
        }
    }
    return nullptr;
}

FVoxelBrickPages::FInstance FVoxelBrickPages::AcquireInstance(const FVoxelBrickKey& PageKey, int32 VertexCapacity, int32 TriangleCapacity)
{
    while (TArray<FInstance>* Free = FindPooled(PageKey.Level, FIntPoint(VertexCapacity, TriangleCapacity)))
    {
        FInstance Instance = Free->Pop();
        NumPooled--;
        URealtimeMeshComponent* Component = Instance.Component.Get();
        if (!Component || !Instance.Mesh.IsValid())
        {
            DestroyInstance(Instance);
            continue;
        }
        // Moved now, filled from the next frame on, so the move reaches the render thread no
        // later than the new contents.
        FVoxelBrickGrid Grid;
        Grid.Origin = Settings.Origin;
        Grid.CellSize = Settings.FinestCell * (double)(1LL << PageKey.Level);
        Grid.CellsPerBrick = Settings.CellsPerBrick;
        Component->SetRelativeLocation(Grid.BrickMin(PageKey.Coord * (1LL << VBP::PageShift)));
        Instance.ReadyFrame = GFrameCounter + 1;
        Perf.Recycled++;
        INC_DWORD_STAT(STAT_VoxelPagesRecycled);
        return Instance;
    }
    // A new one is registered with empty contents, so it is ready when a commit needs it.
    FInstance Instance = CreateInstance(PageKey, VertexCapacity, TriangleCapacity);
    if (Instance.IsValid())
        Push(PageKey, TMap<FVoxelBrickKey, FMember>(), NextVersion++, Instance);
    return Instance;
}

void FVoxelBrickPages::ReleaseInstance(const FVoxelBrickKey& PageKey, FInstance& Instance)
{
    if (!Instance.IsValid()) return;
    if (Instance.Handle == 0 || !Instance.Component.IsValid())
    {
        DestroyInstance(Instance);
        return;
    }
    // Emptied (zero draw arguments) in this frame's commit; pooled once that is read back.
    FReleasing Entry;
    Entry.PageKey = PageKey;
    Entry.Version = NextVersion++;
    Entry.Frame = GFrameCounter;
    Push(PageKey, TMap<FVoxelBrickKey, FMember>(), Entry.Version, Instance);
    MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
    Instance.LastKickFrame = GFrameCounter;
    Entry.Instance = MoveTemp(Instance);
    Instance = FInstance();
    Releasing.Add(MoveTemp(Entry));
}

void FVoxelBrickPages::PoolInstance(const FVoxelBrickKey& PageKey, FInstance& Instance)
{
    if (!Instance.IsValid()) return;
    if (!Instance.Component.IsValid() || NumPooled >= VBP::MaxPooled)
    {
        DestroyInstance(Instance);
        return;
    }
    Pool.FindOrAdd(VBP::PoolKey(PageKey.Level, Instance.VertexCapacity, Instance.TriangleCapacity)).Add(MoveTemp(Instance));
    NumPooled++;
    Instance = FInstance();
}

bool FVoxelBrickPages::IsSettling() const
{
    for (const TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        if (Pair.Value.bVerifying || Pair.Value.bRecommit) return true;
    }
    for (const FReleasing& Entry : Releasing)
    {
        if (Entry.bRefused) return true;
    }
    return false;
}

void FVoxelBrickPages::Recommit()
{
    // Everything the refused frame drew must be read back first, so the pages it refused are
    // committed again together.
    bool bAny = false;
    for (const TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        if (Pair.Value.bVerifying) return;
        bAny |= Pair.Value.bRecommit;
    }
    for (const FReleasing& Entry : Releasing)
        bAny |= Entry.bRefused;
    if (!bAny) return;

    bool bReady = true;
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        if (Pair.Value.bRecommit)
            bReady &= EnsureCapacity(Pair.Key, Pair.Value, Pair.Value.RecommitCapacity, CreatesThisFrame);
    }
    if (!bReady) return;
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        if (!Pair.Value.bRecommit) continue;
        Pair.Value.bRecommit = false;
        Publish(Pair.Key, Pair.Value);
    }
    for (FReleasing& Entry : Releasing)
    {
        if (!Entry.bRefused) continue;
        Entry.bRefused = false;
        Entry.Version = NextVersion++;
        Entry.Frame = GFrameCounter;
        Push(Entry.PageKey, TMap<FVoxelBrickKey, FMember>(), Entry.Version, Entry.Instance);
        MarkRealtimeMeshComputeProviderDirty(Entry.Instance.Handle);
        Entry.Instance.LastKickFrame = GFrameCounter;
    }
    Perf.Recommits++;
}

FIntPoint FVoxelBrickPages::RoundCapacity(double Vertices, double Triangles)
{
    const int32 V = (int32)FMath::RoundUpToPowerOfTwo((uint32)FMath::Clamp(Vertices * VBP::CapacityMargin, (double)VBP::MinVertices, (double)VBP::MaxVertices));
    const int32 T = (int32)FMath::RoundUpToPowerOfTwo((uint32)FMath::Clamp(Triangles * VBP::CapacityMargin, (double)VBP::MinTriangles, (double)VBP::MaxTriangles));
    return FIntPoint(V, T);
}

int32 FVoxelBrickPages::CreateBudget() const
{
    // The longer a commit waits for buffers, the more may be created per frame.
    const int32 Base = FMath::Max(1, CVarVoxelBrickPageCreates.GetValueOnGameThread());
    const int32 Waited = WaitingSince != 0 ? (int32)FMath::Min<uint64>(GFrameCounter - WaitingSince, 64) : 0;
    return Base * FMath::Min(1 + Waited / VBP::CreateRampFrames, VBP::MaxCreateRamp);
}

FIntPoint FVoxelBrickPages::EstimateCapacity(const FPage* Page, const FVoxelBrickKey& PageKey, TConstArrayView<const FVoxelBrickPageChange*> PageChanges) const
{
    // Per member: its read-back counts (a changed brick keeps its previous ones as the
    // estimate), else the level's average surface brick, else a busy surface brick.
    const double Square = (double)Settings.CellsPerBrick * Settings.CellsPerBrick;
    FVector2D Default = LevelAverages.FindRef(PageKey.Level) * VBP::UnknownBrickWeight;
    if (Default.X <= 0.0)
        Default = FVector2D(2.0 * Square, 4.0 * Square);
    auto Add = [&Default](const FMember* Member, double& V, double& T)
        {
            V += Member && Member->Vertices >= 0 ? (double)Member->Vertices : Default.X;
            T += Member && Member->Triangles >= 0 ? (double)Member->Triangles : Default.Y;
        };

    double Vertices = 0.0, Triangles = 0.0;
    TSet<FVoxelBrickKey> Changed;
    for (const FVoxelBrickPageChange* Change : PageChanges)
    {
        Changed.Add(Change->Key);
        if (!Change->bRemove)
            Add(Page ? Page->Members.Find(Change->Key) : nullptr, Vertices, Triangles);
    }
    if (Page)
    {
        for (const TPair<FVoxelBrickKey, FMember>& Pair : Page->Members)
        {
            if (!Changed.Contains(Pair.Key))
                Add(&Pair.Value, Vertices, Triangles);
        }
    }
    return RoundCapacity(Vertices, Triangles);
}

bool FVoxelBrickPages::EnsureCapacity(const FVoxelBrickKey& PageKey, FPage& Page, FIntPoint Capacity, int32& InOutCreates)
{
    Page.NextUsedFrame = GFrameCounter;
    auto Fits = [&Capacity](const FInstance& Instance)
        {
            return Instance.IsValid() && Instance.VertexCapacity >= Capacity.X && Instance.TriangleCapacity >= Capacity.Y;
        };
    auto CanAcquire = [this, &PageKey, &Capacity, &InOutCreates]()
        {
            return FindPooled(PageKey.Level, Capacity) != nullptr || InOutCreates < CreateBudget();
        };
    auto Acquire = [this, &PageKey, &Capacity, &InOutCreates]()
        {
            const int32 CreatedBefore = Perf.Created;
            FInstance Instance = AcquireInstance(PageKey, Capacity.X, Capacity.Y);
            InOutCreates += Perf.Created - CreatedBefore;
            return Instance;
        };

    if (Fits(Page.Current))
        return IsReady(Page.Current);
    if (!Page.Current.IsValid())
    {
        if (!CanAcquire()) return false;
        Page.Current = Acquire();
        return IsReady(Page.Current);
    }
    // Current is too small: a larger replacement takes the contents at the commit.
    if (!Fits(Page.Next))
    {
        if (!CanAcquire()) return false;
        ReleaseInstance(PageKey, Page.Next);
        Page.Next = Acquire();
    }
    return IsReady(Page.Next);
}

void FVoxelBrickPages::Publish(const FVoxelBrickKey& PageKey, FPage& Page)
{
    // Into Next when Current can't hold the members (or is missing), Current emptied in the
    // same frame.
    const FIntPoint Capacity = EstimateCapacity(&Page, PageKey, {});
    const bool bCurrentFits = Page.Current.IsValid() && Page.Current.VertexCapacity >= Capacity.X && Page.Current.TriangleCapacity >= Capacity.Y;
    const bool bUseNext = IsReady(Page.Next) && (!bCurrentFits || !IsReady(Page.Current));
    FInstance& Target = bUseNext ? Page.Next : Page.Current;
    if (!Target.IsValid()) return;

    Page.Published = NextVersion++;
    Page.PublishedFrame = GFrameCounter;
    Page.bVerifying = true;
    Push(PageKey, Page.Members, Page.Published, Target);
    if (Target.Handle != 0)
    {
        MarkRealtimeMeshComputeProviderDirty(Target.Handle);
        Target.LastKickFrame = GFrameCounter;
    }
    if (bUseNext)
    {
        ReleaseInstance(PageKey, Page.Current);
        Page.Current = MoveTemp(Page.Next);
        Page.Next = FInstance();
    }
}

bool FVoxelBrickPages::PreparePages(TConstArrayView<FVoxelBrickPageChange> Changes)
{
    if (CreatesFrame != GFrameCounter)
    {
        CreatesFrame = GFrameCounter;
        CreatesThisFrame = 0;
    }
    TMap<FVoxelBrickKey, TArray<const FVoxelBrickPageChange*>> ByPage;
    for (const FVoxelBrickPageChange& Change : Changes)
        ByPage.FindOrAdd(PageOf(Change.Key)).Add(&Change);

    bool bReady = true;
    for (const TPair<FVoxelBrickKey, TArray<const FVoxelBrickPageChange*>>& Pair : ByPage)
    {
        const FPage* Existing = Pages.Find(Pair.Key);
        int32 After = Existing ? Existing->Members.Num() : 0;
        for (const FVoxelBrickPageChange* Change : Pair.Value)
        {
            const bool bHeld = Existing && Existing->Members.Contains(Change->Key);
            After += Change->bRemove ? (bHeld ? -1 : 0) : (bHeld ? 0 : 1);
        }
        if (After <= 0) continue;   // emptied at the commit: always ready
        const FIntPoint Capacity = EstimateCapacity(Existing, Pair.Key, Pair.Value);
        FPage& Page = Pages.FindOrAdd(Pair.Key);
        bReady &= EnsureCapacity(Pair.Key, Page, Capacity, CreatesThisFrame);
    }
    WaitingSince = bReady ? 0 : (WaitingSince != 0 ? WaitingSince : GFrameCounter);
    return bReady;
}

bool FVoxelBrickPages::PrepareCommit(TConstArrayView<FVoxelBrickPageChange> Changes)
{
    const bool bReady = PreparePages(Changes);
    const bool bSettling = IsSettling();
    if (!bReady || bSettling)
        Perf.PrepareWaits++;
    Perf.VerifyWaits += bSettling ? 1 : 0;
    return bReady && !bSettling;
}

void FVoxelBrickPages::Commit(TConstArrayView<FVoxelBrickPageChange> Changes)
{
    TSet<FVoxelBrickKey> Touched;
    for (const FVoxelBrickPageChange& Change : Changes)
    {
        const FVoxelBrickKey PageKey = PageOf(Change.Key);
        Touched.Add(PageKey);
        if (Change.bRemove)
        {
            if (FPage* Page = Pages.Find(PageKey))
                Page->Members.Remove(Change.Key);
            continue;
        }
        FMember& Member = Pages.FindOrAdd(PageKey).Members.FindOrAdd(Change.Key);
        FMemory::Memcpy(Member.Levels, Change.Levels, sizeof(Member.Levels));
        Member.Meshed = Change.Meshed;
        Member.Stamp = ++NextStamp;
    }
    for (const FVoxelBrickKey& PageKey : Touched)
    {
        FPage* Page = Pages.Find(PageKey);
        if (!Page) continue;
        if (Page->Members.Num() == 0)
        {
            ReleaseInstance(PageKey, Page->Current);
            ReleaseInstance(PageKey, Page->Next);
            Pages.Remove(PageKey);
            continue;
        }
        Publish(PageKey, *Page);
    }
    Perf.Commits++;
    Perf.CommitChanges += Changes.Num();
    Perf.CommitPages += Touched.Num();
    Perf.MaxCommitChanges = FMath::Max(Perf.MaxCommitChanges, Changes.Num());
}

void FVoxelBrickPages::Push(const FVoxelBrickKey& PageKey, const TMap<FVoxelBrickKey, FMember>& Members, uint64 Version, FInstance& Instance) const
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelPagesPush);
    if (!Instance.IsValid()) return;

    FVoxelBrickGrid Grid;
    Grid.Origin = Settings.Origin;
    Grid.CellSize = Settings.FinestCell * (double)(1LL << PageKey.Level);
    Grid.CellsPerBrick = Settings.CellsPerBrick;
    const FInt64Vector FirstBrick = PageKey.Coord * (1LL << VBP::PageShift);
    const FVector PageOrigin = Grid.BrickMin(FirstBrick);

    FVoxelBrickPageProvider::FState State;
    State.Density = Density;
    State.Edits = Edits;
    State.Settings = MesherSettings;
    State.VertexCapacity = Instance.VertexCapacity;
    State.TriangleCapacity = Instance.TriangleCapacity;
    State.Version = Version;
    State.bFull = Instance.PushedVersion == 0;
    State.UVScale = (float)Settings.TriplanarUVScale;
    const FVector UVOrigin = PageOrigin * Settings.TriplanarUVScale;
    State.UVOffset = FVector3f((float)FMath::Frac(UVOrigin.X), (float)FMath::Frac(UVOrigin.Y), (float)FMath::Frac(UVOrigin.Z));
    State.Color = (CVarVoxelDebugChunkColor.GetValueOnGameThread() > 0 ? FVoxelBrickTerrain::LevelColor(PageKey.Level) : FColor::White).DWColor();
    State.Bricks.Reserve(Members.Num());
    State.Offsets.Reserve(Members.Num());
    State.Keys.Reserve(Members.Num());
    State.Stamps.Reserve(Members.Num());
    for (const TPair<FVoxelBrickKey, FMember>& Pair : Members)
    {
        FVoxelMesherBrickGPU Brick;
        if (!VoxelBrickMesherGPU::MakeBrick(Pair.Key, Pair.Value.Levels, Pair.Value.Meshed, MesherSettings, Brick))
            continue;
        State.Bricks.Add(Brick);
        const FInt64Vector Local = Pair.Key.Coord - FirstBrick;
        State.Offsets.Add(FVector4f((float)(Local.X * Grid.BrickSize()), (float)(Local.Y * Grid.BrickSize()), (float)(Local.Z * Grid.BrickSize()), 0.0f));
        State.Keys.Add(Pair.Key);
        State.Stamps.Add(Pair.Value.Stamp);
    }

    // A registered provider runs in the frame it is marked, so its bricks join that frame's
    // batch; one not yet registered meshes its own on its first run.
    const bool bBatch = Instance.Handle != 0;
    ENQUEUE_RENDER_COMMAND(VoxelBrickPagePush)([Provider = Instance.Provider, State = MoveTemp(State), bBatch](FRHICommandListImmediate&) mutable
        {
            Provider->SetState_RenderThread(MoveTemp(State));
            if (bBatch)
                FVoxelBrickPageProvider::QueueForBatch(Provider);
        });
    Instance.PushedVersion = Version;
}

void FVoxelBrickPages::Advance(FInstance& Instance) const
{
    if (!Instance.IsValid()) return;
    URealtimeMeshSimple* Mesh = Instance.Mesh.Get();
    if (!Mesh) return;

    if (Instance.Handle == 0)
    {
        // Registered once the buffers exist and the component has a proxy (two frames, as
        // FMeshChunk::ApplyStream); the driver runs an OnDemand provider when registered.
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
        // On demand: a commit marks a provider when its page changes, so idle pages cost the
        // driver nothing.
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
    if (!Density.IsValid() || !Edits.IsValid()) return;
    if (CreatesFrame != GFrameCounter)
    {
        CreatesFrame = GFrameCounter;
        CreatesThisFrame = 0;
    }

    TArray<TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>> Poll;
    auto CollectCounters = [this](const FInstance& Instance)
        {
            if (!Instance.IsValid()) return;
            FVoxelBrickPageProvider& Provider = *Instance.Provider;
            Perf.Calls += Provider.Calls.exchange(0, std::memory_order_relaxed);
            Perf.IdleCalls += Provider.IdleCalls.exchange(0, std::memory_order_relaxed);
            Perf.Runs += Provider.PerfRuns.exchange(0, std::memory_order_relaxed);
            Perf.BricksMeshed += Provider.BricksMeshed.exchange(0, std::memory_order_relaxed);
            Perf.BricksReused += Provider.BricksReused.exchange(0, std::memory_order_relaxed);
            Perf.RenderMicros += Provider.RenderMicros.exchange(0, std::memory_order_relaxed);
        };

    // Instances being released: pooled once the commit that emptied them is read back as
    // drawn; a refused one still draws its page and is emptied again with the re-commit.
    for (int32 r = 0; r < Releasing.Num(); r++)
    {
        FReleasing& Entry = Releasing[r];
        Advance(Entry.Instance);
        CollectCounters(Entry.Instance);
        bool bDone = !Entry.Instance.IsValid();
        FVoxelBrickPageStats Read;
        if (!bDone && Entry.Instance.Provider->TakeStats(Read) && Read.Version >= Entry.Version)
        {
            bDone = Read.bCommitted;
            Entry.bRefused = !Read.bCommitted;
        }
        if (!bDone && !Entry.bRefused && GFrameCounter > Entry.Frame + VBP::VerifyTimeoutFrames)
            bDone = true;
        if (bDone)
        {
            PoolInstance(Entry.PageKey, Entry.Instance);
            Releasing.RemoveAtSwap(r--);
            continue;
        }
        if (!Entry.bRefused)
            Poll.Add(Entry.Instance.Provider);
    }

    TArray<FVoxelBrickKey> Emptied;
    for (TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        const FVoxelBrickKey& PageKey = Pair.Key;
        FPage& Page = Pair.Value;
        Advance(Page.Current);
        Advance(Page.Next);
        CollectCounters(Page.Current);
        CollectCounters(Page.Next);

        // Read-back counters: each member's counts (for the capacity estimates), and the
        // verdict of the published version. A refused page is committed again with every page
        // refused with it, in buffers that hold what the GPU counted.
        FVoxelBrickPageStats Read;
        if (Page.Current.IsValid() && Page.Current.Provider->TakeStats(Read))
        {
            for (int32 m = 0; m < Read.Keys.Num(); m++)
            {
                FMember* Member = Page.Members.Find(Read.Keys[m]);
                if (!Member || Member->Stamp != Read.Stamps[m]) continue;
                const int32 V = (int32)Read.Counts[2 * m];
                const int32 T = (int32)Read.Counts[2 * m + 1];
                const bool bUnknown = V == 0 && Read.BricksDropped > 0;
                Member->Vertices = bUnknown ? -1 : V;
                Member->Triangles = bUnknown ? -1 : T;
                if (V > 0)
                {
                    FVector2D& Average = LevelAverages.FindOrAdd(PageKey.Level);
                    const FVector2D Counts((double)V, (double)T);
                    Average = Average.X <= 0.0 ? Counts : Average + (Counts - Average) * VBP::AverageBlend;
                }
            }
            if (Page.bVerifying && Read.Version >= Page.Published)
            {
                Page.bVerifying = false;
                if (!Read.bCommitted)
                {
                    const FIntPoint Estimate = EstimateCapacity(&Page, PageKey, {});
                    const FIntPoint Counted = RoundCapacity((double)Read.VerticesNeeded, (double)Read.TrianglesNeeded);
                    Page.RecommitCapacity = FIntPoint(FMath::Max(Estimate.X, Counted.X), FMath::Max(Estimate.Y, Counted.Y));
                    Page.bRecommit = true;
                    Perf.Refused++;
                }
            }
            Page.Stats = MoveTemp(Read);
        }
        if (Page.bVerifying && GFrameCounter > Page.PublishedFrame + VBP::VerifyTimeoutFrames)
            Page.bVerifying = false;

        // Pages a commit prepared but never filled, and replacements no longer wanted.
        if (Page.Members.Num() == 0 && !Page.bVerifying && !Page.bRecommit && GFrameCounter > Page.NextUsedFrame + VBP::UnusedFrames)
        {
            ReleaseInstance(PageKey, Page.Current);
            ReleaseInstance(PageKey, Page.Next);
            Emptied.Add(PageKey);
            continue;
        }
        if (Page.Next.IsValid() && !Page.bRecommit && GFrameCounter > Page.NextUsedFrame + VBP::UnusedFrames)
            ReleaseInstance(PageKey, Page.Next);

        if (Page.Current.IsValid())
        {
            const uint32 MissingBits = Page.Current.Provider->Missing.load(std::memory_order_relaxed);
            if (MissingBits != 0 && (MissingBits & ~WarnedMissing) != 0)
            {
                UE_LOG(LogTemp, Warning, TEXT("[VoxelBricks] A brick page's compute run is missing: %s%s."), *DescribeMissing(MissingBits),
                    (MissingBits & (uint32)EVoxelPageMissing::Required) ? TEXT(" (run skipped, retried)") : TEXT(" (written elsewhere, run continues)"));
                WarnedMissing |= MissingBits;
            }
            if (Page.bVerifying)
                Poll.Add(Page.Current.Provider);
        }
    }
    for (const FVoxelBrickKey& Key : Emptied)
        Pages.Remove(Key);
    Recommit();

    if (Poll.Num() > 0)
    {
        ENQUEUE_RENDER_COMMAND(VoxelBrickPagePoll)([Poll = MoveTemp(Poll)](FRHICommandListImmediate&)
            {
                for (const TSharedPtr<FVoxelBrickPageProvider, ESPMode::ThreadSafe>& Provider : Poll)
                    Provider->PollStats_RenderThread();
            });
    }
    SET_DWORD_STAT(STAT_VoxelPagesCount, Pages.Num());
    SET_DWORD_STAT(STAT_VoxelPagesPooled, NumPooled);
}

FString FVoxelBrickPages::TakePerf()
{
    int32 Instances = 0;
    int64 Bytes = 0, Triangles = 0;
    for (const TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        for (const FInstance* Instance : { &Pair.Value.Current, &Pair.Value.Next })
        {
            if (!Instance->IsValid()) continue;
            Instances++;
            Bytes += (int64)Instance->VertexCapacity * VBP::BytesPerVertex + (int64)Instance->TriangleCapacity * VBP::BytesPerTriangle;
        }
        Triangles += Pair.Value.Stats.TrianglesWritten;
    }
    const int64 Batched = GPUInputs->BatchBricks.exchange(0, std::memory_order_relaxed);
    const double PerFrame = 1.0 / FMath::Max(1, Perf.Frames);
    const FString Out = FString::Printf(TEXT("Pages: %d pages, %d buffer sets (+%d pooled), %.1f MB, %lld triangles. Commits: %d (%d changes and %d pages on average, %d changes at most), %d frames waiting (%d of them for a verdict), %d pages refused by the GPU, %d re-commits. Per frame (average): %.1f provider calls of which %.1f idle, %.2f re-mesh runs, %.1f bricks meshed (%.1f in frame batches; %.0f lattice samples), %.1f copied unchanged, %.2f ms render thread in providers; buffer sets %.2f created (%.2f ms each, %.2f ms max), %.2f recycled. Flush %.2f ms average, %.2f ms max."),
        Pages.Num(), Instances, NumPooled, Bytes / (1024.0 * 1024.0), Triangles,
        Perf.Commits, Perf.Commits > 0 ? Perf.CommitChanges / Perf.Commits : 0, Perf.Commits > 0 ? Perf.CommitPages / Perf.Commits : 0, Perf.MaxCommitChanges,
        Perf.PrepareWaits, Perf.VerifyWaits, Perf.Refused, Perf.Recommits,
        Perf.Calls * PerFrame, Perf.IdleCalls * PerFrame, Perf.Runs * PerFrame, Perf.BricksMeshed * PerFrame, Batched * PerFrame,
        Perf.BricksMeshed * PerFrame * FMath::Cube((double)VoxelBrickMesherGPU::CellsFor(MesherSettings) + 1.0),
        Perf.BricksReused * PerFrame, Perf.RenderMicros * PerFrame / 1000.0,
        Perf.Created * PerFrame, Perf.Created > 0 ? Perf.CreateMs / Perf.Created : 0.0, Perf.MaxCreateMs, Perf.Recycled * PerFrame,
        Perf.FlushMs * PerFrame, Perf.MaxFlushMs);
    Perf = FPerf();
    return Out;
}

FString FVoxelBrickPages::DescribeMissing(uint32 Bits)
{
    static const TCHAR* Names[] = { TEXT("mesher shaders"), TEXT("page shaders"), TEXT("Position stream"), TEXT("Tangents stream"),
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
    int32 Instances = 0, Members = 0, Waiting = 0, Dropped = 0, Lagging = 0, Repairing = 0;
    uint64 MaxLag = 0;
    uint32 Runs = 0, Skipped = 0, MissingBits = 0;
    int64 Bytes = 0, Triangles = 0;
    uint32 Errors = 0;
    for (const TPair<FVoxelBrickKey, FPage>& Pair : Pages)
    {
        const FPage& Page = Pair.Value;
        Members += Page.Members.Num();
        for (const FInstance* Instance : { &Page.Current, &Page.Next })
        {
            if (!Instance->IsValid()) continue;
            Instances++;
            Bytes += (int64)Instance->VertexCapacity * VBP::BytesPerVertex + (int64)Instance->TriangleCapacity * VBP::BytesPerTriangle;
            Waiting += Instance->Handle == 0 ? 1 : 0;
            Runs += Instance->Provider->Runs.load(std::memory_order_relaxed);
            Skipped += Instance->Provider->SkippedRuns.load(std::memory_order_relaxed);
            MissingBits |= Instance->Provider->Missing.load(std::memory_order_relaxed);
        }
        Triangles += Page.Stats.TrianglesWritten;
        if (Page.Current.IsValid() && Page.Current.Provider->ShownVersion.load(std::memory_order_relaxed) < Page.Published)
        {
            Lagging++;
            MaxLag = FMath::Max(MaxLag, GFrameCounter - Page.PublishedFrame);
        }
        Dropped += (int32)Page.Stats.BricksDropped;
        Errors |= Page.Stats.Errors;
        Repairing += Page.bRecommit ? 1 : 0;
    }
    return FString::Printf(TEXT("GPU pages: %d (%d bricks, %d buffer sets, %d not yet registered, %d not yet drawing their newest version, the oldest published %llu frames ago), %u runs, %u skipped%s%s, %.1f MB of buffers, %lld triangles drawn, %d bricks that didn't fit at the last run (%d pages waiting to be committed again), %d buffer sets being released, mesher errors 0x%x"),
        Pages.Num(), Members, Instances, Waiting, Lagging, MaxLag, Runs, Skipped, MissingBits ? TEXT(", missing: ") : TEXT(""), *DescribeMissing(MissingBits),
        Bytes / (1024.0 * 1024.0), Triangles, Dropped, Repairing, Releasing.Num(), Errors);
}
