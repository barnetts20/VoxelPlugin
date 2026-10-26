#include "VoxelLandSolve.h"
#include "VoxelGPUCommon.h"
#include "VoxelPlugin.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RHIGPUReadback.h"
#include "Math/Float16.h"

DECLARE_GPU_STAT(VoxelLandSolve);

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelLandSolveParameters, )
    // Grid and the plate model's fields.
    SHADER_PARAMETER(uint32, SolveN)
    SHADER_PARAMETER(uint32, SolvePad)
    SHADER_PARAMETER(float, SolveRadiusM)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float2>, SolveTan)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, BakeHeight)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, BakeUp)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, BakeCrust)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, BakeAttr)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, BakeForm)
    // The model.
    SHADER_PARAMETER(float, SolvePlainsUplift)
    SHADER_PARAMETER(float, SolveRampWidth)
    SHADER_PARAMETER(float, SolveUpliftTime)
    SHADER_PARAMETER(float, SolveKdt)
    SHADER_PARAMETER(float, SolveDt)
    SHADER_PARAMETER(float, SolveEps)
    SHADER_PARAMETER(uint32, SolveSeed)
    SHADER_PARAMETER(float, SolveFactor)
    SHADER_PARAMETER(float, SolveReliefKnee)
    SHADER_PARAMETER(float, SolveReliefTop)
    SHADER_PARAMETER(float, SolveMinLakeDepth)
    SHADER_PARAMETER(float, SolveMinLakeArea)
    SHADER_PARAMETER(uint32, SolveMaxLakes)
    SHADER_PARAMETER(uint32, SolveErode)
    SHADER_PARAMETER(float, SolveJitter)
    SHADER_PARAMETER(float, SolveRouteNoise)
    SHADER_PARAMETER(float, SolveSpreadCells)
    SHADER_PARAMETER(float, SolveUpliftVariation)
    SHADER_PARAMETER(float, SolveErodibilityVariation)
    SHADER_PARAMETER(float, SolveVariationWavelength)
    SHADER_PARAMETER(float, SolveRoughness)
    SHADER_PARAMETER(float, SolveReliefScale)
    SHADER_PARAMETER(uint32, SolveFillDepressions)
    SHADER_PARAMETER(uint32, SolveModel)
    SHADER_PARAMETER(float, SolvePlainsRelief)
    SHADER_PARAMETER(float, SolveUndulation)
    SHADER_PARAMETER(float, SolveLandformErosion)
    SHADER_PARAMETER(float, SolveIncision)
    SHADER_PARAMETER(float, SolveChannelCells)
    SHADER_PARAMETER(float, SolveChannelVariation)
    SHADER_PARAMETER(float, SolveChannelWavelength)
    SHADER_PARAMETER(uint32, SolveChannelOctaves)
    SHADER_PARAMETER(float, SolveHillslope)
    SHADER_PARAMETER(float, SolveChannelCharacter)
    // Per cell.
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SolveH)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SolveW)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SolveWater)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SolveRec)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SolveSlot)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SolveLabel)
    // Per slot.
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SlotCell)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotU)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotK)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotAc)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotArea)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotDist)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotH0)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotA)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotAOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, SlotWOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, JumpX)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, JumpXOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, JumpY)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, JumpYOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, JumpP)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, JumpPOut)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SolveFlags)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, AccHi)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, AccHiOut)
    SHADER_PARAMETER(float, SolveAccUnit)
    // Lakes and outputs (the last frame; stand-ins before).
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeArea)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeDepth)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeOutlet)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeCount)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeList)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutHeight)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutFlow)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutWater)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutAttr)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutForm)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutRec)
    // Dispatch, compaction, the steps' state, the histograms.
    SHADER_PARAMETER(uint32, GroupsX)
    SHADER_PARAMETER(uint32, NumSlots)
    SHADER_PARAMETER(uint32, SolveDomain)
    SHADER_PARAMETER(uint32, PassMode)
    SHADER_PARAMETER(uint32, GateOnDone)
    SHADER_PARAMETER(uint32, NumScanGroups)
    SHADER_PARAMETER(uint32, SolveMaxSteps)
    SHADER_PARAMETER(uint32, SolveMinSteps)
    SHADER_PARAMETER(float, SolveTolerance)
    SHADER_PARAMETER(float, HistogramMax)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, GroupCounts)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SolveCounts)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SolveState)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, Partials)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, ResidualLog)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, Histogram)
END_SHADER_PARAMETER_STRUCT()

#define VOXEL_LAND_SOLVE_SHADER(ShaderName) \
    class ShaderName : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(ShaderName); \
        using FParameters = FVoxelLandSolveParameters; \
        SHADER_USE_PARAMETER_STRUCT(ShaderName, FGlobalShader); \
        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
        { \
            return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
        } \
        static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment) \
        { \
            FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment); \
            OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath); \
        } \
    };

#define VOXEL_LAND_SOLVE_SHADERS(X) \
    X(FVoxelSolveCountCS, "CountCS") \
    X(FVoxelSolveScanCS, "ScanCS") \
    X(FVoxelSolveScatterCS, "ScatterCS") \
    X(FVoxelSolveInitCS, "InitCS") \
    X(FVoxelSolveResetCS, "ResetCS") \
    X(FVoxelSolveFillTiledCS, "FillTiledCS") \
    X(FVoxelSolveFillJacobiCS, "FillJacobiCS") \
    X(FVoxelSolveFillCopyCS, "FillCopyCS") \
    X(FVoxelSolveUndrainedCS, "UndrainedCS") \
    X(FVoxelSolveReceiversCS, "ReceiversCS") \
    X(FVoxelSolveAccumulateJacobiCS, "AccumulateJacobiCS") \
    X(FVoxelSolveRouteTreeInitCS, "RouteTreeInitCS") \
    X(FVoxelSolveAccJumpInitCS, "AccJumpInitCS") \
    X(FVoxelSolveAccJumpCopyCS, "AccJumpCopyCS") \
    X(FVoxelSolveAccJumpAddCS, "AccJumpAddCS") \
    X(FVoxelSolveAccJumpFinishCS, "AccJumpFinishCS") \
    X(FVoxelSolveSpreadInitCS, "SpreadInitCS") \
    X(FVoxelSolveSpreadCS, "SpreadCS") \
    X(FVoxelSolveTreeFillInitCS, "TreeFillInitCS") \
    X(FVoxelSolveTreeFillJumpCS, "TreeFillJumpCS") \
    X(FVoxelSolveTreeFillFinishCS, "TreeFillFinishCS") \
    X(FVoxelSolveImplicitInitCS, "ImplicitInitCS") \
    X(FVoxelSolveImplicitJumpCS, "ImplicitJumpCS") \
    X(FVoxelSolveImplicitFinishCS, "ImplicitFinishCS") \
    X(FVoxelSolveCapInitCS, "CapInitCS") \
    X(FVoxelSolveCapJumpCS, "CapJumpCS") \
    X(FVoxelSolveCapFinishCS, "CapFinishCS") \
    X(FVoxelSolveResidualCS, "ResidualCS") \
    X(FVoxelSolveSettleMarkCS, "SettleMarkCS") \
    X(FVoxelSolveSettleInitCS, "SettleInitCS") \
    X(FVoxelSolveSettleCS, "SettleCS") \
    X(FVoxelSolveOutletInitCS, "OutletInitCS") \
    X(FVoxelSolveOutletJumpCS, "OutletJumpCS") \
    X(FVoxelSolveHistogramCS, "HistogramCS") \
    X(FVoxelSolveNormaliseCS, "NormaliseCS") \
    X(FVoxelSolveRouteInitCS, "RouteInitCS") \
    X(FVoxelSolveLabelInitCS, "LabelInitCS") \
    X(FVoxelSolveLabelCS, "LabelCS") \
    X(FVoxelSolveLakeStatsCS, "LakeStatsCS") \
    X(FVoxelSolveLakeFinishCS, "LakeFinishCS") \
    X(FVoxelSolveOutputCS, "OutputCS")

#define VOXEL_LAND_SOLVE_DECLARE(ShaderName, Entry) VOXEL_LAND_SOLVE_SHADER(ShaderName)
VOXEL_LAND_SOLVE_SHADERS(VOXEL_LAND_SOLVE_DECLARE)
#undef VOXEL_LAND_SOLVE_DECLARE

#define VOXEL_LAND_SOLVE_IMPLEMENT(ShaderName, Entry) IMPLEMENT_GLOBAL_SHADER(ShaderName, "/Plugin/VoxelPlugin/Private/VoxelLandSolve.usf", Entry, SF_Compute);
VOXEL_LAND_SOLVE_SHADERS(VOXEL_LAND_SOLVE_IMPLEMENT)
#undef VOXEL_LAND_SOLVE_IMPLEMENT

namespace VoxelLandSolveDetail
{
    constexpr uint32 None = 0xffffffffu;           // VOXEL_SOLVE_NONE
    constexpr uint32 HistogramBins = 4096;         // VOXEL_SOLVE_BINS
    constexpr float HistogramMax = 32768.0f;       // m
    constexpr int32 JumpRounds = 20;               // paths up to 2^20 cells (even: the buffers swap back)
    constexpr int32 FillRefinements = 4;           // Planchon-Darboux refinements per step
    constexpr int32 AccumulateIterations = 16;     // drainage iterations per step (even)
    constexpr int32 MaxPending = 3;                // readbacks in flight per phase
    constexpr uint32 MaxLakes = 65536;
    constexpr int32 MinIterationsPerFrame = 2;
    constexpr int32 MaxIterationsPerFrame = 512;
    constexpr int32 MaxStepsPerFrame = 16;
    // S1: the routing recorded ahead in one frame, each loop a fixed number of passes, checked
    // when the fields come back (a loop that hadn't settled routes again by loops). Measured
    // need at 512 cells a face (the harness's passes double-buffered, which bound the GPU's in
    // place): the fills from the tree fill 2-3 tiled passes (13-18 where some land is outside the
    // solve and starts from INF: then N / 8, at most AheadFillPassesMax), the hillslopes'
    // spread 37, the labels 16-53.
    constexpr int32 AheadFillPasses = 8;
    constexpr int32 AheadFillPassesMax = 64;
    constexpr int32 AheadSpreadPasses = 128;
    constexpr int32 AheadLabelPasses = 256;

    static_assert(JumpRounds % 2 == 0 && AccumulateIterations % 2 == 0, "the double buffers must swap back each step");

    /** A buffer copy on its way to the CPU. */
    struct FReadback
    {
        TUniquePtr<FRHIGPUBufferReadback> Readback;
        uint32 Bytes = 0;

        bool IsReady() const { return Readback.IsValid() && Readback->IsReady(); }
    };

    FReadback Enqueue(FRDGBuilder& GraphBuilder, FRDGBufferRef Buffer, uint32 Bytes, const TCHAR* Name)
    {
        FReadback Out;
        Out.Readback = MakeUnique<FRHIGPUBufferReadback>(Name);
        Out.Bytes = Bytes;
        AddEnqueueCopyPass(GraphBuilder, Out.Readback.Get(), Buffer, Bytes);
        return Out;
    }

    /** Copies a ready readback's words; false when it couldn't be read. */
    template<typename T>
    bool Read(FReadback& In, TArray<T>& Out)
    {
        Out.SetNumUninitialized((int32)(In.Bytes / sizeof(T)));
        const void* Data = In.Readback->Lock(In.Bytes);
        if (!Data) return false;
        FMemory::Memcpy(Out.GetData(), Data, In.Bytes);
        In.Readback->Unlock();
        In.Readback.Reset();
        return true;
    }

    /** The value at quantile Q of a histogram over [0, HistogramMax) (linear in a bin). */
    float Quantile(const uint32* Bins, double Q)
    {
        uint64 Total = 0;
        for (uint32 b = 0; b < HistogramBins; b++)
            Total += Bins[b];
        if (Total == 0) return 0.0f;
        const double Target = Q * (double)Total;
        uint64 Cumulative = 0;
        for (uint32 b = 0; b < HistogramBins; b++)
        {
            if (Bins[b] > 0 && (double)(Cumulative + Bins[b]) >= Target)
            {
                const double Inside = FMath::Clamp((Target - (double)Cumulative) / (double)Bins[b], 0.0, 1.0);
                return (float)(((double)b + Inside) / (double)HistogramBins * HistogramMax);
            }
            Cumulative += Bins[b];
        }
        return HistogramMax;
    }
}

namespace VLS = VoxelLandSolveDetail;

bool VoxelLandSolve::AreShadersReady()
{
    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    if (!ShaderMap) return false;
#define VOXEL_LAND_SOLVE_READY(ShaderName, Entry) if (!ShaderMap->GetShader(&ShaderName::GetStaticType(), 0).IsValid()) return false;
    VOXEL_LAND_SOLVE_SHADERS(VOXEL_LAND_SOLVE_READY)
#undef VOXEL_LAND_SOLVE_READY
    return true;
}

struct VoxelLandSolve::FJob::FImpl
{
    enum class EPhase : uint8
    {
        Start,
        CountWait,      // a domain's slot count
        InitFill,       // the solve's first fill (until it converges)
        Steps,          // the solve's steps (until the GPU says done)
        OutletsWait,    // the normalisation's histograms
        Fill0,          // the routing's lake levels
        Fill1,          // the routing's fill (then its drainage, exact in fixed passes: S1)
        Spread,         // the drainage field's spread over the hillslopes (T1b; the channels in fixed passes)
        Labels,         // lake labels
        OutputWait,     // the fields (and the rivers' tables)
        RiverWait,      // the rivers' tables built again with the sizes they asked for
        Done
    };

    FSettings S;
    uint32 NumCells = 0;
    uint32 NumScanGroups = 0;
    EPhase Phase = EPhase::Start;
    uint32 CountDomain = 0;
    uint32 NumSlots = 0;
    int32 JumpParity = 0;
    int32 AParity = 0;
    int32 LoopIterations = 0;
    int32 StepsDispatched = 0;
    bool bSolved = false;          // the solve ran (its receivers start the routing's fills: S1)
    uint32 SolvedSlots = 0;        // the solve's domain (S1: land outside it starts the fills from INF)
    bool bRouteByLoops = false;    // the routing recorded ahead didn't settle: by loops
    TArray<VLS::FReadback> Checks; // the routing ahead's loops' flags (S1)
    VLS::FReadback PreRouting;     // the flags before it (the errors so far)
    uint32 ErrorsBeforeRouting = 0;
    float Factor = 1.0f;
    float Knee = 1.0e30f;
    float Top = 1.0e30f;
    float ReliefScale = 0.0f;
    uint32 Errors = 0;
    double PhaseStart = 0.0;
    int32 PhaseFrames = 0;
    TArray<VLS::FReadback> Pending;
    VoxelLandRiverBuild::FPending Rivers;
    VoxelLandRiverBuild::FSettings RiverSettings;
    int32 RiverBuilds = 0;
    // The last frame's fields, kept for the density and a river build that runs again.
    TRefCountPtr<FRDGPooledBuffer> KeptHeight, KeptWater, KeptFlow, KeptRec;

    // Pooled buffers kept between frames (render thread).
    TRefCountPtr<FRDGPooledBuffer> Tan, H, W, Water, Rec, Slot, Label, Cell, GroupCounts, Counts, Flags, State, Partials, Log, Histogram;
    TRefCountPtr<FRDGPooledBuffer> U, Erodibility, ChannelFactor, Area, Dist, H0, WOut;
    TRefCountPtr<FRDGPooledBuffer> A[2], X[2], Y[2], P[2], Hi[2];
    // S1: the heights and the solve's receivers as the routing ahead found them (it changes
    // both: the lakes not kept are filled into the heights, the receivers become the
    // routing's), for routing again by loops from the same start.
    TRefCountPtr<FRDGPooledBuffer> KeptH, KeptSolveRec;

    /** This frame's view of the buffers. */
    struct FFrame
    {
        FRDGBufferRef BakeHeight = nullptr, BakeUp = nullptr, BakeCrust = nullptr, BakeAttr = nullptr, BakeForm = nullptr;
        FRDGBufferRef Tan = nullptr, H = nullptr, W = nullptr, Water = nullptr, Rec = nullptr, Slot = nullptr, Label = nullptr, Cell = nullptr;
        FRDGBufferRef GroupCounts = nullptr, Counts = nullptr, Flags = nullptr, State = nullptr, Partials = nullptr, Log = nullptr, Histogram = nullptr;
        FRDGBufferRef U = nullptr, Erodibility = nullptr, ChannelFactor = nullptr, Area = nullptr, Dist = nullptr, H0 = nullptr, WOut = nullptr;
        FRDGBufferRef A[2] = {}, X[2] = {}, Y[2] = {}, P[2] = {}, Hi[2] = {};
        // The last frame's lake and output buffers (else stand-ins).
        FRDGBufferRef LakeArea = nullptr, LakeDepth = nullptr, LakeOutlet = nullptr, LakeCount = nullptr, LakeList = nullptr;
        FRDGBufferRef OutHeight = nullptr, OutFlow = nullptr, OutWater = nullptr, OutAttr = nullptr, OutForm = nullptr, OutRec = nullptr;
        /** Every pass's parameters with this frame's views (BuildBase), and the views of the
         *  double buffers, which the passes pick by parity. */
        FVoxelLandSolveParameters Base;
        FRDGBufferUAVRef AUav[2] = {}, XUav[2] = {}, YUav[2] = {}, PUav[2] = {}, HiUav[2] = {};
    };

    explicit FImpl(FSettings InSettings)
        : S(MoveTemp(InSettings))
    {
        const uint32 Side = (uint32)(S.N + 2 * S.Pad);
        NumCells = 6u * Side * Side;
        NumScanGroups = FMath::DivideAndRoundUp(NumCells, 64u);
    }

    /** A pooled buffer of at least Count elements: kept, or (re)created. */
    static FRDGBufferRef Get(FRDGBuilder& GraphBuilder, TRefCountPtr<FRDGPooledBuffer>& Pooled, uint32 Stride, uint32 InElements, const TCHAR* Name)
    {
        const uint32 Elements = FMath::Max(InElements, 1u);
        if (Pooled.IsValid() && Pooled->Desc.BytesPerElement == Stride && Pooled->Desc.NumElements >= Elements)
            return GraphBuilder.RegisterExternalBuffer(Pooled);
        FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(Stride, Elements);
        Desc.Usage |= EBufferUsageFlags::SourceCopy;
        FRDGBufferRef Buffer = GraphBuilder.CreateBuffer(Desc, Name);
        Pooled = GraphBuilder.ConvertToExternalBuffer(Buffer);
        return Buffer;
    }

    static FRDGBufferRef Transient(FRDGBuilder& GraphBuilder, uint32 Stride, uint32 Elements, const TCHAR* Name)
    {
        FRDGBufferDesc Desc = FRDGBufferDesc::CreateStructuredDesc(Stride, FMath::Max(Elements, 1u));
        Desc.Usage |= EBufferUsageFlags::SourceCopy;
        return GraphBuilder.CreateBuffer(Desc, Name);
    }

    static FRDGBufferRef StandIn(FRDGBuilder& GraphBuilder, const TCHAR* Name)
    {
        return GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 1), Name);
    }

    /** Registers this frame's buffers; the per-slot ones hold NumSlots (one before a count). */
    void Register(FRDGBuilder& GraphBuilder, const FPlateFields& Fields, FFrame& F)
    {
        F.BakeHeight = GraphBuilder.RegisterExternalBuffer(Fields.Height);
        F.BakeUp = GraphBuilder.RegisterExternalBuffer(Fields.Up);
        F.BakeCrust = GraphBuilder.RegisterExternalBuffer(Fields.Crust);
        F.BakeAttr = GraphBuilder.RegisterExternalBuffer(Fields.Attr);
        F.BakeForm = GraphBuilder.RegisterExternalBuffer(Fields.Form);
        if (!Tan.IsValid())
        {
            F.Tan = VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandSolveTan"), MakeConstArrayView(S.TanTable));
            Tan = GraphBuilder.ConvertToExternalBuffer(F.Tan);
        }
        else
        {
            F.Tan = GraphBuilder.RegisterExternalBuffer(Tan);
        }
        F.H = Get(GraphBuilder, H, 4, NumCells, TEXT("Voxel.LandSolveH"));
        F.W = Get(GraphBuilder, W, 4, NumCells, TEXT("Voxel.LandSolveW"));
        F.Water = Get(GraphBuilder, Water, 4, NumCells, TEXT("Voxel.LandSolveWater"));
        F.Rec = Get(GraphBuilder, Rec, 4, NumCells, TEXT("Voxel.LandSolveRec"));
        F.Slot = Get(GraphBuilder, Slot, 4, NumCells, TEXT("Voxel.LandSolveSlot"));
        F.Label = Get(GraphBuilder, Label, 4, NumCells, TEXT("Voxel.LandSolveLabel"));
        F.Cell = Get(GraphBuilder, Cell, 4, NumCells, TEXT("Voxel.LandSolveSlotCell"));
        F.GroupCounts = Get(GraphBuilder, GroupCounts, 4, NumScanGroups, TEXT("Voxel.LandSolveGroupCounts"));
        F.Counts = Get(GraphBuilder, Counts, 4, 4, TEXT("Voxel.LandSolveCounts"));
        F.Flags = Get(GraphBuilder, Flags, 4, 2, TEXT("Voxel.LandSolveFlags"));
        F.State = Get(GraphBuilder, State, 4, 2, TEXT("Voxel.LandSolveState"));
        F.Log = Get(GraphBuilder, Log, 4, (uint32)FMath::Max(S.MaxSteps, 1), TEXT("Voxel.LandSolveLog"));
        F.Histogram = Get(GraphBuilder, Histogram, 4, 2 * VLS::HistogramBins, TEXT("Voxel.LandSolveHistogram"));
        RegisterSlots(GraphBuilder, F);
    }

    void RegisterSlots(FRDGBuilder& GraphBuilder, FFrame& F)
    {
        const uint32 SlotCount = FMath::Max(NumSlots, 1u);
        F.Partials = Get(GraphBuilder, Partials, 4, 2 * FMath::DivideAndRoundUp(SlotCount, 64u), TEXT("Voxel.LandSolvePartials"));
        F.U = Get(GraphBuilder, U, 4, SlotCount, TEXT("Voxel.LandSolveU"));
        F.Erodibility = Get(GraphBuilder, Erodibility, 4, SlotCount, TEXT("Voxel.LandSolveK"));
        F.ChannelFactor = Get(GraphBuilder, ChannelFactor, 4, SlotCount, TEXT("Voxel.LandSolveAc"));
        F.Area = Get(GraphBuilder, Area, 4, SlotCount, TEXT("Voxel.LandSolveArea"));
        F.Dist = Get(GraphBuilder, Dist, 4, SlotCount, TEXT("Voxel.LandSolveDist"));
        F.H0 = Get(GraphBuilder, H0, 4, SlotCount, TEXT("Voxel.LandSolveH0"));
        F.WOut = Get(GraphBuilder, WOut, 4, SlotCount, TEXT("Voxel.LandSolveWOut"));
        for (int32 i = 0; i < 2; i++)
        {
            F.A[i] = Get(GraphBuilder, A[i], 4, SlotCount, TEXT("Voxel.LandSolveA"));
            F.X[i] = Get(GraphBuilder, X[i], 4, SlotCount, TEXT("Voxel.LandSolveJumpX"));
            F.Y[i] = Get(GraphBuilder, Y[i], 4, SlotCount, TEXT("Voxel.LandSolveJumpY"));
            F.P[i] = Get(GraphBuilder, P[i], 4, SlotCount, TEXT("Voxel.LandSolveJumpP"));
            F.Hi[i] = Get(GraphBuilder, Hi[i], 4, SlotCount, TEXT("Voxel.LandSolveAccHi"));
        }
        BuildBase(GraphBuilder, F);
    }

    /** This frame's views, once (the passes copy them): after Register, and again whenever a
     *  buffer is replaced. */
    void BuildBase(FRDGBuilder& GraphBuilder, FFrame& F) const
    {
        FVoxelLandSolveParameters& Params = F.Base;
        Params = FVoxelLandSolveParameters();
        Params.SolveN = (uint32)S.N;
        Params.SolvePad = (uint32)S.Pad;
        Params.SolveRadiusM = S.RadiusM;
        Params.SolveTan = GraphBuilder.CreateSRV(F.Tan);
        Params.BakeHeight = GraphBuilder.CreateSRV(F.BakeHeight);
        Params.BakeUp = GraphBuilder.CreateSRV(F.BakeUp);
        Params.BakeCrust = GraphBuilder.CreateSRV(F.BakeCrust);
        Params.BakeAttr = GraphBuilder.CreateSRV(F.BakeAttr);
        Params.BakeForm = GraphBuilder.CreateSRV(F.BakeForm);
        Params.SolvePlainsUplift = S.PlainsUplift;
        Params.SolveRampWidth = S.RampWidth;
        Params.SolveUpliftTime = S.UpliftTime;
        Params.SolveKdt = S.Kdt;
        Params.SolveDt = S.Dt;
        Params.SolveEps = S.Eps;
        Params.SolveSeed = S.Seed;
        Params.SolveMinLakeDepth = S.MinLakeDepth;
        Params.SolveMinLakeArea = S.MinLakeArea;
        Params.SolveMaxLakes = VLS::MaxLakes;
        Params.SolveErode = S.bErode ? 1u : 0u;
        Params.SolveJitter = S.Jitter;
        Params.SolveRouteNoise = S.RouteNoise;
        Params.SolveSpreadCells = S.SpreadCells;
        Params.SolveUpliftVariation = S.UpliftVariation;
        Params.SolveErodibilityVariation = S.ErodibilityVariation;
        Params.SolveVariationWavelength = S.VariationWavelength;
        Params.SolveRoughness = S.Roughness;
        Params.SolveReliefScale = ReliefScale;
        // The T1 model (Design/SurfaceLook.md): the static surface kept, the plains solved.
        Params.SolveFillDepressions = 1u;
        Params.SolveModel = 1u;
        Params.SolvePlainsRelief = S.PlainsRelief;
        Params.SolveUndulation = S.Undulation;
        Params.SolveLandformErosion = S.LandformErosion;
        Params.SolveIncision = S.Incision;
        // T3g: the channel heads and the hillslopes' threshold.
        Params.SolveChannelCells = S.ChannelCells;
        Params.SolveChannelVariation = S.ChannelVariation;
        Params.SolveChannelWavelength = S.ChannelWavelength;
        Params.SolveChannelOctaves = (uint32)FMath::Clamp(S.ChannelOctaves, 1, 6);
        Params.SolveHillslope = S.Hillslope;
        Params.SolveChannelCharacter = S.ChannelCharacter;
        // S1: the exact accumulation's unit, a 4096th of a cell at a face's centre.
        Params.SolveAccUnit = FMath::Square(UE_PI * 0.5f * S.RadiusM / (float)FMath::Max(S.N, 1)) / 4096.0f;
        Params.SolveH = GraphBuilder.CreateUAV(F.H);
        Params.SolveW = GraphBuilder.CreateUAV(F.W);
        Params.SolveWater = GraphBuilder.CreateUAV(F.Water);
        Params.SolveRec = GraphBuilder.CreateUAV(F.Rec);
        Params.SolveSlot = GraphBuilder.CreateUAV(F.Slot);
        Params.SolveLabel = GraphBuilder.CreateUAV(F.Label);
        Params.SlotCell = GraphBuilder.CreateUAV(F.Cell);
        Params.SlotU = GraphBuilder.CreateUAV(F.U);
        Params.SlotK = GraphBuilder.CreateUAV(F.Erodibility);
        Params.SlotAc = GraphBuilder.CreateUAV(F.ChannelFactor);
        Params.SlotArea = GraphBuilder.CreateUAV(F.Area);
        Params.SlotDist = GraphBuilder.CreateUAV(F.Dist);
        Params.SlotH0 = GraphBuilder.CreateUAV(F.H0);
        Params.SlotWOut = GraphBuilder.CreateUAV(F.WOut);
        for (int32 i = 0; i < 2; i++)
        {
            F.AUav[i] = GraphBuilder.CreateUAV(F.A[i]);
            F.XUav[i] = GraphBuilder.CreateUAV(F.X[i]);
            F.YUav[i] = GraphBuilder.CreateUAV(F.Y[i]);
            F.PUav[i] = GraphBuilder.CreateUAV(F.P[i]);
            F.HiUav[i] = GraphBuilder.CreateUAV(F.Hi[i]);
        }
        Params.SolveFlags = GraphBuilder.CreateUAV(F.Flags);
        auto OrStandIn = [&GraphBuilder](FRDGBufferRef Buffer, const TCHAR* Name)
            {
                return GraphBuilder.CreateUAV(Buffer ? Buffer : StandIn(GraphBuilder, Name));
            };
        Params.LakeArea = OrStandIn(F.LakeArea, TEXT("Voxel.LandSolveNoLakeArea"));
        Params.LakeDepth = OrStandIn(F.LakeDepth, TEXT("Voxel.LandSolveNoLakeDepth"));
        Params.LakeOutlet = OrStandIn(F.LakeOutlet, TEXT("Voxel.LandSolveNoLakeOutlet"));
        Params.LakeCount = OrStandIn(F.LakeCount, TEXT("Voxel.LandSolveNoLakeCount"));
        Params.LakeList = OrStandIn(F.LakeList, TEXT("Voxel.LandSolveNoLakeList"));
        Params.OutHeight = OrStandIn(F.OutHeight, TEXT("Voxel.LandSolveNoOutHeight"));
        Params.OutFlow = OrStandIn(F.OutFlow, TEXT("Voxel.LandSolveNoOutFlow"));
        Params.OutWater = OrStandIn(F.OutWater, TEXT("Voxel.LandSolveNoOutWater"));
        Params.OutAttr = OrStandIn(F.OutAttr, TEXT("Voxel.LandSolveNoOutAttr"));
        Params.OutForm = OrStandIn(F.OutForm, TEXT("Voxel.LandSolveNoOutForm"));
        Params.OutRec = OrStandIn(F.OutRec, TEXT("Voxel.LandSolveNoOutRec"));
        Params.NumScanGroups = NumScanGroups;
        Params.SolveMaxSteps = (uint32)FMath::Max(S.MaxSteps, 1);
        Params.SolveMinSteps = (uint32)FMath::Max(S.MinSteps, 1);
        Params.SolveTolerance = S.Tolerance;
        Params.HistogramMax = VLS::HistogramMax;
        Params.GroupCounts = GraphBuilder.CreateUAV(F.GroupCounts);
        Params.SolveCounts = GraphBuilder.CreateUAV(F.Counts);
        Params.SolveState = GraphBuilder.CreateUAV(F.State);
        Params.Partials = GraphBuilder.CreateUAV(F.Partials);
        Params.ResidualLog = GraphBuilder.CreateUAV(F.Log);
        Params.Histogram = GraphBuilder.CreateUAV(F.Histogram);
    }

    /** A pass's parameters: the frame's views, the current parities and values. */
    FVoxelLandSolveParameters MakeParameters(const FFrame& F, uint32 Threads) const
    {
        FVoxelLandSolveParameters Params = F.Base;
        Params.SolveFactor = Factor;
        Params.SolveReliefKnee = Knee;
        Params.SolveReliefTop = Top;
        Params.SolveReliefScale = ReliefScale;
        Params.SlotA = F.AUav[AParity];
        Params.SlotAOut = F.AUav[1 - AParity];
        Params.JumpX = F.XUav[JumpParity];
        Params.JumpXOut = F.XUav[1 - JumpParity];
        Params.JumpY = F.YUav[JumpParity];
        Params.JumpYOut = F.YUav[1 - JumpParity];
        Params.JumpP = F.PUav[JumpParity];
        Params.JumpPOut = F.PUav[1 - JumpParity];
        Params.AccHi = F.HiUav[JumpParity];
        Params.AccHiOut = F.HiUav[1 - JumpParity];
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(Threads, 64u), 1u);
        Params.GroupsX = FMath::Min(NumGroups, 32768u);
        Params.NumSlots = NumSlots;
        Params.SolveDomain = CountDomain;
        Params.PassMode = 0;
        Params.GateOnDone = 0;
        return Params;
    }

    /** Records one pass over Threads threads (cells or slots), with its mode and whether the done flag skips it. */
    template<typename TShader>
    void Pass(FRDGBuilder& GraphBuilder, const FFrame& F, uint32 Threads, const TCHAR* Name, uint32 PassMode = 0, uint32 Gate = 0)
    {
        FVoxelLandSolveParameters* Parameters = GraphBuilder.AllocParameters<FVoxelLandSolveParameters>();
        *Parameters = MakeParameters(F, Threads);
        Parameters->PassMode = PassMode;
        Parameters->GateOnDone = Gate;
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(Threads, 64u), 1u);
        const FIntVector Groups((int32)Parameters->GroupsX, (int32)FMath::DivideAndRoundUp(NumGroups, Parameters->GroupsX), 1);
        TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand %s", Name), Shader, Parameters, Groups);
    }

    /** Records one tiled pass (VoxelSolveFillTiledCS): a group of 16 x 16 threads per tile of a
     *  face (VOXEL_SOLVE_FILL_TILE). */
    template<typename TShader>
    void PassTiles(FRDGBuilder& GraphBuilder, const FFrame& F, const TCHAR* Name, uint32 PassMode = 0)
    {
        const uint32 TilesPerFace = (uint32)FMath::DivideAndRoundUp(S.N, 16);
        const uint32 Tiles = 6u * TilesPerFace * TilesPerFace;
        FVoxelLandSolveParameters* Parameters = GraphBuilder.AllocParameters<FVoxelLandSolveParameters>();
        *Parameters = MakeParameters(F, 1);
        Parameters->GroupsX = FMath::Min(Tiles, 32768u);
        Parameters->PassMode = PassMode;
        const FIntVector Groups((int32)Parameters->GroupsX, (int32)FMath::DivideAndRoundUp(Tiles, Parameters->GroupsX), 1);
        TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand %s", Name), Shader, Parameters, Groups);
    }

    /** One group (the scan, the residual, the reset). */
    template<typename TShader>
    void Single(FRDGBuilder& GraphBuilder, const FFrame& F, const TCHAR* Name, uint32 PassMode = 0)
    {
        FVoxelLandSolveParameters* Parameters = GraphBuilder.AllocParameters<FVoxelLandSolveParameters>();
        *Parameters = MakeParameters(F, 1);
        Parameters->PassMode = PassMode;
        TShaderMapRef<TShader> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLand %s", Name), Shader, Parameters, FIntVector(1, 1, 1));
    }

    void SetPhase(EPhase InPhase, FResult& OutResult)
    {
        const double Now = FPlatformTime::Seconds();
        static const TCHAR* Names[] = { TEXT("start"), TEXT("count"), TEXT("initial fill"), TEXT("steps"), TEXT("normalisation"),
            TEXT("lake levels"), TEXT("routing fill"), TEXT("drainage spread"), TEXT("lake labels"), TEXT("output"), TEXT("rivers again"), TEXT("done") };
        OutResult.PhaseLog.Add(FString::Printf(TEXT("%s %d frames %.2f s"), Names[(int32)Phase], PhaseFrames, Now - PhaseStart));
        Phase = InPhase;
        PhaseStart = Now;
        PhaseFrames = 0;
        LoopIterations = 0;
    }

    /** Compacts a domain into slots (in cell order) and reads the count back. */
    void CountDomainSlots(FRDGBuilder& GraphBuilder, const FFrame& F, uint32 InDomain, FResult& OutResult)
    {
        CountDomain = InDomain;
        Pass<FVoxelSolveCountCS>(GraphBuilder, F, NumCells, TEXT("count"));
        Single<FVoxelSolveScanCS>(GraphBuilder, F, TEXT("scan"));
        Pass<FVoxelSolveScatterCS>(GraphBuilder, F, NumCells, TEXT("scatter"));
        Pending.Reset();
        Pending.Add(VLS::Enqueue(GraphBuilder, F.Counts, 4, TEXT("Voxel.LandSolveCountReadback")));
        SetPhase(EPhase::CountWait, OutResult);
    }

    /** A loop iterated to its fixed point: K passes this frame (the last one counted), unless
     *  enough are in flight; true once a readback says nothing changed (or one failed:
     *  bOutFailed). bTiled: the fill by tiles (PassTiles; a pass covers every cell, not only
     *  the slots, and iterates in the tile: about two slot passes' cost, so half as many a frame). */
    template<typename TShader, bool bTiled = false>
    bool Loop(FRDGBuilder& GraphBuilder, const FFrame& F, const TCHAR* Name, uint32 PassMode, bool& bOutFailed)
    {
        while (Pending.Num() > 0 && Pending[0].IsReady())
        {
            TArray<uint32> Words;
            if (!VLS::Read(Pending[0], Words) || Words.Num() < 2)
            {
                bOutFailed = true;
                Pending.Reset();
                return true;
            }
            Pending.RemoveAt(0);
            Errors |= Words[1];
            if (Words[0] == 0u)
            {
                Pending.Reset();
                return true;
            }
        }
        const int32 Cap = 64 * FMath::Max(S.N, 64);
        if (LoopIterations >= Cap)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The solve's %s didn't converge in %d iterations; going on."), Name, LoopIterations);
            Pending.Reset();
            return true;
        }
        if (Pending.Num() >= VLS::MaxPending) return false;
        int32 K = (int32)FMath::Clamp<int64>(S.IterationCellsPerFrame / FMath::Max<int64>(NumSlots, 1), VLS::MinIterationsPerFrame, VLS::MaxIterationsPerFrame);
        if (bTiled)
            K = FMath::Clamp(K / 2, VLS::MinIterationsPerFrame, 64);
        for (int32 k = 0; k < K; k++)
        {
            if (k == K - 1)
                Single<FVoxelSolveResetCS>(GraphBuilder, F, TEXT("reset flag"));
            if (bTiled)
                PassTiles<TShader>(GraphBuilder, F, Name, PassMode);
            else
                Pass<TShader>(GraphBuilder, F, NumSlots, Name, PassMode);
        }
        LoopIterations += K;
        Pending.Add(VLS::Enqueue(GraphBuilder, F.Flags, 8, TEXT("Voxel.LandSolveFlagsReadback")));
        return false;
    }

    /** R1b: records the rivers' build on this frame's fields (the routing's label and slot
     *  buffers as its per-cell scratch: free after the output pass), with its readbacks. */
    void RecordRivers(FRDGBuilder& GraphBuilder, const FFrame& F, FRDGBufferRef InHeight, FRDGBufferRef InWater, FRDGBufferRef InFlow, FRDGBufferRef InRec,
        const VoxelLandRiverBuild::FCaps& Caps)
    {
        VoxelLandRiverBuild::FInputs In;
        In.Tan = F.Tan;
        In.Height = InHeight;
        In.Water = InWater;
        In.Flow = InFlow;
        In.Rec = InRec;
        In.CellFlags = F.Label;
        In.CellNode = F.Cell;
        const VoxelLandRiverBuild::FOutputs Out = VoxelLandRiverBuild::AddPasses(GraphBuilder, In, RiverSettings, Caps);
        Rivers.Enqueue(GraphBuilder, Out, RiverSettings, Caps);
        RiverBuilds++;
    }

    /** A ready river build: into the result when it fit (true), else recorded again with the sizes
     *  its counts asked for (false: wait); a failed or repeatedly short build leaves no rivers. */
    bool FinishRivers(FRDGBuilder& GraphBuilder, const FFrame& F, FResult& OutResult)
    {
        VoxelLandRiverBuild::FResult R;
        if (!Rivers.Read(R))
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The rivers' readback failed; no rivers."));
            return true;
        }
        if (R.bFits)
        {
            OutResult.Rivers = MoveTemp(R);
            OutResult.bRivers = true;
            OutResult.RiverBuilds = RiverBuilds;
            return true;
        }
        if (!VoxelLandRiverBuild::AreShadersReady())
        {
            // (Recompiled meanwhile: VoxelLand::TickRivers builds them once the bake is ready.)
            OutResult.RiverBuilds = 0;
            return true;
        }
        if (R.Counts[VoxelLandRiverBuild::CountErrors] != 0 || RiverBuilds >= VoxelLandRiverBuild::MaxBuilds || !KeptHeight.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The rivers' build failed (errors 0x%x after %d builds); no rivers."),
                R.Counts[VoxelLandRiverBuild::CountErrors], RiverBuilds);
            return true;
        }
        RecordRivers(GraphBuilder, F, GraphBuilder.RegisterExternalBuffer(KeptHeight), GraphBuilder.RegisterExternalBuffer(KeptWater),
            GraphBuilder.RegisterExternalBuffer(KeptFlow), GraphBuilder.RegisterExternalBuffer(KeptRec), R.Needed);
        return false;
    }

    /** The GPU fields into the result (the bake's GPU copy takes them). */
    void Finish(FResult& OutResult)
    {
        OutResult.GPUHeight = KeptHeight;
        OutResult.GPUWater = KeptWater;
        OutResult.GPUFlow = KeptFlow;
        OutResult.GPUTan = Tan;
    }

    /** Whether the routing's fills start from the solve's last receivers (S1; the routing
     *  ahead's fallback puts them back first). */
    bool UseTreeStart() const { return bSolved; }

    /** S1: Count passes of a loop with no wait (the flag reset before the last one), and its
     *  flags read back into Checks: nothing changed in the last pass, the loop had settled. */
    template<typename TShader, bool bTiled = false>
    void Fixed(FRDGBuilder& GraphBuilder, const FFrame& F, const TCHAR* Name, uint32 PassMode, int32 Count)
    {
        for (int32 k = 0; k < Count; k++)
        {
            if (k == Count - 1)
                Single<FVoxelSolveResetCS>(GraphBuilder, F, TEXT("reset flag"));
            if (bTiled)
                PassTiles<TShader>(GraphBuilder, F, Name, PassMode);
            else
                Pass<TShader>(GraphBuilder, F, NumSlots, Name, PassMode);
        }
        Checks.Add(VLS::Enqueue(GraphBuilder, F.Flags, 8, TEXT("Voxel.LandSolveCheckReadback")));
    }

    /** S1: the whole routing in this frame, from the land's slot count: the fills from the
     *  solve's receivers, the drainage, its spread, the labels, then the output (RecordOutput).
     *  Each loop is a fixed number of passes; OutputWait checks they had settled. */
    void RecordRoutingAhead(FRDGBuilder& GraphBuilder, FFrame& F, FResult& OutResult)
    {
        Checks.Reset();
        // The errors so far, the heights and the solve's receivers, for a fallback.
        ErrorsBeforeRouting = Errors;
        PreRouting = VLS::Enqueue(GraphBuilder, F.Flags, 8, TEXT("Voxel.LandSolveCheckReadback"));
        AddCopyBufferPass(GraphBuilder, Get(GraphBuilder, KeptH, 4, NumCells, TEXT("Voxel.LandSolveKeptH")), F.H);
        AddCopyBufferPass(GraphBuilder, Get(GraphBuilder, KeptSolveRec, 4, NumCells, TEXT("Voxel.LandSolveKeptRec")), F.Rec);
        // Land outside the solve has no receivers to start from: its fills run from INF, more
        // passes the more there is (a few cells at the coasts with the default model, 4 at 1,024
        // cells a face; 5% of the land with the L2 model, 13-18 passes at 512). A heuristic:
        // solved cells now under the sea hide some, and a fill short of passes falls back to
        // the loops.
        const uint32 Outside = NumSlots > SolvedSlots ? NumSlots - SolvedSlots : 0u;
        const int32 FillPasses = Outside == 0u ? VLS::AheadFillPasses
            : (uint64)Outside * 64u < (uint64)NumSlots ? 2 * VLS::AheadFillPasses
            : FMath::Clamp(S.N / 8, VLS::AheadFillPasses, VLS::AheadFillPassesMax);
        Pass<FVoxelSolveRouteInitCS>(GraphBuilder, F, NumCells, TEXT("route init"), 2u);
        RouteTreeFill(GraphBuilder, F, 0);
        Fixed<FVoxelSolveFillTiledCS, true>(GraphBuilder, F, TEXT("lake levels"), 0, FillPasses);
        Pass<FVoxelSolveRouteInitCS>(GraphBuilder, F, NumCells, TEXT("route init"), 3u);
        RouteTreeFill(GraphBuilder, F, 1);
        Fixed<FVoxelSolveFillTiledCS, true>(GraphBuilder, F, TEXT("routing fill"), 1, FillPasses);
        Pass<FVoxelSolveReceiversCS>(GraphBuilder, F, NumSlots, TEXT("receivers"), 0);
        AccumulateByJumps(GraphBuilder, F, 0);
        if (S.SpreadCells > 0.0f)
        {
            Pass<FVoxelSolveSpreadInitCS>(GraphBuilder, F, NumSlots, TEXT("drainage spread init"));
            Fixed<FVoxelSolveSpreadCS>(GraphBuilder, F, TEXT("drainage spread"), 0, VLS::AheadSpreadPasses);
            AccumulateByJumps(GraphBuilder, F, 1);
        }
        Pass<FVoxelSolveLabelInitCS>(GraphBuilder, F, NumSlots, TEXT("lake labels"));
        Fixed<FVoxelSolveLabelCS>(GraphBuilder, F, TEXT("lake labels"), 0, VLS::AheadLabelPasses);
        RecordOutput(GraphBuilder, F, OutResult);
    }

    /** The lakes, the fields and the rivers from the settled routing, with their readbacks
     *  (OutputWait reads them). */
    void RecordOutput(FRDGBuilder& GraphBuilder, FFrame& F, FResult& OutResult)
    {
        F.LakeArea = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveLakeArea"));
        F.LakeDepth = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveLakeDepth"));
        F.LakeOutlet = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveLakeOutlet"));
        F.LakeCount = Transient(GraphBuilder, 4, 1, TEXT("Voxel.LandSolveLakeCount"));
        F.LakeList = Transient(GraphBuilder, 4, 4 * VLS::MaxLakes, TEXT("Voxel.LandSolveLakeList"));
        F.OutHeight = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveOutHeight"));
        F.OutFlow = Transient(GraphBuilder, 4, (NumCells + 1) / 2, TEXT("Voxel.LandSolveOutFlow"));
        F.OutWater = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveOutWater"));
        F.OutAttr = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveOutAttr"));
        F.OutForm = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveOutForm"));
        F.OutRec = Transient(GraphBuilder, 4, NumCells, TEXT("Voxel.LandSolveOutRec"));
        BuildBase(GraphBuilder, F);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeArea), 0u);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeDepth), 0u);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeOutlet), VLS::None);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeCount), 0u);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeList), 0u);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.OutFlow), 0u);
        Pass<FVoxelSolveLakeStatsCS>(GraphBuilder, F, NumSlots, TEXT("lake stats"));
        Pass<FVoxelSolveLakeFinishCS>(GraphBuilder, F, NumSlots, TEXT("lakes"));
        Pass<FVoxelSolveOutputCS>(GraphBuilder, F, NumCells, TEXT("output"));
        // R1b: the rivers, on the fields just made (read back with them). They use the label
        // and slot buffers as scratch: nothing over the slots runs after them.
        RiverBuilds = 0;
        if (S.bRivers && VoxelLandRiverBuild::AreShadersReady())
        {
            RiverSettings.N = S.N;
            RiverSettings.Pad = S.Pad;
            RiverSettings.RadiusM = S.RadiusM;
            RiverSettings.MinAreaKm2 = S.RiverMinAreaKm2;
            RiverSettings.WidthScale = S.RiverWidthScale;
            RiverSettings.ValleyScale = S.RiverValleyScale;
            RiverSettings.LandCells = NumSlots;
            RecordRivers(GraphBuilder, F, F.OutHeight, F.OutWater, F.OutFlow, F.OutRec, VoxelLandRiverBuild::EstimateCaps(RiverSettings));
        }
        // The fields the density samples stay on the GPU (and the receivers, for a river
        // build that has to run again).
        KeptHeight = GraphBuilder.ConvertToExternalBuffer(F.OutHeight);
        KeptWater = GraphBuilder.ConvertToExternalBuffer(F.OutWater);
        KeptFlow = GraphBuilder.ConvertToExternalBuffer(F.OutFlow);
        KeptRec = GraphBuilder.ConvertToExternalBuffer(F.OutRec);
        const uint32 Bytes = NumCells * 4;
        Pending.Reset();
        Pending.Add(VLS::Enqueue(GraphBuilder, F.OutHeight, Bytes, TEXT("Voxel.LandSolveHeightReadback")));
        Pending.Add(VLS::Enqueue(GraphBuilder, F.OutFlow, (NumCells + 1) / 2 * 4, TEXT("Voxel.LandSolveFlowReadback")));
        Pending.Add(VLS::Enqueue(GraphBuilder, F.OutWater, Bytes, TEXT("Voxel.LandSolveWaterReadback")));
        Pending.Add(VLS::Enqueue(GraphBuilder, F.OutAttr, Bytes, TEXT("Voxel.LandSolveAttrReadback")));
        Pending.Add(VLS::Enqueue(GraphBuilder, F.LakeCount, 4, TEXT("Voxel.LandSolveLakeCountReadback")));
        Pending.Add(VLS::Enqueue(GraphBuilder, F.LakeList, 16 * VLS::MaxLakes, TEXT("Voxel.LandSolveLakesReadback")));
        Pending.Add(VLS::Enqueue(GraphBuilder, F.Flags, 8, TEXT("Voxel.LandSolveFlagsReadback")));
        // The plate pass's landform, its character following the drainage density (T3h).
        Pending.Add(VLS::Enqueue(GraphBuilder, F.OutForm, Bytes, TEXT("Voxel.LandSolveFormReadback")));
        // The final routing's receivers (the rivers, R1).
        Pending.Add(VLS::Enqueue(GraphBuilder, F.OutRec, Bytes, TEXT("Voxel.LandSolveRecReadback")));
        SetPhase(EPhase::OutputWait, OutResult);
    }

    /** S1: the routing fill's start from the solve's last receivers (VoxelSolveRouteTreeInitPass;
     *  PassMode 1 with the descent). */
    void RouteTreeFill(FRDGBuilder& GraphBuilder, const FFrame& F, uint32 PassMode)
    {
        Pass<FVoxelSolveRouteTreeInitCS>(GraphBuilder, F, NumSlots, TEXT("routing tree fill"), PassMode);
        for (int32 r = 0; r < VLS::JumpRounds; r++)
        {
            Pass<FVoxelSolveTreeFillJumpCS>(GraphBuilder, F, NumSlots, TEXT("routing tree fill jump"), 0, 0);
            JumpParity ^= 1;
        }
        Pass<FVoxelSolveTreeFillFinishCS>(GraphBuilder, F, NumSlots, TEXT("routing tree fill finish"), 0, 0);
    }

    /** S1: an exact accumulation over the receivers in fixed passes (VoxelSolveAccJumpInitPass:
     *  Mode 0 the D8 drainage, 1 the spread's channels). */
    void AccumulateByJumps(FRDGBuilder& GraphBuilder, const FFrame& F, uint32 Mode)
    {
        Pass<FVoxelSolveAccJumpInitCS>(GraphBuilder, F, NumSlots, TEXT("drainage init"), Mode);
        for (int32 r = 0; r < VLS::JumpRounds; r++)
        {
            Pass<FVoxelSolveAccJumpCopyCS>(GraphBuilder, F, NumSlots, TEXT("drainage copy"));
            Pass<FVoxelSolveAccJumpAddCS>(GraphBuilder, F, NumSlots, TEXT("drainage jump"));
            JumpParity ^= 1;
        }
        Pass<FVoxelSolveAccJumpFinishCS>(GraphBuilder, F, NumSlots, TEXT("drainage finish"), Mode);
    }

    /** One step of the solve, skipped on the GPU once it is done. */
    void Step(FRDGBuilder& GraphBuilder, const FFrame& F)
    {
        const uint32 M = NumSlots;
        Pass<FVoxelSolveTreeFillInitCS>(GraphBuilder, F, M, TEXT("tree fill"), 0, 1);
        for (int32 r = 0; r < VLS::JumpRounds; r++)
        {
            Pass<FVoxelSolveTreeFillJumpCS>(GraphBuilder, F, M, TEXT("tree fill jump"), 0, 1);
            JumpParity ^= 1;
        }
        Pass<FVoxelSolveTreeFillFinishCS>(GraphBuilder, F, M, TEXT("tree fill finish"), 0, 1);
        for (int32 k = 0; k < VLS::FillRefinements; k++)
        {
            Pass<FVoxelSolveFillJacobiCS>(GraphBuilder, F, M, TEXT("fill refine"), 0, 1);
            Pass<FVoxelSolveFillCopyCS>(GraphBuilder, F, M, TEXT("fill copy"), 0, 1);
        }
        Pass<FVoxelSolveReceiversCS>(GraphBuilder, F, M, TEXT("receivers"), 1, 1);
        for (int32 k = 0; k < VLS::AccumulateIterations; k++)
        {
            Pass<FVoxelSolveAccumulateJacobiCS>(GraphBuilder, F, M, TEXT("drainage"), 0, 1);
            AParity ^= 1;
        }
        Pass<FVoxelSolveImplicitInitCS>(GraphBuilder, F, M, TEXT("implicit"), 0, 1);
        for (int32 r = 0; r < VLS::JumpRounds; r++)
        {
            Pass<FVoxelSolveImplicitJumpCS>(GraphBuilder, F, M, TEXT("implicit jump"), 0, 1);
            JumpParity ^= 1;
        }
        Pass<FVoxelSolveImplicitFinishCS>(GraphBuilder, F, M, TEXT("implicit finish"), 0, 1);
        if (S.ChannelCells > 0.0f)
        {
            // T3g: the hillslopes' threshold (its finish writes the residual's partials again).
            Pass<FVoxelSolveCapInitCS>(GraphBuilder, F, M, TEXT("slope cap"), 0, 1);
            for (int32 r = 0; r < VLS::JumpRounds; r++)
            {
                Pass<FVoxelSolveCapJumpCS>(GraphBuilder, F, M, TEXT("slope cap jump"), 0, 1);
                JumpParity ^= 1;
            }
            Pass<FVoxelSolveCapFinishCS>(GraphBuilder, F, M, TEXT("slope cap finish"), 0, 1);
        }
        Single<FVoxelSolveResidualCS>(GraphBuilder, F, TEXT("residual"));
    }

    void Release()
    {
        for (TRefCountPtr<FRDGPooledBuffer>* B : { &Tan, &H, &W, &Water, &Rec, &Slot, &Label, &Cell, &GroupCounts, &Counts, &Flags, &State, &Partials, &Log,
            &Histogram, &U, &Erodibility, &ChannelFactor, &Area, &Dist, &H0, &WOut, &A[0], &A[1], &X[0], &X[1], &Y[0], &Y[1], &P[0], &P[1], &Hi[0], &Hi[1],
            &KeptH, &KeptSolveRec, &KeptHeight, &KeptWater, &KeptFlow, &KeptRec })
            B->SafeRelease();
        Pending.Reset();
        Checks.Reset();
        PreRouting = VLS::FReadback();
        Rivers.Reset();
    }
};

VoxelLandSolve::FJob::FJob(FSettings InSettings)
    : Impl(MakeUnique<FImpl>(MoveTemp(InSettings)))
{
}

VoxelLandSolve::FJob::~FJob() = default;

void VoxelLandSolve::FJob::Tick(FRHICommandListImmediate& RHICmdList, const FPlateFields& Fields)
{
    using EPhase = FImpl::EPhase;
    FImpl& I = *Impl;
    if (Status.load(std::memory_order_acquire) != 0) return;
    if (!AreShadersReady() || !Fields.Height.IsValid() || !Fields.Up.IsValid() || !Fields.Crust.IsValid() || !Fields.Attr.IsValid()
        || !Fields.Form.IsValid())
    {
        I.Release();
        Status.store(-1, std::memory_order_release);
        return;
    }
    if (I.Phase == EPhase::Start)
        I.PhaseStart = FPlatformTime::Seconds();
    I.PhaseFrames++;

    FRDGBuilder GraphBuilder(RHICmdList);
    bool bFailed = false;
    bool bFinished = false;
    // (The event scope closes before Execute: RDG's breadcrumbs assert on one left open.)
    {
        RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelLandSolve, "VoxelLand solve");
        FImpl::FFrame F;
        I.Register(GraphBuilder, Fields, F);
        const FSettings& S = I.S;

        // Phases may follow each other within a frame; the loop stops at one that waits.
        for (int32 Guard = 0; Guard < 8 && !bFailed && !bFinished; Guard++)
        {
            const EPhase Before = I.Phase;
            switch (I.Phase)
            {
            case EPhase::Start:
            {
                I.Single<FVoxelSolveResetCS>(GraphBuilder, F, TEXT("reset"), 1);
                if (S.bErode)
                {
                    I.CountDomainSlots(GraphBuilder, F, 0, Result);
                }
                else
                {
                    I.Pass<FVoxelSolveNormaliseCS>(GraphBuilder, F, I.NumCells, TEXT("plate heights"));
                    I.CountDomainSlots(GraphBuilder, F, 1, Result);
                }
                break;
            }
            case EPhase::CountWait:
            {
                if (I.Pending.Num() == 0 || !I.Pending[0].IsReady()) break;
                TArray<uint32> Words;
                if (!VLS::Read(I.Pending[0], Words) || Words.Num() < 1)
                {
                    bFailed = true;
                    break;
                }
                I.Pending.Reset();
                I.NumSlots = Words[0];
                I.RegisterSlots(GraphBuilder, F);
                if (I.CountDomain == 0)
                {
                    Result.SolvedCells = (int32)I.NumSlots;
                    I.SolvedSlots = I.NumSlots;
                    if (I.NumSlots == 0)
                    {
                        // No mountains: the static heights.
                        I.Pass<FVoxelSolveNormaliseCS>(GraphBuilder, F, I.NumCells, TEXT("normalise"));
                        I.CountDomainSlots(GraphBuilder, F, 1, Result);
                        break;
                    }
                    I.Pass<FVoxelSolveInitCS>(GraphBuilder, F, I.NumCells, TEXT("init"));
                    I.bSolved = true;
                    I.SetPhase(EPhase::InitFill, Result);
                }
                else
                {
                    Result.LandCells = (int32)I.NumSlots;
                    if (I.UseTreeStart() && I.NumSlots > 0 && S.bRouteAhead && !I.bRouteByLoops)
                    {
                        // S1: the whole routing now, checked when the fields come back.
                        I.RecordRoutingAhead(GraphBuilder, F, Result);
                        Progress.store(0.95f, std::memory_order_relaxed);
                        break;
                    }
                    // After a solve, the lake levels start from its last receivers' tree fill.
                    I.Pass<FVoxelSolveRouteInitCS>(GraphBuilder, F, I.NumCells, TEXT("route init"), I.UseTreeStart() ? 2u : 0u);
                    if (I.UseTreeStart() && I.NumSlots > 0)
                        I.RouteTreeFill(GraphBuilder, F, 0);
                    I.SetPhase(EPhase::Fill0, Result);
                }
                break;
            }
            case EPhase::InitFill:
            {
                if (!I.Loop<FVoxelSolveFillTiledCS, true>(GraphBuilder, F, TEXT("initial fill"), 1, bFailed)) break;
                if (bFailed) break;
                I.Pass<FVoxelSolveUndrainedCS>(GraphBuilder, F, I.NumSlots, TEXT("undrained"));
                I.Pass<FVoxelSolveReceiversCS>(GraphBuilder, F, I.NumSlots, TEXT("receivers"), 1);
                I.SetPhase(EPhase::Steps, Result);
                break;
            }
            case EPhase::Steps:
            {
                bool bDone = false;
                while (I.Pending.Num() > 0 && I.Pending[0].IsReady())
                {
                    TArray<uint32> Words;
                    if (!VLS::Read(I.Pending[0], Words) || Words.Num() < 2)
                    {
                        bFailed = true;
                        break;
                    }
                    I.Pending.RemoveAt(0);
                    Progress.store(0.05f + 0.75f * FMath::Clamp((float)Words[1] / (float)FMath::Max(S.MaxSteps, 1), 0.0f, 1.0f), std::memory_order_relaxed);
                    bDone |= Words[0] != 0u;
                }
                if (bFailed) break;
                if (bDone)
                {
                    I.Pending.Reset();
                    // The outlets, and the histograms of the relief above them.
                    I.Pass<FVoxelSolveOutletInitCS>(GraphBuilder, F, I.NumSlots, TEXT("outlets"));
                    for (int32 r = 0; r < VLS::JumpRounds; r++)
                    {
                        I.Pass<FVoxelSolveOutletJumpCS>(GraphBuilder, F, I.NumSlots, TEXT("outlet jump"));
                        I.JumpParity ^= 1;
                    }
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.Histogram), 0u);
                    I.Pass<FVoxelSolveHistogramCS>(GraphBuilder, F, I.NumSlots, TEXT("histograms"));
                    I.Pending.Add(VLS::Enqueue(GraphBuilder, F.Histogram, 2 * VLS::HistogramBins * 4, TEXT("Voxel.LandSolveHistogramReadback")));
                    I.Pending.Add(VLS::Enqueue(GraphBuilder, F.State, 8, TEXT("Voxel.LandSolveStateReadback")));
                    I.Pending.Add(VLS::Enqueue(GraphBuilder, F.Log, (uint32)FMath::Max(S.MaxSteps, 1) * 4, TEXT("Voxel.LandSolveLogReadback")));
                    I.Pending.Add(VLS::Enqueue(GraphBuilder, F.Flags, 8, TEXT("Voxel.LandSolveFlagsReadback")));
                    I.SetPhase(EPhase::OutletsWait, Result);
                    break;
                }
                if (I.Pending.Num() >= VLS::MaxPending) break;
                const int32 Left = S.MaxSteps + 2 - I.StepsDispatched;
                if (Left <= 0)
                {
                    // Every step is sent (the last sets done): read the state again.
                    I.Pending.Add(VLS::Enqueue(GraphBuilder, F.State, 8, TEXT("Voxel.LandSolveStateReadback")));
                    break;
                }
                // The slope cap (T3g) adds about a third to a step's passes.
                const int64 StepCost = S.ChannelCells > 0.0f ? (int64)I.NumSlots * 4 / 3 : (int64)I.NumSlots;
                const int32 Steps = FMath::Min((int32)FMath::Clamp<int64>(S.StepCellsPerFrame / FMath::Max<int64>(StepCost, 1), 1, VLS::MaxStepsPerFrame), Left);
                for (int32 s = 0; s < Steps; s++)
                    I.Step(GraphBuilder, F);
                I.StepsDispatched += Steps;
                I.Pending.Add(VLS::Enqueue(GraphBuilder, F.State, 8, TEXT("Voxel.LandSolveStateReadback")));
                break;
            }
            case EPhase::OutletsWait:
            {
                if (I.Pending.Num() < 4 || !I.Pending[0].IsReady() || !I.Pending[1].IsReady() || !I.Pending[2].IsReady() || !I.Pending[3].IsReady()) break;
                TArray<uint32> Bins, StateWords, FlagWords;
                TArray<float> Residuals;
                if (!VLS::Read(I.Pending[0], Bins) || !VLS::Read(I.Pending[1], StateWords) || !VLS::Read(I.Pending[2], Residuals) || !VLS::Read(I.Pending[3], FlagWords))
                {
                    bFailed = true;
                    break;
                }
                I.Pending.Reset();
                Result.Steps = (int32)FMath::Min<uint32>(StateWords[1], (uint32)S.MaxSteps);
                Result.Residual = Result.Steps > 0 ? Residuals[Result.Steps - 1] : 0.0f;
                I.Errors |= FlagWords[1];
                const float SolvedP90 = VLS::Quantile(Bins.GetData(), 0.9);
                const float TargetP90 = VLS::Quantile(Bins.GetData() + VLS::HistogramBins, 0.9);
                const float TargetTop = VLS::Quantile(Bins.GetData() + VLS::HistogramBins, 0.999);
                if (TargetP90 >= 1.0f && SolvedP90 >= 1.0f)
                {
                    I.Factor = FMath::Clamp(TargetP90 / SolvedP90, 0.01f, 100.0f);
                    I.Knee = TargetP90;
                    I.Top = FMath::Max(S.ReliefTopScale * TargetTop, 1.2f * TargetP90);
                    // Above the knee a logarithm, K + s ln(1 + (R - K) / s) (VoxelSolveCompressRelief),
                    // with s fitted so the solved 99.9th percentile lands on the top: s ln(1 + D / s)
                    // rises from 0 to D with s, so any top is reached (a large s: nearly linear).
                    const float SolvedTop = VLS::Quantile(Bins.GetData(), 0.999);
                    const float D = I.Factor * SolvedTop - I.Knee;
                    const float Want = I.Top - I.Knee;
                    I.ReliefScale = 1.0e9f;
                    if (D > Want && Want > 0.0f)
                    {
                        double Lo = 1.0e-3 * Want, Hi = 1.0e3 * FMath::Max((double)D, 1.0);
                        for (int32 Iteration = 0; Iteration < 80; Iteration++)
                        {
                            const double Mid = FMath::Sqrt(Lo * Hi);
                            (Mid * FMath::Loge(1.0 + D / Mid) > Want ? Hi : Lo) = Mid;
                        }
                        I.ReliefScale = (float)FMath::Sqrt(Lo * Hi);
                    }
                }
                Result.Factor = I.Factor;
                Result.ReliefKnee = I.Knee;
                Result.ReliefTop = I.Top;
                Result.ReliefScale = I.ReliefScale;
                // The links already uphill (lakes behind sills) before the normalisation; after it,
                // no new step uphill along the last step's receivers (VoxelSolveSettlePass): a tree
                // fill that stops at those links, then each solved cell up to its level.
                I.Pass<FVoxelSolveSettleMarkCS>(GraphBuilder, F, I.NumSlots, TEXT("settle mark"));
                I.Pass<FVoxelSolveNormaliseCS>(GraphBuilder, F, I.NumCells, TEXT("normalise"));
                I.Pass<FVoxelSolveSettleInitCS>(GraphBuilder, F, I.NumSlots, TEXT("settle fill"));
                for (int32 r = 0; r < VLS::JumpRounds; r++)
                {
                    I.Pass<FVoxelSolveTreeFillJumpCS>(GraphBuilder, F, I.NumSlots, TEXT("settle fill jump"), 0, 0);
                    I.JumpParity ^= 1;
                }
                I.Pass<FVoxelSolveTreeFillFinishCS>(GraphBuilder, F, I.NumSlots, TEXT("settle fill finish"), 0, 0);
                I.Pass<FVoxelSolveSettleCS>(GraphBuilder, F, I.NumSlots, TEXT("settle"));
                Progress.store(0.85f, std::memory_order_relaxed);
                I.CountDomainSlots(GraphBuilder, F, 1, Result);
                break;
            }
            case EPhase::Fill0:
            {
                if (I.NumSlots > 0 && !I.Loop<FVoxelSolveFillTiledCS, true>(GraphBuilder, F, TEXT("lake levels"), 0, bFailed)) break;
                if (bFailed) break;
                I.Pass<FVoxelSolveRouteInitCS>(GraphBuilder, F, I.NumCells, TEXT("route init"), I.UseTreeStart() ? 3u : 1u);
                if (I.UseTreeStart() && I.NumSlots > 0)
                    I.RouteTreeFill(GraphBuilder, F, 1);
                I.SetPhase(EPhase::Fill1, Result);
                Progress.store(0.88f, std::memory_order_relaxed);
                break;
            }
            case EPhase::Fill1:
            {
                if (I.NumSlots > 0 && !I.Loop<FVoxelSolveFillTiledCS, true>(GraphBuilder, F, TEXT("routing fill"), 1, bFailed)) break;
                if (bFailed) break;
                I.Pass<FVoxelSolveReceiversCS>(GraphBuilder, F, I.NumSlots, TEXT("receivers"), 0);
                // S1: the drainage, exact in fixed passes (it was iterated to convergence: about a
                // pass per cell of the longest path).
                if (I.NumSlots > 0)
                    I.AccumulateByJumps(GraphBuilder, F, 0);
                // The drainage field's spread over the hillslopes (VoxelSolveSpreadPass), from the
                // D8 areas; the solve and the lakes keep D8.
                if (S.SpreadCells > 0.0f && I.NumSlots > 0)
                {
                    I.Pass<FVoxelSolveSpreadInitCS>(GraphBuilder, F, I.NumSlots, TEXT("drainage spread init"));
                    I.SetPhase(EPhase::Spread, Result);
                    Progress.store(0.92f, std::memory_order_relaxed);
                    break;
                }
                I.Pass<FVoxelSolveLabelInitCS>(GraphBuilder, F, I.NumSlots, TEXT("lake labels"));
                I.SetPhase(EPhase::Labels, Result);
                Progress.store(0.94f, std::memory_order_relaxed);
                break;
            }
            case EPhase::Spread:
            {
                if (!I.Loop<FVoxelSolveSpreadCS>(GraphBuilder, F, TEXT("drainage spread"), 0, bFailed)) break;
                if (bFailed) break;
                // S1: the hillslopes converged (a few dozen passes); the channels sum them down the
                // tree in fixed passes.
                I.AccumulateByJumps(GraphBuilder, F, 1);
                I.Pass<FVoxelSolveLabelInitCS>(GraphBuilder, F, I.NumSlots, TEXT("lake labels"));
                I.SetPhase(EPhase::Labels, Result);
                Progress.store(0.94f, std::memory_order_relaxed);
                break;
            }
            case EPhase::Labels:
            {
                if (I.NumSlots > 0 && !I.Loop<FVoxelSolveLabelCS>(GraphBuilder, F, TEXT("lake labels"), 0, bFailed)) break;
                if (bFailed) break;
                // Lakes, then the fields; read back once.
                I.RecordOutput(GraphBuilder, F, Result);
                Progress.store(0.97f, std::memory_order_relaxed);
                break;
            }
            case EPhase::OutputWait:
            {
                bool bReady = I.Pending.Num() == 9 && (!I.Rivers.IsValid() || I.Rivers.IsReady());
                for (const VLS::FReadback& R : I.Pending)
                    bReady &= R.IsReady();
                for (const VLS::FReadback& R : I.Checks)
                    bReady &= R.IsReady();
                if (I.Checks.Num() > 0)
                    bReady &= I.PreRouting.IsReady();
                if (!bReady) break;
                if (I.Checks.Num() > 0)
                {
                    // S1: the routing recorded ahead: every loop had settled, or it routes again by
                    // loops (from INF: the routing replaced the solve's receivers).
                    FString Unsettled;
                    static const TCHAR* CheckNames[] = { TEXT("lake levels"), TEXT("routing fill"), TEXT("drainage spread"), TEXT("lake labels") };
                    const bool bSpread = I.Checks.Num() == 4;
                    TArray<uint32> PreWords;
                    if (!VLS::Read(I.PreRouting, PreWords) || PreWords.Num() < 2)
                    {
                        bFailed = true;
                        break;
                    }
                    for (int32 k = 0; k < I.Checks.Num(); k++)
                    {
                        TArray<uint32> Words;
                        if (!VLS::Read(I.Checks[k], Words) || Words.Num() < 2)
                        {
                            bFailed = true;
                            break;
                        }
                        if (Words[0] != 0u)
                            Unsettled += FString(Unsettled.IsEmpty() ? TEXT("") : TEXT(", ")) + CheckNames[(!bSpread && k == 2) ? 3 : k];
                    }
                    I.Checks.Reset();
                    if (bFailed) break;
                    if (!Unsettled.IsEmpty())
                    {
                        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] The routing recorded ahead hadn't settled (%s); routing again by loops."), *Unsettled);
                        I.Pending.Reset();
                        I.Rivers.Reset();
                        I.KeptHeight.SafeRelease();
                        I.KeptWater.SafeRelease();
                        I.KeptFlow.SafeRelease();
                        I.KeptRec.SafeRelease();
                        I.bRouteByLoops = true;
                        // Back to the state the routing started from: the heights (the lakes not
                        // kept were filled into them), the solve's receivers, the errors so far
                        // (the unsettled attempt's are dropped, with the flags).
                        AddCopyBufferPass(GraphBuilder, F.H, GraphBuilder.RegisterExternalBuffer(I.KeptH));
                        AddCopyBufferPass(GraphBuilder, F.Rec, GraphBuilder.RegisterExternalBuffer(I.KeptSolveRec));
                        I.Errors = I.ErrorsBeforeRouting | PreWords[1];
                        I.Single<FVoxelSolveResetCS>(GraphBuilder, F, TEXT("reset"), 1);
                        // The rivers used the slot buffer as scratch: count the land again.
                        I.CountDomainSlots(GraphBuilder, F, 1, Result);
                        break;
                    }
                }
                TArray<float> Heights, Waters;
                TArray<uint32> Flows, Attrs, LakeCount, LakeWords, FlagWords, Forms, Recs;
                if (!VLS::Read(I.Pending[0], Heights) || !VLS::Read(I.Pending[1], Flows) || !VLS::Read(I.Pending[2], Waters) || !VLS::Read(I.Pending[3], Attrs)
                    || !VLS::Read(I.Pending[4], LakeCount) || !VLS::Read(I.Pending[5], LakeWords) || !VLS::Read(I.Pending[6], FlagWords)
                    || !VLS::Read(I.Pending[7], Forms) || !VLS::Read(I.Pending[8], Recs))
                {
                    bFailed = true;
                    break;
                }
                I.Pending.Reset();
                I.Errors |= FlagWords[1];
                // Only copies here: the baker narrows and checks them on a worker.
                Result.Height = MoveTemp(Heights);
                Result.Water = MoveTemp(Waters);
                Result.Attr = MoveTemp(Attrs);
                Result.FlowHalves = MoveTemp(Flows);
                Result.Form = MoveTemp(Forms);
                Result.Receiver = MoveTemp(Recs);
                Result.LakesFound = (int32)LakeCount[0];
                const int32 Kept = FMath::Min<int32>(Result.LakesFound, (int32)VLS::MaxLakes);
                const double CellAreaKm2 = 1.0e-2;   // the list's area unit (1e4 m^2) in km^2
                Result.Lakes.SetNum(Kept);
                for (int32 k = 0; k < Kept; k++)
                {
                    FVoxelLandLake& Lake = Result.Lakes[k];
                    Lake.Cell = LakeWords[k * 4];
                    FMemory::Memcpy(&Lake.Level, &LakeWords[k * 4 + 1], sizeof(float));
                    Lake.Outlet = LakeWords[k * 4 + 2];
                    Lake.AreaKm2 = (float)((double)LakeWords[k * 4 + 3] * CellAreaKm2);
                }
                // In the order of their ids, whatever order the GPU appended them in.
                Result.Lakes.Sort([](const FVoxelLandLake& A, const FVoxelLandLake& B) { return A.Cell < B.Cell; });
                if (Result.LakesFound > Kept)
                {
                    UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] %d lakes found, %d kept in the list (the rest are still water in the bake)."), Result.LakesFound, Kept);
                }
                Result.Errors = I.Errors;
                if (I.Rivers.IsValid() && !I.FinishRivers(GraphBuilder, F, Result))
                {
                    I.SetPhase(EPhase::RiverWait, Result);
                    break;
                }
                I.Finish(Result);
                I.SetPhase(EPhase::Done, Result);
                I.Release();
                Progress.store(1.0f, std::memory_order_relaxed);
                bFinished = true;
                break;
            }
            case EPhase::RiverWait:
            {
                if (!I.Rivers.IsReady()) break;
                if (!I.FinishRivers(GraphBuilder, F, Result)) break;
                I.Finish(Result);
                I.SetPhase(EPhase::Done, Result);
                I.Release();
                Progress.store(1.0f, std::memory_order_relaxed);
                bFinished = true;
                break;
            }
            case EPhase::Done:
                break;
            }
            if (I.Phase == Before) break;
        }
    }
    GraphBuilder.Execute();
    if (bFailed)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] A solve readback failed."));
        I.Release();
        Status.store(-1, std::memory_order_release);
        return;
    }
    if (bFinished)
        Status.store(1, std::memory_order_release);
}
