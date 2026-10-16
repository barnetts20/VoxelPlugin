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
    // Lakes and outputs (the last frame; stand-ins before).
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeArea)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeDepth)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeOutlet)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeCount)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, LakeList)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutHeight)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutFlow)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutWater)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutAttr)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutForm)
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
    X(FVoxelSolveFillInPlaceCS, "FillInPlaceCS") \
    X(FVoxelSolveFillJacobiCS, "FillJacobiCS") \
    X(FVoxelSolveFillCopyCS, "FillCopyCS") \
    X(FVoxelSolveUndrainedCS, "UndrainedCS") \
    X(FVoxelSolveReceiversCS, "ReceiversCS") \
    X(FVoxelSolveAccumulateJacobiCS, "AccumulateJacobiCS") \
    X(FVoxelSolveAccumulateInPlaceCS, "AccumulateInPlaceCS") \
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
        Fill1,          // the routing's fill
        Accumulate,     // the routing's drainage
        Spread,         // the drainage field's spread (T1b)
        Labels,         // lake labels
        OutputWait,     // the fields
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
    float Factor = 1.0f;
    float Knee = 1.0e30f;
    float Top = 1.0e30f;
    float ReliefScale = 0.0f;
    uint32 Errors = 0;
    double PhaseStart = 0.0;
    int32 PhaseFrames = 0;
    TArray<VLS::FReadback> Pending;

    // Pooled buffers kept between frames (render thread).
    TRefCountPtr<FRDGPooledBuffer> Tan, H, W, Water, Rec, Slot, Label, Cell, GroupCounts, Counts, Flags, State, Partials, Log, Histogram;
    TRefCountPtr<FRDGPooledBuffer> U, Erodibility, ChannelFactor, Area, Dist, H0, WOut;
    TRefCountPtr<FRDGPooledBuffer> A[2], X[2], Y[2], P[2];

    /** This frame's view of the buffers. */
    struct FFrame
    {
        FRDGBufferRef BakeHeight = nullptr, BakeUp = nullptr, BakeCrust = nullptr, BakeAttr = nullptr, BakeForm = nullptr;
        FRDGBufferRef Tan = nullptr, H = nullptr, W = nullptr, Water = nullptr, Rec = nullptr, Slot = nullptr, Label = nullptr, Cell = nullptr;
        FRDGBufferRef GroupCounts = nullptr, Counts = nullptr, Flags = nullptr, State = nullptr, Partials = nullptr, Log = nullptr, Histogram = nullptr;
        FRDGBufferRef U = nullptr, Erodibility = nullptr, ChannelFactor = nullptr, Area = nullptr, Dist = nullptr, H0 = nullptr, WOut = nullptr;
        FRDGBufferRef A[2] = {}, X[2] = {}, Y[2] = {}, P[2] = {};
        // The last frame's lake and output buffers (else stand-ins).
        FRDGBufferRef LakeArea = nullptr, LakeDepth = nullptr, LakeOutlet = nullptr, LakeCount = nullptr, LakeList = nullptr;
        FRDGBufferRef OutHeight = nullptr, OutFlow = nullptr, OutWater = nullptr, OutAttr = nullptr, OutForm = nullptr;
        /** Every pass's parameters with this frame's views (BuildBase), and the views of the
         *  double buffers, which the passes pick by parity. */
        FVoxelLandSolveParameters Base;
        FRDGBufferUAVRef AUav[2] = {}, XUav[2] = {}, YUav[2] = {}, PUav[2] = {};
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
            TEXT("lake levels"), TEXT("routing fill"), TEXT("drainage"), TEXT("drainage spread"), TEXT("lake labels"), TEXT("output"), TEXT("done") };
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
     *  bOutFailed). */
    template<typename TShader>
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
        const int32 K = (int32)FMath::Clamp<int64>(S.IterationCellsPerFrame / FMath::Max<int64>(NumSlots, 1), VLS::MinIterationsPerFrame, VLS::MaxIterationsPerFrame);
        for (int32 k = 0; k < K; k++)
        {
            if (k == K - 1)
                Single<FVoxelSolveResetCS>(GraphBuilder, F, TEXT("reset flag"));
            Pass<TShader>(GraphBuilder, F, NumSlots, Name, PassMode);
        }
        LoopIterations += K;
        Pending.Add(VLS::Enqueue(GraphBuilder, F.Flags, 8, TEXT("Voxel.LandSolveFlagsReadback")));
        return false;
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
            &Histogram, &U, &Erodibility, &ChannelFactor, &Area, &Dist, &H0, &WOut, &A[0], &A[1], &X[0], &X[1], &Y[0], &Y[1], &P[0], &P[1] })
            B->SafeRelease();
        Pending.Reset();
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
    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelLandSolve, "VoxelLand solve");
    FImpl::FFrame F;
    I.Register(GraphBuilder, Fields, F);
    const FSettings& S = I.S;
    bool bFailed = false;
    bool bFinished = false;

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
                if (I.NumSlots == 0)
                {
                    // No mountains: the static heights.
                    I.Pass<FVoxelSolveNormaliseCS>(GraphBuilder, F, I.NumCells, TEXT("normalise"));
                    I.CountDomainSlots(GraphBuilder, F, 1, Result);
                    break;
                }
                I.Pass<FVoxelSolveInitCS>(GraphBuilder, F, I.NumCells, TEXT("init"));
                I.SetPhase(EPhase::InitFill, Result);
            }
            else
            {
                Result.LandCells = (int32)I.NumSlots;
                I.Pass<FVoxelSolveRouteInitCS>(GraphBuilder, F, I.NumCells, TEXT("route init"), 0);
                I.SetPhase(EPhase::Fill0, Result);
            }
            break;
        }
        case EPhase::InitFill:
        {
            if (!I.Loop<FVoxelSolveFillInPlaceCS>(GraphBuilder, F, TEXT("initial fill"), 1, bFailed)) break;
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
            if (I.NumSlots > 0 && !I.Loop<FVoxelSolveFillInPlaceCS>(GraphBuilder, F, TEXT("lake levels"), 0, bFailed)) break;
            if (bFailed) break;
            I.Pass<FVoxelSolveRouteInitCS>(GraphBuilder, F, I.NumCells, TEXT("route init"), 1);
            I.SetPhase(EPhase::Fill1, Result);
            Progress.store(0.88f, std::memory_order_relaxed);
            break;
        }
        case EPhase::Fill1:
        {
            if (I.NumSlots > 0 && !I.Loop<FVoxelSolveFillInPlaceCS>(GraphBuilder, F, TEXT("routing fill"), 1, bFailed)) break;
            if (bFailed) break;
            I.Pass<FVoxelSolveReceiversCS>(GraphBuilder, F, I.NumSlots, TEXT("receivers"), 0);
            I.SetPhase(EPhase::Accumulate, Result);
            Progress.store(0.91f, std::memory_order_relaxed);
            break;
        }
        case EPhase::Accumulate:
        {
            if (I.NumSlots > 0 && !I.Loop<FVoxelSolveAccumulateInPlaceCS>(GraphBuilder, F, TEXT("drainage"), 0, bFailed)) break;
            if (bFailed) break;
            // The drainage field's spread over the hillslopes (VoxelSolveSpreadPass), from the
            // D8 areas just converged; the solve and the lakes keep D8.
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
            F.LakeArea = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveLakeArea"));
            F.LakeDepth = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveLakeDepth"));
            F.LakeOutlet = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveLakeOutlet"));
            F.LakeCount = FImpl::Transient(GraphBuilder, 4, 1, TEXT("Voxel.LandSolveLakeCount"));
            F.LakeList = FImpl::Transient(GraphBuilder, 4, 4 * VLS::MaxLakes, TEXT("Voxel.LandSolveLakeList"));
            F.OutHeight = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveOutHeight"));
            F.OutFlow = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveOutFlow"));
            F.OutWater = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveOutWater"));
            F.OutAttr = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveOutAttr"));
            F.OutForm = FImpl::Transient(GraphBuilder, 4, I.NumCells, TEXT("Voxel.LandSolveOutForm"));
            I.BuildBase(GraphBuilder, F);
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeArea), 0u);
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeDepth), 0u);
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeOutlet), VLS::None);
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeCount), 0u);
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(F.LakeList), 0u);
            I.Pass<FVoxelSolveLakeStatsCS>(GraphBuilder, F, I.NumSlots, TEXT("lake stats"));
            I.Pass<FVoxelSolveLakeFinishCS>(GraphBuilder, F, I.NumSlots, TEXT("lakes"));
            I.Pass<FVoxelSolveOutputCS>(GraphBuilder, F, I.NumCells, TEXT("output"));
            const uint32 Bytes = I.NumCells * 4;
            I.Pending.Reset();
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.OutHeight, Bytes, TEXT("Voxel.LandSolveHeightReadback")));
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.OutFlow, Bytes, TEXT("Voxel.LandSolveFlowReadback")));
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.OutWater, Bytes, TEXT("Voxel.LandSolveWaterReadback")));
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.OutAttr, Bytes, TEXT("Voxel.LandSolveAttrReadback")));
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.LakeCount, 4, TEXT("Voxel.LandSolveLakeCountReadback")));
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.LakeList, 16 * VLS::MaxLakes, TEXT("Voxel.LandSolveLakesReadback")));
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.Flags, 8, TEXT("Voxel.LandSolveFlagsReadback")));
            // The plate pass's landform, its character following the drainage density (T3h).
            I.Pending.Add(VLS::Enqueue(GraphBuilder, F.OutForm, Bytes, TEXT("Voxel.LandSolveFormReadback")));
            I.SetPhase(EPhase::OutputWait, Result);
            Progress.store(0.97f, std::memory_order_relaxed);
            break;
        }
        case EPhase::OutputWait:
        {
            bool bReady = I.Pending.Num() == 8;
            for (const VLS::FReadback& R : I.Pending)
                bReady &= R.IsReady();
            if (!bReady) break;
            TArray<float> Heights, Flows, Waters;
            TArray<uint32> Attrs, LakeCount, LakeWords, FlagWords, Forms;
            if (!VLS::Read(I.Pending[0], Heights) || !VLS::Read(I.Pending[1], Flows) || !VLS::Read(I.Pending[2], Waters) || !VLS::Read(I.Pending[3], Attrs)
                || !VLS::Read(I.Pending[4], LakeCount) || !VLS::Read(I.Pending[5], LakeWords) || !VLS::Read(I.Pending[6], FlagWords)
                || !VLS::Read(I.Pending[7], Forms))
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
            Result.Flow = MoveTemp(Flows);
            Result.Form = MoveTemp(Forms);
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
