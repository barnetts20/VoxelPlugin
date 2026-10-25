#include "VoxelOceanGPUTime.h"
#include "VoxelPlugin.h"
#include "DynamicRHI.h"
#include "RHI.h"
#include "RHICommandList.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "GlobalShader.h"
#include "ShaderParameterStruct.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "RenderingThread.h"
#include <atomic>

namespace VoxelOceanGPUTime
{
    struct FSpan
    {
        uint32 Frame = 0;
        int32 Group = 0;
        FRenderQueryRHIRef BeginQuery;
        FRenderQueryRHIRef EndQuery;
    };
}

// Stage W8g: the reference workload (VoxelOceanGPUTimeRef.usf).
class FVoxelOceanGPUTimeRefCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanGPUTimeRefCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanGPUTimeRefCS, FGlobalShader);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, RefOut)
        SHADER_PARAMETER(uint32, RefIterations)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanGPUTimeRefCS, "/Plugin/VoxelPlugin/Private/VoxelOceanGPUTimeRef.usf", "GPUTimeRefCS", SF_Compute);

namespace VoxelOceanGPUTimeDetail
{
    using namespace VoxelOceanGPUTime;

    /** The reference's size: 512 groups of 64 threads, 1024 steps each (about 0.1-0.3 ms). */
    constexpr int32 RefGroups = 512;
    constexpr uint32 RefIterations = 1024u;

    /** Frames a span waits before it's read (the graph that holds it has run by then). */
    constexpr uint32 ReadAfterFrames = 3;
    /** A span not resolved by then is dropped with its frame (a device lost, a query never run). */
    constexpr uint32 DropAfterFrames = 120;
    constexpr int32 MaxKept = 512;

    struct FFrameSum
    {
        float Ms[NumGroups] = {};
        bool bBad = false;
    };

    // Behind Lock (the graphs are made on the render thread, but a mesh provider's may not be),
    // Done behind its own. Never destructed: nothing of the RHI's is released after the RHI at exit.
    struct FState
    {
        FCriticalSection Lock;
        TArray<TSharedPtr<FSpan>> Pending;
        TArray<FRenderQueryRHIRef> Free;
        TMap<uint32, FFrameSum> Sums;
        FCriticalSection DoneLock;
        TArray<FFrameTimes> Done;
    };

    static FState& State()
    {
        static FState* S = new FState();
        return *S;
    }

    static std::atomic<bool> GEnabled{ false };

    static FRenderQueryRHIRef Allocate()
    {
        FState& S = State();
        if (S.Free.Num() > 0) return S.Free.Pop(EAllowShrinking::No);
        return RHICreateRenderQuery(RQT_AbsoluteTime);
    }

    static void AddTimestamp(FRDGBuilder& GraphBuilder, const FRenderQueryRHIRef& Query)
    {
        // No parameters: never culled, run where the graph puts it among the ocean's passes.
        GraphBuilder.AddPass(RDG_EVENT_NAME("VoxelOceanGPUTime"), ERDGPassFlags::None,
            [Query](FRHICommandListImmediate& RHICmdList)
            {
                RHICmdList.EndRenderQuery(Query);
            });
    }

    /** The spans that have come back, summed into their frames; the frames no span can join any
     *  more published. */
    static void Poll()
    {
        FState& S = State();
        const uint32 Now = GFrameNumberRenderThread;
        for (int32 i = S.Pending.Num() - 1; i >= 0; i--)
        {
            FSpan& Span = *S.Pending[i];
            if (!Span.EndQuery.IsValid() || Now < Span.Frame + ReadAfterFrames)
            {
                // Still open, or too young. An open span that old was never closed.
                if (Now > Span.Frame + DropAfterFrames)
                {
                    S.Sums.FindOrAdd(Span.Frame).bBad = true;
                    S.Pending.RemoveAtSwap(i, EAllowShrinking::No);
                }
                continue;
            }
            uint64 BeginUs = 0, EndUs = 0;
            const bool bBegin = RHIGetRenderQueryResult(Span.BeginQuery.GetReference(), BeginUs, false);
            const bool bEnd = bBegin && RHIGetRenderQueryResult(Span.EndQuery.GetReference(), EndUs, false);
            if (!bEnd)
            {
                if (Now > Span.Frame + DropAfterFrames)
                {
                    S.Sums.FindOrAdd(Span.Frame).bBad = true;
                    S.Pending.RemoveAtSwap(i, EAllowShrinking::No);
                }
                continue;
            }
            // RQT_AbsoluteTime reads in microseconds.
            FFrameSum& Sum = S.Sums.FindOrAdd(Span.Frame);
            Sum.Ms[Span.Group] += EndUs > BeginUs ? (float)((double)(EndUs - BeginUs) * 0.001) : 0.0f;
            S.Free.Add(MoveTemp(Span.BeginQuery));
            S.Free.Add(MoveTemp(Span.EndQuery));
            S.Pending.RemoveAtSwap(i, EAllowShrinking::No);
        }
        // A frame is whole once it's old enough for no new span and none of its own is pending.
        TArray<uint32> Ready;
        for (const TPair<uint32, FFrameSum>& It : S.Sums)
        {
            if (Now < It.Key + ReadAfterFrames + 1) continue;
            bool bWaiting = false;
            for (const TSharedPtr<FSpan>& P : S.Pending)
                if (P->Frame == It.Key) { bWaiting = true; break; }
            if (!bWaiting) Ready.Add(It.Key);
        }
        if (Ready.Num() == 0) return;
        Ready.Sort();
        FScopeLock Lock(&S.DoneLock);
        for (uint32 F : Ready)
        {
            const FFrameSum Sum = S.Sums.FindAndRemoveChecked(F);
            if (Sum.bBad) continue;
            FFrameTimes T;
            T.Frame = F;
            for (int32 g = 0; g < NumGroups; g++)
            {
                T.Ms[g] = Sum.Ms[g];
                if (g < NumOceanGroups) T.Total += Sum.Ms[g];
            }
            S.Done.Add(T);
        }
        if (S.Done.Num() > MaxKept)
            S.Done.RemoveAt(0, S.Done.Num() - MaxKept);
    }

    static void Release_RenderThread()
    {
        FState& S = State();
        FScopeLock Lock(&S.Lock);
        S.Pending.Reset();
        S.Free.Reset();
        S.Sums.Reset();
    }
}

namespace VoxelOceanGPUTime
{
    const TCHAR* GroupName(int32 Group)
    {
        static const TCHAR* Names[NumGroups] = {
            TEXT("cascades"), TEXT("clipmap"), TEXT("mesh"), TEXT("surface"), TEXT("spray"), TEXT("underwater"), TEXT("other"), TEXT("behind"), TEXT("reference") };
        return Group >= 0 && Group < NumGroups ? Names[Group] : TEXT("?");
    }

    void SetEnabled(bool bOn)
    {
        using namespace VoxelOceanGPUTimeDetail;
        const bool bWas = GEnabled.exchange(bOn);
        if (bWas == bOn) return;
        {
            FState& S = State();
            FScopeLock Lock(&S.DoneLock);
            S.Done.Reset();
        }
        if (!bOn)
        {
            // The queries let go behind every graph already made with them (their passes hold
            // their own references until the graph is gone).
            ENQUEUE_RENDER_COMMAND(VoxelOceanGPUTimeRelease)([](FRHICommandListImmediate&)
                {
                    if (!GEnabled.load()) Release_RenderThread();
                });
        }
    }

    bool IsEnabled()
    {
        return VoxelOceanGPUTimeDetail::GEnabled.load();
    }

    void Drain(TArray<FFrameTimes>& Out)
    {
        using namespace VoxelOceanGPUTimeDetail;
        FState& S = State();
        FScopeLock Lock(&S.DoneLock);
        Out.Append(S.Done);
        S.Done.Reset();
    }

    TSharedPtr<FSpan> Begin(FRDGBuilder& GraphBuilder, int32 Group)
    {
        using namespace VoxelOceanGPUTimeDetail;
        if (!GEnabled.load(std::memory_order_relaxed) || Group < 0 || Group >= NumGroups) return nullptr;
        FScopeLock Lock(&State().Lock);
        Poll();
        TSharedPtr<FSpan> Span = MakeShared<FSpan>();
        Span->Frame = GFrameNumberRenderThread;
        Span->Group = Group;
        Span->BeginQuery = Allocate();
        if (!Span->BeginQuery.IsValid()) return nullptr;
        AddTimestamp(GraphBuilder, Span->BeginQuery);
        State().Pending.Add(Span);
        return Span;
    }

    void End(FRDGBuilder& GraphBuilder, const TSharedPtr<FSpan>& Span)
    {
        using namespace VoxelOceanGPUTimeDetail;
        if (!Span.IsValid() || Span->EndQuery.IsValid()) return;
        FScopeLock Lock(&State().Lock);
        FRenderQueryRHIRef Query = Allocate();
        if (!Query.IsValid()) return;
        AddTimestamp(GraphBuilder, Query);
        Span->EndQuery = MoveTemp(Query);
    }

    void AddReference(FRDGBuilder& GraphBuilder)
    {
        using namespace VoxelOceanGPUTimeDetail;
        if (!GEnabled.load(std::memory_order_relaxed)) return;
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        if (!ShaderMap) return;
        TShaderMapRef<FVoxelOceanGPUTimeRefCS> Shader(ShaderMap);
        if (!Shader.IsValid()) return;
        FScope Time(GraphBuilder, Reference);
        FRDGBufferRef Out = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), 1), TEXT("Voxel.OceanGPUTimeRef"));
        FVoxelOceanGPUTimeRefCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanGPUTimeRefCS::FParameters>();
        P->RefOut = GraphBuilder.CreateUAV(Out);
        P->RefIterations = RefIterations;
        // Nothing reads its output: never culled.
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanGPUTime reference"), ERDGPassFlags::Compute | ERDGPassFlags::NeverCull,
            Shader, P, FIntVector(RefGroups, 1, 1));
    }
}
