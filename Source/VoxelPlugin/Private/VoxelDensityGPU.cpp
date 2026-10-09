#include "VoxelDensityGPU.h"
#include "VoxelGPUCommon.h"
#include "VoxelPlugin.h"
#include "FSparseEditStore.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RHIGPUReadback.h"
#include "Async/Async.h"
#include "Misc/App.h"
#include <atomic>

/** Samples the density on a box of lattice points (VoxelDensityGPU.usf). Global shaders must
 *  register before the engine compiles them, hence the module's PostConfigInit loading phase. */
class FVoxelDensityCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelDensityCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelDensityCS, FGlobalShader);

    static constexpr int32 GroupSize = 4;

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOctave>, Octaves)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, OutDensity)
        SHADER_PARAMETER(FIntVector, BlockMin)
        SHADER_PARAMETER(FIntVector, BlockDim)
        SHADER_PARAMETER(int32, BlockStep)
        SHADER_PARAMETER(uint32, OutOffset)
        SHADER_PARAMETER(float, SampleFootprint)
        SHADER_PARAMETER(uint32, OutputMode)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, EditCorners)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, EditChildren)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditFlags)
        SHADER_PARAMETER(uint32, EditNodeCount)
        SHADER_PARAMETER(int32, EditRootExtent)
        SHADER_PARAMETER(FVector2f, HalfFinest)
        SHADER_PARAMETER(uint32, ContinentOctaves)
        SHADER_PARAMETER(uint32, MountainOctaves)
        SHADER_PARAMETER(uint32, DetailOctaves)
        SHADER_PARAMETER(FVector2f, InvNormContinent)
        SHADER_PARAMETER(FVector2f, InvNormMountain)
        SHADER_PARAMETER(FVector2f, InvNormDetail)
        SHADER_PARAMETER(FVector2f, PlanetRadius)
        SHADER_PARAMETER(FVector2f, NoiseAmplitude)
        SHADER_PARAMETER(FVector2f, ContinentWeight)
        SHADER_PARAMETER(FVector2f, MountainWeight)
        SHADER_PARAMETER(FVector2f, MountainStart)
        SHADER_PARAMETER(FVector2f, MountainBlend)
        SHADER_PARAMETER(FVector2f, DetailWeight)
        SHADER_PARAMETER(float, RidgeGain)
        SHADER_PARAMETER(float, SlopeDamping)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.SetDefine(TEXT("GROUP_SIZE"), GroupSize);
        // The double-float code needs exact float rounding (precise alone isn't honoured
        // everywhere, e.g. Metal's fast math).
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelDensityCS, "/Plugin/VoxelPlugin/Private/VoxelDensityGPU.usf", "MainCS", SF_Compute);

namespace VoxelDensityGPUDetail
{
    struct FReadbackState
    {
        TUniquePtr<FRHIGPUBufferReadback> Buffer;
        int32 Count = 0;
        std::atomic<bool> bDone{ false };
        TFunction<void(TArray<float>&&)> OnDone;
    };
}

void VoxelDensityGPU::SampleBlocksAsync(TSharedRef<const FVoxelDensityGPUData> Data, double HalfFinest,
    TArray<FVoxelDensityGPUBlock> Blocks, TFunction<void(TArray<float>&&)> OnDone,
    EVoxelDensityGPUOutput Output, TSharedPtr<const FVoxelEditGPUData> Edits)
{
    using namespace VoxelDensityGPUDetail;
    check(!IsInRenderingThread());

    const TSharedRef<FReadbackState, ESPMode::ThreadSafe> State = MakeShared<FReadbackState, ESPMode::ThreadSafe>();
    State->OnDone = MoveTemp(OnDone);
    for (const FVoxelDensityGPUBlock& Block : Blocks)
        State->Count += FMath::Max(Block.NumPoints(), 0);
    if (State->Count == 0 || !FApp::CanEverRender())
    {
        AsyncTask(ENamedThreads::GameThread, [State]() { State->OnDone(TArray<float>()); });
        return;
    }

    ENQUEUE_RENDER_COMMAND(VoxelDensityGPUSample)([Data, HalfFinest, Blocks = MoveTemp(Blocks), State, Output, Edits](FRHICommandListImmediate& RHICmdList)
        {
            // GetShader by type returns an empty reference when the shader isn't compiled for
            // this platform (TShaderMapRef would assert).
            if (!GetGlobalShaderMap(GMaxRHIFeatureLevel)->GetShader(&FVoxelDensityCS::GetStaticType(), 0).IsValid())
            {
                State->bDone.store(true, std::memory_order_release);
                AsyncTask(ENamedThreads::GameThread, [State]() { State->OnDone(TArray<float>()); });
                return;
            }
            const uint32 NumBytes = (uint32)State->Count * (uint32)sizeof(float);
            FRDGBuilder GraphBuilder(RHICmdList);

            FRDGBufferRef OutBuffer = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), State->Count), TEXT("Voxel.DensityOut"));
            FRDGBufferSRVRef OctaveSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.DensityOctaves"), MakeArrayView(Data->Octaves)));
            const FVoxelEditGPUData NoEdits;
            const FVoxelEditGPUData& EditData = Edits.IsValid() ? *Edits : NoEdits;
            FRDGBufferSRVRef EditCornersSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.DensityEditCorners"), MakeArrayView(EditData.Corners)));
            FRDGBufferSRVRef EditChildrenSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.DensityEditChildren"), MakeArrayView(EditData.Children)));
            FRDGBufferSRVRef EditFlagsSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.DensityEditFlags"), MakeArrayView(EditData.Flags)));
            // Blocks write disjoint ranges, so their passes need no barrier between them.
            FRDGBufferUAVRef OutUAV = GraphBuilder.CreateUAV(OutBuffer, ERDGUnorderedAccessViewFlags::SkipBarrier);
            TShaderMapRef<FVoxelDensityCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));

            uint32 Offset = 0;
            for (const FVoxelDensityGPUBlock& Block : Blocks)
            {
                if (Block.NumPoints() <= 0) continue;
                FVoxelDensityCS::FParameters* Parameters = GraphBuilder.AllocParameters<FVoxelDensityCS::FParameters>();
                Parameters->Octaves = OctaveSRV;
                Parameters->OutDensity = OutUAV;
                Parameters->BlockMin = Block.Min;
                Parameters->BlockDim = Block.Dim;
                Parameters->BlockStep = Block.Step;
                Parameters->OutOffset = Offset;
                Parameters->SampleFootprint = (float)FMath::Max(Block.Footprint, Data->MinFootprint);
                Parameters->OutputMode = (uint32)Output;
                Parameters->EditCorners = EditCornersSRV;
                Parameters->EditChildren = EditChildrenSRV;
                Parameters->EditFlags = EditFlagsSRV;
                Parameters->EditNodeCount = (uint32)EditData.Flags.Num();
                Parameters->EditRootExtent = EditData.RootExtent;
                Parameters->HalfFinest = VoxelDoubleFloat(HalfFinest);
                Parameters->ContinentOctaves = Data->ContinentOctaves;
                Parameters->MountainOctaves = Data->MountainOctaves;
                Parameters->DetailOctaves = Data->DetailOctaves;
                Parameters->InvNormContinent = Data->InvNormContinent;
                Parameters->InvNormMountain = Data->InvNormMountain;
                Parameters->InvNormDetail = Data->InvNormDetail;
                Parameters->PlanetRadius = Data->PlanetRadius;
                Parameters->NoiseAmplitude = Data->NoiseAmplitude;
                Parameters->ContinentWeight = Data->ContinentWeight;
                Parameters->MountainWeight = Data->MountainWeight;
                Parameters->MountainStart = Data->MountainStart;
                Parameters->MountainBlend = Data->MountainBlend;
                Parameters->DetailWeight = Data->DetailWeight;
                Parameters->RidgeGain = Data->RidgeGain;
                Parameters->SlopeDamping = Data->SlopeDamping;
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelDensity %dx%dx%d", Block.Dim.X, Block.Dim.Y, Block.Dim.Z),
                    Shader, Parameters, FComputeShaderUtils::GetGroupCount(Block.Dim, FVoxelDensityCS::GroupSize));
                Offset += (uint32)Block.NumPoints();
            }

            State->Buffer = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.DensityReadback"));
            AddEnqueueCopyPass(GraphBuilder, State->Buffer.Get(), OutBuffer, NumBytes);
            GraphBuilder.Execute();
        });

    // Poll the readback once per frame on the render thread; hand the result to the game thread.
    VoxelGPUCommon::PollRenderThread([State]() -> bool
        {
            if (State->bDone.load(std::memory_order_acquire)) return true;
            if (!State->Buffer.IsValid() || !State->Buffer->IsReady()) return false;
            const uint32 NumBytes = (uint32)State->Count * (uint32)sizeof(float);
            TArray<float> Result;
            if (const void* Source = State->Buffer->Lock(NumBytes))
            {
                Result.SetNumUninitialized(State->Count);
                FMemory::Memcpy(Result.GetData(), Source, NumBytes);
                State->Buffer->Unlock();
            }
            State->Buffer.Reset();
            State->bDone.store(true, std::memory_order_release);
            AsyncTask(ENamedThreads::GameThread, [State, Result = MoveTemp(Result)]() mutable { State->OnDone(MoveTemp(Result)); });
            return true;
        });
}

TSharedRef<FVoxelEditGPUData> FVoxelEditGPUData::Build(const FSparseEditStore* Store, double HalfFinest, int32 MaxDepth)
{
    const TSharedRef<FVoxelEditGPUData> Data = MakeShared<FVoxelEditGPUData>();
    Data->RootExtent = 1 << FMath::Clamp(MaxDepth, 0, 29);
    if (!Store) return Data;
    const TSharedPtr<const FSparseEditNode> Root = Store->GetRootSnapshot();
    if (!Root.IsValid()) return Data;

    // The GPU walks the tree in integer lattice coordinates, which needs the tree to be the
    // terrain root: centred on the actor, half-extent HalfFinest * 2^MaxDepth.
    const double Expected = HalfFinest * (double)Data->RootExtent;
    if (!Store->GetCenter().IsNearlyZero() || FMath::Abs(Store->GetExtent() / Expected - 1.0) > 1e-9)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelMesherGPU] Edit tree (extent %.6g) is not the terrain root (%.6g); edits are left out on the GPU."), Store->GetExtent(), Expected);
        return Data;
    }

    TArray<const FSparseEditNode*> Order;
    Order.Add(Root.Get());
    for (int32 i = 0; i < Order.Num(); i++)
    {
        const FSparseEditNode* Node = Order[i];
        for (int32 c = 0; c < 8; c++)
            Data->Corners.Add((float)Node->CornerDensities[c]);
        Data->Flags.Add(Node->HasEdits ? 1u : 0u);
        for (int32 c = 0; c < 8; c++)
        {
            const FSparseEditNode* Child = Node->Children[c].Get();
            Data->Children.Add(Child ? Order.Num() : -1);
            if (Child)
                Order.Add(Child);
        }
    }
    return Data;
}
