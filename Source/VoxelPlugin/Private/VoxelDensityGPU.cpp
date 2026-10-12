#include "VoxelDensityGPU.h"
#include "VoxelGPUCommon.h"
#include "VoxelLandGPU.h"
#include "VoxelPlugin.h"
#include "FSparseEditStore.h"
#include "VoxelEditCore.h"
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
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditCells)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditOpList)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditOps)
        SHADER_PARAMETER(uint32, EditTableMask)
        SHADER_PARAMETER(uint32, EditCellShift)
        SHADER_PARAMETER(uint32, EditOpCount)
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
        SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelLandShaderParameters, Land)
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
            FRDGBufferSRVRef EditCellsSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.DensityEditCells"), MakeArrayView(EditData.Cells)));
            FRDGBufferSRVRef EditOpListSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.DensityEditOpList"), MakeArrayView(EditData.OpList)));
            FRDGBufferSRVRef EditOpsSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.DensityEditOps"), MakeArrayView(EditData.Ops)));
            // Blocks write disjoint ranges, so their passes need no barrier between them.
            FRDGBufferUAVRef OutUAV = GraphBuilder.CreateUAV(OutBuffer, ERDGUnorderedAccessViewFlags::SkipBarrier);
            TShaderMapRef<FVoxelDensityCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
            const FVoxelLandShaderParameters Land = VoxelLandGPU::GetParameters(GraphBuilder, *Data);

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
                Parameters->EditCells = EditCellsSRV;
                Parameters->EditOpList = EditOpListSRV;
                Parameters->EditOps = EditOpsSRV;
                Parameters->EditTableMask = EditData.TableMask;
                Parameters->EditCellShift = EditData.CellShift;
                Parameters->EditOpCount = EditData.OpCount;
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
                Parameters->Land = Land;
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
    Data->HalfFinest = HalfFinest;
    if (!Store) return Data;
    const TSharedPtr<const VoxelEditCore::FState, ESPMode::ThreadSafe> State = Store->GetSnapshot();
    if (!State.IsValid() || State->Ops.empty()) return Data;

    // The GPU reads the edits on the terrain's integer lattice, which needs the store to be
    // the terrain root's: centred on the actor, with the same half cell.
    if (!Store->GetCenter().IsNearlyZero() || FMath::Abs(Store->GetHalfFinest() / HalfFinest - 1.0) > 1e-9)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelMesherGPU] The edit store's lattice (half cell %.6g) is not the terrain's (%.6g); edits are left out on the GPU."),
            Store->GetHalfFinest(), HalfFinest);
        return Data;
    }

    VoxelEditCore::FGPUArrays Arrays;
    VoxelEditCore::Flatten(*State, Arrays);
    if (Arrays.Truncated > 0)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelMesherGPU] %d edit index cells hold more than 4095 edits; the GPU drops the rest there."), Arrays.Truncated);
    }
    Data->Cells.Append(Arrays.Cells.data(), (int32)Arrays.Cells.size());
    Data->OpList.Append(Arrays.OpList.data(), (int32)Arrays.OpList.size());
    Data->Ops.Append(Arrays.Ops.data(), (int32)Arrays.Ops.size());
    Data->TableMask = Arrays.TableMask;
    Data->CellShift = Arrays.CellShift;
    Data->OpCount = Arrays.OpCount;
    // The cells' bounds: a consumer that walks far from every edit skips the table (the land
    // surface cache's columns).
    bool bAny = false;
    for (size_t Slot = 0; Slot + 3 < Arrays.Cells.size(); Slot += 4)
    {
        if (Arrays.Cells[Slot + 3] == 0u) continue;
        const FIntVector Cell((int32)Arrays.Cells[Slot], (int32)Arrays.Cells[Slot + 1], (int32)Arrays.Cells[Slot + 2]);
        Data->CellMin = bAny ? FIntVector(FMath::Min(Data->CellMin.X, Cell.X), FMath::Min(Data->CellMin.Y, Cell.Y), FMath::Min(Data->CellMin.Z, Cell.Z)) : Cell;
        Data->CellMax = bAny ? FIntVector(FMath::Max(Data->CellMax.X, Cell.X), FMath::Max(Data->CellMax.Y, Cell.Y), FMath::Max(Data->CellMax.Z, Cell.Z)) : Cell;
        bAny = true;
    }
    return Data;
}
