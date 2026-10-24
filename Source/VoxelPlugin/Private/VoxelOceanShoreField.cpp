#include "VoxelOceanShoreField.h"
#include "VoxelOceanShoreCore.h"
#include "VoxelOceanCoastField.h"
#include "VoxelLandBake.h"
#include "VoxelLandSea.h"
#include "VoxelPlugin.h"
#include "VoxelGPUCommon.h"
#include "Async/Async.h"
#include "Async/ParallelFor.h"
#include "Engine/Texture2DArray.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "RHIGPUReadback.h"
#include "TextureResource.h"
#include "Misc/App.h"
#include "HAL/IConsoleManager.h"

// ---------------------------------------------------------------------------
// Shaders (VoxelOceanShoreBuild.usf; parameter names must match it exactly)
// ---------------------------------------------------------------------------

#define VOXEL_OCEAN_SHORE_SHADER_COMMON(ShaderClass) \
    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
    { \
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
    }

class FVoxelOceanShoreSeedCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanShoreSeedCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanShoreSeedCS, FGlobalShader);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, ShoreHeight)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, ShoreSea)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, ShoreSeedsOut)
        SHADER_PARAMETER(uint32, ShoreN)
        SHADER_PARAMETER(uint32, ShoreCount)
        SHADER_PARAMETER(uint32, ShoreGroupsX)
    END_SHADER_PARAMETER_STRUCT()
    VOXEL_OCEAN_SHORE_SHADER_COMMON(FVoxelOceanShoreSeedCS)
};

class FVoxelOceanShoreJumpCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanShoreJumpCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanShoreJumpCS, FGlobalShader);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, ShoreSeedsIn)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, ShoreSeedsOut)
        SHADER_PARAMETER(uint32, ShoreN)
        SHADER_PARAMETER(uint32, ShoreCount)
        SHADER_PARAMETER(uint32, ShoreGroupsX)
        SHADER_PARAMETER(int32, ShoreStep)
    END_SHADER_PARAMETER_STRUCT()
    VOXEL_OCEAN_SHORE_SHADER_COMMON(FVoxelOceanShoreJumpCS)
};

class FVoxelOceanShorePackCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanShorePackCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanShorePackCS, FGlobalShader);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, ShoreHeight)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, ShoreSea)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, ShoreSeedsIn)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, ShoreTexelsOut)
        SHADER_PARAMETER(uint32, ShoreN)
        SHADER_PARAMETER(uint32, ShoreCount)
        SHADER_PARAMETER(uint32, ShoreGroupsX)
        SHADER_PARAMETER(float, ShoreRadiusM)
        SHADER_PARAMETER(float, ShoreReach)
    END_SHADER_PARAMETER_STRUCT()
    VOXEL_OCEAN_SHORE_SHADER_COMMON(FVoxelOceanShorePackCS)
};

class FVoxelOceanShoreSmoothCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanShoreSmoothCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanShoreSmoothCS, FGlobalShader);
    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, ShoreTexelsIn)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, ShoreTexelsOut)
        SHADER_PARAMETER(uint32, ShoreN)
        SHADER_PARAMETER(uint32, ShoreCount)
        SHADER_PARAMETER(uint32, ShoreGroupsX)
        SHADER_PARAMETER(float, ShoreReach)
    END_SHADER_PARAMETER_STRUCT()
    VOXEL_OCEAN_SHORE_SHADER_COMMON(FVoxelOceanShoreSmoothCS)
};

#undef VOXEL_OCEAN_SHORE_SHADER_COMMON

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanShoreSeedCS, "/Plugin/VoxelPlugin/Private/VoxelOceanShoreBuild.usf", "SeedCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanShoreJumpCS, "/Plugin/VoxelPlugin/Private/VoxelOceanShoreBuild.usf", "JumpCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanShorePackCS, "/Plugin/VoxelPlugin/Private/VoxelOceanShoreBuild.usf", "PackCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanShoreSmoothCS, "/Plugin/VoxelPlugin/Private/VoxelOceanShoreBuild.usf", "SmoothCS", SF_Compute);

static TAutoConsoleVariable<int32> CVarVoxelOceanShoreFieldOnCpu(
    TEXT("voxel.Ocean.ShoreFieldOnCpu"), 0,
    TEXT("1: build the shore field on the CPU even where there is a GPU (seconds, in the background), as a dedicated server does: every machine then runs the same code (a GPU's build can differ from it by float rounding, by vendor), for the queries' agreement near the coast in multiplayer."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelOceanShoreFromTerrain(
    TEXT("voxel.Ocean.ShoreFromTerrain"), 1,
    TEXT("Stage W7d: 1 (default): the shore field's sea and coast from the terrain's own heights at the land bake's cells (sampled on the GPU first, a fraction of a second), not the bake's: the terrain samples the bake through a domain warp (and adds hills, peaks, gullies, carves), so its coasts stand kilometres from the bake's. 0: the bake's heights. A server (no GPU) uses the bake's."),
    ECVF_Default);

namespace VoxelOceanShoreFieldDetail
{
    /** Frames to wait for the shaders (compiling) before building on the CPU instead. */
    constexpr int32 ShaderWaitFrames = 300;
    /** Stage W7d: the terrain's heights: cells per sampling chunk, chunks in flight, and frames
     *  to wait for the terrain (no chunk taken) before the bake's heights instead. */
    constexpr int32 SampleChunk = 1 << 18;
    constexpr int32 SampleInFlight = 4;
    constexpr int32 SampleWaitFrames = 1800;
    /** Cells per task of the CPU build. */
    constexpr int32 CpuChunk = 4096;

    bool AreShadersReady()
    {
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        auto Has = [ShaderMap](FShaderType& Type) { return ShaderMap && ShaderMap->GetShader(&Type, 0).IsValid(); };
        return Has(FVoxelOceanShoreSeedCS::GetStaticType()) && Has(FVoxelOceanShoreJumpCS::GetStaticType()) && Has(FVoxelOceanShorePackCS::GetStaticType())
            && Has(FVoxelOceanShoreSmoothCS::GetStaticType());
    }

    FIntVector GroupsFor(uint32 Threads, uint32& OutGroupsX)
    {
        const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp(Threads, 64u), 1u);
        OutGroupsX = FMath::Min(NumGroups, 32768u);
        return FIntVector((int32)OutGroupsX, (int32)FMath::DivideAndRoundUp(NumGroups, OutGroupsX), 1);
    }

    bool HasInputs(const FVoxelLandBake& Bake)
    {
        return Bake.N > 0 && Bake.Height.Num() == Bake.NumCells() && Bake.Attr.Num() == Bake.NumCells()
            && FVoxelLandBake::Pad == VoxelOceanShoreCore::Pad;
    }
}

namespace VOSF = VoxelOceanShoreFieldDetail;

// ---------------------------------------------------------------------------
// Field
// ---------------------------------------------------------------------------

FVoxelOceanShore FVoxelOceanShoreField::Sample(const FVector& PlanetPoint) const
{
    FVoxelOceanShore Out;
    if (!IsValid()) return Out;
    const VoxelShader::FVoxelOceanShoreSample S = VoxelOceanShoreCore::Sample(Texels.GetData(), N, ReachM, PlanetPoint.X, PlanetPoint.Y, PlanetPoint.Z);
    Out.Distance = (double)S.Distance * 100.0;
    Out.Direction = FVector(S.Direction.x, S.Direction.y, S.Direction.z);
    Out.Slope = S.Slope;
    return Out;
}

std::vector<double> FVoxelOceanShoreField::Coastline() const
{
    if (!IsValid()) return std::vector<double>();
    return VoxelOceanShoreCore::Coastline(Texels.GetData(), N, ReachM);
}

bool FVoxelOceanShoreField::IsSea(const FVector& PlanetPoint, double InMarginCm) const
{
    if (!IsValid()) return true;
    const VoxelShader::FVoxelOceanShoreSample S = VoxelOceanShoreCore::Sample(Texels.GetData(), N, ReachM, PlanetPoint.X, PlanetPoint.Y, PlanetPoint.Z);
    return (double)S.Distance * 100.0 > -InMarginCm;
}

double FVoxelOceanShoreField::CellCm() const
{
    return UE_DOUBLE_PI * 0.5 * RadiusCm / FMath::Max(N, 1);
}

bool FVoxelOceanShoreField::IsValid() const
{
    return N > 0 && Texels.Num() == VoxelOceanShoreCore::NumTexels(N);
}

UTexture2DArray* FVoxelOceanShoreField::MakeTexture(const uint32* Extra, int32 ExtraSlices) const
{
    if (!IsValid() || !FApp::CanEverRender()) return nullptr;
    const int32 Side = VoxelOceanShoreCore::Side(N);
    if (!Extra || ExtraSlices <= 0 || 6 + ExtraSlices > 2048) ExtraSlices = 0;
    const int32 Slices = 6 + ExtraSlices;
    UTexture2DArray* Texture = NewObject<UTexture2DArray>(GetTransientPackage(),
        MakeUniqueObjectName(GetTransientPackage(), UTexture2DArray::StaticClass(), FName(TEXT("VoxelOceanShore"))), RF_Transient);
    if (!Texture) return nullptr;
    // The platform data complete before the one resource is made (as the cascades' arrays).
    FTexturePlatformData* Data = new FTexturePlatformData();
    Data->SizeX = Side;
    Data->SizeY = Side;
    Data->SetNumSlices(Slices);
    Data->PixelFormat = PF_R32_FLOAT;
    FTexture2DMipMap* Level = new FTexture2DMipMap(Side, Side, Slices);
    Data->Mips.Add(Level);
    const int64 Bytes = (int64)Texels.Num() * sizeof(uint32);
    const int64 ExtraBytes = (int64)ExtraSlices * Side * Side * (int64)sizeof(uint32);
    Level->BulkData.Lock(LOCK_READ_WRITE);
    uint8* Memory = (uint8*)Level->BulkData.Realloc(Bytes + ExtraBytes);
    FMemory::Memcpy(Memory, Texels.GetData(), Bytes);
    if (ExtraBytes > 0) FMemory::Memcpy(Memory + Bytes, Extra, ExtraBytes);
    Level->BulkData.Unlock();
    Texture->SetPlatformData(Data);
    Texture->SRGB = false;
    Texture->CompressionSettings = TC_HDR;
    Texture->Filter = TF_Nearest;
    Texture->AddressX = TA_Clamp;
    Texture->AddressY = TA_Clamp;
    Texture->AddressZ = TA_Clamp;
    Texture->LODGroup = TEXTUREGROUP_Pixels2D;
    Texture->NeverStream = true;
    Texture->UpdateResource();
    return Texture;
}

TSharedPtr<FVoxelOceanShoreRHI, ESPMode::ThreadSafe> FVoxelOceanShoreRHI::Make(UTexture2DArray* Texture, int32 InN, double InReachM)
{
    if (!Texture || !Texture->GetResource()) return nullptr;
    const TSharedPtr<FVoxelOceanShoreRHI, ESPMode::ThreadSafe> Out = MakeShared<FVoxelOceanShoreRHI, ESPMode::ThreadSafe>();
    Out->N = InN;
    Out->ReachM = (float)InReachM;
    FTextureResource* Resource = Texture->GetResource();
    // After the resource's own initialization (enqueued by UpdateResource).
    ENQUEUE_RENDER_COMMAND(VoxelOceanShoreRHI)([Out, Resource](FRHICommandListImmediate&)
        {
            Out->Texture = Resource->TextureRHI;
            if (!Out->Texture.IsValid())
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The shore field's texture has no RHI resource: the pages, the probe and the underwater view see no field."));
        });
    return Out;
}

// ---------------------------------------------------------------------------
// Builder
// ---------------------------------------------------------------------------

struct FVoxelOceanShoreBuilder::FJob
{
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake;
    int32 N = 0;
    double RadiusM = 0.0;
    double ReachM = 0.0;
    /** The sea-connected bit, 32 cells a word (the GPU's input; made off the game thread). */
    TArray<uint32> SeaMask;
    TFuture<void> SeaMaskReady;
    /** Render thread: the texels' readback. */
    TUniquePtr<FRHIGPUBufferReadback> Readback;
    TArray<uint32> Texels;
    /** Game thread: the passes are enqueued. */
    bool bIssued = false;
    /** 0 running, 1 read back, -1 failed (the shaders). */
    std::atomic<int32> Status{ 0 };
    /** The CPU's build. */
    TFuture<TArray<uint32>> Pending;
    /** Stage W7d: the terrain's depths at the bake's cells (the GPU's), and the heights and the
     *  sea bits made from them (empty: the bake's). */
    TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe> Depth;
    TArray<float> Height;
    TArray<uint16> Attr;
    int32 SeaCells = 0;
    int32 BakeSeaCells = 0;
    double SampleSeconds = 0.0;

    ~FJob()
    {
        // A readback still pending (the poll torn down at exit): released on the render thread.
        if (Readback.IsValid())
        {
            FRHIGPUBufferReadback* Orphan = Readback.Release();
            ENQUEUE_RENDER_COMMAND(VoxelOceanShoreReadbackRelease)([Orphan](FRHICommandListImmediate&) { delete Orphan; });
        }
    }
};

FVoxelOceanShoreBuilder::FVoxelOceanShoreBuilder() = default;
FVoxelOceanShoreBuilder::~FVoxelOceanShoreBuilder() = default;

void FVoxelOceanShoreBuilder::Reset()
{
    Bake.Reset();
    Job.Reset();
    Result.Reset();
    bDone = true;
    bGpu = false;
    bSampling = false;
    Issued = 0;
    SampleWaitFrames = 0;
}

void FVoxelOceanShoreBuilder::Start(const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& InBake, double InRadiusCm, double InReachM)
{
    Reset();
    Bake = InBake;
    RadiusCm = InRadiusCm;
    ReachM = InReachM;
    StartSeconds = FPlatformTime::Seconds();
    WaitFrames = 0;
    if (!Bake.IsValid() || RadiusCm <= 0.0 || ReachM <= 0.0 || !VOSF::HasInputs(*Bake))
    {
        // No land bake (a noise planet): open water everywhere, no field.
        bDone = true;
        return;
    }
    bDone = false;
    Job = MakeShared<FJob, ESPMode::ThreadSafe>();
    Job->Bake = Bake;
    Job->N = Bake->N;
    Job->RadiusM = RadiusCm * 0.01;
    Job->ReachM = ReachM;
    Job->BakeSeaCells = Bake->SeaCells;
    if (FApp::CanEverRender() && GMaxRHIFeatureLevel >= ERHIFeatureLevel::SM5 && CVarVoxelOceanShoreFieldOnCpu.GetValueOnGameThread() == 0)
    {
        // Stage W7d: the terrain's own heights at the bake's cells first (the GPU's), unless off.
        if (CVarVoxelOceanShoreFromTerrain.GetValueOnGameThread() != 0)
        {
            bGpu = true;
            bSampling = true;
            Issued = 0;
            Job->Depth = MakeShared<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>();
            Job->Depth->Mode = 1;
            Job->Depth->N = Job->N;
            Job->Depth->R = 1;
            Job->Depth->FootprintCm = 0.5 * UE_DOUBLE_PI * RadiusCm / (double)FMath::Max(Job->N, 1);
            Job->Depth->Cells = (int64)Bake->NumCells();
            Job->Depth->DepthCm.SetNumZeroed(Bake->NumCells());
            return;
        }
        StartGpu();
    }
    else
        StartCpu();
}

void FVoxelOceanShoreBuilder::FromTerrain()
{
    bSampling = false;
    bGpu = true;
    TSharedPtr<FJob, ESPMode::ThreadSafe> InJob = Job;
    Job->SampleSeconds = FPlatformTime::Seconds() - StartSeconds;
    // The heights (m), the bake's other bits with the sea's from them (VoxelLandSea::Mark, as the
    // bake marks its own: the same share, bridge, spill and lakes), the GPU's mask.
    Job->SeaMaskReady = Async(EAsyncExecution::ThreadPool, [InJob]()
        {
            const FVoxelLandBake& B = *InJob->Bake;
            const int32 Count = B.NumCells();
            InJob->Height.SetNumUninitialized(Count);
            const float* Depth = InJob->Depth->DepthCm.GetData();
            for (int32 i = 0; i < Count; i++) InJob->Height[i] = -0.01f * Depth[i];
            InJob->Depth.Reset();
            InJob->Attr = B.Attr;
            const VoxelLandSea::FStats Stats = VoxelLandSea::Mark(B.N, FVoxelLandBake::Pad, InJob->Height.GetData(), InJob->Attr.GetData(),
                FVoxelLandBake::SeaBit, FVoxelLandBake::SeaMinShare, 3, B.Water.Num() == Count ? B.Water.GetData() : nullptr, FVoxelLandBake::SeaSpillLevel);
            InJob->SeaCells = Stats.SeaCells;
            InJob->SeaMask.Init(0u, (Count + 31) / 32);
            for (int32 i = 0; i < Count; i++)
                if ((InJob->Attr[i] & FVoxelLandBake::SeaBit) != 0)
                    InJob->SeaMask[i >> 5] |= 1u << (i & 31);
        });
}

void FVoxelOceanShoreBuilder::StartGpu()
{
    bGpu = true;
    bSampling = false;
    TSharedPtr<FJob, ESPMode::ThreadSafe> InJob = Job;
    Job->SeaMaskReady = Async(EAsyncExecution::ThreadPool, [InJob]()
        {
            const FVoxelLandBake& B = *InJob->Bake;
            const int32 Count = B.NumCells();
            InJob->SeaMask.Init(0u, (Count + 31) / 32);
            for (int32 i = 0; i < Count; i++)
                if ((B.Attr[i] & FVoxelLandBake::SeaBit) != 0)
                    InJob->SeaMask[i >> 5] |= 1u << (i & 31);
        });
}

void FVoxelOceanShoreBuilder::StartCpu()
{
    bGpu = false;
    bSampling = false;
    TSharedPtr<FJob, ESPMode::ThreadSafe> InJob = Job;
    // (Stage W7d: a GPU build that failed after the terrain's heights came back keeps them.)
    if (Job->SeaMaskReady.IsValid()) Job->SeaMaskReady.Wait();
    Job->Pending = Async(EAsyncExecution::ThreadPool, [InJob]() -> TArray<uint32>
        {
            const FVoxelLandBake& B = *InJob->Bake;
            const bool bTerrain = InJob->Height.Num() == B.NumCells() && InJob->Attr.Num() == B.NumCells();
            const std::vector<uint32_t> Built = VoxelOceanShoreCore::Build(InJob->N, InJob->RadiusM, InJob->ReachM,
                bTerrain ? InJob->Height.GetData() : B.Height.GetData(), bTerrain ? InJob->Attr.GetData() : B.Attr.GetData(),
                [](int Count, auto&& Fn)
                {
                    const int32 Chunks = FMath::DivideAndRoundUp(Count, VOSF::CpuChunk);
                    ParallelFor(Chunks, [&](int32 c)
                        {
                            const int32 End = FMath::Min(Count, (c + 1) * VOSF::CpuChunk);
                            for (int32 i = c * VOSF::CpuChunk; i < End; i++) Fn(i);
                        }, EParallelForFlags::BackgroundPriority);
                });
            TArray<uint32> Out;
            Out.SetNumUninitialized((int32)Built.size());
            FMemory::Memcpy(Out.GetData(), Built.data(), Built.size() * sizeof(uint32));
            return Out;
        });
}

void FVoxelOceanShoreBuilder::Tick(const FSampler& Sampler)
{
    if (bDone || !Job.IsValid()) return;
    if (bSampling)
    {
        // Stage W7d: the terrain's depth at every bake cell, in chunks (the coastal field's).
        const TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>& D = Job->Depth;
        if (D->Failed.load(std::memory_order_acquire) != 0 || (Issued == 0 && ++SampleWaitFrames > VOSF::SampleWaitFrames))
        {
            // No terrain to sample (or the shader gone): the bake's heights, as before.
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The shore field couldn't sample the terrain's heights; it uses the land bake's."));
            Job->Depth.Reset();
            StartGpu();
            return;
        }
        while (Sampler && Issued < D->Cells && D->InFlight.load(std::memory_order_acquire) < VOSF::SampleInFlight)
        {
            const int32 Count = (int32)FMath::Min<int64>(VOSF::SampleChunk, D->Cells - Issued);
            D->InFlight.fetch_add(1, std::memory_order_acq_rel);
            if (!Sampler(D, Issued, Count))
            {
                D->InFlight.fetch_sub(1, std::memory_order_acq_rel);
                break;
            }
            Issued += Count;
        }
        if (D->Received.load(std::memory_order_acquire) < D->Cells) return;
        FromTerrain();
        return;
    }
    if (!bGpu)
    {
        if (!Job->Pending.IsReady()) return;
        TSharedPtr<FVoxelOceanShoreField, ESPMode::ThreadSafe> Field = MakeShared<FVoxelOceanShoreField, ESPMode::ThreadSafe>();
        Field->Texels = Job->Pending.Consume();
        Field->bFromGpu = false;
        Finish(Field);
        return;
    }

    // The GPU: the passes once the shaders are there (else the CPU after a while), then the readback.
    if (!Job->bIssued)
    {
        if (!Job->SeaMaskReady.IsReady()) return;
        if (!VOSF::AreShadersReady())
        {
            if (++WaitFrames >= VOSF::ShaderWaitFrames)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The shore field's shaders aren't available; building it on the CPU."));
                StartCpu();
            }
            return;
        }
        TSharedPtr<FJob, ESPMode::ThreadSafe> InJob = Job;
        ENQUEUE_RENDER_COMMAND(VoxelOceanShoreBuild)([InJob](FRHICommandListImmediate& RHICmdList)
            {
                if (!VOSF::AreShadersReady())
                {
                    InJob->Status.store(-1, std::memory_order_release);
                    return;
                }
                const FVoxelLandBake& B = *InJob->Bake;
                const uint32 Count = (uint32)B.NumCells();
                const uint32 N = (uint32)InJob->N;
                uint32 GroupsX = 1;
                const FIntVector Groups = VOSF::GroupsFor(Count, GroupsX);
                FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
                FRDGBuilder GraphBuilder(RHICmdList);
                // (Stage W7d: the terrain's heights where sampled.)
                const TArray<float>& Heights = InJob->Height.Num() == B.NumCells() ? InJob->Height : B.Height;
                const FRDGBufferRef Height = VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("Voxel.ShoreHeight"), MakeConstArrayView(Heights));
                const FRDGBufferRef Sea = VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.ShoreSea"), MakeConstArrayView(InJob->SeaMask));
                FRDGBufferRef Seeds[2] = {
                    GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), Count), TEXT("Voxel.ShoreSeedsA")),
                    GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), Count), TEXT("Voxel.ShoreSeedsB")) };
                FRDGBufferRef Texels[2] = {
                    GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), Count), TEXT("Voxel.ShoreTexelsA")),
                    GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), Count), TEXT("Voxel.ShoreTexelsB")) };
                {
                    FVoxelOceanShoreSeedCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanShoreSeedCS::FParameters>();
                    P->ShoreHeight = GraphBuilder.CreateSRV(Height);
                    P->ShoreSea = GraphBuilder.CreateSRV(Sea);
                    P->ShoreSeedsOut = GraphBuilder.CreateUAV(Seeds[0]);
                    P->ShoreN = N;
                    P->ShoreCount = Count;
                    P->ShoreGroupsX = GroupsX;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanShore seeds"), TShaderMapRef<FVoxelOceanShoreSeedCS>(ShaderMap), P, Groups);
                }
                int32 Current = 0;
                for (const int K : VoxelOceanShoreCore::Steps(InJob->N, InJob->RadiusM, InJob->ReachM))
                {
                    FVoxelOceanShoreJumpCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanShoreJumpCS::FParameters>();
                    P->ShoreSeedsIn = GraphBuilder.CreateSRV(Seeds[Current]);
                    P->ShoreSeedsOut = GraphBuilder.CreateUAV(Seeds[1 - Current]);
                    P->ShoreN = N;
                    P->ShoreCount = Count;
                    P->ShoreGroupsX = GroupsX;
                    P->ShoreStep = K;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanShore jump %d", K), TShaderMapRef<FVoxelOceanShoreJumpCS>(ShaderMap), P, Groups);
                    Current = 1 - Current;
                }
                {
                    FVoxelOceanShorePackCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanShorePackCS::FParameters>();
                    P->ShoreHeight = GraphBuilder.CreateSRV(Height);
                    P->ShoreSea = GraphBuilder.CreateSRV(Sea);
                    P->ShoreSeedsIn = GraphBuilder.CreateSRV(Seeds[Current]);
                    P->ShoreTexelsOut = GraphBuilder.CreateUAV(Texels[0]);
                    P->ShoreN = N;
                    P->ShoreCount = Count;
                    P->ShoreGroupsX = GroupsX;
                    P->ShoreRadiusM = (float)InJob->RadiusM;
                    P->ShoreReach = (float)InJob->ReachM;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanShore pack"), TShaderMapRef<FVoxelOceanShorePackCS>(ShaderMap), P, Groups);
                }
                int32 CurrentTexels = 0;
                for (int32 Pass = 0; Pass < VoxelOceanShoreCore::SmoothPasses; Pass++)
                {
                    FVoxelOceanShoreSmoothCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanShoreSmoothCS::FParameters>();
                    P->ShoreTexelsIn = GraphBuilder.CreateSRV(Texels[CurrentTexels]);
                    P->ShoreTexelsOut = GraphBuilder.CreateUAV(Texels[1 - CurrentTexels]);
                    P->ShoreN = N;
                    P->ShoreCount = Count;
                    P->ShoreGroupsX = GroupsX;
                    P->ShoreReach = (float)InJob->ReachM;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanShore smooth %d", Pass), TShaderMapRef<FVoxelOceanShoreSmoothCS>(ShaderMap), P, Groups);
                    CurrentTexels = 1 - CurrentTexels;
                }
                InJob->Readback = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.ShoreReadback"));
                AddEnqueueCopyPass(GraphBuilder, InJob->Readback.Get(), Texels[CurrentTexels], Count * (uint32)sizeof(uint32));
                GraphBuilder.Execute();
                // Read back once it's there (polled on the render thread).
                VoxelGPUCommon::PollRenderThread([InJob, Count]() -> bool
                    {
                        if (!InJob->Readback.IsValid() || !InJob->Readback->IsReady()) return false;
                        const void* Data = InJob->Readback->Lock(Count * (uint32)sizeof(uint32));
                        InJob->Texels.SetNumUninitialized((int32)Count);
                        FMemory::Memcpy(InJob->Texels.GetData(), Data, Count * sizeof(uint32));
                        InJob->Readback->Unlock();
                        InJob->Readback.Reset();
                        InJob->Status.store(1, std::memory_order_release);
                        return true;
                    });
            });
        Job->bIssued = true;
        return;
    }
    const int32 Status = Job->Status.load(std::memory_order_acquire);
    if (Status < 0)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The shore field's GPU build failed; building it on the CPU."));
        StartCpu();
        return;
    }
    if (Status != 1) return;
    TSharedPtr<FVoxelOceanShoreField, ESPMode::ThreadSafe> Field = MakeShared<FVoxelOceanShoreField, ESPMode::ThreadSafe>();
    Field->Texels = MoveTemp(Job->Texels);
    Field->bFromGpu = true;
    Finish(Field);
}

void FVoxelOceanShoreBuilder::Finish(TSharedPtr<FVoxelOceanShoreField, ESPMode::ThreadSafe> Field)
{
    Field->N = Bake->N;
    Field->RadiusCm = RadiusCm;
    Field->ReachM = ReachM;
    Field->BakeKey = Bake->Key;
    Field->BuildSeconds = FPlatformTime::Seconds() - StartSeconds;
    // Stage W7d: the heights it was made from, when the terrain's.
    const bool bTerrain = Job.IsValid() && Job->Height.Num() == Bake->NumCells();
    const int32 BakeSeaCells = Job.IsValid() ? Job->BakeSeaCells : 0;
    const double SampleSeconds = Job.IsValid() ? Job->SampleSeconds : 0.0;
    if (bTerrain)
    {
        Field->Height = MoveTemp(Job->Height);
        Field->SeaCells = Job->SeaCells;
    }
    bDone = true;
    Job.Reset();
    if (!Field->IsValid())
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The shore field of bake %08x came back incomplete; open water everywhere."), Bake->Key);
        Result.Reset();
        return;
    }
    Result = Field;
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Shore field of bake %08x: 6 x %d^2 cells, reach %.0f km, %s in %.2f s."),
        Bake->Key, Field->N, ReachM * 1.0e-3, Field->bFromGpu ? TEXT("on the GPU") : TEXT("on the CPU"), Field->BuildSeconds);
    if (bTerrain)
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Shore field from the terrain's own heights (sampled in %.2f s): %d sea cells (the land bake's: %d)."),
            SampleSeconds, Field->SeaCells, BakeSeaCells);
}
