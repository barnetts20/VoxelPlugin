#include "VoxelLandCache.h"
#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "VoxelGPUCommon.h"
#include "VoxelLandBake.h"
#include "VoxelLandGPU.h"
#include "VoxelPlugin.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "Engine/Texture2DDynamic.h"
#include "Engine/TextureRenderTarget2D.h"
#include "GlobalShader.h"
#include "HAL/IConsoleManager.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "RenderGraphUtils.h"
#include "RenderTargetPool.h"
#include "RenderingThread.h"
#include "ShaderParameterStruct.h"
#include "TextureResource.h"

// The constants the shaders share (VoxelLandCache.ush), and the plan.
#include "VoxelLandCacheLayout.ush"
#include "VoxelLandCachePlan.h"

static TAutoConsoleVariable<int32> CVarVoxelLandCacheTexelsPerFrame(
    TEXT("voxel.Land.CacheTexelsPerFrame"), 131072,
    TEXT("The land surface cache's write budget a frame (texels; a level is 262144). A re-centring level's new frame gets up to half first; window moves next, coarse to fine; refills after, coarse to fine; the re-centring again with what's left."),
    ECVF_Default);

static TAutoConsoleVariable<float> CVarVoxelLandCacheReanchorDegrees(
    TEXT("voxel.Land.CacheReanchorDegrees"), 10.0f,
    TEXT("A cache level's projection re-centres once the camera is this far from its centre (degrees of arc, 1 to 20): written into a spare tile while the old one is still read, one level at a time. Past 25 degrees (a teleport) every such level re-anchors at once and is written again."),
    ECVF_Default);


const FName FVoxelLandCache::HeightTextureName(TEXT("VoxelLandCacheHeight"));
const FName FVoxelLandCache::SlopeTextureName(TEXT("VoxelLandCacheSlope"));
const FName FVoxelLandCache::FieldsTextureName(TEXT("VoxelLandCacheFields"));
const FName FVoxelLandCache::StateTextureName(TEXT("VoxelLandCacheState"));

/** Writes a rectangle of one level (VoxelLandCacheWrite.usf). */
class FVoxelLandCacheWriteCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelLandCacheWriteCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelLandCacheWriteCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOctave>, Octaves)
        SHADER_PARAMETER(uint32, ContinentOctaves)
        SHADER_PARAMETER(uint32, MountainOctaves)
        SHADER_PARAMETER(uint32, DetailOctaves)
        SHADER_PARAMETER(FVector2f, InvNormContinent)
        SHADER_PARAMETER(FVector2f, InvNormMountain)
        SHADER_PARAMETER(FVector2f, InvNormDetail)
        SHADER_PARAMETER(FVector2f, PlanetRadius)
        SHADER_PARAMETER(FVector2f, NoiseAmplitude)
        SHADER_PARAMETER(FVector2f, HalfFinest)
        SHADER_PARAMETER(FVector2f, ContinentWeight)
        SHADER_PARAMETER(FVector2f, MountainWeight)
        SHADER_PARAMETER(FVector2f, MountainStart)
        SHADER_PARAMETER(FVector2f, MountainBlend)
        SHADER_PARAMETER(FVector2f, DetailWeight)
        SHADER_PARAMETER(float, RidgeGain)
        SHADER_PARAMETER(float, SlopeDamping)
        SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelLandShaderParameters, Land)
        SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelLandDebugParameters, LandDebug)
        SHADER_PARAMETER(int32, CacheTile)
        SHADER_PARAMETER(float, TexelSize)
        SHADER_PARAMETER(FIntPoint, RectMin)
        SHADER_PARAMETER(FIntPoint, RectSize)
        SHADER_PARAMETER(FVector3f, InverseRow0High)
        SHADER_PARAMETER(FVector3f, InverseRow0Low)
        SHADER_PARAMETER(FVector3f, InverseRow1High)
        SHADER_PARAMETER(FVector3f, InverseRow1Low)
        SHADER_PARAMETER(FVector3f, InverseRow2High)
        SHADER_PARAMETER(FVector3f, InverseRow2Low)
        SHADER_PARAMETER(FVector2f, SeaRadius)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, OutHeight)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, OutSlope)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, OutFields)
    END_SHADER_PARAMETER_STRUCT()

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        // The double-float code needs exact float rounding.
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelLandCacheWriteCS, "/Plugin/VoxelPlugin/Private/VoxelLandCacheWrite.usf", "WriteCS", SF_Compute);

namespace VoxelLandCacheDetail
{
    constexpr int32 N = VOXEL_LAND_CACHE_N;

    /** A level's frame and window, as the render thread writes it. */
    struct FLevelRender
    {
        int32 Level = 0;
        int32 Tile = 0;
        float TexelSize = 0.0f;
        FVector3f Row[3][2];      // the inverse's rows, high and low
    };

    FVector3f SplitHigh(const FVector& V) { return FVector3f((float)V.X, (float)V.Y, (float)V.Z); }
    FVector3f SplitLow(const FVector& V)
    {
        const FVector3f H = SplitHigh(V);
        return FVector3f((float)(V.X - (double)H.X), (float)(V.Y - (double)H.Y), (float)(V.Z - (double)H.Z));
    }

    UTextureRenderTarget2D* MakeTarget(UObject* Outer, int32 Width, int32 Height, ETextureRenderTargetFormat Format)
    {
        UTextureRenderTarget2D* Target = NewObject<UTextureRenderTarget2D>(Outer, NAME_None, RF_Transient | RF_DuplicateTransient);
        Target->bCanCreateUAV = true;
        Target->bAutoGenerateMips = false;
        Target->RenderTargetFormat = Format;
        Target->ClearColor = FLinearColor::Transparent;
        Target->Filter = TF_Nearest;
        Target->AddressX = TA_Clamp;
        Target->AddressY = TA_Clamp;
        Target->InitAutoFormat(Width, Height);
        Target->UpdateResourceImmediate(true);
        return Target;
    }
}

int32 FVoxelLandCache::LevelsFor(double InSeaRadius)
{
    return VoxelLandCachePlan::LevelsFor(InSeaRadius);
}

int32 FVoxelLandCache::TilesFor(int32 InLevels)
{
    return VoxelLandCachePlan::TilesFor(InLevels);
}

bool FVoxelLandCache::MakeTextures(int32 InLevels, UObject* Outer, FVoxelLandCacheTextures& Out)
{
    using namespace VoxelLandCacheDetail;
    Out = FVoxelLandCacheTextures();
    const int32 Rows = (TilesFor(InLevels) + VOXEL_LAND_CACHE_COLUMNS - 1) / VOXEL_LAND_CACHE_COLUMNS;
    const int32 Width = VOXEL_LAND_CACHE_COLUMNS * N;
    const int32 Height = Rows * N;
    Out.Height = MakeTarget(Outer, Width, Height, RTF_R32f);
    Out.Slope = MakeTarget(Outer, Width, Height, RTF_RGBA16f);
    Out.Fields = MakeTarget(Outer, Width, Height, RTF_RGBA16f);
    FTexture2DDynamicCreateInfo Info(PF_A32B32G32R32F, false, false, TF_Nearest, AM_Clamp);
    Out.State = UTexture2DDynamic::Create(VOXEL_LAND_CACHE_STATE_WIDTH, VOXEL_LAND_CACHE_MAX_LEVELS + 1, Info);
    if (Out.State)
        Out.State->SetFlags(RF_Transient | RF_DuplicateTransient);
    // The objects always exist; the resources are what the RHI may refuse.
    if (!Out.IsValid() || !Out.Height->GameThread_GetRenderTargetResource() || !Out.Slope->GameThread_GetRenderTargetResource()
        || !Out.Fields->GameThread_GetRenderTargetResource() || !Out.State->GetResource())
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Couldn't create the land surface cache's textures."));
        Out = FVoxelLandCacheTextures();
        return false;
    }
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Land surface cache: %d levels (and a spare tile), atlases %d x %d (%.0f MB)."), InLevels, Width, Height,
        (double)Width * Height * (4 + 8 + 8) * 1.0e-6);
    return true;
}

FVoxelLandCache::FVoxelLandCache() = default;
FVoxelLandCache::~FVoxelLandCache() = default;

bool FVoxelLandCache::Initialize(const FVoxelDensityCPU& Density, const FVoxelLandCacheTextures& InTextures)
{
    Reset();
    TSharedRef<FVoxelDensityGPUData, ESPMode::ThreadSafe> Data = MakeShared<FVoxelDensityGPUData, ESPMode::ThreadSafe>();
    Density.BuildGPUData(*Data);
    if (Data->LandMode == 0 || !Data->Land.IsValid() || !InTextures.IsValid()) return false;
    SeaRadius = Data->Land->SeaRadius;
    const int32 Count = LevelsFor(SeaRadius);
    const int32 Capacity = (InTextures.Height->SizeY / VOXEL_LAND_CACHE_N) * VOXEL_LAND_CACHE_COLUMNS;
    if (TilesFor(Count) > Capacity) return false;
    GPUData = Data;
    Textures = InTextures;
    Plan = MakeUnique<VoxelLandCachePlan::FPlan>();
    Plan->Init(Count, SeaRadius);
    Generation = 0;
    FailedGeneration = MakeShared<std::atomic<uint32>, ESPMode::ThreadSafe>(0u);
    // The state the material reads from the first frame (the texture starts undefined): no
    // level valid yet. Render commands run in order, so every frame drawn after this reads it.
    if (!Dispatch(nullptr, 0))
    {
        Reset();
        return false;
    }
    return true;
}

void FVoxelLandCache::Reset()
{
    Plan.Reset();
    GPUData.Reset();
    Textures = FVoxelLandCacheTextures();
    SeaRadius = 0.0;
    TexelsWritten = 0;
    LastTickFrame = MAX_uint64;
    Generation = 0;
    FailedGeneration.Reset();
}

int32 FVoxelLandCache::GetLevels() const
{
    return Plan.IsValid() ? (int32)Plan->Levels.size() : 0;
}

void FVoxelLandCache::SetTextureParameters(UMaterialInstanceDynamic* Material) const
{
    if (!Material || !Textures.IsValid()) return;
    Material->SetTextureParameterValue(HeightTextureName, Textures.Height);
    Material->SetTextureParameterValue(SlopeTextureName, Textures.Slope);
    Material->SetTextureParameterValue(FieldsTextureName, Textures.Fields);
    Material->SetTextureParameterValue(StateTextureName, Textures.State);
}

bool FVoxelLandCache::IsWritable() const
{
    // On the game thread, before the plan moves on: a plan whose writes can't happen would
    // mark windows written that aren't.
    if (!IsInitialized() || !Plan.IsValid() || !Textures.IsValid()) return false;
    if (!Textures.Height->GameThread_GetRenderTargetResource() || !Textures.Slope->GameThread_GetRenderTargetResource()
        || !Textures.Fields->GameThread_GetRenderTargetResource() || !Textures.State->GetResource())
        return false;
    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    return ShaderMap && ShaderMap->GetShader(&FVoxelLandCacheWriteCS::GetStaticType(), 0).IsValid();
}

void FVoxelLandCache::Tick(const FVector& CameraPlanet)
{
    if (!IsInitialized() || !Plan.IsValid() || CameraPlanet.IsNearlyZero() || !FMath::IsFinite(CameraPlanet.SizeSquared())) return;
    // Once a frame (the actor's material update can run twice in one).
    if (LastTickFrame == GFrameCounter) return;
    LastTickFrame = GFrameCounter;
    if (!IsWritable()) return;
    // A frame whose writes couldn't run on the render thread: what the plan holds as written may
    // not be (the states since went up with no level valid), so everything is written again.
    const uint32 Failed = FailedGeneration.IsValid() ? FailedGeneration->load() : 0u;
    if (Failed > Generation)
    {
        Plan->InvalidateAll();
        Generation = Failed;
    }

    VoxelLandCachePlan::FSettings Settings;
    Settings.Budget = CVarVoxelLandCacheTexelsPerFrame.GetValueOnGameThread();
    Settings.ReanchorDegrees = (double)CVarVoxelLandCacheReanchorDegrees.GetValueOnGameThread();
    const double Direction[3] = { CameraPlanet.X, CameraPlanet.Y, CameraPlanet.Z };
    std::vector<VoxelLandCachePlan::FJob> Jobs;
    Plan->Tick(Direction, Settings, Jobs);
    Dispatch(Jobs.data(), (int32)Jobs.size());
}

bool FVoxelLandCache::Dispatch(const VoxelLandCachePlan::FJob* InJobs, int32 NumJobs)
{
    using namespace VoxelLandCacheDetail;
    using FJob = VoxelLandCachePlan::FJob;
    if (!Plan.IsValid()) return false;
    const std::vector<VoxelLandCachePlan::FLevel>& Levels = Plan->Levels;
    const int32 Count = (int32)Levels.size();

    // The state the material reads after these writes.
    TArray<FVector4f> State;
    State.Init(FVector4f(0, 0, 0, 0), VOXEL_LAND_CACHE_STATE_WIDTH * (VOXEL_LAND_CACHE_MAX_LEVELS + 1));
    State[0] = FVector4f((float)Count, (float)N, VOXEL_LAND_CACHE_T0, 0.0f);
    // The frames the jobs write with: the levels', and the re-centring level's new one last.
    TArray<FLevelRender> Frames;
    Frames.SetNum(Count + 1);
    for (int32 k = 0; k <= Count; k++)
    {
        const bool bNext = k == Count;
        const VoxelLandCachePlan::FLevel& Level = bNext ? Plan->Next : Levels[k];
        if (!bNext)
        {
            FVector4f* Row = &State[VOXEL_LAND_CACHE_STATE_WIDTH * (1 + k)];
            Row[0] = FVector4f(Level.A[0], Level.A[1], Level.A[2], Level.bValid ? 1.0f : 0.0f);
            Row[1] = FVector4f(Level.E1[0], Level.E1[1], Level.E1[2], (float)Level.TexelSize);
            Row[2] = FVector4f(Level.E2[0], Level.E2[1], Level.E2[2], (float)Level.Tile);
            // The origin as two exact floats each (a multiple of 4096, and the rest).
            const int32 LowI = Level.OriginI & 4095, LowJ = Level.OriginJ & 4095;
            Row[3] = FVector4f((float)(Level.OriginI - LowI), (float)LowI, (float)(Level.OriginJ - LowJ), (float)LowJ);
        }
        FLevelRender& Frame = Frames[k];
        Frame.Level = bNext ? -1 : k;
        Frame.Tile = Level.Tile;
        Frame.TexelSize = (float)Level.TexelSize;
        for (int32 r = 0; r < 3; r++)
        {
            const FVector V(Level.Inverse[r][0], Level.Inverse[r][1], Level.Inverse[r][2]);
            Frame.Row[r][0] = SplitHigh(V);
            Frame.Row[r][1] = SplitLow(V);
        }
    }
    TArray<FJob> Jobs;
    if (InJobs && NumJobs > 0)
        Jobs.Append(InJobs, NumJobs);
    for (const FJob& Job : Jobs)
        TexelsWritten += (uint64)Job.Width * (uint64)Job.Height;

    FTextureRenderTargetResource* HeightResource = Textures.Height->GameThread_GetRenderTargetResource();
    FTextureRenderTargetResource* SlopeResource = Textures.Slope->GameThread_GetRenderTargetResource();
    FTextureRenderTargetResource* FieldsResource = Textures.Fields->GameThread_GetRenderTargetResource();
    FTextureResource* StateResource = Textures.State->GetResource();
    if (!HeightResource || !SlopeResource || !FieldsResource || !StateResource) return false;

    ENQUEUE_RENDER_COMMAND(VoxelLandCacheWrite)(
        [Data = GPUData, Jobs = MoveTemp(Jobs), Frames = MoveTemp(Frames), State = MoveTemp(State), Sea = SeaRadius, Failed = FailedGeneration,
         Gen = Generation, HeightResource, SlopeResource, FieldsResource, StateResource](FRHICommandListImmediate& RHICmdList) mutable
        {
            FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            const bool bCanRun = Data.IsValid() && ShaderMap && ShaderMap->GetShader(&FVoxelLandCacheWriteCS::GetStaticType(), 0).IsValid()
                && HeightResource->GetRenderTargetTexture() && SlopeResource->GetRenderTargetTexture() && FieldsResource->GetRenderTargetTexture();
            // The plan moves on without these writes: the game thread writes everything again in
            // a later generation (only this thread raises the failed one).
            if (Jobs.Num() > 0 && !bCanRun && Failed.IsValid() && Failed->load() < Gen + 1)
                Failed->store(Gen + 1);
            // No level is read while what this state claims may not be written: this frame's
            // writes didn't run, or it was planned before the game thread saw an earlier failure.
            if (!bCanRun || (Failed.IsValid() && Gen < Failed->load()))
            {
                for (int32 k = 0; k < VOXEL_LAND_CACHE_MAX_LEVELS; k++)
                    State[VOXEL_LAND_CACHE_STATE_WIDTH * (1 + k)].W = 0.0f;
            }
            if (Jobs.Num() > 0 && bCanRun)
            {
                FRDGBuilder GraphBuilder(RHICmdList);
                FRDGTextureRef Height = GraphBuilder.RegisterExternalTexture(CreateRenderTarget(HeightResource->GetRenderTargetTexture(), TEXT("Voxel.LandCacheHeight")));
                FRDGTextureRef Slope = GraphBuilder.RegisterExternalTexture(CreateRenderTarget(SlopeResource->GetRenderTargetTexture(), TEXT("Voxel.LandCacheSlope")));
                FRDGTextureRef Fields = GraphBuilder.RegisterExternalTexture(CreateRenderTarget(FieldsResource->GetRenderTargetTexture(), TEXT("Voxel.LandCacheFields")));
                // The jobs write disjoint texels.
                FRDGTextureUAVRef HeightUAV = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Height), ERDGUnorderedAccessViewFlags::SkipBarrier);
                FRDGTextureUAVRef SlopeUAV = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Slope), ERDGUnorderedAccessViewFlags::SkipBarrier);
                FRDGTextureUAVRef FieldsUAV = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Fields), ERDGUnorderedAccessViewFlags::SkipBarrier);
                FRDGBufferSRVRef OctaveSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.LandCacheOctaves"), MakeArrayView(Data->Octaves)));
                const FVoxelLandShaderParameters Land = VoxelLandGPU::GetParameters(GraphBuilder, *Data);
                // The fields need the attributes and the lake levels (the lakes view uploads both).
                const FVoxelLandDebugParameters LandDebug = VoxelLandGPU::GetDebugParameters(GraphBuilder, *Data, 10, 0.0);
                TShaderMapRef<FVoxelLandCacheWriteCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
                for (const FJob& Job : Jobs)
                {
                    if (!Frames.IsValidIndex(Job.Frame)) continue;
                    const FLevelRender& Frame = Frames[Job.Frame];
                    FVoxelLandCacheWriteCS::FParameters* Parameters = GraphBuilder.AllocParameters<FVoxelLandCacheWriteCS::FParameters>();
                    Parameters->Octaves = OctaveSRV;
                    Parameters->ContinentOctaves = Data->ContinentOctaves;
                    Parameters->MountainOctaves = Data->MountainOctaves;
                    Parameters->DetailOctaves = Data->DetailOctaves;
                    Parameters->InvNormContinent = Data->InvNormContinent;
                    Parameters->InvNormMountain = Data->InvNormMountain;
                    Parameters->InvNormDetail = Data->InvNormDetail;
                    Parameters->PlanetRadius = Data->PlanetRadius;
                    Parameters->NoiseAmplitude = Data->NoiseAmplitude;
                    Parameters->HalfFinest = FVector2f::ZeroVector;
                    Parameters->ContinentWeight = Data->ContinentWeight;
                    Parameters->MountainWeight = Data->MountainWeight;
                    Parameters->MountainStart = Data->MountainStart;
                    Parameters->MountainBlend = Data->MountainBlend;
                    Parameters->DetailWeight = Data->DetailWeight;
                    Parameters->RidgeGain = Data->RidgeGain;
                    Parameters->SlopeDamping = Data->SlopeDamping;
                    Parameters->Land = Land;
                    Parameters->LandDebug = LandDebug;
                    Parameters->CacheTile = Frame.Tile;
                    Parameters->TexelSize = Frame.TexelSize;
                    Parameters->RectMin = FIntPoint(Job.I0, Job.J0);
                    Parameters->RectSize = FIntPoint(Job.Width, Job.Height);
                    Parameters->InverseRow0High = Frame.Row[0][0];
                    Parameters->InverseRow0Low = Frame.Row[0][1];
                    Parameters->InverseRow1High = Frame.Row[1][0];
                    Parameters->InverseRow1Low = Frame.Row[1][1];
                    Parameters->InverseRow2High = Frame.Row[2][0];
                    Parameters->InverseRow2Low = Frame.Row[2][1];
                    Parameters->SeaRadius = VoxelDoubleFloat(Sea);
                    Parameters->OutHeight = HeightUAV;
                    Parameters->OutSlope = SlopeUAV;
                    Parameters->OutFields = FieldsUAV;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelLandCache L%d tile %d %dx%d", Frame.Level, Frame.Tile, Job.Width, Job.Height), Shader, Parameters,
                        FIntVector(FMath::DivideAndRoundUp(Job.Width, VOXEL_LAND_CACHE_GROUP), FMath::DivideAndRoundUp(Job.Height, VOXEL_LAND_CACHE_GROUP), 1));
                }
                // Read by the material afterwards.
                GraphBuilder.SetTextureAccessFinal(Height, ERHIAccess::SRVMask);
                GraphBuilder.SetTextureAccessFinal(Slope, ERHIAccess::SRVMask);
                GraphBuilder.SetTextureAccessFinal(Fields, ERHIAccess::SRVMask);
                GraphBuilder.Execute();
            }
            if (StateResource->TextureRHI.IsValid())
            {
                const FUpdateTextureRegion2D Region(0, 0, 0, 0, VOXEL_LAND_CACHE_STATE_WIDTH, VOXEL_LAND_CACHE_MAX_LEVELS + 1);
                RHICmdList.UpdateTexture2D(StateResource->TextureRHI, 0, Region, VOXEL_LAND_CACHE_STATE_WIDTH * sizeof(FVector4f), (const uint8*)State.GetData());
            }
        });
    return true;
}

FString FVoxelLandCache::Describe() const
{
    if (!Plan.IsValid()) return TEXT("land cache: off");
    int32 Valid = 0;
    for (const VoxelLandCachePlan::FLevel& Level : Plan->Levels)
        Valid += Level.bValid ? 1 : 0;
    const FString Pending = Plan->PendingLevel >= 0
        ? FString::Printf(TEXT(", level %d re-centring (%d%%)"), Plan->PendingLevel, Plan->Next.FillRow * 100 / VOXEL_LAND_CACHE_N) : FString();
    return FString::Printf(TEXT("land cache: %d of %d levels written%s, %llu texels written so far"), Valid, (int32)Plan->Levels.size(), *Pending,
        (unsigned long long)TexelsWritten);
}
