#include "VoxelOceanCascades.h"
#include "VoxelGPUCommon.h"
#include "VoxelPlugin.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "Engine/Texture2D.h"
#include "Engine/Texture2DArray.h"
#include "GlobalShader.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderTargetPool.h"
#include "RenderingThread.h"
#include "ShaderParameterStruct.h"
#include "TextureResource.h"
#include "UObject/Package.h"

DECLARE_GPU_STAT(VoxelOceanCascades);

const FName VoxelOceanCascades::DisplacementName(TEXT("VoxelOceanDisplacement"));
const FName VoxelOceanCascades::DerivativesName(TEXT("VoxelOceanDerivatives"));
const FName VoxelOceanCascades::ChartsName(TEXT("VoxelOceanCharts"));
const FName VoxelOceanCascades::FoamName(TEXT("VoxelOceanFoam"));

// The foam's shared constants (VOXEL_OCEAN_FOAM_CHOP, the envelope; VoxelOceanFoam.ush).
namespace VoxelShader
{
#include "VoxelOceanFoam.ush"
}

namespace VoxelOceanCascadesDetail
{
    constexpr int32 Size = VoxelOceanSpectrum::GPUSize;
    constexpr int32 Mips = 9;               // 256 down to 1
    constexpr int32 Group = 8;              // the 2D passes' thread groups (VoxelOceanFFT.usf)
}

namespace VOC = VoxelOceanCascadesDetail;

// ---------------------------------------------------------------------------
// Shaders (VoxelOceanFFT.usf). Parameter names must match the shader's exactly.
// ---------------------------------------------------------------------------

#define VOXEL_OCEAN_FFT_SHADER_COMMON(ClassName) \
    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
    { \
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
    } \
    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment) \
    { \
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment); \
        /* The spectrum's float steps as the CPU's (the dispersion itself is integer). */ \
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath); \
    }

class FVoxelOceanInitSpectrumCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanInitSpectrumCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanInitSpectrumCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOceanSlotGPU>, Slots)
        SHADER_PARAMETER(uint32, SlotCount)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, SpectrumOut)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, MultiplesOut)
    END_SHADER_PARAMETER_STRUCT()

    VOXEL_OCEAN_FFT_SHADER_COMMON(FVoxelOceanInitSpectrumCS)
};

class FVoxelOceanTimeStepCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanTimeStepCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanTimeStepCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOceanSlotGPU>, Slots)
        SHADER_PARAMETER(uint32, SlotCount)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, Spectrum)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, Multiples)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, FieldsOut)
        SHADER_PARAMETER(uint32, Tick)
    END_SHADER_PARAMETER_STRUCT()

    VOXEL_OCEAN_FFT_SHADER_COMMON(FVoxelOceanTimeStepCS)
};

class FVoxelOceanFFTCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanFFTCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanFFTCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOceanSlotGPU>, Slots)
        SHADER_PARAMETER(uint32, SlotCount)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, FFTData)
        SHADER_PARAMETER(uint32, FFTLineStride)
        SHADER_PARAMETER(uint32, FFTElementStride)
        SHADER_PARAMETER(uint32, FFTRowPass)
    END_SHADER_PARAMETER_STRUCT()

    VOXEL_OCEAN_FFT_SHADER_COMMON(FVoxelOceanFFTCS)
};

class FVoxelOceanAssembleCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanAssembleCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanAssembleCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOceanSlotGPU>, Slots)
        SHADER_PARAMETER(uint32, SlotCount)
        SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, Fields)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, DisplacementOut)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, DerivativesOut)
        SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, FoamState)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float2>, FoamOut)
        SHADER_PARAMETER(float, FoamFall)
        SHADER_PARAMETER(uint32, FoamOn)
    END_SHADER_PARAMETER_STRUCT()

    VOXEL_OCEAN_FFT_SHADER_COMMON(FVoxelOceanAssembleCS)
};

class FVoxelOceanMipCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanMipCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanMipCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER(uint32, SlotCount)
        SHADER_PARAMETER(uint32, MipSize)
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2DArray<float4>, DisplacementMipIn)
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2DArray<float4>, DerivativesMipIn)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, DisplacementMipOut)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, DerivativesMipOut)
    END_SHADER_PARAMETER_STRUCT()

    VOXEL_OCEAN_FFT_SHADER_COMMON(FVoxelOceanMipCS)
};

class FVoxelOceanFoamMipCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanFoamMipCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanFoamMipCS, FGlobalShader);

    BEGIN_SHADER_PARAMETER_STRUCT(FParameters, )
        SHADER_PARAMETER(uint32, SlotCount)
        SHADER_PARAMETER(uint32, MipSize)
        SHADER_PARAMETER_RDG_TEXTURE_SRV(Texture2DArray<float2>, FoamMipIn)
        SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float2>, FoamMipOut)
    END_SHADER_PARAMETER_STRUCT()

    VOXEL_OCEAN_FFT_SHADER_COMMON(FVoxelOceanFoamMipCS)
};

#undef VOXEL_OCEAN_FFT_SHADER_COMMON

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanInitSpectrumCS, "/Plugin/VoxelPlugin/Private/VoxelOceanFFT.usf", "InitSpectrumCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanTimeStepCS, "/Plugin/VoxelPlugin/Private/VoxelOceanFFT.usf", "TimeStepCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanFFTCS, "/Plugin/VoxelPlugin/Private/VoxelOceanFFT.usf", "FFTCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanAssembleCS, "/Plugin/VoxelPlugin/Private/VoxelOceanFFT.usf", "AssembleCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanMipCS, "/Plugin/VoxelPlugin/Private/VoxelOceanFFT.usf", "MipCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanFoamMipCS, "/Plugin/VoxelPlugin/Private/VoxelOceanFFT.usf", "FoamMipCS", SF_Compute);

// ---------------------------------------------------------------------------
// Textures
// ---------------------------------------------------------------------------

namespace VoxelOceanCascadesDetail
{
    /** A transient 2D array, zeroed, with every mip (the copy fills all of them). Its platform
     *  data is complete before its one resource is made (CreateTransient makes a resource at
     *  once, with one mip). */
    UTexture2DArray* MakeArray(const TCHAR* Name, EPixelFormat Format = PF_FloatRGBA, int64 BytesPerTexel = 8)
    {
        // A unique name: every ocean makes its own.
        UTexture2DArray* Texture = NewObject<UTexture2DArray>(GetTransientPackage(),
            MakeUniqueObjectName(GetTransientPackage(), UTexture2DArray::StaticClass(), FName(Name)), RF_Transient);
        if (!Texture) return nullptr;
        FTexturePlatformData* Data = new FTexturePlatformData();
        Data->SizeX = Size;
        Data->SizeY = Size;
        Data->SetNumSlices(VoxelOceanSpectrum::SlotCount);
        Data->PixelFormat = Format;
        for (int32 Mip = 0; Mip < Mips; Mip++)
        {
            const int32 MipSize = FMath::Max(Size >> Mip, 1);
            FTexture2DMipMap* Level = new FTexture2DMipMap(MipSize, MipSize, VoxelOceanSpectrum::SlotCount);
            Data->Mips.Add(Level);
            const int64 Bytes = (int64)MipSize * MipSize * VoxelOceanSpectrum::SlotCount * BytesPerTexel;
            Level->BulkData.Lock(LOCK_READ_WRITE);
            void* Memory = Level->BulkData.Realloc(Bytes);
            FMemory::Memzero(Memory, Bytes);
            Level->BulkData.Unlock();
        }
        Texture->SetPlatformData(Data);
        Texture->SRGB = false;
        Texture->CompressionSettings = TC_HDR;
        Texture->Filter = TF_Trilinear;
        Texture->AddressX = TA_Wrap;
        Texture->AddressY = TA_Wrap;
        Texture->AddressZ = TA_Clamp;
        // No LOD bias may drop the top mips: the copy fills the mips the resource has, by size.
        Texture->LODGroup = TEXTUREGROUP_Pixels2D;
        Texture->NeverStream = true;
        Texture->UpdateResource();
        return Texture;
    }

    UTexture2D* MakeChartTexture(const VoxelOceanCharts::FTable& Table)
    {
        if (!Table.IsValid()) return nullptr;
        UTexture2D* Texture = UTexture2D::CreateTransient(Table.Width, Table.Height, PF_A32B32G32R32F,
            MakeUniqueObjectName(GetTransientPackage(), UTexture2D::StaticClass(), FName(TEXT("VoxelOceanCharts"))));
        if (!Texture || !Texture->GetPlatformData() || Texture->GetPlatformData()->Mips.Num() < 1) return nullptr;
        FTexture2DMipMap& Top = Texture->GetPlatformData()->Mips[0];
        const int64 Bytes = (int64)Table.Texels.size() * sizeof(float);
        void* Memory = Top.BulkData.Lock(LOCK_READ_WRITE);
        if (Top.BulkData.GetBulkDataSize() != Bytes)
            Memory = Top.BulkData.Realloc(Bytes);
        FMemory::Memcpy(Memory, Table.Texels.data(), Bytes);
        Top.BulkData.Unlock();
        Texture->SRGB = false;
        Texture->CompressionSettings = TC_HDR;
        Texture->Filter = TF_Nearest;
        Texture->AddressX = TA_Clamp;
        Texture->AddressY = TA_Clamp;
        Texture->LODGroup = TEXTUREGROUP_Pixels2D;
        Texture->NeverStream = true;
        Texture->UpdateResource();
        return Texture;
    }
}

bool VoxelOceanCascades::MakeTextures(const UObject* Owner, const VoxelOceanCharts::FTable& Charts, FVoxelOceanCascadeTextures& InOut)
{
    // The arrays don't depend on the planet: kept when given; the chart table always new.
    FVoxelOceanCascadeTextures Made = InOut;
    if (!Made.Displacement)
        Made.Displacement = VOC::MakeArray(TEXT("VoxelOceanDisplacement"));
    if (!Made.Derivatives)
        Made.Derivatives = VOC::MakeArray(TEXT("VoxelOceanDerivatives"));
    // The open sea's foam (stage O9b): its envelope and the square, half floats (made again if
    // an earlier build's array is held).
    if (!Made.Foam || Made.Foam->GetPixelFormat() != PF_G16R16F)
        Made.Foam = VOC::MakeArray(TEXT("VoxelOceanFoam"), PF_G16R16F, 4);
    Made.Charts = VOC::MakeChartTexture(Charts);
    if (!Made.IsValid() || !Made.Displacement->GetResource() || !Made.Derivatives->GetResource() || !Made.Charts->GetResource()
        || !Made.Foam->GetResource())
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] Couldn't create the wave cascades' textures for %s."), *GetNameSafe(Owner));
        return false;
    }
    InOut = Made;
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Wave cascades: 2 arrays of %d x %d x %d with mips and a foam array, chart table %d x %d (level %d)."),
        VOC::Size, VOC::Size, VoxelOceanSpectrum::SlotCount, Charts.Width, Charts.Height, Charts.Level);
    return true;
}

// ---------------------------------------------------------------------------
// Cascades
// ---------------------------------------------------------------------------

struct FVoxelOceanCascades::FRenderState
{
    // Render thread only.
    TRefCountPtr<FRDGPooledBuffer> Spectrum;
    TRefCountPtr<FRDGPooledBuffer> Multiples;
    /** The open sea's foam kept between frames (stage O9b), per slot and texel. */
    TRefCountPtr<FRDGPooledBuffer> FoamState;
    uint32 Generation = MAX_uint32;
    bool bReportedFailure = false;
};

FVoxelOceanCascades::FVoxelOceanCascades(const FVoxelOceanCascadeTextures& InTextures)
    : Textures(InTextures)
    , State(MakeShared<FRenderState, ESPMode::ThreadSafe>())
{
}

FVoxelOceanCascades::~FVoxelOceanCascades()
{
    // The pooled buffers go on the render thread, after the commands that use them.
    ENQUEUE_RENDER_COMMAND(VoxelOceanCascadesRelease)([RenderState = State](FRHICommandListImmediate&) mutable
        {
            RenderState->Spectrum.SafeRelease();
            RenderState->Multiples.SafeRelease();
            RenderState->FoamState.SafeRelease();
        });
}

void FVoxelOceanCascades::SetSlots(TConstArrayView<VoxelOceanSpectrum::FSlotGPU> InSlots)
{
    const bool bSame = InSlots.Num() == Slots.Num() && FMemory::Memcmp(InSlots.GetData(), Slots.GetData(), Slots.Num() * sizeof(VoxelOceanSpectrum::FSlotGPU)) == 0;
    if (bSame) return;
    Slots = TArray<VoxelOceanSpectrum::FSlotGPU>(InSlots);
    Generation++;
}

void FVoxelOceanCascades::Update(uint32 Tick, double Time, double FoamDecaySeconds)
{
    check(IsInGameThread());
    if (!Textures.IsValid() || Slots.Num() != VoxelOceanSpectrum::SlotCount) return;
    FTextureResource* DisplacementResource = Textures.Displacement->GetResource();
    FTextureResource* DerivativesResource = Textures.Derivatives->GetResource();
    FTextureResource* FoamResource = Textures.Foam->GetResource();
    if (!DisplacementResource || !DerivativesResource || !FoamResource) return;
    // The foam's fall over the clock's step since the last update (one sigma per decay; the step
    // held to a quarter second, so a jump of the clock or a hitch doesn't wipe it, and the first
    // update keeps all). With the clock running fast (a time scale past a quarter second a frame)
    // the foam lingers longer against the clock.
    const double Step = bHasTime ? FMath::Clamp(Time - LastTime, 0.0, 0.25) : 0.0;
    LastTime = Time;
    bHasTime = true;
    const bool bFoam = FoamDecaySeconds > 0.0;
    const float FoamFall = bFoam ? (float)(Step / FoamDecaySeconds) : 0.0f;

    ENQUEUE_RENDER_COMMAND(VoxelOceanCascades)(
        [RenderState = State, SlotTable = Slots, Gen = Generation, Tick, DisplacementResource, DerivativesResource, FoamResource, bFoam, FoamFall](FRHICommandListImmediate& RHICmdList)
        {
            FRHITexture* DisplacementRHI = DisplacementResource->TextureRHI;
            FRHITexture* DerivativesRHI = DerivativesResource->TextureRHI;
            FRHITexture* FoamRHI = FoamResource->TextureRHI;
            FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            // The non-asserting lookup (the shaders may still be compiling, or failed).
            auto Has = [ShaderMap](FShaderType& Type) { return ShaderMap && ShaderMap->GetShader(&Type, 0).IsValid(); };
            const bool bReady = DisplacementRHI && DerivativesRHI && FoamRHI
                && Has(FVoxelOceanInitSpectrumCS::GetStaticType()) && Has(FVoxelOceanTimeStepCS::GetStaticType())
                && Has(FVoxelOceanFFTCS::GetStaticType()) && Has(FVoxelOceanAssembleCS::GetStaticType())
                && Has(FVoxelOceanMipCS::GetStaticType()) && Has(FVoxelOceanFoamMipCS::GetStaticType());
            if (!bReady)
            {
                if (!RenderState->bReportedFailure)
                    UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The wave cascades can't run (textures or shaders missing): the waves stay flat on screen."));
                RenderState->bReportedFailure = true;
                return;
            }
            const uint32 SlotCount = (uint32)SlotTable.Num();
            const uint32 Elements = SlotCount * VOC::Size * VOC::Size;

            FRDGBuilder GraphBuilder(RHICmdList);
            {
                RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanCascades, "VoxelOceanCascades");
                FRDGBufferSRVRef SlotsSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder, TEXT("Voxel.OceanSlots"), MakeArrayView(SlotTable)));
                const FIntVector Grid2D(VOC::Size / VOC::Group, VOC::Size / VOC::Group, (int32)SlotCount);

                // The start spectrum and the modes' frequencies: once per slot table.
                FRDGBufferRef Spectrum;
                FRDGBufferRef Multiples;
                // The foam's envelope kept between frames: zero when new (and with a new slot table;
                // it settles on the waves' compression within seconds).
                const bool bNewSpectrum = RenderState->Generation != Gen || !RenderState->Spectrum.IsValid() || !RenderState->Multiples.IsValid();
                FRDGBufferRef FoamState;
                if (bNewSpectrum || !RenderState->FoamState.IsValid())
                {
                    FoamState = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), Elements), TEXT("Voxel.OceanFoamState"));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(FoamState), 0u);
                    RenderState->FoamState = GraphBuilder.ConvertToExternalBuffer(FoamState);
                }
                else
                {
                    FoamState = GraphBuilder.RegisterExternalBuffer(RenderState->FoamState);
                }
                if (bNewSpectrum)
                {
                    Spectrum = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), Elements), TEXT("Voxel.OceanSpectrum"));
                    Multiples = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), Elements), TEXT("Voxel.OceanMultiples"));
                    FVoxelOceanInitSpectrumCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanInitSpectrumCS::FParameters>();
                    P->Slots = SlotsSRV;
                    P->SlotCount = SlotCount;
                    P->SpectrumOut = GraphBuilder.CreateUAV(Spectrum);
                    P->MultiplesOut = GraphBuilder.CreateUAV(Multiples);
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean InitSpectrum"), TShaderMapRef<FVoxelOceanInitSpectrumCS>(ShaderMap), P, Grid2D);
                    RenderState->Spectrum = GraphBuilder.ConvertToExternalBuffer(Spectrum);
                    RenderState->Multiples = GraphBuilder.ConvertToExternalBuffer(Multiples);
                    RenderState->Generation = Gen;
                }
                else
                {
                    Spectrum = GraphBuilder.RegisterExternalBuffer(RenderState->Spectrum);
                    Multiples = GraphBuilder.RegisterExternalBuffer(RenderState->Multiples);
                }

                // The time step: four packed field spectra per slot, two float4 per mode.
                FRDGBufferRef Fields = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), 2 * Elements), TEXT("Voxel.OceanFields"));
                {
                    FVoxelOceanTimeStepCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanTimeStepCS::FParameters>();
                    P->Slots = SlotsSRV;
                    P->SlotCount = SlotCount;
                    P->Spectrum = GraphBuilder.CreateSRV(Spectrum);
                    P->Multiples = GraphBuilder.CreateSRV(Multiples);
                    P->FieldsOut = GraphBuilder.CreateUAV(Fields);
                    P->Tick = Tick;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean TimeStep"), TShaderMapRef<FVoxelOceanTimeStepCS>(ShaderMap), P, Grid2D);
                }

                // The inverse FFTs: rows (the empty ones skipped), then columns.
                for (int32 Pass = 0; Pass < 2; Pass++)
                {
                    const bool bRows = Pass == 0;
                    FVoxelOceanFFTCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanFFTCS::FParameters>();
                    P->Slots = SlotsSRV;
                    P->SlotCount = SlotCount;
                    P->FFTData = GraphBuilder.CreateUAV(Fields);
                    P->FFTLineStride = bRows ? (uint32)VOC::Size : 1u;
                    P->FFTElementStride = bRows ? 1u : (uint32)VOC::Size;
                    P->FFTRowPass = bRows ? 1u : 0u;
                    FComputeShaderUtils::AddPass(GraphBuilder, bRows ? RDG_EVENT_NAME("VoxelOcean FFT rows") : RDG_EVENT_NAME("VoxelOcean FFT columns"),
                        TShaderMapRef<FVoxelOceanFFTCS>(ShaderMap), P, FIntVector(VOC::Size, 2 * (int32)SlotCount, 1));
                }

                // The outputs and their mips.
                const FRDGTextureDesc OutDesc = FRDGTextureDesc::Create2DArray(FIntPoint(VOC::Size, VOC::Size), PF_FloatRGBA, FClearValueBinding::None,
                    TexCreate_ShaderResource | TexCreate_UAV, (uint16)SlotCount, (uint8)VOC::Mips);
                FRDGTextureRef Displacement = GraphBuilder.CreateTexture(OutDesc, TEXT("Voxel.OceanDisplacementWork"));
                FRDGTextureRef Derivatives = GraphBuilder.CreateTexture(OutDesc, TEXT("Voxel.OceanDerivativesWork"));
                const FRDGTextureDesc FoamDesc = FRDGTextureDesc::Create2DArray(FIntPoint(VOC::Size, VOC::Size), PF_G16R16F, FClearValueBinding::None,
                    TexCreate_ShaderResource | TexCreate_UAV, (uint16)SlotCount, (uint8)VOC::Mips);
                FRDGTextureRef Foam = GraphBuilder.CreateTexture(FoamDesc, TEXT("Voxel.OceanFoamWork"));
                {
                    FVoxelOceanAssembleCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanAssembleCS::FParameters>();
                    P->Slots = SlotsSRV;
                    P->SlotCount = SlotCount;
                    P->Fields = GraphBuilder.CreateSRV(Fields);
                    P->DisplacementOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Displacement, 0));
                    P->DerivativesOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Derivatives, 0));
                    P->FoamState = GraphBuilder.CreateUAV(FoamState);
                    P->FoamOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Foam, 0));
                    P->FoamFall = FoamFall;
                    P->FoamOn = bFoam ? 1u : 0u;
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean Assemble"), TShaderMapRef<FVoxelOceanAssembleCS>(ShaderMap), P, Grid2D);
                }
                for (int32 Mip = 1; Mip < VOC::Mips; Mip++)
                {
                    const int32 MipSize = FMath::Max(VOC::Size >> Mip, 1);
                    FVoxelOceanMipCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanMipCS::FParameters>();
                    P->SlotCount = SlotCount;
                    P->MipSize = (uint32)MipSize;
                    P->DisplacementMipIn = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::CreateForMipLevel(Displacement, Mip - 1));
                    P->DerivativesMipIn = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::CreateForMipLevel(Derivatives, Mip - 1));
                    P->DisplacementMipOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Displacement, (uint8)Mip));
                    P->DerivativesMipOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Derivatives, (uint8)Mip));
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean Mip %d", Mip), TShaderMapRef<FVoxelOceanMipCS>(ShaderMap), P,
                        FIntVector(FMath::DivideAndRoundUp(MipSize, VOC::Group), FMath::DivideAndRoundUp(MipSize, VOC::Group), (int32)SlotCount));
                    FVoxelOceanFoamMipCS::FParameters* F = GraphBuilder.AllocParameters<FVoxelOceanFoamMipCS::FParameters>();
                    F->SlotCount = SlotCount;
                    F->MipSize = (uint32)MipSize;
                    F->FoamMipIn = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::CreateForMipLevel(Foam, Mip - 1));
                    F->FoamMipOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(Foam, (uint8)Mip));
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean Foam mip %d", Mip), TShaderMapRef<FVoxelOceanFoamMipCS>(ShaderMap), F,
                        FIntVector(FMath::DivideAndRoundUp(MipSize, VOC::Group), FMath::DivideAndRoundUp(MipSize, VOC::Group), (int32)SlotCount));
                }

                // Into the textures the materials read: every mip they have, matched by size.
                auto CopyOut = [&GraphBuilder, SlotCount](FRDGTextureRef Source, FRHITexture* Target, const TCHAR* Name)
                {
                    FRDGTextureRef Dest = RegisterExternalTexture(GraphBuilder, Target, Name);
                    const FRDGTextureDesc& DestDesc = Dest->Desc;
                    int32 Skip = 0;
                    while (Skip < VOC::Mips - 1 && (VOC::Size >> Skip) > DestDesc.Extent.X) Skip++;
                    if ((VOC::Size >> Skip) == DestDesc.Extent.X && DestDesc.Format == Source->Desc.Format)
                    {
                        FRHICopyTextureInfo Copy;
                        Copy.SourceMipIndex = (uint32)Skip;
                        Copy.DestMipIndex = 0;
                        Copy.NumMips = (uint32)FMath::Min<int32>(DestDesc.NumMips, VOC::Mips - Skip);
                        Copy.SourceSliceIndex = 0;
                        Copy.DestSliceIndex = 0;
                        Copy.NumSlices = (uint32)FMath::Min<int32>((int32)SlotCount, (int32)DestDesc.ArraySize);
                        AddCopyTexturePass(GraphBuilder, Source, Dest, Copy);
                    }
                    GraphBuilder.SetTextureAccessFinal(Dest, ERHIAccess::SRVMask);
                };
                CopyOut(Displacement, DisplacementRHI, TEXT("Voxel.OceanDisplacement"));
                CopyOut(Derivatives, DerivativesRHI, TEXT("Voxel.OceanDerivatives"));
                CopyOut(Foam, FoamRHI, TEXT("Voxel.OceanFoam"));
            }
            GraphBuilder.Execute();
        });
}
