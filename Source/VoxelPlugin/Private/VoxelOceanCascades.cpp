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
    // Stage W3z: the shortest band's own grid (its slot, the last), and its mips (512 down to 1).
    constexpr int32 FineSize = VoxelOceanSpectrum::FineSize;
    constexpr int32 FineMips = 10;
    constexpr int32 FineSlot = VoxelOceanSpectrum::SlotCount - 1;
    constexpr int32 LongSlots = VoxelOceanSpectrum::SlotCount - 1;
    constexpr int32 Group = 8;              // the 2D passes' thread groups (VoxelOceanFFT.usf)
    static_assert((Size >> (Mips - 1)) == 1 && (FineSize >> (FineMips - 1)) == 1, "full mip chains");
    static_assert(VoxelOceanSpectrum::SizeOfBand(VoxelOceanSpectrum::BandCount - 1) == FineSize, "the last slot is the shortest band's");
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

// Stage W3z: the grid (VoxelOceanFFT.usf's VOXEL_OCEAN_FFT_FINE: 0 the long arrays' 256^2, 1 the
// shortest band's 512^2).
class FVoxelOceanFFTFineDim : SHADER_PERMUTATION_BOOL("VOXEL_OCEAN_FFT_FINE");

class FVoxelOceanInitSpectrumCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanInitSpectrumCS);
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanInitSpectrumCS, FGlobalShader);
    using FPermutationDomain = TShaderPermutationDomain<FVoxelOceanFFTFineDim>;

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
    using FPermutationDomain = TShaderPermutationDomain<FVoxelOceanFFTFineDim>;

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
    using FPermutationDomain = TShaderPermutationDomain<FVoxelOceanFFTFineDim>;

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
    using FPermutationDomain = TShaderPermutationDomain<FVoxelOceanFFTFineDim>;

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
        SHADER_PARAMETER(FVector4f, FoamRule)
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
    UTexture2DArray* MakeArray(const TCHAR* Name, EPixelFormat Format = PF_FloatRGBA, int64 BytesPerTexel = 8, int32 ArraySize = Size,
        int32 ArrayMips = Mips, int32 Slices = VoxelOceanSpectrum::SlotCount)
    {
        // A unique name: every ocean makes its own.
        UTexture2DArray* Texture = NewObject<UTexture2DArray>(GetTransientPackage(),
            MakeUniqueObjectName(GetTransientPackage(), UTexture2DArray::StaticClass(), FName(Name)), RF_Transient);
        if (!Texture) return nullptr;
        FTexturePlatformData* Data = new FTexturePlatformData();
        Data->SizeX = ArraySize;
        Data->SizeY = ArraySize;
        Data->SetNumSlices(Slices);
        Data->PixelFormat = Format;
        for (int32 Mip = 0; Mip < ArrayMips; Mip++)
        {
            const int32 MipSize = FMath::Max(ArraySize >> Mip, 1);
            FTexture2DMipMap* Level = new FTexture2DMipMap(MipSize, MipSize, Slices);
            Data->Mips.Add(Level);
            const int64 Bytes = (int64)MipSize * MipSize * Slices * BytesPerTexel;
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
    // Stage W3z: the shortest band's own cascade, one slice each (made again if an earlier build's
    // size is held).
    auto Fine = [](UTexture2DArray* Held, const TCHAR* Name, EPixelFormat Format, int64 Bytes)
    {
        return Held && Held->GetPixelFormat() == Format && Held->GetSizeX() == VOC::FineSize ? Held
            : VOC::MakeArray(Name, Format, Bytes, VOC::FineSize, VOC::FineMips, 1);
    };
    Made.FineDisplacement = Fine(Made.FineDisplacement, TEXT("VoxelOceanFineDisplacement"), PF_FloatRGBA, 8);
    Made.FineDerivatives = Fine(Made.FineDerivatives, TEXT("VoxelOceanFineDerivatives"), PF_FloatRGBA, 8);
    Made.FineFoam = Fine(Made.FineFoam, TEXT("VoxelOceanFineFoam"), PF_G16R16F, 4);
    Made.Charts = VOC::MakeChartTexture(Charts);
    if (!Made.IsValid() || !Made.Displacement->GetResource() || !Made.Derivatives->GetResource() || !Made.Charts->GetResource()
        || !Made.Foam->GetResource() || !Made.FineDisplacement->GetResource() || !Made.FineDerivatives->GetResource()
        || !Made.FineFoam->GetResource())
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] Couldn't create the wave cascades' textures for %s."), *GetNameSafe(Owner));
        return false;
    }
    InOut = Made;
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Wave cascades: 2 arrays of %d x %d x %d with mips and a foam array, the shortest band's own %d x %d, chart table %d x %d (level %d)."),
        VOC::Size, VOC::Size, VoxelOceanSpectrum::SlotCount, VOC::FineSize, VOC::FineSize, Charts.Width, Charts.Height, Charts.Level);
    return true;
}

// ---------------------------------------------------------------------------
// Cascades
// ---------------------------------------------------------------------------

struct FVoxelOceanCascades::FRenderState
{
    // Render thread only. The start spectra and the modes' frequencies, per grid (stage W3z: the long
    // arrays' slots, and the shortest band's own).
    TRefCountPtr<FRDGPooledBuffer> Spectrum;
    TRefCountPtr<FRDGPooledBuffer> Multiples;
    TRefCountPtr<FRDGPooledBuffer> FineSpectrum;
    TRefCountPtr<FRDGPooledBuffer> FineMultiples;
    /** The fine foam kept between frames (stage W3w), per texel of the shortest band's own grid. */
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
            RenderState->FineSpectrum.SafeRelease();
            RenderState->FineMultiples.SafeRelease();
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

void FVoxelOceanCascades::Update(uint32 Tick, double Time, const FVector4f& FineFoam)
{
    check(IsInGameThread());
    if (!Textures.IsValid() || Slots.Num() != VoxelOceanSpectrum::SlotCount) return;
    FTextureResource* DisplacementResource = Textures.Displacement->GetResource();
    FTextureResource* DerivativesResource = Textures.Derivatives->GetResource();
    FTextureResource* FoamResource = Textures.Foam->GetResource();
    FTextureResource* FineDisplacementResource = Textures.FineDisplacement->GetResource();
    FTextureResource* FineDerivativesResource = Textures.FineDerivatives->GetResource();
    FTextureResource* FineFoamResource = Textures.FineFoam->GetResource();
    if (!DisplacementResource || !DerivativesResource || !FoamResource || !FineDisplacementResource || !FineDerivativesResource
        || !FineFoamResource) return;
    // The foam's step: the clock's since the last update (held to a quarter second, so a jump of the
    // clock or a hitch doesn't wipe it, and the first update keeps all). With the clock running fast
    // (a time scale past a quarter second a frame) the foam lingers longer against the clock.
    const double Step = bHasTime ? FMath::Clamp(Time - LastTime, 0.0, 0.25) : 0.0;
    LastTime = Time;
    bHasTime = true;
    // Stage W3w: the fine foam's step (VoxelOceanFoam.ush: e^(-Decay dt), and the growth per unit
    // of source over it, (Grow / Decay) (1 - e^(-Decay dt))), in the shortest band's slot (stage
    // W3z: its own grid's only slot, 0 there).
    const bool bFoam = FineFoam.X > 0.0f && FineFoam.Y > 0.0f && FMath::RoundToInt(FineFoam.W) == VOC::FineSlot;
    const double FoamKeep = bFoam ? FMath::Exp(-Step * (double)FineFoam.Y) : 1.0;
    const FVector4f FoamRule = bFoam ? FVector4f(FineFoam.X, (float)FoamKeep, (float)((double)FineFoam.Z / (double)FineFoam.Y * (1.0 - FoamKeep)), 0.0f)
        : FVector4f(0.0f, 1.0f, 0.0f, -1.0f);

    struct FTargets
    {
        FTextureResource* Displacement;
        FTextureResource* Derivatives;
        FTextureResource* Foam;
        FTextureResource* FineDisplacement;
        FTextureResource* FineDerivatives;
        FTextureResource* FineFoam;
    };
    const FTargets Targets = { DisplacementResource, DerivativesResource, FoamResource, FineDisplacementResource, FineDerivativesResource, FineFoamResource };

    ENQUEUE_RENDER_COMMAND(VoxelOceanCascades)(
        [RenderState = State, SlotTable = Slots, Gen = Generation, Tick, Targets, bFoam, FoamRule](FRHICommandListImmediate& RHICmdList)
        {
            FRHITexture* DisplacementRHI = Targets.Displacement->TextureRHI;
            FRHITexture* DerivativesRHI = Targets.Derivatives->TextureRHI;
            FRHITexture* FoamRHI = Targets.Foam->TextureRHI;
            FRHITexture* FineDisplacementRHI = Targets.FineDisplacement->TextureRHI;
            FRHITexture* FineDerivativesRHI = Targets.FineDerivatives->TextureRHI;
            FRHITexture* FineFoamRHI = Targets.FineFoam->TextureRHI;
            FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            // The non-asserting lookup (the shaders may still be compiling, or failed): both grids'
            // permutations.
            auto Has = [ShaderMap](FShaderType& Type, int32 Permutations)
            {
                for (int32 Id = 0; Id < Permutations; Id++)
                    if (!ShaderMap || !ShaderMap->GetShader(&Type, Id).IsValid()) return false;
                return true;
            };
            const bool bReady = DisplacementRHI && DerivativesRHI && FoamRHI && FineDisplacementRHI && FineDerivativesRHI && FineFoamRHI
                && Has(FVoxelOceanInitSpectrumCS::GetStaticType(), 2) && Has(FVoxelOceanTimeStepCS::GetStaticType(), 2)
                && Has(FVoxelOceanFFTCS::GetStaticType(), 2) && Has(FVoxelOceanAssembleCS::GetStaticType(), 2)
                && Has(FVoxelOceanMipCS::GetStaticType(), 1) && Has(FVoxelOceanFoamMipCS::GetStaticType(), 1);
            if (!bReady)
            {
                if (!RenderState->bReportedFailure)
                    UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] The wave cascades can't run (textures or shaders missing): the waves stay flat on screen."));
                RenderState->bReportedFailure = true;
                return;
            }
            const int32 LongCount = VOC::LongSlots;
            const int32 FineCount = 1;
            const int32 FineElements = VOC::FineSize * VOC::FineSize;

            FRDGBuilder GraphBuilder(RHICmdList);
            {
                RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanCascades, "VoxelOceanCascades");
                // A new slot table: both grids' start spectra again, and the fine foam from nothing (it
                // settles on the waves' compression within seconds).
                const bool bNewSpectrum = RenderState->Generation != Gen || !RenderState->Spectrum.IsValid() || !RenderState->Multiples.IsValid()
                    || !RenderState->FineSpectrum.IsValid() || !RenderState->FineMultiples.IsValid();
                FRDGBufferRef FoamState;
                if (bNewSpectrum || !RenderState->FoamState.IsValid() || RenderState->FoamState->Desc.NumElements != (uint32)FineElements)
                {
                    FoamState = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), FineElements), TEXT("Voxel.OceanFoamState"));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(FoamState), 0u);
                    RenderState->FoamState = GraphBuilder.ConvertToExternalBuffer(FoamState);
                }
                else
                {
                    FoamState = GraphBuilder.RegisterExternalBuffer(RenderState->FoamState);
                }
                // The long grid runs no foam: its assembly's foam state a stand-in, never written.
                FRDGBufferRef NoFoamState = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), 1), TEXT("Voxel.OceanNoFoamState"));
                AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(NoFoamState), 0u);

                // The work textures (every mip): the long arrays' eight slices (slice 7 the fine grid's
                // mip 1, copied below), the fine grid's one.
                auto MakeWork = [&GraphBuilder](int32 N, int32 MipCount, int32 Slices, EPixelFormat Format, const TCHAR* Name)
                {
                    return GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(N, N), Format, FClearValueBinding::None,
                        TexCreate_ShaderResource | TexCreate_UAV, (uint16)Slices, (uint8)MipCount), Name);
                };
                const int32 AllSlots = VoxelOceanSpectrum::SlotCount;
                FRDGTextureRef Displacement = MakeWork(VOC::Size, VOC::Mips, AllSlots, PF_FloatRGBA, TEXT("Voxel.OceanDisplacementWork"));
                FRDGTextureRef Derivatives = MakeWork(VOC::Size, VOC::Mips, AllSlots, PF_FloatRGBA, TEXT("Voxel.OceanDerivativesWork"));
                FRDGTextureRef Foam = MakeWork(VOC::Size, VOC::Mips, AllSlots, PF_G16R16F, TEXT("Voxel.OceanFoamWork"));
                FRDGTextureRef FineDisplacement = MakeWork(VOC::FineSize, VOC::FineMips, FineCount, PF_FloatRGBA, TEXT("Voxel.OceanFineDisplacementWork"));
                FRDGTextureRef FineDerivatives = MakeWork(VOC::FineSize, VOC::FineMips, FineCount, PF_FloatRGBA, TEXT("Voxel.OceanFineDerivativesWork"));
                FRDGTextureRef FineFoam = MakeWork(VOC::FineSize, VOC::FineMips, FineCount, PF_G16R16F, TEXT("Voxel.OceanFineFoamWork"));

                // One grid's passes, its slots alone in its slot table: the start spectrum once per
                // slot table, the time step, the row and column FFTs, the assembly into mip 0.
                auto RunGrid = [&](bool bFine, TConstArrayView<VoxelOceanSpectrum::FSlotGPU> GridSlots, TRefCountPtr<FRDGPooledBuffer>& PooledSpectrum,
                    TRefCountPtr<FRDGPooledBuffer>& PooledMultiples, FRDGBufferRef GridFoamState, uint32 FoamOn, const FVector4f& Rule,
                    FRDGTextureRef OutDisplacement, FRDGTextureRef OutDerivatives, FRDGTextureRef OutFoam)
                {
                    const int32 N = bFine ? VOC::FineSize : VOC::Size;
                    const uint32 Count = (uint32)GridSlots.Num();
                    const uint32 Elements = Count * (uint32)N * (uint32)N;
                    FVoxelOceanFFTCS::FPermutationDomain Permutation;
                    Permutation.Set<FVoxelOceanFFTFineDim>(bFine);
                    FRDGBufferSRVRef SlotsSRV = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer(GraphBuilder,
                        bFine ? TEXT("Voxel.OceanFineSlots") : TEXT("Voxel.OceanSlots"), GridSlots));
                    const FIntVector Grid2D(N / VOC::Group, N / VOC::Group, (int32)Count);

                    FRDGBufferRef Spectrum;
                    FRDGBufferRef Multiples;
                    if (bNewSpectrum)
                    {
                        Spectrum = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), Elements),
                            bFine ? TEXT("Voxel.OceanFineSpectrum") : TEXT("Voxel.OceanSpectrum"));
                        Multiples = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), Elements),
                            bFine ? TEXT("Voxel.OceanFineMultiples") : TEXT("Voxel.OceanMultiples"));
                        FVoxelOceanInitSpectrumCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanInitSpectrumCS::FParameters>();
                        P->Slots = SlotsSRV;
                        P->SlotCount = Count;
                        P->SpectrumOut = GraphBuilder.CreateUAV(Spectrum);
                        P->MultiplesOut = GraphBuilder.CreateUAV(Multiples);
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean InitSpectrum %s", bFine ? TEXT("fine") : TEXT("long")),
                            TShaderMapRef<FVoxelOceanInitSpectrumCS>(ShaderMap, Permutation), P, Grid2D);
                        PooledSpectrum = GraphBuilder.ConvertToExternalBuffer(Spectrum);
                        PooledMultiples = GraphBuilder.ConvertToExternalBuffer(Multiples);
                    }
                    else
                    {
                        Spectrum = GraphBuilder.RegisterExternalBuffer(PooledSpectrum);
                        Multiples = GraphBuilder.RegisterExternalBuffer(PooledMultiples);
                    }

                    // The time step: four packed field spectra per slot, two float4 per mode.
                    FRDGBufferRef Fields = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), 2 * Elements),
                        bFine ? TEXT("Voxel.OceanFineFields") : TEXT("Voxel.OceanFields"));
                    {
                        FVoxelOceanTimeStepCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanTimeStepCS::FParameters>();
                        P->Slots = SlotsSRV;
                        P->SlotCount = Count;
                        P->Spectrum = GraphBuilder.CreateSRV(Spectrum);
                        P->Multiples = GraphBuilder.CreateSRV(Multiples);
                        P->FieldsOut = GraphBuilder.CreateUAV(Fields);
                        P->Tick = Tick;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean TimeStep %s", bFine ? TEXT("fine") : TEXT("long")),
                            TShaderMapRef<FVoxelOceanTimeStepCS>(ShaderMap, Permutation), P, Grid2D);
                    }

                    // The inverse FFTs: rows (the empty ones skipped), then columns.
                    for (int32 Pass = 0; Pass < 2; Pass++)
                    {
                        const bool bRows = Pass == 0;
                        FVoxelOceanFFTCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanFFTCS::FParameters>();
                        P->Slots = SlotsSRV;
                        P->SlotCount = Count;
                        P->FFTData = GraphBuilder.CreateUAV(Fields);
                        P->FFTLineStride = bRows ? (uint32)N : 1u;
                        P->FFTElementStride = bRows ? 1u : (uint32)N;
                        P->FFTRowPass = bRows ? 1u : 0u;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean FFT %s %s", bRows ? TEXT("rows") : TEXT("columns"), bFine ? TEXT("fine") : TEXT("long")),
                            TShaderMapRef<FVoxelOceanFFTCS>(ShaderMap, Permutation), P, FIntVector(N, 2 * (int32)Count, 1));
                    }

                    // The outputs (mip 0).
                    {
                        FVoxelOceanAssembleCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanAssembleCS::FParameters>();
                        P->Slots = SlotsSRV;
                        P->SlotCount = Count;
                        P->Fields = GraphBuilder.CreateSRV(Fields);
                        P->DisplacementOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(OutDisplacement, 0));
                        P->DerivativesOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(OutDerivatives, 0));
                        P->FoamState = GraphBuilder.CreateUAV(GridFoamState);
                        P->FoamOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(OutFoam, 0));
                        P->FoamFall = 0.0f;
                        P->FoamOn = FoamOn;
                        P->FoamRule = Rule;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean Assemble %s", bFine ? TEXT("fine") : TEXT("long")),
                            TShaderMapRef<FVoxelOceanAssembleCS>(ShaderMap, Permutation), P, Grid2D);
                    }
                };

                // A grid's mips 1 on: the 2 x 2 means of every field.
                auto MakeMips = [&GraphBuilder, ShaderMap](FRDGTextureRef MipDisplacement, FRDGTextureRef MipDerivatives, FRDGTextureRef MipFoam, int32 N,
                    int32 MipCount, uint32 Count)
                {
                    for (int32 Mip = 1; Mip < MipCount; Mip++)
                    {
                        const int32 MipSize = FMath::Max(N >> Mip, 1);
                        const FIntVector Groups(FMath::DivideAndRoundUp(MipSize, VOC::Group), FMath::DivideAndRoundUp(MipSize, VOC::Group), (int32)Count);
                        FVoxelOceanMipCS::FParameters* P = GraphBuilder.AllocParameters<FVoxelOceanMipCS::FParameters>();
                        P->SlotCount = Count;
                        P->MipSize = (uint32)MipSize;
                        P->DisplacementMipIn = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::CreateForMipLevel(MipDisplacement, Mip - 1));
                        P->DerivativesMipIn = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::CreateForMipLevel(MipDerivatives, Mip - 1));
                        P->DisplacementMipOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(MipDisplacement, (uint8)Mip));
                        P->DerivativesMipOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(MipDerivatives, (uint8)Mip));
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean Mip %d (%d)", Mip, N), TShaderMapRef<FVoxelOceanMipCS>(ShaderMap), P, Groups);
                        FVoxelOceanFoamMipCS::FParameters* F = GraphBuilder.AllocParameters<FVoxelOceanFoamMipCS::FParameters>();
                        F->SlotCount = Count;
                        F->MipSize = (uint32)MipSize;
                        F->FoamMipIn = GraphBuilder.CreateSRV(FRDGTextureSRVDesc::CreateForMipLevel(MipFoam, Mip - 1));
                        F->FoamMipOut = GraphBuilder.CreateUAV(FRDGTextureUAVDesc(MipFoam, (uint8)Mip));
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOcean Foam mip %d (%d)", Mip, N), TShaderMapRef<FVoxelOceanFoamMipCS>(ShaderMap), F, Groups);
                    }
                };

                const TConstArrayView<VoxelOceanSpectrum::FSlotGPU> AllTable(SlotTable);
                // The long grid: slots 0-6.
                RunGrid(false, AllTable.Slice(0, LongCount), RenderState->Spectrum, RenderState->Multiples, NoFoamState, 0u,
                    FVector4f(0.0f, 1.0f, 0.0f, -1.0f), Displacement, Derivatives, Foam);
                // The fine grid: the shortest band's slot, its fine foam, and its mips.
                RunGrid(true, AllTable.Slice(VOC::FineSlot, FineCount), RenderState->FineSpectrum, RenderState->FineMultiples, FoamState, bFoam ? 1u : 0u,
                    FoamRule, FineDisplacement, FineDerivatives, FineFoam);
                RenderState->Generation = Gen;
                MakeMips(FineDisplacement, FineDerivatives, FineFoam, VOC::FineSize, VOC::FineMips, (uint32)FineCount);
                // Its mip 1 into the long arrays' slice 7 (their band 3 at half its resolution), then
                // the long arrays' mips (slice 7's the fine grid's own from 1 on: the same means).
                auto CopyFineIntoLong = [&GraphBuilder](FRDGTextureRef Source, FRDGTextureRef Dest)
                {
                    FRHICopyTextureInfo Copy;
                    Copy.Size = FIntVector(VOC::Size, VOC::Size, 1);
                    Copy.SourceMipIndex = 1;
                    Copy.DestMipIndex = 0;
                    Copy.NumMips = 1;
                    Copy.SourceSliceIndex = 0;
                    Copy.DestSliceIndex = (uint32)VOC::FineSlot;
                    Copy.NumSlices = 1;
                    AddCopyTexturePass(GraphBuilder, Source, Dest, Copy);
                };
                CopyFineIntoLong(FineDisplacement, Displacement);
                CopyFineIntoLong(FineDerivatives, Derivatives);
                CopyFineIntoLong(FineFoam, Foam);
                MakeMips(Displacement, Derivatives, Foam, VOC::Size, VOC::Mips, (uint32)AllSlots);

                // Into the textures the materials and passes read: every mip they have, matched by size.
                auto CopyOut = [&GraphBuilder](FRDGTextureRef Source, FRHITexture* Target, const TCHAR* Name, int32 N, int32 MipCount)
                {
                    FRDGTextureRef Dest = RegisterExternalTexture(GraphBuilder, Target, Name);
                    const FRDGTextureDesc& DestDesc = Dest->Desc;
                    int32 Skip = 0;
                    while (Skip < MipCount - 1 && (N >> Skip) > DestDesc.Extent.X) Skip++;
                    if ((N >> Skip) == DestDesc.Extent.X && DestDesc.Format == Source->Desc.Format)
                    {
                        FRHICopyTextureInfo Copy;
                        Copy.SourceMipIndex = (uint32)Skip;
                        Copy.DestMipIndex = 0;
                        Copy.NumMips = (uint32)FMath::Min<int32>(DestDesc.NumMips, MipCount - Skip);
                        Copy.SourceSliceIndex = 0;
                        Copy.DestSliceIndex = 0;
                        Copy.NumSlices = (uint32)FMath::Min<int32>((int32)Source->Desc.ArraySize, (int32)DestDesc.ArraySize);
                        AddCopyTexturePass(GraphBuilder, Source, Dest, Copy);
                    }
                    GraphBuilder.SetTextureAccessFinal(Dest, ERHIAccess::SRVMask);
                };
                CopyOut(Displacement, DisplacementRHI, TEXT("Voxel.OceanDisplacement"), VOC::Size, VOC::Mips);
                CopyOut(Derivatives, DerivativesRHI, TEXT("Voxel.OceanDerivatives"), VOC::Size, VOC::Mips);
                CopyOut(Foam, FoamRHI, TEXT("Voxel.OceanFoam"), VOC::Size, VOC::Mips);
                CopyOut(FineDisplacement, FineDisplacementRHI, TEXT("Voxel.OceanFineDisplacement"), VOC::FineSize, VOC::FineMips);
                CopyOut(FineDerivatives, FineDerivativesRHI, TEXT("Voxel.OceanFineDerivatives"), VOC::FineSize, VOC::FineMips);
                CopyOut(FineFoam, FineFoamRHI, TEXT("Voxel.OceanFineFoam"), VOC::FineSize, VOC::FineMips);
            }
            GraphBuilder.Execute();
        });
}
