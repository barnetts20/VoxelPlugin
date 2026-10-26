#include "VoxelUnderwaterViewExtension.h"
#include "VoxelOceanGPUTime.h"
#include "VoxelUnderwaterMap.h"
#include "VoxelOceanShoreField.h"
#include "VoxelOceanDepthProbe.h"
#include "VoxelGPUCommon.h"
#include "AtmosphereViewExtension.h"

#include "DataDrivenShaderPlatformInfo.h"
#include "GlobalShader.h"
#include "PostProcess/PostProcessMaterialInputs.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderUtils.h"
#include "RHIStaticStates.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "ScreenPass.h"
#include "ShaderCompilerCore.h"
#include "ShaderParameterStruct.h"
#include "TextureResource.h"

DECLARE_GPU_STAT(VoxelUnderwater);

namespace VoxelUnderwaterDetail
{
    constexpr int32 ThreadGroupSize = 8;
    /** Slack (cm) on the test of whether a view can see the water, for the float heights. */
    constexpr double ViewSlack = 1.0;
    /** Stage W8b: the camera's probe: its grid's spacing (cm; 8 x 8 points round the camera's
     *  foot), the readings in flight at most, how old a reading may be (s) and how far the camera
     *  may have moved from it (cm) to be used, the waves' rise per second (cm) and the base margin
     *  (cm) the test allows. */
    constexpr double CameraProbeSpacing = 25.0;
    constexpr int32 CameraProbeInFlight = 4;
    constexpr double CameraProbeMaxAge = 0.5;
    constexpr double CameraProbeMaxMove = 2000.0;
    constexpr double CameraProbeRise = 300.0;
    constexpr double CameraProbeMargin = 30.0;
    /** Stage A8: the caustics' levels, slices and splat margin (texels): VoxelUnderwaterMap.ush's and
     *  VoxelUnderwaterPass.usf's VOXEL_UNDERWATER_CAUSTIC_LEVELS, _SLICES and _MARGIN. The resolve's
     *  blur per metre of path (m): the sun's size in water (0.007) and where the beam spreads the
     *  splat's sources thin (the test sea's stripes went at 0.019). */
    constexpr int32 CausticLevels = 3;
    constexpr int32 CausticSlices = 8;
    constexpr int32 CausticMargin = 32;
    constexpr float CausticBlur = 0.019f;
}
static_assert(VoxelUnderwaterDetail::CausticLevels <= VoxelUnderwaterMap::Levels, "the caustics' levels are the surface map's first");

// ---------------------------------------------------------------------------
// Shader
//
// Parameter names must match VoxelUnderwaterPass.usf exactly.
// ---------------------------------------------------------------------------

// The sea and its waves, shared by both passes.
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelUnderwaterWaveParameters, )
    SHADER_PARAMETER(FVector3f, UnderwaterCenter)
    SHADER_PARAMETER(FVector2f, UnderwaterSphere)
    SHADER_PARAMETER(float, UnderwaterWaveAmplitude)
    SHADER_PARAMETER(FVector4f, UnderwaterWave0)
    SHADER_PARAMETER(FVector4f, UnderwaterWave1)
    SHADER_PARAMETER(FVector4f, UnderwaterWave2)
    SHADER_PARAMETER(FVector4f, UnderwaterShape)
    SHADER_PARAMETER(FVector4f, UnderwaterFFT)
    SHADER_PARAMETER(FVector4f, UnderwaterRefHigh)
    SHADER_PARAMETER(FVector4f, UnderwaterRefLow)
    SHADER_PARAMETER(FVector4f, UnderwaterRotation)
    SHADER_PARAMETER(FVector4f, UnderwaterTiles)
    SHADER_PARAMETER(FVector4f, UnderwaterStates)
    SHADER_PARAMETER(FVector4f, UnderwaterSlopeVar)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, UnderwaterDisplacement)
    SHADER_PARAMETER_SAMPLER(SamplerState, UnderwaterDisplacementSampler)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, UnderwaterDerivatives)
    SHADER_PARAMETER_SAMPLER(SamplerState, UnderwaterDerivativesSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, UnderwaterCharts)
    // The surface map's levels (VoxelUnderwaterMap.h).
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterMapAnchor, [VoxelUnderwaterMap::Levels])
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterMapE1, [VoxelUnderwaterMap::Levels])
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterMapE2, [VoxelUnderwaterMap::Levels])
END_SHADER_PARAMETER_STRUCT()

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelUnderwaterParameters, )
    SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float4>, UnderwaterSceneColor)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, UnderwaterSceneDepth)
    SHADER_PARAMETER(FUintVector2, UnderwaterViewMin)
    SHADER_PARAMETER(FUintVector2, UnderwaterViewSize)
    SHADER_PARAMETER(FMatrix44f, UnderwaterClipToCamera)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelUnderwaterWaveParameters, Waves)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float>, UnderwaterMap)
    // The foam map and the foam's colour from below (stage O9c).
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float>, UnderwaterFoamMap)
    SHADER_PARAMETER(FVector4f, UnderwaterFoamColor)

    SHADER_PARAMETER(FVector3f, UnderwaterAbsorption)
    SHADER_PARAMETER(FVector3f, UnderwaterScattering)
    SHADER_PARAMETER(FVector3f, UnderwaterLight)
    SHADER_PARAMETER(FVector4f, UnderwaterMeniscus)
    // Stage A4: the light from the atmosphere, and down to the camera's depth.
    SHADER_PARAMETER_RDG_BUFFER_SRV(Buffer<float4>, UnderwaterAtmoSun)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D, UnderwaterAtmoSkyIrradiance)
    SHADER_PARAMETER_SAMPLER(SamplerState, UnderwaterAtmoSkySampler)
    SHADER_PARAMETER(FVector4f, UnderwaterAtmoSkySize)
    SHADER_PARAMETER(FVector3f, UnderwaterSunIlluminance)
    SHADER_PARAMETER(FVector3f, UnderwaterSunDirection)
    SHADER_PARAMETER(uint32, UnderwaterAtmoOn)
    SHADER_PARAMETER(FVector3f, UnderwaterDepthLight)
    // Stage A7: the scattered light.
    SHADER_PARAMETER(uint32, UnderwaterScatterOn)
    SHADER_PARAMETER(float, UnderwaterScatterGain)
    SHADER_PARAMETER(float, UnderwaterPhaseG)
    SHADER_PARAMETER(FVector3f, UnderwaterKd)
    // Stage A7c: the atmosphere's march up from the sea over the camera (UnderwaterAtmoOn bit 2).
    SHADER_PARAMETER_RDG_BUFFER_SRV(Buffer<float4>, UnderwaterAtmoSkyRays)
    // Stage A8: the caustics (UnderwaterCausticOn 0: none; strengths and the sun's blur).
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float>, UnderwaterCausticMap)
    SHADER_PARAMETER_SAMPLER(SamplerState, UnderwaterCausticSampler)
    SHADER_PARAMETER(uint32, UnderwaterCausticOn)
    SHADER_PARAMETER(FVector4f, UnderwaterCaustic)

    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, UnderwaterOutput)
END_SHADER_PARAMETER_STRUCT()

class FVoxelUnderwaterCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelUnderwaterCS);
    using FParameters = FVoxelUnderwaterParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelUnderwaterCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.SetDefine(TEXT("VOXEL_UNDERWATER_THREADS"), VoxelUnderwaterDetail::ThreadGroupSize);
        // The surface map's sizes (VoxelUnderwaterMap.ush) from the CPU's.
        OutEnvironment.SetDefine(TEXT("VOXEL_UNDERWATER_MAP_SIZE"), VoxelUnderwaterMap::Size);
        OutEnvironment.SetDefine(TEXT("VOXEL_UNDERWATER_MAP_LEVELS"), VoxelUnderwaterMap::Levels);
        // The stable sphere height and the shell roots were checked in IEEE float order.
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelUnderwaterCS, "/Plugin/VoxelPlugin/Private/VoxelUnderwaterPass.usf", "UnderwaterCS", SF_Compute);

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelUnderwaterMapParameters, )
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelUnderwaterWaveParameters, Waves)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float>, UnderwaterMapOut)
    // The foam map (stage O9c) from the cascades' foam.
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float>, UnderwaterFoamMapOut)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, UnderwaterFoam)
    SHADER_PARAMETER_SAMPLER(SamplerState, UnderwaterFoamSampler)
    SHADER_PARAMETER(uint32, UnderwaterFoamOn)
    // The shore (stage O8): the field's texture and the depth probe (VoxelOceanDepthProbe.h).
    SHADER_PARAMETER_TEXTURE(Texture2DArray, UnderwaterShore)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, UnderwaterProbe)
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterProbeAnchor, [VoxelOceanDepthProbe::Levels])
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterProbeE1, [VoxelOceanDepthProbe::Levels])
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterProbeE2, [VoxelOceanDepthProbe::Levels])
    SHADER_PARAMETER(uint32, UnderwaterProbeOn)
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterShoreFrame, [7])
    // Stage W8b: the camera's probe (mode 1: its 64 heights here, not the map).
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, UnderwaterCameraOut)
    SHADER_PARAMETER(uint32, UnderwaterCameraMode)
END_SHADER_PARAMETER_STRUCT()

class FVoxelUnderwaterMapCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelUnderwaterMapCS);
    using FParameters = FVoxelUnderwaterMapParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelUnderwaterMapCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FVoxelUnderwaterCS::ModifyCompilationEnvironment(Parameters, OutEnvironment);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelUnderwaterMapCS, "/Plugin/VoxelPlugin/Private/VoxelUnderwaterPass.usf", "UnderwaterMapCS", SF_Compute);

// Stage A8: the caustics' splat (the map's slopes into the accumulation) and resolve (into the map).
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelUnderwaterCausticSplatParameters, )
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterMapAnchor, [VoxelUnderwaterMap::Levels])
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float>, UnderwaterMap)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<uint>, UnderwaterCausticAccumOut)
END_SHADER_PARAMETER_STRUCT()

class FVoxelUnderwaterCausticSplatCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelUnderwaterCausticSplatCS);
    using FParameters = FVoxelUnderwaterCausticSplatParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelUnderwaterCausticSplatCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FVoxelUnderwaterCS::ModifyCompilationEnvironment(Parameters, OutEnvironment);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelUnderwaterCausticSplatCS, "/Plugin/VoxelPlugin/Private/VoxelUnderwaterPass.usf", "UnderwaterCausticSplatCS", SF_Compute);

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelUnderwaterCausticBlurParameters, )
    SHADER_PARAMETER_ARRAY(FVector4f, UnderwaterMapAnchor, [VoxelUnderwaterMap::Levels])
    SHADER_PARAMETER(FVector4f, UnderwaterCaustic)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<uint>, UnderwaterCausticAccum)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float>, UnderwaterCausticTempOut)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float>, UnderwaterCausticTemp)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float>, UnderwaterCausticMapOut)
END_SHADER_PARAMETER_STRUCT()

class FVoxelUnderwaterCausticBlurXCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelUnderwaterCausticBlurXCS);
    using FParameters = FVoxelUnderwaterCausticBlurParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelUnderwaterCausticBlurXCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FVoxelUnderwaterCS::ModifyCompilationEnvironment(Parameters, OutEnvironment);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelUnderwaterCausticBlurXCS, "/Plugin/VoxelPlugin/Private/VoxelUnderwaterPass.usf", "UnderwaterCausticBlurXCS", SF_Compute);

class FVoxelUnderwaterCausticBlurYCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelUnderwaterCausticBlurYCS);
    using FParameters = FVoxelUnderwaterCausticBlurParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelUnderwaterCausticBlurYCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FVoxelUnderwaterCS::ModifyCompilationEnvironment(Parameters, OutEnvironment);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelUnderwaterCausticBlurYCS, "/Plugin/VoxelPlugin/Private/VoxelUnderwaterPass.usf", "UnderwaterCausticBlurYCS", SF_Compute);

// ---------------------------------------------------------------------------
// Extension
// ---------------------------------------------------------------------------

FVoxelUnderwaterViewExtension::FVoxelUnderwaterViewExtension(const FAutoRegister& AutoRegister, UWorld* InWorld)
    : FWorldSceneViewExtension(AutoRegister, InWorld)
{
}

void FVoxelUnderwaterViewExtension::SetFrame_GameThread(const FVoxelUnderwaterParams& InParams)
{
    check(IsInGameThread());

    TSharedRef<FVoxelUnderwaterViewExtension, ESPMode::ThreadSafe> Self =
        StaticCastSharedRef<FVoxelUnderwaterViewExtension>(AsShared());

    ENQUEUE_RENDER_COMMAND(VoxelUnderwaterSetFrame)(
        [Self, Snapshot = InParams](FRHICommandListImmediate&)
        {
            // The resources are alive here (a texture's release is enqueued after this command);
            // the references keep their RHI textures past it.
            auto RHIOf = [](FTextureResource* Resource) { return Resource ? Resource->TextureRHI : FTextureRHIRef(); };
            Self->DisplacementRHI = RHIOf(Snapshot.Displacement);
            Self->DerivativesRHI = RHIOf(Snapshot.Derivatives);
            Self->ChartsRHI = RHIOf(Snapshot.Charts);
            Self->FoamRHI = RHIOf(Snapshot.Foam);
            Self->Params = Snapshot;
            Self->Params.Displacement = nullptr;
            Self->Params.Derivatives = nullptr;
            Self->Params.Charts = nullptr;
            Self->Params.Foam = nullptr;
            Self->bHasFrame = true;
        });
}

void FVoxelUnderwaterViewExtension::SetEnabled(bool bInEnabled)
{
    check(IsInGameThread());
    bEnabled.store(bInEnabled);
}

bool FVoxelUnderwaterViewExtension::IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const
{
    return bEnabled.load() && FWorldSceneViewExtension::IsActiveThisFrame_Internal(Context);
}

void FVoxelUnderwaterViewExtension::SubscribeToPostProcessingPass(EPostProcessingPass Pass, const FSceneView& InView,
    FPostProcessingPassDelegateArray& InOutPassCallbacks, bool bIsPassEnabled)
{
    // Ahead of depth of field, after the atmosphere's composite (Priority). PITFALL:
    // TRANSLUCENCY IS NOT IN SCENE COLOUR YET; it is merged at depth of field.
    if (Pass == EPostProcessingPass::BeforeDOF)
    {
        InOutPassCallbacks.Add(FPostProcessingPassDelegate::CreateRaw(
            this, &FVoxelUnderwaterViewExtension::Render_RenderThread));
    }
}

FScreenPassTexture FVoxelUnderwaterViewExtension::Render_RenderThread(
    FRDGBuilder& GraphBuilder, const FSceneView& View, const FPostProcessMaterialInputs& Inputs)
{
    const FScreenPassTextureSlice SceneColorSlice = Inputs.GetInput(EPostProcessMaterialInput::SceneColor);

    const FSceneTextureUniformParameters* SceneTextures = Inputs.SceneTextures.SceneTextures
        ? Inputs.SceneTextures.SceneTextures->GetContents()
        : nullptr;

    if (!bHasFrame || !Params.IsUsable() || !SceneColorSlice.IsValid()
        || !SceneTextures || !SceneTextures->SceneDepthTexture
        || View.bIsReflectionCapture || View.bIsPlanarReflection
        || !View.IsPerspectiveProjection()
        || View.GetFeatureLevel() < ERHIFeatureLevel::SM5)
    {
        return Inputs.ReturnUntouchedSceneColorForPostProcessing(GraphBuilder);
    }

    // -- This view's camera against the sea, in double ---------------------------

    const FVoxelOceanWaveFrame& Waves = Params.Waves;
    const FVector ViewOrigin = View.ViewMatrices.GetViewOrigin();
    const FVector CameraPlanet = Waves.PlanetToWorld.InverseTransformPositionNoScale(ViewOrigin);
    const double CameraHeight = CameraPlanet.Size() - Waves.Radius();

    // The pixel's ray: the jittered clip space (the depth buffer's) to the view, then the
    // view's rotation back to world axes; a point comes out relative to the camera.
    const FMatrix ClipToCamera = View.ViewMatrices.GetInvProjectionMatrix()
        * View.ViewMatrices.GetInvViewMatrix().RemoveTranslation();

    // The water shows only where a pixel's near-plane point is below the surface or within
    // the meniscus of it. The near plane's farthest point is at a corner of the view.
    double NearReach = 0.0;
    for (int32 Corner = 0; Corner < 4; Corner++)
    {
        const FVector4 H = ClipToCamera.TransformFVector4(FVector4((Corner & 1) ? 1.0 : -1.0, (Corner & 2) ? 1.0 : -1.0, 1.0, 1.0));
        if (H.W > 0.0)
            NearReach = FMath::Max(NearReach, FVector(H.X, H.Y, H.Z).Size() / H.W);
    }
    // (Stage W6i: the waves' reach for the sea state at the camera, not the stormiest sea's.)
    if (CameraHeight - Waves.MaxHeightAt(CameraPlanet) - NearReach > FMath::Max(0.0f, Params.Meniscus.W) + VoxelUnderwaterDetail::ViewSlack)
    {
        return Inputs.ReturnUntouchedSceneColorForPostProcessing(GraphBuilder);
    }
    // A camera in an inland basin below sea level (stage O8: not joined to the sea) is in air.
    if (Waves.Shore.IsValid())
    {
        const FVoxelOceanShore Shore = Waves.Shore->Sample(CameraPlanet);
        if (Shore.HasField() && Shore.Distance < -Params.ShoreMarginCm)
            return Inputs.ReturnUntouchedSceneColorForPostProcessing(GraphBuilder);
    }

    // Stage W8b: the camera probe's readings come back (in order).
    while (CameraReadbacks.Num() > 0 && CameraReadbacks[0].Buffer.IsValid() && CameraReadbacks[0].Buffer->IsReady())
    {
        FCameraReadback& R = CameraReadbacks[0];
        const float* Heights = static_cast<const float*>(R.Buffer->Lock(64u * sizeof(float)));
        if (Heights)
        {
            float Highest = Heights[0];
            for (int32 i = 1; i < 64; i++) Highest = FMath::Max(Highest, Heights[i]);
            R.Buffer->Unlock();
            if (FMath::IsFinite(Highest))
            {
                bHasCameraSurface = true;
                CameraSurfaceMax = (double)Highest;
                CameraSurfaceAt = R.CameraPlanet;
                CameraSurfaceSeconds = R.Seconds;
            }
        }
        CameraReadbacks.RemoveAt(0);
    }

    const FScreenPassTexture SceneColor = FScreenPassTexture::CopyFromSlice(GraphBuilder, SceneColorSlice);
    const FIntRect Rect = SceneColor.ViewRect;
    const FIntPoint OutputSize = Rect.Size();

    if (OutputSize.X <= 0 || OutputSize.Y <= 0)
    {
        return Inputs.ReturnUntouchedSceneColorForPostProcessing(GraphBuilder);
    }

    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelUnderwater, "VoxelUnderwater");
    VoxelOceanGPUTime::FScope OceanGPUTime(GraphBuilder, VoxelOceanGPUTime::Underwater);

    // Into a new texture of the scene colour's size, written inside the view rect only: the
    // passes after this one read within it.
    FRDGTextureDesc OutputDesc = SceneColor.Texture->Desc;
    OutputDesc.Flags |= TexCreate_ShaderResource | TexCreate_UAV;

    FRDGTextureRef Output = GraphBuilder.CreateTexture(OutputDesc, TEXT("VoxelUnderwater.SceneColor"));

    // -- The sea and its waves, from this view's camera (formed here in double) --------------

    FVoxelUnderwaterWaveParameters WaveP;
    WaveP.UnderwaterCenter = FVector3f(Waves.PlanetToWorld.GetLocation() - ViewOrigin);
    WaveP.UnderwaterSphere = FVector2f((float)Waves.Radius(), (float)CameraHeight);
    FVoxelOceanWaveShaderParams WaveParams;
    Waves.ShaderParams(CameraPlanet, WaveParams);
    WaveP.UnderwaterWaveAmplitude = WaveParams.Amplitude;
    WaveP.UnderwaterWave0 = WaveParams.Wave[0];
    WaveP.UnderwaterWave1 = WaveParams.Wave[1];
    WaveP.UnderwaterWave2 = WaveParams.Wave[2];
    WaveP.UnderwaterShape = WaveParams.Shape;
    // The FFT model's parameters and textures; stand-ins (and FFT.x 0) without them.
    FRHITexture* Displacement = DisplacementRHI.GetReference();
    FRHITexture* Derivatives = DerivativesRHI.GetReference();
    FRHITexture* Charts = ChartsRHI.GetReference();
    const bool bFFT = WaveParams.IsFFT() && Displacement && Derivatives && Charts;
    WaveP.UnderwaterFFT = bFFT ? WaveParams.FFT : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    WaveP.UnderwaterRefHigh = WaveParams.RefHigh;
    WaveP.UnderwaterRefLow = WaveParams.RefLow;
    WaveP.UnderwaterRotation = WaveParams.Rotation;
    WaveP.UnderwaterTiles = WaveParams.Tiles;
    WaveP.UnderwaterStates = WaveParams.States;
    WaveP.UnderwaterSlopeVar = WaveParams.SlopeVar;
    WaveP.UnderwaterDisplacement = bFFT ? Displacement : GBlackArrayTexture->TextureRHI.GetReference();
    WaveP.UnderwaterDerivatives = bFFT ? Derivatives : GBlackArrayTexture->TextureRHI.GetReference();
    WaveP.UnderwaterCharts = bFFT ? Charts : GBlackTexture->TextureRHI.GetReference();
    WaveP.UnderwaterDisplacementSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    WaveP.UnderwaterDerivativesSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    const VoxelUnderwaterMap::FLevels Levels = VoxelUnderwaterMap::MakeLevels(CameraPlanet, Waves.Radius(), Waves.PlanetToWorld.GetRotation());
    for (int32 Level = 0; Level < VoxelUnderwaterMap::Levels; Level++)
    {
        WaveP.UnderwaterMapAnchor[Level] = Levels.Anchor[Level];
        WaveP.UnderwaterMapE1[Level] = Levels.E1[Level];
        WaveP.UnderwaterMapE2[Level] = Levels.E2[Level];
    }

    // -- The surface map (the FFT model; a cleared stand-in otherwise) ------------------------

    const int32 MapSize = bFFT ? VoxelUnderwaterMap::Size : 1;
    const int32 MapLevels = bFFT ? VoxelUnderwaterMap::Levels : 1;
    FRDGTextureRef Map = GraphBuilder.CreateTexture(
        FRDGTextureDesc::Create2DArray(FIntPoint(MapSize, MapSize), PF_R32_FLOAT, FClearValueBinding::Black,
            TexCreate_ShaderResource | TexCreate_UAV, MapLevels),
        TEXT("VoxelUnderwater.SurfaceMap"));
    // The foam of the surface above each map point (stage O9c), half floats.
    FRDGTextureRef FoamMap = GraphBuilder.CreateTexture(
        FRDGTextureDesc::Create2DArray(FIntPoint(MapSize, MapSize), PF_R16F, FClearValueBinding::Black,
            TexCreate_ShaderResource | TexCreate_UAV, MapLevels),
        TEXT("VoxelUnderwater.FoamMap"));
    FRHITexture* Foam = FoamRHI.GetReference();
    const bool bFoam = bFFT && Foam && Params.FoamColor.W > 0.0f;
    // The map pass's parameters (the probe's too, stage W8b: the same waves, shore and depth probe).
    FRDGBufferRef CameraOut = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), 64u), TEXT("VoxelUnderwater.CameraProbe"));
    auto MakeMapParams = [&](FRDGTextureUAVRef MapUAV, FRDGTextureUAVRef FoamUAV, bool bFoamOn) -> FVoxelUnderwaterMapParameters*
        {
            FVoxelUnderwaterMapParameters* P = GraphBuilder.AllocParameters<FVoxelUnderwaterMapParameters>();
            P->Waves = WaveP;
            P->UnderwaterMapOut = MapUAV;
            P->UnderwaterFoamMapOut = FoamUAV;
            P->UnderwaterFoam = bFoamOn ? Foam : GBlackArrayTexture->TextureRHI.GetReference();
            P->UnderwaterFoamSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
            P->UnderwaterFoamOn = bFoamOn ? 1u : 0u;
            P->UnderwaterCameraOut = GraphBuilder.CreateUAV(CameraOut);
            P->UnderwaterCameraMode = 0u;
            // The shore field's texture, or a stand-in (the waves then see no field: States.x is 0
            // without one, VoxelOceanWaveShaderParams).
            FRHITexture* ShoreTexture = Params.Shore.IsValid() ? Params.Shore->Texture.GetReference() : nullptr;
            P->UnderwaterShore = ShoreTexture ? ShoreTexture : GBlackArrayTexture->TextureRHI.GetReference();
            if (!ShoreTexture)
            {
                P->Waves.UnderwaterStates.X = 0.0f;
            }
            for (int32 i = 0; i < 7; i++)
                P->UnderwaterShoreFrame[i] = ShoreTexture ? WaveParams.ShoreFrame[i] : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
            // The depth probe, its levels from this view's camera.
            const FVoxelOceanProbeState* Probe = Params.Probe.IsValid() && Params.Probe->bValid && Params.Probe->Depth.IsValid() ? Params.Probe.Get() : nullptr;
            if (Probe)
            {
                P->UnderwaterProbe = GraphBuilder.CreateSRV(GraphBuilder.RegisterExternalBuffer(Probe->Depth));
                const FQuat Rotation = Waves.PlanetToWorld.GetRotation();
                for (int32 Level = 0; Level < VoxelOceanDepthProbe::Levels; Level++)
                {
                    const FVector Anchor = Rotation.RotateVector(Probe->Levels.Anchor[Level] - CameraPlanet);
                    const FVector E1 = Rotation.RotateVector(Probe->Levels.E1[Level]);
                    const FVector E2 = Rotation.RotateVector(Probe->Levels.E2[Level]);
                    P->UnderwaterProbeAnchor[Level] = FVector4f((float)Anchor.X, (float)Anchor.Y, (float)Anchor.Z, (float)Probe->Levels.Spacing[Level]);
                    P->UnderwaterProbeE1[Level] = FVector4f((float)E1.X, (float)E1.Y, (float)E1.Z, 0.0f);
                    P->UnderwaterProbeE2[Level] = FVector4f((float)E2.X, (float)E2.Y, (float)E2.Z, 0.0f);
                }
                P->UnderwaterProbeOn = 1u;
            }
            else
            {
                P->UnderwaterProbe = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<float>(GraphBuilder, TEXT("VoxelUnderwater.NoProbe"), TConstArrayView<float>()));
                for (int32 Level = 0; Level < VoxelOceanDepthProbe::Levels; Level++)
                {
                    P->UnderwaterProbeAnchor[Level] = FVector4f(0.0f, 0.0f, 0.0f, 1.0f);
                    P->UnderwaterProbeE1[Level] = FVector4f(1.0f, 0.0f, 0.0f, 0.0f);
                    P->UnderwaterProbeE2[Level] = FVector4f(0.0f, 1.0f, 0.0f, 0.0f);
                }
                P->UnderwaterProbeOn = 0u;
            }
            return P;
        };

    // -- Stage W8b: the camera's probe, and the test against the last reading --------------------
    // The surface round the camera's foot (64 of the map pass's points, 25 cm apart), read back for
    // the next frames; this frame's view is skipped while the camera stands clear above the last
    // reading (its highest point), by more than the near plane's reach and the meniscus, with
    // margins for the reading's age (the waves' rise) and the camera's travel since (a slope of 1).
    if (bFFT)
    {
        const FVector Up = CameraPlanet.GetSafeNormal();
        const FVector Foot = Waves.PlanetToWorld.GetRotation().RotateVector(Up * Waves.Radius() - CameraPlanet);
        const double T = VoxelUnderwaterDetail::CameraProbeSpacing;
        const double Shift = (0.5 * VoxelUnderwaterMap::Size - 4.0) * T;
        const FVector E1(Levels.E1[0].X, Levels.E1[0].Y, Levels.E1[0].Z);
        const FVector E2(Levels.E2[0].X, Levels.E2[0].Y, Levels.E2[0].Z);
        const FVector Anchor = Foot + (E1 + E2) * Shift;
        FRDGTextureRef ProbeMap = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(1, 1), PF_R32_FLOAT, FClearValueBinding::Black,
            TexCreate_ShaderResource | TexCreate_UAV, 1), TEXT("VoxelUnderwater.CameraProbeMap"));
        FRDGTextureRef ProbeFoam = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(1, 1), PF_R16F, FClearValueBinding::Black,
            TexCreate_ShaderResource | TexCreate_UAV, 1), TEXT("VoxelUnderwater.CameraProbeFoam"));
        FVoxelUnderwaterMapParameters* P = MakeMapParams(GraphBuilder.CreateUAV(ProbeMap), GraphBuilder.CreateUAV(ProbeFoam), false);
        P->Waves.UnderwaterMapAnchor[0] = FVector4f((float)Anchor.X, (float)Anchor.Y, (float)Anchor.Z, (float)T);
        P->UnderwaterCameraMode = 1u;
        TShaderMapRef<FVoxelUnderwaterMapCS> ProbeShader(GetGlobalShaderMap(View.GetFeatureLevel()));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Underwater camera probe"), ProbeShader, P,
            FIntVector(FMath::DivideAndRoundUp(8, VoxelUnderwaterDetail::ThreadGroupSize), FMath::DivideAndRoundUp(8, VoxelUnderwaterDetail::ThreadGroupSize), 1));
        if (CameraReadbacks.Num() < VoxelUnderwaterDetail::CameraProbeInFlight)
        {
            FCameraReadback& R = CameraReadbacks.AddDefaulted_GetRef();
            R.Buffer = MakeUnique<FRHIGPUBufferReadback>(TEXT("VoxelUnderwater.CameraProbeReadback"));
            R.CameraPlanet = CameraPlanet;
            R.Seconds = FPlatformTime::Seconds();
            AddEnqueueCopyPass(GraphBuilder, R.Buffer.Get(), CameraOut, 64u * sizeof(float));
        }
        const double Age = FPlatformTime::Seconds() - CameraSurfaceSeconds;
        const double Moved = (CameraPlanet - CameraSurfaceAt).Size();
        if (bHasCameraSurface && Age >= 0.0 && Age < VoxelUnderwaterDetail::CameraProbeMaxAge && Moved < VoxelUnderwaterDetail::CameraProbeMaxMove)
        {
            const double Margin = VoxelUnderwaterDetail::CameraProbeMargin + Moved + VoxelUnderwaterDetail::CameraProbeRise * Age;
            if (CameraHeight - CameraSurfaceMax - NearReach > FMath::Max(0.0f, Params.Meniscus.W) + Margin)
                return Inputs.ReturnUntouchedSceneColorForPostProcessing(GraphBuilder);
        }
    }

    if (bFFT)
    {
        FVoxelUnderwaterMapParameters* P = MakeMapParams(GraphBuilder.CreateUAV(Map), GraphBuilder.CreateUAV(FoamMap), bFoam);
        TShaderMapRef<FVoxelUnderwaterMapCS> Shader(GetGlobalShaderMap(View.GetFeatureLevel()));
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Underwater surface map %dx%dx%d", MapSize, MapSize, MapLevels),
            Shader, P, FIntVector(FMath::DivideAndRoundUp(MapSize, VoxelUnderwaterDetail::ThreadGroupSize),
                FMath::DivideAndRoundUp(MapSize, VoxelUnderwaterDetail::ThreadGroupSize), MapLevels));
    }
    else
    {
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Map), FLinearColor::Black);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(FoamMap), FLinearColor::Black);
    }

    // -- Stage A8: the caustics, from the map's slopes (the FFT model, the scattered light) ------------
    const bool bCaustics = bFFT && Params.bScattering && Params.bCaustics
        && (Params.CausticStrength > 0.0f || Params.ShaftStrength > 0.0f);
    const FVector4f CausticInfo(FMath::Max(Params.CausticStrength, 0.0f), FMath::Max(Params.ShaftStrength, 0.0f),
        VoxelUnderwaterDetail::CausticBlur, 0.0f);
    FRDGTextureRef CausticMap = nullptr;
    {
        using namespace VoxelUnderwaterDetail;
        const int32 Size = bCaustics ? VoxelUnderwaterMap::Size : 1;
        const int32 Layers = bCaustics ? CausticLevels * CausticSlices : 1;
        CausticMap = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_R16F, FClearValueBinding::Black,
            TexCreate_ShaderResource | TexCreate_UAV, Layers), TEXT("VoxelUnderwater.CausticMap"));
        if (bCaustics)
        {
            FRDGTextureRef Accum = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_R32_UINT, FClearValueBinding::Black,
                TexCreate_ShaderResource | TexCreate_UAV, Layers), TEXT("VoxelUnderwater.CausticAccum"));
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Accum), FUintVector4(0u, 0u, 0u, 0u));
            const FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(View.GetFeatureLevel());
            {
                FVoxelUnderwaterCausticSplatParameters* P = GraphBuilder.AllocParameters<FVoxelUnderwaterCausticSplatParameters>();
                for (int32 Level = 0; Level < VoxelUnderwaterMap::Levels; Level++)
                    P->UnderwaterMapAnchor[Level] = WaveP.UnderwaterMapAnchor[Level];
                P->UnderwaterMap = Map;
                P->UnderwaterCausticAccumOut = GraphBuilder.CreateUAV(Accum);
                const int32 Span = Size + 2 * CausticMargin;
                TShaderMapRef<FVoxelUnderwaterCausticSplatCS> Shader(ShaderMap);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Underwater caustics splat %dx%dx%d", Span, Span, CausticLevels), Shader, P,
                    FIntVector(FMath::DivideAndRoundUp(Span, ThreadGroupSize), FMath::DivideAndRoundUp(Span, ThreadGroupSize), CausticLevels));
            }
            // The resolve: x into a halfway texture, then y into the map. Each pass binds only its own
            // input and output; the other two stay unset (its shader doesn't use them).
            FRDGTextureRef Temp = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_R16F, FClearValueBinding::Black,
                TexCreate_ShaderResource | TexCreate_UAV, Layers), TEXT("VoxelUnderwater.CausticHalfway"));
            const FIntVector Groups(FMath::DivideAndRoundUp(Size, ThreadGroupSize), FMath::DivideAndRoundUp(Size, ThreadGroupSize), Layers);
            {
                FVoxelUnderwaterCausticBlurParameters* P = GraphBuilder.AllocParameters<FVoxelUnderwaterCausticBlurParameters>();
                for (int32 Level = 0; Level < VoxelUnderwaterMap::Levels; Level++)
                    P->UnderwaterMapAnchor[Level] = WaveP.UnderwaterMapAnchor[Level];
                P->UnderwaterCaustic = CausticInfo;
                P->UnderwaterCausticAccum = Accum;
                P->UnderwaterCausticTempOut = GraphBuilder.CreateUAV(Temp);
                TShaderMapRef<FVoxelUnderwaterCausticBlurXCS> Shader(ShaderMap);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Underwater caustics blur x %dx%dx%d", Size, Size, Layers), Shader, P, Groups);
            }
            {
                FVoxelUnderwaterCausticBlurParameters* P = GraphBuilder.AllocParameters<FVoxelUnderwaterCausticBlurParameters>();
                for (int32 Level = 0; Level < VoxelUnderwaterMap::Levels; Level++)
                    P->UnderwaterMapAnchor[Level] = WaveP.UnderwaterMapAnchor[Level];
                P->UnderwaterCaustic = CausticInfo;
                P->UnderwaterCausticTemp = Temp;
                P->UnderwaterCausticMapOut = GraphBuilder.CreateUAV(CausticMap);
                TShaderMapRef<FVoxelUnderwaterCausticBlurYCS> Shader(ShaderMap);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Underwater caustics blur y %dx%dx%d", Size, Size, Layers), Shader, P, Groups);
            }
        }
        else
        {
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(CausticMap), FVector4f(1.0f, 1.0f, 1.0f, 1.0f));
        }
    }

    // -- Stage A4: the light from the atmosphere -------------------------------------------------
    // The sun at the sea over the camera (the atmosphere's surface-sun pass at that one point), and
    // the sky's irradiance (its sky tables, made this frame by the ocean's pass already).
    uint32 AtmoOn = 0u;
    // (Its stand-in, kept where the pass doesn't run: the sun unshadowed, nothing to take behind.)
    static const FVector4f NoAtmoSun[2] = { FVector4f(1.0f, 1.0f, 1.0f, 1.0f), FVector4f(1.0f, 1.0f, 1.0f, 0.0f) };
    FRDGBufferRef AtmoSun = CreateVertexBuffer(GraphBuilder, TEXT("VoxelUnderwater.AtmoSun"),
        FRDGBufferDesc::CreateBufferDesc(sizeof(FVector4f), 2), NoAtmoSun, sizeof(NoAtmoSun));
    FRDGTextureRef AtmoSky = nullptr;
    FRDGTextureRef AtmoSkyIrradiance = nullptr;
    // Stage A7c: the cloud in the light coming down. The atmosphere's own march (its reflection
    // pass) up from the sea over the camera along a cosine-weighted set of directions: the zenith and
    // six at 49 degrees (the centroid of the projected disc's outer six sevenths), so their mean
    // light times pi is the irradiance; float4(the light, its transmittance). Its stand-in: clear.
    constexpr int32 SkyRayCount = 7;
    static const FVector4f ClearSkyRays[SkyRayCount] = {
        FVector4f(0.0f, 0.0f, 0.0f, 1.0f), FVector4f(0.0f, 0.0f, 0.0f, 1.0f), FVector4f(0.0f, 0.0f, 0.0f, 1.0f),
        FVector4f(0.0f, 0.0f, 0.0f, 1.0f), FVector4f(0.0f, 0.0f, 0.0f, 1.0f), FVector4f(0.0f, 0.0f, 0.0f, 1.0f),
        FVector4f(0.0f, 0.0f, 0.0f, 1.0f) };
    FRDGBufferRef AtmoSkyRays = CreateVertexBuffer(GraphBuilder, TEXT("VoxelUnderwater.AtmoSkyRays"),
        FRDGBufferDesc::CreateBufferDesc(sizeof(FVector4f), SkyRayCount), ClearSkyRays, sizeof(ClearSkyRays));
    if (const TSharedPtr<FAtmosphereViewExtension, ESPMode::ThreadSafe> Atmosphere = Params.Atmosphere.Pin();
        Atmosphere.IsValid() && !Params.SunIlluminance.IsNearlyZero() && !Params.SunDirection.IsNearlyZero())
    {
        // The sea sphere's point over the camera, from the camera (world axes, cm; formed in double).
        const FVector FromCenter = ViewOrigin - Waves.PlanetToWorld.GetLocation();
        const double Distance = FromCenter.Size();
        const FVector Over = Distance > 1.0 ? FromCenter / Distance * -CameraHeight : FVector::ZeroVector;
        const float Point[3] = { (float)Over.X, (float)Over.Y, (float)Over.Z };
        FRDGBufferRef Points = CreateVertexBuffer(GraphBuilder, TEXT("VoxelUnderwater.AtmoPoint"),
            FRDGBufferDesc::CreateBufferDesc(sizeof(float), 3), Point, sizeof(Point));
        if (Atmosphere->AddSurfaceSunPass_RenderThread(GraphBuilder, View, GraphBuilder.CreateSRV(Points, PF_R32_FLOAT), 1u,
            FMatrix44f::Identity, GraphBuilder.CreateUAV(AtmoSun, PF_A32B32G32R32F)))
            AtmoOn |= 1u;
        if (Atmosphere->GetSkyTables_RenderThread(GraphBuilder, View, AtmoSky, AtmoSkyIrradiance) && AtmoSkyIrradiance)
            AtmoOn |= 2u;
        if ((AtmoOn & 2u) != 0u && Params.bScattering)
        {
            // From a metre over the sea above the camera (camera-relative, as the sun's point).
            const FVector Up = Distance > 1.0 ? FromCenter / Distance : FVector::UpVector;
            FVector T1, T2;
            Up.FindBestAxisVectors(T1, T2);
            const FVector Origin = Over + Up * 100.0;
            float Rays[SkyRayCount * 6];
            for (int32 I = 0; I < SkyRayCount; I++)
            {
                const double Angle = (double)(I - 1) * (UE_DOUBLE_PI / 3.0);
                const FVector Dir = I == 0 ? Up : (Up * 0.6547 + (T1 * FMath::Cos(Angle) + T2 * FMath::Sin(Angle)) * 0.7559).GetSafeNormal();
                Rays[I * 6 + 0] = (float)Origin.X;
                Rays[I * 6 + 1] = (float)Origin.Y;
                Rays[I * 6 + 2] = (float)Origin.Z;
                Rays[I * 6 + 3] = (float)Dir.X;
                Rays[I * 6 + 4] = (float)Dir.Y;
                Rays[I * 6 + 5] = (float)Dir.Z;
            }
            FRDGBufferRef RayBuffer = CreateVertexBuffer(GraphBuilder, TEXT("VoxelUnderwater.AtmoSkyRayDirs"),
                FRDGBufferDesc::CreateBufferDesc(sizeof(float), SkyRayCount * 6), Rays, sizeof(Rays));
            if (Atmosphere->AddReflectionPass_RenderThread(GraphBuilder, View, GraphBuilder.CreateSRV(RayBuffer, PF_R32_FLOAT),
                (uint32)SkyRayCount, (uint32)SkyRayCount, 16, 32, GraphBuilder.CreateUAV(AtmoSkyRays, PF_A32B32G32R32F)))
                AtmoOn |= 4u;
        }
    }
    if ((AtmoOn & 2u) == 0u)
    {
        AtmoSkyIrradiance = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2D(FIntPoint(1, 1), PF_FloatRGBA, FClearValueBinding::Black,
            TexCreate_ShaderResource | TexCreate_UAV), TEXT("VoxelUnderwater.NoAtmoSky"));
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(AtmoSkyIrradiance), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
    }
    // The water's light down to the camera's depth (per metre to per cm).
    const double Depth = FMath::Max(-CameraHeight, 0.0);
    const FVector3f K = (Params.Absorption + Params.Scattering * 0.25f) * (0.01f * FMath::Max(Params.DepthDarkening, 0.0f));
    const FVector3f DepthLight((float)FMath::Exp(-K.X * Depth), (float)FMath::Exp(-K.Y * Depth), (float)FMath::Exp(-K.Z * Depth));

    // -- The view through the water -----------------------------------------------------------

    {
        FVoxelUnderwaterParameters* P = GraphBuilder.AllocParameters<FVoxelUnderwaterParameters>();
        P->UnderwaterAtmoSun = GraphBuilder.CreateSRV(AtmoSun, PF_A32B32G32R32F);
        P->UnderwaterAtmoSkyIrradiance = AtmoSkyIrradiance;
        P->UnderwaterAtmoSkySampler = TStaticSamplerState<SF_Bilinear, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI();
        P->UnderwaterAtmoSkySize = FVector4f((float)AtmosphereSkyTables::ViewZenith, (float)AtmosphereSkyTables::Azimuth,
            (float)AtmosphereSkyTables::SunZenith, (float)AtmosphereSkyTables::Irradiance);
        P->UnderwaterSunIlluminance = Params.SunIlluminance;
        P->UnderwaterSunDirection = Params.SunDirection;
        P->UnderwaterAtmoOn = AtmoOn;
        P->UnderwaterDepthLight = DepthLight;
        P->UnderwaterScatterOn = (Params.bScattering && !Params.SunIlluminance.IsNearlyZero() && !Params.SunDirection.IsNearlyZero()) ? 1u : 0u;
        P->UnderwaterScatterGain = Params.ScatterGain;
        P->UnderwaterPhaseG = Params.PhaseG;
        P->UnderwaterKd = (Params.Absorption + Params.Scattering * 0.25f) * FMath::Max(Params.DepthDarkening, 0.0f);
        P->UnderwaterAtmoSkyRays = GraphBuilder.CreateSRV(AtmoSkyRays, PF_A32B32G32R32F);
        P->UnderwaterCausticMap = CausticMap;
        P->UnderwaterCausticSampler = TStaticSamplerState<SF_Bilinear, AM_Clamp, AM_Clamp, AM_Clamp>::GetRHI();
        P->UnderwaterCausticOn = bCaustics ? 1u : 0u;
        P->UnderwaterCaustic = CausticInfo;

        P->View = View.ViewUniformBuffer;
        P->UnderwaterSceneColor = SceneColor.Texture;
        // At BeforeDOF the scene colour and the depth share the view's rect.
        P->UnderwaterSceneDepth = SceneTextures->SceneDepthTexture;
        P->UnderwaterViewMin = FUintVector2((uint32)Rect.Min.X, (uint32)Rect.Min.Y);
        P->UnderwaterViewSize = FUintVector2((uint32)OutputSize.X, (uint32)OutputSize.Y);
        P->UnderwaterClipToCamera = FMatrix44f(ClipToCamera);
        P->Waves = WaveP;
        P->UnderwaterMap = Map;
        P->UnderwaterFoamMap = FoamMap;
        P->UnderwaterFoamColor = bFoam ? Params.FoamColor : FVector4f::Zero();

        P->UnderwaterAbsorption = Params.Absorption;
        P->UnderwaterScattering = Params.Scattering;
        P->UnderwaterLight = Params.Light;
        P->UnderwaterMeniscus = Params.Meniscus;

        P->UnderwaterOutput = GraphBuilder.CreateUAV(Output);

        TShaderMapRef<FVoxelUnderwaterCS> Shader(GetGlobalShaderMap(View.GetFeatureLevel()));

        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("Underwater %dx%d", OutputSize.X, OutputSize.Y),
            Shader, P, FComputeShaderUtils::GetGroupCount(OutputSize, VoxelUnderwaterDetail::ThreadGroupSize));
    }

    // BeforeDOF is never last in the chain, so OverrideOutput is not expected; honoured when
    // its format matches (as the atmosphere does).
    if (Inputs.OverrideOutput.IsValid() && Inputs.OverrideOutput.Texture->Desc.Format == OutputDesc.Format)
    {
        FRHICopyTextureInfo Copy;
        Copy.SourcePosition = FIntVector(Rect.Min.X, Rect.Min.Y, 0);
        Copy.DestPosition = FIntVector(Inputs.OverrideOutput.ViewRect.Min.X, Inputs.OverrideOutput.ViewRect.Min.Y, 0);
        Copy.Size = FIntVector(OutputSize.X, OutputSize.Y, 1);

        AddCopyTexturePass(GraphBuilder, Output, Inputs.OverrideOutput.Texture, Copy);

        return Inputs.OverrideOutput;
    }

    return FScreenPassTexture(Output, Rect);
}
