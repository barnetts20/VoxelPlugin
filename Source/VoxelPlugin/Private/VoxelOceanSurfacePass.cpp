#include "VoxelOceanSurfacePass.h"
#include "VoxelOceanRingDraw.h"
#include "VoxelOceanFoamTexture.h"

#include "CommonRenderResources.h"
#include "CoreGlobals.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "EngineModule.h"
#include "GlobalShader.h"
#include "PipelineStateCache.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RendererInterface.h"
#include "RenderUtils.h"
#include "RHIStaticStates.h"
#include "SceneView.h"
#include "ShaderParameterStruct.h"
#include "TextureResource.h"
#include "HAL/IConsoleManager.h"

DECLARE_GPU_STAT(VoxelOceanSurface);

static TAutoConsoleVariable<int32> CVarVoxelOceanDebugFoam(
    TEXT("voxel.Ocean.DebugFoam"), 0,
    TEXT("The ocean's own pass (stage W3f): 1 the foam's density (grey), 2 its structure (the lace drawn for the density), 3 the structure's layers (with a foam set, stage W3g: red the lace, green the streaks, blue the whitecaps; with the procedural lace: red the streaks, green the lace, blue the bubbles), 4 the whitecaps' density (stage W3h: the breaking now), 5 where the foam is the pixel's own (stage W3u: green, FoamDetail) or the wave clipmap's (red). 0 off."),
    ECVF_RenderThreadSafe);

namespace VoxelOceanSurfacePassDetail
{
    constexpr int32 ThreadGroupSize = 8;
}

// ---------------------------------------------------------------------------------------------
// Shaders (VoxelOceanSurfacePass.usf). Parameter names must match it exactly.
// ---------------------------------------------------------------------------------------------

// What lies under the water: the view rect's colour and depth before the ocean draws.
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanBehindParameters, )
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D, SurfaceSceneColor)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, SurfaceSceneDepth)
    SHADER_PARAMETER(FUintVector2, SurfaceCopyMin)
    SHADER_PARAMETER(FUintVector2, SurfaceCopySize)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, SurfaceBehindColorOut)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float>, SurfaceBehindDepthOut)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanBehindCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanBehindCS);
    using FParameters = FVoxelOceanBehindParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanBehindCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanBehindCS, "/Plugin/VoxelPlugin/Private/VoxelOceanSurfacePass.usf", "BehindCS", SF_Compute);

// The surface: the rings' vertices, the view, what lies behind, the waves as the material takes
// them, the water and the light.
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanSurfaceParameters, )
    SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
    SHADER_PARAMETER_RDG_BUFFER_SRV(Buffer<float>, SurfacePositions)
    SHADER_PARAMETER_RDG_BUFFER_SRV(Buffer<uint>, SurfaceTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_SRV(Buffer<uint>, SurfaceColors)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, SurfaceIndices)
    SHADER_PARAMETER(uint32, SurfaceTexWords)
    SHADER_PARAMETER(FVector4f, SurfaceOrigin)
    SHADER_PARAMETER(FVector4f, SurfacePlanetRotation)
    SHADER_PARAMETER(FMatrix44f, SurfaceRelToClip)
    SHADER_PARAMETER(FMatrix44f, SurfaceClipToRel)
    SHADER_PARAMETER(FUintVector2, SurfaceViewMin)
    SHADER_PARAMETER(FUintVector2, SurfaceViewSize)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D, SurfaceBehindColor)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, SurfaceBehindDepth)
    SHADER_PARAMETER(FVector4f, SurfaceParams)
    SHADER_PARAMETER(FVector4f, SurfaceWave0)
    SHADER_PARAMETER(FVector4f, SurfaceWave1)
    SHADER_PARAMETER(FVector4f, SurfaceWave2)
    SHADER_PARAMETER(FVector3f, SurfaceCenter)
    SHADER_PARAMETER(FVector4f, SurfaceShape)
    SHADER_PARAMETER(FVector4f, SurfaceFFT)
    SHADER_PARAMETER(FVector4f, SurfaceRefHigh)
    SHADER_PARAMETER(FVector4f, SurfaceRefLow)
    SHADER_PARAMETER(FVector4f, SurfaceRotation)
    SHADER_PARAMETER(FVector4f, SurfaceTiles)
    SHADER_PARAMETER(FVector4f, SurfaceStates)
    SHADER_PARAMETER(FVector4f, SurfaceSlopeVar)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceDisplacement)
    SHADER_PARAMETER_SAMPLER(SamplerState, SurfaceDisplacementSampler)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceDerivatives)
    SHADER_PARAMETER_SAMPLER(SamplerState, SurfaceDerivativesSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, SurfaceCharts)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipDisplacement)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipNormal)
    SHADER_PARAMETER_SAMPLER(SamplerState, SurfaceClipSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, SurfaceClipInfo)
    SHADER_PARAMETER(uint32, SurfaceClipOn)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipWhitecap)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceCascadeFoam)
    SHADER_PARAMETER_SAMPLER(SamplerState, SurfaceCascadeFoamSampler)
    SHADER_PARAMETER(FVector4f, SurfacePixelFoam)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceFineDisplacement)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceFineDerivatives)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceFineFoam)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipFineHistory)
    SHADER_PARAMETER(FVector4f, SurfaceClipFineInfo)
    SHADER_PARAMETER_TEXTURE(Texture2DArray<float4>, SurfaceClipStatic)
    SHADER_PARAMETER(FVector4f, SurfaceFilter)
    SHADER_PARAMETER(FVector4f, SurfaceCrest)
    SHADER_PARAMETER(FVector3f, SurfaceScattering)
    SHADER_PARAMETER(FVector3f, SurfaceExtinction)
    SHADER_PARAMETER(float, SurfacePhaseG)
    SHADER_PARAMETER(FVector3f, SurfaceSunDirection)
    SHADER_PARAMETER(FVector3f, SurfaceSunIlluminance)
    SHADER_PARAMETER(FVector3f, SurfaceSkyZenith)
    SHADER_PARAMETER(FVector3f, SurfaceSkyHorizon)
    SHADER_PARAMETER(FVector4f, SurfaceShading)
    SHADER_PARAMETER(FVector3f, SurfaceFoamAlbedo)
    SHADER_PARAMETER(FVector4f, SurfaceGodotWater)
    SHADER_PARAMETER(FVector4f, SurfaceFoamLook)
    SHADER_PARAMETER_TEXTURE(Texture2D, SurfaceFoamSet)
    SHADER_PARAMETER_SAMPLER(SamplerState, SurfaceFoamSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, SurfaceFoamTable)
    SHADER_PARAMETER(FVector4f, SurfaceFoamInfo)
    RENDER_TARGET_BINDING_SLOTS()
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanSurfaceVS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanSurfaceVS);
    using FParameters = FVoxelOceanSurfaceParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanSurfaceVS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanSurfaceVS, "/Plugin/VoxelPlugin/Private/VoxelOceanSurfacePass.usf", "SurfaceVS", SF_Vertex);

class FVoxelOceanSurfacePS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanSurfacePS);
    using FParameters = FVoxelOceanSurfaceParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanSurfacePS, FGlobalShader);

    /** The scene's velocity written too (as the camera's motion only). */
    class FVelocity : SHADER_PERMUTATION_BOOL("VOXEL_OCEAN_SURFACE_VELOCITY");
    using FPermutationDomain = TShaderPermutationDomain<FVelocity>;

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanSurfacePS, "/Plugin/VoxelPlugin/Private/VoxelOceanSurfacePass.usf", "SurfacePS", SF_Pixel);

// ---------------------------------------------------------------------------------------------
// The spray (stage W3o; VoxelOceanSpray.usf). Parameter names must match it exactly.
// ---------------------------------------------------------------------------------------------

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanSprayUpdateParameters, )
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, SprayParticles)
    SHADER_PARAMETER(FUintVector4, SprayInfo)
    SHADER_PARAMETER(float, SprayDt)
    SHADER_PARAMETER(FVector3f, SprayCenter)
    SHADER_PARAMETER(float, SpraySeaRadius)
    SHADER_PARAMETER(float, SprayReach)
    SHADER_PARAMETER(FVector3f, SprayShift)
    SHADER_PARAMETER(FVector3f, SprayWind)
    SHADER_PARAMETER(FVector3f, SprayAnchorToSample)
    SHADER_PARAMETER(FVector4f, SurfaceRotation)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipDisplacement)
    SHADER_PARAMETER_SAMPLER(SamplerState, SurfaceClipSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, SurfaceClipInfo)
END_SHADER_PARAMETER_STRUCT()

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanSpraySpawnParameters, )
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float4>, SprayParticles)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SprayCounters)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, SprayBorn)
    SHADER_PARAMETER(FUintVector4, SprayInfo)
    SHADER_PARAMETER(float, SprayDt)
    SHADER_PARAMETER(float, SprayReach)
    SHADER_PARAMETER(FVector3f, SprayWind)
    SHADER_PARAMETER(FVector4f, SprayAmount)
    SHADER_PARAMETER(FUintVector2, SurfaceViewMin)
    SHADER_PARAMETER(FUintVector2, SurfaceViewSize)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, SpraySceneDepth)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, SurfaceBehindDepth)
    SHADER_PARAMETER(FMatrix44f, SurfaceRelToClip)
    SHADER_PARAMETER(FMatrix44f, SurfaceClipToRel)
    SHADER_PARAMETER(FVector3f, SprayViewToAnchor)
    SHADER_PARAMETER(FVector3f, SprayViewToSample)
    SHADER_PARAMETER(float, SprayPixelAngle)
    SHADER_PARAMETER(FVector4f, SurfaceParams)
    SHADER_PARAMETER(FVector4f, SurfaceWave0)
    SHADER_PARAMETER(FVector4f, SurfaceWave1)
    SHADER_PARAMETER(FVector4f, SurfaceWave2)
    SHADER_PARAMETER(FVector3f, SurfaceCenter)
    SHADER_PARAMETER(FVector4f, SurfaceShape)
    SHADER_PARAMETER(FVector4f, SurfaceFFT)
    SHADER_PARAMETER(FVector4f, SurfaceRefHigh)
    SHADER_PARAMETER(FVector4f, SurfaceRefLow)
    SHADER_PARAMETER(FVector4f, SurfaceRotation)
    SHADER_PARAMETER(FVector4f, SurfaceTiles)
    SHADER_PARAMETER(FVector4f, SurfaceStates)
    SHADER_PARAMETER(FVector4f, SurfaceSlopeVar)
    SHADER_PARAMETER_TEXTURE(Texture2D, SurfaceCharts)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipDisplacement)
    SHADER_PARAMETER_SAMPLER(SamplerState, SurfaceClipSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, SurfaceClipInfo)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipNormal)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, SurfaceClipWhitecap)
END_SHADER_PARAMETER_STRUCT()

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanSprayDrawParameters, )
    SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, SprayParticlesIn)
    SHADER_PARAMETER(FVector3f, SprayAnchorFromView)
    SHADER_PARAMETER(FVector4f, SprayLook)
    SHADER_PARAMETER_TEXTURE(Texture2D, SprayFoamSet)
    SHADER_PARAMETER_SAMPLER(SamplerState, SprayFoamSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, SprayFoamTable)
    SHADER_PARAMETER(FVector4f, SprayFoamInfo)
    SHADER_PARAMETER(FVector3f, SprayCenter)
    SHADER_PARAMETER(float, SprayReach)
    SHADER_PARAMETER(FMatrix44f, SurfaceRelToClip)
    SHADER_PARAMETER(FMatrix44f, SurfaceClipToRel)
    SHADER_PARAMETER(FUintVector2, SurfaceViewMin)
    SHADER_PARAMETER(FUintVector2, SurfaceViewSize)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, SpraySceneDepth)
    SHADER_PARAMETER(FVector3f, SurfaceSunDirection)
    SHADER_PARAMETER(FVector3f, SurfaceSunIlluminance)
    SHADER_PARAMETER(FVector3f, SurfaceSkyZenith)
    SHADER_PARAMETER(FVector3f, SurfaceSkyHorizon)
    RENDER_TARGET_BINDING_SLOTS()
END_SHADER_PARAMETER_STRUCT()

#define VOXEL_OCEAN_SPRAY_GLOBAL_SHADER(Name, Params) \
    class Name : public FGlobalShader \
    { \
    public: \
        DECLARE_GLOBAL_SHADER(Name); \
        using FParameters = Params; \
        SHADER_USE_PARAMETER_STRUCT(Name, FGlobalShader); \
        static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters) \
        { \
            return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5); \
        } \
    };

VOXEL_OCEAN_SPRAY_GLOBAL_SHADER(FVoxelOceanSprayUpdateCS, FVoxelOceanSprayUpdateParameters)
VOXEL_OCEAN_SPRAY_GLOBAL_SHADER(FVoxelOceanSpraySpawnCS, FVoxelOceanSpraySpawnParameters)
VOXEL_OCEAN_SPRAY_GLOBAL_SHADER(FVoxelOceanSprayVS, FVoxelOceanSprayDrawParameters)
VOXEL_OCEAN_SPRAY_GLOBAL_SHADER(FVoxelOceanSprayPS, FVoxelOceanSprayDrawParameters)
#undef VOXEL_OCEAN_SPRAY_GLOBAL_SHADER

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanSprayUpdateCS, "/Plugin/VoxelPlugin/Private/VoxelOceanSpray.usf", "SprayUpdateCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanSpraySpawnCS, "/Plugin/VoxelPlugin/Private/VoxelOceanSpray.usf", "SpraySpawnCS", SF_Compute);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanSprayVS, "/Plugin/VoxelPlugin/Private/VoxelOceanSpray.usf", "SprayVS", SF_Vertex);
IMPLEMENT_GLOBAL_SHADER(FVoxelOceanSprayPS, "/Plugin/VoxelPlugin/Private/VoxelOceanSpray.usf", "SprayPS", SF_Pixel);

// ---------------------------------------------------------------------------------------------
// Extension
// ---------------------------------------------------------------------------------------------

FVoxelOceanSurfacePass::FVoxelOceanSurfacePass(const FAutoRegister& AutoRegister, UWorld* InWorld)
    : FWorldSceneViewExtension(AutoRegister, InWorld)
{
    // Loaded here, on the game thread; the render thread only uses it.
    RendererModule = &GetRendererModule();
}

FVoxelOceanSurfacePass::~FVoxelOceanSurfacePass()
{
    // (The actor unregisters on the render thread before the last reference goes; this is the
    // fallback for an extension dropped any other way.)
    if (PostOpaqueHandle.IsValid() && RendererModule)
    {
        if (IsInRenderingThread())
        {
            RendererModule->RemovePostOpaqueRenderDelegate(PostOpaqueHandle);
        }
        else
        {
            ENQUEUE_RENDER_COMMAND(VoxelOceanSurfaceUnhook)([Module = RendererModule, Handle = PostOpaqueHandle](FRHICommandListImmediate&)
                {
                    Module->RemovePostOpaqueRenderDelegate(Handle);
                });
        }
        PostOpaqueHandle.Reset();
    }
}

void FVoxelOceanSurfacePass::SetFrame_GameThread(const FVoxelOceanShadingParams& InParams)
{
    check(IsInGameThread());
    TSharedRef<FVoxelOceanSurfacePass, ESPMode::ThreadSafe> Self = StaticCastSharedRef<FVoxelOceanSurfacePass>(AsShared());
    ENQUEUE_RENDER_COMMAND(VoxelOceanSurfaceSetFrame)([Self, Snapshot = InParams](FRHICommandListImmediate&)
        {
            Self->Params = Snapshot;
            Self->bHasFrame = true;
        });
}

void FVoxelOceanSurfacePass::SetEnabled(bool bInEnabled)
{
    check(IsInGameThread());
    bEnabled.store(bInEnabled);
}

void FVoxelOceanSurfacePass::Unregister_RenderThread()
{
    check(IsInRenderingThread());
    if (PostOpaqueHandle.IsValid() && RendererModule)
        RendererModule->RemovePostOpaqueRenderDelegate(PostOpaqueHandle);
    PostOpaqueHandle.Reset();
    Views.Reset();
    Params = FVoxelOceanShadingParams();
    bHasFrame = false;
    SprayParticles.SafeRelease();
    SprayCounters.SafeRelease();
    SprayPool = 0;
    bSprayAnchor = false;
}

bool FVoxelOceanSurfacePass::IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const
{
    return bEnabled.load() && FWorldSceneViewExtension::IsActiveThisFrame_Internal(Context);
}

void FVoxelOceanSurfacePass::PreRenderView_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView)
{
    // On the hook once (the delegate holds a weak reference: a pass gone stops being called).
    if (!PostOpaqueHandle.IsValid() && RendererModule)
    {
        const TSharedRef<FVoxelOceanSurfacePass, ESPMode::ThreadSafe> Self = StaticCastSharedRef<FVoxelOceanSurfacePass>(AsShared());
        PostOpaqueHandle = RendererModule->RegisterPostOpaqueRenderDelegate(FPostOpaqueRenderDelegate::CreateSP(Self, &FVoxelOceanSurfacePass::Render_RenderThread));
    }
    // This world's view, for this frame (older entries are dropped).
    const uint64 Frame = GFrameCounterRenderThread;
    Views.RemoveAll([Frame, &InView](const FViewEntry& E) { return E.Frame != Frame || E.View == &InView; });
    Views.Add({ &InView, Frame });
}

void FVoxelOceanSurfacePass::PostRenderView_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView)
{
    Views.RemoveAll([&InView](const FViewEntry& E) { return E.View == &InView; });
}

const FSceneView* FVoxelOceanSurfacePass::FindView(const FPostOpaqueRenderParameters& Parameters) const
{
    const uint64 Frame = GFrameCounterRenderThread;
    // The renderer's view is the view the extensions saw (its address the hook's Uid) ...
    for (const FViewEntry& E : Views)
    {
        if (E.Frame == Frame && (const void*)E.View == Parameters.Uid) return E.View;
    }
    // ... or, should the addresses differ, the one with its camera.
    for (const FViewEntry& E : Views)
    {
        if (E.Frame == Frame && E.View->ViewMatrices.GetViewMatrix().Equals(Parameters.ViewMatrix, 1.0e-6)) return E.View;
    }
    return nullptr;
}

void FVoxelOceanSurfacePass::Render_RenderThread(FPostOpaqueRenderParameters& Parameters)
{
    using namespace VoxelOceanSurfacePassDetail;
    if (!bEnabled.load() || !bHasFrame || !Params.Rings.IsValid() || !Parameters.GraphBuilder || !Parameters.ColorTexture || !Parameters.DepthTexture)
        return;
    const FSceneView* ViewPtr = FindView(Parameters);
    if (!ViewPtr) return;
    const FSceneView& View = *ViewPtr;
    if (View.bIsReflectionCapture || View.bIsPlanarReflection || View.bIsSceneCapture || !View.IsPerspectiveProjection()
        || View.GetFeatureLevel() < ERHIFeatureLevel::SM5 || !View.ViewUniformBuffer.IsValid())
        return;
    FRDGTextureRef SceneColor = Parameters.ColorTexture;
    FRDGTextureRef SceneDepth = Parameters.DepthTexture;
    if (SceneColor->Desc.NumSamples > 1 || SceneDepth->Desc.NumSamples > 1) return;
    const FIntRect Rect = Parameters.ViewportRect;
    const FIntPoint Size = Rect.Size();
    if (Size.X <= 0 || Size.Y <= 0) return;

    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(View.GetFeatureLevel());
    const bool bVelocity = Parameters.VelocityTexture != nullptr && Parameters.VelocityTexture->Desc.NumSamples <= 1;
    FVoxelOceanSurfacePS::FPermutationDomain Permutation;
    Permutation.Set<FVoxelOceanSurfacePS::FVelocity>(bVelocity);
    TShaderMapRef<FVoxelOceanBehindCS> BehindShader(ShaderMap);
    TShaderMapRef<FVoxelOceanSurfaceVS> VertexShader(ShaderMap);
    TShaderMapRef<FVoxelOceanSurfacePS> PixelShader(ShaderMap, Permutation);
    if (!BehindShader.IsValid() || !VertexShader.IsValid() || !PixelShader.IsValid()) return;

    FRDGBuilder& GraphBuilder = *Parameters.GraphBuilder;
    FVoxelOceanRingDrawFrame Rings;
    if (!Params.Rings->PrepareDraw_RenderThread(GraphBuilder, Rings) || Rings.Triangles == 0) return;

    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanSurface, "VoxelOceanSurface");

    // -- What lies under the water -------------------------------------------------------------
    FRDGTextureRef BehindColor = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2D(Size, PF_FloatRGBA, FClearValueBinding::Black,
        TexCreate_ShaderResource | TexCreate_UAV), TEXT("Voxel.OceanBehindColor"));
    FRDGTextureRef BehindDepth = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2D(Size, PF_R32_FLOAT, FClearValueBinding::Black,
        TexCreate_ShaderResource | TexCreate_UAV), TEXT("Voxel.OceanBehindDepth"));
    {
        FVoxelOceanBehindParameters* P = GraphBuilder.AllocParameters<FVoxelOceanBehindParameters>();
        P->SurfaceSceneColor = SceneColor;
        P->SurfaceSceneDepth = SceneDepth;
        P->SurfaceCopyMin = FUintVector2((uint32)Rect.Min.X, (uint32)Rect.Min.Y);
        P->SurfaceCopySize = FUintVector2((uint32)Size.X, (uint32)Size.Y);
        P->SurfaceBehindColorOut = GraphBuilder.CreateUAV(BehindColor);
        P->SurfaceBehindDepthOut = GraphBuilder.CreateUAV(BehindDepth);
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanBehind %dx%d", Size.X, Size.Y), BehindShader, P,
            FComputeShaderUtils::GetGroupCount(Size, ThreadGroupSize));
    }

    // -- The view, in double: its camera-relative world axes to its clip space and back ----------
    // From its own translated view-projection (the jittered one the scene's depth was drawn with):
    // a point from the camera, plus the camera's translated position, is translated world.
    const FVector ViewOrigin = View.ViewMatrices.GetViewOrigin();
    const FVector CameraTranslated = ViewOrigin + View.ViewMatrices.GetPreViewTranslation();
    const FMatrix RelToClip = FTranslationMatrix(CameraTranslated) * View.ViewMatrices.GetTranslatedViewProjectionMatrix();
    const FMatrix ClipToRel = RelToClip.Inverse();
    const FVector Origin = Rings.PlanetToWorld.TransformPosition(Rings.Origin) - ViewOrigin;
    const FQuat PlanetRotation = Rings.PlanetToWorld.GetRotation();

    // -- The surface -----------------------------------------------------------------------------
    FVoxelOceanSurfaceParameters* P = GraphBuilder.AllocParameters<FVoxelOceanSurfaceParameters>();
    P->View = View.ViewUniformBuffer;
    P->SurfacePositions = Rings.Positions;
    P->SurfaceTexCoords = Rings.TexCoords;
    P->SurfaceColors = Rings.Colors;
    P->SurfaceIndices = Rings.Indices;
    P->SurfaceTexWords = Rings.TexWords;
    // (w: the view's camera above the sea sphere, cm: from near or under the surface the sunk far
    // sphere is left out.)
    const double ViewHeight = (ViewOrigin - Rings.PlanetToWorld.GetLocation()).Size() - Rings.SeaRadius;
    P->SurfaceOrigin = FVector4f((float)Origin.X, (float)Origin.Y, (float)Origin.Z, (float)FMath::Clamp(ViewHeight, -1.0e9, 1.0e9));
    P->SurfacePlanetRotation = FVector4f((float)PlanetRotation.X, (float)PlanetRotation.Y, (float)PlanetRotation.Z, (float)PlanetRotation.W);
    P->SurfaceRelToClip = FMatrix44f(RelToClip);
    P->SurfaceClipToRel = FMatrix44f(ClipToRel);
    P->SurfaceViewMin = FUintVector2((uint32)Rect.Min.X, (uint32)Rect.Min.Y);
    P->SurfaceViewSize = FUintVector2((uint32)Size.X, (uint32)Size.Y);
    P->SurfaceBehindColor = BehindColor;
    P->SurfaceBehindDepth = BehindDepth;

    // The waves as the material takes them; the FFT model off without its textures, the clipmap
    // without its frame (the radial normal then).
    const FVoxelOceanDisplaceFrame& W = Rings.Waves;
    const bool bFFT = W.FFT.X > 0.5f && Rings.Displacement && Rings.Derivatives && Rings.Charts;
    P->SurfaceParams = W.Params;
    P->SurfaceWave0 = W.Wave[0];
    P->SurfaceWave1 = W.Wave[1];
    P->SurfaceWave2 = W.Wave[2];
    P->SurfaceCenter = W.Center;
    P->SurfaceShape = W.Shape;
    P->SurfaceFFT = bFFT ? W.FFT : FVector4f(0.0f, W.FFT.Y, 0.0f, 0.0f);
    P->SurfaceRefHigh = W.RefHigh;
    P->SurfaceRefLow = W.RefLow;
    P->SurfaceRotation = W.Rotation;
    P->SurfaceTiles = W.Tiles;
    P->SurfaceStates = W.States;
    P->SurfaceSlopeVar = W.SlopeVar;
    P->SurfaceDisplacement = bFFT ? Rings.Displacement : GBlackArrayTexture->TextureRHI.GetReference();
    P->SurfaceDerivatives = bFFT ? Rings.Derivatives : GBlackArrayTexture->TextureRHI.GetReference();
    P->SurfaceCharts = Rings.Charts ? Rings.Charts : GBlackTexture->TextureRHI.GetReference();
    P->SurfaceDisplacementSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    P->SurfaceDerivativesSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    P->SurfaceClipDisplacement = Rings.bClip ? Rings.ClipDisplacement : GBlackArrayTexture->TextureRHI.GetReference();
    P->SurfaceClipNormal = Rings.bClip ? Rings.ClipNormal : GBlackArrayTexture->TextureRHI.GetReference();
    P->SurfaceClipInfo = Rings.bClip ? Rings.ClipInfo : GBlackTexture->TextureRHI.GetReference();
    // (Wrapping: the clipmap's levels are toroidal.)
    P->SurfaceClipSampler = TStaticSamplerState<SF_Bilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    P->SurfaceClipOn = Rings.bClip ? 1u : 0u;
    // z, w: the intersection foam's strength and reach (cm; stage W3l).
    P->SurfaceCrest = FVector4f(FMath::Max(Params.CrestGlow, 0.0f), FMath::Max(Params.WhitecapRelief, 1.0f),
        FMath::Max(Params.IntersectionFoam, 0.0f), FMath::Max(Params.IntersectionFoamWidthM, 0.1f) * 100.0f);
    P->SurfaceClipWhitecap = Rings.bClip && Rings.ClipWhitecap ? Rings.ClipWhitecap : GBlackArrayTexture->TextureRHI.GetReference();
    // Stage W3u: the foam per pixel near the camera; x its strength, y its reach (cm), z foam on, w the
    // cubic filtering. (Stage W3w: the cascades' foam texture holds the fine foam, slice 7.)
    const bool bCascadeFoam = bFFT && Params.bFoam;
    P->SurfaceCascadeFoam = Rings.Foam ? Rings.Foam : GBlackArrayTexture->TextureRHI.GetReference();
    // Stage W3z: the shortest band's own 512^2 cascade (the pass reads band 3 from it; without it,
    // none of band 3 near the camera: the cascades make it with the others).
    const bool bFine = bFFT && Rings.FineDisplacement && Rings.FineDerivatives && Rings.FineFoam;
    P->SurfaceFineDisplacement = bFine ? Rings.FineDisplacement : GBlackArrayTexture->TextureRHI.GetReference();
    P->SurfaceFineDerivatives = bFine ? Rings.FineDerivatives : GBlackArrayTexture->TextureRHI.GetReference();
    P->SurfaceFineFoam = bFine ? Rings.FineFoam : GBlackArrayTexture->TextureRHI.GetReference();
    // Stage W4c: the foam's fine history (VoxelOceanClip.usf, ClipFineFoamCS; none: 0 levels).
    const bool bFineHistory = Rings.bClip && Rings.ClipFineHistory && Rings.ClipFineInfo.X > 0.5f;
    P->SurfaceClipFineHistory = bFineHistory ? Rings.ClipFineHistory : GBlackArrayTexture->TextureRHI.GetReference();
    // Stage W5c: the clipmap's static layer (the pixels' shore; SurfaceClipFineInfo.z 1). Without it a
    // stand-in, not read: deep water at the pixels, the open sea alone.
    const bool bStatic = Rings.bClip && Rings.ClipStatic;
    P->SurfaceClipFineInfo = FVector4f(bFineHistory ? Rings.ClipFineInfo.X : 0.0f, bFineHistory ? Rings.ClipFineInfo.Y : 1.0f, bStatic ? 1.0f : 0.0f, 0.0f);
    P->SurfaceClipStatic = bStatic ? Rings.ClipStatic : GBlackArrayTexture->TextureRHI.GetReference();
    P->SurfaceCascadeFoamSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    P->SurfacePixelFoam = FVector4f(FMath::Clamp(Params.FoamDetail, 0.0f, 1.0f), FMath::Max(Params.FoamDetailDistanceM, 1.0f) * 100.0f,
        bCascadeFoam ? 1.0f : 0.0f, Params.bCubicFiltering ? 1.0f : 0.0f);
    // Stage W3y: the anisotropic filter's most lookups along a pixel's longer axis.
    P->SurfaceFilter = FVector4f((float)FMath::Clamp(Params.Anisotropy, 1, 16), 0.0f, 0.0f, 0.0f);

    // The water, per cm.
    const FVector3f Scattering = Params.Scattering * 0.01f;
    const FVector3f Extinction = (Params.Absorption + Params.Scattering) * 0.01f;
    P->SurfaceScattering = Scattering;
    P->SurfaceExtinction = Extinction;
    P->SurfacePhaseG = FMath::Clamp(Params.PhaseG, -0.95f, 0.95f);
    // The light.
    P->SurfaceSunDirection = Params.SunDirection;
    P->SurfaceSunIlluminance = Params.SunDirection.IsNearlyZero() ? FVector3f::ZeroVector : Params.SunIlluminance;
    P->SurfaceSkyZenith = Params.SkyZenith;
    P->SurfaceSkyHorizon = Params.SkyHorizon;
    P->SurfaceShading = FVector4f(FMath::Max(Params.ShortWaveSlopeVariance, 0.0f), FMath::Max(Params.Refraction, 0.0f), Params.bFoam ? 1.0f : 0.0f,
        (float)FMath::Clamp(CVarVoxelOceanDebugFoam.GetValueOnRenderThread(), 0, 5));
    P->SurfaceFoamAlbedo = Params.FoamAlbedo;
    // Stage W4d: Godot's water light (its colour, its share).
    P->SurfaceGodotWater = FVector4f(Params.GodotWaterColor.X, Params.GodotWaterColor.Y, Params.GodotWaterColor.Z, FMath::Clamp(Params.GodotWaterLight, 0.0f, 1.0f));
    // Stage W3s: the foam's opacity, its thinnest edge's and the thickness it is solid from.
    P->SurfaceFoamLook = FVector4f(FMath::Clamp(Params.FoamOpacity, 0.0f, 1.0f), FMath::Clamp(Params.FoamThinOpacity, 0.0f, 1.0f),
        FMath::Max(Params.FoamFalloff, 0.01f), 0.0f);
    // The foam's set (stage W3g), once its textures exist.
    const FVoxelOceanFoamTexture* FoamSet = Params.FoamTexture.Get();
    const bool bFoamSet = FoamSet && FoamSet->bReady && FoamSet->Set.IsValid() && FoamSet->Table.IsValid();
    P->SurfaceFoamSet = bFoamSet ? FoamSet->Set.GetReference() : GBlackTexture->TextureRHI.GetReference();
    P->SurfaceFoamTable = bFoamSet ? FoamSet->Table.GetReference() : GBlackTexture->TextureRHI.GetReference();
    P->SurfaceFoamSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    // (w: the foam's softness, stage W3t.)
    P->SurfaceFoamInfo = FVector4f(1.0f / FMath::Max(Params.FoamTileM, 1.0f), bFoamSet ? (float)FoamSet->Mips : 1.0f, bFoamSet ? 1.0f : 0.0f,
        FMath::Clamp(Params.FoamSoftness, 0.0f, 1.0f));

    P->RenderTargets[0] = FRenderTargetBinding(SceneColor, ERenderTargetLoadAction::ELoad);
    if (bVelocity)
        P->RenderTargets[1] = FRenderTargetBinding(Parameters.VelocityTexture, ERenderTargetLoadAction::ELoad);
    P->RenderTargets.DepthStencil = FDepthStencilBinding(SceneDepth, ERenderTargetLoadAction::ELoad, FExclusiveDepthStencil::DepthWrite_StencilNop);

    // As the mesh drew them: the default winding culled, the view's mirror reversing it.
    const bool bReverseCulling = View.bReverseCulling;
    const uint32 Triangles = Rings.Triangles;
    GraphBuilder.AddPass(RDG_EVENT_NAME("VoxelOceanSurface %u triangles %dx%d", Triangles, Size.X, Size.Y), P, ERDGPassFlags::Raster,
        [P, VertexShader, PixelShader, Rect, Triangles, bReverseCulling](FRHICommandList& RHICmdList)
        {
            RHICmdList.SetViewport((float)Rect.Min.X, (float)Rect.Min.Y, 0.0f, (float)Rect.Max.X, (float)Rect.Max.Y, 1.0f);
            FGraphicsPipelineStateInitializer PSO;
            RHICmdList.ApplyCachedRenderTargets(PSO);
            PSO.BlendState = TStaticBlendState<>::GetRHI();
            PSO.RasterizerState = bReverseCulling ? TStaticRasterizerState<FM_Solid, CM_CCW>::GetRHI() : TStaticRasterizerState<FM_Solid, CM_CW>::GetRHI();
            PSO.DepthStencilState = TStaticDepthStencilState<true, CF_DepthNearOrEqual>::GetRHI();
            PSO.BoundShaderState.VertexDeclarationRHI = GEmptyVertexDeclaration.VertexDeclarationRHI;
            PSO.BoundShaderState.VertexShaderRHI = VertexShader.GetVertexShader();
            PSO.BoundShaderState.PixelShaderRHI = PixelShader.GetPixelShader();
            PSO.PrimitiveType = PT_TriangleList;
            SetGraphicsPipelineState(RHICmdList, PSO, 0);
            SetShaderParameters(RHICmdList, VertexShader, VertexShader.GetVertexShader(), *P);
            SetShaderParameters(RHICmdList, PixelShader, PixelShader.GetPixelShader(), *P);
            RHICmdList.DrawPrimitive(0, Triangles, 1);
        });

    // -- The spray (stage W3o) -------------------------------------------------------------------
    if (Params.SprayAmount > 0.0f && Params.SprayParticles > 0 && bFFT && Rings.bClip && Rings.ClipWhitecap)
    {
        FSprayView SV;
        SV.ViewOrigin = ViewOrigin;
        SV.PlanetCenter = Rings.PlanetToWorld.GetLocation();
        SV.SelectionCamera = W.CameraWorld;
        SV.SeaRadius = Rings.SeaRadius;
        SV.RelToClip = FMatrix44f(RelToClip);
        SV.ClipToRel = FMatrix44f(ClipToRel);
        SV.Rect = Rect;
        const double M11 = View.ViewMatrices.GetProjectionMatrix().M[1][1];
        SV.PixelAngle = M11 > 0.0 ? (float)(2.0 / (M11 * (double)Size.Y)) : 1.0e-3f;
        SV.DeltaSeconds = View.Family ? View.Family->Time.GetDeltaWorldTimeSeconds() : 1.0f / 60.0f;
        RenderSpray_RenderThread(GraphBuilder, View, SV, SceneColor, SceneDepth, BehindDepth, *P);
    }
    else if (Params.SprayAmount <= 0.0f && SprayParticles.IsValid())
    {
        // Off: its ring let go.
        SprayParticles.SafeRelease();
        SprayCounters.SafeRelease();
        SprayPool = 0;
        bSprayAnchor = false;
    }
}

void FVoxelOceanSurfacePass::RenderSpray_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View, const FSprayView& SV,
    FRDGTextureRef SceneColor, FRDGTextureRef SceneDepth, FRDGTextureRef BehindDepth, const FVoxelOceanSurfaceParameters& Surface)
{
    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(View.GetFeatureLevel());
    TShaderMapRef<FVoxelOceanSprayUpdateCS> UpdateShader(ShaderMap);
    TShaderMapRef<FVoxelOceanSpraySpawnCS> SpawnShader(ShaderMap);
    TShaderMapRef<FVoxelOceanSprayVS> VertexShader(ShaderMap);
    TShaderMapRef<FVoxelOceanSprayPS> PixelShader(ShaderMap);
    if (!UpdateShader.IsValid() || !SpawnShader.IsValid() || !VertexShader.IsValid() || !PixelShader.IsValid()) return;

    const uint32 Pool = (uint32)FMath::Clamp(Params.SprayParticles, 1024, 262144);
    const float Reach = FMath::Max(Params.SprayReachM, 10.0f) * 100.0f;

    // The ring: kept from frame to frame, made anew (cleared) when its size changes.
    FRDGBufferRef Particles;
    FRDGBufferRef Counters;
    if (!SprayParticles.IsValid() || !SprayCounters.IsValid() || SprayPool != Pool)
    {
        Particles = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(FVector4f), 3 * Pool), TEXT("Voxel.OceanSprayParticles"));
        Counters = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 4), TEXT("Voxel.OceanSprayCounters"));
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Particles), 0u);
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Counters), 0u);
        SprayParticles = GraphBuilder.ConvertToExternalBuffer(Particles);
        SprayCounters = GraphBuilder.ConvertToExternalBuffer(Counters);
        SprayPool = Pool;
        bSprayAnchor = false;
    }
    else
    {
        Particles = GraphBuilder.RegisterExternalBuffer(SprayParticles);
        Counters = GraphBuilder.RegisterExternalBuffer(SprayCounters);
    }

    const FVector3f Wind = Params.SprayWind * 100.0f;

    // Once a frame, from its first view: the step and the births.
    const uint64 Frame = GFrameCounterRenderThread;
    if (SprayFrame != Frame)
    {
        SprayFrame = Frame;
        const FVector NewAnchor = SV.ViewOrigin;
        FVector Shift = bSprayAnchor ? SprayAnchor - NewAnchor : FVector::ZeroVector;
        // A jump (everything out of reach) or a gap since the last step (the spray off, or its
        // inputs missing): start afresh.
        const bool bGap = SprayStepFrame + 2u < Frame;
        if (Shift.Size() > (double)Reach || bGap)
        {
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Particles), 0u);
            Shift = FVector::ZeroVector;
        }
        SprayStepFrame = Frame;
        SprayAnchor = NewAnchor;
        bSprayAnchor = true;
        const float Dt = FMath::Clamp(SV.DeltaSeconds, 0.0f, 0.1f);
        const FVector Center = SV.PlanetCenter - SprayAnchor;
        {
            FVoxelOceanSprayUpdateParameters* U = GraphBuilder.AllocParameters<FVoxelOceanSprayUpdateParameters>();
            U->SprayParticles = GraphBuilder.CreateUAV(Particles);
            U->SprayInfo = FUintVector4(Pool, 0u, (uint32)Frame, 0u);
            U->SprayDt = Dt;
            U->SprayCenter = FVector3f(Center);
            U->SpraySeaRadius = (float)SV.SeaRadius;
            U->SprayReach = Reach;
            U->SprayShift = FVector3f(Shift);
            U->SprayWind = Wind;
            // Stage W3x: the splashes ride the waves (the clipmap's displacement at their rest points).
            U->SprayAnchorToSample = FVector3f(SprayAnchor - SV.SelectionCamera);
            U->SurfaceRotation = Surface.SurfaceRotation;
            U->SurfaceClipDisplacement = Surface.SurfaceClipDisplacement;
            U->SurfaceClipSampler = Surface.SurfaceClipSampler;
            U->SurfaceClipInfo = Surface.SurfaceClipInfo;
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanSprayUpdate %u", Pool), UpdateShader, U,
                FComputeShaderUtils::GetGroupCount((int32)Pool, 64));
        }
        if (Dt > 0.0f)
        {
            FRDGBufferRef Born = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 1), TEXT("Voxel.OceanSprayBorn"));
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Born), 0u);
            FVoxelOceanSpraySpawnParameters* S = GraphBuilder.AllocParameters<FVoxelOceanSpraySpawnParameters>();
            S->SprayParticles = GraphBuilder.CreateUAV(Particles);
            S->SprayCounters = GraphBuilder.CreateUAV(Counters);
            S->SprayBorn = GraphBuilder.CreateUAV(Born);
            // At most an eighth of the ring a frame.
            S->SprayInfo = FUintVector4(Pool, 0u, (uint32)Frame, FMath::Max(Pool / 8u, 1u));
            S->SprayDt = Dt;
            S->SprayReach = Reach;
            S->SprayWind = Wind;
            // (Stage W3x: x the amount, z the foam the splashes rise from, w their size.)
            S->SprayAmount = FVector4f(Params.SprayAmount, 0.0f, FMath::Clamp(Params.SprayFoam, 0.0f, 1.3f), FMath::Max(Params.SpraySize, 0.05f));
            S->SurfaceViewMin = Surface.SurfaceViewMin;
            S->SurfaceViewSize = Surface.SurfaceViewSize;
            S->SpraySceneDepth = SceneDepth;
            S->SurfaceBehindDepth = BehindDepth;
            S->SurfaceRelToClip = SV.RelToClip;
            S->SurfaceClipToRel = SV.ClipToRel;
            S->SprayViewToAnchor = FVector3f(SV.ViewOrigin - SprayAnchor);
            // The waves' sample frame: world axes from the selection camera, the planet's centre at
            // SurfaceCenter in it.
            S->SprayViewToSample = FVector3f(SV.ViewOrigin - SV.SelectionCamera);
            S->SprayPixelAngle = SV.PixelAngle;
            S->SurfaceParams = Surface.SurfaceParams;
            S->SurfaceWave0 = Surface.SurfaceWave0;
            S->SurfaceWave1 = Surface.SurfaceWave1;
            S->SurfaceWave2 = Surface.SurfaceWave2;
            S->SurfaceCenter = Surface.SurfaceCenter;
            S->SurfaceShape = Surface.SurfaceShape;
            S->SurfaceFFT = Surface.SurfaceFFT;
            S->SurfaceRefHigh = Surface.SurfaceRefHigh;
            S->SurfaceRefLow = Surface.SurfaceRefLow;
            S->SurfaceRotation = Surface.SurfaceRotation;
            S->SurfaceTiles = Surface.SurfaceTiles;
            S->SurfaceStates = Surface.SurfaceStates;
            S->SurfaceSlopeVar = Surface.SurfaceSlopeVar;
            S->SurfaceCharts = Surface.SurfaceCharts;
            S->SurfaceClipDisplacement = Surface.SurfaceClipDisplacement;
            S->SurfaceClipSampler = Surface.SurfaceClipSampler;
            S->SurfaceClipInfo = Surface.SurfaceClipInfo;
            S->SurfaceClipNormal = Surface.SurfaceClipNormal;
            S->SurfaceClipWhitecap = Surface.SurfaceClipWhitecap;
            const FIntPoint Size = SV.Rect.Size();
            const FIntPoint Cells((Size.X + 7) / 8, (Size.Y + 7) / 8);
            FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanSpraySpawn %dx%d", Cells.X, Cells.Y), SpawnShader, S,
                FComputeShaderUtils::GetGroupCount(Cells, 8));
        }
    }

    // Every view: the puffs over the scene's colour.
    FVoxelOceanSprayDrawParameters* D = GraphBuilder.AllocParameters<FVoxelOceanSprayDrawParameters>();
    D->View = View.ViewUniformBuffer;
    D->SprayParticlesIn = GraphBuilder.CreateSRV(Particles);
    D->SprayAnchorFromView = FVector3f(SprayAnchor - SV.ViewOrigin);
    D->SprayLook = FVector4f(FMath::Max(Params.SprayOpacity, 0.0f), FMath::Max(Params.SprayBreakup, 0.0f), 0.0f, 0.0f);
    // The foam set breaks the puffs up (stage W3q; the surface's, or a value noise without it).
    D->SprayFoamSet = Surface.SurfaceFoamSet;
    D->SprayFoamSampler = Surface.SurfaceFoamSampler;
    D->SprayFoamTable = Surface.SurfaceFoamTable;
    D->SprayFoamInfo = Surface.SurfaceFoamInfo;
    D->SprayCenter = FVector3f(SV.PlanetCenter - SprayAnchor);
    D->SprayReach = Reach;
    D->SurfaceRelToClip = SV.RelToClip;
    D->SurfaceClipToRel = SV.ClipToRel;
    D->SurfaceViewMin = Surface.SurfaceViewMin;
    D->SurfaceViewSize = Surface.SurfaceViewSize;
    D->SpraySceneDepth = SceneDepth;
    D->SurfaceSunDirection = Surface.SurfaceSunDirection;
    D->SurfaceSunIlluminance = Surface.SurfaceSunIlluminance;
    D->SurfaceSkyZenith = Surface.SurfaceSkyZenith;
    D->SurfaceSkyHorizon = Surface.SurfaceSkyHorizon;
    D->RenderTargets[0] = FRenderTargetBinding(SceneColor, ERenderTargetLoadAction::ELoad);
    const FIntRect Rect = SV.Rect;
    GraphBuilder.AddPass(RDG_EVENT_NAME("VoxelOceanSpray %u", Pool), D, ERDGPassFlags::Raster,
        [D, VertexShader, PixelShader, Rect, Pool](FRHICommandList& RHICmdList)
        {
            RHICmdList.SetViewport((float)Rect.Min.X, (float)Rect.Min.Y, 0.0f, (float)Rect.Max.X, (float)Rect.Max.Y, 1.0f);
            FGraphicsPipelineStateInitializer PSO;
            RHICmdList.ApplyCachedRenderTargets(PSO);
            // Premultiplied over the scene; its alpha kept.
            PSO.BlendState = TStaticBlendState<CW_RGBA, BO_Add, BF_One, BF_InverseSourceAlpha, BO_Add, BF_Zero, BF_One>::GetRHI();
            PSO.RasterizerState = TStaticRasterizerState<FM_Solid, CM_None>::GetRHI();
            PSO.DepthStencilState = TStaticDepthStencilState<false, CF_Always>::GetRHI();
            PSO.BoundShaderState.VertexDeclarationRHI = GEmptyVertexDeclaration.VertexDeclarationRHI;
            PSO.BoundShaderState.VertexShaderRHI = VertexShader.GetVertexShader();
            PSO.BoundShaderState.PixelShaderRHI = PixelShader.GetPixelShader();
            PSO.PrimitiveType = PT_TriangleList;
            SetGraphicsPipelineState(RHICmdList, PSO, 0);
            SetShaderParameters(RHICmdList, VertexShader, VertexShader.GetVertexShader(), *D);
            SetShaderParameters(RHICmdList, PixelShader, PixelShader.GetPixelShader(), *D);
            RHICmdList.DrawPrimitive(0, 2 * Pool, 1);
        });
}
