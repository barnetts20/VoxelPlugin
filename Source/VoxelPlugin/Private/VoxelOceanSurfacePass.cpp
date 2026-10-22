#include "VoxelOceanSurfacePass.h"
#include "VoxelOceanRingDraw.h"

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
    TEXT("The ocean's own pass (stage W3f): 1 the foam's density (grey), 2 its structure (the lace drawn for the density), 3 the structure's layers (red the streaks, green the lace, blue the bubbles). 0 off."),
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
    SHADER_PARAMETER(FVector3f, SurfaceScattering)
    SHADER_PARAMETER(FVector3f, SurfaceExtinction)
    SHADER_PARAMETER(float, SurfacePhaseG)
    SHADER_PARAMETER(FVector3f, SurfaceSunDirection)
    SHADER_PARAMETER(FVector3f, SurfaceSunIlluminance)
    SHADER_PARAMETER(FVector3f, SurfaceSkyZenith)
    SHADER_PARAMETER(FVector3f, SurfaceSkyHorizon)
    SHADER_PARAMETER(FVector4f, SurfaceShading)
    SHADER_PARAMETER(FVector3f, SurfaceFoamAlbedo)
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
        (float)FMath::Clamp(CVarVoxelOceanDebugFoam.GetValueOnRenderThread(), 0, 3));
    P->SurfaceFoamAlbedo = Params.FoamAlbedo;

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
}
