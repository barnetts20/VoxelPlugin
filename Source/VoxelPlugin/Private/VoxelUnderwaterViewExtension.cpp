#include "VoxelUnderwaterViewExtension.h"

#include "DataDrivenShaderPlatformInfo.h"
#include "GlobalShader.h"
#include "PostProcess/PostProcessMaterialInputs.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderUtils.h"
#include "SceneManagement.h"
#include "SceneView.h"
#include "ScreenPass.h"
#include "ShaderCompilerCore.h"
#include "ShaderParameterStruct.h"

namespace VoxelUnderwaterDetail
{
    constexpr int32 ThreadGroupSize = 8;
    /** Slack (cm) on the test of whether a view can see the water, for the float heights. */
    constexpr double ViewSlack = 1.0;
}

// ---------------------------------------------------------------------------
// Shader
//
// Parameter names must match VoxelUnderwaterPass.usf exactly.
// ---------------------------------------------------------------------------

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelUnderwaterParameters, )
    SHADER_PARAMETER_STRUCT_REF(FViewUniformShaderParameters, View)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float4>, UnderwaterSceneColor)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2D<float>, UnderwaterSceneDepth)
    SHADER_PARAMETER(FUintVector2, UnderwaterViewMin)
    SHADER_PARAMETER(FUintVector2, UnderwaterViewSize)
    SHADER_PARAMETER(FMatrix44f, UnderwaterClipToCamera)

    SHADER_PARAMETER(FVector3f, UnderwaterCenter)
    SHADER_PARAMETER(FVector2f, UnderwaterSphere)
    SHADER_PARAMETER(float, UnderwaterWaveAmplitude)
    SHADER_PARAMETER(FVector4f, UnderwaterWave0)
    SHADER_PARAMETER(FVector4f, UnderwaterWave1)
    SHADER_PARAMETER(FVector4f, UnderwaterWave2)

    SHADER_PARAMETER(FVector3f, UnderwaterAbsorption)
    SHADER_PARAMETER(FVector3f, UnderwaterScattering)
    SHADER_PARAMETER(FVector3f, UnderwaterLight)
    SHADER_PARAMETER(FVector4f, UnderwaterMeniscus)

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
        // The stable sphere height and the shell roots were checked in IEEE float order.
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelUnderwaterCS, "/Plugin/VoxelPlugin/Private/VoxelUnderwaterPass.usf", "UnderwaterCS", SF_Compute);

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
            Self->Params = Snapshot;
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
    const double CameraHeight = CameraPlanet.Size() - Waves.Radius;

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
    if (CameraHeight - Waves.MaxWaveHeight() - NearReach > FMath::Max(0.0f, Params.Meniscus.W) + VoxelUnderwaterDetail::ViewSlack)
    {
        return Inputs.ReturnUntouchedSceneColorForPostProcessing(GraphBuilder);
    }

    const FScreenPassTexture SceneColor = FScreenPassTexture::CopyFromSlice(GraphBuilder, SceneColorSlice);
    const FIntRect Rect = SceneColor.ViewRect;
    const FIntPoint OutputSize = Rect.Size();

    if (OutputSize.X <= 0 || OutputSize.Y <= 0)
    {
        return Inputs.ReturnUntouchedSceneColorForPostProcessing(GraphBuilder);
    }

    RDG_EVENT_SCOPE(GraphBuilder, "VoxelUnderwater");

    // Into a new texture of the scene colour's size, written inside the view rect only: the
    // passes after this one read within it.
    FRDGTextureDesc OutputDesc = SceneColor.Texture->Desc;
    OutputDesc.Flags |= TexCreate_ShaderResource | TexCreate_UAV;

    FRDGTextureRef Output = GraphBuilder.CreateTexture(OutputDesc, TEXT("VoxelUnderwater.SceneColor"));

    {
        FVoxelUnderwaterParameters* P = GraphBuilder.AllocParameters<FVoxelUnderwaterParameters>();

        P->View = View.ViewUniformBuffer;
        P->UnderwaterSceneColor = SceneColor.Texture;
        // At BeforeDOF the scene colour and the depth share the view's rect.
        P->UnderwaterSceneDepth = SceneTextures->SceneDepthTexture;
        P->UnderwaterViewMin = FUintVector2((uint32)Rect.Min.X, (uint32)Rect.Min.Y);
        P->UnderwaterViewSize = FUintVector2((uint32)OutputSize.X, (uint32)OutputSize.Y);
        P->UnderwaterClipToCamera = FMatrix44f(ClipToCamera);

        // Formed here in double from this view's own camera.
        P->UnderwaterCenter = FVector3f(Waves.PlanetToWorld.GetLocation() - ViewOrigin);
        P->UnderwaterSphere = FVector2f((float)Waves.Radius, (float)CameraHeight);
        P->UnderwaterWaveAmplitude = (float)Waves.Amplitude;
        P->UnderwaterWave0 = Waves.ShaderWave(0, CameraPlanet);
        P->UnderwaterWave1 = Waves.ShaderWave(1, CameraPlanet);
        P->UnderwaterWave2 = Waves.ShaderWave(2, CameraPlanet);

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
