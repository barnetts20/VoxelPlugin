// VoxelUnderwaterViewExtension.h - The view from under water (Design/OceanMeshing.md section 5)
// as a render-graph pass of the ocean's own, on the pattern of the CloudAtmosphere plugin's
// FAtmosphereViewExtension: the ocean actor fills a flat snapshot every tick, the render thread
// forms each view's parameters from that view's own camera in double, and one compute pass
// (VoxelUnderwaterPass.usf, the optics in VoxelUnderwater.ush) writes the scene colour.
//
// WHERE IT RUNS: ahead of depth of field (EPostProcessingPass::BeforeDOF), as the atmosphere
// does, and after it (a lower priority): the water then attenuates what the atmosphere left,
// so the sky through Snell's window carries the atmosphere and then the water. PITFALL:
// translucency is not in the scene colour yet there; it is merged at depth of field, so a
// translucent object under water is not fogged by its own distance.
//
// ONE PER OCEAN. The actor creates it, enables it while the ocean is drawn, feeds it a snapshot
// every tick and releases it on the render thread. A view whose camera can't see the water
// from its near plane (higher above the surface than the waves and the near plane reach) is
// left untouched, so the pass costs nothing above the sea.

#pragma once

#include "CoreMinimal.h"
#include "SceneViewExtension.h"
#include "RHIResources.h"
#include "VoxelOceanWaves.h"
#include <atomic>

struct FPostProcessMaterialInputs;
struct FScreenPassTexture;
class FTextureResource;
struct FVoxelOceanShoreRHI;
struct FVoxelOceanProbeState;

/** The pass's inputs, flattened for the render thread. The ocean actor sets every member. */
struct FVoxelUnderwaterParams
{
    /** The sea sphere, its frame, the clock and the wave model (VoxelOceanWaves.h). */
    FVoxelOceanWaveFrame Waves;

    /** Per metre, per channel. */
    FVector3f Absorption = FVector3f::ZeroVector;
    FVector3f Scattering = FVector3f::ZeroVector;
    /** The light the water scatters toward the camera (linear, scene units before exposure). */
    FVector3f Light = FVector3f::ZeroVector;
    /** rgb: the waterline's tint (scene units before exposure); w: its width (cm). */
    FVector4f Meniscus = FVector4f::Zero();

    /** The FFT model's textures (FVoxelOceanCascadeTextures' resources; null for any other
     *  model). Read only by SetFrame_GameThread's render command, which keeps references to their
     *  RHI textures: a texture released afterwards (a new chart table) stays valid for the pass
     *  until the next frame replaces it. */
    FTextureResource* Displacement = nullptr;
    FTextureResource* Derivatives = nullptr;
    FTextureResource* Charts = nullptr;

    /** The shore (stage O8): the shore field's texture (null: none), and the ocean pages' depth
     *  probe (render thread data, null: none). A camera in an inland basin (Waves.Shore's distance
     *  below minus ShoreMarginCm) isn't under water. */
    TSharedPtr<const FVoxelOceanShoreRHI, ESPMode::ThreadSafe> Shore;
    TSharedPtr<FVoxelOceanProbeState, ESPMode::ThreadSafe> Probe;
    double ShoreMarginCm = 0.0;

    bool IsUsable() const { return Waves.Radius() > 0.0; }
};

class FVoxelUnderwaterViewExtension : public FWorldSceneViewExtension
{
public:
    FVoxelUnderwaterViewExtension(const FAutoRegister& AutoRegister, UWorld* InWorld);

    /** After the atmosphere's extension (default priority 0) in the same pass. */
    static constexpr int32 Priority = -1;

    /** Hands the render thread this frame's snapshot. Game thread. */
    void SetFrame_GameThread(const FVoxelUnderwaterParams& InParams);

    /** Off skips every view. Game thread. */
    void SetEnabled(bool bInEnabled);

    // -- ISceneViewExtension

    virtual void SetupViewFamily(FSceneViewFamily& InViewFamily) override {}
    virtual void SetupView(FSceneViewFamily& InViewFamily, FSceneView& InView) override {}
    virtual void BeginRenderViewFamily(FSceneViewFamily& InViewFamily) override {}
    virtual int32 GetPriority() const override { return Priority; }

    virtual void SubscribeToPostProcessingPass(EPostProcessingPass Pass, const FSceneView& InView,
        FPostProcessingPassDelegateArray& InOutPassCallbacks, bool bIsPassEnabled) override;

protected:
    virtual bool IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const override;

private:
    FScreenPassTexture Render_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View,
        const FPostProcessMaterialInputs& Inputs);

    std::atomic<bool> bEnabled { false };

    // Render thread only.
    FVoxelUnderwaterParams Params;
    FTextureRHIRef DisplacementRHI;
    FTextureRHIRef DerivativesRHI;
    FTextureRHIRef ChartsRHI;
    bool bHasFrame = false;
};
