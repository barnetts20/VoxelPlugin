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
#include "RHIGPUReadback.h"
#include "VoxelOceanWaves.h"
#include <atomic>

struct FPostProcessMaterialInputs;
struct FScreenPassTexture;
class FTextureResource;
struct FVoxelOceanShoreRHI;
struct FVoxelOceanProbeState;
class FAtmosphereViewExtension;

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

    /** Stage A4: the planet's atmosphere (null: none). With it, Light, the foam's colour and the
     *  meniscus's are scaled by the daylight at the sea over the camera (the sun through the clouds'
     *  shadow and the air, and the sky's irradiance: its surface-sun pass and sky tables) against
     *  the light above the air, SunIlluminance toward SunDirection. */
    TWeakPtr<FAtmosphereViewExtension, ESPMode::ThreadSafe> Atmosphere;
    FVector3f SunDirection = FVector3f::ZeroVector;
    FVector3f SunIlluminance = FVector3f::ZeroVector;
    /** Stage A4: the water's light dimmed with the camera's depth, exp(-K depth) with K the
     *  absorption plus a quarter of the scattering, times this (0: none). */
    float DepthDarkening = 1.0f;
    /** Stage A7: the water's light as scattered sunlight and skylight (VoxelUnderwater.ush) rather
     *  than the fog of Light, while there's a sun; its gain, and the water's phase g. */
    bool bScattering = true;
    float ScatterGain = 2.0f;
    float PhaseG = 0.8f;
    /** Stage A8: the caustics on the bed and the shafts in the water (the FFT model, with the
     *  scattering), their strengths (1: as the waves focus the beam). */
    bool bCaustics = true;
    float CausticStrength = 1.0f;
    float ShaftStrength = 1.0f;

    /** The FFT model's textures (FVoxelOceanCascadeTextures' resources; null for any other
     *  model). Read only by SetFrame_GameThread's render command, which keeps references to their
     *  RHI textures: a texture released afterwards (a new chart table) stays valid for the pass
     *  until the next frame replaces it. */
    FTextureResource* Displacement = nullptr;
    FTextureResource* Derivatives = nullptr;
    FTextureResource* Charts = nullptr;
    /** The open sea's foam (stage O9c: the cascades' foam array; null: none) and the foam's colour
     *  seen from below (rgb, linear scene units before exposure; w 0: no foam). */
    FTextureResource* Foam = nullptr;
    FVector4f FoamColor = FVector4f::Zero();

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
    FTextureRHIRef FoamRHI;
    bool bHasFrame = false;

    /** Stage W8b: the camera's probe (the surface's height round the camera's foot, an 8 x 8
     *  grid of the map pass's points) read back a few frames later; the pass is skipped while the
     *  camera stands clear above the last reading, with margins for its age and the camera's
     *  travel since. Render thread only. */
    struct FCameraReadback
    {
        TUniquePtr<FRHIGPUBufferReadback> Buffer;
        FVector CameraPlanet = FVector::ZeroVector;
        double Seconds = 0.0;
    };
    TArray<FCameraReadback> CameraReadbacks;
    bool bHasCameraSurface = false;
    double CameraSurfaceMax = 0.0;          // cm above the sphere, the grid's highest
    FVector CameraSurfaceAt = FVector::ZeroVector;
    double CameraSurfaceSeconds = 0.0;
    /** T5d3: the bed's depth under the camera (cm, the depth probe's at its foot), from the same
     *  readings: the caustics' grids' centre (VoxelUnderwaterMap::CausticCentre). */
    bool bHasCameraBed = false;
    double CameraBedDepth = 0.0;
};
