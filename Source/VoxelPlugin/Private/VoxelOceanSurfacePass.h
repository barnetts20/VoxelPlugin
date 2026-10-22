// VoxelOceanSurfacePass.h - The ocean's own pass (stage W3; Design/OceanWaves.md section 30): the
// rings and the far sphere (stage W2) drawn and shaded by the ocean itself, not by a material.
//
// WHY: a material (Single Layer Water) shades with the engine's lights and reflection captures,
// which are made for a scene metres to kilometres across: from high up and at a distance its
// reflections and its absorption go wrong. The pass shades the whole sea with one model
// (VoxelOceanShading.ush: Bruneton 2010's glitter, sky reflection and Fresnel over the slopes the
// mesh can't hold, and the water's single scattering down the refracted ray), from the shore to
// orbit, lit by the atmosphere (stage W3b; a stand-in until then: the world's directional light and
// a gradient sky).
//
// WHERE IT RUNS: the renderer's post-opaque hook (IRendererModule::RegisterPostOpaqueRenderDelegate):
// after the opaque scene is lit, before translucency and post-processing. It draws into the scene's
// colour and depth (and its velocity, as the camera's motion only), so translucency sorts against
// the sea, and the atmosphere's aerial perspective and the underwater pass (both ahead of depth of
// field) see its depth as they saw the material's. It reads a copy of the colour and depth behind
// it first (what lies under the water).
//
// WHICH VIEWS: the hook runs for every view of every world, so the extension (one per ocean, a
// world view extension) notes its own world's views as they begin (PreRenderView_RenderThread) and
// draws only into those; not into reflection captures, scene captures (as the mesh, which was
// hidden from them) or orthographic views.
//
// ONE PER OCEAN: the actor creates it, enables it while the rings draw for it, hands it a snapshot
// every tick, and releases it on the render thread (Unregister_RenderThread first).

#pragma once

#include "CoreMinimal.h"
#include "SceneViewExtension.h"
#include <atomic>

class IVoxelOceanRingDraw;
class IRendererModule;
class FPostOpaqueRenderParameters;

/** What the own pass draws and shades with (game thread snapshot). */
struct FVoxelOceanShadingParams
{
    /** The rings (FVoxelOceanSurface::GetRingDraw); null: nothing to draw. */
    TSharedPtr<IVoxelOceanRingDraw, ESPMode::ThreadSafe> Rings;

    /** The water's optics (per metre, per channel) and its scattering phase's g. */
    FVector3f Absorption = FVector3f::ZeroVector;
    FVector3f Scattering = FVector3f::ZeroVector;
    float PhaseG = 0.8f;

    /** The light (the stand-in's until the atmosphere's is connected): toward the sun (world,
     *  unit; zero: no sun), its illuminance (scene light units), the sky's radiance by day at the
     *  zenith and at the horizon (same units). */
    FVector3f SunDirection = FVector3f::ZeroVector;
    FVector3f SunIlluminance = FVector3f::ZeroVector;
    FVector3f SkyZenith = FVector3f::ZeroVector;
    FVector3f SkyHorizon = FVector3f::ZeroVector;

    /** The short waves' slope variance (both axes) the wave bands don't hold; the refraction's
     *  strength (0: none); the foam on and its albedo. */
    float ShortWaveSlopeVariance = 0.004f;
    float Refraction = 1.0f;
    bool bFoam = true;
    FVector3f FoamAlbedo = FVector3f(0.85f, 0.85f, 0.85f);
};

class FVoxelOceanSurfacePass : public FWorldSceneViewExtension
{
public:
    FVoxelOceanSurfacePass(const FAutoRegister& AutoRegister, UWorld* InWorld);
    virtual ~FVoxelOceanSurfacePass() override;

    /** Hands the render thread this frame's snapshot. Game thread. */
    void SetFrame_GameThread(const FVoxelOceanShadingParams& InParams);

    /** Off draws nothing. Game thread. */
    void SetEnabled(bool bInEnabled);

    /** Render thread, before the last reference goes: off the renderer's post-opaque hook. */
    void Unregister_RenderThread();

    // -- ISceneViewExtension

    virtual void SetupViewFamily(FSceneViewFamily& InViewFamily) override {}
    virtual void SetupView(FSceneViewFamily& InViewFamily, FSceneView& InView) override {}
    virtual void BeginRenderViewFamily(FSceneViewFamily& InViewFamily) override {}
    virtual void PreRenderView_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView) override;
    virtual void PostRenderView_RenderThread(FRDGBuilder& GraphBuilder, FSceneView& InView) override;

protected:
    virtual bool IsActiveThisFrame_Internal(const FSceneViewExtensionContext& Context) const override;

private:
    void Render_RenderThread(FPostOpaqueRenderParameters& Parameters);
    /** This world's view the hook's parameters are for (its renderer's view, by address, checked
     *  by its matrices); null for any other. */
    const FSceneView* FindView(const FPostOpaqueRenderParameters& Parameters) const;

    std::atomic<bool> bEnabled { false };
    IRendererModule* RendererModule = nullptr;

    // Render thread only.
    FVoxelOceanShadingParams Params;
    bool bHasFrame = false;
    FDelegateHandle PostOpaqueHandle;
    struct FViewEntry
    {
        const FSceneView* View = nullptr;
        uint64 Frame = 0;
    };
    TArray<FViewEntry> Views;
};
