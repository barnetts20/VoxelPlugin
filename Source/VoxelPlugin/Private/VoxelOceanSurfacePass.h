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
#include "RenderGraphResources.h"
#include <atomic>

class IVoxelOceanRingDraw;
struct FVoxelOceanFoamTexture;
class IRendererModule;
class FPostOpaqueRenderParameters;
class FRDGBuilder;
class FVoxelOceanSurfaceParameters;
class FAtmosphereViewExtension;

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
    /** Stage A2: the planet's atmosphere (CloudAtmosphere's march; null: none): the sun at the water
     *  from it (its surface-sun pass: the clouds' shadow and the air below SunIlluminance, the light
     *  above the air), and the sea's pixels marked for its march to leave alone. */
    TWeakPtr<FAtmosphereViewExtension, ESPMode::ThreadSafe> Atmosphere;
    /** Stage A6: the rain on the water, from the atmosphere's weather: its amount (0: none) and
     *  how far from the camera its rings are drawn (m). */
    float RainAmount = 1.0f;
    float RainRippleDistanceM = 40.0f;

    /** The short waves' slope variance (both axes) the wave bands don't hold; the refraction's
     *  strength (0: none); the foam on and its albedo. */
    float ShortWaveSlopeVariance = 0.001f;
    float Refraction = 1.0f;
    bool bFoam = true;
    FVector3f FoamAlbedo = FVector3f(0.492f, 0.407f, 0.342f);
    /** Stage W3s: the foam's opacity overall, at its thinnest edges, and the thickness (0 to 1 of
     *  the foam's structure) it is solid from. */
    float FoamOpacity = 1.0f;
    float FoamThinOpacity = 0.55f;
    float FoamFalloff = 0.6f;
    /** Stage W3t: the foam drawn as its smooth density (1; stage W3v: GodotOceanWaves' look) rather
     *  than through its texture's cover (0). */
    float FoamSoftness = 1.0f;
    /** Stage W3u: the foam per pixel near the camera (as GodotOceanWaves: the cascades' own foam and
     *  whitecaps at each pixel; 0: the wave clipmap's), how far from the camera it reaches (m), and
     *  the waves' and the clipmap's lookups as cubic B-splines where their texels are magnified. */
    float FoamDetail = 1.0f;
    /** Stage W8j: the pixel's own foam from the clipmap's strain and the finer bands. */
    bool bFoamDetailFromClip = true;
    float FoamDetailDistanceM = 250.0f;
    /** Stage W5f: all foam faded out by this distance (m; 0: no fade), from the further of half of
     *  it and 70% of the nearer of it and FoamDetailDistanceM. */
    float FoamFadeDistanceM = 0.0f;
    bool bCubicFiltering = true;
    /** Stage W3y: the clipmap's lookups along a pixel's longer axis at grazing views (1: none). */
    int32 Anisotropy = 4;
    /** The foam's texture set (stage W3g; null or not ready: the procedural lace) and its tile (m). */
    TSharedPtr<FVoxelOceanFoamTexture, ESPMode::ThreadSafe> FoamTexture;
    float FoamTileM = 12.0f;
    /** Stage W3h: the light through thin crests (0: none) and the whitecaps' relief (1: the lace's). */
    float CrestGlow = 1.0f;
    /** Stage W4d: Godot's water light's share, and its water colour (linear). */
    float GodotWaterLight = 1.0f;
    FVector3f GodotWaterColor = FVector3f(0.0103f, 0.0194f, 0.0273f);
    /** Stage W6d: the footprint (m) by which Godot's light has handed over to ours (0: never). */
    float GodotWaterLightFadeM = 32.0f;
    /** Stage W6j: the footprint (m) by which the sea is opaque (0: never). */
    float FarOpaqueFootprintM = 64.0f;
    float WhitecapRelief = 3.0f;
    /** Stage W3l: the intersection foam's strength (0: none) and reach (m). */
    float IntersectionFoam = 1.0f;
    float IntersectionFoamWidthM = 1.5f;
    /** Stage W3o: the spray's amount (0: none), how far from the camera it lives (m), its ring of
     *  particles, and the wind at the camera (world axes, m/s). */
    float SprayAmount = 1.0f;
    float SprayReachM = 600.0f;
    int32 SprayParticles = 65536;
    FVector3f SprayWind = FVector3f::ZeroVector;
    /** Stage W3x (GodotOceanWaves' spray): the foam the splashes rise from, their size, opacity
     *  and breakup. */
    float SprayFoam = 0.9f;
    float SpraySize = 1.0f;
    float SprayOpacity = 1.0f;
    float SprayBreakup = 1.0f;
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

    /** Stage W3o: the spray, after the sea is drawn for a view (its step and births once a frame,
     *  from the frame's first view; drawn into every view). */
    struct FSprayView
    {
        FVector ViewOrigin = FVector::ZeroVector;
        FVector PlanetCenter = FVector::ZeroVector;
        /** The waves' selection camera (world): their sample frame's origin. */
        FVector SelectionCamera = FVector::ZeroVector;
        double SeaRadius = 0.0;
        FMatrix44f RelToClip = FMatrix44f::Identity;
        FMatrix44f ClipToRel = FMatrix44f::Identity;
        FIntRect Rect;
        float PixelAngle = 1.0e-3f;
        float DeltaSeconds = 0.0f;
    };
    void RenderSpray_RenderThread(FRDGBuilder& GraphBuilder, const FSceneView& View, const FSprayView& SprayView,
        FRDGTexture* SceneColor, FRDGTexture* SceneDepth, FRDGTexture* BehindDepth, const FVoxelOceanSurfaceParameters& Surface);

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
    // Stage W3o: the spray's ring of particles and its head, kept from frame to frame; the anchor
    // its positions are from (the last frame's first view's camera, world).
    TRefCountPtr<FRDGPooledBuffer> SprayParticles;
    TRefCountPtr<FRDGPooledBuffer> SprayCounters;
    uint32 SprayPool = 0;
    uint64 SprayFrame = ~0ull;
    // The last frame the spray was stepped: after a gap (spray off, or its inputs missing) the ring
    // is cleared, so stale puffs don't come back mid-life.
    uint64 SprayStepFrame = 0;
    FVector SprayAnchor = FVector::ZeroVector;
    bool bSprayAnchor = false;
};
