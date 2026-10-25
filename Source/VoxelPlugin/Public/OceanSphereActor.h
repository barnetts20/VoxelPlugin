// OceanSphereActor.h - The ocean of a planet (or standalone): a sphere at sea level drawn from GPU
// pages of a cube-sphere quadtree (FVoxelOceanSurface, Design/OceanMeshing.md). Scale (max
// component, cm) is the ocean radius; a planet sets it from its sea level and drives the actor.
//
// Stage O1: the plain sphere, its level-of-detail selection, the GPU pages and the morph.
// Stage O2: the terrain depth under every vertex (the planet density with the terrain's
// edits), the land cull and a test wave. Stage O3: the view from under water (a render-graph
// pass per ocean from a view extension, FVoxelUnderwaterViewExtension), the water's optics,
// and height queries. Stage O4: the wave model interface (VoxelOceanWaves.h, the GPU hook in
// VoxelOceanWaves.ush), the surface query on any thread, the ocean clock, and the environment
// inputs (wind, storm) the atmosphere will provide. Stage O6b: the FFT waves (the cascades'
// textures, made and kept here; FVoxelOceanWavesFFT, FVoxelOceanCascades). Stage W3: the ocean's
// own pass draws and shades the rings (FVoxelOceanSurfacePass), lit by a stand-in sun (the world's
// directional light) and sky until the atmosphere's light is connected.

#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include "FDensitySampleCompositor.h"
#include "VoxelDensity.h"
#include "VoxelOceanWaves.h"
#include "VoxelOceanSettings.h"
#include "HAL/CriticalSection.h"
#include "OceanSphereActor.generated.h"

class FVoxelOceanSurface;
class UMaterialInstanceDynamic;
class UTexture2D;
class UTexture2DArray;
class AAdaptiveVoxelActor;
class FVoxelUnderwaterViewExtension;
class FVoxelOceanSurfacePass;
struct FVoxelOceanFoamTexture;
class UDirectionalLightComponent;
class APlanetAtmosphereActor;
class FAtmosphereWeatherFeed;
class FVoxelOceanProceduralWeather;
struct FVoxelDensityGPUData;
struct FVoxelOceanSurfaceInputs;
class FVoxelOceanShoreField;
class FVoxelOceanShoreBuilder;
class FVoxelOceanCoastField;
class FVoxelOceanCoastBuilder;
struct FVoxelOceanShoreRHI;
struct FVoxelOceanLandCacheRHI;
class FTextureResource;

/** The ocean surface: every frame the tiles the camera selects are committed to GPU pages
 *  (one frame per change, the previous tiles kept until the new ones can be drawn), and the
 *  camera they were selected for goes to the material for the morph between levels. */
UCLASS()
class VOXELPLUGIN_API AOceanSphereActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

public:
    AOceanSphereActor();

    /** True when spawned and driven by APlanetActor. Planet-driven properties become
     *  read-only: edit them on the planet instead. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean")
    bool bIsPlanetOwned = false;

    /** Ocean surface material (setup: STAGE_O1_NOTES.md). Empty: the plugin's MT_Ocean. Change it
     *  at run time with SetOceanMaterial. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, BlueprintSetter = SetOceanMaterial, Category = "Ocean|Material",
        meta = (EditCondition = "!bIsPlanetOwned"))
    UMaterialInterface* OceanMaterial = nullptr;

    // --- Shape ---

    /** The sea sphere's radius (cm): the actor's scale at initialization. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Shape")
    double OceanRadius = 107500000.0;

    /** The terrain's planet radius (cm), for the standalone ocean's density (terrain depth,
     *  from stage O2). A planet sets it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "1.0", EditCondition = "!bIsPlanetOwned"))
    double TerrainPlanetRadius = 100000000.0;

    /** The terrain's noise amplitude ratio, for the standalone ocean's density. A planet sets it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "0.01", ClampMax = "1.0", EditCondition = "!bIsPlanetOwned"))
    double NoiseAmplitudeRatio = 0.15;

    /** The terrain's surface settings, for the standalone ocean's density. A planet sets them. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (EditCondition = "!bIsPlanetOwned"))
    FVoxelSurfaceSettings SurfaceSettings;

    // --- Settings ---

    /** The ocean's settings: mesh, land cull, underwater view, optics and environment. A
     *  planet sets them from its own OceanSettings (read-only here then: edit them on the
     *  planet). Change them at run time with SetOceanSettings: only what changed is applied. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ocean", meta = (EditCondition = "!bIsPlanetOwned"))
    FVoxelOceanSettings OceanSettings;

    /** The finest tile level (from FinestSpacing and the radius). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Mesh")
    int32 MaxLevel = 0;

    /** The spacing (cm) between vertices at MaxLevel, through a face's centre. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Mesh")
    double ActualSpacing = 0.0;

    // --- Queries (any thread for the C++ ones; Blueprint on the game thread) ---

    /** The water surface at world points: height above it, the surface point on each point's
     *  radial, its normal, the water's velocity, and (with EVoxelOceanQuery::Depth) the terrain
     *  depth there. Any thread, no GPU: it reads the snapshot of the last tick (the same waves
     *  as drawn, unfaded). Time < 0: the snapshot's clock. False (and the plain sphere) before
     *  the ocean is initialized. */
    bool QuerySurface(TConstArrayView<FVector> WorldPoints, TArrayView<FVoxelOceanSample> Out,
        EVoxelOceanQuery Flags = EVoxelOceanQuery::None, double Time = -1.0) const;

    /** QuerySurface for one point. */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Ocean|Query")
    FVoxelOceanSample QuerySurfaceAt(const FVector& WorldLocation, bool bWithDepth = false) const;

    /** The height (cm) of a world point above the drawn surface; negative under water. */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Ocean|Query")
    double GetHeightAboveSurface(const FVector& WorldLocation) const;

    /** True when the world point is under the drawn surface. */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Ocean|Query")
    bool IsSubmerged(const FVector& WorldLocation) const;

    /** The ocean clock (s): the game state's server time (the same on every machine) times
     *  OceanTimeScale; the world's time without a game state (the editor). Game thread. */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Ocean|Query")
    double GetOceanTime() const;

    /** The weather at a world point now (the weather source: procedural, or the Environment
     *  constants), exactly; the waves follow it through 5-minute keys and whole heading steps.
     *  The wind and the swell (world axes, tangent, m/s; where they blow toward) and the storm
     *  (0-1). False before the ocean is initialized. */
    UFUNCTION(BlueprintCallable, BlueprintPure, Category = "Ocean|Weather")
    bool GetWeatherAt(const FVector& WorldLocation, FVector& Wind, double& Storm, FVector& Swell) const;

    // --- Accessors ---

    USceneComponent* GetMeshAttachmentRoot() const { return MeshAttachmentRoot; }
    TSharedPtr<FDensitySampleCompositor> GetCompositor() const { return Compositor; }
    /** The camera last seen by Tick, in the planet frame (actor space without scale). */
    FVector GetCameraPosition() const { return CameraPosition; }

    // --- Lifecycle ---

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Destroyed() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetActorHiddenInGame(bool bNewHidden) override;

    // --- Editor Property & Transform Locking ---

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual bool CanEditChange(const FProperty* InProperty) const override;
    virtual void PostEditMove(bool bFinished) override;
    virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
#endif

    /** Called by APlanetActor: uses its compositor (shared with the terrain), sets the scale
     *  (ocean radius) first, and re-parents the mesh root to InAttachParent if given. */
    void InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
        USceneComponent* InAttachParent = nullptr,
        FVector InScale = FVector::ZeroVector);

    /** Rebuilds the surface from the current property values (standalone: also the density). */
    void Initialize();

    /** Changes the ocean material on every page without a rebuild (null: the plugin's MT_Ocean). */
    UFUNCTION(BlueprintCallable, Category = "Ocean|Material")
    void SetOceanMaterial(UMaterialInterface* InMaterial);

    /** Stage A1: the sun the ocean shades with, set by the planet to its atmosphere's light
     *  (null: the world's brightest directional light, as before). */
    void SetSunLightOverride(UDirectionalLightComponent* InLight);

    /** Stage A2: the planet's atmosphere (null: none). Its light is the ocean's sun (as
     *  SetSunLightOverride), and its march gives the sun at the water (the clouds' shadow and the
     *  air; voxel.Ocean.AtmosphereSun), leaving the sea's pixels to the ocean. */
    void SetAtmosphere(APlanetAtmosphereActor* InAtmosphere);

    /** New settings, applying only what changed: a mesh change rebuilds the surface (unless
     *  bInteractive: a slider still moving, applied on release), the LOD factor and the optics
     *  apply in place, the rest is read every tick. */
    UFUNCTION(BlueprintCallable, Category = "Ocean")
    void SetOceanSettings(const FVoxelOceanSettings& InSettings, bool bInteractive = false);

    /** Stage W6g: the realistic look (FVoxelOceanSettings::ApplyRealisticPreset) for a standalone
     *  ocean (a planet's: use the planet's). */
    UFUNCTION(CallInEditor, BlueprintCallable, Category = "Ocean")
    void OceanPresetRealistic();

    /** Stage W6g: GodotOceanWaves' look, the defaults (ApplyGodotPreset). */
    UFUNCTION(CallInEditor, BlueprintCallable, Category = "Ocean")
    void OceanPresetGodot();

private:
    /** Page components attach to this one. It inherits the actor's position and rotation but
     *  uses absolute scale (1, 1, 1): the surface is built at world scale. */
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    /** The material instance every page draws with; the surface writes the selection camera
     *  into it every frame. */
    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> SurfaceMaterialInstance;

    /** The FFT cascades' textures (FVoxelOceanCascadeTextures): the outputs the surface material
     *  and the underwater pass sample, and the chart table for this radius. Made once (no
     *  renderer: none). */
    UPROPERTY(Transient)
    TObjectPtr<UTexture2DArray> WaveDisplacement;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DArray> WaveDerivatives;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2D> WaveCharts;

    /** The open sea's foam per slot (stage O9b). */
    UPROPERTY(Transient)
    TObjectPtr<UTexture2DArray> WaveFoam;

    /** Stage W3z: the shortest band's own cascade (displacement, derivatives, fine foam). */
    UPROPERTY(Transient)
    TObjectPtr<UTexture2DArray> WaveFineDisplacement;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DArray> WaveFineDerivatives;

    UPROPERTY(Transient)
    TObjectPtr<UTexture2DArray> WaveFineFoam;

    /** The radius WaveCharts was made for. */
    double WaveChartsRadius = 0.0;

    /** The shore field (stage O8; FVoxelOceanShoreField): made from the planet's land bake at
     *  load (ShoreBuilder), its texture for the material and the passes, and that texture's
     *  render-thread handle. None without a land bake (open water everywhere). */
    TSharedPtr<FVoxelOceanShoreBuilder> ShoreBuilder;
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> ShoreField;
    UPROPERTY(Transient)
    TObjectPtr<UTexture2DArray> ShoreTexture;
    TSharedPtr<const FVoxelOceanShoreRHI, ESPMode::ThreadSafe> ShoreRHI;
    /** The terrain's land surface cache as the ocean's depths read it (stage O9m; null: none, or
     *  not written for the camera) and the newest edit its texels carry. */
    TSharedPtr<const FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe> LandCacheRHI;
    uint32 LandCacheEditSequence = 0;
    /** Frames the cache hasn't been written for the camera: kept through short gaps (dropping it
     *  writes the clipmap's static layer again), let go after a while. */
    int32 LandCacheIdleFrames = 0;
    /** The texture's resource ShoreRHI was made from (compared only). */
    const FTextureResource* ShoreResource = nullptr;
    /** Starts the shore field's build when the land bake, the radius or the reach changed, and
     *  takes it when done. */
    void UpdateShoreField();
    /** voxel.Debug.OceanShore: the shore field around the camera. */
    void DrawShoreDebug() const;

public:
    /** Stage W8a: voxel.Ocean.PerfSweep: the GPU's frame time with each of the ocean's costly
     *  features off in turn (the camera held still), logged as a table; the settings restored. */
    void StartPerfSweep(int32 FramesPerStep, int32 Rounds = 3);

private:
    struct FPerfSweep
    {
        bool bOn = false;
        FVoxelOceanSettings Original;
        /** Stage W8e: the settings measured from: the original with the weather's clock held
         *  (no weather key, no heading turn mid-sweep: a turn doubles the waves' work for 30 s). */
        FVoxelOceanSettings Held;
        int32 Step = 0;
        /** Stage W8d: interleaved: each step's rounds of the baseline then the variant. */
        int32 Round = 0;
        int32 Phase = 0;
        int32 Frame = 0;
        int32 Frames = 120;
        int32 Rounds = 3;
        /** Stage W8g: the reference's time the sweep scales every segment to (its first). */
        float RefNominal = 0.0f;
        TArray<float> RawSamples;
        TArray<float> RefSamples;
        /** Stage W8g: the console variables uncapped while it runs, as they were. */
        TMap<FString, FString> SavedVars;
        /** Stage W8f: what a segment measures: the ocean's own GPU time (VoxelOceanGPUTime, its
         *  pass groups' timestamps) and, for context, the GPU's whole frame. */
        static constexpr int32 MaxGroups = 12;
        struct FMeasure
        {
            float Ocean = 0.0f;         // stage W8g: scaled by the reference (the GPU's speed)
            float Raw = 0.0f;           // the ocean's as timed
            float Ref = 0.0f;           // the reference's
            float Frame = 0.0f;
            float Groups[MaxGroups] = {};
        };
        TArray<float> FrameSamples;
        TArray<float> GroupSamples[MaxGroups];
        FMeasure SegmentBase;
        TArray<FMeasure> Savings;       // this step's, a round each
        TArray<FMeasure> Bases;         // every round's baseline
        struct FResult
        {
            FMeasure Saving;            // the median of the rounds' savings, each part on its own
            float Least = 0.0f;         // the ocean's least and most of them
            float Most = 0.0f;
        };
        TArray<FResult> Results;
    };
    FPerfSweep PerfSweep;
    void TickPerfSweep();
    void StopPerfSweep();
    void RestorePerfSweepVars();

    /** The coastal travel-time field (stage O9h; FVoxelOceanCoastField): baked after the shore
     *  field from the terrain (CoastBuilder, the depths sampled on the GPU through the surface),
     *  its texels in ShoreTexture's slices 6 and up once made (the texture made again). None
     *  without a shore field, a renderer, or with bCoastField off. */
    TSharedPtr<FVoxelOceanCoastBuilder> CoastBuilder;
    /** Stage W7a: the centre (planet frame, unit) of the window last asked of the builder. */
    FVector3d CoastWindowCentre = FVector3d::ZAxisVector;
    TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe> CoastField;
    /** Starts the coastal field's bake when the shore field or its settings changed, and puts it
     *  into the shore texture when done. After the surface's inputs (it samples with them). */
    void UpdateCoastField();

    /** The view from under water: one view extension per ocean, fed every tick. */
    TSharedPtr<FVoxelUnderwaterViewExtension, ESPMode::ThreadSafe> UnderwaterExtension;

    /** The ocean's own pass (stage W3): one per ocean, fed every tick while the rings draw for it. */
    TSharedPtr<FVoxelOceanSurfacePass, ESPMode::ThreadSafe> SurfacePass;
    bool bSurfacePassOn = false;
    /** The foam's texture set (stage W3g; OceanSettings.FoamSet), read when its name changes. */
    TSharedPtr<FVoxelOceanFoamTexture, ESPMode::ThreadSafe> FoamTexture;
    FString FoamTextureName;
    bool bFoamTextureTried = false;
    bool bFoamTextureForFoam = false;
    /** The stand-in sun: the world's directional light (looked for again every second). */
    TWeakObjectPtr<UDirectionalLightComponent> SunLight;
    /** Stage A1: the planet's atmosphere's light, preferred to the search while set and valid. */
    TWeakObjectPtr<UDirectionalLightComponent> SunLightOverride;
    /** Stage A2: the planet's atmosphere (its march's view extension asked for every tick). */
    TWeakObjectPtr<APlanetAtmosphereActor> Atmosphere;
    double SunLightSearchTime = -1.0e9;
    bool bWarnedNoSun = false;

    /** The last tick's waves for the queries, swapped whole under QueryLock. */
    TSharedPtr<const FVoxelOceanWaveFrame, ESPMode::ThreadSafe> QueryFrame;
    mutable FRWLock QueryLock;

    /** The density (shared with the terrain when planet-owned); the terrain depth reads it from
     *  stage O2. */
    TSharedPtr<FDensitySampleCompositor> Compositor;

    /** The rendered surface. */
    TSharedPtr<FVoxelOceanSurface> Surface;

    FVector CameraPosition = FVector::ZeroVector;
    bool bHasCamera = false;       // CameraPosition holds a rendered view's
    bool bInitialized = false;

    /** The planet density on the GPU, built from Compositor (the terrain's is the same). */
    TSharedPtr<const FVoxelDensityGPUData> GPUDensity;

    /** The planet's terrain the depth follows: its lattice, slope bound and edits. */
    TWeakObjectPtr<AAdaptiveVoxelActor> BoundTerrain;
    FDelegateHandle TerrainEditHandle;

    /** The settings the surface was last built or updated with (SetOceanSettings compares
     *  against them; slider drags apply on release). */
    FVoxelOceanSettings AppliedSettings;

    /** The owner planet's terrain (null when standalone). */
    AAdaptiveVoxelActor* FindTerrain() const;
    /** Listens to Terrain's edits (and no longer to the previous terrain's). */
    void BindTerrain(AAdaptiveVoxelActor* Terrain);
    void UnbindTerrain();
    void HandleTerrainEdit(uint32 Sequence, const FVector& Center, double Radius);
    /** The surface's terrain inputs this frame. */
    FVoxelOceanSurfaceInputs GatherInputs(const AAdaptiveVoxelActor* Terrain) const;
    /** Every frame before GatherInputs: the terrain's land surface cache (stage O9m). */
    void UpdateLandCache(const AAdaptiveVoxelActor* Terrain);

    /** The underwater extension, once per world (no renderer: none). */
    void CreateUnderwater();
    /** Enables the underwater extension while the ocean is drawn and hands it this frame. */
    void UpdateUnderwater();
    /** Disables the underwater extension and releases it on the render thread. */
    void ReleaseUnderwater();
    /** The ocean's own pass (stage W3), once per world (no renderer: none). */
    void CreateSurfacePass();
    /** Enables the own pass while the rings draw for it and hands it this frame. */
    void UpdateSurfacePass();
    /** Disables the own pass and releases it on the render thread (off the renderer's hook first). */
    void ReleaseSurfacePass();
    /** The stand-in sun: the world's atmosphere sun light (the first), else its brightest
     *  directional light; null: none. */
    UDirectionalLightComponent* FindSunLight();
    /** The optics properties on the surface material instance (Single Layer Water). */
    void ApplyOptics();
    /** The environment constants from the properties. */
    FVoxelOceanEnvironment GetEnvironment() const;
    /** The weather source for the settings (a new one only when they change: its keys start
     *  over). */
    void UpdateWeatherSource();
    /** voxel.Debug.OceanWeather: the wind (white), swell (cyan) and storms (red) around the
     *  camera. */
    void DrawWeatherDebug() const;
    TSharedPtr<const IVoxelOceanEnvironmentSource, ESPMode::ThreadSafe> WeatherSource;
    /** The procedural weather, when that's the source or the atmosphere's fallback. */
    TSharedPtr<const FVoxelOceanProceduralWeather, ESPMode::ThreadSafe> ProceduralWeather;
    /** Stage A5: the planet's atmosphere's weather feed (held: its read-backs run while held), the
     *  one WeatherSource was made over, and whether a map had landed then. */
    TSharedPtr<FAtmosphereWeatherFeed, ESPMode::ThreadSafe> AtmosphereWeatherFeed;
    TSharedPtr<FAtmosphereWeatherFeed, ESPMode::ThreadSafe> WeatherSourceFeed;
    bool bWeatherSourceHadMap = false;
    /** The settings and radius WeatherSource was made for. */
    FVoxelOceanSettings WeatherSourceSettings;
    double WeatherSourceRadius = -1.0;
    /** Publishes this tick's waves to the queries (and the underwater pass). */
    void PublishWaveFrame();
    /** voxel.Debug.OceanQuery: query points around the camera drawn on the surface. */
    void DrawQueryDebug() const;
    FVector LastInitScale = FVector::ZeroVector;

    /** Cached scale set by the planet actor, used by the transform guard. */
    FVector PlanetDrivenScale = FVector::OneVector;

    /** Bound to RootComponent->TransformUpdated when planet-owned: snaps transforms back to the
     *  planet-driven values. */
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);

    /** Shared by Initialize and InitializeFromPlanet: the material instance, the finest level
     *  and the surface. */
    void InitializeInternal(TSharedPtr<FDensitySampleCompositor> InCompositor);

    /** Destroys RealtimeMesh components left on the mesh root (an old level may have saved the
     *  previous ocean's chunks with it). */
    void CleanSceneRoot();

    /** The plugin's MT_Ocean, or the engine default material when the plugin content is missing. */
    static UMaterialInterface* DefaultOceanMaterial();
};
