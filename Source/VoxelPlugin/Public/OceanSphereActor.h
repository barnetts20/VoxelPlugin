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
// inputs (wind, storm) the atmosphere will provide.

#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include "FDensitySampleCompositor.h"
#include "VoxelDensity.h"
#include "VoxelOceanWaves.h"
#include "HAL/CriticalSection.h"
#include "OceanSphereActor.generated.h"

class FVoxelOceanSurface;
class UMaterialInstanceDynamic;
class AAdaptiveVoxelActor;
class FVoxelUnderwaterViewExtension;
struct FVoxelDensityGPUData;
struct FVoxelOceanSurfaceInputs;

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

    // --- Mesh ---

    /** Spacing between vertices at the finest level near the camera (cm). The finest level is
     *  the one at or just under it (ActualSpacing). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh", meta = (ClampMin = "1.0", UIMin = "10.0", UIMax = "1000.0"))
    double FinestSpacing = 50.0;

    /** Quads along a tile edge: 8, 16 or 32 (rounded to one of them). The number of tiles
     *  doesn't depend on it, so the triangle count grows with its square. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh", meta = (ClampMin = "8", ClampMax = "32"))
    int32 TileQuads = 16;

    /** A tile splits while the camera is within this many tile sizes of it. Higher: more
     *  detail and a wider morph band. At least 2.25 (the morph band closes below it). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh", meta = (ClampMin = "2.25", UIMax = "6.0"))
    double LodDistanceFactor = 2.5;

    /** Buffer sets kept ready, so a commit doesn't wait for one to be created. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh", meta = (ClampMin = "0", UIMax = "32"))
    int32 SpareBufferSets = 8;

    /** The finest tile level (from FinestSpacing and the radius). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Mesh")
    int32 MaxLevel = 0;

    /** The spacing (cm) between vertices at MaxLevel, through a face's centre. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Mesh")
    double ActualSpacing = 0.0;

    // --- Depth ---

    /** Leave out the triangles the land stands well above (the terrain hides them anyway). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Depth")
    bool bCullUnderLand = true;

    /** How far (cm) the land must stand above the sea at a triangle's corners, beyond what the
     *  terrain's slope bound allows for across the triangle, before it is left out. Covers
     *  the terrain mesh's error and the waves' run-up. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Depth", meta = (ClampMin = "0.0", UIMax = "5000.0", EditCondition = "bCullUnderLand"))
    double LandCullMargin = 500.0;

    // --- Underwater and optics ---

    /** The view from under water: a pass of the ocean's own after the atmosphere, ahead of
     *  depth of field (STAGE_O3b_NOTES.md). It decides each pixel itself, so partly submerged
     *  frames split exactly, and leaves a view whose camera can't reach the water untouched. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Underwater")
    bool bUnderwaterView = true;

    /** Absorption of the water per metre, per channel (red goes first). Feeds the underwater
     *  pass and the surface material (VoxelOceanAbsorption). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Optics")
    FLinearColor Absorption = FLinearColor(0.35f, 0.07f, 0.03f);

    /** Scattering of the water per metre, per channel. Feeds the underwater pass and the
     *  surface material (VoxelOceanScattering). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Optics")
    FLinearColor Scattering = FLinearColor(0.02f, 0.03f, 0.035f);

    /** Forward scattering anisotropy, for a Single Layer Water surface (VoxelOceanPhaseG). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Optics", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
    float PhaseG = 0.8f;

    /** The light the water scatters toward the camera under water, in the scene's light units
     *  (linear, before exposure, so it holds as the exposure adapts), until the atmosphere
     *  provides the sun's. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Optics")
    FLinearColor UnderwaterLight = FLinearColor(0.4f, 0.8f, 1.0f);

    /** The thin band (cm) where the near plane meets the surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Optics", meta = (ClampMin = "0.0", UIMax = "10.0"))
    double MeniscusWidth = 1.5;

    /** The waterline's tint, in the scene's light units (as UnderwaterLight). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Optics")
    FLinearColor MeniscusColor = FLinearColor(0.02f, 0.06f, 0.08f);

    // --- Environment (constants until the atmosphere provides fields) ---

    /** Wind speed (m/s) everywhere on this ocean. The waves receive it per point; the test
     *  model ignores it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Environment", meta = (ClampMin = "0.0", UIMax = "40.0"))
    double WindSpeed = 8.0;

    /** Wind heading (degrees from local north toward east; north is the planet's +Z). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Environment", meta = (UIMin = "0.0", UIMax = "360.0"))
    double WindHeading = 0.0;

    /** Storm intensity, 0-1. The waves receive it per point; the test model ignores it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Environment", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double StormIntensity = 0.0;

    /** The ocean clock's rate against the game's (server) time. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Environment", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double OceanTimeScale = 1.0;

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

private:
    /** Page components attach to this one. It inherits the actor's position and rotation but
     *  uses absolute scale (1, 1, 1): the surface is built at world scale. */
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    /** The material instance every page draws with; the surface writes the selection camera
     *  into it every frame. */
    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> SurfaceMaterialInstance;

    /** The view from under water: one view extension per ocean, fed every tick. */
    TSharedPtr<FVoxelUnderwaterViewExtension, ESPMode::ThreadSafe> UnderwaterExtension;

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

    /** LandCullMargin as last applied (slider drags apply on release). */
    double AppliedLandCullMargin = 500.0;

    /** The owner planet's terrain (null when standalone). */
    AAdaptiveVoxelActor* FindTerrain() const;
    /** Listens to Terrain's edits (and no longer to the previous terrain's). */
    void BindTerrain(AAdaptiveVoxelActor* Terrain);
    void UnbindTerrain();
    void HandleTerrainEdit(uint32 Sequence, const FVector& Center, double Radius);
    /** The surface's terrain inputs this frame. */
    FVoxelOceanSurfaceInputs GatherInputs(const AAdaptiveVoxelActor* Terrain) const;

    /** The underwater extension, once per world (no renderer: none). */
    void CreateUnderwater();
    /** Enables the underwater extension while the ocean is drawn and hands it this frame. */
    void UpdateUnderwater();
    /** Disables the underwater extension and releases it on the render thread. */
    void ReleaseUnderwater();
    /** The optics properties on the surface material instance (Single Layer Water). */
    void ApplyOptics();
    /** The environment constants from the properties. */
    FVoxelOceanEnvironment GetEnvironment() const;
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
