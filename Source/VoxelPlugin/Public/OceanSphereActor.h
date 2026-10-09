// OceanSphereActor.h - The ocean of a planet (or standalone): a sphere at sea level drawn from GPU
// pages of a cube-sphere quadtree (FVoxelOceanSurface, Design/OceanMeshing.md). Scale (max
// component, cm) is the ocean radius; a planet sets it from its sea level and drives the actor.
//
// Stage O1: the plain sphere, its level-of-detail selection, the GPU pages and the morph. The
// terrain depth under the sea, the underwater view and the wave and query interfaces come in
// the next stages.

#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include "FDensitySampleCompositor.h"
#include "VoxelDensity.h"
#include "OceanSphereActor.generated.h"

class FVoxelOceanSurface;
class UMaterialInstanceDynamic;

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
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

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

    /** The density (shared with the terrain when planet-owned); the terrain depth reads it from
     *  stage O2. */
    TSharedPtr<FDensitySampleCompositor> Compositor;

    /** The rendered surface. */
    TSharedPtr<FVoxelOceanSurface> Surface;

    FVector CameraPosition = FVector::ZeroVector;
    bool bInitialized = false;
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
