// PlanetAtmosphereActor.h — Manages a post-process volume with 3 blendable
// material instances and a directional light to render a volumetric atmosphere
// and cloud layer around a planet.
//
// When owned by APlanetActor, the actor's scale is set externally to
// max(OceanRadius, PlanetRadius) — the visible surface floor. The actor
// reads its own scale as PlanetRadius for material parameters.
//
// Actor Location  → Planet Center / Atmosphere Center
// Actor Scale max → Planet Radius
// Actor Rotation  → Light Direction + directional light rotation
//
// TWO CLOUD MODELS SHARE ONE MARCH. PlanetType selects which material fills
// slot 1: a terrestrial cloud band, or a gas giant deck driven by the flow
// simulation. Slots 0 and 2 are shared.
//
// The parameters are one Environment set plus one Common set and one model set
// PER TYPE. PlanetType picks which pair goes to the material. See
// AtmosphereParams.h for what lives where.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/PostProcessVolume.h"
#include "Engine/DirectionalLight.h"
#include "Components/DirectionalLightComponent.h"
#include "Engine/VolumeTexture.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "AtmosphereParams.h"
#include "AtmosphereTransmittance.h"
#include "PlanetAtmosphereActor.generated.h"

class UTextureRenderTarget2D;
class UTextureRenderTarget2DArray;

/** Renders a volumetric atmosphere and cloud layer via post-process materials.
 *
 *  Spawns two child actors (APostProcessVolume + ADirectionalLight) and creates
 *  3 dynamic material instances (preprocess, atmosphere, postprocess) that are
 *  assigned as blendables on the post-process volume.
 *
 *  All scattering, cloud, and ray marching parameters are exposed as UPROPERTYs
 *  and pushed to the materials every tick via UpdateMaterialParameters.
 *  The directional light's rotation and color are synced from the actor's
 *  rotation and LightColor property via UpdateLightFromRotation.
 *
 *  When planet-owned: location and scale are locked (driven by the planet),
 *  rotation remains editable (controls light direction).
 *
 *  ONE FUNCTION PICKS THE MATERIAL AND ONE PICKS THE PARAMETERS, BOTH FROM
 *  PlanetType. Setting a parameter a material does not declare does nothing and
 *  logs nothing, so a march material and a parameter sweep that disagree render
 *  something plausible with none of the model-specific inputs bound — which
 *  reads as a simulation or texture bug rather than a wiring one. */
UCLASS()
class VOXELPLUGIN_API APlanetAtmosphereActor : public AActor
{
    GENERATED_BODY()

public:
    APlanetAtmosphereActor();

    // --- Pipeline ---
    //
    // The assets and passes the actor drives, rather than anything the march
    // reads. First in the panel because none of the parameters below mean
    // anything until these are right.
    //
    // Soft material references rather than hardcoded paths: a stale path logs a
    // warning and otherwise just looks like a broken material.

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CloudAtmosphere|Pipeline|Materials")
    TSoftObjectPtr<UMaterialInterface> PreprocessMaterial;

    /** Slot 1 for PlanetType::Terrestrial. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CloudAtmosphere|Pipeline|Materials")
    TSoftObjectPtr<UMaterialInterface> TerrestrialMarchMaterial;

    /** Slot 1 for PlanetType::GasGiant. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CloudAtmosphere|Pipeline|Materials")
    TSoftObjectPtr<UMaterialInterface> GasGiantMarchMaterial;

    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CloudAtmosphere|Pipeline|Materials")
    TSoftObjectPtr<UMaterialInterface> PostprocessMaterial;

    /** Recreates slot 1 against the current PlanetType and repopulates every
     *  slot. Call after changing PlanetType or either march material.
     *
     *  ALSO THE PARAMETER-CHECK RETRIGGER. Every push is verified against the
     *  material and warns once per name; this clears that filter, so pressing it
     *  re-reports anything the material no longer has. Only the ACTIVE model is
     *  pushed, so covering both means pressing it, flipping PlanetType, and
     *  pressing it again. */
    UFUNCTION(BlueprintCallable, CallInEditor, Category = "CloudAtmosphere|Pipeline|Materials")
    void RebuildMaterialInstances();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Pipeline")
    FAtmosphereCompositeParams Composite;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Pipeline")
    FAtmosphereSimulationParams Simulation;

    // --- Atmosphere ---
    //
    // What the actor IS, before anything about how it looks.

    /** True when spawned and driven by APlanetActor. Location and scale become
     *  read-only; rotation remains editable (controls light direction). */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "CloudAtmosphere|Atmosphere")
    bool bIsPlanetOwned = false;

    /** Which cloud model slot 1 renders. Changing this at runtime requires
     *  RebuildMaterialInstances — the material is chosen once, at creation. */
    UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "CloudAtmosphere|Atmosphere")
    EPlanetAtmosphereType PlanetType = EPlanetAtmosphereType::Terrestrial;

    // A GROUP OF ONE GETS NO WRAPPER. The substruct buys a fold-out, which is
    // only worth a click when there is more than one thing behind it.

    /** RGB direction is the hue, RGB magnitude is the intensity. The march and
     *  the directional light both derive from this, so they cannot disagree
     *  about the star. Light DIRECTION comes from the actor's rotation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Atmosphere")
    FLinearColor LightColor = FLinearColor(30.0f, 28.5f, 27.0f, 10.0f);

    /** Enable/disable the atmosphere's unbound post-process volume. This is the ONLY
     *  reliable off-switch for the ray march: the volume is not a primitive component,
     *  so hiding the actor or disabling its tick does not stop it. Parked planets MUST
     *  call this with false, or every pooled atmosphere keeps tinting the whole screen. */
    void SetAtmosphereActive(bool bActive);

    /** Aim the atmosphere's light/raymarch at the star. Points the actor's forward at
     *  StarWorldPos and runs the existing rotation->light sync. Called each frame by
     *  the owning planet from IStarLit::SetStarWorldPosition. */
    void OrientToStar(const FVector& StarWorldPos);

    // --- Parameters ---
    //
    // Common is one definition with an instance per model, so the shell, the
    // air and the march budget can differ without two definitions of what they
    // mean. The model structs are what only one march material declares.
    //
    // Every per-type property is EditConditionHides, so the details panel shows
    // exactly one Common set and one model set at a time.

    // GROUPS COME FROM SUBSTRUCTS, NOT FROM CATEGORY STRINGS. Category metadata
    // on a USTRUCT's members is inert while that struct renders as a row, so a
    // struct declared here shows as Category > struct row > every member flat.
    // Declaring one property per group instead puts the group name one level
    // under the category and its members under that.

    // The Common groups, once per model, each its own panel category.
    //
    // INLINED, so the group name comes from the category rather than from a
    // struct row underneath it. Their members carry no category of their own,
    // which is what lets one shared definition land in a different group per
    // model -- an absolute path on the members could only name one.
    //
    // EditConditionHides does not survive the inlining: the condition lives on
    // the property row, and there is no row left. Both models' groups are
    // visible at once, which the naming is what distinguishes.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Terrestrial|Terrestrial Geometry", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::Terrestrial", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereGeometryParams TerrestrialGeometry;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Terrestrial|Terrestrial Atmosphere Scattering", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::Terrestrial", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereAirScatteringParams TerrestrialAtmosphereScattering;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Terrestrial|Terrestrial Cloud Scattering", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::Terrestrial", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereCloudScatteringParams TerrestrialCloudScattering;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Terrestrial|Terrestrial Raymarch", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::Terrestrial", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereRaymarchParams TerrestrialRaymarch;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Terrestrial|Terrestrial Cloud", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::Terrestrial", EditConditionHides, ShowOnlyInnerProperties))
    FTerrestrialCloudParams Terrestrial;

    // THE SHELL IS THE DECK'S FIRST SHAPE TERM. Every other value under Shape
    // is a fraction of this one, so an absolute at the top of the group is what
    // the rest are read against. It sizes the air as well, which is why the
    // terrestrial copy keeps a group of its own.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Deck|Shape", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereGeometryParams GasGiantGeometry;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Deck", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides, ShowOnlyInnerProperties))
    FGasGiantDeckParams GasGiantDeck;

    /** Destination for the deck shadow bake, in the light's frame: a cascade of
     *  slices, all the same resolution, each covering a smaller radius.
     *
     *  ASSIGNED, NOT CREATED, matching FlowTarget: an asset can be opened beside
     *  the planet and watched while the light moves, which is the whole
     *  debugging loop for a map nothing samples yet.
     *
     *  The asset's own SizeX and SizeY set the resolution and should be square
     *  -- the map has one extent for both axes, so an unequal one stretches the
     *  disc. Format and UAV support are forced to RGBA16F on assignment, since
     *  a target without bCanCreateUAV accepts every dispatch and stays black. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Deck", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides))
    TObjectPtr<UTextureRenderTarget2DArray> GasGiantShadowTarget;

    /** Edge of each cascade slice, in texels. The target is resized to match, so
     *  this rather than the asset's own size is the handle.
     *
     *  EVERY LEVEL SHARES IT, and the world scale falls out of the extents: the
     *  disc slice is coarse, the detail slice is fine, and one number moves all
     *  of them together.
     *
     *  SQUARE BECAUSE THE MAP HAS ONE EXTENT. Both axes cover the same world
     *  distance, so unequal sizes stretch the planet disc.
     *
     *  Costs the square: 512 is 2 MB at RGBA16F, 1024 is 8, 2048 is 32. Spatial
     *  resolution is rarely the limit -- 1024 across a disc already resolves the
     *  flow grid several times over -- so a step or reconstruction artifact will
     *  not respond to this, which makes it a useful thing to rule out. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Deck", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides, ClampMin = "128", ClampMax = "4096"))
    int32 GasGiantShadowResolution = 1024;


    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Atmosphere Scattering", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereAirScatteringParams GasGiantAtmosphereScattering;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Cloud Scattering", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereCloudScatteringParams GasGiantCloudScattering;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Raymarch", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides, ShowOnlyInnerProperties))
    FAtmosphereRaymarchParams GasGiantRaymarch;

    // NO GROUP OF ITS OWN. Every member carries an absolute path -- the bands
    // and the extinction to Cloud Scattering, the four lighting scalars to
    // Terminator, the optical depth and band scale to the Deck -- so a category
    // here would only add an empty node beside the groups they went to. Named
    // for one of those groups so the fallback merges instead of stranding.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "CloudAtmosphere|Gas Giant|Gas Giant Cloud Scattering", meta = (EditCondition = "PlanetType == EPlanetAtmosphereType::GasGiant", EditConditionHides, ShowOnlyInnerProperties))
    FGasGiantScatterParams GasGiantScatter;

    // --- Lifecycle ---

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void Destroyed() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override { return true; }
    virtual void Tick(float DeltaTime) override;

    // --- Editor Property & Transform Locking ---

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual bool CanEditChange(const FProperty* InProperty) const override;
    virtual void PostEditMove(bool bFinished) override;
    virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
#endif

    /** Called by PlanetActor after spawn + attach. Sets bIsPlanetOwned, binds the
     *  transform guard, and runs Initialize. InScale sets the actor's scale before
     *  initialization. Skips the deferred OnConstruction path. */
    void InitializeFromPlanet(USceneComponent* InAttachParent,
        FVector InScale = FVector::ZeroVector);

private:
    /** Root component — child actors (PPV, light) attach here. */
    UPROPERTY()
    TObjectPtr<USceneComponent> AtmosphereRoot;

    /** Unbound post-process volume carrying the 3 blendable material instances. */
    UPROPERTY()
    TObjectPtr<APostProcessVolume> PostProcessVolume = nullptr;

    /** Directional light whose rotation and color are synced from actor rotation
     *  and the LightColor property. */
    UPROPERTY()
    TObjectPtr<ADirectionalLight> SunLight = nullptr;

    // --- Dynamic Material Instances (created from plugin base materials) ---

    /** Pass 0: preprocess (depth/setup). */
    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Preprocess = nullptr;

    /** Pass 1: atmosphere + cloud ray marching. Parent depends on PlanetType. */
    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Atmosphere = nullptr;

    /** Pass 2: distance-based blur compositing. */
    UPROPERTY()
    TObjectPtr<UMaterialInstanceDynamic> MID_Postprocess = nullptr;

    /** Air transmittance table, created on first use. Visible for inspection;
     *  it depends only on the radii and the air profile, so there is nothing
     *  to watch it do. */
    UPROPERTY(Transient, VisibleInstanceOnly, Category = "CloudAtmosphere")
    TObjectPtr<UTextureRenderTarget2D> TransmittanceTable = nullptr;

    /** Inputs and destination of the last enqueued bake. */
    FAtmosphereTransmittanceParams TransmittanceBaked;

    /** Which model MID_Atmosphere was created for. Guards against a PlanetType
     *  change reaching the parameter sweep before the material is rebuilt,
     *  which would push a whole model's parameters at a material that declares
     *  none of them and silently render the other model. */
    EPlanetAtmosphereType BuiltType = EPlanetAtmosphereType::Terrestrial;

    bool bInitialized = false;

    /** When true, Initialize runs on the next Tick. Set by OnConstruction to defer
     *  initialization until the world is fully ready. */
    bool bPendingInitialize = true;

    /** True once this actor has asked the subsystem to start. Cleared on
     *  teardown so a pooled planet does not leave the sim running. */
    bool bStartedSimulation = false;

    /** Cached scale set by the planet actor, used by the transform guard. */
    FVector PlanetDrivenScale = FVector::OneVector;

    /** Bound to RootComponent->TransformUpdated when planet-owned.
     *  Snaps location and scale back to planet-driven values; leaves rotation alone. */
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);

    /** Runs the full initialization: spawns child actors, creates material instances,
     *  pushes all parameters, and syncs the light. */
    void Initialize();

    /** Spawns the APostProcessVolume and ADirectionalLight as child actors,
     *  attaching them to AtmosphereRoot. */
    void SpawnChildActors();

    /** Destroys the post-process volume and directional light, nulls the MID pointers. */
    void DestroyChildActors();

    /** Creates the 3 dynamic material instances and assigns them as blendables.
     *  Slot 1's parent is chosen from PlanetType here and recorded in
     *  BuiltType. */
    void CreateMaterialInstances();

    /** Pushes every parameter to the atmosphere and postprocess instances.
     *  Called every tick and on property changes. Dispatches the cloud half on
     *  BuiltType, not PlanetType. */
    void UpdateMaterialParameters();

    /** The Common instance the live march material was built for.
     *
     *  KEYED ON BuiltType, not PlanetType, for the same reason the parameter
     *  sweep is: a type change that has not been rebuilt yet would otherwise
     *  push the wrong shell and march budget at the material on screen. */
    FAtmosphereCommonView GetCommonParams() const
    {
        const bool bGasGiant = BuiltType == EPlanetAtmosphereType::GasGiant;

        return FAtmosphereCommonView{
            bGasGiant ? GasGiantGeometry : TerrestrialGeometry,
            bGasGiant ? GasGiantAtmosphereScattering : TerrestrialAtmosphereScattering,
            bGasGiant ? GasGiantCloudScattering : TerrestrialCloudScattering,
            bGasGiant ? GasGiantRaymarch : TerrestrialRaymarch };
    }

    /** Geometry, light, air scattering, cloud lighting, raymarching. Both march
     *  materials, and no branching inside — the caller passes whichever Common
     *  instance is live. */
    void ApplyCommonParams(const FAtmosphereCommonView& Common, float PlanetRadius,
        const FVector& PlanetCenter, const FVector& LightDir);

    /** Cloud shell, noise and extinction. Terrestrial material only. */
    void ApplyTerrestrialParams(const FAtmosphereCommonView& Common);

    /** Field, volumes, per-band scattering and the planet's local frame.
     *
     *  The cloud radii are absent on purpose: the gas giant shader derives them
     *  from GG_TopBounds, and pushing them here would create a second source
     *  that can disagree with the bound the march is culling against. */
    void ApplyGasGiantParams(const FAtmosphereCommonView& Common, float PlanetRadius);

    /** Queues this frame's deck shadow bake with the sim subsystem.
     *
     *  SEPARATE FROM ApplyGasGiantParams because it pushes nothing to the
     *  material. The deck values it sends come from the same FGasGiantDeckParams
     *  getters, which is what keeps the deck the light sees identical to the
     *  deck the eye sees -- but its destination is a compute pass, not a MID. */
    void RequestGasGiantShadowBake(const FAtmosphereCommonView& Common, float PlanetRadius,
        const FVector& PlanetCenter, const FVector& LightDir);

    /** Forces the assigned shadow target to RGBA16F with UAV support, resizing
     *  only if the format is wrong. Returns false when there is nothing usable,
     *  having logged the reason at most once per state. */
    bool PrepareGasGiantShadowTarget();

    /** Suppresses the per-tick repeat of the shadow target complaint. Cleared
     *  when a usable target appears, so a fixed asset logs its recovery. */
    bool bWarnedShadowTarget = false;

    /** Creates the transmittance table if needed, rebakes it when its inputs or
     *  its resource change, and pushes it to the march material. */
    void UpdateTransmittanceTable(const FAtmosphereCommonView& Common, float PlanetRadius);

    /** Creates the table if absent and forces its fixed size, float format,
     *  clamp addressing and UAV support. */
    void PrepareTransmittanceTable();

    /** Starts the sim subsystem against the deck's config. */
    void StartGasGiantSimulation();

    /** Simulated time from the sim subsystem, or 0 when it is not running.
     *
     *  NOT WORLD TIME. The field is coherent against the sim's own clock, and
     *  the two diverge the moment the sim pauses, is stepped by hand, or is
     *  restored from a snapshot — after which the warp advects a field that has
     *  not moved. */
    float GetGasGiantTime() const;

    /** Syncs the directional light's rotation and color/intensity from the actor's
     *  rotation and LightColor property. */
    void UpdateLightFromRotation();

    /** Resolves a soft material reference, logging which one failed. */
    static UMaterialInterface* LoadMaterialAsset(const TSoftObjectPtr<UMaterialInterface>& Ref, const TCHAR* Label);
};