#pragma once

#include "CoreMinimal.h"
#include "FastNoise/FastNoise.h"
#include "GameFramework/Actor.h"
#include "Materials/Material.h"
#include "MaterialDomain.h"
#include "FAdaptiveOctree.h"
#include "FMeshingStructs.h"
#include "RealtimeMeshActor.h"
#include "FSparseEditStore.h"
#include "AdaptiveVoxelActor.generated.h"

using namespace RealtimeMesh;

UENUM(BlueprintType)
enum class EChunkCullingMode : uint8
{
    // Only process nodes with density sign changes. Optimal for continuous surfaces (planets).
    Surface     UMETA(DisplayName = "Surface"),
    // Process all nodes within a control volume sphere. Required for scattered surfaces (debris fields).
    Volume      UMETA(DisplayName = "Volume")
};

UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

private:
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;

    // Deferred construction � params are stored on Initialize (game thread),
    // octree is built on the first RunDataUpdateTask (background thread).
    TSharedPtr<FOctreeParams> PendingParams;

    // Mesh chunks attach to this component. Inherits actor position and rotation
    // but uses absolute scale (1,1,1). Octree is built at world scale; scale changes
    // trigger full reconstruction via OnConstruction.
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    FVector CameraPosition = FVector::ZeroVector;

    FVector LastLodUpdatePosition = FVector(FLT_MAX);

    double CameraFOV = 90;

    FRWLock OctreeLock;

    bool bTickInEditor = true;

    std::atomic<bool> Initialized = false;

    std::atomic<bool> IsDestroyed = false;

    FVector LastInitScale = FVector::ZeroVector;

    FastNoise::SmartNode<> Noise;

    // Cached scale set by the planet actor, used by the transform guard to restore on edits.
    FVector PlanetDrivenScale = FVector::OneVector;

    // Bound to RootComponent->TransformUpdated when planet-owned.
    // Reverts any editor-driven transform changes on locked components.
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);

public:
    AAdaptiveVoxelActor();

    // True when this actor is spawned and driven by an APlanetActor.
    // Planet-driven properties (NoiseAmplitudeRatio, SurfaceMaterial) become
    // read-only on this actor — edit them on the planet instead.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain")
    bool bIsPlanetOwned = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Materials",
        meta = (EditCondition = "!bIsPlanetOwned"))
    UMaterialInterface* SurfaceMaterial = nullptr;

    // Planet radius in world units is determined by actor scale (max component).
    // The octree is built at world scale in actor-local space (origin 0,0,0).
    // Position and rotation changes are handled live by the actor transform.
    // Scale changes trigger full reconstruction.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain",
        meta = (ClampMin = "0.01", ClampMax = "1.0", EditCondition = "!bIsPlanetOwned"))
    double NoiseAmplitudeRatio = 0.15;

    // Computed at init time from float precision requirements.
    // At default scale (100M radius, 0.15 NAR): RootExtent=120.75M → ChunkDepth=4.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Octree")
    int ChunkDepth = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree")
    int MinDepth = 8;

    // Target voxel spacing in world units (cm). MaxDepth is computed automatically
    // so the finest LOD voxel cells are approximately this size.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 100.0;

    // Computed from TargetPrecision and planet radius at init time.
    // Clamped to [MinDepth, MaxKeyDepth].
    // At default scale (100M radius, 0.15 NAR, TargetPrecision=100): MaxDepth=22.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Octree")
    int MaxDepth = 22;

    // The actual voxel spacing (cm) achieved at MaxDepth after key-limit clamping.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Terrain|Octree")
    double ActualPrecision = 0.0;

    // Hard limit imposed by FMortonIndex (3 bits per level, 126 bits across two uint64s).
    static constexpr int32 MaxKeyDepth = 42;

    // Depth beyond which noise sampling is replaced by trilinear interpolation
    // from parent corner densities. Noise loses float precision past this depth,
    // but deeper splits still provide geometric detail for editing.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree",
        meta = (ClampMin = "1"))
    int32 PrecisionDepthFloor = 19;

    // Controls how the octree decides which nodes to split to chunk depth.
    // Surface: only split where the surface crosses node corners (planets).
    // Volume: split all nodes within a control sphere (debris fields, asteroids).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree")
    EChunkCullingMode ChunkCullingMode = EChunkCullingMode::Surface;

    // Radius of the control volume for Volume culling mode.
    // 0 = use the full octree root extent. Only relevant when ChunkCullingMode = Volume.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|Octree",
        meta = (EditCondition = "ChunkCullingMode == EChunkCullingMode::Volume", ClampMin = "0.0"))
    double VolumeSdfRadius = 0.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double ScreenSpaceThreshold = 0.075;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double MinDataUpdateInterval = 0.05;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain|LOD")
    double VelocityLookAheadFactor = 2.0;

    virtual void OnConstruction(const FTransform& Transform) override;

    virtual void BeginPlay() override;

    virtual void BeginDestroy() override;

    virtual bool ShouldTickIfViewportsOnly() const override;

    virtual void Tick(float DeltaTime) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
    virtual bool CanEditChange(const FProperty* InProperty) const override;
    virtual void PostEditMove(bool bFinished) override;
    virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
    virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
#endif

    USceneComponent* GetMeshAttachmentRoot() const { return MeshAttachmentRoot; }

    // Called by APlanetActor — uses the provided compositor instead of creating one.
    // The actor's own scale must already be set to the desired planet radius.
    // If InAttachParent is provided, MeshAttachmentRoot is re-parented to it.
    void InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
        USceneComponent* InAttachParent = nullptr);

    // Tears down the current octree and rebuilds from scratch using current property values.
    // Called by OnConstruction (scale change), PostEditChangeProperty (structural param change),
    // InitializeFromPlanet (planet-driven init), and BeginPlay.
    void Initialize();

protected:
    std::atomic<bool> DataUpdateIsRunning = false;

    std::atomic<bool> MeshUpdateIsRunning = false;

    std::atomic<bool> EditUpdateIsRunning = false;

    FTimerHandle DataUpdateTimerHandle;

    void CleanSceneRoot();

    void RunDataUpdateTask();

    void RunMeshUpdateTask();

    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};