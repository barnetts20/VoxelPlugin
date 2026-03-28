// OceanSphereActor.h — Ocean surface actor using a cube-sphere quadtree.
// Renders the ocean as 6 quadtree faces projected onto a sphere at OceanRadius,
// with LOD-driven subdivision and depth-based triangle culling where terrain
// pokes above the waterline.
//
// Scale determines OceanRadius. When planet-owned, the terrain compositor is
// shared so ocean depth = OceanRadius - terrain SDF (positive = underwater).

#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include "FOceanSharedStructs.h"
#include "FOceanQuadTreeNode.h"
#include "FDensitySampleCompositor.h"
#include "FastNoise/FastNoise.h"
#include "OceanSphereActor.generated.h"

/** Ocean surface actor. Manages a cube-sphere quadtree (6 face roots) that
 *  subdivides based on screen-space size and rebuilds mesh chunks with
 *  LOD-matched edge stitching.
 *
 *  The ocean samples the terrain compositor to determine depth at each vertex.
 *  Chunks where all vertices are above water (terrain exposed) are culled.
 *  An async LOD update task runs on a background thread with a configurable
 *  minimum interval, similar to AAdaptiveVoxelActor's pipeline. */
UCLASS()
class VOXELPLUGIN_API AOceanSphereActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

public:
    AOceanSphereActor();

    /** True when spawned and driven by APlanetActor. Planet-driven properties
     *  become read-only -- edit them on the planet instead. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean")
    bool bIsPlanetOwned = false;

    /** Ocean surface material. When standalone, auto-loads the plugin's ocean material. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Material",
        meta = (EditCondition = "!bIsPlanetOwned"))
    UMaterialInterface* OceanMaterial = nullptr;

    // --- Shape ---

    /** Derived from actor scale at init time. The sphere radius of the ocean surface. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Shape")
    double OceanRadius = 107500000.0;

    /** Must match the terrain actor's scale. Used to compute ocean depth:
     *  depth = OceanRadius - (TerrainPlanetRadius + NoiseHeight).
     *  Positive = underwater, negative = terrain above water (land). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "1.0", EditCondition = "!bIsPlanetOwned"))
    double TerrainPlanetRadius = 100000000.0;

    /** Must match the terrain actor's NoiseAmplitudeRatio. Used by the standalone
     *  compositor to generate matching terrain density for depth calculation. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "0.01", ClampMax = "1.0", EditCondition = "!bIsPlanetOwned"))
    double NoiseAmplitudeRatio = 0.15;

    // --- Mesh ---

    /** Number of vertices per edge within each quadtree leaf node.
     *  Higher values produce smoother patches but cost more triangles per node. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh",
        meta = (ClampMin = "3"))
    int32 FaceResolution = 3;

    // --- LOD ---

    /** Quadtree depth at which nodes become mesh chunks. Fixed at 3 for the ocean
     *  quadtree -- cube-sphere faces have coarser float precision requirements
     *  than the voxel octree. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    int32 ChunkDepth = 3;

    /** Minimum subdivision depth maintained regardless of camera distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    int32 MinDepth = 4;

    /** Target vertex spacing in world units (cm). MaxDepth is computed automatically
     *  so the finest LOD achieves roughly this spacing between adjacent vertices. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 200.0;

    /** Computed from TargetPrecision and OceanRadius at init time.
     *  At default scale (107.5M radius, FaceRes=3, TargetPrecision=200): MaxDepth=20. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    int32 MaxDepth = 20;

    /** The actual vertex spacing (cm) achieved at MaxDepth after key-limit clamping. */
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    double ActualPrecision = 0.0;

    /** Hard limit imposed by FQuadIndex (2 bits per level, 62 path bits + 2 sentinel = 64). */
    static constexpr int32 MaxKeyDepth = 31;

    /** Screen-space size threshold for LOD split/merge. The node is subdivided by
     *  FaceResolution, so the effective per-vertex threshold is
     *  ScreenSpaceThreshold / (FaceResolution - 1). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD",
        meta = (ClampMin = "0.001"))
    double ScreenSpaceThreshold = 0.225;

    /** Minimum interval (seconds) between LOD update passes. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    double MinLodInterval = 0.05;

    /** Multiplier on camera velocity for predictive LOD. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    double VelocityLookAheadFactor = 8.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    bool bTickInEditor = true;

    // --- Culling ---

    /** Per-triangle terrain SDF threshold for culling. Triangles where all 3 vertices
     *  have depth below this value are culled (terrain fully above water). Negative
     *  values extend the mesh skirt above the waterline to prevent WPO waves from
     *  exposing gaps. e.g. -50000 = keep triangles up to 500m above the ocean surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Culling")
    float TriangleCullDepthThreshold = -10000.f;

    // --- Public Accessors ---

    FVector GetCameraPosition() const { return CameraPosition; }
    double  GetCameraFOV()      const { return CameraFOV; }
    USceneComponent* GetMeshAttachmentRoot() const { return MeshAttachmentRoot; }
    TSharedPtr<FDensitySampleCompositor> GetCompositor() const { return Compositor; }

    /** Returns the cached triangle grid for a given resolution, building it on first access. */
    const FOceanMeshGrid& GetMeshGrid(int32 Res);

    /** Walks the quadtree to find the node matching a FQuadIndex. Returns nullptr if
     *  the path doesn't exist (node was merged). Used by CheckNeighbors for LOD queries. */
    TSharedPtr<FOceanQuadTreeNode> GetNodeByIndex(const FQuadIndex& Index) const;

    // --- Lifecycle ---

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
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

    /** Called by APlanetActor -- uses the provided compositor (shared with terrain)
     *  instead of creating one. Scale must already be set to OceanRadius. */
    void InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
        USceneComponent* InAttachParent = nullptr);

    /** Tears down the current quadtree and rebuilds from scratch. */
    void Initialize();

private:
    /** The 6 cube-face quadtree roots. Each face is an independent tree that
     *  shares topology via FQuadIndex cross-face neighbor lookups. */
    TSharedPtr<FOceanQuadTreeNode> RootNodes[6];

    /** Maps chunk-depth quadtree nodes to their mesh chunks. */
    TMap<TSharedPtr<FOceanQuadTreeNode>, TSharedPtr<FOceanMeshChunk>> ChunkMap;

    /** Mesh chunks attach to this component. Inherits actor position/rotation,
     *  uses absolute scale. */
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    /** Density compositor -- either shared from PlanetActor or built standalone.
     *  Provides terrain SDF density at ocean vertex positions for depth calculation. */
    TSharedPtr<FDensitySampleCompositor> Compositor;

    /** FastNoise node for standalone mode's terrain heightmap. Unused when planet-owned. */
    FastNoise::SmartNode<> Noise;

    // --- Camera State ---
    FVector CameraPosition = FVector::ZeroVector;
    FVector LastLodCameraPos = FVector(FLT_MAX);
    double  CameraFOV = 90.0;

    // --- Async State ---

    /** Set after Initialize completes. Gates Tick and LOD updates. */
    std::atomic<bool>   bInitialized = false;

    /** Set in BeginDestroy to prevent async tasks from touching a dying actor. */
    std::atomic<bool>   bIsDestroyed = false;

    /** True while the background LOD update task is in flight. */
    std::atomic<bool>   bLodUpdateRunning = false;

    /** Monotonically increasing generation counter. Incremented on each Initialize
     *  so in-flight async tasks from a previous generation can detect they're stale
     *  and exit without touching the new tree state. */
    std::atomic<uint32> InitGeneration = 0;

    /** True during Initialize to prevent re-entrant init from OnConstruction/Tick. */
    bool                bIsInitializing = false;

    // --- Change Detection ---
    FVector LastInitScale = FVector::ZeroVector;
    double  LastTerrainPlanetRadius = 0.0;

    /** Timer for MinLodInterval delay between LOD update completions. */
    FTimerHandle LodTimerHandle;

    /** Cached scale set by the planet actor, used by the transform guard. */
    FVector PlanetDrivenScale = FVector::OneVector;

    /** Bound to RootComponent->TransformUpdated when planet-owned.
     *  Snaps transforms back to planet-driven values. */
    void OnTransformUpdated(USceneComponent* Component, EUpdateTransformFlags Flags, ETeleportType Teleport);

    /** Static mesh grid cache -- keyed by FaceResolution. Built once per resolution,
     *  reused across all nodes at that resolution. */
    TMap<int32, FOceanMeshGrid> MeshGridCache;

    /** Internal init path shared by Initialize and InitializeFromPlanet.
     *  Builds the quadtree, populates chunks, and kicks off the first LOD pass. */
    void InitializeInternal(TSharedPtr<FDensitySampleCompositor> InCompositor);

    /** Destroys all RealtimeMesh components and clears the chunk map. */
    void CleanupComponents();

    /** Creates mesh chunks for all chunk-depth leaf nodes and adds them to ChunkMap.
     *  Runs initial mesh stream builds in parallel. */
    void PopulateChunks();

    /** Rebuilds vertex positions, normals, UVs, and triangles (inner + edge) for a
     *  single chunk. Handles edge stitching based on neighbor LOD differences and
     *  per-triangle depth culling. Static so it can run in a ParallelFor. */
    static void RebuildChunkStreamData(TSharedPtr<FOceanMeshChunk> Chunk,
        TSharedPtr<FOceanQuadTreeNode> ChunkNode);

    /** Background LOD update task: evaluates split/merge for all leaves, checks
     *  neighbor LODs, rebuilds dirty chunks, and pushes mesh updates. Chains back
     *  to itself after MinLodInterval. */
    void RunLodUpdateTask();
};