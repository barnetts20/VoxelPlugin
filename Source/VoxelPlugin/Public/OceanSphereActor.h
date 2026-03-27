#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include "FOceanSharedStructs.h"
#include "FOceanQuadTreeNode.h"
#include "FDensitySampleCompositor.h"
#include "FastNoise/FastNoise.h"
#include "OceanSphereActor.generated.h"

UCLASS()
class VOXELPLUGIN_API AOceanSphereActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

public:
    AOceanSphereActor();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Material")
    UMaterialInterface* OceanMaterial = nullptr;

    // Driven by actor scale. Read-only at runtime � adjust scale to resize.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Shape")
    double OceanRadius = 107500000.0;

    // Must match the terrain actor's scale (GetActorScale3D().GetMax()).
    // density = OceanRadius - (TerrainPlanetRadius + NoiseHeight):
    //   positive = underwater, negative = above water (land).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "1.0"))
    double TerrainPlanetRadius = 100000000.0;

    // Must match the terrain actor's NoiseAmplitudeRatio.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.15;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh",
        meta = (ClampMin = "3"))
    int32 FaceResolution = 3;

    // Quadtree depth at which nodes become mesh chunks.
    // Fixed at 3 for the ocean quadtree — cube-sphere faces have coarser
    // float precision requirements than the voxel octree.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    int32 ChunkDepth = 3;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    int32 MinDepth = 4;

    // Target vertex spacing in world units (cm). MaxDepth is computed automatically
    // from this value and the ocean radius so that the finest LOD achieves roughly
    // this spacing between adjacent vertices on the sphere surface.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 200.0;

    // Computed from TargetPrecision and OceanRadius at init time.
    // Clamped to [MinDepth, MaxKeyDepth].
    // At default scale (107.5M radius, FaceRes=3, TargetPrecision=200): MaxDepth=20.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    int32 MaxDepth = 20;

    // The actual vertex spacing (cm) achieved at MaxDepth after key-limit clamping.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    double ActualPrecision = 0.0;

    // Hard limit imposed by FQuadIndex (2 bits per level, 62 path bits + 2 sentinel = 64).
    static constexpr int32 MaxKeyDepth = 31;

    // LOD screen-space threshold. The node is subdivided by FaceResolution, so
    // the effective vertex threshold is ScreenSpaceThreshold / (FaceResolution - 1).
    // Default 0.225 with FaceResolution=3 gives ~0.1125 per-vertex threshold.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD",
        meta = (ClampMin = "0.001"))
    double ScreenSpaceThreshold = 0.225;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    double MinLodInterval = 0.05;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    double VelocityLookAheadFactor = 8.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    bool bTickInEditor = true;

    // Per-triangle depth threshold for culling. Triangles where all 3 vertices
    // have depth below this value are culled. Negative values extend the ocean
    // mesh skirt above the waterline to prevent WPO waves from exposing gaps.
    // e.g. -50000 = keep triangles up to 500m above the ocean surface.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Culling")
    float TriangleCullDepthThreshold = -10000.f;

    FVector GetCameraPosition() const { return CameraPosition; }
    double  GetCameraFOV()      const { return CameraFOV; }
    USceneComponent* GetMeshAttachmentRoot() const { return MeshAttachmentRoot; }
    TSharedPtr<FDensitySampleCompositor> GetCompositor() const { return Compositor; }

    // Returns the grid cache for a given resolution, building it if needed.
    const FOceanMeshGrid& GetMeshGrid(int32 Res);

    TSharedPtr<FOceanQuadTreeNode> GetNodeByIndex(const FQuadIndex& Index) const;

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    void InitializeFromPlanet(TSharedPtr<FDensitySampleCompositor> InCompositor,
        USceneComponent* InAttachParent = nullptr);

    // Tears down the current quadtree and rebuilds from scratch using current property values.
    // Called by OnConstruction, PostEditChangeProperty, InitializeFromPlanet, and BeginPlay.
    void Initialize();

private:
    TSharedPtr<FOceanQuadTreeNode> RootNodes[6];
    TMap<TSharedPtr<FOceanQuadTreeNode>, TSharedPtr<FOceanMeshChunk>> ChunkMap;

    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    TSharedPtr<FDensitySampleCompositor> Compositor;
    FastNoise::SmartNode<> Noise;

    FVector CameraPosition = FVector::ZeroVector;
    FVector LastLodCameraPos = FVector(FLT_MAX);
    double  CameraFOV = 90.0;

    std::atomic<bool>   bInitialized = false;
    std::atomic<bool>   bIsDestroyed = false;
    std::atomic<bool>   bLodUpdateRunning = false;
    std::atomic<uint32> InitGeneration = 0;
    bool                bIsInitializing = false;

    FVector LastInitScale = FVector::ZeroVector;
    double  LastTerrainPlanetRadius = 0.0;

    FTimerHandle LodTimerHandle;

    // Static mesh grid cache — keyed by resolution.
    TMap<int32, FOceanMeshGrid> MeshGridCache;

    void InitializeInternal(TSharedPtr<FDensitySampleCompositor> InCompositor);
    void CleanupComponents();
    void PopulateChunks();

    static void RebuildChunkStreamData(TSharedPtr<FOceanMeshChunk> Chunk,
        TSharedPtr<FOceanQuadTreeNode> ChunkNode);
    void RunLodUpdateTask();
};