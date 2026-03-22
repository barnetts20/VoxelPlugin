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
    double OceanRadius = 8000000.0;

    // Must match the terrain actor's scale (GetActorScale3D().GetMax()).
    // density = OceanRadius - (TerrainPlanetRadius + NoiseHeight):
    //   positive = underwater, negative = above water (land).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "1.0"))
    double TerrainPlanetRadius = 7200000.0;

    // Must match the terrain actor's NoiseAmplitudeRatio.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.25;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh",
        meta = (ClampMin = "3"))
    int32 FaceResolution = 3;

    // Chunk depth for the ocean quadtree. Will be auto-derived in a future pass.
    int32 ChunkDepth = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    int32 MinDepth = 2;

    // Target vertex spacing in world units (cm). MaxDepth is computed automatically
    // from this value and the ocean radius so that the finest LOD achieves roughly
    // this spacing between adjacent vertices on the sphere surface.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 100.0;

    // Computed from TargetPrecision and OceanRadius at init time.
    // Clamped to [MinDepth, MaxKeyDepth].
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    int32 MaxDepth = 6;

    // The actual vertex spacing (cm) achieved at MaxDepth after key-limit clamping.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|LOD")
    double ActualPrecision = 0.0;

    // Hard limit imposed by FQuadIndex (2 bits per level, 62 path bits + 2 sentinel = 64).
    static constexpr int32 MaxKeyDepth = 31;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD",
        meta = (ClampMin = "0.001"))
    double ScreenSpaceThreshold = 0.075;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    double MinLodInterval = 0.1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    double VelocityLookAheadFactor = 8.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    bool bTickInEditor = true;

    // Chunks where the maximum vertex depth (cm) is below this threshold are culled.
    // Negative values allow chunks that are somewhat above the ocean surface (land) to render.
    // 0 = cull any chunk with no underwater vertices at all.
    // e.g. -500000 = cull only chunks more than 5km above the ocean surface.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Culling")
    float ChunkCullingDepthThreshold = 0.f;

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

    void Initialize();
    void CleanupComponents();
    void PopulateChunks();

    static void RebuildChunkStreamData(TSharedPtr<FOceanMeshChunk> Chunk,
        TSharedPtr<FOceanQuadTreeNode> ChunkNode);
    void RunLodUpdateTask();
};