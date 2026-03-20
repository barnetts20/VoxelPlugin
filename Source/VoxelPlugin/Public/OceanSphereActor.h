#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include "FOceanSharedStructs.h"
#include "FDensitySampleCompositor.h"
#include "FastNoise/FastNoise.h"
#include "OceanSphereActor.generated.h"

class FOceanQuadTreeNode;
struct FOceanMeshChunk;

UCLASS()
class VOXELPLUGIN_API AOceanSphereActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

public:
    AOceanSphereActor();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Material")
    UMaterialInterface* OceanMaterial = nullptr;

    // Driven by actor scale. Read-only at runtime — adjust scale to resize.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Shape")
    double OceanRadius = 100000000.0;

    // Must match the terrain actor's scale (GetActorScale3D().GetMax()).
    // density = OceanRadius - (TerrainPlanetRadius + NoiseHeight):
    //   positive = underwater, negative = above water (land).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "1.0"))
    double TerrainPlanetRadius = 90000000.0;

    // Must match the terrain actor's NoiseAmplitudeRatio.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.25;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh",
        meta = (ClampMin = "3"))
    int32 FaceResolution = 17;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    int32 ChunkDepth = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    int32 MinDepth = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    int32 MaxDepth = 6;

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

    TSharedPtr<FOceanQuadTreeNode> GetNodeByIndex(const FQuadIndex& Index) const;

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

    TSharedPtr<FOceanQuadTreeNode> RootNodes[6];
    TMap<TSharedPtr<FOceanQuadTreeNode>, TSharedPtr<FOceanMeshChunk>> ChunkMap;

private:
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

    void Initialize();
    void CleanupComponents();
    void PopulateChunks();

    static void RebuildChunkStreamData(TSharedPtr<FOceanMeshChunk> Chunk,
        TSharedPtr<FOceanQuadTreeNode> ChunkNode);
    void RunLodUpdateTask();
};