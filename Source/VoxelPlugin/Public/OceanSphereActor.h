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

    // Ocean radius is driven by actor scale (uniform across all axes).
    // Default scale 80,000,000 = 800 km radius, matching the default terrain actor.
    // This property is read-only at runtime — adjust actor scale to resize.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ocean|Shape")
    double OceanRadius = 100000000.0;

    // The terrain planet's base radius — must match the terrain actor's scale (GetActorScale3D().GetMax()).
    // Used to evaluate the same SDF as the terrain so depth values are meaningful:
    // density = OceanRadius - (TerrainPlanetRadius + NoiseHeight).
    // Positive = underwater, negative = above terrain (land).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "1.0"))
    double TerrainPlanetRadius = 90000000.0;

    // Noise amplitude as a ratio of TerrainPlanetRadius.
    // Must match the terrain actor's NoiseAmplitudeRatio so both evaluate the same heightfield.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.25;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Mesh",
        meta = (ClampMin = "3"))
    int32 FaceResolution = 17;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|LOD")
    int32 ChunkDepth = 2;

    // Must be >= ChunkDepth
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

    // Density compositor — built at Initialize() time with the same heightmap layer
    // as AAdaptiveVoxelActor so the ocean evaluates the identical SDF.
    TSharedPtr<FDensitySampleCompositor> Compositor;

    // Noise node kept alive for the compositor lambda lifetime.
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

    // Samples the 4 sphere-projected corners of a root node before any splits.
    void ComputeRootNodeDensities(TSharedPtr<FOceanQuadTreeNode> Node);

    // Recursive post-pass: walks an already-split subtree and pushes density
    // into every child node that doesn't yet have bDensitySampled.
    // Does not touch mesh data, positions, winding, or ChunkAnchorCenter.
    void PushDensityToChildren(TSharedPtr<FOceanQuadTreeNode> Node);

    static void RebuildChunkStreamData(TSharedPtr<FOceanMeshChunk> Chunk,
        TSharedPtr<FOceanQuadTreeNode> ChunkNode);

    void RunLodUpdateTask();
};