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

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Shape",
        meta = (ClampMin = "1.0"))
    double OceanRadius = 1000.0;

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

    // Chunks whose all 4 corner densities are below this threshold contain no ocean
    // surface and are permanently excluded from the chunk map and LOD loop.
    // Negative values represent depth below the terrain surface (same sign convention
    // as the density compositor output). Default -1000 = 10m below surface at planet scale.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Culling")
    float OceanSurfaceThreshold = -1000.f;

    // ---------------------------------------------------------------------------
    // Standalone density params
    // Used only when no compositor has been set via SetCompositor().
    // Should match the terrain actor's params so the ocean sits at the right sea level.
    // When the planet actor orchestrates both, it will call SetCompositor() instead.
    // ---------------------------------------------------------------------------

    // Planet radius in world units — should match AdaptiveVoxelActor scale.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Standalone")
    double StandalonePlanetRadius = 80000000.0;

    // Noise amplitude as a ratio of planet radius — should match AdaptiveVoxelActor.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ocean|Standalone",
        meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double StandaloneNoiseAmplitudeRatio = 0.25;

    // Set from outside (e.g. APlanetActor) before or during Initialize().
    // Replaces any internally-built standalone compositor.
    // If called after initialization, call Initialize() again to rebuild with the new compositor.
    void SetCompositor(TSharedPtr<FDensitySampleCompositor> InCompositor)
    {
        Compositor = InCompositor;
        Noise = nullptr; // release standalone noise node if one was built
    }

    FVector GetCameraPosition()  const { return CameraPosition; }
    double  GetCameraFOV()       const { return CameraFOV; }

    // Accessor for chunk components to attach to — inherits actor position/rotation,
    // absolute scale so the tree-at-origin geometry is never scaled by the actor.
    USceneComponent* GetMeshAttachmentRoot() const { return MeshAttachmentRoot; }

    TSharedPtr<FOceanQuadTreeNode> GetNodeByIndex(const FQuadIndex& Index) const;

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void BeginDestroy() override;
    virtual bool ShouldTickIfViewportsOnly() const override;
    virtual void Tick(float DeltaTime) override;

    TSharedPtr<FOceanQuadTreeNode> RootNodes[6];
    TMap<TSharedPtr<FOceanQuadTreeNode>, TSharedPtr<FOceanMeshChunk>> ChunkMap;

private:
    // Mesh components attach here. Inherits actor position and rotation,
    // but uses absolute scale (1,1,1) so the cube-sphere geometry built at
    // OceanRadius world units is never scaled by the actor transform.
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    // Compositor provided externally. Null = build standalone compositor on Initialize().
    TSharedPtr<FDensitySampleCompositor> Compositor;

    // Noise node for standalone compositor — kept alive for the compositor lambda.
    FastNoise::SmartNode<> Noise;

    FVector CameraPosition = FVector::ZeroVector;
    FVector LastLodCameraPos = FVector(FLT_MAX);
    double  CameraFOV = 90.0;

    std::atomic<bool> bInitialized = false;
    std::atomic<bool> bIsDestroyed = false;
    std::atomic<bool> bLodUpdateRunning = false;
    std::atomic<uint32> InitGeneration = 0;
    bool bIsInitializing = false;

    // Cached params from last Initialize() — only structural changes trigger re-init.
    // Position and rotation changes are handled live by the actor transform.
    double  LastInitOceanRadius = -1.0;
    int32   LastInitFaceResolution = -1;
    int32   LastInitChunkDepth = -1;
    int32   LastInitMinDepth = -1;
    int32   LastInitMaxDepth = -1;
    float   LastInitThreshold = FLT_MAX;
    double  LastInitStandalonePlanetRadius = -1.0;
    double  LastInitStandaloneNoiseAmplitudeRatio = -1.0;

    FTimerHandle LodTimerHandle;

    void Initialize();
    void CleanupComponents();
    void PopulateChunks();

    static void RebuildChunkStreamData(TSharedPtr<FOceanMeshChunk> Chunk,
        TSharedPtr<FOceanQuadTreeNode> ChunkNode);

    void RunLodUpdateTask();
};