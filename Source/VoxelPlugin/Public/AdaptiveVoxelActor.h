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

UCLASS()
class VOXELPLUGIN_API AAdaptiveVoxelActor : public ARealtimeMeshActor
{
    GENERATED_BODY()

private:
    TSharedPtr<FAdaptiveOctree> AdaptiveOctree;

    // Mesh chunks attach to this component. Inherits actor position and rotation
    // but uses absolute scale (1,1,1). Octree is built at world scale; scale changes
    // trigger full reconstruction via OnConstruction.
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    FVector CameraPosition = FVector::ZeroVector;

    FVector LastLodUpdatePosition = FVector(FLT_MAX);

    double CameraFOV = 90;

    FRWLock OctreeLock;

    bool TickInEditor = true;

    std::atomic<bool> Initialized = false;

    std::atomic<bool> IsDestroyed = false;

    FVector LastInitScale = FVector::ZeroVector;

    FastNoise::SmartNode<> Noise;
public:
    AAdaptiveVoxelActor();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Materials")
    UMaterialInterface* SurfaceMaterial;

    // Planet radius in world units is determined by actor scale (max component).
    // The octree is built at world scale in actor-local space (origin 0,0,0).
    // Position and rotation changes are handled live by the actor transform.
    // Scale changes trigger full reconstruction.

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Terrain", meta = (ClampMin = "0.01", ClampMax = "1.0"))
    double NoiseAmplitudeRatio = 0.25;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int ChunkDepth = 4;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree")
    int MinDepth = 7;

    // Target voxel spacing in world units (cm). MaxDepth is computed automatically
    // so the finest LOD voxel cells are approximately this size.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree",
        meta = (ClampMin = "1.0"))
    double TargetPrecision = 100.0;

    // Computed from TargetPrecision and planet radius at init time.
    // Clamped to [MinDepth, MaxKeyDepth].
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Octree")
    int MaxDepth = 18;

    // The actual voxel spacing (cm) achieved at MaxDepth after key-limit clamping.
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Octree")
    double ActualPrecision = 0.0;

    // Hard limit imposed by FMortonIndex (3 bits per level, 126 bits across two uint64s).
    static constexpr int32 MaxKeyDepth = 42;

    // Depth beyond which noise sampling is replaced by trilinear interpolation
    // from parent corner densities. Noise loses float precision past this depth,
    // but deeper splits still provide geometric detail for editing.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Octree",
        meta = (ClampMin = "1"))
    int32 PrecisionDepthFloor = 21;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double ScreenSpaceThreshold = .075;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double MinDataUpdateInterval = .1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LOD")
    double VelocityLookAheadFactor = 8;

    virtual void OnConstruction(const FTransform& Transform) override;

    virtual void BeginPlay() override;

    virtual void BeginDestroy() override;

    virtual bool ShouldTickIfViewportsOnly() const override;

    virtual void Tick(float DeltaTime) override;

protected:
    std::atomic<bool> DataUpdateIsRunning = false;

    std::atomic<bool> MeshUpdateIsRunning = false;

    std::atomic<bool> EditUpdateIsRunning = false;

    FTimerHandle DataUpdateTimerHandle;

    void CleanSceneRoot();

    void Initialize();

    void RunDataUpdateTask();

    void RunMeshUpdateTask();

    void RunEditUpdateTask(FVector InEditCenter, double InEditRadius, double InEditStrength, int InEditResolution);
};