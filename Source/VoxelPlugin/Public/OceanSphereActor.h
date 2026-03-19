#pragma once

#include "CoreMinimal.h"
#include "RealtimeMeshActor.h"
#include "FOceanSharedStructs.h"
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
    double OceanRadius = 80000000.0;

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
    // absolute scale (1,1,1) so the sphere built at world-scale OceanRadius
    // is never additionally scaled by the actor transform.
    UPROPERTY()
    TObjectPtr<USceneComponent> MeshAttachmentRoot;

    // Camera in actor-local space (position + rotation only, no scale).
    // Matches the octree convention so LOD distances are in world units.
    FVector CameraPosition = FVector::ZeroVector;
    FVector LastLodCameraPos = FVector(FLT_MAX);
    double  CameraFOV = 90.0;

    std::atomic<bool>   bInitialized = false;
    std::atomic<bool>   bIsDestroyed = false;
    std::atomic<bool>   bLodUpdateRunning = false;
    std::atomic<uint32> InitGeneration = 0;
    bool                bIsInitializing = false;

    // Scale from last Initialize() — scale changes trigger reconstruction.
    // Position/rotation changes are handled live via MeshAttachmentRoot.
    FVector LastInitScale = FVector::ZeroVector;

    FTimerHandle LodTimerHandle;

    void Initialize();
    void CleanupComponents();
    void PopulateChunks();

    static void RebuildChunkStreamData(TSharedPtr<FOceanMeshChunk> Chunk,
        TSharedPtr<FOceanQuadTreeNode> ChunkNode);

    void RunLodUpdateTask();
};