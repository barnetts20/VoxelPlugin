// VoxelBrickTerrain.h - Terrain rendered from bricks (VoxelBrickMesher) in a level-of-detail
// tree around the camera, built on the CPU. The reference for the GPU path; switched against
// the octree renderer with voxel.Render.Bricks.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "VoxelBrickMesher.h"
#include <atomic>

class AActor;
class USceneComponent;
class UMaterialInterface;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class FDensitySampleCompositor;
struct FMeshStreamData;

/** A brick of the LOD tree: level 0 has the finest cells, each level up doubles the cell size.
 *  Coordinates count bricks of that level from the shared lattice origin. */
struct FVoxelBrickKey
{
    int32 Level = 0;
    FInt64Vector Coord;

    bool operator==(const FVoxelBrickKey& Other) const
    {
        return Level == Other.Level && Coord == Other.Coord;
    }

    friend uint32 GetTypeHash(const FVoxelBrickKey& Key)
    {
        return HashCombine(::GetTypeHash(Key.Level), GetTypeHash(Key.Coord));
    }
};

/** Inputs of the brick terrain, fixed until the next Initialize. */
struct FVoxelBrickTerrainSettings
{
    FVector Origin = FVector::ZeroVector;   // lattice point 0 (-RootExtent on each axis)
    double FinestCell = 100.0;              // level-0 cell size (ActualPrecision)
    int32 CellsPerBrick = 32;
    int32 MaxLevel = 0;                     // level whose single brick covers the whole root
    double LodDistanceFactor = 2.5;         // a brick splits when the camera is closer than this many brick sizes
    double SlopeBound = 3.5;
    double TriplanarUVScale = 0.0001;
    TWeakObjectPtr<UMaterialInterface> Material;
};

/** Brick terrain of one terrain actor.
 *
 *  Every TaskInterval a worker task walks the tree from the root brick: a brick is dropped
 *  when its centre sample proves it free of surface, split while the camera is within
 *  LodDistanceFactor brick sizes, and otherwise wanted. Wanted bricks without a mesh are
 *  meshed nearest first within a sample budget, with skirts over the cracks between levels.
 *  On the game thread a brick that is no longer wanted stays until the bricks replacing it
 *  are on screen, so the swap never opens a hole. */
class VOXELPLUGIN_API FVoxelBrickTerrain : public TSharedFromThis<FVoxelBrickTerrain>
{
public:
    /** Game thread. Drops every brick and starts over. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, TSharedPtr<FDensitySampleCompositor> InDensity,
        const FVoxelBrickTerrainSettings& InSettings);

    /** Game thread. Destroys every brick's component; results of a running task are dropped. */
    void Reset();

    /** Game thread. See FVoxelCollisionBricks::NotifyEdit. */
    void NotifyEdit(uint32 Sequence, const FVector& Center, double Radius);

    /** Game thread, every frame. Camera in actor-local space. */
    void Tick(const FVector& Camera);

    int32 NumBricks() const { return Bricks.Num(); }

private:
    struct FBrick
    {
        TWeakObjectPtr<URealtimeMeshSimple> Mesh;
        TWeakObjectPtr<URealtimeMeshComponent> Component;
        TSharedPtr<FMeshStreamData> Pending;
        /** Set from the update futures (any thread): the first frame some mesh of the current
         *  component can be on screen, 0 until then. */
        TSharedPtr<std::atomic<uint64>> ShownFrame;
        uint64 CreatedFrame = 0;
        bool bHasPending = false;
        bool bApplied = false;        // a mesh (or, empty, nothing) has been applied
        bool bHasSection = false;
        bool bDirty = false;
    };

    struct FBuilt
    {
        FVoxelBrickKey Key;
        TSharedPtr<FMeshStreamData> Stream;   // null when the brick holds no surface
    };

    struct FTaskInput
    {
        uint32 Generation = 0;
        FVector Camera = FVector::ZeroVector;
        TSet<FVoxelBrickKey> Known;
        TArray<FVoxelBrickKey> Rebuild;
        TSharedPtr<FDensitySampleCompositor> Density;
        FVoxelBrickTerrainSettings Settings;
    };

    struct FTaskResult
    {
        uint32 Generation = 0;
        uint32 StartSequence = 0;
        TArray<FVoxelBrickKey> Wanted;
        TArray<FBuilt> Built;
        int64 Samples = 0;
        double Milliseconds = 0.0;
    };

    struct FRecentEdit
    {
        uint32 Sequence;
        FBox Bounds;
    };

    static void RunTask(const TSharedRef<FVoxelBrickTerrain>& Self, FTaskInput Input);

    FVoxelBrickGrid GridFor(int32 Level) const;
    FBox BoundsOf(const FVoxelBrickKey& Key) const;
    bool IsOnScreen(const FVoxelBrickKey& Key) const;
    bool IsCovered(const FVoxelBrickKey& Key) const;

    void DrainResults();
    void ApplyQueued();
    void RemoveReplaced();
    bool EnsureComponent(const FVoxelBrickKey& Key, FBrick& Brick);
    static void DestroyComponent(FBrick& Brick);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    TSharedPtr<FDensitySampleCompositor> Density;
    FVoxelBrickTerrainSettings Settings;

    // Game thread only.
    TMap<FVoxelBrickKey, FBrick> Bricks;
    TSet<FVoxelBrickKey> Wanted;
    TArray<FVoxelBrickKey> ApplyQueue;
    TArray<FRecentEdit> RecentEdits;
    uint32 Generation = 0;
    bool bHasWanted = false;      // a task result has set Wanted since the last Reset
    double LastLaunchTime = 0.0;

    std::atomic<bool> bTaskRunning{ false };
    FCriticalSection ResultCS;
    TArray<FTaskResult> Results;
};
