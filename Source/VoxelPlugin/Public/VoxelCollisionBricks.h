// VoxelCollisionBricks.h - Terrain collision built on the CPU from the density, independent of
// render LOD: bricks of the finest lattice around each collision source are dual contoured
// (VoxelBrickMesher) on a worker and cooked through hidden RealtimeMesh components.

#pragma once

#include "CoreMinimal.h"
#include "HAL/CriticalSection.h"
#include "VoxelBrickMesher.h"
#include <atomic>

class AActor;
class USceneComponent;
class UWorld;
class URealtimeMeshSimple;
class URealtimeMeshComponent;
class UPrimitiveComponent;
class FDensitySampleCompositor;
struct FMeshStreamData;

/** Something that needs collision around it (a pawn): actor-local position and radius (cm). */
struct FVoxelCollisionSource
{
    FVector Position = FVector::ZeroVector;
    double Radius = 0.0;
};

/** Collision bricks of one terrain actor.
 *
 *  Each Tick (game thread) applies finished bricks, drops bricks beyond every source's
 *  radius times the hysteresis, and every CollisionUpdateInterval starts one worker task.
 *  The task finds the bricks within the sources' radii that may hold surface (an octree-like
 *  descent with the slope-bound test), meshes the ones not built yet plus those an edit
 *  touched, and builds their streams. A brick keeps its old collision until the rebuilt one
 *  has cooked (RealtimeMesh cooks asynchronously and swaps the body when done).
 *
 *  Collision geometry goes through a hidden component's section with collision enabled: in
 *  the RealtimeMesh source read on 2026-10-07, custom complex geometry is only cooked when
 *  some section also has collision. Components and meshes are transient, so none is saved
 *  with the level or copied from the editor world into PIE. */
class VOXELPLUGIN_API FVoxelCollisionBricks : public TSharedFromThis<FVoxelCollisionBricks>
{
public:
    /** Game thread. Drops every brick and starts over with these inputs. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, TSharedPtr<FDensitySampleCompositor> InDensity,
        const FVoxelBrickGrid& InGrid, double InSlopeBound);

    /** Game thread. Destroys every brick's component; results of a running task are dropped. */
    void Reset();

    /** Game thread. An edit with this sequence number changed the density within Radius of
     *  Center; bricks it reaches are rebuilt. */
    void NotifyEdit(uint32 Sequence, const FVector& Center, double Radius);

    /** Game thread, every frame. */
    void Tick(const TArray<FVoxelCollisionSource>& Sources, double Hysteresis);

    /** Game thread: outlines bricks with collision (green) and bricks waiting for a rebuild (orange). */
    void DrawDebug(UWorld* World, const FTransform& ToWorld) const;

    const FVoxelBrickGrid& GetGrid() const { return Grid; }

    /** Game thread, diagnostics: what a trace hit at LocalPoint (actor-local) – whether
     *  HitComponent is one of these bricks, and the state of the brick holding the point. */
    FString DescribeHit(const UPrimitiveComponent* HitComponent, const FVector& LocalPoint) const;

private:
    struct FBrick
    {
        TWeakObjectPtr<URealtimeMeshSimple> Mesh;
        TWeakObjectPtr<URealtimeMeshComponent> Component;
        TSharedPtr<FMeshStreamData> Pending;   // built, not yet applied (null: empty)
        /** Cooks requested and not yet finished; decremented by the cook futures. */
        TSharedPtr<std::atomic<int32>> CooksInFlight;
        /** Result of the latest finished cook (ERealtimeMeshCollisionUpdateResult), -1 before any. */
        TSharedPtr<std::atomic<int32>> LastCookResult;
        int32 NumApplies = 0;
        int32 NumTriangles = 0;
        bool bSkipped = false;                 // proven empty by the slope-bound test
        bool bHasPending = false;
        bool bHasSection = false;
        bool bDirty = false;
    };

    struct FBuilt
    {
        FInt64Vector Coord;
        TSharedPtr<FMeshStreamData> Stream;    // null when the brick holds no surface
        bool bSkipped = false;
    };

    struct FTaskInput
    {
        uint32 Generation = 0;
        TArray<FVoxelCollisionSource> Sources;
        TSet<FInt64Vector> Known;
        TArray<FInt64Vector> Rebuild;
        TSharedPtr<FDensitySampleCompositor> Density;
        FVoxelBrickGrid Grid;
        double SlopeBound = 3.0;
    };

    struct FTaskResult
    {
        uint32 Generation = 0;
        uint32 StartSequence = 0;
        TArray<FBuilt> Built;
        int32 NumCandidates = 0;
        int32 NumSkipped = 0;
        int64 Samples = 0;
        float MaxGradient = 0.0f;
        double Milliseconds = 0.0;
    };

    struct FRecentEdit
    {
        uint32 Sequence;
        FBox Bounds;
    };

    static void RunTask(const TSharedRef<FVoxelCollisionBricks>& Self, FTaskInput Input);

    void DrainResults();
    void ApplyQueued();
    void RemoveFar(const TArray<FVoxelCollisionSource>& Sources, double Hysteresis);
    void LaunchTask(const TArray<FVoxelCollisionSource>& Sources);
    bool EnsureComponent(const FInt64Vector& Coord, FBrick& Brick);
    static void DestroyComponent(FBrick& Brick);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    TSharedPtr<FDensitySampleCompositor> Density;
    FVoxelBrickGrid Grid;
    double SlopeBound = 3.0;

    // Game thread only.
    TMap<FInt64Vector, FBrick> Bricks;
    TArray<FInt64Vector> ApplyQueue;
    TArray<FRecentEdit> RecentEdits;
    uint32 Generation = 0;
    double LastLaunchTime = 0.0;
    bool bWarnedSlope = false;

    std::atomic<bool> bTaskRunning{ false };
    FCriticalSection ResultCS;
    TArray<FTaskResult> Results;
};
