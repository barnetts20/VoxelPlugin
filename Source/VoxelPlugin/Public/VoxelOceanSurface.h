// VoxelOceanSurface.h - The ocean surface of one ocean actor (Design/OceanMeshing.md sections 2
// and 3): every frame the CDLOD selection of the cube-sphere quadtree for the camera
// (VoxelOceanTiles), committed to the GPU pages (FVoxelOceanPages) in one frame when it
// changes, and the camera the committed selection was built for handed to the material for
// the morph (VoxelOcean.ush).
//
// A change waits, with the previous selection and its camera kept on screen, until every page
// it writes to has a buffer set ready; so the morph is always evaluated against the camera of
// the selection on screen, and boundaries between levels stay closed however long a commit
// waits. Only the first fill commits page by page, as pages get ready.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanTiles.h"
#include "VoxelOceanPages.h"

class AActor;
class USceneComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;

/** Inputs of the ocean surface, fixed until the next Initialize (LodDistanceFactor and the
 *  material change live). */
struct FVoxelOceanSurfaceSettings
{
    double Radius = 0.0;                  // sea sphere (cm)
    int32 TileQuads = 16;                 // quads per tile edge: 8, 16 or 32
    int32 MaxLevel = 0;                   // finest tile level
    double LodDistanceFactor = 2.5;       // a node splits within this many tile sizes of the camera
    double HorizonHeight = 1000.0;        // tiles stay drawn over the horizon up to this height (cm)
    int32 SpareBufferSets = 8;            // buffer sets kept ready for commits
    /** The ocean's material instance: receives the selection camera every frame. */
    TWeakObjectPtr<UMaterialInstanceDynamic> Material;
};

class VOXELPLUGIN_API FVoxelOceanSurface
{
public:
    FVoxelOceanSurface();
    ~FVoxelOceanSurface();

    /** Game thread. Drops every tile and starts over. Without a renderer nothing is drawn. */
    void Initialize(AActor* InOwner, USceneComponent* InAttachRoot, const FVoxelOceanSurfaceSettings& InSettings);

    /** Game thread. Destroys every page. */
    void Reset();

    /** Game thread. A new material instance for every page, without rebuilding. */
    void SetMaterial(UMaterialInstanceDynamic* InMaterial);

    /** Game thread. A new LOD distance factor (clamped to MinLodDistanceFactor); the next
     *  selection follows it. */
    void SetLodDistanceFactor(double InFactor);

    /** Game thread, every frame. Camera in the planet frame (actor space without scale). */
    void Tick(const FVector& Camera);

    const FVoxelOceanSurfaceSettings& GetSettings() const { return Settings; }

    /** Tiles on screen. */
    int32 NumTiles() const { return Committed.Num(); }

private:
    VoxelOceanTiles::FSelectParams SelectParams() const;
    /** Builds the tiles' vertex words (in parallel when there are many). */
    void BuildWrites(TConstArrayView<FVoxelOceanTileKey> Tiles, TArray<FVoxelOceanPages::FTileWrite>& Out) const;
    /** Writes the selection camera (world, as a float and the rest) and the morph parameters to
     *  the material instance. */
    void UpdateMaterial();
    void LogPerf(double Now);

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    FVoxelOceanSurfaceSettings Settings;
    TUniquePtr<FVoxelOceanPages> Pages;

    /** Tiles on screen and the nodes their selection split (the hysteresis); the camera they
     *  were selected for (planet frame). */
    TSet<FVoxelOceanTileKey> Committed;
    TSet<FVoxelOceanTileKey> CommittedSplit;
    FVector SelectionCamera = FVector::ZeroVector;
    bool bHasSelectionCamera = false;
    /** Nothing complete on screen yet: commits go page by page as pages get ready. */
    bool bFirstFill = true;
    bool bFlipWinding = false;
    bool bAvailable = false;
    bool bReportedMaterial = false;

    struct FPerf
    {
        double Start = 0.0;
        int32 Frames = 0;
        double SelectMs = 0.0;
        double MaxSelectMs = 0.0;
        double CommitMs = 0.0;
        double MaxCommitMs = 0.0;
        int32 Changes = 0;             // frames whose selection differed from the one on screen
        int32 Waits = 0;               // of those, frames waiting for buffer sets
        int32 MaxWait = 0;
        int32 Wait = 0;
        int64 Selected = 0;
    };
    FPerf Perf;
};
