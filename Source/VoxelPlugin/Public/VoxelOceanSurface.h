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
//
// Tiles sample the terrain depth when written (FVoxelOceanPages). A tile on screen is written
// again where the depth under it changed: all of them when the density, the lattice or the
// cull settings change, and those in an edit's reach once the terrain's GPU edit data holds
// that edit (the tile keeps its slot; nothing else moves).

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanTiles.h"
#include "VoxelOceanPages.h"
#include "VoxelOceanWaves.h"

class AActor;
class USceneComponent;
class UMaterialInterface;
class UMaterialInstanceDynamic;

/** What the surface samples the terrain with, refreshed every frame (AOceanSphereActor). */
struct FVoxelOceanSurfaceInputs
{
    FVoxelOceanDensityInputs Density;
    uint32 EditSequence = 0;          // the newest edit Density.Edits holds
    double SlopeBound = 3.5;          // the steepest density gradient assumed (the land cull's reach)
    /** How far (cm) the land must stand above the sea, beyond what the slope bound allows for
     *  over a morphed triangle, before the triangle is culled. Negative: no cull. */
    double LandCullMargin = 500.0;
    /** The ocean clock (s) and the environment constants, this frame (no rewrite on change). */
    double Time = 0.0;
    FVoxelOceanEnvironment Environment;
};

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

    /** Game thread. This frame's waves: the planet frame, the clock, the environment and the
     *  wave model (false before Initialize, or without an attach root). Density is left for
     *  the caller. */
    bool GetWaveFrame(FVoxelOceanWaveFrame& Out) const;

    /** Game thread. A new LOD distance factor (clamped to MinLodDistanceFactor); the next
     *  selection follows it. */
    void SetLodDistanceFactor(double InFactor);

    /** Game thread, before Tick. The terrain inputs; tiles on screen are written again when
     *  the density, the lattice or the cull settings change. */
    void SetInputs(const FVoxelOceanSurfaceInputs& InInputs);

    /** Game thread. An edit of the terrain (planet frame): the tiles on screen within its reach
     *  are written again once the inputs' edit data holds it. */
    void NotifyEdit(uint32 Sequence, const FVector& Center, double Radius);

    /** Game thread, every frame. Camera in the planet frame (actor space without scale). */
    void Tick(const FVector& Camera);

    const FVoxelOceanSurfaceSettings& GetSettings() const { return Settings; }

    /** Tiles on screen. */
    int32 NumTiles() const { return Committed.Num(); }

private:
    VoxelOceanTiles::FSelectParams SelectParams() const;
    /** Builds the tiles' vertex words (in parallel when there are many). */
    void BuildWrites(TConstArrayView<FVoxelOceanTileKey> Tiles, TArray<FVoxelOceanPages::FTileWrite>& Out) const;
    /** Writes again the tiles on screen whose depth changed (SetInputs, NotifyEdit). */
    void ApplyRewrites();
    /** Writes the selection camera (world, as a float and the rest) and the morph parameters to
     *  the material instance. */
    void UpdateMaterial();
    void LogPerf(double Now);
    /** The wave model from the console's test settings (voxel.Ocean.TestWaves); a change of
     *  its reach writes the tiles on screen again (the land cull allows for it). */
    void UpdateWaveModel();

    TWeakObjectPtr<AActor> Owner;
    TWeakObjectPtr<USceneComponent> AttachRoot;
    FVoxelOceanSurfaceSettings Settings;
    TUniquePtr<FVoxelOceanPages> Pages;

    FVoxelOceanSurfaceInputs Inputs;
    /** Null: a flat sea. */
    TSharedPtr<const IVoxelOceanWaveModel, ESPMode::ThreadSafe> WaveModel;
    FVoxelOceanWavesTest::FSettings WaveSettings;
    bool bHasInputs = false;
    bool bRewriteAll = false;
    struct FPendingEdit
    {
        uint32 Sequence = 0;
        FVector Center = FVector::ZeroVector;   // planet frame
        double Reach = 0.0;                     // cm: a vertex nearer than this may sample it
    };
    TArray<FPendingEdit> PendingEdits;

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
    bool bReportedShape = false;

    struct FPerf
    {
        double Start = 0.0;
        int32 Frames = 0;
        double SelectMs = 0.0;
        double MaxSelectMs = 0.0;
        double CommitMs = 0.0;
        double MaxCommitMs = 0.0;
        int32 Changes = 0;             // frames whose selection differed from the one on screen
        int32 Rewritten = 0;           // tiles written again for depth changes
        int32 Waits = 0;               // of those, frames waiting for buffer sets
        int32 MaxWait = 0;
        int32 Wait = 0;
        int64 Selected = 0;
    };
    FPerf Perf;
};
