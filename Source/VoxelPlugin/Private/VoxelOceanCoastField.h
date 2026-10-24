// VoxelOceanCoastField.h - A planet's coastal travel-time field (stage O9h; Design/OceanWaves.md
// section 23; VoxelOceanCoast.ush has the layout and the sampling, VoxelOceanCoastCore.h the plan,
// the solve and the packing).
//
// MADE AT LOAD, after the shore field, from the terrain itself: the plan picks the bake cells near
// a coast and the finest cells the memory budget allows (on a worker thread); the GPU samples the
// terrain's depth at every fine cell (FVoxelOceanPages::SampleCoastDepths: the ocean pages'
// density, in chunks over a few frames) and reads it back; a worker thread solves and packs the
// texels. They go into the shore field's texture as its slices 6 and up (the actor makes the
// texture again), so the material and the passes need nothing new; the CPU keeps them for the
// queries. Up to a minute or so on a large planet with long coasts (the solve runs on its own
// thread); the waves carry on without it meanwhile.
//
// STAGE W7a: THE WINDOW. By default the field covers only the coasts round the camera
// (VoxelOceanCoastCore.h): a second or so to make, again in the background whenever the camera
// has moved WindowMove from the window's centre. The texture keeps its shape from one window to
// the next (the atlas's slices for the most tiles a window holds), so a new window's texels are
// written into it in place (WriteInto: the index's boxes and the atlas's slices it fills) rather
// than the texture made again. The cover blends from the last window's to the new one's over
// WindowBlendSeconds and a field fades in when it first arrives (WindowRows: rows 21 to 23 of the
// chart table's constants, live; the queries apply the same).
//
// A machine without a GPU (a dedicated server) makes none yet (stage O9j).

#pragma once

#include "CoreMinimal.h"
#include "Async/Future.h"
#include "VoxelOceanCoastCore.h"
#include "VoxelOceanWaves.h"
#include <atomic>

struct FVoxelLandBake;
class FVoxelOceanShoreField;
class UTexture2DArray;

/** The fine cells whose depths the GPU samples (FVoxelOceanPages::SampleCoastDepths), and what came
 *  back. Shared by the builder (game thread) and the render thread's readbacks. */
struct FVoxelOceanCoastDepthJob
{
    /** 3 ints a tile: face, bake cell i, j (the plan's interior tiles). */
    TArray<int32> Tiles;
    /** Stage W7d: 0 the tiles' fine cells; 1 the land bake's padded cells (no tiles: the shore
     *  field's heights from the terrain, VoxelOceanShoreField.cpp). */
    int32 Mode = 0;
    int32 R = 1;
    int32 N = 0;
    double FootprintCm = 0.0;
    int64 Cells = 0;
    /** cm (VoxelOceanPage.usf's CoastDepthCS), written by the render thread chunk by chunk. */
    TArray<float> DepthCm;
    std::atomic<int64> Received{ 0 };
    std::atomic<int32> InFlight{ 0 };
    std::atomic<int32> Failed{ 0 };
};

/** A planet's coastal field: immutable once made; shared by the queries on any thread. The
 *  queries' shore source with it (stage O9i): its shore field's sample with the coast's. */
class FVoxelOceanCoastField : public IVoxelOceanShoreSource
{
public:
    int32 N = 0;
    int32 R = 0;
    /** The coast's slices (the index's 6 and the atlas's) of Side^2 texels each. */
    int32 Slices = 0;
    double RadiusCm = 0.0;
    double CellM = 0.0;
    uint32 BakeKey = 0;
    /** As made (the reach as the plan held it) and as asked for (the builder's: the actor
     *  compares these). */
    VoxelOceanCoastCore::FSettings Settings;
    VoxelOceanCoastCore::FSettings Requested;
    /** Rows 19 to 21 of the chart table's constants (VoxelOceanCoast.ush's layout; row 21's
     *  blend, fade and window from WindowRows). */
    FVector4f Rows[3];
    /** Stage W7a: the window (none: the whole planet's field), its centre and the last window's
     *  (planet frame, unit), the cover's radii (m), when the blend from the last window began and
     *  when the field began to fade in (FPlatformTime's seconds; long past: done), how long each
     *  takes. Set by the builder before the field is shared. */
    bool bWindow = false;
    FVector3d Centre = FVector3d::ZAxisVector;
    FVector3d CentreLast = FVector3d::ZAxisVector;
    double InnerM = 0.0;
    double OuterM = 0.0;
    double BlendStart = -1.0e30;
    double FadeStart = -1.0e30;
    double BlendSeconds = 1.5;
    /** Per face the index's texels this field's tiles take (VoxelOceanCoastCore::IndexBoxes), and
     *  the atlas's slices it fills. */
    int32 IndexBox[6][4] = {};
    int32 UsedAtlasSlices = 0;
    /** A number of its own (the clipmap's static layer is written again for a new one). */
    uint32 Serial = 0;
    double SampleSeconds = 0.0;
    double PackSeconds = 0.0;
    int64 Cells = 0;
    /** Slices Side^2 texels (the solve's own buffer, moved in). */
    std::vector<uint32_t> Texels;
    /** Candidates left out to fit the budget (VoxelOceanCoastCore::FPlan::TrimmedTiles). */
    int32 TrimmedTiles = 0;
    /** The shore field it goes with (its texels sample with these). */
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> Shore;
    int32 Tiles = 0;
    int32 InteriorTiles = 0;
    double AtlasBytes = 0.0;
    double BuildSeconds = 0.0;
    double SolveSeconds = 0.0;

    bool IsValid() const;

    /** Stage W7a: rows 21 to 23 of the chart table's constants at FPlatformTime's Now: (radius m,
     *  blend, fade, window on), (centre, inner m), (the last centre, outer m). */
    void WindowRows(double Now, FVector4f Out[3]) const;
    /** Whether the blend from the last window and the fade in are done. */
    bool IsSettled(double Now) const { return Now >= BlendStart + BlendSeconds && Now >= FadeStart + BlendSeconds; }

    /** Game thread. Writes Made's texels into Texture, which holds an earlier field's of the same
     *  shape (Last: its index's boxes are written again too): the atlas's slices Made fills and
     *  the index's boxes, through a staging texture and a copy each on the render thread. False
     *  if it can't (no resource). */
    static bool WriteInto(UTexture2DArray* Texture, const TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe>& Made, const FVoxelOceanCoastField* Last);

    /** Any thread. The field at a point (planet frame, cm): A, B, their gradients (planet frame,
     *  per metre), the cover. */
    VoxelShader::FVoxelOceanCoastSample SampleCoast(const FVector& PlanetPoint) const;

    /** Any thread. The shore field's sample at a point (planet frame, cm) with the coast's (within
     *  the field's reach, as the shaders read it: VoxelOceanShoreAt). */
    virtual FVoxelOceanShore Sample(const FVector& PlanetPoint) const override;
};

/** Makes a planet's coastal field (game thread; the heavy parts on workers and the GPU). */
class FVoxelOceanCoastBuilder
{
public:
    /** Enqueues the GPU's sampling of Count cells from First; false if it can't yet (no terrain). */
    using FSampler = TFunction<bool(const TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>& Job, int64 First, int32 Count)>;

    FVoxelOceanCoastBuilder();
    ~FVoxelOceanCoastBuilder();

    void Reset();
    /** Starts over (stage W7a: with the settings' window, round CentreDir, planet frame, unit;
     *  Previous the field shown now, which a new window blends from). */
    void Start(const TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe>& InShore, const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& InBake,
        const VoxelOceanCoastCore::FSettings& InSettings, const FVector3d& CentreDir = FVector3d::ZAxisVector,
        const TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe>& Previous = nullptr);
    /** Game thread, every frame while not done. */
    void Tick(const FSampler& Sampler);

    bool IsDone() const { return Phase == EPhase::Idle; }
    bool IsFor(const FVoxelOceanShoreField* InShore, const VoxelOceanCoastCore::FSettings& InSettings) const;
    TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe> GetResult() const { return Result; }

private:
    enum class EPhase : uint8 { Idle, Planning, Sampling, Solving };

    EPhase Phase = EPhase::Idle;
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> Shore;
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake;
    VoxelOceanCoastCore::FSettings Settings;
    double StartSeconds = 0.0;
    double SampleStartSeconds = 0.0;
    FVector3d Centre = FVector3d::ZAxisVector;
    TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe> Previous;
    /** Stage W7b: the coast's cells (the plan's candidates over the whole planet), kept for the
     *  shore field and the settings they were found for: a window's plan then touches only its
     *  own (it scanned the planet's bake each time, a quarter second at N 1024). Not reset. */
    TSharedPtr<const VoxelOceanCoastCore::FCoastCells, ESPMode::ThreadSafe> KeptCells;
    TWeakPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> KeptShore;
    VoxelOceanCoastCore::FSettings KeptSettings;
    TSharedPtr<VoxelOceanCoastCore::FCoastCells, ESPMode::ThreadSafe> MadeCells;
    TSharedPtr<VoxelOceanCoastCore::FPlan, ESPMode::ThreadSafe> Plan;
    TFuture<void> Planned;
    TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe> Job;
    int64 Issued = 0;
    TFuture<TSharedPtr<FVoxelOceanCoastField, ESPMode::ThreadSafe>> Solved;
    TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe> Result;
    bool bForThis = false;
};
