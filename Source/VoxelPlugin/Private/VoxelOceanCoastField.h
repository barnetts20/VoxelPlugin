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
// A machine without a GPU (a dedicated server) makes none yet (stage O9j).

#pragma once

#include "CoreMinimal.h"
#include "Async/Future.h"
#include "VoxelOceanCoastCore.h"
#include "VoxelOceanWaves.h"
#include <atomic>

struct FVoxelLandBake;
class FVoxelOceanShoreField;

/** The fine cells whose depths the GPU samples (FVoxelOceanPages::SampleCoastDepths), and what came
 *  back. Shared by the builder (game thread) and the render thread's readbacks. */
struct FVoxelOceanCoastDepthJob
{
    /** 3 ints a tile: face, bake cell i, j (the plan's interior tiles). */
    TArray<int32> Tiles;
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
    /** Rows 19 to 21 of the chart table's constants (VoxelOceanCoast.ush's layout). */
    FVector4f Rows[3];
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
    void Start(const TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe>& InShore, const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& InBake,
        const VoxelOceanCoastCore::FSettings& InSettings);
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
    TSharedPtr<VoxelOceanCoastCore::FPlan, ESPMode::ThreadSafe> Plan;
    TFuture<void> Planned;
    TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe> Job;
    int64 Issued = 0;
    TFuture<TSharedPtr<FVoxelOceanCoastField, ESPMode::ThreadSafe>> Solved;
    TSharedPtr<const FVoxelOceanCoastField, ESPMode::ThreadSafe> Result;
    bool bForThis = false;
};
