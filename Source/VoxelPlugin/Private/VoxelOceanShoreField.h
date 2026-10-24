// VoxelOceanShoreField.h - A planet's shore field (Design/OceanWaves.md section 5; stage O8): per
// land bake cell, the signed distance to the coast, the direction toward it and the beach slope
// (VoxelOceanShore.ush has the layout and the sampling).
//
// MADE AT LOAD from the land bake (heights and the sea-connected bit), derived, not cached
// (decision 4): on the GPU (VoxelOceanShoreBuild.usf, 9 passes or so, a few ms) and read back
// once, so the queries sample the very texels the GPU does; on the CPU where there is no GPU (a
// dedicated server; the same shared functions, VoxelOceanShoreCore.h, in parallel: seconds at the
// default resolution).
//
// USED BY:
// - the wave model (the hook samples the texture, the CPU model this copy, the same code): the
//   warp that slows the waves toward the coast, and the beach slope;
// - the ocean pages: a vertex in an inland basin (below sea level, its field distance below minus
//   the margin) is land, its triangles culled;
// - the underwater view: a camera in such a basin isn't under water.
//
// THE TEXTURE: a transient Texture 2D Array, 6 slices of (N + 2 Pad)^2, R32F holding each texel's
// bits (VoxelOceanShore.ush: always a normal float). The material reads it with Load
// (VoxelOceanShore, a Texture Object Parameter).

#pragma once

#include "CoreMinimal.h"
#include "RHIResources.h"
#include "VoxelOceanWaves.h"
#include <atomic>
#include <vector>

struct FVoxelLandBake;
struct FVoxelOceanCoastDepthJob;
class UTexture2DArray;
class FTextureResource;

/** A planet's shore field: immutable once made; shared by the queries on any thread. */
class FVoxelOceanShoreField : public IVoxelOceanShoreSource
{
public:
    int32 N = 0;                 // cells per face edge (the bake's)
    double RadiusCm = 0.0;       // the sea sphere
    double ReachM = 0.0;         // the farthest distance held (m)
    uint32 BakeKey = 0;          // the bake it was made from
    bool bFromGpu = false;
    double BuildSeconds = 0.0;
    /** 6 (N + 2 Pad)^2 texels' bits. */
    TArray<uint32> Texels;
    /** Stage W7d: the heights it was made from (m about sea level, the bake's layout): the
     *  terrain's own at each bake cell (sampled on the GPU) rather than the bake's, whose coasts
     *  the terrain's domain warp moves by kilometres. Empty: the bake's (a server, or the
     *  sampling off). The coastal field plans from these too. */
    TArray<float> Height;
    int32 SeaCells = 0;

    /** Any thread. The field at a point (planet frame): distance (cm), direction (planet frame,
     *  tangent, length 1 or less), slope. */
    virtual FVoxelOceanShore Sample(const FVector& PlanetPoint) const override;

    /** The coast as segments of unit directions (six doubles each; stage O9c, the ocean mesh's
     *  split bias: VoxelOceanShoreCore::Coastline). Any thread; a pass over every texel. */
    std::vector<double> Coastline() const;

    /** Whether a point is sea or within MarginCm of it (not land, not an inland basin). */
    bool IsSea(const FVector& PlanetPoint, double MarginCm) const;

    /** A bake cell's arc at a face centre (cm). */
    double CellCm() const;

    /** The margin (cm) inland basins lie beyond: Cells bake cells. */
    double MarginCm(double Cells) const { return Cells * CellCm(); }

    bool IsValid() const;

    /** Game thread. The texture the material and the passes read (made from Texels; a server
     *  makes none). Extra: slices after the field's 6 (stage O9h: the coastal field's,
     *  VoxelOceanCoast.ush), ExtraSlices of Side^2 texels. */
    UTexture2DArray* MakeTexture(const uint32* Extra = nullptr, int32 ExtraSlices = 0) const;
};

/** The field's texture as the render thread's passes bind it (the ocean pages' cull, the depth
 *  probe, the underwater pass): its RHI texture (set on the render thread once the texture's
 *  resource exists) and the field's size. Shared; its last holder keeps the RHI texture alive. */
struct FVoxelOceanShoreRHI
{
    FTextureRHIRef Texture;      // render thread
    int32 N = 0;
    float ReachM = 0.0f;
    /** Game thread. The RHI reference for a texture (a render command after its resource's
     *  creation). */
    static TSharedPtr<FVoxelOceanShoreRHI, ESPMode::ThreadSafe> Make(UTexture2DArray* Texture, int32 N, double ReachM);
};

/** Makes a shore field from a land bake: on the GPU (read back once), or on the CPU. Game thread:
 *  Start, then Tick once a frame until IsDone. */
class FVoxelOceanShoreBuilder
{
public:
    FVoxelOceanShoreBuilder();
    ~FVoxelOceanShoreBuilder();

    /** Starts over for a bake (the field's reach in m). Without a bake, or one without the
     *  sea-connected bit's inputs, it is done at once with no field. */
    void Start(const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& InBake, double InRadiusCm, double InReachM);

    /** Enqueues the GPU's sampling of Count cells from First (the terrain's depth at the bake's
     *  cells: FVoxelOceanPages::SampleCoastDepths, mode 1); false if it can't yet. */
    using FSampler = TFunction<bool(const TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>& Job, int64 First, int32 Count)>;

    /** Advances: the terrain's heights (stage W7d, with a Sampler), the GPU's readback, or the
     *  CPU's build. */
    void Tick(const FSampler& Sampler = nullptr);

    bool IsDone() const { return bDone; }

    /** The field once done (null: none could be made, or none asked for). */
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> GetResult() const { return Result; }

    /** What it is building for (the bake, the radius, the reach). */
    bool IsFor(const FVoxelLandBake* InBake, double InRadiusCm, double InReachM) const
    {
        return Bake.Get() == InBake && RadiusCm == InRadiusCm && ReachM == InReachM;
    }

    /** Forgets the build (a GPU build still running finishes unread). */
    void Reset();

private:
    struct FJob;
    void StartGpu();
    void StartCpu();
    /** Stage W7d: the heights back: the sea bits from them, then on to the GPU's build. */
    void FromTerrain();
    void Finish(TSharedPtr<FVoxelOceanShoreField, ESPMode::ThreadSafe> Field);

    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Bake;
    double RadiusCm = 0.0;
    double ReachM = 0.0;
    double StartSeconds = 0.0;
    int32 WaitFrames = 0;
    bool bDone = true;
    bool bGpu = false;
    /** Stage W7d: sampling the terrain's heights first, the chunks issued. */
    bool bSampling = false;
    int64 Issued = 0;
    int32 SampleWaitFrames = 0;
    TSharedPtr<FJob, ESPMode::ThreadSafe> Job;
    TSharedPtr<const FVoxelOceanShoreField, ESPMode::ThreadSafe> Result;
};
