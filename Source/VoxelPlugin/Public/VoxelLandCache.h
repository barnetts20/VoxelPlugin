// VoxelLandCache.h - The land surface cache (Design/LandSurfaceCache.md): the terrestrial
// surface's height, gradient and fields around the camera, as a clipmap of levels written by
// compute (Shaders/Private/VoxelLandCacheWrite.usf) and sampled by the terrain material's land
// node (VoxelLandCacheSample.ush). Geometry: Shaders/Private/VoxelLandCache.ush.
//
// The terrain actor owns one per terrestrial density: the textures (render targets, kept alive
// by the actor's properties) and the plan (Private/VoxelLandCachePlan.h: the levels' anchors and
// windows, and what to write each frame within a texel budget, voxel.Land.CacheTexelsPerFrame).

#pragma once

#include "CoreMinimal.h"
#include <atomic>

class FVoxelDensityCPU;
struct FVoxelDensityGPUData;
class UMaterialInstanceDynamic;
class UTextureRenderTarget2D;
class UTexture2DDynamic;
namespace VoxelLandCachePlan { class FPlan; struct FJob; }

/** The cache's textures (VoxelLandCache.ush lists their contents). */
struct FVoxelLandCacheTextures
{
    UTextureRenderTarget2D* Height = nullptr;   // R32F atlas
    UTextureRenderTarget2D* Slope = nullptr;    // RGBA16F atlas
    UTextureRenderTarget2D* Fields = nullptr;   // RGBA16F atlas
    UTexture2DDynamic* State = nullptr;         // RGBA32F, VOXEL_LAND_CACHE_STATE_WIDTH x (levels + 1)

    bool IsValid() const { return Height && Slope && Fields && State; }
};

class VOXELPLUGIN_API FVoxelLandCache
{
public:
    /** The material's parameter names (Texture Objects; STAGE_C1_NOTES.md). */
    static const FName HeightTextureName;   // VoxelLandCacheHeight
    static const FName SlopeTextureName;    // VoxelLandCacheSlope
    static const FName FieldsTextureName;   // VoxelLandCacheFields
    static const FName StateTextureName;    // VoxelLandCacheState

    /** Levels for a sea-level radius (cm): the coarsest reaches R tan 70 degrees along the plane
     *  (60 degrees of arc about the camera once a level is 10 degrees off it), at least 1, at
     *  most VOXEL_LAND_CACHE_MAX_LEVELS (22: radii to about 49,000 km). */
    static int32 LevelsFor(double SeaRadius);

    /** Tiles the atlases need for Levels levels: one each, and the spare a re-centring level is
     *  written into. */
    static int32 TilesFor(int32 Levels);

    /** Game thread. The textures for Levels levels (transient objects in Outer); false when the
     *  RHI can't create them. */
    static bool MakeTextures(int32 Levels, UObject* Outer, FVoxelLandCacheTextures& Out);

    FVoxelLandCache();
    ~FVoxelLandCache();
    FVoxelLandCache(const FVoxelLandCache&) = delete;
    FVoxelLandCache& operator=(const FVoxelLandCache&) = delete;

    /** Game thread. Starts the cache for a terrestrial density's land (empty: every level is
     *  written as the camera is first seen). False for the noise surface. */
    bool Initialize(const FVoxelDensityCPU& Density, const FVoxelLandCacheTextures& InTextures);
    void Reset();
    /** Initialized: the state the material reads is uploaded (no level valid until written). */
    bool IsInitialized() const { return GPUData.IsValid(); }
    /** Initialized, and the writes can run (the write shader compiled, the textures' resources):
     *  without, the material is told to evaluate the land itself. */
    bool IsWritable() const;
    int32 GetLevels() const;
    double GetSeaRadius() const { return SeaRadius; }

    /** Game thread, every frame (a second call in a frame does nothing): the primary camera in
     *  the planet's frame (cm, no scale). Moves the levels' windows, re-anchors levels the camera
     *  has left, and writes what that exposes, within the budget; the state the material reads
     *  follows what is written. Waits while the write shader or the textures aren't ready. */
    void Tick(const FVector& CameraPlanet);

    /** Game thread. The textures on a material instance. */
    void SetTextureParameters(UMaterialInstanceDynamic* Material) const;

    /** The levels' windows as texels written, valid and pending (voxel.Debug.LandCache). */
    FString Describe() const;

private:
    /** Enqueues the jobs (the plan's, this frame) and the state after them; false when the
     *  textures have no resources. */
    bool Dispatch(const VoxelLandCachePlan::FJob* Jobs, int32 NumJobs);

    TUniquePtr<VoxelLandCachePlan::FPlan> Plan;
    TSharedPtr<const FVoxelDensityGPUData, ESPMode::ThreadSafe> GPUData;
    double SeaRadius = 0.0;
    FVoxelLandCacheTextures Textures;
    uint64 TexelsWritten = 0;
    uint64 LastTickFrame = MAX_uint64;
    /** A frame's writes that couldn't run on the render thread (the plan had moved on): the
     *  render thread raises FailedGeneration past the generation the frame was planned in; the
     *  game thread then writes every level again (a new generation), and until it has, every
     *  state uploaded shows no level valid (frames planned before it noticed). */
    uint32 Generation = 0;
    TSharedPtr<std::atomic<uint32>, ESPMode::ThreadSafe> FailedGeneration;
};
