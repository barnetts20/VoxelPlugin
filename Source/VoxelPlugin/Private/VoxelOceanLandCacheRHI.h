// VoxelOceanLandCacheRHI.h - The land surface cache as the ocean's depth samples read it (stage
// O9m; Design/LandSurfaceCache.md, Design/OceanWaves.md section 28).
//
// The terrain's cache holds the land's height and gradient around the camera at every scale,
// band-limited to its texels and with the terrain's edits. The ocean's tile writes, its
// clipmap's beds and its depth probe read their depths from it (VoxelOceanPage.usf,
// VoxelOceanCacheDepthAt) instead of evaluating the terrain's density, wherever the cache holds
// the point; elsewhere (not covered, not written yet, or edited into more than one surface
// there) they evaluate the density as before.

#pragma once

#include "CoreMinimal.h"
#include "RHI.h"
#include "UObject/WeakObjectPtr.h"

struct FVoxelLandCacheTextures;

/** The cache's textures as the render thread's passes bind them (resolved on the render thread
 *  once their resources exist; the last holder keeps the RHI textures alive) and the two
 *  spheres the depths are measured between. */
struct FVoxelOceanLandCacheRHI
{
    FTextureRHIRef Height;      // render thread
    FTextureRHIRef Slope;
    FTextureRHIRef Fields;
    FTextureRHIRef State;
    /** The cache's sea-level radius (cm): its heights are about it. */
    double LandSeaRadiusCm = 0.0;
    /** The ocean's sea sphere over the cache's (cm): a depth is this less the height. */
    double SeaOffsetCm = 0.0;
    /** The textures it was made from (game thread: whether it must be made again). */
    TWeakObjectPtr<UObject> Sources[4];

    /** Render thread: every texture resolved. */
    bool IsReady() const { return Height.IsValid() && Slope.IsValid() && Fields.IsValid() && State.IsValid(); }

    /** Game thread. For a cache's textures (null without their resources). */
    static TSharedPtr<FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe> Make(const FVoxelLandCacheTextures& Textures, double LandSeaRadiusCm, double OceanRadiusCm);
    /** Game thread. Made from these textures and spheres. */
    bool IsFor(const FVoxelLandCacheTextures& Textures, double LandSeaRadiusCm, double OceanRadiusCm) const;
};
