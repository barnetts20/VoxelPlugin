// VoxelOceanFoamTexture.h - The foam's texture set on the GPU (stage W3g; Design/OceanWaves.md
// section 32; VoxelOceanFoamTex.ush samples it): a .vxnt foam set (VoxelNoiseTiles.h, CheckFoam)
// read from the plugin's NoiseSets folder, uploaded as one RGBA16 UNORM texture with its mips, and
// its thresholds' table (VoxelOceanFoamTable.h).

#pragma once

#include "CoreMinimal.h"
#include "RHIResources.h"
#include <atomic>

/** The set's GPU resources (created on the render thread; bReady once they exist). */
struct FVoxelOceanFoamTexture
{
    FTextureRHIRef Set;
    FTextureRHIRef Table;
    // Written by the loading task before the upload command that sets bReady.
    uint32 N = 0;
    uint32 Mips = 0;
    FString Name;
    std::atomic<bool> bReady { false };
};

namespace VoxelOceanFoamTexture
{
    /** Starts reading NoiseSets/<Name>.vxnt in the background: checks it's a foam set, builds its
     *  table and has the render thread create the textures (bReady then; a failure is logged).
     *  Game thread. Null (OutError why) when the file isn't there. */
    TSharedPtr<FVoxelOceanFoamTexture, ESPMode::ThreadSafe> Load(const FString& Name, FString& OutError);
}
