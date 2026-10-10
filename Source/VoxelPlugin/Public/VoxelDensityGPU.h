// VoxelDensityGPU.h - The planet density on the GPU: the data the compute shader needs
// (filled by FVoxelDensityCPU::BuildGPUData) and an asynchronous sampling call used to check
// the GPU against the CPU. The shader is VoxelDensityGPU.usf (VoxelDensitySample.ush).

#pragma once

#include "CoreMinimal.h"

class FSparseEditStore;
struct FVoxelLandBake;

/** A double as the shaders take it (VoxelDF64.ush): the nearest float and the remainder. */
inline FVector2f VoxelDoubleFloat(double V)
{
    const float Hi = (float)V;
    return FVector2f(Hi, (float)(V - (double)Hi));
}

/** One noise octave as the shader reads it (FVoxelOctave in VoxelDensityCore.ush). Double
 *  values are double-float pairs (high float, remainder). */
struct FVoxelDensityGPUOctave
{
    FVector2f Frequency = FVector2f::ZeroVector;
    FVector2f OffsetX = FVector2f::ZeroVector;
    FVector2f OffsetY = FVector2f::ZeroVector;
    FVector2f OffsetZ = FVector2f::ZeroVector;
    float Wavelength = 0.0f;
    float Amplitude = 0.0f;
    uint32 Seed = 0;
    uint32 Shift = 0xffffffffu;      // fixed-point path (VoxelNoFixedPoint when off)
    uint32 OffsetCell[3] = {};       // the offset in 32.32 fixed point
    uint32 OffsetFrac[3] = {};
    uint32 Pad[2] = {};
};
static_assert(sizeof(FVoxelDensityGPUOctave) == 80, "Must match FVoxelOctave in VoxelDensityCore.ush");

/** Everything the shader needs to evaluate one planet's density (without edits). */
struct FVoxelDensityGPUData
{
    TArray<FVoxelDensityGPUOctave> Octaves;   // continent, mountain, then detail octaves
    uint32 ContinentOctaves = 0;
    uint32 MountainOctaves = 0;
    uint32 DetailOctaves = 0;
    FVector2f InvNormContinent = FVector2f::ZeroVector;
    FVector2f InvNormMountain = FVector2f::ZeroVector;
    FVector2f InvNormDetail = FVector2f::ZeroVector;
    FVector2f PlanetRadius = FVector2f::ZeroVector;
    FVector2f NoiseAmplitude = FVector2f::ZeroVector;
    FVector2f ContinentWeight = FVector2f::ZeroVector;
    FVector2f MountainWeight = FVector2f::ZeroVector;
    FVector2f MountainStart = FVector2f::ZeroVector;
    FVector2f MountainBlend = FVector2f::ZeroVector;
    FVector2f DetailWeight = FVector2f::ZeroVector;
    float RidgeGain = 0.0f;
    float SlopeDamping = 0.0f;
    double MinFootprint = 0.0;   // footprints below it are raised to it, as on the CPU

    // The terrestrial generator (VoxelLandSample.ush; LandMode 0 for the noise surface). The
    // height is the bake's (m) plus the gullies (VoxelLandGully.ush) and the detail layer,
    // offset to the height above PlanetRadius; the warp's three layers follow the detail
    // octaves in Octaves.
    uint32 LandMode = 0;
    TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> Land;
    uint32 LandWarpOctaves = 0;          // per warp component
    float LandWarpInvNorm = 0.0f;
    float LandWarpAmplitude = 0.0f;      // cm
    float LandDetailAmplitude = 0.0f;    // cm
    FVector2f LandHeightOffset = FVector2f::ZeroVector;   // cm, double-float
    float LandCellM = 1.0f;              // the bake cell's arc at the face centre (m)
    float LandGullyStrength = 0.0f;      // VoxelLandGully.ush (0: none)
    uint32 LandGullyOctaves = 0;
    float LandGullyFlowGain = 0.0f;
    float LandGullyDetail = 1.0f;
    uint32 LandGullySeed = 0;
};

/** The sparse edit tree flattened for the GPU (VoxelEditSample.ush). */
struct FVoxelEditGPUData
{
    TArray<float> Corners;      // 8 per node
    TArray<int32> Children;     // 8 per node, -1 for none
    TArray<uint32> Flags;       // 1 when the node holds edits
    int32 RootExtent = 0;       // root half-extent in half finest cells (MaxDepth up to 29)

    /** Flattens Store's current tree. Empty (no edits on the GPU) with a warning when the
     *  store is not centred on the actor or its extent is not HalfFinest * 2^MaxDepth. */
    static TSharedRef<FVoxelEditGPUData> Build(const FSparseEditStore* Store, double HalfFinest, int32 MaxDepth);
};

/** What SampleBlocksAsync returns per point. */
enum class EVoxelDensityGPUOutput : uint8
{
    Density,   // the density without edits (FVoxelDensityCPU::Sample)
    Radial,    // |p| - R only: the double-float position path alone
    Edits      // the edit delta only (FSparseEditStore::Sample)
};

/** A box of lattice points: Dim points per axis from Min, Step apart, all sampled at
 *  Footprint (cm). Lattice indices count half finest cells from the root centre, so point
 *  I is at I * HalfFinest in actor space. Results are written x fastest. */
struct FVoxelDensityGPUBlock
{
    FIntVector Min = FIntVector::ZeroValue;
    int32 Step = 1;
    FIntVector Dim = FIntVector::ZeroValue;
    double Footprint = 0.0;

    int32 NumPoints() const { return Dim.X * Dim.Y * Dim.Z; }
};

namespace VoxelDensityGPU
{
    /** Samples every block on the GPU and calls OnDone on the game thread with the densities,
     *  block after block. Results arrive a few frames later (readback); an empty array when
     *  the GPU can't run the shader (no renderer, unsupported feature level). Call from any
     *  thread but the render thread. Edits are used by Output Edits only (null: none). */
    VOXELPLUGIN_API void SampleBlocksAsync(TSharedRef<const FVoxelDensityGPUData> Data, double HalfFinest,
        TArray<FVoxelDensityGPUBlock> Blocks, TFunction<void(TArray<float>&&)> OnDone,
        EVoxelDensityGPUOutput Output = EVoxelDensityGPUOutput::Density, TSharedPtr<const FVoxelEditGPUData> Edits = nullptr);
}
