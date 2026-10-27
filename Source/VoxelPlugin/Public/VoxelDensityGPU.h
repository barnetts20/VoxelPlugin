// VoxelDensityGPU.h - The planet density on the GPU: the data the compute shader needs
// (filled by FVoxelDensityCPU::BuildGPUData) and an asynchronous sampling call used to check
// the GPU against the CPU. The shader is VoxelDensityGPU.usf (VoxelDensitySample.ush).

#pragma once

#include "CoreMinimal.h"

class FSparseEditStore;
struct FVoxelLandBake;
struct FVoxelLandClimateField;
struct FVoxelLandNoiseSet;
struct FVoxelLandRiverSet;

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
    // height is the bake's (m) plus the hills (VoxelLandHills.ush), the peaks (VoxelLandPeaks.ush), the gullies
    // (VoxelLandGully.ush) and the detail layer,
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
    float LandHillRelief = 0.0f;         // VoxelLandHills.ush (0: none)
    FVector2f LandHillFrequency = FVector2f::ZeroVector;   // cycles per cm, double-float
    uint32 LandHillOctaves = 0;
    uint32 LandHillSeed = 0;
    TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> LandNoise;   // the noise pack (null: no peaks, hill set or detail set)
    float LandPeakStrength = 0.0f;       // VoxelLandPeaks.ush (FVoxelDensityCPU's)
    float LandPeakTexelsPerCell = 0.0f;  // 0: none
    uint32 LandPeakSeed = 0;
    float LandPeakSpacingM = 0.0f;
    float LandPeakReliefM = 0.0f;
    float LandPeakReliefStart = 500.0f;   // T4b: the peaks' fade-in over the local relief (m)
    float LandPeakReliefEnd = 1500.0f;
    float LandPeakMeanR = 0.0f;
    float LandPeakMeanG = 0.0f;
    float LandPeakScaleR = 1.0f;
    float LandPeakScaleG = 1.0f;
    float LandPeakScaleB = 1.0f;
    float LandPeakDetailTexelsPerCell = 0.0f;  // the peaks' detail (T3e; the detail set's chain: LandNoise's Detail*)
    float LandDetailMeanHard = 0.0f;
    float LandDetailMeanSoft = 0.0f;
    float LandPeakDetailScaleHard = 0.0f;
    float LandPeakDetailScaleSoft = 0.0f;
    float LandPeakDetailFade = 0.0f;
    // The hills' set (VoxelLandHillSet.ush, T4a; its chain: LandNoise's Hill*). 0 texels: none
    // (the procedural hills, LandHillFrequency and LandHillOctaves).
    float LandHillTexelsPerCell = 0.0f;
    float LandHillScaleR = 1.0f;
    float LandHillScaleG = 1.0f;
    float LandHillScaleB = 1.0f;
    float LandHillMeanR = 0.0f;       // N1b: R's and G's means (unit), for the face blend
    float LandHillMeanG = 0.0f;
    float LandHillSpacingM = 0.0f;
    float LandHillSwellTexelsPerCell = 0.0f;   // 0: no swells
    float LandHillSwellM = 0.0f;
    float LandHillSwellScaleG = 1.0f;
    float LandHillDetailTexelsPerCell = 0.0f;  // 0: no detail
    float LandHillDetailScaleHard = 0.0f;
    float LandHillDetailScaleSoft = 0.0f;
    float LandHillDetailFade = 0.0f;
    float LandHillFoothill = 0.0f;         // T4b: the hills' growth in the foothills
    float LandStrataTexelsPerCell = 0.0f;      // T4c: the strata set's placement (0: none)
    float LandStrataScaleR = 1.0f;
    float LandStrataScaleG = 1.0f;
    float LandStrataScaleB = 1.0f;
    float LandStrataMeanR = 0.0f;
    float LandStrataMeanG = 0.0f;
    float LandStrataSpacingM = 0.0f;
    float LandStrataRelief = 0.0f;             // m: the set's relief times the strength, capped
    float LandStrataFlat = 0.0f;               // its share on the flats
    float LandStrataThreshold = 2.0f;          // the province switch (above 1: none)
    float LandStrataWidth = 0.05f;
    uint32 LandStrataSeed = 0;
    float LandStrataDetailTexelsPerCell = 0.0f;   // 0: no detail
    float LandStrataDetailScaleHard = 0.0f;
    float LandStrataDetailScaleSoft = 0.0f;
    float LandStrataDetailFade = 0.0f;
    TSharedPtr<const FVoxelLandRiverSet, ESPMode::ThreadSafe> LandRivers;   // R1: the rivers (null: none)
    uint32 LandRiverTilesPerFace = 0;          // R1: the rivers (0: none)
    float LandRiverRadiusM = 1.0f;
    float LandRiverWidthScale = 1.0f;
    float LandRiverValleyScale = 1.0f;
    // R2: the rivers' and lakes' carve (VoxelLandCarve.ush; FVoxelDensityCPU's).
    float LandRiverCarve = 0.0f;
    float LandRiverDepthScale = 1.0f;
    float LandRiverFloodplainScale = 1.0f;
    float LandRiverMinLog2Area = 0.0f;
    float LandLakeShoreBand = 0.0f;
    float LandRiverMeander = 0.0f;
    float LandRiverFalls = 0.0f;
    uint32 LandHasWater = 0;               // 1: the bake's lake levels are bound (LandWater)
    // N1: the detail layer from the pack's detail set (VoxelLandDetail.ush; FVoxelDensityCPU's).
    float LandDetailTexelsPerCell = 0.0f;
    uint32 LandDetailScales = 0;
    float LandDetailMeanA = 0.0f;
    float LandDetailScaleA = 0.0f;
    float LandDetailScaleGain = 0.0f;
    float LandDetailFade = 0.0f;
    uint32 LandDetailSeed = 0;
    // T5c: the dunes (VoxelLandDunes.ush; FVoxelDensityCPU's; their chains: LandNoise's Dune* and Sand*), where the
    // climate (LandClimate: the land climate's cells and the sand's drift) makes desert.
    TSharedPtr<const FVoxelLandClimateField, ESPMode::ThreadSafe> LandClimate;   // null: none (no dunes)
    float LandClimateLapse = 6.5f;             // C per km
    float LandDuneTexelsPerCell = 0.0f;        // 0: no dunes
    float LandDuneScaleR = 1.0f;
    float LandDuneScaleG = 1.0f;
    float LandDuneMeanR = 0.0f;
    float LandDuneMeanG = 0.0f;
    float LandDuneRelief = 0.0f;               // m
    uint32 LandDuneSeed = 0;
    float LandDuneErgFrequency = 1.0f;
    float LandDuneErgThreshold = 2.0f;         // above 1: no ergs
    float LandDuneErgWidth = 0.06f;
    float LandDuneReliefMax = 400.0f;          // m
    float LandDuneDetailTexelsPerCell = 0.0f;  // 0: no sand detail
    float LandDuneDetailScale = 0.0f;
    float LandDuneDetailMean = 0.0f;
    float LandDuneDetailFade = 0.0f;
    // T5d: the glacial layer (VoxelLandPeaks.ush; its chain: LandNoise's Glacial*), in the peaks'
    // place where the climate (LandClimate) has had glaciers.
    float LandGlacialTexelsPerCell = 0.0f;     // 0: none
    float LandGlacialScaleR = 1.0f;
    float LandGlacialScaleG = 1.0f;
    float LandGlacialScaleB = 1.0f;
    float LandGlacialMeanR = 0.0f;
    float LandGlacialMeanG = 0.0f;
    float LandGlacialReliefM = 0.0f;
    float LandGlacialStrength = 0.0f;
    uint32 LandGlacialSeed = 0;
    float LandGlacialSpacingM = 0.0f;
    float LandGlacialCooling = 0.0f;
    float LandGlacialDetailScaleHard = 0.0f;
    float LandGlacialDetailScaleSoft = 0.0f;
    // T5d2: the ice sheets (VoxelLandIce.ush; their swells' chain: LandNoise's Ice*).
    float LandIceMaxThickness = 0.0f;          // m (0: none)
    bool bLandIceShelf = false;                // T5e: the planet has an ice shelf (the climate bound for its share)
    float LandIceBlend = 0.0f;                 // m
    float LandIceSwellTexelsPerCell = 0.0f;    // 0: no swells
    float LandIceMeanG = 0.0f;
    float LandIceSwellM = 0.0f;
    uint32 LandIceSeed = 0;
    float LandIceSwellSpacingM = 0.0f;
    // T5d3: the polar surfaces (VoxelLandPolar.ush; their chain: LandNoise's Polar*).
    uint32 LandPolarSeed = 0;
    float LandPolarCrevasseTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarCrevasseM = 0.0f;
    float LandPolarCrevasseMean = 0.0f;
    float LandPolarCrevasseFade = 0.0f;
    float LandPolarSastrugiTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarSastrugiM = 0.0f;
    float LandPolarSastrugiMean = 0.0f;
    float LandPolarSastrugiFade = 0.0f;
    float LandPolarPolygonTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarPolygonM = 0.0f;
    float LandPolarPolygonMean = 0.0f;
    float LandPolarPolygonFade = 0.0f;
    float LandPolarPermafrostT = -6.0f;    // C
    float LandPolarCrevasseTurn = 0.0f;    // radians: the set's grain
    float LandPolarSastrugiTurn = 0.0f;
    // The snow dunes: the dune set's G (its chain: LandNoise's Dune*).
    float LandPolarDuneTexelsPerCell = 0.0f;                 // 0: none
    float LandPolarDuneM = 0.0f;
    float LandPolarDuneMean = 0.0f;
    float LandPolarDuneFade = 0.0f;
};

/** The edit store's edits and index for the GPU (VoxelEditSample.ush; the layout is
 *  VoxelEditCore::Flatten's). */
struct FVoxelEditGPUData
{
    TArray<uint32> Cells;       // 4 per slot of the index cells' hash table
    TArray<uint32> OpList;      // op indices per cell, in order
    TArray<uint32> Ops;         // 16 per op
    uint32 TableMask = 0;
    uint32 CellShift = 0;
    uint32 OpCount = 0;         // 0: no edits
    FIntVector CellMin = FIntVector(1);     // the index cells listed (inclusive; empty when none)
    FIntVector CellMax = FIntVector(0);
    double HalfFinest = 0.0;    // the lattice's half cell (cm): lattice point I is at I * HalfFinest

    /** Flattens Store's current edits. Empty (no edits on the GPU) with a warning when the
     *  store is not centred on the actor or its lattice is not the terrain's (HalfFinest). */
    static TSharedRef<FVoxelEditGPUData> Build(const FSparseEditStore* Store, double HalfFinest, int32 MaxDepth);
};

/** What SampleBlocksAsync returns per point. */
enum class EVoxelDensityGPUOutput : uint8
{
    Density,   // the density without edits (FVoxelDensityCPU::Sample)
    Radial,    // |p| - R only: the double-float position path alone
    Edits      // the density with the edits (FSparseEditStore::Apply on FVoxelDensityCPU::Sample)
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
