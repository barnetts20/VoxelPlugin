// VoxelLandGPU.h - The land bake on the GPU for sampling (VoxelLandSample.ush): its buffers,
// uploaded once per bake on the render thread, and the shader parameters every pass that
// evaluates the density binds (the density check, the brick mesher, the ocean pages). Private
// to the module: it pulls in the render graph headers.

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphResources.h"
#include "ShaderParameterMacros.h"

struct FVoxelDensityGPUData;

/** The GPU copy of an FVoxelLandBake (render thread only). The heights, the tan table, the
 *  drainage and the landform are the density's; the attributes, the plates and the lake levels are
 *  uploaded only for the land debug views. */
struct FVoxelLandBakeGPU
{
    TRefCountPtr<FRDGPooledBuffer> Height;
    TRefCountPtr<FRDGPooledBuffer> TanTable;
    TRefCountPtr<FRDGPooledBuffer> Attr;     // two cells per word
    TRefCountPtr<FRDGPooledBuffer> Plates;
    TRefCountPtr<FRDGPooledBuffer> Flow;     // two cells per word (f16)
    TRefCountPtr<FRDGPooledBuffer> Form;     // two cells per word (the landform)
    TRefCountPtr<FRDGPooledBuffer> Relief;   // four cells per word (the local relief)
    TRefCountPtr<FRDGPooledBuffer> Province; // four cells per word (the province, T4c)
    TRefCountPtr<FRDGPooledBuffer> Water;
};

/** T5c: the GPU copy of an FVoxelLandClimateField (render thread only): its entries. */
struct FVoxelLandClimateGPU
{
    TRefCountPtr<FRDGPooledBuffer> Entries;
};

/** The GPU copy of an FVoxelLandRiverSet (render thread only): its tiles and data. */
struct FVoxelLandRiverGPU
{
    TRefCountPtr<FRDGPooledBuffer> Tiles;
    TRefCountPtr<FRDGPooledBuffer> Data;
};

/** The GPU copy of an FVoxelLandNoiseSet (render thread only): its chain's words. */
struct FVoxelLandNoiseGPU
{
    TRefCountPtr<FRDGPooledBuffer> Words;
};

/** Declared in the shader by VoxelLandParameters.ush, read by VoxelLandSample.ush and the land
 *  branch of VoxelDensitySample.ush. Included flat in a pass's parameters
 *  (SHADER_PARAMETER_STRUCT_INCLUDE). */
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelLandShaderParameters, )
    SHADER_PARAMETER(uint32, LandMode)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, LandHeight)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float2>, LandTanTable)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandFlow)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandForm)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandRelief)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandProvince)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandRiverTiles)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, LandRiverData)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandNoise)
    SHADER_PARAMETER(uint32, LandN)
    SHADER_PARAMETER(uint32, LandPad)
    SHADER_PARAMETER(uint32, LandHasFlow)
    SHADER_PARAMETER(uint32, LandHasForm)
    SHADER_PARAMETER(float, LandCellM)
    SHADER_PARAMETER(uint32, LandWarpOctaves)
    SHADER_PARAMETER(float, LandWarpInvNorm)
    SHADER_PARAMETER(float, LandWarpAmplitude)
    SHADER_PARAMETER(float, LandDetailAmplitude)
    SHADER_PARAMETER(FVector2f, LandHeightOffset)
    SHADER_PARAMETER(float, LandGullyStrength)
    SHADER_PARAMETER(uint32, LandGullyOctaves)
    SHADER_PARAMETER(float, LandGullyFlowGain)
    SHADER_PARAMETER(float, LandGullyDetail)
    SHADER_PARAMETER(uint32, LandGullySeed)
    SHADER_PARAMETER(float, LandHillRelief)
    SHADER_PARAMETER(FVector2f, LandHillFrequency)
    SHADER_PARAMETER(uint32, LandHillOctaves)
    SHADER_PARAMETER(uint32, LandHillSeed)
    SHADER_PARAMETER(float, LandPeakStrength)
    SHADER_PARAMETER(float, LandPeakTexelsPerCell)
    SHADER_PARAMETER(uint32, LandPeakSeed)
    SHADER_PARAMETER(float, LandPeakSpacingM)
    SHADER_PARAMETER(float, LandPeakReliefM)
    SHADER_PARAMETER(float, LandPeakReliefStart)
    SHADER_PARAMETER(float, LandPeakReliefEnd)
    SHADER_PARAMETER(float, LandPeakMeanR)
    SHADER_PARAMETER(float, LandPeakMeanG)
    SHADER_PARAMETER(float, LandPeakScaleR)
    SHADER_PARAMETER(float, LandPeakScaleG)
    SHADER_PARAMETER(float, LandPeakScaleB)
    SHADER_PARAMETER(uint32, LandNoiseN)
    SHADER_PARAMETER(uint32, LandNoiseMips)
    SHADER_PARAMETER(uint32, LandDetailBase)
    SHADER_PARAMETER(uint32, LandDetailN)
    SHADER_PARAMETER(uint32, LandDetailMips)
    SHADER_PARAMETER(float, LandPeakDetailTexelsPerCell)
    SHADER_PARAMETER(float, LandDetailMeanHard)
    SHADER_PARAMETER(float, LandDetailMeanSoft)
    SHADER_PARAMETER(float, LandPeakDetailScaleHard)
    SHADER_PARAMETER(float, LandPeakDetailScaleSoft)
    SHADER_PARAMETER(float, LandPeakDetailFade)
    SHADER_PARAMETER(float, LandHillTexelsPerCell)
    SHADER_PARAMETER(uint32, LandHillBase)
    SHADER_PARAMETER(uint32, LandHillN)
    SHADER_PARAMETER(uint32, LandHillMips)
    SHADER_PARAMETER(float, LandHillScaleR)
    SHADER_PARAMETER(float, LandHillScaleG)
    SHADER_PARAMETER(float, LandHillScaleB)
    SHADER_PARAMETER(float, LandHillMeanR)
    SHADER_PARAMETER(float, LandHillMeanG)
    SHADER_PARAMETER(float, LandHillSpacingM)
    SHADER_PARAMETER(float, LandHillSwellTexelsPerCell)
    SHADER_PARAMETER(float, LandHillSwellM)
    SHADER_PARAMETER(float, LandHillSwellScaleG)
    SHADER_PARAMETER(float, LandHillDetailTexelsPerCell)
    SHADER_PARAMETER(float, LandHillDetailScaleHard)
    SHADER_PARAMETER(float, LandHillDetailScaleSoft)
    SHADER_PARAMETER(float, LandHillDetailFade)
    SHADER_PARAMETER(float, LandHillFoothill)
    SHADER_PARAMETER(float, LandStrataTexelsPerCell)
    SHADER_PARAMETER(uint32, LandStrataBase)
    SHADER_PARAMETER(uint32, LandStrataN)
    SHADER_PARAMETER(uint32, LandStrataMips)
    SHADER_PARAMETER(float, LandStrataScaleR)
    SHADER_PARAMETER(float, LandStrataScaleG)
    SHADER_PARAMETER(float, LandStrataScaleB)
    SHADER_PARAMETER(float, LandStrataMeanR)
    SHADER_PARAMETER(float, LandStrataMeanG)
    SHADER_PARAMETER(float, LandStrataSpacingM)
    SHADER_PARAMETER(float, LandStrataRelief)
    SHADER_PARAMETER(float, LandStrataFlat)
    SHADER_PARAMETER(float, LandStrataThreshold)
    SHADER_PARAMETER(float, LandStrataWidth)
    SHADER_PARAMETER(uint32, LandStrataSeed)
    SHADER_PARAMETER(float, LandStrataDetailTexelsPerCell)
    SHADER_PARAMETER(float, LandStrataDetailScaleHard)
    SHADER_PARAMETER(float, LandStrataDetailScaleSoft)
    SHADER_PARAMETER(float, LandStrataDetailFade)
    SHADER_PARAMETER(uint32, LandRiverTilesPerFace)
    SHADER_PARAMETER(float, LandRiverRadiusM)
    SHADER_PARAMETER(float, LandRiverWidthScale)
    SHADER_PARAMETER(float, LandRiverValleyScale)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, LandWater)   // R2: the lakes' levels (the debug views read them too)
    SHADER_PARAMETER(uint32, LandHasWater)
    SHADER_PARAMETER(float, LandRiverCarve)
    SHADER_PARAMETER(float, LandRiverDepthScale)
    SHADER_PARAMETER(float, LandRiverFloodplainScale)
    SHADER_PARAMETER(float, LandRiverMinLog2Area)
    SHADER_PARAMETER(float, LandLakeShoreBand)
    SHADER_PARAMETER(float, LandRiverMeander)
    SHADER_PARAMETER(float, LandRiverFalls)
    SHADER_PARAMETER(float, LandDetailTexelsPerCell)     // N1: the detail layer (VoxelLandDetail.ush)
    SHADER_PARAMETER(uint32, LandDetailScales)
    SHADER_PARAMETER(float, LandDetailMeanA)
    SHADER_PARAMETER(float, LandDetailScaleA)
    SHADER_PARAMETER(float, LandDetailScaleGain)
    SHADER_PARAMETER(float, LandDetailFade)
    SHADER_PARAMETER(uint32, LandDetailSeed)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, LandClimate)   // T5c: the climate map (VoxelLandDunes.ush)
    SHADER_PARAMETER(uint32, LandHasClimate)
    SHADER_PARAMETER(float, LandClimateLapse)
    SHADER_PARAMETER(float, LandDuneTexelsPerCell)
    SHADER_PARAMETER(uint32, LandDuneBase)
    SHADER_PARAMETER(uint32, LandDuneN)
    SHADER_PARAMETER(uint32, LandDuneMips)
    SHADER_PARAMETER(float, LandDuneScaleR)
    SHADER_PARAMETER(float, LandDuneScaleG)
    SHADER_PARAMETER(float, LandDuneMeanR)
    SHADER_PARAMETER(float, LandDuneMeanG)
    SHADER_PARAMETER(float, LandDuneRelief)
    SHADER_PARAMETER(uint32, LandDuneSeed)
    SHADER_PARAMETER(float, LandDuneErgFrequency)
    SHADER_PARAMETER(float, LandDuneErgThreshold)
    SHADER_PARAMETER(float, LandDuneErgWidth)
    SHADER_PARAMETER(float, LandDuneReliefMax)
    SHADER_PARAMETER(float, LandDuneDetailTexelsPerCell)
    SHADER_PARAMETER(uint32, LandDuneDetailBase)
    SHADER_PARAMETER(uint32, LandDuneDetailN)
    SHADER_PARAMETER(uint32, LandDuneDetailMips)
    SHADER_PARAMETER(float, LandDuneDetailScale)
    SHADER_PARAMETER(float, LandDuneDetailMean)
    SHADER_PARAMETER(float, LandDuneDetailFade)
    SHADER_PARAMETER(float, LandGlacialTexelsPerCell)   // T5d: the glacial layer (VoxelLandPeaks.ush)
    SHADER_PARAMETER(uint32, LandGlacialBase)
    SHADER_PARAMETER(uint32, LandGlacialN)
    SHADER_PARAMETER(uint32, LandGlacialMips)
    SHADER_PARAMETER(float, LandGlacialScaleR)
    SHADER_PARAMETER(float, LandGlacialScaleG)
    SHADER_PARAMETER(float, LandGlacialScaleB)
    SHADER_PARAMETER(float, LandGlacialMeanR)
    SHADER_PARAMETER(float, LandGlacialMeanG)
    SHADER_PARAMETER(float, LandGlacialReliefM)
    SHADER_PARAMETER(float, LandGlacialStrength)
    SHADER_PARAMETER(uint32, LandGlacialSeed)
    SHADER_PARAMETER(float, LandGlacialSpacingM)
    SHADER_PARAMETER(float, LandGlacialCooling)
    SHADER_PARAMETER(float, LandGlacialDetailScaleHard)
    SHADER_PARAMETER(float, LandGlacialDetailScaleSoft)
    SHADER_PARAMETER(float, LandIceMaxThickness)   // T5d2: the ice sheets (VoxelLandIce.ush)
    SHADER_PARAMETER(float, LandIceBlend)
    SHADER_PARAMETER(float, LandIceSwellTexelsPerCell)
    SHADER_PARAMETER(uint32, LandIceBase)
    SHADER_PARAMETER(uint32, LandIceN)
    SHADER_PARAMETER(uint32, LandIceMips)
    SHADER_PARAMETER(float, LandIceMeanG)
    SHADER_PARAMETER(float, LandIceSwellM)
    SHADER_PARAMETER(uint32, LandIceSeed)
    SHADER_PARAMETER(float, LandIceSwellSpacingM)
    SHADER_PARAMETER(uint32, LandPolarBase)   // T5d3: the polar surfaces (VoxelLandPolar.ush)
    SHADER_PARAMETER(uint32, LandPolarN)
    SHADER_PARAMETER(uint32, LandPolarMips)
    SHADER_PARAMETER(uint32, LandPolarSeed)
    SHADER_PARAMETER(float, LandPolarCrevasseTexelsPerCell)
    SHADER_PARAMETER(float, LandPolarCrevasseM)
    SHADER_PARAMETER(float, LandPolarCrevasseMean)
    SHADER_PARAMETER(float, LandPolarCrevasseFade)
    SHADER_PARAMETER(float, LandPolarSastrugiTexelsPerCell)
    SHADER_PARAMETER(float, LandPolarSastrugiM)
    SHADER_PARAMETER(float, LandPolarSastrugiMean)
    SHADER_PARAMETER(float, LandPolarSastrugiFade)
    SHADER_PARAMETER(float, LandPolarPolygonTexelsPerCell)
    SHADER_PARAMETER(float, LandPolarPolygonM)
    SHADER_PARAMETER(float, LandPolarPolygonMean)
    SHADER_PARAMETER(float, LandPolarPolygonFade)
    SHADER_PARAMETER(float, LandPolarPermafrostT)
    SHADER_PARAMETER(float, LandPolarCrevasseTurn)
    SHADER_PARAMETER(float, LandPolarSastrugiTurn)
    SHADER_PARAMETER(uint32, LandPolarDuneBase)
    SHADER_PARAMETER(uint32, LandPolarDuneN)
    SHADER_PARAMETER(uint32, LandPolarDuneMips)
    SHADER_PARAMETER(float, LandPolarDuneTexelsPerCell)
    SHADER_PARAMETER(float, LandPolarDuneM)
    SHADER_PARAMETER(float, LandPolarDuneMean)
    SHADER_PARAMETER(float, LandPolarDuneFade)
END_SHADER_PARAMETER_STRUCT()

/** The land debug views' inputs (VoxelLandDebug.ush, declared in VoxelBrickMesherGPU.usf):
 *  the view, the bake's attributes, the sea radius, and the plate model's inputs
 *  (VoxelLandPlates.ush). Included in the brick mesher's parameters, with the density's
 *  (FVoxelLandShaderParameters: the drainage and, from R2, the lake levels are there). */
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelLandDebugParameters, )
    SHADER_PARAMETER(uint32, LandDebugMode)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandAttr)
    SHADER_PARAMETER(FVector2f, LandDebugSeaRadius)
    SHADER_PARAMETER(uint32, LandDebugHasHydro)
    SHADER_PARAMETER(uint32, LandDebugHasAttr)      // 1 when LandAttr is the bake's (else a stand-in)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, LandPlates)
    SHADER_PARAMETER(uint32, LandPlateCount)
    SHADER_PARAMETER(uint32, LandSeed)
    SHADER_PARAMETER(float, LandRadiusM)
    SHADER_PARAMETER(float, LandWidthScale)
    SHADER_PARAMETER(float, LandReliefScale)
    SHADER_PARAMETER(float, LandShelfDepth)
    SHADER_PARAMETER(float, LandAbyssDepth)
    SHADER_PARAMETER(float, LandCoastHeight)
    SHADER_PARAMETER(float, LandInteriorHeight)
END_SHADER_PARAMETER_STRUCT()

namespace VoxelLandGPU
{
    /** Render thread. The land parameters of Density: its bake's buffers (uploaded the first
     *  time any pass asks), or one-element stand-ins and LandMode 0 for the noise surface. */
    FVoxelLandShaderParameters GetParameters(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density);

    /** Render thread. The debug views' parameters for Mode (VOXEL_LAND_VIEW_*, 0 none) and the
     *  sea radius (cm; 0: the density's sea level). The bake's attributes and plates are
     *  uploaded the first time a view needs them and kept with the bake; otherwise, or on the
     *  noise surface, one-element stand-ins. */
    FVoxelLandDebugParameters GetDebugParameters(FRDGBuilder& GraphBuilder, const FVoxelDensityGPUData& Density, int32 Mode, double SeaRadius);
}
