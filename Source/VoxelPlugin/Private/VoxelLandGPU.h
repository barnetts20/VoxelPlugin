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

/** The GPU copy of an FVoxelLandBake (render thread only). The heights, the tan table and
 *  the drainage are the density's; the attributes, the plates and the lake levels are
 *  uploaded only for the land debug views. */
struct FVoxelLandBakeGPU
{
    TRefCountPtr<FRDGPooledBuffer> Height;
    TRefCountPtr<FRDGPooledBuffer> TanTable;
    TRefCountPtr<FRDGPooledBuffer> Attr;     // two cells per word
    TRefCountPtr<FRDGPooledBuffer> Plates;
    TRefCountPtr<FRDGPooledBuffer> Flow;     // two cells per word (f16)
    TRefCountPtr<FRDGPooledBuffer> Water;
};

/** Declared in the shader by VoxelLandParameters.ush, read by VoxelLandSample.ush and the land
 *  branch of VoxelDensitySample.ush. Included flat in a pass's parameters
 *  (SHADER_PARAMETER_STRUCT_INCLUDE). */
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelLandShaderParameters, )
    SHADER_PARAMETER(uint32, LandMode)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, LandHeight)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float2>, LandTanTable)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandFlow)
    SHADER_PARAMETER(uint32, LandN)
    SHADER_PARAMETER(uint32, LandPad)
    SHADER_PARAMETER(uint32, LandHasFlow)
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
END_SHADER_PARAMETER_STRUCT()

/** The land debug views' inputs (VoxelLandDebug.ush, declared in VoxelBrickMesherGPU.usf):
 *  the view, the bake's attributes, the sea radius, the lake levels, and the plate model's
 *  inputs (VoxelLandPlates.ush). Included in the brick mesher's parameters, with the
 *  density's (FVoxelLandShaderParameters: the drainage is there). */
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelLandDebugParameters, )
    SHADER_PARAMETER(uint32, LandDebugMode)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, LandAttr)
    SHADER_PARAMETER(FVector2f, LandDebugSeaRadius)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, LandWater)
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
