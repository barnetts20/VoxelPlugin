#include "VoxelOceanPages.h"
#include "VoxelOceanGPUTime.h"
#include "VoxelPlugin.h"
#include "VoxelGPUCommon.h"
#include "VoxelLandGPU.h"
#include "VoxelBrickTerrain.h"
#include "VoxelBrickMesherGPU.h"
#include "VoxelDensityGPU.h"
#include "VoxelTerrainStats.h"
#include "VoxelOceanShoreField.h"
#include "VoxelOceanDepthProbe.h"
#include "VoxelOceanCoastField.h"
#include "VoxelOceanClipmap.h"
#include "VoxelOceanLandCacheRHI.h"
#include "VoxelLandCache.h"
#include "VoxelOceanRingDraw.h"
#include "CoreGlobals.h"
#include "Engine/Texture2D.h"
#include "Engine/Texture2DArray.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/Texture2DDynamic.h"
#include "RHIGPUReadback.h"
#include "HAL/IConsoleManager.h"
#include "RenderUtils.h"
#include "TextureResource.h"
#include "RealtimeMeshSimple.h"
#include "RealtimeMeshComponent.h"
#include "Compute/RealtimeMeshComputeProvider.h"
#include "GameFramework/Actor.h"
#include "Materials/MaterialInterface.h"
#include "GlobalShader.h"
#include "DataDrivenShaderPlatformInfo.h"
#include "ShaderParameterStruct.h"
#include "RenderGraphBuilder.h"
#include "RenderGraphUtils.h"
#include "RenderingThread.h"
#include "Misc/ScopeExit.h"
#include "Misc/App.h"

DECLARE_CYCLE_STAT(TEXT("Ocean: page flush"), STAT_VoxelOceanFlush, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Ocean: page commit"), STAT_VoxelOceanCommit, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Ocean: create buffer set"), STAT_VoxelOceanCreate, STATGROUP_VoxelTerrain);
DECLARE_CYCLE_STAT(TEXT("Ocean: provider (render thread)"), STAT_VoxelOceanProvider, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Ocean: pages"), STAT_VoxelOceanPages, STATGROUP_VoxelTerrain);
DECLARE_DWORD_ACCUMULATOR_STAT(TEXT("Ocean: pooled buffer sets"), STAT_VoxelOceanPooled, STATGROUP_VoxelTerrain);
DECLARE_GPU_STAT(VoxelOceanPageWrite);
DECLARE_GPU_STAT(VoxelOceanProbe);
DECLARE_GPU_STAT(VoxelOceanDisplace);
DECLARE_GPU_STAT(VoxelOceanClipmap);
DECLARE_GPU_STAT(VoxelOceanRings);

using namespace RealtimeMesh;

// The density (VoxelDensitySample.ush) and the edits (VoxelEditSample.ush), as the terrain's
// FVoxelDensityCS takes them, and the shore field's inland-basin cull (stage O8): what a page's
// vertices and the depth probe sample alike.
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanDensityParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<FVoxelOctave>, Octaves)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditCells)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditOpList)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, EditOps)
    SHADER_PARAMETER(uint32, EditTableMask)
    SHADER_PARAMETER(uint32, EditCellShift)
    SHADER_PARAMETER(uint32, EditOpCount)
    SHADER_PARAMETER(FVector2f, HalfFinest)
    SHADER_PARAMETER(uint32, ContinentOctaves)
    SHADER_PARAMETER(uint32, MountainOctaves)
    SHADER_PARAMETER(uint32, DetailOctaves)
    SHADER_PARAMETER(FVector2f, InvNormContinent)
    SHADER_PARAMETER(FVector2f, InvNormMountain)
    SHADER_PARAMETER(FVector2f, InvNormDetail)
    SHADER_PARAMETER(FVector2f, PlanetRadius)
    SHADER_PARAMETER(FVector2f, NoiseAmplitude)
    SHADER_PARAMETER(FVector2f, ContinentWeight)
    SHADER_PARAMETER(FVector2f, MountainWeight)
    SHADER_PARAMETER(FVector2f, MountainStart)
    SHADER_PARAMETER(FVector2f, MountainBlend)
    SHADER_PARAMETER(FVector2f, DetailWeight)
    SHADER_PARAMETER(float, RidgeGain)
    SHADER_PARAMETER(float, SlopeDamping)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelLandShaderParameters, Land)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ShoreTexture)
    SHADER_PARAMETER(uint32, ShoreN)
    SHADER_PARAMETER(float, ShoreReach)
    SHADER_PARAMETER(float, ShoreMarginM)
    SHADER_PARAMETER(float, DepthFootprint)
    SHADER_PARAMETER(FVector2f, SeaRadius)
    // The land surface cache (stage O9m; VoxelOceanTerrainDepth.ush).
    SHADER_PARAMETER_TEXTURE(Texture2D, LandCacheHeight)
    SHADER_PARAMETER_TEXTURE(Texture2D, LandCacheSlope)
    SHADER_PARAMETER_TEXTURE(Texture2D, LandCacheFields)
    SHADER_PARAMETER_TEXTURE(Texture2D, LandCacheState)
    SHADER_PARAMETER(uint32, LandCacheOn)
    SHADER_PARAMETER(FVector2f, LandCacheSeaRadius)
    SHADER_PARAMETER(float, LandCacheSeaOffset)
END_SHADER_PARAMETER_STRUCT()

BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanWriteParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, Jobs)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, VertexWords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float>, OutPositions)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTangents)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutColors)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutIndices)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, OutArgs)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<uint>, OutBase)
    SHADER_PARAMETER(uint32, BaseOn)
    SHADER_PARAMETER(uint32, NumJobs)
    SHADER_PARAMETER(uint32, TileQuads)
    SHADER_PARAMETER(uint32, DrawSlots)
    SHADER_PARAMETER(uint32, VertexCapacity)
    SHADER_PARAMETER(uint32, TriangleCapacity)
    SHADER_PARAMETER(uint32, Color)
    SHADER_PARAMETER(uint32, StreamMask)
    SHADER_PARAMETER(uint32, FlipWinding)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelOceanDensityParameters, Density)
END_SHADER_PARAMETER_STRUCT()

// The depth probe (VoxelOceanDepthProbe.h): one thread per point.
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanProbeParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, ProbePoints)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, ProbeOut)
    SHADER_PARAMETER(uint32, ProbeCount)
    SHADER_PARAMETER(uint32, ProbeLevelPoints)
    SHADER_PARAMETER(FVector4f, ProbeFootprint)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelOceanDensityParameters, Density)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanWriteCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanWriteCS);
    using FParameters = FVoxelOceanWriteParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanWriteCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        // The density's double-float code needs exact float rounding (as FVoxelDensityCS).
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanWriteCS, "/Plugin/VoxelPlugin/Private/VoxelOceanPage.usf", "WriteCS", SF_Compute);

class FVoxelOceanProbeCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanProbeCS);
    using FParameters = FVoxelOceanProbeParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanProbeCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanProbeCS, "/Plugin/VoxelPlugin/Private/VoxelOceanPage.usf", "ProbeCS", SF_Compute);

// The coastal field's depths (stage O9h, VoxelOceanCoastField.h): one thread per fine cell.
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanCoastDepthParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, CoastTiles)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, CoastOut)
    SHADER_PARAMETER(uint32, CoastFirst)
    SHADER_PARAMETER(uint32, CoastCount)
    SHADER_PARAMETER(uint32, CoastR)
    SHADER_PARAMETER(uint32, CoastN)
    SHADER_PARAMETER(uint32, CoastGroupsX)
    SHADER_PARAMETER(float, CoastFootprint)
    SHADER_PARAMETER(float, CoastLatticeScale)
    SHADER_PARAMETER(uint32, CoastMode)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelOceanDensityParameters, Density)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanCoastDepthCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanCoastDepthCS);
    using FParameters = FVoxelOceanCoastDepthParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanCoastDepthCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanCoastDepthCS, "/Plugin/VoxelPlugin/Private/VoxelOceanPage.usf", "CoastDepthCS", SF_Compute);

// The waves in compute (stage O9j, VoxelOceanDisplace.usf): one thread per drawn vertex.
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanDisplaceParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<uint>, BaseWords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float>, DisplacePositions)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, DisplaceTexCoords)
    SHADER_PARAMETER(uint32, DisplaceVertices)
    SHADER_PARAMETER(FVector3f, PageToCamera)
    SHADER_PARAMETER(FVector4f, PageRotation)
    SHADER_PARAMETER(float, PageUnit)
    SHADER_PARAMETER(float, PageLevel)
    SHADER_PARAMETER(float, MorphEnd0)
    SHADER_PARAMETER(FVector4f, DisplaceParams)
    SHADER_PARAMETER(FVector4f, DisplaceWave0)
    SHADER_PARAMETER(FVector4f, DisplaceWave1)
    SHADER_PARAMETER(FVector4f, DisplaceWave2)
    SHADER_PARAMETER(FVector3f, DisplaceCenter)
    SHADER_PARAMETER(FVector4f, DisplaceShape)
    SHADER_PARAMETER(FVector4f, DisplaceFFT)
    SHADER_PARAMETER(FVector4f, DisplaceRefHigh)
    SHADER_PARAMETER(FVector4f, DisplaceRefLow)
    SHADER_PARAMETER(FVector4f, DisplaceRotation)
    SHADER_PARAMETER(FVector4f, DisplaceTiles)
    SHADER_PARAMETER(FVector4f, DisplaceStates)
    SHADER_PARAMETER(FVector4f, DisplaceSlopeVar)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, DisplaceDisplacement)
    SHADER_PARAMETER_SAMPLER(SamplerState, DisplaceDisplacementSampler)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, DisplaceDerivatives)
    SHADER_PARAMETER_SAMPLER(SamplerState, DisplaceDerivativesSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, DisplaceCharts)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, DisplaceShore)
    SHADER_PARAMETER(uint32, ClipOn)
    SHADER_PARAMETER_ARRAY(FVector4f, ClipLevels, [VoxelOceanClipmap::InfoRows])
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ClipDisplacementLevels)
    SHADER_PARAMETER_SAMPLER(SamplerState, ClipSampler)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanDisplaceCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanDisplaceCS);
    using FParameters = FVoxelOceanDisplaceParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanDisplaceCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanDisplaceCS, "/Plugin/VoxelPlugin/Private/VoxelOceanDisplace.usf", "DisplaceCS", SF_Compute);

/** The displacement's inputs this frame (render thread; FVoxelOceanPages::SetDisplaceFrame), and
 *  the wave clipmap's levels when it is on (stage O9k: FVoxelOceanPages::UpdateClipmap). */
struct FVoxelOceanDisplaceState
{
    FVoxelOceanDisplaceFrame Frame;
    FTextureRHIRef Displacement;
    FTextureRHIRef Derivatives;
    FTextureRHIRef Charts;
    FTextureRHIRef Shore;
    /** Stage W3u: the cascades' foam envelopes (the own pass's foam per pixel). */
    FTextureRHIRef Foam;
    /** Stage W3z: the shortest band's own cascade (the own pass). */
    FTextureRHIRef FineDisplacement;
    FTextureRHIRef FineDerivatives;
    FTextureRHIRef FineFoam;
    bool bClip = false;
    FVector4f ClipRows[VoxelOceanClipmap::InfoRows];
    FTextureRHIRef ClipDisplacement;
    /** The clipmap's normal and info textures (the ocean's own pass shades from them; stage W3). */
    FTextureRHIRef ClipNormal;
    /** Stage W3h: the whitecaps' density (the own pass). */
    FTextureRHIRef ClipWhitecap;
    /** Stage W8j: the summed surface's strain per texel (the own pass's pixel foam; the state's
     *  pooled texture, null before the first clipmap frame). */
    FTextureRHIRef ClipStrain;
    FTextureRHIRef ClipInfo;
    /** Stage W4c: the foam's fine history, its levels and factor (none: 0 levels). */
    FTextureRHIRef ClipFineHistory;
    /** Stage W5c: the clipmap's static layer (the own pass's shore at its pixels). */
    FTextureRHIRef ClipStatic;
    FVector2f ClipFineInfo = FVector2f(0.0f, 0.0f);
    /** The rings this frame (stage W2; UpdateClipmap's command, with the clipmap's levels): per
     *  level (the camera about the square's centre in cells, the cell, unused), (the hole's
     *  centre, the centre on the next level's grid); the frame rows (VoxelOceanRings.usf). */
    bool bRings = false;
    uint32 RingCount = 0;
    FVector4f RingLevel[VoxelOceanClipmap::MaxLevels];
    FIntVector4 RingInts[VoxelOceanClipmap::MaxLevels];
    /** The rings' frame (FVoxelOceanPages::UpdateRings, every frame whether or not the clipmap
     *  ran: the far sphere draws without it): VoxelOceanRings.usf's RingFrame0 to 2 and RingCull. */
    bool bRingsFar = false;
    FVector4f RingFrame0 = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f RingFrame1 = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f RingFrame2 = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    FVector4f RingCull = FVector4f(-1.0f, 0.0f, 0.0f, 0.0f);
    /** The rings' origin (planet frame, cm; stage W3: the own pass places their vertices from it). */
    FVector RingOrigin = FVector::ZeroVector;
};

// The wave clipmap (stages O9k, W1): the static layer's rectangles (VoxelOceanClipStatic.usf) ...
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanClipStaticParameters, )
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, StaticOut)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float>, StaticFoamHistoryOut)
    SHADER_PARAMETER(uint32, StaticFoamHistoryOn)
    SHADER_PARAMETER(uint32, StaticSlot)
    SHADER_PARAMETER(FIntPoint, StaticRectMin)
    SHADER_PARAMETER(FIntPoint, StaticRectSize)
    SHADER_PARAMETER(FIntPoint, StaticOriginMod)
    SHADER_PARAMETER(FVector2f, StaticInWindow)
    SHADER_PARAMETER(float, StaticSpacing)
    SHADER_PARAMETER_ARRAY(FVector4f, StaticFrame, [VoxelOceanClipmap::SharedRows])
    SHADER_PARAMETER(FIntVector, StaticCameraLattice)
    SHADER_PARAMETER(FVector3f, StaticCameraFrac)
    SHADER_PARAMETER(FVector4f, StaticParams)
    SHADER_PARAMETER(FVector4f, StaticWave0)
    SHADER_PARAMETER(FVector4f, StaticWave1)
    SHADER_PARAMETER(FVector4f, StaticWave2)
    SHADER_PARAMETER(FVector3f, StaticCenter)
    SHADER_PARAMETER(FVector4f, StaticShape)
    SHADER_PARAMETER(FVector4f, StaticFFT)
    SHADER_PARAMETER(FVector4f, StaticRefHigh)
    SHADER_PARAMETER(FVector4f, StaticRefLow)
    SHADER_PARAMETER(FVector4f, StaticRotation)
    SHADER_PARAMETER(FVector4f, StaticTiles)
    SHADER_PARAMETER(FVector4f, StaticStates)
    SHADER_PARAMETER(FVector4f, StaticSlopeVar)
    SHADER_PARAMETER_TEXTURE(Texture2D, StaticCharts)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, StaticShore)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelOceanDensityParameters, Density)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanClipStaticCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanClipStaticCS);
    using FParameters = FVoxelOceanClipStaticParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanClipStaticCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanClipStaticCS, "/Plugin/VoxelPlugin/Private/VoxelOceanClipStatic.usf", "ClipStaticCS", SF_Compute);

// ... and every level's waves (VoxelOceanClip.usf, ClipWaveCS).
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanClipWaveParameters, )
    SHADER_PARAMETER_ARRAY(FVector4f, ClipInfoRows, [VoxelOceanClipmap::InfoRows])
    SHADER_PARAMETER_ARRAY(FUintVector4, ClipDue, [VoxelOceanClipmap::MaxLevels / 4])
    SHADER_PARAMETER(uint32, ClipDueCount)
    SHADER_PARAMETER_ARRAY(FVector4f, ClipWindow, [VoxelOceanClipmap::MaxLevels])
    SHADER_PARAMETER_ARRAY(FIntVector4, ClipWindowMod, [VoxelOceanClipmap::MaxLevels])
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float4>, ClipStatic)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, ClipDisplacementOut)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, ClipNormalOut)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, ClipWhitecapOut)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, ClipStrainOut)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, ClipStrain0Out)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float4>, ClipStrain1Out)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float>, ClipFoamHistory)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float4>, ClipWhitecapPrev)
    SHADER_PARAMETER_ARRAY(FVector4f, ClipFoamStep, [VoxelOceanClipmap::MaxLevels])
    SHADER_PARAMETER(uint32, ClipFoamHistoryOn)
    SHADER_PARAMETER(FVector4f, ClipFoamSurf)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2D<float4>, ClipInfoOut)
    SHADER_PARAMETER(FVector4f, ClipParams)
    SHADER_PARAMETER(FVector4f, ClipWave0)
    SHADER_PARAMETER(FVector4f, ClipWave1)
    SHADER_PARAMETER(FVector4f, ClipWave2)
    SHADER_PARAMETER(FVector3f, ClipCenter)
    SHADER_PARAMETER(FVector4f, ClipShape)
    SHADER_PARAMETER(FVector4f, ClipFFT)
    SHADER_PARAMETER(FVector4f, ClipRefHigh)
    SHADER_PARAMETER(FVector4f, ClipRefLow)
    SHADER_PARAMETER(FVector4f, ClipRotation)
    SHADER_PARAMETER(FVector4f, ClipTiles)
    SHADER_PARAMETER(FVector4f, ClipStates)
    SHADER_PARAMETER(FVector4f, ClipSlopeVar)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ClipDisplacement)
    SHADER_PARAMETER_SAMPLER(SamplerState, ClipDisplacementSampler)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ClipDerivatives)
    SHADER_PARAMETER_SAMPLER(SamplerState, ClipDerivativesSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, ClipCharts)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ClipFoam)
    SHADER_PARAMETER_SAMPLER(SamplerState, ClipFoamSampler)
    SHADER_PARAMETER(uint32, ClipFoamOn)
    SHADER_PARAMETER(FVector4f, ClipRing)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanClipWaveCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanClipWaveCS);
    using FParameters = FVoxelOceanClipWaveParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanClipWaveCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanClipWaveCS, "/Plugin/VoxelPlugin/Private/VoxelOceanClip.usf", "ClipWaveCS", SF_Compute);

// Stage W4c: the foam's fine history, one level (VoxelOceanClip.usf, ClipFineFoamCS).
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanClipFineParameters, )
    SHADER_PARAMETER_ARRAY(FVector4f, ClipInfoRows, [VoxelOceanClipmap::InfoRows])
    SHADER_PARAMETER(FVector4f, FineWindow)
    SHADER_PARAMETER(FIntVector4, FineWindowMod)
    SHADER_PARAMETER(FVector4f, FineStep)
    SHADER_PARAMETER(FIntVector4, FineRect0)
    SHADER_PARAMETER(FIntVector4, FineRect1)
    SHADER_PARAMETER(uint32, FineWhole)
    SHADER_PARAMETER(FVector4f, FineSurf)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float4>, ClipStatic)
    SHADER_PARAMETER(FVector4f, ClipRing)
    SHADER_PARAMETER_RDG_TEXTURE_UAV(RWTexture2DArray<float>, FineHistory)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float>, FineCoarseHistory)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float4>, FineCoarseWhitecap)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float4>, FineStrain0)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray<float4>, FineStrain1)
    SHADER_PARAMETER(uint32, FineHeldOn)
    SHADER_PARAMETER(FVector4f, ClipParams)
    SHADER_PARAMETER(FVector4f, ClipWave0)
    SHADER_PARAMETER(FVector4f, ClipWave1)
    SHADER_PARAMETER(FVector4f, ClipWave2)
    SHADER_PARAMETER(FVector3f, ClipCenter)
    SHADER_PARAMETER(FVector4f, ClipShape)
    SHADER_PARAMETER(FVector4f, ClipFFT)
    SHADER_PARAMETER(FVector4f, ClipRefHigh)
    SHADER_PARAMETER(FVector4f, ClipRefLow)
    SHADER_PARAMETER(FVector4f, ClipRotation)
    SHADER_PARAMETER(FVector4f, ClipTiles)
    SHADER_PARAMETER(FVector4f, ClipStates)
    SHADER_PARAMETER(FVector4f, ClipSlopeVar)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ClipDisplacement)
    SHADER_PARAMETER_SAMPLER(SamplerState, ClipDisplacementSampler)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ClipDerivatives)
    SHADER_PARAMETER_SAMPLER(SamplerState, ClipDerivativesSampler)
    SHADER_PARAMETER_TEXTURE(Texture2D, ClipCharts)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, ClipFoam)
    SHADER_PARAMETER_SAMPLER(SamplerState, ClipFoamSampler)
    SHADER_PARAMETER(uint32, ClipFoamOn)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanClipFineFoamCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanClipFineFoamCS);
    using FParameters = FVoxelOceanClipFineParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanClipFineFoamCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanClipFineFoamCS, "/Plugin/VoxelPlugin/Private/VoxelOceanClip.usf", "ClipFineFoamCS", SF_Compute);

// The rings (stage W2): every vertex of the rings and the far sphere, once a frame
// (VoxelOceanRings.usf) ...
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanRingParameters, )
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<float>, RingPositions)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, RingTangents)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, RingTexCoords)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, RingColors)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWBuffer<uint>, RingArgs)
    SHADER_PARAMETER(uint32, RingIndexCount)
    SHADER_PARAMETER(uint32, RingSlots)
    SHADER_PARAMETER(int32, RingHalf)
    SHADER_PARAMETER(float, RingMorphStart)
    SHADER_PARAMETER(float, RingMorphEnd)
    SHADER_PARAMETER(uint32, RingFarCells)
    SHADER_PARAMETER(uint32, RingVertices)
    SHADER_PARAMETER(uint32, RingTexWords)
    SHADER_PARAMETER_ARRAY(FVector4f, RingInfo, [VoxelOceanClipmap::InfoRows])
    SHADER_PARAMETER_ARRAY(FVector4f, RingLevel, [VoxelOceanClipmap::MaxLevels])
    SHADER_PARAMETER_ARRAY(FIntVector4, RingInts, [VoxelOceanClipmap::MaxLevels])
    SHADER_PARAMETER(uint32, RingCount)
    SHADER_PARAMETER(FVector4f, RingFrame0)
    SHADER_PARAMETER(FVector4f, RingFrame1)
    SHADER_PARAMETER(FVector4f, RingFrame2)
    SHADER_PARAMETER(FVector4f, RingCull)
    SHADER_PARAMETER(FVector4f, RingRotation)
    SHADER_PARAMETER_TEXTURE(Texture2DArray, RingDisplacement)
    SHADER_PARAMETER_RDG_TEXTURE(Texture2DArray, RingStatic)
    SHADER_PARAMETER_SAMPLER(SamplerState, RingSampler)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float4>, FarDirections)
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<float>, FarLand)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanRingCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanRingCS);
    using FParameters = FVoxelOceanRingParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanRingCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanRingCS, "/Plugin/VoxelPlugin/Private/VoxelOceanRings.usf", "RingCS", SF_Compute);

// ... and the far sphere's land, once (VoxelOceanPage.usf, FarDepthCS).
BEGIN_SHADER_PARAMETER_STRUCT(FVoxelOceanFarDepthParameters, )
    SHADER_PARAMETER_RDG_BUFFER_SRV(StructuredBuffer<int>, FarLattice)
    SHADER_PARAMETER_RDG_BUFFER_UAV(RWStructuredBuffer<float>, FarLandOut)
    SHADER_PARAMETER(uint32, FarCount)
    SHADER_PARAMETER(float, FarFootprint)
    SHADER_PARAMETER_STRUCT_INCLUDE(FVoxelOceanDensityParameters, Density)
END_SHADER_PARAMETER_STRUCT()

class FVoxelOceanFarDepthCS : public FGlobalShader
{
public:
    DECLARE_GLOBAL_SHADER(FVoxelOceanFarDepthCS);
    using FParameters = FVoxelOceanFarDepthParameters;
    SHADER_USE_PARAMETER_STRUCT(FVoxelOceanFarDepthCS, FGlobalShader);

    static bool ShouldCompilePermutation(const FGlobalShaderPermutationParameters& Parameters)
    {
        return IsFeatureLevelSupported(Parameters.Platform, ERHIFeatureLevel::SM5);
    }

    static void ModifyCompilationEnvironment(const FGlobalShaderPermutationParameters& Parameters, FShaderCompilerEnvironment& OutEnvironment)
    {
        FGlobalShader::ModifyCompilationEnvironment(Parameters, OutEnvironment);
        OutEnvironment.CompilerFlags.Add(CFLAG_NoFastMath);
    }
};

IMPLEMENT_GLOBAL_SHADER(FVoxelOceanFarDepthCS, "/Plugin/VoxelPlugin/Private/VoxelOceanPage.usf", "FarDepthCS", SF_Compute);

/** The wave clipmap's render-thread state: the static layer and the levels' waves as last
 *  evaluated, in their slots (a level not due this frame keeps its own). Lost: a frame's passes
 *  couldn't run (the game thread writes everything again). */
struct FVoxelOceanClipState
{
    TRefCountPtr<IPooledRenderTarget> Static;
    TRefCountPtr<IPooledRenderTarget> DisplacementWork;
    TRefCountPtr<IPooledRenderTarget> NormalWork;
    TRefCountPtr<IPooledRenderTarget> WhitecapWork;
    // Stage W8j: the summed surface's strain (read by the own pass from this texture itself: nothing
    // reads last frame's, so no copy).
    TRefCountPtr<IPooledRenderTarget> StrainWork;
    // Stage W8k: the two long bands' strains apart (the fine foam's held bands; read in the graph).
    TRefCountPtr<IPooledRenderTarget> Strain0Work;
    TRefCountPtr<IPooledRenderTarget> Strain1Work;
    // Stage W3t: the foam's history (R32F a slot; -1 a texel to seed) and each slot's last step's
    // ocean time (s; below 0: never).
    TRefCountPtr<IPooledRenderTarget> FoamHistory;
    double SlotTime[VoxelOceanClipmap::MaxLevels];
    bool bHistoryOn = false;
    // Stage W4c: the foam's fine history (R32F, FineMaxLevels slices of Size FineFactor square; -1 a
    // texel to seed) and each fine level's last step's ocean time (s; below 0: never).
    TRefCountPtr<IPooledRenderTarget> FineHistory;
    double FineTime[VoxelOceanClipmap::FineMaxLevels];
    int32 Size = 0;
    int32 Slots = 0;
    std::atomic<bool> bLost{ false };
    FVoxelOceanClipState()
    {
        for (double& T : SlotTime) T = -1.0;
        for (double& T : FineTime) T = -1.0;
    }
};

/** The wave clipmap's game-thread plan (stage W1): the last frame's levels, and per slot what its
 *  static layer and its waves hold. */
struct FVoxelOceanClipPlan
{
    VoxelOceanClipmap::FFrame Frame;
    bool bHasFrame = false;
    struct FSlot
    {
        // The static layer: written for this plane, level and window, from these inputs.
        bool bStatic = false;
        uint32 Plane = 0;
        int32 K = MIN_int32;
        FIntPoint Origin = FIntPoint(0, 0);
        uint64 Key = 0;
        /** Stage W7a: the coastal field it was written with (a window written into the shore
         *  texture in place: written again whole, the foam's history kept). */
        uint32 CoastSerial = 0;
        // The waves: evaluated for this plane, level and window.
        bool bEvaluated = false;
        uint32 EvalPlane = 0;
        int32 EvalK = MIN_int32;
        FIntPoint EvalOrigin = FIntPoint(0, 0);
    };
    FSlot Slots[VoxelOceanClipmap::MaxLevels];
    uint64 Turn = 0;
    /** Stage W4c: the foam's fine history per level index: the plane, level and window it was last
     *  stepped for (a different plane or level: written whole). */
    struct FFine
    {
        bool bStepped = false;
        uint32 Plane = 0;
        int32 K = MIN_int32;
        FIntPoint Origin = FIntPoint(0, 0);
    };
    FFine Fine[VoxelOceanClipmap::FineMaxLevels];
    uint64 FineTurn = 0;
    /** Edits the static layer writes again (planet frame, cm; their reach). */
    struct FEdit
    {
        FVector Center = FVector::ZeroVector;
        double Reach = 0.0;
    };
    TArray<FEdit> Edits;
    /** Texels the static layer wrote in the last second, for the log. */
    int64 StaticTexels = 0;
    int32 Reanchors = 0;
};

static TAutoConsoleVariable<int32> CVarVoxelOceanClipStagger(
    TEXT("voxel.Ocean.ClipStagger"), 1,
    TEXT("The wave clipmap's coarse levels evaluated every few frames (stage O9k): 0 every level every frame; 1 levels 4 to 7 every second frame and 8 on every fourth (a level that moved, at once)."),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelOceanPageCreates(
    TEXT("voxel.Ocean.PageCreates"), 8,
    TEXT("Ocean page buffer sets created per frame at most (recycled ones don't count)."),
    ECVF_Default);

namespace VoxelOceanPagesDetail
{
    constexpr uint32 JobWords = 6;                 // VOXEL_OCEAN_JOB_WORDS (VoxelOceanPage.usf)
    constexpr uint32 BaseWords = 16;               // VOXEL_OCEAN_BASE_WORDS (VoxelOceanPage.usf, VoxelOceanDisplace.usf)
    constexpr int32 TexCoordChannels = 6;          // morph xy | morph z, depth | target depth, coast codes | grand xy | grand z, grand depth | bed codes, land depth
    constexpr int32 MaxPooled = 48;                // emptied buffer sets kept for reuse
    constexpr uint64 KickFrames = 3;               // frames before a run that didn't happen is asked for again
    constexpr uint64 UnusedFrames = 120;           // an empty page no commit wanted is pooled after
    constexpr uint64 EmptyingTimeoutFrames = 60;   // an emptying run not recorded by then: destroyed instead
    constexpr int32 BytesPerVertex = 12 + 8 + 8 * TexCoordChannels + 4;
    constexpr int32 BytesPerTriangle = 12;

    const FName PageGroupName(TEXT("OCEANPAGE"));

    /** What a page's compute run lacked. Runs missing a Required item are skipped. */
    enum EMissing : uint32
    {
        Shaders = 1 << 0,
        Positions = 1 << 1,
        Tangents = 1 << 2,
        TexCoords = 1 << 3,
        Colors = 1 << 4,
        Triangles = 1 << 5,
        Args = 1 << 6,
        Capacity = 1 << 7,
        Required = Shaders | Positions | Triangles | Args | Capacity
    };

    FString DescribeMissing(uint32 Bits)
    {
        static const TCHAR* Names[] = { TEXT("page shader"), TEXT("Position stream"), TEXT("Tangents stream"), TEXT("TexCoords stream"),
            TEXT("Color stream"), TEXT("Triangles stream"), TEXT("indirect args"), TEXT("capacity") };
        FString Out;
        for (int32 i = 0; i < UE_ARRAY_COUNT(Names); i++)
        {
            if (Bits & (1u << i))
                Out += FString::Printf(TEXT("%s%s"), Out.IsEmpty() ? TEXT("") : TEXT(", "), Names[i]);
        }
        return Out;
    }

    int32 TileVertices(int32 Quads) { return (Quads + 1) * (Quads + 1); }
    int32 TileTriangles(int32 Quads) { return 2 * Quads * Quads; }

    FRealtimeMeshBufferSetKey GroupKey()
    {
        return FRealtimeMeshBufferSetKey::Create(FRealtimeMeshLODKey(0), PageGroupName);
    }

    FRealtimeMeshSectionKey SectionKey()
    {
        return FRealtimeMeshSectionKey::CreateForPolyGroup(GroupKey(), 0);
    }

    bool AreShadersReady()
    {
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        // The displacement's too (stage O9j): a run with the waves in compute writes, then displaces.
        return ShaderMap && ShaderMap->GetShader(&FVoxelOceanWriteCS::GetStaticType(), 0).IsValid()
            && ShaderMap->GetShader(&FVoxelOceanDisplaceCS::GetStaticType(), 0).IsValid();
    }

    uint32 FloatBits(float Value)
    {
        uint32 Bits;
        FMemory::Memcpy(&Bits, &Value, sizeof(Bits));
        return Bits;
    }

    /** As VoxelBrickPageGPU.usf's PackSnorm8: x, y, z and w in the four bytes. */
    uint32 PackSnorm8(const FVector3f& V, float W)
    {
        auto Byte = [](float X) { return (uint32)(FMath::RoundToInt(FMath::Clamp(X, -1.0f, 1.0f) * 127.0f) & 0xff); };
        return Byte(V.X) | (Byte(V.Y) << 8) | (Byte(V.Z) << 16) | (Byte(W) << 24);
    }
}

namespace VOP = VoxelOceanPagesDetail;

namespace VoxelOceanPagesDetail
{
    /** A coastal depth chunk's readback (stage O9h), deleted on the render thread whichever
     *  thread lets it go last (a poll torn down at exit: as the shore builder's). */
    struct FCoastReadback
    {
        TUniquePtr<FRHIGPUBufferReadback> Buffer;
        ~FCoastReadback();
    };

    FCoastReadback::~FCoastReadback()
    {
        if (!Buffer.IsValid()) return;
        FRHIGPUBufferReadback* Orphan = Buffer.Release();
        if (IsInRenderingThread())
        {
            delete Orphan;
            return;
        }
        ENQUEUE_RENDER_COMMAND(VoxelOceanCoastReadbackRelease)([Orphan](FRHICommandListImmediate&) { delete Orphan; });
    }
}

/** The density and edit inputs as graph buffers, uploaded once for every page's runs while
 *  they stay the same (render thread; as the terrain pages' GetInputs). */
class FVoxelOceanInputCache
{
public:
    VoxelBrickMesherGPU::FInputs Get(FRDGBuilder& GraphBuilder, const TSharedPtr<const FVoxelDensityGPUData>& InDensity,
        const TSharedPtr<const FVoxelEditGPUData>& InEdits)
    {
        VoxelBrickMesherGPU::FInputs Inputs;
        if (InDensity != Density || InEdits != Edits || !Octaves.IsValid())
        {
            Inputs = VoxelBrickMesherGPU::UploadInputs(GraphBuilder, *InDensity, *InEdits);
            Octaves = GraphBuilder.ConvertToExternalBuffer(Inputs.Octaves);
            EditCells = GraphBuilder.ConvertToExternalBuffer(Inputs.EditCells);
            EditOpList = GraphBuilder.ConvertToExternalBuffer(Inputs.EditOpList);
            EditOps = GraphBuilder.ConvertToExternalBuffer(Inputs.EditOps);
            Density = InDensity;
            Edits = InEdits;
            return Inputs;
        }
        Inputs.Octaves = GraphBuilder.RegisterExternalBuffer(Octaves);
        Inputs.EditCells = GraphBuilder.RegisterExternalBuffer(EditCells);
        Inputs.EditOpList = GraphBuilder.RegisterExternalBuffer(EditOpList);
        Inputs.EditOps = GraphBuilder.RegisterExternalBuffer(EditOps);
        Inputs.EditTableMask = Edits->TableMask;
        Inputs.EditCellShift = Edits->CellShift;
        Inputs.EditOpCount = Edits->OpCount;
        return Inputs;
    }

private:
    TSharedPtr<const FVoxelDensityGPUData> Density;
    TSharedPtr<const FVoxelEditGPUData> Edits;
    TRefCountPtr<FRDGPooledBuffer> Octaves;
    TRefCountPtr<FRDGPooledBuffer> EditCells;
    TRefCountPtr<FRDGPooledBuffer> EditOpList;
    TRefCountPtr<FRDGPooledBuffer> EditOps;
};

namespace VoxelOceanPagesDetail
{
    /** Render thread. The density, edits and shore cull a page write or the probe binds. */
    void FillDensity(FRDGBuilder& GraphBuilder, FVoxelOceanInputCache& Cache, const TSharedPtr<const FVoxelDensityGPUData>& InDensity,
        const TSharedPtr<const FVoxelEditGPUData>& InEdits, double HalfFinest, const TSharedPtr<const FVoxelOceanShoreRHI, ESPMode::ThreadSafe>& Shore,
        double ShoreMarginCm, double DepthFootprintCm, double SeaRadiusCm, const TSharedPtr<const FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe>& LandCache,
        FVoxelOceanDensityParameters& Out)
    {
        const FVoxelDensityGPUData& Density = *InDensity;
        const VoxelBrickMesherGPU::FInputs Inputs = Cache.Get(GraphBuilder, InDensity, InEdits);
        Out.Octaves = GraphBuilder.CreateSRV(Inputs.Octaves);
        Out.EditCells = GraphBuilder.CreateSRV(Inputs.EditCells);
        Out.EditOpList = GraphBuilder.CreateSRV(Inputs.EditOpList);
        Out.EditOps = GraphBuilder.CreateSRV(Inputs.EditOps);
        Out.EditTableMask = Inputs.EditTableMask;
        Out.EditCellShift = Inputs.EditCellShift;
        Out.EditOpCount = Inputs.EditOpCount;
        Out.HalfFinest = VoxelDoubleFloat(HalfFinest);
        Out.ContinentOctaves = Density.ContinentOctaves;
        Out.MountainOctaves = Density.MountainOctaves;
        Out.DetailOctaves = Density.DetailOctaves;
        Out.InvNormContinent = Density.InvNormContinent;
        Out.InvNormMountain = Density.InvNormMountain;
        Out.InvNormDetail = Density.InvNormDetail;
        Out.PlanetRadius = Density.PlanetRadius;
        Out.NoiseAmplitude = Density.NoiseAmplitude;
        Out.ContinentWeight = Density.ContinentWeight;
        Out.MountainWeight = Density.MountainWeight;
        Out.MountainStart = Density.MountainStart;
        Out.MountainBlend = Density.MountainBlend;
        Out.DetailWeight = Density.DetailWeight;
        Out.RidgeGain = Density.RidgeGain;
        Out.SlopeDamping = Density.SlopeDamping;
        Out.Land = VoxelLandGPU::GetParameters(GraphBuilder, Density);
        // The shore field, or a stand-in and no cull.
        FRHITexture* ShoreTexture = Shore.IsValid() ? Shore->Texture.GetReference() : nullptr;
        Out.ShoreTexture = ShoreTexture ? ShoreTexture : GBlackArrayTexture->TextureRHI.GetReference();
        Out.ShoreN = ShoreTexture ? (uint32)Shore->N : 0u;
        Out.ShoreReach = ShoreTexture ? Shore->ReachM : 0.0f;
        Out.ShoreMarginM = (float)(ShoreMarginCm * 0.01);
        Out.DepthFootprint = (float)FMath::Max(DepthFootprintCm, 0.0);
        // The sea sphere: each lattice point's height over it comes off its depth (stage O9b2).
        Out.SeaRadius = VoxelDoubleFloat(FMath::Max(SeaRadiusCm, 0.0));
        // The land surface cache (stage O9m), or stand-ins and none.
        const bool bCache = LandCache.IsValid() && LandCache->IsReady();
        Out.LandCacheHeight = bCache ? LandCache->Height.GetReference() : GBlackTexture->TextureRHI.GetReference();
        Out.LandCacheSlope = bCache ? LandCache->Slope.GetReference() : GBlackTexture->TextureRHI.GetReference();
        Out.LandCacheFields = bCache ? LandCache->Fields.GetReference() : GBlackTexture->TextureRHI.GetReference();
        Out.LandCacheState = bCache ? LandCache->State.GetReference() : GBlackTexture->TextureRHI.GetReference();
        Out.LandCacheOn = bCache ? 1u : 0u;
        Out.LandCacheSeaRadius = VoxelDoubleFloat(bCache ? LandCache->LandSeaRadiusCm : 0.0);
        Out.LandCacheSeaOffset = bCache ? (float)LandCache->SeaOffsetCm : 0.0f;
    }
}

/** Writes one buffer set's commits when the compute driver runs it (render thread). */
class FVoxelOceanPageProvider final : public IRealtimeMeshComputeProvider
{
public:
    /** A commit's work: VOP::JobWords per job and the written tiles' vertex words. */
    struct FState
    {
        TArray<uint32> Jobs;
        TArray<uint32> Words;
        uint32 DrawSlots = 0;
        uint32 TileQuads = 16;
        uint32 Color = 0xffffffffu;
        uint32 FlipWinding = 0;
        uint64 Version = 0;
        /** What the writes sample the depth from (never null). */
        TSharedPtr<const FVoxelDensityGPUData> Density;
        TSharedPtr<const FVoxelEditGPUData> Edits;
        double HalfFinest = 50.0;
        /** The shore field's inland-basin cull (null: none). */
        TSharedPtr<const FVoxelOceanShoreRHI, ESPMode::ThreadSafe> Shore;
        double ShoreMarginCm = 0.0;
        /** The depth the waves feel: its footprint (cm). The sea sphere (cm). */
        double DepthFootprintCm = 0.0;
        double SeaRadiusCm = 0.0;
        /** The land surface cache the depths read first (stage O9m; null: the density only). */
        TSharedPtr<const FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe> LandCache;
    };

    FVoxelOceanPageProvider(const TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe>& InInputCache,
        const TSharedPtr<FVoxelOceanDisplaceState, ESPMode::ThreadSafe>& InDisplace, bool bInDisplace)
        : InputCache(InInputCache)
        , Displace(InDisplace)
        , bDisplace(bInDisplace)
    {
    }

    /** The page this buffer set is placed on (stage O9j): its origin (planet frame, cm), its unit
     *  (cm per page unit) and level. */
    void SetPlacement_RenderThread(const FVector& InOrigin, double InUnit, int32 InLevel)
    {
        Origin = InOrigin;
        Unit = InUnit;
        Level = InLevel;
    }

    /** States not yet run accumulate: each is written in order, so none is lost. */
    void AddState_RenderThread(FState&& InState)
    {
        States.Add(MoveTemp(InState));
    }

    virtual void BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context) override;

    /** The newest state version recorded into a frame's graph. */
    std::atomic<uint64> ShownVersion{ 0 };
    /** Runs skipped (no shader or a required stream missing), runs recorded, and the OR of
     *  VOP::EMissing bits seen. */
    std::atomic<uint32> SkippedRuns{ 0 };
    std::atomic<uint32> Runs{ 0 };
    std::atomic<uint32> Missing{ 0 };
    /** For voxel.Debug.Ocean, taken and reset by Flush. */
    std::atomic<uint32> Calls{ 0 };
    std::atomic<uint32> IdleCalls{ 0 };
    std::atomic<uint32> PerfRuns{ 0 };
    std::atomic<uint64> RenderMicros{ 0 };

private:
    /** The waves in compute (stage O9j): every frame, the drawn vertices from Base. */
    void AddDisplacePass(FRDGBuilder& GraphBuilder, FRDGBufferRef BaseBuffer, FRDGBufferUAVRef Positions, FRDGBufferUAVRef TexCoords, uint32 VertexCapacity);

    TArray<FState> States;
    TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe> InputCache;
    TSharedPtr<FVoxelOceanDisplaceState, ESPMode::ThreadSafe> Displace;
    const bool bDisplace = false;
    /** Each vertex's static data (VoxelOceanPagesDetail::BaseWords words; stage O9j). */
    TRefCountPtr<FRDGPooledBuffer> Base;
    FVector Origin = FVector::ZeroVector;
    double Unit = 1.0;
    int32 Level = 0;
    /** The slots drawn and the tile size, as the last state set them. */
    uint32 DrawSlots = 0;
    uint32 TileQuads = 16;
};

void FVoxelOceanPageProvider::BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanProvider);
    const uint64 StartCycles = FPlatformTime::Cycles64();
    ON_SCOPE_EXIT
    {
        RenderMicros.fetch_add((uint64)(FPlatformTime::ToSeconds64(FPlatformTime::Cycles64() - StartCycles) * 1.0e6), std::memory_order_relaxed);
    };
    Calls.fetch_add(1, std::memory_order_relaxed);
    const bool bDisplaceNow = bDisplace && Displace.IsValid() && Displace->Frame.bOn && DrawSlots > 0u;
    if (States.Num() == 0 && !bDisplaceNow)
    {
        IdleCalls.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    // Every stream is written as 32-bit words.
    FRDGBufferUAVRef Positions = Context.GetStreamUAV(FRealtimeMeshStreams::Position, PF_R32_FLOAT);
    FRDGBufferUAVRef Tangents = Context.GetStreamUAV(FRealtimeMeshStreams::Tangents, PF_R32_UINT);
    FRDGBufferUAVRef TexCoords = Context.GetStreamUAV(FRealtimeMeshStreams::TexCoords, PF_R32_UINT);
    FRDGBufferUAVRef Colors = Context.GetStreamUAV(FRealtimeMeshStreams::Color, PF_R32_UINT);
    FRDGBufferUAVRef Triangles = Context.GetStreamUAV(FRealtimeMeshStreams::Triangles, PF_R32_UINT);
    FRDGBufferUAVRef Args = Context.GetIndirectArgsUAV();
    uint32 MissingNow = 0;
    MissingNow |= VOP::AreShadersReady() ? 0u : (uint32)VOP::Shaders;
    MissingNow |= Positions ? 0u : (uint32)VOP::Positions;
    MissingNow |= Tangents ? 0u : (uint32)VOP::Tangents;
    MissingNow |= TexCoords ? 0u : (uint32)VOP::TexCoords;
    MissingNow |= Colors ? 0u : (uint32)VOP::Colors;
    MissingNow |= Triangles ? 0u : (uint32)VOP::Triangles;
    MissingNow |= Args ? 0u : (uint32)VOP::Args;
    if (Context.GetVertexCapacity() <= 0 || Context.GetIndexCapacity() <= 0)
        MissingNow |= (uint32)VOP::Capacity;
    Missing.fetch_or(MissingNow, std::memory_order_relaxed);
    if (MissingNow & (uint32)VOP::Required)
    {
        // Kept for the next run (the game thread asks again after VOP::KickFrames).
        SkippedRuns.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    Runs.fetch_add(1, std::memory_order_relaxed);
    PerfRuns.fetch_add(1, std::memory_order_relaxed);

    // Streams the driver doesn't expose are written to a scratch word and left alone.
    const uint32 StreamMask = (Tangents ? 1u : 0u) | (TexCoords ? 2u : 0u) | (Colors ? 4u : 0u);
    const bool bHasTexCoords = TexCoords != nullptr;
    auto Scratch = [&GraphBuilder]()
        {
            return GraphBuilder.CreateUAV(GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 1), TEXT("Voxel.OceanScratch")), PF_R32_UINT);
        };
    Tangents = Tangents ? Tangents : Scratch();
    TexCoords = TexCoords ? TexCoords : Scratch();
    Colors = Colors ? Colors : Scratch();

    const uint32 VertexCapacity = (uint32)FMath::Max(0, Context.GetVertexCapacity());
    const uint32 TriangleCapacity = (uint32)FMath::Max(0, Context.GetIndexCapacity() / 3);

    // The vertices' static data (stage O9j): made once a buffer set draws, cleared (a slot never
    // written reads as zeros: its triangles are degenerate anyway).
    FRDGBufferRef BaseBuffer = nullptr;
    bool bDrawsAny = DrawSlots > 0;
    for (const FState& State : States) bDrawsAny |= State.DrawSlots > 0;
    // Not before a state draws something (spare and pooled sets carry an empty one).
    if (bDisplace && (Base.IsValid() || bDrawsAny))
    {
        const uint32 BaseCount = FMath::Max(1u, VertexCapacity * VOP::BaseWords);
        if (Base.IsValid() && Base->Desc.NumElements != BaseCount) Base.SafeRelease();
        if (Base.IsValid())
        {
            BaseBuffer = GraphBuilder.RegisterExternalBuffer(Base);
        }
        else
        {
            BaseBuffer = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), BaseCount), TEXT("Voxel.OceanBase"));
            AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(BaseBuffer), 0u);
            Base = GraphBuilder.ConvertToExternalBuffer(BaseBuffer);
        }
    }
    FRDGBufferUAVRef BaseUAV = BaseBuffer ? GraphBuilder.CreateUAV(BaseBuffer) : GraphBuilder.CreateUAV(GraphBuilder.CreateBuffer(
        FRDGBufferDesc::CreateStructuredDesc(sizeof(uint32), 1), TEXT("Voxel.OceanBaseScratch")));

    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanPageWrite, "VoxelOceanPage write");
    VoxelOceanGPUTime::FScope OceanGPUTime(GraphBuilder, VoxelOceanGPUTime::Other);
    TShaderMapRef<FVoxelOceanWriteCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
    for (const FState& State : States)
    {
        const uint32 NumJobs = (uint32)State.Jobs.Num() / VOP::JobWords;
        FVoxelOceanWriteParameters* Parameters = GraphBuilder.AllocParameters<FVoxelOceanWriteParameters>();
        VOP::FillDensity(GraphBuilder, *InputCache, State.Density, State.Edits, State.HalfFinest, State.Shore, State.ShoreMarginCm, State.DepthFootprintCm,
            State.SeaRadiusCm, State.LandCache, Parameters->Density);
        Parameters->Jobs = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.OceanJobs"), State.Jobs));
        Parameters->VertexWords = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.OceanVertexWords"), State.Words));
        Parameters->OutPositions = Positions;
        Parameters->OutTangents = Tangents;
        Parameters->OutTexCoords = TexCoords;
        Parameters->OutColors = Colors;
        Parameters->OutIndices = Triangles;
        Parameters->OutArgs = Args;
        Parameters->OutBase = BaseUAV;
        Parameters->BaseOn = BaseBuffer ? 1u : 0u;
        Parameters->NumJobs = NumJobs;
        Parameters->TileQuads = State.TileQuads;
        Parameters->DrawSlots = State.DrawSlots;
        Parameters->VertexCapacity = VertexCapacity;
        Parameters->TriangleCapacity = TriangleCapacity;
        Parameters->Color = State.Color;
        Parameters->StreamMask = StreamMask;
        Parameters->FlipWinding = State.FlipWinding;
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanPage write %u jobs", NumJobs), Shader, Parameters,
            FIntVector((int32)FMath::Max(1u, NumJobs), 1, 1));
        DrawSlots = State.DrawSlots;
        TileQuads = State.TileQuads;
    }
    if (States.Num() > 0)
    {
        ShownVersion.store(States.Last().Version, std::memory_order_release);
        States.Reset();
    }
    OceanGPUTime.End();
    // The waves (stage O9j), after the writes: every drawn vertex from its static data.
    if (BaseBuffer && bHasTexCoords && Displace.IsValid() && Displace->Frame.bOn && DrawSlots > 0)
        AddDisplacePass(GraphBuilder, BaseBuffer, Positions, TexCoords, VertexCapacity);
}

void FVoxelOceanPageProvider::AddDisplacePass(FRDGBuilder& GraphBuilder, FRDGBufferRef BaseBuffer, FRDGBufferUAVRef Positions, FRDGBufferUAVRef TexCoords,
    uint32 VertexCapacity)
{
    const FVoxelOceanDisplaceState& D = *Displace;
    const FVoxelOceanDisplaceFrame& F = D.Frame;
    // (Ready: VOP::AreShadersReady checked it for this run.)
    TShaderMapRef<FVoxelOceanDisplaceCS> Shader(GetGlobalShaderMap(GMaxRHIFeatureLevel));
    const uint32 Vertices = FMath::Min(DrawSlots * (uint32)VOP::TileVertices((int32)TileQuads), VertexCapacity);
    if (Vertices == 0) return;
    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanDisplace, "VoxelOceanDisplace");
    VoxelOceanGPUTime::FScope OceanGPUTime(GraphBuilder, VoxelOceanGPUTime::Mesh);
    FVoxelOceanDisplaceParameters* P = GraphBuilder.AllocParameters<FVoxelOceanDisplaceParameters>();
    P->BaseWords = GraphBuilder.CreateSRV(BaseBuffer);
    P->DisplacePositions = Positions;
    P->DisplaceTexCoords = TexCoords;
    P->DisplaceVertices = Vertices;
    // The page from the selection camera, in double (the component's transform: the planet's
    // rotation, the page's origin and unit).
    P->PageToCamera = FVector3f(F.PlanetToWorld.TransformPosition(Origin) - F.CameraWorld);
    const FQuat Q = F.PlanetToWorld.GetRotation();
    P->PageRotation = FVector4f((float)Q.X, (float)Q.Y, (float)Q.Z, (float)Q.W);
    P->PageUnit = (float)Unit;
    P->PageLevel = (float)Level;
    P->MorphEnd0 = (float)F.MorphEnd0;
    P->DisplaceParams = F.Params;
    P->DisplaceWave0 = F.Wave[0];
    P->DisplaceWave1 = F.Wave[1];
    P->DisplaceWave2 = F.Wave[2];
    P->DisplaceCenter = F.Center;
    P->DisplaceShape = F.Shape;
    const bool bFFT = F.FFT.X > 0.5f && D.Displacement && D.Derivatives && D.Charts;
    P->DisplaceFFT = bFFT ? F.FFT : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    P->DisplaceRefHigh = F.RefHigh;
    P->DisplaceRefLow = F.RefLow;
    P->DisplaceRotation = F.Rotation;
    P->DisplaceTiles = F.Tiles;
    FVector4f DisplaceStates = F.States;
    if (!D.Shore) DisplaceStates.X = 0.0f;
    P->DisplaceStates = DisplaceStates;
    P->DisplaceSlopeVar = F.SlopeVar;
    P->DisplaceDisplacement = bFFT ? D.Displacement.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
    P->DisplaceDerivatives = bFFT ? D.Derivatives.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
    P->DisplaceCharts = D.Charts ? D.Charts.GetReference() : GBlackTexture->TextureRHI.GetReference();
    P->DisplaceShore = D.Shore ? D.Shore.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
    // As the material's texture objects sample them (the underwater pass's too).
    P->DisplaceDisplacementSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    P->DisplaceDerivativesSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    // The wave clipmap (stage O9k): the vertices sample its levels instead.
    const bool bClip = D.bClip && D.ClipDisplacement.IsValid();
    P->ClipOn = bClip ? 1u : 0u;
    for (int32 i = 0; i < VoxelOceanClipmap::InfoRows; i++)
        P->ClipLevels[i] = bClip ? D.ClipRows[i] : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    P->ClipDisplacementLevels = bClip ? D.ClipDisplacement.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
    // The levels are stored toroidally (stage W1): the sampler wraps.
    P->ClipSampler = TStaticSamplerState<SF_Bilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanDisplace %u vertices", Vertices), Shader, P,
        FIntVector((int32)FMath::DivideAndRoundUp(Vertices, 64u), 1, 1));
}

// ---------------------------------------------------------------------------------------------
// The rings (stage W2; VoxelOceanRingLayout.ush, VoxelOceanRings.usf)
// ---------------------------------------------------------------------------------------------

namespace VoxelOceanPagesDetail
{
    int32 RingHalf(int32 Size) { return VoxelOceanClipmap::RingHalf(Size); }

    int32 RingSide(int32 Half) { return 2 * Half + 1; }

    /** The far sphere's face axes: N outward, U x V = N (so its cells wind as the rings'). */
    void FarAxes(int32 Face, FVector& N, FVector& U, FVector& V)
    {
        const int32 Axis = Face >> 1;
        const bool bNegative = (Face & 1) != 0;
        N = FVector::ZeroVector;
        U = FVector::ZeroVector;
        V = FVector::ZeroVector;
        N[Axis] = bNegative ? -1.0 : 1.0;
        U[bNegative ? (Axis + 2) % 3 : (Axis + 1) % 3] = 1.0;
        V[bNegative ? (Axis + 1) % 3 : (Axis + 2) % 3] = 1.0;
    }

    /** The far sphere's vertex directions (planet frame, unit): face by face, (Cells + 1)^2 each
     *  (equal-angle; a face's edge exactly the neighbour's, so the faces meet). */
    void FarDirections(int32 Cells, TArray<FVector>& Out)
    {
        const int32 Side = Cells + 1;
        Out.SetNum(6 * Side * Side);
        auto Tan = [Cells](int32 I) { return I == 0 ? -1.0 : (I == Cells ? 1.0 : FMath::Tan((-1.0 + 2.0 * I / Cells) * UE_DOUBLE_PI * 0.25)); };
        for (int32 Face = 0; Face < 6; Face++)
        {
            FVector N, U, V;
            FarAxes(Face, N, U, V);
            for (int32 J = 0; J < Side; J++)
                for (int32 I = 0; I < Side; I++)
                    Out[(Face * Side + J) * Side + I] = (N + U * Tan(I) + V * Tan(J)).GetSafeNormal();
        }
    }

    /** A cell's two triangles (lower-left vertex V00, the next along x V10, along y V01): the
     *  diagonal toward the centre in every quadrant (so every cell of a ring's hole has a vertex
     *  strictly inside it: VoxelOceanRingLayout.ush), wound as the tiles (QuadTriangles: (B - A) x
     *  (C - A) into the sphere); bFlip reverses them. */
    void CellTriangles(uint32 V00, uint32 V10, uint32 V01, uint32 V11, bool bToward11, bool bFlip, TArray<uint32>& Out)
    {
        uint32 T[6];
        if (bToward11)
        {
            T[0] = V00; T[1] = V11; T[2] = V10;
            T[3] = V00; T[4] = V01; T[5] = V11;
        }
        else
        {
            T[0] = V00; T[1] = V01; T[2] = V10;
            T[3] = V10; T[4] = V01; T[5] = V11;
        }
        if (bFlip)
        {
            Swap(T[1], T[2]);
            Swap(T[4], T[5]);
        }
        Out.Append(T, 6);
    }
}

/** Writes the rings' vertices every frame (render thread; stage W2): into the mesh's streams as its
 *  compute provider, or (stage W3, the ocean's own pass) into buffers of its own, which the pass
 *  draws (IVoxelOceanRingDraw). */
class FVoxelOceanRingProvider final : public IRealtimeMeshComputeProvider, public IVoxelOceanRingDraw
{
public:
    /** The far sphere's land to sample (again): what the depths read, the vertices' lattice points. */
    struct FFarJob
    {
        TSharedPtr<const FVoxelDensityGPUData> Density;
        TSharedPtr<const FVoxelEditGPUData> Edits;
        double HalfFinest = 50.0;
        TSharedPtr<const FVoxelOceanShoreRHI, ESPMode::ThreadSafe> Shore;
        double ShoreMarginCm = 0.0;
        double DepthFootprintCm = 0.0;
        double SeaRadiusCm = 0.0;
        TSharedPtr<const FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe> LandCache;
        TArray<int32> Lattice;
        float FootprintCm = 0.0f;
    };

    /** InOwnIndices: the triangles for the own pass (empty: the mesh draws them). */
    FVoxelOceanRingProvider(const TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe>& InInputCache,
        const TSharedPtr<FVoxelOceanDisplaceState, ESPMode::ThreadSafe>& InDisplace, const TSharedPtr<FVoxelOceanClipState, ESPMode::ThreadSafe>& InClip,
        int32 InHalf, int32 InSlots, int32 InFarCells, TArray<FVector4f>&& InFarDirections, uint32 InIndexCount, TArray<uint32>&& InOwnIndices)
        : InputCache(InInputCache)
        , Displace(InDisplace)
        , Clip(InClip)
        , Half(InHalf)
        , Slots(InSlots)
        , FarCells(InFarCells)
        , FarDirectionWords(MoveTemp(InFarDirections))
        , IndexCount(InIndexCount)
        , OwnIndices(MoveTemp(InOwnIndices))
    {
    }

    void SetFarJob_RenderThread(FFarJob&& InJob)
    {
        FarJob = MoveTemp(InJob);
        bFarJob = true;
    }

    virtual void BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context) override;

    virtual bool PrepareDraw_RenderThread(FRDGBuilder& GraphBuilder, FVoxelOceanRingDrawFrame& Out) override;

    std::atomic<uint32> Runs{ 0 };
    std::atomic<uint32> SkippedRuns{ 0 };
    std::atomic<uint32> DrawnRuns{ 0 };

private:
    /** Every vertex this frame into the given views (the far sphere's land first when asked);
     *  TexWords texture coordinate words a vertex. False: a shader is missing (nothing written). */
    bool Write(FRDGBuilder& GraphBuilder, FRDGBufferUAVRef Positions, FRDGBufferUAVRef Tangents, FRDGBufferUAVRef TexCoords, FRDGBufferUAVRef Colors,
        FRDGBufferUAVRef Args, uint32 TexWords);
    uint32 VertexCount() const { return (uint32)(Slots * VoxelOceanPagesDetail::RingSide(Half) * VoxelOceanPagesDetail::RingSide(Half)) + (uint32)FarDirectionWords.Num(); }

    TSharedPtr<FVoxelOceanInputCache, ESPMode::ThreadSafe> InputCache;
    TSharedPtr<FVoxelOceanDisplaceState, ESPMode::ThreadSafe> Displace;
    TSharedPtr<FVoxelOceanClipState, ESPMode::ThreadSafe> Clip;
    const int32 Half;
    const int32 Slots;
    const int32 FarCells;
    TArray<FVector4f> FarDirectionWords;
    const uint32 IndexCount;
    TRefCountPtr<FRDGPooledBuffer> FarDirectionBuffer;
    TRefCountPtr<FRDGPooledBuffer> FarLandBuffer;
    FFarJob FarJob;
    bool bFarJob = false;
    // The own pass's buffers (stage W3), the triangles until uploaded, and the frame last written.
    TArray<uint32> OwnIndices;
    TRefCountPtr<FRDGPooledBuffer> OwnPositions;
    TRefCountPtr<FRDGPooledBuffer> OwnTangents;
    TRefCountPtr<FRDGPooledBuffer> OwnTexCoords;
    TRefCountPtr<FRDGPooledBuffer> OwnColors;
    TRefCountPtr<FRDGPooledBuffer> OwnArgs;
    TRefCountPtr<FRDGPooledBuffer> OwnIndexBuffer;
    uint64 OwnWrittenFrame = ~0ull;
    bool bOwnWritten = false;
};

void FVoxelOceanRingProvider::BuildComputePasses(FRDGBuilder& GraphBuilder, const FRealtimeMeshComputeContext& Context)
{
    FRDGBufferUAVRef Positions = Context.GetStreamUAV(FRealtimeMeshStreams::Position, PF_R32_FLOAT);
    FRDGBufferUAVRef Tangents = Context.GetStreamUAV(FRealtimeMeshStreams::Tangents, PF_R32_UINT);
    FRDGBufferUAVRef TexCoords = Context.GetStreamUAV(FRealtimeMeshStreams::TexCoords, PF_R32_UINT);
    FRDGBufferUAVRef Colors = Context.GetStreamUAV(FRealtimeMeshStreams::Color, PF_R32_UINT);
    FRDGBufferUAVRef Args = Context.GetIndirectArgsUAV();
    if (!Positions || !Tangents || !TexCoords || !Colors || !Args || !Write(GraphBuilder, Positions, Tangents, TexCoords, Colors, Args, 2u * VOP::TexCoordChannels))
        SkippedRuns.fetch_add(1, std::memory_order_relaxed);
}

bool FVoxelOceanRingProvider::Write(FRDGBuilder& GraphBuilder, FRDGBufferUAVRef Positions, FRDGBufferUAVRef Tangents, FRDGBufferUAVRef TexCoords,
    FRDGBufferUAVRef Colors, FRDGBufferUAVRef Args, uint32 TexWords)
{
    FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
    const bool bShaders = ShaderMap && ShaderMap->GetShader(&FVoxelOceanRingCS::GetStaticType(), 0).IsValid()
        && ShaderMap->GetShader(&FVoxelOceanFarDepthCS::GetStaticType(), 0).IsValid();
    if (!bShaders) return false;
    Runs.fetch_add(1, std::memory_order_relaxed);
    const uint32 FarCount = (uint32)FarDirectionWords.Num();
    const uint32 Vertices = VertexCount();
    RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanRings, "VoxelOceanRings");
    VoxelOceanGPUTime::FScope OceanGPUTime(GraphBuilder, VoxelOceanGPUTime::Mesh);

    // The far sphere's directions, once; its land when asked (zero, not culled, until then).
    FRDGBufferRef FarDirections = FarDirectionBuffer.IsValid() ? GraphBuilder.RegisterExternalBuffer(FarDirectionBuffer) : nullptr;
    if (!FarDirections)
    {
        FarDirections = VoxelGPUCommon::UploadStructuredBuffer<FVector4f>(GraphBuilder, TEXT("Voxel.OceanFarDirections"), FarDirectionWords);
        FarDirectionBuffer = GraphBuilder.ConvertToExternalBuffer(FarDirections);
    }
    FRDGBufferRef FarLand = FarLandBuffer.IsValid() ? GraphBuilder.RegisterExternalBuffer(FarLandBuffer) : nullptr;
    if (!FarLand)
    {
        FarLand = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), FMath::Max(FarCount, 1u)), TEXT("Voxel.OceanFarLand"));
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(FarLand), 0u);
        FarLandBuffer = GraphBuilder.ConvertToExternalBuffer(FarLand);
    }
    if (bFarJob && FarJob.Density.IsValid() && FarJob.Lattice.Num() == 3 * (int32)FarCount && FarCount > 0)
    {
        FVoxelOceanFarDepthParameters* P = GraphBuilder.AllocParameters<FVoxelOceanFarDepthParameters>();
        P->FarLattice = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<int32>(GraphBuilder, TEXT("Voxel.OceanFarLattice"), MakeConstArrayView(FarJob.Lattice)));
        P->FarLandOut = GraphBuilder.CreateUAV(FarLand);
        P->FarCount = FarCount;
        P->FarFootprint = FarJob.FootprintCm;
        VOP::FillDensity(GraphBuilder, *InputCache, FarJob.Density, FarJob.Edits, FarJob.HalfFinest, FarJob.Shore, FarJob.ShoreMarginCm, FarJob.DepthFootprintCm,
            FarJob.SeaRadiusCm, FarJob.LandCache, P->Density);
        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanFarDepth %u vertices", FarCount), TShaderMapRef<FVoxelOceanFarDepthCS>(ShaderMap), P,
            FIntVector((int32)FMath::DivideAndRoundUp(FarCount, 64u), 1, 1));
        bFarJob = false;
        FarJob = FFarJob();
    }

    // The vertices; nothing drawn (no indices) until the clipmap's levels are there this frame.
    const FVoxelOceanDisplaceState& D = *Displace;
    // The rings with the clipmap's levels this frame; the far sphere alone without (never written
    // its frame: nothing).
    const bool bLevels = D.bClip && D.bRings && D.ClipDisplacement.IsValid() && Clip.IsValid() && Clip->Static.IsValid();
    const bool bReady = D.bRingsFar;
    FVoxelOceanRingParameters* P = GraphBuilder.AllocParameters<FVoxelOceanRingParameters>();
    P->RingPositions = Positions;
    P->RingTangents = Tangents;
    P->RingTexCoords = TexCoords;
    P->RingColors = Colors;
    P->RingArgs = Args;
    P->RingIndexCount = bReady ? IndexCount : 0u;
    P->RingSlots = (uint32)Slots;
    P->RingHalf = Half;
    P->RingMorphStart = (float)Half * (2.0f / 3.0f);
    P->RingMorphEnd = (float)Half - 2.0f;
    P->RingFarCells = (uint32)FarCells;
    P->RingVertices = bReady ? Vertices : 0u;
    P->RingTexWords = TexWords;
    for (int32 i = 0; i < VoxelOceanClipmap::InfoRows; i++) P->RingInfo[i] = bLevels ? D.ClipRows[i] : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    for (int32 i = 0; i < VoxelOceanClipmap::MaxLevels; i++)
    {
        P->RingLevel[i] = D.RingLevel[i];
        P->RingInts[i] = D.RingInts[i];
    }
    P->RingCount = bLevels ? D.RingCount : 0u;
    P->RingFrame0 = D.RingFrame0;
    P->RingFrame1 = D.RingFrame1;
    P->RingFrame2 = D.RingFrame2;
    P->RingCull = D.RingCull;
    P->RingRotation = D.Frame.Rotation;
    P->RingDisplacement = bLevels ? D.ClipDisplacement.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
    FRDGTextureRef Static = bLevels ? GraphBuilder.RegisterExternalTexture(Clip->Static) : nullptr;
    if (!Static)
    {
        Static = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(1, 1), PF_A32B32G32R32F, FClearValueBinding::Black,
            TexCreate_ShaderResource | TexCreate_UAV, 1), TEXT("Voxel.OceanRingsNoStatic"));
        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Static), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
    }
    P->RingStatic = Static;
    P->RingSampler = TStaticSamplerState<SF_Bilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
    P->FarDirections = GraphBuilder.CreateSRV(FarDirections);
    P->FarLand = GraphBuilder.CreateSRV(FarLand);
    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanRings %u vertices", bReady ? Vertices : 0u), TShaderMapRef<FVoxelOceanRingCS>(ShaderMap), P,
        FIntVector((int32)FMath::Max(FMath::DivideAndRoundUp(bReady ? Vertices : 1u, 64u), 1u), 1, 1));
    if (bLevels) DrawnRuns.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool FVoxelOceanRingProvider::PrepareDraw_RenderThread(FRDGBuilder& GraphBuilder, FVoxelOceanRingDrawFrame& Out)
{
    check(IsInRenderingThread());
    const FVoxelOceanDisplaceState& D = *Displace;
    // Nothing until the frame's numbers have come (UpdateRings), and only with triangles to draw.
    if (!D.bRingsFar || IndexCount == 0) return false;
    const uint32 Vertices = VertexCount();
    constexpr uint32 TexWords = 6;

    // The buffers, kept between frames (any graph registers them).
    auto Keep = [&GraphBuilder](TRefCountPtr<FRDGPooledBuffer>& Pooled, const FRDGBufferDesc& Desc, const TCHAR* Name)
    {
        if (Pooled.IsValid()) return GraphBuilder.RegisterExternalBuffer(Pooled);
        FRDGBufferRef Buffer = GraphBuilder.CreateBuffer(Desc, Name);
        Pooled = GraphBuilder.ConvertToExternalBuffer(Buffer);
        return Buffer;
    };
    FRDGBufferRef Positions = Keep(OwnPositions, FRDGBufferDesc::CreateBufferDesc(sizeof(float), 3 * Vertices), TEXT("Voxel.OceanRingPositions"));
    FRDGBufferRef Tangents = Keep(OwnTangents, FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 2 * Vertices), TEXT("Voxel.OceanRingTangents"));
    FRDGBufferRef TexCoords = Keep(OwnTexCoords, FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), TexWords * Vertices), TEXT("Voxel.OceanRingTexCoords"));
    FRDGBufferRef Colors = Keep(OwnColors, FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), Vertices), TEXT("Voxel.OceanRingColors"));
    FRDGBufferRef Args = Keep(OwnArgs, FRDGBufferDesc::CreateBufferDesc(sizeof(uint32), 5), TEXT("Voxel.OceanRingArgs"));
    FRDGBufferRef Indices = OwnIndexBuffer.IsValid() ? GraphBuilder.RegisterExternalBuffer(OwnIndexBuffer) : nullptr;
    if (!Indices)
    {
        if (OwnIndices.Num() != (int32)IndexCount) return false;
        Indices = VoxelGPUCommon::UploadStructuredBuffer<uint32>(GraphBuilder, TEXT("Voxel.OceanRingIndices"), OwnIndices);
        OwnIndexBuffer = GraphBuilder.ConvertToExternalBuffer(Indices);
        OwnIndices.Empty();
    }

    // Written by the frame's first call (the same frame's other views and graphs read them).
    if (!bOwnWritten || OwnWrittenFrame != GFrameCounterRenderThread)
    {
        if (!Write(GraphBuilder, GraphBuilder.CreateUAV(Positions, PF_R32_FLOAT), GraphBuilder.CreateUAV(Tangents, PF_R32_UINT),
            GraphBuilder.CreateUAV(TexCoords, PF_R32_UINT), GraphBuilder.CreateUAV(Colors, PF_R32_UINT), GraphBuilder.CreateUAV(Args, PF_R32_UINT), TexWords))
        {
            SkippedRuns.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        OwnWrittenFrame = GFrameCounterRenderThread;
        bOwnWritten = true;
    }

    Out.Positions = GraphBuilder.CreateSRV(Positions, PF_R32_FLOAT);
    Out.TexCoords = GraphBuilder.CreateSRV(TexCoords, PF_R32_UINT);
    Out.Colors = GraphBuilder.CreateSRV(Colors, PF_R32_UINT);
    Out.Indices = GraphBuilder.CreateSRV(Indices);
    Out.TexWords = TexWords;
    Out.Triangles = IndexCount / 3;
    Out.Origin = D.RingOrigin;
    Out.SeaRadius = (double)D.RingFrame2.W;
    Out.PlanetToWorld = D.Frame.PlanetToWorld;
    Out.Waves = D.Frame;
    Out.Displacement = D.Displacement.GetReference();
    Out.Derivatives = D.Derivatives.GetReference();
    Out.Charts = D.Charts.GetReference();
    Out.Foam = D.Foam.GetReference();
    Out.FineDisplacement = D.FineDisplacement.GetReference();
    Out.FineDerivatives = D.FineDerivatives.GetReference();
    Out.FineFoam = D.FineFoam.GetReference();
    Out.bClip = D.bClip && D.ClipDisplacement.IsValid() && D.ClipNormal.IsValid() && D.ClipInfo.IsValid();
    Out.ClipDisplacement = D.ClipDisplacement.GetReference();
    Out.ClipNormal = D.ClipNormal.GetReference();
    Out.ClipWhitecap = D.ClipWhitecap.GetReference();
    Out.ClipStrain = D.ClipStrain.GetReference();
    Out.ClipInfo = D.ClipInfo.GetReference();
    for (int32 i = 0; i < VoxelOceanClipmap::InfoRows; i++) Out.ClipRows[i] = D.ClipRows[i];
    Out.ClipFineHistory = D.ClipFineHistory.GetReference();
    Out.ClipStatic = D.ClipStatic.GetReference();
    Out.ClipFineInfo = D.ClipFineHistory.IsValid() ? D.ClipFineInfo : FVector2f(0.0f, 0.0f);
    return true;
}

/** The rings' mesh (game thread; stage W2): one buffer set for every level and the far sphere. */
struct FVoxelOceanRingSet
{
    TWeakObjectPtr<URealtimeMeshSimple> Mesh;
    TWeakObjectPtr<URealtimeMeshComponent> Component;
    TSharedPtr<FVoxelOceanRingProvider, ESPMode::ThreadSafe> Provider;
    TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> bBuffersReady;
    /** Stage W3: drawn by the ocean's own pass (no mesh, no component, no registration). */
    bool bOwnPass = false;
    uint64 Handle = 0;
    uint64 CreatedFrame = 0;
    int32 Half = 0;
    int32 Slots = 0;
    int32 FarCells = 0;
    /** The far sphere's directions (double, for its lattice points) and cell (cm). */
    TArray<FVector> FarDirections;
    double FarCellCm = 0.0;
    /** The density version and lattice step its land was last asked for (0: never). */
    uint32 FarVersion = 0;
    double FarHalfFinest = 0.0;
    /** The component's origin (planet frame, cm). */
    FVector Origin = FVector::ZeroVector;
    bool bHasOrigin = false;
};

FVoxelOceanPages::FVoxelOceanPages()
    : DisplaceState(MakeShared<FVoxelOceanDisplaceState, ESPMode::ThreadSafe>())
    , ClipPlan(MakeUnique<FVoxelOceanClipPlan>())
    , ClipState(MakeShared<FVoxelOceanClipState, ESPMode::ThreadSafe>())
    , NoDensity(MakeShared<FVoxelDensityGPUData>())
    , NoEdits(MakeShared<FVoxelEditGPUData>())
    , InputCache(MakeShared<FVoxelOceanInputCache, ESPMode::ThreadSafe>())
    , ProbeState(MakeShared<FVoxelOceanProbeState, ESPMode::ThreadSafe>())
{
}

void FVoxelOceanPages::SetWavesInCompute(bool bInCompute)
{
    check(IsInGameThread());
    if (bInCompute == bWavesInCompute) return;
    // Every buffer set registers again: with the driver's every-frame trigger and a base buffer,
    // or on demand without.
    Reset();
    bWavesInCompute = bInCompute;
}

void FVoxelOceanPages::SetDisplaceFrame(const FVoxelOceanDisplaceFrame& InFrame)
{
    check(IsInGameThread());
    if (!bWavesInCompute) return;
    FVoxelOceanDisplaceFrame Frame = InFrame;
    Frame.MorphEnd0 = MorphEnd0;
    ENQUEUE_RENDER_COMMAND(VoxelOceanDisplaceFrame)([State = DisplaceState, Frame](FRHICommandListImmediate&)
        {
            auto RHIOf = [](FTextureResource* Resource) { return Resource ? Resource->TextureRHI : FTextureRHIRef(); };
            State->Frame = Frame;
            // The clipmap's levels only for a frame whose clipmap ran (UpdateClipmap's command,
            // after this one, sets it again; stage O9k).
            State->bClip = false;
            State->bRings = false;
            State->Displacement = RHIOf(Frame.Displacement);
            State->Derivatives = RHIOf(Frame.Derivatives);
            State->Charts = RHIOf(Frame.Charts);
            State->Shore = RHIOf(Frame.Shore);
            State->Foam = RHIOf(Frame.Foam);
            State->FineDisplacement = RHIOf(Frame.FineDisplacement);
            State->FineDerivatives = RHIOf(Frame.FineDerivatives);
            State->FineFoam = RHIOf(Frame.FineFoam);
            // The resources are only read here.
            State->Frame.Displacement = nullptr;
            State->Frame.Derivatives = nullptr;
            State->Frame.Charts = nullptr;
            State->Frame.Shore = nullptr;
            State->Frame.Foam = nullptr;
            State->Frame.FineDisplacement = nullptr;
            State->Frame.FineDerivatives = nullptr;
            State->Frame.FineFoam = nullptr;
        });
}

void FVoxelOceanPages::ProbeDepth(const VoxelOceanDepthProbe::FLevels& Levels, double InRadius)
{
    check(IsInGameThread());
    if (!FApp::CanEverRender()) return;
    TArray<int32> Lattice;
    VoxelOceanDepthProbe::LatticeOf(Levels, InRadius, DensityInputs.HalfFinest, Lattice);
    static_assert(VoxelOceanDepthProbe::Levels == 4, "the probe's footprints go to the shader as one float4");
    const FVector4f Footprint((float)Levels.Spacing[0], (float)Levels.Spacing[1], (float)Levels.Spacing[2], (float)Levels.Spacing[3]);
    const TSharedPtr<const FVoxelDensityGPUData> Density = DensityInputs.Density.IsValid() ? DensityInputs.Density : NoDensity;
    const TSharedPtr<const FVoxelEditGPUData> Edits = DensityInputs.Edits.IsValid() ? DensityInputs.Edits : NoEdits;
    ENQUEUE_RENDER_COMMAND(VoxelOceanProbeDepth)([State = ProbeState, Cache = InputCache, Density, Edits, HalfFinest = DensityInputs.HalfFinest,
        Shore = DensityInputs.Shore, Margin = DensityInputs.ShoreMarginCm, DepthFootprintCm = DensityInputs.DepthFootprintCm, InRadius, Lattice = MoveTemp(Lattice),
        Footprint, Levels, LandCache = DensityInputs.LandCache](FRHICommandListImmediate& RHICmdList)
        {
            FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            if (!ShaderMap || !ShaderMap->GetShader(&FVoxelOceanProbeCS::GetStaticType(), 0).IsValid())
            {
                State->bRetry.store(true, std::memory_order_release);
                return;
            }
            FRDGBuilder GraphBuilder(RHICmdList);
            // (The event scope closes before Execute: RDG's breadcrumbs assert on one left open.)
            {
                RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanProbe, "VoxelOceanProbe");
                VoxelOceanGPUTime::FScope OceanGPUTime(GraphBuilder, VoxelOceanGPUTime::Other);
                const uint32 Count = (uint32)VoxelOceanDepthProbe::Points;
                // The depths, then the bed's slopes (stage O9b2).
                FRDGBufferRef Out = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), 2 * Count), TEXT("Voxel.OceanProbe"));
                FVoxelOceanProbeParameters* P = GraphBuilder.AllocParameters<FVoxelOceanProbeParameters>();
                P->ProbePoints = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<int32>(GraphBuilder, TEXT("Voxel.OceanProbePoints"), MakeConstArrayView(Lattice)));
                P->ProbeOut = GraphBuilder.CreateUAV(Out);
                P->ProbeCount = Count;
                P->ProbeLevelPoints = (uint32)(VoxelOceanDepthProbe::Size * VoxelOceanDepthProbe::Size);
                P->ProbeFootprint = Footprint;
                VOP::FillDensity(GraphBuilder, *Cache, Density, Edits, HalfFinest, Shore, Margin, DepthFootprintCm, InRadius, LandCache, P->Density);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanProbe %u points", Count), TShaderMapRef<FVoxelOceanProbeCS>(ShaderMap), P,
                    FIntVector((int32)FMath::DivideAndRoundUp(Count, 64u), 1, 1));
                GraphBuilder.QueueBufferExtraction(Out, &State->Depth);
                OceanGPUTime.End();
            }
            GraphBuilder.Execute();
            State->Levels = Levels;
            State->bValid = State->Depth.IsValid();
        });
}

bool FVoxelOceanPages::SampleCoastDepths(const TSharedPtr<FVoxelOceanCoastDepthJob, ESPMode::ThreadSafe>& Job, int64 First, int32 Count, double InRadius)
{
    check(IsInGameThread());
    if (!FApp::CanEverRender() || !Job.IsValid() || Count <= 0 || First < 0 || First + Count > Job->Cells) return false;
    // The terrain's density must be there (the stand-in has no terrain: every point deep water),
    // and the shader compiled.
    if (!DensityInputs.Density.IsValid()) return false;
    {
        FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
        if (!ShaderMap || !ShaderMap->GetShader(&FVoxelOceanCoastDepthCS::GetStaticType(), 0).IsValid()) return false;
    }
    const TSharedPtr<const FVoxelDensityGPUData> Density = DensityInputs.Density;
    const TSharedPtr<const FVoxelEditGPUData> Edits = DensityInputs.Edits.IsValid() ? DensityInputs.Edits : NoEdits;
    const double HalfFinest = FMath::Max(DensityInputs.HalfFinest, 1.0e-3);
    ENQUEUE_RENDER_COMMAND(VoxelOceanCoastDepths)([Job, Cache = InputCache, Density, Edits, HalfFinest, Shore = DensityInputs.Shore,
        Margin = DensityInputs.ShoreMarginCm, DepthFootprintCm = DensityInputs.DepthFootprintCm, InRadius, First, Count](FRHICommandListImmediate& RHICmdList)
        {
            FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            if (!ShaderMap || !ShaderMap->GetShader(&FVoxelOceanCoastDepthCS::GetStaticType(), 0).IsValid())
            {
                // Gone since the game thread looked: the job fails (the builder reports it).
                Job->InFlight.fetch_sub(1, std::memory_order_acq_rel);
                Job->Failed.store(1, std::memory_order_release);
                return;
            }
            FRDGBuilder GraphBuilder(RHICmdList);
            // The readback, released on the render thread whichever thread lets it go last (a poll
            // torn down at exit: as the shore builder's).
            TSharedPtr<VOP::FCoastReadback, ESPMode::ThreadSafe> Readback = MakeShared<VOP::FCoastReadback, ESPMode::ThreadSafe>();
            Readback->Buffer = MakeUnique<FRHIGPUBufferReadback>(TEXT("Voxel.OceanCoastDepthsReadback"));
            // (The event scope closes before Execute: RDG's breadcrumbs assert on one left open.)
            {
                RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanProbe, "VoxelOceanCoastDepths");
                VoxelOceanGPUTime::FScope OceanGPUTime(GraphBuilder, VoxelOceanGPUTime::Other);
                FRDGBufferRef Out = GraphBuilder.CreateBuffer(FRDGBufferDesc::CreateStructuredDesc(sizeof(float), (uint32)Count), TEXT("Voxel.OceanCoastDepths"));
                const uint32 NumGroups = FMath::Max(FMath::DivideAndRoundUp((uint32)Count, 64u), 1u);
                const uint32 GroupsX = FMath::Min(NumGroups, 32768u);
                FVoxelOceanCoastDepthParameters* P = GraphBuilder.AllocParameters<FVoxelOceanCoastDepthParameters>();
                // (Stage W7d: none for the shore field's bake cells; the upload stands one in.)
                P->CoastTiles = GraphBuilder.CreateSRV(VoxelGPUCommon::UploadStructuredBuffer<int32>(GraphBuilder, TEXT("Voxel.OceanCoastTiles"), MakeConstArrayView(Job->Tiles)));
                P->CoastMode = (uint32)FMath::Max(Job->Mode, 0);
                P->CoastOut = GraphBuilder.CreateUAV(Out);
                P->CoastFirst = (uint32)First;
                P->CoastCount = (uint32)Count;
                P->CoastR = (uint32)Job->R;
                P->CoastN = (uint32)Job->N;
                P->CoastGroupsX = GroupsX;
                P->CoastFootprint = (float)Job->FootprintCm;
                P->CoastLatticeScale = (float)(InRadius / HalfFinest);
                // The density only (stage O9m): the coastal field is the queries' and the server's too,
                // which sample the terrain on the CPU.
                VOP::FillDensity(GraphBuilder, *Cache, Density, Edits, HalfFinest, Shore, Margin, DepthFootprintCm, InRadius, nullptr, P->Density);
                FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanCoastDepths %d cells", Count), TShaderMapRef<FVoxelOceanCoastDepthCS>(ShaderMap), P,
                    FIntVector((int32)GroupsX, (int32)FMath::DivideAndRoundUp(NumGroups, GroupsX), 1));
                AddEnqueueCopyPass(GraphBuilder, Readback->Buffer.Get(), Out, (uint32)Count * (uint32)sizeof(float));
                OceanGPUTime.End();
            }
            GraphBuilder.Execute();
            VoxelGPUCommon::PollRenderThread([Job, Readback, First, Count]() -> bool
                {
                    if (!Readback->Buffer.IsValid() || !Readback->Buffer->IsReady()) return false;
                    const void* Data = Readback->Buffer->Lock((uint32)Count * (uint32)sizeof(float));
                    FMemory::Memcpy(Job->DepthCm.GetData() + First, Data, (SIZE_T)Count * sizeof(float));
                    Readback->Buffer->Unlock();
                    Readback->Buffer.Reset();
                    Job->Received.fetch_add(Count, std::memory_order_acq_rel);
                    Job->InFlight.fetch_sub(1, std::memory_order_acq_rel);
                    return true;
                });
        });
    return true;
}

FVoxelOceanPages::~FVoxelOceanPages()
{
    Reset();
    DestroyRings();
    // The cached graph buffers are let go on the render thread (the providers may still hold
    // the cache there too).
    ENQUEUE_RENDER_COMMAND(VoxelOceanReleaseInputs)([Cache = MoveTemp(InputCache), Clip = MoveTemp(ClipState), Displace = MoveTemp(DisplaceState)](FRHICommandListImmediate&) mutable
        {
            Cache.Reset();
            Clip.Reset();
            Displace.Reset();
        });
}

void FVoxelOceanPages::SetDensityInputs(const FVoxelOceanDensityInputs& InInputs)
{
    // The clipmap's static layer is written again when what it samples changes (stages O9k, W1;
    // the edits reach it as rectangles instead: NotifyClipEdit).
    if (InInputs.Density != DensityInputs.Density || InInputs.Shore != DensityInputs.Shore || InInputs.LandCache != DensityInputs.LandCache
        || InInputs.HalfFinest != DensityInputs.HalfFinest || InInputs.ShoreMarginCm != DensityInputs.ShoreMarginCm
        || InInputs.DepthFootprintCm != DensityInputs.DepthFootprintCm)
        DensityVersion++;
    DensityInputs = InInputs;
}

namespace VoxelOceanPagesDetail
{
    /** A transient texture array of the clipmap (no mips, bilinear, wrapping: the levels are
     *  stored toroidally, stage W1), zeroed. */
    UTexture2DArray* MakeClipArray(const TCHAR* Name, int32 Size, int32 Slices, EPixelFormat Format, int64 BytesPerTexel)
    {
        UTexture2DArray* Texture = NewObject<UTexture2DArray>(GetTransientPackage(),
            MakeUniqueObjectName(GetTransientPackage(), UTexture2DArray::StaticClass(), FName(Name)), RF_Transient);
        if (!Texture) return nullptr;
        FTexturePlatformData* Data = new FTexturePlatformData();
        Data->SizeX = Size;
        Data->SizeY = Size;
        Data->SetNumSlices(Slices);
        Data->PixelFormat = Format;
        FTexture2DMipMap* Level = new FTexture2DMipMap(Size, Size, Slices);
        Data->Mips.Add(Level);
        const int64 Bytes = (int64)Size * Size * Slices * BytesPerTexel;
        Level->BulkData.Lock(LOCK_READ_WRITE);
        void* Memory = Level->BulkData.Realloc(Bytes);
        FMemory::Memzero(Memory, Bytes);
        Level->BulkData.Unlock();
        Texture->SetPlatformData(Data);
        Texture->SRGB = false;
        Texture->CompressionSettings = TC_HDR;
        Texture->Filter = TF_Bilinear;
        Texture->AddressX = TA_Wrap;
        Texture->AddressY = TA_Wrap;
        Texture->AddressZ = TA_Clamp;
        Texture->LODGroup = TEXTUREGROUP_Pixels2D;
        Texture->NeverStream = true;
        Texture->UpdateResource();
        return Texture;
    }
}

void FVoxelOceanPages::SetClipmap(const FVoxelOceanClipSettings& InSettings)
{
    check(IsInGameThread());
    FVoxelOceanClipSettings S = InSettings;
    S.Size = (int32)FMath::Clamp<uint32>(FMath::RoundUpToPowerOfTwo((uint32)FMath::Max(S.Size, 8)), 32u, 512u);
    S.Levels = FMath::Clamp(S.Levels, 1, VoxelOceanClipmap::MaxLevels);
    S.FinestCm = FMath::Max(S.FinestCm, 1.0);
    const bool bSameTextures = ClipDisplacement.IsValid() && S.Size == ClipSettings.Size && S.Levels == ClipSettings.Levels;
    // The rings' mesh is made for the size, the levels and the far sphere's cells (stage W2).
    if (S != ClipSettings) DestroyRings();
    ClipSettings = S;
    ClipPlan = MakeUnique<FVoxelOceanClipPlan>();
    if (!S.bOn || !FApp::CanEverRender())
    {
        ClipDisplacement.Reset();
        ClipNormal.Reset();
        ClipWhitecap.Reset();
        ClipInfo.Reset();
        return;
    }
    if (bSameTextures) return;
    ClipDisplacement.Reset(VOP::MakeClipArray(TEXT("VoxelOceanClipDisplacement"), S.Size, S.Levels, PF_A32B32G32R32F, 16));
    ClipNormal.Reset(VOP::MakeClipArray(TEXT("VoxelOceanClipNormal"), S.Size, S.Levels, PF_FloatRGBA, 8));
    // (Stage W3u: four channels, the surf's zone and own foam besides.)
    ClipWhitecap.Reset(VOP::MakeClipArray(TEXT("VoxelOceanClipWhitecap"), S.Size, S.Levels, PF_FloatRGBA, 8));
    UTexture2D* Info = UTexture2D::CreateTransient(VoxelOceanClipmap::InfoRows, 1, PF_A32B32G32R32F,
        MakeUniqueObjectName(GetTransientPackage(), UTexture2D::StaticClass(), FName(TEXT("VoxelOceanClipInfo"))));
    if (Info)
    {
        Info->SRGB = false;
        Info->CompressionSettings = TC_HDR;
        Info->Filter = TF_Nearest;
        Info->AddressX = TA_Clamp;
        Info->AddressY = TA_Clamp;
        Info->LODGroup = TEXTUREGROUP_Pixels2D;
        Info->NeverStream = true;
        Info->UpdateResource();
    }
    ClipInfo.Reset(Info);
    if (!ClipDisplacement.IsValid() || !ClipNormal.IsValid() || !ClipWhitecap.IsValid() || !ClipInfo.IsValid())
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] Couldn't create the wave clipmap's textures: the vertices evaluate the waves themselves."));
        ClipDisplacement.Reset();
        ClipNormal.Reset();
        ClipWhitecap.Reset();
        ClipInfo.Reset();
        return;
    }
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Wave clipmap: %d levels of %d x %d, finest %.0f cm."), S.Levels, S.Size, S.Size, S.FinestCm);
}

void FVoxelOceanPages::UpdateClipmap(const FVector& CameraPlanet, const FVoxelOceanDisplaceFrame& Waves, FTextureResource* Foam,
    double OceanTime, double FoamDecayPerSecond, double FoamGrowthPerSecond, bool bWavesStepped, int32 FoamDetailLevels,
    double SurfFoamDecayPerSecond, double FoamBreakup, double FoamCutCm, bool bFineFromClip)
{
    // (Aliased: UE has a global FFrame.)
    using namespace VoxelOceanClipmap;
    using FClipFrame = VoxelOceanClipmap::FFrame;
    using FClipLevel = VoxelOceanClipmap::FLevel;
    using FClipRect = VoxelOceanClipmap::FRect;
    check(IsInGameThread());
    if (!IsClipmapOn() || !bWavesInCompute) return;
    FTextureResource* DispResource = ClipDisplacement->GetResource();
    FTextureResource* NormalResource = ClipNormal->GetResource();
    FTextureResource* WhitecapResource = ClipWhitecap.IsValid() ? ClipWhitecap->GetResource() : nullptr;
    FTextureResource* InfoResource = ClipInfo.IsValid() ? ClipInfo->GetResource() : nullptr;
    if (!DispResource || !NormalResource || !WhitecapResource || !InfoResource) return;
    if (!ClipPlan.IsValid()) ClipPlan = MakeUnique<FVoxelOceanClipPlan>();
    FVoxelOceanClipPlan& Plan = *ClipPlan;
    // A frame whose passes couldn't run: everything is written again.
    if (ClipState->bLost.exchange(false))
    {
        for (FVoxelOceanClipPlan::FSlot& Slot : Plan.Slots) Slot = FVoxelOceanClipPlan::FSlot();
        for (FVoxelOceanClipPlan::FFine& Fine : Plan.Fine) Fine = FVoxelOceanClipPlan::FFine();
    }

    // The levels: on the shared plane, the first's spacing by the camera's height.
    const int32 Slots = ClipSettings.Levels;
    const FClipFrame Frame = MakeFrame(CameraPlanet, Radius, ClipSettings.Size, Slots, ClipSettings.FinestCm, Plan.bHasFrame ? &Plan.Frame : nullptr);
    // Stage W6f part 2: THE CLIPMAP OFF FROM HIGH UP. Above ClipMaxHeightCm, or once the plane bounds
    // the coarsest level short of the horizon (its square showed on the globe from orbit), or with no
    // level at all: no static layer, no waves, no foam passes; the far sphere draws the sea, shaded
    // by the sea state's statistics (VoxelOceanFFTSlopeVarianceFar, VoxelOceanFFTFoamFarOf): from
    // there every wave is under a pixel. On again below 0.9 of the height it went off at.
    {
        const double Height = CameraPlanet.Size() - Radius;
        const double MaxHeight = ClipSettings.ClipMaxHeightCm;
        const bool bWantOff = !Frame.IsValid() || Frame.bPlaneLimited || (MaxHeight > 0.0 && Height > MaxHeight);
        if (!bClipHigh && bWantOff)
        {
            bClipHigh = true;
            ClipOffHeightCm = Height;
            if (MaxHeight > 0.0) ClipOffHeightCm = FMath::Min(ClipOffHeightCm, MaxHeight);
        }
        else if (bClipHigh && !bWantOff && Height < 0.9 * ClipOffHeightCm)
            bClipHigh = false;
        if (bClipHigh)
        {
            ENQUEUE_RENDER_COMMAND(VoxelOceanClipOff)([Displace = DisplaceState](FRHICommandListImmediate&)
                {
                    Displace->bClip = false;
                });
            // Every level written afresh when it comes back.
            for (FVoxelOceanClipPlan::FSlot& Slot : Plan.Slots) Slot = FVoxelOceanClipPlan::FSlot();
            for (FVoxelOceanClipPlan::FFine& Fine : Plan.Fine) Fine = FVoxelOceanClipPlan::FFine();
            Plan.bHasFrame = false;
            return;
        }
    }
    if (!Frame.IsValid()) return;
    if (Plan.bHasFrame && Frame.Plane.Id != Plan.Frame.Plane.Id) Plan.Reanchors++;
    const int32 N = Frame.Size;

    // What the static layer was written from: a texel written for other inputs is written again.
    auto Mix = [](uint64 H, uint64 V) { return H ^ (V + 0x9e3779b97f4a7c15ull + (H << 6) + (H >> 2)); };
    uint64 Key = Mix(0x51ed270b0b2a5ull, (uint64)DensityVersion);
    Key = Mix(Key, (uint64)(UPTRINT)Waves.Shore);
    Key = Mix(Key, Waves.States.X > 0.5f ? 1u : 0u);
    Key = Mix(Key, Waves.States.Y > 0.0f ? 1u : 0u);
    Key = Mix(Key, (uint64)FMath::RoundToInt(Waves.SlopeVar.Z));
    Key = Mix(Key, (uint64)FMath::RoundToInt(Waves.FFT.Y));

    // The static layer's rectangles: a level new to its slot (or written for another plane or
    // other inputs) whole, else the strips its window gained and the edits' rectangles.
    struct FStaticJob
    {
        int32 Level = 0;
        FClipRect Rect;
        /** Stage W3t: the foam's history starts afresh there (texels new to the window; not an
         *  edit's rectangle, where the water was already). */
        bool bReseed = true;
    };
    TArray<FStaticJob> StaticJobs;
    bool bWhole[MaxLevels] = {};
    bool bEdited[MaxLevels] = {};
    bool bAllWhole = true;
    for (int32 i = 0; i < Frame.Count; i++)
    {
        const FClipLevel& L = Frame.Levels[i];
        FVoxelOceanClipPlan::FSlot& Slot = Plan.Slots[L.Slot];
        const bool bSame = Slot.bStatic && Slot.Plane == Frame.Plane.Id && Slot.K == L.K && Slot.Key == Key;
        if (!bSame)
        {
            StaticJobs.Add({ i, FClipRect{ 0, 0, N, N } });
            bWhole[i] = true;
        }
        else
        {
            bAllWhole = false;
            FClipRect Gained[2];
            const int32 NumGained = GainedStrips(Slot.Origin, L.Origin, N, Gained);
            for (int32 g = 0; g < NumGained; g++)
            {
                StaticJobs.Add({ i, Gained[g] });
                if (Gained[g].Width == N && Gained[g].Height == N) bWhole[i] = true;
            }
            // An edit reaches a texel through the depth's footprint (the waves feel it smoothed
            // over twice the spacing or the depth smoothing) and the bed's slope step, and the
            // land cache's texel at that footprint; then its waves are due at once.
            const double Footprint = FMath::Max(FMath::Max(2.0 * L.Spacing, DensityInputs.DepthFootprintCm), 100.0);
            for (const FVoxelOceanClipPlan::FEdit& Edit : Plan.Edits)
            {
                const FClipRect E = RectAround(Frame, L, Edit.Center, Edit.Reach + 2.0 * Footprint, 2.0);
                if (E.IsEmpty()) continue;
                StaticJobs.Add({ i, E, false });
                bEdited[i] = true;
            }
            // Stage W7a: a new coastal window in the same texture: the whole level again (its
            // coastal values), the water already there (the foam's history kept), its waves due.
            if (Slot.CoastSerial != DensityInputs.CoastSerial)
            {
                StaticJobs.Add({ i, FClipRect{ 0, 0, N, N }, false });
                bEdited[i] = true;
            }
        }
        Slot.bStatic = true;
        Slot.Plane = Frame.Plane.Id;
        Slot.K = L.K;
        Slot.Origin = L.Origin;
        Slot.Key = Key;
        Slot.CoastSerial = DensityInputs.CoastSerial;
    }
    Plan.Edits.Reset();
    for (const FStaticJob& J : StaticJobs) Plan.StaticTexels += (int64)J.Rect.Width * J.Rect.Height;

    // The waves due: every level whose static layer is new, or whose window has moved on from
    // the one its waves were evaluated for; the fine ones every frame, the coarse ones in turn.
    const int32 Stagger = FMath::Clamp(CVarVoxelOceanClipStagger.GetValueOnGameThread(), 0, 1);
    // (Stage W3y: the turns, and the levels' regular evaluation, only with the cascades' steps:
    // between them the waves are the same.)
    const uint64 Turn = bWavesStepped ? Plan.Turn++ : Plan.Turn;
    int32 Due[MaxLevels];
    int32 NumDue = 0;
    int32 Shift[MaxLevels] = {};
    FIntPoint Evaluated[MaxLevels];
    bool Valid[MaxLevels];
    for (int32 i = 0; i < Frame.Count; i++)
    {
        const FClipLevel& L = Frame.Levels[i];
        FVoxelOceanClipPlan::FSlot& Slot = Plan.Slots[L.Slot];
        const bool bSameLevel = Slot.bEvaluated && Slot.EvalPlane == Frame.Plane.Id && Slot.EvalK == L.K;
        const bool bNear = bSameLevel && FMath::Abs(Slot.EvalOrigin.X - L.Origin.X) <= EvaluatedSlack && FMath::Abs(Slot.EvalOrigin.Y - L.Origin.Y) <= EvaluatedSlack;
        const uint64 Period = Stagger == 0 || i < 4 ? 1u : (i < 8 ? 2u : 4u);
        if (bWhole[i] || bEdited[i] || !bNear || (bWavesStepped && Turn % Period == (uint64)i % Period))
        {
            // (Stage W3t: how far its window moved since its last step, texels: the foam history's
            // margin of possibly new neighbours.)
            Shift[i] = bSameLevel ? FMath::Max(FMath::Abs(Slot.EvalOrigin.X - L.Origin.X), FMath::Abs(Slot.EvalOrigin.Y - L.Origin.Y)) : 0;
            Due[NumDue++] = i;
            Slot.bEvaluated = true;
            Slot.EvalPlane = Frame.Plane.Id;
            Slot.EvalK = L.K;
            Slot.EvalOrigin = L.Origin;
        }
        Evaluated[i] = Slot.EvalOrigin;
        Valid[i] = true;
    }

    struct FJob
    {
        FVoxelOceanDisplaceFrame Waves;
        int32 Size = 0;
        int32 Slots = 0;
        FVector4f Rows[InfoRows];
        FVector4f Window[MaxLevels];
        FIntVector4 WindowMod[MaxLevels];
        int32 Due[MaxLevels] = {};
        int32 NumDue = 0;
        TArray<FStaticJob> Static;
        bool bAllWhole = false;
        FIntVector CameraLattice = FIntVector::ZeroValue;
        FVector3f CameraFrac = FVector3f::ZeroVector;
        double Radius = 0.0;
        double HalfFinest = 50.0;
        // The rings this frame (stage W2).
        bool bRings = false;
        uint32 RingCount = 0;
        FVector4f RingLevel[MaxLevels];
        FIntVector4 RingInts[MaxLevels];
        /** The rings' half side and morph (VoxelOceanClip.usf's ClipRing). */
        FVector4f RingLayout = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        /** Stage W3t: the ocean's clock (s); stage W3v: the foam's decay and growth rates (per
         *  second; a decay of 0: no foam buffer). */
        double OceanTime = 0.0;
        double FoamDecay = 0.0;
        double FoamGrowth = 0.0;
        int32 FoamShift[MaxLevels] = {};
        /** Stage W5a: the surf's foam's decay (per second) and the foam's breakup. */
        double SurfFoamDecay = 0.4;
        double FoamBreakup = 0.0;
        /** Stage W5f: the foam's far cut (cm; 0: none). */
        double FoamCut = 0.0;
        /** Stage W4c: the foam's fine history: its levels (the first FineCount in use), and those
         *  stepped this frame with the window strips new to them since their last step. */
        int32 FineCount = 0;
        /** Stage W8k: the fine foam's resolved long bands from the levels' strains. */
        bool bFineHeld = true;
        struct FFineStep
        {
            int32 Level = 0;
            bool bWhole = true;
            FClipRect Rect[2];
        };
        FFineStep FineSteps[FineMaxLevels];
        int32 NumFineSteps = 0;
    };
    TSharedRef<FJob> Job = MakeShared<FJob>();
    Job->Waves = Waves;
    Job->Size = N;
    Job->Slots = Slots;
    MakeInfoRows(Frame, Evaluated, Valid, Job->Rows);
    for (int32 i = 0; i < MaxLevels; i++)
    {
        Job->Window[i] = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        Job->WindowMod[i] = FIntVector4(0, 0, 0, 0);
        if (i >= Frame.Count) continue;
        const FClipLevel& L = Frame.Levels[i];
        const FVector2D InWindow = CameraInWindow(Frame, L.Spacing, L.Origin);
        Job->Window[i] = FVector4f((float)InWindow.X, (float)InWindow.Y, (float)L.Spacing, (float)L.Slot);
        Job->WindowMod[i] = FIntVector4(PositiveMod(L.Origin.X, N), PositiveMod(L.Origin.Y, N), 0, 0);
    }
    for (int32 i = 0; i < NumDue; i++) Job->Due[i] = Due[i];
    Job->NumDue = NumDue;
    Job->Static = MoveTemp(StaticJobs);
    Job->bAllWhole = bAllWhole;
    Job->Radius = Radius;
    Job->OceanTime = OceanTime;
    Job->FoamDecay = FMath::Max(FoamDecayPerSecond, 0.0);
    Job->FoamGrowth = FMath::Max(FoamGrowthPerSecond, 0.0);
    Job->SurfFoamDecay = FMath::Max(SurfFoamDecayPerSecond, 0.0);
    Job->FoamBreakup = FMath::Max(FoamBreakup, 0.0);
    Job->FoamCut = FMath::Max(FoamCutCm, 0.0);
    Job->bFineHeld = bFineFromClip;
    for (int32 i = 0; i < MaxLevels; i++) Job->FoamShift[i] = FMath::Clamp(Shift[i], 0, 64);
    // Stage W4c: the foam's fine history: every level each wave step (stage W4d: W4c stepped one a
    // step in turn, 12.5 Hz each, and the foam moved in visible increments), any whose window moved
    // (or that is new) at once; the strips it gained since its last step start afresh.
    Job->FineCount = Job->FoamDecay > 0.0 ? FMath::Clamp(FoamDetailLevels, 0, FMath::Min(Frame.Count, FineMaxLevels)) : 0;
    if (Job->FineCount > 0)
    {
        for (int32 i = 0; i < Job->FineCount; i++)
        {
            const FClipLevel& L = Frame.Levels[i];
            FVoxelOceanClipPlan::FFine& Fine = Plan.Fine[i];
            const bool bSame = Fine.bStepped && Fine.Plane == Frame.Plane.Id && Fine.K == L.K;
            const bool bMoved = bSame && Fine.Origin != L.Origin;
            const bool bTurn = bWavesStepped;
            if (bSame && !bMoved && !bTurn) continue;
            FJob::FFineStep& Step = Job->FineSteps[Job->NumFineSteps++];
            Step.Level = i;
            Step.bWhole = !bSame;
            if (bMoved)
            {
                const int32 NumGained = GainedStrips(Fine.Origin, L.Origin, N, Step.Rect);
                for (int32 g = 0; g < NumGained; g++)
                    if (Step.Rect[g].Width == N && Step.Rect[g].Height == N) Step.bWhole = true;
            }
            Fine.bStepped = true;
            Fine.Plane = Frame.Plane.Id;
            Fine.K = L.K;
            Fine.Origin = L.Origin;
        }
    }
    else
    {
        // (Off: written whole when on again.)
        for (FVoxelOceanClipPlan::FFine& Fine : Plan.Fine) Fine = FVoxelOceanClipPlan::FFine();
    }
    Job->HalfFinest = FMath::Max(DensityInputs.HalfFinest, 1.0e-3);
    {
        // The camera's nearest lattice point, in double (the static layer counts from it).
        const FVector Lattice(FMath::RoundToDouble(CameraPlanet.X / Job->HalfFinest), FMath::RoundToDouble(CameraPlanet.Y / Job->HalfFinest),
            FMath::RoundToDouble(CameraPlanet.Z / Job->HalfFinest));
        Job->CameraLattice = FIntVector((int32)Lattice.X, (int32)Lattice.Y, (int32)Lattice.Z);
        Job->CameraFrac = FVector3f(CameraPlanet - Lattice * Job->HalfFinest);
    }
    Plan.Frame = Frame;
    Plan.bHasFrame = true;

    // The rings (stage W2; VoxelOceanRingLayout.ush): level i's cell is twice its texel; its square
    // centred on the even vertex nearest the camera (so it lies on the next level's grid); its hole
    // where the previous level's square lies, its centre on the next level's grid.
    if (Rings.IsValid())
    {
        // Stage W6f: from high up none (the far sphere draws the whole sea, unsunk): their waves are
        // under a pixel, and the outermost ring's stretched cells sagged below the sphere (a square
        // that showed from orbit). Off above RingsMaxHeightCm, on again below 0.9 of it.
        const double MaxHeight = ClipSettings.RingsMaxHeightCm;
        if (MaxHeight <= 0.0) bRingsHigh = false;
        else if (Frame.CameraHeight > MaxHeight) bRingsHigh = true;
        else if (Frame.CameraHeight < 0.9 * MaxHeight) bRingsHigh = false;
        const int32 Count = bRingsHigh ? 0 : FMath::Min(Frame.Count, Rings->Slots);
        FRingRow RingRows[MaxLevels];
        MakeRings(Frame, Count, RingRows);
        for (int32 i = 0; i < MaxLevels; i++)
        {
            const FRingRow& Row = RingRows[i];
            Job->RingLevel[i] = FVector4f((float)Row.CamX, (float)Row.CamY, (float)Row.Spacing, 0.0f);
            Job->RingInts[i] = FIntVector4(Row.HoleX, Row.HoleY, Row.CoarseX, Row.CoarseY);
        }
        Job->bRings = true;
        Job->RingCount = (uint32)Count;
        // As the rings' pass morphs (FVoxelOceanRingProvider::Write).
        const float Half = (float)Rings->Half;
        Job->RingLayout = FVector4f(Half, Half * (2.0f / 3.0f), Half - 2.0f, (float)Count);
    }

    const TSharedPtr<const FVoxelDensityGPUData> Density = DensityInputs.Density.IsValid() ? DensityInputs.Density : NoDensity;
    const TSharedPtr<const FVoxelEditGPUData> Edits = DensityInputs.Edits.IsValid() ? DensityInputs.Edits : NoEdits;
    ENQUEUE_RENDER_COMMAND(VoxelOceanClipmap)([Job, State = ClipState, Displace = DisplaceState, Cache = InputCache, Density, Edits,
        Shore = DensityInputs.Shore, Margin = DensityInputs.ShoreMarginCm, DepthFootprintCm = DensityInputs.DepthFootprintCm, LandCache = DensityInputs.LandCache,
        DispResource, NormalResource, WhitecapResource, InfoResource, Foam](FRHICommandListImmediate& RHICmdList)
        {
            FGlobalShaderMap* ShaderMap = GetGlobalShaderMap(GMaxRHIFeatureLevel);
            auto Has = [ShaderMap](FShaderType& Type) { return ShaderMap && ShaderMap->GetShader(&Type, 0).IsValid(); };
            FRHITexture* DispRHI = DispResource->TextureRHI;
            FRHITexture* NormalRHI = NormalResource->TextureRHI;
            FRHITexture* WhitecapRHI = WhitecapResource->TextureRHI;
            FRHITexture* InfoRHI = InfoResource->TextureRHI;
            if (!Has(FVoxelOceanClipWaveCS::GetStaticType()) || !Has(FVoxelOceanClipStaticCS::GetStaticType()) || !DispRHI || !NormalRHI || !WhitecapRHI || !InfoRHI)
            {
                // Nothing written this frame: the game thread plans everything again.
                State->bLost.store(true);
                Displace->bClip = false;
                return;
            }
            const int32 Size = Job->Size;
            const FVoxelOceanDisplaceFrame& W = Job->Waves;
            auto RHIOf = [](FTextureResource* Resource) { return Resource ? Resource->TextureRHI : FTextureRHIRef(); };
            const FTextureRHIRef Displacement = RHIOf(W.Displacement);
            const FTextureRHIRef Derivatives = RHIOf(W.Derivatives);
            const FTextureRHIRef Charts = RHIOf(W.Charts);
            const FTextureRHIRef ShoreTexture = RHIOf(W.Shore);
            const FTextureRHIRef FoamTexture = RHIOf(Foam);
            const bool bFFT = W.FFT.X > 0.5f && Displacement && Derivatives && Charts;
            FVector4f States = W.States;
            if (!ShoreTexture) States.X = 0.0f;
            FRHITexture* ChartsRHI = Charts ? Charts.GetReference() : GBlackTexture->TextureRHI.GetReference();
            FRHITexture* ShoreRHI = ShoreTexture ? ShoreTexture.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();

            FRDGBuilder GraphBuilder(RHICmdList);
            {
                RDG_EVENT_SCOPE_STAT(GraphBuilder, VoxelOceanClipmap, "VoxelOceanClipmap");
                VoxelOceanGPUTime::FScope OceanGPUTime(GraphBuilder, VoxelOceanGPUTime::Clipmap);
                // The textures kept between frames (made again for another size: then written whole).
                const bool bNew = !State->Static.IsValid() || !State->DisplacementWork.IsValid() || !State->NormalWork.IsValid() || !State->WhitecapWork.IsValid()
                    || !State->StrainWork.IsValid() || !State->Strain0Work.IsValid() || !State->Strain1Work.IsValid() || !State->FoamHistory.IsValid() || State->Size != Size || State->Slots != Job->Slots;
                const bool bHistory = Job->FoamDecay > 0.0;
                FRDGTextureRef StaticTex;
                FRDGTextureRef DispWork;
                FRDGTextureRef NormalWork;
                FRDGTextureRef WhitecapWork;
                FRDGTextureRef StrainWork;
                FRDGTextureRef Strain0Work;
                FRDGTextureRef Strain1Work;
                FRDGTextureRef FoamHistory;
                if (bNew)
                {
                    StaticTex = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_A32B32G32R32F, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, 4 * Job->Slots), TEXT("Voxel.OceanClipStatic"));
                    DispWork = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_A32B32G32R32F, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, Job->Slots), TEXT("Voxel.OceanClipDisplacementWork"));
                    NormalWork = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_FloatRGBA, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, Job->Slots), TEXT("Voxel.OceanClipNormalWork"));
                    WhitecapWork = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_FloatRGBA, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, Job->Slots), TEXT("Voxel.OceanClipWhitecapWork"));
                    StrainWork = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_FloatRGBA, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, Job->Slots), TEXT("Voxel.OceanClipStrain"));
                    Strain0Work = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_FloatRGBA, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, Job->Slots), TEXT("Voxel.OceanClipStrain0"));
                    Strain1Work = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_FloatRGBA, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, Job->Slots), TEXT("Voxel.OceanClipStrain1"));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(StaticTex), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(DispWork), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(NormalWork), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(WhitecapWork), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(StrainWork), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Strain0Work), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(Strain1Work), FVector4f(0.0f, 0.0f, 0.0f, 0.0f));
                    FoamHistory = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(Size, Size), PF_R32_FLOAT, FClearValueBinding::Black,
                        TexCreate_ShaderResource | TexCreate_UAV, Job->Slots), TEXT("Voxel.OceanClipFoamHistory"));
                    AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(FoamHistory), FVector4f(-1.0f, -1.0f, -1.0f, -1.0f));
                    State->FoamHistory = GraphBuilder.ConvertToExternalTexture(FoamHistory);
                    for (double& T : State->SlotTime) T = -1.0;
                    State->bHistoryOn = false;
                    State->Static = GraphBuilder.ConvertToExternalTexture(StaticTex);
                    State->DisplacementWork = GraphBuilder.ConvertToExternalTexture(DispWork);
                    State->NormalWork = GraphBuilder.ConvertToExternalTexture(NormalWork);
                    State->WhitecapWork = GraphBuilder.ConvertToExternalTexture(WhitecapWork);
                    State->StrainWork = GraphBuilder.ConvertToExternalTexture(StrainWork);
                    State->Strain0Work = GraphBuilder.ConvertToExternalTexture(Strain0Work);
                    State->Strain1Work = GraphBuilder.ConvertToExternalTexture(Strain1Work);
                    State->FineHistory.SafeRelease();
                    State->Size = Size;
                    State->Slots = Job->Slots;
                    // Planned only in part (the plan's slots outlived the textures): nothing shown
                    // this frame, everything written again from the next.
                    if (!Job->bAllWhole)
                    {
                        State->bLost.store(true);
                        OceanGPUTime.End();
                        GraphBuilder.Execute();
                        Displace->bClip = false;
                        return;
                    }
                }
                else
                {
                    StaticTex = GraphBuilder.RegisterExternalTexture(State->Static);
                    DispWork = GraphBuilder.RegisterExternalTexture(State->DisplacementWork);
                    NormalWork = GraphBuilder.RegisterExternalTexture(State->NormalWork);
                    WhitecapWork = GraphBuilder.RegisterExternalTexture(State->WhitecapWork);
                    StrainWork = GraphBuilder.RegisterExternalTexture(State->StrainWork);
                    Strain0Work = GraphBuilder.RegisterExternalTexture(State->Strain0Work);
                    Strain1Work = GraphBuilder.RegisterExternalTexture(State->Strain1Work);
                    FoamHistory = GraphBuilder.RegisterExternalTexture(State->FoamHistory);
                    // Stage W3t: the history turned on again: it starts afresh (not a pattern from
                    // whenever it was turned off).
                    if (bHistory && !State->bHistoryOn)
                    {
                        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(FoamHistory), FVector4f(-1.0f, -1.0f, -1.0f, -1.0f));
                        for (double& T : State->SlotTime) T = -1.0;
                    }
                }

                // The static layer's rectangles.
                if (Job->Static.Num() > 0)
                {
                    FVoxelOceanDensityParameters DensityP;
                    VOP::FillDensity(GraphBuilder, *Cache, Density, Edits, Job->HalfFinest, Shore, Margin, DepthFootprintCm, Job->Radius, LandCache, DensityP);
                    FRDGTextureUAVRef StaticUAV = GraphBuilder.CreateUAV(StaticTex);
                    FRDGTextureUAVRef HistoryUAV = GraphBuilder.CreateUAV(FoamHistory);
                    for (const auto& SJ : Job->Static)
                    {
                        const VoxelOceanClipmap::FRect& R = SJ.Rect;
                        if (R.IsEmpty()) continue;
                        const FVector4f& Window = Job->Window[SJ.Level];
                        const FIntVector4& Mod = Job->WindowMod[SJ.Level];
                        FVoxelOceanClipStaticParameters* P = GraphBuilder.AllocParameters<FVoxelOceanClipStaticParameters>();
                        P->StaticOut = StaticUAV;
                        P->StaticFoamHistoryOut = HistoryUAV;
                        P->StaticFoamHistoryOn = SJ.bReseed ? 1u : 0u;
                        P->StaticSlot = (uint32)FMath::RoundToInt(Window.W);
                        P->StaticRectMin = FIntPoint(R.I0, R.J0);
                        P->StaticRectSize = FIntPoint(R.Width, R.Height);
                        P->StaticOriginMod = FIntPoint(Mod.X, Mod.Y);
                        P->StaticInWindow = FVector2f(Window.X, Window.Y);
                        P->StaticSpacing = Window.Z;
                        for (int32 r = 0; r < VoxelOceanClipmap::SharedRows; r++) P->StaticFrame[r] = Job->Rows[r];
                        P->StaticCameraLattice = Job->CameraLattice;
                        P->StaticCameraFrac = Job->CameraFrac;
                        P->StaticParams = W.Params;
                        P->StaticWave0 = W.Wave[0];
                        P->StaticWave1 = W.Wave[1];
                        P->StaticWave2 = W.Wave[2];
                        P->StaticCenter = W.Center;
                        P->StaticShape = W.Shape;
                        P->StaticFFT = bFFT ? W.FFT : FVector4f(0.0f, W.FFT.Y, 0.0f, 0.0f);
                        P->StaticRefHigh = W.RefHigh;
                        P->StaticRefLow = W.RefLow;
                        P->StaticRotation = W.Rotation;
                        P->StaticTiles = W.Tiles;
                        P->StaticStates = States;
                        P->StaticSlopeVar = W.SlopeVar;
                        P->StaticCharts = ChartsRHI;
                        P->StaticShore = ShoreRHI;
                        P->Density = DensityP;
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanClip static L%d %dx%d", SJ.Level, R.Width, R.Height),
                            TShaderMapRef<FVoxelOceanClipStaticCS>(ShaderMap), P,
                            FIntVector(FMath::DivideAndRoundUp(R.Width, 8), FMath::DivideAndRoundUp(R.Height, 8), 1));
                    }
                }

                // The textures the material and the vertices read (the last frame's copy: the foam's
                // history's neighbours, stage W3t).
                FRDGTextureRef DispDest = RegisterExternalTexture(GraphBuilder, DispRHI, TEXT("Voxel.OceanClipDisplacement"));
                FRDGTextureRef NormalDest = RegisterExternalTexture(GraphBuilder, NormalRHI, TEXT("Voxel.OceanClipNormal"));
                FRDGTextureRef WhitecapDest = RegisterExternalTexture(GraphBuilder, WhitecapRHI, TEXT("Voxel.OceanClipWhitecap"));
                FRDGTextureRef InfoDest = RegisterExternalTexture(GraphBuilder, InfoRHI, TEXT("Voxel.OceanClipInfo"));
                const bool bArraysMatch = DispDest->Desc.Format == PF_A32B32G32R32F && NormalDest->Desc.Format == PF_FloatRGBA
                    && DispDest->Desc.Extent == FIntPoint(Size, Size) && NormalDest->Desc.Extent == FIntPoint(Size, Size)
                    && (int32)DispDest->Desc.ArraySize >= Job->Slots && (int32)NormalDest->Desc.ArraySize >= Job->Slots
                    && WhitecapDest->Desc.Format == PF_FloatRGBA && WhitecapDest->Desc.Extent == FIntPoint(Size, Size) && (int32)WhitecapDest->Desc.ArraySize >= Job->Slots;

                // Stage W3t: each due level's step of the foam's history (the time since its slot was
                // last stepped, on the ocean's clock; its fade).
                // (Its clock only advances when it steps: a frame without it loses no fading.)
                const bool bHistoryStep = bHistory && bArraysMatch;
                FVector4f FoamStep[VoxelOceanClipmap::MaxLevels];
                for (int32 i = 0; i < VoxelOceanClipmap::MaxLevels; i++) FoamStep[i] = FVector4f(0.0f, 1.0f, 0.0f, 0.0f);
                for (int32 d = 0; d < Job->NumDue; d++)
                {
                    const int32 Level = Job->Due[d];
                    const int32 Slot = FMath::Clamp(FMath::RoundToInt(Job->Window[Level].W), 0, VoxelOceanClipmap::MaxLevels - 1);
                    const double Last = State->SlotTime[Slot];
                    const double Dt = Last < 0.0 ? 0.0 : FMath::Clamp(Job->OceanTime - Last, 0.0, 1.0);
                    if (bHistoryStep) State->SlotTime[Slot] = Job->OceanTime;
                    // (Stage W3v: the decay a rate, per second.)
                    const double Keep = bHistory ? FMath::Exp(-Dt * Job->FoamDecay) : 1.0;
                    // (Stage W3v: the growth per unit of source over the step, integrated exactly:
                    // (Grow / Decay) (1 - Keep); VoxelOceanFoam.ush.)
                    const double GrowDt = bHistory ? Job->FoamGrowth / Job->FoamDecay * (1.0 - Keep) : 0.0;
                    FoamStep[Level] = FVector4f((float)Dt, (float)Keep, (float)GrowDt, (float)Job->FoamShift[Level]);
                }
                State->bHistoryOn = bHistoryStep;

                // The waves of the levels due, into their slots; the info.
                FRDGTextureRef InfoWork = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2D(FIntPoint(VoxelOceanClipmap::InfoRows, 1), PF_A32B32G32R32F,
                    FClearValueBinding::Black, TexCreate_ShaderResource | TexCreate_UAV), TEXT("Voxel.OceanClipInfoWork"));
                {
                    FVoxelOceanClipWaveParameters* P = GraphBuilder.AllocParameters<FVoxelOceanClipWaveParameters>();
                    for (int32 i = 0; i < VoxelOceanClipmap::InfoRows; i++) P->ClipInfoRows[i] = Job->Rows[i];
                    for (int32 i = 0; i < VoxelOceanClipmap::MaxLevels / 4; i++)
                    {
                        auto At = [&](int32 j) { return j < Job->NumDue ? (uint32)Job->Due[j] : (uint32)VoxelOceanClipmap::MaxLevels; };
                        P->ClipDue[i] = FUintVector4(At(4 * i), At(4 * i + 1), At(4 * i + 2), At(4 * i + 3));
                    }
                    P->ClipDueCount = (uint32)Job->NumDue;
                    for (int32 i = 0; i < VoxelOceanClipmap::MaxLevels; i++)
                    {
                        P->ClipWindow[i] = Job->Window[i];
                        P->ClipWindowMod[i] = Job->WindowMod[i];
                    }
                    P->ClipStatic = StaticTex;
                    P->ClipDisplacementOut = GraphBuilder.CreateUAV(DispWork);
                    P->ClipNormalOut = GraphBuilder.CreateUAV(NormalWork);
                    P->ClipWhitecapOut = GraphBuilder.CreateUAV(WhitecapWork);
                    P->ClipStrainOut = GraphBuilder.CreateUAV(StrainWork);
                    P->ClipStrain0Out = GraphBuilder.CreateUAV(Strain0Work);
                    P->ClipStrain1Out = GraphBuilder.CreateUAV(Strain1Work);
                    P->ClipFoamHistory = GraphBuilder.CreateUAV(FoamHistory);
                    P->ClipWhitecapPrev = WhitecapDest;
                    for (int32 i = 0; i < VoxelOceanClipmap::MaxLevels; i++) P->ClipFoamStep[i] = FoamStep[i];
                    // (Its neighbours from the last frame's copy: only where that copy is this shape.)
                    P->ClipFoamHistoryOn = bHistoryStep ? 1u : 0u;
                    P->ClipFoamSurf = FVector4f((float)Job->SurfFoamDecay, (float)Job->FoamBreakup, (float)Job->FoamCut, 0.0f);
                    P->ClipInfoOut = GraphBuilder.CreateUAV(InfoWork);
                    P->ClipParams = W.Params;
                    P->ClipWave0 = W.Wave[0];
                    P->ClipWave1 = W.Wave[1];
                    P->ClipWave2 = W.Wave[2];
                    P->ClipCenter = W.Center;
                    P->ClipShape = W.Shape;
                    P->ClipFFT = bFFT ? W.FFT : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
                    P->ClipRefHigh = W.RefHigh;
                    P->ClipRefLow = W.RefLow;
                    P->ClipRotation = W.Rotation;
                    P->ClipTiles = W.Tiles;
                    P->ClipStates = States;
                    P->ClipSlopeVar = W.SlopeVar;
                    P->ClipDisplacement = bFFT ? Displacement.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
                    P->ClipDerivatives = bFFT ? Derivatives.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
                    P->ClipCharts = ChartsRHI;
                    P->ClipFoam = bFFT && FoamTexture ? FoamTexture.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
                    P->ClipFoamOn = bFFT && FoamTexture ? 1u : 0u;
                    // The rings' layout (stage W3f): the surf decides by their spacing at each point.
                    P->ClipRing = Job->bRings ? Job->RingLayout : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
                    P->ClipDisplacementSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
                    P->ClipDerivativesSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
                    P->ClipFoamSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
                    // (One layer at least: its first threads write the info.)
                    FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanClip waves %d levels of %d", Job->NumDue, Size),
                        TShaderMapRef<FVoxelOceanClipWaveCS>(ShaderMap), P,
                        FIntVector(FMath::DivideAndRoundUp(Size, 8), FMath::DivideAndRoundUp(Size, 8), FMath::Max(Job->NumDue, 1)));
                }
                // Into the textures the material and the vertices read (stage W3y: only when a level
                // was evaluated; else they hold what they did). Stage W8m: only the slots of the levels
                // evaluated (the others' work is as last copied), runs of neighbouring slots together:
                // the coarse levels are due in turn, so most frames copy four slots of twelve.
                if (bArraysMatch && Job->NumDue > 0)
                {
                    bool bSlotDue[VoxelOceanClipmap::MaxLevels] = {};
                    for (int32 d = 0; d < Job->NumDue; d++)
                    {
                        const int32 Slot = FMath::RoundToInt(Job->Window[Job->Due[d]].W);
                        if (Slot >= 0 && Slot < VoxelOceanClipmap::MaxLevels && Slot < Job->Slots) bSlotDue[Slot] = true;
                    }
                    for (int32 First = 0; First < Job->Slots; First++)
                    {
                        if (!bSlotDue[First]) continue;
                        int32 Count = 1;
                        while (First + Count < Job->Slots && bSlotDue[First + Count]) Count++;
                        FRHICopyTextureInfo Copy;
                        Copy.Size = FIntVector(Size, Size, 1);
                        Copy.SourceSliceIndex = (uint32)First;
                        Copy.DestSliceIndex = (uint32)First;
                        Copy.NumSlices = (uint32)Count;
                        AddCopyTexturePass(GraphBuilder, DispWork, DispDest, Copy);
                        AddCopyTexturePass(GraphBuilder, NormalWork, NormalDest, Copy);
                        AddCopyTexturePass(GraphBuilder, WhitecapWork, WhitecapDest, Copy);
                        First += Count - 1;
                    }
                }

                // Stage W4c: the foam's fine history, its levels due (after the buffer's step: it seeds
                // from it and is gated by the levels' mean source, this frame's).
                FRDGTextureRef FineHistory = nullptr;
                if (Job->FineCount > 0 && bHistoryStep && Has(FVoxelOceanClipFineFoamCS::GetStaticType()))
                {
                    const int32 FineSize = Size * VoxelOceanClipmap::FineFactor;
                    if (!State->FineHistory.IsValid() || State->FineHistory->GetDesc().Extent.X != FineSize)
                    {
                        FineHistory = GraphBuilder.CreateTexture(FRDGTextureDesc::Create2DArray(FIntPoint(FineSize, FineSize), PF_R32_FLOAT, FClearValueBinding::Black,
                            TexCreate_ShaderResource | TexCreate_UAV, VoxelOceanClipmap::FineMaxLevels), TEXT("Voxel.OceanClipFineHistory"));
                        AddClearUAVPass(GraphBuilder, GraphBuilder.CreateUAV(FineHistory), FVector4f(-1.0f, -1.0f, -1.0f, -1.0f));
                        State->FineHistory = GraphBuilder.ConvertToExternalTexture(FineHistory);
                        for (double& T : State->FineTime) T = -1.0;
                    }
                    else
                    {
                        FineHistory = GraphBuilder.RegisterExternalTexture(State->FineHistory);
                    }
                    FRDGTextureUAVRef FineUAV = GraphBuilder.CreateUAV(FineHistory);
                    for (int32 f = 0; f < Job->NumFineSteps; f++)
                    {
                        const auto& Step = Job->FineSteps[f];
                        const int32 Level = Step.Level;
                        const double Last = State->FineTime[Level];
                        const double Dt = Step.bWhole || Last < 0.0 ? 0.0 : FMath::Clamp(Job->OceanTime - Last, 0.0, 1.0);
                        State->FineTime[Level] = Job->OceanTime;
                        const double Keep = FMath::Exp(-Dt * Job->FoamDecay);
                        const double GrowDt = Job->FoamGrowth / Job->FoamDecay * (1.0 - Keep);
                        FVoxelOceanClipFineParameters* P = GraphBuilder.AllocParameters<FVoxelOceanClipFineParameters>();
                        for (int32 i = 0; i < VoxelOceanClipmap::InfoRows; i++) P->ClipInfoRows[i] = Job->Rows[i];
                        P->FineWindow = Job->Window[Level];
                        P->FineWindowMod = FIntVector4(Job->WindowMod[Level].X, Job->WindowMod[Level].Y, FMath::RoundToInt(Job->Window[Level].W), Level);
                        P->FineStep = FVector4f((float)Keep, (float)GrowDt, (float)VoxelOceanClipmap::FineFactor, (float)Job->FoamBreakup);
                        auto RectOf = [](const VoxelOceanClipmap::FRect& R) { return FIntVector4(R.I0, R.J0, R.IsEmpty() ? 0 : R.Width, R.Height); };
                        P->FineRect0 = RectOf(Step.Rect[0]);
                        P->FineRect1 = RectOf(Step.Rect[1]);
                        P->FineWhole = Step.bWhole ? 1u : 0u;
                        // Stage W5c: the surf's foam's keep over the step; the shore's static layer and
                        // the rings' layout (the composed sea, as the clipmap's).
                        P->FineSurf = FVector4f((float)FMath::Exp(-Dt * Job->SurfFoamDecay), 0.0f, 0.0f, 0.0f);
                        P->ClipStatic = StaticTex;
                        P->ClipRing = Job->bRings ? Job->RingLayout : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
                        P->FineHistory = FineUAV;
                        P->FineCoarseHistory = FoamHistory;
                        P->FineCoarseWhitecap = WhitecapWork;
                        P->FineStrain0 = Strain0Work;
                        P->FineStrain1 = Strain1Work;
                        P->FineHeldOn = Job->bFineHeld ? 1u : 0u;
                        P->ClipParams = W.Params;
                        P->ClipWave0 = W.Wave[0];
                        P->ClipWave1 = W.Wave[1];
                        P->ClipWave2 = W.Wave[2];
                        P->ClipCenter = W.Center;
                        P->ClipShape = W.Shape;
                        P->ClipFFT = bFFT ? W.FFT : FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
                        P->ClipRefHigh = W.RefHigh;
                        P->ClipRefLow = W.RefLow;
                        P->ClipRotation = W.Rotation;
                        P->ClipTiles = W.Tiles;
                        P->ClipStates = States;
                        P->ClipSlopeVar = W.SlopeVar;
                        P->ClipDisplacement = bFFT ? Displacement.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
                        P->ClipDerivatives = bFFT ? Derivatives.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
                        P->ClipCharts = ChartsRHI;
                        P->ClipFoam = bFFT && FoamTexture ? FoamTexture.GetReference() : GBlackArrayTexture->TextureRHI.GetReference();
                        P->ClipFoamOn = bFFT && FoamTexture ? 1u : 0u;
                        P->ClipDisplacementSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
                        P->ClipDerivativesSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
                        P->ClipFoamSampler = TStaticSamplerState<SF_Trilinear, AM_Wrap, AM_Wrap, AM_Clamp>::GetRHI();
                        FComputeShaderUtils::AddPass(GraphBuilder, RDG_EVENT_NAME("VoxelOceanClip fine foam L%d %d", Level, FineSize),
                            TShaderMapRef<FVoxelOceanClipFineFoamCS>(ShaderMap), P,
                            FIntVector(FMath::DivideAndRoundUp(FineSize, 8), FMath::DivideAndRoundUp(FineSize, 8), 1));
                    }
                    GraphBuilder.SetTextureAccessFinal(FineHistory, ERHIAccess::SRVMask);
                }
                if (InfoDest->Desc.Format == PF_A32B32G32R32F && InfoDest->Desc.Extent == FIntPoint(VoxelOceanClipmap::InfoRows, 1))
                {
                    FRHICopyTextureInfo Copy;
                    Copy.Size = FIntVector(VoxelOceanClipmap::InfoRows, 1, 1);
                    AddCopyTexturePass(GraphBuilder, InfoWork, InfoDest, Copy);
                }
                GraphBuilder.SetTextureAccessFinal(DispDest, ERHIAccess::SRVMask);
                GraphBuilder.SetTextureAccessFinal(NormalDest, ERHIAccess::SRVMask);
                GraphBuilder.SetTextureAccessFinal(WhitecapDest, ERHIAccess::SRVMask);
                GraphBuilder.SetTextureAccessFinal(StrainWork, ERHIAccess::SRVMask);
                GraphBuilder.SetTextureAccessFinal(InfoDest, ERHIAccess::SRVMask);
                // (Stage W5c: the static layer read outside this graph too, by the own pass.)
                GraphBuilder.SetTextureAccessFinal(StaticTex, ERHIAccess::SRVMask);
                // A destination of another shape (being made again): the vertices don't sample it.
                if (!bArraysMatch)
                {
                    OceanGPUTime.End();
                    GraphBuilder.Execute();
                    Displace->bClip = false;
                    return;
                }
            }
            GraphBuilder.Execute();
            // The pages' vertices sample the levels this frame, and the rings draw from them.
            for (int32 i = 0; i < VoxelOceanClipmap::InfoRows; i++) Displace->ClipRows[i] = Job->Rows[i];
            Displace->ClipDisplacement = DispRHI;
            Displace->ClipNormal = NormalRHI;
            Displace->ClipWhitecap = WhitecapRHI;
            Displace->ClipStrain = State->StrainWork.IsValid() ? State->StrainWork->GetRHI() : nullptr;
            Displace->ClipInfo = InfoRHI;
            // Stage W4c: the foam's fine history (its pooled texture outlives the frame: the state holds it).
            const bool bFine = Job->FineCount > 0 && State->FineHistory.IsValid() && State->bHistoryOn;
            Displace->ClipFineHistory = bFine ? State->FineHistory->GetRHI() : nullptr;
            Displace->ClipStatic = State->Static.IsValid() ? State->Static->GetRHI() : nullptr;
            Displace->ClipFineInfo = bFine ? FVector2f((float)Job->FineCount, (float)VoxelOceanClipmap::FineFactor) : FVector2f(0.0f, 0.0f);
            Displace->bClip = true;
            Displace->bRings = Job->bRings;
            Displace->RingCount = Job->RingCount;
            for (int32 i = 0; i < VoxelOceanClipmap::MaxLevels; i++)
            {
                Displace->RingLevel[i] = Job->RingLevel[i];
                Displace->RingInts[i] = Job->RingInts[i];
            }

        });
}

void FVoxelOceanPages::NotifyClipEdit(const FVector& Center, double Reach)
{
    check(IsInGameThread());
    if (!ClipPlan.IsValid() || Reach <= 0.0) return;
    // Many at once (a long stroke): one sphere around them all.
    if (ClipPlan->Edits.Num() >= 64)
    {
        FBox Box(ForceInit);
        for (const FVoxelOceanClipPlan::FEdit& E : ClipPlan->Edits) Box += FBox(E.Center - FVector(E.Reach), E.Center + FVector(E.Reach));
        Box += FBox(Center - FVector(Reach), Center + FVector(Reach));
        ClipPlan->Edits.Reset();
        ClipPlan->Edits.Add({ Box.GetCenter(), Box.GetExtent().Size() });
        return;
    }
    ClipPlan->Edits.Add({ Center, Reach });
}

void FVoxelOceanPages::InvalidateClip()
{
    check(IsInGameThread());
    if (!ClipPlan.IsValid()) return;
    for (FVoxelOceanClipPlan::FSlot& Slot : ClipPlan->Slots) Slot = FVoxelOceanClipPlan::FSlot();
    ClipPlan->Edits.Reset();
}

FString FVoxelOceanPages::TakeClipPerf()
{
    if (!ClipPlan.IsValid() || !ClipPlan->bHasFrame) return FString();
    const VoxelOceanClipmap::FFrame& F = ClipPlan->Frame;
    const FString Out = FString::Printf(TEXT("Clipmap: %d levels (spacing %.2f to %.0f m), %lld static texels written, %d re-anchors."), F.Count,
        F.Count > 0 ? F.Levels[0].Spacing * 0.01 : 0.0, F.Count > 0 ? F.Levels[F.Count - 1].Spacing * 0.01 : 0.0, (long long)ClipPlan->StaticTexels,
        ClipPlan->Reanchors);
    ClipPlan->StaticTexels = 0;
    ClipPlan->Reanchors = 0;
    if (Rings.IsValid() && Rings->Provider.IsValid())
    {
        // The rings' runs in the last second: drawn, and waiting for the clipmap or a stream.
        const uint32 Runs = Rings->Provider->Runs.exchange(0, std::memory_order_relaxed);
        const uint32 Drawn = Rings->Provider->DrawnRuns.exchange(0, std::memory_order_relaxed);
        const uint32 Skipped = Rings->Provider->SkippedRuns.exchange(0, std::memory_order_relaxed);
        return Out + FString::Printf(TEXT(" Rings%s: %u runs drew, %u waited for the clipmap, %u skipped (a stream or a shader missing)%s."),
            Rings->bOwnPass ? TEXT(" (own pass)") : TEXT(""), Drawn, Runs - Drawn, Skipped,
            !Rings->bOwnPass && Rings->Handle == 0 ? TEXT(", not registered yet") : TEXT(""));
    }
    return Out;
}

bool FVoxelOceanPages::CanDrawRings() const
{
    return ClipSettings.bRings && IsClipmapOn() && bWavesInCompute && VOP::RingHalf(ClipSettings.Size) > 0;
}

bool FVoxelOceanPages::IsRingsOn() const
{
    return Rings.IsValid();
}

TSharedPtr<IVoxelOceanRingDraw, ESPMode::ThreadSafe> FVoxelOceanPages::GetRingDraw() const
{
    if (!Rings.IsValid() || !Rings->bOwnPass || !Rings->Provider.IsValid()) return nullptr;
    return Rings->Provider;
}

void FVoxelOceanPages::SetRings(bool bOn)
{
    check(IsInGameThread());
    if (!bOn || !CanDrawRings())
    {
        DestroyRings();
        return;
    }
    if (Rings.IsValid()) return;
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!OwnerActor || !Root || !FApp::CanEverRender()) return;

    TUniquePtr<FVoxelOceanRingSet> Set = MakeUnique<FVoxelOceanRingSet>();
    Set->Half = VOP::RingHalf(ClipSettings.Size);
    Set->Slots = ClipSettings.Levels;
    Set->FarCells = FMath::Clamp(ClipSettings.FarCells, 4, 256);
    const int32 Side = VOP::RingSide(Set->Half);
    const int32 RingVertices = Set->Slots * Side * Side;
    VOP::FarDirections(Set->FarCells, Set->FarDirections);
    Set->FarCellCm = Radius * UE_DOUBLE_PI * 0.5 / Set->FarCells;
    const int32 FarSide = Set->FarCells + 1;
    const int32 Vertices = RingVertices + Set->FarDirections.Num();

    // The triangles, fixed: each level's square, then the far sphere's faces.
    TArray<uint32> Indices;
    Indices.Reserve(6 * (Set->Slots * 4 * Set->Half * Set->Half + 6 * Set->FarCells * Set->FarCells));
    for (int32 Level = 0; Level < Set->Slots; Level++)
    {
        const uint32 Base = (uint32)(Level * Side * Side);
        auto Index = [&](int32 X, int32 Y) { return Base + (uint32)((Y + Set->Half) * Side + (X + Set->Half)); };
        for (int32 Y = -Set->Half; Y < Set->Half; Y++)
            for (int32 X = -Set->Half; X < Set->Half; X++)
                VOP::CellTriangles(Index(X, Y), Index(X + 1, Y), Index(X, Y + 1), Index(X + 1, Y + 1), (X >= 0) == (Y >= 0), bFlipWinding, Indices);
    }
    for (int32 Face = 0; Face < 6; Face++)
    {
        const uint32 Base = (uint32)(RingVertices + Face * FarSide * FarSide);
        auto Index = [&](int32 I, int32 J) { return Base + (uint32)(J * FarSide + I); };
        for (int32 J = 0; J < Set->FarCells; J++)
            for (int32 I = 0; I < Set->FarCells; I++)
                VOP::CellTriangles(Index(I, J), Index(I + 1, J), Index(I, J + 1), Index(I + 1, J + 1), true, bFlipWinding, Indices);
    }
    TArray<FVector4f> FarWords;
    FarWords.SetNum(Set->FarDirections.Num());
    for (int32 i = 0; i < FarWords.Num(); i++)
        FarWords[i] = FVector4f((float)Set->FarDirections[i].X, (float)Set->FarDirections[i].Y, (float)Set->FarDirections[i].Z, 0.0f);

    // Stage W3: the ocean's own pass draws them from buffers of their own; no mesh.
    if (ClipSettings.bOwnPass)
    {
        const uint32 IndexCount = (uint32)Indices.Num();
        Set->bOwnPass = true;
        Set->Provider = MakeShared<FVoxelOceanRingProvider, ESPMode::ThreadSafe>(InputCache, DisplaceState, ClipState, Set->Half, Set->Slots, Set->FarCells,
            MoveTemp(FarWords), IndexCount, MoveTemp(Indices));
        Set->CreatedFrame = GFrameCounter;
        Rings = MoveTemp(Set);
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Rings for the ocean's own pass: %d levels of %d x %d cells, a far sphere of %d cells a face edge (%d vertices, %u triangles)."),
            Rings->Slots, 2 * Rings->Half, 2 * Rings->Half, Rings->FarCells, Vertices, IndexCount / 3);
        return;
    }

    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetupMaterialSlot(0, "Ocean Material");
    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->bHiddenInSceneCapture = true;
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->SetCastShadow(false);
    Component->RegisterComponent();
    Component->SetMaterial(0, Material.Get());
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetRealtimeMesh(Mesh);
    Component->SetVisibleInRayTracing(false);
    Component->SetCustomPrimitiveDataFloat(0, 0.0f);
    Component->SetCustomPrimitiveDataFloat(1, (float)MorphEnd0);

    // The uploaded positions only size the buffers and give the bounds: the whole planet around
    // the origin (the provider writes them every frame).
    FRealtimeMeshStreamSet Streams;
    FRealtimeMeshStream& Positions = Streams.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
    Positions.SetNumZeroed(Vertices);
    TArrayView<FVector3f> PositionView = Positions.GetArrayView<FVector3f>();
    PositionView[0] = FVector3f((float)(-2.0 * Radius));
    PositionView[1] = FVector3f((float)(2.0 * Radius));
    Streams.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>()).SetNumZeroed(Vertices);
    Streams.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<TRealtimeMeshTexCoords<FVector2f, VOP::TexCoordChannels>>()).SetNumZeroed(Vertices);
    Streams.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>()).SetNumZeroed(Vertices);
    FRealtimeMeshStream& Triangles = Streams.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>());
    Triangles.SetNumUninitialized(Indices.Num() / 3);
    TArrayView<TIndex3<uint32>> TriangleView = Triangles.GetArrayView<TIndex3<uint32>>();
    for (int32 t = 0; t < TriangleView.Num(); t++)
        TriangleView[t] = TIndex3<uint32>(Indices[3 * t], Indices[3 * t + 1], Indices[3 * t + 2]);

    Set->bBuffersReady = MakeShared<std::atomic<bool>, ESPMode::ThreadSafe>(false);
    const TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> Ready = Set->bBuffersReady;
    Mesh->CreateBufferSet(VOP::GroupKey(), MoveTemp(Streams), FRealtimeMeshBufferSetConfig(ERealtimeMeshSectionDrawType::Static, true))
        .Next([Ready](ERealtimeMeshProxyUpdateStatus) { Ready->store(true, std::memory_order_release); });
    Mesh->UpdateSectionConfig(VOP::SectionKey(), FRealtimeMeshSectionConfig(0), false);
    // Hidden until the provider is registered (as the pages: CreateInstance).
    Mesh->SetSectionVisibility(VOP::SectionKey(), false);

    Set->Mesh = Mesh;
    Set->Component = Component;
    Set->Provider = MakeShared<FVoxelOceanRingProvider, ESPMode::ThreadSafe>(InputCache, DisplaceState, ClipState, Set->Half, Set->Slots, Set->FarCells,
        MoveTemp(FarWords), (uint32)Indices.Num(), TArray<uint32>());
    Set->CreatedFrame = GFrameCounter;
    Rings = MoveTemp(Set);
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelOcean] Rings: %d levels of %d x %d cells, a far sphere of %d cells a face edge (%d vertices, %d triangles)."),
        Rings->Slots, 2 * Rings->Half, 2 * Rings->Half, Rings->FarCells, Vertices, Indices.Num() / 3);
}

void FVoxelOceanPages::DestroyRings()
{
    if (!Rings.IsValid()) return;
    if (Rings->Handle != 0)
        UnregisterRealtimeMeshComputeProvider(Rings->Handle);
    if (URealtimeMeshComponent* Component = Rings->Component.Get())
        Component->DestroyComponent();
    // Its GPU buffers are released on the render thread, after any run already queued.
    if (Rings->Provider.IsValid())
    {
        ENQUEUE_RENDER_COMMAND(VoxelOceanRingsRelease)([Provider = MoveTemp(Rings->Provider)](FRHICommandListImmediate&) mutable
            {
                Provider.Reset();
            });
    }
    Rings.Reset();
}

FVector FVoxelOceanPages::AdvanceRings(const FVector& CameraPlanet)
{
    check(IsInGameThread() && Rings.IsValid());
    FVoxelOceanRingSet& Set = *Rings;
    URealtimeMeshSimple* Mesh = Set.Mesh.Get();
    URealtimeMeshComponent* Component = Set.Component.Get();
    if (!Set.bOwnPass && (!Mesh || !Component)) return FVector::ZeroVector;
    // Registered once the buffers exist and the component has a proxy (as the pages: Advance);
    // every frame from then.
    if (!Set.bOwnPass && Set.Handle == 0 && Set.bBuffersReady->load(std::memory_order_acquire) && GFrameCounter >= Set.CreatedFrame + 2)
    {
        Mesh->SetSectionVisibility(VOP::SectionKey(), true);
        FRealtimeMeshComputeRegistration Registration;
        Registration.Mesh = Mesh->GetMesh();
        Registration.BufferSetKey = VOP::GroupKey();
        Registration.SectionKey = VOP::SectionKey();
        Registration.Provider = Set.Provider;
        Registration.Trigger = ERealtimeMeshComputeTrigger::EveryFrame;
        Registration.bUseIndirect = true;
        Set.Handle = RegisterRealtimeMeshComputeProvider(Registration);
    }
    // The origin near the camera, so the positions are small floats: moved when the camera is more
    // than 2 km from it (the velocity reset: the move isn't motion).
    constexpr double RebaseCm = 200000.0;
    if (!Set.bHasOrigin || FVector::DistSquared(CameraPlanet, Set.Origin) > RebaseCm * RebaseCm)
    {
        Set.Origin = CameraPlanet;
        Set.bHasOrigin = true;
        // (The own pass places them from the origin itself, every view.)
        if (Component)
        {
            Component->SetRelativeTransform(FTransform(FQuat::Identity, Set.Origin, FVector::OneVector));
            Component->ResetSceneVelocity();
        }
    }
    // The far sphere's land: again when the terrain's inputs (or their lattice) changed.
    const double HalfFinest = FMath::Max(DensityInputs.HalfFinest, 1.0e-3);
    if (Set.Provider.IsValid() && (Set.FarVersion != DensityVersion || Set.FarHalfFinest != HalfFinest) && DensityInputs.Density.IsValid())
    {
        FVoxelOceanRingProvider::FFarJob Job;
        Job.Density = DensityInputs.Density;
        Job.Edits = DensityInputs.Edits.IsValid() ? DensityInputs.Edits : NoEdits;
        Job.HalfFinest = HalfFinest;
        Job.Shore = DensityInputs.Shore;
        Job.ShoreMarginCm = DensityInputs.ShoreMarginCm;
        Job.DepthFootprintCm = DensityInputs.DepthFootprintCm;
        Job.SeaRadiusCm = Radius;
        Job.LandCache = DensityInputs.LandCache;
        Job.FootprintCm = (float)Set.FarCellCm;
        Job.Lattice.SetNum(3 * Set.FarDirections.Num());
        for (int32 i = 0; i < Set.FarDirections.Num(); i++)
        {
            const FVector P = Set.FarDirections[i] * (Radius / HalfFinest);
            Job.Lattice[3 * i] = (int32)FMath::RoundToDouble(P.X);
            Job.Lattice[3 * i + 1] = (int32)FMath::RoundToDouble(P.Y);
            Job.Lattice[3 * i + 2] = (int32)FMath::RoundToDouble(P.Z);
        }
        ENQUEUE_RENDER_COMMAND(VoxelOceanFarJob)([Provider = Set.Provider, Job = MoveTemp(Job)](FRHICommandListImmediate&) mutable
            {
                Provider->SetFarJob_RenderThread(MoveTemp(Job));
            });
        Set.FarVersion = DensityVersion;
        Set.FarHalfFinest = HalfFinest;
    }
    return CameraPlanet - Set.Origin;
}

void FVoxelOceanPages::UpdateRings(const FVector& CameraPlanet, const FVector4f& Cull)
{
    check(IsInGameThread());
    if (!Rings.IsValid()) return;
    const FVector CamToOrigin = AdvanceRings(CameraPlanet);
    const bool bLevelColor = CVarVoxelDebugLevelColor.GetValueOnGameThread() > 0;
    // The camera as a float and the rest: the far sphere's points from it stay precise.
    const FVector3f High(CameraPlanet);
    const FVector Low = CameraPlanet - FVector(High);
    const FVector4f Frame0(FVector3f(CamToOrigin), bLevelColor ? 1.0f : 0.0f);
    const FVector4f Frame1(High, FMath::Max(Cull.W, 0.0f));
    const FVector4f Frame2(FVector3f(Low), (float)Radius);
    // The far sphere's cell for its own cull.
    const FVector4f RingCull(Cull.X, Cull.Y, Cull.Z, (float)Rings->FarCellCm);
    ENQUEUE_RENDER_COMMAND(VoxelOceanRingsFrame)([State = DisplaceState, Frame0, Frame1, Frame2, RingCull, Origin = Rings->Origin](FRHICommandListImmediate&)
        {
            State->bRingsFar = true;
            State->RingOrigin = Origin;
            State->RingFrame0 = Frame0;
            State->RingFrame1 = Frame1;
            State->RingFrame2 = Frame2;
            State->RingCull = RingCull;
        });
}

TSharedPtr<FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe> FVoxelOceanLandCacheRHI::Make(const FVoxelLandCacheTextures& Textures, double LandSeaRadiusCm,
    double OceanRadiusCm)
{
    if (!Textures.IsValid()) return nullptr;
    FTextureResource* Resources[4] = { Textures.Height->GameThread_GetRenderTargetResource(), Textures.Slope->GameThread_GetRenderTargetResource(),
        Textures.Fields->GameThread_GetRenderTargetResource(), Textures.State->GetResource() };
    for (FTextureResource* Resource : Resources)
        if (!Resource) return nullptr;
    const TSharedPtr<FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe> Out = MakeShared<FVoxelOceanLandCacheRHI, ESPMode::ThreadSafe>();
    Out->LandSeaRadiusCm = LandSeaRadiusCm;
    Out->SeaOffsetCm = OceanRadiusCm - LandSeaRadiusCm;
    Out->Sources[0] = Textures.Height;
    Out->Sources[1] = Textures.Slope;
    Out->Sources[2] = Textures.Fields;
    Out->Sources[3] = Textures.State;
    // After the resources' own initialization (render commands run in order).
    ENQUEUE_RENDER_COMMAND(VoxelOceanLandCacheRHI)([Out, H = Resources[0], S = Resources[1], F = Resources[2], T = Resources[3]](FRHICommandListImmediate&)
        {
            Out->Height = H->TextureRHI;
            Out->Slope = S->TextureRHI;
            Out->Fields = F->TextureRHI;
            Out->State = T->TextureRHI;
        });
    return Out;
}

bool FVoxelOceanLandCacheRHI::IsFor(const FVoxelLandCacheTextures& Textures, double InLandSeaRadiusCm, double OceanRadiusCm) const
{
    // Weak: a texture made again at a freed one's address is another texture.
    return Sources[0].Get() == Textures.Height && Sources[1].Get() == Textures.Slope && Sources[2].Get() == Textures.Fields
        && Sources[3].Get() == Textures.State && Textures.IsValid() && LandSeaRadiusCm == InLandSeaRadiusCm && SeaOffsetCm == OceanRadiusCm - InLandSeaRadiusCm;
}

void FVoxelOceanPages::Initialize(AActor* InOwner, USceneComponent* InAttachRoot, UMaterialInterface* InMaterial, double InRadius, int32 InTileQuads,
    double InMorphEnd0, bool bInFlipWinding)
{
    Reset();
    DestroyRings();
    Owner = InOwner;
    AttachRoot = InAttachRoot;
    Material = InMaterial;
    Radius = InRadius;
    TileQuads = InTileQuads;
    MorphEnd0 = InMorphEnd0;
    bFlipWinding = bInFlipWinding;
}

void FVoxelOceanPages::Reset()
{
    for (TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
        DestroyInstance(Pair.Value.Instance);
    Pages.Reset();
    for (FInstance& Instance : Pool)
        DestroyInstance(Instance);
    Pool.Reset();
    for (FReleasing& Entry : Releasing)
        DestroyInstance(Entry.Instance);
    Releasing.Reset();
}

void FVoxelOceanPages::SetMaterial(UMaterialInterface* InMaterial)
{
    Material = InMaterial;
    auto Apply = [InMaterial](const FInstance& Instance)
        {
            if (URealtimeMeshComponent* Component = Instance.Component.Get())
                Component->SetMaterial(0, InMaterial);
        };
    for (const TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
        Apply(Pair.Value.Instance);
    for (const FInstance& Instance : Pool)
        Apply(Instance);
    for (const FReleasing& Entry : Releasing)
        Apply(Entry.Instance);
    if (Rings.IsValid())
        if (URealtimeMeshComponent* Component = Rings->Component.Get())
            Component->SetMaterial(0, InMaterial);
}

void FVoxelOceanPages::SetWaveReach(double InReach)
{
    if (InReach == WaveReach) return;
    WaveReach = FMath::Max(InReach, 0.0);
    for (TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
        PlaceInstance(Pair.Key, Pair.Value.Instance);
}

void FVoxelOceanPages::SetMorphEnd0(double InMorphEnd0)
{
    MorphEnd0 = InMorphEnd0;
    auto Apply = [this](const FInstance& Instance)
        {
            if (URealtimeMeshComponent* Component = Instance.Component.Get())
                Component->SetCustomPrimitiveDataFloat(1, (float)MorphEnd0);
        };
    for (const TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
        Apply(Pair.Value.Instance);
    for (const FInstance& Instance : Pool)
        Apply(Instance);
    for (const FReleasing& Entry : Releasing)
        Apply(Entry.Instance);
}

void FVoxelOceanPages::PackVertex(const VoxelOceanTiles::FTileVertex& Vertex, const FIntVector& Lattice, float Coast, uint32* OutWords)
{
    OutWords[0] = VOP::FloatBits(Vertex.Position.X);
    OutWords[1] = VOP::FloatBits(Vertex.Position.Y);
    OutWords[2] = VOP::FloatBits(Vertex.Position.Z);
    OutWords[3] = VOP::FloatBits(Vertex.Morph.X);
    OutWords[4] = VOP::FloatBits(Vertex.Morph.Y);
    OutWords[5] = VOP::FloatBits(Vertex.Morph.Z);
    OutWords[6] = VOP::PackSnorm8(Vertex.Tangent, 1.0f);
    OutWords[7] = VOP::PackSnorm8(Vertex.Normal, 1.0f);
    OutWords[8] = VOP::FloatBits(Vertex.Grand.X);
    OutWords[9] = VOP::FloatBits(Vertex.Grand.Y);
    OutWords[10] = VOP::FloatBits(Vertex.Grand.Z);
    OutWords[11] = (uint32)Lattice.X;
    OutWords[12] = (uint32)Lattice.Y;
    OutWords[13] = (uint32)Lattice.Z;
    OutWords[14] = ((uint32)Vertex.TargetIndex & 0xffffu) | ((uint32)Vertex.GrandIndex << 16);
    OutWords[15] = VOP::FloatBits(Coast);
}

FVoxelOceanPages::FInstance FVoxelOceanPages::CreateInstance()
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanCreate);
    const double StartTime = FPlatformTime::Seconds();
    ON_SCOPE_EXIT
    {
        const double Ms = (FPlatformTime::Seconds() - StartTime) * 1000.0;
        Perf.Created++;
        Perf.CreateMs += Ms;
        Perf.MaxCreateMs = FMath::Max(Perf.MaxCreateMs, Ms);
    };
    FInstance Instance;
    AActor* OwnerActor = Owner.Get();
    USceneComponent* Root = AttachRoot.Get();
    if (!OwnerActor || !Root) return Instance;

    const int32 VertexCapacity = VoxelOceanTiles::PageSlots * VOP::TileVertices(TileQuads);
    const int32 TriangleCapacity = VoxelOceanTiles::PageSlots * VOP::TileTriangles(TileQuads);

    URealtimeMeshSimple* Mesh = NewObject<URealtimeMeshSimple>(OwnerActor, NAME_None, RF_Transient);
    Mesh->SetupMaterialSlot(0, "Ocean Material");
    URealtimeMeshComponent* Component = NewObject<URealtimeMeshComponent>(OwnerActor, URealtimeMeshComponent::StaticClass(), NAME_None, RF_Transient);
    Component->bHiddenInSceneCapture = true;
    Component->SetCanEverAffectNavigation(false);
    Component->SetGenerateOverlapEvents(false);
    Component->SetCastShadow(false);
    Component->RegisterComponent();
    Component->SetMaterial(0, Material.Get());
    Component->AttachToComponent(Root, FAttachmentTransformRules::KeepRelativeTransform);
    Component->SetMobility(EComponentMobility::Movable);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetRealtimeMesh(Mesh);
    Component->SetVisibleInRayTracing(false);
    // For the morph (VoxelOcean.ush): where level 0's morph ends in world units. Data 0, the
    // level, is set with the page (PlaceInstance).
    Component->SetCustomPrimitiveDataFloat(1, (float)MorphEnd0);

    // The uploaded streams only size the buffers; the provider writes them. Two corner
    // positions give the bounds: in page units a page's tiles and their morph stay within the
    // unit cube around its origin (its node's half-width is about 0.5, a face sags 0.27).
    FRealtimeMeshStreamSet Streams;
    FRealtimeMeshStream& Positions = Streams.AddStream(FRealtimeMeshStreams::Position, GetRealtimeMeshBufferLayout<FVector3f>());
    Positions.SetNumZeroed(VertexCapacity);
    TArrayView<FVector3f> PositionView = Positions.GetArrayView<FVector3f>();
    PositionView[0] = FVector3f(-1.0f);
    PositionView[1] = FVector3f(1.0f);
    // Nothing draws these before the provider writes them: no need to clear them here.
    Streams.AddStream(FRealtimeMeshStreams::Tangents, GetRealtimeMeshBufferLayout<FRealtimeMeshTangentsNormalPrecision>()).SetNumUninitialized(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::TexCoords, GetRealtimeMeshBufferLayout<TRealtimeMeshTexCoords<FVector2f, VOP::TexCoordChannels>>()).SetNumUninitialized(VertexCapacity);
    Streams.AddStream(FRealtimeMeshStreams::Color, GetRealtimeMeshBufferLayout<FColor>()).SetNumUninitialized(VertexCapacity);
    // Zeroed: slots never written draw degenerate triangles.
    Streams.AddStream(FRealtimeMeshStreams::Triangles, GetRealtimeMeshBufferLayout<TIndex3<uint32>>()).SetNumZeroed(TriangleCapacity);
    // No PolyGroups stream: the buffer set then makes one section over every vertex and index.

    Instance.bBuffersReady = MakeShared<std::atomic<bool>, ESPMode::ThreadSafe>(false);
    const TSharedPtr<std::atomic<bool>, ESPMode::ThreadSafe> Ready = Instance.bBuffersReady;
    Mesh->CreateBufferSet(VOP::GroupKey(), MoveTemp(Streams), FRealtimeMeshBufferSetConfig(ERealtimeMeshSectionDrawType::Static, true))
        .Next([Ready](ERealtimeMeshProxyUpdateStatus) { Ready->store(true, std::memory_order_release); });
    Mesh->UpdateSectionConfig(VOP::SectionKey(), FRealtimeMeshSectionConfig(0), false);
    // Hidden until the provider is registered: until then the section draws its whole (zero)
    // range. The component stays visible: a hidden component has no render proxy, and the
    // driver runs providers only for meshes with one.
    Mesh->SetSectionVisibility(VOP::SectionKey(), false);

    Instance.Mesh = Mesh;
    Instance.Component = Component;
    Instance.Provider = MakeShared<FVoxelOceanPageProvider, ESPMode::ThreadSafe>(InputCache, DisplaceState, bWavesInCompute);
    Instance.CreatedFrame = GFrameCounter;
    return Instance;
}

void FVoxelOceanPages::DestroyInstance(FInstance& Instance)
{
    if (Instance.Handle != 0)
        UnregisterRealtimeMeshComputeProvider(Instance.Handle);
    if (URealtimeMeshComponent* Component = Instance.Component.Get())
        Component->DestroyComponent();
    // The provider's GPU buffers (the static copy, stage O9j) are released on the render thread:
    // this reference is dropped there, after any run already queued.
    if (Instance.Provider.IsValid())
    {
        ENQUEUE_RENDER_COMMAND(VoxelOceanPageRelease)([Provider = MoveTemp(Instance.Provider)](FRHICommandListImmediate&) mutable
            {
                Provider.Reset();
            });
    }
    Instance = FInstance();
}

void FVoxelOceanPages::PlaceInstance(const FVoxelOceanTileKey& PageKey, FInstance& Instance) const
{
    URealtimeMeshComponent* Component = Instance.Component.Get();
    if (!Component) return;
    FVector Origin;
    double Unit = 1.0;
    VoxelOceanTiles::PageFrame(PageKey, Radius, Origin, Unit);
    Component->SetRelativeTransform(FTransform(FQuat::Identity, Origin, FVector(Unit)));
    Component->SetCustomPrimitiveDataFloat(0, (float)PageKey.Level);
    // The displacement's page (stage O9j): its origin, unit and level, before its next run.
    if (bWavesInCompute && Instance.Provider.IsValid())
    {
        ENQUEUE_RENDER_COMMAND(VoxelOceanPagePlace)([Provider = Instance.Provider, Origin, Unit, Level = (int32)PageKey.Level](FRHICommandListImmediate&)
            {
                Provider->SetPlacement_RenderThread(Origin, Unit, Level);
            });
    }
    // The node and its sag reach 0.77 of the unit cube's half-width (CreateInstance), leaving
    // 0.23 of a unit around them; the waves may need more.
    const float BoundsScale = (float)FMath::Max(1.0, (0.77 * Unit + WaveReach) / Unit);
    if (Component->BoundsScale != BoundsScale)
        Component->SetBoundsScale(BoundsScale);
}

bool FVoxelOceanPages::IsReady(const FInstance& Instance)
{
    return Instance.IsValid() && Instance.Handle != 0 && GFrameCounter > Instance.RegisteredFrame && GFrameCounter >= Instance.ReadyFrame;
}

void FVoxelOceanPages::BeginCreates()
{
    if (CreatesFrame == GFrameCounter) return;
    CreatesFrame = GFrameCounter;
    CreatesThisFrame = 0;
}

FVoxelOceanPages::FInstance FVoxelOceanPages::AcquireInstance(const FVoxelOceanTileKey& PageKey)
{
    while (Pool.Num() > 0)
    {
        FInstance Instance = Pool.Pop();
        if (!Instance.Component.IsValid() || !Instance.Mesh.IsValid())
        {
            DestroyInstance(Instance);
            continue;
        }
        // Moved now, filled from the next frame on, so the move reaches the render thread no
        // later than the new contents.
        PlaceInstance(PageKey, Instance);
        Instance.ReadyFrame = GFrameCounter + 1;
        Perf.Recycled++;
        return Instance;
    }
    if (CreatesThisFrame >= FMath::Max(1, CVarVoxelOceanPageCreates.GetValueOnGameThread()))
        return FInstance();
    CreatesThisFrame++;
    // A new one is registered with empty contents, so it is ready when a commit needs it.
    FInstance Instance = CreateInstance();
    if (Instance.IsValid())
    {
        PlaceInstance(PageKey, Instance);
        Push(PageKey, Instance, TArray<uint32>(), TArray<uint32>(), 0);
    }
    return Instance;
}

void FVoxelOceanPages::PoolInstance(FInstance& Instance)
{
    if (!Instance.IsValid()) return;
    if (!Instance.Component.IsValid() || Pool.Num() >= VOP::MaxPooled)
    {
        DestroyInstance(Instance);
        return;
    }
    Pool.Add(MoveTemp(Instance));
    Instance = FInstance();
}

bool FVoxelOceanPages::PreparePages(const TSet<FVoxelOceanTileKey>& InPages)
{
    BeginCreates();
    bool bReady = true;
    for (const FVoxelOceanTileKey& PageKey : InPages)
    {
        FPage& Page = Pages.FindOrAdd(PageKey);
        Page.WantedFrame = GFrameCounter;
        if (!Page.Instance.IsValid())
            Page.Instance = AcquireInstance(PageKey);
        bReady &= IsReady(Page.Instance);
    }
    return bReady;
}

bool FVoxelOceanPages::IsPageReady(const FVoxelOceanTileKey& PageKey) const
{
    const FPage* Page = Pages.Find(PageKey);
    return Page && IsReady(Page->Instance);
}

int32 FVoxelOceanPages::FindSlot(const FPage& Page, const FVoxelOceanTileKey& Tile)
{
    for (int32 s = 0; s < VoxelOceanTiles::PageSlots; s++)
    {
        if (Page.Slots[s] == Tile) return s;
    }
    return INDEX_NONE;
}

int32 FVoxelOceanPages::DrawSlotsOf(const FPage& Page)
{
    for (int32 s = VoxelOceanTiles::PageSlots - 1; s >= 0; s--)
    {
        if (Page.Slots[s] != FreeSlot()) return s + 1;
    }
    return 0;
}

void FVoxelOceanPages::Commit(TConstArrayView<FTileWrite> Writes, TConstArrayView<FVoxelOceanTileKey> Clears)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanCommit);
    struct FPageWork
    {
        uint32 Cleared = 0;            // slots emptied by this commit and not refilled
        TArray<uint32> Jobs;
        TArray<uint32> Words;
    };
    TMap<FVoxelOceanTileKey, FPageWork> Work;

    // Departing tiles free their slots first, so new tiles of the same commit can take them.
    for (const FVoxelOceanTileKey& Tile : Clears)
    {
        const FVoxelOceanTileKey PageKey = VoxelOceanTiles::PageOf(Tile);
        FPage* Page = Pages.Find(PageKey);
        const int32 Slot = Page ? FindSlot(*Page, Tile) : INDEX_NONE;
        if (Slot == INDEX_NONE) continue;
        Page->Slots[Slot] = FreeSlot();
        Page->NumTiles--;
        Work.FindOrAdd(PageKey).Cleared |= 1u << Slot;
        Perf.TilesCleared++;
    }

    // New tiles: the lowest free slot (a tile already held is rewritten in place). A slot
    // freed and refilled in this commit is only written: one job per slot, since the jobs of
    // a run go in parallel.
    for (const FTileWrite& Write : Writes)
    {
        const FVoxelOceanTileKey PageKey = VoxelOceanTiles::PageOf(Write.Tile);
        FPage* Page = Pages.Find(PageKey);
        if (!Page || !Page->Instance.IsValid())
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] A tile was committed to a page without a buffer set; it is not drawn until it changes."));
            continue;
        }
        int32 Slot = FindSlot(*Page, Write.Tile);
        if (Slot == INDEX_NONE)
        {
            Slot = FindSlot(*Page, FreeSlot());
            if (Slot == INDEX_NONE)
            {
                UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] A page has no free slot for a tile; it is not drawn."));
                continue;
            }
            Page->Slots[Slot] = Write.Tile;
            Page->NumTiles++;
        }
        FPageWork& PageWork = Work.FindOrAdd(PageKey);
        PageWork.Cleared &= ~(1u << Slot);
        PageWork.Jobs.Append({ 0u, (uint32)Slot, (uint32)PageWork.Words.Num(), VOP::FloatBits(Write.Footprint), VOP::FloatBits(Write.CullHeight),
            VOP::FloatBits(Write.Nominal) });
        PageWork.Words.Append(Write.Words);
        Perf.TilesWritten++;
    }

    for (TPair<FVoxelOceanTileKey, FPageWork>& Pair : Work)
    {
        FPage* Page = Pages.Find(Pair.Key);
        if (!Page) continue;
        FPageWork& PageWork = Pair.Value;
        for (int32 s = 0; s < VoxelOceanTiles::PageSlots; s++)
        {
            if (PageWork.Cleared & (1u << s))
                PageWork.Jobs.Append({ 1u, (uint32)s, 0u, 0u, 0u, 0u });
        }
        Push(Pair.Key, Page->Instance, MoveTemp(PageWork.Jobs), MoveTemp(PageWork.Words), DrawSlotsOf(*Page));
        Perf.PagesPushed++;
        if (Page->NumTiles > 0) continue;
        // Emptied by this run: pooled once it is recorded.
        FReleasing& Entry = Releasing.AddDefaulted_GetRef();
        Entry.Instance = MoveTemp(Page->Instance);
        Entry.Version = Entry.Instance.PushedVersion;
        Entry.Frame = GFrameCounter;
        Pages.Remove(Pair.Key);
    }
    Perf.Commits++;
}

void FVoxelOceanPages::Push(const FVoxelOceanTileKey& PageKey, FInstance& Instance, TArray<uint32>&& Jobs, TArray<uint32>&& Words, int32 DrawSlots)
{
    if (!Instance.IsValid()) return;
    FVoxelOceanPageProvider::FState State;
    State.Jobs = MoveTemp(Jobs);
    State.Words = MoveTemp(Words);
    State.DrawSlots = (uint32)FMath::Max(0, DrawSlots);
    State.TileQuads = (uint32)TileQuads;
    State.Color = (CVarVoxelDebugLevelColor.GetValueOnGameThread() > 0 ? FVoxelBrickTerrain::LevelColor(PageKey.Level) : FColor::White).DWColor();
    State.FlipWinding = bFlipWinding ? 1u : 0u;
    State.Density = DensityInputs.Density.IsValid() ? DensityInputs.Density : NoDensity;
    State.Edits = DensityInputs.Edits.IsValid() ? DensityInputs.Edits : NoEdits;
    State.HalfFinest = DensityInputs.HalfFinest;
    State.Shore = DensityInputs.Shore;
    State.ShoreMarginCm = DensityInputs.ShoreMarginCm;
    State.DepthFootprintCm = DensityInputs.DepthFootprintCm;
    State.SeaRadiusCm = Radius;
    State.LandCache = DensityInputs.LandCache;
    State.Version = NextVersion++;
    Instance.PushedVersion = State.Version;
    // The state reaches the provider before the dirty mark: both are render commands.
    ENQUEUE_RENDER_COMMAND(VoxelOceanPagePush)([Provider = Instance.Provider, State = MoveTemp(State)](FRHICommandListImmediate&) mutable
        {
            Provider->AddState_RenderThread(MoveTemp(State));
        });
    if (Instance.Handle != 0)
    {
        MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
        Instance.LastKickFrame = GFrameCounter;
    }
}

void FVoxelOceanPages::Advance(FInstance& Instance) const
{
    if (!Instance.IsValid()) return;
    URealtimeMeshSimple* Mesh = Instance.Mesh.Get();
    if (!Mesh) return;

    if (Instance.Handle == 0)
    {
        // Registered once the buffers exist and the component has a proxy (two frames after
        // its creation); the driver runs an OnDemand provider when registered.
        if (!Instance.bBuffersReady->load(std::memory_order_acquire) || GFrameCounter < Instance.CreatedFrame + 2 || Instance.PushedVersion == 0)
            return;
        // Shown first: the driver binds the indirect args to the section proxy, and a later
        // section update from the data side would rebuild that proxy without them.
        Mesh->SetSectionVisibility(VOP::SectionKey(), true);
        FRealtimeMeshComputeRegistration Registration;
        Registration.Mesh = Mesh->GetMesh();
        Registration.BufferSetKey = VOP::GroupKey();
        Registration.SectionKey = VOP::SectionKey();
        Registration.Provider = Instance.Provider;
        // With the waves in compute (stage O9j) the provider runs every frame: its writes, then
        // the displacement.
        Registration.Trigger = bWavesInCompute ? ERealtimeMeshComputeTrigger::EveryFrame : ERealtimeMeshComputeTrigger::OnDemand;
        Registration.bUseIndirect = true;
        Instance.Handle = RegisterRealtimeMeshComputeProvider(Registration);
        Instance.LastKickFrame = GFrameCounter;
        Instance.RegisteredFrame = GFrameCounter;
        return;
    }

    // A run that didn't happen (no proxy yet, shader still compiling) is asked for again.
    if (Instance.Provider->ShownVersion.load(std::memory_order_acquire) < Instance.PushedVersion && GFrameCounter > Instance.LastKickFrame + VOP::KickFrames)
    {
        MarkRealtimeMeshComputeProviderDirty(Instance.Handle);
        Instance.LastKickFrame = GFrameCounter;
    }
}

void FVoxelOceanPages::CollectCounters(const FInstance& Instance)
{
    if (!Instance.IsValid()) return;
    FVoxelOceanPageProvider& Provider = *Instance.Provider;
    Perf.Calls += Provider.Calls.exchange(0, std::memory_order_relaxed);
    Perf.IdleCalls += Provider.IdleCalls.exchange(0, std::memory_order_relaxed);
    Perf.Runs += Provider.PerfRuns.exchange(0, std::memory_order_relaxed);
    Perf.RenderMicros += Provider.RenderMicros.exchange(0, std::memory_order_relaxed);
}

void FVoxelOceanPages::Flush(int32 SpareTarget)
{
    SCOPE_CYCLE_COUNTER(STAT_VoxelOceanFlush);
    Perf.Frames++;
    BeginCreates();

    // Emptied buffer sets, pooled once their emptying run is recorded.
    for (int32 r = 0; r < Releasing.Num(); r++)
    {
        FReleasing& Entry = Releasing[r];
        Advance(Entry.Instance);
        CollectCounters(Entry.Instance);
        if (Entry.Instance.IsValid() && Entry.Instance.Provider->ShownVersion.load(std::memory_order_acquire) >= Entry.Version)
            PoolInstance(Entry.Instance);
        else if (!Entry.Instance.IsValid() || GFrameCounter > Entry.Frame + VOP::EmptyingTimeoutFrames)
            DestroyInstance(Entry.Instance);
        else
            continue;
        Releasing.RemoveAtSwap(r--);
    }

    // Registration and kicks; empty pages no commit wanted lately go back to the pool.
    TArray<FVoxelOceanTileKey> Unused;
    for (TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
    {
        FPage& Page = Pair.Value;
        Advance(Page.Instance);
        CollectCounters(Page.Instance);
        const uint32 MissingBits = Page.Instance.IsValid() ? Page.Instance.Provider->Missing.load(std::memory_order_relaxed) : 0u;
        if (MissingBits != 0 && (MissingBits & ~WarnedMissing) != 0)
        {
            UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelOcean] An ocean page's compute run is missing: %s%s."), *VOP::DescribeMissing(MissingBits),
                (MissingBits & (uint32)VOP::Required) ? TEXT(" (run skipped, retried)") : TEXT(" (written elsewhere, run continues)"));
            WarnedMissing |= MissingBits;
        }
        if (Page.NumTiles == 0 && GFrameCounter > Page.WantedFrame + VOP::UnusedFrames)
            Unused.Add(Pair.Key);
    }
    for (const FVoxelOceanTileKey& Key : Unused)
    {
        PoolInstance(Pages[Key].Instance);
        Pages.Remove(Key);
    }

    // Spare buffer sets, registered ahead so commits don't wait for creation.
    for (FInstance& Instance : Pool)
    {
        Advance(Instance);
        CollectCounters(Instance);
    }
    const int32 Budget = FMath::Max(1, CVarVoxelOceanPageCreates.GetValueOnGameThread());
    while (Pool.Num() < FMath::Min(SpareTarget, VOP::MaxPooled) && CreatesThisFrame < Budget)
    {
        CreatesThisFrame++;
        FInstance Instance = CreateInstance();
        if (!Instance.IsValid()) break;
        Push(FVoxelOceanTileKey(), Instance, TArray<uint32>(), TArray<uint32>(), 0);
        Pool.Add(MoveTemp(Instance));
    }

    SET_DWORD_STAT(STAT_VoxelOceanPages, Pages.Num());
    SET_DWORD_STAT(STAT_VoxelOceanPooled, Pool.Num());
}

FString FVoxelOceanPages::Describe() const
{
    int32 Tiles = 0, Unregistered = 0, Lagging = 0;
    uint32 Runs = 0, Skipped = 0, MissingBits = 0;
    auto Count = [&](const FInstance& Instance)
        {
            if (!Instance.IsValid()) return;
            Unregistered += Instance.Handle == 0 ? 1 : 0;
            Lagging += Instance.Provider->ShownVersion.load(std::memory_order_relaxed) < Instance.PushedVersion ? 1 : 0;
            Runs += Instance.Provider->Runs.load(std::memory_order_relaxed);
            Skipped += Instance.Provider->SkippedRuns.load(std::memory_order_relaxed);
            MissingBits |= Instance.Provider->Missing.load(std::memory_order_relaxed);
        };
    for (const TPair<FVoxelOceanTileKey, FPage>& Pair : Pages)
    {
        Tiles += Pair.Value.NumTiles;
        Count(Pair.Value.Instance);
    }
    for (const FInstance& Instance : Pool)
        Count(Instance);
    const int32 Instances = Pages.Num() + Pool.Num() + Releasing.Num();
    const double MB = Instances * (double)VoxelOceanTiles::PageSlots
        * (VOP::TileVertices(TileQuads) * VOP::BytesPerVertex + VOP::TileTriangles(TileQuads) * VOP::BytesPerTriangle) / (1024.0 * 1024.0);
    return FString::Printf(TEXT("Ocean pages: %d (%d tiles), %d pooled, %d being emptied, %.1f MB of buffers; %d not yet registered, %d not yet drawing their newest state; %u runs, %u skipped%s%s"),
        Pages.Num(), Tiles, Pool.Num(), Releasing.Num(), MB, Unregistered, Lagging, Runs, Skipped,
        MissingBits ? TEXT(", missing: ") : TEXT(""), *VOP::DescribeMissing(MissingBits));
}

FString FVoxelOceanPages::TakePerf()
{
    const double PerFrame = 1.0 / FMath::Max(1, Perf.Frames);
    const FString Out = FString::Printf(TEXT("Ocean pages per frame: %.2f commits, %.1f tiles written, %.1f cleared, %.1f pages pushed; providers %.1f calls (%.1f idle), %.2f runs, %.3f ms render thread; buffer sets %.2f created (%.2f ms each, %.2f ms max), %.2f recycled."),
        Perf.Commits * PerFrame, Perf.TilesWritten * PerFrame, Perf.TilesCleared * PerFrame, Perf.PagesPushed * PerFrame,
        Perf.Calls * PerFrame, Perf.IdleCalls * PerFrame, Perf.Runs * PerFrame, Perf.RenderMicros * PerFrame / 1000.0,
        Perf.Created * PerFrame, Perf.Created > 0 ? Perf.CreateMs / Perf.Created : 0.0, Perf.MaxCreateMs, Perf.Recycled * PerFrame);
    Perf = FPerf();
    const FString Clip = TakeClipPerf();
    return Clip.IsEmpty() ? Out : Out + TEXT(" ") + Clip;
}
