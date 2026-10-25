// VoxelOceanRingDraw.h - The rings for the ocean's own pass (stage W3; Design/OceanWaves.md
// section 30). With the own pass on, the rings (stage W2) have no mesh component: their vertices
// go to buffers of their own (VoxelOceanRings.usf, once a frame), and the pass
// (FVoxelOceanSurfacePass) draws them with the frame's waves and wave clipmap, the material's own
// inputs, so the surface it shades is the one the material shaded.

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphDefinitions.h"
#include "RHIResources.h"
#include "VoxelOceanPages.h"
#include "VoxelOceanClipmap.h"

/** One frame of the rings, in the caller's graph (render thread). */
struct FVoxelOceanRingDrawFrame
{
    /** The vertices (VoxelOceanRings.usf): the displaced position from the origin (planet axes, cm;
     *  3 floats, Buffer<float>), TexWords words of what the pixel takes (Buffer<uint>), the colour
     *  (Buffer<uint>); the triangles' vertex indices (StructuredBuffer<uint>). */
    FRDGBufferSRVRef Positions = nullptr;
    FRDGBufferSRVRef TexCoords = nullptr;
    FRDGBufferSRVRef Colors = nullptr;
    FRDGBufferSRVRef Indices = nullptr;
    uint32 TexWords = 0;
    uint32 Triangles = 0;
    /** The vertices' origin (planet frame, cm), and the planet's frame in the world. */
    FVector Origin = FVector::ZeroVector;
    FTransform PlanetToWorld = FTransform::Identity;
    /** The sea sphere's radius (cm). */
    double SeaRadius = 0.0;
    /** The waves as the material takes them this frame (from the selection camera; its resource
     *  pointers null), and their textures. */
    FVoxelOceanDisplaceFrame Waves;
    FRHITexture* Displacement = nullptr;
    FRHITexture* Derivatives = nullptr;
    FRHITexture* Charts = nullptr;
    /** Stage W3u: the cascades' foam envelopes (null: none). */
    FRHITexture* Foam = nullptr;
    /** Stage W3z: the shortest band's own cascade (VoxelOceanCascades' fine arrays; null: none). */
    FRHITexture* FineDisplacement = nullptr;
    FRHITexture* FineDerivatives = nullptr;
    FRHITexture* FineFoam = nullptr;
    /** The wave clipmap's textures, when it ran this frame (bClip). */
    bool bClip = false;
    FRHITexture* ClipDisplacement = nullptr;
    FRHITexture* ClipNormal = nullptr;
    /** Stage W3h: the whitecaps' density (null before the first clipmap frame). */
    FRHITexture* ClipWhitecap = nullptr;
    /** Stage W8j: the summed surface's strain (null: none; the pixels' own foam looks every band up). */
    FRHITexture* ClipStrain = nullptr;
    FRHITexture* ClipInfo = nullptr;
    /** Stage W8l: the info's rows themselves (what ClipInfo holds this frame), for the own pass's
     *  constants (VoxelOceanClipInfoConstants). */
    FVector4f ClipRows[VoxelOceanClipmap::InfoRows];
    /** Stage W4c: the foam's fine history (null: none), its levels and its factor. */
    FRHITexture* ClipFineHistory = nullptr;
    FVector2f ClipFineInfo = FVector2f(0.0f, 0.0f);
    /** Stage W5c: the clipmap's static layer (null: none; the pixels then evaluate the open sea alone). */
    FRHITexture* ClipStatic = nullptr;
};

/** The rings' side of the own pass (FVoxelOceanPages; render thread). */
class IVoxelOceanRingDraw
{
public:
    virtual ~IVoxelOceanRingDraw() = default;

    /** This frame's rings in GraphBuilder: their vertices are written by the frame's first call
     *  (in whichever graph makes it), the later calls read them. False: nothing to draw (no frame
     *  yet, or a shader missing). */
    virtual bool PrepareDraw_RenderThread(FRDGBuilder& GraphBuilder, FVoxelOceanRingDrawFrame& Out) = 0;
};
