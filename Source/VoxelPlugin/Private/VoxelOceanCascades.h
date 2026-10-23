// VoxelOceanCascades.h - The ocean's FFT cascades on the GPU (Design/OceanWaves.md section 2;
// stage O6b): the passes of VoxelOceanFFT.usf run once per tick, and their outputs copied into
// the textures the surface material and the underwater pass sample.
//
// THE TEXTURES (FVoxelOceanCascadeTextures, made once per ocean by MakeTextures; their owner, the
// ocean actor, keeps them alive):
// - Displacement and Derivatives: Texture 2D Arrays, 256^2 x 8 slots, RGBA16F, full mip chains,
//   wrapping, trilinear (VoxelOceanFFT.usf, AssembleCS: chop Dx, chop Dy, eta, chop Dxy and
//   eta_x, eta_y, chop Dxx, chop Dyy);
// - Charts: the chart table (VoxelOceanCharts.ush), RGBA32F, point sampled;
// - Foam (stage O9b): a Texture 2D Array, 256^2 x 8 slots, G16R16F, full mip chain, wrapping,
//   trilinear: each slot's open-sea foam envelope Z and its square (VoxelOceanFoam.ush; the mips
//   hold their means), Z kept between frames in a buffer of the render state and falling one
//   sigma per the foam's decay.
// A material binds them as Texture Object Parameters VoxelOceanDisplacement,
// VoxelOceanDerivatives, VoxelOceanCharts and VoxelOceanFoam (the names below).
//
// EACH TICK (Update, game thread): a render command builds its own render graph, as the land
// cache's: the start spectrum again if the slots changed (InitSpectrumCS, into buffers kept
// between frames), then the time step, the row and column FFTs, the assembly into transient
// textures, their mips, and a copy of every mip into the arrays above, left readable by shaders.
// Rendered frames read the arrays after the command (render commands run in order).

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanSpectrum.h"
#include "VoxelOceanCharts.h"
#include "VoxelOceanSurface.h"

class UObject;

namespace VoxelOceanCascades
{
    /** The material's texture parameters. */
    VOXELPLUGIN_API extern const FName DisplacementName;
    VOXELPLUGIN_API extern const FName DerivativesName;
    VOXELPLUGIN_API extern const FName ChartsName;
    VOXELPLUGIN_API extern const FName FoamName;

    /** Game thread. The cascade arrays (zeroed; those InOut already holds are kept: they don't
     *  depend on the planet) and a new chart table's texture, all in the transient package (the
     *  owner keeps them alive); false (InOut unchanged) if the RHI refuses any. */
    VOXELPLUGIN_API bool MakeTextures(const UObject* Owner, const VoxelOceanCharts::FTable& Charts, FVoxelOceanCascadeTextures& InOut);
}

class VOXELPLUGIN_API FVoxelOceanCascades
{
public:
    explicit FVoxelOceanCascades(const FVoxelOceanCascadeTextures& InTextures);
    ~FVoxelOceanCascades();

    /** Game thread. The slot table (VoxelOceanSpectrum::MakeSlots, ToGPU): a change builds the
     *  start spectrum again on the next update. */
    void SetSlots(TConstArrayView<VoxelOceanSpectrum::FSlotGPU> InSlots);

    /** Game thread, at each wave update: the cascades at this tick of the ocean clock (TickOf;
     *  Time the clock itself, s, for the foam's step between updates). Stage W3w: the fine foam
     *  (GodotOceanWaves' rule in one slot's own tile, VoxelOceanFoam.ush): x its whitecap
     *  threshold (0: none), y its decay and z its growth (per second), w the slot it runs on. */
    void Update(uint32 Tick, double Time, const FVector4f& FineFoam);

    const FVoxelOceanCascadeTextures& GetTextures() const { return Textures; }

private:
    struct FRenderState;

    FVoxelOceanCascadeTextures Textures;
    TArray<VoxelOceanSpectrum::FSlotGPU> Slots;
    uint32 Generation = 0;
    double LastTime = 0.0;
    bool bHasTime = false;
    TSharedRef<FRenderState, ESPMode::ThreadSafe> State;
};
