#include "VoxelLandMaterial.h"
#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "VoxelLandBake.h"
#include "VoxelNoiseTiles.h"
#include "VoxelPlugin.h"
#include "Async/ParallelFor.h"
#include "HAL/IConsoleManager.h"
#include "Engine/Texture2DDynamic.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Materials/MaterialInterface.h"
#include "RenderingThread.h"
#include "RHI.h"
#include "RHICommandList.h"
#include "TextureResource.h"

// The layout the material decodes (shared with VoxelLandMaterialAccess.ush).
#include "VoxelLandMaterialLayout.ush"

// The scalars must fit their texels, or the next slot overwrites the tan table that follows.
static_assert(VOXEL_LAND_MC_COUNT <= 4 * VOXEL_LAND_MC_TEXELS, "VOXEL_LAND_MC_COUNT overruns VOXEL_LAND_MC_TEXELS");

const FName VoxelLandMaterial::HeightTextureName(TEXT("VoxelLandHeight"));
const FName VoxelLandMaterial::FieldTextureName(TEXT("VoxelLandFields"));
const FName VoxelLandMaterial::ConstantTextureName(TEXT("VoxelLandConstants"));
const FName VoxelLandMaterial::NoiseTextureName(TEXT("VoxelLandNoise"));
const FName VoxelLandMaterial::OriginHighName(TEXT("VoxelLandOriginHigh"));
const FName VoxelLandMaterial::OriginLowName(TEXT("VoxelLandOriginLow"));
const FName VoxelLandMaterial::AxisXName(TEXT("VoxelLandAxisX"));
const FName VoxelLandMaterial::AxisYName(TEXT("VoxelLandAxisY"));
const FName VoxelLandMaterial::AxisZName(TEXT("VoxelLandAxisZ"));
const FName VoxelLandMaterial::AxisXLowName(TEXT("VoxelLandAxisXLow"));
const FName VoxelLandMaterial::AxisYLowName(TEXT("VoxelLandAxisYLow"));
const FName VoxelLandMaterial::AxisZLowName(TEXT("VoxelLandAxisZLow"));
const FName VoxelLandMaterial::ParamsName(TEXT("VoxelLandParams"));

static TAutoConsoleVariable<int32> CVarVoxelLandNormalMode(
    TEXT("voxel.Debug.LandNormalMode"), 0,
    TEXT("The terrain material's land node normal, for comparison (STAGE_M1d_NOTES.md):\n")
    TEXT(" 0: the bake's and the detail's exact gradients, the gullies' screen-space change (default)\n")
    TEXT(" 1: the screen-space change of gullies and detail together (M1)\n")
    TEXT(" 2: the bake and the detail exact, no gullies\n")
    TEXT(" 3: the bake alone"),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandMaterialCost(
    TEXT("voxel.Land.MaterialCost"), 0,
    TEXT("Profiling: what the terrain material's land node evaluates (STAGE_M1f_NOTES.md):\n")
    TEXT(" 0: everything (default)\n")
    TEXT(" 1: no fixed-step gully difference close up (screen-space instead)\n")
    TEXT(" 2: no gullies\n")
    TEXT(" 3: no gullies, no detail layer, no hills and no peaks (the bake alone)\n")
    TEXT(" 4: the node off (the mesh's normal; the vertex colours carry the views)"),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelLandMaterialSource(
    TEXT("voxel.Land.MaterialSource"), 0,
    TEXT("Where the terrain material's land node reads the land (Design/LandSurfaceCache.md):\n")
    TEXT(" 0: the land surface cache, written by compute around the camera (default)\n")
    TEXT(" 1: the analytic node, the land chain per pixel (the reference; M1)"),
    ECVF_Default);

static TAutoConsoleVariable<int32> CVarVoxelDebugLandCache(
    TEXT("voxel.Debug.LandCache"), 0,
    TEXT("1: colour the terrain by the land surface cache's level each pixel reads (a hue a level, darker where two blend; magenta where none is written yet), and print the cache's state."),
    ECVF_Default);

bool VoxelLandMaterial::UsesCache()
{
    return CVarVoxelLandMaterialSource.GetValueOnGameThread() == 0;
}

bool VoxelLandMaterial::ShowsCacheLevels()
{
    return CVarVoxelDebugLandCache.GetValueOnGameThread() != 0;
}

namespace VoxelLandMaterialDetail
{
    /** A 32-bit integer as two floats of 16 bits (low, high): exact in a float texel. */
    void PutU32(float* S, int32 Index, uint32 V)
    {
        S[Index] = (float)(V & 0xffffu);
        S[Index + 1] = (float)(V >> 16);
    }

    /** A world position or direction as a double-float pair. */
    void Split(const FVector& V, FVector3f& OutHigh, FVector3f& OutLow)
    {
        OutHigh = FVector3f((float)V.X, (float)V.Y, (float)V.Z);
        OutLow = FVector3f((float)(V.X - (double)OutHigh.X), (float)(V.Y - (double)OutHigh.Y), (float)(V.Z - (double)OutHigh.Z));
    }

    /** Fills Texture on the render thread, after its resource (created with it) exists. Owner
     *  holds the texels until then; GetData(Owner) gives them. */
    template <typename TOwner, typename TGetData>
    void Upload(UTexture2DDynamic* Texture, TOwner Owner, TGetData GetData, int32 Width, int32 Height, int32 BytesPerTexel)
    {
        FTextureResource* Resource = Texture ? Texture->GetResource() : nullptr;
        if (!Resource) return;
        ENQUEUE_RENDER_COMMAND(VoxelLandMaterialUpload)([Resource, Owner = MoveTemp(Owner), GetData, Width, Height, BytesPerTexel](FRHICommandListImmediate& RHICmdList)
            {
                if (!Resource->TextureRHI.IsValid()) return;
                const FUpdateTextureRegion2D Region(0, 0, 0, 0, (uint32)Width, (uint32)Height);
                RHICmdList.UpdateTexture2D(Resource->TextureRHI, 0, Region, (uint32)(Width * BytesPerTexel), (const uint8*)GetData(Owner));
            });
    }

    UTexture2DDynamic* Make(int32 Width, int32 Height, EPixelFormat Format)
    {
        FTexture2DDynamicCreateInfo Info(Format, false, false, TF_Nearest, AM_Clamp);
        return UTexture2DDynamic::Create(Width, Height, Info);
    }

    /** The row a chain (from texel Base, N texels a side) ends in, in a pack's rows of RowN. */
    uint32 ChainEndRow(const FVoxelLandNoiseSet& Noise, uint32 Base, uint32 N)
    {
        return (uint32)((Base + VoxelNoiseTiles::ChainTexels(N) + Noise.RowN - 1) / Noise.RowN);
    }

    /** The rows of the noise pack's words the material's texture takes on this RHI: all of them,
     *  or as many whole chains from the start as fit (the material then shades without the
     *  rest: the hills' set, then the detail tile; the registry's limit, the same on every
     *  machine, decides what the geometry has); 0 if not even the first fits. */
    uint32 NoiseRowsFor(const FVoxelLandNoiseSet* Noise)
    {
        const uint32 Max = (uint32)GetMax2DTextureDimension();
        if (!Noise || !Noise->IsComplete() || Noise->RowN > Max) return 0;
        if (Noise->Rows <= Max) return Noise->Rows;
        uint32 Best = 0;
        for (const uint32 End : { Noise->N > 0 ? ChainEndRow(*Noise, 0u, Noise->N) : 0u,
                 Noise->DetailN > 0 ? ChainEndRow(*Noise, Noise->DetailBase, Noise->DetailN) : 0u,
                 Noise->HillN > 0 ? ChainEndRow(*Noise, Noise->HillBase, Noise->HillN) : 0u })
            if (End <= Max) Best = FMath::Max(Best, End);
        return Best;
    }

    /** Whether the noise pack goes into the material (its texture fits this RHI): the constants
     *  turn a layer on exactly when the texture holds its chain, not the stand-in. */
    bool NoiseFits(const FVoxelLandNoiseSet* Noise)
    {
        return NoiseRowsFor(Noise) > 0;
    }

    /** Whether the texture holds a chain of the pack. */
    bool NoiseChainFits(const FVoxelLandNoiseSet* Noise, uint32 Base, uint32 N)
    {
        return Noise && N > 0 && ChainEndRow(*Noise, Base, N) <= NoiseRowsFor(Noise);
    }

    /** Whether the detail tile goes into the material. */
    bool NoiseDetailFits(const FVoxelLandNoiseSet* Noise)
    {
        return Noise && NoiseChainFits(Noise, Noise->DetailBase, Noise->DetailN);
    }
}

void VoxelLandMaterial::BuildFieldTexels(const FVoxelLandBake& Bake, TArray<FFloat16Color>& Out)
{
    const int32 NumCells = Bake.NumCells();
    // The fields: drainage, the lake's level above the cell's own height (small, so the half
    // keeps it to centimetres in shallow lakes where an absolute level would round to metres),
    // the attributes' two parts.
    const bool bFlow = Bake.Flow.Num() == NumCells;
    const bool bWater = Bake.Water.Num() == NumCells && Bake.Height.Num() == NumCells;
    const bool bAttr = Bake.Attr.Num() == NumCells;
    Out.SetNumUninitialized(NumCells);
    constexpr int32 Chunk = 1 << 16;
    ParallelFor((NumCells + Chunk - 1) / Chunk, [&](int32 c)
        {
            const int32 End = FMath::Min(NumCells, (c + 1) * Chunk);
            for (int32 i = c * Chunk; i < End; i++)
            {
                FFloat16Color& T = Out[i];
                T.R = bFlow ? Bake.Flow[i] : FFloat16(0.0f);
                const float Level = bWater ? Bake.Water[i] : -3.0e38f;
                T.G = FFloat16(Level > -1.0e30f
                    ? FMath::Clamp(Level - Bake.Height[i], VOXEL_LAND_MATERIAL_NO_LAKE + 1000.0f, 65000.0f)
                    : VOXEL_LAND_MATERIAL_NO_LAKE);
                const uint32 Attr = bAttr ? (uint32)Bake.Attr[i] : 0u;
                T.B = FFloat16((float)(Attr & 0xffu));
                T.A = FFloat16((float)(Attr >> 8));
            }
        });
}

void VoxelLandMaterial::BuildHeightTexels(const FVoxelLandBake& Bake, TArray<FVector2f>& Out)
{
    const int32 NumCells = Bake.NumCells();
    const bool bForm = Bake.Form.Num() == NumCells && Bake.Relief.Num() == NumCells;
    Out.SetNumUninitialized(NumCells);
    constexpr int32 Chunk = 1 << 16;
    ParallelFor((NumCells + Chunk - 1) / Chunk, [&](int32 c)
        {
            const int32 End = FMath::Min(NumCells, (c + 1) * Chunk);
            for (int32 i = c * Chunk; i < End; i++)
                Out[i] = FVector2f(Bake.Height[i], bForm ? (float)((uint32)Bake.Form[i] | ((uint32)Bake.Relief[i] << 16)) : 0.0f);
        });
}

void VoxelLandMaterial::BuildConstantTexels(const FVoxelDensityGPUData& Density, const FVoxelLandBake& Bake, TArray<FVector4f>& Out, int32& OutRows)
{
    using namespace VoxelLandMaterialDetail;
    const int32 NumCells = Bake.NumCells();
    const bool bFlow = Bake.Flow.Num() == NumCells;
    const bool bWater = Bake.Water.Num() == NumCells;
    const bool bForm = Bake.Form.Num() == NumCells && Bake.Relief.Num() == NumCells;

    // The constants: scalars, then the tan table, the octaves, the plates.
    const int32 TanBase = VOXEL_LAND_MC_TEXELS;
    const int32 TanTexels = (Bake.TanTable.Num() + 1) / 2;
    const int32 OctaveBase = TanBase + TanTexels;
    const int32 OctaveTexels = Density.Octaves.Num() * VOXEL_LAND_MATERIAL_OCTAVE_TEXELS;
    const int32 PlateBase = OctaveBase + OctaveTexels;
    const int32 PlateCount = Bake.Plates.Num() / 8;
    const int32 Texels = PlateBase + PlateCount * 2;
    OutRows = (Texels + VOXEL_LAND_MATERIAL_CONST_WIDTH - 1) / VOXEL_LAND_MATERIAL_CONST_WIDTH;
    Out.Init(FVector4f(0.0f, 0.0f, 0.0f, 0.0f), OutRows * VOXEL_LAND_MATERIAL_CONST_WIDTH);
    float* const S = &Out[0].X;

    S[VOXEL_LAND_MC_N] = (float)Bake.N;
    S[VOXEL_LAND_MC_PAD] = (float)FVoxelLandBake::Pad;
    S[VOXEL_LAND_MC_HAS_FLOW] = bFlow ? 1.0f : 0.0f;
    S[VOXEL_LAND_MC_CELL_M] = Density.LandCellM;
    S[VOXEL_LAND_MC_GULLY_STRENGTH] = Density.LandGullyStrength;
    S[VOXEL_LAND_MC_GULLY_OCTAVES] = (float)Density.LandGullyOctaves;
    S[VOXEL_LAND_MC_GULLY_FLOW_GAIN] = Density.LandGullyFlowGain;
    S[VOXEL_LAND_MC_GULLY_DETAIL] = Density.LandGullyDetail;
    PutU32(S, VOXEL_LAND_MC_GULLY_SEED, Density.LandGullySeed);
    S[VOXEL_LAND_MC_WARP_OCTAVES] = (float)Density.LandWarpOctaves;
    S[VOXEL_LAND_MC_WARP_INV_NORM] = Density.LandWarpInvNorm;
    S[VOXEL_LAND_MC_WARP_AMPLITUDE] = Density.LandWarpAmplitude;
    S[VOXEL_LAND_MC_DETAIL_AMPLITUDE] = Density.LandDetailAmplitude;
    S[VOXEL_LAND_MC_HEIGHT_OFFSET] = Density.LandHeightOffset.X;
    S[VOXEL_LAND_MC_HEIGHT_OFFSET + 1] = Density.LandHeightOffset.Y;
    S[VOXEL_LAND_MC_PLANET_RADIUS] = Density.PlanetRadius.X;
    S[VOXEL_LAND_MC_PLANET_RADIUS + 1] = Density.PlanetRadius.Y;
    S[VOXEL_LAND_MC_NOISE_AMPLITUDE] = Density.NoiseAmplitude.X;
    S[VOXEL_LAND_MC_NOISE_AMPLITUDE + 1] = Density.NoiseAmplitude.Y;
    S[VOXEL_LAND_MC_INV_NORM_DETAIL] = Density.InvNormDetail.X;
    S[VOXEL_LAND_MC_INV_NORM_DETAIL + 1] = Density.InvNormDetail.Y;
    S[VOXEL_LAND_MC_RIDGE_GAIN] = Density.RidgeGain;
    S[VOXEL_LAND_MC_SLOPE_DAMPING] = Density.SlopeDamping;
    S[VOXEL_LAND_MC_CONTINENT_OCTAVES] = (float)Density.ContinentOctaves;
    S[VOXEL_LAND_MC_MOUNTAIN_OCTAVES] = (float)Density.MountainOctaves;
    S[VOXEL_LAND_MC_DETAIL_OCTAVES] = (float)Density.DetailOctaves;
    S[VOXEL_LAND_MC_LAND_MODE] = (float)Density.LandMode;
    S[VOXEL_LAND_MC_PLATE_COUNT] = (float)PlateCount;
    PutU32(S, VOXEL_LAND_MC_PLATE_SEED, Bake.PlateSeed);
    S[VOXEL_LAND_MC_HAS_HYDRO] = bFlow && bWater ? 1.0f : 0.0f;
    S[VOXEL_LAND_MC_TAN_BASE] = (float)TanBase;
    S[VOXEL_LAND_MC_OCTAVE_BASE] = (float)OctaveBase;
    S[VOXEL_LAND_MC_PLATE_BASE] = (float)PlateBase;
    S[VOXEL_LAND_MC_SIDE] = (float)Bake.Side();
    S[VOXEL_LAND_MC_PLATE_RADIUS_M] = Bake.PlateRadiusM;
    S[VOXEL_LAND_MC_PLATE_WIDTH_SCALE] = Bake.PlateWidthScale;
    S[VOXEL_LAND_MC_HILL_RELIEF] = Density.LandHillRelief;
    S[VOXEL_LAND_MC_HILL_FREQUENCY] = Density.LandHillFrequency.X;
    S[VOXEL_LAND_MC_HILL_FREQUENCY + 1] = Density.LandHillFrequency.Y;
    S[VOXEL_LAND_MC_HILL_OCTAVES] = (float)Density.LandHillOctaves;
    PutU32(S, VOXEL_LAND_MC_HILL_SEED, Density.LandHillSeed);
    S[VOXEL_LAND_MC_HAS_FORM] = bForm ? 1.0f : 0.0f;
    // The noise pack (VoxelLandNoise.ush): the peaks, the detail tile, the hills' set; each off
    // without its chain in the texture. The hills' set's relief without it would be the
    // procedural hills at the set's height: none then.
    if (Density.LandHillTexelsPerCell > 0.0f && !(Density.LandNoise.IsValid() && NoiseChainFits(Density.LandNoise.Get(), Density.LandNoise->HillBase, Density.LandNoise->HillN)))
        S[VOXEL_LAND_MC_HILL_RELIEF] = 0.0f;
    const FVoxelLandNoiseSet* Pack = NoiseFits(Density.LandNoise.Get()) ? Density.LandNoise.Get() : nullptr;
    if (Pack)
    {
        S[VOXEL_LAND_MC_NOISE_SHIFT] = (float)FMath::FloorLog2(Pack->RowN);
        if (NoiseDetailFits(Pack))
        {
            PutU32(S, VOXEL_LAND_MC_PEAK_DETAIL_BASE, Pack->DetailBase);
            S[VOXEL_LAND_MC_PEAK_DETAIL_N] = (float)Pack->DetailN;
            S[VOXEL_LAND_MC_PEAK_DETAIL_MIPS] = (float)Pack->DetailMips;
            S[VOXEL_LAND_MC_PEAK_DETAIL_MEAN_R] = Density.LandPeakDetailMeanR;
            S[VOXEL_LAND_MC_PEAK_DETAIL_MEAN_G] = Density.LandPeakDetailMeanG;
        }
        if (Density.LandHillTexelsPerCell > 0.0f && NoiseChainFits(Pack, Pack->HillBase, Pack->HillN))
        {
            PutU32(S, VOXEL_LAND_MC_HILL_BASE, Pack->HillBase);
            S[VOXEL_LAND_MC_HILL_N] = (float)Pack->HillN;
            S[VOXEL_LAND_MC_HILL_MIPS] = (float)Pack->HillMips;
            S[VOXEL_LAND_MC_HILL_TEXELS_PER_CELL] = Density.LandHillTexelsPerCell;
            S[VOXEL_LAND_MC_HILL_SCALE_R] = Density.LandHillScaleR;
            S[VOXEL_LAND_MC_HILL_SCALE_G] = Density.LandHillScaleG;
            S[VOXEL_LAND_MC_HILL_SCALE_B] = Density.LandHillScaleB;
            S[VOXEL_LAND_MC_HILL_SPACING_M] = Density.LandHillSpacingM;
            S[VOXEL_LAND_MC_HILL_SWELL_TEXELS_PER_CELL] = Density.LandHillSwellTexelsPerCell;
            S[VOXEL_LAND_MC_HILL_SWELL_M] = Density.LandHillSwellM;
            S[VOXEL_LAND_MC_HILL_SWELL_SCALE_G] = Density.LandHillSwellScaleG;
            S[VOXEL_LAND_MC_HILL_FOOTHILL] = Density.LandHillFoothill;
            if (NoiseDetailFits(Pack))
            {
                S[VOXEL_LAND_MC_HILL_DETAIL_TEXELS_PER_CELL] = Density.LandHillDetailTexelsPerCell;
                S[VOXEL_LAND_MC_HILL_DETAIL_SCALE_R] = Density.LandHillDetailScaleR;
                S[VOXEL_LAND_MC_HILL_DETAIL_SCALE_G] = Density.LandHillDetailScaleG;
                S[VOXEL_LAND_MC_HILL_DETAIL_FADE] = Density.LandHillDetailFade;
            }
        }
    }
    if (const FVoxelLandNoiseSet* Noise = Pack && Pack->N > 0 && NoiseChainFits(Pack, 0u, Pack->N) ? Pack : nullptr)
    {
        S[VOXEL_LAND_MC_PEAK_STRENGTH] = Density.LandPeakStrength;
        S[VOXEL_LAND_MC_PEAK_TEXELS_PER_CELL] = Density.LandPeakTexelsPerCell;
        PutU32(S, VOXEL_LAND_MC_PEAK_SEED, Density.LandPeakSeed);
        S[VOXEL_LAND_MC_NOISE_N] = (float)Noise->N;
        S[VOXEL_LAND_MC_NOISE_MIPS] = (float)Noise->Mips;
        S[VOXEL_LAND_MC_PEAK_SPACING_M] = Density.LandPeakSpacingM;
        S[VOXEL_LAND_MC_PEAK_RELIEF_M] = Density.LandPeakReliefM;
        S[VOXEL_LAND_MC_PEAK_RELIEF_START] = Density.LandPeakReliefStart;
        S[VOXEL_LAND_MC_PEAK_RELIEF_END] = Density.LandPeakReliefEnd;
        S[VOXEL_LAND_MC_PEAK_MEAN_R] = Density.LandPeakMeanR;
        S[VOXEL_LAND_MC_PEAK_MEAN_G] = Density.LandPeakMeanG;
        S[VOXEL_LAND_MC_PEAK_SCALE_R] = Density.LandPeakScaleR;
        S[VOXEL_LAND_MC_PEAK_SCALE_G] = Density.LandPeakScaleG;
        S[VOXEL_LAND_MC_PEAK_SCALE_B] = Density.LandPeakScaleB;
        // The detail tile, in the same texture after the set's chain (T3e).
        if (Density.LandPeakDetailTexelsPerCell > 0.0f && NoiseDetailFits(Noise))
        {
            S[VOXEL_LAND_MC_PEAK_DETAIL_TEXELS_PER_CELL] = Density.LandPeakDetailTexelsPerCell;
            S[VOXEL_LAND_MC_PEAK_DETAIL_SCALE_R] = Density.LandPeakDetailScaleR;
            S[VOXEL_LAND_MC_PEAK_DETAIL_SCALE_G] = Density.LandPeakDetailScaleG;
            S[VOXEL_LAND_MC_PEAK_DETAIL_FADE] = Density.LandPeakDetailFade;
        }
    }

    for (int32 i = 0; i < Bake.TanTable.Num(); i++)
    {
        float* T = S + 4 * (TanBase + i / 2) + 2 * (i & 1);
        T[0] = Bake.TanTable[i].X;
        T[1] = Bake.TanTable[i].Y;
    }
    for (int32 o = 0; o < Density.Octaves.Num(); o++)
    {
        const FVoxelDensityGPUOctave& Oct = Density.Octaves[o];
        float* T = S + 4 * (OctaveBase + o * VOXEL_LAND_MATERIAL_OCTAVE_TEXELS);
        T[0] = Oct.Frequency.X;  T[1] = Oct.Frequency.Y;  T[2] = Oct.OffsetX.X;  T[3] = Oct.OffsetX.Y;
        T[4] = Oct.OffsetY.X;    T[5] = Oct.OffsetY.Y;    T[6] = Oct.OffsetZ.X;  T[7] = Oct.OffsetZ.Y;
        T[8] = Oct.Wavelength;   T[9] = Oct.Amplitude;
        PutU32(T, 10, Oct.Seed);
        PutU32(T, 12, Oct.Shift);
        PutU32(T, 14, Oct.OffsetCell[0]);
        PutU32(T, 16, Oct.OffsetCell[1]);
        PutU32(T, 18, Oct.OffsetCell[2]);
        PutU32(T, 20, Oct.OffsetFrac[0]);
        PutU32(T, 22, Oct.OffsetFrac[1]);
        PutU32(T, 24, Oct.OffsetFrac[2]);
    }
    for (int32 i = 0; i < PlateCount * 8; i++)
        S[4 * PlateBase + i] = Bake.Plates[i];
}

bool VoxelLandMaterial::HasLandNode(const UMaterialInterface* Material)
{
    FLinearColor Value;
    return Material && Material->GetVectorParameterValue(FHashedMaterialParameterInfo(ParamsName), Value);
}

bool VoxelLandMaterial::MakeBakeTextures(const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& Bake,
    UTexture2DDynamic*& OutHeight, UTexture2DDynamic*& OutFields)
{
    using namespace VoxelLandMaterialDetail;
    OutHeight = nullptr;
    OutFields = nullptr;
    if (!Bake.IsValid() || Bake->Height.Num() != Bake->NumCells()) return false;

    // A face a band, six bands tall: 6 x 2056 texels at the largest bake (2048), within every
    // desktop RHI's 16384.
    const int32 Side = Bake->Side();
    if ((uint32)(6 * Side) > GetMax2DTextureDimension())
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] The land material's textures (%d x %d) exceed this RHI's largest texture (%u); the material keeps the mesh's normals."),
            Side, 6 * Side, GetMax2DTextureDimension());
        return false;
    }
    UTexture2DDynamic* Height = Make(Side, 6 * Side, PF_G32R32F);
    UTexture2DDynamic* Fields = Make(Side, 6 * Side, PF_FloatRGBA);
    if (!Height || !Fields)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Couldn't create the land material's textures."));
        return false;
    }
    TArray<FFloat16Color> FieldTexels;
    BuildFieldTexels(*Bake, FieldTexels);
    TArray<FVector2f> HeightTexels;
    BuildHeightTexels(*Bake, HeightTexels);
    Upload(Height, MoveTemp(HeightTexels), [](const TArray<FVector2f>& T) { return (const void*)T.GetData(); },
        Side, 6 * Side, (int32)sizeof(FVector2f));
    Upload(Fields, MoveTemp(FieldTexels), [](const TArray<FFloat16Color>& T) { return (const void*)T.GetData(); },
        Side, 6 * Side, (int32)sizeof(FFloat16Color));
    OutHeight = Height;
    OutFields = Fields;
    return true;
}

UTexture2DDynamic* VoxelLandMaterial::MakeConstantTexture(const FVoxelDensityCPU& Density)
{
    using namespace VoxelLandMaterialDetail;
    FVoxelDensityGPUData Data;
    Density.BuildGPUData(Data);
    const FVoxelLandBake* Bake = Data.LandMode != 0 ? Data.Land.Get() : nullptr;
    if (!Bake || Bake->TanTable.Num() != Bake->Side()) return nullptr;
    TArray<FVector4f> Texels;
    int32 Rows = 0;
    BuildConstantTexels(Data, *Bake, Texels, Rows);
    UTexture2DDynamic* Constants = Make(VOXEL_LAND_MATERIAL_CONST_WIDTH, Rows, PF_A32B32G32R32F);
    if (!Constants)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Couldn't create the land material's constants texture."));
        return nullptr;
    }
    Upload(Constants, MoveTemp(Texels), [](const TArray<FVector4f>& T) { return (const void*)T.GetData(); },
        VOXEL_LAND_MATERIAL_CONST_WIDTH, Rows, (int32)sizeof(FVector4f));
    return Constants;
}

UTexture2DDynamic* VoxelLandMaterial::MakeNoiseTexture(const FVoxelDensityCPU& Density)
{
    using namespace VoxelLandMaterialDetail;
    const TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Noise = Density.GetLandNoise();
    const bool bSet = NoiseFits(Noise.Get());
    // One texture a set, for every planet that uses it (game thread; each planet holds its own).
    struct FShared
    {
        TWeakPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe> Set;
        TWeakObjectPtr<UTexture2DDynamic> Texture;
    };
    static TArray<FShared> Shared;
    static TWeakObjectPtr<UTexture2DDynamic> StandIn;
    Shared.RemoveAll([](const FShared& E) { return !E.Set.IsValid() || !E.Texture.IsValid(); });
    if (bSet)
        for (const FShared& E : Shared)
            if (E.Set.Pin() == Noise) return E.Texture.Get();
    if (!bSet && StandIn.IsValid()) return StandIn.Get();
    const int32 Width = bSet ? (int32)Noise->RowN : 1;
    const int32 Height = bSet ? (int32)NoiseRowsFor(Noise.Get()) : 1;
    UTexture2DDynamic* Texture = Make(Width, Height, PF_A16B16G16R16);
    if (!Texture)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Couldn't create the land material's noise texture."));
        return nullptr;
    }
    // The chains in rows of RowN texels (VoxelLandMaterialAccess.ush): two words a texel are the
    // RGBA16 texel as it lies in memory (R, G, B, A, little-endian); the shared set stays alive
    // until the upload ran.
    if (bSet)
    {
        Upload(Texture, Noise, [](const TSharedPtr<const FVoxelLandNoiseSet, ESPMode::ThreadSafe>& N) { return (const void*)N->Words.GetData(); },
            Width, Height, 8);
        Shared.Add({ Noise, Texture });
    }
    else
    {
        TArray<uint32> Zero;
        Zero.Init(0u, 2);
        Upload(Texture, MoveTemp(Zero), [](const TArray<uint32>& T) { return (const void*)T.GetData(); }, 1, 1, 8);
        StandIn = Texture;
    }
    return Texture;
}

void VoxelLandMaterial::SetTextureParameters(UMaterialInstanceDynamic* Material, UTexture2DDynamic* Height, UTexture2DDynamic* Fields,
    UTexture2DDynamic* Constants, UTexture2DDynamic* Noise)
{
    if (!Material) return;
    Material->SetTextureParameterValue(HeightTextureName, Height);
    Material->SetTextureParameterValue(FieldTextureName, Fields);
    Material->SetTextureParameterValue(ConstantTextureName, Constants);
    if (Noise)
        Material->SetTextureParameterValue(NoiseTextureName, Noise);
}

void VoxelLandMaterial::SetFrameParameters(UMaterialInstanceDynamic* Material, const FTransform& PlanetFrame,
    int32 View, double Tolerance, double NearFootprint, bool bEnabled, bool bCacheAvailable)
{
    using namespace VoxelLandMaterialDetail;
    if (!Material) return;
    FVector3f OriginHigh, OriginLow;
    Split(PlanetFrame.GetLocation(), OriginHigh, OriginLow);
    auto Set = [Material](const FName& Name, const FVector3f& V) { Material->SetVectorParameterValue(Name, FLinearColor(V.X, V.Y, V.Z, 0.0f)); };
    Set(OriginHighName, OriginHigh);
    Set(OriginLowName, OriginLow);
    const FName HighNames[3] = { AxisXName, AxisYName, AxisZName };
    const FName LowNames[3] = { AxisXLowName, AxisYLowName, AxisZLowName };
    const EAxis::Type Axes[3] = { EAxis::X, EAxis::Y, EAxis::Z };
    for (int32 a = 0; a < 3; a++)
    {
        FVector3f High, Low;
        Split(PlanetFrame.GetUnitAxis(Axes[a]), High, Low);
        Set(HighNames[a], High);
        Set(LowNames[a], Low);
    }
    // x: the view, plus 32 x the normal's sources, 128 x the cost switch, 1024 x the analytic
    // source and 2048 x the cache's level view (VoxelLandMaterial.ush).
    const int32 NormalMode = FMath::Clamp(CVarVoxelLandNormalMode.GetValueOnGameThread(), 0, 3);
    const int32 Cost = FMath::Clamp(CVarVoxelLandMaterialCost.GetValueOnGameThread(), 0, 4);
    if (Cost == 4) bEnabled = false;
    // The cache when it's asked for and there; else the analytic node.
    const bool bCache = UsesCache() && bCacheAvailable;
    const int32 Analytic = bCache ? 0 : 1;
    const int32 LevelView = bCache && ShowsCacheLevels() ? 1 : 0;
    const float Packed = (float)(FMath::Clamp(View, 0, 31) + 32 * NormalMode + 128 * Cost + 1024 * Analytic + 2048 * LevelView);
    Material->SetVectorParameterValue(ParamsName, FLinearColor(bEnabled ? Packed : 0.0f, (float)FMath::Max(Tolerance, 0.0),
        bEnabled ? 1.0f : 0.0f, (float)FMath::Max(NearFootprint, 0.0)));
}
