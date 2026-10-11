#include "VoxelLandMaterial.h"
#include "VoxelDensity.h"
#include "VoxelDensityGPU.h"
#include "VoxelLandBake.h"
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

const FName VoxelLandMaterial::HeightTextureName(TEXT("VoxelLandHeight"));
const FName VoxelLandMaterial::FieldTextureName(TEXT("VoxelLandFields"));
const FName VoxelLandMaterial::ConstantTextureName(TEXT("VoxelLandConstants"));
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
    TEXT(" 3: no gullies and no detail layer (the bake alone)\n")
    TEXT(" 4: the node off (the mesh's normal; the vertex colours carry the views)"),
    ECVF_Default);

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

void VoxelLandMaterial::BuildConstantTexels(const FVoxelDensityGPUData& Density, const FVoxelLandBake& Bake, TArray<FVector4f>& Out, int32& OutRows)
{
    using namespace VoxelLandMaterialDetail;
    const int32 NumCells = Bake.NumCells();
    const bool bFlow = Bake.Flow.Num() == NumCells;
    const bool bWater = Bake.Water.Num() == NumCells;

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
    UTexture2DDynamic* Height = Make(Side, 6 * Side, PF_R32_FLOAT);
    UTexture2DDynamic* Fields = Make(Side, 6 * Side, PF_FloatRGBA);
    if (!Height || !Fields)
    {
        UE_LOG(LogVoxelPlugin, Warning, TEXT("[VoxelLand] Couldn't create the land material's textures."));
        return false;
    }
    TArray<FFloat16Color> FieldTexels;
    BuildFieldTexels(*Bake, FieldTexels);
    // The heights straight from the bake (immutable, held by the command until it ran).
    Upload(Height, Bake, [](const TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe>& B) { return (const void*)B->Height.GetData(); },
        Side, 6 * Side, (int32)sizeof(float));
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

void VoxelLandMaterial::SetTextureParameters(UMaterialInstanceDynamic* Material, UTexture2DDynamic* Height, UTexture2DDynamic* Fields,
    UTexture2DDynamic* Constants)
{
    if (!Material) return;
    Material->SetTextureParameterValue(HeightTextureName, Height);
    Material->SetTextureParameterValue(FieldTextureName, Fields);
    Material->SetTextureParameterValue(ConstantTextureName, Constants);
}

void VoxelLandMaterial::SetFrameParameters(UMaterialInstanceDynamic* Material, const FTransform& PlanetFrame,
    int32 View, double Tolerance, double NearFootprint, bool bEnabled)
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
    // x: the view, plus 16 x the normal's sources and 64 x the cost switch (VoxelLandMaterial.ush).
    const int32 NormalMode = FMath::Clamp(CVarVoxelLandNormalMode.GetValueOnGameThread(), 0, 3);
    const int32 Cost = FMath::Clamp(CVarVoxelLandMaterialCost.GetValueOnGameThread(), 0, 4);
    if (Cost == 4) bEnabled = false;
    const float Packed = (float)(FMath::Clamp(View, 0, 15) + 16 * NormalMode + 64 * Cost);
    Material->SetVectorParameterValue(ParamsName, FLinearColor(bEnabled ? Packed : 0.0f, (float)FMath::Max(Tolerance, 0.0),
        bEnabled ? 1.0f : 0.0f, (float)FMath::Max(NearFootprint, 0.0)));
}
