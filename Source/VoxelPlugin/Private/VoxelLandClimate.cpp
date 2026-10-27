#include "VoxelLandClimate.h"
#include "VoxelLandClimateGrid.h"
#include "VoxelLandBake.h"
#include "VoxelLandGPU.h"
#include "RenderingThread.h"
#include "VoxelCubeSphere.h"
#include "AtmosphereWeather.h"
#include "VoxelShaderCompat.h"
#include "Async/ParallelFor.h"
#include "VoxelLandSettings.h"
#include "VoxelLandShelf.h"
#include <atomic>

// The point's climate: the material's own code (VoxelLandClimate.ush).
namespace VoxelShader
{
#include "VoxelLandClimate.ush"
}

uint32 FVoxelLandClimateSettings::GetMapHash() const
{
    uint32 Hash = GetTypeHash(LandPrecipitation);
    Hash = HashCombine(Hash, GetTypeHash(PrecipitationContrast));
    Hash = HashCombine(Hash, GetTypeHash(Seasonality));
    Hash = HashCombine(Hash, GetTypeHash(MaxWarmthAnomaly));
    return Hash;
}

bool FVoxelLandClimateMap::IsValid() const
{
    return Cells.Num() == VOXEL_LAND_CLIMATE_WIDTH * VOXEL_LAND_CLIMATE_HEIGHT;
}

FVector4f FVoxelLandClimateMap::Sample(const FVector& Dir) const
{
    if (!IsValid()) return FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
    const FVector D = Dir.GetSafeNormal();
    const VoxelShader::FVoxelLandClimateTaps T = VoxelShader::VoxelLandClimateTapsOf(VoxelShader::float3((float)D.X, (float)D.Y, (float)D.Z));
    auto At = [this](int32 X, int32 Y) { return Cells[Y * VOXEL_LAND_CLIMATE_WIDTH + X]; };
    const FVector4f A = At(T.X0, T.Y0) * (1.0f - T.FX) + At(T.X1, T.Y0) * T.FX;
    const FVector4f B = At(T.X0, T.Y1) * (1.0f - T.FX) + At(T.X1, T.Y1) * T.FX;
    return A * (1.0f - T.FY) + B * T.FY;
}

FVoxelLandClimateField::FVoxelLandClimateField()
    : GPU(MakeShared<FVoxelLandClimateGPU, ESPMode::ThreadSafe>())
{
}

FVoxelLandClimateField::~FVoxelLandClimateField()
{
    if (GPU.IsValid())
    {
        TSharedPtr<FVoxelLandClimateGPU, ESPMode::ThreadSafe> Resource = MoveTemp(GPU);
        ENQUEUE_RENDER_COMMAND(VoxelLandReleaseClimate)([Resource](FRHICommandListImmediate&) mutable
            {
                Resource.Reset();
            });
    }
}

bool FVoxelLandClimateField::IsValid() const
{
    return Entries.Num() == 3 * VOXEL_LAND_CLIMATE_WIDTH * VOXEL_LAND_CLIMATE_HEIGHT;
}

int32 VoxelLandClimate::Width()
{
    return VOXEL_LAND_CLIMATE_WIDTH;
}

int32 VoxelLandClimate::Height()
{
    return VOXEL_LAND_CLIMATE_HEIGHT;
}

void VoxelLandClimate::BuildBase(const FVoxelLandBake& Bake, double SeaRadiusM, TArray<float>& OutLand, TArray<float>& OutCoast,
    TArray<float>* OutHeight)
{
    constexpr int32 W = VOXEL_LAND_CLIMATE_WIDTH;
    constexpr int32 H = VOXEL_LAND_CLIMATE_HEIGHT;
    constexpr int32 Sub = 4;
    OutLand.SetNumZeroed(W * H);
    OutCoast.SetNumZeroed(W * H);
    if (OutHeight) OutHeight->SetNumZeroed(W * H);
    if (Bake.Height.Num() != Bake.NumCells()) return;
    const bool bAttr = Bake.Attr.Num() == Bake.NumCells();
    const int32 N = Bake.N;
    const int32 Side = Bake.Side();
    // The land's share: 4 x 4 points of the bake a cell, land where not sea-connected (a lake
    // below sea level is land to the climate, as to the atmosphere's surface map, stage A9).
    ParallelFor(H, [&](int32 J)
        {
            for (int32 I = 0; I < W; I++)
            {
                float Land = 0.0f;
                float HeightSum = 0.0f;
                float LandCount = 0.0f;
                for (int32 SJ = 0; SJ < Sub; SJ++)
                {
                    const double Mu = -1.0 + 2.0 * ((double)J + ((double)SJ + 0.5) / Sub) / H;
                    const double Cos = FMath::Sqrt(FMath::Max(1.0 - Mu * Mu, 0.0));
                    for (int32 SI = 0; SI < Sub; SI++)
                    {
                        const double Lon = 2.0 * UE_DOUBLE_PI * ((double)I + ((double)SI + 0.5) / Sub) / W;
                        int32 Face = 0;
                        double S = 0.0, T = 0.0;
                        VoxelCubeSphere::Locate(FVector(Cos * FMath::Cos(Lon), Cos * FMath::Sin(Lon), Mu), Face, S, T);
                        const int32 CI = FMath::Clamp((int32)FMath::FloorToInt((S + 1.0) * 0.5 * N), 0, N - 1);
                        const int32 CJ = FMath::Clamp((int32)FMath::FloorToInt((T + 1.0) * 0.5 * N), 0, N - 1);
                        const int32 Index = Face * Side * Side + (CJ + FVoxelLandBake::Pad) * Side + CI + FVoxelLandBake::Pad;
                        const bool bSea = bAttr ? (Bake.Attr[Index] & FVoxelLandBake::SeaBit) != 0 : Bake.Height[Index] <= 0.0f;
                        Land += bSea ? 0.0f : 1.0f;
                        HeightSum += bSea ? 0.0f : FMath::Max(Bake.Height[Index], 0.0f);
                        LandCount += bSea ? 0.0f : 1.0f;
                    }
                }
                OutLand[J * W + I] = Land / (Sub * Sub);
                // The land's own mean (the sea's points left out: a coast cell's ground isn't halved).
                if (OutHeight) (*OutHeight)[J * W + I] = LandCount > 0.0f ? HeightSum / LandCount : 0.0f;
            }
        });
    VoxelLandClimateGrid::CoastDistance(OutLand.GetData(), W, H, SeaRadiusM, OutCoast.GetData());
}

bool VoxelLandClimate::BuildMap(const TArray<float>& Land, const TArray<float>& Coast, const FAtmosphereClimateMap* Sim,
    const FVoxelLandClimateSettings& Settings, double SeaRadiusM, float EquatorTemperature, float PoleTemperature, float LatitudeExponent,
    float WarmthScale, FVoxelLandClimateMap& Out, const TArray<float>* LandHeight, const FVoxelLandShelfParams* Shelf)
{
    constexpr int32 W = VOXEL_LAND_CLIMATE_WIDTH;
    constexpr int32 H = VOXEL_LAND_CLIMATE_HEIGHT;
    Out.Cells.SetNumZeroed(W * H);
    if (Land.Num() != W * H || Coast.Num() != W * H) return false;
    // The sim's climate on this grid (bilinear at the cells' centres: the same layout, so a sim at
    // 512 x 256 maps one to one).
    TArray<FVector4f> Resampled;
    if (Sim && Sim->Width > 0 && Sim->Height > 0 && Sim->Cells.Num() == Sim->Width * Sim->Height)
    {
        Resampled.SetNumUninitialized(W * H);
        ParallelFor(H, [&](int32 J)
            {
                for (int32 I = 0; I < W; I++)
                {
                    double X = 0.0, Y = 0.0, Z = 0.0;
                    VoxelLandClimateGrid::CellDirection(I, J, W, H, X, Y, Z);
                    Resampled[J * W + I] = Sim->Sample(FVector(X, Y, Z));
                }
            });
    }
    FVoxelLandClimateGridParams P;
    P.RadiusM = SeaRadiusM;
    P.EquatorTemperature = EquatorTemperature;
    P.PoleTemperature = PoleTemperature;
    P.LatitudeExponent = LatitudeExponent;
    P.WarmthScale = WarmthScale;
    P.LandPrecipitation = Settings.LandPrecipitation;
    P.PrecipitationContrast = Settings.PrecipitationContrast;
    P.Seasonality = Settings.Seasonality;
    P.MaxWarmthAnomaly = Settings.MaxWarmthAnomaly;
    if (Shelf) P.Shelf = *Shelf;
    static_assert(sizeof(FVector4f) == 4 * sizeof(float), "FVector4f is four floats");
    return VoxelLandClimateGrid::BuildCells(Land.GetData(), Coast.GetData(), Resampled.Num() == W * H ? &Resampled[0].X : nullptr,
        W, H, P, &Out.Cells[0].X, LandHeight && LandHeight->Num() == W * H ? LandHeight->GetData() : nullptr);
}

TSharedPtr<const FVoxelLandClimateField, ESPMode::ThreadSafe> VoxelLandClimate::MakeField(const FVoxelLandClimateMap& Map,
    const FAtmosphereClimateMap* Sim, float LapseRate, const TArray<float>& Land, const TArray<float>& LandHeight, double SeaRadiusM,
    float IceMaxThickness)
{
    constexpr int32 W = VOXEL_LAND_CLIMATE_WIDTH;
    constexpr int32 H = VOXEL_LAND_CLIMATE_HEIGHT;
    // The drift's box filter (cells each way): 2 is about 400 km on an Earth-sized planet.
    constexpr int32 BlurCells = 2;
    if (!Map.IsValid()) return nullptr;
    const TSharedPtr<FVoxelLandClimateField, ESPMode::ThreadSafe> Out = MakeShared<FVoxelLandClimateField, ESPMode::ThreadSafe>();
    Out->Entries.SetNumZeroed(3 * W * H);
    FMemory::Memcpy(Out->Entries.GetData(), Map.Cells.GetData(), sizeof(FVector4f) * W * H);
    // The sim's drift on this grid (bilinear at the cells' centres, as BuildMap's climate).
    TArray<FVector4f> Resampled;
    if (Sim && Sim->Width > 0 && Sim->Height > 0 && Sim->Drift.Num() == Sim->Width * Sim->Height)
    {
        Resampled.SetNumUninitialized(W * H);
        ParallelFor(H, [&](int32 J)
            {
                for (int32 I = 0; I < W; I++)
                {
                    double X = 0.0, Y = 0.0, Z = 0.0;
                    VoxelLandClimateGrid::CellDirection(I, J, W, H, X, Y, Z);
                    Resampled[J * W + I] = Sim->SampleDrift(FVector(X, Y, Z));
                }
            });
        // A sim without wind (its drift all zero): the provisional winds.
        bool bAny = false;
        for (const FVector4f& D : Resampled)
            if (D.Z > 0.0f) { bAny = true; break; }
        if (!bAny) Resampled.Empty();
    }
    Out->bDriftFromSim = Resampled.Num() == W * H;
    VoxelLandClimateGrid::BuildDrift(Out->bDriftFromSim ? &Resampled[0].X : nullptr, W, H, BlurCells, &Out->Entries[W * H].X);
    // T5d2: the ice sheets: the cells today's climate ices at their mean height, on land.
    Out->IceMaxThickness = FMath::Max(IceMaxThickness, 0.0f);
    if (Out->IceMaxThickness > 0.0f && Land.Num() == W * H && LandHeight.Num() == W * H)
    {
        TArray<float> Iced;
        Iced.SetNumZeroed(W * H);
        for (int32 C = 0; C < W * H; C++)
        {
            if (Land[C] < 0.5f) continue;
            VoxelShader::FVoxelLandClimateCell Cell;
            Cell.SeaTemperature = Map.Cells[C].X;
            Cell.Precipitation = Map.Cells[C].Y;
            Cell.Seasonality = Map.Cells[C].Z;
            Cell.Continentality = Map.Cells[C].W;
            const float Margin = VoxelShader::VoxelLandClimateGlacierMargin(Cell, LapseRate, LandHeight[C], 0.0f);
            Iced[C] = VoxelShader::VoxelLandClimateRamp(-VoxelLandClimate::IceWidth, VoxelLandClimate::IceWidth, Margin);
        }
        VoxelLandClimateGrid::IceSheet(Iced.GetData(), LandHeight.GetData(), W, H, SeaRadiusM, &Out->Entries[2 * W * H].X);
    }
    // The climate by latitude over the land (a check that the cold sits where it should): per 15
    // degree band from the south pole, the land's share, its cells' mean year and summer at sea
    // level and at their mean height, the precipitation, and the share iced at their mean height.
    if (Land.Num() == W * H && LandHeight.Num() == W * H)
    {
        constexpr int32 Bands = 12;
        double Count[Bands] = {}, Cells[Bands] = {}, Year[Bands] = {}, Summer[Bands] = {}, SummerUp[Bands] = {}, Rain[Bands] = {}, IcedShare[Bands] = {};
        for (int32 J = 0; J < H; J++)
        {
            const double Lat = FMath::RadiansToDegrees(FMath::Asin(FMath::Clamp(VoxelLandClimateGrid::RowMu(J, H), -1.0, 1.0)));
            const int32 B = FMath::Clamp((int32)((Lat + 90.0) / 15.0), 0, Bands - 1);
            for (int32 I = 0; I < W; I++)
            {
                const int32 C = J * W + I;
                Cells[B] += 1.0;
                if (Land[C] < 0.5f) continue;
                const FVector4f& V = Map.Cells[C];
                VoxelShader::FVoxelLandClimateCell Cell;
                Cell.SeaTemperature = V.X;
                Cell.Precipitation = V.Y;
                Cell.Seasonality = V.Z;
                Cell.Continentality = V.W;
                Count[B] += 1.0;
                Year[B] += V.X;
                Summer[B] += V.X + FMath::Max(V.Z, 0.0f);
                SummerUp[B] += V.X + FMath::Max(V.Z, 0.0f) - LapseRate * FMath::Max(LandHeight[C], 0.0f) * 0.001f;
                Rain[B] += V.Y;
                IcedShare[B] += VoxelShader::VoxelLandClimateGlacierMargin(Cell, LapseRate, LandHeight[C], 0.0f) > 0.0f ? 1.0 : 0.0;
            }
        }
        FString Table;
        for (int32 B = 0; B < Bands; B++)
        {
            const double N = FMath::Max(Count[B], 1.0);
            Table += FString::Printf(TEXT("\n  %+3d..%+3d: land %3.0f%%, year %6.1f C, summer %6.1f C (at its height %6.1f), %6.0f mm, iced %3.0f%%"),
                -90 + 15 * B, -75 + 15 * B, 100.0 * Count[B] / FMath::Max(Cells[B], 1.0), Year[B] / N, Summer[B] / N, SummerUp[B] / N, Rain[B] / N,
                100.0 * IcedShare[B] / N);
        }
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] The land climate by latitude (local Z the pole; over the land, sea level unless said):%s"), *Table);
    }
    Out->LapseRate = LapseRate;
    Out->Hash = HashCombine(FCrc::MemCrc32(Out->Entries.GetData(), Out->Entries.Num() * sizeof(FVector4f)), GetTypeHash(LapseRate));
    return Out;
}

float VoxelLandClimate::ErgThreshold(float Frequency, float Coverage)
{
    if (!(Coverage > 0.0f)) return 2.0f;
    if (Coverage >= 1.0f) return -2.0f;
    // A Fibonacci lattice over the sphere (equal areas), the noise sorted, the quantile.
    constexpr int32 Count = 32768;
    TArray<float> Values;
    Values.SetNumUninitialized(Count);
    const double Golden = UE_DOUBLE_PI * (3.0 - FMath::Sqrt(5.0));
    ParallelFor(Count, [&](int32 i)
        {
            const double Z = 1.0 - 2.0 * ((double)i + 0.5) / (double)Count;
            const double R = FMath::Sqrt(FMath::Max(1.0 - Z * Z, 0.0));
            const double A = Golden * (double)i;
            Values[i] = VoxelShader::VoxelLandClimateErgNoise(VoxelShader::float3((float)(R * FMath::Cos(A)), (float)(R * FMath::Sin(A)), (float)Z), Frequency);
        });
    Values.Sort();
    return Values[FMath::Clamp((int32)((1.0f - Coverage) * (float)Count), 0, Count - 1)];
}

FVoxelLandClimatePointResult VoxelLandClimate::PointAt(const FVoxelLandClimateMap& Map, const FVoxelLandClimateSettings& Settings, float LapseRate,
    const FVector& LocalM, const FVector& NormalLocal, double SeaRadiusM)
{
    using namespace VoxelShader;
    FVoxelLandClimatePointResult Out;
    const double Radius = LocalM.Size();
    if (!Map.IsValid() || Radius < 1.0) return Out;
    const FVector Up = LocalM / Radius;
    const FVector4f V = Map.Sample(Up);
    FVoxelLandClimateCell Cell;
    Cell.SeaTemperature = V.X;
    Cell.Precipitation = V.Y;
    Cell.Seasonality = V.Z;
    Cell.Continentality = V.W;
    FVoxelLandClimateLook Look;
    Look.LapseRate = LapseRate;
    Look.Season = Settings.Season;
    Look.AspectWarming = Settings.AspectWarming;
    Look.BreakupAmplitude = Settings.BreakupAmplitude;
    Look.BreakupScale = Settings.BreakupScale;
    Look.SnowWidth = Settings.SnowWidth;
    const FVector Normal = NormalLocal.IsNearlyZero() ? Up : NormalLocal.GetSafeNormal();
    const FVoxelLandClimatePoint P = VoxelLandClimateAt(Cell, Look, float3((float)LocalM.X, (float)LocalM.Y, (float)LocalM.Z),
        float3((float)Up.X, (float)Up.Y, (float)Up.Z), float3((float)Normal.X, (float)Normal.Y, (float)Normal.Z), (float)(Radius - SeaRadiusM));
    Out.Temperature = P.Temperature;
    Out.Summer = P.Summer;
    Out.Now = P.Now;
    Out.Precipitation = P.Precipitation;
    Out.Aridity = P.Aridity;
    Out.Snow = P.Snow;
    Out.Glacier = P.Glacier;
    Out.Desert = P.Desert;
    Out.Vegetation = P.Vegetation;
    return Out;
}

FVoxelLandShelfParams VoxelLandClimate::ShelfParamsFor(const FVoxelTerrestrialSettings& Settings, uint32 Seed)
{
    FVoxelLandShelfParams P;
    P.bOn = Settings.IceShelfHeight > 0.0;
    P.Temperature = Settings.IceShelfTemperature;
    P.FrontNoise = FMath::Max(Settings.IceShelfFrontNoise, 0.0);
    P.Height = FMath::Max(Settings.IceShelfHeight, 0.0);
    P.Rise = FMath::Max(Settings.IceShelfRise, 0.0);
    P.Roll = FMath::Max(Settings.IceShelfRoll, 0.0);
    P.Draft = FMath::Max(Settings.IceShelfDraft, 0.0);
    P.Seed = Seed ^ 0x1CE5E1F0u;
    return P;
}

TSharedPtr<const FVoxelLandBake, ESPMode::ThreadSafe> VoxelLandClimate::MakeIceShelfBake(const FVoxelLandBake& Base,
    const FVoxelLandClimateField& Field, const FVoxelTerrestrialSettings& Settings)
{
    const FVoxelLandShelfParams P = ShelfParamsFor(Settings, Base.PlateSeed);
    if (!P.bOn || !Field.IsValid() || Base.Height.Num() != Base.NumCells()) return nullptr;
    const double StartSeconds = FPlatformTime::Seconds();
    const int32 N = Base.N;
    const int32 Side = Base.Side();
    const int32 NumCells = Base.NumCells();
    const int32 Pad = FVoxelLandBake::Pad;

    TSharedPtr<FVoxelLandBake, ESPMode::ThreadSafe> Out = MakeShared<FVoxelLandBake, ESPMode::ThreadSafe>();
    Out->N = N;
    Out->SeaRadius = Base.SeaRadius;
    Out->SettingsCheck = Base.SettingsCheck;
    Out->CrustThreshold = Base.CrustThreshold;
    Out->bFromCache = Base.bFromCache;
    Out->bEroded = Base.bEroded;
    Out->SolveSteps = Base.SolveSteps;
    Out->SolveResidual = Base.SolveResidual;
    Out->ReliefFactor = Base.ReliefFactor;
    Out->Height = Base.Height;
    Out->Attr = Base.Attr;
    Out->Form = Base.Form;
    Out->Province = Base.Province;
    Out->Flow = Base.Flow;
    Out->Water = Base.Water;
    Out->Receiver = Base.Receiver;
    Out->Lakes = Base.Lakes;
    Out->TanTable = Base.TanTable;
    Out->Plates = Base.Plates;
    Out->PlateSeed = Base.PlateSeed;
    Out->PlateRadiusM = Base.PlateRadiusM;
    Out->PlateWidthScale = Base.PlateWidthScale;
    {
        // The rivers are Base's (the chain fades them on the shelf: VoxelLandGully.ush).
        FScopeLock Lock(&Base.RiverLock);
        Out->Rivers = Base.Rivers;
    }

    // The drainage off the land: the least a cell holds (its own area).
    const bool bFlow = Out->Flow.Num() == NumCells;
    const bool bWater = Out->Water.Num() == NumCells;
    const bool bForm = Out->Form.Num() == NumCells;
    const bool bProvince = Out->Province.Num() == NumCells;
    const bool bAttr = Out->Attr.Num() == NumCells;
    float MinFlow = 0.0f;
    if (bFlow)
    {
        MinFlow = 3.0e38f;
        for (const FFloat16 F : Out->Flow) MinFlow = FMath::Min(MinFlow, (float)F);
    }

    // The year's mean at sea level from the field's map (its first block; bilinear, as the chain
    // and the material read it).
    const FVector4f* Cells = Field.Entries.GetData();
    auto SeaTemperatureAt = [Cells](const FVector& D)
    {
        const VoxelShader::FVoxelLandClimateTaps T = VoxelShader::VoxelLandClimateTapsOf(VoxelShader::float3((float)D.X, (float)D.Y, (float)D.Z));
        const int32 W = VOXEL_LAND_CLIMATE_WIDTH;
        const double A = (double)Cells[T.Y0 * W + T.X0].X * (1.0 - T.FX) + (double)Cells[T.Y0 * W + T.X1].X * T.FX;
        const double B = (double)Cells[T.Y1 * W + T.X0].X * (1.0 - T.FX) + (double)Cells[T.Y1 * W + T.X1].X * T.FX;
        return A * (1.0 - T.FY) + B * T.FY;
    };

    // The front's octaves down to a few cells.
    const double MaxFrequency = FMath::Max((double)N / 4.0, 8.0);
    TArray<float> Influence;
    Influence.SetNumZeroed(NumCells);
    // Where the plateau reaches (its top; 0 outside its front): the sea it cuts off is filled to it.
    TArray<float> FrontTop;
    FrontTop.SetNumZeroed(NumCells);
    std::atomic<int32> RaisedCells(0);
    std::atomic<int32> TouchedCells(0);
    // The kept lakes the shelf reaches (by their level: a lake's cells share it), removed whole.
    FCriticalSection LakeLock;
    TSet<float> ShelfLakeLevels;
    ParallelFor(6 * Side, [&](int32 Row)
        {
            const int32 Face = Row / Side;
            const int32 J = Row % Side;
            int32 Raised = 0;
            int32 Touched = 0;
            for (int32 I = 0; I < Side; I++)
            {
                const int32 Index = Face * Side * Side + J * Side + I;
                const FVector D = VoxelCubeSphere::Direction(Face, VoxelCubeSphere::CellCentre(I - Pad, N), VoxelCubeSphere::CellCentre(J - Pad, N));
                const double T = SeaTemperatureAt(D);
                if (VoxelLandShelf::IsOutside(T, P)) continue;
                const double H = (double)Base.Height[Index];
                const VoxelLandShelf::FCell C = VoxelLandShelf::Apply(D.X, D.Y, D.Z, T, H, P, MaxFrequency);
                Out->Height[Index] = (float)C.Height;
                Influence[Index] = (float)C.Influence;
                FrontTop[Index] = C.Front > 0.0 ? (float)C.Top : 0.0f;
                if (C.Influence > 0.0) Touched++;
                const double K = C.Raised;
                if (K <= 0.0) continue;
                if (J >= Pad && J < Pad + N && I >= Pad && I < Pad + N && K >= 0.5) Raised++;
                const double Keep = 1.0 - K;
                // Its form flattened (no hills: the plains' 0, smooth), no strata, no drainage of its
                // own, and no lake.
                if (bForm)
                {
                    const uint16 F = Out->Form[Index];
                    const uint16 Hilly = (uint16)FMath::RoundToInt((double)(F & 0xFFu) * Keep);
                    const uint16 Character = (uint16)FMath::RoundToInt((double)(F >> 8) * Keep);
                    Out->Form[Index] = (uint16)(Hilly | (Character << 8));
                }
                if (bProvince)
                    Out->Province[Index] = (uint8)FMath::RoundToInt((double)Out->Province[Index] * Keep);
                if (bFlow)
                    Out->Flow[Index] = FFloat16((float)(MinFlow + ((double)(float)Out->Flow[Index] - MinFlow) * Keep));
                if (bWater && Out->Water[Index] > -1.0e38f)
                {
                    FScopeLock Lock(&LakeLock);
                    ShelfLakeLevels.Add(Out->Water[Index]);
                }
            }
            RaisedCells.fetch_add(Raised);
            TouchedCells.fetch_add(Touched);
        });
    // Nothing reached (a warm planet): the planet's bake is the terrain's.
    if (TouchedCells.load() == 0)
    {
        UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Ice shelf: none (nowhere colder than %.1f C at sea level)."), P.Temperature);
        return nullptr;
    }
    // The lakes it reaches go whole (a lake cut where the shelf rises under it would stand in the ice).
    if (ShelfLakeLevels.Num() > 0)
    {
        ParallelFor(NumCells, [&](int32 Index)
            {
                if (Out->Water[Index] > -1.0e38f && ShelfLakeLevels.Contains(Out->Water[Index]))
                {
                    Out->Water[Index] = -3.0e38f;
                    if (bAttr) Out->Attr[Index] = (uint16)(Out->Attr[Index] & ~FVoxelLandBake::LakeBit);
                }
            });
        Out->Lakes.RemoveAll([&](const FVoxelLandLake& Lake) { return ShelfLakeLevels.Contains(Lake.Level); });
    }

    // The sea mask for the new heights, then the sea the shelf cut off (it was the sea's, now in no
    // sea region): inside the plateau's front filled to its top (no dry pits in the ice), outside it
    // kept as sea (enclosed water at sea level, as a polynya's).
    Out->BuildSeaConnected();
    int32 Filled = 0;
    int32 Enclosed = 0;
    if (bAttr && Base.Attr.Num() == NumCells)
    {
        for (int32 Index = 0; Index < NumCells; Index++)
        {
            const bool bWasSea = (Base.Attr[Index] & FVoxelLandBake::SeaBit) != 0;
            const bool bIsSea = (Out->Attr[Index] & FVoxelLandBake::SeaBit) != 0;
            if (!bWasSea || bIsSea || Out->Height[Index] > 0.0f) continue;
            if (FrontTop[Index] > 0.0f)
            {
                Out->Height[Index] = FrontTop[Index];
                Influence[Index] = 1.0f;
                if (bForm) Out->Form[Index] = 0;
                if (bProvince) Out->Province[Index] = 0;
                if (bFlow) Out->Flow[Index] = FFloat16(MinFlow);
                Filled++;
            }
            else
            {
                Out->Attr[Index] = (uint16)(Out->Attr[Index] | FVoxelLandBake::SeaBit);
                Enclosed++;
            }
        }
    }
    Out->UpdateRange();
    Out->BuildRelief();
    // The peaks' relief gives way at its front and on it (the front's own rise isn't a mountain's).
    if (Out->Relief.Num() == NumCells)
    {
        ParallelFor(NumCells, [&](int32 Index)
            {
                const float Inf = Influence[Index];
                if (Inf > 0.0f)
                    Out->Relief[Index] = (uint8)FMath::RoundToInt((float)Out->Relief[Index] * FMath::Sqrt(FMath::Max(1.0f - Inf, 0.0f)));
            });
    }
    // The strata's threshold stays the planet's (the shelf isn't land of any province: counting it
    // would spread the strata on the continents, by the capture).
    Out->ProvinceLand = Base.ProvinceLand;

    // A key of its own (the caches and fields made from a bake key on it): Base's, the shelf's
    // settings and the climate's.
    uint32 Key = HashCombine(Base.Key, Field.Hash);
    for (const double V : { P.Temperature, P.FrontNoise, P.Height, P.Rise, P.Roll, P.Draft })
        Key = HashCombine(Key, ::GetTypeHash(V));
    Key = HashCombine(Key, 0x5E1Fu);
    Out->Key = Key == Base.Key ? Key ^ 0x80000000u : Key;

    const int32 Interior = 6 * N * N;
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Ice shelf: %.2f%% of the surface (front %.1f C, wanders %.1f C; %.0f m rising %.0f m, roll %.0f m, draft %.0f m), heights %.0f .. %.0f m, %d sea cells; %.2f s."),
        100.0 * (double)RaisedCells.load() / (double)FMath::Max(Interior, 1), P.Temperature, P.FrontNoise, P.Height, P.Rise, P.Roll, P.Draft,
        Out->MinHeight, Out->MaxHeight, Out->SeaCells, FPlatformTime::Seconds() - StartSeconds);
    UE_LOG(LogVoxelPlugin, Log, TEXT("[VoxelLand] Ice shelf: %d lakes under it removed; %d cells of sea it cut off filled, %d kept as enclosed sea."),
        ShelfLakeLevels.Num(), Filled, Enclosed);
    return Out;
}

