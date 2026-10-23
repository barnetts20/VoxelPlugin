// O6b: a top-down relief map of the sea as the hook draws it (hook_emulation.h, all bands, mips
// by the pixel's footprint), around a cube corner where three faces' charts meet, to look for
// seams; optionally with the charts' borders traced (where the heaviest chart changes).
//
//   g++ -O2 -std=c++20 -ffp-contract=off -IUEStandIn -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private
//       -I../../Shaders/Private -I. hook_view.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWavesFFT.cpp
//       ../../Source/VoxelPlugin/Private/VoxelOceanWeather.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWeatherTable.cpp -o hook_view
//   ./hook_view <wind m/s> <extent km> <borders 0|1> <out.ppm>
#include "hook_emulation.h"

#include <cstdio>
#include <cstdlib>

using namespace VoxelShader;
using namespace HookEmulation;

int main(int argc, char** argv)
{
    if (argc < 5) { std::fprintf(stderr, "usage: hook_view wind extent_km borders out.ppm\n"); return 1; }
    const double Wind = std::atof(argv[1]), Extent = std::atof(argv[2]) * 1.0e5;
    const bool bBorders = std::atoi(argv[3]) != 0;
    const double Radius = 1.075e8;
    FVoxelOceanFFTSettings Settings;
    Settings.Seed = 7;
    Settings.WindSpeed = Wind;
    const FVoxelOceanWavesFFT Model(Settings, Radius);
    FVoxelOceanWaveContext Context;
    Context.Time = 300.0;
    Context.Radius = Radius;
    Context.Environment.WindSpeed = Wind;
    Context.Environment.WindHeading = 0.7;
    const auto Weather = ConstantWeather(Model, Wind, 0.7);
    Context.Weather = Weather;
    FTextures Tx;
    const VoxelOceanCharts::FTable& Charts = Weather->Table;
    Tx.Charts = &Charts;
    const uint32_t Tick = VoxelOceanSpectrum::TickOf(Context.Time);
    for (int s = 0; s < VoxelOceanSpectrum::SlotCount; s++)
    {
        const OceanFFTEmulation::FGPUSlot G = OceanFFTEmulation::LongCopy(OceanFFTEmulation::EmulateGPU(Model.GetSlots()[s], Tick));
        Tx.Displacement.Build(s, G.Displacement);
        Tx.Derivatives.Build(s, G.Derivatives);
    }
    // A frame at the cube corner (1, 1, 1).
    const FVector C = FVector(1, 1, 1).GetSafeNormal();
    const FVector E1 = (FVector(1, -1, 0)).GetSafeNormal(), E2 = C ^ E1;
    const FVector Ref = C * (Radius + 2.0e5);
    FVoxelOceanWaveShaderParams SP;
    Model.GetShaderParams(Context, Ref, FTransform::Identity, SP);
    const FParams W = ParamsOf(SP);

    const int Size = 1024;
    const float Footprint = float(Extent / Size);
    std::vector<unsigned char> Img(size_t(Size) * Size * 3);
    std::vector<int> Dominant(size_t(Size) * Size);
    const float3 L = normalize(float3(0.6f, 0.3f, 0.55f));
    for (int j = 0; j < Size; j++)
        for (int i = 0; i < Size; i++)
        {
            const double X = ((i + 0.5) / Size - 0.5) * Extent, Y = ((j + 0.5) / Size - 0.5) * Extent;
            const FVector Point = (C * Radius + E1 * X + E2 * Y).GetSafeNormal() * Radius;
            const FVector Up = Point.GetSafeNormal();
            const FPoint Pt = HookEvaluate(Tx, float3(float(Point.X - Ref.X), float(Point.Y - Ref.Y), float(Point.Z - Ref.Z)), float3(float(Up.X), float(Up.Y), float(Up.Z)),
                Footprint, Footprint, W, 3u);
            // The normal in the local frame (x along E1, y along E2, z up).
            const float3 N = Pt.R.Normal;
            const float3 Local(dot(N, float3(float(E1.X), float(E1.Y), float(E1.Z))), dot(N, float3(float(E2.X), float(E2.Y), float(E2.Z))), dot(N, float3(float(Up.X), float(Up.Y), float(Up.Z))));
            const float G = std::clamp(0.5f + 2.5f * (dot(Local, L) - L.z), 0.0f, 1.0f);
            // The heaviest chart, for the borders.
            const FVoxelOceanChartCandidates Cand = VoxelOceanChartCandidatesOf(float3(float(Up.X), float(Up.Y), float(Up.Z)), Charts.N);
            const FVoxelOceanChartBlend B = VoxelOceanChartBlendOf(Cand.Count, VoxelOceanChartDistancesOf(Cand, float3(float(Up.X), float(Up.Y), float(Up.Z))), Charts.N);
            int Best = 0;
            for (int m = 1; m < B.Count; m++) if (B.Weight[m] > B.Weight[Best]) Best = m;
            Dominant[size_t(j) * Size + i] = Cand.Texel[B.Index[Best]].x * 100000 + Cand.Texel[B.Index[Best]].y;
            for (int c = 0; c < 3; c++) Img[(size_t(j) * Size + i) * 3 + c] = (unsigned char)(255.0f * G);
        }
    if (bBorders)
        for (int j = 1; j < Size; j++)
            for (int i = 1; i < Size; i++)
                if (Dominant[size_t(j) * Size + i] != Dominant[size_t(j) * Size + i - 1] || Dominant[size_t(j) * Size + i] != Dominant[size_t(j - 1) * Size + i])
                {
                    unsigned char* P = &Img[(size_t(j) * Size + i) * 3];
                    P[0] = 255; P[1] = (unsigned char)(P[1] * 0.4); P[2] = (unsigned char)(P[2] * 0.4);
                }
    FILE* F = std::fopen(argv[4], "wb");
    std::fprintf(F, "P6\n%d %d\n255\n", Size, Size);
    std::fwrite(Img.data(), 1, Img.size(), F);
    std::fclose(F);
    return 0;
}
