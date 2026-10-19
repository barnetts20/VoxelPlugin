// O7: a map of the procedural weather (VoxelOceanWeather.cpp): the whole planet, longitude across
// and latitude up, coloured by the wind speed (calm dark blue, 10 m/s pale, 20 m/s and over red),
// short strokes along the wind and the storms' radii traced.
//
//   g++ -O2 -std=c++20 -IUEStandIn -I../../Source/VoxelPlugin/Public -I../../Source/VoxelPlugin/Private -I../../Shaders/Private -I.
//       weather_view.cpp ../../Source/VoxelPlugin/Private/VoxelOceanWeather.cpp -o weather_view
//   ./weather_view <weather time s> <out.ppm>
#include "UEStandIn/CoreMinimal.h"
#include "VoxelOceanWeather.h"

#include <cstdio>
#include <cstdlib>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 3) { std::fprintf(stderr, "usage: weather_view time out.ppm\n"); return 1; }
    const double Time = std::atof(argv[1]);
    const double Radius = 1.075e8;
    FVoxelOceanWeatherSettings Settings;
    Settings.Seed = 3;
    const FVoxelOceanProceduralWeather Weather(Settings, Radius);
    const int W = 1024, H = 512;
    std::vector<unsigned char> Img(size_t(W) * H * 3);
    auto DirOf = [&](double X, double Y) {
        const double Lon = (X / W) * 2.0 * 3.14159265358979 - 3.14159265358979;
        const double Lat = (0.5 - Y / H) * 3.14159265358979;
        return FVector(std::cos(Lat) * std::cos(Lon), std::cos(Lat) * std::sin(Lon), std::sin(Lat));
    };
    TArray<FVoxelOceanStorm> Storms;
    Weather.StormsAt(Time, Storms);
    TArray<FVoxelOceanEnvironment> Row;
    Row.SetNum(W);
    TArray<FVector> Points;
    Points.SetNum(W);
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++) Points[x] = DirOf(x + 0.5, y + 0.5) * Radius;
        Weather.SampleWeatherBatch(TConstArrayView<FVector>(Points.GetData(), W), Time, TArrayView<FVoxelOceanEnvironment>(Row.GetData(), W));
        for (int x = 0; x < W; x++)
        {
            const double U = Row[x].WindSpeed;
            // Calm dark blue, 10 m/s pale, 20 m/s red.
            double R, G, B;
            if (U < 10.0) { const double t = U / 10.0; R = 0.05 + 0.80 * t; G = 0.10 + 0.78 * t; B = 0.35 + 0.57 * t; }
            else { const double t = std::min((U - 10.0) / 10.0, 1.0); R = 0.85 + 0.15 * t; G = 0.88 - 0.73 * t; B = 0.92 - 0.82 * t; }
            unsigned char* P = &Img[(size_t(y) * W + x) * 3];
            P[0] = (unsigned char)(255 * R); P[1] = (unsigned char)(255 * G); P[2] = (unsigned char)(255 * B);
        }
    }
    // Strokes along the wind on a coarse grid.
    for (int gy = 8; gy < H; gy += 16)
        for (int gx = 8; gx < W; gx += 16)
        {
            const FVector Dir = DirOf(gx, gy);
            const FVoxelOceanEnvironment E = Weather.SampleWeather(Dir * Radius, Time);
            const double Len = std::min(E.WindSpeed, 20.0) * 0.6;
            // East is +x, north is -y on the map.
            const double DX = std::sin(E.WindHeading), DY = -std::cos(E.WindHeading);
            for (int s = 0; s <= int(Len); s++)
            {
                const int px = gx + int(DX * s), py = gy + int(DY * s);
                if (px < 0 || px >= W || py < 0 || py >= H) continue;
                unsigned char* P = &Img[(size_t(py) * W + px) * 3];
                const unsigned char V = s == int(Len) ? 0 : 30;
                P[0] = V; P[1] = V; P[2] = V;
            }
        }
    // The storms' radii.
    for (const FVoxelOceanStorm& S : Storms)
    {
        if (S.Strength < 0.05) continue;
        FVector A, B;
        A = (FVector(0, 0, 1) ^ S.Centre).GetSafeNormal();
        if (A.Size() < 0.5) A = FVector(1, 0, 0);
        B = S.Centre ^ A;
        for (int k = 0; k < 720; k++)
        {
            const double Ang = k * 3.14159265358979 / 360.0;
            const FVector P = (S.Centre * std::cos(S.Radius) + (A * std::cos(Ang) + B * std::sin(Ang)) * std::sin(S.Radius)).GetSafeNormal();
            const double Lon = std::atan2(P.Y, P.X), Lat = std::asin(P.Z);
            const int px = int((Lon + 3.14159265358979) / (2.0 * 3.14159265358979) * W), py = int((0.5 - Lat / 3.14159265358979) * H);
            if (px < 0 || px >= W || py < 0 || py >= H) continue;
            unsigned char* Q = &Img[(size_t(py) * W + px) * 3];
            Q[0] = 255; Q[1] = 255; Q[2] = 255;
        }
    }
    FILE* F = std::fopen(argv[2], "wb");
    std::fprintf(F, "P6\n%d %d\n255\n", W, H);
    std::fwrite(Img.data(), 1, Img.size(), F);
    std::fclose(F);
    return 0;
}
