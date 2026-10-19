// Pages, slots and triangles the selection produces, for the page memory estimate.
#include "CoreMinimal.h"
#include VOT_HEADER
#include <cstdio>
#include <random>
#include <map>

using namespace VoxelOceanTiles;
using Key = FVoxelOceanTileKey;

int main()
{
    const double Radius = 1.075e8;
    for (int32 Quads : { 16, 32 })
    {
        FSelectParams P;
        P.Radius = Radius;
        P.MaxLevel = LevelForSpacing(Radius, Quads, 50.0);
        P.LodDistanceFactor = 2.5;
        std::printf("Quads %d, MaxLevel %d, spacing %.1f cm\n", Quads, P.MaxLevel, NominalArc(P.MaxLevel) * Radius / Quads);
        std::mt19937_64 Rng(5);
        std::normal_distribution<double> G;
        for (double Altitude : { 2e2, 2e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9 })
        {
            double Tiles = 0, Pages = 0, MaxPages = 0, MaxTiles = 0;
            const int32 N = 40;
            for (int32 c = 0; c < N; c++)
            {
                FVector D = FVector(G(Rng), G(Rng), G(Rng)).GetSafeNormal();
                TArray<Key> Sel; TSet<Key> Split;
                Select(D * (Radius + Altitude), P, nullptr, Sel, Split);
                TSet<Key> PageSet;
                for (const Key& K : Sel) PageSet.Add(PageOf(K));
                Tiles += Sel.Num(); Pages += PageSet.Num();
                MaxPages = std::max(MaxPages, (double)PageSet.Num()); MaxTiles = std::max(MaxTiles, (double)Sel.Num());
            }
            const double V = (Quads + 1.0) * (Quads + 1.0), T = 2.0 * Quads * Quads;
            const double PageBytes = PageSlots * (V * 48.0 + T * 12.0);
            std::printf("  altitude %8.0f m: %6.0f tiles (%5.0f max), %5.0f pages (%4.0f max), %4.1f tiles per page; %.2f M triangles drawn (%.2f M with empty slots), %.0f MB of pages (%.0f max)\n",
                Altitude / 100.0, Tiles / N, MaxTiles, Pages / N, MaxPages, Tiles / Pages, Tiles / N * T / 1e6, Pages / N * PageSlots * T / 1e6,
                Pages / N * PageBytes / 1048576.0, MaxPages * PageBytes / 1048576.0);
        }
    }
    return 0;
}
