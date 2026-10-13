// VoxelLandRelief.h - The bake's local relief (Design/SurfaceLook.md, T3): per cell, the height
// range (m) over the cells within about 2.2 cells (a 5 x 5 window without its corners), heights
// below sea level taken as sea level. The peaks layer (VoxelLandPeaks.ush) scales with it: it is
// the mountains' relief at the bake's scale, near zero on plains.
//
// Derived from the heights when a bake is ready (FVoxelLandBake::BuildRelief, on every path:
// GPU, CPU, cache), not stored: the cache file and the bake key don't change. Each face reads its
// own padded grid; the window stays inside it, so the outermost two apron cells see a smaller
// window (the sampler reads the inner one of them within a cell of a face edge, where the faces
// blend: a slightly lower relief there, under 1% of the peaks' amplitude).
//
// Stored in 8 bits a cell: R = q^2 x VOXEL_LAND_RELIEF_SCALE (12 km at 255; steps of 5% at 300 m,
// 2.7% at 1 km), decoded with two multiplications (VoxelLandAccess.ush), the same on every host.
// Plain C++ without engine types: the harnesses run the same code.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

// Keep equal to VOXEL_LAND_RELIEF_SCALE (VoxelLandAccess.ush).
#define VOXEL_LAND_RELIEF_SCALE_CPP (12000.0f / 65025.0f)

namespace VoxelLandRelief
{
    constexpr float Scale = VOXEL_LAND_RELIEF_SCALE_CPP;   // m per q^2

    inline uint8_t Encode(float ReliefM)
    {
        if (!(ReliefM > 0.0f)) return 0;
        const float Q = std::sqrt(ReliefM / Scale);
        return (uint8_t)std::min(255.0f, std::floor(Q + 0.5f));
    }

    inline float Decode(uint32_t Q)
    {
        const float F = (float)Q;
        return F * F * Scale;
    }

    /** The relief of one row of one face: Height and Out are the bake's arrays (6 faces of
     *  Side x Side cells, rows along U), Row the padded row index. */
    inline void BuildRow(const float* Height, int32_t Side, int32_t Face, int32_t Row, uint8_t* Out)
    {
        const int64_t Base = (int64_t)Face * Side * Side;
        for (int32_t i = 0; i < Side; i++)
        {
            float Lo = 3.0e38f;
            float Hi = -3.0e38f;
            for (int32_t dj = -2; dj <= 2; dj++)
            {
                const int32_t j = Row + dj;
                if (j < 0 || j >= Side) continue;
                for (int32_t di = -2; di <= 2; di++)
                {
                    if (di * di + dj * dj > 5) continue;
                    const int32_t k = i + di;
                    if (k < 0 || k >= Side) continue;
                    const float H = std::max(Height[Base + (int64_t)j * Side + k], 0.0f);
                    Lo = std::min(Lo, H);
                    Hi = std::max(Hi, H);
                }
            }
            Out[Base + (int64_t)Row * Side + i] = Encode(Hi - Lo);
        }
    }
}
