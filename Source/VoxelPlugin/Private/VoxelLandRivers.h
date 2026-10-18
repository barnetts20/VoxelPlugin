// VoxelLandRivers.h - The rivers' hydraulic geometry (Design/RiversLakes.md 2.3), as the C++ side
// sizes them: the channel's width, the belt (the floodplain's and the valley's measure) and the
// valley's half-width for a drainage area. The same
// formulas run on the GPU (VoxelLandRivers.ush for the samples, VoxelLandRiverBuild.ush for the
// tiles' corridors); keep them equal. The network and its tables are made on the GPU
// (VoxelLandRiverBuild.h, R1b; R1 made them here on the CPU). Plain C++ without engine types:
// the harnesses use it too.

#pragma once

#include <cmath>

namespace VoxelLandRivers
{
    /** A channel's width (m) for a log2 drainage area (m^2): Scale x 2.35 (km^2)^0.4 (R3: 20 m at
     *  200 km^2, 235 m at 100,000 km^2, 1 km at 3 million: Earth's rivers widen about as Q^0.5
     *  and Q about as A^0.8). */
    inline double WidthM(double Log2Area, double Scale)
    {
        return Scale * 2.35 * std::exp2(0.4 * (Log2Area - 19.931568569324174));
    }

    /** The belt (m) the floodplain and the valley are measured in: 2.5 sqrt(km^2) (R1/R2's channel
     *  width: 35 m at 200 km^2, 790 m at 100,000 km^2), whatever the channel's width. */
    inline double BeltM(double Log2Area)
    {
        return 2.5 * std::sqrt(std::exp2(Log2Area) * 1.0e-6);
    }

    /** The valley's half-width (m) for a belt: Scale x (400 m + 15 belts). */
    inline double ValleyHalfWidthM(double Belt, double Scale)
    {
        return Scale * (400.0 + 15.0 * Belt);
    }

    /** The corridor's reach over the valley's half-width (the tiles list a segment as far; R2's
     *  profiles stay inside it). */
    constexpr double ReachScale = 1.25;
}
