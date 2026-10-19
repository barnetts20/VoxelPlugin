// VoxelOceanDepthProbe.h - The terrain's depth around the camera for the underwater pass (stage
// O8): the surface map (VoxelUnderwaterMap.ush) shoals its waves as the drawn surface does, so it
// needs the depth the mesh carries, which a post-process pass doesn't have. The ocean pages'
// compute run samples it (VoxelOceanPage.usf, ProbeCS: the same density, edits and inland-basin
// cull as a tile's vertices) on Levels nested grids of Size^2 points around the selection camera,
// whenever a grid moves or the terrain changes; the underwater pass reads it bilinearly
// (VoxelUnderwaterPass.usf).
//
// A level's grid lies in the plane tangent at its anchor (a sphere point near the camera's
// radial, snapped on the camera's cube face to Snap spacings, so it moves rarely), E1 along the
// face's u axis made tangent, E2 = Up x E1; its point (i, j) is the sea sphere's point under
//   Anchor + ((i + 0.5) - Size / 2) Spacing E1 + ((j + 0.5) - Size / 2) Spacing E2,
// sampled at the terrain lattice point nearest it, with the spacing as the footprint (the mesh's
// tiles near the camera are about as fine). Planet frame, double.

#pragma once

#include "CoreMinimal.h"
#include "RenderGraphResources.h"
#include <atomic>

namespace VoxelOceanDepthProbe
{
    constexpr int32 Levels = 4;
    constexpr int32 Size = 64;
    /** The finest spacing (cm) and each next level's factor: 0.5, 2, 8 and 32 m (32 m to 2 km
     *  across). */
    constexpr double Spacing0 = 50.0;
    constexpr double Ratio = 4.0;
    /** A level's anchor moves in steps of this many spacings. */
    constexpr int32 Snap = 8;
    constexpr int32 Points = Levels * Size * Size;

    struct FLevels
    {
        FVector Anchor[Levels];      // planet frame, on the sphere (cm)
        FVector E1[Levels];
        FVector E2[Levels];
        double Spacing[Levels] = {};

        bool operator==(const FLevels& Other) const
        {
            for (int32 L = 0; L < Levels; L++)
            {
                if (Anchor[L] != Other.Anchor[L] || E1[L] != Other.E1[L] || Spacing[L] != Other.Spacing[L]) return false;
            }
            return true;
        }
        bool operator!=(const FLevels& Other) const { return !(*this == Other); }
    };

    /** The levels around a camera (planet frame) over the sea sphere of Radius (cm). */
    inline FLevels MakeLevels(const FVector& CameraPlanet, double Radius)
    {
        FLevels Out;
        const FVector Dir = CameraPlanet.GetSafeNormal();
        int32 Axis = 0;
        if (FMath::Abs(Dir.Y) > FMath::Abs(Dir[Axis])) Axis = 1;
        if (FMath::Abs(Dir.Z) > FMath::Abs(Dir[Axis])) Axis = 2;
        FVector N(0, 0, 0), A(0, 0, 0), B(0, 0, 0);
        N[Axis] = Dir[Axis] >= 0.0 ? 1.0 : -1.0;
        A[(Axis + 1) % 3] = 1.0;
        B[(Axis + 2) % 3] = 1.0;
        const double Along = FMath::Max(Dir | N, 1.0e-6);
        const double U = (Dir | A) / Along;
        const double V = (Dir | B) / Along;
        double Spacing = Spacing0;
        for (int32 Level = 0; Level < Levels; Level++)
        {
            const double Step = Snap * Spacing / FMath::Max(Radius, 1.0);
            const double SU = FMath::FloorToDouble(U / Step + 0.5) * Step;
            const double SV = FMath::FloorToDouble(V / Step + 0.5) * Step;
            const FVector Up = (N + A * SU + B * SV).GetSafeNormal();
            Out.Anchor[Level] = Up * Radius;
            Out.E1[Level] = (A - Up * (A | Up)).GetSafeNormal();
            Out.E2[Level] = Up ^ Out.E1[Level];
            Out.Spacing[Level] = Spacing;
            Spacing *= Ratio;
        }
        return Out;
    }

    /** A grid point on the sphere (planet frame). */
    inline FVector PointOf(const FLevels& L, int32 Level, int32 I, int32 J, double Radius)
    {
        const double Half = 0.5 * Size;
        const FVector P = L.Anchor[Level] + L.E1[Level] * ((I + 0.5 - Half) * L.Spacing[Level]) + L.E2[Level] * ((J + 0.5 - Half) * L.Spacing[Level]);
        return P.GetSafeNormal() * Radius;
    }

    /** Every point's terrain lattice point (3 ints each, level by level, rows along I). */
    inline void LatticeOf(const FLevels& L, double Radius, double HalfFinest, TArray<int32>& Out)
    {
        Out.SetNumUninitialized(Points * 3);
        const double Inv = 1.0 / FMath::Max(HalfFinest, 1.0e-3);
        int32 k = 0;
        for (int32 Level = 0; Level < Levels; Level++)
            for (int32 J = 0; J < Size; J++)
                for (int32 I = 0; I < Size; I++)
                {
                    const FVector P = PointOf(L, Level, I, J, Radius) * Inv;
                    Out[k++] = (int32)FMath::Clamp(FMath::RoundToDouble(P.X), -2147483000.0, 2147483000.0);
                    Out[k++] = (int32)FMath::Clamp(FMath::RoundToDouble(P.Y), -2147483000.0, 2147483000.0);
                    Out[k++] = (int32)FMath::Clamp(FMath::RoundToDouble(P.Z), -2147483000.0, 2147483000.0);
                }
    }
}

/** The probe's last result (render thread): the depths (cm, Points floats, as LatticeOf orders
 *  them) and the levels they were sampled on. The underwater pass reads it. */
struct FVoxelOceanProbeState
{
    TRefCountPtr<FRDGPooledBuffer> Depth;
    VoxelOceanDepthProbe::FLevels Levels;
    bool bValid = false;
    /** Set by the render thread when a probe was skipped (its shader still compiling): the game
     *  thread probes again (FVoxelOceanSurface::UpdateProbe). */
    std::atomic<bool> bRetry{ false };
};
