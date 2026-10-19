// VoxelUnderwaterMap.h - The CPU half of the underwater pass's surface map
// (VoxelUnderwaterMap.ush): where each level's grid lies around a view's camera.
//
// A level's anchor is a sphere point next to the camera's radial: the camera's direction on its
// cube face (gnomonic u, v), snapped to steps of texel / radius in u and v, put back on the
// sphere. The grid lies in the plane tangent at the anchor, E1 along the face's u axis (made
// tangent), E2 = Up x E1. Between snaps a level's grid stays put as the camera moves; a snap
// moves it by half a texel to a texel (a gnomonic step shrinks toward the face's edges), so its
// points don't land on the old ones and the map's interpolation shifts once, by what its texel
// can hold (it is mipped to the texel). Everything is formed in double, in the planet frame, then
// turned to world axes and taken from the camera, as the pass wants it.
//
// Size and Levels go to the shaders through their compile environment
// (VOXEL_UNDERWATER_MAP_SIZE, VOXEL_UNDERWATER_MAP_LEVELS).

#pragma once

#include "CoreMinimal.h"

namespace VoxelUnderwaterMap
{
    /** The levels, their grids' size, the finest texel (cm) and each next level's factor. */
    constexpr int32 Levels = 5;
    constexpr int32 Size = 256;
    constexpr double Texel0 = 2.0;
    constexpr double Ratio = 4.0;

    struct FLevels
    {
        /** Per level: the anchor from the camera, world axes (xyz), and the texel, cm (w). */
        FVector4f Anchor[Levels];
        /** The plane's axes (world, unit). */
        FVector4f E1[Levels];
        FVector4f E2[Levels];
    };

    inline double TexelOf(int32 Level)
    {
        double Texel = Texel0;
        for (int32 L = 0; L < Level; L++) Texel *= Ratio;
        return Texel;
    }

    /** CameraPlanet: the camera in the planet frame (cm); Radius: the sea sphere's; Rotation: the
     *  planet frame to world. */
    inline FLevels MakeLevels(const FVector& CameraPlanet, double Radius, const FQuat& Rotation)
    {
        FLevels Out;
        const FVector Dir = CameraPlanet.GetSafeNormal();
        // The cube face the camera is over: its normal N and its axes A (u) and B (v).
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
        for (int32 Level = 0; Level < Levels; Level++)
        {
            const double Texel = TexelOf(Level);
            const double Step = Texel / FMath::Max(Radius, 1.0);
            const double SU = FMath::FloorToDouble(U / Step + 0.5) * Step;
            const double SV = FMath::FloorToDouble(V / Step + 0.5) * Step;
            const FVector Up = (N + A * SU + B * SV).GetSafeNormal();
            const FVector AnchorPlanet = Up * Radius;
            const FVector E1 = (A - Up * (A | Up)).GetSafeNormal();
            const FVector E2 = Up ^ E1;
            const FVector AnchorWorld = Rotation.RotateVector(AnchorPlanet - CameraPlanet);
            const FVector E1World = Rotation.RotateVector(E1);
            const FVector E2World = Rotation.RotateVector(E2);
            Out.Anchor[Level] = FVector4f((float)AnchorWorld.X, (float)AnchorWorld.Y, (float)AnchorWorld.Z, (float)Texel);
            Out.E1[Level] = FVector4f((float)E1World.X, (float)E1World.Y, (float)E1World.Z, 0.0f);
            Out.E2[Level] = FVector4f((float)E2World.X, (float)E2World.Y, (float)E2World.Z, 0.0f);
        }
        return Out;
    }
}
