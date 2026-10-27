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
//
// THE CAUSTICS' GRIDS (stage A8; their own since T5d3, the user's "the finest clip is off centre"):
// CausticLevels nested grids of the same Size, from CausticTexel0 by Ratio, laid the same way but
// round a centre of their own: the sea's point the sun lights the bed under the camera through
// (the camera's foot moved up-sun by the bed's depth times the refracted sun's tangent). A bed
// point's caustics come from the surface up-sun of it, so grids round the camera's foot would sharpen
// the bed beside the camera, not under it. Their slopes are the surface map's heights wherever they
// lie (VoxelUnderwaterPass.usf, the splat). VOXEL_UNDERWATER_CAUSTIC_LEVELS goes to the shaders.

#pragma once

#include "CoreMinimal.h"

namespace VoxelUnderwaterMap
{
    /** The levels, their grids' size, the finest texel (cm) and each next level's factor. */
    constexpr int32 Levels = 5;
    constexpr int32 Size = 256;
    constexpr double Texel0 = 4.0;   // T5d3: was 2 cm (the finest level 5 m across: too small to the eye)
    constexpr double Ratio = 4.0;
    /** The caustics' grids: their count and finest texel (cm; 4 cm: 10 m, 41 m, 164 m, 655 m across). */
    constexpr int32 CausticLevels = 4;
    constexpr double CausticTexel0 = 4.0;

    struct FLevels
    {
        /** Per level: the anchor from the camera, world axes (xyz), and the texel, cm (w). */
        FVector4f Anchor[Levels];
        /** The plane's axes (world, unit). */
        FVector4f E1[Levels];
        FVector4f E2[Levels];
    };

    struct FCausticLevels
    {
        FVector4f Anchor[CausticLevels];
        FVector4f E1[CausticLevels];
        FVector4f E2[CausticLevels];
    };

    inline double TexelOf(int32 Level)
    {
        double Texel = Texel0;
        for (int32 L = 0; L < Level; L++) Texel *= Ratio;
        return Texel;
    }

    /** Count grids from FirstTexel (cm) by Ratio round CentrePlanet's radial (each anchor snapped on
     *  its cube face), from the camera (CameraPlanet; planet frame, cm), turned to world axes by
     *  Rotation: into OutAnchor (xyz, w the texel), OutE1, OutE2. */
    inline void MakeGrids(const FVector& CameraPlanet, const FVector& CentrePlanet, double Radius, const FQuat& Rotation, double FirstTexel, int32 Count,
        FVector4f* OutAnchor, FVector4f* OutE1, FVector4f* OutE2)
    {
        const FVector Dir = CentrePlanet.GetSafeNormal();
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
        double Texel = FirstTexel;
        for (int32 Level = 0; Level < Count; Level++, Texel *= Ratio)
        {
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
            OutAnchor[Level] = FVector4f((float)AnchorWorld.X, (float)AnchorWorld.Y, (float)AnchorWorld.Z, (float)Texel);
            OutE1[Level] = FVector4f((float)E1World.X, (float)E1World.Y, (float)E1World.Z, 0.0f);
            OutE2[Level] = FVector4f((float)E2World.X, (float)E2World.Y, (float)E2World.Z, 0.0f);
        }
    }

    /** CameraPlanet: the camera in the planet frame (cm); Radius: the sea sphere's; Rotation: the
     *  planet frame to world. */
    inline FLevels MakeLevels(const FVector& CameraPlanet, double Radius, const FQuat& Rotation)
    {
        FLevels Out;
        MakeGrids(CameraPlanet, CameraPlanet, Radius, Rotation, Texel0, Levels, Out.Anchor, Out.E1, Out.E2);
        return Out;
    }

    /** The point on the sea sphere (planet frame, cm) the sun lights the bed under the camera
     *  through: the camera's foot moved toward the sun by BedDepth (cm under the sea, clamped to
     *  0 .. MaxDepth) times the refracted sun's tangent (Snell, water 1.333; at most about 1.13).
     *  SunPlanet: toward the sun (planet frame, unit); the foot itself with the sun down. */
    inline FVector CausticCentre(const FVector& CameraPlanet, double Radius, const FVector& SunPlanet, double BedDepth, double MaxDepth)
    {
        const FVector Up = CameraPlanet.GetSafeNormal();
        const FVector Foot = Up * Radius;
        const double MuS = SunPlanet | Up;
        if (!(MuS > 0.0) || !(BedDepth > 0.0)) return Foot;
        // The refracted beam's sine and cosine from the vertical.
        const double SinA = FMath::Sqrt(FMath::Max(1.0 - MuS * MuS, 0.0));
        const double SinW = SinA / 1.333;
        const double CosW = FMath::Sqrt(FMath::Max(1.0 - SinW * SinW, 1.0e-6));
        const FVector Toward = (SunPlanet - Up * MuS).GetSafeNormal();
        const FVector Shifted = Foot + Toward * (FMath::Min(BedDepth, MaxDepth) * SinW / CosW);
        return Shifted.GetSafeNormal() * Radius;
    }

    /** The caustics' grids round Centre (CausticCentre), from the camera. */
    inline FCausticLevels MakeCausticLevels(const FVector& CameraPlanet, const FVector& Centre, double Radius, const FQuat& Rotation)
    {
        FCausticLevels Out;
        MakeGrids(CameraPlanet, Centre, Radius, Rotation, CausticTexel0, CausticLevels, Out.Anchor, Out.E1, Out.E2);
        return Out;
    }
}
