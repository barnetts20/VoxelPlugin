// VoxelCubeSphere.h - The equal-angle cube-sphere mapping shared by the ocean's quadtree
// (VoxelOceanTiles.h) and the land bake (VoxelLandBake.h; VoxelCubeSphere.ush is the same for
// shader code). Pure functions, in double; any thread.
//
// A face has an outward normal N and axes U, V with U x V = N. Face coordinates s (along U) and
// t (along V) run over [-1, 1] and map to normalize(N + tan(pi s / 4) U + tan(pi t / 4) V): the
// equal-angle mapping, whose cells vary 1.41x in edge length across a face (2.1x for the plain
// cube). Beyond [-1, 1] the same formula continues the face past its edges.

#pragma once

#include "CoreMinimal.h"

namespace VoxelCubeSphere
{
    struct FFace
    {
        FVector N;
        FVector U;
        FVector V;
    };

    /** Face f: normal along axis f / 2, negative for odd f (VoxelCubeSphere.ush lists them). */
    inline const FFace& GetFace(int32 Face)
    {
        static const FFace Faces[6] = {
            { FVector(1.0, 0.0, 0.0), FVector(0.0, 1.0, 0.0), FVector(0.0, 0.0, 1.0) },
            { FVector(-1.0, 0.0, 0.0), FVector(0.0, 0.0, 1.0), FVector(0.0, 1.0, 0.0) },
            { FVector(0.0, 1.0, 0.0), FVector(0.0, 0.0, 1.0), FVector(1.0, 0.0, 0.0) },
            { FVector(0.0, -1.0, 0.0), FVector(1.0, 0.0, 0.0), FVector(0.0, 0.0, 1.0) },
            { FVector(0.0, 0.0, 1.0), FVector(1.0, 0.0, 0.0), FVector(0.0, 1.0, 0.0) },
            { FVector(0.0, 0.0, -1.0), FVector(0.0, 1.0, 0.0), FVector(1.0, 0.0, 0.0) },
        };
        return Faces[Face];
    }

    /** Equal-angle warp of a face coordinate onto the cube face, and its inverse. */
    inline double Warp(double S) { return FMath::Tan(S * (UE_DOUBLE_PI * 0.25)); }
    inline double Unwarp(double X) { return FMath::Atan(X) * (4.0 / UE_DOUBLE_PI); }

    /** Face coordinate of grid line K of a grid with 2^Bits cells per face edge (exact). */
    inline double GridCoord(int64 K, int32 Bits)
    {
        return -1.0 + 2.0 * (double)K / (double)(1LL << Bits);
    }

    /** Face coordinate of the centre of cell I of a face with N cells per edge (I may be
     *  negative or N and up: the apron past the edge). */
    inline double CellCentre(int32 I, int32 N)
    {
        return -1.0 + (2.0 * (double)I + 1.0) / (double)N;
    }

    /** Unit direction of face coordinates (s, t). */
    inline FVector Direction(int32 Face, double S, double T)
    {
        const FFace& F = GetFace(Face);
        return (F.N + F.U * Warp(S) + F.V * Warp(T)).GetSafeNormal();
    }

    /** Face and face coordinates of a direction (the inverse of Direction). */
    inline void Locate(const FVector& Dir, int32& OutFace, double& OutS, double& OutT)
    {
        int32 Best = 0;
        double BestDot = -2.0;
        for (int32 Face = 0; Face < 6; Face++)
        {
            const double Dot = GetFace(Face).N | Dir;
            if (Dot > BestDot)
            {
                BestDot = Dot;
                Best = Face;
            }
        }
        const FFace& F = GetFace(Best);
        OutFace = Best;
        OutS = Unwarp((F.U | Dir) / BestDot);
        OutT = Unwarp((F.V | Dir) / BestDot);
    }
}
