// VoxelOceanClipmap.h - The wave clipmap's levels (stages O9k, W1; Design/OceanWaves.md section 28;
// VoxelOceanClip.ush has the layout and the sampling, VoxelOceanClip.usf the per-frame waves,
// VoxelOceanClipStatic.usf the static layer, VoxelOceanPages.cpp the passes). Plain C++ in double,
// shared with the harness.
//
// THE PLANE. Every level lies on one gnomonic plane: tangent to the sea sphere (radius S) at an
// anchor A (planet frame, unit), axes E1 (the anchor's cube face's u axis made tangent) and
// E2 = A x E1. A sphere point P has plane coordinates p = S (P.E1, P.E2) / (P.A) (cm): straight
// lines through the planet's centre, exact both ways. The anchor stays put until the camera's
// radial is ReanchorDegrees from it (then every level is laid out again on the new plane): so the
// levels' grids nest exactly (the rings that will draw them, stage W2, share their vertices) and
// a level's texels stay on the planet as the camera moves.
//
// THE LEVELS. Level k (absolute: k = 0 is ClipFinestSpacing) has texels of Finest 2^k on the plane,
// and a window of Size x Size of them around the camera: texel (I, J) of the plane grid covers
// [I, I + 1) T x [J, J + 1) T, and the window holds I in [Origin.X, Origin.X + Size) (likewise J).
// The origins are even (a level's window starts on a texel of the next level's grid), and move by
// two texels when the camera is more than OriginSlack texels from the window's centre. The levels
// in use run from the base exponent (by the camera's height, as O9k: a 64th of it, with
// hysteresis) up to the first level whose reach covers HorizonCover horizon distances, at most
// the setting's count, and none whose window would pass MaxPlaneDegrees from the anchor.
//
// TOROIDAL STORAGE. A level owns a slot of the clipmap's texture arrays; plane texel (I, J) is
// stored at (I mod Size, J mod Size) of its slot. A window that moves keeps every texel it still
// holds where it is: the static layer writes only the strips the window gains (VoxelOceanClip.usf
// evaluates the waves of a whole level each time it's due).

#pragma once

#include "CoreMinimal.h"

namespace VoxelOceanClipmap
{
    constexpr int32 MaxLevels = 16;            // VOXEL_OCEAN_CLIP_MAX_LEVELS
    /** Level 0's spacing per metre of the camera's height (before the power of two). */
    constexpr double SpacingPerHeight = 1.0 / 64.0;
    /** The plane's anchor is laid again when the camera's radial is this far from it (every
     *  level's static layer is then written again; the camera-relative forms keep the precision,
     *  and the plane's stretch near the camera, 1 / cos^2, stays under 3%). */
    constexpr double ReanchorDegrees = 10.0;
    /** No level's window reaches further than this from the anchor (the gnomonic stretch there:
     *  1 / cos^2, four times at 60 degrees). */
    constexpr double MaxPlaneDegrees = 60.0;
    /** Levels until the coarsest one's reach covers this many horizon distances (and at least
     *  MinCoverCm). */
    constexpr double HorizonCover = 1.5;
    constexpr double MinCoverCm = 100000.0;
    /** A window moves (by two texels) when the camera is further than this from its centre
     *  (texels, either axis). */
    constexpr double OriginSlack = 1.5;
    /** A level not evaluated since its window moved further than this (texels) is due at once
     *  (VOXEL_OCEAN_CLIP_SLACK covers the camera's offset from an evaluated window's centre:
     *  OriginSlack plus this). */
    constexpr int32 EvaluatedSlack = 2;
    /** The info's rows: the plane's, then each level's (VoxelOceanClip.ush). */
    constexpr int32 SharedRows = 4;
    constexpr int32 LevelRows = 2;
    constexpr int32 InfoRows = SharedRows + LevelRows * MaxLevels;
    /** Stage W4c: the foam's fine history (VoxelOceanClip.usf, ClipFineFoamCS): at most this many
     *  levels (VOXEL_OCEAN_CLIP_FINE_MAX), at FineFactor times their resolution. */
    constexpr int32 FineMaxLevels = 4;
    constexpr int32 FineFactor = 4;

    struct FPlane
    {
        FVector A = FVector::ZeroVector;
        FVector E1 = FVector::ZeroVector;
        FVector E2 = FVector::ZeroVector;
        /** Each new anchor's (0: none). */
        uint32 Id = 0;

        bool IsValid() const { return Id != 0; }
    };

    struct FLevel
    {
        int32 K = 0;                           // absolute: spacing Finest 2^K
        double Spacing = 0.0;                  // cm
        FIntPoint Origin = FIntPoint(0, 0);    // the window's first plane texel (even)
        int32 Slot = 0;                        // its texture arrays' slot
    };

    struct FFrame
    {
        FPlane Plane;
        int32 Size = 0;
        int32 Count = 0;
        int32 BaseExponent = -1;
        double Radius = 0.0;                   // the sea sphere (S, cm)
        double Finest = 20.0;                  // cm
        FVector Camera = FVector::ZeroVector;  // planet frame, cm
        FVector2D CameraPlane = FVector2D::ZeroVector;  // its plane coordinates (cm)
        double CameraAlong = 0.0;              // Camera . A (cm)
        double CameraHeight = 0.0;             // |Camera| - S (cm)
        /** Stage W6f: the coarsest level stops short of the cover (the plane's 60 degrees bound
         *  it: from orbit, its square shows on the globe). */
        bool bPlaneLimited = false;
        FLevel Levels[MaxLevels];

        bool IsValid() const { return Plane.IsValid() && Size > 0 && Count > 0; }
        /** The level whose absolute index is K (null: not in use). */
        const FLevel* Find(int32 K) const
        {
            for (int32 i = 0; i < Count; i++)
                if (Levels[i].K == K) return &Levels[i];
            return nullptr;
        }
    };

    /** Level 0's spacing exponent (Base = Finest 2^Exponent) for a camera Height cm over the sea,
     *  kept at Previous unless the height asks for one at least three quarters of a step away. */
    inline int32 BaseExponent(double HeightCm, double FinestCm, int32 Previous)
    {
        const double Want = FMath::Log2(FMath::Max(HeightCm * SpacingPerHeight, 1.0e-3) / FMath::Max(FinestCm, 1.0));
        const double Clamped = FMath::Clamp(Want, 0.0, 30.0);
        if (Previous >= 0 && FMath::Abs(Clamped - (double)Previous) < 0.75) return Previous;
        return FMath::Clamp(FMath::RoundToInt(Clamped), 0, 30);
    }

    /** A plane anchored at the unit radial Dir: E1 the face's u axis made tangent. */
    inline FPlane MakePlane(const FVector& Dir, uint32 Id)
    {
        FPlane P;
        P.A = Dir.GetSafeNormal();
        int32 Axis = 0;
        if (FMath::Abs(P.A.Y) > FMath::Abs(P.A[Axis])) Axis = 1;
        if (FMath::Abs(P.A.Z) > FMath::Abs(P.A[Axis])) Axis = 2;
        FVector U(0, 0, 0);
        U[(Axis + 1) % 3] = 1.0;
        P.E1 = (U - P.A * (U | P.A)).GetSafeNormal();
        P.E2 = P.A ^ P.E1;
        P.Id = FMath::Max(Id, 1u);
        return P;
    }

    /** The plane coordinates (cm) of the sphere point on Dir's radial (Dir . A > 0). */
    inline FVector2D PlaneOf(const FPlane& Plane, double Radius, const FVector& Dir)
    {
        const double Along = FMath::Max(Dir | Plane.A, 1.0e-9);
        return FVector2D(Radius * (Dir | Plane.E1) / Along, Radius * (Dir | Plane.E2) / Along);
    }

    /** The sphere point (planet frame, cm) at plane coordinates Q. */
    inline FVector SphereOf(const FPlane& Plane, double Radius, const FVector2D& Q)
    {
        return (Plane.A + Plane.E1 * (Q.X / Radius) + Plane.E2 * (Q.Y / Radius)).GetSafeNormal() * Radius;
    }

    /** x mod m in [0, m). */
    inline double PositiveMod(double X, double M)
    {
        const double R = FMath::Fmod(X, M);
        return R < 0.0 ? R + M : R;
    }

    inline int32 PositiveMod(int32 X, int32 M)
    {
        const int32 R = X % M;
        return R < 0 ? R + M : R;
    }

    /** The levels around a camera (planet frame) over the sea sphere of Radius (cm): Size texels a
     *  side, at most MaxCount levels (and slots 0 to MaxCount - 1) from level 0's spacing Finest
     *  (cm). Previous: the last frame's (its anchor, windows and slots are kept where they still
     *  serve). */
    inline FFrame MakeFrame(const FVector& CameraPlanet, double Radius, int32 Size, int32 MaxCount, double Finest, const FFrame* Previous)
    {
        FFrame Out;
        Out.Size = FMath::Max(Size, 16) & ~1;
        Out.Radius = Radius;
        Out.Finest = FMath::Max(Finest, 1.0);
        Out.Camera = CameraPlanet;
        const FVector Dir = CameraPlanet.GetSafeNormal();
        MaxCount = FMath::Clamp(MaxCount, 0, MaxLevels);
        if (Dir.IsNearlyZero() || Radius <= 0.0 || MaxCount <= 0) return Out;
        const bool bPrevious = Previous && Previous->Plane.IsValid() && Previous->Size == Out.Size && Previous->Radius == Radius
            && Previous->Finest == Out.Finest;

        // The anchor: kept within ReanchorDegrees of the camera's radial.
        const double CosReanchor = FMath::Cos(FMath::DegreesToRadians(ReanchorDegrees));
        if (bPrevious && (Dir | Previous->Plane.A) >= CosReanchor)
            Out.Plane = Previous->Plane;
        else
            Out.Plane = MakePlane(Dir, (Previous ? Previous->Plane.Id : 0u) + 1u);
        const bool bSamePlane = bPrevious && Out.Plane.Id == Previous->Plane.Id;
        Out.CameraPlane = PlaneOf(Out.Plane, Radius, Dir);
        Out.CameraAlong = CameraPlanet | Out.Plane.A;
        Out.CameraHeight = CameraPlanet.Size() - Radius;

        // The levels in use.
        const double Height = FMath::Max(Out.CameraHeight, 0.0);
        Out.BaseExponent = BaseExponent(Height, Out.Finest, bPrevious ? Previous->BaseExponent : -1);
        const double Horizon = FMath::Sqrt(Height * (2.0 * Radius + Height));
        const double Cover = FMath::Max(HorizonCover * Horizon, MinCoverCm);
        const double PlaneLimit = Radius * FMath::Tan(FMath::DegreesToRadians(MaxPlaneDegrees));
        const double CameraFromAnchor = Out.CameraPlane.Size();
        // Every window stays on the plane's useful part: far out (from orbit) the first level is
        // the coarsest that fits.
        const double Fits = (PlaneLimit - CameraFromAnchor) / (0.5 * Out.Size * UE_SQRT_2);
        if (Fits < Out.Finest) return Out;
        const int32 FitExponent = (int32)FMath::FloorToDouble(FMath::Log2(Fits / Out.Finest));
        const int32 First = FMath::Min(Out.BaseExponent, FitExponent);
        bool bSlotUsed[MaxLevels] = {};
        int32 Count = 0;
        for (int32 i = 0; i < MaxCount; i++)
        {
            const int32 K = First + i;
            const double T = Out.Finest * FMath::Pow(2.0, (double)K);
            if (K > FitExponent) break;
            FLevel& L = Out.Levels[Count];
            L.K = K;
            L.Spacing = T;
            const FVector2D C = Out.CameraPlane / T;
            // The window: kept while the camera is within OriginSlack texels of its centre, else
            // the even origin nearest to centring it.
            const FLevel* Was = bSamePlane ? Previous->Find(K) : nullptr;
            const double Half = 0.5 * Out.Size;
            auto OriginFor = [&](double Cam, int32 Kept, bool bKept) -> int32
            {
                if (bKept && FMath::Abs(Cam - ((double)Kept + Half)) <= OriginSlack) return Kept;
                return 2 * (int32)FMath::FloorToDouble((Cam - Half) * 0.5 + 0.5);
            };
            L.Origin.X = OriginFor(C.X, Was ? Was->Origin.X : 0, Was != nullptr);
            L.Origin.Y = OriginFor(C.Y, Was ? Was->Origin.Y : 0, Was != nullptr);
            L.Slot = -1;
            if (bPrevious)
            {
                // The same level keeps its slot (on a new plane too: only its contents are new).
                for (int32 p = 0; p < Previous->Count; p++)
                {
                    const int32 PreviousSlot = Previous->Levels[p].Slot;
                    if (Previous->Levels[p].K == K && PreviousSlot >= 0 && PreviousSlot < MaxCount && !bSlotUsed[PreviousSlot])
                    {
                        L.Slot = PreviousSlot;
                        break;
                    }
                }
            }
            if (L.Slot >= 0) bSlotUsed[L.Slot] = true;
            Count++;
            if (0.5 * Out.Size * T >= Cover) break;
        }
        Out.Count = Count;
        Out.bPlaneLimited = Count > 0 && 0.5 * Out.Size * Out.Levels[Count - 1].Spacing < Cover && Count < MaxCount;
        // New levels take the free slots.
        for (int32 i = 0; i < Out.Count; i++)
        {
            FLevel& L = Out.Levels[i];
            if (L.Slot >= 0) continue;
            for (int32 s = 0; s < MaxCount; s++)
            {
                if (bSlotUsed[s]) continue;
                L.Slot = s;
                bSlotUsed[s] = true;
                break;
            }
        }
        return Out;
    }

    /** A rectangle of a level's window to write (window texels: I in [I0, I0 + Width), likewise J). */
    struct FRect
    {
        int32 I0 = 0;
        int32 J0 = 0;
        int32 Width = 0;
        int32 Height = 0;

        bool IsEmpty() const { return Width <= 0 || Height <= 0; }
    };

    /** The strips a window gains moving from Was to Now (window texels of the new one; empty when
     *  it didn't move; the whole window when it moved Size or more). At most two rectangles. */
    inline int32 GainedStrips(const FIntPoint& Was, const FIntPoint& Now, int32 Size, FRect Out[2])
    {
        const int32 DX = Now.X - Was.X;
        const int32 DY = Now.Y - Was.Y;
        if (DX == 0 && DY == 0) return 0;
        if (FMath::Abs(DX) >= Size || FMath::Abs(DY) >= Size)
        {
            Out[0] = FRect{ 0, 0, Size, Size };
            return 1;
        }
        int32 N = 0;
        // The columns the window gains (every row), then the rows (the other columns).
        int32 C0 = 0, C1 = Size;
        if (DX > 0)
        {
            Out[N++] = FRect{ Size - DX, 0, DX, Size };
            C1 = Size - DX;
        }
        else if (DX < 0)
        {
            Out[N++] = FRect{ 0, 0, -DX, Size };
            C0 = -DX;
        }
        if (DY > 0)
            Out[N++] = FRect{ C0, Size - DY, C1 - C0, DY };
        else if (DY < 0)
            Out[N++] = FRect{ C0, 0, C1 - C0, -DY };
        return N;
    }

    /** The window texels within Reach cm of a planet-frame point (an edit), widened by Pad texels
     *  and clipped to the window; empty when it's off the window or behind the plane. */
    inline FRect RectAround(const FFrame& Frame, const FLevel& Level, const FVector& Center, double Reach, double Pad)
    {
        const FVector Dir = Center.GetSafeNormal();
        if ((Dir | Frame.Plane.A) <= 0.1) return FRect();
        const FVector2D Q = PlaneOf(Frame.Plane, Frame.Radius, Dir);
        // The plane stretches the sphere by up to 1 / cos^2 of the angle from the anchor.
        const double Along = Dir | Frame.Plane.A;
        const double R = Reach / (Along * Along) / Level.Spacing + Pad;
        const int32 I0 = FMath::Max((int32)FMath::FloorToDouble(Q.X / Level.Spacing - R) - Level.Origin.X, 0);
        const int32 J0 = FMath::Max((int32)FMath::FloorToDouble(Q.Y / Level.Spacing - R) - Level.Origin.Y, 0);
        const int32 I1 = FMath::Min((int32)FMath::CeilToDouble(Q.X / Level.Spacing + R) - Level.Origin.X, Frame.Size);
        const int32 J1 = FMath::Min((int32)FMath::CeilToDouble(Q.Y / Level.Spacing + R) - Level.Origin.Y, Frame.Size);
        if (I1 <= I0 || J1 <= J0) return FRect();
        return FRect{ I0, J0, I1 - I0, J1 - J0 };
    }

    /** The camera's texel coordinate in a window whose origin is Origin (texel units; the window's
     *  texel centres at i + 0.5). */
    inline FVector2D CameraInWindow(const FFrame& Frame, double Spacing, const FIntPoint& Origin)
    {
        return FVector2D(Frame.CameraPlane.X / Spacing - (double)Origin.X, Frame.CameraPlane.Y / Spacing - (double)Origin.Y);
    }

    /** Stage W2 (VoxelOceanRingLayout.ush): a ring's half side (cells) for the clipmap's level
     *  size, the most (even) whose vertices sample every level inside its reach before the fade
     *  (VoxelOceanClip.ush: Size / 2 less the edge and slack, less the fade of Size / 16, in
     *  texels; a cell is two), less one for the square's snap. 0: too small for rings. */
    inline int32 RingHalf(int32 Size)
    {
        const double Reach = (0.5 * Size - 5.5 - Size / 16.0) * 0.5 - 1.0;
        const int32 Half = 2 * (int32)FMath::FloorToDouble(Reach * 0.5);
        return Half >= 6 ? Half : 0;
    }

    /** A level's ring (FVoxelOceanRingLevel): the camera about the square's centre (cells), the
     *  cell (cm), the hole's centre (cells), the centre on the next level's grid. */
    struct FRingRow
    {
        double CamX = 0.0;
        double CamY = 0.0;
        double Spacing = 1.0;
        int32 HoleX = 0;
        int32 HoleY = 0;
        int32 CoarseX = 0;
        int32 CoarseY = 0;
    };

    /** The rings of a frame's first Count levels: level i's cell twice its texel, its square
     *  centred on the even vertex nearest the camera (so it lies on the next level's grid), its
     *  hole where the previous level's square lies, its centre on the next level's grid. */
    inline void MakeRings(const FFrame& Frame, int32 Count, FRingRow Out[MaxLevels])
    {
        int32 Centre[MaxLevels][2] = {};
        Count = FMath::Clamp(Count, 0, FMath::Min(Frame.Count, MaxLevels));
        for (int32 i = 0; i < Count; i++)
        {
            const double G = 2.0 * Frame.Levels[i].Spacing;
            Centre[i][0] = 2 * (int32)FMath::FloorToDouble(Frame.CameraPlane.X / (2.0 * G) + 0.5);
            Centre[i][1] = 2 * (int32)FMath::FloorToDouble(Frame.CameraPlane.Y / (2.0 * G) + 0.5);
        }
        for (int32 i = 0; i < MaxLevels; i++)
        {
            Out[i] = FRingRow();
            if (i >= Count) continue;
            const double G = 2.0 * Frame.Levels[i].Spacing;
            Out[i].CamX = Frame.CameraPlane.X / G - Centre[i][0];
            Out[i].CamY = Frame.CameraPlane.Y / G - Centre[i][1];
            Out[i].Spacing = G;
            Out[i].HoleX = i > 0 ? Centre[i - 1][0] / 2 - Centre[i][0] : 0;
            Out[i].HoleY = i > 0 ? Centre[i - 1][1] / 2 - Centre[i][1] : 0;
            Out[i].CoarseX = i + 1 < Count ? Centre[i][0] / 2 - Centre[i + 1][0] : 0;
            Out[i].CoarseY = i + 1 < Count ? Centre[i][1] / 2 - Centre[i + 1][1] : 0;
        }
    }

    /** The info rows (VoxelOceanClip.ush): the plane's, then for each level in use, (the camera in
     *  the window its waves were evaluated for, the spacing, the slot) and (the camera's texel
     *  coordinate in the toroidal storage, valid, K). Evaluated: each level's window as last
     *  evaluated (Valid false: never: the level is skipped). */
    inline void MakeInfoRows(const FFrame& Frame, const FIntPoint* Evaluated, const bool* Valid, FVector4f Out[InfoRows])
    {
        for (int32 r = 0; r < InfoRows; r++) Out[r] = FVector4f(0.0f, 0.0f, 0.0f, 0.0f);
        const FPlane& P = Frame.Plane;
        Out[0] = FVector4f((float)P.A.X, (float)P.A.Y, (float)P.A.Z, (float)Frame.Radius);
        Out[1] = FVector4f((float)P.E1.X, (float)P.E1.Y, (float)P.E1.Z, (float)Frame.CameraAlong);
        Out[2] = FVector4f((float)P.E2.X, (float)P.E2.Y, (float)P.E2.Z, (float)Frame.CameraHeight);
        Out[3] = FVector4f((float)(Frame.CameraPlane.X / Frame.Radius), (float)(Frame.CameraPlane.Y / Frame.Radius), (float)Frame.Count, (float)Frame.Size);
        for (int32 i = 0; i < Frame.Count; i++)
        {
            const FLevel& L = Frame.Levels[i];
            const FVector2D InWindow = CameraInWindow(Frame, L.Spacing, Evaluated[i]);
            const double ModX = PositiveMod(Frame.CameraPlane.X / L.Spacing, (double)Frame.Size);
            const double ModY = PositiveMod(Frame.CameraPlane.Y / L.Spacing, (double)Frame.Size);
            Out[SharedRows + LevelRows * i] = FVector4f((float)InWindow.X, (float)InWindow.Y, (float)L.Spacing, (float)L.Slot);
            Out[SharedRows + LevelRows * i + 1] = FVector4f((float)ModX, (float)ModY, Valid[i] ? 1.0f : 0.0f, (float)L.K);
        }
    }
}
