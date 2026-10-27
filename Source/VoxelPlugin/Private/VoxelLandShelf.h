// VoxelLandShelf.h - The polar ice shelf (stage T5e, Design/LandClimate.md): where the climate
// snapshot's year's mean at sea level is cold enough, the polar sea (and any low land there)
// becomes a shelf of ice in the base elevation: a solid plateau, like a plate placed by the cold
// rather than by the plate model. Placed once per capture (the terrain is built after it, and the
// seasons don't move it).
//
// THE FRONT: the line where the year's mean at sea level is IceShelfTemperature, wandering by up
// to IceShelfFrontNoise C (a gradient-noise fBm on the sphere: capes and bays from thousands of
// kilometres down to a few bake cells), so the shelf's edge isn't an isotherm. Phi (C) is how far
// inside the front a point is: the front's temperature less the point's, plus the noise.
//
// THE SHELF (Apply, per bake cell):
//   - the front: the plateau comes up over +-FrontWidth C of Phi (a bake cell or two);
//   - its top: IceShelfHeight at the front, rising IceShelfRise over RiseSpan C inland, rolling
//     gently (IceShelfRoll: an fBm tens to a hundred kilometres across; the chain's snow dunes and
//     sastrugi ride on it);
//   - anything higher than the top stands through it (mountains, islands) and keeps its own form;
//     lower ground is raised to it;
//   - in front of it, the sea floor rises to IceShelfDraft below the sea over SkirtSpan C, so the
//     front stands that far into the water, not over the abyss.
// The terrain is a heightfield: the shelf is solid, with no water under it.
//
// THE MATERIAL AND THE CHAIN: the climate map stores the shelf's share at its cells (MapShare, from
// Phi without the octaves finer than the map: about 0.08 C of the front's 3) as minus its w, its
// continentality's slot (VoxelLandClimateGrid.h BuildCells), so the climate node whitens it, the
// polar layer puts the snow dunes and sastrugi on it and the rivers give way on it, all without
// the bake.
//
// Plain C++ (no engine types): the climate harness (Tools/LandClimateHarness) runs it.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

/** The shelf's settings (FVoxelTerrestrialSettings' IceShelf*) and the planet's seed. */
struct FVoxelLandShelfParams
{
    bool bOn = false;
    double Temperature = -15.0;   // C: the year's mean at sea level at the front
    double FrontNoise = 5.0;      // C: how far the front wanders (at most)
    double Height = 40.0;         // m: the front's height above the sea
    double Rise = 40.0;           // m: the height gained inland, over RiseSpan
    double Roll = 10.0;           // m: the surface's roll
    double Draft = 300.0;         // m: the sea floor's depth in front of it (0: as it is)
    uint32_t Seed = 0;
};

namespace VoxelLandShelf
{
    /** C inside the front over which the top gains IceShelfRise. */
    constexpr double RiseSpan = 3.0;
    /** C of Phi either side of the front over which the plateau comes up. */
    constexpr double FrontWidth = 0.15;
    /** C outside the front over which the sea floor rises to the draft. */
    constexpr double SkirtSpan = 1.5;
    /** C outside the front over which the climate map's share fades in. */
    constexpr double MapBand = 1.5;
    /** C inside the front over which the roll comes in (the front's edge stays even). */
    constexpr double RollSpan = 1.0;
    /** m: the smooth maximum's width where the sea floor meets the draft. */
    constexpr double DraftBlend = 100.0;
    /** The front's fBm: its lowest frequency (per unit of the sphere's radius) and octaves. */
    constexpr double FrontFrequency = 2.0;
    constexpr int FrontOctaves = 8;
    /** Its octaves' amplitude ratio (above a half: the capes and bays of a few hundred kilometres
     *  carry weight, as a coastline's do). */
    constexpr double FrontGain = 0.62;
    /** The roll's fBm. */
    constexpr double RollFrequency = 16.0;
    constexpr int RollOctaves = 4;
    /** The highest frequency the climate map resolves (two of its cells). */
    constexpr double MapMaxFrequency = 40.0;
    /** The front's fBm scaled so IceShelfFrontNoise is about its extremes (the normalised fBm
     *  rarely passes +-0.55; its typical spread is a third of that). */
    constexpr double FrontScale = 2.0;
    /** The scaled noise's bound (the gradient noise's range, a little over 1, times FrontScale). */
    constexpr double NoiseBound = 1.05 * FrontScale;

    /** Smooth ramp, 0 at E0 to 1 at E1 (either order). */
    inline double Ramp(double E0, double E1, double X)
    {
        const double T = std::min(std::max((X - E0) / (E1 - E0), 0.0), 1.0);
        return T * T * (3.0 - 2.0 * T);
    }

    /** A lattice point's hash. */
    inline uint32_t Hash(int32_t X, int32_t Y, int32_t Z, uint32_t Seed)
    {
        uint32_t H = Seed * 0x9E3779B9u;
        H ^= (uint32_t)X * 0x85EBCA6Bu;
        H = (H << 13) | (H >> 19);
        H ^= (uint32_t)Y * 0xC2B2AE35u;
        H = (H << 17) | (H >> 15);
        H ^= (uint32_t)Z * 0x27D4EB2Fu;
        H ^= H >> 16;
        H *= 0x7FEB352Du;
        H ^= H >> 15;
        H *= 0x846CA68Bu;
        H ^= H >> 16;
        return H;
    }

    /** One of Perlin's twelve gradients (the cube's edge midpoints) dotted with the offset. */
    inline double Grad(uint32_t H, double X, double Y, double Z)
    {
        switch (H % 12u)
        {
        case 0: return X + Y;
        case 1: return -X + Y;
        case 2: return X - Y;
        case 3: return -X - Y;
        case 4: return X + Z;
        case 5: return -X + Z;
        case 6: return X - Z;
        case 7: return -X - Z;
        case 8: return Y + Z;
        case 9: return -Y + Z;
        case 10: return Y - Z;
        default: return -Y - Z;
        }
    }

    inline double Fade(double T)
    {
        return T * T * T * (T * (T * 6.0 - 15.0) + 10.0);
    }

    /** Perlin's improved gradient noise (2002), seeded: about -1 .. 1, smooth. */
    inline double Noise(double X, double Y, double Z, uint32_t Seed)
    {
        const double FX = std::floor(X);
        const double FY = std::floor(Y);
        const double FZ = std::floor(Z);
        const int32_t IX = (int32_t)FX;
        const int32_t IY = (int32_t)FY;
        const int32_t IZ = (int32_t)FZ;
        const double X0 = X - FX;
        const double Y0 = Y - FY;
        const double Z0 = Z - FZ;
        const double U = Fade(X0);
        const double V = Fade(Y0);
        const double W = Fade(Z0);
        auto G = [&](int32_t DX, int32_t DY, int32_t DZ)
        {
            return Grad(Hash(IX + DX, IY + DY, IZ + DZ, Seed), X0 - (double)DX, Y0 - (double)DY, Z0 - (double)DZ);
        };
        const double X00 = G(0, 0, 0) + U * (G(1, 0, 0) - G(0, 0, 0));
        const double X10 = G(0, 1, 0) + U * (G(1, 1, 0) - G(0, 1, 0));
        const double X01 = G(0, 0, 1) + U * (G(1, 0, 1) - G(0, 0, 1));
        const double X11 = G(0, 1, 1) + U * (G(1, 1, 1) - G(0, 1, 1));
        const double Y0V = X00 + V * (X10 - X00);
        const double Y1V = X01 + V * (X11 - X01);
        return Y0V + W * (Y1V - Y0V);
    }

    /** An fBm on the unit sphere's direction (X, Y, Z): Octaves from frequency F0, each twice the
     *  last at half the amplitude, normalised by all the octaves' amplitudes (so the octaves above
     *  MaxFrequency are left out without rescaling the rest: a coarser grid sees the same shape,
     *  less its fine detail). */
    inline double Fbm(double X, double Y, double Z, uint32_t Seed, double F0, int Octaves, double MaxFrequency, double Gain = 0.5)
    {
        double Sum = 0.0;
        double Norm = 0.0;
        double A = 1.0;
        double F = F0;
        for (int I = 0; I < Octaves; I++)
        {
            Norm += A;
            if (F <= MaxFrequency)
            {
                // Each octave its own lattice offset and seed (no shared lattice points).
                const double O = 17.31 * (double)(I + 1);
                Sum += A * Noise(X * F + O, Y * F - 0.5 * O, Z * F + 0.25 * O, Seed + 0x632BE5ABu * (uint32_t)(I + 1));
            }
            A *= Gain;
            F *= 2.0;
        }
        return Norm > 0.0 ? Sum / Norm : 0.0;
    }

    /** The front's fBm (unscaled). */
    inline double FrontFbm(double X, double Y, double Z, uint32_t Seed, double MaxFrequency)
    {
        return Fbm(X, Y, Z, Seed, FrontFrequency, FrontOctaves, MaxFrequency, FrontGain);
    }

    /** Whether a year's mean at sea level T (C) is warm enough that nothing can change. */
    inline bool IsOutside(double T, const FVoxelLandShelfParams& P)
    {
        return !P.bOn || P.Height <= 0.0 || T > P.Temperature + NoiseBound * std::max(P.FrontNoise, 0.0) + SkirtSpan;
    }

    /** Phi (C): how far inside the front the direction (unit) is, at a year's mean at sea level T,
     *  with the front's octaves up to MaxFrequency. */
    inline double PhiAt(double X, double Y, double Z, double T, const FVoxelLandShelfParams& P, double MaxFrequency)
    {
        return P.Temperature - T + std::max(P.FrontNoise, 0.0) * FrontScale * FrontFbm(X, Y, Z, P.Seed, MaxFrequency);
    }

    /** The climate map's share at Phi (its octaves the map's: MapMaxFrequency): 1 from the front
     *  in, fading over MapBand outside it. */
    inline double MapShare(double Phi)
    {
        return Ramp(-MapBand, 0.0, Phi);
    }

    /** A bake cell under the shelf. */
    struct FCell
    {
        double Height;      // m: the cell's height with the shelf
        double Raised;      // 0..1: the shelf's share of it (its own form, hills and water give way)
        double Influence;   // 0..1: that, or the skirt's on the sea floor (the peaks' relief gives way)
        double Front;       // 0..1: the plateau's share (0 outside its front)
        double Top;         // m: the plateau's top there (0 outside the skirt)
    };

    /** The shelf at a bake cell: its direction (unit), the year's mean at sea level there T (C),
     *  its height H (m), the octaves up to MaxFrequency (the bake's: a few cells). */
    inline FCell Apply(double X, double Y, double Z, double T, double H, const FVoxelLandShelfParams& P, double MaxFrequency)
    {
        FCell Out;
        Out.Height = H;
        Out.Raised = 0.0;
        Out.Influence = 0.0;
        Out.Front = 0.0;
        Out.Top = 0.0;
        if (IsOutside(T, P)) return Out;
        const double Phi = PhiAt(X, Y, Z, T, P, MaxFrequency);
        // The sea floor in front of it and under it, up to the draft (a smooth maximum: a polynomial
        // one, as the chain's).
        const double Skirt = Ramp(-SkirtSpan, 0.0, Phi);
        double Base = H;
        if (P.Draft > 0.0 && Skirt > 0.0)
        {
            const double B = -P.Draft;
            const double K = DraftBlend;
            const double Hm = std::max(K - std::fabs(H - B), 0.0) / K;
            const double Smooth = std::max(H, B) + K * Hm * Hm * 0.25;
            Base = H + Skirt * (Smooth - H);
        }
        // The plateau's top: the front's height, the rise inland, the roll.
        const double Front = Ramp(-FrontWidth, FrontWidth, Phi);
        const double Roll = P.Roll > 0.0
            ? P.Roll * 2.0 * Fbm(X, Y, Z, P.Seed ^ 0x5EA1CE5u, RollFrequency, RollOctaves, MaxFrequency) * Ramp(0.0, RollSpan, Phi)
            : 0.0;
        const double Top = std::max(P.Height + std::max(P.Rise, 0.0) * Ramp(0.0, RiseSpan, Phi) + Roll, 0.5 * P.Height);
        // Ground higher than the top stands through it; lower ground is raised to it.
        Out.Height = H >= Top ? H : Base + Front * (Top - Base);
        Out.Raised = Front * (1.0 - Ramp(Top - 10.0, Top + 10.0, H));
        Out.Influence = std::max(Out.Raised, Skirt * (1.0 - Ramp(-10.0, 10.0, H)));
        Out.Front = Front;
        Out.Top = Top;
        return Out;
    }
}
