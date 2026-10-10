// VoxelOceanSettings.h - The ocean's settings as one struct, held by the ocean actor and by the
// planet that owns it (stage O5): a planet forwards its copy whole, so a respawned ocean comes
// back with the same settings and nothing is set in two places. The ocean applies only what
// changed (AOceanSphereActor::SetOceanSettings): the mesh rebuilds, the optics apply to the
// surface material, everything else is read every tick.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanSettings.generated.h"

USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelOceanSettings
{
    GENERATED_BODY()

    // --- Mesh (a change rebuilds the surface) ---

    /** Spacing between vertices at the finest level near the camera (cm). The finest level is
     *  the one at or just under it (the ocean's ActualSpacing). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "1.0", UIMin = "10.0", UIMax = "1000.0"))
    double FinestSpacing = 50.0;

    /** Quads along a tile edge: 8, 16 or 32 (snapped to one of them). The number of tiles
     *  doesn't depend on it, so the triangle count grows with its square. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "8", ClampMax = "32"))
    int32 TileQuads = 16;

    /** A tile splits while the camera is within this many tile sizes of it. Higher: more
     *  detail and a wider morph band. At least 2.25 (the morph band closes below it). Applies
     *  without a rebuild. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "2.25", UIMax = "6.0"))
    double LodDistanceFactor = 2.5;

    /** Buffer sets kept ready, so a commit doesn't wait for one to be created. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (ClampMin = "0", UIMax = "32"))
    int32 SpareBufferSets = 8;

    // --- Depth ---

    /** Leave out the triangles the land stands well above (the terrain hides them anyway). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Depth")
    bool bCullUnderLand = true;

    /** How far (cm) the land must stand above the sea at a triangle's corners, beyond what the
     *  terrain's slope bound and the waves' reach allow for, before it is left out. Covers the
     *  terrain mesh's error. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Depth", meta = (ClampMin = "0.0", UIMax = "5000.0", EditCondition = "bCullUnderLand"))
    double LandCullMargin = 500.0;

    // --- Underwater ---

    /** The view from under water: a pass of the ocean's own after the atmosphere, ahead of
     *  depth of field. It decides each pixel itself and leaves a view whose camera can't reach
     *  the water untouched. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Underwater")
    bool bUnderwaterView = true;

    // --- Optics ---

    /** Absorption of the water per metre, per channel (red goes first). Feeds the underwater
     *  pass and the surface material (VoxelOceanAbsorption). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor Absorption = FLinearColor(0.35f, 0.07f, 0.03f);

    /** Scattering of the water per metre, per channel. Feeds the underwater pass and the
     *  surface material (VoxelOceanScattering). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor Scattering = FLinearColor(0.02f, 0.03f, 0.035f);

    /** Forward scattering anisotropy, for a Single Layer Water surface (VoxelOceanPhaseG). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics", meta = (ClampMin = "-1.0", ClampMax = "1.0"))
    float PhaseG = 0.8f;

    /** The light the water scatters toward the camera under water, in the scene's light units
     *  (linear, before exposure, so it holds as the exposure adapts), until the atmosphere
     *  provides the sun's. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor UnderwaterLight = FLinearColor(0.4f, 0.8f, 1.0f);

    /** The thin band (cm) where the near plane meets the surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics", meta = (ClampMin = "0.0", UIMax = "10.0"))
    double MeniscusWidth = 1.5;

    /** The waterline's tint, in the scene's light units (as UnderwaterLight). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor MeniscusColor = FLinearColor(0.02f, 0.06f, 0.08f);

    // --- Environment (constants until the atmosphere provides fields) ---

    /** Wind speed (m/s). The waves receive it per point; the test model ignores it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", UIMax = "40.0"))
    double WindSpeed = 8.0;

    /** Wind heading (degrees from local north toward east; north is the planet's +Z). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (UIMin = "0.0", UIMax = "360.0"))
    double WindHeading = 0.0;

    /** Storm intensity, 0-1. The waves receive it per point; the test model ignores it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double StormIntensity = 0.0;

    /** The ocean clock's rate against the game's (server) time. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double OceanTimeScale = 1.0;

    /** TileQuads snapped to 8, 16 or 32 and the other fields to their limits. */
    void Sanitize()
    {
        TileQuads = 1 << FMath::Clamp(FMath::RoundToInt(FMath::Log2((double)FMath::Max(TileQuads, 1))), 3, 5);
        FinestSpacing = FMath::Max(FinestSpacing, 1.0);
        LodDistanceFactor = FMath::Max(LodDistanceFactor, 2.25);
        SpareBufferSets = FMath::Max(SpareBufferSets, 0);
        LandCullMargin = FMath::Max(LandCullMargin, 0.0);
        MeniscusWidth = FMath::Max(MeniscusWidth, 0.0);
        WindSpeed = FMath::Max(WindSpeed, 0.0);
        StormIntensity = FMath::Clamp(StormIntensity, 0.0, 1.0);
        OceanTimeScale = FMath::Max(OceanTimeScale, 0.0);
    }

    /** The fields whose change rebuilds the surface. */
    bool SameMesh(const FVoxelOceanSettings& Other) const
    {
        return FinestSpacing == Other.FinestSpacing && TileQuads == Other.TileQuads && SpareBufferSets == Other.SpareBufferSets;
    }

    /** The fields the surface material receives. */
    bool SameOptics(const FVoxelOceanSettings& Other) const
    {
        return Absorption == Other.Absorption && Scattering == Other.Scattering && PhaseG == Other.PhaseG;
    }
};
