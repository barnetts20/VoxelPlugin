// VoxelOceanSettings.h - The ocean's settings as one struct, held by the ocean actor and by the
// planet that owns it (stage O5): a planet forwards its copy whole, so a respawned ocean comes
// back with the same settings and nothing is set in two places. The ocean applies only what
// changed (AOceanSphereActor::SetOceanSettings): the mesh rebuilds, the optics apply to the
// surface material, everything else is read every tick.

#pragma once

#include "CoreMinimal.h"
#include "VoxelOceanSettings.generated.h"

/** The wave equalizer (stage W3k; Design/OceanWaves.md section 35): a gain on the waves' height
 *  per octave of wavelength, on top of the spectrum the wind makes (1: as it is; 0: none of
 *  those waves; 2: twice as tall). Between two octaves the gain blends smoothly. The foam, the
 *  whitecaps and the shading measure the sea after it, so they follow. A change rebuilds the
 *  waves. Stage W4a: 2.83 everywhere by default, GodotOceanWaves' height: its spectrum's modes
 *  carry 8 times the variance of the spectrum's own (its h0 is gaussian sqrt(2 S d w_norm) with
 *  unit components, ours the physical S dk^2 per wave), so its waves stand sqrt(8) = 2.83 times
 *  taller at the same wind. 1 everywhere: the physical spectrum. */
USTRUCT(BlueprintType)
struct VOXELPLUGIN_API FVoxelOceanWaveEqualizer
{
    GENERATED_BODY()

    static constexpr int32 Knots = 13;

    /** Capillary-gravity ripples, and the sub-mesh roughness the shading adds (its slope
     *  variance follows this gain squared). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "25 cm", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain25cm = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "50 cm", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain50cm = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "1 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain1m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "2 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain2m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "4 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain4m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "8 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain8m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "16 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain16m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "32 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain32m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "64 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain64m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "128 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain128m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "256 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain256m = 2.83f;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "512 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain512m = 2.83f;
    /** The longest swell the waves hold (the 1 km band's longest are 997 m). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Equalizer", meta = (DisplayName = "1024 m", ClampMin = "0.0", ClampMax = "8.0", UIMax = "3.0"))
    float Gain1024m = 2.83f;

    /** Knot i's gain, shortest first (25 cm at 0, 1,024 m at 12). */
    float Get(int32 Knot) const
    {
        const float G[Knots] = { Gain25cm, Gain50cm, Gain1m, Gain2m, Gain4m, Gain8m, Gain16m, Gain32m, Gain64m, Gain128m, Gain256m, Gain512m, Gain1024m };
        return G[FMath::Clamp(Knot, 0, Knots - 1)];
    }

    void Sanitize()
    {
        float* G[Knots] = { &Gain25cm, &Gain50cm, &Gain1m, &Gain2m, &Gain4m, &Gain8m, &Gain16m, &Gain32m, &Gain64m, &Gain128m, &Gain256m, &Gain512m, &Gain1024m };
        for (float* V : G) *V = FMath::Clamp(*V, 0.0f, 8.0f);
    }
};

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

    /** Evaluate the waves once a frame per vertex in compute (stage O9j) instead of in the
     *  material's vertex shader in every pass that draws the ocean (depth, velocity, base). The
     *  material needs nothing new; disconnecting its World Position Offset also takes the ocean
     *  out of the velocity pass. Off: the old path, for comparison. Rebuilds the surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh")
    bool bWavesInCompute = true;

    /** The wave clipmap (stage O9k; with the waves in compute): the waves evaluated once a frame
     *  on camera-centred levels of texels, which the vertices and the material's pixel node
     *  (VoxelOceanPixelClip) only sample. Its cost is the levels' texels, not the mesh's vertices
     *  or the screen's pixels. Rebuilds the surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh")
    bool bWaveClipmap = true;

    /** Texels along a clipmap level's side (a power of two, 32 to 512). The cost grows with its
     *  square. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (EditCondition = "bWaveClipmap", ClampMin = "32", ClampMax = "512"))
    int32 ClipSize = 128;

    /** Clipmap levels at most, each twice as coarse and wide as the one before (at most 16; stage
     *  W1: only as many as reach one and a half horizon distances are in use, and those whose
     *  window stays within 60 degrees of the clipmap's anchor). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (EditCondition = "bWaveClipmap", ClampMin = "1", ClampMax = "16"))
    int32 ClipLevels = 12;

    /** The finest clipmap level's texel (cm), near the sea; coarser with the camera's height
     *  (about a 64th of it). Waves shorter than about four of these near the camera shade as
     *  roughness only. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (EditCondition = "bWaveClipmap", ClampMin = "1.0", UIMax = "200.0"))
    double ClipFinestSpacing = 20.0;

    /** Stage W2: the surface drawn as rings on the clipmap's levels, with a far sphere for the
     *  rest, instead of the tile pages (whenever the clipmap runs: the FFT waves). No tile
     *  selection, no page writes. Off: the tile pages as before. Rebuilds the surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (EditCondition = "bWaveClipmap"))
    bool bWaveRings = true;

    /** The far sphere's cells along each cube face's edge (under and past the rings). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Mesh", meta = (EditCondition = "bWaveClipmap && bWaveRings", ClampMin = "4", ClampMax = "256"))
    int32 FarSphereCells = 64;

    // --- Shading (stage W3) ---

    /** The ocean's own pass draws and shades the rings (an atmosphere-lit sea, the same model from
     *  the shore to orbit) instead of the ocean material. Only with the rings (bWaveRings, the FFT
     *  waves in the clipmap); the tile pages always use the material. Off: the rings draw with the
     *  material as in stage W2. Rebuilds the surface. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bWaveClipmap && bWaveRings"))
    bool bOwnShading = true;

    /** The slope variance (both axes together) of the short waves no wave band holds: the
     *  capillaries and the shortest gravity waves. Widens the sun's glitter and blurs the sky's
     *  reflection everywhere, on top of what the bands leave unresolved at a distance. Stage W3z:
     *  0.001 (was 0.004): the shortest band now holds waves down to 10.7 cm (from 21 cm), about
     *  0.0033 of slope variance at every wind. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", UIMax = "0.05"))
    float ShortWaveSlopeVariance = 0.001f;

    /** How far the bed seen through the water shifts with the waves' refraction (1: as the
     *  refracted ray meets it; 0: no shift). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", UIMax = "2.0"))
    float Refraction = 1.0f;

    /** The foam's albedo, lit by the sun and the sky. Stage W4d: GodotOceanWaves' foam colour, sRGB
     *  (0.73, 0.67, 0.62) in linear (was a neutral 0.85). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading"))
    FLinearColor FoamAlbedo = FLinearColor(0.492f, 0.407f, 0.342f);

    /** How opaque the foam is overall (stage W3s; 1: solid where it is thick, lower to let the
     *  water show through all of it). Stage W4a: 1, GodotOceanWaves' (its foam's density alone). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", ClampMax = "1.0"))
    float FoamOpacity = 1.0f;

    /** The foam's look (stage W3v): 1, GodotOceanWaves' (its smooth density mixed over the water,
     *  lit flat with a soft sheen); 0, through the foam texture's lace (crisp patches lit through
     *  their relief, as before W3t); or between. The amount of foam is the same either way. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", ClampMax = "1.0"))
    float FoamSoftness = 1.0f;

    /** The foam's detail near the camera (stage W3u, as GodotOceanWaves draws its foam): 1, the
     *  waves' own foam and whitecaps looked up at every pixel (from the wave cascades, a few
     *  centimetres a texel), where the wave clipmap's texels (20 cm to over a metre) would span
     *  several pixels and show as blocks; 0, the clipmap's (as before W3u). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", ClampMax = "1.0"))
    float FoamDetail = 1.0f;

    /** How far from the camera the foam's per-pixel detail reaches (m; stage W3u). Past it (fading
     *  over its last 30%), the clipmap's foam, which resolves the pixels there anyway. It costs a
     *  second look at the wave cascades per pixel within it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", AdvancedDisplay, meta = (EditCondition = "bOwnShading", ClampMin = "10.0", UIMax = "1000.0"))
    float FoamDetailDistance = 500.0f;

    /** Cubic filtering of the waves' textures where the pixels are finer than their texels (stage
     *  W3u, as GodotOceanWaves' bicubic lookups): smooth slopes, whitecaps and foam close up instead
     *  of facets and blocks. Four lookups instead of one there. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", AdvancedDisplay, meta = (EditCondition = "bOwnShading"))
    bool bCubicFiltering = true;

    /** Anisotropic filtering of the wave clipmap at grazing views (stage W3y): up to this many
     *  lookups along the water a pixel covers lengthwise, so the waves' and the foam's detail
     *  across the view survives at a distance instead of blurring. 1: none. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", AdvancedDisplay, meta = (EditCondition = "bOwnShading", ClampMin = "1", ClampMax = "16"))
    int32 WaveAnisotropy = 4;

    /** How opaque the foam's thinnest parts are (its edges, fading trails) against its solid
     *  middles (stage W3s). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", ClampMax = "1.0"))
    float FoamThinOpacity = 0.25f;

    /** How far into the foam (0 to 1 of its thickness) it turns from its thin opacity to solid
     *  (stage W3s): higher, a wider translucent falloff at its edges. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.01", ClampMax = "1.0"))
    float FoamFalloff = 0.6f;

    /** The foam's texture set (stage W3g): a foam .vxnt in the plugin's NoiseSets folder, without
     *  the extension (Tools/NoiseTileTool/foam_tile.py makes one). Its lace, streaks and
     *  whitecaps are drawn where the foam's density puts foam. Empty or missing: the procedural
     *  lace. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading"))
    FString FoamSet = TEXT("foam_b");

    /** How wide the foam set's tile is on the water (m). Larger: larger holes and strands, and a
     *  longer repeat. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "1.0", UIMax = "200.0"))
    float FoamTile = 32.0f;

    /** How strongly sunlight shines through thin, steep crests toward the eye (stage W3h): the
     *  teal glow of a wave seen against the sun. 0: none; 1 as the water's scattering gives it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", UIMax = "4.0"))
    float CrestGlow = 1.0f;

    /** How much of the water's own light (what comes up through the surface) is GodotOceanWaves'
     *  (stage W4d; its water.gdshader, after Atlas' GDC 2019 water: a crest's glow by its height and
     *  the sun behind it, the water seen head on brighter, a half Lambert, all tinted by its water
     *  colour) against ours (the water's single scattering and the crest glow above). 1: Godot's,
     *  0: ours. The bed seen through shallow water stays ours either way. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", ClampMax = "1.0"))
    float GodotWaterLight = 1.0f;

    /** The water's colour in Godot's light (GodotWaterLight; linear): GodotOceanWaves' water_color,
     *  sRGB (0.1, 0.15, 0.18). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading"))
    FLinearColor GodotWaterColor = FLinearColor(0.0103f, 0.0194f, 0.0273f);

    /** How much higher a whitecap's billows stand than the lace's relief (stage W3h; 1: the same). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", AdvancedDisplay, meta = (EditCondition = "bOwnShading", ClampMin = "1.0", UIMax = "8.0"))
    float WhitecapRelief = 3.0f;

    /** Foam where the water meets anything steep standing in it: rocks, cliffs, piers, hulls
     *  (stage W3l; from the scene's depth behind the water, so only what the view sees under the
     *  surface). It surges as a crest arrives and drains in the trough; the very contact whitens.
     *  Gentle beaches are the surf's (their slope is under the gate). 0: none; 1 as designed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.0", UIMax = "2.0"))
    float IntersectionFoam = 1.0f;

    /** How far the intersection foam reaches out from the contact (m), and how deep under the
     *  surface a contact may lie and still foam it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", meta = (EditCondition = "bOwnShading", ClampMin = "0.1", UIMax = "10.0"))
    float IntersectionFoamWidth = 1.5f;

    /** Spray (stage W3x; GodotOceanWaves' sea spray): large soft splashes rising off the dense foam
     *  near the camera, riding the waves, sinking back and dissolving; GPU billboards. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", meta = (EditCondition = "bOwnShading"))
    bool bSpray = true;

    /** How many splashes are born (1: Godot's, 0.24 candidates per square metre of dense foam per
     *  second; 0: none). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", meta = (EditCondition = "bOwnShading && bSpray", ClampMin = "0.0", UIMax = "4.0"))
    float SprayAmount = 1.0f;

    /** How far from the camera spray lives (m); it's born within half of it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", meta = (EditCondition = "bOwnShading && bSpray", ClampMin = "10.0", UIMax = "2000.0"))
    float SprayDistance = 300.0f;

    /** The most splashes alive at once (each 48 bytes on the GPU; up to an eighth of them are born
     *  a frame). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", AdvancedDisplay, meta = (EditCondition = "bOwnShading && bSpray", ClampMin = "1024", ClampMax = "262144"))
    int32 SprayParticles = 64000;

    /** The foam splashes rise from (stage W3x; Godot's 0.9; the foam's own value, 1 / 0.75 at its
     *  most): lower, from thinner foam too. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", meta = (EditCondition = "bOwnShading && bSpray", ClampMin = "0.0", ClampMax = "1.3"))
    float SprayFoam = 0.9f;

    /** The splashes' size (1: Godot's, up to 20 m wide and 8.5 m tall on the densest foam). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", meta = (EditCondition = "bOwnShading && bSpray", ClampMin = "0.05", UIMax = "3.0"))
    float SpraySize = 1.0f;

    /** The splashes' opacity (1: Godot's, two thirds at their most, before their dissolve). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", meta = (EditCondition = "bOwnShading && bSpray", ClampMin = "0.0", UIMax = "1.5"))
    float SprayOpacity = 1.0f;

    /** How far each splash is broken up by the foam texture's billows (0: a soft blob; 1: as
     *  designed; up to 3: sparser), besides its dissolve. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spray", meta = (EditCondition = "bOwnShading && bSpray", ClampMin = "0.0", ClampMax = "3.0"))
    float SprayBreakup = 1.0f;

    /** The stand-in sky until the atmosphere is connected (stage W3b): its radiance at the zenith
     *  and at the horizon per unit of the sun's illuminance (the world's directional light), by
     *  day; it dims through twilight. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", AdvancedDisplay, meta = (EditCondition = "bOwnShading"))
    FLinearColor StandInSkyZenith = FLinearColor(0.035f, 0.07f, 0.14f);
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shading", AdvancedDisplay, meta = (EditCondition = "bOwnShading"))
    FLinearColor StandInSkyHorizon = FLinearColor(0.09f, 0.11f, 0.13f);

    // --- Depth ---

    /** Leave out the triangles the land stands well above (the terrain hides them anyway). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Depth")
    bool bCullUnderLand = true;

    /** How far (cm) the land must stand above the sea at a triangle's corners, beyond what the
     *  terrain's slope bound and the waves' reach allow for, before it is left out. Covers the
     *  terrain mesh's error. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Depth", meta = (ClampMin = "0.0", UIMax = "5000.0", EditCondition = "bCullUnderLand"))
    double LandCullMargin = 498.0;

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

    /** Forward scattering anisotropy: the ocean's own pass (stage W3), or a Single Layer Water
     *  surface (VoxelOceanPhaseG). */
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

    /** The foam on the surface seen from under water (whitecaps, the surf's bores and swash: the
     *  Foam category's), in the scene's light units (as UnderwaterLight): lit from above, brighter
     *  than the water around it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optics")
    FLinearColor UnderwaterFoamColor = FLinearColor(0.6f, 0.7f, 0.75f);

    // --- Environment (the constant weather: everywhere when bProceduralWeather is off) ---

    /** Wind speed (m/s) with the procedural weather off. The FFT waves blend their sea states by
     *  it (5, 10 and 18 m/s). Stage W4a: 10, GodotOceanWaves' demo's main cascade. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", UIMax = "40.0"))
    double WindSpeed = 10.0;

    /** Wind heading (degrees from local north toward east; north is the planet's +Z): the
     *  direction the waves travel, with the procedural weather off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (UIMin = "0.0", UIMax = "360.0"))
    double WindHeading = 0.0;

    /** Storm intensity, 0-1, with the procedural weather off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double StormIntensity = 1.0;

    /** The ocean clock's rate against the game's (server) time. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Environment", meta = (ClampMin = "0.0", UIMax = "4.0"))
    double OceanTimeScale = 1.0;

    // --- Waves (the FFT model, voxel.Ocean.WaveModel 2; a change rebuilds the waves) ---

    /** The waves' seed: every machine with the same seed, clock and wind draws the same sea. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves")
    int32 WaveSeed = 1;

    /** The distance (km) the wind has blown over open water: a shorter fetch makes younger,
     *  shorter, lower waves. Past what the wind needs to raise a fully developed sea, nothing
     *  changes (about 300 km at 15 m/s). Stage W4a: 150, GodotOceanWaves' demo's. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "1.0", UIMax = "2000.0"))
    double WaveFetch = 150.0;

    /** Horizontal displacement per unit of the waves' own (0: rounded sines; 1: sharp crests).
     *  Higher folds the surface where crests break. Stage W4a: 1, GodotOceanWaves' (its displacement
     *  and its J at the spectrum's own). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "0.0", ClampMax = "2.0"))
    double WaveChop = 1.0;

    /** Long waves narrowed into parallel trains, 0-1 (Horvath's swell). Stage W4a: 0.8,
     *  GodotOceanWaves' demo's (the same spreading). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "0.0", ClampMax = "1.0"))
    double WaveSwell = 0.8;

    /** The detiling lattice's cell (m) for the 1 km band: every band blends three shifted copies
     *  over a lattice this size scaled by its own tile (stage O9b3: 1,500, 363, 89 and 21 m at the
     *  default), so no band's tile repeats. 0: off. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "0.0", UIMax = "5000.0"))
    double WaveDetileCell = 1500.0;

    /** The surface query includes the 3-15 m waves too (most of a calm sea; 0.7 ms more per
     *  update). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves")
    bool bQueryShortWaves = false;

    /** How often the wave cascades update (Hz; stage W3w, as GodotOceanWaves: 50): their FFTs run
     *  at this rate whatever the frame rate, the waves stepping at it. 0: every frame. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (ClampMin = "0.0", UIMax = "120.0"))
    double WaveUpdateRate = 50.0;

    /** Mip bias of the wave lookups: above 0 blurs sooner (cheaper, softer), below sharpens. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves", meta = (UIMin = "-2.0", UIMax = "2.0"))
    float WaveMipBias = 0.0f;

    /** Per-octave gains on the waves' height (stage W3k): raise the 64-256 m octaves for a
     *  heavier sea, the 1-4 m ones for a choppier one, lower the shortest for a glassier surface.
     *  All 1: the spectrum as the wind makes it. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Waves")
    FVoxelOceanWaveEqualizer WaveEqualizer;

    // --- Weather (the procedural wind, storms and swell until the atmosphere; Design/OceanWaves.md section 4) ---

    /** The procedural weather: latitude bands, noise and drifting storms, the same on every
     *  machine. Off: the Environment constants everywhere. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather")
    bool bProceduralWeather = true;

    /** The weather's seed. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather"))
    int32 WeatherSeed = 1;

    /** The trade winds' speed (m/s, easterly, below 30 degrees). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "20.0"))
    double TradeWind = 7.0;

    /** The westerlies' speed (m/s, 30 to 60 degrees). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "20.0"))
    double Westerly = 10.0;

    /** The polar easterlies' speed (m/s, above 60 degrees). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "20.0"))
    double PolarWind = 5.0;

    /** The wind's noise (m/s) and its size (km). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "10.0"))
    double WindNoise = 3.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "10.0", UIMax = "3000.0"))
    double WindNoiseSize = 600.0;

    /** Storms alive at once. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0", ClampMax = "64"))
    int32 StormCount = 6;

    /** A storm's peak wind (m/s), radius (km) and life (hours of weather time). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "40.0"))
    double StormWind = 22.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "10.0", UIMax = "1000.0"))
    double StormRadius = 500.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.1", UIMax = "24.0"))
    double StormHours = 3.0;

    /** The swell a storm sends (a fraction of its wind) and how far it carries (km). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", ClampMax = "1.0"))
    double SwellStrength = 0.6;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "10.0", UIMax = "5000.0"))
    double SwellRange = 100000.0;

    /** The weather clock against the ocean's (above 1 to watch storms move) and its offset (s:
     *  where in the weather a level starts). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather", ClampMin = "0.0", UIMax = "100.0"))
    double WeatherTimeScale = 1.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", meta = (EditCondition = "bProceduralWeather"))
    double WeatherTimeOffset = 0.0;

    /** Weather seconds between the waves' weather keys; a heading's step (degrees); how long a
     *  turn of the waves takes (weather seconds). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", AdvancedDisplay, meta = (ClampMin = "10.0", UIMax = "3600.0"))
    double WeatherKeySeconds = 300.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "45.0"))
    double WaveHeadingStep = 10.0;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Weather", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "300.0"))
    double WaveTurnSeconds = 30.0;

    // -- Shore (stage O8; Design/OceanWaves.md section 5) --------------------------------------

    /** Waves shoal in shallow water (they grow, steepen, break and fade at the coast) and slow
     *  toward the coast. Off: deep-water waves everywhere (the shore field still keeps inland
     *  basins dry). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore")
    bool bShoaling = true;

    /** The breaking index: the waves' significant height is held under this times the depth. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.1", ClampMax = "2.0"))
    double ShoreBreaking = 0.78;

    /** How much the orbits flatten in shallow water: the horizontal displacement grows as
     *  1 / tanh(k d) to this power (0: not at all, 1: as linear waves). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", ClampMax = "2.0"))
    double ShoreChopGrowth = 1.0;

    /** The FFT's own waves shorten and slow toward the coast (the shallow-water dispersion, along
     *  the shore field's direction): 1 as the dispersion says, 0 off. Off by default since stage
     *  O9f: it moves the FFT's lookups by up to kilometres along the field's direction, and
     *  wherever that direction turns quickly (midway between coasts, across a bay's mouth) the
     *  FFT's pattern tears into a seam. The swell's and the surf's trains (stages O9a, O9d) turn
     *  and shorten toward the shore themselves. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (EditCondition = "bShoaling", ClampMin = "0.0", ClampMax = "2.0"))
    double ShoreWarp = 0.0;

    /** The FFT waves fade out over this distance to the coast (m; the depth over the beach
     *  slope). 0: no fade; the surf (Surf category) takes the long waves over near the coast by
     *  itself. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", UIMax = "1000.0"))
    double ShoreFadeDistance = 999.0;

    /** The least share of their open-sea height the FFT waves keep near the coast and under the
     *  land (stage W3g): the shoaling's breaking and fade stop here, so no part of the ocean mesh
     *  stands still and a channel dug into the shore has waves at once. The terrain hides them
     *  elsewhere, and the thin water's drop keeps them under the ground. 0: as before (none at
     *  the shoreline). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", ClampMax = "1.0"))
    double ShoreWaveFloor = 0.15;

    /** The depth the waves feel leaves out terrain features narrower than about twice this (m):
     *  waves don't feel bumps much shorter than themselves, and the surf's timing follows the
     *  depth so closely on a gentle beach that the terrain's fine detail broke its crests into
     *  spikes. Larger: smoother crests that follow the bottom less. 0: the mesh's own detail.
     *  The land cull and the query's reported depth keep the terrain's real height. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (EditCondition = "bShoaling", ClampMin = "0.0", UIMax = "50.0"))
    double ShoreDepthSmoothing = 10.0;

    /** The gentlest beach slope the shoaling assumes (the land bake's coastal slope, at least). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (EditCondition = "bShoaling", ClampMin = "0.0001", UIMax = "0.1"))
    double ShoreMinSlope = 0.005;

    /** The ocean mesh near coasts (stage O9c): within ShoreDetailBand of the coast it is as fine
     *  as if the camera were this many times closer, so the surf's bores keep their shape from a
     *  few hundred metres off the beach. 1: off. Costs tiles with the camera by a coast (3: about
     *  a third more by a straight one, up to twice as many by a busy one with islands). With the
     *  FFT waves (their vertex code; no other wiring). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", meta = (ClampMin = "1.0", UIMax = "4.0"))
    double ShoreDetail = 3.0;

    /** The band of finer mesh (m from the coastline, the shore field's zero line: either side);
     *  outside it the refinement fades over the camera's own distance. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "2000.0"))
    double ShoreDetailBand = 1000.0;

    /** How far the finer mesh reaches from the camera (m, up to 2 km): it saves at most
     *  (1 - 1 / ShoreDetail) of this distance, so far coasts stay nearly as coarse as without it.
     *  The coast is measured to 10 km: past that, every point saves the same little (a third of a
     *  kilometre at the defaults, from 10 km out: a percent or two more tiles there). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "0.0", ClampMax = "2000.0"))
    double ShoreDetailReach = 1000.0;

    /** The shore field holds distances up to this (km); the slowing reaches no farther. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "1.0", UIMax = "200.0"))
    double ShoreFieldReach = 1000.0;

    /** Inland basins below sea level lose their ocean water beyond this many land bake cells from
     *  the sea (the bake's coast is a cell or so from the terrain's own): from here to 1.5 times it
     *  the sea's surface sinks below the ground (stage O9b3), past that the basin is land. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Shore", AdvancedDisplay, meta = (ClampMin = "0.0", UIMax = "8.0"))
    double ShoreBasinMargin = 1.5;

    // -- Coast (stage O9h; Design/OceanWaves.md section 23) -------------------------------------

    /** The coastal travel-time field: baked at load from the terrain along every sea coast, the
     *  time the waves take to come in from deep water to each point. It gives the shore's waves
     *  their direction and crest spacing (stage O9i). Seconds to bake, after the shore field. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coast", meta = (EditCondition = "bShoaling"))
    bool bCoastField = true;

    /** The memory the field's atlas may take on the GPU (MB; its index takes 6 more slices of
     *  the shore texture, 25 MB on a 1024 bake): it picks the finest cells that fit, 25 to 150 m,
     *  and where even 150 m cells don't fit it leaves out the tiles farthest from the coast (the
     *  log says which). Finer cells space the crests more exactly near steep beaches; the bake's
     *  time grows with the cells (at most 40 million: about a minute's solve). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coast", meta = (EditCondition = "bCoastField", ClampMin = "8.0", ClampMax = "1024.0", UIMax = "512.0"))
    double CoastMemoryMB = 128.0;

    /** The field's outer edge (m of depth): the waves' rays start shore-normal there. The shore's
     *  waves ramp in from shallower water than this. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coast", AdvancedDisplay, meta = (EditCondition = "bCoastField", ClampMin = "5.0", UIMax = "100.0"))
    double CoastSeedDepth = 40.0;

    /** The field reaches no farther out than this (km of the shore field's distance), on shelves
     *  too gentle to reach the seed depth sooner. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Coast", AdvancedDisplay, meta = (EditCondition = "bCoastField", ClampMin = "0.5", UIMax = "30.0"))
    double CoastReach = 8.0;

    // -- Surf (stage O9a; Design/OceanWaves.md section 6) ----------------------------------------

    /** Surf near the coast: the long waves become trains that follow the depth's contours, break
     *  into bores and run up the beach as swash. Needs bShoaling and the shore field. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bShoaling"))
    bool bSurf = true;

    /** The surf's height against the sea state's long waves (1: their significant height). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", UIMax = "3.0"))
    double SurfStrength = 1.0;

    /** How much the waves come in sets (groups of six, larger and smaller) and vary along the
     *  coast in height and timing: 0 every wave alike everywhere, 1 full. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", ClampMax = "1.0"))
    double SurfSets = 1.0;

    /** The swash's run-up against Stockdon's (2006) estimate for the waves and the beach slope
     *  (0: the waterline stays put). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", UIMax = "3.0"))
    double SurfSwash = 0.33;

    /** The swash's swing either side of its mean is held within this distance up and down the
     *  beach (m, at the shore field's beach slope): on a gentle beach Stockdon's infragravity
     *  swash alone would sweep tens of metres. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "1.0", UIMax = "50.0"))
    double SurfSwashReach = 1000.0;

    /** How far a breaking wave's landward face steepens: its front grows 1 / (1 - this) times as
     *  steep as its back once the wave is saturated (0: symmetric). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", ClampMax = "0.95"))
    double SurfSkew = 0.85;

    /** Where the shore's directed waves begin (stage O9i): where the coastal field's depth is
     *  shallower than this many deep wavelengths of the sea's peak period (fully in from 40% of
     *  it). 0.2: about 25 m deep for a 9 s swell. Deeper, the FFT's waves alone. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", ClampMax = "0.5"))
    double SurfCutoff = 0.2;

    /** The share of the long waves' energy the FFT's waves always keep near the coast (stage O9i):
     *  their irregular crests and chop under the shore's directed waves. 1: no directed waves.
     *  Stage W5b: 0.5 (was 0.25), the open sea's waves blended further into the surf, breaking up
     *  its long straight crests and easing the sea's flattening toward the coast. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Surf", meta = (EditCondition = "bSurf", ClampMin = "0.0", ClampMax = "1.0"))
    double SurfFFTFloor = 0.5;

    // -- Foam (stage O9b; Design/OceanWaves.md section 6) ----------------------------------------

    /** Foam: whitecaps on the open sea (the material's VoxelOceanFoam texture and Foam output) and
     *  white water on the surf's bores and swash. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam")
    bool bFoam = true;

    /** Where the sea foams (stage W3v; GodotOceanWaves' "whitecap"): the surface's area ratio J
     *  (1 flat, lower where a crest squeezes it) under which foam builds, by how far under. Higher:
     *  more foam, from gentler crests. Stage W4a: 0.5, GodotOceanWaves' own: at its height (the
     *  equalizer at 2.83), chop 1 and the demo's sea, 0.49 on our summed surface foams as much as its
     *  cascade at 0.5 (about 18% of the sea at 10 m/s; Tools/OceanHarness/godot_match). At the
     *  physical height (the equalizer at 1) the same foam takes 0.85. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", ClampMax = "1.0"))
    double FoamWhitecap = 0.5;

    /** How gradually the foam comes in (stage W3y): the span of J under the whitecap threshold
     *  over which a crest's foam goes from none (fully translucent) at the threshold to solid at
     *  its peak. Wider: softer edges and more translucent foam around the crests, less blobby;
     *  lower the threshold's rise with it to keep the amount. 0: Godot's own (its growth over its
     *  decay: nearly on or off; the default since stage W4a). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", UIMax = "0.5"))
    double FoamRamp = 0.0;

    /** How much foam builds and how long it lasts (stage W3v; GodotOceanWaves' "foam amount", 0 to
     *  10): foam grows at 7.5 x this per second while the surface is squeezed and decays at 1.15 x
     *  (10 - this) per second (at least 0.575): higher, denser patches and longer trails. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", UIMax = "10.0"))
    double FoamAmount = 8.0;

    /** The fine foam (stage W3w; GodotOceanWaves' short cascade, its own whitecap threshold): the
     *  shortest waves' own foam (crests a few metres long and under), built in their own tile at a
     *  few centimetres a texel and added to the sea's. 0: none. Stage W4a: 0.38, matching the foam of
     *  GodotOceanWaves' 16 m cascade (its whitecap 0.25 at 20 m/s) at its height (godot_match). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", ClampMax = "1.0"))
    double FineFoamWhitecap = 0.38;

    /** The fine foam's amount (stage W3w; Godot's foam amount for its short cascade, 3 in its
     *  demo, now the default): quick specks at low values, denser and lasting at high. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", UIMax = "10.0"))
    double FineFoamAmount = 3.0;

    /** How many of the wave clipmap's levels around the camera keep the foam's history at four
     *  times their resolution (stage W4c; 0 to 4): the trails keep the shape of the crests that left
     *  them, as GodotOceanWaves' per-cascade foam does, instead of the clipmap texel's (20 cm and up,
     *  coarser with height). Past them, and on the surf's foam, the clipmap's own history. Each level
     *  is 4 MB at the default clipmap size; all are stepped each wave update. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0", ClampMax = "4"))
    int32 FoamDetailLevels = 4;

    /** The surf's foam (0: none): the bores' white water, their short wake and the swash's foamy
     *  edge, fed into the same foam as the open sea's (stage W3v), so it builds and lingers there
     *  as the crests' does. Stage W5a: a density (1: a breaking face solid white), no longer times
     *  the open sea's gain; the default 1 (was 0.5). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", UIMax = "2.0"))
    double SurfFoam = 1.0;

    /** How long the surf's white water lingers behind a bore (stage W5a; s, its fade's time
     *  constant; the open sea's whitecaps fade by FoamAmount). */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.05", UIMax = "10.0"))
    double SurfFoamLinger = 2.5;

    /** How much faster foam fades where its water stretches (stage W5a; 0: evenly): trails break
     *  into lacy patches along the short waves riding through them, as real foam gathers where the
     *  surface converges and thins where it spreads, rather than fading as one smooth sheet. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Foam", meta = (EditCondition = "bFoam", ClampMin = "0.0", UIMax = "20.0"))
    double FoamBreakup = 6.0;

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
        WaveFetch = FMath::Max(WaveFetch, 1.0);
        WaveChop = FMath::Clamp(WaveChop, 0.0, 2.0);
        WaveSwell = FMath::Clamp(WaveSwell, 0.0, 1.0);
        WaveEqualizer.Sanitize();
        WaveDetileCell = FMath::Max(WaveDetileCell, 0.0);
        TradeWind = FMath::Max(TradeWind, 0.0);
        Westerly = FMath::Max(Westerly, 0.0);
        PolarWind = FMath::Max(PolarWind, 0.0);
        WindNoise = FMath::Max(WindNoise, 0.0);
        WindNoiseSize = FMath::Max(WindNoiseSize, 10.0);
        StormCount = FMath::Clamp(StormCount, 0, 64);
        StormWind = FMath::Max(StormWind, 0.0);
        StormRadius = FMath::Max(StormRadius, 10.0);
        StormHours = FMath::Max(StormHours, 0.1);
        SwellStrength = FMath::Clamp(SwellStrength, 0.0, 1.0);
        SwellRange = FMath::Max(SwellRange, 10.0);
        WeatherTimeScale = FMath::Max(WeatherTimeScale, 0.0);
        WeatherKeySeconds = FMath::Max(WeatherKeySeconds, 10.0);
        WaveHeadingStep = FMath::Clamp(WaveHeadingStep, 0.0, 45.0);
        // A turn ends within its key (the next key's starts from this one's headings).
        WaveTurnSeconds = FMath::Clamp(WaveTurnSeconds, 0.0, WeatherKeySeconds);
        ShoreBreaking = FMath::Clamp(ShoreBreaking, 0.1, 2.0);
        ShoreChopGrowth = FMath::Clamp(ShoreChopGrowth, 0.0, 2.0);
        ShoreWarp = FMath::Clamp(ShoreWarp, 0.0, 2.0);
        ShoreFadeDistance = FMath::Max(ShoreFadeDistance, 0.0);
        ShoreWaveFloor = FMath::Clamp(ShoreWaveFloor, 0.0, 1.0);
        ShoreMinSlope = FMath::Max(ShoreMinSlope, 1.0e-4);
        ShoreDetail = FMath::Clamp(ShoreDetail, 1.0, 8.0);
        ShoreDetailBand = FMath::Clamp(ShoreDetailBand, 0.0, 5000.0);
        ShoreDetailReach = FMath::Clamp(ShoreDetailReach, 0.0, 2000.0);
        ShoreFieldReach = FMath::Max(ShoreFieldReach, 1.0);
        ShoreBasinMargin = FMath::Max(ShoreBasinMargin, 0.0);
        ShoreDepthSmoothing = FMath::Clamp(ShoreDepthSmoothing, 0.0, 500.0);
        SurfStrength = FMath::Max(SurfStrength, 0.0);
        SurfSets = FMath::Clamp(SurfSets, 0.0, 1.0);
        SurfSwash = FMath::Max(SurfSwash, 0.0);
        SurfSwashReach = FMath::Max(SurfSwashReach, 1.0);
        SurfSkew = FMath::Clamp(SurfSkew, 0.0, 0.95);
        SurfCutoff = FMath::Clamp(SurfCutoff, 0.0, 0.5);
        SurfFFTFloor = FMath::Clamp(SurfFFTFloor, 0.0, 1.0);
        FoamWhitecap = FMath::Clamp(FoamWhitecap, 0.0, 1.0);
        FoamAmount = FMath::Max(FoamAmount, 0.0);
        FoamRamp = FMath::Max(FoamRamp, 0.0);
        FineFoamWhitecap = FMath::Clamp(FineFoamWhitecap, 0.0, 1.0);
        FineFoamAmount = FMath::Max(FineFoamAmount, 0.0);
        FoamDetailLevels = FMath::Clamp(FoamDetailLevels, 0, 4);
        WaveUpdateRate = FMath::Max(WaveUpdateRate, 0.0);
        SurfFoam = FMath::Max(SurfFoam, 0.0);
        SurfFoamLinger = FMath::Max(SurfFoamLinger, 0.05);
        FoamBreakup = FMath::Max(FoamBreakup, 0.0);
        ShortWaveSlopeVariance = FMath::Max(ShortWaveSlopeVariance, 0.0f);
        Refraction = FMath::Max(Refraction, 0.0f);
        FoamTile = FMath::Max(FoamTile, 1.0f);
        CrestGlow = FMath::Max(CrestGlow, 0.0f);
        GodotWaterLight = FMath::Clamp(GodotWaterLight, 0.0f, 1.0f);
        WhitecapRelief = FMath::Max(WhitecapRelief, 1.0f);
        IntersectionFoam = FMath::Max(IntersectionFoam, 0.0f);
        IntersectionFoamWidth = FMath::Max(IntersectionFoamWidth, 0.1f);
        SprayAmount = FMath::Max(SprayAmount, 0.0f);
        SprayDistance = FMath::Max(SprayDistance, 10.0f);
        SprayParticles = FMath::Clamp(SprayParticles, 1024, 262144);
        SprayFoam = FMath::Clamp(SprayFoam, 0.0f, 1.3f);
        SpraySize = FMath::Max(SpraySize, 0.05f);
        SprayOpacity = FMath::Max(SprayOpacity, 0.0f);
        SprayBreakup = FMath::Clamp(SprayBreakup, 0.0f, 3.0f);
        FoamOpacity = FMath::Clamp(FoamOpacity, 0.0f, 1.0f);
        FoamThinOpacity = FMath::Clamp(FoamThinOpacity, 0.0f, 1.0f);
        FoamFalloff = FMath::Clamp(FoamFalloff, 0.01f, 1.0f);
        FoamSoftness = FMath::Clamp(FoamSoftness, 0.0f, 1.0f);
        FoamDetail = FMath::Clamp(FoamDetail, 0.0f, 1.0f);
        FoamDetailDistance = FMath::Max(FoamDetailDistance, 10.0f);
        WaveAnisotropy = FMath::Clamp(WaveAnisotropy, 1, 16);
    }

    /** The fields whose change rebuilds the surface. */
    bool SameMesh(const FVoxelOceanSettings& Other) const
    {
        return FinestSpacing == Other.FinestSpacing && TileQuads == Other.TileQuads && SpareBufferSets == Other.SpareBufferSets
            && bWavesInCompute == Other.bWavesInCompute && bWaveClipmap == Other.bWaveClipmap && ClipSize == Other.ClipSize
            && ClipLevels == Other.ClipLevels && ClipFinestSpacing == Other.ClipFinestSpacing && bWaveRings == Other.bWaveRings
            && FarSphereCells == Other.FarSphereCells && bOwnShading == Other.bOwnShading;
    }

    /** The fields the surface material receives. */
    bool SameOptics(const FVoxelOceanSettings& Other) const
    {
        return Absorption == Other.Absorption && Scattering == Other.Scattering && PhaseG == Other.PhaseG;
    }
};
