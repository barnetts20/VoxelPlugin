Ocean harnesses (stages O1-O4, O6a, O6b, O7, O8, O9a-O9c). Plain C++20; no engine. Each file's head comment says what it checks.

  ocean_tiles_harness.cpp   the selection and tile geometry of VoxelOceanTiles.h, compiled unchanged:
                            balance, morph chains, seams and cracks under test waves, rays; O9c: the same
                            with the split bias near shores on a synthetic coast by each camera (args:
                            cameras per factor, morph start fraction, bias cameras)
      g++ -O2 -std=c++20 -DVOT_HEADER='"<plugin>/Source/VoxelPlugin/Public/VoxelOceanTiles.h"' -I. ocean_tiles_harness.cpp -o ocean_tiles_harness
      (several minutes; expected_tiles_o4.txt is the O4 run's output)
  ocean_pages_stats.cpp     pages, slots and triangles per frame (same -DVOT_HEADER and -I.)
  waves_check.cpp           O4: the wave hook's maths (normal, Jacobian, velocity), the CPU query's
                            convergence, the shader's float height against the query, the underwater exit
  underwater_check.cpp      O3b: VoxelUnderwater.ush's float maths against double references
  ray_check.cpp             O3b: the underwater pass's ray from the depth buffer
  normal_check.cpp          O2b: VoxelOceanPixel's normal against finite differences
      g++ -O2 -std=c++20 <file>.cpp -o <name>

  spectrum_check.cpp        O6a: the FFT cascades' shared code (VoxelOceanSpectrum.ush) and CPU side
                            (VoxelOceanSpectrum.h): spreading, Hs per wind, band split, exact dispersion,
                            the GPU passes emulated in float against a direct DFT, variance, time loop,
                            the 128^2 query cascades, fp16, folding, detiling, determinism, CPU cost
  spectrum_view.cpp         O6a: pictures of the sea from the emulated cascades (perspective from an
                            altitude, top-down relief), with or without detiling
  ocean_fft_emulation.h     VoxelOceanFFT.usf's passes in float, shared by the two above
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private -I. spectrum_check.cpp -o spectrum_check     (same for spectrum_view)
      ./spectrum_view <wind> persp|top <altitude or extent m> <detile cell m, 0 off> <detiled bands> out.ppm

  chart_check.cpp           O6b: the waves' charts (VoxelOceanCharts.ush/.h): the table, coverage, continuity
                            across face edges and corners, the GPU's lookup precision against the CPU's,
                            the wind turn
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private chart_check.cpp -o chart_check
  hook_check.cpp            O6b, O7: the wave hook's FFT path (ported from VoxelOceanWaves.ush, over
                            emulated textures and the weather's table) against the FFT model
                            (VoxelOceanWavesFFT.cpp compiled unchanged) in the procedural weather at a
                            stormy key: drawn against queried waves (settled and turning), continuity,
                            the query's inversion, the underwater waterline, detiling off, storm against
                            calm seas, no jump between keys, the reach, the model's cost
      g++ -O2 -std=c++20 -ffp-contract=off -IUEStandIn -I<plugin>/Source/VoxelPlugin/Public
          -I<plugin>/Source/VoxelPlugin/Private -I<plugin>/Shaders/Private -I. hook_check.cpp
          <plugin>/Source/VoxelPlugin/Private/VoxelOceanWavesFFT.cpp <plugin>/Source/VoxelPlugin/Private/VoxelOceanWeather.cpp
          <plugin>/Source/VoxelPlugin/Private/VoxelOceanWeatherTable.cpp -o hook_check
  hook_view.cpp             O6b: a top-down relief image of the hook's sea around a cube corner (seams)
  underwater_map_check.cpp  O6b: the underwater pass's surface map (VoxelUnderwaterMap.ush/.h) against
                            the exact search it replaces: heights by level, the waterline, continuity
                            across levels, normals, exits; O9c: the foam map's reads and the map pass's
                            surface point; optional wind argument (default 12 m/s)
      (both built as hook_check, with their own .cpp)
  weather_check.cpp         O7: the procedural weather (VoxelOceanWeather.cpp) and its table
                            (VoxelOceanWeatherTable.cpp, read through VoxelOceanSeaState.ush): bands,
                            storms (cyclonic, drifting), swell, smoothness, determinism, the field against
                            the source and across face edges, the headings' steps, the key's cost
      (built as hook_check, with weather_check.cpp)
  weather_view.cpp          O7: a map of the planet's weather (wind speed, wind strokes, storms)
      g++ -O2 -std=c++20 -IUEStandIn -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private -I. weather_view.cpp <plugin>/Source/VoxelPlugin/Private/VoxelOceanWeather.cpp -o weather_view
      ./weather_view <weather time s> out.ppm
  shore_check.cpp           O8: the land bake's sea-connected flood (VoxelLandSea.h) and the shore field (VoxelOceanShore.ush,
                            VoxelOceanShoreBuild.ush through VoxelOceanShoreCore.h) on a synthetic planet with an inland
                            basin: the flood's rule, the jump flooding against exact distances (the gradient near the
                            coast), directions, slopes, the smoothing passes, the texels' packing, sampling across face edges and corners;
                            O9c: the coastline (the split bias's segments) against the field's zero line; the shoaling maths (VoxelOceanShoal.ush:
                            dispersion, the warp integral, gains, breaking, the fade, the lookup transform)
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private shore_check.cpp -o shore_check
      ./shore_check [N]   (cells per face edge, default 96)
  shore_hook_check.cpp      O8: the shoaled hook (hook_emulation.h) against the model near a round continent's coast,
                            a wind sea and a swell: the field's direction and distance along the coast at a fixed
                            distance (the wobble), drawn against queried waves, the shoaling and the warp at work,
                            the surf (drawn against queried from the beach out, the swash wetting and drying the beach),
                            shorter waves toward the coast, no seams (the texel-space expansion near the camera), the
                            underwater waterline against the query, the reach
      (built as hook_check, with shore_hook_check.cpp and -pthread)
  surf_check.cpp            O9i: the shore's waves (VoxelOceanSurf.ush) from a coastal field integrated along straight rays
                            on uniform beaches: the composition (cutoff, ramp, the FFT's floor, cover, reach), the crests'
                            spacing against the exact wavelength, the gradient and rate against finite differences,
                            continuity (depth, the still level, the clock's wrap), the swash and its fade on land by the
                            mesh, the trains' fade by the mesh, the bores' least front, the foam, the bound
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private surf_check.cpp -o surf_check
  foam_check.cpp            O9b: the open sea's foam (VoxelOceanFoam.ush: the per-slot envelope, the point's combination) on
                            the emulated cascades with the runtime's layout and the hook's detiling, weights and mips (the
                            spread estimate, coverage against the calibration from 5 to 25 m/s, its evenness over the
                            detiling lattice and with distance, the trails, the coverage setting) and the surf's foam
                            (VoxelOceanSurf.ush: bores, no step at the trough, the mean unresolved, the swash's edge, off)
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private -I. foam_check.cpp -o foam_check
  coast_check.cpp           O9h: the coastal travel-time field (VoxelOceanCoastCore.h's plan, solve and packing,
                            VoxelOceanCoast.ush's sampling) on a planet of known depths: the budget, the crests' spacing
                            against the exact local wavenumber, the rays' direction, continuity along the coast across
                            tiles and face edges, the texels against the solve, up the beach, the channel behind an island
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private coast_check.cpp -o coast_check
  clip_check.cpp            W1: the wave clipmap's levels (VoxelOceanClipmap.h, unchanged) and the shader's
                            camera-relative projection in float: precision from 2 m to 20,000 km up, a descent
                            from orbit and a flight along the sea with the static layer's strips and the waves'
                            staggering simulated texel by texel (every sample finds its own texel), edit rectangles
      g++ -O2 -std=c++20 -IClipStandIn -I<plugin>/Source/VoxelPlugin/Private clip_check.cpp -o clip_check
  shading_check.cpp         W3: the ocean's light model (VoxelOceanShading.ush) as C++: the sun glitter's reflectance
                            against mirror facets drawn from its slopes, Smith's Lambda fit against erfc, the mean
                            Fresnel fit against the facets the eye sees, the water's single scattering against a march
      g++ -O2 -std=c++20 -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Shaders/Private shading_check.cpp -o shading_check
  lace_check.cpp            W3f: the foam's structure (VoxelOceanFoamLace.ush) as C++: the patterns' distributions and
                            the thresholds' tables (--fit writes Shaders/Private/VoxelOceanFoamLaceFit.ush), the structure
                            keeping the density at every footprint, previews (PNG) near and far
      g++ -O2 -std=c++20 -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Shaders/Private lace_check.cpp -o lace_check
  ClipStandIn/              stand-ins for the engine types VoxelOceanClipmap.h uses
  UEStandIn/                stand-ins for the few engine types the FFT model and the weather use

CoreMinimal.h is a stand-in for the UE types VoxelOceanTiles.h uses. waves_check, underwater_check,
ray_check and normal_check reimplement the test model and the shader maths in double/float; the
FFT model (O6) should replace them with checks built on its shared HLSL/C++ code.
