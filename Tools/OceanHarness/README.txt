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
  surf_check.cpp            O9a: the surf (VoxelOceanSurf.ush) on a uniform beach: its gradient and rate against finite
                            differences, continuity in depth, across the clock's wrap and between trains, breaking under
                            0.78 d, the bores' steep fronts, the swash climbing the beach and falling below the still level,
                            the handover, the sets (wave to wave, along the coast), the resolution fade, the reach
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private surf_check.cpp -o surf_check
  foam_check.cpp            O9b: the open sea's foam (VoxelOceanFoam.ush: the per-slot envelope, the point's combination) on
                            the emulated cascades with the runtime's layout and the hook's detiling, weights and mips (the
                            spread estimate, coverage against the calibration from 5 to 25 m/s, its evenness over the
                            detiling lattice and with distance, the trails, the coverage setting) and the surf's foam
                            (VoxelOceanSurf.ush: bores, no step at the trough, the mean unresolved, the swash's edge, off)
      g++ -O2 -std=c++20 -ffp-contract=off -I<plugin>/Source/VoxelPlugin/Public -I<plugin>/Source/VoxelPlugin/Private
          -I<plugin>/Shaders/Private -I. foam_check.cpp -o foam_check
  UEStandIn/                stand-ins for the few engine types the FFT model and the weather use

CoreMinimal.h is a stand-in for the UE types VoxelOceanTiles.h uses. waves_check, underwater_check,
ray_check and normal_check reimplement the test model and the shader maths in double/float; the
FFT model (O6) should replace them with checks built on its shared HLSL/C++ code.
