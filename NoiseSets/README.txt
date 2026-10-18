The authored packed noise sets (.vxnt v3, from the noise generator project; Design/NoiseTiles2D_Handoff.md, Addenda 1 to 3).

Put the peaks sets here as peaks_<variant>.vxnt. They must have Addendum 2's eroded layout (R hard-rock height, G soft-rock height,
B blurred R, A flow accumulation); older sets (the first peaks_a, Addendum 1's cellular layout) are skipped with a warning in the log.

Every planet picks one by its seed, unless its PeakNoiseSet setting names one (the file name without .vxnt). Without any, the plugin
generates a stand-in set at load. The log lists the sets it found ("Noise sets in ...").

Put the hills sets here as hills_<variant>.vxnt (Addendum 3: R dissected height, G rounded height, B blurred R, A flow; the base
network at the tile's lowest height). Every planet picks one by its seed unless HillNoiseSet names one ("none": the procedural
hills). Without any, the procedural hills. The stand-in (hills_standin.vxnt, from Tools/NoiseTileTool/erode_tile.cpp BaseMode=2) is no longer shipped:
hills_a replaces it.

Put the strata sets here as strata_<variant>.vxnt (Addendum 3 section 2: R caprock height (mesas, benches, cliff bands), G badland
height, B blurred R, A flow; the base network at the tile's lowest height). They replace the hills in the bake's sedimentary
provinces (T4c). Every planet picks one by its seed unless StrataNoiseSet names one ("none": no strata). Without any, the hills
cover those provinces too. The stand-in (strata_standin.vxnt) is no longer shipped: strata_a
replaces it.

Put the detail sets here as detail_<variant>.vxnt (the handoff's section 6: R fbm, G ridged, B billow, A eroded fbm; 4 octaves
at lacunarity 2, the same coarsest lattice). They give the detail layer on all land (A, at up to four scales) and the peaks',
hills' and strata's detail (G hard rock, R soft rock) (N1). Every planet picks one by its seed unless DetailNoiseSet names one
("none": no detail). Without any, the plugin generates a stand-in set in the same layout at load.

Each planet loads one set of each family, whichever it picks: the others in this folder cost nothing at run time.

VoxelPlugin.Build.cs stages *.vxnt from this folder with a cooked game.
