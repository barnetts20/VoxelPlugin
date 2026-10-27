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

Put the dune sets here as dunes_<variant>.vxnt (T5c: R sharp crests, G rounded crests, simulated heights with the slip faces
toward +x, the same crest count; B and A aren't read). They make the sand seas where the planet's climate snapshot makes desert,
turned to its sand drift (Design/LandClimate.md). Every planet picks one by its seed unless DuneNoiseSet names one ("none": no
dunes). They ride the hill set's layer: none without a hill set. A set whose R and G aren't simulated heights is left out with a
warning in the log.

Put the sand detail sets here as sanddetail_<variant>.vxnt (T5c: G megaripples, crests across +x; R, B and A aren't read). Every
planet with dunes picks one by its seed unless SandDetailNoiseSet names one ("none": the dunes go without).

Put the glacial sets here as glacial_<variant>.vxnt (T5d: the peaks' layout: R the bedrock carved by ice, G the ice surface over
it, B R blurred, A R's flow). They take the peaks' place where the planet's climate snapshot puts glaciers, now or at the last
glacial maximum (GlacialEraCooling): carved bedrock where the ice has gone, the ice surface where it stays. Every planet picks one
by its seed unless GlacialNoiseSet names one ("none": the peaks everywhere). None without peaks.

Put the ice sheet sets here as icesheet_<variant>.vxnt (T5d2: the peaks' layout: R the buried bed, G the ice surface, B R blurred,
A R's flow; its G only is read, as the ice sheets' swells). The ice sheets themselves come from the climate (IceSheetMaxThickness);
the set only adds their swells. Every planet picks one by its seed unless IceSheetNoiseSet names one ("none": smooth sheets).

Put the polar surface sets here as polar_<variant>.vxnt (T5d3: R crevasses, G sastrugi, B ice-wedge polygons, A sea ice, each a simulated
height; R, G and B are read, A is T5e's). They're laid where the climate makes ice or frozen ground: crevasses on steep glacier ice and near
the ice sheets' margins, sastrugi on cold ice along the wind, polygons on flat bare ground over permafrost. The snow plains' low dunes use the
planet's dune set (its G). Every planet picks one by its seed unless PolarNoiseSet names one ("none": no polar surfaces).

The dune, sand detail, glacial, ice sheet and polar sets are read from their 1024 mip (a 2048 set's mip 1), so a planet's pack stays within its texture rows.

Each planet loads one set of each family, whichever it picks: the others in this folder cost nothing at run time.

VoxelPlugin.Build.cs stages *.vxnt from this folder with a cooked game.
