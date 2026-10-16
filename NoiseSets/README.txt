The authored packed noise sets (.vxnt v3, from the noise generator project; Design/NoiseTiles2D_Handoff.md, Addenda 1 to 3).

Put the peaks sets here as peaks_<variant>.vxnt. They must have Addendum 2's eroded layout (R hard-rock height, G soft-rock height,
B blurred R, A flow accumulation); older sets (the first peaks_a, Addendum 1's cellular layout) are skipped with a warning in the log.

Every planet picks one by its seed, unless its PeakNoiseSet setting names one (the file name without .vxnt). Without any, the plugin
generates a stand-in set at load. The log lists the sets it found ("Noise sets in ...").

Put the hills sets here as hills_<variant>.vxnt (Addendum 3: R dissected height, G rounded height, B blurred R, A flow; the base
network at the tile's lowest height). Every planet picks one by its seed unless HillNoiseSet names one ("none": the procedural
hills). Without any, the procedural hills. hills_standin.vxnt is a stand-in made with Tools/NoiseTileTool/erode_tile.cpp
(BaseMode=2) until hills_a arrives; remove it then, or keep it in the basket.

VoxelPlugin.Build.cs stages *.vxnt from this folder with a cooked game.
