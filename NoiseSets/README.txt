The authored packed noise sets (.vxnt v3, from the noise generator utility; Design/NoiseTiles2D_Handoff.md).

Put the peaks sets here as peaks_<variant>.vxnt (e.g. peaks_a.vxnt). Every planet picks one by its seed,
unless its PeakNoiseSet setting names one (the file name without .vxnt). Without any, the plugin generates
a stand-in set at load. The log lists the sets it found ("Peaks noise sets in ...").

VoxelPlugin.Build.cs stages *.vxnt from this folder with a cooked game.
