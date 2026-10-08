// VoxelTerrainStats.h - Profiling group shared by the octree renderer and the brick terrain
// (`stat VoxelTerrain`), so both can be measured on the same flight. Each source file
// declares its own cycle and counter stats in this group.

#pragma once

#include "Stats/Stats.h"

DECLARE_STATS_GROUP(TEXT("VoxelTerrain"), STATGROUP_VoxelTerrain, STATCAT_Advanced);
