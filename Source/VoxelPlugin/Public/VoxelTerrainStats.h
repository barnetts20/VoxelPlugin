// VoxelTerrainStats.h - Profiling group shared by the terrain actor, the brick terrain, the
// GPU pages and the collision bricks (`stat VoxelTerrain`), so all can be measured on the
// same flight. Each source file declares its own cycle and counter stats in this group.

#pragma once

#include "Stats/Stats.h"

DECLARE_STATS_GROUP(TEXT("VoxelTerrain"), STATGROUP_VoxelTerrain, STATCAT_Advanced);
