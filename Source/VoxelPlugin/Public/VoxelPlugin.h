// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"
#include "Logging/LogMacros.h"

/** The plugin's log category: terrain, pages, mesher, density and collision messages. */
VOXELPLUGIN_API DECLARE_LOG_CATEGORY_EXTERN(LogVoxelPlugin, Log, All);

class FVoxelPluginModule : public IModuleInterface
{
public:

	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
