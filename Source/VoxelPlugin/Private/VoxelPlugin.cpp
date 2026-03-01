// Copyright Epic Games, Inc. All Rights Reserved.

#include "VoxelPlugin.h"

#define LOCTEXT_NAMESPACE "FVoxelPluginModule"

void FVoxelPluginModule::StartupModule()
{

}

void FVoxelPluginModule::ShutdownModule()
{
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FVoxelPluginModule, VoxelPlugin)