// Copyright Epic Games, Inc. All Rights Reserved.

#include "VoxelPlugin.h"
#include "VoxelLandBake.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

#define LOCTEXT_NAMESPACE "FVoxelPluginModule"

DEFINE_LOG_CATEGORY(LogVoxelPlugin);

void FVoxelPluginModule::StartupModule()
{
	// Where the authored noise sets are (found here, on the game thread, for the workers that
	// read them).
	VoxelLand::NoiseSetDirectory();

	// The module loads at PostConfigInit (VoxelPlugin.uplugin) so its global shaders register
	// before the engine compiles them. Map /Plugin/VoxelPlugin to the plugin's Shaders folder
	// unless the engine already mapped it.
	const FString VirtualDirectory = TEXT("/Plugin/VoxelPlugin");
	if (AllShaderSourceDirectoryMappings().Contains(VirtualDirectory)) return;

	const TSharedPtr<IPlugin> Plugin = IPluginManager::Get().FindPlugin(TEXT("VoxelPlugin"));
	if (ensureMsgf(Plugin.IsValid(), TEXT("VoxelPlugin: plugin not found; shaders are not mapped.")))
		AddShaderSourceDirectoryMapping(VirtualDirectory, FPaths::Combine(Plugin->GetBaseDir(), TEXT("Shaders")));
}

void FVoxelPluginModule::ShutdownModule()
{
	// The shared noise packs (VoxelLandBake.cpp), if the engine's pre-exit didn't drop them.
	VoxelLand::ReleaseLandNoise();
}

#undef LOCTEXT_NAMESPACE

IMPLEMENT_MODULE(FVoxelPluginModule, VoxelPlugin)
