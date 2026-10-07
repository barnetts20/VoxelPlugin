// Copyright Epic Games, Inc. All Rights Reserved.

#include "VoxelPlugin.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "ShaderCore.h"

#define LOCTEXT_NAMESPACE "FVoxelPluginModule"

void FVoxelPluginModule::StartupModule()
{
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
	// This function may be called during shutdown to clean up your module.  For modules that support dynamic reloading,
	// we call this function before unloading the module.
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FVoxelPluginModule, VoxelPlugin)
