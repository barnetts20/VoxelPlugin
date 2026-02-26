// Copyright Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

public class VoxelPlugin : ModuleRules
{
	public VoxelPlugin(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange(
			new string[] {
                Path.Combine(ModuleDirectory, "Shaders/Public")
            }
			);
				
		//string RealTimeMeshLibPath = Path.Combine(ModuleDirectory, "../RealTimeMesh/RealtimeMeshComponent/Binaries/Win64");//D:\Projects\vxl\Plugins\VoxelPlugin\Source\RealTimeMesh\RealtimeMeshComponent\Binaries\Win64
        PrivateIncludePaths.AddRange(
            new string[] {
				Path.Combine(ModuleDirectory, "../RealtimeMeshComponent/Source/RealtimeMeshComponent/Public"),           
				Path.Combine(ModuleDirectory, "../RealtimeMeshComponent/Source/RealtimeMeshComponent/Private"),
				
				//Path.Combine(RealTimeMeshLibPath, "RealtimeMeshComponent.lib"),
				//Path.Combine(RealTimeMeshLibPath, "../RealTimeMesh/RealtimeMeshComponent/Source/RealtimeMeshComponent/Public"),
				//Path.Combine(RealTimeMeshLibPath, "../RealTimeMesh/RealtimeMeshComponent/Source/RealtimeMeshComponent/Private"),
				// Include RealtimeMeshExt
				//Path.Combine(ModuleDirectory, "../RealTimeMesh/RealtimeMeshComponent/Source/RealtimeMeshExt/Public"),
				//Path.Combine(ModuleDirectory, "../RealTimeMesh/RealtimeMeshComponent/Source/RealtimeMeshExt/Private"),
				// Include RealtimeMeshSpatial
				//Path.Combine(ModuleDirectory, "../RealTimeMesh/RealtimeMeshComponent/Source/RealtimeMeshSpatial/Public"),
				//Path.Combine(ModuleDirectory, "../RealTimeMesh/RealtimeMeshComponent/Source/RealtimeMeshSpatial/Private")
    }
            );

		PublicDependencyModuleNames.AddRange(
			new string[]
			{
				"Core",
				"CoreUObject",
				"Engine",
				"RenderCore",
				"InputCore",
				"RHI",                   
				"Renderer",               
				"Projects",
				"RealtimeMeshComponent",
				//"RealtimeMeshExt",
				//"RealtimeMeshSpatial"
			}
			);
			
		
		PrivateDependencyModuleNames.AddRange(
			new string[]
			{
				"CoreUObject",
				"Engine",
				"Slate",
				"SlateCore",
				"RealtimeMeshComponent",
				//"RealtimeMeshExt",
				//"RealtimeMeshSpatial"
				// ... add private dependencies that you statically link with here ...	
			}
			);
		
		
		DynamicallyLoadedModuleNames.AddRange(
			new string[]
			{
				// ... add any modules that your module loads dynamically here ...
			}
			);
	}
}
