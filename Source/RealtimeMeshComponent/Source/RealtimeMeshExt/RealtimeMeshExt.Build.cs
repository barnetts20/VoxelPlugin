// Copyright (c) 2015-2025 TriAxis Games, L.L.C. All Rights Reserved.

using System;
using System.IO;
using UnrealBuildTool;

public class RealtimeMeshExt : ModuleRules
{
    public RealtimeMeshExt(ReadOnlyTargetRules Target) : base(Target)
    {
        bUseUnity = false;
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
#if UE_5_1_OR_LATER
        IncludeOrderVersion = EngineIncludeOrderVersion.Latest;
#endif

        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
                "RealtimeMeshComponent",
                "StructUtils"
            }
        );

        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "GeometryCore",
                "GeometryFramework",
                "RenderCore",
                "RHI",
            }
        );


        PrivateIncludePaths.AddRange(
            new string[]
            {
                //System.IO.Path.Combine(PluginDirectory, "Source", "RealTimeMesh")
            }
        );
        
        PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "Public", "Interface"));

        //PublicDefinitions.Add("REALTIME_MESH_INTERFACE_ROOT=");
    }
}

public class Paths
{
}