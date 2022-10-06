// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class UnrealDemo : ModuleRules
{
	public UnrealDemo(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "HeadMountedDisplay" });


        PrivateIncludePaths.AddRange(
            new string[]
            {
                "UnrealDemo/gphysics/include/collision",
                "UnrealDemo/gphysics/include/dynamic",
                "UnrealDemo/gphysics/include/math",
                "UnrealDemo/gphysics/include/utils",
                "UnrealDemo/gphysics/include/world",



            });
        
    }
}
