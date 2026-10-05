using UnrealBuildTool;

// Box3DRollback owns the single source-built Box3D implementation and its
// private snapshot machinery. Never link a second prebuilt Box3D here.
public class Box3DUnreal : ModuleRules
{
    public Box3DUnreal(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        FPSemantics = FPSemanticsMode.Precise;
        PublicDependencyModuleNames.AddRange(new[] { "Core", "Box3DRollback" });
        PrivateDependencyModuleNames.AddRange(new[] { "CoreUObject", "Engine", "PhysicsCore", "InputCore", "Landscape" });
    }
}

// Foundation collision and full snapshot replay automation live in this module.
