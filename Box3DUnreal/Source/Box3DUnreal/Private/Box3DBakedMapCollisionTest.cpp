#include "Misc/AutomationTest.h"
#include "Box3DCollisionData.h"
#include "Box3DStaticGeometry.h"
#include "Box3DConversion.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DBakedMapCollisionTest,
    "RollbackCombat.Box3D.BakedMapCollision",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBox3DBakedMapCollisionTest::RunTest(const FString& Parameters)
{
    UBox3DCollisionData* Data = LoadObject<UBox3DCollisionData>(nullptr,
        TEXT("/Game/RollbackCombat/Maps/BC_Box3D_RollbackExample.BC_Box3D_RollbackExample"));
    if (!TestNotNull(TEXT("Demo collision bake"), Data)) return false;
    TestFalse(TEXT("Geometry digest present"), Data->GeometryDigest.IsEmpty());
    TestEqual(TEXT("Geometry content matches manifest"), Data->ComputeGeometryDigest(), Data->GeometryDigest);
    b3WorldDef Def = b3DefaultWorldDef();
    Def.workerCount = 1;
    const b3WorldId World = b3CreateWorld(&Def);
    TArray<b3MeshData*> Meshes;
    for (const auto& Body : Data->Bodies)
    {
        b3BodyDef BodyDef = b3DefaultBodyDef();
        BodyDef.position = Box3D::ToBox3DPosition(Body.WorldTransform.GetLocation());
        BodyDef.rotation = Box3D::ToBox3DQuat(Body.WorldTransform.GetRotation());
        const b3BodyId Id = b3CreateBody(World, &BodyDef);
        const b3ShapeDef ShapeDef = b3DefaultShapeDef();
        TestTrue(*FString::Printf(TEXT("Imported %s"), *Body.ActorKey),
            Box3D::StaticGeometry::AddBakedShapes(Id, ShapeDef, Body, Meshes));
    }
    auto Cast = [&](FVector Origin, FVector Delta)
    {
        return b3World_CastRayClosest(World, Box3D::ToBox3DPosition(Origin),
            Box3D::ToBox3DVector(Delta), b3DefaultQueryFilter());
    };
    const auto Floor = Cast(FVector(1500,1500,500), FVector(0,0,-1000));
    TestTrue(TEXT("Ordinary floor blocks physics ray"), Floor.hit);
    TestTrue(TEXT("Floor is near its authored height"), FMath::Abs(Floor.fraction - 0.5f) < 0.01f);
    const auto Wall = Cast(FVector(1500,1500,100), FVector(1000,0,0));
    TestTrue(TEXT("Ordinary wall blocks physics ray"), Wall.hit);
    TestTrue(TEXT("Wall is near its authored position"), FMath::Abs(Wall.fraction - 0.475f) < 0.01f);
    const auto Ramp=Cast(FVector(800,500,500),FVector(0,0,-1000));
    AddInfo(FString::Printf(TEXT("Ramp ray hit=%d fraction=%f normal=(%f,%f,%f)"),Ramp.hit,Ramp.fraction,Ramp.normal.x,Ramp.normal.y,Ramp.normal.z));
    TestTrue(TEXT("Offset ramp is at its authored positive Unreal Y"),Ramp.hit && Ramp.fraction<.4f && Ramp.normal.z>.9f && Ramp.normal.z<.99f);
    b3DestroyWorld(World);
    for (b3MeshData* Mesh : Meshes) b3DestroyMesh(Mesh);
    AddInfo(FString::Printf(TEXT("Imported %d static bodies; geometry=%s"), Data->Bodies.Num(), *Data->GeometryDigest));
    return !HasAnyErrors();
}
#endif
