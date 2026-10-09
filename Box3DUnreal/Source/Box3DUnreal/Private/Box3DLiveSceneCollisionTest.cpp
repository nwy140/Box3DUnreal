#include "Misc/AutomationTest.h"
#include "Box3DStaticGeometry.h"
#include "Box3DCollisionData.h"
#include "Box3DSubsystem.h"
#include "Engine/World.h"
#include "Engine/StaticMesh.h"
#include "Components/StaticMeshComponent.h"
#include "Components/InstancedStaticMeshComponent.h"
#include "LandscapeHeightfieldCollisionComponent.h"
#include "Landscape.h"
#include "PhysicsEngine/BodySetup.h"
#include "Chaos/HeightField.h"
#include "box3d_rollback/rollback_shim.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DLiveSceneCollisionTest,"RollbackCombat.Box3D.LiveStaticScene",
    EAutomationTestFlags::EditorContext|EAutomationTestFlags::EngineFilter)
bool FBox3DLiveSceneCollisionTest::RunTest(const FString&)
{
    auto* World=UWorld::CreateWorld(EWorldType::Game,false);
    auto* Cube=LoadObject<UStaticMesh>(nullptr,TEXT("/Engine/BasicShapes/Cube.Cube"));
    auto* Floor=World->SpawnActor<AActor>();
    auto* Mesh=NewObject<UStaticMeshComponent>(Floor); Floor->AddInstanceComponent(Mesh); Floor->SetRootComponent(Mesh);
    Mesh->SetMobility(EComponentMobility::Static); Mesh->SetStaticMesh(Cube); Mesh->SetCollisionProfileName(TEXT("BlockAll"));
    Mesh->SetWorldTransform(FTransform(FQuat::Identity,FVector(0,0,-50),FVector(20,20,1)));
    FBox3DBakedBody Complex;
    TestTrue(TEXT("Cooked complex mesh extracts"),Box3D::StaticGeometry::ExtractStaticCollision(Floor,Box3D::StaticGeometry::ESource::ComplexCollision,false,Complex));
    if(!Complex.Shapes.IsEmpty()) TestEqual(TEXT("Complex as simple uses triangles"),int(Complex.Shapes[0].Kind),int(EBox3DBakedShapeKind::Mesh));
    auto* Authored=NewObject<UStaticMesh>(); Authored->CreateBodySetup();
    auto* Setup=Authored->GetBodySetup(); Setup->CollisionTraceFlag=CTF_UseSimpleAsComplex;
    FKBoxElem Box; Box.X=Box.Y=Box.Z=100; Setup->AggGeom.BoxElems.Add(Box);
    FKSphereElem Round; Round.Radius=25; Setup->AggGeom.SphereElems.Add(Round);
    FKSphylElem Capsule; Capsule.Radius=20; Capsule.Length=60; Setup->AggGeom.SphylElems.Add(Capsule);
    FKConvexElem Convex; Convex.VertexData={FVector(0,0,0),FVector(100,0,0),FVector(0,100,0),FVector(0,0,100)}; Setup->AggGeom.ConvexElems.Add(Convex);
    Mesh->SetStaticMesh(Authored); FBox3DBakedBody Primitives;
    TestTrue(TEXT("Authored simple shapes extract"),Box3D::StaticGeometry::ExtractStaticCollision(Floor,Box3D::StaticGeometry::ESource::Auto,false,Primitives));
    TestEqual(TEXT("Box convex sphere capsule preserved"),Primitives.Shapes.Num(),4);
    if(Primitives.Shapes.Num()==4)
    { TestEqual(TEXT("Sphere stays sphere"),int(Primitives.Shapes[2].Kind),int(EBox3DBakedShapeKind::Sphere)); TestEqual(TEXT("Capsule stays capsule"),int(Primitives.Shapes[3].Kind),int(EBox3DBakedShapeKind::Capsule)); }
    Mesh->SetStaticMesh(Cube);
    auto* Foliage=World->SpawnActor<AActor>();
    auto* Instances=NewObject<UInstancedStaticMeshComponent>(Foliage); Foliage->AddInstanceComponent(Instances); Foliage->SetRootComponent(Instances);
    Instances->SetMobility(EComponentMobility::Static); Instances->SetStaticMesh(Cube); Instances->SetCollisionProfileName(TEXT("BlockAll"));
    Instances->AddInstance(FTransform(FVector(1500,0,0))); Instances->AddInstance(FTransform(FRotator(0,45,0),FVector(1700,0,0),FVector(2,1,1)));
    auto* Terrain=World->SpawnActor<ALandscape>();
    auto* Landscape=NewObject<ULandscapeHeightfieldCollisionComponent>(Terrain); Terrain->AddInstanceComponent(Landscape); Terrain->SetRootComponent(Landscape);
    Landscape->SetMobility(EComponentMobility::Static); Landscape->SetCollisionProfileName(TEXT("BlockAll")); Landscape->SetWorldLocation(FVector(0,2000,0));
    TArray<Chaos::FReal> Heights; Heights.Init(0,9); TArray<uint8> Materials; Materials.Init(0,4); Materials[3]=255;
    Landscape->LocalHeightfieldGeometryRef=new Chaos::FHeightField(MoveTemp(Heights),MoveTemp(Materials),3,3,Chaos::FVec3(100,100,1));
    auto* Scene=NewObject<UBox3DCollisionData>();
    if(!TestTrue(TEXT("Current mesh, foliage and landscape extract"),Box3D::StaticGeometry::ExtractSceneCollision(World,Scene->Bodies))) {World->DestroyWorld(false);return false;}
    TestEqual(TEXT("Three authored actors"),Scene->Bodies.Num(),3);
    int32 InstanceShapes=0,TerrainTriangles=0;
    for(const auto& Body:Scene->Bodies)
    {if(Body.ActorKey.Contains(Foliage->GetName())) InstanceShapes=Body.Shapes.Num();if(Body.ActorKey.Contains(Terrain->GetName())) TerrainTriangles=Body.Shapes[0].Indices.Num()/3;}
    TestEqual(TEXT("Both foliage instances imported"),InstanceShapes,2);
    TestEqual(TEXT("Landscape visibility hole omitted"),TerrainTriangles,6);
    const FString Original=Scene->ComputeGeometryDigest(); Floor->SetActorLocation(FVector(0,0,-100));
    Box3D::StaticGeometry::ExtractSceneCollision(World,Scene->Bodies);
    TestTrue(TEXT("Moving actor changes geometry digest"),Original!=Scene->ComputeGeometryDigest()); Floor->SetActorLocation(FVector(0,0,-50));
    auto* Physics=World->GetSubsystem<UBox3DSubsystem>(); Physics->bExternalSimulationDriven=true; Physics->OnWorldBeginPlay(*World);
    TestTrue(TEXT("Live scene health"),Physics->IsExternalSimulationHealthy());
    TestEqual(TEXT("No manual collider required"),b3World_GetCounters(Physics->GetWorldId()).bodyCount,3);
    b3BodyDef Def=b3DefaultBodyDef(); Def.type=b3_dynamicBody; Def.position=b3Pos{0,0,2};
    auto Ball=b3CreateBody(Physics->GetWorldId(),&Def); b3ShapeDef Shape=b3DefaultShapeDef(); b3Sphere Sphere{{0,0,0},.2f}; b3CreateSphereShape(Ball,&Shape,&Sphere);
    Def.position=b3Pos{.5,-20.5,2}; auto TerrainBall=b3CreateBody(Physics->GetWorldId(),&Def); b3CreateSphereShape(TerrainBall,&Shape,&Sphere);
    TestTrue(TEXT("External ownership starts"),Physics->SetExternalSimulationMode());
    TArray<uint8> Checkpoint; TestTrue(TEXT("Mesh scene snapshot saves"),Physics->CaptureExternalPhysicsSnapshot(Checkpoint));
    TArray<uint64> Hashes;
    auto Hash=[&](){uint64 Value=0;TestTrue(TEXT("Canonical scene supported"),b3r_canonical_world_hash(Physics->GetWorldId(),&Value));return Value;};
    for(int32 Frame=0;Frame<180;++Frame) {TestTrue(TEXT("Step"),Physics->AdvanceExternalPhysicsFrame(Frame));Hashes.Add(Hash());}
    TestTrue(TEXT("Ordinary mesh floor supports body"),FMath::Abs(b3Body_GetPosition(Ball).z-.2)<.03);
    TestTrue(TEXT("Landscape supports body"),FMath::Abs(b3Body_GetPosition(TerrainBall).z-.2)<.03);
    TestTrue(TEXT("Mesh scene snapshot restores"),Physics->RestoreExternalPhysicsSnapshot(Checkpoint));
    for(int32 Frame=0;Frame<180;++Frame) {Physics->AdvanceExternalPhysicsFrame(Frame);TestEqual(TEXT("Static scene replay hash"),Hash(),Hashes[Frame]);}
    World->DestroyWorld(false); return !HasAnyErrors();
}
#endif
