#include "Misc/AutomationTest.h"
#include "Box3DSubsystem.h"
#include "Engine/World.h"
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d_rollback/rollback_shim.h"

#if WITH_DEV_AUTOMATION_TESTS
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DFullSnapshotReplayTest,
    "RollbackCombat.Box3D.FullSnapshotReplay",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FBox3DFullSnapshotReplayTest::RunTest(const FString& Parameters)
{
    b3WorldDef WorldDef = b3DefaultWorldDef();
    WorldDef.gravity = b3Vec3{0, 0, -9.8f};
    WorldDef.workerCount = 1;
    const b3WorldId World = b3CreateWorld(&WorldDef);
    B3RollbackCtx* Context = b3r_create(World, 2);
    if (!TestNotNull(TEXT("Snapshot context"), Context))
    {
        b3DestroyWorld(World);
        return false;
    }
    b3BodyDef BodyDef = b3DefaultBodyDef();
    const b3BodyId Floor = b3CreateBody(World, &BodyDef);
    b3ShapeDef ShapeDef = b3DefaultShapeDef();
    const b3BoxHull FloorHull = b3MakeBoxHull(10, 10, 0.5f);
    b3CreateHullShape(Floor, &ShapeDef, &FloorHull.base);
    const b3BoxHull CubeHull = b3MakeBoxHull(0.3f, 0.3f, 0.3f);
    TArray<b3BodyId> Bodies;
    for (int i = 0; i < 8; ++i)
    {
        BodyDef.type = b3_dynamicBody;
        BodyDef.position = b3Pos{0.02 * i, 0, 1.0 + 0.65 * i};
        const b3BodyId Body = b3CreateBody(World, &BodyDef);
        b3CreateHullShape(Body, &ShapeDef, &CubeHull.base);
        Bodies.Add(Body);
    }
    b3DistanceJointDef Joint=b3DefaultDistanceJointDef();Joint.base.bodyIdA=Bodies[6];Joint.base.bodyIdB=Bodies[7];Joint.length=.65f;
    b3CreateDistanceJoint(World,&Joint);
    BodyDef=b3DefaultBodyDef();BodyDef.type=b3_kinematicBody;BodyDef.position=b3Pos{0,0,.65};
    const b3BodyId Platform=b3CreateBody(World,&BodyDef);const b3BoxHull PlatformHull=b3MakeBoxHull(.6f,.6f,.1f);
    b3CreateHullShape(Platform,&ShapeDef,&PlatformHull.base);
    BodyDef=b3DefaultBodyDef();BodyDef.position=b3Pos{2,0,.8};BodyDef.rotation=b3Quat{0,.1305261922f,0,.9914448614f};
    const b3BodyId Ramp=b3CreateBody(World,&BodyDef);const b3BoxHull RampHull=b3MakeBoxHull(.75f,.75f,.1f);
    b3CreateHullShape(Ramp,&ShapeDef,&RampHull.base);
    BodyDef=b3DefaultBodyDef();BodyDef.type=b3_dynamicBody;BodyDef.position=b3Pos{2,0,2};
    const b3BodyId SlopeCube=b3CreateBody(World,&BodyDef);b3CreateHullShape(SlopeCube,&ShapeDef,&CubeHull.base);Bodies.Add(SlopeCube);
    const b3BoxHull WallHull=b3MakeBoxHull(.1f,1.f,1.f);
    for(int Wall=0;Wall<2;++Wall)
    {BodyDef=b3DefaultBodyDef();BodyDef.position=b3Pos{Wall?1.0:3.0,0,1};const b3BodyId WallBody=b3CreateBody(World,&BodyDef);b3CreateHullShape(WallBody,&ShapeDef,&WallHull.base);}
    uint32 CommandSeed=0;
    auto Advance = [&](int Frame)
    {
        // Identical frame-indexed commands must be replayed, including wakeups.
        const uint32 Random=1664525u*(uint32(Frame)+CommandSeed*7919u)+1013904223u;
        if (Frame % 17 == 0)
            b3Body_ApplyLinearImpulseToCenter(Bodies[Random % Bodies.Num()], b3Vec3{(int32((Random>>8)%7)-3)*.01f,(int32((Random>>16)%7)-3)*.01f,.2f}, true);
        b3Body_SetLinearVelocity(Platform,b3Vec3{Frame/30%2?.02f:-.02f,0,Frame/60%2?.05f:-.05f});
        b3World_Step(World, 1.0f / 60.0f, 4);
    };
    for (int Frame = 0; Frame < 200; ++Frame) Advance(Frame);
    const int Size = b3r_save(Context, 0);
    TestTrue(TEXT("Complete image saved"), Size > 0);
    TArray<uint8> Owned;
    Owned.SetNumUninitialized(FMath::Max(Size, 0));
    if (Size > 0)
    {
        TestEqual(TEXT("Caller-owned image copied"), b3r_copy_snapshot(Context, 0, Owned.GetData(), Owned.Num()), Size);
        TestEqual(TEXT("Insufficient output rejected"), b3r_copy_snapshot(Context, 0, Owned.GetData(), Size - 1), -1);
        TArray<uint64> Expected;
        TArray<uint64> CanonicalExpected;
        for (int Frame = 200; Frame < 300; ++Frame)
        {
            Advance(Frame);
            Expected.Add(b3r_world_hash(World));
            uint64 Canonical=0;
            TestTrue(TEXT("Physics fields supported by canonical visitor"), b3r_canonical_world_hash(World, &Canonical));
            CanonicalExpected.Add(Canonical);
        }
        // Overwrite the reusable slot. The independently owned image must survive.
        b3r_save(Context, 0);
        for (int Pass = 0; Pass < 10; ++Pass)
        {
            CommandSeed=uint32(Pass+1);
            if(!TestTrue(TEXT("Restore baseline for new command seed"),b3r_load_snapshot(Context,Owned.GetData(),Owned.Num())))break;
            Expected.Reset();CanonicalExpected.Reset();
            for(int Frame=200;Frame<300;++Frame)
            {Advance(Frame);Expected.Add(b3r_world_hash(World));uint64 Canonical=0;b3r_canonical_world_hash(World,&Canonical);CanonicalExpected.Add(Canonical);}
            if (!TestTrue(TEXT("Restore owned checkpoint"), b3r_load_snapshot(Context, Owned.GetData(), Owned.Num()))) break;
            for (int Frame = 200; Frame < 300; ++Frame)
            {
                Advance(Frame);
                uint64 Canonical=0;
                const bool Supported=b3r_canonical_world_hash(World, &Canonical);
                if (!Supported || Canonical != CanonicalExpected[Frame-200] || b3r_world_hash(World) != Expected[Frame - 200])
                {
                    AddError(FString::Printf(TEXT("Replay divergence: pass %d frame %d"), Pass, Frame));
                    break;
                }
            }
        }
    }
    AddInfo(FString::Printf(TEXT("Snapshot bytes=%d; fingerprint=%llu; 1000 replay frames over ten command seeds, including a moving platform, slope, walls, stacked contacts and a distance joint."), Size, b3r_determinism_fingerprint()));
    // Restored geometry may refer to context-owned registry memory.
    b3DestroyWorld(World);
    b3r_destroy(Context);
    return !HasAnyErrors();
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DExternalClockTest,
    "RollbackCombat.Box3D.ExternalClock",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBox3DExternalClockTest::RunTest(const FString& Parameters)
{
    UWorld* World = UWorld::CreateWorld(EWorldType::Game, false);
    UBox3DSubsystem* Subsystem = World->GetSubsystem<UBox3DSubsystem>();
    Subsystem->bExternalSimulationDriven = true;
    Subsystem->OnWorldBeginPlay(*World);
    TestTrue(TEXT("External world created"), Subsystem->IsWorldValid());
    Subsystem->Tick(1.0f);
    TestEqual(TEXT("Render Tick cannot advance rollback physics"), Subsystem->GetSimulationFrame(), int64(0));
    TestTrue(TEXT("One requested step accepted"), Subsystem->AdvanceExternalPhysicsFrame(0));
    TestEqual(TEXT("Exactly one physics frame"), Subsystem->GetSimulationFrame(), int64(1));
    TestFalse(TEXT("Duplicate frame rejected"), Subsystem->AdvanceExternalPhysicsFrame(0));
    TestFalse(TEXT("Skipped frame rejected"), Subsystem->AdvanceExternalPhysicsFrame(3));
    Subsystem->Tick(1.0f);
    TestEqual(TEXT("No accumulator catchup"), Subsystem->GetSimulationFrame(), int64(1));
    TArray<uint8> Checkpoint;
    TestTrue(TEXT("Composite physics checkpoint captured"), Subsystem->CaptureExternalPhysicsSnapshot(Checkpoint));
    TestTrue(TEXT("Second frame accepted"), Subsystem->AdvanceExternalPhysicsFrame(1));
    uint64 ExpectedHash=0;
    b3r_canonical_world_hash(Subsystem->GetWorldId(), &ExpectedHash);
    TestTrue(TEXT("World and frame restored together"), Subsystem->RestoreExternalPhysicsSnapshot(Checkpoint));
    TestEqual(TEXT("Restored frame"), Subsystem->GetSimulationFrame(), int64(1));
    TestTrue(TEXT("Replay step accepted"), Subsystem->AdvanceExternalPhysicsFrame(1));
    uint64 ReplayHash=0;
    b3r_canonical_world_hash(Subsystem->GetWorldId(), &ReplayHash);
    TestEqual(TEXT("Physics replay equals baseline"), ReplayHash, ExpectedHash);
    if(Checkpoint.Num()>8)
    {
        Checkpoint[8]^=0xff;
        TestFalse(TEXT("Corrupt snapshot rejected"), Subsystem->RestoreExternalPhysicsSnapshot(Checkpoint));
        TestFalse(TEXT("Faulted world cannot advance"), Subsystem->AdvanceExternalPhysicsFrame(2));
    }

    World->DestroyWorld(false);
    return !HasAnyErrors();
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBox3DSnapshotEventTest,
    "RollbackCombat.Box3D.PendingEndEventRestore",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FBox3DSnapshotEventTest::RunTest(const FString& Parameters)
{
    b3WorldDef Def = b3DefaultWorldDef();
    Def.gravity = b3Vec3{0,0,0}; Def.workerCount = 1;
    const b3WorldId World = b3CreateWorld(&Def);
    B3RollbackCtx* Context = b3r_create(World, 2);
    b3BodyDef BodyDef = b3DefaultBodyDef();
    const b3BodyId SensorBody = b3CreateBody(World, &BodyDef);
    b3ShapeDef ShapeDef = b3DefaultShapeDef();
    ShapeDef.isSensor = true; ShapeDef.enableSensorEvents = true;
    const b3Sphere Sphere = {{0,0,0},1};
    b3CreateSphereShape(SensorBody, &ShapeDef, &Sphere);
    BodyDef.type = b3_dynamicBody;
    const b3BodyId Visitor = b3CreateBody(World, &BodyDef);
    ShapeDef.isSensor = false;
    b3CreateSphereShape(Visitor, &ShapeDef, &Sphere);
    b3World_Step(World, 1.0f/60.0f, 4);
    TestTrue(TEXT("Overlap began"), b3World_GetSensorEvents(World).beginCount > 0);
    // Destroying between steps queues a pending end event in the other buffer.
    b3DestroyBody(Visitor);
    TestTrue(TEXT("Checkpoint includes pending end event"), b3r_save(Context, 0) > 0);
    b3World_Step(World, 1.0f/60.0f, 4);
    const int Expected = b3World_GetSensorEvents(World).endCount;
    TestTrue(TEXT("End event delivered"), Expected > 0);
    b3World_Step(World, 1.0f/60.0f, 4);
    b3World_Step(World, 1.0f/60.0f, 4);
    TestEqual(TEXT("Future event buffers emptied"), b3World_GetSensorEvents(World).endCount, 0);
    TestTrue(TEXT("Checkpoint restored"), b3r_load(Context, 0));
    b3World_Step(World, 1.0f/60.0f, 4);
    TestEqual(TEXT("Pending end event replayed exactly"), b3World_GetSensorEvents(World).endCount, Expected);
    b3World_Step(World, 1.0f/60.0f, 4);
    TestEqual(TEXT("End event not repeated"), b3World_GetSensorEvents(World).endCount, 0);
    b3DestroyWorld(World); b3r_destroy(Context);
    return !HasAnyErrors();
}
#endif
