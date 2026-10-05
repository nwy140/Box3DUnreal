#include "Misc/AutomationTest.h"
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
    auto Advance = [&](int Frame)
    {
        // Identical frame-indexed commands must be replayed, including wakeups.
        if (Frame % 17 == 0)
            b3Body_ApplyLinearImpulseToCenter(Bodies[Frame % Bodies.Num()], b3Vec3{0.03f, 0.01f, 0.2f}, true);
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
        for (int Frame = 200; Frame < 300; ++Frame)
        {
            Advance(Frame);
            Expected.Add(b3r_world_hash(World));
        }
        // Overwrite the reusable slot. The independently owned image must survive.
        b3r_save(Context, 0);
        for (int Pass = 0; Pass < 10; ++Pass)
        {
            if (!TestTrue(TEXT("Restore owned checkpoint"), b3r_load_snapshot(Context, Owned.GetData(), Owned.Num()))) break;
            for (int Frame = 200; Frame < 300; ++Frame)
            {
                Advance(Frame);
                if (b3r_world_hash(World) != Expected[Frame - 200])
                {
                    AddError(FString::Printf(TEXT("Replay divergence: pass %d frame %d"), Pass, Frame));
                    break;
                }
            }
        }
    }
    AddInfo(FString::Printf(TEXT("Snapshot bytes=%d; fingerprint=%llu; 1000 replay frames checked (body-state hash only)."), Size, b3r_determinism_fingerprint()));
    // Restored geometry may refer to context-owned registry memory.
    b3DestroyWorld(World);
    b3r_destroy(Context);
    return !HasAnyErrors();
}
#endif
