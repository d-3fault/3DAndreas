#include <open.mp>
#include <3DAndreas>

/**
 * Four stress scenarios that measure the time 3DAndreas spends and the object updates it sends.
 *
 * - Scenario 1, pool: `PHYBENCH_COUNT` tables of 16 balls, a shot on every table every six seconds.
 * - Scenario 2, throws: waves of `PHYBENCH_COUNT` items thrown every ten seconds, destroyed when they stop.
 * - Scenario 3, crowd: `PHYBENCH_COUNT` boxes in a walled arena, fifty of them kicked every two seconds.
 * - Scenario 4, idle: the boxes of scenario 3 left at rest.
 *
 * The component counts its own time and the object updates it sends. The numbers printed at the
 * end cover the seconds after `BENCH_WARMUP`.
 */

#if !defined PHYBENCH_SCENARIO
    #define PHYBENCH_SCENARIO (1)
#endif

#if !defined PHYBENCH_SECONDS
    #define PHYBENCH_SECONDS (60)
#endif

#if !defined PHYBENCH_BOTS
    #define PHYBENCH_BOTS (0)
#endif

#if !defined PHYBENCH_COUNT
    #define PHYBENCH_COUNT (1)
#endif

#define BENCH_X                         (1700.0)
#define BENCH_Y                         (-2500.0)
#define BENCH_GROUND                    (12.55)
#define BENCH_RANGE                     (30)
#define BENCH_WARMUP                    (15)

#define POOL_BALLS                      (16)
#define POOL_BALL_SIZE                  (0.05)
#define POOL_LENGTH                     (2.2)
#define POOL_WIDTH                      (1.1)

static
    gBotsCreated,
    gShots,
    gKicks,
    gThrown,
    gStopped,
    gWarmUpdates,
    Float:gWarmMilliseconds
;

#if PHYBENCH_SCENARIO == 1
    static
        Physic:gPoolBall[PHYBENCH_COUNT][POOL_BALLS]
    ;
#endif

#if PHYBENCH_SCENARIO == 3 || PHYBENCH_SCENARIO == 4
    static
        Physic:gBox[PHYBENCH_COUNT]
    ;
#endif

main() {}

static stock RandomPointInternal(&Float:x, &Float:y) {
    x = BENCH_X + float(random(BENCH_RANGE * 2 - 3) - BENCH_RANGE + 2);
    y = BENCH_Y + float(random(BENCH_RANGE * 2 - 3) - BENCH_RANGE + 2);
}

static stock WalkInternal(npcid) {
    new
        Float:x,
        Float:y
    ;

    RandomPointInternal(x, y);
    NPC_Move(npcid, x, y, BENCH_GROUND + 1.0, NPC_MOVE_TYPE_WALK);
}

#if PHYBENCH_SCENARIO == 1
    static stock MakeTableInternal(table) {
        new const
            Float:x = BENCH_X + float(table % 4) * 4.0,
            Float:y = BENCH_Y + float(table / 4) * 3.0,
            Float:z = BENCH_GROUND + 0.9
        ;

        A3D_CreateStaticBox(POOL_LENGTH + 0.4, POOL_WIDTH + 0.4, 0.1, x, y, z - POOL_BALL_SIZE - 0.05);
        A3D_CreateStaticBox(POOL_LENGTH + 0.4, 0.2, 0.3, x, y - POOL_WIDTH / 2.0 - 0.1, z);
        A3D_CreateStaticBox(POOL_LENGTH + 0.4, 0.2, 0.3, x, y + POOL_WIDTH / 2.0 + 0.1, z);
        A3D_CreateStaticBox(0.2, POOL_WIDTH + 0.4, 0.3, x - POOL_LENGTH / 2.0 - 0.1, y, z);
        A3D_CreateStaticBox(0.2, POOL_WIDTH + 0.4, 0.3, x + POOL_LENGTH / 2.0 + 0.1, y, z);

        new
            ball
        ;

        gPoolBall[table][ball++] = A3D_CreateSphere(3003, POOL_BALL_SIZE, x - POOL_LENGTH / 4.0, y, z, 0.17);

        for (new row; row != 5; ++row) {
            for (new i; i <= row; ++i) {
                gPoolBall[table][ball++] = A3D_CreateSphere(3003, POOL_BALL_SIZE, x + POOL_LENGTH / 4.0 + float(row) * POOL_BALL_SIZE * 1.8, y + (float(i) - float(row) / 2.0) * POOL_BALL_SIZE * 2.1, z, 0.17);
            }
        }

        for (new i; i != POOL_BALLS; ++i) {
            A3D_SetDamping(gPoolBall[table][i], 0.3, 0.3);
            A3D_SetBounce(gPoolBall[table][i], 0.8);
        }
    }

    forward @Bench_Shot();
    public @Bench_Shot() {
        for (new i; i != PHYBENCH_COUNT; ++i) {
            new const
                Float:angle = float(random(360))
            ;

            A3D_SetVelocity(gPoolBall[i][0], 6.0 * floatcos(angle, degrees), 6.0 * floatsin(angle, degrees), 0.0);
            ++gShots;
        }

        return 1;
    }
#endif

#if PHYBENCH_SCENARIO == 2
    forward @Bench_Wave();
    public @Bench_Wave() {
        printf("[physbench] wave at %i s: %i items still moving", GetTickCount() / 1000, gThrown - gStopped);

        for (new i; i != PHYBENCH_COUNT; ++i) {
            new
                Float:x,
                Float:y
            ;

            RandomPointInternal(x, y);

            new const
                Float:angle = float(random(360)),
                Float:speed = 3.0 + float(random(50)) / 10.0
            ;

            new const
                Physic:item = A3D_CreateBox(1575, 0.3, 0.2, 0.2, x, y, BENCH_GROUND + 1.0, 0.0, 0.0, angle)
            ;

            if (item == INVALID_PHYSIC_ID) {
                break;
            }

            A3D_SetVelocity(item, speed * floatcos(angle, degrees), speed * floatsin(angle, degrees), 2.0 + float(random(30)) / 10.0);
            ++gThrown;
        }

        return 1;
    }

    public A3D_OnStopped(Physic:physicid, Float:x, Float:y, Float:z) {
        A3D_DestroyObject(physicid);
        ++gStopped;

        return 1;
    }
#endif

#if PHYBENCH_SCENARIO == 3 || PHYBENCH_SCENARIO == 4
    static stock MakeBoxesInternal() {
        new const
            Float:size = float(BENCH_RANGE * 2)
        ;

        A3D_CreateStaticBox(size, 1.0, 4.0, BENCH_X, BENCH_Y - float(BENCH_RANGE), BENCH_GROUND + 2.0);
        A3D_CreateStaticBox(size, 1.0, 4.0, BENCH_X, BENCH_Y + float(BENCH_RANGE), BENCH_GROUND + 2.0);
        A3D_CreateStaticBox(1.0, size, 4.0, BENCH_X - float(BENCH_RANGE), BENCH_Y, BENCH_GROUND + 2.0);
        A3D_CreateStaticBox(1.0, size, 4.0, BENCH_X + float(BENCH_RANGE), BENCH_Y, BENCH_GROUND + 2.0);

        for (new i; i != PHYBENCH_COUNT; ++i) {
            new
                Float:x,
                Float:y
            ;

            RandomPointInternal(x, y);
            gBox[i] = A3D_CreateBox(1220, 0.8, 0.8, 0.8, x, y, BENCH_GROUND + 0.4, 0.0, 0.0, float(random(360)), 2.0);
        }
    }

    forward @Bench_Kick();
    public @Bench_Kick() {
        for (new i; i != 50; ++i) {
            new const
                Float:angle = float(random(360)),
                Float:speed = 2.0 + float(random(40)) / 10.0
            ;

            A3D_SetVelocity(gBox[random(PHYBENCH_COUNT)], speed * floatcos(angle, degrees), speed * floatsin(angle, degrees), float(random(40)) / 10.0);
            ++gKicks;
        }

        return 1;
    }
#endif

forward @Bench_Join();
public @Bench_Join() {
    for (new i; i != 2 && gBotsCreated != PHYBENCH_BOTS; ++i) {
        new
            name[MAX_PLAYER_NAME]
        ;

        format(name, sizeof (name), "PhyBot_%i", gBotsCreated++);

        new const
            npcid = NPC_Create(name)
        ;

        if (npcid != INVALID_NPC_ID) {
            NPC_Spawn(npcid);
        }
    }

    return 1;
}

forward @Bench_Warm();
public @Bench_Warm() {
    new
        active
    ;

    A3D_GetStats(active, gWarmUpdates, gWarmMilliseconds);

    return 1;
}

forward @Bench_End();
public @Bench_End() {
    new
        active,
        updates,
        Float:milliseconds
    ;

    A3D_GetStats(active, updates, milliseconds);

    new const
        Float:seconds = float(PHYBENCH_SECONDS - BENCH_WARMUP)
    ;

    printf("[physbench] scenario %i, count %i, bots %i: %i shots, %i kicks, %i thrown, %i stopped, %i active", PHYBENCH_SCENARIO, PHYBENCH_COUNT, PHYBENCH_BOTS, gShots, gKicks, gThrown, gStopped, active);
    printf("[physbench] component: %.2f ms per second, %.0f object updates per second", (milliseconds - gWarmMilliseconds) / seconds, float(updates - gWarmUpdates) / seconds);

    #if PHYBENCH_SCENARIO == 3 || PHYBENCH_SCENARIO == 4
        for (new i; i != PHYBENCH_COUNT; ++i) {
            if (!A3D_IsMoving(gBox[i])) {
                continue;
            }

            new
                Float:x,
                Float:y,
                Float:z,
                Float:vx,
                Float:vy,
                Float:vz
            ;

            A3D_GetPosition(gBox[i], x, y, z);
            A3D_GetVelocity(gBox[i], vx, vy, vz);
            printf("[physbench] awake box %i at %.2f %.2f %.2f, velocity %.2f %.2f %.2f", i, x, y, z, vx, vy, vz);
        }
    #endif
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    #if PHYBENCH_SCENARIO == 1
        for (new i; i != PHYBENCH_COUNT; ++i) {
            MakeTableInternal(i);
        }

        SetTimer("@Bench_Shot", 6000, true);
    #endif

    #if PHYBENCH_SCENARIO == 2
        SetTimer("@Bench_Wave", 10000, true);
    #endif

    #if PHYBENCH_SCENARIO == 3 || PHYBENCH_SCENARIO == 4
        MakeBoxesInternal();
    #endif

    #if PHYBENCH_SCENARIO == 3
        SetTimer("@Bench_Kick", 2000, true);
    #endif

    #if PHYBENCH_BOTS != 0
        SetTimer("@Bench_Join", 250, true);
    #endif

    SetTimer("@Bench_Warm", BENCH_WARMUP * 1000, false);
    SetTimer("@Bench_End", PHYBENCH_SECONDS * 1000, false);
    printf("[physbench] scenario %i started", PHYBENCH_SCENARIO);

    return 1;
}

public OnNPCSpawn(npcid) {
    new
        Float:x,
        Float:y
    ;

    RandomPointInternal(x, y);
    NPC_SetPos(npcid, x, y, BENCH_GROUND + 1.0);
    WalkInternal(npcid);

    return 1;
}

public OnNPCFinishMove(npcid) {
    WalkInternal(npcid);

    return 1;
}
