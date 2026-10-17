#include <open.mp>
#include <3DAndreas>

/**
 * Checks the world queries of 3DAndreas on the Los Santos airport: rays, the ground,
 * line of sight, sweeps, overlaps, physics objects in two worlds, bots and vehicles in shapes, and
 * the cost of each native over many calls.
 */

#define GROUND_X 1700.0
#define GROUND_Y -2500.0

static
    gBots[3] = { INVALID_PLAYER_ID, ... },
    gSpawned,
    Physic:gWallWorld5,
    Physic:gWallWorld0,
    Physic:gBall,
    gVehicles[2]
;

main() {}

static stock PrintHit(const label[], bool:hit, Float:x, Float:y, Float:z, A3D_HIT:type, hitid) {
    if (!hit) {
        printf("[queries] %s: clear", label);

        return;
    }

    printf("[queries] %s: hit type %i id %i at %.2f %.2f %.2f", label, _:type, hitid, x, y, z);
}

static stock CheckRays() {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:nx,
        Float:ny,
        Float:nz,
        A3D_HIT:type,
        hitid,
        bool:hit
    ;

    hit = A3D_FindGround(GROUND_X + 10.0, GROUND_Y, 60.0, z);
    printf("[queries] ground under the tarmac: %i at %.3f", hit, z);

    hit = A3D_FindGround(2495.0, -1687.0, 40.0, z);
    printf("[queries] ground in Grove Street: %i at %.3f", hit, z);

    hit = A3D_FindGround(GROUND_X + 10.0, GROUND_Y, 60.0, z, .depth = 10.0);
    printf("[queries] ground within 10 metres of 60: %i", hit);

    hit = A3D_RayCastNormal(GROUND_X + 10.0, GROUND_Y, 60.0, GROUND_X + 10.0, GROUND_Y, 0.0, x, y, z, nx, ny, nz, type, hitid);
    printf("[queries] normal of the tarmac: %i, %.3f %.3f %.3f, model %i", hit, nx, ny, nz, hitid);

    hit = A3D_RayCastNormal(-2350.0, -1620.0, 600.0, -2350.0, -1620.0, 0.0, x, y, z, nx, ny, nz, type, hitid);
    printf("[queries] normal on Mount Chiliad: %i at %.1f, %.3f %.3f %.3f", hit, z, nx, ny, nz);

    hit = A3D_RayCast(1680.0, -2500.0, 15.0, 1680.0, -2100.0, 15.0, x, y, z, type, hitid);
    PrintHit("north from the tarmac", hit, x, y, z, type, hitid);

    hit = A3D_RayCast(1680.0, -2100.0, 15.0, 1680.0, -2500.0, 15.0, x, y, z, type, hitid);
    PrintHit("south towards the tarmac", hit, x, y, z, type, hitid);

    printf("[queries] line of sight across the airport: %i, along the tarmac: %i",
        A3D_IsLineOfSight(1680.0, -2500.0, 15.0, 1680.0, -2100.0, 15.0),
        A3D_IsLineOfSight(1650.0, -2500.0, 15.0, 1750.0, -2500.0, 15.0)
    );

    new
        Float:xs[16],
        Float:ys[16],
        Float:zs[16],
        A3D_HIT:types[16],
        ids[16],
        materials[16]
    ;

    new const
        count = A3D_RayCastAll(1680.0, -2500.0, 15.0, 1680.0, -2100.0, 15.0, xs, ys, zs, types, ids, materials)
    ;

    printf("[queries] everything north of the tarmac: %i hits", count);

    for (new i; i != count && i != 6; ++i) {
        printf("[queries]   %i: model %i at %.2f %.2f %.2f", i, ids[i], xs[i], ys[i], zs[i]);
    }

    hit = A3D_SweepSphere(0.5, GROUND_X + 10.0, GROUND_Y, 60.0, GROUND_X + 10.0, GROUND_Y, 0.0, x, y, z, type, hitid);
    printf("[queries] sphere of 0.5 dropped on the tarmac: %i, centre at %.3f", hit, z);

    hit = A3D_SweepSphere(0.5, GROUND_X, GROUND_Y, 60.0, GROUND_X, GROUND_Y, 50.0, x, y, z, type, hitid);
    printf("[queries] sphere stopping in the air: %i, centre at %.3f", hit, z);
}

static stock CheckObjects() {
    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid,
        bool:hit
    ;

    hit = A3D_RayCast(GROUND_X - 1.0, GROUND_Y - 6.0, 14.5, GROUND_X - 1.0, GROUND_Y + 6.0, 14.5, x, y, z, type, hitid, .worldid = 0);
    PrintHit("through the wall of world 5, from world 0", hit, x, y, z, type, hitid);

    hit = A3D_RayCast(GROUND_X - 1.0, GROUND_Y - 6.0, 14.5, GROUND_X - 1.0, GROUND_Y + 6.0, 14.5, x, y, z, type, hitid, .worldid = 5);
    PrintHit("through the wall of world 5, from world 5", hit, x, y, z, type, hitid);

    hit = A3D_RayCast(GROUND_X - 1.0, GROUND_Y - 6.0, 14.5, GROUND_X - 1.0, GROUND_Y + 6.0, 14.5, x, y, z, type, hitid, .flags = A3D_COLLIDE_MAP);
    PrintHit("through the wall, map only", hit, x, y, z, type, hitid);

    hit = A3D_RayCast(GROUND_X + 20.0, GROUND_Y - 6.0, 13.05, GROUND_X + 20.0, GROUND_Y + 6.0, 13.05, x, y, z, type, hitid);
    PrintHit("through the ball", hit, x, y, z, type, hitid);

    hit = A3D_RayCast(GROUND_X + 20.0, GROUND_Y - 6.0, 13.05, GROUND_X + 20.0, GROUND_Y + 6.0, 13.05, x, y, z, type, hitid, .flags = A3D_COLLIDE_ALL & ~A3D_COLLIDE_OBJECTS);
    PrintHit("through the ball, objects left out", hit, x, y, z, type, hitid);

    new
        A3D_HIT:types[16],
        ids[16]
    ;

    new
        count = A3D_OverlapSphere(GROUND_X, GROUND_Y, 14.5, 3.0, types, ids)
    ;

    printf("[queries] overlap of 3 metres at the wall: %i", count);

    for (new i; i != count; ++i) {
        printf("[queries]   type %i id %i", _:types[i], ids[i]);
    }

    count = A3D_OverlapBox(4.0, 4.0, 2.0, GROUND_X + 20.0, GROUND_Y, 14.0, 0.0, 0.0, 45.0, types, ids, .flags = A3D_COLLIDE_OBJECTS | A3D_COLLIDE_STATIC);
    printf("[queries] overlap of a box around the ball: %i, first type %i id %i (ball %i)", count, _:types[0], ids[0], _:gBall);
}

static stock CheckPlayers() {
    new
        found[8],
        count
    ;

    count = A3D_GetPlayersInSphere(GROUND_X, GROUND_Y, 14.0, 6.0, found);
    printf("[queries] players within 6 metres: %i (%i %i), bots %i %i %i", count, found[0], found[1], gBots[0], gBots[1], gBots[2]);

    count = A3D_GetPlayersInCone(GROUND_X, GROUND_Y - 30.0, 14.5, 0.0, 1.0, 0.0, 10.0, 80.0, found);
    printf("[queries] players in a cone of 10 degrees to the north: %i (%i %i)", count, found[0], found[1]);

    count = A3D_GetPlayersInCone(GROUND_X, GROUND_Y - 30.0, 14.5, 0.0, 1.0, 0.0, 10.0, 80.0, found, .sight = true);
    printf("[queries] the same cone with sight: %i (%i)", count, found[0]);

    count = A3D_GetPlayersInCone(GROUND_X, GROUND_Y - 30.0, 14.5, 0.0, 1.0, 0.0, 10.0, 80.0, found, .worldid = 7);
    printf("[queries] the same cone in world 7: %i", count);

    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid
    ;

    new const
        bool:hit = A3D_RayCast(GROUND_X - 3.0, GROUND_Y + 20.0, 13.6, GROUND_X + 3.0, GROUND_Y + 20.0, 13.6, x, y, z, type, hitid)
    ;

    PrintHit("across the third bot", hit, x, y, z, type, hitid);

    count = A3D_GetVehiclesInSphere(GROUND_X, GROUND_Y, 14.0, 30.0, found);
    printf("[queries] vehicles within 30 metres: %i (%i %i), created %i %i", count, found[0], found[1], gVehicles[0], gVehicles[1]);
}

static stock Bench() {
    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid,
        hits,
        start,
        found[8],
        A3D_HIT:types[16],
        ids[16]
    ;

    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        new const
            Float:baseX = 1000.0 + float(i % 100) * 20.0,
            Float:baseY = -2600.0 + float(i / 100) * 20.0
        ;

        hits += _:A3D_RayCast(baseX, baseY, 15.0, baseX + 100.0, baseY, 15.0, x, y, z, type, hitid);
    }

    printf("[queries] bench: 10000 rays of 100 metres in %i ms, %i hit", GetTickCount() - start, hits);

    hits = 0;
    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        hits += _:A3D_IsLineOfSight(1000.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 15.0, 1100.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 15.0);
    }

    printf("[queries] bench: 10000 lines of sight in %i ms, %i clear", GetTickCount() - start, hits);

    hits = 0;
    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        hits += _:A3D_FindGround(1000.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 100.0, z);
    }

    printf("[queries] bench: 10000 grounds in %i ms, %i found", GetTickCount() - start, hits);

    hits = 0;
    start = GetTickCount();

    for (new i; i != 1000; ++i) {
        hits += A3D_OverlapSphere(1000.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 14.0, 5.0, types, ids);
    }

    printf("[queries] bench: 1000 overlaps of 5 metres in %i ms, %i things", GetTickCount() - start, hits);

    hits = 0;
    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        hits += A3D_GetPlayersInCone(GROUND_X, GROUND_Y - 30.0, 14.5, 0.0, 1.0, 0.0, 10.0, 80.0, found, .sight = true);
    }

    printf("[queries] bench: 10000 cones with sight in %i ms, %i players", GetTickCount() - start, hits);
}

forward @Queries_Run();
public @Queries_Run() {
    CheckRays();
    CheckObjects();
    CheckPlayers();
    Bench();
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    gWallWorld5 = A3D_CreateStaticBox(4.0, 0.4, 3.0, GROUND_X, GROUND_Y, 15.0, 0.0, 0.0, 0.0, 5);
    gWallWorld0 = A3D_CreateStaticBox(4.0, 0.4, 3.0, GROUND_X, GROUND_Y + 10.0, 15.0, 0.0, 0.0, 0.0, 0);
    gBall = A3D_CreateSphere(-1, 0.5, GROUND_X + 20.0, GROUND_Y, 14.1, 1.0, 0);
    gVehicles[0] = CreateVehicle(411, GROUND_X + 8.0, GROUND_Y, 14.0, 0.0, -1, -1, 60);
    gVehicles[1] = CreateVehicle(411, GROUND_X - 15.0, GROUND_Y, 14.0, 0.0, -1, -1, 60);
    printf("[queries] walls %i %i, ball %i", _:gWallWorld5, _:gWallWorld0, _:gBall);

    for (new i; i != 3; ++i) {
        new
            name[MAX_PLAYER_NAME]
        ;

        format(name, sizeof (name), "QueryBot_%i", i);
        gBots[i] = NPC_Create(name);
        NPC_Spawn(gBots[i]);
    }

    return 1;
}

public OnNPCSpawn(npcid) {
    if (npcid == gBots[0]) {
        NPC_SetPos(npcid, GROUND_X, GROUND_Y + 2.0, 13.55);
    } else if (npcid == gBots[1]) {
        NPC_SetPos(npcid, GROUND_X + 4.0, GROUND_Y + 2.0, 13.55);
    } else {
        NPC_SetPos(npcid, GROUND_X, GROUND_Y + 20.0, 13.55);
    }

    if (++gSpawned == 3) {
        SetTimer("@Queries_Run", 1500, false);
    }

    return 1;
}
