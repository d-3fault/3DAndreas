#include <open.mp>
#include <3DAndreas>

/**
 * Checks vehicles in the physics world, explosions and the target of a player: the box of a car
 * turned with it, a ball dropped on a roof, objects pushed by distance, and a player with no camera.
 */

#define CAR_X 1700.0
#define CAR_Y -2500.0
#define CAR_Z 14.0

static
    gCarNorth,
    gCarEast,
    Physic:gDropped,
    Physic:gBoxes[3],
    gBot = INVALID_PLAYER_ID,
    gHits
;

main() {}

static stock Ray(const label[], Float:startX, Float:startY, Float:endX, Float:endY) {
    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid
    ;

    if (!A3D_RayCast(startX, startY, CAR_Z, endX, endY, CAR_Z, x, y, z, type, hitid, .flags = A3D_COLLIDE_VEHICLES)) {
        printf("[vehicles] %s: clear", label);

        return;
    }

    printf("[vehicles] %s: type %i id %i at %.2f %.2f", label, _:type, hitid, x, y);
}

static stock Down(const label[], Float:x, Float:y) {
    new
        Float:hitX,
        Float:hitY,
        Float:hitZ,
        A3D_HIT:type,
        hitid
    ;

    if (!A3D_RayCast(x, y, CAR_Z + 3.0, x, y, CAR_Z - 3.0, hitX, hitY, hitZ, type, hitid, .flags = A3D_COLLIDE_VEHICLES)) {
        printf("[vehicles] %s: clear", label);

        return;
    }

    printf("[vehicles] %s: vehicle %i, roof at %.2f", label, hitid, hitZ);
}

forward @Vehicles_Check();
public @Vehicles_Check() {
    Ray("north car, along x at y+2.0", CAR_X - 8.0, CAR_Y + 2.0, CAR_X + 8.0, CAR_Y + 2.0);
    Ray("north car, along x at y+0.0", CAR_X - 8.0, CAR_Y, CAR_X + 8.0, CAR_Y);
    Ray("north car, along y at x+1.5", CAR_X + 1.5, CAR_Y - 8.0, CAR_X + 1.5, CAR_Y + 8.0);
    Down("car at 45, 2 m ahead (north west)", CAR_X + 30.0 - 1.414, CAR_Y + 1.414);
    Down("car at 45, 2 m to the mirror (north east)", CAR_X + 30.0 + 1.414, CAR_Y + 1.414);
    Down("car at 45, centre", CAR_X + 30.0, CAR_Y);

    new
        Float:x,
        Float:y,
        Float:z,
        Float:quatW,
        Float:quatX,
        Float:quatY,
        Float:quatZ
    ;

    GetVehicleRotationQuat(gCarEast, quatW, quatX, quatY, quatZ);
    printf("[vehicles] east car quaternion %.3f %.3f %.3f %.3f", quatW, quatX, quatY, quatZ);

    A3D_GetPosition(gDropped, x, y, z);
    printf("[vehicles] ball dropped on the north car rests at z %.2f, %i vehicle impacts", z, gHits);

    for (new i; i != 3; ++i) {
        printf("[vehicles] box %i speed %.2f before the explosion", i, A3D_GetSpeed(gBoxes[i]));
    }

    printf("[vehicles] explosion pushed %i", A3D_CreateExplosion(CAR_X + 60.0, CAR_Y, CAR_Z - 1.0, 5.0, 10.0));

    for (new i; i != 3; ++i) {
        printf("[vehicles] box %i speed %.2f after", i, A3D_GetSpeed(gBoxes[i]));
    }

    new
        A3D_HIT:type,
        hitid
    ;

    printf("[vehicles] target of a bot: %i", A3D_GetPlayerTarget(gBot, x, y, z, type, hitid));
    printf("[vehicles] target of nobody: %i", A3D_GetPlayerTarget(999, x, y, z, type, hitid));

    SetVehiclePos(gCarNorth, CAR_X + 100.0, CAR_Y, CAR_Z);
    SetTimer("@Vehicles_Moved", 300, false);

    return 1;
}

forward @Vehicles_Moved();
public @Vehicles_Moved() {
    Ray("north car after it moved, old place", CAR_X - 8.0, CAR_Y, CAR_X + 8.0, CAR_Y);
    Ray("north car after it moved, new place", CAR_X + 92.0, CAR_Y, CAR_X + 108.0, CAR_Y);

    DestroyVehicle(gCarNorth);
    SetTimer("@Vehicles_Gone", 300, false);

    return 1;
}

forward @Vehicles_Gone();
public @Vehicles_Gone() {
    Ray("north car destroyed", CAR_X + 92.0, CAR_Y, CAR_X + 108.0, CAR_Y);
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    gCarNorth = CreateVehicle(411, CAR_X, CAR_Y, CAR_Z, 0.0, -1, -1, 60);
    gCarEast = CreateVehicle(411, CAR_X + 30.0, CAR_Y, CAR_Z, 45.0, -1, -1, 60);
    gDropped = A3D_CreateSphere(-1, 0.3, CAR_X, CAR_Y, CAR_Z + 4.0, 1.0, 0);

    for (new i; i != 3; ++i) {
        gBoxes[i] = A3D_CreateBox(-1, 0.4, 0.4, 0.4, CAR_X + 60.0 + float(i * i + 1) * 1.0, CAR_Y, CAR_Z - 1.2, 0.0, 0.0, 0.0, 1.0, 0);
    }

    gBot = NPC_Create("VehicleBot");
    NPC_Spawn(gBot);
    SetTimer("@Vehicles_Check", 3000, false);

    return 1;
}

public OnNPCSpawn(npcid) {
    NPC_SetPos(npcid, CAR_X - 20.0, CAR_Y, 13.55);

    return 1;
}

public A3D_OnCollided(Physic:physicid, A3D_HIT:type, hitid, Float:x, Float:y, Float:z, Float:speed) {
    if (type == A3D_HIT_VEHICLE) {
        ++gHits;
        printf("[vehicles] %i hit vehicle %i at %.2f %.2f %.2f, %.1f m/s", _:physicid, hitid, x, y, z, speed);
    }

    return 1;
}
