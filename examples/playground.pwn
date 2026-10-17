#define FILTERSCRIPT

#include <open.mp>
#include <3DAndreas>

/**
 * A filterscript to try 3DAndreas in game. Every command works where the player stands.
 *
 * - `/ball` throws a beach ball where the camera looks.
 * - `/crate` drops a cardboard box in front of the player, with the physics the game gives it.
 * - `/look` names what the camera looks at and its surface.
 * - `/ground` gives the height and the surface of the ground under the player.
 * - `/inside` tells whether the player stands inside the collision of the map.
 * - `/water` gives the level of the water where the player stands.
 * - `/spot` moves the player to a free spot on the ground nearby.
 * - `/throw` marks where a grenade thrown where the camera looks would land.
 * - `/blast` pushes every physics object within ten metres away.
 * - `/clear` destroys the objects the player created.
 *
 * The playground keeps the last `PLAYGROUND_OBJECTS` objects and destroys the oldest to make room.
 */

#define PLAYGROUND_OBJECTS 64
#define PLAYGROUND_OWNER 0

static
    Physic:gObjects[PLAYGROUND_OBJECTS] = { INVALID_PHYSIC_ID, ... },
    gNextObject
;

static const gHitName[A3D_HIT][] = {
    "the map",
    "a physics object",
    "a static object",
    "a player",
    "a vehicle",
    "the water",
    "an actor"
};

static stock Keep(playerid, Physic:physicid) {
    A3D_DestroyObject(gObjects[gNextObject]);
    A3D_SetExtra(physicid, PLAYGROUND_OWNER, playerid);
    gObjects[gNextObject] = physicid;
    gNextObject = (gNextObject + 1) % PLAYGROUND_OBJECTS;
}

static stock ThrowBall(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:frontX,
        Float:frontY,
        Float:frontZ,
        Physic:ball
    ;

    GetPlayerPos(playerid, x, y, z);
    GetPlayerCameraFrontVector(playerid, frontX, frontY, frontZ);
    ball = A3D_CreateSphere(1598, 0.3, x + frontX, y + frontY, z + 0.6, 0.5, GetPlayerVirtualWorld(playerid));

    if (ball == INVALID_PHYSIC_ID) {
        SendClientMessage(playerid, -1, "The ball could not be created.");

        return;
    }

    A3D_SetBounce(ball, 0.8);
    A3D_SetBuoyancy(ball, 4.0);
    A3D_SetVelocity(ball, frontX * 12.0, frontY * 12.0, frontZ * 12.0 + 2.0);
    Keep(playerid, ball);
}

static stock DropCrate(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:angle,
        Float:mass,
        Float:bounce,
        Float:buoyancy,
        Float:damping,
        Physic:crate
    ;

    GetPlayerPos(playerid, x, y, z);
    GetPlayerFacingAngle(playerid, angle);
    A3D_GetModelPhysics(1220, mass, bounce, buoyancy, damping);
    crate = A3D_CreateObject(1220, x - 2.0 * floatsin(angle, degrees), y + 2.0 * floatcos(angle, degrees), z + 1.5, 0.0, 0.0, angle, mass, GetPlayerVirtualWorld(playerid));

    if (crate == INVALID_PHYSIC_ID) {
        SendClientMessage(playerid, -1, "The crate could not be created.");

        return;
    }

    A3D_SetBounce(crate, bounce);
    A3D_SetBuoyancy(crate, buoyancy);
    A3D_SetDamping(crate, damping, 0.3);
    Keep(playerid, crate);
}

static stock Look(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid,
        material,
        surface[32],
        message[144]
    ;

    if (!A3D_GetPlayerTarget(playerid, x, y, z, type, hitid, material, 300.0, A3D_COLLIDE_ALL | A3D_COLLIDE_WATER)) {
        SendClientMessage(playerid, -1, "Nothing within 300 metres.");

        return;
    }

    if (type == A3D_HIT_WATER) {
        surface = "water";
    } else if (!A3D_GetMaterialName(material, surface)) {
        surface = "no surface";
    }

    format(message, sizeof (message), "You look at %s (id %i) at %.2f, %.2f, %.2f, %s.", gHitName[type], hitid, x, y, z, surface);
    SendClientMessage(playerid, -1, message);
}

static stock Ground(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:groundX,
        Float:groundY,
        Float:groundZ,
        A3D_HIT:type,
        hitid,
        material,
        surface[32],
        message[144]
    ;

    GetPlayerPos(playerid, x, y, z);

    if (!A3D_RayCast(x, y, z, x, y, z - 100.0, groundX, groundY, groundZ, type, hitid, material, GetPlayerVirtualWorld(playerid), A3D_COLLIDE_MAP | A3D_COLLIDE_STATIC)) {
        SendClientMessage(playerid, -1, "No ground within 100 metres.");

        return;
    }

    if (!A3D_GetMaterialName(material, surface)) {
        surface = "no surface";
    }

    format(message, sizeof (message), "The ground is at %.2f, %.2f metres below you, made of %s.", groundZ, z - groundZ, surface);
    SendClientMessage(playerid, -1, message);
}

static stock Inside(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:depth,
        message[144]
    ;

    GetPlayerPos(playerid, x, y, z);

    if (!A3D_IsPointInsideMap(x, y, z, depth, GetPlayerVirtualWorld(playerid))) {
        SendClientMessage(playerid, -1, "You stand in the open.");

        return;
    }

    format(message, sizeof (message), "You are inside the map, %.2f metres under its surface.", depth);
    SendClientMessage(playerid, -1, message);
}

static stock Water(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:level,
        Float:depth,
        message[144]
    ;

    GetPlayerPos(playerid, x, y, z);

    if (!A3D_GetWaterLevel(x, y, level)) {
        SendClientMessage(playerid, -1, "No water covers this place.");

        return;
    }

    if (A3D_IsPointInWater(x, y, z, depth)) {
        format(message, sizeof (message), "The water is at %.2f, you are %.2f metres under it.", level, depth);
    } else {
        format(message, sizeof (message), "The water is at %.2f, you are out of it.", level);
    }

    SendClientMessage(playerid, -1, message);
}

static stock Spot(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:spotX,
        Float:spotY,
        Float:spotZ
    ;

    GetPlayerPos(playerid, x, y, z);

    if (!A3D_FindRandomPoint(x, y, z, 15.0, spotX, spotY, spotZ, 5.0, 0.6, GetPlayerVirtualWorld(playerid))) {
        SendClientMessage(playerid, -1, "No free spot within 15 metres.");

        return;
    }

    SetPlayerPos(playerid, spotX, spotY, spotZ + 1.0);
}

static stock Throw(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:frontX,
        Float:frontY,
        Float:frontZ,
        Float:hitX,
        Float:hitY,
        Float:hitZ,
        Float:time,
        A3D_HIT:type,
        hitid,
        material,
        message[144]
    ;

    GetPlayerPos(playerid, x, y, z);
    GetPlayerCameraFrontVector(playerid, frontX, frontY, frontZ);

    if (!A3D_PredictTrajectory(x, y, z + 0.7, frontX * 15.0, frontY * 15.0, frontZ * 15.0 + 4.0, hitX, hitY, hitZ, time, type, hitid, material, 0.1, .worldid = GetPlayerVirtualWorld(playerid), .flags = A3D_COLLIDE_ALL & ~A3D_COLLIDE_PLAYERS)) {
        SendClientMessage(playerid, -1, "The grenade would not land within ten seconds.");

        return;
    }

    SetPlayerCheckpoint(playerid, hitX, hitY, hitZ, 1.0);
    format(message, sizeof (message), "The grenade would land on %s after %.2f seconds.", gHitName[type], time);
    SendClientMessage(playerid, -1, message);
}

static stock Blast(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        message[144]
    ;

    GetPlayerPos(playerid, x, y, z);
    format(message, sizeof (message), "%i objects pushed.", A3D_CreateExplosion(x, y, z, 10.0, 40.0, GetPlayerVirtualWorld(playerid)));
    SendClientMessage(playerid, -1, message);
}

static stock Clear(playerid) {
    new
        count,
        message[144]
    ;

    for (new i; i != PLAYGROUND_OBJECTS; ++i) {
        if (!A3D_IsValidObject(gObjects[i])) {
            continue;
        }

        if (A3D_GetExtra(gObjects[i], PLAYGROUND_OWNER) != playerid) {
            continue;
        }

        A3D_DestroyObject(gObjects[i]);
        gObjects[i] = INVALID_PHYSIC_ID;
        ++count;
    }

    format(message, sizeof (message), "%i objects destroyed.", count);
    SendClientMessage(playerid, -1, message);
}

public OnFilterScriptInit() {
    print("3DAndreas playground: /ball /crate /look /ground /inside /water /spot /throw /blast /clear");

    return 1;
}

public OnPlayerCommandText(playerid, cmdtext[]) {
    if (strcmp(cmdtext, "/ball", true) == 0) {
        ThrowBall(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/crate", true) == 0) {
        DropCrate(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/look", true) == 0) {
        Look(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/ground", true) == 0) {
        Ground(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/inside", true) == 0) {
        Inside(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/water", true) == 0) {
        Water(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/spot", true) == 0) {
        Spot(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/throw", true) == 0) {
        Throw(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/blast", true) == 0) {
        Blast(playerid);

        return 1;
    }

    if (strcmp(cmdtext, "/clear", true) == 0) {
        Clear(playerid);

        return 1;
    }

    return 0;
}

public A3D_OnCollided(Physic:physicid, A3D_HIT:type, hitid, Float:x, Float:y, Float:z, Float:speed) {
    if (type != A3D_HIT_PLAYER) {
        return 1;
    }

    if (speed < 5.0) {
        return 1;
    }

    new
        ownerid = A3D_GetExtra(physicid, PLAYGROUND_OWNER),
        message[144]
    ;

    if (ownerid == hitid) {
        return 1;
    }

    if (!IsPlayerConnected(ownerid)) {
        return 1;
    }

    format(message, sizeof (message), "Your object hit player %i at %.1f metres per second.", hitid, speed);
    SendClientMessage(ownerid, -1, message);

    return 1;
}
