#include <open.mp>
#include <3DAndreas>

/**
 * Exercises the zone, rolling friction, planar, gravity, acceleration, collision flags,
 * cylinder and area features of 3DAndreas and prints what happens.
 */

static
    Physic:gZone,
    Physic:gRoller,
    Physic:gPlanar,
    Physic:gFloater,
    Physic:gPushed,
    Physic:gGhost,
    Physic:gPillarBall,
    Physic:gPillar,
    Physic:gWalls[4],
    gStart,
    gTicks
;

main() {}

forward @Features_Print();
public @Features_Print() {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:px,
        Float:py,
        Float:pz,
        Float:fx,
        Float:fy,
        Float:fz
    ;

    A3D_GetPosition(gRoller, x, y, z);
    A3D_GetPosition(gPlanar, px, py, pz);
    A3D_GetPosition(gFloater, fx, fy, fz);
    printf("[features] %i: roller x %.2f speed %.2f angle %.0f, planar z %.2f, floater z %.2f, pushed speed %.2f, ghost exists %i", gTicks++, x, A3D_GetSpeed(gRoller), A3D_GetMoveAngle(gRoller), pz, fz, A3D_GetSpeed(gPushed), A3D_IsValidObject(gGhost));

    return 1;
}

forward @Features_End();
public @Features_End() {
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    gStart = GetTickCount();
    gZone = A3D_CreateZoneSphere(0.3, 1702.0, -2500.0, 12.75);
    gRoller = A3D_CreateSphere(-1, 0.15, 1700.0, -2500.0, 12.71, 1.0);
    A3D_SetRollingFriction(gRoller, 0.5);
    A3D_SetVelocity(gRoller, 3.0, 0.0, 0.0);

    gPlanar = A3D_CreateBox(-1, 0.5, 0.5, 0.5, 1690.0, -2490.0, 20.0, 0.0, 0.0, 0.0, 2.0);
    A3D_SetPlanar(gPlanar);
    A3D_SetVelocity(gPlanar, 1.0, 0.0, 0.0);

    gFloater = A3D_CreateSphere(-1, 0.2, 1690.0, -2495.0, 20.0, 1.0);
    A3D_SetGravity(gFloater, 0.0);

    gPushed = A3D_CreateSphere(-1, 0.2, 1690.0, -2480.0, 25.0, 2.0);
    A3D_SetGravity(gPushed, 0.0);
    A3D_SetAcceleration(gPushed, 1.0, 0.0, 0.0);

    gGhost = A3D_CreateSphere(-1, 0.2, 1710.0, -2500.0, 14.0, 1.0);
    A3D_SetCollisions(gGhost, A3D_COLLIDE_OBJECTS | A3D_COLLIDE_STATIC);

    gPillar = A3D_CreateStaticCylinder(0.3, 2.0, 1715.0, -2500.0, 13.55);
    gPillarBall = A3D_CreateSphere(-1, 0.15, 1713.0, -2500.0, 12.71, 1.0);
    A3D_SetVelocity(gPillarBall, 4.0, 0.0, 0.0);

    printf("[features] area %i, collisions of ghost %i, world of zone %i", A3D_CreateArea(1720.0, -2510.0, 1730.0, -2500.0, 12.55, 13.55, gWalls), _:A3D_GetCollisions(gGhost), A3D_GetWorld(gZone));
    SetTimer("@Features_Print", 1000, true);
    SetTimer("@Features_End", 9000, false);

    return 1;
}

public A3D_OnEntered(Physic:zoneid, Physic:physicid) {
    printf("[features] %i ms: %i entered zone %i", GetTickCount() - gStart, _:physicid, _:zoneid);

    return 1;
}

public A3D_OnLeft(Physic:zoneid, Physic:physicid) {
    printf("[features] %i ms: %i left zone %i", GetTickCount() - gStart, _:physicid, _:zoneid);

    return 1;
}

public A3D_OnStopped(Physic:physicid, Float:x, Float:y, Float:z) {
    printf("[features] %i ms: %i stopped at %.2f %.2f %.2f", GetTickCount() - gStart, _:physicid, x, y, z);

    return 1;
}

public A3D_OnCollided(Physic:physicid, A3D_HIT:type, hitid, Float:x, Float:y, Float:z, Float:speed) {
    if (physicid == gPillarBall && type == A3D_HIT_STATIC) {
        printf("[features] pillar ball hit static %i (pillar %i) at %.2f, %.1f m/s", hitid, _:gPillar, x, speed);
    }

    return 1;
}
