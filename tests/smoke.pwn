#include <open.mp>
#include <3DAndreas>

/**
 * Drops a sphere, a box and a model shaped object on Los Santos airport and prints where they go.
 */

static
    Physic:gBall,
    Physic:gBox,
    Physic:gCrate,
    gTicks,
    gCollisions
;

main() {}

forward @Smoke_Print();
public @Smoke_Print() {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:bx,
        Float:by,
        Float:bz,
        Float:cx,
        Float:cy,
        Float:cz
    ;

    A3D_GetPosition(gBall, x, y, z);
    A3D_GetPosition(gBox, bx, by, bz);
    A3D_GetPosition(gCrate, cx, cy, cz);
    printf("[smoke] %i: ball %.2f %.2f %.2f moving %i, box %.2f %.2f %.2f moving %i, crate %.2f %.2f %.2f moving %i", gTicks++, x, y, z, A3D_IsMoving(gBall), bx, by, bz, A3D_IsMoving(gBox), cx, cy, cz, A3D_IsMoving(gCrate));

    return 1;
}

forward @Smoke_End();
public @Smoke_End() {
    new
        active,
        updates,
        Float:milliseconds
    ;

    A3D_GetStats(active, updates, milliseconds);
    printf("[smoke] done: %i active, %i updates, %.1f ms, %i collisions, object of ball %i", active, updates, milliseconds, gCollisions, A3D_GetObjectID(gBall));
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    gBall = A3D_CreateSphere(1598, 0.3, 1700.0, -2500.0, 20.0, 1.0);
    gBox = A3D_CreateBox(1220, 0.8, 0.8, 0.8, 1703.0, -2500.0, 20.0, 30.0, 20.0, 10.0, 5.0);
    gCrate = A3D_CreateObject(1220, 1706.0, -2500.0, 20.0, 0.0, 0.0, 45.0, 5.0);
    A3D_SetVelocity(gBall, 3.0, 0.0, 0.0);
    printf("[smoke] created %i %i %i", _:gBall, _:gBox, _:gCrate);
    SetTimer("@Smoke_Print", 500, true);
    SetTimer("@Smoke_End", 8000, false);

    return 1;
}

public A3D_OnCollided(Physic:physicid, A3D_HIT:type, hitid, Float:x, Float:y, Float:z, Float:speed) {
    if (++gCollisions <= 6) {
        printf("[smoke] collided %i with type %i id %i at %.2f %.2f %.2f, speed %.2f", _:physicid, _:type, hitid, x, y, z, speed);
    }

    return 1;
}

public A3D_OnStopped(Physic:physicid, Float:x, Float:y, Float:z) {
    printf("[smoke] stopped %i at %.2f %.2f %.2f", _:physicid, x, y, z);

    return 1;
}
