#include <open.mp>
#include <3DAndreas>

/**
 * Checks building removals for every world and for one world, the wake up of what rested on them,
 * model overlaps, extra data on objects, and the rotation and reflection natives.
 */

#define GROUND_X 1700.0
#define GROUND_Y -2500.0
#define TARMAC 5002

static
    Physic:gResting,
    Physic:gWorld5Ball,
    Physic:gWorld0Ball,
    gVehicle
;

main() {}

static stock PrintRay(const label[], worldid) {
    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid
    ;

    if (!A3D_RayCast(GROUND_X + 10.0, GROUND_Y, 60.0, GROUND_X + 10.0, GROUND_Y, -50.0, x, y, z, type, hitid, .worldid = worldid, .flags = A3D_COLLIDE_MAP)) {
        printf("[buildings] %s: clear", label);

        return;
    }

    printf("[buildings] %s: model %i at %.3f", label, hitid, z);
}

static stock PrintBall(const label[], Physic:physicid) {
    new
        Float:x,
        Float:y,
        Float:z
    ;

    A3D_GetPosition(physicid, x, y, z);
    printf("[buildings] %s: centre at %.3f, moving %i", label, z, A3D_IsMoving(physicid));
}

static stock Float:Rotate(Float:w, Float:x, Float:y, Float:z, &Float:outX, &Float:outY, &Float:outZ) {
    new const
        Float:crossX = y * 1.0 - z * 0.0,
        Float:crossY = z * 0.0 - x * 1.0,
        Float:crossZ = x * 0.0 - y * 0.0
    ;

    outX = 0.0 + 2.0 * w * crossX + 2.0 * (y * crossZ - z * crossY);
    outY = 0.0 + 2.0 * w * crossY + 2.0 * (z * crossX - x * crossZ);
    outZ = 1.0 + 2.0 * w * crossZ + 2.0 * (x * crossY - y * crossX);

    return 0.0;
}

static stock CheckRotation(Float:normalX, Float:normalY, Float:normalZ, Float:heading) {
    new
        Float:rx,
        Float:ry,
        Float:rz,
        Float:w,
        Float:x,
        Float:y,
        Float:z,
        Float:upX,
        Float:upY,
        Float:upZ
    ;

    A3D_GetSurfaceRotation(normalX, normalY, normalZ, rx, ry, rz, heading);
    A3D_EulerToQuaternion(rx, ry, rz, w, x, y, z);
    Rotate(w, -x, -y, -z, upX, upY, upZ);

    new const
        Float:length = floatsqroot(normalX * normalX + normalY * normalY + normalZ * normalZ)
    ;

    printf("[buildings] normal %.2f %.2f %.2f heading %.0f: rotation %.2f %.2f %.2f, up axis %.3f %.3f %.3f, off by %.4f",
        normalX, normalY, normalZ, heading,
        rx, ry, rz,
        upX, upY, upZ,
        VectorSize(upX - normalX / length, upY - normalY / length, upZ - normalZ / length)
    );
}

static stock CheckGeometry() {
    new
        Float:w,
        Float:x,
        Float:y,
        Float:z,
        Float:rx,
        Float:ry,
        Float:rz
    ;

    A3D_EulerToQuaternion(10.0, 20.0, 30.0, w, x, y, z);
    A3D_QuaternionToEuler(w, x, y, z, rx, ry, rz);
    printf("[buildings] euler 10 20 30 to %.4f %.4f %.4f %.4f and back to %.3f %.3f %.3f", w, x, y, z, rx, ry, rz);

    GetVehicleRotationQuat(gVehicle, w, x, y, z);
    A3D_QuaternionToEuler(w, x, y, z, rx, ry, rz);
    printf("[buildings] car created at 45 degrees: %.4f %.4f %.4f %.4f, rotation %.2f %.2f %.2f", w, x, y, z, rx, ry, rz);

    CheckRotation(0.0, 0.0, 1.0, 30.0);
    CheckRotation(1.0, 0.0, 0.0, 0.0);
    CheckRotation(0.0, -1.0, 0.0, 90.0);
    CheckRotation(0.3, 0.4, 0.866, 120.0);
    CheckRotation(0.0, 0.0, -1.0, 0.0);

    A3D_GetReflection(1.0, 0.0, -1.0, 0.0, 0.0, 2.0, x, y, z);
    printf("[buildings] reflection of 1 0 -1 off the floor: %.3f %.3f %.3f, of a zero normal: %i", x, y, z, A3D_GetReflection(1.0, 0.0, -1.0, 0.0, 0.0, 0.0, x, y, z));
}

static stock CheckModels() {
    printf("[buildings] crate sunk in the tarmac: %i, above it: %i, sunk with the map left out: %i, model 0: %i",
        A3D_IsModelColliding(1220, GROUND_X + 20.0, GROUND_Y, 12.8),
        A3D_IsModelColliding(1220, GROUND_X + 20.0, GROUND_Y, 20.0),
        A3D_IsModelColliding(1220, GROUND_X + 20.0, GROUND_Y, 12.8, .flags = A3D_COLLIDE_ALL & ~A3D_COLLIDE_MAP),
        A3D_IsModelColliding(0, GROUND_X + 20.0, GROUND_Y, 12.8)
    );
    printf("[buildings] crate on the resting ball, map left out: %i", A3D_IsModelColliding(1220, GROUND_X + 10.0, GROUND_Y, 12.9, .flags = A3D_COLLIDE_OBJECTS));

    new
        A3D_HIT:types[8],
        ids[8]
    ;

    new const
        count = A3D_OverlapModel(1220, GROUND_X + 20.0, GROUND_Y, 12.8, 0.0, 0.0, 45.0, types, ids)
    ;

    printf("[buildings] overlap of a crate sunk in the tarmac: %i, first type %i id %i", count, _:types[0], ids[0]);

    new const
        bool:third = A3D_SetExtra(gResting, 3, 42),
        bool:first = A3D_SetExtra(gResting, 0, -7),
        bool:ninth = A3D_SetExtra(gResting, 8, 1),
        bool:invalid = A3D_SetExtra(Physic:9999, 0, 1)
    ;

    printf("[buildings] extra: set %i %i, read %i %i %i, slot 8 %i read %i, invalid object %i", third, first, A3D_GetExtra(gResting, 3), A3D_GetExtra(gResting, 0), A3D_GetExtra(gResting, 1), ninth, A3D_GetExtra(gResting, 8), invalid);
}

forward @Buildings_Restore();
public @Buildings_Restore() {
    PrintBall("resting ball 2 s after its tarmac left every world", gResting);
    PrintBall("ball of world 0 after that", gWorld0Ball);

    printf("[buildings] restore in world 5: %i", A3D_RestoreBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0, 5));
    PrintRay("ray of world 5 after restoring it", 5);
    printf("[buildings] restore for every world: %i", A3D_RestoreBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0));
    PrintRay("ray of every world after restoring", -1);
    printf("[buildings] restore again: %i", A3D_RestoreBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0));

    new const
        start = GetTickCount()
    ;

    for (new i; i != 100; ++i) {
        A3D_RemoveBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 1.0, 7);
    }

    printf("[buildings] bench: 100 removals in %i ms", GetTickCount() - start);
    SendRconCommand("exit");

    return 1;
}

forward @Buildings_Everywhere();
public @Buildings_Everywhere() {
    PrintBall("ball of world 5 after 2 s", gWorld5Ball);
    PrintBall("ball of world 0 after 2 s", gWorld0Ball);
    PrintBall("resting ball of world 0", gResting);

    printf("[buildings] remove for every world: %i", A3D_RemoveBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0));
    PrintRay("ray of world 0 after that", 0);
    printf("[buildings] remove in world 5 while removed everywhere: %i", A3D_RemoveBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0, 5));
    printf("[buildings] restore in world 0 while removed everywhere: %i", A3D_RestoreBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0, 0));
    PrintBall("resting ball right after", gResting);
    SetTimer("@Buildings_Restore", 2000, false);

    return 1;
}

forward @Buildings_Remove();
public @Buildings_Remove() {
    CheckGeometry();
    CheckModels();
    PrintBall("resting ball before", gResting);
    PrintRay("ray of every world", -1);

    printf("[buildings] remove in world 5: %i", A3D_RemoveBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0, 5));
    printf("[buildings] remove in world 5 again: %i", A3D_RemoveBuilding(TARMAC, GROUND_X, GROUND_Y, 13.0, 500.0, 5));
    PrintRay("ray of world 5", 5);
    PrintRay("ray of world 0", 0);
    PrintRay("ray of every world", -1);
    PrintBall("resting ball of world 0 after the removal in world 5", gResting);

    gWorld5Ball = A3D_CreateSphere(-1, 0.3, GROUND_X + 4.0, GROUND_Y, 14.0, 1.0, 5);
    gWorld0Ball = A3D_CreateSphere(-1, 0.3, GROUND_X + 6.0, GROUND_Y, 14.0, 1.0, 0);

    printf("[buildings] gym %i %i %i %i, lockup %i, nothing there %i",
        A3D_RemoveBuilding(2627, 759.6328, -48.1250, 999.6719, 0.25),
        A3D_RemoveBuilding(2629, 766.3047, -48.3047, 999.6719, 0.25),
        A3D_RemoveBuilding(2631, 756.4063, -47.9219, 999.7266, 0.25),
        A3D_RemoveBuilding(2630, 769.2422, -47.8984, 999.6797, 0.25),
        A3D_RemoveBuilding(1750, 315.6797, 1116.6563, 1082.8750, 0.25),
        A3D_RemoveBuilding(1750, 0.0, 0.0, 0.0, 0.25)
    );

    SetTimer("@Buildings_Everywhere", 2000, false);

    return 1;
}

public OnGameModeInit() {
    gResting = A3D_CreateSphere(-1, 0.3, GROUND_X + 10.0, GROUND_Y, 13.2, 1.0, 0);
    gVehicle = CreateVehicle(411, GROUND_X - 20.0, GROUND_Y, 14.0, 45.0, -1, -1, 60);
    SetTimer("@Buildings_Remove", 4000, false);

    return 1;
}
