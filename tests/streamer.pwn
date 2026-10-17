#include <open.mp>
#include <streamer>
#include <3DAndreas>

/**
 * Throws a box shown by a streamer object and prints where the body and the object are.
 */

static
    Physic:gItem,
    gObject,
    gTicks
;

main() {}

forward @Attach_Print();
public @Attach_Print() {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:ox,
        Float:oy,
        Float:oz
    ;

    A3D_GetPosition(gItem, x, y, z);
    GetDynamicObjectPos(gObject, ox, oy, oz);
    printf("[attach] %i: body %.2f %.2f %.2f, object %.2f %.2f %.2f, apart %.2f, object moving %i, global object %i", gTicks++, x, y, z, ox, oy, oz, VectorSize(x - ox, y - oy, z - oz), IsDynamicObjectMoving(gObject), A3D_GetObjectID(gItem));

    return 1;
}

forward @Attach_End();
public @Attach_End() {
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    gObject = CreateDynamicObject(1575, 1700.0, -2500.0, 14.0, 0.0, 0.0, 0.0);
    gItem = A3D_CreateBox(1575, 0.3, 0.2, 0.2, 1700.0, -2500.0, 14.0);
    A3D_AttachDynamicObject(gItem, gObject);
    A3D_SetVelocity(gItem, 4.0, 1.0, 4.0);
    SetTimer("@Attach_Print", 250, true);
    SetTimer("@Attach_End", 5000, false);

    return 1;
}

public A3D_OnStopped(Physic:physicid, Float:x, Float:y, Float:z) {
    new
        Float:ox,
        Float:oy,
        Float:oz
    ;

    GetDynamicObjectPos(gObject, ox, oy, oz);
    printf("[attach] stopped at %.2f %.2f %.2f, object at %.2f %.2f %.2f", x, y, z, ox, oy, oz);

    return 1;
}
