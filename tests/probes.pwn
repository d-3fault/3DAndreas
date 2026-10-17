#include <open.mp>
#include <3DAndreas>

/**
 * Checks actors in the world, sight in sphere searches, points inside the map, random free spots,
 * predicted throws, offsets and rotations relative to objects and vehicles, vehicle parts, and the
 * screen of a player read from an NPC that aims.
 */

#define GROUND_X 1700.0
#define GROUND_Y -2500.0
#define TARMAC 12.554
#define PED_Z 13.55
#define ACTOR_X 1700.0
#define ACTOR_Y -2480.0
#define CAR_X 1730.0
#define CAR_Y -2480.0
#define VIEW_X 1690.0
#define VIEW_Y -2520.0

static
    gActor = INVALID_ACTOR_ID,
    gCar = INVALID_VEHICLE_ID,
    gViewer = INVALID_PLAYER_ID,
    gTarget = INVALID_PLAYER_ID,
    gSpawned,
    Physic:gBall
;

main() {}

static stock Float:Dot(Float:ax, Float:ay, Float:az, Float:bx, Float:by, Float:bz) {
    return ax * bx + ay * by + az * bz;
}

static stock PrintRay(const label[], Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, A3D_COLLIDE:flags = A3D_COLLIDE_ALL) {
    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid,
        material
    ;

    if (!A3D_RayCast(x1, y1, z1, x2, y2, z2, x, y, z, type, hitid, material, .flags = flags)) {
        printf("[groups] %s: clear", label);

        return;
    }

    printf("[groups] %s: type %i id %i surface %i at %.3f %.3f %.3f", label, _:type, hitid, material, x, y, z);
}

static stock CheckActors() {
    PrintRay("ray across the actor", ACTOR_X - 5.0, ACTOR_Y, PED_Z, ACTOR_X + 2.0, ACTOR_Y, PED_Z);
    PrintRay("same ray without actors", ACTOR_X - 5.0, ACTOR_Y, PED_Z, ACTOR_X + 2.0, ACTOR_Y, PED_Z, A3D_COLLIDE_ALL & ~A3D_COLLIDE_ACTORS);

    new
        found[4]
    ;

    new const
        near = A3D_GetActorsInSphere(ACTOR_X, ACTOR_Y, PED_Z, 3.0, found),
        elsewhere = A3D_GetActorsInSphere(ACTOR_X, ACTOR_Y, PED_Z, 3.0, found, .worldid = 9),
        cone = A3D_GetActorsInCone(ACTOR_X - 10.0, ACTOR_Y, PED_Z, 1.0, 0.0, 0.0, 10.0, 30.0, found),
        behind = A3D_GetActorsInCone(ACTOR_X - 10.0, ACTOR_Y, PED_Z, -1.0, 0.0, 0.0, 10.0, 30.0, found),
        seen = A3D_GetActorsInSphere(ACTOR_X + 10.0, ACTOR_Y, PED_Z, 15.0, found, .sight = true),
        unseen = A3D_GetActorsInSphere(ACTOR_X + 10.0, ACTOR_Y, PED_Z, 15.0, found)
    ;

    printf("[groups] actors within 3 m %i, in world 9 %i, cone east %i, cone west %i, past the wall with sight %i, without %i (actor %i)", near, elsewhere, cone, behind, seen, unseen, gActor);

    new
        vehicles[4]
    ;

    new const
        carSeen = A3D_GetVehiclesInSphere(ACTOR_X + 10.0, CAR_Y, PED_Z, 25.0, vehicles, .sight = true),
        carAny = A3D_GetVehiclesInSphere(ACTOR_X + 10.0, CAR_Y, PED_Z, 25.0, vehicles)
    ;

    printf("[groups] car past the second wall with sight %i, without %i", carSeen, carAny);
}

static stock CheckInside() {
    new
        Float:depth
    ;

    new const
        bool:under = A3D_IsPointInsideMap(GROUND_X + 10.0, GROUND_Y, 10.0, depth)
    ;

    printf("[groups] 2.5 m under the tarmac: %i, depth %.3f", under, depth);
    printf("[groups] above the tarmac: %i", A3D_IsPointInsideMap(GROUND_X + 10.0, GROUND_Y, 14.0));

    new const
        bool:mountain = A3D_IsPointInsideMap(-2350.0, -1620.0, 300.0, depth)
    ;

    printf("[groups] inside Mount Chiliad: %i, depth %.1f", mountain, depth);

    new
        open,
        buried,
        columns,
        Float:groundZ
    ;

    for (new i; i != 400; ++i) {
        new const
            Float:x = 1100.0 + float(i % 20) * 75.0,
            Float:y = -2100.0 + float(i / 20) * 60.0
        ;

        if (!A3D_FindGround(x, y, 400.0, groundZ, .flags = A3D_COLLIDE_MAP, .depth = 500.0)) {
            continue;
        }

        ++columns;
        open += _:!A3D_IsPointInsideMap(x, y, groundZ + 1.0);
        buried += _:A3D_IsPointInsideMap(x, y, groundZ - 3.0);
    }

    printf("[groups] %i columns of Los Santos: 1 m above the ground in the open %i, 3 m under it inside %i", columns, open, buried);

    new
        Float:xs[8],
        Float:ys[8],
        Float:zs[8],
        A3D_HIT:types[8],
        ids[8],
        materials[8],
        covered,
        coveredOpen
    ;

    for (new i; i != 2500; ++i) {
        new const
            Float:x = 1100.0 + float(i % 50) * 30.0,
            Float:y = -2100.0 + float(i / 50) * 24.0,
            count = A3D_RayCastAll(x, y, 400.0, x, y, -50.0, xs, ys, zs, types, ids, materials, .flags = A3D_COLLIDE_MAP)
        ;

        for (new j = 1; j < count; ++j) {
            if (zs[j - 1] - zs[j] < 4.0) {
                continue;
            }

            ++covered;

            if (!A3D_IsPointInsideMap(x, y, zs[j] + 1.0)) {
                ++coveredOpen;
            } else if (covered - coveredOpen <= 3) {
                printf("[groups]   inside under model %i at %.1f %.1f, floor %.2f of model %i, roof %.2f", ids[j - 1], x, y, zs[j], ids[j], zs[j - 1]);
            }

            break;
        }
    }

    printf("[groups] spots with something 4 m or more above the floor: %i, 1 m above that floor in the open %i", covered, coveredOpen);
}

static stock CheckSpots() {
    new
        Float:x,
        Float:y,
        Float:z
    ;

    new bool:found = A3D_FindRandomPoint(GROUND_X, GROUND_Y, PED_Z, 10.0, x, y, z);
    printf("[groups] free spot on the tarmac: %i at %.2f %.2f %.3f, %.1f m away, inside the map %i", found, x, y, z, VectorSize(x - GROUND_X, y - GROUND_Y, 0.0), A3D_IsPointInsideMap(x, y, z + 0.5));

    found = A3D_FindRandomPoint(2000.0, -2900.0, 2.0, 10.0, x, y, z);
    printf("[groups] free spot in the sea: %i", found);

    found = A3D_FindRandomPoint(ACTOR_X, ACTOR_Y, PED_Z, 0.1, x, y, z);
    printf("[groups] free spot under the actor: %i", found);

    new
        hits
    ;

    for (new i; i != 100; ++i) {
        hits += _:A3D_FindRandomPoint(1500.0 + float(i) * 10.0, -1700.0, 20.0, 15.0, x, y, z, .height = 20.0);
    }

    printf("[groups] free spots in 100 tries across Los Santos: %i", hits);
}

static stock CheckThrows() {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:time,
        A3D_HIT:type,
        hitid,
        material
    ;

    new bool:lands = A3D_PredictTrajectory(GROUND_X, GROUND_Y, 30.0, 10.0, 0.0, 0.0, x, y, z, time, type, hitid, material);
    new Float:expected = floatsqroot(2.0 * (30.0 - TARMAC) / 9.81);
    printf("[groups] point thrown east from 30 m: %i at %.3f %.3f %.3f after %.4f s, type %i id %i, expected %.3f after %.4f s", lands, x, y, z, time, _:type, hitid, GROUND_X + 10.0 * expected, expected);

    lands = A3D_PredictTrajectory(GROUND_X, GROUND_Y, 30.0, 10.0, 0.0, 0.0, x, y, z, time, type, hitid, material, .radius = 0.3);
    expected = floatsqroot(2.0 * (30.0 - TARMAC - 0.3) / 9.81);
    printf("[groups] ball of 0.3 m thrown the same way: %i, centre at %.3f %.3f after %.4f s, expected %.3f %.3f after %.4f s", lands, x, z, time, GROUND_X + 10.0 * expected, TARMAC + 0.3, expected);

    lands = A3D_PredictTrajectory(2000.0, -2900.0, 10.0, 5.0, 0.0, 0.0, x, y, z, time, type, hitid, material, .flags = A3D_COLLIDE_ALL | A3D_COLLIDE_WATER);
    printf("[groups] thrown at the sea, water counts: %i, type %i at %.3f after %.3f s", lands, _:type, z, time);

    lands = A3D_PredictTrajectory(2000.0, -2900.0, 10.0, 5.0, 0.0, 0.0, x, y, z, time, type, hitid, material);
    printf("[groups] thrown at the sea, water does not count: %i, type %i surface %i at %.3f after %.3f s", lands, _:type, material, z, time);

    lands = A3D_PredictTrajectory(GROUND_X, GROUND_Y, 13.0, 0.0, 0.0, 50.0, x, y, z, time, .duration = 1.0);
    printf("[groups] thrown up for a second: %i", lands);

    new const
        start = GetTickCount()
    ;

    for (new i; i != 1000; ++i) {
        A3D_PredictTrajectory(1100.0 + float(i % 100) * 15.0, -2100.0 + float(i / 100) * 50.0, 60.0, 8.0, 3.0, 5.0, x, y, z, time);
    }

    printf("[groups] bench: 1000 throws from 60 m in %i ms", GetTickCount() - start);
}

static stock CheckOffsets() {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:ox,
        Float:oy,
        Float:oz,
        Float:rx,
        Float:ry,
        Float:rz
    ;

    A3D_GetOffsetPosition(10.0, 20.0, 30.0, 30.0, 45.0, 60.0, 1.0, 2.0, 3.0, x, y, z);
    A3D_GetPositionOffset(10.0, 20.0, 30.0, 30.0, 45.0, 60.0, x, y, z, ox, oy, oz);
    printf("[groups] offset 1 2 3 on an object turned 30 45 60: %.3f %.3f %.3f, back to %.4f %.4f %.4f", x, y, z, ox, oy, oz);

    A3D_GetLookRotation(0.0, 0.0, 0.0, 10.0, 5.0, 3.0, rx, ry, rz);
    A3D_GetOffsetPosition(0.0, 0.0, 0.0, rx, ry, rz, 0.0, 1.0, 0.0, x, y, z);

    new const
        Float:size = VectorSize(10.0, 5.0, 3.0)
    ;

    printf("[groups] look from 0 0 0 at 10 5 3: rotation %.2f %.2f %.2f, front %.4f %.4f %.4f, agreement %.5f", rx, ry, rz, x, y, z, Dot(x, y, z, 10.0 / size, 5.0 / size, 3.0 / size));

    A3D_GetOffsetRotation(0.0, 0.0, 90.0, 0.0, 0.0, 90.0, rx, ry, rz);
    printf("[groups] 90 degrees turned by 90: %.2f %.2f %.2f", rx, ry, rz);

    new
        Float:cx,
        Float:cy,
        Float:cz
    ;

    GetVehiclePos(gCar, cx, cy, cz);
    A3D_GetVehicleOffsetPosition(gCar, 0.0, 2.0, 0.0, x, y, z);
    printf("[groups] 2 m in front of the car at 45 degrees: %.3f %.3f %.3f, expected %.3f %.3f %.3f", x, y, z, cx - 2.0 * floatsin(45.0, degrees), cy + 2.0 * floatcos(45.0, degrees), cz);

    A3D_GetVehiclePositionOffset(gCar, x, y, z, ox, oy, oz);
    printf("[groups] and back to the car: %.4f %.4f %.4f", ox, oy, oz);

    static const gPartName[][] = {
        "front bumper", "rear bumper", "roof", "driver seat", "passenger seat", "driver door", "passenger door", "rear left door",
        "rear right door", "petrol cap", "front left wheel", "front right wheel", "rear left wheel", "rear right wheel", "middle left wheel", "middle right wheel"
    };

    for (new i; i != sizeof (gPartName); ++i) {
        if (!A3D_GetVehiclePartPosition(gCar, A3D_PART:i, x, y, z)) {
            printf("[groups]   %s: none", gPartName[i]);

            continue;
        }

        A3D_GetVehiclePositionOffset(gCar, x, y, z, ox, oy, oz);
        printf("[groups]   %s: %.3f %.3f %.3f, on the car %.3f %.3f %.3f", gPartName[i], x, y, z, ox, oy, oz);
    }
}

static stock CheckScreen() {
    new
        Float:fx,
        Float:fy,
        Float:fz,
        Float:cx,
        Float:cy,
        Float:cz
    ;

    GetPlayerCameraFrontVector(gViewer, fx, fy, fz);
    GetPlayerCameraPos(gViewer, cx, cy, cz);
    printf("[groups] camera of the viewer at %.2f %.2f %.2f looking %.3f %.3f %.3f, zoom %.1f, aspect %.3f", cx, cy, cz, fx, fy, fz, GetPlayerCameraZoom(gViewer), GetPlayerCameraAspectRatio(gViewer));

    new
        Float:sx,
        Float:sy
    ;

    new bool:shown = A3D_WorldToScreen(gViewer, cx + fx * 20.0, cy + fy * 20.0, cz + fz * 20.0, sx, sy);
    printf("[groups] 20 m along the camera: %i at %.2f %.2f", shown, sx, sy);

    shown = A3D_WorldToScreen(gViewer, cx - fx * 20.0, cy - fy * 20.0, cz - fz * 20.0, sx, sy);
    printf("[groups] 20 m behind the camera: %i", shown);

    new
        Float:dx,
        Float:dy,
        Float:dz
    ;

    A3D_ScreenToWorld(gViewer, 320.0, 224.0, dx, dy, dz);
    printf("[groups] centre of the screen looks along the camera: %.5f", Dot(dx, dy, dz, fx, fy, fz));

    new const
        Float:length = VectorSize(fx, fy, 0.0),
        Float:rightX = fy / length,
        Float:rightY = -fx / length
    ;

    for (new angle = 30; angle <= 38; angle += 2) {
        new const
            Float:px = cx + 20.0 * (fx * floatcos(float(angle), degrees) + rightX * floatsin(float(angle), degrees)),
            Float:py = cy + 20.0 * (fy * floatcos(float(angle), degrees) + rightY * floatsin(float(angle), degrees)),
            Float:pz = cz + 20.0 * fz * floatcos(float(angle), degrees)
        ;

        A3D_WorldToScreen(gViewer, px, py, pz, sx, sy);
        A3D_ScreenToWorld(gViewer, sx, sy, dx, dy, dz);

        new const
            Float:distance = VectorSize(px - cx, py - cy, pz - cz)
        ;

        printf("[groups]   %i degrees right: screen %.1f %.1f, on screen %i, back to the same direction %.5f", angle, sx, sy, A3D_IsPointOnScreen(gViewer, px, py, pz), Dot(dx, dy, dz, (px - cx) / distance, (py - cy) / distance, (pz - cz) / distance));
    }

    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid,
        material
    ;

    new const
        bool:target = A3D_GetScreenTarget(gViewer, 320.0, 224.0, x, y, z, type, hitid, material)
    ;

    printf("[groups] centre of the screen shows: %i, type %i id %i (target %i) at %.2f %.2f %.2f", target, _:type, hitid, gTarget, x, y, z);

    new
        found[8]
    ;

    new const
        players = A3D_GetPlayersOnScreen(gViewer, found),
        firstPlayer = found[0],
        vehicles = A3D_GetVehiclesOnScreen(gViewer, found),
        actors = A3D_GetActorsOnScreen(gViewer, found, .sight = true),
        firstActor = found[0]
    ;

    printf("[groups] on the screen: %i players (first %i), %i vehicles, %i actors seen (first %i, actor %i)", players, firstPlayer, vehicles, actors, firstActor, gActor);
    printf("[groups] screen of a player with no camera: %i", A3D_WorldToScreen(gTarget, GROUND_X, GROUND_Y, PED_Z, sx, sy));
}

forward @Groups_Moved();
public @Groups_Moved() {
    PrintRay("ray across where the actor was", ACTOR_X - 5.0, ACTOR_Y, PED_Z, ACTOR_X + 2.0, ACTOR_Y, PED_Z);
    PrintRay("ray across where it went", ACTOR_X - 5.0, ACTOR_Y + 10.0, PED_Z, ACTOR_X + 2.0, ACTOR_Y + 10.0, PED_Z);
    DestroyActor(gActor);
    SetTimer("@Groups_End", 300, false);

    return 1;
}

forward @Groups_End();
public @Groups_End() {
    PrintRay("ray across it after it was destroyed", ACTOR_X - 5.0, ACTOR_Y + 10.0, PED_Z, ACTOR_X + 2.0, ACTOR_Y + 10.0, PED_Z);
    SendRconCommand("exit");

    return 1;
}

forward @Groups_Run();
public @Groups_Run() {
    CheckActors();
    CheckInside();
    CheckSpots();
    CheckThrows();
    CheckOffsets();
    CheckScreen();
    SetActorPos(gActor, ACTOR_X, ACTOR_Y + 10.0, PED_Z);
    SetTimer("@Groups_Moved", 300, false);

    return 1;
}

forward @Groups_Aim();
public @Groups_Aim() {
    NPC_SetWeapon(gViewer, WEAPON_M4);
    NPC_AimAt(gViewer, VIEW_X, VIEW_Y + 40.0, PED_Z + 0.5);
    gBall = A3D_CreateSphere(-1, 0.2, ACTOR_X, ACTOR_Y, 16.0, 1.0);
    SetTimer("@Groups_Run", 2500, false);

    return 1;
}

public A3D_OnCollided(Physic:physicid, A3D_HIT:type, hitid, Float:x, Float:y, Float:z, Float:speed) {
    if (physicid == gBall && type == A3D_HIT_ACTOR) {
        printf("[groups] ball fell on actor %i at %.2f, %.1f m/s", hitid, z, speed);
    }

    return 1;
}

public OnGameModeInit() {
    gActor = CreateActor(0, ACTOR_X, ACTOR_Y, PED_Z, 90.0);
    gCar = CreateVehicle(411, CAR_X, CAR_Y, 13.6, 45.0, -1, -1, 60);
    A3D_CreateStaticBox(0.4, 6.0, 4.0, ACTOR_X + 5.0, ACTOR_Y, 14.5);
    A3D_CreateStaticBox(0.4, 6.0, 4.0, CAR_X - 10.0, CAR_Y, 14.5);
    gViewer = NPC_Create("ScreenViewer");
    gTarget = NPC_Create("ScreenTarget");
    NPC_Spawn(gViewer);
    NPC_Spawn(gTarget);

    return 1;
}

public OnNPCSpawn(npcid) {
    if (npcid == gViewer) {
        NPC_SetPos(npcid, VIEW_X, VIEW_Y, PED_Z);
    } else {
        NPC_SetPos(npcid, VIEW_X, VIEW_Y + 20.0, PED_Z);
    }

    if (++gSpawned == 2) {
        SetTimer("@Groups_Aim", 1000, false);
    }

    return 1;
}
