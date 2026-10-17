#include <open.mp>
#include <3DAndreas>

/**
 * Checks what the database of 3DAndreas adds: the surface of every hit, the water of the
 * map and floating objects, the size and the physics of models, and the names and flags of the
 * surfaces.
 */

#define GROUND_X 1700.0
#define GROUND_Y -2500.0
#define SEA_X 2000.0
#define SEA_Y -2900.0

#if !defined FLOAT_CHECKS
    #define FLOAT_CHECKS 4
#endif

static
    gBot = INVALID_PLAYER_ID,
    Physic:gFloaters[3],
    gChecks
;

static const Float:gFloaterBuoyancy[] = { 2.0, 4.0, 0.0 };

main() {}

static stock MaterialName(material) {
    new
        name[32]
    ;

    if (!A3D_GetMaterialName(material, name)) {
        name = "unknown";
    }

    return name;
}

static stock PrintSurface(const label[], Float:x, Float:y, Float:fromZ, Float:toZ) {
    new
        Float:hitX,
        Float:hitY,
        Float:hitZ,
        A3D_HIT:type,
        hitid,
        material
    ;

    if (!A3D_RayCast(x, y, fromZ, x, y, toZ, hitX, hitY, hitZ, type, hitid, material)) {
        printf("[database] %s: nothing", label);

        return;
    }

    printf("[database] %s: model %i at %.2f, surface %i %s, flags %08x", label, hitid, hitZ, material, MaterialName(material), _:A3D_GetMaterialFlags(material));
}

static stock CheckSurfaces() {
    PrintSurface("airport tarmac", GROUND_X + 10.0, GROUND_Y, 60.0, 0.0);
    PrintSurface("Grove Street", 2495.0, -1687.0, 40.0, 0.0);
    PrintSurface("Santa Maria beach", 330.0, -1800.0, 30.0, -10.0);
    PrintSurface("Mount Chiliad", -2350.0, -1620.0, 600.0, 0.0);
    PrintSurface("Glen Park", 1970.0, -1180.0, 60.0, 0.0);
    PrintSurface("sea floor", SEA_X, SEA_Y, 0.0, -300.0);
    PrintSurface("roof of the car", GROUND_X - 8.0, GROUND_Y, 30.0, 0.0);

    new
        Float:x,
        Float:y,
        Float:z,
        A3D_HIT:type,
        hitid,
        material
    ;

    new const
        bool:hit = A3D_RayCast(GROUND_X - 3.0, GROUND_Y + 2.0, 13.6, GROUND_X + 3.0, GROUND_Y + 2.0, 13.6, x, y, z, type, hitid, material)
    ;

    printf("[database] across the bot: %i, type %i id %i, surface %i %s", hit, _:type, hitid, material, MaterialName(material));

    new
        Float:xs[8],
        Float:ys[8],
        Float:zs[8],
        A3D_HIT:types[8],
        ids[8],
        materials[8]
    ;

    new const
        count = A3D_RayCastAll(1680.0, -2500.0, 15.0, 1680.0, -2100.0, 15.0, xs, ys, zs, types, ids, materials)
    ;

    printf("[database] everything north of the tarmac: %i hits", count);

    for (new i; i != count; ++i) {
        printf("[database]   %i: model %i at %.2f, surface %i %s", i, ids[i], ys[i], materials[i], MaterialName(materials[i]));
    }

    printf("[database] surface 0 %s, 55 %s, 62 %s, 63 %s, 178 %s, 179 %s, -1 %s", MaterialName(0), MaterialName(55), MaterialName(62), MaterialName(63), MaterialName(178), MaterialName(179), MaterialName(-1));

    new const
        A3D_MATERIAL:fence = A3D_GetMaterialFlags(55)
    ;

    printf("[database] chain fence: shoot through %i, see through %i, sand %i, flags of 179: %i",
        (fence & A3D_MATERIAL_SHOOT_THROUGH) != A3D_MATERIAL:0,
        (fence & A3D_MATERIAL_SEE_THROUGH) != A3D_MATERIAL:0,
        (fence & A3D_MATERIAL_SAND) != A3D_MATERIAL:0,
        _:A3D_GetMaterialFlags(179)
    );
}

static stock PrintLevel(const label[], Float:x, Float:y) {
    new
        Float:level
    ;

    if (!A3D_GetWaterLevel(x, y, level)) {
        printf("[database] water at %s: none", label);

        return;
    }

    printf("[database] water at %s: %.2f", label, level);
}

static stock PrintInWater(const label[], Float:x, Float:y, Float:z) {
    new
        Float:depth
    ;

    new const
        bool:inside = A3D_IsPointInWater(x, y, z, depth)
    ;

    printf("[database] %s in water: %i, depth %.2f", label, inside, depth);
}

static stock CheckWater() {
    PrintLevel("the sea south of the airport", SEA_X, SEA_Y);
    PrintLevel("the tarmac", GROUND_X, GROUND_Y);
    PrintLevel("the indoor pool of Madd Dogg", 1280.0, -774.0);
    PrintLevel("the pool in Vinewood", 1094.0, -672.0);
    PrintLevel("past the edge of the map", 5000.0, 0.0);

    new
        Float:seaFloor
    ;

    A3D_FindGround(SEA_X, SEA_Y, 0.0, seaFloor);
    PrintInWater("3 metres under the sea", SEA_X, SEA_Y, -3.0);
    PrintInWater("under the sea floor", SEA_X, SEA_Y, seaFloor - 5.0);
    PrintInWater("the indoor pool of Madd Dogg", 1280.0, -774.0, 1082.0);
    PrintInWater("the grounds under that pool", 1280.0, -774.0, 100.0);
    PrintInWater("the tarmac", GROUND_X, GROUND_Y, 13.0);
}

static stock CheckModels() {
    new
        Float:minX,
        Float:minY,
        Float:minZ,
        Float:maxX,
        Float:maxY,
        Float:maxZ,
        Float:radius
    ;

    if (A3D_GetModelBoundingBox(2964, minX, minY, minZ, maxX, maxY, maxZ)) {
        printf("[database] box of the pool table: %.3f %.3f %.3f to %.3f %.3f %.3f", minX, minY, minZ, maxX, maxY, maxZ);
    }

    if (A3D_GetModelBoundingSphere(2964, minX, minY, minZ, radius)) {
        printf("[database] sphere of the pool table: %.3f %.3f %.3f radius %.3f", minX, minY, minZ, radius);
    }

    printf("[database] box of model 0: %i, of 20000: %i", A3D_GetModelBoundingBox(0, minX, minY, minZ, maxX, maxY, maxZ), A3D_GetModelBoundingBox(20000, minX, minY, minZ, maxX, maxY, maxZ));

    static const gModels[] = { 1220, 1221, 1230, 1264, 1265, 1328, 1334, 1337, 1598, 1225, 1218, 2964, 3594 };

    for (new i; i != sizeof (gModels); ++i) {
        new
            Float:mass,
            Float:bounce,
            Float:buoyancy,
            Float:damping
        ;

        if (!A3D_GetModelPhysics(gModels[i], mass, bounce, buoyancy, damping)) {
            printf("[database] physics of %i: none", gModels[i]);

            continue;
        }

        printf("[database] physics of %i: mass %.1f, bounce %.2f, buoyancy %.2f, damping %.3f", gModels[i], mass, bounce, buoyancy, damping);
    }
}

static stock Bench() {
    new
        start = GetTickCount(),
        found,
        Float:depth
    ;

    for (new i; i != 10000; ++i) {
        found += _:A3D_IsPointInWater(SEA_X + float(i % 100), SEA_Y + float(i / 100), -1.0, depth);
    }

    printf("[database] bench: 10000 points in the sea in %i ms, %i in water", GetTickCount() - start, found);

    new
        Float:level
    ;

    found = 0;
    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        found += _:A3D_GetWaterLevel(-3000.0 + float(i % 100) * 60.0, -3000.0 + float(i / 100) * 60.0, level);
    }

    printf("[database] bench: 10000 water levels across the map in %i ms, %i over water", GetTickCount() - start, found);
}

forward @Database_Floaters();
public @Database_Floaters() {
    for (new i; i != sizeof (gFloaters); ++i) {
        new
            Float:x,
            Float:y,
            Float:z
        ;

        A3D_GetPosition(gFloaters[i], x, y, z);
        printf("[database] %i s, ball with buoyancy %.1f: centre at %.3f, speed %.4f, moving %i", (gChecks + 1) * 3, gFloaterBuoyancy[i], z, A3D_GetSpeed(gFloaters[i]), A3D_IsMoving(gFloaters[i]));
    }

    new
        active,
        updates,
        Float:milliseconds
    ;

    A3D_GetStats(active, updates, milliseconds);
    printf("[database] %i s, active %i, updates %i, %.1f ms", (gChecks + 1) * 3, active, updates, milliseconds);

    if (++gChecks == FLOAT_CHECKS) {
        SendRconCommand("exit");
    }

    return 1;
}

forward @Database_Run();
public @Database_Run() {
    CheckSurfaces();
    CheckWater();
    CheckModels();
    Bench();

    return 1;
}

public OnGameModeInit() {
    CreateVehicle(411, GROUND_X - 8.0, GROUND_Y, 14.0, 0.0, -1, -1, 60);

    for (new i; i != sizeof (gFloaters); ++i) {
        gFloaters[i] = A3D_CreateSphere(1598, 0.5, SEA_X + float(i) * 3.0, SEA_Y, 3.0, 10.0);
        A3D_SetBuoyancy(gFloaters[i], gFloaterBuoyancy[i]);
    }

    gBot = NPC_Create("DatabaseBot");
    NPC_Spawn(gBot);
    SetTimer("@Database_Floaters", 3000, true);

    return 1;
}

public OnNPCSpawn(npcid) {
    if (npcid != gBot) {
        return 1;
    }

    NPC_SetPos(npcid, GROUND_X, GROUND_Y + 2.0, 13.55);
    SetTimer("@Database_Run", 1500, false);

    return 1;
}
