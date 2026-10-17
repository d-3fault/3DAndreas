#include <open.mp>

#if defined BENCH_COLANDREAS
    #include <colandreas>
    #define BENCH_TAG "[cabench]"
#else
    #include <3DAndreas>
    #define BENCH_TAG "[3dandreas]"
#endif

/**
 * The same queries on ColAndreas 1.5 (BENCH_COLANDREAS) and on 3DAndreas: rays, lines
 * of sight, grounds, every hit along a ray and contact tests, then the height of 120 vertical rays
 * across the map to compare the answers.
 */

main() {}

static stock bool:CastRay(Float:x1, Float:y1, Float:z1, Float:x2, Float:y2, Float:z2, &Float:x, &Float:y, &Float:z) {
    #if defined BENCH_COLANDREAS
        return CA_RayCastLine(x1, y1, z1, x2, y2, z2, x, y, z) != 0;
    #elseif defined BENCH_WATER
        return A3D_RayCast(x1, y1, z1, x2, y2, z2, x, y, z, .flags = A3D_COLLIDE_MAP | A3D_COLLIDE_WATER);
    #else
        return A3D_RayCast(x1, y1, z1, x2, y2, z2, x, y, z, .flags = A3D_COLLIDE_MAP);
    #endif
}

static stock Bench() {
    new
        Float:x,
        Float:y,
        Float:z,
        hits,
        start
    ;

    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        new const
            Float:baseX = 1000.0 + float(i % 100) * 20.0,
            Float:baseY = -2600.0 + float(i / 100) * 20.0
        ;

        hits += _:CastRay(baseX, baseY, 15.0, baseX + 100.0, baseY, 15.0, x, y, z);
    }

    printf(BENCH_TAG " 10000 rays of 100 metres in %i ms, %i hit", GetTickCount() - start, hits);

    hits = 0;
    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        #if defined BENCH_COLANDREAS
            hits += CA_RayCastLine(1000.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 15.0, 1100.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 15.0, x, y, z) == 0;
        #else
            hits += _:A3D_IsLineOfSight(1000.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 15.0, 1100.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 15.0, .flags = A3D_COLLIDE_MAP);
        #endif
    }

    printf(BENCH_TAG " 10000 lines of sight in %i ms, %i clear", GetTickCount() - start, hits);

    hits = 0;
    start = GetTickCount();

    for (new i; i != 10000; ++i) {
        #if defined BENCH_COLANDREAS
            hits += CA_FindZ_For2DCoord(1000.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, z) != 0;
        #else
            hits += _:A3D_FindGround(1000.0 + float(i % 100) * 20.0, -2600.0 + float(i / 100) * 20.0, 700.0, z, .flags = A3D_COLLIDE_MAP, .depth = 1700.0);
        #endif
    }

    printf(BENCH_TAG " 10000 grounds from 700 to -1000 in %i ms, %i found", GetTickCount() - start, hits);

    new
        Float:xs[32],
        Float:ys[32],
        Float:zs[32]
    ;

    hits = 0;
    start = GetTickCount();

    for (new i; i != 1000; ++i) {
        #if defined BENCH_COLANDREAS
            new
                Float:distances[32],
                models[32]
            ;

            new const
                count = CA_RayCastMultiLine(1680.0 + float(i % 10), -2500.0, 15.0, 1680.0 + float(i % 10), -2100.0, 15.0, xs, ys, zs, distances, models)
            ;

            hits += count > 0 ? count : 0;
        #else
            new
                A3D_HIT:types[32],
                ids[32],
                materials[32]
            ;

            hits += A3D_RayCastAll(1680.0 + float(i % 10), -2500.0, 15.0, 1680.0 + float(i % 10), -2100.0, 15.0, xs, ys, zs, types, ids, materials, .flags = A3D_COLLIDE_MAP);
        #endif
    }

    printf(BENCH_TAG " 1000 rays with every hit in %i ms, %i hits", GetTickCount() - start, hits);

    hits = 0;
    start = GetTickCount();

    for (new i; i != 1000; ++i) {
        #if defined BENCH_COLANDREAS
            hits += CA_ContactTest(1220, 1690.0 + float(i % 20), -2500.0 + float(i / 20) * 0.5, 12.9, 0.0, 0.0, 0.0) != 0;
        #else
            hits += _:A3D_IsModelColliding(1220, 1690.0 + float(i % 20), -2500.0 + float(i / 20) * 0.5, 12.9, .flags = A3D_COLLIDE_MAP);
        #endif
    }

    printf(BENCH_TAG " 1000 contact tests of a crate on the tarmac in %i ms, %i touching", GetTickCount() - start, hits);
}

static stock Heights() {
    new
        Float:x,
        Float:y,
        Float:z
    ;

    for (new i; i != 120; ++i) {
        new const
            Float:pointX = -2900.0 + float(i % 12) * 500.0 + float(i) * 7.3,
            Float:pointY = -2900.0 + float(i / 12) * 580.0 + float(i) * 3.1
        ;

        if (!CastRay(pointX, pointY, 700.0, pointX, pointY, -1000.0, x, y, z)) {
            printf(BENCH_TAG " height %i: none", i);

            continue;
        }

        printf(BENCH_TAG " height %i: %.3f", i, z);
    }
}

forward @Bench_Run();
public @Bench_Run() {
    Bench();
    Heights();
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    #if defined BENCH_COLANDREAS
        new const
            start = GetTickCount(),
            loaded = CA_Init()
        ;

        printf(BENCH_TAG " CA_Init %i in %i ms", loaded, GetTickCount() - start);
    #endif

    print(BENCH_TAG " ready");
    SetTimer("@Bench_Run", 4000, false);

    return 1;
}
