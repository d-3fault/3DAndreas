#include <open.mp>
#include <3DAndreas>

/**
 * Checks the navigation mesh of 3DAndreas: the nearest walk point, random walk points, straight walks
 * across a house and along a street, walks found on the mesh alone and walks that join the mesh to
 * the paths, and the cost of each native over many calls.
 */

#define RUNS 1000

main() {}

forward OnLoaded();
forward OnExit();

static stock PrintWalk(const label[], Route:routeid) {
    if (routeid == INVALID_ROUTE_ID) {
        printf("[walks] %s: no route", label);

        return;
    }

    new
        Float:x,
        Float:y,
        Float:z,
        Float:endX,
        Float:endY,
        Float:endZ
    ;

    A3D_GetRoutePoint(routeid, 0, x, y, z);
    A3D_GetRoutePoint(routeid, A3D_GetRoutePointCount(routeid) - 1, endX, endY, endZ);
    printf("[walks] %s: %.0f m, %.0f s, %i points from %.1f %.1f %.1f to %.1f %.1f %.1f", label, A3D_GetRouteLength(routeid), A3D_GetRouteTime(routeid), A3D_GetRoutePointCount(routeid), x, y, z, endX, endY, endZ);
}

public OnGameModeInit() {
    SetTimer("OnLoaded", 5000, false);

    return 1;
}

public OnLoaded() {
    printf("[walks] mesh loaded: %i", A3D_IsWalkMeshLoaded());

    new
        Float:x,
        Float:y,
        Float:z
    ;

    printf("[walks] nearest walk point to Grove Street: %i", A3D_GetNearestWalkPoint(2495.0, -1687.0, 13.5, x, y, z));
    printf("[walks]   at %.2f %.2f %.2f", x, y, z);
    printf("[walks] nearest walk point high in the sky: %i", A3D_GetNearestWalkPoint(2495.0, -1687.0, 300.0, x, y, z));

    for (new index; index != 5; ++index) {
        if (A3D_GetRandomWalkPoint(2495.0, -1687.0, 13.5, 60.0, x, y, z)) {
            printf("[walks] random walk point %i: %.1f %.1f %.1f, %.1f m away", index, x, y, z, VectorSize(x - 2495.0, y + 1687.0, 0.0));
        }
    }

    printf("[walks] straight across the houses north of Grove Street blocked: %i", A3D_WalkRayCast(2440.0, -1680.0, 13.5, 2440.0, -1620.0, 13.5, x, y, z));
    printf("[walks]   stopped at %.1f %.1f %.1f", x, y, z);
    printf("[walks] straight along the street blocked: %i", A3D_WalkRayCast(2380.0, -1660.0, 13.4, 2460.0, -1660.0, 13.4));

    new Route:routeid = A3D_FindRoute(2470.0, -1700.0, 13.5, 2380.0, -1660.0, 13.4, A3D_ROUTE_WALK);

    PrintWalk("short walk in Grove Street", routeid);
    A3D_DestroyRoute(routeid);

    routeid = A3D_FindRoute(1132.0, -1418.0, 13.5, 2495.0, -1687.0, 13.5, A3D_ROUTE_WALK);
    PrintWalk("long walk from the Market to Grove Street", routeid);
    A3D_DestroyRoute(routeid);

    routeid = A3D_FindRoute(1185.0, -1335.0, 13.6, 2034.0, -1405.0, 17.2, A3D_ROUTE_WALK);
    PrintWalk("walk from All Saints to County General", routeid);
    A3D_DestroyRoute(routeid);

    new
        tick = GetTickCount()
    ;

    for (new run; run != RUNS; ++run) {
        A3D_GetNearestWalkPoint(2400.0 + float(run % 100), -1700.0, 13.5, x, y, z);
    }

    printf("[walks] bench: nearest walk point %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS));
    tick = GetTickCount();

    for (new run; run != RUNS; ++run) {
        A3D_GetRandomWalkPoint(2495.0, -1687.0, 13.5, 60.0, x, y, z);
    }

    printf("[walks] bench: random walk point %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS));
    tick = GetTickCount();

    for (new run; run != RUNS; ++run) {
        A3D_WalkRayCast(2380.0, -1660.0, 13.4, 2460.0, -1660.0, 13.4);
    }

    printf("[walks] bench: walk ray cast of 80 m %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS));
    tick = GetTickCount();

    for (new run; run != RUNS / 10; ++run) {
        routeid = A3D_FindRoute(2470.0, -1700.0, 13.5, 2380.0, -1660.0, 13.4, A3D_ROUTE_WALK);
        A3D_DestroyRoute(routeid);
    }

    printf("[walks] bench: short walk on the mesh %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS / 10));
    tick = GetTickCount();

    for (new run; run != RUNS / 10; ++run) {
        routeid = A3D_FindRoute(1132.0, -1418.0, 13.5, 2495.0, -1687.0, 13.5, A3D_ROUTE_WALK);
        A3D_DestroyRoute(routeid);
    }

    printf("[walks] bench: long walk on the paths and the mesh %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS / 10));
    SetTimer("OnExit", 300, false);
}

public OnExit() {
    SendRconCommand("exit");
}
