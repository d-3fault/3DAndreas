#include <open.mp>
#include <GPS>
#include <3DAndreas>

/**
 * Finds the same routes with the GPS plugin of AmyrAhmady and with 3DAndreas, on the same server:
 * the length each finds and the time of each step, over many calls.
 */

#define RUNS 50
#define OWN_RUNS 2000

static const Float:gPlaces[][3] = {
    { 2495.0, -1687.0, 13.5 },
    { -2026.0, 156.0, 29.0 },
    { 2027.0, 1007.0, 10.8 },
    { -2161.0, -2384.0, 30.6 },
    { 1132.0, -1418.0, 13.5 },
    { 2495.0, -1687.0, 13.5 }
};

static const gPlaceNames[][] = { "Grove Street", "Doherty garage", "Four Dragons", "Angel Pine", "Market", "Grove Street" };

main() {}

forward OnLoaded();
forward OnExit();

static stock Float:Micro(start, runs) {
    return float((GetTickCount() - start) * 1000) / float(runs);
}

static stock Compare(from, to) {
    new
        MapNode:source,
        MapNode:target,
        Path:path,
        Float:pluginLength,
        size,
        tick = GetTickCount()
    ;

    for (new run; run != RUNS; ++run) {
        GetClosestMapNodeToPoint(gPlaces[from][0], gPlaces[from][1], gPlaces[from][2], source);
        GetClosestMapNodeToPoint(gPlaces[to][0], gPlaces[to][1], gPlaces[to][2], target);
    }

    new Float:closest = Micro(tick, RUNS * 2);

    tick = GetTickCount();

    for (new run; run != RUNS; ++run) {
        if (FindPath(source, target, path) == GPS_ERROR_NONE) {
            if (run == RUNS - 1) {
                GetPathLength(path, pluginLength);
                GetPathSize(path, size);
            }

            DestroyPath(path);
        }
    }

    new Float:pluginTime = Micro(tick, RUNS);

    new
        Float:length,
        Float:time,
        Float:fastestLength,
        Route:routeid,
        points
    ;

    tick = GetTickCount();

    for (new run; run != OWN_RUNS; ++run) {
        routeid = A3D_FindRoute(gPlaces[from][0], gPlaces[from][1], gPlaces[from][2], gPlaces[to][0], gPlaces[to][1], gPlaces[to][2], A3D_ROUTE_SHORTEST);

        if (run == OWN_RUNS - 1) {
            length = A3D_GetRouteLength(routeid);
            points = A3D_GetRoutePointCount(routeid);
        }

        A3D_DestroyRoute(routeid);
    }

    new Float:shortestTime = Micro(tick, OWN_RUNS);

    tick = GetTickCount();

    for (new run; run != OWN_RUNS; ++run) {
        routeid = A3D_FindRoute(gPlaces[from][0], gPlaces[from][1], gPlaces[from][2], gPlaces[to][0], gPlaces[to][1], gPlaces[to][2], A3D_ROUTE_FASTEST);

        if (run == OWN_RUNS - 1) {
            fastestLength = A3D_GetRouteLength(routeid);
        }

        A3D_DestroyRoute(routeid);
    }

    new Float:fastestTime = Micro(tick, OWN_RUNS);

    tick = GetTickCount();

    for (new run; run != OWN_RUNS; ++run) {
        A3D_MeasureRoute(gPlaces[from][0], gPlaces[from][1], gPlaces[from][2], gPlaces[to][0], gPlaces[to][1], gPlaces[to][2], length, time, A3D_ROUTE_SHORTEST);
    }

    new Float:measureTime = Micro(tick, OWN_RUNS);

    printf("[compare] %s to %s: plugin %.0f m, %i nodes, path %.0f us, closest node %.0f us | 3DAndreas shortest %.0f m, %i points, %.1f us, measured in %.1f us, fastest %.0f m in %.1f us",
        gPlaceNames[from], gPlaceNames[to], pluginLength, size, pluginTime, closest, length, points, shortestTime, measureTime, fastestLength, fastestTime);
}

public OnGameModeInit() {
    SetTimer("OnLoaded", 5000, false);

    return 1;
}

public OnLoaded() {
    for (new from; from != sizeof (gPlaces) - 1; ++from) {
        Compare(from, from + 1);
    }

    SetTimer("OnExit", 300, false);
}

public OnExit() {
    SendRconCommand("exit");
}
