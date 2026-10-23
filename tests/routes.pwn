#include <open.mp>
#include <3DAndreas>

/**
 * Checks the routes of 3DAndreas: a route found while the hierarchies are contracted, routes of every
 * type, their points, turns and progress, measures, the nearest point by route, the nearest path
 * point, ids that do not exist, and the cost of each native over many calls.
 */

#define RUNS 1000

static const
    gTypeNames[][] = { "fastest", "shortest", "any way", "traffic", "boat", "walk" },
    gTurnNames[][] = { "none", "slight left", "left", "sharp left", "slight right", "right", "sharp right", "back" }
;

static const Float:gPlaces[][3] = {
    { 2495.0, -1687.0, 13.5 },
    { -2026.0, 156.0, 29.0 },
    { 2027.0, 1007.0, 10.8 },
    { -2161.0, -2384.0, 30.6 },
    { 1132.0, -1418.0, 13.5 }
};

static const gPlaceNames[][] = { "Grove Street", "Doherty garage", "Four Dragons", "Angel Pine", "Market" };

main() {}

forward OnContracted();
forward OnExit();

static stock PrintRoute(const label[], Route:routeid) {
    if (routeid == INVALID_ROUTE_ID) {
        printf("[routes] %s: no route", label);

        return;
    }

    printf("[routes] %s: %.0f m, %.0f s, %i points, %i turns", label, A3D_GetRouteLength(routeid), A3D_GetRouteTime(routeid), A3D_GetRoutePointCount(routeid), A3D_GetRouteTurnCount(routeid));
}

static stock CheckTypes() {
    new
        label[64],
        Route:routeid
    ;

    for (new type; type != sizeof (gTypeNames); ++type) {
        if (A3D_ROUTE:type == A3D_ROUTE_BOAT) {
            continue;
        }

        for (new from; from != sizeof (gPlaces) - 1; ++from) {
            routeid = A3D_FindRoute(gPlaces[from][0], gPlaces[from][1], gPlaces[from][2], gPlaces[from + 1][0], gPlaces[from + 1][1], gPlaces[from + 1][2], A3D_ROUTE:type);
            format(label, sizeof (label), "%s %s to %s", gTypeNames[type], gPlaceNames[from], gPlaceNames[from + 1]);
            PrintRoute(label, routeid);
            A3D_DestroyRoute(routeid);
        }
    }

    new
        Float:x,
        Float:y,
        Float:z,
        Float:endX,
        Float:endY,
        Float:endZ
    ;

    if (!A3D_GetNearestPathPoint(2800.0, -2400.0, 0.0, x, y, z, .type = A3D_ROUTE_BOAT, .reach = 2000.0) || !A3D_GetNearestPathPoint(-1500.0, 1500.0, 0.0, endX, endY, endZ, .type = A3D_ROUTE_BOAT, .reach = 2000.0)) {
        print("[routes] boat: no lane near");

        return;
    }

    routeid = A3D_FindRoute(x, y, z, endX, endY, endZ, A3D_ROUTE_BOAT);
    format(label, sizeof (label), "boat %.0f %.0f to %.0f %.0f", x, y, endX, endY);
    PrintRoute(label, routeid);
    A3D_DestroyRoute(routeid);
}

static stock CheckRoute() {
    new Route:routeid = A3D_FindRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], gPlaces[1][0], gPlaces[1][1], gPlaces[1][2], A3D_ROUTE_FASTEST, 90.0);

    PrintRoute("Grove Street to Doherty garage facing west", routeid);

    new
        count = A3D_GetRoutePointCount(routeid),
        Float:x,
        Float:y,
        Float:z
    ;

    A3D_GetRoutePoint(routeid, 0, x, y, z);
    printf("[routes] first point %.1f %.1f %.1f", x, y, z);
    A3D_GetRoutePoint(routeid, count - 1, x, y, z);
    printf("[routes] last point %.1f %.1f %.1f, point %i read: %i", x, y, z, count, A3D_GetRoutePoint(routeid, count, x, y, z));

    new
        Float:xs[64],
        Float:ys[64],
        Float:zs[64]
    ;

    printf("[routes] copied %i points from 0, %i from %i", A3D_GetRoutePoints(routeid, xs, ys, zs), A3D_GetRoutePoints(routeid, xs, ys, zs, count - 10), count - 10);

    new
        Float:distance,
        A3D_TURN:turn
    ;

    for (new index, turns = A3D_GetRouteTurnCount(routeid); index != turns; ++index) {
        turn = A3D_GetRouteTurn(routeid, index, x, y, z, distance);
        printf("[routes] turn %i: %s at %.0f m, %.0f %.0f", index, gTurnNames[_:turn], distance, x, y);
    }

    new
        Float:remaining,
        Float:away,
        next,
        Float:turnDistance
    ;

    A3D_GetRoutePoint(routeid, count / 2, x, y, z);
    A3D_GetRouteProgress(routeid, x, y, z, remaining, away, next, turn, turnDistance);
    printf("[routes] at point %i: %.0f m left, %.2f m away, next point %i, then %s in %.0f m", count / 2, remaining, away, next, gTurnNames[_:turn], turnDistance);
    A3D_GetRouteProgress(routeid, x + 50.0, y, z, remaining, away, next, turn, turnDistance);
    printf("[routes] 50 m east of it: %.0f m left, %.1f m away", remaining, away);
    A3D_GetRouteProgress(routeid, gPlaces[1][0], gPlaces[1][1], gPlaces[1][2], remaining, away, next, turn, turnDistance);
    printf("[routes] at the end: %.0f m left, %.1f m away, next %s", remaining, away, gTurnNames[_:turn]);

    new
        Float:length,
        Float:time
    ;

    A3D_MeasureRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], gPlaces[1][0], gPlaces[1][1], gPlaces[1][2], length, time, A3D_ROUTE_FASTEST, 90.0);
    printf("[routes] measured %.1f m %.1f s against found %.1f m %.1f s", length, time, A3D_GetRouteLength(routeid), A3D_GetRouteTime(routeid));

    printf("[routes] shown to player 0 who is not connected: %i", A3D_ShowRouteForPlayer(0, routeid));

    new bool:destroyed = A3D_DestroyRoute(routeid);

    printf("[routes] destroyed: %i, valid after: %i, length after: %.1f", destroyed, A3D_IsValidRoute(routeid), A3D_GetRouteLength(routeid));
    printf("[routes] destroyed again: %i", A3D_DestroyRoute(routeid));
    printf("[routes] invalid id: %i points, turn %s, progress %i", A3D_GetRoutePointCount(INVALID_ROUTE_ID), gTurnNames[_:A3D_GetRouteTurn(INVALID_ROUTE_ID, 0, x, y, z)], A3D_GetRouteProgress(INVALID_ROUTE_ID, 0.0, 0.0, 0.0, remaining));
}

static stock CheckNearest() {
    static const Float:hospitals[][3] = {
        { 1172.0, -1323.0, 15.4 },
        { 2034.0, -1405.0, 17.2 },
        { -2655.0, 639.0, 14.5 },
        { 1607.0, 1818.0, 10.8 }
    };

    new
        Float:x[sizeof (hospitals)],
        Float:y[sizeof (hospitals)],
        Float:z[sizeof (hospitals)]
    ;

    for (new index; index != sizeof (hospitals); ++index) {
        x[index] = hospitals[index][0];
        y[index] = hospitals[index][1];
        z[index] = hospitals[index][2];
    }

    printf("[routes] nearest hospital by road from Grove Street: %i, from the Doherty garage: %i, from Four Dragons: %i",
        A3D_FindNearestByRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], x, y, z),
        A3D_FindNearestByRoute(gPlaces[1][0], gPlaces[1][1], gPlaces[1][2], x, y, z),
        A3D_FindNearestByRoute(gPlaces[2][0], gPlaces[2][1], gPlaces[2][2], x, y, z)
    );

    new
        Float:pathX,
        Float:pathY,
        Float:pathZ,
        Float:angle
    ;

    for (new type; type != sizeof (gTypeNames); ++type) {
        if (A3D_GetNearestPathPoint(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], pathX, pathY, pathZ, angle, A3D_ROUTE:type, 500.0)) {
            printf("[routes] nearest %s path to Grove Street: %.1f %.1f %.1f facing %.0f", gTypeNames[type], pathX, pathY, pathZ, angle);
        } else {
            printf("[routes] nearest %s path to Grove Street: none within 500 m", gTypeNames[type]);
        }
    }

    printf("[routes] far from any road: %i", A3D_GetNearestPathPoint(0.0, 0.0, 1000.0, pathX, pathY, pathZ, angle, .reach = 50.0));
}

static stock Bench() {
    new
        tick = GetTickCount(),
        Route:routeid
    ;

    for (new run; run != RUNS; ++run) {
        routeid = A3D_FindRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], gPlaces[1][0], gPlaces[1][1], gPlaces[1][2]);
        A3D_DestroyRoute(routeid);
    }

    printf("[routes] bench: find and destroy Grove Street to Doherty garage %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS));

    new
        Float:length,
        Float:time
    ;

    tick = GetTickCount();

    for (new run; run != RUNS; ++run) {
        A3D_MeasureRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], gPlaces[1][0], gPlaces[1][1], gPlaces[1][2], length, time);
    }

    printf("[routes] bench: measure %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS));

    routeid = A3D_FindRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], gPlaces[1][0], gPlaces[1][1], gPlaces[1][2]);

    new
        Float:remaining,
        Float:away
    ;

    tick = GetTickCount();

    for (new run; run != RUNS; ++run) {
        A3D_GetRouteProgress(routeid, 1000.0 + float(run), -1500.0, 20.0, remaining, away);
    }

    printf("[routes] bench: progress on a route of %i points %.1f us", A3D_GetRoutePointCount(routeid), float((GetTickCount() - tick) * 1000) / float(RUNS));
    A3D_DestroyRoute(routeid);

    new
        Float:pathX,
        Float:pathY,
        Float:pathZ
    ;

    tick = GetTickCount();

    for (new run; run != RUNS; ++run) {
        A3D_GetNearestPathPoint(1000.0 + float(run), -1500.0, 20.0, pathX, pathY, pathZ);
    }

    printf("[routes] bench: nearest path point %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS));

    new
        Float:x[50],
        Float:y[50],
        Float:z[50]
    ;

    for (new index; index != sizeof (x); ++index) {
        x[index] = -2500.0 + float(index) * 100.0;
        y[index] = -1700.0 + float(index % 7) * 400.0;
        z[index] = 20.0;
    }

    tick = GetTickCount();

    for (new run; run != RUNS / 10; ++run) {
        A3D_FindNearestByRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], x, y, z);
    }

    printf("[routes] bench: nearest of 50 points by route %.1f us", float((GetTickCount() - tick) * 1000) / float(RUNS / 10));
}

public OnGameModeInit() {
    new
        tick = GetTickCount(),
        Route:routeid = A3D_FindRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], gPlaces[1][0], gPlaces[1][1], gPlaces[1][2])
    ;

    printf("[routes] first route in %i ms", GetTickCount() - tick);
    PrintRoute("first route, while the hierarchies are contracted", routeid);
    A3D_DestroyRoute(routeid);

    tick = GetTickCount();

    for (new run; run != 20; ++run) {
        routeid = A3D_FindRoute(gPlaces[0][0], gPlaces[0][1], gPlaces[0][2], gPlaces[1][0], gPlaces[1][1], gPlaces[1][2]);
        A3D_DestroyRoute(routeid);
    }

    printf("[routes] while contracted: %.0f us a route", float((GetTickCount() - tick) * 1000) / 20.0);
    SetTimer("OnContracted", 4000, false);

    return 1;
}

public OnContracted() {
    CheckTypes();
    CheckRoute();
    CheckNearest();
    Bench();
    SetTimer("OnExit", 500, false);
}

public OnExit() {
    SendRconCommand("exit");
}

public A3D_OnRouteArrived(playerid, Route:routeid) {
    return 1;
}

public A3D_OnRouteLeft(playerid, Route:routeid, Float:distance) {
    return 1;
}
