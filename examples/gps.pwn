#define FILTERSCRIPT

#include <open.mp>
#include <3DAndreas>

/**
 * A filterscript that makes a GPS of the routes of 3DAndreas. A player marks a place on the map of
 * the pause menu, and a line on the radar leads there with the next turn on the screen.
 *
 * - Clicking the map sets the destination. In a vehicle the route drives, on foot it walks.
 * - `/gps` tells the distance and the time left, and `/gps off` hides the route.
 *
 * The route is found again when the player strays from it, or gets in or out of a vehicle. The new
 * route is shown before the old one is destroyed, so the line keeps the part the two share. The next
 * turn is written when `A3D_OnRouteAdvanced` says it changed, so no timer runs.
 */

enum E_GPS {
    Route:E_GPS_ROUTE,
    Float:E_GPS_X,
    Float:E_GPS_Y,
    Float:E_GPS_Z,
    PlayerText:E_GPS_TEXT
};

static
    gGps[MAX_PLAYERS][E_GPS]
;

static const gTurnText[A3D_TURN][] = {
    "Arrive",
    "Keep left",
    "Turn left",
    "Turn sharp left",
    "Keep right",
    "Turn right",
    "Turn sharp right",
    "Turn back"
};

static stock CreateGpsText(playerid) {
    new PlayerText:text = CreatePlayerTextDraw(playerid, 320.0, 360.0, "_");

    PlayerTextDrawAlignment(playerid, text, TEXT_DRAW_ALIGN_CENTER);
    PlayerTextDrawFont(playerid, text, TEXT_DRAW_FONT_1);
    PlayerTextDrawLetterSize(playerid, text, 0.32, 1.4);
    PlayerTextDrawColour(playerid, text, 0xFFFFFFFF);
    PlayerTextDrawSetOutline(playerid, text, 1);
    gGps[playerid][E_GPS_TEXT] = text;
    gGps[playerid][E_GPS_ROUTE] = INVALID_ROUTE_ID;
}

static stock StopGps(playerid) {
    A3D_DestroyRoute(gGps[playerid][E_GPS_ROUTE]);
    gGps[playerid][E_GPS_ROUTE] = INVALID_ROUTE_ID;
    PlayerTextDrawHide(playerid, gGps[playerid][E_GPS_TEXT]);
}

static stock ShowGpsTurn(playerid, Float:remaining, A3D_TURN:turn, Float:turnDistance) {
    new
        text[96]
    ;

    if (turn == A3D_TURN_NONE) {
        format(text, sizeof (text), "Arrive in %i m", floatround(remaining));
    } else {
        format(text, sizeof (text), "%s in %i m~n~%.1f km to go", gTurnText[turn], floatround(turnDistance), remaining / 1000.0);
    }

    PlayerTextDrawSetString(playerid, gGps[playerid][E_GPS_TEXT], text);
}

static stock bool:FindGpsRoute(playerid) {
    new
        Float:x,
        Float:y,
        Float:z,
        Float:heading = -1.0,
        A3D_ROUTE:type = A3D_ROUTE_WALK,
        vehicleid = GetPlayerVehicleID(playerid)
    ;

    GetPlayerPos(playerid, x, y, z);

    if (vehicleid != 0) {
        type = A3D_ROUTE_FASTEST;
    }

    if (GetPlayerState(playerid) == PLAYER_STATE_DRIVER) {
        GetVehicleZAngle(vehicleid, heading);
    }

    new Route:routeid = A3D_FindRoute(x, y, z, gGps[playerid][E_GPS_X], gGps[playerid][E_GPS_Y], gGps[playerid][E_GPS_Z], type, heading);

    if (routeid == INVALID_ROUTE_ID) {
        return false;
    }

    new
        Float:remaining,
        A3D_TURN:turn,
        Float:turnDistance
    ;

    A3D_ShowRouteForPlayer(playerid, routeid);
    A3D_DestroyRoute(gGps[playerid][E_GPS_ROUTE]);
    gGps[playerid][E_GPS_ROUTE] = routeid;
    A3D_GetRouteProgress(routeid, x, y, z, remaining, .turn = turn, .turnDistance = turnDistance);
    ShowGpsTurn(playerid, remaining, turn, turnDistance);
    PlayerTextDrawShow(playerid, gGps[playerid][E_GPS_TEXT]);

    return true;
}

public OnFilterScriptInit() {
    for (new i; i != MAX_PLAYERS; ++i) {
        if (IsPlayerConnected(i)) {
            CreateGpsText(i);
        }
    }

    return 1;
}

public OnFilterScriptExit() {
    for (new i; i != MAX_PLAYERS; ++i) {
        if (IsPlayerConnected(i)) {
            PlayerTextDrawDestroy(i, gGps[i][E_GPS_TEXT]);
        }
    }

    return 1;
}

public OnPlayerConnect(playerid) {
    CreateGpsText(playerid);

    return 1;
}

public OnPlayerDisconnect(playerid, reason) {
    A3D_DestroyRoute(gGps[playerid][E_GPS_ROUTE]);
    gGps[playerid][E_GPS_ROUTE] = INVALID_ROUTE_ID;

    return 1;
}

public OnPlayerClickMap(playerid, Float:fX, Float:fY, Float:fZ) {
    new
        Float:z,
        message[96]
    ;

    if (!A3D_FindGround(fX, fY, 1000.0, z)) {
        z = fZ;
    }

    gGps[playerid][E_GPS_X] = fX;
    gGps[playerid][E_GPS_Y] = fY;
    gGps[playerid][E_GPS_Z] = z;

    if (!FindGpsRoute(playerid)) {
        StopGps(playerid);
        SendClientMessage(playerid, -1, "No route leads there.");

        return 1;
    }

    format(message, sizeof (message), "Route set: %.1f km, about %i minutes.", A3D_GetRouteLength(gGps[playerid][E_GPS_ROUTE]) / 1000.0, floatround(A3D_GetRouteTime(gGps[playerid][E_GPS_ROUTE]) / 60.0, floatround_ceil));
    SendClientMessage(playerid, -1, message);

    return 1;
}

public OnPlayerStateChange(playerid, PLAYER_STATE:newstate, PLAYER_STATE:oldstate) {
    if (gGps[playerid][E_GPS_ROUTE] == INVALID_ROUTE_ID) {
        return 1;
    }

    if (newstate != PLAYER_STATE_DRIVER && newstate != PLAYER_STATE_PASSENGER && newstate != PLAYER_STATE_ONFOOT) {
        return 1;
    }

    if (!FindGpsRoute(playerid)) {
        StopGps(playerid);
    }

    return 1;
}

public OnPlayerCommandText(playerid, cmdtext[]) {
    if (strcmp(cmdtext, "/gps off", true) == 0) {
        StopGps(playerid);
        SendClientMessage(playerid, -1, "The route is hidden.");

        return 1;
    }

    if (strcmp(cmdtext, "/gps", true) != 0) {
        return 0;
    }

    if (gGps[playerid][E_GPS_ROUTE] == INVALID_ROUTE_ID) {
        SendClientMessage(playerid, -1, "Mark a place on the map of the pause menu to get a route there.");

        return 1;
    }

    new
        Float:x,
        Float:y,
        Float:z,
        Float:remaining,
        Route:routeid = gGps[playerid][E_GPS_ROUTE],
        message[96]
    ;

    GetPlayerPos(playerid, x, y, z);
    A3D_GetRouteProgress(routeid, x, y, z, remaining);
    format(message, sizeof (message), "%.1f km left, about %i minutes.", remaining / 1000.0, floatround(A3D_GetRouteTime(routeid) * remaining / A3D_GetRouteLength(routeid) / 60.0, floatround_ceil));
    SendClientMessage(playerid, -1, message);

    return 1;
}

public A3D_OnRouteAdvanced(playerid, Route:routeid, Float:remaining, A3D_TURN:turn, Float:turnDistance) {
    if (routeid != gGps[playerid][E_GPS_ROUTE]) {
        return 1;
    }

    ShowGpsTurn(playerid, remaining, turn, turnDistance);

    return 1;
}

public A3D_OnRouteLeft(playerid, Route:routeid, Float:distance) {
    if (routeid != gGps[playerid][E_GPS_ROUTE]) {
        return 1;
    }

    if (!FindGpsRoute(playerid)) {
        StopGps(playerid);
        SendClientMessage(playerid, -1, "The route was lost.");
    }

    return 1;
}

public A3D_OnRouteArrived(playerid, Route:routeid) {
    if (routeid != gGps[playerid][E_GPS_ROUTE]) {
        return 1;
    }

    StopGps(playerid);
    SendClientMessage(playerid, -1, "You arrived.");

    return 1;
}
