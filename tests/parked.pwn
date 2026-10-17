#include <open.mp>
#include <3DAndreas>

/**
 * Parks a thousand vehicles and reads what the component spends over ten seconds with nothing
 * else to simulate.
 */

static
    gStartMilliseconds,
    gParked
;

main() {}

forward @Parked_Start();
public @Parked_Start() {
    new
        active,
        updates,
        Float:milliseconds
    ;

    A3D_GetStats(active, updates, milliseconds);
    gStartMilliseconds = floatround(milliseconds * 1000.0);
    SetTimer("@Parked_End", 10000, false);

    return 1;
}

forward @Parked_End();
public @Parked_End() {
    new
        active,
        updates,
        Float:milliseconds
    ;

    A3D_GetStats(active, updates, milliseconds);
    printf("[parked] %i vehicles, %.2f ms per second in the component, %i bodies awake", gParked, (milliseconds - float(gStartMilliseconds) / 1000.0) / 10.0, active);
    SendRconCommand("exit");

    return 1;
}

public OnGameModeInit() {
    for (new i; i != 1000; ++i) {
        if (CreateVehicle(411, 1000.0 + float(i % 40) * 8.0, -2600.0 + float(i / 40) * 8.0, 13.5, 0.0, -1, -1, 60) != INVALID_VEHICLE_ID) {
            ++gParked;
        }
    }

    SetTimer("@Parked_Start", 3000, false);

    return 1;
}
