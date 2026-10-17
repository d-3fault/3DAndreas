# 3DAndreas

> [!NOTE]
> 3DAndreas is experimental and already runs in production. Its natives may still change from one version to the next. It was made with Claude Opus 5.5, a model by [Anthropic](https://www.anthropic.com).

An open.mp component that simulates physics on the map of San Andreas with [Jolt Physics](https://github.com/jrouwe/JoltPhysics) and answers questions about that map: what a ray hits, where the ground is, whether two points see each other, what a shape overlaps, where the water lies and what a player has on screen. It loads from the `components` folder, is configured in `config.json` and reads the map from a database built from the files of the game.

## Features

- **Physics objects.** Spheres, boxes and objects shaped like a model fall, roll, bounce, float and push each other. Players see them as objects the component moves. Static objects and zones never move and make walls, tables and goals.
- **World queries.** Rays give the point, the normal, the model and the surface of the game they hit. There are rays that return every hit, line of sight, ground height, sphere sweeps and the overlaps of a sphere, a box or a model.
- **Players, vehicles and actors.** Players on foot and actors are capsules and vehicles are boxes that move with them, so objects bounce off them and queries find them. Searches in a sphere or a cone can require line of sight, and the camera of a player gives what they look at.
- **The map.** Water levels, points under water or inside the collision of the map, free spots on the ground, predicted throws, the size and the `object.dat` data of models, and building removal for every virtual world or for one.
- **The screen.** Where a point of the world shows on the screen of a player, and what a point of the screen shows, in text draw units.
- **Geometry.** The rotation that stands an object on a slope, reflections, offsets relative to objects and vehicles, the parts of a vehicle, and quaternions.

## Compared with ColAndreas

The same queries on ColAndreas 1.5 and on 3DAndreas, measured with [`tests/colandreas.pwn`](tests/colandreas.pwn) on one computer:

| | ColAndreas 1.5 | 3DAndreas |
| --- | --- | --- |
| Server ready after launch | 2.8 s | 0.47 s |
| Private memory of the server | 557 MB | 178 MB |
| 10,000 rays of 100 metres | 70 ms | 16 ms |
| 10,000 lines of sight | 70 ms | 14 ms |
| 10,000 ground heights | 73 ms | 18 ms |
| 1,000 rays that return every hit | 19 ms | 9 ms |
| 1,000 contact tests | 7 ms | 2 ms |

A bare server, with neither of them, is ready in 0.25 s and uses 51 MB. When rays stop at the water, as they do in ColAndreas, 113 of 120 ground heights across the map are identical and the other 7 lie within 5 mm.

## Installation

1. Download the latest release from [Releases](https://github.com/imshooter/3DAndreas/releases).
2. Extract it into the folder of the server. It holds `components/3DAndreas.dll`, `qawno/include/3DAndreas.inc` and `scriptfiles/3dandreas/world.db`.
3. Add `#include <3DAndreas>` to the scripts that use it, after `streamer.inc` when the script shows physics objects with streamer objects.

Tested with open.mp 1.6.7 on Windows. Only a Windows build is available for now.

## Usage

```pawn
#include <open.mp>
#include <3DAndreas>

public OnPlayerCommandText(playerid, cmdtext[]) {
    if (strcmp(cmdtext, "/ball", true) == 0) {
        new
            Float:x,
            Float:y,
            Float:z,
            Physic:ball
        ;

        GetPlayerPos(playerid, x, y, z);
        ball = A3D_CreateSphere(1598, 0.3, x, y + 1.0, z + 1.0, 0.5, GetPlayerVirtualWorld(playerid));
        A3D_SetBounce(ball, 0.8);
        A3D_SetVelocity(ball, 0.0, 8.0, 4.0);

        return 1;
    }

    return 0;
}

public A3D_OnStopped(Physic:physicid, Float:x, Float:y, Float:z) {
    A3D_DestroyObject(physicid);

    return 1;
}
```

Every native, callback and setting is described in the [wiki](https://github.com/imshooter/3DAndreas/wiki). [`examples/playground.pwn`](examples/playground.pwn) is a filterscript with a command for each kind of feature.

## Configuration

Every setting is optional. These are the defaults:

```json
"3dandreas": {
    "database": "scriptfiles/3dandreas/world.db",
    "threads": 2,
    "rate": 60.0,
    "sync_error": 0.1,
    "sync_horizon": 0.3,
    "collision_speed": 1.0,
    "bounce_speed": 0.1,
    "contact_distance": 0.02,
    "draw_distance": 0.0,
    "floor": -100.0,
    "vehicles": true
}
```

| Key | Description |
| --- | --- |
| `database` | Map database, relative to the server. When the default file is missing, `scriptfiles/colandreas/ColAndreas.cadb` is read instead. |
| `threads` | Worker threads of the simulation. `0` runs it on the thread of the server. |
| `rate` | Simulation steps per second, at least `10`. |
| `sync_error` | Distance in metres the object players see may stray from its body before a new move is sent. |
| `sync_horizon` | Seconds ahead each move sent to the players aims. |
| `collision_speed` | Slowest impact in metres per second that calls `A3D_OnCollided`. |
| `bounce_speed` | Slowest impact in metres per second that bounces. |
| `contact_distance` | Distance in metres at which two bodies start to keep a contact. |
| `draw_distance` | Draw distance of the objects the component creates. `0.0` takes the default of the game. |
| `floor` | Height under which a falling object is stopped. |
| `vehicles` | Whether vehicles take part in the physics world and in the queries. |

The [Configuration](https://github.com/imshooter/3DAndreas/wiki/Configuration) page of the wiki tells when to change each one.

## Map database

`world.db` holds the collision of every model of the game with the surface of each part, the map objects with their interiors, the water, the surfaces of `surfinfo.dat` and the `object.dat` data of the objects that can be knocked over. The release ships one built from GTA San Andreas and the models SA-MP adds. `phydb`, built with the component, makes it again from a game folder, and the file goes to `scriptfiles/3dandreas`:

```
phydb "C:\Games\GTA San Andreas" world.db
```

`--no-samp` after the output leaves out the models of the SA-MP client. A ColAndreas database still loads as a fallback, without surfaces, water and model data.

## Building

Windows with Visual Studio 2022 or its Build Tools, and CMake 3.19 or later. CMake fetches the open.mp SDK, the Pawn headers and Jolt Physics.

```
cmake -S . -B build -A Win32
cmake --build build --config Release
```

The component is written to `build/Release/3DAndreas.dll` and the database builder to `build/Release/phydb.exe`. The server is 32 bit, so the component must be too.

## Tests

The scripts in [`tests`](tests) run as the main script, print what they check and shut the server down. They work on the Los Santos airport, and the ones that use bots need the NPC component of open.mp with `max_bots` set to three or more.

| Script | What it checks |
| --- | --- |
| [`smoke.pwn`](tests/smoke.pwn) | A sphere, a box and a model shaped object dropped on the ground. |
| [`features.pwn`](tests/features.pwn) | Zones, rolling friction, planar motion, gravity, acceleration, collision flags, cylinders and areas. |
| [`streamer.pwn`](tests/streamer.pwn) | A box shown by a streamer object. Needs the streamer plugin. |
| [`queries.pwn`](tests/queries.pwn) | Rays, ground, sight, sweeps, overlaps, worlds, bots and vehicles in shapes, and the cost of each native. |
| [`vehicles.pwn`](tests/vehicles.pwn) | Vehicles in the physics world, explosions and the target of a player. |
| [`parked.pwn`](tests/parked.pwn) | The time a thousand parked vehicles cost. |
| [`database.pwn`](tests/database.pwn) | The surface of hits, water and floating objects, model data and the names and flags of surfaces. |
| [`buildings.pwn`](tests/buildings.pwn) | Building removal for every world and for one, model overlaps, extra data, rotations and reflections. |
| [`probes.pwn`](tests/probes.pwn) | Actors, sight in searches, points inside the map, free spots, throws, offsets, vehicle parts and the screen. |
| [`stress.pwn`](tests/stress.pwn) | Four load scenarios, picked with `PHYBENCH_SCENARIO=1` to `4` and sized with `PHYBENCH_COUNT` on the pawncc command line. |
| [`colandreas.pwn`](tests/colandreas.pwn) | The comparison above. Compiled with `BENCH_COLANDREAS` defined, it runs the same queries on ColAndreas. |

## Thanks to

- [@jrouwe](https://github.com/jrouwe) for [Jolt Physics](https://github.com/jrouwe/JoltPhysics), which simulates the world and answers its queries.
- [@openmultiplayer](https://github.com/openmultiplayer) for the [SDK](https://github.com/openmultiplayer/open.mp-sdk) and the [Pawn compiler](https://github.com/openmultiplayer/compiler).
- [@Pottus](https://github.com/Pottus) and the contributors of [ColAndreas](https://github.com/Pottus/ColAndreas), whose database the component still reads.
- [@AbyssMorgan](https://github.com/AbyssMorgan) for 3DTryg, whose functions inspired the probes, the offsets and the screen natives.

## License

Released under the [MIT License](LICENSE). Jolt Physics keeps its own MIT license, which the release ships as `LICENSE-Jolt.txt`.
