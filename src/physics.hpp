#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Collision/GroupFilter.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <sdk.hpp>
#include <Server/Components/Actors/actors.hpp>
#include <Server/Components/Objects/objects.hpp>
#include <Server/Components/Pawn/pawn.hpp>
#include <Server/Components/Vehicles/vehicles.hpp>

#include "database.hpp"

namespace physics {

constexpr int InvalidPhys = -1;
constexpr int InvalidObject = 0xFFFF;
constexpr float StandardGravity = 9.81f;

/**
 * @brief      Room for every surface id of the game, and the surfaces the game gives to peds and to
 *             vehicles, which the bodies of players and vehicles take.
 */
constexpr int SurfaceCount = 256;
constexpr uint8_t SurfacePed = 62;
constexpr uint8_t SurfaceCar = 63;

/**
 * @brief      What a Jolt body stands for, kept in the high half of its user data.
 */
enum class Kind : uint32_t {
	Map = 1,
	Object = 2,
	Static = 3,
	Player = 4,
	Zone = 5,
	Vehicle = 6,
	Actor = 7
};

/**
 * @brief      What a collision hit, in the order of `A3D_HIT` in 3DAndreas.inc.
 */
enum class Hit : int {
	Map = 0,
	Object = 1,
	Static = 2,
	Player = 3,
	Vehicle = 4,
	Water = 5,
	Actor = 6
};

/**
 * @brief      What a body collides with, in the order of `A3D_COLLIDE` in 3DAndreas.inc.
 * `CollideAll` holds every kind of body and not the water.
 */
constexpr uint32_t CollideMap = 1;
constexpr uint32_t CollideObjects = 2;
constexpr uint32_t CollideStatic = 4;
constexpr uint32_t CollidePlayers = 8;
constexpr uint32_t CollideVehicles = 16;
constexpr uint32_t CollideActors = 64;
constexpr uint32_t CollideAll = 95;

/**
 * @brief      Rays and lines of sight stop at the surface of the water. Bodies never collide with it.
 */
constexpr uint32_t CollideWater = 32;

/**
 * @brief      How far above the position of a ped its chest is, the point sight and the screen aim at.
 */
constexpr float ChestHeight = 0.5f;

/**
 * @brief      The collision group of a map object holds this bit and the index of its placement, so
 *             a removal in one virtual world can be told apart from the rest.
 */
constexpr uint32_t MapGroup = 0x80000000u;

/**
 * @brief      Numbers a script may keep on each physics object with `A3D_SetExtra`.
 */
constexpr int ExtraSlots = 8;

/**
 * @brief      How the object that players see follows a body.
 */
enum class Sync {
	None,
	Object,
	Script
};

/**
 * @brief      A body created by a script: a moving object, a static object or a zone.
 *
 * `from`, `to`, `speed`, `sentAt` and `duration` describe the last move sent, so the position the
 * client shows can be worked out at any time.
 */
struct Body {
	JPH::BodyID id;
	bool used = false;
	Kind kind = Kind::Object;
	int modelid = -1;
	int world = -1;
	uint32_t collisions = CollideAll;
	AMX* owner = nullptr;
	float mass = 0.0f;
	JPH::Vec3 acceleration = JPH::Vec3::sZero();
	float rollingFriction = 0.0f;
	float collisionSpeed = 1.0f;
	Sync sync = Sync::None;
	int objectid = -1;
	Vector3 from = Vector3(0.0f);
	Vector3 to = Vector3(0.0f);
	Vector3 rotation = Vector3(0.0f);
	float syncError = 0.1f;
	float speed = 0.0f;
	double sentAt = 0.0;
	double duration = 0.0;
	float buoyancy = 0.0f;
	float drag = 1.0f;
	std::array<cell, ExtraSlots> extra = {};
};

struct Collision {
	int phys;
	JPH::BodyID id;
	Hit hit;
	int hitid;
	Vector3 point;
	float speed;
};

struct Stop {
	int phys;
	JPH::BodyID id;
};

/**
 * @brief      A moving object that entered or left a zone.
 */
struct Crossing {
	int zone;
	JPH::BodyID zoneId;
	int phys;
	JPH::BodyID physId;
	bool entered;
};

/**
 * @brief      A loaded script.
 *
 * Publics are found by name at each call, because a library can rename and sort the table of
 * publics after the script loads.
 */
struct Script {
	IPawnScript* api = nullptr;
};

struct Settings {
	std::string database = "scriptfiles/3dandreas/world.db";
	int threads = 2;
	float rate = 60.0f;
	float syncError = 0.1f;
	float syncHorizon = 0.3f;
	float collisionSpeed = 1.0f;
	float bounceSpeed = 0.1f;
	float contactDistance = 0.02f;
	float drawDistance = 0.0f;
	float floor = -100.0f;
	bool vehicles = true;
};

struct State {
	Settings settings;
	ICore* core = nullptr;
	IPawnComponent* pawn = nullptr;
	IObjectsComponent* objects = nullptr;
	IVehiclesComponent* vehicles = nullptr;
	IActorsComponent* actors = nullptr;
	IPlayerPool* players = nullptr;
	std::mt19937 random;
	std::unordered_map<int, Model> models;
	std::vector<Placement> placements;
	std::vector<JPH::BodyID> placementBodies;
	std::vector<uint8_t> placementRemoved;
	std::vector<uint8_t> placementHidden;
	std::unordered_map<uint32_t, std::vector<int>> hiddenWorlds;
	std::vector<Water> waters;
	std::vector<Surface> surfaces;
	bool colAndreas = false;
	std::vector<std::vector<int>> waterCells;
	float waterMinX = 0.0f;
	float waterMinY = 0.0f;
	int waterColumns = 0;
	int waterRows = 0;
	std::unordered_map<int, JPH::ShapeRefC> staticShapes;
	std::unordered_map<int, JPH::ShapeRefC> dynamicShapes;
	JPH::ShapeRefC playerShape;
	std::unique_ptr<JPH::TempAllocatorImpl> temp;
	std::unique_ptr<JPH::JobSystemThreadPool> jobs;
	std::unique_ptr<JPH::PhysicsSystem> system;
	JPH::Ref<JPH::GroupFilter> worlds;
	std::vector<uint64_t> userDataById;
	std::vector<Body> bodies;
	std::vector<int> freeBodies;
	std::vector<int> accelerated;
	std::vector<int> floating;
	std::vector<JPH::BodyID> playerBodies;
	std::vector<int> playerWorlds;
	std::unordered_map<int, JPH::ShapeRefC> vehicleShapes;
	std::vector<JPH::BodyID> vehicleBodies;
	std::vector<int> vehicleModels;
	std::vector<int> vehicleWorlds;
	std::vector<Vector3> vehiclePositions;
	std::vector<glm::quat> vehicleRotations;
	std::vector<uint8_t> vehicleMoving;
	std::vector<JPH::BodyID> actorBodies;
	std::vector<int> actorWorlds;
	std::vector<Vector3> actorPositions;
	std::vector<uint8_t> actorMoving;
	std::unordered_map<AMX*, Script> scripts;
	std::mutex eventLock;
	std::vector<Collision> collisions;
	std::vector<Stop> stops;
	std::vector<Crossing> crossings;
	double accumulator = 0.0;
	double now = 0.0;
	uint64_t updates = 0;
	double milliseconds = 0.0;
	int mapBodies = 0;
};

/**
 * @brief      The state of the component.
 */
State& state();

/**
 * @brief      Start Jolt and create the physics system.
 *
 * @param      maxBodies  Room for map bodies, script bodies and players together.
 */
void startWorld(int maxBodies);
void stopWorld();

/**
 * @brief      Hand the contact and activation listeners to the physics system.
 */
void listen();

/**
 * @brief      Read the collision of every model, the map placements, the surfaces and the water from
 *             a database of the component, or from a ColAndreas database.
 *
 * @param      path  Path of the database.
 *
 * @return     - `true` the database was read
 *             - `false` the file is missing, truncated or of an unknown kind
 */
bool loadDatabase(const std::string& path);

/**
 * @brief      The surface of the game a part of a body is made of, `0` for the default surface.
 */
uint8_t surfaceOf(const JPH::Body& body, const JPH::SubShapeID& part);

/**
 * @brief      The Jolt material of a surface of the game.
 */
const JPH::PhysicsMaterial* surfaceMaterial(uint8_t surface);

/**
 * @brief      Sort the water polygons into cells, for `waterLevel`.
 */
void indexWater();

/**
 * @brief      The height of the water at a point of the map, the highest where two polygons cover
 *             it. Past the polygons lies the sea at level `0.0`, as in the game.
 *
 * @return     - `true` water covers the point
 *             - `false` it does not, or the database has no water
 */
bool waterLevel(float x, float y, float& level);

/**
 * @brief      Whether a point lies under the surface of the water with nothing of the map between
 *             them, so the pool of an interior placed in the sky does not fill the land below it.
 */
bool underWater(Vector3 point, float& level);

/**
 * @brief      Whether the map lies between two points.
 */
bool mapBetween(Vector3 from, Vector3 to);

/**
 * @brief      Where a segment first crosses the surface of the water, as a fraction of its length.
 *
 * @return     - `true` the segment crosses the water
 *             - `false` it does not, or the database has no water
 */
bool waterAlong(Vector3 from, Vector3 to, float& fraction);

/**
 * @brief      Push the floating bodies that are in water up by their buoyancy.
 */
void applyBuoyancy(float elapsed);

/**
 * @brief      Float a body in water, or stop it floating when `buoyancy` is `0.0`.
 */
void setBuoyancy(int index, float buoyancy, float drag);

/**
 * @brief      Add every map placement of the database to the world as a static body.
 *
 * @return     the number of map bodies added
 */
int buildMap();

/**
 * @brief      Whether a map object was removed in a virtual world by `A3D_RemoveBuilding`. A body of
 *             every world, `-1`, only misses the map objects removed for every world, which leave
 *             the physics system.
 */
bool mapHidden(uint32_t placement, int world);

/**
 * @brief      The shape a model collides with when it stands still.
 *
 * @return     - the shape
 *             - `nullptr` the model has no collision in the database
 */
JPH::ShapeRefC staticShape(int modelid);

/**
 * @brief      The shape a model collides with when it moves, a convex hull of its collision.
 *
 * @return     - the shape
 *             - `nullptr` the model has no collision in the database
 */
JPH::ShapeRefC dynamicShape(int modelid);

/**
 * @brief      The collision group of a body: its virtual world, what it is, and what it collides with.
 *
 * @param      world       Virtual world, or `-1` for every world.
 * @param      kind        What the body stands for.
 * @param      collisions  What it collides with, from `CollideMap` to `CollidePlayers`.
 */
JPH::CollisionGroup bodyGroup(int world, Kind kind, uint32_t collisions);

/**
 * @brief      Turn the rotation of a SA-MP object into a Jolt quaternion.
 */
JPH::Quat toJolt(Vector3 euler);

/**
 * @brief      Turn a Jolt quaternion into the rotation of a SA-MP object.
 */
Vector3 toEuler(JPH::QuatArg rotation);

uint64_t userData(Kind kind, int index);
Kind kindOf(uint64_t data);
int indexOf(uint64_t data);

/**
 * @brief      Keep the user data of a body by its id, so listeners can read it after the body is gone.
 */
void rememberBody(const JPH::BodyID& id, uint64_t data);

/**
 * @brief      The user data of a body by its id, or `0` when the id is unknown.
 */
uint64_t recalledData(const JPH::BodyID& id);

JPH::ObjectLayer staticLayer();
JPH::ObjectLayer movingLayer();
JPH::ObjectLayer playerLayer();

/**
 * @brief      Create a body and, for a moving body with a model, the object players see.
 *
 * @param      kind  `Kind::Object` for a moving body, `Kind::Static` or `Kind::Zone` for one that never moves.
 *
 * @return     - the index of the body
 *             - `InvalidPhys` the world is full
 */
int createBody(AMX* owner, const JPH::Shape* shape, int modelid, Vector3 position, JPH::QuatArg rotation, float mass, int world, Kind kind);

/**
 * @brief      Remove a body and the object it moves.
 */
bool destroyBody(int index);

/**
 * @brief      The body of an index, or `nullptr` when the index holds none.
 */
Body* findBody(int index);

/**
 * @brief      Give a body a new collision group after its world or its collisions changed.
 */
void regroupBody(Body& body);

/**
 * @brief      Push a body by a constant acceleration on every step, or stop pushing it.
 */
void setAcceleration(int index, JPH::Vec3Arg acceleration);

/**
 * @brief      Remove every body a script created.
 */
void forgetScript(AMX* amx);

void removePlayer(int playerid);

/**
 * @brief      Give every vehicle a kinematic box the size of its model, and move the boxes with
 *             the vehicles, so physics objects and queries meet them.
 *
 * @param      elapsed  Seconds the next step lasts.
 */
void updateVehicles(float elapsed);

void removeVehicle(int vehicleid);

/**
 * @brief      Give every actor an upright capsule like the one of a player, and move the capsules with
 *             the actors, so physics objects and queries meet them.
 *
 * @param      elapsed  Seconds the next step lasts.
 */
void updateActors(float elapsed);

void removeActor(int actorid);

/**
 * @brief      The upright capsule of a ped, shared by players and actors.
 */
JPH::ShapeRefC pedShape();

/**
 * @brief      Advance the simulation by the time since the last tick, in fixed steps.
 *
 * @param      elapsed  Seconds since the last tick.
 */
void step(double elapsed);

/**
 * @brief      Send a new move to every moving object whose client position strayed from its body.
 */
void syncBodies();

/**
 * @brief      Put the object of a body that fell asleep exactly where the body rests.
 */
void syncRest(Body& body);

/**
 * @brief      Move the object of a body to a new place at once.
 */
void syncTeleport(Body& body);

/**
 * @brief      Call a public of a script, if the script has it.
 *
 * @param      script     Script to call.
 * @param      name       Name of the public.
 * @param      arguments  Arguments in the order the public takes them.
 *
 * @return     - `true` the public was called
 *             - `false` the script has no such public
 */
bool callPublic(Script& script, const char* name, const std::vector<cell>& arguments);

/**
 * @brief      The record of the script that created a body, or `nullptr` when it is gone.
 */
Script* ownerOf(const Body& body);

/**
 * @brief      Register the natives of the component in a script.
 */
void registerNatives(IPawnScript& script);

/**
 * @brief      Register the natives that look into the world: rays, sweeps, overlaps, and players and
 *             vehicles inside a shape.
 */
void registerQueryNatives(IPawnScript& script);

/**
 * @brief      Register the natives that read the data of the database: water, models and surfaces,
 *             and the natives that remove map objects.
 */
void registerMapNatives(IPawnScript& script);

/**
 * @brief      Register the natives that turn rotations and directions: Euler angles and quaternions,
 *             the rotation that stands an object on a surface, and reflections.
 */
void registerGeometryNatives(IPawnScript& script);

/**
 * @brief      What a query hit: the point, the side of the surface facing the query, what the body
 *             stands for, its id in Pawn and the surface of the part hit.
 */
struct QueryHit {
	JPH::RVec3 point;
	JPH::Vec3 normal = JPH::Vec3::sAxisZ();
	Hit type = Hit::Map;
	int id = 0;
	int material = 0;
};

/**
 * @brief      Lets a query see the bodies of the kinds asked for in one virtual world, bodies of
 *             every world included, and never a zone, a removed map object or the bodies it leaves
 *             out.
 */
class QueryFilter final : public JPH::BodyFilter {
public:
	QueryFilter(int world, uint32_t flags, JPH::BodyID first = JPH::BodyID(), JPH::BodyID second = JPH::BodyID());
	bool ShouldCollideLocked(const JPH::Body& body) const override;

private:
	int world_;
	int64_t group_;
	uint32_t flags_;
	JPH::BodyID first_;
	JPH::BodyID second_;
};

JPH::RVec3 toJoltPoint(Vector3 point);
Vector3 toPoint(JPH::RVec3Arg point);

/**
 * @brief      The first thing a segment hits, the water too when `flags` has `CollideWater`.
 *
 * Triangles are hit from both sides, as the collision of the game is one sided. A segment that
 * starts inside a box, a sphere or a ped passes out of it unseen.
 */
bool castRay(Vector3 from, Vector3 to, int world, uint32_t flags, bool normal, QueryHit& hit, JPH::BodyID first = JPH::BodyID(), JPH::BodyID second = JPH::BodyID());

/**
 * @brief      Whether anything of the kinds asked for lies between two points.
 */
bool blocked(Vector3 from, Vector3 to, int world, uint32_t flags);

/**
 * @brief      Where a sphere moved along a segment first touches something, as the centre of the
 *             sphere then.
 */
bool sweepSphere(float radius, Vector3 from, Vector3 to, int world, uint32_t flags, QueryHit& hit);

/**
 * @brief      Whether a shape placed at a transform touches anything of the kinds asked for.
 */
bool touching(const JPH::Shape* shape, JPH::RMat44Arg transform, int world, uint32_t flags);

/**
 * @brief      Write ids nearest first into a Pawn array, as many as fit.
 *
 * @return     the number of ids written
 */
cell writeNearest(AMX* amx, std::vector<std::pair<float, int>>& found, cell address, cell size);

/**
 * @brief      Whether a player is in the world: spawned, alive and not spectating.
 */
bool present(IPlayer& player);

/**
 * @brief      The capsule of a player and the box of the vehicle they are in, which a ray from their
 *             camera leaves out.
 */
void ownBodies(IPlayer& player, JPH::BodyID& own, JPH::BodyID& ride);

/**
 * @brief      The rotation of a vehicle for Jolt, the conjugate of the quaternion the game syncs.
 */
JPH::Quat vehicleRotation(const GTAQuat& rotation);

/**
 * @brief      Register the natives that probe the map around a point: whether it lies inside the
 *             map, a free spot nearby and where a throw lands.
 */
void registerProbeNatives(IPawnScript& script);

/**
 * @brief      Register the natives that read the screen of a player from the camera the client sends.
 */
void registerScreenNatives(IPawnScript& script);

float readFloat(const cell* params, int index);

/**
 * @brief      An argument a script may leave out because an older include did not have it, or
 *             `fallback` when the call did not pass it.
 */
cell optional(const cell* params, int index, cell fallback);
Vector3 readVector(const cell* params, int index);

/**
 * @brief      The cell an argument passed by reference or an array points to.
 *
 * @return     - the cell
 *             - `nullptr` the address is outside the script
 */
cell* addressOf(AMX* amx, cell address);

bool writeCell(AMX* amx, cell address, cell value);
bool writeFloat(AMX* amx, cell address, float value);
bool writeVector(AMX* amx, const cell* params, int index, Vector3 value);

}
