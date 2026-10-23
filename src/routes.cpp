#include "physics.hpp"

#include <Server/Components/GangZones/gangzones.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace physics {

namespace {

constexpr float DefaultReach = 300.0f;
constexpr float ArriveDistance = 15.0f;
constexpr float StrayDistance = 30.0f;
constexpr float ReturnDistance = 15.0f;
constexpr float Continuity = 0.05f;
constexpr uint32_t BehindSegments = 16;
constexpr uint32_t AheadSegments = 64;
constexpr double DisplayInterval = 0.2;
constexpr int MaxSquares = 512;
constexpr int ShowGangZoneRPC = 108;
constexpr int HideGangZoneRPC = 120;
constexpr float SlightTurn = 60.0f;
constexpr float SharpTurn = 120.0f;
constexpr float BackTurn = 160.0f;
constexpr float WalkDirect = 500.0f;
constexpr float WalkLeg = 3.0f;

enum class Turn : cell {
	None,
	SlightLeft,
	Left,
	SharpLeft,
	SlightRight,
	Right,
	SharpRight,
	Back
};

struct Progress {
	float position = 0.0f;
	float away = 0.0f;
	uint32_t segment = 0;
};

const RouteNetwork& networkOf(RouteType type) {
	State& s = state();
	return type == RouteType::Walk ? s.walkNetwork : s.roadNetwork;
}

void output(AMX* amx, const cell* params, int index, cell value) {
	if (params[0] >= cell(index * sizeof(cell))) {
		writeCell(amx, params[index], value);
	}
}

bool typeOf(cell value, RouteType& type) {
	if (value < 0 || value >= RouteTypes) {
		return false;
	}
	type = RouteType(value);
	return true;
}

Route* findRoute(cell routeid) {
	State& s = state();
	if (routeid < 0 || routeid >= cell(s.routes.size()) || !s.routes[size_t(routeid)].used) {
		return nullptr;
	}
	return &s.routes[size_t(routeid)];
}

Turn turnOf(float angle) {
	float size = std::abs(angle);
	if (size >= BackTurn) {
		return Turn::Back;
	}
	int step = size < SlightTurn ? 1 : size < SharpTurn ? 2 : 3;
	return Turn((angle > 0.0f ? 0 : 3) + step);
}

bool places(const RouteNetwork& network, Vector3 from, Vector3 to, RouteType type, float reach, bool connected, NetworkPlace& start, NetworkPlace& end) {
	return network.nearest(from.x, from.y, from.z, reach, type, connected, start) && network.nearest(to.x, to.y, to.z, reach, type, connected, end);
}

bool findPath(RouteType type, Vector3 from, Vector3 to, float heading, float reach, RouteResult& result) {
	const RouteNetwork& network = networkOf(type);
	NetworkPlace start;
	NetworkPlace end;
	if (!places(network, from, to, type, reach, false, start, end)) {
		return false;
	}
	if (network.route(start, end, type, heading, result)) {
		return true;
	}
	if (network.connects(start, type) && network.connects(end, type)) {
		return false;
	}
	return places(network, from, to, type, reach, true, start, end) && network.route(start, end, type, heading, result);
}

float spacing(const RoutePoint& a, const RoutePoint& b) {
	return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

bool walkPath(Vector3 from, Vector3 to, float reach, RouteResult& result) {
	const WalkMesh& mesh = state().walkMesh;
	RoutePoint start = { from.x, from.y, from.z };
	RoutePoint end = { to.x, to.y, to.z };
	bool walkable = mesh.loaded();
	std::vector<RoutePoint> leg;
	float length;
	if (walkable && std::hypot(to.x - from.x, to.y - from.y) <= WalkDirect && mesh.path(start, end, leg, length)) {
		result.points = std::move(leg);
		result.turns.clear();
		result.length = length;
		result.time = length / WalkSpeed;
		result.cost = result.time;
		return true;
	}
	if (!findPath(RouteType::Walk, from, to, -1.0f, reach, result)) {
		return false;
	}
	if (!walkable) {
		return true;
	}
	if (spacing(start, result.points.front()) > WalkLeg && mesh.path(start, result.points.front(), leg, length)) {
		leg.pop_back();
		for (RouteTurn& turn : result.turns) {
			turn.point += uint32_t(leg.size());
		}
		result.points.insert(result.points.begin(), leg.begin(), leg.end());
		result.length += length;
		result.time += length / WalkSpeed;
	}
	if (spacing(result.points.back(), end) > WalkLeg && mesh.path(result.points.back(), end, leg, length)) {
		result.points.insert(result.points.end(), leg.begin() + 1, leg.end());
		result.length += length;
		result.time += length / WalkSpeed;
	}
	return true;
}

RoutePoint pointAlong(const Route& route, float distance) {
	const std::vector<RoutePoint>& points = route.result.points;
	auto after = std::upper_bound(route.along.begin(), route.along.end(), distance);
	if (after == route.along.begin()) {
		return points.front();
	}
	if (after == route.along.end()) {
		return points.back();
	}
	size_t index = size_t(after - route.along.begin()) - 1;
	float span = route.along[index + 1] - route.along[index];
	float t = span > 0.0f ? (distance - route.along[index]) / span : 0.0f;
	const RoutePoint& a = points[index];
	const RoutePoint& b = points[index + 1];
	return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
}

Progress follow(const Route& route, Vector3 at, float previous, uint32_t hint) {
	const std::vector<RoutePoint>& points = route.result.points;
	Progress progress;
	if (points.size() < 2) {
		progress.away = points.empty() ? 0.0f : glm::distance(at, Vector3(points[0].x, points[0].y, points[0].z));
		return progress;
	}
	uint32_t segments = uint32_t(points.size()) - 1;
	auto scan = [&](uint32_t first, uint32_t last) {
		float best = std::numeric_limits<float>::infinity();
		for (uint32_t index = first; index < last; ++index) {
			const RoutePoint& a = points[index];
			const RoutePoint& b = points[index + 1];
			float dx = b.x - a.x;
			float dy = b.y - a.y;
			float dz = (b.z - a.z) * 2.0f;
			float span = dx * dx + dy * dy + dz * dz;
			float t = span > 0.0f ? std::clamp(((at.x - a.x) * dx + (at.y - a.y) * dy + (at.z - a.z) * 2.0f * dz) / span, 0.0f, 1.0f) : 0.0f;
			float px = a.x + dx * t - at.x;
			float py = a.y + dy * t - at.y;
			float pz = (a.z - at.z) * 2.0f + dz * t;
			float position = route.along[index] + (route.along[index + 1] - route.along[index]) * t;
			float score = px * px + py * py + pz * pz + std::abs(position - previous) * Continuity;
			if (score < best) {
				best = score;
				progress.position = position;
				progress.segment = index;
				progress.away = std::sqrt(px * px + py * py + pz * pz * 0.25f);
			}
		}
	};
	uint32_t first = hint > BehindSegments ? hint - BehindSegments : 0;
	uint32_t last = std::min(hint + AheadSegments, segments);
	scan(first, last);
	if (progress.away > StrayDistance && (first != 0 || last != segments)) {
		scan(0, segments);
	}
	return progress;
}

void nextTurn(const Route& route, float position, Turn& turn, float& distance) {
	turn = Turn::None;
	distance = route.along.empty() ? 0.0f : std::max(route.along.back() - position, 0.0f);
	for (const RouteTurn& found : route.result.turns) {
		float at = route.along[found.point];
		if (at > position + 1.0f || (found.point == 0 && position <= 1.0f)) {
			turn = turnOf(found.angle);
			distance = std::max(at - position, 0.0f);
			return;
		}
	}
}

void sendShow(IPlayer& player, int clientid, const RoutePoint& centre, float size, uint32_t colour) {
	uint8_t data[22];
	int16_t id = int16_t(clientid);
	float corners[4] = { centre.x - size * 0.5f, centre.y - size * 0.5f, centre.x + size * 0.5f, centre.y + size * 0.5f };
	std::memcpy(data, &id, sizeof id);
	std::memcpy(data + 2, corners, sizeof corners);
	std::memcpy(data + 18, &colour, sizeof colour);
	player.sendRPC(ShowGangZoneRPC, Span<uint8_t>(data, sizeof data * 8), OrderingChannel_SyncRPC);
}

void sendHide(IPlayer& player, int clientid) {
	int16_t id = int16_t(clientid);
	uint8_t data[2];
	std::memcpy(data, &id, sizeof id);
	player.sendRPC(HideGangZoneRPC, Span<uint8_t>(data, sizeof data * 8), OrderingChannel_SyncRPC);
}

void hideSquares(IPlayer* player, RouteDisplay& display) {
	IPlayerGangZoneData* data = player != nullptr ? queryExtension<IPlayerGangZoneData>(*player) : nullptr;
	for (const RouteSquare& square : display.shown) {
		if (data != nullptr) {
			sendHide(*player, square.clientid);
			data->releaseClientID(square.clientid);
		}
	}
	display.shown.clear();
}

void draw(IPlayer& player, RouteDisplay& display, const Route& route) {
	IPlayerGangZoneData* data = queryExtension<IPlayerGangZoneData>(player);
	if (data == nullptr || route.along.empty()) {
		return;
	}
	float length = route.along.back();
	float step = display.width;
	int64_t first = int64_t(std::floor(display.position / step));
	int64_t last = std::min(first + display.squares / 2, int64_t(std::floor(length / step)));
	std::vector<RouteSquare> wanted;
	for (int64_t key = first; key <= last; ++key) {
		wanted.push_back({ key, -1, step });
	}
	int left = display.squares - int(wanted.size());
	float rest = length - float(last) * step;
	if (left > 0 && rest > step) {
		int64_t stride = 2;
		while (float(stride) * step * float(left) < rest) {
			stride *= 2;
		}
		for (int64_t key = (last / stride + 1) * stride; float(key) * step <= length && left > 0; key += stride, --left) {
			wanted.push_back({ key, -1, std::max(step, float(stride) * step * 0.5f) });
		}
	}
	std::vector<RouteSquare> kept;
	size_t at = 0;
	for (const RouteSquare& square : display.shown) {
		while (at != wanted.size() && wanted[at].key < square.key) {
			++at;
		}
		if (at != wanted.size() && wanted[at].key == square.key && wanted[at].size == square.size) {
			kept.push_back(square);
			continue;
		}
		sendHide(player, square.clientid);
		data->releaseClientID(square.clientid);
	}
	std::vector<RouteSquare> shown;
	shown.reserve(wanted.size());
	size_t keep = 0;
	for (RouteSquare& square : wanted) {
		while (keep != kept.size() && kept[keep].key < square.key) {
			++keep;
		}
		if (keep != kept.size() && kept[keep].key == square.key) {
			shown.push_back(kept[keep]);
			continue;
		}
		int clientid = data->reserveClientID();
		if (clientid == INVALID_GANG_ZONE_ID) {
			break;
		}
		square.clientid = clientid;
		sendShow(player, clientid, pointAlong(route, float(square.key) * step), square.size, display.colour);
		shown.push_back(square);
	}
	display.shown = std::move(shown);
}

void hideDisplay(int playerid) {
	State& s = state();
	if (playerid < 0 || playerid >= int(s.routeDisplays.size()) || s.routeDisplays[size_t(playerid)].route < 0) {
		return;
	}
	RouteDisplay& display = s.routeDisplays[size_t(playerid)];
	hideSquares(s.players != nullptr ? s.players->get(playerid) : nullptr, display);
	display = RouteDisplay();
}

void destroyRoute(int routeid) {
	State& s = state();
	for (int playerid = 0; playerid != int(s.routeDisplays.size()); ++playerid) {
		if (s.routeDisplays[size_t(playerid)].route == routeid) {
			hideDisplay(playerid);
		}
	}
	s.routes[size_t(routeid)] = Route();
	s.freeRoutes.push_back(routeid);
}

cell AMX_NATIVE_CALL A3D_FindRoute(AMX* amx, const cell* params) {
	State& s = state();
	RouteType type;
	if (!typeOf(params[7], type)) {
		return -1;
	}
	float heading = readFloat(params, 8);
	float reach = optionalFloat(params, 9, DefaultReach);
	RouteResult result;
	Vector3 from = readVector(params, 1);
	Vector3 to = readVector(params, 4);
	if (!(type == RouteType::Walk ? walkPath(from, to, reach, result) : findPath(type, from, to, heading, reach, result))) {
		return -1;
	}
	int routeid;
	if (!s.freeRoutes.empty()) {
		routeid = s.freeRoutes.back();
		s.freeRoutes.pop_back();
	} else {
		routeid = int(s.routes.size());
		s.routes.emplace_back();
	}
	Route& route = s.routes[size_t(routeid)];
	route.used = true;
	route.owner = amx;
	route.type = type;
	route.result = std::move(result);
	route.along.assign(route.result.points.size(), 0.0f);
	for (size_t index = 1; index < route.result.points.size(); ++index) {
		const RoutePoint& a = route.result.points[index - 1];
		const RoutePoint& b = route.result.points[index];
		route.along[index] = route.along[index - 1] + glm::distance(Vector3(a.x, a.y, a.z), Vector3(b.x, b.y, b.z));
	}
	return routeid;
}

cell AMX_NATIVE_CALL A3D_DestroyRoute(AMX* amx, const cell* params) {
	if (findRoute(params[1]) == nullptr) {
		return false;
	}
	destroyRoute(params[1]);
	return true;
}

cell AMX_NATIVE_CALL A3D_IsValidRoute(AMX* amx, const cell* params) {
	return findRoute(params[1]) != nullptr;
}

cell AMX_NATIVE_CALL A3D_GetRouteLength(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	float length = route == nullptr ? 0.0f : route->result.length;
	return amx_ftoc(length);
}

cell AMX_NATIVE_CALL A3D_GetRouteTime(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	float time = route == nullptr ? 0.0f : route->result.time;
	return amx_ftoc(time);
}

cell AMX_NATIVE_CALL A3D_GetRoutePointCount(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	return route == nullptr ? 0 : cell(route->result.points.size());
}

cell AMX_NATIVE_CALL A3D_GetRoutePoint(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	if (route == nullptr || params[2] < 0 || params[2] >= cell(route->result.points.size())) {
		return false;
	}
	const RoutePoint& point = route->result.points[size_t(params[2])];
	return writeVector(amx, params, 3, Vector3(point.x, point.y, point.z));
}

cell AMX_NATIVE_CALL A3D_GetRoutePoints(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	cell* x = addressOf(amx, params[2]);
	cell* y = addressOf(amx, params[3]);
	cell* z = addressOf(amx, params[4]);
	if (route == nullptr || x == nullptr || y == nullptr || z == nullptr || params[5] < 0) {
		return 0;
	}
	cell count = std::clamp(cell(route->result.points.size()) - params[5], cell(0), std::max(params[6], cell(0)));
	for (cell index = 0; index != count; ++index) {
		const RoutePoint& point = route->result.points[size_t(params[5] + index)];
		x[index] = amx_ftoc(point.x);
		y[index] = amx_ftoc(point.y);
		z[index] = amx_ftoc(point.z);
	}
	return count;
}

cell AMX_NATIVE_CALL A3D_GetRouteTurnCount(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	return route == nullptr ? 0 : cell(route->result.turns.size());
}

cell AMX_NATIVE_CALL A3D_GetRouteTurn(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	if (route == nullptr || params[2] < 0 || params[2] >= cell(route->result.turns.size())) {
		return cell(Turn::None);
	}
	const RouteTurn& turn = route->result.turns[size_t(params[2])];
	const RoutePoint& point = route->result.points[turn.point];
	writeVector(amx, params, 3, Vector3(point.x, point.y, point.z));
	output(amx, params, 6, amx_ftoc(route->along[turn.point]));
	return cell(turnOf(turn.angle));
}

cell AMX_NATIVE_CALL A3D_GetRouteProgress(AMX* amx, const cell* params) {
	Route* route = findRoute(params[1]);
	if (route == nullptr) {
		return false;
	}
	Progress progress = follow(*route, readVector(params, 2), route->position, route->segment);
	route->position = progress.position;
	route->segment = progress.segment;
	Turn turn;
	float turnDistance;
	nextTurn(*route, progress.position, turn, turnDistance);
	float remaining = std::max(route->along.back() - progress.position, 0.0f);
	writeFloat(amx, params[5], remaining);
	output(amx, params, 6, amx_ftoc(progress.away));
	output(amx, params, 7, cell(progress.segment + 1));
	output(amx, params, 8, cell(turn));
	output(amx, params, 9, amx_ftoc(turnDistance));
	return true;
}

cell AMX_NATIVE_CALL A3D_MeasureRoute(AMX* amx, const cell* params) {
	RouteType type;
	if (!typeOf(params[9], type)) {
		return false;
	}
	const RouteNetwork& network = networkOf(type);
	Vector3 from = readVector(params, 1);
	Vector3 to = readVector(params, 4);
	float heading = readFloat(params, 10);
	float reach = optionalFloat(params, 11, DefaultReach);
	NetworkPlace start;
	NetworkPlace end;
	float length;
	float time;
	if (type == RouteType::Walk) {
		RouteResult result;
		if (!walkPath(from, to, reach, result)) {
			return false;
		}
		writeFloat(amx, params[7], result.length);
		writeFloat(amx, params[8], result.time);
		return true;
	}
	if (!places(network, from, to, type, reach, false, start, end)) {
		return false;
	}
	bool found = network.measure(start, end, type, heading, length, time);
	if (!found && !(network.connects(start, type) && network.connects(end, type))) {
		found = places(network, from, to, type, reach, true, start, end) && network.measure(start, end, type, heading, length, time);
	}
	if (!found) {
		return false;
	}
	writeFloat(amx, params[7], length);
	writeFloat(amx, params[8], time);
	return true;
}

cell AMX_NATIVE_CALL A3D_FindNearestByRoute(AMX* amx, const cell* params) {
	RouteType type;
	cell* x = addressOf(amx, params[4]);
	cell* y = addressOf(amx, params[5]);
	cell* z = addressOf(amx, params[6]);
	if (!typeOf(params[8], type) || x == nullptr || y == nullptr || z == nullptr || params[7] <= 0) {
		return -1;
	}
	const RouteNetwork& network = networkOf(type);
	Vector3 from = readVector(params, 1);
	float heading = readFloat(params, 9);
	float reach = optionalFloat(params, 10, DefaultReach);
	std::vector<NetworkPlace> targets;
	std::vector<cell> owners;
	for (cell index = 0; index != params[7]; ++index) {
		NetworkPlace nearby;
		NetworkPlace main;
		bool anywhere = network.nearest(amx_ctof(x[index]), amx_ctof(y[index]), amx_ctof(z[index]), reach, type, false, nearby);
		if (anywhere) {
			targets.push_back(nearby);
			owners.push_back(index);
		}
		if (anywhere && network.connects(nearby, type)) {
			continue;
		}
		if (network.nearest(amx_ctof(x[index]), amx_ctof(y[index]), amx_ctof(z[index]), reach, type, true, main)) {
			targets.push_back(main);
			owners.push_back(index);
		}
	}
	cell best = -1;
	float bestCost = std::numeric_limits<float>::infinity();
	for (int connected = 0; connected != 2; ++connected) {
		NetworkPlace start;
		if (!network.nearest(from.x, from.y, from.z, reach, type, connected != 0, start)) {
			continue;
		}
		float cost;
		int found = network.closest(start, targets, type, heading, cost);
		if (found >= 0 && cost < bestCost) {
			bestCost = cost;
			best = owners[size_t(found)];
		}
		if (network.connects(start, type)) {
			break;
		}
	}
	return best;
}

cell AMX_NATIVE_CALL A3D_GetNearestPathPoint(AMX* amx, const cell* params) {
	RouteType type;
	if (!typeOf(params[8], type)) {
		return false;
	}
	const RouteNetwork& network = networkOf(type);
	float reach = optionalFloat(params, 9, 100.0f);
	NetworkPlace place;
	if (!network.nearest(readFloat(params, 1), readFloat(params, 2), readFloat(params, 3), reach, type, false, place)) {
		return false;
	}
	float angle = network.headingAt(place, type);
	writeVector(amx, params, 4, Vector3(place.point.x, place.point.y, place.point.z));
	output(amx, params, 7, amx_ftoc(angle));
	return true;
}

cell AMX_NATIVE_CALL A3D_ShowRouteForPlayer(AMX* amx, const cell* params) {
	State& s = state();
	IPlayer* player = s.players->get(params[1]);
	Route* route = findRoute(params[2]);
	float width = readFloat(params, 4);
	if (player == nullptr || player->isBot() || route == nullptr || width <= 0.0f || params[5] <= 0) {
		return false;
	}
	int playerid = player->getID();
	if (playerid >= int(s.routeDisplays.size())) {
		s.routeDisplays.resize(size_t(playerid) + 1);
	}
	hideDisplay(playerid);
	RouteDisplay& display = s.routeDisplays[size_t(playerid)];
	display.route = params[2];
	display.colour = Colour::FromRGBA(uint32_t(params[3])).ABGR();
	display.width = width;
	display.squares = std::min(int(params[5]), MaxSquares);
	Progress progress = follow(*route, player->getPosition(), 0.0f, 0);
	display.position = progress.position;
	display.segment = progress.segment;
	draw(*player, display, *route);
	return true;
}

cell AMX_NATIVE_CALL A3D_HideRouteForPlayer(AMX* amx, const cell* params) {
	State& s = state();
	int playerid = params[1];
	if (playerid < 0 || playerid >= int(s.routeDisplays.size()) || s.routeDisplays[size_t(playerid)].route < 0) {
		return false;
	}
	hideDisplay(playerid);
	return true;
}

cell AMX_NATIVE_CALL A3D_GetPlayerRoute(AMX* amx, const cell* params) {
	State& s = state();
	int playerid = params[1];
	if (playerid < 0 || playerid >= int(s.routeDisplays.size())) {
		return -1;
	}
	return s.routeDisplays[size_t(playerid)].route;
}

const AMX_NATIVE_INFO natives[] = {
	{ "A3D_FindRoute", A3D_FindRoute },
	{ "A3D_DestroyRoute", A3D_DestroyRoute },
	{ "A3D_IsValidRoute", A3D_IsValidRoute },
	{ "A3D_GetRouteLength", A3D_GetRouteLength },
	{ "A3D_GetRouteTime", A3D_GetRouteTime },
	{ "A3D_GetRoutePointCount", A3D_GetRoutePointCount },
	{ "A3D_GetRoutePoint", A3D_GetRoutePoint },
	{ "A3D_GetRoutePoints", A3D_GetRoutePoints },
	{ "A3D_GetRouteTurnCount", A3D_GetRouteTurnCount },
	{ "A3D_GetRouteTurn", A3D_GetRouteTurn },
	{ "A3D_GetRouteProgress", A3D_GetRouteProgress },
	{ "A3D_MeasureRoute", A3D_MeasureRoute },
	{ "A3D_FindNearestByRoute", A3D_FindNearestByRoute },
	{ "A3D_GetNearestPathPoint", A3D_GetNearestPathPoint },
	{ "A3D_ShowRouteForPlayer", A3D_ShowRouteForPlayer },
	{ "A3D_HideRouteForPlayer", A3D_HideRouteForPlayer },
	{ "A3D_GetPlayerRoute", A3D_GetPlayerRoute },
};

}

void loadRoutes(PathGraph roads, PathGraph walks) {
	State& s = state();
	s.routesStarted = std::chrono::steady_clock::now();
	s.routesAnnounced = roads.nodes.empty();
	if (s.routesAnnounced) {
		return;
	}
	s.walkNetwork.buildWalks(std::move(walks), roads);
	s.roadNetwork.buildRoads(std::move(roads));
}

void stopRoutes() {
	State& s = state();
	s.roadNetwork.clear();
	s.walkNetwork.clear();
	s.walkMesh.clear();
}

void forgetRoutes(AMX* amx) {
	State& s = state();
	for (int routeid = 0; routeid != int(s.routes.size()); ++routeid) {
		if (s.routes[size_t(routeid)].used && s.routes[size_t(routeid)].owner == amx) {
			destroyRoute(routeid);
		}
	}
}

void removeRoutePlayer(int playerid) {
	State& s = state();
	if (playerid >= 0 && playerid < int(s.routeDisplays.size())) {
		s.routeDisplays[size_t(playerid)] = RouteDisplay();
	}
}

void updateRoutes() {
	State& s = state();
	if (!s.routesAnnounced && s.roadNetwork.ready() && s.walkNetwork.ready() && s.walkMesh.ready()) {
		s.routesAnnounced = true;
		double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - s.routesStarted).count();
		if (s.walkMesh.loaded()) {
			s.core->printLn("3DAndreas " A3D_VERSION ": routes on %u road and %u path junctions, and %u tiles of navigation mesh in %.1f MB, ready in %.1f s", unsigned(s.roadNetwork.junctionCount()), unsigned(s.walkNetwork.junctionCount()), unsigned(s.walkMesh.tileCount()), double(s.walkMesh.byteCount()) / 1048576.0, seconds);
		} else {
			s.core->printLn("3DAndreas " A3D_VERSION ": routes on %u road and %u path junctions, ready in %.1f s, without %s", unsigned(s.roadNetwork.junctionCount()), unsigned(s.walkNetwork.junctionCount()), seconds, s.settings.navmesh.c_str());
		}
	}
	if (s.now - s.routesUpdatedAt < DisplayInterval) {
		return;
	}
	s.routesUpdatedAt = s.now;
	for (int playerid = 0; playerid < int(s.routeDisplays.size()); ++playerid) {
		RouteDisplay& display = s.routeDisplays[size_t(playerid)];
		if (display.route < 0) {
			continue;
		}
		IPlayer* player = s.players->get(playerid);
		Route* route = findRoute(display.route);
		if (player == nullptr || route == nullptr) {
			hideDisplay(playerid);
			continue;
		}
		int routeid = display.route;
		Progress progress = follow(*route, player->getPosition(), display.position, display.segment);
		display.position = progress.position;
		display.segment = progress.segment;
		draw(*player, display, *route);
		auto found = s.scripts.find(route->owner);
		Script* script = found == s.scripts.end() ? nullptr : &found->second;
		float remaining = route->along.back() - progress.position;
		if (!display.arrived && remaining <= ArriveDistance && progress.away <= StrayDistance) {
			display.arrived = true;
			if (script != nullptr) {
				callPublic(*script, "A3D_OnRouteArrived", { cell(playerid), cell(routeid) });
			}
			continue;
		}
		if (!display.strayed && progress.away > StrayDistance) {
			display.strayed = true;
			if (script != nullptr) {
				callPublic(*script, "A3D_OnRouteLeft", { cell(playerid), cell(routeid), amx_ftoc(progress.away) });
			}
		} else if (display.strayed && progress.away < ReturnDistance) {
			display.strayed = false;
		}
	}
}

void registerRouteNatives(IPawnScript& script) {
	script.Register(natives, int(sizeof natives / sizeof natives[0]));
}

}
