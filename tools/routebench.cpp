#include "../src/database.hpp"
#include "../src/routing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace physics;

namespace {

using Clock = std::chrono::steady_clock;

const char* const TypeNames[RouteTypes] = { "fastest", "shortest", "any way", "traffic", "boat", "walk" };

double since(Clock::time_point start) {
	return std::chrono::duration<double, std::micro>(Clock::now() - start).count();
}

struct Plain {
	std::vector<float> cost;
	std::vector<uint32_t> stamp;
	uint32_t mark = 0;
};

float plainSearch(const PathGraph& graph, uint32_t source, uint32_t target, Plain& plain) {
	using Entry = std::pair<float, uint32_t>;
	if (plain.cost.size() != graph.nodes.size()) {
		plain.cost.assign(graph.nodes.size(), 0.0f);
		plain.stamp.assign(graph.nodes.size(), 0);
	}
	uint32_t mark = ++plain.mark;
	std::vector<Entry> heap;
	plain.cost[source] = 0.0f;
	plain.stamp[source] = mark;
	heap.emplace_back(0.0f, source);
	while (!heap.empty()) {
		std::pop_heap(heap.begin(), heap.end(), std::greater<Entry>());
		Entry top = heap.back();
		heap.pop_back();
		if (top.first > plain.cost[top.second]) {
			continue;
		}
		if (top.second == target) {
			return top.first;
		}
		for (uint32_t index = graph.first[top.second]; index != graph.first[top.second + 1]; ++index) {
			const PathLink& link = graph.links[index];
			if (link.lanes == 0) {
				continue;
			}
			float next = top.first + link.length;
			if (plain.stamp[link.target] != mark || next < plain.cost[link.target]) {
				plain.stamp[link.target] = mark;
				plain.cost[link.target] = next;
				heap.emplace_back(next, link.target);
				std::push_heap(heap.begin(), heap.end(), std::greater<Entry>());
			}
		}
	}
	return std::numeric_limits<float>::infinity();
}

struct Named {
	const char* name;
	float x;
	float y;
	float z;
};

}

int main(int argc, char** argv) {
	std::setvbuf(stdout, nullptr, _IONBF, 0);
	if (argc < 2) {
		std::printf("usage: routebench <world.db> [routes.txt]\n");
		return 1;
	}
	Clock::time_point start = Clock::now();
	Database database;
	if (!readDatabase(argv[1], database)) {
		std::printf("cannot read %s\n", argv[1]);
		return 1;
	}
	std::printf("database read in %.1f ms: %zu road nodes, %zu walk nodes\n", since(start) / 1000.0, database.roads.nodes.size(), database.walks.nodes.size());
	RouteNetwork roads;
	RouteNetwork walks;
	start = Clock::now();
	roads.buildRoads(database.roads);
	walks.buildWalks(database.walks, database.roads);
	double launchTime = since(start) / 1000.0;
	start = Clock::now();
	size_t roadJunctions = roads.junctionCount();
	double roadTime = since(start) / 1000.0;
	size_t walkJunctions = walks.junctionCount();
	double walkTime = since(start) / 1000.0;
	PathGraph joined = joinWalks(database.walks, database.roads);
	NetworkPlace early;
	NetworkPlace late;
	RouteResult before;
	bool fallback = roads.nearest(2495.0f, -1687.0f, 13.5f, 300.0f, RouteType::Fastest, true, early) && roads.nearest(-2026.0f, 156.0f, 29.0f, 300.0f, RouteType::Fastest, true, late);
	bool contractedEarly = roads.contracted(RouteType::Fastest);
	Clock::time_point began = Clock::now();
	fallback = fallback && roads.route(early, late, RouteType::Fastest, -1.0f, before);
	double fallbackTime = since(began);
	start = Clock::now();
	roads.wait();
	double roadContract = since(start) / 1000.0;
	walks.wait();
	double walkContract = since(start) / 1000.0;
	std::printf("started in %.2f ms\n", launchTime);
	std::printf("roads: %zu junctions, %zu chains, built %.1f ms after the start, contracted %.1f ms later\n", roadJunctions, roads.chainCount(), roadTime, roadContract);
	std::printf("walks: %zu nodes and %zu links joined, %zu junctions, %zu chains, built %.1f ms after the start, contracted %.1f ms later\n", joined.nodes.size(), joined.links.size(), walkJunctions, walks.chainCount(), walkTime, walkContract);
	RouteResult after;
	roads.route(early, late, RouteType::Fastest, -1.0f, after);
	std::printf("networks without hierarchies: roads %.1f MB, walks %.1f MB\n", double(roads.memory()) / 1048576.0, double(walks.memory()) / 1048576.0);
	std::printf("first route %s: %.1f us, %.0f m against %.0f m once contracted\n", contractedEarly ? "by the hierarchy" : "by A* while contracting", fallbackTime, before.length, after.length);
	for (int index = 0; index != RouteTypes; ++index) {
		RouteType type = RouteType(index);
		const RouteNetwork& network = type == RouteType::Walk ? walks : roads;
		if (!network.serves(type)) {
			continue;
		}
		std::printf("  %-8s %7zu arcs %7zu shortcuts, main part %zu states, %.1f MB\n", TypeNames[index], network.arcCount(type), network.shortcutCount(type), network.componentSize(type), double(network.memory(type)) / 1048576.0);
	}

	std::mt19937 random(20261022);
	for (int index = 0; index != RouteTypes; ++index) {
		RouteType type = RouteType(index);
		const RouteNetwork& network = type == RouteType::Walk ? walks : roads;
		const PathGraph& graph = type == RouteType::Walk ? joined : database.roads;
		if (!network.serves(type)) {
			continue;
		}
		std::vector<NetworkPlace> places;
		std::uniform_int_distribution<size_t> pick(0, graph.nodes.size() - 1);
		std::uniform_real_distribution<float> jitter(-25.0f, 25.0f);
		double nearestTime = 0.0;
		int nearestCalls = 0;
		while (places.size() < 1000) {
			const PathNode& node = graph.nodes[pick(random)];
			NetworkPlace place;
			Clock::time_point began = Clock::now();
			bool found = network.nearest(node.x + jitter(random), node.y + jitter(random), node.z, 200.0f, type, true, place);
			nearestTime += since(began);
			++nearestCalls;
			if (found) {
				places.push_back(place);
			}
		}
		int pairs = 0;
		int mismatches = 0;
		int failures = 0;
		double hierarchyTime = 0.0;
		double measureTime = 0.0;
		double searchTime = 0.0;
		double points = 0.0;
		double turns = 0.0;
		float worst = 0.0f;
		RouteResult fast;
		RouteResult slow;
		for (size_t pair = 0; pair + 1 < places.size(); pair += 2) {
			const NetworkPlace& from = places[pair];
			const NetworkPlace& to = places[pair + 1];
			float heading = float(pair % 360);
			Clock::time_point began = Clock::now();
			bool first = network.route(from, to, type, heading, fast);
			hierarchyTime += since(began);
			float length;
			float time;
			began = Clock::now();
			network.measure(from, to, type, heading, length, time);
			measureTime += since(began);
			began = Clock::now();
			bool second = network.search(from, to, type, heading, slow);
			searchTime += since(began);
			++pairs;
			if (first != second) {
				++mismatches;
				continue;
			}
			if (!first) {
				++failures;
				continue;
			}
			points += double(fast.points.size());
			turns += double(fast.turns.size());
			float difference = std::abs(fast.cost - slow.cost) / std::max(1.0f, slow.cost);
			worst = std::max(worst, difference);
			if (difference > 1e-3f || std::abs(length - fast.length) > 0.5f + fast.length * 1e-3f || std::abs(time - fast.time) > 0.05f + fast.time * 1e-3f) {
				if (mismatches < 5) {
					std::printf("    mismatch: hierarchy %.3f (%.1f m, %.1f s) search %.3f (%.1f m, %.1f s) measure %.1f m %.1f s\n", fast.cost, fast.length, fast.time, slow.cost, slow.length, slow.time, length, time);
				}
				++mismatches;
			}
		}
		int found = pairs - failures - mismatches;
		std::printf("%-8s %d pairs, %d not connected, %d mismatches (worst %.2e): nearest %.2f us, hierarchy %.1f us, measure %.1f us, A* %.1f us, %.0f points and %.1f turns a route\n",
			TypeNames[index], pairs, failures, mismatches, double(worst), nearestTime / nearestCalls, hierarchyTime / pairs, measureTime / pairs, searchTime / pairs, found > 0 ? points / found : 0.0, found > 0 ? turns / found : 0.0);
		if (type == RouteType::Fastest || type == RouteType::Walk) {
			Plain plain;
			std::uniform_int_distribution<uint32_t> node(0, uint32_t(graph.nodes.size() - 1));
			double plainTime = 0.0;
			int reached = 0;
			for (int run = 0; run != 200; ++run) {
				uint32_t a = node(random);
				uint32_t b = node(random);
				Clock::time_point began = Clock::now();
				float cost = plainSearch(graph, a, b, plain);
				plainTime += since(began);
				reached += std::isfinite(cost) ? 1 : 0;
			}
			std::printf("         Dijkstra on every node, as a plain plugin does: %.1f us a search (%d of 200 reached)\n", plainTime / 200.0, reached);
		}
		std::vector<NetworkPlace> targets(places.begin(), places.begin() + 50);
		began = Clock::now();
		int runs = 0;
		for (size_t pair = 100; pair != 200; ++pair) {
			float cost;
			network.closest(places[pair], targets, type, -1.0f, cost);
			++runs;
		}
		std::printf("         closest of 50 places: %.1f us\n", since(began) / runs);
	}

	const Named named[] = {
		{ "Grove Street", 2495.0f, -1687.0f, 13.5f },
		{ "Doherty garage", -2026.0f, 156.0f, 29.0f },
		{ "Four Dragons", 2027.0f, 1007.0f, 10.8f },
		{ "Angel Pine", -2161.0f, -2384.0f, 30.6f },
		{ "Las Barrancas", -828.0f, 1504.0f, 19.0f },
		{ "Market", 1132.0f, -1418.0f, 13.5f },
	};
	FILE* output = argc > 2 ? std::fopen(argv[2], "w") : nullptr;
	for (int type = 0; type != RouteTypes; ++type) {
		const RouteNetwork& network = RouteType(type) == RouteType::Walk ? walks : roads;
		if (!network.serves(RouteType(type)) || RouteType(type) == RouteType::Boat) {
			continue;
		}
		for (size_t a = 0; a != sizeof named / sizeof named[0]; ++a) {
			size_t b = (a + 1) % (sizeof named / sizeof named[0]);
			NetworkPlace from;
			NetworkPlace to;
			if (!network.nearest(named[a].x, named[a].y, named[a].z, 500.0f, RouteType(type), true, from) || !network.nearest(named[b].x, named[b].y, named[b].z, 500.0f, RouteType(type), true, to)) {
				std::printf("%-8s %s to %s: no place near\n", TypeNames[type], named[a].name, named[b].name);
				continue;
			}
			RouteResult result;
			Clock::time_point began = Clock::now();
			bool found = network.route(from, to, RouteType(type), -1.0f, result);
			double took = since(began);
			if (!found) {
				std::printf("%-8s %s to %s: not connected\n", TypeNames[type], named[a].name, named[b].name);
				continue;
			}
			std::printf("%-8s %s to %s: %.0f m, %.0f s, %zu points, %zu turns, %.1f us\n", TypeNames[type], named[a].name, named[b].name, result.length, result.time, result.points.size(), result.turns.size(), took);
			if (output != nullptr) {
				std::fprintf(output, "route %d %s|%s\n", type, named[a].name, named[b].name);
				for (const RoutePoint& point : result.points) {
					std::fprintf(output, "%.2f %.2f %.2f\n", point.x, point.y, point.z);
				}
				for (const RouteTurn& turn : result.turns) {
					std::fprintf(output, "turn %u %.1f\n", turn.point, turn.angle);
				}
			}
		}
	}
	if (output != nullptr) {
		std::fclose(output);
	}
	return 0;
}
