#include "routing.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>

namespace physics {

namespace {

constexpr float Infinite = std::numeric_limits<float>::infinity();
constexpr uint32_t None = UINT32_MAX;
constexpr uint32_t SeedMark = 0x80000000u;

constexpr uint8_t WayForward = 1;
constexpr uint8_t WayBackward = 2;
constexpr uint8_t LinkForward = 4;
constexpr uint8_t LinkBackward = 8;
constexpr uint8_t ChainMixed = 0x80;

constexpr float HighwaySpeed = 36.0f;
constexpr float RoadSpeed = 22.0f;
constexpr float QuietSpeed = 14.0f;
constexpr float BoatSpeed = 15.0f;
constexpr float RoadWalk = 1.25f;
constexpr float HighwayWalk = 2.5f;

constexpr float RightTurn = 2.0f;
constexpr float LeftTurn = 5.0f;
constexpr float BackTurn = 30.0f;
constexpr float BackMetres = 100.0f;
constexpr float StraightAngle = 30.0f;
constexpr float BackAngle = 150.0f;
constexpr float Opposed = 0.3f;

constexpr float LookAhead = 12.0f;
constexpr float TurnMerge = 25.0f;
constexpr float TurnShown = 25.0f;
constexpr float GridCell = 50.0f;
constexpr float JoinReach = 20.0f;
constexpr int SimulatedSettles = 8;
constexpr int WitnessSettles = 32;

constexpr float Degrees = 57.29577951f;

using Entry = std::pair<float, uint32_t>;

bool driving(RouteType type) {
	return type == RouteType::Fastest || type == RouteType::Shortest || type == RouteType::AnyWay || type == RouteType::Traffic;
}

bool turnless(RouteType type) {
	return type == RouteType::Walk || type == RouteType::Boat;
}

bool usable(RouteType type, uint8_t flags) {
	if ((flags & ChainMixed) != 0) {
		return false;
	}
	switch (type) {
	case RouteType::Boat:
		return (flags & PathBoat) != 0;
	case RouteType::Traffic:
		return (flags & (PathBoat | PathNoTraffic)) == 0;
	default:
		return (flags & PathBoat) == 0;
	}
}

bool allowed(RouteType type, uint8_t ways, bool reverse) {
	if (type == RouteType::AnyWay || type == RouteType::Boat || type == RouteType::Walk) {
		return (ways & (LinkForward | LinkBackward)) != 0;
	}
	return (ways & (reverse ? WayBackward : WayForward)) != 0;
}

float speedOf(RouteType type, uint8_t flags) {
	if (type == RouteType::Walk) {
		return WalkSpeed;
	}
	if (type == RouteType::Boat) {
		return BoatSpeed;
	}
	if ((flags & PathHighway) != 0) {
		return HighwaySpeed;
	}
	return (flags & PathNoTraffic) != 0 ? QuietSpeed : RoadSpeed;
}

float factorOf(RouteType type, uint8_t flags) {
	if (type != RouteType::Walk || (flags & PathRoad) == 0) {
		return 1.0f;
	}
	return (flags & PathHighway) != 0 ? HighwayWalk : RoadWalk;
}

uint8_t flip(uint8_t ways) {
	return uint8_t(((ways & WayForward) << 1) | ((ways & WayBackward) >> 1) | ((ways & LinkForward) << 1) | ((ways & LinkBackward) >> 1));
}

void push(std::vector<Entry>& heap, float cost, uint32_t node) {
	heap.emplace_back(cost, node);
	std::push_heap(heap.begin(), heap.end(), std::greater<Entry>());
}

Entry pop(std::vector<Entry>& heap) {
	std::pop_heap(heap.begin(), heap.end(), std::greater<Entry>());
	Entry top = heap.back();
	heap.pop_back();
	return top;
}

float distance(const RoutePoint& a, const RoutePoint& b) {
	return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

uint64_t cellKey(int32_t column, int32_t row) {
	return (uint64_t(uint32_t(column)) << 32) | uint32_t(row);
}

}

PathGraph joinWalks(const PathGraph& walks, const PathGraph& roads) {
	uint32_t base = uint32_t(walks.nodes.size());
	uint32_t total = base + uint32_t(roads.nodes.size());
	std::vector<std::vector<PathLink>> links(total);
	for (uint32_t node = 0; node != base; ++node) {
		for (uint32_t index = walks.first[node]; index != walks.first[node + 1]; ++index) {
			links[node].push_back(walks.links[index]);
		}
	}
	for (uint32_t node = 0; node != uint32_t(roads.nodes.size()); ++node) {
		for (uint32_t index = roads.first[node]; index != roads.first[node + 1]; ++index) {
			PathLink link = roads.links[index];
			link.target += base;
			link.lanes = 1;
			links[base + node].push_back(link);
		}
	}
	PathGraph joined;
	joined.nodes = walks.nodes;
	for (const PathNode& node : roads.nodes) {
		PathNode road = node;
		road.flags |= PathRoad;
		joined.nodes.push_back(road);
	}
	std::unordered_map<uint64_t, std::vector<uint32_t>> walkCells;
	std::unordered_map<uint64_t, std::vector<uint32_t>> roadCells;
	for (uint32_t node = 0; node != total; ++node) {
		const PathNode& found = joined.nodes[node];
		if ((found.flags & PathBoat) == 0) {
			int32_t column = int32_t(std::floor(found.x / JoinReach));
			int32_t row = int32_t(std::floor(found.y / JoinReach));
			(node < base ? walkCells : roadCells)[cellKey(column, row)].push_back(node);
		}
	}
	auto nearest = [&](const std::unordered_map<uint64_t, std::vector<uint32_t>>& cells, const PathNode& from) {
		uint32_t best = None;
		float bestDistance = JoinReach * JoinReach;
		int32_t column = int32_t(std::floor(from.x / JoinReach));
		int32_t row = int32_t(std::floor(from.y / JoinReach));
		for (int32_t x = column - 1; x <= column + 1; ++x) {
			for (int32_t y = row - 1; y <= row + 1; ++y) {
				auto cell = cells.find(cellKey(x, y));
				if (cell == cells.end()) {
					continue;
				}
				for (uint32_t node : cell->second) {
					const PathNode& to = joined.nodes[node];
					float dz = (to.z - from.z) * 2.0f;
					float squared = (to.x - from.x) * (to.x - from.x) + (to.y - from.y) * (to.y - from.y) + dz * dz;
					if (squared < bestDistance) {
						bestDistance = squared;
						best = node;
					}
				}
			}
		}
		return best;
	};
	auto join = [&](uint32_t a, uint32_t b) {
		for (const PathLink& existing : links[a]) {
			if (existing.target == b) {
				return;
			}
		}
		const PathNode& first = joined.nodes[a];
		const PathNode& second = joined.nodes[b];
		PathLink forward;
		forward.target = b;
		forward.length = std::sqrt((second.x - first.x) * (second.x - first.x) + (second.y - first.y) * (second.y - first.y) + (second.z - first.z) * (second.z - first.z));
		forward.lanes = 1;
		PathLink backward = forward;
		backward.target = a;
		links[a].push_back(forward);
		links[b].push_back(backward);
	};
	for (uint32_t node = 0; node != total; ++node) {
		const PathNode& found = joined.nodes[node];
		uint32_t degree = node < base ? walks.first[node + 1] - walks.first[node] : roads.first[node - base + 1] - roads.first[node - base];
		if (degree == 2 || (found.flags & PathBoat) != 0) {
			continue;
		}
		uint32_t other = nearest(node < base ? roadCells : walkCells, found);
		if (other != None) {
			join(node, other);
		}
	}
	joined.first.reserve(total + 1);
	for (uint32_t node = 0; node != total; ++node) {
		joined.first.push_back(uint32_t(joined.links.size()));
		joined.links.insert(joined.links.end(), links[node].begin(), links[node].end());
	}
	joined.first.push_back(uint32_t(joined.links.size()));
	return joined;
}

RouteNetwork::~RouteNetwork() {
	clear();
}

void RouteNetwork::clear() {
	stopping_.store(true);
	if (worker_.joinable()) {
		worker_.join();
	}
	stopping_.store(false);
	built_.store(false);
	started_ = false;
	for (Hierarchy& found : hierarchies_) {
		found.used = false;
		found.cost.clear();
		found.time.clear();
		found.arcs.clear();
		found.outFirst.clear();
		found.outArcs.clear();
		found.component.clear();
		found.mainSize = 0;
		found.contraction.reset();
		found.ready.store(false);
	}
	nodes_.clear();
	degrees_.clear();
	junctionNodes_.clear();
	chains_.clear();
	chainNodes_.clear();
	chainOffsets_.clear();
	segmentChains_.clear();
	gridColumns_ = 0;
	gridRows_ = 0;
	gridFirst_.clear();
	gridSegments_.clear();
}

void RouteNetwork::buildRoads(PathGraph roads) {
	auto shared = std::make_shared<PathGraph>(std::move(roads));
	launch([shared]() {
		return std::move(*shared);
	}, true);
}

void RouteNetwork::buildWalks(PathGraph walks, PathGraph roads) {
	auto shared = std::make_shared<std::pair<PathGraph, PathGraph>>(std::move(walks), std::move(roads));
	launch([shared]() {
		return joinWalks(shared->first, shared->second);
	}, false);
}

void RouteNetwork::launch(std::function<PathGraph()> make, bool vehicles) {
	clear();
	vehicles_ = vehicles;
	started_ = true;
	worker_ = std::thread([this, make]() {
		PathGraph graph = make();
		buildChains(graph);
		buildDirections();
		for (int index = 0; index != RouteTypes; ++index) {
			RouteType type = RouteType(index);
			if ((type == RouteType::Walk) != vehicles_) {
				buildHierarchy(type, hierarchies_[size_t(index)]);
			}
		}
		buildGrid();
		size_t states = chains_.size() * 2 + junctionNodes_.size();
		forwardCost_.assign(states, 0.0f);
		forwardParent_.assign(states, None);
		forwardStamp_.assign(states, 0);
		backwardCost_.assign(states, 0.0f);
		backwardParent_.assign(states, None);
		backwardStamp_.assign(states, 0);
		stamp_ = 0;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			built_.store(true, std::memory_order_release);
		}
		changed_.notify_all();
		for (Hierarchy& found : hierarchies_) {
			if (!found.used) {
				continue;
			}
			std::unique_ptr<Contraction> contraction(new Contraction());
			if (!contract(found, *contraction)) {
				return;
			}
			found.contraction = std::move(contraction);
			found.ready.store(true, std::memory_order_release);
		}
	});
}

bool RouteNetwork::built() const {
	if (built_.load(std::memory_order_acquire)) {
		return true;
	}
	if (!started_) {
		return false;
	}
	std::unique_lock<std::mutex> lock(mutex_);
	changed_.wait(lock, [this]() {
		return built_.load(std::memory_order_acquire);
	});
	return true;
}

bool RouteNetwork::ready() const {
	if (!built_.load(std::memory_order_acquire)) {
		return false;
	}
	for (const Hierarchy& found : hierarchies_) {
		if (found.used && !found.ready.load(std::memory_order_acquire)) {
			return false;
		}
	}
	return true;
}

void RouteNetwork::wait() {
	if (worker_.joinable()) {
		worker_.join();
	}
}

size_t RouteNetwork::junctionCount() const {
	return built() ? junctionNodes_.size() : 0;
}

size_t RouteNetwork::chainCount() const {
	return built() ? chains_.size() : 0;
}

bool RouteNetwork::serves(RouteType type) const {
	return built() && hierarchy(type).used;
}

bool RouteNetwork::contracted(RouteType type) const {
	return built() && contractionOf(hierarchy(type)) != nullptr;
}

size_t RouteNetwork::arcCount(RouteType type) const {
	return built() ? hierarchy(type).arcs.size() : 0;
}

size_t RouteNetwork::shortcutCount(RouteType type) const {
	if (!built()) {
		return 0;
	}
	const Hierarchy& found = hierarchy(type);
	const Contraction* contraction = contractionOf(found);
	return contraction == nullptr ? 0 : contraction->arcs.size() - found.arcs.size();
}

template <typename T>
size_t bytesOf(const std::vector<T>& values) {
	return values.capacity() * sizeof(T);
}

size_t RouteNetwork::memory(RouteType type) const {
	if (!built()) {
		return 0;
	}
	const Hierarchy& found = hierarchy(type);
	size_t total = bytesOf(found.cost) + bytesOf(found.time) + bytesOf(found.arcs) + bytesOf(found.outFirst) + bytesOf(found.outArcs) + bytesOf(found.component);
	const Contraction* contraction = contractionOf(found);
	if (contraction != nullptr) {
		total += bytesOf(contraction->arcs) + bytesOf(contraction->upFirst) + bytesOf(contraction->up) + bytesOf(contraction->downFirst) + bytesOf(contraction->down);
	}
	return total;
}

size_t RouteNetwork::memory() const {
	if (!built()) {
		return 0;
	}
	return bytesOf(nodes_) + bytesOf(degrees_) + bytesOf(junctionNodes_) + bytesOf(chains_) + bytesOf(chainNodes_) + bytesOf(chainOffsets_) + bytesOf(segmentChains_) + bytesOf(leaveX_) * 4 + bytesOf(startFirst_) + bytesOf(startStates_) + bytesOf(gridFirst_) + bytesOf(gridSegments_) + bytesOf(forwardCost_) * 6;
}

size_t RouteNetwork::componentSize(RouteType type) const {
	return built() ? hierarchy(type).mainSize : 0;
}

const RouteNetwork::Hierarchy& RouteNetwork::hierarchy(RouteType type) const {
	return hierarchies_[size_t(type)];
}

const RouteNetwork::Contraction* RouteNetwork::contractionOf(const Hierarchy& hierarchy) const {
	return hierarchy.ready.load(std::memory_order_acquire) ? hierarchy.contraction.get() : nullptr;
}

uint32_t RouteNetwork::startOf(uint32_t state) const {
	const Chain& chain = chains_[state >> 1];
	return (state & 1) != 0 ? chain.to : chain.from;
}

uint32_t RouteNetwork::endOf(uint32_t state) const {
	const Chain& chain = chains_[state >> 1];
	return (state & 1) != 0 ? chain.from : chain.to;
}

RoutePoint RouteNetwork::junctionPoint(uint32_t junction) const {
	const PathNode& node = nodes_[junctionNodes_[junction]];
	return { node.x, node.y, node.z };
}

void RouteNetwork::buildChains(const PathGraph& graph) {
	struct Side {
		uint32_t node;
		uint8_t ways;
		float length;
	};
	uint32_t count = uint32_t(graph.nodes.size());
	nodes_ = graph.nodes;
	std::vector<std::vector<Side>> sides(count);
	auto side = [&](uint32_t from, uint32_t to) -> Side& {
		for (Side& found : sides[from]) {
			if (found.node == to) {
				return found;
			}
		}
		sides[from].push_back({ to, 0, 0.0f });
		return sides[from].back();
	};
	for (uint32_t node = 0; node != count; ++node) {
		for (uint32_t index = graph.first[node]; index != graph.first[node + 1]; ++index) {
			const PathLink& link = graph.links[index];
			if (link.target == node || link.target >= count) {
				continue;
			}
			uint8_t lanes = link.lanes != 0 ? WayForward : 0;
			Side& ahead = side(node, link.target);
			ahead.ways |= LinkForward | lanes;
			ahead.length = link.length;
			Side& behind = side(link.target, node);
			behind.ways |= flip(LinkForward | lanes);
			behind.length = link.length;
		}
	}
	auto kind = [&](uint32_t a, uint32_t b) {
		uint8_t first = nodes_[a].flags;
		uint8_t second = nodes_[b].flags;
		uint8_t flags = uint8_t((first & second & (PathBoat | PathHighway | PathRoad)) | ((first | second) & PathNoTraffic));
		if ((first & PathBoat) != (second & PathBoat)) {
			flags |= ChainMixed;
		}
		return flags;
	};
	degrees_.assign(count, 0);
	std::vector<uint32_t> junctionOf(count, None);
	for (uint32_t node = 0; node != count; ++node) {
		degrees_[node] = uint8_t(std::min<size_t>(sides[node].size(), 255));
		bool passing = sides[node].size() == 2;
		if (passing) {
			const Side& before = sides[node][0];
			const Side& after = sides[node][1];
			passing = flip(before.ways) == after.ways && kind(before.node, node) == kind(node, after.node);
		}
		if (!passing && !sides[node].empty()) {
			junctionOf[node] = uint32_t(junctionNodes_.size());
			junctionNodes_.push_back(node);
		}
	}
	std::vector<std::vector<uint8_t>> used(count);
	for (uint32_t node = 0; node != count; ++node) {
		used[node].assign(sides[node].size(), 0);
	}
	auto mark = [&](uint32_t from, uint32_t to) {
		for (size_t index = 0; index != sides[from].size(); ++index) {
			if (sides[from][index].node == to) {
				used[from][index] = 1;
			}
		}
	};
	std::vector<uint8_t> visited(count, 0);
	auto walk = [&](uint32_t start, size_t first) {
		const Side& opening = sides[start][first];
		Chain chain;
		chain.from = junctionOf[start];
		chain.first = uint32_t(chainNodes_.size());
		chain.flags = kind(start, opening.node);
		chain.ways = opening.ways;
		uint32_t chainIndex = uint32_t(chains_.size());
		chainNodes_.push_back(start);
		chainOffsets_.push_back(0.0f);
		segmentChains_.push_back(chainIndex);
		uint32_t previous = start;
		uint32_t current = opening.node;
		float length = opening.length;
		while (true) {
			chainNodes_.push_back(current);
			chainOffsets_.push_back(length);
			segmentChains_.push_back(chainIndex);
			if (junctionOf[current] != None) {
				break;
			}
			visited[current] = 1;
			const Side& next = sides[current][0].node == previous ? sides[current][1] : sides[current][0];
			previous = current;
			current = next.node;
			length += next.length;
		}
		mark(start, opening.node);
		mark(current, previous);
		chain.to = junctionOf[current];
		chain.count = uint32_t(chainNodes_.size()) - chain.first;
		chain.length = length;
		chains_.push_back(chain);
	};
	for (uint32_t junction = 0; junction != uint32_t(junctionNodes_.size()); ++junction) {
		uint32_t node = junctionNodes_[junction];
		for (size_t index = 0; index != sides[node].size(); ++index) {
			if (used[node][index] == 0) {
				walk(node, index);
			}
		}
	}
	for (uint32_t node = 0; node != count; ++node) {
		if (junctionOf[node] != None || visited[node] != 0 || sides[node].empty()) {
			continue;
		}
		junctionOf[node] = uint32_t(junctionNodes_.size());
		junctionNodes_.push_back(node);
		for (size_t index = 0; index != sides[node].size(); ++index) {
			if (used[node][index] == 0) {
				walk(node, index);
			}
		}
	}
}

RoutePoint RouteNetwork::pointAt(uint32_t chain, float offset) const {
	const Chain& found = chains_[chain];
	const float* begin = chainOffsets_.data() + found.first;
	const float* end = begin + found.count;
	const float* after = std::upper_bound(begin, end, offset);
	if (after == begin) {
		const PathNode& node = nodes_[chainNodes_[found.first]];
		return { node.x, node.y, node.z };
	}
	if (after == end) {
		const PathNode& node = nodes_[chainNodes_[found.first + found.count - 1]];
		return { node.x, node.y, node.z };
	}
	size_t index = size_t(after - begin) - 1;
	const PathNode& a = nodes_[chainNodes_[found.first + index]];
	const PathNode& b = nodes_[chainNodes_[found.first + index + 1]];
	float span = *after - *(after - 1);
	float t = span > 0.0f ? (offset - *(after - 1)) / span : 0.0f;
	return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t, a.z + (b.z - a.z) * t };
}

void RouteNetwork::buildDirections() {
	size_t states = chains_.size() * 2;
	leaveX_.assign(states, 0.0f);
	leaveY_.assign(states, 0.0f);
	arriveX_.assign(states, 0.0f);
	arriveY_.assign(states, 0.0f);
	auto unit = [](float x, float y, float& outX, float& outY) {
		float length = std::sqrt(x * x + y * y);
		outX = length > 0.0f ? x / length : 0.0f;
		outY = length > 0.0f ? y / length : 0.0f;
	};
	for (uint32_t index = 0; index != uint32_t(chains_.size()); ++index) {
		const Chain& chain = chains_[index];
		RoutePoint start = pointAt(index, 0.0f);
		RoutePoint end = pointAt(index, chain.length);
		RoutePoint ahead = pointAt(index, std::min(LookAhead, chain.length));
		RoutePoint behind = pointAt(index, std::max(chain.length - LookAhead, 0.0f));
		uint32_t forward = index * 2;
		unit(ahead.x - start.x, ahead.y - start.y, leaveX_[forward], leaveY_[forward]);
		unit(end.x - behind.x, end.y - behind.y, arriveX_[forward], arriveY_[forward]);
		leaveX_[forward + 1] = -arriveX_[forward];
		leaveY_[forward + 1] = -arriveY_[forward];
		arriveX_[forward + 1] = -leaveX_[forward];
		arriveY_[forward + 1] = -leaveY_[forward];
	}
	startFirst_.assign(junctionNodes_.size() + 1, 0);
	for (uint32_t state = 0; state != uint32_t(states); ++state) {
		++startFirst_[startOf(state) + 1];
	}
	for (size_t junction = 0; junction != junctionNodes_.size(); ++junction) {
		startFirst_[junction + 1] += startFirst_[junction];
	}
	startStates_.assign(states, 0);
	std::vector<uint32_t> fill(startFirst_.begin(), startFirst_.end() - 1);
	for (uint32_t state = 0; state != uint32_t(states); ++state) {
		startStates_[fill[startOf(state)]++] = state;
	}
}

float RouteNetwork::turnAngle(uint32_t from, uint32_t to) const {
	if (to == (from ^ 1)) {
		return 180.0f;
	}
	float x = arriveX_[from];
	float y = arriveY_[from];
	float nextX = leaveX_[to];
	float nextY = leaveY_[to];
	return std::atan2(x * nextY - y * nextX, x * nextX + y * nextY) * Degrees;
}

float RouteNetwork::turnCost(RouteType type, uint32_t from, uint32_t to, float& seconds) const {
	seconds = 0.0f;
	if (!driving(type)) {
		return 0.0f;
	}
	float angle = turnAngle(from, to);
	bool back = to == (from ^ 1) || std::abs(angle) > BackAngle;
	if (back) {
		seconds = BackTurn;
	} else if (degrees_[junctionNodes_[endOf(from)]] > 2) {
		seconds = angle > StraightAngle ? LeftTurn : angle < -StraightAngle ? RightTurn : 0.0f;
	}
	if (type == RouteType::Shortest) {
		return back ? BackMetres : 0.0f;
	}
	return seconds;
}

void RouteNetwork::buildHierarchy(RouteType type, Hierarchy& hierarchy) {
	uint32_t states = uint32_t(chains_.size() * 2);
	uint32_t junctions = turnless(type) ? uint32_t(junctionNodes_.size()) : 0;
	hierarchy.cost.assign(states + junctions, Infinite);
	hierarchy.time.assign(states + junctions, 0.0f);
	hierarchy.least = Infinite;
	for (uint32_t state = 0; state != states; ++state) {
		const Chain& chain = chains_[state >> 1];
		if (!usable(type, chain.flags) || !allowed(type, chain.ways, (state & 1) != 0)) {
			continue;
		}
		float speed = speedOf(type, chain.flags);
		float factor = factorOf(type, chain.flags);
		hierarchy.time[state] = chain.length / speed;
		hierarchy.cost[state] = type == RouteType::Shortest ? chain.length : hierarchy.time[state] * factor;
		hierarchy.least = std::min(hierarchy.least, type == RouteType::Shortest ? 1.0f : factor / speed);
		hierarchy.used = true;
	}
	if (!hierarchy.used) {
		return;
	}
	hierarchy.back = type == RouteType::Shortest ? BackMetres : driving(type) ? BackTurn : 0.0f;
	hierarchy.backTime = driving(type) ? BackTurn : 0.0f;
	hierarchy.arcs.clear();
	for (uint32_t junction = 0; junction != junctions; ++junction) {
		hierarchy.cost[states + junction] = 0.0f;
	}
	for (uint32_t from = 0; from != states && junctions != 0; ++from) {
		if (hierarchy.cost[from] == Infinite) {
			continue;
		}
		Arc arrive;
		arrive.source = from;
		arrive.target = states + endOf(from);
		arrive.cost = hierarchy.cost[from];
		arrive.length = chains_[from >> 1].length;
		arrive.time = hierarchy.time[from];
		hierarchy.arcs.push_back(arrive);
		Arc leave;
		leave.source = states + startOf(from);
		leave.target = from;
		hierarchy.arcs.push_back(leave);
	}
	for (uint32_t from = 0; from != states && junctions == 0; ++from) {
		if (hierarchy.cost[from] == Infinite) {
			continue;
		}
		uint32_t junction = endOf(from);
		for (uint32_t index = startFirst_[junction]; index != startFirst_[junction + 1]; ++index) {
			uint32_t to = startStates_[index];
			if (to == from || hierarchy.cost[to] == Infinite || (to == (from ^ 1) && degrees_[junctionNodes_[junction]] == 2)) {
				continue;
			}
			float seconds;
			float turn = turnCost(type, from, to, seconds);
			Arc arc;
			arc.source = from;
			arc.target = to;
			arc.cost = hierarchy.cost[from] + turn;
			arc.length = chains_[from >> 1].length;
			arc.time = hierarchy.time[from] + seconds;
			hierarchy.arcs.push_back(arc);
		}
	}
	hierarchy.outFirst.assign(states + junctions + 1, 0);
	for (const Arc& arc : hierarchy.arcs) {
		++hierarchy.outFirst[arc.source + 1];
	}
	for (uint32_t state = 0; state != states + junctions; ++state) {
		hierarchy.outFirst[state + 1] += hierarchy.outFirst[state];
	}
	hierarchy.outArcs.assign(hierarchy.arcs.size(), 0);
	std::vector<uint32_t> fill(hierarchy.outFirst.begin(), hierarchy.outFirst.end() - 1);
	for (uint32_t index = 0; index != uint32_t(hierarchy.arcs.size()); ++index) {
		hierarchy.outArcs[fill[hierarchy.arcs[index].source]++] = index;
	}
	buildComponents(hierarchy);
}

void RouteNetwork::buildComponents(Hierarchy& hierarchy) {
	struct Frame {
		uint32_t node;
		uint32_t next;
	};
	uint32_t count = uint32_t(hierarchy.cost.size());
	hierarchy.component.assign(count, None);
	std::vector<uint32_t> order(count, None);
	std::vector<uint32_t> low(count, 0);
	std::vector<uint8_t> held(count, 0);
	std::vector<uint32_t> stack;
	std::vector<Frame> frames;
	std::vector<size_t> sizes;
	uint32_t counter = 0;
	for (uint32_t root = 0; root != count; ++root) {
		if (hierarchy.cost[root] == Infinite || order[root] != None) {
			continue;
		}
		order[root] = low[root] = counter++;
		stack.push_back(root);
		held[root] = 1;
		frames.push_back({ root, hierarchy.outFirst[root] });
		while (!frames.empty()) {
			uint32_t node = frames.back().node;
			if (frames.back().next != hierarchy.outFirst[node + 1]) {
				uint32_t next = hierarchy.arcs[hierarchy.outArcs[frames.back().next++]].target;
				if (order[next] == None) {
					order[next] = low[next] = counter++;
					stack.push_back(next);
					held[next] = 1;
					frames.push_back({ next, hierarchy.outFirst[next] });
				} else if (held[next] != 0) {
					low[node] = std::min(low[node], order[next]);
				}
				continue;
			}
			if (low[node] == order[node]) {
				uint32_t component = uint32_t(sizes.size());
				size_t size = 0;
				uint32_t member;
				do {
					member = stack.back();
					stack.pop_back();
					held[member] = 0;
					hierarchy.component[member] = component;
					++size;
				} while (member != node);
				sizes.push_back(size);
			}
			frames.pop_back();
			if (!frames.empty()) {
				uint32_t parent = frames.back().node;
				low[parent] = std::min(low[parent], low[node]);
			}
		}
	}
	auto largest = std::max_element(sizes.begin(), sizes.end());
	hierarchy.main = largest == sizes.end() ? None : uint32_t(largest - sizes.begin());
	hierarchy.mainSize = largest == sizes.end() ? 0 : *largest;
}

bool RouteNetwork::contract(const Hierarchy& hierarchy, Contraction& contraction) {
	struct Link {
		uint32_t node;
		uint32_t arc;
	};
	struct Pending {
		uint32_t from;
		uint32_t to;
		uint32_t first;
		uint32_t second;
		float cost;
	};
	std::vector<Arc>& arcs = contraction.arcs;
	arcs = hierarchy.arcs;
	uint32_t count = uint32_t(hierarchy.cost.size());
	std::vector<std::vector<Link>> outs(count);
	std::vector<std::vector<Link>> ins(count);
	for (uint32_t index = 0; index != uint32_t(arcs.size()); ++index) {
		outs[arcs[index].source].push_back({ arcs[index].target, index });
		ins[arcs[index].target].push_back({ arcs[index].source, index });
	}
	std::vector<uint8_t> done(count, 0);
	std::vector<uint32_t> rank(count, None);
	std::vector<int> removed(count, 0);
	std::vector<int> level(count, 0);
	std::vector<float> reached(count, Infinite);
	std::vector<uint32_t> touched;
	std::vector<Entry> heap;
	auto witness = [&](uint32_t source, uint32_t skip, float limit, int settles) {
		for (uint32_t node : touched) {
			reached[node] = Infinite;
		}
		touched.clear();
		heap.clear();
		reached[source] = 0.0f;
		touched.push_back(source);
		push(heap, 0.0f, source);
		int settled = 0;
		while (!heap.empty()) {
			Entry top = pop(heap);
			if (top.first > reached[top.second]) {
				continue;
			}
			if (top.first > limit || ++settled > settles) {
				break;
			}
			for (const Link& link : outs[top.second]) {
				if (link.node == skip) {
					continue;
				}
				float next = top.first + arcs[link.arc].cost;
				if (next < reached[link.node]) {
					if (reached[link.node] == Infinite) {
						touched.push_back(link.node);
					}
					reached[link.node] = next;
					push(heap, next, link.node);
				}
			}
		}
	};
	auto shortcuts = [&](uint32_t node, int settles, std::vector<Pending>* found) {
		int needed = 0;
		float farthest = 0.0f;
		for (const Link& out : outs[node]) {
			farthest = std::max(farthest, arcs[out.arc].cost);
		}
		for (const Link& in : ins[node]) {
			float before = arcs[in.arc].cost;
			witness(in.node, node, before + farthest, settles);
			for (const Link& out : outs[node]) {
				float through = before + arcs[out.arc].cost;
				if (out.node == in.node || reached[out.node] <= through) {
					continue;
				}
				++needed;
				if (found != nullptr) {
					found->push_back({ in.node, out.node, in.arc, out.arc, through });
				}
			}
		}
		return needed;
	};
	auto priority = [&](uint32_t node) {
		int edges = int(ins[node].size() + outs[node].size());
		return 2 * (shortcuts(node, SimulatedSettles, nullptr) - edges) + removed[node] + level[node];
	};
	auto add = [&](const Pending& pending) {
		Link* existing = nullptr;
		for (Link& link : outs[pending.from]) {
			if (link.node == pending.to) {
				existing = &link;
				break;
			}
		}
		if (existing != nullptr && arcs[existing->arc].cost <= pending.cost) {
			return;
		}
		Arc arc;
		arc.source = pending.from;
		arc.target = pending.to;
		arc.cost = pending.cost;
		arc.length = arcs[pending.first].length + arcs[pending.second].length;
		arc.time = arcs[pending.first].time + arcs[pending.second].time;
		arc.first = pending.first;
		arc.second = pending.second;
		uint32_t index = uint32_t(arcs.size());
		arcs.push_back(arc);
		if (existing == nullptr) {
			outs[pending.from].push_back({ pending.to, index });
			ins[pending.to].push_back({ pending.from, index });
			return;
		}
		uint32_t replaced = existing->arc;
		existing->arc = index;
		for (Link& back : ins[pending.to]) {
			if (back.node == pending.from && back.arc == replaced) {
				back.arc = index;
			}
		}
	};
	using Ranked = std::pair<int, uint32_t>;
	std::vector<Ranked> queue;
	for (uint32_t node = 0; node != count; ++node) {
		if (hierarchy.cost[node] != Infinite) {
			queue.emplace_back(priority(node), node);
		}
	}
	std::make_heap(queue.begin(), queue.end(), std::greater<Ranked>());
	std::vector<Pending> pending;
	uint32_t order = 0;
	while (!queue.empty()) {
		if (stopping_.load(std::memory_order_relaxed)) {
			return false;
		}
		std::pop_heap(queue.begin(), queue.end(), std::greater<Ranked>());
		uint32_t node = queue.back().second;
		queue.pop_back();
		if (done[node] != 0) {
			continue;
		}
		int current = priority(node);
		if (!queue.empty() && current > queue.front().first) {
			queue.emplace_back(current, node);
			std::push_heap(queue.begin(), queue.end(), std::greater<Ranked>());
			continue;
		}
		pending.clear();
		shortcuts(node, WitnessSettles, &pending);
		for (const Pending& shortcut : pending) {
			add(shortcut);
		}
		done[node] = 1;
		rank[node] = order++;
		auto detach = [node](std::vector<Link>& links) {
			links.erase(std::remove_if(links.begin(), links.end(), [node](const Link& link) {
				return link.node == node;
			}), links.end());
		};
		for (const Link& link : ins[node]) {
			removed[link.node] += 1;
			level[link.node] = std::max(level[link.node], level[node] + 1);
			detach(outs[link.node]);
		}
		for (const Link& link : outs[node]) {
			removed[link.node] += 1;
			level[link.node] = std::max(level[link.node], level[node] + 1);
			detach(ins[link.node]);
		}
	}
	contraction.upFirst.assign(count + 1, 0);
	contraction.downFirst.assign(count + 1, 0);
	for (const Arc& arc : arcs) {
		if (rank[arc.target] > rank[arc.source]) {
			++contraction.upFirst[arc.source + 1];
		} else {
			++contraction.downFirst[arc.target + 1];
		}
	}
	for (uint32_t node = 0; node != count; ++node) {
		contraction.upFirst[node + 1] += contraction.upFirst[node];
		contraction.downFirst[node + 1] += contraction.downFirst[node];
	}
	contraction.up.assign(contraction.upFirst[count], Step());
	contraction.down.assign(contraction.downFirst[count], Step());
	std::vector<uint32_t> upFill(contraction.upFirst.begin(), contraction.upFirst.end() - 1);
	std::vector<uint32_t> downFill(contraction.downFirst.begin(), contraction.downFirst.end() - 1);
	for (uint32_t index = 0; index != uint32_t(arcs.size()); ++index) {
		const Arc& arc = arcs[index];
		if (rank[arc.target] > rank[arc.source]) {
			contraction.up[upFill[arc.source]++] = { arc.target, arc.cost, index };
		} else {
			contraction.down[downFill[arc.target]++] = { arc.source, arc.cost, index };
		}
	}
	return true;
}

void RouteNetwork::buildGrid() {
	gridColumns_ = 0;
	gridRows_ = 0;
	gridFirst_.clear();
	gridSegments_.clear();
	if (chains_.empty()) {
		return;
	}
	float minX = Infinite;
	float minY = Infinite;
	float maxX = -Infinite;
	float maxY = -Infinite;
	for (uint32_t node : chainNodes_) {
		minX = std::min(minX, nodes_[node].x);
		minY = std::min(minY, nodes_[node].y);
		maxX = std::max(maxX, nodes_[node].x);
		maxY = std::max(maxY, nodes_[node].y);
	}
	gridMinX_ = minX;
	gridMinY_ = minY;
	gridColumns_ = int((maxX - minX) / GridCell) + 1;
	gridRows_ = int((maxY - minY) / GridCell) + 1;
	gridFirst_.assign(size_t(gridColumns_) * size_t(gridRows_) + 1, 0);
	std::vector<uint32_t> fill;
	for (int pass = 0; pass != 2; ++pass) {
		if (pass == 1) {
			for (size_t cell = 0; cell + 1 < gridFirst_.size(); ++cell) {
				gridFirst_[cell + 1] += gridFirst_[cell];
			}
			gridSegments_.assign(gridFirst_.back(), 0);
			fill.assign(gridFirst_.begin(), gridFirst_.end() - 1);
		}
		for (const Chain& chain : chains_) {
			for (uint32_t position = chain.first; position + 1 < chain.first + chain.count; ++position) {
				const PathNode& a = nodes_[chainNodes_[position]];
				const PathNode& b = nodes_[chainNodes_[position + 1]];
				int left = int((std::min(a.x, b.x) - gridMinX_) / GridCell);
				int right = int((std::max(a.x, b.x) - gridMinX_) / GridCell);
				int bottom = int((std::min(a.y, b.y) - gridMinY_) / GridCell);
				int top = int((std::max(a.y, b.y) - gridMinY_) / GridCell);
				for (int row = bottom; row <= top; ++row) {
					for (int column = left; column <= right; ++column) {
						size_t cell = size_t(row) * size_t(gridColumns_) + size_t(column);
						if (pass == 0) {
							++gridFirst_[cell + 1];
						} else {
							gridSegments_[fill[cell]++] = position;
						}
					}
				}
			}
		}
	}
}

bool RouteNetwork::nearest(float x, float y, float z, float reach, RouteType type, bool connected, NetworkPlace& place) const {
	if (!built()) {
		return false;
	}
	const Hierarchy& found = hierarchy(type);
	if (!found.used || gridColumns_ == 0 || !(reach > 0.0f)) {
		return false;
	}
	auto fits = [&](uint32_t chain) {
		uint32_t forward = chain * 2;
		bool ahead = found.cost[forward] != Infinite;
		bool back = found.cost[forward + 1] != Infinite;
		if (!connected) {
			return ahead || back;
		}
		return (ahead && found.component[forward] == found.main) || (back && found.component[forward + 1] == found.main);
	};
	float best = Infinite;
	uint32_t bestPosition = None;
	float bestT = 0.0f;
	float radius = std::min(GridCell, reach);
	while (true) {
		int left = std::max(int(std::floor((x - radius - gridMinX_) / GridCell)), 0);
		int right = std::min(int(std::floor((x + radius - gridMinX_) / GridCell)), gridColumns_ - 1);
		int bottom = std::max(int(std::floor((y - radius - gridMinY_) / GridCell)), 0);
		int top = std::min(int(std::floor((y + radius - gridMinY_) / GridCell)), gridRows_ - 1);
		for (int row = bottom; row <= top; ++row) {
			for (int column = left; column <= right; ++column) {
				size_t cell = size_t(row) * size_t(gridColumns_) + size_t(column);
				for (uint32_t index = gridFirst_[cell]; index != gridFirst_[cell + 1]; ++index) {
					uint32_t position = gridSegments_[index];
					if (!fits(segmentChains_[position])) {
						continue;
					}
					const PathNode& a = nodes_[chainNodes_[position]];
					const PathNode& b = nodes_[chainNodes_[position + 1]];
					float dx = b.x - a.x;
					float dy = b.y - a.y;
					float dz = (b.z - a.z) * 2.0f;
					float span = dx * dx + dy * dy + dz * dz;
					float t = span > 0.0f ? ((x - a.x) * dx + (y - a.y) * dy + (z - a.z) * 2.0f * dz) / span : 0.0f;
					t = std::clamp(t, 0.0f, 1.0f);
					float px = a.x + dx * t - x;
					float py = a.y + dy * t - y;
					float pz = (a.z - z) * 2.0f + dz * t;
					float squared = px * px + py * py + pz * pz;
					if (squared < best) {
						best = squared;
						bestPosition = position;
						bestT = t;
					}
				}
			}
		}
		if ((bestPosition != None && best <= radius * radius) || radius >= reach) {
			break;
		}
		radius = std::min(radius * 2.0f, reach);
	}
	if (bestPosition == None || best > reach * reach) {
		return false;
	}
	const PathNode& a = nodes_[chainNodes_[bestPosition]];
	const PathNode& b = nodes_[chainNodes_[bestPosition + 1]];
	place.chain = segmentChains_[bestPosition];
	place.offset = chainOffsets_[bestPosition] + (chainOffsets_[bestPosition + 1] - chainOffsets_[bestPosition]) * bestT;
	place.point = { a.x + (b.x - a.x) * bestT, a.y + (b.y - a.y) * bestT, a.z + (b.z - a.z) * bestT };
	float length = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
	place.directionX = length > 0.0f ? (b.x - a.x) / length : 0.0f;
	place.directionY = length > 0.0f ? (b.y - a.y) / length : 0.0f;
	return true;
}

bool RouteNetwork::connects(const NetworkPlace& place, RouteType type) const {
	if (!built() || place.chain >= chains_.size()) {
		return false;
	}
	const Hierarchy& found = hierarchy(type);
	if (!found.used) {
		return false;
	}
	uint32_t forward = place.chain * 2;
	return (found.cost[forward] != Infinite && found.component[forward] == found.main) || (found.cost[forward + 1] != Infinite && found.component[forward + 1] == found.main);
}

float RouteNetwork::headingAt(const NetworkPlace& place, RouteType type) const {
	if (!built()) {
		return 0.0f;
	}
	const Hierarchy& found = hierarchy(type);
	float x = place.directionX;
	float y = place.directionY;
	if (place.chain < chains_.size() && found.used && found.cost[place.chain * 2] == Infinite && found.cost[place.chain * 2 + 1] != Infinite) {
		x = -x;
		y = -y;
	}
	float angle = std::atan2(-x, y) * Degrees;
	return angle < 0.0f ? angle + 360.0f : angle;
}

void RouteNetwork::ends(const NetworkPlace& from, const NetworkPlace& to, const Hierarchy& hierarchy, RouteType type, float heading, Ends& ends) const {
	ends.sources.clear();
	ends.targets.clear();
	ends.hasDirect = false;
	ends.direct = Infinite;
	bool facing = heading >= 0.0f && driving(type);
	float headingX = -std::sin(heading / Degrees);
	float headingY = std::cos(heading / Degrees);
	float along = facing ? from.directionX * headingX + from.directionY * headingY : 0.0f;
	auto penalty = [&](bool reverse, float& seconds) {
		bool against = facing && (reverse ? along > Opposed : along < -Opposed);
		seconds = against ? hierarchy.backTime : 0.0f;
		return against ? hierarchy.back : 0.0f;
	};
	const Chain& source = chains_[from.chain];
	float done = source.length > 0.0f ? std::clamp(from.offset / source.length, 0.0f, 1.0f) : 0.0f;
	for (uint32_t reverse = 0; reverse != 2; ++reverse) {
		uint32_t state = from.chain * 2 + reverse;
		if (hierarchy.cost[state] == Infinite) {
			continue;
		}
		float rest = reverse != 0 ? done : 1.0f - done;
		float seconds;
		float extra = penalty(reverse != 0, seconds);
		for (uint32_t index = hierarchy.outFirst[state]; index != hierarchy.outFirst[state + 1]; ++index) {
			uint32_t arcIndex = hierarchy.outArcs[index];
			const Arc& arc = hierarchy.arcs[arcIndex];
			Seed seed;
			seed.state = arc.target;
			seed.cost = hierarchy.cost[state] * rest + extra + (arc.cost - hierarchy.cost[state]);
			seed.length = source.length * rest;
			seed.time = hierarchy.time[state] * rest + seconds + (arc.time - hierarchy.time[state]);
			seed.arc = arcIndex;
			ends.sources.push_back(seed);
		}
	}
	const Chain& target = chains_[to.chain];
	float reached = target.length > 0.0f ? std::clamp(to.offset / target.length, 0.0f, 1.0f) : 0.0f;
	for (uint32_t reverse = 0; reverse != 2; ++reverse) {
		uint32_t state = to.chain * 2 + reverse;
		if (hierarchy.cost[state] == Infinite) {
			continue;
		}
		float part = reverse != 0 ? 1.0f - reached : reached;
		Seed seed;
		seed.state = state;
		seed.cost = hierarchy.cost[state] * part;
		seed.length = target.length * part;
		seed.time = hierarchy.time[state] * part;
		ends.targets.push_back(seed);
		if (from.chain != to.chain || (reverse != 0 ? to.offset > from.offset : to.offset < from.offset)) {
			continue;
		}
		float span = std::abs(reached - done);
		float seconds;
		float cost = hierarchy.cost[state] * span + penalty(reverse != 0, seconds);
		if (cost < ends.direct) {
			ends.hasDirect = true;
			ends.direct = cost;
			ends.directLength = target.length * span;
			ends.directTime = hierarchy.time[state] * span + seconds;
			ends.directForward = reverse == 0;
		}
	}
}

bool RouteNetwork::query(const Contraction& contraction, const Ends& ends, Found& found) const {
	uint32_t forwardMark = ++stamp_;
	uint32_t backwardMark = ++stamp_;
	std::vector<Entry>& forward = forwardHeap_;
	std::vector<Entry>& backward = backwardHeap_;
	forward.clear();
	backward.clear();
	for (uint32_t index = 0; index != uint32_t(ends.sources.size()); ++index) {
		const Seed& seed = ends.sources[index];
		if (forwardStamp_[seed.state] == forwardMark && forwardCost_[seed.state] <= seed.cost) {
			continue;
		}
		forwardStamp_[seed.state] = forwardMark;
		forwardCost_[seed.state] = seed.cost;
		forwardParent_[seed.state] = SeedMark | index;
		push(forward, seed.cost, seed.state);
	}
	for (uint32_t index = 0; index != uint32_t(ends.targets.size()); ++index) {
		const Seed& seed = ends.targets[index];
		if (backwardStamp_[seed.state] == backwardMark && backwardCost_[seed.state] <= seed.cost) {
			continue;
		}
		backwardStamp_[seed.state] = backwardMark;
		backwardCost_[seed.state] = seed.cost;
		backwardParent_[seed.state] = SeedMark | index;
		push(backward, seed.cost, seed.state);
	}
	float best = ends.hasDirect ? ends.direct : Infinite;
	uint32_t meet = None;
	while (true) {
		float forwardTop = forward.empty() ? Infinite : forward.front().first;
		float backwardTop = backward.empty() ? Infinite : backward.front().first;
		if (std::min(forwardTop, backwardTop) >= best) {
			break;
		}
		bool ahead = forwardTop <= backwardTop;
		Entry top = pop(ahead ? forward : backward);
		uint32_t node = top.second;
		if (ahead) {
			if (top.first > forwardCost_[node]) {
				continue;
			}
			if (backwardStamp_[node] == backwardMark && top.first + backwardCost_[node] < best) {
				best = top.first + backwardCost_[node];
				meet = node;
			}
			bool stalled = false;
			for (uint32_t index = contraction.downFirst[node]; index != contraction.downFirst[node + 1] && !stalled; ++index) {
				const Step& step = contraction.down[index];
				stalled = forwardStamp_[step.node] == forwardMark && forwardCost_[step.node] + step.cost < top.first;
			}
			if (stalled) {
				continue;
			}
			for (uint32_t index = contraction.upFirst[node]; index != contraction.upFirst[node + 1]; ++index) {
				const Step& step = contraction.up[index];
				float next = top.first + step.cost;
				if (forwardStamp_[step.node] != forwardMark || next < forwardCost_[step.node]) {
					forwardStamp_[step.node] = forwardMark;
					forwardCost_[step.node] = next;
					forwardParent_[step.node] = step.arc;
					push(forward, next, step.node);
				}
			}
		} else {
			if (top.first > backwardCost_[node]) {
				continue;
			}
			if (forwardStamp_[node] == forwardMark && top.first + forwardCost_[node] < best) {
				best = top.first + forwardCost_[node];
				meet = node;
			}
			bool stalled = false;
			for (uint32_t index = contraction.upFirst[node]; index != contraction.upFirst[node + 1] && !stalled; ++index) {
				const Step& step = contraction.up[index];
				stalled = backwardStamp_[step.node] == backwardMark && backwardCost_[step.node] + step.cost < top.first;
			}
			if (stalled) {
				continue;
			}
			for (uint32_t index = contraction.downFirst[node]; index != contraction.downFirst[node + 1]; ++index) {
				const Step& step = contraction.down[index];
				float next = top.first + step.cost;
				if (backwardStamp_[step.node] != backwardMark || next < backwardCost_[step.node]) {
					backwardStamp_[step.node] = backwardMark;
					backwardCost_[step.node] = next;
					backwardParent_[step.node] = step.arc;
					push(backward, next, step.node);
				}
			}
		}
	}
	found.cost = best;
	found.arcs.clear();
	found.source = nullptr;
	found.target = nullptr;
	found.direct = meet == None;
	if (meet == None) {
		return ends.hasDirect;
	}
	uint32_t node = meet;
	while ((forwardParent_[node] & SeedMark) == 0) {
		uint32_t arc = forwardParent_[node];
		found.arcs.push_back(arc);
		node = contraction.arcs[arc].source;
	}
	found.source = &ends.sources[forwardParent_[node] & ~SeedMark];
	std::reverse(found.arcs.begin(), found.arcs.end());
	node = meet;
	while ((backwardParent_[node] & SeedMark) == 0) {
		uint32_t arc = backwardParent_[node];
		found.arcs.push_back(arc);
		node = contraction.arcs[arc].target;
	}
	found.target = &ends.targets[backwardParent_[node] & ~SeedMark];
	return true;
}

bool RouteNetwork::searchStates(const Hierarchy& hierarchy, const Ends& ends, const RoutePoint& target, Found& found) const {
	uint32_t mark = ++stamp_;
	std::vector<Entry>& open = forwardHeap_;
	open.clear();
	uint32_t states = uint32_t(chains_.size() * 2);
	auto estimate = [&](uint32_t state) {
		return distance(junctionPoint(state < states ? startOf(state) : state - states), target) * hierarchy.least;
	};
	for (uint32_t index = 0; index != uint32_t(ends.sources.size()); ++index) {
		const Seed& seed = ends.sources[index];
		if (forwardStamp_[seed.state] == mark && forwardCost_[seed.state] <= seed.cost) {
			continue;
		}
		forwardStamp_[seed.state] = mark;
		forwardCost_[seed.state] = seed.cost;
		forwardParent_[seed.state] = SeedMark | index;
		push(open, seed.cost + estimate(seed.state), seed.state);
	}
	float best = ends.hasDirect ? ends.direct : Infinite;
	uint32_t last = None;
	uint32_t lastTarget = None;
	while (!open.empty()) {
		Entry top = pop(open);
		if (top.first >= best) {
			break;
		}
		uint32_t node = top.second;
		float cost = forwardCost_[node];
		if (top.first > cost + estimate(node) + 1e-3f) {
			continue;
		}
		for (uint32_t index = 0; index != uint32_t(ends.targets.size()); ++index) {
			if (ends.targets[index].state == node && cost + ends.targets[index].cost < best) {
				best = cost + ends.targets[index].cost;
				last = node;
				lastTarget = index;
			}
		}
		for (uint32_t index = hierarchy.outFirst[node]; index != hierarchy.outFirst[node + 1]; ++index) {
			const Arc& arc = hierarchy.arcs[hierarchy.outArcs[index]];
			float next = cost + arc.cost;
			if (forwardStamp_[arc.target] != mark || next < forwardCost_[arc.target]) {
				forwardStamp_[arc.target] = mark;
				forwardCost_[arc.target] = next;
				forwardParent_[arc.target] = hierarchy.outArcs[index];
				push(open, next + estimate(arc.target), arc.target);
			}
		}
	}
	found.cost = best;
	found.arcs.clear();
	found.source = nullptr;
	found.target = nullptr;
	found.direct = last == None;
	if (last == None) {
		return ends.hasDirect;
	}
	uint32_t node = last;
	while ((forwardParent_[node] & SeedMark) == 0) {
		uint32_t arc = forwardParent_[node];
		found.arcs.push_back(arc);
		node = hierarchy.arcs[arc].source;
	}
	found.source = &ends.sources[forwardParent_[node] & ~SeedMark];
	found.target = &ends.targets[lastTarget];
	std::reverse(found.arcs.begin(), found.arcs.end());
	return true;
}

void RouteNetwork::unpack(const std::vector<Arc>& arcs, uint32_t arc, std::vector<uint32_t>& unpacked) const {
	std::vector<uint32_t> stack(1, arc);
	while (!stack.empty()) {
		uint32_t top = stack.back();
		stack.pop_back();
		const Arc& found = arcs[top];
		if (found.first == None) {
			unpacked.push_back(top);
			continue;
		}
		stack.push_back(found.second);
		stack.push_back(found.first);
	}
}

void RouteNetwork::addPoints(uint32_t chain, float start, float end, std::vector<RoutePoint>& points) const {
	auto add = [&](const RoutePoint& point) {
		if (!points.empty()) {
			const RoutePoint& last = points.back();
			if (std::abs(last.x - point.x) < 0.01f && std::abs(last.y - point.y) < 0.01f && std::abs(last.z - point.z) < 0.01f) {
				return;
			}
		}
		points.push_back(point);
	};
	const Chain& found = chains_[chain];
	const float* offsets = chainOffsets_.data() + found.first;
	add(pointAt(chain, start));
	if (start <= end) {
		for (uint32_t index = uint32_t(std::upper_bound(offsets, offsets + found.count, start) - offsets); index < found.count && offsets[index] < end; ++index) {
			const PathNode& node = nodes_[chainNodes_[found.first + index]];
			add({ node.x, node.y, node.z });
		}
	} else {
		for (uint32_t index = uint32_t(std::lower_bound(offsets, offsets + found.count, start) - offsets); index-- != 0 && offsets[index] > end;) {
			const PathNode& node = nodes_[chainNodes_[found.first + index]];
			add({ node.x, node.y, node.z });
		}
	}
	add(pointAt(chain, end));
}

void RouteNetwork::assemble(const NetworkPlace& from, const NetworkPlace& to, RouteType type, const Hierarchy& hierarchy, const Ends& ends, const Found& found, float heading, RouteResult& result) const {
	result.points.clear();
	result.turns.clear();
	result.cost = found.cost;
	bool facing = heading >= 0.0f && driving(type);
	float headingX = -std::sin(heading / Degrees);
	float headingY = std::cos(heading / Degrees);
	float along = facing ? from.directionX * headingX + from.directionY * headingY : 0.0f;
	auto against = [&](bool reverse) {
		return facing && (reverse ? along > Opposed : along < -Opposed);
	};
	if (found.direct) {
		addPoints(from.chain, from.offset, to.offset, result.points);
		result.length = ends.directLength;
		result.time = ends.directTime;
		if (against(!ends.directForward)) {
			result.turns.push_back({ 0, 180.0f });
		}
		return;
	}
	const Contraction* contraction = contractionOf(hierarchy);
	const std::vector<Arc>& arcs = contraction != nullptr ? contraction->arcs : hierarchy.arcs;
	std::vector<uint32_t> unpacked;
	for (uint32_t arc : found.arcs) {
		unpack(arcs, arc, unpacked);
	}
	uint32_t departure = arcs[found.source->arc].source;
	const Chain& first = chains_[from.chain];
	addPoints(from.chain, from.offset, (departure & 1) != 0 ? 0.0f : first.length, result.points);
	if (against((departure & 1) != 0)) {
		result.turns.push_back({ 0, 180.0f });
	}
	struct Passage {
		uint32_t point;
		uint32_t from;
		uint32_t to;
	};
	std::vector<Passage> passages;
	uint32_t states = uint32_t(chains_.size() * 2);
	uint32_t previous = departure;
	uint32_t current = found.source->state;
	result.length = found.source->length;
	result.time = found.source->time;
	size_t next = 0;
	while (true) {
		if (current < states) {
			passages.push_back({ uint32_t(result.points.size() - 1), previous, current });
			previous = current;
		}
		if (next == unpacked.size()) {
			break;
		}
		const Arc& arc = arcs[unpacked[next++]];
		if (current < states) {
			const Chain& chain = chains_[current >> 1];
			addPoints(current >> 1, (current & 1) != 0 ? chain.length : 0.0f, (current & 1) != 0 ? 0.0f : chain.length, result.points);
		}
		result.length += arc.length;
		result.time += arc.time;
		current = arc.target;
	}
	const Chain& last = chains_[to.chain];
	addPoints(to.chain, (current & 1) != 0 ? last.length : 0.0f, to.offset, result.points);
	result.length += found.target->length;
	result.time += found.target->time;
	std::vector<float> walked(result.points.size(), 0.0f);
	for (size_t index = 1; index < result.points.size(); ++index) {
		walked[index] = walked[index - 1] + distance(result.points[index - 1], result.points[index]);
	}
	for (const Passage& passage : passages) {
		if (passage.from == passage.to) {
			continue;
		}
		bool back = passage.to == (passage.from ^ 1);
		if (!back && degrees_[junctionNodes_[endOf(passage.from)]] <= 2) {
			continue;
		}
		float angle = turnAngle(passage.from, passage.to);
		if (!result.turns.empty() && walked[passage.point] - walked[result.turns.back().point] < TurnMerge) {
			result.turns.back().angle += angle;
			continue;
		}
		result.turns.push_back({ passage.point, angle });
	}
	result.turns.erase(std::remove_if(result.turns.begin(), result.turns.end(), [](const RouteTurn& turn) {
		return std::abs(turn.angle) < TurnShown;
	}), result.turns.end());
	for (RouteTurn& turn : result.turns) {
		turn.angle = std::clamp(turn.angle, -180.0f, 180.0f);
	}
}

bool RouteNetwork::route(const NetworkPlace& from, const NetworkPlace& to, RouteType type, float heading, RouteResult& result) const {
	if (!built()) {
		return false;
	}
	const Hierarchy& found = hierarchy(type);
	if (!found.used || from.chain >= chains_.size() || to.chain >= chains_.size()) {
		return false;
	}
	Ends edges;
	ends(from, to, found, type, heading, edges);
	Found path;
	const Contraction* contraction = contractionOf(found);
	if (!(contraction != nullptr ? query(*contraction, edges, path) : searchStates(found, edges, to.point, path))) {
		return false;
	}
	assemble(from, to, type, found, edges, path, heading, result);
	return true;
}

bool RouteNetwork::search(const NetworkPlace& from, const NetworkPlace& to, RouteType type, float heading, RouteResult& result) const {
	if (!built()) {
		return false;
	}
	const Hierarchy& found = hierarchy(type);
	if (!found.used || from.chain >= chains_.size() || to.chain >= chains_.size()) {
		return false;
	}
	Ends edges;
	ends(from, to, found, type, heading, edges);
	Found path;
	if (!searchStates(found, edges, to.point, path)) {
		return false;
	}
	assemble(from, to, type, found, edges, path, heading, result);
	return true;
}

bool RouteNetwork::measure(const NetworkPlace& from, const NetworkPlace& to, RouteType type, float heading, float& length, float& time) const {
	if (!built()) {
		return false;
	}
	const Hierarchy& found = hierarchy(type);
	if (!found.used || from.chain >= chains_.size() || to.chain >= chains_.size()) {
		return false;
	}
	Ends edges;
	ends(from, to, found, type, heading, edges);
	Found path;
	const Contraction* contraction = contractionOf(found);
	if (!(contraction != nullptr ? query(*contraction, edges, path) : searchStates(found, edges, to.point, path))) {
		return false;
	}
	if (path.direct) {
		length = edges.directLength;
		time = edges.directTime;
		return true;
	}
	const std::vector<Arc>& arcs = contraction != nullptr ? contraction->arcs : found.arcs;
	length = path.source->length + path.target->length;
	time = path.source->time + path.target->time;
	for (uint32_t arc : path.arcs) {
		length += arcs[arc].length;
		time += arcs[arc].time;
	}
	return true;
}

int RouteNetwork::closest(const NetworkPlace& from, const std::vector<NetworkPlace>& targets, RouteType type, float heading, float& cost) const {
	cost = Infinite;
	if (!built()) {
		return -1;
	}
	const Hierarchy& found = hierarchy(type);
	if (!found.used || from.chain >= chains_.size()) {
		return -1;
	}
	Ends edges;
	ends(from, from, found, type, heading, edges);
	const Contraction* contraction = contractionOf(found);
	uint32_t forwardMark = ++stamp_;
	std::vector<Entry>& open = forwardHeap_;
	open.clear();
	for (const Seed& seed : edges.sources) {
		if (forwardStamp_[seed.state] == forwardMark && forwardCost_[seed.state] <= seed.cost) {
			continue;
		}
		forwardStamp_[seed.state] = forwardMark;
		forwardCost_[seed.state] = seed.cost;
		push(open, seed.cost, seed.state);
	}
	float best = Infinite;
	int bestIndex = -1;
	Ends goal;
	if (contraction == nullptr) {
		struct Goal {
			uint32_t state;
			int target;
			float cost;
		};
		auto earlier = [](const Goal& a, const Goal& b) {
			return a.state < b.state;
		};
		std::vector<Goal> goals;
		for (int target = 0; target != int(targets.size()); ++target) {
			const NetworkPlace& place = targets[size_t(target)];
			if (place.chain >= chains_.size()) {
				continue;
			}
			ends(from, place, found, type, heading, goal);
			for (const Seed& seed : goal.targets) {
				goals.push_back({ seed.state, target, seed.cost });
			}
			if (goal.hasDirect && goal.direct < best) {
				best = goal.direct;
				bestIndex = target;
			}
		}
		std::sort(goals.begin(), goals.end(), earlier);
		while (!open.empty()) {
			Entry top = pop(open);
			if (top.first >= best) {
				break;
			}
			if (top.first > forwardCost_[top.second]) {
				continue;
			}
			auto range = std::equal_range(goals.begin(), goals.end(), Goal{ top.second, 0, 0.0f }, earlier);
			for (auto at = range.first; at != range.second; ++at) {
				if (top.first + at->cost < best) {
					best = top.first + at->cost;
					bestIndex = at->target;
				}
			}
			for (uint32_t index = found.outFirst[top.second]; index != found.outFirst[top.second + 1]; ++index) {
				const Arc& arc = found.arcs[found.outArcs[index]];
				float next = top.first + arc.cost;
				if (forwardStamp_[arc.target] != forwardMark || next < forwardCost_[arc.target]) {
					forwardStamp_[arc.target] = forwardMark;
					forwardCost_[arc.target] = next;
					push(open, next, arc.target);
				}
			}
		}
		cost = best;
		return bestIndex;
	}
	while (!open.empty()) {
		Entry top = pop(open);
		if (top.first > forwardCost_[top.second]) {
			continue;
		}
		for (uint32_t index = contraction->upFirst[top.second]; index != contraction->upFirst[top.second + 1]; ++index) {
			const Step& step = contraction->up[index];
			float next = top.first + step.cost;
			if (forwardStamp_[step.node] != forwardMark || next < forwardCost_[step.node]) {
				forwardStamp_[step.node] = forwardMark;
				forwardCost_[step.node] = next;
				push(open, next, step.node);
			}
		}
	}
	for (int target = 0; target != int(targets.size()); ++target) {
		const NetworkPlace& place = targets[size_t(target)];
		if (place.chain >= chains_.size()) {
			continue;
		}
		ends(from, place, found, type, heading, goal);
		float reached = goal.hasDirect ? goal.direct : Infinite;
		uint32_t backwardMark = ++stamp_;
		open.clear();
		for (const Seed& seed : goal.targets) {
			if (backwardStamp_[seed.state] == backwardMark && backwardCost_[seed.state] <= seed.cost) {
				continue;
			}
			backwardStamp_[seed.state] = backwardMark;
			backwardCost_[seed.state] = seed.cost;
			push(open, seed.cost, seed.state);
		}
		while (!open.empty()) {
			Entry top = pop(open);
			if (top.first >= std::min(reached, best)) {
				break;
			}
			if (top.first > backwardCost_[top.second]) {
				continue;
			}
			if (forwardStamp_[top.second] == forwardMark) {
				reached = std::min(reached, top.first + forwardCost_[top.second]);
			}
			for (uint32_t index = contraction->downFirst[top.second]; index != contraction->downFirst[top.second + 1]; ++index) {
				const Step& step = contraction->down[index];
				float next = top.first + step.cost;
				if (backwardStamp_[step.node] != backwardMark || next < backwardCost_[step.node]) {
					backwardStamp_[step.node] = backwardMark;
					backwardCost_[step.node] = next;
					push(open, next, step.node);
				}
			}
		}
		if (reached < best) {
			best = reached;
			bestIndex = target;
		}
	}
	cost = best;
	return bestIndex;
}

}
