#pragma once

#include "database.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace physics {

/**
 * @brief      How a route weighs the paths, in the order of `A3D_ROUTE` in 3DAndreas.inc.
 *
 * `Fastest` takes the least time and `Shortest` the least distance. `AnyWay` takes the least time
 * driving one way roads both ways, as the emergency services do. `Traffic` keeps to the roads the
 * traffic of the game drives. `Boat` keeps to the lanes for boats and `Walk` to the paths of
 * pedestrians, and to the roads where they have none.
 */
enum class RouteType : uint8_t {
	Fastest,
	Shortest,
	AnyWay,
	Traffic,
	Boat,
	Walk
};

constexpr int RouteTypes = 6;

/**
 * @brief      Metres a second a walk goes at.
 */
constexpr float WalkSpeed = 5.0f;

/**
 * @brief      The node is on a road, in the graph `joinWalks` makes of the paths of pedestrians and the
 *             roads.
 */
constexpr uint8_t PathRoad = 8;

struct RoutePoint {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
};

/**
 * @brief      A place on a network: a chain, the distance along it from its first node, the point
 *             there and the direction of the chain at that point.
 */
struct NetworkPlace {
	uint32_t chain = UINT32_MAX;
	float offset = 0.0f;
	RoutePoint point;
	float directionX = 0.0f;
	float directionY = 0.0f;
};

/**
 * @brief      A turn of a route at a junction where more than two ways meet: the index of the point
 *             of the junction and the angle in degrees, positive to the left. A route that starts by
 *             turning back has a turn of `180.0` at its first point.
 */
struct RouteTurn {
	uint32_t point = 0;
	float angle = 0.0f;
};

/**
 * @brief      What a route found: its points, its turns, its length in metres, its time in seconds and
 *             what it cost by the weights of its type.
 */
struct RouteResult {
	std::vector<RoutePoint> points;
	std::vector<RouteTurn> turns;
	float length = 0.0f;
	float time = 0.0f;
	float cost = 0.0f;
};

/**
 * @brief      The paths of pedestrians joined to the roads, so a walk can go where there is no path.
 *             The ends and the junctions of the paths link to the nearest road node close by, and the
 *             ends and the junctions of the roads to the nearest path node. The road nodes come after
 *             the path nodes and carry `PathRoad`.
 */
PathGraph joinWalks(const PathGraph& walks, const PathGraph& roads);

/**
 * @brief      The paths of the game as chains of nodes between junctions, and for each type of route a
 *             contraction hierarchy of the chains taken one way or the other.
 *
 * A node is a junction unless it has two neighbours and the links on both sides go the same ways
 * and are of the same kind of path. Routes go from chain to chain, so a turn at a junction can cost
 * time: turning left more than turning right, and turning back most.
 *
 * The network is built and its hierarchies contracted on a thread of its own. A question asked before
 * the network is built waits for it, and until a hierarchy is contracted its routes are found with A*
 * on the chains, which finds the same routes more slowly. Walks and boats turn at no cost, so their
 * routes go from chain to junction and from junction to chain, which keeps their hierarchies small.
 */
class RouteNetwork {
public:
	RouteNetwork() = default;
	RouteNetwork(const RouteNetwork&) = delete;
	RouteNetwork& operator=(const RouteNetwork&) = delete;
	~RouteNetwork();

	/**
	 * @brief      Start building the network of the graph of vehicles.
	 */
	void buildRoads(PathGraph roads);

	/**
	 * @brief      Start building the network of the paths of pedestrians joined to the roads.
	 */
	void buildWalks(PathGraph walks, PathGraph roads);

	/**
	 * @brief      Wait until the network is built and every hierarchy contracted.
	 */
	void wait();

	/**
	 * @brief      Stop contracting and forget the network.
	 */
	void clear();

	/**
	 * @brief      Whether the network is built and every hierarchy contracted, without waiting.
	 */
	bool ready() const;

	/**
	 * @brief      When the network was built and every hierarchy contracted, once `ready` says so.
	 */
	std::chrono::steady_clock::time_point readyAt() const;

	/**
	 * @brief      Whether the network has a chain a type of route can use.
	 */
	bool serves(RouteType type) const;

	/**
	 * @brief      Whether the hierarchy of a type is contracted.
	 */
	bool contracted(RouteType type) const;

	/**
	 * @brief      The nearest place on a chain a type of route can use. Height counts twice, so a road
	 *             on a bridge above is farther than a road beside.
	 *
	 * @param      reach      How far to look, in metres.
	 * @param      connected  Only chains that every other place of the main part of the network can
	 *                        be reached from and can reach.
	 */
	bool nearest(float x, float y, float z, float reach, RouteType type, bool connected, NetworkPlace& place) const;

	/**
	 * @brief      Whether every other place of the main part of the network can be reached from a place
	 *             and can reach it, as `nearest` finds with `connected`.
	 */
	bool connects(const NetworkPlace& place, RouteType type) const;

	/**
	 * @brief      The best route between two places.
	 *
	 * @param      heading  Direction of travel at the start in degrees, as the game gives it, or a
	 *                      negative value for none. Starting against it costs a turn back.
	 *
	 * @return     - `true` a route was found
	 *             - `false` the places are not connected
	 */
	bool route(const NetworkPlace& from, const NetworkPlace& to, RouteType type, float heading, RouteResult& result) const;

	/**
	 * @brief      The length and time of the best route, without its points.
	 */
	bool measure(const NetworkPlace& from, const NetworkPlace& to, RouteType type, float heading, float& length, float& time) const;

	/**
	 * @brief      Which place the best route from a place reaches first, and what that route costs by
	 *             the weights of its type.
	 *
	 * @return     - the index of the place in `targets`
	 *             - `-1` none can be reached
	 */
	int closest(const NetworkPlace& from, const std::vector<NetworkPlace>& targets, RouteType type, float heading, float& cost) const;

	/**
	 * @brief      The best route found with A* on the chains instead of the hierarchy, to check it.
	 */
	bool search(const NetworkPlace& from, const NetworkPlace& to, RouteType type, float heading, RouteResult& result) const;

	/**
	 * @brief      The direction a route of a type may leave a place in, as a heading of the game.
	 */
	float headingAt(const NetworkPlace& place, RouteType type) const;

	size_t junctionCount() const;
	size_t chainCount() const;
	size_t arcCount(RouteType type) const;
	size_t shortcutCount(RouteType type) const;
	size_t componentSize(RouteType type) const;
	size_t memory(RouteType type) const;
	size_t memory() const;

private:
	struct Chain {
		uint32_t from = 0;
		uint32_t to = 0;
		uint32_t first = 0;
		uint32_t count = 0;
		float length = 0.0f;
		uint8_t flags = 0;
		uint8_t ways = 0;
	};

	struct Arc {
		uint32_t source = 0;
		uint32_t target = 0;
		float cost = 0.0f;
		float length = 0.0f;
		float time = 0.0f;
		uint32_t first = UINT32_MAX;
		uint32_t second = UINT32_MAX;
	};

	struct Step {
		uint32_t node = 0;
		float cost = 0.0f;
		uint32_t arc = 0;
	};

	struct Contraction {
		std::vector<Arc> arcs;
		std::vector<uint32_t> upFirst;
		std::vector<Step> up;
		std::vector<uint32_t> downFirst;
		std::vector<Step> down;
	};

	struct Hierarchy {
		bool used = false;
		float least = 0.0f;
		float back = 0.0f;
		float backTime = 0.0f;
		std::vector<float> cost;
		std::vector<float> time;
		std::vector<Arc> arcs;
		std::vector<uint32_t> outFirst;
		std::vector<uint32_t> outArcs;
		std::vector<uint32_t> component;
		uint32_t main = 0;
		size_t mainSize = 0;
		std::unique_ptr<Contraction> contraction;
		std::atomic<bool> ready{ false };
	};

	struct Seed {
		uint32_t state = 0;
		float cost = 0.0f;
		float length = 0.0f;
		float time = 0.0f;
		uint32_t arc = UINT32_MAX;
	};

	struct Ends {
		std::vector<Seed> sources;
		std::vector<Seed> targets;
		float direct = 0.0f;
		float directLength = 0.0f;
		float directTime = 0.0f;
		bool directForward = true;
		bool hasDirect = false;
	};

	struct Found {
		float cost = 0.0f;
		bool direct = false;
		const Seed* source = nullptr;
		const Seed* target = nullptr;
		std::vector<uint32_t> arcs;
	};

	using Entry = std::pair<float, uint32_t>;

	const Hierarchy& hierarchy(RouteType type) const;
	const Contraction* contractionOf(const Hierarchy& hierarchy) const;
	void launch(std::function<PathGraph()> make, bool vehicles);
	bool built() const;
	void buildChains(const PathGraph& graph);
	void buildDirections();
	void buildHierarchy(RouteType type, Hierarchy& hierarchy);
	void buildComponents(Hierarchy& hierarchy);
	bool contract(const Hierarchy& hierarchy, Contraction& contraction);
	void buildGrid();
	float turnCost(RouteType type, uint32_t from, uint32_t to, float& seconds) const;
	float turnAngle(uint32_t from, uint32_t to) const;
	void ends(const NetworkPlace& from, const NetworkPlace& to, const Hierarchy& hierarchy, RouteType type, float heading, Ends& ends) const;
	bool query(const Contraction& contraction, const Ends& ends, Found& found) const;
	bool searchStates(const Hierarchy& hierarchy, const Ends& ends, const RoutePoint& target, Found& found) const;
	void unpack(const std::vector<Arc>& arcs, uint32_t arc, std::vector<uint32_t>& unpacked) const;
	void assemble(const NetworkPlace& from, const NetworkPlace& to, RouteType type, const Hierarchy& hierarchy, const Ends& ends, const Found& found, float heading, RouteResult& result) const;
	void addPoints(uint32_t chain, float start, float end, std::vector<RoutePoint>& points) const;
	RoutePoint pointAt(uint32_t chain, float offset) const;
	uint32_t startOf(uint32_t state) const;
	uint32_t endOf(uint32_t state) const;
	RoutePoint junctionPoint(uint32_t junction) const;

	bool vehicles_ = false;
	std::vector<PathNode> nodes_;
	std::vector<uint8_t> degrees_;
	std::vector<uint32_t> junctionNodes_;
	std::vector<Chain> chains_;
	std::vector<uint32_t> chainNodes_;
	std::vector<float> chainOffsets_;
	std::vector<uint32_t> segmentChains_;
	std::vector<float> leaveX_;
	std::vector<float> leaveY_;
	std::vector<float> arriveX_;
	std::vector<float> arriveY_;
	std::vector<uint32_t> startFirst_;
	std::vector<uint32_t> startStates_;
	std::array<Hierarchy, RouteTypes> hierarchies_;
	float gridMinX_ = 0.0f;
	float gridMinY_ = 0.0f;
	int gridColumns_ = 0;
	int gridRows_ = 0;
	std::vector<uint32_t> gridFirst_;
	std::vector<uint32_t> gridSegments_;
	std::thread worker_;
	std::atomic<bool> stopping_{ false };
	std::atomic<bool> built_{ false };
	std::atomic<std::chrono::steady_clock::rep> readyAt_{ 0 };
	bool started_ = false;
	mutable std::mutex mutex_;
	mutable std::condition_variable changed_;
	mutable std::vector<float> forwardCost_;
	mutable std::vector<uint32_t> forwardParent_;
	mutable std::vector<uint32_t> forwardStamp_;
	mutable std::vector<float> backwardCost_;
	mutable std::vector<uint32_t> backwardParent_;
	mutable std::vector<uint32_t> backwardStamp_;
	mutable std::vector<Entry> forwardHeap_;
	mutable std::vector<Entry> backwardHeap_;
	mutable uint32_t stamp_ = 0;
};

}
