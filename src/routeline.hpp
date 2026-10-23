#pragma once

#include "routing.hpp"

#include <cstdint>
#include <vector>

namespace physics {

/**
 * @brief      A gang zone of the line of a route on the map of a player: how many widths of the line
 *             before the end of the route it stands, its size, its centre and the client id it was
 *             given.
 *
 * Counting from the end keeps a square in its place while the player moves, and lets two routes to
 * the same place share the squares of the way they have in common.
 */
struct LineSquare {
	int64_t key = 0;
	float size = 0.0f;
	float x = 0.0f;
	float y = 0.0f;
	int clientid = -1;
};

/**
 * @brief      What changes when a line goes from the squares shown to the squares wanted: those kept,
 *             with their client ids, those to hide, and those to show, without client ids yet.
 */
struct LineChange {
	std::vector<LineSquare> kept;
	std::vector<LineSquare> hidden;
	std::vector<LineSquare> added;
};

/**
 * @brief      The squares of the line of a route seen from a distance along it, at most `budget`,
 *             sorted by `lineBefore`.
 *
 * The line from the player to the end is whole when it takes no more than `budget` squares as wide
 * as the line. Otherwise half the squares draw the way ahead without a gap, and the rest mark the way
 * beyond as dots, down to a dot at the end. The longer the way, the farther apart the dots, and the
 * bigger up to four widths of the line.
 */
void planLine(const std::vector<RoutePoint>& points, const std::vector<float>& along, float position, float width, int budget, std::vector<LineSquare>& squares);

/**
 * @brief      Compare the squares shown with the squares wanted, both sorted by `lineBefore`. A square
 *             is kept when one is wanted with the same key and size in the same place.
 */
LineChange compareLine(const std::vector<LineSquare>& shown, const std::vector<LineSquare>& wanted);

/**
 * @brief      The order of the squares of a line: by key, then by size.
 */
bool lineBefore(const LineSquare& a, const LineSquare& b);

}
