#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace physics {

/**
 * @brief      What a navigation mesh file holds before its tiles: the Detour parameters of the mesh and
 *             the size of the walker it was built for, in the metres of the game.
 *
 * Detour works with the Y axis up, so a point of the game `(x, y, z)` is `(x, z, -y)` in the mesh.
 */
struct NavmeshHeader {
	float origin[3] = {};
	float tileWidth = 0.0f;
	float tileHeight = 0.0f;
	uint32_t maxTiles = 0;
	uint32_t maxPolys = 0;
	float radius = 0.0f;
	float height = 0.0f;
	float climb = 0.0f;
	float cellSize = 0.0f;
	float cellHeight = 0.0f;
};

/**
 * @brief      A tile of a navigation mesh: where it is in the grid of tiles and its Detour data.
 */
struct NavmeshTile {
	int32_t x = 0;
	int32_t y = 0;
	std::vector<uint8_t> data;
};

/**
 * @brief      Reads a navigation mesh file one tile at a time, so each tile can go straight into the
 *             memory that keeps it.
 */
class NavmeshReader {
public:
	/**
	 * @brief      Open a file and read what it holds before its tiles.
	 *
	 * @return     - `true` the file was opened
	 *             - `false` it is missing or of an unknown version
	 */
	bool open(const std::string& path, NavmeshHeader& header);

	/**
	 * @brief      Read where the next tile is and the size of its data, which `read` then takes.
	 *
	 * @return     - `true` a tile follows
	 *             - `false` no tile is left, or the file is truncated
	 */
	bool next(int32_t& x, int32_t& y, uint32_t& size);

	bool read(void* data, uint32_t size);

private:
	std::ifstream file_;
	uint32_t left_ = 0;
};

/**
 * @brief      Write a navigation mesh file.
 *
 * @return     - `true` the file was written
 *             - `false` it could not be opened
 */
bool writeNavmesh(const std::string& path, const NavmeshHeader& header, const std::vector<NavmeshTile>& tiles);

}
