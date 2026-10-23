#include "navmesh.hpp"

#include <cstring>
#include <fstream>

namespace physics {

namespace {

constexpr char Magic[4] = { 'A', '3', 'N', 'V' };
constexpr uint32_t Version = 1;

}

bool NavmeshReader::open(const std::string& path, NavmeshHeader& header) {
	file_.open(path, std::ios::binary);
	if (!file_) {
		return false;
	}
	char magic[4];
	uint32_t version = 0;
	file_.read(magic, sizeof magic);
	file_.read(reinterpret_cast<char*>(&version), sizeof version);
	file_.read(reinterpret_cast<char*>(&header), sizeof header);
	file_.read(reinterpret_cast<char*>(&left_), sizeof left_);
	return bool(file_) && std::memcmp(magic, Magic, sizeof Magic) == 0 && version == Version;
}

bool NavmeshReader::next(int32_t& x, int32_t& y, uint32_t& size) {
	if (left_ == 0) {
		return false;
	}
	--left_;
	file_.read(reinterpret_cast<char*>(&x), sizeof x);
	file_.read(reinterpret_cast<char*>(&y), sizeof y);
	file_.read(reinterpret_cast<char*>(&size), sizeof size);
	return bool(file_) && size <= (64u << 20);
}

bool NavmeshReader::read(void* data, uint32_t size) {
	file_.read(static_cast<char*>(data), size);
	return bool(file_);
}

bool writeNavmesh(const std::string& path, const NavmeshHeader& header, const std::vector<NavmeshTile>& tiles) {
	std::ofstream file(path, std::ios::binary | std::ios::trunc);
	if (!file) {
		return false;
	}
	uint32_t count = uint32_t(tiles.size());
	file.write(Magic, sizeof Magic);
	file.write(reinterpret_cast<const char*>(&Version), sizeof Version);
	file.write(reinterpret_cast<const char*>(&header), sizeof header);
	file.write(reinterpret_cast<const char*>(&count), sizeof count);
	for (const NavmeshTile& tile : tiles) {
		uint32_t size = uint32_t(tile.data.size());
		file.write(reinterpret_cast<const char*>(&tile.x), sizeof tile.x);
		file.write(reinterpret_cast<const char*>(&tile.y), sizeof tile.y);
		file.write(reinterpret_cast<const char*>(&size), sizeof size);
		file.write(reinterpret_cast<const char*>(tile.data.data()), size);
	}
	return bool(file);
}

}
