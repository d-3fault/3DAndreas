#include "../src/database.hpp"
#include "../src/navmesh.hpp"

#include <DetourCommon.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <Recast.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace physics;

namespace {

constexpr float CellSize = 0.2f;
constexpr float CellHeight = 0.2f;
constexpr int TileCells = 192;
constexpr float AgentRadius = 0.35f;
constexpr float AgentHeight = 1.8f;
constexpr float AgentClimb = 0.45f;
constexpr float SlopeAngle = 45.0f;
constexpr float WaterDepth = 1.0f;
constexpr float SeaEdge = 3000.0f;
constexpr float MapEdge = 4000.0f;
constexpr unsigned short WalkFlag = 1;

struct LocalMesh {
	std::vector<float> vertices;
	std::vector<int> triangles;
	float radius = 0.0f;
	float centre[3] = {};
};

struct Placed {
	const LocalMesh* mesh = nullptr;
	float rotation[9] = {};
	float position[3] = {};
	float centre[3] = {};
	float radius = 0.0f;
};

struct Settings {
	int threads = 0;
	float cellSize = CellSize;
	float detailDistance = 4.0f;
	float detailError = 0.5f;
	float simplification = 2.0f;
	float edge = 24.0f;
	int region = 16;
	float minX = -MapEdge;
	float minY = -MapEdge;
	float maxX = MapEdge;
	float maxY = MapEdge;
};

void addBox(LocalMesh& mesh, float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
	int base = int(mesh.vertices.size() / 3);
	const float corners[8][3] = {
		{ minX, minY, minZ }, { maxX, minY, minZ }, { maxX, maxY, minZ }, { minX, maxY, minZ },
		{ minX, minY, maxZ }, { maxX, minY, maxZ }, { maxX, maxY, maxZ }, { minX, maxY, maxZ }
	};
	for (const auto& corner : corners) {
		mesh.vertices.insert(mesh.vertices.end(), { corner[0], corner[1], corner[2] });
	}
	const int faces[12][3] = {
		{ 0, 2, 1 }, { 0, 3, 2 }, { 4, 5, 6 }, { 4, 6, 7 }, { 0, 1, 5 }, { 0, 5, 4 },
		{ 1, 2, 6 }, { 1, 6, 5 }, { 2, 3, 7 }, { 2, 7, 6 }, { 3, 0, 4 }, { 3, 4, 7 }
	};
	for (const auto& face : faces) {
		mesh.triangles.insert(mesh.triangles.end(), { base + face[0], base + face[1], base + face[2] });
	}
}

LocalMesh meshOf(const Model& model) {
	LocalMesh mesh;
	Bounds bounds = model.bounds.radius > 0.0f ? model.bounds : measureModel(model);
	for (const Vertex& vertex : model.vertices) {
		mesh.vertices.insert(mesh.vertices.end(), { vertex.x / VertexScale, vertex.y / VertexScale, vertex.z / VertexScale });
	}
	for (const Face& face : model.faces) {
		mesh.triangles.insert(mesh.triangles.end(), { int(face.a), int(face.b), int(face.c) });
	}
	for (const Box& box : model.boxes) {
		addBox(mesh, box.minX, box.minY, box.minZ, box.maxX, box.maxY, box.maxZ);
	}
	for (const Sphere& sphere : model.spheres) {
		addBox(mesh, sphere.x - sphere.radius, sphere.y - sphere.radius, sphere.z - sphere.radius, sphere.x + sphere.radius, sphere.y + sphere.radius, sphere.z + sphere.radius);
	}
	mesh.centre[0] = bounds.centreX;
	mesh.centre[1] = bounds.centreY;
	mesh.centre[2] = bounds.centreZ;
	mesh.radius = bounds.radius;
	return mesh;
}

void rotationOf(const Placement& placement, float matrix[9]) {
	float x = placement.qx;
	float y = placement.qy;
	float z = placement.qz;
	float w = -placement.qw;
	float length = std::sqrt(x * x + y * y + z * z + w * w);
	if (length > 0.0f) {
		x /= length;
		y /= length;
		z /= length;
		w /= length;
	}
	matrix[0] = 1.0f - 2.0f * (y * y + z * z);
	matrix[1] = 2.0f * (x * y - z * w);
	matrix[2] = 2.0f * (x * z + y * w);
	matrix[3] = 2.0f * (x * y + z * w);
	matrix[4] = 1.0f - 2.0f * (x * x + z * z);
	matrix[5] = 2.0f * (y * z - x * w);
	matrix[6] = 2.0f * (x * z - y * w);
	matrix[7] = 2.0f * (y * z + x * w);
	matrix[8] = 1.0f - 2.0f * (x * x + y * y);
}

void transform(const float matrix[9], const float position[3], const float* local, float* world) {
	world[0] = matrix[0] * local[0] + matrix[1] * local[1] + matrix[2] * local[2] + position[0];
	world[1] = matrix[3] * local[0] + matrix[4] * local[1] + matrix[5] * local[2] + position[1];
	world[2] = matrix[6] * local[0] + matrix[7] * local[1] + matrix[8] * local[2] + position[2];
}

bool overTriangle(const Water& water, int a, int b, int c, float x, float y, float& level) {
	float across = (water.y[b] - water.y[c]) * (water.x[a] - water.x[c]) + (water.x[c] - water.x[b]) * (water.y[a] - water.y[c]);
	if (std::abs(across) < 1e-5f) {
		return false;
	}
	float first = ((water.y[b] - water.y[c]) * (x - water.x[c]) + (water.x[c] - water.x[b]) * (y - water.y[c])) / across;
	float second = ((water.y[c] - water.y[a]) * (x - water.x[c]) + (water.x[a] - water.x[c]) * (y - water.y[c])) / across;
	float third = 1.0f - first - second;
	if (first < -1e-5f || second < -1e-5f || third < -1e-5f) {
		return false;
	}
	level = first * water.z[a] + second * water.z[b] + third * water.z[c];
	return true;
}

bool waterAt(const std::vector<const Water*>& waters, float x, float y, float& level) {
	bool found = false;
	level = -1e30f;
	for (const Water* water : waters) {
		float height;
		if (overTriangle(*water, 0, 1, 2, x, y, height) || (water->corners == 4 && overTriangle(*water, 1, 3, 2, x, y, height))) {
			level = std::max(level, height);
			found = true;
		}
	}
	if (!found && (std::abs(x) > SeaEdge || std::abs(y) > SeaEdge)) {
		level = 0.0f;
		found = true;
	}
	return found;
}

class Builder {
public:
	Builder(const Database& database, const Settings& settings) : database_(database), settings_(settings) {
	}

	bool build(std::vector<NavmeshTile>& tiles, NavmeshHeader& header) {
		prepare();
		header.origin[0] = originX_;
		header.origin[1] = 0.0f;
		header.origin[2] = originZ_;
		header.tileWidth = TileCells * settings_.cellSize;
		header.tileHeight = TileCells * settings_.cellSize;
		uint32_t count = uint32_t(columns_ * rows_);
		header.maxTiles = 1;
		while (header.maxTiles < count) {
			header.maxTiles <<= 1;
		}
		header.maxPolys = 1u << 16;
		header.radius = AgentRadius;
		header.height = AgentHeight;
		header.climb = AgentClimb;
		header.cellSize = settings_.cellSize;
		header.cellHeight = CellHeight;
		int threads = settings_.threads > 0 ? settings_.threads : int(std::max(1u, std::thread::hardware_concurrency()));
		std::atomic<int> next(0);
		std::atomic<int> done(0);
		std::mutex lock;
		std::vector<std::thread> workers;
		auto started = std::chrono::steady_clock::now();
		for (int worker = 0; worker != threads; ++worker) {
			workers.emplace_back([&]() {
				rcContext context(false);
				while (true) {
					int index = next++;
					if (index >= int(count)) {
						break;
					}
					NavmeshTile tile;
					tile.x = index % columns_;
					tile.y = index / columns_;
					if (buildTile(context, tile)) {
						std::lock_guard<std::mutex> guard(lock);
						tiles.push_back(std::move(tile));
					}
					int finished = ++done;
					if (finished % 2000 == 0) {
						double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
						std::printf("%d of %u tiles in %.0f s\n", finished, count, seconds);
					}
				}
			});
		}
		for (std::thread& worker : workers) {
			worker.join();
		}
		std::sort(tiles.begin(), tiles.end(), [](const NavmeshTile& a, const NavmeshTile& b) {
			return a.y != b.y ? a.y < b.y : a.x < b.x;
		});
		return true;
	}

	size_t polygons() const {
		return polygons_;
	}

private:
	void prepare() {
		for (const auto& entry : database_.models) {
			meshes_.emplace(entry.first, meshOf(entry.second));
		}
		for (const Placement& placement : database_.placements) {
			auto mesh = meshes_.find(placement.modelid);
			if (mesh == meshes_.end() || mesh->second.triangles.empty()) {
				continue;
			}
			Placed placed;
			placed.mesh = &mesh->second;
			rotationOf(placement, placed.rotation);
			placed.position[0] = placement.x;
			placed.position[1] = placement.y;
			placed.position[2] = placement.z;
			transform(placed.rotation, placed.position, mesh->second.centre, placed.centre);
			placed.radius = mesh->second.radius;
			if (placed.centre[0] + placed.radius < settings_.minX || placed.centre[0] - placed.radius > settings_.maxX || placed.centre[1] + placed.radius < settings_.minY || placed.centre[1] - placed.radius > settings_.maxY) {
				continue;
			}
			placed_.push_back(placed);
		}
		float tile = TileCells * settings_.cellSize;
		originX_ = settings_.minX;
		originZ_ = -settings_.maxY;
		columns_ = int(std::ceil((settings_.maxX - settings_.minX) / tile));
		rows_ = int(std::ceil((settings_.maxY - settings_.minY) / tile));
		cells_.assign(size_t(columns_) * size_t(rows_), {});
		float border = (AgentRadius / settings_.cellSize + 4.0f) * settings_.cellSize;
		for (uint32_t index = 0; index != uint32_t(placed_.size()); ++index) {
			const Placed& placed = placed_[index];
			float minX = placed.centre[0] - placed.radius - border;
			float maxX = placed.centre[0] + placed.radius + border;
			float minZ = -placed.centre[1] - placed.radius - border;
			float maxZ = -placed.centre[1] + placed.radius + border;
			int left = std::max(int(std::floor((minX - originX_) / tile)), 0);
			int right = std::min(int(std::floor((maxX - originX_) / tile)), columns_ - 1);
			int bottom = std::max(int(std::floor((minZ - originZ_) / tile)), 0);
			int top = std::min(int(std::floor((maxZ - originZ_) / tile)), rows_ - 1);
			for (int row = bottom; row <= top; ++row) {
				for (int column = left; column <= right; ++column) {
					cells_[size_t(row) * size_t(columns_) + size_t(column)].push_back(index);
				}
			}
		}
		for (const Water& water : database_.waters) {
			waters_.push_back(&water);
		}
		std::printf("%zu placements in %d by %d tiles of %.1f m\n", placed_.size(), columns_, rows_, tile);
	}

	bool buildTile(rcContext& context, NavmeshTile& tile) {
		const std::vector<uint32_t>& found = cells_[size_t(tile.y) * size_t(columns_) + size_t(tile.x)];
		if (found.empty()) {
			return false;
		}
		rcConfig config;
		std::memset(&config, 0, sizeof config);
		config.cs = settings_.cellSize;
		config.ch = CellHeight;
		config.walkableSlopeAngle = SlopeAngle;
		config.walkableHeight = int(std::ceil(AgentHeight / CellHeight));
		config.walkableClimb = int(std::floor(AgentClimb / CellHeight));
		config.walkableRadius = int(std::ceil(AgentRadius / settings_.cellSize));
		config.maxEdgeLen = int(settings_.edge / settings_.cellSize);
		config.maxSimplificationError = settings_.simplification;
		config.minRegionArea = settings_.region * settings_.region;
		config.mergeRegionArea = 20 * 20;
		config.maxVertsPerPoly = 6;
		config.tileSize = TileCells;
		config.borderSize = config.walkableRadius + 3;
		config.width = config.tileSize + config.borderSize * 2;
		config.height = config.tileSize + config.borderSize * 2;
		config.detailSampleDist = settings_.detailDistance;
		config.detailSampleMaxError = settings_.detailError;
		float cell = settings_.cellSize;
		float tileSize = TileCells * cell;
		config.bmin[0] = originX_ + tile.x * tileSize - config.borderSize * cell;
		config.bmin[2] = originZ_ + tile.y * tileSize - config.borderSize * cell;
		config.bmax[0] = originX_ + (tile.x + 1) * tileSize + config.borderSize * cell;
		config.bmax[2] = originZ_ + (tile.y + 1) * tileSize + config.borderSize * cell;
		std::vector<float> vertices;
		std::vector<int> triangles;
		float low = 1e30f;
		float high = -1e30f;
		for (uint32_t index : found) {
			const Placed& placed = placed_[index];
			int base = int(vertices.size() / 3);
			const std::vector<float>& local = placed.mesh->vertices;
			for (size_t vertex = 0; vertex < local.size(); vertex += 3) {
				float world[3];
				transform(placed.rotation, placed.position, &local[vertex], world);
				vertices.insert(vertices.end(), { world[0], world[2], -world[1] });
				low = std::min(low, world[2]);
				high = std::max(high, world[2]);
			}
			const std::vector<int>& corners = placed.mesh->triangles;
			for (size_t corner = 0; corner < corners.size(); corner += 3) {
				triangles.insert(triangles.end(), { base + corners[corner], base + corners[corner + 2], base + corners[corner + 1] });
			}
		}
		if (triangles.empty()) {
			return false;
		}
		config.bmin[1] = low - 1.0f;
		config.bmax[1] = high + AgentHeight + 1.0f;
		int vertexCount = int(vertices.size() / 3);
		int triangleCount = int(triangles.size() / 3);
		std::vector<unsigned char> areas(size_t(triangleCount), 0);
		rcHeightfield* solid = rcAllocHeightfield();
		rcCompactHeightfield* compact = nullptr;
		rcContourSet* contours = nullptr;
		rcPolyMesh* mesh = nullptr;
		rcPolyMeshDetail* detail = nullptr;
		auto release = [&]() {
			rcFreeHeightField(solid);
			rcFreeCompactHeightfield(compact);
			rcFreeContourSet(contours);
			rcFreePolyMesh(mesh);
			rcFreePolyMeshDetail(detail);
		};
		if (!rcCreateHeightfield(&context, *solid, config.width, config.height, config.bmin, config.bmax, config.cs, config.ch)) {
			release();
			return false;
		}
		rcMarkWalkableTriangles(&context, config.walkableSlopeAngle, vertices.data(), vertexCount, triangles.data(), triangleCount, areas.data());
		if (!rcRasterizeTriangles(&context, vertices.data(), vertexCount, triangles.data(), areas.data(), triangleCount, *solid, config.walkableClimb)) {
			release();
			return false;
		}
		rcFilterLowHangingWalkableObstacles(&context, config.walkableClimb, *solid);
		rcFilterLedgeSpans(&context, config.walkableHeight, config.walkableClimb, *solid);
		rcFilterWalkableLowHeightSpans(&context, config.walkableHeight, *solid);
		drown(*solid);
		compact = rcAllocCompactHeightfield();
		if (!rcBuildCompactHeightfield(&context, config.walkableHeight, config.walkableClimb, *solid, *compact)) {
			release();
			return false;
		}
		rcFreeHeightField(solid);
		solid = nullptr;
		if (!rcErodeWalkableArea(&context, config.walkableRadius, *compact)) {
			release();
			return false;
		}
		if (!rcBuildLayerRegions(&context, *compact, config.borderSize, config.minRegionArea)) {
			release();
			return false;
		}
		contours = rcAllocContourSet();
		if (!rcBuildContours(&context, *compact, config.maxSimplificationError, config.maxEdgeLen, *contours) || contours->nconts == 0) {
			release();
			return false;
		}
		mesh = rcAllocPolyMesh();
		if (!rcBuildPolyMesh(&context, *contours, config.maxVertsPerPoly, *mesh) || mesh->npolys == 0) {
			release();
			return false;
		}
		detail = rcAllocPolyMeshDetail();
		if (!rcBuildPolyMeshDetail(&context, *mesh, *compact, config.detailSampleDist, config.detailSampleMaxError, *detail)) {
			release();
			return false;
		}
		for (int index = 0; index != mesh->npolys; ++index) {
			mesh->flags[index] = mesh->areas[index] == RC_WALKABLE_AREA ? WalkFlag : 0;
		}
		dtNavMeshCreateParams params;
		std::memset(&params, 0, sizeof params);
		params.verts = mesh->verts;
		params.vertCount = mesh->nverts;
		params.polys = mesh->polys;
		params.polyAreas = mesh->areas;
		params.polyFlags = mesh->flags;
		params.polyCount = mesh->npolys;
		params.nvp = mesh->nvp;
		params.detailMeshes = detail->meshes;
		params.detailVerts = detail->verts;
		params.detailVertsCount = detail->nverts;
		params.detailTris = detail->tris;
		params.detailTriCount = detail->ntris;
		params.walkableHeight = AgentHeight;
		params.walkableRadius = AgentRadius;
		params.walkableClimb = AgentClimb;
		params.tileX = tile.x;
		params.tileY = tile.y;
		params.tileLayer = 0;
		rcVcopy(params.bmin, mesh->bmin);
		rcVcopy(params.bmax, mesh->bmax);
		params.cs = config.cs;
		params.ch = config.ch;
		params.buildBvTree = true;
		unsigned char* data = nullptr;
		int size = 0;
		int count = mesh->npolys;
		bool created = dtCreateNavMeshData(&params, &data, &size);
		release();
		if (!created) {
			return false;
		}
		tile.data.assign(data, data + size);
		dtFree(data);
		polygons_ += size_t(count);
		return true;
	}

	void drown(rcHeightfield& solid) {
		std::vector<const Water*> nearby;
		float minX = solid.bmin[0];
		float maxX = solid.bmax[0];
		float minY = -solid.bmax[2];
		float maxY = -solid.bmin[2];
		for (const Water* water : waters_) {
			float waterMinX = *std::min_element(water->x, water->x + water->corners);
			float waterMaxX = *std::max_element(water->x, water->x + water->corners);
			float waterMinY = *std::min_element(water->y, water->y + water->corners);
			float waterMaxY = *std::max_element(water->y, water->y + water->corners);
			if (waterMaxX >= minX && waterMinX <= maxX && waterMaxY >= minY && waterMinY <= maxY) {
				nearby.push_back(water);
			}
		}
		bool sea = minX < -SeaEdge || maxX > SeaEdge || minY < -SeaEdge || maxY > SeaEdge;
		if (nearby.empty() && !sea) {
			return;
		}
		for (int row = 0; row != solid.height; ++row) {
			for (int column = 0; column != solid.width; ++column) {
				float x = solid.bmin[0] + (column + 0.5f) * solid.cs;
				float y = -(solid.bmin[2] + (row + 0.5f) * solid.cs);
				float level;
				if (!waterAt(nearby, x, y, level)) {
					continue;
				}
				for (rcSpan* span = solid.spans[column + row * solid.width]; span != nullptr; span = span->next) {
					float top = solid.bmin[1] + span->smax * solid.ch;
					if (top < level - WaterDepth) {
						span->area = RC_NULL_AREA;
					}
				}
			}
		}
	}

	const Database& database_;
	Settings settings_;
	std::unordered_map<int, LocalMesh> meshes_;
	std::vector<Placed> placed_;
	std::vector<const Water*> waters_;
	std::vector<std::vector<uint32_t>> cells_;
	float originX_ = 0.0f;
	float originZ_ = 0.0f;
	int columns_ = 0;
	int rows_ = 0;
	std::atomic<size_t> polygons_{ 0 };
};

void dump(const std::vector<NavmeshTile>& tiles, const std::string& path) {
	FILE* file = std::fopen(path.c_str(), "w");
	if (file == nullptr) {
		return;
	}
	for (const NavmeshTile& tile : tiles) {
		const unsigned char* data = tile.data.data();
		const dtMeshHeader* header = reinterpret_cast<const dtMeshHeader*>(data);
		const float* vertices = reinterpret_cast<const float*>(data + dtAlign4(sizeof(dtMeshHeader)));
		const dtPoly* polys = reinterpret_cast<const dtPoly*>(data + dtAlign4(sizeof(dtMeshHeader)) + dtAlign4(sizeof(float) * 3 * header->vertCount));
		for (int index = 0; index != header->polyCount; ++index) {
			const dtPoly& poly = polys[index];
			std::fprintf(file, "poly");
			for (int corner = 0; corner != poly.vertCount; ++corner) {
				const float* vertex = &vertices[poly.verts[corner] * 3];
				std::fprintf(file, " %.2f %.2f %.2f", vertex[0], -vertex[2], vertex[1]);
			}
			std::fprintf(file, "\n");
		}
	}
	std::fclose(file);
}

}

int main(int argc, char** argv) {
	if (argc < 3) {
		std::printf("usage: navbuild <world.db> <navmesh.db> [--threads n] [--area minX minY maxX maxY]\n");
		return 1;
	}
	Settings settings;
	std::string dumpPath;
	for (int index = 3; index < argc; ++index) {
		std::string option = argv[index];
		if (option == "--threads" && index + 1 < argc) {
			settings.threads = std::atoi(argv[++index]);
		} else if (option == "--cell" && index + 1 < argc) {
			settings.cellSize = float(std::atof(argv[++index]));
		} else if (option == "--detail" && index + 2 < argc) {
			settings.detailDistance = float(std::atof(argv[++index]));
			settings.detailError = float(std::atof(argv[++index]));
		} else if (option == "--simplify" && index + 2 < argc) {
			settings.simplification = float(std::atof(argv[++index]));
			settings.edge = float(std::atof(argv[++index]));
		} else if (option == "--region" && index + 1 < argc) {
			settings.region = std::atoi(argv[++index]);
		} else if (option == "--dump" && index + 1 < argc) {
			dumpPath = argv[++index];
		} else if (option == "--area" && index + 4 < argc) {
			settings.minX = float(std::atof(argv[++index]));
			settings.minY = float(std::atof(argv[++index]));
			settings.maxX = float(std::atof(argv[++index]));
			settings.maxY = float(std::atof(argv[++index]));
		}
	}
	auto start = std::chrono::steady_clock::now();
	Database database;
	if (!readDatabase(argv[1], database) || database.colAndreas) {
		std::printf("cannot read %s as a database of the component\n", argv[1]);
		return 1;
	}
	Builder builder(database, settings);
	std::vector<NavmeshTile> tiles;
	NavmeshHeader header;
	builder.build(tiles, header);
	if (!dumpPath.empty()) {
		dump(tiles, dumpPath);
	}
	if (!writeNavmesh(argv[2], header, tiles)) {
		std::printf("cannot write %s\n", argv[2]);
		return 1;
	}
	double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
	std::printf("%zu tiles and %zu polygons, %s written in %.1f s, %ju bytes\n", tiles.size(), builder.polygons(), argv[2], seconds, uintmax_t(std::filesystem::file_size(argv[2])));
	return 0;
}
