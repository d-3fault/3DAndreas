#include "physics.hpp"

#include <random>

namespace physics {

namespace {

float randomUnit() {
	return std::uniform_real_distribution<float>(0.0f, 1.0f)(state().random);
}

RoutePoint pointOf(const cell* params, int index) {
	return { readFloat(params, index), readFloat(params, index + 1), readFloat(params, index + 2) };
}

cell AMX_NATIVE_CALL A3D_GetNearestWalkPoint(AMX* amx, const cell* params) {
	RoutePoint point;
	if (!state().walkMesh.nearest(pointOf(params, 1), optionalFloat(params, 7, 3.0f), point)) {
		return false;
	}
	return writeVector(amx, params, 4, Vector3(point.x, point.y, point.z));
}

cell AMX_NATIVE_CALL A3D_GetRandomWalkPoint(AMX* amx, const cell* params) {
	RoutePoint point;
	if (!state().walkMesh.random(pointOf(params, 1), readFloat(params, 4), randomUnit, point)) {
		return false;
	}
	return writeVector(amx, params, 5, Vector3(point.x, point.y, point.z));
}

cell AMX_NATIVE_CALL A3D_WalkRayCast(AMX* amx, const cell* params) {
	RoutePoint hit;
	if (!state().walkMesh.raycast(pointOf(params, 1), pointOf(params, 4), hit)) {
		return false;
	}
	writeVector(amx, params, 7, Vector3(hit.x, hit.y, hit.z));
	return true;
}

cell AMX_NATIVE_CALL A3D_IsWalkMeshLoaded(AMX* amx, const cell* params) {
	return state().walkMesh.loaded();
}

const AMX_NATIVE_INFO natives[] = {
	{ "A3D_GetNearestWalkPoint", A3D_GetNearestWalkPoint },
	{ "A3D_GetRandomWalkPoint", A3D_GetRandomWalkPoint },
	{ "A3D_WalkRayCast", A3D_WalkRayCast },
	{ "A3D_IsWalkMeshLoaded", A3D_IsWalkMeshLoaded },
};

}

void registerWalkNatives(IPawnScript& script) {
	script.Register(natives, int(sizeof natives / sizeof natives[0]));
}

}
