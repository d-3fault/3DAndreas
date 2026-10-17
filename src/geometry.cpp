#include "physics.hpp"

#include <Server/Components/Vehicles/vehicle_models.hpp>

#include <cmath>

namespace physics {

namespace {

enum class Part : int {
	FrontBumper,
	RearBumper,
	Roof,
	DriverSeat,
	PassengerSeat,
	DriverDoor,
	PassengerDoor,
	RearLeftDoor,
	RearRightDoor,
	PetrolCap,
	FrontLeftWheel,
	FrontRightWheel,
	RearLeftWheel,
	RearRightWheel,
	MiddleLeftWheel,
	MiddleRightWheel
};

JPH::Vec3 joltVector(Vector3 vector) {
	return JPH::Vec3(vector.x, vector.y, vector.z);
}

Vector3 gameVector(JPH::Vec3Arg vector) {
	return Vector3(vector.GetX(), vector.GetY(), vector.GetZ());
}

bool modelInfo(int model, VehicleModelInfoType type, Vector3& value) {
	return Impl::getVehicleModelInfo(model, type, value) && (value.x != 0.0f || value.y != 0.0f || value.z != 0.0f);
}

bool partOffset(int model, int part, Vector3& offset) {
	Vector3 size;
	Vector3 info;
	if (!modelInfo(model, VehicleModelInfo_Size, size)) {
		return false;
	}
	switch (Part(part)) {
	case Part::FrontBumper:
		Impl::getVehicleModelInfo(model, VehicleModelInfo_FrontBumperZ, info);
		offset = Vector3(0.0f, size.y * 0.5f, info.z);
		return true;
	case Part::RearBumper:
		Impl::getVehicleModelInfo(model, VehicleModelInfo_RearBumperZ, info);
		offset = Vector3(0.0f, -size.y * 0.5f, info.z);
		return true;
	case Part::Roof:
		offset = Vector3(0.0f, 0.0f, size.z * 0.5f);
		return true;
	case Part::DriverSeat:
	case Part::PassengerSeat:
		if (!modelInfo(model, VehicleModelInfo_FrontSeat, info)) {
			return false;
		}
		offset = Vector3(Part(part) == Part::DriverSeat ? -std::abs(info.x) : std::abs(info.x), info.y, info.z);
		return true;
	case Part::DriverDoor:
	case Part::PassengerDoor:
		if (!modelInfo(model, VehicleModelInfo_FrontSeat, info)) {
			return false;
		}
		offset = Vector3(Part(part) == Part::DriverDoor ? -size.x * 0.5f : size.x * 0.5f, info.y, info.z);
		return true;
	case Part::RearLeftDoor:
	case Part::RearRightDoor:
		if (!modelInfo(model, VehicleModelInfo_RearSeat, info)) {
			return false;
		}
		offset = Vector3(Part(part) == Part::RearLeftDoor ? -size.x * 0.5f : size.x * 0.5f, info.y, info.z);
		return true;
	case Part::PetrolCap:
		if (!modelInfo(model, VehicleModelInfo_PetrolCap, info)) {
			return false;
		}
		offset = info;
		return true;
	case Part::FrontLeftWheel:
	case Part::FrontRightWheel:
		if (!modelInfo(model, VehicleModelInfo_WheelsFront, info)) {
			return false;
		}
		offset = Vector3(Part(part) == Part::FrontLeftWheel ? -std::abs(info.x) : std::abs(info.x), info.y, info.z);
		return true;
	case Part::RearLeftWheel:
	case Part::RearRightWheel:
		if (!modelInfo(model, VehicleModelInfo_WheelsRear, info)) {
			return false;
		}
		offset = Vector3(Part(part) == Part::RearLeftWheel ? -std::abs(info.x) : std::abs(info.x), info.y, info.z);
		return true;
	case Part::MiddleLeftWheel:
	case Part::MiddleRightWheel:
		if (!modelInfo(model, VehicleModelInfo_WheelsMid, info)) {
			return false;
		}
		offset = Vector3(Part(part) == Part::MiddleLeftWheel ? -std::abs(info.x) : std::abs(info.x), info.y, info.z);
		return true;
	}
	return false;
}

IVehicle* vehicleOf(cell vehicleid) {
	State& s = state();
	return s.vehicles == nullptr ? nullptr : s.vehicles->get(vehicleid);
}

cell AMX_NATIVE_CALL A3D_GetOffsetPosition(AMX* amx, const cell* params) {
	JPH::Quat rotation = toJolt(readVector(params, 4));
	return writeVector(amx, params, 10, readVector(params, 1) + gameVector(rotation * joltVector(readVector(params, 7))));
}

cell AMX_NATIVE_CALL A3D_GetPositionOffset(AMX* amx, const cell* params) {
	JPH::Quat rotation = toJolt(readVector(params, 4));
	return writeVector(amx, params, 10, gameVector(rotation.Conjugated() * joltVector(readVector(params, 7) - readVector(params, 1))));
}

cell AMX_NATIVE_CALL A3D_GetOffsetRotation(AMX* amx, const cell* params) {
	JPH::Quat rotation = toJolt(readVector(params, 1)) * toJolt(readVector(params, 4));
	return writeVector(amx, params, 7, toEuler(rotation.Normalized()));
}

cell AMX_NATIVE_CALL A3D_GetLookRotation(AMX* amx, const cell* params) {
	Vector3 offset = readVector(params, 4) - readVector(params, 1);
	float flat = std::sqrt(offset.x * offset.x + offset.y * offset.y);
	if (flat <= 0.0f && offset.z == 0.0f) {
		return false;
	}
	JPH::Quat heading = JPH::Quat::sRotation(JPH::Vec3::sAxisZ(), std::atan2(-offset.x, offset.y));
	JPH::Quat pitch = JPH::Quat::sRotation(JPH::Vec3::sAxisX(), std::atan2(offset.z, flat));
	return writeVector(amx, params, 7, toEuler((heading * pitch).Normalized()));
}

cell AMX_NATIVE_CALL A3D_GetVehicleOffsetPosition(AMX* amx, const cell* params) {
	IVehicle* vehicle = vehicleOf(params[1]);
	if (vehicle == nullptr) {
		return false;
	}
	JPH::Quat rotation = vehicleRotation(vehicle->getRotation());
	return writeVector(amx, params, 5, vehicle->getPosition() + gameVector(rotation * joltVector(readVector(params, 2))));
}

cell AMX_NATIVE_CALL A3D_GetVehiclePositionOffset(AMX* amx, const cell* params) {
	IVehicle* vehicle = vehicleOf(params[1]);
	if (vehicle == nullptr) {
		return false;
	}
	JPH::Quat rotation = vehicleRotation(vehicle->getRotation());
	return writeVector(amx, params, 5, gameVector(rotation.Conjugated() * joltVector(readVector(params, 2) - vehicle->getPosition())));
}

cell AMX_NATIVE_CALL A3D_GetVehiclePartPosition(AMX* amx, const cell* params) {
	IVehicle* vehicle = vehicleOf(params[1]);
	Vector3 offset;
	if (vehicle == nullptr || !partOffset(vehicle->getModel(), params[2], offset)) {
		return false;
	}
	JPH::Quat rotation = vehicleRotation(vehicle->getRotation());
	return writeVector(amx, params, 3, vehicle->getPosition() + gameVector(rotation * joltVector(offset)));
}

cell AMX_NATIVE_CALL A3D_EulerToQuaternion(AMX* amx, const cell* params) {
	GTAQuat rotation(readVector(params, 1));
	writeFloat(amx, params[4], rotation.q.w);
	writeFloat(amx, params[5], rotation.q.x);
	writeFloat(amx, params[6], rotation.q.y);
	return writeFloat(amx, params[7], rotation.q.z);
}

cell AMX_NATIVE_CALL A3D_QuaternionToEuler(AMX* amx, const cell* params) {
	GTAQuat rotation(readFloat(params, 1), readFloat(params, 2), readFloat(params, 3), readFloat(params, 4));
	return writeVector(amx, params, 5, rotation.ToEuler());
}

cell AMX_NATIVE_CALL A3D_GetSurfaceRotation(AMX* amx, const cell* params) {
	Vector3 normal = readVector(params, 1);
	JPH::Vec3 up(normal.x, normal.y, normal.z);
	if (up.IsNearZero()) {
		return false;
	}
	JPH::Quat tilt = JPH::Quat::sFromTo(JPH::Vec3::sAxisZ(), up.Normalized());
	JPH::Quat rotation = tilt * toJolt(Vector3(0.0f, 0.0f, readFloat(params, 7)));
	return writeVector(amx, params, 4, toEuler(rotation.Normalized()));
}

cell AMX_NATIVE_CALL A3D_GetReflection(AMX* amx, const cell* params) {
	Vector3 direction = readVector(params, 1);
	Vector3 normal = readVector(params, 4);
	float length = glm::length(normal);
	if (length <= 0.0f) {
		return false;
	}
	normal /= length;
	return writeVector(amx, params, 7, direction - normal * (2.0f * glm::dot(direction, normal)));
}

const AMX_NATIVE_INFO natives[] = {
	{ "A3D_EulerToQuaternion", A3D_EulerToQuaternion },
	{ "A3D_QuaternionToEuler", A3D_QuaternionToEuler },
	{ "A3D_GetSurfaceRotation", A3D_GetSurfaceRotation },
	{ "A3D_GetReflection", A3D_GetReflection },
	{ "A3D_GetOffsetPosition", A3D_GetOffsetPosition },
	{ "A3D_GetPositionOffset", A3D_GetPositionOffset },
	{ "A3D_GetOffsetRotation", A3D_GetOffsetRotation },
	{ "A3D_GetLookRotation", A3D_GetLookRotation },
	{ "A3D_GetVehicleOffsetPosition", A3D_GetVehicleOffsetPosition },
	{ "A3D_GetVehiclePositionOffset", A3D_GetVehiclePositionOffset },
	{ "A3D_GetVehiclePartPosition", A3D_GetVehiclePartPosition },
};

}

void registerGeometryNatives(IPawnScript& script) {
	script.Register(natives, int(sizeof natives / sizeof natives[0]));
}

}
