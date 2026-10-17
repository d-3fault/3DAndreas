#include "physics.hpp"

#include <cmath>

namespace physics {

namespace {

constexpr float StillSpeed = 0.05f;

Vector3 clientPosition(const Body& body) {
	State& s = state();
	if (body.speed <= 0.0f || body.duration <= 0.0) {
		return body.to;
	}
	double progress = std::min(std::max((s.now - body.sentAt) / body.duration, 0.0), 1.0);
	return body.from + (body.to - body.from) * float(progress);
}

float nearestAngle(float angle, float previous) {
	while (angle - previous > 180.0f) {
		angle -= 360.0f;
	}
	while (angle - previous < -180.0f) {
		angle += 360.0f;
	}
	return angle;
}

Vector3 nearestEuler(Vector3 euler, Vector3 previous) {
	return Vector3(nearestAngle(euler.x, previous.x), nearestAngle(euler.y, previous.y), nearestAngle(euler.z, previous.z));
}

void callMover(Body& body, Vector3 position, float speed, Vector3 rotation) {
	Script* script = ownerOf(body);
	if (script == nullptr) {
		return;
	}
	callPublic(*script, "@A3D_MoveDynamicObject", { cell(body.objectid), amx_ftoc(position.x), amx_ftoc(position.y), amx_ftoc(position.z), amx_ftoc(speed), amx_ftoc(rotation.x), amx_ftoc(rotation.y), amx_ftoc(rotation.z) });
}

void place(Body& body, Vector3 position, Vector3 rotation) {
	State& s = state();
	if (body.sync == Sync::Object && s.objects != nullptr) {
		IObject* object = s.objects->get(body.objectid);
		if (object == nullptr) {
			return;
		}
		if (object->isMoving()) {
			object->stop();
			++s.updates;
		}
		object->setPosition(position);
		object->setRotation(GTAQuat(rotation));
		s.updates += 2;
	} else if (body.sync == Sync::Script) {
		callMover(body, position, 0.0f, rotation);
		++s.updates;
	}
	body.from = position;
	body.to = position;
	body.rotation = rotation;
	body.speed = 0.0f;
	body.duration = 0.0;
	body.sentAt = s.now;
}

void move(Body& body, Vector3 client, Vector3 target, float speed, Vector3 rotation, float horizon) {
	State& s = state();
	if (body.sync == Sync::Object && s.objects != nullptr) {
		IObject* object = s.objects->get(body.objectid);
		if (object == nullptr) {
			return;
		}
		object->move(ObjectMoveData { target, rotation, speed });
	} else if (body.sync == Sync::Script) {
		callMover(body, target, speed, rotation);
	}
	++s.updates;
	body.from = client;
	body.to = target;
	body.rotation = rotation;
	body.speed = speed;
	body.duration = horizon;
	body.sentAt = s.now;
}

}

void syncBodies() {
	State& s = state();
	JPH::BodyIDVector active;
	s.system->GetActiveBodies(JPH::EBodyType::RigidBody, active);
	JPH::BodyInterface& bodies = s.system->GetBodyInterfaceNoLock();
	float horizon = s.settings.syncHorizon;
	for (const JPH::BodyID& id : active) {
		uint64_t data = bodies.GetUserData(id);
		if (kindOf(data) != Kind::Object) {
			continue;
		}
		Body* body = findBody(indexOf(data));
		if (body == nullptr) {
			continue;
		}
		JPH::RVec3 position = bodies.GetPosition(id);
		if (position.GetZ() < s.settings.floor) {
			bodies.SetLinearAndAngularVelocity(id, JPH::Vec3::sZero(), JPH::Vec3::sZero());
			bodies.DeactivateBody(id);
			continue;
		}
		if (body->sync == Sync::None) {
			continue;
		}
		JPH::Vec3 velocity = bodies.GetLinearVelocity(id);
		Vector3 current(float(position.GetX()), float(position.GetY()), float(position.GetZ()));
		Vector3 client = clientPosition(*body);
		bool arrived = s.now >= body->sentAt + body->duration;
		bool moving = velocity.Length() > StillSpeed;
		if (glm::distance(current, client) < body->syncError && !(arrived && moving)) {
			continue;
		}
		Vector3 target = current + Vector3(velocity.GetX(), velocity.GetY(), velocity.GetZ()) * horizon;
		JPH::Quat rotation = bodies.GetRotation(id);
		JPH::Vec3 spin = bodies.GetAngularVelocity(id);
		if (spin.Length() > 0.001f) {
			rotation = (JPH::Quat::sRotation(spin.Normalized(), spin.Length() * horizon) * rotation).Normalized();
		}
		Vector3 euler = nearestEuler(toEuler(rotation), body->rotation);
		float speed = glm::distance(client, target) / horizon;
		if (speed < 0.01f) {
			continue;
		}
		move(*body, client, target, speed, euler, horizon);
	}
}

void syncRest(Body& body) {
	State& s = state();
	const JPH::BodyInterface& bodies = s.system->GetBodyInterfaceNoLock();
	JPH::RVec3 position = bodies.GetPosition(body.id);
	Vector3 euler = nearestEuler(toEuler(bodies.GetRotation(body.id)), body.rotation);
	place(body, Vector3(float(position.GetX()), float(position.GetY()), float(position.GetZ())), euler);
}

void syncTeleport(Body& body) {
	syncRest(body);
}

}
