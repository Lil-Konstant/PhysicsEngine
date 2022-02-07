#pragma once

#include "PhysicsObject.h"

using namespace glm;

class RigidBody : public PhysicsObject
{
public:
	RigidBody(ShapeType shapeID, vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass);
	~RigidBody() {}

	virtual void fixedUpdate(vec2 gravity, float timeStep);
	void applyForce(vec2 force, vec2 contactPoint);
	//void applyForceToActor(RigidBody* actor2, vec2 force);
	void resolveCollision(RigidBody* actor2, vec2 contact, vec2* collisionNormal = nullptr);

	float getKineticEnergy() { return 0.5f * m_mass * glm::length(m_velocity) * glm::length(m_velocity); }
	vec2 getPosition() { return m_position; }
	float getOrientation() { return m_orientation; }
	vec2 getVelocity() { return m_velocity; }
	float getMass() { return m_mass; }
	float getMoment() { return m_moment; }

	void setVelocity(vec2 value) { m_velocity = value; }

protected:
	vec2 m_position;
	vec2 m_velocity;
	float m_mass;
	float m_orientation;
	float m_angularVelocity;
	float m_moment;
};

