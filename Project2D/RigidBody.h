#pragma once

#include "PhysicsObject.h"

using namespace glm;

class RigidBody : public PhysicsObject
{
public:
	RigidBody(ShapeType shapeID, vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass);
	~RigidBody() {}

	virtual void fixedUpdate(vec2 gravity, float timeStep) override;
	void applyForce(vec2 force, vec2 contactPoint);
	//void applyForceToActor(RigidBody* actor2, vec2 force);
	void resolveCollision(PhysicsObject* actor2, vec2 contact, vec2 collisionNormal = vec2(0,0));

	vec2 toWorld(vec2 localPoint) { return m_position + (localPoint.x * m_localX) + (localPoint.y * m_localY); }
	vec2 toLocal(vec2 worldPoint) { return vec2(dot(worldPoint - m_position, m_localX), dot(worldPoint - m_position, m_localY)); }

	// Getters
	float getKineticEnergy() { return 0.5f * m_mass * glm::length(m_velocity) * glm::length(m_velocity); }
	vec2 getPosition() { return m_position; }
	float getOrientation() { return m_orientation; }
	vec2 getVelocity() { return m_velocity; }
	float getAngularVelocity() { return m_angularVelocity; }
	float getMass() { return m_mass; }
	float getMoment() { return m_moment; }

	vec2 getLocalX() const { return m_localX; }
	vec2 getLocalY() const { return m_localY; }

	// Setter
	void setVelocity(vec2 value) { m_velocity = value; }
	void setAngularVelocity(float value) { m_angularVelocity = value; }

protected:
	vec2 m_position;
	vec2 m_velocity;
	float m_angularVelocity;

	// Store the x and y axis vectors based on the OBBs current m_orientation
	vec2 m_localX;
	vec2 m_localY;
	float m_orientation;

	float m_mass;
	float m_moment;
};

