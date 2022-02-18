#pragma once
#include "PhysicsObject.h"

using namespace glm;

/// <summary>
/// RigidBody is a pure abstract class that derives from PhysicsObject, and further
/// extends functionality to implement the logic for rigid body dynamics, which includes
/// a fixedUpdate override which updates the position and rotation of the object, an apply
/// force function for applying linear force and torque, and a resolve collision function
/// which uses Newton's law of restitution to model collisions. The member variables of
/// RigidBody store the position, velocity, orientation, mass and moment of the object.
/// Each primitive that derives from RigidBody also uses a local X and Y axis vector to
/// configure it's rotation.
/// </summary>
class RigidBody : public PhysicsObject
{
public:
	RigidBody(ShapeType shapeID, vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass);
	~RigidBody() {}

	// Physics implementers
	virtual void fixedUpdate(vec2 gravity, float timeStep) override;
	void applyForce(vec2 force, vec2 contactPoint);
	void resolveCollision(PhysicsObject* actor2, vec2 contact, vec2 collisionNormal = vec2(0,0));

	// Conversion functions to convert between local and world coordinates based on the local axes of this rigidbody
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
	// Setters
	void setVelocity(vec2 value) { m_velocity = value; }
	void setAngularVelocity(float value) { m_angularVelocity = value; }

protected:
	vec2 m_position;
	vec2 m_velocity;
	float m_angularVelocity;
	float m_mass;
	float m_moment;

	// Store the x and y axis vectors based on the OBBs current orientation
	vec2 m_localX;
	vec2 m_localY;
	float m_orientation;
};

