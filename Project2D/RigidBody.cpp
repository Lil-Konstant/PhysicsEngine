#include "RigidBody.h"

RigidBody::RigidBody(ShapeType shapeID, vec2 position, vec2 velocity, float orientation, float mass) : PhysicsObject(shapeID)
{
	m_position = position;
	m_velocity = velocity;
	m_orientation = orientation;
	m_mass = mass;
}

void RigidBody::fixedUpdate(vec2 gravity, float timeStep)
{
	m_position += m_velocity * timeStep;
	applyForce(gravity * m_mass * timeStep);
}

void RigidBody::applyForce(vec2 force)
{
	m_velocity += force / m_mass;
}

void RigidBody::applyForceToActor(RigidBody* actor2, vec2 force)
{
	// First, apply the force to the input actor
	actor2->applyForce(force);

	// Then, apply the equal but opposite force to this actor
	applyForce(-force);
}
