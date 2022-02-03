#include "RigidBody.h"
#include <iostream>

RigidBody::RigidBody(ShapeType shapeID, vec2 position, vec2 velocity, float orientation, float mass) : PhysicsObject(shapeID)
{
	m_position = position;
	m_velocity = velocity;
	m_orientation = orientation;
	m_mass = mass;
}

void RigidBody::fixedUpdate(vec2 gravity, float timeStep)
{
	// Move the rigs position based on it's linear velocity during the time step
	m_position += m_velocity * timeStep;

	// Apply gravity to the rig
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

void RigidBody::resolveCollision(RigidBody* actor2)
{
	// We assume the collision normal to be the difference in positions (correct for sphere's with no friction)
	vec2 collisionNormal = normalize(actor2->getPosition() - m_position);
	vec2 relativeVelocity = actor2->getVelocity() - m_velocity;

	// From Newton's law of restitution equation
	float elasticity = 1;
	float impulseMagnitude = (dot(-(1 + elasticity) * relativeVelocity, collisionNormal)) / ((1 / m_mass) + (1 / actor2->getMass()));

	// The impulse force on each actor is then jn/-jn
	vec2 impulseForce = impulseMagnitude * collisionNormal;

	float kePre = getKineticEnergy() + actor2->getKineticEnergy();

	applyForceToActor(actor2, impulseForce);

	float kePost = getKineticEnergy() + actor2->getKineticEnergy();
	float deltaKE = kePost - kePre;

	if (deltaKE > kePost * 0.01f) { std::cout << "KE discrepency greater than 1%"; }
}
