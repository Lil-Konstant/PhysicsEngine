#include "RigidBody.h"
#include <iostream>

RigidBody::RigidBody(ShapeType shapeID, vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass) : PhysicsObject(shapeID)
{
	m_position = position;
	m_orientation = orientation;
	m_velocity = velocity;
	m_angularVelocity = angularVelocity;
	m_mass = mass;
}

void RigidBody::fixedUpdate(vec2 gravity, float timeStep)
{
	// Update the rigs position and rotation based on it's linear and angular velocity during the time step
	m_position += m_velocity * timeStep;
	m_orientation += m_angularVelocity * timeStep;

	// Apply gravity to the rig
	applyForce(gravity * m_mass * timeStep, vec2(0,0));
}

/// <summary>
/// Applies both a linear and rotational force based on the input vector force and contact point
/// </summary>
/// <param name="force"></param>
/// <param name="contactPoint"></param>
void RigidBody::applyForce(vec2 force, vec2 contactDisplacement)
{
	m_velocity += force / getMass();
	m_angularVelocity += (contactDisplacement.x * force.y - contactDisplacement.y * force.x) / getMoment();
}

void RigidBody::resolveCollision(RigidBody* actor2, vec2 contact, vec2 collisionNormal)
{
	// If a collision normal has been passed then use it, otherwise calculate based on the actors centres
	vec2 normal = normalize(collisionNormal == vec2(0,0) ? actor2->getPosition() - m_position : collisionNormal);
	vec2 relativeVelocity = actor2->getVelocity() - m_velocity;

	// From Newton's law of restitution
	float elasticity = (m_elasticity + actor2->m_elasticity) / 2;
	float impulseMagnitude = (dot(-(1 + elasticity) * relativeVelocity, normal)) / ((1 / m_mass) + (1 / actor2->getMass()));
	// The impulse force on each actor is then jn/-jn
	vec2 impulseForce = impulseMagnitude * normal;

	//float kePre = getKineticEnergy() + actor2->getKineticEnergy();

	// First, apply the force to the input actor
	actor2->applyForce(impulseForce, actor2->getPosition() - contact);

	// Then, apply the equal but opposite force to this actor
	applyForce(-impulseForce, m_position - contact);

	//float kePost = getKineticEnergy() + actor2->getKineticEnergy();
	//float deltaKE = kePost - kePre;

	//if (deltaKE > kePost * 0.01f) { std::cout << "KE discrepency greater than 1%"; }
}
