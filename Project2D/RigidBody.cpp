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

	// Apply gravity to the rig
	applyForce(gravity * m_mass * timeStep, vec2(0,0));

	m_orientation += m_angularVelocity * timeStep;
}

/// <summary>
/// Applies both a linear and rotational force based on the input vector force and contact point
/// </summary>
/// <param name="force"></param>
/// <param name="contactPoint"></param>
void RigidBody::applyForce(vec2 force, vec2 contactDisplacement)
{
	m_velocity += force / getMass();
	m_angularVelocity += (force.y * contactDisplacement.x + force.x * contactDisplacement.y) / getMoment();
}

void RigidBody::resolveCollision(RigidBody* actor2, vec2 contact, vec2* collisionNormal)
{
	// If a collision normal has been passed then use it, otherwise calculate based on the actors centres
	vec2 normal = normalize(collisionNormal ? *collisionNormal : actor2->getPosition() - m_position);
	// Get the vector perpendicular to the collision normal
	vec2 perp(normal.y, -normal.x);

	float r1 = dot(contact - m_position, -perp);
	float r2 = dot(contact - actor2->getPosition(), perp);

	// Determine the total velocity at the contact points (v = rw)
	float v1 = dot(m_velocity, normal) - r1 * m_angularVelocity;
	float v2 = dot(actor2->m_velocity, normal) - r2 * actor2->m_angularVelocity;

	// They're moving closer so resolve the collision
	if (v1 > v2)
	{
		// Calculate the effective mass at contact point for each object
		float mass1 = 1.0f / (1.0f / m_mass + (r1 * r1) / m_moment);
		float mass2 = 1.0f / (1.0f / actor2->m_mass + (r2 * r2) / actor2->m_moment);

		float elasticity = (m_elasticity + actor2->m_elasticity) / 2;
		vec2 force = (1.0f + elasticity) * mass1 * mass2 / (mass1 + mass2) * (v1 - v2) * normal;

		// Apply equal and opposite forces
		applyForce(-force, contact - m_position);
		actor2->applyForce(force, contact - actor2->m_position);
	}

	//vec2 relativeVelocity = actor2->getVelocity() - m_velocity;

	//// From Newton's law of restitution equation
	//float elasticity = 1;
	//float impulseMagnitude = (dot(-(1 + elasticity) * relativeVelocity, normal)) / ((1 / m_mass) + (1 / actor2->getMass()));

	//// The impulse force on each actor is then jn/-jn
	//vec2 impulseForce = impulseMagnitude * normal;

	//float kePre = getKineticEnergy() + actor2->getKineticEnergy();

	/// First, apply the force to the input actor
	//	actor2->applyForce(force);

	//// Then, apply the equal but opposite force to this actor
	//applyForce(-force);

	//float kePost = getKineticEnergy() + actor2->getKineticEnergy();
	//float deltaKE = kePost - kePre;

	//if (deltaKE > kePost * 0.01f) { std::cout << "KE discrepency greater than 1%"; }
}
