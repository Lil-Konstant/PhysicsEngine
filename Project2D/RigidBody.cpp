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

void RigidBody::resolveCollision(PhysicsObject* other, vec2 contact, vec2 collisionNormal)
{
    RigidBody* actor2 = dynamic_cast<RigidBody*>(other);

    // If a collision normal has been passed then use it, otherwise calculate based on the actors centres
    vec2 normal = normalize(collisionNormal == vec2(0, 0) ? actor2->getPosition() - m_position : collisionNormal);

    // Find r at collision point p for both actors
    vec2 contactDisplacementA = contact - m_position;
    vec2 contactDisplacementB = vec2(0, 0);
    // If this rigidbody is not colliding with a plane or kinematic object, calculate the contact displacement
    if (actor2 && !actor2->m_isKinematic)
    {
        contactDisplacementB = contact - actor2->getPosition();
    }

    // Debug, draw the contact point as a red circle and r as a red line
    //aie::Gizmos::add2DCircle(contact, 2, 100, { 1, 0, 0, 1 });
    //aie::Gizmos::add2DLine(m_position, contact, { 1, 0, 0, 1 });
    //aie::Gizmos::add2DLine(actor2->getPosition(), contact, { 1, 0, 0, 1 });

    // Find the total relative velocity between both actors (linear vel + r x w)
    vec2 velocityAtA = m_velocity + vec2(-m_angularVelocity * contactDisplacementA.y, m_angularVelocity * contactDisplacementA.x);
    vec2 velocityAtB = vec2(0, 0);
    // If this rigidbody is not colliding with a plane or kinematic object, calculate the velocity of actor2
    if (actor2 && !actor2->m_isKinematic)
    {
        velocityAtB = actor2->getVelocity() + vec2(actor2->getAngularVelocity() * -contactDisplacementB.y, actor2->getAngularVelocity() * contactDisplacementB.x);
    }

    vec2 vRel = velocityAtA - velocityAtB;

    if (dot(vRel, collisionNormal) > 0 || !actor2)
    {
        // Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
        float elasticity = (m_elasticity + other->getElasticity()) / 2;

        // Store the cross products (r x n)^2 for both contact displacements as they are long and obscure clarity
        float contactDisplacementACrossNormalSquared = dot((contactDisplacementA.x * normal.y - contactDisplacementA.y * normal.x), (contactDisplacementA.x * normal.y - contactDisplacementA.y * normal.x)) / m_moment;
        float contactDisplacementBCrossNormalSquared = 0;
        if (actor2 && !actor2->m_isKinematic)
        {
            contactDisplacementBCrossNormalSquared = dot((contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x), (contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x)) / actor2->getMoment();
        }

        float inverseMassA = 1 / m_mass;
        float inverseMassB = 0;
        if (actor2 && !actor2->m_isKinematic)
        {
            inverseMassB = 1 / actor2->getMass();
        }

        // Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
        float impuluseMagnitude = (-(1 + elasticity) * dot(vRel, normal)) / (inverseMassA + inverseMassB + contactDisplacementACrossNormalSquared + contactDisplacementBCrossNormalSquared);

        // This j is applied down the collision normal
        vec2 impulseForce = impuluseMagnitude * normal;
        applyForce(impulseForce, contactDisplacementA);
        if (actor2 && !actor2->m_isKinematic)
        {
            actor2->applyForce(-impulseForce, contactDisplacementB);
        }
    }
}
