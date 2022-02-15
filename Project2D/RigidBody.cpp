#include "RigidBody.h"
#include <iostream>

RigidBody::RigidBody(ShapeType shapeID, vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass) : PhysicsObject(shapeID)
{
	m_position = position;
	m_orientation = orientation;
	m_velocity = velocity;
	m_angularVelocity = angularVelocity;
	m_mass = mass;

    // Calculate and store the local axis vectors based on the OBBs new orientation
    float cs = cosf(m_orientation);
    float sn = sinf(m_orientation);
    m_localX = normalize(vec2(cs, sn));
    m_localY = normalize(vec2(-sn, cs));
}

void RigidBody::fixedUpdate(vec2 gravity, float timeStep)
{
    // Calculate and store the local axis vectors based on the OBBs new orientation
    float cs = cosf(m_orientation);
    float sn = sinf(m_orientation);
    m_localX = normalize(vec2(cs, sn));
    m_localY = normalize(vec2(-sn, cs));

	// Update the rigs position and rotation based on it's linear and angular velocity during the time step
	m_position += m_velocity * timeStep;
	m_orientation += m_angularVelocity * timeStep;

	// Apply gravity to the rig
    if (!m_isKinematic) { applyForce(gravity * m_mass * timeStep, vec2(0, 0)); }
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
    vec2 contactDisplacementB = actor2 ? contact - actor2->getPosition() : vec2(0,0);

    // Find the total relative velocity between both actors (linear vel + r x w)
    vec2 velocityAtA = m_velocity + vec2(-m_angularVelocity * contactDisplacementA.y, m_angularVelocity * contactDisplacementA.x);
    vec2 velocityAtB = actor2 ? actor2->getVelocity() + vec2(actor2->getAngularVelocity() * -contactDisplacementB.y, actor2->getAngularVelocity() * contactDisplacementB.x) : vec2(0, 0);
    vec2 vRel = velocityAtA - velocityAtB;

    if (dot(vRel, collisionNormal) > 0 || !actor2)
    {
        // If both bodies are kinematic, simply apply a contact force between them to keep them separated and return early
        if ((m_isKinematic && actor2) && other->getIsKinematic()) { return; }

        // Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
        float elasticity = (m_elasticity + other->getElasticity()) / 2;

        // (r x n)^2 / I
        float contactDisplacementACrossNormalSquared = 0;
        float contactDisplacementBCrossNormalSquared = 0;
        // 1 / mass
        float inverseMassA = 0;
        float inverseMassB = 0;

        // If this rigid body is kinematic and colliding with a plane, we still want it to resolve on the rigidbody
        if (m_isKinematic && !actor2)
        {
            contactDisplacementACrossNormalSquared = dot((contactDisplacementA.x * normal.y - contactDisplacementA.y * normal.x), (contactDisplacementA.x * normal.y - contactDisplacementA.y * normal.x)) / m_moment;
            inverseMassA = 1 / m_mass;

            // Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
            float impuluseMagnitude = (-(1 + elasticity) * dot(vRel, normal)) / (inverseMassA + inverseMassB + contactDisplacementACrossNormalSquared + contactDisplacementBCrossNormalSquared);
            // This j is applied down the collision normal
            vec2 impulseForce = impuluseMagnitude * normal;

            // Apply the force to just the kinematic body and not the plane
            applyForce(impulseForce, contactDisplacementA);
            return;
        }

        // If this rigid body is kinematic and colliding with a non-kinematic object, leave this objects variables j variables as 0 and resolve on just actor2
        else if (m_isKinematic)
        {
            contactDisplacementBCrossNormalSquared = dot((contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x), (contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x)) / actor2->getMoment();
            inverseMassB = 1 / actor2->getMass();

            // Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
            float impuluseMagnitude = (-(1 + elasticity) * dot(vRel, normal)) / (inverseMassA + inverseMassB + contactDisplacementACrossNormalSquared + contactDisplacementBCrossNormalSquared);
            // This j is applied down the collision normal
            vec2 impulseForce = impuluseMagnitude * normal;

            // Resolve on just actor2
            actor2->applyForce(-impulseForce, contactDisplacementB);
            return;
        }

        // Otherwise neither object is kinematic or a plane, so resolve as normal
        else
        {
            contactDisplacementACrossNormalSquared = dot((contactDisplacementA.x * normal.y - contactDisplacementA.y * normal.x), (contactDisplacementA.x * normal.y - contactDisplacementA.y * normal.x)) / m_moment;
            inverseMassA = 1 / m_mass;
            if (!other->getIsKinematic()) 
            {
                contactDisplacementBCrossNormalSquared = dot((contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x), (contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x)) / actor2->getMoment();
                inverseMassB = 1 / actor2->getMass();
            }

            // Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
            float impuluseMagnitude = (-(1 + elasticity) * dot(vRel, normal)) / (inverseMassA + inverseMassB + contactDisplacementACrossNormalSquared + contactDisplacementBCrossNormalSquared);
            // This j is applied down the collision normal
            vec2 impulseForce = impuluseMagnitude * normal;

            // Resolve on both bodies as neither are static
            applyForce(impulseForce, contactDisplacementA);
            if (!other->getIsKinematic()) { actor2->applyForce(-impulseForce, contactDisplacementB); }
        }
    }
}
