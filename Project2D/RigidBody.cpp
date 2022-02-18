#include "RigidBody.h"

/// <summary>
/// RigidBody has no default constructor, the custom constructor takes a ShapeID for the shape of the collision primitive, a position,
/// linear and angular velocity, orientation and mass as parameters. The function simply sets the member variables with the passed
/// parameters, calls the constructor on the PhysicsObject base class to pass the ShapeID, and also uses the starting orientation
/// to calculate the starting values for the local X and Y axis vectors of this rigidbody.
/// </summary>
/// <param name="shapeID">The ShapeID for the child collision primitive.</param>
/// <param name="position">The starting position of this rigidbody.</param>
/// <param name="orientation">The starting orientation of this rigidbody.</param>
/// <param name="velocity">The starting velocity of this rigidbody.</param>
/// <param name="angularVelocity">The starting angular velocity of this rigidbody.</param>
/// <param name="mass">The mass of this rigidbody.</param>
RigidBody::RigidBody(ShapeType shapeID, vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass) : PhysicsObject(shapeID)
{
	m_position = position;
	m_orientation = orientation;
	m_velocity = velocity;
	m_angularVelocity = angularVelocity;
	m_mass = mass;

    // Calculate and store the initial local axis vectors based on the OBBs starting orientation
    float cs = cosf(m_orientation);
    float sn = sinf(m_orientation);
    m_localX = normalize(vec2(cs, sn));
    m_localY = normalize(vec2(-sn, cs));
}

/// <summary>
/// fixedUpdate first recalculates the local axis vectors based on the body's current orientation.
/// It then updates the position and orientation of this body based on it's current linear and
/// angular velocity (scaled by the fixed time step), and finally applies gravity to the object
/// if gravity is non-zero and this body is not kinematic.
/// </summary>
/// <param name="gravity">The vec2 value of gravity for the physics sim.</param>
/// <param name="timeStep">The fixed time step of the sim.</param>
void RigidBody::fixedUpdate(vec2 gravity, float timeStep)
{
    // Update the rigs position and rotation based on it's linear and angular velocity during the time step
    m_position += m_velocity * timeStep;
    m_orientation += m_angularVelocity * timeStep;

    // Calculate and store the local axis vectors based on the OBBs new orientation
    float cs = cosf(m_orientation);
    float sn = sinf(m_orientation);
    m_localX = normalize(vec2(cs, sn));
    m_localY = normalize(vec2(-sn, cs));

	// Apply gravity to the rig
    if (!m_isKinematic) { applyForce(gravity * m_mass * timeStep, vec2(0, 0)); }
}

/// <summary>
/// applyForce() simply applies both a linear and rotational force based on the input vector force 
/// and contact displacement of the force application (the contact point minus the position of this
/// body), and uses F = ma to apply these forces to the body's linear and rotational velocity as
/// an impulse force, based on the bodies mass and moment of inertia.
/// </summary>
/// <param name="force">The vec2 force to apply to this body.</param>
/// <param name="contactPoint">The point of force application on this body.</param>
void RigidBody::applyForce(vec2 force, vec2 contactDisplacement)
{
	m_velocity += force / getMass();
	m_angularVelocity += (contactDisplacement.x * force.y - contactDisplacement.y * force.x) / getMoment();
}

/// <summary>
/// resolveCollision() is a mathematical function that takes the parameters of the other object being collided with,
/// the point of contact between the two, and the collision normal, and uses these to calculate the restitution force
/// between the two bodies. The function has 3 special cases of collision, the first being if this body is kinematic and
/// colliding with a plane, the second being if this body is kinematic and colliding with a non-static rigidbody, and the
/// final being if this body is non-kinematic. One the function has calculated the appropriate force of restitution, it calls
/// applyForce on the appropriate bodies depending on which special case of collision it's dealing with.
/// </summary>
/// <param name="other">The other PhysicsObject this rigidbody is colliding with, either a plane or a rigid body.</param>
/// <param name="contact">The contact point in world coords of collision.</param>
/// <param name="collisionNormal">A vec2 that is normal to the plane of collision.</param>
void RigidBody::resolveCollision(PhysicsObject* other, vec2 contact, vec2 collisionNormal)
{
    // actor2 will be nullptr if other is a plane, otherwise it will be non-null
    RigidBody* actor2 = dynamic_cast<RigidBody*>(other);

    // If a collision normal has been passed then use it, otherwise calculate based on the actors centres
    vec2 normal = normalize(collisionNormal == vec2(0, 0) ? actor2->getPosition() - m_position : collisionNormal);

    // Find the contact displacement at collision point p for both actors (if actor2 is a plane then set B to 0)
    vec2 contactDisplacementA = contact - m_position;
    vec2 contactDisplacementB = actor2 ? contact - actor2->getPosition() : vec2(0,0);

    // Find the total relative velocity between both actors (linear vel + r x w) - if actor2 is a plane then set B to 0
    vec2 velocityAtA = m_velocity + vec2(-m_angularVelocity * contactDisplacementA.y, m_angularVelocity * contactDisplacementA.x);
    vec2 velocityAtB = actor2 ? actor2->getVelocity() + vec2(actor2->getAngularVelocity() * -contactDisplacementB.y, actor2->getAngularVelocity() * contactDisplacementB.x) : vec2(0, 0);
    vec2 vRel = velocityAtA - velocityAtB;

    // If the total relative velocity along the collision plane is positive, or we are colliding with a plane, then resolve collision
    if (dot(vRel, collisionNormal) > 0 || !actor2)
    {
        // If both bodies are kinematic, return early
        if ((m_isKinematic && actor2) && other->getIsKinematic()) { return; }

        // The total elasticity of the system is just the average of the two actor's elasticities
        float elasticity = (m_elasticity + other->getElasticity()) / 2;

        // Used to store (r x n)^2 / I in the equation for J for simplicity
        float contactDisplacementACrossNormalSquared = 0;
        float contactDisplacementBCrossNormalSquared = 0;
        // used to store 1 / mass in the equation for J for simplicity
        float inverseMassA = 0;
        float inverseMassB = 0;

        // If this rigid body is kinematic and colliding with a plane, we still want it to resolve on the kinematic rigidbody
        if (m_isKinematic && !actor2)
        {
            // Ignore these for B, as it is a plane
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

        // If this rigidbody is kinematic and colliding with a non-kinematic object, leave this object's J variables as 0 and resolve on just actor2
        else if (m_isKinematic)
        {
            // Ignore these for A, as it is kinematic
            contactDisplacementBCrossNormalSquared = dot((contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x), (contactDisplacementB.x * normal.y - contactDisplacementB.y * normal.x)) / actor2->getMoment();
            inverseMassB = 1 / actor2->getMass();

            // Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
            float impuluseMagnitude = (-(1 + elasticity) * dot(vRel, normal)) / (inverseMassA + inverseMassB + contactDisplacementACrossNormalSquared + contactDisplacementBCrossNormalSquared);
            // This j is applied down the collision normal
            vec2 impulseForce = impuluseMagnitude * normal;

            // Resolve on just actor2 and not the kinematic body
            actor2->applyForce(-impulseForce, contactDisplacementB);
            return;
        }

        // Otherwise attempt to resolve as normal (with checks for if the other rigidbody is kinematic)
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
