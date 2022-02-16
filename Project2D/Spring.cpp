#include "Spring.h"

/// <summary>
/// Spring has no default constructor, the custom constructor for spring takes two rigid bodies to connect between, the local
/// contact points on each, parameters for the spring coefficient, rest length and damping, as well as the colour to draw the
/// spring. The second RigidBody can be left null and the spring will act solely on the first body (used for spring pulls with the mouse).
/// </summary>
/// <param name="body1">The first rigid body to attach to.</param>
/// <param name="body2">The second rigid body to attach to, can be nullptr.</param>
/// <param name="colour">The colour to draw the spring as.</param>
/// <param name="springCoefficient">Scalar that determines the strength of the spring.</param>
/// <param name="restLength">The length at which the spring will attempt to stay at.</param>
/// <param name="damping">Scalar value that represents friction in the spring.</param>
/// <param name="contact1">Local spring contact point on rigid body 1.</param>
/// <param name="contact2">Local spring contact point on rigid body 2.</param>
Spring::Spring(RigidBody* body1, RigidBody* body2, vec4 colour, float springCoefficient, float restLength, float damping, vec2 contact1, vec2 contact2) : PhysicsObject(ShapeType::JOINT, true)
{
	m_body1 = body1;
	m_body2 = body2;
	m_springCoefficient = springCoefficient;
	m_restLength = restLength;
	m_damping = damping;
	m_contact1 = contact1;
	m_contact2 = contact2;
	m_isActive = true;
	m_colour = colour;
}

/// <summary>
/// fixedUpdate() is a PhysicsObject override that implements the core spring physics of the class. Each call, the function calculates the worldspace distance
/// between the spring's two contact points, and will then use Hooke's law to apply a force along the contact point displacement given by F = -kx - bv, where x is
/// the restLength of the spring minus the current length of the spring, k is the spring coefficient, b is the damping coefficient, and v is the scalar value representing
/// the total relative velocity of the two rigid bodies that lies along the line of the spring. The force calculated for the spring is multiplied by the timeStep of the
/// simulation before being applied to either rig, as this force is an acceleration force, not an impulse force.
/// </summary>
/// <param name="gravity">A vector representing the gravity value of the simulation, not used in this function call.</param>
/// <param name="timeStep">The discrete time step of the physics simulation.</param>
void Spring::fixedUpdate(vec2 gravity, float timeStep)
{
	if (m_isActive)
	{
		// Get the world-space coordinates of the spring's anchor points
		vec2 p1 = getContact1();
		vec2 p2 = getContact2();

		// Find the length and direction between them in world-space
		float length = distance(p1, p2);
		vec2 direction = normalize(p2 - p1);

		// Find the total relative velocity of the two rigid bodies wrt the springs anchor points
		vec2 springDisplacement1 = p1 - m_body1->getPosition();
		vec2 body1Velocity = m_body1->getVelocity() + vec2(-m_body1->getAngularVelocity() * springDisplacement1.y, m_body1->getAngularVelocity() * springDisplacement1.x);
		vec2 body2Velocity = vec2(0, 0);
		if (m_body2)
		{
			vec2 springDisplacement2 = p2 - m_body2->getPosition();
			vec2 body2Velocity = m_body2->getVelocity() + vec2(-m_body2->getAngularVelocity() * springDisplacement2.y, m_body2->getAngularVelocity() * springDisplacement2.x);
		}
		vec2 vRel = body2Velocity - body1Velocity;

		// Find the magnitude of the total relative velocity along the spring for damping
		float relativeVelocity = (dot(vRel, direction));

		// Using F = -kx - bv
		float magnitude = (m_springCoefficient * (m_restLength - length)) - (m_damping * relativeVelocity);
		vec2 force = (direction * magnitude);

		// Apply equal and opposing forces to both bodies if they are non-null and non-kinematic
		if (m_body1 && !m_body1->getIsKinematic()) { m_body1->applyForce(-force * timeStep, p1 - m_body1->getPosition()); }
		if (m_body2 && !m_body2->getIsKinematic()) { m_body2->applyForce(force * timeStep, p2 - m_body2->getPosition()); }
	}

}

/// <summary>
/// draw() is a PhysicsObject override that simply draws a 2D line between the two contact points of the spring in world
/// coordinates.
/// </summary>
void Spring::draw()
{
	if (m_isActive) { aie::Gizmos::add2DLine(getContact1(), getContact2(), m_colour); }
}
