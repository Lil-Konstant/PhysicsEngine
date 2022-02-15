#include "Spring.h"

Spring::Spring(RigidBody* body1, RigidBody* body2, vec4 colour, float springCoefficient, float restLength, float damping, vec2 contact1, vec2 contact2) : PhysicsObject(ShapeType::JOINT, true)
{
	m_body1 = body1;
	m_body2 = body2;
	m_springCoefficient = springCoefficient;
	m_restLength = restLength;
	m_damping = damping;
	m_contact1 = contact1;
	m_contact2 = contact2;

	m_colour = colour;
}

void Spring::fixedUpdate(vec2 gravity, float timeStep)
{
	// Get the world-space coordinates of the spring's anchor points
	vec2 p1 = getContact1();
	vec2 p2 = getContact2();

	// Find the distance and direction between them in world-space
	float length = distance(p1, p2);
	vec2 direction = normalize(p2 - p1);

	// Find the relatvie velocity along the spring for damping
	float relativeVelocity = (dot(m_body2->getVelocity() - m_body1->getVelocity(), direction));

	// F = -kx - bv
	float magnitude = (m_springCoefficient * (m_restLength - length)) - (m_damping * relativeVelocity);
	vec2 force = (direction * magnitude);

	if (m_body1) { m_body1->applyForce(-force * timeStep, p1 - m_body1->getPosition()); }
	if (m_body2) { m_body2->applyForce(force * timeStep, p2 - m_body2->getPosition()); }

}

void Spring::draw()
{
	aie::Gizmos::add2DLine(getContact1(), getContact2(), m_colour);
}
