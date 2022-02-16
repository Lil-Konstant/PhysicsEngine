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
	m_isActive = true;
	m_colour = colour;
}

void Spring::fixedUpdate(vec2 gravity, float timeStep)
{
	if (m_isActive)
	{
		// Get the world-space coordinates of the spring's anchor points
		vec2 p1 = getContact1();
		vec2 p2 = getContact2();

		// Find the distance and direction between them in world-space
		float length = distance(p1, p2);
		vec2 direction = normalize(p2 - p1);

		vec2 springDisplacement1 = p1 - m_body1->getPosition();
		vec2 body1Velocity = m_body1->getVelocity() + vec2(-m_body1->getAngularVelocity() * springDisplacement1.y, m_body1->getAngularVelocity() * springDisplacement1.x);

		vec2 body2Velocity = vec2(0, 0);
		if (m_body2)
		{
			vec2 springDisplacement2 = p2 - m_body2->getPosition();
			vec2 body2Velocity = m_body2->getVelocity() + vec2(-m_body2->getAngularVelocity() * springDisplacement2.y, m_body2->getAngularVelocity() * springDisplacement2.x);
		}

		vec2 vRel = body2Velocity - body1Velocity;

		// Find the relatvie velocity along the spring for damping
		float relativeVelocity = (dot(vRel, direction));

		// F = -kx - bv
		float magnitude = (m_springCoefficient * (m_restLength - length)) - (m_damping * relativeVelocity);
		vec2 force = (direction * magnitude);

		if (m_body1 && !m_body1->getIsKinematic()) { m_body1->applyForce(-force * timeStep, p1 - m_body1->getPosition()); }
		if (m_body2 && !m_body2->getIsKinematic()) { m_body2->applyForce(force * timeStep, p2 - m_body2->getPosition()); }
	}

}

void Spring::draw()
{
	if (m_isActive) { aie::Gizmos::add2DLine(getContact1(), getContact2(), m_colour); }
}
