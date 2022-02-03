#include "Plane.h"

void Plane::draw()
{
	float lineSegmentLength = 300;
	vec4 colourFade = m_colour;
	colourFade.a = 0;
	vec2 centrePoint = m_normal * m_originDistance;

	vec2 parallel(m_normal.y, -m_normal.x);

	vec2 start = centrePoint + (parallel * lineSegmentLength);
	vec2 end = centrePoint - (parallel * lineSegmentLength);
	aie::Gizmos::add2DTri(start, end, start - (m_normal * 10.0f), m_colour, m_colour, colourFade);
	aie::Gizmos::add2DTri(end, end - (m_normal * 10.0f), start - m_normal, m_colour, m_colour, colourFade);
}

void Plane::resolveCollision(RigidBody* actor2)
{
	// We assume the collision normal to be the difference in positions (correct for sphere's with no friction)
	vec2 collisionNormal = m_normal;
	vec2 relativeVelocity = actor2->getVelocity();

	// From Newton's law of restitution equation
	float elasticity = 1;
	float impulseMagnitude = (dot(-(1 + elasticity) * relativeVelocity, collisionNormal)) / (1 / actor2->getMass());

	// The impulse force on each actor is then jn/-jn
	vec2 impulseForce = impulseMagnitude * collisionNormal;

	float kePre = actor2->getKineticEnergy();

	actor2->applyForce(impulseForce);

	float kePost = actor2->getKineticEnergy();
	float deltaKE = kePost - kePre;

	if (deltaKE > kePost * 0.01f) { std::cout << "KE discrepency greater than 1%"; }
}
