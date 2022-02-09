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

void Plane::resolveCollision(RigidBody* actor2, vec2 contact)
{
	// Find r at collision point p
	vec2 contactDisplacement = contact - actor2->getPosition();

	// Debug, draw the contact point as a red circle and r as a red line
	aie::Gizmos::add2DCircle(contact, 2, 100, { 1, 0, 0, 1 });
	aie::Gizmos::add2DLine(actor2->getPosition(), contact, { 1, 0, 0, 1 });

	// Find the total relative velocity of actor2 (linear vel + r x w)
	vec2 vRel = actor2->getVelocity() + vec2(-actor2->getAngularVelocity() *contactDisplacement.y, actor2->getAngularVelocity() * contactDisplacement.x);

	// Using the equation for J from newton's law of restitution, find the magnitude of force due to the actors relative velocity and contact displacement
	float elasticity = (m_elasticity + actor2->getElasticity()) / 2;
	float contactDisplacementCrossNormalSquared = dot((contactDisplacement.x * m_normal.y - contactDisplacement.y * m_normal.x), (contactDisplacement.x * m_normal.y - contactDisplacement.y * m_normal.x));
	float impuluseMagnitude = (-(1 + elasticity) * dot(vRel, m_normal)) / ((1 / actor2->getMass()) + (contactDisplacementCrossNormalSquared / actor2->getMoment()));
	
	// This j is applied down the collision normal
	vec2 impulseForce = impuluseMagnitude * m_normal;
	actor2->applyForce(impulseForce, contactDisplacement);
}
