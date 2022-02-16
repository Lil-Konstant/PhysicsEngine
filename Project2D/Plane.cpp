#include "Plane.h"

Plane::Plane(vec2 normal, float distance, vec4 colour) : PhysicsObject(ShapeType::PLANE, true, 1.0f)
{
	m_normal = normal;
	m_originDistance = distance;
	m_colour = colour;
}

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
