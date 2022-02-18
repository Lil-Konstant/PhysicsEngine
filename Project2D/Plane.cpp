#include "Plane.h"

/// <summary>
/// Plane has no default constructor, the custom constructor for plane takes a vec2 representing the normal direction of the plane, a float
/// representing the planes closest distance from the origin, as well as the colour to draw the plane (as a vec4).
/// </summary>
/// <param name="normal">The facing direction of the plane.</param>
/// <param name="distance">The shortest magnitude distance to the origin.</param>
/// <param name="colour">The colour to draw the plane as.</param>
Plane::Plane(vec2 normal, float distance, vec4 colour) : PhysicsObject(ShapeType::PLANE, true, 1.0f)
{
	m_normal = normal;
	m_originDistance = distance;
	m_colour = colour;
}

/// <summary>
/// The draw() override for plane simply finds the centre point of the plane to draw from by projecting
/// along the plane normal by the origin distance amount. The function then draws to tris that run
/// along the planes surface to create a fade effect behind the plane.
/// </summary>
void Plane::draw()
{
	// Find the centre point on the plan to draw from, and the parallel vector that runs along the plane
	vec2 centrePoint = m_normal * m_originDistance;
	vec2 parallel(m_normal.y, -m_normal.x);

	// Set the alpha value of colourFade to 0 so the tri fades out
	vec4 colourFade = m_colour;
	colourFade.a = 0;

	// Translate up and down the plane face by an amount given by the line segment length
	float lineSegmentLength = 300;
	vec2 start = centrePoint + (parallel * lineSegmentLength);
	vec2 end = centrePoint - (parallel * lineSegmentLength);

	aie::Gizmos::add2DTri(start, end, start - (m_normal * 10.0f), m_colour, m_colour, colourFade);
	aie::Gizmos::add2DTri(end, end - (m_normal * 10.0f), start - (m_normal * 10.0f), m_colour, colourFade, colourFade);
}
