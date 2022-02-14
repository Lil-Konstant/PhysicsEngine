#include "Sphere.h"

Sphere::Sphere(vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass, float radius, float elasticity, vec4 colour) : RigidBody(ShapeType::SPHERE, position, orientation, velocity, angularVelocity, mass)
{
	m_radius = radius;
	m_colour = colour;
	m_elasticity = elasticity;
	// Calculate the moment using the moment equation for a circle
	m_moment = 0.5f * mass * m_radius * m_radius;
}

void Sphere::draw()
{
	vec2 end = vec2(cos(m_orientation), sin(m_orientation)) * m_radius;

	aie::Gizmos::add2DCircle(m_position, m_radius, 100, m_colour);
	aie::Gizmos::add2DLine(m_position, m_position + end, vec4(0, 0, 0, 1));
}

bool Sphere::isInside(vec2 point)
{
	return distance(point, m_position) <= m_radius;
}
