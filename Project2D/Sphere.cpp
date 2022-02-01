#include "Sphere.h"

Sphere::Sphere(vec2 position, vec2 velocity, float mass, float radius, vec4 colour) : RigidBody(ShapeType::SPHERE, position, velocity, 0, mass)
{
	m_radius = radius;
	m_colour = colour;
}

void Sphere::draw()
{
	aie::Gizmos::add2DCircle(m_position, m_radius, 100, m_colour);
}
