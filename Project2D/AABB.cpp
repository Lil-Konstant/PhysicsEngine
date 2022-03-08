#include "AABB.h"
// --------------------- NOT USED IN SUBMISSION ----------------------- //

AABB::AABB(vec2 position, float width, float height, vec2 velocity, float mass, vec4 colour) : RigidBody(ShapeType::AABB, position, 0, velocity, 0, mass)
{
	m_extents.x = width / 2;
	m_extents.y = height / 2;
	m_colour = colour;

	m_moment = 1;
}

void AABB::fixedUpdate(vec2 gravity, float timeStep)
{
	m_orientation = 0;
	m_angularVelocity = 0;

	RigidBody::fixedUpdate(gravity, timeStep);
}

void AABB::draw()
{
	aie::Gizmos::add2DAABB(m_position, m_extents, m_colour);
}

vector<vec2> AABB::getCorners() const
{
	vector<vec2> corners(4);

	corners[0] = m_position - m_extents;
	corners[1] = m_position + (m_extents.x * vec2(1, 0)) - (m_extents.y * vec2(0, 1));
	corners[2] = m_position - (m_extents.x * vec2(1, 0)) + (m_extents.y * vec2(0, 1));
	corners[3] = m_position + m_extents;

	return corners;
}

vec2 AABB::getExtents()
{
	return m_extents;
}

// --------------------- NOT USED IN SUBMISSION ----------------------- //