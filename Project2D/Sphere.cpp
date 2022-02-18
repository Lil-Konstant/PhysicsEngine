#include "Sphere.h"

/// <summary>
/// Sphere has no default constructor, the custom constructor for sphere takes a position, orientation, linear and
/// angular velocity, mass, radius, elasticity and colour for the sphere. The function initialises it's member variables
/// with the passed parameters, and then uses the mass and radius to calculate the moment of inertia using the equation
/// for circles, I = 1/2 * m * r^2.
/// </summary>
/// <param name="position">The position to spawn the sphere at.</param>
/// <param name="orientation">The orientation to spawn the sphere at (in radians).</param>
/// <param name="velocity">The starting velocity for the sphere.</param>
/// <param name="angularVelocity">The starting angular velocity for the sphere.</param>
/// <param name="mass">The mass of the sphere.</param>
/// <param name="radius">The radius of the sphere.</param>
/// <param name="elasticity">The collision elasticity for the sphere.</param>
/// <param name="colour"></param>
Sphere::Sphere(vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass, float radius, float elasticity, vec4 colour) : RigidBody(ShapeType::SPHERE, position, orientation, velocity, angularVelocity, mass)
{
	m_radius = radius;
	m_colour = colour;
	m_elasticity = elasticity;
	
	// Calculate the moment using the moment equation for a circle
	m_moment = 0.5f * mass * m_radius * m_radius;
}

/// <summary>
/// draw() is an override function which simply uses the add2DCircle function in the 
/// AIE Gizmos namespace to draw a circle of radius m_radius at m_position, the function
/// also draws a line from the centre of the circle out to the radius based on the circle's
/// current orientation, so as to visualise the rotation.
/// </summary>
void Sphere::draw()
{
	vec2 end = vec2(cos(m_orientation), sin(m_orientation)) * m_radius;
	aie::Gizmos::add2DCircle(m_position, m_radius, 100, m_colour);
	aie::Gizmos::add2DLine(m_position, m_position + end, vec4(0, 0, 0, 1));
}

/// <summary>
/// Returns true if the vec2 point is inside this sphere.
/// </summary>
/// <param name="point">The input point to check against.</param>
/// <returns>True if point is inside this sphere.</returns>
bool Sphere::isInside(vec2 point)
{
	return distance(point, m_position) <= m_radius;
}
