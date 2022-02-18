#include "OBB.h"

/// <summary>
/// OBB has no default constructor, the custom constructor for OBB takes a position, width and height, orientation, linear and
/// angular velocity, mass, and colour for the box. The function initialises it's member variables with the passed parameters, 
/// and then uses the mass and dimensions to calculate the moment of inertia using the equation for rectangles, I = 1/12 * m * (width^2 + height^2).
/// The float width and height passed in are stored internally as have x and y extents in a singular vec2.
/// </summary>
/// <param name="position">The vec2 position to spawn the OBB at.</param>
/// <param name="width">The x-axis length of the box.</param>
/// <param name="height">The y-axis length of the box.</param>
/// <param name="orientation">The rotation to spawn this box as, in radians.</param>
/// <param name="velocity">The starting linear velocity to spawn with.</param>
/// <param name="angularVelocity">The starting rotational velocity to spawn with.</param>
/// <param name="mass">The mass of the OBB.</param>
/// <param name="colour">The colour to draw this OBB as.</param>
OBB::OBB(vec2 position, float width, float height, float orientation, vec2 velocity, float angularVelocity, float mass, vec4 colour) : RigidBody(ShapeType::OBB, position, orientation, velocity, angularVelocity, mass)
{
	m_extents.x = width / 2;
	m_extents.y = height / 2;
	m_colour = colour;

	// Calculate the moment using the moment equation for a box about it's centroid (COM)
	m_moment = (mass * (width * width + height * height))/ 12;
}

/// <summary>
/// The draw() override for OBB simply gets the 4 corners of this OBB based on it's position
/// and current orientation, and draws two tris between them to appear as a box.
/// </summary>
void OBB::draw()
{
	vector<vec2> corners = getCorners();

	aie::Gizmos::add2DTri(corners[0], corners[1], corners[3], m_colour);
	aie::Gizmos::add2DTri(corners[0], corners[3], corners[2], m_colour);
}

/// <summary>
/// checkOBBCorners() is a utility function that is called during the OBB2OBB collision detection logic, and
/// checks to find if the passed OBB overlaps with this OBB. The function simply returns true if so and false
/// otherwise, but the parameters contact, numContact, pen and edgeNormal are all passed as references, and
/// are set or added to by this function if collisions are detected, to be used then in collision resolution.
/// The function uses a special case of SAT to check for overlapping boxes on this box's local axes.
/// </summary>
/// <param name="otherOBB">The other OBB to check overlap with, passed as a constant reference.</param>
/// <param name="contact">The summation of all contact points found during collision, passed as a reference.</param>
/// <param name="numContacts">The number of contact points found during collision.</param>
/// <param name="pen">The penetration amount, passed as a reference.</param>
/// <param name="edgeNormal">The collision normal, passed as a reference.</param>
/// <returns>Returns true if any of the otherOBB's corners overlap with this OBB.</returns>
bool OBB::checkOBBCorners(const OBB& otherOBB, vec2& contact, int& numContacts, float& pen, vec2& edgeNormal)
{
	float minX, maxX, minY, maxY;
	int numLocalContacts = 0;
	vec2 localContact(0, 0);
	bool first = true;

	vector<vec2> otherCorners = otherOBB.getCorners();
	for (vec2 otherCorner : otherCorners)
	{
		// Get the position of the other OBBs corner local to this OBBs axes
		vec2 cornerLocalPos( dot(otherCorner - m_position, m_localX), dot(otherCorner - m_position, m_localY));

		// Update the min/max extents of otherOBB along each axes of this OBBs space
		if (first || cornerLocalPos.x < minX) minX = cornerLocalPos.x;
		if (first || cornerLocalPos.x > maxX) maxX = cornerLocalPos.x;
		if (first || cornerLocalPos.y < minY) minY = cornerLocalPos.y;
		if (first || cornerLocalPos.y > maxY) maxY = cornerLocalPos.y;

		// If the other OBBs corner is inside this OBB, add it to the sum of contact points
		if (cornerLocalPos.x >= -m_extents.x && cornerLocalPos.x <= m_extents.x && cornerLocalPos.y >= -m_extents.y && cornerLocalPos.y <= m_extents.y)
		{
			numLocalContacts++;
			localContact += cornerLocalPos;
		}

		first = false;
	}

	// If the otherOBB is entirely separated on one axis of this box, we've found a separating axis and can exit early
	if (minX >= m_extents.x || maxX <= -m_extents.x || minY >= m_extents.y || maxY <= -m_extents.y)
	{
		return false;
	}
	// If none of the corners overlap in this box, return false (the boxes still may be colliding, so CheckOBBCorners() must be called on both boxes
	if (numLocalContacts == 0)
	{
		return false;
	}

	bool result = false;
	// Convert all local contacts into world space and divide by their number to find the average contact point
	contact += m_position + (localContact.x * m_localX + localContact.y * m_localY) / (float)numLocalContacts;
	numContacts++;

	// Find the minimum penetration vector as a penetration amount and normal
	// Checking for right face penetration
	float pen0 = m_extents.x - minX;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = m_localX;
		pen = pen0;
		result = true;
	}
	// Checking for left face penetration
	pen0 = maxX + m_extents.x;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = -m_localX;
		pen = pen0;
		result = true;
	}
	// Checking for top face penetration
	pen0 = m_extents.y - minY;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = m_localY;
		pen = pen0;
		result = true;
	}
	// Checking for bottom face penetration
	pen0 = maxY + m_extents.y;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = -m_localY;
		pen = pen0;
		result = true;
	}

	return result;
}

/// <summary>
/// Returns true if the vec2 point is inside this OBB.
/// </summary>
/// <param name="point">The input point to check against.</param>
/// <returns>True if point is inside this OBB.</returns>
bool OBB::isInside(vec2 point)
{
	vec2 pointDisplacement = point - m_position;
	// Convert the input point from world space to the local space of this OBB
	vec2 localPos = vec2(dot(pointDisplacement, m_localX), dot(pointDisplacement, m_localY));

	// If the point locally lies within all extents, then is lies within this OBB
	return localPos.x >= -m_extents.x && localPos.x <= m_extents.x && localPos.y >= -m_extents.y && localPos.y <= m_extents.y;
}

/// <summary>
/// getCorners() simply uses the current m_position of this OBB, and the current local X and Y axis
/// vectors to find the positions of each corner of the box in world space coordinates. The function
/// returns these corners in an STL vector of Vec2's.
/// </summary>
/// <returns></returns>
vector<vec2> OBB::getCorners() const
{
	vector<vec2> corners (4);

	corners[0] = m_position - m_localX * m_extents.x - m_localY * m_extents.y;
	corners[1] = m_position + m_localX * m_extents.x - m_localY * m_extents.y;
	corners[2] = m_position - m_localX * m_extents.x + m_localY * m_extents.y;
	corners[3] = m_position + m_localX * m_extents.x + m_localY * m_extents.y;

	return corners;
}
