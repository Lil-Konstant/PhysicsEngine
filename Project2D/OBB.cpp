#include "OBB.h"

OBB::OBB(vec2 position, float width, float height, float orientation, vec2 velocity, float angularVelocity, float mass, vec4 colour) : RigidBody(ShapeType::OBB, position, orientation, velocity, angularVelocity, mass)
{
	m_extents.x = width / 2;
	m_extents.y = height / 2;
	m_colour = colour;

	// Calculate and store the local axis vectors based on the OBBs new orientation
	float cs = cosf(m_orientation);
	float sn = sinf(m_orientation);
	m_localX = normalize(vec2(cs, sn));
	m_localY = normalize(vec2(-sn, cs));

	// Calculate the moment using the moment equation for a box about it's centroid (COM)
	m_moment = (mass * (width * width + height * height))/ 12;
}

void OBB::fixedUpdate(vec2 gravity, float timeStep)
{
	RigidBody::fixedUpdate(gravity, timeStep);

	// Calculate and store the local axis vectors based on the OBBs new orientation
	float cs = cosf(m_orientation);
	float sn = sinf(m_orientation);
	m_localX = normalize(vec2(cs, sn));
	m_localY = normalize(vec2(-sn, cs));
}

void OBB::draw()
{
	vector<vec2> corners = getCorners();

	aie::Gizmos::add2DTri(corners[0], corners[1], corners[3], m_colour);
	aie::Gizmos::add2DTri(corners[0], corners[3], corners[2], m_colour);
}

bool OBB::checkOBBCorners(const OBB& otherOBB, vec2& contact, int& numContacts, float& pen, vec2& edgeNormal)
{
	float minX, maxX, minY, maxY;
	float otherOBBW = otherOBB.getWidth();
	float otherOBBH = otherOBB.getHeight();
	int numLocalContacts = 0;
	vec2 localContact(0, 0);
	bool first = true;

	vector<vec2> otherCorners = otherOBB.getCorners();
	for (vec2 otherCorner : otherCorners)
	{
		// Get the position of the other OBBs corner relative to this OBBs axes
		vec2 cornerLocalPos( dot(otherCorner - m_position, m_localX), dot(otherCorner - m_position, m_localY));

		// Update the min/max extents of otherOBB along each axes of this OBBs space
		if (first || cornerLocalPos.x < minX) minX = cornerLocalPos.x;
		if (first || cornerLocalPos.x > maxX) maxX = cornerLocalPos.x;
		if (first || cornerLocalPos.y < minY) minY = cornerLocalPos.x;
		if (first || cornerLocalPos.y > maxY) maxY = cornerLocalPos.x;

		// If the other OBBs corner is inside this OBB, add it to the list of contact points
		if (cornerLocalPos.x >= -m_extents.x && cornerLocalPos.x <= m_extents.x && cornerLocalPos.y >= -m_extents.y && cornerLocalPos.y <= m_extents.y)
		{
			numLocalContacts++;
			localContact += cornerLocalPos;
		}

		first = false;
	}

	// If the otherOBB lies entirely to one side of the box along one axis, we've found a separating axis and can exit early
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
	// Convert all local contact into world space and divide by their number to find the average contact point
	contact += m_position + (localContact.x * m_localX + localContact.y * m_localY) / (float)numLocalContacts;
	numContacts++;

	// Find the minimum penetration vecotr as a penetration amount and normal
	// Checking right face
	float pen0 = m_extents.x - minX;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = m_localX;
		pen = pen0;
		result = true;
	}
	// Checking left face
	pen0 = maxX + m_extents.x;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = -m_localX;
		pen = pen0;
		result = true;
	}
	// Checking top face
	pen0 = m_extents.y - minY;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = m_localY;
		pen = pen0;
		result = true;
	}
	// Checking bottom face
	pen0 = maxY + m_extents.y;
	if (pen0 > 0 && (pen0 < pen || pen == 0))
	{
		edgeNormal = -m_localY;
		pen = pen0;
		result = true;
	}

	return result;
}

vector<vec2> OBB::getCorners() const
{
	vector<vec2> corners (4);

	corners[0] = m_position - m_localX * m_extents.x - m_localY * m_extents.y;
	corners[1] = m_position + m_localX * m_extents.x - m_localY * m_extents.y;
	corners[2] = m_position - m_localX * m_extents.x + m_localY * m_extents.y;
	corners[3] = m_position + m_localX * m_extents.x + m_localY * m_extents.y;

	return corners;
}
