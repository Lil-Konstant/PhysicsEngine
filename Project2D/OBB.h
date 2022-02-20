#pragma once
#include "RigidBody.h"
#include <vector>

using namespace std;

/// <summary>
/// OBB is a collision primative class that implements the internal structure of a rotatable 2D box, as well
/// as the logic for determining if another OBB's corners overlaps with this OBB.
/// </summary>
class OBB :
    public RigidBody
{
public:
    OBB(vec2 position, float width, float height, float orientation, vec2 velocity, float angularVelocity, float mass, vec4 colour);
    ~OBB() {}

    void draw() override;

    bool isInside(vec2 point) override;
    // Used for OBB2OBB collision detection
    bool checkOBBCorners(const OBB& obb, vec2& contact, int& numContacts, float& pen, vec2& edgeNormal);

    // Getters
    vec2 getExtents() const { return m_extents; }
    float getWidth() const { return m_extents.x * 2; }
    float getHeight() const { return m_extents.y * 2; }
    vector<vec2> getCorners() const;

protected:
    vec2 m_extents; // half-edge extents
};

