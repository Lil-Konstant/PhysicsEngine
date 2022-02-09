#pragma once
#include "RigidBody.h"
#include <vector>

using namespace std;

class OBB :
    public RigidBody
{
public:
    OBB(vec2 position, float width, float height, float orientation, vec2 velocity, float angularVelocity, float mass, vec4 colour);
    ~OBB() {}

    void fixedUpdate(vec2 gravity, float timeStep) override;
    void draw() override;

    // Used for OBB2OBB collision detection
    bool checkOBBCorners(const OBB& obb, vec2& contact, int& numContacts, float& pen, vec2& edgeNormal);

    // Getters
    vec2 getExtents() const { return m_extents; }
    float getWidth() const { return m_extents.x * 2; }
    float getHeight() const { return m_extents.y * 2; }
    vec2 getLocalX() const { return m_localX; }
    vec2 getLocalY() const { return m_localY; }
    vector<vec2> getCorners() const;

protected:
    vec2 m_extents; // half-edge extents
    vec4 m_colour;

    // Store the x and y axis vectors based on the OBBs current m_orientation
    vec2 m_localX;
    vec2 m_localY;
};

