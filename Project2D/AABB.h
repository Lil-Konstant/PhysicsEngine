#pragma once
#include "RigidBody.h"
#include <vector>

// --------------------- NOT USED IN SUBMISSION ----------------------- //

using namespace std;

class AABB :
    public RigidBody
{
public:
    AABB(vec2 position, float width, float height, vec2 velocity, float mass, vec4 colour);
    ~AABB() {}

    void fixedUpdate(vec2 gravity, float timeStep) override;
    void draw() override;

    vector<vec2> getCorners() const;
    vec2 getExtents();

protected:
    vec2 m_extents;
    vec4 m_colour;
};

// --------------------- NOT USED IN SUBMISSION ----------------------- //