#pragma once

#include "RigidBody.h"

using namespace glm;

class Sphere :
    public RigidBody
{
public:
    Sphere(vec2 position, vec2 velocity, float mass, float radius, vec4 colour);
    ~Sphere();

    virtual void draw();

    float getRadius() { return m_radius; }
    vec4 getColour() { return m_colour; }

protected:
    float m_radius;
    vec4 m_colour;
};

