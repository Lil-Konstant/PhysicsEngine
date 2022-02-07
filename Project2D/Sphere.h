#pragma once

#include "RigidBody.h"

using namespace glm;

class Sphere :
    public RigidBody
{
public:
    Sphere(vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass, float radius, float elasticity, vec4 colour);
    ~Sphere();

    virtual void draw();

    float getRadius() { return m_radius; }
    vec4 getColour() { return m_colour; }

protected:
    float m_radius;
    vec4 m_colour;
};

