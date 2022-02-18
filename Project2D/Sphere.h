#pragma once
#include "RigidBody.h"

using namespace glm;

/// <summary>
/// Sphere is a collision primitive class that derives from RigidBody, and implements the basic internal
/// structure of a 2D circle, as well as a means of drawing said circle.
/// </summary>
class Sphere :
    public RigidBody
{
public:
    Sphere(vec2 position, float orientation, vec2 velocity, float angularVelocity, float mass, float radius, float elasticity, vec4 colour);
    ~Sphere() {}

    // Draws the sphere class as a 2D circle
    void draw() override;

    bool isInside(vec2 point) override;
    
    // Getters
    float getRadius() { return m_radius; }
    vec4 getColour() { return m_colour; }

protected:
    float m_radius;
};

