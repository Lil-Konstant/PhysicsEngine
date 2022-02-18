#pragma once
#include "PhysicsObject.h"

/// <summary>
/// Plane is a static collision primitive class that derives from PhysicsObject, and implements the basic internal
/// structure of a 2D plane (a plane normal and a distance from the origin), as well as a means of drawing said plane.
/// </summary>
class Plane :
    public PhysicsObject
{
public:
    Plane(vec2 normal, float distance, vec4 colour);
    ~Plane() {}

    virtual void fixedUpdate(vec2 gravity, float timeStep) override {}
    void draw() override;

    // Getters
    vec2 getNormal() { return m_normal; }
    float getOriginDistance() { return m_originDistance; }

protected:
    vec2 m_normal;
    float m_originDistance;
};