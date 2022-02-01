#pragma once
#include "PhysicsObject.h"
class Plane :
    public PhysicsObject
{
public:
    Plane(vec2 normal, float distance, vec4 colour) : PhysicsObject(ShapeType::PLANE), m_normal(normal), m_originDistance(distance), m_colour(colour) {}
    ~Plane();

    virtual void fixedUpdate(vec2 gravity, float timeStep) {}
    virtual void draw();

    vec2 getNormal() { return m_normal; }
    float getOriginDistance() { return m_originDistance; }

protected:
    vec2 m_normal;
    vec4 m_colour;
    float m_originDistance;
};

