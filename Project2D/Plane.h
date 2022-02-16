#pragma once
#include "PhysicsObject.h"
#include "RigidBody.h"
#include <iostream>
class Plane :
    public PhysicsObject
{
public:
    Plane(vec2 normal, float distance, vec4 colour);
    ~Plane() {}

    virtual void fixedUpdate(vec2 gravity, float timeStep) {}
    virtual void draw();

    vec2 getNormal() { return m_normal; }
    float getOriginDistance() { return m_originDistance; }

protected:
    vec2 m_normal;
    float m_originDistance;
};

