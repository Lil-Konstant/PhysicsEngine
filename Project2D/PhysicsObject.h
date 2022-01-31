#pragma once

#include <glm/vec2.hpp>
using namespace glm;

class PhysicsObject
{
protected:
	PhysicsObject();

public:
	virtual void fixedUpdate(vec2 gravity, float timeStep) = 0;
	virtual void draw() = 0;
	virtual void resetPosition() {};
};

