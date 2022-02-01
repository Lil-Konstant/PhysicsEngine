#pragma once

#include <glm/glm.hpp>
#include "Gizmos.h"
using namespace glm;

enum class ShapeType
{
	PLANE = 0,
	SPHERE,
	BOX
};

class PhysicsObject
{
protected:
	PhysicsObject(ShapeType shapeID) : m_shapeID(shapeID) {}

public:
	virtual void fixedUpdate(vec2 gravity, float timeStep) = 0;
	virtual void draw() = 0;
	virtual void resetPosition() {}

protected:
	ShapeType m_shapeID;
};

