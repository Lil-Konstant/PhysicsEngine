#pragma once

#include <glm/glm.hpp>
#include "Gizmos.h"
using namespace glm;

enum class ShapeType
{
	PLANE = 0,
	SPHERE,
	LAST = SPHERE
};

#define SHAPE_COUNT static_cast<int>(ShapeType::LAST) + 1

class PhysicsObject
{
protected:
	PhysicsObject(ShapeType shapeID) : m_shapeID(shapeID) {}

public:
	virtual void fixedUpdate(vec2 gravity, float timeStep) = 0;
	virtual void draw() = 0;
	virtual void resetPosition() {}

	int getShapeID() { return static_cast<int>(m_shapeID); }

protected:
	ShapeType m_shapeID;
};

