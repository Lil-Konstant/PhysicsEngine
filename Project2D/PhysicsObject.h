#pragma once

#include <glm/glm.hpp>
#include "Gizmos.h"
using namespace glm;

enum class ShapeType
{
	PLANE = 0,
	SPHERE,
	OBB
};

#define SHAPE_COUNT 3

class PhysicsObject
{
protected:
	PhysicsObject(ShapeType shapeID, bool isKinematic = false, float elasticity = 1.0f) : m_shapeID(shapeID), m_isKinematic(isKinematic), m_elasticity(elasticity) {}

public:
	virtual void fixedUpdate(vec2 gravity, float timeStep) = 0;
	virtual void draw() = 0;
	virtual void resetPosition() {}

	int getShapeID() { return static_cast<int>(m_shapeID); }
	float getElasticity() { return m_elasticity; }

protected:
	ShapeType m_shapeID;
	bool m_isKinematic;
	float m_elasticity;
};

