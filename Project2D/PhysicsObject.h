#pragma once

#include <glm/glm.hpp>
#include "Gizmos.h"
using namespace glm;

enum class ShapeType
{
	PLANE = 0,
	SPHERE,
	AABB,
	OBB
};

#define SHAPE_COUNT 4

class PhysicsObject
{
protected:
	PhysicsObject(ShapeType shapeID, bool isKinematic = false, float elasticity = 1.0f) : m_shapeID(shapeID), m_isKinematic(isKinematic), m_elasticity(elasticity) {}

public:
	virtual void fixedUpdate(vec2 gravity, float timeStep) = 0;
	virtual void draw() = 0;
	virtual void resetPosition() {}

	virtual bool isInside(vec2 point) {}

	int getShapeID() { return static_cast<int>(m_shapeID); }
	float getElasticity() { return m_elasticity; }

protected:
	ShapeType m_shapeID;
	bool m_isKinematic;
	float m_elasticity;
};

