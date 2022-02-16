#pragma once

#include <glm/glm.hpp>
#include "Gizmos.h"
using namespace glm;

enum class ShapeType
{
	JOINT = -1,
	PLANE,
	SPHERE,
	AABB,
	OBB,
	SHAPE_COUNT
};

//#define SHAPE_COUNT 4

class PhysicsObject
{
protected:
	PhysicsObject(ShapeType shapeID, bool isKinematic = false, float elasticity = 1.0f) : m_shapeID(shapeID), m_isKinematic(isKinematic), m_elasticity(elasticity) {}

public:
	virtual void fixedUpdate(vec2 gravity, float timeStep) = 0;
	virtual void draw() = 0;
	virtual void resetPosition() {}

	virtual bool isInside(vec2 point) { return false; }

	int getShapeID() { return static_cast<int>(m_shapeID); }
	bool getIsKinematic() { return m_isKinematic; }
	float getElasticity() { return m_elasticity; }

	void setIsKinematic(bool value) { m_isKinematic = value; }

protected:
	ShapeType m_shapeID;
	vec4 m_colour;
	bool m_isKinematic;
	float m_elasticity;
};

