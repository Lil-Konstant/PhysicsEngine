#pragma once
#include <glm/glm.hpp>
#include "Gizmos.h"

using namespace glm;

/// <summary>
/// A ShapeType is assigned to each collision primitive that derives from PhysicsObject, and
/// is used to determine which collision detection routine to use when two shapes are colliding.
/// Joints (springs) are assigned a ShapeType of -1 so as to be ignored in collision detection.
/// </summary>
enum class ShapeType
{
	JOINT = -1,
	PLANE,
	SPHERE,
	AABB,
	OBB,
	SHAPE_COUNT
};

/// <summary>
/// PhysicsObject is the base class that all collision primitives and scene objects derive from.
/// It is a pure abstract class, and implements the skeleton of pure virtual fixedUpdate and
/// draw functions that children must override to be instantiatable. The member variables store
/// the shapeID of the child, colour, kinematic mode and collision elasticity.
/// </summary>
class PhysicsObject
{
protected:
	PhysicsObject(ShapeType shapeID, bool isKinematic = false, float elasticity = 1.0f) : m_shapeID(shapeID), m_isKinematic(isKinematic), m_elasticity(elasticity) {}

public:
	virtual void fixedUpdate(vec2 gravity, float timeStep) = 0;
	virtual void draw() = 0;
	virtual bool isInside(vec2 point) { return false; }

	// Getters
	int getShapeID() { return static_cast<int>(m_shapeID); }
	bool getIsKinematic() { return m_isKinematic; }
	float getElasticity() { return m_elasticity; }
	// Setters
	void setIsKinematic(bool value) { m_isKinematic = value; }

protected:
	ShapeType m_shapeID;
	vec4 m_colour;
	bool m_isKinematic;
	float m_elasticity;
};

