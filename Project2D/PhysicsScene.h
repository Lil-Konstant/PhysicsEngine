#pragma once

// Include the OpenGL maths for vec2's and the std lib vectors
#include <vector>
#include "glm/glm.hpp"
#include "PhysicsObject.h"
#include "Sphere.h"
#include "Plane.h"
#include "AABB.h"
#include "OBB.h"

using namespace std;
using namespace glm;

class PhysicsScene
{
public:
	PhysicsScene();
	~PhysicsScene();

	void addActor(PhysicsObject* actor);
	void removeActor(PhysicsObject* actor);

	void update(float dt);
	void draw();

	void checkForCollisions();
	// Collision detection functions between all collision primitives
	static bool plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool AABB2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool plane2AABB(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool AABB2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool sphere2AABB(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool AABB2OBB(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool OBB2AABB(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool AABB2AABB(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool OBB2Plane(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool plane2OBB(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool OBB2Sphere(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool sphere2OBB(PhysicsObject* obj1, PhysicsObject* obj2);
	static bool OBB2OBB(PhysicsObject* obj1, PhysicsObject* obj2);

	PhysicsObject* objectUnderPoint(vec2 point);

	// Accessor functions for m_gravity
	void setGravity(const vec2 gravity) { m_gravity = gravity; };
	vec2 getGravity() const { return m_gravity; };

	// Accessor functions for m_timeStep
	void setTimeStep(const float timeStep) { m_timeStep = timeStep; };
	float getTimeStep() const { return m_timeStep; };

protected:
	vec2 m_gravity;
	float m_timeStep;
	vector<PhysicsObject*> m_actors;
};

