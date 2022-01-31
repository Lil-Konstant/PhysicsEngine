#pragma once

// Include the OpenGL maths for vec2's and the std lib vectors
#include "glm/vec2.hpp"
#include <vector>
#include "PhysicsObject.h"

using namespace std;
using namespace glm;

//// Forward declaration of PhysicsObject
//class PhysicsObject;

class PhysicsScene
{
public:
	PhysicsScene();
	~PhysicsScene();

	void addActor(PhysicsObject* actor);
	void removeActor(PhysicsObject* actor);

	void update(float dt);
	void draw();

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

