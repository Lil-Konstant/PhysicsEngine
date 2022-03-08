#pragma once

#include "Application.h"
#include "glm/ext.hpp"
#include "Gizmos.h"
#include "Renderer2D.h"
#include "PhysicsScene.h"
#include "Spring.h"

/// <summary>
/// PhysicsApp is an extension of the aie::Application class that implements the
/// specific logic for initialising the physics scene and populating it with rigid bodies.
/// The class also supplies utility functions for converting from screen to world coordinates,
/// as well as creating and attaching a player spring, which is a spring that connects between
/// a chosen rigid body in the scene and the player mouse.
/// </summary>
class PhysicsApp : public aie::Application {
public:

	PhysicsApp();
	virtual ~PhysicsApp();

	virtual bool startup();
	virtual void shutdown();
	virtual void update(float deltaTime);
	virtual void draw();

	// Utility conversion function
	vec2 screenToWorld(vec2 screenPos);

	// Used to attach rigid bodies in the scene to the player's mouse
	void attachPlayerSpring(RigidBody* other, vec2 contact, vec2 mousePos);

protected:

	aie::Renderer2D*	m_2dRenderer;
	aie::Font*			m_font;

	static const float extents;
	static const float aspectRatio;

	float m_timer;
	PhysicsScene* m_physicsScene;
	Spring* m_playerSpring;
};