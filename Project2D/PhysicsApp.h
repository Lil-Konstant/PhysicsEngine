#pragma once

#include "Application.h"
#include "glm/ext.hpp"
#include "Gizmos.h"
#include "Renderer2D.h"
#include "PhysicsScene.h"
#include "Spring.h"


class PhysicsApp : public aie::Application {
public:

	PhysicsApp();
	virtual ~PhysicsApp();

	virtual bool startup();
	virtual void shutdown();

	virtual void update(float deltaTime);
	virtual void draw();

	vec2 screenToWorld(vec2 screenPos);

	void attachPlayerSpring(RigidBody* other, vec2 contact, vec2 mousePos);

	void createBridge(vec2 startingPos, int segments, float spacing, float springConstant);

protected:

	aie::Renderer2D*	m_2dRenderer;
	aie::Texture*		m_texture;
	aie::Texture*		m_shipTexture;
	aie::Font*			m_font;

	static const float extents;
	static const float aspectRatio;

	float m_timer;
	PhysicsScene* m_physicsScene;
	Spring* m_playerSpring;
};