#include "PhysicsApp.h"
#include "Texture.h"
#include "Font.h"
#include "Input.h"
#include "Sphere.h"
#include "Plane.h"
#include "AABB.h"
#include "OBB.h"
#include "Spring.h"

const float PhysicsApp::extents = 100;
const float PhysicsApp::aspectRatio = 16.0f / 9.0f;

PhysicsApp::PhysicsApp() {

}

PhysicsApp::~PhysicsApp() {

}

bool PhysicsApp::startup() 
{
	m_playerSpring = nullptr;
	constexpr float pi = glm::pi<float>();

	// increase the 2D line count to maximize the number of objects we can draw
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	m_2dRenderer = new aie::Renderer2D();

	m_texture = new aie::Texture("./textures/numbered_grid.tga");
	m_shipTexture = new aie::Texture("./textures/ship.png");
	m_font = new aie::Font("./font/consolas.ttf", 32);
	
	m_timer = 0;

	m_physicsScene = new PhysicsScene();

	//createBridge({-40, 10}, 5, 30, 50.0f);

	// Sphere creation
	//m_physicsScene->addActor(new Sphere({ 50, -4 }, 0, { -10, 0 }, 2, 0.5f, 5, 1, { 0.5f, 1, 1, 1 }));
	//Sphere* sphere1 = new Sphere({ 0, 50 }, 0, { 0, 0 }, 1, 1, 25, 1, { 1, 0.5f, 1, 1 });
	//m_physicsScene->addActor(sphere1);
	//Sphere* sphere2 = new Sphere({ -20, 0 }, 0, { 0, 0 }, 1, 1, 10, 1, { 1, 1, 0.5f, 1 });
	//sphere2->setIsKinematic(true);
	//Spring* spring = new Spring(sphere1, sphere2, { 1, 0, 0, 1 }, 10.0f, 25.0f, 0.5f, vec2(0, 0), vec2(sphere2->getRadius() / 2, 0));
	//Spring* spring2 = new Spring(sphere1, box1, { 1, 0, 0, 1 }, 10.0f, 25.0f, 0.5f, vec2(0, 0), vec2( 0, box1->getExtents().y));

	//sphere->setIsKinematic(true);
	/*m_physicsScene->addActor(sphere1);
	m_physicsScene->addActor(sphere2);
	m_physicsScene->addActor(box1);
	m_physicsScene->addActor(spring);
	m_physicsScene->addActor(spring2);*/
	//m_physicsScene->addActor(new Sphere({ 0, 0 }, 0, { 15, 15 }, 0, 25, 25, 0.8f, { 0.5f, 0.5f, 0.5f, 1 }));
	 

	m_physicsScene->addActor(new OBB({ 0, 0 }, 5, 40, 0, { 5, 0 }, 0.0f, 1, { 0.5f, 0.5f, 1, 1 }));
	m_physicsScene->addActor(new OBB({ 20, 0 }, 10, 30, pi/4 + pi, { -12, 5}, 0.0f, 1, { 0.5f, 0.15f, 0.5f, 1 }));
	//m_physicsScene->addActor(new OBB({ 50, 50 }, 2, 70, pi/4 + pi, { 20, -40}, 4.0f, 1, { 0.15f, 0.15f, 0.8f, 1 }));

	//m_physicsScene->addActor(new AABB({ 0, 0 }, 40, 40, { 100, 40 }, 1, { 1, 0, 0, 1 }));

	// build the walls of the screen
	m_physicsScene->addActor(new Plane({ 0, 1 }, -51, {0.5f, 0.5f, 1, 0.5}));
	m_physicsScene->addActor(new Plane({ 0, -1 }, -51, {0.5f, 0.5f, 1, 0.5}));
	m_physicsScene->addActor(new Plane({ 1, 0 }, -95, {0.5f, 0.5f, 1, 0.5}));
	m_physicsScene->addActor(new Plane({ -1, 0 }, -95, {0.5f, 0.5f, 1, 0.5}));

	return true;
}

void PhysicsApp::shutdown() {
	
	delete m_font;
	delete m_texture;
	delete m_shipTexture;
	delete m_2dRenderer;
}

void PhysicsApp::update(float deltaTime) {

	m_timer += deltaTime;

	// input example
	aie::Input* input = aie::Input::getInstance();

	aie::Gizmos::clear();

	m_physicsScene->update(deltaTime);
	m_physicsScene->draw();

	// Update the camera position using the arrow keys
	float camPosX;
	float camPosY;
	m_2dRenderer->getCameraPos(camPosX, camPosY);

	// Camera movements
	if (input->isKeyDown(aie::INPUT_KEY_UP))
		camPosY += 500.0f * deltaTime;

	if (input->isKeyDown(aie::INPUT_KEY_DOWN))
		camPosY -= 500.0f * deltaTime;

	if (input->isKeyDown(aie::INPUT_KEY_LEFT))
		camPosX -= 500.0f * deltaTime;

	if (input->isKeyDown(aie::INPUT_KEY_RIGHT))
		camPosX += 500.0f * deltaTime;

	m_2dRenderer->setCameraPos(camPosX, camPosY);

	// Exit the application
	if (input->isKeyDown(aie::INPUT_KEY_ESCAPE))
		quit();



	// Update the second contact point of the player spring with the current mouse pos this frame
	if (m_playerSpring && m_playerSpring->isActive())
	{
		int xScreen, yScreen;
		input->getMouseXY(&xScreen, &yScreen);
		vec2 worldPos = screenToWorld(vec2(xScreen, yScreen));
		m_playerSpring->setContact2(worldPos);
	}

	// If the player is left 
	if (input->wasMouseButtonPressed(0))
	{
		// Get the mouse position in world coordinates and draw it as a circle on the screen
		int xScreen, yScreen;
		input->getMouseXY(&xScreen, &yScreen);
		vec2 worldPos = screenToWorld(vec2(xScreen, yScreen));
		aie::Gizmos::add2DCircle(worldPos, 1, 32, { 1, 0, 0, 1 });
		
		// Find the rig underneath the mouse
		RigidBody* rig = m_physicsScene->objectUnderPoint(worldPos);
		
		if (!m_playerSpring && rig)
		{
			m_playerSpring = new Spring(rig, nullptr, { 1,0,0,1 }, 5.0f, 0.0f, 0.5f, rig->toLocal(worldPos), worldPos);
			m_physicsScene->addActor(m_playerSpring);
		}
		else if (rig)
		{
			attachPlayerSpring(rig, rig->toLocal(worldPos), worldPos);
		}
	}
	if (input->wasMouseButtonReleased(0) && m_playerSpring && m_playerSpring->isActive())
	{
		m_playerSpring->setActive(false);
	}

	
}

void PhysicsApp::draw() {

	// wipe the screen to the background colour
	clearScreen();

	// begin drawing sprites
	m_2dRenderer->begin();
	
	// output some text, uses the last used colour
	char fps[32];
	sprintf_s(fps, 32, "FPS: %i", getFPS());
	m_2dRenderer->drawText(m_font, fps, 0, 720 - 32);
	m_2dRenderer->drawText(m_font, "Press ESC to quit!", 0, 720 - 64);

	static float aspectRatio = 16 / 9.f;
	aie::Gizmos::draw2D(glm::ortho<float>(-extents, extents, -extents / aspectRatio, extents / aspectRatio, -1.0f, 1.0f));


	// done drawing sprites
	m_2dRenderer->end();
}

vec2 PhysicsApp::screenToWorld(vec2 screenPos)
{
	vec2 worldPos = screenPos;

	// move the centre of the screen to (0, 0)
	worldPos.x -= getWindowWidth() / 2;
	worldPos.y -= getWindowHeight() / 2;

	// scale according to our extents
	worldPos.x *= 2.0f * extents / getWindowWidth();
	worldPos.y *= 2.0f * extents / (aspectRatio * getWindowHeight());

	return worldPos;
}

void PhysicsApp::attachPlayerSpring(RigidBody* other, vec2 contact, vec2 mousePos)
{
	m_playerSpring->setActive(true);
	m_playerSpring->setBody1(other);
	m_playerSpring->setBody2(nullptr);
	m_playerSpring->setContact1(contact);
	m_playerSpring->setContact2(mousePos);
}

void PhysicsApp::createBridge(vec2 startingPos, int segments, float spacing, float springConstant)
{
	vec2 boxPos = startingPos;
	OBB* box;
	OBB* previousBox;
	Spring* spring;

	float boxWidth = 20.0f;
	float boxHeight = 3.0f;
	for (int i = 0; i < segments; i++)
	{
		if (i == 0)
		{
			box = new OBB(boxPos, boxWidth, boxHeight, 0, { 0, 0 }, 0, 1.0f, { 1, 1, 1, 1 });
			box->setIsKinematic(true);
		}
		else if (i == segments - 1)
		{
			box = new OBB(boxPos, boxWidth, boxHeight, 0, { 0, 0 }, 0, 1.0f, { 1, 1, 1, 1 });
			box->setIsKinematic(true);
			spring = new Spring(previousBox, box, { 165, 42, 42, 255 }, springConstant, spacing - boxWidth, 5.0f, vec2(previousBox->getExtents().x, 0), vec2(-box->getExtents().x, 0));
			m_physicsScene->addActor(spring);
		}
		else
		{
			box = new OBB(boxPos, boxWidth, boxHeight, 0, { 0, 0 }, 0, 1.0f, { 0.2f, 0.3f, 1, 1 });
			spring = new Spring(previousBox, box, { 165, 42, 42, 255 }, springConstant, spacing - boxWidth, 5.0f, vec2(previousBox->getExtents().x, 0), vec2(-box->getExtents().x, 0));
			m_physicsScene->addActor(spring);
		}
		m_physicsScene->addActor(box);

		previousBox = box;
		boxPos.x += spacing;
	}
}
