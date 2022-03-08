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

/// <summary>
/// startup() is called once at the start of the application, and instantiates a new renderer
/// and physics scene. The function then populates the physics scene with two OBB shapes, and one
/// sphere, as well as four planes that act as borders of the scene.
/// </summary>
/// <returns></returns>
bool PhysicsApp::startup() 
{
	m_playerSpring = nullptr;
	constexpr float pi = glm::pi<float>();

	// increase the 2D line count to maximize the number of objects we can draw
	aie::Gizmos::create(255U, 255U, 65535U, 65535U);

	m_2dRenderer = new aie::Renderer2D();

	m_font = new aie::Font("./font/consolas.ttf", 32);
	
	m_timer = 0;

	m_physicsScene = new PhysicsScene();

	// Sphere creation
	m_physicsScene->addActor(new Sphere({ 0, 0 }, 0, { 5, 10 }, 1, 5, 25, 1, { 1, 0.5f, 1, 1 }));
	 
	// OBB creation
	m_physicsScene->addActor(new OBB({ -50, 0 }, 5, 40, 0, { -25, 10 }, 0.5f, 1, { 0.5f, 0.5f, 1, 1 }));
	m_physicsScene->addActor(new OBB({ 50, 0 }, 10, 30, pi/4 + pi, { -12, 5}, 0.0f, 1, { 0.5f, 0.15f, 0.5f, 1 }));

	// build the walls of the screen
	m_physicsScene->addActor(new Plane({ 0, 1 }, -51, {0.5f, 0.5f, 1, 0.5}));
	m_physicsScene->addActor(new Plane({ 0, -1 }, -51, {0.5f, 0.5f, 1, 0.5}));
	m_physicsScene->addActor(new Plane({ 1, 0 }, -95, {0.5f, 0.5f, 1, 0.5}));
	m_physicsScene->addActor(new Plane({ -1, 0 }, -95, {0.5f, 0.5f, 1, 0.5}));

	return true;
}

/// <summary>
/// shutdown() simply deletes any allocated memory used in the app.
/// </summary>
void PhysicsApp::shutdown() {
	
	delete m_font;
	delete m_2dRenderer;
	delete m_physicsScene;
}

/// <summary>
/// update() is called by the aie::Application loop, and first checks for
/// user arrow key input to move the camera port around. The function then
/// checks for left mouse input to attempt to attach the player spring to
/// a rigid body in the scene. If the player spring is already active then
/// the function simply updates the second contact point with the current
/// mouse position so the spring always connects between the mouse and the
/// rigid body.
/// </summary>
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

	// If the player is left clicking
	if (input->wasMouseButtonPressed(0))
	{
		// Get the mouse position in world coordinates and draw it as a circle on the screen
		int xScreen, yScreen;
		input->getMouseXY(&xScreen, &yScreen);
		vec2 worldPos = screenToWorld(vec2(xScreen, yScreen));
		aie::Gizmos::add2DCircle(worldPos, 1, 32, { 1, 0, 0, 1 });
		
		// Find the rig underneath the mouse
		RigidBody* rig = m_physicsScene->objectUnderPoint(worldPos);
		
		// If the player spring is yet to be initialised, create a new spring with one rig attachment, attached to the player mouse
		if (!m_playerSpring && rig)
		{
			m_playerSpring = new Spring(rig, nullptr, { 1,0,0,1 }, 5.0f, 0.0f, 0.5f, rig->toLocal(worldPos), worldPos);
			m_physicsScene->addActor(m_playerSpring);
		}
		// Otherwise if the player spring already exists, reassign it to this rig
		else if (rig)
		{
			attachPlayerSpring(rig, rig->toLocal(worldPos), worldPos);
		}
	}
	// If the player has released left mouse this update, deactivate the player spring
	if (input->wasMouseButtonReleased(0) && m_playerSpring && m_playerSpring->isActive())
	{
		m_playerSpring->setActive(false);
	}

	
}

/// <summary>
/// draw() simply displays the fps, escape to quit and info text on the screen, and then
/// calls aie::Gizmos::draw2D, which draws all shapes that have been added to the most
/// recent draw call.
/// </summary>
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
	m_2dRenderer->drawText(m_font, "Click and drag on shapes to pull them!", 50, 50);

	static float aspectRatio = 16 / 9.f;
	aie::Gizmos::draw2D(glm::ortho<float>(-extents, extents, -extents / aspectRatio, extents / aspectRatio, -1.0f, 1.0f));

	// done drawing sprites
	m_2dRenderer->end();
}

/// <summary>
/// screenToWorld is a utility function that takes a vec2 screen position as input,
/// and converts it into the orthographic world coordinates of the game world, and
/// returns this conversion.
/// </summary>
/// <param name="screenPos">The screen coordinates to convert.</param>
/// <returns>A converted vec2 in world coordinates.</returns>
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

/// <summary>
/// attachPlayerSpring simply sets the player spring to active and attaches it 
/// to the inputted rigidbody, between the inputted contact point and mouse position
/// </summary>
/// <param name="other">The rigid body to attach the player spring to.</param>
/// <param name="contact">The point on the rigid body to attach to.</param>
/// <param name="mousePos">The position of the mouse to attach to.</param>
void PhysicsApp::attachPlayerSpring(RigidBody* other, vec2 contact, vec2 mousePos)
{
	m_playerSpring->setActive(true);
	m_playerSpring->setBody1(other);
	m_playerSpring->setBody2(nullptr);
	m_playerSpring->setContact1(contact);
	m_playerSpring->setContact2(mousePos);
}
