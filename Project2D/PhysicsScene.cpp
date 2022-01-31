#include "PhysicsScene.h"

PhysicsScene::PhysicsScene()
{
	setTimeStep(0.01f);
	setGravity(vec2(0, 0));
}

PhysicsScene::~PhysicsScene()
{

}

void PhysicsScene::addActor(PhysicsObject* actor)
{
	m_actors.push_back(actor);
}

void PhysicsScene::removeActor(PhysicsObject* actor)
{
	remove(m_actors.begin(), m_actors.end(), actor);
}

void PhysicsScene::update(float dt)
{
	// Define a static accumulatedTime that is incremented by dt every update
	static float accumulatedTime = 0.0f;
	accumulatedTime += dt;

	// While we have accumulated more time than our fixed timestep, continue to run physics loops
	while (accumulatedTime >= m_timeStep)
	{
		for (auto pActor : m_actors)
		{
			pActor->fixedUpdate(m_gravity, dt);
		}

		accumulatedTime -= m_timeStep;
	}
}

void PhysicsScene::draw()
{
	for (auto pActor : m_actors)
	{
		pActor->draw();
	}
}