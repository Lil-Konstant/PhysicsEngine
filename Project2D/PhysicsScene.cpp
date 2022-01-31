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

