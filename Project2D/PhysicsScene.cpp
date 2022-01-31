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
}

void PhysicsScene::removeActor(PhysicsObject* actor)
{
}

