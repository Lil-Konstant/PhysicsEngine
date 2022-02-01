#include "PhysicsScene.h"

PhysicsScene::PhysicsScene()
{
	setTimeStep(0.01f);
	setGravity(vec2(0, 0.0f));
}

PhysicsScene::~PhysicsScene()
{
	for (auto pActor : m_actors)
	{
		delete pActor;
	}
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

	// Check for collisions between all actors, assume they are spheres
	int actorCount = m_actors.size();
	for (int outer = 0; outer < actorCount - 1; outer++)
	{
		for (int inner = outer + 1; inner < actorCount; inner++)
		{
			PhysicsObject* object1 = m_actors[outer];
			PhysicsObject* object2 = m_actors[inner];

			// assume they are both spheres and check for collision between the two
			sphere2Sphere(object1, object2);
		}
	}
}

void PhysicsScene::draw()
{
	for (auto pActor : m_actors)
	{
		pActor->draw();
	}
}

bool PhysicsScene::sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Sphere* sphere1 = dynamic_cast<Sphere*>(obj1);
	Sphere* sphere2 = dynamic_cast<Sphere*>(obj2);

	if (sphere1 && sphere2)
	{
		float distance = glm::distance(sphere1->getPosition(), sphere2->getPosition());

		// If the spheres collide, just stop their movement for now
		if (distance <= (sphere1->getRadius() + sphere2->getRadius()))
		{
			sphere1->setVelocity({ 0, 0 });
			sphere2->setVelocity({ 0, 0 });

			//sphere1->applyForceToActor(sphere2, (sphere1->getMass() * sphere1->getVelocity()) + (sphere2->getMass() * sphere2->getVelocity()));
		}
	}

	return false;
}
