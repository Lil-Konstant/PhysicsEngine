#include "PhysicsScene.h"

PhysicsScene::PhysicsScene()
{
	setTimeStep(0.02f);
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
		checkForCollisions();
	}
}

void PhysicsScene::draw()
{
	for (auto pActor : m_actors)
	{
		pActor->draw();
	}
}

typedef bool(*fn)(PhysicsObject*, PhysicsObject*);
static fn collisionFunctionArray[] =
{
	PhysicsScene::plane2Plane, PhysicsScene::plane2Sphere,
	PhysicsScene::sphere2Plane, PhysicsScene::sphere2Sphere,
};

void PhysicsScene::checkForCollisions()
{
	// Check for collisions between all actors, assume they are spheres
	int actorCount = m_actors.size();
	for (int outer = 0; outer < actorCount - 1; outer++)
	{
		for (int inner = outer + 1; inner < actorCount; inner++)
		{
			PhysicsObject* object1 = m_actors[outer];
			PhysicsObject* object2 = m_actors[inner];
			int shapeId1 = object1->getShapeID();
			int shapeId2 = object2->getShapeID();

			int functionIdx = (shapeId1 * SHAPE_COUNT) + shapeId2;
			//float overlap = 0.0f;
			fn collisionFunctionPtr = collisionFunctionArray[functionIdx];
			if (collisionFunctionPtr)
			{
				collisionFunctionPtr(object1, object2);
				// correct position by overlap amount
				// object1->setPosition(object1->getPosition
			}
		}
	}
}

// Planes can't collide so simply return false
bool PhysicsScene::plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return false;
}

bool PhysicsScene::sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);

	if (sphere && plane)
	{
		float sphereToOriginProjection = dot(sphere->getPosition(), plane->getNormal());
		float result = sphereToOriginProjection - plane->getOriginDistance() - sphere->getRadius();

		// Used to check if the sphere is already moving out of the plane
		float speedOutOfPlane = dot(sphere->getVelocity(), plane->getNormal());

		// If result is negative (meaning collision) and sphere is moving into plane, move it out
		if (result <= 0 && speedOutOfPlane < 0)
		{
			vec2 contact = sphere->getPosition() + (-(plane->getNormal()) * sphere->getRadius());
			plane->resolveCollision(sphere, contact);
			return true;
		}
	}
}

// We want to use the same function for plane/sphere or sphere/plane, so just call the first with the order swapped
bool PhysicsScene::plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return sphere2Plane(obj2, obj1);
}

bool PhysicsScene::sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Sphere* sphere1 = dynamic_cast<Sphere*>(obj1);
	Sphere* sphere2 = dynamic_cast<Sphere*>(obj2);

	if (sphere1 && sphere2)
	{
		float distance = glm::distance(sphere1->getPosition(), sphere2->getPosition());

		if (distance <= (sphere1->getRadius() + sphere2->getRadius()))
		{
			vec2 collisionNormal = normalize(sphere2->getPosition() - sphere1->getPosition());
			vec2 contactPoint = sphere1->getPosition() + (collisionNormal * sphere1->getRadius());
			sphere1->resolveCollision(sphere2, contactPoint, collisionNormal);
		}
	}

	return false;
}
