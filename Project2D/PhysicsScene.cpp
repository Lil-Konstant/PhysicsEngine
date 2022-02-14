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
	PhysicsScene::plane2Plane, PhysicsScene::plane2Sphere, PhysicsScene::plane2AABB, PhysicsScene::plane2OBB,
	PhysicsScene::sphere2Plane, PhysicsScene::sphere2Sphere, PhysicsScene::sphere2AABB, PhysicsScene::sphere2OBB,
	PhysicsScene::AABB2Plane, PhysicsScene::AABB2Sphere, PhysicsScene::AABB2AABB, PhysicsScene::AABB2OBB,
	PhysicsScene::OBB2Plane, PhysicsScene::OBB2Sphere, PhysicsScene::OBB2AABB, PhysicsScene::OBB2OBB
};

/// <summary>
/// Called every fixedTimestep by the PhysicsScene's Update()
/// </summary>
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
			fn collisionFunctionPtr = collisionFunctionArray[functionIdx];
			if (collisionFunctionPtr)
			{
				collisionFunctionPtr(object1, object2);
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
			sphere->resolveCollision(plane, contact, plane->getNormal());

			// Draw a line to the contact point
			aie::Gizmos::add2DCircle(contact, 2, 100, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(sphere->getPosition(), contact, { 1, 0, 0, 1 });

			return true;
		}
	}

	return false;
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

			// Draw a line to the contact point
			aie::Gizmos::add2DCircle(contactPoint, 2, 100, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(sphere1->getPosition(), contactPoint, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(sphere2->getPosition(), contactPoint, { 1, 0, 0, 1 });

			return true;
		}
	}

	return false;
}

bool PhysicsScene::AABB2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	AABB* aabb = dynamic_cast<AABB*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);

	if (aabb && plane)
	{
		int numContacts = 0;
		vec2 contact(0, 0);
		float contactV = 0;

		vec2 planeOrigin = plane->getNormal() * plane->getOriginDistance();

		// Check the position and velocity of each corner relative to the plane
		vector<vec2> corners = aabb->getCorners();
		for (auto corner : corners)
		{
			float distFromPlane = dot(corner - planeOrigin, plane->getNormal());

			// Find the component of the corner's velocity into the plan
			float velocityIntoPlane = dot(aabb->getVelocity(), plane->getNormal());
			// If the corner is below the plane and also moving into it
			if (distFromPlane < 0 && velocityIntoPlane <= 0)
			{
				numContacts++;
				contact += corner;
				contactV += velocityIntoPlane;
			}
		}

		if (numContacts > 0)
		{
			aabb->resolveCollision(plane, contact / (float)numContacts, plane->getNormal());

			// Draw a line to the contact point
			aie::Gizmos::add2DCircle(contact / (float)numContacts, 2, 100, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(aabb->getPosition(), contact, { 1, 0, 0, 1 });

			return true;
		}
	}

	return false;
}

bool PhysicsScene::plane2AABB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return AABB2Plane(obj2, obj1);
}

bool PhysicsScene::AABB2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	AABB* aabb = dynamic_cast<AABB*>(obj1);
	Sphere* sphere = dynamic_cast<Sphere*>(obj2);

	if (aabb && sphere)
	{
		// Find the min and max coordinates of the AABB
		vec2 aabbMin = aabb->getPosition() - aabb->getExtents();
		vec2 aabbMax = aabb->getPosition() + aabb->getExtents();

		vec2 possibleContact = glm::clamp(sphere->getPosition(), aabbMin, aabbMax);
		float closestDistance = glm::distance(possibleContact, sphere->getPosition());

		// If the distance to the closest point on the OBB is less than the sphere's radius (and the sphere is moving into the OBB), then resolve collision
		if (closestDistance < sphere->getRadius())
		{
			vec2 collisionNormal = vec2(0, 0);
			if (sphere->getPosition().x >= aabbMax.x)
			{
				collisionNormal += vec2(1, 0);
			}
			if (sphere->getPosition().x <= aabbMin.x)
			{
				collisionNormal += vec2(-1, 0);
			}
			if (sphere->getPosition().y >= aabbMax.y)
			{
				collisionNormal += vec2(0, 1);
			}
			if (sphere->getPosition().y <= aabbMin.y)
			{
				collisionNormal += vec2(0, -1);
			}

			aabb->resolveCollision(sphere, possibleContact, collisionNormal);

			// Draw a line to the contact point
			aie::Gizmos::add2DCircle(possibleContact, 2, 100, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(aabb->getPosition(), possibleContact, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(sphere->getPosition(), possibleContact, { 1, 0, 0, 1 });

			return true;
		}
	}

	return false;
}

bool PhysicsScene::sphere2AABB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return AABB2Sphere(obj2, obj1);
}

bool PhysicsScene::AABB2OBB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return false;
}

bool PhysicsScene::OBB2AABB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return AABB2OBB(obj2, obj1);
}

bool PhysicsScene::AABB2AABB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return false;
}

bool PhysicsScene::OBB2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	OBB* obb = dynamic_cast<OBB*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);

	if (obb && plane)
	{
		int numContacts = 0;
		vec2 contact(0, 0);
		float contactV = 0;

		vec2 planeOrigin = plane->getNormal() * plane->getOriginDistance();

		// Check the position and velocity of each corner relative to the plane
		vector<vec2> corners = obb->getCorners();
		for (auto corner : corners)
		{
			float distFromPlane = dot(corner - planeOrigin, plane->getNormal());

			// Total velocity of point in world space
			vec2 cornerDisplacement = corner - obb->getPosition();
			vec2 pointVelocity = obb->getVelocity() + (obb->getAngularVelocity() * vec2(-cornerDisplacement.y, cornerDisplacement.x));
			// Find the component of the corner's velocity into the plan
			float velocityIntoPlane = dot(pointVelocity, plane->getNormal());

			// If the corner is below the plane and also moving into it
			if (distFromPlane < 0 && velocityIntoPlane <= 0)
			{
				numContacts++;
				contact += corner;
				contactV += velocityIntoPlane;
			}
		}

		if (numContacts > 0)
		{
			obb->resolveCollision(plane, contact / (float)numContacts, plane->getNormal());

			// Draw a line to the contact point
			aie::Gizmos::add2DCircle(contact / (float)numContacts, 2, 100, {1, 0, 0, 1});
			aie::Gizmos::add2DLine(obb->getPosition(), contact, { 1, 0, 0, 1 });

			return true;
		}
	}

	return false;
}

bool PhysicsScene::plane2OBB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return OBB2Plane(obj2, obj1);
}

bool PhysicsScene::OBB2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	OBB* obb = dynamic_cast<OBB*>(obj1);
	Sphere* sphere = dynamic_cast<Sphere*>(obj2);

	if (obb && sphere)
	{
		// First find the sphere's coordinates relative to the OBBs local axes
		vec2 sphereDisplacement = sphere->getPosition() - obb->getPosition();
		vec2 localSphere = vec2( dot(sphereDisplacement, obb->getLocalX()), dot(sphereDisplacement, obb->getLocalY()));
		// Find the local coordiantes of the bottom left and top right corners of the OBB
		vec2 bottomLeftLocal = vec2(-obb->getExtents().x, -obb->getExtents().y);
		vec2 topRightLocal = vec2(obb->getExtents().x, obb->getExtents().y);

		vec2 possibleContactLocal = glm::clamp(localSphere, bottomLeftLocal, topRightLocal);
		float closestDistance =  glm::distance(possibleContactLocal, localSphere);

		// If the distance to the closest point on the OBB is less than the sphere's radius (and the sphere is moving into the OBB), then resolve collision
		if (closestDistance < sphere->getRadius())
		{
			// Convert the local contact point on the obb into world coordinates
			vec2 contact = obb->getPosition() + (possibleContactLocal.x * obb->getLocalX()) + (possibleContactLocal.y * obb->getLocalY());
			vec2 collisionNormal = glm::normalize(sphere->getPosition() - contact);
			obb->resolveCollision(sphere, contact, collisionNormal);

			// Draw a line to the contact point
			aie::Gizmos::add2DCircle(contact, 2, 100, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(obb->getPosition(), contact, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(sphere->getPosition(), contact, { 1, 0, 0, 1 });

			return true;
		}
	}

	return false;
}

bool PhysicsScene::sphere2OBB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return OBB2Sphere(obj2, obj1);
}

bool PhysicsScene::OBB2OBB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	static int callCount = 0;

	OBB* obb1 = dynamic_cast<OBB*>(obj1);
	OBB* obb2 = dynamic_cast<OBB*>(obj2);

	if (obb1 && obb2)
	{
		vec2 collisionNormal(0, 0);
		vec2 contact(0, 0);
		float pen = 0;
		int numContacts = 0;

		// Check for overlap from both boxes perspective to find the smallest penetration, if found in the second object, flip the collision normal so it is always the same for obb1
		obb1->checkOBBCorners(*obb2, contact, numContacts, pen, collisionNormal);
		if (obb2->checkOBBCorners(*obb1, contact, numContacts, pen, collisionNormal))
		{
			collisionNormal = -collisionNormal;
		}
		// If a penetration was found from both collision checks, resolve on obb1
		if (pen > 0)
		{
			obb1->resolveCollision(obb2, contact / (float)numContacts, collisionNormal);

			// Draw a line to the contact point
			aie::Gizmos::add2DCircle(contact / (float)numContacts, 2, 100, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(obb1->getPosition(), contact / (float)numContacts, { 1, 0, 0, 1 });
			aie::Gizmos::add2DLine(obb2->getPosition(), contact / (float)numContacts, { 1, 0, 0, 1 });

			return true;
		}
	}

	return false;
}
