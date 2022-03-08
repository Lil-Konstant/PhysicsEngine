#include "PhysicsScene.h"

/// <summary>
/// PhysicsScene() simply sets the fixed timestep of
/// the physics to be 0.01 (100 fps) and sets the gravity
/// to be 0, 0 as gravity is not used in the current build
/// of this simulation.
/// </summary>
PhysicsScene::PhysicsScene()
{
	setTimeStep(0.01f);
	setGravity(vec2(0, 0.0f));
}

/// <summary>
/// ~PhysicsScene() simply iterates through the scene's list
/// of actors and calls delete on all of them.
/// </summary>
PhysicsScene::~PhysicsScene()
{
	for (auto pActor : m_actors)
	{
		delete pActor;
	}
}

/// <summary>
/// addActor() takes an input of the PhysicsObject to add to the physics
/// scene, and pushes it to the back of the m_actors list.
/// </summary>
/// <param name="actor">The PhysicsObject to add.</param>
void PhysicsScene::addActor(PhysicsObject* actor)
{
	m_actors.push_back(actor);
}

/// <summary>
/// removeActor() takes an input of the PhysicsObject to remove from the
/// physics scene, and uses the std::remove() function to remove it from
/// the m_actors vector.
/// </summary>
/// <param name="actor">The PhysicsObject to remove.</param>
void PhysicsScene::removeActor(PhysicsObject* actor)
{
	remove(m_actors.begin(), m_actors.end(), actor);
}

/// <summary>
/// update() keeps track of the amount of time that has accumulated, and will
/// call fixedUpdate an all of the actors in the scene each time the accumulated
/// time has reached the amount of time defined by the fixed timeStep. Every
/// accumulated time step the function also calls checkForCollisions() which
/// checks collisions between all actors in the scene.
/// </summary>
/// <param name="dt">The amount of time past since last frame.</param>
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

/// <summary>
/// draw() simply iterates through all actors in the scene and calls
/// their individual draw() functions. This function is called by the
/// update() loop in the PhysicsApp.
/// </summary>
void PhysicsScene::draw()
{
	for (auto pActor : m_actors)
	{
		pActor->draw();
	}
}

// Indexed into during the checkForCollisions() function call, see below for explanation
typedef bool(*fn)(PhysicsObject*, PhysicsObject*);
static fn collisionFunctionArray[] =
{
	PhysicsScene::plane2Plane, PhysicsScene::plane2Sphere, PhysicsScene::plane2AABB, PhysicsScene::plane2OBB,
	PhysicsScene::sphere2Plane, PhysicsScene::sphere2Sphere, PhysicsScene::sphere2AABB, PhysicsScene::sphere2OBB,
	PhysicsScene::AABB2Plane, PhysicsScene::AABB2Sphere, PhysicsScene::AABB2AABB, PhysicsScene::AABB2OBB,
	PhysicsScene::OBB2Plane, PhysicsScene::OBB2Sphere, PhysicsScene::OBB2AABB, PhysicsScene::OBB2OBB
};

/// <summary>
/// Called every fixedTimestep by the PhysicsScene's Update(), the function iterates through all actors in the scene,
/// and for each actors it checks for collisions with each other actor. The function does this by using the enum ShapeID's
/// of the two objects being checked, and uses their values to index into the collisionFunctionArray to get a pointer to
/// the correct collision detection function for the two objects.
/// </summary>
void PhysicsScene::checkForCollisions()
{
	// For each actor, check against all other actors
	int actorCount = m_actors.size();
	for (int outer = 0; outer < actorCount - 1; outer++)
	{
		for (int inner = outer + 1; inner < actorCount; inner++)
		{
			PhysicsObject* object1 = m_actors[outer];
			PhysicsObject* object2 = m_actors[inner];
			int shapeId1 = object1->getShapeID();
			int shapeId2 = object2->getShapeID();

			// If either shape is a spring joint, skip collision detection
			if (shapeId1 < 0 || shapeId2 < 0)
			{
				continue;
			}

			// Index into the collisionFunctionArray using the 2D array equation
			int functionIdx = (shapeId1 * (int)ShapeType::SHAPE_COUNT) + shapeId2;
			fn collisionFunctionPtr = collisionFunctionArray[functionIdx];
			if (collisionFunctionPtr)
			{
				// Trigger the correct collision detection function for the two objects
				collisionFunctionPtr(object1, object2);
			}
		}
	}
}

/// <summary>
/// The collision detection function for 2 planes, planes can't collide so simply return false.
/// </summary>
/// <param name="obj1">The first plane.</param>
/// <param name="obj2">The second plane.</param>
/// <returns>True if colliding, false otherwise (always false in this case).</returns>
bool PhysicsScene::plane2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return false;
}

/// <summary>
/// The collision detection function for a sphere and a plane. The function uses the distance to
/// plane equation for points to determine whether the sphere is above or below the plane. If below
/// the plane and the sphere is actively moving into the plane, then the function calls resolveCollision().
/// </summary>
/// <param name="obj1">The sphere that is possibly colliding.</param>
/// <param name="obj2">The plane possibly being collided with.</param>
/// <returns>True if colliding, false otherwise.</returns>
bool PhysicsScene::sphere2Plane(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Sphere* sphere = dynamic_cast<Sphere*>(obj1);
	Plane* plane = dynamic_cast<Plane*>(obj2);

	if (sphere && plane)
	{
		// Project the sphere onto the planes normal to get its distance from the origin along the normal
		float sphereToOriginProjection = dot(sphere->getPosition(), plane->getNormal());
		// Subtract the planes origin displacement and the radius of the sphere
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

/// <summary>
/// For plane2Sphere, we want to reuse the sphere2Plane function, so simply call that function
/// with the parameters swapped.
/// </summary>
bool PhysicsScene::plane2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return sphere2Plane(obj2, obj1);
}

/// <summary>
/// sphere2Sphere simply checks to see if the distance between the two spheres is greater than
/// their combined radii. If not, and if the spheres are moving towards eachother, then the function
/// calls resolveCollision().
/// </summary>
/// <param name="obj1">The first sphere.</param>
/// <param name="obj2">The second sphere.</param>
/// <returns>True if colliding, false otherwise.</returns>
bool PhysicsScene::sphere2Sphere(PhysicsObject* obj1, PhysicsObject* obj2)
{
	Sphere* sphere1 = dynamic_cast<Sphere*>(obj1);
	Sphere* sphere2 = dynamic_cast<Sphere*>(obj2);

	if (sphere1 && sphere2)
	{
		float distance = glm::distance(sphere1->getPosition(), sphere2->getPosition());

		if (distance <= (sphere1->getRadius() + sphere2->getRadius()))
		{
			// The collision normal for spheres is just their normalised displacement
			vec2 collisionNormal = normalize(sphere2->getPosition() - sphere1->getPosition());
			// Move along the collision normal by sphere1's radius to get to the point of contact
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

/// <summary>
/// OBB2Plane checks for collisions between boxes and planes, and does so by performing
/// point to plane checks for each of the four corners of the box. If a point is both
/// below the plane and also moving into it, then it is added to the contact sum. The average
/// contact point is then used when resolving collisions.
/// </summary>
/// <param name="obj1">The OBB that is possibly colliding.</param>
/// <param name="obj2">The plane that is possibly being collided with.</param>
/// <returns></returns>
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
			// Find the component of the corner's velocity into the plane
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

/// <summary>
/// For plane2OBB, we want to reuse the OBB2Plane function, so simply call that function
/// with the parameters swapped.
/// </summary>
bool PhysicsScene::plane2OBB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return OBB2Plane(obj2, obj1);
}

/// <summary>
/// OBB2Sphere() is used to check for collisions between boxes and spheres. The function does this by
/// first converting the world space coordinates of the sphere into the local space coordinates of the
/// box. The function then uses standard AABB2Sphere logic, by clamping the sphere's position to the OBB's
/// min and max points, and checking if this clamped position is within range of the sphere's radius.
/// </summary>
/// <param name="obj1">The OBB that is possibly colliding.</param>
/// <param name="obj2">The Sphere that is possibly colliding.</param>
/// <returns></returns>
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

/// <summary>
/// For sphere2OBB, we want to reuse the OBB2Sphere function, so simply call that function
/// with the parameters swapped.
/// </summary>
bool PhysicsScene::sphere2OBB(PhysicsObject* obj1, PhysicsObject* obj2)
{
	return OBB2Sphere(obj2, obj1);
}

/// <summary>
/// OBB2OBB() is used to check for collisions between colliding boxes, and does so using a special case
/// implementation of the SAT algorithm. Most of the logic for collision checks here is actually contained
/// within the OBB method checkOBBCorners, so see there for a futher explanation of the process.
/// </summary>
/// <param name="obj1"></param>
/// <param name="obj2"></param>
/// <returns></returns>
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

/// <summary>
/// objectUnderPoint() takes an input of the worldspace point to check against,
/// and calls the isInside() function on all actors in the scene to check if any
/// of the objects contain the passed point.
/// </summary>
/// <param name="point">Worldspace point to check under.</param>
/// <returns>The rigidbody that is underneath the inputted point, nullptr if none.</returns>
RigidBody* PhysicsScene::objectUnderPoint(vec2 point)
{
	for (auto actor : m_actors)
	{
		if (actor->isInside(point))
		{
			return dynamic_cast<RigidBody*>(actor);
		}
	}

	return nullptr;
}

#pragma region AABB related functions not implemented in final submission
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
#pragma endregion