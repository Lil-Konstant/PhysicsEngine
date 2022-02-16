#pragma once
#include "RigidBody.h"
class Spring :
    public PhysicsObject
{
public:
    Spring(RigidBody* body1, RigidBody* body2, vec4 colour, float springCoefficient, float restLength = 0.0f, float damping = 0.1f, vec2 contact1 = vec2(0, 0), vec2 contact2 = vec2(0, 0));
    ~Spring() {}

    void fixedUpdate(vec2 gravity, float timeStep) override;
    void draw() override;

    // Converts the contact points of each body into world coordinates and returns the position (or just returns m_contact if already in world coords)
    vec2 getContact1() { return m_body1 ? m_body1->toWorld(m_contact1) : m_contact1; }
    vec2 getContact2() { return m_body2 ? m_body2->toWorld(m_contact2) : m_contact2; }

    void setBody1(RigidBody* rig) { m_body1 = rig; }
    void setBody2(RigidBody* rig) { m_body2 = rig; }

    void setContact1(vec2 contact) { m_contact1 = contact; }
    void setContact2(vec2 contact) { m_contact2 = contact; }

    bool isActive() { return m_isActive; }
    void setActive(bool value) { m_isActive = value; }

protected:
    // The two bodies this spring is attached between
    RigidBody* m_body1;
    RigidBody* m_body2;

    // Joint contacts for each side of the spring, local to their bodies axes system if they have a body to attach to
    vec2 m_contact1;
    vec2 m_contact2;

    // Spring variables
    float m_damping;
    float m_restLength;
    float m_springCoefficient;

    bool m_isActive;
    vec4 m_colour;
};

