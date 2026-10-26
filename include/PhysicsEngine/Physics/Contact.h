#pragma once

// Contact.h -- Contact point and collision manifold representation.
//
// Conventions:
//   - bodyA, bodyB: The two intersecting bodies involved in the contact.
//   - normal: Unit vector pointing from bodyA toward bodyB.
//             In the impulse solver (Milestone 5), this convention ensures that
//             a separating impulse +J*normal accelerates bodyB away from bodyA,
//             and -J*normal accelerates bodyA away from bodyB.
//   - penetration: Non-negative penetration depth along normal (> 0 for collisions).
//                  Touching boundaries (penetration <= 0) do not generate contacts.
//   - point: Representative contact point in world-space coordinates on the
//            contact interface between the two bodies.

#include "PhysicsEngine/Math/Vec2.h"
#include "PhysicsEngine/Physics/Body.h"

namespace PhysicsEngine {

struct Contact {
    BodyHandle bodyA;
    BodyHandle bodyB;
    Vec2       normal      = {0.f, 0.f}; // Unit normal: bodyA -> bodyB
    float      penetration = 0.f;        // Penetration depth (> 0)
    Vec2       point       = {0.f, 0.f}; // Contact position in world coordinates
};

} // namespace PhysicsEngine
