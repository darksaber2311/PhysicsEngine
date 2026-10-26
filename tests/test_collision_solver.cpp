// test_collision_solver.cpp -- Catch2 tests for impulse-based collision response
// and contact positional resolution (Milestone 5).
//
// Coverage:
//   1. Two circles approaching each other collide and bounce.
//   2. Two circles separating do not receive an attractive impulse.
//   3. Equal-mass elastic collision behaves as expected.
//   4. Inelastic collision uses the configured restitution.
//   5. Dynamic body colliding with static body does not move the static body.
//   6. Static-static contact does not cause division by zero or NaNs.
//   7. AABB-AABB collision response works.
//   8. Circle-AABB response works with both body orderings.
//   9. Penetrating bodies are separated by positional correction.
//  10. Positional correction does not change velocities.
//  11. Coincident circles remain finite and are resolved safely.
//  12. Circle inside AABB receives correction in correct escape direction.
//  13. Invalid or stale handles are handled safely.
//  14. Repeated fixed steps do not introduce NaNs or infinities.
//  15. SimulationController integration: substeps, pause, and bouncing.

#include "PhysicsEngine/Physics/CollisionSolver.h"
#include "PhysicsEngine/Physics/Collision.h"
#include "PhysicsEngine/Physics/Contact.h"
#include "PhysicsEngine/Physics/Engine.h"
#include "PhysicsEngine/Physics/SimulationController.h"
#include "PhysicsEngine/Physics/WorldConfig.h"
#include "PhysicsEngine/Physics/Body.h"
#include "PhysicsEngine/Math/Vec2.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <cmath>
#include <limits>
#include <vector>

using namespace PhysicsEngine;
using Catch::Approx;

namespace {

Engine makeTestEngine(Vec2 gravity = {0.f, 0.f}, float fixedDt = 1.f / 120.f)
{
    WorldConfig cfg;
    cfg.gravity = gravity;
    cfg.fixedDt = fixedDt;
    cfg.maxBodies = 64;
    return Engine{cfg};
}

BodyDesc makeCircle(Vec2 pos, float radius, float mass = 1.f, Vec2 vel = {0.f, 0.f}, float restitution = 0.5f)
{
    BodyDesc d;
    d.shapeType   = ShapeType::Circle;
    d.position    = pos;
    d.radius      = radius;
    d.mass        = mass;
    d.velocity    = vel;
    d.restitution = restitution;
    return d;
}

BodyDesc makeAABB(Vec2 pos, float halfW, float halfH, float mass = 1.f, Vec2 vel = {0.f, 0.f}, float restitution = 0.5f)
{
    BodyDesc d;
    d.shapeType   = ShapeType::AABB;
    d.position    = pos;
    d.radius      = halfW;
    d.halfHeight  = halfH;
    d.mass        = mass;
    d.velocity    = vel;
    d.restitution = restitution;
    return d;
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────
// 1. Two circles approaching each other collide and bounce
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: approaching circles collide and bounce", "[solver][impulse]")
{
    Engine engine = makeTestEngine();
    // Circle A: mass 1 kg, pos (-0.8, 0), moving right at +2 m/s, restitution 0.5
    BodyHandle hA = engine.addBody(makeCircle({-0.8f, 0.f}, 1.f, 1.f, {2.f, 0.f}, 0.5f));
    // Circle B: mass 1 kg, pos (0.8, 0), moving left at -2 m/s, restitution 0.5
    BodyHandle hB = engine.addBody(makeCircle({0.8f, 0.f}, 1.f, 1.f, {-2.f, 0.f}, 0.5f));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);

    resolveVelocityImpulses(engine, contacts);

    // v_rel = (-2 - 2) = -4.
    // j = -(1 + 0.5) * (-4) / (1 + 1) = 3.0.
    // velA = 2 - 3 * 1 = -1.0.
    // velB = -2 + 3 * 1 = +1.0.
    REQUIRE(engine.getVelocity(hA).x == Approx(-1.f));
    REQUIRE(engine.getVelocity(hA).y == Approx(0.f).margin(1e-5f));
    REQUIRE(engine.getVelocity(hB).x == Approx(1.f));
    REQUIRE(engine.getVelocity(hB).y == Approx(0.f).margin(1e-5f));
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Two circles separating do not receive an attractive impulse
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: separating circles do not receive attractive impulse", "[solver][impulse]")
{
    Engine engine = makeTestEngine();
    // Two circles currently overlapping (dist = 1.6 < 2.0), but moving apart:
    BodyHandle hA = engine.addBody(makeCircle({-0.8f, 0.f}, 1.f, 1.f, {-2.f, 0.f}, 1.f));
    BodyHandle hB = engine.addBody(makeCircle({0.8f, 0.f}, 1.f, 1.f, {2.f, 0.f}, 1.f));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);

    resolveVelocityImpulses(engine, contacts);

    // Velocities must remain unchanged because v_rel . normal >= 0
    REQUIRE(engine.getVelocity(hA).x == Approx(-2.f));
    REQUIRE(engine.getVelocity(hB).x == Approx(2.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. Equal-mass elastic collision behaves as expected
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: equal-mass elastic collision transfers momentum completely", "[solver][elastic]")
{
    Engine engine = makeTestEngine();
    // Circle A moving right at 3 m/s, Circle B stationary at rest. Both mass 1 kg, e = 1.0.
    BodyHandle hA = engine.addBody(makeCircle({-0.5f, 0.f}, 1.f, 1.f, {3.f, 0.f}, 1.0f));
    BodyHandle hB = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 1.f, {0.f, 0.f}, 1.0f));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);

    resolveVelocityImpulses(engine, contacts);

    // Elastic equal mass collision: A stops (vel = 0), B moves at 3 m/s
    REQUIRE(engine.getVelocity(hA).x == Approx(0.f).margin(1e-5f));
    REQUIRE(engine.getVelocity(hB).x == Approx(3.f));
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Inelastic collision uses the configured restitution
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: inelastic collision uses minimum restitution", "[solver][restitution]")
{
    // Subcase A: e = 0.0 (completely inelastic)
    {
        Engine engine = makeTestEngine();
        BodyHandle hA = engine.addBody(makeCircle({-0.5f, 0.f}, 1.f, 1.f, {2.f, 0.f}, 0.0f));
        BodyHandle hB = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 1.f, {-2.f, 0.f}, 0.0f));

        std::vector<Contact> contacts;
        detectCollisionsBruteForce(engine, contacts);
        REQUIRE(contacts.size() == 1);

        resolveVelocityImpulses(engine, contacts);

        // Relative velocity after = 0 -> both bodies stop
        REQUIRE(engine.getVelocity(hA).x == Approx(0.f).margin(1e-5f));
        REQUIRE(engine.getVelocity(hB).x == Approx(0.f).margin(1e-5f));
    }

    // Subcase B: eA = 0.8, eB = 0.2 -> combined e = min(0.8, 0.2) = 0.2
    {
        Engine engine = makeTestEngine();
        BodyHandle hA = engine.addBody(makeCircle({-0.5f, 0.f}, 1.f, 1.f, {2.f, 0.f}, 0.8f));
        BodyHandle hB = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 1.f, {-2.f, 0.f}, 0.2f));

        std::vector<Contact> contacts;
        detectCollisionsBruteForce(engine, contacts);
        REQUIRE(contacts.size() == 1);

        resolveVelocityImpulses(engine, contacts);

        // j = -(1 + 0.2) * (-4) / 2 = 2.4
        // velA = 2 - 2.4 = -0.4
        // velB = -2 + 2.4 = +0.4
        REQUIRE(engine.getVelocity(hA).x == Approx(-0.4f));
        REQUIRE(engine.getVelocity(hB).x == Approx(0.4f));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. Dynamic body colliding with static body does not move static body
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: collision with static body does not move static body", "[solver][static]")
{
    Engine engine = makeTestEngine();
    // Static circle (mass = 0) at origin
    BodyHandle hStatic  = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 0.f, {0.f, 0.f}, 1.0f));
    // Dynamic circle (mass = 2 kg) moving right toward static circle
    BodyHandle hDynamic = engine.addBody(makeCircle({-1.5f, 0.f}, 1.f, 2.f, {4.f, 0.f}, 1.0f));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);

    solveCollisions(engine, contacts);

    // Static body remains at (0, 0) with zero velocity
    REQUIRE(engine.getVelocity(hStatic).x == 0.f);
    REQUIRE(engine.getVelocity(hStatic).y == 0.f);
    REQUIRE(engine.getPosition(hStatic).x == 0.f);
    REQUIRE(engine.getPosition(hStatic).y == 0.f);

    // Dynamic body rebounds with -4 m/s
    REQUIRE(engine.getVelocity(hDynamic).x == Approx(-4.f));
    // Dynamic body position is pushed back by positional correction (x < -1.5)
    REQUIRE(engine.getPosition(hDynamic).x < -1.5f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. Static-static contact does not cause division by zero or NaNs
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: static-static contact does not cause division by zero or NaNs", "[solver][safety]")
{
    Engine engine = makeTestEngine();
    BodyHandle h1 = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 0.f));
    BodyHandle h2 = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 0.f));

    Contact contact;
    contact.bodyA       = h1;
    contact.bodyB       = h2;
    contact.normal      = {1.f, 0.f};
    contact.penetration = 1.5f;
    contact.point       = {0.25f, 0.f};

    std::vector<Contact> contacts{contact};
    solveCollisions(engine, contacts);

    // Neither body moved, no NaNs
    REQUIRE(engine.getVelocity(h1).x == 0.f);
    REQUIRE(engine.getVelocity(h2).x == 0.f);
    REQUIRE(engine.getPosition(h1).x == 0.f);
    REQUIRE(engine.getPosition(h2).x == 0.5f);
    REQUIRE(std::isfinite(engine.getPosition(h1).x));
    REQUIRE(std::isfinite(engine.getPosition(h2).x));
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. AABB-AABB collision response works
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: AABB-AABB collision response resolves velocities and positions", "[solver][aabb]")
{
    Engine engine = makeTestEngine();
    // Two 2x2 boxes (half extents 1, 1), colliding along X
    BodyHandle hA = engine.addBody(makeAABB({-1.5f, 0.f}, 1.f, 1.f, 1.f, {3.f, 0.f}, 1.0f));
    BodyHandle hB = engine.addBody(makeAABB({0.f, 0.f}, 1.f, 1.f, 1.f, {0.f, 0.f}, 1.0f));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);

    solveCollisions(engine, contacts);

    // Box A stops, Box B moves with 3 m/s
    REQUIRE(engine.getVelocity(hA).x == Approx(0.f).margin(1e-5f));
    REQUIRE(engine.getVelocity(hB).x == Approx(3.f));
    // Overlap was reduced
    const float distAfter = engine.getPosition(hB).x - engine.getPosition(hA).x;
    REQUIRE(distAfter > 1.5f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. Circle-AABB response works with both body orderings
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: Circle-AABB response is consistent regardless of shape order", "[solver][circle_aabb]")
{
    // Order 1: bodyA is Circle, bodyB is AABB
    {
        Engine engine = makeTestEngine();
        BodyHandle hCircle = engine.addBody(makeCircle({-1.5f, 0.f}, 1.f, 1.f, {2.f, 0.f}, 1.0f));
        BodyHandle hAABB   = engine.addBody(makeAABB({0.f, 0.f}, 1.f, 1.f, 1.f, {0.f, 0.f}, 1.0f));

        std::vector<Contact> contacts;
        detectCollisionsBruteForce(engine, contacts);
        REQUIRE(contacts.size() == 1);

        solveCollisions(engine, contacts);

        REQUIRE(engine.getVelocity(hCircle).x == Approx(0.f).margin(1e-5f));
        REQUIRE(engine.getVelocity(hAABB).x == Approx(2.f));
    }

    // Order 2: bodyA is AABB, bodyB is Circle (with Circle moving toward AABB)
    {
        Engine engine = makeTestEngine();
        // AABB at origin at rest
        BodyHandle hAABB   = engine.addBody(makeAABB({0.f, 0.f}, 1.f, 1.f, 1.f, {0.f, 0.f}, 1.0f));
        // Circle at x = 1.5 moving left at -2 m/s
        BodyHandle hCircle = engine.addBody(makeCircle({1.5f, 0.f}, 1.f, 1.f, {-2.f, 0.f}, 1.0f));

        std::vector<Contact> contacts;
        detectCollisionsBruteForce(engine, contacts);
        REQUIRE(contacts.size() == 1);

        solveCollisions(engine, contacts);

        REQUIRE(engine.getVelocity(hCircle).x == Approx(0.f).margin(1e-5f));
        REQUIRE(engine.getVelocity(hAABB).x == Approx(-2.f));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 9. Penetrating bodies are separated by positional correction
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: penetrating bodies are separated by positional correction", "[solver][position]")
{
    Engine engine = makeTestEngine();
    // Two circles of radius 1 at (-0.8, 0) and (0.8, 0).
    // Distance = 1.6, penetration = 2.0 - 1.6 = 0.4.
    BodyHandle hA = engine.addBody(makeCircle({-0.8f, 0.f}, 1.f, 1.f));
    BodyHandle hB = engine.addBody(makeCircle({0.8f, 0.f}, 1.f, 1.f));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].penetration == Approx(0.4f));

    SolverConfig cfg;
    cfg.slop = 0.01f;
    cfg.percent = 0.5f;

    resolvePositionalCorrections(engine, contacts, cfg);

    // Initial separation = 1.6
    // depth = 0.4 - 0.01 = 0.39
    // rawMag = (0.39 / 2) * 0.5 = 0.0975
    // posA.x = -0.8 - 0.0975 = -0.8975
    // posB.x = 0.8 + 0.0975 = 0.8975
    // Distance after = 1.795 > 1.6
    const float distAfter = engine.getPosition(hB).x - engine.getPosition(hA).x;
    REQUIRE(distAfter == Approx(1.795f).margin(1e-4f));
}

// ─────────────────────────────────────────────────────────────────────────────
// 10. Positional correction does not change velocities
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: positional correction preserves velocities exactly", "[solver][position]")
{
    Engine engine = makeTestEngine();
    const Vec2 velA{3.5f, -1.2f};
    const Vec2 velB{-4.8f, 2.1f};

    BodyHandle hA = engine.addBody(makeCircle({-0.5f, 0.f}, 1.f, 1.f, velA));
    BodyHandle hB = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 1.f, velB));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);

    resolvePositionalCorrections(engine, contacts);

    // Positions changed
    REQUIRE(engine.getPosition(hA).x < -0.5f);
    REQUIRE(engine.getPosition(hB).x > 0.5f);

    // Velocities must be identical
    REQUIRE(engine.getVelocity(hA).x == velA.x);
    REQUIRE(engine.getVelocity(hA).y == velA.y);
    REQUIRE(engine.getVelocity(hB).x == velB.x);
    REQUIRE(engine.getVelocity(hB).y == velB.y);
}

// ─────────────────────────────────────────────────────────────────────────────
// 11. Coincident circles remain finite and are resolved safely
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: coincident circles are resolved safely along fallback normal", "[solver][coincident]")
{
    Engine engine = makeTestEngine();
    BodyHandle hA = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 1.f, {0.f, 0.f}));
    BodyHandle hB = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 1.f, {0.f, 0.f}));

    std::vector<Contact> contacts;
    detectCollisionsBruteForce(engine, contacts);
    REQUIRE(contacts.size() == 1);
    REQUIRE(contacts[0].normal.x == Approx(0.f));
    REQUIRE(contacts[0].normal.y == Approx(1.f));

    solveCollisions(engine, contacts);

    // All state must remain finite
    REQUIRE(std::isfinite(engine.getPosition(hA).x));
    REQUIRE(std::isfinite(engine.getPosition(hA).y));
    REQUIRE(std::isfinite(engine.getPosition(hB).x));
    REQUIRE(std::isfinite(engine.getPosition(hB).y));
    REQUIRE(std::isfinite(engine.getVelocity(hA).x));
    REQUIRE(std::isfinite(engine.getVelocity(hB).x));

    // Separation along Y axis (bodyA downward, bodyB upward)
    REQUIRE(engine.getPosition(hA).y < 0.f);
    REQUIRE(engine.getPosition(hB).y > 0.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 12. Circle inside AABB receives correction in correct escape direction
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: circle inside AABB receives correction toward nearest face", "[solver][circle_aabb]")
{
    // Subcase A: Circle near top face of AABB
    {
        Engine engine = makeTestEngine();
        // Static AABB at origin, half extents (3, 2). Top face at y = 2.
        (void)engine.addBody(makeAABB({0.f, 0.f}, 3.f, 2.f, 0.f));
        // Dynamic circle at (0, 1.5) radius 1 (inside, nearest to top face)
        BodyHandle hCircle = engine.addBody(makeCircle({0.f, 1.5f}, 1.f, 1.f));

        std::vector<Contact> contacts;
        detectCollisionsBruteForce(engine, contacts);
        REQUIRE(contacts.size() == 1);

        resolvePositionalCorrections(engine, contacts);

        // Circle must be pushed +Y toward and through the top face
        REQUIRE(engine.getPosition(hCircle).y > 1.5f);
        REQUIRE(engine.getPosition(hCircle).x == Approx(0.f).margin(1e-5f));
    }

    // Subcase B: Circle near right face of AABB
    {
        Engine engine = makeTestEngine();
        // Static AABB at origin, half extents (3, 2). Right face at x = 3.
        (void)engine.addBody(makeAABB({0.f, 0.f}, 3.f, 2.f, 0.f));
        // Dynamic circle at (2.5, 0) radius 1 (inside, nearest to right face)
        BodyHandle hCircle = engine.addBody(makeCircle({2.5f, 0.f}, 1.f, 1.f));

        std::vector<Contact> contacts;
        detectCollisionsBruteForce(engine, contacts);
        REQUIRE(contacts.size() == 1);

        resolvePositionalCorrections(engine, contacts);

        // Circle must be pushed +X toward and through the right face
        REQUIRE(engine.getPosition(hCircle).x > 2.5f);
        REQUIRE(engine.getPosition(hCircle).y == Approx(0.f).margin(1e-5f));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 13. Invalid or stale handles are handled safely
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: invalid or stale handles are rejected safely", "[solver][safety]")
{
    Engine engine = makeTestEngine();
    BodyHandle h1 = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 1.f, {1.f, 0.f}));
    BodyHandle h2 = engine.addBody(makeCircle({0.5f, 0.f}, 1.f, 1.f, {-1.f, 0.f}));

    // Remove h2 to make it stale
    engine.removeBody(h2);

    Contact contact;
    contact.bodyA       = h1;
    contact.bodyB       = h2; // stale
    contact.normal      = {1.f, 0.f};
    contact.penetration = 1.5f;

    std::vector<Contact> contacts{contact};

    // Should not crash or modify h1
    solveCollisions(engine, contacts);
    REQUIRE(engine.getVelocity(h1).x == 1.f);

    // Null handle
    contact.bodyB = BodyHandle::null();
    solveCollisions(engine, contacts);
    REQUIRE(engine.getVelocity(h1).x == 1.f);
}

// ─────────────────────────────────────────────────────────────────────────────
// 14. Repeated fixed steps do not introduce NaNs or infinities
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: repeated fixed steps under gravity remain finite and stable", "[solver][stability]")
{
    // Gravity pointing downward
    Engine engine = makeTestEngine({0.f, -9.81f}, 1.f / 120.f);

    // Static floor box at y = -2, half extents (10, 1) -> top surface at y = -1
    (void)engine.addBody(makeAABB({0.f, -2.f}, 10.f, 1.f, 0.f, {0.f, 0.f}, 0.5f));

    // Dynamic circles falling onto the floor
    BodyHandle b1 = engine.addBody(makeCircle({-0.5f, 2.f}, 0.5f, 1.f, {0.f, 0.f}, 0.6f));
    BodyHandle b2 = engine.addBody(makeCircle({0.5f, 3.f}, 0.5f, 1.f, {0.f, 0.f}, 0.4f));
    BodyHandle b3 = engine.addBody(makeAABB({0.f, 4.5f}, 0.5f, 0.5f, 1.f, {0.f, 0.f}, 0.3f));

    SimulationController controller;

    // Run 240 fixed steps (2 seconds)
    for (int i = 0; i < 240; ++i) {
        controller.step(engine);
    }

    // Verify all bodies have finite positions and velocities
    for (BodyHandle h : {b1, b2, b3}) {
        Vec2 pos = engine.getPosition(h);
        Vec2 vel = engine.getVelocity(h);
        REQUIRE(std::isfinite(pos.x));
        REQUIRE(std::isfinite(pos.y));
        REQUIRE(std::isfinite(vel.x));
        REQUIRE(std::isfinite(vel.y));

        // The floor is at y = -1. None of the bodies should have fallen through to -inf
        REQUIRE(pos.y > -2.f);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 15. SimulationController integration: substeps, pause, and bouncing
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("CollisionSolver: SimulationController integrates collision response into advance loop", "[controller][solver]")
{
    const float dt = 1.f / 120.f;
    Engine engine = makeTestEngine({0.f, 0.f}, dt);

    // Static wall at x = 2
    (void)engine.addBody(makeAABB({2.5f, 0.f}, 0.5f, 5.f, 0.f, {0.f, 0.f}, 1.0f));

    // Dynamic circle moving toward the wall at 6 m/s
    // Initial pos = 0. Radius = 1. Wall left face = 2.0.
    BodyHandle h = engine.addBody(makeCircle({0.f, 0.f}, 1.f, 1.f, {6.f, 0.f}, 1.0f));

    SimulationController controller;

    // Advance 30 frames of duration dt
    uint32_t steps = 0;
    for (int i = 0; i < 30; ++i) {
        steps += controller.advance(engine, dt);
    }
    REQUIRE(steps == 30);

    // Circle should have hit wall and bounced backward (vel.x < 0)
    REQUIRE(engine.getVelocity(h).x < 0.f);
    REQUIRE(engine.getPosition(h).x < 2.0f);

    // Pause prevents further motion
    controller.pause();
    const Vec2 posPaused = engine.getPosition(h);
    (void)controller.advance(engine, 0.1f);
    REQUIRE(engine.getPosition(h).x == posPaused.x);
}
